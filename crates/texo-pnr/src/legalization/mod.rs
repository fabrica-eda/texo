//! Capacity projection from analytical unit targets to physical assignments.

mod auction;
mod repair;

use std::cmp::Reverse;
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;
use std::time::Instant;

use texo_model::{
    BelId, CellPinId, NetId, PinDirection, Point, ResourceKind, UnifiedGraph, WireId,
};

use self::auction::{AuctionError, AuctionStats, BestObjects, PricedObject, assign_implicitly};
use self::repair::{RepairStats, repair_relaxed};
use super::{
    PlacementChoices, PlacementConstraints, PlacementResourceUsage, PlacementUnit, PnrError,
    SpatialChoiceIndex, assignment_resources_are_legal, candidate_pin_wire, install_assignment,
};

/// Classification and implicit-edge work for one analytical projection.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct ProjectionStats {
    fixed_units: usize,
    overlapping_shared_families: usize,
    overlapping_shared_units: usize,
    disjoint_shared_families: usize,
    disjoint_shared_units: usize,
    singleton_units: usize,
    relaxed_unmatched_units: usize,
    auction: AuctionStats,
}

struct ProjectionState {
    stats: ProjectionStats,
    preferred: Vec<Option<usize>>,
    fixed: Vec<bool>,
    placed: Vec<Option<BelId>>,
    occupied: BTreeSet<BelId>,
    occupied_mask: Vec<bool>,
    base_usage: PlacementResourceUsage,
}

struct ProjectionCohorts {
    overlapping_units: Vec<usize>,
    disjoint_families: Vec<((u8, usize), Vec<usize>)>,
    singletons: Vec<usize>,
}

struct ProjectionProgress {
    enabled: bool,
    started: Instant,
    phase_started: Instant,
}

impl ProjectionProgress {
    fn new() -> Self {
        let now = Instant::now();
        Self {
            enabled: std::env::var_os("TEXO_PNR_METRICS").is_some(),
            started: now,
            phase_started: now,
        }
    }

    fn completed(&mut self, phase: &str, stats: ProjectionStats) {
        let now = Instant::now();
        if self.enabled {
            eprintln!(
                "texo-pnr analytical projection progress: phase={phase} phase-ms={} total-ms={} fixed={} shared-overlap-families={} shared-overlap-units={} shared-disjoint-families={} shared-disjoint-units={} singleton={} relaxed-unmatched={} bids={} candidates={}",
                now.duration_since(self.phase_started).as_millis(),
                now.duration_since(self.started).as_millis(),
                stats.fixed_units,
                stats.overlapping_shared_families,
                stats.overlapping_shared_units,
                stats.disjoint_shared_families,
                stats.disjoint_shared_units,
                stats.singleton_units,
                stats.relaxed_unmatched_units,
                stats.auction.bids,
                stats.auction.candidates_examined,
            );
        }
        self.phase_started = now;
    }
}

impl ProjectionState {
    fn new(unit_count: usize, cell_count: usize, bel_count: usize) -> Self {
        Self {
            stats: ProjectionStats::default(),
            preferred: vec![None; unit_count],
            fixed: vec![false; unit_count],
            placed: vec![None; cell_count],
            occupied: BTreeSet::new(),
            occupied_mask: vec![false; bel_count],
            base_usage: PlacementResourceUsage::default(),
        }
    }
}

#[derive(Clone, Copy)]
enum AuctionObjects {
    SharedRows,
    SingletonBels,
}

/// Legal candidate rings discovered for one bidder during one auction.
///
/// Occupancy and shared/pin-resource usage are immutable for the lifetime of
/// an `auction_units` call; only object prices change between bids. Keeping
/// each completely visited ring therefore avoids repeating the comparatively
/// expensive legality checks after a bidder is displaced. `ring_ends[r]` is
/// the exclusive end of the legal choices discovered through Manhattan ring
/// `r`. Empty rings are represented too, which proves that a later bid may
/// resume at the first genuinely unseen radius without a search cap.
#[derive(Default)]
struct BidderRingCache {
    objects: Vec<usize>,
    ring_ends: Vec<usize>,
}

/// Allocation-free scratch for repeated exact resource-legality queries.
///
/// The generic placement helper intentionally owns its shared-resource scratch
/// because most call sites perform only a few queries. Analytical projection
/// performs tens of millions, so retaining both vectors for the complete
/// projection avoids one heap allocation per candidate without weakening any
/// pin-capacity or target-defined shared-resource rule.
#[derive(Default)]
struct AssignmentLegalityWorkspace {
    pin_resources: Vec<(WireId, NetId)>,
    shared_resources: Vec<((usize, u64), u64)>,
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
enum PinMappingIdentity {
    CandidateSpecific(CellPinId),
    Named(String, PinDirection),
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct PhysicalResourceShape {
    pin_columns: Vec<(ResourceKind, Vec<PinMappingIdentity>)>,
    shared: Vec<(usize, usize)>,
}

/// Unit-dependent half of a resource-legality query, prepared once per
/// projection instead of rediscovered for every spatial candidate.
struct PreparedUnitLegality {
    shape: PhysicalResourceShape,
    pin_representatives: Vec<(usize, CellPinId)>,
    pins: Vec<(usize, NetId)>,
    shared_values: Vec<u64>,
}

impl PreparedUnitLegality {
    fn new(
        graph: &UnifiedGraph<'_>,
        constraints: &PlacementConstraints,
        unit: &PlacementUnit,
    ) -> Self {
        let mut pin_columns = Vec::new();
        let mut pin_representatives = Vec::new();
        let mut pins = Vec::new();
        for (column, &cell) in unit.cells.iter().enumerate() {
            let logical_cell = &graph.design().cells()[cell.0];
            let mut identities = Vec::new();
            for &pin in graph.design().cells()[cell.0].pins() {
                let logical_pin = &graph.design().pins()[pin.0];
                let candidate_specific = constraints
                    .pin_bindings
                    .keys()
                    .any(|(bound, _)| *bound == pin);
                identities.push(if candidate_specific {
                    PinMappingIdentity::CandidateSpecific(pin)
                } else {
                    PinMappingIdentity::Named(
                        constraints
                            .pin_name_bindings
                            .get(&pin)
                            .cloned()
                            .unwrap_or_else(|| logical_pin.name.clone()),
                        logical_pin.direction,
                    )
                });
                let slot = pin_representatives.len();
                pin_representatives.push((column, pin));
                if let Some(net) = logical_pin.net() {
                    pins.push((slot, net));
                }
            }
            pin_columns.push((logical_cell.kind, identities));
        }
        let mut shared = Vec::new();
        let mut shared_values = Vec::new();
        for (rule, constraint) in constraints.shared_resources.iter().enumerate() {
            for (column, &cell) in unit.cells.iter().enumerate() {
                if let Some(&value) = constraint.cell_values.get(&cell) {
                    shared.push((rule, column));
                    shared_values.push(value);
                }
            }
        }
        Self {
            shape: PhysicalResourceShape {
                pin_columns,
                shared,
            },
            pin_representatives,
            pins,
            shared_values,
        }
    }
}

struct PreparedPhysicalResources {
    pin_count: usize,
    shared_count: usize,
    pin_wires: Vec<WireId>,
    shared_resources: Vec<Option<u64>>,
}

impl PreparedPhysicalResources {
    fn new(
        graph: &UnifiedGraph<'_>,
        constraints: &PlacementConstraints,
        unit: &PlacementUnit,
        prepared: &PreparedUnitLegality,
    ) -> Result<Self, AuctionError> {
        let pin_count = prepared.pin_representatives.len();
        let shared_count = prepared.shape.shared.len();
        let pin_capacity = unit
            .choices
            .len()
            .checked_mul(pin_count)
            .ok_or(AuctionError::ArithmeticOverflow)?;
        let shared_capacity = unit
            .choices
            .len()
            .checked_mul(shared_count)
            .ok_or(AuctionError::ArithmeticOverflow)?;
        let mut pin_wires = Vec::with_capacity(pin_capacity);
        let mut shared_resources = Vec::with_capacity(shared_capacity);
        for choice in 0..unit.choices.len() {
            let assignment = unit.choices.assignment(choice);
            for &(column, pin) in &prepared.pin_representatives {
                pin_wires.push(
                    candidate_pin_wire(graph, constraints, pin, assignment[column])
                        .expect("placement candidate has every bound physical pin"),
                );
            }
            for &(rule, column) in &prepared.shape.shared {
                shared_resources.push(
                    constraints.shared_resources[rule]
                        .bel_resources
                        .get(&assignment[column])
                        .copied(),
                );
            }
        }
        debug_assert_eq!(pin_wires.len(), pin_capacity);
        debug_assert_eq!(shared_resources.len(), shared_capacity);
        Ok(Self {
            pin_count,
            shared_count,
            pin_wires,
            shared_resources,
        })
    }

    fn pin_wire(&self, choice: usize, slot: usize) -> WireId {
        self.pin_wires[choice * self.pin_count + slot]
    }

    fn shared_resource(&self, choice: usize, slot: usize) -> Option<u64> {
        self.shared_resources[choice * self.shared_count + slot]
    }
}

fn prepare_physical_resources(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    units: &[PlacementUnit],
    bidders: &[usize],
    prepared: &[PreparedUnitLegality],
) -> Result<(Vec<PreparedPhysicalResources>, Vec<usize>), AuctionError> {
    let mut known = BTreeMap::<((u8, usize), PhysicalResourceShape), usize>::new();
    let mut tables = Vec::new();
    let mut bidder_tables = Vec::with_capacity(bidders.len());
    for (bidder, (&unit_index, prepared)) in bidders.iter().zip(prepared).enumerate() {
        let unit = &units[unit_index];
        let key = (unit.choices.cache_key(), prepared.shape.clone());
        let table = if let Some(&table) = known.get(&key) {
            table
        } else {
            let table = tables.len();
            tables.push(PreparedPhysicalResources::new(
                graph,
                constraints,
                unit,
                prepared,
            )?);
            known.insert(key, table);
            table
        };
        debug_assert_eq!(bidder_tables.len(), bidder);
        bidder_tables.push(table);
    }
    Ok((tables, bidder_tables))
}

/// Projects every movable unit onto actual assignment capacity.
///
/// Shared tables are auction-safe only when every BEL occurs in at most one
/// row. Such a family is solved as a separate composite-slot relaxation before
/// the global singleton-BEL relaxation. This ordering is deterministic and
/// capacity preserving, but is deliberately not claimed to be a global
/// optimum across families. Full BEL/shared/pin legality is restored by the
/// transactional repair after all relaxed cohorts have been proposed.
pub(super) fn project(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    units: &[PlacementUnit],
    spatial_indexes: &BTreeMap<(u8, usize), Arc<SpatialChoiceIndex>>,
    targets: &[Point],
) -> Result<Vec<Option<BelId>>, PnrError> {
    debug_assert_eq!(units.len(), targets.len());
    let device = graph.device();
    debug_assert!(
        targets
            .iter()
            .all(|target| target.x < device.width() && target.y < device.height())
    );
    let mut progress = ProjectionProgress::new();
    let mut state = ProjectionState::new(
        units.len(),
        graph.design().cells().len(),
        graph.device().bels().len(),
    );
    install_fixed_units(graph, constraints, units, &mut state)?;
    progress.completed("fixed", state.stats);
    let mut cohorts = classify_units(units, &mut state.stats);
    progress.completed("classify", state.stats);
    seed_overlapping_units(
        graph,
        constraints,
        units,
        spatial_indexes,
        targets,
        &mut cohorts.overlapping_units,
        &mut state,
    )?;
    progress.completed("overlap", state.stats);
    auction_disjoint_families(
        graph,
        constraints,
        units,
        spatial_indexes,
        targets,
        &mut cohorts.disjoint_families,
        &mut state,
    )?;
    progress.completed("disjoint-auction", state.stats);
    auction_singletons(
        graph,
        constraints,
        units,
        spatial_indexes,
        targets,
        &cohorts.singletons,
        &mut state,
    )?;
    progress.completed("singleton-auction", state.stats);
    progress.completed("before-repair", state.stats);
    let (placed, repair) = repair_relaxed(
        graph,
        constraints,
        units,
        spatial_indexes,
        targets,
        &state.preferred,
        &state.fixed,
    )?;
    progress.completed("repair", state.stats);
    report_projection_stats(state.stats, repair);
    Ok(placed)
}

fn install_fixed_units(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    units: &[PlacementUnit],
    state: &mut ProjectionState,
) -> Result<(), PnrError> {
    for (index, unit) in units.iter().enumerate() {
        if unit.choices.len() != 1 {
            continue;
        }
        let assignment = unit.choices.assignment(0);
        if !assignment_is_unoccupied(assignment, &state.occupied_mask)
            || !assignment_resources_are_legal(
                graph,
                constraints,
                &unit.cells,
                assignment,
                &state.base_usage,
            )
        {
            return Err(projection_error(
                "fixed analytical placement assignments conflict",
            ));
        }
        state.preferred[index] = Some(0);
        state.fixed[index] = true;
        state.stats.fixed_units += 1;
        install_assignment(
            graph,
            constraints,
            unit,
            assignment,
            &mut state.placed,
            &mut state.occupied,
            &mut state.base_usage,
        );
        mark_assignment_occupied(assignment, &mut state.occupied_mask);
    }
    Ok(())
}

fn classify_units(units: &[PlacementUnit], stats: &mut ProjectionStats) -> ProjectionCohorts {
    let mut shared_families = BTreeMap::<(u8, usize), Vec<usize>>::new();
    let mut singletons = Vec::new();
    for (index, unit) in units.iter().enumerate() {
        if unit.choices.len() == 1 {
            continue;
        }
        match &unit.choices {
            PlacementChoices::Shared(_) => {
                shared_families
                    .entry(unit.choices.cache_key())
                    .or_default()
                    .push(index);
            }
            PlacementChoices::SingleCell(_) => singletons.push(index),
        }
    }

    let mut overlapping_units = Vec::new();
    let mut disjoint_families = Vec::new();
    for (key, mut family) in shared_families {
        family.sort_unstable_by_key(|&index| units[index].cells[0]);
        if rows_are_pairwise_bel_disjoint(&units[family[0]].choices) {
            stats.disjoint_shared_families += 1;
            stats.disjoint_shared_units += family.len();
            disjoint_families.push((key, family));
        } else {
            stats.overlapping_shared_families += 1;
            stats.overlapping_shared_units += family.len();
            overlapping_units.extend(family);
        }
    }
    stats.singleton_units = singletons.len();
    singletons.sort_unstable_by_key(|&index| units[index].cells[0]);
    ProjectionCohorts {
        overlapping_units,
        disjoint_families,
        singletons,
    }
}

fn seed_overlapping_units(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    units: &[PlacementUnit],
    spatial_indexes: &BTreeMap<(u8, usize), Arc<SpatialChoiceIndex>>,
    targets: &[Point],
    overlapping: &mut [usize],
    state: &mut ProjectionState,
) -> Result<(), PnrError> {
    overlapping.sort_unstable_by_key(|&index| {
        let unit = &units[index];
        (unit.choices.len(), Reverse(unit.cells.len()), unit.cells[0])
    });
    for &index in overlapping.iter() {
        let unit = &units[index];
        let spatial = &spatial_indexes[&unit.choices.cache_key()];
        let Some(choice) = nearest_direct_choice(
            graph,
            constraints,
            unit,
            spatial,
            targets[index],
            &state.occupied_mask,
            &state.base_usage,
        )?
        else {
            continue;
        };
        let assignment = unit.choices.assignment(choice);
        state.preferred[index] = Some(choice);
        install_assignment(
            graph,
            constraints,
            unit,
            assignment,
            &mut state.placed,
            &mut state.occupied,
            &mut state.base_usage,
        );
        mark_assignment_occupied(assignment, &mut state.occupied_mask);
    }
    Ok(())
}

fn nearest_direct_choice(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    unit: &PlacementUnit,
    spatial: &SpatialChoiceIndex,
    target: Point,
    occupied: &[bool],
    usage: &PlacementResourceUsage,
) -> Result<Option<usize>, PnrError> {
    let device = graph.device();
    let max_radius = device
        .width()
        .checked_sub(1)
        .and_then(|width| {
            device
                .height()
                .checked_sub(1)
                .and_then(|height| width.checked_add(height))
        })
        .ok_or_else(|| projection_error("overlapping-unit ring radius overflow"))?;
    let prepared = PreparedUnitLegality::new(graph, constraints, unit);
    let mut legality_workspace = AssignmentLegalityWorkspace::default();
    for radius in 0..=max_radius {
        let mut nearest = None::<(Point, usize)>;
        for dy in 0..=radius {
            let dx = radius - dy;
            for y in super::ring_coordinates(target.y, dy, device.height()) {
                for x in super::ring_coordinates(target.x, dx, device.width()) {
                    let point = Point::new(x, y);
                    let point_index = y
                        .checked_mul(device.width())
                        .and_then(|row| row.checked_add(x))
                        .and_then(|index| usize::try_from(index).ok())
                        .ok_or_else(|| projection_error("spatial choice index overflow"))?;
                    for &choice in &spatial.by_point[point_index] {
                        let assignment = unit.choices.assignment(choice);
                        if assignment_is_unoccupied(assignment, occupied)
                            && assignment_resources_are_legal_prepared_dynamic(
                                graph,
                                constraints,
                                assignment,
                                usage,
                                &prepared,
                                &mut legality_workspace,
                            )
                            && nearest.is_none_or(|current| (point, choice) < current)
                        {
                            nearest = Some((point, choice));
                        }
                    }
                }
            }
        }
        if let Some((_, choice)) = nearest {
            return Ok(Some(choice));
        }
    }
    Ok(None)
}

fn auction_disjoint_families(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    units: &[PlacementUnit],
    spatial_indexes: &BTreeMap<(u8, usize), Arc<SpatialChoiceIndex>>,
    targets: &[Point],
    families: &mut [((u8, usize), Vec<usize>)],
    state: &mut ProjectionState,
) -> Result<(), PnrError> {
    families.sort_by(|left, right| {
        let left_unit = &units[left.1[0]];
        let right_unit = &units[right.1[0]];
        let left_scarcity = (left_unit.choices.len() as u128) * (right.1.len() as u128);
        let right_scarcity = (right_unit.choices.len() as u128) * (left.1.len() as u128);
        left_scarcity
            .cmp(&right_scarcity)
            .then_with(|| right_unit.cells.len().cmp(&left_unit.cells.len()))
            .then_with(|| left_unit.cells[0].cmp(&right_unit.cells[0]))
    });
    let verbose_metrics_enabled = std::env::var_os("TEXO_PNR_VERBOSE_METRICS").is_some();
    for (ordinal, (key, family)) in families.iter().enumerate() {
        let ordinal = ordinal + 1;
        debug_assert!(
            family
                .iter()
                .all(|&index| units[index].choices.cache_key() == *key)
        );
        let unit = &units[family[0]];
        let first_row_bel = unit
            .choices
            .assignment(0)
            .first()
            .map_or(usize::MAX, |bel| bel.0);
        let last_row_bel = unit
            .choices
            .assignment(unit.choices.len() - 1)
            .first()
            .map_or(usize::MAX, |bel| bel.0);
        let started = Instant::now();
        if verbose_metrics_enabled {
            eprintln!(
                "TEXO_PNR_VERBOSE_METRICS disjoint-family start ordinal={ordinal}/{} units={} rows={} cells_per_unit={} first_cell={} first_row_bel={} last_row_bel={} cache_kind={}",
                families.len(),
                family.len(),
                unit.choices.len(),
                unit.cells.len(),
                unit.cells[0].0,
                first_row_bel,
                last_row_bel,
                key.0,
            );
        }
        let result = auction_units(
            graph,
            constraints,
            units,
            spatial_indexes,
            targets,
            family,
            AuctionObjects::SharedRows,
            &state.occupied_mask,
            &state.base_usage,
        )?;
        if verbose_metrics_enabled {
            eprintln!(
                "TEXO_PNR_VERBOSE_METRICS disjoint-family complete ordinal={ordinal}/{} elapsed_ms={} units={} rows={} bids={} candidates_examined={} unmatched={}",
                families.len(),
                started.elapsed().as_millis(),
                family.len(),
                unit.choices.len(),
                result.stats.bids,
                result.stats.candidates_examined,
                result.stats.unmatched,
            );
        }
        record_auction_result(units, family, &result, state, false)?;
    }
    Ok(())
}

fn auction_singletons(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    units: &[PlacementUnit],
    spatial_indexes: &BTreeMap<(u8, usize), Arc<SpatialChoiceIndex>>,
    targets: &[Point],
    singletons: &[usize],
    state: &mut ProjectionState,
) -> Result<(), PnrError> {
    let result = auction_units(
        graph,
        constraints,
        units,
        spatial_indexes,
        targets,
        singletons,
        AuctionObjects::SingletonBels,
        &state.occupied_mask,
        &state.base_usage,
    )?;
    record_auction_result(units, singletons, &result, state, true)
}

fn record_auction_result(
    units: &[PlacementUnit],
    bidders: &[usize],
    result: &auction::AuctionResult,
    state: &mut ProjectionState,
    objects_are_bels: bool,
) -> Result<(), PnrError> {
    merge_auction_stats(&mut state.stats, result.stats)?;
    state.stats.relaxed_unmatched_units = state
        .stats
        .relaxed_unmatched_units
        .checked_add(result.stats.unmatched)
        .ok_or_else(|| projection_error("relaxed unmatched count overflow"))?;
    for (&unit_index, &object) in bidders.iter().zip(&result.assignment) {
        let Some(object) = object else {
            continue;
        };
        let unit = &units[unit_index];
        let choice = if objects_are_bels {
            let bel = BelId(object);
            match &unit.choices {
                PlacementChoices::SingleCell(candidates) => candidates
                    .binary_search(&bel)
                    .map_err(|_| projection_error("auction returned a noncandidate BEL"))?,
                PlacementChoices::Shared(_) => {
                    return Err(projection_error("BEL auction received a composite unit"));
                }
            }
        } else {
            object
        };
        let assignment = unit.choices.assignment(choice);
        if !assignment_is_unoccupied(assignment, &state.occupied_mask) {
            return Err(projection_error(
                "capacity auction returned an occupied assignment",
            ));
        }
        state.preferred[unit_index] = Some(choice);
        state.occupied.extend(assignment.iter().copied());
        mark_assignment_occupied(assignment, &mut state.occupied_mask);
    }
    Ok(())
}

fn report_projection_stats(stats: ProjectionStats, repair: RepairStats) {
    if std::env::var_os("TEXO_PNR_METRICS").is_some() {
        eprintln!(
            "texo-pnr analytical projection: fixed={} shared-overlap-families={} shared-overlap-units={} shared-disjoint-families={} shared-disjoint-units={} singleton={} relaxed-unmatched={} bids={} auction-candidates={} repair-initial-pending={} repair-states={} repair-failed-states={} repair-max-unplaced={} repair-max-branches={} repair-evictions={} repair-choices={}",
            stats.fixed_units,
            stats.overlapping_shared_families,
            stats.overlapping_shared_units,
            stats.disjoint_shared_families,
            stats.disjoint_shared_units,
            stats.singleton_units,
            stats.relaxed_unmatched_units,
            stats.auction.bids,
            stats.auction.candidates_examined,
            repair.initial_pending,
            repair.states_visited,
            repair.failed_states,
            repair.max_unplaced,
            repair.max_blocker_branches,
            repair.units_evicted,
            repair.choices_examined,
        );
    }
}

fn rows_are_pairwise_bel_disjoint(choices: &PlacementChoices) -> bool {
    let PlacementChoices::Shared(_) = choices else {
        return false;
    };
    let mut seen = BTreeSet::new();
    (0..choices.len()).all(|choice| {
        choices
            .assignment(choice)
            .iter()
            .all(|&bel| seen.insert(bel))
    })
}

fn merge_auction_stats(stats: &mut ProjectionStats, auction: AuctionStats) -> Result<(), PnrError> {
    stats.auction.bids = stats
        .auction
        .bids
        .checked_add(auction.bids)
        .ok_or_else(|| projection_error("auction bid count overflow"))?;
    stats.auction.candidates_examined = stats
        .auction
        .candidates_examined
        .checked_add(auction.candidates_examined)
        .ok_or_else(|| projection_error("auction candidate count overflow"))?;
    stats.auction.unmatched = stats
        .auction
        .unmatched
        .checked_add(auction.unmatched)
        .ok_or_else(|| projection_error("auction unmatched count overflow"))?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn auction_units(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    units: &[PlacementUnit],
    spatial_indexes: &BTreeMap<(u8, usize), Arc<SpatialChoiceIndex>>,
    targets: &[Point],
    bidders: &[usize],
    objects: AuctionObjects,
    occupied: &[bool],
    base_usage: &PlacementResourceUsage,
) -> Result<auction::AuctionResult, PnrError> {
    if bidders.is_empty() {
        return assign_implicitly(0, 0, 0, |_, _, _| Ok(BestObjects::default()))
            .map_err(auction_error);
    }
    let device = graph.device();
    let object_count = match objects {
        AuctionObjects::SharedRows => units[bidders[0]].choices.len(),
        AuctionObjects::SingletonBels => device.bels().len(),
    };
    let max_base_cost = u64::from(device.width() - 1) + u64::from(device.height() - 1);
    let mut legality_workspace = AssignmentLegalityWorkspace::default();
    let mut bidder_caches = (0..bidders.len())
        .map(|_| BidderRingCache::default())
        .collect::<Vec<_>>();
    let bidder_legality = bidders
        .iter()
        .map(|&unit| PreparedUnitLegality::new(graph, constraints, &units[unit]))
        .collect::<Vec<_>>();
    let (physical_resources, bidder_physical_resources) =
        prepare_physical_resources(graph, constraints, units, bidders, &bidder_legality)
            .map_err(auction_error)?;
    assign_implicitly(
        bidders.len(),
        object_count,
        max_base_cost,
        |bidder, prices, scale| {
            let unit_index = bidders[bidder];
            let unit = &units[unit_index];
            let spatial = &spatial_indexes[&unit.choices.cache_key()];
            best_objects_on_cached_rings(
                graph,
                unit,
                spatial,
                targets[unit_index],
                objects,
                occupied,
                base_usage,
                &bidder_legality[bidder],
                &physical_resources[bidder_physical_resources[bidder]],
                &mut legality_workspace,
                prices,
                scale,
                &mut bidder_caches[bidder],
            )
        },
    )
    .map_err(auction_error)
}

#[allow(clippy::too_many_arguments)]
#[cfg(test)]
fn best_objects_on_rings(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    unit: &PlacementUnit,
    spatial: &SpatialChoiceIndex,
    target: Point,
    objects: AuctionObjects,
    occupied: &[bool],
    base_usage: &PlacementResourceUsage,
    legality_workspace: &mut AssignmentLegalityWorkspace,
    prices: &[u64],
    scale: u64,
) -> Result<BestObjects, AuctionError> {
    let prepared = PreparedUnitLegality::new(graph, constraints, unit);
    let physical = PreparedPhysicalResources::new(graph, constraints, unit, &prepared)?;
    best_objects_on_cached_rings(
        graph,
        unit,
        spatial,
        target,
        objects,
        occupied,
        base_usage,
        &prepared,
        &physical,
        legality_workspace,
        prices,
        scale,
        &mut BidderRingCache::default(),
    )
}

#[allow(clippy::too_many_arguments)]
fn best_objects_on_cached_rings(
    graph: &UnifiedGraph<'_>,
    unit: &PlacementUnit,
    spatial: &SpatialChoiceIndex,
    target: Point,
    objects: AuctionObjects,
    occupied: &[bool],
    base_usage: &PlacementResourceUsage,
    prepared: &PreparedUnitLegality,
    physical: &PreparedPhysicalResources,
    legality_workspace: &mut AssignmentLegalityWorkspace,
    prices: &[u64],
    scale: u64,
    cache: &mut BidderRingCache,
) -> Result<BestObjects, AuctionError> {
    let device = graph.device();
    let max_radius = device
        .width()
        .checked_sub(1)
        .and_then(|width| {
            device
                .height()
                .checked_sub(1)
                .and_then(|height| width.checked_add(height))
        })
        .ok_or(AuctionError::ArithmeticOverflow)?;
    let mut best = None::<PricedObject>;
    let mut second = None::<PricedObject>;
    let mut examined = 0_u64;
    let mut choice_start = 0;
    let mut radius = 0_u32;
    loop {
        if usize::try_from(radius).is_ok_and(|radius| radius < cache.ring_ends.len()) {
            let radius_index =
                usize::try_from(radius).map_err(|_| AuctionError::ArithmeticOverflow)?;
            let choice_end = cache.ring_ends[radius_index];
            for &object in &cache.objects[choice_start..choice_end] {
                examined = examined
                    .checked_add(1)
                    .ok_or(AuctionError::ArithmeticOverflow)?;
                insert_priced_object(object, radius, prices, scale, &mut best, &mut second)?;
            }
            choice_start = choice_end;
        } else {
            debug_assert_eq!(usize::try_from(radius).ok(), Some(cache.ring_ends.len()));
            let discovered_examined = discover_legal_ring(
                graph,
                unit,
                spatial,
                target,
                radius,
                objects,
                occupied,
                base_usage,
                prepared,
                physical,
                legality_workspace,
                cache,
            )?;
            examined = examined
                .checked_add(discovered_examined)
                .ok_or(AuctionError::ArithmeticOverflow)?;
            let choice_end = cache.objects.len();
            for &object in &cache.objects[choice_start..choice_end] {
                insert_priced_object(object, radius, prices, scale, &mut best, &mut second)?;
            }
            choice_start = choice_end;
            cache.ring_ends.push(choice_end);
        }

        if radius == max_radius {
            break;
        }
        let next_radius = radius
            .checked_add(1)
            .ok_or(AuctionError::ArithmeticOverflow)?;
        if let Some(second) = second {
            let unseen_lower_bound = scale
                .checked_mul(u64::from(next_radius))
                .ok_or(AuctionError::ArithmeticOverflow)?;
            // Strict inequality preserves the stable object-ID tie break when
            // an unseen object can equal the current second-best cost.
            if second.reduced_cost < unseen_lower_bound {
                break;
            }
        }
        radius = next_radius;
    }
    Ok(BestObjects {
        best,
        second,
        examined,
    })
}

#[allow(clippy::too_many_arguments)]
fn discover_legal_ring(
    graph: &UnifiedGraph<'_>,
    unit: &PlacementUnit,
    spatial: &SpatialChoiceIndex,
    target: Point,
    radius: u32,
    objects: AuctionObjects,
    occupied: &[bool],
    base_usage: &PlacementResourceUsage,
    prepared: &PreparedUnitLegality,
    physical: &PreparedPhysicalResources,
    legality_workspace: &mut AssignmentLegalityWorkspace,
    cache: &mut BidderRingCache,
) -> Result<u64, AuctionError> {
    let device = graph.device();
    let mut examined = 0_u64;
    for dy in 0..=radius {
        let dx = radius - dy;
        for y in super::ring_coordinates(target.y, dy, device.height()) {
            for x in super::ring_coordinates(target.x, dx, device.width()) {
                let point_index = y
                    .checked_mul(device.width())
                    .and_then(|row| row.checked_add(x))
                    .and_then(|index| usize::try_from(index).ok())
                    .ok_or(AuctionError::ArithmeticOverflow)?;
                for &choice in &spatial.by_point[point_index] {
                    examined = examined
                        .checked_add(1)
                        .ok_or(AuctionError::ArithmeticOverflow)?;
                    let assignment = unit.choices.assignment(choice);
                    if assignment_is_unoccupied(assignment, occupied)
                        && assignment_resources_are_legal_reusing_workspace(
                            graph,
                            base_usage,
                            prepared,
                            physical,
                            choice,
                            legality_workspace,
                        )
                    {
                        cache.objects.push(match objects {
                            AuctionObjects::SharedRows => choice,
                            AuctionObjects::SingletonBels => assignment[0].0,
                        });
                    }
                }
            }
        }
    }
    Ok(examined)
}

fn assignment_resources_are_legal_reusing_workspace(
    graph: &UnifiedGraph<'_>,
    usage: &PlacementResourceUsage,
    prepared: &PreparedUnitLegality,
    physical: &PreparedPhysicalResources,
    choice: usize,
    workspace: &mut AssignmentLegalityWorkspace,
) -> bool {
    workspace.pin_resources.clear();
    for &(slot, net) in &prepared.pins {
        workspace
            .pin_resources
            .push((physical.pin_wire(choice, slot), net));
    }
    workspace.shared_resources.clear();
    for (slot, (&(rule, _), &value)) in prepared
        .shape
        .shared
        .iter()
        .zip(&prepared.shared_values)
        .enumerate()
    {
        if let Some(resource) = physical.shared_resource(choice, slot) {
            workspace.shared_resources.push(((rule, resource), value));
        }
    }
    prepared_resource_lists_are_legal(graph, usage, workspace)
}

fn assignment_resources_are_legal_prepared_dynamic(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    assignment: &[BelId],
    usage: &PlacementResourceUsage,
    prepared: &PreparedUnitLegality,
    workspace: &mut AssignmentLegalityWorkspace,
) -> bool {
    workspace.pin_resources.clear();
    for &(slot, net) in &prepared.pins {
        let (column, pin) = prepared.pin_representatives[slot];
        let wire = candidate_pin_wire(graph, constraints, pin, assignment[column])
            .expect("placement candidate has every bound physical pin");
        workspace.pin_resources.push((wire, net));
    }
    workspace.shared_resources.clear();
    for (&(rule, column), &value) in prepared.shape.shared.iter().zip(&prepared.shared_values) {
        if let Some(&resource) = constraints.shared_resources[rule]
            .bel_resources
            .get(&assignment[column])
        {
            workspace.shared_resources.push(((rule, resource), value));
        }
    }
    prepared_resource_lists_are_legal(graph, usage, workspace)
}

fn prepared_resource_lists_are_legal(
    graph: &UnifiedGraph<'_>,
    usage: &PlacementResourceUsage,
    workspace: &mut AssignmentLegalityWorkspace,
) -> bool {
    let pin_resources = &mut workspace.pin_resources;
    pin_resources.sort_unstable();
    pin_resources.dedup();
    let mut start = 0;
    while start < pin_resources.len() {
        let wire = pin_resources[start].0;
        let mut end = start + 1;
        while end < pin_resources.len() && pin_resources[end].0 == wire {
            end += 1;
        }
        let existing = usage.pin_wires.get(&wire);
        let new_nets = pin_resources[start..end]
            .iter()
            .filter(|(_, net)| existing.is_none_or(|nets| !nets.contains_key(net)))
            .count();
        if existing.map_or(0, std::collections::HashMap::len) + new_nets
            > usize::from(graph.device().wires()[wire.0].capacity)
        {
            return false;
        }
        start = end;
    }

    let shared_resources = &mut workspace.shared_resources;
    shared_resources.sort_unstable();
    shared_resources.dedup();
    let mut start = 0;
    while start < shared_resources.len() {
        let resource = shared_resources[start].0;
        let mut end = start + 1;
        while end < shared_resources.len() && shared_resources[end].0 == resource {
            end += 1;
        }
        let existing = usage.shared.get(&resource);
        let new_values = shared_resources[start..end]
            .iter()
            .filter(|(_, value)| existing.is_none_or(|values| !values.contains_key(value)))
            .count();
        if existing.map_or(0, std::collections::HashMap::len) + new_values > 1 {
            return false;
        }
        start = end;
    }
    true
}

#[allow(clippy::too_many_arguments)]
fn insert_priced_object(
    object: usize,
    radius: u32,
    prices: &[u64],
    scale: u64,
    best: &mut Option<PricedObject>,
    second: &mut Option<PricedObject>,
) -> Result<(), AuctionError> {
    let base_cost = u64::from(radius);
    let reduced_cost = scale
        .checked_mul(base_cost)
        .and_then(|cost| cost.checked_add(prices[object]))
        .ok_or(AuctionError::ArithmeticOverflow)?;
    insert_best_two(
        best,
        second,
        PricedObject {
            object,
            base_cost,
            reduced_cost,
        },
    );
    Ok(())
}

fn assignment_is_unoccupied(assignment: &[BelId], occupied: &[bool]) -> bool {
    assignment.iter().all(|bel| !occupied[bel.0])
}

fn mark_assignment_occupied(assignment: &[BelId], occupied: &mut [bool]) {
    for &bel in assignment {
        occupied[bel.0] = true;
    }
}

fn insert_best_two(
    best: &mut Option<PricedObject>,
    second: &mut Option<PricedObject>,
    candidate: PricedObject,
) {
    let key = |choice: PricedObject| (choice.reduced_cost, choice.object);
    if best.is_none_or(|current| key(candidate) < key(current)) {
        *second = best.replace(candidate);
    } else if second.is_none_or(|current| key(candidate) < key(current)) {
        *second = Some(candidate);
    }
}

fn auction_error(error: AuctionError) -> PnrError {
    projection_error(&format!("implicit auction invariant failed: {error:?}"))
}

fn projection_error(reason: &str) -> PnrError {
    PnrError::InvalidPlacement {
        reason: reason.into(),
    }
}

#[cfg(test)]
mod tests {
    use super::{
        AssignmentLegalityWorkspace, AuctionObjects, BidderRingCache, PreparedPhysicalResources,
        PreparedUnitLegality, assignment_is_unoccupied, best_objects_on_cached_rings,
        best_objects_on_rings, prepare_physical_resources, project, rows_are_pairwise_bel_disjoint,
    };
    use crate::{
        PlacementChoices, PlacementConstraints, PlacementResourceUsage, PlacementUnit,
        SpatialChoiceIndex,
    };
    use std::collections::{BTreeMap, BTreeSet};
    use std::sync::Arc;
    use texo_model::{BelId, Design, Device, PinDirection, Point, ResourceKind, UnifiedGraph};

    fn projection_problem(
        coordinates: &[u32],
        candidate_sets: &[Vec<usize>],
        targets: &[u32],
    ) -> (Design, Device, Vec<PlacementUnit>, Vec<Point>) {
        let mut design = Design::new();
        for index in 0..candidate_sets.len() {
            design.add_cell(format!("cell{index}"), ResourceKind::Logic);
        }
        let width = coordinates.iter().copied().max().unwrap_or(0) + 1;
        let mut device = Device::new("projection-mre", width, 1).unwrap();
        let bels = coordinates
            .iter()
            .enumerate()
            .map(|(index, &x)| {
                device
                    .add_bel(format!("bel{index}"), ResourceKind::Logic, Point::new(x, 0))
                    .unwrap()
            })
            .collect::<Vec<_>>();
        let units = candidate_sets
            .iter()
            .enumerate()
            .map(|(cell, candidates)| PlacementUnit {
                cells: vec![texo_model::CellId(cell)],
                choices: PlacementChoices::SingleCell(
                    candidates
                        .iter()
                        .map(|&candidate| bels[candidate])
                        .collect::<Vec<_>>()
                        .into(),
                ),
            })
            .collect();
        let targets = targets.iter().map(|&x| Point::new(x, 0)).collect();
        (design, device, units, targets)
    }

    fn run_projection(
        design: &Design,
        device: &Device,
        units: &[PlacementUnit],
        targets: &[Point],
    ) -> Vec<BelId> {
        let graph = UnifiedGraph::new(design, device);
        let indexes = units
            .iter()
            .map(|unit| {
                (
                    unit.choices.cache_key(),
                    Arc::new(SpatialChoiceIndex::new(&unit.choices, device)),
                )
            })
            .collect::<BTreeMap<_, _>>();
        project(
            &graph,
            &PlacementConstraints::new(),
            units,
            &indexes,
            targets,
        )
        .unwrap()
        .into_iter()
        .map(Option::unwrap)
        .collect()
    }

    #[test]
    fn singleton_projection_avoids_far_greedy_spill() {
        let (design, device, units, targets) =
            projection_problem(&[0, 1, 100], &[vec![0, 1, 2], vec![0, 2]], &[0, 0]);
        let placed = run_projection(&design, &device, &units, &targets);

        assert_eq!(placed, [BelId(1), BelId(0)]);
    }

    #[test]
    fn singleton_projection_avoids_false_no_bel_behind_fixed_owner() {
        let (design, device, units, targets) = projection_problem(
            &[0, 1, 100],
            &[vec![0, 1], vec![0, 2], vec![2]],
            &[0, 0, 100],
        );
        let placed = run_projection(&design, &device, &units, &targets);

        assert_eq!(placed, [BelId(1), BelId(0), BelId(2)]);
    }

    #[test]
    fn projection_uses_four_real_bels_at_one_point() {
        let (design, device, units, targets) =
            projection_problem(&[0, 0, 0, 0], &vec![vec![0, 1, 2, 3]; 4], &[0, 0, 0, 0]);
        let placed = run_projection(&design, &device, &units, &targets);

        assert_eq!(placed.into_iter().collect::<BTreeSet<_>>().len(), 4);
    }

    #[test]
    fn projection_reaches_candidate_after_legacy_sixty_four_limit() {
        let mut candidate_sets = (0..65).map(|index| vec![index]).collect::<Vec<_>>();
        candidate_sets.push((0..66).collect());
        let (design, device, units, targets) =
            projection_problem(&vec![0; 66], &candidate_sets, &vec![0; 66]);
        let placed = run_projection(&design, &device, &units, &targets);

        assert_eq!(placed[65], BelId(65));
    }

    #[test]
    fn composite_slot_classification_requires_disjoint_complete_rows() {
        let safe = PlacementChoices::Shared(
            vec![vec![BelId(0), BelId(1)], vec![BelId(2), BelId(3)]].into(),
        );
        let sliding = PlacementChoices::Shared(
            vec![vec![BelId(0), BelId(1)], vec![BelId(1), BelId(2)]].into(),
        );

        assert!(rows_are_pairwise_bel_disjoint(&safe));
        assert!(!rows_are_pairwise_bel_disjoint(&sliding));
        assert!(!assignment_is_unoccupied(
            safe.assignment(0),
            &[false, true, false, false]
        ));
    }

    #[test]
    fn disjoint_composite_rows_remain_atomic_before_singleton_projection() {
        let mut design = Design::new();
        let macro_a = [
            design.add_cell("macro-a0", ResourceKind::Logic),
            design.add_cell("macro-a1", ResourceKind::Logic),
        ];
        let macro_b = [
            design.add_cell("macro-b0", ResourceKind::Logic),
            design.add_cell("macro-b1", ResourceKind::Logic),
        ];
        let singleton = design.add_cell("singleton", ResourceKind::Logic);
        let mut device = Device::new("atomic-disjoint", 3, 1).unwrap();
        let bels = [0, 0, 1, 1, 2].map(|x| {
            let index = device.bels().len();
            device
                .add_bel(format!("bel{index}"), ResourceKind::Logic, Point::new(x, 0))
                .unwrap()
        });
        let rows: Arc<[Vec<BelId>]> = Arc::from([vec![bels[0], bels[1]], vec![bels[2], bels[3]]]);
        let units = vec![
            PlacementUnit {
                cells: macro_a.into(),
                choices: PlacementChoices::Shared(Arc::clone(&rows)),
            },
            PlacementUnit {
                cells: macro_b.into(),
                choices: PlacementChoices::Shared(rows),
            },
            PlacementUnit {
                cells: vec![singleton],
                choices: PlacementChoices::SingleCell(Arc::from([bels[1], bels[4]])),
            },
        ];
        let placed = run_projection(&design, &device, &units, &[Point::new(0, 0); 3]);

        assert_eq!(placed.iter().copied().collect::<BTreeSet<_>>().len(), 5);
        assert_eq!(placed[singleton.0], bels[4]);
        for unit in &units[..2] {
            let assignment = unit
                .cells
                .iter()
                .map(|cell| placed[cell.0])
                .collect::<Vec<_>>();
            assert!(unit.choices.contains(&assignment));
        }
    }

    #[test]
    fn lazy_ring_continues_across_equal_lower_bound_for_stable_tie() {
        // The two center objects establish a reduced-cost second best of 2.
        // The unseen radius-one lower bound is also 2, so object 0 must still
        // be visited and win the stable equal-cost object-ID tie.
        let (design, device, units, targets) =
            projection_problem(&[1, 0, 0], &[vec![0, 1, 2]], &[0]);
        let graph = UnifiedGraph::new(&design, &device);
        let constraints = PlacementConstraints::new();
        let spatial = SpatialChoiceIndex::new(&units[0].choices, &device);
        let result = best_objects_on_rings(
            &graph,
            &constraints,
            &units[0],
            &spatial,
            targets[0],
            AuctionObjects::SingletonBels,
            &vec![false; device.bels().len()],
            &PlacementResourceUsage::default(),
            &mut AssignmentLegalityWorkspace::default(),
            &[0, 2, 2],
            2,
        )
        .unwrap();

        assert_eq!(result.best.unwrap().object, 0);
        assert_eq!(result.best.unwrap().reduced_cost, 2);
        assert_eq!(result.second.unwrap().object, 1);
    }

    #[test]
    fn lazy_ring_filters_occupied_and_handles_zero_or_one_real_candidate() {
        let (design, device, units, targets) =
            projection_problem(&[1, 0, 0], &[vec![0, 1, 2]], &[0]);
        let graph = UnifiedGraph::new(&design, &device);
        let constraints = PlacementConstraints::new();
        let spatial = SpatialChoiceIndex::new(&units[0].choices, &device);
        let usage = PlacementResourceUsage::default();
        let mut workspace = AssignmentLegalityWorkspace::default();
        let one = best_objects_on_rings(
            &graph,
            &constraints,
            &units[0],
            &spatial,
            targets[0],
            AuctionObjects::SingletonBels,
            &[true, true, false],
            &usage,
            &mut workspace,
            &[0, 0, 0],
            1,
        )
        .unwrap();
        assert_eq!(one.best.unwrap().object, 2);
        assert!(one.second.is_none());

        let none = best_objects_on_rings(
            &graph,
            &constraints,
            &units[0],
            &spatial,
            targets[0],
            AuctionObjects::SingletonBels,
            &[true, true, true],
            &usage,
            &mut workspace,
            &[0, 0, 0],
            1,
        )
        .unwrap();
        assert!(none.best.is_none());
        assert!(none.second.is_none());
    }

    #[test]
    fn repeated_price_queries_reuse_legality_complete_rings_exactly() {
        let (design, device, units, targets) =
            projection_problem(&[0, 0, 1, 2], &[vec![0, 1, 2, 3]], &[0]);
        let graph = UnifiedGraph::new(&design, &device);
        let constraints = PlacementConstraints::new();
        let spatial = SpatialChoiceIndex::new(&units[0].choices, &device);
        let usage = PlacementResourceUsage::default();
        let occupied = vec![false; device.bels().len()];
        let mut workspace = AssignmentLegalityWorkspace::default();
        let mut cache = BidderRingCache::default();
        let prepared = PreparedUnitLegality::new(&graph, &constraints, &units[0]);
        let physical =
            PreparedPhysicalResources::new(&graph, &constraints, &units[0], &prepared).unwrap();

        let first = best_objects_on_cached_rings(
            &graph,
            &units[0],
            &spatial,
            targets[0],
            AuctionObjects::SingletonBels,
            &occupied,
            &usage,
            &prepared,
            &physical,
            &mut workspace,
            &[0, 0, 0, 0],
            1,
            &mut cache,
        )
        .unwrap();
        assert_eq!(first.best.unwrap().object, 0);
        assert_eq!(first.second.unwrap().object, 1);
        assert_eq!(cache.ring_ends.len(), 1);
        assert_eq!(cache.objects.len(), 2);

        // Raising both radius-zero prices forces discovery of radius one.
        // The answer must match an uncached exact ring walk while the old
        // ring remains represented exactly once in the cache.
        let prices = [3, 3, 0, 0];
        let cached = best_objects_on_cached_rings(
            &graph,
            &units[0],
            &spatial,
            targets[0],
            AuctionObjects::SingletonBels,
            &occupied,
            &usage,
            &prepared,
            &physical,
            &mut workspace,
            &prices,
            2,
            &mut cache,
        )
        .unwrap();
        let fresh = best_objects_on_rings(
            &graph,
            &constraints,
            &units[0],
            &spatial,
            targets[0],
            AuctionObjects::SingletonBels,
            &occupied,
            &usage,
            &mut workspace,
            &prices,
            2,
        )
        .unwrap();
        assert_eq!((cached.best, cached.second), (fresh.best, fresh.second));
        assert_eq!(cache.ring_ends.len(), 2);
        assert_eq!(cache.objects, [0, 1, 2]);

        let cached_choice_count = cache.objects.len();
        let repeated = best_objects_on_cached_rings(
            &graph,
            &units[0],
            &spatial,
            targets[0],
            AuctionObjects::SingletonBels,
            &occupied,
            &usage,
            &prepared,
            &physical,
            &mut workspace,
            &prices,
            2,
            &mut cache,
        )
        .unwrap();
        assert_eq!((repeated.best, repeated.second), (fresh.best, fresh.second));
        assert_eq!(cache.objects.len(), cached_choice_count);
    }

    #[test]
    fn shared_physical_table_matches_generic_pin_and_shared_legality() {
        let mut design = Design::new();
        let driver_a = design.add_cell("driver-a", ResourceKind::Logic);
        let driver_b = design.add_cell("driver-b", ResourceKind::Logic);
        let sink_a = design.add_cell("sink-a", ResourceKind::Logic);
        let sink_b = design.add_cell("sink-b", ResourceKind::Logic);
        let first_driver_output = design
            .add_pin(driver_a, "out", PinDirection::Output)
            .unwrap();
        let second_driver_output = design
            .add_pin(driver_b, "out", PinDirection::Output)
            .unwrap();
        let first_sink_input = design.add_pin(sink_a, "in", PinDirection::Input).unwrap();
        let second_sink_input = design.add_pin(sink_b, "in", PinDirection::Input).unwrap();
        let net_a = design
            .add_net("a", first_driver_output, [first_sink_input])
            .unwrap();
        let net_b = design
            .add_net("b", second_driver_output, [second_sink_input])
            .unwrap();
        let device = Device::rectangular_logic(2, 1).unwrap();
        let bels = Arc::<[BelId]>::from((0..device.bels().len()).map(BelId).collect::<Vec<_>>());
        let units = [sink_a, sink_b]
            .map(|cell| PlacementUnit {
                cells: vec![cell],
                choices: PlacementChoices::SingleCell(Arc::clone(&bels)),
            })
            .to_vec();
        let mut constraints = PlacementConstraints::new();
        constraints.add_shared_resource(
            [(sink_a, 10), (sink_b, 11)],
            bels.iter().copied().map(|bel| (bel, 7)),
        );
        let graph = UnifiedGraph::new(&design, &device);
        let prepared = units
            .iter()
            .map(|unit| PreparedUnitLegality::new(&graph, &constraints, unit))
            .collect::<Vec<_>>();
        let (physical, bidder_tables) =
            prepare_physical_resources(&graph, &constraints, &units, &[0, 1], &prepared).unwrap();

        // The units have the same physical pin/resource shape but distinct
        // logical nets and shared values, so only the candidate-side table is
        // reusable between them.
        assert_eq!(physical.len(), 1);
        assert_eq!(bidder_tables, [0, 0]);
        let assert_equivalent = |usage: &PlacementResourceUsage| {
            for (bidder, unit) in units.iter().enumerate() {
                for choice in 0..unit.choices.len() {
                    let expected = crate::assignment_resources_are_legal_with_workspace(
                        &graph,
                        &constraints,
                        &unit.cells,
                        unit.choices.assignment(choice),
                        usage,
                        &mut Vec::new(),
                    );
                    let actual = super::assignment_resources_are_legal_reusing_workspace(
                        &graph,
                        usage,
                        &prepared[bidder],
                        &physical[bidder_tables[bidder]],
                        choice,
                        &mut AssignmentLegalityWorkspace::default(),
                    );
                    assert_eq!(actual, expected, "bidder={bidder} choice={choice}");
                }
            }
        };

        assert_equivalent(&PlacementResourceUsage::default());

        let mut pin_conflict = PlacementResourceUsage::default();
        let first_input_wire = physical[0].pin_wire(0, 0);
        pin_conflict
            .pin_wires
            .entry(first_input_wire)
            .or_default()
            .insert(net_b, 1);
        assert_equivalent(&pin_conflict);
        assert_ne!(net_a, net_b);

        let mut shared_conflict = PlacementResourceUsage::default();
        shared_conflict
            .shared
            .entry((0, 7))
            .or_default()
            .insert(10, 1);
        assert_equivalent(&shared_conflict);
    }
}
