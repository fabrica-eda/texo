//! Exact transactional repair of a relaxed analytical placement.
//!
//! The auction deliberately ignores constraints which are not a rectangular
//! one-bidder/one-object assignment.  This module restores those constraints
//! without an artificial radius, candidate, displacement, or search-depth
//! cap.  A search state is always a legal partial placement.  Trying one
//! assignment atomically removes every blocking placement unit, installs the
//! complete assignment, and repairs displaced units with explicit search
//! frames so a long augmenting chain cannot overflow the process stack.

use std::cmp::Reverse;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::sync::Arc;
use std::time::Instant;

use texo_model::{BelId, NetId, Point, UnifiedGraph, WireId};

use super::super::{
    PlacementConstraints, PlacementResourceUsage, PlacementUnit, PnrError, SpatialChoiceIndex,
    assignment_resources_are_legal, update_placement_resource_usage,
    visit_assignment_pin_resources, visit_assignment_shared_resources,
};
use super::projection_error;

type UnitId = usize;
type ChoiceId = usize;
type SharedResource = (usize, u64);
const UNPLACED_CHOICE: ChoiceId = usize::MAX;

/// Work performed by one exact-feasibility repair.
///
/// These counters expose whether the relaxed projection leaves a genuinely
/// local cleanup problem or falls into the generalized-placement exponential
/// fallback.  They never alter or limit the search.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(super) struct RepairStats {
    pub(super) initial_pending: usize,
    pub(super) direct_fast_choices: u64,
    pub(super) direct_fast_moves: u64,
    pub(super) direct_fast_fallbacks: u64,
    pub(super) augmenting_roots: u64,
    pub(super) augmenting_fallbacks: u64,
    pub(super) states_visited: u64,
    pub(super) failed_states: usize,
    pub(super) max_unplaced: usize,
    pub(super) max_blocker_branches: usize,
    pub(super) units_evicted: u64,
    pub(super) choices_examined: u64,
}

/// Repairs every conflict in a relaxed unit-to-assignment projection.
///
/// `preferred` is allowed to contain unmatched units and mutually conflicting
/// assignments.  `fixed` units must be matched and mutually legal.  The
/// returned vector is indexed by logical cell ID, like the former greedy
/// legalization seam.  This is a complete feasibility search, not a proof of
/// globally minimum legalization cost.  Candidate ordering preserves the
/// auction proposal first and then walks increasing Manhattan distance.
#[allow(clippy::too_many_arguments)]
pub(super) fn repair_relaxed(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    units: &[PlacementUnit],
    spatial_indexes: &BTreeMap<(u8, usize), Arc<SpatialChoiceIndex>>,
    targets: &[Point],
    preferred: &[Option<ChoiceId>],
    fixed: &[bool],
) -> Result<(Vec<Option<BelId>>, RepairStats), PnrError> {
    if preferred.len() != units.len() || fixed.len() != units.len() || targets.len() != units.len()
    {
        return Err(projection_error(
            "transactional repair input lengths do not match placement units",
        ));
    }
    for (unit, choice) in preferred.iter().enumerate() {
        if choice.is_some_and(|choice| choice >= units[unit].choices.len()) {
            return Err(projection_error(
                "transactional repair received an out-of-range assignment",
            ));
        }
        if fixed[unit] && choice.is_none() {
            return Err(projection_error(
                "transactional repair received an unmatched fixed unit",
            ));
        }
    }

    let pending = initial_pending_units(graph, constraints, units, preferred, fixed)?;
    let metrics_enabled = std::env::var_os("TEXO_PNR_METRICS").is_some();
    let verbose_metrics = std::env::var_os("TEXO_PNR_VERBOSE_METRICS").is_some();
    let metrics_started = (metrics_enabled || verbose_metrics).then(Instant::now);
    if let Some(started) = metrics_started {
        eprintln!(
            "TEXO_PNR_METRICS repair start elapsed_ms={} initial_pending={} total_units={}",
            started.elapsed().as_millis(),
            pending.len(),
            units.len()
        );
    }
    let mut state = RepairState::new(graph, constraints, units, preferred, fixed);
    for unit in fixed
        .iter()
        .enumerate()
        .filter_map(|(unit, &is_fixed)| is_fixed.then_some(unit))
        .chain(
            fixed
                .iter()
                .enumerate()
                .filter_map(|(unit, &is_fixed)| (!is_fixed).then_some(unit)),
        )
    {
        let Some(choice) = preferred[unit] else {
            continue;
        };
        if pending.contains(&unit) {
            continue;
        }
        if !state.can_install(unit, choice) {
            return Err(projection_error(
                "relaxed-conflict removal did not leave a legal repair base",
            ));
        }
        state.raw_install(unit, choice);
    }
    state.assert_consistent();

    let first_pending = state.next_unplaced();
    let mut search = RepairSearch {
        state,
        spatial_indexes,
        targets,
        visiting: HashSet::new(),
        failed: HashSet::new(),
        metrics_started,
        verbose_metrics,
        stats: RepairStats {
            initial_pending: pending.len(),
            max_unplaced: pending.len(),
            ..RepairStats::default()
        },
    };
    if !search.solve() {
        search.report_metrics("failed");
        let unit = first_pending.unwrap_or(0);
        return Err(PnrError::NoBel {
            cell: graph.design().cells()[units[unit].cells[0].0].name.clone(),
        });
    }
    search.state.assert_consistent();
    search.stats.failed_states = search.failed.len();
    search.report_metrics("complete");
    Ok((search.state.placed, search.stats))
}

/// Candidate resources normalized to the units used by the legality model.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct AssignmentFootprint {
    pin_wires: Vec<(WireId, NetId)>,
    shared: Vec<(SharedResource, u64)>,
}

fn assignment_footprint(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    unit: &PlacementUnit,
    choice: ChoiceId,
) -> AssignmentFootprint {
    let assignment = unit.choices.assignment(choice);
    let mut pin_wires = Vec::new();
    visit_assignment_pin_resources(graph, constraints, &unit.cells, assignment, |wire, net| {
        pin_wires.push((wire, net));
    });
    pin_wires.sort_unstable();
    pin_wires.dedup();

    let mut shared = Vec::new();
    visit_assignment_shared_resources(
        constraints,
        &unit.cells,
        assignment,
        |rule, resource, value| shared.push(((rule, resource), value)),
    );
    shared.sort_unstable();
    shared.dedup();
    AssignmentFootprint { pin_wires, shared }
}

/// Ownership for one legal partial placement.
///
/// `PlacementResourceUsage` stores multiplicities but intentionally not unit
/// identity.  Repair needs both, so this sparse index is updated in the same
/// transaction as the canonical usage object.
#[derive(Default)]
struct PlacementOwners {
    bels: Vec<Option<UnitId>>,
    pin_wires: HashMap<WireId, HashMap<NetId, BTreeSet<UnitId>>>,
    shared: HashMap<SharedResource, HashMap<u64, BTreeSet<UnitId>>>,
}

impl PlacementOwners {
    fn new(bel_count: usize) -> Self {
        Self {
            bels: vec![None; bel_count],
            ..Self::default()
        }
    }

    fn insert(&mut self, unit: UnitId, assignment: &[BelId], footprint: &AssignmentFootprint) {
        for &bel in assignment {
            debug_assert!(self.bels[bel.0].is_none());
            self.bels[bel.0] = Some(unit);
        }
        for &(wire, net) in &footprint.pin_wires {
            self.pin_wires
                .entry(wire)
                .or_default()
                .entry(net)
                .or_default()
                .insert(unit);
        }
        for &(resource, value) in &footprint.shared {
            self.shared
                .entry(resource)
                .or_default()
                .entry(value)
                .or_default()
                .insert(unit);
        }
    }

    fn remove(&mut self, unit: UnitId, assignment: &[BelId], footprint: &AssignmentFootprint) {
        for &bel in assignment {
            debug_assert_eq!(self.bels[bel.0], Some(unit));
            self.bels[bel.0] = None;
        }
        for &(wire, net) in &footprint.pin_wires {
            let remove_wire = {
                let nets = self
                    .pin_wires
                    .get_mut(&wire)
                    .expect("installed pin wire has owner index");
                let remove_net = {
                    let owners = nets
                        .get_mut(&net)
                        .expect("installed pin net has owner index");
                    assert!(owners.remove(&unit));
                    owners.is_empty()
                };
                if remove_net {
                    nets.remove(&net);
                }
                nets.is_empty()
            };
            if remove_wire {
                self.pin_wires.remove(&wire);
            }
        }
        for &(resource, value) in &footprint.shared {
            let remove_resource = {
                let values = self
                    .shared
                    .get_mut(&resource)
                    .expect("installed shared resource has owner index");
                let remove_value = {
                    let owners = values
                        .get_mut(&value)
                        .expect("installed shared value has owner index");
                    assert!(owners.remove(&unit));
                    owners.is_empty()
                };
                if remove_value {
                    values.remove(&value);
                }
                values.is_empty()
            };
            if remove_resource {
                self.shared.remove(&resource);
            }
        }
    }
}

/// Complete ownership of the relaxed proposal, which may be illegal.
#[derive(Default)]
struct RelaxedOwners {
    bels: Vec<Vec<UnitId>>,
    pin_wires: HashMap<WireId, HashMap<NetId, BTreeSet<UnitId>>>,
    shared: HashMap<SharedResource, HashMap<u64, BTreeSet<UnitId>>>,
}

impl RelaxedOwners {
    fn new(bel_count: usize) -> Self {
        Self {
            bels: vec![Vec::new(); bel_count],
            ..Self::default()
        }
    }

    fn insert(&mut self, unit: UnitId, assignment: &[BelId], footprint: &AssignmentFootprint) {
        for &bel in assignment {
            self.bels[bel.0].push(unit);
        }
        for &(wire, net) in &footprint.pin_wires {
            self.pin_wires
                .entry(wire)
                .or_default()
                .entry(net)
                .or_default()
                .insert(unit);
        }
        for &(resource, value) in &footprint.shared {
            self.shared
                .entry(resource)
                .or_default()
                .entry(value)
                .or_default()
                .insert(unit);
        }
    }
}

fn initial_pending_units(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    units: &[PlacementUnit],
    preferred: &[Option<ChoiceId>],
    fixed: &[bool],
) -> Result<BTreeSet<UnitId>, PnrError> {
    // Keep a deterministic legal subset of every relaxed conflict instead of
    // removing every participant.  The retained movable owners remain normal
    // repair blockers and may be evicted later, so this shrinks the initial
    // frontier without restricting the feasible state space.
    let mut owners = RelaxedOwners::new(graph.device().bels().len());
    let mut pending = BTreeSet::new();
    for (unit, &choice) in preferred.iter().enumerate() {
        let Some(choice) = choice else {
            pending.insert(unit);
            continue;
        };
        let assignment = units[unit].choices.assignment(choice);
        let footprint = assignment_footprint(graph, constraints, &units[unit], choice);
        owners.insert(unit, assignment, &footprint);
    }

    for bel_owners in owners.bels.iter().filter(|owners| owners.len() > 1) {
        let fixed_owners = bel_owners
            .iter()
            .copied()
            .filter(|&unit| fixed[unit])
            .collect::<Vec<_>>();
        if fixed_owners.len() > 1 {
            return Err(projection_error(
                "multiple fixed units occupy one BEL during repair",
            ));
        }
        let survivor = fixed_owners.first().copied().or_else(|| {
            bel_owners.iter().copied().min_by_key(|&unit| {
                (
                    units[unit].choices.len(),
                    Reverse(units[unit].cells.len()),
                    unit,
                )
            })
        });
        pending.extend(
            bel_owners
                .iter()
                .copied()
                .filter(|unit| Some(*unit) != survivor),
        );
    }
    for values in owners.shared.values().filter(|values| values.len() > 1) {
        let fixed_values = values
            .iter()
            .filter_map(|(&value, owners)| owners.iter().any(|&unit| fixed[unit]).then_some(value))
            .collect::<Vec<_>>();
        if fixed_values.len() > 1 {
            return Err(projection_error(
                "fixed units require incompatible shared-resource values",
            ));
        }
        let survivor = fixed_values.first().copied().or_else(|| {
            values
                .iter()
                .min_by_key(|&(&value, owners)| (Reverse(owners.len()), value))
                .map(|(&value, _)| value)
        });
        pending.extend(
            values
                .iter()
                .filter(|(value, _)| Some(**value) != survivor)
                .flat_map(|(_, owners)| owners.iter().copied()),
        );
    }
    for (&wire, nets) in &owners.pin_wires {
        let capacity = usize::from(graph.device().wires()[wire.0].capacity);
        if nets.len() <= capacity {
            continue;
        }
        let mut ranked = nets.iter().collect::<Vec<_>>();
        ranked.sort_unstable_by_key(|entry| {
            let (net, owners) = *entry;
            (
                Reverse(owners.iter().any(|&unit| fixed[unit])),
                Reverse(owners.len()),
                *net,
            )
        });
        let fixed_nets = ranked
            .iter()
            .filter(|(_, owners)| owners.iter().any(|&unit| fixed[unit]))
            .count();
        if fixed_nets > capacity {
            return Err(projection_error(
                "fixed units exceed a placement pin-wire capacity",
            ));
        }
        for (_, owners) in ranked.into_iter().skip(capacity) {
            debug_assert!(owners.iter().all(|&unit| !fixed[unit]));
            pending.extend(owners.iter().copied());
        }
    }
    Ok(pending)
}

#[derive(Clone, Copy, Debug)]
enum JournalEntry {
    Removed { unit: UnitId, choice: ChoiceId },
    Installed { unit: UnitId, choice: ChoiceId },
}

struct RepairState<'a, 'graph> {
    graph: &'a UnifiedGraph<'graph>,
    constraints: &'a PlacementConstraints,
    units: &'a [PlacementUnit],
    preferred: &'a [Option<ChoiceId>],
    fixed: &'a [bool],
    choice_by_unit: Vec<Option<ChoiceId>>,
    unplaced_count: usize,
    // Exact sparse state relative to `preferred`, maintained by the same raw
    // operations as the resource indexes so memoization is O(changed units).
    deviations: BTreeMap<UnitId, ChoiceId>,
    placed: Vec<Option<BelId>>,
    usage: PlacementResourceUsage,
    owners: PlacementOwners,
    journal: Vec<JournalEntry>,
}

impl<'a, 'graph> RepairState<'a, 'graph> {
    fn new(
        graph: &'a UnifiedGraph<'graph>,
        constraints: &'a PlacementConstraints,
        units: &'a [PlacementUnit],
        preferred: &'a [Option<ChoiceId>],
        fixed: &'a [bool],
    ) -> Self {
        let deviations = preferred
            .iter()
            .enumerate()
            .filter_map(|(unit, choice)| choice.is_some().then_some((unit, UNPLACED_CHOICE)))
            .collect();
        Self {
            graph,
            constraints,
            units,
            preferred,
            fixed,
            choice_by_unit: vec![None; units.len()],
            unplaced_count: units.len(),
            deviations,
            placed: vec![None; graph.design().cells().len()],
            usage: PlacementResourceUsage::default(),
            owners: PlacementOwners::new(graph.device().bels().len()),
            journal: Vec::new(),
        }
    }

    fn record_choice(&mut self, unit: UnitId, choice: Option<ChoiceId>) {
        if choice == self.preferred[unit] {
            self.deviations.remove(&unit);
        } else {
            self.deviations
                .insert(unit, choice.unwrap_or(UNPLACED_CHOICE));
        }
    }

    fn can_install(&self, unit: UnitId, choice: ChoiceId) -> bool {
        let placement_unit = &self.units[unit];
        let assignment = placement_unit.choices.assignment(choice);
        assignment
            .iter()
            .all(|&bel| self.owners.bels[bel.0].is_none())
            && assignment_resources_are_legal(
                self.graph,
                self.constraints,
                &placement_unit.cells,
                assignment,
                &self.usage,
            )
    }

    fn raw_install(&mut self, unit: UnitId, choice: ChoiceId) {
        debug_assert!(self.choice_by_unit[unit].is_none());
        debug_assert!(self.can_install(unit, choice));
        let placement_unit = &self.units[unit];
        let assignment = placement_unit.choices.assignment(choice).to_vec();
        let footprint = assignment_footprint(self.graph, self.constraints, placement_unit, choice);
        self.owners.insert(unit, &assignment, &footprint);
        update_placement_resource_usage(
            self.graph,
            self.constraints,
            &placement_unit.cells,
            &assignment,
            &mut self.usage,
            true,
        );
        for (&cell, &bel) in placement_unit.cells.iter().zip(&assignment) {
            debug_assert!(self.placed[cell.0].is_none());
            self.placed[cell.0] = Some(bel);
        }
        self.choice_by_unit[unit] = Some(choice);
        self.unplaced_count -= 1;
        self.record_choice(unit, Some(choice));
    }

    fn raw_remove(&mut self, unit: UnitId, choice: ChoiceId) {
        debug_assert_eq!(self.choice_by_unit[unit], Some(choice));
        let placement_unit = &self.units[unit];
        let assignment = placement_unit.choices.assignment(choice).to_vec();
        let footprint = assignment_footprint(self.graph, self.constraints, placement_unit, choice);
        update_placement_resource_usage(
            self.graph,
            self.constraints,
            &placement_unit.cells,
            &assignment,
            &mut self.usage,
            false,
        );
        self.owners.remove(unit, &assignment, &footprint);
        for &cell in &placement_unit.cells {
            self.placed[cell.0] = None;
        }
        self.choice_by_unit[unit] = None;
        self.unplaced_count += 1;
        self.record_choice(unit, None);
    }

    fn remove_for_branch(&mut self, unit: UnitId) {
        debug_assert!(!self.fixed[unit]);
        let choice = self.choice_by_unit[unit].expect("blocker is installed");
        self.raw_remove(unit, choice);
        self.journal.push(JournalEntry::Removed { unit, choice });
    }

    fn install_for_branch(&mut self, unit: UnitId, choice: ChoiceId) {
        self.raw_install(unit, choice);
        self.journal.push(JournalEntry::Installed { unit, choice });
    }

    fn rollback(&mut self, checkpoint: usize) {
        while self.journal.len() > checkpoint {
            match self
                .journal
                .pop()
                .expect("journal is longer than checkpoint")
            {
                JournalEntry::Installed { unit, choice } => self.raw_remove(unit, choice),
                JournalEntry::Removed { unit, choice } => self.raw_install(unit, choice),
            }
        }
    }

    fn next_unplaced(&self) -> Option<UnitId> {
        (0..self.units.len())
            .filter(|&unit| self.choice_by_unit[unit].is_none())
            .min_by_key(|&unit| {
                (
                    self.units[unit].choices.len(),
                    Reverse(self.units[unit].cells.len()),
                    self.units[unit].cells[0],
                )
            })
    }

    /// Exact sparse key relative to the immutable auction proposal.
    fn key(&self) -> Vec<(UnitId, ChoiceId)> {
        self.deviations
            .iter()
            .map(|(&unit, &choice)| (unit, choice))
            .collect()
    }

    fn assert_consistent(&self) {
        #[cfg(debug_assertions)]
        {
            debug_assert_eq!(
                self.unplaced_count,
                self.choice_by_unit
                    .iter()
                    .filter(|choice| choice.is_none())
                    .count()
            );
            for (unit, choice) in self.choice_by_unit.iter().enumerate() {
                let deviation = self.deviations.get(&unit).copied();
                let expected =
                    (*choice != self.preferred[unit]).then_some(choice.unwrap_or(UNPLACED_CHOICE));
                debug_assert_eq!(deviation, expected);
                for &cell in &self.units[unit].cells {
                    debug_assert_eq!(self.placed[cell.0].is_some(), choice.is_some());
                }
                if self.fixed[unit] {
                    debug_assert!(choice.is_some());
                }
            }
            for (&wire, nets) in &self.owners.pin_wires {
                debug_assert!(
                    nets.len() <= usize::from(self.graph.device().wires()[wire.0].capacity)
                );
                debug_assert_eq!(
                    self.usage.pin_wires.get(&wire).map(HashMap::len),
                    Some(nets.len())
                );
            }
            for (resource, values) in &self.owners.shared {
                debug_assert!(values.len() <= 1);
                debug_assert_eq!(
                    self.usage.shared.get(resource).map(HashMap::len),
                    Some(values.len())
                );
            }
            debug_assert_eq!(self.usage.pin_wires.len(), self.owners.pin_wires.len());
            debug_assert_eq!(self.usage.shared.len(), self.owners.shared.len());
        }
    }
}

struct RepairSearch<'a, 'graph> {
    state: RepairState<'a, 'graph>,
    spatial_indexes: &'a BTreeMap<(u8, usize), Arc<SpatialChoiceIndex>>,
    targets: &'a [Point],
    visiting: HashSet<Vec<(UnitId, ChoiceId)>>,
    failed: HashSet<Vec<(UnitId, ChoiceId)>>,
    metrics_started: Option<Instant>,
    verbose_metrics: bool,
    stats: RepairStats,
}

/// Constant-memory Manhattan-ring cursor over one shared spatial domain.
struct ChoiceCursor<'a> {
    spatial: &'a SpatialChoiceIndex,
    width: u32,
    height: u32,
    target: Point,
    preferred: Option<ChoiceId>,
    preferred_pending: bool,
    radius: u64,
    max_radius: u64,
    next_x: Option<u32>,
    x_end: u32,
    pending_y: [Option<u32>; 2],
    next_y: usize,
    point: Option<Point>,
    bucket_index: usize,
}

impl<'a> ChoiceCursor<'a> {
    fn new(
        spatial: &'a SpatialChoiceIndex,
        width: u32,
        height: u32,
        target: Point,
        preferred: Option<ChoiceId>,
    ) -> Self {
        let mut cursor = Self {
            spatial,
            width,
            height,
            target,
            preferred,
            preferred_pending: preferred.is_some(),
            radius: 0,
            max_radius: u64::from(width - 1) + u64::from(height - 1),
            next_x: None,
            x_end: 0,
            pending_y: [None, None],
            next_y: 2,
            point: None,
            bucket_index: 0,
        };
        cursor.begin_ring();
        cursor
    }

    fn begin_ring(&mut self) {
        let coordinate_radius = u32::try_from(self.radius).unwrap_or(u32::MAX);
        self.next_x = Some(self.target.x.saturating_sub(coordinate_radius));
        self.x_end = self
            .target
            .x
            .saturating_add(coordinate_radius)
            .min(self.width - 1);
        self.pending_y = [None, None];
        self.next_y = 2;
        self.point = None;
        self.bucket_index = 0;
    }

    fn take_next_x(&mut self) -> bool {
        let Some(x) = self.next_x else {
            return false;
        };
        self.next_x = (x < self.x_end).then(|| {
            x.checked_add(1)
                .expect("x below the device endpoint can advance")
        });
        let dx = u64::from(x.abs_diff(self.target.x));
        if dx > self.radius {
            self.pending_y = [None, None];
            self.next_y = 2;
            return true;
        }
        let dy = u32::try_from(self.radius - dx).ok();
        let below = dy
            .and_then(|dy| self.target.y.checked_sub(dy))
            .filter(|&y| y < self.height);
        let above = dy
            .filter(|&dy| dy != 0)
            .and_then(|dy| self.target.y.checked_add(dy))
            .filter(|&y| y < self.height);
        self.pending_y = match (below, above) {
            (Some(below), Some(above)) => [Some(below), Some(above)],
            (Some(y), None) | (None, Some(y)) => [Some(y), None],
            (None, None) => [None, None],
        };
        self.next_y = 0;
        true
    }
}

impl Iterator for ChoiceCursor<'_> {
    type Item = ChoiceId;

    fn next(&mut self) -> Option<Self::Item> {
        if self.preferred_pending {
            self.preferred_pending = false;
            return self.preferred;
        }
        loop {
            if let Some(point) = self.point {
                let point_index = u64::from(point.y)
                    .checked_mul(u64::from(self.width))
                    .and_then(|row| row.checked_add(u64::from(point.x)))
                    .and_then(|index| usize::try_from(index).ok())
                    .expect("validated device area fits the spatial index");
                let bucket = &self.spatial.by_point[point_index];
                while self.bucket_index < bucket.len() {
                    let choice = bucket[self.bucket_index];
                    self.bucket_index += 1;
                    if Some(choice) != self.preferred {
                        return Some(choice);
                    }
                }
                self.point = None;
                self.bucket_index = 0;
            }
            while self.next_y < self.pending_y.len() {
                let y = self.pending_y[self.next_y];
                self.next_y += 1;
                if let Some(y) = y {
                    let x = self
                        .next_x
                        .map_or(self.x_end, |next| next.saturating_sub(1));
                    self.point = Some(Point::new(x, y));
                    break;
                }
            }
            if self.point.is_some() {
                continue;
            }
            if self.take_next_x() {
                continue;
            }
            if self.radius == self.max_radius {
                return None;
            }
            self.radius = self
                .radius
                .checked_add(1)
                .expect("finite u32 device Manhattan radius fits u64");
            self.begin_ring();
        }
    }
}

struct ExactFrame<'a> {
    unit: UnitId,
    key: Vec<(UnitId, ChoiceId)>,
    choices: ChoiceCursor<'a>,
    current_choice: Option<ChoiceId>,
    branches: Vec<BTreeSet<UnitId>>,
    next_branch: usize,
    /// Transaction installed by this frame while its child is active.
    resume_checkpoint: Option<usize>,
}

struct AugmentFrame<'a> {
    unit: UnitId,
    choices: ChoiceCursor<'a>,
    /// Transaction installed by this frame while its child is active.
    resume_checkpoint: Option<usize>,
}

fn choice_cursor_for<'a>(
    spatial_indexes: &'a BTreeMap<(u8, usize), Arc<SpatialChoiceIndex>>,
    state: &RepairState<'_, '_>,
    targets: &[Point],
    unit: UnitId,
) -> ChoiceCursor<'a> {
    let placement_unit = &state.units[unit];
    let spatial = spatial_indexes[&placement_unit.choices.cache_key()].as_ref();
    let device = state.graph.device();
    ChoiceCursor::new(
        spatial,
        device.width(),
        device.height(),
        targets[unit],
        state.preferred[unit],
    )
}

impl<'a> RepairSearch<'a, '_> {
    fn report_metrics(&self, event: &str) {
        let Some(started) = self.metrics_started else {
            return;
        };
        eprintln!(
            "TEXO_PNR_METRICS repair {event} elapsed_ms={} direct_fast_choices={} direct_fast_moves={} direct_fast_fallbacks={} augmenting_roots={} augmenting_fallbacks={} states_visited={} failed_states={} choices_examined={} units_evicted={} max_unplaced={} max_blocker_branches={}",
            started.elapsed().as_millis(),
            self.stats.direct_fast_choices,
            self.stats.direct_fast_moves,
            self.stats.direct_fast_fallbacks,
            self.stats.augmenting_roots,
            self.stats.augmenting_fallbacks,
            self.stats.states_visited,
            self.failed.len(),
            self.stats.choices_examined,
            self.stats.units_evicted,
            self.stats.max_unplaced,
            self.stats.max_blocker_branches
        );
    }

    fn report_verbose_metrics(&self, event: &str) {
        if self.verbose_metrics {
            self.report_metrics(event);
        }
    }

    fn record_search_frame(&mut self) {
        self.stats.states_visited = self.stats.states_visited.saturating_add(1);
        self.stats.max_unplaced = self.stats.max_unplaced.max(self.state.unplaced_count);
        if self.stats.states_visited.is_power_of_two() {
            self.report_verbose_metrics("progress");
        }
    }

    fn solve(&mut self) -> bool {
        if self.state.next_unplaced().is_none() {
            return true;
        }
        if self.try_direct_empty_fast_path() {
            return true;
        }
        if self.try_single_blocker_fast_path() {
            return true;
        }
        self.solve_exact_iterative()
    }

    /// Greedily uses only assignments which are legal in the current partial
    /// placement.  Failure rolls back every move, so the later exact search
    /// sees precisely the original legal base.
    fn try_direct_empty_fast_path(&mut self) -> bool {
        let checkpoint = self.state.journal.len();
        while let Some(unit) = self.state.next_unplaced() {
            let mut choices =
                choice_cursor_for(self.spatial_indexes, &self.state, self.targets, unit);
            let mut installed = false;
            for choice in &mut choices {
                self.stats.direct_fast_choices = self.stats.direct_fast_choices.saturating_add(1);
                self.stats.choices_examined = self.stats.choices_examined.saturating_add(1);
                if self.stats.direct_fast_choices.is_power_of_two() {
                    self.report_verbose_metrics("direct-progress");
                }
                if self.state.can_install(unit, choice) {
                    self.state.install_for_branch(unit, choice);
                    self.stats.direct_fast_moves = self.stats.direct_fast_moves.saturating_add(1);
                    installed = true;
                    break;
                }
            }
            if installed {
                continue;
            }
            self.state.rollback(checkpoint);
            self.stats.direct_fast_fallbacks = self.stats.direct_fast_fallbacks.saturating_add(1);
            self.state.assert_consistent();
            self.report_verbose_metrics("direct-fallback");
            return false;
        }
        self.state.journal.truncate(checkpoint);
        self.state.assert_consistent();
        true
    }

    /// Polynomial alternating-path attempt for matching-like conflicts.
    ///
    /// Each candidate may displace at most one unit.  Unit and assignment
    /// visits are monotone within one root, so a cyclic preferred-placement
    /// walk cannot enumerate exponentially many complete matchings.  This is
    /// only a fast path: any failure rolls back all committed roots before the
    /// complete exact search starts.
    fn try_single_blocker_fast_path(&mut self) -> bool {
        let checkpoint = self.state.journal.len();
        while let Some(root) = self.state.next_unplaced() {
            self.stats.augmenting_roots = self.stats.augmenting_roots.saturating_add(1);
            if self.augment_one(root) {
                continue;
            }
            self.state.rollback(checkpoint);
            self.stats.augmenting_fallbacks = self.stats.augmenting_fallbacks.saturating_add(1);
            self.state.assert_consistent();
            self.report_verbose_metrics("augmenting-fallback");
            return false;
        }
        self.state.journal.truncate(checkpoint);
        self.state.assert_consistent();
        true
    }

    fn augment_one(&mut self, root: UnitId) -> bool {
        let root_checkpoint = self.state.journal.len();
        let mut visited_units = HashSet::from([root]);
        let mut visited_assignments = HashSet::<(UnitId, ChoiceId)>::new();
        self.record_search_frame();
        let mut stack = vec![AugmentFrame {
            unit: root,
            choices: choice_cursor_for(self.spatial_indexes, &self.state, self.targets, root),
            resume_checkpoint: None,
        }];

        while let Some(mut frame) = stack.pop() {
            if let Some(checkpoint) = frame.resume_checkpoint.take() {
                self.state.rollback(checkpoint);
            }
            let Some(choice) = frame.choices.next() else {
                continue;
            };
            if !visited_assignments.insert((frame.unit, choice)) {
                stack.push(frame);
                continue;
            }
            self.stats.choices_examined = self.stats.choices_examined.saturating_add(1);
            let footprint = assignment_footprint(
                self.state.graph,
                self.state.constraints,
                &self.state.units[frame.unit],
                choice,
            );
            let Some(mandatory) = self.mandatory_blockers(frame.unit, choice, &footprint) else {
                stack.push(frame);
                continue;
            };
            let branches = self.pin_blocker_branches(&footprint, mandatory);
            self.stats.max_blocker_branches = self.stats.max_blocker_branches.max(branches.len());
            if branches.len() != 1 || branches[0].len() > 1 {
                stack.push(frame);
                continue;
            }
            let blocker = branches[0].first().copied();
            if blocker.is_some_and(|blocker| visited_units.contains(&blocker)) {
                stack.push(frame);
                continue;
            }

            let checkpoint = self.state.journal.len();
            if let Some(blocker) = blocker {
                self.state.remove_for_branch(blocker);
                self.stats.units_evicted = self.stats.units_evicted.saturating_add(1);
            }
            if !self.state.can_install(frame.unit, choice) {
                self.state.rollback(checkpoint);
                stack.push(frame);
                continue;
            }
            self.state.install_for_branch(frame.unit, choice);
            let Some(blocker) = blocker else {
                return true;
            };

            visited_units.insert(blocker);
            self.record_search_frame();
            frame.resume_checkpoint = Some(checkpoint);
            stack.push(frame);
            stack.push(AugmentFrame {
                unit: blocker,
                choices: choice_cursor_for(
                    self.spatial_indexes,
                    &self.state,
                    self.targets,
                    blocker,
                ),
                resume_checkpoint: None,
            });
        }

        self.state.rollback(root_checkpoint);
        false
    }

    fn begin_exact_frame(&mut self) -> Option<ExactFrame<'a>> {
        let unit = self.state.next_unplaced()?;
        let key = self.state.key();
        if self.failed.contains(&key) || !self.visiting.insert(key.clone()) {
            return None;
        }
        self.record_search_frame();
        Some(ExactFrame {
            unit,
            key,
            choices: choice_cursor_for(self.spatial_indexes, &self.state, self.targets, unit),
            current_choice: None,
            branches: Vec::new(),
            next_branch: 0,
            resume_checkpoint: None,
        })
    }

    fn next_exact_attempt(
        &mut self,
        frame: &mut ExactFrame<'a>,
    ) -> Option<(ChoiceId, BTreeSet<UnitId>)> {
        loop {
            if frame.next_branch < frame.branches.len() {
                let blockers = frame.branches[frame.next_branch].clone();
                frame.next_branch += 1;
                return Some((
                    frame.current_choice.expect("a blocker branch has a choice"),
                    blockers,
                ));
            }
            let choice = frame.choices.next()?;
            self.stats.choices_examined = self.stats.choices_examined.saturating_add(1);
            let footprint = assignment_footprint(
                self.state.graph,
                self.state.constraints,
                &self.state.units[frame.unit],
                choice,
            );
            let Some(mandatory) = self.mandatory_blockers(frame.unit, choice, &footprint) else {
                continue;
            };
            let branches = self.pin_blocker_branches(&footprint, mandatory);
            self.stats.max_blocker_branches = self.stats.max_blocker_branches.max(branches.len());
            frame.current_choice = Some(choice);
            frame.branches = branches;
            frame.next_branch = 0;
        }
    }

    fn solve_exact_iterative(&mut self) -> bool {
        let Some(root) = self.begin_exact_frame() else {
            return self.state.next_unplaced().is_none();
        };
        let mut stack = vec![root];

        while let Some(mut frame) = stack.pop() {
            if let Some(checkpoint) = frame.resume_checkpoint.take() {
                self.state.rollback(checkpoint);
                self.state.assert_consistent();
            }
            let Some((choice, blockers)) = self.next_exact_attempt(&mut frame) else {
                self.visiting.remove(&frame.key);
                self.failed.insert(frame.key);
                if stack.is_empty() {
                    return false;
                }
                continue;
            };

            let checkpoint = self.state.journal.len();
            for blocker in blockers {
                if self.state.choice_by_unit[blocker].is_some() {
                    self.state.remove_for_branch(blocker);
                    self.stats.units_evicted = self.stats.units_evicted.saturating_add(1);
                }
            }
            if self.state.can_install(frame.unit, choice) {
                self.state.install_for_branch(frame.unit, choice);
                self.state.assert_consistent();
                if self.state.next_unplaced().is_none() {
                    self.visiting.clear();
                    return true;
                }
                if let Some(child) = self.begin_exact_frame() {
                    frame.resume_checkpoint = Some(checkpoint);
                    stack.push(frame);
                    stack.push(child);
                    continue;
                }
            }
            self.state.rollback(checkpoint);
            self.state.assert_consistent();
            stack.push(frame);
        }
        false
    }

    fn mandatory_blockers(
        &self,
        unit: UnitId,
        choice: ChoiceId,
        footprint: &AssignmentFootprint,
    ) -> Option<BTreeSet<UnitId>> {
        debug_assert_eq!(
            self.state.units[unit].choices.assignment(choice).len(),
            self.state.units[unit].cells.len()
        );
        let mut blockers = BTreeSet::new();
        for &bel in self.state.units[unit].choices.assignment(choice) {
            if let Some(owner) = self.state.owners.bels[bel.0] {
                if self.state.fixed[owner] {
                    return None;
                }
                blockers.insert(owner);
            }
        }

        let mut start = 0;
        while start < footprint.shared.len() {
            let resource = footprint.shared[start].0;
            let mut end = start + 1;
            while end < footprint.shared.len() && footprint.shared[end].0 == resource {
                end += 1;
            }
            if footprint.shared[start..end]
                .iter()
                .map(|(_, value)| *value)
                .collect::<BTreeSet<_>>()
                .len()
                > 1
            {
                return None;
            }
            let value = footprint.shared[start].1;
            if let Some(existing) = self.state.owners.shared.get(&resource) {
                for (&known, owners) in existing {
                    if known == value {
                        continue;
                    }
                    for &owner in owners {
                        if self.state.fixed[owner] {
                            return None;
                        }
                        blockers.insert(owner);
                    }
                }
            }
            start = end;
        }
        Some(blockers)
    }

    /// Enumerates exact distinct-net-class evictions for every touched wire.
    fn pin_blocker_branches(
        &self,
        footprint: &AssignmentFootprint,
        mandatory: BTreeSet<UnitId>,
    ) -> Vec<BTreeSet<UnitId>> {
        let mut candidate_by_wire = BTreeMap::<WireId, BTreeSet<NetId>>::new();
        for &(wire, net) in &footprint.pin_wires {
            candidate_by_wire.entry(wire).or_default().insert(net);
        }
        let mut branches = vec![mandatory];
        for (wire, candidate_nets) in candidate_by_wire {
            let capacity = usize::from(self.state.graph.device().wires()[wire.0].capacity);
            if candidate_nets.len() > capacity {
                return Vec::new();
            }
            let mut next = Vec::new();
            for blockers in branches {
                let existing = self.state.owners.pin_wires.get(&wire);
                let mut effective = Vec::<(NetId, Vec<UnitId>)>::new();
                if let Some(existing) = existing {
                    for (&net, owners) in existing {
                        let remaining = owners
                            .iter()
                            .copied()
                            .filter(|owner| !blockers.contains(owner))
                            .collect::<Vec<_>>();
                        if !remaining.is_empty() {
                            effective.push((net, remaining));
                        }
                    }
                }
                let distinct = effective
                    .iter()
                    .map(|(net, _)| *net)
                    .chain(candidate_nets.iter().copied())
                    .collect::<BTreeSet<_>>()
                    .len();
                let required = distinct.saturating_sub(capacity);
                if required == 0 {
                    next.push(blockers);
                    continue;
                }
                let removable = effective
                    .into_iter()
                    .filter(|(net, _)| !candidate_nets.contains(net))
                    .filter(|(_, owners)| owners.iter().all(|&owner| !self.state.fixed[owner]))
                    .collect::<Vec<_>>();
                if removable.len() < required {
                    continue;
                }
                for combination in enumerate_combinations(removable.len(), required) {
                    let mut branch = blockers.clone();
                    for index in combination {
                        branch.extend(removable[index].1.iter().copied());
                    }
                    next.push(branch);
                }
            }
            next.sort_unstable();
            next.dedup();
            branches = next;
            if branches.is_empty() {
                break;
            }
        }
        branches
    }
}

fn enumerate_combinations(item_count: usize, choose: usize) -> Vec<Vec<usize>> {
    if choose > item_count {
        return Vec::new();
    }
    if choose == 0 {
        return vec![Vec::new()];
    }

    let mut result = Vec::new();
    let mut current = (0..choose).collect::<Vec<_>>();
    loop {
        result.push(current.clone());
        let Some(pivot) = (0..choose)
            .rev()
            .find(|&index| current[index] < item_count - choose + index)
        else {
            break;
        };
        current[pivot] += 1;
        for index in pivot + 1..choose {
            current[index] = current[index - 1] + 1;
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use std::collections::{BTreeMap, BTreeSet, HashSet};
    use std::sync::Arc;

    use texo_model::{BelId, Design, Device, PinDirection, Point, ResourceKind, UnifiedGraph};

    use super::super::super::{
        PlacementChoices, PlacementConstraints, PlacementUnit, SpatialChoiceIndex, placement_units,
    };
    use super::{
        ChoiceCursor, RepairSearch, RepairState, RepairStats, assignment_footprint,
        enumerate_combinations, initial_pending_units, repair_relaxed,
    };

    type UnitProblem = (
        Vec<super::super::super::PlacementUnit>,
        BTreeMap<(u8, usize), Arc<SpatialChoiceIndex>>,
    );

    fn units_and_indexes(
        graph: &UnifiedGraph<'_>,
        constraints: &PlacementConstraints,
    ) -> UnitProblem {
        let mut cache = BTreeMap::new();
        let units = placement_units(graph, constraints, &mut cache).unwrap();
        let mut indexes = BTreeMap::new();
        for unit in &units {
            indexes.entry(unit.choices.cache_key()).or_insert_with(|| {
                Arc::new(SpatialChoiceIndex::new(&unit.choices, graph.device()))
            });
        }
        (units, indexes)
    }

    fn add_logic_bels(device: &mut Device, count: usize) -> Vec<BelId> {
        (0..count)
            .map(|index| {
                device
                    .add_bel(
                        format!("bel{index}"),
                        ResourceKind::Logic,
                        Point::new(u32::try_from(index).unwrap(), 0),
                    )
                    .unwrap()
            })
            .collect()
    }

    #[test]
    fn choice_cursor_is_preferred_first_then_point_stable_and_exhaustive() {
        let mut device = Device::new("cursor-order", 3, 3).unwrap();
        let bels = [
            Point::new(2, 1),
            Point::new(0, 1),
            Point::new(1, 2),
            Point::new(1, 1),
            Point::new(1, 0),
        ]
        .map(|point| {
            device
                .add_bel(
                    format!("bel-{}-{}", point.x, point.y),
                    ResourceKind::Logic,
                    point,
                )
                .unwrap()
        });
        let choices = PlacementChoices::SingleCell(Arc::from(bels));
        let spatial = SpatialChoiceIndex::new(&choices, &device);

        let order =
            ChoiceCursor::new(&spatial, 3, 3, Point::new(1, 1), Some(0)).collect::<Vec<_>>();

        assert_eq!(order, [0, 3, 1, 4, 2]);
    }

    #[test]
    fn twelve_thousand_unit_augmenting_chain_uses_explicit_frames() {
        const CHAIN_LEN: usize = 12_000;

        let mut design = Design::new();
        let cells = (0..CHAIN_LEN)
            .map(|index| design.add_cell(format!("cell-{index}"), ResourceKind::Logic))
            .collect::<Vec<_>>();
        let mut device = Device::new("long-augmenting-chain", 1, 1).unwrap();
        let bels = (0..CHAIN_LEN)
            .map(|index| {
                device
                    .add_bel(
                        format!("bel-{index}"),
                        ResourceKind::Logic,
                        Point::new(0, 0),
                    )
                    .unwrap()
            })
            .collect::<Vec<_>>();

        let mut units = Vec::with_capacity(CHAIN_LEN);
        let mut indexes = BTreeMap::new();
        for unit in 0..CHAIN_LEN {
            let candidates: Arc<[BelId]> = if unit == 0 {
                Arc::from([bels[0]])
            } else {
                Arc::from([bels[unit - 1], bels[unit]])
            };
            let choices = PlacementChoices::SingleCell(candidates);
            indexes.insert(
                choices.cache_key(),
                Arc::new(SpatialChoiceIndex::new(&choices, &device)),
            );
            units.push(PlacementUnit {
                cells: vec![cells[unit]],
                choices,
            });
        }
        let graph = UnifiedGraph::new(&design, &device);
        let mut preferred = vec![Some(0); CHAIN_LEN];
        preferred[0] = None;

        let (placed, stats) = repair_relaxed(
            &graph,
            &PlacementConstraints::new(),
            &units,
            &indexes,
            &vec![Point::new(0, 0); CHAIN_LEN],
            &preferred,
            &vec![false; CHAIN_LEN],
        )
        .unwrap();

        for (unit, bel) in bels.into_iter().enumerate() {
            assert_eq!(placed[cells[unit].0], Some(bel));
        }
        assert_eq!(stats.direct_fast_fallbacks, 1);
        assert_eq!(stats.augmenting_roots, 1);
        assert_eq!(stats.augmenting_fallbacks, 0);
        assert!(stats.states_visited >= u64::try_from(CHAIN_LEN).unwrap());
    }

    #[test]
    fn deep_single_combination_is_enumerated_without_recursion() {
        const CHOOSE: usize = 12_000;
        let combinations = enumerate_combinations(CHOOSE, CHOOSE);

        assert_eq!(combinations.len(), 1);
        assert_eq!(combinations[0].len(), CHOOSE);
        assert_eq!(combinations[0][0], 0);
        assert_eq!(combinations[0][CHOOSE - 1], CHOOSE - 1);
    }

    #[test]
    fn multi_blocker_candidate_falls_back_to_exact_iterative_frames() {
        let mut design = Design::new();
        let bundle_cells = [
            design.add_cell("bundle-a", ResourceKind::Logic),
            design.add_cell("bundle-b", ResourceKind::Logic),
        ];
        let first = design.add_cell("first", ResourceKind::Logic);
        let second = design.add_cell("second", ResourceKind::Logic);
        let mut device = Device::new("exact-multi-blocker", 4, 1).unwrap();
        let bels = add_logic_bels(&mut device, 4);
        let graph = UnifiedGraph::new(&design, &device);

        let bundle_choices = PlacementChoices::Shared(Arc::from([vec![bels[0], bels[1]]]));
        let first_choices = PlacementChoices::SingleCell(Arc::from([bels[0], bels[2]]));
        let second_choices = PlacementChoices::SingleCell(Arc::from([bels[1], bels[3]]));
        let units = vec![
            PlacementUnit {
                cells: bundle_cells.to_vec(),
                choices: bundle_choices,
            },
            PlacementUnit {
                cells: vec![first],
                choices: first_choices,
            },
            PlacementUnit {
                cells: vec![second],
                choices: second_choices,
            },
        ];
        let mut indexes = BTreeMap::new();
        for unit in &units {
            indexes.insert(
                unit.choices.cache_key(),
                Arc::new(SpatialChoiceIndex::new(&unit.choices, &device)),
            );
        }

        let (placed, stats) = repair_relaxed(
            &graph,
            &PlacementConstraints::new(),
            &units,
            &indexes,
            &[Point::new(0, 0); 3],
            &[None, Some(0), Some(0)],
            &[false; 3],
        )
        .unwrap();

        assert_eq!(placed[bundle_cells[0].0], Some(bels[0]));
        assert_eq!(placed[bundle_cells[1].0], Some(bels[1]));
        assert_eq!(placed[first.0], Some(bels[2]));
        assert_eq!(placed[second.0], Some(bels[3]));
        assert_eq!(stats.direct_fast_fallbacks, 1);
        assert_eq!(stats.augmenting_fallbacks, 1);
        assert!(stats.states_visited >= 3);
    }

    #[test]
    fn partial_direct_fast_path_rolls_back_before_augmenting() {
        let mut design = Design::new();
        let cells = (0..4)
            .map(|index| design.add_cell(format!("cell-{index}"), ResourceKind::Logic))
            .collect::<Vec<_>>();
        let mut device = Device::new("direct-rollback", 1, 1).unwrap();
        let bels = (0..4)
            .map(|index| {
                device
                    .add_bel(
                        format!("bel-{index}"),
                        ResourceKind::Logic,
                        Point::new(0, 0),
                    )
                    .unwrap()
            })
            .collect::<Vec<_>>();
        let domains: [Arc<[BelId]>; 4] = [
            Arc::from([bels[3]]),
            Arc::from([bels[0], bels[1]]),
            Arc::from([bels[0], bels[2]]),
            Arc::from([bels[1]]),
        ];
        let units = cells
            .iter()
            .copied()
            .zip(domains)
            .map(|(cell, candidates)| PlacementUnit {
                cells: vec![cell],
                choices: PlacementChoices::SingleCell(candidates),
            })
            .collect::<Vec<_>>();
        let mut indexes = BTreeMap::new();
        for unit in &units {
            indexes.insert(
                unit.choices.cache_key(),
                Arc::new(SpatialChoiceIndex::new(&unit.choices, &device)),
            );
        }
        let graph = UnifiedGraph::new(&design, &device);

        let (placed, stats) = repair_relaxed(
            &graph,
            &PlacementConstraints::new(),
            &units,
            &indexes,
            &[Point::new(0, 0); 4],
            &[None, None, Some(0), Some(0)],
            &[false, false, false, true],
        )
        .unwrap();

        assert_eq!(placed[cells[0].0], Some(bels[3]));
        assert_eq!(placed[cells[1].0], Some(bels[0]));
        assert_eq!(placed[cells[2].0], Some(bels[2]));
        assert_eq!(placed[cells[3].0], Some(bels[1]));
        assert_eq!(stats.direct_fast_moves, 1);
        assert_eq!(stats.direct_fast_fallbacks, 1);
        assert_eq!(stats.augmenting_roots, 2);
        assert_eq!(stats.augmenting_fallbacks, 0);
    }

    #[test]
    fn far_spill_displaces_a_clean_bel_owner_instead_of_returning_no_bel() {
        let mut design = Design::new();
        let wanted = design.add_cell("wanted", ResourceKind::Logic);
        let flexible = design.add_cell("flexible", ResourceKind::Logic);
        let fixed_cell = design.add_cell("fixed", ResourceKind::Logic);
        let mut device = Device::new("far-spill", 65, 1).unwrap();
        let bels = add_logic_bels(&mut device, 65);
        let mut constraints = PlacementConstraints::new();
        constraints.add_group([wanted], [vec![bels[0]], vec![bels[1]]]);
        constraints.add_group([flexible], [vec![bels[0]], vec![bels[64]]]);
        constraints.add_group([fixed_cell], [vec![bels[1]]]);
        let graph = UnifiedGraph::new(&design, &device);
        let (units, indexes) = units_and_indexes(&graph, &constraints);
        let targets = vec![Point::new(0, 0); units.len()];

        let (placed, _) = repair_relaxed(
            &graph,
            &constraints,
            &units,
            &indexes,
            &targets,
            &[None, Some(0), Some(0)],
            &[false, false, true],
        )
        .unwrap();

        assert_eq!(placed[wanted.0], Some(bels[0]));
        assert_eq!(placed[flexible.0], Some(bels[64]));
        assert_eq!(placed[fixed_cell.0], Some(bels[1]));
    }

    #[test]
    fn overlapping_bundle_rows_are_repaired_atomically() {
        let mut design = Design::new();
        let cells = (0..4)
            .map(|index| design.add_cell(format!("cell{index}"), ResourceKind::Logic))
            .collect::<Vec<_>>();
        let mut device = Device::new("overlapping-bundles", 4, 1).unwrap();
        let bels = add_logic_bels(&mut device, 4);
        let rows: Arc<[Vec<BelId>]> = Arc::from([
            vec![bels[0], bels[1]],
            vec![bels[1], bels[2]],
            vec![bels[2], bels[3]],
        ]);
        let mut constraints = PlacementConstraints::new();
        constraints.add_group_with_shared_assignments([cells[0], cells[1]], Arc::clone(&rows));
        constraints.add_group_with_shared_assignments([cells[2], cells[3]], rows);
        let graph = UnifiedGraph::new(&design, &device);
        let (units, indexes) = units_and_indexes(&graph, &constraints);

        let (placed, _) = repair_relaxed(
            &graph,
            &constraints,
            &units,
            &indexes,
            &[Point::new(0, 0), Point::new(1, 0)],
            &[Some(0), Some(1)],
            &[false, false],
        )
        .unwrap();

        assert_eq!(placed.iter().flatten().collect::<BTreeSet<_>>().len(), 4);
        for unit in &units {
            let assignment = unit
                .cells
                .iter()
                .map(|cell| placed[cell.0].unwrap())
                .collect::<Vec<_>>();
            assert!(unit.choices.contains(&assignment));
        }
    }

    #[test]
    fn incompatible_shared_values_move_to_different_resources() {
        let mut design = Design::new();
        let zero = design.add_cell("zero", ResourceKind::Logic);
        let one = design.add_cell("one", ResourceKind::Logic);
        let mut device = Device::new("shared-values", 4, 1).unwrap();
        let bels = add_logic_bels(&mut device, 4);
        let mut constraints = PlacementConstraints::new();
        constraints.add_group([zero], [vec![bels[0]], vec![bels[2]]]);
        constraints.add_group([one], [vec![bels[1]], vec![bels[3]]]);
        constraints.add_shared_resource(
            [(zero, 0), (one, 1)],
            [(bels[0], 0), (bels[1], 0), (bels[2], 1), (bels[3], 1)],
        );
        let graph = UnifiedGraph::new(&design, &device);
        let (units, indexes) = units_and_indexes(&graph, &constraints);

        let (placed, stats) = repair_relaxed(
            &graph,
            &constraints,
            &units,
            &indexes,
            &[Point::new(0, 0); 2],
            &[Some(0), Some(0)],
            &[false, false],
        )
        .unwrap();

        assert_ne!(placed[zero.0].unwrap().0 / 2, placed[one.0].unwrap().0 / 2);
        assert_eq!(stats.initial_pending, 1);
    }

    #[test]
    fn initial_pending_keeps_a_fixed_net_class_and_fills_remaining_capacity() {
        let mut design = Design::new();
        let fixed_cell = design.add_cell("fixed", ResourceKind::Logic);
        let fixed_pin = design
            .add_pin(fixed_cell, "P", PinDirection::Output)
            .unwrap();
        let same_class = design.add_cell("same-class", ResourceKind::Logic);
        let same_pin = design
            .add_pin(same_class, "P", PinDirection::Input)
            .unwrap();
        design
            .add_net("fixed-class", fixed_pin, [same_pin])
            .unwrap();

        let class_b = design.add_cell("class-b", ResourceKind::Logic);
        {
            let output = design.add_pin(class_b, "P", PinDirection::Output).unwrap();
            let input = design.add_pin(class_b, "Q", PinDirection::Input).unwrap();
            design.add_net("class-b", output, [input]).unwrap();
        }

        let class_c = design.add_cell("class-c", ResourceKind::Logic);
        {
            let output = design.add_pin(class_c, "P", PinDirection::Output).unwrap();
            let input = design.add_pin(class_c, "Q", PinDirection::Input).unwrap();
            design.add_net("class-c", output, [input]).unwrap();
        }

        let mut device = Device::new("fixed-pin-class", 1, 1).unwrap();
        let wire = device.add_wire("shared", Point::new(0, 0), 2).unwrap();
        let mut bels = Vec::new();
        for (name, pins) in [
            ("fixed", &[("P", PinDirection::Output)][..]),
            ("same", &[("P", PinDirection::Input)][..]),
            (
                "class-b",
                &[("P", PinDirection::Output), ("Q", PinDirection::Input)][..],
            ),
            (
                "class-c",
                &[("P", PinDirection::Output), ("Q", PinDirection::Input)][..],
            ),
        ] {
            let bel = device
                .add_bel(name, ResourceKind::Logic, Point::new(0, 0))
                .unwrap();
            for &(pin, direction) in pins {
                device.add_bel_pin(bel, pin, direction, wire).unwrap();
            }
            bels.push(bel);
        }

        let mut constraints = PlacementConstraints::new();
        for (cell, bel) in [fixed_cell, same_class, class_b, class_c]
            .into_iter()
            .zip(bels)
        {
            constraints.add_group([cell], [vec![bel]]);
        }
        let graph = UnifiedGraph::new(&design, &device);
        let (units, _) = units_and_indexes(&graph, &constraints);

        let pending = initial_pending_units(
            &graph,
            &constraints,
            &units,
            &[Some(0); 4],
            &[true, false, false, false],
        )
        .unwrap();

        assert_eq!(pending, BTreeSet::from([3]));
    }

    #[test]
    fn unavoidable_shared_value_conflict_exhausts_finite_states() {
        let mut design = Design::new();
        let zero = design.add_cell("zero", ResourceKind::Logic);
        let one = design.add_cell("one", ResourceKind::Logic);
        let mut device = Device::new("unavoidable-shared", 4, 1).unwrap();
        let bels = add_logic_bels(&mut device, 4);
        let mut constraints = PlacementConstraints::new();
        constraints.add_group([zero], [vec![bels[0]], vec![bels[1]]]);
        constraints.add_group([one], [vec![bels[2]], vec![bels[3]]]);
        constraints.add_shared_resource(
            [(zero, 0), (one, 1)],
            bels.iter().copied().map(|bel| (bel, 0)),
        );
        let graph = UnifiedGraph::new(&design, &device);
        let (units, indexes) = units_and_indexes(&graph, &constraints);

        let result = repair_relaxed(
            &graph,
            &constraints,
            &units,
            &indexes,
            &[Point::new(0, 0); 2],
            &[Some(0), Some(0)],
            &[false, false],
        );

        assert!(matches!(
            result,
            Err(super::super::super::PnrError::NoBel { .. })
        ));
    }

    #[test]
    fn pin_class_remains_until_its_last_owner_is_blocked_and_rollback_is_exact() {
        let mut design = Design::new();
        let source = design.add_cell("source", ResourceKind::Logic);
        let source_pin = design.add_pin(source, "P", PinDirection::Output).unwrap();
        let sink = design.add_cell("sink", ResourceKind::Logic);
        let sink_pin = design.add_pin(sink, "P", PinDirection::Input).unwrap();
        design.add_net("a", source_pin, [sink_pin]).unwrap();
        let candidate = design.add_cell("candidate", ResourceKind::Logic);
        let candidate_pin = design
            .add_pin(candidate, "P", PinDirection::Output)
            .unwrap();
        let candidate_sink = design.add_pin(candidate, "Q", PinDirection::Input).unwrap();
        design
            .add_net("b", candidate_pin, [candidate_sink])
            .unwrap();

        let mut device = Device::new("pin-owner", 3, 1).unwrap();
        let shared = device.add_wire("shared", Point::new(0, 0), 1).unwrap();
        let alternate_a = device.add_wire("alternate-a", Point::new(1, 0), 1).unwrap();
        let alternate_b = device.add_wire("alternate-b", Point::new(2, 0), 1).unwrap();
        let source_bel = device
            .add_bel("source", ResourceKind::Logic, Point::new(0, 0))
            .unwrap();
        device
            .add_bel_pin(source_bel, "P", PinDirection::Output, shared)
            .unwrap();
        let sink_bel = device
            .add_bel("sink", ResourceKind::Logic, Point::new(0, 0))
            .unwrap();
        device
            .add_bel_pin(sink_bel, "P", PinDirection::Input, shared)
            .unwrap();
        let candidate_bel = device
            .add_bel("candidate", ResourceKind::Logic, Point::new(0, 0))
            .unwrap();
        device
            .add_bel_pin(candidate_bel, "P", PinDirection::Output, shared)
            .unwrap();
        device
            .add_bel_pin(candidate_bel, "Q", PinDirection::Input, shared)
            .unwrap();
        let source_alt = device
            .add_bel("source-alt", ResourceKind::Logic, Point::new(1, 0))
            .unwrap();
        device
            .add_bel_pin(source_alt, "P", PinDirection::Output, alternate_a)
            .unwrap();
        let sink_alt = device
            .add_bel("sink-alt", ResourceKind::Logic, Point::new(2, 0))
            .unwrap();
        device
            .add_bel_pin(sink_alt, "P", PinDirection::Input, alternate_b)
            .unwrap();

        let mut constraints = PlacementConstraints::new();
        constraints.add_group([source], [vec![source_bel], vec![source_alt]]);
        constraints.add_group([sink], [vec![sink_bel], vec![sink_alt]]);
        constraints.add_group([candidate], [vec![candidate_bel]]);
        let graph = UnifiedGraph::new(&design, &device);
        let (units, indexes) = units_and_indexes(&graph, &constraints);
        let preferred = [Some(0), Some(0), None];
        let fixed = [false, false, false];
        let mut state = RepairState::new(&graph, &constraints, &units, &preferred, &fixed);
        state.raw_install(0, 0);
        state.raw_install(1, 0);
        let targets = [Point::new(0, 0); 3];
        let search = RepairSearch {
            state,
            spatial_indexes: &indexes,
            targets: &targets,
            visiting: HashSet::default(),
            failed: HashSet::default(),
            metrics_started: None,
            verbose_metrics: false,
            stats: RepairStats::default(),
        };
        let footprint = assignment_footprint(&graph, &constraints, &units[2], 0);
        let branches = search.pin_blocker_branches(&footprint, BTreeSet::from([0]));

        assert_eq!(branches, [BTreeSet::from([0, 1])]);

        let mut state = search.state;
        let before = state.key();
        assert!(before.is_empty());
        let checkpoint = state.journal.len();
        state.remove_for_branch(0);
        state.remove_for_branch(1);
        state.install_for_branch(2, 0);
        assert!(!state.key().is_empty());
        state.rollback(checkpoint);
        state.assert_consistent();
        assert_eq!(state.key(), before);
        assert_eq!(state.choice_by_unit, [Some(0), Some(0), None]);
    }

    #[test]
    fn candidate_distinct_nets_share_the_candidate_wire_capacity() {
        let mut design = Design::new();
        let incumbent = design.add_cell("incumbent", ResourceKind::Logic);
        let incumbent_pin = design
            .add_pin(incumbent, "A", PinDirection::Output)
            .unwrap();
        let incumbent_sink = design
            .add_pin(incumbent, "AI", PinDirection::Input)
            .unwrap();
        design
            .add_net("a", incumbent_pin, [incumbent_sink])
            .unwrap();
        let first = design.add_cell("first", ResourceKind::Logic);
        let first_pin = design.add_pin(first, "C", PinDirection::Output).unwrap();
        let first_sink = design.add_pin(first, "CI", PinDirection::Input).unwrap();
        design.add_net("c", first_pin, [first_sink]).unwrap();
        let second = design.add_cell("second", ResourceKind::Logic);
        let second_pin = design.add_pin(second, "D", PinDirection::Output).unwrap();
        let second_sink = design.add_pin(second, "DI", PinDirection::Input).unwrap();
        design.add_net("d", second_pin, [second_sink]).unwrap();

        let mut device = Device::new("candidate-classes", 1, 1).unwrap();
        let wire = device.add_wire("shared", Point::new(0, 0), 2).unwrap();
        let incumbent_bel = device
            .add_bel("incumbent", ResourceKind::Logic, Point::new(0, 0))
            .unwrap();
        device
            .add_bel_pin(incumbent_bel, "A", PinDirection::Output, wire)
            .unwrap();
        device
            .add_bel_pin(incumbent_bel, "AI", PinDirection::Input, wire)
            .unwrap();
        let first_bel = device
            .add_bel("first", ResourceKind::Logic, Point::new(0, 0))
            .unwrap();
        device
            .add_bel_pin(first_bel, "C", PinDirection::Output, wire)
            .unwrap();
        device
            .add_bel_pin(first_bel, "CI", PinDirection::Input, wire)
            .unwrap();
        let second_bel = device
            .add_bel("second", ResourceKind::Logic, Point::new(0, 0))
            .unwrap();
        device
            .add_bel_pin(second_bel, "D", PinDirection::Output, wire)
            .unwrap();
        device
            .add_bel_pin(second_bel, "DI", PinDirection::Input, wire)
            .unwrap();
        let mut constraints = PlacementConstraints::new();
        constraints.add_group([incumbent], [vec![incumbent_bel]]);
        constraints.add_group([first, second], [vec![first_bel, second_bel]]);
        let graph = UnifiedGraph::new(&design, &device);
        let (units, indexes) = units_and_indexes(&graph, &constraints);
        let preferred = [Some(0), None];
        let fixed = [false, false];
        let mut state = RepairState::new(&graph, &constraints, &units, &preferred, &fixed);
        state.raw_install(0, 0);
        let targets = [Point::new(0, 0); 2];
        let search = RepairSearch {
            state,
            spatial_indexes: &indexes,
            targets: &targets,
            visiting: HashSet::default(),
            failed: HashSet::default(),
            metrics_started: None,
            verbose_metrics: false,
            stats: RepairStats::default(),
        };
        let footprint = assignment_footprint(&graph, &constraints, &units[1], 0);

        assert_eq!(
            search.pin_blocker_branches(&footprint, BTreeSet::new()),
            [BTreeSet::from([0])]
        );
    }
}
