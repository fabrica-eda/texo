//! Deterministic reference placement and routing on the unified problem graph.

use std::cmp::Reverse;
use std::collections::{BTreeMap, BTreeSet, BinaryHeap};
use std::error::Error;
use std::fmt;
use std::sync::Arc;

use texo_model::{
    BelId, BelPinId, CellId, CellPinId, Design, Device, ModelError, NetId, PipId, Point,
    UnifiedGraph, WireId,
};

/// Cell-to-BEL bindings indexed by stable cell ID.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Placement {
    bindings: Vec<BelId>,
    pin_bindings: BTreeMap<CellPinId, BelPinId>,
}

/// One atomically placed group and its legal BEL assignments.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PlacementGroup {
    /// Cells assigned together, in assignment-column order.
    pub cells: Vec<CellId>,
    /// Legal assignments; every row must have one BEL per cell.
    pub assignments: Arc<[Vec<BelId>]>,
}

/// Optional grouped/fixed placement rules supplied by a target packer.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct PlacementConstraints {
    groups: Vec<PlacementGroup>,
    pin_bindings: BTreeMap<(CellPinId, BelId), BelPinId>,
    pin_name_bindings: BTreeMap<CellPinId, String>,
}

impl PlacementConstraints {
    /// Creates an unconstrained placement problem.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            groups: Vec::new(),
            pin_bindings: BTreeMap::new(),
            pin_name_bindings: BTreeMap::new(),
        }
    }

    /// Adds one atomic group. Structural and compatibility checks run before
    /// placement because they require the complete design and device.
    pub fn add_group(
        &mut self,
        cells: impl IntoIterator<Item = CellId>,
        assignments: impl IntoIterator<Item = Vec<BelId>>,
    ) {
        self.groups.push(PlacementGroup {
            cells: cells.into_iter().collect(),
            assignments: assignments.into_iter().collect::<Vec<_>>().into(),
        });
    }

    /// Adds an atomic group backed by a shared legal-assignment table.
    ///
    /// Targets use this when many logical groups implement the same physical
    /// relationship, such as every ECP5 LUT/FF pair. Cloning the [`Arc`] does
    /// not duplicate the device-wide table.
    pub fn add_group_with_shared_assignments(
        &mut self,
        cells: impl IntoIterator<Item = CellId>,
        assignments: Arc<[Vec<BelId>]>,
    ) {
        self.groups.push(PlacementGroup {
            cells: cells.into_iter().collect(),
            assignments,
        });
    }

    /// Target-supplied atomic groups in insertion order.
    #[must_use]
    pub fn groups(&self) -> &[PlacementGroup] {
        &self.groups
    }

    /// Overrides one logical pin's physical pin for a particular BEL choice.
    ///
    /// This models target packing transformations such as an ECP5 FF data
    /// input using `DI` when paired with a LUT and `M` when independently
    /// routed. Validation occurs before placement.
    pub fn bind_pin(&mut self, pin: CellPinId, bel: BelId, bel_pin: BelPinId) {
        self.pin_bindings.insert((pin, bel), bel_pin);
    }

    /// Candidate-specific pin binding overrides.
    #[must_use]
    pub const fn pin_bindings(&self) -> &BTreeMap<(CellPinId, BelId), BelPinId> {
        &self.pin_bindings
    }

    /// Overrides one logical pin by physical pin name for every BEL choice.
    ///
    /// The concrete BEL pin is resolved after placement. This avoids expanding
    /// a target rule that applies uniformly to every compatible BEL into one
    /// binding per candidate.
    pub fn bind_pin_name(&mut self, pin: CellPinId, physical_name: impl Into<String>) {
        self.pin_name_bindings.insert(pin, physical_name.into());
    }

    /// BEL-independent physical pin-name overrides.
    #[must_use]
    pub const fn pin_name_bindings(&self) -> &BTreeMap<CellPinId, String> {
        &self.pin_name_bindings
    }
}

impl Placement {
    /// BEL assigned to a cell, if the cell ID exists.
    #[must_use]
    pub fn bel(&self, cell: CellId) -> Option<BelId> {
        self.bindings.get(cell.0).copied()
    }

    /// Cell-to-BEL bindings in stable cell ID order.
    #[must_use]
    pub fn bindings(&self) -> &[BelId] {
        &self.bindings
    }

    /// Target-selected physical pin override, when packing changed the
    /// logical-to-physical port name.
    #[must_use]
    pub fn pin_binding(&self, pin: CellPinId) -> Option<BelPinId> {
        self.pin_bindings.get(&pin).copied()
    }

    /// Physical point assigned to a cell.
    #[must_use]
    pub fn point(&self, cell: CellId, device: &Device) -> Option<Point> {
        self.bel(cell)
            .and_then(|bel| device.bels().get(bel.0))
            .map(|bel| bel.point)
    }
}

/// Routed tree for one logical net.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NetRoute {
    /// Logical net represented by this tree.
    pub net: NetId,
    /// Occupied routing wires in stable ID order.
    pub wires: Vec<WireId>,
    /// Enabled programmable interconnect points in stable ID order.
    pub pips: Vec<PipId>,
}

/// Complete result of the reference `PnR` engine.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PnrResult {
    /// Legal cell-to-BEL assignment.
    pub placement: Placement,
    /// One physical route tree per logical net.
    pub routes: Vec<NetRoute>,
    /// Number of unique PIPs used across all net trees.
    pub total_pips: usize,
}

/// Places and routes a design on a typed unified graph.
///
/// The placer traverses lazily generated `Cell → BEL` candidates and orders
/// cells by logical connectivity. The router resolves each placed cell pin to
/// `BEL pin → Wire`, then grows a tree through directed `Wire → PIP → Wire`
/// edges without exceeding resource capacity.
///
/// # Errors
///
/// Returns a descriptive model, legality, or routability error.
pub fn place_and_route(design: &Design, device: &Device) -> Result<PnrResult, PnrError> {
    place_and_route_with_constraints(design, device, &PlacementConstraints::new())
}

/// Places and routes with target-supplied atomic placement groups.
///
/// # Errors
///
/// Returns a descriptive constraint, model, legality, or routability error.
pub fn place_and_route_with_constraints(
    design: &Design,
    device: &Device,
    constraints: &PlacementConstraints,
) -> Result<PnrResult, PnrError> {
    let graph = UnifiedGraph::new(design, device);
    let placement = place(&graph, constraints)?;
    let routes = route(&graph, &placement)?;
    let total_pips = routes.iter().map(|route| route.pips.len()).sum();
    Ok(PnrResult {
        placement,
        routes,
        total_pips,
    })
}

/// Places a design without routing it.
///
/// # Errors
///
/// Returns a descriptive constraint, model, or BEL-exhaustion error.
pub fn place_with_constraints(
    design: &Design,
    device: &Device,
    constraints: &PlacementConstraints,
) -> Result<Placement, PnrError> {
    place(&UnifiedGraph::new(design, device), constraints)
}

#[derive(Clone, Debug)]
struct PlacementUnit {
    cells: Vec<CellId>,
    choices: PlacementChoices,
}

#[derive(Clone, Debug)]
enum PlacementChoices {
    Shared(Arc<[Vec<BelId>]>),
    SingleCell(Arc<[BelId]>),
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct PlacementCandidateKey {
    kind: texo_model::ResourceKind,
    pins: Vec<(String, texo_model::PinDirection)>,
}

fn place(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
) -> Result<Placement, PnrError> {
    let design = graph.design();
    let device = graph.device();
    let mut degree = vec![0_usize; design.cells().len()];
    let mut neighbors = vec![Vec::new(); design.cells().len()];
    for net in design.nets() {
        let driver = design.pins()[net.driver.0].cell;
        for sink in &net.sinks {
            let sink = design.pins()[sink.0].cell;
            if driver != sink {
                degree[driver.0] += 1;
                degree[sink.0] += 1;
                neighbors[driver.0].push(sink);
                neighbors[sink.0].push(driver);
            }
        }
    }

    let mut candidate_cache = BTreeMap::new();
    let mut units = placement_units(graph, constraints, &mut candidate_cache)?;
    units.sort_by_key(|unit| {
        (
            std::cmp::Reverse(unit.cells.iter().map(|cell| degree[cell.0]).sum::<usize>()),
            unit.cells[0],
        )
    });

    let mut placed = vec![None; design.cells().len()];
    let mut occupied = BTreeSet::new();
    for unit in units {
        let choice = match &unit.choices {
            PlacementChoices::Shared(assignments) => choose_assignment(
                &unit.cells,
                assignments.iter().map(Vec::as_slice),
                device,
                &neighbors,
                &placed,
                &occupied,
            ),
            PlacementChoices::SingleCell(candidates) => choose_assignment(
                &unit.cells,
                candidates.iter().map(std::slice::from_ref),
                device,
                &neighbors,
                &placed,
                &occupied,
            ),
        };
        let assignment = choice.ok_or_else(|| PnrError::NoBel {
            cell: design.cells()[unit.cells[0].0].name.clone(),
        })?;
        for (cell, bel) in unit.cells.into_iter().zip(assignment) {
            occupied.insert(bel);
            placed[cell.0] = Some(bel);
        }
    }

    let bindings = placed
        .into_iter()
        .map(|bel| bel.expect("every ordered cell was placed"))
        .collect::<Vec<_>>();
    let mut pin_bindings = constraints
        .pin_bindings
        .iter()
        .filter(|((pin, bel), _)| bindings[design.pins()[pin.0].cell.0] == *bel)
        .map(|((pin, _), bel_pin)| (*pin, *bel_pin))
        .collect::<BTreeMap<_, _>>();
    for (&pin, physical_name) in &constraints.pin_name_bindings {
        if pin_bindings.contains_key(&pin) {
            continue;
        }
        let bel = bindings[design.pins()[pin.0].cell.0];
        let bel_pin = physical_pin_by_name(graph, pin, bel, physical_name).ok_or_else(|| {
            PnrError::InvalidPinNameBinding {
                pin,
                name: physical_name.clone(),
                reason: format!("selected BEL {} does not expose a compatible pin", bel.0),
            }
        })?;
        pin_bindings.insert(pin, bel_pin);
    }
    Ok(Placement {
        bindings,
        pin_bindings,
    })
}

fn choose_assignment<'a>(
    cells: &[CellId],
    assignments: impl Iterator<Item = &'a [BelId]>,
    device: &Device,
    neighbors: &[Vec<CellId>],
    placed: &[Option<BelId>],
    occupied: &BTreeSet<BelId>,
) -> Option<Vec<BelId>> {
    assignments
        .filter(|assignment| assignment.iter().all(|bel| !occupied.contains(bel)))
        .map(|assignment| {
            let cost = cells
                .iter()
                .zip(assignment)
                .map(|(cell, bel)| {
                    let point = device.bels()[bel.0].point;
                    neighbors[cell.0]
                        .iter()
                        .filter_map(|neighbor| placed[neighbor.0])
                        .map(|neighbor_bel| point.manhattan(device.bels()[neighbor_bel.0].point))
                        .sum::<u64>()
                })
                .sum::<u64>();
            let points = assignment
                .iter()
                .map(|bel| device.bels()[bel.0].point)
                .collect::<Vec<_>>();
            (cost, points, assignment)
        })
        .min()
        .map(|(_, _, assignment)| assignment.to_vec())
}

fn placement_units(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    candidate_cache: &mut BTreeMap<PlacementCandidateKey, Arc<[BelId]>>,
) -> Result<Vec<PlacementUnit>, PnrError> {
    validate_pin_bindings(graph, constraints)?;
    let mut constrained = BTreeSet::new();
    let mut units = Vec::new();
    for (group_index, group) in constraints.groups.iter().enumerate() {
        if group.cells.is_empty() || group.assignments.is_empty() {
            return Err(PnrError::InvalidPlacementConstraint {
                group: group_index,
                reason: "group and assignment set must be non-empty".into(),
            });
        }
        for &cell in &group.cells {
            if cell.0 >= graph.design().cells().len() {
                return Err(PnrError::InvalidPlacementConstraint {
                    group: group_index,
                    reason: format!("unknown cell ID {}", cell.0),
                });
            }
            if !constrained.insert(cell) {
                return Err(PnrError::InvalidPlacementConstraint {
                    group: group_index,
                    reason: format!("cell ID {} occurs in more than one group", cell.0),
                });
            }
        }
        let candidate_sets = group
            .cells
            .iter()
            .map(|&cell| cached_placement_candidates(graph, constraints, cell, candidate_cache))
            .collect::<Result<Vec<_>, _>>()?;
        for assignment in group.assignments.iter() {
            if assignment.len() != group.cells.len() {
                return Err(PnrError::InvalidPlacementConstraint {
                    group: group_index,
                    reason: "assignment width does not match group width".into(),
                });
            }
            let mut unique_bels = BTreeSet::new();
            for ((&cell, candidates), &bel) in
                group.cells.iter().zip(&candidate_sets).zip(assignment)
            {
                if !unique_bels.insert(bel) {
                    return Err(PnrError::InvalidPlacementConstraint {
                        group: group_index,
                        reason: format!("BEL ID {} is assigned more than once", bel.0),
                    });
                }
                if candidates.binary_search(&bel).is_err() {
                    return Err(PnrError::InvalidPlacementConstraint {
                        group: group_index,
                        reason: format!("BEL ID {} is incompatible with cell ID {}", bel.0, cell.0),
                    });
                }
            }
        }
        units.push(PlacementUnit {
            cells: group.cells.clone(),
            choices: PlacementChoices::Shared(Arc::clone(&group.assignments)),
        });
    }
    for index in 0..graph.design().cells().len() {
        let cell = CellId(index);
        if !constrained.contains(&cell) {
            let candidates =
                cached_placement_candidates(graph, constraints, cell, candidate_cache)?;
            units.push(PlacementUnit {
                cells: vec![cell],
                choices: PlacementChoices::SingleCell(candidates),
            });
        }
    }
    Ok(units)
}

fn cached_placement_candidates(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    cell: CellId,
    cache: &mut BTreeMap<PlacementCandidateKey, Arc<[BelId]>>,
) -> Result<Arc<[BelId]>, PnrError> {
    let logical_cell = graph
        .design()
        .cells()
        .get(cell.0)
        .ok_or(PnrError::Model(ModelError::UnknownCell(cell)))?;
    let has_candidate_specific_binding = logical_cell.pins().iter().any(|pin| {
        constraints
            .pin_bindings
            .keys()
            .any(|(bound_pin, _)| bound_pin == pin)
    });
    if has_candidate_specific_binding {
        return Ok(placement_candidates(graph, constraints, cell)?.into());
    }
    let key = PlacementCandidateKey {
        kind: logical_cell.kind,
        pins: logical_cell
            .pins()
            .iter()
            .map(|pin| {
                let logical = &graph.design().pins()[pin.0];
                (
                    constraints
                        .pin_name_bindings
                        .get(pin)
                        .cloned()
                        .unwrap_or_else(|| logical.name.clone()),
                    logical.direction,
                )
            })
            .collect(),
    };
    if let Some(candidates) = cache.get(&key) {
        return Ok(Arc::clone(candidates));
    }
    let candidates: Arc<[BelId]> = placement_candidates(graph, constraints, cell)?.into();
    cache.insert(key, Arc::clone(&candidates));
    Ok(candidates)
}

fn validate_pin_bindings(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
) -> Result<(), PnrError> {
    for (&(pin, bel), &bel_pin) in &constraints.pin_bindings {
        let Some(logical) = graph.design().pins().get(pin.0) else {
            return Err(PnrError::InvalidPinBinding {
                pin,
                bel,
                reason: "unknown logical pin".into(),
            });
        };
        let Some(physical_bel) = graph.device().bels().get(bel.0) else {
            return Err(PnrError::InvalidPinBinding {
                pin,
                bel,
                reason: "unknown BEL".into(),
            });
        };
        let Some(physical_pin) = graph.device().bel_pins().get(bel_pin.0) else {
            return Err(PnrError::InvalidPinBinding {
                pin,
                bel,
                reason: "unknown BEL pin".into(),
            });
        };
        if physical_pin.bel != bel {
            return Err(PnrError::InvalidPinBinding {
                pin,
                bel,
                reason: "physical pin belongs to another BEL".into(),
            });
        }
        if graph.design().cells()[logical.cell.0].kind != physical_bel.kind {
            return Err(PnrError::InvalidPinBinding {
                pin,
                bel,
                reason: "cell and BEL resource classes differ".into(),
            });
        }
        if logical.direction != physical_pin.direction {
            return Err(PnrError::InvalidPinBinding {
                pin,
                bel,
                reason: "logical and physical pin directions differ".into(),
            });
        }
    }
    for (&pin, name) in &constraints.pin_name_bindings {
        if graph.design().pins().get(pin.0).is_none() {
            return Err(PnrError::InvalidPinNameBinding {
                pin,
                name: name.clone(),
                reason: "unknown logical pin".into(),
            });
        }
        if name.is_empty() {
            return Err(PnrError::InvalidPinNameBinding {
                pin,
                name: name.clone(),
                reason: "physical pin name must not be empty".into(),
            });
        }
    }
    Ok(())
}

fn physical_pin_by_name(
    graph: &UnifiedGraph<'_>,
    logical_pin: CellPinId,
    bel: BelId,
    physical_name: &str,
) -> Option<BelPinId> {
    let logical = graph.design().pins().get(logical_pin.0)?;
    graph
        .device()
        .bels()
        .get(bel.0)?
        .pins()
        .iter()
        .copied()
        .find(|pin| {
            let physical = &graph.device().bel_pins()[pin.0];
            physical.name == physical_name && physical.direction == logical.direction
        })
}

fn placement_candidates(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    cell: CellId,
) -> Result<Vec<BelId>, PnrError> {
    let Some(logical_cell) = graph.design().cells().get(cell.0) else {
        return Err(PnrError::Model(ModelError::UnknownCell(cell)));
    };
    Ok(graph
        .device()
        .bels_of_kind(logical_cell.kind)
        .iter()
        .copied()
        .filter(|bel| {
            let physical_bel = &graph.device().bels()[bel.0];
            logical_cell.pins().iter().all(|logical_pin| {
                let logical = &graph.design().pins()[logical_pin.0];
                if let Some(physical_pin) = constraints.pin_bindings.get(&(*logical_pin, *bel)) {
                    let physical = &graph.device().bel_pins()[physical_pin.0];
                    physical.bel == *bel && physical.direction == logical.direction
                } else if let Some(physical_name) = constraints.pin_name_bindings.get(logical_pin) {
                    physical_pin_by_name(graph, *logical_pin, *bel, physical_name).is_some()
                } else {
                    physical_bel.pins().iter().any(|physical_pin| {
                        let physical = &graph.device().bel_pins()[physical_pin.0];
                        physical.name == logical.name && physical.direction == logical.direction
                    })
                }
            })
        })
        .collect())
}

const MAX_ROUTING_ITERATIONS: u32 = 32;

fn route(graph: &UnifiedGraph<'_>, placement: &Placement) -> Result<Vec<NetRoute>, PnrError> {
    let design = graph.design();
    let device = graph.device();
    let mut wire_occupancy = vec![0_u16; device.wires().len()];
    let mut pip_occupancy = vec![0_u16; device.pips().len()];
    let mut wire_history = vec![0_u32; device.wires().len()];
    let mut pip_history = vec![0_u32; device.pips().len()];
    let mut routes = vec![None; design.nets().len()];
    let mut search = RouteSearch::new(device.wires().len());
    for iteration in 0..MAX_ROUTING_ITERATIONS {
        let present_factor = 1_u32 << iteration.min(12);
        for (index, net) in design.nets().iter().enumerate() {
            if let Some(previous) = routes[index].take() {
                remove_route(&previous, &mut wire_occupancy, &mut pip_occupancy);
            }
            let net_id = NetId(index);
            let driver_cell = design.pins()[net.driver.0].cell;
            let driver_bel = placement
                .bel(driver_cell)
                .ok_or(PnrError::MissingPlacement { cell: driver_cell })?;
            let driver_wire = bound_wire(graph, placement, net.driver, driver_bel)?;
            let mut tree_wires = BTreeSet::from([driver_wire]);
            let mut tree_pips = BTreeSet::new();

            for sink_pin in &net.sinks {
                let sink_cell = design.pins()[sink_pin.0].cell;
                let sink_bel = placement
                    .bel(sink_cell)
                    .ok_or(PnrError::MissingPlacement { cell: sink_cell })?;
                let sink_wire = bound_wire(graph, placement, *sink_pin, sink_bel)?;
                if tree_wires.contains(&sink_wire) {
                    continue;
                }
                let (path_wires, path_pips) = search
                    .shortest_path(
                        graph,
                        &tree_wires,
                        sink_wire,
                        &wire_occupancy,
                        &pip_occupancy,
                        &wire_history,
                        &pip_history,
                        present_factor,
                    )
                    .ok_or_else(|| PnrError::Unroutable {
                        net: net.name.clone(),
                        driver: format!(
                            "{}.{} via {}",
                            design.cells()[driver_cell.0].name,
                            design.pins()[net.driver.0].name,
                            device.wires()[driver_wire.0].name
                        ),
                        sink: format!(
                            "{}.{} via {}",
                            design.cells()[sink_cell.0].name,
                            design.pins()[sink_pin.0].name,
                            device.wires()[sink_wire.0].name
                        ),
                    })?;
                tree_wires.extend(path_wires);
                tree_pips.extend(path_pips);
            }

            let route = NetRoute {
                net: net_id,
                wires: tree_wires.into_iter().collect(),
                pips: tree_pips.into_iter().collect(),
            };
            add_route(&route, &mut wire_occupancy, &mut pip_occupancy);
            routes[index] = Some(route);
        }

        let overused_wires = update_congestion_history(
            &wire_occupancy,
            device.wires().iter().map(|wire| wire.capacity),
            &mut wire_history,
        );
        let overused_pips = update_congestion_history(
            &pip_occupancy,
            device.pips().iter().map(|pip| pip.capacity),
            &mut pip_history,
        );
        if overused_wires == 0 && overused_pips == 0 {
            return Ok(routes
                .into_iter()
                .map(|route| route.expect("every net was routed in this iteration"))
                .collect());
        }
    }

    let overused_wires = count_overused(
        &wire_occupancy,
        device.wires().iter().map(|wire| wire.capacity),
    );
    let overused_pips =
        count_overused(&pip_occupancy, device.pips().iter().map(|pip| pip.capacity));
    Err(PnrError::CongestionNotResolved {
        iterations: MAX_ROUTING_ITERATIONS,
        overused_wires,
        overused_pips,
    })
}

fn add_route(route: &NetRoute, wire_occupancy: &mut [u16], pip_occupancy: &mut [u16]) {
    for wire in &route.wires {
        wire_occupancy[wire.0] += 1;
    }
    for pip in &route.pips {
        pip_occupancy[pip.0] += 1;
    }
}

fn remove_route(route: &NetRoute, wire_occupancy: &mut [u16], pip_occupancy: &mut [u16]) {
    for wire in &route.wires {
        wire_occupancy[wire.0] -= 1;
    }
    for pip in &route.pips {
        pip_occupancy[pip.0] -= 1;
    }
}

fn count_overused(occupancy: &[u16], capacities: impl Iterator<Item = u16>) -> usize {
    occupancy
        .iter()
        .zip(capacities)
        .filter(|(used, capacity)| **used > *capacity)
        .count()
}

fn update_congestion_history(
    occupancy: &[u16],
    capacities: impl Iterator<Item = u16>,
    history: &mut [u32],
) -> usize {
    let mut overused = 0;
    for ((&used, capacity), history) in occupancy.iter().zip(capacities).zip(history) {
        if used > capacity {
            overused += 1;
            *history = history.saturating_add(u32::from(used - capacity));
        }
    }
    overused
}

fn bound_wire(
    graph: &UnifiedGraph<'_>,
    placement: &Placement,
    cell_pin: CellPinId,
    bel: BelId,
) -> Result<WireId, PnrError> {
    if let Some(bel_pin) = placement.pin_binding(cell_pin) {
        Ok(graph.device().bel_pins()[bel_pin.0].wire)
    } else {
        Ok(graph.bound_wire(cell_pin, bel)?)
    }
}

struct RouteSearch {
    epoch: u32,
    seen: Vec<u32>,
    distance: Vec<u64>,
    previous_wire: Vec<usize>,
    previous_pip: Vec<usize>,
}

impl RouteSearch {
    fn new(wire_count: usize) -> Self {
        Self {
            epoch: 0,
            seen: vec![0; wire_count],
            distance: vec![0; wire_count],
            previous_wire: vec![usize::MAX; wire_count],
            previous_pip: vec![usize::MAX; wire_count],
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn shortest_path(
        &mut self,
        graph: &UnifiedGraph<'_>,
        starts: &BTreeSet<WireId>,
        goal: WireId,
        wire_occupancy: &[u16],
        pip_occupancy: &[u16],
        wire_history: &[u32],
        pip_history: &[u32],
        present_factor: u32,
    ) -> Option<(Vec<WireId>, Vec<PipId>)> {
        self.epoch = self.epoch.wrapping_add(1);
        if self.epoch == 0 {
            self.seen.fill(0);
            self.epoch = 1;
        }
        let epoch = self.epoch;
        let device = graph.device();
        let goal_point = device.wires()[goal.0].point;
        let mut queue = BinaryHeap::new();
        for &start in starts {
            self.seen[start.0] = epoch;
            self.distance[start.0] = 0;
            self.previous_wire[start.0] = usize::MAX;
            self.previous_pip[start.0] = usize::MAX;
            queue.push(Reverse((
                device.wires()[start.0].point.manhattan(goal_point),
                0_u64,
                start,
            )));
        }

        while let Some(Reverse((_, distance, wire))) = queue.pop() {
            if self.seen[wire.0] != epoch || self.distance[wire.0] != distance {
                continue;
            }
            if wire == goal {
                let mut path_wires = vec![wire];
                let mut path_pips = Vec::new();
                let mut cursor = wire.0;
                while self.previous_wire[cursor] != usize::MAX {
                    path_pips.push(PipId(self.previous_pip[cursor]));
                    cursor = self.previous_wire[cursor];
                    path_wires.push(WireId(cursor));
                }
                return Some((path_wires, path_pips));
            }

            for (neighbor, pip) in graph.routing_neighbors(wire).ok()? {
                let step = 1_u64
                    + congestion_cost(
                        wire_occupancy[neighbor.0],
                        device.wires()[neighbor.0].capacity,
                        wire_history[neighbor.0],
                        present_factor,
                    )
                    + congestion_cost(
                        pip_occupancy[pip.0],
                        device.pips()[pip.0].capacity,
                        pip_history[pip.0],
                        present_factor,
                    );
                let next_distance = distance.saturating_add(step);
                if self.seen[neighbor.0] == epoch && self.distance[neighbor.0] <= next_distance {
                    continue;
                }
                self.seen[neighbor.0] = epoch;
                self.distance[neighbor.0] = next_distance;
                self.previous_wire[neighbor.0] = wire.0;
                self.previous_pip[neighbor.0] = pip.0;
                let estimate = next_distance
                    .saturating_add(device.wires()[neighbor.0].point.manhattan(goal_point));
                queue.push(Reverse((estimate, next_distance, neighbor)));
            }
        }
        None
    }
}

fn congestion_cost(occupancy: u16, capacity: u16, history: u32, present: u32) -> u64 {
    let prospective_overuse = occupancy.saturating_add(1).saturating_sub(capacity);
    u64::from(history) + u64::from(present) * u64::from(prospective_overuse)
}

/// `PnR` failure with the responsible object identified.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PnrError {
    /// Logical or physical model was invalid.
    Model(ModelError),
    /// No compatible unoccupied BEL exists.
    NoBel {
        /// Cell that could not be placed.
        cell: String,
    },
    /// A target packer supplied a malformed or incompatible atomic group.
    InvalidPlacementConstraint {
        /// Group index in [`PlacementConstraints`].
        group: usize,
        /// Specific invariant that failed.
        reason: String,
    },
    /// A target packer supplied an invalid candidate-specific pin binding.
    InvalidPinBinding {
        /// Logical pin being overridden.
        pin: CellPinId,
        /// BEL choice for which the override applies.
        bel: BelId,
        /// Specific invariant that failed.
        reason: String,
    },
    /// A target packer supplied an invalid BEL-independent pin-name binding.
    InvalidPinNameBinding {
        /// Logical pin being overridden.
        pin: CellPinId,
        /// Requested physical pin name.
        name: String,
        /// Specific invariant that failed.
        reason: String,
    },
    /// Internal or externally supplied placement omitted a cell.
    MissingPlacement {
        /// Missing cell ID.
        cell: CellId,
    },
    /// No capacity-respecting directed route exists.
    Unroutable {
        /// Net that could not be connected.
        net: String,
        /// Logical driver and its bound physical wire.
        driver: String,
        /// Logical sink and its bound physical wire.
        sink: String,
    },
    /// Negotiated routing did not remove all resource overuse.
    CongestionNotResolved {
        /// Routing iterations attempted.
        iterations: u32,
        /// Wires still over capacity.
        overused_wires: usize,
        /// PIPs still over capacity.
        overused_pips: usize,
    },
}

impl fmt::Display for PnrError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Model(error) => write!(f, "invalid problem graph: {error}"),
            Self::NoBel { cell } => write!(f, "no compatible free BEL for cell `{cell}`"),
            Self::InvalidPlacementConstraint { group, reason } => {
                write!(f, "invalid placement constraint group {group}: {reason}")
            }
            Self::InvalidPinBinding { pin, bel, reason } => {
                write!(
                    f,
                    "invalid physical pin binding for pin {} on BEL {}: {reason}",
                    pin.0, bel.0
                )
            }
            Self::InvalidPinNameBinding { pin, name, reason } => {
                write!(
                    f,
                    "invalid physical pin-name binding for pin {} as `{name}`: {reason}",
                    pin.0
                )
            }
            Self::MissingPlacement { cell } => {
                write!(f, "placement is missing cell ID {}", cell.0)
            }
            Self::Unroutable { net, driver, sink } => {
                write!(f, "net `{net}` is unroutable from {driver} to {sink}")
            }
            Self::CongestionNotResolved {
                iterations,
                overused_wires,
                overused_pips,
            } => write!(
                f,
                "routing congestion remains after {iterations} iterations: \
                 {overused_wires} overused wires, {overused_pips} overused PIPs"
            ),
        }
    }
}

impl Error for PnrError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Model(error) => Some(error),
            Self::NoBel { .. }
            | Self::InvalidPlacementConstraint { .. }
            | Self::InvalidPinBinding { .. }
            | Self::InvalidPinNameBinding { .. }
            | Self::MissingPlacement { .. }
            | Self::Unroutable { .. }
            | Self::CongestionNotResolved { .. } => None,
        }
    }
}

impl From<ModelError> for PnrError {
    fn from(value: ModelError) -> Self {
        Self::Model(value)
    }
}

#[cfg(test)]
mod tests {
    use texo_model::{BelId, CellId, Design, Device, PinDirection, Point, ResourceKind};

    use super::{PlacementConstraints, PnrError, place_and_route, place_with_constraints};

    fn two_cell_design() -> Design {
        let mut design = Design::new();
        let source = design.add_cell("source", ResourceKind::Logic);
        let source_out = design.add_pin(source, "out", PinDirection::Output).unwrap();
        let sink = design.add_cell("sink", ResourceKind::Logic);
        let sink_in = design.add_pin(sink, "in", PinDirection::Input).unwrap();
        design
            .add_net("source_to_sink", source_out, [sink_in])
            .unwrap();
        design
    }

    #[test]
    fn binds_bels_and_routes_through_wires_and_pips() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(4, 4).unwrap();

        let result = place_and_route(&design, &device).unwrap();

        assert_ne!(
            result.placement.bindings()[0],
            result.placement.bindings()[1]
        );
        assert_eq!(result.routes.len(), 1);
        assert!(!result.routes[0].wires.is_empty());
        assert!(!result.routes[0].pips.is_empty());
        assert_eq!(result.total_pips, result.routes[0].pips.len());
    }

    #[test]
    fn reports_bel_exhaustion() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(1, 1).unwrap();

        assert_eq!(
            place_and_route(&design, &device),
            Err(PnrError::NoBel {
                cell: "sink".into()
            })
        );
    }

    #[test]
    fn places_a_target_group_atomically() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(2, 1).unwrap();
        let mut constraints = PlacementConstraints::new();
        constraints.add_group([CellId(0), CellId(1)], [vec![BelId(1), BelId(0)]]);

        let placement = place_with_constraints(&design, &device, &constraints).unwrap();

        assert_eq!(placement.bindings(), &[BelId(1), BelId(0)]);
    }

    #[test]
    fn rejects_a_group_that_reuses_one_bel() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(2, 1).unwrap();
        let mut constraints = PlacementConstraints::new();
        constraints.add_group([CellId(0), CellId(1)], [vec![BelId(0), BelId(0)]]);

        assert!(matches!(
            place_with_constraints(&design, &device, &constraints),
            Err(PnrError::InvalidPlacementConstraint { .. })
        ));
    }

    #[test]
    fn target_pin_override_participates_in_candidate_generation() {
        let mut design = Design::new();
        let cell = design.add_cell("cell", ResourceKind::Logic);
        let logical_pin = design
            .add_pin(cell, "logical", PinDirection::Output)
            .unwrap();
        let mut device = Device::new("renamed-pin", 1, 1).unwrap();
        let wire = device.add_wire("wire", Point::new(0, 0), 1).unwrap();
        let bel = device
            .add_bel("bel", ResourceKind::Logic, Point::new(0, 0))
            .unwrap();
        let physical_pin = device
            .add_bel_pin(bel, "physical", PinDirection::Output, wire)
            .unwrap();
        let mut constraints = PlacementConstraints::new();
        constraints.bind_pin(logical_pin, bel, physical_pin);

        let placement = place_with_constraints(&design, &device, &constraints).unwrap();

        assert_eq!(placement.bel(cell), Some(bel));
        assert_eq!(placement.pin_binding(logical_pin), Some(physical_pin));
    }

    #[test]
    fn target_pin_name_override_is_resolved_after_candidate_selection() {
        let mut design = Design::new();
        let cell = design.add_cell("cell", ResourceKind::Logic);
        let logical_pin = design
            .add_pin(cell, "logical", PinDirection::Output)
            .unwrap();
        let mut device = Device::new("renamed-pin", 2, 1).unwrap();
        let compatible_wire = device.add_wire("compatible", Point::new(0, 0), 1).unwrap();
        let incompatible_wire = device
            .add_wire("incompatible", Point::new(1, 0), 1)
            .unwrap();
        let compatible = device
            .add_bel("compatible", ResourceKind::Logic, Point::new(0, 0))
            .unwrap();
        let physical_pin = device
            .add_bel_pin(
                compatible,
                "physical",
                PinDirection::Output,
                compatible_wire,
            )
            .unwrap();
        let incompatible = device
            .add_bel("incompatible", ResourceKind::Logic, Point::new(1, 0))
            .unwrap();
        device
            .add_bel_pin(
                incompatible,
                "other",
                PinDirection::Output,
                incompatible_wire,
            )
            .unwrap();
        let mut constraints = PlacementConstraints::new();
        constraints.bind_pin_name(logical_pin, "physical");

        let placement = place_with_constraints(&design, &device, &constraints).unwrap();

        assert_eq!(placement.bel(cell), Some(compatible));
        assert_eq!(placement.pin_binding(logical_pin), Some(physical_pin));
    }

    #[test]
    #[allow(clippy::similar_names)]
    fn rejects_a_second_net_at_a_saturated_wire() {
        let mut design = Design::new();
        let source_a = design.add_cell("source_a", ResourceKind::Logic);
        let source_a_pin = design
            .add_pin(source_a, "source_a", PinDirection::Output)
            .unwrap();
        let sink_a = design.add_cell("sink_a", ResourceKind::Logic);
        let sink_a_pin = design
            .add_pin(sink_a, "sink_a", PinDirection::Input)
            .unwrap();
        let source_b = design.add_cell("source_b", ResourceKind::Logic);
        let source_b_pin = design
            .add_pin(source_b, "source_b", PinDirection::Output)
            .unwrap();
        let sink_b = design.add_cell("sink_b", ResourceKind::Logic);
        let sink_b_pin = design
            .add_pin(sink_b, "sink_b", PinDirection::Input)
            .unwrap();
        design.add_net("first", source_a_pin, [sink_a_pin]).unwrap();
        design
            .add_net("second", source_b_pin, [sink_b_pin])
            .unwrap();

        let mut device = Device::new("bottleneck", 5, 1).unwrap();
        let source_a_wire = device.add_wire("source_a", Point::new(0, 0), 1).unwrap();
        let source_b_wire = device.add_wire("source_b", Point::new(1, 0), 1).unwrap();
        let shared = device.add_wire("shared", Point::new(2, 0), 1).unwrap();
        let sink_a_wire = device.add_wire("sink_a", Point::new(3, 0), 1).unwrap();
        let sink_b_wire = device.add_wire("sink_b", Point::new(4, 0), 1).unwrap();

        let source_a_bel = device
            .add_bel("source_a", ResourceKind::Logic, Point::new(0, 0))
            .unwrap();
        device
            .add_bel_pin(
                source_a_bel,
                "source_a",
                PinDirection::Output,
                source_a_wire,
            )
            .unwrap();
        let source_b_bel = device
            .add_bel("source_b", ResourceKind::Logic, Point::new(1, 0))
            .unwrap();
        device
            .add_bel_pin(
                source_b_bel,
                "source_b",
                PinDirection::Output,
                source_b_wire,
            )
            .unwrap();
        let sink_a_bel = device
            .add_bel("sink_a", ResourceKind::Logic, Point::new(3, 0))
            .unwrap();
        device
            .add_bel_pin(sink_a_bel, "sink_a", PinDirection::Input, sink_a_wire)
            .unwrap();
        let sink_b_bel = device
            .add_bel("sink_b", ResourceKind::Logic, Point::new(4, 0))
            .unwrap();
        device
            .add_bel_pin(sink_b_bel, "sink_b", PinDirection::Input, sink_b_wire)
            .unwrap();

        device.add_pip(source_a_wire, shared, false, 1).unwrap();
        device.add_pip(source_b_wire, shared, false, 1).unwrap();
        device.add_pip(shared, sink_a_wire, false, 1).unwrap();
        device.add_pip(shared, sink_b_wire, false, 1).unwrap();

        assert!(matches!(
            place_and_route(&design, &device),
            Err(PnrError::CongestionNotResolved {
                overused_wires: 1,
                overused_pips: 0,
                ..
            })
        ));
    }
}
