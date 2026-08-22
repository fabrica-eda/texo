//! Deterministic reference placement and routing on the unified problem graph.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::error::Error;
use std::fmt;

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
    pub assignments: Vec<Vec<BelId>>,
}

/// Optional grouped/fixed placement rules supplied by a target packer.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct PlacementConstraints {
    groups: Vec<PlacementGroup>,
    pin_bindings: BTreeMap<(CellPinId, BelId), BelPinId>,
}

impl PlacementConstraints {
    /// Creates an unconstrained placement problem.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            groups: Vec::new(),
            pin_bindings: BTreeMap::new(),
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
            assignments: assignments.into_iter().collect(),
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
    assignments: Vec<Vec<BelId>>,
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

    let mut units = placement_units(graph, constraints)?;
    units.sort_by_key(|unit| {
        (
            std::cmp::Reverse(unit.cells.iter().map(|cell| degree[cell.0]).sum::<usize>()),
            unit.cells[0],
        )
    });

    let mut placed = vec![None; design.cells().len()];
    let mut occupied = BTreeSet::new();
    for unit in units {
        let choice = unit
            .assignments
            .into_iter()
            .filter(|assignment| assignment.iter().all(|bel| !occupied.contains(bel)))
            .map(|assignment| {
                let cost = unit
                    .cells
                    .iter()
                    .zip(&assignment)
                    .map(|(cell, bel)| {
                        let point = device.bels()[bel.0].point;
                        neighbors[cell.0]
                            .iter()
                            .filter_map(|neighbor| placed[neighbor.0])
                            .map(|neighbor_bel: BelId| {
                                point.manhattan(device.bels()[neighbor_bel.0].point)
                            })
                            .sum::<u64>()
                    })
                    .sum::<u64>();
                let points = assignment
                    .iter()
                    .map(|bel| device.bels()[bel.0].point)
                    .collect::<Vec<_>>();
                (cost, points, assignment)
            })
            .min();
        let (_, _, assignment) = choice.ok_or_else(|| PnrError::NoBel {
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
    let pin_bindings = constraints
        .pin_bindings
        .iter()
        .filter(|((pin, bel), _)| bindings[design.pins()[pin.0].cell.0] == *bel)
        .map(|((pin, _), bel_pin)| (*pin, *bel_pin))
        .collect();
    Ok(Placement {
        bindings,
        pin_bindings,
    })
}

fn placement_units(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
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
        for assignment in &group.assignments {
            if assignment.len() != group.cells.len() {
                return Err(PnrError::InvalidPlacementConstraint {
                    group: group_index,
                    reason: "assignment width does not match group width".into(),
                });
            }
            let mut unique_bels = BTreeSet::new();
            for (&cell, &bel) in group.cells.iter().zip(assignment) {
                if !unique_bels.insert(bel) {
                    return Err(PnrError::InvalidPlacementConstraint {
                        group: group_index,
                        reason: format!("BEL ID {} is assigned more than once", bel.0),
                    });
                }
                if !placement_candidates(graph, constraints, cell)?.contains(&bel) {
                    return Err(PnrError::InvalidPlacementConstraint {
                        group: group_index,
                        reason: format!("BEL ID {} is incompatible with cell ID {}", bel.0, cell.0),
                    });
                }
            }
        }
        units.push(PlacementUnit {
            cells: group.cells.clone(),
            assignments: group.assignments.clone(),
        });
    }
    for index in 0..graph.design().cells().len() {
        let cell = CellId(index);
        if !constrained.contains(&cell) {
            units.push(PlacementUnit {
                cells: vec![cell],
                assignments: placement_candidates(graph, constraints, cell)?
                    .into_iter()
                    .map(|bel| vec![bel])
                    .collect(),
            });
        }
    }
    Ok(units)
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
    Ok(())
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
        .bels()
        .iter()
        .enumerate()
        .filter(|(index, physical_bel)| {
            if physical_bel.kind != logical_cell.kind {
                return false;
            }
            let bel = BelId(*index);
            logical_cell.pins().iter().all(|logical_pin| {
                let logical = &graph.design().pins()[logical_pin.0];
                if let Some(physical_pin) = constraints.pin_bindings.get(&(*logical_pin, bel)) {
                    let physical = &graph.device().bel_pins()[physical_pin.0];
                    physical.bel == bel && physical.direction == logical.direction
                } else {
                    physical_bel.pins().iter().any(|physical_pin| {
                        let physical = &graph.device().bel_pins()[physical_pin.0];
                        physical.name == logical.name && physical.direction == logical.direction
                    })
                }
            })
        })
        .map(|(index, _)| BelId(index))
        .collect())
}

fn route(graph: &UnifiedGraph<'_>, placement: &Placement) -> Result<Vec<NetRoute>, PnrError> {
    let design = graph.design();
    let device = graph.device();
    let mut wire_occupancy = vec![0_u16; device.wires().len()];
    let mut pip_occupancy = vec![0_u16; device.pips().len()];
    let mut routes = Vec::with_capacity(design.nets().len());

    for (index, net) in design.nets().iter().enumerate() {
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
            let (path_wires, path_pips) = shortest_path(
                graph,
                &tree_wires,
                sink_wire,
                &tree_pips,
                &wire_occupancy,
                &pip_occupancy,
            )
            .ok_or_else(|| PnrError::Unroutable {
                net: net.name.clone(),
            })?;
            tree_wires.extend(path_wires);
            tree_pips.extend(path_pips);
        }

        for wire in &tree_wires {
            wire_occupancy[wire.0] += 1;
        }
        for pip in &tree_pips {
            pip_occupancy[pip.0] += 1;
        }
        routes.push(NetRoute {
            net: net_id,
            wires: tree_wires.into_iter().collect(),
            pips: tree_pips.into_iter().collect(),
        });
    }
    Ok(routes)
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

fn shortest_path(
    graph: &UnifiedGraph<'_>,
    starts: &BTreeSet<WireId>,
    goal: WireId,
    current_pips: &BTreeSet<PipId>,
    wire_occupancy: &[u16],
    pip_occupancy: &[u16],
) -> Option<(Vec<WireId>, Vec<PipId>)> {
    let mut queue = VecDeque::new();
    let mut predecessor: BTreeMap<WireId, Option<(WireId, PipId)>> = BTreeMap::new();
    for &start in starts {
        queue.push_back(start);
        predecessor.insert(start, None);
    }

    while let Some(wire) = queue.pop_front() {
        if wire == goal {
            let mut path_wires = vec![wire];
            let mut path_pips = Vec::new();
            let mut cursor = wire;
            while let Some(Some((previous, pip))) = predecessor.get(&cursor) {
                path_pips.push(*pip);
                cursor = *previous;
                path_wires.push(cursor);
            }
            return Some((path_wires, path_pips));
        }

        for (neighbor, pip) in graph.routing_neighbors(wire).ok()? {
            if predecessor.contains_key(&neighbor) {
                continue;
            }
            let wire_available = starts.contains(&neighbor)
                || wire_occupancy[neighbor.0] < graph.device().wires()[neighbor.0].capacity;
            let pip_available =
                current_pips.contains(&pip) || pip_occupancy[pip.0] < graph.pip(pip).ok()?.capacity;
            if wire_available && pip_available {
                predecessor.insert(neighbor, Some((wire, pip)));
                queue.push_back(neighbor);
            }
        }
    }
    None
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
    /// Internal or externally supplied placement omitted a cell.
    MissingPlacement {
        /// Missing cell ID.
        cell: CellId,
    },
    /// No capacity-respecting directed route exists.
    Unroutable {
        /// Net that could not be connected.
        net: String,
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
            Self::MissingPlacement { cell } => {
                write!(f, "placement is missing cell ID {}", cell.0)
            }
            Self::Unroutable { net } => write!(f, "net `{net}` is unroutable"),
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
            | Self::MissingPlacement { .. }
            | Self::Unroutable { .. } => None,
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

        assert_eq!(
            place_and_route(&design, &device),
            Err(PnrError::Unroutable {
                net: "second".into()
            })
        );
    }
}
