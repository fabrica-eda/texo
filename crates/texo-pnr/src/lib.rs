//! Deterministic reference placement and routing on the unified problem graph.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::error::Error;
use std::fmt;

use texo_model::{
    BelId, CellId, Design, Device, ModelError, NetId, PipId, Point, UnifiedGraph, WireId,
};

/// Cell-to-BEL bindings indexed by stable cell ID.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Placement {
    bindings: Vec<BelId>,
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
    let graph = UnifiedGraph::new(design, device);
    let placement = place(&graph)?;
    let routes = route(&graph, &placement)?;
    let total_pips = routes.iter().map(|route| route.pips.len()).sum();
    Ok(PnrResult {
        placement,
        routes,
        total_pips,
    })
}

fn place(graph: &UnifiedGraph<'_>) -> Result<Placement, PnrError> {
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

    let mut order: Vec<_> = (0..design.cells().len()).map(CellId).collect();
    order.sort_by_key(|id| (std::cmp::Reverse(degree[id.0]), *id));

    let mut placed = vec![None; design.cells().len()];
    let mut occupied = BTreeSet::new();
    for cell_id in order {
        let cell = &design.cells()[cell_id.0];
        let choice = graph
            .placement_candidates(cell_id)?
            .into_iter()
            .filter(|bel| !occupied.contains(bel))
            .map(|bel| {
                let point = device.bels()[bel.0].point;
                let cost: u64 = neighbors[cell_id.0]
                    .iter()
                    .filter_map(|neighbor| placed[neighbor.0])
                    .map(|neighbor_bel: BelId| point.manhattan(device.bels()[neighbor_bel.0].point))
                    .sum();
                (cost, point, bel)
            })
            .min();
        let (_, _, bel) = choice.ok_or_else(|| PnrError::NoBel {
            cell: cell.name.clone(),
        })?;
        occupied.insert(bel);
        placed[cell_id.0] = Some(bel);
    }

    Ok(Placement {
        bindings: placed
            .into_iter()
            .map(|bel| bel.expect("every ordered cell was placed"))
            .collect(),
    })
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
        let driver_wire = graph.bound_wire(net.driver, driver_bel)?;
        let mut tree_wires = BTreeSet::from([driver_wire]);
        let mut tree_pips = BTreeSet::new();

        for sink_pin in &net.sinks {
            let sink_cell = design.pins()[sink_pin.0].cell;
            let sink_bel = placement
                .bel(sink_cell)
                .ok_or(PnrError::MissingPlacement { cell: sink_cell })?;
            let sink_wire = graph.bound_wire(*sink_pin, sink_bel)?;
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
            Self::NoBel { .. } | Self::MissingPlacement { .. } | Self::Unroutable { .. } => None,
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
    use texo_model::{Design, Device, PinDirection, Point, ResourceKind};

    use super::{PnrError, place_and_route};

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
