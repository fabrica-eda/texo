//! Deterministic reference placement and routing on the unified problem graph.

use std::cmp::Reverse;
use std::collections::{BTreeMap, BTreeSet, BinaryHeap, HashMap};
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

/// Target-supplied immutable portions of logical net trees.
///
/// This is used for architecture resources whose legal topology cannot be
/// discovered from local congestion costs alone, such as an ECP5 primary
/// clock spine. The generic router grows any still-unconnected sinks from the
/// locked tree and accounts for every fixed wire and PIP normally.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct RoutingConstraints {
    routes: BTreeMap<NetId, NetRoute>,
}

/// Characterized costs used by timing-driven negotiated routing.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RoutingCosts {
    pip_delays_ps: Vec<u32>,
    pip_min_delays_ps: Vec<u32>,
    net_criticalities: BTreeMap<NetId, u64>,
    sink_min_delays_ps: BTreeMap<(NetId, CellPinId), u64>,
    detailed_timing_nets: BTreeSet<NetId>,
    detailed_delay_quantum_ps: u64,
}

impl RoutingCosts {
    /// Creates costs indexed by stable PIP and logical net IDs.
    #[must_use]
    pub fn new(pip_delays_ps: Vec<u32>, net_criticalities: BTreeMap<NetId, u64>) -> Self {
        Self {
            pip_min_delays_ps: pip_delays_ps.clone(),
            pip_delays_ps,
            net_criticalities,
            sink_min_delays_ps: BTreeMap::new(),
            detailed_timing_nets: BTreeSet::new(),
            detailed_delay_quantum_ps: 1,
        }
    }

    /// Estimated maximum delay for every physical PIP.
    #[must_use]
    pub fn pip_delays_ps(&self) -> &[u32] {
        &self.pip_delays_ps
    }

    /// Replaces minimum-corner PIP delays used for hold repair.
    pub fn set_pip_min_delays_ps(&mut self, pip_min_delays_ps: Vec<u32>) {
        self.pip_min_delays_ps = pip_min_delays_ps;
    }

    /// Estimated minimum delay for every physical PIP.
    #[must_use]
    pub fn pip_min_delays_ps(&self) -> &[u32] {
        &self.pip_min_delays_ps
    }

    /// Criticality weights indexed by logical net.
    #[must_use]
    pub const fn net_criticalities(&self) -> &BTreeMap<NetId, u64> {
        &self.net_criticalities
    }

    /// Replaces logical-net criticalities while retaining the device delay table.
    pub fn set_net_criticalities(&mut self, net_criticalities: BTreeMap<NetId, u64>) {
        self.net_criticalities = net_criticalities;
    }

    /// Replaces per-sink minimum route delays used for hold repair.
    pub fn set_sink_min_delays_ps(
        &mut self,
        sink_min_delays_ps: BTreeMap<(NetId, CellPinId), u64>,
    ) {
        self.sink_min_delays_ps = sink_min_delays_ps;
    }

    /// Per-sink minimum route delays used for hold repair.
    #[must_use]
    pub const fn sink_min_delays_ps(&self) -> &BTreeMap<(NetId, CellPinId), u64> {
        &self.sink_min_delays_ps
    }

    /// Replaces the nets routed with exact picosecond delay resolution.
    pub fn set_detailed_timing_nets(&mut self, detailed_timing_nets: BTreeSet<NetId>) {
        self.detailed_timing_nets = detailed_timing_nets;
    }

    /// Nets routed with exact picosecond delay resolution.
    #[must_use]
    pub const fn detailed_timing_nets(&self) -> &BTreeSet<NetId> {
        &self.detailed_timing_nets
    }

    /// Sets the positive delay quantum used only for detailed timing nets.
    pub fn set_detailed_delay_quantum_ps(&mut self, detailed_delay_quantum_ps: u64) {
        self.detailed_delay_quantum_ps = detailed_delay_quantum_ps;
    }
}

impl RoutingConstraints {
    /// Creates an unconstrained routing problem.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            routes: BTreeMap::new(),
        }
    }

    /// Sets the immutable route tree for one logical net.
    pub fn add_route(&mut self, route: NetRoute) {
        self.routes.insert(route.net, route);
    }

    /// Immutable route trees indexed by logical net.
    #[must_use]
    pub const fn routes(&self) -> &BTreeMap<NetId, NetRoute> {
        &self.routes
    }
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

/// Retains only the branches needed to reach `sinks` from a routed net's
/// driver. The returned partial tree can be supplied as a routing constraint;
/// the router will preserve its shared trunk and grow the missing sink arcs.
///
/// # Errors
///
/// Returns an invalid-routing-constraint or placement error when the supplied
/// route is not a driver-rooted tree for the current placement.
pub fn retain_route_for_sinks(
    design: &Design,
    device: &Device,
    placement: &Placement,
    route: &NetRoute,
    sinks: &BTreeSet<CellPinId>,
) -> Result<Option<NetRoute>, PnrError> {
    if sinks.is_empty() {
        return Ok(None);
    }
    let graph = UnifiedGraph::new(design, device);
    let net = design
        .nets()
        .get(route.net.0)
        .ok_or_else(|| PnrError::InvalidRoutingConstraint {
            net: route.net,
            reason: "net ID is outside the design".into(),
        })?;
    if let Some(&sink) = sinks.iter().find(|sink| !net.sinks.contains(sink)) {
        return Err(PnrError::InvalidRoutingConstraint {
            net: route.net,
            reason: format!("pin {} is not a sink of the routed net", sink.0),
        });
    }
    let driver_cell = design.pins()[net.driver.0].cell;
    let driver_bel = placement
        .bel(driver_cell)
        .ok_or(PnrError::MissingPlacement { cell: driver_cell })?;
    let driver_wire = bound_wire(&graph, placement, net.driver, driver_bel)?;
    let route_wires = route.wires.iter().copied().collect::<BTreeSet<_>>();
    if !route_wires.contains(&driver_wire) {
        return Err(PnrError::InvalidRoutingConstraint {
            net: route.net,
            reason: "route does not contain its placed driver wire".into(),
        });
    }

    let mut adjacent = BTreeMap::<WireId, Vec<(WireId, PipId)>>::new();
    for &pip_id in &route.pips {
        let pip =
            device
                .pips()
                .get(pip_id.0)
                .ok_or_else(|| PnrError::InvalidRoutingConstraint {
                    net: route.net,
                    reason: format!("unknown PIP {pip_id:?}"),
                })?;
        adjacent.entry(pip.from).or_default().push((pip.to, pip_id));
        adjacent.entry(pip.to).or_default().push((pip.from, pip_id));
    }
    let mut parent = BTreeMap::<WireId, (WireId, PipId)>::new();
    let mut visited = BTreeSet::from([driver_wire]);
    let mut pending = vec![driver_wire];
    while let Some(wire) = pending.pop() {
        for &(next, pip) in adjacent.get(&wire).map_or(&[][..], Vec::as_slice) {
            if visited.insert(next) {
                parent.insert(next, (wire, pip));
                pending.push(next);
            }
        }
    }
    if visited != route_wires || route.pips.len().saturating_add(1) != route.wires.len() {
        return Err(PnrError::InvalidRoutingConstraint {
            net: route.net,
            reason: "route is not one connected tree rooted at its driver".into(),
        });
    }

    let mut retained_wires = BTreeSet::from([driver_wire]);
    let mut retained_pips = BTreeSet::new();
    for &sink in sinks {
        let sink_cell = design.pins()[sink.0].cell;
        let sink_bel = placement
            .bel(sink_cell)
            .ok_or(PnrError::MissingPlacement { cell: sink_cell })?;
        let mut wire = bound_wire(&graph, placement, sink, sink_bel)?;
        if !route_wires.contains(&wire) {
            return Err(PnrError::InvalidRoutingConstraint {
                net: route.net,
                reason: format!("route does not reach retained sink pin {}", sink.0),
            });
        }
        while wire != driver_wire {
            let &(previous, pip) =
                parent
                    .get(&wire)
                    .ok_or_else(|| PnrError::InvalidRoutingConstraint {
                        net: route.net,
                        reason: format!("retained sink pin {} is disconnected", sink.0),
                    })?;
            retained_wires.insert(wire);
            retained_pips.insert(pip);
            wire = previous;
        }
    }
    Ok(Some(NetRoute {
        net: route.net,
        wires: retained_wires.into_iter().collect(),
        pips: retained_pips.into_iter().collect(),
    }))
}

/// Deterministic progress event from negotiated routing.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RoutingProgress {
    /// One Pathfinder iteration is about to reroute its conflicting nets.
    Iteration {
        /// Zero-based iteration number.
        iteration: u32,
        /// Nets selected for this iteration.
        nets: usize,
    },
    /// One logical net is about to be routed.
    Net {
        /// Zero-based iteration number.
        iteration: u32,
        /// One-based ordinal within this iteration.
        ordinal: usize,
        /// Nets selected for this iteration.
        total: usize,
        /// Logical net being routed.
        net: NetId,
    },
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
    place_and_route_with_all_constraints(design, device, constraints, &RoutingConstraints::new())
}

/// Places and routes with target-supplied placement and routing constraints.
///
/// # Errors
///
/// Returns a descriptive constraint, model, legality, or routability error.
pub fn place_and_route_with_all_constraints(
    design: &Design,
    device: &Device,
    placement_constraints: &PlacementConstraints,
    routing_constraints: &RoutingConstraints,
) -> Result<PnrResult, PnrError> {
    let graph = UnifiedGraph::new(design, device);
    let placement = place(&graph, placement_constraints, None, None)?;
    finish_routing(&graph, placement, routing_constraints, None, &mut |_| {})
}

/// Routes a design from an already selected legal placement.
///
/// # Errors
///
/// Returns an invalid routing-constraint, model, or routability error.
pub fn route_with_placement(
    design: &Design,
    device: &Device,
    placement: Placement,
    routing_constraints: &RoutingConstraints,
) -> Result<PnrResult, PnrError> {
    route_with_placement_and_progress(design, device, placement, routing_constraints, |_| {})
}

/// Routes an existing placement while reporting deterministic progress.
///
/// # Errors
///
/// Returns an invalid routing-constraint, model, or routability error.
pub fn route_with_placement_and_progress(
    design: &Design,
    device: &Device,
    placement: Placement,
    routing_constraints: &RoutingConstraints,
    mut progress: impl FnMut(RoutingProgress),
) -> Result<PnrResult, PnrError> {
    finish_routing(
        &UnifiedGraph::new(design, device),
        placement,
        routing_constraints,
        None,
        &mut progress,
    )
}

/// Routes an existing placement with characterized timing costs and progress.
///
/// # Errors
///
/// Returns an invalid cost/constraint, model, or routability error.
pub fn route_with_timing_costs_and_progress(
    design: &Design,
    device: &Device,
    placement: Placement,
    routing_constraints: &RoutingConstraints,
    routing_costs: &RoutingCosts,
    mut progress: impl FnMut(RoutingProgress),
) -> Result<PnrResult, PnrError> {
    finish_routing(
        &UnifiedGraph::new(design, device),
        placement,
        routing_constraints,
        Some(routing_costs),
        &mut progress,
    )
}

fn finish_routing(
    graph: &UnifiedGraph<'_>,
    placement: Placement,
    routing_constraints: &RoutingConstraints,
    routing_costs: Option<&RoutingCosts>,
    progress: &mut impl FnMut(RoutingProgress),
) -> Result<PnrResult, PnrError> {
    validate_routing_constraints(graph, &placement, routing_constraints)?;
    validate_routing_costs(graph, routing_costs)?;
    let routes = route(
        graph,
        &placement,
        routing_constraints,
        routing_costs,
        progress,
    )?;
    for route in &routes {
        if route.pips.len().saturating_add(1) != route.wires.len() {
            return Err(PnrError::RouteIsNotTree {
                net: route.net,
                wires: route.wires.len(),
                pips: route.pips.len(),
            });
        }
    }
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
    place(&UnifiedGraph::new(design, device), constraints, None, None)
}

/// Places a design with deterministic per-net timing weights.
///
/// A missing net has weight one. Larger values make Manhattan distance on
/// that net more expensive during both initial placement and refinement.
///
/// # Errors
///
/// Returns a descriptive constraint, model, or BEL-exhaustion error.
pub fn place_with_net_weights(
    design: &Design,
    device: &Device,
    constraints: &PlacementConstraints,
    net_weights: &BTreeMap<NetId, u64>,
) -> Result<Placement, PnrError> {
    place(
        &UnifiedGraph::new(design, device),
        constraints,
        Some(net_weights),
        None,
    )
}

/// Places a design with deterministic per-net-sink timing weights.
///
/// A missing sink has weight one. Unlike a per-net weight, this lets timing
/// feedback pull one critical fanout toward its driver without also pulling
/// every non-critical fanout of the same logical net.
///
/// # Errors
///
/// Returns a descriptive constraint, model, or BEL-exhaustion error.
pub fn place_with_net_sink_weights(
    design: &Design,
    device: &Device,
    constraints: &PlacementConstraints,
    sink_weights: &BTreeMap<(NetId, CellPinId), u64>,
) -> Result<Placement, PnrError> {
    place(
        &UnifiedGraph::new(design, device),
        constraints,
        None,
        Some(sink_weights),
    )
}

/// Places a design by solving a sparse quadratic connectivity objective and
/// deterministically legalizing placement units onto compatible BELs.
///
/// Fixed units act as boundary conditions. Per-sink timing weights strengthen
/// only the corresponding logical edge. The solve is deterministic and does
/// not use random seeds.
///
/// # Errors
///
/// Returns a descriptive constraint, model, or BEL-exhaustion error.
pub fn place_analytically_with_net_sink_weights(
    design: &Design,
    device: &Device,
    constraints: &PlacementConstraints,
    sink_weights: &BTreeMap<(NetId, CellPinId), u64>,
) -> Result<Placement, PnrError> {
    analytical_place(
        &UnifiedGraph::new(design, device),
        constraints,
        sink_weights,
    )
}

/// Refines an existing legal placement with deterministic per-net timing weights.
///
/// Unlike [`place_with_net_weights`], this preserves the current solution as
/// the starting point and accepts only moves that reduce the weighted graph
/// objective. Atomic placement groups remain intact.
///
/// # Errors
///
/// Returns a descriptive error when the supplied placement does not match the
/// design, device, or placement constraints.
pub fn refine_placement_with_net_weights(
    design: &Design,
    device: &Device,
    constraints: &PlacementConstraints,
    placement: Placement,
    net_weights: &BTreeMap<NetId, u64>,
) -> Result<Placement, PnrError> {
    let graph = UnifiedGraph::new(design, device);
    let (_, neighbors) = placement_neighbors(design, Some(net_weights), None);
    let mut candidate_cache = BTreeMap::new();
    let units = placement_units(&graph, constraints, &mut candidate_cache)?;
    let mut placed = validate_refinement_start(&graph, &units, placement)?;
    let mut occupied = placed.iter().copied().flatten().collect::<BTreeSet<_>>();
    refine_placement(
        &graph,
        constraints,
        &units,
        &neighbors,
        &mut placed,
        &mut occupied,
        None,
    );
    finish_placement(&graph, constraints, placed)
}

/// Refines an existing legal placement with per-net-sink timing weights.
///
/// Only the specified driver-to-sink edges receive the larger timing cost, so
/// a high-fanout net does not collapse all of its sinks around the driver when
/// only one sink is timing-critical.
///
/// # Errors
///
/// Returns a descriptive error when the supplied placement does not match the
/// design, device, or placement constraints.
pub fn refine_placement_with_net_sink_weights(
    design: &Design,
    device: &Device,
    constraints: &PlacementConstraints,
    placement: Placement,
    sink_weights: &BTreeMap<(NetId, CellPinId), u64>,
) -> Result<Placement, PnrError> {
    let graph = UnifiedGraph::new(design, device);
    let (_, neighbors) = placement_neighbors(design, None, Some(sink_weights));
    let mut candidate_cache = BTreeMap::new();
    let units = placement_units(&graph, constraints, &mut candidate_cache)?;
    let mut placed = validate_refinement_start(&graph, &units, placement)?;
    let mut occupied = placed.iter().copied().flatten().collect::<BTreeSet<_>>();
    refine_placement(
        &graph,
        constraints,
        &units,
        &neighbors,
        &mut placed,
        &mut occupied,
        None,
    );
    finish_placement(&graph, constraints, placed)
}

/// Refines at most `max_moved_units` placement units using per-sink weights.
///
/// Units are considered in deterministic criticality order. Bounding accepted
/// moves keeps incremental routing local instead of invalidating the complete
/// design after every timing update.
///
/// # Errors
///
/// Returns a descriptive error when the supplied placement does not match the
/// design, device, or placement constraints.
pub fn refine_placement_with_net_sink_weights_limited(
    design: &Design,
    device: &Device,
    constraints: &PlacementConstraints,
    placement: Placement,
    sink_weights: &BTreeMap<(NetId, CellPinId), u64>,
    max_moved_units: usize,
) -> Result<Placement, PnrError> {
    PlacementRefiner::new(design, device, constraints)?.refine_with_net_sink_weights_limited(
        placement,
        sink_weights,
        max_moved_units,
    )
}

/// Reusable legal-placement problem for iterative timing refinement.
///
/// Compatible BEL assignments are independent of timing weights and are
/// expensive to enumerate on large devices. This object builds them once and
/// reuses them across deterministic STA/refinement generations.
pub struct PlacementRefiner<'a> {
    graph: UnifiedGraph<'a>,
    constraints: &'a PlacementConstraints,
    units: Vec<PlacementUnit>,
}

impl<'a> PlacementRefiner<'a> {
    /// Builds and caches all legal placement-unit assignments.
    ///
    /// # Errors
    ///
    /// Returns an error if a placement group or candidate binding is invalid.
    pub fn new(
        design: &'a Design,
        device: &'a Device,
        constraints: &'a PlacementConstraints,
    ) -> Result<Self, PnrError> {
        let graph = UnifiedGraph::new(design, device);
        let mut candidate_cache = BTreeMap::new();
        let units = placement_units(&graph, constraints, &mut candidate_cache)?;
        Ok(Self {
            graph,
            constraints,
            units,
        })
    }

    /// Refines a legal placement while moving at most the requested units.
    ///
    /// # Errors
    ///
    /// Returns an error if the starting placement is incompatible with the
    /// cached problem.
    pub fn refine_with_net_sink_weights_limited(
        &self,
        placement: Placement,
        sink_weights: &BTreeMap<(NetId, CellPinId), u64>,
        max_moved_units: usize,
    ) -> Result<Placement, PnrError> {
        let (_, neighbors) = placement_neighbors(self.graph.design(), None, Some(sink_weights));
        let mut placed = validate_refinement_start(&self.graph, &self.units, placement)?;
        let mut occupied = placed.iter().copied().flatten().collect::<BTreeSet<_>>();
        refine_placement(
            &self.graph,
            self.constraints,
            &self.units,
            &neighbors,
            &mut placed,
            &mut occupied,
            Some(max_moved_units),
        );
        finish_placement(&self.graph, self.constraints, placed)
    }

    /// Moves one endpoint placement unit to an unoccupied nearby assignment
    /// only when the characterized unloaded connection gets shorter.
    ///
    /// This is a deterministic detailed-placement primitive. The caller must
    /// reroute and run STA before accepting the proposal because congestion
    /// and other connections of the moved unit are deliberately excluded.
    ///
    /// # Errors
    ///
    /// Returns an error when the starting placement or timing table is invalid.
    ///
    /// # Panics
    ///
    /// Panics only if an internally validated placement-unit table is
    /// inconsistent with the design.
    #[allow(clippy::too_many_lines)]
    pub fn refine_connection_delay(
        &self,
        placement: Placement,
        driver_pin: CellPinId,
        sink_pin: CellPinId,
        move_driver: bool,
        pip_delays_ps: &[u32],
        max_move_distance: u64,
    ) -> Result<Option<Placement>, PnrError> {
        if pip_delays_ps.len() != self.graph.device().pips().len() {
            return Err(PnrError::InvalidRoutingCosts {
                reason: format!(
                    "expected {} PIP delays, received {}",
                    self.graph.device().pips().len(),
                    pip_delays_ps.len()
                ),
            });
        }
        let design = self.graph.design();
        let device = self.graph.device();
        let driver_cell = design
            .pins()
            .get(driver_pin.0)
            .ok_or_else(|| PnrError::InvalidPlacement {
                reason: format!("connection driver pin {} does not exist", driver_pin.0),
            })?
            .cell;
        let sink_cell = design
            .pins()
            .get(sink_pin.0)
            .ok_or_else(|| PnrError::InvalidPlacement {
                reason: format!("connection sink pin {} does not exist", sink_pin.0),
            })?
            .cell;
        let (moving_cell, moving_pin, fixed_cell, fixed_pin) = if move_driver {
            (driver_cell, driver_pin, sink_cell, sink_pin)
        } else {
            (sink_cell, sink_pin, driver_cell, driver_pin)
        };
        let Some(unit) = self
            .units
            .iter()
            .find(|unit| unit.cells.contains(&moving_cell))
        else {
            return Ok(None);
        };
        if unit.cells.contains(&fixed_cell) || unit.choices.len() <= 1 {
            return Ok(None);
        }
        let mut placed = validate_refinement_start(&self.graph, &self.units, placement)?;
        let current = unit
            .cells
            .iter()
            .map(|cell| placed[cell.0].expect("validated placement is complete"))
            .collect::<Vec<_>>();
        let moving_column = unit
            .cells
            .iter()
            .position(|&cell| cell == moving_cell)
            .expect("the selected unit contains its moving endpoint");
        let fixed_bel = placed[fixed_cell.0].expect("validated placement is complete");
        let fixed_wire = candidate_pin_wire(&self.graph, self.constraints, fixed_pin, fixed_bel)
            .ok_or_else(|| PnrError::InvalidPlacement {
                reason: format!("fixed pin {} has no physical wire", fixed_pin.0),
            })?;
        let current_moving_wire = candidate_pin_wire(
            &self.graph,
            self.constraints,
            moving_pin,
            current[moving_column],
        )
        .ok_or_else(|| PnrError::InvalidPlacement {
            reason: format!("moving pin {} has no physical wire", moving_pin.0),
        })?;
        let (current_start, current_goal) = if move_driver {
            (current_moving_wire, fixed_wire)
        } else {
            (fixed_wire, current_moving_wire)
        };
        let Some(current_delay) =
            local_connection_delay(&self.graph, current_start, current_goal, pip_delays_ps)
        else {
            return Ok(None);
        };

        let mut occupied = placed.iter().copied().flatten().collect::<BTreeSet<_>>();
        let mut pin_usage = HashMap::new();
        for known in &self.units {
            let assignment = known
                .cells
                .iter()
                .map(|cell| placed[cell.0].expect("validated placement is complete"))
                .collect::<Vec<_>>();
            update_pin_usage(
                &self.graph,
                self.constraints,
                &known.cells,
                &assignment,
                &mut pin_usage,
                true,
            );
        }
        for &bel in &current {
            occupied.remove(&bel);
        }
        for &cell in &unit.cells {
            placed[cell.0] = None;
        }
        update_pin_usage(
            &self.graph,
            self.constraints,
            &unit.cells,
            &current,
            &mut pin_usage,
            false,
        );
        let current_point = device.bels()[current[moving_column].0].point;
        let mut best: Option<(u64, Vec<BelId>)> = None;
        for choice in 0..unit.choices.len() {
            let assignment = unit.choices.assignment(choice);
            if assignment == current
                || assignment.iter().any(|bel| occupied.contains(bel))
                || device.bels()[assignment[moving_column].0]
                    .point
                    .manhattan(current_point)
                    > max_move_distance
                || !assignment_pin_wires_are_legal(
                    &self.graph,
                    self.constraints,
                    &unit.cells,
                    assignment,
                    &pin_usage,
                )
            {
                continue;
            }
            let Some(moving_wire) = candidate_pin_wire(
                &self.graph,
                self.constraints,
                moving_pin,
                assignment[moving_column],
            ) else {
                continue;
            };
            let (start, goal) = if move_driver {
                (moving_wire, fixed_wire)
            } else {
                (fixed_wire, moving_wire)
            };
            let Some(delay) = local_connection_delay(&self.graph, start, goal, pip_delays_ps)
            else {
                continue;
            };
            if delay < current_delay
                && best.as_ref().is_none_or(|(best_delay, best_assignment)| {
                    (delay, assignment) < (*best_delay, best_assignment.as_slice())
                })
            {
                best = Some((delay, assignment.to_vec()));
            }
        }
        let Some((_, selected)) = best else {
            return Ok(None);
        };
        for (&cell, &bel) in unit.cells.iter().zip(&selected) {
            placed[cell.0] = Some(bel);
        }
        Ok(Some(finish_placement(
            &self.graph,
            self.constraints,
            placed,
        )?))
    }

    /// Proposes placements for one cell's unit that reduce incident connection
    /// delay locally or physical path span during a broad critical-path move.
    ///
    /// # Errors
    ///
    /// Returns an error when the starting placement or timing table is invalid.
    ///
    /// # Panics
    ///
    /// Panics only if an internally validated placement-unit table is
    /// inconsistent with the design.
    #[allow(clippy::too_many_lines)]
    pub fn refine_cell_connection_delays(
        &self,
        placement: Placement,
        moving_cell: CellId,
        connections: &[(CellPinId, CellPinId)],
        pip_delays_ps: &[u32],
        max_move_distance: u64,
        max_candidates: usize,
    ) -> Result<Vec<Placement>, PnrError> {
        if pip_delays_ps.len() != self.graph.device().pips().len() {
            return Err(PnrError::InvalidRoutingCosts {
                reason: format!(
                    "expected {} PIP delays, received {}",
                    self.graph.device().pips().len(),
                    pip_delays_ps.len()
                ),
            });
        }
        let device = self.graph.device();
        let Some(unit) = self
            .units
            .iter()
            .find(|unit| unit.cells.contains(&moving_cell))
        else {
            return Ok(Vec::new());
        };
        if unit.choices.len() <= 1 || connections.is_empty() {
            return Ok(Vec::new());
        }
        let placed = validate_refinement_start(&self.graph, &self.units, placement)?;
        let current = unit
            .cells
            .iter()
            .map(|cell| placed[cell.0].expect("validated placement is complete"))
            .collect::<Vec<_>>();
        let moving_column = unit
            .cells
            .iter()
            .position(|&cell| cell == moving_cell)
            .expect("the selected unit contains its moving cell");
        let broad_path_move = max_move_distance > 2;
        let current_delay = if broad_path_move {
            None
        } else {
            assignment_connection_delay(
                &self.graph,
                self.constraints,
                unit,
                &current,
                moving_cell,
                connections,
                &placed,
                pip_delays_ps,
            )
        };
        let Some(current_span) = assignment_connection_span(
            &self.graph,
            unit,
            &current,
            moving_cell,
            connections,
            &placed,
        ) else {
            return Ok(Vec::new());
        };
        if !broad_path_move && current_delay.is_none() {
            return Ok(Vec::new());
        }

        let mut occupied = placed.iter().copied().flatten().collect::<BTreeSet<_>>();
        let mut pin_usage = HashMap::new();
        for known in &self.units {
            let assignment = known
                .cells
                .iter()
                .map(|cell| placed[cell.0].expect("validated placement is complete"))
                .collect::<Vec<_>>();
            update_pin_usage(
                &self.graph,
                self.constraints,
                &known.cells,
                &assignment,
                &mut pin_usage,
                true,
            );
        }
        for &bel in &current {
            occupied.remove(&bel);
        }
        update_pin_usage(
            &self.graph,
            self.constraints,
            &unit.cells,
            &current,
            &mut pin_usage,
            false,
        );
        let current_point = device.bels()[current[moving_column].0].point;
        let mut best = Vec::<(u64, u64, Vec<BelId>)>::new();
        for choice in 0..unit.choices.len() {
            let assignment = unit.choices.assignment(choice);
            if assignment == current
                || assignment.iter().any(|bel| occupied.contains(bel))
                || device.bels()[assignment[moving_column].0]
                    .point
                    .manhattan(current_point)
                    > max_move_distance
                || !assignment_pin_wires_are_legal(
                    &self.graph,
                    self.constraints,
                    &unit.cells,
                    assignment,
                    &pin_usage,
                )
            {
                continue;
            }
            let Some(span) = assignment_connection_span(
                &self.graph,
                unit,
                assignment,
                moving_cell,
                connections,
                &placed,
            ) else {
                continue;
            };
            let score = if broad_path_move {
                if span >= current_span {
                    continue;
                }
                (span, 0)
            } else {
                let Some(delay) = assignment_connection_delay(
                    &self.graph,
                    self.constraints,
                    unit,
                    assignment,
                    moving_cell,
                    connections,
                    &placed,
                    pip_delays_ps,
                ) else {
                    continue;
                };
                let current_delay = current_delay.expect("local moves require a current delay");
                if delay >= current_delay {
                    continue;
                }
                (delay, span)
            };
            best.push((score.0, score.1, assignment.to_vec()));
        }
        best.sort_unstable_by(|left, right| {
            (left.0, left.1, left.2.as_slice()).cmp(&(right.0, right.1, right.2.as_slice()))
        });
        best.dedup_by(|left, right| left.2 == right.2);
        best.truncate(max_candidates.max(1));
        let mut proposals = Vec::with_capacity(best.len());
        for (_, _, selected) in best {
            let mut proposed = placed.clone();
            for (&cell, &bel) in unit.cells.iter().zip(&selected) {
                proposed[cell.0] = Some(bel);
            }
            proposals.push(finish_placement(&self.graph, self.constraints, proposed)?);
        }
        Ok(proposals)
    }
}

fn assignment_connection_span(
    graph: &UnifiedGraph<'_>,
    unit: &PlacementUnit,
    assignment: &[BelId],
    moving_cell: CellId,
    connections: &[(CellPinId, CellPinId)],
    placed: &[Option<BelId>],
) -> Option<u64> {
    let design = graph.design();
    let device = graph.device();
    let mut span = 0_u64;
    for &(driver_pin, sink_pin) in connections {
        let driver_cell = design.pins().get(driver_pin.0)?.cell;
        let sink_cell = design.pins().get(sink_pin.0)?.cell;
        if driver_cell != moving_cell && sink_cell != moving_cell {
            return None;
        }
        let driver_bel = assignment_bel(unit, assignment, driver_cell, placed)?;
        let sink_bel = assignment_bel(unit, assignment, sink_cell, placed)?;
        let driver_point = device.bels().get(driver_bel.0)?.point;
        let sink_point = device.bels().get(sink_bel.0)?.point;
        span = span.saturating_add(driver_point.manhattan(sink_point));
    }
    Some(span)
}

fn assignment_bel(
    unit: &PlacementUnit,
    assignment: &[BelId],
    cell: CellId,
    placed: &[Option<BelId>],
) -> Option<BelId> {
    unit.cells
        .iter()
        .position(|&member| member == cell)
        .map_or_else(
            || placed.get(cell.0).copied().flatten(),
            |column| assignment.get(column).copied(),
        )
}

#[allow(clippy::too_many_arguments)]
fn assignment_connection_delay(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    unit: &PlacementUnit,
    assignment: &[BelId],
    moving_cell: CellId,
    connections: &[(CellPinId, CellPinId)],
    placed: &[Option<BelId>],
    pip_delays_ps: &[u32],
) -> Option<u64> {
    let design = graph.design();
    let mut total = 0_u64;
    for &(driver_pin, sink_pin) in connections {
        let driver_cell = design.pins().get(driver_pin.0)?.cell;
        let sink_cell = design.pins().get(sink_pin.0)?.cell;
        if driver_cell != moving_cell && sink_cell != moving_cell {
            return None;
        }
        let driver_bel = assignment_bel(unit, assignment, driver_cell, placed)?;
        let sink_bel = assignment_bel(unit, assignment, sink_cell, placed)?;
        let driver_wire = candidate_pin_wire(graph, constraints, driver_pin, driver_bel)?;
        let sink_wire = candidate_pin_wire(graph, constraints, sink_pin, sink_bel)?;
        total = total.saturating_add(local_connection_delay(
            graph,
            driver_wire,
            sink_wire,
            pip_delays_ps,
        )?);
    }
    Some(total)
}

fn local_connection_delay(
    graph: &UnifiedGraph<'_>,
    start: WireId,
    goal: WireId,
    pip_delays_ps: &[u32],
) -> Option<u64> {
    // Long-line entry/exit PIPs can put even a modest tile displacement over
    // eight graph edges.  A too-small bound made a badly displaced critical
    // vertex impossible to score, so the detailed placer could never move it
    // back toward its path.  The one-tile corridor keeps this search local.
    const MAX_LOCAL_HOPS: u8 = 16;
    const LOCAL_MARGIN: u32 = 1;
    let device = graph.device();
    let corridor = routing_corridor(
        device.wires()[start.0].point,
        device.wires()[goal.0].point,
        device,
        LOCAL_MARGIN,
    );
    let mut queue = BinaryHeap::from([Reverse((0_u64, 0_u8, start))]);
    let mut best = HashMap::from([((start, 0_u8), 0_u64)]);
    while let Some(Reverse((delay, hops, wire))) = queue.pop() {
        if wire == goal {
            return Some(delay);
        }
        if hops == MAX_LOCAL_HOPS || best.get(&(wire, hops)).is_some_and(|known| *known < delay) {
            continue;
        }
        for &(neighbor, pip) in graph.routing_neighbors(wire).ok()? {
            if !point_inside_corridor(device.wires()[neighbor.0].point, corridor) {
                continue;
            }
            let next_hops = hops + 1;
            let next_delay = delay.saturating_add(u64::from(pip_delays_ps[pip.0]));
            let key = (neighbor, next_hops);
            if best.get(&key).is_none_or(|known| next_delay < *known) {
                best.insert(key, next_delay);
                queue.push(Reverse((next_delay, next_hops, neighbor)));
            }
        }
    }
    None
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

impl PlacementChoices {
    fn len(&self) -> usize {
        match self {
            Self::Shared(assignments) => assignments.len(),
            Self::SingleCell(candidates) => candidates.len(),
        }
    }

    fn assignment(&self, index: usize) -> &[BelId] {
        match self {
            Self::Shared(assignments) => &assignments[index],
            Self::SingleCell(candidates) => std::slice::from_ref(&candidates[index]),
        }
    }

    fn cache_key(&self) -> (u8, usize) {
        match self {
            Self::Shared(assignments) => (0, Arc::as_ptr(assignments).cast::<()>() as usize),
            Self::SingleCell(candidates) => (1, Arc::as_ptr(candidates).cast::<()>() as usize),
        }
    }

    fn contains(&self, assignment: &[BelId]) -> bool {
        match self {
            Self::Shared(assignments) => assignments.iter().any(|known| known == assignment),
            Self::SingleCell(candidates) => {
                assignment.len() == 1 && candidates.contains(&assignment[0])
            }
        }
    }
}

#[derive(Clone, Debug)]
struct SpatialChoiceIndex {
    by_point: Vec<Vec<usize>>,
}

impl SpatialChoiceIndex {
    fn new(choices: &PlacementChoices, device: &Device) -> Self {
        let mut by_point = vec![Vec::new(); (device.width() * device.height()) as usize];
        for index in 0..choices.len() {
            let point = device.bels()[choices.assignment(index)[0].0].point;
            by_point[(point.y * device.width() + point.x) as usize].push(index);
        }
        Self { by_point }
    }
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct PlacementCandidateKey {
    kind: texo_model::ResourceKind,
    pins: Vec<(String, texo_model::PinDirection)>,
}

#[derive(Clone, Copy, Debug)]
struct PlacementNeighbor {
    cell: CellId,
    weight: u64,
    timing_driven: bool,
}

fn refinement_edge_cost(edge: PlacementNeighbor, distance: u64) -> u64 {
    let distance = if edge.timing_driven {
        distance.saturating_mul(distance)
    } else {
        distance
    };
    edge.weight.saturating_mul(distance)
}

fn place(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    net_weights: Option<&BTreeMap<NetId, u64>>,
    sink_weights: Option<&BTreeMap<(NetId, CellPinId), u64>>,
) -> Result<Placement, PnrError> {
    let design = graph.design();
    let device = graph.device();
    let (degree, neighbors) = placement_neighbors(design, net_weights, sink_weights);

    let mut candidate_cache = BTreeMap::new();
    let mut units = placement_units(graph, constraints, &mut candidate_cache)?;
    units.sort_by_key(|unit| {
        (
            unit.choices.len(),
            std::cmp::Reverse(unit.cells.iter().map(|cell| degree[cell.0]).sum::<usize>()),
            unit.cells[0],
        )
    });

    let mut placed = vec![None; design.cells().len()];
    let mut occupied = BTreeSet::new();
    for unit in &units {
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
        for (&cell, bel) in unit.cells.iter().zip(assignment) {
            occupied.insert(bel);
            placed[cell.0] = Some(bel);
        }
    }

    refine_placement(
        graph,
        constraints,
        &units,
        &neighbors,
        &mut placed,
        &mut occupied,
        None,
    );

    finish_placement(graph, constraints, placed)
}

#[allow(clippy::too_many_lines)]
fn analytical_place(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    sink_weights: &BTreeMap<(NetId, CellPinId), u64>,
) -> Result<Placement, PnrError> {
    const CENTER_WEIGHT: f64 = 0.01;
    let design = graph.design();
    let device = graph.device();
    let (_, neighbors) = placement_neighbors(design, None, Some(sink_weights));
    let mut candidate_cache = BTreeMap::new();
    let units = placement_units(graph, constraints, &mut candidate_cache)?;
    let mut unit_by_cell = vec![usize::MAX; design.cells().len()];
    for (unit_index, unit) in units.iter().enumerate() {
        for &cell in &unit.cells {
            unit_by_cell[cell.0] = unit_index;
        }
    }

    let fixed = units
        .iter()
        .map(|unit| {
            (unit.choices.len() == 1).then(|| device.bels()[unit.choices.assignment(0)[0].0].point)
        })
        .collect::<Vec<_>>();
    let mut edge_weights = BTreeMap::<(usize, usize), f64>::new();
    for (cell_index, edges) in neighbors.iter().enumerate() {
        let unit = unit_by_cell[cell_index];
        for edge in edges {
            let other = unit_by_cell[edge.cell.0];
            if unit >= other {
                continue;
            }
            let weight = u32::try_from(edge.weight).expect("placement edge weight fits u32");
            *edge_weights.entry((unit, other)).or_default() += f64::from(weight);
        }
    }

    let center = Point::new(device.width() / 2, device.height() / 2);
    let mut diagonal = vec![CENTER_WEIGHT; units.len()];
    let mut rhs_x = vec![CENTER_WEIGHT * f64::from(center.x); units.len()];
    let mut rhs_y = vec![CENTER_WEIGHT * f64::from(center.y); units.len()];
    let mut adjacency = vec![Vec::<(usize, f64)>::new(); units.len()];
    for ((left, right), weight) in edge_weights {
        match (fixed[left], fixed[right]) {
            (Some(_), Some(_)) => {}
            (Some(point), None) => {
                diagonal[right] += weight;
                rhs_x[right] += weight * f64::from(point.x);
                rhs_y[right] += weight * f64::from(point.y);
            }
            (None, Some(point)) => {
                diagonal[left] += weight;
                rhs_x[left] += weight * f64::from(point.x);
                rhs_y[left] += weight * f64::from(point.y);
            }
            (None, None) => {
                diagonal[left] += weight;
                diagonal[right] += weight;
                adjacency[left].push((right, weight));
                adjacency[right].push((left, weight));
            }
        }
    }
    let initial_x = vec![f64::from(center.x); units.len()];
    let initial_y = vec![f64::from(center.y); units.len()];
    let mut solved_x = solve_quadratic(&diagonal, &adjacency, &rhs_x, initial_x);
    let mut solved_y = solve_quadratic(&diagonal, &adjacency, &rhs_y, initial_y);
    for density_weight in [0.05, 0.10, 0.20, 0.40] {
        let (target_x, target_y) =
            analytic_spread_targets(&units, &fixed, device, solved_x.clone(), solved_y.clone());
        let mut spread_diagonal = diagonal.clone();
        let mut spread_rhs_x = rhs_x.clone();
        let mut spread_rhs_y = rhs_y.clone();
        for index in 0..units.len() {
            if fixed[index].is_some() {
                continue;
            }
            let anchor_weight = diagonal[index].max(1.0) * density_weight;
            spread_diagonal[index] += anchor_weight;
            spread_rhs_x[index] += anchor_weight * target_x[index];
            spread_rhs_y[index] += anchor_weight * target_y[index];
        }
        solved_x = solve_quadratic(&spread_diagonal, &adjacency, &spread_rhs_x, solved_x);
        solved_y = solve_quadratic(&spread_diagonal, &adjacency, &spread_rhs_y, solved_y);
    }

    let mut placed = vec![None; design.cells().len()];
    let mut occupied = BTreeSet::new();
    let mut pin_usage = HashMap::new();
    let mut point_usage = vec![0_usize; (device.width() * device.height()) as usize];
    for unit in units.iter().filter(|unit| unit.choices.len() == 1) {
        let assignment = unit.choices.assignment(0);
        if assignment.iter().any(|bel| occupied.contains(bel))
            || !assignment_pin_wires_are_legal(
                graph,
                constraints,
                &unit.cells,
                assignment,
                &pin_usage,
            )
        {
            return Err(PnrError::InvalidPlacement {
                reason: format!(
                    "fixed unit beginning at cell {} is not legal",
                    unit.cells[0].0
                ),
            });
        }
        install_assignment(
            graph,
            constraints,
            unit,
            assignment,
            &mut placed,
            &mut occupied,
            &mut pin_usage,
        );
        update_point_usage(device, assignment, &mut point_usage);
    }

    let mut order = units
        .iter()
        .enumerate()
        .filter(|(_, unit)| unit.choices.len() > 1)
        .map(|(index, unit)| {
            let criticality = unit
                .cells
                .iter()
                .flat_map(|cell| &neighbors[cell.0])
                .map(|edge| edge.weight)
                .sum::<u64>();
            (index, Reverse(criticality), unit.cells[0])
        })
        .collect::<Vec<_>>();
    order.sort_by_key(|&(_, criticality, cell)| (criticality, cell));
    let mut spatial_indexes = BTreeMap::new();
    for (index, _, _) in order {
        let unit = &units[index];
        let target = Point::new(
            rounded_coordinate(solved_x[index], device.width()),
            rounded_coordinate(solved_y[index], device.height()),
        );
        let spatial_index = spatial_indexes
            .entry(unit.choices.cache_key())
            .or_insert_with(|| SpatialChoiceIndex::new(&unit.choices, device));
        let assignment_index = nearest_legal_assignments_with_density(
            unit,
            spatial_index,
            graph,
            constraints,
            target,
            &occupied,
            &pin_usage,
            &point_usage,
        )
        .into_iter()
        .min_by_key(|&choice| {
            let point = device.bels()[unit.choices.assignment(choice)[0].0].point;
            (point.manhattan(target), point, choice)
        })
        .ok_or_else(|| PnrError::NoBel {
            cell: design.cells()[unit.cells[0].0].name.clone(),
        })?;
        let assignment = unit.choices.assignment(assignment_index);
        install_assignment(
            graph,
            constraints,
            unit,
            assignment,
            &mut placed,
            &mut occupied,
            &mut pin_usage,
        );
        update_point_usage(device, assignment, &mut point_usage);
    }
    finish_placement(graph, constraints, placed)
}

#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn rounded_coordinate(value: f64, extent: u32) -> u32 {
    value.round().clamp(0.0, f64::from(extent - 1)) as u32
}

fn analytic_spread_targets(
    units: &[PlacementUnit],
    fixed: &[Option<Point>],
    device: &Device,
    mut x: Vec<f64>,
    mut y: Vec<f64>,
) -> (Vec<f64>, Vec<f64>) {
    let mut movable = units
        .iter()
        .enumerate()
        .filter_map(|(index, _)| fixed[index].is_none().then_some(index))
        .collect::<Vec<_>>();
    if movable.is_empty() {
        return (x, y);
    }
    let count = u32::try_from(movable.len()).expect("placement unit count fits u32");
    let aspect = f64::from(device.width()) / f64::from(device.height());
    let columns = ceil_coordinate((f64::from(count) * aspect).sqrt()).clamp(1, device.width());
    let rows = count.div_ceil(columns).clamp(1, device.height());
    let mean_x = movable.iter().map(|&index| x[index]).sum::<f64>() / f64::from(count);
    let mean_y = movable.iter().map(|&index| y[index]).sum::<f64>() / f64::from(count);
    let start_x = rounded_coordinate(mean_x - f64::from(columns) / 2.0, device.width())
        .min(device.width() - columns);
    let start_y = rounded_coordinate(mean_y - f64::from(rows) / 2.0, device.height())
        .min(device.height() - rows);
    movable.sort_by(|&left, &right| {
        x[left]
            .total_cmp(&x[right])
            .then_with(|| y[left].total_cmp(&y[right]))
            .then_with(|| units[left].cells[0].cmp(&units[right].cells[0]))
    });
    for (column, chunk) in movable.chunks_mut(rows as usize).enumerate() {
        chunk.sort_by(|&left, &right| {
            y[left]
                .total_cmp(&y[right])
                .then_with(|| x[left].total_cmp(&x[right]))
                .then_with(|| units[left].cells[0].cmp(&units[right].cells[0]))
        });
        for (row, &index) in chunk.iter().enumerate() {
            let column = u32::try_from(column).expect("spread column fits u32");
            let row = u32::try_from(row).expect("spread row fits u32");
            x[index] = f64::from(start_x + column);
            y[index] = f64::from(start_y + row);
        }
    }
    (x, y)
}

#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn ceil_coordinate(value: f64) -> u32 {
    value.ceil() as u32
}

fn install_assignment(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    unit: &PlacementUnit,
    assignment: &[BelId],
    placed: &mut [Option<BelId>],
    occupied: &mut BTreeSet<BelId>,
    pin_usage: &mut HashMap<WireId, HashMap<NetId, usize>>,
) {
    for (&cell, &bel) in unit.cells.iter().zip(assignment) {
        occupied.insert(bel);
        placed[cell.0] = Some(bel);
    }
    update_pin_usage(graph, constraints, &unit.cells, assignment, pin_usage, true);
}

fn solve_quadratic(
    diagonal: &[f64],
    adjacency: &[Vec<(usize, f64)>],
    rhs: &[f64],
    mut solution: Vec<f64>,
) -> Vec<f64> {
    const MAX_ITERATIONS: usize = 100;
    const RELATIVE_TOLERANCE: f64 = 1e-8;
    let multiply = |values: &[f64]| {
        diagonal
            .iter()
            .zip(adjacency)
            .enumerate()
            .map(|(index, (&diagonal, edges))| {
                edges
                    .iter()
                    .fold(diagonal * values[index], |sum, &(other, weight)| {
                        sum - weight * values[other]
                    })
            })
            .collect::<Vec<_>>()
    };
    let product = multiply(&solution);
    let mut residual = rhs
        .iter()
        .zip(product)
        .map(|(&rhs, product)| rhs - product)
        .collect::<Vec<_>>();
    let mut direction = residual.clone();
    let mut residual_norm = dot(&residual, &residual);
    let initial_norm = residual_norm.max(f64::EPSILON);
    for _ in 0..MAX_ITERATIONS {
        if residual_norm <= initial_norm * RELATIVE_TOLERANCE {
            break;
        }
        let product = multiply(&direction);
        let denominator = dot(&direction, &product);
        if denominator <= f64::EPSILON {
            break;
        }
        let alpha = residual_norm / denominator;
        for ((solution, residual), (&direction, product)) in solution
            .iter_mut()
            .zip(&mut residual)
            .zip(direction.iter().zip(product))
        {
            *solution += alpha * direction;
            *residual -= alpha * product;
        }
        let next_norm = dot(&residual, &residual);
        let beta = next_norm / residual_norm;
        for (direction, &residual) in direction.iter_mut().zip(&residual) {
            *direction = residual + beta * *direction;
        }
        residual_norm = next_norm;
    }
    solution
}

fn dot(left: &[f64], right: &[f64]) -> f64 {
    left.iter()
        .zip(right)
        .map(|(left, right)| left * right)
        .sum()
}

fn finish_placement(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    placed: Vec<Option<BelId>>,
) -> Result<Placement, PnrError> {
    let design = graph.design();
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

fn validate_refinement_start(
    graph: &UnifiedGraph<'_>,
    units: &[PlacementUnit],
    placement: Placement,
) -> Result<Vec<Option<BelId>>, PnrError> {
    if placement.bindings.len() != graph.design().cells().len() {
        return Err(PnrError::InvalidPlacement {
            reason: format!(
                "expected {} cell bindings, received {}",
                graph.design().cells().len(),
                placement.bindings.len()
            ),
        });
    }
    let mut occupied = BTreeSet::new();
    for &bel in &placement.bindings {
        if bel.0 >= graph.device().bels().len() {
            return Err(PnrError::InvalidPlacement {
                reason: format!("binding names unknown BEL {}", bel.0),
            });
        }
        if !occupied.insert(bel) {
            return Err(PnrError::InvalidPlacement {
                reason: format!("BEL {} is assigned more than once", bel.0),
            });
        }
    }
    for unit in units {
        let assignment = unit
            .cells
            .iter()
            .map(|cell| placement.bindings[cell.0])
            .collect::<Vec<_>>();
        if !unit.choices.contains(&assignment) {
            return Err(PnrError::InvalidPlacement {
                reason: format!(
                    "cell group beginning at {} has an incompatible assignment",
                    unit.cells[0].0
                ),
            });
        }
    }
    Ok(placement.bindings.into_iter().map(Some).collect())
}

fn placement_neighbors(
    design: &Design,
    net_weights: Option<&BTreeMap<NetId, u64>>,
    sink_weights: Option<&BTreeMap<(NetId, CellPinId), u64>>,
) -> (Vec<usize>, Vec<Vec<PlacementNeighbor>>) {
    let mut degree = vec![0_usize; design.cells().len()];
    let mut neighbors = vec![Vec::new(); design.cells().len()];
    for (net_index, net) in design.nets().iter().enumerate() {
        let driver = design.pins()[net.driver.0].cell;
        let net_timing_weight = net_weights
            .and_then(|weights| weights.get(&NetId(net_index)))
            .copied()
            .unwrap_or(1);
        let fanout_weight = (64_u64 / net.sinks.len().max(1) as u64).max(1);
        for &sink_pin in &net.sinks {
            let timing_weight = sink_weights
                .and_then(|weights| weights.get(&(NetId(net_index), sink_pin)))
                .copied()
                .unwrap_or(net_timing_weight);
            let edge_weight = if design.cells()[driver.0].kind == texo_model::ResourceKind::Clock
                || net.sinks.len() > MAX_PLACEMENT_FANOUT
            {
                0
            } else {
                fanout_weight.saturating_mul(timing_weight)
            };
            let sink = design.pins()[sink_pin.0].cell;
            if driver != sink {
                degree[driver.0] += 1;
                degree[sink.0] += 1;
                if edge_weight != 0 {
                    neighbors[driver.0].push(PlacementNeighbor {
                        cell: sink,
                        weight: edge_weight,
                        timing_driven: timing_weight > 1,
                    });
                    neighbors[sink.0].push(PlacementNeighbor {
                        cell: driver,
                        weight: edge_weight,
                        timing_driven: timing_weight > 1,
                    });
                }
            }
        }
    }
    (degree, neighbors)
}

const MAX_PLACEMENT_FANOUT: usize = 256;

fn choose_assignment<'a>(
    cells: &[CellId],
    assignments: impl Iterator<Item = &'a [BelId]>,
    device: &Device,
    neighbors: &[Vec<PlacementNeighbor>],
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
                        .filter_map(|&edge| {
                            placed[edge.cell.0].map(|neighbor_bel| (neighbor_bel, edge.weight))
                        })
                        .map(|(neighbor_bel, weight)| {
                            weight * point.manhattan(device.bels()[neighbor_bel.0].point)
                        })
                        .sum::<u64>()
                })
                .sum::<u64>();
            let points = assignment
                .iter()
                .map(|bel| device.bels()[bel.0].point)
                .collect::<Vec<_>>();
            let center = Point::new(device.width() / 2, device.height() / 2);
            let center_distance = points
                .iter()
                .map(|point| point.manhattan(center))
                .sum::<u64>();
            (cost, center_distance, points, assignment)
        })
        .min()
        .map(|(_, _, _, assignment)| assignment.to_vec())
}

const MAX_PLACEMENT_REFINEMENT_PASSES: usize = 4;
const PLACEMENT_REFINEMENT_CANDIDATES: usize = 64;

#[allow(clippy::too_many_lines)]
fn refine_placement(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    units: &[PlacementUnit],
    neighbors: &[Vec<PlacementNeighbor>],
    placed: &mut [Option<BelId>],
    occupied: &mut BTreeSet<BelId>,
    move_limit: Option<usize>,
) {
    let device = graph.device();
    let mut pin_usage = HashMap::new();
    for unit in units {
        let assignment = unit
            .cells
            .iter()
            .map(|cell| placed[cell.0].expect("initial placement is complete"))
            .collect::<Vec<_>>();
        update_pin_usage(
            graph,
            constraints,
            &unit.cells,
            &assignment,
            &mut pin_usage,
            true,
        );
    }
    let mut spatial_indexes = BTreeMap::new();
    let mut order = (0..units.len()).collect::<Vec<_>>();
    order.sort_by_key(|&index| {
        let unit = &units[index];
        let maximum_weight = unit
            .cells
            .iter()
            .flat_map(|cell| &neighbors[cell.0])
            .map(|edge| edge.weight)
            .max()
            .unwrap_or(0);
        let total_weight = unit
            .cells
            .iter()
            .flat_map(|cell| &neighbors[cell.0])
            .map(|edge| edge.weight)
            .sum::<u64>();
        (
            Reverse(maximum_weight),
            Reverse(total_weight),
            unit.cells[0],
        )
    });

    for _ in 0..MAX_PLACEMENT_REFINEMENT_PASSES {
        let mut moved = 0;
        for &index in &order {
            let unit = &units[index];
            if unit.choices.len() <= 1 {
                continue;
            }
            let current = unit
                .cells
                .iter()
                .map(|cell| placed[cell.0].expect("initial placement is complete"))
                .collect::<Vec<_>>();
            let current_cost =
                assignment_wirelength(&unit.cells, &current, device, neighbors, placed);
            for &bel in &current {
                occupied.remove(&bel);
            }
            for &cell in &unit.cells {
                placed[cell.0] = None;
            }
            update_pin_usage(
                graph,
                constraints,
                &unit.cells,
                &current,
                &mut pin_usage,
                false,
            );
            let current_is_legal = assignment_pin_wires_are_legal(
                graph,
                constraints,
                &unit.cells,
                &current,
                &pin_usage,
            );
            let spatial_index = spatial_indexes
                .entry(unit.choices.cache_key())
                .or_insert_with(|| SpatialChoiceIndex::new(&unit.choices, device));
            let Some(best) = choose_refined_assignment(
                unit,
                spatial_index,
                graph,
                constraints,
                neighbors,
                placed,
                occupied,
                &pin_usage,
            ) else {
                for (&cell, &bel) in unit.cells.iter().zip(&current) {
                    occupied.insert(bel);
                    placed[cell.0] = Some(bel);
                }
                update_pin_usage(
                    graph,
                    constraints,
                    &unit.cells,
                    &current,
                    &mut pin_usage,
                    true,
                );
                continue;
            };
            let best_cost = assignment_wirelength(&unit.cells, &best, device, neighbors, placed);
            let selected = if !current_is_legal || best_cost < current_cost {
                moved += 1;
                best
            } else {
                current
            };
            for (&cell, &bel) in unit.cells.iter().zip(&selected) {
                occupied.insert(bel);
                placed[cell.0] = Some(bel);
            }
            update_pin_usage(
                graph,
                constraints,
                &unit.cells,
                &selected,
                &mut pin_usage,
                true,
            );
            if move_limit.is_some_and(|limit| moved >= limit) {
                break;
            }
        }
        if moved == 0 || move_limit.is_some_and(|limit| moved >= limit) {
            break;
        }
    }
}

fn refinement_target(
    unit: &PlacementUnit,
    device: &Device,
    neighbors: &[Vec<PlacementNeighbor>],
    placed: &[Option<BelId>],
) -> Option<Point> {
    let mut weighted_x = 0_u64;
    let mut weighted_y = 0_u64;
    let mut total_weight = 0_u64;
    for &cell in &unit.cells {
        for &edge in &neighbors[cell.0] {
            if unit.cells.contains(&edge.cell) {
                continue;
            }
            let Some(neighbor_bel) = placed[edge.cell.0] else {
                continue;
            };
            let point = device.bels()[neighbor_bel.0].point;
            weighted_x = weighted_x.saturating_add(u64::from(point.x) * edge.weight);
            weighted_y = weighted_y.saturating_add(u64::from(point.y) * edge.weight);
            total_weight = total_weight.saturating_add(edge.weight);
        }
    }
    (total_weight != 0).then(|| {
        Point::new(
            u32::try_from(weighted_x / total_weight).expect("device x coordinate fits u32"),
            u32::try_from(weighted_y / total_weight).expect("device y coordinate fits u32"),
        )
    })
}

#[allow(clippy::too_many_arguments)]
fn choose_refined_assignment(
    unit: &PlacementUnit,
    spatial_index: &SpatialChoiceIndex,
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    neighbors: &[Vec<PlacementNeighbor>],
    placed: &[Option<BelId>],
    occupied: &BTreeSet<BelId>,
    pin_usage: &HashMap<WireId, HashMap<NetId, usize>>,
) -> Option<Vec<BelId>> {
    let device = graph.device();
    let target = refinement_target(unit, device, neighbors, placed)?;
    let nearest = nearest_legal_assignments(
        unit,
        spatial_index,
        graph,
        constraints,
        target,
        occupied,
        pin_usage,
    );
    nearest
        .into_iter()
        .map(|index| {
            let assignment = unit.choices.assignment(index);
            (
                assignment_wirelength(&unit.cells, assignment, device, neighbors, placed),
                index,
            )
        })
        .min()
        .map(|(_, index)| unit.choices.assignment(index).to_vec())
}

fn nearest_legal_assignments(
    unit: &PlacementUnit,
    spatial_index: &SpatialChoiceIndex,
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    target: Point,
    occupied: &BTreeSet<BelId>,
    pin_usage: &HashMap<WireId, HashMap<NetId, usize>>,
) -> Vec<usize> {
    nearest_legal_assignments_impl(
        unit,
        spatial_index,
        graph,
        constraints,
        target,
        occupied,
        pin_usage,
        None,
    )
}

#[allow(clippy::too_many_arguments)]
fn nearest_legal_assignments_with_density(
    unit: &PlacementUnit,
    spatial_index: &SpatialChoiceIndex,
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    target: Point,
    occupied: &BTreeSet<BelId>,
    pin_usage: &HashMap<WireId, HashMap<NetId, usize>>,
    point_usage: &[usize],
) -> Vec<usize> {
    nearest_legal_assignments_impl(
        unit,
        spatial_index,
        graph,
        constraints,
        target,
        occupied,
        pin_usage,
        Some(point_usage),
    )
}

#[allow(clippy::too_many_arguments)]
fn nearest_legal_assignments_impl(
    unit: &PlacementUnit,
    spatial_index: &SpatialChoiceIndex,
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    target: Point,
    occupied: &BTreeSet<BelId>,
    pin_usage: &HashMap<WireId, HashMap<NetId, usize>>,
    point_usage: Option<&[usize]>,
) -> Vec<usize> {
    let device = graph.device();
    let mut nearest = Vec::new();
    let max_radius = device.width() + device.height();
    for radius in 0..max_radius {
        for y in 0..device.height() {
            let dy = y.abs_diff(target.y);
            if dy > radius {
                continue;
            }
            let dx = radius - dy;
            for (side, x) in [target.x.checked_sub(dx), target.x.checked_add(dx)]
                .into_iter()
                .flatten()
                .enumerate()
            {
                if x >= device.width() || (dx == 0 && side == 1) {
                    continue;
                }
                let bucket = &spatial_index.by_point[(y * device.width() + x) as usize];
                for &index in bucket {
                    let assignment = unit.choices.assignment(index);
                    if assignment.iter().all(|bel| !occupied.contains(bel))
                        && point_usage.is_none_or(|usage| {
                            density_allows_assignment(graph, unit, assignment, usage)
                        })
                        && assignment_pin_wires_are_legal(
                            graph,
                            constraints,
                            &unit.cells,
                            assignment,
                            pin_usage,
                        )
                    {
                        nearest.push(index);
                    }
                }
            }
        }
        if nearest.len() >= PLACEMENT_REFINEMENT_CANDIDATES {
            nearest.sort_unstable();
            nearest.truncate(PLACEMENT_REFINEMENT_CANDIDATES);
            break;
        }
    }
    nearest
}

const MAX_LOGIC_CELLS_PER_POINT: usize = 2;

fn density_allows_assignment(
    graph: &UnifiedGraph<'_>,
    unit: &PlacementUnit,
    assignment: &[BelId],
    point_usage: &[usize],
) -> bool {
    if unit
        .cells
        .iter()
        .any(|cell| graph.design().cells()[cell.0].kind != texo_model::ResourceKind::Logic)
    {
        return true;
    }
    let device = graph.device();
    let mut added = BTreeMap::<Point, usize>::new();
    for &bel in assignment {
        *added.entry(device.bels()[bel.0].point).or_default() += 1;
    }
    added.into_iter().all(|(point, count)| {
        let index = (point.y * device.width() + point.x) as usize;
        point_usage[index] + count <= MAX_LOGIC_CELLS_PER_POINT
    })
}

fn update_point_usage(device: &Device, assignment: &[BelId], point_usage: &mut [usize]) {
    for &bel in assignment {
        let point = device.bels()[bel.0].point;
        point_usage[(point.y * device.width() + point.x) as usize] += 1;
    }
}

fn assignment_pin_wires_are_legal(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    cells: &[CellId],
    assignment: &[BelId],
    usage: &HashMap<WireId, HashMap<NetId, usize>>,
) -> bool {
    let mut candidate = HashMap::<WireId, BTreeSet<NetId>>::new();
    for (wire, net) in assignment_pin_resources(graph, constraints, cells, assignment) {
        candidate.entry(wire).or_default().insert(net);
    }
    candidate.into_iter().all(|(wire, nets)| {
        let mut distinct = nets;
        if let Some(existing) = usage.get(&wire) {
            distinct.extend(existing.keys().copied());
        }
        distinct.len() <= usize::from(graph.device().wires()[wire.0].capacity)
    })
}

fn update_pin_usage(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    cells: &[CellId],
    assignment: &[BelId],
    usage: &mut HashMap<WireId, HashMap<NetId, usize>>,
    add: bool,
) {
    for (wire, net) in assignment_pin_resources(graph, constraints, cells, assignment) {
        if add {
            *usage.entry(wire).or_default().entry(net).or_default() += 1;
        } else {
            let remove_wire = {
                let nets = usage
                    .get_mut(&wire)
                    .expect("placed pin wire is present in usage");
                let count = nets
                    .get_mut(&net)
                    .expect("placed pin net is present in usage");
                *count -= 1;
                if *count == 0 {
                    nets.remove(&net);
                }
                nets.is_empty()
            };
            if remove_wire {
                usage.remove(&wire);
            }
        }
    }
}

fn assignment_pin_resources(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    cells: &[CellId],
    assignment: &[BelId],
) -> Vec<(WireId, NetId)> {
    cells
        .iter()
        .zip(assignment)
        .flat_map(|(&cell, &bel)| {
            graph.design().cells()[cell.0]
                .pins()
                .iter()
                .filter_map(move |&pin| {
                    let net = graph.design().pins()[pin.0].net()?;
                    let wire = candidate_pin_wire(graph, constraints, pin, bel)
                        .expect("placement candidate has every bound physical pin");
                    Some((wire, net))
                })
        })
        .collect()
}

fn candidate_pin_wire(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    pin: CellPinId,
    bel: BelId,
) -> Option<WireId> {
    if let Some(&bel_pin) = constraints.pin_bindings.get(&(pin, bel)) {
        return Some(graph.device().bel_pins()[bel_pin.0].wire);
    }
    if let Some(name) = constraints.pin_name_bindings.get(&pin) {
        return physical_pin_by_name(graph, pin, bel, name)
            .map(|bel_pin| graph.device().bel_pins()[bel_pin.0].wire);
    }
    graph.bound_wire(pin, bel).ok()
}

fn assignment_wirelength(
    cells: &[CellId],
    assignment: &[BelId],
    device: &Device,
    neighbors: &[Vec<PlacementNeighbor>],
    placed: &[Option<BelId>],
) -> u64 {
    cells
        .iter()
        .zip(assignment)
        .map(|(&cell, &bel)| {
            let point = device.bels()[bel.0].point;
            neighbors[cell.0]
                .iter()
                .filter(|edge| !cells.contains(&edge.cell))
                .filter_map(|&edge| {
                    placed[edge.cell.0].map(|neighbor_bel| {
                        refinement_edge_cost(
                            edge,
                            point.manhattan(device.bels()[neighbor_bel.0].point),
                        )
                    })
                })
                .sum::<u64>()
        })
        .sum()
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

fn validate_routing_constraints(
    graph: &UnifiedGraph<'_>,
    placement: &Placement,
    constraints: &RoutingConstraints,
) -> Result<(), PnrError> {
    let design = graph.design();
    let device = graph.device();
    for (&net_id, route) in constraints.routes() {
        let Some(net) = design.nets().get(net_id.0) else {
            return Err(PnrError::InvalidRoutingConstraint {
                net: net_id,
                reason: "net ID is outside the design".into(),
            });
        };
        if route.net != net_id {
            return Err(PnrError::InvalidRoutingConstraint {
                net: net_id,
                reason: "route key and route net differ".into(),
            });
        }
        let wires = route.wires.iter().copied().collect::<BTreeSet<_>>();
        if wires.len() != route.wires.len() {
            return Err(PnrError::InvalidRoutingConstraint {
                net: net_id,
                reason: "wire list contains duplicates".into(),
            });
        }
        if route.pips.iter().copied().collect::<BTreeSet<_>>().len() != route.pips.len() {
            return Err(PnrError::InvalidRoutingConstraint {
                net: net_id,
                reason: "PIP list contains duplicates".into(),
            });
        }
        for &wire in &route.wires {
            if wire.0 >= device.wires().len() {
                return Err(PnrError::InvalidRoutingConstraint {
                    net: net_id,
                    reason: format!("unknown wire {wire:?}"),
                });
            }
        }
        for &pip in &route.pips {
            let Some(pip_data) = device.pips().get(pip.0) else {
                return Err(PnrError::InvalidRoutingConstraint {
                    net: net_id,
                    reason: format!("unknown PIP {pip:?}"),
                });
            };
            if !wires.contains(&pip_data.from) || !wires.contains(&pip_data.to) {
                return Err(PnrError::InvalidRoutingConstraint {
                    net: net_id,
                    reason: format!("PIP {pip:?} has an endpoint outside the locked tree"),
                });
            }
        }
        let driver_cell = design.pins()[net.driver.0].cell;
        let driver_bel = placement
            .bel(driver_cell)
            .ok_or(PnrError::MissingPlacement { cell: driver_cell })?;
        let driver_wire = bound_wire(graph, placement, net.driver, driver_bel)?;
        if !wires.contains(&driver_wire) {
            return Err(PnrError::InvalidRoutingConstraint {
                net: net_id,
                reason: "locked tree does not contain the placed driver wire".into(),
            });
        }
        let mut outgoing = BTreeMap::<WireId, Vec<WireId>>::new();
        for &pip in &route.pips {
            let pip = &device.pips()[pip.0];
            outgoing.entry(pip.from).or_default().push(pip.to);
            if pip.bidirectional {
                outgoing.entry(pip.to).or_default().push(pip.from);
            }
        }
        let mut reachable = BTreeSet::from([driver_wire]);
        let mut pending = vec![driver_wire];
        while let Some(wire) = pending.pop() {
            for &next in outgoing.get(&wire).map_or(&[][..], Vec::as_slice) {
                if reachable.insert(next) {
                    pending.push(next);
                }
            }
        }
        if reachable != wires {
            return Err(PnrError::InvalidRoutingConstraint {
                net: net_id,
                reason: format!(
                    "locked tree has {} wires disconnected from its driver",
                    wires.len() - reachable.len()
                ),
            });
        }
    }
    Ok(())
}

fn validate_routing_costs(
    graph: &UnifiedGraph<'_>,
    costs: Option<&RoutingCosts>,
) -> Result<(), PnrError> {
    let Some(costs) = costs else {
        return Ok(());
    };
    if costs.pip_delays_ps.len() != graph.device().pips().len() {
        return Err(PnrError::InvalidRoutingCosts {
            reason: format!(
                "expected {} PIP delays, received {}",
                graph.device().pips().len(),
                costs.pip_delays_ps.len()
            ),
        });
    }
    if costs.pip_min_delays_ps.len() != graph.device().pips().len() {
        return Err(PnrError::InvalidRoutingCosts {
            reason: format!(
                "expected {} minimum PIP delays, received {}",
                graph.device().pips().len(),
                costs.pip_min_delays_ps.len()
            ),
        });
    }
    for (&net, &criticality) in &costs.net_criticalities {
        if net.0 >= graph.design().nets().len() {
            return Err(PnrError::InvalidRoutingCosts {
                reason: format!("criticality names unknown net {}", net.0),
            });
        }
        if !(1..=64).contains(&criticality) {
            return Err(PnrError::InvalidRoutingCosts {
                reason: format!("net {} criticality {criticality} is outside 1..=64", net.0),
            });
        }
    }
    for &net in &costs.detailed_timing_nets {
        if net.0 >= graph.design().nets().len() {
            return Err(PnrError::InvalidRoutingCosts {
                reason: format!("detailed timing names unknown net {}", net.0),
            });
        }
    }
    if costs.detailed_delay_quantum_ps == 0
        || costs.detailed_delay_quantum_ps > ROUTING_DELAY_QUANTUM_PS
    {
        return Err(PnrError::InvalidRoutingCosts {
            reason: format!(
                "detailed delay quantum {} ps is outside 1..={ROUTING_DELAY_QUANTUM_PS}",
                costs.detailed_delay_quantum_ps
            ),
        });
    }
    for (&(net, sink), &minimum_ps) in &costs.sink_min_delays_ps {
        let Some(net_data) = graph.design().nets().get(net.0) else {
            return Err(PnrError::InvalidRoutingCosts {
                reason: format!("minimum delay names unknown net {}", net.0),
            });
        };
        if !net_data.sinks.contains(&sink) {
            return Err(PnrError::InvalidRoutingCosts {
                reason: format!("pin {} is not a sink of net {}", sink.0, net.0),
            });
        }
        if minimum_ps == 0 {
            return Err(PnrError::InvalidRoutingCosts {
                reason: format!("net {} sink {} has a zero minimum delay", net.0, sink.0),
            });
        }
    }
    Ok(())
}

fn route_reaches_all_sinks(
    graph: &UnifiedGraph<'_>,
    placement: &Placement,
    route: &NetRoute,
) -> Result<bool, PnrError> {
    let net = &graph.design().nets()[route.net.0];
    let wires = route.wires.iter().copied().collect::<BTreeSet<_>>();
    for &sink in &net.sinks {
        let cell = graph.design().pins()[sink.0].cell;
        let bel = placement
            .bel(cell)
            .ok_or(PnrError::MissingPlacement { cell })?;
        if !wires.contains(&bound_wire(graph, placement, sink, bel)?) {
            return Ok(false);
        }
    }
    Ok(true)
}

fn route(
    graph: &UnifiedGraph<'_>,
    placement: &Placement,
    constraints: &RoutingConstraints,
    costs: Option<&RoutingCosts>,
    progress: &mut impl FnMut(RoutingProgress),
) -> Result<Vec<NetRoute>, PnrError> {
    let design = graph.design();
    let device = graph.device();
    let mut wire_occupancy = vec![0_u16; device.wires().len()];
    let mut pip_occupancy = vec![0_u16; device.pips().len()];
    let mut wire_history = vec![0_u32; device.wires().len()];
    let mut pip_history = vec![0_u32; device.pips().len()];
    let mut routes = vec![None; design.nets().len()];
    for (&net, route) in constraints.routes() {
        routes[net.0] = Some(route.clone());
        add_route(route, &mut wire_occupancy, &mut pip_occupancy);
    }
    let mut routing_order = (0..design.nets().len()).collect::<Vec<_>>();
    routing_order.sort_by_key(|&index| routing_order_key(design, constraints, costs, index));
    let mut dirty = BTreeSet::new();
    for index in 0..design.nets().len() {
        let complete = if let Some(route) = constraints.routes().get(&NetId(index)) {
            route_reaches_all_sinks(graph, placement, route)?
        } else {
            false
        };
        if !complete {
            dirty.insert(index);
        }
    }
    let mut search = RouteSearch::new(device.wires().len());
    for iteration in 0..MAX_ROUTING_ITERATIONS {
        let present_factor = 1_u32 << iteration.min(12);
        progress(RoutingProgress::Iteration {
            iteration,
            nets: dirty.len(),
        });
        for &index in &dirty {
            if let Some(previous) = routes[index].take() {
                remove_route(&previous, &mut wire_occupancy, &mut pip_occupancy);
            }
        }
        let mut ordinal = 0;
        for &index in &routing_order {
            if !dirty.contains(&index) {
                continue;
            }
            ordinal += 1;
            progress(RoutingProgress::Net {
                iteration,
                ordinal,
                total: dirty.len(),
                net: NetId(index),
            });
            let net_id = NetId(index);
            let route = route_net(
                graph,
                placement,
                constraints.routes().get(&net_id),
                net_id,
                &wire_occupancy,
                &pip_occupancy,
                &wire_history,
                &pip_history,
                present_factor,
                costs,
                &mut search,
            )?;
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
        dirty = congested_routes(device, &routes, &wire_occupancy, &pip_occupancy);
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

fn routing_order_key(
    design: &Design,
    constraints: &RoutingConstraints,
    costs: Option<&RoutingCosts>,
    index: usize,
) -> (bool, Reverse<u64>, Reverse<bool>, Reverse<usize>, usize) {
    let net = NetId(index);
    let criticality = costs
        .and_then(|costs| costs.net_criticalities.get(&net))
        .copied()
        .unwrap_or(0);
    let hold_constrained = costs.is_some_and(|costs| {
        costs
            .sink_min_delays_ps
            .keys()
            .any(|(candidate, _)| *candidate == net)
    });
    (
        !constraints.routes().contains_key(&net),
        Reverse(criticality),
        Reverse(hold_constrained),
        Reverse(design.nets()[index].sinks.len()),
        index,
    )
}

fn congested_routes(
    device: &Device,
    routes: &[Option<NetRoute>],
    wire_occupancy: &[u16],
    pip_occupancy: &[u16],
) -> BTreeSet<usize> {
    routes
        .iter()
        .enumerate()
        .filter_map(|(index, route)| {
            let route = route
                .as_ref()
                .expect("every net was routed before congestion analysis");
            (route
                .wires
                .iter()
                .any(|wire| wire_occupancy[wire.0] > device.wires()[wire.0].capacity)
                || route
                    .pips
                    .iter()
                    .any(|pip| pip_occupancy[pip.0] > device.pips()[pip.0].capacity))
            .then_some(index)
        })
        .collect()
}

#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn route_net(
    graph: &UnifiedGraph<'_>,
    placement: &Placement,
    fixed: Option<&NetRoute>,
    net_id: NetId,
    wire_occupancy: &[u16],
    pip_occupancy: &[u16],
    wire_history: &[u32],
    pip_history: &[u32],
    present_factor: u32,
    costs: Option<&RoutingCosts>,
    search: &mut RouteSearch,
) -> Result<NetRoute, PnrError> {
    let design = graph.design();
    let device = graph.device();
    let net = &design.nets()[net_id.0];
    let driver_cell = design.pins()[net.driver.0].cell;
    let driver_bel = placement
        .bel(driver_cell)
        .ok_or(PnrError::MissingPlacement { cell: driver_cell })?;
    let driver_wire = bound_wire(graph, placement, net.driver, driver_bel)?;
    let mut tree_wires =
        fixed.map_or_else(BTreeSet::new, |route| route.wires.iter().copied().collect());
    tree_wires.insert(driver_wire);
    let mut tree_pips =
        fixed.map_or_else(BTreeSet::new, |route| route.pips.iter().copied().collect());
    let criticality = routing_criticality(costs, net_id);
    let delay_quantum_ps = costs.map_or(ROUTING_DELAY_QUANTUM_PS, |costs| {
        if costs.detailed_timing_nets.contains(&net_id) {
            costs.detailed_delay_quantum_ps
        } else {
            ROUTING_DELAY_QUANTUM_PS
        }
    });
    let mut tree_delays_ps = tree_wires
        .iter()
        .copied()
        .map(|wire| (wire, 0_u64))
        .collect::<BTreeMap<_, _>>();
    let sinks = ordered_sinks(net_id, &net.sinks, costs);
    for sink_pin in &sinks {
        let sink_cell = design.pins()[sink_pin.0].cell;
        let sink_bel = placement
            .bel(sink_cell)
            .ok_or(PnrError::MissingPlacement { cell: sink_cell })?;
        let sink_wire = bound_wire(graph, placement, *sink_pin, sink_bel)?;
        let minimum_arrival_ps = costs
            .and_then(|costs| costs.sink_min_delays_ps.get(&(net_id, *sink_pin)))
            .copied()
            .unwrap_or(0);
        if tree_wires.contains(&sink_wire) {
            if tree_delays_ps.get(&sink_wire).copied().unwrap_or(0) >= minimum_arrival_ps {
                continue;
            }
            return Err(PnrError::Unroutable {
                net: net.name.clone(),
                driver: format!(
                    "hold-constrained tree via {}",
                    device.wires()[driver_wire.0].name
                ),
                sink: format!(
                    "{}.{} requires at least {minimum_arrival_ps} ps",
                    design.cells()[sink_cell.0].name,
                    design.pins()[sink_pin.0].name
                ),
            });
        }
        let (path_wires, path_pips) = search
            .shortest_path(
                graph,
                &tree_wires,
                sink_wire,
                wire_occupancy,
                pip_occupancy,
                wire_history,
                pip_history,
                present_factor,
                costs,
                criticality,
                delay_quantum_ps,
                &tree_delays_ps,
                minimum_arrival_ps,
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
        if let Some(costs) = costs {
            let mut arrival_ps = tree_delays_ps[path_wires
                .last()
                .expect("a routed path includes its tree start")];
            for (&wire, &pip) in path_wires.iter().rev().skip(1).zip(path_pips.iter().rev()) {
                let delay_ps = routed_tree_pip_delay(costs, pip, minimum_arrival_ps);
                arrival_ps = arrival_ps.saturating_add(u64::from(delay_ps));
                tree_delays_ps
                    .entry(wire)
                    .and_modify(|known| *known = (*known).min(arrival_ps))
                    .or_insert(arrival_ps);
            }
        }
        tree_wires.extend(path_wires);
        tree_pips.extend(path_pips);
    }
    Ok(NetRoute {
        net: net_id,
        wires: tree_wires.into_iter().collect(),
        pips: tree_pips.into_iter().collect(),
    })
}

fn ordered_sinks(net: NetId, sinks: &[CellPinId], costs: Option<&RoutingCosts>) -> Vec<CellPinId> {
    let mut ordered = sinks.to_vec();
    ordered.sort_by_key(|&sink| {
        let minimum = costs
            .and_then(|costs| costs.sink_min_delays_ps.get(&(net, sink)))
            .copied()
            .unwrap_or(0);
        (Reverse(minimum), sink)
    });
    ordered
}

fn routing_criticality(costs: Option<&RoutingCosts>, net: NetId) -> u64 {
    costs
        .and_then(|costs| costs.net_criticalities.get(&net))
        .copied()
        .unwrap_or(0)
}

fn routed_tree_pip_delay(costs: &RoutingCosts, pip: PipId, minimum_arrival_ps: u64) -> u32 {
    if minimum_arrival_ps == 0 {
        costs.pip_delays_ps[pip.0]
    } else {
        costs.pip_min_delays_ps[pip.0]
    }
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
    arrival_ps: Vec<u64>,
    previous_wire: Vec<usize>,
    previous_pip: Vec<usize>,
}

type HoldRouteState = (WireId, u32);
type HoldRouteVisit = (u64, u64, Option<(HoldRouteState, PipId)>);

impl RouteSearch {
    fn new(wire_count: usize) -> Self {
        Self {
            epoch: 0,
            seen: vec![0; wire_count],
            distance: vec![0; wire_count],
            arrival_ps: vec![0; wire_count],
            previous_wire: vec![usize::MAX; wire_count],
            previous_pip: vec![usize::MAX; wire_count],
        }
    }

    #[allow(clippy::too_many_arguments, clippy::too_many_lines)]
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
        costs: Option<&RoutingCosts>,
        criticality: u64,
        delay_quantum_ps: u64,
        tree_delays_ps: &BTreeMap<WireId, u64>,
        minimum_arrival_ps: u64,
    ) -> Option<(Vec<WireId>, Vec<PipId>)> {
        if minimum_arrival_ps != 0 {
            return shortest_hold_path(
                graph,
                starts,
                goal,
                wire_occupancy,
                pip_occupancy,
                wire_history,
                pip_history,
                present_factor,
                costs?,
                criticality,
                tree_delays_ps,
                minimum_arrival_ps,
            );
        }
        self.epoch = self.epoch.wrapping_add(1);
        if self.epoch == 0 {
            self.seen.fill(0);
            self.epoch = 1;
        }
        let epoch = self.epoch;
        let device = graph.device();
        let goal_point = device.wires()[goal.0].point;
        let corridor = (criticality > 1).then(|| {
            let start_point = starts
                .iter()
                .map(|start| device.wires()[start.0].point)
                .min_by_key(|point| (point.manhattan(goal_point), *point))
                .expect("a route tree always contains its driver");
            routing_corridor(start_point, goal_point, device, TIMING_ROUTE_MARGIN)
        });
        let mut queue = BinaryHeap::new();
        for &start in starts {
            let arrival_ps = tree_delays_ps.get(&start).copied().unwrap_or(0);
            let distance = timing_tree_cost(arrival_ps, criticality, delay_quantum_ps);
            self.seen[start.0] = epoch;
            self.distance[start.0] = distance;
            self.arrival_ps[start.0] = arrival_ps;
            self.previous_wire[start.0] = usize::MAX;
            self.previous_pip[start.0] = usize::MAX;
            queue.push(Reverse((
                distance.saturating_add(device.wires()[start.0].point.manhattan(goal_point)),
                distance,
                arrival_ps,
                start,
            )));
        }

        while let Some(Reverse((_, distance, arrival_ps, wire))) = queue.pop() {
            if self.seen[wire.0] != epoch
                || (self.distance[wire.0], self.arrival_ps[wire.0]) != (distance, arrival_ps)
            {
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

            for &(neighbor, pip) in graph.routing_neighbors(wire).ok()? {
                if starts.contains(&neighbor) {
                    continue;
                }
                if corridor.is_some_and(|corridor| {
                    !point_inside_corridor(device.wires()[neighbor.0].point, corridor)
                }) {
                    continue;
                }
                let congestion = congestion_cost(
                    wire_occupancy[neighbor.0],
                    device.wires()[neighbor.0].capacity,
                    wire_history[neighbor.0],
                    present_factor,
                ) + congestion_cost(
                    pip_occupancy[pip.0],
                    device.pips()[pip.0].capacity,
                    pip_history[pip.0],
                    present_factor,
                );
                let pip_delay_ps = costs.map_or(0, |costs| costs.pip_delays_ps[pip.0]);
                let next_arrival_ps = arrival_ps.saturating_add(u64::from(pip_delay_ps));
                let step = if delay_quantum_ps == ROUTING_DELAY_QUANTUM_PS {
                    routing_step_cost(pip_delay_ps, criticality, congestion, delay_quantum_ps)
                } else {
                    routing_transition_cost(
                        arrival_ps,
                        next_arrival_ps,
                        criticality,
                        congestion,
                        delay_quantum_ps,
                    )
                };
                let next_distance = distance.saturating_add(step);
                if neighbor == goal && next_arrival_ps < minimum_arrival_ps {
                    continue;
                }
                if self.seen[neighbor.0] == epoch
                    && (self.distance[neighbor.0], self.arrival_ps[neighbor.0])
                        <= (next_distance, next_arrival_ps)
                {
                    continue;
                }
                self.seen[neighbor.0] = epoch;
                self.distance[neighbor.0] = next_distance;
                self.arrival_ps[neighbor.0] = next_arrival_ps;
                self.previous_wire[neighbor.0] = wire.0;
                self.previous_pip[neighbor.0] = pip.0;
                let estimate = next_distance
                    .saturating_add(device.wires()[neighbor.0].point.manhattan(goal_point));
                queue.push(Reverse((
                    estimate,
                    next_distance,
                    next_arrival_ps,
                    neighbor,
                )));
            }
        }
        if corridor.is_some() {
            return self.shortest_path(
                graph,
                starts,
                goal,
                wire_occupancy,
                pip_occupancy,
                wire_history,
                pip_history,
                present_factor,
                None,
                0,
                ROUTING_DELAY_QUANTUM_PS,
                tree_delays_ps,
                minimum_arrival_ps,
            );
        }
        None
    }
}

type RoutingCorridor = (u32, u32, u32, u32);

const TIMING_ROUTE_MARGIN: u32 = 12;

fn routing_corridor(start: Point, goal: Point, device: &Device, margin: u32) -> RoutingCorridor {
    (
        start.x.min(goal.x).saturating_sub(margin),
        start
            .x
            .max(goal.x)
            .saturating_add(margin)
            .min(device.width() - 1),
        start.y.min(goal.y).saturating_sub(margin),
        start
            .y
            .max(goal.y)
            .saturating_add(margin)
            .min(device.height() - 1),
    )
}

fn point_inside_corridor(point: Point, corridor: RoutingCorridor) -> bool {
    let (minimum_x, maximum_x, minimum_y, maximum_y) = corridor;
    (minimum_x..=maximum_x).contains(&point.x) && (minimum_y..=maximum_y).contains(&point.y)
}

#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn shortest_hold_path(
    graph: &UnifiedGraph<'_>,
    starts: &BTreeSet<WireId>,
    goal: WireId,
    wire_occupancy: &[u16],
    pip_occupancy: &[u16],
    wire_history: &[u32],
    pip_history: &[u32],
    present_factor: u32,
    costs: &RoutingCosts,
    criticality: u64,
    tree_delays_ps: &BTreeMap<WireId, u64>,
    minimum_arrival_ps: u64,
) -> Option<(Vec<WireId>, Vec<PipId>)> {
    let device = graph.device();
    let goal_point = device.wires()[goal.0].point;
    let mut visits = HashMap::<HoldRouteState, HoldRouteVisit>::new();
    let mut queue = BinaryHeap::new();
    for &start in starts {
        let arrival_ps = tree_delays_ps.get(&start).copied().unwrap_or(0);
        let state = (start, hold_delay_bucket(arrival_ps, minimum_arrival_ps));
        let distance = timing_tree_cost(arrival_ps, criticality, ROUTING_DELAY_QUANTUM_PS);
        visits.insert(state, (distance, arrival_ps, None));
        queue.push(Reverse((
            distance.saturating_add(device.wires()[start.0].point.manhattan(goal_point)),
            distance,
            arrival_ps,
            state,
        )));
    }

    while let Some(Reverse((_, distance, arrival_ps, state))) = queue.pop() {
        let (wire, _) = state;
        let Some(&(known_distance, known_arrival, _)) = visits.get(&state) else {
            continue;
        };
        if (known_distance, known_arrival) != (distance, arrival_ps) {
            continue;
        }
        if wire == goal {
            if arrival_ps >= minimum_arrival_ps {
                let path = reconstruct_hold_path(state, &visits);
                if path.0.iter().copied().collect::<BTreeSet<_>>().len() == path.0.len() {
                    return Some(path);
                }
            }
            continue;
        }

        for &(neighbor, pip) in graph.routing_neighbors(wire).ok()? {
            if starts.contains(&neighbor) {
                continue;
            }
            let congestion = congestion_cost(
                wire_occupancy[neighbor.0],
                device.wires()[neighbor.0].capacity,
                wire_history[neighbor.0],
                present_factor,
            ) + congestion_cost(
                pip_occupancy[pip.0],
                device.pips()[pip.0].capacity,
                pip_history[pip.0],
                present_factor,
            );
            let pip_delay_ps = costs.pip_min_delays_ps[pip.0];
            let next_distance = distance.saturating_add(routing_step_cost(
                pip_delay_ps,
                criticality,
                congestion,
                ROUTING_DELAY_QUANTUM_PS,
            ));
            let next_arrival_ps = arrival_ps.saturating_add(u64::from(pip_delay_ps));
            let next_state = (
                neighbor,
                hold_delay_bucket(next_arrival_ps, minimum_arrival_ps),
            );
            let improves =
                visits
                    .get(&next_state)
                    .is_none_or(|&(known_distance, known_arrival, _)| {
                        next_distance < known_distance
                            || (next_distance == known_distance && next_arrival_ps > known_arrival)
                    });
            if !improves {
                continue;
            }
            visits.insert(
                next_state,
                (next_distance, next_arrival_ps, Some((state, pip))),
            );
            let estimate = next_distance
                .saturating_add(device.wires()[neighbor.0].point.manhattan(goal_point));
            queue.push(Reverse((
                estimate,
                next_distance,
                next_arrival_ps,
                next_state,
            )));
        }
    }
    None
}

const HOLD_DELAY_QUANTUM_PS: u64 = 50;

fn hold_delay_bucket(arrival_ps: u64, minimum_arrival_ps: u64) -> u32 {
    if arrival_ps >= minimum_arrival_ps {
        return minimum_arrival_ps
            .div_ceil(HOLD_DELAY_QUANTUM_PS)
            .try_into()
            .unwrap_or(u32::MAX);
    }
    (arrival_ps / HOLD_DELAY_QUANTUM_PS)
        .try_into()
        .unwrap_or(u32::MAX - 1)
}

fn reconstruct_hold_path(
    mut state: HoldRouteState,
    visits: &HashMap<HoldRouteState, HoldRouteVisit>,
) -> (Vec<WireId>, Vec<PipId>) {
    let mut wires = vec![state.0];
    let mut pips = Vec::new();
    while let Some((previous, pip)) = visits[&state].2 {
        pips.push(pip);
        state = previous;
        wires.push(state.0);
    }
    (wires, pips)
}

const ROUTING_CRITICALITY_SCALE: u64 = 64;
const ROUTING_DELAY_QUANTUM_PS: u64 = 50;

fn timing_tree_cost(arrival_ps: u64, criticality: u64, delay_quantum_ps: u64) -> u64 {
    arrival_ps
        .saturating_mul(criticality)
        .div_ceil(ROUTING_CRITICALITY_SCALE * delay_quantum_ps)
}

fn routing_step_cost(
    pip_delay_ps: u32,
    criticality: u64,
    congestion: u64,
    delay_quantum_ps: u64,
) -> u64 {
    let congestion_scale = ROUTING_DELAY_QUANTUM_PS.div_ceil(delay_quantum_ps);
    if criticality == 0 {
        return 1_u64.saturating_add(congestion.saturating_mul(congestion_scale));
    }
    let delay_ps = u64::from(pip_delay_ps);
    let blended_ps = criticality
        .saturating_mul(delay_ps)
        .saturating_add(
            (ROUTING_CRITICALITY_SCALE - criticality).saturating_mul(ROUTING_DELAY_QUANTUM_PS),
        )
        .div_ceil(ROUTING_CRITICALITY_SCALE);
    blended_ps
        .div_ceil(delay_quantum_ps)
        .max(1)
        .saturating_add(congestion.saturating_mul(congestion_scale))
}

fn routing_transition_cost(
    arrival_ps: u64,
    next_arrival_ps: u64,
    criticality: u64,
    congestion: u64,
    delay_quantum_ps: u64,
) -> u64 {
    let congestion_scale = ROUTING_DELAY_QUANTUM_PS.div_ceil(delay_quantum_ps);
    if criticality == 0 {
        return 1_u64.saturating_add(congestion.saturating_mul(congestion_scale));
    }
    let timing_increment = timing_tree_cost(next_arrival_ps, criticality, delay_quantum_ps)
        .saturating_sub(timing_tree_cost(arrival_ps, criticality, delay_quantum_ps));
    let hop_bias = (ROUTING_CRITICALITY_SCALE - criticality)
        .saturating_mul(ROUTING_DELAY_QUANTUM_PS)
        .div_ceil(ROUTING_CRITICALITY_SCALE * delay_quantum_ps);
    timing_increment
        .saturating_add(hop_bias)
        .saturating_add(congestion.saturating_mul(congestion_scale))
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
    /// A target supplied a malformed immutable route tree.
    InvalidRoutingConstraint {
        /// Logical net named by the constraint.
        net: NetId,
        /// Specific invariant that failed.
        reason: String,
    },
    /// Target timing costs did not match the routed design or device.
    InvalidRoutingCosts {
        /// Specific invariant that failed.
        reason: String,
    },
    /// A supplied placement was incomplete, incompatible, or overlapping.
    InvalidPlacement {
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
    /// A completed route contained a cycle or disconnected component.
    RouteIsNotTree {
        /// Logical net whose physical route was malformed.
        net: NetId,
        /// Unique wires in the route.
        wires: usize,
        /// Unique PIPs in the route.
        pips: usize,
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
            Self::InvalidRoutingConstraint { net, reason } => {
                write!(f, "invalid routing constraint for net {}: {reason}", net.0)
            }
            Self::InvalidRoutingCosts { reason } => write!(f, "invalid routing costs: {reason}"),
            Self::InvalidPlacement { reason } => write!(f, "invalid placement: {reason}"),
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
            Self::RouteIsNotTree { net, wires, pips } => write!(
                f,
                "route for net {} is not a tree: {wires} wires, {pips} PIPs",
                net.0
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
            | Self::InvalidRoutingConstraint { .. }
            | Self::InvalidRoutingCosts { .. }
            | Self::InvalidPlacement { .. }
            | Self::MissingPlacement { .. }
            | Self::Unroutable { .. }
            | Self::CongestionNotResolved { .. }
            | Self::RouteIsNotTree { .. } => None,
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
    use std::collections::{BTreeMap, BTreeSet};

    use texo_model::{
        BelId, CellId, Design, Device, NetId, PinDirection, Point, ResourceKind, UnifiedGraph,
        WireId,
    };

    use super::{
        Placement, PlacementConstraints, PlacementNeighbor, PnrError, RouteSearch,
        RoutingConstraints, RoutingCosts, place_analytically_with_net_sink_weights,
        place_and_route, place_with_constraints, placement_neighbors,
        refine_placement_with_net_sink_weights_limited, refine_placement_with_net_weights,
        refinement_edge_cost, retain_route_for_sinks, route_reaches_all_sinks,
        route_with_placement_and_progress, route_with_timing_costs_and_progress, routing_corridor,
        routing_step_cost, routing_transition_cost, timing_tree_cost,
    };

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
    fn analytical_placement_is_deterministic_and_legal() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(4, 4).unwrap();
        let constraints = PlacementConstraints::new();

        let first = place_analytically_with_net_sink_weights(
            &design,
            &device,
            &constraints,
            &BTreeMap::new(),
        )
        .unwrap();
        let second = place_analytically_with_net_sink_weights(
            &design,
            &device,
            &constraints,
            &BTreeMap::new(),
        )
        .unwrap();

        assert_eq!(first, second);
        assert_ne!(first.bindings()[0], first.bindings()[1]);
    }

    #[test]
    fn timing_routing_corridor_is_clipped_to_the_device() {
        let device = Device::rectangular_logic(8, 6).unwrap();

        assert_eq!(
            routing_corridor(Point::new(1, 1), Point::new(6, 4), &device, 3),
            (0, 7, 0, 5),
        );
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
    fn seeds_an_unconnected_unit_at_the_device_center() {
        let mut design = Design::new();
        let cell = design.add_cell("cell", ResourceKind::Logic);
        design.add_pin(cell, "out", PinDirection::Output).unwrap();
        let device = Device::rectangular_logic(5, 1).unwrap();

        let placement =
            place_with_constraints(&design, &device, &PlacementConstraints::new()).unwrap();
        let point = device.bels()[placement.bel(cell).unwrap().0].point;

        assert_eq!(point, Point::new(2, 0));
    }

    #[test]
    fn timing_driven_refinement_penalizes_long_edges_quadratically() {
        let ordinary = PlacementNeighbor {
            cell: CellId(0),
            weight: 2,
            timing_driven: false,
        };
        let critical = PlacementNeighbor {
            timing_driven: true,
            ..ordinary
        };

        assert_eq!(refinement_edge_cost(ordinary, 3), 6);
        assert_eq!(refinement_edge_cost(critical, 3), 18);
    }

    #[test]
    fn sink_weights_do_not_promote_unrelated_fanout_edges() {
        let mut design = Design::new();
        let source = design.add_cell("source", ResourceKind::Logic);
        let source_out = design.add_pin(source, "out", PinDirection::Output).unwrap();
        let critical = design.add_cell("critical", ResourceKind::Logic);
        let critical_in = design.add_pin(critical, "in", PinDirection::Input).unwrap();
        let ordinary = design.add_cell("ordinary", ResourceKind::Logic);
        let ordinary_in = design.add_pin(ordinary, "in", PinDirection::Input).unwrap();
        design
            .add_net("fanout", source_out, [critical_in, ordinary_in])
            .unwrap();

        let (_, neighbors) = placement_neighbors(
            &design,
            None,
            Some(&BTreeMap::from([((NetId(0), critical_in), 64)])),
        );
        let critical_edge = neighbors[source.0]
            .iter()
            .find(|edge| edge.cell == critical)
            .unwrap();
        let ordinary_edge = neighbors[source.0]
            .iter()
            .find(|edge| edge.cell == ordinary)
            .unwrap();

        assert_eq!(critical_edge.weight, 2_048);
        assert!(critical_edge.timing_driven);
        assert_eq!(ordinary_edge.weight, 32);
        assert!(!ordinary_edge.timing_driven);
    }

    #[test]
    fn incremental_refinement_starts_from_the_supplied_placement() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(5, 1).unwrap();
        let initial = Placement {
            bindings: vec![BelId(0), BelId(4)],
            pin_bindings: BTreeMap::new(),
        };

        let refined = refine_placement_with_net_weights(
            &design,
            &device,
            &PlacementConstraints::new(),
            initial,
            &BTreeMap::from([(NetId(0), 64)]),
        )
        .unwrap();
        let source = refined.point(CellId(0), &device).unwrap();
        let sink = refined.point(CellId(1), &device).unwrap();

        assert!(source.manhattan(sink) < 4);
    }

    #[test]
    fn locked_routes_are_not_marked_dirty() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(4, 1).unwrap();
        let implementation = place_and_route(&design, &device).unwrap();
        let mut constraints = RoutingConstraints::new();
        constraints.add_route(implementation.routes[0].clone());
        let mut iterations = Vec::new();

        route_with_placement_and_progress(
            &design,
            &device,
            implementation.placement,
            &constraints,
            |event| {
                if let super::RoutingProgress::Iteration { nets, .. } = event {
                    iterations.push(nets);
                }
            },
        )
        .unwrap();

        assert_eq!(iterations, vec![0]);
    }

    #[test]
    fn partial_route_preserves_shared_tree_and_routes_only_missing_sink_arc() {
        let mut design = Design::new();
        let source = design.add_cell("source", ResourceKind::Logic);
        let source_out = design.add_pin(source, "out", PinDirection::Output).unwrap();
        let near = design.add_cell("near", ResourceKind::Logic);
        let near_in = design.add_pin(near, "in", PinDirection::Input).unwrap();
        let far = design.add_cell("far", ResourceKind::Logic);
        let far_in = design.add_pin(far, "in", PinDirection::Input).unwrap();
        design
            .add_net("fanout", source_out, [near_in, far_in])
            .unwrap();
        let device = Device::rectangular_logic(3, 1).unwrap();
        let placement = Placement {
            bindings: vec![BelId(0), BelId(1), BelId(2)],
            pin_bindings: BTreeMap::new(),
        };
        let full = route_with_placement_and_progress(
            &design,
            &device,
            placement.clone(),
            &RoutingConstraints::new(),
            |_| {},
        )
        .unwrap();
        let partial = retain_route_for_sinks(
            &design,
            &device,
            &placement,
            &full.routes[0],
            &BTreeSet::from([near_in]),
        )
        .unwrap()
        .unwrap();
        assert!(partial.pips.len() < full.routes[0].pips.len());

        let mut constraints = RoutingConstraints::new();
        constraints.add_route(partial.clone());
        let mut iterations = Vec::new();
        let rerouted = route_with_placement_and_progress(
            &design,
            &device,
            placement.clone(),
            &constraints,
            |event| {
                if let super::RoutingProgress::Iteration { nets, .. } = event {
                    iterations.push(nets);
                }
            },
        )
        .unwrap();

        assert_eq!(iterations, vec![1]);
        assert!(
            partial
                .pips
                .iter()
                .all(|pip| rerouted.routes[0].pips.contains(pip))
        );
        assert!(
            route_reaches_all_sinks(
                &UnifiedGraph::new(&design, &device),
                &placement,
                &rerouted.routes[0]
            )
            .unwrap()
        );
    }

    #[test]
    fn limited_refinement_bounds_changed_units() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(5, 1).unwrap();
        let initial = Placement {
            bindings: vec![BelId(0), BelId(4)],
            pin_bindings: BTreeMap::new(),
        };

        let refined = refine_placement_with_net_sink_weights_limited(
            &design,
            &device,
            &PlacementConstraints::new(),
            initial.clone(),
            &BTreeMap::new(),
            1,
        )
        .unwrap();
        let changed = initial
            .bindings()
            .iter()
            .zip(refined.bindings())
            .filter(|(before, after)| before != after)
            .count();

        assert!(changed <= 1);
    }

    #[test]
    fn timing_routing_blends_delay_with_congestion() {
        assert_eq!(routing_step_cost(1_000, 0, 2, 50), 3);
        assert_eq!(routing_step_cost(200, 64, 0, 50), 4);
        assert_eq!(routing_step_cost(200, 32, 0, 50), 3);
        assert_eq!(routing_step_cost(200, 64, 2, 50), 6);
        assert_eq!(timing_tree_cost(200, 64, 50), 4);
    }

    #[test]
    fn cumulative_quantization_does_not_round_every_pip() {
        assert_eq!(routing_transition_cost(0, 24, 64, 0, 50), 1);
        assert_eq!(routing_transition_cost(24, 48, 64, 0, 50), 0);
        assert_eq!(routing_transition_cost(48, 72, 64, 0, 10), 3);
        assert_eq!(routing_transition_cost(48, 72, 64, 0, 1), 24);
    }

    #[test]
    fn timing_routing_accounts_for_delay_to_each_tree_source() {
        let design = Design::new();
        let mut device = Device::new("tree-delay", 1, 1).unwrap();
        let slow_tree = device.add_wire("slow-tree", Point::new(0, 0), 1).unwrap();
        let fast_tree = device.add_wire("fast-tree", Point::new(0, 0), 1).unwrap();
        let goal = device.add_wire("goal", Point::new(0, 0), 1).unwrap();
        device.add_pip(slow_tree, goal, false, 1).unwrap();
        device.add_pip(fast_tree, goal, false, 1).unwrap();
        let graph = UnifiedGraph::new(&design, &device);
        let costs = RoutingCosts::new(vec![10, 10], BTreeMap::new());
        let mut search = RouteSearch::new(device.wires().len());
        let starts = [slow_tree, fast_tree].into_iter().collect();
        let tree_delays = BTreeMap::from([(slow_tree, 100), (fast_tree, 0)]);

        let (wires, _) = search
            .shortest_path(
                &graph,
                &starts,
                goal,
                &[0; 3],
                &[0; 2],
                &[0; 3],
                &[0; 2],
                1,
                Some(&costs),
                64,
                50,
                &tree_delays,
                0,
            )
            .unwrap();

        assert_eq!(wires.last(), Some(&fast_tree));
        assert_ne!(wires.last(), Some(&WireId(0)));
    }

    #[test]
    fn hold_routing_rejects_a_path_below_the_sink_minimum() {
        let design = Design::new();
        let mut device = Device::new("hold-detour", 1, 1).unwrap();
        let start = device.add_wire("start", Point::new(0, 0), 1).unwrap();
        let fast = device.add_wire("fast", Point::new(0, 0), 1).unwrap();
        let slow = device.add_wire("slow", Point::new(0, 0), 1).unwrap();
        let goal = device.add_wire("goal", Point::new(0, 0), 1).unwrap();
        device.add_pip(start, fast, false, 1).unwrap();
        device.add_pip(fast, goal, false, 1).unwrap();
        let slow_first = device.add_pip(start, slow, false, 1).unwrap();
        let slow_last = device.add_pip(slow, goal, false, 1).unwrap();
        let graph = UnifiedGraph::new(&design, &device);
        let costs = RoutingCosts::new(vec![10, 10, 400, 400], BTreeMap::new());
        let mut search = RouteSearch::new(device.wires().len());

        let (_, pips) = search
            .shortest_path(
                &graph,
                &BTreeSet::from([start]),
                goal,
                &[0; 4],
                &[0; 4],
                &[0; 4],
                &[0; 4],
                1,
                Some(&costs),
                0,
                50,
                &BTreeMap::from([(start, 0)]),
                500,
            )
            .unwrap();

        assert_eq!(pips, vec![slow_last, slow_first]);
    }

    #[test]
    fn rejects_a_timing_table_that_does_not_cover_the_device() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(2, 1).unwrap();
        let placement =
            place_with_constraints(&design, &device, &PlacementConstraints::new()).unwrap();
        let costs = RoutingCosts::new(Vec::new(), BTreeMap::from([(NetId(0), 64)]));

        assert!(matches!(
            route_with_timing_costs_and_progress(
                &design,
                &device,
                placement,
                &RoutingConstraints::new(),
                &costs,
                |_| {},
            ),
            Err(PnrError::InvalidRoutingCosts { .. })
        ));
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
    fn placement_separates_distinct_nets_on_shared_bel_pin_wires() {
        let mut design = Design::new();
        let source_a = design.add_cell("source_a", ResourceKind::Logic);
        let source_a_pin = design.add_pin(source_a, "A", PinDirection::Output).unwrap();
        let source_b = design.add_cell("source_b", ResourceKind::Logic);
        let source_b_pin = design.add_pin(source_b, "B", PinDirection::Output).unwrap();
        let sink_a = design.add_cell("sink_a", ResourceKind::Register);
        let sink_a_pin = design.add_pin(sink_a, "D", PinDirection::Input).unwrap();
        let sink_b = design.add_cell("sink_b", ResourceKind::Register);
        let sink_b_pin = design.add_pin(sink_b, "D", PinDirection::Input).unwrap();
        design.add_net("a", source_a_pin, [sink_a_pin]).unwrap();
        design.add_net("b", source_b_pin, [sink_b_pin]).unwrap();

        let mut device = Device::new("shared-register-input", 4, 1).unwrap();
        let source_a_wire = device.add_wire("A", Point::new(0, 0), 1).unwrap();
        let shared_input = device.add_wire("shared-D", Point::new(1, 0), 1).unwrap();
        let separate_input = device.add_wire("separate-D", Point::new(2, 0), 1).unwrap();
        let source_b_wire = device.add_wire("B", Point::new(3, 0), 1).unwrap();
        let source_a_bel = device
            .add_bel("source-A", ResourceKind::Logic, Point::new(0, 0))
            .unwrap();
        device
            .add_bel_pin(source_a_bel, "A", PinDirection::Output, source_a_wire)
            .unwrap();
        let source_b_bel = device
            .add_bel("source-B", ResourceKind::Logic, Point::new(3, 0))
            .unwrap();
        device
            .add_bel_pin(source_b_bel, "B", PinDirection::Output, source_b_wire)
            .unwrap();
        for (name, point, wire) in [
            ("ff-shared-0", Point::new(1, 0), shared_input),
            ("ff-shared-1", Point::new(2, 0), shared_input),
            ("ff-separate", Point::new(2, 0), separate_input),
        ] {
            let bel = device.add_bel(name, ResourceKind::Register, point).unwrap();
            device
                .add_bel_pin(bel, "D", PinDirection::Input, wire)
                .unwrap();
        }

        let placement =
            place_with_constraints(&design, &device, &PlacementConstraints::new()).unwrap();
        let sink_wire = |cell| {
            let bel = placement.bel(cell).unwrap();
            let pin = device.bels()[bel.0].pins()[0];
            device.bel_pins()[pin.0].wire
        };

        assert_ne!(sink_wire(sink_a), sink_wire(sink_b));
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
