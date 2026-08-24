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
    sink_criticalities: BTreeMap<(NetId, CellPinId), u64>,
    detailed_timing_nets: BTreeSet<NetId>,
    detailed_delay_quantum_ps: u64,
    max_iterations: u32,
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
            sink_criticalities: BTreeMap::new(),
            detailed_timing_nets: BTreeSet::new(),
            detailed_delay_quantum_ps: 1,
            max_iterations: MAX_ROUTING_ITERATIONS,
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

    /// Caps negotiated-congestion iterations for this routing trial.
    ///
    /// Full routing defaults to 32 iterations. Callers evaluating disposable
    /// local ECO candidates can use a smaller cap so an infeasible move fails
    /// quickly without weakening the final whole-design negotiation.
    pub fn set_max_iterations(&mut self, max_iterations: u32) {
        self.max_iterations = max_iterations.clamp(1, MAX_ROUTING_ITERATIONS);
    }

    /// Restores the full negotiated-congestion iteration budget.
    pub fn reset_max_iterations(&mut self) {
        self.max_iterations = MAX_ROUTING_ITERATIONS;
    }

    /// Negotiated-congestion iteration limit for this routing trial.
    #[must_use]
    pub const fn max_iterations(&self) -> u32 {
        self.max_iterations
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

    /// Replaces setup criticalities for individual driver-to-sink arcs.
    pub fn set_sink_criticalities(
        &mut self,
        sink_criticalities: BTreeMap<(NetId, CellPinId), u64>,
    ) {
        self.sink_criticalities = sink_criticalities;
    }

    /// Setup criticalities for individual driver-to-sink arcs.
    #[must_use]
    pub const fn sink_criticalities(&self) -> &BTreeMap<(NetId, CellPinId), u64> {
        &self.sink_criticalities
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

    /// Replaces one grouped cell while preserving the group's assignment
    /// column. Returns false when the old cell is not grouped or the new cell
    /// already belongs to any group.
    pub fn replace_group_cell(&mut self, old: CellId, new: CellId) -> bool {
        if self.groups.iter().any(|group| group.cells.contains(&new)) {
            return false;
        }
        let Some((group_index, cell_index)) =
            self.groups.iter().enumerate().find_map(|(g, group)| {
                group
                    .cells
                    .iter()
                    .position(|&cell| cell == old)
                    .map(|c| (g, c))
            })
        else {
            return false;
        };
        self.groups[group_index].cells[cell_index] = new;
        true
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

    /// Restores target-default physical pin selection for one logical pin.
    pub fn unbind_pin_name(&mut self, pin: CellPinId) {
        self.pin_name_bindings.remove(&pin);
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

/// One ordered driver-to-endpoint route.
///
/// Logical sink arcs carry `Some(sink)`. `None` is reserved for immutable
/// architecture topology, such as an enabled global-clock quadrant with no
/// logical sink at its leaf.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RouteArc {
    /// Logical sink reached by this arc, when it represents a design edge.
    pub sink: Option<CellPinId>,
    /// Wires ordered from the placed driver to the endpoint.
    pub wires: Vec<WireId>,
    /// PIPs ordered from the placed driver to the endpoint. PIP `i` connects
    /// `wires[i]` to `wires[i + 1]`.
    pub pips: Vec<PipId>,
}

/// Per-sink routes and shared-resource reference counts for one logical net.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NetRoute {
    /// Logical net represented by these arcs.
    pub net: NetId,
    /// Canonical driver-to-endpoint arcs, ordered by logical sink then path.
    pub arcs: Vec<RouteArc>,
    wire_refs: Vec<(WireId, u32)>,
    pip_refs: Vec<(PipId, u32)>,
}

/// Sparse incumbent-route occupancy used to rank placement moves before a
/// negotiated-routing trial.
///
/// Every resource owner carries the greatest criticality of the owner's arcs
/// that use that resource.  A critical shared trunk is therefore protected,
/// while a resource used only by a noncritical branch remains a cheap victim.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct RouteCapacityProjection {
    wire_owners: HashMap<WireId, Vec<(NetId, u64)>>,
    pip_owners: HashMap<PipId, Vec<(NetId, u64)>>,
}

impl RouteCapacityProjection {
    /// Projects routed arcs onto the resources they occupy.
    #[must_use]
    pub fn new(routes: &[NetRoute], costs: &RoutingCosts) -> Self {
        let mut projection = Self::default();
        for route in routes {
            for arc in &route.arcs {
                let criticality = arc
                    .sink
                    .and_then(|sink| costs.sink_criticalities.get(&(route.net, sink)).copied())
                    .or_else(|| costs.net_criticalities.get(&route.net).copied())
                    .unwrap_or(0);
                for &wire in &arc.wires {
                    update_projected_owner(
                        projection.wire_owners.entry(wire).or_default(),
                        route.net,
                        criticality,
                    );
                }
                for &pip in &arc.pips {
                    update_projected_owner(
                        projection.pip_owners.entry(pip).or_default(),
                        route.net,
                        criticality,
                    );
                }
            }
        }
        projection
    }
}

fn update_projected_owner(owners: &mut Vec<(NetId, u64)>, net: NetId, criticality: u64) {
    if let Some((_, known)) = owners.iter_mut().find(|(owner, _)| *owner == net) {
        *known = (*known).max(criticality);
    } else {
        owners.push((net, criticality));
    }
}

impl NetRoute {
    /// Builds a route and derives its shared-resource reference counts.
    #[must_use]
    pub fn new(net: NetId, mut arcs: Vec<RouteArc>) -> Self {
        arcs.sort_by(|left, right| {
            (left.sink, &left.wires, &left.pips).cmp(&(right.sink, &right.wires, &right.pips))
        });
        let mut wire_refs = BTreeMap::<WireId, u32>::new();
        let mut pip_refs = BTreeMap::<PipId, u32>::new();
        for arc in &arcs {
            for &wire in &arc.wires {
                *wire_refs.entry(wire).or_default() += 1;
            }
            for &pip in &arc.pips {
                *pip_refs.entry(pip).or_default() += 1;
            }
        }
        Self {
            net,
            arcs,
            wire_refs: wire_refs.into_iter().collect(),
            pip_refs: pip_refs.into_iter().collect(),
        }
    }

    /// Decomposes a driver-rooted physical tree into canonical endpoint arcs.
    /// Extra leaves are retained as architecture topology arcs.
    ///
    /// # Errors
    ///
    /// Returns a topology description when a PIP is unknown, the selected
    /// resources are not one driver-rooted tree, or a sink is disconnected.
    pub fn from_tree(
        net: NetId,
        driver: WireId,
        sinks: impl IntoIterator<Item = (CellPinId, WireId)>,
        pips: impl IntoIterator<Item = PipId>,
        device: &Device,
    ) -> Result<Self, String> {
        let pips = pips.into_iter().collect::<BTreeSet<_>>();
        let mut adjacent = BTreeMap::<WireId, Vec<(WireId, PipId)>>::new();
        for &pip_id in &pips {
            let pip = device
                .pips()
                .get(pip_id.0)
                .ok_or_else(|| format!("unknown PIP {pip_id:?}"))?;
            adjacent
                .entry(pip.from())
                .or_default()
                .push((pip.to(), pip_id));
            if pip.bidirectional() {
                adjacent
                    .entry(pip.to())
                    .or_default()
                    .push((pip.from(), pip_id));
            }
        }
        let mut parent = BTreeMap::<WireId, (WireId, PipId)>::new();
        let mut reached_pips = BTreeSet::new();
        let mut children = BTreeMap::<WireId, usize>::new();
        let mut visited = BTreeSet::from([driver]);
        let mut pending = vec![driver];
        while let Some(wire) = pending.pop() {
            for &(next, pip) in adjacent.get(&wire).map_or(&[][..], Vec::as_slice) {
                if visited.insert(next) {
                    parent.insert(next, (wire, pip));
                    reached_pips.insert(pip);
                    *children.entry(wire).or_default() += 1;
                    pending.push(next);
                }
            }
        }
        if reached_pips != pips {
            return Err("physical route is not one driver-rooted tree".into());
        }

        let sinks = sinks.into_iter().collect::<Vec<_>>();
        let sink_wires = sinks.iter().map(|&(_, wire)| wire).collect::<BTreeSet<_>>();
        let mut arcs = Vec::new();
        for (sink, sink_wire) in sinks {
            arcs.push(
                reconstruct_endpoint_arc(Some(sink), driver, sink_wire, &parent)
                    .ok_or_else(|| format!("sink pin {} is disconnected", sink.0))?,
            );
        }
        for &leaf in visited
            .iter()
            .filter(|wire| !children.contains_key(wire) && !sink_wires.contains(wire))
        {
            arcs.push(
                reconstruct_endpoint_arc(None, driver, leaf, &parent)
                    .ok_or_else(|| "topology leaf is disconnected".to_owned())?,
            );
        }
        Ok(Self::new(net, arcs))
    }

    /// Unique occupied wires in stable ID order.
    #[must_use]
    pub fn wires(&self) -> impl ExactSizeIterator<Item = WireId> + '_ {
        self.wire_refs.iter().map(|&(wire, _)| wire)
    }

    /// Unique enabled PIPs in stable ID order.
    #[must_use]
    pub fn pips(&self) -> impl ExactSizeIterator<Item = PipId> + '_ {
        self.pip_refs.iter().map(|&(pip, _)| pip)
    }

    /// Number of arcs sharing `wire` inside this net.
    #[must_use]
    pub fn wire_ref_count(&self, wire: WireId) -> u32 {
        self.wire_refs
            .binary_search_by_key(&wire, |&(candidate, _)| candidate)
            .map_or(0, |index| self.wire_refs[index].1)
    }

    /// Number of arcs sharing `pip` inside this net.
    #[must_use]
    pub fn pip_ref_count(&self, pip: PipId) -> u32 {
        self.pip_refs
            .binary_search_by_key(&pip, |&(candidate, _)| candidate)
            .map_or(0, |index| self.pip_refs[index].1)
    }

    /// Finds the route arc for one logical sink.
    #[must_use]
    pub fn arc(&self, sink: CellPinId) -> Option<&RouteArc> {
        self.arcs.iter().find(|arc| arc.sink == Some(sink))
    }
}

fn reconstruct_endpoint_arc(
    sink: Option<CellPinId>,
    driver: WireId,
    endpoint: WireId,
    parent: &BTreeMap<WireId, (WireId, PipId)>,
) -> Option<RouteArc> {
    let mut wires = vec![endpoint];
    let mut pips = Vec::new();
    let mut cursor = endpoint;
    while cursor != driver {
        let &(previous, pip) = parent.get(&cursor)?;
        pips.push(pip);
        wires.push(previous);
        cursor = previous;
    }
    wires.reverse();
    pips.reverse();
    Some(RouteArc { sink, wires, pips })
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

/// Reusable negotiated-routing state sized for one physical device.
///
/// Production devices contain millions of wires and tens of millions of
/// PIPs. Keeping occupancy, history, and A* scratch allocations alive across
/// timing trials avoids repeatedly allocating and releasing hundreds of
/// megabytes while preserving the stateless routing result contract.
#[derive(Debug)]
pub struct RoutingWorkspace {
    device_identity: usize,
    wire_occupancy: Vec<u16>,
    pip_occupancy: Vec<u16>,
    wire_history: Vec<u32>,
    pip_history: Vec<u32>,
    touched_wires: Vec<usize>,
    touched_pips: Vec<usize>,
    search: RouteSearch,
    tree_arrival_ps: Vec<u64>,
    wire_points: Vec<Point>,
    wire_capacities: Vec<u16>,
    pip_capacities: Vec<u16>,
    /// Routes whose resource usage is currently reflected by occupancy.
    /// A failed negotiation invalidates this snapshot; the next call then
    /// falls back to a full sparse reset.
    resident_routes: Vec<Option<NetRoute>>,
    resident_valid: bool,
}

impl RoutingWorkspace {
    /// Allocates routing state for `device` once.
    #[must_use]
    pub fn new(device: &Device) -> Self {
        Self {
            device_identity: std::ptr::from_ref(device) as usize,
            wire_occupancy: vec![0; device.wires().len()],
            pip_occupancy: vec![0; device.pips().len()],
            wire_history: vec![0; device.wires().len()],
            pip_history: vec![0; device.pips().len()],
            touched_wires: Vec::new(),
            touched_pips: Vec::new(),
            search: RouteSearch::new(device.wires().len()),
            tree_arrival_ps: vec![UNROUTED_ARRIVAL_PS; device.wires().len()],
            wire_points: device.wires().iter().map(|wire| wire.point).collect(),
            wire_capacities: device.wires().iter().map(|wire| wire.capacity).collect(),
            pip_capacities: device
                .pips()
                .iter()
                .map(texo_model::Pip::capacity)
                .collect(),
            resident_routes: Vec::new(),
            resident_valid: false,
        }
    }

    fn prepare(&mut self, device: &Device) {
        if self.device_identity != std::ptr::from_ref(device) as usize
            || self.wire_occupancy.len() != device.wires().len()
            || self.pip_occupancy.len() != device.pips().len()
        {
            *self = Self::new(device);
            return;
        }
        for index in self.touched_wires.drain(..) {
            self.wire_occupancy[index] = 0;
            self.wire_history[index] = 0;
        }
        for index in self.touched_pips.drain(..) {
            self.pip_occupancy[index] = 0;
            self.pip_history[index] = 0;
        }
        self.resident_routes.clear();
        self.resident_valid = false;
    }

    /// Synchronizes persistent occupancy to a new set of frozen routes.
    /// Unchanged net trees cost one equality check; only changed trees touch
    /// resource counters. History remains trial-local so rejected searches do
    /// not bias the next transaction.
    fn prepare_routes(
        &mut self,
        device: &Device,
        net_count: usize,
        constraints: &RoutingConstraints,
    ) -> Vec<Option<NetRoute>> {
        if self.device_identity != std::ptr::from_ref(device) as usize
            || self.wire_occupancy.len() != device.wires().len()
            || self.pip_occupancy.len() != device.pips().len()
        {
            *self = Self::new(device);
        }
        for &index in &self.touched_wires {
            self.wire_history[index] = 0;
        }
        for &index in &self.touched_pips {
            self.pip_history[index] = 0;
        }
        let mut target = vec![None; net_count];
        for (&net, route) in constraints.routes() {
            target[net.0] = Some(route.clone());
        }
        if !self.resident_valid || self.resident_routes.len() != net_count {
            self.prepare(device);
            for route in target.iter().flatten() {
                add_route_occupancy(self, route);
            }
        } else {
            for (index, new) in target.iter().enumerate() {
                let old = self.resident_routes[index].clone();
                let new = new.as_ref();
                if old.as_ref() == new {
                    continue;
                }
                if let Some(old) = old.as_ref() {
                    remove_route_occupancy(self, old, new);
                }
                if let Some(new) = new {
                    add_route_occupancy_delta(self, new, old.as_ref());
                }
            }
        }
        // Until negotiation succeeds, occupancy no longer has a committed
        // route snapshot to synchronize from safely.
        self.resident_valid = false;
        target
    }

    fn commit_routes(&mut self, routes: &[NetRoute]) {
        self.resident_routes = routes.iter().cloned().map(Some).collect();
        self.resident_valid = true;
    }
}

fn add_route_occupancy(workspace: &mut RoutingWorkspace, route: &NetRoute) {
    for wire in route.wires() {
        increment_occupancy(
            &mut workspace.wire_occupancy,
            &mut workspace.touched_wires,
            wire.0,
        );
    }
    for pip in route.pips() {
        increment_occupancy(
            &mut workspace.pip_occupancy,
            &mut workspace.touched_pips,
            pip.0,
        );
    }
}

fn remove_route_occupancy(
    workspace: &mut RoutingWorkspace,
    old: &NetRoute,
    replacement: Option<&NetRoute>,
) {
    for wire in old
        .wires()
        .filter(|&wire| replacement.is_none_or(|route| route.wire_ref_count(wire) == 0))
    {
        workspace.wire_occupancy[wire.0] -= 1;
    }
    for pip in old
        .pips()
        .filter(|&pip| replacement.is_none_or(|route| route.pip_ref_count(pip) == 0))
    {
        workspace.pip_occupancy[pip.0] -= 1;
    }
}

fn add_route_occupancy_delta(
    workspace: &mut RoutingWorkspace,
    new: &NetRoute,
    previous: Option<&NetRoute>,
) {
    for wire in new
        .wires()
        .filter(|&wire| previous.is_none_or(|route| route.wire_ref_count(wire) == 0))
    {
        increment_occupancy(
            &mut workspace.wire_occupancy,
            &mut workspace.touched_wires,
            wire.0,
        );
    }
    for pip in new
        .pips()
        .filter(|&pip| previous.is_none_or(|route| route.pip_ref_count(pip) == 0))
    {
        increment_occupancy(
            &mut workspace.pip_occupancy,
            &mut workspace.touched_pips,
            pip.0,
        );
    }
}

#[derive(Clone, Copy)]
struct RoutingResourceMetadata<'a> {
    wire_points: &'a [Point],
    wire_capacities: &'a [u16],
    pip_capacities: &'a [u16],
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
    for &sink in sinks {
        let sink_cell = design.pins()[sink.0].cell;
        let sink_bel = placement
            .bel(sink_cell)
            .ok_or(PnrError::MissingPlacement { cell: sink_cell })?;
        let sink_wire = bound_wire(&graph, placement, sink, sink_bel)?;
        let Some(arc) = route.arc(sink) else {
            return Err(PnrError::InvalidRoutingConstraint {
                net: route.net,
                reason: format!("route does not reach retained sink pin {}", sink.0),
            });
        };
        if arc.wires.last().copied() != Some(sink_wire) {
            return Err(PnrError::InvalidRoutingConstraint {
                net: route.net,
                reason: format!("route arc for sink pin {} ends on another wire", sink.0),
            });
        }
    }
    let arcs = route
        .arcs
        .iter()
        .filter(|arc| arc.sink.is_none_or(|sink| sinks.contains(&sink)))
        .cloned()
        .collect();
    Ok(Some(NetRoute::new(route.net, arcs)))
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
    let mut workspace = RoutingWorkspace::new(device);
    finish_routing_with_workspace(
        &UnifiedGraph::new(design, device),
        placement,
        routing_constraints,
        None,
        &mut workspace,
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
    let mut workspace = RoutingWorkspace::new(device);
    finish_routing_with_workspace(
        &UnifiedGraph::new(design, device),
        placement,
        routing_constraints,
        Some(routing_costs),
        &mut workspace,
        &mut progress,
    )
}

/// Routes an existing placement using reusable device-sized state.
///
/// # Errors
///
/// Returns an invalid routing-constraint, model, or routability error.
pub fn route_with_workspace_and_progress(
    design: &Design,
    device: &Device,
    placement: Placement,
    routing_constraints: &RoutingConstraints,
    workspace: &mut RoutingWorkspace,
    mut progress: impl FnMut(RoutingProgress),
) -> Result<PnrResult, PnrError> {
    finish_routing_with_workspace(
        &UnifiedGraph::new(design, device),
        placement,
        routing_constraints,
        None,
        workspace,
        &mut progress,
    )
}

/// Routes with characterized costs using reusable device-sized state.
///
/// # Errors
///
/// Returns an invalid cost/constraint, model, or routability error.
pub fn route_with_timing_costs_workspace_and_progress(
    design: &Design,
    device: &Device,
    placement: Placement,
    routing_constraints: &RoutingConstraints,
    routing_costs: &RoutingCosts,
    workspace: &mut RoutingWorkspace,
    mut progress: impl FnMut(RoutingProgress),
) -> Result<PnrResult, PnrError> {
    finish_routing_with_workspace(
        &UnifiedGraph::new(design, device),
        placement,
        routing_constraints,
        Some(routing_costs),
        workspace,
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
    let mut workspace = RoutingWorkspace::new(graph.device());
    finish_routing_with_workspace(
        graph,
        placement,
        routing_constraints,
        routing_costs,
        &mut workspace,
        progress,
    )
}

fn finish_routing_with_workspace(
    graph: &UnifiedGraph<'_>,
    placement: Placement,
    routing_constraints: &RoutingConstraints,
    routing_costs: Option<&RoutingCosts>,
    workspace: &mut RoutingWorkspace,
    progress: &mut impl FnMut(RoutingProgress),
) -> Result<PnrResult, PnrError> {
    validate_routing_constraints(graph, &placement, routing_constraints)?;
    validate_routing_costs(graph, routing_costs)?;
    let routes = workspace.prepare_routes(
        graph.device(),
        graph.design().nets().len(),
        routing_constraints,
    );
    let routes = route(
        graph,
        &placement,
        routing_constraints,
        routing_costs,
        workspace,
        routes,
        progress,
    )?;
    for route in &routes {
        if route.pips().len().saturating_add(1) != route.wires().len() {
            return Err(PnrError::RouteIsNotTree {
                net: route.net,
                wires: route.wires().len(),
                pips: route.pips().len(),
            });
        }
    }
    let total_pips = routes.iter().map(|route| route.pips().len()).sum();
    workspace.commit_routes(&routes);
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

/// Completes and validates a legal placement from caller-selected bindings.
///
/// Bindings may omit cells that belong to an atomic placement group. The
/// first legal group assignment consistent with every supplied binding is
/// selected, which lets target adapters recover synthetic carry feed-in/out
/// cells from the positions of the shared logical carry chain. Unconstrained
/// omitted cells use their first compatible BEL. No placement optimization is
/// performed.
///
/// # Errors
///
/// Returns an error when a cell/BEL ID is unknown, two completed units overlap,
/// or no legal assignment agrees with the supplied bindings.
pub fn placement_from_partial_bindings(
    design: &Design,
    device: &Device,
    constraints: &PlacementConstraints,
    bindings: &BTreeMap<CellId, BelId>,
) -> Result<Placement, PnrError> {
    let graph = UnifiedGraph::new(design, device);
    let mut candidate_cache = BTreeMap::new();
    let units = placement_units(&graph, constraints, &mut candidate_cache)?;
    let mut placed = vec![None; design.cells().len()];
    let mut occupied = BTreeSet::new();

    for (&cell, &bel) in bindings {
        if cell.0 >= design.cells().len() {
            return Err(PnrError::InvalidPlacement {
                reason: format!("binding names unknown cell {}", cell.0),
            });
        }
        if bel.0 >= device.bels().len() {
            return Err(PnrError::InvalidPlacement {
                reason: format!("binding names unknown BEL {}", bel.0),
            });
        }
    }

    for unit in &units {
        let assignment = (0..unit.choices.len())
            .map(|index| unit.choices.assignment(index))
            .find(|assignment| {
                unit.cells
                    .iter()
                    .zip(*assignment)
                    .all(|(&cell, &bel)| bindings.get(&cell).is_none_or(|wanted| *wanted == bel))
            })
            .ok_or_else(|| PnrError::InvalidPlacement {
                reason: format!(
                    "cell group beginning at {} has no assignment matching the supplied bindings",
                    unit.cells[0].0
                ),
            })?;
        for (&cell, &bel) in unit.cells.iter().zip(assignment) {
            if !occupied.insert(bel) {
                return Err(PnrError::InvalidPlacement {
                    reason: format!("BEL {} is assigned more than once", bel.0),
                });
            }
            placed[cell.0] = Some(bel);
        }
    }

    finish_placement(&graph, constraints, placed)
}

/// Swaps two same-kind cells in an otherwise unchanged legal placement and
/// rebuilds target-selected physical pin bindings.
///
/// This is a constant-scope detailed-placement ECO. Atomic groups containing
/// either cell are checked against their existing legal assignment tables;
/// unrelated placement units are not regenerated or legalized.
///
/// # Errors
///
/// Returns an error for unknown/different-kind cells or an illegal resulting
/// group assignment.
pub fn swap_placement_cells(
    design: &Design,
    device: &Device,
    constraints: &PlacementConstraints,
    placement: &Placement,
    left: CellId,
    right: CellId,
) -> Result<Placement, PnrError> {
    let Some(left_cell) = design.cells().get(left.0) else {
        return Err(PnrError::InvalidPlacement {
            reason: format!("swap names unknown cell {}", left.0),
        });
    };
    let Some(right_cell) = design.cells().get(right.0) else {
        return Err(PnrError::InvalidPlacement {
            reason: format!("swap names unknown cell {}", right.0),
        });
    };
    if left_cell.kind != right_cell.kind {
        return Err(PnrError::InvalidPlacement {
            reason: "placement ECO cells have different resource kinds".into(),
        });
    }
    if placement.bindings.len() != design.cells().len() {
        return Err(PnrError::InvalidPlacement {
            reason: "placement ECO input is incomplete".into(),
        });
    }
    let mut bindings = placement.bindings.clone();
    bindings.swap(left.0, right.0);
    for group in constraints
        .groups()
        .iter()
        .filter(|group| group.cells.contains(&left) || group.cells.contains(&right))
    {
        let assignment = group
            .cells
            .iter()
            .map(|cell| bindings[cell.0])
            .collect::<Vec<_>>();
        if !group.assignments.contains(&assignment) {
            return Err(PnrError::InvalidPlacement {
                reason: format!(
                    "placement ECO violates group beginning at cell {}",
                    group.cells[0].0
                ),
            });
        }
    }
    finish_placement(
        &UnifiedGraph::new(design, device),
        constraints,
        bindings.into_iter().map(Some).collect(),
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
    let (_, neighbors) = placement_neighbors(design, Some(net_weights), None, None);
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
    let (_, neighbors) = placement_neighbors(design, None, Some(sink_weights), None);
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
    sink_budgets: Option<&BTreeMap<(NetId, CellPinId), u32>>,
    max_moved_units: usize,
) -> Result<Placement, PnrError> {
    PlacementRefiner::new(design, device, constraints)?.refine_with_net_sink_weights_limited(
        placement,
        sink_weights,
        sink_budgets,
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
        sink_budgets: Option<&BTreeMap<(NetId, CellPinId), u32>>,
        max_moved_units: usize,
    ) -> Result<Placement, PnrError> {
        let (_, neighbors) =
            placement_neighbors(self.graph.design(), None, Some(sink_weights), sink_budgets);
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

    /// Proposes placements for one cell's unit that reduce the incident
    /// connections' delay excess over their allowance targets locally, or the
    /// physical path span during a broad critical-path move.
    ///
    /// # Errors
    ///
    /// Returns an error when the starting placement or timing table is invalid.
    ///
    /// # Panics
    ///
    /// Panics only if an internally validated placement-unit table is
    /// inconsistent with the design.
    #[allow(clippy::too_many_lines, clippy::too_many_arguments)]
    pub fn refine_cell_connection_delays(
        &self,
        placement: Placement,
        moving_cell: CellId,
        connections: &[(CellPinId, CellPinId)],
        targets_ps: &[u64],
        pip_delays_ps: &[u32],
        capacity_projection: Option<&RouteCapacityProjection>,
        max_move_distance: u64,
        max_candidates: usize,
    ) -> Result<Vec<Placement>, PnrError> {
        if pip_delays_ps.len() != self.graph.device().pips().len() {
            return Err(PnrError::InvalidRoutingCosts {
                reason: format!(
                    "expected {} PIP delays, received {}",
                    pip_delays_ps.len(),
                    self.graph.device().pips().len()
                ),
            });
        }
        if targets_ps.len() != connections.len() {
            return Err(PnrError::InvalidPlacement {
                reason: format!(
                    "expected {} connection targets, received {}",
                    connections.len(),
                    targets_ps.len()
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
        // Local moves score by the routed-delay excess over the per-connection
        // allowance targets so already-satisfied connections stop absorbing
        // moves. Broad path moves stay span-ranked: measuring excess on every
        // distant candidate cost ~30 s per run without moving WNS, because
        // span already orders long-haul proposals well enough before the full
        // route trial decides.
        let current_excess = if broad_path_move {
            None
        } else {
            assignment_connection_excess(
                &self.graph,
                self.constraints,
                unit,
                &current,
                moving_cell,
                connections,
                targets_ps,
                &placed,
                pip_delays_ps,
            )
            .map(|(excess, _)| excess)
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
        if !broad_path_move && current_excess.is_none() {
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
            if broad_path_move {
                if span >= current_span {
                    continue;
                }
                best.push((span, 0, assignment.to_vec()));
            } else {
                let Some((excess, _)) = assignment_connection_excess(
                    &self.graph,
                    self.constraints,
                    unit,
                    assignment,
                    moving_cell,
                    connections,
                    targets_ps,
                    &placed,
                    pip_delays_ps,
                ) else {
                    continue;
                };
                let current_excess = current_excess.expect("local moves require a current excess");
                if excess >= current_excess {
                    continue;
                }
                best.push((excess, span, assignment.to_vec()));
            }
        }
        best.sort_unstable_by(|left, right| {
            (left.0, left.1, left.2.as_slice()).cmp(&(right.0, right.1, right.2.as_slice()))
        });
        best.dedup_by(|left, right| left.2 == right.2);
        if broad_path_move && let Some(projection) = capacity_projection {
            // Physical span is only a cheap coarse index.  Project a small
            // neighborhood through the incumbent route topology, including
            // which lower-criticality arcs would have to retreat, before
            // paying for a complete negotiated route and STA trial.
            const PROJECTION_SHORTLIST: usize = 16;
            best.truncate(PROJECTION_SHORTLIST.max(max_candidates));
            let fallback = best.clone();
            let mut projected = best
                .into_iter()
                .filter_map(|(span, _, assignment)| {
                    assignment_connection_projected_cost(
                        &self.graph,
                        self.constraints,
                        unit,
                        &assignment,
                        moving_cell,
                        connections,
                        &placed,
                        pip_delays_ps,
                        projection,
                    )
                    .map(|cost| (cost, span, assignment))
                })
                .collect::<Vec<_>>();
            if projected.is_empty() {
                best = fallback;
            } else {
                projected.sort_unstable_by(|left, right| {
                    (left.0, left.1, left.2.as_slice()).cmp(&(right.0, right.1, right.2.as_slice()))
                });
                best = projected;
            }
        }
        best.truncate(max_candidates.max(1));
        if broad_path_move {
            // Broad topology search operates on physical tile nodes.  BEL
            // slots inside one tile are a lower hierarchy level and produced
            // nearly identical negotiated routes; detailed local refinement
            // still resolves those slots later.  Do not spend multiple full
            // route+STA trials on the same coarse node.
            let mut seen_points = BTreeSet::new();
            best.retain(|(_, _, assignment)| {
                seen_points.insert(device.bels()[assignment[moving_column].0].point)
            });
        }
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

#[allow(clippy::too_many_arguments)]
fn assignment_connection_projected_cost(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    unit: &PlacementUnit,
    assignment: &[BelId],
    moving_cell: CellId,
    connections: &[(CellPinId, CellPinId)],
    placed: &[Option<BelId>],
    pip_delays_ps: &[u32],
    projection: &RouteCapacityProjection,
) -> Option<u64> {
    let design = graph.design();
    let mut total = 0_u64;
    for &(driver_pin, sink_pin) in connections {
        let driver = design.pins().get(driver_pin.0)?;
        let sink = design.pins().get(sink_pin.0)?;
        if driver.cell != moving_cell && sink.cell != moving_cell {
            return None;
        }
        let net = driver.net()?;
        let driver_bel = assignment_bel(unit, assignment, driver.cell, placed)?;
        let sink_bel = assignment_bel(unit, assignment, sink.cell, placed)?;
        let driver_wire = candidate_pin_wire(graph, constraints, driver_pin, driver_bel)?;
        let sink_wire = candidate_pin_wire(graph, constraints, sink_pin, sink_bel)?;
        total = total.saturating_add(local_connection_projected_cost(
            graph,
            driver_wire,
            sink_wire,
            pip_delays_ps,
            net,
            projection,
        )?);
    }
    Some(total)
}

fn local_connection_projected_cost(
    graph: &UnifiedGraph<'_>,
    start: WireId,
    goal: WireId,
    pip_delays_ps: &[u32],
    net: NetId,
    projection: &RouteCapacityProjection,
) -> Option<u64> {
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
    while let Some(Reverse((cost, hops, wire))) = queue.pop() {
        if wire == goal {
            return Some(cost);
        }
        if hops == MAX_LOCAL_HOPS || best.get(&(wire, hops)).is_some_and(|known| *known < cost) {
            continue;
        }
        for (neighbor, pip) in graph.routing_neighbors(wire).ok()? {
            if !point_inside_corridor(device.wires()[neighbor.0].point, corridor) {
                continue;
            }
            let next_hops = hops + 1;
            let conflict = projected_resource_penalty(
                projection.wire_owners.get(&neighbor),
                net,
                device.wires()[neighbor.0].capacity,
            )
            .saturating_add(projected_resource_penalty(
                projection.pip_owners.get(&pip),
                net,
                device.pips()[pip.0].capacity(),
            ));
            let next_cost = cost
                .saturating_add(u64::from(pip_delays_ps[pip.0]))
                .saturating_add(conflict);
            let key = (neighbor, next_hops);
            if best.get(&key).is_none_or(|known| next_cost < *known) {
                best.insert(key, next_cost);
                queue.push(Reverse((next_cost, next_hops, neighbor)));
            }
        }
    }
    None
}

fn projected_resource_penalty(
    owners: Option<&Vec<(NetId, u64)>>,
    moving_net: NetId,
    capacity: u16,
) -> u64 {
    const RIPUP_BASE_PS: u64 = 150;
    const CRITICALITY_PENALTY_PS: u64 = 10;
    let Some(owners) = owners else {
        return 0;
    };
    let mut victims = owners
        .iter()
        .filter(|(net, _)| *net != moving_net)
        .map(|&(_, criticality)| criticality)
        .collect::<Vec<_>>();
    let required = victims
        .len()
        .saturating_add(1)
        .saturating_sub(usize::from(capacity));
    if required == 0 {
        return 0;
    }
    victims.sort_unstable();
    victims
        .into_iter()
        .take(required)
        .fold(0, |total, criticality| {
            total.saturating_add(
                RIPUP_BASE_PS.saturating_add(criticality.saturating_mul(CRITICALITY_PENALTY_PS)),
            )
        })
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

/// Sums per-connection routing delay and its excess over the allowance target.
///
/// The excess objective lets a move that already satisfies some connections
/// concentrate on the ones still blowing their budget instead of chasing the
/// largest absolute delays.
#[allow(clippy::too_many_arguments)]
fn assignment_connection_excess(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    unit: &PlacementUnit,
    assignment: &[BelId],
    moving_cell: CellId,
    connections: &[(CellPinId, CellPinId)],
    targets_ps: &[u64],
    placed: &[Option<BelId>],
    pip_delays_ps: &[u32],
) -> Option<(u64, u64)> {
    let design = graph.design();
    let mut total = 0_u64;
    let mut excess = 0_u64;
    for (&(driver_pin, sink_pin), &target_ps) in connections.iter().zip(targets_ps) {
        let driver_cell = design.pins().get(driver_pin.0)?.cell;
        let sink_cell = design.pins().get(sink_pin.0)?.cell;
        if driver_cell != moving_cell && sink_cell != moving_cell {
            return None;
        }
        let driver_bel = assignment_bel(unit, assignment, driver_cell, placed)?;
        let sink_bel = assignment_bel(unit, assignment, sink_cell, placed)?;
        let driver_wire = candidate_pin_wire(graph, constraints, driver_pin, driver_bel)?;
        let sink_wire = candidate_pin_wire(graph, constraints, sink_pin, sink_bel)?;
        let delay = local_connection_delay(graph, driver_wire, sink_wire, pip_delays_ps)?;
        total = total.saturating_add(delay);
        excess = excess.saturating_add(delay.saturating_sub(target_ps));
    }
    Some((excess, total))
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
        for (neighbor, pip) in graph.routing_neighbors(wire).ok()? {
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
    /// Free Manhattan distance before this edge accrues placement cost.
    /// Derived from routed-delay budgets: a connection inside its delay
    /// budget behaves like slack rubber, and only the excess over the
    /// allowance is pulled like a spring. Zero means unconstrained.
    budget: u32,
}

fn refinement_edge_cost(edge: PlacementNeighbor, distance: u64) -> u64 {
    let distance = distance.saturating_sub(u64::from(edge.budget));
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
    let (degree, neighbors) = placement_neighbors(design, net_weights, sink_weights, None);

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
    let (_, neighbors) = placement_neighbors(design, None, Some(sink_weights), None);
    let mut candidate_cache = BTreeMap::new();
    let units = placement_units(graph, constraints, &mut candidate_cache)?;
    let mut unit_by_cell = vec![usize::MAX; design.cells().len()];
    let mut column_by_cell = vec![usize::MAX; design.cells().len()];
    let mut macro_offset_by_cell = vec![(0.0, 0.0); design.cells().len()];
    for (unit_index, unit) in units.iter().enumerate() {
        let reference = unit.choices.assignment(0);
        let origin = device.bels()[reference[0].0].point;
        for (column, (&cell, &bel)) in unit.cells.iter().zip(reference).enumerate() {
            unit_by_cell[cell.0] = unit_index;
            column_by_cell[cell.0] = column;
            let point = device.bels()[bel.0].point;
            macro_offset_by_cell[cell.0] = (
                f64::from(point.x) - f64::from(origin.x),
                f64::from(point.y) - f64::from(origin.y),
            );
        }
    }

    let fixed = units
        .iter()
        .map(|unit| {
            (unit.choices.len() == 1).then(|| device.bels()[unit.choices.assignment(0)[0].0].point)
        })
        .collect::<Vec<_>>();
    let center = Point::new(device.width() / 2, device.height() / 2);
    let mut diagonal = vec![CENTER_WEIGHT; units.len()];
    let mut rhs_x = vec![CENTER_WEIGHT * f64::from(center.x); units.len()];
    let mut rhs_y = vec![CENTER_WEIGHT * f64::from(center.y); units.len()];
    let mut adjacency = vec![Vec::<(usize, f64)>::new(); units.len()];
    for (cell_index, edges) in neighbors.iter().enumerate() {
        let left = unit_by_cell[cell_index];
        for edge in edges {
            let right_cell = edge.cell.0;
            let right = unit_by_cell[right_cell];
            if left >= right {
                continue;
            }
            let weight =
                f64::from(u32::try_from(edge.weight).expect("placement edge weight fits u32"));
            let (left_offset_x, left_offset_y) = macro_offset_by_cell[cell_index];
            let (right_offset_x, right_offset_y) = macro_offset_by_cell[right_cell];
            match (fixed[left], fixed[right]) {
                (Some(_), Some(_)) => {}
                (Some(_), None) => {
                    let left_bel = units[left].choices.assignment(0)[column_by_cell[cell_index]];
                    let point = device.bels()[left_bel.0].point;
                    diagonal[right] += weight;
                    rhs_x[right] += weight * (f64::from(point.x) - right_offset_x);
                    rhs_y[right] += weight * (f64::from(point.y) - right_offset_y);
                }
                (None, Some(_)) => {
                    let right_bel = units[right].choices.assignment(0)[column_by_cell[right_cell]];
                    let point = device.bels()[right_bel.0].point;
                    diagonal[left] += weight;
                    rhs_x[left] += weight * (f64::from(point.x) - left_offset_x);
                    rhs_y[left] += weight * (f64::from(point.y) - left_offset_y);
                }
                (None, None) => {
                    diagonal[left] += weight;
                    diagonal[right] += weight;
                    adjacency[left].push((right, weight));
                    adjacency[right].push((left, weight));
                    rhs_x[left] += weight * (right_offset_x - left_offset_x);
                    rhs_x[right] += weight * (left_offset_x - right_offset_x);
                    rhs_y[left] += weight * (right_offset_y - left_offset_y);
                    rhs_y[right] += weight * (left_offset_y - right_offset_y);
                }
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
    let units_per_point = analytic_spread_units_per_point(units, &movable, device);
    let occupied_points = count.div_ceil(units_per_point);
    let aspect = f64::from(device.width()) / f64::from(device.height());
    let columns =
        ceil_coordinate((f64::from(occupied_points) * aspect).sqrt()).clamp(1, device.width());
    let rows = occupied_points.div_ceil(columns).clamp(1, device.height());
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
    let units_per_column =
        usize::try_from(rows * units_per_point).expect("spread column capacity fits usize");
    for (column, chunk) in movable.chunks_mut(units_per_column).enumerate() {
        chunk.sort_by(|&left, &right| {
            y[left]
                .total_cmp(&y[right])
                .then_with(|| x[left].total_cmp(&x[right]))
                .then_with(|| units[left].cells[0].cmp(&units[right].cells[0]))
        });
        for (slot, &index) in chunk.iter().enumerate() {
            let column = u32::try_from(column).expect("spread column fits u32");
            let row = u32::try_from(slot).expect("spread slot fits u32") / units_per_point;
            x[index] = f64::from(start_x + column);
            y[index] = f64::from(start_y + row);
        }
    }
    (x, y)
}

fn analytic_spread_units_per_point(
    units: &[PlacementUnit],
    movable: &[usize],
    device: &Device,
) -> u32 {
    let mut capacity_by_choices = BTreeMap::new();
    let mut capacities = movable
        .iter()
        .map(|&index| {
            let choices = &units[index].choices;
            *capacity_by_choices
                .entry(choices.cache_key())
                .or_insert_with(|| {
                    let mut by_point = BTreeMap::<Point, u32>::new();
                    for choice in 0..choices.len() {
                        let point = device.bels()[choices.assignment(choice)[0].0].point;
                        *by_point.entry(point).or_default() += 1;
                    }
                    by_point.values().copied().max().unwrap_or(1)
                })
        })
        .collect::<Vec<_>>();
    capacities.sort_unstable();
    let physical_capacity = capacities[capacities.len() / 2].clamp(1, 32);
    // Leave ample whitespace for legalization and routing, but model that a
    // physical tile can host more than one independently placed unit. The old
    // one-unit-per-coordinate spread expanded sparse ECP5 designs to roughly
    // three times nextpnr's placement area before legalization even began.
    physical_capacity.saturating_mul(3).div_ceil(8).max(1)
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
    sink_budgets: Option<&BTreeMap<(NetId, CellPinId), u32>>,
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
            let budget = sink_budgets
                .and_then(|budgets| budgets.get(&(NetId(net_index), sink_pin)))
                .copied()
                .unwrap_or(0);
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
                        budget,
                    });
                    neighbors[sink.0].push(PlacementNeighbor {
                        cell: driver,
                        weight: edge_weight,
                        timing_driven: timing_weight > 1,
                        budget,
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
        for dy in 0..=radius {
            let dx = radius - dy;
            for y in ring_coordinates(target.y, dy, device.height()) {
                for x in ring_coordinates(target.x, dx, device.width()) {
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
        }
        let enough = if point_usage.is_some() {
            !nearest.is_empty()
        } else {
            nearest.len() >= PLACEMENT_REFINEMENT_CANDIDATES
        };
        if enough {
            nearest.sort_unstable();
            nearest.truncate(PLACEMENT_REFINEMENT_CANDIDATES);
            break;
        }
    }
    nearest
}

/// Coordinates exactly `offset` away from `center`, clipped to `extent`.
///
/// The zero offset yields only the center so ring enumeration never visits a
/// coordinate twice.
fn ring_coordinates(center: u32, offset: u32, extent: u32) -> impl IntoIterator<Item = u32> {
    let minus = center.checked_sub(offset);
    let plus = (offset != 0).then(|| center.checked_add(offset)).flatten();
    [minus, plus]
        .into_iter()
        .flatten()
        .filter(move |&value| value < extent)
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
        let driver_cell = design.pins()[net.driver.0].cell;
        let driver_bel = placement
            .bel(driver_cell)
            .ok_or(PnrError::MissingPlacement { cell: driver_cell })?;
        let driver_wire = bound_wire(graph, placement, net.driver, driver_bel)?;
        let rebuilt = NetRoute::new(net_id, route.arcs.clone());
        if rebuilt.wire_refs != route.wire_refs || rebuilt.pip_refs != route.pip_refs {
            return Err(PnrError::InvalidRoutingConstraint {
                net: net_id,
                reason: "route resource reference counts are stale".into(),
            });
        }
        let mut routed_sinks = BTreeSet::new();
        for arc in &route.arcs {
            if arc.wires.first().copied() != Some(driver_wire) {
                return Err(PnrError::InvalidRoutingConstraint {
                    net: net_id,
                    reason: "route arc does not start at the placed driver wire".into(),
                });
            }
            if arc.pips.len().saturating_add(1) != arc.wires.len() {
                return Err(PnrError::InvalidRoutingConstraint {
                    net: net_id,
                    reason: "route arc does not have one PIP between each wire".into(),
                });
            }
            if arc.wires.iter().copied().collect::<BTreeSet<_>>().len() != arc.wires.len() {
                return Err(PnrError::InvalidRoutingConstraint {
                    net: net_id,
                    reason: "route arc contains a cycle".into(),
                });
            }
            for ((&from, &to), &pip_id) in arc
                .wires
                .iter()
                .zip(arc.wires.iter().skip(1))
                .zip(&arc.pips)
            {
                let Some(pip) = device.pips().get(pip_id.0) else {
                    return Err(PnrError::InvalidRoutingConstraint {
                        net: net_id,
                        reason: format!("unknown PIP {pip_id:?}"),
                    });
                };
                if !((pip.from() == from && pip.to() == to)
                    || (pip.bidirectional() && pip.from() == to && pip.to() == from))
                {
                    return Err(PnrError::InvalidRoutingConstraint {
                        net: net_id,
                        reason: format!("PIP {pip_id:?} does not connect its adjacent arc wires"),
                    });
                }
            }
            if let Some(sink) = arc.sink {
                if !net.sinks.contains(&sink) || !routed_sinks.insert(sink) {
                    return Err(PnrError::InvalidRoutingConstraint {
                        net: net_id,
                        reason: format!("sink pin {} is unknown or has multiple arcs", sink.0),
                    });
                }
                let sink_cell = design.pins()[sink.0].cell;
                let sink_bel = placement
                    .bel(sink_cell)
                    .ok_or(PnrError::MissingPlacement { cell: sink_cell })?;
                if arc.wires.last().copied() != Some(bound_wire(graph, placement, sink, sink_bel)?)
                {
                    return Err(PnrError::InvalidRoutingConstraint {
                        net: net_id,
                        reason: format!("route arc ends away from sink pin {}", sink.0),
                    });
                }
            }
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
    for (&(net, sink), &criticality) in &costs.sink_criticalities {
        let Some(net_data) = graph.design().nets().get(net.0) else {
            return Err(PnrError::InvalidRoutingCosts {
                reason: format!("arc criticality names unknown net {}", net.0),
            });
        };
        if !net_data.sinks.contains(&sink) {
            return Err(PnrError::InvalidRoutingCosts {
                reason: format!("pin {} is not a sink of net {}", sink.0, net.0),
            });
        }
        if !(1..=64).contains(&criticality) {
            return Err(PnrError::InvalidRoutingCosts {
                reason: format!(
                    "net {} sink {} criticality {criticality} is outside 1..=64",
                    net.0, sink.0
                ),
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

/// Pin-to-wire resolutions for one fixed placement, computed once.
///
/// Resolving a logical pin scans its BEL's pins with name comparisons, and
/// negotiated routing resolves the same pins on every net and iteration, so
/// this cache performs each scan once per placement instead of once per use.
#[derive(Clone, Debug)]
struct PinWireCache {
    wires: Vec<Option<WireId>>,
}

impl PinWireCache {
    fn build(graph: &UnifiedGraph<'_>, placement: &Placement) -> Self {
        let design = graph.design();
        let mut wires = Vec::with_capacity(design.pins().len());
        for index in 0..design.pins().len() {
            let pin = CellPinId(index);
            let cell = design.pins()[pin.0].cell;
            let wire = placement
                .bel(cell)
                .and_then(|bel| bound_wire(graph, placement, pin, bel).ok());
            wires.push(wire);
        }
        Self { wires }
    }

    /// Resolves a logical pin, falling back to the error-producing scan when
    /// the cached resolution failed during construction.
    fn resolve(
        &self,
        graph: &UnifiedGraph<'_>,
        placement: &Placement,
        cell_pin: CellPinId,
        bel: BelId,
    ) -> Result<WireId, PnrError> {
        self.wires[cell_pin.0].map_or_else(|| bound_wire(graph, placement, cell_pin, bel), Ok)
    }
}

#[cfg(test)]
fn route_reaches_all_sinks(
    graph: &UnifiedGraph<'_>,
    placement: &Placement,
    pin_wires: &PinWireCache,
    route: &NetRoute,
) -> Result<bool, PnrError> {
    let net = &graph.design().nets()[route.net.0];
    for &sink in &net.sinks {
        let cell = graph.design().pins()[sink.0].cell;
        let bel = placement
            .bel(cell)
            .ok_or(PnrError::MissingPlacement { cell })?;
        let sink_wire = pin_wires.resolve(graph, placement, sink, bel)?;
        if route
            .arc(sink)
            .is_none_or(|arc| arc.wires.last().copied() != Some(sink_wire))
        {
            return Ok(false);
        }
    }
    Ok(true)
}

#[allow(clippy::too_many_lines)]
fn route(
    graph: &UnifiedGraph<'_>,
    placement: &Placement,
    constraints: &RoutingConstraints,
    costs: Option<&RoutingCosts>,
    workspace: &mut RoutingWorkspace,
    mut routes: Vec<Option<NetRoute>>,
    progress: &mut impl FnMut(RoutingProgress),
) -> Result<Vec<NetRoute>, PnrError> {
    let design = graph.design();
    let metadata = RoutingResourceMetadata {
        wire_points: &workspace.wire_points,
        wire_capacities: &workspace.wire_capacities,
        pip_capacities: &workspace.pip_capacities,
    };
    let wire_occupancy = &mut workspace.wire_occupancy;
    let pip_occupancy = &mut workspace.pip_occupancy;
    let wire_history = &mut workspace.wire_history;
    let pip_history = &mut workspace.pip_history;
    let mut overuse = OveruseTracker::default();
    for route in routes.iter().flatten() {
        for wire in route.wires() {
            track_entry(
                &mut overuse.wires,
                wire_occupancy[wire.0],
                metadata.wire_capacities[wire.0],
                wire.0,
            );
        }
        for pip in route.pips() {
            track_entry(
                &mut overuse.pips,
                pip_occupancy[pip.0],
                metadata.pip_capacities[pip.0],
                pip.0,
            );
        }
    }
    let pin_wires = PinWireCache::build(graph, placement);
    let mut routing_order = routing_order(design, constraints, costs);
    routing_order.sort_unstable();
    let mut dirty = BTreeMap::<usize, BTreeSet<CellPinId>>::new();
    for (index, (net, route)) in design.nets().iter().zip(&routes).enumerate() {
        let route = route.as_ref();
        for &sink in &net.sinks {
            if route.is_none_or(|route| route.arc(sink).is_none()) {
                dirty.entry(index).or_default().insert(sink);
            }
        }
    }
    let max_iterations = costs.map_or(MAX_ROUTING_ITERATIONS, RoutingCosts::max_iterations);
    for iteration in 0..max_iterations {
        let present_factor = 1_u32 << iteration.min(12);
        progress(RoutingProgress::Iteration {
            iteration,
            nets: dirty.len(),
        });
        for (&index, dirty_sinks) in &dirty {
            if let Some(previous) = routes[index].take() {
                let preserved = NetRoute::new(
                    previous.net,
                    previous
                        .arcs
                        .iter()
                        .filter(|arc| arc.sink.is_none_or(|sink| !dirty_sinks.contains(&sink)))
                        .cloned()
                        .collect(),
                );
                for wire in previous
                    .wires()
                    .filter(|&wire| preserved.wire_ref_count(wire) == 0)
                {
                    wire_occupancy[wire.0] -= 1;
                    track_entry(
                        &mut overuse.wires,
                        wire_occupancy[wire.0],
                        metadata.wire_capacities[wire.0],
                        wire.0,
                    );
                }
                for pip in previous
                    .pips()
                    .filter(|&pip| preserved.pip_ref_count(pip) == 0)
                {
                    pip_occupancy[pip.0] -= 1;
                    track_entry(
                        &mut overuse.pips,
                        pip_occupancy[pip.0],
                        metadata.pip_capacities[pip.0],
                        pip.0,
                    );
                }
                routes[index] = Some(preserved);
            }
        }
        let mut ordinal = 0;
        for &(_, index) in &routing_order {
            if !dirty.contains_key(&index) {
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
            let preserved = routes[index].take();
            let route = route_net(
                graph,
                placement,
                &pin_wires,
                preserved.as_ref(),
                net_id,
                wire_occupancy,
                pip_occupancy,
                wire_history,
                pip_history,
                present_factor,
                costs,
                &mut workspace.search,
                &mut workspace.tree_arrival_ps,
                metadata,
            )?;
            for wire in route.wires().filter(|&wire| {
                preserved
                    .as_ref()
                    .is_none_or(|old| old.wire_ref_count(wire) == 0)
            }) {
                increment_occupancy(wire_occupancy, &mut workspace.touched_wires, wire.0);
                track_entry(
                    &mut overuse.wires,
                    wire_occupancy[wire.0],
                    metadata.wire_capacities[wire.0],
                    wire.0,
                );
            }
            for pip in route.pips().filter(|&pip| {
                preserved
                    .as_ref()
                    .is_none_or(|old| old.pip_ref_count(pip) == 0)
            }) {
                increment_occupancy(pip_occupancy, &mut workspace.touched_pips, pip.0);
                track_entry(
                    &mut overuse.pips,
                    pip_occupancy[pip.0],
                    metadata.pip_capacities[pip.0],
                    pip.0,
                );
            }
            routes[index] = Some(route);
        }

        // History grows once per iteration for exactly the resources that are
        // currently overused; the trackers make that O(conflicts) instead of
        // a full device rescan, with identical resulting history values.
        for &index in &overuse.wires {
            let excess = wire_occupancy[index] - metadata.wire_capacities[index];
            wire_history[index] = wire_history[index].saturating_add(u32::from(excess));
        }
        for &index in &overuse.pips {
            let excess = pip_occupancy[index] - metadata.pip_capacities[index];
            pip_history[index] = pip_history[index].saturating_add(u32::from(excess));
        }
        let overused_wires = overuse.wires.len();
        let overused_pips = overuse.pips.len();
        if overused_wires == 0 && overused_pips == 0 {
            return Ok(routes
                .into_iter()
                .map(|route| route.expect("every net was routed in this iteration"))
                .collect());
        }
        dirty = congested_route_arcs(
            metadata,
            &routes,
            constraints,
            costs,
            wire_occupancy,
            pip_occupancy,
        );
    }

    Err(PnrError::CongestionNotResolved {
        iterations: max_iterations,
        overused_wires: overuse.wires.len(),
        overused_pips: overuse.pips.len(),
    })
}

fn increment_occupancy(occupancy: &mut [u16], touched: &mut Vec<usize>, index: usize) {
    if occupancy[index] == 0 {
        touched.push(index);
    }
    occupancy[index] += 1;
}

/// Deterministic net routing order: locked trees first, then criticality,
/// hold constraints, fanout, and stable ID.
type RoutingOrderEntry = (
    (bool, Reverse<u64>, Reverse<bool>, Reverse<usize>, usize),
    usize,
);

fn routing_order(
    design: &Design,
    constraints: &RoutingConstraints,
    costs: Option<&RoutingCosts>,
) -> Vec<RoutingOrderEntry> {
    let hold_constrained_nets = costs
        .map(|costs| {
            costs
                .sink_min_delays_ps
                .keys()
                .map(|(net, _)| *net)
                .collect::<BTreeSet<_>>()
        })
        .unwrap_or_default();
    (0..design.nets().len())
        .map(|index| {
            (
                routing_order_key(design, constraints, costs, &hold_constrained_nets, index),
                index,
            )
        })
        .collect()
}

fn routing_order_key(
    design: &Design,
    constraints: &RoutingConstraints,
    costs: Option<&RoutingCosts>,
    hold_constrained_nets: &BTreeSet<NetId>,
    index: usize,
) -> (bool, Reverse<u64>, Reverse<bool>, Reverse<usize>, usize) {
    let net = NetId(index);
    let criticality = costs
        .and_then(|costs| costs.net_criticalities.get(&net))
        .copied()
        .unwrap_or(0);
    let hold_constrained = hold_constrained_nets.contains(&net);
    (
        !constraints.routes().contains_key(&net),
        Reverse(criticality),
        Reverse(hold_constrained),
        Reverse(design.nets()[index].sinks.len()),
        index,
    )
}

#[derive(Default)]
struct ArcOwnerIndex {
    wires: HashMap<WireId, BTreeMap<NetId, BTreeSet<Option<CellPinId>>>>,
    pips: HashMap<PipId, BTreeMap<NetId, BTreeSet<Option<CellPinId>>>>,
}

impl ArcOwnerIndex {
    fn build(
        routes: &[Option<NetRoute>],
        metadata: RoutingResourceMetadata<'_>,
        wire_occupancy: &[u16],
        pip_occupancy: &[u16],
    ) -> Self {
        let mut owners = Self::default();
        for route in routes.iter().flatten() {
            for arc in &route.arcs {
                for &wire in &arc.wires {
                    if wire_occupancy[wire.0] > metadata.wire_capacities[wire.0] {
                        owners
                            .wires
                            .entry(wire)
                            .or_default()
                            .entry(route.net)
                            .or_default()
                            .insert(arc.sink);
                    }
                }
                for &pip in &arc.pips {
                    if pip_occupancy[pip.0] > metadata.pip_capacities[pip.0] {
                        owners
                            .pips
                            .entry(pip)
                            .or_default()
                            .entry(route.net)
                            .or_default()
                            .insert(arc.sink);
                    }
                }
            }
        }
        owners
    }
}

fn congested_route_arcs(
    metadata: RoutingResourceMetadata<'_>,
    routes: &[Option<NetRoute>],
    constraints: &RoutingConstraints,
    costs: Option<&RoutingCosts>,
    wire_occupancy: &[u16],
    pip_occupancy: &[u16],
) -> BTreeMap<usize, BTreeSet<CellPinId>> {
    let owners = ArcOwnerIndex::build(routes, metadata, wire_occupancy, pip_occupancy);
    let mut dirty = BTreeMap::<usize, BTreeSet<CellPinId>>::new();
    for (wire, resource_owners) in owners.wires {
        select_arc_victims(
            &resource_owners,
            usize::from(metadata.wire_capacities[wire.0]),
            constraints,
            costs,
            &mut dirty,
        );
    }
    for (pip, resource_owners) in owners.pips {
        select_arc_victims(
            &resource_owners,
            usize::from(metadata.pip_capacities[pip.0]),
            constraints,
            costs,
            &mut dirty,
        );
    }
    dirty
}

fn select_arc_victims(
    owners: &BTreeMap<NetId, BTreeSet<Option<CellPinId>>>,
    capacity: usize,
    constraints: &RoutingConstraints,
    costs: Option<&RoutingCosts>,
    dirty: &mut BTreeMap<usize, BTreeSet<CellPinId>>,
) {
    if owners.len() <= capacity {
        return;
    }
    let mut ranked = owners
        .iter()
        .map(|(&net, sinks)| {
            let locked = sinks.iter().any(|sink| {
                constraints
                    .routes()
                    .get(&net)
                    .is_some_and(|route| route.arcs.iter().any(|arc| arc.sink == *sink))
            });
            let criticality = sinks
                .iter()
                .filter_map(|sink| sink.map(|sink| routing_arc_criticality(costs, net, sink)))
                .max()
                .unwrap_or(u64::MAX);
            ((Reverse(locked), Reverse(criticality), net), net, sinks)
        })
        .collect::<Vec<_>>();
    ranked.sort_by_key(|(key, _, _)| *key);
    for &(_, net, sinks) in ranked.iter().skip(capacity) {
        for &sink in sinks.iter().flatten() {
            if constraints
                .routes()
                .get(&net)
                .is_none_or(|route| route.arc(sink).is_none())
            {
                dirty.entry(net.0).or_default().insert(sink);
            }
        }
    }
}

#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn route_net(
    graph: &UnifiedGraph<'_>,
    placement: &Placement,
    pin_wires: &PinWireCache,
    fixed: Option<&NetRoute>,
    net_id: NetId,
    wire_occupancy: &[u16],
    pip_occupancy: &[u16],
    wire_history: &[u32],
    pip_history: &[u32],
    present_factor: u32,
    costs: Option<&RoutingCosts>,
    search: &mut RouteSearch,
    tree_arrival_ps: &mut [u64],
    metadata: RoutingResourceMetadata<'_>,
) -> Result<NetRoute, PnrError> {
    let design = graph.design();
    let device = graph.device();
    let net = &design.nets()[net_id.0];
    let driver_cell = design.pins()[net.driver.0].cell;
    let driver_bel = placement
        .bel(driver_cell)
        .ok_or(PnrError::MissingPlacement { cell: driver_cell })?;
    let driver_wire = pin_wires.resolve(graph, placement, net.driver, driver_bel)?;
    let mut arcs = fixed.map_or_else(Vec::new, |route| route.arcs.clone());
    let mut tree_wires = fixed.map_or_else(BTreeSet::new, |route| route.wires().collect());
    tree_wires.insert(driver_wire);
    let mut parent = BTreeMap::<WireId, (WireId, PipId)>::new();
    for arc in &arcs {
        for ((&from, &to), &pip) in arc
            .wires
            .iter()
            .zip(arc.wires.iter().skip(1))
            .zip(&arc.pips)
        {
            parent.entry(to).or_insert((from, pip));
        }
    }
    let delay_quantum_ps = costs.map_or(ROUTING_DELAY_QUANTUM_PS, |costs| {
        if costs.detailed_timing_nets.contains(&net_id) {
            costs.detailed_delay_quantum_ps
        } else {
            ROUTING_DELAY_QUANTUM_PS
        }
    });
    // `tree_arrival_ps` is a caller-owned scratch buffer reused across nets.
    // The sentinel means "wire is outside the routed tree". Tree wires always
    // carry a real nonnegative arrival, matching the previous map-based
    // representation where absent entries were only ever written by insertion
    // rather than merged with a zero seed.
    tree_arrival_ps[driver_wire.0] = 0;
    for arc in &arcs {
        let mut arrival_ps = 0_u64;
        for (&wire, &pip) in arc.wires.iter().skip(1).zip(&arc.pips) {
            arrival_ps = arrival_ps.saturating_add(u64::from(
                costs.map_or(0, |costs| costs.pip_delays_ps[pip.0]),
            ));
            tree_arrival_ps[wire.0] = match tree_arrival_ps[wire.0] {
                UNROUTED_ARRIVAL_PS => arrival_ps,
                known => known.min(arrival_ps),
            };
        }
    }
    let sinks = ordered_sinks(net_id, &net.sinks, costs);
    for sink_pin in &sinks {
        let sink_cell = design.pins()[sink_pin.0].cell;
        let sink_bel = placement
            .bel(sink_cell)
            .ok_or(PnrError::MissingPlacement { cell: sink_cell })?;
        let sink_wire = pin_wires.resolve(graph, placement, *sink_pin, sink_bel)?;
        let minimum_arrival_ps = costs
            .and_then(|costs| costs.sink_min_delays_ps.get(&(net_id, *sink_pin)))
            .copied()
            .unwrap_or(0);
        let criticality = routing_arc_criticality(costs, net_id, *sink_pin);
        if arcs.iter().any(|arc| arc.sink == Some(*sink_pin)) {
            if tree_arrival_ps[sink_wire.0] >= minimum_arrival_ps {
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
        if tree_wires.contains(&sink_wire) {
            if tree_arrival_ps[sink_wire.0] >= minimum_arrival_ps {
                arcs.push(
                    reconstruct_route_arc(*sink_pin, driver_wire, sink_wire, &parent).ok_or_else(
                        || PnrError::InvalidRoutingConstraint {
                            net: net_id,
                            reason: format!(
                                "tree path to sink pin {} has no unique parent",
                                sink_pin.0
                            ),
                        },
                    )?,
                );
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
                tree_arrival_ps,
                minimum_arrival_ps,
                metadata,
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
            let mut arrival_ps = tree_arrival_ps[path_wires
                .last()
                .expect("a routed path includes its tree start")
                .0];
            for (&wire, &pip) in path_wires.iter().rev().skip(1).zip(path_pips.iter().rev()) {
                let delay_ps = routed_tree_pip_delay(costs, pip, minimum_arrival_ps);
                arrival_ps = arrival_ps.saturating_add(u64::from(delay_ps));
                tree_arrival_ps[wire.0] = match tree_arrival_ps[wire.0] {
                    UNROUTED_ARRIVAL_PS => arrival_ps,
                    known => known.min(arrival_ps),
                };
            }
        } else {
            for &wire in &path_wires {
                tree_arrival_ps[wire.0] = 0;
            }
        }
        for (&wire, (&previous, &pip)) in path_wires
            .iter()
            .zip(path_wires.iter().skip(1).zip(&path_pips))
        {
            parent.insert(wire, (previous, pip));
        }
        tree_wires.extend(path_wires);
        arcs.push(
            reconstruct_route_arc(*sink_pin, driver_wire, sink_wire, &parent).ok_or_else(|| {
                PnrError::InvalidRoutingConstraint {
                    net: net_id,
                    reason: format!("new route to sink pin {} has no driver path", sink_pin.0),
                }
            })?,
        );
    }
    for &wire in &tree_wires {
        tree_arrival_ps[wire.0] = UNROUTED_ARRIVAL_PS;
    }
    Ok(NetRoute::new(net_id, arcs))
}

fn reconstruct_route_arc(
    sink: CellPinId,
    driver: WireId,
    sink_wire: WireId,
    parent: &BTreeMap<WireId, (WireId, PipId)>,
) -> Option<RouteArc> {
    reconstruct_endpoint_arc(Some(sink), driver, sink_wire, parent)
}

/// Arrival sentinel for wires outside the routed tree under optimization.
const UNROUTED_ARRIVAL_PS: u64 = u64::MAX;

fn ordered_sinks(net: NetId, sinks: &[CellPinId], costs: Option<&RoutingCosts>) -> Vec<CellPinId> {
    let mut ordered = sinks.to_vec();
    ordered.sort_by_key(|&sink| {
        let criticality = routing_arc_criticality(costs, net, sink);
        let minimum = costs
            .and_then(|costs| costs.sink_min_delays_ps.get(&(net, sink)))
            .copied()
            .unwrap_or(0);
        (Reverse(criticality), Reverse(minimum), sink)
    });
    ordered
}

fn routing_criticality(costs: Option<&RoutingCosts>, net: NetId) -> u64 {
    costs
        .and_then(|costs| costs.net_criticalities.get(&net))
        .copied()
        .unwrap_or(0)
}

fn routing_arc_criticality(costs: Option<&RoutingCosts>, net: NetId, sink: CellPinId) -> u64 {
    costs
        .and_then(|costs| costs.sink_criticalities.get(&(net, sink)).copied())
        .unwrap_or_else(|| routing_criticality(costs, net))
}

fn routed_tree_pip_delay(costs: &RoutingCosts, pip: PipId, minimum_arrival_ps: u64) -> u32 {
    if minimum_arrival_ps == 0 {
        costs.pip_delays_ps[pip.0]
    } else {
        costs.pip_min_delays_ps[pip.0]
    }
}

/// Indices whose occupancy currently exceeds capacity.
///
/// Maintained incrementally by [`track_entry`] so the per-iteration history
/// update touches only conflicting resources instead of rescanning every wire
/// and PIP on the device; resulting history values are identical to a full
/// rescan because entries are independent.
#[derive(Default)]
struct OveruseTracker {
    wires: BTreeSet<usize>,
    pips: BTreeSet<usize>,
}

fn track_entry(tracker: &mut BTreeSet<usize>, used: u16, capacity: u16, index: usize) {
    if used > capacity {
        tracker.insert(index);
    } else {
        tracker.remove(&index);
    }
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

#[derive(Debug)]
struct RouteSearch {
    epoch: u32,
    seen: Vec<u32>,
    /// Epoch-stamped members of the currently growing tree. Replaces
    /// `starts.contains` on every edge relaxation with one array load.
    start_mark: Vec<u32>,
    distance: Vec<u64>,
    arrival_ps: Vec<u64>,
    previous_wire: Vec<usize>,
    previous_pip: Vec<usize>,
    /// Frontier storage retained across sink and placement trials. Large
    /// critical searches can grow this to hundreds of thousands of entries;
    /// clearing keeps the allocation while preserving an empty logical queue.
    queue: BinaryHeap<Reverse<RouteQueueEntry>>,
}

type RouteQueueEntry = (u64, u64, u64, WireId);

type HoldRouteState = (WireId, u32);
type HoldRouteVisit = (u64, u64, Option<(HoldRouteState, PipId)>);

const ROUTING_ESTIMATE_BASE_DELAY_PS: u64 = 100;
const ROUTING_ESTIMATE_DELAY_PER_TILE_PS: u64 = 100;

impl RouteSearch {
    fn new(wire_count: usize) -> Self {
        Self {
            epoch: 0,
            seen: vec![0; wire_count],
            start_mark: vec![0; wire_count],
            distance: vec![0; wire_count],
            arrival_ps: vec![0; wire_count],
            previous_wire: vec![usize::MAX; wire_count],
            previous_pip: vec![usize::MAX; wire_count],
            queue: BinaryHeap::new(),
        }
    }

    /// Architecture-scaled remaining cost for timing-driven A*.
    ///
    /// Raw Manhattan distance is in tiles while the accumulated path score
    /// blends picosecond delay with congestion. Converting a lightweight
    /// geometry delay prediction into that same score keeps the heuristic
    /// strong without overwhelming detours onto fast long-line resources.
    fn remaining_cost_estimate(
        point: Point,
        goal: Point,
        criticality: u64,
        delay_quantum_ps: u64,
    ) -> u64 {
        let distance = point.manhattan(goal);
        if criticality == 0 {
            return distance;
        }
        let predicted_delay_ps = ROUTING_ESTIMATE_BASE_DELAY_PS
            .saturating_add(distance.saturating_mul(ROUTING_ESTIMATE_DELAY_PER_TILE_PS));
        let timing = timing_tree_cost(predicted_delay_ps, criticality, delay_quantum_ps);
        let hop_bias = distance
            .saturating_mul(ROUTING_CRITICALITY_SCALE - criticality)
            .div_ceil(ROUTING_CRITICALITY_SCALE);
        timing.saturating_add(hop_bias)
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
        tree_delays_ps: &[u64],
        minimum_arrival_ps: u64,
        metadata: RoutingResourceMetadata<'_>,
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
                metadata,
            );
        }
        self.epoch = self.epoch.wrapping_add(1);
        if self.epoch == 0 {
            self.seen.fill(0);
            self.start_mark.fill(0);
            self.epoch = 1;
        }
        let epoch = self.epoch;
        let device = graph.device();
        let goal_point = metadata.wire_points[goal.0];
        let corridor = (criticality != 0).then(|| {
            let start_point = starts
                .iter()
                .map(|start| metadata.wire_points[start.0])
                .min_by_key(|point| (point.manhattan(goal_point), *point))
                .expect("a route tree always contains its driver");
            routing_corridor(start_point, goal_point, device, TIMING_ROUTE_MARGIN)
        });
        self.queue.clear();
        for &start in starts {
            self.start_mark[start.0] = epoch;
            let arrival_ps = tree_delays_ps[start.0];
            let distance = timing_tree_cost(arrival_ps, criticality, delay_quantum_ps);
            self.seen[start.0] = epoch;
            self.distance[start.0] = distance;
            self.arrival_ps[start.0] = arrival_ps;
            self.previous_wire[start.0] = usize::MAX;
            self.previous_pip[start.0] = usize::MAX;
            self.queue.push(Reverse((
                distance.saturating_add(Self::remaining_cost_estimate(
                    metadata.wire_points[start.0],
                    goal_point,
                    criticality,
                    delay_quantum_ps,
                )),
                distance,
                arrival_ps,
                start,
            )));
        }

        while let Some(Reverse((_, distance, arrival_ps, wire))) = self.queue.pop() {
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

            for (neighbor, pip) in graph.routing_neighbors(wire).ok()? {
                if self.start_mark[neighbor.0] == epoch {
                    continue;
                }
                if corridor.is_some_and(|corridor| {
                    !point_inside_corridor(metadata.wire_points[neighbor.0], corridor)
                }) {
                    continue;
                }
                let congestion = congestion_cost(
                    wire_occupancy[neighbor.0],
                    metadata.wire_capacities[neighbor.0],
                    wire_history[neighbor.0],
                    present_factor,
                ) + congestion_cost(
                    pip_occupancy[pip.0],
                    metadata.pip_capacities[pip.0],
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
                let estimate = next_distance.saturating_add(Self::remaining_cost_estimate(
                    metadata.wire_points[neighbor.0],
                    goal_point,
                    criticality,
                    delay_quantum_ps,
                ));
                self.queue.push(Reverse((
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
                metadata,
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
    tree_delays_ps: &[u64],
    minimum_arrival_ps: u64,
    metadata: RoutingResourceMetadata<'_>,
) -> Option<(Vec<WireId>, Vec<PipId>)> {
    let goal_point = metadata.wire_points[goal.0];
    let mut visits = HashMap::<HoldRouteState, HoldRouteVisit>::new();
    let mut queue = BinaryHeap::new();
    for &start in starts {
        let arrival_ps = tree_delays_ps[start.0];
        let state = (start, hold_delay_bucket(arrival_ps, minimum_arrival_ps));
        let distance = timing_tree_cost(arrival_ps, criticality, ROUTING_DELAY_QUANTUM_PS);
        visits.insert(state, (distance, arrival_ps, None));
        queue.push(Reverse((
            distance.saturating_add(metadata.wire_points[start.0].manhattan(goal_point)),
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

        for (neighbor, pip) in graph.routing_neighbors(wire).ok()? {
            if starts.contains(&neighbor) {
                continue;
            }
            let congestion = congestion_cost(
                wire_occupancy[neighbor.0],
                metadata.wire_capacities[neighbor.0],
                wire_history[neighbor.0],
                present_factor,
            ) + congestion_cost(
                pip_occupancy[pip.0],
                metadata.pip_capacities[pip.0],
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
                .saturating_add(metadata.wire_points[neighbor.0].manhattan(goal_point));
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
/// Default routing delay quantum in picoseconds.
///
/// Detailed timing nets may pass a finer quantum, but measuring the AXI4
/// self-test showed the finest setting bought nothing: the final placement
/// and WNS were unchanged while small-trial routing slowed by ~27% from the
/// arrival-dimension state growth in [`routing_transition_cost`] searches.
pub const ROUTING_DELAY_QUANTUM_PS: u64 = 50;

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
        MAX_ROUTING_ITERATIONS, NetRoute, PinWireCache, Placement, PlacementConstraints,
        PlacementNeighbor, PnrError, RouteArc, RouteCapacityProjection, RouteSearch,
        RoutingConstraints, RoutingCosts, RoutingResourceMetadata, RoutingWorkspace,
        congested_route_arcs, place_analytically_with_net_sink_weights, place_and_route,
        place_with_constraints, placement_neighbors, projected_resource_penalty,
        refine_placement_with_net_sink_weights_limited, refine_placement_with_net_weights,
        refinement_edge_cost, retain_route_for_sinks, route_reaches_all_sinks,
        route_with_placement_and_progress, route_with_timing_costs_and_progress,
        route_with_workspace_and_progress, routing_corridor, routing_step_cost,
        routing_transition_cost, timing_tree_cost,
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
        assert!(result.routes[0].wires().next().is_some());
        assert!(result.routes[0].pips().next().is_some());
        assert_eq!(result.total_pips, result.routes[0].pips().len());
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
    fn analytical_placement_accounts_for_group_member_offsets() {
        let mut design = Design::new();
        let macro_root = design.add_cell("macro_root", ResourceKind::Logic);
        let macro_far = design.add_cell("macro_far", ResourceKind::Logic);
        let far_out = design
            .add_pin(macro_far, "out", PinDirection::Output)
            .unwrap();
        let sink = design.add_cell("sink", ResourceKind::Logic);
        let sink_in = design.add_pin(sink, "in", PinDirection::Input).unwrap();
        design.add_net("far_to_sink", far_out, [sink_in]).unwrap();
        let device = Device::rectangular_logic(9, 1).unwrap();
        let mut constraints = PlacementConstraints::new();
        constraints.add_group(
            [macro_root, macro_far],
            (0..6).map(|root| vec![BelId(root), BelId(root + 3)]),
        );

        let placement = place_analytically_with_net_sink_weights(
            &design,
            &device,
            &constraints,
            &BTreeMap::new(),
        )
        .unwrap();
        let far = device.bels()[placement.bel(macro_far).unwrap().0].point;
        let sink = device.bels()[placement.bel(sink).unwrap().0].point;

        assert!(far.manhattan(sink) <= 1, "far={far:?}, sink={sink:?}");
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
    fn timing_route_estimate_uses_the_path_cost_scale() {
        let source = Point::new(1, 1);
        let sink = Point::new(6, 1);

        assert_eq!(RouteSearch::remaining_cost_estimate(source, sink, 0, 50), 5);
        assert_eq!(
            RouteSearch::remaining_cost_estimate(source, sink, 32, 50),
            9
        );
        assert_eq!(
            RouteSearch::remaining_cost_estimate(source, sink, 64, 50),
            12
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
            budget: 0,
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
            None,
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
    fn reusable_workspace_does_not_leak_occupancy_between_trials() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(4, 1).unwrap();
        let placement =
            place_with_constraints(&design, &device, &PlacementConstraints::new()).unwrap();
        let mut workspace = RoutingWorkspace::new(&device);

        let first = route_with_workspace_and_progress(
            &design,
            &device,
            placement.clone(),
            &RoutingConstraints::new(),
            &mut workspace,
            |_| {},
        )
        .unwrap();
        let second = route_with_workspace_and_progress(
            &design,
            &device,
            placement,
            &RoutingConstraints::new(),
            &mut workspace,
            |_| {},
        )
        .unwrap();

        assert_eq!(first, second);
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
        assert!(partial.pips().len() < full.routes[0].pips().len());

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
                .pips()
                .all(|pip| rerouted.routes[0].pips().any(|item| item == pip))
        );
        assert!(
            route_reaches_all_sinks(
                &UnifiedGraph::new(&design, &device),
                &placement,
                &PinWireCache::build(&UnifiedGraph::new(&design, &device), &placement),
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
            None,
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
    fn local_routing_iteration_limit_stays_within_negotiator_bounds() {
        let mut costs = RoutingCosts::new(Vec::new(), BTreeMap::new());
        assert_eq!(costs.max_iterations(), MAX_ROUTING_ITERATIONS);
        costs.set_max_iterations(8);
        assert_eq!(costs.max_iterations(), 8);
        costs.set_max_iterations(0);
        assert_eq!(costs.max_iterations(), 1);
        costs.reset_max_iterations();
        assert_eq!(costs.max_iterations(), MAX_ROUTING_ITERATIONS);
        costs.set_max_iterations(MAX_ROUTING_ITERATIONS + 1);
        assert_eq!(costs.max_iterations(), MAX_ROUTING_ITERATIONS);
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
        let tree_delays = vec![100, 0];
        let wire_points = device
            .wires()
            .iter()
            .map(|wire| wire.point)
            .collect::<Vec<_>>();
        let wire_capacities = device
            .wires()
            .iter()
            .map(|wire| wire.capacity)
            .collect::<Vec<_>>();
        let pip_capacities = device
            .pips()
            .iter()
            .map(texo_model::Pip::capacity)
            .collect::<Vec<_>>();
        let metadata = RoutingResourceMetadata {
            wire_points: &wire_points,
            wire_capacities: &wire_capacities,
            pip_capacities: &pip_capacities,
        };

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
                metadata,
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
        let wire_points = device
            .wires()
            .iter()
            .map(|wire| wire.point)
            .collect::<Vec<_>>();
        let wire_capacities = device
            .wires()
            .iter()
            .map(|wire| wire.capacity)
            .collect::<Vec<_>>();
        let pip_capacities = device
            .pips()
            .iter()
            .map(texo_model::Pip::capacity)
            .collect::<Vec<_>>();
        let metadata = RoutingResourceMetadata {
            wire_points: &wire_points,
            wire_capacities: &wire_capacities,
            pip_capacities: &pip_capacities,
        };

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
                &[0],
                500,
                metadata,
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

    #[test]
    fn critical_arc_evicts_only_the_conflicting_noncritical_arc() {
        let critical_sink = texo_model::CellPinId(0);
        let conflicting_sink = texo_model::CellPinId(1);
        let retained_sink = texo_model::CellPinId(2);
        let shared = WireId(1);
        let routes = vec![
            Some(NetRoute::new(
                NetId(0),
                vec![RouteArc {
                    sink: Some(critical_sink),
                    wires: vec![WireId(0), shared, WireId(2)],
                    pips: vec![],
                }],
            )),
            Some(NetRoute::new(
                NetId(1),
                vec![
                    RouteArc {
                        sink: Some(conflicting_sink),
                        wires: vec![WireId(3), shared, WireId(4)],
                        pips: vec![],
                    },
                    RouteArc {
                        sink: Some(retained_sink),
                        wires: vec![WireId(3), WireId(5), WireId(6)],
                        pips: vec![],
                    },
                ],
            )),
        ];
        let wire_points = vec![Point::new(0, 0); 7];
        let wire_capacities = vec![1; 7];
        let pip_capacities = vec![];
        let metadata = RoutingResourceMetadata {
            wire_points: &wire_points,
            wire_capacities: &wire_capacities,
            pip_capacities: &pip_capacities,
        };
        let mut costs = RoutingCosts::new(vec![], BTreeMap::new());
        costs.set_sink_criticalities(BTreeMap::from([
            ((NetId(0), critical_sink), 64),
            ((NetId(1), conflicting_sink), 1),
            ((NetId(1), retained_sink), 1),
        ]));

        let dirty = congested_route_arcs(
            metadata,
            &routes,
            &RoutingConstraints::new(),
            Some(&costs),
            &[1, 2, 1, 1, 1, 1, 1],
            &[],
        );

        assert_eq!(
            dirty,
            BTreeMap::from([(NetId(1).0, BTreeSet::from([conflicting_sink]))])
        );
        assert!(!dirty[&NetId(1).0].contains(&retained_sink));
    }

    #[test]
    fn capacity_projection_prices_the_arc_using_each_resource() {
        let low_sink = texo_model::CellPinId(0);
        let critical_sink = texo_model::CellPinId(1);
        let owner = NetId(1);
        let moving = NetId(2);
        let low_only = WireId(0);
        let critical_only = WireId(1);
        let shared = WireId(2);
        let route = NetRoute::new(
            owner,
            vec![
                RouteArc {
                    sink: Some(low_sink),
                    wires: vec![low_only, shared],
                    pips: vec![],
                },
                RouteArc {
                    sink: Some(critical_sink),
                    wires: vec![critical_only, shared],
                    pips: vec![],
                },
            ],
        );
        let mut costs = RoutingCosts::new(vec![], BTreeMap::new());
        costs.set_sink_criticalities(BTreeMap::from([
            ((owner, low_sink), 1),
            ((owner, critical_sink), 64),
        ]));
        let projection = RouteCapacityProjection::new(&[route], &costs);

        assert_eq!(
            projected_resource_penalty(projection.wire_owners.get(&low_only), moving, 1),
            160
        );
        assert_eq!(
            projected_resource_penalty(projection.wire_owners.get(&critical_only), moving, 1),
            790
        );
        assert_eq!(
            projected_resource_penalty(projection.wire_owners.get(&shared), moving, 1),
            790
        );
        assert_eq!(
            projected_resource_penalty(projection.wire_owners.get(&shared), moving, 2),
            0
        );
        assert_eq!(
            projected_resource_penalty(projection.wire_owners.get(&shared), owner, 1),
            0
        );
    }
}
