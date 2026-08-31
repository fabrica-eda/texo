//! Deterministic reference placement and routing on the unified problem graph.

mod analytical_placement;
mod electrostatic_placement;
mod eplace;
mod global_placement;
mod instance_area;
mod legalization;
mod register_clustering;
mod routing_demand;

pub use routing_demand::{
    RoutingCapacityMap, RoutingChannelOrientation, RoutingDemandBin, RoutingDemandMap,
    routing_capacity_map, routing_demand_map, routing_demand_map_with_capacity,
};

use std::cmp::Reverse;
use std::collections::{BTreeMap, BTreeSet, BinaryHeap, HashMap};
use std::error::Error;
use std::fmt;
use std::hash::{BuildHasherDefault, Hasher};
use std::sync::Arc;

use texo_model::{
    BelId, BelPinId, CellId, CellPinId, Design, Device, ModelError, NetId, PipId, Point,
    ResourceKind, UnifiedGraph, WireId,
};

#[cfg(test)]
use analytical_placement::AnalyticalObjective;
use analytical_placement::{AnalyticalHypergraph, AxisEdge};

/// Cell-to-BEL bindings indexed by stable cell ID.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Placement {
    bindings: Vec<BelId>,
    pin_bindings: BTreeMap<CellPinId, BelPinId>,
}

/// Architecture-specific unloaded delay prediction used by detailed placement.
///
/// The placer owns connectivity, criticality, and normalization. Targets only
/// characterize the physical delay between two candidate BEL pins, including
/// dedicated local interconnect that a coordinate-only model cannot see.
pub trait PlacementDelayEstimator {
    /// Predicts the maximum data delay in picoseconds for one placed net arc.
    fn estimate_delay_ps(
        &self,
        driver_bel: BelId,
        driver_pin: BelPinId,
        sink_bel: BelId,
        sink_pin: BelPinId,
    ) -> u64;
}

/// One atomically placed group and its legal BEL assignments.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PlacementGroup {
    /// Cells assigned together, in assignment-column order.
    pub cells: Vec<CellId>,
    /// Legal assignments; every row must have one BEL per cell.
    pub assignments: Arc<[Vec<BelId>]>,
}

/// One target configuration value shared by multiple BELs in a physical site.
///
/// A placement is legal when every cell assigned to the same resource ID has
/// the same value. Resource IDs and values are opaque target-defined numbers.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PlacementSharedResource {
    cell_values: BTreeMap<CellId, u64>,
    bel_resources: BTreeMap<BelId, u64>,
}

type PlacementGroupRowIndex = Arc<BTreeMap<BelId, Vec<usize>>>;
type SharedPlacementGroupRowIndex = (Arc<[Vec<BelId>]>, PlacementGroupRowIndex);

/// Optional grouped/fixed placement rules supplied by a target packer.
#[derive(Clone, Debug, Default)]
pub struct PlacementConstraints {
    groups: Vec<PlacementGroup>,
    group_row_indexes: Vec<PlacementGroupRowIndex>,
    shared_group_row_indexes: BTreeMap<usize, SharedPlacementGroupRowIndex>,
    pin_bindings: BTreeMap<(CellPinId, BelId), BelPinId>,
    pin_name_bindings: BTreeMap<CellPinId, String>,
    shared_resources: Vec<PlacementSharedResource>,
}

impl PartialEq for PlacementConstraints {
    fn eq(&self, other: &Self) -> bool {
        self.groups == other.groups
            && self.pin_bindings == other.pin_bindings
            && self.pin_name_bindings == other.pin_name_bindings
            && self.shared_resources == other.shared_resources
    }
}

impl Eq for PlacementConstraints {}

/// Target-supplied immutable portions of logical net trees.
///
/// This is used for architecture resources whose legal topology cannot be
/// discovered from local congestion costs alone, such as an ECP5 primary
/// clock spine. The generic router grows any still-unconnected sinks from the
/// locked tree and accounts for every fixed wire and PIP normally.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct RoutingConstraints {
    routes: BTreeMap<NetId, Arc<NetRoute>>,
    blocked_pips: BTreeSet<PipId>,
    blocked_pip_words: Option<Arc<Vec<u64>>>,
}

/// Characterized costs used by timing-driven negotiated routing.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RoutingCosts {
    pip_delays_ps: Arc<[u32]>,
    pip_min_delays_ps: Arc<[u32]>,
    net_criticalities: BTreeMap<NetId, u64>,
    sink_min_delays_ps: BTreeMap<(NetId, CellPinId), u64>,
    sink_criticalities: BTreeMap<(NetId, CellPinId), u64>,
    detailed_timing_nets: BTreeSet<NetId>,
    detailed_delay_quantum_ps: u64,
    alternate_source_delay_per_tile_ps: Option<u64>,
    max_iterations: u32,
}

impl RoutingCosts {
    /// Creates costs indexed by stable PIP and logical net IDs.
    #[must_use]
    pub fn new(pip_delays_ps: Vec<u32>, net_criticalities: BTreeMap<NetId, u64>) -> Self {
        let pip_delays_ps = Arc::<[u32]>::from(pip_delays_ps);
        Self {
            pip_min_delays_ps: Arc::clone(&pip_delays_ps),
            pip_delays_ps,
            net_criticalities,
            sink_min_delays_ps: BTreeMap::new(),
            sink_criticalities: BTreeMap::new(),
            detailed_timing_nets: BTreeSet::new(),
            detailed_delay_quantum_ps: 1,
            alternate_source_delay_per_tile_ps: None,
            max_iterations: MAX_ROUTING_ITERATIONS,
        }
    }

    /// Estimated maximum delay for every physical PIP.
    #[must_use]
    pub fn pip_delays_ps(&self) -> &[u32] {
        self.pip_delays_ps.as_ref()
    }

    /// Replaces minimum-corner PIP delays used for hold repair.
    pub fn set_pip_min_delays_ps(&mut self, pip_min_delays_ps: Vec<u32>) {
        self.pip_min_delays_ps = pip_min_delays_ps.into();
    }

    /// Estimated minimum delay for every physical PIP.
    #[must_use]
    pub fn pip_min_delays_ps(&self) -> &[u32] {
        self.pip_min_delays_ps.as_ref()
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

    /// Enables a characterized long-line lower estimate for recovering a
    /// faster source of a timing-driven Steiner tree.
    ///
    /// The normal A-star estimate remains unchanged. When that search joins a
    /// sink through a late-arriving tree branch, this coefficient cheaply
    /// identifies whether another existing tree wire could plausibly arrive
    /// sooner through architecture long lines. Only that one source is then
    /// searched, and its exact route score must improve before it is used.
    pub fn set_alternate_source_delay_per_tile_ps(&mut self, delay_per_tile_ps: u64) {
        self.alternate_source_delay_per_tile_ps =
            (delay_per_tile_ps != 0).then_some(delay_per_tile_ps);
    }

    /// Characterized long-line coefficient for alternate-source recovery.
    #[must_use]
    pub const fn alternate_source_delay_per_tile_ps(&self) -> Option<u64> {
        self.alternate_source_delay_per_tile_ps
    }
}

impl RoutingConstraints {
    /// Creates an unconstrained routing problem.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            routes: BTreeMap::new(),
            blocked_pips: BTreeSet::new(),
            blocked_pip_words: None,
        }
    }

    /// Sets the immutable route tree for one logical net.
    pub fn add_route(&mut self, route: impl Into<Arc<NetRoute>>) {
        let route = route.into();
        self.routes.insert(route.net, route);
    }

    /// Immutable route trees indexed by logical net.
    #[must_use]
    pub const fn routes(&self) -> &BTreeMap<NetId, Arc<NetRoute>> {
        &self.routes
    }

    /// Prevents the router from using target-defined illegal PIPs.
    pub fn block_pips(&mut self, pips: impl IntoIterator<Item = PipId>) {
        let mut pips = pips.into_iter();
        let Some(first) = pips.next() else {
            return;
        };
        let words = Arc::make_mut(
            self.blocked_pip_words
                .get_or_insert_with(|| Arc::new(Vec::new())),
        );
        for pip in std::iter::once(first).chain(pips) {
            self.blocked_pips.insert(pip);
            let word = pip.0 / u64::BITS as usize;
            if words.len() <= word {
                words.resize(word + 1, 0);
            }
            words[word] |= 1_u64 << (pip.0 % u64::BITS as usize);
        }
    }

    /// PIPs unavailable in this placement-specific routing problem.
    #[must_use]
    pub const fn blocked_pips(&self) -> &BTreeSet<PipId> {
        &self.blocked_pips
    }

    fn blocked_pip_words(&self) -> &[u64] {
        self.blocked_pip_words.as_deref().map_or(&[], Vec::as_slice)
    }
}

fn pip_is_blocked(words: &[u64], pip: PipId) -> bool {
    let word = pip.0 / u64::BITS as usize;
    words
        .get(word)
        .is_some_and(|bits| bits & (1_u64 << (pip.0 % u64::BITS as usize)) != 0)
}

impl PlacementConstraints {
    /// Creates an unconstrained placement problem.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            groups: Vec::new(),
            group_row_indexes: Vec::new(),
            shared_group_row_indexes: BTreeMap::new(),
            pin_bindings: BTreeMap::new(),
            pin_name_bindings: BTreeMap::new(),
            shared_resources: Vec::new(),
        }
    }

    /// Adds one atomic group. Structural and compatibility checks run before
    /// placement because they require the complete design and device.
    pub fn add_group(
        &mut self,
        cells: impl IntoIterator<Item = CellId>,
        assignments: impl IntoIterator<Item = Vec<BelId>>,
    ) {
        let assignments = assignments.into_iter().collect::<Vec<_>>();
        self.group_row_indexes
            .push(Arc::new(index_assignment_rows(&assignments)));
        self.groups.push(PlacementGroup {
            cells: cells.into_iter().collect(),
            assignments: assignments.into(),
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
        let shared_key = assignments.as_ptr() as usize;
        let (indexed_assignments, row_index) = self
            .shared_group_row_indexes
            .entry(shared_key)
            .or_insert_with(|| {
                (
                    Arc::clone(&assignments),
                    Arc::new(index_assignment_rows(&assignments)),
                )
            });
        debug_assert!(Arc::ptr_eq(indexed_assignments, &assignments));
        let row_index = Arc::clone(row_index);
        self.group_row_indexes.push(row_index);
        self.groups.push(PlacementGroup {
            cells: cells.into_iter().collect(),
            assignments,
        });
    }

    /// Removes the atomic group whose ordered cell columns exactly match
    /// `cells`.
    ///
    /// Target-specific post-route packing ECOs use this to release one
    /// dedicated edge without rebuilding or weakening unrelated constraints.
    pub fn remove_group(&mut self, cells: &[CellId]) -> bool {
        let Some(index) = self.groups.iter().position(|group| group.cells == cells) else {
            return false;
        };
        self.groups.remove(index);
        self.group_row_indexes.remove(index);
        true
    }

    /// Replaces one atomic group transactionally.
    ///
    /// The old ordered cell list identifies the group. Every replacement row
    /// must have one BEL per replacement cell, replacement cells must be
    /// unique, and no replacement cell may belong to another group. This is
    /// used when a target extends a rigid macro with optional dedicated-path
    /// members after the macro's base shape has been constructed.
    pub fn replace_group(
        &mut self,
        old_cells: &[CellId],
        cells: impl IntoIterator<Item = CellId>,
        assignments: impl IntoIterator<Item = Vec<BelId>>,
    ) -> bool {
        let Some(index) = self
            .groups
            .iter()
            .position(|group| group.cells == old_cells)
        else {
            return false;
        };
        let cells = cells.into_iter().collect::<Vec<_>>();
        let assignments = assignments.into_iter().collect::<Vec<_>>();
        if cells.is_empty()
            || assignments.is_empty()
            || assignments.iter().any(|row| row.len() != cells.len())
        {
            return false;
        }
        let unique = cells.iter().copied().collect::<BTreeSet<_>>();
        if unique.len() != cells.len()
            || self.groups.iter().enumerate().any(|(other_index, group)| {
                other_index != index && group.cells.iter().any(|cell| unique.contains(cell))
            })
        {
            return false;
        }
        self.group_row_indexes[index] = Arc::new(index_assignment_rows(&assignments));
        self.groups[index] = PlacementGroup {
            cells,
            assignments: assignments.into(),
        };
        true
    }

    /// Removes one cell column from an atomic group while preserving every
    /// other member and assignment row.
    ///
    /// A two-cell group is removed completely rather than leaving a redundant
    /// singleton group. Returns false when the cell is not grouped.
    pub fn remove_group_cell(&mut self, cell: CellId) -> bool {
        let Some((group_index, cell_index)) =
            self.groups
                .iter()
                .enumerate()
                .find_map(|(group_index, group)| {
                    group
                        .cells
                        .iter()
                        .position(|&candidate| candidate == cell)
                        .map(|cell_index| (group_index, cell_index))
                })
        else {
            return false;
        };
        if self.groups[group_index].cells.len() <= 2 {
            self.groups.remove(group_index);
            self.group_row_indexes.remove(group_index);
            return true;
        }
        let group = &self.groups[group_index];
        let mut cells = group.cells.clone();
        cells.remove(cell_index);
        let assignments = group
            .assignments
            .iter()
            .map(|row| {
                let mut row = row.clone();
                row.remove(cell_index);
                row
            })
            .collect::<Vec<_>>();
        self.group_row_indexes[group_index] = Arc::new(index_assignment_rows(&assignments));
        self.groups[group_index] = PlacementGroup {
            cells,
            assignments: assignments.into(),
        };
        true
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

    /// Adds one target-defined shared-site configuration rule.
    ///
    /// Cells with no value and BELs with no resource ID do not participate.
    /// Values only need to be stable and comparable within this rule.
    pub fn add_shared_resource(
        &mut self,
        cell_values: impl IntoIterator<Item = (CellId, u64)>,
        bel_resources: impl IntoIterator<Item = (BelId, u64)>,
    ) {
        self.shared_resources.push(PlacementSharedResource {
            cell_values: cell_values.into_iter().collect(),
            bel_resources: bel_resources.into_iter().collect(),
        });
    }

    /// Target-defined shared-site configuration rules.
    #[must_use]
    pub fn shared_resources(&self) -> &[PlacementSharedResource] {
        &self.shared_resources
    }
}

fn index_assignment_rows(assignments: &[Vec<BelId>]) -> BTreeMap<BelId, Vec<usize>> {
    let mut index = BTreeMap::<BelId, Vec<usize>>::new();
    for (row_index, row) in assignments.iter().enumerate() {
        if let Some(&first_bel) = row.first() {
            index.entry(first_bel).or_default().push(row_index);
        }
    }
    index
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

/// Re-resolves target-selected physical pins without changing any BEL.
///
/// This is the cheap path for a packing ECO that only changes pin-name
/// bindings while preserving an already legal placement. Callers must ensure
/// the new constraints do not add a stricter placement group; releasing a
/// dedicated edge satisfies that requirement.
///
/// # Errors
///
/// Returns an error when the placement has the wrong number of cells or a new
/// physical pin name is unavailable on the existing BEL.
pub fn rebind_placement_pins(
    design: &Design,
    device: &Device,
    constraints: &PlacementConstraints,
    placement: &Placement,
) -> Result<Placement, PnrError> {
    if placement.bindings.len() != design.cells().len() {
        return Err(PnrError::InvalidPlacement {
            reason: format!(
                "expected {} cell bindings, received {}",
                design.cells().len(),
                placement.bindings.len()
            ),
        });
    }
    let graph = UnifiedGraph::new(design, device);
    finish_placement(
        &graph,
        constraints,
        placement.bindings.iter().copied().map(Some).collect(),
    )
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
    wire_owners: HashMap<WireId, Vec<(NetId, u64, usize)>>,
    pip_owners: HashMap<PipId, Vec<(NetId, u64, usize)>>,
    routes: BTreeMap<NetId, Arc<NetRoute>>,
}

impl RouteCapacityProjection {
    /// Projects routed arcs onto the resources they occupy.
    #[must_use]
    pub fn new(routes: &[Arc<NetRoute>], costs: &RoutingCosts) -> Self {
        let mut projection = Self::default();
        for route in routes {
            projection.routes.insert(route.net, route.clone());
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

fn projected_release_scope_penalty(
    owners: Option<&Vec<(NetId, u64, usize)>>,
    moving_net: NetId,
    capacity: u16,
) -> u64 {
    const ARC_RELEASE_PENALTY_PS: u64 = 25;
    const OVERSIZED_TRANSACTION_PENALTY_PS: u64 = 1_000_000;
    let victims = projected_victim_nets(owners, moving_net, capacity);
    let affected = victims
        .into_iter()
        .filter_map(|net| {
            owners?
                .iter()
                .find(|(owner, _, _)| *owner == net)
                .map(|(_, _, arcs)| *arcs)
        })
        .sum::<usize>();
    if affected > MAX_PROJECTED_BLOCKER_SINKS {
        OVERSIZED_TRANSACTION_PENALTY_PS
    } else {
        (affected as u64).saturating_mul(ARC_RELEASE_PENALTY_PS)
    }
}

const MAX_PROJECTED_BLOCKER_SINKS: usize = 8;

fn projected_victim_nets(
    owners: Option<&Vec<(NetId, u64, usize)>>,
    moving_net: NetId,
    capacity: u16,
) -> Vec<NetId> {
    let Some(owners) = owners else {
        return Vec::new();
    };
    let mut victims = owners
        .iter()
        .filter(|(net, _, _)| *net != moving_net)
        .map(|&(net, criticality, _)| (net, criticality))
        .collect::<Vec<_>>();
    let required = victims
        .len()
        .saturating_add(1)
        .saturating_sub(usize::from(capacity));
    victims.sort_unstable_by_key(|&(net, criticality)| (criticality, net));
    victims
        .into_iter()
        .take(required)
        .map(|(net, _)| net)
        .collect()
}

fn update_projected_owner(owners: &mut Vec<(NetId, u64, usize)>, net: NetId, criticality: u64) {
    if let Some((_, known, arcs)) = owners.iter_mut().find(|(owner, _, _)| *owner == net) {
        *known = (*known).max(criticality);
        *arcs += 1;
    } else {
        owners.push((net, criticality, 1));
    }
}

impl NetRoute {
    /// Builds a route and derives its shared-resource reference counts.
    #[must_use]
    pub fn new(net: NetId, mut arcs: Vec<RouteArc>) -> Self {
        arcs.sort_by(|left, right| {
            (left.sink, &left.wires, &left.pips).cmp(&(right.sink, &right.wires, &right.pips))
        });
        let (wire_refs, pip_refs) = route_resource_refs(&arcs);
        Self {
            net,
            arcs,
            wire_refs,
            pip_refs,
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
        self.arcs
            .binary_search_by_key(&Some(sink), |arc| arc.sink)
            .ok()
            .map(|index| &self.arcs[index])
    }
}

type RouteResourceRefs = (Vec<(WireId, u32)>, Vec<(PipId, u32)>);

fn route_resource_refs(arcs: &[RouteArc]) -> RouteResourceRefs {
    let mut wire_refs = BTreeMap::<WireId, u32>::new();
    let mut pip_refs = BTreeMap::<PipId, u32>::new();
    for arc in arcs {
        for &wire in &arc.wires {
            *wire_refs.entry(wire).or_default() += 1;
        }
        for &pip in &arc.pips {
            *pip_refs.entry(pip).or_default() += 1;
        }
    }
    (
        wire_refs.into_iter().collect(),
        pip_refs.into_iter().collect(),
    )
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
    pub routes: Vec<Arc<NetRoute>>,
    /// Number of unique PIPs used across all net trees.
    pub total_pips: usize,
}

/// One routed driver-to-sink connection considered by a legal timing ECO.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct LegalRouteEcoConnection {
    /// Logical net containing the connection.
    pub net: NetId,
    /// Logical sink pin whose route arc may change.
    pub sink: CellPinId,
}

impl LegalRouteEcoConnection {
    /// Creates one connection-local ECO request.
    #[must_use]
    pub const fn new(net: NetId, sink: CellPinId) -> Self {
        Self { net, sink }
    }
}

/// Search controls for one legal timing ECO candidate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LegalRouteEcoOptions {
    /// Geometry-to-delay coefficient used by the hard-occupancy A-star search.
    pub estimate_delay_per_tile_ps: u64,
}

impl LegalRouteEcoOptions {
    /// Creates connection-local ECO search controls.
    #[must_use]
    pub const fn new(estimate_delay_per_tile_ps: u64) -> Self {
        Self {
            estimate_delay_per_tile_ps,
        }
    }
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
    wire_congestion: Vec<u32>,
    pip_congestion: Vec<u32>,
    touched_wires: Vec<usize>,
    touched_pips: Vec<usize>,
    search: RouteSearch,
    tree_arrival_ps: Vec<u64>,
    wire_points: Vec<Point>,
    wire_capacities: Vec<u16>,
    pip_capacities: Vec<u16>,
    /// Flat connection-owner records reused when selecting congestion
    /// victims. Only overused resources enter these vectors.
    connection_owners: ConnectionOwnerScratch,
    /// Sparse resource-to-net ownership rebuilt from the resident connection
    /// trees and updated as dirty connections are routed.
    resource_owners: ResourceOwnerIndex,
    /// Routes whose resource usage is currently reflected by occupancy.
    /// A failed negotiation invalidates this snapshot; the next call then
    /// falls back to a full sparse reset.
    resident_routes: Vec<Option<Arc<NetRoute>>>,
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
            wire_congestion: vec![0; device.wires().len()],
            pip_congestion: vec![0; device.pips().len()],
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
            connection_owners: ConnectionOwnerScratch::default(),
            resource_owners: ResourceOwnerIndex::default(),
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
            self.wire_congestion[index] = 0;
        }
        for index in self.touched_pips.drain(..) {
            self.pip_occupancy[index] = 0;
            self.pip_history[index] = 0;
            self.pip_congestion[index] = 0;
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
    ) -> Vec<Option<Arc<NetRoute>>> {
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
                if old
                    .as_ref()
                    .zip(new)
                    .is_some_and(|(old, new)| Arc::ptr_eq(old, new) || old.as_ref() == new.as_ref())
                    || old.is_none() && new.is_none()
                {
                    continue;
                }
                if let Some(old) = old.as_deref() {
                    remove_route_occupancy(self, old, new.map(Arc::as_ref));
                }
                if let Some(new) = new {
                    add_route_occupancy_delta(self, new, old.as_deref());
                }
            }
        }
        // Until negotiation succeeds, occupancy no longer has a committed
        // route snapshot to synchronize from safely.
        self.resident_valid = false;
        target
    }

    fn commit_routes(&mut self, routes: &[Arc<NetRoute>]) {
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

/// Occupancy treated as immutable while improving one already-legal arc.
///
/// The negotiated router may temporarily overuse resources to escape a
/// congested solution.  Legal-route polishing is different: every resident
/// connection except the released arc is a hard obstacle, so each accepted
/// replacement remains legal without another Pathfinder round.
#[derive(Clone, Copy)]
struct HardRoutingOccupancy<'a> {
    wires: &'a [u16],
    pips: &'a [u16],
    /// Keep the configured timing heuristic for a bounded legal ECO search.
    use_estimate: bool,
}

/// Saturated resources pruned by one hard-occupancy route search.
///
/// Legal-route polishing subscribes the attempted connection to these exact
/// resources.  The connection only needs another search after one of them is
/// released, rather than after every unrelated route improvement.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct HardRoutingBlockers {
    wires: BTreeSet<WireId>,
    pips: BTreeSet<PipId>,
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
    let pin_wires = PinWireCache::build(graph, &placement);
    validate_routing_constraints(graph, &placement, &pin_wires, routing_constraints)?;
    validate_routing_costs(graph, routing_costs)?;
    let routes = workspace.prepare_routes(
        graph.device(),
        graph.design().nets().len(),
        routing_constraints,
    );
    let alternate_attempts_before = workspace.search.alternate_source_attempts;
    let alternate_improvements_before = workspace.search.alternate_source_improvements;
    let routes = route(
        graph,
        &placement,
        &pin_wires,
        routing_constraints,
        routing_costs,
        workspace,
        routes,
        progress,
    );
    if std::env::var_os("TEXO_PNR_METRICS").is_some() {
        eprintln!(
            "[metrics] alternate_source_recovery attempts={} improvements={}",
            workspace
                .search
                .alternate_source_attempts
                .saturating_sub(alternate_attempts_before),
            workspace
                .search
                .alternate_source_improvements
                .saturating_sub(alternate_improvements_before),
        );
    }
    let routes = routes?;
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
    PlacementRefiner::new(design, device, constraints)?.place_analytically(sink_weights)
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
    let mut resource_usage = PlacementResourceUsage::default();

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
                    && assignment_resources_are_legal(
                        &graph,
                        constraints,
                        &unit.cells,
                        assignment,
                        &resource_usage,
                    )
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
        update_placement_resource_usage(
            &graph,
            constraints,
            &unit.cells,
            assignment,
            &mut resource_usage,
            true,
        );
    }

    finish_placement(&graph, constraints, placed)
}

/// Validates and materializes a placement from one BEL binding per cell.
///
/// Unlike [`placement_from_partial_bindings`], this path does not regenerate
/// placement units or search their candidate tables. It is intended for a
/// bounded target ECO that changes a few BELs in an already complete
/// placement. Every supplied binding is still checked for cell/BEL
/// compatibility, unique BEL ownership, physical-pin compatibility, complete
/// atomic-group membership, and shared-resource legality.
///
/// # Errors
///
/// Returns an error when the table is incomplete, names an unknown or
/// incompatible BEL, reuses a BEL, or violates a placement constraint.
pub fn placement_from_complete_bindings(
    design: &Design,
    device: &Device,
    constraints: &PlacementConstraints,
    bindings: Vec<BelId>,
) -> Result<Placement, PnrError> {
    let graph = UnifiedGraph::new(design, device);
    if bindings.len() != design.cells().len() {
        return Err(PnrError::InvalidPlacement {
            reason: format!(
                "expected {} cell bindings, received {}",
                design.cells().len(),
                bindings.len()
            ),
        });
    }
    validate_pin_bindings(&graph, constraints)?;
    validate_shared_resources(&graph, constraints)?;

    let mut occupied = vec![false; device.bels().len()];
    for (index, &bel) in bindings.iter().enumerate() {
        let Some(physical) = device.bels().get(bel.0) else {
            return Err(PnrError::InvalidPlacement {
                reason: format!("binding names unknown BEL {}", bel.0),
            });
        };
        let logical = &design.cells()[index];
        if logical.kind != physical.kind {
            return Err(PnrError::InvalidPlacement {
                reason: format!("BEL {} is incompatible with cell ID {}", bel.0, index),
            });
        }
        if occupied[bel.0] {
            return Err(PnrError::InvalidPlacement {
                reason: format!("BEL {} is assigned more than once", bel.0),
            });
        }
        occupied[bel.0] = true;
        for &pin in logical.pins() {
            if candidate_bel_pin(&graph, constraints, pin, bel).is_none() {
                return Err(PnrError::InvalidPlacement {
                    reason: format!(
                        "BEL {} has no compatible physical pin for cell {} pin {}",
                        bel.0,
                        logical.name,
                        design.pins()[pin.0].name
                    ),
                });
            }
        }
    }

    finish_placement(
        &graph,
        constraints,
        bindings.into_iter().map(Some).collect(),
    )
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
    let mut placed = validate_refinement_start(&graph, constraints, &units, placement, None)?;
    let mut occupied = dense_placement_occupancy(device, &placed);
    let _ = refine_placement(
        &graph,
        constraints,
        &units,
        &neighbors,
        &mut placed,
        &mut occupied,
        None,
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
    let mut placed = validate_refinement_start(&graph, constraints, &units, placement, None)?;
    let mut occupied = dense_placement_occupancy(device, &placed);
    let _ = refine_placement(
        &graph,
        constraints,
        &units,
        &neighbors,
        &mut placed,
        &mut occupied,
        None,
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
    spatial_indexes: BTreeMap<(u8, usize), Arc<SpatialChoiceIndex>>,
}

/// Architecture control-set identity used by analytical register clustering.
///
/// Registers with equal `clock_lsr` may share one logic tile, while `ce`
/// identifies the smaller register pair that shares a clock-enable input.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct RegisterControlSet {
    /// Logical register cell.
    pub cell: CellId,
    /// Clock, edge, reset, reset polarity, and reset mode identity.
    pub clock_lsr: (u64, u64),
    /// Clock-enable net, constant state, and polarity identity.
    pub ce: u64,
}

/// Reusable architecture-level tables for rebuilding placement refiners after
/// a packing change.
#[derive(Default)]
pub struct PlacementRefinementWorkspace {
    candidate_cache: BTreeMap<PlacementCandidateKey, Arc<[BelId]>>,
    spatial_indexes: BTreeMap<(u8, usize), Arc<SpatialChoiceIndex>>,
    validated_group_shapes: Vec<ValidatedGroupShape>,
}

/// Scratch storage for exact local connection-delay scoring.
///
/// A timing-refinement pass can reuse both completed endpoint queries and the
/// allocation behind each bounded route search. Create a fresh workspace when
/// the device or PIP delay table changes.
#[derive(Default)]
pub struct PlacementConnectionDelayWorkspace {
    delays: PackedRouteMap<Option<u64>>,
    queue: BinaryHeap<Reverse<(u64, u8, WireId)>>,
    best: PackedRouteMap<[u64; LOCAL_HOP_STATES]>,
}

#[derive(Default)]
struct PackedRouteHasher(u64);

impl Hasher for PackedRouteHasher {
    fn finish(&self) -> u64 {
        self.0
    }

    fn write(&mut self, bytes: &[u8]) {
        // Packed route maps use u64 keys, whose Hash implementation calls
        // `write_u64`; retain a deterministic fallback for trait completeness.
        self.0 = bytes.iter().fold(0xcbf2_9ce4_8422_2325, |hash, byte| {
            (hash ^ u64::from(*byte)).wrapping_mul(0x100_0000_01b3)
        });
    }

    fn write_u64(&mut self, value: u64) {
        let mut mixed = value.wrapping_add(0x9e37_79b9_7f4a_7c15);
        mixed = (mixed ^ (mixed >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        mixed = (mixed ^ (mixed >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        self.0 = mixed ^ (mixed >> 31);
    }
}

type PackedRouteMap<Value> = HashMap<u64, Value, BuildHasherDefault<PackedRouteHasher>>;

fn packed_wire_pair(start: WireId, goal: WireId) -> u64 {
    ((start.0 as u64) << 32) | goal.0 as u64
}

impl PlacementConnectionDelayWorkspace {
    /// Creates empty pass-local delay-search storage.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }
}

struct ValidatedGroupShape {
    assignments: Arc<[Vec<BelId>]>,
    candidate_sets: Vec<Arc<[BelId]>>,
}

impl PlacementRefinementWorkspace {
    /// Creates an empty placement-graph cache.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            candidate_cache: BTreeMap::new(),
            spatial_indexes: BTreeMap::new(),
            validated_group_shapes: Vec::new(),
        }
    }
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
        let mut workspace = PlacementRefinementWorkspace::new();
        Self::new_with_workspace(design, device, constraints, &mut workspace)
    }

    /// Builds a refiner while reusing architecture-level candidate tables and
    /// spatial indexes retained across packing generations.
    ///
    /// # Errors
    ///
    /// Returns an error if a placement group or candidate binding is invalid.
    pub fn new_with_workspace(
        design: &'a Design,
        device: &'a Device,
        constraints: &'a PlacementConstraints,
        workspace: &mut PlacementRefinementWorkspace,
    ) -> Result<Self, PnrError> {
        let graph = UnifiedGraph::new(design, device);
        let units = placement_units_cached(
            &graph,
            constraints,
            &mut workspace.candidate_cache,
            &mut workspace.validated_group_shapes,
        )?;
        let mut spatial_indexes = BTreeMap::new();
        for unit in &units {
            let index = workspace
                .spatial_indexes
                .entry(unit.choices.cache_key())
                .or_insert_with(|| Arc::new(SpatialChoiceIndex::new(&unit.choices, device)));
            spatial_indexes.insert(unit.choices.cache_key(), Arc::clone(index));
        }
        Ok(Self {
            graph,
            constraints,
            units,
            spatial_indexes,
        })
    }

    /// Solves analytical placement on this refiner's cached legal-assignment
    /// graph and spatial hierarchy.
    ///
    /// # Errors
    ///
    /// Returns an error when the cached placement problem cannot be legalized.
    pub fn place_analytically(
        &self,
        sink_weights: &BTreeMap<(NetId, CellPinId), u64>,
    ) -> Result<Placement, PnrError> {
        analytical_place(
            &self.graph,
            self.constraints,
            &self.units,
            &self.spatial_indexes,
            sink_weights,
            None,
            AnalyticalGlobalPlacement::Electrostatic,
            None,
            &[],
        )
    }

    /// Solves timing-weighted electrostatic placement with architecture
    /// routing capacity available to elfPlace instance-area adjustment.
    ///
    /// The capacity map is immutable across the continuous solve.  It should
    /// be built from the legal coarse seed's placement-specific routing
    /// restrictions so blocked architecture resources are not counted.
    ///
    /// # Errors
    ///
    /// Returns an error when the cached placement problem cannot be legalized
    /// or continuous global placement fails.
    pub fn place_analytically_with_routing_capacity(
        &self,
        sink_weights: &BTreeMap<(NetId, CellPinId), u64>,
        routing_capacity: &RoutingCapacityMap,
    ) -> Result<Placement, PnrError> {
        analytical_place(
            &self.graph,
            self.constraints,
            &self.units,
            &self.spatial_indexes,
            sink_weights,
            None,
            AnalyticalGlobalPlacement::Electrostatic,
            Some(routing_capacity),
            &[],
        )
    }

    /// Solves timing-weighted electrostatic placement with routing capacity
    /// and architecture register-control identities available to area tuning.
    ///
    /// # Errors
    ///
    /// Returns an error when a movable register lacks a control identity or
    /// the cached placement problem cannot be legalized.
    pub fn place_analytically_with_routing_capacity_and_register_controls(
        &self,
        sink_weights: &BTreeMap<(NetId, CellPinId), u64>,
        routing_capacity: &RoutingCapacityMap,
        register_controls: &[RegisterControlSet],
    ) -> Result<Placement, PnrError> {
        analytical_place(
            &self.graph,
            self.constraints,
            &self.units,
            &self.spatial_indexes,
            sink_weights,
            None,
            AnalyticalGlobalPlacement::Electrostatic,
            Some(routing_capacity),
            register_controls,
        )
    }

    /// Solves only the coarse quadratic connectivity system and legalizes it.
    ///
    /// This intentionally skips electrostatic density optimization.  It is a
    /// cheap legal seed for placement-delay estimation before the one full
    /// timing-weighted global-placement pass; it is not a substitute for that
    /// pass in a completed implementation flow.
    ///
    /// # Errors
    ///
    /// Returns an error when the cached placement problem cannot be legalized.
    pub fn place_analytically_coarse(
        &self,
        sink_weights: &BTreeMap<(NetId, CellPinId), u64>,
    ) -> Result<Placement, PnrError> {
        analytical_place(
            &self.graph,
            self.constraints,
            &self.units,
            &self.spatial_indexes,
            sink_weights,
            None,
            AnalyticalGlobalPlacement::Coarse,
            None,
            &[],
        )
    }

    /// Re-solves analytical placement while softly anchoring every movable
    /// unit to a previously legalized placement.
    ///
    /// The anchor prevents timing feedback from replacing a good placement
    /// basin wholesale. Successive solve/legalize iterations can therefore
    /// adjust critical connectivity continuously. `iteration` starts at one
    /// and strengthens the anchor as the iteration progresses.
    ///
    /// # Errors
    ///
    /// Returns an error when the anchor is incompatible with this placement
    /// problem or when the resulting analytical placement cannot be legalized.
    pub fn place_analytically_anchored(
        &self,
        sink_weights: &BTreeMap<(NetId, CellPinId), u64>,
        anchor: &Placement,
        iteration: u32,
    ) -> Result<Placement, PnrError> {
        let _ = validate_refinement_start(
            &self.graph,
            self.constraints,
            &self.units,
            anchor.clone(),
            Some(&self.spatial_indexes),
        )?;
        analytical_place(
            &self.graph,
            self.constraints,
            &self.units,
            &self.spatial_indexes,
            sink_weights,
            Some((anchor, iteration.max(1))),
            AnalyticalGlobalPlacement::Coarse,
            None,
            &[],
        )
    }

    /// Refines a legal placement against normalized net-bounding-box and
    /// architecture-predicted timing costs.
    ///
    /// Each accepted move strictly lowers the same combined objective used
    /// throughout a pass. Wirelength and timing deltas are normalized by their
    /// respective whole-design totals, so neither picoseconds nor tile units
    /// need an architecture-specific tuning coefficient. Candidate placement
    /// units remain atomic and every target-defined shared resource is checked
    /// before the move is scored.
    ///
    /// `sink_criticalities` uses one as the noncritical baseline. Values above
    /// one strengthen only that exact driver-to-sink arc.
    ///
    /// # Errors
    ///
    /// Returns an error if the input placement is incompatible with this
    /// cached placement problem.
    pub fn refine_with_predicted_timing(
        &self,
        placement: Placement,
        sink_criticalities: &BTreeMap<(NetId, CellPinId), u64>,
        delay_estimator: &impl PlacementDelayEstimator,
    ) -> Result<Placement, PnrError> {
        self.refine_with_predicted_timing_impl(
            placement,
            sink_criticalities,
            delay_estimator,
            PredictedPlacementPasses::UntilFixedPoint,
        )
        .map(|(placement, _)| placement)
    }

    /// Runs exactly one deterministic predicted-timing refinement sweep.
    ///
    /// The returned count is the number of placement units moved by the
    /// sweep. Callers that refresh timing between sweeps can stop without
    /// another STA evaluation when this count is zero. Unlike
    /// [`Self::refine_with_predicted_timing`], this method never reuses one
    /// set of criticalities for multiple sweeps.
    ///
    /// # Errors
    ///
    /// Returns an error if the input placement is incompatible with this
    /// cached placement problem.
    pub fn refine_with_predicted_timing_pass(
        &self,
        placement: Placement,
        sink_criticalities: &BTreeMap<(NetId, CellPinId), u64>,
        delay_estimator: &impl PlacementDelayEstimator,
    ) -> Result<(Placement, usize), PnrError> {
        self.refine_with_predicted_timing_impl(
            placement,
            sink_criticalities,
            delay_estimator,
            PredictedPlacementPasses::One,
        )
    }

    fn refine_with_predicted_timing_impl(
        &self,
        placement: Placement,
        sink_criticalities: &BTreeMap<(NetId, CellPinId), u64>,
        delay_estimator: &impl PlacementDelayEstimator,
        passes: PredictedPlacementPasses,
    ) -> Result<(Placement, usize), PnrError> {
        let (_, neighbors) =
            placement_neighbors(self.graph.design(), None, Some(sink_criticalities), None);
        let mut placed = validate_refinement_start(
            &self.graph,
            self.constraints,
            &self.units,
            placement,
            Some(&self.spatial_indexes),
        )?;
        let mut occupied = dense_placement_occupancy(self.graph.device(), &placed);
        let moved = refine_predicted_placement(
            &self.graph,
            self.constraints,
            &self.units,
            &self.spatial_indexes,
            &neighbors,
            sink_criticalities,
            delay_estimator,
            &mut placed,
            &mut occupied,
            passes,
        );
        finish_placement(&self.graph, self.constraints, placed).map(|placement| (placement, moved))
    }

    /// Predicts one logical net arc under an existing legal placement.
    ///
    /// This resolves every target-selected candidate pin before delegating to
    /// the architecture estimator, so placement-time STA and detailed move
    /// scoring use exactly the same physical endpoint model.
    #[must_use]
    pub fn predicted_arc_delay_ps(
        &self,
        placement: &Placement,
        net: NetId,
        sink: CellPinId,
        delay_estimator: &impl PlacementDelayEstimator,
    ) -> Option<u64> {
        let design = self.graph.design();
        let driver = design.nets().get(net.0)?.driver;
        let driver_cell = design.pins().get(driver.0)?.cell;
        let sink_cell = design.pins().get(sink.0)?.cell;
        let driver_bel = placement.bindings.get(driver_cell.0).copied()?;
        let sink_bel = placement.bindings.get(sink_cell.0).copied()?;
        let driver_pin = candidate_bel_pin(&self.graph, self.constraints, driver, driver_bel)?;
        let sink_pin = candidate_bel_pin(&self.graph, self.constraints, sink, sink_bel)?;
        Some(delay_estimator.estimate_delay_ps(driver_bel, driver_pin, sink_bel, sink_pin))
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
        self.refine_with_net_sink_weights_limited_and_move_peak(
            placement,
            sink_weights,
            sink_budgets,
            max_moved_units,
        )
        .map(|(placement, _)| placement)
    }

    /// Refines a placement and also reports the largest number of units moved
    /// in any pass.
    ///
    /// When this peak is below the requested limit, the limit never affected
    /// the result. Timing-closure portfolios can use that fact to skip an
    /// exactly equivalent trial at another still-higher limit.
    ///
    /// # Errors
    ///
    /// Returns the same errors as
    /// [`Self::refine_with_net_sink_weights_limited`].
    pub fn refine_with_net_sink_weights_limited_and_move_peak(
        &self,
        placement: Placement,
        sink_weights: &BTreeMap<(NetId, CellPinId), u64>,
        sink_budgets: Option<&BTreeMap<(NetId, CellPinId), u32>>,
        max_moved_units: usize,
    ) -> Result<(Placement, usize), PnrError> {
        let (_, neighbors) =
            placement_neighbors(self.graph.design(), None, Some(sink_weights), sink_budgets);
        let mut placed = validate_refinement_start(
            &self.graph,
            self.constraints,
            &self.units,
            placement,
            Some(&self.spatial_indexes),
        )?;
        let mut occupied = dense_placement_occupancy(self.graph.device(), &placed);
        let move_peak = refine_placement(
            &self.graph,
            self.constraints,
            &self.units,
            &neighbors,
            &mut placed,
            &mut occupied,
            Some(max_moved_units),
            Some(&self.spatial_indexes),
        );
        Ok((
            finish_placement(&self.graph, self.constraints, placed)?,
            move_peak,
        ))
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
        let mut placed = validate_refinement_start(
            &self.graph,
            self.constraints,
            &self.units,
            placement,
            Some(&self.spatial_indexes),
        )?;
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
        let mut local_queue = BinaryHeap::new();
        let mut local_best = PackedRouteMap::default();
        let Some(current_delay) = local_connection_delay(
            &self.graph,
            current_start,
            current_goal,
            pip_delays_ps,
            &mut local_queue,
            &mut local_best,
        ) else {
            return Ok(None);
        };

        let mut occupied = placed.iter().copied().flatten().collect::<BTreeSet<_>>();
        let mut pin_usage = PlacementResourceUsage::default();
        for known in &self.units {
            let assignment = known
                .cells
                .iter()
                .map(|cell| placed[cell.0].expect("validated placement is complete"))
                .collect::<Vec<_>>();
            update_placement_resource_usage(
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
        update_placement_resource_usage(
            &self.graph,
            self.constraints,
            &unit.cells,
            &current,
            &mut pin_usage,
            false,
        );
        let current_point = device.bels()[current[moving_column].0].point;
        let mut best: Option<(u64, Vec<BelId>)> = None;
        let index_point = device.bels()[current[0].0].point;
        let spatial_index = &self.spatial_indexes[&unit.choices.cache_key()];
        for choice in spatial_choices_within(spatial_index, index_point, max_move_distance, device)
        {
            let assignment = unit.choices.assignment(choice);
            if assignment == current
                || assignment.iter().any(|bel| occupied.contains(bel))
                || device.bels()[assignment[moving_column].0]
                    .point
                    .manhattan(current_point)
                    > max_move_distance
                || !assignment_resources_are_legal(
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
            let Some(delay) = local_connection_delay(
                &self.graph,
                start,
                goal,
                pip_delays_ps,
                &mut local_queue,
                &mut local_best,
            ) else {
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
        let mut workspace = PlacementConnectionDelayWorkspace::new();
        self.refine_cell_connection_delays_with_cache(
            placement,
            moving_cell,
            connections,
            targets_ps,
            pip_delays_ps,
            capacity_projection,
            max_move_distance,
            max_candidates,
            &mut workspace,
        )
    }

    /// Proposes placements while reusing exact local-route delays computed by
    /// other cells in the same refinement pass.
    ///
    /// `workspace` must be fresh when the device or PIP delay table changes.
    /// Its cached entries depend only on the two endpoint wires and that table,
    /// so placements within one pass may safely share it.
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
    pub fn refine_cell_connection_delays_with_cache(
        &self,
        placement: Placement,
        moving_cell: CellId,
        connections: &[(CellPinId, CellPinId)],
        targets_ps: &[u64],
        pip_delays_ps: &[u32],
        capacity_projection: Option<&RouteCapacityProjection>,
        max_move_distance: u64,
        max_candidates: usize,
        workspace: &mut PlacementConnectionDelayWorkspace,
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
        let placed = validate_refinement_start(
            &self.graph,
            self.constraints,
            &self.units,
            placement,
            Some(&self.spatial_indexes),
        )?;
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
                workspace,
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
        let mut pin_usage = PlacementResourceUsage::default();
        for known in &self.units {
            let assignment = known
                .cells
                .iter()
                .map(|cell| placed[cell.0].expect("validated placement is complete"))
                .collect::<Vec<_>>();
            update_placement_resource_usage(
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
        update_placement_resource_usage(
            &self.graph,
            self.constraints,
            &unit.cells,
            &current,
            &mut pin_usage,
            false,
        );
        let current_point = device.bels()[current[moving_column].0].point;
        let mut best = Vec::<(u64, u64, Vec<BelId>)>::new();
        let index_point = device.bels()[current[0].0].point;
        let spatial_index = &self.spatial_indexes[&unit.choices.cache_key()];
        for choice in spatial_choices_within(spatial_index, index_point, max_move_distance, device)
        {
            let assignment = unit.choices.assignment(choice);
            if assignment == current
                || assignment.iter().any(|bel| occupied.contains(bel))
                || device.bels()[assignment[moving_column].0]
                    .point
                    .manhattan(current_point)
                    > max_move_distance
                || !assignment_resources_are_legal(
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
                    workspace,
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
            // Moving one member moves the complete rigid placement unit.  A
            // carry/FF macro therefore has to project every connection that
            // crosses the macro boundary, not only the critical connection
            // which selected this move.  Internal dedicated arcs retain their
            // relative placement and are deliberately absent.
            let projected_connections = if unit.cells.len() == 1 {
                connections.to_vec()
            } else {
                external_unit_connections(self.graph.design(), unit)
            };
            let retained_starts = projected_retained_tree_starts(
                self.graph.design(),
                unit,
                &projected_connections,
                projection,
            );
            let mut projected = best
                .into_iter()
                .filter_map(|(span, _, assignment)| {
                    assignment_connection_projected_cost(
                        &self.graph,
                        self.constraints,
                        unit,
                        &assignment,
                        &projected_connections,
                        &placed,
                        pip_delays_ps,
                        projection,
                        &retained_starts,
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

fn external_unit_connections(design: &Design, unit: &PlacementUnit) -> Vec<(CellPinId, CellPinId)> {
    let mut connections = BTreeSet::new();
    let incident_nets = unit
        .cells
        .iter()
        .flat_map(|cell| design.cells()[cell.0].pins())
        .filter_map(|pin| design.pins()[pin.0].net())
        .collect::<BTreeSet<_>>();
    for net in incident_nets.into_iter().map(|net| &design.nets()[net.0]) {
        let driver_pin = net.driver;
        let driver_cell = design.pins()[driver_pin.0].cell;
        let driver_inside = unit.cells.contains(&driver_cell);
        for &sink_pin in &net.sinks {
            let sink_cell = design.pins()[sink_pin.0].cell;
            let sink_inside = unit.cells.contains(&sink_cell);
            if driver_inside != sink_inside {
                connections.insert((driver_pin, sink_pin));
            }
        }
    }
    connections.into_iter().collect()
}

fn projected_retained_tree_starts(
    design: &Design,
    unit: &PlacementUnit,
    connections: &[(CellPinId, CellPinId)],
    projection: &RouteCapacityProjection,
) -> BTreeMap<(NetId, CellPinId), Arc<[WireId]>> {
    let mut retained = BTreeMap::new();
    for &(driver_pin, sink_pin) in connections {
        let Some(driver) = design.pins().get(driver_pin.0) else {
            continue;
        };
        let Some(sink) = design.pins().get(sink_pin.0) else {
            continue;
        };
        if !unit.cells.contains(&sink.cell) {
            continue;
        }
        let Some(net) = driver.net() else {
            continue;
        };
        let Some(route) = projection.routes.get(&net) else {
            continue;
        };
        let wires = route
            .arcs
            .iter()
            .filter(|arc| {
                arc.sink.is_none_or(|route_sink| {
                    !unit.cells.contains(&design.pins()[route_sink.0].cell)
                })
            })
            .flat_map(|arc| arc.wires.iter().copied())
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect::<Vec<_>>();
        if !wires.is_empty() {
            retained.insert((net, sink_pin), wires.into());
        }
    }
    retained
}

#[allow(clippy::too_many_arguments)]
fn assignment_connection_projected_cost(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    unit: &PlacementUnit,
    assignment: &[BelId],
    connections: &[(CellPinId, CellPinId)],
    placed: &[Option<BelId>],
    pip_delays_ps: &[u32],
    projection: &RouteCapacityProjection,
    retained_starts: &BTreeMap<(NetId, CellPinId), Arc<[WireId]>>,
) -> Option<u64> {
    let design = graph.design();
    let mut total = 0_u64;
    for &(driver_pin, sink_pin) in connections {
        let driver = design.pins().get(driver_pin.0)?;
        let sink = design.pins().get(sink_pin.0)?;
        if unit.cells.contains(&driver.cell) == unit.cells.contains(&sink.cell) {
            return None;
        }
        let net = driver.net()?;
        let driver_bel = assignment_bel(unit, assignment, driver.cell, placed)?;
        let sink_bel = assignment_bel(unit, assignment, sink.cell, placed)?;
        let driver_wire = candidate_pin_wire(graph, constraints, driver_pin, driver_bel)?;
        let sink_wire = candidate_pin_wire(graph, constraints, sink_pin, sink_bel)?;
        let single_start = [driver_wire];
        let starts = retained_starts
            .get(&(net, sink_pin))
            .map_or(&single_start[..], Arc::as_ref);
        total = total.saturating_add(local_connection_projected_cost_from_starts(
            graph,
            starts,
            sink_wire,
            pip_delays_ps,
            net,
            projection,
        )?);
    }
    Some(total)
}

fn local_connection_projected_cost_from_starts(
    graph: &UnifiedGraph<'_>,
    starts: &[WireId],
    goal: WireId,
    pip_delays_ps: &[u32],
    net: NetId,
    projection: &RouteCapacityProjection,
) -> Option<u64> {
    const MAX_LOCAL_HOPS: u8 = 16;
    const LOCAL_MARGIN: u32 = 1;
    let device = graph.device();
    let start_point = starts
        .iter()
        .map(|wire| device.wires()[wire.0].point)
        .min_by_key(|point| (point.manhattan(device.wires()[goal.0].point), *point))?;
    let corridor = routing_corridor(
        start_point,
        device.wires()[goal.0].point,
        device,
        LOCAL_MARGIN,
    );
    let mut queue = BinaryHeap::new();
    let mut best = HashMap::new();
    for &start in starts {
        if point_inside_corridor(device.wires()[start.0].point, corridor) {
            queue.push(Reverse((0_u64, 0_u8, start)));
            best.insert((start, 0_u8), 0_u64);
        }
    }
    while let Some(Reverse((cost, hops, wire))) = queue.pop() {
        if wire == goal {
            return Some(cost);
        }
        if hops == MAX_LOCAL_HOPS || best.get(&(wire, hops)).is_some_and(|known| *known < cost) {
            continue;
        }
        for (neighbor, pip) in graph.routing_neighbors(wire).ok()? {
            if starts.binary_search(&neighbor).is_ok() {
                continue;
            }
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
            ))
            .saturating_add(projected_release_scope_penalty(
                projection.wire_owners.get(&neighbor),
                net,
                device.wires()[neighbor.0].capacity,
            ))
            .saturating_add(projected_release_scope_penalty(
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
    owners: Option<&Vec<(NetId, u64, usize)>>,
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
        .filter(|(net, _, _)| *net != moving_net)
        .map(|&(_, criticality, _)| criticality)
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
    workspace: &mut PlacementConnectionDelayWorkspace,
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
        let endpoint_key = packed_wire_pair(driver_wire, sink_wire);
        let delay = if let Some(delay) = workspace.delays.get(&endpoint_key) {
            *delay
        } else {
            let delay = local_connection_delay(
                graph,
                driver_wire,
                sink_wire,
                pip_delays_ps,
                &mut workspace.queue,
                &mut workspace.best,
            );
            workspace.delays.insert(endpoint_key, delay);
            delay
        }?;
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
    queue: &mut BinaryHeap<Reverse<(u64, u8, WireId)>>,
    best: &mut PackedRouteMap<[u64; LOCAL_HOP_STATES]>,
) -> Option<u64> {
    // Long-line entry/exit PIPs can put even a modest tile displacement over
    // eight graph edges.  A too-small bound made a badly displaced critical
    // vertex impossible to score, so the detailed placer could never move it
    // back toward its path.  The one-tile corridor keeps this search local.
    const LOCAL_MARGIN: u32 = 1;
    let device = graph.device();
    let corridor = routing_corridor(
        device.wires()[start.0].point,
        device.wires()[goal.0].point,
        device,
        LOCAL_MARGIN,
    );
    queue.clear();
    best.clear();
    queue.push(Reverse((0_u64, 0_u8, start)));
    best.insert(start.0 as u64, [0; LOCAL_HOP_STATES]);
    while let Some(Reverse((delay, hops, wire))) = queue.pop() {
        if wire == goal {
            return Some(delay);
        }
        if hops == MAX_LOCAL_HOPS
            || best
                .get(&(wire.0 as u64))
                .is_some_and(|frontier| frontier[usize::from(hops)] < delay)
        {
            continue;
        }
        for (neighbor, pip) in graph.routing_neighbors(wire).ok()? {
            if !point_inside_corridor(device.wires()[neighbor.0].point, corridor) {
                continue;
            }
            let next_hops = hops + 1;
            let next_delay = delay.saturating_add(u64::from(pip_delays_ps[pip.0]));
            let frontier = best
                .entry(neighbor.0 as u64)
                .or_insert([u64::MAX; LOCAL_HOP_STATES]);
            if frontier[usize::from(next_hops)] <= next_delay {
                continue;
            }
            // A route reaching the same wire in fewer hops and no more delay
            // dominates this state for every remaining hop budget.  Store the
            // cumulative Pareto frontier so those states never enter the heap.
            for known in &mut frontier[usize::from(next_hops)..] {
                *known = (*known).min(next_delay);
            }
            queue.push(Reverse((next_delay, next_hops, neighbor)));
        }
    }
    None
}

const MAX_LOCAL_HOPS: u8 = 16;
const LOCAL_HOP_STATES: usize = MAX_LOCAL_HOPS as usize + 1;

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
                assignment.len() == 1 && candidates.binary_search(&assignment[0]).is_ok()
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

fn spatial_choices_within(
    spatial_index: &SpatialChoiceIndex,
    center: Point,
    max_distance: u64,
    device: &Device,
) -> Vec<usize> {
    let max_radius =
        u32::try_from(max_distance.min(u64::from(device.width().saturating_add(device.height()))))
            .expect("bounded device radius fits u32");
    let mut choices = Vec::new();
    for radius in 0..=max_radius {
        for dy in 0..=radius {
            let dx = radius - dy;
            for y in ring_coordinates(center.y, dy, device.height()) {
                for x in ring_coordinates(center.x, dx, device.width()) {
                    choices.extend_from_slice(
                        &spatial_index.by_point[(y * device.width() + x) as usize],
                    );
                }
            }
        }
    }
    choices
}

/// All usable assignments on the nearest Manhattan ring around `target`.
///
/// Unlike local cleanup, this search spans the finite device and therefore
/// lets a detailed-placement unit cross an arbitrarily wide legalization
/// basin. A nonempty ring containing only fixed, multiply occupied, or
/// shared-resource-incompatible assignments is skipped. Returning the whole
/// first usable ring, rather than a fixed number of assignments, keeps
/// equivalent same-tile BELs available for empty moves and compatible swaps.
fn visit_spatial_choices_on_nearest_usable_ring<T>(
    spatial_index: &SpatialChoiceIndex,
    target: Point,
    include_target_point: bool,
    device: &Device,
    mut classify: impl FnMut(usize) -> Option<T>,
    mut visit: impl FnMut(usize, T),
) -> usize {
    let first_radius = u32::from(!include_target_point);
    for radius in first_radius..device.width().saturating_add(device.height()) {
        let mut usable_count = 0_usize;
        for dy in 0..=radius {
            let dx = radius - dy;
            for y in ring_coordinates(target.y, dy, device.height()) {
                for x in ring_coordinates(target.x, dx, device.width()) {
                    for &choice in &spatial_index.by_point[(y * device.width() + x) as usize] {
                        if let Some(classification) = classify(choice) {
                            visit(choice, classification);
                            usable_count += 1;
                        }
                    }
                }
            }
        }
        if usable_count != 0 {
            return usable_count;
        }
    }
    0
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

#[derive(Default)]
struct RefinementChoiceWorkspace {
    nearest: Vec<usize>,
    pin_resources: Vec<(WireId, NetId)>,
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
    let mut resource_usage = PlacementResourceUsage::default();
    for unit in &units {
        let choice = match &unit.choices {
            PlacementChoices::Shared(assignments) => choose_assignment(
                &unit.cells,
                assignments.iter().map(Vec::as_slice),
                graph,
                constraints,
                device,
                &neighbors,
                &placed,
                &occupied,
                &resource_usage,
            ),
            PlacementChoices::SingleCell(candidates) => choose_assignment(
                &unit.cells,
                candidates.iter().map(std::slice::from_ref),
                graph,
                constraints,
                device,
                &neighbors,
                &placed,
                &occupied,
                &resource_usage,
            ),
        };
        let assignment = choice.ok_or_else(|| PnrError::NoBel {
            cell: design.cells()[unit.cells[0].0].name.clone(),
        })?;
        for (&cell, &bel) in unit.cells.iter().zip(&assignment) {
            occupied.insert(bel);
            placed[cell.0] = Some(bel);
        }
        update_placement_resource_usage(
            graph,
            constraints,
            &unit.cells,
            &assignment,
            &mut resource_usage,
            true,
        );
    }

    let mut refinement_occupied = dense_placement_occupancy(device, &placed);
    let _ = refine_placement(
        graph,
        constraints,
        &units,
        &neighbors,
        &mut placed,
        &mut refinement_occupied,
        None,
        None,
    );

    finish_placement(graph, constraints, placed)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum AnalyticalGlobalPlacement {
    Electrostatic,
    Coarse,
}

#[allow(
    clippy::similar_names,
    clippy::too_many_arguments,
    clippy::too_many_lines
)]
fn analytical_place(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    units: &[PlacementUnit],
    spatial_indexes: &BTreeMap<(u8, usize), Arc<SpatialChoiceIndex>>,
    sink_weights: &BTreeMap<(NetId, CellPinId), u64>,
    anchor: Option<(&Placement, u32)>,
    global_placement: AnalyticalGlobalPlacement,
    routing_capacity: Option<&RoutingCapacityMap>,
    register_controls: &[RegisterControlSet],
) -> Result<Placement, PnrError> {
    const ANCHOR_ALPHA: f64 = 0.1;
    let design = graph.design();
    let device = graph.device();
    let mut unit_by_cell = vec![usize::MAX; design.cells().len()];
    let mut column_by_cell = vec![usize::MAX; design.cells().len()];
    let mut macro_offset_by_cell = vec![(0.0, 0.0); design.cells().len()];
    let mut fixed_x_by_cell = vec![None; design.cells().len()];
    let mut fixed_y_by_cell = vec![None; design.cells().len()];
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
            if unit.choices.len() == 1 {
                fixed_x_by_cell[cell.0] = Some(f64::from(point.x));
                fixed_y_by_cell[cell.0] = Some(f64::from(point.y));
            }
        }
    }
    let macro_offset_x = macro_offset_by_cell
        .iter()
        .map(|&(x, _)| x)
        .collect::<Vec<_>>();
    let macro_offset_y = macro_offset_by_cell
        .iter()
        .map(|&(_, y)| y)
        .collect::<Vec<_>>();
    let hypergraph = AnalyticalHypergraph::new(design, &unit_by_cell, units.len(), sink_weights);

    let fixed = units
        .iter()
        .map(|unit| {
            (unit.choices.len() == 1).then(|| device.bels()[unit.choices.assignment(0)[0].0].point)
        })
        .collect::<Vec<_>>();
    let center = Point::new(device.width() / 2, device.height() / 2);
    let initial_x = vec![f64::from(center.x); units.len()];
    let initial_y = vec![f64::from(center.y); units.len()];
    let initial_edges_x = hypergraph.linearize_axis(&initial_x, &fixed_x_by_cell, &macro_offset_x);
    let initial_edges_y = hypergraph.linearize_axis(&initial_y, &fixed_y_by_cell, &macro_offset_y);
    let equations_x = analytical_axis_equations(
        units,
        device,
        &fixed,
        &column_by_cell,
        &macro_offset_by_cell,
        &initial_edges_x,
        false,
    );
    let equations_y = analytical_axis_equations(
        units,
        device,
        &fixed,
        &column_by_cell,
        &macro_offset_by_cell,
        &initial_edges_y,
        true,
    );
    let mut solved_x = solve_analytical_axis(equations_x, initial_x, center.x)?;
    let mut solved_y = solve_analytical_axis(equations_y, initial_y, center.y)?;
    if let Some((anchor, iteration)) = anchor {
        let edges_x = hypergraph.linearize_axis(&solved_x, &fixed_x_by_cell, &macro_offset_x);
        let edges_y = hypergraph.linearize_axis(&solved_y, &fixed_y_by_cell, &macro_offset_y);
        let mut equations_x = analytical_axis_equations(
            units,
            device,
            &fixed,
            &column_by_cell,
            &macro_offset_by_cell,
            &edges_x,
            false,
        );
        let mut equations_y = analytical_axis_equations(
            units,
            device,
            &fixed,
            &column_by_cell,
            &macro_offset_by_cell,
            &edges_y,
            true,
        );
        for (index, unit) in units.iter().enumerate() {
            if fixed[index].is_some() {
                continue;
            }
            let anchor_bel = anchor.bindings()[unit.cells[0].0];
            let anchor_point = device.bels()[anchor_bel.0].point;
            let distance = (solved_x[index] - f64::from(anchor_point.x)).abs()
                + (solved_y[index] - f64::from(anchor_point.y)).abs();
            // Placement edges use a fanout-normalized integer scale (a
            // one-sink edge is 64), so express the anchor relative to this
            // unit's diagonal instead of relying on an architecture-specific
            // absolute coefficient.
            let x_weight =
                equations_x.diagonal[index].max(1.0) * ANCHOR_ALPHA * f64::from(iteration)
                    / distance.max(1.0);
            let y_weight =
                equations_y.diagonal[index].max(1.0) * ANCHOR_ALPHA * f64::from(iteration)
                    / distance.max(1.0);
            equations_x.add_anchor(index, x_weight, f64::from(anchor_point.x));
            equations_y.add_anchor(index, y_weight, f64::from(anchor_point.y));
        }
        solved_x = solve_analytical_axis(equations_x, solved_x, center.x)?;
        solved_y = solve_analytical_axis(equations_y, solved_y, center.y)?;
    }
    let targets = if anchor.is_some() || global_placement == AnalyticalGlobalPlacement::Coarse {
        solved_x
            .iter()
            .zip(&solved_y)
            .map(|(&x, &y)| {
                Point::new(
                    rounded_coordinate(x, device.width()),
                    rounded_coordinate(y, device.height()),
                )
            })
            .collect::<Vec<_>>()
    } else {
        let fixed_coordinates = fixed_x_by_cell
            .iter()
            .zip(&fixed_y_by_cell)
            .map(|(&x, &y)| x.zip(y))
            .collect::<Vec<_>>();
        eplace::place(
            graph,
            units,
            &hypergraph,
            &solved_x,
            &solved_y,
            &fixed_coordinates,
            &macro_offset_by_cell,
            routing_capacity,
            register_controls,
        )
        .map_err(|error| PnrError::InvalidPlacement {
            reason: format!("electrostatic global placement failed: {error:?}"),
        })?
    };
    let placed = legalization::project(graph, constraints, units, spatial_indexes, &targets)?;
    if global_placement == AnalyticalGlobalPlacement::Electrostatic
        && std::env::var_os("TEXO_PNR_METRICS").is_some()
    {
        emit_eplace_legalization_metrics(graph, units, &targets, &placed);
    }
    finish_placement(graph, constraints, placed)
}

#[derive(Default)]
struct EplaceLegalizationMetric {
    targets: Vec<Point>,
    legalized: Vec<Point>,
    displacement: Vec<u64>,
}

/// Compares the rounded continuous solution with the exact legal projection.
///
/// Macro members contribute independently to their own physical resource
/// field, but their target points are reconstructed from the one shared unit
/// origin and the immutable assignment-row offsets.  Thus this diagnostic
/// cannot accidentally split a carry/LUT/register rigid macro.
fn emit_eplace_legalization_metrics(
    graph: &UnifiedGraph<'_>,
    units: &[PlacementUnit],
    targets: &[Point],
    placed: &[Option<BelId>],
) {
    const CONFLICT_MIN_X: u32 = 59;
    const CONFLICT_MAX_X: u32 = 79;
    const CONFLICT_MIN_Y: u32 = 35;
    const CONFLICT_MAX_Y: u32 = 45;

    let design = graph.design();
    let device = graph.device();
    let mut metrics = BTreeMap::<Option<ResourceKind>, EplaceLegalizationMetric>::new();
    for (unit_index, unit) in units.iter().enumerate() {
        let reference = unit.choices.assignment(0);
        let reference_origin = device.bels()[reference[0].0].point;
        let target_origin = targets[unit_index];
        for (&cell, &reference_bel) in unit.cells.iter().zip(reference) {
            let reference_point = device.bels()[reference_bel.0].point;
            let target_x = i64::from(target_origin.x) + i64::from(reference_point.x)
                - i64::from(reference_origin.x);
            let target_y = i64::from(target_origin.y) + i64::from(reference_point.y)
                - i64::from(reference_origin.y);
            let Ok(target_x) = u32::try_from(target_x) else {
                continue;
            };
            let Ok(target_y) = u32::try_from(target_y) else {
                continue;
            };
            let target = Point::new(target_x, target_y);
            let Some(legal_bel) = placed[cell.0] else {
                continue;
            };
            let legal = device.bels()[legal_bel.0].point;
            let kind = design.cells()[cell.0].kind;
            for key in [None, Some(kind)] {
                let metric = metrics.entry(key).or_default();
                metric.targets.push(target);
                metric.legalized.push(legal);
                metric.displacement.push(target.manhattan(legal));
            }
        }
    }

    for (kind, mut metric) in metrics {
        metric.displacement.sort_unstable();
        let cells = metric.displacement.len();
        let total = metric.displacement.iter().copied().sum::<u64>();
        let p50 = nearest_rank(&metric.displacement, 50);
        let p95 = nearest_rank(&metric.displacement, 95);
        let maximum = metric.displacement.last().copied().unwrap_or(0);
        let label = kind.map_or_else(|| "All".to_owned(), |kind| format!("{kind:?}"));
        eprintln!(
            "TEXO_PNR_METRICS eplace-legalization-displacement kind={label} cells={cells} total={total} p50={p50} p95={p95} max={maximum}"
        );
        for (state, points) in [
            ("target", metric.targets.as_slice()),
            ("legalized", metric.legalized.as_slice()),
        ] {
            let conflict_box = points
                .iter()
                .filter(|point| {
                    (CONFLICT_MIN_X..=CONFLICT_MAX_X).contains(&point.x)
                        && (CONFLICT_MIN_Y..=CONFLICT_MAX_Y).contains(&point.y)
                })
                .count();
            eprintln!(
                "TEXO_PNR_METRICS eplace-legalization-density state={state} kind={label} conflict_box={conflict_box} window3={} window5={} window15={} conflict_x={CONFLICT_MIN_X}..{CONFLICT_MAX_X} conflict_y={CONFLICT_MIN_Y}..{CONFLICT_MAX_Y}",
                maximum_window_occupancy(points, device.width(), device.height(), 3),
                maximum_window_occupancy(points, device.width(), device.height(), 5),
                maximum_window_occupancy(points, device.width(), device.height(), 15),
            );
        }
    }
}

fn nearest_rank(sorted: &[u64], percentile: usize) -> u64 {
    if sorted.is_empty() {
        return 0;
    }
    let rank = sorted.len().saturating_mul(percentile).saturating_add(99) / 100;
    sorted[rank.saturating_sub(1).min(sorted.len() - 1)]
}

fn maximum_window_occupancy(points: &[Point], width: u32, height: u32, side: u32) -> usize {
    let width = width as usize;
    let height = height as usize;
    let stride = width + 1;
    let mut prefix = vec![0_usize; (height + 1).saturating_mul(stride)];
    for point in points {
        let x = point.x as usize;
        let y = point.y as usize;
        if x < width && y < height {
            prefix[(y + 1) * stride + x + 1] += 1;
        }
    }
    for y in 1..=height {
        let mut row = 0_usize;
        for x in 1..=width {
            row += prefix[y * stride + x];
            prefix[y * stride + x] = row + prefix[(y - 1) * stride + x];
        }
    }
    let side = side as usize;
    let mut maximum = 0_usize;
    for y0 in 0..height {
        let y1 = y0.saturating_add(side).min(height);
        for x0 in 0..width {
            let x1 = x0.saturating_add(side).min(width);
            let count = prefix[y1 * stride + x1]
                .saturating_add(prefix[y0 * stride + x0])
                .saturating_sub(prefix[y0 * stride + x1])
                .saturating_sub(prefix[y1 * stride + x0]);
            maximum = maximum.max(count);
        }
    }
    maximum
}

#[cfg(test)]
struct ProjectedDescent<T> {
    value: T,
    objective: AnalyticalObjective,
    alpha: f64,
}

#[cfg(test)]
fn first_dyadic_strict_descent<T, E>(
    incumbent: AnalyticalObjective,
    mut evaluate: impl FnMut(f64) -> Result<Option<(T, AnalyticalObjective)>, E>,
) -> Result<Option<ProjectedDescent<T>>, E> {
    let mut alpha = 1.0_f64;
    loop {
        let Some((value, objective)) = evaluate(alpha)? else {
            return Ok(None);
        };
        if objective.total < incumbent.total {
            return Ok(Some(ProjectedDescent {
                value,
                objective,
                alpha,
            }));
        }
        alpha *= 0.5;
        if alpha == 0.0 {
            return Ok(None);
        }
    }
}

#[derive(Clone, Debug)]
struct AxisEquations {
    diagonal: Vec<f64>,
    adjacency: Vec<Vec<(usize, f64)>>,
    rhs: Vec<f64>,
    anchored: Vec<bool>,
}

impl AxisEquations {
    fn add_anchor(&mut self, unit: usize, weight: f64, coordinate: f64) {
        debug_assert!(weight.is_finite() && weight > 0.0);
        self.diagonal[unit] += weight;
        self.rhs[unit] += weight * coordinate;
        self.anchored[unit] = true;
    }

    fn finalize_component_gauges(&mut self) -> Vec<Vec<usize>> {
        let mut visited = vec![false; self.diagonal.len()];
        let mut floating = Vec::new();
        let mut gauge = vec![false; self.diagonal.len()];
        for start in 0..self.diagonal.len() {
            if visited[start] {
                continue;
            }
            visited[start] = true;
            let mut pending = vec![start];
            let mut component = Vec::new();
            while let Some(unit) = pending.pop() {
                component.push(unit);
                for &(other, _) in &self.adjacency[unit] {
                    if !visited[other] {
                        visited[other] = true;
                        pending.push(other);
                    }
                }
            }
            component.sort_unstable();
            if component.iter().any(|&unit| self.anchored[unit]) {
                continue;
            }
            gauge[component[0]] = true;
            floating.push(component);
        }

        // Eliminate one stable representative per floating component at zero.
        // Every neighbor keeps the representative edge in its diagonal, while
        // both sparse off-diagonal entries disappear, preserving symmetry.
        for unit in 0..self.adjacency.len() {
            if gauge[unit] {
                self.diagonal[unit] = 1.0;
                self.rhs[unit] = 0.0;
                self.adjacency[unit].clear();
            } else {
                self.adjacency[unit].retain(|&(other, _)| !gauge[other]);
            }
        }
        debug_assert!(self.diagonal.iter().all(|&entry| entry > 0.0));
        floating
    }
}

fn analytical_axis_equations(
    units: &[PlacementUnit],
    device: &Device,
    fixed: &[Option<Point>],
    column_by_cell: &[usize],
    macro_offset_by_cell: &[(f64, f64)],
    edges: &[AxisEdge],
    y_axis: bool,
) -> AxisEquations {
    let mut diagonal = vec![0.0; units.len()];
    let mut adjacency = vec![Vec::<(usize, f64)>::new(); units.len()];
    let mut rhs = vec![0.0; units.len()];
    let mut anchored = vec![false; units.len()];
    for (unit, point) in fixed.iter().copied().enumerate() {
        if let Some(point) = point {
            diagonal[unit] = 1.0;
            rhs[unit] = f64::from(if y_axis { point.y } else { point.x });
            anchored[unit] = true;
        }
    }
    let offset = |cell: CellId| {
        let offsets = macro_offset_by_cell[cell.0];
        if y_axis { offsets.1 } else { offsets.0 }
    };
    let fixed_coordinate = |unit: usize, cell: CellId| {
        let bel = units[unit].choices.assignment(0)[column_by_cell[cell.0]];
        let point = device.bels()[bel.0].point;
        f64::from(if y_axis { point.y } else { point.x })
    };
    for edge in edges {
        let left_offset = offset(edge.left_cell);
        let right_offset = offset(edge.right_cell);
        let weight = edge.weight;
        match (fixed[edge.left], fixed[edge.right]) {
            (Some(_), Some(_)) => {}
            (Some(_), None) => {
                diagonal[edge.right] += weight;
                rhs[edge.right] +=
                    weight * (fixed_coordinate(edge.left, edge.left_cell) - right_offset);
                anchored[edge.right] = true;
            }
            (None, Some(_)) => {
                diagonal[edge.left] += weight;
                rhs[edge.left] +=
                    weight * (fixed_coordinate(edge.right, edge.right_cell) - left_offset);
                anchored[edge.left] = true;
            }
            (None, None) => {
                diagonal[edge.left] += weight;
                diagonal[edge.right] += weight;
                adjacency[edge.left].push((edge.right, weight));
                adjacency[edge.right].push((edge.left, weight));
                rhs[edge.left] += weight * (right_offset - left_offset);
                rhs[edge.right] += weight * (left_offset - right_offset);
            }
        }
    }
    AxisEquations {
        diagonal,
        adjacency,
        rhs,
        anchored,
    }
}

fn solve_analytical_axis(
    mut equations: AxisEquations,
    mut solution: Vec<f64>,
    center: u32,
) -> Result<Vec<f64>, PnrError> {
    let floating = equations.finalize_component_gauges();
    for component in &floating {
        let gauge_coordinate = solution[component[0]];
        for &unit in component {
            solution[unit] -= gauge_coordinate;
        }
    }
    let mut solution = solve_quadratic(
        &equations.diagonal,
        &equations.adjacency,
        &equations.rhs,
        solution,
    )?;
    for component in floating {
        let mean = component.iter().map(|&unit| solution[unit]).sum::<f64>()
            / component_len_as_f64(component.len());
        let translation = f64::from(center) - mean;
        for unit in component {
            solution[unit] += translation;
        }
    }
    Ok(solution)
}

#[allow(clippy::cast_precision_loss)]
fn component_len_as_f64(length: usize) -> f64 {
    // A component large enough for this cast to lose an integer cannot fit in
    // addressable memory, while avoiding a narrower integer adds no bound.
    length as f64
}

#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn rounded_coordinate(value: f64, extent: u32) -> u32 {
    value.round().clamp(0.0, f64::from(extent - 1)) as u32
}

fn install_assignment(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    unit: &PlacementUnit,
    assignment: &[BelId],
    placed: &mut [Option<BelId>],
    occupied: &mut BTreeSet<BelId>,
    pin_usage: &mut PlacementResourceUsage,
) {
    for (&cell, &bel) in unit.cells.iter().zip(assignment) {
        occupied.insert(bel);
        placed[cell.0] = Some(bel);
    }
    update_placement_resource_usage(graph, constraints, &unit.cells, assignment, pin_usage, true);
}

fn solve_quadratic(
    diagonal: &[f64],
    adjacency: &[Vec<(usize, f64)>],
    rhs: &[f64],
    mut solution: Vec<f64>,
) -> Result<Vec<f64>, PnrError> {
    // Preserve the former stopping test, which compares squared residuals.
    // Its value therefore corresponds to a 1e-4 relative residual norm.
    const RELATIVE_RESIDUAL_SQUARED_TOLERANCE: f64 = 1e-8;
    debug_assert_eq!(diagonal.len(), adjacency.len());
    debug_assert_eq!(diagonal.len(), rhs.len());
    debug_assert_eq!(diagonal.len(), solution.len());
    debug_assert!(
        diagonal
            .iter()
            .all(|&entry| entry.is_finite() && entry > 0.0)
    );
    let started = std::time::Instant::now();
    let mut product = vec![0.0; diagonal.len()];
    multiply_quadratic(diagonal, adjacency, &solution, &mut product);
    let mut residual = rhs
        .iter()
        .zip(&product)
        .map(|(&rhs, &product)| rhs - product)
        .collect::<Vec<_>>();
    let mut residual_squared = dot(&residual, &residual);
    let initial_residual_squared = residual_squared;
    let reference_squared = initial_residual_squared.max(f64::EPSILON);
    let target_squared = reference_squared * RELATIVE_RESIDUAL_SQUARED_TOLERANCE;
    let mut preconditioned = vec![0.0; diagonal.len()];
    let mut rho = apply_jacobi_preconditioner(&residual, diagonal, &mut preconditioned);
    let mut direction = preconditioned.clone();
    let mut iterations = 0_usize;
    let mut breakdown = false;
    // In exact arithmetic, (preconditioned) conjugate gradients spans at most
    // the matrix dimension. This is a mathematical Krylov-space bound, not a
    // placement-tuned iteration budget.
    for _ in 0..diagonal.len() {
        if residual_squared <= target_squared {
            break;
        }
        multiply_quadratic(diagonal, adjacency, &direction, &mut product);
        let denominator = dot(&direction, &product);
        if !denominator.is_finite() || denominator <= 0.0 || !rho.is_finite() || rho <= 0.0 {
            breakdown = true;
            break;
        }
        let alpha = rho / denominator;
        if !alpha.is_finite() {
            breakdown = true;
            break;
        }
        for ((solution, residual), (&direction, &product)) in solution
            .iter_mut()
            .zip(&mut residual)
            .zip(direction.iter().zip(&product))
        {
            *solution += alpha * direction;
            *residual -= alpha * product;
        }
        iterations += 1;
        residual_squared = dot(&residual, &residual);
        if residual_squared <= target_squared {
            // Recursive CG residuals drift in finite precision. Recompute the
            // true residual before accepting convergence; if it is not small
            // enough, restart the Krylov recurrence from that exact residual.
            multiply_quadratic(diagonal, adjacency, &solution, &mut product);
            for ((residual, &rhs), &product) in residual.iter_mut().zip(rhs).zip(&product) {
                *residual = rhs - product;
            }
            residual_squared = dot(&residual, &residual);
            if residual_squared <= target_squared {
                break;
            }
            rho = apply_jacobi_preconditioner(&residual, diagonal, &mut preconditioned);
            if !rho.is_finite() || rho <= 0.0 {
                breakdown = true;
                break;
            }
            direction.clone_from(&preconditioned);
            continue;
        }
        let next_rho = apply_jacobi_preconditioner(&residual, diagonal, &mut preconditioned);
        if !next_rho.is_finite() || next_rho <= 0.0 {
            breakdown = true;
            break;
        }
        let beta = next_rho / rho;
        for (direction, &preconditioned) in direction.iter_mut().zip(&preconditioned) {
            *direction = preconditioned + beta * *direction;
        }
        rho = next_rho;
    }
    finish_quadratic_solve(
        diagonal,
        adjacency,
        rhs,
        solution,
        &QuadraticSolveProgress {
            initial_residual_squared,
            target_squared,
            iterations,
            breakdown,
            started,
        },
    )
}

struct QuadraticSolveProgress {
    initial_residual_squared: f64,
    target_squared: f64,
    iterations: usize,
    breakdown: bool,
    started: std::time::Instant,
}

fn finish_quadratic_solve(
    diagonal: &[f64],
    adjacency: &[Vec<(usize, f64)>],
    rhs: &[f64],
    solution: Vec<f64>,
    progress: &QuadraticSolveProgress,
) -> Result<Vec<f64>, PnrError> {
    // Report the true residual, not only the recursively updated CG vector.
    let mut product = vec![0.0; diagonal.len()];
    multiply_quadratic(diagonal, adjacency, &solution, &mut product);
    let final_residual_squared = rhs
        .iter()
        .zip(&product)
        .map(|(&rhs, &product)| {
            let residual = rhs - product;
            residual * residual
        })
        .sum::<f64>();
    let residual_ratio = if progress.initial_residual_squared > 0.0 {
        (final_residual_squared / progress.initial_residual_squared).sqrt()
    } else {
        0.0
    };
    let converged = final_residual_squared <= progress.target_squared;
    let stop = if converged {
        "residual"
    } else if progress.breakdown {
        "breakdown"
    } else {
        "dimension"
    };
    if std::env::var_os("TEXO_PNR_METRICS").is_some() {
        eprintln!(
            "TEXO_PNR_METRICS analytical-pcg dimension={} iterations={} residual_ratio={residual_ratio:.6e} final_residual={:.6e} stop={stop} elapsed_ms={}",
            diagonal.len(),
            progress.iterations,
            final_residual_squared.sqrt(),
            progress.started.elapsed().as_millis(),
        );
    }
    if !converged {
        return Err(PnrError::InvalidPlacement {
            reason: format!(
                "analytical PCG {stop} after {}/{} iterations (relative residual {residual_ratio:.6e})",
                progress.iterations,
                diagonal.len(),
            ),
        });
    }
    Ok(solution)
}

fn apply_jacobi_preconditioner(
    residual: &[f64],
    diagonal: &[f64],
    preconditioned: &mut [f64],
) -> f64 {
    for ((preconditioned, &residual), &diagonal) in
        preconditioned.iter_mut().zip(residual).zip(diagonal)
    {
        *preconditioned = residual / diagonal;
    }
    dot(residual, preconditioned)
}

fn multiply_quadratic(
    diagonal: &[f64],
    adjacency: &[Vec<(usize, f64)>],
    values: &[f64],
    product: &mut [f64],
) {
    for (index, ((product, &diagonal), edges)) in
        product.iter_mut().zip(diagonal).zip(adjacency).enumerate()
    {
        *product = edges
            .iter()
            .fold(diagonal * values[index], |sum, &(other, weight)| {
                sum - weight * values[other]
            });
    }
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
    validate_complete_placement_resources(graph, constraints, &bindings)?;
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

fn validate_complete_placement_resources(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    bindings: &[BelId],
) -> Result<(), PnrError> {
    for (group_index, group) in constraints.groups.iter().enumerate() {
        if group
            .cells
            .iter()
            .any(|cell| bindings.get(cell.0).is_none())
        {
            return Err(PnrError::InvalidPlacement {
                reason: format!(
                    "placement group {group_index} refers to a cell outside the complete binding table"
                ),
            });
        }
        let row_matches = |row: &[BelId]| {
            row.len() == group.cells.len()
                && group
                    .cells
                    .iter()
                    .zip(row)
                    .all(|(cell, bel)| bindings[cell.0] == *bel)
        };
        let matches_complete_row = if group.cells.is_empty() {
            group.assignments.iter().any(Vec::is_empty)
        } else {
            constraints.group_row_indexes[group_index]
                .get(&bindings[group.cells[0].0])
                .is_some_and(|candidate_rows| {
                    candidate_rows
                        .iter()
                        .any(|&row_index| row_matches(&group.assignments[row_index]))
                })
        };
        if !matches_complete_row {
            return Err(PnrError::InvalidPlacement {
                reason: format!(
                    "placement group {group_index} does not match one complete legal assignment row"
                ),
            });
        }
    }

    let mut usage = PlacementResourceUsage::default();
    for (index, &bel) in bindings.iter().enumerate() {
        let cell = CellId(index);
        if !assignment_resources_are_legal(graph, constraints, &[cell], &[bel], &usage) {
            return Err(PnrError::InvalidPlacement {
                reason: format!(
                    "cell {} conflicts with a shared physical placement resource",
                    graph.design().cells()[index].name
                ),
            });
        }
        update_placement_resource_usage(graph, constraints, &[cell], &[bel], &mut usage, true);
    }
    Ok(())
}

fn validate_refinement_start(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    units: &[PlacementUnit],
    placement: Placement,
    spatial_indexes: Option<&BTreeMap<(u8, usize), Arc<SpatialChoiceIndex>>>,
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
        let assignment_is_known = spatial_indexes.map_or_else(
            || unit.choices.contains(&assignment),
            |indexes| {
                let point = graph.device().bels()[assignment[0].0].point;
                let spatial_index = &indexes[&unit.choices.cache_key()];
                spatial_index.by_point[(point.y * graph.device().width() + point.x) as usize]
                    .iter()
                    .any(|&choice| unit.choices.assignment(choice) == assignment)
            },
        );
        if !assignment_is_known {
            return Err(PnrError::InvalidPlacement {
                reason: format!(
                    "cell group beginning at {} has an incompatible assignment",
                    unit.cells[0].0
                ),
            });
        }
    }
    validate_complete_placement_resources(graph, constraints, &placement.bindings)?;
    Ok(placement.bindings.into_iter().map(Some).collect())
}

fn dense_placement_occupancy(device: &Device, placed: &[Option<BelId>]) -> Vec<bool> {
    let mut occupied = vec![false; device.bels().len()];
    for bel in placed.iter().flatten() {
        occupied[bel.0] = true;
    }
    occupied
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
        let fanout_weight = fanout_placement_weight(net.sinks.len());
        // Wide nets stay out of the star model, but discarding every edge also
        // discards sink-local STA feedback. Retain only the strongest timed
        // sink(s); restoring every non-unit weight collapses a broad decode
        // tree around its driver and recreates the original congestion.
        let strongest_sink_weight = (net.sinks.len() > MAX_LOCAL_STAR_FANOUT)
            .then(|| {
                sink_weights.and_then(|weights| {
                    net.sinks
                        .iter()
                        .filter_map(|sink| weights.get(&(NetId(net_index), *sink)))
                        .copied()
                        .filter(|&weight| weight > 1)
                        .max()
                })
            })
            .flatten();
        for &sink_pin in &net.sinks {
            let sink_timing_weight = sink_weights
                .and_then(|weights| weights.get(&(NetId(net_index), sink_pin)))
                .copied();
            let timing_weight = sink_timing_weight.unwrap_or(net_timing_weight);
            let budget = sink_budgets
                .and_then(|budgets| budgets.get(&(NetId(net_index), sink_pin)))
                .copied()
                .unwrap_or(0);
            let retains_high_fanout_sink = sink_timing_weight
                .zip(strongest_sink_weight)
                .is_some_and(|(weight, strongest)| weight == strongest);
            let edge_weight = if design.cells()[driver.0].kind == texo_model::ResourceKind::Clock
                || (net.sinks.len() > MAX_LOCAL_STAR_FANOUT && !retains_high_fanout_sink)
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

// The local greedy/refinement objective remains a star for compatibility with
// its public APIs. Analytical global placement uses `AnalyticalHypergraph`
// instead and never consults this legacy local-work bound.
const MAX_LOCAL_STAR_FANOUT: usize = 256;

/// Per-sink star-model weight with square-root fanout normalization.
///
/// Dividing by the full fanout made a wide net's *total* placement influence
/// constant: one outlying sink on a 64-way decode net cost no more than 1/64
/// of an ordinary edge, and sink-local timing feedback was diluted by the same
/// factor. No normalization makes wide control nets dominate the solve. The
/// square-root model keeps their aggregate influence proportional to
/// `sqrt(fanout)` while preserving the historical 64-point scale.
fn fanout_placement_weight(fanout: usize) -> u64 {
    let fanout = u64::try_from(fanout.max(1)).expect("fanout fits u64");
    let root = fanout.isqrt();
    let divisor = root + u64::from(root.saturating_mul(root) < fanout);
    (64 / divisor).max(1)
}

#[allow(clippy::too_many_arguments)]
fn choose_assignment<'a>(
    cells: &[CellId],
    assignments: impl Iterator<Item = &'a [BelId]>,
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    device: &Device,
    neighbors: &[Vec<PlacementNeighbor>],
    placed: &[Option<BelId>],
    occupied: &BTreeSet<BelId>,
    resource_usage: &PlacementResourceUsage,
) -> Option<Vec<BelId>> {
    assignments
        .filter(|assignment| assignment.iter().all(|bel| !occupied.contains(bel)))
        .filter(|assignment| {
            assignment_resources_are_legal(graph, constraints, cells, assignment, resource_usage)
        })
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

#[derive(Default)]
struct DetailedPlacementIncident {
    nets: Vec<NetId>,
    timing_arcs: Vec<(NetId, CellPinId)>,
}

struct DetailedPlacementCosts {
    net_bbox: DetailedBoundingBoxCache,
    timing_arcs: BTreeMap<(NetId, CellPinId), u128>,
    totals: (u128, u128),
}

#[derive(Default)]
struct DetailedCoordinateMultiset {
    counts: BTreeMap<u32, usize>,
}

#[derive(Default)]
struct DetailedCoordinateDelta {
    removals: Vec<(u32, usize)>,
    additions: Vec<(u32, usize)>,
}

impl DetailedCoordinateDelta {
    fn increment(entries: &mut Vec<(u32, usize)>, coordinate: u32, multiplicity: usize) {
        match entries.binary_search_by_key(&coordinate, |&(coordinate, _)| coordinate) {
            Ok(index) => {
                entries[index].1 = entries[index]
                    .1
                    .checked_add(multiplicity)
                    .expect("net endpoint multiplicity fits usize");
            }
            Err(index) => entries.insert(index, (coordinate, multiplicity)),
        }
    }

    fn count(entries: &[(u32, usize)], coordinate: u32) -> usize {
        entries
            .binary_search_by_key(&coordinate, |&(coordinate, _)| coordinate)
            .map_or(0, |index| entries[index].1)
    }

    fn remove(&mut self, coordinate: u32, multiplicity: usize) {
        Self::increment(&mut self.removals, coordinate, multiplicity);
    }

    fn add(&mut self, coordinate: u32, multiplicity: usize) {
        Self::increment(&mut self.additions, coordinate, multiplicity);
    }

    fn clear(&mut self) {
        self.removals.clear();
        self.additions.clear();
    }
}

impl DetailedCoordinateMultiset {
    fn insert(&mut self, coordinate: u32, multiplicity: usize) {
        let count = self.counts.entry(coordinate).or_default();
        *count = count
            .checked_add(multiplicity)
            .expect("net endpoint multiplicity fits usize");
    }

    fn count_after_delta(&self, coordinate: u32, delta: &DetailedCoordinateDelta) -> Option<usize> {
        self.counts
            .get(&coordinate)
            .copied()
            .unwrap_or(0)
            .checked_sub(DetailedCoordinateDelta::count(&delta.removals, coordinate))?
            .checked_add(DetailedCoordinateDelta::count(&delta.additions, coordinate))
    }

    fn extrema_after_delta(&self, delta: &DetailedCoordinateDelta) -> Option<(u32, u32)> {
        // Only coordinates present in `removals` can disappear from the
        // current extrema.  Consequently these searches skip at most the
        // number of coordinates touched by the candidate, not the fanout of
        // the net.  A newly inserted coordinate is considered separately so
        // an addition outside the old bounding box is still exact.
        let retained_minimum = self.counts.keys().copied().find(|&coordinate| {
            self.count_after_delta(coordinate, delta)
                .is_some_and(|count| count != 0)
        });
        let retained_maximum = self.counts.keys().rev().copied().find(|&coordinate| {
            self.count_after_delta(coordinate, delta)
                .is_some_and(|count| count != 0)
        });
        let added_minimum = delta.additions.first().map(|&(coordinate, _)| coordinate);
        let added_maximum = delta.additions.last().map(|&(coordinate, _)| coordinate);
        match (
            retained_minimum.into_iter().chain(added_minimum).min(),
            retained_maximum.into_iter().chain(added_maximum).max(),
        ) {
            (Some(minimum), Some(maximum)) => Some((minimum, maximum)),
            (None, None) => None,
            _ => unreachable!("a coordinate multiset has either both extrema or neither"),
        }
    }

    fn apply_delta(&mut self, delta: &DetailedCoordinateDelta) -> Option<()> {
        for &(coordinate, removed) in &delta.removals {
            let count = self.counts.get_mut(&coordinate)?;
            *count = count.checked_sub(removed)?;
            if *count == 0 {
                self.counts.remove(&coordinate);
            }
        }
        for &(coordinate, added) in &delta.additions {
            self.insert(coordinate, added);
        }
        Some(())
    }
}

#[derive(Default)]
struct DetailedNetBoundingBox {
    x: DetailedCoordinateMultiset,
    y: DetailedCoordinateMultiset,
}

#[derive(Default)]
struct DetailedNetBoundingBoxDelta {
    x: DetailedCoordinateDelta,
    y: DetailedCoordinateDelta,
}

impl DetailedNetBoundingBoxDelta {
    fn clear(&mut self) {
        self.x.clear();
        self.y.clear();
    }
}

struct DetailedBoundingBoxDeltaWorkspace {
    deltas: Vec<DetailedNetBoundingBoxDelta>,
    touched: Vec<NetId>,
    active: Vec<bool>,
}

impl DetailedBoundingBoxDeltaWorkspace {
    fn new(net_count: usize) -> Self {
        Self {
            deltas: (0..net_count)
                .map(|_| DetailedNetBoundingBoxDelta::default())
                .collect(),
            touched: Vec::new(),
            active: vec![false; net_count],
        }
    }

    fn clear(&mut self) {
        for net in self.touched.drain(..) {
            self.deltas[net.0].clear();
            self.active[net.0] = false;
        }
    }

    fn touch(&mut self, net: NetId) -> Option<&mut DetailedNetBoundingBoxDelta> {
        let active = self.active.get_mut(net.0)?;
        if !*active {
            *active = true;
            self.touched.push(net);
        }
        self.deltas.get_mut(net.0)
    }

    fn finish(&mut self) {
        self.touched.sort_unstable();
    }

    fn get(&self, net: NetId) -> Option<&DetailedNetBoundingBoxDelta> {
        self.active
            .get(net.0)
            .copied()
            .unwrap_or(false)
            .then(|| &self.deltas[net.0])
    }

    fn iter(&self) -> impl Iterator<Item = (NetId, &DetailedNetBoundingBoxDelta)> {
        self.touched
            .iter()
            .copied()
            .map(|net| (net, &self.deltas[net.0]))
    }
}

impl DetailedNetBoundingBox {
    fn insert(&mut self, point: Point, multiplicity: usize) {
        self.x.insert(point.x, multiplicity);
        self.y.insert(point.y, multiplicity);
    }

    fn cost_after_delta(&self, delta: &DetailedNetBoundingBoxDelta) -> Option<u64> {
        match (
            self.x.extrema_after_delta(&delta.x),
            self.y.extrema_after_delta(&delta.y),
        ) {
            (Some((minimum_x, maximum_x)), Some((minimum_y, maximum_y))) => {
                Some(u64::from(maximum_x - minimum_x) + u64::from(maximum_y - minimum_y))
            }
            // Clock nets are intentionally absent from this cache and retain
            // the historical zero placement cost.
            (None, None) => Some(0),
            _ => None,
        }
    }

    fn cost(&self) -> u64 {
        self.cost_after_delta(&DetailedNetBoundingBoxDelta::default())
            .expect("cached net bounding box has matching coordinate axes")
    }

    fn apply_delta(&mut self, delta: &DetailedNetBoundingBoxDelta) -> Option<()> {
        self.x.apply_delta(&delta.x)?;
        self.y.apply_delta(&delta.y)
    }
}

struct DetailedBoundingBoxCache {
    nets: Vec<DetailedNetBoundingBox>,
    cell_endpoint_multiplicities: Vec<Vec<(NetId, usize)>>,
}

impl DetailedBoundingBoxCache {
    fn new(graph: &UnifiedGraph<'_>, placed: &[Option<BelId>]) -> Option<Self> {
        let design = graph.design();
        let mut cell_endpoint_multiplicities =
            vec![BTreeMap::<NetId, usize>::new(); design.cells().len()];
        for (index, net) in design.nets().iter().enumerate() {
            let driver_cell = design.pins().get(net.driver.0)?.cell;
            if design.cells().get(driver_cell.0)?.kind == texo_model::ResourceKind::Clock {
                continue;
            }
            let mut add_endpoint = |cell: CellId| {
                let count = cell_endpoint_multiplicities[cell.0]
                    .entry(NetId(index))
                    .or_default();
                *count = count
                    .checked_add(1)
                    .expect("net endpoint multiplicity fits usize");
            };
            add_endpoint(driver_cell);
            for &sink in &net.sinks {
                add_endpoint(design.pins().get(sink.0)?.cell);
            }
        }
        let cell_endpoint_multiplicities = cell_endpoint_multiplicities
            .into_iter()
            .map(|incidents| incidents.into_iter().collect::<Vec<_>>())
            .collect::<Vec<_>>();
        let mut nets = (0..design.nets().len())
            .map(|_| DetailedNetBoundingBox::default())
            .collect::<Vec<_>>();
        for (cell_index, incidents) in cell_endpoint_multiplicities.iter().enumerate() {
            if incidents.is_empty() {
                continue;
            }
            let bel = placed.get(cell_index).copied().flatten()?;
            let point = graph.device().bels().get(bel.0)?.point;
            for &(net, multiplicity) in incidents {
                nets[net.0].insert(point, multiplicity);
            }
        }
        Some(Self {
            nets,
            cell_endpoint_multiplicities,
        })
    }

    fn net_cost(&self, net: NetId) -> Option<u64> {
        self.nets.get(net.0).map(DetailedNetBoundingBox::cost)
    }

    fn net_cost_after_delta(
        &self,
        net: NetId,
        delta: Option<&DetailedNetBoundingBoxDelta>,
    ) -> Option<u64> {
        let bounding_box = self.nets.get(net.0)?;
        delta.map_or_else(
            || Some(bounding_box.cost()),
            |delta| bounding_box.cost_after_delta(delta),
        )
    }

    fn trial_deltas(
        &self,
        graph: &UnifiedGraph<'_>,
        replacements: &[(&PlacementUnit, &[BelId])],
        placed: &[Option<BelId>],
        workspace: &mut DetailedBoundingBoxDeltaWorkspace,
    ) -> Option<()> {
        workspace.clear();
        for &(unit, assignment) in replacements {
            if assignment.len() != unit.cells.len() {
                return None;
            }
            for (&cell, &new_bel) in unit.cells.iter().zip(assignment) {
                let old_bel = placed.get(cell.0).copied().flatten()?;
                let old_point = graph.device().bels().get(old_bel.0)?.point;
                let new_point = graph.device().bels().get(new_bel.0)?.point;
                if old_point == new_point {
                    continue;
                }
                for &(net, multiplicity) in self.cell_endpoint_multiplicities.get(cell.0)? {
                    let delta = workspace.touch(net)?;
                    delta.x.remove(old_point.x, multiplicity);
                    delta.x.add(new_point.x, multiplicity);
                    delta.y.remove(old_point.y, multiplicity);
                    delta.y.add(new_point.y, multiplicity);
                }
            }
        }
        workspace.finish();
        Some(())
    }

    fn apply_delta(
        &mut self,
        net: NetId,
        delta: &DetailedNetBoundingBoxDelta,
    ) -> Option<(u64, u64)> {
        let bounding_box = self.nets.get_mut(net.0)?;
        let old = bounding_box.cost();
        let new = bounding_box.cost_after_delta(delta)?;
        bounding_box.apply_delta(delta)?;
        debug_assert_eq!(bounding_box.cost(), new);
        Some((old, new))
    }
}

#[cfg(test)]
mod detailed_bbox_tests {
    use texo_model::Point;

    use super::{DetailedNetBoundingBox, DetailedNetBoundingBoxDelta};

    fn reference_cost(points: &[Point]) -> u64 {
        let minimum_x = points.iter().map(|point| point.x).min().unwrap();
        let maximum_x = points.iter().map(|point| point.x).max().unwrap();
        let minimum_y = points.iter().map(|point| point.y).min().unwrap();
        let maximum_y = points.iter().map(|point| point.y).max().unwrap();
        u64::from(maximum_x - minimum_x) + u64::from(maximum_y - minimum_y)
    }

    fn move_endpoints(
        delta: &mut DetailedNetBoundingBoxDelta,
        old: Point,
        new: Point,
        multiplicity: usize,
    ) {
        delta.x.remove(old.x, multiplicity);
        delta.x.add(new.x, multiplicity);
        delta.y.remove(old.y, multiplicity);
        delta.y.add(new.y, multiplicity);
    }

    #[test]
    fn incremental_bbox_is_exact_for_endpoint_multiplicity_and_simultaneous_moves() {
        let before = [
            Point::new(0, 0),
            Point::new(0, 0),
            Point::new(0, 0),
            Point::new(2, 5),
            Point::new(9, 1),
            Point::new(9, 1),
            Point::new(4, 7),
        ];
        let after = [
            Point::new(0, 0),
            Point::new(12, 3),
            Point::new(12, 3),
            Point::new(2, 5),
            Point::new(2, 5),
            Point::new(9, 1),
            Point::new(0, 0),
        ];
        let mut bounding_box = DetailedNetBoundingBox::default();
        bounding_box.insert(Point::new(0, 0), 3);
        bounding_box.insert(Point::new(2, 5), 1);
        bounding_box.insert(Point::new(9, 1), 2);
        bounding_box.insert(Point::new(4, 7), 1);
        let mut delta = DetailedNetBoundingBoxDelta::default();
        move_endpoints(&mut delta, Point::new(0, 0), Point::new(12, 3), 2);
        move_endpoints(&mut delta, Point::new(9, 1), Point::new(2, 5), 1);
        move_endpoints(&mut delta, Point::new(4, 7), Point::new(0, 0), 1);

        assert_eq!(bounding_box.cost(), reference_cost(&before));
        assert_eq!(
            bounding_box.cost_after_delta(&delta),
            Some(reference_cost(&after))
        );
        bounding_box.apply_delta(&delta).unwrap();
        assert_eq!(bounding_box.cost(), reference_cost(&after));
        assert_eq!(bounding_box.x.counts.get(&0), Some(&2));
        assert_eq!(bounding_box.x.counts.get(&12), Some(&2));
        assert_eq!(bounding_box.y.counts.get(&7), None);
    }
}

fn detailed_placement_incidents(
    design: &Design,
    units: &[PlacementUnit],
    sink_criticalities: &BTreeMap<(NetId, CellPinId), u64>,
) -> Vec<DetailedPlacementIncident> {
    let mut cell_units = vec![None; design.cells().len()];
    for (unit_index, unit) in units.iter().enumerate() {
        for &cell in &unit.cells {
            cell_units[cell.0] = Some(unit_index);
        }
    }
    let mut net_incidents = vec![BTreeSet::new(); units.len()];
    for (index, net) in design.nets().iter().enumerate() {
        let mut touched = BTreeSet::new();
        let driver = design.pins()[net.driver.0].cell;
        if let Some(unit) = cell_units[driver.0] {
            touched.insert(unit);
        }
        for &sink in &net.sinks {
            if let Some(unit) = cell_units[design.pins()[sink.0].cell.0] {
                touched.insert(unit);
            }
        }
        for unit in touched {
            net_incidents[unit].insert(NetId(index));
        }
    }
    let mut timing_incidents = vec![BTreeSet::new(); units.len()];
    for (&arc @ (net, sink), &criticality) in sink_criticalities {
        if criticality <= 1 {
            continue;
        }
        let Some(logical) = design.nets().get(net.0) else {
            continue;
        };
        let Some(sink_pin) = design.pins().get(sink.0) else {
            continue;
        };
        let mut touched = BTreeSet::new();
        let driver = design.pins()[logical.driver.0].cell;
        if let Some(unit) = cell_units[driver.0] {
            touched.insert(unit);
        }
        if let Some(unit) = cell_units[sink_pin.cell.0] {
            touched.insert(unit);
        }
        for unit in touched {
            timing_incidents[unit].insert(arc);
        }
    }
    net_incidents
        .into_iter()
        .zip(timing_incidents)
        .map(|(nets, timing_arcs)| DetailedPlacementIncident {
            nets: nets.into_iter().collect(),
            timing_arcs: timing_arcs.into_iter().collect(),
        })
        .collect()
}

fn weighted_median_coordinate(
    points: &BTreeMap<Point, u128>,
    coordinate: impl Fn(Point) -> u32,
) -> Option<u32> {
    let mut weights = BTreeMap::<u32, u128>::new();
    for (&point, &weight) in points {
        let total = weights.entry(coordinate(point)).or_default();
        *total = total
            .checked_add(weight)
            .expect("physical placement weights fit u128");
    }
    let total = weights
        .values()
        .try_fold(0_u128, |total, &weight| total.checked_add(weight))?;
    let threshold = total.div_ceil(2);
    let mut cumulative = 0_u128;
    weights.into_iter().find_map(|(coordinate, weight)| {
        cumulative = cumulative
            .checked_add(weight)
            .expect("physical placement weights fit u128");
        (cumulative >= threshold).then_some(coordinate)
    })
}

fn detailed_interval_median(intervals: &[(i64, i64)], current: u32, extent: u32) -> Option<u32> {
    let mut endpoints = Vec::with_capacity(intervals.len().checked_mul(2)?);
    for &(lower, upper) in intervals {
        debug_assert!(lower <= upper);
        endpoints.extend([lower, upper]);
    }
    if endpoints.is_empty() {
        return None;
    }
    endpoints.sort_unstable();
    let lower = endpoints[intervals.len() - 1];
    let upper = endpoints[intervals.len()];
    let coordinate = i64::from(current).clamp(lower, upper);
    Some(u32::try_from(coordinate.clamp(0, i64::from(extent - 1))).expect("coordinate is clamped"))
}

/// Exact coordinate-wise minimizer of the incident-net HPWL for a translated
/// placement unit while every other endpoint stays fixed.
///
/// For one net, all translations whose moving endpoint interval overlaps (or
/// contains) the fixed endpoint interval have minimum span. The sum over nets
/// is a sum of distances to those intervals, whose minimizer is the middle
/// pair of their sorted endpoints. This gives one canonical target without an
/// arbitrary spatial radius and retains every rigid macro member offset.
fn detailed_bbox_target(
    graph: &UnifiedGraph<'_>,
    unit: &PlacementUnit,
    incident: &DetailedPlacementIncident,
    placed: &[Option<BelId>],
) -> Option<Point> {
    let design = graph.design();
    let device = graph.device();
    let origin_bel = placed.get(unit.cells[0].0).copied().flatten()?;
    let origin = device.bels().get(origin_bel.0)?.point;
    let mut x_intervals = Vec::new();
    let mut y_intervals = Vec::new();
    for &net in &incident.nets {
        let logical = design.nets().get(net.0)?;
        let driver = design.pins().get(logical.driver.0)?.cell;
        if design.cells().get(driver.0)?.kind == texo_model::ResourceKind::Clock {
            continue;
        }
        let mut external_minimum = None::<Point>;
        let mut external_maximum = None::<Point>;
        let mut moving_minimum = None::<(i64, i64)>;
        let mut moving_maximum = None::<(i64, i64)>;
        let mut include_endpoint = |cell: CellId| -> Option<()> {
            let bel = placed.get(cell.0).copied().flatten()?;
            let point = device.bels().get(bel.0)?.point;
            if unit.cells.contains(&cell) {
                let offset = (
                    i64::from(point.x) - i64::from(origin.x),
                    i64::from(point.y) - i64::from(origin.y),
                );
                moving_minimum = Some(moving_minimum.map_or(offset, |minimum| {
                    (minimum.0.min(offset.0), minimum.1.min(offset.1))
                }));
                moving_maximum = Some(moving_maximum.map_or(offset, |maximum| {
                    (maximum.0.max(offset.0), maximum.1.max(offset.1))
                }));
            } else {
                external_minimum = Some(external_minimum.map_or(point, |minimum| {
                    Point::new(minimum.x.min(point.x), minimum.y.min(point.y))
                }));
                external_maximum = Some(external_maximum.map_or(point, |maximum| {
                    Point::new(maximum.x.max(point.x), maximum.y.max(point.y))
                }));
            }
            Some(())
        };
        include_endpoint(driver)?;
        for &sink in &logical.sinks {
            include_endpoint(design.pins().get(sink.0)?.cell)?;
        }
        let (Some(external_minimum), Some(external_maximum)) = (external_minimum, external_maximum)
        else {
            continue;
        };
        let (Some(moving_minimum), Some(moving_maximum)) = (moving_minimum, moving_maximum) else {
            continue;
        };
        let x_endpoints = (
            i64::from(external_minimum.x) - moving_minimum.0,
            i64::from(external_maximum.x) - moving_maximum.0,
        );
        let y_endpoints = (
            i64::from(external_minimum.y) - moving_minimum.1,
            i64::from(external_maximum.y) - moving_maximum.1,
        );
        x_intervals.push((
            x_endpoints.0.min(x_endpoints.1),
            x_endpoints.0.max(x_endpoints.1),
        ));
        y_intervals.push((
            y_endpoints.0.min(y_endpoints.1),
            y_endpoints.0.max(y_endpoints.1),
        ));
    }
    Some(Point::new(
        detailed_interval_median(&x_intervals, origin.x, device.width())?,
        detailed_interval_median(&y_intervals, origin.y, device.height())?,
    ))
}

fn detailed_member_target_origin(
    graph: &UnifiedGraph<'_>,
    unit: &PlacementUnit,
    member: CellId,
    target: Point,
    placed: &[Option<BelId>],
) -> Option<Point> {
    let device = graph.device();
    let origin_bel = placed.get(unit.cells[0].0).copied().flatten()?;
    let member_bel = placed.get(member.0).copied().flatten()?;
    let origin = device.bels().get(origin_bel.0)?.point;
    let member = device.bels().get(member_bel.0)?.point;
    let translate = |origin: u32, member: u32, target: u32, extent: u32| {
        let coordinate = i64::from(origin) + i64::from(target) - i64::from(member);
        u32::try_from(coordinate.clamp(0, i64::from(extent - 1))).expect("coordinate is clamped")
    };
    Some(Point::new(
        translate(origin.x, member.x, target.x, device.width()),
        translate(origin.y, member.y, target.y, device.height()),
    ))
}

/// Canonical physical targets for one detailed-placement unit.
///
/// The exact incident-net HPWL minimizer supplies the ordinary wirelength
/// target. Every critical arc additionally contributes the origin translation
/// that puts its moving member on the opposite endpoint, together with the
/// criticality-weighted coordinate median of those translations. Targets are
/// recomputed after each accepted placement change and projected onto their
/// nearest legal assignment ring by the caller.
fn detailed_placement_targets(
    graph: &UnifiedGraph<'_>,
    unit: &PlacementUnit,
    incident: &DetailedPlacementIncident,
    placed: &[Option<BelId>],
    sink_criticalities: &BTreeMap<(NetId, CellPinId), u64>,
) -> Vec<Point> {
    let design = graph.design();
    let device = graph.device();
    let mut timing_origins = BTreeMap::<Point, u128>::new();
    for &arc @ (net, sink) in &incident.timing_arcs {
        let logical = &design.nets()[net.0];
        let driver_cell = design.pins()[logical.driver.0].cell;
        let sink_cell = design.pins()[sink.0].cell;
        let contains_driver = unit.cells.contains(&driver_cell);
        let contains_sink = unit.cells.contains(&sink_cell);
        let (moving, opposite) = match (contains_driver, contains_sink) {
            (true, false) => (driver_cell, sink_cell),
            (false, true) => (sink_cell, driver_cell),
            // An internal arc has no external point to pull this atomic unit.
            (true, true) | (false, false) => continue,
        };
        let bel = placed[opposite.0].expect("complete placement has every timing endpoint");
        let opposite_point = device.bels()[bel.0].point;
        let point = detailed_member_target_origin(graph, unit, moving, opposite_point, placed)
            .expect("complete placement has every atomic-unit member");
        let weight = u128::from(
            sink_criticalities[&arc]
                .checked_sub(1)
                .expect("detailed timing incidents have criticality above baseline"),
        );
        let total = timing_origins.entry(point).or_default();
        *total = total
            .checked_add(weight)
            .expect("physical placement weights fit u128");
    }
    let mut targets = timing_origins.keys().copied().collect::<BTreeSet<_>>();
    if let Some(target) = detailed_bbox_target(graph, unit, incident, placed) {
        targets.insert(target);
    }
    if let (Some(x), Some(y)) = (
        weighted_median_coordinate(&timing_origins, |point| point.x),
        weighted_median_coordinate(&timing_origins, |point| point.y),
    ) {
        targets.insert(Point::new(x, y));
    }
    targets.into_iter().collect()
}

fn detailed_assignment_bel(
    cell: CellId,
    replacements: &[(&PlacementUnit, &[BelId])],
    placed: &[Option<BelId>],
) -> Option<BelId> {
    for &(unit, assignment) in replacements {
        if let Some(column) = unit.cells.iter().position(|&member| member == cell) {
            return assignment.get(column).copied();
        }
    }
    placed.get(cell.0).copied().flatten()
}

fn with_detailed_replacements<'a, T>(
    first: (&'a PlacementUnit, &'a [BelId]),
    second: Option<(&'a PlacementUnit, &'a [BelId])>,
    evaluate: impl FnOnce(&[(&'a PlacementUnit, &'a [BelId])]) -> T,
) -> T {
    match second {
        Some(second) => evaluate(&[first, second]),
        None => evaluate(std::slice::from_ref(&first)),
    }
}

#[allow(clippy::too_many_arguments)]
fn candidate_arc_timing_cost(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    replacements: &[(&PlacementUnit, &[BelId])],
    placed: &[Option<BelId>],
    arc: (NetId, CellPinId),
    criticality: u64,
    delay_estimator: &impl PlacementDelayEstimator,
) -> Option<u128> {
    let design = graph.design();
    let (net, sink) = arc;
    let driver = design.nets().get(net.0)?.driver;
    let driver_cell = design.pins().get(driver.0)?.cell;
    if design.cells().get(driver_cell.0)?.kind == texo_model::ResourceKind::Clock {
        return Some(0);
    }
    let sink_cell = design.pins().get(sink.0)?.cell;
    let driver_bel = detailed_assignment_bel(driver_cell, replacements, placed)?;
    let sink_bel = detailed_assignment_bel(sink_cell, replacements, placed)?;
    let driver_pin = candidate_bel_pin(graph, constraints, driver, driver_bel)?;
    let sink_pin = candidate_bel_pin(graph, constraints, sink, sink_bel)?;
    let delay = delay_estimator.estimate_delay_ps(driver_bel, driver_pin, sink_bel, sink_pin);
    u128::from(delay).checked_mul(u128::from(criticality.checked_sub(1)?))
}

fn detailed_placement_costs(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    placed: &[Option<BelId>],
    sink_criticalities: &BTreeMap<(NetId, CellPinId), u64>,
    delay_estimator: &impl PlacementDelayEstimator,
) -> DetailedPlacementCosts {
    let net_bbox = DetailedBoundingBoxCache::new(graph, placed)
        .expect("complete legal placement has every net endpoint");
    let timing_arcs = sink_criticalities
        .iter()
        .filter(|&(_, &criticality)| criticality > 1)
        .filter_map(|(&arc, &criticality)| {
            candidate_arc_timing_cost(
                graph,
                constraints,
                &[],
                placed,
                arc,
                criticality,
                delay_estimator,
            )
            .map(|cost| (arc, cost))
        })
        .collect::<BTreeMap<_, _>>();
    let totals = (
        net_bbox
            .nets
            .iter()
            .map(DetailedNetBoundingBox::cost)
            .map(u128::from)
            .try_fold(0_u128, u128::checked_add)
            .expect("physical placement bbox total fits u128"),
        timing_arcs
            .values()
            .copied()
            .try_fold(0_u128, u128::checked_add)
            .expect("physical placement timing total fits u128"),
    );
    DetailedPlacementCosts {
        net_bbox,
        timing_arcs,
        totals,
    }
}

fn merge_sorted_unique<T: Copy + Ord>(left: &[T], right: &[T], merged: &mut Vec<T>) {
    merged.clear();
    let (mut left_index, mut right_index) = (0, 0);
    while left_index < left.len() || right_index < right.len() {
        let next = match (left.get(left_index), right.get(right_index)) {
            (Some(&left), Some(&right)) if left < right => {
                left_index += 1;
                left
            }
            (Some(&left), Some(&right)) if right < left => {
                right_index += 1;
                right
            }
            (Some(&left), Some(_)) => {
                left_index += 1;
                right_index += 1;
                left
            }
            (Some(&left), None) => {
                left_index += 1;
                left
            }
            (None, Some(&right)) => {
                right_index += 1;
                right
            }
            (None, None) => break,
        };
        merged.push(next);
    }
}

#[allow(clippy::too_many_arguments)]
fn detailed_candidate_costs(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    replacements: &[(&PlacementUnit, &[BelId])],
    placed: &[Option<BelId>],
    affected_nets: &[NetId],
    bbox_deltas: &DetailedBoundingBoxDeltaWorkspace,
    affected_arcs: &[(NetId, CellPinId)],
    sink_criticalities: &BTreeMap<(NetId, CellPinId), u64>,
    delay_estimator: &impl PlacementDelayEstimator,
    costs: &DetailedPlacementCosts,
) -> Option<((u128, u128), (u128, u128))> {
    let old_bbox = affected_nets.iter().try_fold(0_u128, |total, &net| {
        total.checked_add(u128::from(costs.net_bbox.net_cost(net)?))
    })?;
    let new_bbox = affected_nets.iter().try_fold(0_u128, |total, &net| {
        total.checked_add(u128::from(
            costs
                .net_bbox
                .net_cost_after_delta(net, bbox_deltas.get(net))?,
        ))
    })?;
    let mut old_timing = 0_u128;
    let mut new_timing = 0_u128;
    for &arc in affected_arcs {
        let Some(&old) = costs.timing_arcs.get(&arc) else {
            continue;
        };
        old_timing = old_timing.checked_add(old)?;
        let criticality = *sink_criticalities.get(&arc)?;
        new_timing = new_timing.checked_add(candidate_arc_timing_cost(
            graph,
            constraints,
            replacements,
            placed,
            arc,
            criticality,
            delay_estimator,
        )?)?;
    }
    Some(((old_bbox, old_timing), (new_bbox, new_timing)))
}

fn detailed_placement_objective(totals: (u128, u128), normalizer: (u128, u128)) -> Option<u128> {
    totals.0.checked_mul(normalizer.1).and_then(|bbox| {
        totals
            .1
            .checked_mul(normalizer.0)
            .and_then(|timing| bbox.checked_add(timing))
    })
}

fn detailed_totals_after_replacement(
    totals: (u128, u128),
    old: (u128, u128),
    new: (u128, u128),
) -> Option<(u128, u128)> {
    Some((
        totals.0.checked_sub(old.0)?.checked_add(new.0)?,
        totals.1.checked_sub(old.1)?.checked_add(new.1)?,
    ))
}

enum DetailedTargetOccupancy {
    Empty,
    Unit(usize),
    Blocked,
}

fn detailed_target_occupancy(
    moving: usize,
    assignment: &[BelId],
    bel_owner: &[Option<usize>],
) -> DetailedTargetOccupancy {
    let mut other = None;
    for &bel in assignment {
        let Some(owner) = bel_owner[bel.0] else {
            continue;
        };
        if owner == moving {
            continue;
        }
        if other.is_some_and(|known| known != owner) {
            return DetailedTargetOccupancy::Blocked;
        }
        other = Some(owner);
    }
    other.map_or(
        DetailedTargetOccupancy::Empty,
        DetailedTargetOccupancy::Unit,
    )
}

#[derive(Clone, Copy)]
enum DetailedCandidateDestination {
    Empty,
    Swap(usize),
}

impl DetailedCandidateDestination {
    fn partner(self) -> Option<usize> {
        match self {
            Self::Empty => None,
            Self::Swap(partner) => Some(partner),
        }
    }
}

fn detailed_candidate_destination(
    units: &[PlacementUnit],
    assignments: &[Vec<BelId>],
    moving: usize,
    candidate: &[BelId],
    bel_owner: &[Option<usize>],
) -> Option<DetailedCandidateDestination> {
    match detailed_target_occupancy(moving, candidate, bel_owner) {
        DetailedTargetOccupancy::Empty => Some(DetailedCandidateDestination::Empty),
        DetailedTargetOccupancy::Unit(partner) => {
            let moving_unit = &units[moving];
            let other = &units[partner];
            (other.choices.len() > 1
                && other.choices.cache_key() == moving_unit.choices.cache_key()
                && assignments[partner] == candidate)
                .then_some(DetailedCandidateDestination::Swap(partner))
        }
        DetailedTargetOccupancy::Blocked => None,
    }
}

#[allow(clippy::too_many_arguments)]
fn detailed_candidate_is_legal(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    units: &[PlacementUnit],
    assignments: &[Vec<BelId>],
    moving: usize,
    candidate: &[BelId],
    partner: Option<usize>,
    resource_usage_without_moving: &mut PlacementResourceUsage,
) -> bool {
    let unit = &units[moving];
    let Some(partner) = partner else {
        return assignment_resources_are_legal(
            graph,
            constraints,
            &unit.cells,
            candidate,
            resource_usage_without_moving,
        );
    };
    let other = &units[partner];
    let other_current = &assignments[partner];
    update_placement_resource_usage(
        graph,
        constraints,
        &other.cells,
        other_current,
        resource_usage_without_moving,
        false,
    );
    let first_legal = assignment_resources_are_legal(
        graph,
        constraints,
        &unit.cells,
        candidate,
        resource_usage_without_moving,
    );
    if first_legal {
        update_placement_resource_usage(
            graph,
            constraints,
            &unit.cells,
            candidate,
            resource_usage_without_moving,
            true,
        );
    }
    let second_legal = first_legal
        && assignment_resources_are_legal(
            graph,
            constraints,
            &other.cells,
            &assignments[moving],
            resource_usage_without_moving,
        );
    if first_legal {
        update_placement_resource_usage(
            graph,
            constraints,
            &unit.cells,
            candidate,
            resource_usage_without_moving,
            false,
        );
    }
    update_placement_resource_usage(
        graph,
        constraints,
        &other.cells,
        other_current,
        resource_usage_without_moving,
        true,
    );
    second_legal
}

#[allow(clippy::too_many_arguments)]
fn update_detailed_cost_cache(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    placed: &[Option<BelId>],
    bbox_deltas: &DetailedBoundingBoxDeltaWorkspace,
    affected_arcs: &[(NetId, CellPinId)],
    sink_criticalities: &BTreeMap<(NetId, CellPinId), u64>,
    delay_estimator: &impl PlacementDelayEstimator,
    costs: &mut DetailedPlacementCosts,
) {
    for (net, delta) in bbox_deltas.iter() {
        let (old, new) = costs
            .net_bbox
            .apply_delta(net, delta)
            .expect("accepted legal move has a valid endpoint delta");
        costs.totals.0 = costs
            .totals
            .0
            .checked_sub(u128::from(old))
            .and_then(|total| total.checked_add(u128::from(new)))
            .expect("accepted bbox delta preserves an exact u128 total");
    }
    for &arc in affected_arcs {
        let Some(old) = costs.timing_arcs.get(&arc).copied() else {
            continue;
        };
        let criticality = sink_criticalities[&arc];
        let new = candidate_arc_timing_cost(
            graph,
            constraints,
            &[],
            placed,
            arc,
            criticality,
            delay_estimator,
        )
        .expect("accepted legal move has every timing endpoint");
        costs.timing_arcs.insert(arc, new);
        costs.totals.1 = costs
            .totals
            .1
            .checked_sub(old)
            .and_then(|total| total.checked_add(new))
            .expect("accepted timing delta preserves an exact u128 total");
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PredictedPlacementPasses {
    One,
    UntilFixedPoint,
}

#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn refine_predicted_placement(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    units: &[PlacementUnit],
    spatial_indexes: &BTreeMap<(u8, usize), Arc<SpatialChoiceIndex>>,
    neighbors: &[Vec<PlacementNeighbor>],
    sink_criticalities: &BTreeMap<(NetId, CellPinId), u64>,
    delay_estimator: &impl PlacementDelayEstimator,
    placed: &mut [Option<BelId>],
    occupied: &mut [bool],
    passes: PredictedPlacementPasses,
) -> usize {
    let device = graph.device();
    let incidents = detailed_placement_incidents(graph.design(), units, sink_criticalities);
    let mut costs = detailed_placement_costs(
        graph,
        constraints,
        placed,
        sink_criticalities,
        delay_estimator,
    );
    // Keep one objective for the whole refinement run.  Renormalizing after
    // every accepted move changes the wirelength/timing tradeoff mid-pass and
    // makes the greedy ordering affect which objective is being optimized.
    let normalizer = (costs.totals.0.max(1), costs.totals.1.max(1));
    let mut objective = detailed_placement_objective(costs.totals, normalizer)
        .expect("physical placement objective fits u128");
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
    let mut assignments = units
        .iter()
        .map(|unit| {
            unit.cells
                .iter()
                .map(|cell| placed[cell.0].expect("initial placement is complete"))
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let mut bel_owner = vec![None; device.bels().len()];
    let mut resource_usage = PlacementResourceUsage::default();
    for (index, unit) in units.iter().enumerate() {
        for &bel in &assignments[index] {
            bel_owner[bel.0] = Some(index);
        }
        update_placement_resource_usage(
            graph,
            constraints,
            &unit.cells,
            &assignments[index],
            &mut resource_usage,
            true,
        );
    }

    let mut affected_nets = Vec::new();
    let mut affected_arcs = Vec::new();
    let mut bbox_delta_workspace =
        DetailedBoundingBoxDeltaWorkspace::new(costs.net_bbox.nets.len());
    let mut candidates = Vec::<(usize, Option<usize>)>::new();
    let empty_incident = DetailedPlacementIncident::default();
    let mut pass = 0_usize;
    let mut total_moved = 0_usize;
    loop {
        pass += 1;
        let pass_started = std::time::Instant::now();
        let mut moved = 0_usize;
        let mut target_count = 0_usize;
        let mut candidate_count = 0_usize;
        let mut scored_count = 0_usize;
        let mut swap_count = 0_usize;
        let mut target_elapsed = std::time::Duration::ZERO;
        let mut scoring_elapsed = std::time::Duration::ZERO;
        for &index in &order {
            let unit = &units[index];
            if unit.choices.len() <= 1 {
                continue;
            }
            let current = assignments[index].clone();
            update_placement_resource_usage(
                graph,
                constraints,
                &unit.cells,
                &current,
                &mut resource_usage,
                false,
            );

            let spatial_index = &spatial_indexes[&unit.choices.cache_key()];
            candidates.clear();
            let target_started = std::time::Instant::now();
            let targets = detailed_placement_targets(
                graph,
                unit,
                &incidents[index],
                placed,
                sink_criticalities,
            );
            target_count = target_count.saturating_add(targets.len());
            for target in targets {
                let mut classify = |candidate_index: usize| {
                    let candidate = unit.choices.assignment(candidate_index);
                    if candidate == current {
                        return None;
                    }
                    let destination = detailed_candidate_destination(
                        units,
                        &assignments,
                        index,
                        candidate,
                        &bel_owner,
                    )?;
                    let partner = destination.partner();
                    // Exchanging the two endpoints of a target net leaves
                    // that net's geometry unchanged. Do not let such a
                    // structurally null swap terminate ring projection at the
                    // exact target point; the next legal ring can contain a
                    // displacement that contracts both units' incident nets.
                    if partner.is_some_and(|partner| {
                        device.bels()[candidate[0].0].point == target
                            && incidents[index]
                                .nets
                                .iter()
                                .any(|net| incidents[partner].nets.binary_search(net).is_ok())
                    }) {
                        return None;
                    }
                    detailed_candidate_is_legal(
                        graph,
                        constraints,
                        units,
                        &assignments,
                        index,
                        candidate,
                        partner,
                        &mut resource_usage,
                    )
                    .then_some(partner)
                };
                visit_spatial_choices_on_nearest_usable_ring(
                    spatial_index,
                    target,
                    true,
                    device,
                    &mut classify,
                    |candidate, partner| candidates.push((candidate, partner)),
                );
            }
            candidates.sort_unstable();
            candidates.dedup_by_key(|candidate| candidate.0);
            candidate_count = candidate_count.saturating_add(candidates.len());
            target_elapsed += target_started.elapsed();
            let scoring_started = std::time::Instant::now();
            let mut best = None::<(u128, usize, Option<usize>)>;
            for &(candidate_index, partner) in &candidates {
                let candidate = unit.choices.assignment(candidate_index);
                swap_count = swap_count.saturating_add(usize::from(partner.is_some()));
                let partner_incident = partner.map_or(&empty_incident, |p| &incidents[p]);
                merge_sorted_unique(
                    &incidents[index].nets,
                    &partner_incident.nets,
                    &mut affected_nets,
                );
                merge_sorted_unique(
                    &incidents[index].timing_arcs,
                    &partner_incident.timing_arcs,
                    &mut affected_arcs,
                );
                let first = (unit, candidate);
                let second = partner.map(|p| (&units[p], current.as_slice()));
                let Some((old, new)) = with_detailed_replacements(first, second, |replacements| {
                    costs.net_bbox.trial_deltas(
                        graph,
                        replacements,
                        placed,
                        &mut bbox_delta_workspace,
                    )?;
                    detailed_candidate_costs(
                        graph,
                        constraints,
                        replacements,
                        placed,
                        &affected_nets,
                        &bbox_delta_workspace,
                        &affected_arcs,
                        sink_criticalities,
                        delay_estimator,
                        &costs,
                    )
                }) else {
                    continue;
                };
                scored_count = scored_count.saturating_add(1);
                let Some(candidate_totals) =
                    detailed_totals_after_replacement(costs.totals, old, new)
                else {
                    continue;
                };
                let Some(candidate_objective) =
                    detailed_placement_objective(candidate_totals, normalizer)
                else {
                    continue;
                };
                let score = (candidate_objective, candidate_index, partner);
                if candidate_objective < objective && best.is_none_or(|known| score < known) {
                    best = Some(score);
                }
            }
            scoring_elapsed += scoring_started.elapsed();
            update_placement_resource_usage(
                graph,
                constraints,
                &unit.cells,
                &current,
                &mut resource_usage,
                true,
            );
            let Some((accepted_objective, candidate_index, partner)) = best else {
                continue;
            };
            let candidate = unit.choices.assignment(candidate_index).to_vec();
            let partner_current = partner.map(|p| assignments[p].clone());
            {
                let first = (unit, candidate.as_slice());
                let second = partner.map(|p| (&units[p], current.as_slice()));
                with_detailed_replacements(first, second, |replacements| {
                    costs.net_bbox.trial_deltas(
                        graph,
                        replacements,
                        placed,
                        &mut bbox_delta_workspace,
                    )
                })
                .expect("accepted legal move has a valid endpoint delta");
            }
            update_placement_resource_usage(
                graph,
                constraints,
                &unit.cells,
                &current,
                &mut resource_usage,
                false,
            );
            if let Some(partner) = partner {
                update_placement_resource_usage(
                    graph,
                    constraints,
                    &units[partner].cells,
                    partner_current
                        .as_ref()
                        .expect("swap partner has an assignment"),
                    &mut resource_usage,
                    false,
                );
            }
            for &bel in &current {
                bel_owner[bel.0] = None;
                occupied[bel.0] = false;
            }
            if let Some(old) = &partner_current {
                for &bel in old {
                    bel_owner[bel.0] = None;
                    occupied[bel.0] = false;
                }
            }
            assignments[index] = candidate;
            if let Some(partner) = partner {
                assignments[partner] = current;
            }
            for (&cell, &bel) in unit.cells.iter().zip(&assignments[index]) {
                placed[cell.0] = Some(bel);
                bel_owner[bel.0] = Some(index);
                occupied[bel.0] = true;
            }
            update_placement_resource_usage(
                graph,
                constraints,
                &unit.cells,
                &assignments[index],
                &mut resource_usage,
                true,
            );
            if let Some(partner) = partner {
                for (&cell, &bel) in units[partner].cells.iter().zip(&assignments[partner]) {
                    placed[cell.0] = Some(bel);
                    bel_owner[bel.0] = Some(partner);
                    occupied[bel.0] = true;
                }
                update_placement_resource_usage(
                    graph,
                    constraints,
                    &units[partner].cells,
                    &assignments[partner],
                    &mut resource_usage,
                    true,
                );
            }
            let partner_incident = partner.map_or(&empty_incident, |p| &incidents[p]);
            merge_sorted_unique(
                &incidents[index].nets,
                &partner_incident.nets,
                &mut affected_nets,
            );
            merge_sorted_unique(
                &incidents[index].timing_arcs,
                &partner_incident.timing_arcs,
                &mut affected_arcs,
            );
            update_detailed_cost_cache(
                graph,
                constraints,
                placed,
                &bbox_delta_workspace,
                &affected_arcs,
                sink_criticalities,
                delay_estimator,
                &mut costs,
            );
            let verified_objective = detailed_placement_objective(costs.totals, normalizer)
                .expect("physical placement objective fits u128");
            assert_eq!(
                verified_objective, accepted_objective,
                "accepted candidate scoring and incremental cache update use one exact objective"
            );
            assert!(
                verified_objective < objective,
                "every accepted detailed placement move strictly lowers its integer objective"
            );
            objective = verified_objective;
            moved += 1;
        }
        if std::env::var_os("TEXO_PNR_METRICS").is_some() {
            eprintln!(
                "[metrics] predicted_detail_pass pass={} moved={} targets={} candidates={} scored={} swaps={} bbox_hpwl={} timing_cost={} objective={} target_elapsed={:?} scoring_elapsed={:?} elapsed={:?}",
                pass,
                moved,
                target_count,
                candidate_count,
                scored_count,
                swap_count,
                costs.totals.0,
                costs.totals.1,
                objective,
                target_elapsed,
                scoring_elapsed,
                pass_started.elapsed(),
            );
        }
        total_moved = total_moved.saturating_add(moved);
        if moved == 0 || passes == PredictedPlacementPasses::One {
            break;
        }
    }
    total_moved
}

const MAX_PLACEMENT_REFINEMENT_PASSES: usize = 4;
const PLACEMENT_REFINEMENT_CANDIDATES: usize = 64;

#[allow(clippy::too_many_lines)]
#[allow(clippy::too_many_arguments)]
fn refine_placement(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    units: &[PlacementUnit],
    neighbors: &[Vec<PlacementNeighbor>],
    placed: &mut [Option<BelId>],
    occupied: &mut [bool],
    move_limit: Option<usize>,
    cached_spatial_indexes: Option<&BTreeMap<(u8, usize), Arc<SpatialChoiceIndex>>>,
) -> usize {
    let device = graph.device();
    let mut pin_usage = PlacementResourceUsage::default();
    for unit in units {
        let assignment = unit
            .cells
            .iter()
            .map(|cell| placed[cell.0].expect("initial placement is complete"))
            .collect::<Vec<_>>();
        update_placement_resource_usage(
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

    let mut move_peak = 0;
    let mut choice_workspace = RefinementChoiceWorkspace::default();
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
                occupied[bel.0] = false;
            }
            for &cell in &unit.cells {
                placed[cell.0] = None;
            }
            update_placement_resource_usage(
                graph,
                constraints,
                &unit.cells,
                &current,
                &mut pin_usage,
                false,
            );
            let current_is_legal = assignment_resources_are_legal(
                graph,
                constraints,
                &unit.cells,
                &current,
                &pin_usage,
            );
            let spatial_index = if let Some(cached) = cached_spatial_indexes {
                cached[&unit.choices.cache_key()].as_ref()
            } else {
                spatial_indexes
                    .entry(unit.choices.cache_key())
                    .or_insert_with(|| SpatialChoiceIndex::new(&unit.choices, device))
            };
            let Some(best) = choose_refined_assignment(
                unit,
                spatial_index,
                graph,
                constraints,
                neighbors,
                placed,
                occupied,
                &pin_usage,
                &mut choice_workspace,
            ) else {
                for (&cell, &bel) in unit.cells.iter().zip(&current) {
                    occupied[bel.0] = true;
                    placed[cell.0] = Some(bel);
                }
                update_placement_resource_usage(
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
                occupied[bel.0] = true;
                placed[cell.0] = Some(bel);
            }
            update_placement_resource_usage(
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
        move_peak = move_peak.max(moved);
        if moved == 0 || move_limit.is_some_and(|limit| moved >= limit) {
            break;
        }
    }
    move_peak
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
    occupied: &[bool],
    pin_usage: &PlacementResourceUsage,
    workspace: &mut RefinementChoiceWorkspace,
) -> Option<Vec<BelId>> {
    let device = graph.device();
    let target = refinement_target(unit, device, neighbors, placed)?;
    nearest_legal_assignments_impl(
        unit,
        spatial_index,
        graph,
        constraints,
        target,
        |bel| occupied[bel.0],
        pin_usage,
        false,
        &mut workspace.nearest,
        &mut workspace.pin_resources,
    );
    workspace
        .nearest
        .iter()
        .map(|&index| {
            let assignment = unit.choices.assignment(index);
            (
                assignment_wirelength(&unit.cells, assignment, device, neighbors, placed),
                index,
            )
        })
        .min()
        .map(|(_, index)| unit.choices.assignment(index).to_vec())
}

#[allow(clippy::too_many_arguments)]
fn nearest_legal_assignments_impl(
    unit: &PlacementUnit,
    spatial_index: &SpatialChoiceIndex,
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    target: Point,
    is_occupied: impl Fn(BelId) -> bool,
    pin_usage: &PlacementResourceUsage,
    nearest_ring_only: bool,
    nearest: &mut Vec<usize>,
    pin_resources: &mut Vec<(WireId, NetId)>,
) {
    let device = graph.device();
    nearest.clear();
    let max_radius = device.width() + device.height();
    for radius in 0..max_radius {
        for dy in 0..=radius {
            let dx = radius - dy;
            for y in ring_coordinates(target.y, dy, device.height()) {
                for x in ring_coordinates(target.x, dx, device.width()) {
                    let bucket = &spatial_index.by_point[(y * device.width() + x) as usize];
                    for &index in bucket {
                        let assignment = unit.choices.assignment(index);
                        if assignment.iter().all(|&bel| !is_occupied(bel))
                            && assignment_resources_are_legal_with_workspace(
                                graph,
                                constraints,
                                &unit.cells,
                                assignment,
                                pin_usage,
                                pin_resources,
                            )
                        {
                            nearest.push(index);
                        }
                    }
                }
            }
        }
        let enough = if nearest_ring_only {
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

#[derive(Default)]
struct PlacementResourceUsage {
    pin_wires: HashMap<WireId, HashMap<NetId, usize>>,
    shared: HashMap<(usize, u64), HashMap<u64, usize>>,
}

fn assignment_resources_are_legal(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    cells: &[CellId],
    assignment: &[BelId],
    usage: &PlacementResourceUsage,
) -> bool {
    let mut candidate = Vec::new();
    assignment_resources_are_legal_with_workspace(
        graph,
        constraints,
        cells,
        assignment,
        usage,
        &mut candidate,
    )
}

fn assignment_resources_are_legal_with_workspace(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    cells: &[CellId],
    assignment: &[BelId],
    usage: &PlacementResourceUsage,
    candidate: &mut Vec<(WireId, NetId)>,
) -> bool {
    candidate.clear();
    visit_assignment_pin_resources(graph, constraints, cells, assignment, |wire, net| {
        candidate.push((wire, net));
    });
    candidate.sort_unstable();
    candidate.dedup();
    let mut start = 0;
    while start < candidate.len() {
        let wire = candidate[start].0;
        let mut end = start + 1;
        while end < candidate.len() && candidate[end].0 == wire {
            end += 1;
        }
        let existing = usage.pin_wires.get(&wire);
        let new_nets = candidate[start..end]
            .iter()
            .filter(|(_, net)| existing.is_none_or(|nets| !nets.contains_key(net)))
            .count();
        let distinct = existing.map_or(0, HashMap::len) + new_nets;
        if distinct > usize::from(graph.device().wires()[wire.0].capacity) {
            return false;
        }
        start = end;
    }
    let mut shared = Vec::new();
    visit_assignment_shared_resources(constraints, cells, assignment, |rule, resource, value| {
        shared.push(((rule, resource), value));
    });
    shared.sort_unstable();
    shared.dedup();
    let mut start = 0;
    while start < shared.len() {
        let resource = shared[start].0;
        let mut end = start + 1;
        while end < shared.len() && shared[end].0 == resource {
            end += 1;
        }
        let existing = usage.shared.get(&resource);
        let new_values = shared[start..end]
            .iter()
            .filter(|(_, value)| existing.is_none_or(|values| !values.contains_key(value)))
            .count();
        if existing.map_or(0, HashMap::len) + new_values > 1 {
            return false;
        }
        start = end;
    }
    true
}

fn update_placement_resource_usage(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    cells: &[CellId],
    assignment: &[BelId],
    usage: &mut PlacementResourceUsage,
    add: bool,
) {
    visit_assignment_pin_resources(graph, constraints, cells, assignment, |wire, net| {
        if add {
            *usage
                .pin_wires
                .entry(wire)
                .or_default()
                .entry(net)
                .or_default() += 1;
        } else {
            let remove_wire = {
                let nets = usage
                    .pin_wires
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
                usage.pin_wires.remove(&wire);
            }
        }
    });
    visit_assignment_shared_resources(constraints, cells, assignment, |rule, resource, value| {
        let resource = (rule, resource);
        if add {
            *usage
                .shared
                .entry(resource)
                .or_default()
                .entry(value)
                .or_default() += 1;
        } else {
            let remove_resource = {
                let values = usage
                    .shared
                    .get_mut(&resource)
                    .expect("placed shared resource is present in usage");
                let count = values
                    .get_mut(&value)
                    .expect("placed shared value is present in usage");
                *count -= 1;
                if *count == 0 {
                    values.remove(&value);
                }
                values.is_empty()
            };
            if remove_resource {
                usage.shared.remove(&resource);
            }
        }
    });
}

fn visit_assignment_shared_resources(
    constraints: &PlacementConstraints,
    cells: &[CellId],
    assignment: &[BelId],
    mut visit: impl FnMut(usize, u64, u64),
) {
    for (rule_index, rule) in constraints.shared_resources.iter().enumerate() {
        for (&cell, &bel) in cells.iter().zip(assignment) {
            if let (Some(&value), Some(&resource)) =
                (rule.cell_values.get(&cell), rule.bel_resources.get(&bel))
            {
                visit(rule_index, resource, value);
            }
        }
    }
}

fn visit_assignment_pin_resources(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    cells: &[CellId],
    assignment: &[BelId],
    mut visit: impl FnMut(WireId, NetId),
) {
    for (&cell, &bel) in cells.iter().zip(assignment) {
        for &pin in graph.design().cells()[cell.0].pins() {
            let Some(net) = graph.design().pins()[pin.0].net() else {
                continue;
            };
            let wire = candidate_pin_wire(graph, constraints, pin, bel)
                .expect("placement candidate has every bound physical pin");
            visit(wire, net);
        }
    }
}

fn candidate_pin_wire(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    pin: CellPinId,
    bel: BelId,
) -> Option<WireId> {
    candidate_bel_pin(graph, constraints, pin, bel)
        .map(|bel_pin| graph.device().bel_pins()[bel_pin.0].wire)
}

fn candidate_bel_pin(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    pin: CellPinId,
    bel: BelId,
) -> Option<BelPinId> {
    if let Some(&bel_pin) = constraints.pin_bindings.get(&(pin, bel)) {
        return Some(bel_pin);
    }
    if let Some(name) = constraints.pin_name_bindings.get(&pin) {
        return physical_pin_by_name(graph, pin, bel, name);
    }
    graph.bound_bel_pin(pin, bel).ok()
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
    placement_units_cached(graph, constraints, candidate_cache, &mut Vec::new())
}

fn placement_units_cached(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
    candidate_cache: &mut BTreeMap<PlacementCandidateKey, Arc<[BelId]>>,
    validated_group_shapes: &mut Vec<ValidatedGroupShape>,
) -> Result<Vec<PlacementUnit>, PnrError> {
    validate_pin_bindings(graph, constraints)?;
    validate_shared_resources(graph, constraints)?;
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
        let shape_is_validated = validated_group_shapes.iter().any(|shape| {
            Arc::ptr_eq(&shape.assignments, &group.assignments)
                && shape.candidate_sets.len() == candidate_sets.len()
                && shape
                    .candidate_sets
                    .iter()
                    .zip(&candidate_sets)
                    .all(|(known, candidate)| Arc::ptr_eq(known, candidate))
        });
        if !shape_is_validated {
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
                            reason: format!(
                                "BEL ID {} is incompatible with cell ID {}",
                                bel.0, cell.0
                            ),
                        });
                    }
                }
            }
            validated_group_shapes.push(ValidatedGroupShape {
                assignments: Arc::clone(&group.assignments),
                candidate_sets: candidate_sets.clone(),
            });
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

fn validate_shared_resources(
    graph: &UnifiedGraph<'_>,
    constraints: &PlacementConstraints,
) -> Result<(), PnrError> {
    for (index, resource) in constraints.shared_resources.iter().enumerate() {
        let group = constraints.groups.len() + index;
        if resource.cell_values.is_empty() || resource.bel_resources.is_empty() {
            return Err(PnrError::InvalidPlacementConstraint {
                group,
                reason: "shared resource cell and BEL maps must be non-empty".into(),
            });
        }
        if let Some(cell) = resource
            .cell_values
            .keys()
            .find(|cell| cell.0 >= graph.design().cells().len())
        {
            return Err(PnrError::InvalidPlacementConstraint {
                group,
                reason: format!("shared resource names unknown cell ID {}", cell.0),
            });
        }
        if let Some(bel) = resource
            .bel_resources
            .keys()
            .find(|bel| bel.0 >= graph.device().bels().len())
        {
            return Err(PnrError::InvalidPlacementConstraint {
                group,
                reason: format!("shared resource names unknown BEL ID {}", bel.0),
            });
        }
    }
    Ok(())
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

fn validate_routing_restrictions(
    device: &Device,
    constraints: &RoutingConstraints,
) -> Result<(), PnrError> {
    if let Some(pip) = constraints
        .blocked_pips()
        .iter()
        .find(|pip| pip.0 >= device.pips().len())
    {
        return Err(PnrError::InvalidRoutingRestriction {
            reason: format!("blocked PIP ID {} is outside the device", pip.0),
        });
    }
    Ok(())
}

fn validate_routing_constraints(
    graph: &UnifiedGraph<'_>,
    placement: &Placement,
    pin_wires: &PinWireCache,
    constraints: &RoutingConstraints,
) -> Result<(), PnrError> {
    let design = graph.design();
    let device = graph.device();
    validate_routing_restrictions(device, constraints)?;
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
        let driver_wire = pin_wires.resolve(graph, placement, net.driver, driver_bel)?;
        let (wire_refs, pip_refs) = route_resource_refs(&route.arcs);
        if wire_refs != route.wire_refs || pip_refs != route.pip_refs {
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
                if constraints.blocked_pips().contains(&pip_id) {
                    return Err(PnrError::InvalidRoutingConstraint {
                        net: net_id,
                        reason: format!("immutable route uses blocked PIP {}", pip_id.0),
                    });
                }
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
                if arc.wires.last().copied()
                    != Some(pin_wires.resolve(graph, placement, sink, sink_bel)?)
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

#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn route(
    graph: &UnifiedGraph<'_>,
    placement: &Placement,
    pin_wires: &PinWireCache,
    constraints: &RoutingConstraints,
    costs: Option<&RoutingCosts>,
    workspace: &mut RoutingWorkspace,
    mut routes: Vec<Option<Arc<NetRoute>>>,
    progress: &mut impl FnMut(RoutingProgress),
) -> Result<Vec<Arc<NetRoute>>, PnrError> {
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
    let wire_congestion = &mut workspace.wire_congestion;
    let pip_congestion = &mut workspace.pip_congestion;
    let connection_owners = &mut workspace.connection_owners;
    let resource_owners = &mut workspace.resource_owners;
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
    let mut routing_order = routing_order(design, constraints, costs);
    routing_order.sort_unstable();
    let mut dirty = BTreeMap::<usize, BTreeSet<CellPinId>>::new();
    let mut cycle_priority = BTreeSet::<usize>::new();
    let mut conflict_cycles = RoutingConflictCycleDetector::default();
    for (index, (net, route)) in design.nets().iter().zip(&routes).enumerate() {
        let route = route.as_ref();
        for &sink in &net.sinks {
            if route.is_none_or(|route| route.arc(sink).is_none()) {
                dirty.entry(index).or_default().insert(sink);
            }
        }
    }
    resource_owners.prepare(&routes, metadata);
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
                    resource_owners.release_wire(
                        wire,
                        previous.net,
                        metadata.wire_capacities[wire.0],
                        wire_occupancy[wire.0],
                    );
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
                    resource_owners.release_pip(
                        pip,
                        previous.net,
                        metadata.pip_capacities[pip.0],
                        pip_occupancy[pip.0],
                    );
                    track_entry(
                        &mut overuse.pips,
                        pip_occupancy[pip.0],
                        metadata.pip_capacities[pip.0],
                        pip.0,
                    );
                }
                routes[index] = Some(Arc::new(preserved));
            }
        }
        resource_owners.repair_stale(&routes);
        for &index in &workspace.touched_wires {
            wire_congestion[index] = cached_congestion_cost(
                wire_occupancy[index],
                metadata.wire_capacities[index],
                wire_history[index],
                present_factor,
            );
        }
        for &index in &workspace.touched_pips {
            pip_congestion[index] = cached_congestion_cost(
                pip_occupancy[index],
                metadata.pip_capacities[index],
                pip_history[index],
                present_factor,
            );
        }
        let mut iteration_order = routing_order
            .iter()
            .copied()
            .filter(|&(_, index)| dirty.contains_key(&index))
            .collect::<Vec<_>>();
        if !cycle_priority.is_empty() {
            prioritize_cycle_connections(&mut iteration_order, &cycle_priority);
        }
        cycle_priority.clear();
        for (ordinal, (_, index)) in iteration_order.into_iter().enumerate() {
            progress(RoutingProgress::Net {
                iteration,
                ordinal: ordinal + 1,
                total: dirty.len(),
                net: NetId(index),
            });
            let net_id = NetId(index);
            let preserved = routes[index].take();
            let route = route_net(
                graph,
                placement,
                pin_wires,
                preserved.as_deref(),
                net_id,
                wire_congestion,
                pip_congestion,
                constraints.blocked_pip_words(),
                None,
                None,
                iteration == 0,
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
                wire_congestion[wire.0] = cached_congestion_cost(
                    wire_occupancy[wire.0],
                    metadata.wire_capacities[wire.0],
                    wire_history[wire.0],
                    present_factor,
                );
                track_entry(
                    &mut overuse.wires,
                    wire_occupancy[wire.0],
                    metadata.wire_capacities[wire.0],
                    wire.0,
                );
                resource_owners.claim_wire(wire, net_id, metadata.wire_capacities[wire.0]);
            }
            for pip in route.pips().filter(|&pip| {
                preserved
                    .as_ref()
                    .is_none_or(|old| old.pip_ref_count(pip) == 0)
            }) {
                increment_occupancy(pip_occupancy, &mut workspace.touched_pips, pip.0);
                pip_congestion[pip.0] = cached_congestion_cost(
                    pip_occupancy[pip.0],
                    metadata.pip_capacities[pip.0],
                    pip_history[pip.0],
                    present_factor,
                );
                track_entry(
                    &mut overuse.pips,
                    pip_occupancy[pip.0],
                    metadata.pip_capacities[pip.0],
                    pip.0,
                );
                resource_owners.claim_pip(pip, net_id, metadata.pip_capacities[pip.0]);
            }
            routes[index] = Some(Arc::new(route));
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
        let mut next_dirty = if overuse
            .wires
            .iter()
            .all(|&wire| metadata.wire_capacities[wire] == 1)
            && overuse
                .pips
                .iter()
                .all(|&pip| metadata.pip_capacities[pip] == 1)
        {
            congested_route_arcs_indexed(
                metadata,
                &routes,
                constraints,
                costs,
                resource_owners,
                connection_owners,
            )
        } else {
            congested_route_arcs(
                metadata,
                &routes,
                constraints,
                costs,
                wire_occupancy,
                pip_occupancy,
                connection_owners,
            )
        };
        let objective =
            routing_congestion_objective(&overuse, wire_occupancy, pip_occupancy, metadata);
        if let Some(cycle) =
            conflict_cycles.observe(objective, &overuse.wires, &overuse.pips, &next_dirty)
        {
            let connection_count = cycle.connections.values().map(BTreeSet::len).sum::<usize>();
            cycle_priority.extend(cycle.connections.keys().copied());
            for (net, sinks) in cycle.connections {
                next_dirty.entry(net).or_default().extend(sinks);
            }
            if std::env::var_os("TEXO_PNR_METRICS").is_some() {
                eprintln!(
                    "[metrics] routing_cycle_escape length={} nets={} connections={}",
                    cycle.length,
                    cycle_priority.len(),
                    connection_count,
                );
            }
        }
        dirty = next_dirty;
        resource_owners.resolve_conflicts(&routes, &dirty);
    }

    if std::env::var_os("TEXO_PNR_METRICS").is_some() {
        for &index in &overuse.wires {
            let wire = WireId(index);
            let owners = routes
                .iter()
                .flatten()
                .filter(|route| route.wire_ref_count(wire) != 0)
                .map(|route| {
                    let net = &design.nets()[route.net.0];
                    let driver_pin = &design.pins()[net.driver.0];
                    let driver_cell = &design.cells()[driver_pin.cell.0];
                    let driver_bel = placement.bel(driver_pin.cell).map_or("<unplaced>", |bel| {
                        graph.device().bels()[bel.0].name.as_str()
                    });
                    let affected_sinks = route
                        .arcs
                        .iter()
                        .filter(|arc| arc.wires.contains(&wire))
                        .filter_map(|arc| arc.sink)
                        .map(|sink| {
                            let pin = &design.pins()[sink.0];
                            let cell = &design.cells()[pin.cell.0];
                            let bel = placement.bel(pin.cell).map_or("<unplaced>", |bel| {
                                graph.device().bels()[bel.0].name.as_str()
                            });
                            format!("{}.{}/{}", cell.name, pin.name, bel)
                        })
                        .collect::<Vec<_>>()
                        .join("|");
                    format!(
                        "{}:{} driver={}.{}/{} sinks={}{}",
                        route.net.0,
                        net.name,
                        driver_cell.name,
                        driver_pin.name,
                        driver_bel,
                        affected_sinks,
                        if dirty.contains_key(&route.net.0) {
                            " (dirty)"
                        } else {
                            " (preserved)"
                        }
                    )
                })
                .collect::<Vec<_>>();
            eprintln!(
                "[metrics] unresolved_wire id={} name={} point={:?} occupancy={} capacity={} owners=[{}]",
                index,
                graph.device().wires()[index].name,
                graph.device().wires()[index].point,
                wire_occupancy[index],
                metadata.wire_capacities[index],
                owners.join(", ")
            );
        }
    }
    Err(PnrError::CongestionNotResolved {
        iterations: max_iterations,
        overused_wires: overuse.wires.len(),
        overused_pips: overuse.pips.len(),
    })
}

#[cfg(test)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct LegalRoutePolishMetrics {
    /// One event-queue run ending at a dependency fixed point.
    passes: usize,
    initial_candidates: usize,
    attempts: usize,
    wakeups: usize,
    improvements: usize,
    objective_reduction: u64,
}

#[cfg(test)]
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct LegalRoutePolishConnection {
    net: NetId,
    sink: CellPinId,
}

#[cfg(test)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct LegalRoutePolishCandidate {
    connection: LegalRoutePolishConnection,
    criticality: u64,
    realized_cost: u64,
}

#[cfg(test)]
type LegalRoutePolishQueueEntry = (Reverse<u64>, Reverse<u64>, NetId, CellPinId);

#[cfg(test)]
#[derive(Default)]
struct LegalRoutePolishSubscriptions {
    blockers: BTreeMap<LegalRoutePolishConnection, HardRoutingBlockers>,
    wire_subscribers: HashMap<WireId, BTreeSet<LegalRoutePolishConnection>>,
    pip_subscribers: HashMap<PipId, BTreeSet<LegalRoutePolishConnection>>,
}

#[cfg(test)]
impl LegalRoutePolishSubscriptions {
    fn replace(&mut self, connection: LegalRoutePolishConnection, blockers: HardRoutingBlockers) {
        if let Some(previous) = self.blockers.remove(&connection) {
            for wire in previous.wires {
                let remove_entry =
                    self.wire_subscribers
                        .get_mut(&wire)
                        .is_some_and(|subscribers| {
                            subscribers.remove(&connection);
                            subscribers.is_empty()
                        });
                if remove_entry {
                    self.wire_subscribers.remove(&wire);
                }
            }
            for pip in previous.pips {
                let remove_entry = self
                    .pip_subscribers
                    .get_mut(&pip)
                    .is_some_and(|subscribers| {
                        subscribers.remove(&connection);
                        subscribers.is_empty()
                    });
                if remove_entry {
                    self.pip_subscribers.remove(&pip);
                }
            }
        }
        for &wire in &blockers.wires {
            self.wire_subscribers
                .entry(wire)
                .or_default()
                .insert(connection);
        }
        for &pip in &blockers.pips {
            self.pip_subscribers
                .entry(pip)
                .or_default()
                .insert(connection);
        }
        self.blockers.insert(connection, blockers);
    }

    fn subscribers(
        &self,
        wires: impl IntoIterator<Item = WireId>,
        pips: impl IntoIterator<Item = PipId>,
    ) -> BTreeSet<LegalRoutePolishConnection> {
        let mut result = BTreeSet::new();
        for wire in wires {
            if let Some(subscribers) = self.wire_subscribers.get(&wire) {
                result.extend(subscribers);
            }
        }
        for pip in pips {
            if let Some(subscribers) = self.pip_subscribers.get(&pip) {
                result.extend(subscribers);
            }
        }
        result
    }
}

#[cfg(test)]
fn legal_route_polish_candidate(
    connection: LegalRoutePolishConnection,
    routes: &[Option<Arc<NetRoute>>],
    constraints: &RoutingConstraints,
    costs: &RoutingCosts,
) -> Option<LegalRoutePolishCandidate> {
    let route = routes.get(connection.net.0)?.as_deref()?;
    let arc = route.arc(connection.sink)?;
    let criticality = routing_arc_criticality(Some(costs), connection.net, connection.sink);
    (criticality > 1
        && constraints
            .routes()
            .get(&connection.net)
            .is_none_or(|locked| locked.arc(connection.sink).is_none()))
    .then(|| LegalRoutePolishCandidate {
        connection,
        criticality,
        realized_cost: unloaded_arc_cost(connection.net, arc, costs, criticality),
    })
}

#[cfg(test)]
fn enqueue_legal_route_polish_candidate(
    connection: LegalRoutePolishConnection,
    routes: &[Option<Arc<NetRoute>>],
    constraints: &RoutingConstraints,
    costs: &RoutingCosts,
    queue: &mut BTreeSet<LegalRoutePolishQueueEntry>,
    pending: &mut BTreeMap<LegalRoutePolishConnection, LegalRoutePolishQueueEntry>,
) -> bool {
    let previous = pending.remove(&connection);
    if let Some(previous) = previous {
        queue.remove(&previous);
    }
    let Some(candidate) = legal_route_polish_candidate(connection, routes, constraints, costs)
    else {
        return false;
    };
    let entry = (
        Reverse(candidate.realized_cost),
        Reverse(candidate.criticality),
        connection.net,
        connection.sink,
    );
    queue.insert(entry);
    pending.insert(connection, entry);
    previous.is_none()
}

#[cfg(test)]
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn polish_legal_timing_routes(
    graph: &UnifiedGraph<'_>,
    placement: &Placement,
    pin_wires: &PinWireCache,
    constraints: &RoutingConstraints,
    costs: &RoutingCosts,
    routes: &mut [Option<Arc<NetRoute>>],
    wire_occupancy: &mut [u16],
    pip_occupancy: &mut [u16],
    wire_congestion: &[u32],
    pip_congestion: &[u32],
    touched_wires: &mut Vec<usize>,
    touched_pips: &mut Vec<usize>,
    search: &mut RouteSearch,
    tree_arrival_ps: &mut [u64],
    metadata: RoutingResourceMetadata<'_>,
) -> LegalRoutePolishMetrics {
    let mut metrics = LegalRoutePolishMetrics {
        passes: 1,
        ..LegalRoutePolishMetrics::default()
    };
    let mut net_candidates = vec![Vec::new(); routes.len()];
    for route in routes.iter().flatten() {
        for arc in &route.arcs {
            let Some(sink) = arc.sink else {
                continue;
            };
            let connection = LegalRoutePolishConnection {
                net: route.net,
                sink,
            };
            if legal_route_polish_candidate(connection, routes, constraints, costs).is_some() {
                net_candidates[route.net.0].push(connection);
            }
        }
    }
    for candidates in &mut net_candidates {
        candidates.sort_unstable();
        candidates.dedup();
    }

    let mut queue = BTreeSet::new();
    let mut pending = BTreeMap::new();
    for &connection in net_candidates.iter().flatten() {
        enqueue_legal_route_polish_candidate(
            connection,
            routes,
            constraints,
            costs,
            &mut queue,
            &mut pending,
        );
    }
    metrics.initial_candidates = pending.len();
    let mut subscriptions = LegalRoutePolishSubscriptions::default();

    while let Some(entry) = queue.pop_first() {
        let connection = LegalRoutePolishConnection {
            net: entry.2,
            sink: entry.3,
        };
        pending.remove(&connection);
        let Some(candidate) = legal_route_polish_candidate(connection, routes, constraints, costs)
        else {
            continue;
        };
        metrics.attempts += 1;
        let index = connection.net.0;
        let old = routes[index]
            .as_ref()
            .expect("a legal route exists for every polish candidate")
            .clone();
        let Some(old_arc) = old.arc(connection.sink) else {
            continue;
        };
        let old_cost = unloaded_arc_cost(connection.net, old_arc, costs, candidate.criticality);
        let preserved = NetRoute::new(
            connection.net,
            old.arcs
                .iter()
                .filter(|arc| arc.sink != Some(connection.sink))
                .cloned()
                .collect(),
        );

        // Rip up only resources exclusive to this connection. Shared parts
        // of its net tree remain legal starts for the replacement. Remember
        // resources that transition from full so only their exact sleepers
        // need to be reconsidered if the replacement leaves them available.
        let mut released_full_wires = BTreeSet::new();
        for &wire in &old_arc.wires {
            if old.wire_ref_count(wire) == 1 {
                if wire_occupancy[wire.0] == metadata.wire_capacities[wire.0] {
                    released_full_wires.insert(wire);
                }
                wire_occupancy[wire.0] -= 1;
            }
        }
        let mut released_full_pips = BTreeSet::new();
        for &pip in &old_arc.pips {
            if old.pip_ref_count(pip) == 1 {
                if pip_occupancy[pip.0] == metadata.pip_capacities[pip.0] {
                    released_full_pips.insert(pip);
                }
                pip_occupancy[pip.0] -= 1;
            }
        }

        // A failed route can leave prefix arrivals in caller-owned scratch.
        // Clear the old tree both before and after the caught error so a later
        // event retry cannot inherit a phantom low-arrival source.
        for wire in old.wires() {
            tree_arrival_ps[wire.0] = UNROUTED_ARRIVAL_PS;
        }
        let mut blockers = HardRoutingBlockers::default();
        let replacement = route_net(
            graph,
            placement,
            pin_wires,
            Some(&preserved),
            connection.net,
            wire_congestion,
            pip_congestion,
            constraints.blocked_pip_words(),
            Some(HardRoutingOccupancy {
                wires: wire_occupancy,
                pips: pip_occupancy,
                use_estimate: false,
            }),
            Some(&mut blockers),
            false,
            Some(costs),
            search,
            tree_arrival_ps,
            metadata,
        );
        subscriptions.replace(connection, blockers);
        let Ok(replacement) = replacement else {
            for wire in old.wires() {
                tree_arrival_ps[wire.0] = UNROUTED_ARRIVAL_PS;
            }
            restore_released_arc_occupancy(old.as_ref(), old_arc, wire_occupancy, pip_occupancy);
            continue;
        };
        let replacement_arc = replacement
            .arc(connection.sink)
            .expect("polishing routes exactly the released sink");
        let replacement_cost = unloaded_arc_cost(
            connection.net,
            replacement_arc,
            costs,
            candidate.criticality,
        );
        if replacement_cost >= old_cost {
            restore_released_arc_occupancy(old.as_ref(), old_arc, wire_occupancy, pip_occupancy);
            continue;
        }

        // Every other connection was a hard obstacle during search. New
        // resources therefore fit without overuse; shared resources in the
        // preserved same-net tree were already counted once.
        for wire in replacement
            .wires()
            .filter(|&wire| preserved.wire_ref_count(wire) == 0)
        {
            debug_assert!(wire_occupancy[wire.0] < metadata.wire_capacities[wire.0]);
            increment_occupancy(wire_occupancy, touched_wires, wire.0);
        }
        for pip in replacement
            .pips()
            .filter(|&pip| preserved.pip_ref_count(pip) == 0)
        {
            debug_assert!(pip_occupancy[pip.0] < metadata.pip_capacities[pip.0]);
            increment_occupancy(pip_occupancy, touched_pips, pip.0);
        }
        debug_assert!(replacement_cost < old_cost);
        routes[index] = Some(Arc::new(replacement));
        metrics.improvements += 1;
        metrics.objective_reduction = metrics
            .objective_reduction
            .saturating_add(old_cost - replacement_cost);

        let available_wires = released_full_wires
            .into_iter()
            .filter(|wire| wire_occupancy[wire.0] < metadata.wire_capacities[wire.0]);
        let available_pips = released_full_pips
            .into_iter()
            .filter(|pip| pip_occupancy[pip.0] < metadata.pip_capacities[pip.0]);
        for sleeper in subscriptions.subscribers(available_wires, available_pips) {
            if enqueue_legal_route_polish_candidate(
                sleeper,
                routes,
                constraints,
                costs,
                &mut queue,
                &mut pending,
            ) {
                metrics.wakeups += 1;
            }
        }
        // A new branch changes the set of legal zero-conflict starts for all
        // siblings on this net even when no foreign resource was released.
        for &sibling in &net_candidates[index] {
            if sibling != connection
                && enqueue_legal_route_polish_candidate(
                    sibling,
                    routes,
                    constraints,
                    costs,
                    &mut queue,
                    &mut pending,
                )
            {
                metrics.wakeups += 1;
            }
        }
    }

    if std::env::var_os("TEXO_PNR_METRICS").is_some() {
        eprintln!(
            "[metrics] legal_route_polish_events initial_candidates={} attempts={} wakeups={} improvements={} objective_reduction={}",
            metrics.initial_candidates,
            metrics.attempts,
            metrics.wakeups,
            metrics.improvements,
            metrics.objective_reduction,
        );
    }
    metrics
}

#[cfg(test)]
fn restore_released_arc_occupancy(
    route: &NetRoute,
    arc: &RouteArc,
    wire_occupancy: &mut [u16],
    pip_occupancy: &mut [u16],
) {
    for &wire in &arc.wires {
        if route.wire_ref_count(wire) == 1 {
            wire_occupancy[wire.0] += 1;
        }
    }
    for &pip in &arc.pips {
        if route.pip_ref_count(pip) == 1 {
            pip_occupancy[pip.0] += 1;
        }
    }
}

/// Builds one connection-local, conflict-free route candidate.
///
/// The incumbent result is never mutated. Every route except `connection` is
/// installed as hard occupancy; same-net sibling arcs remain the replacement
/// tree, and only resources exclusively owned by the selected arc are
/// released. The caller is responsible for running full STA and committing
/// the returned [`PnrResult`] only when its whole-design objective improves.
///
/// `workspace` retains device-sized allocations across disposable ECO trials.
/// Its negotiated-congestion history is reset for every candidate, so a
/// rejected search cannot bias a later connection.
///
/// # Errors
///
/// Returns an invalid-model, route, cost, placement, or restriction error.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
pub fn legal_route_eco_candidate_with_workspace(
    design: &Design,
    device: &Device,
    incumbent: &PnrResult,
    routing_constraints: &RoutingConstraints,
    routing_costs: &RoutingCosts,
    connection: LegalRouteEcoConnection,
    options: LegalRouteEcoOptions,
    workspace: &mut RoutingWorkspace,
) -> Result<Option<PnrResult>, PnrError> {
    if options.estimate_delay_per_tile_ps == 0 {
        return Err(PnrError::InvalidRoutingCosts {
            reason: "legal route ECO estimate must be positive".into(),
        });
    }
    if incumbent.routes.len() != design.nets().len() {
        return Err(PnrError::InvalidRoutingConstraint {
            net: connection.net,
            reason: format!(
                "incumbent contains {} route trees for {} logical nets",
                incumbent.routes.len(),
                design.nets().len(),
            ),
        });
    }
    let Some(net) = design.nets().get(connection.net.0) else {
        return Err(PnrError::InvalidRoutingConstraint {
            net: connection.net,
            reason: "legal route ECO net ID is outside the design".into(),
        });
    };
    if !net.sinks.contains(&connection.sink) {
        return Err(PnrError::InvalidRoutingConstraint {
            net: connection.net,
            reason: format!(
                "legal route ECO sink {} is not on the selected net",
                connection.sink.0
            ),
        });
    }
    if routing_constraints
        .routes()
        .get(&connection.net)
        .is_some_and(|locked| locked.arc(connection.sink).is_some())
    {
        return Ok(None);
    }

    let graph = UnifiedGraph::new(design, device);
    let pin_wires = PinWireCache::build(&graph, &incumbent.placement);
    let mut complete_constraints = routing_constraints.clone();
    for (index, route) in incumbent.routes.iter().enumerate() {
        if route.net != NetId(index) {
            return Err(PnrError::InvalidRoutingConstraint {
                net: route.net,
                reason: format!(
                    "incumbent route tree {} is stored at net index {index}",
                    route.net.0
                ),
            });
        }
        complete_constraints.add_route(route.clone());
    }
    validate_routing_constraints(
        &graph,
        &incumbent.placement,
        &pin_wires,
        &complete_constraints,
    )?;
    validate_routing_costs(&graph, Some(routing_costs))?;

    workspace.prepare(device);
    for route in &incumbent.routes {
        add_route_occupancy(workspace, route);
    }
    for &index in &workspace.touched_wires {
        let occupancy = workspace.wire_occupancy[index];
        let capacity = workspace.wire_capacities[index];
        if occupancy > capacity {
            return Err(PnrError::InvalidRoutingConstraint {
                net: connection.net,
                reason: format!("incumbent overuses wire {index}: {occupancy}/{capacity}"),
            });
        }
    }
    for &index in &workspace.touched_pips {
        let occupancy = workspace.pip_occupancy[index];
        let capacity = workspace.pip_capacities[index];
        if occupancy > capacity {
            return Err(PnrError::InvalidRoutingConstraint {
                net: connection.net,
                reason: format!("incumbent overuses PIP {index}: {occupancy}/{capacity}"),
            });
        }
    }

    let old = incumbent.routes[connection.net.0].clone();
    let old_arc = old
        .arc(connection.sink)
        .ok_or_else(|| PnrError::InvalidRoutingConstraint {
            net: connection.net,
            reason: format!("incumbent route omits selected sink {}", connection.sink.0),
        })?;
    let preserved = NetRoute::new(
        connection.net,
        old.arcs
            .iter()
            .filter(|arc| arc.sink != Some(connection.sink))
            .cloned()
            .collect(),
    );
    for &wire in &old_arc.wires {
        if old.wire_ref_count(wire) == 1 {
            workspace.wire_occupancy[wire.0] -= 1;
        }
    }
    for &pip in &old_arc.pips {
        if old.pip_ref_count(pip) == 1 {
            workspace.pip_occupancy[pip.0] -= 1;
        }
    }
    for wire in old.wires() {
        workspace.tree_arrival_ps[wire.0] = UNROUTED_ARRIVAL_PS;
    }

    let previous_base_estimate = workspace.search.estimate_base_delay_ps;
    let previous_estimate = workspace.search.estimate_delay_per_tile_ps;
    workspace.search.estimate_base_delay_ps = ROUTING_ESTIMATE_BASE_DELAY_PS;
    workspace.search.estimate_delay_per_tile_ps = options.estimate_delay_per_tile_ps;
    let metadata = RoutingResourceMetadata {
        wire_points: &workspace.wire_points,
        wire_capacities: &workspace.wire_capacities,
        pip_capacities: &workspace.pip_capacities,
    };
    let replacement = route_net(
        &graph,
        &incumbent.placement,
        &pin_wires,
        Some(&preserved),
        connection.net,
        &workspace.wire_congestion,
        &workspace.pip_congestion,
        routing_constraints.blocked_pip_words(),
        Some(HardRoutingOccupancy {
            wires: &workspace.wire_occupancy,
            pips: &workspace.pip_occupancy,
            use_estimate: true,
        }),
        None,
        false,
        Some(routing_costs),
        &mut workspace.search,
        &mut workspace.tree_arrival_ps,
        metadata,
    );
    workspace.search.estimate_base_delay_ps = previous_base_estimate;
    workspace.search.estimate_delay_per_tile_ps = previous_estimate;
    for wire in old.wires() {
        workspace.tree_arrival_ps[wire.0] = UNROUTED_ARRIVAL_PS;
    }
    let replacement = match replacement {
        Ok(replacement) => replacement,
        Err(PnrError::Unroutable { .. }) => return Ok(None),
        Err(error) => return Err(error),
    };
    if replacement == *old {
        return Ok(None);
    }

    let mut routes = incumbent.routes.clone();
    routes[connection.net.0] = Arc::new(replacement);
    debug_assert!(
        routes
            .iter()
            .enumerate()
            .all(|(index, route)| index == connection.net.0
                || Arc::ptr_eq(route, &incumbent.routes[index]))
    );
    let total_pips = routes.iter().map(|route| route.pips().len()).sum();
    Ok(Some(PnrResult {
        placement: incumbent.placement.clone(),
        routes,
        total_pips,
    }))
}

/// Builds one conflict-free route candidate for a cohort of whole nets.
///
/// All movable occupancy of every selected net is released simultaneously.
/// The nets are then rebuilt in descending maximum sink criticality, with
/// stable net IDs breaking ties. Every unselected net and each already rebuilt
/// cohort member remains hard occupancy. Target-owned immutable topology is
/// retained as each net's fixed seed tree, so global-clock and other
/// architecture routes cannot be displaced by this ECO.
///
/// `net_ids` must contain at least one net. Duplicate IDs are accepted and
/// treated as one cohort member. Candidate construction is transactional:
/// `incumbent` is immutable and the reusable workspace is restored to its
/// incumbent occupancy before every return. The caller is responsible for
/// whole-design timing analysis and may commit the returned [`PnrResult`] only
/// when its exact objective strictly improves.
///
/// # Errors
///
/// Returns an invalid-model, route, cost, placement, or restriction error.
#[allow(clippy::too_many_arguments)]
pub fn legal_nets_route_eco_candidate_with_workspace(
    design: &Design,
    device: &Device,
    incumbent: &PnrResult,
    routing_constraints: &RoutingConstraints,
    routing_costs: &RoutingCosts,
    net_ids: &[NetId],
    options: LegalRouteEcoOptions,
    workspace: &mut RoutingWorkspace,
) -> Result<Option<PnrResult>, PnrError> {
    let mut workspace_staged = false;
    let result = legal_nets_route_eco_candidate(
        design,
        device,
        incumbent,
        routing_constraints,
        routing_costs,
        net_ids,
        options,
        workspace,
        &mut workspace_staged,
    );
    if workspace_staged {
        restore_legal_eco_workspace(device, incumbent, workspace);
    }
    result
}

#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn legal_nets_route_eco_candidate(
    design: &Design,
    device: &Device,
    incumbent: &PnrResult,
    routing_constraints: &RoutingConstraints,
    routing_costs: &RoutingCosts,
    net_ids: &[NetId],
    options: LegalRouteEcoOptions,
    workspace: &mut RoutingWorkspace,
    workspace_staged: &mut bool,
) -> Result<Option<PnrResult>, PnrError> {
    if options.estimate_delay_per_tile_ps == 0 {
        return Err(PnrError::InvalidRoutingCosts {
            reason: "legal nets route ECO estimate must be positive".into(),
        });
    }
    let selected = net_ids.iter().copied().collect::<BTreeSet<_>>();
    let Some(&first_net) = selected.first() else {
        return Err(PnrError::InvalidRoutingConstraint {
            net: NetId(0),
            reason: "legal nets route ECO cohort is empty".into(),
        });
    };
    if incumbent.routes.len() != design.nets().len() {
        return Err(PnrError::InvalidRoutingConstraint {
            net: first_net,
            reason: format!(
                "incumbent contains {} route trees for {} logical nets",
                incumbent.routes.len(),
                design.nets().len(),
            ),
        });
    }
    for &net_id in &selected {
        if design.nets().get(net_id.0).is_none() {
            return Err(PnrError::InvalidRoutingConstraint {
                net: net_id,
                reason: "legal nets route ECO net ID is outside the design".into(),
            });
        }
    }

    let graph = UnifiedGraph::new(design, device);
    let pin_wires = PinWireCache::build(&graph, &incumbent.placement);
    validate_routing_constraints(
        &graph,
        &incumbent.placement,
        &pin_wires,
        routing_constraints,
    )?;
    let mut complete_constraints = routing_constraints.clone();
    for (index, route) in incumbent.routes.iter().enumerate() {
        if route.net != NetId(index) {
            return Err(PnrError::InvalidRoutingConstraint {
                net: route.net,
                reason: format!(
                    "incumbent route tree {} is stored at net index {index}",
                    route.net.0
                ),
            });
        }
        complete_constraints.add_route(route.clone());
    }
    validate_routing_constraints(
        &graph,
        &incumbent.placement,
        &pin_wires,
        &complete_constraints,
    )?;
    validate_routing_costs(&graph, Some(routing_costs))?;

    for &net_id in &selected {
        let old = &incumbent.routes[net_id.0];
        let fixed = routing_constraints.routes().get(&net_id);
        if fixed.is_some_and(|fixed| {
            fixed
                .arcs
                .iter()
                .any(|arc| !old.arcs.iter().any(|incumbent_arc| incumbent_arc == arc))
        }) {
            return Err(PnrError::InvalidRoutingConstraint {
                net: net_id,
                reason: "incumbent route does not contain the immutable target tree".into(),
            });
        }
    }

    // From here onward every route has been structurally validated against the
    // device, so rebuilding incumbent occupancy is safe on every exit. Earlier
    // argument errors deliberately preserve the caller's prior workspace.
    *workspace_staged = true;
    workspace.prepare(device);
    for route in &incumbent.routes {
        add_route_occupancy(workspace, route);
    }
    validate_legal_eco_capacity(workspace, first_net)?;

    // Release the complete cohort before rebuilding any member. Immutable
    // target-owned resources stay occupied and seed their corresponding tree.
    for &net_id in &selected {
        let old = &incumbent.routes[net_id.0];
        let fixed = routing_constraints.routes().get(&net_id);
        remove_route_occupancy(workspace, old, fixed.map(Arc::as_ref));
        if let Some(fixed) = fixed {
            add_route_occupancy_delta(workspace, fixed, Some(old));
        }
    }
    workspace.tree_arrival_ps.fill(UNROUTED_ARRIVAL_PS);

    let mut route_order = selected.iter().copied().collect::<Vec<_>>();
    route_order.sort_by_key(|&net_id| {
        let maximum_sink_criticality = design.nets()[net_id.0]
            .sinks
            .iter()
            .map(|&sink| routing_arc_criticality(Some(routing_costs), net_id, sink))
            .max()
            .unwrap_or_else(|| routing_criticality(Some(routing_costs), net_id));
        (Reverse(maximum_sink_criticality), net_id)
    });

    let previous_base_estimate = workspace.search.estimate_base_delay_ps;
    let previous_estimate = workspace.search.estimate_delay_per_tile_ps;
    workspace.search.estimate_base_delay_ps = ROUTING_ESTIMATE_BASE_DELAY_PS;
    workspace.search.estimate_delay_per_tile_ps = options.estimate_delay_per_tile_ps;
    let mut routes = incumbent.routes.clone();
    let mut changed = false;
    let rebuild_result = (|| {
        for net_id in route_order {
            let old = &incumbent.routes[net_id.0];
            let fixed = routing_constraints.routes().get(&net_id);
            let metadata = RoutingResourceMetadata {
                wire_points: &workspace.wire_points,
                wire_capacities: &workspace.wire_capacities,
                pip_capacities: &workspace.pip_capacities,
            };
            let replacement = match route_net(
                &graph,
                &incumbent.placement,
                &pin_wires,
                fixed.map(Arc::as_ref),
                net_id,
                &workspace.wire_congestion,
                &workspace.pip_congestion,
                routing_constraints.blocked_pip_words(),
                Some(HardRoutingOccupancy {
                    wires: &workspace.wire_occupancy,
                    pips: &workspace.pip_occupancy,
                    use_estimate: true,
                }),
                None,
                false,
                Some(routing_costs),
                &mut workspace.search,
                &mut workspace.tree_arrival_ps,
                metadata,
            ) {
                Ok(replacement) => replacement,
                Err(PnrError::Unroutable { .. }) => return Ok(false),
                Err(error) => return Err(error),
            };
            workspace.tree_arrival_ps.fill(UNROUTED_ARRIVAL_PS);

            if fixed.is_some_and(|fixed| {
                fixed
                    .arcs
                    .iter()
                    .any(|arc| !replacement.arcs.iter().any(|candidate| candidate == arc))
            }) {
                return Err(PnrError::InvalidRoutingConstraint {
                    net: net_id,
                    reason: "whole-net ECO changed immutable target topology".into(),
                });
            }
            let reaches_all_sinks =
                route_reaches_all_sinks(&graph, &incumbent.placement, &pin_wires, &replacement)?;
            // A connected driver-rooted tree has exactly one fewer unique PIP
            // than unique wire. This is the same completed-route invariant
            // enforced by the full negotiated router.
            if replacement.pips().len().saturating_add(1) != replacement.wires().len()
                || !reaches_all_sinks
            {
                return Err(PnrError::InvalidRoutingConstraint {
                    net: net_id,
                    reason: "whole-net ECO replacement is not one complete driver-rooted tree"
                        .into(),
                });
            }

            add_route_occupancy_delta(workspace, &replacement, fixed.map(Arc::as_ref));
            validate_legal_eco_capacity(workspace, net_id)?;
            if replacement != **old {
                changed = true;
                routes[net_id.0] = Arc::new(replacement);
            }
        }
        Ok(true)
    })();
    workspace.search.estimate_base_delay_ps = previous_base_estimate;
    workspace.search.estimate_delay_per_tile_ps = previous_estimate;
    workspace.tree_arrival_ps.fill(UNROUTED_ARRIVAL_PS);
    if !rebuild_result? || !changed {
        return Ok(None);
    }

    let mut candidate_constraints = routing_constraints.clone();
    for route in &routes {
        candidate_constraints.add_route(route.clone());
    }
    validate_routing_constraints(
        &graph,
        &incumbent.placement,
        &pin_wires,
        &candidate_constraints,
    )?;

    debug_assert!(
        routes
            .iter()
            .enumerate()
            .all(|(index, route)| selected.contains(&NetId(index))
                || Arc::ptr_eq(route, &incumbent.routes[index]))
    );
    let total_pips = routes.iter().map(|route| route.pips().len()).sum();
    Ok(Some(PnrResult {
        placement: incumbent.placement.clone(),
        routes,
        total_pips,
    }))
}

/// Builds one whole-net, conflict-free route candidate.
///
/// This is the single-net compatibility wrapper around
/// [`legal_nets_route_eco_candidate_with_workspace`].
///
/// # Errors
///
/// Returns an invalid-model, route, cost, placement, or restriction error.
#[allow(clippy::too_many_arguments)]
pub fn legal_net_route_eco_candidate_with_workspace(
    design: &Design,
    device: &Device,
    incumbent: &PnrResult,
    routing_constraints: &RoutingConstraints,
    routing_costs: &RoutingCosts,
    net_id: NetId,
    options: LegalRouteEcoOptions,
    workspace: &mut RoutingWorkspace,
) -> Result<Option<PnrResult>, PnrError> {
    legal_nets_route_eco_candidate_with_workspace(
        design,
        device,
        incumbent,
        routing_constraints,
        routing_costs,
        &[net_id],
        options,
        workspace,
    )
}

fn validate_legal_eco_capacity(workspace: &RoutingWorkspace, net: NetId) -> Result<(), PnrError> {
    for &index in &workspace.touched_wires {
        let occupancy = workspace.wire_occupancy[index];
        let capacity = workspace.wire_capacities[index];
        if occupancy > capacity {
            return Err(PnrError::InvalidRoutingConstraint {
                net,
                reason: format!("route ECO overuses wire {index}: {occupancy}/{capacity}"),
            });
        }
    }
    for &index in &workspace.touched_pips {
        let occupancy = workspace.pip_occupancy[index];
        let capacity = workspace.pip_capacities[index];
        if occupancy > capacity {
            return Err(PnrError::InvalidRoutingConstraint {
                net,
                reason: format!("route ECO overuses PIP {index}: {occupancy}/{capacity}"),
            });
        }
    }
    Ok(())
}

fn restore_legal_eco_workspace(
    device: &Device,
    incumbent: &PnrResult,
    workspace: &mut RoutingWorkspace,
) {
    workspace.prepare(device);
    for route in &incumbent.routes {
        add_route_occupancy(workspace, route);
    }
    workspace.tree_arrival_ps.fill(UNROUTED_ARRIVAL_PS);
    workspace.commit_routes(&incumbent.routes);
}

#[cfg(test)]
fn unloaded_arc_cost(net: NetId, arc: &RouteArc, costs: &RoutingCosts, criticality: u64) -> u64 {
    let delay_quantum_ps = if costs.detailed_timing_nets.contains(&net) {
        costs.detailed_delay_quantum_ps
    } else {
        ROUTING_DELAY_QUANTUM_PS
    };
    let arrival_ps = arc.pips.iter().fold(0_u64, |arrival, pip| {
        arrival.saturating_add(u64::from(costs.pip_delays_ps[pip.0]))
    });
    let hop_bias = (ROUTING_CRITICALITY_SCALE - criticality)
        .saturating_mul(ROUTING_DELAY_QUANTUM_PS)
        .div_ceil(ROUTING_CRITICALITY_SCALE * delay_quantum_ps)
        .saturating_mul(arc.pips.len().try_into().unwrap_or(u64::MAX));
    timing_tree_cost(arrival_ps, criticality, delay_quantum_ps).saturating_add(hop_bias)
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct RoutingConflictState {
    wires: Vec<usize>,
    pips: Vec<usize>,
    dirty: Vec<(usize, Vec<CellPinId>)>,
}

impl RoutingConflictState {
    fn new(
        wires: &BTreeSet<usize>,
        pips: &BTreeSet<usize>,
        dirty: &BTreeMap<usize, BTreeSet<CellPinId>>,
    ) -> Self {
        Self {
            wires: wires.iter().copied().collect(),
            pips: pips.iter().copied().collect(),
            dirty: dirty
                .iter()
                .map(|(&net, sinks)| (net, sinks.iter().copied().collect()))
                .collect(),
        }
    }
}

#[derive(Debug, Eq, PartialEq)]
struct RoutingCycleEscape {
    length: usize,
    connections: BTreeMap<usize, BTreeSet<CellPinId>>,
}

/// Detects deterministic Pathfinder cycles without an arbitrary stall count.
///
/// A strict reduction in total excess (then conflicting-resource count) starts
/// a new epoch.  Re-entering an exact route-conflict state within an epoch
/// proves a cycle even when the victims rotate between iterations.  The
/// returned component contains every movable connection observed in that
/// cycle, so the caller can release and reorder the whole component once.
#[derive(Debug, Default)]
struct RoutingConflictCycleDetector {
    best_objective: Option<(u64, usize)>,
    states: BTreeMap<RoutingConflictState, usize>,
    dirty_history: Vec<BTreeMap<usize, BTreeSet<CellPinId>>>,
}

impl RoutingConflictCycleDetector {
    fn observe(
        &mut self,
        objective: (u64, usize),
        wires: &BTreeSet<usize>,
        pips: &BTreeSet<usize>,
        dirty: &BTreeMap<usize, BTreeSet<CellPinId>>,
    ) -> Option<RoutingCycleEscape> {
        if self.best_objective.is_none_or(|best| objective < best) {
            self.best_objective = Some(objective);
            self.states.clear();
            self.dirty_history.clear();
        }
        let state = RoutingConflictState::new(wires, pips, dirty);
        if let Some(&start) = self.states.get(&state) {
            let mut connections = BTreeMap::<usize, BTreeSet<CellPinId>>::new();
            for observed in &self.dirty_history[start..] {
                for (&net, sinks) in observed {
                    connections
                        .entry(net)
                        .or_default()
                        .extend(sinks.iter().copied());
                }
            }
            for (&net, sinks) in dirty {
                connections
                    .entry(net)
                    .or_default()
                    .extend(sinks.iter().copied());
            }
            let length = self.dirty_history.len() - start;
            self.states.clear();
            self.dirty_history.clear();
            return Some(RoutingCycleEscape {
                length,
                connections,
            });
        }
        self.states.insert(state, self.dirty_history.len());
        self.dirty_history.push(dirty.clone());
        None
    }
}

fn routing_congestion_objective(
    overuse: &OveruseTracker,
    wire_occupancy: &[u16],
    pip_occupancy: &[u16],
    metadata: RoutingResourceMetadata<'_>,
) -> (u64, usize) {
    let total_excess = overuse
        .wires
        .iter()
        .map(|&wire| u64::from(wire_occupancy[wire] - metadata.wire_capacities[wire]))
        .chain(
            overuse
                .pips
                .iter()
                .map(|&pip| u64::from(pip_occupancy[pip] - metadata.pip_capacities[pip])),
        )
        .sum();
    (total_excess, overuse.wires.len() + overuse.pips.len())
}

fn prioritize_cycle_connections(order: &mut [RoutingOrderEntry], component: &BTreeSet<usize>) {
    order.sort_unstable_by_key(|&(key, index)| (!component.contains(&index), key));
    let component_len = order.partition_point(|&(_, index)| component.contains(&index));
    order[..component_len].reverse();
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

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct ConnectionOwner {
    resource: usize,
    net: NetId,
    sink: Option<CellPinId>,
}

#[derive(Debug, Default)]
struct ConnectionOwnerScratch {
    wires: Vec<ConnectionOwner>,
    pips: Vec<ConnectionOwner>,
    ranked: Vec<RankedConnectionOwner>,
}

#[derive(Debug, Default)]
struct ResourceOwnerIndex {
    wire_owners: HashMap<WireId, NetId>,
    pip_owners: HashMap<PipId, NetId>,
    stale_wires: Vec<WireId>,
    stale_pips: Vec<PipId>,
    wire_conflicts: Vec<(WireId, NetId)>,
    pip_conflicts: Vec<(PipId, NetId)>,
}

impl ResourceOwnerIndex {
    fn prepare(&mut self, routes: &[Option<Arc<NetRoute>>], metadata: RoutingResourceMetadata<'_>) {
        self.wire_owners.clear();
        self.pip_owners.clear();
        self.stale_wires.clear();
        self.stale_pips.clear();
        self.wire_conflicts.clear();
        self.pip_conflicts.clear();
        for route in routes.iter().flatten() {
            for wire in route.wires() {
                self.claim_wire(wire, route.net, metadata.wire_capacities[wire.0]);
            }
            for pip in route.pips() {
                self.claim_pip(pip, route.net, metadata.pip_capacities[pip.0]);
            }
        }
    }

    fn claim_wire(&mut self, wire: WireId, net: NetId, capacity: u16) {
        if capacity != 1 {
            return;
        }
        if let Some(&owner) = self.wire_owners.get(&wire) {
            if owner == net {
                return;
            }
            self.wire_conflicts.push((wire, owner));
            self.wire_conflicts.push((wire, net));
        } else {
            self.wire_owners.insert(wire, net);
        }
    }

    fn claim_pip(&mut self, pip: PipId, net: NetId, capacity: u16) {
        if capacity != 1 {
            return;
        }
        if let Some(&owner) = self.pip_owners.get(&pip) {
            if owner == net {
                return;
            }
            self.pip_conflicts.push((pip, owner));
            self.pip_conflicts.push((pip, net));
        } else {
            self.pip_owners.insert(pip, net);
        }
    }

    fn release_wire(&mut self, wire: WireId, net: NetId, capacity: u16, occupancy: u16) {
        if capacity == 1 && self.wire_owners.get(&wire) == Some(&net) {
            self.wire_owners.remove(&wire);
            if occupancy != 0 {
                self.stale_wires.push(wire);
            }
        }
    }

    fn release_pip(&mut self, pip: PipId, net: NetId, capacity: u16, occupancy: u16) {
        if capacity == 1 && self.pip_owners.get(&pip) == Some(&net) {
            self.pip_owners.remove(&pip);
            if occupancy != 0 {
                self.stale_pips.push(pip);
            }
        }
    }

    fn repair_stale(&mut self, routes: &[Option<Arc<NetRoute>>]) {
        self.stale_wires.sort_unstable();
        self.stale_wires.dedup();
        for wire in self.stale_wires.drain(..) {
            if let Some(net) = routes
                .iter()
                .flatten()
                .find(|route| route.wire_ref_count(wire) != 0)
                .map(|route| route.net)
            {
                self.wire_owners.insert(wire, net);
            }
        }
        self.stale_pips.sort_unstable();
        self.stale_pips.dedup();
        for pip in self.stale_pips.drain(..) {
            if let Some(net) = routes
                .iter()
                .flatten()
                .find(|route| route.pip_ref_count(pip) != 0)
                .map(|route| route.net)
            {
                self.pip_owners.insert(pip, net);
            }
        }
    }

    fn resolve_conflicts(
        &mut self,
        routes: &[Option<Arc<NetRoute>>],
        dirty: &BTreeMap<usize, BTreeSet<CellPinId>>,
    ) {
        self.wire_conflicts.sort_unstable();
        self.wire_conflicts.dedup();
        let mut start = 0;
        while start < self.wire_conflicts.len() {
            let wire = self.wire_conflicts[start].0;
            let end = self.wire_conflicts[start..].partition_point(|entry| entry.0 == wire) + start;
            if let Some(net) =
                surviving_wire_owner(&self.wire_conflicts[start..end], routes, dirty, wire)
            {
                self.wire_owners.insert(wire, net);
            } else {
                self.wire_owners.remove(&wire);
            }
            start = end;
        }
        self.wire_conflicts.clear();

        self.pip_conflicts.sort_unstable();
        self.pip_conflicts.dedup();
        let mut start = 0;
        while start < self.pip_conflicts.len() {
            let pip = self.pip_conflicts[start].0;
            let end = self.pip_conflicts[start..].partition_point(|entry| entry.0 == pip) + start;
            if let Some(net) =
                surviving_pip_owner(&self.pip_conflicts[start..end], routes, dirty, pip)
            {
                self.pip_owners.insert(pip, net);
            } else {
                self.pip_owners.remove(&pip);
            }
            start = end;
        }
        self.pip_conflicts.clear();
    }
}

fn surviving_wire_owner(
    contenders: &[(WireId, NetId)],
    routes: &[Option<Arc<NetRoute>>],
    dirty: &BTreeMap<usize, BTreeSet<CellPinId>>,
    wire: WireId,
) -> Option<NetId> {
    contenders.iter().find_map(|&(_, net)| {
        routes
            .get(net.0)
            .and_then(Option::as_deref)
            .filter(|route| {
                route.arcs.iter().any(|arc| {
                    arc.wires.contains(&wire)
                        && arc.sink.is_none_or(|sink| {
                            dirty.get(&net.0).is_none_or(|set| !set.contains(&sink))
                        })
                })
            })
            .map(|_| net)
    })
}

fn surviving_pip_owner(
    contenders: &[(PipId, NetId)],
    routes: &[Option<Arc<NetRoute>>],
    dirty: &BTreeMap<usize, BTreeSet<CellPinId>>,
    pip: PipId,
) -> Option<NetId> {
    contenders.iter().find_map(|&(_, net)| {
        routes
            .get(net.0)
            .and_then(Option::as_deref)
            .filter(|route| {
                route.arcs.iter().any(|arc| {
                    arc.pips.contains(&pip)
                        && arc.sink.is_none_or(|sink| {
                            dirty.get(&net.0).is_none_or(|set| !set.contains(&sink))
                        })
                })
            })
            .map(|_| net)
    })
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct RankedConnectionOwner {
    net: NetId,
    first: usize,
    end: usize,
    locked: bool,
    criticality: u64,
}

impl ConnectionOwnerScratch {
    fn rebuild(
        &mut self,
        routes: &[Option<Arc<NetRoute>>],
        metadata: RoutingResourceMetadata<'_>,
        wire_occupancy: &[u16],
        pip_occupancy: &[u16],
    ) {
        self.wires.clear();
        self.pips.clear();
        for route in routes.iter().flatten() {
            for arc in &route.arcs {
                for &wire in &arc.wires {
                    if wire_occupancy[wire.0] > metadata.wire_capacities[wire.0] {
                        self.wires.push(ConnectionOwner {
                            resource: wire.0,
                            net: route.net,
                            sink: arc.sink,
                        });
                    }
                }
                for &pip in &arc.pips {
                    if pip_occupancy[pip.0] > metadata.pip_capacities[pip.0] {
                        self.pips.push(ConnectionOwner {
                            resource: pip.0,
                            net: route.net,
                            sink: arc.sink,
                        });
                    }
                }
            }
        }
        self.wires.sort_unstable();
        self.wires.dedup();
        self.pips.sort_unstable();
        self.pips.dedup();
    }

    fn rebuild_indexed(
        &mut self,
        routes: &[Option<Arc<NetRoute>>],
        index: &mut ResourceOwnerIndex,
    ) {
        self.wires.clear();
        self.pips.clear();
        index.wire_conflicts.sort_unstable();
        index.wire_conflicts.dedup();
        index.pip_conflicts.sort_unstable();
        index.pip_conflicts.dedup();
        for &(wire, net) in &index.wire_conflicts {
            let Some(route) = routes.get(net.0).and_then(Option::as_deref) else {
                continue;
            };
            for arc in &route.arcs {
                if arc.wires.contains(&wire) {
                    self.wires.push(ConnectionOwner {
                        resource: wire.0,
                        net,
                        sink: arc.sink,
                    });
                }
            }
        }
        for &(pip, net) in &index.pip_conflicts {
            let Some(route) = routes.get(net.0).and_then(Option::as_deref) else {
                continue;
            };
            for arc in &route.arcs {
                if arc.pips.contains(&pip) {
                    self.pips.push(ConnectionOwner {
                        resource: pip.0,
                        net,
                        sink: arc.sink,
                    });
                }
            }
        }
        self.wires.sort_unstable();
        self.wires.dedup();
        self.pips.sort_unstable();
        self.pips.dedup();
    }
}

fn congested_route_arcs(
    metadata: RoutingResourceMetadata<'_>,
    routes: &[Option<Arc<NetRoute>>],
    constraints: &RoutingConstraints,
    costs: Option<&RoutingCosts>,
    wire_occupancy: &[u16],
    pip_occupancy: &[u16],
    owner_scratch: &mut ConnectionOwnerScratch,
) -> BTreeMap<usize, BTreeSet<CellPinId>> {
    owner_scratch.rebuild(routes, metadata, wire_occupancy, pip_occupancy);
    let mut dirty = BTreeMap::<usize, BTreeSet<CellPinId>>::new();
    select_connection_victims(
        &owner_scratch.wires,
        &mut owner_scratch.ranked,
        metadata.wire_capacities,
        constraints,
        costs,
        &mut dirty,
    );
    select_connection_victims(
        &owner_scratch.pips,
        &mut owner_scratch.ranked,
        metadata.pip_capacities,
        constraints,
        costs,
        &mut dirty,
    );
    dirty
}

fn congested_route_arcs_indexed(
    metadata: RoutingResourceMetadata<'_>,
    routes: &[Option<Arc<NetRoute>>],
    constraints: &RoutingConstraints,
    costs: Option<&RoutingCosts>,
    resource_index: &mut ResourceOwnerIndex,
    owner_scratch: &mut ConnectionOwnerScratch,
) -> BTreeMap<usize, BTreeSet<CellPinId>> {
    owner_scratch.rebuild_indexed(routes, resource_index);
    let mut dirty = BTreeMap::<usize, BTreeSet<CellPinId>>::new();
    select_connection_victims(
        &owner_scratch.wires,
        &mut owner_scratch.ranked,
        metadata.wire_capacities,
        constraints,
        costs,
        &mut dirty,
    );
    select_connection_victims(
        &owner_scratch.pips,
        &mut owner_scratch.ranked,
        metadata.pip_capacities,
        constraints,
        costs,
        &mut dirty,
    );
    dirty
}

fn select_connection_victims(
    records: &[ConnectionOwner],
    ranked: &mut Vec<RankedConnectionOwner>,
    capacities: &[u16],
    constraints: &RoutingConstraints,
    costs: Option<&RoutingCosts>,
    dirty: &mut BTreeMap<usize, BTreeSet<CellPinId>>,
) {
    let mut resource_start = 0;
    while resource_start < records.len() {
        let resource = records[resource_start].resource;
        let resource_end = records[resource_start..]
            .partition_point(|record| record.resource == resource)
            + resource_start;
        ranked.clear();
        let mut owner_start = resource_start;
        while owner_start < resource_end {
            let net = records[owner_start].net;
            let owner_end = records[owner_start..resource_end]
                .partition_point(|record| record.net == net)
                + owner_start;
            let sinks = &records[owner_start..owner_end];
            let locked = sinks.iter().any(|owner| {
                constraints
                    .routes()
                    .get(&net)
                    .is_some_and(|route| route.arcs.iter().any(|arc| arc.sink == owner.sink))
            });
            let criticality = sinks
                .iter()
                .filter_map(|owner| {
                    owner
                        .sink
                        .map(|sink| routing_arc_criticality(costs, net, sink))
                })
                .max()
                .unwrap_or(u64::MAX);
            ranked.push(RankedConnectionOwner {
                net,
                first: owner_start,
                end: owner_end,
                locked,
                criticality,
            });
            owner_start = owner_end;
        }
        let capacity = usize::from(capacities[resource]);
        if ranked.len() > capacity {
            if capacity == 1 {
                // Standard Pathfinder releases every movable connection from
                // a capacity-one conflict.  Criticality chooses which net
                // routes first and reacquires the resource; keeping a
                // critical incumbent fixed here can strand all other owners
                // in a ring of equivalent neighboring resources.
                for owner in ranked.iter().filter(|owner| !owner.locked) {
                    mark_connection_owner(records, owner, constraints, dirty);
                }
            } else {
                ranked.sort_unstable_by_key(|owner| {
                    (Reverse(owner.locked), Reverse(owner.criticality), owner.net)
                });
                for owner in ranked.iter().skip(capacity) {
                    mark_connection_owner(records, owner, constraints, dirty);
                }
            }
        }
        resource_start = resource_end;
    }
}

fn mark_connection_owner(
    records: &[ConnectionOwner],
    owner: &RankedConnectionOwner,
    constraints: &RoutingConstraints,
    dirty: &mut BTreeMap<usize, BTreeSet<CellPinId>>,
) {
    for record in &records[owner.first..owner.end] {
        if let Some(sink) = record.sink
            && constraints
                .routes()
                .get(&owner.net)
                .is_none_or(|route| route.arc(sink).is_none())
        {
            dirty.entry(owner.net.0).or_default().insert(sink);
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
    wire_congestion: &[u32],
    pip_congestion: &[u32],
    blocked_pip_words: &[u64],
    hard_occupancy: Option<HardRoutingOccupancy<'_>>,
    mut hard_blockers: Option<&mut HardRoutingBlockers>,
    allow_alternate_source: bool,
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
    let mut routed_sinks = arcs
        .iter()
        .filter_map(|arc| arc.sink)
        .collect::<BTreeSet<_>>();
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
    let driver_point = device.bels()[driver_bel.0].point;
    let sinks = ordered_sinks(net_id, &net.sinks, costs, |sink| {
        let sink_cell = design.pins()[sink.0].cell;
        placement.bel(sink_cell).map_or(u64::MAX, |bel| {
            device.bels()[bel.0].point.manhattan(driver_point)
        })
    });
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
        if routed_sinks.contains(sink_pin) {
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
                routed_sinks.insert(*sink_pin);
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
        let (mut path_wires, mut path_pips) = search
            .shortest_path(
                graph,
                &tree_wires,
                None,
                sink_wire,
                wire_congestion,
                pip_congestion,
                blocked_pip_words,
                hard_occupancy,
                hard_blockers.as_deref_mut(),
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
        if allow_alternate_source
            && minimum_arrival_ps == 0
            && criticality >= ROUTING_CRITICALITY_SCALE * 5 / 8
            && let Some(costs) = costs
            && let Some(delay_per_tile_ps) = costs.alternate_source_delay_per_tile_ps
        {
            let incumbent_score = route_path_score(
                &path_wires,
                &path_pips,
                wire_congestion,
                pip_congestion,
                costs,
                criticality,
                delay_quantum_ps,
                tree_arrival_ps,
            );
            let incumbent_start = *path_wires
                .last()
                .expect("a routed path includes its tree start");
            let goal_point = metadata.wire_points[sink_wire.0];
            let incumbent_start_distance =
                metadata.wire_points[incumbent_start.0].manhattan(goal_point);
            let alternate = tree_wires
                .iter()
                .copied()
                .filter(|&wire| wire != incumbent_start)
                .map(|wire| {
                    let score =
                        timing_tree_cost(tree_arrival_ps[wire.0], criticality, delay_quantum_ps)
                            .saturating_add(search.remaining_cost_estimate_with_delay(
                                metadata.wire_points[wire.0],
                                goal_point,
                                criticality,
                                delay_quantum_ps,
                                delay_per_tile_ps,
                            ));
                    let distance = metadata.wire_points[wire.0].manhattan(goal_point);
                    (score, tree_arrival_ps[wire.0], distance, wire)
                })
                .min();
            if let Some((
                alternate_estimate,
                alternate_arrival_ps,
                alternate_distance,
                alternate_start,
            )) = alternate
                && alternate_estimate <= incumbent_score
                && incumbent_start_distance <= TIMING_ROUTE_MARGIN.into()
                && alternate_distance > TIMING_ROUTE_MARGIN.into()
                && alternate_arrival_ps.saturating_add(delay_quantum_ps.saturating_mul(10))
                    < tree_arrival_ps[incumbent_start.0]
            {
                search.alternate_source_attempts += 1;
                let alternate_starts = BTreeSet::from([alternate_start]);
                if let Some((alternate_wires, alternate_pips)) = search.shortest_path(
                    graph,
                    &alternate_starts,
                    Some(&tree_wires),
                    sink_wire,
                    wire_congestion,
                    pip_congestion,
                    blocked_pip_words,
                    hard_occupancy,
                    hard_blockers.as_deref_mut(),
                    Some(costs),
                    criticality,
                    delay_quantum_ps,
                    tree_arrival_ps,
                    minimum_arrival_ps,
                    metadata,
                ) {
                    let alternate_score = route_path_score(
                        &alternate_wires,
                        &alternate_pips,
                        wire_congestion,
                        pip_congestion,
                        costs,
                        criticality,
                        delay_quantum_ps,
                        tree_arrival_ps,
                    );
                    if alternate_score < incumbent_score {
                        search.alternate_source_improvements += 1;
                        path_wires = alternate_wires;
                        path_pips = alternate_pips;
                    }
                }
            }
        }
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
        routed_sinks.insert(*sink_pin);
    }
    for &wire in &tree_wires {
        tree_arrival_ps[wire.0] = UNROUTED_ARRIVAL_PS;
    }
    Ok(NetRoute::new(net_id, arcs))
}

#[allow(clippy::too_many_arguments)]
fn route_path_score(
    path_wires: &[WireId],
    path_pips: &[PipId],
    wire_congestion: &[u32],
    pip_congestion: &[u32],
    costs: &RoutingCosts,
    criticality: u64,
    delay_quantum_ps: u64,
    tree_arrival_ps: &[u64],
) -> u64 {
    let start = *path_wires
        .last()
        .expect("a routed path includes its tree start");
    let mut arrival_ps = tree_arrival_ps[start.0];
    let mut score = timing_tree_cost(arrival_ps, criticality, delay_quantum_ps);
    for (&wire, &pip) in path_wires.iter().rev().skip(1).zip(path_pips.iter().rev()) {
        let next_arrival_ps = arrival_ps.saturating_add(u64::from(costs.pip_delays_ps[pip.0]));
        let congestion = u64::from(wire_congestion[wire.0]) + u64::from(pip_congestion[pip.0]);
        let step = if delay_quantum_ps == ROUTING_DELAY_QUANTUM_PS {
            routing_step_cost(
                costs.pip_delays_ps[pip.0],
                criticality,
                congestion,
                delay_quantum_ps,
            )
        } else {
            routing_transition_cost(
                arrival_ps,
                next_arrival_ps,
                criticality,
                congestion,
                delay_quantum_ps,
            )
        };
        score = score.saturating_add(step);
        arrival_ps = next_arrival_ps;
    }
    score
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

fn ordered_sinks(
    net: NetId,
    sinks: &[CellPinId],
    costs: Option<&RoutingCosts>,
    mut distance: impl FnMut(CellPinId) -> u64,
) -> Vec<CellPinId> {
    let mut ordered = sinks.to_vec();
    ordered.sort_by_key(|&sink| {
        let criticality = routing_arc_criticality(costs, net, sink);
        let minimum = costs
            .and_then(|costs| costs.sink_min_delays_ps.get(&(net, sink)))
            .copied()
            .unwrap_or(0);
        (Reverse(criticality), Reverse(minimum), distance(sink), sink)
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
    distance: Vec<u32>,
    arrival_ps: Vec<u32>,
    previous_wire: Vec<u32>,
    previous_pip: Vec<u32>,
    /// Frontier storage retained across sink and placement trials. Large
    /// critical searches can grow this to hundreds of thousands of entries;
    /// clearing keeps the allocation while preserving an empty logical queue.
    queue: BinaryHeap<Reverse<RouteQueueEntry>>,
    estimate_base_delay_ps: u64,
    estimate_delay_per_tile_ps: u64,
    alternate_source_attempts: u64,
    alternate_source_improvements: u64,
}

type RouteQueueEntry = (u32, u32, u32, u32);

fn compact_route_value(value: u64) -> u32 {
    value
        .try_into()
        .expect("physical route delay and cost fit u32")
}

fn compact_route_index(index: usize) -> u32 {
    index.try_into().expect("physical routing IDs fit u32")
}

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
            previous_wire: vec![u32::MAX; wire_count],
            previous_pip: vec![u32::MAX; wire_count],
            queue: BinaryHeap::new(),
            estimate_base_delay_ps: ROUTING_ESTIMATE_BASE_DELAY_PS,
            estimate_delay_per_tile_ps: ROUTING_ESTIMATE_DELAY_PER_TILE_PS,
            alternate_source_attempts: 0,
            alternate_source_improvements: 0,
        }
    }

    /// Architecture-scaled remaining cost for timing-driven A*.
    ///
    /// Raw Manhattan distance is in tiles while the accumulated path score
    /// blends picosecond delay with congestion. Converting a lightweight
    /// geometry delay prediction into that same score keeps the heuristic
    /// strong without overwhelming detours onto fast long-line resources.
    fn remaining_cost_estimate(
        &self,
        point: Point,
        goal: Point,
        criticality: u64,
        delay_quantum_ps: u64,
    ) -> u64 {
        self.remaining_cost_estimate_with_delay(
            point,
            goal,
            criticality,
            delay_quantum_ps,
            self.estimate_delay_per_tile_ps,
        )
    }

    fn remaining_cost_estimate_with_delay(
        &self,
        point: Point,
        goal: Point,
        criticality: u64,
        delay_quantum_ps: u64,
        delay_per_tile_ps: u64,
    ) -> u64 {
        let distance = point.manhattan(goal);
        if criticality == 0 {
            return distance;
        }
        let predicted_delay_ps = self
            .estimate_base_delay_ps
            .saturating_add(distance.saturating_mul(delay_per_tile_ps));
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
        avoid_wires: Option<&BTreeSet<WireId>>,
        goal: WireId,
        wire_congestion: &[u32],
        pip_congestion: &[u32],
        blocked_pip_words: &[u64],
        hard_occupancy: Option<HardRoutingOccupancy<'_>>,
        mut hard_blockers: Option<&mut HardRoutingBlockers>,
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
                wire_congestion,
                pip_congestion,
                blocked_pip_words,
                hard_occupancy,
                hard_blockers.as_deref_mut(),
                costs?,
                criticality,
                tree_delays_ps,
                minimum_arrival_ps,
                metadata,
            );
        }
        if criticality != 0 {
            let goal_point = metadata.wire_points[goal.0];
            // A geometrically close branch can have reached this area through
            // a very slow detour.  Centering the timing corridor on that
            // branch excludes the driver (and every other low-arrival tree
            // source), then accepts the slow route merely because it is the
            // only route inside the corridor.  Anchor the bounded search at
            // the earliest-arriving tree source instead.  Zero-delay shared
            // tree wires remain eligible, with geometry only breaking ties.
            let start_point = starts
                .iter()
                .min_by_key(|start| {
                    (
                        tree_delays_ps[start.0],
                        metadata.wire_points[start.0].manhattan(goal_point),
                        **start,
                    )
                })
                .map(|start| metadata.wire_points[start.0])
                .expect("a route tree always contains its driver");
            let corridor =
                routing_corridor(start_point, goal_point, graph.device(), TIMING_ROUTE_MARGIN);
            if let Some(path) = self.shortest_path_attempt(
                graph,
                starts,
                avoid_wires,
                goal,
                wire_congestion,
                pip_congestion,
                blocked_pip_words,
                hard_occupancy,
                hard_blockers.as_deref_mut(),
                costs,
                criticality,
                delay_quantum_ps,
                tree_delays_ps,
                minimum_arrival_ps,
                metadata,
                Some(corridor),
            ) {
                return Some(path);
            }
        }
        self.shortest_path_attempt(
            graph,
            starts,
            avoid_wires,
            goal,
            wire_congestion,
            pip_congestion,
            blocked_pip_words,
            hard_occupancy,
            hard_blockers,
            costs,
            criticality,
            delay_quantum_ps,
            tree_delays_ps,
            minimum_arrival_ps,
            metadata,
            None,
        )
    }

    #[allow(clippy::too_many_arguments, clippy::too_many_lines)]
    fn shortest_path_attempt(
        &mut self,
        graph: &UnifiedGraph<'_>,
        starts: &BTreeSet<WireId>,
        avoid_wires: Option<&BTreeSet<WireId>>,
        goal: WireId,
        wire_congestion: &[u32],
        pip_congestion: &[u32],
        blocked_pip_words: &[u64],
        hard_occupancy: Option<HardRoutingOccupancy<'_>>,
        mut hard_blockers: Option<&mut HardRoutingBlockers>,
        costs: Option<&RoutingCosts>,
        criticality: u64,
        delay_quantum_ps: u64,
        tree_delays_ps: &[u64],
        minimum_arrival_ps: u64,
        metadata: RoutingResourceMetadata<'_>,
        corridor: Option<RoutingCorridor>,
    ) -> Option<(Vec<WireId>, Vec<PipId>)> {
        self.epoch = self.epoch.wrapping_add(1);
        if self.epoch == 0 {
            self.seen.fill(0);
            self.start_mark.fill(0);
            self.epoch = 1;
        }
        let epoch = self.epoch;
        let goal_point = metadata.wire_points[goal.0];
        self.queue.clear();
        if let Some(avoid_wires) = avoid_wires {
            for &wire in avoid_wires {
                self.start_mark[wire.0] = epoch;
            }
        }
        for &start in starts {
            self.start_mark[start.0] = epoch;
        }
        for &start in starts {
            // A retained high-fanout tree can contain thousands of legal
            // sources while a local sink search is confined to a small timing
            // corridor. An outside source with no direct edge into that
            // corridor cannot relax any state in the search below, so putting
            // it through the heap is pure work. Keep every source marked as a
            // tree member, but seed only sources that can enter the allowed
            // subgraph in zero or one hop.
            if let Some(corridor) = corridor
                && !point_inside_corridor(metadata.wire_points[start.0], corridor)
                && !graph.routing_neighbors(start).ok()?.any(|(neighbor, _)| {
                    point_inside_corridor(metadata.wire_points[neighbor.0], corridor)
                })
            {
                continue;
            }
            let arrival_ps = tree_delays_ps[start.0];
            let distance = timing_tree_cost(arrival_ps, criticality, delay_quantum_ps);
            let compact_arrival = compact_route_value(arrival_ps);
            let compact_distance = compact_route_value(distance);
            self.seen[start.0] = epoch;
            self.distance[start.0] = compact_distance;
            self.arrival_ps[start.0] = compact_arrival;
            self.previous_wire[start.0] = u32::MAX;
            self.previous_pip[start.0] = u32::MAX;
            let estimate = if hard_occupancy.is_none_or(|occupancy| occupancy.use_estimate) {
                distance.saturating_add(self.remaining_cost_estimate(
                    metadata.wire_points[start.0],
                    goal_point,
                    criticality,
                    delay_quantum_ps,
                ))
            } else {
                distance
            };
            self.queue.push(Reverse((
                compact_route_value(estimate),
                compact_distance,
                compact_arrival,
                compact_route_index(start.0),
            )));
        }

        while let Some(Reverse((_, compact_distance, compact_arrival, compact_wire))) =
            self.queue.pop()
        {
            let wire = WireId(compact_wire as usize);
            if self.seen[wire.0] != epoch
                || (self.distance[wire.0], self.arrival_ps[wire.0])
                    != (compact_distance, compact_arrival)
            {
                continue;
            }
            let distance = u64::from(compact_distance);
            let arrival_ps = u64::from(compact_arrival);
            if wire == goal {
                let mut path_wires = vec![wire];
                let mut path_pips = Vec::new();
                let mut cursor = wire.0;
                while self.previous_wire[cursor] != u32::MAX {
                    path_pips.push(PipId(self.previous_pip[cursor] as usize));
                    cursor = self.previous_wire[cursor] as usize;
                    path_wires.push(WireId(cursor));
                }
                return Some((path_wires, path_pips));
            }

            for (neighbor, pip) in graph.routing_neighbors(wire).ok()? {
                if pip_is_blocked(blocked_pip_words, pip) {
                    continue;
                }
                if self.start_mark[neighbor.0] == epoch {
                    continue;
                }
                if corridor.is_some_and(|corridor| {
                    !point_inside_corridor(metadata.wire_points[neighbor.0], corridor)
                }) {
                    continue;
                }
                if let Some(occupancy) = hard_occupancy {
                    let wire_blocked =
                        occupancy.wires[neighbor.0] >= metadata.wire_capacities[neighbor.0];
                    let pip_blocked = occupancy.pips[pip.0] >= metadata.pip_capacities[pip.0];
                    if wire_blocked || pip_blocked {
                        if let Some(blockers) = hard_blockers.as_deref_mut() {
                            if wire_blocked {
                                blockers.wires.insert(neighbor);
                            }
                            if pip_blocked {
                                blockers.pips.insert(pip);
                            }
                        }
                        continue;
                    }
                }
                let congestion =
                    u64::from(wire_congestion[neighbor.0]) + u64::from(pip_congestion[pip.0]);
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
                let compact_next_distance = compact_route_value(next_distance);
                let compact_next_arrival = compact_route_value(next_arrival_ps);
                if neighbor == goal && next_arrival_ps < minimum_arrival_ps {
                    continue;
                }
                if self.seen[neighbor.0] == epoch
                    && (self.distance[neighbor.0], self.arrival_ps[neighbor.0])
                        <= (compact_next_distance, compact_next_arrival)
                {
                    continue;
                }
                self.seen[neighbor.0] = epoch;
                self.distance[neighbor.0] = compact_next_distance;
                self.arrival_ps[neighbor.0] = compact_next_arrival;
                self.previous_wire[neighbor.0] = compact_route_index(wire.0);
                self.previous_pip[neighbor.0] = compact_route_index(pip.0);
                let estimate = if hard_occupancy.is_none_or(|occupancy| occupancy.use_estimate) {
                    next_distance.saturating_add(self.remaining_cost_estimate(
                        metadata.wire_points[neighbor.0],
                        goal_point,
                        criticality,
                        delay_quantum_ps,
                    ))
                } else {
                    next_distance
                };
                self.queue.push(Reverse((
                    compact_route_value(estimate),
                    compact_next_distance,
                    compact_next_arrival,
                    compact_route_index(neighbor.0),
                )));
            }
        }
        None
    }
}

type RoutingCorridor = (u32, u32, u32, u32);

// The 85K AXI4 closure search reaches every timing sink without falling back
// at this margin. Larger margins explore resources that never win; smaller
// margins perturb the critical topology enough to lose timing closure.
const TIMING_ROUTE_MARGIN: u32 = 4;

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
    wire_congestion: &[u32],
    pip_congestion: &[u32],
    blocked_pip_words: &[u64],
    hard_occupancy: Option<HardRoutingOccupancy<'_>>,
    mut hard_blockers: Option<&mut HardRoutingBlockers>,
    costs: &RoutingCosts,
    criticality: u64,
    tree_delays_ps: &[u64],
    minimum_arrival_ps: u64,
    metadata: RoutingResourceMetadata<'_>,
) -> Option<(Vec<WireId>, Vec<PipId>)> {
    let goal_point = metadata.wire_points[goal.0];
    // Hold repair is a local ECO. Searching the whole device in the
    // (wire, delay-bucket) state space is both disproportionately expensive
    // and liable to trade a short hold deficit for a destructive long detour.
    // Use the same architecture-scaled corridor as timing-driven setup
    // routing; the retained tree may still enter through any source adjacent
    // to the corridor.
    let start_point = starts
        .iter()
        .map(|start| metadata.wire_points[start.0])
        .min_by_key(|point| (point.manhattan(goal_point), *point))
        .expect("a route tree always contains its driver");
    let corridor = routing_corridor(start_point, goal_point, graph.device(), TIMING_ROUTE_MARGIN);
    let mut visits = HashMap::<HoldRouteState, HoldRouteVisit>::new();
    let mut queue = BinaryHeap::new();
    for &start in starts {
        if !point_inside_corridor(metadata.wire_points[start.0], corridor)
            && !graph.routing_neighbors(start).ok()?.any(|(neighbor, _)| {
                point_inside_corridor(metadata.wire_points[neighbor.0], corridor)
            })
        {
            continue;
        }
        let arrival_ps = tree_delays_ps[start.0];
        let state = (start, hold_delay_bucket(arrival_ps, minimum_arrival_ps));
        let distance = timing_tree_cost(arrival_ps, criticality, ROUTING_DELAY_QUANTUM_PS);
        visits.insert(state, (distance, arrival_ps, None));
        let estimate = if hard_occupancy.is_some() {
            distance
        } else {
            distance.saturating_add(metadata.wire_points[start.0].manhattan(goal_point))
        };
        queue.push(Reverse((estimate, distance, arrival_ps, state)));
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
            if pip_is_blocked(blocked_pip_words, pip) {
                continue;
            }
            if starts.contains(&neighbor) {
                continue;
            }
            if !point_inside_corridor(metadata.wire_points[neighbor.0], corridor) {
                continue;
            }
            if let Some(occupancy) = hard_occupancy {
                let wire_blocked =
                    occupancy.wires[neighbor.0] >= metadata.wire_capacities[neighbor.0];
                let pip_blocked = occupancy.pips[pip.0] >= metadata.pip_capacities[pip.0];
                if wire_blocked || pip_blocked {
                    if let Some(blockers) = hard_blockers.as_deref_mut() {
                        if wire_blocked {
                            blockers.wires.insert(neighbor);
                        }
                        if pip_blocked {
                            blockers.pips.insert(pip);
                        }
                    }
                    continue;
                }
            }
            let congestion =
                u64::from(wire_congestion[neighbor.0]) + u64::from(pip_congestion[pip.0]);
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
            let estimate = if hard_occupancy.is_some() {
                next_distance
            } else {
                next_distance.saturating_add(metadata.wire_points[neighbor.0].manhattan(goal_point))
            };
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

// Keep physical arrival delay at full weight for every timing-routed sink.
// Criticality still controls routing order, rip-up priority, and hop bias;
// discounting delay here makes a late shared-tree branch look artificially
// cheap, then turns that formerly noncritical sink into the next worst path.
fn timing_tree_cost(arrival_ps: u64, criticality: u64, delay_quantum_ps: u64) -> u64 {
    if criticality == 0 {
        0
    } else {
        arrival_ps.div_ceil(delay_quantum_ps)
    }
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
    u64::from(pip_delay_ps)
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

fn cached_congestion_cost(occupancy: u16, capacity: u16, history: u32, present: u32) -> u32 {
    // `present` is capped at 4096 and history can grow for at most 32 routing
    // iterations. Even the maximum u16 occupancy therefore remains below
    // u32::MAX, while the search accumulator itself stays u64.
    congestion_cost(occupancy, capacity, history, present)
        .try_into()
        .expect("negotiated congestion fits u32")
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
    /// A target supplied an invalid device-wide routing restriction.
    InvalidRoutingRestriction {
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
            Self::InvalidRoutingRestriction { reason } => {
                write!(f, "invalid routing restriction: {reason}")
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
            | Self::InvalidRoutingRestriction { .. }
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
    use std::cmp::Reverse;
    use std::collections::{BTreeMap, BTreeSet};
    use std::convert::Infallible;
    use std::sync::Arc;

    use texo_model::{
        BelId, BelPinId, CellId, CellPinId, Design, Device, NetId, PinDirection, PipId, Point,
        ResourceKind, UnifiedGraph, WireId,
    };

    use super::analytical_placement::AnalyticalObjective;

    use super::{
        AxisEquations, ConnectionOwnerScratch, HardRoutingBlockers, LegalRouteEcoConnection,
        LegalRouteEcoOptions, LegalRoutePolishConnection, LegalRoutePolishMetrics,
        LegalRoutePolishSubscriptions, MAX_ROUTING_ITERATIONS, NetRoute, PinWireCache, Placement,
        PlacementConstraints, PlacementDelayEstimator, PlacementNeighbor,
        PlacementRefinementWorkspace, PlacementRefiner, PnrError, PnrResult, ResourceOwnerIndex,
        RouteArc, RouteCapacityProjection, RouteQueueEntry, RouteSearch,
        RoutingConflictCycleDetector, RoutingConstraints, RoutingCosts, RoutingProgress,
        RoutingResourceMetadata, RoutingWorkspace, congested_route_arcs,
        congested_route_arcs_indexed, fanout_placement_weight, first_dyadic_strict_descent,
        legal_net_route_eco_candidate_with_workspace,
        legal_nets_route_eco_candidate_with_workspace, legal_route_eco_candidate_with_workspace,
        local_connection_projected_cost_from_starts, maximum_window_occupancy, nearest_rank,
        ordered_sinks, pip_is_blocked, place_analytically_with_net_sink_weights, place_and_route,
        place_with_constraints, placement_from_complete_bindings, placement_from_partial_bindings,
        placement_neighbors, polish_legal_timing_routes, prioritize_cycle_connections,
        projected_release_scope_penalty, projected_resource_penalty,
        refine_placement_with_net_sink_weights_limited, refine_placement_with_net_weights,
        refinement_edge_cost, retain_route_for_sinks, rounded_coordinate, route_reaches_all_sinks,
        route_with_placement_and_progress, route_with_timing_costs_and_progress,
        route_with_workspace_and_progress, routing_corridor, routing_step_cost,
        routing_transition_cost, solve_analytical_axis, solve_quadratic, timing_tree_cost,
        unloaded_arc_cost,
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
    fn legalization_metric_quantiles_use_nearest_rank() {
        let sorted = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9];
        assert_eq!(nearest_rank(&sorted, 50), 4);
        assert_eq!(nearest_rank(&sorted, 95), 9);
        assert_eq!(nearest_rank(&[], 95), 0);
    }

    #[test]
    fn legalization_metric_window_counts_boundary_points_once() {
        let points = [
            Point::new(0, 0),
            Point::new(1, 1),
            Point::new(2, 2),
            Point::new(3, 3),
            Point::new(3, 3),
        ];
        assert_eq!(maximum_window_occupancy(&points, 4, 4, 1), 2);
        assert_eq!(maximum_window_occupancy(&points, 4, 4, 3), 4);
        assert_eq!(maximum_window_occupancy(&points, 4, 4, 4), 5);
    }

    #[test]
    fn complete_bindings_reject_a_group_spliced_from_different_assignment_rows() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(4, 1).unwrap();
        let mut constraints = PlacementConstraints::new();
        constraints.add_group(
            [CellId(0), CellId(1)],
            [vec![BelId(0), BelId(1)], vec![BelId(2), BelId(3)]],
        );
        let error = placement_from_complete_bindings(
            &design,
            &device,
            &constraints,
            vec![BelId(0), BelId(3)],
        )
        .unwrap_err();

        assert!(matches!(error, PnrError::InvalidPlacement { .. }));
        assert!(
            error
                .to_string()
                .contains("does not match one complete legal assignment row")
        );
    }

    #[test]
    fn complete_bindings_match_partial_bindings_and_resolve_pin_names() {
        let mut design = Design::new();
        let cell = design.add_cell("cell", ResourceKind::Logic);
        let logical_pin = design
            .add_pin(cell, "logical", PinDirection::Output)
            .unwrap();
        let mut device = Device::new("complete-bindings", 2, 1).unwrap();
        let first_wire = device.add_wire("first-wire", Point::new(0, 0), 1).unwrap();
        let second_wire = device.add_wire("second-wire", Point::new(1, 0), 1).unwrap();
        let first = device
            .add_bel("first", ResourceKind::Logic, Point::new(0, 0))
            .unwrap();
        device
            .add_bel_pin(first, "physical", PinDirection::Output, first_wire)
            .unwrap();
        let second = device
            .add_bel("second", ResourceKind::Logic, Point::new(1, 0))
            .unwrap();
        let second_pin = device
            .add_bel_pin(second, "physical", PinDirection::Output, second_wire)
            .unwrap();
        let mut constraints = PlacementConstraints::new();
        constraints.bind_pin_name(logical_pin, "physical");
        constraints.add_group([cell], [vec![second]]);
        let partial_bindings = BTreeMap::from([(cell, second)]);

        let partial =
            placement_from_partial_bindings(&design, &device, &constraints, &partial_bindings)
                .unwrap();
        let complete =
            placement_from_complete_bindings(&design, &device, &constraints, vec![second]).unwrap();

        assert_eq!(complete, partial);
        assert_eq!(complete.pin_binding(logical_pin), Some(second_pin));
    }

    #[test]
    fn complete_bindings_validate_table_shape_ownership_and_kind() {
        let design = two_cell_design();
        let mut device = Device::rectangular_logic(2, 1).unwrap();
        let io = device
            .add_bel("io", ResourceKind::Io, Point::new(0, 0))
            .unwrap();

        for bindings in [vec![BelId(0)], vec![BelId(0), BelId(0)], vec![io, BelId(1)]] {
            assert!(matches!(
                placement_from_complete_bindings(
                    &design,
                    &device,
                    &PlacementConstraints::new(),
                    bindings,
                ),
                Err(PnrError::InvalidPlacement { .. })
            ));
        }

        let mut shared = PlacementConstraints::new();
        shared.add_shared_resource(
            [(CellId(0), 0), (CellId(1), 1)],
            [(BelId(0), 7), (BelId(1), 7)],
        );
        assert!(matches!(
            placement_from_complete_bindings(&design, &device, &shared, vec![BelId(0), BelId(1)],),
            Err(PnrError::InvalidPlacement { .. })
        ));
    }

    #[test]
    #[ignore = "release-only microbenchmark; run explicitly with --ignored --nocapture"]
    fn benchmark_complete_binding_fast_path() {
        const COUNT: u32 = 4_096;
        let mut design = Design::new();
        let mut device = Device::new("complete-binding-benchmark", COUNT, 1).unwrap();
        let mut complete_bindings = Vec::new();
        let mut partial_bindings = BTreeMap::new();
        for index in 0..COUNT {
            let cell = design.add_cell(format!("cell-{index}"), ResourceKind::Logic);
            let bel = device
                .add_bel(
                    format!("bel-{index}"),
                    ResourceKind::Logic,
                    Point::new(index, 0),
                )
                .unwrap();
            complete_bindings.push(bel);
            partial_bindings.insert(cell, bel);
        }
        let constraints = PlacementConstraints::new();

        let partial_iterations = 3_u32;
        let partial_started = std::time::Instant::now();
        for _ in 0..partial_iterations {
            std::hint::black_box(
                placement_from_partial_bindings(&design, &device, &constraints, &partial_bindings)
                    .unwrap(),
            );
        }
        let partial_elapsed =
            partial_started.elapsed().as_secs_f64() / f64::from(partial_iterations);

        let complete_iterations = 100_u32;
        let complete_started = std::time::Instant::now();
        for _ in 0..complete_iterations {
            std::hint::black_box(
                placement_from_complete_bindings(
                    &design,
                    &device,
                    &constraints,
                    complete_bindings.clone(),
                )
                .unwrap(),
            );
        }
        let complete_elapsed =
            complete_started.elapsed().as_secs_f64() / f64::from(complete_iterations);
        eprintln!(
            "complete binding cells={COUNT} partial_ms={:.3} complete_ms={:.3} speedup={:.2}x",
            partial_elapsed * 1.0e3,
            complete_elapsed * 1.0e3,
            partial_elapsed / complete_elapsed,
        );
        assert!(complete_elapsed < partial_elapsed);
    }

    #[test]
    fn rigid_macro_boundary_enumeration_includes_every_member_net() {
        let mut design = Design::new();
        let first = design.add_cell("first", ResourceKind::Logic);
        let first_out = design.add_pin(first, "O", PinDirection::Output).unwrap();
        let second = design.add_cell("second", ResourceKind::Logic);
        let second_out = design.add_pin(second, "F", PinDirection::Output).unwrap();
        let internal = design.add_pin(second, "FCI", PinDirection::Input).unwrap();
        let packed_ff = design.add_cell("packed-ff", ResourceKind::Register);
        let packed_di = design
            .add_pin(packed_ff, "DI", PinDirection::Input)
            .unwrap();
        let sink_a = design.add_cell("sink-a", ResourceKind::Register);
        let sink_a_in = design.add_pin(sink_a, "I", PinDirection::Input).unwrap();
        let sink_b = design.add_cell("sink-b", ResourceKind::Register);
        let second_sink_input = design.add_pin(sink_b, "I", PinDirection::Input).unwrap();
        let sink_c = design.add_cell("sink-c", ResourceKind::Register);
        let third_sink_input = design.add_pin(sink_c, "I", PinDirection::Input).unwrap();
        design
            .add_net("first-external", first_out, [sink_a_in])
            .unwrap();
        design
            .add_net(
                "mixed-result-fanout",
                second_out,
                [packed_di, second_sink_input, third_sink_input],
            )
            .unwrap();
        let internal_out = design.add_pin(first, "OI", PinDirection::Output).unwrap();
        design
            .add_net("internal", internal_out, [internal])
            .unwrap();
        let unit = super::PlacementUnit {
            cells: vec![first, second, packed_ff],
            choices: super::PlacementChoices::Shared(vec![Vec::new()].into()),
        };

        assert_eq!(
            super::external_unit_connections(&design, &unit),
            vec![
                (first_out, sink_a_in),
                (second_out, second_sink_input),
                (second_out, third_sink_input),
            ]
        );
    }

    #[test]
    fn rigid_macro_retained_tree_excludes_every_moving_sink_branch() {
        let mut design = Design::new();
        let source = design.add_cell("source", ResourceKind::Logic);
        let driver = design.add_pin(source, "out", PinDirection::Output).unwrap();
        let first = design.add_cell("first-macro-member", ResourceKind::Register);
        let first_sink = design.add_pin(first, "in", PinDirection::Input).unwrap();
        let second = design.add_cell("second-macro-member", ResourceKind::Register);
        let second_sink = design.add_pin(second, "in", PinDirection::Input).unwrap();
        let net = design
            .add_net("two-moving-sinks", driver, [first_sink, second_sink])
            .unwrap();
        let unit = super::PlacementUnit {
            cells: vec![first, second],
            choices: super::PlacementChoices::Shared(vec![Vec::new()].into()),
        };
        let trunk = WireId(0);
        let first_leaf = WireId(1);
        let second_leaf = WireId(2);
        let route = NetRoute::new(
            net,
            vec![
                RouteArc {
                    sink: None,
                    wires: vec![trunk],
                    pips: Vec::new(),
                },
                RouteArc {
                    sink: Some(first_sink),
                    wires: vec![first_leaf],
                    pips: Vec::new(),
                },
                RouteArc {
                    sink: Some(second_sink),
                    wires: vec![second_leaf],
                    pips: Vec::new(),
                },
            ],
        );
        let projection = RouteCapacityProjection::new(
            &[Arc::new(route)],
            &RoutingCosts::new(Vec::new(), BTreeMap::new()),
        );

        let retained = super::projected_retained_tree_starts(
            &design,
            &unit,
            &[(driver, first_sink), (driver, second_sink)],
            &projection,
        );
        assert_eq!(retained[&(net, first_sink)].as_ref(), &[trunk]);
        assert_eq!(retained[&(net, second_sink)].as_ref(), &[trunk]);
    }

    fn add_equivalent_logic_cell(
        design: &mut Design,
        name: &str,
    ) -> (CellId, CellPinId, CellPinId) {
        let cell = design.add_cell(name, ResourceKind::Logic);
        let input = design.add_pin(cell, "in", PinDirection::Input).unwrap();
        let output = design.add_pin(cell, "out", PinDirection::Output).unwrap();
        (cell, input, output)
    }

    struct LegalPolishFixture {
        design: Design,
        device: Device,
        placement: Placement,
        costs: RoutingCosts,
        routes: Vec<Arc<NetRoute>>,
        target_sink: CellPinId,
        fast_pips: [PipId; 2],
        occupied_detour: WireId,
    }

    #[allow(clippy::too_many_lines)]
    fn legal_polish_fixture() -> LegalPolishFixture {
        let mut design = Design::new();
        let target_source = design.add_cell("target-source", ResourceKind::Logic);
        let target_output = design
            .add_pin(target_source, "O", PinDirection::Output)
            .unwrap();
        let target_sink_cell = design.add_cell("target-sink", ResourceKind::Register);
        let target_sink = design
            .add_pin(target_sink_cell, "I", PinDirection::Input)
            .unwrap();
        let obstacle_source = design.add_cell("obstacle-source", ResourceKind::Logic);
        let obstacle_output = design
            .add_pin(obstacle_source, "OO", PinDirection::Output)
            .unwrap();
        let obstacle_sink_cell = design.add_cell("obstacle-sink", ResourceKind::Register);
        let obstacle_sink = design
            .add_pin(obstacle_sink_cell, "II", PinDirection::Input)
            .unwrap();
        design
            .add_net("target", target_output, [target_sink])
            .unwrap();
        design
            .add_net("obstacle", obstacle_output, [obstacle_sink])
            .unwrap();

        let mut device = Device::new("legal-polish", 5, 2).unwrap();
        let driver = device.add_wire("driver", Point::new(0, 0), 1).unwrap();
        let slow = device.add_wire("slow", Point::new(1, 1), 1).unwrap();
        let fast = device.add_wire("fast", Point::new(2, 0), 1).unwrap();
        let target_goal = device.add_wire("target-goal", Point::new(4, 0), 1).unwrap();
        let obstacle_driver = device
            .add_wire("obstacle-driver", Point::new(0, 1), 1)
            .unwrap();
        let occupied_detour = device
            .add_wire("occupied-detour", Point::new(2, 1), 1)
            .unwrap();
        let obstacle_goal = device
            .add_wire("obstacle-goal", Point::new(4, 1), 1)
            .unwrap();

        let slow_first = device.add_pip(driver, slow, false, 1).unwrap();
        let slow_last = device.add_pip(slow, target_goal, false, 1).unwrap();
        let fast_first = device.add_pip(driver, fast, false, 1).unwrap();
        let fast_last = device.add_pip(fast, target_goal, false, 1).unwrap();
        // This would be the cheapest target path if polishing were allowed to
        // displace an already-legal connection. The obstacle net owns its
        // middle wire and must remain untouched.
        device.add_pip(driver, occupied_detour, false, 1).unwrap();
        device
            .add_pip(occupied_detour, target_goal, false, 1)
            .unwrap();
        let obstacle_first = device
            .add_pip(obstacle_driver, occupied_detour, false, 1)
            .unwrap();
        let obstacle_last = device
            .add_pip(occupied_detour, obstacle_goal, false, 1)
            .unwrap();

        let add_bel = |device: &mut Device,
                       name: &str,
                       kind: ResourceKind,
                       point: Point,
                       pin: &str,
                       direction: PinDirection,
                       wire: WireId| {
            let bel = device.add_bel(name, kind, point).unwrap();
            device.add_bel_pin(bel, pin, direction, wire).unwrap();
            bel
        };
        let target_source_bel = add_bel(
            &mut device,
            "target-source",
            ResourceKind::Logic,
            Point::new(0, 0),
            "O",
            PinDirection::Output,
            driver,
        );
        let target_sink_bel = add_bel(
            &mut device,
            "target-sink",
            ResourceKind::Register,
            Point::new(4, 0),
            "I",
            PinDirection::Input,
            target_goal,
        );
        let obstacle_source_bel = add_bel(
            &mut device,
            "obstacle-source",
            ResourceKind::Logic,
            Point::new(0, 1),
            "OO",
            PinDirection::Output,
            obstacle_driver,
        );
        let obstacle_sink_bel = add_bel(
            &mut device,
            "obstacle-sink",
            ResourceKind::Register,
            Point::new(4, 1),
            "II",
            PinDirection::Input,
            obstacle_goal,
        );
        let placement = Placement {
            bindings: vec![
                target_source_bel,
                target_sink_bel,
                obstacle_source_bel,
                obstacle_sink_bel,
            ],
            pin_bindings: BTreeMap::new(),
        };
        let target_route = Arc::new(NetRoute::new(
            NetId(0),
            vec![RouteArc {
                sink: Some(target_sink),
                wires: vec![driver, slow, target_goal],
                pips: vec![slow_first, slow_last],
            }],
        ));
        let obstacle_route = Arc::new(NetRoute::new(
            NetId(1),
            vec![RouteArc {
                sink: Some(obstacle_sink),
                wires: vec![obstacle_driver, occupied_detour, obstacle_goal],
                pips: vec![obstacle_first, obstacle_last],
            }],
        ));
        let mut costs = RoutingCosts::new(
            vec![300, 300, 100, 100, 1, 1, 10, 10],
            BTreeMap::from([(NetId(0), 64), (NetId(1), 1)]),
        );
        costs.set_sink_criticalities(BTreeMap::from([
            ((NetId(0), target_sink), 64),
            ((NetId(1), obstacle_sink), 1),
        ]));
        LegalPolishFixture {
            design,
            device,
            placement,
            costs,
            routes: vec![target_route, obstacle_route],
            target_sink,
            fast_pips: [fast_first, fast_last],
            occupied_detour,
        }
    }

    struct WholeNetEcoFixture {
        design: Design,
        device: Device,
        incumbent: PnrResult,
        costs: RoutingCosts,
        sinks: Vec<CellPinId>,
        driver: WireId,
        slow: WireId,
        branch: WireId,
        slow_pips: [PipId; 2],
        fast_pips: [PipId; 2],
        leaf_pips: Vec<PipId>,
    }

    #[allow(clippy::too_many_lines)]
    fn whole_net_eco_fixture() -> WholeNetEcoFixture {
        let mut design = Design::new();
        let source = design.add_cell("target-source", ResourceKind::Logic);
        let output = design.add_pin(source, "O", PinDirection::Output).unwrap();
        let mut sinks = Vec::new();
        for index in 0..4 {
            let cell = design.add_cell(format!("target-sink-{index}"), ResourceKind::Register);
            let sink = design.add_pin(cell, "I", PinDirection::Input).unwrap();
            sinks.push(sink);
        }
        let obstacle_source = design.add_cell("obstacle-source", ResourceKind::Logic);
        let obstacle_output = design
            .add_pin(obstacle_source, "OO", PinDirection::Output)
            .unwrap();
        let obstacle_sink_cell = design.add_cell("obstacle-sink", ResourceKind::Register);
        let obstacle_sink = design
            .add_pin(obstacle_sink_cell, "II", PinDirection::Input)
            .unwrap();
        let target_net = design
            .add_net("target", output, sinks.iter().copied())
            .unwrap();
        let obstacle_net = design
            .add_net("obstacle", obstacle_output, [obstacle_sink])
            .unwrap();

        let mut device = Device::new("whole-net-eco", 4, 5).unwrap();
        let driver = device.add_wire("driver", Point::new(0, 0), 1).unwrap();
        let slow = device.add_wire("slow", Point::new(1, 1), 1).unwrap();
        let fast = device.add_wire("fast", Point::new(1, 0), 1).unwrap();
        let branch = device.add_wire("branch", Point::new(2, 0), 1).unwrap();
        let leaves = (0..4)
            .map(|index| {
                device
                    .add_wire(format!("goal-{index}"), Point::new(3, index), 1)
                    .unwrap()
            })
            .collect::<Vec<_>>();
        let obstacle_driver = device
            .add_wire("obstacle-driver", Point::new(0, 4), 1)
            .unwrap();
        let occupied_shortcut = device
            .add_wire("occupied-shortcut", Point::new(1, 3), 1)
            .unwrap();
        let obstacle_goal = device
            .add_wire("obstacle-goal", Point::new(3, 4), 1)
            .unwrap();

        let slow_pips = [
            device.add_pip(driver, slow, false, 1).unwrap(),
            device.add_pip(slow, branch, false, 1).unwrap(),
        ];
        let fast_pips = [
            device.add_pip(driver, fast, false, 1).unwrap(),
            device.add_pip(fast, branch, false, 1).unwrap(),
        ];
        // The physically occupied shortcut is faster than either legal target
        // prefix. A hard whole-net transaction must not displace its owner.
        device.add_pip(driver, occupied_shortcut, false, 1).unwrap();
        device.add_pip(occupied_shortcut, branch, false, 1).unwrap();
        let leaf_pips = leaves
            .iter()
            .map(|&leaf| device.add_pip(branch, leaf, false, 1).unwrap())
            .collect::<Vec<_>>();
        let obstacle_pips = [
            device
                .add_pip(obstacle_driver, occupied_shortcut, false, 1)
                .unwrap(),
            device
                .add_pip(occupied_shortcut, obstacle_goal, false, 1)
                .unwrap(),
        ];

        let add_bel = |device: &mut Device,
                       name: &str,
                       kind: ResourceKind,
                       point: Point,
                       pin: &str,
                       direction: PinDirection,
                       wire: WireId| {
            let bel = device.add_bel(name, kind, point).unwrap();
            device.add_bel_pin(bel, pin, direction, wire).unwrap();
            bel
        };
        let source_bel = add_bel(
            &mut device,
            "target-source",
            ResourceKind::Logic,
            Point::new(0, 0),
            "O",
            PinDirection::Output,
            driver,
        );
        let sink_bels = leaves
            .iter()
            .enumerate()
            .map(|(index, &leaf)| {
                add_bel(
                    &mut device,
                    &format!("target-sink-{index}"),
                    ResourceKind::Register,
                    Point::new(3, u32::try_from(index).unwrap()),
                    "I",
                    PinDirection::Input,
                    leaf,
                )
            })
            .collect::<Vec<_>>();
        let obstacle_source_bel = add_bel(
            &mut device,
            "obstacle-source",
            ResourceKind::Logic,
            Point::new(0, 4),
            "OO",
            PinDirection::Output,
            obstacle_driver,
        );
        let obstacle_sink_bel = add_bel(
            &mut device,
            "obstacle-sink",
            ResourceKind::Register,
            Point::new(3, 4),
            "II",
            PinDirection::Input,
            obstacle_goal,
        );
        let mut bindings = vec![source_bel];
        bindings.extend(sink_bels);
        bindings.extend([obstacle_source_bel, obstacle_sink_bel]);
        let placement = Placement {
            bindings,
            pin_bindings: BTreeMap::new(),
        };

        let target_route = Arc::new(NetRoute::new(
            target_net,
            sinks
                .iter()
                .zip(&leaves)
                .zip(&leaf_pips)
                .map(|((&sink, &leaf), &leaf_pip)| RouteArc {
                    sink: Some(sink),
                    wires: vec![driver, slow, branch, leaf],
                    pips: vec![slow_pips[0], slow_pips[1], leaf_pip],
                })
                .collect(),
        ));
        let obstacle_route = Arc::new(NetRoute::new(
            obstacle_net,
            vec![RouteArc {
                sink: Some(obstacle_sink),
                wires: vec![obstacle_driver, occupied_shortcut, obstacle_goal],
                pips: obstacle_pips.to_vec(),
            }],
        ));
        let routes = vec![target_route, obstacle_route];
        let incumbent = PnrResult {
            placement,
            total_pips: routes.iter().map(|route| route.pips().len()).sum(),
            routes,
        };

        let mut pip_delays = vec![1_u32; device.pips().len()];
        for pip in slow_pips {
            pip_delays[pip.0] = 400;
        }
        for pip in fast_pips {
            pip_delays[pip.0] = 50;
        }
        for &pip in &leaf_pips {
            pip_delays[pip.0] = 10;
        }
        let mut costs = RoutingCosts::new(
            pip_delays,
            BTreeMap::from([(target_net, 64), (obstacle_net, 1)]),
        );
        costs.set_sink_criticalities(
            sinks
                .iter()
                .enumerate()
                .map(|(index, &sink)| ((target_net, sink), 64 - u64::try_from(index).unwrap() * 8))
                .chain([((obstacle_net, obstacle_sink), 1)])
                .collect(),
        );

        WholeNetEcoFixture {
            design,
            device,
            incumbent,
            costs,
            sinks,
            driver,
            slow,
            branch,
            slow_pips,
            fast_pips,
            leaf_pips,
        }
    }

    struct NetCohortEcoFixture {
        design: Design,
        device: Device,
        incumbent: PnrResult,
        costs: RoutingCosts,
        a_sink: CellPinId,
        b_sink: CellPinId,
        a_fast_pips: [PipId; 2],
        b_alternate_pips: [PipId; 2],
    }

    #[allow(clippy::too_many_lines)]
    fn net_cohort_eco_fixture() -> NetCohortEcoFixture {
        let mut design = Design::new();
        let a_source = design.add_cell("a-source", ResourceKind::Logic);
        let a_output = design
            .add_pin(a_source, "AO", PinDirection::Output)
            .unwrap();
        let a_sink_cell = design.add_cell("a-sink", ResourceKind::Register);
        let a_sink = design
            .add_pin(a_sink_cell, "AI", PinDirection::Input)
            .unwrap();
        let b_source = design.add_cell("b-source", ResourceKind::Logic);
        let b_output = design
            .add_pin(b_source, "BO", PinDirection::Output)
            .unwrap();
        let b_sink_cell = design.add_cell("b-sink", ResourceKind::Register);
        let b_sink = design
            .add_pin(b_sink_cell, "BI", PinDirection::Input)
            .unwrap();
        let fixed_source = design.add_cell("fixed-source", ResourceKind::Logic);
        let fixed_output = design
            .add_pin(fixed_source, "FO", PinDirection::Output)
            .unwrap();
        let fixed_sink_cell = design.add_cell("fixed-sink", ResourceKind::Register);
        let fixed_sink = design
            .add_pin(fixed_sink_cell, "FI", PinDirection::Input)
            .unwrap();
        let a_net = design.add_net("a", a_output, [a_sink]).unwrap();
        let b_net = design.add_net("b", b_output, [b_sink]).unwrap();
        let fixed_net = design
            .add_net("unselected", fixed_output, [fixed_sink])
            .unwrap();

        let mut device = Device::new("net-cohort-eco", 5, 3).unwrap();
        let a_driver = device.add_wire("a-driver", Point::new(0, 0), 1).unwrap();
        let a_slow = device.add_wire("a-slow", Point::new(2, 0), 1).unwrap();
        let a_goal = device.add_wire("a-goal", Point::new(4, 0), 1).unwrap();
        let b_driver = device.add_wire("b-driver", Point::new(0, 1), 1).unwrap();
        let shared = device.add_wire("shared-fast", Point::new(2, 1), 1).unwrap();
        let b_alternate = device.add_wire("b-alternate", Point::new(2, 2), 1).unwrap();
        let b_goal = device.add_wire("b-goal", Point::new(4, 1), 1).unwrap();
        let fixed_driver = device
            .add_wire("fixed-driver", Point::new(0, 2), 1)
            .unwrap();
        let fixed_goal = device.add_wire("fixed-goal", Point::new(4, 2), 1).unwrap();

        let a_slow_pips = [
            device.add_pip(a_driver, a_slow, false, 1).unwrap(),
            device.add_pip(a_slow, a_goal, false, 1).unwrap(),
        ];
        let a_fast_pips = [
            device.add_pip(a_driver, shared, false, 1).unwrap(),
            device.add_pip(shared, a_goal, false, 1).unwrap(),
        ];
        let b_shared_pips = [
            device.add_pip(b_driver, shared, false, 1).unwrap(),
            device.add_pip(shared, b_goal, false, 1).unwrap(),
        ];
        let b_alternate_pips = [
            device.add_pip(b_driver, b_alternate, false, 1).unwrap(),
            device.add_pip(b_alternate, b_goal, false, 1).unwrap(),
        ];
        let fixed_pip = device.add_pip(fixed_driver, fixed_goal, false, 1).unwrap();

        let add_bel = |device: &mut Device,
                       name: &str,
                       kind: ResourceKind,
                       point: Point,
                       pin: &str,
                       direction: PinDirection,
                       wire: WireId| {
            let bel = device.add_bel(name, kind, point).unwrap();
            device.add_bel_pin(bel, pin, direction, wire).unwrap();
            bel
        };
        let bindings = vec![
            add_bel(
                &mut device,
                "a-source",
                ResourceKind::Logic,
                Point::new(0, 0),
                "AO",
                PinDirection::Output,
                a_driver,
            ),
            add_bel(
                &mut device,
                "a-sink",
                ResourceKind::Register,
                Point::new(4, 0),
                "AI",
                PinDirection::Input,
                a_goal,
            ),
            add_bel(
                &mut device,
                "b-source",
                ResourceKind::Logic,
                Point::new(0, 1),
                "BO",
                PinDirection::Output,
                b_driver,
            ),
            add_bel(
                &mut device,
                "b-sink",
                ResourceKind::Register,
                Point::new(4, 1),
                "BI",
                PinDirection::Input,
                b_goal,
            ),
            add_bel(
                &mut device,
                "fixed-source",
                ResourceKind::Logic,
                Point::new(0, 2),
                "FO",
                PinDirection::Output,
                fixed_driver,
            ),
            add_bel(
                &mut device,
                "fixed-sink",
                ResourceKind::Register,
                Point::new(4, 2),
                "FI",
                PinDirection::Input,
                fixed_goal,
            ),
        ];
        let placement = Placement {
            bindings,
            pin_bindings: BTreeMap::new(),
        };
        let routes = vec![
            Arc::new(NetRoute::new(
                a_net,
                vec![RouteArc {
                    sink: Some(a_sink),
                    wires: vec![a_driver, a_slow, a_goal],
                    pips: a_slow_pips.to_vec(),
                }],
            )),
            Arc::new(NetRoute::new(
                b_net,
                vec![RouteArc {
                    sink: Some(b_sink),
                    wires: vec![b_driver, shared, b_goal],
                    pips: b_shared_pips.to_vec(),
                }],
            )),
            Arc::new(NetRoute::new(
                fixed_net,
                vec![RouteArc {
                    sink: Some(fixed_sink),
                    wires: vec![fixed_driver, fixed_goal],
                    pips: vec![fixed_pip],
                }],
            )),
        ];
        let incumbent = PnrResult {
            placement,
            total_pips: routes.iter().map(|route| route.pips().len()).sum(),
            routes,
        };

        let mut pip_delays = vec![1_u32; device.pips().len()];
        for pip in a_slow_pips {
            pip_delays[pip.0] = 400;
        }
        for pip in a_fast_pips {
            pip_delays[pip.0] = 20;
        }
        for pip in b_shared_pips {
            pip_delays[pip.0] = 20;
        }
        for pip in b_alternate_pips {
            pip_delays[pip.0] = 100;
        }
        let mut costs = RoutingCosts::new(
            pip_delays,
            BTreeMap::from([(a_net, 64), (b_net, 8), (fixed_net, 1)]),
        );
        costs.set_sink_criticalities(BTreeMap::from([
            ((a_net, a_sink), 64),
            ((b_net, b_sink), 8),
            ((fixed_net, fixed_sink), 1),
        ]));

        NetCohortEcoFixture {
            design,
            device,
            incumbent,
            costs,
            a_sink,
            b_sink,
            a_fast_pips,
            b_alternate_pips,
        }
    }

    type LegalPolishRun = (
        Vec<Arc<NetRoute>>,
        LegalRoutePolishMetrics,
        Vec<u16>,
        Vec<u16>,
    );

    fn run_legal_polish(fixture: &LegalPolishFixture) -> LegalPolishRun {
        let graph = UnifiedGraph::new(&fixture.design, &fixture.device);
        let pin_wires = PinWireCache::build(&graph, &fixture.placement);
        let mut routes = fixture.routes.iter().cloned().map(Some).collect::<Vec<_>>();
        let mut wire_occupancy = vec![0_u16; fixture.device.wires().len()];
        let mut pip_occupancy = vec![0_u16; fixture.device.pips().len()];
        for route in &fixture.routes {
            for wire in route.wires() {
                wire_occupancy[wire.0] += 1;
            }
            for pip in route.pips() {
                pip_occupancy[pip.0] += 1;
            }
        }
        let mut touched_wires = wire_occupancy
            .iter()
            .enumerate()
            .filter_map(|(index, &occupancy)| (occupancy != 0).then_some(index))
            .collect::<Vec<_>>();
        let mut touched_pips = pip_occupancy
            .iter()
            .enumerate()
            .filter_map(|(index, &occupancy)| (occupancy != 0).then_some(index))
            .collect::<Vec<_>>();
        let wire_points = fixture
            .device
            .wires()
            .iter()
            .map(|wire| wire.point)
            .collect::<Vec<_>>();
        let wire_capacities = fixture
            .device
            .wires()
            .iter()
            .map(|wire| wire.capacity)
            .collect::<Vec<_>>();
        let pip_capacities = fixture
            .device
            .pips()
            .iter()
            .map(texo_model::Pip::capacity)
            .collect::<Vec<_>>();
        let metadata = RoutingResourceMetadata {
            wire_points: &wire_points,
            wire_capacities: &wire_capacities,
            pip_capacities: &pip_capacities,
        };
        let mut search = RouteSearch::new(fixture.device.wires().len());
        let mut tree_arrival_ps = vec![super::UNROUTED_ARRIVAL_PS; fixture.device.wires().len()];
        let metrics = polish_legal_timing_routes(
            &graph,
            &fixture.placement,
            &pin_wires,
            &RoutingConstraints::new(),
            &fixture.costs,
            &mut routes,
            &mut wire_occupancy,
            &mut pip_occupancy,
            &vec![0; fixture.device.wires().len()],
            &vec![0; fixture.device.pips().len()],
            &mut touched_wires,
            &mut touched_pips,
            &mut search,
            &mut tree_arrival_ps,
            metadata,
        );
        (
            routes.into_iter().map(Option::unwrap).collect(),
            metrics,
            wire_occupancy,
            pip_occupancy,
        )
    }

    #[test]
    fn legal_route_polish_improves_one_arc_without_displacing_an_owner() {
        let fixture = legal_polish_fixture();
        let old_arc = fixture.routes[0].arc(fixture.target_sink).unwrap();
        let old_cost = unloaded_arc_cost(NetId(0), old_arc, &fixture.costs, 64);
        let obstacle = fixture.routes[1].clone();

        let (routes, metrics, wire_occupancy, pip_occupancy) = run_legal_polish(&fixture);
        let new_arc = routes[0].arc(fixture.target_sink).unwrap();
        let new_cost = unloaded_arc_cost(NetId(0), new_arc, &fixture.costs, 64);

        assert_eq!(new_arc.pips, fixture.fast_pips);
        assert!(new_cost < old_cost, "old={old_cost}, new={new_cost}");
        assert_eq!(routes[1], obstacle);
        assert!(!new_arc.wires.contains(&fixture.occupied_detour));
        assert_eq!(wire_occupancy[fixture.occupied_detour.0], 1);
        assert!(wire_occupancy.iter().all(|&occupancy| occupancy <= 1));
        assert!(pip_occupancy.iter().all(|&occupancy| occupancy <= 1));
        assert_eq!(
            metrics,
            LegalRoutePolishMetrics {
                passes: 1,
                initial_candidates: 1,
                attempts: 1,
                wakeups: 0,
                improvements: 1,
                objective_reduction: old_cost - new_cost,
            }
        );
    }

    fn legal_polish_incumbent(fixture: &LegalPolishFixture) -> PnrResult {
        PnrResult {
            placement: fixture.placement.clone(),
            total_pips: fixture.routes.iter().map(|route| route.pips().len()).sum(),
            routes: fixture.routes.clone(),
        }
    }

    #[test]
    fn legal_route_eco_changes_only_the_selected_connection_under_hard_occupancy() {
        let fixture = legal_polish_fixture();
        let incumbent = legal_polish_incumbent(&fixture);
        let before = incumbent.clone();
        let obstacle = incumbent.routes[1].clone();
        let mut workspace = RoutingWorkspace::new(&fixture.device);
        workspace.search.estimate_delay_per_tile_ps = 73;

        let candidate = legal_route_eco_candidate_with_workspace(
            &fixture.design,
            &fixture.device,
            &incumbent,
            &RoutingConstraints::new(),
            &fixture.costs,
            LegalRouteEcoConnection::new(NetId(0), fixture.target_sink),
            LegalRouteEcoOptions::new(52),
            &mut workspace,
        )
        .unwrap()
        .expect("the target has a faster conflict-free route");

        assert_eq!(incumbent, before, "candidate construction is transactional");
        assert_eq!(candidate.placement, incumbent.placement);
        assert_eq!(
            candidate.routes[0].arc(fixture.target_sink).unwrap().pips,
            fixture.fast_pips
        );
        assert!(Arc::ptr_eq(&candidate.routes[1], &obstacle));
        assert_eq!(candidate.routes[1], incumbent.routes[1]);
        assert!(
            !candidate.routes[0]
                .wires()
                .any(|wire| wire == fixture.occupied_detour)
        );
        assert_eq!(workspace.search.estimate_delay_per_tile_ps, 73);
    }

    #[test]
    fn failed_legal_route_eco_leaves_the_incumbent_bit_exact() {
        let fixture = legal_polish_fixture();
        let incumbent = legal_polish_incumbent(&fixture);
        let before = incumbent.clone();
        let mut costs = fixture.costs.clone();
        costs.set_sink_min_delays_ps(BTreeMap::from([((NetId(0), fixture.target_sink), 10_000)]));
        let mut workspace = RoutingWorkspace::new(&fixture.device);

        let candidate = legal_route_eco_candidate_with_workspace(
            &fixture.design,
            &fixture.device,
            &incumbent,
            &RoutingConstraints::new(),
            &costs,
            LegalRouteEcoConnection::new(NetId(0), fixture.target_sink),
            LegalRouteEcoOptions::new(52),
            &mut workspace,
        )
        .unwrap();

        assert!(candidate.is_none());
        assert_eq!(incumbent, before);
        assert!(
            incumbent
                .routes
                .iter()
                .zip(&before.routes)
                .all(|(current, original)| Arc::ptr_eq(current, original))
        );
    }

    fn assert_workspace_restored_to_incumbent(
        fixture: &WholeNetEcoFixture,
        workspace: &RoutingWorkspace,
    ) {
        assert_workspace_matches_incumbent(&fixture.device, &fixture.incumbent, workspace);
    }

    fn assert_workspace_matches_incumbent(
        device: &Device,
        incumbent: &PnrResult,
        workspace: &RoutingWorkspace,
    ) {
        let mut expected_wires = vec![0_u16; device.wires().len()];
        let mut expected_pips = vec![0_u16; device.pips().len()];
        for route in &incumbent.routes {
            for wire in route.wires() {
                expected_wires[wire.0] += 1;
            }
            for pip in route.pips() {
                expected_pips[pip.0] += 1;
            }
        }
        assert_eq!(workspace.wire_occupancy, expected_wires);
        assert_eq!(workspace.pip_occupancy, expected_pips);
        assert!(workspace.wire_history.iter().all(|&history| history == 0));
        assert!(workspace.pip_history.iter().all(|&history| history == 0));
        assert!(workspace.wire_congestion.iter().all(|&cost| cost == 0));
        assert!(workspace.pip_congestion.iter().all(|&cost| cost == 0));
        assert!(workspace.resident_valid);
        assert_eq!(workspace.resident_routes.len(), incumbent.routes.len());
        assert!(workspace.resident_routes.iter().zip(&incumbent.routes).all(
            |(resident, incumbent)| {
                resident
                    .as_ref()
                    .is_some_and(|resident| Arc::ptr_eq(resident, incumbent))
            }
        ));
    }

    #[test]
    #[allow(clippy::too_many_lines)]
    fn net_cohort_eco_releases_a_competing_route_before_criticality_ordered_rebuild() {
        let fixture = net_cohort_eco_fixture();
        let before = fixture.incumbent.clone();
        let unselected = fixture.incumbent.routes[2].clone();
        let mut reused_workspace = RoutingWorkspace::new(&fixture.device);

        let single = legal_net_route_eco_candidate_with_workspace(
            &fixture.design,
            &fixture.device,
            &fixture.incumbent,
            &RoutingConstraints::new(),
            &fixture.costs,
            NetId(0),
            LegalRouteEcoOptions::new(52),
            &mut reused_workspace,
        )
        .unwrap();
        assert!(
            single.is_none(),
            "net B's hard occupancy must hide net A's fast resource"
        );

        let mut malformed = fixture.incumbent.clone();
        malformed.routes.pop();
        let error = legal_nets_route_eco_candidate_with_workspace(
            &fixture.design,
            &fixture.device,
            &malformed,
            &RoutingConstraints::new(),
            &fixture.costs,
            &[NetId(0), NetId(1)],
            LegalRouteEcoOptions::new(52),
            &mut reused_workspace,
        )
        .unwrap_err();
        assert!(
            error
                .to_string()
                .contains("incumbent contains 2 route trees")
        );
        assert_workspace_matches_incumbent(&fixture.device, &fixture.incumbent, &reused_workspace);

        // Reverse order, with a duplicate, deliberately differs from the
        // required criticality order. Canonical cohort scheduling must still
        // rebuild A first, then move B to its alternate route.
        let candidate = legal_nets_route_eco_candidate_with_workspace(
            &fixture.design,
            &fixture.device,
            &fixture.incumbent,
            &RoutingConstraints::new(),
            &fixture.costs,
            &[NetId(1), NetId(0), NetId(1)],
            LegalRouteEcoOptions::new(52),
            &mut reused_workspace,
        )
        .unwrap()
        .expect("simultaneous release must expose the fast resource to net A");

        assert_eq!(fixture.incumbent, before);
        assert_eq!(
            candidate.routes[0].arc(fixture.a_sink).unwrap().pips,
            fixture.a_fast_pips
        );
        assert_eq!(
            candidate.routes[1].arc(fixture.b_sink).unwrap().pips,
            fixture.b_alternate_pips
        );
        assert!(Arc::ptr_eq(&candidate.routes[2], &unselected));
        assert_workspace_matches_incumbent(&fixture.device, &fixture.incumbent, &reused_workspace);

        let mut impossible_costs = fixture.costs.clone();
        impossible_costs
            .set_sink_min_delays_ps(BTreeMap::from([((NetId(1), fixture.b_sink), 10_000)]));
        let failed = legal_nets_route_eco_candidate_with_workspace(
            &fixture.design,
            &fixture.device,
            &fixture.incumbent,
            &RoutingConstraints::new(),
            &impossible_costs,
            &[NetId(0), NetId(1)],
            LegalRouteEcoOptions::new(52),
            &mut reused_workspace,
        )
        .unwrap();
        assert!(
            failed.is_none(),
            "a later unroutable cohort member must roll back earlier rebuilds"
        );
        assert_workspace_matches_incumbent(&fixture.device, &fixture.incumbent, &reused_workspace);

        let reused = legal_nets_route_eco_candidate_with_workspace(
            &fixture.design,
            &fixture.device,
            &fixture.incumbent,
            &RoutingConstraints::new(),
            &fixture.costs,
            &[NetId(0), NetId(1)],
            LegalRouteEcoOptions::new(52),
            &mut reused_workspace,
        )
        .unwrap();
        let mut fresh_workspace = RoutingWorkspace::new(&fixture.device);
        let fresh = legal_nets_route_eco_candidate_with_workspace(
            &fixture.design,
            &fixture.device,
            &fixture.incumbent,
            &RoutingConstraints::new(),
            &fixture.costs,
            &[NetId(0), NetId(1)],
            LegalRouteEcoOptions::new(52),
            &mut fresh_workspace,
        )
        .unwrap();
        assert_eq!(reused, fresh);
    }

    #[test]
    fn whole_net_eco_rebuilds_a_shared_slow_prefix_transactionally() {
        let fixture = whole_net_eco_fixture();
        let before = fixture.incumbent.clone();
        let obstacle = fixture.incumbent.routes[1].clone();
        let mut workspace = RoutingWorkspace::new(&fixture.device);

        let connection_candidate = legal_route_eco_candidate_with_workspace(
            &fixture.design,
            &fixture.device,
            &fixture.incumbent,
            &RoutingConstraints::new(),
            &fixture.costs,
            LegalRouteEcoConnection::new(NetId(0), fixture.sinks[0]),
            LegalRouteEcoOptions::new(52),
            &mut workspace,
        )
        .unwrap();
        assert!(
            connection_candidate.is_none(),
            "the retained sibling tree prevents a shared-prefix repair"
        );

        let candidate = legal_net_route_eco_candidate_with_workspace(
            &fixture.design,
            &fixture.device,
            &fixture.incumbent,
            &RoutingConstraints::new(),
            &fixture.costs,
            NetId(0),
            LegalRouteEcoOptions::new(52),
            &mut workspace,
        )
        .unwrap()
        .expect("whole-net release exposes the fast shared prefix");

        assert_eq!(fixture.incumbent, before);
        assert!(
            fixture
                .incumbent
                .routes
                .iter()
                .zip(&before.routes)
                .all(|(current, original)| Arc::ptr_eq(current, original))
        );
        assert_eq!(candidate.placement, fixture.incumbent.placement);
        assert!(Arc::ptr_eq(&candidate.routes[1], &obstacle));
        for ((&sink, &leaf_pip), arc) in fixture
            .sinks
            .iter()
            .zip(&fixture.leaf_pips)
            .zip(candidate.routes[0].arcs.iter())
        {
            assert_eq!(arc.sink, Some(sink));
            assert_eq!(
                arc.pips,
                [fixture.fast_pips[0], fixture.fast_pips[1], leaf_pip]
            );
            assert!(arc.pips.iter().all(|pip| !fixture.slow_pips.contains(pip)));
        }
        assert_workspace_restored_to_incumbent(&fixture, &workspace);

        let mut fresh_workspace = RoutingWorkspace::new(&fixture.device);
        let repeated = legal_net_route_eco_candidate_with_workspace(
            &fixture.design,
            &fixture.device,
            &fixture.incumbent,
            &RoutingConstraints::new(),
            &fixture.costs,
            NetId(0),
            LegalRouteEcoOptions::new(52),
            &mut workspace,
        )
        .unwrap();
        let fresh = legal_net_route_eco_candidate_with_workspace(
            &fixture.design,
            &fixture.device,
            &fixture.incumbent,
            &RoutingConstraints::new(),
            &fixture.costs,
            NetId(0),
            LegalRouteEcoOptions::new(52),
            &mut fresh_workspace,
        )
        .unwrap();
        assert_eq!(repeated, fresh);
    }

    #[test]
    fn failed_whole_net_eco_restores_workspace_before_a_reused_trial() {
        let fixture = whole_net_eco_fixture();
        let before = fixture.incumbent.clone();
        let mut impossible_costs = fixture.costs.clone();
        impossible_costs.set_sink_min_delays_ps(
            fixture
                .sinks
                .iter()
                .map(|&sink| ((NetId(0), sink), 10_000))
                .collect(),
        );
        let mut reused_workspace = RoutingWorkspace::new(&fixture.device);
        let failed = legal_net_route_eco_candidate_with_workspace(
            &fixture.design,
            &fixture.device,
            &fixture.incumbent,
            &RoutingConstraints::new(),
            &impossible_costs,
            NetId(0),
            LegalRouteEcoOptions::new(52),
            &mut reused_workspace,
        )
        .unwrap();
        assert!(failed.is_none());
        assert_eq!(fixture.incumbent, before);
        assert_workspace_restored_to_incumbent(&fixture, &reused_workspace);

        let reused = legal_net_route_eco_candidate_with_workspace(
            &fixture.design,
            &fixture.device,
            &fixture.incumbent,
            &RoutingConstraints::new(),
            &fixture.costs,
            NetId(0),
            LegalRouteEcoOptions::new(52),
            &mut reused_workspace,
        )
        .unwrap();
        let mut fresh_workspace = RoutingWorkspace::new(&fixture.device);
        let fresh = legal_net_route_eco_candidate_with_workspace(
            &fixture.design,
            &fixture.device,
            &fixture.incumbent,
            &RoutingConstraints::new(),
            &fixture.costs,
            NetId(0),
            LegalRouteEcoOptions::new(52),
            &mut fresh_workspace,
        )
        .unwrap();
        assert_eq!(reused, fresh);
    }

    #[test]
    fn whole_net_eco_preserves_partial_locked_architecture_topology() {
        let mut fixture = whole_net_eco_fixture();
        let locked_arc = RouteArc {
            sink: None,
            wires: vec![fixture.driver, fixture.slow, fixture.branch],
            pips: fixture.slow_pips.to_vec(),
        };
        let mut arcs = fixture.incumbent.routes[0].arcs.clone();
        arcs.push(locked_arc.clone());
        fixture.incumbent.routes[0] = Arc::new(NetRoute::new(NetId(0), arcs));
        fixture.incumbent.total_pips = fixture
            .incumbent
            .routes
            .iter()
            .map(|route| route.pips().len())
            .sum();
        let mut constraints = RoutingConstraints::new();
        constraints.add_route(NetRoute::new(NetId(0), vec![locked_arc]));
        let before = fixture.incumbent.clone();
        let mut workspace = RoutingWorkspace::new(&fixture.device);

        let candidate = legal_net_route_eco_candidate_with_workspace(
            &fixture.design,
            &fixture.device,
            &fixture.incumbent,
            &constraints,
            &fixture.costs,
            NetId(0),
            LegalRouteEcoOptions::new(52),
            &mut workspace,
        )
        .unwrap();

        assert!(candidate.is_none());
        assert_eq!(fixture.incumbent, before);
        assert_workspace_restored_to_incumbent(&fixture, &workspace);
    }

    #[test]
    fn legal_route_eco_never_changes_a_locked_target_arc() {
        let fixture = legal_polish_fixture();
        let incumbent = legal_polish_incumbent(&fixture);
        let before = incumbent.clone();
        let mut constraints = RoutingConstraints::new();
        constraints.add_route(incumbent.routes[0].clone());
        let mut workspace = RoutingWorkspace::new(&fixture.device);

        let candidate = legal_route_eco_candidate_with_workspace(
            &fixture.design,
            &fixture.device,
            &incumbent,
            &constraints,
            &fixture.costs,
            LegalRouteEcoConnection::new(NetId(0), fixture.target_sink),
            LegalRouteEcoOptions::new(52),
            &mut workspace,
        )
        .unwrap();

        assert!(candidate.is_none());
        assert_eq!(incumbent, before);
        assert!(
            incumbent
                .routes
                .iter()
                .zip(&before.routes)
                .all(|(current, original)| Arc::ptr_eq(current, original))
        );
    }

    #[test]
    fn legal_route_polish_is_deterministic_and_stops_at_a_strict_fixed_point() {
        let fixture = legal_polish_fixture();
        let first = run_legal_polish(&fixture);
        let second = run_legal_polish(&fixture);

        assert_eq!(first, second);
        assert_eq!(first.1.passes, 1);
        assert_eq!(first.1.improvements, 1);
        // Re-running from the fixed point attempts the initial candidate once
        // and accepts no equal-cost replacement.
        let fixed_fixture = LegalPolishFixture {
            routes: first.0.clone(),
            ..fixture
        };
        let fixed = run_legal_polish(&fixed_fixture);
        assert_eq!(fixed.0, first.0);
        assert_eq!(
            fixed.1,
            LegalRoutePolishMetrics {
                passes: 1,
                initial_candidates: 1,
                attempts: 1,
                wakeups: 0,
                improvements: 0,
                objective_reduction: 0,
            }
        );
    }

    #[test]
    fn legal_route_polish_replaces_stale_resource_subscriptions() {
        let connection = LegalRoutePolishConnection {
            net: NetId(0),
            sink: CellPinId(1),
        };
        let mut subscriptions = LegalRoutePolishSubscriptions::default();
        subscriptions.replace(
            connection,
            HardRoutingBlockers {
                wires: BTreeSet::from([WireId(0)]),
                pips: BTreeSet::from([PipId(0)]),
            },
        );
        assert_eq!(
            subscriptions.subscribers([WireId(0)], [PipId(0)]),
            BTreeSet::from([connection])
        );

        subscriptions.replace(
            connection,
            HardRoutingBlockers {
                wires: BTreeSet::from([WireId(1)]),
                pips: BTreeSet::new(),
            },
        );
        assert!(
            subscriptions
                .subscribers([WireId(0)], [PipId(0)])
                .is_empty()
        );
        assert_eq!(
            subscriptions.subscribers([WireId(1)], []),
            BTreeSet::from([connection])
        );
    }

    #[test]
    #[allow(clippy::too_many_lines)]
    fn legal_route_polish_wakes_only_a_connection_blocked_by_a_released_resource() {
        let mut design = Design::new();
        let source_a = design.add_cell("source-a", ResourceKind::Logic);
        let output_a = design.add_pin(source_a, "O", PinDirection::Output).unwrap();
        let alpha_sink_cell = design.add_cell("sink-a", ResourceKind::Register);
        let sink_a = design
            .add_pin(alpha_sink_cell, "I", PinDirection::Input)
            .unwrap();
        let source_b = design.add_cell("source-b", ResourceKind::Logic);
        let output_b = design.add_pin(source_b, "O", PinDirection::Output).unwrap();
        let beta_sink_cell = design.add_cell("sink-b", ResourceKind::Register);
        let sink_b = design
            .add_pin(beta_sink_cell, "I", PinDirection::Input)
            .unwrap();
        let net_a = design.add_net("a", output_a, [sink_a]).unwrap();
        let net_b = design.add_net("b", output_b, [sink_b]).unwrap();

        let mut device = Device::new("polish-events", 1, 1).unwrap();
        let point = Point::new(0, 0);
        let driver_a = device.add_wire("driver-a", point, 1).unwrap();
        let slow_a = device.add_wire("slow-a", point, 1).unwrap();
        let goal_a = device.add_wire("goal-a", point, 1).unwrap();
        let driver_b = device.add_wire("driver-b", point, 1).unwrap();
        let shared = device.add_wire("shared", point, 1).unwrap();
        let fast_b = device.add_wire("fast-b", point, 1).unwrap();
        let goal_b = device.add_wire("goal-b", point, 1).unwrap();

        let a_slow_first = device.add_pip(driver_a, slow_a, false, 1).unwrap();
        let a_slow_last = device.add_pip(slow_a, goal_a, false, 1).unwrap();
        let a_fast_first = device.add_pip(driver_a, shared, false, 1).unwrap();
        let a_fast_last = device.add_pip(shared, goal_a, false, 1).unwrap();
        let b_shared_first = device.add_pip(driver_b, shared, false, 1).unwrap();
        let b_shared_last = device.add_pip(shared, goal_b, false, 1).unwrap();
        let b_fast_first = device.add_pip(driver_b, fast_b, false, 1).unwrap();
        let b_fast_last = device.add_pip(fast_b, goal_b, false, 1).unwrap();

        let add_bel = |device: &mut Device,
                       name: &str,
                       kind: ResourceKind,
                       pin: &str,
                       direction: PinDirection,
                       wire: WireId| {
            let bel = device.add_bel(name, kind, point).unwrap();
            device.add_bel_pin(bel, pin, direction, wire).unwrap();
            bel
        };
        let alpha_source_bel = add_bel(
            &mut device,
            "source-a",
            ResourceKind::Logic,
            "O",
            PinDirection::Output,
            driver_a,
        );
        let alpha_sink_bel = add_bel(
            &mut device,
            "sink-a",
            ResourceKind::Register,
            "I",
            PinDirection::Input,
            goal_a,
        );
        let beta_source_bel = add_bel(
            &mut device,
            "source-b",
            ResourceKind::Logic,
            "O",
            PinDirection::Output,
            driver_b,
        );
        let beta_sink_bel = add_bel(
            &mut device,
            "sink-b",
            ResourceKind::Register,
            "I",
            PinDirection::Input,
            goal_b,
        );
        let placement = Placement {
            bindings: vec![
                alpha_source_bel,
                alpha_sink_bel,
                beta_source_bel,
                beta_sink_bel,
            ],
            pin_bindings: BTreeMap::new(),
        };
        let route_a = Arc::new(NetRoute::new(
            net_a,
            vec![RouteArc {
                sink: Some(sink_a),
                wires: vec![driver_a, slow_a, goal_a],
                pips: vec![a_slow_first, a_slow_last],
            }],
        ));
        let route_b = Arc::new(NetRoute::new(
            net_b,
            vec![RouteArc {
                sink: Some(sink_b),
                wires: vec![driver_b, shared, goal_b],
                pips: vec![b_shared_first, b_shared_last],
            }],
        ));
        let mut costs = RoutingCosts::new(
            vec![500, 500, 50, 50, 300, 300, 100, 100],
            BTreeMap::from([(net_a, 64), (net_b, 32)]),
        );
        costs.set_sink_criticalities(BTreeMap::from([
            ((net_a, sink_a), 64),
            ((net_b, sink_b), 32),
        ]));
        let fixture = LegalPolishFixture {
            design,
            device,
            placement,
            costs,
            routes: vec![route_a, route_b],
            target_sink: sink_a,
            fast_pips: [a_fast_first, a_fast_last],
            occupied_detour: shared,
        };

        let first = run_legal_polish(&fixture);
        let second = run_legal_polish(&fixture);
        assert_eq!(first, second);
        assert_eq!(first.0[0].arc(sink_a).unwrap().pips, fixture.fast_pips);
        assert_eq!(
            first.0[1].arc(sink_b).unwrap().pips,
            [b_fast_first, b_fast_last]
        );
        assert_eq!(
            first.1,
            LegalRoutePolishMetrics {
                passes: 1,
                initial_candidates: 2,
                attempts: 3,
                wakeups: 1,
                improvements: 2,
                objective_reduction: 26,
            }
        );
        assert!(first.2.iter().all(|&occupancy| occupancy <= 1));
        assert!(first.3.iter().all(|&occupancy| occupancy <= 1));
    }

    #[test]
    #[allow(clippy::too_many_lines)]
    fn legal_route_polish_rechecks_siblings_after_growing_their_shared_tree() {
        let mut design = Design::new();
        let source = design.add_cell("source", ResourceKind::Logic);
        let output = design.add_pin(source, "O", PinDirection::Output).unwrap();
        let high_cell = design.add_cell("high", ResourceKind::Register);
        let high_sink = design.add_pin(high_cell, "I", PinDirection::Input).unwrap();
        let low_cell = design.add_cell("low", ResourceKind::Register);
        let low_sink = design.add_pin(low_cell, "I", PinDirection::Input).unwrap();
        let net = design
            .add_net("two-sink", output, [high_sink, low_sink])
            .unwrap();

        let mut device = Device::new("polish-siblings", 1, 1).unwrap();
        let point = Point::new(0, 0);
        let driver = device.add_wire("driver", point, 1).unwrap();
        let high_mid = device.add_wire("high-mid", point, 1).unwrap();
        let high_goal = device.add_wire("high-goal", point, 1).unwrap();
        let low_slow = device.add_wire("low-slow", point, 1).unwrap();
        let low_fast = device.add_wire("low-fast", point, 1).unwrap();
        let low_goal = device.add_wire("low-goal", point, 1).unwrap();
        let high_first = device.add_pip(driver, high_mid, false, 1).unwrap();
        let high_last = device.add_pip(high_mid, high_goal, false, 1).unwrap();
        let low_slow_first = device.add_pip(driver, low_slow, false, 1).unwrap();
        let low_slow_last = device.add_pip(low_slow, low_goal, false, 1).unwrap();
        let low_fast_first = device.add_pip(driver, low_fast, false, 1).unwrap();
        let low_fast_last = device.add_pip(low_fast, low_goal, false, 1).unwrap();

        let add_bel = |device: &mut Device,
                       name: &str,
                       kind: ResourceKind,
                       pin: &str,
                       direction: PinDirection,
                       wire: WireId| {
            let bel = device.add_bel(name, kind, point).unwrap();
            device.add_bel_pin(bel, pin, direction, wire).unwrap();
            bel
        };
        let source_bel = add_bel(
            &mut device,
            "source",
            ResourceKind::Logic,
            "O",
            PinDirection::Output,
            driver,
        );
        let high_bel = add_bel(
            &mut device,
            "high",
            ResourceKind::Register,
            "I",
            PinDirection::Input,
            high_goal,
        );
        let low_bel = add_bel(
            &mut device,
            "low",
            ResourceKind::Register,
            "I",
            PinDirection::Input,
            low_goal,
        );
        let placement = Placement {
            bindings: vec![source_bel, high_bel, low_bel],
            pin_bindings: BTreeMap::new(),
        };
        let route = Arc::new(NetRoute::new(
            net,
            vec![
                RouteArc {
                    sink: Some(high_sink),
                    wires: vec![driver, high_mid, high_goal],
                    pips: vec![high_first, high_last],
                },
                RouteArc {
                    sink: Some(low_sink),
                    wires: vec![driver, low_slow, low_goal],
                    pips: vec![low_slow_first, low_slow_last],
                },
            ],
        ));
        let mut costs = RoutingCosts::new(
            vec![500, 500, 300, 300, 50, 50],
            BTreeMap::from([(net, 64)]),
        );
        costs.set_sink_criticalities(BTreeMap::from([
            ((net, high_sink), 64),
            ((net, low_sink), 64),
        ]));
        let fixture = LegalPolishFixture {
            design,
            device,
            placement,
            costs,
            routes: vec![route],
            target_sink: low_sink,
            fast_pips: [low_fast_first, low_fast_last],
            occupied_detour: low_slow,
        };

        let (routes, metrics, wire_occupancy, pip_occupancy) = run_legal_polish(&fixture);
        assert_eq!(routes[0].arc(low_sink).unwrap().pips, fixture.fast_pips);
        assert_eq!(
            routes[0].arc(high_sink).unwrap().pips,
            [high_first, high_last]
        );
        assert_eq!(
            metrics,
            LegalRoutePolishMetrics {
                passes: 1,
                initial_candidates: 2,
                attempts: 3,
                wakeups: 1,
                improvements: 1,
                objective_reduction: 10,
            }
        );
        assert!(wire_occupancy.iter().all(|&occupancy| occupancy <= 1));
        assert!(pip_occupancy.iter().all(|&occupancy| occupancy <= 1));
    }

    #[test]
    #[allow(clippy::too_many_lines)]
    fn capacity_one_conflicts_release_every_unlocked_connection() {
        let mut design = Design::new();
        let mut endpoints = Vec::new();
        for name in ["a", "b", "c"] {
            let source = design.add_cell(format!("source_{name}"), ResourceKind::Logic);
            let output = design.add_pin(source, "O", PinDirection::Output).unwrap();
            let sink = design.add_cell(format!("sink_{name}"), ResourceKind::Register);
            let input = design.add_pin(sink, "I", PinDirection::Input).unwrap();
            let net = design.add_net(name, output, [input]).unwrap();
            endpoints.push((source, sink, input, net));
        }

        let mut device = Device::new("polish-mre", 1, 1).unwrap();
        let point = Point::new(0, 0);
        let mut endpoint_wires = Vec::new();
        let mut endpoint_bels = Vec::new();
        for (name, kind, direction, pin) in [
            ("source_a", ResourceKind::Logic, PinDirection::Output, "O"),
            ("sink_a", ResourceKind::Register, PinDirection::Input, "I"),
            ("source_b", ResourceKind::Logic, PinDirection::Output, "O"),
            ("sink_b", ResourceKind::Register, PinDirection::Input, "I"),
            ("source_c", ResourceKind::Logic, PinDirection::Output, "O"),
            ("sink_c", ResourceKind::Register, PinDirection::Input, "I"),
        ] {
            let wire = device.add_wire(name, point, 1).unwrap();
            let bel = device.add_bel(name, kind, point).unwrap();
            device.add_bel_pin(bel, pin, direction, wire).unwrap();
            endpoint_wires.push(wire);
            endpoint_bels.push(bel);
        }
        let [source_a, sink_a, source_b, sink_b, source_c, sink_c] = endpoint_wires.as_slice()
        else {
            unreachable!()
        };
        let x_entry = device.add_wire("x_entry", point, 1).unwrap();
        let x_exit = device.add_wire("x_exit", point, 1).unwrap();
        let z_entry = device.add_wire("z_entry", point, 1).unwrap();
        let z_exit = device.add_wire("z_exit", point, 1).unwrap();
        let a_alt = (0..4)
            .map(|index| device.add_wire(format!("a_alt_{index}"), point, 1).unwrap())
            .collect::<Vec<_>>();
        let b_alt = (0..2)
            .map(|index| device.add_wire(format!("b_alt_{index}"), point, 1).unwrap())
            .collect::<Vec<_>>();
        let mut delays = Vec::new();
        let add =
            |device: &mut Device, delays: &mut Vec<u32>, from: WireId, to: WireId, delay: u32| {
                let pip = device.add_pip(from, to, false, 1).unwrap();
                assert_eq!(pip.0, delays.len());
                delays.push(delay);
            };
        add(&mut device, &mut delays, *source_a, x_entry, 50); // 0
        add(&mut device, &mut delays, *source_b, x_entry, 50); // 1
        add(&mut device, &mut delays, x_entry, x_exit, 50); // 2
        add(&mut device, &mut delays, x_exit, z_entry, 50); // 3
        add(&mut device, &mut delays, *source_c, z_entry, 50); // 4
        add(&mut device, &mut delays, z_entry, z_exit, 50); // 5
        add(&mut device, &mut delays, z_exit, *sink_a, 50); // 6
        add(&mut device, &mut delays, x_exit, *sink_b, 50); // 7
        add(&mut device, &mut delays, z_exit, *sink_c, 50); // 8
        let mut previous = *source_a;
        for (index, &wire) in a_alt.iter().chain(std::iter::once(sink_a)).enumerate() {
            add(
                &mut device,
                &mut delays,
                previous,
                wire,
                if index == 0 { 800 } else { 50 },
            );
            previous = wire;
        }
        previous = *source_b;
        for (index, &wire) in b_alt.iter().chain(std::iter::once(sink_b)).enumerate() {
            add(
                &mut device,
                &mut delays,
                previous,
                wire,
                if index == 0 { 350 } else { 50 },
            );
            previous = wire;
        }

        let mut placement_constraints = PlacementConstraints::new();
        let cells = [
            endpoints[0].0,
            endpoints[0].1,
            endpoints[1].0,
            endpoints[1].1,
            endpoints[2].0,
            endpoints[2].1,
        ];
        for (&cell, &bel) in cells.iter().zip(&endpoint_bels) {
            placement_constraints.add_group([cell], [vec![bel]]);
        }
        let placement = place_with_constraints(&design, &device, &placement_constraints).unwrap();
        let mut costs = RoutingCosts::new(
            delays,
            BTreeMap::from([
                (endpoints[0].3, 32),
                (endpoints[1].3, 2),
                (endpoints[2].3, 64),
            ]),
        );
        costs.set_sink_criticalities(BTreeMap::from([
            ((endpoints[0].3, endpoints[0].2), 32),
            ((endpoints[1].3, endpoints[1].2), 2),
            ((endpoints[2].3, endpoints[2].2), 64),
        ]));
        let mut iterations = Vec::new();
        let routed = route_with_timing_costs_and_progress(
            &design,
            &device,
            placement,
            &RoutingConstraints::new(),
            &costs,
            |event| {
                if let RoutingProgress::Iteration { iteration, nets } = event {
                    iterations.push((iteration, nets));
                }
            },
        )
        .unwrap();
        let arc_pips = |net: usize| {
            routed.routes[net].arcs[0]
                .pips
                .iter()
                .map(|pip| pip.0)
                .collect::<Vec<_>>()
        };

        assert_eq!(iterations, [(0, 3), (1, 3), (2, 2)]);
        assert_eq!(arc_pips(0), [9, 10, 11, 12, 13]);
        assert_eq!(arc_pips(1), [14, 15, 16]);
        assert_eq!(arc_pips(2), [4, 5, 8]);
        assert_eq!(routed.total_pips, 11);
    }

    #[test]
    fn routing_cost_clones_share_immutable_pip_tables() {
        let mut costs = RoutingCosts::new(vec![10, 20], BTreeMap::new());
        costs.set_pip_min_delays_ps(vec![4, 8]);
        let clone = costs.clone();

        assert!(Arc::ptr_eq(&costs.pip_delays_ps, &clone.pip_delays_ps));
        assert!(Arc::ptr_eq(
            &costs.pip_min_delays_ps,
            &clone.pip_min_delays_ps
        ));
        assert_eq!(clone.pip_delays_ps(), [10, 20]);
        assert_eq!(clone.pip_min_delays_ps(), [4, 8]);
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
    fn coarse_analytical_seed_is_deterministic_and_legal() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(4, 4).unwrap();
        let constraints = PlacementConstraints::new();
        let refiner = PlacementRefiner::new(&design, &device, &constraints).unwrap();

        let first = refiner.place_analytically_coarse(&BTreeMap::new()).unwrap();
        let second = refiner.place_analytically_coarse(&BTreeMap::new()).unwrap();

        assert_eq!(first, second);
        assert_ne!(first.bindings()[0], first.bindings()[1]);
    }

    #[test]
    fn projected_mm_rejects_full_step_and_accepts_first_decreasing_half_step() {
        let incumbent = AnalyticalObjective {
            hpwl: 10,
            total: 10,
        };
        let mut evaluated = Vec::new();
        let descent = first_dyadic_strict_descent(incumbent, |alpha| {
            evaluated.push(alpha);
            let objective = if evaluated.len() == 1 {
                AnalyticalObjective {
                    hpwl: 12,
                    total: 12,
                }
            } else {
                AnalyticalObjective { hpwl: 9, total: 9 }
            };
            Ok::<_, Infallible>(Some((alpha, objective)))
        })
        .unwrap()
        .unwrap();

        assert_eq!(evaluated, [1.0, 0.5]);
        assert!((descent.alpha - 0.5).abs() <= f64::EPSILON);
        assert!((descent.value - 0.5).abs() <= f64::EPSILON);
        assert_eq!(descent.objective.total, 9);
    }

    #[test]
    fn projected_mm_non_improvement_reaches_rounded_fixed_point_without_a_cap() {
        let incumbent = AnalyticalObjective {
            hpwl: 10,
            total: 10,
        };
        let mut evaluated = Vec::new();
        let descent = first_dyadic_strict_descent(incumbent, |alpha| {
            evaluated.push(alpha);
            // Search from legal origin zero toward analytical coordinate two.
            // At alpha=1/8 the rounded target returns to the legal origin and
            // the projector is not called again.
            if rounded_coordinate(2.0 * alpha, 8) == 0 {
                return Ok::<_, Infallible>(None);
            }
            Ok(Some((
                alpha,
                AnalyticalObjective {
                    hpwl: 11,
                    total: 11,
                },
            )))
        })
        .unwrap();

        assert!(descent.is_none());
        assert_eq!(evaluated, [1.0, 0.5, 0.25, 0.125]);
    }

    fn floating_pair_axis_equations() -> AxisEquations {
        AxisEquations {
            diagonal: vec![1.0, 1.0],
            adjacency: vec![vec![(1, 1.0)], vec![(0, 1.0)]],
            rhs: vec![4.0, -4.0],
            anchored: vec![false, false],
        }
    }

    #[test]
    fn component_gauge_preserves_relative_optimum_and_centers_each_component() {
        let equations = AxisEquations {
            diagonal: vec![1.0, 1.0, 0.0],
            adjacency: vec![vec![(1, 1.0)], vec![(0, 1.0)], Vec::new()],
            rhs: vec![4.0, -4.0, 0.0],
            anchored: vec![false, false, false],
        };
        let mut inspected = equations.clone();
        let floating = inspected.finalize_component_gauges();
        assert_eq!(floating, [vec![0, 1], vec![2]]);
        for (unit, edges) in inspected.adjacency.iter().enumerate() {
            for &(other, weight) in edges {
                assert!(
                    inspected.adjacency[other]
                        .iter()
                        .any(|&(back, back_weight)| {
                            back == unit && (back_weight - weight).abs() <= f64::EPSILON
                        })
                );
            }
        }

        let solution = solve_analytical_axis(equations, vec![10.0; 3], 10).unwrap();
        assert!((solution[0] - 12.0).abs() < 1.0e-10);
        assert!((solution[1] - 8.0).abs() < 1.0e-10);
        assert!((solution[2] - 10.0).abs() < 1.0e-10);
        assert!((f64::midpoint(solution[0], solution[1]) - 10.0).abs() < 1.0e-10);
        let edge_objective = (solution[0] - solution[1] - 4.0).powi(2);
        let translated_objective = (solution[0] + 37.0 - (solution[1] + 37.0) - 4.0).powi(2);
        assert!(edge_objective < 1.0e-20);
        assert!((translated_objective - edge_objective).abs() < f64::EPSILON);
    }

    #[test]
    fn fixed_connected_component_uses_absolute_constraint_without_a_gauge() {
        // Unit zero is a fixed identity row at x=3. Unit one represents a
        // fixed-to-movable edge whose Dirichlet equation is 2*x=10.
        let equations = AxisEquations {
            diagonal: vec![1.0, 2.0],
            adjacency: vec![Vec::new(), Vec::new()],
            rhs: vec![3.0, 10.0],
            anchored: vec![true, true],
        };
        let mut inspected = equations.clone();
        assert!(inspected.finalize_component_gauges().is_empty());

        let solution = solve_analytical_axis(equations, vec![100.0; 2], 100).unwrap();
        assert!((solution[0] - 3.0).abs() < 1.0e-10);
        assert!((solution[1] - 5.0).abs() < 1.0e-10);
    }

    #[test]
    fn explicit_anchor_prevents_an_artificial_component_gauge() {
        let mut equations = floating_pair_axis_equations();
        equations.add_anchor(1, 2.0, 7.0);
        let mut inspected = equations.clone();
        assert!(inspected.finalize_component_gauges().is_empty());

        let solution = solve_analytical_axis(equations, vec![100.0; 2], 100).unwrap();
        assert!((solution[0] - 11.0).abs() < 1.0e-10);
        assert!((solution[1] - 7.0).abs() < 1.0e-10);
    }

    fn floating_path_axis_equations(dimension: usize, reverse_edges: bool) -> AxisEquations {
        let mut diagonal = vec![0.0; dimension];
        let mut adjacency = vec![Vec::new(); dimension];
        for left in 0..dimension - 1 {
            let right = left + 1;
            diagonal[left] += 1.0;
            diagonal[right] += 1.0;
            adjacency[left].push((right, 1.0));
            adjacency[right].push((left, 1.0));
        }
        if reverse_edges {
            for edges in &mut adjacency {
                edges.reverse();
            }
        }
        let expected = (0..dimension)
            .map(|index| f64::from(u32::try_from(index).unwrap()))
            .collect::<Vec<_>>();
        let rhs = (0..dimension)
            .map(|unit| {
                adjacency[unit]
                    .iter()
                    .fold(diagonal[unit] * expected[unit], |sum, &(other, weight)| {
                        sum - weight * expected[other]
                    })
            })
            .collect();
        AxisEquations {
            diagonal,
            adjacency,
            rhs,
            anchored: vec![false; dimension],
        }
    }

    #[test]
    fn long_floating_laplacian_converges_and_is_insertion_order_deterministic() {
        let dimension = 129;
        let forward = solve_analytical_axis(
            floating_path_axis_equations(dimension, false),
            vec![64.0; dimension],
            64,
        )
        .unwrap();
        let reverse = solve_analytical_axis(
            floating_path_axis_equations(dimension, true),
            vec![64.0; dimension],
            64,
        )
        .unwrap();

        for (index, (&forward, &reverse)) in forward.iter().zip(&reverse).enumerate() {
            let expected = f64::from(u32::try_from(index).unwrap());
            assert!((forward - expected).abs() < 1.0e-7);
            assert!((reverse - expected).abs() < 1.0e-7);
            assert!((forward - reverse).abs() < 1.0e-10);
        }
    }

    #[test]
    fn jacobi_pcg_is_not_truncated_at_the_legacy_hundred_iteration_budget() {
        let dimension = 128_usize;
        let denominator = f64::from(u32::try_from(dimension - 1).unwrap());
        let diagonal = (0..dimension)
            .map(|index| {
                let index = f64::from(u32::try_from(index).unwrap());
                10.0_f64.powf(-12.0 + 24.0 * index / denominator)
            })
            .collect::<Vec<_>>();
        let rhs = vec![1.0; dimension];
        let adjacency = vec![Vec::new(); dimension];
        let squared_norm = |values: &[f64]| values.iter().map(|value| value * value).sum::<f64>();

        // Reproduce the former raw-CG iteration budget on this diagonal SPD
        // system. Its 24-decade condition range leaves a material residual at
        // the arbitrary 100-step cutoff even though the system has 128
        // independent Krylov dimensions.
        let mut legacy_solution = vec![0.0; dimension];
        let mut legacy_residual = rhs.clone();
        let mut legacy_direction = legacy_residual.clone();
        let initial_squared = squared_norm(&legacy_residual);
        let mut residual_squared = initial_squared;
        for _ in 0..100 {
            let product = legacy_direction
                .iter()
                .zip(&diagonal)
                .map(|(&direction, &diagonal)| direction * diagonal)
                .collect::<Vec<_>>();
            let direction_product = legacy_direction
                .iter()
                .zip(&product)
                .map(|(&direction, &product)| direction * product)
                .sum::<f64>();
            if direction_product <= f64::EPSILON {
                break;
            }
            let alpha = residual_squared / direction_product;
            for ((solution, residual), (&direction, &product)) in legacy_solution
                .iter_mut()
                .zip(&mut legacy_residual)
                .zip(legacy_direction.iter().zip(&product))
            {
                *solution += alpha * direction;
                *residual -= alpha * product;
            }
            let next_squared = squared_norm(&legacy_residual);
            let beta = next_squared / residual_squared;
            for (direction, &residual) in legacy_direction.iter_mut().zip(&legacy_residual) {
                *direction = residual + beta * *direction;
            }
            residual_squared = next_squared;
        }
        let legacy_ratio = (residual_squared / initial_squared).sqrt();

        let solution = solve_quadratic(&diagonal, &adjacency, &rhs, vec![0.0; dimension]).unwrap();
        let final_residual = solution
            .iter()
            .zip(&diagonal)
            .map(|(&solution, &diagonal)| 1.0 - diagonal * solution)
            .collect::<Vec<_>>();
        let pcg_ratio = (squared_norm(&final_residual) / initial_squared).sqrt();

        assert!(legacy_ratio > 1.0e-4, "legacy ratio={legacy_ratio:e}");
        assert!(pcg_ratio < 1.0e-12, "PCG ratio={pcg_ratio:e}");
    }

    #[test]
    fn analytical_legalizer_uses_three_legal_logic_bels_at_one_point() {
        let mut design = Design::new();
        let cells = (0..3)
            .map(|index| design.add_cell(format!("cell{index}"), ResourceKind::Logic))
            .collect::<Vec<_>>();
        let mut device = Device::new("multi-logic-point", 1, 1).unwrap();
        let bels = (0..4)
            .map(|index| {
                device
                    .add_bel(
                        format!("logic{index}"),
                        ResourceKind::Logic,
                        Point::new(0, 0),
                    )
                    .unwrap()
            })
            .collect::<Vec<_>>();
        let mut constraints = PlacementConstraints::new();
        constraints.add_shared_resource(
            [(cells[0], 0), (cells[1], 0), (cells[2], 1)],
            [(bels[0], 0), (bels[1], 0), (bels[2], 1), (bels[3], 1)],
        );

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
        assert_eq!(
            first
                .bindings()
                .iter()
                .copied()
                .collect::<BTreeSet<_>>()
                .len(),
            3
        );
        assert!(
            first
                .bindings()
                .iter()
                .all(|bel| device.bels()[bel.0].point == Point::new(0, 0))
        );
    }

    struct ManhattanPlacementDelay<'a>(&'a Device);

    impl PlacementDelayEstimator for ManhattanPlacementDelay<'_> {
        fn estimate_delay_ps(
            &self,
            driver_bel: BelId,
            _driver_pin: BelPinId,
            sink_bel: BelId,
            _sink_pin: BelPinId,
        ) -> u64 {
            self.0.bels()[driver_bel.0]
                .point
                .manhattan(self.0.bels()[sink_bel.0].point)
        }
    }

    #[test]
    fn predicted_timing_refinement_reduces_normalized_arc_cost() {
        let design = two_cell_design();
        let sink = design.nets()[0].sinks[0];
        let device = Device::rectangular_logic(7, 1).unwrap();
        let constraints = PlacementConstraints::new();
        let initial = Placement {
            bindings: vec![BelId(0), BelId(6)],
            pin_bindings: BTreeMap::new(),
        };
        let detailed_placer = PlacementRefiner::new(&design, &device, &constraints).unwrap();
        let predictor = ManhattanPlacementDelay(&device);

        let refined = detailed_placer
            .refine_with_predicted_timing(
                initial.clone(),
                &BTreeMap::from([((NetId(0), sink), 64)]),
                &predictor,
            )
            .unwrap();
        let distance = |placement: &Placement| {
            device.bels()[placement.bindings()[0].0]
                .point
                .manhattan(device.bels()[placement.bindings()[1].0].point)
        };

        assert!(distance(&refined) < distance(&initial));
    }

    #[test]
    fn predicted_detail_uses_an_ordinary_incident_net_without_a_timing_overlay() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(15, 1).unwrap();
        let mut constraints = PlacementConstraints::new();
        constraints.add_group([CellId(0)], [vec![BelId(0)], vec![BelId(10)]]);
        constraints.add_group([CellId(1)], [vec![BelId(14)]]);
        let initial = Placement {
            bindings: vec![BelId(0), BelId(14)],
            pin_bindings: BTreeMap::new(),
        };
        let placement_refiner = PlacementRefiner::new(&design, &device, &constraints).unwrap();

        let result = placement_refiner
            .refine_with_predicted_timing(
                initial,
                &BTreeMap::new(),
                &ManhattanPlacementDelay(&device),
            )
            .unwrap();

        assert_eq!(result.bel(CellId(0)), Some(BelId(10)));
        assert_eq!(result.bel(CellId(1)), Some(BelId(14)));
    }

    #[test]
    fn predicted_detail_incident_target_retains_rigid_macro_offsets() {
        let mut design = Design::new();
        let anchor = add_equivalent_logic_cell(&mut design, "anchor");
        let connected = add_equivalent_logic_cell(&mut design, "connected");
        let fixed = add_equivalent_logic_cell(&mut design, "fixed");
        design
            .add_net("macro_to_fixed", connected.2, [fixed.1])
            .unwrap();
        let device = Device::rectangular_logic(10, 1).unwrap();
        let mut constraints = PlacementConstraints::new();
        constraints.add_group(
            [anchor.0, connected.0],
            [vec![BelId(0), BelId(2)], vec![BelId(5), BelId(7)]],
        );
        constraints.add_group([fixed.0], [vec![BelId(9)]]);
        let initial = Placement {
            bindings: vec![BelId(0), BelId(2), BelId(9)],
            pin_bindings: BTreeMap::new(),
        };
        let placement_refiner = PlacementRefiner::new(&design, &device, &constraints).unwrap();

        let result = placement_refiner
            .refine_with_predicted_timing(
                initial,
                &BTreeMap::new(),
                &ManhattanPlacementDelay(&device),
            )
            .unwrap();

        assert_eq!(result.bel(anchor.0), Some(BelId(5)));
        assert_eq!(result.bel(connected.0), Some(BelId(7)));
        assert_eq!(result.bel(fixed.0), Some(BelId(9)));
    }

    #[test]
    fn predicted_detail_jumps_across_a_local_basin_and_reaches_a_fixed_point() {
        let design = two_cell_design();
        let sink = design.nets()[0].sinks[0];
        let device = Device::rectangular_logic(15, 1).unwrap();
        let mut constraints = PlacementConstraints::new();
        constraints.add_group(
            [CellId(0)],
            [vec![BelId(0)], vec![BelId(1)], vec![BelId(10)]],
        );
        constraints.add_group([CellId(1)], [vec![BelId(14)]]);
        let initial = Placement {
            bindings: vec![BelId(0), BelId(14)],
            pin_bindings: BTreeMap::new(),
        };
        let criticalities = BTreeMap::from([((NetId(0), sink), 64)]);
        let predictor = ManhattanPlacementDelay(&device);
        let refiner = PlacementRefiner::new(&design, &device, &constraints).unwrap();

        let (one_sweep, moved) = refiner
            .refine_with_predicted_timing_pass(initial.clone(), &criticalities, &predictor)
            .unwrap();
        let (one_sweep_fixed, fixed_moved) = refiner
            .refine_with_predicted_timing_pass(one_sweep.clone(), &criticalities, &predictor)
            .unwrap();
        let first = refiner
            .refine_with_predicted_timing(initial.clone(), &criticalities, &predictor)
            .unwrap();
        let second = refiner
            .refine_with_predicted_timing(initial, &criticalities, &predictor)
            .unwrap();
        let fixed = refiner
            .refine_with_predicted_timing(first.clone(), &criticalities, &predictor)
            .unwrap();

        assert!(moved > 0, "the first sweep must report its accepted move");
        assert_eq!(fixed_moved, 0, "the fixed-point sweep reports no moves");
        assert_eq!(one_sweep_fixed, one_sweep);
        assert_eq!(one_sweep, first);
        assert_eq!(first, second, "far-target refinement is deterministic");
        assert_eq!(fixed, first, "a second run is already at the fixed point");
        assert_eq!(first.bel(CellId(0)), Some(BelId(10)));
        assert_eq!(first.bel(CellId(1)), Some(BelId(14)));
        assert_eq!(
            device.bels()[BelId(0).0]
                .point
                .manhattan(device.bels()[first.bel(CellId(0)).unwrap().0].point),
            10,
            "the canonical incident-net target crosses the basin in one candidate step"
        );
    }

    #[test]
    fn predicted_detail_skips_a_blocked_target_ring_for_the_next_legal_ring() {
        let design = two_cell_design();
        let sink = design.nets()[0].sinks[0];
        let device = Device::rectangular_logic(6, 1).unwrap();
        let mut constraints = PlacementConstraints::new();
        constraints.add_group(
            [CellId(0)],
            [
                vec![BelId(0)],
                vec![BelId(1)],
                vec![BelId(4)],
                vec![BelId(5)],
            ],
        );
        // The critical endpoint itself is the only assignment on target ring
        // zero, but it belongs to this fixed unit and cannot be swapped.
        constraints.add_group([CellId(1)], [vec![BelId(4)]]);
        let initial = Placement {
            bindings: vec![BelId(0), BelId(4)],
            pin_bindings: BTreeMap::new(),
        };
        let placer = PlacementRefiner::new(&design, &device, &constraints).unwrap();
        let predictor = ManhattanPlacementDelay(&device);

        let refined = placer
            .refine_with_predicted_timing(
                initial,
                &BTreeMap::from([((NetId(0), sink), 64)]),
                &predictor,
            )
            .unwrap();

        assert_eq!(refined.bel(CellId(0)), Some(BelId(5)));
        assert_eq!(refined.bel(CellId(1)), Some(BelId(4)));
    }

    #[test]
    fn detailed_timing_target_uses_the_criticality_weighted_median() {
        let points = BTreeMap::from([
            (Point::new(1, 9), 2_u128),
            (Point::new(7, 4), 8_u128),
            (Point::new(12, 1), 3_u128),
        ]);

        assert_eq!(
            super::weighted_median_coordinate(&points, |point| point.x),
            Some(7)
        );
        assert_eq!(
            super::weighted_median_coordinate(&points, |point| point.y),
            Some(4)
        );
    }

    #[test]
    fn predicted_detail_swaps_occupied_compatible_units() {
        let mut design = Design::new();
        let cells = (0..4)
            .map(|index| add_equivalent_logic_cell(&mut design, &format!("cell{index}")))
            .collect::<Vec<_>>();
        let first_net = design.add_net("first", cells[0].2, [cells[1].1]).unwrap();
        let second_net = design.add_net("second", cells[2].2, [cells[3].1]).unwrap();
        let device = Device::rectangular_logic(4, 1).unwrap();
        let constraints = PlacementConstraints::new();
        let initial = Placement {
            bindings: vec![BelId(0), BelId(3), BelId(1), BelId(2)],
            pin_bindings: BTreeMap::new(),
        };
        let predictor = ManhattanPlacementDelay(&device);
        let detailed_placer = PlacementRefiner::new(&design, &device, &constraints).unwrap();
        let criticalities = BTreeMap::from([
            ((first_net, cells[1].1), 64),
            ((second_net, cells[3].1), 64),
        ]);
        let total_span = |placement: &Placement| {
            [(cells[0].0, cells[1].0), (cells[2].0, cells[3].0)]
                .into_iter()
                .map(|(driver, sink)| {
                    let driver = device.bels()[placement.bel(driver).unwrap().0].point;
                    let sink = device.bels()[placement.bel(sink).unwrap().0].point;
                    driver.manhattan(sink)
                })
                .sum::<u64>()
        };

        let refined = detailed_placer
            .refine_with_predicted_timing(initial.clone(), &criticalities, &predictor)
            .unwrap();

        assert!(total_span(&refined) < total_span(&initial));
        assert_eq!(
            refined.bindings().iter().copied().collect::<BTreeSet<_>>(),
            BTreeSet::from([BelId(0), BelId(1), BelId(2), BelId(3)])
        );
        assert!(
            initial
                .bindings()
                .iter()
                .zip(refined.bindings())
                .filter(|(before, after)| before != after)
                .count()
                >= 2
        );
    }

    #[test]
    fn predicted_detail_swaps_atomic_groups_as_whole_units() {
        let mut design = Design::new();
        let cells = (0..6)
            .map(|index| add_equivalent_logic_cell(&mut design, &format!("cell{index}")))
            .collect::<Vec<_>>();
        let first_net = design.add_net("first", cells[0].2, [cells[5].1]).unwrap();
        let second_net = design.add_net("second", cells[2].2, [cells[4].1]).unwrap();
        let device = Device::rectangular_logic(6, 1).unwrap();
        let assignments: Arc<[Vec<BelId>]> =
            vec![vec![BelId(1), BelId(2)], vec![BelId(3), BelId(4)]].into();
        let mut constraints = PlacementConstraints::new();
        constraints
            .add_group_with_shared_assignments([cells[0].0, cells[1].0], Arc::clone(&assignments));
        constraints
            .add_group_with_shared_assignments([cells[2].0, cells[3].0], Arc::clone(&assignments));
        constraints.add_group([cells[4].0], [vec![BelId(0)]]);
        constraints.add_group([cells[5].0], [vec![BelId(5)]]);
        let initial = Placement {
            bindings: vec![BelId(1), BelId(2), BelId(3), BelId(4), BelId(0), BelId(5)],
            pin_bindings: BTreeMap::new(),
        };
        let predictor = ManhattanPlacementDelay(&device);
        let detailed_placer = PlacementRefiner::new(&design, &device, &constraints).unwrap();
        let criticalities = BTreeMap::from([
            ((first_net, cells[5].1), 64),
            ((second_net, cells[4].1), 64),
        ]);
        let total_span = |placement: &Placement| {
            [(cells[0].0, cells[5].0), (cells[2].0, cells[4].0)]
                .into_iter()
                .map(|(driver, sink)| {
                    let driver = device.bels()[placement.bel(driver).unwrap().0].point;
                    let sink = device.bels()[placement.bel(sink).unwrap().0].point;
                    driver.manhattan(sink)
                })
                .sum::<u64>()
        };

        let refined = detailed_placer
            .refine_with_predicted_timing(initial.clone(), &criticalities, &predictor)
            .unwrap();
        let first_assignment = vec![refined.bindings()[0], refined.bindings()[1]];
        let second_assignment = vec![refined.bindings()[2], refined.bindings()[3]];

        assert!(total_span(&refined) < total_span(&initial));
        assert!(assignments.contains(&first_assignment));
        assert!(assignments.contains(&second_assignment));
        assert_ne!(first_assignment, second_assignment);
    }

    #[test]
    fn predicted_detail_checks_both_sides_of_shared_resource_swap() {
        let mut design = Design::new();
        let cells = (0..3)
            .map(|index| add_equivalent_logic_cell(&mut design, &format!("cell{index}")))
            .collect::<Vec<_>>();
        let net = design
            .add_net("critical", cells[0].2, [cells[2].1])
            .unwrap();
        let device = Device::rectangular_logic(3, 1).unwrap();
        let mut constraints = PlacementConstraints::new();
        constraints.add_group([cells[2].0], [vec![BelId(2)]]);
        constraints.add_shared_resource(
            [(cells[0].0, 0), (cells[1].0, 1), (cells[2].0, 0)],
            [(BelId(0), 0), (BelId(1), 1), (BelId(2), 0)],
        );
        let initial = Placement {
            bindings: vec![BelId(0), BelId(1), BelId(2)],
            pin_bindings: BTreeMap::new(),
        };
        let predictor = ManhattanPlacementDelay(&device);
        let detailed_placer = PlacementRefiner::new(&design, &device, &constraints).unwrap();

        let refined = detailed_placer
            .refine_with_predicted_timing(
                initial.clone(),
                &BTreeMap::from([((net, cells[2].1), 64)]),
                &predictor,
            )
            .unwrap();

        assert_eq!(refined, initial);
    }

    #[test]
    fn anchored_analytical_placement_stays_near_the_legalized_incumbent() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(9, 1).unwrap();
        let mut constraints = PlacementConstraints::new();
        constraints.add_group([CellId(1)], [vec![BelId(8)]]);
        let refiner = PlacementRefiner::new(&design, &device, &constraints).unwrap();
        let anchor = Placement {
            bindings: vec![BelId(0), BelId(8)],
            pin_bindings: BTreeMap::new(),
        };

        let unanchored = refiner.place_analytically(&BTreeMap::new()).unwrap();
        let anchored = refiner
            .place_analytically_anchored(&BTreeMap::new(), &anchor, 100)
            .unwrap();
        let anchor_point = device.bels()[anchor.bindings()[0].0].point;
        let unanchored_point = device.bels()[unanchored.bindings()[0].0].point;
        let anchored_point = device.bels()[anchored.bindings()[0].0].point;

        assert!(
            anchored_point.manhattan(anchor_point) < unanchored_point.manhattan(anchor_point),
            "anchor={anchor_point:?}, anchored={anchored_point:?}, unanchored={unanchored_point:?}"
        );
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
    fn placement_workspace_reuses_validated_group_shapes() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(2, 1).unwrap();
        let assignments: Arc<[Vec<BelId>]> = vec![vec![BelId(0), BelId(1)]].into();
        let mut constraints = PlacementConstraints::new();
        constraints
            .add_group_with_shared_assignments([CellId(0), CellId(1)], Arc::clone(&assignments));
        let mut workspace = PlacementRefinementWorkspace::new();

        let first =
            PlacementRefiner::new_with_workspace(&design, &device, &constraints, &mut workspace)
                .unwrap();
        let first_placement = first.place_analytically(&BTreeMap::new()).unwrap();
        drop(first);
        assert_eq!(workspace.validated_group_shapes.len(), 1);

        let second =
            PlacementRefiner::new_with_workspace(&design, &device, &constraints, &mut workspace)
                .unwrap();
        let second_placement = second.place_analytically(&BTreeMap::new()).unwrap();

        assert_eq!(workspace.validated_group_shapes.len(), 1);
        assert_eq!(first_placement, second_placement);
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
    fn timing_route_estimate_includes_full_delay_and_hop_bias() {
        let source = Point::new(1, 1);
        let sink = Point::new(6, 1);
        let search = RouteSearch::new(0);

        assert_eq!(search.remaining_cost_estimate(source, sink, 0, 50), 5);
        assert_eq!(search.remaining_cost_estimate(source, sink, 32, 50), 15);
        assert_eq!(search.remaining_cost_estimate(source, sink, 64, 50), 12);
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
    fn fanout_weight_preserves_square_root_aggregate_influence() {
        assert_eq!(fanout_placement_weight(1), 64);
        assert_eq!(fanout_placement_weight(2), 32);
        assert_eq!(fanout_placement_weight(4), 32);
        assert_eq!(fanout_placement_weight(5), 21);
        assert_eq!(fanout_placement_weight(64), 8);
        assert_eq!(fanout_placement_weight(65), 7);
        assert_eq!(fanout_placement_weight(256), 4);
    }

    #[test]
    fn timing_critical_sink_survives_the_local_star_fanout_cutoff() {
        let mut design = Design::new();
        let source = design.add_cell("source", ResourceKind::Logic);
        let source_out = design.add_pin(source, "out", PinDirection::Output).unwrap();
        let mut sinks = Vec::new();
        for index in 0..=super::MAX_LOCAL_STAR_FANOUT {
            let sink = design.add_cell(format!("sink{index}"), ResourceKind::Logic);
            sinks.push((
                sink,
                design.add_pin(sink, "in", PinDirection::Input).unwrap(),
            ));
        }
        design
            .add_net("wide_fanout", source_out, sinks.iter().map(|&(_, pin)| pin))
            .unwrap();
        let critical = sinks[0];
        let secondary = sinks[1];

        let (_, neighbors) = placement_neighbors(
            &design,
            None,
            Some(&BTreeMap::from([
                ((NetId(0), critical.1), 64),
                ((NetId(0), secondary.1), 32),
            ])),
            None,
        );

        assert_eq!(neighbors[source.0].len(), 1);
        assert_eq!(neighbors[source.0][0].cell, critical.0);
        assert_eq!(neighbors[source.0][0].weight, 192);
        assert!(neighbors[source.0][0].timing_driven);
        assert!(neighbors[secondary.0.0].is_empty());
    }

    #[test]
    fn equal_priority_sinks_grow_the_route_tree_nearest_first() {
        let sinks = [CellPinId(7), CellPinId(3), CellPinId(5)];
        let distances =
            BTreeMap::from([(CellPinId(3), 20), (CellPinId(5), 10), (CellPinId(7), 30)]);

        let ordered = ordered_sinks(NetId(0), &sinks, None, |sink| distances[&sink]);

        assert_eq!(ordered, [CellPinId(5), CellPinId(3), CellPinId(7)]);
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
    fn inactive_refinement_limits_produce_the_same_placement() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(5, 1).unwrap();
        let constraints = PlacementConstraints::new();
        let initial = Placement {
            bindings: vec![BelId(0), BelId(4)],
            pin_bindings: BTreeMap::new(),
        };
        let refiner = PlacementRefiner::new(&design, &device, &constraints).unwrap();
        let (broad, move_peak) = refiner
            .refine_with_net_sink_weights_limited_and_move_peak(
                initial.clone(),
                &BTreeMap::new(),
                None,
                10,
            )
            .unwrap();

        assert!(move_peak < 10);
        let equivalent = refiner
            .refine_with_net_sink_weights_limited(initial, &BTreeMap::new(), None, move_peak + 1)
            .unwrap();
        assert_eq!(broad, equivalent);
    }

    #[test]
    fn timing_routing_prices_full_delay_and_congestion() {
        assert_eq!(routing_step_cost(1_000, 0, 2, 50), 3);
        assert_eq!(routing_step_cost(200, 64, 0, 50), 4);
        assert_eq!(routing_step_cost(200, 32, 0, 50), 4);
        assert_eq!(routing_step_cost(200, 64, 2, 50), 6);
        assert_eq!(timing_tree_cost(200, 64, 50), 4);
        assert_eq!(timing_tree_cost(200, 1, 50), 4);
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
    fn alternate_source_estimate_is_explicit_and_positive() {
        let mut costs = RoutingCosts::new(Vec::new(), BTreeMap::new());
        assert_eq!(costs.alternate_source_delay_per_tile_ps(), None);
        costs.set_alternate_source_delay_per_tile_ps(33);
        assert_eq!(costs.alternate_source_delay_per_tile_ps(), Some(33));
        costs.set_alternate_source_delay_per_tile_ps(0);
        assert_eq!(costs.alternate_source_delay_per_tile_ps(), None);
    }

    #[test]
    fn cumulative_quantization_does_not_round_every_pip() {
        assert_eq!(routing_transition_cost(0, 24, 64, 0, 50), 1);
        assert_eq!(routing_transition_cost(24, 48, 64, 0, 50), 0);
        assert_eq!(routing_transition_cost(48, 72, 64, 0, 10), 3);
        assert_eq!(routing_transition_cost(48, 72, 64, 0, 1), 24);
    }

    #[test]
    fn route_frontier_entry_stays_compact() {
        assert_eq!(std::mem::size_of::<RouteQueueEntry>(), 16);
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
                None,
                goal,
                &[0; 3],
                &[0; 2],
                &[],
                None,
                None,
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
    fn low_criticality_does_not_discount_a_late_tree_branch() {
        let design = Design::new();
        let mut device = Device::new("late-tree", 8, 1).unwrap();
        let driver = device.add_wire("driver", Point::new(0, 0), 1).unwrap();
        let mut direct_wires = vec![driver];
        for x in 1..7 {
            direct_wires.push(
                device
                    .add_wire(format!("direct-{x}"), Point::new(x, 0), 1)
                    .unwrap(),
            );
        }
        let goal = device.add_wire("goal", Point::new(7, 0), 1).unwrap();
        let late_tree = device
            .add_wire("late-tree-source", Point::new(7, 0), 1)
            .unwrap();
        let mut direct_pips = Vec::new();
        for pair in direct_wires.windows(2) {
            direct_pips.push(device.add_pip(pair[0], pair[1], false, 1).unwrap());
        }
        direct_pips.push(
            device
                .add_pip(*direct_wires.last().unwrap(), goal, false, 1)
                .unwrap(),
        );
        device.add_pip(late_tree, goal, false, 1).unwrap();
        let graph = UnifiedGraph::new(&design, &device);
        let costs = RoutingCosts::new(vec![50; device.pips().len()], BTreeMap::new());
        let mut search = RouteSearch::new(device.wires().len());
        let starts = BTreeSet::from([driver, late_tree]);
        let mut tree_delays = vec![u64::MAX; device.wires().len()];
        tree_delays[driver.0] = 0;
        tree_delays[late_tree.0] = 5_000;
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

        let (wires, pips) = search
            .shortest_path(
                &graph,
                &starts,
                None,
                goal,
                &vec![0; device.wires().len()],
                &vec![0; device.pips().len()],
                &[],
                None,
                None,
                Some(&costs),
                1,
                50,
                &tree_delays,
                0,
                metadata,
            )
            .unwrap();

        assert_eq!(wires.last(), Some(&driver));
        assert_eq!(pips.len(), direct_pips.len());
    }

    #[test]
    fn timing_corridor_does_not_hide_the_early_tree_source() {
        let design = Design::new();
        let mut device = Device::new("tree-corridor", 21, 11).unwrap();
        let driver = device.add_wire("driver", Point::new(0, 0), 1).unwrap();
        let direct = device.add_wire("direct", Point::new(10, 0), 1).unwrap();
        let goal = device.add_wire("goal", Point::new(20, 0), 1).unwrap();
        let late_tree = device.add_wire("late-tree", Point::new(20, 5), 1).unwrap();
        let direct_first = device.add_pip(driver, direct, false, 1).unwrap();
        let direct_last = device.add_pip(direct, goal, false, 1).unwrap();
        device.add_pip(late_tree, goal, false, 1).unwrap();
        let graph = UnifiedGraph::new(&design, &device);
        let costs = RoutingCosts::new(vec![100, 100, 1_000], BTreeMap::new());
        let mut search = RouteSearch::new(device.wires().len());
        let starts = BTreeSet::from([driver, late_tree]);
        let mut tree_delays = vec![u64::MAX; device.wires().len()];
        tree_delays[driver.0] = 0;
        tree_delays[late_tree.0] = 5_000;
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

        let (wires, pips) = search
            .shortest_path(
                &graph,
                &starts,
                None,
                goal,
                &[0; 4],
                &[0; 3],
                &[],
                None,
                None,
                Some(&costs),
                64,
                50,
                &tree_delays,
                0,
                metadata,
            )
            .unwrap();

        assert_eq!(wires.last(), Some(&driver));
        assert_eq!(pips, vec![direct_last, direct_first]);
    }

    #[test]
    fn single_source_recovery_does_not_reenter_the_retained_tree() {
        let design = Design::new();
        let mut device = Device::new("retained-tree", 3, 2).unwrap();
        let start = device.add_wire("start", Point::new(0, 0), 1).unwrap();
        let retained = device
            .add_wire("retained-tree", Point::new(1, 0), 1)
            .unwrap();
        let detour = device
            .add_wire("legal-detour", Point::new(1, 1), 1)
            .unwrap();
        let goal = device.add_wire("goal", Point::new(2, 0), 1).unwrap();
        device.add_pip(start, retained, false, 1).unwrap();
        device.add_pip(retained, goal, false, 1).unwrap();
        let detour_first = device.add_pip(start, detour, false, 1).unwrap();
        let detour_last = device.add_pip(detour, goal, false, 1).unwrap();
        let graph = UnifiedGraph::new(&design, &device);
        let costs = RoutingCosts::new(vec![10; 4], BTreeMap::new());
        let mut search = RouteSearch::new(device.wires().len());
        let starts = BTreeSet::from([start]);
        let retained_tree = BTreeSet::from([start, retained]);
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
        let mut tree_delays = vec![u64::MAX; device.wires().len()];
        tree_delays[start.0] = 0;
        tree_delays[retained.0] = 10;

        let (wires, pips) = search
            .shortest_path(
                &graph,
                &starts,
                Some(&retained_tree),
                goal,
                &[0; 4],
                &[0; 4],
                &[],
                None,
                None,
                Some(&costs),
                64,
                50,
                &tree_delays,
                0,
                metadata,
            )
            .unwrap();

        assert_eq!(wires, vec![goal, detour, start]);
        assert_eq!(pips, vec![detour_last, detour_first]);
    }

    #[test]
    fn routing_avoids_target_blocked_pips() {
        let design = Design::new();
        let mut device = Device::new("blocked-pip", 1, 1).unwrap();
        let start = device.add_wire("start", Point::new(0, 0), 1).unwrap();
        let direct = device.add_wire("direct", Point::new(0, 0), 1).unwrap();
        let detour = device.add_wire("detour", Point::new(0, 0), 1).unwrap();
        let goal = device.add_wire("goal", Point::new(0, 0), 1).unwrap();
        let blocked = device.add_pip(start, direct, false, 1).unwrap();
        device.add_pip(direct, goal, false, 1).unwrap();
        let detour_first = device.add_pip(start, detour, false, 1).unwrap();
        let detour_last = device.add_pip(detour, goal, false, 1).unwrap();
        let graph = UnifiedGraph::new(&design, &device);
        let mut search = RouteSearch::new(device.wires().len());
        let mut empty_constraints = RoutingConstraints::new();
        empty_constraints.block_pips(std::iter::empty());
        assert_eq!(empty_constraints, RoutingConstraints::new());
        assert!(empty_constraints.blocked_pip_words.is_none());

        let mut constraints = RoutingConstraints::new();
        constraints.block_pips([blocked]);
        let mut cloned_constraints = constraints.clone();
        cloned_constraints.block_pips([detour_first]);
        assert_eq!(
            cloned_constraints.blocked_pips(),
            &BTreeSet::from([blocked, detour_first])
        );
        assert!(pip_is_blocked(
            cloned_constraints.blocked_pip_words(),
            blocked
        ));
        assert!(pip_is_blocked(
            cloned_constraints.blocked_pip_words(),
            detour_first
        ));
        assert_eq!(constraints.blocked_pips(), &BTreeSet::from([blocked]));
        assert!(pip_is_blocked(constraints.blocked_pip_words(), blocked));
        assert!(!pip_is_blocked(
            constraints.blocked_pip_words(),
            detour_first
        ));
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
                None,
                goal,
                &[0; 4],
                &[0; 4],
                constraints.blocked_pip_words(),
                None,
                None,
                None,
                0,
                50,
                &[0; 4],
                0,
                metadata,
            )
            .unwrap();

        assert_eq!(pips, vec![detour_last, detour_first]);
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
                None,
                goal,
                &[0; 4],
                &[0; 4],
                &[],
                None,
                None,
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
    fn shared_assignment_groups_reuse_the_complete_row_index() {
        let rows: Arc<[Vec<BelId>]> = vec![
            vec![BelId(0), BelId(1)],
            vec![BelId(0), BelId(2)],
            vec![BelId(3), BelId(4)],
        ]
        .into();
        let mut constraints = PlacementConstraints::new();
        constraints.add_group_with_shared_assignments([CellId(0), CellId(1)], Arc::clone(&rows));
        constraints.add_group_with_shared_assignments([CellId(2), CellId(3)], rows);

        assert!(Arc::ptr_eq(
            &constraints.group_row_indexes[0],
            &constraints.group_row_indexes[1]
        ));
        assert_eq!(constraints.group_row_indexes[0][&BelId(0)], vec![0, 1]);
        assert_eq!(constraints.group_row_indexes[0][&BelId(3)], vec![2]);
    }

    #[test]
    fn replaces_and_shrinks_atomic_group_columns_transactionally() {
        let mut constraints = PlacementConstraints::new();
        constraints.add_group(
            [CellId(0), CellId(1)],
            [vec![BelId(0), BelId(1)], vec![BelId(2), BelId(3)]],
        );

        assert!(constraints.replace_group(
            &[CellId(0), CellId(1)],
            [CellId(0), CellId(1), CellId(2)],
            [
                vec![BelId(0), BelId(1), BelId(4)],
                vec![BelId(2), BelId(3), BelId(5)],
            ],
        ));
        assert!(!constraints.replace_group(
            &[CellId(0), CellId(1), CellId(2)],
            [CellId(0), CellId(0)],
            [vec![BelId(0), BelId(1)]],
        ));
        assert_eq!(
            constraints.groups()[0].cells,
            [CellId(0), CellId(1), CellId(2)]
        );

        assert!(constraints.remove_group_cell(CellId(2)));
        assert_eq!(constraints.groups()[0].cells, [CellId(0), CellId(1)]);
        assert_eq!(
            constraints.groups()[0].assignments.as_ref(),
            [vec![BelId(0), BelId(1)], vec![BelId(2), BelId(3)]]
        );
        assert!(constraints.remove_group_cell(CellId(1)));
        assert!(constraints.groups().is_empty());
    }

    #[test]
    fn separates_incompatible_values_of_a_shared_site_resource() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(4, 1).unwrap();
        let mut constraints = PlacementConstraints::new();
        constraints.add_shared_resource(
            [(CellId(0), 0), (CellId(1), 1)],
            [(BelId(0), 0), (BelId(1), 0), (BelId(2), 1), (BelId(3), 1)],
        );

        let placement = place_with_constraints(&design, &device, &constraints).unwrap();

        assert_ne!(placement.bindings()[0].0 / 2, placement.bindings()[1].0 / 2);
    }

    #[test]
    fn permits_equal_values_of_a_shared_site_resource() {
        let design = two_cell_design();
        let device = Device::rectangular_logic(2, 1).unwrap();
        let mut constraints = PlacementConstraints::new();
        constraints.add_shared_resource(
            [(CellId(0), 7), (CellId(1), 7)],
            [(BelId(0), 0), (BelId(1), 0)],
        );
        let bindings = BTreeMap::from([(CellId(0), BelId(0)), (CellId(1), BelId(1))]);

        let placement =
            placement_from_partial_bindings(&design, &device, &constraints, &bindings).unwrap();

        assert_eq!(placement.bindings(), &[BelId(0), BelId(1)]);
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
    #[allow(clippy::too_many_lines)]
    fn stalled_victim_reroutes_the_incumbent_with_an_escape_path() {
        let mut design = Design::new();
        let movable_source = design.add_cell("source_a", ResourceKind::Logic);
        let movable_output = design
            .add_pin(movable_source, "O", PinDirection::Output)
            .unwrap();
        let movable_sink = design.add_cell("sink_a", ResourceKind::Register);
        let movable_input = design
            .add_pin(movable_sink, "I", PinDirection::Input)
            .unwrap();
        let bottlenecked_source = design.add_cell("source_b", ResourceKind::Logic);
        let bottlenecked_output = design
            .add_pin(bottlenecked_source, "O", PinDirection::Output)
            .unwrap();
        let bottlenecked_sink = design.add_cell("sink_b", ResourceKind::Register);
        let bottlenecked_input = design
            .add_pin(bottlenecked_sink, "I", PinDirection::Input)
            .unwrap();
        design
            .add_net("movable", movable_output, [movable_input])
            .unwrap();
        design
            .add_net("bottlenecked", bottlenecked_output, [bottlenecked_input])
            .unwrap();

        let mut device = Device::new("stall-escape", 7, 1).unwrap();
        let movable_source_wire = device.add_wire("source_a", Point::new(0, 0), 1).unwrap();
        let bottlenecked_source_wire = device.add_wire("source_b", Point::new(1, 0), 1).unwrap();
        let shared = device.add_wire("shared", Point::new(2, 0), 1).unwrap();
        let alternate_a = device.add_wire("alternate_a", Point::new(3, 0), 1).unwrap();
        let alternate_b = device.add_wire("alternate_b", Point::new(4, 0), 1).unwrap();
        let movable_sink_wire = device.add_wire("sink_a", Point::new(5, 0), 1).unwrap();
        let bottlenecked_sink_wire = device.add_wire("sink_b", Point::new(6, 0), 1).unwrap();

        let movable_source_bel = device
            .add_bel("source_a", ResourceKind::Logic, Point::new(0, 0))
            .unwrap();
        device
            .add_bel_pin(
                movable_source_bel,
                "O",
                PinDirection::Output,
                movable_source_wire,
            )
            .unwrap();
        let movable_sink_bel = device
            .add_bel("sink_a", ResourceKind::Register, Point::new(5, 0))
            .unwrap();
        device
            .add_bel_pin(
                movable_sink_bel,
                "I",
                PinDirection::Input,
                movable_sink_wire,
            )
            .unwrap();
        let bottlenecked_source_bel = device
            .add_bel("source_b", ResourceKind::Logic, Point::new(1, 0))
            .unwrap();
        device
            .add_bel_pin(
                bottlenecked_source_bel,
                "O",
                PinDirection::Output,
                bottlenecked_source_wire,
            )
            .unwrap();
        let bottlenecked_sink_bel = device
            .add_bel("sink_b", ResourceKind::Register, Point::new(6, 0))
            .unwrap();
        device
            .add_bel_pin(
                bottlenecked_sink_bel,
                "I",
                PinDirection::Input,
                bottlenecked_sink_wire,
            )
            .unwrap();

        device
            .add_pip(movable_source_wire, shared, false, 1)
            .unwrap();
        device.add_pip(shared, movable_sink_wire, false, 1).unwrap();
        device
            .add_pip(movable_source_wire, alternate_a, false, 1)
            .unwrap();
        device.add_pip(alternate_a, alternate_b, false, 1).unwrap();
        device
            .add_pip(alternate_b, movable_sink_wire, false, 1)
            .unwrap();
        device
            .add_pip(bottlenecked_source_wire, shared, false, 1)
            .unwrap();
        device
            .add_pip(shared, bottlenecked_sink_wire, false, 1)
            .unwrap();

        let bindings = BTreeMap::from([
            (movable_source, movable_source_bel),
            (movable_sink, movable_sink_bel),
            (bottlenecked_source, bottlenecked_source_bel),
            (bottlenecked_sink, bottlenecked_sink_bel),
        ]);
        let placement = placement_from_partial_bindings(
            &design,
            &device,
            &PlacementConstraints::new(),
            &bindings,
        )
        .unwrap();
        let routed = route_with_placement_and_progress(
            &design,
            &device,
            placement,
            &RoutingConstraints::new(),
            |_| {},
        )
        .unwrap();

        assert!(!routed.routes[0].wires().any(|wire| wire == shared));
        assert!(routed.routes[1].wires().any(|wire| wire == shared));
    }

    #[test]
    fn capacity_one_conflict_releases_all_unlocked_connections() {
        let critical_sink = texo_model::CellPinId(0);
        let conflicting_sink = texo_model::CellPinId(1);
        let retained_sink = texo_model::CellPinId(2);
        let shared = WireId(1);
        let routes = vec![
            Some(Arc::new(NetRoute::new(
                NetId(0),
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
            ))),
            Some(Arc::new(NetRoute::new(
                NetId(1),
                vec![RouteArc {
                    sink: Some(critical_sink),
                    wires: vec![WireId(0), shared, WireId(2)],
                    pips: vec![],
                }],
            ))),
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
            ((NetId(0), conflicting_sink), 1),
            ((NetId(0), retained_sink), 1),
            ((NetId(1), critical_sink), 64),
        ]));

        let dirty = congested_route_arcs(
            metadata,
            &routes,
            &RoutingConstraints::new(),
            Some(&costs),
            &[1, 2, 1, 1, 1, 1, 1],
            &[],
            &mut ConnectionOwnerScratch::default(),
        );
        let mut resource_index = ResourceOwnerIndex::default();
        resource_index.prepare(&routes, metadata);
        let indexed_dirty = congested_route_arcs_indexed(
            metadata,
            &routes,
            &RoutingConstraints::new(),
            Some(&costs),
            &mut resource_index,
            &mut ConnectionOwnerScratch::default(),
        );

        assert_eq!(
            dirty,
            BTreeMap::from([
                (NetId(0).0, BTreeSet::from([conflicting_sink])),
                (NetId(1).0, BTreeSet::from([critical_sink])),
            ])
        );
        assert_eq!(indexed_dirty, dirty);
        resource_index.resolve_conflicts(&routes, &indexed_dirty);
        assert_eq!(resource_index.wire_owners.get(&shared), None);
        assert!(!dirty[&NetId(0).0].contains(&retained_sink));

        let mut locked = RoutingConstraints::new();
        locked.add_route(routes[1].as_ref().unwrap().clone());
        let locked_dirty = congested_route_arcs(
            metadata,
            &routes,
            &locked,
            Some(&costs),
            &[1, 2, 1, 1, 1, 1, 1],
            &[],
            &mut ConnectionOwnerScratch::default(),
        );
        assert_eq!(
            locked_dirty,
            BTreeMap::from([(NetId(0).0, BTreeSet::from([conflicting_sink]))]),
            "a route constraint, unlike criticality, remains immovable",
        );
    }

    #[test]
    fn exact_conflict_cycle_releases_only_the_recurrent_component() {
        let prefix_sink = CellPinId(0);
        let first_sink = CellPinId(1);
        let second_sink = CellPinId(2);
        let empty = BTreeSet::new();
        let mut detector = RoutingConflictCycleDetector::default();

        assert_eq!(
            detector.observe(
                (2, 2),
                &BTreeSet::from([90]),
                &empty,
                &BTreeMap::from([(9, BTreeSet::from([prefix_sink]))]),
            ),
            None,
        );
        let first_dirty = BTreeMap::from([(1, BTreeSet::from([first_sink]))]);
        assert_eq!(
            detector.observe((1, 1), &BTreeSet::from([10]), &empty, &first_dirty,),
            None,
        );
        assert_eq!(
            detector.observe(
                (1, 1),
                &BTreeSet::from([11]),
                &empty,
                &BTreeMap::from([(2, BTreeSet::from([second_sink]))]),
            ),
            None,
        );

        let cycle = detector
            .observe((1, 1), &BTreeSet::from([10]), &empty, &first_dirty)
            .expect("the exact recurrent state closes the two-step cycle");
        assert_eq!(cycle.length, 2);
        assert_eq!(
            cycle.connections,
            BTreeMap::from([
                (1, BTreeSet::from([first_sink])),
                (2, BTreeSet::from([second_sink])),
            ])
        );
        assert!(!cycle.connections.contains_key(&9));

        assert_eq!(
            detector.observe((1, 1), &BTreeSet::from([10]), &empty, &first_dirty,),
            None,
            "an escape starts a fresh cycle epoch",
        );
    }

    #[test]
    fn cycle_component_routes_first_in_the_opposite_stable_order() {
        let key = |index| (false, Reverse(64), Reverse(false), Reverse(1), index);
        let mut order = (0..4).map(|index| (key(index), index)).collect::<Vec<_>>();

        prioritize_cycle_connections(&mut order, &BTreeSet::from([0, 2]));

        assert_eq!(
            order
                .into_iter()
                .map(|(_, index)| index)
                .collect::<Vec<_>>(),
            vec![2, 0, 1, 3],
        );
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
        let projection = RouteCapacityProjection::new(&[Arc::new(route)], &costs);

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
        assert_eq!(
            projected_release_scope_penalty(projection.wire_owners.get(&low_only), moving, 1),
            25
        );
        assert_eq!(
            projected_release_scope_penalty(projection.wire_owners.get(&shared), moving, 1),
            50
        );
    }

    #[test]
    #[allow(clippy::similar_names, clippy::too_many_lines)]
    fn projected_macro_cost_observes_every_external_member_route() {
        let mut design = Design::new();
        let macro_a = design.add_cell("macro-a", ResourceKind::Logic);
        let macro_a_out = design.add_pin(macro_a, "O", PinDirection::Output).unwrap();
        let macro_b = design.add_cell("macro-b", ResourceKind::Logic);
        let macro_b_out = design.add_pin(macro_b, "O", PinDirection::Output).unwrap();
        let sink_a = design.add_cell("sink-a", ResourceKind::Register);
        let sink_a_in = design.add_pin(sink_a, "I", PinDirection::Input).unwrap();
        let sink_b = design.add_cell("sink-b", ResourceKind::Register);
        let sink_b_in = design.add_pin(sink_b, "I", PinDirection::Input).unwrap();
        design
            .add_net("macro-a-net", macro_a_out, [sink_a_in])
            .unwrap();
        design
            .add_net("macro-b-net", macro_b_out, [sink_b_in])
            .unwrap();

        let mut device = Device::new("macro-sibling-projection", 5, 1).unwrap();
        let a0_wire = device.add_wire("a0", Point::new(0, 0), 1).unwrap();
        let a1_wire = device.add_wire("a1", Point::new(4, 0), 1).unwrap();
        let a0_mid = device.add_wire("a0-mid", Point::new(1, 0), 1).unwrap();
        let a1_mid = device.add_wire("a1-mid", Point::new(3, 0), 1).unwrap();
        let a_goal = device.add_wire("a-goal", Point::new(2, 0), 1).unwrap();
        let b0_wire = device.add_wire("b0", Point::new(0, 0), 1).unwrap();
        let b1_wire = device.add_wire("b1", Point::new(4, 0), 1).unwrap();
        let occupied_mid = device
            .add_wire("b0-occupied-mid", Point::new(1, 0), 1)
            .unwrap();
        let free_mid = device.add_wire("b1-free-mid", Point::new(3, 0), 1).unwrap();
        let b_goal = device.add_wire("b-goal", Point::new(2, 0), 1).unwrap();
        for (from, to) in [
            (a0_wire, a0_mid),
            (a0_mid, a_goal),
            (a1_wire, a1_mid),
            (a1_mid, a_goal),
            (b0_wire, occupied_mid),
            (occupied_mid, b_goal),
            (b1_wire, free_mid),
            (free_mid, b_goal),
        ] {
            device.add_pip(from, to, false, 1).unwrap();
        }
        let add_bel_pin = |device: &mut Device,
                           name: &str,
                           kind: ResourceKind,
                           point: Point,
                           pin_name: &str,
                           direction: PinDirection,
                           wire: WireId| {
            let bel = device.add_bel(name, kind, point).unwrap();
            device.add_bel_pin(bel, pin_name, direction, wire).unwrap();
            bel
        };
        let a0 = add_bel_pin(
            &mut device,
            "a0-bel",
            ResourceKind::Logic,
            Point::new(0, 0),
            "O",
            PinDirection::Output,
            a0_wire,
        );
        let b0 = add_bel_pin(
            &mut device,
            "b0-bel",
            ResourceKind::Logic,
            Point::new(0, 0),
            "O",
            PinDirection::Output,
            b0_wire,
        );
        let a1 = add_bel_pin(
            &mut device,
            "a1-bel",
            ResourceKind::Logic,
            Point::new(4, 0),
            "O",
            PinDirection::Output,
            a1_wire,
        );
        let b1 = add_bel_pin(
            &mut device,
            "b1-bel",
            ResourceKind::Logic,
            Point::new(4, 0),
            "O",
            PinDirection::Output,
            b1_wire,
        );
        let sink_a_bel = add_bel_pin(
            &mut device,
            "sink-a-bel",
            ResourceKind::Register,
            Point::new(2, 0),
            "I",
            PinDirection::Input,
            a_goal,
        );
        let sink_b_bel = add_bel_pin(
            &mut device,
            "sink-b-bel",
            ResourceKind::Register,
            Point::new(2, 0),
            "I",
            PinDirection::Input,
            b_goal,
        );
        let assignments: Arc<[Vec<BelId>]> = vec![vec![a0, b0], vec![a1, b1]].into();
        let unit = super::PlacementUnit {
            cells: vec![macro_a, macro_b],
            choices: super::PlacementChoices::Shared(assignments),
        };
        let placed = vec![Some(a0), Some(b0), Some(sink_a_bel), Some(sink_b_bel)];
        let graph = UnifiedGraph::new(&design, &device);
        let constraints = PlacementConstraints::new();
        let owner = NetId(2);
        let owner_route = NetRoute::new(
            owner,
            vec![RouteArc {
                sink: None,
                wires: vec![occupied_mid],
                pips: Vec::new(),
            }],
        );
        let projection = RouteCapacityProjection::new(
            &[Arc::new(owner_route)],
            &RoutingCosts::new(vec![5; 8], BTreeMap::from([(owner, 64)])),
        );
        let retained = BTreeMap::new();
        let score = |assignment: &[BelId], connections: &[_]| {
            super::assignment_connection_projected_cost(
                &graph,
                &constraints,
                &unit,
                assignment,
                connections,
                &placed,
                &[5; 8],
                &projection,
                &retained,
            )
            .unwrap()
        };
        let row0 = unit.choices.assignment(0);
        let row1 = unit.choices.assignment(1);
        let selected_connection = [(macro_a_out, sink_a_in)];
        let sibling_connection = [(macro_b_out, sink_b_in)];
        let external_connections = super::external_unit_connections(&design, &unit);

        assert_eq!(score(row0, &selected_connection), 10);
        assert_eq!(score(row1, &selected_connection), 10);
        assert_eq!(
            (
                score(row0, &sibling_connection),
                score(row1, &sibling_connection),
            ),
            (825, 10),
            "the sibling route sees the occupied resource"
        );
        assert_eq!(
            external_connections,
            vec![selected_connection[0], sibling_connection[0]],
        );
        assert_eq!(
            (
                score(row0, &external_connections),
                score(row1, &external_connections)
            ),
            (835, 20),
            "a rigid macro candidate must be ranked by all of its external connections"
        );
    }

    #[test]
    fn projected_connection_grows_from_the_retained_route_tree() {
        let design = Design::new();
        let mut device = Device::new("projected-tree", 1, 1).unwrap();
        let driver = device.add_wire("driver", Point::new(0, 0), 1).unwrap();
        let retained = device.add_wire("retained", Point::new(0, 0), 1).unwrap();
        let goal = device.add_wire("goal", Point::new(0, 0), 1).unwrap();
        device.add_pip(driver, goal, false, 1).unwrap();
        device.add_pip(retained, goal, false, 1).unwrap();
        let graph = UnifiedGraph::new(&design, &device);

        assert_eq!(
            local_connection_projected_cost_from_starts(
                &graph,
                &[driver, retained],
                goal,
                &[100, 10],
                NetId(0),
                &RouteCapacityProjection::default(),
            ),
            Some(10),
        );
    }
}
