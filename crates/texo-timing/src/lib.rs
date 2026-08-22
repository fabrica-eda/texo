//! Post-route static timing analysis over Texo's unified graph.
//!
//! Both early/minimum and late/maximum propagation are modeled so setup and
//! hold checks can share one characterized target timing model.

use std::cmp::Reverse;
use std::collections::{BTreeMap, BinaryHeap, VecDeque};
use std::error::Error;
use std::fmt;

use texo_model::{
    CellId, CellPinId, Design, Device, ModelError, NetId, PipId, UnifiedGraph, WireId,
};
use texo_pnr::{NetRoute, Placement, PnrResult};

/// One trillion picoseconds per second.
pub const PICOSECONDS_PER_SECOND: u64 = 1_000_000_000_000;

/// Clock periods applied to logical clock nets.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct TimingConstraints {
    clock_periods_ps: BTreeMap<NetId, u64>,
}

impl TimingConstraints {
    /// Creates an empty constraint set.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            clock_periods_ps: BTreeMap::new(),
        }
    }

    /// Sets or replaces one clock period in picoseconds.
    pub fn set_clock_period_ps(&mut self, net: NetId, period_ps: u64) {
        self.clock_periods_ps.insert(net, period_ps);
    }

    /// Constrained logical clock nets in stable ID order.
    #[must_use]
    pub const fn clock_periods_ps(&self) -> &BTreeMap<NetId, u64> {
        &self.clock_periods_ps
    }
}

/// Minimum and maximum delay in picoseconds.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DelayRange {
    /// Earliest delay.
    pub min_ps: u64,
    /// Latest delay.
    pub max_ps: u64,
}

impl DelayRange {
    /// Creates a validated delay range.
    ///
    /// # Errors
    ///
    /// Returns an error when the minimum exceeds the maximum.
    pub const fn new(min_ps: u64, max_ps: u64) -> Result<Self, TimingError> {
        if min_ps <= max_ps {
            Ok(Self { min_ps, max_ps })
        } else {
            Err(TimingError::InvalidDelayRange { min_ps, max_ps })
        }
    }

    /// Creates early and late values fitted independently at two timing corners.
    ///
    /// Unlike [`Self::new`], this preserves the characterized values even when
    /// model fitting makes the nominal minimum numerically greater than the
    /// nominal maximum. Static timing propagates the two corners separately.
    #[must_use]
    pub const fn from_independent_corners(min_ps: u64, max_ps: u64) -> Self {
        Self { min_ps, max_ps }
    }

    /// Zero-delay range.
    #[must_use]
    pub const fn zero() -> Self {
        Self {
            min_ps: 0,
            max_ps: 0,
        }
    }

    fn checked_add(self, other: Self) -> Result<Self, TimingError> {
        Ok(Self {
            min_ps: self
                .min_ps
                .checked_add(other.min_ps)
                .ok_or(TimingError::DelayOverflow)?,
            max_ps: self
                .max_ps
                .checked_add(other.max_ps)
                .ok_or(TimingError::DelayOverflow)?,
        })
    }
}

/// Characterized timing attached to stable logical pins.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct TimingModel {
    cell_arcs: BTreeMap<(CellPinId, CellPinId), DelayRange>,
    clock_to_q: BTreeMap<CellPinId, (CellPinId, DelayRange)>,
    setup_holds: BTreeMap<CellPinId, (CellPinId, DelayRange, DelayRange)>,
}

impl TimingModel {
    /// Creates an empty target timing model.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            cell_arcs: BTreeMap::new(),
            clock_to_q: BTreeMap::new(),
            setup_holds: BTreeMap::new(),
        }
    }

    /// Adds one combinational cell arc.
    ///
    /// # Errors
    ///
    /// Returns an error when the same pin pair already has an arc.
    pub fn add_cell_arc(
        &mut self,
        from: CellPinId,
        to: CellPinId,
        delay: DelayRange,
    ) -> Result<(), TimingError> {
        if self.cell_arcs.insert((from, to), delay).is_some() {
            Err(TimingError::DuplicateCellArc { from, to })
        } else {
            Ok(())
        }
    }

    /// Adds a sequential clock-to-output arc.
    ///
    /// # Errors
    ///
    /// Returns an error when the output already has a clock-to-Q model.
    pub fn add_clock_to_q(
        &mut self,
        clock: CellPinId,
        output: CellPinId,
        delay: DelayRange,
    ) -> Result<(), TimingError> {
        if self.clock_to_q.insert(output, (clock, delay)).is_some() {
            Err(TimingError::DuplicateClockToQ(output))
        } else {
            Ok(())
        }
    }

    /// Adds setup and hold requirements for one sequential input.
    ///
    /// # Errors
    ///
    /// Returns an error when the input already has a timing check.
    pub fn add_setup_hold(
        &mut self,
        clock: CellPinId,
        signal: CellPinId,
        setup: DelayRange,
        hold: DelayRange,
    ) -> Result<(), TimingError> {
        if self
            .setup_holds
            .insert(signal, (clock, setup, hold))
            .is_some()
        {
            Err(TimingError::DuplicateSetupHold(signal))
        } else {
            Ok(())
        }
    }

    /// Combinational delay for one exact logical pin pair.
    #[must_use]
    pub fn cell_arc(&self, from: CellPinId, to: CellPinId) -> Option<DelayRange> {
        self.cell_arcs.get(&(from, to)).copied()
    }

    /// Clock pin and delay for one sequential output.
    #[must_use]
    pub fn clock_to_q(&self, output: CellPinId) -> Option<(CellPinId, DelayRange)> {
        self.clock_to_q.get(&output).copied()
    }

    /// Clock pin, setup, and hold ranges for one sequential input.
    #[must_use]
    pub fn setup_hold(&self, signal: CellPinId) -> Option<(CellPinId, DelayRange, DelayRange)> {
        self.setup_holds.get(&signal).copied()
    }
}

/// Post-route delay from one logical net driver to one sink pin.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct NetDelay {
    /// Logical net.
    pub net: NetId,
    /// Logical sink pin.
    pub sink: CellPinId,
    /// Sum of selected PIP delay ranges on the route to this sink.
    pub delay: DelayRange,
}

/// One register data setup check.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SetupCheck {
    /// Register cell.
    pub cell: CellId,
    /// Register data input pin.
    pub data_pin: CellPinId,
    /// Constrained clock net.
    pub clock_net: NetId,
    /// Longest combinational arrival at the data pin.
    pub arrival_ps: u64,
    /// Earliest clock arrival at this register.
    pub clock_arrival_ps: u64,
    /// Latest characterized setup requirement.
    pub setup_ps: u64,
    /// Required arrival after setup uncertainty.
    pub required_ps: i128,
    /// Required minus actual arrival.
    pub slack_ps: i128,
}

/// One register data hold check.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct HoldCheck {
    /// Register cell.
    pub cell: CellId,
    /// Register data input pin.
    pub data_pin: CellPinId,
    /// Constrained clock net.
    pub clock_net: NetId,
    /// Earliest data arrival at the input.
    pub arrival_ps: u64,
    /// Latest clock arrival at this register.
    pub clock_arrival_ps: u64,
    /// Latest characterized hold requirement.
    pub hold_ps: u64,
    /// Earliest allowed data arrival.
    pub required_ps: u64,
    /// Actual minus required arrival.
    pub slack_ps: i128,
}

/// Complete result of one static timing pass.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TimingReport {
    /// Per-sink routed net delays.
    pub net_delays: Vec<NetDelay>,
    /// Constrained register setup checks.
    pub setup_checks: Vec<SetupCheck>,
    /// Constrained register hold checks.
    pub hold_checks: Vec<HoldCheck>,
    /// Smallest setup slack, or `None` when no endpoint was constrained.
    pub worst_slack_ps: Option<i128>,
    /// Smallest hold slack, or `None` when no endpoint was constrained.
    pub worst_hold_slack_ps: Option<i128>,
}

impl TimingReport {
    /// Whether at least one setup endpoint was checked and every check passed.
    #[must_use]
    pub fn met_timing(&self) -> bool {
        self.worst_slack_ps.is_some_and(|slack| slack >= 0)
            && self.worst_hold_slack_ps.is_some_and(|slack| slack >= 0)
    }
}

/// Analyzes characterized routed and cell delays for setup and hold paths.
///
/// `pip_delays` must contain every PIP selected by the router. The timing model
/// supplies explicit combinational, clock-to-Q, setup, and hold arcs.
///
/// # Errors
///
/// Returns an error for inconsistent routes or placement, missing PIP delays,
/// unreachable routed sinks, invalid clock constraints, arithmetic overflow,
/// or a combinational cycle.
pub fn analyze_timing(
    design: &Design,
    device: &Device,
    implementation: &PnrResult,
    pip_delays: &BTreeMap<PipId, DelayRange>,
    model: &TimingModel,
    constraints: &TimingConstraints,
) -> Result<TimingReport, TimingError> {
    validate_constraints(design, constraints)?;
    validate_model(design, model)?;
    let net_delays = routed_net_delays(design, device, implementation, pip_delays)?;
    let delays_by_sink = net_delays
        .iter()
        .map(|delay| ((delay.net, delay.sink), delay.delay))
        .collect::<BTreeMap<_, _>>();
    let clock_arrivals = pin_arrivals(design, &delays_by_sink, model, &BTreeMap::new())?;
    let mut register_starts = BTreeMap::new();
    for (&output, &(clock, delay)) in &model.clock_to_q {
        let clock_arrival = clock_arrivals[clock.0].unwrap_or(DelayRange::zero());
        register_starts.insert(output, clock_arrival.checked_add(delay)?);
    }
    let arrivals = pin_arrivals(design, &delays_by_sink, model, &register_starts)?;

    let mut setup_checks = Vec::new();
    let mut hold_checks = Vec::new();
    for (&data_pin, &(clock_pin, setup, hold)) in &model.setup_holds {
        let Some(clock_net) = design.pins()[clock_pin.0].net() else {
            continue;
        };
        let Some(&period_ps) = constraints.clock_periods_ps.get(&clock_net) else {
            continue;
        };
        let clock_arrival = clock_arrivals[clock_pin.0].unwrap_or(DelayRange::zero());
        let arrival = arrivals[data_pin.0].unwrap_or(DelayRange::zero());
        let required_ps =
            i128::from(period_ps) + i128::from(clock_arrival.min_ps) - i128::from(setup.max_ps);
        setup_checks.push(SetupCheck {
            cell: design.pins()[data_pin.0].cell,
            data_pin,
            clock_net,
            arrival_ps: arrival.max_ps,
            clock_arrival_ps: clock_arrival.min_ps,
            setup_ps: setup.max_ps,
            required_ps,
            slack_ps: required_ps - i128::from(arrival.max_ps),
        });
        let hold_required_ps = clock_arrival
            .max_ps
            .checked_add(hold.max_ps)
            .ok_or(TimingError::DelayOverflow)?;
        hold_checks.push(HoldCheck {
            cell: design.pins()[data_pin.0].cell,
            data_pin,
            clock_net,
            arrival_ps: arrival.min_ps,
            clock_arrival_ps: clock_arrival.max_ps,
            hold_ps: hold.max_ps,
            required_ps: hold_required_ps,
            slack_ps: i128::from(arrival.min_ps) - i128::from(hold_required_ps),
        });
    }
    let worst_slack_ps = setup_checks.iter().map(|check| check.slack_ps).min();
    let worst_hold_slack_ps = hold_checks.iter().map(|check| check.slack_ps).min();
    Ok(TimingReport {
        net_delays,
        setup_checks,
        hold_checks,
        worst_slack_ps,
        worst_hold_slack_ps,
    })
}

fn validate_model(design: &Design, model: &TimingModel) -> Result<(), TimingError> {
    for (&(from, to), delay) in &model.cell_arcs {
        validate_pin_pair(design, from, to)?;
        validate_range(*delay)?;
    }
    for (&output, &(clock, delay)) in &model.clock_to_q {
        validate_pin_pair(design, clock, output)?;
        validate_range(delay)?;
    }
    for (&signal, &(clock, setup, hold)) in &model.setup_holds {
        validate_pin_pair(design, clock, signal)?;
        validate_range(setup)?;
        validate_range(hold)?;
    }
    Ok(())
}

fn validate_pin_pair(
    design: &Design,
    first: CellPinId,
    second: CellPinId,
) -> Result<(), TimingError> {
    let first_pin = design
        .pins()
        .get(first.0)
        .ok_or(TimingError::Model(ModelError::UnknownCellPin(first)))?;
    let second_pin = design
        .pins()
        .get(second.0)
        .ok_or(TimingError::Model(ModelError::UnknownCellPin(second)))?;
    if first_pin.cell != second_pin.cell {
        return Err(TimingError::CrossCellTimingArc { first, second });
    }
    Ok(())
}

const fn validate_range(delay: DelayRange) -> Result<(), TimingError> {
    if delay.min_ps <= delay.max_ps {
        Ok(())
    } else {
        Err(TimingError::InvalidDelayRange {
            min_ps: delay.min_ps,
            max_ps: delay.max_ps,
        })
    }
}

fn validate_constraints(
    design: &Design,
    constraints: &TimingConstraints,
) -> Result<(), TimingError> {
    for (&net, &period_ps) in &constraints.clock_periods_ps {
        if net.0 >= design.nets().len() {
            return Err(TimingError::UnknownClockNet(net));
        }
        if period_ps == 0 {
            return Err(TimingError::ZeroClockPeriod(net));
        }
    }
    Ok(())
}

fn routed_net_delays(
    design: &Design,
    device: &Device,
    implementation: &PnrResult,
    pip_delays: &BTreeMap<PipId, DelayRange>,
) -> Result<Vec<NetDelay>, TimingError> {
    let mut routes = BTreeMap::new();
    for route in &implementation.routes {
        if route.net.0 >= design.nets().len() {
            return Err(TimingError::UnknownRoutedNet(route.net));
        }
        if routes.insert(route.net, route).is_some() {
            return Err(TimingError::DuplicateRoute(route.net));
        }
    }
    let graph = UnifiedGraph::new(design, device);
    let mut result = Vec::new();
    for (index, net) in design.nets().iter().enumerate() {
        let net_id = NetId(index);
        let route = routes
            .get(&net_id)
            .copied()
            .ok_or(TimingError::MissingRoute(net_id))?;
        let driver_wire = bound_wire(&graph, &implementation.placement, net.driver, device)?;
        let distances = route_distances(route, driver_wire, device, pip_delays)?;
        for &sink in &net.sinks {
            let sink_wire = bound_wire(&graph, &implementation.placement, sink, device)?;
            let delay = distances
                .get(&sink_wire)
                .copied()
                .ok_or(TimingError::UnreachableSink { net: net_id, sink })?;
            result.push(NetDelay {
                net: net_id,
                sink,
                delay,
            });
        }
    }
    Ok(result)
}

fn bound_wire(
    graph: &UnifiedGraph<'_>,
    placement: &Placement,
    pin: CellPinId,
    device: &Device,
) -> Result<WireId, TimingError> {
    if pin.0 >= graph.design().pins().len() {
        return Err(TimingError::Model(ModelError::UnknownCellPin(pin)));
    }
    if let Some(bel_pin) = placement.pin_binding(pin) {
        return device
            .bel_pins()
            .get(bel_pin.0)
            .map(|pin| pin.wire)
            .ok_or(TimingError::Model(ModelError::UnknownBelPin(bel_pin)));
    }
    let cell = graph.design().pins()[pin.0].cell;
    let bel = placement
        .bel(cell)
        .ok_or(TimingError::MissingPlacement(cell))?;
    graph.bound_wire(pin, bel).map_err(TimingError::Model)
}

fn route_distances(
    route: &NetRoute,
    source: WireId,
    device: &Device,
    pip_delays: &BTreeMap<PipId, DelayRange>,
) -> Result<BTreeMap<WireId, DelayRange>, TimingError> {
    if source.0 >= device.wires().len() {
        return Err(TimingError::Model(ModelError::UnknownWire(source)));
    }
    let mut adjacency: BTreeMap<WireId, Vec<(WireId, DelayRange)>> = BTreeMap::new();
    for &pip_id in &route.pips {
        let pip = device
            .pips()
            .get(pip_id.0)
            .ok_or(TimingError::UnknownRoutedPip(pip_id))?;
        let delay = pip_delays
            .get(&pip_id)
            .copied()
            .ok_or(TimingError::MissingPipDelay(pip_id))?;
        adjacency.entry(pip.from).or_default().push((pip.to, delay));
        if pip.bidirectional {
            adjacency.entry(pip.to).or_default().push((pip.from, delay));
        }
    }

    let minimum = scalar_route_distances(source, &adjacency, |delay| delay.min_ps)?;
    let maximum = scalar_route_distances(source, &adjacency, |delay| delay.max_ps)?;
    Ok(minimum
        .into_iter()
        .filter_map(|(wire, min_ps)| {
            maximum
                .get(&wire)
                .copied()
                .map(|max_ps| (wire, DelayRange { min_ps, max_ps }))
        })
        .collect())
}

fn scalar_route_distances(
    source: WireId,
    adjacency: &BTreeMap<WireId, Vec<(WireId, DelayRange)>>,
    select: impl Fn(DelayRange) -> u64,
) -> Result<BTreeMap<WireId, u64>, TimingError> {
    let mut distances = BTreeMap::from([(source, 0_u64)]);
    let mut pending = BinaryHeap::from([Reverse((0_u64, source))]);
    while let Some(Reverse((distance, wire))) = pending.pop() {
        if distances.get(&wire).copied() != Some(distance) {
            continue;
        }
        for &(next, edge_delay) in adjacency.get(&wire).map_or(&[][..], Vec::as_slice) {
            let candidate = distance
                .checked_add(select(edge_delay))
                .ok_or(TimingError::DelayOverflow)?;
            if distances.get(&next).is_none_or(|&known| candidate < known) {
                distances.insert(next, candidate);
                pending.push(Reverse((candidate, next)));
            }
        }
    }
    Ok(distances)
}

fn pin_arrivals(
    design: &Design,
    delays: &BTreeMap<(NetId, CellPinId), DelayRange>,
    model: &TimingModel,
    starts: &BTreeMap<CellPinId, DelayRange>,
) -> Result<Vec<Option<DelayRange>>, TimingError> {
    let mut edges = vec![Vec::<(CellPinId, DelayRange)>::new(); design.pins().len()];
    let mut indegree = vec![0_usize; design.pins().len()];
    for (net_index, net) in design.nets().iter().enumerate() {
        let net_id = NetId(net_index);
        for &sink in &net.sinks {
            let delay = delays
                .get(&(net_id, sink))
                .copied()
                .ok_or(TimingError::MissingNetDelay { net: net_id, sink })?;
            edges[net.driver.0].push((sink, delay));
            indegree[sink.0] += 1;
        }
    }
    for (&(from, to), &delay) in &model.cell_arcs {
        edges[from.0].push((to, delay));
        indegree[to.0] += 1;
    }

    let mut arrivals: Vec<Option<DelayRange>> = vec![None; design.pins().len()];
    let mut ready = VecDeque::new();
    for (index, &degree) in indegree.iter().enumerate() {
        if degree == 0 {
            let pin = CellPinId(index);
            arrivals[index] = Some(starts.get(&pin).copied().unwrap_or(DelayRange::zero()));
            ready.push_back(pin);
        }
    }
    let mut visited = 0_usize;
    while let Some(pin) = ready.pop_front() {
        visited += 1;
        let arrival = arrivals[pin.0].unwrap_or(DelayRange::zero());
        for &(next, delay) in &edges[pin.0] {
            let candidate = arrival.checked_add(delay)?;
            arrivals[next.0] = Some(arrivals[next.0].map_or(candidate, |known| DelayRange {
                min_ps: known.min_ps.min(candidate.min_ps),
                max_ps: known.max_ps.max(candidate.max_ps),
            }));
            indegree[next.0] -= 1;
            if indegree[next.0] == 0 {
                ready.push_back(next);
            }
        }
    }
    if visited != design.pins().len() {
        return Err(TimingError::CombinationalCycle);
    }
    Ok(arrivals)
}

/// Static timing model or analysis failure.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TimingError {
    /// Logical/physical graph validation failed.
    Model(ModelError),
    /// A delay range had its minimum above its maximum.
    InvalidDelayRange {
        /// Invalid minimum.
        min_ps: u64,
        /// Invalid maximum.
        max_ps: u64,
    },
    /// A combinational cell arc was added twice.
    DuplicateCellArc {
        /// Source pin.
        from: CellPinId,
        /// Destination pin.
        to: CellPinId,
    },
    /// A sequential output had more than one clock-to-Q arc.
    DuplicateClockToQ(CellPinId),
    /// A sequential input had more than one setup/hold model.
    DuplicateSetupHold(CellPinId),
    /// Both ends of a cell timing arc must belong to the same cell.
    CrossCellTimingArc {
        /// First pin.
        first: CellPinId,
        /// Second pin.
        second: CellPinId,
    },
    /// A placed cell has no BEL binding.
    MissingPlacement(CellId),
    /// A route references no logical net.
    UnknownRoutedNet(NetId),
    /// More than one route was supplied for a logical net.
    DuplicateRoute(NetId),
    /// A logical net has no physical route record.
    MissingRoute(NetId),
    /// A route references no physical PIP.
    UnknownRoutedPip(PipId),
    /// No delay was provided for a selected PIP.
    MissingPipDelay(PipId),
    /// A selected route tree does not reach one logical sink.
    UnreachableSink {
        /// Logical net.
        net: NetId,
        /// Unreachable sink pin.
        sink: CellPinId,
    },
    /// A timing constraint references no logical net.
    UnknownClockNet(NetId),
    /// A constrained clock period is zero.
    ZeroClockPeriod(NetId),
    /// A routed net/sink pair had no calculated delay.
    MissingNetDelay {
        /// Logical net.
        net: NetId,
        /// Logical sink pin.
        sink: CellPinId,
    },
    /// Delay accumulation exceeded `u64` picoseconds.
    DelayOverflow,
    /// The cell timing graph contains a combinational cycle.
    CombinationalCycle,
}

impl fmt::Display for TimingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Model(error) => write!(f, "invalid timing graph: {error}"),
            Self::InvalidDelayRange { min_ps, max_ps } => {
                write!(f, "invalid timing range {min_ps}..={max_ps} ps")
            }
            Self::DuplicateCellArc { from, to } => {
                write!(f, "cell timing arc {} -> {} was added twice", from.0, to.0)
            }
            Self::DuplicateClockToQ(output) => {
                write!(f, "pin {} has more than one clock-to-Q arc", output.0)
            }
            Self::DuplicateSetupHold(signal) => {
                write!(f, "pin {} has more than one setup/hold check", signal.0)
            }
            Self::CrossCellTimingArc { first, second } => write!(
                f,
                "timing arc pins {} and {} belong to different cells",
                first.0, second.0
            ),
            Self::MissingPlacement(cell) => write!(f, "cell {} has no placement", cell.0),
            Self::UnknownRoutedNet(net) => write!(f, "route references unknown net {}", net.0),
            Self::DuplicateRoute(net) => write!(f, "net {} has more than one route", net.0),
            Self::MissingRoute(net) => write!(f, "net {} has no route", net.0),
            Self::UnknownRoutedPip(pip) => write!(f, "route references unknown PIP {}", pip.0),
            Self::MissingPipDelay(pip) => write!(f, "selected PIP {} has no delay", pip.0),
            Self::UnreachableSink { net, sink } => write!(
                f,
                "route for net {} does not reach sink pin {}",
                net.0, sink.0
            ),
            Self::UnknownClockNet(net) => {
                write!(f, "clock constraint references unknown net {}", net.0)
            }
            Self::ZeroClockPeriod(net) => write!(f, "clock net {} has a zero period", net.0),
            Self::MissingNetDelay { net, sink } => {
                write!(f, "net {} sink pin {} has no routed delay", net.0, sink.0)
            }
            Self::DelayOverflow => write!(f, "timing delay arithmetic overflowed"),
            Self::CombinationalCycle => write!(f, "timing graph contains a combinational cycle"),
        }
    }
}

impl Error for TimingError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Model(error) => Some(error),
            _ => None,
        }
    }
}

impl From<ModelError> for TimingError {
    fn from(value: ModelError) -> Self {
        Self::Model(value)
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use texo_model::{Design, Device, PinDirection, Point, ResourceKind};
    use texo_pnr::place_and_route;

    use super::{DelayRange, TimingConstraints, TimingModel, analyze_timing};

    #[test]
    fn reports_positive_and_negative_post_route_setup_slack() {
        let (design, device, clock_net, model) = registered_path(10);
        let implementation = place_and_route(&design, &device).unwrap();
        let pip_delays = device
            .pips()
            .iter()
            .enumerate()
            .map(|(index, _)| (texo_model::PipId(index), DelayRange::new(100, 100).unwrap()))
            .collect::<BTreeMap<_, _>>();

        let mut constraints = TimingConstraints::new();
        constraints.set_clock_period_ps(clock_net, 50);
        let failed = analyze_timing(
            &design,
            &device,
            &implementation,
            &pip_delays,
            &model,
            &constraints,
        )
        .unwrap();
        assert_eq!(failed.setup_checks.len(), 1);
        assert_eq!(failed.worst_slack_ps, Some(-90));
        assert!(!failed.met_timing());

        constraints.set_clock_period_ps(clock_net, 150);
        let passed = analyze_timing(
            &design,
            &device,
            &implementation,
            &pip_delays,
            &model,
            &constraints,
        )
        .unwrap();
        assert_eq!(passed.worst_slack_ps, Some(10));
        assert_eq!(passed.worst_hold_slack_ps, Some(110));
        assert!(passed.met_timing());
    }

    #[test]
    fn a_hold_violation_blocks_timing_closure() {
        let (design, device, clock_net, model) = registered_path(250);
        let implementation = place_and_route(&design, &device).unwrap();
        let pip_delays = device
            .pips()
            .iter()
            .enumerate()
            .map(|(index, _)| (texo_model::PipId(index), DelayRange::new(100, 100).unwrap()))
            .collect::<BTreeMap<_, _>>();
        let mut constraints = TimingConstraints::new();
        constraints.set_clock_period_ps(clock_net, 1_000);

        let report = analyze_timing(
            &design,
            &device,
            &implementation,
            &pip_delays,
            &model,
            &constraints,
        )
        .unwrap();

        assert_eq!(report.worst_hold_slack_ps, Some(-130));
        assert!(!report.met_timing());
    }

    fn registered_path(hold_ps: u64) -> (Design, Device, texo_model::NetId, TimingModel) {
        let mut design = Design::new();
        let input = design.add_cell("input", ResourceKind::Io);
        let input_o = design.add_pin(input, "O", PinDirection::Output).unwrap();
        let clock = design.add_cell("clock", ResourceKind::Io);
        let clock_o = design.add_pin(clock, "O", PinDirection::Output).unwrap();
        let lut = design.add_cell("lut", ResourceKind::Lut(4));
        let lut_a = design.add_pin(lut, "A", PinDirection::Input).unwrap();
        let lut_f = design.add_pin(lut, "F", PinDirection::Output).unwrap();
        let ff = design.add_cell("ff", ResourceKind::Register);
        let ff_di = design.add_pin(ff, "DI", PinDirection::Input).unwrap();
        let ff_clk = design.add_pin(ff, "CLK", PinDirection::Input).unwrap();
        let ff_q = design.add_pin(ff, "Q", PinDirection::Output).unwrap();
        design.add_net("input", input_o, [lut_a]).unwrap();
        design.add_net("data", lut_f, [ff_di]).unwrap();
        let clock_net = design.add_net("clock", clock_o, [ff_clk]).unwrap();

        let mut device = Device::new("timing", 4, 1).unwrap();
        let io_data_wire = device.add_wire("io-data", Point::new(0, 0), 1).unwrap();
        let io_clock_wire = device.add_wire("io-clock", Point::new(0, 0), 1).unwrap();
        let logic_input_wire = device.add_wire("lut-a", Point::new(1, 0), 1).unwrap();
        let logic_output_wire = device.add_wire("lut-f", Point::new(1, 0), 1).unwrap();
        let ff_di_wire = device.add_wire("ff-di", Point::new(2, 0), 1).unwrap();
        let ff_clk_wire = device.add_wire("ff-clk", Point::new(2, 0), 1).unwrap();
        let ff_q_wire = device.add_wire("ff-q", Point::new(2, 0), 1).unwrap();
        let io_data = device
            .add_bel("IO-DATA", ResourceKind::Io, Point::new(0, 0))
            .unwrap();
        device
            .add_bel_pin(io_data, "O", PinDirection::Output, io_data_wire)
            .unwrap();
        let io_clock = device
            .add_bel("IO-CLOCK", ResourceKind::Io, Point::new(0, 0))
            .unwrap();
        device
            .add_bel_pin(io_clock, "O", PinDirection::Output, io_clock_wire)
            .unwrap();
        let lut_bel = device
            .add_bel("LUT", ResourceKind::Lut(4), Point::new(1, 0))
            .unwrap();
        device
            .add_bel_pin(lut_bel, "A", PinDirection::Input, logic_input_wire)
            .unwrap();
        device
            .add_bel_pin(lut_bel, "F", PinDirection::Output, logic_output_wire)
            .unwrap();
        let ff_bel = device
            .add_bel("FF", ResourceKind::Register, Point::new(2, 0))
            .unwrap();
        device
            .add_bel_pin(ff_bel, "DI", PinDirection::Input, ff_di_wire)
            .unwrap();
        device
            .add_bel_pin(ff_bel, "CLK", PinDirection::Input, ff_clk_wire)
            .unwrap();
        device
            .add_bel_pin(ff_bel, "Q", PinDirection::Output, ff_q_wire)
            .unwrap();
        device
            .add_pip(io_data_wire, logic_input_wire, false, 1)
            .unwrap();
        device
            .add_pip(logic_output_wire, ff_di_wire, false, 1)
            .unwrap();
        device
            .add_pip(io_clock_wire, ff_clk_wire, false, 1)
            .unwrap();
        let mut model = TimingModel::new();
        model
            .add_cell_arc(lut_a, lut_f, DelayRange::new(20, 30).unwrap())
            .unwrap();
        model
            .add_clock_to_q(ff_clk, ff_q, DelayRange::new(40, 50).unwrap())
            .unwrap();
        model
            .add_setup_hold(
                ff_clk,
                ff_di,
                DelayRange::new(10, 10).unwrap(),
                DelayRange::new(hold_ps, hold_ps).unwrap(),
            )
            .unwrap();
        (design, device, clock_net, model)
    }
}
