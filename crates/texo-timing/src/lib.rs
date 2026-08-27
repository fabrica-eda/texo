//! Post-route static timing analysis over Texo's unified graph.
//!
//! Both early/minimum and late/maximum propagation are modeled so setup and
//! hold checks can share one characterized target timing model. Register
//! launches are propagated independently per clock net; unconstrained primary
//! inputs and cross-clock paths are not treated as zero-time synchronous
//! launches.

use std::collections::{BTreeMap, VecDeque};
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

/// Setup slack contributed by one logical net edge to a constrained endpoint.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct NetSetupSlack {
    /// Logical net.
    pub net: NetId,
    /// Logical sink pin for this fanout edge.
    pub sink: CellPinId,
    /// Required arrival minus the late arrival through this edge.
    pub slack_ps: i128,
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
    pub required_ps: i128,
    /// Actual minus required arrival.
    pub slack_ps: i128,
}

/// Why a modeled sequential endpoint was not checked by this timing pass.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum UncheckedEndpointReason {
    /// The modeled clock pin is not connected to a logical net.
    UnconnectedClock,
    /// The endpoint's clock net has no period constraint.
    UnconstrainedClock,
    /// No launch in the capture clock domain reaches the endpoint.
    NoSynchronousLaunch,
}

impl UncheckedEndpointReason {
    /// Stable diagnostic name used by checkpoints and other reports.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::UnconnectedClock => "unconnected_clock",
            Self::UnconstrainedClock => "unconstrained_clock",
            Self::NoSynchronousLaunch => "no_synchronous_launch",
        }
    }
}

/// One modeled sequential endpoint omitted from setup and hold checks.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct UncheckedEndpoint {
    /// Register cell.
    pub cell: CellId,
    /// Register data input pin.
    pub data_pin: CellPinId,
    /// Modeled register clock pin.
    pub clock_pin: CellPinId,
    /// Logical clock net, when the clock pin is connected.
    pub clock_net: Option<NetId>,
    /// Reason this endpoint was not analyzed.
    pub reason: UncheckedEndpointReason,
}

/// Complete result of one static timing pass.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TimingReport {
    /// Per-sink routed net delays.
    pub net_delays: Vec<NetDelay>,
    /// Per-sink setup slack after backward required-time propagation.
    pub net_setup_slacks: Vec<NetSetupSlack>,
    /// Constrained register setup checks.
    pub setup_checks: Vec<SetupCheck>,
    /// Constrained register hold checks.
    pub hold_checks: Vec<HoldCheck>,
    /// Modeled endpoints omitted from setup and hold checks, with reasons.
    pub unchecked_endpoints: Vec<UncheckedEndpoint>,
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

    /// Number of sequential endpoints present in the timing model.
    #[must_use]
    pub fn modeled_endpoint_count(&self) -> usize {
        self.setup_checks.len() + self.unchecked_endpoints.len()
    }

    /// Whether every modeled sequential endpoint was checked.
    ///
    /// This is deliberately separate from [`Self::met_timing`]: primary-input
    /// and cross-clock paths require constraints that the current constraint
    /// model cannot yet express, so existing flows may intentionally omit them.
    #[must_use]
    pub fn all_modeled_endpoints_checked(&self) -> bool {
        self.unchecked_endpoints.is_empty()
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
    timing_report_from_net_delays(design, model, constraints, net_delays)
}

/// Estimates one net-edge routing delay from placement geometry.
///
/// Driver-to-sink Manhattan distance times `ps_per_tile_ps` plus
/// `hop_overhead_ps` for pin access. The absolute picoseconds are not
/// characterized values; they exist to rank or pre-screen placement deltas
/// without paying for routing. Returns `None` when either pin has no
/// physical wire under the placement.
#[must_use]
pub fn estimate_edge_delay(
    design: &Design,
    device: &Device,
    placement: &Placement,
    driver_pin: CellPinId,
    sink_pin: CellPinId,
    ps_per_tile_ps: u64,
    hop_overhead_ps: u64,
) -> Option<u64> {
    let graph = UnifiedGraph::new(design, device);
    let driver_wire = bound_wire(&graph, placement, driver_pin, device).ok()?;
    let sink_wire = bound_wire(&graph, placement, sink_pin, device).ok()?;
    Some(
        device.wires()[driver_wire.0]
            .point
            .manhattan(device.wires()[sink_wire.0].point)
            .saturating_mul(ps_per_tile_ps)
            .saturating_add(hop_overhead_ps),
    )
}

/// Builds the full setup/hold report from a set of per-sink net delays.
fn timing_report_from_net_delays(
    design: &Design,
    model: &TimingModel,
    constraints: &TimingConstraints,
    net_delays: Vec<NetDelay>,
) -> Result<TimingReport, TimingError> {
    let delays_by_sink = net_delays
        .iter()
        .map(|delay| ((delay.net, delay.sink), delay.delay))
        .collect::<BTreeMap<_, _>>();
    let clock_arrivals = pin_arrivals(design, &delays_by_sink, model, &BTreeMap::new(), true)?;
    let mut register_starts_by_clock = BTreeMap::<NetId, BTreeMap<CellPinId, DelayRange>>::new();
    for (&output, &(clock, delay)) in &model.clock_to_q {
        let Some(clock_net) = design.pins()[clock.0].net() else {
            continue;
        };
        let clock_arrival = clock_arrivals[clock.0].unwrap_or(DelayRange::zero());
        register_starts_by_clock
            .entry(clock_net)
            .or_default()
            .insert(output, clock_arrival.checked_add(delay)?);
    }
    let arrivals_by_clock = register_starts_by_clock
        .iter()
        .map(|(&clock_net, starts)| {
            Ok((
                clock_net,
                pin_arrivals(design, &delays_by_sink, model, starts, false)?,
            ))
        })
        .collect::<Result<BTreeMap<_, _>, TimingError>>()?;

    let mut setup_checks = Vec::new();
    let mut hold_checks = Vec::new();
    let mut unchecked_endpoints = Vec::new();
    for (&data_pin, &(clock_pin, setup, hold)) in &model.setup_holds {
        let Some(clock_net) = design.pins()[clock_pin.0].net() else {
            unchecked_endpoints.push(unchecked_endpoint(
                design,
                data_pin,
                clock_pin,
                None,
                UncheckedEndpointReason::UnconnectedClock,
            ));
            continue;
        };
        let Some(&period_ps) = constraints.clock_periods_ps.get(&clock_net) else {
            unchecked_endpoints.push(unchecked_endpoint(
                design,
                data_pin,
                clock_pin,
                Some(clock_net),
                UncheckedEndpointReason::UnconstrainedClock,
            ));
            continue;
        };
        let Some(arrival) = arrivals_by_clock
            .get(&clock_net)
            .and_then(|arrivals| arrivals[data_pin.0])
        else {
            // Primary-input and cross-clock paths need explicit timing
            // constraints. Do not invent a synchronous launch at time zero.
            unchecked_endpoints.push(unchecked_endpoint(
                design,
                data_pin,
                clock_pin,
                Some(clock_net),
                UncheckedEndpointReason::NoSynchronousLaunch,
            ));
            continue;
        };
        let clock_arrival = clock_arrivals[clock_pin.0].unwrap_or(DelayRange::zero());
        let common_clock_arrival =
            clock_arrivals[design.nets()[clock_net.0].driver.0].unwrap_or(DelayRange::zero());
        let (setup_check, hold_check) = endpoint_checks(EndpointCheckContext {
            design,
            data_pin,
            clock_net,
            period_ps,
            arrival,
            clock_arrival,
            common_clock_arrival,
            setup,
            hold,
        });
        setup_checks.push(setup_check);
        hold_checks.push(hold_check);
    }
    let worst_slack_ps = setup_checks.iter().map(|check| check.slack_ps).min();
    let worst_hold_slack_ps = hold_checks.iter().map(|check| check.slack_ps).min();
    let net_setup_slacks = net_setup_slacks(
        design,
        &net_delays,
        model,
        &arrivals_by_clock,
        &setup_checks,
    )?;
    Ok(TimingReport {
        net_delays,
        net_setup_slacks,
        setup_checks,
        hold_checks,
        unchecked_endpoints,
        worst_slack_ps,
        worst_hold_slack_ps,
    })
}

#[derive(Clone, Copy)]
struct EndpointCheckContext<'a> {
    design: &'a Design,
    data_pin: CellPinId,
    clock_net: NetId,
    period_ps: u64,
    arrival: DelayRange,
    clock_arrival: DelayRange,
    common_clock_arrival: DelayRange,
    setup: DelayRange,
    hold: DelayRange,
}

fn endpoint_checks(context: EndpointCheckContext<'_>) -> (SetupCheck, HoldCheck) {
    let EndpointCheckContext {
        design,
        data_pin,
        clock_net,
        period_ps,
        arrival,
        clock_arrival,
        common_clock_arrival,
        setup,
        hold,
    } = context;
    // Every launch in this analysis group and the capture endpoint share the
    // path up to the constrained clock net's driver. Its corner range is
    // common-mode delay, not clock skew, and therefore cancels through CPPR.
    // Early and late values are independently fitted and need not be ordered,
    // so the correction must remain signed.
    let common_clock_pessimism_ps =
        i128::from(common_clock_arrival.max_ps) - i128::from(common_clock_arrival.min_ps);
    let setup_required_ps =
        i128::from(period_ps) + i128::from(clock_arrival.min_ps) + common_clock_pessimism_ps
            - i128::from(setup.max_ps);
    let hold_required_ps =
        i128::from(clock_arrival.max_ps) + i128::from(hold.max_ps) - common_clock_pessimism_ps;
    (
        SetupCheck {
            cell: design.pins()[data_pin.0].cell,
            data_pin,
            clock_net,
            arrival_ps: arrival.max_ps,
            clock_arrival_ps: clock_arrival.min_ps,
            setup_ps: setup.max_ps,
            required_ps: setup_required_ps,
            slack_ps: setup_required_ps - i128::from(arrival.max_ps),
        },
        HoldCheck {
            cell: design.pins()[data_pin.0].cell,
            data_pin,
            clock_net,
            arrival_ps: arrival.min_ps,
            clock_arrival_ps: clock_arrival.max_ps,
            hold_ps: hold.max_ps,
            required_ps: hold_required_ps,
            slack_ps: i128::from(arrival.min_ps) - hold_required_ps,
        },
    )
}

fn unchecked_endpoint(
    design: &Design,
    data_pin: CellPinId,
    clock_pin: CellPinId,
    clock_net: Option<NetId>,
    reason: UncheckedEndpointReason,
) -> UncheckedEndpoint {
    UncheckedEndpoint {
        cell: design.pins()[data_pin.0].cell,
        data_pin,
        clock_pin,
        clock_net,
        reason,
    }
}

fn net_setup_slacks(
    design: &Design,
    net_delays: &[NetDelay],
    model: &TimingModel,
    arrivals_by_clock: &BTreeMap<NetId, Vec<Option<DelayRange>>>,
    setup_checks: &[SetupCheck],
) -> Result<Vec<NetSetupSlack>, TimingError> {
    let mut edges = vec![Vec::<(CellPinId, u64)>::new(); design.pins().len()];
    let mut indegree = vec![0_usize; design.pins().len()];
    for delay in net_delays {
        let driver = design.nets()[delay.net.0].driver;
        edges[driver.0].push((delay.sink, delay.delay.max_ps));
        indegree[delay.sink.0] += 1;
    }
    for (&(from, to), &delay) in &model.cell_arcs {
        edges[from.0].push((to, delay.max_ps));
        indegree[to.0] += 1;
    }

    let mut ready = VecDeque::new();
    for (index, &degree) in indegree.iter().enumerate() {
        if degree == 0 {
            ready.push_back(CellPinId(index));
        }
    }
    let mut order = Vec::with_capacity(design.pins().len());
    while let Some(pin) = ready.pop_front() {
        order.push(pin);
        for &(next, _) in &edges[pin.0] {
            indegree[next.0] -= 1;
            if indegree[next.0] == 0 {
                ready.push_back(next);
            }
        }
    }
    if order.len() != design.pins().len() {
        return Err(TimingError::CombinationalCycle);
    }

    let mut slacks = BTreeMap::<(NetId, CellPinId), i128>::new();
    for (&clock_net, arrivals) in arrivals_by_clock {
        let mut required = vec![None::<i128>; design.pins().len()];
        for check in setup_checks
            .iter()
            .filter(|check| check.clock_net == clock_net)
        {
            let entry = &mut required[check.data_pin.0];
            *entry = Some(entry.map_or(check.required_ps, |known| known.min(check.required_ps)));
        }
        for &from in order.iter().rev() {
            for &(to, delay_ps) in &edges[from.0] {
                let Some(to_required) = required[to.0] else {
                    continue;
                };
                let candidate = to_required - i128::from(delay_ps);
                let entry = &mut required[from.0];
                *entry = Some(entry.map_or(candidate, |known| known.min(candidate)));
            }
        }
        for delay in net_delays {
            let driver = design.nets()[delay.net.0].driver;
            let (Some(required_ps), Some(arrival)) = (required[delay.sink.0], arrivals[driver.0])
            else {
                continue;
            };
            let slack_ps =
                required_ps - i128::from(arrival.max_ps) - i128::from(delay.delay.max_ps);
            slacks
                .entry((delay.net, delay.sink))
                .and_modify(|known| *known = (*known).min(slack_ps))
                .or_insert(slack_ps);
        }
    }

    Ok(slacks
        .into_iter()
        .map(|((net, sink), slack_ps)| NetSetupSlack {
            net,
            sink,
            slack_ps,
        })
        .collect())
}

fn validate_model(design: &Design, model: &TimingModel) -> Result<(), TimingError> {
    for &(from, to) in model.cell_arcs.keys() {
        validate_pin_pair(design, from, to)?;
    }
    for (&output, &(clock, _)) in &model.clock_to_q {
        validate_pin_pair(design, clock, output)?;
    }
    for (&signal, &(clock, _, _)) in &model.setup_holds {
        validate_pin_pair(design, clock, signal)?;
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
        for &sink in &net.sinks {
            let sink_wire = bound_wire(&graph, &implementation.placement, sink, device)?;
            let delay = route_arc_delay(route, net_id, sink, driver_wire, sink_wire, pip_delays)?;
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

fn route_arc_delay(
    route: &NetRoute,
    net: NetId,
    sink: CellPinId,
    source: WireId,
    sink_wire: WireId,
    pip_delays: &BTreeMap<PipId, DelayRange>,
) -> Result<DelayRange, TimingError> {
    let arc = route
        .arc(sink)
        .filter(|arc| {
            arc.wires.first().copied() == Some(source)
                && arc.wires.last().copied() == Some(sink_wire)
        })
        .ok_or(TimingError::UnreachableSink { net, sink })?;
    let mut delay = DelayRange::zero();
    for &pip_id in &arc.pips {
        let edge_delay = pip_delays
            .get(&pip_id)
            .copied()
            .ok_or(TimingError::MissingPipDelay(pip_id))?;
        delay = delay.checked_add(edge_delay)?;
    }
    Ok(delay)
}

fn pin_arrivals(
    design: &Design,
    delays: &BTreeMap<(NetId, CellPinId), DelayRange>,
    model: &TimingModel,
    starts: &BTreeMap<CellPinId, DelayRange>,
    implicit_root_starts: bool,
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
            arrivals[index] = starts
                .get(&pin)
                .copied()
                .or_else(|| implicit_root_starts.then_some(DelayRange::zero()));
            ready.push_back(pin);
        }
    }
    let mut visited = 0_usize;
    while let Some(pin) = ready.pop_front() {
        visited += 1;
        for &(next, delay) in &edges[pin.0] {
            if let Some(arrival) = arrivals[pin.0] {
                let candidate = arrival.checked_add(delay)?;
                arrivals[next.0] = Some(arrivals[next.0].map_or(candidate, |known| DelayRange {
                    min_ps: known.min_ps.min(candidate.min_ps),
                    max_ps: known.max_ps.max(candidate.max_ps),
                }));
            }
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

    use super::{
        DelayRange, NetDelay, TimingConstraints, TimingModel, UncheckedEndpointReason,
        analyze_timing, timing_report_from_net_delays,
    };

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
        // The second modeled endpoint is driven only by an unconstrained
        // primary input and therefore is not fabricated into this clock domain.
        assert_eq!(failed.setup_checks.len(), 1);
        assert_eq!(failed.modeled_endpoint_count(), 2);
        assert_eq!(failed.unchecked_endpoints.len(), 1);
        assert_eq!(
            failed.unchecked_endpoints[0].reason,
            UncheckedEndpointReason::NoSynchronousLaunch
        );
        assert!(!failed.all_modeled_endpoints_checked());
        assert_eq!(failed.worst_slack_ps, Some(-240));
        assert!(!failed.met_timing());

        constraints.set_clock_period_ps(clock_net, 300);
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
        assert_eq!(passed.worst_hold_slack_ps, Some(250));
        assert_eq!(passed.net_setup_slacks.len(), 2);
        assert!(
            passed
                .net_setup_slacks
                .iter()
                .all(|edge| edge.slack_ps == 10)
        );
        assert!(passed.met_timing());
    }

    #[test]
    fn reports_why_modeled_endpoints_are_unchecked() {
        let mut design = Design::new();
        let register = design.add_cell("register", ResourceKind::Register);
        let data = design.add_pin(register, "DI", PinDirection::Input).unwrap();
        let clock = design
            .add_pin(register, "CLK", PinDirection::Input)
            .unwrap();
        let mut model = TimingModel::new();
        model
            .add_setup_hold(
                clock,
                data,
                DelayRange::new(10, 10).unwrap(),
                DelayRange::new(10, 10).unwrap(),
            )
            .unwrap();

        let report =
            timing_report_from_net_delays(&design, &model, &TimingConstraints::new(), Vec::new())
                .unwrap();

        assert_eq!(report.modeled_endpoint_count(), 1);
        assert_eq!(report.setup_checks.len(), 0);
        assert_eq!(report.hold_checks.len(), 0);
        assert_eq!(report.unchecked_endpoints.len(), 1);
        assert_eq!(
            report.unchecked_endpoints[0].reason,
            UncheckedEndpointReason::UnconnectedClock
        );
        assert!(!report.all_modeled_endpoints_checked());
        assert!(!report.met_timing());
    }

    #[test]
    fn reports_unconstrained_clock_endpoints() {
        let mut design = Design::new();
        let clock_source = design.add_cell("clock", ResourceKind::Io);
        let clock_output = design
            .add_pin(clock_source, "O", PinDirection::Output)
            .unwrap();
        let register = design.add_cell("register", ResourceKind::Register);
        let data = design.add_pin(register, "DI", PinDirection::Input).unwrap();
        let clock = design
            .add_pin(register, "CLK", PinDirection::Input)
            .unwrap();
        let clock_net = design.add_net("clock", clock_output, [clock]).unwrap();
        let mut model = TimingModel::new();
        model
            .add_setup_hold(
                clock,
                data,
                DelayRange::new(10, 10).unwrap(),
                DelayRange::new(10, 10).unwrap(),
            )
            .unwrap();

        let report = timing_report_from_net_delays(
            &design,
            &model,
            &TimingConstraints::new(),
            vec![NetDelay {
                net: clock_net,
                sink: clock,
                delay: DelayRange::zero(),
            }],
        )
        .unwrap();

        assert_eq!(report.unchecked_endpoints.len(), 1);
        assert_eq!(report.unchecked_endpoints[0].clock_net, Some(clock_net));
        assert_eq!(
            report.unchecked_endpoints[0].reason,
            UncheckedEndpointReason::UnconstrainedClock
        );
    }

    #[test]
    fn a_hold_violation_blocks_timing_closure() {
        let (design, device, clock_net, model) = registered_path(400);
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

        assert_eq!(report.worst_hold_slack_ps, Some(-140));
        assert!(!report.met_timing());
    }

    #[test]
    #[allow(clippy::too_many_lines)]
    fn removes_common_clock_path_pessimism() {
        let mut design = Design::new();
        let clock = design.add_cell("clock", ResourceKind::Io);
        let clock_o = design.add_pin(clock, "O", PinDirection::Output).unwrap();
        let clock_buffer = design.add_cell("clock-buffer", ResourceKind::Lut(1));
        let buffer_a = design
            .add_pin(clock_buffer, "A", PinDirection::Input)
            .unwrap();
        let buffer_f = design
            .add_pin(clock_buffer, "F", PinDirection::Output)
            .unwrap();
        let register = design.add_cell("register", ResourceKind::Register);
        let register_di = design.add_pin(register, "DI", PinDirection::Input).unwrap();
        let register_clk = design
            .add_pin(register, "CLK", PinDirection::Input)
            .unwrap();
        let register_q = design.add_pin(register, "Q", PinDirection::Output).unwrap();
        design.add_net("clock-input", clock_o, [buffer_a]).unwrap();
        let clock_net = design
            .add_net("clock-tree", buffer_f, [register_clk])
            .unwrap();
        design
            .add_net("feedback", register_q, [register_di])
            .unwrap();

        let mut device = Device::new("cppr", 3, 1).unwrap();
        let clock_wire = device.add_wire("clock", Point::new(0, 0), 1).unwrap();
        let buffer_input_wire = device.add_wire("buffer-a", Point::new(1, 0), 1).unwrap();
        let buffer_output_wire = device.add_wire("buffer-f", Point::new(1, 0), 1).unwrap();
        let register_clk_wire = device
            .add_wire("register-clk", Point::new(2, 0), 1)
            .unwrap();
        let register_di_wire = device.add_wire("register-di", Point::new(2, 0), 1).unwrap();
        let register_q_wire = device.add_wire("register-q", Point::new(2, 0), 1).unwrap();
        let clock_bel = device
            .add_bel("CLOCK", ResourceKind::Io, Point::new(0, 0))
            .unwrap();
        device
            .add_bel_pin(clock_bel, "O", PinDirection::Output, clock_wire)
            .unwrap();
        let buffer_bel = device
            .add_bel("BUFFER", ResourceKind::Lut(1), Point::new(1, 0))
            .unwrap();
        device
            .add_bel_pin(buffer_bel, "A", PinDirection::Input, buffer_input_wire)
            .unwrap();
        device
            .add_bel_pin(buffer_bel, "F", PinDirection::Output, buffer_output_wire)
            .unwrap();
        let register_bel = device
            .add_bel("REGISTER", ResourceKind::Register, Point::new(2, 0))
            .unwrap();
        device
            .add_bel_pin(register_bel, "DI", PinDirection::Input, register_di_wire)
            .unwrap();
        device
            .add_bel_pin(register_bel, "CLK", PinDirection::Input, register_clk_wire)
            .unwrap();
        device
            .add_bel_pin(register_bel, "Q", PinDirection::Output, register_q_wire)
            .unwrap();
        for (from, to) in [
            (clock_wire, buffer_input_wire),
            (buffer_output_wire, register_clk_wire),
            (register_q_wire, register_di_wire),
        ] {
            device.add_pip(from, to, false, 1).unwrap();
        }

        let mut model = TimingModel::new();
        model
            .add_cell_arc(buffer_a, buffer_f, DelayRange::new(100, 500).unwrap())
            .unwrap();
        model
            .add_clock_to_q(register_clk, register_q, DelayRange::new(40, 50).unwrap())
            .unwrap();
        model
            .add_setup_hold(
                register_clk,
                register_di,
                DelayRange::new(10, 10).unwrap(),
                DelayRange::new(100, 100).unwrap(),
            )
            .unwrap();
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

        // The 400 ps min/max width on the common clock-buffer arc cancels.
        // Without CPPR this path is incorrectly reported at -360 ps.
        assert_eq!(report.worst_hold_slack_ps, Some(40));
        assert!(report.met_timing());

        // Independently fitted early/late corners can be numerically inverted.
        // CPPR is a signed corner delta, so swapping the common clock corner
        // values must not change setup or hold slack and must not overflow.
        let mut inverted_model = TimingModel::new();
        inverted_model
            .add_cell_arc(
                buffer_a,
                buffer_f,
                DelayRange::from_independent_corners(500, 100),
            )
            .unwrap();
        inverted_model
            .add_clock_to_q(register_clk, register_q, DelayRange::new(40, 50).unwrap())
            .unwrap();
        inverted_model
            .add_setup_hold(
                register_clk,
                register_di,
                DelayRange::new(10, 10).unwrap(),
                DelayRange::new(100, 100).unwrap(),
            )
            .unwrap();
        let inverted_report = analyze_timing(
            &design,
            &device,
            &implementation,
            &pip_delays,
            &inverted_model,
            &constraints,
        )
        .unwrap();

        assert_eq!(inverted_report.worst_slack_ps, report.worst_slack_ps);
        assert_eq!(
            inverted_report.worst_hold_slack_ps,
            report.worst_hold_slack_ps
        );
        assert!(inverted_report.met_timing());
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
        let launch = design.add_cell("launch", ResourceKind::Register);
        let launch_di = design.add_pin(launch, "DI", PinDirection::Input).unwrap();
        let launch_clk = design.add_pin(launch, "CLK", PinDirection::Input).unwrap();
        let launch_q = design.add_pin(launch, "Q", PinDirection::Output).unwrap();
        let capture = design.add_cell("capture", ResourceKind::Register);
        let capture_di = design.add_pin(capture, "DI", PinDirection::Input).unwrap();
        let capture_clk = design.add_pin(capture, "CLK", PinDirection::Input).unwrap();
        let capture_q = design.add_pin(capture, "Q", PinDirection::Output).unwrap();
        design.add_net("input", input_o, [launch_di]).unwrap();
        design.add_net("launch-q", launch_q, [lut_a]).unwrap();
        design.add_net("data", lut_f, [capture_di]).unwrap();
        let clock_net = design
            .add_net("clock", clock_o, [launch_clk, capture_clk])
            .unwrap();

        let device = timing_test_device();
        let mut model = TimingModel::new();
        model
            .add_cell_arc(lut_a, lut_f, DelayRange::new(20, 30).unwrap())
            .unwrap();
        model
            .add_clock_to_q(launch_clk, launch_q, DelayRange::new(40, 50).unwrap())
            .unwrap();
        model
            .add_setup_hold(
                launch_clk,
                launch_di,
                DelayRange::new(10, 10).unwrap(),
                DelayRange::new(hold_ps, hold_ps).unwrap(),
            )
            .unwrap();
        model
            .add_clock_to_q(capture_clk, capture_q, DelayRange::new(40, 50).unwrap())
            .unwrap();
        model
            .add_setup_hold(
                capture_clk,
                capture_di,
                DelayRange::new(10, 10).unwrap(),
                DelayRange::new(hold_ps, hold_ps).unwrap(),
            )
            .unwrap();
        (design, device, clock_net, model)
    }

    fn timing_test_device() -> Device {
        let mut device = Device::new("timing", 4, 1).unwrap();
        let io_data_wire = device.add_wire("io-data", Point::new(0, 0), 1).unwrap();
        let io_clock_wire = device.add_wire("io-clock", Point::new(0, 0), 1).unwrap();
        let launch_di_wire = device.add_wire("launch-di", Point::new(1, 0), 1).unwrap();
        let launch_clk_wire = device.add_wire("launch-clk", Point::new(1, 0), 1).unwrap();
        let launch_q_wire = device.add_wire("launch-q", Point::new(1, 0), 1).unwrap();
        let logic_input_wire = device.add_wire("lut-a", Point::new(2, 0), 1).unwrap();
        let logic_output_wire = device.add_wire("lut-f", Point::new(2, 0), 1).unwrap();
        let capture_di_wire = device.add_wire("capture-di", Point::new(3, 0), 1).unwrap();
        let capture_clk_wire = device.add_wire("capture-clk", Point::new(3, 0), 1).unwrap();
        let capture_q_wire = device.add_wire("capture-q", Point::new(3, 0), 1).unwrap();

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
        let launch = device
            .add_bel("LAUNCH", ResourceKind::Register, Point::new(1, 0))
            .unwrap();
        device
            .add_bel_pin(launch, "DI", PinDirection::Input, launch_di_wire)
            .unwrap();
        device
            .add_bel_pin(launch, "CLK", PinDirection::Input, launch_clk_wire)
            .unwrap();
        device
            .add_bel_pin(launch, "Q", PinDirection::Output, launch_q_wire)
            .unwrap();
        let lut = device
            .add_bel("LUT", ResourceKind::Lut(4), Point::new(2, 0))
            .unwrap();
        device
            .add_bel_pin(lut, "A", PinDirection::Input, logic_input_wire)
            .unwrap();
        device
            .add_bel_pin(lut, "F", PinDirection::Output, logic_output_wire)
            .unwrap();
        let capture = device
            .add_bel("CAPTURE", ResourceKind::Register, Point::new(3, 0))
            .unwrap();
        device
            .add_bel_pin(capture, "DI", PinDirection::Input, capture_di_wire)
            .unwrap();
        device
            .add_bel_pin(capture, "CLK", PinDirection::Input, capture_clk_wire)
            .unwrap();
        device
            .add_bel_pin(capture, "Q", PinDirection::Output, capture_q_wire)
            .unwrap();

        for (from, to) in [
            (io_data_wire, launch_di_wire),
            (io_clock_wire, launch_di_wire),
            (launch_q_wire, logic_input_wire),
            (logic_output_wire, capture_di_wire),
            (io_clock_wire, launch_clk_wire),
            (io_clock_wire, capture_clk_wire),
            (io_data_wire, launch_clk_wire),
            (io_data_wire, capture_clk_wire),
        ] {
            device.add_pip(from, to, false, 1).unwrap();
        }
        device
    }
}
