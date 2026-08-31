//! Post-route static timing analysis over Texo's unified graph.
//!
//! Both early/minimum and late/maximum propagation are modeled so setup and
//! hold checks can share one characterized target timing model. Register
//! launches are propagated independently per clock net. Related generated
//! clocks are checked from their exact integer frequency ratio and phase;
//! primary inputs and unrelated clocks are not fabricated as synchronous.

use std::collections::{BTreeMap, VecDeque};
use std::error::Error;
use std::fmt;

use texo_model::{
    CellId, CellPinId, Design, Device, ModelError, NetId, PipId, UnifiedGraph, WireId,
};
use texo_pnr::{NetRoute, Placement, PnrResult};

mod clock_relation;

use clock_relation::{
    ClockWaveform, GeneratedClockConstraint, related_edge_offsets, resolve_clock_waveforms,
    validate_clock_relations,
};

/// One trillion picoseconds per second.
pub const PICOSECONDS_PER_SECOND: u64 = 1_000_000_000_000;

/// Active edge of a sequential timing arc.
#[derive(Clone, Copy, Debug, Default, Eq, Ord, PartialEq, PartialOrd)]
pub enum ClockEdge {
    /// Rising-edge triggered.
    #[default]
    Rising,
    /// Falling-edge triggered.
    Falling,
}

impl ClockEdge {
    /// Stable lowercase name for reports and checkpoints.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Rising => "rising",
            Self::Falling => "falling",
        }
    }
}

/// Clock periods and relationships applied to logical clock nets.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct TimingConstraints {
    clock_periods_ps: BTreeMap<NetId, u64>,
    generated_clocks: BTreeMap<NetId, GeneratedClockConstraint>,
    setup_uncertainties_ps: BTreeMap<NetId, u64>,
}

impl TimingConstraints {
    /// Creates an empty constraint set.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            clock_periods_ps: BTreeMap::new(),
            generated_clocks: BTreeMap::new(),
            setup_uncertainties_ps: BTreeMap::new(),
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

    /// Sets or replaces one generated-clock relationship.
    ///
    /// `multiply_by / divide_by` is the generated-to-source frequency ratio;
    /// `phase_ps` is its rising-edge offset from the source clock.
    /// The generated clock and clocks used as sequential endpoints still need
    /// period constraints from [`Self::set_clock_period_ps`]. A source clock
    /// used only as the common relationship root does not need a period.
    /// Invalid zero ratios and relationship cycles are reported by analysis.
    pub fn set_generated_clock(
        &mut self,
        net: NetId,
        source: NetId,
        multiply_by: u32,
        divide_by: u32,
        phase_ps: i64,
    ) {
        self.generated_clocks.insert(
            net,
            GeneratedClockConstraint {
                source,
                multiply_by,
                divide_by,
                phase_ps,
            },
        );
    }

    /// Sets or replaces the setup uncertainty for one clock in picoseconds.
    ///
    /// Setup uncertainty reserves timing margin for clock jitter and residual
    /// error in characterized delay models. It reduces the available data-path
    /// time without changing the nominal clock period.
    pub fn set_setup_uncertainty_ps(&mut self, net: NetId, uncertainty_ps: u64) {
        self.setup_uncertainties_ps.insert(net, uncertainty_ps);
    }

    /// Per-clock setup uncertainties in stable net-ID order.
    #[must_use]
    pub const fn setup_uncertainties_ps(&self) -> &BTreeMap<NetId, u64> {
        &self.setup_uncertainties_ps
    }

    /// Setup uncertainty for one clock, or zero when none was specified.
    #[must_use]
    pub fn setup_uncertainty_ps(&self, net: NetId) -> u64 {
        self.setup_uncertainties_ps.get(&net).copied().unwrap_or(0)
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
    clock_to_q: BTreeMap<CellPinId, (CellPinId, ClockEdge, DelayRange)>,
    setup_holds: BTreeMap<CellPinId, (CellPinId, ClockEdge, DelayRange, DelayRange)>,
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
        edge: ClockEdge,
        delay: DelayRange,
    ) -> Result<(), TimingError> {
        if self
            .clock_to_q
            .insert(output, (clock, edge, delay))
            .is_some()
        {
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
        edge: ClockEdge,
        setup: DelayRange,
        hold: DelayRange,
    ) -> Result<(), TimingError> {
        if self
            .setup_holds
            .insert(signal, (clock, edge, setup, hold))
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
    pub fn clock_to_q(&self, output: CellPinId) -> Option<(CellPinId, ClockEdge, DelayRange)> {
        self.clock_to_q.get(&output).copied()
    }

    /// Clock pin, setup, and hold ranges for one sequential input.
    #[must_use]
    pub fn setup_hold(
        &self,
        signal: CellPinId,
    ) -> Option<(CellPinId, ClockEdge, DelayRange, DelayRange)> {
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

/// Maximum setup criticality of one logical net edge across clock-domain pairs.
///
/// The ratio `path_delay_ps / domain_worst_path_delay_ps` is in `[0, 1]`.
/// Both values are retained instead of rounding to a floating-point number so
/// timing-driven consumers can choose their own precision and exponent.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct NetSetupCriticality {
    /// Logical net.
    pub net: NetId,
    /// Logical sink pin for this fanout edge.
    pub sink: CellPinId,
    /// Constraint-normalized path delay through this edge's most critical
    /// launch/capture domain pair.
    pub path_delay_ps: u128,
    /// Worst constraint-normalized path delay in that same domain pair.
    pub domain_worst_path_delay_ps: u128,
}

/// One register data setup check.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SetupCheck {
    /// Register cell.
    pub cell: CellId,
    /// Register data input pin.
    pub data_pin: CellPinId,
    /// Capture clock net.
    pub clock_net: NetId,
    /// Active edge that launches the worst setup path.
    pub launch_edge: ClockEdge,
    /// Active edge of the capture endpoint.
    pub capture_edge: ClockEdge,
    /// Longest combinational arrival at the data pin.
    pub arrival_ps: u64,
    /// Earliest clock arrival at this register.
    pub clock_arrival_ps: u64,
    /// Latest characterized setup requirement.
    pub setup_ps: u64,
    /// Reserved setup uncertainty for this clock domain.
    pub uncertainty_ps: u64,
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
    /// Capture clock net.
    pub clock_net: NetId,
    /// Active edge that launches the worst hold path.
    pub launch_edge: ClockEdge,
    /// Active edge of the capture endpoint.
    pub capture_edge: ClockEdge,
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
    /// No launch from the capture clock or a related clock reaches the endpoint.
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
    /// Per-sink maximum setup criticality across launch/capture domain pairs.
    pub net_setup_criticalities: Vec<NetSetupCriticality>,
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
    /// and unrelated cross-clock paths may intentionally remain omitted.
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

/// Analyzes timing from caller-supplied per-sink net-delay estimates.
///
/// This is the placement-time counterpart to [`analyze_timing`]. It runs the
/// same cell-delay propagation, clock-domain checks, required-time
/// propagation, and per-edge slack calculation without requiring a physical
/// route. Callers can therefore drive placement from a complete timing graph
/// instead of using routed trials as the placement objective.
///
/// # Errors
///
/// Returns an error for invalid constraints or model data, missing net-edge
/// estimates, arithmetic overflow, or a combinational cycle.
pub fn analyze_timing_from_net_delays(
    design: &Design,
    model: &TimingModel,
    constraints: &TimingConstraints,
    net_delays: Vec<NetDelay>,
) -> Result<TimingReport, TimingError> {
    validate_constraints(design, constraints)?;
    validate_model(design, model)?;
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
#[allow(clippy::too_many_lines)]
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
    let mut register_starts_by_clock =
        BTreeMap::<(NetId, ClockEdge), BTreeMap<CellPinId, DelayRange>>::new();
    for (&output, &(clock, edge, delay)) in &model.clock_to_q {
        let Some(clock_net) = design.pins()[clock.0].net() else {
            continue;
        };
        let clock_arrival = clock_arrivals[clock.0].unwrap_or(DelayRange::zero());
        register_starts_by_clock
            .entry((clock_net, edge))
            .or_default()
            .insert(output, clock_arrival.checked_add(delay)?);
    }
    let arrivals_by_clock = register_starts_by_clock
        .iter()
        .map(|(&clock, starts)| {
            Ok((
                clock,
                pin_arrivals(design, &delays_by_sink, model, starts, false)?,
            ))
        })
        .collect::<Result<BTreeMap<_, _>, TimingError>>()?;

    let clock_nets = arrivals_by_clock.keys().map(|&(net, _)| net).chain(
        model
            .setup_holds
            .values()
            .filter_map(|&(clock, _, _, _)| design.pins()[clock.0].net()),
    );
    let clock_waveforms = resolve_clock_waveforms(constraints, clock_nets)?;

    let mut selected_setup_checks = Vec::new();
    let mut domain_setup_checks = Vec::new();
    let mut selected_hold_checks = Vec::new();
    let mut unchecked_endpoints = Vec::new();
    for (&data_pin, &(clock_pin, capture_edge, setup, hold)) in &model.setup_holds {
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
        let launch_arrivals = synchronous_launch_arrivals(
            &arrivals_by_clock,
            &clock_waveforms,
            clock_net,
            period_ps,
            capture_edge,
            data_pin,
        )?;
        if launch_arrivals.is_empty() {
            // Primary-input and unrelated cross-clock paths need explicit
            // timing constraints. Do not invent a synchronous launch at zero.
            unchecked_endpoints.push(unchecked_endpoint(
                design,
                data_pin,
                clock_pin,
                Some(clock_net),
                UncheckedEndpointReason::NoSynchronousLaunch,
            ));
            continue;
        }
        let clock_arrival = clock_arrivals[clock_pin.0].unwrap_or(DelayRange::zero());
        let checks = launch_arrivals
            .into_iter()
            .map(|launch| {
                let common_clock_arrival = if launch.clock_net == clock_net {
                    clock_arrivals[design.nets()[clock_net.0].driver.0]
                        .unwrap_or(DelayRange::zero())
                } else {
                    // Distinct generated clock trees are related in time, but
                    // their physically common path is not represented by either
                    // logical output net. Omitting CPPR here is conservative.
                    DelayRange::zero()
                };
                endpoint_checks(EndpointCheckContext {
                    design,
                    data_pin,
                    clock_net,
                    launch_clock_net: launch.clock_net,
                    uncertainty_ps: constraints.setup_uncertainty_ps(clock_net),
                    arrival: launch.arrival,
                    clock_arrival,
                    common_clock_arrival,
                    setup,
                    hold,
                    launch_edge: launch.edge,
                    capture_edge,
                    setup_edge_separation_ps: launch.setup_edge_separation_ps,
                    hold_launch_offset_ps: launch.hold_launch_offset_ps,
                })
            })
            .collect::<Vec<_>>();
        domain_setup_checks.extend(checks.iter().map(|(setup, _)| *setup));
        let (setup_check, hold_check) = select_worst_edge_checks(checks);
        selected_setup_checks.push(setup_check);
        selected_hold_checks.push(hold_check);
    }
    let setup_checks = selected_setup_checks
        .iter()
        .map(|selected| selected.check)
        .collect::<Vec<_>>();
    let hold_checks = selected_hold_checks
        .iter()
        .map(|selected| selected.check)
        .collect::<Vec<_>>();
    let worst_slack_ps = setup_checks.iter().map(|check| check.slack_ps).min();
    let worst_hold_slack_ps = hold_checks.iter().map(|check| check.slack_ps).min();
    let (net_setup_slacks, net_setup_criticalities) = net_setup_metrics(
        design,
        &net_delays,
        model,
        &arrivals_by_clock,
        &domain_setup_checks,
    )?;
    Ok(TimingReport {
        net_delays,
        net_setup_slacks,
        net_setup_criticalities,
        setup_checks,
        hold_checks,
        unchecked_endpoints,
        worst_slack_ps,
        worst_hold_slack_ps,
    })
}

fn synchronous_launch_arrivals(
    arrivals_by_clock: &BTreeMap<(NetId, ClockEdge), Vec<Option<DelayRange>>>,
    clock_waveforms: &BTreeMap<NetId, ClockWaveform>,
    capture_clock_net: NetId,
    capture_period_ps: u64,
    capture_edge: ClockEdge,
    data_pin: CellPinId,
) -> Result<Vec<SynchronousLaunch>, TimingError> {
    let capture_waveform = clock_waveforms
        .get(&capture_clock_net)
        .copied()
        .expect("capture clock waveform was resolved");
    let mut launches = Vec::new();
    for (&(launch_clock_net, launch_edge), arrivals) in arrivals_by_clock {
        let Some(arrival) = arrivals[data_pin.0] else {
            continue;
        };
        let launch_waveform = clock_waveforms
            .get(&launch_clock_net)
            .copied()
            .expect("launch clock waveform was resolved");
        if launch_waveform.root != capture_waveform.root {
            continue;
        }
        let offsets = related_edge_offsets(
            launch_waveform,
            capture_waveform,
            capture_period_ps,
            launch_edge,
            capture_edge,
        )?;
        launches.push(SynchronousLaunch {
            clock_net: launch_clock_net,
            edge: launch_edge,
            arrival,
            setup_edge_separation_ps: offsets.setup_ps,
            hold_launch_offset_ps: offsets.hold_ps,
        });
    }
    Ok(launches)
}

#[derive(Clone, Copy)]
struct SynchronousLaunch {
    clock_net: NetId,
    edge: ClockEdge,
    arrival: DelayRange,
    setup_edge_separation_ps: u64,
    hold_launch_offset_ps: u64,
}

fn select_worst_edge_checks(
    checks: impl IntoIterator<
        Item = (
            SelectedClockCheck<SetupCheck>,
            SelectedClockCheck<HoldCheck>,
        ),
    >,
) -> (
    SelectedClockCheck<SetupCheck>,
    SelectedClockCheck<HoldCheck>,
) {
    checks
        .into_iter()
        .reduce(|(setup_best, hold_best), (setup, hold)| {
            (
                if setup.check.slack_ps < setup_best.check.slack_ps {
                    setup
                } else {
                    setup_best
                },
                if hold.check.slack_ps < hold_best.check.slack_ps {
                    hold
                } else {
                    hold_best
                },
            )
        })
        .expect("at least one synchronous launch was checked")
}

#[derive(Clone, Copy)]
struct SelectedClockCheck<T> {
    check: T,
    launch_clock_net: NetId,
    setup_edge_separation_ps: u64,
}

#[derive(Clone, Copy)]
struct EndpointCheckContext<'a> {
    design: &'a Design,
    data_pin: CellPinId,
    clock_net: NetId,
    launch_clock_net: NetId,
    uncertainty_ps: u64,
    arrival: DelayRange,
    clock_arrival: DelayRange,
    common_clock_arrival: DelayRange,
    setup: DelayRange,
    hold: DelayRange,
    launch_edge: ClockEdge,
    capture_edge: ClockEdge,
    setup_edge_separation_ps: u64,
    hold_launch_offset_ps: u64,
}

fn endpoint_checks(
    context: EndpointCheckContext<'_>,
) -> (
    SelectedClockCheck<SetupCheck>,
    SelectedClockCheck<HoldCheck>,
) {
    let EndpointCheckContext {
        design,
        data_pin,
        clock_net,
        launch_clock_net,
        uncertainty_ps,
        arrival,
        clock_arrival,
        common_clock_arrival,
        setup,
        hold,
        launch_edge,
        capture_edge,
        setup_edge_separation_ps,
        hold_launch_offset_ps,
    } = context;
    // Every launch in this analysis group and the capture endpoint share the
    // path up to the constrained clock net's driver. Its corner range is
    // common-mode delay, not clock skew, and therefore cancels through CPPR.
    // Early and late values are independently fitted and need not be ordered,
    // so the correction must remain signed.
    let common_clock_pessimism_ps =
        i128::from(common_clock_arrival.max_ps) - i128::from(common_clock_arrival.min_ps);
    let setup_required_ps = i128::from(setup_edge_separation_ps)
        + i128::from(clock_arrival.min_ps)
        + common_clock_pessimism_ps
        - i128::from(setup.max_ps)
        - i128::from(uncertainty_ps);
    let hold_required_ps =
        i128::from(clock_arrival.max_ps) + i128::from(hold.max_ps) - common_clock_pessimism_ps;
    (
        SelectedClockCheck {
            launch_clock_net,
            setup_edge_separation_ps,
            check: SetupCheck {
                cell: design.pins()[data_pin.0].cell,
                data_pin,
                clock_net,
                launch_edge,
                capture_edge,
                arrival_ps: arrival.max_ps,
                clock_arrival_ps: clock_arrival.min_ps,
                setup_ps: setup.max_ps,
                uncertainty_ps,
                required_ps: setup_required_ps,
                slack_ps: setup_required_ps - i128::from(arrival.max_ps),
            },
        },
        SelectedClockCheck {
            launch_clock_net,
            setup_edge_separation_ps,
            check: HoldCheck {
                cell: design.pins()[data_pin.0].cell,
                data_pin,
                clock_net,
                launch_edge,
                capture_edge,
                arrival_ps: arrival.min_ps.saturating_add(hold_launch_offset_ps),
                clock_arrival_ps: clock_arrival.max_ps,
                hold_ps: hold.max_ps,
                required_ps: hold_required_ps,
                slack_ps: i128::from(arrival.min_ps) + i128::from(hold_launch_offset_ps)
                    - hold_required_ps,
            },
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

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct SetupDomainPair {
    launch_clock_net: NetId,
    launch_edge: ClockEdge,
    capture_clock_net: NetId,
    capture_edge: ClockEdge,
    edge_separation_ps: u64,
}

struct SetupPropagationGraph {
    edges: Vec<Vec<(CellPinId, u64)>>,
    order: Vec<CellPinId>,
}

fn setup_propagation_graph(
    design: &Design,
    net_delays: &[NetDelay],
    model: &TimingModel,
) -> Result<SetupPropagationGraph, TimingError> {
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
    let mut ready = indegree
        .iter()
        .enumerate()
        .filter_map(|(index, &degree)| (degree == 0).then_some(CellPinId(index)))
        .collect::<VecDeque<_>>();
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
    Ok(SetupPropagationGraph { edges, order })
}

fn net_setup_metrics(
    design: &Design,
    net_delays: &[NetDelay],
    model: &TimingModel,
    arrivals_by_clock: &BTreeMap<(NetId, ClockEdge), Vec<Option<DelayRange>>>,
    setup_checks: &[SelectedClockCheck<SetupCheck>],
) -> Result<(Vec<NetSetupSlack>, Vec<NetSetupCriticality>), TimingError> {
    let propagation = setup_propagation_graph(design, net_delays, model)?;
    let mut checks_by_domain = BTreeMap::<SetupDomainPair, Vec<SetupCheck>>::new();
    for selected in setup_checks {
        let check = selected.check;
        checks_by_domain
            .entry(SetupDomainPair {
                launch_clock_net: selected.launch_clock_net,
                launch_edge: check.launch_edge,
                capture_clock_net: check.clock_net,
                capture_edge: check.capture_edge,
                edge_separation_ps: selected.setup_edge_separation_ps,
            })
            .or_default()
            .push(check);
    }

    let mut slacks = BTreeMap::<(NetId, CellPinId), i128>::new();
    let mut criticalities = BTreeMap::<(NetId, CellPinId), (u128, u128)>::new();
    for (domain, checks) in checks_by_domain {
        let Some(arrivals) = arrivals_by_clock.get(&(domain.launch_clock_net, domain.launch_edge))
        else {
            continue;
        };
        let mut required = vec![None::<i128>; design.pins().len()];
        for check in &checks {
            let entry = &mut required[check.data_pin.0];
            *entry = Some(entry.map_or(check.required_ps, |known| known.min(check.required_ps)));
        }
        for &from in propagation.order.iter().rev() {
            for &(to, delay_ps) in &propagation.edges[from.0] {
                let Some(to_required) = required[to.0] else {
                    continue;
                };
                let candidate = to_required - i128::from(delay_ps);
                let entry = &mut required[from.0];
                *entry = Some(entry.map_or(candidate, |known| known.min(candidate)));
            }
        }
        let domain_worst_path_delay_ps = checks
            .iter()
            .map(|check| constrained_path_delay_ps(domain.edge_separation_ps, check.slack_ps))
            .max()
            .unwrap_or(0);
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
            if domain_worst_path_delay_ps == 0 {
                continue;
            }
            let path_delay_ps = constrained_path_delay_ps(domain.edge_separation_ps, slack_ps)
                .min(domain_worst_path_delay_ps);
            criticalities
                .entry((delay.net, delay.sink))
                .and_modify(|known| {
                    if compare_nonnegative_fractions(
                        path_delay_ps,
                        domain_worst_path_delay_ps,
                        known.0,
                        known.1,
                    )
                    .is_gt()
                    {
                        *known = (path_delay_ps, domain_worst_path_delay_ps);
                    }
                })
                .or_insert((path_delay_ps, domain_worst_path_delay_ps));
        }
    }

    let slacks = slacks
        .into_iter()
        .map(|((net, sink), slack_ps)| NetSetupSlack {
            net,
            sink,
            slack_ps,
        })
        .collect();
    let criticalities = criticalities
        .into_iter()
        .map(
            |((net, sink), (path_delay_ps, domain_worst_path_delay_ps))| NetSetupCriticality {
                net,
                sink,
                path_delay_ps,
                domain_worst_path_delay_ps,
            },
        )
        .collect();
    Ok((slacks, criticalities))
}

fn constrained_path_delay_ps(edge_separation_ps: u64, slack_ps: i128) -> u128 {
    let separation = u128::from(edge_separation_ps);
    if slack_ps >= 0 {
        separation.saturating_sub(slack_ps.unsigned_abs())
    } else {
        separation.saturating_add(slack_ps.unsigned_abs())
    }
}

fn compare_nonnegative_fractions(
    mut left_numerator: u128,
    mut left_denominator: u128,
    mut right_numerator: u128,
    mut right_denominator: u128,
) -> std::cmp::Ordering {
    debug_assert!(left_denominator > 0 && right_denominator > 0);
    let mut reverse = false;
    loop {
        let left_quotient = left_numerator / left_denominator;
        let right_quotient = right_numerator / right_denominator;
        if left_quotient != right_quotient {
            let ordering = left_quotient.cmp(&right_quotient);
            return if reverse {
                ordering.reverse()
            } else {
                ordering
            };
        }
        let left_remainder = left_numerator % left_denominator;
        let right_remainder = right_numerator % right_denominator;
        match (left_remainder == 0, right_remainder == 0) {
            (true, true) => return std::cmp::Ordering::Equal,
            (true, false) => {
                return if reverse {
                    std::cmp::Ordering::Greater
                } else {
                    std::cmp::Ordering::Less
                };
            }
            (false, true) => {
                return if reverse {
                    std::cmp::Ordering::Less
                } else {
                    std::cmp::Ordering::Greater
                };
            }
            (false, false) => {}
        }
        (left_numerator, left_denominator) = (left_denominator, left_remainder);
        (right_numerator, right_denominator) = (right_denominator, right_remainder);
        reverse = !reverse;
    }
}

fn validate_model(design: &Design, model: &TimingModel) -> Result<(), TimingError> {
    for &(from, to) in model.cell_arcs.keys() {
        validate_pin_pair(design, from, to)?;
    }
    for (&output, &(clock, _, _)) in &model.clock_to_q {
        validate_pin_pair(design, clock, output)?;
    }
    for (&signal, &(clock, _, _, _)) in &model.setup_holds {
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
    validate_clock_relations(constraints, design.nets().len())
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
    /// A generated clock has a zero frequency ratio component.
    InvalidGeneratedClockRatio(NetId),
    /// Generated-clock source relationships contain a cycle.
    GeneratedClockCycle(NetId),
    /// Generated-clock ratio or phase arithmetic overflowed.
    ClockRelationOverflow,
    /// Related constrained clocks disagree with their declared frequency ratio.
    InconsistentRelatedClockPeriods {
        /// First conflicting logical clock net.
        first: NetId,
        /// Second conflicting logical clock net.
        second: NetId,
    },
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
            Self::InvalidGeneratedClockRatio(net) => write!(
                f,
                "generated clock net {} has a zero ratio component",
                net.0
            ),
            Self::GeneratedClockCycle(net) => write!(
                f,
                "generated-clock relationships contain a cycle at net {}",
                net.0
            ),
            Self::ClockRelationOverflow => {
                write!(f, "generated-clock relationship arithmetic overflowed")
            }
            Self::InconsistentRelatedClockPeriods { first, second } => write!(
                f,
                "related clock nets {} and {} have inconsistent periods",
                first.0, second.0
            ),
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
        ClockEdge, DelayRange, NetDelay, TimingConstraints, TimingModel, UncheckedEndpointReason,
        analyze_timing, analyze_timing_from_net_delays, compare_nonnegative_fractions,
        timing_report_from_net_delays,
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
        assert_eq!(failed.net_setup_criticalities.len(), 2);
        assert!(
            failed.net_setup_criticalities.iter().all(|edge| {
                edge.path_delay_ps == 290 && edge.domain_worst_path_delay_ps == 290
            })
        );
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
        assert_eq!(
            passed.net_setup_criticalities,
            failed.net_setup_criticalities
        );
        assert!(passed.met_timing());

        constraints.set_setup_uncertainty_ps(clock_net, 20);
        let guarded = analyze_timing(
            &design,
            &device,
            &implementation,
            &pip_delays,
            &model,
            &constraints,
        )
        .unwrap();
        assert_eq!(guarded.setup_checks[0].uncertainty_ps, 20);
        assert_eq!(guarded.worst_slack_ps, Some(-10));
        assert!(
            guarded
                .net_setup_slacks
                .iter()
                .all(|edge| edge.slack_ps == -10)
        );
        assert!(!guarded.met_timing());
    }

    #[test]
    fn fraction_comparison_never_cross_multiplies() {
        use std::cmp::Ordering;

        assert_eq!(compare_nonnegative_fractions(1, 2, 2, 4), Ordering::Equal);
        assert_eq!(compare_nonnegative_fractions(1, 3, 1, 2), Ordering::Less);
        assert_eq!(compare_nonnegative_fractions(3, 4, 2, 3), Ordering::Greater);
        assert_eq!(
            compare_nonnegative_fractions(u128::MAX - 1, u128::MAX, 1, 2),
            Ordering::Greater
        );
    }

    #[test]
    fn opposite_edge_path_uses_half_a_clock_period() {
        let (design, device, clock_net, model) =
            registered_path_edges(10, ClockEdge::Falling, ClockEdge::Rising);
        let implementation = place_and_route(&design, &device).unwrap();
        let pip_delays = device
            .pips()
            .iter()
            .enumerate()
            .map(|(index, _)| (texo_model::PipId(index), DelayRange::new(100, 100).unwrap()))
            .collect::<BTreeMap<_, _>>();
        let mut constraints = TimingConstraints::new();
        constraints.set_clock_period_ps(clock_net, 300);

        let report = analyze_timing(
            &design,
            &device,
            &implementation,
            &pip_delays,
            &model,
            &constraints,
        )
        .unwrap();

        assert_eq!(report.worst_slack_ps, Some(-140));
        assert_eq!(report.setup_checks[0].launch_edge, ClockEdge::Falling);
        assert_eq!(report.setup_checks[0].capture_edge, ClockEdge::Rising);
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
                ClockEdge::Rising,
                DelayRange::new(10, 10).unwrap(),
                DelayRange::new(10, 10).unwrap(),
            )
            .unwrap();

        let report =
            analyze_timing_from_net_delays(&design, &model, &TimingConstraints::new(), Vec::new())
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
                ClockEdge::Rising,
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
            .add_clock_to_q(
                register_clk,
                register_q,
                ClockEdge::Rising,
                DelayRange::new(40, 50).unwrap(),
            )
            .unwrap();
        model
            .add_setup_hold(
                register_clk,
                register_di,
                ClockEdge::Rising,
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
            .add_clock_to_q(
                register_clk,
                register_q,
                ClockEdge::Rising,
                DelayRange::new(40, 50).unwrap(),
            )
            .unwrap();
        inverted_model
            .add_setup_hold(
                register_clk,
                register_di,
                ClockEdge::Rising,
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
        registered_path_edges(hold_ps, ClockEdge::Rising, ClockEdge::Rising)
    }

    fn registered_path_edges(
        hold_ps: u64,
        launch_edge: ClockEdge,
        capture_edge: ClockEdge,
    ) -> (Design, Device, texo_model::NetId, TimingModel) {
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
            .add_clock_to_q(
                launch_clk,
                launch_q,
                launch_edge,
                DelayRange::new(40, 50).unwrap(),
            )
            .unwrap();
        model
            .add_setup_hold(
                launch_clk,
                launch_di,
                launch_edge,
                DelayRange::new(10, 10).unwrap(),
                DelayRange::new(hold_ps, hold_ps).unwrap(),
            )
            .unwrap();
        model
            .add_clock_to_q(
                capture_clk,
                capture_q,
                capture_edge,
                DelayRange::new(40, 50).unwrap(),
            )
            .unwrap();
        model
            .add_setup_hold(
                capture_clk,
                capture_di,
                capture_edge,
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
