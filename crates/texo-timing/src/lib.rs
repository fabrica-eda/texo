//! Post-route static timing analysis over Texo's unified graph.
//!
//! The first delay model accounts for selected routing PIPs. Cell-internal,
//! clock-to-Q, setup, and hold delays are intentionally zero until target
//! timing tables are imported.

use std::cmp::Reverse;
use std::collections::{BTreeMap, BinaryHeap, VecDeque};
use std::error::Error;
use std::fmt;

use texo_model::{
    CellId, CellPinId, Design, Device, ModelError, NetId, PinDirection, PipId, ResourceKind,
    UnifiedGraph, WireId,
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

/// Post-route delay from one logical net driver to one sink pin.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct NetDelay {
    /// Logical net.
    pub net: NetId,
    /// Logical sink pin.
    pub sink: CellPinId,
    /// Sum of selected PIP delays on the route to this sink.
    pub delay_ps: u64,
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
    /// Clock routing arrival at this register.
    pub clock_arrival_ps: u64,
    /// Required arrival, including the zero setup-time model.
    pub required_ps: u64,
    /// Required minus actual arrival.
    pub slack_ps: i128,
}

/// Complete result of one static timing pass.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TimingReport {
    /// Per-sink routed net delays.
    pub net_delays: Vec<NetDelay>,
    /// Constrained register setup checks.
    pub setup_checks: Vec<SetupCheck>,
    /// Smallest setup slack, or `None` when no endpoint was constrained.
    pub worst_slack_ps: Option<i128>,
}

impl TimingReport {
    /// Whether at least one setup endpoint was checked and every check passed.
    #[must_use]
    pub fn met_timing(&self) -> bool {
        self.worst_slack_ps.is_some_and(|slack| slack >= 0)
    }
}

/// Analyzes routed delays and zero-internal-delay setup paths.
///
/// `pip_delays_ps` must contain every PIP selected by the router. Register
/// outputs and primary/constant outputs are path starts. LUT, generic logic,
/// and clock buffers are combinational; register and memory cells terminate
/// propagation.
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
    pip_delays_ps: &BTreeMap<PipId, u64>,
    constraints: &TimingConstraints,
) -> Result<TimingReport, TimingError> {
    validate_constraints(design, constraints)?;
    let net_delays = routed_net_delays(design, device, implementation, pip_delays_ps)?;
    let delays_by_sink = net_delays
        .iter()
        .map(|delay| ((delay.net, delay.sink), delay.delay_ps))
        .collect::<BTreeMap<_, _>>();
    let arrivals = pin_arrivals(design, &delays_by_sink)?;

    let mut setup_checks = Vec::new();
    for (cell_index, cell) in design.cells().iter().enumerate() {
        if cell.kind != ResourceKind::Register {
            continue;
        }
        let data_pin = named_pin(design, cell.pins(), "DI");
        let clock_pin = named_pin(design, cell.pins(), "CLK");
        let (Some(data_pin), Some(clock_pin)) = (data_pin, clock_pin) else {
            continue;
        };
        let Some(clock_net) = design.pins()[clock_pin.0].net() else {
            continue;
        };
        let Some(&period_ps) = constraints.clock_periods_ps.get(&clock_net) else {
            continue;
        };
        let clock_arrival_ps = delays_by_sink
            .get(&(clock_net, clock_pin))
            .copied()
            .unwrap_or(0);
        let required_ps = period_ps
            .checked_add(clock_arrival_ps)
            .ok_or(TimingError::DelayOverflow)?;
        let arrival_ps = arrivals[data_pin.0].unwrap_or(0);
        setup_checks.push(SetupCheck {
            cell: CellId(cell_index),
            data_pin,
            clock_net,
            arrival_ps,
            clock_arrival_ps,
            required_ps,
            slack_ps: i128::from(required_ps) - i128::from(arrival_ps),
        });
    }
    let worst_slack_ps = setup_checks.iter().map(|check| check.slack_ps).min();
    Ok(TimingReport {
        net_delays,
        setup_checks,
        worst_slack_ps,
    })
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
    pip_delays_ps: &BTreeMap<PipId, u64>,
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
        let distances = route_distances(route, driver_wire, device, pip_delays_ps)?;
        for &sink in &net.sinks {
            let sink_wire = bound_wire(&graph, &implementation.placement, sink, device)?;
            let delay_ps = distances
                .get(&sink_wire)
                .copied()
                .ok_or(TimingError::UnreachableSink { net: net_id, sink })?;
            result.push(NetDelay {
                net: net_id,
                sink,
                delay_ps,
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
    pip_delays_ps: &BTreeMap<PipId, u64>,
) -> Result<BTreeMap<WireId, u64>, TimingError> {
    if source.0 >= device.wires().len() {
        return Err(TimingError::Model(ModelError::UnknownWire(source)));
    }
    let mut adjacency: BTreeMap<WireId, Vec<(WireId, u64)>> = BTreeMap::new();
    for &pip_id in &route.pips {
        let pip = device
            .pips()
            .get(pip_id.0)
            .ok_or(TimingError::UnknownRoutedPip(pip_id))?;
        let delay = pip_delays_ps
            .get(&pip_id)
            .copied()
            .ok_or(TimingError::MissingPipDelay(pip_id))?;
        adjacency.entry(pip.from).or_default().push((pip.to, delay));
        if pip.bidirectional {
            adjacency.entry(pip.to).or_default().push((pip.from, delay));
        }
    }

    let mut distances = BTreeMap::from([(source, 0_u64)]);
    let mut pending = BinaryHeap::from([Reverse((0_u64, source))]);
    while let Some(Reverse((distance, wire))) = pending.pop() {
        if distances.get(&wire).copied() != Some(distance) {
            continue;
        }
        for &(next, edge_delay) in adjacency.get(&wire).map_or(&[][..], Vec::as_slice) {
            let candidate = distance
                .checked_add(edge_delay)
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
    delays: &BTreeMap<(NetId, CellPinId), u64>,
) -> Result<Vec<Option<u64>>, TimingError> {
    let mut edges = vec![Vec::<(CellPinId, u64)>::new(); design.pins().len()];
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
    for cell in design.cells() {
        if !is_combinational(cell.kind) {
            continue;
        }
        let inputs = cell
            .pins()
            .iter()
            .copied()
            .filter(|pin| design.pins()[pin.0].direction != PinDirection::Output)
            .collect::<Vec<_>>();
        let outputs = cell
            .pins()
            .iter()
            .copied()
            .filter(|pin| design.pins()[pin.0].direction != PinDirection::Input)
            .collect::<Vec<_>>();
        for &input in &inputs {
            for &output in &outputs {
                if input != output {
                    edges[input.0].push((output, 0));
                    indegree[output.0] += 1;
                }
            }
        }
    }

    let mut arrivals: Vec<Option<u64>> = vec![None; design.pins().len()];
    let mut ready = VecDeque::new();
    for (index, &degree) in indegree.iter().enumerate() {
        if degree == 0 {
            arrivals[index] = Some(0);
            ready.push_back(CellPinId(index));
        }
    }
    let mut visited = 0_usize;
    while let Some(pin) = ready.pop_front() {
        visited += 1;
        let arrival = arrivals[pin.0].unwrap_or(0);
        for &(next, delay) in &edges[pin.0] {
            let candidate = arrival
                .checked_add(delay)
                .ok_or(TimingError::DelayOverflow)?;
            arrivals[next.0] =
                Some(arrivals[next.0].map_or(candidate, |known| known.max(candidate)));
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

const fn is_combinational(kind: ResourceKind) -> bool {
    matches!(
        kind,
        ResourceKind::Logic | ResourceKind::Lut(_) | ResourceKind::Clock
    )
}

fn named_pin(design: &Design, pins: &[CellPinId], name: &str) -> Option<CellPinId> {
    pins.iter()
        .copied()
        .find(|pin| design.pins()[pin.0].name == name)
}

/// Static timing model or analysis failure.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TimingError {
    /// Logical/physical graph validation failed.
    Model(ModelError),
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
    /// The zero-cell-delay timing graph contains a combinational cycle.
    CombinationalCycle,
}

impl fmt::Display for TimingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Model(error) => write!(f, "invalid timing graph: {error}"),
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

    use super::{TimingConstraints, analyze_timing};

    #[test]
    fn reports_positive_and_negative_post_route_setup_slack() {
        let (design, device, clock_net) = registered_path();
        let implementation = place_and_route(&design, &device).unwrap();
        let pip_delays = device
            .pips()
            .iter()
            .enumerate()
            .map(|(index, _)| (texo_model::PipId(index), 100_u64))
            .collect::<BTreeMap<_, _>>();

        let mut constraints = TimingConstraints::new();
        constraints.set_clock_period_ps(clock_net, 50);
        let failed =
            analyze_timing(&design, &device, &implementation, &pip_delays, &constraints).unwrap();
        assert_eq!(failed.setup_checks.len(), 1);
        assert_eq!(failed.worst_slack_ps, Some(-50));
        assert!(!failed.met_timing());

        constraints.set_clock_period_ps(clock_net, 150);
        let passed =
            analyze_timing(&design, &device, &implementation, &pip_delays, &constraints).unwrap();
        assert_eq!(passed.worst_slack_ps, Some(50));
        assert!(passed.met_timing());
    }

    fn registered_path() -> (Design, Device, texo_model::NetId) {
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
        design.add_pin(ff, "Q", PinDirection::Output).unwrap();
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
        (design, device, clock_net)
    }
}
