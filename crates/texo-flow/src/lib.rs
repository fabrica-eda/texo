//! Flow orchestration and explicit verification evidence.

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;

use texo_model::{CellId, Design, Device, NetId, PinDirection, PipId};
use texo_pnr::{PlacementConstraints, PnrError, PnrResult, place_and_route_with_constraints};
use texo_struo::{ImportedEcp5Design, PrimitiveMetadata};
use texo_target_ecp5::{
    BlockRamRequirement, DEFAULT_GLOBAL_CLOCK_FANOUT, Ecp5Architecture, Ecp5Packing,
    LpfConstraints, LpfError, PackingError, find_global_clock_requirements, pack_lut_ffs,
    resolve_lpf_port_cells,
};
use texo_timing::{
    PICOSECONDS_PER_SECOND, TimingConstraints, TimingError, TimingReport, analyze_timing,
};

/// Evidence required before a programmable artifact may be released.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum Gate {
    /// Source-level functional simulation passed.
    RtlSimulation,
    /// Synthesized logic is equivalent to the RTL reference.
    SynthesisEquivalence,
    /// No unresolved mapped primitive remains.
    MappedNetlistComplete,
    /// Celox post-map simulation passed.
    PostMapSimulation,
    /// `PnR` completed and independent physical checks passed.
    PhysicalImplementation,
    /// Static timing constraints were met.
    TimingClosure,
}

/// Accumulated immutable-style verification record.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Evidence {
    passed: BTreeSet<Gate>,
}

impl Evidence {
    /// Creates an empty evidence set.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            passed: BTreeSet::new(),
        }
    }

    /// Records a passed gate.
    pub fn record(&mut self, gate: Gate) {
        self.passed.insert(gate);
    }

    /// Whether a gate has passed.
    #[must_use]
    pub fn contains(&self, gate: Gate) -> bool {
        self.passed.contains(&gate)
    }

    /// Checks all bitstream release gates.
    ///
    /// # Errors
    ///
    /// Returns every missing gate.
    pub fn authorize_bitstream(&self) -> Result<(), MissingEvidence> {
        let missing = REQUIRED_GATES
            .into_iter()
            .filter(|gate| !self.contains(*gate))
            .collect::<Vec<_>>();
        if missing.is_empty() {
            Ok(())
        } else {
            Err(MissingEvidence { missing })
        }
    }
}

const REQUIRED_GATES: [Gate; 6] = [
    Gate::RtlSimulation,
    Gate::SynthesisEquivalence,
    Gate::MappedNetlistComplete,
    Gate::PostMapSimulation,
    Gate::PhysicalImplementation,
    Gate::TimingClosure,
];

/// Runs the physical implementation stage and records its evidence.
///
/// # Errors
///
/// Propagates placement or routing failures without recording the gate.
pub fn implement(
    design: &Design,
    device: &Device,
    evidence: &mut Evidence,
) -> Result<PnrResult, PnrError> {
    implement_with_constraints(design, device, &PlacementConstraints::new(), evidence)
}

/// Runs physical implementation with target packing/placement constraints.
///
/// # Errors
///
/// Propagates constraint, placement, or routing failures without recording the
/// physical implementation gate.
pub fn implement_with_constraints(
    design: &Design,
    device: &Device,
    constraints: &PlacementConstraints,
    evidence: &mut Evidence,
) -> Result<PnrResult, PnrError> {
    let result = place_and_route_with_constraints(design, device, constraints)?;
    evidence.record(Gate::PhysicalImplementation);
    Ok(result)
}

/// Runs a caller-provided Celox post-map testbench and records its gate.
///
/// The evidence is changed only when the testbench returns successfully.
///
/// # Errors
///
/// Propagates the testbench error without recording `PostMapSimulation`.
pub fn verify_post_map_with_celox<E>(
    evidence: &mut Evidence,
    testbench: impl FnOnce() -> Result<(), E>,
) -> Result<(), E> {
    testbench()?;
    evidence.record(Gate::PostMapSimulation);
    Ok(())
}

/// Configuration for the complete Struo-to-ECP5 physical implementation flow.
#[derive(Clone, Copy, Debug)]
pub struct Ecp5FlowOptions<'a> {
    /// Exact architecture package used to resolve LPF pin names.
    pub package: Option<&'a str>,
    /// Parsed LPF constraints, when supplied by the user.
    pub lpf: Option<&'a LpfConstraints>,
    /// Whether LPF resolution may leave top-level IO bits unconstrained.
    pub allow_unconstrained_io: bool,
    /// Minimum recognized clock-pin fanout for automatic DCCA promotion.
    pub global_clock_fanout: usize,
}

impl Default for Ecp5FlowOptions<'_> {
    fn default() -> Self {
        Self {
            package: None,
            lpf: None,
            allow_unconstrained_io: false,
            global_clock_fanout: DEFAULT_GLOBAL_CLOCK_FANOUT,
        }
    }
}

/// Owned output of a successful Struo-to-ECP5 implementation run.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Ecp5FlowResult {
    /// Logical design after target-inserted resources such as DCCA buffers.
    pub design: Design,
    /// Original mapped primitive configuration indexed by stable cell ID.
    pub primitive_metadata: BTreeMap<CellId, PrimitiveMetadata>,
    /// Constant primitive inputs absorbed into ECP5 configuration muxes.
    pub absorbed_inputs: BTreeMap<CellId, BTreeMap<String, bool>>,
    /// Target packing decisions and placement constraints.
    pub packing: Ecp5Packing,
    /// Legal placement and routed logical nets.
    pub implementation: PnrResult,
    /// Post-route PIP-delay timing analysis.
    pub timing: TimingReport,
}

/// Runs the complete physical flow for one directly imported Struo design.
///
/// The mapped object remains immutable for Celox. Its Texo design is cloned,
/// then LUT/FF, DP16KD, DCCA, and optional LPF packing run before placement and
/// routing. `PostMapSimulation` evidence is mandatory. Mapped-netlist and
/// physical-implementation evidence are committed only after every stage
/// succeeds.
///
/// # Errors
///
/// Returns an error for missing simulation evidence or package selection, LPF
/// resolution, target packing, placement, or routing. The input import and
/// caller's evidence remain unchanged on every failure.
pub fn implement_struo_ecp5(
    imported: &ImportedEcp5Design,
    architecture: &Ecp5Architecture,
    options: Ecp5FlowOptions<'_>,
    evidence: &mut Evidence,
) -> Result<Ecp5FlowResult, Ecp5FlowError> {
    if !evidence.contains(Gate::PostMapSimulation) {
        return Err(Ecp5FlowError::MissingPostMapSimulation);
    }

    let mut design = imported.design().clone();
    let mut packing = pack_lut_ffs(&design, architecture)?;
    let block_rams = imported
        .metadata()
        .iter()
        .filter_map(|(&cell, metadata)| match metadata {
            PrimitiveMetadata::BlockRam {
                depth,
                word_width,
                physical_width,
                ..
            } => Some(BlockRamRequirement {
                cell,
                depth: *depth,
                word_width: *word_width,
                physical_width: *physical_width,
            }),
            _ => None,
        })
        .collect::<Vec<_>>();
    packing.pack_block_rams(&design, architecture, block_rams)?;

    let global_clocks = find_global_clock_requirements(&design, options.global_clock_fanout);
    packing.promote_global_clocks(&mut design, architecture, global_clocks)?;

    if let Some(lpf) = options.lpf {
        let package = options.package.ok_or(Ecp5FlowError::MissingPackageForLpf)?;
        let resolved = resolve_lpf_port_cells(
            lpf,
            imported
                .ports()
                .iter()
                .map(|port| (port.name.as_str(), port.bits.as_slice())),
            options.allow_unconstrained_io,
        )?;
        packing.apply_resolved_lpf(&design, architecture, package, &resolved)?;
    }

    let mut staged_evidence = evidence.clone();
    staged_evidence.record(Gate::MappedNetlistComplete);
    let implementation = implement_with_constraints(
        &design,
        architecture.device(),
        packing.constraints(),
        &mut staged_evidence,
    )?;
    let pip_delays_ps = architecture
        .pip_metadata()
        .iter()
        .map(|(&pip, metadata)| {
            u64::try_from(metadata.delay)
                .map(|delay| (pip, delay))
                .map_err(|_| Ecp5FlowError::NegativePipDelay {
                    pip,
                    delay: metadata.delay,
                })
        })
        .collect::<Result<BTreeMap<PipId, u64>, _>>()?;
    let timing_constraints = ecp5_timing_constraints(&design, &packing)?;
    let timing = analyze_timing(
        &design,
        architecture.device(),
        &implementation,
        &pip_delays_ps,
        &timing_constraints,
    )?;
    if timing.met_timing() {
        staged_evidence.record(Gate::TimingClosure);
    }
    *evidence = staged_evidence;

    Ok(Ecp5FlowResult {
        design,
        primitive_metadata: imported.metadata().clone(),
        absorbed_inputs: imported.absorbed_inputs().clone(),
        packing,
        implementation,
        timing,
    })
}

fn ecp5_timing_constraints(
    design: &Design,
    packing: &Ecp5Packing,
) -> Result<TimingConstraints, Ecp5FlowError> {
    let mut constraints = TimingConstraints::new();
    for (&cell_id, &frequency_hz) in packing.clock_frequencies_hz() {
        let cell = &design.cells()[cell_id.0];
        let driven_nets = cell
            .pins()
            .iter()
            .filter_map(|&pin_id| {
                let pin = &design.pins()[pin_id.0];
                (pin.direction != PinDirection::Input)
                    .then(|| pin.net())
                    .flatten()
            })
            .collect::<BTreeSet<_>>();
        if driven_nets.len() != 1 {
            return Err(Ecp5FlowError::ClockIoNet {
                cell: cell.name.clone(),
            });
        }
        let period_ps = PICOSECONDS_PER_SECOND
            .checked_div(frequency_hz)
            .filter(|period| *period != 0)
            .ok_or_else(|| Ecp5FlowError::ClockFrequencyOutOfRange {
                cell: cell.name.clone(),
                frequency_hz,
            })?;
        let source_net = *driven_nets.first().expect("set length was checked");
        insert_clock_period(&mut constraints, source_net, period_ps)?;
    }
    for clock in packing.global_clocks() {
        if let Some(&period_ps) = constraints.clock_periods_ps().get(&clock.source_net) {
            insert_clock_period(&mut constraints, clock.global_net, period_ps)?;
        }
    }
    Ok(constraints)
}

fn insert_clock_period(
    constraints: &mut TimingConstraints,
    net: NetId,
    period_ps: u64,
) -> Result<(), Ecp5FlowError> {
    if let Some(&previous) = constraints.clock_periods_ps().get(&net)
        && previous != period_ps
    {
        return Err(Ecp5FlowError::ConflictingClockPeriods { net });
    }
    constraints.set_clock_period_ps(net, period_ps);
    Ok(())
}

/// Complete ECP5 flow orchestration failed.
#[derive(Debug)]
pub enum Ecp5FlowError {
    /// Celox post-map simulation has not been recorded as passing.
    MissingPostMapSimulation,
    /// LPF constraints were supplied without an exact package name.
    MissingPackageForLpf,
    /// LPF name resolution failed.
    Lpf(LpfError),
    /// ECP5 packing failed.
    Packing(PackingError),
    /// Placement or routing failed.
    Pnr(PnrError),
    /// Project Trellis supplied a negative PIP delay.
    NegativePipDelay {
        /// Physical PIP.
        pip: PipId,
        /// Invalid delay value.
        delay: i32,
    },
    /// A frequency-constrained IO cell does not drive exactly one net.
    ClockIoNet {
        /// Logical IO cell name.
        cell: String,
    },
    /// A clock is too fast to represent with a non-zero picosecond period.
    ClockFrequencyOutOfRange {
        /// Logical IO cell name.
        cell: String,
        /// Requested frequency.
        frequency_hz: u64,
    },
    /// More than one source assigned different periods to one logical net.
    ConflictingClockPeriods {
        /// Logical clock net.
        net: NetId,
    },
    /// Static timing analysis failed.
    Timing(TimingError),
}

impl fmt::Display for Ecp5FlowError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingPostMapSimulation => {
                write!(f, "Celox post-map simulation evidence is required")
            }
            Self::MissingPackageForLpf => {
                write!(f, "an exact ECP5 package is required when LPF is supplied")
            }
            Self::Lpf(error) => write!(f, "LPF resolution failed: {error}"),
            Self::Packing(error) => write!(f, "ECP5 packing failed: {error}"),
            Self::Pnr(error) => write!(f, "ECP5 physical implementation failed: {error}"),
            Self::NegativePipDelay { pip, delay } => {
                write!(f, "ECP5 PIP {} has negative delay {delay}", pip.0)
            }
            Self::ClockIoNet { cell } => write!(
                f,
                "frequency-constrained IO cell `{cell}` must drive exactly one net"
            ),
            Self::ClockFrequencyOutOfRange { cell, frequency_hz } => write!(
                f,
                "clock IO cell `{cell}` frequency {frequency_hz} Hz is outside picosecond resolution"
            ),
            Self::ConflictingClockPeriods { net } => {
                write!(f, "clock net {} has conflicting periods", net.0)
            }
            Self::Timing(error) => write!(f, "ECP5 static timing analysis failed: {error}"),
        }
    }
}

impl Error for Ecp5FlowError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Lpf(error) => Some(error),
            Self::Packing(error) => Some(error),
            Self::Pnr(error) => Some(error),
            Self::Timing(error) => Some(error),
            Self::MissingPostMapSimulation
            | Self::MissingPackageForLpf
            | Self::NegativePipDelay { .. }
            | Self::ClockIoNet { .. }
            | Self::ClockFrequencyOutOfRange { .. }
            | Self::ConflictingClockPeriods { .. } => None,
        }
    }
}

impl From<LpfError> for Ecp5FlowError {
    fn from(value: LpfError) -> Self {
        Self::Lpf(value)
    }
}

impl From<PackingError> for Ecp5FlowError {
    fn from(value: PackingError) -> Self {
        Self::Packing(value)
    }
}

impl From<PnrError> for Ecp5FlowError {
    fn from(value: PnrError) -> Self {
        Self::Pnr(value)
    }
}

impl From<TimingError> for Ecp5FlowError {
    fn from(value: TimingError) -> Self {
        Self::Timing(value)
    }
}

/// Missing bitstream release evidence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MissingEvidence {
    missing: Vec<Gate>,
}

impl MissingEvidence {
    /// Missing gates in stable pipeline order.
    #[must_use]
    pub fn gates(&self) -> &[Gate] {
        &self.missing
    }
}

impl fmt::Display for MissingEvidence {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} verification gate(s) are missing", self.missing.len())
    }
}

impl Error for MissingEvidence {}

#[cfg(test)]
mod tests {
    use struo_celox::ecp5_simulator;
    use struo_ir::{ClockEdge, Netlist, RegisterCell};
    use struo_target_ecp5::{Ecp5Netlist, map_to_ecp5};
    use texo_model::{BelId, Design, Device, PinDirection, ResourceKind};
    use texo_pnr::PlacementConstraints;
    use texo_struo::import_ecp5;
    use texo_target_ecp5::{
        find_global_clock_requirements, pack_lut_ffs, parse_lpf, read_architecture,
        resolve_lpf_port_cells,
    };

    use super::{
        Ecp5FlowError, Ecp5FlowOptions, Evidence, Gate, ecp5_timing_constraints, implement,
        implement_struo_ecp5, implement_with_constraints, verify_post_map_with_celox,
    };

    const ECP5_FIXTURE: &str = include_str!("../../texo-target-ecp5/fixtures/minimal-ecp5.json");

    #[test]
    fn implementation_records_only_its_own_gate() {
        let mut design = Design::new();
        let a = design.add_cell("a", ResourceKind::Logic);
        let a_out = design.add_pin(a, "out", PinDirection::Output).unwrap();
        let b = design.add_cell("b", ResourceKind::Logic);
        let b_in = design.add_pin(b, "in", PinDirection::Input).unwrap();
        design.add_net("n", a_out, [b_in]).unwrap();
        let device = Device::rectangular_logic(4, 4).unwrap();
        let mut evidence = Evidence::new();

        implement(&design, &device, &mut evidence).unwrap();

        assert!(evidence.contains(Gate::PhysicalImplementation));
        assert!(!evidence.contains(Gate::TimingClosure));
        assert!(evidence.authorize_bitstream().is_err());
    }

    #[test]
    fn rejected_packing_constraints_do_not_record_evidence() {
        let mut design = Design::new();
        let a = design.add_cell("a", ResourceKind::Logic);
        design.add_pin(a, "out", PinDirection::Output).unwrap();
        let b = design.add_cell("b", ResourceKind::Logic);
        design.add_pin(b, "in", PinDirection::Input).unwrap();
        let device = Device::rectangular_logic(2, 1).unwrap();
        let mut constraints = PlacementConstraints::new();
        constraints.add_group([a, b], [vec![BelId(0), BelId(0)]]);
        let mut evidence = Evidence::new();

        assert!(implement_with_constraints(&design, &device, &constraints, &mut evidence).is_err());
        assert!(!evidence.contains(Gate::PhysicalImplementation));
    }

    #[test]
    fn runs_celox_lpf_packing_placement_and_routing_as_one_flow() {
        let mapped = mapped_xor();
        let imported = import_ecp5(&mapped).unwrap();
        let architecture = read_architecture(ECP5_FIXTURE.as_bytes()).unwrap();
        let lpf = parse_lpf(
            br"
                LOCATE COMP lhs SITE A10;
                LOCATE COMP rhs SITE B10;
                LOCATE COMP value SITE C10;
                IOBUF PORT value IO_TYPE=LVCMOS33;
            "
            .as_slice(),
        )
        .unwrap();
        let mut evidence = Evidence::new();
        verify_post_map_with_celox(
            &mut evidence,
            || -> Result<(), Box<dyn std::error::Error>> {
                let mut simulator = ecp5_simulator(&mapped)?.build_native()?;
                let lhs = simulator.signal("lhs");
                let rhs = simulator.signal("rhs");
                let value = simulator.signal("value");
                simulator.modify(|io| {
                    io.set(lhs, 1_u8);
                    io.set(rhs, 0_u8);
                })?;
                if simulator.get(value) != 1_u8.into() {
                    return Err("mapped XOR returned the wrong value".into());
                }
                Ok(())
            },
        )
        .unwrap();

        let result = implement_struo_ecp5(
            &imported,
            &architecture,
            Ecp5FlowOptions {
                package: Some("CABGA381"),
                lpf: Some(&lpf),
                ..Ecp5FlowOptions::default()
            },
            &mut evidence,
        )
        .unwrap();

        assert!(evidence.contains(Gate::PostMapSimulation));
        assert!(evidence.contains(Gate::MappedNetlistComplete));
        assert!(evidence.contains(Gate::PhysicalImplementation));
        assert!(!evidence.contains(Gate::TimingClosure));
        assert_eq!(result.design.cells().len(), 4);
        assert_eq!(result.primitive_metadata.len(), 4);
        assert!(!result.absorbed_inputs.is_empty());
        assert_eq!(result.implementation.routes.len(), 3);
        assert_eq!(result.packing.io_attributes().len(), 1);
        assert_eq!(result.packing.constraints().groups().len(), 3);
    }

    #[test]
    fn requires_celox_evidence_before_the_ecp5_flow() {
        let mapped = mapped_xor();
        let imported = import_ecp5(&mapped).unwrap();
        let architecture = read_architecture(ECP5_FIXTURE.as_bytes()).unwrap();
        let mut evidence = Evidence::new();

        assert!(matches!(
            implement_struo_ecp5(
                &imported,
                &architecture,
                Ecp5FlowOptions::default(),
                &mut evidence,
            ),
            Err(Ecp5FlowError::MissingPostMapSimulation)
        ));
        assert_eq!(evidence, Evidence::new());
    }

    #[test]
    fn propagates_an_lpf_clock_period_through_a_global_buffer() {
        let mut source = Netlist::new("registered");
        let data = source.add_input("data");
        let clock = source.add_input("clock");
        let state = source.add_register_output("state");
        source.add_register(RegisterCell::new(
            "state",
            state,
            data,
            clock,
            ClockEdge::Rising,
            None,
            None,
        ));
        let mapped = map_to_ecp5(&source).unwrap();
        let imported = import_ecp5(&mapped).unwrap();
        let architecture = read_architecture(ECP5_FIXTURE.as_bytes()).unwrap();
        let lpf = parse_lpf(
            br"
                FREQUENCY PORT clock 25 MHZ;
            "
            .as_slice(),
        )
        .unwrap();
        let mut design = imported.design().clone();
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();
        let global_clocks = find_global_clock_requirements(&design, 1);
        packing
            .promote_global_clocks(&mut design, &architecture, global_clocks)
            .unwrap();
        let resolved = resolve_lpf_port_cells(
            &lpf,
            imported
                .ports()
                .iter()
                .map(|port| (port.name.as_str(), port.bits.as_slice())),
            true,
        )
        .unwrap();
        packing
            .apply_resolved_lpf(&design, &architecture, "CABGA381", &resolved)
            .unwrap();

        let constraints = ecp5_timing_constraints(&design, &packing).unwrap();
        let global_net = packing.global_clocks()[0].global_net;

        assert_eq!(packing.clock_frequencies_hz().len(), 1);
        assert_eq!(constraints.clock_periods_ps().len(), 2);
        assert_eq!(constraints.clock_periods_ps()[&global_net], 40_000);
    }

    #[test]
    fn failed_ecp5_flow_commits_no_new_evidence() {
        let mapped = mapped_xor();
        let imported = import_ecp5(&mapped).unwrap();
        let architecture = read_architecture(ECP5_FIXTURE.as_bytes()).unwrap();
        let lpf = parse_lpf(b"LOCATE COMP missing SITE A10;".as_slice()).unwrap();
        let mut evidence = Evidence::new();
        evidence.record(Gate::PostMapSimulation);
        let original = evidence.clone();

        assert!(matches!(
            implement_struo_ecp5(
                &imported,
                &architecture,
                Ecp5FlowOptions {
                    package: Some("CABGA381"),
                    lpf: Some(&lpf),
                    allow_unconstrained_io: true,
                    ..Ecp5FlowOptions::default()
                },
                &mut evidence,
            ),
            Err(Ecp5FlowError::Lpf(_))
        ));
        assert_eq!(evidence, original);
        assert!(!evidence.contains(Gate::MappedNetlistComplete));
        assert!(!evidence.contains(Gate::PhysicalImplementation));
    }

    #[test]
    fn failed_celox_testbench_does_not_record_its_gate() {
        let mut evidence = Evidence::new();
        let result = verify_post_map_with_celox(&mut evidence, || Err("mismatch"));

        assert_eq!(result, Err("mismatch"));
        assert!(!evidence.contains(Gate::PostMapSimulation));
    }

    fn mapped_xor() -> Ecp5Netlist {
        let mut source = Netlist::new("logic");
        let lhs = source.add_input("lhs");
        let rhs = source.add_input("rhs");
        let value = source.add_xor(lhs, rhs);
        source.add_output("value", value);
        map_to_ecp5(&source).unwrap()
    }
}
