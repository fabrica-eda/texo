//! Flow orchestration and explicit verification evidence.

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;

use texo_model::{CellId, CellPinId, Design, Device, NetId, PinDirection, PipId, ResourceKind};
use texo_pnr::{PlacementConstraints, PnrError, PnrResult, place_and_route_with_constraints};
use texo_struo::{ImportedEcp5Design, PrimitiveMetadata};
use texo_target_ecp5::{
    BlockRamRequirement, DEFAULT_GLOBAL_CLOCK_FANOUT, DelayRangeRecord, Ecp5Architecture,
    Ecp5Packing, LpfConstraints, LpfError, PackingError, SpeedGradeRecord,
    find_global_clock_requirements, pack_lut_ffs, resolve_lpf_port_cells,
};
use texo_timing::{
    DelayRange, PICOSECONDS_PER_SECOND, TimingConstraints, TimingError, TimingModel, TimingReport,
    analyze_timing,
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
    /// Exact ECP5 speed grade used for all timing arcs.
    pub speed_grade: Option<&'a str>,
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
            speed_grade: None,
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
    /// Exact speed-grade timing table used by STA.
    pub speed_grade: String,
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
/// Returns an error for missing simulation evidence, speed grade, or package
/// selection, LPF resolution, target packing, placement, routing, or timing.
/// The input import and caller's evidence remain unchanged on every failure.
pub fn implement_struo_ecp5(
    imported: &ImportedEcp5Design,
    architecture: &Ecp5Architecture,
    options: Ecp5FlowOptions<'_>,
    evidence: &mut Evidence,
) -> Result<Ecp5FlowResult, Ecp5FlowError> {
    if !evidence.contains(Gate::PostMapSimulation) {
        return Err(Ecp5FlowError::MissingPostMapSimulation);
    }
    let speed_grade_name = options
        .speed_grade
        .ok_or(Ecp5FlowError::MissingSpeedGrade)?;
    let speed_grade = architecture
        .speed_grades()
        .get(speed_grade_name)
        .ok_or_else(|| Ecp5FlowError::UnknownSpeedGrade(speed_grade_name.into()))?;

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
    let pip_delays = ecp5_pip_delays(architecture, speed_grade, &implementation)?;
    let timing_model = ecp5_timing_model(&design, &packing, speed_grade)?;
    let timing_constraints = ecp5_timing_constraints(&design, &packing)?;
    let timing = analyze_timing(
        &design,
        architecture.device(),
        &implementation,
        &pip_delays,
        &timing_model,
        &timing_constraints,
    )?;
    if timing.met_timing() {
        staged_evidence.record(Gate::TimingClosure);
    }
    *evidence = staged_evidence;

    Ok(Ecp5FlowResult {
        speed_grade: speed_grade_name.into(),
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

fn ecp5_pip_delays(
    architecture: &Ecp5Architecture,
    speed_grade: &SpeedGradeRecord,
    implementation: &PnrResult,
) -> Result<BTreeMap<PipId, DelayRange>, Ecp5FlowError> {
    let device = architecture.device();
    let selected = implementation
        .routes
        .iter()
        .flat_map(|route| route.pips.iter().copied())
        .collect::<BTreeSet<_>>();
    let mut source_fanout = BTreeMap::new();
    for &pip_id in &selected {
        let pip = &device.pips()[pip_id.0];
        *source_fanout.entry(pip.from).or_insert(0_u64) += 1;
    }
    selected
        .into_iter()
        .map(|pip_id| {
            let metadata = &architecture.pip_metadata()[&pip_id];
            let class = speed_grade
                .pip_classes
                .get(&metadata.timing_class)
                .ok_or_else(|| Ecp5FlowError::MissingPipTimingClass {
                    speed_grade: speed_grade.name.clone(),
                    timing_class: metadata.timing_class.clone(),
                })?;
            let fanout = source_fanout[&device.pips()[pip_id.0].from];
            let min_ps = class
                .min_fanout_adder_ps
                .checked_mul(fanout)
                .and_then(|delay| class.min_base_ps.checked_add(delay))
                .ok_or(Ecp5FlowError::TimingDelayOverflow)?;
            let max_ps = class
                .max_fanout_adder_ps
                .checked_mul(fanout)
                .and_then(|delay| class.max_base_ps.checked_add(delay))
                .ok_or(Ecp5FlowError::TimingDelayOverflow)?;
            Ok((pip_id, DelayRange::new(min_ps, max_ps)?))
        })
        .collect()
}

fn ecp5_timing_model(
    design: &Design,
    packing: &Ecp5Packing,
    speed_grade: &SpeedGradeRecord,
) -> Result<TimingModel, Ecp5FlowError> {
    let records = speed_grade
        .cells
        .iter()
        .map(|record| (record.cell_type.as_str(), record))
        .collect::<BTreeMap<_, _>>();
    let general_routing_ffs = packing
        .general_routing_ffs()
        .iter()
        .copied()
        .collect::<BTreeSet<_>>();
    let mut model = TimingModel::new();
    for (index, cell) in design.cells().iter().enumerate() {
        let cell_id = CellId(index);
        let cell_type = match cell.kind {
            ResourceKind::Lut(4) => "TRELLIS_COMB",
            ResourceKind::Register => "TRELLIS_FF",
            ResourceKind::Clock => "DCCA",
            _ => continue,
        };
        let record =
            records
                .get(cell_type)
                .copied()
                .ok_or_else(|| Ecp5FlowError::MissingCellTiming {
                    speed_grade: speed_grade.name.clone(),
                    cell_type: cell_type.into(),
                })?;
        for arc in &record.arcs {
            let Some(from) = find_cell_pin(design, cell_id, &arc.from_pin) else {
                continue;
            };
            let Some(to) = find_cell_pin(design, cell_id, &arc.to_pin) else {
                continue;
            };
            let delay = timing_delay(arc.delay)?;
            if cell.kind == ResourceKind::Register && arc.from_pin == "CLK" && arc.to_pin == "Q" {
                model.add_clock_to_q(from, to, delay)?;
            } else {
                model.add_cell_arc(from, to, delay)?;
            }
        }
        for check in &record.setup_holds {
            let using_general_routing = general_routing_ffs.contains(&cell_id);
            if (check.signal_pin == "DI" && using_general_routing)
                || (check.signal_pin == "M" && !using_general_routing)
            {
                continue;
            }
            let logical_signal = if check.signal_pin == "M" {
                "DI"
            } else {
                &check.signal_pin
            };
            let Some(signal) = find_cell_pin(design, cell_id, logical_signal) else {
                continue;
            };
            let Some(clock) = find_cell_pin(design, cell_id, &check.clock_pin) else {
                continue;
            };
            model.add_setup_hold(
                clock,
                signal,
                timing_delay(check.setup)?,
                timing_delay(check.hold)?,
            )?;
        }
    }
    Ok(model)
}

fn timing_delay(record: DelayRangeRecord) -> Result<DelayRange, TimingError> {
    DelayRange::new(record.min_ps, record.max_ps)
}

fn find_cell_pin(design: &Design, cell: CellId, name: &str) -> Option<CellPinId> {
    design.cells()[cell.0]
        .pins()
        .iter()
        .copied()
        .find(|pin| design.pins()[pin.0].name == name)
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
    /// No exact ECP5 speed grade was selected.
    MissingSpeedGrade,
    /// Selected speed grade is absent from the architecture snapshot.
    UnknownSpeedGrade(String),
    /// LPF name resolution failed.
    Lpf(LpfError),
    /// ECP5 packing failed.
    Packing(PackingError),
    /// Placement or routing failed.
    Pnr(PnrError),
    /// One selected PIP class was absent from the speed-grade table.
    MissingPipTimingClass {
        /// Speed-grade name.
        speed_grade: String,
        /// Missing timing class.
        timing_class: String,
    },
    /// A required split-cell timing record was absent.
    MissingCellTiming {
        /// Speed-grade name.
        speed_grade: String,
        /// Missing cell type.
        cell_type: String,
    },
    /// Speed-grade delay arithmetic overflowed.
    TimingDelayOverflow,
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
            Self::MissingSpeedGrade => write!(f, "an exact ECP5 speed grade is required"),
            Self::UnknownSpeedGrade(speed_grade) => {
                write!(f, "architecture has no ECP5 speed grade `{speed_grade}`")
            }
            Self::Lpf(error) => write!(f, "LPF resolution failed: {error}"),
            Self::Packing(error) => write!(f, "ECP5 packing failed: {error}"),
            Self::Pnr(error) => write!(f, "ECP5 physical implementation failed: {error}"),
            Self::MissingPipTimingClass {
                speed_grade,
                timing_class,
            } => write!(
                f,
                "ECP5 speed grade `{speed_grade}` has no PIP timing class `{timing_class}`"
            ),
            Self::MissingCellTiming {
                speed_grade,
                cell_type,
            } => write!(
                f,
                "ECP5 speed grade `{speed_grade}` has no cell timing for `{cell_type}`"
            ),
            Self::TimingDelayOverflow => write!(f, "ECP5 timing delay arithmetic overflowed"),
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
            | Self::MissingSpeedGrade
            | Self::UnknownSpeedGrade(_)
            | Self::MissingPipTimingClass { .. }
            | Self::MissingCellTiming { .. }
            | Self::TimingDelayOverflow
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
    use texo_model::{BelId, CellId, Design, Device, PinDirection, ResourceKind};
    use texo_pnr::PlacementConstraints;
    use texo_struo::import_ecp5;
    use texo_target_ecp5::{
        find_global_clock_requirements, pack_lut_ffs, parse_lpf, read_architecture,
        resolve_lpf_port_cells,
    };

    use super::{
        Ecp5FlowError, Ecp5FlowOptions, Evidence, Gate, ecp5_timing_constraints, ecp5_timing_model,
        find_cell_pin, implement, implement_struo_ecp5, implement_with_constraints,
        verify_post_map_with_celox,
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
                speed_grade: Some("6"),
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
        assert_eq!(result.speed_grade, "6");
        assert!(
            result
                .timing
                .net_delays
                .iter()
                .all(|delay| delay.delay.min_ps <= delay.delay.max_ps)
        );
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
    fn requires_an_exact_ecp5_speed_grade() {
        let mapped = mapped_xor();
        let imported = import_ecp5(&mapped).unwrap();
        let architecture = read_architecture(ECP5_FIXTURE.as_bytes()).unwrap();
        let mut evidence = Evidence::new();
        evidence.record(Gate::PostMapSimulation);

        assert!(matches!(
            implement_struo_ecp5(
                &imported,
                &architecture,
                Ecp5FlowOptions::default(),
                &mut evidence,
            ),
            Err(Ecp5FlowError::MissingSpeedGrade)
        ));
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
        let timing_model =
            ecp5_timing_model(&design, &packing, &architecture.speed_grades()["6"]).unwrap();
        let global_net = packing.global_clocks()[0].global_net;
        let ff = design
            .cells()
            .iter()
            .position(|cell| cell.kind == ResourceKind::Register)
            .map(CellId)
            .unwrap();
        let ff_data = find_cell_pin(&design, ff, "DI").unwrap();
        let ff_q = find_cell_pin(&design, ff, "Q").unwrap();

        assert_eq!(packing.clock_frequencies_hz().len(), 1);
        assert_eq!(constraints.clock_periods_ps().len(), 2);
        assert_eq!(constraints.clock_periods_ps()[&global_net], 40_000);
        assert_eq!(timing_model.clock_to_q(ff_q).unwrap().1.max_ps, 525);
        assert_eq!(timing_model.setup_hold(ff_data).unwrap().2.min_ps, 233);
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
                    speed_grade: Some("6"),
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
