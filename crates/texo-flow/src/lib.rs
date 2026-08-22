//! Flow orchestration and explicit verification evidence.

use std::cmp::Reverse;
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;

use texo_model::{CellId, CellPinId, Design, Device, NetId, PinDirection, PipId, ResourceKind};
pub use texo_pnr::RoutingProgress;
use texo_pnr::{
    NetRoute, Placement, PlacementConstraints, PnrError, PnrResult, RoutingConstraints,
    RoutingCosts, place_and_route_with_constraints, place_with_constraints,
    place_with_net_sink_weights, refine_placement_with_net_sink_weights_limited,
    route_with_placement_and_progress, route_with_timing_costs_and_progress,
};
use texo_struo::{ImportedEcp5Design, PrimitiveMetadata};
use texo_target_ecp5::{
    BlockRamRequirement, DEFAULT_GLOBAL_CLOCK_FANOUT, DelayRangeRecord, Ecp5Architecture,
    Ecp5GlobalRoutingCache, Ecp5Packing, LpfConstraints, LpfError, PackingError,
    PipClassTimingRecord, SpeedGradeRecord, find_global_clock_requirements, pack_lut_ffs_excluding,
    resolve_lpf_port_cells,
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

/// Completed milestones emitted by the native ECP5 implementation flow.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Ecp5FlowStage {
    /// Target packing and external constraints are complete.
    Packed,
    /// Deterministic cost-based placement is complete.
    Placed,
    /// Dedicated primary-clock trees are locked.
    GlobalClocksRouted,
    /// Progress within deterministic negotiated routing.
    Routing(RoutingProgress),
    /// Negotiated routing is complete.
    Routed,
    /// STA-weighted deterministic replacement is complete.
    TimingDrivenPlaced,
    /// Dedicated primary-clock trees for the timing-driven placement are locked.
    TimingDrivenGlobalClocksRouted,
    /// Progress within timing-driven negotiated routing.
    TimingDrivenRouting(RoutingProgress),
    /// Timing-driven negotiated routing is complete.
    TimingDrivenRouted,
    /// Post-route static timing analysis is complete.
    Timed,
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
    implement_struo_ecp5_with_progress(imported, architecture, options, evidence, |_| {})
}

/// Runs the complete Struo-to-ECP5 flow and reports completed phase boundaries.
///
/// # Errors
///
/// Returns the same errors as [`implement_struo_ecp5`].
pub fn implement_struo_ecp5_with_progress(
    imported: &ImportedEcp5Design,
    architecture: &Ecp5Architecture,
    options: Ecp5FlowOptions<'_>,
    evidence: &mut Evidence,
    mut progress: impl FnMut(Ecp5FlowStage),
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
    let constant_luts = imported.metadata().iter().filter_map(|(&cell, metadata)| {
        matches!(metadata, PrimitiveMetadata::Constant { .. }).then_some(cell)
    });
    let mut packing = pack_lut_ffs_excluding(&design, architecture, constant_luts)?;
    packing.pack_carry_pairs(
        &design,
        architecture,
        imported.carry_pairs().iter().copied(),
    )?;
    packing.pack_block_rams(&design, architecture, block_ram_requirements(imported))?;

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
    progress(Ecp5FlowStage::Packed);

    let mut staged_evidence = evidence.clone();
    staged_evidence.record(Gate::MappedNetlistComplete);
    let placement = place_with_constraints(&design, architecture.device(), packing.constraints())?;
    progress(Ecp5FlowStage::Placed);
    let mut global_routing_cache = architecture.global_routing_cache();
    let routing = packing.global_routing_constraints_cached(
        &design,
        architecture,
        &placement,
        &mut global_routing_cache,
    )?;
    progress(Ecp5FlowStage::GlobalClocksRouted);
    let initial_implementation = route_with_placement_and_progress(
        &design,
        architecture.device(),
        placement,
        &routing,
        |event| progress(Ecp5FlowStage::Routing(event)),
    )?;
    progress(Ecp5FlowStage::Routed);
    let timing_model = ecp5_timing_model(&design, &packing, speed_grade)?;
    let timing_constraints = ecp5_timing_constraints(&design, &packing)?;
    let initial_timing = analyze_ecp5_implementation(
        &design,
        architecture,
        speed_grade,
        &initial_implementation,
        &timing_model,
        &timing_constraints,
    )?;

    let (implementation, timing) = TimingDrivenContext {
        design: &design,
        architecture,
        packing: &packing,
        global_routing_cache: &mut global_routing_cache,
        speed_grade,
        timing_model: &timing_model,
        timing_constraints: &timing_constraints,
    }
    .optimize(initial_implementation, initial_timing, &mut progress)?;
    progress(Ecp5FlowStage::Timed);
    staged_evidence.record(Gate::PhysicalImplementation);
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

fn block_ram_requirements(imported: &ImportedEcp5Design) -> Vec<BlockRamRequirement> {
    imported
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
        .collect()
}

struct TimingDrivenContext<'a> {
    design: &'a Design,
    architecture: &'a Ecp5Architecture,
    packing: &'a Ecp5Packing,
    global_routing_cache: &'a mut Ecp5GlobalRoutingCache<'a>,
    speed_grade: &'a SpeedGradeRecord,
    timing_model: &'a TimingModel,
    timing_constraints: &'a TimingConstraints,
}

impl TimingDrivenContext<'_> {
    fn optimize(
        &mut self,
        initial_implementation: PnrResult,
        initial_timing: TimingReport,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<(PnrResult, TimingReport), Ecp5FlowError> {
        if initial_timing.met_timing() {
            return Ok((initial_implementation, initial_timing));
        }
        let placement_weights = timing_placement_weights(&initial_timing, self.timing_constraints);
        let placement = place_with_net_sink_weights(
            self.design,
            self.architecture.device(),
            self.packing.constraints(),
            &placement_weights,
        )?;
        progress(Ecp5FlowStage::TimingDrivenPlaced);
        let routing = self.packing.global_routing_constraints_cached(
            self.design,
            self.architecture,
            &placement,
            self.global_routing_cache,
        )?;
        progress(Ecp5FlowStage::TimingDrivenGlobalClocksRouted);
        let (implementation, timing) =
            self.route_and_analyze(placement, &routing, None, progress)?;
        let routing_weights = timing_net_weights(&timing, self.timing_constraints);
        let mut routing_costs =
            ecp5_routing_costs(self.architecture, self.speed_grade, routing_weights)?;
        routing_costs.set_sink_min_delays_ps(hold_sink_min_delays(&timing));
        let (timing_implementation, timing_routed) = self.route_and_analyze(
            implementation.placement.clone(),
            &routing,
            Some(&routing_costs),
            progress,
        )?;
        let candidates = vec![
            (initial_implementation, initial_timing),
            (implementation, timing),
            (timing_implementation, timing_routed),
        ];
        let mut archive = select_timing_frontier(candidates);
        let mut active = archive.clone();
        for _ in 0..MAX_INCREMENTAL_REFINEMENTS {
            if archive.iter().any(|(_, timing)| timing.met_timing()) {
                break;
            }
            let mut children = Vec::with_capacity(active.len());
            for (implementation, timing) in &active {
                children.extend(self.refine_candidates(
                    implementation,
                    timing,
                    &mut routing_costs,
                    progress,
                )?);
            }
            let setup_focused = archive
                .iter()
                .any(|(_, timing)| timing.worst_hold_slack_ps.is_some_and(|slack| slack >= 0))
                || children
                    .iter()
                    .any(|(_, timing)| timing.worst_hold_slack_ps.is_some_and(|slack| slack >= 0));
            active = select_timing_beam(children, setup_focused);
            let mut expanded_archive = archive;
            expanded_archive.extend(active.iter().cloned());
            archive = select_timing_frontier(expanded_archive);
        }
        let mut hold_repairs = Vec::new();
        for (implementation, timing) in &archive {
            if let Some(repaired) =
                self.repair_hold_locally(implementation, timing, &mut routing_costs, progress)?
            {
                hold_repairs.push(repaired);
            }
        }
        archive.extend(hold_repairs);
        archive = select_timing_frontier(archive);
        Ok(archive
            .into_iter()
            .max_by_key(|(_, timing)| timing_score(timing))
            .expect("the timing archive is non-empty"))
    }

    fn repair_hold_locally(
        &self,
        implementation: &PnrResult,
        timing: &TimingReport,
        routing_costs: &mut RoutingCosts,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<Option<TimingCandidate>, Ecp5FlowError> {
        let minimums = hold_sink_min_delays(timing);
        if minimums.is_empty() {
            return Ok(None);
        }
        let repair_nets = minimums
            .keys()
            .map(|(net, _)| *net)
            .collect::<BTreeSet<_>>();
        let frozen = freeze_routes_except(&implementation.routes, &repair_nets);
        routing_costs.set_net_criticalities(timing_net_weights(timing, self.timing_constraints));
        routing_costs.set_sink_min_delays_ps(minimums);
        match self.route_and_analyze(
            implementation.placement.clone(),
            &frozen,
            Some(routing_costs),
            progress,
        ) {
            Ok(repaired) => Ok(Some(repaired)),
            Err(Ecp5FlowError::Pnr(_)) => Ok(None),
            Err(error) => Err(error),
        }
    }

    fn refine_candidates(
        &mut self,
        implementation: &PnrResult,
        timing: &TimingReport,
        routing_costs: &mut RoutingCosts,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<Vec<(PnrResult, TimingReport)>, Ecp5FlowError> {
        let refinement_weights = timing_placement_weights(timing, self.timing_constraints);
        let refined_placement = refine_placement_with_net_sink_weights_limited(
            self.design,
            self.architecture.device(),
            self.packing.constraints(),
            implementation.placement.clone(),
            &refinement_weights,
            MAX_REFINED_PLACEMENT_UNITS,
        )?;
        progress(Ecp5FlowStage::TimingDrivenPlaced);
        let refined_routing = self.packing.global_routing_constraints_cached(
            self.design,
            self.architecture,
            &refined_placement,
            self.global_routing_cache,
        )?;
        progress(Ecp5FlowStage::TimingDrivenGlobalClocksRouted);
        let criticalities = timing_net_weights(timing, self.timing_constraints);
        let mut ranked_critical_nets = criticalities
            .iter()
            .filter_map(|(&net, &weight)| (weight > 1).then_some((Reverse(weight), net)))
            .collect::<Vec<_>>();
        ranked_critical_nets.sort_unstable();
        let mut released = ranked_critical_nets
            .into_iter()
            .take(MAX_RELEASED_CRITICAL_NETS)
            .map(|(_, net)| net)
            .collect::<BTreeSet<_>>();
        released.extend(hold_sink_min_delays(timing).keys().map(|(net, _)| *net));
        let incremental_routing = freeze_unchanged_routes(
            self.design,
            implementation,
            &refined_placement,
            &refined_routing,
            &released,
        );
        routing_costs.set_net_criticalities(criticalities);
        routing_costs.set_sink_min_delays_ps(hold_sink_min_delays(timing));
        let refined = self.route_and_analyze(
            refined_placement,
            &incremental_routing,
            Some(routing_costs),
            progress,
        )?;
        Ok(vec![refined])
    }

    fn route_and_analyze(
        &self,
        placement: Placement,
        routing: &RoutingConstraints,
        costs: Option<&RoutingCosts>,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<(PnrResult, TimingReport), Ecp5FlowError> {
        let implementation = if let Some(costs) = costs {
            route_with_timing_costs_and_progress(
                self.design,
                self.architecture.device(),
                placement,
                routing,
                costs,
                |event| progress(Ecp5FlowStage::TimingDrivenRouting(event)),
            )?
        } else {
            route_with_placement_and_progress(
                self.design,
                self.architecture.device(),
                placement,
                routing,
                |event| progress(Ecp5FlowStage::TimingDrivenRouting(event)),
            )?
        };
        progress(Ecp5FlowStage::TimingDrivenRouted);
        let timing = analyze_ecp5_implementation(
            self.design,
            self.architecture,
            self.speed_grade,
            &implementation,
            self.timing_model,
            self.timing_constraints,
        )?;
        Ok((implementation, timing))
    }
}

const MAX_INCREMENTAL_REFINEMENTS: usize = 4;
const TIMING_FRONTIER_WIDTH: usize = 3;
const SETUP_FOCUSED_BEAM_WIDTH: usize = 2;
const MAX_REFINED_PLACEMENT_UNITS: usize = 32;
const MAX_RELEASED_CRITICAL_NETS: usize = 64;

fn ecp5_routing_costs(
    architecture: &Ecp5Architecture,
    speed_grade: &SpeedGradeRecord,
    net_criticalities: BTreeMap<NetId, u64>,
) -> Result<RoutingCosts, Ecp5FlowError> {
    let class_delays = speed_grade
        .pip_classes
        .iter()
        .map(|(name, class)| {
            let delay = pip_class_delay(class, 1)?;
            let minimum =
                u32::try_from(delay.min_ps).map_err(|_| Ecp5FlowError::TimingDelayOverflow)?;
            let maximum =
                u32::try_from(delay.max_ps).map_err(|_| Ecp5FlowError::TimingDelayOverflow)?;
            Ok((name.as_str(), (minimum, maximum)))
        })
        .collect::<Result<BTreeMap<_, _>, Ecp5FlowError>>()?;
    let mut pip_min_delays_ps = Vec::with_capacity(architecture.device().pips().len());
    let mut pip_delays_ps = Vec::with_capacity(architecture.device().pips().len());
    for (_, metadata) in architecture.pip_metadata_iter() {
        let &(minimum, maximum) = class_delays.get(metadata.timing_class).ok_or_else(|| {
            Ecp5FlowError::MissingPipTimingClass {
                speed_grade: speed_grade.name.clone(),
                timing_class: metadata.timing_class.to_owned(),
            }
        })?;
        pip_min_delays_ps.push(minimum);
        pip_delays_ps.push(maximum);
    }
    let mut costs = RoutingCosts::new(pip_delays_ps, net_criticalities);
    costs.set_pip_min_delays_ps(pip_min_delays_ps);
    Ok(costs)
}

fn analyze_ecp5_implementation(
    design: &Design,
    architecture: &Ecp5Architecture,
    speed_grade: &SpeedGradeRecord,
    implementation: &PnrResult,
    timing_model: &TimingModel,
    timing_constraints: &TimingConstraints,
) -> Result<TimingReport, Ecp5FlowError> {
    let pip_delays = ecp5_pip_delays(architecture, speed_grade, implementation)?;
    Ok(analyze_timing(
        design,
        architecture.device(),
        implementation,
        &pip_delays,
        timing_model,
        timing_constraints,
    )?)
}

fn timing_net_weights(
    timing: &TimingReport,
    constraints: &TimingConstraints,
) -> BTreeMap<NetId, u64> {
    let Some(period_ps) = constraints.clock_periods_ps().values().copied().min() else {
        return BTreeMap::new();
    };
    let Some(worst_slack_ps) = timing.worst_slack_ps else {
        return BTreeMap::new();
    };
    let period_ps = i128::from(period_ps.max(1));
    let critical_limit = worst_slack_ps + period_ps;
    let mut weights = BTreeMap::<NetId, u64>::new();
    for edge in &timing.net_setup_slacks {
        let urgency = (critical_limit - edge.slack_ps).clamp(0, period_ps);
        let weight = criticality_weight(urgency, period_ps);
        weights
            .entry(edge.net)
            .and_modify(|known| *known = (*known).max(weight))
            .or_insert(weight);
    }
    weights
}

fn timing_placement_weights(
    timing: &TimingReport,
    constraints: &TimingConstraints,
) -> BTreeMap<(NetId, CellPinId), u64> {
    let Some(period_ps) = constraints.clock_periods_ps().values().copied().min() else {
        return BTreeMap::new();
    };
    let Some(worst_slack_ps) = timing.worst_slack_ps else {
        return BTreeMap::new();
    };
    let delays = timing
        .net_delays
        .iter()
        .map(|edge| ((edge.net, edge.sink), edge.delay.max_ps))
        .collect::<BTreeMap<_, _>>();
    let period = i128::from(period_ps.max(1));
    let critical_limit = worst_slack_ps + period;
    timing
        .net_setup_slacks
        .iter()
        .map(|edge| {
            let urgency = (critical_limit - edge.slack_ps).clamp(0, period);
            let criticality = criticality_weight(urgency, period);
            let key = (edge.net, edge.sink);
            let delay_ps = delays.get(&key).copied().unwrap_or(0);
            (
                key,
                delay_weighted_criticality(criticality, delay_ps, period_ps),
            )
        })
        .collect()
}

fn delay_weighted_criticality(criticality: u64, delay_ps: u64, period_ps: u64) -> u64 {
    const ROUTING_SEGMENTS_PER_PERIOD: u64 = 4;
    let period_ps = period_ps.max(1);
    let delay_fraction = delay_ps
        .saturating_mul(ROUTING_SEGMENTS_PER_PERIOD)
        .min(period_ps);
    1 + criticality.saturating_sub(1).saturating_mul(delay_fraction) / period_ps
}

fn hold_sink_min_delays(timing: &TimingReport) -> BTreeMap<(NetId, CellPinId), u64> {
    let delays_by_sink = timing
        .net_delays
        .iter()
        .map(|delay| (delay.sink, delay))
        .collect::<BTreeMap<_, _>>();
    let mut minimums = BTreeMap::<(NetId, CellPinId), u64>::new();
    for check in &timing.hold_checks {
        if check.slack_ps >= 0 {
            continue;
        }
        let Some(delay) = delays_by_sink.get(&check.data_pin) else {
            continue;
        };
        let deficit_ps = u64::try_from(check.slack_ps.unsigned_abs()).unwrap_or(u64::MAX);
        let minimum_ps = delay.delay.min_ps.saturating_add(deficit_ps);
        minimums
            .entry((delay.net, delay.sink))
            .and_modify(|known| *known = (*known).max(minimum_ps))
            .or_insert(minimum_ps);
    }
    minimums
}

fn freeze_routes_except(routes: &[NetRoute], released: &BTreeSet<NetId>) -> RoutingConstraints {
    let mut frozen = RoutingConstraints::new();
    for route in routes {
        if !released.contains(&route.net) {
            frozen.add_route(route.clone());
        }
    }
    frozen
}

fn freeze_unchanged_routes(
    design: &Design,
    implementation: &PnrResult,
    placement: &Placement,
    base: &RoutingConstraints,
    released: &BTreeSet<NetId>,
) -> RoutingConstraints {
    let moved = implementation
        .placement
        .bindings()
        .iter()
        .zip(placement.bindings())
        .enumerate()
        .filter_map(|(index, (before, after))| (before != after).then_some(CellId(index)))
        .collect::<BTreeSet<_>>();
    let mut frozen = base.clone();
    for route in &implementation.routes {
        if base.routes().contains_key(&route.net) || released.contains(&route.net) {
            continue;
        }
        let net = &design.nets()[route.net.0];
        let touches_moved_cell = std::iter::once(net.driver)
            .chain(net.sinks.iter().copied())
            .any(|pin| moved.contains(&design.pins()[pin.0].cell));
        if !touches_moved_cell {
            frozen.add_route(route.clone());
        }
    }
    frozen
}

fn criticality_weight(urgency: i128, period_ps: i128) -> u64 {
    const SCALE: u64 = 1 << 10;
    const MAX_EXTRA_WEIGHT: u64 = 63;
    let scaled = u64::try_from((urgency * i128::from(SCALE)) / period_ps)
        .unwrap_or(SCALE)
        .min(SCALE);
    let powered = scaled.pow(4);
    1 + powered.saturating_mul(MAX_EXTRA_WEIGHT) / SCALE.pow(4)
}

fn timing_score(timing: &TimingReport) -> (i128, i128, i128) {
    let setup = timing.worst_slack_ps.unwrap_or(i128::MIN);
    let hold = timing.worst_hold_slack_ps.unwrap_or(i128::MIN);
    (setup.min(hold), setup, hold)
}

type TimingCandidate = (PnrResult, TimingReport);
type TimingAxes = (i128, i128, usize);

fn select_timing_frontier(candidates: Vec<TimingCandidate>) -> Vec<TimingCandidate> {
    let axes = candidates
        .iter()
        .map(|(implementation, timing)| {
            (
                timing.worst_slack_ps.unwrap_or(i128::MIN),
                timing.worst_hold_slack_ps.unwrap_or(i128::MIN),
                implementation.total_pips,
            )
        })
        .collect::<Vec<_>>();
    let selected = pareto_axes_indices(&axes, TIMING_FRONTIER_WIDTH);
    let mut candidates = candidates.into_iter().map(Some).collect::<Vec<_>>();
    selected
        .into_iter()
        .map(|index| {
            candidates[index]
                .take()
                .expect("a frontier index is unique")
        })
        .collect()
}

fn select_timing_beam(
    candidates: Vec<TimingCandidate>,
    setup_focused: bool,
) -> Vec<TimingCandidate> {
    let axes = candidates
        .iter()
        .map(|(implementation, timing)| {
            (
                timing.worst_slack_ps.unwrap_or(i128::MIN),
                timing.worst_hold_slack_ps.unwrap_or(i128::MIN),
                implementation.total_pips,
            )
        })
        .collect::<Vec<_>>();
    let eligible = (0..axes.len()).collect::<Vec<_>>();
    let selected = if setup_focused {
        setup_focused_axes_indices(&axes, &eligible, SETUP_FOCUSED_BEAM_WIDTH)
    } else {
        extreme_axes_indices(&axes, &eligible, TIMING_FRONTIER_WIDTH)
    };
    let mut candidates = candidates.into_iter().map(Some).collect::<Vec<_>>();
    selected
        .into_iter()
        .map(|index| candidates[index].take().expect("a beam index is unique"))
        .collect()
}

fn setup_focused_axes_indices(axes: &[TimingAxes], eligible: &[usize], width: usize) -> Vec<usize> {
    let mut selected = BTreeSet::new();
    if let Some(balanced) = eligible
        .iter()
        .copied()
        .max_by_key(|&index| timing_objective_rank(axes[index], index, TimingObjective::Balanced))
    {
        selected.insert(balanced);
    }
    let hold_clean = eligible
        .iter()
        .copied()
        .filter(|&index| axes[index].1 >= 0)
        .collect::<Vec<_>>();
    let setup_pool = if hold_clean.is_empty() {
        eligible
    } else {
        &hold_clean
    };
    if let Some(setup) = setup_pool
        .iter()
        .copied()
        .max_by_key(|&index| timing_objective_rank(axes[index], index, TimingObjective::Setup))
    {
        selected.insert(setup);
    }
    let mut balanced = eligible.to_vec();
    balanced.sort_by_key(|&index| {
        Reverse(timing_objective_rank(
            axes[index],
            index,
            TimingObjective::Balanced,
        ))
    });
    for index in balanced {
        if selected.len() == width {
            break;
        }
        selected.insert(index);
    }
    let mut selected = selected.into_iter().collect::<Vec<_>>();
    selected.sort_by_key(|&index| {
        Reverse(timing_objective_rank(
            axes[index],
            index,
            TimingObjective::Balanced,
        ))
    });
    selected
}

fn pareto_axes_indices(axes: &[TimingAxes], width: usize) -> Vec<usize> {
    let nondominated = (0..axes.len())
        .filter(|&candidate| {
            !(0..axes.len()).any(|other| {
                other != candidate
                    && timing_axes_dominate(axes[other], other, axes[candidate], candidate)
            })
        })
        .collect::<Vec<_>>();
    extreme_axes_indices(axes, &nondominated, width)
}

fn extreme_axes_indices(axes: &[TimingAxes], eligible: &[usize], width: usize) -> Vec<usize> {
    let mut selected = BTreeSet::new();
    for objective in [
        TimingObjective::Balanced,
        TimingObjective::Setup,
        TimingObjective::Hold,
    ] {
        if let Some(best) = eligible
            .iter()
            .copied()
            .max_by_key(|&index| timing_objective_rank(axes[index], index, objective))
        {
            selected.insert(best);
        }
        if selected.len() == width {
            break;
        }
    }
    let mut balanced = eligible.to_vec();
    balanced.sort_by_key(|&index| {
        Reverse(timing_objective_rank(
            axes[index],
            index,
            TimingObjective::Balanced,
        ))
    });
    for index in balanced {
        if selected.len() == width {
            break;
        }
        selected.insert(index);
    }
    let mut selected = selected.into_iter().collect::<Vec<_>>();
    selected.sort_by_key(|&index| {
        Reverse(timing_objective_rank(
            axes[index],
            index,
            TimingObjective::Balanced,
        ))
    });
    selected
}

fn timing_axes_dominate(
    left: TimingAxes,
    left_index: usize,
    right: TimingAxes,
    right_index: usize,
) -> bool {
    let (left_setup, left_hold, left_pips) = left;
    let (right_setup, right_hold, right_pips) = right;
    left_setup >= right_setup
        && left_hold >= right_hold
        && (left_setup > right_setup
            || left_hold > right_hold
            || left_pips < right_pips
            || (left_pips == right_pips && left_index < right_index))
}

#[derive(Clone, Copy)]
enum TimingObjective {
    Balanced,
    Setup,
    Hold,
}

fn timing_objective_rank(
    (setup, hold, pips): TimingAxes,
    index: usize,
    objective: TimingObjective,
) -> (i128, i128, i128, Reverse<usize>, Reverse<usize>) {
    let balanced = setup.min(hold);
    let (first, second, third) = match objective {
        TimingObjective::Balanced => (balanced, setup, hold),
        TimingObjective::Setup => (setup, hold, balanced),
        TimingObjective::Hold => (hold, setup, balanced),
    };
    (first, second, third, Reverse(pips), Reverse(index))
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
            let metadata = architecture.pip_metadata(pip_id);
            let class = speed_grade
                .pip_classes
                .get(metadata.timing_class)
                .ok_or_else(|| Ecp5FlowError::MissingPipTimingClass {
                    speed_grade: speed_grade.name.clone(),
                    timing_class: metadata.timing_class.to_owned(),
                })?;
            let fanout = source_fanout[&device.pips()[pip_id.0].from];
            Ok((pip_id, pip_class_delay(class, fanout)?))
        })
        .collect()
}

fn pip_class_delay(class: &PipClassTimingRecord, fanout: u64) -> Result<DelayRange, Ecp5FlowError> {
    let min_ps = class
        .fanout_adder
        .min_ps
        .checked_mul(fanout)
        .and_then(|delay| class.base.min_ps.checked_add(delay))
        .ok_or(Ecp5FlowError::TimingDelayOverflow)?;
    let max_ps = class
        .fanout_adder
        .max_ps
        .checked_mul(fanout)
        .and_then(|delay| class.base.max_ps.checked_add(delay))
        .ok_or(Ecp5FlowError::TimingDelayOverflow)?;
    Ok(DelayRange::from_independent_corners(min_ps, max_ps))
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
    let carry_slices = packing
        .carry_pairs()
        .iter()
        .flat_map(|pair| [(pair[0], "TRELLIS_CARRY0"), (pair[1], "TRELLIS_CARRY1")])
        .collect::<BTreeMap<_, _>>();
    let mut model = TimingModel::new();
    for (index, cell) in design.cells().iter().enumerate() {
        let cell_id = CellId(index);
        let cell_type = if let Some(&cell_type) = carry_slices.get(&cell_id) {
            cell_type
        } else {
            match cell.kind {
                ResourceKind::Lut(4) => "TRELLIS_COMB",
                ResourceKind::Register => "TRELLIS_FF",
                ResourceKind::Clock => "DCCA",
                _ => continue,
            }
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
    use std::num::NonZeroU32;

    use struo_celox::ecp5_simulator;
    use struo_ir::{ArithmeticOp, ClockEdge, Netlist, RegisterCell};
    use struo_target_ecp5::{
        ArithmeticMapping, Ecp5Netlist, MappingOptions, map_to_ecp5, map_to_ecp5_with_options,
    };
    use texo_model::{BelId, CellId, Design, Device, NetId, PinDirection, ResourceKind};
    use texo_pnr::{NetRoute, PlacementConstraints};
    use texo_struo::import_ecp5;
    use texo_target_ecp5::{
        PipClassTimingRecord, TimingCornersRecord, find_global_clock_requirements, pack_lut_ffs,
        parse_lpf, read_architecture, resolve_lpf_port_cells,
    };

    use super::{
        Ecp5FlowError, Ecp5FlowOptions, Evidence, Gate, criticality_weight,
        delay_weighted_criticality, ecp5_timing_constraints, ecp5_timing_model,
        extreme_axes_indices, find_cell_pin, freeze_routes_except, implement, implement_struo_ecp5,
        implement_with_constraints, pareto_axes_indices, pip_class_delay,
        setup_focused_axes_indices, verify_post_map_with_celox,
    };

    const ECP5_FIXTURE: &str = include_str!("../../texo-target-ecp5/fixtures/minimal-ecp5.json");

    #[test]
    fn timing_criticality_emphasizes_the_worst_edges() {
        assert_eq!(criticality_weight(0, 4_000), 1);
        assert_eq!(criticality_weight(2_000, 4_000), 4);
        assert_eq!(criticality_weight(3_000, 4_000), 20);
        assert_eq!(criticality_weight(4_000, 4_000), 64);
    }

    #[test]
    fn placement_criticality_favors_delay_consuming_edges() {
        assert_eq!(delay_weighted_criticality(64, 0, 4_000), 1);
        assert_eq!(delay_weighted_criticality(64, 500, 4_000), 32);
        assert_eq!(delay_weighted_criticality(64, 1_000, 4_000), 64);
        assert_eq!(delay_weighted_criticality(64, 2_000, 4_000), 64);
    }

    #[test]
    fn timing_frontier_keeps_balanced_setup_and_hold_extremes() {
        let axes = [
            (-100, -100, 100),
            (10, -300, 90),
            (-300, 20, 90),
            (-200, -200, 80),
            (-100, -100, 110),
        ];

        assert_eq!(pareto_axes_indices(&axes, 3), vec![0, 1, 2]);
    }

    #[test]
    fn timing_beam_keeps_non_monotonic_search_trajectories() {
        let axes = [
            (-500, -500, 100),
            (-400, -800, 90),
            (-800, -300, 90),
            (-900, -900, 80),
        ];

        assert_eq!(extreme_axes_indices(&axes, &[0, 1, 2, 3], 3), vec![0, 1, 2]);
    }

    #[test]
    fn setup_focused_beam_protects_a_hold_clean_candidate() {
        let axes = [
            (-100, -10, 100),
            (-200, 20, 90),
            (-300, 50, 80),
            (-50, -500, 70),
        ];

        assert_eq!(
            setup_focused_axes_indices(&axes, &[0, 1, 2, 3], 2),
            vec![0, 1]
        );
    }

    #[test]
    fn local_hold_repair_releases_only_violating_nets() {
        let routes = [
            NetRoute {
                net: NetId(0),
                wires: Vec::new(),
                pips: Vec::new(),
            },
            NetRoute {
                net: NetId(1),
                wires: Vec::new(),
                pips: Vec::new(),
            },
        ];

        let frozen = freeze_routes_except(&routes, &std::collections::BTreeSet::from([NetId(1)]));

        assert!(frozen.routes().contains_key(&NetId(0)));
        assert!(!frozen.routes().contains_key(&NetId(1)));
    }

    #[test]
    fn preserves_non_monotonic_pip_corners_for_sta() {
        let class = PipClassTimingRecord {
            base: TimingCornersRecord {
                min_ps: 59,
                typ_ps: 54,
                max_ps: 48,
            },
            fanout_adder: TimingCornersRecord {
                min_ps: 7,
                typ_ps: 5,
                max_ps: 3,
            },
        };

        let delay = pip_class_delay(&class, 2).unwrap();
        assert_eq!((delay.min_ps, delay.max_ps), (73, 54));
    }

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
    fn applies_characterized_timing_to_split_carry_slices() {
        let mut source = Netlist::new("carry");
        let width = NonZeroU32::new(2).unwrap();
        let lhs = source.add_input_port("lhs", width);
        let rhs = source.add_input_port("rhs", width);
        let sum = source
            .add_arithmetic(ArithmeticOp::Add, &lhs, &rhs)
            .unwrap();
        source.add_output_port("sum", &sum).unwrap();
        let mapped = map_to_ecp5_with_options(
            &source,
            MappingOptions {
                arithmetic: ArithmeticMapping::CarryChain,
                ..MappingOptions::default()
            },
        )
        .unwrap();
        let imported = import_ecp5(&mapped).unwrap();
        let architecture = read_architecture(ECP5_FIXTURE.as_bytes()).unwrap();
        let mut packing = pack_lut_ffs(imported.design(), &architecture).unwrap();
        packing
            .pack_carry_pairs(
                imported.design(),
                &architecture,
                imported.carry_pairs().iter().copied(),
            )
            .unwrap();

        let model = ecp5_timing_model(
            imported.design(),
            &packing,
            &architecture.speed_grades()["6"],
        )
        .unwrap();
        let pair = imported.carry_pairs()[0];
        let first_a = find_cell_pin(imported.design(), pair[0], "A").unwrap();
        let first_fco = find_cell_pin(imported.design(), pair[0], "FCO").unwrap();
        let second_fci = find_cell_pin(imported.design(), pair[1], "FCI").unwrap();
        let second_f = find_cell_pin(imported.design(), pair[1], "F").unwrap();

        assert_eq!(model.cell_arc(first_a, first_fco).unwrap().max_ps, 447);
        assert_eq!(model.cell_arc(second_fci, second_f).unwrap().max_ps, 474);
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
