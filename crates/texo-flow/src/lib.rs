//! Flow orchestration and explicit verification evidence.

use std::cmp::Reverse;
use std::collections::hash_map::DefaultHasher;
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;
use std::hash::{Hash, Hasher};
use std::sync::Arc;
use std::time::{Duration, Instant};

use texo_model::{
    BelId, CellId, CellPinId, Design, Device, NetId, PinDirection, PipId, ResourceKind,
};
pub use texo_pnr::RoutingProgress;
use texo_pnr::{
    NetRoute, Placement, PlacementConstraints, PlacementRefinementWorkspace, PlacementRefiner,
    PnrError, PnrResult, RouteCapacityProjection, RoutingConstraints, RoutingCosts,
    RoutingWorkspace, place_analytically_with_net_sink_weights, place_and_route_with_constraints,
    placement_from_partial_bindings, retain_route_for_sinks,
    route_with_timing_costs_workspace_and_progress, route_with_workspace_and_progress,
    swap_placement_cells,
};
use texo_struo::{ImportedEcp5Design, PrimitiveMetadata};
use texo_target_ecp5::{
    BlockRamRequirement, DEFAULT_GLOBAL_CLOCK_FANOUT, DelayRangeRecord, Ecp5Architecture,
    Ecp5GlobalRoutingCache, Ecp5Packing, LpfConstraints, LpfError, LutFfPair, PackingError,
    PipClassTimingRecord, SpeedGradeRecord, find_global_clock_requirements, lut_ff_pair_candidates,
    pack_lut_ffs_excluding, pack_lut_ffs_with_pairs, resolve_lpf_port_cells,
};
use texo_timing::{
    DelayRange, PICOSECONDS_PER_SECOND, TimingConstraints, TimingError, TimingModel, TimingReport,
    analyze_timing, estimate_edge_delay,
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
    /// Exponent applied to sink criticality weights in the timing-driven
    /// analytical solve. One keeps the plain weights; larger values sharpen
    /// the contrast around critical paths and select a different placement
    /// basin. Recorded in checkpoints so artifacts remain reproducible.
    pub placement_weight_exponent: u32,
    /// Optional cell-name to BEL-name bindings used instead of native initial
    /// placement. Missing synthetic cells are completed from packing groups.
    pub initial_placement: Option<&'a BTreeMap<String, String>>,
    /// Optional explicit dedicated-path LUT-to-FF pairs, named as
    /// `LUT -> FF`. This must accompany placements imported after packing.
    pub lut_ff_pairs: Option<&'a BTreeMap<String, String>>,
    /// Run one characterized timing-driven full reroute before any placement
    /// refinement. Useful for separating placement quality from the
    /// connectivity-only bootstrap route.
    pub initial_timing_reroute: bool,
    /// Whether post-route timing closure may change the initial placement and
    /// routing. Disable this for placement A/B measurements.
    pub optimize_timing: bool,
}

impl Default for Ecp5FlowOptions<'_> {
    fn default() -> Self {
        Self {
            speed_grade: None,
            package: None,
            lpf: None,
            allow_unconstrained_io: false,
            global_clock_fanout: DEFAULT_GLOBAL_CLOCK_FANOUT,
            placement_weight_exponent: 1,
            initial_placement: None,
            lut_ff_pairs: None,
            initial_timing_reroute: false,
            optimize_timing: true,
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
    /// Placement-weight exponent the flow was configured with.
    pub placement_weight_exponent: u32,
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
    /// A concrete worst-path cell proposal is about to be routed and timed.
    CriticalPathMove {
        /// Cell whose placement unit is being moved.
        cell: CellId,
        /// Current BEL of the selected cell.
        from: BelId,
        /// Proposed BEL of the selected cell.
        to: BelId,
    },
    /// Dedicated primary-clock trees for the timing-driven placement are locked.
    TimingDrivenGlobalClocksRouted,
    /// Progress within timing-driven negotiated routing.
    TimingDrivenRouting(RoutingProgress),
    /// Timing-driven negotiated routing is complete.
    TimingDrivenRouted,
    /// One routed candidate has been evaluated by static timing analysis.
    TimingSnapshot {
        /// Smallest setup slack, when a setup endpoint is constrained.
        worst_setup_ps: Option<i128>,
        /// Sum of negative setup slacks; zero means no setup violation.
        setup_tns_ps: i128,
        /// Number of setup endpoints with negative slack.
        setup_violations: usize,
        /// Smallest hold slack, when a hold endpoint is constrained.
        worst_hold_ps: Option<i128>,
        /// Sum of negative hold slacks; zero means no hold violation.
        hold_ths_ps: i128,
        /// Number of hold endpoints with negative slack.
        hold_violations: usize,
    },
    /// Whether the immediately preceding timing trial improves the incumbent objective.
    TimingTrialDecision {
        /// `true` when the trial is eligible to replace the incumbent.
        improves_objective: bool,
    },
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
#[allow(clippy::too_many_lines)]
pub fn implement_struo_ecp5_with_progress(
    imported: &ImportedEcp5Design,
    architecture: &Ecp5Architecture,
    options: Ecp5FlowOptions<'_>,
    evidence: &mut Evidence,
    mut progress: impl FnMut(Ecp5FlowStage),
) -> Result<Ecp5FlowResult, Ecp5FlowError> {
    let flow_started = Instant::now();
    let mut phase_started = flow_started;
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
    let mut packing = match options.lut_ff_pairs {
        Some(pairs) => {
            pack_lut_ffs_with_pairs(&design, architecture, named_lut_ff_pairs(&design, pairs)?)?
        }
        None => pack_lut_ffs_excluding(&design, architecture, constant_luts)?,
    };
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
    report_metric_phase("packing", &mut phase_started);

    let mut placement_refinement_workspace = PlacementRefinementWorkspace::new();

    let mut staged_evidence = evidence.clone();
    staged_evidence.record(Gate::MappedNetlistComplete);
    let placement = if let Some(bindings) = options.initial_placement {
        named_initial_placement(&design, architecture, &packing, bindings)?
    } else {
        let placement_refiner = PlacementRefiner::new_with_workspace(
            &design,
            architecture.device(),
            packing.constraints(),
            &mut placement_refinement_workspace,
        )?;
        initial_analytical_placement(&design, architecture, &placement_refiner)?
    };
    progress(Ecp5FlowStage::Placed);
    report_metric_phase("initial_placement", &mut phase_started);
    let mut global_routing_cache = architecture.global_routing_cache();
    let routing = packing.global_routing_constraints_cached(
        &design,
        architecture,
        &placement,
        &mut global_routing_cache,
    )?;
    progress(Ecp5FlowStage::GlobalClocksRouted);
    report_metric_phase("initial_global_routing", &mut phase_started);
    let mut routing_workspace = RoutingWorkspace::new(architecture.device());
    let mut initial_implementation = route_with_workspace_and_progress(
        &design,
        architecture.device(),
        placement,
        &routing,
        &mut routing_workspace,
        |event| progress(Ecp5FlowStage::Routing(event)),
    )?;
    progress(Ecp5FlowStage::Routed);
    let mut timing_model = ecp5_timing_model(&design, &packing, speed_grade)?;
    let mut timing_constraints = ecp5_timing_constraints(&design, &packing)?;
    let mut initial_timing = analyze_ecp5_implementation(
        &design,
        architecture,
        speed_grade,
        &initial_implementation,
        &timing_model,
        &timing_constraints,
    )?;
    progress(timing_snapshot(&initial_timing));
    if options.initial_timing_reroute && !initial_timing.met_timing() {
        let mut costs = ecp5_routing_costs(
            architecture,
            speed_grade,
            timing_net_weights(&initial_timing, &timing_constraints),
        )?;
        costs.set_sink_criticalities(timing_arc_weights(&initial_timing, &timing_constraints));
        initial_implementation = route_with_timing_costs_workspace_and_progress(
            &design,
            architecture.device(),
            initial_implementation.placement.clone(),
            &routing,
            &costs,
            &mut routing_workspace,
            |event| progress(Ecp5FlowStage::TimingDrivenRouting(event)),
        )?;
        progress(Ecp5FlowStage::TimingDrivenRouted);
        initial_timing = analyze_ecp5_implementation(
            &design,
            architecture,
            speed_grade,
            &initial_implementation,
            &timing_model,
            &timing_constraints,
        )?;
        progress(timing_snapshot(&initial_timing));
    }
    report_metric_phase("initial_route_and_timing", &mut phase_started);
    let mut closure_routing_costs = options
        .optimize_timing
        .then(|| {
            ecp5_routing_costs(
                architecture,
                speed_grade,
                timing_net_weights(&initial_timing, &timing_constraints),
            )
        })
        .transpose()?;
    if let Some(costs) = closure_routing_costs.as_mut() {
        costs.set_sink_criticalities(timing_arc_weights(&initial_timing, &timing_constraints));
    }

    if options.optimize_timing
        && options.initial_placement.is_none()
        && options.lut_ff_pairs.is_none()
        && !initial_timing.met_timing()
        && let Some(costs) = closure_routing_costs.as_mut()
        && let Some(candidate) = optimize_dedicated_lut_ff_edge(
            &design,
            architecture,
            speed_grade,
            &packing,
            &initial_implementation,
            &initial_timing,
            costs,
            &mut global_routing_cache,
            &mut routing_workspace,
            &mut progress,
        )?
    {
        packing = candidate.packing;
        initial_implementation = candidate.implementation;
        initial_timing = candidate.timing;
        timing_model = ecp5_timing_model(&design, &packing, speed_grade)?;
        timing_constraints = ecp5_timing_constraints(&design, &packing)?;
        costs.set_net_criticalities(timing_net_weights(&initial_timing, &timing_constraints));
        costs.set_sink_criticalities(timing_arc_weights(&initial_timing, &timing_constraints));
    }
    report_metric_phase("dedicated_edge_search", &mut phase_started);

    let (implementation, timing) = if let Some(costs) = closure_routing_costs.as_mut() {
        let placement_refiner = PlacementRefiner::new_with_workspace(
            &design,
            architecture.device(),
            packing.constraints(),
            &mut placement_refinement_workspace,
        )?;
        report_metric_phase("closure_refiner_build", &mut phase_started);
        TimingDrivenContext {
            design: &design,
            architecture,
            packing: &packing,
            placement_refiner: &placement_refiner,
            global_routing_cache: &mut global_routing_cache,
            speed_grade,
            timing_model: &timing_model,
            timing_constraints: &timing_constraints,
            routing_workspace: &mut routing_workspace,
            stalled_ripup_wns_ps: None,
            critical_move_trials: BTreeSet::new(),
        }
        .optimize(initial_implementation, initial_timing, costs, &mut progress)?
    } else {
        (initial_implementation, initial_timing)
    };
    report_metric_phase("timing_closure", &mut phase_started);
    progress(Ecp5FlowStage::Timed);
    staged_evidence.record(Gate::PhysicalImplementation);
    if timing.met_timing() {
        staged_evidence.record(Gate::TimingClosure);
    }
    *evidence = staged_evidence;
    if metrics_enabled() {
        eprintln!("[metrics] flow_total={:?}", flow_started.elapsed());
    }

    Ok(Ecp5FlowResult {
        speed_grade: speed_grade_name.into(),
        design,
        primitive_metadata: imported.metadata().clone(),
        absorbed_inputs: imported.absorbed_inputs().clone(),
        packing,
        implementation,
        timing,
        placement_weight_exponent: options.placement_weight_exponent,
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

struct DedicatedEdgeCandidate {
    packing: Ecp5Packing,
    implementation: PnrResult,
    timing: TimingReport,
}

/// Evaluates a bounded portfolio of dedicated LUT→FF edge transfers against
/// the routed topology. Each proposal swaps the paired and general-routed FF
/// BELs, reroutes only nets incident to those cells, and is accepted only when
/// full STA improves the incumbent objective.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn optimize_dedicated_lut_ff_edge(
    design: &Design,
    architecture: &Ecp5Architecture,
    speed_grade: &SpeedGradeRecord,
    packing: &Ecp5Packing,
    implementation: &PnrResult,
    timing: &TimingReport,
    routing_costs: &mut RoutingCosts,
    global_routing_cache: &mut Ecp5GlobalRoutingCache<'_>,
    routing_workspace: &mut RoutingWorkspace,
    progress: &mut impl FnMut(Ecp5FlowStage),
) -> Result<Option<DedicatedEdgeCandidate>, Ecp5FlowError> {
    let current_by_lut = packing
        .lut_ff_pairs()
        .iter()
        .map(|pair| (pair.lut, pair.ff))
        .collect::<BTreeMap<_, _>>();
    let general_ffs = packing
        .general_routing_ffs()
        .iter()
        .copied()
        .collect::<BTreeSet<_>>();
    let legal = lut_ff_pair_candidates(design, []);
    let legal = legal
        .into_iter()
        .map(|pair| (pair.lut, pair.ff))
        .collect::<BTreeSet<_>>();
    let delays = timing
        .net_delays
        .iter()
        .map(|delay| ((delay.net, delay.sink), delay.delay.max_ps))
        .collect::<BTreeMap<_, _>>();
    let mut trials = timing
        .net_setup_slacks
        .iter()
        .filter_map(|edge| {
            if edge.slack_ps >= 0 {
                return None;
            }
            let sink_pin = design.pins().get(edge.sink.0)?;
            let ff = sink_pin.cell;
            if sink_pin.name != "DI" || !general_ffs.contains(&ff) {
                return None;
            }
            let lut = design.pins()[design.nets().get(edge.net.0)?.driver.0].cell;
            let old_ff = *current_by_lut.get(&lut)?;
            legal.contains(&(lut, ff)).then_some((
                edge.slack_ps,
                Reverse(delays.get(&(edge.net, edge.sink)).copied().unwrap_or(0)),
                lut,
                old_ff,
                ff,
            ))
        })
        .collect::<Vec<_>>();
    trials.sort_unstable();
    let mut selected_luts = BTreeSet::new();
    trials.retain(|trial| selected_luts.insert(trial.2));
    trials.truncate(MAX_DEDICATED_EDGE_TRIALS);
    if trials.is_empty() {
        return Ok(None);
    }

    routing_costs.set_max_iterations(LOCAL_TRIAL_ROUTING_ITERATIONS);
    let incumbent_score = timing_score(timing);
    let mut best = None;
    let mut best_score = incumbent_score;
    for (slack_ps, _, lut, old_ff, new_ff) in trials {
        let mut trial_packing = packing.clone();
        let displaced = trial_packing.reassign_lut_ff_pair(design, lut, new_ff)?;
        debug_assert_eq!(displaced, old_ff);
        let trial_placement = match swap_placement_cells(
            design,
            architecture.device(),
            trial_packing.constraints(),
            &implementation.placement,
            old_ff,
            new_ff,
        ) {
            Ok(placement) => placement,
            Err(error) => {
                if metrics_enabled() {
                    eprintln!(
                        "[metrics] dedicated_edge_trial lut={} old_ff={} new_ff={} slack={} rejected=placement error={error}",
                        lut.0, old_ff.0, new_ff.0, slack_ps
                    );
                }
                continue;
            }
        };
        progress(Ecp5FlowStage::TimingDrivenPlaced);
        let base = match trial_packing.global_routing_constraints_cached(
            design,
            architecture,
            &trial_placement,
            global_routing_cache,
        ) {
            Ok(base) => base,
            Err(error) => {
                if metrics_enabled() {
                    eprintln!(
                        "[metrics] dedicated_edge_trial lut={} old_ff={} new_ff={} slack={} rejected=global_route error={error}",
                        lut.0, old_ff.0, new_ff.0, slack_ps
                    );
                }
                continue;
            }
        };
        progress(Ecp5FlowStage::TimingDrivenGlobalClocksRouted);
        let frozen = freeze_unchanged_routes(
            design,
            implementation,
            &trial_placement,
            &base,
            &BTreeSet::new(),
        );
        let trial_implementation = match route_with_timing_costs_workspace_and_progress(
            design,
            architecture.device(),
            trial_placement,
            &frozen,
            routing_costs,
            routing_workspace,
            |event| progress(Ecp5FlowStage::TimingDrivenRouting(event)),
        ) {
            Ok(implementation) => implementation,
            Err(error) => {
                if metrics_enabled() {
                    eprintln!(
                        "[metrics] dedicated_edge_trial lut={} old_ff={} new_ff={} slack={} rejected=routing error={error}",
                        lut.0, old_ff.0, new_ff.0, slack_ps
                    );
                }
                continue;
            }
        };
        progress(Ecp5FlowStage::TimingDrivenRouted);
        let trial_timing_model = ecp5_timing_model(design, &trial_packing, speed_grade)?;
        let trial_timing_constraints = ecp5_timing_constraints(design, &trial_packing)?;
        let trial_timing = analyze_ecp5_implementation(
            design,
            architecture,
            speed_grade,
            &trial_implementation,
            &trial_timing_model,
            &trial_timing_constraints,
        )?;
        progress(timing_snapshot(&trial_timing));
        let score = timing_score(&trial_timing);
        let improves = score > best_score;
        progress(Ecp5FlowStage::TimingTrialDecision {
            improves_objective: improves,
        });
        if metrics_enabled() {
            eprintln!(
                "[metrics] dedicated_edge_trial lut={} old_ff={} new_ff={} slack={} wns={:?} whs={:?} pips={} accepted={improves}",
                lut.0,
                old_ff.0,
                new_ff.0,
                slack_ps,
                trial_timing.worst_slack_ps,
                trial_timing.worst_hold_slack_ps,
                trial_implementation.total_pips,
            );
        }
        if improves {
            best_score = score;
            best = Some(DedicatedEdgeCandidate {
                packing: trial_packing,
                implementation: trial_implementation,
                timing: trial_timing,
            });
        }
    }
    routing_costs.reset_max_iterations();
    Ok(best)
}

struct TimingDrivenContext<'a, 'work> {
    design: &'a Design,
    architecture: &'a Ecp5Architecture,
    packing: &'a Ecp5Packing,
    placement_refiner: &'work PlacementRefiner<'a>,
    global_routing_cache: &'work mut Ecp5GlobalRoutingCache<'a>,
    speed_grade: &'a SpeedGradeRecord,
    timing_model: &'a TimingModel,
    timing_constraints: &'a TimingConstraints,
    routing_workspace: &'work mut RoutingWorkspace,
    /// Setup slack at which a full data-route ripup last failed. Re-arm only
    /// after placement improves WNS by roughly one general-routing tile; tiny
    /// local changes otherwise trigger the same expensive failed search in
    /// every closure round.
    stalled_ripup_wns_ps: Option<i128>,
    /// Exact seed-route/proposed-placement pairs already sent through a local
    /// negotiated route and STA. Closure rounds often rediscover the same
    /// move; only a changed route topology makes it worth evaluating again.
    critical_move_trials: BTreeSet<(u64, u64)>,
}

impl TimingDrivenContext<'_, '_> {
    fn optimize(
        &mut self,
        initial_implementation: PnrResult,
        initial_timing: TimingReport,
        routing_costs: &mut RoutingCosts,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<(PnrResult, TimingReport), Ecp5FlowError> {
        let mut phase_started = Instant::now();
        if initial_timing.met_timing() {
            return Ok((initial_implementation, initial_timing));
        }
        // The separate timing-driven analytical seed never won archive
        // selection on the measured designs (the connectivity-only solve wins
        // after routing), so refinement descends directly from the initial
        // implementation and the second solve's route-and-analyze is skipped.
        routing_costs
            .set_sink_criticalities(timing_arc_weights(&initial_timing, self.timing_constraints));
        let mut archive = vec![(initial_implementation, initial_timing)];
        let placement_refiner = self.placement_refiner;
        archive =
            self.refine_setup_monotonically(archive, placement_refiner, routing_costs, progress)?;
        report_metric_phase("closure_monotonic_refinement", &mut phase_started);
        emit_archive_metric("closure_monotonic", self, &archive);
        for _ in 0..MAX_LOCAL_CONNECTION_REFINEMENTS {
            let seed = archive
                .iter()
                .max_by_key(|(_, timing)| timing_score(timing))
                .expect("the timing archive is non-empty")
                .clone();
            let refinements = self.refine_local_connections(
                &seed.0,
                &seed.1,
                placement_refiner,
                routing_costs,
                progress,
            )?;
            let Some(improved) = refinements
                .into_iter()
                .max_by_key(|(_, timing)| timing_score(timing))
                .filter(|(_, timing)| timing_score(timing) > timing_score(&seed.1))
            else {
                break;
            };
            archive.push(improved);
            archive = select_timing_frontier(archive);
        }
        report_metric_phase("closure_local_connections", &mut phase_started);
        emit_archive_metric("closure_local", self, &archive);
        archive =
            self.close_setup_critically(archive, placement_refiner, routing_costs, progress)?;
        report_metric_phase("closure_critical_vertices", &mut phase_started);
        emit_archive_metric("closure_critical", self, &archive);
        if !archive
            .iter()
            .any(|(_, timing)| timing.worst_slack_ps.is_some_and(|slack| slack >= 0))
        {
            archive =
                self.escape_placement_basin(archive, placement_refiner, routing_costs, progress)?;
        }
        report_metric_phase("closure_basin_escape", &mut phase_started);
        emit_archive_metric("closure_escape", self, &archive);
        let mut hold_repairs = Vec::new();
        for (implementation, timing) in &archive {
            if timing.worst_slack_ps.is_none_or(|slack| slack < 0) {
                continue;
            }
            if let Some(repaired) =
                self.repair_hold_locally(implementation, timing, routing_costs, progress)?
                && repaired.1.worst_slack_ps.is_some_and(|slack| slack >= 0)
                && timing_score(&repaired.1) > timing_score(timing)
            {
                hold_repairs.push(repaired);
            }
        }
        archive.extend(hold_repairs);
        report_metric_phase("closure_hold_repair", &mut phase_started);
        archive = select_timing_frontier(archive);
        let (final_implementation, final_timing) = archive
            .into_iter()
            .max_by_key(|(_, timing)| timing_score(timing))
            .expect("the timing archive is non-empty");
        emit_placement_metric(
            "final",
            self.design,
            self.architecture.device(),
            &final_implementation.placement,
            Some(&final_timing),
        );
        Ok((final_implementation, final_timing))
    }

    /// Nets touching a cell whose BEL differs between the two placements.
    ///
    /// Unchanged placements keep their exact routes under
    /// `freeze_unchanged_routes`, so this is exactly the net set a proposal
    /// would renegotiate.
    fn moved_nets<'a>(
        &'a self,
        old: &Placement,
        new: &Placement,
    ) -> impl Iterator<Item = NetId> + 'a {
        let mut nets = BTreeSet::<NetId>::new();
        for (index, _) in old
            .bindings()
            .iter()
            .zip(new.bindings())
            .enumerate()
            .filter(|&(_, (old_bel, new_bel))| old_bel != new_bel)
        {
            for &pin_id in self.design.cells()[index].pins() {
                if let Some(net) = self.design.pins()[pin_id.0].net() {
                    nets.insert(net);
                }
            }
        }
        nets.into_iter()
    }

    /// Criticality-weighted estimated routing delay over the given nets.
    ///
    /// Comparing old and new placements on the same net set pre-screens
    /// proposals without paying a full route trial.
    fn weighted_net_estimate(
        &self,
        placement: &Placement,
        nets: &BTreeSet<NetId>,
        weights: &BTreeMap<NetId, u64>,
    ) -> Option<u64> {
        let mut total = 0_u64;
        for net in nets {
            let Some(&weight) = weights.get(net) else {
                continue;
            };
            let logical = &self.design.nets()[net.0];
            for &sink in &logical.sinks {
                total = total.saturating_add(weight.saturating_mul(estimate_edge_delay(
                    self.design,
                    self.architecture.device(),
                    placement,
                    logical.driver,
                    sink,
                    PRESCREEN_PS_PER_TILE_PS,
                    PRESCREEN_HOP_OVERHEAD_PS,
                )?));
            }
        }
        Some(total)
    }

    /// Rejects a placement proposal before routing when its criticality-weighted
    /// estimated delay over the renegotiated nets is clearly worse.
    ///
    /// The geometric model cannot see long-line shortcuts, so mildly negative
    /// estimates stay eligible: only a decisive regression skips the route
    /// trial. Measured unguarded, the filter cost 388 ps of WNS; with the
    /// margin it trims hopeless trials without touching the descent path.
    fn estimate_rejects(
        &self,
        old: &Placement,
        new: &Placement,
        weights: &BTreeMap<NetId, u64>,
    ) -> bool {
        let nets = self.moved_nets(old, new).collect::<BTreeSet<_>>();
        if nets.is_empty() {
            return true;
        }
        match (
            self.weighted_net_estimate(old, &nets, weights),
            self.weighted_net_estimate(new, &nets, weights),
        ) {
            (Some(old_estimate), Some(new_estimate)) => {
                new_estimate > old_estimate.saturating_add(old_estimate / 4)
            }
            _ => false,
        }
    }

    fn close_setup_critically(
        &mut self,
        mut archive: Vec<TimingCandidate>,
        placement_refiner: &PlacementRefiner<'_>,
        routing_costs: &mut RoutingCosts,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<Vec<TimingCandidate>, Ecp5FlowError> {
        for _ in 0..MAX_CRITICAL_CLOSURE_ROUNDS {
            archive = self.refine_critical_path_vertices(
                archive,
                placement_refiner,
                routing_costs,
                progress,
            )?;
            archive =
                self.refine_critical_routes_multiresolution(archive, routing_costs, progress)?;
            let setup_closed = archive
                .iter()
                .any(|(_, timing)| timing.worst_slack_ps.is_some_and(|slack| slack >= 0));
            if setup_closed {
                break;
            }
        }
        Ok(archive)
    }

    fn refine_setup_monotonically(
        &mut self,
        mut archive: Vec<TimingCandidate>,
        placement_refiner: &PlacementRefiner<'_>,
        routing_costs: &mut RoutingCosts,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<Vec<TimingCandidate>, Ecp5FlowError> {
        for _ in 0..MAX_INCREMENTAL_REFINEMENTS {
            if archive.iter().any(|(_, timing)| timing.met_timing()) {
                break;
            }
            let seed = archive
                .iter()
                .max_by_key(|(implementation, timing)| {
                    (timing_score(timing), Reverse(implementation.total_pips))
                })
                .expect("the timing archive is non-empty")
                .clone();
            let mut improved = None;
            for max_units in REFINED_PLACEMENT_UNIT_LIMITS {
                let Some(child) = self.refine_candidate(
                    &seed.0,
                    &seed.1,
                    placement_refiner,
                    routing_costs,
                    max_units,
                    progress,
                )?
                else {
                    progress(Ecp5FlowStage::TimingTrialDecision {
                        improves_objective: false,
                    });
                    continue;
                };
                let improves_objective = timing_score(&child.1) > timing_score(&seed.1);
                progress(Ecp5FlowStage::TimingTrialDecision { improves_objective });
                if improves_objective {
                    improved = Some(child);
                    break;
                }
            }
            let Some(improved) = improved else {
                break;
            };
            archive.push(improved);
            archive = select_timing_frontier(archive);
        }
        Ok(archive)
    }
    fn refine_critical_path_vertices(
        &mut self,
        mut archive: Vec<TimingCandidate>,
        placement_refiner: &PlacementRefiner<'_>,
        routing_costs: &mut RoutingCosts,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<Vec<TimingCandidate>, Ecp5FlowError> {
        for move_distance in CRITICAL_PATH_MOVE_DISTANCES {
            for _ in 0..MAX_CRITICAL_PATH_VERTEX_REFINEMENTS {
                if archive
                    .iter()
                    .any(|(_, timing)| timing.worst_slack_ps.is_some_and(|slack| slack >= 0))
                {
                    return Ok(archive);
                }
                let seed = archive
                    .iter()
                    .max_by_key(|(_, timing)| timing_score(timing))
                    .expect("the timing archive is non-empty")
                    .clone();
                let Some(improved) = self
                    .refine_critical_path_cells(
                        &seed.0,
                        &seed.1,
                        placement_refiner,
                        routing_costs,
                        move_distance,
                        progress,
                    )?
                    .into_iter()
                    .max_by_key(|(_, timing)| timing_score(timing))
                    .filter(|(_, timing)| timing_score(timing) > timing_score(&seed.1))
                else {
                    break;
                };
                archive.push(improved);
                archive = select_timing_frontier(archive);
            }
        }
        Ok(archive)
    }

    fn refine_critical_routes_multiresolution(
        &mut self,
        mut archive: Vec<TimingCandidate>,
        routing_costs: &mut RoutingCosts,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<Vec<TimingCandidate>, Ecp5FlowError> {
        for delay_quantum_ps in DETAILED_ROUTING_QUANTA_PS {
            let seed = archive
                .iter()
                .max_by_key(|(implementation, timing)| {
                    (timing_score(timing), Reverse(implementation.total_pips))
                })
                .expect("the timing archive is non-empty")
                .clone();
            if seed.1.worst_slack_ps.is_none() {
                break;
            }
            let seed_wns_ps = seed.1.worst_slack_ps.expect("checked above");
            if !should_retry_global_ripup(self.stalled_ripup_wns_ps, seed_wns_ps) {
                // A previous global renegotiation failed, and intervening
                // placement refinement has not shifted the timing basin far
                // enough to justify scanning every data net again.
                break;
            }
            // Timing-driven ripup: lift every data-net route so the whole
            // design renegotiates while the failing connections pull toward
            // fast resources with exact picosecond costs. Freezing satisfied
            // nets here would let them keep hoarding the short resources the
            // failing paths need, which is why only clock trunks stay locked.
            let detailed_sinks = seed
                .1
                .net_setup_slacks
                .iter()
                .filter_map(|edge| (edge.slack_ps < 0).then_some((edge.net, edge.sink)))
                .collect::<BTreeSet<_>>();
            if detailed_sinks.is_empty() {
                break;
            }
            let detailed_nets = detailed_sinks
                .iter()
                .map(|(net, _)| *net)
                .collect::<BTreeSet<_>>();
            let frozen = self.packing.global_routing_constraints_cached(
                self.design,
                self.architecture,
                &seed.0.placement,
                self.global_routing_cache,
            )?;
            routing_costs
                .set_net_criticalities(timing_net_weights(&seed.1, self.timing_constraints));
            routing_costs
                .set_sink_criticalities(timing_arc_weights(&seed.1, self.timing_constraints));
            routing_costs.set_sink_min_delays_ps(BTreeMap::new());
            routing_costs.set_detailed_timing_nets(detailed_nets);
            routing_costs.set_detailed_delay_quantum_ps(delay_quantum_ps);
            let trial = self.route_and_analyze(
                seed.0.placement.clone(),
                &frozen,
                Some(routing_costs),
                progress,
            );
            routing_costs.set_detailed_timing_nets(BTreeSet::new());
            let trial = match trial {
                Ok(trial) => trial,
                Err(Ecp5FlowError::Pnr(_)) => break,
                Err(error) => return Err(error),
            };
            let improves_objective = timing_score(&trial.1) > timing_score(&seed.1);
            progress(Ecp5FlowStage::TimingTrialDecision { improves_objective });
            if !improves_objective {
                self.stalled_ripup_wns_ps = Some(seed_wns_ps);
                break;
            }
            archive.push(trial);
            archive = select_timing_frontier(archive);
        }
        Ok(archive)
    }

    /// Deterministic basin escape for designs that stall with negative setup
    /// slack. Each round re-solves the analytical placement from the
    /// incumbent's per-sink criticality weights raised to a fixed power, which
    /// sharpens the contrast around critical paths and lands in a different
    /// placement basin without randomness. The kicked candidate is kept only
    /// when a full route-and-analysis beats the archive's best; the greedy
    /// refinement then descends again from the new basin.
    #[allow(clippy::too_many_lines)]
    fn escape_placement_basin(
        &mut self,
        mut archive: Vec<TimingCandidate>,
        placement_refiner: &PlacementRefiner<'_>,
        routing_costs: &mut RoutingCosts,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<Vec<TimingCandidate>, Ecp5FlowError> {
        for _ in 0..MAX_BASIN_ESCAPE_ROUNDS {
            let best_score = archive
                .iter()
                .map(|(_, timing)| timing_score(timing))
                .max()
                .expect("the timing archive is non-empty");
            if archive
                .iter()
                .any(|(_, timing)| timing.worst_slack_ps.is_some_and(|slack| slack >= 0))
            {
                break;
            }
            let seed = archive
                .iter()
                .max_by_key(|(implementation, timing)| {
                    (timing_score(timing), Reverse(implementation.total_pips))
                })
                .expect("the timing archive is non-empty")
                .clone();
            let weights = timing_placement_weights(&seed.1, self.timing_constraints);
            let amplified = weights
                .into_iter()
                .map(|(sink, weight)| (sink, weight.saturating_pow(BASIN_ESCAPE_WEIGHT_EXPONENT)))
                .collect::<BTreeMap<_, _>>();
            let kicked = place_analytically_with_net_sink_weights(
                self.design,
                self.architecture.device(),
                self.packing.constraints(),
                &amplified,
            )?;
            progress(Ecp5FlowStage::TimingDrivenPlaced);
            // Every kick measured so far routed far worse than the incumbent;
            // a decisive estimated regression skips the route entirely. The
            // valve stays open for kicks the estimate likes.
            if self.estimate_rejects(
                &seed.0.placement,
                &kicked,
                &timing_net_weights(&seed.1, self.timing_constraints),
            ) {
                break;
            }
            let routing = self.packing.global_routing_constraints_cached(
                self.design,
                self.architecture,
                &kicked,
                self.global_routing_cache,
            )?;
            progress(Ecp5FlowStage::TimingDrivenGlobalClocksRouted);
            routing_costs
                .set_net_criticalities(timing_net_weights(&seed.1, self.timing_constraints));
            routing_costs
                .set_sink_criticalities(timing_arc_weights(&seed.1, self.timing_constraints));
            routing_costs.set_sink_min_delays_ps(BTreeMap::new());
            let trial =
                match self.route_and_analyze(kicked, &routing, Some(routing_costs), progress) {
                    Ok(trial) => trial,
                    Err(Ecp5FlowError::Pnr(_)) => break,
                    Err(error) => return Err(error),
                };
            let improves_objective = timing_score(&trial.1) > best_score;
            progress(Ecp5FlowStage::TimingTrialDecision { improves_objective });
            if !improves_objective {
                break;
            }
            archive.push(trial);
            archive = select_timing_frontier(archive);
            archive = self.refine_setup_monotonically(
                archive,
                placement_refiner,
                routing_costs,
                progress,
            )?;
        }
        Ok(archive)
    }

    fn repair_hold_locally(
        &mut self,
        implementation: &PnrResult,
        timing: &TimingReport,
        routing_costs: &mut RoutingCosts,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<Option<TimingCandidate>, Ecp5FlowError> {
        let minimums = hold_sink_min_delays(timing);
        if minimums.is_empty() {
            return Ok(None);
        }
        let repair_sinks = minimums.keys().copied().collect::<BTreeSet<_>>();
        let frozen = freeze_route_sinks_except(
            self.design,
            self.architecture.device(),
            &implementation.placement,
            &implementation.routes,
            &repair_sinks,
        )?;
        routing_costs.set_net_criticalities(timing_net_weights(timing, self.timing_constraints));
        routing_costs.set_sink_criticalities(timing_arc_weights(timing, self.timing_constraints));
        routing_costs.set_sink_min_delays_ps(minimums);
        match self.route_local_trial_and_analyze(
            implementation.placement.clone(),
            &frozen,
            routing_costs,
            progress,
        ) {
            Ok(repaired) => Ok(Some(repaired)),
            Err(Ecp5FlowError::Pnr(_)) => Ok(None),
            Err(error) => Err(error),
        }
    }

    fn refine_candidate(
        &mut self,
        implementation: &PnrResult,
        timing: &TimingReport,
        placement_refiner: &PlacementRefiner<'_>,
        routing_costs: &mut RoutingCosts,
        max_refined_units: usize,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<Option<TimingCandidate>, Ecp5FlowError> {
        let refinement_weights = timing_placement_weights(timing, self.timing_constraints);
        let sink_budgets = placement_sink_budgets(
            self.design,
            self.architecture.device(),
            &implementation.placement,
            timing,
        );
        let refined_placement = placement_refiner.refine_with_net_sink_weights_limited(
            implementation.placement.clone(),
            &refinement_weights,
            Some(&sink_budgets),
            max_refined_units,
        )?;
        progress(Ecp5FlowStage::TimingDrivenPlaced);
        if self.estimate_rejects(
            &implementation.placement,
            &refined_placement,
            &timing_net_weights(timing, self.timing_constraints),
        ) {
            return Ok(None);
        }
        let refined_routing = self.packing.global_routing_constraints_cached(
            self.design,
            self.architecture,
            &refined_placement,
            self.global_routing_cache,
        )?;
        progress(Ecp5FlowStage::TimingDrivenGlobalClocksRouted);
        let released = released_timing_sinks(timing, self.timing_constraints);
        let incremental_routing = freeze_unchanged_routes(
            self.design,
            implementation,
            &refined_placement,
            &refined_routing,
            &released,
        );
        routing_costs.set_net_criticalities(timing_net_weights(timing, self.timing_constraints));
        routing_costs.set_sink_criticalities(timing_arc_weights(timing, self.timing_constraints));
        routing_costs.set_sink_min_delays_ps(BTreeMap::new());
        match self.route_local_trial_and_analyze(
            refined_placement,
            &incremental_routing,
            routing_costs,
            progress,
        ) {
            Ok(candidate) => Ok(Some(candidate)),
            Err(Ecp5FlowError::Pnr(_)) => Ok(None),
            Err(error) => Err(error),
        }
    }

    fn refine_local_connections(
        &mut self,
        implementation: &PnrResult,
        timing: &TimingReport,
        placement_refiner: &PlacementRefiner<'_>,
        routing_costs: &mut RoutingCosts,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<Vec<TimingCandidate>, Ecp5FlowError> {
        let Some(worst_slack) = timing.worst_slack_ps else {
            return Ok(Vec::new());
        };
        let delays = timing
            .net_delays
            .iter()
            .map(|edge| ((edge.net, edge.sink), edge.delay.max_ps))
            .collect::<BTreeMap<_, _>>();
        let mut edges = timing
            .net_setup_slacks
            .iter()
            .filter(|edge| edge.slack_ps == worst_slack)
            .map(|edge| {
                (
                    Reverse(delays.get(&(edge.net, edge.sink)).copied().unwrap_or(0)),
                    edge.net,
                    edge.sink,
                )
            })
            .collect::<Vec<_>>();
        edges.sort_unstable();
        edges.truncate(MAX_LOCAL_CONNECTION_CANDIDATES);
        let prescreen_weights = timing_net_weights(timing, self.timing_constraints);
        let mut candidates = Vec::new();
        for (_, net, sink) in edges {
            let driver = self.design.nets()[net.0].driver;
            let Some(placement) = placement_refiner.refine_connection_delay(
                implementation.placement.clone(),
                driver,
                sink,
                false,
                routing_costs.pip_delays_ps(),
                1,
            )?
            else {
                continue;
            };
            progress(Ecp5FlowStage::TimingDrivenPlaced);
            if self.estimate_rejects(&implementation.placement, &placement, &prescreen_weights) {
                continue;
            }
            let base = self.packing.global_routing_constraints_cached(
                self.design,
                self.architecture,
                &placement,
                self.global_routing_cache,
            )?;
            progress(Ecp5FlowStage::TimingDrivenGlobalClocksRouted);
            let released = BTreeSet::from([(net, sink)]);
            let frozen =
                freeze_unchanged_routes(self.design, implementation, &placement, &base, &released);
            routing_costs
                .set_net_criticalities(timing_net_weights(timing, self.timing_constraints));
            routing_costs
                .set_sink_criticalities(timing_arc_weights(timing, self.timing_constraints));
            routing_costs.set_sink_min_delays_ps(BTreeMap::new());
            if let Ok(candidate) =
                self.route_local_trial_and_analyze(placement, &frozen, routing_costs, progress)
            {
                progress(Ecp5FlowStage::TimingTrialDecision {
                    improves_objective: timing_score(&candidate.1) > timing_score(timing),
                });
                candidates.push(candidate);
            }
        }
        Ok(candidates)
    }

    #[allow(clippy::too_many_lines)]
    fn refine_critical_path_cells(
        &mut self,
        implementation: &PnrResult,
        timing: &TimingReport,
        placement_refiner: &PlacementRefiner<'_>,
        routing_costs: &mut RoutingCosts,
        max_move_distance: u64,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<Vec<TimingCandidate>, Ecp5FlowError> {
        let profile_started = Instant::now();
        let mut proposal_time = Duration::ZERO;
        let mut prescreen_time = Duration::ZERO;
        let mut constraint_time = Duration::ZERO;
        let mut routed_trials = 0_usize;
        let profile = metrics_enabled();
        let mut best_trial = None::<(TimingScore, CellId, usize)>;
        let seed_fingerprint = implementation_topology_fingerprint(self.design, implementation);
        let Some(worst_slack_ps) = timing.worst_slack_ps else {
            return Ok(Vec::new());
        };
        let worst_connections = worst_setup_connections(self.design, timing, worst_slack_ps);
        let connection_delays = timing
            .net_delays
            .iter()
            .map(|delay| ((delay.net, delay.sink), delay.delay.max_ps))
            .collect::<BTreeMap<_, _>>();
        // Per-connection delay targets mirror the routed-delay budgets: a
        // failing connection must shed its whole slack deficit, but never more
        // than half of its realized delay, so vertex moves score proposals by
        // the budget they still exceed rather than raw delay alone.
        let deficit_ps = u64::try_from(worst_slack_ps.unsigned_abs()).unwrap_or(u64::MAX);
        let mut by_cell = BTreeMap::<CellId, (u64, Vec<(CellPinId, CellPinId)>, Vec<u64>)>::new();
        for &(net, driver, sink) in &worst_connections {
            let driver_cell = self.design.pins()[driver.0].cell;
            let sink_cell = self.design.pins()[sink.0].cell;
            let delay = connection_delays.get(&(net, sink)).copied().unwrap_or(0);
            let target = delay.saturating_sub(deficit_ps).max(delay / 2).max(1);
            let driver_entry = by_cell.entry(driver_cell).or_default();
            driver_entry.0 = driver_entry.0.saturating_add(delay);
            driver_entry.1.push((driver, sink));
            driver_entry.2.push(target);
            let sink_entry = by_cell.entry(sink_cell).or_default();
            sink_entry.0 = sink_entry.0.saturating_add(delay);
            sink_entry.1.push((driver, sink));
            sink_entry.2.push(target);
        }
        // Endpoint cells with a single critical connection must move too:
        // excluding them froze the driving FF of the worst carry-cluster feed
        // and the sink FF behind it in place, leaving their general-routing
        // hops permanently over budget.
        let mut cells = by_cell.into_iter().collect::<Vec<_>>();
        cells.sort_unstable_by_key(|(cell, (delay, _, _))| (Reverse(*delay), *cell));
        let critical_sinks = worst_connections
            .iter()
            .map(|(net, _, sink)| (*net, *sink))
            .collect::<BTreeSet<_>>();
        let prescreen_weights = timing_net_weights(timing, self.timing_constraints);
        let route_arc_weights = timing_arc_weights(timing, self.timing_constraints);
        routing_costs.set_net_criticalities(prescreen_weights.clone());
        routing_costs.set_sink_criticalities(route_arc_weights.clone());
        let capacity_projection = (max_move_distance > 2)
            .then(|| RouteCapacityProjection::new(&implementation.routes, routing_costs));
        let incumbent_global_routing =
            global_routes_from_implementation(self.packing, implementation);
        let mut placement = implementation.placement.clone();
        let mut candidates = Vec::new();
        for (cell, (_, connections, targets)) in cells.into_iter().take(MAX_CRITICAL_PATH_CELLS) {
            let proposal_started = Instant::now();
            let proposals = placement_refiner.refine_cell_connection_delays(
                placement.clone(),
                cell,
                &connections,
                &targets,
                routing_costs.pip_delays_ps(),
                capacity_projection.as_ref(),
                max_move_distance,
                if max_move_distance > 2 {
                    MAX_PROJECTED_PATH_CELL_CANDIDATES
                } else {
                    1
                },
            )?;
            // The topology projection ranks the failing path itself. Before
            // paying for negotiated routing, score its small shortlist by
            // every moved net as a second, independent objective. Route the
            // Pareto frontier in projection order: the topology winner is
            // always retained, while a deeper alternative survives only when
            // it establishes a lower collateral timing estimate.
            let mut proposals = proposals
                .into_iter()
                .enumerate()
                .map(|(projected_rank, proposal)| {
                    let nets = self
                        .moved_nets(&placement, &proposal)
                        .collect::<BTreeSet<_>>();
                    let estimate = self
                        .weighted_net_estimate(&proposal, &nets, &prescreen_weights)
                        .unwrap_or(u64::MAX);
                    (estimate, projected_rank, proposal)
                })
                .collect::<Vec<_>>();
            if max_move_distance > 2 {
                retain_projection_timing_frontier(&mut proposals);
            }
            let proposals = proposals
                .into_iter()
                .map(|(_, projected_rank, proposal)| (projected_rank, proposal))
                .collect::<Vec<_>>();
            proposal_time += proposal_started.elapsed();
            if proposals.is_empty() {
                continue;
            }
            let from = placement.bindings()[cell.0];
            let mut best_for_cell = None;
            for (proposal_rank, refined) in proposals {
                let prescreen_started = Instant::now();
                if self.estimate_rejects(&placement, &refined, &prescreen_weights) {
                    prescreen_time += prescreen_started.elapsed();
                    if profile {
                        eprintln!(
                            "[metrics] critical_trial distance={max_move_distance} cell={} rank={} rejected=prescreen",
                            cell.0,
                            proposal_rank + 1,
                        );
                    }
                    continue;
                }
                let trial_key = (
                    seed_fingerprint,
                    placement_fingerprint(self.design, &refined),
                );
                if !self.critical_move_trials.insert(trial_key) {
                    prescreen_time += prescreen_started.elapsed();
                    if profile {
                        eprintln!(
                            "[metrics] critical_trial distance={max_move_distance} cell={} rank={} rejected=duplicate",
                            cell.0,
                            proposal_rank + 1,
                        );
                    }
                    continue;
                }
                prescreen_time += prescreen_started.elapsed();
                let to = refined.bindings()[cell.0];
                progress(Ecp5FlowStage::CriticalPathMove { cell, from, to });
                let constraint_started = Instant::now();
                let recomputed_global_routing;
                let base = if let Some(incumbent) = incumbent_global_routing.as_ref()
                    && global_clock_endpoints_unchanged(
                        self.design,
                        self.packing,
                        &implementation.placement,
                        &refined,
                    ) {
                    incumbent
                } else {
                    recomputed_global_routing = self.packing.global_routing_constraints_cached(
                        self.design,
                        self.architecture,
                        &refined,
                        self.global_routing_cache,
                    )?;
                    &recomputed_global_routing
                };
                progress(Ecp5FlowStage::TimingDrivenGlobalClocksRouted);
                let frozen = freeze_unchanged_routes(
                    self.design,
                    implementation,
                    &refined,
                    base,
                    &critical_sinks,
                );
                constraint_time += constraint_started.elapsed();
                routing_costs.set_net_criticalities(prescreen_weights.clone());
                routing_costs.set_sink_criticalities(route_arc_weights.clone());
                routing_costs.set_sink_min_delays_ps(BTreeMap::new());
                routing_costs.set_detailed_timing_nets(released_net_ids(&critical_sinks));
                // Measured: the finest quantum left WNS and hold untouched while slowing
                // small-trial routing by ~27%, so detailed searches here run at the
                // default granularity and only the multiresolution ripup keeps a
                // fine quantum.
                routing_costs.set_detailed_delay_quantum_ps(texo_pnr::ROUTING_DELAY_QUANTUM_PS);
                let trial =
                    self.route_local_trial_and_analyze(refined, &frozen, routing_costs, progress);
                routed_trials += 1;
                routing_costs.set_detailed_timing_nets(BTreeSet::new());
                if let Ok(candidate) = trial {
                    let score = timing_score(&candidate.1);
                    let improves_objective = score > timing_score(timing);
                    let closes_setup = candidate
                        .1
                        .worst_slack_ps
                        .is_some_and(|slack_ps| slack_ps >= 0);
                    if best_trial
                        .as_ref()
                        .is_none_or(|(best_score, _, _)| score > *best_score)
                    {
                        best_trial = Some((score, cell, proposal_rank));
                    }
                    if profile {
                        eprintln!(
                            "[metrics] critical_trial distance={max_move_distance} cell={} rank={} wns={:?} tns={} improves={improves_objective}",
                            cell.0,
                            proposal_rank + 1,
                            candidate.1.worst_slack_ps,
                            slack_violations(
                                candidate.1.setup_checks.iter().map(|check| check.slack_ps)
                            )
                            .total_negative_slack_ps(),
                        );
                    }
                    progress(Ecp5FlowStage::TimingTrialDecision { improves_objective });
                    if best_for_cell.as_ref().is_none_or(|(_, best_timing)| {
                        timing_score(&candidate.1) > timing_score(best_timing)
                    }) {
                        best_for_cell = Some(candidate.clone());
                    }
                    candidates.push(candidate);
                    if closes_setup {
                        // Detailed 10 ps and 1 ps rerouting still runs after
                        // this vertex pass. Do not spend full routing trials
                        // evaluating alternative placements once setup is
                        // already feasible.
                        return Ok(candidates);
                    }
                } else if profile {
                    eprintln!(
                        "[metrics] critical_trial distance={max_move_distance} cell={} rank={} rejected=routing",
                        cell.0,
                        proposal_rank + 1,
                    );
                }
            }
            if let Some((best_implementation, best_timing)) = best_for_cell
                && timing_score(&best_timing) > timing_score(timing)
            {
                placement = best_implementation.placement;
            }
        }
        if profile {
            let winner = best_trial.map_or_else(
                || "none".to_owned(),
                |(_, cell, proposal_rank)| format!("{}:{}", cell.0, proposal_rank + 1),
            );
            eprintln!(
                "[metrics] critical_cells distance={max_move_distance} total={:?} proposals={proposal_time:?} prescreen={prescreen_time:?} constraints={constraint_time:?} trials={routed_trials} winner={winner}",
                profile_started.elapsed(),
            );
        }
        Ok(candidates)
    }

    fn route_and_analyze(
        &mut self,
        placement: Placement,
        routing: &RoutingConstraints,
        costs: Option<&RoutingCosts>,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<(PnrResult, TimingReport), Ecp5FlowError> {
        let profile = metrics_enabled();
        let started = Instant::now();
        let implementation = if let Some(costs) = costs {
            route_with_timing_costs_workspace_and_progress(
                self.design,
                self.architecture.device(),
                placement,
                routing,
                costs,
                self.routing_workspace,
                |event| progress(Ecp5FlowStage::TimingDrivenRouting(event)),
            )?
        } else {
            route_with_workspace_and_progress(
                self.design,
                self.architecture.device(),
                placement,
                routing,
                self.routing_workspace,
                |event| progress(Ecp5FlowStage::TimingDrivenRouting(event)),
            )?
        };
        let routed = started.elapsed();
        progress(Ecp5FlowStage::TimingDrivenRouted);
        let analysis_started = Instant::now();
        let timing = analyze_ecp5_implementation(
            self.design,
            self.architecture,
            self.speed_grade,
            &implementation,
            self.timing_model,
            self.timing_constraints,
        )?;
        if profile {
            eprintln!(
                "[metrics] route_and_analyze route={routed:?} analyze={:?} pips={}",
                analysis_started.elapsed(),
                implementation.total_pips
            );
        }
        progress(timing_snapshot(&timing));
        Ok((implementation, timing))
    }

    fn route_local_trial_and_analyze(
        &mut self,
        placement: Placement,
        routing: &RoutingConstraints,
        costs: &RoutingCosts,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<(PnrResult, TimingReport), Ecp5FlowError> {
        let mut bounded = costs.clone();
        bounded.set_max_iterations(LOCAL_TRIAL_ROUTING_ITERATIONS);
        self.route_and_analyze(placement, routing, Some(&bounded), progress)
    }
}

fn emit_archive_metric(
    stage: &str,
    context: &TimingDrivenContext<'_, '_>,
    archive: &[TimingCandidate],
) {
    if !metrics_enabled() {
        return;
    }
    if let Some((implementation, timing)) = archive.iter().max_by_key(|(implementation, timing)| {
        (timing_score(timing), Reverse(implementation.total_pips))
    }) {
        eprintln!(
            "[metrics] closure_stage={stage} wns={:?} tns={} whs={:?} pips={} hpwl={}",
            timing.worst_slack_ps,
            slack_violations(timing.setup_checks.iter().map(|check| check.slack_ps))
                .total_negative_slack_ps(),
            timing.worst_hold_slack_ps,
            implementation.total_pips,
            placement_hpwl(
                context.design,
                context.architecture.device(),
                &implementation.placement,
            ),
        );
    }
}

fn retain_projection_timing_frontier<T>(candidates: &mut Vec<(u64, usize, T)>) {
    let mut best_estimate = u64::MAX;
    candidates.retain(|(estimate, projected_rank, _)| {
        let keep = *projected_rank == 0 || *estimate < best_estimate;
        best_estimate = best_estimate.min(*estimate);
        keep
    });
}

fn worst_setup_connections(
    design: &Design,
    timing: &TimingReport,
    worst_slack_ps: i128,
) -> Vec<(NetId, CellPinId, CellPinId)> {
    timing
        .net_setup_slacks
        .iter()
        .filter(|edge| edge.slack_ps == worst_slack_ps)
        .map(|edge| (edge.net, design.nets()[edge.net.0].driver, edge.sink))
        .collect()
}

const MAX_INCREMENTAL_REFINEMENTS: usize = 8;
const LOCAL_TRIAL_ROUTING_ITERATIONS: u32 = 5;
const MAX_DEDICATED_EDGE_TRIALS: usize = 4;
// Geometry delay model for pre-screening route trials. Calibrated loosely
// against the measured AXI4 hops: same-tile LUT-to-FF edges land near 300 ps,
// multi-tile general-routing hops near 250 ps per tile. A proposal whose
// criticality-weighted incident estimate does not improve is rejected without
// routing, which removes most of the ~45% of trials that the real gate
// rejects anyway.
const PRESCREEN_PS_PER_TILE_PS: u64 = 250;
const PRESCREEN_HOP_OVERHEAD_PS: u64 = 300;
const MAX_LOCAL_CONNECTION_REFINEMENTS: usize = 4;
const MAX_LOCAL_CONNECTION_CANDIDATES: usize = 8;
const TIMING_FRONTIER_WIDTH: usize = 1;
const REFINED_PLACEMENT_UNIT_LIMITS: [usize; 4] = [256, 128, 64, 32];
const DETAILED_ROUTING_QUANTA_PS: [u64; 1] = [10];
// A failed full-chip negotiation is retried only after placement has improved
// setup by approximately one measured general-routing tile. This is a basin
// change, unlike the small local moves between adjacent closure rounds.
const GLOBAL_RIPUP_REARM_SLACK_PS: i128 = 250;
// The second 1 ps quantum measured as a pure extra full renegotiation on the
// AXI4 self-test: final WNS and placement were bit-identical without it while
// each multiresolution round paid one more ~30 s global ripup.
const MAX_CRITICAL_PATH_CELLS: usize = 6;
const MAX_PROJECTED_PATH_CELL_CANDIDATES: usize = 4;
const MAX_CRITICAL_CLOSURE_ROUNDS: usize = 4;
const MAX_CRITICAL_PATH_VERTEX_REFINEMENTS: usize = 4;
// Basin-escape budget for designs that stall with negative setup slack after
// every refinement phase. Kicks re-solve the analytical placement with the
// incumbent's criticality weights amplified by a fixed power, which lands in a
// different deterministic basin; no randomness or recorded seed is involved.
const MAX_BASIN_ESCAPE_ROUNDS: usize = 2;
const BASIN_ESCAPE_WEIGHT_EXPONENT: u32 = 4;
// Start with cheap local legalization, then let only an internal vertex of the
// actual worst path escape a bad placement basin.  The broad pass is still a
// deterministic exhaustive choice over that one unit's legal BEL assignments;
// it is not a random restart or a whole-design perturbation.
const CRITICAL_PATH_MOVE_DISTANCES: [u64; 3] = [1, 2, 16];
const MAX_RELEASED_CRITICAL_NETS: usize = 64;

fn should_retry_global_ripup(last_failed_wns_ps: Option<i128>, seed_wns_ps: i128) -> bool {
    last_failed_wns_ps.is_none_or(|failed_wns_ps| {
        seed_wns_ps >= failed_wns_ps.saturating_add(GLOBAL_RIPUP_REARM_SLACK_PS)
    })
}

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

/// Total driver-to-sink Manhattan distance over every net edge.
fn placement_hpwl(design: &Design, device: &Device, placement: &Placement) -> u64 {
    let mut total = 0_u64;
    for net in design.nets() {
        let driver_cell = design.pins()[net.driver.0].cell;
        let Some(driver_point) = placement.point(driver_cell, device) else {
            continue;
        };
        for &sink in &net.sinks {
            let sink_cell = design.pins()[sink.0].cell;
            if let Some(sink_point) = placement.point(sink_cell, device) {
                total = total.saturating_add(driver_point.manhattan(sink_point));
            }
        }
    }
    total
}

fn metrics_enabled() -> bool {
    std::env::var_os("TEXO_METRICS").is_some()
}

fn report_metric_phase(name: &str, started: &mut Instant) {
    if metrics_enabled() {
        eprintln!(
            "[metrics] flow_phase={name} elapsed={:?}",
            started.elapsed()
        );
    }
    *started = Instant::now();
}

/// Emits one stage-wise placement quality line when `TEXO_METRICS` is set.
///
/// This is a measurement hook for initial-solution studies, not part of the
/// release contract.
fn emit_placement_metric(
    stage: &str,
    design: &Design,
    device: &Device,
    placement: &Placement,
    timing: Option<&TimingReport>,
) {
    if !metrics_enabled() {
        return;
    }
    let hpwl = placement_hpwl(design, device, placement);
    match timing {
        Some(timing) => eprintln!(
            "[metrics] stage={stage} hpwl={hpwl} wns={:?} hold={:?} pips={}",
            timing.worst_slack_ps, timing.worst_hold_slack_ps, 0,
        ),
        None => eprintln!("[metrics] stage={stage} hpwl={hpwl}"),
    }
}

/// Derives per-connection free-distance allowances from routed delays and
/// setup slack.
///
/// Each connection's allowance is the Manhattan distance that still meets its
/// delay target at the realized picoseconds-per-tile rate, so the refinement
/// objective charges only connections exceeding their share of the period:
/// satisfied connections behave like slack rubber and violated ones like
/// springs. Per-endpoint slacks attribute a whole failing path's deficit to
/// every connection on it, so the tightening is capped at half the current
/// delay.
fn placement_sink_budgets(
    design: &Design,
    device: &Device,
    placement: &Placement,
    timing: &TimingReport,
) -> BTreeMap<(NetId, CellPinId), u32> {
    let slacks = timing
        .net_setup_slacks
        .iter()
        .map(|edge| ((edge.net, edge.sink), edge.slack_ps))
        .collect::<BTreeMap<_, _>>();
    let mut budgets = BTreeMap::new();
    for edge in &timing.net_delays {
        let driver_cell = design.pins()[design.nets()[edge.net.0].driver.0].cell;
        let Some(driver_point) = placement.point(driver_cell, device) else {
            continue;
        };
        let Some(sink_point) = placement.point(design.pins()[edge.sink.0].cell, device) else {
            continue;
        };
        let distance = driver_point.manhattan(sink_point).max(1);
        let delay = edge.delay.max_ps.max(1);
        let slack = slacks.get(&(edge.net, edge.sink)).copied().unwrap_or(0);
        let target_delay = if slack < 0 {
            delay
                .saturating_sub(u64::try_from(slack.unsigned_abs()).unwrap_or(u64::MAX))
                .max(delay / 2)
                .max(1)
        } else {
            delay
        };
        let allowance = distance.saturating_mul(target_delay) / delay;
        budgets.insert(
            (edge.net, edge.sink),
            u32::try_from(allowance).unwrap_or(u32::MAX),
        );
    }
    budgets
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

fn timing_arc_weights(
    timing: &TimingReport,
    constraints: &TimingConstraints,
) -> BTreeMap<(NetId, CellPinId), u64> {
    let Some(period_ps) = constraints.clock_periods_ps().values().copied().min() else {
        return BTreeMap::new();
    };
    let Some(worst_slack_ps) = timing.worst_slack_ps else {
        return BTreeMap::new();
    };
    let period_ps = i128::from(period_ps.max(1));
    let critical_limit = worst_slack_ps + period_ps;
    let mut weights = BTreeMap::<(NetId, CellPinId), u64>::new();
    for edge in &timing.net_setup_slacks {
        let urgency = (critical_limit - edge.slack_ps).clamp(0, period_ps);
        let weight = criticality_weight(urgency, period_ps);
        weights
            .entry((edge.net, edge.sink))
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

fn freeze_route_sinks_except(
    design: &Design,
    device: &Device,
    placement: &Placement,
    routes: &[Arc<NetRoute>],
    released: &BTreeSet<(NetId, CellPinId)>,
) -> Result<RoutingConstraints, PnrError> {
    let mut frozen = RoutingConstraints::new();
    for route in routes {
        let net = &design.nets()[route.net.0];
        let retained = net
            .sinks
            .iter()
            .copied()
            .filter(|sink| !released.contains(&(route.net, *sink)))
            .collect::<BTreeSet<_>>();
        if retained.len() == net.sinks.len() {
            frozen.add_route(route.clone());
        } else if let Some(partial) =
            retain_route_for_sinks(design, device, placement, route, &retained)?
        {
            frozen.add_route(partial);
        }
    }
    Ok(frozen)
}

fn released_timing_sinks(
    timing: &TimingReport,
    constraints: &TimingConstraints,
) -> BTreeSet<(NetId, CellPinId)> {
    let mut ranked = timing_net_weights(timing, constraints)
        .into_iter()
        .filter_map(|(net, weight)| (weight > 1).then_some((Reverse(weight), net)))
        .collect::<Vec<_>>();
    ranked.sort_unstable();
    ranked
        .into_iter()
        .take(MAX_RELEASED_CRITICAL_NETS)
        .flat_map(|(_, net)| {
            timing
                .net_setup_slacks
                .iter()
                .filter_map(move |edge| (edge.net == net).then_some((net, edge.sink)))
        })
        .collect()
}

fn released_net_ids(released: &BTreeSet<(NetId, CellPinId)>) -> BTreeSet<NetId> {
    released.iter().map(|(net, _)| *net).collect()
}

fn freeze_unchanged_routes(
    design: &Design,
    implementation: &PnrResult,
    placement: &Placement,
    base: &RoutingConstraints,
    released: &BTreeSet<(NetId, CellPinId)>,
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
        if base.routes().contains_key(&route.net) {
            continue;
        }
        let net = &design.nets()[route.net.0];
        if moved.contains(&design.pins()[net.driver.0].cell) {
            continue;
        }
        let mut released_sinks = route
            .arcs
            .iter()
            .filter_map(|arc| {
                arc.sink.filter(|&sink| {
                    released.contains(&(route.net, sink))
                        || moved.contains(&design.pins()[sink.0].cell)
                })
            })
            .collect::<BTreeSet<_>>();
        loop {
            let released_pips = route
                .arcs
                .iter()
                .filter(|arc| arc.sink.is_some_and(|sink| released_sinks.contains(&sink)))
                .flat_map(|arc| arc.pips.iter().copied())
                .collect::<BTreeSet<_>>();
            let coupled = route
                .arcs
                .iter()
                .filter_map(|arc| {
                    arc.sink.filter(|sink| {
                        !released_sinks.contains(sink)
                            && arc.pips.iter().any(|pip| released_pips.contains(pip))
                    })
                })
                .collect::<Vec<_>>();
            let before = released_sinks.len();
            released_sinks.extend(coupled);
            if released_sinks.len() == before {
                break;
            }
        }
        let retained = route
            .arcs
            .iter()
            .filter(|arc| arc.sink.is_none_or(|sink| !released_sinks.contains(&sink)))
            .cloned()
            .collect::<Vec<_>>();
        if !retained.is_empty() {
            frozen.add_route(NetRoute::new(route.net, retained));
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

type ViolationScore = (Reverse<u128>, Reverse<u128>, Reverse<u128>, Reverse<usize>);
type TimingScore = (bool, ViolationScore, i128, i128);

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct SlackViolations {
    maximum_deficit_ps: u128,
    squared_penalty_ps2: u128,
    total_deficit_ps: u128,
    endpoints: usize,
}

impl SlackViolations {
    fn score(self) -> ViolationScore {
        let worst_penalty = self
            .maximum_deficit_ps
            .saturating_mul(self.maximum_deficit_ps);
        let objective_penalty = self.squared_penalty_ps2.saturating_add(worst_penalty);
        (
            Reverse(objective_penalty),
            Reverse(self.maximum_deficit_ps),
            Reverse(self.total_deficit_ps),
            Reverse(self.endpoints),
        )
    }

    fn total_negative_slack_ps(self) -> i128 {
        -i128::try_from(self.total_deficit_ps).unwrap_or(i128::MAX)
    }
}

/// Solves the connectivity-only analytical placement that starts the flow.
///
/// Measured: adding static moderate weights on nets touching packed carry
/// slices pulled feeding logic toward the clusters globally but landed the
/// whole design in a worse basin (WNS −509 vs −287 ps), matching the earlier
/// finding that wirelength-dominant solves win at 1.6% utilization.
fn initial_analytical_placement(
    design: &Design,
    architecture: &Ecp5Architecture,
    placement_refiner: &PlacementRefiner<'_>,
) -> Result<Placement, Ecp5FlowError> {
    let placement = placement_refiner.place_analytically(&BTreeMap::new())?;
    emit_placement_metric(
        "initial_place",
        design,
        architecture.device(),
        &placement,
        None,
    );
    Ok(placement)
}

fn named_initial_placement(
    design: &Design,
    architecture: &Ecp5Architecture,
    packing: &Ecp5Packing,
    named_bindings: &BTreeMap<String, String>,
) -> Result<Placement, Ecp5FlowError> {
    let cells = design
        .cells()
        .iter()
        .enumerate()
        .map(|(index, cell)| (cell.name.as_str(), CellId(index)))
        .collect::<BTreeMap<_, _>>();
    let bels = architecture
        .device()
        .bels()
        .iter()
        .enumerate()
        .map(|(index, bel)| (bel.name.as_str(), BelId(index)))
        .collect::<BTreeMap<_, _>>();
    let mut bindings = BTreeMap::new();
    for (cell_name, bel_name) in named_bindings {
        let cell =
            cells
                .get(cell_name.as_str())
                .copied()
                .ok_or_else(|| PnrError::InvalidPlacement {
                    reason: format!("binding names unknown cell `{cell_name}`"),
                })?;
        let bel =
            bels.get(bel_name.as_str())
                .copied()
                .ok_or_else(|| PnrError::InvalidPlacement {
                    reason: format!("binding names unknown BEL `{bel_name}`"),
                })?;
        bindings.insert(cell, bel);
    }
    let placement = placement_from_partial_bindings(
        design,
        architecture.device(),
        packing.constraints(),
        &bindings,
    )?;
    emit_placement_metric(
        "external_initial_place",
        design,
        architecture.device(),
        &placement,
        None,
    );
    Ok(placement)
}

fn named_lut_ff_pairs(
    design: &Design,
    named_pairs: &BTreeMap<String, String>,
) -> Result<Vec<LutFfPair>, PnrError> {
    let cells = design
        .cells()
        .iter()
        .enumerate()
        .map(|(index, cell)| (cell.name.as_str(), CellId(index)))
        .collect::<BTreeMap<_, _>>();
    named_pairs
        .iter()
        .map(|(lut_name, ff_name)| {
            let lut = cells.get(lut_name.as_str()).copied().ok_or_else(|| {
                PnrError::InvalidPlacement {
                    reason: format!("LUT/FF packing names unknown LUT `{lut_name}`"),
                }
            })?;
            let ff =
                cells
                    .get(ff_name.as_str())
                    .copied()
                    .ok_or_else(|| PnrError::InvalidPlacement {
                        reason: format!("LUT/FF packing names unknown FF `{ff_name}`"),
                    })?;
            Ok(LutFfPair { lut, ff })
        })
        .collect()
}

fn timing_snapshot(timing: &TimingReport) -> Ecp5FlowStage {
    let setup = slack_violations(timing.setup_checks.iter().map(|check| check.slack_ps));
    let hold = slack_violations(timing.hold_checks.iter().map(|check| check.slack_ps));
    Ecp5FlowStage::TimingSnapshot {
        worst_setup_ps: timing.worst_slack_ps,
        setup_tns_ps: setup.total_negative_slack_ps(),
        setup_violations: setup.endpoints,
        worst_hold_ps: timing.worst_hold_slack_ps,
        hold_ths_ps: hold.total_negative_slack_ps(),
        hold_violations: hold.endpoints,
    }
}

fn timing_score(timing: &TimingReport) -> TimingScore {
    let setup_score =
        slack_violations(timing.setup_checks.iter().map(|check| check.slack_ps)).score();
    let hold_score =
        slack_violations(timing.hold_checks.iter().map(|check| check.slack_ps)).score();
    let setup_slack = timing.worst_slack_ps.unwrap_or(i128::MIN);
    let hold_slack = timing.worst_hold_slack_ps.unwrap_or(i128::MIN);
    staged_timing_score(setup_score, hold_score, setup_slack, hold_slack)
}

fn staged_timing_score(
    setup_score: ViolationScore,
    hold_score: ViolationScore,
    setup_slack: i128,
    hold_slack: i128,
) -> TimingScore {
    if setup_slack < 0 {
        (false, setup_score, setup_slack, hold_slack)
    } else {
        (true, hold_score, setup_slack, hold_slack)
    }
}

fn slack_violations(slacks: impl Iterator<Item = i128>) -> SlackViolations {
    let mut violations = SlackViolations::default();
    for slack in slacks {
        if slack >= 0 {
            continue;
        }
        let deficit = slack.unsigned_abs();
        violations.squared_penalty_ps2 = violations
            .squared_penalty_ps2
            .saturating_add(deficit.saturating_mul(deficit));
        violations.maximum_deficit_ps = violations.maximum_deficit_ps.max(deficit);
        violations.total_deficit_ps = violations.total_deficit_ps.saturating_add(deficit);
        violations.endpoints += 1;
    }
    violations
}

type TimingCandidate = (PnrResult, TimingReport);

fn placement_fingerprint(design: &Design, placement: &Placement) -> u64 {
    let mut fingerprint = DefaultHasher::new();
    placement.bindings().hash(&mut fingerprint);
    for pin in 0..design.pins().len() {
        placement.pin_binding(CellPinId(pin)).hash(&mut fingerprint);
    }
    fingerprint.finish()
}

fn implementation_topology_fingerprint(design: &Design, implementation: &PnrResult) -> u64 {
    let mut fingerprint = DefaultHasher::new();
    placement_fingerprint(design, &implementation.placement).hash(&mut fingerprint);
    for route in &implementation.routes {
        route.net.hash(&mut fingerprint);
        for arc in &route.arcs {
            arc.sink.hash(&mut fingerprint);
            arc.wires.hash(&mut fingerprint);
            arc.pips.hash(&mut fingerprint);
        }
    }
    fingerprint.finish()
}

fn global_routes_from_implementation(
    packing: &Ecp5Packing,
    implementation: &PnrResult,
) -> Option<RoutingConstraints> {
    let mut constraints = RoutingConstraints::new();
    for clock in packing.global_clocks() {
        let route = implementation
            .routes
            .iter()
            .find(|route| route.net == clock.global_net)?;
        constraints.add_route(route.clone());
    }
    Some(constraints)
}

fn global_clock_endpoints_unchanged(
    design: &Design,
    packing: &Ecp5Packing,
    old: &Placement,
    new: &Placement,
) -> bool {
    packing.global_clocks().iter().all(|clock| {
        let net = &design.nets()[clock.global_net.0];
        std::iter::once(net.driver)
            .chain(net.sinks.iter().copied())
            .all(|pin| {
                let cell = design.pins()[pin.0].cell;
                old.bel(cell) == new.bel(cell) && old.pin_binding(pin) == new.pin_binding(pin)
            })
    })
}

fn select_timing_frontier(candidates: Vec<TimingCandidate>) -> Vec<TimingCandidate> {
    select_timing_candidates(candidates, TIMING_FRONTIER_WIDTH)
}

fn select_timing_candidates(
    mut candidates: Vec<TimingCandidate>,
    width: usize,
) -> Vec<TimingCandidate> {
    candidates.sort_by(
        |(left_implementation, left_timing), (right_implementation, right_timing)| {
            (
                timing_score(right_timing),
                Reverse(right_implementation.total_pips),
            )
                .cmp(&(
                    timing_score(left_timing),
                    Reverse(left_implementation.total_pips),
                ))
        },
    );
    candidates.truncate(width);
    candidates
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
        .flat_map(|route| route.pips())
        .collect::<BTreeSet<_>>();
    let mut source_fanout = BTreeMap::new();
    for &pip_id in &selected {
        let pip = &device.pips()[pip_id.0];
        *source_fanout.entry(pip.from()).or_insert(0_u64) += 1;
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
            let fanout = source_fanout[&device.pips()[pip_id.0].from()];
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
            // ECP5 LSR is an asynchronous set/reset input. Its characterized
            // recovery/removal values must not become synchronous data
            // setup/hold checks or constrain the register-to-register Fmax.
            if check.signal_pin == "LSR" {
                continue;
            }
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
    use std::collections::BTreeSet;
    use std::num::NonZeroU32;

    use struo_celox::ecp5_simulator;
    use struo_ir::{ActiveLevel, ArithmeticOp, ClockEdge, Netlist, RegisterCell, ResetControl};
    use struo_target_ecp5::{
        ArithmeticMapping, Ecp5Netlist, MappingOptions, map_to_ecp5, map_to_ecp5_with_options,
    };
    use texo_model::{BelId, CellId, Design, Device, NetId, PinDirection, ResourceKind};
    use texo_pnr::{PlacementConstraints, place_and_route};
    use texo_struo::import_ecp5;
    use texo_target_ecp5::{
        PipClassTimingRecord, TimingCornersRecord, find_global_clock_requirements, pack_lut_ffs,
        parse_lpf, read_architecture, resolve_lpf_port_cells,
    };

    use super::{
        Ecp5FlowError, Ecp5FlowOptions, Evidence, Gate, criticality_weight,
        delay_weighted_criticality, ecp5_timing_constraints, ecp5_timing_model, find_cell_pin,
        freeze_route_sinks_except, implement, implement_struo_ecp5, implement_with_constraints,
        pip_class_delay, retain_projection_timing_frontier, should_retry_global_ripup,
        slack_violations, staged_timing_score, verify_post_map_with_celox,
    };

    const ECP5_FIXTURE: &str = include_str!("../../texo-target-ecp5/fixtures/minimal-ecp5.json");

    #[test]
    fn timing_criticality_concentrates_on_the_worst_paths() {
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
    fn topology_trial_frontier_keeps_only_new_timing_records() {
        let mut candidates = vec![
            (50, 0, "topology winner"),
            (60, 1, "dominated"),
            (40, 2, "timing record"),
            (45, 3, "dominated later"),
        ];

        retain_projection_timing_frontier(&mut candidates);

        assert_eq!(
            candidates,
            vec![(50, 0, "topology winner"), (40, 2, "timing record")]
        );
    }

    #[test]
    fn failed_global_ripup_requires_a_material_wns_change_to_rearm() {
        assert!(should_retry_global_ripup(None, -1_000));
        assert!(!should_retry_global_ripup(Some(-500), -251));
        assert!(should_retry_global_ripup(Some(-500), -250));
    }

    #[test]
    fn timing_objective_is_zero_exactly_at_closure() {
        assert!(
            slack_violations([0, 10, 20].into_iter()).score()
                > slack_violations([-1, 10].into_iter()).score()
        );
        assert!(
            slack_violations([-10, -10].into_iter()).score()
                > slack_violations([-20, 0].into_iter()).score()
        );
        let violations = slack_violations([-20, 0, -5, 10].into_iter());
        assert_eq!(violations.total_negative_slack_ps(), -25);
        assert_eq!(violations.endpoints, 2);
    }

    #[test]
    fn timing_objective_rejects_small_wns_gains_that_destroy_tns() {
        let narrow = slack_violations([-100, 0].into_iter()).score();
        let widespread = slack_violations(std::iter::repeat_n(-99, 100)).score();

        assert!(
            staged_timing_score(narrow, narrow, -100, -100)
                > staged_timing_score(widespread, widespread, -99, -99)
        );
    }

    #[test]
    fn timing_objective_closes_setup_before_hold_eco() {
        let closed = slack_violations([0].into_iter()).score();
        let setup_near = slack_violations([-10].into_iter()).score();
        let setup_far = slack_violations([-20].into_iter()).score();
        let hold_bad = slack_violations([-1_000].into_iter()).score();

        assert!(
            staged_timing_score(setup_near, hold_bad, -10, -1_000)
                > staged_timing_score(setup_far, closed, -20, 10)
        );
        assert!(
            staged_timing_score(closed, hold_bad, 0, -1_000)
                > staged_timing_score(setup_near, closed, -10, 10)
        );
    }

    #[test]
    fn local_hold_repair_releases_only_violating_sink_arcs() {
        let mut design = Design::new();
        let first_driver = design.add_cell("source_a", ResourceKind::Logic);
        let first_output = design
            .add_pin(first_driver, "out", PinDirection::Output)
            .unwrap();
        let first_receiver = design.add_cell("sink_a", ResourceKind::Logic);
        let first_input = design
            .add_pin(first_receiver, "in", PinDirection::Input)
            .unwrap();
        let second_driver = design.add_cell("source_b", ResourceKind::Logic);
        let second_output = design
            .add_pin(second_driver, "out", PinDirection::Output)
            .unwrap();
        let second_receiver = design.add_cell("sink_b", ResourceKind::Logic);
        let second_input = design
            .add_pin(second_receiver, "in", PinDirection::Input)
            .unwrap();
        design.add_net("a", first_output, [first_input]).unwrap();
        design.add_net("b", second_output, [second_input]).unwrap();
        let device = Device::rectangular_logic(4, 4).unwrap();
        let implementation = place_and_route(&design, &device).unwrap();

        let frozen = freeze_route_sinks_except(
            &design,
            &device,
            &implementation.placement,
            &implementation.routes,
            &BTreeSet::from([(NetId(1), second_input)]),
        )
        .unwrap();

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
        let reset = source.add_input("reset");
        let state = source.add_register_output("state");
        source.add_register(RegisterCell::new(
            "state",
            state,
            data,
            clock,
            ClockEdge::Rising,
            None,
            Some(ResetControl {
                signal: reset,
                active: ActiveLevel::High,
                asynchronous: true,
                value: false,
            }),
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
        let ff_lsr = find_cell_pin(&design, ff, "LSR").unwrap();
        let ff_q = find_cell_pin(&design, ff, "Q").unwrap();

        assert_eq!(packing.clock_frequencies_hz().len(), 1);
        assert_eq!(constraints.clock_periods_ps().len(), 2);
        assert_eq!(constraints.clock_periods_ps()[&global_net], 40_000);
        assert_eq!(timing_model.clock_to_q(ff_q).unwrap().1.max_ps, 525);
        assert_eq!(timing_model.setup_hold(ff_data).unwrap().2.min_ps, 233);
        assert!(timing_model.setup_hold(ff_lsr).is_none());
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
