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
    NetRoute, Placement, PlacementConnectionDelayWorkspace, PlacementConstraints,
    PlacementRefinementWorkspace, PlacementRefiner, PnrError, PnrResult, RouteCapacityProjection,
    RoutingConstraints, RoutingCosts, RoutingWorkspace, place_analytically_with_net_sink_weights,
    place_and_route_with_constraints, placement_from_partial_bindings, rebind_placement_pins,
    retain_route_for_sinks, route_with_timing_costs_workspace_and_progress,
    route_with_workspace_and_progress, swap_placement_cells,
};
use texo_struo::{ActiveLevel, ImportedEcp5Design, PrimitiveMetadata};
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

/// Policy for caller-provided post-map functional-simulation evidence.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum PostMapSimulationPolicy {
    /// Reject physical implementation until the evidence gate is present.
    #[default]
    RequireEvidence,
    /// Permit implementation without a testbench and leave the gate absent.
    AllowMissing,
}

/// Configuration for the complete Struo-to-ECP5 physical implementation flow.
#[derive(Clone, Copy, Debug)]
pub struct Ecp5FlowOptions<'a> {
    /// Policy for caller-provided post-map functional-simulation evidence.
    pub post_map_simulation: PostMapSimulationPolicy,
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
            post_map_simulation: PostMapSimulationPolicy::default(),
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
/// routing. `PostMapSimulation` evidence is required by default; callers that
/// accept arbitrary designs without a testbench may explicitly disable that
/// precondition without recording the missing gate. Mapped-netlist and
/// physical-implementation evidence are committed only after every stage
/// succeeds.
///
/// # Errors
///
/// Returns an error for required-but-missing simulation evidence, speed grade,
/// or package selection, LPF resolution, target packing, placement, routing, or timing.
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
    if options.post_map_simulation == PostMapSimulationPolicy::RequireEvidence
        && !evidence.contains(Gate::PostMapSimulation)
    {
        return Err(Ecp5FlowError::MissingPostMapSimulation);
    }
    let requested_speed_grade = options
        .speed_grade
        .ok_or(Ecp5FlowError::MissingSpeedGrade)?;
    // LFE5UM5G's externally visible speed grade is `8`, but Project Trellis
    // stores its distinct characterization in the `8_5G` timing table. Match
    // nextpnr's target selection instead of silently timing a 5G part with the
    // ordinary LFE5U/M speed-8 model.
    let speed_grade_name =
        project_trellis_speed_grade(architecture.device().name(), requested_speed_grade);
    let speed_grade = architecture
        .speed_grades()
        .get(speed_grade_name)
        .ok_or_else(|| Ecp5FlowError::UnknownSpeedGrade(speed_grade_name.into()))?;

    let mut design = imported.design().clone();
    let constant_luts = imported
        .metadata()
        .iter()
        .filter_map(|(&cell, metadata)| {
            matches!(metadata, PrimitiveMetadata::Constant { .. }).then_some(cell)
        })
        .collect::<BTreeSet<_>>();
    let wide_luts = imported
        .wide_lut_clusters()
        .iter()
        .flatten()
        .copied()
        .collect::<BTreeSet<_>>();
    let mut packing = match options.lut_ff_pairs {
        Some(pairs) => {
            let pairs = named_lut_ff_pairs(&design, pairs)?;
            if let Some(pair) = pairs.iter().find(|pair| wide_luts.contains(&pair.lut)) {
                return Err(PackingError::InvalidLutFfPair {
                    lut: design.cells()[pair.lut.0].name.clone(),
                    ff: design.cells()[pair.ff.0].name.clone(),
                    reason: "wide-LUT members cannot use the ordinary LUT/FF path".into(),
                }
                .into());
            }
            pack_lut_ffs_with_pairs(&design, architecture, pairs)?
        }
        None => pack_lut_ffs_excluding(
            &design,
            architecture,
            constant_luts.iter().chain(&wide_luts).copied(),
        )?,
    };
    packing.pack_wide_luts(
        &design,
        architecture,
        imported.wide_lut_clusters().iter().cloned(),
    )?;
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
    packing.constrain_ff_slice_ce_muxes(
        architecture,
        ff_ce_control_sets(&design, imported.metadata()),
    );
    packing.constrain_ff_clock_muxes(
        architecture,
        ff_clock_control_sets(&design, imported.metadata()),
    );
    constrain_pll_outputs(&design, imported.metadata(), &mut packing)?;
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
    let mut timing_model = ecp5_timing_model(&design, &packing, speed_grade, &constant_luts)?;
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
            &constant_luts,
            costs,
            &mut global_routing_cache,
            &mut routing_workspace,
            &mut progress,
        )?
    {
        packing = candidate.packing;
        initial_implementation = candidate.implementation;
        initial_timing = candidate.timing;
        timing_model = ecp5_timing_model(&design, &packing, speed_grade, &constant_luts)?;
        timing_constraints = ecp5_timing_constraints(&design, &packing)?;
        costs.set_net_criticalities(timing_net_weights(&initial_timing, &timing_constraints));
        costs.set_sink_criticalities(timing_arc_weights(&initial_timing, &timing_constraints));
    }
    report_metric_phase("dedicated_edge_search", &mut phase_started);

    let (mut implementation, mut timing) = if let Some(costs) = closure_routing_costs.as_mut() {
        // Keep the closure context's long-lived placement-refiner borrow off
        // the authoritative packing so a post-closure packing ECO can mutate
        // it after this scope. Assignment tables are Arc-backed, making this
        // clone shallow for the device-sized data.
        let closure_packing = packing.clone();
        let placement_refiner = PlacementRefiner::new_with_workspace(
            &design,
            architecture.device(),
            closure_packing.constraints(),
            &mut placement_refinement_workspace,
        )?;
        report_metric_phase("closure_refiner_build", &mut phase_started);
        let optimized = TimingDrivenContext {
            design: &design,
            architecture,
            packing: &closure_packing,
            placement_refiner: &placement_refiner,
            global_routing_cache: &mut global_routing_cache,
            speed_grade,
            timing_model: &timing_model,
            timing_constraints: &timing_constraints,
            placement_weight_exponent: options.placement_weight_exponent,
            routing_workspace: &mut routing_workspace,
            global_ripup_attempted: false,
            critical_move_trials: BTreeSet::new(),
        }
        .optimize(initial_implementation, initial_timing, costs, &mut progress)?;
        drop(placement_refiner);
        optimized
    } else {
        (initial_implementation, initial_timing)
    };
    if let Some(costs) = closure_routing_costs.as_mut()
        && timing.worst_slack_ps.is_some_and(|slack| slack >= 0)
        && timing.worst_hold_slack_ps.is_some_and(|slack| slack < 0)
    {
        repair_hold_with_dedicated_edge_release(
            &design,
            architecture,
            speed_grade,
            &constant_luts,
            &mut packing,
            &mut implementation,
            &mut timing,
            costs,
            &mut global_routing_cache,
            &mut routing_workspace,
            &mut placement_refinement_workspace,
            &mut progress,
        )?;
    }
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

fn ff_ce_control_sets(
    design: &Design,
    metadata: &BTreeMap<CellId, PrimitiveMetadata>,
) -> Vec<(CellId, u64)> {
    metadata
        .iter()
        .filter_map(|(&cell, primitive)| {
            let PrimitiveMetadata::FlipFlop { enable, .. } = primitive else {
                return None;
            };
            let value = enable.map_or(0, |active| {
                let ce = design.cells()[cell.0]
                    .pins()
                    .iter()
                    .copied()
                    .find(|pin| design.pins()[pin.0].name == "CE")
                    .expect("enabled FF has a CE pin");
                let net = design.pins()[ce.0]
                    .net()
                    .expect("enabled FF CE is connected");
                1 + u64::try_from(net.0).expect("net ID fits u64") * 2
                    + u64::from(active == ActiveLevel::Low)
            });
            Some((cell, value))
        })
        .collect()
}

fn ff_clock_control_sets(
    design: &Design,
    metadata: &BTreeMap<CellId, PrimitiveMetadata>,
) -> Vec<(CellId, u64)> {
    metadata
        .iter()
        .filter_map(|(&cell, primitive)| {
            let PrimitiveMetadata::FlipFlop { edge, .. } = primitive else {
                return None;
            };
            let clock = design.cells()[cell.0]
                .pins()
                .iter()
                .copied()
                .find(|pin| design.pins()[pin.0].name == "CLK")
                .expect("FF has a CLK pin");
            let net = design.pins()[clock.0].net().expect("FF CLK is connected");
            let value = u64::try_from(net.0).expect("net ID fits u64") * 2
                + u64::from(*edge == texo_struo::ClockEdge::Falling);
            Some((cell, value))
        })
        .collect()
}

fn project_trellis_speed_grade<'a>(device: &str, requested: &'a str) -> &'a str {
    if requested == "8" && device.starts_with("LFE5UM5G") {
        "8_5G"
    } else {
        requested
    }
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

/// Repairs post-setup hold failures whose routing freedom was removed by an
/// earlier dedicated LUT→FF packing choice. Pairs are released one at a time,
/// locally rerouted with their required minimum delay, and committed only
/// after full STA preserves setup closure and improves the timing objective.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn repair_hold_with_dedicated_edge_release(
    design: &Design,
    architecture: &Ecp5Architecture,
    speed_grade: &SpeedGradeRecord,
    constant_cells: &BTreeSet<CellId>,
    packing: &mut Ecp5Packing,
    implementation: &mut PnrResult,
    timing: &mut TimingReport,
    routing_costs: &mut RoutingCosts,
    global_routing_cache: &mut Ecp5GlobalRoutingCache<'_>,
    routing_workspace: &mut RoutingWorkspace,
    placement_refinement_workspace: &mut PlacementRefinementWorkspace,
    progress: &mut impl FnMut(Ecp5FlowStage),
) -> Result<(), Ecp5FlowError> {
    loop {
        let mut candidates = timing
            .hold_checks
            .iter()
            .filter(|check| check.slack_ps < 0)
            .filter_map(|check| {
                packing
                    .lut_ff_pairs()
                    .iter()
                    .copied()
                    .find(|pair| pair.ff == check.cell)
                    .map(|pair| (check.slack_ps, check.data_pin, pair))
            })
            .collect::<Vec<_>>();
        candidates.sort_unstable_by_key(|&(slack, pin, pair)| (slack, pair.lut, pair.ff, pin));
        if candidates.is_empty() {
            break;
        }

        let minimums = hold_sink_min_delays(timing);
        let mut accepted = false;
        for (slack_ps, data_pin, pair) in candidates {
            let Some(net) = design.pins()[data_pin.0].net() else {
                continue;
            };
            let key = (net, data_pin);
            let Some(&minimum_ps) = minimums.get(&key) else {
                continue;
            };
            let mut trial_packing = packing.clone();
            trial_packing.release_lut_ff_pair(design, pair.lut, pair.ff)?;
            let trial_model =
                ecp5_timing_model(design, &trial_packing, speed_grade, constant_cells)?;
            let trial_constraints = ecp5_timing_constraints(design, &trial_packing)?;
            let requested_minimums = BTreeMap::from([(key, minimum_ps)]);
            let Some((mut trial_implementation, mut trial_timing)) = route_hold_trial(
                design,
                architecture,
                speed_grade,
                &trial_packing,
                implementation,
                timing,
                &trial_model,
                &trial_constraints,
                requested_minimums.clone(),
                routing_costs,
                global_routing_cache,
                routing_workspace,
                progress,
            )?
            else {
                if metrics_enabled() {
                    eprintln!(
                        "[metrics] hold_pair_release lut={} ff={} slack={} rejected=routing",
                        pair.lut.0, pair.ff.0, slack_ps
                    );
                }
                continue;
            };
            // Re-score a newly general-routed edge after STA. Before release
            // it was a short dedicated arc and therefore carried little
            // setup criticality; one feedback route lets the router select a
            // path that satisfies the same hold floor without needlessly
            // crossing setup closure.
            for _ in 0..MAX_HOLD_ROUTE_FEEDBACKS {
                if trial_timing.worst_slack_ps.is_some_and(|setup| setup >= 0) {
                    break;
                }
                let previous_fingerprint =
                    implementation_topology_fingerprint(design, &trial_implementation);
                let Some((refined_implementation, refined_timing)) = route_hold_trial(
                    design,
                    architecture,
                    speed_grade,
                    &trial_packing,
                    &trial_implementation,
                    &trial_timing,
                    &trial_model,
                    &trial_constraints,
                    requested_minimums.clone(),
                    routing_costs,
                    global_routing_cache,
                    routing_workspace,
                    progress,
                )?
                else {
                    break;
                };
                let unchanged =
                    implementation_topology_fingerprint(design, &refined_implementation)
                        == previous_fingerprint;
                if timing_score(&refined_timing) <= timing_score(&trial_timing) {
                    break;
                }
                trial_implementation = refined_implementation;
                trial_timing = refined_timing;
                if unchanged {
                    break;
                }
            }
            if trial_timing
                .worst_slack_ps
                .is_some_and(|setup| (-MAX_HOLD_SETUP_RECOVERY_PS..0).contains(&setup))
            {
                let placement_refiner = PlacementRefiner::new_with_workspace(
                    design,
                    architecture.device(),
                    trial_packing.constraints(),
                    placement_refinement_workspace,
                )?;
                let recovered = TimingDrivenContext {
                    design,
                    architecture,
                    packing: &trial_packing,
                    placement_refiner: &placement_refiner,
                    global_routing_cache,
                    speed_grade,
                    timing_model: &trial_model,
                    timing_constraints: &trial_constraints,
                    placement_weight_exponent: 1,
                    routing_workspace,
                    global_ripup_attempted: true,
                    critical_move_trials: BTreeSet::new(),
                }
                .refine_critical_path_vertices(
                    vec![(trial_implementation, trial_timing)],
                    &placement_refiner,
                    routing_costs,
                    false,
                    progress,
                )?;
                (trial_implementation, trial_timing) = recovered
                    .into_iter()
                    .max_by_key(|(implementation, timing)| {
                        (timing_score(timing), Reverse(implementation.total_pips))
                    })
                    .expect("setup recovery archive is non-empty");
            }
            let improves = trial_timing.worst_slack_ps.is_some_and(|setup| setup >= 0)
                && timing_score(&trial_timing) > timing_score(timing);
            progress(Ecp5FlowStage::TimingTrialDecision {
                improves_objective: improves,
            });
            if metrics_enabled() {
                eprintln!(
                    "[metrics] hold_pair_release lut={} ff={} slack={} wns={:?} whs={:?} accepted={improves}",
                    pair.lut.0,
                    pair.ff.0,
                    slack_ps,
                    trial_timing.worst_slack_ps,
                    trial_timing.worst_hold_slack_ps,
                );
            }
            if improves {
                *packing = trial_packing;
                *implementation = trial_implementation;
                *timing = trial_timing;
                accepted = true;
                break;
            }
        }
        if !accepted {
            break;
        }
    }

    // Once dedicated blockers have been released, repair all remaining
    // general-routing hold arcs together. This also handles designs whose
    // violations never involved a dedicated edge.
    let model = ecp5_timing_model(design, packing, speed_grade, constant_cells)?;
    let constraints = ecp5_timing_constraints(design, packing)?;
    let mut rolling_implementation = implementation.clone();
    let mut rolling_timing = timing.clone();
    let mut best_score = timing_score(timing);
    let mut best = None;
    let mut seen = BTreeSet::from([implementation_topology_fingerprint(
        design,
        &rolling_implementation,
    )]);
    let mut accumulated_minimums = BTreeMap::<(NetId, CellPinId), u64>::new();
    for _ in 0..MAX_GENERAL_HOLD_REPAIRS {
        let new_minimums = hold_sink_min_delays(&rolling_timing);
        if new_minimums.is_empty() {
            break;
        }
        accumulate_hold_minimums(&mut accumulated_minimums, new_minimums);
        let Some((trial_implementation, trial_timing)) = route_hold_trial(
            design,
            architecture,
            speed_grade,
            packing,
            &rolling_implementation,
            &rolling_timing,
            &model,
            &constraints,
            accumulated_minimums.clone(),
            routing_costs,
            global_routing_cache,
            routing_workspace,
            progress,
        )?
        else {
            break;
        };
        if trial_timing.worst_slack_ps.is_none_or(|setup| setup < 0) {
            break;
        }
        let fingerprint = implementation_topology_fingerprint(design, &trial_implementation);
        if !seen.insert(fingerprint) {
            break;
        }
        let score = timing_score(&trial_timing);
        let improves = score > best_score;
        progress(Ecp5FlowStage::TimingTrialDecision {
            improves_objective: improves,
        });
        if metrics_enabled() {
            eprintln!(
                "[metrics] general_hold_feedback wns={:?} whs={:?} best={improves}",
                trial_timing.worst_slack_ps, trial_timing.worst_hold_slack_ps,
            );
        }
        if improves {
            best_score = score;
            best = Some((trial_implementation.clone(), trial_timing.clone()));
        }
        let closed = trial_timing.met_timing();
        rolling_implementation = trial_implementation;
        rolling_timing = trial_timing;
        if closed {
            break;
        }
    }
    if let Some((best_implementation, best_timing)) = best {
        *implementation = best_implementation;
        *timing = best_timing;
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn route_hold_trial(
    design: &Design,
    architecture: &Ecp5Architecture,
    speed_grade: &SpeedGradeRecord,
    packing: &Ecp5Packing,
    implementation: &PnrResult,
    timing: &TimingReport,
    timing_model: &TimingModel,
    timing_constraints: &TimingConstraints,
    minimums: BTreeMap<(NetId, CellPinId), u64>,
    routing_costs: &RoutingCosts,
    global_routing_cache: &mut Ecp5GlobalRoutingCache<'_>,
    routing_workspace: &mut RoutingWorkspace,
    progress: &mut impl FnMut(Ecp5FlowStage),
) -> Result<Option<TimingCandidate>, Ecp5FlowError> {
    let released = minimums.keys().copied().collect::<BTreeSet<_>>();
    // Re-resolve physical pin bindings under the trial packing. Merely
    // cloning the incumbent placement would retain the old dedicated `DI`
    // binding even after the logical FF input was rebound to general `M`.
    let trial_placement = rebind_placement_pins(
        design,
        architecture.device(),
        packing.constraints(),
        &implementation.placement,
    )?;
    let recomputed_global_routing;
    let base = if global_clock_endpoints_unchanged(
        design,
        packing,
        &implementation.placement,
        &trial_placement,
    ) {
        let restrictions = packing.routing_restrictions_cached(
            design,
            architecture,
            &trial_placement,
            global_routing_cache,
        )?;
        if let Some(incumbent) =
            global_routes_from_implementation(packing, implementation, restrictions)
        {
            recomputed_global_routing = incumbent;
            &recomputed_global_routing
        } else {
            recomputed_global_routing = packing.global_routing_constraints_cached(
                design,
                architecture,
                &trial_placement,
                global_routing_cache,
            )?;
            &recomputed_global_routing
        }
    } else {
        recomputed_global_routing = packing.global_routing_constraints_cached(
            design,
            architecture,
            &trial_placement,
            global_routing_cache,
        )?;
        &recomputed_global_routing
    };
    progress(Ecp5FlowStage::TimingDrivenGlobalClocksRouted);
    let frozen = freeze_unchanged_routes(design, implementation, &trial_placement, base, &released);
    let mut costs = routing_costs.clone();
    costs.set_max_iterations(LOCAL_TRIAL_ROUTING_ITERATIONS);
    costs.set_net_criticalities(timing_net_weights(timing, timing_constraints));
    costs.set_sink_criticalities(timing_arc_weights(timing, timing_constraints));
    costs.set_sink_min_delays_ps(minimums);
    let routed = match route_with_timing_costs_workspace_and_progress(
        design,
        architecture.device(),
        trial_placement,
        &frozen,
        &costs,
        routing_workspace,
        |event| progress(Ecp5FlowStage::TimingDrivenRouting(event)),
    ) {
        Ok(routed) => routed,
        Err(PnrError::CongestionNotResolved { .. } | PnrError::Unroutable { .. }) => {
            return Ok(None);
        }
        Err(error) => return Err(error.into()),
    };
    progress(Ecp5FlowStage::TimingDrivenRouted);
    let report = analyze_ecp5_implementation(
        design,
        architecture,
        speed_grade,
        &routed,
        timing_model,
        timing_constraints,
    )?;
    progress(timing_snapshot(&report));
    Ok(Some((routed, report)))
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
    constant_cells: &BTreeSet<CellId>,
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
        let trial_timing_model =
            ecp5_timing_model(design, &trial_packing, speed_grade, constant_cells)?;
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

struct TimingDrivenContext<'a, 'work, 'cache> {
    design: &'a Design,
    architecture: &'a Ecp5Architecture,
    packing: &'a Ecp5Packing,
    placement_refiner: &'work PlacementRefiner<'a>,
    global_routing_cache: &'work mut Ecp5GlobalRoutingCache<'cache>,
    speed_grade: &'a SpeedGradeRecord,
    timing_model: &'a TimingModel,
    timing_constraints: &'a TimingConstraints,
    /// User-selected sharpening for timing-driven analytical placement.
    /// An exponent of one preserves the historical weighting exactly.
    placement_weight_exponent: u32,
    routing_workspace: &'work mut RoutingWorkspace,
    /// Whether the post-global-placement data routes have already received
    /// their one full-chip timing renegotiation. Later critical moves reroute
    /// every affected net incrementally and must not reopen the entire chip.
    global_ripup_attempted: bool,
    /// Exact seed-route/proposed-placement pairs already sent through a local
    /// negotiated route and STA. Closure rounds often rediscover the same
    /// move; only a changed route topology makes it worth evaluating again.
    critical_move_trials: BTreeSet<(u64, u64)>,
}

impl TimingDrivenContext<'_, '_, '_> {
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
        routing_costs
            .set_sink_criticalities(timing_arc_weights(&initial_timing, self.timing_constraints));
        let initial = (&initial_implementation, &initial_timing);
        let timed_candidate = self.timing_driven_seed(initial, routing_costs, progress)?;
        let mut seeds = vec![(initial_implementation, initial_timing)];
        if let Some(candidate) = timed_candidate {
            seeds.push(candidate);
        }
        let mut archive = select_timing_frontier(seeds);
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
        archive = self.repair_setup_and_reenter_critical(
            archive,
            placement_refiner,
            routing_costs,
            progress,
        )?;
        report_metric_phase("closure_setup_eco", &mut phase_started);
        emit_archive_metric("closure_setup_eco", self, &archive);
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
        // A timing-clean implementation always wins. Before closure, report
        // the best achieved period instead of silently replacing it with an
        // aggregate-slack candidate from the parallel search trajectory.
        let timing_closed = archive.iter().any(|(_, timing)| timing.met_timing());
        let (final_implementation, final_timing) = archive
            .into_iter()
            .filter(|(_, timing)| !timing_closed || timing.met_timing())
            .max_by_key(|(_, timing)| {
                if timing_closed {
                    (None, timing_score(timing))
                } else {
                    (timing.worst_slack_ps, timing_score(timing))
                }
            })
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

    fn repair_setup_and_reenter_critical(
        &mut self,
        mut archive: Vec<TimingCandidate>,
        placement_refiner: &PlacementRefiner<'_>,
        routing_costs: &mut RoutingCosts,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<Vec<TimingCandidate>, Ecp5FlowError> {
        let previous_topologies = archive
            .iter()
            .map(|(implementation, _)| {
                implementation_topology_fingerprint(self.design, implementation)
            })
            .collect::<BTreeSet<_>>();
        archive = self.repair_setup_archive(archive, routing_costs, progress)?;
        let changed_topology = archive.iter().any(|(implementation, _)| {
            !previous_topologies.contains(&implementation_topology_fingerprint(
                self.design,
                implementation,
            ))
        });
        let setup_closed = archive
            .iter()
            .any(|(_, timing)| timing.worst_slack_ps.is_some_and(|slack| slack >= 0));
        if changed_topology && !setup_closed {
            // A setup reroute can expose a different worst endpoint while
            // leaving placement unchanged. Re-enter critical placement from
            // that new route topology instead of sending the stale placement
            // directly to a whole-design basin kick.
            archive =
                self.close_setup_critically(archive, placement_refiner, routing_costs, progress)?;
            archive = self.repair_setup_archive(archive, routing_costs, progress)?;
        }
        Ok(archive)
    }

    /// Iterates timing-weighted analytical placement from a legalized anchor.
    ///
    /// Each accepted route becomes the next anchor and supplies fresh timing
    /// weights. This keeps the solve in the incumbent's placement basin while
    /// letting timing feedback move it continuously; real routed STA gates
    /// every iteration, so a regressing speculative solve cannot replace the
    /// already legal implementation.
    fn timing_driven_seed(
        &mut self,
        initial: (&PnrResult, &TimingReport),
        routing_costs: &mut RoutingCosts,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<Option<TimingCandidate>, Ecp5FlowError> {
        let (initial_implementation, initial_timing) = initial;
        let mut anchor = initial_implementation.placement.clone();
        let mut feedback = initial_timing.clone();
        let mut best_score = timing_score(initial_timing);
        let mut best = None;
        for iteration in 1..=MAX_ANCHORED_PLACEMENT_ROUNDS {
            let weights = timing_placement_weights_with_exponent(
                &feedback,
                self.timing_constraints,
                self.placement_weight_exponent,
            );
            let placement = self
                .placement_refiner
                .place_analytically_anchored(&weights, &anchor, iteration)?;
            progress(Ecp5FlowStage::TimingDrivenPlaced);
            if placement == anchor {
                break;
            }
            let routing = self.packing.global_routing_constraints_cached(
                self.design,
                self.architecture,
                &placement,
                self.global_routing_cache,
            )?;
            progress(Ecp5FlowStage::TimingDrivenGlobalClocksRouted);
            routing_costs
                .set_net_criticalities(timing_net_weights(&feedback, self.timing_constraints));
            routing_costs
                .set_sink_criticalities(timing_arc_weights(&feedback, self.timing_constraints));
            routing_costs.set_sink_min_delays_ps(BTreeMap::new());
            let candidate =
                match self.route_and_analyze(placement, &routing, Some(routing_costs), progress) {
                    Ok(candidate) => candidate,
                    Err(Ecp5FlowError::Pnr(_)) => break,
                    Err(error) => return Err(error),
                };
            let score = timing_score(&candidate.1);
            let improves_objective = score > best_score;
            progress(Ecp5FlowStage::TimingTrialDecision { improves_objective });
            if !improves_objective {
                break;
            }
            best_score = score;
            anchor = candidate.0.placement.clone();
            feedback = candidate.1.clone();
            best = Some(candidate);
        }
        Ok(best)
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

    /// Returns whether the cumulative geometric estimate of a speculative
    /// multi-cell move does not regress.  Batches use this stricter gate than
    /// single-cell proposals because their independent moves can interact.
    fn batch_estimate_improves(
        &self,
        old: &Placement,
        new: &Placement,
        weights: &BTreeMap<NetId, u64>,
    ) -> bool {
        let nets = self.moved_nets(old, new).collect::<BTreeSet<_>>();
        match (
            self.weighted_net_estimate(old, &nets, weights),
            self.weighted_net_estimate(new, &nets, weights),
        ) {
            (Some(old_estimate), Some(new_estimate)) => new_estimate <= old_estimate,
            _ => true,
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
                false,
                progress,
            )?;
            archive =
                self.refine_critical_routes_multiresolution(archive, routing_costs, progress)?;
            let setup_closed = archive
                .iter()
                .any(|(_, timing)| timing.worst_slack_ps.is_some_and(|slack| slack >= 0));
            if setup_closed {
                return Ok(archive);
            }
        }
        // Aggregate closure deliberately rejects some WNS gains to protect
        // TNS. Revisit those physical proposals under an explicitly Fmax-first
        // objective only after the established trajectory has completed.
        self.critical_move_trials.clear();
        for _ in 0..MAX_FMAX_CLOSURE_ROUNDS {
            let incumbent = archive
                .iter()
                .map(|(implementation, timing)| setup_closure_score(implementation, timing))
                .max()
                .expect("the timing archive is non-empty");
            archive = self.refine_critical_path_vertices(
                archive,
                placement_refiner,
                routing_costs,
                true,
                progress,
            )?;
            archive =
                self.refine_critical_routes_multiresolution(archive, routing_costs, progress)?;
            if archive
                .iter()
                .any(|(_, timing)| timing.worst_slack_ps.is_some_and(|slack| slack >= 0))
            {
                break;
            }
            let improved = archive
                .iter()
                .map(|(implementation, timing)| setup_closure_score(implementation, timing))
                .max()
                .is_some_and(|score| score > incumbent);
            if !improved {
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
        let mut consecutive_wns_regressions = 0_usize;
        for refinement_round in 0..MAX_INCREMENTAL_REFINEMENTS {
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
            let mut equivalent_move_peak = None;
            for max_units in REFINED_PLACEMENT_UNIT_LIMITS {
                if equivalent_move_peak.is_some_and(|peak| max_units > peak) {
                    if metrics_enabled() {
                        eprintln!(
                            "[metrics] monotonic_trial round={} units={max_units} skipped=inactive_limit",
                            refinement_round + 1,
                        );
                    }
                    continue;
                }
                let trial_started = Instant::now();
                let (candidate, move_peak) = self.refine_candidate(
                    &seed.0,
                    &seed.1,
                    placement_refiner,
                    routing_costs,
                    max_units,
                    progress,
                )?;
                equivalent_move_peak = Some(move_peak);
                if metrics_enabled() {
                    eprintln!(
                        "[metrics] monotonic_trial round={} units={max_units} elapsed={:?} wns={:?} tns={:?}",
                        refinement_round + 1,
                        trial_started.elapsed(),
                        candidate
                            .as_ref()
                            .and_then(|candidate| candidate.1.worst_slack_ps),
                        candidate.as_ref().map(|candidate| slack_violations(
                            candidate.1.setup_checks.iter().map(|check| check.slack_ps)
                        )
                        .total_negative_slack_ps()),
                    );
                }
                let Some(child) = candidate else {
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
            consecutive_wns_regressions = next_wns_regression_streak(
                consecutive_wns_regressions,
                seed.1.worst_slack_ps,
                improved.1.worst_slack_ps,
            );
            archive.push(improved);
            archive = select_timing_frontier(archive);
            // Once two accepted global moves improve the aggregate objective
            // only by sacrificing the worst path, another large displacement
            // portfolio is the wrong hierarchy. Hand the unchanged winning
            // seed to the critical-vertex layer instead of routing four more
            // global candidates merely to prove that none wins.
            if consecutive_wns_regressions >= MAX_CONSECUTIVE_MONOTONIC_WNS_REGRESSIONS {
                if metrics_enabled() {
                    eprintln!(
                        "[metrics] monotonic_transition reason=wns_regression_streak streak={consecutive_wns_regressions}"
                    );
                }
                break;
            }
        }
        Ok(archive)
    }
    #[allow(clippy::too_many_lines)]
    fn refine_critical_path_vertices(
        &mut self,
        mut archive: Vec<TimingCandidate>,
        placement_refiner: &PlacementRefiner<'_>,
        routing_costs: &mut RoutingCosts,
        prefer_fmax: bool,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<Vec<TimingCandidate>, Ecp5FlowError> {
        let mut setup_eco_attempted = false;
        for move_distance in CRITICAL_PATH_MOVE_DISTANCES {
            for refinement_round in 0..MAX_CRITICAL_PATH_VERTEX_REFINEMENTS {
                if archive
                    .iter()
                    .any(|(_, timing)| timing.worst_slack_ps.is_some_and(|slack| slack >= 0))
                {
                    return Ok(archive);
                }
                let seed = archive
                    .iter()
                    .max_by_key(|(implementation, timing)| {
                        if prefer_fmax {
                            setup_closure_score(implementation, timing)
                        } else {
                            (
                                i128::MIN,
                                timing_score(timing),
                                Reverse(implementation.total_pips),
                            )
                        }
                    })
                    .expect("the timing archive is non-empty")
                    .clone();
                if !setup_eco_attempted
                    && seed
                        .1
                        .worst_slack_ps
                        .is_some_and(|slack| (-LOCAL_SETUP_ECO_TRIGGER_PS..0).contains(&slack))
                {
                    setup_eco_attempted = true;
                    if let Some(repaired) = self.repair_setup_locally(
                        &seed.0,
                        &seed.1,
                        routing_costs,
                        texo_pnr::ROUTING_DELAY_QUANTUM_PS,
                        progress,
                    )? {
                        let improves_objective = timing_score(&repaired.1) > timing_score(&seed.1);
                        progress(Ecp5FlowStage::TimingTrialDecision { improves_objective });
                        if improves_objective {
                            let setup_closed =
                                repaired.1.worst_slack_ps.is_some_and(|slack| slack >= 0);
                            archive.push(repaired);
                            archive = select_timing_frontier(archive);
                            if setup_closed {
                                return Ok(archive);
                            }
                        }
                    }
                }
                let trial_started = Instant::now();
                let candidates = self.refine_critical_path_cells(
                    &seed.0,
                    &seed.1,
                    placement_refiner,
                    routing_costs,
                    move_distance,
                    prefer_fmax,
                    progress,
                )?;
                let aggregate_refinement = candidates
                    .iter()
                    .max_by_key(|(_, timing)| timing_score(timing))
                    .filter(|(_, timing)| timing_score(timing) > timing_score(&seed.1))
                    .cloned();
                let fmax_refinement = candidates
                    .into_iter()
                    .max_by_key(|(implementation, timing)| {
                        setup_closure_score(implementation, timing)
                    })
                    .filter(|(implementation, timing)| {
                        setup_closure_score(implementation, timing)
                            > setup_closure_score(&seed.0, &seed.1)
                    });
                let refinement = if prefer_fmax {
                    fmax_refinement
                } else {
                    aggregate_refinement
                };
                if metrics_enabled() {
                    let objective = if prefer_fmax { "fmax" } else { "aggregate" };
                    eprintln!(
                        "[metrics] critical_transition objective={objective} distance={move_distance} round={} elapsed={:?} seed_wns={:?} child_wns={:?} child_tns={:?}",
                        refinement_round + 1,
                        trial_started.elapsed(),
                        seed.1.worst_slack_ps,
                        refinement
                            .as_ref()
                            .and_then(|candidate| candidate.1.worst_slack_ps),
                        refinement.as_ref().map(|candidate| slack_violations(
                            candidate.1.setup_checks.iter().map(|check| check.slack_ps)
                        )
                        .total_negative_slack_ps()),
                    );
                }
                let Some(improved) = refinement else { break };
                let regresses_wns = improved
                    .1
                    .worst_slack_ps
                    .zip(seed.1.worst_slack_ps)
                    .is_some_and(|(child, parent)| child <= parent);
                archive.push(improved);
                archive = select_timing_frontier(archive);
                // Once the whole-design rip-up has already supplied a
                // fresh topology, an aggregate-slack move that sacrifices
                // WNS is a hierarchy transition. The Fmax pass never
                // accepts such a move and may keep walking its frontier.
                if !prefer_fmax && self.global_ripup_attempted && regresses_wns {
                    if metrics_enabled() {
                        eprintln!("[metrics] critical_transition reason=post_ripup_wns_regression");
                    }
                    return Ok(archive);
                }
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
            if self.global_ripup_attempted {
                if metrics_enabled() {
                    eprintln!("[metrics] global_ripup_transition reason=already_attempted");
                }
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
            self.global_ripup_attempted = true;
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
            // A basin kick is speculative and starts from a placement with no
            // resident data routes.  Successful full routes in the measured
            // portfolio settle within four negotiations; a candidate that is
            // still congested after eight iterations has historically run to
            // the 32-iteration hard limit and then been discarded.  Bound
            // only this disposable trial, leaving authoritative full routes
            // and accepted incremental work at their normal budget.
            let mut bounded_costs = routing_costs.clone();
            bounded_costs.set_max_iterations(SPECULATIVE_FULL_ROUTING_ITERATIONS);
            let trial =
                match self.route_and_analyze(kicked, &routing, Some(&bounded_costs), progress) {
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
        let restrictions = self.packing.routing_restrictions_cached(
            self.design,
            self.architecture,
            &implementation.placement,
            self.global_routing_cache,
        )?;
        let frozen = freeze_route_sinks_except(
            self.design,
            self.architecture.device(),
            &implementation.placement,
            &implementation.routes,
            &restrictions,
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

    fn repair_setup_locally(
        &mut self,
        implementation: &PnrResult,
        timing: &TimingReport,
        routing_costs: &mut RoutingCosts,
        delay_quantum_ps: u64,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<Option<TimingCandidate>, Ecp5FlowError> {
        let repair_sinks = timing
            .net_setup_slacks
            .iter()
            .filter_map(|edge| (edge.slack_ps < 0).then_some((edge.net, edge.sink)))
            .collect::<BTreeSet<_>>();
        if repair_sinks.is_empty() {
            return Ok(None);
        }
        let restrictions = self.packing.routing_restrictions_cached(
            self.design,
            self.architecture,
            &implementation.placement,
            self.global_routing_cache,
        )?;
        let frozen = freeze_route_sinks_except(
            self.design,
            self.architecture.device(),
            &implementation.placement,
            &implementation.routes,
            &restrictions,
            &repair_sinks,
        )?;
        routing_costs.set_net_criticalities(timing_net_weights(timing, self.timing_constraints));
        routing_costs.set_sink_criticalities(timing_arc_weights(timing, self.timing_constraints));
        routing_costs.set_sink_min_delays_ps(BTreeMap::new());
        routing_costs.set_detailed_timing_nets(released_net_ids(&repair_sinks));
        routing_costs.set_detailed_delay_quantum_ps(delay_quantum_ps);
        let repaired = self.route_local_trial_and_analyze(
            implementation.placement.clone(),
            &frozen,
            routing_costs,
            progress,
        );
        routing_costs.set_detailed_timing_nets(BTreeSet::new());
        routing_costs.set_detailed_delay_quantum_ps(texo_pnr::ROUTING_DELAY_QUANTUM_PS);
        match repaired {
            Ok(repaired) => Ok(Some(repaired)),
            Err(Ecp5FlowError::Pnr(_)) => Ok(None),
            Err(error) => Err(error),
        }
    }

    fn repair_setup_archive(
        &mut self,
        mut archive: Vec<TimingCandidate>,
        routing_costs: &mut RoutingCosts,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<Vec<TimingCandidate>, Ecp5FlowError> {
        for delay_quantum_ps in LOCAL_SETUP_REPAIR_QUANTA_PS {
            let seed = archive
                .iter()
                .max_by_key(|(_, timing)| timing_score(timing))
                .expect("the timing archive is non-empty")
                .clone();
            if seed.1.worst_slack_ps.is_none_or(|slack| slack >= 0) {
                break;
            }
            let Some(repaired) = self.repair_setup_locally(
                &seed.0,
                &seed.1,
                routing_costs,
                delay_quantum_ps,
                progress,
            )?
            else {
                break;
            };
            let improves_objective = timing_score(&repaired.1) > timing_score(&seed.1);
            progress(Ecp5FlowStage::TimingTrialDecision { improves_objective });
            if !improves_objective {
                continue;
            }
            archive.push(repaired);
            archive = select_timing_frontier(archive);
        }
        Ok(archive)
    }

    fn refine_candidate(
        &mut self,
        implementation: &PnrResult,
        timing: &TimingReport,
        placement_refiner: &PlacementRefiner<'_>,
        routing_costs: &mut RoutingCosts,
        max_refined_units: usize,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<(Option<TimingCandidate>, usize), Ecp5FlowError> {
        let refinement_weights = timing_placement_weights_with_exponent(
            timing,
            self.timing_constraints,
            self.placement_weight_exponent,
        );
        let sink_budgets = placement_sink_budgets(
            self.design,
            self.architecture.device(),
            &implementation.placement,
            timing,
        );
        let (refined_placement, move_peak) = placement_refiner
            .refine_with_net_sink_weights_limited_and_move_peak(
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
            return Ok((None, move_peak));
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
        let candidate = match self.route_local_trial_and_analyze(
            refined_placement,
            &incremental_routing,
            routing_costs,
            progress,
        ) {
            Ok(candidate) => Some(candidate),
            Err(Ecp5FlowError::Pnr(_)) => None,
            Err(error) => return Err(error),
        };
        Ok((candidate, move_peak))
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

    #[allow(clippy::too_many_lines, clippy::too_many_arguments)]
    fn refine_critical_path_cells(
        &mut self,
        implementation: &PnrResult,
        timing: &TimingReport,
        placement_refiner: &PlacementRefiner<'_>,
        routing_costs: &mut RoutingCosts,
        max_move_distance: u64,
        prefer_fmax: bool,
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
        let incumbent_global_routing = global_routes_from_implementation(
            self.packing,
            implementation,
            RoutingConstraints::new(),
        );
        let mut placement = implementation.placement.clone();
        let mut rolling_implementation = implementation.clone();
        let mut rolling_moves = 0_usize;
        let mut candidates = Vec::new();
        // A critical pass revisits many of the same physical endpoint pairs
        // while considering adjacent path cells. Their bounded local-route
        // delays are placement-independent once the wires are known, so keep
        // the exact results for this pass rather than re-running A* per cell.
        let mut local_delay_workspace = PlacementConnectionDelayWorkspace::new();
        if max_move_distance <= 2
            && (worst_slack_ps <= LOCAL_BATCH_RECOVERY_WNS_PS
                || worst_slack_ps >= LOCAL_BATCH_NEAR_CLOSURE_WNS_PS)
        {
            let mut batch = placement.clone();
            let mut batch_moves = Vec::new();
            for (cell, (_, connections, targets)) in cells.iter().take(LOCAL_CRITICAL_BATCH_SIZE) {
                let proposals = placement_refiner.refine_cell_connection_delays_with_cache(
                    batch.clone(),
                    *cell,
                    connections,
                    targets,
                    routing_costs.pip_delays_ps(),
                    None,
                    max_move_distance,
                    1,
                    &mut local_delay_workspace,
                )?;
                let Some(refined) = proposals.into_iter().next() else {
                    break;
                };
                if self.estimate_rejects(&batch, &refined, &prescreen_weights) {
                    break;
                }
                batch_moves.push((*cell, batch.bindings()[cell.0], refined.bindings()[cell.0]));
                batch = refined;
            }
            if batch_moves.len() == LOCAL_CRITICAL_BATCH_SIZE
                && self.batch_estimate_improves(&placement, &batch, &prescreen_weights)
            {
                let trial_key = (seed_fingerprint, placement_fingerprint(self.design, &batch));
                if self.critical_move_trials.insert(trial_key) {
                    for &(cell, from, to) in &batch_moves {
                        progress(Ecp5FlowStage::CriticalPathMove { cell, from, to });
                    }
                    let mut recomputed_global_routing;
                    let base = if let Some(incumbent) = incumbent_global_routing.as_ref()
                        && global_clock_endpoints_unchanged(
                            self.design,
                            self.packing,
                            &implementation.placement,
                            &batch,
                        ) {
                        recomputed_global_routing = self.packing.routing_restrictions_cached(
                            self.design,
                            self.architecture,
                            &batch,
                            self.global_routing_cache,
                        )?;
                        for route in incumbent.routes().values() {
                            recomputed_global_routing.add_route(route.clone());
                        }
                        &recomputed_global_routing
                    } else {
                        recomputed_global_routing =
                            self.packing.global_routing_constraints_cached(
                                self.design,
                                self.architecture,
                                &batch,
                                self.global_routing_cache,
                            )?;
                        &recomputed_global_routing
                    };
                    progress(Ecp5FlowStage::TimingDrivenGlobalClocksRouted);
                    let frozen = freeze_unchanged_routes(
                        self.design,
                        implementation,
                        &batch,
                        base,
                        &critical_sinks,
                    );
                    routing_costs.set_net_criticalities(prescreen_weights.clone());
                    routing_costs.set_sink_criticalities(route_arc_weights.clone());
                    routing_costs.set_sink_min_delays_ps(BTreeMap::new());
                    routing_costs.set_detailed_timing_nets(released_net_ids(&critical_sinks));
                    routing_costs.set_detailed_delay_quantum_ps(texo_pnr::ROUTING_DELAY_QUANTUM_PS);
                    let trial =
                        self.route_local_trial_and_analyze(batch, &frozen, routing_costs, progress);
                    routing_costs.set_detailed_timing_nets(BTreeSet::new());
                    if let Ok(candidate) = trial {
                        let wns_gain = candidate
                            .1
                            .worst_slack_ps
                            .zip(timing.worst_slack_ps)
                            .map_or(0, |(candidate, incumbent)| candidate - incumbent);
                        let improves_objective = improves_setup_objective(
                            prefer_fmax,
                            &candidate.0,
                            &candidate.1,
                            implementation,
                            timing,
                        );
                        let accepts_batch = improves_objective
                            && (prefer_fmax || wns_gain >= LOCAL_CRITICAL_BATCH_MIN_WNS_GAIN_PS);
                        progress(Ecp5FlowStage::TimingTrialDecision {
                            improves_objective: accepts_batch,
                        });
                        if profile {
                            eprintln!(
                                "[metrics] critical_batch distance={max_move_distance} moves={} wns_gain={wns_gain} accepted={accepts_batch}",
                                batch_moves.len(),
                            );
                        }
                        if accepts_batch {
                            return Ok(vec![candidate]);
                        }
                    }
                    // A rejected speculative checkpoint must not suppress the
                    // established per-cell fallback when it later reaches the
                    // same cumulative placement through routed feedback.
                    self.critical_move_trials.remove(&trial_key);
                }
            }
        }
        for (cell, (_, connections, targets)) in cells.into_iter().take(MAX_CRITICAL_PATH_CELLS) {
            let proposal_started = Instant::now();
            let proposals = placement_refiner.refine_cell_connection_delays_with_cache(
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
                &mut local_delay_workspace,
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
            let proposal_elapsed = proposal_started.elapsed();
            proposal_time += proposal_elapsed;
            if profile && proposal_elapsed >= Duration::from_millis(20) {
                eprintln!(
                    "[metrics] critical_proposals distance={max_move_distance} cell={} connections={} elapsed={proposal_elapsed:?} candidates={}",
                    cell.0,
                    connections.len(),
                    proposals.len(),
                );
            }
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
                let rebase_topology = rolling_moves >= MAX_ROLLING_CRITICAL_MOVES;
                let route_incumbent = if rebase_topology {
                    implementation
                } else {
                    &rolling_implementation
                };
                let mut recomputed_global_routing;
                let base = if let Some(incumbent) = incumbent_global_routing.as_ref()
                    && global_clock_endpoints_unchanged(
                        self.design,
                        self.packing,
                        &route_incumbent.placement,
                        &refined,
                    ) {
                    recomputed_global_routing = self.packing.routing_restrictions_cached(
                        self.design,
                        self.architecture,
                        &refined,
                        self.global_routing_cache,
                    )?;
                    for route in incumbent.routes().values() {
                        recomputed_global_routing.add_route(route.clone());
                    }
                    &recomputed_global_routing
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
                    route_incumbent,
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
                    let improves_objective = improves_setup_objective(
                        prefer_fmax,
                        &candidate.0,
                        &candidate.1,
                        implementation,
                        timing,
                    );
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
                    if best_for_cell
                        .as_ref()
                        .is_none_or(|(best_implementation, best_timing)| {
                            improves_setup_objective(
                                prefer_fmax,
                                &candidate.0,
                                &candidate.1,
                                best_implementation,
                                best_timing,
                            )
                        })
                    {
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
                && improves_setup_objective(
                    prefer_fmax,
                    &best_implementation,
                    &best_timing,
                    implementation,
                    timing,
                )
            {
                placement = best_implementation.placement.clone();
                rolling_implementation = best_implementation;
                rolling_moves = if rolling_moves >= MAX_ROLLING_CRITICAL_MOVES {
                    0
                } else {
                    rolling_moves + 1
                };
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
    context: &TimingDrivenContext<'_, '_, '_>,
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
const MAX_CONSECUTIVE_MONOTONIC_WNS_REGRESSIONS: usize = 2;
const LOCAL_TRIAL_ROUTING_ITERATIONS: u32 = 5;
const MAX_HOLD_ROUTE_FEEDBACKS: usize = 2;
const LOCAL_SETUP_ECO_TRIGGER_PS: i128 = 12;
const LOCAL_SETUP_REPAIR_QUANTA_PS: [u64; 4] = [50, 50, 10, 1];
const MAX_GENERAL_HOLD_REPAIRS: usize = 6;
const MAX_HOLD_SETUP_RECOVERY_PS: i128 = 100;
const MAX_DEDICATED_EDGE_TRIALS: usize = 1;
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
// Preserve the aggregate-objective search seed and the best achieved WNS.
// Refinement still descends from the aggregate winner, so retaining the Fmax
// candidate does not perturb the established search trajectory.
const TIMING_FRONTIER_WIDTH: usize = 2;
const REFINED_PLACEMENT_UNIT_LIMITS: [usize; 4] = [256, 128, 64, 32];
const DETAILED_ROUTING_QUANTA_PS: [u64; 1] = [10];
// The second 1 ps quantum measured as a pure extra full renegotiation on the
// AXI4 self-test: final WNS and placement were bit-identical without it while
// each multiresolution round paid one more ~30 s global ripup.
const MAX_CRITICAL_PATH_CELLS: usize = 6;
// Retain one accepted local route in place, then rebase the next move onto the
// pass seed so collateral nets periodically regain coarse topology freedom.
const MAX_ROLLING_CRITICAL_MOVES: usize = 1;
const LOCAL_CRITICAL_BATCH_SIZE: usize = 2;
const LOCAL_CRITICAL_BATCH_MIN_WNS_GAIN_PS: i128 = 32;
const LOCAL_BATCH_RECOVERY_WNS_PS: i128 = -800;
const LOCAL_BATCH_NEAR_CLOSURE_WNS_PS: i128 = -400;
const MAX_PROJECTED_PATH_CELL_CANDIDATES: usize = 4;
const MAX_ANCHORED_PLACEMENT_ROUNDS: u32 = 4;
const MAX_CRITICAL_CLOSURE_ROUNDS: usize = 4;
const MAX_FMAX_CLOSURE_ROUNDS: usize = 4;
const MAX_CRITICAL_PATH_VERTEX_REFINEMENTS: usize = 4;
// Basin-escape budget for designs that stall with negative setup slack after
// every refinement phase. Kicks re-solve the analytical placement with the
// incumbent's criticality weights amplified by a fixed power, which lands in a
// different deterministic basin; no randomness or recorded seed is involved.
const MAX_BASIN_ESCAPE_ROUNDS: usize = 2;
const BASIN_ESCAPE_WEIGHT_EXPONENT: u32 = 4;
const SPECULATIVE_FULL_ROUTING_ITERATIONS: u32 = 8;
// Start with cheap local legalization, then let only an internal vertex of the
// actual worst path escape a bad placement basin.  The broad pass is still a
// deterministic exhaustive choice over that one unit's legal BEL assignments;
// it is not a random restart or a whole-design perturbation.
const CRITICAL_PATH_MOVE_DISTANCES: [u64; 3] = [1, 2, 16];
const MAX_RELEASED_CRITICAL_NETS: usize = 64;

fn next_wns_regression_streak(
    current: usize,
    parent_wns_ps: Option<i128>,
    child_wns_ps: Option<i128>,
) -> usize {
    if child_wns_ps
        .zip(parent_wns_ps)
        .is_some_and(|(child, parent)| child <= parent)
    {
        current + 1
    } else {
        0
    }
}

fn ecp5_routing_costs(
    architecture: &Ecp5Architecture,
    speed_grade: &SpeedGradeRecord,
    net_criticalities: BTreeMap<NetId, u64>,
) -> Result<RoutingCosts, Ecp5FlowError> {
    let mut class_delays = vec![None; architecture.metadata_string_count()];
    for (index, resolved) in class_delays.iter_mut().enumerate() {
        let id = u32::try_from(index).expect("architecture metadata IDs fit u32");
        let Some(name) = architecture.metadata_string_by_id(id) else {
            continue;
        };
        let Some(class) = speed_grade.pip_classes.get(name) else {
            continue;
        };
        let delay = pip_class_delay(class, 1)?;
        let minimum =
            u32::try_from(delay.min_ps).map_err(|_| Ecp5FlowError::TimingDelayOverflow)?;
        let maximum =
            u32::try_from(delay.max_ps).map_err(|_| Ecp5FlowError::TimingDelayOverflow)?;
        *resolved = Some((minimum, maximum));
    }
    let mut pip_min_delays_ps = Vec::with_capacity(architecture.device().pips().len());
    let mut pip_delays_ps = Vec::with_capacity(architecture.device().pips().len());
    for class_id in architecture.pip_timing_class_ids() {
        let (minimum, maximum) = class_delays[class_id as usize].ok_or_else(|| {
            Ecp5FlowError::MissingPipTimingClass {
                speed_grade: speed_grade.name.clone(),
                timing_class: architecture
                    .metadata_string_by_id(class_id)
                    .unwrap_or("<invalid metadata ID>")
                    .to_owned(),
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
            "[metrics] stage={stage} hpwl={hpwl} wns={:?} hold={:?} pips={} timing_endpoints_checked={} timing_endpoints_modeled={}",
            timing.worst_slack_ps,
            timing.worst_hold_slack_ps,
            0,
            timing.setup_checks.len(),
            timing.modeled_endpoint_count(),
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

fn timing_placement_weights_with_exponent(
    timing: &TimingReport,
    constraints: &TimingConstraints,
    exponent: u32,
) -> BTreeMap<(NetId, CellPinId), u64> {
    timing_placement_weights(timing, constraints)
        .into_iter()
        .map(|(sink, weight)| (sink, exponentiate_placement_weight(weight, exponent)))
        .collect()
}

fn exponentiate_placement_weight(weight: u64, exponent: u32) -> u64 {
    weight.saturating_pow(exponent.max(1))
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

fn accumulate_hold_minimums(
    accumulated: &mut BTreeMap<(NetId, CellPinId), u64>,
    new_minimums: BTreeMap<(NetId, CellPinId), u64>,
) {
    for (sink, minimum) in new_minimums {
        accumulated
            .entry(sink)
            .and_modify(|known| *known = (*known).max(minimum))
            .or_insert(minimum);
    }
}

fn freeze_route_sinks_except(
    design: &Design,
    device: &Device,
    placement: &Placement,
    routes: &[Arc<NetRoute>],
    base: &RoutingConstraints,
    released: &BTreeSet<(NetId, CellPinId)>,
) -> Result<RoutingConstraints, PnrError> {
    let mut frozen = base.clone();
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
    let rebound = (0..design.pins().len())
        .map(CellPinId)
        .filter(|&pin| implementation.placement.pin_binding(pin) != placement.pin_binding(pin))
        .collect::<BTreeSet<_>>();
    let mut frozen = base.clone();
    for route in &implementation.routes {
        if base.routes().contains_key(&route.net) {
            continue;
        }
        let net = &design.nets()[route.net.0];
        if moved.contains(&design.pins()[net.driver.0].cell) || rebound.contains(&net.driver) {
            continue;
        }
        let mut released_sinks = route
            .arcs
            .iter()
            .filter_map(|arc| {
                arc.sink.filter(|&sink| {
                    released.contains(&(route.net, sink))
                        || moved.contains(&design.pins()[sink.0].cell)
                        || rebound.contains(&sink)
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

/// Orders setup-failing candidates for critical-path closure.
///
/// The broad placement/routing objective intentionally balances WNS and TNS,
/// but a critical-path walk must be allowed to cross a small TNS regression
/// when it shortens the worst path. Otherwise a better-Fmax candidate is kept
/// in the archive yet can never become the seed for the next critical move.
fn setup_closure_score(
    implementation: &PnrResult,
    timing: &TimingReport,
) -> (i128, TimingScore, Reverse<usize>) {
    setup_closure_key(
        timing.worst_slack_ps.unwrap_or(i128::MIN),
        timing_score(timing),
        implementation.total_pips,
    )
}

fn improves_setup_objective(
    prefer_fmax: bool,
    candidate_implementation: &PnrResult,
    candidate_timing: &TimingReport,
    incumbent_implementation: &PnrResult,
    incumbent_timing: &TimingReport,
) -> bool {
    if prefer_fmax {
        setup_closure_score(candidate_implementation, candidate_timing)
            > setup_closure_score(incumbent_implementation, incumbent_timing)
    } else {
        timing_score(candidate_timing) > timing_score(incumbent_timing)
    }
}

fn setup_closure_key(
    worst_slack_ps: i128,
    timing_score: TimingScore,
    total_pips: usize,
) -> (i128, TimingScore, Reverse<usize>) {
    (worst_slack_ps, timing_score, Reverse(total_pips))
}

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
    mut constraints: RoutingConstraints,
) -> Option<RoutingConstraints> {
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
    // A candidate can win both the aggregate and Fmax objectives. Keeping
    // both clones would consume the two-entry frontier and evict the other
    // physical trajectory that the archive exists to preserve.
    candidates.dedup();
    if width > 1 && candidates.len() > width {
        let best_fmax = candidates
            .iter()
            .enumerate()
            .max_by_key(|(_, (implementation, timing))| {
                (
                    timing.worst_slack_ps,
                    timing_score(timing),
                    Reverse(implementation.total_pips),
                )
            })
            .map(|(index, _)| index)
            .expect("the timing archive is non-empty");
        if best_fmax >= width {
            let candidate = candidates.remove(best_fmax);
            candidates.truncate(width - 1);
            candidates.push(candidate);
            return candidates;
        }
    }
    candidates.truncate(width);
    candidates
}

fn ecp5_timing_constraints(
    design: &Design,
    packing: &Ecp5Packing,
) -> Result<TimingConstraints, Ecp5FlowError> {
    // Project Trellis timing is an empirical, reverse-engineered model rather
    // than a sign-off speed file. A routed LFE5UM5G-85F design measured on
    // hardware failed with 152 ps of reported setup slack at 124 MHz and
    // passed with 418 ps at 120 MHz. Reserve 250 ps inside that measured
    // transition interval so marginal routes are repaired instead of being
    // reported as closed.
    const ECP5_SETUP_UNCERTAINTY_PS: u64 = 250;
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
    for (&net, &period_ps) in packing.generated_clock_periods_ps() {
        insert_clock_period(&mut constraints, net, period_ps)?;
    }
    for clock in packing.global_clocks() {
        if let Some(&period_ps) = constraints.clock_periods_ps().get(&clock.source_net) {
            insert_clock_period(&mut constraints, clock.global_net, period_ps)?;
        }
    }
    for net in constraints
        .clock_periods_ps()
        .keys()
        .copied()
        .collect::<Vec<_>>()
    {
        constraints.set_setup_uncertainty_ps(net, ECP5_SETUP_UNCERTAINTY_PS);
    }
    Ok(constraints)
}

fn constrain_pll_outputs(
    design: &Design,
    metadata: &BTreeMap<CellId, PrimitiveMetadata>,
    packing: &mut Ecp5Packing,
) -> Result<(), Ecp5FlowError> {
    for (&cell, primitive) in metadata {
        let PrimitiveMetadata::Pll {
            fabric_output,
            attributes,
            ..
        } = primitive
        else {
            continue;
        };
        let attribute = format!("FREQUENCY_PIN_{}", fabric_output.port());
        let frequency_mhz =
            attributes
                .get(&attribute)
                .ok_or_else(|| Ecp5FlowError::MissingPllOutputFrequency {
                    cell: design.cells()[cell.0].name.clone(),
                    attribute: attribute.clone(),
                })?;
        let period_ps = decimal_mhz_period_ps(frequency_mhz).ok_or_else(|| {
            Ecp5FlowError::InvalidPllOutputFrequency {
                cell: design.cells()[cell.0].name.clone(),
                attribute,
                value: frequency_mhz.clone(),
            }
        })?;
        let pin = find_cell_pin(design, cell, fabric_output.port()).ok_or_else(|| {
            Ecp5FlowError::MissingPllOutputPin {
                cell: design.cells()[cell.0].name.clone(),
                pin: fabric_output.port().into(),
            }
        })?;
        let net = design.pins()[pin.0]
            .net()
            .ok_or_else(|| Ecp5FlowError::MissingPllOutputNet {
                cell: design.cells()[cell.0].name.clone(),
                pin: fabric_output.port().into(),
            })?;
        if let Some(previous) = packing.set_generated_clock_period_ps(net, period_ps)
            && previous != period_ps
        {
            return Err(Ecp5FlowError::ConflictingClockPeriods { net });
        }
    }
    Ok(())
}

fn decimal_mhz_period_ps(value: &str) -> Option<u64> {
    let (whole, fraction) = value.split_once('.').unwrap_or((value, ""));
    if whole.is_empty()
        || !whole.bytes().all(|byte| byte.is_ascii_digit())
        || !fraction.bytes().all(|byte| byte.is_ascii_digit())
        || value.matches('.').count() > 1
    {
        return None;
    }
    let scale = 10_u128.checked_pow(u32::try_from(fraction.len()).ok()?)?;
    let whole = whole.parse::<u128>().ok()?;
    let fraction = if fraction.is_empty() {
        0
    } else {
        fraction.parse::<u128>().ok()?
    };
    let scaled_mhz = whole.checked_mul(scale)?.checked_add(fraction)?;
    let numerator = 1_000_000_u128.checked_mul(scale)?;
    let period_ps = numerator
        .checked_add(scaled_mhz / 2)?
        .checked_div(scaled_mhz)?;
    u64::try_from(period_ps).ok().filter(|period| *period != 0)
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
    constant_cells: &BTreeSet<CellId>,
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
    let constant_nets = design
        .nets()
        .iter()
        .enumerate()
        .filter_map(|(index, net)| {
            constant_cells
                .contains(&design.pins()[net.driver.0].cell)
                .then_some(NetId(index))
        })
        .collect::<BTreeSet<_>>();
    let mut model = TimingModel::new();
    for (index, cell) in design.cells().iter().enumerate() {
        let cell_id = CellId(index);
        let cell_type = if let Some(&cell_type) = carry_slices.get(&cell_id) {
            cell_type
        } else {
            match cell.kind {
                ResourceKind::Lut(4) => "TRELLIS_COMB",
                ResourceKind::Register => "TRELLIS_FF",
                ResourceKind::Memory => "DP16KD",
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
            if (cell.kind == ResourceKind::Register && arc.from_pin == "CLK" && arc.to_pin == "Q")
                || (cell.kind == ResourceKind::Memory
                    && matches!(arc.from_pin.as_str(), "CLKA" | "CLKB"))
            {
                model.add_clock_to_q(from, to, delay)?;
            } else {
                model.add_cell_arc(from, to, delay)?;
            }
        }
        add_wide_lut_timing_arcs(&mut model, design, cell_id, &records, &speed_grade.name)?;
        add_setup_hold_timing(
            &mut model,
            design,
            cell_id,
            record,
            general_routing_ffs.contains(&cell_id),
            &constant_nets,
        )?;
    }
    Ok(model)
}

fn add_setup_hold_timing(
    model: &mut TimingModel,
    design: &Design,
    cell: CellId,
    record: &texo_target_ecp5::CellTimingRecord,
    using_general_routing: bool,
    constant_nets: &BTreeSet<NetId>,
) -> Result<(), Ecp5FlowError> {
    for check in &record.setup_holds {
        // ECP5 LSR is an asynchronous set/reset input. Its characterized
        // recovery/removal values must not become synchronous data setup/hold
        // checks or constrain the register-to-register Fmax.
        if check.signal_pin == "LSR"
            || (check.signal_pin == "DI" && using_general_routing)
            || (check.signal_pin == "M" && !using_general_routing)
        {
            continue;
        }
        let logical_signal = if check.signal_pin == "M" {
            "DI"
        } else {
            &check.signal_pin
        };
        let Some(signal) = find_cell_pin(design, cell, logical_signal) else {
            continue;
        };
        // A directly driven constant cannot launch a transition, so it is not
        // a setup/hold endpoint. Keeping these checks made the hold repairer
        // search enormous constant fanout nets for meaningless delay.
        if design.pins()[signal.0]
            .net()
            .is_some_and(|net| constant_nets.contains(&net))
        {
            continue;
        }
        let Some(clock) = find_cell_pin(design, cell, &check.clock_pin) else {
            continue;
        };
        model.add_setup_hold(
            clock,
            signal,
            timing_delay(check.setup)?,
            timing_delay(check.hold)?,
        )?;
    }
    Ok(())
}

fn add_wide_lut_timing_arcs(
    model: &mut TimingModel,
    design: &Design,
    cell: CellId,
    records: &BTreeMap<&str, &texo_target_ecp5::CellTimingRecord>,
    speed_grade: &str,
) -> Result<(), Ecp5FlowError> {
    if find_cell_pin(design, cell, "OFX").is_none() {
        return Ok(());
    }
    let cell_type = if find_cell_pin(design, cell, "F1").is_some() {
        "TRELLIS_PFUMX"
    } else {
        "TRELLIS_L6MUX21"
    };
    let record =
        records
            .get(cell_type)
            .copied()
            .ok_or_else(|| Ecp5FlowError::MissingCellTiming {
                speed_grade: speed_grade.into(),
                cell_type: cell_type.into(),
            })?;
    for arc in &record.arcs {
        let Some(from) = find_cell_pin(design, cell, &arc.from_pin) else {
            continue;
        };
        let Some(to) = find_cell_pin(design, cell, &arc.to_pin) else {
            continue;
        };
        model.add_cell_arc(from, to, timing_delay(arc.delay)?)?;
    }
    Ok(())
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
    /// The selected PLL output has no generated-clock frequency attribute.
    MissingPllOutputFrequency {
        /// Logical PLL cell name.
        cell: String,
        /// Required frequency attribute.
        attribute: String,
    },
    /// A PLL frequency attribute is not a positive decimal MHz value.
    InvalidPllOutputFrequency {
        /// Logical PLL cell name.
        cell: String,
        /// Attribute containing the invalid value.
        attribute: String,
        /// Invalid attribute value.
        value: String,
    },
    /// An imported PLL is missing its selected output pin.
    MissingPllOutputPin {
        /// Logical PLL cell name.
        cell: String,
        /// Expected primitive output pin.
        pin: String,
    },
    /// An imported PLL output pin does not drive a logical net.
    MissingPllOutputNet {
        /// Logical PLL cell name.
        cell: String,
        /// Expected primitive output pin.
        pin: String,
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
            Self::InvalidPllOutputFrequency {
                cell,
                attribute,
                value,
            } => write!(
                f,
                "PLL `{cell}` attribute `{attribute}` has invalid MHz value `{value}`"
            ),
            Self::MissingPllOutputFrequency { cell, attribute } => write!(
                f,
                "PLL `{cell}` requires generated-clock attribute `{attribute}`"
            ),
            Self::MissingPllOutputPin { cell, pin } => {
                write!(f, "PLL `{cell}` has no selected output pin `{pin}`")
            }
            Self::MissingPllOutputNet { cell, pin } => {
                write!(f, "PLL `{cell}` output pin `{pin}` does not drive a net")
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
            | Self::ConflictingClockPeriods { .. }
            | Self::MissingPllOutputFrequency { .. }
            | Self::InvalidPllOutputFrequency { .. }
            | Self::MissingPllOutputPin { .. }
            | Self::MissingPllOutputNet { .. } => None,
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
    use std::collections::{BTreeMap, BTreeSet};
    use std::num::NonZeroU32;
    use std::sync::Arc;

    use struo_celox::ecp5_simulator;
    use struo_ir::{ActiveLevel, ClockEdge, Netlist, RegisterCell, ResetControl};
    use struo_target_ecp5::{
        Ecp5Netlist, IoTimingConstraints, MappingOptions, OpenDrainIo, map_to_ecp5,
        map_to_ecp5_with_constraints, map_to_ecp5_with_open_drain_ios,
    };
    use texo_model::{
        BelId, CellId, Design, Device, NetId, PinDirection, PipId, Point, ResourceKind,
    };
    use texo_pnr::{
        NetRoute, PlacementConstraints, PnrResult, RouteArc, RoutingConstraints, place_and_route,
        placement_from_partial_bindings, rebind_placement_pins,
    };
    use texo_struo::{
        ActiveLevel as ImportedActiveLevel, ClockEdge as ImportedClockEdge, PrimitiveMetadata,
        import_ecp5,
    };
    use texo_target_ecp5::{
        ArchitectureFile, Ecp5Packing, PipClassTimingRecord, PipRecord, RelativeRef,
        TimingCornersRecord, expand, find_global_clock_requirements, pack_lut_ffs,
        pack_lut_ffs_excluding, parse_lpf, read_architecture, resolve_lpf_port_cells,
    };
    use texo_timing::DelayRange;

    use super::{
        Ecp5FlowError, Ecp5FlowOptions, Evidence, Gate, PostMapSimulationPolicy,
        accumulate_hold_minimums, criticality_weight, decimal_mhz_period_ps,
        delay_weighted_criticality, ecp5_timing_constraints, ecp5_timing_model, ff_ce_control_sets,
        ff_clock_control_sets, find_cell_pin, freeze_route_sinks_except, freeze_unchanged_routes,
        implement, implement_struo_ecp5, implement_with_constraints, next_wns_regression_streak,
        pip_class_delay, project_trellis_speed_grade, retain_projection_timing_frontier,
        slack_violations, staged_timing_score, verify_post_map_with_celox,
    };

    const ECP5_FIXTURE: &str = include_str!("../../texo-target-ecp5/fixtures/minimal-ecp5.json");

    #[test]
    fn selects_the_5g_characterization_for_a_um5g_speed_8_part() {
        assert_eq!(project_trellis_speed_grade("LFE5UM5G-85F", "8"), "8_5G");
        assert_eq!(project_trellis_speed_grade("LFE5UM-85F", "8"), "8");
        assert_eq!(project_trellis_speed_grade("LFE5UM5G-85F", "7"), "7");
    }

    #[test]
    fn models_characterized_pfumx_and_l6mux21_arcs() {
        let mut source = Netlist::new("six_input_parity");
        let inputs = source.add_input_port("inputs", NonZeroU32::new(6).unwrap());
        let parity = inputs[1..]
            .iter()
            .fold(inputs[0], |value, input| source.add_xor(value, *input));
        source.add_output("result", parity);
        let constraints = IoTimingConstraints::new()
            .with_input_delay_ps("inputs", 0)
            .with_output_delay_ps("result", 0);
        let mapped = map_to_ecp5_with_constraints(
            &source,
            MappingOptions {
                timing_goal_mhz: 1_500,
                ..MappingOptions::default()
            },
            &constraints,
        )
        .unwrap();
        let imported = import_ecp5(&mapped).unwrap();
        let architecture = read_architecture(ECP5_FIXTURE.as_bytes()).unwrap();
        let packing = pack_lut_ffs_excluding(
            imported.design(),
            &architecture,
            imported.wide_lut_clusters().iter().flatten().copied(),
        )
        .unwrap();
        let speed_grade = &architecture.speed_grades()["6"];
        let model =
            ecp5_timing_model(imported.design(), &packing, speed_grade, &BTreeSet::new()).unwrap();
        let pfu = imported
            .design()
            .cells()
            .iter()
            .enumerate()
            .find_map(|(index, cell)| {
                cell.pins()
                    .iter()
                    .any(|pin| imported.design().pins()[pin.0].name == "F1")
                    .then_some(CellId(index))
            })
            .unwrap();
        let l6 = imported
            .design()
            .cells()
            .iter()
            .enumerate()
            .find_map(|(index, cell)| {
                cell.pins()
                    .iter()
                    .any(|pin| imported.design().pins()[pin.0].name == "FXA")
                    .then_some(CellId(index))
            })
            .unwrap();

        assert_eq!(
            model.cell_arc(
                find_cell_pin(imported.design(), pfu, "F1").unwrap(),
                find_cell_pin(imported.design(), pfu, "OFX").unwrap(),
            ),
            Some(DelayRange::from_independent_corners(68, 165))
        );
        assert_eq!(
            model.cell_arc(
                find_cell_pin(imported.design(), pfu, "M").unwrap(),
                find_cell_pin(imported.design(), pfu, "OFX").unwrap(),
            ),
            Some(DelayRange::from_independent_corners(187, 256))
        );
        assert_eq!(
            model.cell_arc(
                find_cell_pin(imported.design(), l6, "FXA").unwrap(),
                find_cell_pin(imported.design(), l6, "OFX").unwrap(),
            ),
            Some(DelayRange::from_independent_corners(189, 239))
        );
        assert_eq!(
            model.cell_arc(
                find_cell_pin(imported.design(), l6, "FXB").unwrap(),
                find_cell_pin(imported.design(), l6, "OFX").unwrap(),
            ),
            Some(DelayRange::from_independent_corners(189, 242))
        );
        assert_eq!(
            model.cell_arc(
                find_cell_pin(imported.design(), l6, "M").unwrap(),
                find_cell_pin(imported.design(), l6, "OFX").unwrap(),
            ),
            Some(DelayRange::from_independent_corners(186, 252))
        );
    }

    #[test]
    fn ff_ce_control_sets_preserve_compatible_slice_placements() {
        let mut design = Design::new();
        let first_driver = design.add_cell("first_ce", ResourceKind::Logic);
        let first_ce = design
            .add_pin(first_driver, "out", PinDirection::Output)
            .unwrap();
        let second_driver = design.add_cell("second_ce", ResourceKind::Logic);
        let second_ce = design
            .add_pin(second_driver, "out", PinDirection::Output)
            .unwrap();
        let always = design.add_cell("always", ResourceKind::Register);
        let high_a = design.add_cell("high_a", ResourceKind::Register);
        let shared_ce_first = design.add_pin(high_a, "CE", PinDirection::Input).unwrap();
        let high_b = design.add_cell("high_b", ResourceKind::Register);
        let shared_ce_second = design.add_pin(high_b, "CE", PinDirection::Input).unwrap();
        let low = design.add_cell("low", ResourceKind::Register);
        let low_ce = design.add_pin(low, "CE", PinDirection::Input).unwrap();
        let other = design.add_cell("other", ResourceKind::Register);
        let other_ce = design.add_pin(other, "CE", PinDirection::Input).unwrap();
        design
            .add_net(
                "shared_ce",
                first_ce,
                [shared_ce_first, shared_ce_second, low_ce],
            )
            .unwrap();
        design.add_net("other_ce", second_ce, [other_ce]).unwrap();
        let flip_flop = |enable| PrimitiveMetadata::FlipFlop {
            edge: ImportedClockEdge::Rising,
            enable,
            reset: None,
        };
        let metadata = BTreeMap::from([
            (always, flip_flop(None)),
            (high_a, flip_flop(Some(ImportedActiveLevel::High))),
            (high_b, flip_flop(Some(ImportedActiveLevel::High))),
            (low, flip_flop(Some(ImportedActiveLevel::Low))),
            (other, flip_flop(Some(ImportedActiveLevel::High))),
        ]);

        let sets = ff_ce_control_sets(&design, &metadata)
            .into_iter()
            .collect::<BTreeMap<_, _>>();

        assert_eq!(sets[&always], 0);
        assert_eq!(sets[&high_a], sets[&high_b]);
        assert_ne!(sets[&high_a], sets[&low]);
        assert_ne!(sets[&high_a], sets[&other]);
    }

    #[test]
    fn ff_clock_control_sets_separate_nets_and_edges() {
        let mut design = Design::new();
        let first_driver = design.add_cell("first_clock", ResourceKind::Logic);
        let first_clock = design
            .add_pin(first_driver, "out", PinDirection::Output)
            .unwrap();
        let second_driver = design.add_cell("second_clock", ResourceKind::Logic);
        let second_clock = design
            .add_pin(second_driver, "out", PinDirection::Output)
            .unwrap();
        let rising_a = design.add_cell("rising_a", ResourceKind::Register);
        let rising_b = design.add_cell("rising_b", ResourceKind::Register);
        let falling = design.add_cell("falling", ResourceKind::Register);
        let shared_clock_sinks = [rising_a, rising_b, falling]
            .map(|cell| design.add_pin(cell, "CLK", PinDirection::Input).unwrap());
        let other = design.add_cell("other", ResourceKind::Register);
        let other_clock = design.add_pin(other, "CLK", PinDirection::Input).unwrap();
        design
            .add_net("shared_clock", first_clock, shared_clock_sinks)
            .unwrap();
        design
            .add_net("other_clock", second_clock, [other_clock])
            .unwrap();
        let flip_flop = |edge| PrimitiveMetadata::FlipFlop {
            edge,
            enable: None,
            reset: None,
        };
        let metadata = BTreeMap::from([
            (rising_a, flip_flop(ImportedClockEdge::Rising)),
            (rising_b, flip_flop(ImportedClockEdge::Rising)),
            (falling, flip_flop(ImportedClockEdge::Falling)),
            (other, flip_flop(ImportedClockEdge::Rising)),
        ]);

        let sets = ff_clock_control_sets(&design, &metadata)
            .into_iter()
            .collect::<BTreeMap<_, _>>();

        assert_eq!(sets[&rising_a], sets[&rising_b]);
        assert_ne!(sets[&rising_a], sets[&falling]);
        assert_ne!(sets[&rising_a], sets[&other]);
    }

    #[test]
    fn converts_decimal_pll_frequencies_to_picosecond_periods() {
        assert_eq!(decimal_mhz_period_ps("250"), Some(4_000));
        assert_eq!(decimal_mhz_period_ps("125.0"), Some(8_000));
        assert_eq!(decimal_mhz_period_ps("12.5"), Some(80_000));
        assert_eq!(decimal_mhz_period_ps("0"), None);
        assert_eq!(decimal_mhz_period_ps("250MHz"), None);
    }

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
    fn placement_weight_exponent_controls_analytical_weights() {
        assert_eq!(super::exponentiate_placement_weight(8, 1), 8);
        assert_eq!(super::exponentiate_placement_weight(8, 2), 64);
        assert_eq!(super::exponentiate_placement_weight(8, 0), 8);
    }

    #[test]
    fn monotonic_hierarchy_transitions_after_consecutive_wns_regressions() {
        assert_eq!(next_wns_regression_streak(0, Some(-100), Some(-80)), 0);
        assert_eq!(next_wns_regression_streak(0, Some(-80), Some(-90)), 1);
        assert_eq!(next_wns_regression_streak(1, Some(-90), Some(-90)), 2);
        assert_eq!(next_wns_regression_streak(2, Some(-90), Some(-70)), 0);
        assert_eq!(next_wns_regression_streak(1, None, Some(-70)), 0);
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
    fn critical_path_closure_can_follow_a_wns_gain_across_a_tns_regression() {
        let aggregate_winner = slack_violations([-100, 0].into_iter()).score();
        let fmax_winner = slack_violations(std::iter::repeat_n(-99, 100)).score();
        let aggregate_score = staged_timing_score(aggregate_winner, aggregate_winner, -100, 0);
        let fmax_score = staged_timing_score(fmax_winner, fmax_winner, -99, 0);

        assert!(aggregate_score > fmax_score);
        assert!(
            super::setup_closure_key(-99, fmax_score, 110)
                > super::setup_closure_key(-100, aggregate_score, 100)
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
    fn hold_feedback_retains_and_strengthens_prior_sink_floors() {
        let first = (NetId(1), texo_model::CellPinId(10));
        let second = (NetId(2), texo_model::CellPinId(20));
        let mut accumulated = BTreeMap::from([(first, 120)]);

        accumulate_hold_minimums(
            &mut accumulated,
            BTreeMap::from([(first, 100), (second, 80)]),
        );
        assert_eq!(accumulated, BTreeMap::from([(first, 120), (second, 80)]));

        accumulate_hold_minimums(&mut accumulated, BTreeMap::from([(first, 140)]));
        assert_eq!(accumulated, BTreeMap::from([(first, 140), (second, 80)]));
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
        let mut base = RoutingConstraints::new();
        base.block_pips([PipId(0)]);

        let frozen = freeze_route_sinks_except(
            &design,
            &device,
            &implementation.placement,
            &implementation.routes,
            &base,
            &BTreeSet::from([(NetId(1), second_input)]),
        )
        .unwrap();

        assert!(frozen.routes().contains_key(&NetId(0)));
        assert!(!frozen.routes().contains_key(&NetId(1)));
        assert_eq!(frozen.blocked_pips(), &BTreeSet::from([PipId(0)]));
    }

    #[test]
    fn hold_repair_does_not_freeze_a_route_to_a_rebound_physical_pin() {
        let mut design = Design::new();
        let source = design.add_cell("source", ResourceKind::Logic);
        let output = design.add_pin(source, "out", PinDirection::Output).unwrap();
        let register = design.add_cell("register", ResourceKind::Register);
        let input = design.add_pin(register, "DI", PinDirection::Input).unwrap();
        let net = design.add_net("data", output, [input]).unwrap();

        let mut device = Device::new("rebound-pin", 1, 1).unwrap();
        let point = Point::new(0, 0);
        let driver_wire = device.add_wire("driver", point, 1).unwrap();
        let di_wire = device.add_wire("DI", point, 1).unwrap();
        let m_wire = device.add_wire("M", point, 1).unwrap();
        let source_bel = device
            .add_bel("source", ResourceKind::Logic, point)
            .unwrap();
        device
            .add_bel_pin(source_bel, "out", PinDirection::Output, driver_wire)
            .unwrap();
        let register_bel = device
            .add_bel("register", ResourceKind::Register, point)
            .unwrap();
        let di_pin = device
            .add_bel_pin(register_bel, "DI", PinDirection::Input, di_wire)
            .unwrap();
        let m_pin = device
            .add_bel_pin(register_bel, "M", PinDirection::Input, m_wire)
            .unwrap();
        let dedicated_pip = device.add_pip(driver_wire, di_wire, false, 1).unwrap();
        device.add_pip(driver_wire, m_wire, false, 1).unwrap();
        let bindings = BTreeMap::from([(source, source_bel), (register, register_bel)]);
        let mut dedicated = PlacementConstraints::new();
        dedicated.bind_pin(input, register_bel, di_pin);
        let original =
            placement_from_partial_bindings(&design, &device, &dedicated, &bindings).unwrap();
        let mut general = PlacementConstraints::new();
        general.bind_pin(input, register_bel, m_pin);
        let rebound = rebind_placement_pins(&design, &device, &general, &original).unwrap();
        let implementation = PnrResult {
            placement: original,
            routes: vec![Arc::new(NetRoute::new(
                net,
                vec![RouteArc {
                    sink: Some(input),
                    wires: vec![driver_wire, di_wire],
                    pips: vec![dedicated_pip],
                }],
            ))],
            total_pips: 1,
        };

        let frozen = freeze_unchanged_routes(
            &design,
            &implementation,
            &rebound,
            &RoutingConstraints::new(),
            &BTreeSet::new(),
        );

        assert_eq!(
            implementation.placement.bel(register),
            rebound.bel(register)
        );
        assert_eq!(implementation.placement.pin_binding(input), Some(di_pin));
        assert_eq!(rebound.pin_binding(input), Some(m_pin));
        assert!(!frozen.routes().contains_key(&net));
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

    fn open_drain_test_architecture() -> texo_target_ecp5::Ecp5Architecture {
        let mut file: ArchitectureFile = serde_json::from_str(ECP5_FIXTURE).unwrap();
        for (from, to) in [(4, 7), (4, 8), (35, 7), (35, 8)] {
            file.location_types[0].pips.push(PipRecord {
                from: RelativeRef {
                    dx: 0,
                    dy: 0,
                    index: from,
                },
                to: RelativeRef {
                    dx: 0,
                    dy: 0,
                    index: to,
                },
                fixed: false,
                tile_type: "PLC2".into(),
                timing_class: "default".into(),
                lutperm_flags: 0,
            });
        }
        for to in 22..=25 {
            file.location_types[1].pips.push(PipRecord {
                from: RelativeRef {
                    dx: 0,
                    dy: 0,
                    index: 13,
                },
                to: RelativeRef {
                    dx: 0,
                    dy: 0,
                    index: to,
                },
                fixed: false,
                tile_type: "PLC2".into(),
                timing_class: "default".into(),
                lutperm_flags: 0,
            });
        }
        for (from, to) in [(4, 7), (4, 8), (26, 7), (26, 8)] {
            file.location_types[1].pips.push(PipRecord {
                from: RelativeRef {
                    dx: 0,
                    dy: 0,
                    index: from,
                },
                to: RelativeRef {
                    dx: -1,
                    dy: 0,
                    index: to,
                },
                fixed: false,
                tile_type: "PLC2".into(),
                timing_class: "default".into(),
                lutperm_flags: 0,
            });
        }
        expand(file).unwrap()
    }

    #[test]
    fn routes_one_open_drain_pad_through_all_three_pio_pins() {
        let mut source = Netlist::new("open_drain");
        let sda_i = source.add_input("sda_i");
        let drive_low = source.add_input("drive_low");
        source.add_output("sda_drive_low", drive_low);
        let sampled = source.add_not(sda_i);
        source.add_output("sampled", sampled);
        let mapped = map_to_ecp5_with_open_drain_ios(
            &source,
            &[OpenDrainIo::new("sda", "sda_i", "sda_drive_low")],
        )
        .unwrap();
        let imported = import_ecp5(&mapped).unwrap();
        let architecture = open_drain_test_architecture();
        let lpf = parse_lpf(
            br"
                LOCATE COMP sda SITE A10;
                LOCATE COMP drive_low SITE B10;
                LOCATE COMP sampled SITE C10;
                IOBUF PORT sda IO_TYPE=LVCMOS33 PULLMODE=UP;
            "
            .as_slice(),
        )
        .unwrap();
        let mut evidence = Evidence::new();

        let result = implement_struo_ecp5(
            &imported,
            &architecture,
            Ecp5FlowOptions {
                post_map_simulation: PostMapSimulationPolicy::AllowMissing,
                speed_grade: Some("6"),
                package: Some("CABGA381"),
                lpf: Some(&lpf),
                optimize_timing: false,
                ..Ecp5FlowOptions::default()
            },
            &mut evidence,
        )
        .unwrap();

        let sda = imported
            .ports()
            .iter()
            .find(|port| port.name == "sda")
            .unwrap()
            .bits[0];
        let sda_bel = result.implementation.placement.bel(sda).unwrap();
        assert_eq!(architecture.device().bels()[sda_bel.0].name, "R0C0/PIOA");
        let routed_pin_names = result.design.cells()[sda.0]
            .pins()
            .iter()
            .filter(|pin| result.design.pins()[pin.0].net().is_some())
            .map(|pin| result.design.pins()[pin.0].name.as_str())
            .collect::<BTreeSet<_>>();
        assert_eq!(routed_pin_names, BTreeSet::from(["I", "O", "T"]));
        assert_eq!(result.packing.io_attributes()[&sda]["PULLMODE"], "UP");
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
    fn arbitrary_design_mode_does_not_invent_simulation_evidence() {
        let mapped = mapped_xor();
        let imported = import_ecp5(&mapped).unwrap();
        let architecture = read_architecture(ECP5_FIXTURE.as_bytes()).unwrap();
        let lpf = parse_lpf(
            br"
                LOCATE COMP lhs SITE A10;
                LOCATE COMP rhs SITE B10;
                LOCATE COMP value SITE C10;
            "
            .as_slice(),
        )
        .unwrap();
        let mut evidence = Evidence::new();

        implement_struo_ecp5(
            &imported,
            &architecture,
            Ecp5FlowOptions {
                post_map_simulation: PostMapSimulationPolicy::AllowMissing,
                speed_grade: Some("6"),
                package: Some("CABGA381"),
                lpf: Some(&lpf),
                ..Ecp5FlowOptions::default()
            },
            &mut evidence,
        )
        .unwrap();

        assert!(!evidence.contains(Gate::PostMapSimulation));
        assert!(evidence.contains(Gate::MappedNetlistComplete));
        assert!(evidence.contains(Gate::PhysicalImplementation));
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
        let timing_model = ecp5_timing_model(
            &design,
            &packing,
            &architecture.speed_grades()["6"],
            &BTreeSet::new(),
        )
        .unwrap();
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
        assert_eq!(constraints.setup_uncertainties_ps().len(), 2);
        assert!(
            constraints
                .setup_uncertainties_ps()
                .values()
                .all(|&uncertainty_ps| uncertainty_ps == 250)
        );
        assert_eq!(timing_model.clock_to_q(ff_q).unwrap().1.max_ps, 525);
        assert_eq!(timing_model.setup_hold(ff_data).unwrap().2.min_ps, 233);
        assert!(timing_model.setup_hold(ff_lsr).is_none());
    }

    #[test]
    fn omits_setup_and_hold_checks_for_constant_driven_register_inputs() {
        let mut source = Netlist::new("constant_register");
        let data = source.add_constant(false);
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
        source.add_output("state", state);

        let imported = import_ecp5(&map_to_ecp5(&source).unwrap()).unwrap();
        let architecture = read_architecture(ECP5_FIXTURE.as_bytes()).unwrap();
        let constant_cells = imported
            .metadata()
            .iter()
            .filter_map(|(&cell, metadata)| {
                matches!(metadata, PrimitiveMetadata::Constant { .. }).then_some(cell)
            })
            .collect::<BTreeSet<_>>();
        let packing = pack_lut_ffs_excluding(
            imported.design(),
            &architecture,
            constant_cells.iter().copied(),
        )
        .unwrap();
        let model = ecp5_timing_model(
            imported.design(),
            &packing,
            &architecture.speed_grades()["6"],
            &constant_cells,
        )
        .unwrap();
        let register = imported
            .design()
            .cells()
            .iter()
            .position(|cell| cell.kind == ResourceKind::Register)
            .map(CellId);

        assert_eq!(constant_cells.len(), 1);
        let Some(register) = register else {
            // Current Struo folds a constant-driven register during ECP5
            // mapping, before it can become a timing endpoint.
            return;
        };
        let data_pin = find_cell_pin(imported.design(), register, "DI").unwrap();

        assert!(model.setup_hold(data_pin).is_none());
    }

    #[test]
    fn applies_characterized_dp16kd_clock_to_output_and_input_checks() {
        let architecture = read_architecture(ECP5_FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let memory = design.add_cell("memory", ResourceKind::Memory);
        for name in ["CLKA", "CLKB", "DIA0", "ADB0"] {
            design.add_pin(memory, name, PinDirection::Input).unwrap();
        }
        design
            .add_pin(memory, "DOB0", PinDirection::Output)
            .unwrap();
        let model = ecp5_timing_model(
            &design,
            &Ecp5Packing::default(),
            &architecture.speed_grades()["6"],
            &BTreeSet::new(),
        )
        .unwrap();
        let clkb = find_cell_pin(&design, memory, "CLKB").unwrap();
        let output = find_cell_pin(&design, memory, "DOB0").unwrap();
        let write_data = find_cell_pin(&design, memory, "DIA0").unwrap();
        let read_address = find_cell_pin(&design, memory, "ADB0").unwrap();

        assert_eq!(model.clock_to_q(output).unwrap().0, clkb);
        assert_eq!(model.clock_to_q(output).unwrap().1.max_ps, 5830);
        assert_eq!(model.setup_hold(write_data).unwrap().1.max_ps, 220);
        assert_eq!(model.setup_hold(write_data).unwrap().2.max_ps, 43);
        assert_eq!(model.setup_hold(read_address).unwrap().0, clkb);
        assert_eq!(model.setup_hold(read_address).unwrap().1.max_ps, 251);
        assert_eq!(model.setup_hold(read_address).unwrap().2.max_ps, 123);
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
        let mut design = Design::new();
        let first = design.add_cell("carry0", ResourceKind::Lut(4));
        let first_a = design.add_pin(first, "A", PinDirection::Input).unwrap();
        let first_carry_in = design.add_pin(first, "FCI", PinDirection::Input).unwrap();
        let first_carry_out = design.add_pin(first, "FCO", PinDirection::Output).unwrap();
        let second = design.add_cell("carry1", ResourceKind::Lut(4));
        let second_carry_in = design.add_pin(second, "FCI", PinDirection::Input).unwrap();
        let second_f = design.add_pin(second, "F", PinDirection::Output).unwrap();
        design.add_pin(second, "FCO", PinDirection::Output).unwrap();
        design
            .add_net("internal_carry", first_carry_out, [second_carry_in])
            .unwrap();
        let architecture = read_architecture(ECP5_FIXTURE.as_bytes()).unwrap();
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();
        packing
            .pack_carry_pairs(&design, &architecture, [[first, second]])
            .unwrap();

        let model = ecp5_timing_model(
            &design,
            &packing,
            &architecture.speed_grades()["6"],
            &BTreeSet::new(),
        )
        .unwrap();
        let second_carry_out = find_cell_pin(&design, second, "FCO").unwrap();

        assert_eq!(
            model.cell_arc(first_a, first_carry_out).unwrap().max_ps,
            447
        );
        assert_eq!(
            model
                .cell_arc(first_carry_in, first_carry_out)
                .unwrap()
                .max_ps,
            71
        );
        assert_eq!(
            model.cell_arc(second_carry_in, second_f).unwrap().max_ps,
            403
        );
        assert_eq!(
            model
                .cell_arc(second_carry_in, second_carry_out)
                .unwrap()
                .max_ps,
            0
        );
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
