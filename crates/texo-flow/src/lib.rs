//! Flow orchestration and explicit verification evidence.

mod clock_constraints;
mod ecp5_pll;
mod timing_coverage;

pub use clock_constraints::ClockConstraint;
pub use timing_coverage::{TimingCoverageError, TimingEndpointException, validate_timing_coverage};

use std::cmp::Reverse;
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::error::Error;
use std::fmt;
use std::sync::Arc;
use std::time::Instant;

use texo_model::{
    BelId, BelPinId, CellId, CellPinId, Design, Device, NetId, PinDirection, PipId, ResourceKind,
    WireId,
};
pub use texo_pnr::RoutingProgress;
use texo_pnr::{
    LegalRouteEcoOptions, NetRoute, Placement, PlacementConstraints, PlacementRefinementWorkspace,
    PlacementRefiner, PnrError, PnrResult, RegisterControlSet, RoutingConstraints, RoutingCosts,
    RoutingWorkspace, legal_net_route_eco_candidate_with_workspace,
    legal_nets_route_eco_candidate_with_workspace, place_and_route_with_constraints,
    placement_from_complete_bindings, placement_from_partial_bindings, rebind_placement_pins,
    route_with_timing_costs_workspace_and_progress, route_with_workspace_and_progress,
    routing_capacity_map,
};
use texo_struo::{ActiveLevel, DistributedRamRole, ImportedEcp5Design, PrimitiveMetadata};
use texo_target_ecp5::{
    BlockRamRequirement, DEFAULT_GLOBAL_CLOCK_FANOUT, DelayRangeRecord, DistributedRamRequirement,
    Ecp5Architecture, Ecp5DelayPredictorError, Ecp5GlobalRoutingCache, Ecp5Packing,
    Ecp5PlacementDelayPredictor, FfControlSet, LpfConstraints, LpfError, LutFfPair, PackingError,
    PipClassTimingRecord, SpeedGradeRecord, find_global_clock_requirements, pack_lut_ffs_excluding,
    pack_lut_ffs_with_pairs, resolve_lpf_port_cells,
};
use texo_timing::{
    ClockEdge as TimingClockEdge, DelayRange, NetDelay, PICOSECONDS_PER_SECOND,
    TimingAnalysisSession, TimingConstraints, TimingError, TimingModel, TimingReport,
    analyze_timing, analyze_timing_from_net_delays,
};

use ecp5_pll::{GeneratedClockRelations, constrain_pll_outputs};

/// Stable checkpoint name for the ECP5 analytical-placement timing overlay.
pub const ECP5_PLACEMENT_TIMING_WEIGHT_MODEL: &str = "ecp5_1_plus_10_criticality_power_v1";
/// Stable checkpoint name for the ECP5 routability-driven area model.
pub const ECP5_PLACEMENT_ROUTABILITY_MODEL: &str = "directional_rudy_area_adjustment_v1";

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
    /// Modeled setup/hold constraints passed and unchecked endpoints were reviewed.
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

    fn record_timing(
        &mut self,
        design: &Design,
        timing: &TimingReport,
        exceptions: &[TimingEndpointException],
    ) {
        // A caller can reuse evidence from a previous implementation. Never
        // carry its timing gate into a run with failing or incomplete timing.
        self.passed.remove(&Gate::TimingClosure);
        if timing.met_timing()
            && timing_coverage::validate_report_coverage(design, timing, exceptions).is_ok()
        {
            self.record(Gate::TimingClosure);
        }
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
    /// Exact reviewed `no_synchronous_launch` exceptions, saved in the checkpoint.
    /// Unconstrained or unconnected capture clocks cannot be excepted.
    pub timing_exceptions: &'a [TimingEndpointException],
    /// Primary periods on exact mapped source pins, including internal clocks.
    pub clock_constraints: &'a [ClockConstraint],
    /// Setup margin reserved on every constrained capture clock, in picoseconds.
    /// This leaves nominal periods, generated-clock relations and hold intact.
    pub setup_uncertainty_ps: u64,
    /// Minimum recognized clock-pin fanout for automatic DCCA promotion.
    pub global_clock_fanout: usize,
    /// Exponent in the ECP5 analytical-placement connection weight
    /// `1 + 10 * criticality^exponent`. Larger values concentrate force on
    /// the most critical paths. Recorded in checkpoints so artifacts remain
    /// reproducible.
    pub placement_weight_exponent: u32,
    /// Optional cell-name to BEL-name bindings used instead of native initial
    /// placement. Missing synthetic cells are completed from packing groups.
    pub initial_placement: Option<&'a BTreeMap<String, String>>,
    /// Optional explicit dedicated-path LUT-to-FF pairs, named as
    /// `LUT -> FF`. This must accompany placements imported after packing.
    pub lut_ff_pairs: Option<&'a BTreeMap<String, String>>,
    /// Deprecated compatibility alias for selecting a timing-driven initial
    /// route when post-route timing optimization is otherwise disabled.
    ///
    /// Timing optimization now uses one characterized route from the start,
    /// so enabling this no longer adds a second bootstrap reroute.
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
            timing_exceptions: &[],
            clock_constraints: &[],
            setup_uncertainty_ps: 0,
            global_clock_fanout: DEFAULT_GLOBAL_CLOCK_FANOUT,
            placement_weight_exponent: 4,
            initial_placement: None,
            lut_ff_pairs: None,
            initial_timing_reroute: false,
            optimize_timing: true,
        }
    }
}

/// Initial placement algorithm used by one completed ECP5 flow.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Ecp5InitialPlacementAlgorithm {
    /// A complete caller-provided cell-to-BEL assignment was imported.
    Imported,
    /// Connectivity-only electrostatic placement generated the assignment.
    ConnectivityDrivenElectrostatic,
    /// ECP5 timing weights and directional RUDY area adjustment generated it.
    TimingDrivenRoutabilityElectrostatic,
}

impl Ecp5InitialPlacementAlgorithm {
    /// Stable name recorded in implementation checkpoints.
    #[must_use]
    pub const fn checkpoint_name(self) -> &'static str {
        match self {
            Self::Imported => "imported_v1",
            Self::ConnectivityDrivenElectrostatic => "connectivity_electrostatic_v1",
            Self::TimingDrivenRoutabilityElectrostatic => {
                "ecp5_timing_routability_electrostatic_v1"
            }
        }
    }

    /// Whether the algorithm applies ECP5 timing weights and RUDY area tuning.
    #[must_use]
    pub const fn is_timing_driven_routability(self) -> bool {
        matches!(self, Self::TimingDrivenRoutabilityElectrostatic)
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
    /// Exact caller-supplied exceptions retained for independent bitgen validation.
    pub timing_exceptions: Vec<TimingEndpointException>,
    /// Explicit source periods used before deriving PLL and global clocks.
    pub clock_constraints: Vec<ClockConstraint>,
    /// Uniform setup margin applied throughout placement, routing and repair.
    pub setup_uncertainty_ps: u64,
    /// Placement-weight exponent the flow was configured with.
    pub placement_weight_exponent: u32,
    /// Initial placement algorithm used to build this implementation.
    pub initial_placement_algorithm: Ecp5InitialPlacementAlgorithm,
}

impl Ecp5FlowResult {
    /// Validates that every omitted modeled endpoint has an exact reviewed exception.
    ///
    /// # Errors
    ///
    /// Reports missing clock constraints, unreviewed endpoints, or invalid exceptions.
    pub fn validate_timing_coverage(&self) -> Result<(), TimingCoverageError> {
        timing_coverage::validate_report_coverage(
            &self.design,
            &self.timing,
            &self.timing_exceptions,
        )
    }

    /// Whether modeled timing checks pass and all omitted endpoints are reviewed.
    ///
    /// This does not characterize primitives absent from the timing model.
    #[must_use]
    pub fn meets_timing_closure(&self) -> bool {
        self.timing.met_timing() && self.validate_timing_coverage().is_ok()
    }
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
    /// Legacy compatibility event for the removed critical-cell portfolio.
    /// The generic timing-feedback loop does not emit this variant.
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
    // Preserve the public API's historical zero-as-one behavior while
    // recording the exponent that was actually used.
    let placement_weight_exponent = options.placement_weight_exponent.max(1);
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
    let distributed_ram_cells = imported
        .distributed_ram_clusters()
        .iter()
        .flat_map(|cluster| {
            cluster
                .data
                .into_iter()
                .chain(cluster.blockers)
                .chain([cluster.write_port])
        })
        .collect::<BTreeSet<_>>();
    let explicit_pairs = options
        .lut_ff_pairs
        .map(|pairs| named_lut_ff_pairs(&design, pairs))
        .transpose()?;
    if let Some(pair) = explicit_pairs
        .as_deref()
        .and_then(|pairs| pairs.iter().find(|pair| wide_luts.contains(&pair.lut)))
    {
        return Err(PackingError::InvalidLutFfPair {
            lut: design.cells()[pair.lut.0].name.clone(),
            ff: design.cells()[pair.ff.0].name.clone(),
            reason: "wide-LUT members cannot use the ordinary LUT/FF path".into(),
        }
        .into());
    }
    if let Some(pair) = explicit_pairs.as_deref().and_then(|pairs| {
        pairs
            .iter()
            .find(|pair| distributed_ram_cells.contains(&pair.lut))
    }) {
        return Err(PackingError::InvalidLutFfPair {
            lut: design.cells()[pair.lut.0].name.clone(),
            ff: design.cells()[pair.ff.0].name.clone(),
            reason: "distributed-RAM members cannot use the ordinary LUT/FF path".into(),
        }
        .into());
    }
    let (ordinary_pairs, carry_pairs) = explicit_pairs.as_deref().map_or_else(
        || (None, None),
        |pairs| {
            let (carry, ordinary): (Vec<_>, Vec<_>) = pairs.iter().copied().partition(|pair| {
                matches!(
                    imported.metadata().get(&pair.lut),
                    Some(PrimitiveMetadata::CarrySlice { .. })
                )
            });
            (Some(ordinary), Some(carry))
        },
    );
    let mut packing = match ordinary_pairs {
        Some(pairs) => {
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
            constant_luts
                .iter()
                .chain(&wide_luts)
                .chain(&distributed_ram_cells)
                .copied(),
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
    packing.pack_distributed_rams(
        &design,
        architecture,
        imported
            .distributed_ram_clusters()
            .iter()
            .map(|cluster| DistributedRamRequirement {
                data: cluster.data,
                blockers: cluster.blockers,
                write_port: cluster.write_port,
            }),
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
    let ff_control_sets = ff_control_sets(&design, imported.metadata(), imported.absorbed_inputs());
    if let Some(carry_pairs) = carry_pairs {
        packing.pack_carry_lut_ffs_with_pairs(
            &design,
            architecture,
            ff_control_sets.iter().copied(),
            carry_pairs,
        )?;
    } else {
        packing.pack_carry_lut_ffs(&design, architecture, ff_control_sets.iter().copied())?;
    }
    packing.constrain_ff_control_sets(architecture, &ff_control_sets);
    clock_constraints::apply_clock_constraints(&design, &mut packing, options.clock_constraints)?;
    let pll_relations = constrain_pll_outputs(&design, imported.metadata(), &mut packing)?;
    progress(Ecp5FlowStage::Packed);
    report_metric_phase("packing", &mut phase_started);

    let mut placement_refinement_workspace = PlacementRefinementWorkspace::new();
    let timing_model = ecp5_timing_model(
        &design,
        &packing,
        speed_grade,
        &constant_luts,
        imported.metadata(),
    )?;
    let mut timing_constraints = ecp5_timing_constraints(&design, &packing, &pll_relations)?;
    apply_setup_uncertainty(&mut timing_constraints, options.setup_uncertainty_ps);
    let mut staged_evidence = evidence.clone();
    staged_evidence.record(Gate::MappedNetlistComplete);
    let use_timing_route = options.optimize_timing || options.initial_timing_reroute;
    let placement_delay_predictor = use_timing_route
        .then(|| Ecp5PlacementDelayPredictor::new(architecture, &speed_grade.name))
        .transpose()?;
    let mut global_routing_cache = architecture.global_routing_cache();
    let initial_placement_algorithm = if options.initial_placement.is_some() {
        Ecp5InitialPlacementAlgorithm::Imported
    } else if options.optimize_timing {
        Ecp5InitialPlacementAlgorithm::TimingDrivenRoutabilityElectrostatic
    } else {
        Ecp5InitialPlacementAlgorithm::ConnectivityDrivenElectrostatic
    };
    let mut placement = if let Some(bindings) = options.initial_placement {
        named_initial_placement(&design, architecture, &packing, bindings)?
    } else {
        let placement_refiner = PlacementRefiner::new_with_workspace(
            &design,
            architecture.device(),
            packing.constraints(),
            &mut placement_refinement_workspace,
        )?;
        initial_analytical_placement(
            &design,
            architecture,
            &packing,
            &global_routing_cache,
            &placement_refiner,
            &ff_control_sets,
            placement_delay_predictor
                .as_ref()
                .filter(|_| options.optimize_timing)
                .map(|delay_predictor| {
                    (
                        &timing_model,
                        &timing_constraints,
                        placement_weight_exponent,
                        delay_predictor,
                    )
                }),
        )?
    };
    let clock_bindings = if options.initial_placement.is_none() {
        packing.lock_global_clock_buffers_to_shortest_sources(&design, architecture, &placement)?
    } else {
        BTreeMap::new()
    };
    if !clock_bindings.is_empty() {
        let mut bindings = placement.bindings().to_vec();
        for (cell, bel) in clock_bindings {
            bindings[cell.0] = bel;
        }
        placement = placement_from_complete_bindings(
            &design,
            architecture.device(),
            packing.constraints(),
            bindings,
        )?;
    }
    progress(Ecp5FlowStage::Placed);
    report_metric_phase("initial_placement", &mut phase_started);
    let predicted_timing = if let Some(delay_predictor) = placement_delay_predictor.as_ref() {
        let placement_refiner = PlacementRefiner::new_with_workspace(
            &design,
            architecture.device(),
            packing.constraints(),
            &mut placement_refinement_workspace,
        )?;
        Some(estimated_placement_timing(
            &design,
            &placement,
            &timing_model,
            &timing_constraints,
            &placement_refiner,
            delay_predictor,
        )?)
    } else {
        None
    };
    let mut timing_routing_costs = predicted_timing
        .as_ref()
        .map(|estimate| {
            let mut costs = ecp5_routing_costs(
                architecture,
                speed_grade,
                timing_net_weights(estimate, &timing_constraints),
            )?;
            costs.set_sink_criticalities(timing_arc_weights(estimate, &timing_constraints));
            Ok::<_, Ecp5FlowError>(costs)
        })
        .transpose()?;
    let routing = packing.global_routing_constraints_cached(
        &design,
        architecture,
        &placement,
        &mut global_routing_cache,
    )?;
    progress(Ecp5FlowStage::GlobalClocksRouted);
    report_metric_phase("initial_global_routing", &mut phase_started);
    let mut routing_workspace = RoutingWorkspace::new(architecture.device());
    let initial_implementation = if let Some(costs) = timing_routing_costs.as_ref() {
        route_with_timing_costs_workspace_and_progress(
            &design,
            architecture.device(),
            placement,
            &routing,
            costs,
            &mut routing_workspace,
            |event| progress(Ecp5FlowStage::Routing(event)),
        )?
    } else {
        route_with_workspace_and_progress(
            &design,
            architecture.device(),
            placement,
            &routing,
            &mut routing_workspace,
            |event| progress(Ecp5FlowStage::Routing(event)),
        )?
    };
    progress(Ecp5FlowStage::Routed);
    let initial_timing = analyze_ecp5_implementation(
        &design,
        architecture,
        speed_grade,
        &initial_implementation,
        &timing_model,
        &timing_constraints,
    )?;
    progress(timing_snapshot(&initial_timing));
    if let Some(costs) = timing_routing_costs.as_mut() {
        costs.set_net_criticalities(timing_net_weights(&initial_timing, &timing_constraints));
        costs.set_sink_criticalities(timing_arc_weights(&initial_timing, &timing_constraints));
    }
    report_metric_phase("initial_route_and_timing", &mut phase_started);
    let (mut implementation, mut timing) = (initial_implementation, initial_timing);
    let mut route_eco_worklist = WorstSetupRouteEcoWorklist::default();
    if options.optimize_timing
        && timing.worst_slack_ps.is_some_and(|slack| slack < 0)
        && let Some(costs) = timing_routing_costs.as_mut()
    {
        let eco_routing = packing.global_routing_constraints_cached(
            &design,
            architecture,
            &implementation.placement,
            &mut global_routing_cache,
        )?;
        improve_worst_setup_net_route_ecos(
            &design,
            architecture,
            speed_grade,
            &timing_model,
            &timing_constraints,
            &eco_routing,
            costs,
            &mut routing_workspace,
            &mut implementation,
            &mut timing,
            &mut route_eco_worklist,
            &mut progress,
        )?;
    }
    let mut placement_feedback_changed_implementation = false;
    if options.optimize_timing
        && let (Some(costs), Some(delay_predictor), Some(initial_predicted_timing)) = (
            timing_routing_costs.as_mut(),
            placement_delay_predictor.as_ref(),
            predicted_timing.as_ref(),
        )
    {
        let initial_feedback_placement = implementation.placement.clone();
        let ((next_implementation, next_timing), feedback_ran) = run_setup_feedback_fallback(
            (implementation, timing),
            |(_, timing)| timing.worst_slack_ps.is_some_and(|slack| slack < 0),
            |(implementation, timing)| {
                // Keep construction of the global placement machinery inside
                // the fallback. A successful local route ECO therefore avoids
                // both the packing clone and placement-refiner setup.
                let closure_packing = packing.clone();
                let placement_refiner = PlacementRefiner::new_with_workspace(
                    &design,
                    architecture.device(),
                    closure_packing.constraints(),
                    &mut placement_refinement_workspace,
                )?;
                let optimized = TimingFeedbackContext {
                    design: &design,
                    architecture,
                    packing: &closure_packing,
                    placement_refiner: &placement_refiner,
                    global_routing_cache: &mut global_routing_cache,
                    speed_grade,
                    timing_model: &timing_model,
                    timing_constraints: &timing_constraints,
                    delay_predictor,
                    routing_workspace: &mut routing_workspace,
                }
                .improve_setup(
                    implementation,
                    timing,
                    initial_predicted_timing,
                    costs,
                    &mut progress,
                )?;
                drop(placement_refiner);
                Ok::<_, Ecp5FlowError>(optimized)
            },
        )?;
        implementation = next_implementation;
        timing = next_timing;
        placement_feedback_changed_implementation =
            feedback_ran && implementation.placement != initial_feedback_placement;
    }
    if options.optimize_timing
        && placement_feedback_changed_implementation
        && timing.worst_slack_ps.is_some_and(|slack| slack < 0)
        && let Some(costs) = timing_routing_costs.as_mut()
    {
        // Global placement and routing replaced the incumbent physical state,
        // so a previously tried net is no longer the same route candidate.
        // Reopen every net because its route candidate now belongs to a new
        // physical state, then exhaust the refreshed exact-WNS cone.
        route_eco_worklist.reset_attempted_after_global_change();
        let eco_routing = packing.global_routing_constraints_cached(
            &design,
            architecture,
            &implementation.placement,
            &mut global_routing_cache,
        )?;
        improve_worst_setup_net_route_ecos(
            &design,
            architecture,
            speed_grade,
            &timing_model,
            &timing_constraints,
            &eco_routing,
            costs,
            &mut routing_workspace,
            &mut implementation,
            &mut timing,
            &mut route_eco_worklist,
            &mut progress,
        )?;
    }
    if let Some(costs) = timing_routing_costs.as_mut()
        && timing.worst_slack_ps.is_some_and(|slack| slack >= 0)
        && timing.worst_hold_slack_ps.is_some_and(|slack| slack < 0)
    {
        repair_hold_after_setup(
            &design,
            architecture,
            speed_grade,
            &constant_luts,
            imported.metadata(),
            &timing_constraints,
            &mut packing,
            &mut implementation,
            &mut timing,
            costs,
            &mut global_routing_cache,
            &mut routing_workspace,
            &mut progress,
        )?;
    }
    emit_placement_metric(
        "final_place",
        &design,
        architecture.device(),
        &implementation.placement,
        Some(&timing),
    );
    report_metric_phase("timing_closure", &mut phase_started);
    progress(Ecp5FlowStage::Timed);
    staged_evidence.record(Gate::PhysicalImplementation);
    staged_evidence.record_timing(&design, &timing, options.timing_exceptions);
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
        timing_exceptions: options.timing_exceptions.to_vec(),
        clock_constraints: options.clock_constraints.to_vec(),
        setup_uncertainty_ps: options.setup_uncertainty_ps,
        placement_weight_exponent,
        initial_placement_algorithm,
    })
}

fn ff_ce_control_sets(
    design: &Design,
    metadata: &BTreeMap<CellId, PrimitiveMetadata>,
    absorbed_inputs: &BTreeMap<CellId, BTreeMap<String, bool>>,
) -> Vec<(CellId, u64)> {
    let keys = metadata
        .iter()
        .filter_map(|(&cell, primitive)| {
            let PrimitiveMetadata::FlipFlop { enable, .. } = primitive else {
                return None;
            };
            let ce_net = design.cells()[cell.0]
                .pins()
                .iter()
                .copied()
                .find(|pin| design.pins()[pin.0].name == "CE")
                .and_then(|pin| design.pins()[pin.0].net())
                .map(|net| net.0);
            let absorbed = absorbed_inputs
                .get(&cell)
                .and_then(|pins| pins.get("CE"))
                .copied();
            let active_low = enable.is_some_and(|active| active == ActiveLevel::Low);
            let key = match (*enable, ce_net, absorbed) {
                (None, _, _) => (0, None, true),
                (Some(_), Some(net), _) => (1, Some(net), active_low),
                (Some(_), None, Some(value)) => (0, None, value != active_low),
                (Some(_), None, None) => (2, None, active_low),
            };
            Some((cell, key))
        })
        .collect::<Vec<_>>();
    let classes = keys
        .iter()
        .map(|(_, key)| *key)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .enumerate()
        .map(|(index, key)| {
            (
                key,
                u64::try_from(index).expect("FF CE control-set count fits u64"),
            )
        })
        .collect::<BTreeMap<_, _>>();
    keys.into_iter()
        .map(|(cell, key)| (cell, classes[&key]))
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

fn ff_lsr_control_sets(
    design: &Design,
    metadata: &BTreeMap<CellId, PrimitiveMetadata>,
    absorbed_inputs: &BTreeMap<CellId, BTreeMap<String, bool>>,
) -> Vec<(CellId, u64)> {
    let keys = metadata
        .iter()
        .filter_map(|(&cell, primitive)| {
            let PrimitiveMetadata::FlipFlop { reset, .. } = primitive else {
                return None;
            };
            let lsr_net = design.cells()[cell.0]
                .pins()
                .iter()
                .copied()
                .find(|pin| design.pins()[pin.0].name == "LSR")
                .and_then(|pin| design.pins()[pin.0].net())
                .map(|net| net.0);
            let absorbed = absorbed_inputs
                .get(&cell)
                .and_then(|pins| pins.get("LSR"))
                .copied();
            let key = reset.map_or((0, None, false, false), |reset| {
                let active_low = reset.active == ActiveLevel::Low;
                match (lsr_net, absorbed) {
                    (Some(net), _) => (1, Some(net), active_low, reset.asynchronous),
                    (None, Some(value)) if value == active_low => (0, None, false, false),
                    (None, Some(_)) => (2, None, active_low, reset.asynchronous),
                    (None, None) => (3, None, active_low, reset.asynchronous),
                }
            });
            Some((cell, key))
        })
        .collect::<Vec<_>>();
    let classes = keys
        .iter()
        .map(|(_, key)| *key)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .enumerate()
        .map(|(index, key)| {
            (
                key,
                u64::try_from(index).expect("FF LSR control-set count fits u64"),
            )
        })
        .collect::<BTreeMap<_, _>>();
    keys.into_iter()
        .map(|(cell, key)| (cell, classes[&key]))
        .collect()
}

fn ff_control_sets(
    design: &Design,
    metadata: &BTreeMap<CellId, PrimitiveMetadata>,
    absorbed_inputs: &BTreeMap<CellId, BTreeMap<String, bool>>,
) -> Vec<FfControlSet> {
    let ce = ff_ce_control_sets(design, metadata, absorbed_inputs)
        .into_iter()
        .collect::<BTreeMap<_, _>>();
    let clock = ff_clock_control_sets(design, metadata)
        .into_iter()
        .collect::<BTreeMap<_, _>>();
    let lsr = ff_lsr_control_sets(design, metadata, absorbed_inputs)
        .into_iter()
        .collect::<BTreeMap<_, _>>();
    metadata
        .iter()
        .filter_map(|(&cell, primitive)| match primitive {
            PrimitiveMetadata::FlipFlop { .. } => Some(FfControlSet {
                cell,
                slice_ce: ce[&cell],
                tile_clock: clock[&cell],
                tile_lsr: lsr[&cell],
            }),
            _ => None,
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

type PlacementIdentity = (Vec<BelId>, Vec<Option<BelPinId>>);

fn placement_identity(design: &Design, placement: &Placement) -> PlacementIdentity {
    (
        placement.bindings().to_vec(),
        (0..design.pins().len())
            .map(|pin| placement.pin_binding(CellPinId(pin)))
            .collect(),
    )
}

/// One deterministic STA-feedback trajectory.
///
/// Detailed placement and routing each own their generic physical objective.
/// Full related-clock STA is the sole authority for accepting a completed
/// placement-and-route trial.
struct TimingFeedbackContext<'a, 'work, 'cache> {
    design: &'a Design,
    architecture: &'a Ecp5Architecture,
    packing: &'a Ecp5Packing,
    placement_refiner: &'work PlacementRefiner<'a>,
    global_routing_cache: &'work mut Ecp5GlobalRoutingCache<'cache>,
    speed_grade: &'a SpeedGradeRecord,
    timing_model: &'a TimingModel,
    timing_constraints: &'a TimingConstraints,
    delay_predictor: &'a Ecp5PlacementDelayPredictor<'a>,
    routing_workspace: &'work mut RoutingWorkspace,
}

fn report_timing_feedback_stop(round: usize, reason: &str) {
    if metrics_enabled() {
        eprintln!("[metrics] timing_feedback_stop round={round} reason={reason}");
    }
}

fn report_timing_feedback_prescreen(
    round: usize,
    incumbent: &TimingReport,
    candidate: &TimingReport,
    rejected: bool,
    elapsed_ms: f64,
) {
    if !metrics_enabled() {
        return;
    }
    let incumbent_violations =
        slack_violations(incumbent.setup_checks.iter().map(|check| check.slack_ps));
    let candidate_violations =
        slack_violations(candidate.setup_checks.iter().map(|check| check.slack_ps));
    let incumbent_slacks = incumbent
        .setup_checks
        .iter()
        .map(|check| ((check.cell, check.data_pin), check.slack_ps))
        .collect::<BTreeMap<_, _>>();
    let (mut endpoints_better, mut endpoints_equal, mut endpoints_worse) =
        (0_usize, 0_usize, 0_usize);
    for check in &candidate.setup_checks {
        match incumbent_slacks
            .get(&(check.cell, check.data_pin))
            .map(|incumbent| check.slack_ps.cmp(incumbent))
        {
            Some(std::cmp::Ordering::Greater) => endpoints_better += 1,
            Some(std::cmp::Ordering::Equal) => endpoints_equal += 1,
            Some(std::cmp::Ordering::Less) => endpoints_worse += 1,
            None => {}
        }
    }
    let predicted_improves = strictly_improves_timing_objective(
        timing_objective(candidate),
        timing_objective(incumbent),
    );
    eprintln!(
        "[metrics] timing_feedback_prescreen round={round} incumbent_wns={:?} incumbent_tns={} incumbent_violations={} candidate_wns={:?} candidate_tns={} candidate_violations={} endpoints_better={endpoints_better} endpoints_equal={endpoints_equal} endpoints_worse={endpoints_worse} predicted_improves={predicted_improves} rejected={rejected} elapsed_ms={elapsed_ms:.3}",
        incumbent.worst_slack_ps,
        incumbent_violations.total_negative_slack_ps(),
        incumbent_violations.endpoints,
        candidate.worst_slack_ps,
        candidate_violations.total_negative_slack_ps(),
        candidate_violations.endpoints,
    );
}

impl TimingFeedbackContext<'_, '_, '_> {
    fn prescreen_placement(
        &self,
        round: usize,
        placement: &Placement,
        incumbent: &TimingReport,
    ) -> Result<Option<TimingReport>, Ecp5FlowError> {
        let started = Instant::now();
        let candidate = estimated_placement_timing(
            self.design,
            placement,
            self.timing_model,
            self.timing_constraints,
            self.placement_refiner,
            self.delay_predictor,
        )?;
        let rejected = predicted_setup_candidate_is_pareto_dominated(incumbent, &candidate);
        report_timing_feedback_prescreen(
            round,
            incumbent,
            &candidate,
            rejected,
            started.elapsed().as_secs_f64() * 1_000.0,
        );
        if rejected {
            report_timing_feedback_stop(round, "predicted_setup_pareto_dominated");
            Ok(None)
        } else {
            Ok(Some(candidate))
        }
    }

    fn improve_setup(
        &mut self,
        mut implementation: PnrResult,
        mut timing: TimingReport,
        initial_predicted_timing: &TimingReport,
        routing_costs: &mut RoutingCosts,
        progress: &mut impl FnMut(Ecp5FlowStage),
    ) -> Result<(PnrResult, TimingReport), Ecp5FlowError> {
        let mut seen = BTreeSet::from([placement_identity(self.design, &implementation.placement)]);
        let mut objective = timing_objective(&timing);
        let mut incumbent_predicted = initial_predicted_timing.clone();
        let mut round = 0_usize;
        loop {
            if timing.worst_slack_ps.is_none_or(|slack| slack >= 0) {
                report_timing_feedback_stop(round, "timing_met");
                break;
            }
            round += 1;
            let criticalities = timing_arc_weights(&timing, self.timing_constraints);
            let (placement, moved) = self.placement_refiner.refine_with_predicted_timing_pass(
                implementation.placement.clone(),
                &criticalities,
                self.delay_predictor,
            )?;
            progress(Ecp5FlowStage::TimingDrivenPlaced);
            if moved == 0 || placement == implementation.placement {
                report_timing_feedback_stop(round, "placement_fixed_point");
                break;
            }
            if !seen.insert(placement_identity(self.design, &placement)) {
                report_timing_feedback_stop(round, "seen_placement");
                break;
            }

            let Some(candidate_predicted) =
                self.prescreen_placement(round, &placement, &incumbent_predicted)?
            else {
                break;
            };

            let routing = self.packing.global_routing_constraints_cached(
                self.design,
                self.architecture,
                &placement,
                self.global_routing_cache,
            )?;
            progress(Ecp5FlowStage::TimingDrivenGlobalClocksRouted);
            routing_costs
                .set_net_criticalities(timing_net_weights(&timing, self.timing_constraints));
            routing_costs.set_sink_criticalities(criticalities);
            routing_costs.set_sink_min_delays_ps(BTreeMap::new());
            let candidate = route_with_timing_costs_workspace_and_progress(
                self.design,
                self.architecture.device(),
                placement,
                &routing,
                routing_costs,
                self.routing_workspace,
                |event| progress(Ecp5FlowStage::TimingDrivenRouting(event)),
            );
            let candidate = match candidate {
                Ok(candidate) => candidate,
                Err(PnrError::CongestionNotResolved { .. } | PnrError::Unroutable { .. }) => {
                    progress(Ecp5FlowStage::TimingTrialDecision {
                        improves_objective: false,
                    });
                    report_timing_feedback_stop(round, "route_failure");
                    break;
                }
                Err(error) => return Err(error.into()),
            };
            progress(Ecp5FlowStage::TimingDrivenRouted);
            let candidate_timing = analyze_ecp5_implementation(
                self.design,
                self.architecture,
                self.speed_grade,
                &candidate,
                self.timing_model,
                self.timing_constraints,
            )?;
            progress(timing_snapshot(&candidate_timing));
            let candidate_objective = timing_objective(&candidate_timing);
            let improves = strictly_improves_timing_objective(candidate_objective, objective);
            progress(Ecp5FlowStage::TimingTrialDecision {
                improves_objective: improves,
            });
            if metrics_enabled() {
                eprintln!(
                    "[metrics] timing_feedback round={round} wns={:?} tns={} accepted={improves}",
                    candidate_timing.worst_slack_ps,
                    slack_violations(
                        candidate_timing
                            .setup_checks
                            .iter()
                            .map(|check| check.slack_ps)
                    )
                    .total_negative_slack_ps(),
                );
            }
            if !improves {
                report_timing_feedback_stop(round, "strict_non_improvement");
                break;
            }
            implementation = candidate;
            timing = candidate_timing;
            objective = candidate_objective;
            incumbent_predicted = candidate_predicted;
        }
        Ok((implementation, timing))
    }
}

/// Repairs post-setup hold failures whose routing freedom was removed by an
/// earlier dedicated LUT→FF packing choice. Pairs are released one at a time,
/// locally rerouted with their required minimum delay, and committed only
/// after full STA preserves setup closure and improves the timing objective.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn repair_hold_after_setup(
    design: &Design,
    architecture: &Ecp5Architecture,
    speed_grade: &SpeedGradeRecord,
    constant_cells: &BTreeSet<CellId>,
    metadata: &BTreeMap<CellId, PrimitiveMetadata>,
    timing_constraints: &TimingConstraints,
    packing: &mut Ecp5Packing,
    implementation: &mut PnrResult,
    timing: &mut TimingReport,
    routing_costs: &mut RoutingCosts,
    global_routing_cache: &mut Ecp5GlobalRoutingCache<'_>,
    routing_workspace: &mut RoutingWorkspace,
    progress: &mut impl FnMut(Ecp5FlowStage),
) -> Result<(), Ecp5FlowError> {
    repair_general_hold_routes(
        design,
        architecture,
        speed_grade,
        constant_cells,
        metadata,
        timing_constraints,
        packing,
        implementation,
        timing,
        routing_costs,
        global_routing_cache,
        routing_workspace,
        progress,
    )?;
    if timing.met_timing() {
        return Ok(());
    }

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
            let trial_model = ecp5_timing_model(
                design,
                &trial_packing,
                speed_grade,
                constant_cells,
                metadata,
            )?;
            // LUT/FF pair release changes data arcs only. Reuse the original
            // clock constraints, including the caller's setup uncertainty.
            let requested_minimums = BTreeMap::from([(key, minimum_ps)]);
            let Some((mut trial_implementation, mut trial_timing)) = route_hold_trial(
                design,
                architecture,
                speed_grade,
                &trial_packing,
                implementation,
                timing,
                &trial_model,
                timing_constraints,
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
            // setup criticality. Keep only strict objective improvements and
            // stop as soon as setup closes or routing reaches a fixed point.
            loop {
                if trial_timing.worst_slack_ps.is_some_and(|setup| setup >= 0) {
                    break;
                }
                let Some((refined_implementation, refined_timing)) = route_hold_trial(
                    design,
                    architecture,
                    speed_grade,
                    &trial_packing,
                    &trial_implementation,
                    &trial_timing,
                    &trial_model,
                    timing_constraints,
                    requested_minimums.clone(),
                    routing_costs,
                    global_routing_cache,
                    routing_workspace,
                    progress,
                )?
                else {
                    break;
                };
                let unchanged = refined_implementation == trial_implementation;
                let refined_objective = timing_objective(&refined_timing);
                if !strictly_improves_timing_objective(
                    refined_objective,
                    timing_objective(&trial_timing),
                ) {
                    break;
                }
                trial_implementation = refined_implementation;
                trial_timing = refined_timing;
                if unchanged {
                    break;
                }
            }
            let improves = trial_timing.worst_slack_ps.is_some_and(|setup| setup >= 0)
                && strictly_improves_timing_objective(
                    timing_objective(&trial_timing),
                    timing_objective(timing),
                );
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

    repair_general_hold_routes(
        design,
        architecture,
        speed_grade,
        constant_cells,
        metadata,
        timing_constraints,
        packing,
        implementation,
        timing,
        routing_costs,
        global_routing_cache,
        routing_workspace,
        progress,
    )
}

/// Repairs all general-routing hold arcs together after setup has closed.
#[allow(clippy::too_many_arguments)]
fn repair_general_hold_routes(
    design: &Design,
    architecture: &Ecp5Architecture,
    speed_grade: &SpeedGradeRecord,
    constant_cells: &BTreeSet<CellId>,
    metadata: &BTreeMap<CellId, PrimitiveMetadata>,
    timing_constraints: &TimingConstraints,
    packing: &Ecp5Packing,
    implementation: &mut PnrResult,
    timing: &mut TimingReport,
    routing_costs: &mut RoutingCosts,
    global_routing_cache: &mut Ecp5GlobalRoutingCache<'_>,
    routing_workspace: &mut RoutingWorkspace,
    progress: &mut impl FnMut(Ecp5FlowStage),
) -> Result<(), Ecp5FlowError> {
    let model = ecp5_timing_model(design, packing, speed_grade, constant_cells, metadata)?;
    let mut accumulated_minimums = BTreeMap::<(NetId, CellPinId), u64>::new();
    loop {
        let new_minimums = hold_sink_min_delays(timing);
        if new_minimums.is_empty() {
            break;
        }
        accumulate_hold_minimums(&mut accumulated_minimums, new_minimums);
        let Some((trial_implementation, trial_timing)) = route_hold_trial(
            design,
            architecture,
            speed_grade,
            packing,
            implementation,
            timing,
            &model,
            timing_constraints,
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
        let improves = strictly_improves_timing_objective(
            timing_objective(&trial_timing),
            timing_objective(timing),
        );
        progress(Ecp5FlowStage::TimingTrialDecision {
            improves_objective: improves,
        });
        if metrics_enabled() {
            eprintln!(
                "[metrics] general_hold_feedback wns={:?} whs={:?} accepted={improves}",
                trial_timing.worst_slack_ps, trial_timing.worst_hold_slack_ps,
            );
        }
        if !improves {
            break;
        }
        let closed = trial_timing.met_timing();
        *implementation = trial_implementation;
        *timing = trial_timing;
        if closed {
            break;
        }
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
            progress(Ecp5FlowStage::TimingTrialDecision {
                improves_objective: false,
            });
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

fn ecp5_routing_costs(
    architecture: &Ecp5Architecture,
    speed_grade: &SpeedGradeRecord,
    net_criticalities: BTreeMap<NetId, u64>,
) -> Result<RoutingCosts, Ecp5FlowError> {
    let mut class_delays = vec![None; architecture.metadata_string_count()];
    let mut span6_delay_per_tile_ps = None::<u64>;
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
        if is_ecp5_span6_continuation(name) {
            let per_tile = u64::from(maximum).div_ceil(6);
            span6_delay_per_tile_ps =
                Some(span6_delay_per_tile_ps.map_or(per_tile, |known| known.min(per_tile)));
        }
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
    if let Some(delay_per_tile_ps) = span6_delay_per_tile_ps {
        costs.set_alternate_source_delay_per_tile_ps(delay_per_tile_ps);
    }
    Ok(costs)
}

fn is_ecp5_span6_continuation(name: &str) -> bool {
    matches!(
        name,
        "span6hw_to_span6hw_w6"
            | "span6he_to_span6he_e6"
            | "span6vn_to_span6vn_n6"
            | "span6vs_to_span6vs_s6"
    )
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
fn placement_star_length(design: &Design, device: &Device, placement: &Placement) -> u64 {
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

/// Sum of one half-perimeter bounding box per non-clock net.
fn placement_bbox_hpwl(design: &Design, device: &Device, placement: &Placement) -> u64 {
    design.nets().iter().fold(0_u64, |total, net| {
        let driver_cell = design.pins()[net.driver.0].cell;
        if design.cells()[driver_cell.0].kind == ResourceKind::Clock {
            return total;
        }
        let Some(driver) = placement.point(driver_cell, device) else {
            return total;
        };
        let mut min_x = driver.x;
        let mut max_x = driver.x;
        let mut min_y = driver.y;
        let mut max_y = driver.y;
        for &sink in &net.sinks {
            let sink_cell = design.pins()[sink.0].cell;
            if let Some(point) = placement.point(sink_cell, device) {
                min_x = min_x.min(point.x);
                max_x = max_x.max(point.x);
                min_y = min_y.min(point.y);
                max_y = max_y.max(point.y);
            }
        }
        total
            .saturating_add(u64::from(max_x - min_x))
            .saturating_add(u64::from(max_y - min_y))
    })
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
    let bbox_hpwl = placement_bbox_hpwl(design, device, placement);
    let star_length = placement_star_length(design, device, placement);
    match timing {
        Some(timing) => eprintln!(
            "[metrics] stage={stage} bbox_hpwl={bbox_hpwl} star_length={star_length} wns={:?} hold={:?} timing_endpoints_checked={} timing_endpoints_modeled={}",
            timing.worst_slack_ps,
            timing.worst_hold_slack_ps,
            timing.setup_checks.len(),
            timing.modeled_endpoint_count(),
        ),
        None => {
            eprintln!("[metrics] stage={stage} bbox_hpwl={bbox_hpwl} star_length={star_length}");
        }
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

/// ECP5 analytical-placement connection weights, normalized independently in
/// each related-clock domain. The user exponent controls the documented
/// `1 + 10 * criticality^exponent` model directly; estimated edge delay does
/// not attenuate the timing force a second time.
fn ecp5_timing_placement_weights(
    timing: &TimingReport,
    exponent: u32,
) -> BTreeMap<(NetId, CellPinId), u64> {
    let mut weights = BTreeMap::<(NetId, CellPinId), u64>::new();
    for edge in &timing.net_setup_criticalities {
        let weight = ecp5_placement_criticality_weight(
            edge.path_delay_ps,
            edge.domain_worst_path_delay_ps,
            exponent,
        );
        weights
            .entry((edge.net, edge.sink))
            .and_modify(|known| *known = (*known).max(weight))
            .or_insert(weight);
    }
    weights
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

fn ecp5_placement_criticality_weight(
    path_delay_ps: u128,
    worst_path_delay_ps: u128,
    exponent: u32,
) -> u64 {
    const SCALE: u64 = 1 << 10;
    const TIMING_WEIGHT: u64 = 10;
    let exponent = exponent.max(1);
    let scaled = if worst_path_delay_ps == 0 {
        0
    } else {
        scaled_binary_fraction(
            path_delay_ps.min(worst_path_delay_ps),
            worst_path_delay_ps,
            SCALE.ilog2(),
        )
    };
    let exact = u128::from(scaled)
        .checked_pow(exponent)
        .zip(u128::from(SCALE).checked_pow(exponent))
        .and_then(|(numerator, denominator)| {
            u128::from(TIMING_WEIGHT)
                .checked_mul(numerator)
                .map(|weighted| (weighted + denominator / 2) / denominator)
        });
    let extra = exact.unwrap_or_else(|| {
        let powered = scaled_fraction_power(scaled, exponent, SCALE);
        u128::from(
            TIMING_WEIGHT
                .saturating_mul(powered)
                .saturating_add(SCALE / 2)
                / SCALE,
        )
    });
    1 + u64::try_from(extra)
        .unwrap_or(TIMING_WEIGHT)
        .min(TIMING_WEIGHT)
}

/// Fixed-point fallback for exponents whose exact rational power does not fit
/// in `u128`. Values remain within `scale`, so binary exponentiation cannot
/// overflow and very large public API inputs remain deterministic.
fn scaled_fraction_power(scaled: u64, mut exponent: u32, scale: u64) -> u64 {
    let scale = u128::from(scale);
    let mut result = scale;
    let mut factor = u128::from(scaled);
    while exponent != 0 {
        if exponent & 1 != 0 {
            result = (result * factor + scale / 2) / scale;
        }
        exponent >>= 1;
        if exponent != 0 {
            factor = (factor * factor + scale / 2) / scale;
        }
    }
    u64::try_from(result).expect("a normalized fixed-point power remains within scale")
}

fn scaled_binary_fraction(numerator: u128, denominator: u128, bits: u32) -> u64 {
    debug_assert!(denominator > 0);
    if numerator >= denominator {
        return 1_u64 << bits;
    }
    let mut remainder = numerator;
    let mut scaled = 0_u64;
    for _ in 0..bits {
        scaled <<= 1;
        // Compare 2*r with d as r >= d-r. This form also covers d=2^127
        // and never constructs an overflowing doubled remainder.
        let complement = denominator - remainder;
        if remainder >= complement {
            remainder -= complement;
            scaled |= 1;
        } else {
            remainder *= 2;
        }
    }
    scaled
}

type ViolationScore = (Reverse<u128>, Reverse<u128>, Reverse<u128>, Reverse<usize>);

const WORST_SETUP_ROUTE_ECO_ESTIMATE_DELAY_PER_TILE_PS: u64 = 52;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct WorstSetupNetRouteEcoCandidate {
    net: NetId,
    sink: CellPinId,
    slack_ps: i128,
    route_delay_ps: u64,
    shared_prefix_delay_ps: u64,
    worst_sinks: usize,
    fanout: usize,
}

#[derive(Clone, Debug)]
struct WorstSetupNetRouteEcoAggregate {
    candidate: WorstSetupNetRouteEcoCandidate,
    sinks: BTreeSet<CellPinId>,
}

/// Deterministic finite scheduler for whole-net route ECO trials.
///
/// A rejected item must not be retried: a strict commit changes another net
/// and the global timing objective, but leaves every rejected net's incumbent
/// route unchanged. Filtering attempted nets makes exhaustion of the finite
/// design-net set the natural stop when no candidate closes setup.
#[derive(Debug, Default)]
struct WorstSetupRouteEcoWorklist {
    trials: usize,
    attempted: BTreeSet<NetId>,
}

impl WorstSetupRouteEcoWorklist {
    fn next(
        &mut self,
        candidates: impl IntoIterator<Item = WorstSetupNetRouteEcoCandidate>,
    ) -> Option<WorstSetupNetRouteEcoCandidate> {
        let candidate = candidates
            .into_iter()
            .find(|candidate| self.attempted.insert(candidate.net))?;
        self.trials += 1;
        Some(candidate)
    }

    fn trials(&self) -> usize {
        self.trials
    }

    fn reset_attempted_after_global_change(&mut self) {
        self.attempted.clear();
    }
}

/// Selects every unique net from the current worst setup cone.
///
/// `net_setup_slacks` is built from the same forward maximum arrivals and
/// backward minimum required times as endpoint STA. A net edge whose slack is
/// whole-design WNS therefore belongs to a worst-slack timing cone. Nets are
/// ranked by their largest exact realized edge delay; exact delay on resources
/// shared by sibling arcs is a deterministic tie-breaker, while raw fanout is
/// deliberately not an optimization heuristic.
fn worst_setup_net_route_eco_candidates(
    timing: &TimingReport,
    routes: &[Arc<NetRoute>],
    pip_delays_ps: &[u32],
) -> Vec<WorstSetupNetRouteEcoCandidate> {
    let Some(worst_slack_ps) = timing.worst_slack_ps else {
        return Vec::new();
    };
    let delays = timing
        .net_delays
        .iter()
        .map(|delay| ((delay.net, delay.sink), delay.delay.max_ps))
        .collect::<BTreeMap<_, _>>();
    let mut aggregates = BTreeMap::<NetId, WorstSetupNetRouteEcoAggregate>::new();
    for edge in timing
        .net_setup_slacks
        .iter()
        .filter(|edge| edge.slack_ps == worst_slack_ps)
    {
        let Some(&route_delay_ps) = delays.get(&(edge.net, edge.sink)) else {
            continue;
        };
        let Some(route) = routes.get(edge.net.0).filter(|route| route.net == edge.net) else {
            continue;
        };
        let Some(arc) = route.arc(edge.sink) else {
            continue;
        };
        let shared_prefix_delay_ps = arc
            .pips
            .iter()
            .filter(|&&pip| route.pip_ref_count(pip) > 1)
            .filter_map(|pip| pip_delays_ps.get(pip.0))
            .fold(0_u64, |sum, &delay| sum.saturating_add(u64::from(delay)));
        let aggregate =
            aggregates
                .entry(edge.net)
                .or_insert_with(|| WorstSetupNetRouteEcoAggregate {
                    candidate: WorstSetupNetRouteEcoCandidate {
                        net: edge.net,
                        sink: edge.sink,
                        slack_ps: edge.slack_ps,
                        route_delay_ps: 0,
                        shared_prefix_delay_ps: 0,
                        worst_sinks: 0,
                        fanout: route.arcs.iter().filter(|arc| arc.sink.is_some()).count(),
                    },
                    sinks: BTreeSet::new(),
                });
        if (route_delay_ps, Reverse(edge.sink))
            > (
                aggregate.candidate.route_delay_ps,
                Reverse(aggregate.candidate.sink),
            )
        {
            aggregate.candidate.route_delay_ps = route_delay_ps;
            aggregate.candidate.sink = edge.sink;
        }
        aggregate.candidate.shared_prefix_delay_ps = aggregate
            .candidate
            .shared_prefix_delay_ps
            .max(shared_prefix_delay_ps);
        aggregate.sinks.insert(edge.sink);
    }
    let mut candidates = aggregates
        .into_values()
        .map(|mut aggregate| {
            aggregate.candidate.worst_sinks = aggregate.sinks.len();
            aggregate.candidate
        })
        .collect::<Vec<_>>();
    candidates.sort_unstable_by_key(|candidate| {
        (
            Reverse(candidate.route_delay_ps),
            Reverse(candidate.shared_prefix_delay_ps),
            Reverse(candidate.worst_sinks),
            candidate.net,
        )
    });
    candidates
}

/// Nets competing for the input-permutation resources of one critical LUT.
///
/// ECP5 LUT inputs are logically named A-D but may reach those pins through a
/// one-to-one set of local permutation wires. Rebuilding only the critical net
/// can therefore fail while a noncritical sibling owns its fast local entry.
/// Releasing the small sink-cell cohort lets the router solve that assignment
/// transactionally without moving the LUT, its packed FF, or any carry macro.
fn worst_setup_route_eco_cohort(
    design: &Design,
    candidate: WorstSetupNetRouteEcoCandidate,
) -> Vec<NetId> {
    const MAX_LUT_INPUT_COHORT: usize = 8;

    let Some(sink_pin) = design.pins().get(candidate.sink.0) else {
        return vec![candidate.net];
    };
    let Some(sink_cell) = design.cells().get(sink_pin.cell.0) else {
        return vec![candidate.net];
    };
    if !matches!(sink_cell.kind, ResourceKind::Lut(_)) {
        return vec![candidate.net];
    }

    let mut nets = sink_cell
        .pins()
        .iter()
        .filter_map(|&pin| {
            let pin = design.pins().get(pin.0)?;
            if pin.direction == PinDirection::Input {
                pin.net()
            } else {
                None
            }
        })
        .collect::<BTreeSet<_>>();
    nets.insert(candidate.net);
    if nets.len() <= 1 || nets.len() > MAX_LUT_INPUT_COHORT {
        return vec![candidate.net];
    }
    nets.into_iter().collect()
}

#[allow(clippy::too_many_arguments)]
fn legal_worst_setup_route_eco_candidate(
    design: &Design,
    device: &Device,
    implementation: &PnrResult,
    routing_constraints: &RoutingConstraints,
    routing_costs: &RoutingCosts,
    candidate: WorstSetupNetRouteEcoCandidate,
    cohort: &[NetId],
    routing_workspace: &mut RoutingWorkspace,
) -> Result<Option<PnrResult>, PnrError> {
    let options = LegalRouteEcoOptions::new(WORST_SETUP_ROUTE_ECO_ESTIMATE_DELAY_PER_TILE_PS);
    if cohort.len() == 1 {
        legal_net_route_eco_candidate_with_workspace(
            design,
            device,
            implementation,
            routing_constraints,
            routing_costs,
            candidate.net,
            options,
            routing_workspace,
        )
    } else {
        legal_nets_route_eco_candidate_with_workspace(
            design,
            device,
            implementation,
            routing_constraints,
            routing_costs,
            cohort,
            options,
            routing_workspace,
        )
    }
}

/// Sole post-route acceptance objective for timing feedback.
///
/// Setup closes before hold participates. Within the active stage, worst slack
/// is monotonic and therefore cannot be traded away for a better aggregate
/// score. The aggregate violations and inactive-stage slack provide stable
/// deterministic tie-breakers.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct TimingObjective {
    setup_closed: bool,
    active_worst_slack_ps: i128,
    active_violations: ViolationScore,
    inactive_worst_slack_ps: i128,
}

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

/// Returns whether the candidate is no better on any predicted setup metric.
///
/// This deliberately uses Pareto dominance instead of the routed acceptance
/// objective's lexicographic ordering.  A placement-delay model cannot see the
/// route topology that a speculative full reroute may discover, so a mixed or
/// tied prediction remains eligible.  Only a candidate whose predicted WNS,
/// worst deficit, squared deficit, TNS, and violation count all fail to improve
/// is cheap enough to reject without routing.
fn predicted_setup_candidate_is_pareto_dominated(
    incumbent: &TimingReport,
    candidate: &TimingReport,
) -> bool {
    let (Some(incumbent_wns), Some(candidate_wns)) =
        (incumbent.worst_slack_ps, candidate.worst_slack_ps)
    else {
        return false;
    };
    if incumbent_wns >= 0
        || candidate_wns >= 0
        || !incumbent
            .setup_checks
            .iter()
            .map(|check| (check.cell, check.data_pin))
            .eq(candidate
                .setup_checks
                .iter()
                .map(|check| (check.cell, check.data_pin)))
    {
        return false;
    }
    let incumbent_violations =
        slack_violations(incumbent.setup_checks.iter().map(|check| check.slack_ps));
    let candidate_violations =
        slack_violations(candidate.setup_checks.iter().map(|check| check.slack_ps));
    let no_metric_improves = candidate_wns <= incumbent_wns
        && candidate_violations.maximum_deficit_ps >= incumbent_violations.maximum_deficit_ps
        && candidate_violations.squared_penalty_ps2 >= incumbent_violations.squared_penalty_ps2
        && candidate_violations.total_deficit_ps >= incumbent_violations.total_deficit_ps
        && candidate_violations.endpoints >= incumbent_violations.endpoints;
    let some_metric_worsens = candidate_wns < incumbent_wns
        || candidate_violations.maximum_deficit_ps > incumbent_violations.maximum_deficit_ps
        || candidate_violations.squared_penalty_ps2 > incumbent_violations.squared_penalty_ps2
        || candidate_violations.total_deficit_ps > incumbent_violations.total_deficit_ps
        || candidate_violations.endpoints > incumbent_violations.endpoints;
    no_metric_improves && some_metric_worsens
}

/// Builds a cheap legal delay-estimation seed, then solves one timing- and
/// routability-weighted electrostatic placement before routing.
///
/// Local placement feedback is intentionally reserved for the post-route
/// loop, where complete ECP5 STA can accept or reject the resulting route.
#[allow(clippy::too_many_lines)]
fn initial_analytical_placement(
    design: &Design,
    architecture: &Ecp5Architecture,
    packing: &Ecp5Packing,
    global_routing_cache: &Ecp5GlobalRoutingCache<'_>,
    placement_refiner: &PlacementRefiner<'_>,
    ff_control_sets: &[FfControlSet],
    timing: Option<(
        &TimingModel,
        &TimingConstraints,
        u32,
        &Ecp5PlacementDelayPredictor<'_>,
    )>,
) -> Result<Placement, Ecp5FlowError> {
    let Some((timing_model, timing_constraints, weight_exponent, delay_predictor)) = timing else {
        let placement = placement_refiner.place_analytically(&BTreeMap::new())?;
        if metrics_enabled() {
            eprintln!("[metrics] placement_global_legalizations count=1");
        }
        emit_placement_metric(
            "initial_place",
            design,
            architecture.device(),
            &placement,
            None,
        );
        return Ok(placement);
    };

    let coarse_started = Instant::now();
    let coarse = placement_refiner.place_analytically_coarse(&BTreeMap::new())?;
    let coarse_elapsed = coarse_started.elapsed();
    let coarse_estimate = estimated_placement_timing(
        design,
        &coarse,
        timing_model,
        timing_constraints,
        placement_refiner,
        delay_predictor,
    )?;
    let coarse_objective = timing_objective(&coarse_estimate);
    let sink_weights = ecp5_timing_placement_weights(&coarse_estimate, weight_exponent);
    let capacity_started = Instant::now();
    let restrictions =
        packing.routing_restrictions_cached(design, architecture, &coarse, global_routing_cache)?;
    let routing_capacity = routing_capacity_map(architecture.device(), &restrictions);
    let capacity_elapsed = capacity_started.elapsed();
    let controls = ff_control_sets
        .iter()
        .map(|set| RegisterControlSet {
            cell: set.cell,
            clock_lsr: (set.tile_clock, set.tile_lsr),
            ce: set.slice_ce,
        })
        .collect::<Vec<_>>();
    let eplace_started = Instant::now();
    let eplace_candidate = placement_refiner
        .place_analytically_with_routing_capacity_and_register_controls(
            &sink_weights,
            &routing_capacity,
            &controls,
        )?;
    let eplace_elapsed = eplace_started.elapsed();
    let eplace_estimate = estimated_placement_timing(
        design,
        &eplace_candidate,
        timing_model,
        timing_constraints,
        placement_refiner,
        delay_predictor,
    )?;
    let eplace_objective = timing_objective(&eplace_estimate);
    let eplace_accepted = eplace_objective > coarse_objective;
    if metrics_enabled() {
        let strengthened = sink_weights.values().filter(|&&weight| weight > 1).count();
        let maximum_weight = sink_weights.values().copied().max().unwrap_or(1);
        let mut histogram = BTreeMap::<u64, usize>::new();
        let mut extra_weight_sum = 0_u128;
        for &weight in sink_weights.values().filter(|&&weight| weight > 1) {
            *histogram.entry(weight).or_default() += 1;
            extra_weight_sum = extra_weight_sum.saturating_add(u128::from(weight - 1));
        }
        eprintln!(
            "[metrics] initial_eplace_timing_weights model={ECP5_PLACEMENT_TIMING_WEIGHT_MODEL} exponent={weight_exponent} histogram={histogram:?} extra_weight_sum={extra_weight_sum}",
        );
        eprintln!(
            "[metrics] initial_eplace_seed coarse_elapsed={coarse_elapsed:?} coarse_bbox_hpwl={} coarse_star_length={} coarse_wns={:?} weighted_arcs={} strengthened_arcs={} maximum_weight={} routing_capacity_elapsed={capacity_elapsed:?} routability_model={ECP5_PLACEMENT_ROUTABILITY_MODEL} eplace_elapsed={eplace_elapsed:?} eplace_bbox_hpwl={} eplace_star_length={} eplace_wns={:?} accepted={eplace_accepted}",
            placement_bbox_hpwl(design, architecture.device(), &coarse),
            placement_star_length(design, architecture.device(), &coarse),
            coarse_estimate.worst_slack_ps,
            sink_weights.len(),
            strengthened,
            maximum_weight,
            placement_bbox_hpwl(design, architecture.device(), &eplace_candidate),
            placement_star_length(design, architecture.device(), &eplace_candidate),
            eplace_estimate.worst_slack_ps,
        );
        eprintln!("[metrics] placement_global_legalizations count=2");
    }
    let placement = if eplace_accepted {
        eplace_candidate
    } else {
        coarse
    };
    emit_placement_metric(
        "initial_place",
        design,
        architecture.device(),
        &placement,
        None,
    );
    Ok(placement)
}

/// Runs the complete timing graph on architecture placement-delay estimates.
///
/// Clock distribution is treated as ideal before global routing. Every data
/// edge is evaluated through the target predictor after resolving its actual
/// candidate BEL pins, so analytical placement receives path slack and
/// per-sink criticality without requiring a speculative physical route.
fn estimated_placement_timing(
    design: &Design,
    placement: &Placement,
    timing_model: &TimingModel,
    timing_constraints: &TimingConstraints,
    placement_refiner: &PlacementRefiner<'_>,
    delay_predictor: &Ecp5PlacementDelayPredictor<'_>,
) -> Result<TimingReport, Ecp5FlowError> {
    let clock_nets = timing_constraints
        .clock_periods_ps()
        .keys()
        .copied()
        .collect::<BTreeSet<_>>();
    let mut delays = Vec::new();
    for (index, net) in design.nets().iter().enumerate() {
        let net_id = NetId(index);
        for &sink in &net.sinks {
            let delay_ps = if clock_nets.contains(&net_id) {
                0
            } else {
                placement_refiner
                    .predicted_arc_delay_ps(placement, net_id, sink, delay_predictor)
                    .unwrap_or(0)
            };
            delays.push(NetDelay {
                net: net_id,
                sink,
                delay: DelayRange::from_independent_corners(delay_ps / 2, delay_ps),
            });
        }
    }
    Ok(analyze_timing_from_net_delays(
        design,
        timing_model,
        timing_constraints,
        delays,
    )?)
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

fn timing_objective(timing: &TimingReport) -> TimingObjective {
    let setup_score =
        slack_violations(timing.setup_checks.iter().map(|check| check.slack_ps)).score();
    let hold_score =
        slack_violations(timing.hold_checks.iter().map(|check| check.slack_ps)).score();
    let setup_slack = timing.worst_slack_ps.unwrap_or(i128::MIN);
    let hold_slack = timing.worst_hold_slack_ps.unwrap_or(i128::MIN);
    staged_timing_objective(setup_score, hold_score, setup_slack, hold_slack)
}

fn strictly_improves_timing_objective(
    candidate: TimingObjective,
    incumbent: TimingObjective,
) -> bool {
    candidate > incumbent
}

fn commit_strict_route_eco_candidate(
    implementation: &mut PnrResult,
    timing: &mut TimingReport,
    candidate_implementation: PnrResult,
    candidate_timing: TimingReport,
) -> bool {
    if !strictly_improves_timing_objective(
        timing_objective(&candidate_timing),
        timing_objective(timing),
    ) {
        return false;
    }
    *implementation = candidate_implementation;
    *timing = candidate_timing;
    true
}

/// Runs global placement feedback only when the preceding local route pass did
/// not close setup timing.
///
/// Keeping this gate around the fallback itself is intentional: callers can
/// construct expensive placement state inside `feedback`, and therefore pay
/// none of that setup cost when route ECO already produced nonnegative WNS.
fn run_setup_feedback_fallback<State, Error>(
    state: State,
    has_setup_violation: impl FnOnce(&State) -> bool,
    feedback: impl FnOnce(State) -> Result<State, Error>,
) -> Result<(State, bool), Error> {
    if has_setup_violation(&state) {
        feedback(state).map(|state| (state, true))
    } else {
        Ok((state, false))
    }
}

/// Tries route ECOs, refreshing the exact worst setup cone after every strict
/// commit and stopping at setup closure or candidate exhaustion.
///
/// A critical LUT releases all of its input nets as one small cohort so that a
/// noncritical sibling cannot retain the fastest local permutation resource.
/// Other cells retain the whole-net behavior. Candidate construction is
/// transactional in `texo-pnr`: all unrelated routes are hard occupancy,
/// immutable target topology remains fixed, and cohort nets are rebuilt in
/// deterministic criticality order. This flow commits only after a complete
/// ECP5 STA pass strictly improves the same staged timing objective used by
/// placement feedback. Each net is attempted at most once for one unchanged
/// physical state, so a nonclosing search terminates after exhausting the
/// finite set of candidate nets.
struct Ecp5EcoTimingSession<'a> {
    timing: TimingAnalysisSession<'a>,
    architecture: &'a Ecp5Architecture,
    speed_grade: &'a SpeedGradeRecord,
    pip_classes: HashMap<PipId, &'a PipClassTimingRecord>,
    selected: Vec<bool>,
    touched_pips: Vec<PipId>,
    source_fanout: Vec<u64>,
    touched_sources: Vec<WireId>,
    pip_delays: HashMap<PipId, DelayRange>,
}

impl<'a> Ecp5EcoTimingSession<'a> {
    fn new(
        design: &'a Design,
        architecture: &'a Ecp5Architecture,
        speed_grade: &'a SpeedGradeRecord,
        model: &'a TimingModel,
        constraints: &'a TimingConstraints,
    ) -> Result<Self, Ecp5FlowError> {
        Ok(Self {
            timing: TimingAnalysisSession::new(design, model, constraints)?,
            architecture,
            speed_grade,
            pip_classes: HashMap::new(),
            selected: vec![false; architecture.device().pips().len()],
            touched_pips: Vec::new(),
            source_fanout: vec![0; architecture.device().wires().len()],
            touched_sources: Vec::new(),
            pip_delays: HashMap::new(),
        })
    }

    fn analyze(&mut self, implementation: &PnrResult) -> Result<TimingReport, Ecp5FlowError> {
        for pip in self.touched_pips.drain(..) {
            self.selected[pip.0] = false;
        }
        for wire in self.touched_sources.drain(..) {
            self.source_fanout[wire.0] = 0;
        }
        for pip in implementation.routes.iter().flat_map(|route| route.pips()) {
            if pip.0 >= self.selected.len() {
                return Err(TimingError::UnknownRoutedPip(pip).into());
            }
            if !self.selected[pip.0] {
                self.selected[pip.0] = true;
                self.touched_pips.push(pip);
                let source = self.architecture.device().pips()[pip.0].from();
                if self.source_fanout[source.0] == 0 {
                    self.touched_sources.push(source);
                }
                self.source_fanout[source.0] += 1;
            }
        }
        for &pip in &self.touched_pips {
            if !self.pip_classes.contains_key(&pip) {
                let timing_class = self.architecture.pip_metadata(pip).timing_class;
                let class = self
                    .speed_grade
                    .pip_classes
                    .get(timing_class)
                    .ok_or_else(|| Ecp5FlowError::MissingPipTimingClass {
                        speed_grade: self.speed_grade.name.clone(),
                        timing_class: timing_class.to_owned(),
                    })?;
                self.pip_classes.insert(pip, class);
            }
            let class = self.pip_classes[&pip];
            self.pip_delays.insert(
                pip,
                pip_class_delay(
                    class,
                    self.source_fanout[self.architecture.device().pips()[pip.0].from().0],
                )?,
            );
        }
        Ok(self
            .timing
            .analyze_routed(self.architecture.device(), implementation, |pip| {
                self.pip_delays.get(&pip).copied()
            })?)
    }
}

#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn improve_worst_setup_net_route_ecos(
    design: &Design,
    architecture: &Ecp5Architecture,
    speed_grade: &SpeedGradeRecord,
    timing_model: &TimingModel,
    timing_constraints: &TimingConstraints,
    routing_constraints: &RoutingConstraints,
    routing_costs: &mut RoutingCosts,
    routing_workspace: &mut RoutingWorkspace,
    implementation: &mut PnrResult,
    timing: &mut TimingReport,
    worklist: &mut WorstSetupRouteEcoWorklist,
    progress: &mut impl FnMut(Ecp5FlowStage),
) -> Result<(), Ecp5FlowError> {
    let eco_started = Instant::now();
    let mut timing_session = Ecp5EcoTimingSession::new(
        design,
        architecture,
        speed_grade,
        timing_model,
        timing_constraints,
    )?;
    let mut objective = timing_objective(timing);
    let initial_trials = worklist.trials();
    let mut accepted = 0_usize;
    loop {
        // `timing` and `routes` change together only after a strict commit.
        // Rebuilding this cheap selector on each iteration makes that refresh
        // immediate; the worklist filters the unchanged cone after rejection.
        let candidates = worst_setup_net_route_eco_candidates(
            timing,
            &implementation.routes,
            routing_costs.pip_delays_ps(),
        );
        let Some(candidate) = worklist.next(candidates) else {
            break;
        };
        let ordinal = worklist.trials();
        routing_costs.set_net_criticalities(timing_net_weights(timing, timing_constraints));
        routing_costs.set_sink_criticalities(timing_arc_weights(timing, timing_constraints));
        routing_costs.set_sink_min_delays_ps(BTreeMap::new());
        let cohort = worst_setup_route_eco_cohort(design, candidate);
        let route_started = Instant::now();
        let candidate_implementation = legal_worst_setup_route_eco_candidate(
            design,
            architecture.device(),
            implementation,
            routing_constraints,
            routing_costs,
            candidate,
            &cohort,
            routing_workspace,
        )?;
        let route_elapsed = route_started.elapsed();
        let Some(candidate_implementation) = candidate_implementation else {
            if metrics_enabled() {
                eprintln!(
                    "[metrics] worst_setup_net_route_eco candidate={} net={} sink={} cohort_nets={} slack={} route_delay={} shared_prefix_delay={} worst_sinks={} fanout={} routed=false route_ms={:.3} sta_ms=0.000 accepted=false",
                    ordinal,
                    candidate.net.0,
                    candidate.sink.0,
                    cohort.len(),
                    candidate.slack_ps,
                    candidate.route_delay_ps,
                    candidate.shared_prefix_delay_ps,
                    candidate.worst_sinks,
                    candidate.fanout,
                    route_elapsed.as_secs_f64() * 1_000.0,
                );
            }
            continue;
        };
        let sta_started = Instant::now();
        let candidate_timing = timing_session.analyze(&candidate_implementation)?;
        let sta_elapsed = sta_started.elapsed();
        progress(timing_snapshot(&candidate_timing));
        let candidate_objective = timing_objective(&candidate_timing);
        let improves = strictly_improves_timing_objective(candidate_objective, objective);
        progress(Ecp5FlowStage::TimingTrialDecision {
            improves_objective: improves,
        });
        if metrics_enabled() {
            eprintln!(
                "[metrics] worst_setup_net_route_eco candidate={} net={} sink={} cohort_nets={} slack={} route_delay={} shared_prefix_delay={} worst_sinks={} fanout={} routed=true route_ms={:.3} sta_ms={:.3} wns={:?} accepted={improves}",
                ordinal,
                candidate.net.0,
                candidate.sink.0,
                cohort.len(),
                candidate.slack_ps,
                candidate.route_delay_ps,
                candidate.shared_prefix_delay_ps,
                candidate.worst_sinks,
                candidate.fanout,
                route_elapsed.as_secs_f64() * 1_000.0,
                sta_elapsed.as_secs_f64() * 1_000.0,
                candidate_timing.worst_slack_ps,
            );
        }
        let committed = commit_strict_route_eco_candidate(
            implementation,
            timing,
            candidate_implementation,
            candidate_timing,
        );
        debug_assert_eq!(committed, improves);
        if committed {
            objective = candidate_objective;
            accepted += 1;
            if timing.worst_slack_ps.is_some_and(|slack| slack >= 0) {
                break;
            }
        }
    }
    if metrics_enabled() {
        eprintln!(
            "[metrics] worst_setup_net_route_eco_summary candidates={} accepted={accepted} total_ms={:.3}",
            worklist.trials() - initial_trials,
            eco_started.elapsed().as_secs_f64() * 1_000.0,
        );
    }
    Ok(())
}

fn staged_timing_objective(
    setup_score: ViolationScore,
    hold_score: ViolationScore,
    setup_slack: i128,
    hold_slack: i128,
) -> TimingObjective {
    if setup_slack < 0 {
        TimingObjective {
            setup_closed: false,
            active_worst_slack_ps: setup_slack,
            active_violations: setup_score,
            inactive_worst_slack_ps: hold_slack,
        }
    } else {
        TimingObjective {
            setup_closed: true,
            active_worst_slack_ps: hold_slack,
            active_violations: hold_score,
            inactive_worst_slack_ps: setup_slack,
        }
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

fn ecp5_timing_constraints(
    design: &Design,
    packing: &Ecp5Packing,
    pll_relations: &GeneratedClockRelations,
) -> Result<TimingConstraints, Ecp5FlowError> {
    // Match nextpnr's ECP5 QoR model: constraints provide only the nominal
    // period. Project Trellis cell/PIP min/max values are applied by STA, but
    // clock-tree skew, PLL jitter, and additional setup uncertainty are not
    // deducted from that period by default. The caller can apply explicit
    // setup uncertainty after all generated and promoted clocks are resolved.
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
    for (&net, relation) in pll_relations {
        constraints.set_generated_clock(
            net,
            relation.source,
            relation.multiply_by,
            relation.divide_by,
            relation.phase_ps,
        );
    }
    for clock in packing.global_clocks() {
        if let Some(&period_ps) = constraints.clock_periods_ps().get(&clock.source_net) {
            insert_clock_period(&mut constraints, clock.global_net, period_ps)?;
            // A DCCA changes only physical distribution. Retain the source
            // clock waveform explicitly so generated-clock relationships keep
            // working after one or more promotion stages.
            constraints.set_generated_clock(clock.global_net, clock.source_net, 1, 1, 0);
        }
    }
    Ok(constraints)
}

fn apply_setup_uncertainty(constraints: &mut TimingConstraints, uncertainty_ps: u64) {
    let clocks = constraints
        .clock_periods_ps()
        .keys()
        .copied()
        .collect::<Vec<_>>();
    for clock in clocks {
        constraints.set_setup_uncertainty_ps(clock, uncertainty_ps);
    }
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
    metadata: &BTreeMap<CellId, PrimitiveMetadata>,
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
        let primitive_metadata = metadata.get(&cell_id);
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
                model.add_clock_to_q(
                    from,
                    to,
                    primitive_clock_edge(primitive_metadata, &arc.from_pin)
                        .unwrap_or(TimingClockEdge::Rising),
                    delay,
                )?;
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
            primitive_metadata,
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
    metadata: Option<&PrimitiveMetadata>,
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
            primitive_clock_edge(metadata, &check.clock_pin).unwrap_or(TimingClockEdge::Rising),
            timing_delay(check.setup)?,
            timing_delay(check.hold)?,
        )?;
    }
    Ok(())
}

fn primitive_clock_edge(
    metadata: Option<&PrimitiveMetadata>,
    clock_pin: &str,
) -> Option<TimingClockEdge> {
    let edge = match metadata? {
        PrimitiveMetadata::FlipFlop { edge, .. } => *edge,
        PrimitiveMetadata::BlockRam {
            edge, second_port, ..
        } => {
            if clock_pin == "CLKB" {
                second_port.map_or(*edge, |port| port.edge)
            } else {
                *edge
            }
        }
        PrimitiveMetadata::DistributedRam {
            role: DistributedRamRole::Data(_),
            edge,
            ..
        } if clock_pin == "WCK" => *edge,
        _ => return None,
    };
    Some(match edge {
        texo_struo::ClockEdge::Rising => TimingClockEdge::Rising,
        texo_struo::ClockEdge::Falling => TimingClockEdge::Falling,
    })
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
    /// The architecture delay predictor could not be constructed.
    PlacementDelay(Ecp5DelayPredictorError),
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
    /// An explicit internal/source clock constraint cannot be applied.
    InvalidClockConstraint {
        /// Requested source cell name.
        cell: String,
        /// Requested source pin name.
        pin: String,
        /// Exact reason the constraint was rejected.
        reason: String,
    },
    /// More than one source assigned different periods to one logical net.
    ConflictingClockPeriods {
        /// Logical clock net.
        net: NetId,
    },
    /// More than one primitive assigned different relationships to one clock.
    ConflictingClockRelations {
        /// Logical generated clock net.
        net: NetId,
    },
    /// Legacy error for a missing PLL frequency attribute.
    #[deprecated(note = "PLL periods now derive from LPF constraints and physical dividers")]
    MissingPllOutputFrequency {
        /// Logical PLL cell name.
        cell: String,
        /// Required frequency attribute.
        attribute: String,
    },
    /// Legacy error for an invalid PLL frequency attribute.
    #[deprecated(note = "PLL periods now derive from LPF constraints and physical dividers")]
    InvalidPllOutputFrequency {
        /// Logical PLL cell name.
        cell: String,
        /// Attribute containing the invalid value.
        attribute: String,
        /// Invalid attribute value.
        value: String,
    },
    /// An imported PLL has no connected `CLKI` pin.
    MissingPllInputClock {
        /// Logical PLL cell name.
        cell: String,
    },
    /// The PLL input net has no resolvable clock-period constraint.
    MissingPllInputClockConstraint {
        /// Logical PLL cell name.
        cell: String,
        /// Unconstrained PLL input net.
        net: NetId,
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
    /// One PLL integer parameter is absent, malformed, or outside its range.
    InvalidPllParameter {
        /// Logical PLL cell name.
        cell: String,
        /// Parameter name.
        parameter: String,
        /// Raw value, when present.
        value: Option<String>,
    },
    /// Legacy error for an invalid PLL input divider.
    #[deprecated(note = "invalid PLL integers are now reported by InvalidPllParameter")]
    InvalidPllInputDivider {
        /// Logical PLL cell name.
        cell: String,
        /// Raw divider value, when present.
        value: Option<String>,
    },
    /// Legacy error for the removed phase-detector cutoff policy.
    #[deprecated(note = "Texo no longer invents a phase-detector frequency cutoff")]
    PllPhaseDetectorTooSlow {
        /// Logical PLL cell name.
        cell: String,
        /// Phase-detector period in picoseconds.
        period_ps: u64,
    },
    /// A PLL output cannot be represented as an exact related clock.
    UnsupportedPllClockRelation {
        /// Logical PLL cell name.
        cell: String,
        /// Primitive output port.
        output: String,
        /// Exact reason the relationship was rejected.
        reason: String,
    },
    /// Static timing analysis failed.
    Timing(TimingError),
}

#[allow(deprecated, clippy::too_many_lines)]
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
            Self::PlacementDelay(error) => {
                write!(f, "ECP5 placement delay prediction failed: {error}")
            }
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
            Self::InvalidClockConstraint { cell, pin, reason } => {
                write!(f, "invalid clock constraint `{cell}.{pin}`: {reason}")
            }
            Self::ConflictingClockPeriods { net } => {
                write!(f, "clock net {} has conflicting periods", net.0)
            }
            Self::ConflictingClockRelations { net } => {
                write!(
                    f,
                    "clock net {} has conflicting generated-clock relationships",
                    net.0
                )
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
            Self::MissingPllInputClock { cell } => {
                write!(f, "PLL `{cell}` has no connected CLKI pin")
            }
            Self::MissingPllInputClockConstraint { cell, net } => write!(
                f,
                "PLL `{cell}` input net {} has no resolvable clock-period constraint",
                net.0
            ),
            Self::MissingPllOutputPin { cell, pin } => {
                write!(f, "PLL `{cell}` has no selected output pin `{pin}`")
            }
            Self::MissingPllOutputNet { cell, pin } => {
                write!(f, "PLL `{cell}` output pin `{pin}` does not drive a net")
            }
            Self::InvalidPllParameter {
                cell,
                parameter,
                value,
            } => write!(
                f,
                "PLL `{cell}` has invalid {parameter} value {}",
                value.as_deref().unwrap_or("<missing>")
            ),
            Self::InvalidPllInputDivider { cell, value } => write!(
                f,
                "PLL `{cell}` has invalid CLKI_DIV value {}",
                value.as_deref().unwrap_or("<missing>")
            ),
            Self::PllPhaseDetectorTooSlow { cell, period_ps } => write!(
                f,
                "PLL `{cell}` phase-detector period {period_ps} ps is not below the 100000 ps guaranteed-jitter limit"
            ),
            Self::UnsupportedPllClockRelation {
                cell,
                output,
                reason,
            } => write!(
                f,
                "PLL `{cell}` output `{output}` has no exact generated-clock model: {reason}"
            ),
            Self::Timing(error) => write!(f, "ECP5 static timing analysis failed: {error}"),
        }
    }
}

#[allow(deprecated)]
impl Error for Ecp5FlowError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Lpf(error) => Some(error),
            Self::Packing(error) => Some(error),
            Self::Pnr(error) => Some(error),
            Self::PlacementDelay(error) => Some(error),
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
            | Self::InvalidClockConstraint { .. }
            | Self::ConflictingClockPeriods { .. }
            | Self::ConflictingClockRelations { .. }
            | Self::MissingPllOutputFrequency { .. }
            | Self::InvalidPllOutputFrequency { .. }
            | Self::MissingPllInputClock { .. }
            | Self::MissingPllInputClockConstraint { .. }
            | Self::MissingPllOutputPin { .. }
            | Self::MissingPllOutputNet { .. }
            | Self::InvalidPllParameter { .. }
            | Self::InvalidPllInputDivider { .. }
            | Self::PllPhaseDetectorTooSlow { .. }
            | Self::UnsupportedPllClockRelation { .. } => None,
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

impl From<Ecp5DelayPredictorError> for Ecp5FlowError {
    fn from(value: Ecp5DelayPredictorError) -> Self {
        Self::PlacementDelay(value)
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
        BelId, CellId, CellPinId, Design, Device, NetId, PinDirection, PipId, Point, ResourceKind,
        WireId,
    };
    use texo_pnr::{
        NetRoute, PlacementConstraints, PnrResult, RouteArc, RoutingConstraints,
        placement_from_partial_bindings, rebind_placement_pins,
    };
    use texo_struo::{
        ActiveLevel as ImportedActiveLevel, BlockRamPortMetadata, ClockEdge as ImportedClockEdge,
        PllOutput, PrimitiveMetadata, ResetMetadata, import_ecp5,
    };
    use texo_target_ecp5::{
        ArchitectureFile, Ecp5Packing, GlobalClockRequirement, PipClassTimingRecord, PipRecord,
        RelativeRef, ResolvedLpf, TimingCornersRecord, expand, find_global_clock_requirements,
        pack_lut_ffs, pack_lut_ffs_excluding, parse_lpf, read_architecture, resolve_lpf_port_cells,
    };
    use texo_timing::{
        ClockEdge as TimingClockEdge, DelayRange, NetDelay, NetSetupCriticality, NetSetupSlack,
        SetupCheck, TimingConstraints, TimingError, TimingModel, TimingReport,
        analyze_timing_from_net_delays,
    };

    use super::{
        Ecp5EcoTimingSession, Ecp5FlowError, Ecp5FlowOptions, Evidence, Gate,
        GeneratedClockRelations, PostMapSimulationPolicy, WorstSetupNetRouteEcoCandidate,
        WorstSetupRouteEcoWorklist, accumulate_hold_minimums, commit_strict_route_eco_candidate,
        constrain_pll_outputs, criticality_weight, ecp5_placement_criticality_weight,
        ecp5_timing_constraints, ecp5_timing_model, ecp5_timing_placement_weights,
        ff_ce_control_sets, ff_clock_control_sets, ff_lsr_control_sets, find_cell_pin,
        freeze_unchanged_routes, implement, implement_struo_ecp5, implement_with_constraints,
        is_ecp5_span6_continuation, pip_class_delay, placement_bbox_hpwl, placement_identity,
        placement_star_length, predicted_setup_candidate_is_pareto_dominated, primitive_clock_edge,
        project_trellis_speed_grade, run_setup_feedback_fallback, slack_violations,
        staged_timing_objective, strictly_improves_timing_objective, verify_post_map_with_celox,
        worst_setup_net_route_eco_candidates, worst_setup_route_eco_cohort,
    };

    const ECP5_FIXTURE: &str = include_str!("../../texo-target-ecp5/fixtures/minimal-ecp5.json");

    #[test]
    fn timing_gate_rejects_an_unconstrained_domain_even_when_checked_paths_pass() {
        let mut design = Design::new();
        let mut model = TimingModel::new();
        let mut delays = Vec::new();
        let mut clocks = Vec::new();
        for name in ["cpu", "jtag"] {
            let source = design.add_cell(format!("{name}_clock"), ResourceKind::Io);
            let output = design.add_pin(source, "O", PinDirection::Output).unwrap();
            let ff = design.add_cell(name, ResourceKind::Register);
            let clock = design.add_pin(ff, "CLK", PinDirection::Input).unwrap();
            let data = design.add_pin(ff, "DI", PinDirection::Input).unwrap();
            let q = design.add_pin(ff, "Q", PinDirection::Output).unwrap();
            let clock_net = design
                .add_net(format!("{name}_clock"), output, [clock])
                .unwrap();
            let data_net = design.add_net(format!("{name}_data"), q, [data]).unwrap();
            clocks.push(clock_net);
            delays.extend([
                NetDelay {
                    net: clock_net,
                    sink: clock,
                    delay: DelayRange::zero(),
                },
                NetDelay {
                    net: data_net,
                    sink: data,
                    delay: DelayRange::new(30, 30).unwrap(),
                },
            ]);
            model
                .add_clock_to_q(
                    clock,
                    q,
                    TimingClockEdge::Rising,
                    DelayRange::new(40, 40).unwrap(),
                )
                .unwrap();
            model
                .add_setup_hold(
                    clock,
                    data,
                    TimingClockEdge::Rising,
                    DelayRange::new(10, 10).unwrap(),
                    DelayRange::new(10, 10).unwrap(),
                )
                .unwrap();
        }
        let mut constraints = TimingConstraints::new();
        constraints.set_clock_period_ps(clocks[0], 1_000);
        let mut evidence = Evidence::new();
        for gate in super::REQUIRED_GATES {
            evidence.record(gate);
        }
        let report =
            analyze_timing_from_net_delays(&design, &model, &constraints, delays.clone()).unwrap();
        assert!(report.met_timing());
        assert_eq!(report.setup_checks.len(), 1);
        assert_eq!(report.unchecked_endpoints.len(), 1);
        assert_eq!(
            report.unchecked_endpoints[0].reason.as_str(),
            "unconstrained_clock"
        );
        evidence.record_timing(&design, &report, &[]);
        assert!(!evidence.contains(Gate::TimingClosure));
        assert!(evidence.authorize_bitstream().is_err());

        constraints.set_clock_period_ps(clocks[1], 1_000);
        let report =
            analyze_timing_from_net_delays(&design, &model, &constraints, delays.clone()).unwrap();
        assert_eq!(report.setup_checks.len(), 2);
        assert!(report.all_modeled_endpoints_checked());
        evidence.record_timing(&design, &report, &[]);
        assert!(evidence.authorize_bitstream().is_ok());

        constraints.set_clock_period_ps(clocks[1], 50);
        let report = analyze_timing_from_net_delays(&design, &model, &constraints, delays).unwrap();
        assert!(!report.met_timing());
        evidence.record_timing(&design, &report, &[]);
        assert!(!evidence.contains(Gate::TimingClosure));
    }

    #[test]
    #[allow(deprecated)]
    fn legacy_pll_error_variants_keep_their_display_contract() {
        assert_eq!(
            Ecp5FlowError::MissingPllOutputFrequency {
                cell: "pll".into(),
                attribute: "FREQUENCY_PIN_CLKOS".into(),
            }
            .to_string(),
            "PLL `pll` requires generated-clock attribute `FREQUENCY_PIN_CLKOS`"
        );
        assert_eq!(
            Ecp5FlowError::InvalidPllOutputFrequency {
                cell: "pll".into(),
                attribute: "FREQUENCY_PIN_CLKOS".into(),
                value: "bad".into(),
            }
            .to_string(),
            "PLL `pll` attribute `FREQUENCY_PIN_CLKOS` has invalid MHz value `bad`"
        );
        assert_eq!(
            Ecp5FlowError::InvalidPllInputDivider {
                cell: "pll".into(),
                value: None,
            }
            .to_string(),
            "PLL `pll` has invalid CLKI_DIV value <missing>"
        );
        assert_eq!(
            Ecp5FlowError::PllPhaseDetectorTooSlow {
                cell: "pll".into(),
                period_ps: 100_000,
            }
            .to_string(),
            "PLL `pll` phase-detector period 100000 ps is not below the 100000 ps guaranteed-jitter limit"
        );
    }

    #[test]
    fn placement_metrics_distinguish_net_bbox_from_fanout_star_length() {
        let mut design = Design::new();
        let driver = design.add_cell("driver", ResourceKind::Logic);
        let output = design.add_pin(driver, "out", PinDirection::Output).unwrap();
        let near = design.add_cell("near", ResourceKind::Logic);
        let near_input = design.add_pin(near, "in", PinDirection::Input).unwrap();
        let far = design.add_cell("far", ResourceKind::Logic);
        let far_input = design.add_pin(far, "in", PinDirection::Input).unwrap();
        design
            .add_net("fanout", output, [near_input, far_input])
            .unwrap();
        let device = Device::rectangular_logic(3, 1).unwrap();
        let placement = placement_from_partial_bindings(
            &design,
            &device,
            &PlacementConstraints::new(),
            &BTreeMap::from([(driver, BelId(0)), (near, BelId(1)), (far, BelId(2))]),
        )
        .unwrap();

        assert_eq!(placement_bbox_hpwl(&design, &device, &placement), 2);
        assert_eq!(placement_star_length(&design, &device, &placement), 3);
    }

    #[test]
    fn derives_all_pll_outputs_from_lpf_and_dividers() {
        let fixture = DualPllFixture::new();
        let mut packing = packing_with_input_clock(&fixture.design, fixture.input, 12_000_000);
        let metadata = dual_output_pll_metadata(fixture.pll);

        let relations = constrain_pll_outputs(&fixture.design, &metadata, &mut packing).unwrap();

        assert_eq!(
            packing.generated_clock_periods_ps(),
            &BTreeMap::from([(fixture.cpu_net, 8_013), (fixture.memory_net, 16_026)])
        );
        let input_net = fixture.design.pins()[find_cell_pin(&fixture.design, fixture.pll, "CLKI")
            .unwrap()
            .0]
            .net()
            .unwrap();
        let cpu = relations[&fixture.cpu_net];
        assert_eq!(cpu.source, input_net);
        assert_eq!((cpu.multiply_by, cpu.divide_by, cpu.phase_ps), (52, 5, 0));
        let memory = relations[&fixture.memory_net];
        assert_eq!(memory.source, fixture.cpu_net);
        assert_eq!(
            (memory.multiply_by, memory.divide_by, memory.phase_ps),
            (1, 2, 0)
        );
    }

    #[test]
    fn explicit_periods_cannot_override_lpf_or_pll_derived_clocks() {
        let fixture = DualPllFixture::new();
        let metadata = dual_output_pll_metadata(fixture.pll);
        let mut reference = packing_with_input_clock(&fixture.design, fixture.input, 12_000_000);
        let expected = constrain_pll_outputs(&fixture.design, &metadata, &mut reference).unwrap();
        for (cell, pin, period_ps, passes) in [
            ("pll", "CLKOS", 8_013, true),
            ("pll", "CLKOS", 8_100, false),
            ("clock_input", "O", 40_000, false),
        ] {
            let mut packing = packing_with_input_clock(&fixture.design, fixture.input, 12_000_000);
            super::clock_constraints::apply_clock_constraints(
                &fixture.design,
                &mut packing,
                &[super::ClockConstraint {
                    cell: cell.into(),
                    pin: pin.into(),
                    period_ps,
                }],
            )
            .unwrap();
            let result = constrain_pll_outputs(&fixture.design, &metadata, &mut packing);
            if passes {
                assert_eq!(result.unwrap(), expected);
            } else {
                assert!(matches!(
                    result,
                    Err(Ecp5FlowError::ConflictingClockPeriods { .. })
                ));
            }
        }
    }

    #[test]
    fn lpf_input_clock_wins_and_clki_div_defaults_to_one() {
        let fixture = DualPllFixture::new();
        let mut packing = packing_with_input_clock(&fixture.design, fixture.input, 24_000_000);
        let metadata = dual_output_pll_metadata(fixture.pll);
        assert!(!pll_parameters(&metadata).contains_key("CLKI_DIV"));

        constrain_pll_outputs(&fixture.design, &metadata, &mut packing).unwrap();

        // FREQUENCY_PIN_* intentionally claims unrelated values. The physical
        // dividers and the 24 MHz LPF input constraint are authoritative.
        assert_eq!(
            packing.generated_clock_periods_ps()[&fixture.cpu_net],
            4_006
        );
        assert_eq!(
            packing.generated_clock_periods_ps()[&fixture.memory_net],
            8_013
        );
    }

    #[test]
    fn omitted_pll_output_dividers_use_the_bitstream_default() {
        let fixture = DualPllFixture::new();
        let mut packing = packing_with_input_clock(&fixture.design, fixture.input, 12_000_000);
        let mut metadata = dual_output_pll_metadata(fixture.pll);
        let parameters = pll_parameters_mut(&mut metadata);
        parameters.remove("CLKOP_DIV");
        parameters.remove("CLKOS_DIV");
        parameters.remove("CLKOS2_DIV");

        let relations = constrain_pll_outputs(&fixture.design, &metadata, &mut packing).unwrap();

        // CLKOP is the INT_OP feedback divider while CLKOS/CLKOS2 are the two
        // fabric dividers. All three omitted parameters program divide-by-eight;
        // do not copy nextpnr 0.6 pack.cc's timing-only fallback of one.
        assert_eq!(
            packing.generated_clock_periods_ps(),
            &BTreeMap::from([(fixture.cpu_net, 83_333), (fixture.memory_net, 83_333),])
        );
        let memory = relations[&fixture.memory_net];
        assert_eq!((memory.multiply_by, memory.divide_by), (1, 1));
    }

    #[test]
    fn resolves_pll_input_from_an_existing_generated_clock_period() {
        let fixture = DualPllFixture::new();
        let input_net = fixture.design.pins()[find_cell_pin(&fixture.design, fixture.pll, "CLKI")
            .unwrap()
            .0]
            .net()
            .unwrap();
        let mut packing = Ecp5Packing::default();
        packing.set_generated_clock_period_ps(input_net, 10_000);

        constrain_pll_outputs(
            &fixture.design,
            &dual_output_pll_metadata(fixture.pll),
            &mut packing,
        )
        .unwrap();

        assert_eq!(packing.generated_clock_periods_ps()[&fixture.cpu_net], 962);
        assert_eq!(
            packing.generated_clock_periods_ps()[&fixture.memory_net],
            1_923
        );
    }

    #[test]
    #[allow(clippy::too_many_lines)]
    fn resolves_cascaded_plls_through_a_promoted_dcca_in_dependency_order() {
        let architecture = read_architecture(ECP5_FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let input = design.add_cell("clock_input", ResourceKind::Io);
        let input_o = design.add_pin(input, "O", PinDirection::Output).unwrap();

        // Give the downstream PLL the smaller CellId so metadata iteration is
        // deliberately opposite the clock dependency order.
        let downstream = design.add_cell("downstream_pll", ResourceKind::Logic);
        let downstream_clki = design
            .add_pin(downstream, "CLKI", PinDirection::Input)
            .unwrap();
        let downstream_clkos = design
            .add_pin(downstream, "CLKOS", PinDirection::Output)
            .unwrap();
        let upstream = design.add_cell("upstream_pll", ResourceKind::Logic);
        let upstream_clki = design
            .add_pin(upstream, "CLKI", PinDirection::Input)
            .unwrap();
        let upstream_clkos = design
            .add_pin(upstream, "CLKOS", PinDirection::Output)
            .unwrap();
        let launch = add_timing_register(&mut design, "upstream_launch");
        let reference = design
            .add_net("reference", input_o, [upstream_clki])
            .unwrap();
        let bridge = design
            .add_net(
                "pll_bridge",
                upstream_clkos,
                [downstream_clki, launch.clock],
            )
            .unwrap();
        let capture = add_timing_register(&mut design, "downstream_capture");
        let output = design
            .add_net("cascaded_clock", downstream_clkos, [capture.clock])
            .unwrap();
        design
            .add_net("cascaded_data", launch.q, [capture.data])
            .unwrap();

        let mut packing = Ecp5Packing::default();
        packing
            .promote_global_clocks(
                &mut design,
                &architecture,
                [GlobalClockRequirement { net: bridge }],
            )
            .unwrap();
        apply_input_clock(&mut packing, &design, input, 12_000_000);
        let parameters = |feedback_divider: &str, output_divider: &str| {
            BTreeMap::from([
                ("CLKFB_DIV".into(), feedback_divider.into()),
                ("FEEDBK_PATH".into(), "INT_OP".into()),
                ("CLKOP_DIV".into(), "8".into()),
                ("CLKOS_DIV".into(), output_divider.into()),
                ("CLKOS_ENABLE".into(), "ENABLED".into()),
                ("OUTDIVIDER_MUXB".into(), "DIVB".into()),
            ])
        };
        let metadata = BTreeMap::from([
            (
                downstream,
                PrimitiveMetadata::Pll {
                    fabric_output: PllOutput::Clkos,
                    feedback_output: PllOutput::Clkintfb,
                    parameters: parameters("1", "16"),
                    attributes: BTreeMap::new(),
                },
            ),
            (
                upstream,
                PrimitiveMetadata::Pll {
                    fabric_output: PllOutput::Clkos,
                    feedback_output: PllOutput::Clkintfb,
                    parameters: parameters("5", "4"),
                    attributes: BTreeMap::new(),
                },
            ),
        ]);

        let relations = constrain_pll_outputs(&design, &metadata, &mut packing).unwrap();

        assert_eq!(packing.generated_clock_periods_ps()[&bridge], 8_333);
        assert_eq!(packing.generated_clock_periods_ps()[&output], 16_667);
        let promoted = packing.global_clocks()[0].global_net;
        let upstream_relation = relations[&bridge];
        assert_eq!(upstream_relation.source, reference);
        assert_eq!(
            (upstream_relation.multiply_by, upstream_relation.divide_by),
            (10, 1)
        );
        let downstream_relation = relations[&output];
        assert_eq!(downstream_relation.source, promoted);
        assert_eq!(
            (
                downstream_relation.multiply_by,
                downstream_relation.divide_by
            ),
            (1, 2)
        );
        let constraints = ecp5_timing_constraints(&design, &packing, &relations).unwrap();
        assert_eq!(constraints.clock_periods_ps()[&promoted], 8_333);

        let mut model = TimingModel::new();
        model
            .add_clock_to_q(
                launch.clock,
                launch.q,
                TimingClockEdge::Rising,
                DelayRange::zero(),
            )
            .unwrap();
        model
            .add_setup_hold(
                capture.clock,
                capture.data,
                TimingClockEdge::Rising,
                DelayRange::zero(),
                DelayRange::zero(),
            )
            .unwrap();
        let delays = design
            .nets()
            .iter()
            .enumerate()
            .flat_map(|(index, net)| {
                net.sinks.iter().map(move |&sink| NetDelay {
                    net: NetId(index),
                    sink,
                    delay: DelayRange::zero(),
                })
            })
            .collect();
        let report = analyze_timing_from_net_delays(&design, &model, &constraints, delays).unwrap();
        assert!(
            report
                .setup_checks
                .iter()
                .any(|check| check.data_pin == capture.data && check.clock_net == output)
        );
    }

    #[test]
    fn does_not_invent_a_phase_detector_frequency_cutoff() {
        let fixture = DualPllFixture::new();
        let mut packing = packing_with_input_clock(&fixture.design, fixture.input, 6_000_000);

        constrain_pll_outputs(
            &fixture.design,
            &dual_output_pll_metadata(fixture.pll),
            &mut packing,
        )
        .unwrap();

        assert_eq!(
            packing.generated_clock_periods_ps()[&fixture.cpu_net],
            16_026
        );
        assert_eq!(
            packing.generated_clock_periods_ps()[&fixture.memory_net],
            32_051
        );
    }

    #[test]
    fn rejects_disabled_bypassed_dynamic_and_phase_shifted_outputs() {
        for (parameter, value, expected) in [
            ("CLKOS2_ENABLE", "DISABLED", "not enabled"),
            ("OUTDIVIDER_MUXC", "REFCLK", "bypassed"),
            ("DPHASE_SOURCE", "ENABLED", "runtime phase"),
            ("CLKOS_FPHASE", "1", "absolute output phase"),
            ("CLKOS2_FPHASE", "1", "phase differs"),
        ] {
            let fixture = DualPllFixture::new();
            let mut packing = packing_with_input_clock(&fixture.design, fixture.input, 12_000_000);
            let mut metadata = dual_output_pll_metadata(fixture.pll);
            pll_parameters_mut(&mut metadata).insert(parameter.into(), value.into());

            let error =
                constrain_pll_outputs(&fixture.design, &metadata, &mut packing).unwrap_err();

            assert!(matches!(
                error,
                Ecp5FlowError::UnsupportedPllClockRelation { reason, .. }
                    if reason.contains(expected)
            ));
        }
    }

    #[test]
    fn rejects_clkintfb_as_a_fabric_output() {
        let fixture = DualPllFixture::new();
        let mut packing = packing_with_input_clock(&fixture.design, fixture.input, 12_000_000);
        let mut metadata = dual_output_pll_metadata(fixture.pll);
        let PrimitiveMetadata::Pll { fabric_output, .. } = metadata.get_mut(&fixture.pll).unwrap()
        else {
            unreachable!();
        };
        *fabric_output = PllOutput::Clkintfb;

        let error = constrain_pll_outputs(&fixture.design, &metadata, &mut packing).unwrap_err();

        assert!(matches!(
            error,
            Ecp5FlowError::UnsupportedPllClockRelation { output, .. }
                if output == "CLKINTFB"
        ));
    }

    #[test]
    fn checks_pll_clock_domains_in_both_directions_after_global_promotion() {
        let architecture = read_architecture(ECP5_FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let input = design.add_cell("clock_input", ResourceKind::Io);
        let input_o = design.add_pin(input, "O", PinDirection::Output).unwrap();
        let pll = design.add_cell("pll", ResourceKind::Logic);
        let clki = design.add_pin(pll, "CLKI", PinDirection::Input).unwrap();
        let clkos = design.add_pin(pll, "CLKOS", PinDirection::Output).unwrap();
        let clkos2 = design.add_pin(pll, "CLKOS2", PinDirection::Output).unwrap();
        design.add_net("pll_input", input_o, [clki]).unwrap();
        let fast_launch = add_timing_register(&mut design, "fast_launch");
        let fast_capture = add_timing_register(&mut design, "fast_capture");
        let slow_launch = add_timing_register(&mut design, "slow_launch");
        let slow_capture = add_timing_register(&mut design, "slow_capture");
        let fast_source = design
            .add_net(
                "cpu_clock_source",
                clkos,
                [fast_launch.clock, fast_capture.clock],
            )
            .unwrap();
        let slow_source = design
            .add_net(
                "memory_clock_source",
                clkos2,
                [slow_launch.clock, slow_capture.clock],
            )
            .unwrap();
        design
            .add_net("cpu_to_memory", fast_launch.q, [slow_capture.data])
            .unwrap();
        design
            .add_net("memory_to_cpu", slow_launch.q, [fast_capture.data])
            .unwrap();

        let mut packing = Ecp5Packing::default();
        packing
            .promote_global_clocks(
                &mut design,
                &architecture,
                [GlobalClockRequirement { net: fast_source }],
            )
            .unwrap();
        apply_input_clock(&mut packing, &design, input, 12_000_000);
        let metadata = dual_output_pll_metadata(pll);
        let relations = constrain_pll_outputs(&design, &metadata, &mut packing).unwrap();
        let constraints = ecp5_timing_constraints(&design, &packing, &relations).unwrap();

        let mut model = TimingModel::new();
        for register in [&fast_launch, &slow_launch] {
            model
                .add_clock_to_q(
                    register.clock,
                    register.q,
                    TimingClockEdge::Rising,
                    DelayRange::zero(),
                )
                .unwrap();
        }
        for register in [&fast_capture, &slow_capture] {
            model
                .add_setup_hold(
                    register.clock,
                    register.data,
                    TimingClockEdge::Rising,
                    DelayRange::zero(),
                    DelayRange::zero(),
                )
                .unwrap();
        }
        let delays = design
            .nets()
            .iter()
            .enumerate()
            .flat_map(|(index, net)| {
                net.sinks.iter().map(move |&sink| NetDelay {
                    net: NetId(index),
                    sink,
                    delay: DelayRange::zero(),
                })
            })
            .collect();
        let report = analyze_timing_from_net_delays(&design, &model, &constraints, delays).unwrap();

        let fast_global = packing.global_clocks()[0].global_net;
        assert!(report.all_modeled_endpoints_checked());
        assert!(report.setup_checks.iter().any(|check| {
            check.data_pin == slow_capture.data && check.clock_net == slow_source
        }));
        assert!(report.setup_checks.iter().any(|check| {
            check.data_pin == fast_capture.data && check.clock_net == fast_global
        }));
    }

    struct DualPllFixture {
        design: Design,
        input: CellId,
        pll: CellId,
        cpu_net: NetId,
        memory_net: NetId,
    }

    impl DualPllFixture {
        fn new() -> Self {
            let mut design = Design::new();
            let input = design.add_cell("clock_input", ResourceKind::Io);
            let input_o = design.add_pin(input, "O", PinDirection::Output).unwrap();
            let pll = design.add_cell("pll", ResourceKind::Logic);
            let clki = design.add_pin(pll, "CLKI", PinDirection::Input).unwrap();
            let clkos = design.add_pin(pll, "CLKOS", PinDirection::Output).unwrap();
            let clkos2 = design.add_pin(pll, "CLKOS2", PinDirection::Output).unwrap();
            design.add_net("pll_input", input_o, [clki]).unwrap();
            let cpu = design.add_cell("cpu", ResourceKind::Register);
            let cpu_clock = design.add_pin(cpu, "CLK", PinDirection::Input).unwrap();
            let memory = design.add_cell("memory", ResourceKind::Memory);
            let memory_clock = design.add_pin(memory, "CLK", PinDirection::Input).unwrap();
            let cpu_net = design.add_net("cpu_clock", clkos, [cpu_clock]).unwrap();
            let memory_net = design
                .add_net("memory_clock", clkos2, [memory_clock])
                .unwrap();
            Self {
                design,
                input,
                pll,
                cpu_net,
                memory_net,
            }
        }
    }

    fn packing_with_input_clock(design: &Design, input: CellId, frequency_hz: u64) -> Ecp5Packing {
        let mut packing = Ecp5Packing::default();
        apply_input_clock(&mut packing, design, input, frequency_hz);
        packing
    }

    fn apply_input_clock(
        packing: &mut Ecp5Packing,
        design: &Design,
        input: CellId,
        frequency_hz: u64,
    ) {
        let architecture = read_architecture(ECP5_FIXTURE.as_bytes()).unwrap();
        packing
            .apply_resolved_lpf(
                design,
                &architecture,
                "CABGA381",
                &ResolvedLpf {
                    clock_frequencies_hz: BTreeMap::from([(input, frequency_hz)]),
                    ..ResolvedLpf::default()
                },
            )
            .unwrap();
    }

    fn dual_output_pll_metadata(pll: CellId) -> BTreeMap<CellId, PrimitiveMetadata> {
        BTreeMap::from([(
            pll,
            PrimitiveMetadata::Pll {
                fabric_output: PllOutput::Clkos,
                feedback_output: PllOutput::Clkintfb,
                parameters: BTreeMap::from([
                    ("CLKFB_DIV".into(), "1".into()),
                    ("FEEDBK_PATH".into(), "INT_OP".into()),
                    ("CLKOP_DIV".into(), "52".into()),
                    ("CLKOS_ENABLE".into(), "ENABLED".into()),
                    ("CLKOS_DIV".into(), "5".into()),
                    ("CLKOS_CPHASE".into(), "0".into()),
                    ("CLKOS_FPHASE".into(), "0".into()),
                    ("CLKOS2_ENABLE".into(), "ENABLED".into()),
                    ("CLKOS2_DIV".into(), "10".into()),
                    ("CLKOS2_CPHASE".into(), "0".into()),
                    ("CLKOS2_FPHASE".into(), "0".into()),
                    ("OUTDIVIDER_MUXB".into(), "DIVB".into()),
                    ("OUTDIVIDER_MUXC".into(), "DIVC".into()),
                ]),
                attributes: BTreeMap::from([
                    ("FREQUENCY_PIN_CLKI".into(), "1".into()),
                    ("FREQUENCY_PIN_CLKOS".into(), "999".into()),
                    ("FREQUENCY_PIN_CLKOS2".into(), "777".into()),
                ]),
            },
        )])
    }

    fn pll_parameters(metadata: &BTreeMap<CellId, PrimitiveMetadata>) -> &BTreeMap<String, String> {
        let PrimitiveMetadata::Pll { parameters, .. } = metadata.values().next().unwrap() else {
            unreachable!();
        };
        parameters
    }

    fn pll_parameters_mut(
        metadata: &mut BTreeMap<CellId, PrimitiveMetadata>,
    ) -> &mut BTreeMap<String, String> {
        let PrimitiveMetadata::Pll { parameters, .. } = metadata.values_mut().next().unwrap()
        else {
            unreachable!();
        };
        parameters
    }

    struct TestRegisterPins {
        clock: CellPinId,
        data: CellPinId,
        q: CellPinId,
    }

    fn add_timing_register(design: &mut Design, name: &str) -> TestRegisterPins {
        let cell = design.add_cell(name, ResourceKind::Register);
        TestRegisterPins {
            clock: design.add_pin(cell, "CLK", PinDirection::Input).unwrap(),
            data: design.add_pin(cell, "DI", PinDirection::Input).unwrap(),
            q: design.add_pin(cell, "Q", PinDirection::Output).unwrap(),
        }
    }

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
        let model = ecp5_timing_model(
            imported.design(),
            &packing,
            speed_grade,
            &BTreeSet::new(),
            imported.metadata(),
        )
        .unwrap();
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
        let tied_high = design.add_cell("tied_high", ResourceKind::Register);
        let tied_low = design.add_cell("tied_low", ResourceKind::Register);
        let tied_active_low = design.add_cell("tied_active_low", ResourceKind::Register);
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
            (tied_high, flip_flop(Some(ImportedActiveLevel::High))),
            (tied_low, flip_flop(Some(ImportedActiveLevel::High))),
            (tied_active_low, flip_flop(Some(ImportedActiveLevel::Low))),
        ]);
        let absorbed_inputs = BTreeMap::from([
            (tied_high, BTreeMap::from([("CE".into(), true)])),
            (tied_low, BTreeMap::from([("CE".into(), false)])),
            (tied_active_low, BTreeMap::from([("CE".into(), false)])),
        ]);

        let sets = ff_ce_control_sets(&design, &metadata, &absorbed_inputs)
            .into_iter()
            .collect::<BTreeMap<_, _>>();

        assert_eq!(sets[&high_a], sets[&high_b]);
        assert_ne!(sets[&high_a], sets[&low]);
        assert_ne!(sets[&high_a], sets[&other]);
        assert_eq!(sets[&always], sets[&tied_high]);
        assert_eq!(sets[&always], sets[&tied_active_low]);
        assert_ne!(sets[&tied_high], sets[&tied_low]);
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
    fn ff_lsr_control_sets_separate_net_polarity_mode_and_no_reset() {
        let mut design = Design::new();
        let reset_driver = design.add_cell("reset_driver", ResourceKind::Logic);
        let reset_output = design
            .add_pin(reset_driver, "out", PinDirection::Output)
            .unwrap();
        let cells = [
            "none",
            "high_async_a",
            "high_async_b",
            "low_async",
            "high_sync",
            "absorbed_inactive_high",
            "absorbed_inactive_low",
            "absorbed_asserted",
        ]
        .map(|name| design.add_cell(name, ResourceKind::Register));
        let reset_pins = cells[1..5]
            .iter()
            .map(|&cell| design.add_pin(cell, "LSR", PinDirection::Input).unwrap())
            .collect::<Vec<_>>();
        design.add_net("reset", reset_output, reset_pins).unwrap();
        let flip_flop = |reset| PrimitiveMetadata::FlipFlop {
            edge: ImportedClockEdge::Rising,
            enable: None,
            reset,
        };
        let reset = |active, asynchronous| {
            Some(ResetMetadata {
                active,
                asynchronous,
                value: false,
            })
        };
        let metadata = BTreeMap::from([
            (cells[0], flip_flop(None)),
            (cells[1], flip_flop(reset(ImportedActiveLevel::High, true))),
            (cells[2], flip_flop(reset(ImportedActiveLevel::High, true))),
            (cells[3], flip_flop(reset(ImportedActiveLevel::Low, true))),
            (cells[4], flip_flop(reset(ImportedActiveLevel::High, false))),
            (cells[5], flip_flop(reset(ImportedActiveLevel::High, true))),
            (cells[6], flip_flop(reset(ImportedActiveLevel::Low, false))),
            (cells[7], flip_flop(reset(ImportedActiveLevel::High, true))),
        ]);
        let absorbed_inputs = BTreeMap::from([
            (cells[5], BTreeMap::from([("LSR".into(), false)])),
            (cells[6], BTreeMap::from([("LSR".into(), true)])),
            (cells[7], BTreeMap::from([("LSR".into(), true)])),
        ]);

        let sets = ff_lsr_control_sets(&design, &metadata, &absorbed_inputs)
            .into_iter()
            .collect::<BTreeMap<_, _>>();

        assert_eq!(sets[&cells[1]], sets[&cells[2]]);
        assert_ne!(sets[&cells[0]], sets[&cells[1]]);
        assert_ne!(sets[&cells[1]], sets[&cells[3]]);
        assert_ne!(sets[&cells[1]], sets[&cells[4]]);
        assert_eq!(sets[&cells[0]], sets[&cells[5]]);
        assert_eq!(sets[&cells[0]], sets[&cells[6]]);
        assert_ne!(sets[&cells[0]], sets[&cells[7]]);
    }

    #[test]
    fn true_dual_port_ram_uses_each_physical_clock_edge() {
        let metadata = PrimitiveMetadata::BlockRam {
            depth: 4,
            word_width: 2,
            physical_width: 2,
            edge: ImportedClockEdge::Falling,
            write_enable: ImportedActiveLevel::High,
            read_enable: None,
            second_port: Some(BlockRamPortMetadata {
                edge: ImportedClockEdge::Rising,
                write_enable: ImportedActiveLevel::High,
                read_enable: None,
            }),
        };

        assert_eq!(
            primitive_clock_edge(Some(&metadata), "CLKA"),
            Some(TimingClockEdge::Falling)
        );
        assert_eq!(
            primitive_clock_edge(Some(&metadata), "CLKB"),
            Some(TimingClockEdge::Rising)
        );
    }

    #[test]
    fn timing_criticality_concentrates_on_the_worst_paths() {
        assert_eq!(criticality_weight(0, 4_000), 1);
        assert_eq!(criticality_weight(2_000, 4_000), 4);
        assert_eq!(criticality_weight(3_000, 4_000), 20);
        assert_eq!(criticality_weight(4_000, 4_000), 64);
    }

    #[test]
    fn ecp5_placement_weight_is_quartic_by_default_without_delay_attenuation() {
        assert_eq!(Ecp5FlowOptions::default().placement_weight_exponent, 4);
        assert_eq!(ecp5_placement_criticality_weight(0, 4_000, 4), 1);
        assert_eq!(ecp5_placement_criticality_weight(1_000, 4_000, 4), 1);
        assert_eq!(ecp5_placement_criticality_weight(2_000, 4_000, 4), 2);
        assert_eq!(ecp5_placement_criticality_weight(3_000, 4_000, 4), 4);
        assert_eq!(ecp5_placement_criticality_weight(4_000, 4_000, 4), 11);
        assert_eq!(ecp5_placement_criticality_weight(5_000, 4_000, 4), 11);
        assert_eq!(ecp5_placement_criticality_weight(1, 0, 4), 1);
        assert_eq!(
            ecp5_placement_criticality_weight(u128::MAX, u128::MAX, 4),
            11
        );
    }

    #[test]
    fn ecp5_weights_remain_domain_normalized_after_timing_closes() {
        let timing = TimingReport {
            net_delays: Vec::new(),
            net_setup_slacks: Vec::new(),
            net_setup_criticalities: vec![
                NetSetupCriticality {
                    net: NetId(2),
                    sink: CellPinId(20),
                    path_delay_ps: 1,
                    domain_worst_path_delay_ps: 2,
                },
                NetSetupCriticality {
                    net: NetId(3),
                    sink: CellPinId(30),
                    path_delay_ps: 7,
                    domain_worst_path_delay_ps: 7,
                },
            ],
            setup_checks: Vec::new(),
            hold_checks: Vec::new(),
            unchecked_endpoints: Vec::new(),
            worst_slack_ps: Some(1_000),
            worst_hold_slack_ps: Some(1_000),
        };
        let weights = ecp5_timing_placement_weights(&timing, 4);
        assert_eq!(weights[&(NetId(2), CellPinId(20))], 2);
        assert_eq!(weights[&(NetId(3), CellPinId(30))], 11);
    }

    #[test]
    fn placement_weight_exponent_controls_the_ecp5_power() {
        assert_eq!(ecp5_placement_criticality_weight(2_000, 4_000, 1), 6);
        assert_eq!(ecp5_placement_criticality_weight(2_000, 4_000, 2), 4);
        assert_eq!(ecp5_placement_criticality_weight(2_000, 4_000, 4), 2);
        assert_eq!(
            ecp5_placement_criticality_weight(2_000, 4_000, 0),
            ecp5_placement_criticality_weight(2_000, 4_000, 1),
        );
        assert_eq!(ecp5_placement_criticality_weight(2_000, 4_000, u32::MAX), 1);
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
    fn timing_objective_never_trades_setup_wns_for_aggregate_score() {
        let narrow = slack_violations([-100, 0].into_iter()).score();
        let widespread = slack_violations(std::iter::repeat_n(-99, 100)).score();

        assert!(
            staged_timing_objective(widespread, widespread, -99, -99)
                > staged_timing_objective(narrow, narrow, -100, -100)
        );

        let incumbent = slack_violations(std::iter::repeat_n(-100, 100)).score();
        let aggregate_winner = slack_violations([-101].into_iter()).score();
        assert!(!strictly_improves_timing_objective(
            staged_timing_objective(aggregate_winner, aggregate_winner, -101, 0),
            staged_timing_objective(incumbent, incumbent, -100, 0),
        ));

        let equal_wns_aggregate_winner = slack_violations([-100].into_iter()).score();
        assert!(strictly_improves_timing_objective(
            staged_timing_objective(
                equal_wns_aggregate_winner,
                equal_wns_aggregate_winner,
                -100,
                0,
            ),
            staged_timing_objective(incumbent, incumbent, -100, 0),
        ));
    }

    #[test]
    fn timing_objective_never_trades_hold_wns_after_setup_closes() {
        let setup_closed = slack_violations([0].into_iter()).score();
        let incumbent_hold = slack_violations(std::iter::repeat_n(-100, 100)).score();
        let aggregate_winner = slack_violations([-101].into_iter()).score();

        assert!(!strictly_improves_timing_objective(
            staged_timing_objective(setup_closed, aggregate_winner, 0, -101),
            staged_timing_objective(setup_closed, incumbent_hold, 0, -100),
        ));

        let wns_winner = slack_violations(std::iter::repeat_n(-99, 100)).score();
        let narrow = slack_violations([-100].into_iter()).score();
        assert!(strictly_improves_timing_objective(
            staged_timing_objective(setup_closed, wns_winner, 0, -99),
            staged_timing_objective(setup_closed, narrow, 0, -100),
        ));
    }

    #[test]
    fn timing_objective_closes_setup_before_hold_repair() {
        let closed = slack_violations([0].into_iter()).score();
        let setup_near = slack_violations([-10].into_iter()).score();
        let setup_far = slack_violations([-20].into_iter()).score();
        let hold_bad = slack_violations([-1_000].into_iter()).score();

        assert!(
            staged_timing_objective(setup_near, hold_bad, -10, -1_000)
                > staged_timing_objective(setup_far, closed, -20, 10)
        );
        assert!(
            staged_timing_objective(closed, hold_bad, 0, -1_000)
                > staged_timing_objective(setup_near, closed, -10, 10)
        );
    }

    #[test]
    fn timing_feedback_accepts_only_strict_objective_improvement() {
        let incumbent_violations = slack_violations([-100, -20].into_iter()).score();
        let candidate_violations = slack_violations([-90, -20].into_iter()).score();
        let incumbent =
            staged_timing_objective(incumbent_violations, incumbent_violations, -100, 0);
        let candidate = staged_timing_objective(candidate_violations, candidate_violations, -90, 0);

        assert!(strictly_improves_timing_objective(candidate, incumbent));
        assert!(!strictly_improves_timing_objective(incumbent, incumbent));
        assert!(!strictly_improves_timing_objective(incumbent, candidate));
    }

    #[test]
    fn route_first_closure_skips_feedback_after_route_eco_closes_setup() {
        // Model the state immediately after the route-ECO pass: zero slack is
        // closed, so the fallback that would increment the call count must not
        // execute.
        let ((slack_ps, feedback_calls), feedback_ran) = run_setup_feedback_fallback(
            (0_i128, 0_usize),
            |state| state.0 < 0,
            |(slack_ps, feedback_calls)| Ok::<_, ()>((slack_ps, feedback_calls + 1)),
        )
        .unwrap();

        assert_eq!(slack_ps, 0);
        assert_eq!(feedback_calls, 0);
        assert!(!feedback_ran);
    }

    #[test]
    fn route_first_closure_runs_feedback_when_route_eco_misses_setup() {
        let ((slack_ps, feedback_calls), feedback_ran) = run_setup_feedback_fallback(
            (-1_i128, 0_usize),
            |state| state.0 < 0,
            |(_, feedback_calls)| Ok::<_, ()>((0, feedback_calls + 1)),
        )
        .unwrap();

        assert_eq!(slack_ps, 0);
        assert_eq!(feedback_calls, 1);
        assert!(feedback_ran);
    }

    fn predicted_setup_report(slacks: &[i128]) -> TimingReport {
        let setup_checks = slacks
            .iter()
            .copied()
            .enumerate()
            .map(|(index, slack_ps)| SetupCheck {
                cell: CellId(index),
                data_pin: CellPinId(index),
                clock_net: NetId(0),
                launch_edge: TimingClockEdge::Rising,
                capture_edge: TimingClockEdge::Rising,
                arrival_ps: 0,
                clock_arrival_ps: 0,
                setup_ps: 0,
                uncertainty_ps: 0,
                required_ps: 0,
                slack_ps,
            })
            .collect::<Vec<_>>();
        TimingReport {
            net_delays: Vec::new(),
            net_setup_slacks: Vec::new(),
            net_setup_criticalities: Vec::new(),
            worst_slack_ps: setup_checks.iter().map(|check| check.slack_ps).min(),
            setup_checks,
            hold_checks: Vec::new(),
            unchecked_endpoints: Vec::new(),
            worst_hold_slack_ps: Some(0),
        }
    }

    #[test]
    fn timing_feedback_prescreen_rejects_only_predicted_pareto_regressions() {
        let incumbent = predicted_setup_report(&[-100, -20, 10]);
        let dominated = predicted_setup_report(&[-120, -30, -1]);
        assert!(predicted_setup_candidate_is_pareto_dominated(
            &incumbent, &dominated
        ));

        let equal = predicted_setup_report(&[-100, -20, 10]);
        assert!(!predicted_setup_candidate_is_pareto_dominated(
            &incumbent, &equal
        ));

        let improved = predicted_setup_report(&[-90, -20, 10]);
        assert!(!predicted_setup_candidate_is_pareto_dominated(
            &incumbent, &improved
        ));

        let closed_incumbent = predicted_setup_report(&[100, 200]);
        let lower_margin = predicted_setup_report(&[50, 150]);
        assert!(!predicted_setup_candidate_is_pareto_dominated(
            &closed_incumbent,
            &lower_margin,
        ));
    }

    #[test]
    fn timing_feedback_prescreen_routes_mixed_predictions_to_avoid_false_negatives() {
        let incumbent = predicted_setup_report(&[-100, -50]);
        // Routed acceptance would prioritize the 10 ps WNS loss, but this
        // prediction also recovers 40 ps on another endpoint.  The placement
        // model cannot decide whether a full router will retain that trade, so
        // the candidate must remain eligible for physical evaluation.
        let mixed = predicted_setup_report(&[-110, -10]);
        assert!(!strictly_improves_timing_objective(
            super::timing_objective(&mixed),
            super::timing_objective(&incumbent),
        ));
        assert!(!predicted_setup_candidate_is_pareto_dominated(
            &incumbent, &mixed
        ));

        let unconstrained = predicted_setup_report(&[]);
        assert!(!predicted_setup_candidate_is_pareto_dominated(
            &incumbent,
            &unconstrained,
        ));
    }

    fn route_eco_candidate(net: usize, route_delay_ps: u64) -> WorstSetupNetRouteEcoCandidate {
        WorstSetupNetRouteEcoCandidate {
            net: NetId(net),
            sink: CellPinId(net),
            slack_ps: -100,
            route_delay_ps,
            shared_prefix_delay_ps: 0,
            worst_sinks: 1,
            fanout: 1,
        }
    }

    #[test]
    fn route_eco_cohort_collects_only_sibling_lut_inputs() {
        let mut design = Design::new();
        let first_driver = design.add_cell("first_driver", ResourceKind::Logic);
        let second_driver = design.add_cell("second_driver", ResourceKind::Logic);
        let lut = design.add_cell("lut", ResourceKind::Lut(4));
        let first_output = design
            .add_pin(first_driver, "out", PinDirection::Output)
            .unwrap();
        let second_output = design
            .add_pin(second_driver, "out", PinDirection::Output)
            .unwrap();
        let first_input = design.add_pin(lut, "A", PinDirection::Input).unwrap();
        let second_input = design.add_pin(lut, "B", PinDirection::Input).unwrap();
        let first_net = design
            .add_net("first", first_output, [first_input])
            .unwrap();
        let second_net = design
            .add_net("second", second_output, [second_input])
            .unwrap();
        let candidate = WorstSetupNetRouteEcoCandidate {
            net: second_net,
            sink: second_input,
            slack_ps: -100,
            route_delay_ps: 500,
            shared_prefix_delay_ps: 0,
            worst_sinks: 1,
            fanout: 1,
        };

        assert_eq!(
            worst_setup_route_eco_cohort(&design, candidate),
            vec![first_net, second_net],
        );
    }

    #[test]
    fn worst_setup_route_eco_worklist_uses_refreshed_cone_order_after_commit() {
        let mut worklist = WorstSetupRouteEcoWorklist::default();

        assert_eq!(
            worklist
                .next([route_eco_candidate(1, 500), route_eco_candidate(2, 400)])
                .unwrap()
                .net,
            NetId(1),
        );
        // Model a strict exact-STA commit.  The refreshed cone introduces net
        // 3 ahead of the stale second candidate and must take precedence.
        assert_eq!(
            worklist
                .next([
                    route_eco_candidate(3, 700),
                    route_eco_candidate(1, 450),
                    route_eco_candidate(2, 400),
                ])
                .unwrap()
                .net,
            NetId(3),
        );
    }

    #[test]
    fn worst_setup_route_eco_worklist_never_retries_an_unchanged_net() {
        let candidate = route_eco_candidate(1, 500);
        let mut worklist = WorstSetupRouteEcoWorklist::default();

        assert_eq!(worklist.next([candidate]), Some(candidate));
        assert_eq!(worklist.next([candidate]), None);
        assert_eq!(worklist.trials(), 1);
    }

    #[test]
    fn worst_setup_route_eco_worklist_stops_at_unique_candidate_exhaustion() {
        let mut worklist = WorstSetupRouteEcoWorklist::default();

        assert!(worklist.next([route_eco_candidate(1, 500)]).is_some());
        assert!(worklist.next([route_eco_candidate(2, 400)]).is_some());
        assert!(
            worklist
                .next([route_eco_candidate(1, 600), route_eco_candidate(2, 300)])
                .is_none()
        );
        assert_eq!(worklist.trials(), 2);
    }

    #[test]
    fn route_eco_retry_after_global_change_reopens_changed_candidates() {
        let candidate = route_eco_candidate(1, 500);
        let mut worklist = WorstSetupRouteEcoWorklist::default();

        assert_eq!(worklist.next([candidate]), Some(candidate));
        worklist.reset_attempted_after_global_change();
        assert_eq!(worklist.next([candidate]), Some(candidate));
        assert_eq!(
            worklist.next([route_eco_candidate(2, 400)]),
            Some(route_eco_candidate(2, 400))
        );
        assert_eq!(worklist.trials(), 3);
    }

    #[test]
    fn worst_setup_route_eco_worklist_advances_after_reject_reject_accept_refresh() {
        let candidates = [
            route_eco_candidate(1, 700),
            route_eco_candidate(2, 600),
            route_eco_candidate(3, 500),
            route_eco_candidate(4, 400),
        ];
        let mut worklist = WorstSetupRouteEcoWorklist::default();

        // Reject A, reject B, then accept C under exact STA.
        assert_eq!(worklist.next(candidates).unwrap().net, NetId(1));
        assert_eq!(worklist.next(candidates).unwrap().net, NetId(2));
        assert_eq!(worklist.next(candidates).unwrap().net, NetId(3));

        // Recomputing the exact-WNS cone can rank A first again, but A's
        // incumbent route is unchanged. The worklist must inspect newly
        // exposed D instead of retrying A.
        let refreshed = [route_eco_candidate(1, 800), route_eco_candidate(4, 750)];
        assert_eq!(worklist.next(refreshed).unwrap().net, NetId(4));
    }

    #[test]
    fn worst_setup_net_route_eco_selects_every_unique_worst_cone_net() {
        let mut net_delays = Vec::new();
        let mut net_setup_slacks = Vec::new();
        let mut routes = Vec::new();
        for (index, delay) in [100, 600, 300, 500, 200, 400].into_iter().enumerate() {
            let connection = (NetId(index), CellPinId(index + 10));
            net_delays.push(NetDelay {
                net: connection.0,
                sink: connection.1,
                delay: DelayRange::new(delay, delay).unwrap(),
            });
            net_setup_slacks.push(NetSetupSlack {
                net: connection.0,
                sink: connection.1,
                slack_ps: -100,
            });
            routes.push(Arc::new(NetRoute::new(
                connection.0,
                vec![RouteArc {
                    sink: Some(connection.1),
                    wires: vec![WireId(index * 2), WireId(index * 2 + 1)],
                    pips: vec![PipId(index)],
                }],
            )));
        }
        // A second exact-worst sink on net 1 must not consume a second
        // candidate slot. Its shared physical PIP is measured exactly but raw
        // fanout does not outrank the maximum realized route delay.
        net_delays.push(NetDelay {
            net: NetId(1),
            sink: CellPinId(101),
            delay: DelayRange::new(550, 550).unwrap(),
        });
        net_setup_slacks.push(NetSetupSlack {
            net: NetId(1),
            sink: CellPinId(101),
            slack_ps: -100,
        });
        routes[1] = Arc::new(NetRoute::new(
            NetId(1),
            vec![
                RouteArc {
                    sink: Some(CellPinId(11)),
                    wires: vec![WireId(2), WireId(3)],
                    pips: vec![PipId(1)],
                },
                RouteArc {
                    sink: Some(CellPinId(101)),
                    wires: vec![WireId(2), WireId(3)],
                    pips: vec![PipId(1)],
                },
            ],
        ));
        // A larger route outside the exact worst-slack cone is ineligible.
        net_delays.push(NetDelay {
            net: NetId(99),
            sink: CellPinId(99),
            delay: DelayRange::new(10_000, 10_000).unwrap(),
        });
        net_setup_slacks.push(NetSetupSlack {
            net: NetId(99),
            sink: CellPinId(99),
            slack_ps: -99,
        });
        let timing = TimingReport {
            net_delays,
            net_setup_slacks,
            net_setup_criticalities: Vec::new(),
            setup_checks: Vec::new(),
            hold_checks: Vec::new(),
            unchecked_endpoints: Vec::new(),
            worst_slack_ps: Some(-100),
            worst_hold_slack_ps: Some(0),
        };

        let pip_delays = [100, 600, 300, 500, 200, 400];
        let candidates = worst_setup_net_route_eco_candidates(&timing, &routes, &pip_delays);
        assert_eq!(candidates.len(), 6);
        assert_eq!(
            candidates
                .iter()
                .map(|candidate| (
                    candidate.net,
                    candidate.route_delay_ps,
                    candidate.shared_prefix_delay_ps,
                    candidate.worst_sinks,
                ))
                .collect::<Vec<_>>(),
            vec![
                (NetId(1), 600, 600, 2),
                (NetId(3), 500, 0, 1),
                (NetId(5), 400, 0, 1),
                (NetId(2), 300, 0, 1),
                (NetId(4), 200, 0, 1),
                (NetId(0), 100, 0, 1),
            ]
        );
    }

    #[test]
    fn rejected_route_eco_sta_trial_keeps_the_incumbent_bit_exact() {
        let design = Design::new();
        let device = Device::rectangular_logic(1, 1).unwrap();
        let placement = placement_from_partial_bindings(
            &design,
            &device,
            &PlacementConstraints::new(),
            &BTreeMap::new(),
        )
        .unwrap();
        let mut implementation = PnrResult {
            placement,
            routes: Vec::new(),
            total_pips: 0,
        };
        let timing_at = |worst_slack_ps| TimingReport {
            net_delays: Vec::new(),
            net_setup_slacks: Vec::new(),
            net_setup_criticalities: Vec::new(),
            setup_checks: Vec::new(),
            hold_checks: Vec::new(),
            unchecked_endpoints: Vec::new(),
            worst_slack_ps: Some(worst_slack_ps),
            worst_hold_slack_ps: Some(0),
        };
        let mut timing = timing_at(-100);
        let before_implementation = implementation.clone();
        let before_timing = timing.clone();
        let mut candidate = implementation.clone();
        candidate.total_pips = 99;

        assert!(!commit_strict_route_eco_candidate(
            &mut implementation,
            &mut timing,
            candidate,
            timing_at(-100),
        ));
        assert_eq!(implementation, before_implementation);
        assert_eq!(timing, before_timing);
    }

    #[test]
    fn eco_timing_session_rejects_unknown_pips_without_panicking() {
        let architecture = read_architecture(ECP5_FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let source = design.add_cell("source", ResourceKind::Logic);
        let driver = design
            .add_pin(source, "driver", PinDirection::Output)
            .unwrap();
        let target = design.add_cell("target", ResourceKind::Logic);
        let sink = design.add_pin(target, "sink", PinDirection::Input).unwrap();
        let net = design.add_net("invalid_route", driver, [sink]).unwrap();
        let empty_design = Design::new();
        let placement = placement_from_partial_bindings(
            &empty_design,
            architecture.device(),
            &PlacementConstraints::new(),
            &BTreeMap::new(),
        )
        .unwrap();
        let unknown = PipId(usize::MAX);
        let implementation = PnrResult {
            placement,
            routes: vec![Arc::new(NetRoute::new(
                net,
                vec![RouteArc {
                    sink: Some(sink),
                    wires: Vec::new(),
                    pips: vec![unknown],
                }],
            ))],
            total_pips: 1,
        };
        let model = TimingModel::new();
        let constraints = TimingConstraints::new();
        let mut session = Ecp5EcoTimingSession::new(
            &design,
            &architecture,
            &architecture.speed_grades()["6"],
            &model,
            &constraints,
        )
        .unwrap();

        assert!(matches!(
            session.analyze(&implementation),
            Err(Ecp5FlowError::Timing(TimingError::UnknownRoutedPip(pip))) if pip == unknown
        ));
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
        assert_ne!(
            placement_identity(&design, &implementation.placement),
            placement_identity(&design, &rebound),
        );
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
    fn recognizes_only_same_axis_span6_continuations() {
        for name in [
            "span6hw_to_span6hw_w6",
            "span6he_to_span6he_e6",
            "span6vn_to_span6vn_n6",
            "span6vs_to_span6vs_s6",
        ] {
            assert!(is_ecp5_span6_continuation(name), "{name}");
        }
        assert!(!is_ecp5_span6_continuation("q_to_span6hw_w3"));
        assert!(!is_ecp5_span6_continuation("span6hw_to_span2vn_n1w3"));
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

        let constraints =
            ecp5_timing_constraints(&design, &packing, &GeneratedClockRelations::new()).unwrap();
        let timing_model = ecp5_timing_model(
            &design,
            &packing,
            &architecture.speed_grades()["6"],
            &BTreeSet::new(),
            imported.metadata(),
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
        assert!(constraints.setup_uncertainties_ps().is_empty());
        assert_eq!(timing_model.clock_to_q(ff_q).unwrap().2.max_ps, 525);
        assert_eq!(timing_model.setup_hold(ff_data).unwrap().3.min_ps, 233);
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
            imported.metadata(),
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
            &BTreeMap::new(),
        )
        .unwrap();
        let clkb = find_cell_pin(&design, memory, "CLKB").unwrap();
        let output = find_cell_pin(&design, memory, "DOB0").unwrap();
        let write_data = find_cell_pin(&design, memory, "DIA0").unwrap();
        let read_address = find_cell_pin(&design, memory, "ADB0").unwrap();

        assert_eq!(model.clock_to_q(output).unwrap().0, clkb);
        assert_eq!(model.clock_to_q(output).unwrap().2.max_ps, 5830);
        assert_eq!(model.setup_hold(write_data).unwrap().2.max_ps, 220);
        assert_eq!(model.setup_hold(write_data).unwrap().3.max_ps, 43);
        assert_eq!(model.setup_hold(read_address).unwrap().0, clkb);
        assert_eq!(model.setup_hold(read_address).unwrap().2.max_ps, 251);
        assert_eq!(model.setup_hold(read_address).unwrap().3.max_ps, 123);
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
            &BTreeMap::new(),
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
