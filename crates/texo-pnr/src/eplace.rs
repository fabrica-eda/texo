//! Heterogeneous electrostatic global placement.

use std::collections::BTreeMap;
use std::f64::consts::TAU;
use std::sync::atomic::{AtomicU64, Ordering};

use texo_model::{Device, Point, ResourceKind, UnifiedGraph};

use super::analytical_placement::{AnalyticalHypergraph, WeightedAverageObjective};
use super::electrostatic_placement::{
    DensityFieldResult, DensityFiller, DensityMember, DensityModel, DensityResult, DensityUnit,
    ElectrostaticError, FixedOccupancy,
};
use super::global_placement::{
    CoordinateBounds, DynamicNesterovState, DynamicNesterovStatus, NesterovError,
    ObjectiveEvaluation, dynamic_nesterov_step,
};
use super::instance_area::{
    InstanceAreaAdjustment, InstanceAreaError, InstanceAreaFiller, InstanceAreaMember,
    adjust_instance_areas, routability_optimized_area,
};
use super::register_clustering::{MovableRegister, clustering_areas};
use super::{
    PlacementUnit, RegisterControlSet, RoutingCapacityMap, RoutingChannelOrientation,
    rounded_coordinate,
};

#[derive(Clone, Debug, PartialEq)]
pub(super) enum EplaceError {
    Density(ElectrostaticError),
    Optimizer(NesterovError<ElectrostaticError>),
    InstanceArea(InstanceAreaError),
    DensityFieldChanged,
    InvalidNormalization,
    DidNotConverge {
        iterations: u128,
        overflows: Vec<(ResourceKind, f64)>,
    },
}

impl From<ElectrostaticError> for EplaceError {
    fn from(error: ElectrostaticError) -> Self {
        Self::Density(error)
    }
}

impl From<InstanceAreaError> for EplaceError {
    fn from(error: InstanceAreaError) -> Self {
        Self::InstanceArea(error)
    }
}

struct ContinuousProblem<'a> {
    hypergraph: &'a AnalyticalHypergraph,
    density_model: DensityModel,
    unit_count: usize,
    movable_instance_count: usize,
    movable_units: Vec<usize>,
    density_units: Vec<DensityUnit>,
    density_member_cells: Vec<Vec<texo_model::CellId>>,
    register_controls: BTreeMap<texo_model::CellId, RegisterControlSet>,
    density_gamma_weights: BTreeMap<ResourceKind, f64>,
    fillers: Vec<DensityFiller>,
    fixed_coordinates: &'a [Option<(f64, f64)>],
    cell_offsets: &'a [(f64, f64)],
}

#[derive(Clone, Debug)]
struct ContinuousRoutingDemand {
    width: u32,
    horizontal: Vec<f64>,
    vertical: Vec<f64>,
}

#[derive(Clone, Debug)]
struct AreaAdjustmentMetrics {
    maximum_relative_increase: f64,
    total_relative_increase: f64,
    maximum_horizontal_utilization: f64,
    maximum_vertical_utilization: f64,
    members_inflated: usize,
    adjustment: InstanceAreaAdjustment,
}

struct ContinuousEvaluation {
    wirelength: WeightedAverageObjective,
    density: DensityResult,
}

#[derive(Clone, Debug)]
struct PlacementCheckpoint {
    targets: Vec<Point>,
    maximum_overflow_excess: f64,
    total_overflow_excess: f64,
}

impl PlacementCheckpoint {
    fn new(targets: Vec<Point>, fields: &[DensityFieldResult]) -> Self {
        let (maximum_overflow_excess, total_overflow_excess) = overflow_excess_score(fields);
        Self {
            targets,
            maximum_overflow_excess,
            total_overflow_excess,
        }
    }

    fn consider(&mut self, targets: Vec<Point>, fields: &[DensityFieldResult]) {
        let (maximum_overflow_excess, total_overflow_excess) = overflow_excess_score(fields);
        let better = maximum_overflow_excess
            .total_cmp(&self.maximum_overflow_excess)
            .then_with(|| total_overflow_excess.total_cmp(&self.total_overflow_excess))
            .is_lt();
        if better {
            *self = Self {
                targets,
                maximum_overflow_excess,
                total_overflow_excess,
            };
        }
    }
}

// elfPlace equations (12), (18), and (22).  These are dimensionless except
// for the density multiplier, whose initial value is derived from the raw
// wirelength and electrostatic forces.
const AUGMENTED_DENSITY_BETA: f64 = 2_000.0;
const INITIAL_DENSITY_WEIGHT: f64 = 1.0e-4;
const AREA_RESET_DENSITY_WEIGHT: f64 = 0.1;
const MULTIPLIER_ALPHA_LOW: f64 = 1.05;
const MULTIPLIER_ALPHA_HIGH: f64 = 1.06;
const AREA_ADJUSTMENT_OVERFLOW_TARGET: f64 = 0.15;
const PAPER_INITIALIZATION_STDDEV_FRACTION: f64 = 1.0e-3;
// ePlace equation (38) uses `base = 8 * bin_width`.  The electrostatic
// model's Poisson grid is the device tile grid, hence bin_width is one tile.
const WIRELENGTH_GAMMA_BASE_TILES: f64 = 8.0;
const MAX_EPLACE_ITERATIONS: u128 = 3_000;

impl ContinuousProblem<'_> {
    fn evaluate(
        &self,
        coordinates: &[f64],
        global_gamma: Option<f64>,
    ) -> Result<ContinuousEvaluation, ElectrostaticError> {
        let started = std::time::Instant::now();
        debug_assert_eq!(
            coordinates.len(),
            2 * (self.unit_count + self.fillers.len())
        );
        let unit_positions = coordinates[..2 * self.unit_count]
            .as_chunks::<2>()
            .0
            .iter()
            .map(|pair| (pair[0], pair[1]))
            .collect::<Vec<_>>();
        // Wirelength and electrostatic density are independent functions of
        // the same immutable candidate coordinates. Both already expose
        // deterministic fixed-order reductions internally, so executing the
        // two objective components concurrently changes neither arithmetic
        // nor rigid-macro semantics and lets their independent Rayon tasks
        // share the placement-wide worker pool.
        let ((wirelength, wirelength_elapsed), (density, density_elapsed)) = rayon::join(
            || {
                let started = std::time::Instant::now();
                let objective = global_gamma.map_or_else(
                    || {
                        self.hypergraph.weighted_average_objective(
                            &unit_positions,
                            self.fixed_coordinates,
                            self.cell_offsets,
                        )
                    },
                    |gamma| {
                        self.hypergraph
                            .weighted_average_objective_with_global_gamma(
                                &unit_positions,
                                self.fixed_coordinates,
                                self.cell_offsets,
                                gamma,
                            )
                    },
                );
                (objective, started.elapsed())
            },
            || {
                let started = std::time::Instant::now();
                let result = self.density_model.evaluate_with_positions(
                    &self.density_units,
                    &self.fillers,
                    |index| unit_positions[self.movable_units[index]],
                    |index| {
                        let coordinate = 2 * (self.unit_count + index);
                        (coordinates[coordinate], coordinates[coordinate + 1])
                    },
                );
                (result, started.elapsed())
            },
        );
        let density = density?;
        let evaluation = ContinuousEvaluation {
            wirelength,
            density,
        };
        if std::env::var_os("TEXO_PNR_TRACE_EPLACE_EVALUATIONS").is_some() {
            static EVALUATIONS: AtomicU64 = AtomicU64::new(0);
            let evaluation_index = EVALUATIONS.fetch_add(1, Ordering::Relaxed) + 1;
            eprintln!(
                "TEXO_PNR_TRACE eplace-evaluation index={evaluation_index} elapsed_us={} wirelength_us={} density_us={} units={} fillers={} fields={}",
                started.elapsed().as_micros(),
                wirelength_elapsed.as_micros(),
                density_elapsed.as_micros(),
                self.unit_count,
                self.fillers.len(),
                evaluation.density.fields.len(),
            );
        }
        Ok(evaluation)
    }

    fn field_gradient(&self, field: &DensityFieldResult, coordinate_count: usize) -> Vec<f64> {
        let mut gradient = vec![0.0; coordinate_count];
        self.add_scaled_field_gradient(field, 1.0, &mut gradient);
        gradient
    }

    fn add_scaled_field_gradient(
        &self,
        field: &DensityFieldResult,
        coefficient: f64,
        gradient: &mut [f64],
    ) {
        debug_assert_eq!(gradient.len(), 2 * (self.unit_count + self.fillers.len()));
        for (&unit, &(gradient_x, gradient_y)) in
            self.movable_units.iter().zip(&field.unit_gradients)
        {
            gradient[2 * unit] += coefficient * gradient_x;
            gradient[2 * unit + 1] += coefficient * gradient_y;
        }
        for &(filler, gradient_x, gradient_y) in &field.filler_gradients {
            let coordinate = 2 * (self.unit_count + filler);
            gradient[coordinate] += coefficient * gradient_x;
            gradient[coordinate + 1] += coefficient * gradient_y;
        }
    }
}

/// Applies one monotone, diagonally preconditioned electrostatic spreading
/// operator and returns a single set of unit targets for exact legalization.
///
/// This is deliberately one composite global-placement operator rather than
/// an iteration budget. Repeated full-device legalization dominates runtime on
/// FPGA assignment graphs, so convergence after this global proposal belongs
/// to strict descent on the legal discrete placement.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
pub(super) fn place(
    graph: &UnifiedGraph<'_>,
    units: &[PlacementUnit],
    hypergraph: &AnalyticalHypergraph,
    initial_x: &[f64],
    initial_y: &[f64],
    fixed_coordinates: &[Option<(f64, f64)>],
    cell_offsets: &[(f64, f64)],
    routing_capacity: Option<&RoutingCapacityMap>,
    register_controls: &[RegisterControlSet],
) -> Result<Vec<Point>, EplaceError> {
    let device = graph.device();
    let include_special_density =
        std::env::var_os("TEXO_PNR_EPLACE_INCLUDE_SPECIAL_DENSITY").is_some();
    let cell_pin_weights = hypergraph.baseline_cell_incidence_weights(graph.design().cells().len());
    let density_gamma_weights =
        density_gamma_weights(units, device, &cell_pin_weights, include_special_density);
    let mut fixed_occupancy = Vec::new();
    let mut movable_units = Vec::new();
    let mut density_units = Vec::new();
    let mut density_member_cells = Vec::new();
    for (unit_index, unit) in units.iter().enumerate() {
        let assignment = unit.choices.assignment(0);
        if unit.choices.len() == 1 {
            fixed_occupancy.extend(assignment.iter().filter_map(|&bel| {
                let physical = &device.bels()[bel.0];
                density_kind_is_exchangeable(physical.kind, include_special_density).then_some(
                    FixedOccupancy {
                        kind: physical.kind,
                        point: physical.point,
                    },
                )
            }));
            continue;
        }
        let origin = device.bels()[assignment[0].0].point;
        let mut members = Vec::new();
        let mut member_cells = Vec::new();
        for (&cell, &bel) in unit.cells.iter().zip(assignment) {
            let physical = &device.bels()[bel.0];
            if density_kind_is_exchangeable(physical.kind, include_special_density) {
                members.push(DensityMember {
                    kind: physical.kind,
                    offset_x: f64::from(physical.point.x) - f64::from(origin.x),
                    offset_y: f64::from(physical.point.y) - f64::from(origin.y),
                    charge: 1.0,
                });
                member_cells.push(cell);
            }
        }
        if members.is_empty() {
            // Dedicated clocks and the ECP5 catch-all Logic class retain
            // their wirelength coordinate and exact discrete legalization.
            // They are not one interchangeable electrostatic capacity.
            continue;
        }
        movable_units.push(unit_index);
        density_units.push(DensityUnit {
            origin_x: initial_x[unit_index],
            origin_y: initial_y[unit_index],
            members,
        });
        density_member_cells.push(member_cells);
    }

    if density_units.is_empty() {
        return Ok(rounded_unit_targets(
            initial_x,
            initial_y,
            device.width(),
            device.height(),
        ));
    }

    let density_model = DensityModel::new(device, &fixed_occupancy)?;
    let fillers = density_model.initial_fillers(&density_units)?;
    let (mut coordinates, bounds) =
        initial_coordinates_and_bounds(units, device, initial_x, initial_y, &fillers);
    if std::env::var_os("TEXO_PNR_EPLACE_PAPER_INITIALIZATION").is_some() {
        apply_paper_initialization(
            &mut coordinates,
            &bounds,
            units,
            device.width(),
            device.height(),
        );
    }
    for (index, coordinate) in coordinates.iter_mut().enumerate() {
        *coordinate = coordinate.clamp(bounds.lower[index], bounds.upper[index]);
    }
    let register_control_count = register_controls.len();
    let register_controls = register_controls
        .iter()
        .copied()
        .map(|control| (control.cell, control))
        .collect::<BTreeMap<_, _>>();
    if register_controls.len() != register_control_count {
        return Err(EplaceError::InvalidNormalization);
    }
    let movable_instance_count = units
        .iter()
        .filter(|unit| unit.choices.len() != 1)
        .map(|unit| unit.cells.len())
        .sum();
    let mut problem = ContinuousProblem {
        hypergraph,
        density_model,
        unit_count: units.len(),
        movable_instance_count,
        movable_units,
        density_units,
        density_member_cells,
        register_controls,
        density_gamma_weights,
        fillers,
        fixed_coordinates,
        cell_offsets,
    };

    let use_global_gamma = std::env::var_os("TEXO_PNR_EPLACE_LEGACY_NET_GAMMA").is_none();
    // Supplying characterized architecture capacity selects the routability-
    // driven solve. Callers that need the pure density objective omit it.
    let adjust_area = routing_capacity.is_some();
    let area_round_limit = area_adjustment_round_limit(&problem)?;
    let mut completed_iterations = 0_u128;
    let mut area_round = 0_usize;
    let mut previous_area_change = None;
    let mut density_scales = None;
    let mut reset_after_area_adjustment = false;
    loop {
        // elfPlace Figure 2 starts each routability update as soon as both
        // LUT and register overflow reach 15%.  Once the installed area
        // increase falls below one percent, the same restarted optimization
        // continues to the field-specific final targets (10/10/20%).
        let stop_criterion =
            if adjust_area && !previous_area_change.is_some_and(|change| change < 0.01) {
                DensityStopCriterion::AreaAdjustmentReady
            } else {
                DensityStopCriterion::Final
            };
        let round_started = std::time::Instant::now();
        let round_first_iteration = completed_iterations;
        let optimized = optimize_density_once(
            &problem,
            &coordinates,
            &bounds,
            units.len(),
            device.width(),
            device.height(),
            use_global_gamma,
            density_scales.as_deref(),
            reset_after_area_adjustment,
            stop_criterion,
            completed_iterations,
        )?;
        if density_scales.is_none() {
            density_scales = Some(optimized.density_scales.clone());
        }
        completed_iterations = optimized.completed_iterations;
        coordinates = optimized.coordinates;
        if std::env::var_os("TEXO_PNR_METRICS").is_some() {
            eprintln!(
                "[metrics] eplace_area_round round={area_round} elapsed={:?} iterations={} previous_area_change={:?}",
                round_started.elapsed(),
                completed_iterations - round_first_iteration,
                previous_area_change,
            );
        }
        if stop_criterion == DensityStopCriterion::Final {
            return Ok(optimized.best_targets);
        }
        let capacity = routing_capacity.expect("area adjustment requires routing capacity");
        let metrics = propose_area_adjustment(&problem, &coordinates, capacity)?;
        report_area_adjustment(area_round, &metrics);
        if !metrics.adjustment.requires_optimizer_reset {
            previous_area_change = Some(0.0);
            // Reaching the routability-update boundary starts a new
            // optimization phase even when Eq. (23) installs no new charge.
            // Reusing the initial density multiplier here carries stale
            // conditioning across elfPlace Figure 2's phase boundary.
            reset_after_area_adjustment = true;
            continue;
        }
        if area_round >= area_round_limit {
            return Err(EplaceError::InvalidNormalization);
        }
        apply_area_adjustment(&mut problem, &metrics.adjustment)?;
        previous_area_change = Some(metrics.total_relative_increase);
        reset_after_area_adjustment = true;
        area_round = area_round
            .checked_add(1)
            .ok_or(EplaceError::InvalidNormalization)?;
    }
}

struct DensityOptimization {
    coordinates: Vec<f64>,
    best_targets: Vec<Point>,
    completed_iterations: u128,
    density_scales: Vec<f64>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum DensityStopCriterion {
    AreaAdjustmentReady,
    Final,
}

#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn optimize_density_once(
    problem: &ContinuousProblem<'_>,
    coordinates: &[f64],
    bounds: &CoordinateBounds,
    unit_count: usize,
    width: u32,
    height: u32,
    use_global_gamma: bool,
    fixed_density_scales: Option<&[f64]>,
    reset_after_area_adjustment: bool,
    stop_criterion: DensityStopCriterion,
    mut completed_iterations: u128,
) -> Result<DensityOptimization, EplaceError> {
    // Density is independent of wirelength smoothing. Probe it once per area
    // round, then rebuild all normalization, multiplier, and Nesterov state as
    // required by elfPlace equation (26).
    let initial_probe = problem.evaluate(coordinates, Some(WIRELENGTH_GAMMA_BASE_TILES))?;
    validate_initial_evaluation(&initial_probe)?;
    let initial_gamma = use_global_gamma
        .then(|| adaptive_wirelength_gamma(problem, &initial_probe.density.fields))
        .transpose()?;
    let initial = problem.evaluate(coordinates, initial_gamma)?;
    validate_initial_evaluation(&initial)?;
    let density_scales = fixed_density_scales.map_or_else(
        || {
            initial
                .density
                .fields
                .iter()
                .map(|field| field.energy.max(1.0))
                .collect::<Vec<_>>()
        },
        <[f64]>::to_vec,
    );
    validate_fields(&initial.density.fields, &density_scales)?;
    let (mut multipliers, mut multiplier_step) = if reset_after_area_adjustment {
        let multipliers = area_adjusted_density_multipliers(&initial, &density_scales)?;
        let step = area_adjusted_multiplier_step(&multipliers)?;
        (multipliers, step)
    } else {
        let initial_multiplier = initial_density_multiplier(problem, &initial, coordinates.len())?;
        let multipliers = vec![initial_multiplier; density_scales.len()];
        let step = initial_multiplier_step(
            &multipliers,
            std::env::var_os("TEXO_PNR_EPLACE_SOURCE_MULTIPLIER_STEP").is_some(),
        )?;
        (multipliers, step)
    };
    let mut optimizer = DynamicNesterovState::new(coordinates, bounds);
    let mut current = initial;
    let mut best = PlacementCheckpoint::new(
        rounded_targets_from_coordinates(optimizer.coordinates(), unit_count, width, height),
        &current.density.fields,
    );
    let mut previous_stationary_targets = None;
    loop {
        if density_stop_reached(&current.density.fields, stop_criterion) {
            break;
        }
        if eplace_iteration_limit_reached(completed_iterations) {
            return Err(did_not_converge(
                completed_iterations,
                &current.density.fields,
            ));
        }
        let global_gamma = use_global_gamma
            .then(|| adaptive_wirelength_gamma(problem, &current.density.fields))
            .transpose()?;
        completed_iterations = completed_iterations
            .checked_add(1)
            .ok_or(EplaceError::InvalidNormalization)?;
        let preconditioner =
            diagonal_preconditioner(problem, &current, &multipliers, coordinates.len())?;
        let step = dynamic_nesterov_step(&mut optimizer, bounds, &preconditioner, |candidate| {
            let evaluation = problem.evaluate(candidate, global_gamma)?;
            let objective = combined_objective(
                problem,
                &evaluation,
                &density_scales,
                &multipliers,
                candidate.len(),
            );
            Ok((objective, evaluation))
        })
        .map_err(EplaceError::Optimizer)?;
        validate_fields(&step.payload.density.fields, &density_scales)?;
        report(
            completed_iterations,
            step.objective,
            step.stationarity,
            step.coordinate_change,
            step.line_search_trials,
            global_gamma,
            step.payload.wirelength.value,
            &step.payload.density.fields,
        );
        report_force_balance(
            completed_iterations,
            problem,
            optimizer.coordinates(),
            global_gamma,
            &step.payload,
            &density_scales,
            &multipliers,
        );
        current = step.payload;
        let rounded_targets =
            rounded_targets_from_coordinates(optimizer.coordinates(), unit_count, width, height);
        best.consider(rounded_targets.clone(), &current.density.fields);
        if density_stop_reached(&current.density.fields, stop_criterion) {
            break;
        }
        if stationary_target_is_fixed(
            &mut previous_stationary_targets,
            step.status,
            &rounded_targets,
        ) {
            return Err(did_not_converge(
                completed_iterations,
                &current.density.fields,
            ));
        }
        update_density_multipliers(
            &mut multipliers,
            multiplier_step,
            &current.density.fields,
            &density_scales,
        )?;
        multiplier_step =
            next_multiplier_step(multiplier_step, &current.density.fields, &density_scales)?;
    }
    Ok(DensityOptimization {
        coordinates: optimizer.coordinates().to_vec(),
        best_targets: best.targets,
        completed_iterations,
        density_scales,
    })
}

fn area_adjustment_round_limit(problem: &ContinuousProblem<'_>) -> Result<usize, EplaceError> {
    let initial_area = problem
        .density_units
        .iter()
        .flat_map(|unit| &unit.members)
        .map(|member| member.charge)
        .sum::<f64>();
    let mut available = 0.0;
    let mut kinds = problem
        .density_units
        .iter()
        .flat_map(|unit| unit.members.iter().map(|member| member.kind))
        .collect::<Vec<_>>();
    kinds.sort_unstable();
    kinds.dedup();
    for kind in kinds {
        let capacity = problem.density_model.available_capacity(kind)?;
        available +=
            f64::from(u32::try_from(capacity).map_err(|_| EplaceError::InvalidNormalization)?);
    }
    if !(initial_area.is_finite() && initial_area > 0.0 && available >= initial_area) {
        return Err(EplaceError::InvalidNormalization);
    }
    // Every nonterminal paper round increases total physical area by at least
    // one percent. Since physical area can consume only the finite filler
    // charge, this capacity-derived bound is exhaustive rather than an
    // empirical iteration cap.
    let mut threshold_area = initial_area;
    let mut rounds = 0_usize;
    while threshold_area < available {
        threshold_area *= 1.01;
        if !threshold_area.is_finite() {
            return Err(EplaceError::InvalidNormalization);
        }
        rounds = rounds
            .checked_add(1)
            .ok_or(EplaceError::InvalidNormalization)?;
    }
    rounds
        .checked_add(1)
        .ok_or(EplaceError::InvalidNormalization)
}

fn propose_area_adjustment(
    problem: &ContinuousProblem<'_>,
    coordinates: &[f64],
    capacity: &RoutingCapacityMap,
) -> Result<AreaAdjustmentMetrics, EplaceError> {
    if capacity.width() == 0 || capacity.height() == 0 {
        return Err(EplaceError::InvalidNormalization);
    }
    let positions = coordinates[..2 * problem.unit_count]
        .as_chunks::<2>()
        .0
        .iter()
        .map(|pair| (pair[0], pair[1]))
        .collect::<Vec<_>>();
    let boxes = problem.hypergraph.external_baseline_net_bounding_boxes(
        &positions,
        problem.fixed_coordinates,
        problem.cell_offsets,
    );
    let demand = continuous_routing_demand(capacity.width(), capacity.height(), &boxes)?;
    let clustering_areas = propose_register_clustering_areas(problem, &positions)?;
    let mut members = Vec::new();
    let mut maximum_horizontal_utilization: f64 = 0.0;
    let mut maximum_vertical_utilization: f64 = 0.0;
    for ((&unit, density_unit), member_cells) in problem
        .movable_units
        .iter()
        .zip(&problem.density_units)
        .zip(&problem.density_member_cells)
    {
        debug_assert_eq!(density_unit.members.len(), member_cells.len());
        for (member, &cell) in density_unit.members.iter().zip(member_cells) {
            let origin = positions[unit];
            let point = Point::new(
                rounded_coordinate(origin.0 + member.offset_x, capacity.width()),
                rounded_coordinate(origin.1 + member.offset_y, capacity.height()),
            );
            let horizontal =
                demand.utilization(capacity, point, RoutingChannelOrientation::Horizontal)?;
            let vertical =
                demand.utilization(capacity, point, RoutingChannelOrientation::Vertical)?;
            maximum_horizontal_utilization = maximum_horizontal_utilization.max(horizontal);
            maximum_vertical_utilization = maximum_vertical_utilization.max(vertical);
            let routing_area = routability_adjusted_member_area(
                member.kind,
                member.charge,
                horizontal.min(std::f64::consts::SQRT_2),
                vertical.min(std::f64::consts::SQRT_2),
            )?;
            members.push(InstanceAreaMember {
                unit,
                kind: member.kind,
                current_area: member.charge,
                routability_area: routing_area,
                // Pin-density adjustment is supplied independently by the
                // architecture pin map.
                pin_area: 0.0,
                clustering_area: clustering_areas.get(&cell).copied().unwrap_or(0.0),
            });
        }
    }
    let fillers = problem
        .fillers
        .iter()
        .map(|filler| InstanceAreaFiller {
            kind: filler.kind,
            current_area: filler.charge,
        })
        .collect::<Vec<_>>();
    let adjustment = adjust_instance_areas(&members, &fillers)?;
    let mut before_by_kind = BTreeMap::<ResourceKind, f64>::new();
    let mut increase_by_kind = BTreeMap::<ResourceKind, f64>::new();
    let mut members_inflated = 0;
    for ((member, area), metadata) in members
        .iter()
        .zip(&adjustment.member_areas)
        .zip(problem.density_units.iter().flat_map(|unit| &unit.members))
    {
        debug_assert_eq!(member.kind, metadata.kind);
        *before_by_kind.entry(member.kind).or_default() += member.current_area;
        let increase = area - member.current_area;
        *increase_by_kind.entry(member.kind).or_default() += increase;
        members_inflated += usize::from(increase > 0.0);
    }
    let before_total = before_by_kind.values().sum::<f64>();
    let increase_total = increase_by_kind.values().sum::<f64>();
    let total_relative_increase = increase_total / before_total;
    let maximum_relative_increase = before_by_kind
        .iter()
        .map(|(kind, before)| increase_by_kind.get(kind).copied().unwrap_or(0.0) / before)
        .fold(0.0_f64, f64::max);
    if !total_relative_increase.is_finite() || !maximum_relative_increase.is_finite() {
        return Err(EplaceError::InvalidNormalization);
    }
    Ok(AreaAdjustmentMetrics {
        maximum_relative_increase,
        total_relative_increase,
        maximum_horizontal_utilization,
        maximum_vertical_utilization,
        members_inflated,
        adjustment,
    })
}

/// Converts routing congestion into artificial density area only for the
/// exchangeable slice resources that can actually consume that area.
///
/// A BRAM remains one indivisible hard-site occupant.  Inflating its charge in
/// the Memory-only electrostatic field does not reserve any of the congested
/// general-routing or slice capacity; it merely asks one BRAM to occupy a
/// fractional second BRAM site.  On sparse hard-block rows this can make the
/// continuous stopping overflow unattainable even though exact legalization
/// has ample sites.  BRAMs still move through the wirelength and Memory
/// density objectives, but retain unit physical charge during RUDY updates.
fn routability_adjusted_member_area(
    kind: ResourceKind,
    current_area: f64,
    horizontal_utilization: f64,
    vertical_utilization: f64,
) -> Result<f64, InstanceAreaError> {
    let inflated =
        routability_optimized_area(current_area, horizontal_utilization, vertical_utilization)?;
    match kind {
        ResourceKind::Lut(_) | ResourceKind::Register => Ok(inflated),
        ResourceKind::Memory
        | ResourceKind::Logic
        | ResourceKind::Clock
        | ResourceKind::Io
        | ResourceKind::Constant => Ok(current_area),
    }
}

fn propose_register_clustering_areas(
    problem: &ContinuousProblem<'_>,
    positions: &[(f64, f64)],
) -> Result<BTreeMap<texo_model::CellId, f64>, EplaceError> {
    if problem.register_controls.is_empty() {
        return Ok(BTreeMap::new());
    }
    let movable_registers = problem
        .movable_units
        .iter()
        .zip(&problem.density_units)
        .zip(&problem.density_member_cells)
        .flat_map(|((&unit, density_unit), cells)| {
            let origin = positions[unit];
            cells
                .iter()
                .zip(&density_unit.members)
                .filter(|(_, member)| member.kind == ResourceKind::Register)
                .map(move |(&cell, member)| MovableRegister {
                    cell,
                    x: origin.0 + member.offset_x,
                    y: origin.1 + member.offset_y,
                })
        })
        .collect::<Vec<_>>();
    clustering_areas(
        &movable_registers,
        &problem.register_controls,
        problem.movable_instance_count,
    )
    .ok_or(EplaceError::InvalidNormalization)
}

fn apply_area_adjustment(
    problem: &mut ContinuousProblem<'_>,
    adjustment: &InstanceAreaAdjustment,
) -> Result<(), EplaceError> {
    let member_count = problem
        .density_units
        .iter()
        .map(|unit| unit.members.len())
        .sum::<usize>();
    if member_count != adjustment.member_areas.len()
        || problem.fillers.len() != adjustment.filler_areas.len()
    {
        return Err(EplaceError::InvalidNormalization);
    }
    for (member, &area) in problem
        .density_units
        .iter_mut()
        .flat_map(|unit| &mut unit.members)
        .zip(&adjustment.member_areas)
    {
        member.charge = area;
    }
    for (filler, &area) in problem.fillers.iter_mut().zip(&adjustment.filler_areas) {
        filler.charge = area;
    }
    Ok(())
}

impl ContinuousRoutingDemand {
    fn utilization(
        &self,
        capacity: &RoutingCapacityMap,
        point: Point,
        orientation: RoutingChannelOrientation,
    ) -> Result<f64, EplaceError> {
        if point.x >= self.width || point.y >= capacity.height() {
            return Err(EplaceError::InvalidNormalization);
        }
        let index = usize::try_from(point.y * self.width + point.x)
            .map_err(|_| EplaceError::InvalidNormalization)?;
        let demand = match orientation {
            RoutingChannelOrientation::Horizontal => self.horizontal[index],
            RoutingChannelOrientation::Vertical => self.vertical[index],
        };
        let channel_capacity = capacity
            .capacity(point, orientation)
            .ok_or(EplaceError::InvalidNormalization)?;
        if channel_capacity == 0 {
            return Ok(if demand == 0.0 {
                0.0
            } else {
                std::f64::consts::SQRT_2
            });
        }
        let utilization = demand / f64::from(channel_capacity);
        if utilization.is_finite() && utilization >= 0.0 {
            Ok(utilization)
        } else {
            Err(EplaceError::InvalidNormalization)
        }
    }
}

fn continuous_routing_demand(
    width: u32,
    height: u32,
    boxes: &[(f64, f64, f64, f64)],
) -> Result<ContinuousRoutingDemand, EplaceError> {
    let tile_count = usize::try_from(width)
        .ok()
        .and_then(|width| {
            usize::try_from(height)
                .ok()
                .and_then(|height| width.checked_mul(height))
        })
        .ok_or(EplaceError::InvalidNormalization)?;
    let mut demand = ContinuousRoutingDemand {
        width,
        horizontal: vec![0.0; tile_count],
        vertical: vec![0.0; tile_count],
    };
    for &(minimum_x, maximum_x, minimum_y, maximum_y) in boxes {
        if ![minimum_x, maximum_x, minimum_y, maximum_y]
            .iter()
            .all(|coordinate| coordinate.is_finite())
            || minimum_x > maximum_x
            || minimum_y > maximum_y
        {
            return Err(EplaceError::InvalidNormalization);
        }
        let dx = maximum_x - minimum_x;
        let dy = maximum_y - minimum_y;
        let rectangle_minimum_x = minimum_x - 0.5;
        let rectangle_maximum_x = maximum_x + 0.5;
        let rectangle_minimum_y = minimum_y - 0.5;
        let rectangle_maximum_y = maximum_y + 0.5;
        let area = (dx + 1.0) * (dy + 1.0);
        if !area.is_finite() || area <= 0.0 {
            return Err(EplaceError::InvalidNormalization);
        }
        let horizontal_density = dx / area;
        let vertical_density = dy / area;
        let mut y_overlaps = Vec::new();
        for y in 0..height {
            let tile_minimum_y = f64::from(y) - 0.5;
            let tile_maximum_y = f64::from(y) + 0.5;
            let overlap_y = (tile_maximum_y.min(rectangle_maximum_y)
                - tile_minimum_y.max(rectangle_minimum_y))
            .max(0.0);
            if overlap_y > 0.0 {
                y_overlaps.push((y, overlap_y));
            }
        }
        let mut x_overlaps = Vec::new();
        for x in 0..width {
            let tile_minimum_x = f64::from(x) - 0.5;
            let tile_maximum_x = f64::from(x) + 0.5;
            let overlap_x = (tile_maximum_x.min(rectangle_maximum_x)
                - tile_minimum_x.max(rectangle_minimum_x))
            .max(0.0);
            if overlap_x > 0.0 {
                x_overlaps.push((x, overlap_x));
            }
        }
        for &(y, overlap_y) in &y_overlaps {
            for &(x, overlap_x) in &x_overlaps {
                let overlap = overlap_x * overlap_y;
                let index = usize::try_from(y * width + x)
                    .map_err(|_| EplaceError::InvalidNormalization)?;
                demand.horizontal[index] += horizontal_density * overlap;
                demand.vertical[index] += vertical_density * overlap;
            }
        }
    }
    if demand
        .horizontal
        .iter()
        .chain(&demand.vertical)
        .all(|entry| entry.is_finite() && *entry >= 0.0)
    {
        Ok(demand)
    } else {
        Err(EplaceError::InvalidNormalization)
    }
}

fn report_area_adjustment(round: usize, metrics: &AreaAdjustmentMetrics) {
    if std::env::var_os("TEXO_PNR_METRICS").is_none() {
        return;
    }
    eprintln!(
        "[metrics] eplace_area_adjustment round={round} members_inflated={} total_relative_increase={:.6} maximum_field_relative_increase={:.6} maximum_horizontal_utilization={:.6} maximum_vertical_utilization={:.6} resource_scales={:?}",
        metrics.members_inflated,
        metrics.total_relative_increase,
        metrics.maximum_relative_increase,
        metrics.maximum_horizontal_utilization,
        metrics.maximum_vertical_utilization,
        metrics.adjustment.resource_scales,
    );
}

fn rounded_targets_from_coordinates(
    coordinates: &[f64],
    unit_count: usize,
    width: u32,
    height: u32,
) -> Vec<Point> {
    coordinates[..2 * unit_count]
        .as_chunks::<2>()
        .0
        .iter()
        .map(|pair| {
            Point::new(
                rounded_coordinate(pair[0], width),
                rounded_coordinate(pair[1], height),
            )
        })
        .collect()
}

fn stationary_target_is_fixed(
    previous: &mut Option<Vec<Point>>,
    status: DynamicNesterovStatus,
    targets: &[Point],
) -> bool {
    if status != DynamicNesterovStatus::NumericallyStationary {
        *previous = None;
        return false;
    }
    let fixed = previous.as_deref() == Some(targets);
    *previous = Some(targets.to_vec());
    fixed
}

fn did_not_converge(iterations: u128, fields: &[DensityFieldResult]) -> EplaceError {
    EplaceError::DidNotConverge {
        iterations,
        overflows: fields
            .iter()
            .map(|field| (field.kind, field.normalized_positive_overflow))
            .collect(),
    }
}

fn eplace_iteration_limit_reached(completed_iterations: u128) -> bool {
    completed_iterations >= MAX_EPLACE_ITERATIONS
}

fn rounded_unit_targets(x: &[f64], y: &[f64], width: u32, height: u32) -> Vec<Point> {
    x.iter()
        .zip(y)
        .map(|(&x, &y)| Point::new(rounded_coordinate(x, width), rounded_coordinate(y, height)))
        .collect()
}

fn combined_objective(
    problem: &ContinuousProblem<'_>,
    evaluation: &ContinuousEvaluation,
    density_scales: &[f64],
    multipliers: &[f64],
    coordinate_count: usize,
) -> ObjectiveEvaluation {
    debug_assert_eq!(density_scales.len(), evaluation.density.fields.len());
    debug_assert_eq!(multipliers.len(), evaluation.density.fields.len());
    let mut value = evaluation.wirelength.value;
    let mut gradient = wirelength_gradient(&evaluation.wirelength);
    gradient.resize(coordinate_count, 0.0);
    for (index, field) in evaluation.density.fields.iter().enumerate() {
        let (field_value, coefficient) = augmented_density_value_and_coefficient(
            field.energy,
            density_scales[index],
            multipliers[index],
        );
        value += field_value;
        problem.add_scaled_field_gradient(field, coefficient, &mut gradient);
    }
    ObjectiveEvaluation { value, gradient }
}

fn diagonal_preconditioner(
    problem: &ContinuousProblem<'_>,
    evaluation: &ContinuousEvaluation,
    multipliers: &[f64],
    coordinate_count: usize,
) -> Result<Vec<f64>, EplaceError> {
    let wirelength = problem
        .hypergraph
        .wirelength_preconditioner(problem.unit_count);
    let mut diagonal = Vec::with_capacity(coordinate_count);
    for degree in wirelength {
        diagonal.extend([degree; 2]);
    }
    diagonal.resize(coordinate_count, 0.0);

    for (index, field) in evaluation.density.fields.iter().enumerate() {
        let coefficient = multipliers[index];
        if !coefficient.is_finite() || coefficient < 0.0 {
            return Err(EplaceError::InvalidNormalization);
        }
        for (&unit, density_unit) in problem.movable_units.iter().zip(&problem.density_units) {
            let charge = density_unit
                .members
                .iter()
                .filter(|member| member.kind == field.kind)
                .map(|member| member.charge)
                .sum::<f64>();
            if charge == 0.0 {
                continue;
            }
            let curvature = coefficient * charge;
            diagonal[2 * unit] += curvature;
            diagonal[2 * unit + 1] += curvature;
        }
        for (filler_index, filler) in problem.fillers.iter().enumerate() {
            if filler.kind != field.kind {
                continue;
            }
            let coordinate = 2 * (problem.unit_count + filler_index);
            let curvature = coefficient * filler.charge;
            diagonal[coordinate] += curvature;
            diagonal[coordinate + 1] += curvature;
        }
    }

    for entry in &mut diagonal {
        if !entry.is_finite() || *entry < 0.0 {
            return Err(EplaceError::InvalidNormalization);
        }
        // elfPlace equation (16) floors the Jacobi approximation at one,
        // including for disconnected fillers.
        *entry = entry.max(1.0);
    }
    Ok(diagonal)
}

fn wirelength_gradient(objective: &WeightedAverageObjective) -> Vec<f64> {
    objective
        .gradient_x
        .iter()
        .zip(&objective.gradient_y)
        .flat_map(|(&x, &y)| [x, y])
        .collect()
}

fn augmented_density_value_and_coefficient(
    energy: f64,
    initial_energy: f64,
    multiplier: f64,
) -> (f64, f64) {
    let normalized_energy = energy / initial_energy;
    (
        multiplier
            * initial_energy
            * (normalized_energy
                + 0.5 * AUGMENTED_DENSITY_BETA * normalized_energy * normalized_energy),
        multiplier * (1.0 + AUGMENTED_DENSITY_BETA * normalized_energy),
    )
}

fn initial_density_multiplier(
    problem: &ContinuousProblem<'_>,
    evaluation: &ContinuousEvaluation,
    coordinate_count: usize,
) -> Result<f64, EplaceError> {
    let wirelength_force = l1_norm(&wirelength_gradient(&evaluation.wirelength));
    let density_force = evaluation
        .density
        .fields
        .iter()
        .map(|field| l1_norm(&problem.field_gradient(field, coordinate_count)))
        .sum::<f64>();
    let multiplier = if wirelength_force == 0.0 || density_force == 0.0 {
        1.0
    } else {
        INITIAL_DENSITY_WEIGHT * wirelength_force / density_force
    };
    if multiplier.is_finite() && multiplier > 0.0 {
        Ok(multiplier)
    } else {
        Err(EplaceError::InvalidNormalization)
    }
}

fn l1_norm(values: &[f64]) -> f64 {
    values.iter().map(|value| value.abs()).sum()
}

fn normalized_density_subgradient(
    fields: &[DensityFieldResult],
    initial_energies: &[f64],
) -> Result<Vec<f64>, EplaceError> {
    validate_fields(fields, initial_energies)?;
    let subgradient = fields
        .iter()
        .zip(initial_energies)
        .map(|(field, &initial_energy)| {
            let normalized_energy = field.energy / initial_energy;
            normalized_energy + 0.5 * AUGMENTED_DENSITY_BETA * normalized_energy * normalized_energy
        })
        .collect::<Vec<_>>();
    if subgradient
        .iter()
        .all(|value| value.is_finite() && *value >= 0.0)
    {
        Ok(subgradient)
    } else {
        Err(EplaceError::InvalidNormalization)
    }
}

/// Reinitializes the heterogeneous density multipliers after equation (26).
///
/// elfPlace equation (27) scales the normalized augmented-energy
/// subgradient by the ratio between wirelength force and the charge-weighted
/// electrostatic force.  `force_l1` is accumulated per physical member before
/// rigid-macro forces can cancel at their shared optimizer coordinate.
fn area_adjusted_density_multipliers(
    evaluation: &ContinuousEvaluation,
    initial_energies: &[f64],
) -> Result<Vec<f64>, EplaceError> {
    let subgradient = normalized_density_subgradient(&evaluation.density.fields, initial_energies)?;
    let wirelength_force = l1_norm(&wirelength_gradient(&evaluation.wirelength));
    let density_force = evaluation
        .density
        .fields
        .iter()
        .zip(&subgradient)
        .map(|(field, &subgradient)| field.force_l1 * subgradient)
        .sum::<f64>();
    if !wirelength_force.is_finite()
        || wirelength_force <= 0.0
        || !density_force.is_finite()
        || density_force <= 0.0
    {
        return Err(EplaceError::InvalidNormalization);
    }
    let scale = AREA_RESET_DENSITY_WEIGHT * wirelength_force / density_force;
    let multipliers = subgradient
        .into_iter()
        .map(|entry| scale * entry)
        .collect::<Vec<_>>();
    if multipliers
        .iter()
        .all(|multiplier| multiplier.is_finite() && *multiplier >= 0.0)
        && multipliers.iter().any(|multiplier| *multiplier > 0.0)
    {
        Ok(multipliers)
    } else {
        Err(EplaceError::InvalidNormalization)
    }
}

/// elfPlace equation (28): `t0 = (alpha_high - 1) * ||lambda'||_2`.
fn area_adjusted_multiplier_step(multipliers: &[f64]) -> Result<f64, EplaceError> {
    let norm = multipliers
        .iter()
        .map(|multiplier| multiplier * multiplier)
        .sum::<f64>()
        .sqrt();
    let step = (MULTIPLIER_ALPHA_HIGH - 1.0) * norm;
    if step.is_finite() && step > 0.0 {
        Ok(step)
    } else {
        Err(EplaceError::InvalidNormalization)
    }
}

fn initial_multiplier_step(multipliers: &[f64], source_scaled: bool) -> Result<f64, EplaceError> {
    let step = if source_scaled {
        let norm = multipliers
            .iter()
            .map(|multiplier| multiplier * multiplier)
            .sum::<f64>()
            .sqrt();
        (MULTIPLIER_ALPHA_LOW - 1.0) * norm
    } else {
        MULTIPLIER_ALPHA_HIGH - 1.0
    };
    if step.is_finite() && step > 0.0 {
        Ok(step)
    } else {
        Err(EplaceError::InvalidNormalization)
    }
}

fn update_density_multipliers(
    multipliers: &mut [f64],
    step: f64,
    fields: &[DensityFieldResult],
    initial_energies: &[f64],
) -> Result<(), EplaceError> {
    let subgradient = normalized_density_subgradient(fields, initial_energies)?;
    let norm = subgradient
        .iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt();
    if norm == 0.0 {
        return Ok(());
    }
    if !norm.is_finite() || !step.is_finite() || step <= 0.0 {
        return Err(EplaceError::InvalidNormalization);
    }
    for (multiplier, subgradient) in multipliers.iter_mut().zip(subgradient) {
        *multiplier += step * subgradient / norm;
        if !multiplier.is_finite() || *multiplier < 0.0 {
            return Err(EplaceError::InvalidNormalization);
        }
    }
    Ok(())
}

fn next_multiplier_step(
    previous: f64,
    fields: &[DensityFieldResult],
    initial_energies: &[f64],
) -> Result<f64, EplaceError> {
    validate_fields(fields, initial_energies)?;
    let normalized_energy_norm = fields
        .iter()
        .zip(initial_energies)
        .map(|(field, &initial_energy)| {
            let normalized = field.energy / initial_energy;
            normalized * normalized
        })
        .sum::<f64>()
        .sqrt();
    let logarithm = (AUGMENTED_DENSITY_BETA * normalized_energy_norm + 1.0).ln();
    let growth = multiplier_growth_from_logarithm(logarithm)?;
    let next = previous * growth;
    if next.is_finite() && next > 0.0 {
        Ok(next)
    } else {
        Err(EplaceError::InvalidNormalization)
    }
}

fn multiplier_growth_from_logarithm(logarithm: f64) -> Result<f64, EplaceError> {
    if !logarithm.is_finite() || logarithm < 0.0 {
        return Err(EplaceError::InvalidNormalization);
    }
    let interpolation = 1.0 - 1.0 / (1.0 + logarithm);
    let growth =
        MULTIPLIER_ALPHA_LOW + interpolation * (MULTIPLIER_ALPHA_HIGH - MULTIPLIER_ALPHA_LOW);
    if growth.is_finite() && (MULTIPLIER_ALPHA_LOW..=MULTIPLIER_ALPHA_HIGH).contains(&growth) {
        Ok(growth)
    } else {
        Err(EplaceError::InvalidNormalization)
    }
}

fn density_overflow_target(kind: ResourceKind) -> f64 {
    match kind {
        ResourceKind::Memory => 0.20,
        ResourceKind::Lut(_) | ResourceKind::Register => 0.10,
        ResourceKind::Logic | ResourceKind::Clock | ResourceKind::Io | ResourceKind::Constant => {
            0.0
        }
    }
}

fn overflow_excess_score(fields: &[DensityFieldResult]) -> (f64, f64) {
    fields.iter().fold((0.0_f64, 0.0_f64), |score, field| {
        let excess =
            (field.normalized_positive_overflow - density_overflow_target(field.kind)).max(0.0);
        (score.0.max(excess), score.1 + excess)
    })
}

/// `DREAMPlaceFPGA`'s heterogeneous elfPlace gamma update.
///
/// For field `s`, `gamma_s = base_s * 10^(k_s O_s + b_s)`, where
/// `k_s = 2/(1-target_s)` and `b_s = 1-k_s`.  This makes gamma one tenth of
/// its base at the field's stopping overflow and ten times its base at unit
/// overflow.  The placement-wide gamma is the field gamma mean weighted by
/// the incident-net preconditioner of nodes belonging to that field.
fn adaptive_wirelength_gamma(
    problem: &ContinuousProblem<'_>,
    fields: &[DensityFieldResult],
) -> Result<f64, EplaceError> {
    let weighted_fields = fields
        .iter()
        .map(|field| {
            let target = density_overflow_target(field.kind);
            let gamma = field_wirelength_gamma(
                field.normalized_positive_overflow,
                target,
                WIRELENGTH_GAMMA_BASE_TILES,
            )?;
            let weight = problem
                .density_gamma_weights
                .get(&field.kind)
                .copied()
                .unwrap_or(0.0);
            Ok((gamma, weight))
        })
        .collect::<Result<Vec<_>, EplaceError>>()?;
    weighted_gamma_mean(&weighted_fields, WIRELENGTH_GAMMA_BASE_TILES)
}

fn field_wirelength_gamma(overflow: f64, target: f64, base: f64) -> Result<f64, EplaceError> {
    if !overflow.is_finite()
        || overflow < 0.0
        || !target.is_finite()
        || !(0.0..1.0).contains(&target)
        || !base.is_finite()
        || base <= 0.0
    {
        return Err(EplaceError::InvalidNormalization);
    }
    let slope = 2.0 / (1.0 - target);
    let intercept = 1.0 - slope;
    let gamma = base * 10.0_f64.powf(slope * overflow + intercept);
    if gamma.is_finite() && gamma > 0.0 {
        Ok(gamma)
    } else {
        Err(EplaceError::InvalidNormalization)
    }
}

fn weighted_gamma_mean(
    gamma_and_weights: &[(f64, f64)],
    zero_weight_fallback: f64,
) -> Result<f64, EplaceError> {
    if !zero_weight_fallback.is_finite() || zero_weight_fallback <= 0.0 {
        return Err(EplaceError::InvalidNormalization);
    }
    let mut weighted_sum = 0.0;
    let mut weight_sum = 0.0;
    for &(gamma, weight) in gamma_and_weights {
        if !gamma.is_finite() || gamma <= 0.0 || !weight.is_finite() || weight < 0.0 {
            return Err(EplaceError::InvalidNormalization);
        }
        weighted_sum += gamma * weight;
        weight_sum += weight;
    }
    let mean = if weight_sum == 0.0 {
        zero_weight_fallback
    } else {
        weighted_sum / weight_sum
    };
    if mean.is_finite() && mean > 0.0 {
        Ok(mean)
    } else {
        Err(EplaceError::InvalidNormalization)
    }
}

fn density_is_converged(fields: &[DensityFieldResult]) -> bool {
    fields
        .iter()
        .all(|field| field.normalized_positive_overflow <= density_overflow_target(field.kind))
}

fn area_adjustment_is_ready(fields: &[DensityFieldResult]) -> bool {
    fields
        .iter()
        .filter(|field| matches!(field.kind, ResourceKind::Lut(_) | ResourceKind::Register))
        .all(|field| field.normalized_positive_overflow <= AREA_ADJUSTMENT_OVERFLOW_TARGET)
}

fn density_stop_reached(fields: &[DensityFieldResult], criterion: DensityStopCriterion) -> bool {
    match criterion {
        DensityStopCriterion::AreaAdjustmentReady => area_adjustment_is_ready(fields),
        DensityStopCriterion::Final => density_is_converged(fields),
    }
}

fn density_kind_is_exchangeable(kind: ResourceKind, include_special: bool) -> bool {
    include_special
        || matches!(
            kind,
            ResourceKind::Lut(_) | ResourceKind::Register | ResourceKind::Memory
        )
}

fn density_gamma_weights(
    units: &[PlacementUnit],
    device: &Device,
    cell_pin_weights: &[f64],
    include_special: bool,
) -> BTreeMap<ResourceKind, f64> {
    let mut weights = BTreeMap::<ResourceKind, f64>::new();
    for unit in units.iter().filter(|unit| unit.choices.len() > 1) {
        for (&cell, &bel) in unit.cells.iter().zip(unit.choices.assignment(0)) {
            let kind = device.bels()[bel.0].kind;
            if density_kind_is_exchangeable(kind, include_special) {
                *weights.entry(kind).or_default() += cell_pin_weights[cell.0];
            }
        }
    }
    weights
}

fn apply_paper_initialization(
    coordinates: &mut [f64],
    bounds: &CoordinateBounds,
    units: &[PlacementUnit],
    device_width: u32,
    device_height: u32,
) {
    let movable = units
        .iter()
        .enumerate()
        .filter_map(|(index, unit)| (unit.choices.len() > 1).then_some(index))
        .collect::<Vec<_>>();
    let offsets = deterministic_gaussian_offsets(&movable, device_width, device_height);
    apply_origin_offsets(coordinates, bounds, &movable, &offsets);
}

fn apply_origin_offsets(
    coordinates: &mut [f64],
    bounds: &CoordinateBounds,
    movable_units: &[usize],
    offsets: &[(f64, f64)],
) {
    debug_assert_eq!(movable_units.len(), offsets.len());
    for (&unit, &(offset_x, offset_y)) in movable_units.iter().zip(offsets) {
        let coordinate = 2 * unit;
        coordinates[coordinate] = (coordinates[coordinate] + offset_x)
            .clamp(bounds.lower[coordinate], bounds.upper[coordinate]);
        coordinates[coordinate + 1] = (coordinates[coordinate + 1] + offset_y)
            .clamp(bounds.lower[coordinate + 1], bounds.upper[coordinate + 1]);
    }
}

/// Generates one Gaussian origin perturbation per placement unit.
///
/// Hashing only the stable unit index makes the sequence reproducible without
/// a global PRNG state.  Removing each axis mean prevents the perturbation
/// itself from translating the whole placement.  Atomic macro members never
/// appear here: their shared origin receives exactly one `(dx, dy)` pair.
fn deterministic_gaussian_offsets(
    movable_units: &[usize],
    device_width: u32,
    device_height: u32,
) -> Vec<(f64, f64)> {
    if movable_units.is_empty() {
        return Vec::new();
    }
    let standard_deviation_x = PAPER_INITIALIZATION_STDDEV_FRACTION * f64::from(device_width);
    let standard_deviation_y = PAPER_INITIALIZATION_STDDEV_FRACTION * f64::from(device_height);
    let mut offsets = movable_units
        .iter()
        .map(|&unit| {
            let ordinal = u64::try_from(unit).expect("placement unit index fits u64");
            let uniform_radius = open_unit_interval(splitmix64(ordinal ^ 0x243f_6a88_85a3_08d3));
            let uniform_angle = open_unit_interval(splitmix64(ordinal ^ 0x1319_8a2e_0370_7344));
            let radius = (-2.0 * uniform_radius.ln()).sqrt();
            let angle = TAU * uniform_angle;
            let (sine, cosine) = angle.sin_cos();
            (
                standard_deviation_x * radius * cosine,
                standard_deviation_y * radius * sine,
            )
        })
        .collect::<Vec<_>>();
    let denominator = usize_as_f64(offsets.len());
    let mean_x = offsets.iter().map(|(x, _)| x).sum::<f64>() / denominator;
    let mean_y = offsets.iter().map(|(_, y)| y).sum::<f64>() / denominator;
    for (x, y) in &mut offsets {
        *x -= mean_x;
        *y -= mean_y;
    }
    offsets
}

fn splitmix64(mut value: u64) -> u64 {
    value = value.wrapping_add(0x9e37_79b9_7f4a_7c15);
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
}

fn open_unit_interval(value: u64) -> f64 {
    let upper = u32::try_from(value >> 32).expect("upper half of u64 fits u32");
    (f64::from(upper) + 0.5) / (f64::from(u32::MAX) + 1.0)
}

fn initial_coordinates_and_bounds(
    units: &[PlacementUnit],
    device: &Device,
    initial_x: &[f64],
    initial_y: &[f64],
    fillers: &[DensityFiller],
) -> (Vec<f64>, CoordinateBounds) {
    let mut bounds_cache = BTreeMap::new();
    let mut coordinates = Vec::with_capacity(2 * (units.len() + fillers.len()));
    let mut lower = Vec::with_capacity(coordinates.capacity());
    let mut upper = Vec::with_capacity(coordinates.capacity());
    for (unit_index, unit) in units.iter().enumerate() {
        coordinates.extend([initial_x[unit_index], initial_y[unit_index]]);
        let (minimum, maximum) = *bounds_cache
            .entry(unit.choices.cache_key())
            .or_insert_with(|| choice_bounds(unit, device));
        lower.extend([f64::from(minimum.x), f64::from(minimum.y)]);
        upper.extend([f64::from(maximum.x), f64::from(maximum.y)]);
    }
    let maximum_x = f64::from(device.width() - 1);
    let maximum_y = f64::from(device.height() - 1);
    for filler in fillers {
        coordinates.extend([filler.x, filler.y]);
        lower.extend([0.0, 0.0]);
        upper.extend([maximum_x, maximum_y]);
    }
    (coordinates, CoordinateBounds { lower, upper })
}

fn choice_bounds(unit: &PlacementUnit, device: &Device) -> (Point, Point) {
    let mut minimum = Point::new(u32::MAX, u32::MAX);
    let mut maximum = Point::new(0, 0);
    for index in 0..unit.choices.len() {
        let point = device.bels()[unit.choices.assignment(index)[0].0].point;
        minimum.x = minimum.x.min(point.x);
        minimum.y = minimum.y.min(point.y);
        maximum.x = maximum.x.max(point.x);
        maximum.y = maximum.y.max(point.y);
    }
    (minimum, maximum)
}

fn validate_fields(fields: &[DensityFieldResult], scales: &[f64]) -> Result<(), EplaceError> {
    if fields.len() != scales.len()
        || fields
            .iter()
            .zip(scales)
            .any(|(field, &scale)| !field.energy.is_finite() || !scale.is_finite() || scale <= 0.0)
    {
        return Err(EplaceError::DensityFieldChanged);
    }
    Ok(())
}

fn validate_initial_evaluation(evaluation: &ContinuousEvaluation) -> Result<(), EplaceError> {
    let wirelength_is_finite = evaluation.wirelength.value.is_finite()
        && evaluation
            .wirelength
            .gradient_x
            .iter()
            .chain(&evaluation.wirelength.gradient_y)
            .all(|value| value.is_finite());
    let density_is_finite = evaluation.density.fields.iter().all(|field| {
        field.energy.is_finite()
            && field.energy >= 0.0
            && field.normalized_positive_overflow.is_finite()
            && field.net_charge.is_finite()
            && field.force_l1.is_finite()
            && field.force_l1 >= 0.0
            && field
                .unit_gradients
                .iter()
                .all(|(x, y)| x.is_finite() && y.is_finite())
            && field
                .filler_gradients
                .iter()
                .all(|(_, x, y)| x.is_finite() && y.is_finite())
    });
    if wirelength_is_finite && density_is_finite {
        Ok(())
    } else {
        Err(EplaceError::InvalidNormalization)
    }
}

fn raw_positive_overflow(field: &DensityFieldResult) -> f64 {
    field.normalized_positive_overflow * field.real_area
}

fn usize_as_f64(value: usize) -> f64 {
    f64::from(u32::try_from(value).expect("placement resource count fits u32"))
}

#[allow(clippy::too_many_arguments)]
fn report(
    outer_iteration: u128,
    objective: f64,
    attained_stationarity: f64,
    coordinate_change: f64,
    line_search_trials: u128,
    global_gamma: Option<f64>,
    wirelength: f64,
    fields: &[DensityFieldResult],
) {
    if std::env::var_os("TEXO_PNR_METRICS").is_none() {
        return;
    }
    let gamma = global_gamma.map_or_else(
        || "legacy-net-local".to_owned(),
        |value| format!("{value:.9e}"),
    );
    eprintln!(
        "TEXO_PNR_METRICS eplace iteration={outer_iteration} objective={objective:.9e} stationarity={attained_stationarity:.9e} coordinate_change={coordinate_change:.9e} line_search_trials={line_search_trials} gamma={gamma} wa={wirelength:.9e}"
    );
    for field in fields {
        eprintln!(
            "TEXO_PNR_METRICS eplace-density iteration={outer_iteration} kind={:?} energy={:.9e} overflow={:.9e} real={} area={:.9e} filler={:.9e} net_charge={:.9e}",
            field.kind,
            field.energy,
            raw_positive_overflow(field),
            field.real_charge,
            field.real_area,
            field.filler_charge,
            field.net_charge,
        );
    }
}

fn report_force_balance(
    outer_iteration: u128,
    problem: &ContinuousProblem<'_>,
    coordinates: &[f64],
    global_gamma: Option<f64>,
    evaluation: &ContinuousEvaluation,
    density_scales: &[f64],
    multipliers: &[f64],
) {
    if std::env::var_os("TEXO_PNR_METRICS").is_none() {
        return;
    }
    let Some(gamma) = global_gamma else {
        return;
    };
    let unit_positions = coordinates[..2 * problem.unit_count]
        .as_chunks::<2>()
        .0
        .iter()
        .map(|pair| (pair[0], pair[1]))
        .collect::<Vec<_>>();
    let (baseline, timing) = problem
        .hypergraph
        .weighted_average_components_with_global_gamma(
            &unit_positions,
            problem.fixed_coordinates,
            problem.cell_offsets,
            gamma,
        );
    let baseline_l1 = l1_norm(&wirelength_gradient(&baseline));
    let timing_l1 = l1_norm(&wirelength_gradient(&timing));
    let wirelength_l1 = l1_norm(&wirelength_gradient(&evaluation.wirelength));
    let mut total_density_gradient = vec![0.0; coordinates.len()];
    for (index, field) in evaluation.density.fields.iter().enumerate() {
        let (_, coefficient) = augmented_density_value_and_coefficient(
            field.energy,
            density_scales[index],
            multipliers[index],
        );
        let mut field_gradient = problem.field_gradient(field, coordinates.len());
        let raw_l1 = l1_norm(&field_gradient);
        for (total, field) in total_density_gradient.iter_mut().zip(&mut field_gradient) {
            *field *= coefficient;
            *total += *field;
        }
        eprintln!(
            "TEXO_PNR_METRICS eplace-field-force iteration={outer_iteration} kind={:?} multiplier={:.9e} coefficient={coefficient:.9e} raw_l1={raw_l1:.9e} weighted_l1={:.9e} normalized_overflow={:.9e} target={:.9e}",
            field.kind,
            multipliers[index],
            l1_norm(&field_gradient),
            field.normalized_positive_overflow,
            density_overflow_target(field.kind),
        );
    }
    let density_l1 = l1_norm(&total_density_gradient);
    eprintln!(
        "TEXO_PNR_METRICS eplace-force iteration={outer_iteration} gamma={gamma:.9e} baseline_l1={baseline_l1:.9e} timing_l1={timing_l1:.9e} wirelength_l1={wirelength_l1:.9e} density_l1={density_l1:.9e} timing_to_baseline={:.9e} density_to_wirelength={:.9e}",
        timing_l1 / baseline_l1.max(f64::MIN_POSITIVE),
        density_l1 / wirelength_l1.max(f64::MIN_POSITIVE),
    );
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use texo_model::{CellId, Device, Point, ResourceKind};

    use crate::{PlacementChoices, PlacementUnit};

    use super::{
        AREA_ADJUSTMENT_OVERFLOW_TARGET, AUGMENTED_DENSITY_BETA, ContinuousEvaluation,
        CoordinateBounds, DensityFieldResult, DensityResult, DynamicNesterovStatus,
        MULTIPLIER_ALPHA_HIGH, MULTIPLIER_ALPHA_LOW, PlacementCheckpoint, WeightedAverageObjective,
        apply_origin_offsets, area_adjusted_density_multipliers, area_adjusted_multiplier_step,
        area_adjustment_is_ready, augmented_density_value_and_coefficient,
        continuous_routing_demand, density_gamma_weights, density_is_converged,
        density_kind_is_exchangeable, deterministic_gaussian_offsets,
        eplace_iteration_limit_reached, field_wirelength_gamma, initial_multiplier_step,
        multiplier_growth_from_logarithm, open_unit_interval, routability_adjusted_member_area,
        splitmix64, stationary_target_is_fixed, usize_as_f64, weighted_gamma_mean,
    };

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {expected:.17e}, got {actual:.17e}"
        );
    }

    fn density_field(
        kind: ResourceKind,
        real_charge: usize,
        normalized_positive_overflow: f64,
    ) -> DensityFieldResult {
        DensityFieldResult {
            kind,
            available_capacity: real_charge,
            real_charge,
            real_area: usize_as_f64(real_charge),
            filler_charge: 0.0,
            density: Vec::new(),
            energy: 0.0,
            normalized_positive_overflow,
            net_charge: 0.0,
            force_l1: 0.0,
            unit_gradients: Vec::new(),
            filler_gradients: Vec::new(),
        }
    }

    #[test]
    fn raw_phi_augmented_objective_and_gradient_have_matching_units() {
        let energy = 12.5;
        let initial_energy = 50.0;
        let multiplier = 0.75;
        let (value, coefficient) =
            augmented_density_value_and_coefficient(energy, initial_energy, multiplier);
        let normalized = energy / initial_energy;
        assert_close(
            value,
            multiplier * (energy + 0.5 * AUGMENTED_DENSITY_BETA * energy * energy / initial_energy),
            1.0e-12,
        );
        assert_close(
            coefficient,
            multiplier * (1.0 + AUGMENTED_DENSITY_BETA * normalized),
            1.0e-12,
        );
    }

    #[test]
    fn overlap_weighted_continuous_rudy_conserves_directional_hpwl() {
        let bounds = [(1.25, 4.75, 2.50, 5.00), (0.0, 0.0, 0.0, 7.0)];
        let demand = continuous_routing_demand(10, 10, &bounds).unwrap();

        assert_close(demand.horizontal.iter().sum(), 3.5, 1.0e-12);
        assert_close(demand.vertical.iter().sum(), 9.5, 1.0e-12);
        assert!(demand.horizontal.iter().all(|entry| *entry >= 0.0));
        assert!(demand.vertical.iter().all(|entry| *entry >= 0.0));
    }

    #[test]
    fn multiplier_growth_matches_elfplace_endpoints_and_initial_step() {
        assert_eq!(MULTIPLIER_ALPHA_LOW.to_bits(), 1.05_f64.to_bits());
        assert_eq!(MULTIPLIER_ALPHA_HIGH.to_bits(), 1.06_f64.to_bits());
        assert_close(MULTIPLIER_ALPHA_HIGH - 1.0, 0.06, 1.0e-16);
        assert_close(
            initial_multiplier_step(&[3.0, 4.0], false).unwrap(),
            0.06,
            1.0e-16,
        );
        assert_close(
            initial_multiplier_step(&[3.0, 4.0], true).unwrap(),
            0.25,
            1.0e-15,
        );
        assert_close(
            area_adjusted_multiplier_step(&[3.0, 4.0]).unwrap(),
            0.30,
            1.0e-15,
        );
        assert_eq!(
            multiplier_growth_from_logarithm(0.0).unwrap().to_bits(),
            MULTIPLIER_ALPHA_LOW.to_bits()
        );
        assert_eq!(
            multiplier_growth_from_logarithm(f64::MAX)
                .unwrap()
                .to_bits(),
            MULTIPLIER_ALPHA_HIGH.to_bits()
        );
        assert_close(
            multiplier_growth_from_logarithm(1.0).unwrap(),
            1.055,
            1.0e-15,
        );
    }

    #[test]
    fn heterogeneous_field_gamma_hits_target_and_unit_overflow_endpoints() {
        for target in [0.10, 0.20] {
            assert_close(
                field_wirelength_gamma(target, target, 8.0).unwrap(),
                0.8,
                1.0e-14,
            );
            assert_close(
                field_wirelength_gamma(1.0, target, 8.0).unwrap(),
                80.0,
                1.0e-12,
            );
        }
    }

    #[test]
    fn global_gamma_is_pin_weighted_field_mean() {
        assert_close(
            weighted_gamma_mean(&[(2.0, 1.0), (10.0, 3.0)], 8.0).unwrap(),
            8.0,
            0.0,
        );
        assert_close(weighted_gamma_mean(&[(80.0, 0.0)], 8.0).unwrap(), 8.0, 0.0);
    }

    #[test]
    fn scarce_memory_overflow_has_an_independent_stop_condition() {
        let lut = density_field(ResourceKind::Lut(4), 100_000, 0.09);
        let memory_overflowing = density_field(ResourceKind::Memory, 1, 0.21);
        assert!(!density_is_converged(&[lut.clone(), memory_overflowing]));
        let memory_converged = density_field(ResourceKind::Memory, 1, 0.20);
        assert!(density_is_converged(&[lut, memory_converged]));
    }

    #[test]
    fn area_adjustment_starts_at_lut_and_register_fifteen_percent() {
        assert_eq!(
            AREA_ADJUSTMENT_OVERFLOW_TARGET.to_bits(),
            0.15_f64.to_bits()
        );
        let lut = density_field(ResourceKind::Lut(4), 100, 0.15);
        let register = density_field(ResourceKind::Register, 100, 0.149);
        let memory = density_field(ResourceKind::Memory, 1, 0.99);
        assert!(area_adjustment_is_ready(&[
            lut.clone(),
            register.clone(),
            memory
        ]));
        let overflowing_lut = density_field(ResourceKind::Lut(4), 100, 0.151);
        assert!(!area_adjustment_is_ready(&[overflowing_lut, register]));
    }

    #[test]
    fn rudy_inflates_exchangeable_slice_area_but_not_hard_memory_charge() {
        let congested = 2.0_f64.sqrt();
        assert_close(
            routability_adjusted_member_area(ResourceKind::Lut(4), 1.0, congested, 0.0).unwrap(),
            2.0,
            f64::EPSILON,
        );
        assert_close(
            routability_adjusted_member_area(ResourceKind::Register, 1.0, 0.0, congested).unwrap(),
            2.0,
            f64::EPSILON,
        );
        assert_close(
            routability_adjusted_member_area(ResourceKind::Memory, 1.0, congested, congested)
                .unwrap(),
            1.0,
            0.0,
        );
    }

    #[test]
    fn area_adjusted_multiplier_matches_elfplace_force_balance() {
        let mut lut = density_field(ResourceKind::Lut(4), 100, 0.10);
        lut.energy = 5.0;
        lut.force_l1 = 2.0;
        let mut register = density_field(ResourceKind::Register, 100, 0.10);
        register.energy = 10.0;
        register.force_l1 = 3.0;
        let evaluation = ContinuousEvaluation {
            wirelength: WeightedAverageObjective {
                value: 0.0,
                gradient_x: vec![3.0, -1.0],
                gradient_y: vec![4.0, 0.0],
            },
            density: DensityResult {
                fields: vec![lut, register],
            },
        };
        let multipliers = area_adjusted_density_multipliers(&evaluation, &[10.0, 20.0]).unwrap();
        assert_close(multipliers[0], multipliers[1], 1.0e-15);
        // Equation (27) guarantees <sum_i q_i ||xi_i||_1, lambda'>
        // equals eta' ||gradient W||_1.
        assert_close(2.0 * multipliers[0] + 3.0 * multipliers[1], 0.8, 1.0e-14);
    }

    #[test]
    fn stationary_rounded_target_must_survive_a_multiplier_update() {
        let target = [Point::new(3, 5)];
        let mut previous = None;
        assert!(!stationary_target_is_fixed(
            &mut previous,
            DynamicNesterovStatus::NumericallyStationary,
            &target,
        ));
        assert!(stationary_target_is_fixed(
            &mut previous,
            DynamicNesterovStatus::NumericallyStationary,
            &target,
        ));
        assert!(!stationary_target_is_fixed(
            &mut previous,
            DynamicNesterovStatus::Accepted,
            &target,
        ));
        assert!(previous.is_none());
    }

    #[test]
    fn paper_iteration_cap_is_a_hard_nonconvergence_boundary() {
        assert!(!eplace_iteration_limit_reached(2_999));
        assert!(eplace_iteration_limit_reached(3_000));
        assert!(eplace_iteration_limit_reached(u128::MAX));
    }

    #[test]
    fn checkpoint_retains_the_lowest_field_overflow_excess() {
        let first = [density_field(ResourceKind::Memory, 136, 0.40)];
        let better = [density_field(ResourceKind::Memory, 136, 0.25)];
        let worse = [density_field(ResourceKind::Memory, 136, 0.30)];
        let mut checkpoint = PlacementCheckpoint::new(vec![Point::new(0, 0)], &first);
        checkpoint.consider(vec![Point::new(1, 1)], &better);
        checkpoint.consider(vec![Point::new(2, 2)], &worse);
        assert_eq!(checkpoint.targets, [Point::new(1, 1)]);
    }

    #[test]
    fn paper_noise_is_deterministic_zero_mean_and_one_pair_per_unit() {
        let movable = [7, 2, 91, 13, 42, 5, 1];
        let first = deterministic_gaussian_offsets(&movable, 90, 70);
        let second = deterministic_gaussian_offsets(&movable, 90, 70);
        assert_eq!(first, second);
        assert_eq!(first.len(), movable.len());
        assert_close(first.iter().map(|(x, _)| x).sum(), 0.0, 1.0e-15);
        assert_close(first.iter().map(|(_, y)| y).sum(), 0.0, 1.0e-15);
        assert!(first.iter().all(|(x, y)| x.is_finite() && y.is_finite()));
    }

    #[test]
    fn origin_noise_clamps_choice_bounds_and_does_not_touch_other_variables() {
        let mut coordinates = vec![1.0, 2.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0];
        let bounds = CoordinateBounds {
            lower: vec![0.0, 0.0, 5.0, 15.0, 25.0, 35.0, 45.0, 55.0],
            upper: vec![2.0, 3.0, 15.0, 25.0, 35.0, 45.0, 55.0, 65.0],
        };
        apply_origin_offsets(
            &mut coordinates,
            &bounds,
            &[0, 2],
            &[(100.0, -100.0), (-100.0, 100.0)],
        );
        assert_eq!(
            coordinates,
            vec![2.0, 0.0, 10.0, 20.0, 25.0, 45.0, 50.0, 60.0]
        );
    }

    #[test]
    fn rigid_lut_register_unit_keeps_resource_specific_cell_pin_weights() {
        let mut device = Device::new("carry", 2, 1).unwrap();
        let lut0 = device
            .add_bel("LUT0", ResourceKind::Lut(4), Point::new(0, 0))
            .unwrap();
        let ff0 = device
            .add_bel("FF0", ResourceKind::Register, Point::new(0, 0))
            .unwrap();
        let lut1 = device
            .add_bel("LUT1", ResourceKind::Lut(4), Point::new(1, 0))
            .unwrap();
        let ff1 = device
            .add_bel("FF1", ResourceKind::Register, Point::new(1, 0))
            .unwrap();
        let unit = PlacementUnit {
            cells: vec![CellId(0), CellId(1)],
            choices: PlacementChoices::Shared(Arc::from([vec![lut0, ff0], vec![lut1, ff1]])),
        };

        let weights = density_gamma_weights(&[unit], &device, &[10.0, 1.0], false);
        assert_eq!(weights[&ResourceKind::Lut(4)].to_bits(), 10.0_f64.to_bits());
        assert_eq!(
            weights[&ResourceKind::Register].to_bits(),
            1.0_f64.to_bits()
        );
    }

    #[test]
    fn only_interchangeable_ecp5_resource_classes_get_density_fields() {
        assert!(density_kind_is_exchangeable(ResourceKind::Lut(4), false));
        assert!(density_kind_is_exchangeable(ResourceKind::Register, false));
        assert!(density_kind_is_exchangeable(ResourceKind::Memory, false));
        assert!(!density_kind_is_exchangeable(ResourceKind::Clock, false));
        assert!(!density_kind_is_exchangeable(ResourceKind::Logic, false));
        assert!(!density_kind_is_exchangeable(ResourceKind::Io, false));
        assert!(!density_kind_is_exchangeable(ResourceKind::Constant, false));
    }

    #[test]
    fn box_muller_uniform_inputs_are_strictly_open_and_repeatable() {
        for index in 0..1_000_u64 {
            let value = open_unit_interval(splitmix64(index));
            assert!(value > 0.0 && value < 1.0);
            assert_eq!(
                value.to_bits(),
                open_unit_interval(splitmix64(index)).to_bits()
            );
        }
    }
}
