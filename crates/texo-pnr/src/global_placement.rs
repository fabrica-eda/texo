//! Box-constrained continuous optimization used by global placement.

/// One differentiable objective evaluation.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct ObjectiveEvaluation {
    pub(super) value: f64,
    pub(super) gradient: Vec<f64>,
}

/// Coordinate bounds for a projected continuous placement.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct CoordinateBounds {
    pub(super) lower: Vec<f64>,
    pub(super) upper: Vec<f64>,
}

/// Result of an exact or requested-inexact monotone Nesterov solve.
#[derive(Clone, Debug, PartialEq)]
#[cfg(test)]
pub(super) struct NesterovSolution {
    pub(super) coordinates: Vec<f64>,
    pub(super) objective: f64,
    pub(super) stationarity: f64,
    pub(super) iterations: u128,
}

/// Momentum and local smoothness estimate retained across a changing
/// placement objective.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct DynamicNesterovState {
    coordinates: Vec<f64>,
    previous_coordinates: Vec<f64>,
    momentum_parameter: f64,
    lipschitz: Option<f64>,
}

impl DynamicNesterovState {
    pub(super) fn new(initial: &[f64], bounds: &CoordinateBounds) -> Self {
        // Construction must not panic even when a caller supplies malformed
        // bounds.  Preserve the input in that case so the first step can
        // return the corresponding typed validation error transactionally.
        let coordinates = if bounds_are_projectable(initial, bounds) {
            project(initial, bounds)
        } else {
            initial.to_vec()
        };
        Self {
            previous_coordinates: coordinates.clone(),
            coordinates,
            momentum_parameter: 1.0,
            lipschitz: None,
        }
    }

    pub(super) fn coordinates(&self) -> &[f64] {
        &self.coordinates
    }
}

/// How a dynamic Nesterov call completed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum DynamicNesterovStatus {
    /// A distinct candidate was accepted for the current objective.
    Accepted,
    /// No representable projected step exists at the current coordinates.
    NumericallyStationary,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct DynamicNesterovStep<T> {
    pub(super) status: DynamicNesterovStatus,
    pub(super) objective: f64,
    pub(super) stationarity: f64,
    pub(super) coordinate_change: f64,
    pub(super) line_search_trials: u128,
    /// Caller data evaluated at the returned state coordinates.
    pub(super) payload: T,
}

/// Failure before the projected gradient reaches numerical stationarity.
#[derive(Clone, Debug, PartialEq)]
pub(super) enum NesterovError<E> {
    InvalidBounds,
    InvalidMetricLength {
        expected: usize,
        actual: usize,
    },
    NonPositiveMetric {
        index: usize,
    },
    #[cfg(test)]
    InvalidStationarityTolerance,
    InvalidInitialCoordinate {
        index: usize,
    },
    InvalidMomentumParameter,
    InvalidLipschitzEstimate,
    InvalidObjective,
    InvalidGradientLength {
        expected: usize,
        actual: usize,
    },
    NonFiniteGradient {
        index: usize,
    },
    LineSearchOverflow,
    IterationOverflow,
    Evaluation(E),
}

/// Takes one accepted Nesterov step for the caller's current objective.
///
/// Placement changes its density multipliers after every accepted primal
/// step.  Keeping momentum and the local Lipschitz estimate here avoids
/// restarting as gradient descent, while monotonicity is checked against the
/// newly evaluated current objective before every acceptance. `metric` is a
/// frozen positive diagonal preconditioner for this complete call; candidate
/// generation, the smoothness majorizer, and stationarity all use the same
/// transformed coordinate system.
#[allow(clippy::too_many_lines)]
pub(super) fn dynamic_nesterov_step<E, T>(
    state: &mut DynamicNesterovState,
    bounds: &CoordinateBounds,
    metric: &[f64],
    mut evaluate: impl FnMut(&[f64]) -> Result<(ObjectiveEvaluation, T), E>,
) -> Result<DynamicNesterovStep<T>, NesterovError<E>> {
    validate_bounds(&state.coordinates, bounds)?;
    validate_bounds(&state.previous_coordinates, bounds)?;
    validate_metric(state.coordinates.len(), metric)?;
    if !state.momentum_parameter.is_finite() || state.momentum_parameter < 1.0 {
        return Err(NesterovError::InvalidMomentumParameter);
    }
    if state
        .lipschitz
        .is_some_and(|lipschitz| !lipschitz.is_finite() || lipschitz <= 0.0)
    {
        return Err(NesterovError::InvalidLipschitzEstimate);
    }

    // Work exclusively on local state until an accepted/stationary return.
    // Thus bounds, objective, gradient, line-search, and evaluator failures
    // leave every bit of the persistent optimizer state unchanged.
    let origin = project(&state.coordinates, bounds);
    let previous_coordinates = project(&state.previous_coordinates, bounds);
    let (origin_evaluation, origin_payload) =
        evaluate(&origin).map_err(NesterovError::Evaluation)?;
    let origin_evaluation = checked_evaluation(&origin, origin_evaluation)?;
    let mut origin_payload = Some(origin_payload);
    let mut restart = false;
    let mut working_lipschitz = state.lipschitz;
    let mut total_line_search_trials = 0_u128;
    'step: loop {
        let momentum = if restart {
            1.0
        } else {
            state.momentum_parameter
        };
        let next_momentum = 1.0_f64.midpoint((1.0 + 4.0 * momentum * momentum).sqrt());
        if !next_momentum.is_finite() {
            return Err(NesterovError::IterationOverflow);
        }
        let extrapolation = (momentum - 1.0) / next_momentum;
        let y = if restart {
            origin.clone()
        } else {
            origin
                .iter()
                .zip(&previous_coordinates)
                .enumerate()
                .map(|(index, (&current, &previous))| {
                    (current + extrapolation * (current - previous))
                        .clamp(bounds.lower[index], bounds.upper[index])
                })
                .collect::<Vec<_>>()
        };
        let y_evaluation = if y == origin {
            origin_evaluation.clone()
        } else {
            let (evaluation, _) = evaluate(&y).map_err(NesterovError::Evaluation)?;
            checked_evaluation(&y, evaluation)?
        };
        let y_stationarity = box_metric_kkt_residual(&y_evaluation.gradient, &y, bounds, metric);
        if !y_stationarity.is_finite() {
            return Err(NesterovError::LineSearchOverflow);
        }
        if y_stationarity == 0.0 {
            if y != origin {
                restart = true;
                continue;
            }
            state.coordinates.clone_from(&origin);
            state.previous_coordinates.clone_from(&origin);
            state.momentum_parameter = 1.0;
            return Ok(DynamicNesterovStep {
                status: DynamicNesterovStatus::NumericallyStationary,
                objective: origin_evaluation.value,
                stationarity: box_metric_kkt_residual(
                    &origin_evaluation.gradient,
                    &origin,
                    bounds,
                    metric,
                ),
                coordinate_change: 0.0,
                line_search_trials: total_line_search_trials,
                payload: origin_payload
                    .take()
                    .expect("the origin payload is returned at most once"),
            });
        }
        let initial_lipschitz = working_lipschitz.unwrap_or(y_stationarity);
        let mut trial_lipschitz = half_nonzero(initial_lipschitz);
        let mut may_reduce_lipschitz_for_resolution = true;
        let (candidate, candidate_evaluation, candidate_payload) = loop {
            total_line_search_trials = total_line_search_trials
                .checked_add(1)
                .ok_or(NesterovError::IterationOverflow)?;
            let mut denominator_underflow = false;
            for &entry in metric {
                let denominator = trial_lipschitz * entry;
                if !denominator.is_finite() {
                    return Err(NesterovError::LineSearchOverflow);
                }
                denominator_underflow |= denominator == 0.0;
            }
            if denominator_underflow {
                trial_lipschitz *= 2.0;
                may_reduce_lipschitz_for_resolution = false;
                if !trial_lipschitz.is_finite() {
                    return Err(NesterovError::LineSearchOverflow);
                }
                continue;
            }
            let candidate = projected_metric_gradient_step(
                &y,
                &y_evaluation.gradient,
                trial_lipschitz,
                metric,
                bounds,
            );
            if candidate == y {
                if may_reduce_lipschitz_for_resolution {
                    let smaller_lipschitz = half_nonzero(trial_lipschitz);
                    if smaller_lipschitz < trial_lipschitz {
                        trial_lipschitz = smaller_lipschitz;
                        continue;
                    }
                }
                if y != origin {
                    restart = true;
                    working_lipschitz = Some(trial_lipschitz);
                    continue 'step;
                }
                state.coordinates.clone_from(&origin);
                state.previous_coordinates.clone_from(&origin);
                state.momentum_parameter = 1.0;
                state.lipschitz = Some(trial_lipschitz);
                return Ok(DynamicNesterovStep {
                    status: DynamicNesterovStatus::NumericallyStationary,
                    objective: origin_evaluation.value,
                    stationarity: box_metric_kkt_residual(
                        &origin_evaluation.gradient,
                        &origin,
                        bounds,
                        metric,
                    ),
                    coordinate_change: 0.0,
                    line_search_trials: total_line_search_trials,
                    payload: origin_payload
                        .take()
                        .expect("the origin payload is returned at most once"),
                });
            }
            let (evaluation, payload) = evaluate(&candidate).map_err(NesterovError::Evaluation)?;
            let evaluation = checked_evaluation(&candidate, evaluation)?;
            let delta = candidate
                .iter()
                .zip(&y)
                .map(|(&candidate, &from)| candidate - from)
                .collect::<Vec<_>>();
            let upper_bound = y_evaluation.value
                + dot(&y_evaluation.gradient, &delta)
                + 0.5 * trial_lipschitz * metric_squared_norm(&delta, metric);
            if !upper_bound.is_finite() {
                return Err(NesterovError::LineSearchOverflow);
            }
            let satisfies_smoothness =
                evaluation.value <= upper_bound + rounding_margin(evaluation.value, upper_bound);
            let satisfies_monotonicity = evaluation.value
                <= origin_evaluation.value
                    + rounding_margin(evaluation.value, origin_evaluation.value);
            if satisfies_smoothness && satisfies_monotonicity {
                break (candidate, evaluation, payload);
            }
            if satisfies_smoothness && !restart {
                // Momentum, rather than local smoothness, made this candidate
                // worse than the current point.  Retry once from the current
                // point without mutating persistent state.
                restart = true;
                working_lipschitz = Some(trial_lipschitz);
                continue 'step;
            }
            // Once restarted, monotonicity is part of line-search acceptance.
            // A pathological evaluator therefore increases L (and eventually
            // reports overflow/stationarity) instead of repeating the same
            // rejected candidate forever.
            trial_lipschitz *= 2.0;
            may_reduce_lipschitz_for_resolution = false;
            if !trial_lipschitz.is_finite() {
                return Err(NesterovError::LineSearchOverflow);
            }
        };
        let coordinate_change = candidate
            .iter()
            .zip(&origin)
            .map(|(&next, &previous)| (next - previous).abs())
            .fold(0.0_f64, f64::max);
        let stationarity =
            box_metric_kkt_residual(&candidate_evaluation.gradient, &candidate, bounds, metric);
        if !stationarity.is_finite() {
            return Err(NesterovError::LineSearchOverflow);
        }
        state.previous_coordinates = origin;
        state.coordinates = candidate;
        state.momentum_parameter = next_momentum;
        state.lipschitz = Some(trial_lipschitz);
        return Ok(DynamicNesterovStep {
            status: DynamicNesterovStatus::Accepted,
            objective: candidate_evaluation.value,
            stationarity,
            coordinate_change,
            line_search_trials: total_line_search_trials,
            payload: candidate_payload,
        });
    }
}

/// Minimizes a smooth objective under independent coordinate bounds.
///
/// Backtracking accepts exactly the quadratic smoothness upper bound. A
/// monotonicity restart removes harmful momentum without a tuned restart
/// threshold. The solve ends only when the projected gradient mapping and the
/// accepted coordinate change reach floating-point resolution; there is no
/// placement-dependent iteration budget.
#[allow(clippy::too_many_lines)]
#[cfg(test)]
pub(super) fn monotone_nesterov<E>(
    initial: &[f64],
    bounds: &CoordinateBounds,
    mut evaluate: impl FnMut(&[f64]) -> Result<ObjectiveEvaluation, E>,
) -> Result<NesterovSolution, NesterovError<E>> {
    monotone_nesterov_until(initial, bounds, &mut evaluate, 0.0)
}

/// Minimizes until exact numerical stationarity or an inexact KKT tolerance.
///
/// The latter lets an augmented-Lagrangian caller supply a summable sequence
/// of inner KKT errors, avoiding a machine-precision solve of every early
/// subproblem while retaining the standard inexact-convergence condition.
#[allow(clippy::too_many_lines)]
#[cfg(test)]
pub(super) fn monotone_nesterov_until<E>(
    initial: &[f64],
    bounds: &CoordinateBounds,
    mut evaluate: impl FnMut(&[f64]) -> Result<ObjectiveEvaluation, E>,
    stationarity_tolerance: f64,
) -> Result<NesterovSolution, NesterovError<E>> {
    validate_bounds(initial, bounds)?;
    if !stationarity_tolerance.is_finite() || stationarity_tolerance < 0.0 {
        return Err(NesterovError::InvalidStationarityTolerance);
    }
    let mut x = project(initial, bounds);
    let mut x_evaluation =
        checked_evaluation(&x, evaluate(&x).map_err(NesterovError::Evaluation)?)?;
    let initial_stationarity = box_kkt_residual(&x_evaluation.gradient, &x, bounds);
    if initial_stationarity <= stationarity_tolerance {
        return Ok(NesterovSolution {
            coordinates: x,
            objective: x_evaluation.value,
            stationarity: initial_stationarity,
            iterations: 0,
        });
    }
    let mut y = x.clone();
    let mut momentum_parameter = 1.0_f64;
    let mut lipschitz = initial_stationarity;
    let mut iterations = 0_u128;
    loop {
        let y_evaluation = if y == x {
            x_evaluation.clone()
        } else {
            checked_evaluation(&y, evaluate(&y).map_err(NesterovError::Evaluation)?)?
        };
        let mut trial_lipschitz = half_nonzero(lipschitz);
        let mut line_search_trials = 0_u128;
        let (candidate, candidate_evaluation, gradient_mapping_norm) = loop {
            line_search_trials = line_search_trials
                .checked_add(1)
                .ok_or(NesterovError::IterationOverflow)?;
            let candidate =
                projected_gradient_step(&y, &y_evaluation.gradient, trial_lipschitz, bounds);
            if candidate == y {
                if y != x {
                    y.clone_from(&x);
                    momentum_parameter = 1.0;
                    break (x.clone(), x_evaluation.clone(), 0.0);
                }
                return Ok(NesterovSolution {
                    stationarity: box_kkt_residual(&x_evaluation.gradient, &x, bounds),
                    coordinates: x,
                    objective: x_evaluation.value,
                    iterations,
                });
            }
            let evaluation = checked_evaluation(
                &candidate,
                evaluate(&candidate).map_err(NesterovError::Evaluation)?,
            )?;
            let delta = candidate
                .iter()
                .zip(&y)
                .map(|(&candidate, &origin)| candidate - origin)
                .collect::<Vec<_>>();
            let linear = dot(&y_evaluation.gradient, &delta);
            let quadratic = 0.5 * trial_lipschitz * dot(&delta, &delta);
            let upper_bound = y_evaluation.value + linear + quadratic;
            if evaluation.value <= upper_bound + rounding_margin(evaluation.value, upper_bound) {
                let mapping_norm = delta
                    .iter()
                    .map(|entry| trial_lipschitz * entry.abs())
                    .fold(0.0_f64, f64::max);
                break (candidate, evaluation, mapping_norm);
            }
            trial_lipschitz *= 2.0;
            if !trial_lipschitz.is_finite() {
                return Err(NesterovError::LineSearchOverflow);
            }
        };

        if candidate == x && gradient_mapping_norm == 0.0 {
            continue;
        }
        if candidate_evaluation.value
            > x_evaluation.value + rounding_margin(candidate_evaluation.value, x_evaluation.value)
        {
            y.clone_from(&x);
            momentum_parameter = 1.0;
            lipschitz = trial_lipschitz;
            continue;
        }

        iterations = iterations
            .checked_add(1)
            .ok_or(NesterovError::IterationOverflow)?;
        let coordinate_change = candidate
            .iter()
            .zip(&x)
            .map(|(&next, &previous)| (next - previous).abs())
            .fold(0.0_f64, f64::max);
        let coordinate_scale = candidate
            .iter()
            .map(|coordinate| coordinate.abs())
            .fold(1.0_f64, f64::max);
        let gradient_scale = y_evaluation
            .gradient
            .iter()
            .map(|gradient| gradient.abs())
            .fold(1.0_f64, f64::max);
        let stationarity = box_kkt_residual(&candidate_evaluation.gradient, &candidate, bounds);
        let resolution = f64::EPSILON.sqrt();
        if std::env::var_os("TEXO_PNR_TRACE_EPLACE").is_some() {
            eprintln!(
                "TEXO_PNR_TRACE nesterov iteration={iterations} objective={:.17e} coordinate_change={coordinate_change:.9e} gradient_mapping={gradient_mapping_norm:.9e} stationarity={stationarity:.9e} gradient_scale={gradient_scale:.9e} lipschitz={trial_lipschitz:.9e} line_search_trials={line_search_trials}",
                candidate_evaluation.value,
            );
        }
        if coordinate_change <= resolution * coordinate_scale
            && gradient_mapping_norm <= resolution * gradient_scale
        {
            return Ok(NesterovSolution {
                coordinates: candidate,
                objective: candidate_evaluation.value,
                stationarity,
                iterations,
            });
        }
        if stationarity <= stationarity_tolerance {
            return Ok(NesterovSolution {
                coordinates: candidate,
                objective: candidate_evaluation.value,
                stationarity,
                iterations,
            });
        }

        let next_momentum =
            1.0_f64.midpoint((1.0 + 4.0 * momentum_parameter * momentum_parameter).sqrt());
        if !next_momentum.is_finite() {
            return Err(NesterovError::IterationOverflow);
        }
        let extrapolation = (momentum_parameter - 1.0) / next_momentum;
        let next_y = candidate
            .iter()
            .zip(&x)
            .enumerate()
            .map(|(index, (&next, &previous))| {
                (next + extrapolation * (next - previous))
                    .clamp(bounds.lower[index], bounds.upper[index])
            })
            .collect::<Vec<_>>();
        x = candidate;
        x_evaluation = candidate_evaluation;
        y = next_y;
        momentum_parameter = next_momentum;
        lipschitz = trial_lipschitz;
    }
}

fn validate_bounds<E>(initial: &[f64], bounds: &CoordinateBounds) -> Result<(), NesterovError<E>> {
    if bounds.lower.len() != initial.len() || bounds.upper.len() != initial.len() {
        return Err(NesterovError::InvalidBounds);
    }
    for (index, ((&coordinate, &lower), &upper)) in initial
        .iter()
        .zip(&bounds.lower)
        .zip(&bounds.upper)
        .enumerate()
    {
        if !lower.is_finite() || !upper.is_finite() || lower > upper {
            return Err(NesterovError::InvalidBounds);
        }
        if !coordinate.is_finite() {
            return Err(NesterovError::InvalidInitialCoordinate { index });
        }
    }
    Ok(())
}

fn validate_metric<E>(expected: usize, metric: &[f64]) -> Result<(), NesterovError<E>> {
    if metric.len() != expected {
        return Err(NesterovError::InvalidMetricLength {
            expected,
            actual: metric.len(),
        });
    }
    if let Some(index) = metric
        .iter()
        .position(|entry| !entry.is_finite() || *entry <= 0.0)
    {
        return Err(NesterovError::NonPositiveMetric { index });
    }
    Ok(())
}

fn bounds_are_projectable(coordinates: &[f64], bounds: &CoordinateBounds) -> bool {
    bounds.lower.len() == coordinates.len()
        && bounds.upper.len() == coordinates.len()
        && coordinates.iter().all(|coordinate| coordinate.is_finite())
        && bounds
            .lower
            .iter()
            .zip(&bounds.upper)
            .all(|(&lower, &upper)| lower.is_finite() && upper.is_finite() && lower <= upper)
}

fn checked_evaluation<E>(
    coordinates: &[f64],
    evaluation: ObjectiveEvaluation,
) -> Result<ObjectiveEvaluation, NesterovError<E>> {
    if !evaluation.value.is_finite() {
        return Err(NesterovError::InvalidObjective);
    }
    if evaluation.gradient.len() != coordinates.len() {
        return Err(NesterovError::InvalidGradientLength {
            expected: coordinates.len(),
            actual: evaluation.gradient.len(),
        });
    }
    if let Some(index) = evaluation
        .gradient
        .iter()
        .position(|gradient| !gradient.is_finite())
    {
        return Err(NesterovError::NonFiniteGradient { index });
    }
    Ok(evaluation)
}

fn project(coordinates: &[f64], bounds: &CoordinateBounds) -> Vec<f64> {
    coordinates
        .iter()
        .enumerate()
        .map(|(index, &coordinate)| coordinate.clamp(bounds.lower[index], bounds.upper[index]))
        .collect()
}

#[cfg(test)]
fn projected_gradient_step(
    coordinates: &[f64],
    gradient: &[f64],
    lipschitz: f64,
    bounds: &CoordinateBounds,
) -> Vec<f64> {
    coordinates
        .iter()
        .zip(gradient)
        .enumerate()
        .map(|(index, (&coordinate, &gradient))| {
            (coordinate - gradient / lipschitz).clamp(bounds.lower[index], bounds.upper[index])
        })
        .collect()
}

fn projected_metric_gradient_step(
    coordinates: &[f64],
    gradient: &[f64],
    lipschitz: f64,
    metric: &[f64],
    bounds: &CoordinateBounds,
) -> Vec<f64> {
    debug_assert_eq!(coordinates.len(), gradient.len());
    debug_assert_eq!(coordinates.len(), metric.len());
    coordinates
        .iter()
        .zip(gradient)
        .zip(metric)
        .enumerate()
        .map(|(index, ((&coordinate, &gradient), &metric))| {
            (coordinate - gradient / (lipschitz * metric))
                .clamp(bounds.lower[index], bounds.upper[index])
        })
        .collect()
}

#[cfg(test)]
pub(super) fn box_kkt_residual(
    gradient: &[f64],
    coordinates: &[f64],
    bounds: &CoordinateBounds,
) -> f64 {
    debug_assert_eq!(gradient.len(), coordinates.len());
    gradient
        .iter()
        .enumerate()
        .map(|(index, &gradient)| kkt_component(index, gradient, coordinates, bounds))
        .fold(0.0_f64, f64::max)
}

fn box_metric_kkt_residual(
    gradient: &[f64],
    coordinates: &[f64],
    bounds: &CoordinateBounds,
    metric: &[f64],
) -> f64 {
    debug_assert_eq!(gradient.len(), coordinates.len());
    debug_assert_eq!(gradient.len(), metric.len());
    gradient
        .iter()
        .enumerate()
        .map(|(index, &gradient)| {
            kkt_component(index, gradient, coordinates, bounds) / metric[index].sqrt()
        })
        .fold(0.0_f64, f64::max)
}

#[cfg(test)]
pub(super) fn box_kkt_l1(gradient: &[f64], coordinates: &[f64], bounds: &CoordinateBounds) -> f64 {
    debug_assert_eq!(gradient.len(), coordinates.len());
    gradient
        .iter()
        .enumerate()
        .map(|(index, &gradient)| kkt_component(index, gradient, coordinates, bounds))
        .sum()
}

fn kkt_component(
    index: usize,
    gradient: f64,
    coordinates: &[f64],
    bounds: &CoordinateBounds,
) -> f64 {
    if bounds.lower[index].total_cmp(&bounds.upper[index]).is_eq()
        || coordinates[index] <= bounds.lower[index] && gradient > 0.0
        || coordinates[index] >= bounds.upper[index] && gradient < 0.0
    {
        0.0
    } else {
        gradient.abs()
    }
}

fn half_nonzero(value: f64) -> f64 {
    let half = value * 0.5;
    if half == 0.0 { value } else { half }
}

fn rounding_margin(left: f64, right: f64) -> f64 {
    f64::EPSILON * (1.0 + left.abs() + right.abs())
}

fn metric_squared_norm(vector: &[f64], metric: &[f64]) -> f64 {
    debug_assert_eq!(vector.len(), metric.len());
    vector
        .iter()
        .zip(metric)
        .map(|(&entry, &metric)| metric * entry * entry)
        .sum()
}

fn dot(left: &[f64], right: &[f64]) -> f64 {
    debug_assert_eq!(left.len(), right.len());
    left.iter().zip(right).map(|(&a, &b)| a * b).sum()
}

#[cfg(test)]
mod tests {
    use super::{
        CoordinateBounds, DynamicNesterovState, DynamicNesterovStatus, NesterovError,
        ObjectiveEvaluation, box_kkt_l1, box_kkt_residual, box_metric_kkt_residual,
        dynamic_nesterov_step, monotone_nesterov, monotone_nesterov_until,
    };

    fn quadratic(coordinates: &[f64]) -> ObjectiveEvaluation {
        let dx = coordinates[0] - 3.0;
        let dy = coordinates[1] + 2.0;
        ObjectiveEvaluation {
            value: 0.5 * dx * dx + 2.0 * dy * dy,
            gradient: vec![dx, 4.0 * dy],
        }
    }

    fn one_dimensional_quadratic(coordinates: &[f64], target: f64) -> ObjectiveEvaluation {
        let delta = coordinates[0] - target;
        ObjectiveEvaluation {
            value: 0.5 * delta * delta,
            gradient: vec![delta],
        }
    }

    fn one_dimensional_bounds() -> CoordinateBounds {
        CoordinateBounds {
            lower: vec![-10.0],
            upper: vec![10.0],
        }
    }

    #[test]
    fn dynamic_step_returns_the_accepted_candidate_payload() {
        let bounds = one_dimensional_bounds();
        let mut state = DynamicNesterovState::new(&[2.0], &bounds);
        let step = dynamic_nesterov_step(&mut state, &bounds, &[1.0], |coordinates| {
            Ok::<_, ()>((
                one_dimensional_quadratic(coordinates, 0.0),
                coordinates[0].to_bits(),
            ))
        })
        .unwrap();

        assert_eq!(step.status, DynamicNesterovStatus::Accepted);
        assert_eq!(state.coordinates(), [0.0]);
        assert_eq!(step.payload, state.coordinates()[0].to_bits());
        assert_eq!(step.objective.to_bits(), 0.0_f64.to_bits());
        assert_eq!(step.stationarity.to_bits(), 0.0_f64.to_bits());
        assert_eq!(step.coordinate_change.to_bits(), 2.0_f64.to_bits());
    }

    #[test]
    fn diagonal_metric_conditions_an_anisotropic_quadratic() {
        let bounds = CoordinateBounds {
            lower: vec![-10.0, -10.0],
            upper: vec![10.0, 10.0],
        };
        let mut state = DynamicNesterovState {
            coordinates: vec![2.0, 2.0],
            previous_coordinates: vec![2.0, 2.0],
            momentum_parameter: 1.0,
            // The first trial halves this to the exact transformed
            // Lipschitz constant of one.
            lipschitz: Some(2.0),
        };
        let step = dynamic_nesterov_step(&mut state, &bounds, &[1.0, 100.0], |coordinates| {
            Ok::<_, ()>((
                ObjectiveEvaluation {
                    value: 0.5 * coordinates[0] * coordinates[0]
                        + 50.0 * coordinates[1] * coordinates[1],
                    gradient: vec![coordinates[0], 100.0 * coordinates[1]],
                },
                coordinates.to_vec(),
            ))
        })
        .unwrap();

        assert_eq!(step.status, DynamicNesterovStatus::Accepted);
        assert_eq!(state.coordinates(), [0.0, 0.0]);
        assert_eq!(step.payload, [0.0, 0.0]);
        assert_eq!(step.line_search_trials, 1);
    }

    #[test]
    fn metric_kkt_residual_scales_active_components_and_keeps_bound_signs() {
        let coordinates = [0.0, 0.0, 2.0, 2.0];
        let gradient = [3.0, -40.0, -50.0, 6.0];
        let bounds = CoordinateBounds {
            lower: vec![0.0; 4],
            upper: vec![2.0; 4],
        };
        let residual =
            box_metric_kkt_residual(&gradient, &coordinates, &bounds, &[9.0, 100.0, 25.0, 4.0]);
        assert_eq!(residual.to_bits(), 4.0_f64.to_bits());
    }

    #[test]
    fn unchanged_candidate_reduces_lipschitz_before_declaring_stationarity() {
        let bounds = one_dimensional_bounds();
        let mut state = DynamicNesterovState {
            coordinates: vec![1.0],
            previous_coordinates: vec![1.0],
            momentum_parameter: 1.0,
            lipschitz: Some(2.0_f64.powi(60)),
        };
        let step = dynamic_nesterov_step(&mut state, &bounds, &[1.0], |coordinates| {
            Ok::<_, ()>((one_dimensional_quadratic(coordinates, 0.0), ()))
        })
        .unwrap();

        assert_eq!(step.status, DynamicNesterovStatus::Accepted);
        assert!(step.coordinate_change > 0.0);
        assert!(step.line_search_trials > 1);
    }

    #[test]
    fn overflowing_metric_step_denominator_is_transactional() {
        let bounds = one_dimensional_bounds();
        let mut state = DynamicNesterovState {
            coordinates: vec![1.0],
            previous_coordinates: vec![1.0],
            momentum_parameter: 1.0,
            lipschitz: Some(f64::MAX),
        };
        let before = state.clone();
        let error = dynamic_nesterov_step(&mut state, &bounds, &[4.0], |coordinates| {
            Ok::<_, ()>((one_dimensional_quadratic(coordinates, 0.0), ()))
        })
        .unwrap_err();

        assert_eq!(error, NesterovError::LineSearchOverflow);
        assert_eq!(state, before);
    }

    #[test]
    fn dynamic_step_re_evaluates_a_changed_objective_and_keeps_momentum() {
        let bounds = one_dimensional_bounds();
        let mut state = DynamicNesterovState::new(&[2.0], &bounds);
        dynamic_nesterov_step(&mut state, &bounds, &[1.0], |coordinates| {
            Ok::<_, ()>((one_dimensional_quadratic(coordinates, 0.0), ()))
        })
        .unwrap();
        let momentum_after_first_step = state.momentum_parameter;
        let old_coordinate = state.coordinates()[0];
        let old_objective = one_dimensional_quadratic(&[old_coordinate], 3.0).value;
        let mut evaluated_coordinates = Vec::new();

        let step = dynamic_nesterov_step(&mut state, &bounds, &[1.0], |coordinates| {
            evaluated_coordinates.push(coordinates.to_vec());
            Ok::<_, ()>((one_dimensional_quadratic(coordinates, 3.0), coordinates[0]))
        })
        .unwrap();

        assert_eq!(evaluated_coordinates[0], [old_coordinate]);
        assert_eq!(step.status, DynamicNesterovStatus::Accepted);
        assert!(step.objective <= old_objective);
        assert_eq!(step.payload.to_bits(), state.coordinates()[0].to_bits());
        assert!(state.momentum_parameter > momentum_after_first_step);
    }

    #[test]
    fn evaluator_failure_leaves_dynamic_state_bit_identical() {
        let bounds = one_dimensional_bounds();
        let mut state = DynamicNesterovState::new(&[2.0], &bounds);
        dynamic_nesterov_step(&mut state, &bounds, &[1.0], |coordinates| {
            Ok::<_, &'static str>((one_dimensional_quadratic(coordinates, 0.0), ()))
        })
        .unwrap();
        let before = state.clone();
        let mut evaluations = 0_u32;

        let error = dynamic_nesterov_step(&mut state, &bounds, &[1.0], |coordinates| {
            evaluations += 1;
            if evaluations == 2 {
                return Err("injected evaluator failure");
            }
            Ok((one_dimensional_quadratic(coordinates, 3.0), ()))
        })
        .unwrap_err();

        assert_eq!(
            error,
            NesterovError::Evaluation("injected evaluator failure")
        );
        assert_eq!(evaluations, 2);
        assert_eq!(state, before);
    }

    #[test]
    fn stationary_status_carries_the_origin_payload_and_a_new_objective_moves() {
        let bounds = one_dimensional_bounds();
        let mut state = DynamicNesterovState::new(&[1.0], &bounds);
        let stationary = dynamic_nesterov_step(&mut state, &bounds, &[1.0], |coordinates| {
            Ok::<_, ()>((
                one_dimensional_quadratic(coordinates, 1.0),
                coordinates[0].to_bits(),
            ))
        })
        .unwrap();

        assert_eq!(
            stationary.status,
            DynamicNesterovStatus::NumericallyStationary
        );
        assert_eq!(stationary.payload, 1.0_f64.to_bits());
        assert_eq!(stationary.line_search_trials, 0);

        let accepted = dynamic_nesterov_step(&mut state, &bounds, &[1.0], |coordinates| {
            Ok::<_, ()>((one_dimensional_quadratic(coordinates, 2.0), ()))
        })
        .unwrap();
        assert_eq!(accepted.status, DynamicNesterovStatus::Accepted);
        assert_eq!(state.coordinates(), [2.0]);
    }

    #[test]
    fn restart_backtracks_instead_of_repeating_a_rejected_candidate() {
        let bounds = CoordinateBounds {
            lower: vec![-20.0],
            upper: vec![20.0],
        };
        let mut state = DynamicNesterovState {
            coordinates: vec![0.0],
            previous_coordinates: vec![-10.0],
            momentum_parameter: 2.0,
            lipschitz: Some(100.0),
        };
        let mut evaluations = 0_u32;

        let step = dynamic_nesterov_step(&mut state, &bounds, &[1.0], |coordinates| {
            evaluations += 1;
            let value = match evaluations {
                1 => 0.0,
                2 => 100.0,
                3 => 99.98,
                4 | 5 => 1.0,
                _ => coordinates[0],
            };
            Ok::<_, ()>((
                ObjectiveEvaluation {
                    value,
                    gradient: vec![1.0],
                },
                evaluations,
            ))
        })
        .unwrap();

        assert_eq!(step.status, DynamicNesterovStatus::Accepted);
        assert_eq!(evaluations, 6);
        assert_eq!(step.payload, evaluations);
        assert_eq!(step.line_search_trials, 4);
        assert_eq!(state.coordinates(), [-0.01]);
    }

    #[test]
    fn malformed_bounds_and_nonfinite_initial_coordinates_are_typed_errors() {
        let malformed_bounds = CoordinateBounds {
            lower: vec![],
            upper: vec![1.0],
        };
        let mut malformed_state = DynamicNesterovState::new(&[0.5], &malformed_bounds);
        let malformed_before = malformed_state.clone();
        let mut evaluations = 0_u32;
        let error = dynamic_nesterov_step(&mut malformed_state, &malformed_bounds, &[1.0], |_| {
            evaluations += 1;
            Ok::<_, ()>((
                ObjectiveEvaluation {
                    value: 0.0,
                    gradient: vec![0.0],
                },
                (),
            ))
        })
        .unwrap_err();
        assert_eq!(error, NesterovError::InvalidBounds);
        assert_eq!(evaluations, 0);
        assert_eq!(malformed_state, malformed_before);

        let bounds = CoordinateBounds {
            lower: vec![0.0],
            upper: vec![1.0],
        };
        let mut nonfinite_state = DynamicNesterovState::new(&[f64::NAN], &bounds);
        let error = dynamic_nesterov_step(&mut nonfinite_state, &bounds, &[1.0], |_| {
            Ok::<_, ()>((
                ObjectiveEvaluation {
                    value: 0.0,
                    gradient: vec![0.0],
                },
                (),
            ))
        })
        .unwrap_err();
        assert_eq!(error, NesterovError::InvalidInitialCoordinate { index: 0 });
        assert!(nonfinite_state.coordinates()[0].is_nan());
    }

    #[test]
    fn invalid_diagonal_metrics_are_rejected_before_evaluation() {
        let bounds = one_dimensional_bounds();
        let mut state = DynamicNesterovState::new(&[1.0], &bounds);
        let before = state.clone();
        let mut evaluations = 0_u32;
        let error = dynamic_nesterov_step(&mut state, &bounds, &[], |_| {
            evaluations += 1;
            Ok::<_, ()>((
                ObjectiveEvaluation {
                    value: 0.0,
                    gradient: vec![0.0],
                },
                (),
            ))
        })
        .unwrap_err();
        assert_eq!(
            error,
            NesterovError::InvalidMetricLength {
                expected: 1,
                actual: 0
            }
        );
        assert_eq!(evaluations, 0);
        assert_eq!(state, before);

        for metric in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let error = dynamic_nesterov_step(&mut state, &bounds, &[metric], |_| {
                evaluations += 1;
                Ok::<_, ()>((
                    ObjectiveEvaluation {
                        value: 0.0,
                        gradient: vec![0.0],
                    },
                    (),
                ))
            })
            .unwrap_err();
            assert_eq!(error, NesterovError::NonPositiveMetric { index: 0 });
            assert_eq!(evaluations, 0);
            assert_eq!(state, before);
        }
    }

    #[test]
    fn invalid_dynamic_evaluation_is_transactional() {
        let bounds = one_dimensional_bounds();
        let mut state = DynamicNesterovState::new(&[2.0], &bounds);
        let before = state.clone();
        let error = dynamic_nesterov_step(&mut state, &bounds, &[1.0], |_| {
            Ok::<_, ()>((
                ObjectiveEvaluation {
                    value: f64::NAN,
                    gradient: vec![0.0],
                },
                (),
            ))
        })
        .unwrap_err();
        assert_eq!(error, NesterovError::InvalidObjective);
        assert_eq!(state, before);
    }

    #[test]
    fn invalid_dynamic_parameters_are_rejected_before_evaluation() {
        let bounds = one_dimensional_bounds();
        let mut state = DynamicNesterovState::new(&[2.0], &bounds);
        state.momentum_parameter = 0.5;
        let before = state.clone();
        let mut evaluations = 0_u32;
        let error = dynamic_nesterov_step(&mut state, &bounds, &[1.0], |_| {
            evaluations += 1;
            Ok::<_, ()>((
                ObjectiveEvaluation {
                    value: 0.0,
                    gradient: vec![0.0],
                },
                (),
            ))
        })
        .unwrap_err();
        assert_eq!(error, NesterovError::InvalidMomentumParameter);
        assert_eq!(evaluations, 0);
        assert_eq!(state, before);

        state.momentum_parameter = 1.0;
        state.lipschitz = Some(f64::INFINITY);
        let before = state.clone();
        let error = dynamic_nesterov_step(&mut state, &bounds, &[1.0], |_| {
            evaluations += 1;
            Ok::<_, ()>((
                ObjectiveEvaluation {
                    value: 0.0,
                    gradient: vec![0.0],
                },
                (),
            ))
        })
        .unwrap_err();
        assert_eq!(error, NesterovError::InvalidLipschitzEstimate);
        assert_eq!(evaluations, 0);
        assert_eq!(state, before);
    }

    #[test]
    fn dynamic_steps_are_bit_deterministic() {
        let bounds = one_dimensional_bounds();
        let mut first_state = DynamicNesterovState::new(&[2.0], &bounds);
        let mut second_state = first_state.clone();
        let first = dynamic_nesterov_step(&mut first_state, &bounds, &[1.0], |coordinates| {
            Ok::<_, ()>((
                one_dimensional_quadratic(coordinates, -3.0),
                coordinates[0].to_bits(),
            ))
        })
        .unwrap();
        let second = dynamic_nesterov_step(&mut second_state, &bounds, &[1.0], |coordinates| {
            Ok::<_, ()>((
                one_dimensional_quadratic(coordinates, -3.0),
                coordinates[0].to_bits(),
            ))
        })
        .unwrap();
        assert_eq!(first, second);
        assert_eq!(first_state, second_state);
    }

    #[test]
    fn converges_to_an_unconstrained_quadratic_minimum() {
        let bounds = CoordinateBounds {
            lower: vec![-10.0, -10.0],
            upper: vec![10.0, 10.0],
        };
        let solution = monotone_nesterov(&[-7.0, 8.0], &bounds, |coordinates| {
            Ok::<_, ()>(quadratic(coordinates))
        })
        .unwrap();
        assert!((solution.coordinates[0] - 3.0).abs() < 1.0e-7);
        assert!((solution.coordinates[1] + 2.0).abs() < 1.0e-7);
        assert!(solution.objective < 1.0e-14);
    }

    #[test]
    fn projected_gradient_stops_at_a_box_boundary() {
        let bounds = CoordinateBounds {
            lower: vec![0.0],
            upper: vec![2.0],
        };
        let solution = monotone_nesterov(&[1.0], &bounds, |coordinates| {
            let delta = coordinates[0] - 7.0;
            Ok::<_, ()>(ObjectiveEvaluation {
                value: 0.5 * delta * delta,
                gradient: vec![delta],
            })
        })
        .unwrap();
        assert_eq!(solution.coordinates, [2.0]);
    }

    #[test]
    fn fixed_coordinates_are_not_moved() {
        let bounds = CoordinateBounds {
            lower: vec![1.25, -10.0],
            upper: vec![1.25, 10.0],
        };
        let solution = monotone_nesterov(&[1.25, 8.0], &bounds, |coordinates| {
            Ok::<_, ()>(quadratic(coordinates))
        })
        .unwrap();
        assert_eq!(solution.coordinates[0].to_bits(), 1.25_f64.to_bits());
        assert!((solution.coordinates[1] + 2.0).abs() < 1.0e-7);
    }

    #[test]
    fn repeat_runs_are_bit_identical() {
        let bounds = CoordinateBounds {
            lower: vec![-10.0, -10.0],
            upper: vec![10.0, 10.0],
        };
        let first = monotone_nesterov(&[-7.0, 8.0], &bounds, |coordinates| {
            Ok::<_, ()>(quadratic(coordinates))
        })
        .unwrap();
        let second = monotone_nesterov(&[-7.0, 8.0], &bounds, |coordinates| {
            Ok::<_, ()>(quadratic(coordinates))
        })
        .unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn inexact_solve_stops_at_the_requested_kkt_residual() {
        let bounds = CoordinateBounds {
            lower: vec![-10.0, -10.0],
            upper: vec![10.0, 10.0],
        };
        let solution = monotone_nesterov_until(
            &[-7.0, 8.0],
            &bounds,
            |coordinates| Ok::<_, ()>(quadratic(coordinates)),
            5.0,
        )
        .unwrap();
        let exact = monotone_nesterov(&[-7.0, 8.0], &bounds, |coordinates| {
            Ok::<_, ()>(quadratic(coordinates))
        })
        .unwrap();
        assert!(solution.stationarity <= 5.0);
        assert!(solution.iterations < exact.iterations);
        assert!(solution.objective > 1.0e-10);
    }

    #[test]
    fn inexact_solve_checks_the_initial_kkt_residual() {
        let bounds = CoordinateBounds {
            lower: vec![-10.0, -10.0],
            upper: vec![10.0, 10.0],
        };
        let mut evaluations = 0_u32;
        let solution = monotone_nesterov_until(
            &[-7.0, 8.0],
            &bounds,
            |coordinates| {
                evaluations += 1;
                Ok::<_, ()>(quadratic(coordinates))
            },
            40.0,
        )
        .unwrap();
        assert_eq!(evaluations, 1);
        assert_eq!(solution.iterations, 0);
        assert_eq!(solution.stationarity.to_bits(), 40.0_f64.to_bits());
    }

    #[test]
    fn box_kkt_norms_ignore_fixed_and_outward_bound_forces() {
        let coordinates = [0.0, 0.0, 2.0, 2.0, 1.0];
        let gradient = [3.0, -4.0, -5.0, 6.0, 100.0];
        let bounds = CoordinateBounds {
            lower: vec![0.0, 0.0, 0.0, 0.0, 1.0],
            upper: vec![2.0, 2.0, 2.0, 2.0, 1.0],
        };
        assert_eq!(
            box_kkt_residual(&gradient, &coordinates, &bounds).to_bits(),
            6.0_f64.to_bits()
        );
        assert_eq!(
            box_kkt_l1(&gradient, &coordinates, &bounds).to_bits(),
            10.0_f64.to_bits()
        );
    }

    #[test]
    fn rejects_a_nonfinite_objective() {
        let bounds = CoordinateBounds {
            lower: vec![0.0],
            upper: vec![1.0],
        };
        let error = monotone_nesterov(&[0.5], &bounds, |_| {
            Ok::<_, ()>(ObjectiveEvaluation {
                value: f64::NAN,
                gradient: vec![0.0],
            })
        })
        .unwrap_err();
        assert_eq!(error, NesterovError::InvalidObjective);
    }
}
