//! Threshold-free hypergraph model used by analytical global placement.

use std::collections::BTreeMap;

#[cfg(test)]
use texo_model::{BelId, Device, Point};
use texo_model::{CellId, CellPinId, Design, NetId, ResourceKind};

/// Retains the historical one-sink analytical edge scale. Scaling every
/// baseline equation uniformly does not change HPWL, but keeping 64 avoids
/// changing the relative strength of the existing center, density, and anchor
/// terms while the star model is replaced.
const PLACEMENT_WEIGHT_SCALE: f64 = 64.0;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct HypergraphPin {
    unit: usize,
    cell: CellId,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct Hypernet {
    fanout: usize,
    pins: Vec<HypergraphPin>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct TimedArc {
    driver: HypergraphPin,
    sink: HypergraphPin,
    extra_weight: u64,
}

/// Failure to turn a complete legal placement into analytical coordinates or
/// an exact integer objective.
#[cfg(test)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum AnalyticalPlacementError {
    /// A logical cell has no physical binding.
    MissingBinding(CellId),
    /// A binding names no BEL in the device.
    UnknownBel(BelId),
    /// A cell-to-unit map names no unit origin.
    UnknownUnit { cell: CellId, unit: usize },
    /// The exact placement objective exceeded its integer representation.
    ObjectiveOverflow,
}

/// Exact integer objective of a legal projected placement.
#[cfg(test)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct AnalyticalObjective {
    /// Baseline HPWL of every retained non-clock net.
    pub(super) hpwl: u128,
    /// Baseline HPWL plus every sink-local timing overlay.
    pub(super) total: u128,
}

/// Smooth wirelength value and its gradient with respect to unit origins.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct WeightedAverageObjective {
    /// Baseline weighted-average hypernet wirelength plus timing overlays.
    pub(super) value: f64,
    /// Derivative with respect to each placement-unit x coordinate.
    pub(super) gradient_x: Vec<f64>,
    /// Derivative with respect to each placement-unit y coordinate.
    pub(super) gradient_y: Vec<f64>,
}

/// Actual legal unit origins and member offsets for one projected placement.
///
/// Offsets are derived from the selected BEL row, not assignment row zero.
/// Generic atomic groups are not required to be translated copies of one
/// another, so relinearization must not reuse a reference-row shape.
#[cfg(test)]
#[derive(Clone, Debug, PartialEq)]
pub(super) struct LegalizedCoordinates {
    pub(super) origins: Vec<Point>,
    pub(super) cell_offsets: Vec<(f64, f64)>,
}

/// Origin of one linearized analytical edge.
#[cfg(test)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum AxisEdgeKind {
    /// Fanout-normalized bound-to-bound HPWL baseline.
    Baseline,
    /// Sink-local timing weight beyond the ordinary baseline weight of one.
    TimingOverlay,
}

/// One weighted pairwise term in an axis-specific quadratic system.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct AxisEdge {
    pub(super) left: usize,
    pub(super) right: usize,
    pub(super) left_cell: CellId,
    pub(super) right_cell: CellId,
    pub(super) weight: f64,
    #[cfg(test)]
    pub(super) kind: AxisEdgeKind,
}

/// Logical non-clock hypernets and exact sink-local timing overlays.
///
/// The baseline never drops a net because of fanout. Timing is represented by
/// a separate driver-to-sink term so one critical sink cannot strengthen the
/// unrelated sinks selected as the current HPWL bounds.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(super) struct AnalyticalHypergraph {
    nets: Vec<Hypernet>,
    timed_arcs: Vec<TimedArc>,
}

impl AnalyticalHypergraph {
    pub(super) fn new(
        design: &Design,
        unit_by_cell: &[usize],
        _unit_count: usize,
        sink_weights: &BTreeMap<(NetId, CellPinId), u64>,
    ) -> Self {
        let mut nets = Vec::new();
        let mut timed_arcs = Vec::new();
        for (net_index, net) in design.nets().iter().enumerate() {
            if net.sinks.is_empty() {
                continue;
            }
            let driver_cell = design.pins()[net.driver.0].cell;
            if design.cells()[driver_cell.0].kind == ResourceKind::Clock {
                continue;
            }
            let driver = HypergraphPin {
                unit: unit_by_cell[driver_cell.0],
                cell: driver_cell,
            };
            let mut pins = Vec::with_capacity(net.sinks.len() + 1);
            pins.push(driver);
            for &sink_pin in &net.sinks {
                let sink_cell = design.pins()[sink_pin.0].cell;
                let sink = HypergraphPin {
                    unit: unit_by_cell[sink_cell.0],
                    cell: sink_cell,
                };
                pins.push(sink);
                if let Some(extra_weight) = sink_weights
                    .get(&(NetId(net_index), sink_pin))
                    .copied()
                    .unwrap_or(1)
                    .checked_sub(1)
                    .filter(|&weight| weight != 0)
                {
                    timed_arcs.push(TimedArc {
                        driver,
                        sink,
                        extra_weight,
                    });
                }
            }
            // Keep same-unit nets for the exact discrete objective. They add
            // no continuous degree of freedom, but generic atomic assignment
            // rows may have different member geometry.
            nets.push(Hypernet {
                fanout: net.sinks.len(),
                pins,
            });
        }
        Self { nets, timed_arcs }
    }

    /// Diagonal wirelength-Hessian approximation for nonlinear placement.
    ///
    /// ePlace uses the incident-net degree of each movable object in place of
    /// the expensive exact smooth-wirelength Hessian.  Atomic placement units
    /// may contain several logical cells, so a baseline hypernet contributes
    /// once to each distinct unit it can actually move.  Sink-local timing
    /// overlays retain their exact excess weight.
    pub(super) fn wirelength_preconditioner(&self, unit_count: usize) -> Vec<f64> {
        let mut diagonal = self.baseline_wirelength_preconditioner(unit_count);
        for arc in &self.timed_arcs {
            if arc.driver.unit == arc.sink.unit {
                continue;
            }
            let weight = u64_to_f64(arc.extra_weight);
            diagonal[arc.driver.unit] += weight;
            diagonal[arc.sink.unit] += weight;
        }
        diagonal
    }

    /// Baseline movable hypernet incidence used by elfPlace's heterogeneous
    /// gamma aggregation.
    ///
    /// Timing overlays belong to the optimization preconditioner but are not
    /// additional physical pins. Including their criticality weights here
    /// would let timing analysis distort the resource-field smoothing mean.
    pub(super) fn baseline_wirelength_preconditioner(&self, unit_count: usize) -> Vec<f64> {
        let mut diagonal = vec![0.0; unit_count];
        for net in &self.nets {
            let mut units = net.pins.iter().map(|pin| pin.unit).collect::<Vec<_>>();
            units.sort_unstable();
            units.dedup();
            if units.len() < 2 {
                continue;
            }
            let weight = baseline_net_preconditioner_weight(net.pins.len());
            for unit in units {
                diagonal[unit] += weight;
            }
        }
        diagonal
    }

    /// Baseline net-incidence count for each logical cell.
    ///
    /// This is the resource-specific node weight used by heterogeneous
    /// elfPlace gamma aggregation. Distinct logical cells in one rigid macro
    /// retain their own resource kind and pin incidence even though all of
    /// their physical forces act on one placement-unit origin.
    pub(super) fn baseline_cell_incidence_weights(&self, cell_count: usize) -> Vec<f64> {
        let mut weights = vec![0.0; cell_count];
        for net in &self.nets {
            let mut cells = net.pins.iter().map(|pin| pin.cell).collect::<Vec<_>>();
            cells.sort_unstable_by_key(|cell| cell.0);
            cells.dedup();
            let weight = baseline_net_preconditioner_weight(net.pins.len());
            for cell in cells {
                weights[cell.0] += weight;
            }
        }
        weights
    }

    /// Continuous bounding boxes of the ordinary non-clock hypernets.
    ///
    /// This reuses the exact fixed-cell and rigid-macro coordinate model of
    /// the wirelength objective so routability estimation cannot split macro
    /// members or silently move fixed endpoints.
    pub(super) fn external_baseline_net_bounding_boxes(
        &self,
        positions: &[(f64, f64)],
        fixed_coordinates: &[Option<(f64, f64)>],
        offsets: &[(f64, f64)],
    ) -> Vec<(f64, f64, f64, f64)> {
        let coordinate = |pin: HypergraphPin| {
            fixed_coordinates[pin.cell.0].unwrap_or_else(|| {
                let origin = positions[pin.unit];
                let offset = offsets[pin.cell.0];
                (origin.0 + offset.0, origin.1 + offset.1)
            })
        };
        self.nets
            .iter()
            .filter_map(|net| {
                // A net wholly contained in one rigid PlacementUnit is not a
                // general-routing demand. In particular, CCU carry/member
                // connections may have nonzero immutable offsets while using
                // dedicated intra-macro wires; their geometrical bbox must
                // not become RUDY merely because the exact legal objective
                // retains assignment-row geometry.
                let first_unit = net.pins.first()?.unit;
                if net.pins.iter().all(|pin| pin.unit == first_unit) {
                    return None;
                }
                let mut pins = net.pins.iter().copied().map(coordinate);
                let first = pins.next()?;
                let mut bounds = (first.0, first.0, first.1, first.1);
                for (x, y) in pins {
                    bounds.0 = bounds.0.min(x);
                    bounds.1 = bounds.1.max(x);
                    bounds.2 = bounds.2.min(y);
                    bounds.3 = bounds.3.max(y);
                }
                Some(bounds)
            })
            .collect()
    }

    /// Linearizes bound-to-bound HPWL and exact timing overlays for one axis.
    ///
    /// `positions` contains placement-unit origins. Fixed cells use their
    /// exact coordinates instead; movable macro members add their stable
    /// offsets to the unit origin.
    pub(super) fn linearize_axis(
        &self,
        positions: &[f64],
        fixed_coordinates: &[Option<f64>],
        offsets: &[f64],
    ) -> Vec<AxisEdge> {
        let coordinate = |pin: HypergraphPin| {
            fixed_coordinates[pin.cell.0]
                .unwrap_or_else(|| positions[pin.unit] + offsets[pin.cell.0])
        };
        let mut edges = Vec::new();
        for net in &self.nets {
            let pin_positions = net
                .pins
                .iter()
                .copied()
                .map(&coordinate)
                .collect::<Vec<_>>();
            let Some((lower, upper)) = axis_bounds(&net.pins, &pin_positions) else {
                continue;
            };
            for pin_index in 0..net.pins.len() {
                for bound_index in [lower, upper] {
                    if pin_index == bound_index {
                        continue;
                    }
                    // The lower/upper pair is visited from both endpoints;
                    // every other pin-bound pair is unique.
                    if pin_index == upper && bound_index == lower {
                        continue;
                    }
                    let left = net.pins[pin_index];
                    let right = net.pins[bound_index];
                    if left.unit == right.unit {
                        continue;
                    }
                    let separation = (pin_positions[pin_index] - pin_positions[bound_index])
                        .abs()
                        .max(1.0);
                    let fanout =
                        u32::try_from(net.fanout.max(1)).expect("analytical net fanout fits u32");
                    edges.push(AxisEdge {
                        left: left.unit,
                        right: right.unit,
                        left_cell: left.cell,
                        right_cell: right.cell,
                        weight: PLACEMENT_WEIGHT_SCALE / (f64::from(fanout) * separation),
                        #[cfg(test)]
                        kind: AxisEdgeKind::Baseline,
                    });
                }
            }
        }
        for arc in &self.timed_arcs {
            if arc.driver.unit == arc.sink.unit {
                continue;
            }
            let separation = (coordinate(arc.driver) - coordinate(arc.sink))
                .abs()
                .max(1.0);
            edges.push(AxisEdge {
                left: arc.driver.unit,
                right: arc.sink.unit,
                left_cell: arc.driver.cell,
                right_cell: arc.sink.cell,
                // The fanout-independent extra term is intentional: adding
                // unrelated users must not dilute an exact critical sink.
                weight: PLACEMENT_WEIGHT_SCALE * u64_to_f64(arc.extra_weight) / separation,
                #[cfg(test)]
                kind: AxisEdgeKind::TimingOverlay,
            });
        }
        edges
    }

    /// Evaluates weighted-average wirelength with one placement-wide
    /// smoothing parameter.
    ///
    /// ePlace adapts one global `gamma` from density overflow after every
    /// nonlinear iteration.  In particular, it does not sharpen high-fanout
    /// nets independently: every net and timing overlay must describe the
    /// same smooth objective during one optimizer step.
    #[must_use]
    pub(super) fn weighted_average_objective_with_global_gamma(
        &self,
        unit_positions: &[(f64, f64)],
        fixed_coordinates: &[Option<(f64, f64)>],
        cell_offsets: &[(f64, f64)],
        gamma: f64,
    ) -> WeightedAverageObjective {
        assert!(gamma.is_finite() && gamma > 0.0);
        let (mut baseline, timing) = self.weighted_average_components_with_global_gamma(
            unit_positions,
            fixed_coordinates,
            cell_offsets,
            gamma,
        );
        baseline.value += timing.value;
        for (target, contribution) in baseline.gradient_x.iter_mut().zip(timing.gradient_x) {
            *target += contribution;
        }
        for (target, contribution) in baseline.gradient_y.iter_mut().zip(timing.gradient_y) {
            *target += contribution;
        }
        baseline
    }

    pub(super) fn weighted_average_components_with_global_gamma(
        &self,
        unit_positions: &[(f64, f64)],
        fixed_coordinates: &[Option<(f64, f64)>],
        cell_offsets: &[(f64, f64)],
        gamma: f64,
    ) -> (WeightedAverageObjective, WeightedAverageObjective) {
        assert!(gamma.is_finite() && gamma > 0.0);
        // Every axis and the baseline/timing overlays write disjoint gradient
        // vectors. Evaluate those four fixed-order reductions concurrently;
        // this preserves deterministic accumulation within each vector while
        // avoiding a serial pass over all hypernets on every Nesterov trial.
        let ((baseline_x, baseline_y), (timing_x, timing_y)) = rayon::join(
            || {
                rayon::join(
                    || {
                        weighted_average_hypernets_axis(
                            &self.nets,
                            gamma,
                            unit_positions,
                            fixed_coordinates,
                            cell_offsets,
                            false,
                        )
                    },
                    || {
                        weighted_average_hypernets_axis(
                            &self.nets,
                            gamma,
                            unit_positions,
                            fixed_coordinates,
                            cell_offsets,
                            true,
                        )
                    },
                )
            },
            || {
                rayon::join(
                    || {
                        weighted_average_timed_arcs_axis(
                            &self.timed_arcs,
                            gamma,
                            unit_positions,
                            fixed_coordinates,
                            cell_offsets,
                            false,
                        )
                    },
                    || {
                        weighted_average_timed_arcs_axis(
                            &self.timed_arcs,
                            gamma,
                            unit_positions,
                            fixed_coordinates,
                            cell_offsets,
                            true,
                        )
                    },
                )
            },
        );
        let baseline = WeightedAverageObjective {
            value: baseline_x.0 + baseline_y.0,
            gradient_x: baseline_x.1,
            gradient_y: baseline_y.1,
        };
        let timing = WeightedAverageObjective {
            value: timing_x.0 + timing_y.0,
            gradient_x: timing_x.1,
            gradient_y: timing_y.1,
        };
        (baseline, timing)
    }

    /// Computes the exact discrete objective represented by this model.
    ///
    /// The common analytical scale of 64 cancels when comparing placements.
    /// The remaining integer is baseline non-clock-net HPWL plus every
    /// sink-local excess weight times its exact driver-to-sink Manhattan
    /// distance. Cell coordinates come from their selected BELs, including
    /// non-origin members of atomic groups.
    #[cfg(test)]
    pub(super) fn exact_objective(
        &self,
        device: &Device,
        bindings: &[Option<BelId>],
    ) -> Result<AnalyticalObjective, AnalyticalPlacementError> {
        let mut hpwl_objective = 0_u128;
        for net in &self.nets {
            let mut minimum = Point::new(u32::MAX, u32::MAX);
            let mut maximum = Point::new(0, 0);
            for &pin in &net.pins {
                let point = placed_cell_point(device, bindings, pin.cell)?;
                minimum.x = minimum.x.min(point.x);
                minimum.y = minimum.y.min(point.y);
                maximum.x = maximum.x.max(point.x);
                maximum.y = maximum.y.max(point.y);
            }
            let hpwl = u128::from(maximum.x - minimum.x)
                .checked_add(u128::from(maximum.y - minimum.y))
                .ok_or(AnalyticalPlacementError::ObjectiveOverflow)?;
            hpwl_objective = hpwl_objective
                .checked_add(hpwl)
                .ok_or(AnalyticalPlacementError::ObjectiveOverflow)?;
        }
        let mut objective = hpwl_objective;
        for arc in &self.timed_arcs {
            let driver = placed_cell_point(device, bindings, arc.driver.cell)?;
            let sink = placed_cell_point(device, bindings, arc.sink.cell)?;
            let weighted_distance = u128::from(arc.extra_weight)
                .checked_mul(u128::from(driver.manhattan(sink)))
                .ok_or(AnalyticalPlacementError::ObjectiveOverflow)?;
            objective = objective
                .checked_add(weighted_distance)
                .ok_or(AnalyticalPlacementError::ObjectiveOverflow)?;
        }
        Ok(AnalyticalObjective {
            hpwl: hpwl_objective,
            total: objective,
        })
    }
}

fn baseline_net_preconditioner_weight(pin_count: usize) -> f64 {
    assert!(pin_count >= 2, "analytical net needs a driver and sink");
    let fanout = u32::try_from(pin_count - 1).expect("analytical net fanout fits u32");
    1.0 / f64::from(fanout)
}

fn weighted_average_hypernets_axis(
    nets: &[Hypernet],
    gamma: f64,
    unit_positions: &[(f64, f64)],
    fixed_coordinates: &[Option<(f64, f64)>],
    cell_offsets: &[(f64, f64)],
    y_axis: bool,
) -> (f64, Vec<f64>) {
    let mut value = 0.0;
    let mut gradient = vec![0.0; unit_positions.len()];
    let mut scratch = WeightedAverageAxisScratch::default();
    for net in nets {
        value += add_weighted_average_axis(
            &net.pins,
            1.0,
            gamma,
            unit_positions,
            fixed_coordinates,
            cell_offsets,
            y_axis,
            &mut gradient,
            &mut scratch,
        );
    }
    (value, gradient)
}

fn weighted_average_timed_arcs_axis(
    arcs: &[TimedArc],
    gamma: f64,
    unit_positions: &[(f64, f64)],
    fixed_coordinates: &[Option<(f64, f64)>],
    cell_offsets: &[(f64, f64)],
    y_axis: bool,
) -> (f64, Vec<f64>) {
    let mut value = 0.0;
    let mut gradient = vec![0.0; unit_positions.len()];
    let mut scratch = WeightedAverageAxisScratch::default();
    for arc in arcs {
        let pins = [arc.driver, arc.sink];
        value += add_weighted_average_axis(
            &pins,
            u64_to_f64(arc.extra_weight),
            gamma,
            unit_positions,
            fixed_coordinates,
            cell_offsets,
            y_axis,
            &mut gradient,
            &mut scratch,
        );
    }
    (value, gradient)
}

#[allow(clippy::too_many_arguments)]
fn add_weighted_average_axis(
    pins: &[HypergraphPin],
    weight: f64,
    gamma: f64,
    unit_positions: &[(f64, f64)],
    fixed_coordinates: &[Option<(f64, f64)>],
    cell_offsets: &[(f64, f64)],
    y_axis: bool,
    unit_gradient: &mut [f64],
    scratch: &mut WeightedAverageAxisScratch,
) -> f64 {
    debug_assert!(weight.is_finite() && weight >= 0.0);
    debug_assert_eq!(unit_positions.len(), unit_gradient.len());
    let coordinate = |index: usize| {
        let pin = pins[index];
        let (x, y) = fixed_coordinates[pin.cell.0].unwrap_or_else(|| {
            let origin = unit_positions[pin.unit];
            let offset = cell_offsets[pin.cell.0];
            (origin.0 + offset.0, origin.1 + offset.1)
        });
        if y_axis { y } else { x }
    };
    let value = weighted_average_axis(
        pins.len(),
        gamma,
        coordinate,
        |index, derivative| {
            let pin = pins[index];
            if fixed_coordinates[pin.cell.0].is_none() {
                unit_gradient[pin.unit] += weight * derivative;
            }
        },
        scratch,
    );
    weight * value
}

#[derive(Default)]
struct WeightedAverageAxisScratch {
    coordinates: Vec<f64>,
    positive_weights: Vec<f64>,
    negative_weights: Vec<f64>,
}

impl WeightedAverageAxisScratch {
    fn ensure_len(&mut self, pin_count: usize) {
        if self.coordinates.len() < pin_count {
            self.coordinates.resize(pin_count, 0.0);
            self.positive_weights.resize(pin_count, 0.0);
            self.negative_weights.resize(pin_count, 0.0);
        }
    }
}

/// Evaluates a numerically stable one-axis weighted-average span.
///
/// Subtracting the hard maximum/minimum before exponentiation prevents
/// overflow without changing either soft distribution. Means are accumulated
/// as deltas from those extrema, which also preserves translation invariance
/// for coordinates far from zero.
fn weighted_average_axis(
    pin_count: usize,
    gamma: f64,
    coordinate: impl Fn(usize) -> f64,
    mut add_gradient: impl FnMut(usize, f64),
    scratch: &mut WeightedAverageAxisScratch,
) -> f64 {
    debug_assert!(pin_count >= 2);
    debug_assert!(gamma.is_finite() && gamma > 0.0);
    scratch.ensure_len(pin_count);
    let first = coordinate(0);
    scratch.coordinates[0] = first;
    debug_assert!(first.is_finite());
    let (mut minimum, mut maximum) = (first, first);
    for index in 1..pin_count {
        let value = coordinate(index);
        scratch.coordinates[index] = value;
        debug_assert!(value.is_finite());
        minimum = minimum.min(value);
        maximum = maximum.max(value);
    }

    let mut positive_weight_sum = 0.0;
    let mut positive_delta_sum = 0.0;
    let mut negative_weight_sum = 0.0;
    let mut negative_delta_sum = 0.0;
    for index in 0..pin_count {
        let value = scratch.coordinates[index];
        let maximum_delta = value - maximum;
        let minimum_delta = value - minimum;
        // Every term has at least one exact maximum and minimum. Their
        // exponent is exactly zero, so bypass the comparatively expensive
        // transcendental call without changing the resulting bit pattern.
        let positive_weight = if maximum_delta == 0.0 {
            1.0
        } else {
            (maximum_delta / gamma).exp()
        };
        let negative_weight = if minimum_delta == 0.0 {
            1.0
        } else {
            (-minimum_delta / gamma).exp()
        };
        scratch.positive_weights[index] = positive_weight;
        scratch.negative_weights[index] = negative_weight;
        positive_weight_sum += positive_weight;
        positive_delta_sum += positive_weight * maximum_delta;
        negative_weight_sum += negative_weight;
        negative_delta_sum += negative_weight * minimum_delta;
    }
    debug_assert!(positive_weight_sum >= 1.0 && negative_weight_sum >= 1.0);
    let positive_mean_delta = positive_delta_sum / positive_weight_sum;
    let negative_mean_delta = negative_delta_sum / negative_weight_sum;

    for index in 0..pin_count {
        let value = scratch.coordinates[index];
        let maximum_delta = value - maximum;
        let minimum_delta = value - minimum;
        let positive_probability = scratch.positive_weights[index] / positive_weight_sum;
        let negative_probability = scratch.negative_weights[index] / negative_weight_sum;
        let derivative = positive_probability
            * (1.0 + (maximum_delta - positive_mean_delta) / gamma)
            - negative_probability * (1.0 - (minimum_delta - negative_mean_delta) / gamma);
        add_gradient(index, derivative);
    }

    (maximum - minimum) + positive_mean_delta - negative_mean_delta
}

fn u64_to_f64(value: u64) -> f64 {
    let high = u32::try_from(value >> 32).expect("upper half of u64 fits u32");
    let low = u32::try_from(value & u64::from(u32::MAX)).expect("lower half of u64 fits u32");
    f64::from(high) * 4_294_967_296.0 + f64::from(low)
}

#[cfg(test)]
fn placed_cell_point(
    device: &Device,
    bindings: &[Option<BelId>],
    cell: CellId,
) -> Result<Point, AnalyticalPlacementError> {
    let bel = bindings
        .get(cell.0)
        .copied()
        .flatten()
        .ok_or(AnalyticalPlacementError::MissingBinding(cell))?;
    device
        .bels()
        .get(bel.0)
        .map(|physical| physical.point)
        .ok_or(AnalyticalPlacementError::UnknownBel(bel))
}

/// Extracts the exact shape selected for every atomic placement unit.
#[cfg(test)]
pub(super) fn legalized_coordinates(
    device: &Device,
    bindings: &[Option<BelId>],
    unit_by_cell: &[usize],
    origin_cells: &[CellId],
) -> Result<LegalizedCoordinates, AnalyticalPlacementError> {
    let origins = origin_cells
        .iter()
        .map(|&cell| placed_cell_point(device, bindings, cell))
        .collect::<Result<Vec<_>, _>>()?;
    let mut cell_offsets = Vec::with_capacity(unit_by_cell.len());
    for (cell_index, &unit) in unit_by_cell.iter().enumerate() {
        let cell = CellId(cell_index);
        let point = placed_cell_point(device, bindings, cell)?;
        let origin = origins
            .get(unit)
            .copied()
            .ok_or(AnalyticalPlacementError::UnknownUnit { cell, unit })?;
        cell_offsets.push((
            f64::from(point.x) - f64::from(origin.x),
            f64::from(point.y) - f64::from(origin.y),
        ));
    }
    Ok(LegalizedCoordinates {
        origins,
        cell_offsets,
    })
}

/// Returns one dyadic line-search target from legal origins toward an MM solve.
///
/// For legal origins `z`, the new analytical solution `a`, and step `alpha`,
/// the target is `z + alpha * (a - z)`. Fixed units remain exact.
#[cfg(test)]
pub(super) fn projected_mm_targets(
    next: (&[f64], &[f64]),
    legal: &LegalizedCoordinates,
    fixed: &[Option<Point>],
    alpha: f64,
) -> Vec<(f64, f64)> {
    let (next_x, next_y) = next;
    let length = legal.origins.len();
    assert_eq!(next_x.len(), length);
    assert_eq!(next_y.len(), length);
    assert_eq!(fixed.len(), length);
    debug_assert!(alpha.is_finite() && alpha > 0.0 && alpha <= 1.0);
    (0..length)
        .map(|index| {
            if let Some(point) = fixed[index] {
                return (f64::from(point.x), f64::from(point.y));
            }
            let origin = legal.origins[index];
            (
                f64::from(origin.x) + alpha * (next_x[index] - f64::from(origin.x)),
                f64::from(origin.y) + alpha * (next_y[index] - f64::from(origin.y)),
            )
        })
        .collect()
}

/// Returns stable, distinct-unit bounds when the axis is initially collapsed.
/// A net entirely inside one atomic unit has no movable HPWL on this axis.
fn axis_bounds(pins: &[HypergraphPin], positions: &[f64]) -> Option<(usize, usize)> {
    let first = pins.first()?;
    if pins.iter().all(|pin| pin.unit == first.unit) {
        return None;
    }
    let lower = (0..pins.len()).min_by(|&left, &right| {
        positions[left]
            .total_cmp(&positions[right])
            .then_with(|| left.cmp(&right))
    })?;
    let mut upper = (0..pins.len()).max_by(|&left, &right| {
        positions[left]
            .total_cmp(&positions[right])
            .then_with(|| left.cmp(&right))
    })?;
    if positions[lower].total_cmp(&positions[upper]).is_eq() && pins[lower].unit == pins[upper].unit
    {
        upper = (0..pins.len())
            .rev()
            .find(|&index| pins[index].unit != pins[lower].unit)?;
    }
    Some((lower, upper))
}

#[cfg(test)]
mod tests {
    use super::{
        AnalyticalHypergraph, AxisEdgeKind, LegalizedCoordinates,
        baseline_net_preconditioner_weight, legalized_coordinates, projected_mm_targets,
    };
    use std::collections::BTreeMap;
    use texo_model::{Design, Device, NetId, PinDirection, Point, ResourceKind};

    const TEST_GLOBAL_GAMMA: f64 = 0.25;

    fn wide_net(fanout: usize, critical_weight: Option<u64>) -> (AnalyticalHypergraph, Vec<f64>) {
        let mut design = Design::new();
        let source = design.add_cell("source", ResourceKind::Logic);
        let output = design.add_pin(source, "out", PinDirection::Output).unwrap();
        let mut sinks = Vec::new();
        for index in 0..fanout {
            let cell = design.add_cell(format!("sink{index}"), ResourceKind::Logic);
            sinks.push(design.add_pin(cell, "in", PinDirection::Input).unwrap());
        }
        design
            .add_net("wide", output, sinks.iter().copied())
            .unwrap();
        let unit_by_cell = (0..design.cells().len()).collect::<Vec<_>>();
        let weights = critical_weight
            .map(|weight| BTreeMap::from([((texo_model::NetId(0), sinks[0]), weight)]))
            .unwrap_or_default();
        let graph =
            AnalyticalHypergraph::new(&design, &unit_by_cell, design.cells().len(), &weights);
        (graph, vec![0.0; design.cells().len()])
    }

    fn assert_close(actual: f64, expected: f64, relative_tolerance: f64) {
        let scale = expected.abs().max(1.0);
        assert!(
            (actual - expected).abs() <= relative_tolerance * scale,
            "expected {expected:.16e}, got {actual:.16e}"
        );
    }

    #[test]
    fn wirelength_preconditioner_counts_movable_nets_and_timing_weight() {
        let (graph, positions) = wide_net(3, Some(7));

        let baseline = graph.baseline_wirelength_preconditioner(positions.len());
        for weight in baseline {
            assert_close(weight, 1.0 / 3.0, 0.0);
        }
        let preconditioner = graph.wirelength_preconditioner(positions.len());
        assert_close(preconditioner[0], 6.0 + 1.0 / 3.0, 0.0);
        assert_close(preconditioner[1], 6.0 + 1.0 / 3.0, 0.0);
        assert_close(preconditioner[2], 1.0 / 3.0, 0.0);
        assert_close(preconditioner[3], 1.0 / 3.0, 0.0);
    }

    #[test]
    fn baseline_preconditioner_is_inverse_fanout() {
        assert_eq!(
            baseline_net_preconditioner_weight(2).to_bits(),
            1.0_f64.to_bits()
        );
        assert_eq!(
            baseline_net_preconditioner_weight(5).to_bits(),
            0.25_f64.to_bits()
        );
    }

    #[test]
    fn weighted_average_two_pin_value_matches_closed_form() {
        let (graph, _) = wide_net(1, None);
        let positions = [(0.0, 1.0), (4.0, -2.0)];
        let objective = graph.weighted_average_objective_with_global_gamma(
            &positions,
            &[None, None],
            &[(0.0, 0.0), (0.0, 0.0)],
            TEST_GLOBAL_GAMMA,
        );
        let gamma = TEST_GLOBAL_GAMMA;
        let expected = 4.0 * (4.0 / (2.0 * gamma)).tanh() + 3.0 * (3.0 / (2.0 * gamma)).tanh();

        assert_close(objective.value, expected, 1.0e-14);
        assert_close(objective.gradient_x.iter().sum(), 0.0, 1.0e-14);
        assert_close(objective.gradient_y.iter().sum(), 0.0, 1.0e-14);
    }

    #[test]
    fn weighted_average_multi_pin_objective_is_translation_invariant_and_deterministic() {
        let (graph, _) = wide_net(4, None);
        let positions = [
            (-3.0, 7.0),
            (11.0, -2.0),
            (1.0, 5.0),
            (4.0, 15.0),
            (8.0, 0.0),
        ];
        let fixed = vec![None; positions.len()];
        let offsets = vec![(0.0, 0.0); positions.len()];
        let first = graph.weighted_average_objective_with_global_gamma(
            &positions,
            &fixed,
            &offsets,
            TEST_GLOBAL_GAMMA,
        );
        let repeated = graph.weighted_average_objective_with_global_gamma(
            &positions,
            &fixed,
            &offsets,
            TEST_GLOBAL_GAMMA,
        );
        let translated = positions.map(|(x, y)| (x + 1_000_000.0, y - 2_000_000.0));
        let shifted = graph.weighted_average_objective_with_global_gamma(
            &translated,
            &fixed,
            &offsets,
            TEST_GLOBAL_GAMMA,
        );

        assert_eq!(first, repeated);
        assert_eq!(first, shifted);
    }

    #[test]
    fn weighted_average_gradient_matches_finite_differences() {
        let (graph, _) = wide_net(3, Some(7));
        let positions = [(0.00, 0.18), (0.08, 0.02), (0.15, 0.11), (0.22, 0.27)];
        let fixed = vec![None; positions.len()];
        let offsets = vec![(0.0, 0.0); positions.len()];
        let objective = graph.weighted_average_objective_with_global_gamma(
            &positions,
            &fixed,
            &offsets,
            TEST_GLOBAL_GAMMA,
        );
        let step = 1.0e-6;

        for unit in 0..positions.len() {
            for y_axis in [false, true] {
                let mut lower = positions;
                let mut upper = positions;
                if y_axis {
                    lower[unit].1 -= step;
                    upper[unit].1 += step;
                } else {
                    lower[unit].0 -= step;
                    upper[unit].0 += step;
                }
                let lower_value = graph
                    .weighted_average_objective_with_global_gamma(
                        &lower,
                        &fixed,
                        &offsets,
                        TEST_GLOBAL_GAMMA,
                    )
                    .value;
                let upper_value = graph
                    .weighted_average_objective_with_global_gamma(
                        &upper,
                        &fixed,
                        &offsets,
                        TEST_GLOBAL_GAMMA,
                    )
                    .value;
                let finite_difference = (upper_value - lower_value) / (2.0 * step);
                let analytical = if y_axis {
                    objective.gradient_y[unit]
                } else {
                    objective.gradient_x[unit]
                };
                assert_close(analytical, finite_difference, 2.0e-8);
            }
        }
    }

    #[test]
    fn weighted_average_uses_fixed_coordinates_and_macro_offsets() {
        let mut design = Design::new();
        let _root = design.add_cell("root", ResourceKind::Logic);
        let member = design.add_cell("member", ResourceKind::Logic);
        let output = design.add_pin(member, "out", PinDirection::Output).unwrap();
        let fixed_sink = design.add_cell("fixed", ResourceKind::Logic);
        let fixed_input = design
            .add_pin(fixed_sink, "in", PinDirection::Input)
            .unwrap();
        let moving_sink = design.add_cell("moving", ResourceKind::Logic);
        let moving_input = design
            .add_pin(moving_sink, "in", PinDirection::Input)
            .unwrap();
        design
            .add_net("macro", output, [fixed_input, moving_input])
            .unwrap();
        let graph = AnalyticalHypergraph::new(&design, &[0, 0, 1, 2], 3, &BTreeMap::new());
        let positions = [(2.0, 4.0), (99.0, 99.0), (7.0, 8.0)];
        let fixed = [None, None, Some((10.0, 5.0)), None];
        let offsets = [(0.0, 0.0), (3.0, -2.0), (0.0, 0.0), (-1.0, 1.0)];
        let objective = graph.weighted_average_objective_with_global_gamma(
            &positions,
            &fixed,
            &offsets,
            TEST_GLOBAL_GAMMA,
        );

        let (reference, _) = wide_net(2, None);
        let reference_objective = reference.weighted_average_objective_with_global_gamma(
            &[(5.0, 2.0), (0.0, 0.0), (6.0, 9.0)],
            &[None, Some((10.0, 5.0)), None],
            &[(0.0, 0.0); 3],
            TEST_GLOBAL_GAMMA,
        );
        assert_close(objective.value, reference_objective.value, 1.0e-14);
        assert_close(
            objective.gradient_x[0],
            reference_objective.gradient_x[0],
            1.0e-14,
        );
        assert_close(
            objective.gradient_y[0],
            reference_objective.gradient_y[0],
            1.0e-14,
        );
        assert_close(
            objective.gradient_x[2],
            reference_objective.gradient_x[2],
            1.0e-14,
        );
        assert_close(
            objective.gradient_y[2],
            reference_objective.gradient_y[2],
            1.0e-14,
        );
        assert_eq!(
            (objective.gradient_x[1], objective.gradient_y[1]),
            (0.0, 0.0)
        );

        let mut changed_fixed_origin = positions;
        changed_fixed_origin[1] = (-1_000.0, -1_000.0);
        assert_eq!(
            objective,
            graph.weighted_average_objective_with_global_gamma(
                &changed_fixed_origin,
                &fixed,
                &offsets,
                TEST_GLOBAL_GAMMA,
            )
        );
    }

    #[test]
    fn rudy_boxes_exclude_internal_carry_nets_but_keep_every_external_fanout() {
        let mut design = Design::new();
        let carry = design.add_cell("carry", ResourceKind::Lut(4));
        let carry_out = design.add_pin(carry, "FCO", PinDirection::Output).unwrap();
        let successor = design.add_cell("successor", ResourceKind::Lut(4));
        let carry_in = design
            .add_pin(successor, "FCI", PinDirection::Input)
            .unwrap();
        let result = design
            .add_pin(successor, "F", PinDirection::Output)
            .unwrap();
        let packed_ff = design.add_cell("packed-ff", ResourceKind::Register);
        let packed_di = design
            .add_pin(packed_ff, "DI", PinDirection::Input)
            .unwrap();
        let external_a = design.add_cell("external-a", ResourceKind::Register);
        let near_input = design
            .add_pin(external_a, "D", PinDirection::Input)
            .unwrap();
        let external_b = design.add_cell("external-b", ResourceKind::Register);
        let far_input = design
            .add_pin(external_b, "D", PinDirection::Input)
            .unwrap();
        design.add_net("dedicated", carry_out, [carry_in]).unwrap();
        design
            .add_net(
                "result-with-local-and-external-fanout",
                result,
                [packed_di, near_input, far_input],
            )
            .unwrap();
        let graph = AnalyticalHypergraph::new(&design, &[0, 0, 0, 1, 2], 3, &BTreeMap::new());
        let boxes = graph.external_baseline_net_bounding_boxes(
            &[(4.0, 7.0), (12.0, 2.0), (1.0, 11.0)],
            &[None, None, None, None, None],
            &[(0.0, 0.0), (3.0, 1.0), (3.0, 1.0), (0.0, 0.0), (0.0, 0.0)],
        );

        assert_eq!(boxes, vec![(1.0, 12.0, 2.0, 11.0)]);
    }

    #[test]
    fn weighted_average_timing_overlay_is_exactly_sink_local() {
        let (baseline, _) = wide_net(2, None);
        let (critical, _) = wide_net(2, Some(5));
        let (two_pin, _) = wide_net(1, None);
        let positions = [(0.0, 0.0), (2.0, 1.0), (-1.0, 3.0)];
        let fixed = [None; 3];
        let offsets = [(0.0, 0.0); 3];
        let baseline_objective = baseline.weighted_average_objective_with_global_gamma(
            &positions,
            &fixed,
            &offsets,
            TEST_GLOBAL_GAMMA,
        );
        let critical_objective = critical.weighted_average_objective_with_global_gamma(
            &positions,
            &fixed,
            &offsets,
            TEST_GLOBAL_GAMMA,
        );
        let arc = two_pin.weighted_average_objective_with_global_gamma(
            &positions[..2],
            &fixed[..2],
            &offsets[..2],
            TEST_GLOBAL_GAMMA,
        );

        assert_close(
            critical_objective.value - baseline_objective.value,
            4.0 * arc.value,
            1.0e-14,
        );
        for unit in 0..2 {
            assert_close(
                critical_objective.gradient_x[unit] - baseline_objective.gradient_x[unit],
                4.0 * arc.gradient_x[unit],
                1.0e-14,
            );
            assert_close(
                critical_objective.gradient_y[unit] - baseline_objective.gradient_y[unit],
                4.0 * arc.gradient_y[unit],
                1.0e-14,
            );
        }
        assert_close(
            critical_objective.gradient_x[2],
            baseline_objective.gradient_x[2],
            1.0e-14,
        );
        assert_close(
            critical_objective.gradient_y[2],
            baseline_objective.gradient_y[2],
            1.0e-14,
        );

        let gamma = 2.5;
        let (critical_baseline, critical_timing) = critical
            .weighted_average_components_with_global_gamma(&positions, &fixed, &offsets, gamma);
        let critical_total = critical
            .weighted_average_objective_with_global_gamma(&positions, &fixed, &offsets, gamma);
        assert_close(
            critical_total.value,
            critical_baseline.value + critical_timing.value,
            1.0e-14,
        );
        for unit in 0..positions.len() {
            assert_close(
                critical_total.gradient_x[unit],
                critical_baseline.gradient_x[unit] + critical_timing.gradient_x[unit],
                1.0e-14,
            );
            assert_close(
                critical_total.gradient_y[unit],
                critical_baseline.gradient_y[unit] + critical_timing.gradient_y[unit],
                1.0e-14,
            );
        }
    }

    #[test]
    fn b2b_influence_is_continuous_across_legacy_fanout_boundary() {
        let baseline_weight = |fanout| {
            let (graph, positions) = wide_net(fanout, None);
            graph
                .linearize_axis(
                    &positions,
                    &vec![None; positions.len()],
                    &vec![0.0; positions.len()],
                )
                .into_iter()
                .filter(|edge| edge.kind == AxisEdgeKind::Baseline)
                .map(|edge| edge.weight)
                .sum::<f64>()
        };
        let at_256 = baseline_weight(256);
        let at_257 = baseline_weight(257);
        let at_402 = baseline_weight(402);

        assert!(at_256 > 0.0 && at_257 > 0.0 && at_402 > 0.0);
        assert!((at_257 / at_256 - 1.0).abs() < 0.01);
        assert!((at_402 / at_256 - 1.0).abs() < 0.01);
    }

    #[test]
    fn timed_overlay_is_sink_local_and_not_diluted_by_fanout() {
        let overlay = |fanout| {
            let (graph, mut positions) = wide_net(fanout, Some(64));
            positions[1] = 8.0;
            graph
                .linearize_axis(
                    &positions,
                    &vec![None; positions.len()],
                    &vec![0.0; positions.len()],
                )
                .into_iter()
                .filter(|edge| edge.kind == AxisEdgeKind::TimingOverlay)
                .collect::<Vec<_>>()
        };
        let narrow = overlay(2);
        let wide = overlay(257);

        assert_eq!(narrow.len(), 1);
        assert_eq!(wide.len(), 1);
        assert_eq!((narrow[0].left, narrow[0].right), (0, 1));
        assert_eq!((wide[0].left, wide[0].right), (0, 1));
        assert!((narrow[0].weight - wide[0].weight).abs() <= f64::EPSILON);
    }

    #[test]
    fn full_u64_timing_weight_linearizes_without_narrowing_or_panicking() {
        let (graph, mut positions) = wide_net(1, Some(u64::MAX));
        positions[1] = 1.0;
        let overlay = graph
            .linearize_axis(&positions, &[None, None], &[0.0, 0.0])
            .into_iter()
            .find(|edge| edge.kind == AxisEdgeKind::TimingOverlay)
            .unwrap();

        assert!(overlay.weight.is_finite());
        assert!(overlay.weight > 64.0 * f64::from(u32::MAX));
    }

    #[test]
    fn collapsed_bounds_are_deterministic_and_span_distinct_units() {
        let (graph, positions) = wide_net(4, None);
        let fixed = vec![None; positions.len()];
        let offsets = vec![0.0; positions.len()];

        let first = graph.linearize_axis(&positions, &fixed, &offsets);
        let second = graph.linearize_axis(&positions, &fixed, &offsets);

        assert_eq!(first, second);
        assert!(first.iter().all(|edge| edge.left != edge.right));
    }

    #[test]
    fn legal_spread_changes_bounds_selected_from_a_collapsed_tie() {
        let (graph, collapsed) = wide_net(3, None);
        let fixed = vec![None; collapsed.len()];
        let offsets = vec![0.0; collapsed.len()];
        let collapsed_edges = graph.linearize_axis(&collapsed, &fixed, &offsets);
        let legal_edges = graph.linearize_axis(&[5.0, 0.0, 10.0, 6.0], &fixed, &offsets);
        let has_pair = |edges: &[super::AxisEdge], left, right| {
            edges.iter().any(|edge| {
                (edge.left == left && edge.right == right)
                    || (edge.left == right && edge.right == left)
            })
        };

        assert!(has_pair(&collapsed_edges, 0, 3));
        assert!(!has_pair(&collapsed_edges, 1, 2));
        assert!(has_pair(&legal_edges, 1, 2));
        assert!(!has_pair(&legal_edges, 0, 3));
    }

    #[test]
    fn exact_objective_uses_selected_member_bels_and_every_sink_endpoint() {
        let mut design = Design::new();
        let macro_root = design.add_cell("macro_root", ResourceKind::Logic);
        let macro_member = design.add_cell("macro_member", ResourceKind::Logic);
        let member_out = design
            .add_pin(macro_member, "out", PinDirection::Output)
            .unwrap();
        let sink = design.add_cell("sink", ResourceKind::Logic);
        let sink_a = design.add_pin(sink, "a", PinDirection::Input).unwrap();
        let sink_b = design.add_pin(sink, "b", PinDirection::Input).unwrap();
        design
            .add_net("member_to_sink", member_out, [sink_a, sink_b])
            .unwrap();
        let clock = design.add_cell("clock", ResourceKind::Clock);
        let clock_out = design.add_pin(clock, "out", PinDirection::Output).unwrap();
        let clock_sink = design.add_cell("clock_sink", ResourceKind::Logic);
        let clock_in = design
            .add_pin(clock_sink, "clock", PinDirection::Input)
            .unwrap();
        design
            .add_net("clock_network", clock_out, [clock_in])
            .unwrap();

        let unit_by_cell = [0, 0, 1, 2, 3];
        let weights = BTreeMap::from([
            ((NetId(0), sink_a), 3),
            ((NetId(0), sink_b), 4),
            ((NetId(1), clock_in), u64::MAX),
        ]);
        let graph = AnalyticalHypergraph::new(&design, &unit_by_cell, 4, &weights);
        assert_eq!(graph.nets.len(), 1, "clock-driver net must be excluded");
        assert_eq!(graph.nets[0].pins.len(), 3, "sink pins retain multiplicity");
        assert_eq!(graph.timed_arcs.len(), 2);

        let mut device = Device::new("objective", 32, 16).unwrap();
        let bindings = [
            device
                .add_bel("root", ResourceKind::Logic, Point::new(0, 0))
                .unwrap(),
            device
                .add_bel("member", ResourceKind::Logic, Point::new(4, 1))
                .unwrap(),
            device
                .add_bel("sink", ResourceKind::Logic, Point::new(10, 3))
                .unwrap(),
            device
                .add_bel("clock", ResourceKind::Clock, Point::new(1, 1))
                .unwrap(),
            device
                .add_bel("clock_sink", ResourceKind::Logic, Point::new(31, 15))
                .unwrap(),
        ]
        .map(Some);

        // Member-to-sink distance is 8. Baseline HPWL is 8 and the two
        // distinct sink pins add excess weights (3 - 1) + (4 - 1) = 5.
        assert_eq!(
            graph.exact_objective(&device, &bindings).unwrap(),
            super::AnalyticalObjective { hpwl: 8, total: 48 }
        );

        let coordinates = legalized_coordinates(
            &device,
            &bindings,
            &unit_by_cell,
            &[macro_root, sink, clock, clock_sink],
        )
        .unwrap();
        assert_eq!(coordinates.origins[0], Point::new(0, 0));
        assert_eq!(coordinates.cell_offsets[macro_member.0], (4.0, 1.0));
    }

    #[test]
    fn same_unit_geometry_is_scored_but_remains_outside_continuous_system() {
        let mut design = Design::new();
        let source = design.add_cell("source", ResourceKind::Logic);
        let output = design.add_pin(source, "out", PinDirection::Output).unwrap();
        let sink = design.add_cell("sink", ResourceKind::Logic);
        let input = design.add_pin(sink, "in", PinDirection::Input).unwrap();
        design.add_net("internal", output, [input]).unwrap();
        let graph = AnalyticalHypergraph::new(
            &design,
            &[0, 0],
            1,
            &BTreeMap::from([((NetId(0), input), 5)]),
        );

        // Retaining the net for exact discrete scoring must not introduce a
        // false degree of freedom into the established continuous model.
        assert!(
            graph
                .linearize_axis(&[1.0], &[None, None], &[0.0, 6.0])
                .is_empty()
        );

        let mut device = Device::new("internal", 8, 5).unwrap();
        let bindings = [
            Some(
                device
                    .add_bel("source", ResourceKind::Logic, Point::new(1, 1))
                    .unwrap(),
            ),
            Some(
                device
                    .add_bel("sink", ResourceKind::Logic, Point::new(7, 4))
                    .unwrap(),
            ),
        ];
        // Distance 9 plus excess weight four times distance 9.
        assert_eq!(
            graph.exact_objective(&device, &bindings).unwrap(),
            super::AnalyticalObjective { hpwl: 9, total: 45 }
        );
    }

    #[test]
    fn projected_mm_targets_start_at_legal_spread_and_keep_fixed_units_exact() {
        let legal = LegalizedCoordinates {
            origins: vec![
                Point::new(1, 2),
                Point::new(3, 4),
                Point::new(7, 6),
                Point::new(9, 8),
            ],
            cell_offsets: Vec::new(),
        };
        let full = projected_mm_targets(
            (&[2.0, 2.0, 7.0, -100.0], &[0.0, 7.0, 6.0, -100.0]),
            &legal,
            &[None, None, None, Some(Point::new(9, 8))],
            1.0,
        );
        assert_eq!(full, [(2.0, 0.0), (2.0, 7.0), (7.0, 6.0), (9.0, 8.0)]);

        let half = projected_mm_targets(
            (&[3.0, 1.0, 7.0, -100.0], &[0.0, 10.0, 6.0, -100.0]),
            &legal,
            &[None, None, None, Some(Point::new(9, 8))],
            0.5,
        );
        assert_eq!(half, [(2.0, 1.0), (2.0, 7.0), (7.0, 6.0), (9.0, 8.0)]);
    }
}
