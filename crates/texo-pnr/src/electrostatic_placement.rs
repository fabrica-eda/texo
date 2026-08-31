//! Deterministic electrostatic density primitives for global placement.

#![cfg_attr(not(test), allow(dead_code))]

use std::collections::BTreeMap;
use std::f64::consts::PI;
use std::fmt;
use std::sync::Arc;

use rayon::prelude::*;
use rustdct::{DctPlanner, TransformType2And3};
use texo_model::{Device, Point, ResourceKind};

/// Failure to construct or solve one discrete Poisson system.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum ElectrostaticError {
    EmptyGrid,
    GridTooLarge,
    DensityLength {
        expected: usize,
        actual: usize,
    },
    NonFiniteDensity {
        index: usize,
    },
    EmptyUnit {
        unit: usize,
    },
    NonFiniteUnitCoordinate {
        unit: usize,
        member: usize,
    },
    InvalidMemberCharge {
        unit: usize,
        member: usize,
    },
    PointOutsideGrid {
        point: Point,
    },
    FixedCapacityExhausted {
        kind: ResourceKind,
        point: Point,
    },
    InsufficientCapacity {
        kind: ResourceKind,
        demand: usize,
        available: usize,
    },
    InsufficientArea {
        kind: ResourceKind,
        demand_bits: u64,
        available_bits: u64,
    },
    NonFiniteFiller {
        filler: usize,
    },
    NegativeFillerCharge {
        filler: usize,
    },
    UnexpectedFillerKind {
        filler: usize,
        kind: ResourceKind,
    },
    FillerChargeMismatch {
        kind: ResourceKind,
        expected_bits: u64,
        actual_bits: u64,
    },
}

/// One physical-resource charge rigidly attached to a placement-unit origin.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct DensityMember {
    pub(super) kind: ResourceKind,
    pub(super) offset_x: f64,
    pub(super) offset_y: f64,
    /// Physical area/charge after elfPlace instance-area adjustment.
    pub(super) charge: f64,
}

/// One atomic optimizer variable. All member forces act on this one origin.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct DensityUnit {
    pub(super) origin_x: f64,
    pub(super) origin_y: f64,
    pub(super) members: Vec<DensityMember>,
}

/// One occupied BEL removed from physical capacity before filler construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct FixedOccupancy {
    pub(super) kind: ResourceKind,
    pub(super) point: Point,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct DensityFiller {
    pub(super) kind: ResourceKind,
    pub(super) x: f64,
    pub(super) y: f64,
    pub(super) charge: f64,
}

/// One resource field, including the exact charge that future fillers supply.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct DensityFieldResult {
    pub(super) kind: ResourceKind,
    pub(super) available_capacity: usize,
    /// Number of physical movable instances in this field.
    pub(super) real_charge: usize,
    /// Total adjusted physical instance area used by Eq. (7).
    pub(super) real_area: f64,
    pub(super) filler_charge: f64,
    pub(super) density: Vec<f64>,
    pub(super) energy: f64,
    pub(super) normalized_positive_overflow: f64,
    pub(super) net_charge: f64,
    /// Equation (27)'s per-field density-force norm before rigid-macro
    /// member forces are accumulated at their shared optimizer coordinate.
    pub(super) force_l1: f64,
    pub(super) unit_gradients: Vec<(f64, f64)>,
    pub(super) filler_gradients: Vec<(usize, f64, f64)>,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct DensityResult {
    pub(super) fields: Vec<DensityFieldResult>,
}

/// Resource-separated fields sharing placement-unit optimizer variables.
#[derive(Clone, Debug)]
pub(super) struct DensityModel {
    solver: Poisson2d,
    capacity: BTreeMap<ResourceKind, Vec<u32>>,
    smoothed_capacity: BTreeMap<ResourceKind, Vec<f64>>,
}

impl DensityModel {
    pub(super) fn new(
        device: &Device,
        fixed: &[FixedOccupancy],
    ) -> Result<Self, ElectrostaticError> {
        let solver = Poisson2d::new(device.width(), device.height())?;
        let grid_len = solver.width * solver.height;
        let mut capacity = BTreeMap::<ResourceKind, Vec<u32>>::new();
        for bel in device.bels() {
            let index = point_index(bel.point, solver.width, solver.height)?;
            let entry = &mut capacity
                .entry(bel.kind)
                .or_insert_with(|| vec![0; grid_len])[index];
            *entry = entry
                .checked_add(1)
                .ok_or(ElectrostaticError::GridTooLarge)?;
        }
        for occupancy in fixed {
            let index = point_index(occupancy.point, solver.width, solver.height)?;
            let Some(entry) = capacity
                .get_mut(&occupancy.kind)
                .map(|field| &mut field[index])
            else {
                return Err(ElectrostaticError::FixedCapacityExhausted {
                    kind: occupancy.kind,
                    point: occupancy.point,
                });
            };
            *entry = entry
                .checked_sub(1)
                .ok_or(ElectrostaticError::FixedCapacityExhausted {
                    kind: occupancy.kind,
                    point: occupancy.point,
                })?;
        }
        let mut smoothed_capacity = BTreeMap::new();
        for (&kind, discrete) in &capacity {
            let mut smooth = vec![0.0; grid_len];
            for (index, &charge) in discrete.iter().enumerate() {
                if charge == 0 {
                    continue;
                }
                deposit_quadratic_bspline(
                    &mut smooth,
                    solver.width,
                    solver.height,
                    usize_to_f64(index % solver.width)?,
                    usize_to_f64(index / solver.width)?,
                    f64::from(charge),
                );
            }
            smoothed_capacity.insert(kind, smooth);
        }
        Ok(Self {
            solver,
            capacity,
            smoothed_capacity,
        })
    }

    pub(super) fn initial_fillers(
        &self,
        units: &[DensityUnit],
    ) -> Result<Vec<DensityFiller>, ElectrostaticError> {
        validate_units(units)?;
        let counts = real_counts(units);
        let areas = real_areas(units);
        let mut fillers = Vec::new();
        for (kind, real) in counts {
            let (capacity, available) = self.capacity_and_total(kind)?;
            ensure_capacity(kind, real, available)?;
            let available_area = usize_to_f64(available)?;
            let real_area = areas[&kind];
            ensure_area_capacity(kind, real_area, available_area)?;
            let filler_area = available_area - real_area;
            if filler_area == 0.0 {
                continue;
            }
            let total = filler_area;
            let denominator = available_area;
            let occupied_bins = capacity
                .iter()
                .enumerate()
                .filter(|(_, capacity)| **capacity != 0)
                .collect::<Vec<_>>();
            let mut assigned = 0.0;
            for (ordinal, &(index, &bin_capacity)) in occupied_bins.iter().enumerate() {
                let charge = if ordinal + 1 == occupied_bins.len() {
                    total - assigned
                } else {
                    total * f64::from(bin_capacity) / denominator
                };
                assigned += charge;
                fillers.push(DensityFiller {
                    kind,
                    x: usize_to_f64(index % self.solver.width)?,
                    y: usize_to_f64(index / self.solver.width)?,
                    charge,
                });
            }
        }
        Ok(fillers)
    }

    #[allow(clippy::too_many_lines)]
    pub(super) fn evaluate(
        &self,
        units: &[DensityUnit],
        fillers: &[DensityFiller],
    ) -> Result<DensityResult, ElectrostaticError> {
        self.evaluate_with_positions(
            units,
            fillers,
            |index| (units[index].origin_x, units[index].origin_y),
            |index| (fillers[index].x, fillers[index].y),
        )
    }

    /// Evaluates unchanged density topology at caller-owned optimizer
    /// coordinates.
    ///
    /// Nonlinear placement changes only unit origins and filler positions.
    /// Keeping those coordinates outside [`DensityUnit`] and
    /// [`DensityFiller`] avoids deep-cloning every rigid macro's member list
    /// for each line-search trial while retaining one shared origin for all
    /// macro members and all resource fields.
    #[allow(clippy::too_many_lines)]
    pub(super) fn evaluate_with_positions<UnitPosition, FillerPosition>(
        &self,
        units: &[DensityUnit],
        fillers: &[DensityFiller],
        unit_position: UnitPosition,
        filler_position: FillerPosition,
    ) -> Result<DensityResult, ElectrostaticError>
    where
        UnitPosition: Fn(usize) -> (f64, f64) + Sync,
        FillerPosition: Fn(usize) -> (f64, f64) + Sync,
    {
        validate_units_with_positions(units, &unit_position)?;
        validate_fillers_with_positions(fillers, &filler_position)?;
        let counts = real_counts(units);
        let areas = real_areas(units);
        validate_filler_totals(self, &areas, fillers)?;
        let field_inputs = counts
            .into_iter()
            .map(|(kind, real_charge)| (kind, real_charge, areas[&kind]))
            .collect::<Vec<_>>();
        // Resource fields are algebraically independent. `collect` on this
        // indexed parallel iterator retains the BTreeMap's stable kind order,
        // so parallel evaluation does not change floating-point reduction or
        // output ordering within any field.
        let evaluated = field_inputs
            .into_par_iter()
            .map(|(kind, real_charge, real_area)| {
                let deposit_started = std::time::Instant::now();
                let (_, available_capacity) = self.capacity_and_total(kind)?;
                ensure_capacity(kind, real_charge, available_capacity)?;
                ensure_area_capacity(kind, real_area, usize_to_f64(available_capacity)?)?;
                let mut density = self.smoothed_capacity[&kind]
                    .iter()
                    .map(|&entry| -entry)
                    .collect::<Vec<_>>();
                for (unit_index, unit) in units.iter().enumerate() {
                    let (origin_x, origin_y) = unit_position(unit_index);
                    for member in unit.members.iter().filter(|member| member.kind == kind) {
                        deposit_quadratic_bspline(
                            &mut density,
                            self.solver.width,
                            self.solver.height,
                            origin_x + member.offset_x,
                            origin_y + member.offset_y,
                            member.charge,
                        );
                    }
                }
                // ePlace equation (37), and elfPlace equation (7), define
                // overflow from physical movable demand above capacity.  Fillers
                // complete the electrostatic charge field but are not physical
                // instances and therefore must not affect gamma or termination.
                let physical_positive_overflow =
                    density.iter().map(|&entry| entry.max(0.0)).sum::<f64>();
                for (filler_index, filler) in fillers.iter().enumerate() {
                    if filler.kind != kind {
                        continue;
                    }
                    let (x, y) = filler_position(filler_index);
                    deposit_quadratic_bspline(
                        &mut density,
                        self.solver.width,
                        self.solver.height,
                        x,
                        y,
                        filler.charge,
                    );
                }
                let deposit_elapsed = deposit_started.elapsed();
                let poisson_started = std::time::Instant::now();
                let solution = self.solver.solve(&density)?;
                let poisson_elapsed = poisson_started.elapsed();
                let gradient_started = std::time::Instant::now();
                let mut unit_gradients = vec![(0.0, 0.0); units.len()];
                let mut force_l1 = 0.0;
                for (unit_index, unit) in units.iter().enumerate() {
                    let (origin_x, origin_y) = unit_position(unit_index);
                    for member in unit.members.iter().filter(|member| member.kind == kind) {
                        let (_, gradient_x, gradient_y) = sample_quadratic_bspline_with_gradient(
                            &solution.potential,
                            self.solver.width,
                            self.solver.height,
                            origin_x + member.offset_x,
                            origin_y + member.offset_y,
                        );
                        force_l1 += member.charge * (gradient_x.abs() + gradient_y.abs());
                        unit_gradients[unit_index].0 += member.charge * gradient_x;
                        unit_gradients[unit_index].1 += member.charge * gradient_y;
                    }
                }
                let mut filler_gradients = Vec::new();
                for (filler_index, filler) in fillers.iter().enumerate() {
                    if filler.kind != kind {
                        continue;
                    }
                    let (x, y) = filler_position(filler_index);
                    let (_, gradient_x, gradient_y) = sample_quadratic_bspline_with_gradient(
                        &solution.potential,
                        self.solver.width,
                        self.solver.height,
                        x,
                        y,
                    );
                    force_l1 += filler.charge * (gradient_x.abs() + gradient_y.abs());
                    filler_gradients.push((
                        filler_index,
                        filler.charge * gradient_x,
                        filler.charge * gradient_y,
                    ));
                }
                let net_charge = density.iter().sum();
                let gradient_elapsed = gradient_started.elapsed();
                Ok((
                    DensityFieldResult {
                        kind,
                        available_capacity,
                        real_charge,
                        real_area,
                        filler_charge: usize_to_f64(available_capacity)? - real_area,
                        density,
                        energy: solution.energy,
                        normalized_positive_overflow: physical_positive_overflow / real_area,
                        net_charge,
                        force_l1,
                        unit_gradients,
                        filler_gradients,
                    },
                    deposit_elapsed,
                    poisson_elapsed,
                    gradient_elapsed,
                ))
            })
            .collect::<Vec<Result<_, ElectrostaticError>>>();
        let mut deposit_elapsed = std::time::Duration::ZERO;
        let mut poisson_elapsed = std::time::Duration::ZERO;
        let mut gradient_elapsed = std::time::Duration::ZERO;
        let mut fields = Vec::with_capacity(evaluated.len());
        for result in evaluated {
            let (field, deposit, poisson, gradient) = result?;
            deposit_elapsed += deposit;
            poisson_elapsed += poisson;
            gradient_elapsed += gradient;
            fields.push(field);
        }
        if std::env::var_os("TEXO_PNR_TRACE_DENSITY_EVALUATIONS").is_some() {
            eprintln!(
                "TEXO_PNR_TRACE density-evaluation deposit_us={} poisson_us={} gradient_us={} fields={} units={} fillers={}",
                deposit_elapsed.as_micros(),
                poisson_elapsed.as_micros(),
                gradient_elapsed.as_micros(),
                fields.len(),
                units.len(),
                fillers.len(),
            );
        }
        Ok(DensityResult { fields })
    }

    fn capacity_and_total(
        &self,
        kind: ResourceKind,
    ) -> Result<(&[u32], usize), ElectrostaticError> {
        let capacity = self.capacity.get(&kind).map_or(&[][..], Vec::as_slice);
        let total = capacity.iter().try_fold(0_usize, |sum, &entry| {
            sum.checked_add(usize::try_from(entry).map_err(|_| ElectrostaticError::GridTooLarge)?)
                .ok_or(ElectrostaticError::GridTooLarge)
        })?;
        Ok((capacity, total))
    }

    pub(super) fn available_capacity(
        &self,
        kind: ResourceKind,
    ) -> Result<usize, ElectrostaticError> {
        self.capacity_and_total(kind).map(|(_, total)| total)
    }
}

fn real_counts(units: &[DensityUnit]) -> BTreeMap<ResourceKind, usize> {
    let mut counts = BTreeMap::new();
    for unit in units {
        for member in &unit.members {
            *counts.entry(member.kind).or_default() += 1;
        }
    }
    counts
}

fn real_areas(units: &[DensityUnit]) -> BTreeMap<ResourceKind, f64> {
    let mut areas = BTreeMap::new();
    for unit in units {
        for member in &unit.members {
            *areas.entry(member.kind).or_default() += member.charge;
        }
    }
    areas
}

fn ensure_capacity(
    kind: ResourceKind,
    demand: usize,
    available: usize,
) -> Result<(), ElectrostaticError> {
    if demand > available {
        return Err(ElectrostaticError::InsufficientCapacity {
            kind,
            demand,
            available,
        });
    }
    Ok(())
}

fn ensure_area_capacity(
    kind: ResourceKind,
    demand: f64,
    available: f64,
) -> Result<(), ElectrostaticError> {
    if !demand.is_finite() || demand < 0.0 || demand > available + 1.0e-9 {
        return Err(ElectrostaticError::InsufficientArea {
            kind,
            demand_bits: demand.to_bits(),
            available_bits: available.to_bits(),
        });
    }
    Ok(())
}

fn validate_fillers_with_positions<Position>(
    fillers: &[DensityFiller],
    position: &Position,
) -> Result<(), ElectrostaticError>
where
    Position: Fn(usize) -> (f64, f64) + ?Sized,
{
    for (index, filler) in fillers.iter().enumerate() {
        let (x, y) = position(index);
        if !x.is_finite() || !y.is_finite() || !filler.charge.is_finite() {
            return Err(ElectrostaticError::NonFiniteFiller { filler: index });
        }
        if filler.charge < 0.0 {
            return Err(ElectrostaticError::NegativeFillerCharge { filler: index });
        }
    }
    Ok(())
}

fn validate_filler_totals(
    model: &DensityModel,
    areas: &BTreeMap<ResourceKind, f64>,
    fillers: &[DensityFiller],
) -> Result<(), ElectrostaticError> {
    let mut actual = BTreeMap::<ResourceKind, f64>::new();
    for (index, filler) in fillers.iter().enumerate() {
        if !areas.contains_key(&filler.kind) {
            return Err(ElectrostaticError::UnexpectedFillerKind {
                filler: index,
                kind: filler.kind,
            });
        }
        *actual.entry(filler.kind).or_default() += filler.charge;
    }
    for (&kind, &real) in areas {
        let (_, available) = model.capacity_and_total(kind)?;
        let available = usize_to_f64(available)?;
        ensure_area_capacity(kind, real, available)?;
        let expected = available - real;
        let supplied = actual.get(&kind).copied().unwrap_or(0.0);
        if (supplied - expected).abs() > 1.0e-9 * expected.max(1.0) {
            return Err(ElectrostaticError::FillerChargeMismatch {
                kind,
                expected_bits: expected.to_bits(),
                actual_bits: supplied.to_bits(),
            });
        }
    }
    Ok(())
}

fn validate_units(units: &[DensityUnit]) -> Result<(), ElectrostaticError> {
    validate_units_with_positions(units, &|index| {
        (units[index].origin_x, units[index].origin_y)
    })
}

fn validate_units_with_positions<Position>(
    units: &[DensityUnit],
    position: &Position,
) -> Result<(), ElectrostaticError>
where
    Position: Fn(usize) -> (f64, f64) + ?Sized,
{
    for (unit_index, unit) in units.iter().enumerate() {
        if unit.members.is_empty() {
            return Err(ElectrostaticError::EmptyUnit { unit: unit_index });
        }
        let (origin_x, origin_y) = position(unit_index);
        for (member_index, member) in unit.members.iter().enumerate() {
            if !origin_x.is_finite()
                || !origin_y.is_finite()
                || !member.offset_x.is_finite()
                || !member.offset_y.is_finite()
            {
                return Err(ElectrostaticError::NonFiniteUnitCoordinate {
                    unit: unit_index,
                    member: member_index,
                });
            }
            if !member.charge.is_finite() || member.charge <= 0.0 {
                return Err(ElectrostaticError::InvalidMemberCharge {
                    unit: unit_index,
                    member: member_index,
                });
            }
        }
    }
    Ok(())
}

fn point_index(point: Point, width: usize, height: usize) -> Result<usize, ElectrostaticError> {
    let x = usize::try_from(point.x).map_err(|_| ElectrostaticError::GridTooLarge)?;
    let y = usize::try_from(point.y).map_err(|_| ElectrostaticError::GridTooLarge)?;
    if x >= width || y >= height {
        return Err(ElectrostaticError::PointOutsideGrid { point });
    }
    y.checked_mul(width)
        .and_then(|row| row.checked_add(x))
        .ok_or(ElectrostaticError::GridTooLarge)
}

/// Potential and electrostatic energy of one zero-mean density field.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct ElectrostaticSolution {
    pub(super) potential: Vec<f64>,
    pub(super) energy: f64,
}

/// Spectral inverse of the cell-centred Neumann grid Laplacian.
///
/// The orthonormal two-dimensional DCT-II/III pair is evaluated by `RustDCT`.
/// Its planner supports arbitrary architecture dimensions in `O(n log n)`;
/// no radix-two padding changes the physical coordinate grid.
#[derive(Clone, Debug)]
pub(super) struct Poisson2d {
    width: usize,
    height: usize,
    transform_x: OrthonormalDct,
    transform_y: OrthonormalDct,
    inverse_eigenvalues: Vec<f64>,
}

#[derive(Clone)]
struct OrthonormalDct {
    length: usize,
    transform: Arc<dyn TransformType2And3<f64>>,
    dc_forward_scale: f64,
    dc_inverse_scale: f64,
    non_dc_scale: f64,
}

impl fmt::Debug for OrthonormalDct {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("OrthonormalDct")
            .field("length", &self.length)
            .finish_non_exhaustive()
    }
}

impl OrthonormalDct {
    fn new(length: usize, planner: &mut DctPlanner<f64>) -> Self {
        let length_f64 = f64::from(u32::try_from(length).expect("DCT dimension fits u32"));
        Self {
            length,
            transform: planner.plan_dct2(length),
            dc_forward_scale: length_f64.recip().sqrt(),
            dc_inverse_scale: 2.0 * length_f64.recip().sqrt(),
            non_dc_scale: (2.0 / length_f64).sqrt(),
        }
    }

    fn scratch(&self) -> Vec<f64> {
        vec![0.0; self.transform.get_scratch_len()]
    }

    /// `RustDCT`'s unnormalised DCT-II is `sum(x[n] cos(...))`.
    /// Scale it into the orthonormal basis used by the density equations.
    fn forward(&self, values: &mut [f64], scratch: &mut [f64]) {
        debug_assert_eq!(values.len(), self.length);
        self.transform.process_dct2_with_scratch(values, scratch);
        values[0] *= self.dc_forward_scale;
        for value in &mut values[1..] {
            *value *= self.non_dc_scale;
        }
    }

    /// `RustDCT`'s unnormalised DCT-III is
    /// `input[0]/2 + sum(input[k] cos(...))`. Pre-scale orthonormal
    /// coefficients so the transform is the exact algebraic inverse above.
    fn inverse(&self, values: &mut [f64], scratch: &mut [f64]) {
        debug_assert_eq!(values.len(), self.length);
        values[0] *= self.dc_inverse_scale;
        for value in &mut values[1..] {
            *value *= self.non_dc_scale;
        }
        self.transform.process_dct3_with_scratch(values, scratch);
    }
}

impl Poisson2d {
    pub(super) fn new(width: u32, height: u32) -> Result<Self, ElectrostaticError> {
        if width == 0 || height == 0 {
            return Err(ElectrostaticError::EmptyGrid);
        }
        let width = usize::try_from(width).map_err(|_| ElectrostaticError::GridTooLarge)?;
        let height = usize::try_from(height).map_err(|_| ElectrostaticError::GridTooLarge)?;
        let grid_len = width
            .checked_mul(height)
            .ok_or(ElectrostaticError::GridTooLarge)?;
        let mut planner = DctPlanner::new();
        let transform_x = OrthonormalDct::new(width, &mut planner);
        let transform_y = OrthonormalDct::new(height, &mut planner);
        let mut inverse_eigenvalues = Vec::with_capacity(grid_len);
        let width_f64 = usize_to_f64(width)?;
        let height_f64 = usize_to_f64(height)?;
        for ky in 0..height {
            let vertical_frequency = usize_to_f64(ky)?;
            let lambda_y = 4.0 * (PI * vertical_frequency / (2.0 * height_f64)).sin().powi(2);
            for kx in 0..width {
                let horizontal_frequency = usize_to_f64(kx)?;
                let lambda_x = 4.0
                    * (PI * horizontal_frequency / (2.0 * width_f64))
                        .sin()
                        .powi(2);
                let lambda = lambda_x + lambda_y;
                inverse_eigenvalues.push(if kx == 0 && ky == 0 {
                    0.0
                } else {
                    lambda.recip()
                });
            }
        }
        Ok(Self {
            width,
            height,
            transform_x,
            transform_y,
            inverse_eigenvalues,
        })
    }

    pub(super) const fn width(&self) -> usize {
        self.width
    }

    pub(super) const fn height(&self) -> usize {
        self.height
    }

    /// Solves `-Laplacian(phi) = rho` after removing the density DC term.
    pub(super) fn solve(
        &self,
        density: &[f64],
    ) -> Result<ElectrostaticSolution, ElectrostaticError> {
        let expected = self.width * self.height;
        if density.len() != expected {
            return Err(ElectrostaticError::DensityLength {
                expected,
                actual: density.len(),
            });
        }
        if let Some(index) = density.iter().position(|entry| !entry.is_finite()) {
            return Err(ElectrostaticError::NonFiniteDensity { index });
        }
        let count = usize_to_f64(expected)?;
        let mean = density.iter().sum::<f64>() / count;
        let centered = density
            .iter()
            .map(|&entry| entry - mean)
            .collect::<Vec<_>>();
        let mut transformed = centered.clone();

        let mut scratch_x = self.transform_x.scratch();
        for row in transformed.chunks_exact_mut(self.width) {
            self.transform_x.forward(row, &mut scratch_x);
        }

        // Transform, solve, and invert one frequency column at a time. This
        // keeps only one height-sized temporary while preserving a stable
        // row-major traversal for every field.
        let mut column = vec![0.0; self.height];
        let mut scratch_y = self.transform_y.scratch();
        let mut potential = vec![0.0; expected];
        for kx in 0..self.width {
            for (y, value) in column.iter_mut().enumerate() {
                *value = transformed[y * self.width + kx];
            }
            self.transform_y.forward(&mut column, &mut scratch_y);
            for (ky, value) in column.iter_mut().enumerate() {
                *value *= self.inverse_eigenvalues[ky * self.width + kx];
            }
            self.transform_y.inverse(&mut column, &mut scratch_y);
            for (y, &value) in column.iter().enumerate() {
                potential[y * self.width + kx] = value;
            }
        }
        for row in potential.chunks_exact_mut(self.width) {
            self.transform_x.inverse(row, &mut scratch_x);
        }
        let energy = 0.5 * dot(&centered, &potential);
        Ok(ElectrostaticSolution { potential, energy })
    }

    /// Matrix-form oracle for validating the fast transform in tests.
    #[cfg(test)]
    fn solve_reference(
        &self,
        density: &[f64],
    ) -> Result<ElectrostaticSolution, ElectrostaticError> {
        let basis_x = orthonormal_dct2_basis(self.width)?;
        let basis_y = orthonormal_dct2_basis(self.height)?;
        self.solve_reference_with_bases(density, &basis_x, &basis_y)
    }

    #[cfg(test)]
    fn solve_reference_with_bases(
        &self,
        density: &[f64],
        basis_x: &[f64],
        basis_y: &[f64],
    ) -> Result<ElectrostaticSolution, ElectrostaticError> {
        let expected = self.width * self.height;
        if density.len() != expected {
            return Err(ElectrostaticError::DensityLength {
                expected,
                actual: density.len(),
            });
        }
        if let Some(index) = density.iter().position(|entry| !entry.is_finite()) {
            return Err(ElectrostaticError::NonFiniteDensity { index });
        }
        debug_assert_eq!(basis_x.len(), self.width * self.width);
        debug_assert_eq!(basis_y.len(), self.height * self.height);
        let count = usize_to_f64(expected)?;
        let mean = density.iter().sum::<f64>() / count;
        let centered = density
            .iter()
            .map(|&entry| entry - mean)
            .collect::<Vec<_>>();

        let mut row_spectrum = vec![0.0; expected];
        for y in 0..self.height {
            let row = &centered[y * self.width..(y + 1) * self.width];
            for kx in 0..self.width {
                let basis = &basis_x[kx * self.width..(kx + 1) * self.width];
                row_spectrum[y * self.width + kx] = dot(basis, row);
            }
        }

        let mut spectrum = vec![0.0; expected];
        for ky in 0..self.height {
            let basis = &basis_y[ky * self.height..(ky + 1) * self.height];
            for kx in 0..self.width {
                let mut coefficient = 0.0;
                for y in 0..self.height {
                    coefficient += basis[y] * row_spectrum[y * self.width + kx];
                }
                let index = ky * self.width + kx;
                spectrum[index] = coefficient * self.inverse_eigenvalues[index];
            }
        }

        let mut column_inverse = vec![0.0; expected];
        for y in 0..self.height {
            for kx in 0..self.width {
                let mut value = 0.0;
                for ky in 0..self.height {
                    value += basis_y[ky * self.height + y] * spectrum[ky * self.width + kx];
                }
                column_inverse[y * self.width + kx] = value;
            }
        }
        let mut potential = vec![0.0; expected];
        for y in 0..self.height {
            for x in 0..self.width {
                let mut value = 0.0;
                for kx in 0..self.width {
                    value += basis_x[kx * self.width + x] * column_inverse[y * self.width + kx];
                }
                potential[y * self.width + x] = value;
            }
        }
        let energy = 0.5 * dot(&centered, &potential);
        Ok(ElectrostaticSolution { potential, energy })
    }
}

#[cfg(test)]
fn orthonormal_dct2_basis(length: usize) -> Result<Vec<f64>, ElectrostaticError> {
    let length_f64 = usize_to_f64(length)?;
    let mut basis = Vec::with_capacity(
        length
            .checked_mul(length)
            .ok_or(ElectrostaticError::GridTooLarge)?,
    );
    for frequency in 0..length {
        let frequency_f64 = usize_to_f64(frequency)?;
        let scale = if frequency == 0 {
            length_f64.recip().sqrt()
        } else {
            (2.0 / length_f64).sqrt()
        };
        for coordinate in 0..length {
            let coordinate_f64 = usize_to_f64(coordinate)?;
            basis.push(scale * (PI * frequency_f64 * (coordinate_f64 + 0.5) / length_f64).cos());
        }
    }
    Ok(basis)
}

fn usize_to_f64(value: usize) -> Result<f64, ElectrostaticError> {
    let value = u32::try_from(value).map_err(|_| ElectrostaticError::GridTooLarge)?;
    Ok(f64::from(value))
}

fn dot(left: &[f64], right: &[f64]) -> f64 {
    debug_assert_eq!(left.len(), right.len());
    left.iter().zip(right).map(|(&a, &b)| a * b).sum()
}

/// Deposits one point charge with a C1-continuous quadratic B-spline stencil.
///
/// Folding stencil entries outside the device onto its boundary preserves both
/// total charge and the derivative sum.  Using the same basis for potential
/// sampling makes the returned electrostatic force the exact energy gradient.
fn deposit_quadratic_bspline(
    density: &mut [f64],
    width: usize,
    height: usize,
    x: f64,
    y: f64,
    charge: f64,
) {
    assert_eq!(density.len(), width * height);
    let x_axis = quadratic_bspline_axis(width, x);
    let y_axis = quadratic_bspline_axis(height, y);
    for row in y_axis {
        for column in x_axis {
            density[row.index * width + column.index] += charge * column.weight * row.weight;
        }
    }
}

/// Samples a potential and its exact quadratic B-spline position derivative.
fn sample_quadratic_bspline_with_gradient(
    potential: &[f64],
    width: usize,
    height: usize,
    x: f64,
    y: f64,
) -> (f64, f64, f64) {
    assert_eq!(potential.len(), width * height);
    let x_axis = quadratic_bspline_axis(width, x);
    let y_axis = quadratic_bspline_axis(height, y);
    let mut value = 0.0;
    let mut gradient_x = 0.0;
    let mut gradient_y = 0.0;
    for row in y_axis {
        for column in x_axis {
            let potential = potential[row.index * width + column.index];
            value += column.weight * row.weight * potential;
            gradient_x += column.derivative * row.weight * potential;
            gradient_y += column.weight * row.derivative * potential;
        }
    }
    (value, gradient_x, gradient_y)
}

#[derive(Clone, Copy, Debug)]
struct QuadraticAxisEntry {
    index: usize,
    weight: f64,
    derivative: f64,
}

fn quadratic_bspline_axis(length: usize, coordinate: f64) -> [QuadraticAxisEntry; 3] {
    assert!(length != 0);
    assert!(coordinate.is_finite());
    let maximum = f64::from(u32::try_from(length - 1).expect("grid dimension fits u32"));
    let coordinate = coordinate.clamp(0.0, maximum);
    let center_coordinate = (coordinate + 0.5).floor().min(maximum);
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let center = center_coordinate as usize;
    let offset = coordinate - center_coordinate;
    [
        QuadraticAxisEntry {
            index: center.saturating_sub(1),
            weight: 0.5 * (0.5 - offset).powi(2),
            derivative: offset - 0.5,
        },
        QuadraticAxisEntry {
            index: center,
            weight: 0.75 - offset.powi(2),
            derivative: -2.0 * offset,
        },
        QuadraticAxisEntry {
            index: center.saturating_add(1).min(length - 1),
            weight: 0.5 * (0.5 + offset).powi(2),
            derivative: offset + 0.5,
        },
    ]
}

/// Deposits one point charge with a cloud-in-cell stencil.
pub(super) fn deposit_bilinear(
    density: &mut [f64],
    width: usize,
    height: usize,
    x: f64,
    y: f64,
    charge: f64,
) {
    assert_eq!(density.len(), width * height);
    let x_axis = interpolation_axis(width, x);
    let y_axis = interpolation_axis(height, y);
    for (row, wy) in [
        (y_axis.lower, 1.0 - y_axis.fraction),
        (y_axis.upper, y_axis.fraction),
    ] {
        for (column, wx) in [
            (x_axis.lower, 1.0 - x_axis.fraction),
            (x_axis.upper, x_axis.fraction),
        ] {
            density[row * width + column] += charge * wx * wy;
        }
    }
}

/// Interpolates a potential and its exact cloud-in-cell position derivative.
pub(super) fn sample_bilinear_with_gradient(
    potential: &[f64],
    width: usize,
    height: usize,
    x: f64,
    y: f64,
) -> (f64, f64, f64) {
    assert_eq!(potential.len(), width * height);
    let x_axis = interpolation_axis(width, x);
    let y_axis = interpolation_axis(height, y);
    let at = |row: usize, column: usize| potential[row * width + column];
    let p00 = at(y_axis.lower, x_axis.lower);
    let p10 = at(y_axis.lower, x_axis.upper);
    let p01 = at(y_axis.upper, x_axis.lower);
    let p11 = at(y_axis.upper, x_axis.upper);
    let tx = x_axis.fraction;
    let ty = y_axis.fraction;
    let lower = p00 + tx * (p10 - p00);
    let upper = p01 + tx * (p11 - p01);
    let value = lower + ty * (upper - lower);
    let gradient_x = (1.0 - ty) * (p10 - p00) + ty * (p11 - p01);
    let gradient_y = (1.0 - tx) * (p01 - p00) + tx * (p11 - p10);
    (value, gradient_x, gradient_y)
}

#[derive(Clone, Copy, Debug)]
struct InterpolationAxis {
    lower: usize,
    upper: usize,
    fraction: f64,
}

fn interpolation_axis(length: usize, coordinate: f64) -> InterpolationAxis {
    assert!(length != 0);
    assert!(coordinate.is_finite());
    if length == 1 {
        return InterpolationAxis {
            lower: 0,
            upper: 0,
            fraction: 0.0,
        };
    }
    let maximum = f64::from(u32::try_from(length - 1).expect("grid dimension fits u32"));
    let coordinate = coordinate.clamp(0.0, maximum);
    if coordinate >= maximum {
        return InterpolationAxis {
            lower: length - 2,
            upper: length - 1,
            fraction: 1.0,
        };
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let lower = coordinate.floor() as usize;
    InterpolationAxis {
        lower,
        upper: lower + 1,
        fraction: coordinate - f64::from(u32::try_from(lower).expect("grid coordinate fits u32")),
    }
}

#[cfg(test)]
mod tests {
    use texo_model::{Device, Point, ResourceKind};

    use super::{
        DensityMember, DensityModel, DensityUnit, ElectrostaticError, FixedOccupancy, Poisson2d,
        deposit_bilinear, deposit_quadratic_bspline, quadratic_bspline_axis,
        sample_bilinear_with_gradient, sample_quadratic_bspline_with_gradient,
    };

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "actual={actual:.17e} expected={expected:.17e} tolerance={tolerance:.3e}"
        );
    }

    fn evaluate_initial(model: &DensityModel, units: &[DensityUnit]) -> super::DensityResult {
        let fillers = model.initial_fillers(units).unwrap();
        model.evaluate(units, &fillers).unwrap()
    }

    fn apply_neumann_laplacian(width: usize, height: usize, values: &[f64]) -> Vec<f64> {
        let mut result = vec![0.0; width * height];
        for y in 0..height {
            for x in 0..width {
                let index = y * width + x;
                let mut diagonal = 0.0;
                if x != 0 {
                    result[index] -= values[index - 1];
                    diagonal += 1.0;
                }
                if x + 1 != width {
                    result[index] -= values[index + 1];
                    diagonal += 1.0;
                }
                if y != 0 {
                    result[index] -= values[index - width];
                    diagonal += 1.0;
                }
                if y + 1 != height {
                    result[index] -= values[index + width];
                    diagonal += 1.0;
                }
                result[index] += diagonal * values[index];
            }
        }
        result
    }

    #[test]
    fn constant_density_is_only_the_removed_dc_mode() {
        let solver = Poisson2d::new(7, 5).unwrap();
        let solution = solver.solve(&vec![3.25; 35]).unwrap();
        assert_close(solution.energy, 0.0, 1.0e-24);
        assert!(solution.potential.iter().all(|&entry| entry == 0.0));
    }

    #[test]
    fn spectral_solution_matches_the_direct_neumann_operator() {
        let solver = Poisson2d::new(7, 5).unwrap();
        let density = (0_u32..35)
            .map(|index| f64::from((index * 17 + 3) % 23) - 11.0)
            .collect::<Vec<_>>();
        let mean = density.iter().sum::<f64>() / 35.0;
        let expected = density.iter().map(|entry| entry - mean).collect::<Vec<_>>();
        let solution = solver.solve(&density).unwrap();
        let reconstructed = apply_neumann_laplacian(7, 5, &solution.potential);
        for (&actual, &expected) in reconstructed.iter().zip(&expected) {
            assert_close(actual, expected, 2.0e-12);
        }
    }

    #[test]
    fn manufactured_cosine_is_one_laplacian_eigenmode() {
        let solver = Poisson2d::new(11, 1).unwrap();
        let frequency = 3.0;
        let density = (0_u32..11)
            .map(|coordinate| {
                (std::f64::consts::PI * frequency * (f64::from(coordinate) + 0.5) / 11.0).cos()
            })
            .collect::<Vec<_>>();
        let eigenvalue = 4.0
            * (std::f64::consts::PI * frequency / (2.0 * 11.0))
                .sin()
                .powi(2);
        let solution = solver.solve(&density).unwrap();
        for (&actual, &rho) in solution.potential.iter().zip(&density) {
            assert_close(actual, rho / eigenvalue, 2.0e-13);
        }
    }

    #[test]
    fn cloud_in_cell_deposit_preserves_charge_at_edges_and_inside() {
        let mut density = vec![0.0; 30];
        deposit_bilinear(&mut density, 6, 5, 2.25, 3.5, 7.0);
        deposit_bilinear(&mut density, 6, 5, 5.0, 0.0, 11.0);
        assert_close(density.iter().sum(), 18.0, 1.0e-14);
    }

    #[test]
    fn interpolated_potential_gradient_matches_energy_finite_difference() {
        let solver = Poisson2d::new(13, 9).unwrap();
        let energy = |x: f64, y: f64| {
            let mut density = vec![-2.0 / 117.0; 117];
            deposit_bilinear(&mut density, 13, 9, x, y, 2.0);
            solver.solve(&density).unwrap().energy
        };
        let x = 4.25;
        let y = 5.375;
        let mut density = vec![-2.0 / 117.0; 117];
        deposit_bilinear(&mut density, 13, 9, x, y, 2.0);
        let solution = solver.solve(&density).unwrap();
        let (_, gradient_x, gradient_y) =
            sample_bilinear_with_gradient(&solution.potential, 13, 9, x, y);
        let epsilon = 1.0e-6;
        let finite_x = (energy(x + epsilon, y) - energy(x - epsilon, y)) / (2.0 * epsilon);
        let finite_y = (energy(x, y + epsilon) - energy(x, y - epsilon)) / (2.0 * epsilon);
        assert_close(2.0 * gradient_x, finite_x, 2.0e-8);
        assert_close(2.0 * gradient_y, finite_y, 2.0e-8);
    }

    #[test]
    fn quadratic_bspline_preserves_charge_and_has_zero_derivative_sum() {
        for coordinate in [0.0, 0.25, 1.0, 1.5, 3.75, 5.0] {
            let axis = quadratic_bspline_axis(6, coordinate);
            assert_close(axis.iter().map(|entry| entry.weight).sum(), 1.0, 1.0e-15);
            assert_close(
                axis.iter().map(|entry| entry.derivative).sum(),
                0.0,
                1.0e-15,
            );
        }
        let mut density = vec![0.0; 30];
        deposit_quadratic_bspline(&mut density, 6, 5, 2.25, 3.5, 7.0);
        deposit_quadratic_bspline(&mut density, 6, 5, 5.0, 0.0, 11.0);
        assert_close(density.iter().sum(), 18.0, 1.0e-14);
    }

    #[test]
    fn quadratic_bspline_energy_gradient_is_smooth_at_stencil_knots() {
        let solver = Poisson2d::new(13, 9).unwrap();
        let energy = |x: f64, y: f64| {
            let mut density = vec![-2.0 / 117.0; 117];
            deposit_quadratic_bspline(&mut density, 13, 9, x, y, 2.0);
            solver.solve(&density).unwrap().energy
        };
        for (x, y) in [(4.0, 5.0), (4.5, 5.5)] {
            let mut density = vec![-2.0 / 117.0; 117];
            deposit_quadratic_bspline(&mut density, 13, 9, x, y, 2.0);
            let solution = solver.solve(&density).unwrap();
            let (_, gradient_x, gradient_y) =
                sample_quadratic_bspline_with_gradient(&solution.potential, 13, 9, x, y);
            let epsilon = 1.0e-6;
            let finite_x = (energy(x + epsilon, y) - energy(x - epsilon, y)) / (2.0 * epsilon);
            let finite_y = (energy(x, y + epsilon) - energy(x, y - epsilon)) / (2.0 * epsilon);
            assert_close(2.0 * gradient_x, finite_x, 2.0e-8);
            assert_close(2.0 * gradient_y, finite_y, 2.0e-8);
        }
    }

    #[test]
    fn prime_width_architecture_grid_is_supported_and_repeatable() {
        let solver = Poisson2d::new(127, 96).unwrap();
        assert_eq!(solver.width(), 127);
        assert_eq!(solver.height(), 96);
        let density = (0_u32..12_192)
            .map(|index| f64::from((index * 31 + 7) % 101) - 50.0)
            .collect::<Vec<_>>();
        let first = solver.solve(&density).unwrap();
        let second = solver.solve(&density).unwrap();
        assert_eq!(first, second);
        assert!(first.energy.is_finite() && first.energy > 0.0);
    }

    #[test]
    fn fast_prime_grid_solver_matches_orthonormal_matrix_oracle() {
        let solver = Poisson2d::new(127, 96).unwrap();
        let density = (0_u32..12_192)
            .map(|index| {
                let coarse = f64::from((index * 31 + 7) % 101) - 50.0;
                let fine = f64::from((index * 13 + 5) % 17) / 17.0;
                coarse + fine
            })
            .collect::<Vec<_>>();
        let fast = solver.solve(&density).unwrap();
        let reference = solver.solve_reference(&density).unwrap();
        let potential_scale = reference
            .potential
            .iter()
            .map(|value| value.abs())
            .fold(1.0_f64, f64::max);
        for (&actual, &expected) in fast.potential.iter().zip(&reference.potential) {
            assert_close(actual, expected, 2.0e-11 * potential_scale);
        }
        assert_close(
            fast.energy,
            reference.energy,
            2.0e-12 * reference.energy.abs().max(1.0),
        );
    }

    #[test]
    #[ignore = "release-only microbenchmark; run explicitly with --ignored --nocapture"]
    fn benchmark_fast_prime_grid_solver_against_matrix_oracle() {
        let solver = Poisson2d::new(127, 96).unwrap();
        let density = (0_u32..12_192)
            .map(|index| f64::from((index * 31 + 7) % 101) - 50.0)
            .collect::<Vec<_>>();
        let basis_x = super::orthonormal_dct2_basis(127).unwrap();
        let basis_y = super::orthonormal_dct2_basis(96).unwrap();

        let fast_iterations = 100_u32;
        let fast_started = std::time::Instant::now();
        for _ in 0..fast_iterations {
            std::hint::black_box(solver.solve(std::hint::black_box(&density)).unwrap());
        }
        let fast_elapsed = fast_started.elapsed().as_secs_f64() / f64::from(fast_iterations);

        let reference_iterations = 10_u32;
        let reference_started = std::time::Instant::now();
        for _ in 0..reference_iterations {
            std::hint::black_box(
                solver
                    .solve_reference_with_bases(std::hint::black_box(&density), &basis_x, &basis_y)
                    .unwrap(),
            );
        }
        let reference_elapsed =
            reference_started.elapsed().as_secs_f64() / f64::from(reference_iterations);
        eprintln!(
            "Poisson2d 127x96 fast_us={:.3} matrix_us={:.3} speedup={:.2}x",
            fast_elapsed * 1.0e6,
            reference_elapsed * 1.0e6,
            reference_elapsed / fast_elapsed,
        );
        assert!(fast_elapsed < reference_elapsed);
    }

    fn mixed_device() -> Device {
        let mut device = Device::new("mixed", 4, 3).unwrap();
        for x in 0..4 {
            device
                .add_bel(format!("LUT{x}"), ResourceKind::Lut(4), Point::new(x, 0))
                .unwrap();
            device
                .add_bel(format!("FF{x}"), ResourceKind::Register, Point::new(x, 1))
                .unwrap();
        }
        device
    }

    #[test]
    fn fixed_occupancy_is_subtracted_from_its_exact_kind_and_point() {
        let model = DensityModel::new(
            &mixed_device(),
            &[FixedOccupancy {
                kind: ResourceKind::Lut(4),
                point: Point::new(0, 0),
            }],
        )
        .unwrap();
        assert_eq!(model.capacity[&ResourceKind::Lut(4)][0], 0);
        assert_eq!(model.capacity[&ResourceKind::Register][4], 1);
        assert!(matches!(
            DensityModel::new(
                &mixed_device(),
                &[
                    FixedOccupancy {
                        kind: ResourceKind::Lut(4),
                        point: Point::new(0, 0),
                    },
                    FixedOccupancy {
                        kind: ResourceKind::Lut(4),
                        point: Point::new(0, 0),
                    },
                ],
            ),
            Err(ElectrostaticError::FixedCapacityExhausted {
                kind: ResourceKind::Lut(4),
                point: Point { x: 0, y: 0 },
            })
        ));
    }

    #[test]
    fn lut_and_register_fields_are_separate_for_one_atomic_unit() {
        let model = DensityModel::new(&mixed_device(), &[]).unwrap();
        let unit = DensityUnit {
            origin_x: 1.25,
            origin_y: 0.25,
            members: vec![
                DensityMember {
                    kind: ResourceKind::Lut(4),
                    offset_x: 0.0,
                    offset_y: 0.0,
                    charge: 1.0,
                },
                DensityMember {
                    kind: ResourceKind::Register,
                    offset_x: 0.5,
                    offset_y: 0.5,
                    charge: 1.0,
                },
            ],
        };
        let units = [unit];
        let result = evaluate_initial(&model, &units);
        let repeated = evaluate_initial(&model, &units);
        assert_eq!(result, repeated);
        assert_eq!(
            result
                .fields
                .iter()
                .map(|field| field.kind)
                .collect::<Vec<_>>(),
            vec![ResourceKind::Lut(4), ResourceKind::Register]
        );
        assert_eq!(result.fields[0].unit_gradients.len(), 1);
        assert_eq!(result.fields[0].real_charge, 1);
        assert_eq!(result.fields[1].real_charge, 1);
        assert_eq!(result.fields[0].filler_charge.to_bits(), 3.0_f64.to_bits());
        assert_eq!(result.fields[1].filler_charge.to_bits(), 3.0_f64.to_bits());
        assert!(result.fields.iter().all(|field| field.energy.is_finite()));
    }

    #[test]
    fn model_gradient_matches_total_energy_finite_difference() {
        let model = DensityModel::new(&mixed_device(), &[]).unwrap();
        let make_unit = |x| DensityUnit {
            origin_x: x,
            origin_y: 0.4,
            members: vec![
                DensityMember {
                    kind: ResourceKind::Lut(4),
                    offset_x: 0.0,
                    offset_y: 0.0,
                    charge: 1.0,
                },
                DensityMember {
                    kind: ResourceKind::Register,
                    offset_x: 0.3,
                    offset_y: 0.4,
                    charge: 1.0,
                },
            ],
        };
        let energy = |x| {
            evaluate_initial(&model, &[make_unit(x)])
                .fields
                .iter()
                .map(|field| field.energy)
                .sum::<f64>()
        };
        let x = 1.2;
        let result = evaluate_initial(&model, &[make_unit(x)]);
        let epsilon = 1.0e-6;
        let finite = (energy(x + epsilon) - energy(x - epsilon)) / (2.0 * epsilon);
        let gradient = result
            .fields
            .iter()
            .map(|field| field.unit_gradients[0].0)
            .sum::<f64>();
        assert_close(gradient, finite, 2.0e-8);
    }

    #[test]
    fn capacity_background_and_real_deposit_preserve_charge() {
        let model = DensityModel::new(&mixed_device(), &[]).unwrap();
        let units = (0_u32..3)
            .map(|x| DensityUnit {
                origin_x: f64::from(x) + 0.25,
                origin_y: 0.5,
                members: vec![DensityMember {
                    kind: ResourceKind::Lut(4),
                    offset_x: 0.0,
                    offset_y: 0.0,
                    charge: 1.0,
                }],
            })
            .collect::<Vec<_>>();
        let first = evaluate_initial(&model, &units);
        let second = evaluate_initial(&model, &units);
        assert_eq!(first, second);
        assert_close(first.fields[0].density.iter().sum(), 0.0, 1.0e-14);
        assert_close(first.fields[0].net_charge, 0.0, 1.0e-14);
        assert_eq!(first.fields[0].filler_charge.to_bits(), 1.0_f64.to_bits());
        assert!(first.fields[0].normalized_positive_overflow <= 1.0);
    }

    #[test]
    fn physical_overflow_is_independent_of_filler_coordinates() {
        let model = DensityModel::new(&mixed_device(), &[]).unwrap();
        let units = [DensityUnit {
            origin_x: 0.0,
            origin_y: 0.0,
            members: vec![DensityMember {
                kind: ResourceKind::Lut(4),
                offset_x: 0.0,
                offset_y: 0.0,
                charge: 1.0,
            }],
        }];
        let fillers = model.initial_fillers(&units).unwrap();
        let baseline = model.evaluate(&units, &fillers).unwrap();
        let mut moved = fillers.clone();
        for (index, filler) in moved.iter_mut().enumerate() {
            filler.x = if index % 2 == 0 { 0.0 } else { 3.0 };
            filler.y = 2.0;
        }
        let moved = model.evaluate(&units, &moved).unwrap();

        assert_eq!(
            baseline.fields[0].normalized_positive_overflow.to_bits(),
            moved.fields[0].normalized_positive_overflow.to_bits()
        );
        assert_ne!(
            baseline.fields[0].energy.to_bits(),
            moved.fields[0].energy.to_bits()
        );
    }

    #[test]
    fn separating_overlapping_physical_cells_reduces_overflow() {
        let model = DensityModel::new(&mixed_device(), &[]).unwrap();
        let unit = |x| DensityUnit {
            origin_x: x,
            origin_y: 0.0,
            members: vec![DensityMember {
                kind: ResourceKind::Lut(4),
                offset_x: 0.0,
                offset_y: 0.0,
                charge: 1.0,
            }],
        };
        let overlapping = [unit(0.0), unit(0.0)];
        let separated = [unit(0.0), unit(1.0)];
        let fillers = model.initial_fillers(&overlapping).unwrap();
        let overlapping = model.evaluate(&overlapping, &fillers).unwrap();
        let separated = model.evaluate(&separated, &fillers).unwrap();

        assert!(
            separated.fields[0].normalized_positive_overflow
                < overlapping.fields[0].normalized_positive_overflow
        );
    }

    #[test]
    fn four_lut_carry_like_macro_is_one_rigid_variable() {
        let model = DensityModel::new(&mixed_device(), &[]).unwrap();
        let unit = DensityUnit {
            origin_x: 0.25,
            origin_y: 0.0,
            members: (0_u32..4)
                .map(|offset| DensityMember {
                    kind: ResourceKind::Lut(4),
                    offset_x: f64::from(offset),
                    offset_y: 0.0,
                    charge: 1.0,
                })
                .collect(),
        };
        let result = evaluate_initial(&model, &[unit]);
        assert_eq!(result.fields[0].unit_gradients.len(), 1);
        assert_eq!(result.fields.len(), 1);
        assert_eq!(result.fields[0].real_charge, 4);
        assert_eq!(result.fields[0].filler_charge.to_bits(), 0.0_f64.to_bits());
        assert_close(result.fields[0].density.iter().sum(), 0.0, 1.0e-14);
    }

    #[test]
    fn adjusted_carry_macro_and_register_share_one_origin_across_density_fields() {
        let mut device = Device::new("carry-with-register", 8, 1).unwrap();
        for x in 0..8 {
            let point = Point::new(x, 0);
            device
                .add_bel(format!("LUT{x}"), ResourceKind::Lut(4), point)
                .unwrap();
            device
                .add_bel(format!("FF{x}"), ResourceKind::Register, point)
                .unwrap();
        }
        let model = DensityModel::new(&device, &[]).unwrap();
        let make_unit = |origin_x| DensityUnit {
            origin_x,
            origin_y: 0.0,
            members: (0_u32..4)
                .map(|offset| DensityMember {
                    kind: ResourceKind::Lut(4),
                    offset_x: f64::from(offset),
                    offset_y: 0.0,
                    charge: 1.0 + 0.1 * f64::from(offset),
                })
                .chain([DensityMember {
                    kind: ResourceKind::Register,
                    offset_x: 2.0,
                    offset_y: 0.0,
                    charge: 1.5,
                }])
                .collect(),
        };
        let evaluate = |origin_x| evaluate_initial(&model, &[make_unit(origin_x)]);
        let origin_x = 2.0;
        let result = evaluate(origin_x);
        assert_eq!(
            result
                .fields
                .iter()
                .map(|field| (field.kind, field.real_charge))
                .collect::<Vec<_>>(),
            vec![(ResourceKind::Lut(4), 4), (ResourceKind::Register, 1),]
        );
        assert_close(result.fields[0].filler_charge, 3.4, 1.0e-14);
        assert_close(result.fields[1].filler_charge, 6.5, 1.0e-14);
        assert!(
            result
                .fields
                .iter()
                .all(|field| field.unit_gradients.len() == 1)
        );

        let epsilon = 1.0e-6;
        let total_energy = |x| {
            evaluate(x)
                .fields
                .iter()
                .map(|field| field.energy)
                .sum::<f64>()
        };
        let finite_difference =
            (total_energy(origin_x + epsilon) - total_energy(origin_x - epsilon)) / (2.0 * epsilon);
        let shared_origin_gradient = result
            .fields
            .iter()
            .map(|field| field.unit_gradients[0].0)
            .sum::<f64>();
        assert_close(shared_origin_gradient, finite_difference, 2.0e-8);
    }

    #[test]
    fn external_optimizer_positions_exactly_match_materialized_rigid_macros() {
        let model = DensityModel::new(&mixed_device(), &[]).unwrap();
        let units = vec![DensityUnit {
            origin_x: 0.25,
            origin_y: 0.5,
            members: vec![
                DensityMember {
                    kind: ResourceKind::Lut(4),
                    offset_x: 0.0,
                    offset_y: 0.0,
                    charge: 1.25,
                },
                DensityMember {
                    kind: ResourceKind::Register,
                    offset_x: 0.5,
                    offset_y: 0.25,
                    charge: 1.5,
                },
            ],
        }];
        let fillers = model.initial_fillers(&units).unwrap();
        let unit_positions = [(1.25, 0.75)];
        let filler_positions = fillers
            .iter()
            .enumerate()
            .map(|(index, filler)| {
                (
                    filler.x + 0.01 * f64::from(u32::try_from(index % 3).unwrap()),
                    filler.y + 0.02 * f64::from(u32::try_from(index % 2).unwrap()),
                )
            })
            .collect::<Vec<_>>();

        let external = model
            .evaluate_with_positions(
                &units,
                &fillers,
                |index| unit_positions[index],
                |index| filler_positions[index],
            )
            .unwrap();
        let mut materialized_units = units.clone();
        for (unit, &(x, y)) in materialized_units.iter_mut().zip(&unit_positions) {
            unit.origin_x = x;
            unit.origin_y = y;
        }
        let mut materialized_fillers = fillers.clone();
        for (filler, &(x, y)) in materialized_fillers.iter_mut().zip(&filler_positions) {
            filler.x = x;
            filler.y = y;
        }

        assert_eq!(
            external,
            model
                .evaluate(&materialized_units, &materialized_fillers)
                .unwrap()
        );
    }

    #[test]
    fn legal_site_charges_exactly_cancel_the_smoothed_capacity() {
        let model = DensityModel::new(&mixed_device(), &[]).unwrap();
        let units = [DensityUnit {
            origin_x: 0.0,
            origin_y: 0.0,
            members: (0_u32..4)
                .map(|offset| DensityMember {
                    kind: ResourceKind::Lut(4),
                    offset_x: f64::from(offset),
                    offset_y: 0.0,
                    charge: 1.0,
                })
                .collect(),
        }];
        let result = evaluate_initial(&model, &units);
        assert_eq!(result.fields[0].filler_charge.to_bits(), 0.0_f64.to_bits());
        assert!(result.fields[0].density.iter().all(|&entry| entry == 0.0));
        assert_close(result.fields[0].energy, 0.0, 0.0);
        assert_close(result.fields[0].normalized_positive_overflow, 0.0, 0.0);
    }

    #[test]
    fn initial_fillers_are_capacity_proportional_and_kind_separated() {
        let model = DensityModel::new(&mixed_device(), &[]).unwrap();
        let units = [DensityUnit {
            origin_x: 1.0,
            origin_y: 0.5,
            members: vec![
                DensityMember {
                    kind: ResourceKind::Lut(4),
                    offset_x: 0.0,
                    offset_y: 0.0,
                    charge: 1.0,
                },
                DensityMember {
                    kind: ResourceKind::Register,
                    offset_x: 0.0,
                    offset_y: 0.0,
                    charge: 1.0,
                },
            ],
        }];
        let fillers = model.initial_fillers(&units).unwrap();
        assert_eq!(fillers.len(), 8);
        for kind in [ResourceKind::Lut(4), ResourceKind::Register] {
            let field = fillers
                .iter()
                .filter(|filler| filler.kind == kind)
                .collect::<Vec<_>>();
            assert_eq!(field.len(), 4);
            assert!(
                field
                    .iter()
                    .all(|filler| (filler.charge - 0.75).abs() <= f64::EPSILON)
            );
            assert_close(field.iter().map(|filler| filler.charge).sum(), 3.0, 0.0);
        }
        let result = model.evaluate(&units, &fillers).unwrap();
        assert!(result.fields.iter().all(|field| field.net_charge == 0.0));
    }

    #[test]
    fn adjusted_physical_area_consumes_only_filler_charge() {
        let model = DensityModel::new(&mixed_device(), &[]).unwrap();
        let units = [DensityUnit {
            origin_x: 1.25,
            origin_y: 0.0,
            members: vec![DensityMember {
                kind: ResourceKind::Lut(4),
                offset_x: 0.0,
                offset_y: 0.0,
                charge: 1.75,
            }],
        }];
        let fillers = model.initial_fillers(&units).unwrap();
        assert_close(
            fillers.iter().map(|filler| filler.charge).sum(),
            2.25,
            1.0e-15,
        );
        let result = model.evaluate(&units, &fillers).unwrap();
        assert_close(result.fields[0].real_area, 1.75, 0.0);
        assert_close(result.fields[0].filler_charge, 2.25, 0.0);
        assert_close(result.fields[0].net_charge, 0.0, 1.0e-14);
    }

    #[test]
    fn filler_gradient_matches_energy_finite_difference() {
        let model = DensityModel::new(&mixed_device(), &[]).unwrap();
        let units = [DensityUnit {
            origin_x: 2.25,
            origin_y: 0.5,
            members: vec![DensityMember {
                kind: ResourceKind::Lut(4),
                offset_x: 0.0,
                offset_y: 0.0,
                charge: 1.0,
            }],
        }];
        let fillers = model.initial_fillers(&units).unwrap();
        let filler_index = fillers
            .iter()
            .position(|filler| {
                filler.kind == ResourceKind::Lut(4) && filler.x.to_bits() == 1.0_f64.to_bits()
            })
            .unwrap();
        let energy = |x: f64| {
            let mut trial = fillers.clone();
            trial[filler_index].x = x;
            model.evaluate(&units, &trial).unwrap().fields[0].energy
        };
        let result = model.evaluate(&units, &fillers).unwrap();
        let (_, gradient_x, _) = result.fields[0]
            .filler_gradients
            .iter()
            .find(|(index, _, _)| *index == filler_index)
            .copied()
            .unwrap();
        let epsilon = 1.0e-6;
        let finite = (energy(1.0 + epsilon) - energy(1.0 - epsilon)) / (2.0 * epsilon);
        assert_close(gradient_x, finite, 2.0e-8);
    }

    #[test]
    fn capacity_shortage_and_malformed_units_are_typed_errors() {
        let model = DensityModel::new(&mixed_device(), &[]).unwrap();
        let unit = || DensityUnit {
            origin_x: 0.0,
            origin_y: 0.0,
            members: vec![DensityMember {
                kind: ResourceKind::Memory,
                offset_x: 0.0,
                offset_y: 0.0,
                charge: 1.0,
            }],
        };
        assert_eq!(
            model.initial_fillers(&[unit()]),
            Err(ElectrostaticError::InsufficientCapacity {
                kind: ResourceKind::Memory,
                demand: 1,
                available: 0,
            })
        );
        assert_eq!(
            model.initial_fillers(&[DensityUnit {
                origin_x: 0.0,
                origin_y: 0.0,
                members: Vec::new(),
            }]),
            Err(ElectrostaticError::EmptyUnit { unit: 0 })
        );

        let real = [DensityUnit {
            origin_x: 0.0,
            origin_y: 0.0,
            members: vec![DensityMember {
                kind: ResourceKind::Lut(4),
                offset_x: 0.0,
                offset_y: 0.0,
                charge: 1.0,
            }],
        }];
        let mut fillers = model.initial_fillers(&real).unwrap();
        fillers[0].charge = -1.0;
        assert_eq!(
            model.evaluate(&real, &fillers),
            Err(ElectrostaticError::NegativeFillerCharge { filler: 0 })
        );
        let mut fillers = model.initial_fillers(&real).unwrap();
        fillers[0].charge += 1.0;
        assert!(matches!(
            model.evaluate(&real, &fillers),
            Err(ElectrostaticError::FillerChargeMismatch {
                kind: ResourceKind::Lut(4),
                ..
            })
        ));
    }
}
