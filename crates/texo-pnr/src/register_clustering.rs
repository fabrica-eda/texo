//! ECP5-compatible elfPlace register-clustering area estimation.

use std::collections::BTreeMap;

use rayon::prelude::*;
use texo_model::CellId;

use super::RegisterControlSet;

#[derive(Clone, Copy, Debug)]
pub(super) struct MovableRegister {
    pub(super) cell: CellId,
    pub(super) x: f64,
    pub(super) y: f64,
}

pub(super) fn clustering_areas(
    registers: &[MovableRegister],
    controls: &BTreeMap<CellId, RegisterControlSet>,
    movable_instance_count: usize,
) -> Option<BTreeMap<CellId, f64>> {
    if registers.is_empty() || movable_instance_count == 0 {
        return Some(BTreeMap::new());
    }
    let count = u32::try_from(movable_instance_count).ok()?;
    let sigma = (1.0e-5 * f64::from(count)).sqrt();
    if !sigma.is_finite() || sigma <= 0.0 {
        return None;
    }
    let half_window = 2.5 * sigma;
    let mut registers_by_clock_lsr = BTreeMap::<(u64, u64), Vec<MovableRegister>>::new();
    for &register in registers {
        let control = controls.get(&register.cell)?;
        registers_by_clock_lsr
            .entry(control.clock_lsr)
            .or_default()
            .push(register);
    }
    let areas = registers
        .par_iter()
        .map(|&register| {
            let focal = controls.get(&register.cell)?;
            let mut expected_by_ce = BTreeMap::<u64, f64>::new();
            for &other in &registers_by_clock_lsr[&focal.clock_lsr] {
                let other_control = controls.get(&other.cell)?;
                let probability_x = gaussian_interval_probability(
                    register.x - half_window,
                    register.x + half_window,
                    other.x,
                    sigma,
                );
                let probability_y = gaussian_interval_probability(
                    register.y - half_window,
                    register.y + half_window,
                    other.y,
                    sigma,
                );
                *expected_by_ce.entry(other_control.ce).or_default() +=
                    probability_x * probability_y;
            }
            let expected_focal = expected_by_ce.get(&focal.ce).copied().unwrap_or(0.0);
            if !(expected_focal.is_finite() && expected_focal > 0.0) {
                return None;
            }
            let slice_demands = expected_by_ce
                .values()
                .copied()
                .map(|expected| soft_division_ceiling(expected, 2.0))
                .collect::<Vec<_>>();
            let total_slice_demand = slice_demands.iter().sum::<f64>();
            let focal_slice_demand = soft_division_ceiling(expected_focal, 2.0);
            let area = 8.0 / expected_focal * focal_slice_demand / total_slice_demand
                * soft_division_ceiling(total_slice_demand, 4.0);
            if !(area.is_finite() && area >= 0.0) {
                return None;
            }
            Some((register.cell, area))
        })
        .collect::<Option<Vec<_>>>()?;
    Some(areas.into_iter().collect())
}

fn gaussian_interval_probability(low: f64, high: f64, mean: f64, sigma: f64) -> f64 {
    let scale = sigma * std::f64::consts::SQRT_2;
    (0.5 * (libm::erf((high - mean) / scale) - libm::erf((low - mean) / scale))).clamp(0.0, 1.0)
}

fn soft_division_ceiling(value: f64, divisor: f64) -> f64 {
    let quotient = value / divisor;
    let floor = quotient.floor();
    if quotient - floor < 1.0 / divisor {
        value + (1.0 - divisor) * floor
    } else {
        quotient.ceil()
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use texo_model::CellId;

    use super::{MovableRegister, clustering_areas, soft_division_ceiling};
    use crate::RegisterControlSet;

    fn control(cell: usize, clock: u64, lsr: u64, ce: u64) -> RegisterControlSet {
        RegisterControlSet {
            cell: CellId(cell),
            clock_lsr: (clock, lsr),
            ce,
        }
    }

    #[test]
    fn soft_ceiling_linearizes_only_the_start_of_each_step() {
        assert_eq!(soft_division_ceiling(2.0, 2.0).to_bits(), 1.0_f64.to_bits());
        assert_eq!(soft_division_ceiling(2.5, 2.0).to_bits(), 1.5_f64.to_bits());
        assert_eq!(soft_division_ceiling(3.5, 2.0).to_bits(), 2.0_f64.to_bits());
        assert_eq!(soft_division_ceiling(8.0, 4.0).to_bits(), 2.0_f64.to_bits());
    }

    #[test]
    fn ce_partition_changes_charge_but_other_tile_controls_do_not() {
        let registers = (0..4)
            .map(|cell| MovableRegister {
                cell: CellId(cell),
                x: 10.0,
                y: 10.0,
            })
            .collect::<Vec<_>>();
        let controls = [
            control(0, 1, 2, 3),
            control(1, 1, 2, 3),
            control(2, 1, 2, 4),
            control(3, 9, 2, 3),
        ]
        .into_iter()
        .map(|set| (set.cell, set))
        .collect::<BTreeMap<_, _>>();
        let areas = clustering_areas(&registers, &controls, registers.len()).unwrap();
        assert_eq!(areas[&CellId(0)].to_bits(), areas[&CellId(1)].to_bits());
        let compatible_controls = [
            control(0, 1, 2, 3),
            control(1, 1, 2, 3),
            control(2, 1, 2, 3),
            control(3, 9, 2, 3),
        ]
        .into_iter()
        .map(|set| (set.cell, set))
        .collect::<BTreeMap<_, _>>();
        let compatible =
            clustering_areas(&registers, &compatible_controls, registers.len()).unwrap();
        assert_ne!(
            areas[&CellId(0)].to_bits(),
            compatible[&CellId(0)].to_bits()
        );
        assert_eq!(
            areas[&CellId(3)].to_bits(),
            compatible[&CellId(3)].to_bits()
        );
    }

    #[test]
    fn rigid_members_are_not_duplicated_by_the_charge_estimator() {
        let registers = [MovableRegister {
            cell: CellId(7),
            x: 3.25,
            y: 4.5,
        }];
        let set = control(7, 1, 2, 3);
        let areas = clustering_areas(&registers, &BTreeMap::from([(set.cell, set)]), 2).unwrap();
        assert_eq!(areas.len(), 1);
        assert!((areas[&CellId(7)] - 8.0).abs() < 1.0e-12);
    }

    #[test]
    fn fixed_control_records_are_excluded_from_movable_expectation() {
        let registers = [MovableRegister {
            cell: CellId(1),
            x: 2.0,
            y: 3.0,
        }];
        let movable = control(1, 4, 5, 6);
        let fixed = control(99, 4, 5, 7);
        let with_fixed_metadata = BTreeMap::from([(movable.cell, movable), (fixed.cell, fixed)]);
        let without_fixed_metadata = BTreeMap::from([(movable.cell, movable)]);
        let with_fixed = clustering_areas(&registers, &with_fixed_metadata, 2).unwrap();
        let without_fixed = clustering_areas(&registers, &without_fixed_metadata, 2).unwrap();
        assert_eq!(with_fixed, without_fixed);
    }
}
