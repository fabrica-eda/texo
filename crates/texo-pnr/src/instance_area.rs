//! Charge-conserving elfPlace instance-area adjustment.

#![cfg_attr(not(test), allow(dead_code))]

use std::collections::BTreeMap;

use texo_model::ResourceKind;

use super::routing_demand::RoutingDemandBin;

/// One resource-specific density charge attached to a placement-unit origin.
///
/// Members with the same `unit` remain one optimizer variable.  In particular,
/// a rigid LUT/FF or carry macro is not split merely because its members use
/// different electrostatic fields.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct InstanceAreaMember {
    pub(super) unit: usize,
    pub(super) kind: ResourceKind,
    pub(super) current_area: f64,
    pub(super) routability_area: f64,
    pub(super) pin_area: f64,
    pub(super) clustering_area: f64,
}

/// One filler charge in a resource-specific electrostatic field.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct InstanceAreaFiller {
    pub(super) kind: ResourceKind,
    pub(super) current_area: f64,
}

/// Resource-specific scale from elfPlace equation (24).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct ResourceAreaScale {
    pub(super) kind: ResourceKind,
    pub(super) scale: f64,
}

/// A transactional area-adjustment proposal.
///
/// When `requires_optimizer_reset` is true, callers must install all member and
/// filler areas together, reevaluate density, and rebuild both the optimizer
/// and density multipliers.  Continuing an old Nesterov/multiplier state after
/// charge redistribution is not a valid operation.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct InstanceAreaAdjustment {
    pub(super) member_areas: Vec<f64>,
    pub(super) filler_areas: Vec<f64>,
    pub(super) resource_scales: Vec<ResourceAreaScale>,
    pub(super) requires_optimizer_reset: bool,
}

/// Invalid input to instance-area adjustment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum InstanceAreaError {
    InvalidMemberArea { member: usize },
    InvalidFillerArea { filler: usize },
    PostconditionViolation,
    ArithmeticOverflow,
}

/// elfPlace equation (29), before equation (23) prevents deflation.
pub(super) fn routability_optimized_area(
    current_area: f64,
    horizontal_utilization: f64,
    vertical_utilization: f64,
) -> Result<f64, InstanceAreaError> {
    if !current_area.is_finite()
        || current_area < 0.0
        || !horizontal_utilization.is_finite()
        || horizontal_utilization < 0.0
        || !vertical_utilization.is_finite()
        || vertical_utilization < 0.0
    {
        return Err(InstanceAreaError::ArithmeticOverflow);
    }
    let utilization = horizontal_utilization.max(vertical_utilization);
    let factor = (utilization * utilization).min(2.0);
    let area = current_area * factor;
    area.is_finite()
        .then_some(area)
        .ok_or(InstanceAreaError::ArithmeticOverflow)
}

/// Computes equation (29) from one architecture-capacity RUDY bin.
///
/// A direction with demand but no usable channel capacity is treated as a
/// hotspot and receives equation (29)'s maximum inflation.  A direction with
/// neither demand nor capacity contributes zero utilization.
pub(super) fn routability_optimized_area_from_bin(
    current_area: f64,
    bin: &RoutingDemandBin,
) -> Result<f64, InstanceAreaError> {
    fn utilization(demand: f64, capacity: u64) -> Result<f64, InstanceAreaError> {
        const MAX_EXACT_F64_INTEGER: u64 = 1_u64 << 53;
        if !demand.is_finite() || demand < 0.0 {
            return Err(InstanceAreaError::ArithmeticOverflow);
        }
        if capacity == 0 {
            return Ok(if demand == 0.0 { 0.0 } else { f64::INFINITY });
        }
        if capacity > MAX_EXACT_F64_INTEGER {
            return Err(InstanceAreaError::ArithmeticOverflow);
        }
        let high =
            u32::try_from(capacity >> 32).map_err(|_| InstanceAreaError::ArithmeticOverflow)?;
        let low = u32::try_from(capacity & u64::from(u32::MAX))
            .map_err(|_| InstanceAreaError::ArithmeticOverflow)?;
        let capacity = f64::from(high) * 4_294_967_296.0 + f64::from(low);
        Ok(demand / capacity)
    }

    let horizontal = utilization(bin.horizontal_demand, bin.horizontal_capacity)?;
    let vertical = utilization(bin.vertical_demand, bin.vertical_capacity)?;
    if horizontal.is_infinite() || vertical.is_infinite() {
        if !current_area.is_finite() || current_area < 0.0 {
            return Err(InstanceAreaError::ArithmeticOverflow);
        }
        let area = current_area * 2.0;
        return area
            .is_finite()
            .then_some(area)
            .ok_or(InstanceAreaError::ArithmeticOverflow);
    }
    routability_optimized_area(current_area, horizontal, vertical)
}

/// elfPlace equation (30), before equation (23) prevents deflation.
pub(super) fn pin_optimized_area(
    pin_count: usize,
    unit_area_pin_capacity: f64,
    local_pin_utilization: f64,
) -> Result<f64, InstanceAreaError> {
    if !unit_area_pin_capacity.is_finite()
        || unit_area_pin_capacity <= 0.0
        || !local_pin_utilization.is_finite()
        || local_pin_utilization < 0.0
    {
        return Err(InstanceAreaError::ArithmeticOverflow);
    }
    let area = usize_to_f64(pin_count)? / unit_area_pin_capacity * local_pin_utilization.min(1.5);
    area.is_finite()
        .then_some(area)
        .ok_or(InstanceAreaError::ArithmeticOverflow)
}

/// Computes elfPlace equations (23)--(26) without mutating placement state.
///
/// The physical increase of each member is limited only by filler charge of
/// the same resource kind.  Fillers are reduced proportionally, which is the
/// charge-preserving generalization of equation (26) for non-uniform fillers.
/// This is important for FPGA capacity maps where one filler may represent
/// more BELs than another. `clustering_area` is an independent architecture
/// estimate; the transactional scaling below treats it exactly like the paper's
/// other Eq. (23) target areas.
pub(super) fn adjust_instance_areas(
    members: &[InstanceAreaMember],
    fillers: &[InstanceAreaFiller],
) -> Result<InstanceAreaAdjustment, InstanceAreaError> {
    validate_inputs(members, fillers)?;

    let requested_areas = members
        .iter()
        .map(|member| {
            member
                .routability_area
                .max(member.pin_area)
                .max(member.clustering_area)
                .max(member.current_area)
        })
        .collect::<Vec<_>>();
    let deltas = members
        .iter()
        .zip(&requested_areas)
        .map(|(member, requested)| requested - member.current_area)
        .collect::<Vec<_>>();
    let mut desired_increase = BTreeMap::<ResourceKind, f64>::new();
    for (member, &delta) in members.iter().zip(&deltas) {
        checked_add(&mut desired_increase, member.kind, delta)?;
    }
    let mut available_filler = BTreeMap::<ResourceKind, f64>::new();
    for filler in fillers {
        checked_add(&mut available_filler, filler.kind, filler.current_area)?;
    }

    let mut scales = BTreeMap::<ResourceKind, f64>::new();
    for (&kind, &desired) in &desired_increase {
        let available = available_filler.get(&kind).copied().unwrap_or(0.0);
        let scale = if desired == 0.0 {
            1.0
        } else {
            (available / desired).min(1.0)
        };
        scales.insert(kind, scale);
    }

    let target_increase = desired_increase
        .iter()
        .map(|(&kind, &desired)| {
            let available = available_filler.get(&kind).copied().unwrap_or(0.0);
            (kind, desired.min(available))
        })
        .collect::<BTreeMap<_, _>>();
    let mut last_member_by_kind = BTreeMap::new();
    for (index, member) in members.iter().enumerate() {
        last_member_by_kind.insert(member.kind, index);
    }
    let mut actual_increase = BTreeMap::<ResourceKind, f64>::new();
    let mut member_areas = Vec::with_capacity(members.len());
    for (index, ((member, &delta), &requested)) in members
        .iter()
        .zip(&deltas)
        .zip(&requested_areas)
        .enumerate()
    {
        let proportional = delta * scales[&member.kind];
        let increase = if last_member_by_kind[&member.kind] == index {
            let assigned = actual_increase.get(&member.kind).copied().unwrap_or(0.0);
            (target_increase[&member.kind] - assigned).clamp(0.0, delta)
        } else {
            proportional
        };
        let adjusted = if scales[&member.kind].to_bits() == 1.0_f64.to_bits() {
            requested
        } else {
            member.current_area + increase
        };
        if !adjusted.is_finite() {
            return Err(InstanceAreaError::ArithmeticOverflow);
        }
        checked_add(
            &mut actual_increase,
            member.kind,
            adjusted - member.current_area,
        )?;
        member_areas.push(adjusted);
    }

    let filler_areas = adjusted_filler_areas(fillers, &available_filler, &actual_increase)?;
    validate_postconditions(members, &member_areas, fillers, &filler_areas)?;
    let requires_optimizer_reset = actual_increase.values().any(|&increase| increase > 0.0);
    let resource_scales = scales
        .into_iter()
        .map(|(kind, scale)| ResourceAreaScale { kind, scale })
        .collect();
    Ok(InstanceAreaAdjustment {
        member_areas,
        filler_areas,
        resource_scales,
        requires_optimizer_reset,
    })
}

fn validate_inputs(
    members: &[InstanceAreaMember],
    fillers: &[InstanceAreaFiller],
) -> Result<(), InstanceAreaError> {
    for (index, member) in members.iter().enumerate() {
        if !member.current_area.is_finite()
            || member.current_area < 0.0
            || !member.routability_area.is_finite()
            || member.routability_area < 0.0
            || !member.pin_area.is_finite()
            || member.pin_area < 0.0
            || !member.clustering_area.is_finite()
            || member.clustering_area < 0.0
        {
            return Err(InstanceAreaError::InvalidMemberArea { member: index });
        }
    }
    for (index, filler) in fillers.iter().enumerate() {
        if !filler.current_area.is_finite() || filler.current_area < 0.0 {
            return Err(InstanceAreaError::InvalidFillerArea { filler: index });
        }
    }
    Ok(())
}

fn checked_add(
    totals: &mut BTreeMap<ResourceKind, f64>,
    kind: ResourceKind,
    value: f64,
) -> Result<(), InstanceAreaError> {
    let total = totals.entry(kind).or_default();
    *total += value;
    if !total.is_finite() {
        return Err(InstanceAreaError::ArithmeticOverflow);
    }
    Ok(())
}

fn adjusted_filler_areas(
    fillers: &[InstanceAreaFiller],
    available_filler: &BTreeMap<ResourceKind, f64>,
    actual_increase: &BTreeMap<ResourceKind, f64>,
) -> Result<Vec<f64>, InstanceAreaError> {
    if actual_increase.values().all(|&increase| increase == 0.0) {
        return Ok(fillers.iter().map(|filler| filler.current_area).collect());
    }
    let mut remaining = BTreeMap::new();
    for (&kind, &available) in available_filler {
        let increase = actual_increase.get(&kind).copied().unwrap_or(0.0);
        remaining.insert(kind, (available - increase).max(0.0));
    }

    let mut result = Vec::with_capacity(fillers.len());
    let mut assigned = BTreeMap::<ResourceKind, f64>::new();
    let mut last_by_kind = BTreeMap::new();
    for (index, filler) in fillers.iter().enumerate() {
        last_by_kind.insert(filler.kind, index);
    }
    for (index, filler) in fillers.iter().enumerate() {
        let total = available_filler[&filler.kind];
        let target = remaining[&filler.kind];
        let area = if last_by_kind[&filler.kind] == index {
            (target - assigned.get(&filler.kind).copied().unwrap_or(0.0)).max(0.0)
        } else if total == 0.0 {
            0.0
        } else {
            filler.current_area * target / total
        };
        let tolerance = 64.0 * f64::EPSILON * filler.current_area.abs().max(1.0);
        if !area.is_finite() || area > filler.current_area + tolerance {
            return Err(InstanceAreaError::ArithmeticOverflow);
        }
        let area = area.min(filler.current_area);
        checked_add(&mut assigned, filler.kind, area)?;
        result.push(area);
    }
    Ok(result)
}

fn validate_postconditions(
    members: &[InstanceAreaMember],
    member_areas: &[f64],
    fillers: &[InstanceAreaFiller],
    filler_areas: &[f64],
) -> Result<(), InstanceAreaError> {
    let mut before = BTreeMap::<ResourceKind, f64>::new();
    let mut after = BTreeMap::<ResourceKind, f64>::new();
    for (member, &area) in members.iter().zip(member_areas) {
        if area < member.current_area || !area.is_finite() {
            return Err(InstanceAreaError::PostconditionViolation);
        }
        checked_add(&mut before, member.kind, member.current_area)?;
        checked_add(&mut after, member.kind, area)?;
    }
    for (filler, &area) in fillers.iter().zip(filler_areas) {
        if area < 0.0 || area > filler.current_area || !area.is_finite() {
            return Err(InstanceAreaError::PostconditionViolation);
        }
        checked_add(&mut before, filler.kind, filler.current_area)?;
        checked_add(&mut after, filler.kind, area)?;
    }
    for (&kind, &before_total) in &before {
        let after_total = after.get(&kind).copied().unwrap_or(0.0);
        let term_count = u32::try_from(members.len().saturating_add(fillers.len()))
            .map_err(|_| InstanceAreaError::ArithmeticOverflow)?;
        let tolerance =
            64.0 * f64::EPSILON * f64::from(term_count).max(1.0) * before_total.abs().max(1.0);
        if (before_total - after_total).abs() > tolerance {
            return Err(InstanceAreaError::PostconditionViolation);
        }
    }
    Ok(())
}

fn usize_to_f64(value: usize) -> Result<f64, InstanceAreaError> {
    let value = u32::try_from(value).map_err(|_| InstanceAreaError::ArithmeticOverflow)?;
    Ok(f64::from(value))
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use texo_model::ResourceKind;

    use super::{
        InstanceAreaFiller, InstanceAreaMember, adjust_instance_areas, pin_optimized_area,
        routability_optimized_area, routability_optimized_area_from_bin,
    };
    use crate::RoutingDemandBin;

    fn member(
        unit: usize,
        kind: ResourceKind,
        current_area: f64,
        routability_area: f64,
        pin_area: f64,
    ) -> InstanceAreaMember {
        InstanceAreaMember {
            unit,
            kind,
            current_area,
            routability_area,
            pin_area,
            clustering_area: 0.0,
        }
    }

    fn filler(kind: ResourceKind, current_area: f64) -> InstanceAreaFiller {
        InstanceAreaFiller { kind, current_area }
    }

    fn totals_by_kind(
        members: &[InstanceAreaMember],
        member_areas: &[f64],
        fillers: &[InstanceAreaFiller],
        filler_areas: &[f64],
    ) -> BTreeMap<ResourceKind, f64> {
        let mut totals = BTreeMap::<ResourceKind, f64>::new();
        for (member, &area) in members.iter().zip(member_areas) {
            *totals.entry(member.kind).or_default() += area;
        }
        for (filler, &area) in fillers.iter().zip(filler_areas) {
            *totals.entry(filler.kind).or_default() += area;
        }
        totals
    }

    fn assert_close(actual: f64, expected: f64) {
        assert!((actual - expected).abs() < 1.0e-9, "{actual} != {expected}");
    }

    #[test]
    fn fractional_charge_is_conserved_per_resource() {
        let members = [
            member(0, ResourceKind::Lut(4), 0.75, 1.50, 0.50),
            member(1, ResourceKind::Lut(4), 1.25, 1.50, 2.00),
            member(1, ResourceKind::Register, 0.50, 0.75, 0.50),
        ];
        let fillers = [
            filler(ResourceKind::Lut(4), 0.30),
            filler(ResourceKind::Lut(4), 0.70),
            filler(ResourceKind::Register, 0.75),
        ];
        let before_member = members
            .iter()
            .map(|member| member.current_area)
            .collect::<Vec<_>>();
        let before_filler = fillers
            .iter()
            .map(|filler| filler.current_area)
            .collect::<Vec<_>>();
        let before = totals_by_kind(&members, &before_member, &fillers, &before_filler);

        let adjusted = adjust_instance_areas(&members, &fillers).unwrap();
        let after = totals_by_kind(
            &members,
            &adjusted.member_areas,
            &fillers,
            &adjusted.filler_areas,
        );
        assert_eq!(
            before.keys().collect::<Vec<_>>(),
            after.keys().collect::<Vec<_>>()
        );
        for (&kind, &total) in &before {
            assert_close(after[&kind], total);
        }
        assert!(adjusted.requires_optimizer_reset);
    }

    #[test]
    fn resource_scaling_never_exceeds_capacity_or_borrows_other_fillers() {
        for lut_filler_quarters in 0..=8 {
            for requested_quarters in 0..=16 {
                let lut_filler = f64::from(lut_filler_quarters) / 4.0;
                let requested = 1.0 + f64::from(requested_quarters) / 4.0;
                let members = [
                    member(0, ResourceKind::Lut(4), 1.0, requested, 0.0),
                    member(0, ResourceKind::Register, 1.0, 5.0, 0.0),
                ];
                let fillers = [
                    filler(ResourceKind::Lut(4), lut_filler),
                    filler(ResourceKind::Register, 10.0),
                ];
                let adjusted = adjust_instance_areas(&members, &fillers).unwrap();
                assert!(adjusted.member_areas[0] <= 1.0 + lut_filler + 1.0e-12);
                assert!(adjusted.member_areas[0] <= requested + 1.0e-12);
                assert!(adjusted.filler_areas[0] >= 0.0);
                assert_close(
                    adjusted.member_areas[0] + adjusted.filler_areas[0],
                    1.0 + lut_filler,
                );
            }
        }
    }

    #[test]
    fn varied_fractional_inputs_preserve_charge_and_only_shrink_fillers() {
        for first_eighths in 0..=16 {
            for second_eighths in 0..=16 {
                for requested_eighths in 0..=32 {
                    let first = f64::from(first_eighths) / 8.0;
                    let second = f64::from(second_eighths) / 8.0;
                    let requested = 0.625 + f64::from(requested_eighths) / 8.0;
                    let members = [member(0, ResourceKind::Lut(4), 0.625, requested, 0.0)];
                    let fillers = [
                        filler(ResourceKind::Lut(4), first),
                        filler(ResourceKind::Lut(4), second),
                    ];
                    let adjusted = adjust_instance_areas(&members, &fillers).unwrap();
                    assert!(adjusted.member_areas[0] >= members[0].current_area);
                    assert!(adjusted.filler_areas[0] <= first);
                    assert!(adjusted.filler_areas[1] <= second);
                    assert_close(
                        adjusted.member_areas[0]
                            + adjusted.filler_areas[0]
                            + adjusted.filler_areas[1],
                        0.625 + first + second,
                    );
                }
            }
        }
    }

    #[test]
    fn physical_members_never_deflate() {
        let members = [
            member(0, ResourceKind::Lut(4), 1.25, 0.0, 0.5),
            member(1, ResourceKind::Register, 0.5, 0.1, 0.0),
        ];
        let fillers = [filler(ResourceKind::Lut(4), 2.0)];
        let adjusted = adjust_instance_areas(&members, &fillers).unwrap();
        for (member, &area) in members.iter().zip(&adjusted.member_areas) {
            assert!(area >= member.current_area);
        }
        assert!(!adjusted.requires_optimizer_reset);
    }

    #[test]
    fn heterogeneous_macro_has_one_origin_and_distinct_member_forces() {
        let members = [
            member(7, ResourceKind::Lut(4), 0.75, 1.50, 0.0),
            member(7, ResourceKind::Register, 0.50, 0.50, 1.25),
        ];
        let fillers = [
            filler(ResourceKind::Lut(4), 2.0),
            filler(ResourceKind::Register, 2.0),
        ];
        let adjusted = adjust_instance_areas(&members, &fillers).unwrap();
        assert_eq!(members[0].unit, members[1].unit);
        assert!((adjusted.member_areas[0] - adjusted.member_areas[1]).abs() > 1.0e-9);

        // Each resource has a different potential, but both derivatives act
        // on the one shared placement-unit origin.
        let energy = |x: f64, y: f64| {
            let lut_x = x + 0.25;
            let lut_y = y - 0.50;
            let ff_x = x - 0.75;
            let ff_y = y + 0.25;
            adjusted.member_areas[0] * (0.5 * lut_x * lut_x + 1.5 * lut_y * lut_y)
                + adjusted.member_areas[1] * (2.0 * ff_x * ff_x + 0.25 * ff_y * ff_y)
        };
        let (x, y) = (1.3, 2.1);
        let analytic_x =
            adjusted.member_areas[0] * (x + 0.25) + adjusted.member_areas[1] * 4.0 * (x - 0.75);
        let analytic_y = adjusted.member_areas[0] * 3.0 * (y - 0.50)
            + adjusted.member_areas[1] * 0.5 * (y + 0.25);
        let epsilon = 1.0e-6;
        let finite_x = (energy(x + epsilon, y) - energy(x - epsilon, y)) / (2.0 * epsilon);
        let finite_y = (energy(x, y + epsilon) - energy(x, y - epsilon)) / (2.0 * epsilon);
        assert!((analytic_x - finite_x).abs() < 1.0e-8);
        assert!((analytic_y - finite_y).abs() < 1.0e-8);
    }

    #[test]
    fn zero_congestion_is_identity_and_does_not_request_reset() {
        let current = 1.25;
        let routing = routability_optimized_area(current, 0.0, 0.0).unwrap();
        let pin = pin_optimized_area(0, 4.0, 0.0).unwrap();
        let members = [member(0, ResourceKind::Lut(4), current, routing, pin)];
        let fillers = [filler(ResourceKind::Lut(4), 3.75)];
        let adjusted = adjust_instance_areas(&members, &fillers).unwrap();
        assert_eq!(adjusted.member_areas, [current]);
        assert_eq!(adjusted.filler_areas, [3.75]);
        assert!(!adjusted.requires_optimizer_reset);
    }

    #[test]
    fn full_scale_installs_the_requested_bits_without_a_fake_followup_reset() {
        let current = 1.0_f64;
        let requested = f64::from_bits(current.to_bits() + 1);
        let members = [member(0, ResourceKind::Register, current, requested, 0.0)];
        let fillers = [filler(ResourceKind::Register, 1.0)];
        let first = adjust_instance_areas(&members, &fillers).unwrap();
        assert_eq!(first.member_areas[0].to_bits(), requested.to_bits());
        assert!(first.requires_optimizer_reset);

        let settled = [member(
            0,
            ResourceKind::Register,
            first.member_areas[0],
            requested,
            0.0,
        )];
        let second = adjust_instance_areas(&settled, &fillers).unwrap();
        assert_eq!(second.member_areas[0].to_bits(), requested.to_bits());
        assert_eq!(
            second.filler_areas[0].to_bits(),
            fillers[0].current_area.to_bits()
        );
        assert!(!second.requires_optimizer_reset);
    }

    #[test]
    fn rudy_bin_uses_architecture_capacity_and_caps_blocked_direction_inflation() {
        let empty = RoutingDemandBin {
            x: 0,
            y: 0,
            width: 4,
            height: 4,
            horizontal_capacity: 0,
            vertical_capacity: 0,
            horizontal_demand: 0.0,
            vertical_demand: 0.0,
        };
        assert_close(
            routability_optimized_area_from_bin(1.25, &empty).unwrap(),
            0.0,
        );

        let blocked_hotspot = RoutingDemandBin {
            horizontal_demand: 0.25,
            ..empty
        };
        assert_close(
            routability_optimized_area_from_bin(1.25, &blocked_hotspot).unwrap(),
            2.5,
        );

        let wide_capacity = RoutingDemandBin {
            horizontal_capacity: u64::from(u32::MAX) + 17,
            horizontal_demand: f64::from(u32::MAX) + 17.0,
            ..empty
        };
        assert_close(
            routability_optimized_area_from_bin(1.25, &wide_capacity).unwrap(),
            1.25,
        );
    }

    #[test]
    fn adjustment_is_deterministic() {
        let members = [
            member(3, ResourceKind::Register, 0.5, 1.25, 0.0),
            member(3, ResourceKind::Lut(4), 0.75, 0.0, 2.0),
            member(9, ResourceKind::Register, 1.0, 1.125, 1.25),
        ];
        let fillers = [
            filler(ResourceKind::Register, 0.125),
            filler(ResourceKind::Lut(4), 0.25),
            filler(ResourceKind::Register, 0.875),
        ];
        let first = adjust_instance_areas(&members, &fillers).unwrap();
        for _ in 0..100 {
            assert_eq!(adjust_instance_areas(&members, &fillers).unwrap(), first);
        }
    }
}
