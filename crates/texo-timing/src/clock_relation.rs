use std::collections::{BTreeMap, BTreeSet};

use texo_model::NetId;

use super::{ClockEdge, TimingConstraints, TimingError};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct GeneratedClockConstraint {
    pub(crate) source: NetId,
    pub(crate) multiply_by: u32,
    pub(crate) divide_by: u32,
    pub(crate) phase_ps: i64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct ClockWaveform {
    pub(crate) root: NetId,
    period_scale_num: u128,
    period_scale_den: u128,
    phase_ps: i128,
}

#[derive(Clone, Copy)]
pub(crate) struct RelatedEdgeOffsets {
    pub(crate) setup_ps: u64,
    pub(crate) hold_ps: u64,
}

pub(crate) fn validate_clock_relations(
    constraints: &TimingConstraints,
    net_count: usize,
) -> Result<(), TimingError> {
    let mut relation_nets = BTreeSet::new();
    for (&net, &generated) in &constraints.generated_clocks {
        for referenced_net in [net, generated.source] {
            if referenced_net.0 >= net_count {
                return Err(TimingError::UnknownClockNet(referenced_net));
            }
            relation_nets.insert(referenced_net);
        }
        if generated.multiply_by == 0 || generated.divide_by == 0 {
            return Err(TimingError::InvalidGeneratedClockRatio(net));
        }
    }
    relation_nets.extend(constraints.clock_periods_ps.keys().copied());
    let waveforms = resolve_clock_waveforms(constraints, relation_nets)?;
    let mut clocks_by_root = BTreeMap::<NetId, Vec<(NetId, u64, ClockWaveform)>>::new();
    for (&net, &period_ps) in &constraints.clock_periods_ps {
        let waveform = waveforms[&net];
        clocks_by_root
            .entry(waveform.root)
            .or_default()
            .push((net, period_ps, waveform));
    }
    for clocks in clocks_by_root.values() {
        for (index, &left) in clocks.iter().enumerate() {
            for &right in &clocks[index + 1..] {
                validate_clock_period_pair(left, right)?;
            }
        }
    }
    Ok(())
}

fn validate_clock_period_pair(
    left: (NetId, u64, ClockWaveform),
    right: (NetId, u64, ClockWaveform),
) -> Result<(), TimingError> {
    let (ratio_num, ratio_den) = period_ratio(left.2, right.2)?;
    let left_ticks = checked_mul(u128::from(left.1), ratio_den)?;
    let right_ticks = checked_mul(u128::from(right.1), ratio_num)?;
    // Each independently quantized endpoint period may be within one
    // picosecond of the exact common-source waveform.
    let tolerance = ratio_num
        .checked_add(ratio_den)
        .ok_or(TimingError::ClockRelationOverflow)?;
    if left_ticks.abs_diff(right_ticks) <= tolerance {
        Ok(())
    } else {
        Err(TimingError::InconsistentRelatedClockPeriods {
            first: left.0,
            second: right.0,
        })
    }
}

pub(crate) fn resolve_clock_waveforms(
    constraints: &TimingConstraints,
    clock_nets: impl IntoIterator<Item = NetId>,
) -> Result<BTreeMap<NetId, ClockWaveform>, TimingError> {
    let mut waveforms = BTreeMap::new();
    let mut visiting = BTreeSet::new();
    for clock_net in clock_nets {
        resolve_clock_waveform(clock_net, constraints, &mut waveforms, &mut visiting)?;
    }
    Ok(waveforms)
}

fn resolve_clock_waveform(
    clock_net: NetId,
    constraints: &TimingConstraints,
    waveforms: &mut BTreeMap<NetId, ClockWaveform>,
    visiting: &mut BTreeSet<NetId>,
) -> Result<ClockWaveform, TimingError> {
    if let Some(&waveform) = waveforms.get(&clock_net) {
        return Ok(waveform);
    }
    if !visiting.insert(clock_net) {
        return Err(TimingError::GeneratedClockCycle(clock_net));
    }
    let waveform = if let Some(&generated) = constraints.generated_clocks.get(&clock_net) {
        if generated.multiply_by == 0 || generated.divide_by == 0 {
            return Err(TimingError::InvalidGeneratedClockRatio(clock_net));
        }
        let source = resolve_clock_waveform(generated.source, constraints, waveforms, visiting)?;
        let (period_scale_num, period_scale_den) = multiply_ratios(
            source.period_scale_num,
            source.period_scale_den,
            u128::from(generated.divide_by),
            u128::from(generated.multiply_by),
        )?;
        ClockWaveform {
            root: source.root,
            period_scale_num,
            period_scale_den,
            phase_ps: source
                .phase_ps
                .checked_add(i128::from(generated.phase_ps))
                .ok_or(TimingError::ClockRelationOverflow)?,
        }
    } else {
        ClockWaveform {
            root: clock_net,
            period_scale_num: 1,
            period_scale_den: 1,
            phase_ps: 0,
        }
    };
    visiting.remove(&clock_net);
    waveforms.insert(clock_net, waveform);
    Ok(waveform)
}

pub(crate) fn related_edge_offsets(
    launch: ClockWaveform,
    capture: ClockWaveform,
    capture_period_ps: u64,
    launch_edge: ClockEdge,
    capture_edge: ClockEdge,
) -> Result<RelatedEdgeOffsets, TimingError> {
    debug_assert_eq!(launch.root, capture.root);
    let (ratio_num, ratio_den) = period_ratio(launch, capture)?;

    // A 1/(2*ratio_den) ps lattice represents both active clock edges exactly.
    let ticks_per_ps = checked_mul(ratio_den, 2)?;
    let launch_period = checked_mul(checked_mul(u128::from(capture_period_ps), ratio_num)?, 2)?;
    let capture_period = checked_mul(u128::from(capture_period_ps), ticks_per_ps)?;
    let ticks_per_ps_signed = to_i128(ticks_per_ps)?;
    let phase_delta = capture
        .phase_ps
        .checked_sub(launch.phase_ps)
        .and_then(|phase| phase.checked_mul(ticks_per_ps_signed))
        .ok_or(TimingError::ClockRelationOverflow)?;
    let launch_edge = edge_offset(launch_edge, launch_period)?;
    let capture_edge = edge_offset(capture_edge, capture_period)?;
    let difference = phase_delta
        .checked_add(capture_edge)
        .and_then(|value| value.checked_sub(launch_edge))
        .ok_or(TimingError::ClockRelationOverflow)?;
    let lattice = gcd(launch_period, capture_period);
    let residue = difference.rem_euclid(to_i128(lattice)?) as u128;
    let setup = if residue == 0 { lattice } else { residue };
    let hold = if residue == 0 { 0 } else { lattice - residue };
    Ok(RelatedEdgeOffsets {
        setup_ps: to_u64(setup / ticks_per_ps)?,
        hold_ps: to_u64(hold / ticks_per_ps)?,
    })
}

fn edge_offset(edge: ClockEdge, period_ticks: u128) -> Result<i128, TimingError> {
    to_i128(match edge {
        ClockEdge::Rising => 0,
        ClockEdge::Falling => period_ticks / 2,
    })
}

fn checked_mul(left: u128, right: u128) -> Result<u128, TimingError> {
    left.checked_mul(right)
        .ok_or(TimingError::ClockRelationOverflow)
}

fn period_ratio(left: ClockWaveform, right: ClockWaveform) -> Result<(u128, u128), TimingError> {
    multiply_ratios(
        left.period_scale_num,
        left.period_scale_den,
        right.period_scale_den,
        right.period_scale_num,
    )
}

fn multiply_ratios(
    mut left_num: u128,
    mut left_den: u128,
    mut right_num: u128,
    mut right_den: u128,
) -> Result<(u128, u128), TimingError> {
    let cross = gcd(left_num, right_den);
    (left_num, right_den) = (left_num / cross, right_den / cross);
    let cross = gcd(right_num, left_den);
    (right_num, left_den) = (right_num / cross, left_den / cross);
    Ok((
        checked_mul(left_num, right_num)?,
        checked_mul(left_den, right_den)?,
    ))
}

fn gcd(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        (left, right) = (right, left % right);
    }
    left
}

fn to_i128(value: u128) -> Result<i128, TimingError> {
    i128::try_from(value).map_err(|_| TimingError::ClockRelationOverflow)
}

fn to_u64(value: u128) -> Result<u64, TimingError> {
    u64::try_from(value).map_err(|_| TimingError::ClockRelationOverflow)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ratio_chaining_cancels_before_multiplication() {
        let factor = u32::MAX;
        let mut constraints = TimingConstraints::new();
        for index in 1..=4 {
            constraints.set_generated_clock(NetId(index), NetId(index - 1), 1, factor, 0);
        }
        constraints.set_generated_clock(NetId(5), NetId(4), factor, factor, 0);

        let waveform = resolve_clock_waveforms(&constraints, [NetId(5)]).unwrap()[&NetId(5)];
        assert_eq!(waveform.period_scale_num, u128::from(factor).pow(4));
        assert_eq!(waveform.period_scale_den, 1);
    }

    #[test]
    fn edge_ratio_cancels_before_cross_multiplication() {
        let numerator = (1_u128 << 127) - 1;
        let denominator = numerator - 1;
        let waveform = ClockWaveform {
            root: NetId(0),
            period_scale_num: numerator,
            period_scale_den: denominator,
            phase_ps: 0,
        };

        let offsets = related_edge_offsets(
            waveform,
            waveform,
            100,
            ClockEdge::Rising,
            ClockEdge::Rising,
        )
        .unwrap();
        assert_eq!(offsets.setup_ps, 100);
        assert_eq!(offsets.hold_ps, 0);
    }
}
