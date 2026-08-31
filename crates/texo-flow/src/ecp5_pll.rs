//! ECP5 PLL clock derivation for the flow constraint layer.

use std::collections::{BTreeMap, BTreeSet};

use texo_model::{CellId, Design, NetId, PinDirection};
use texo_struo::{PllOutput, PrimitiveMetadata};
use texo_target_ecp5::{ECP5_PLL_OUTPUT_DIVIDER_DEFAULT, Ecp5Packing};

use super::{Ecp5FlowError, find_cell_pin};

const PICOSECONDS_PER_SECOND: u128 = 1_000_000_000_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ClockPeriod {
    numerator_ps: u128,
    denominator: u128,
}

impl ClockPeriod {
    fn from_frequency_hz(frequency_hz: u64) -> Option<Self> {
        (frequency_hz != 0).then(|| Self::normalized(PICOSECONDS_PER_SECOND, frequency_hz.into()))
    }

    fn from_picoseconds(period_ps: u64) -> Option<Self> {
        (period_ps != 0).then(|| Self {
            numerator_ps: period_ps.into(),
            denominator: 1,
        })
    }

    fn scaled(self, numerator: u64, denominator: u64) -> Result<Self, Ecp5FlowError> {
        let mut left_numerator = self.numerator_ps;
        let mut left_denominator = self.denominator;
        let mut right_numerator = u128::from(numerator);
        let mut right_denominator = u128::from(denominator);
        let cross = gcd_u128(left_numerator, right_denominator);
        left_numerator /= cross;
        right_denominator /= cross;
        let cross = gcd_u128(right_numerator, left_denominator);
        right_numerator /= cross;
        left_denominator /= cross;
        let numerator_ps = left_numerator
            .checked_mul(right_numerator)
            .ok_or(Ecp5FlowError::TimingDelayOverflow)?;
        let denominator = left_denominator
            .checked_mul(right_denominator)
            .filter(|value| *value != 0)
            .ok_or(Ecp5FlowError::TimingDelayOverflow)?;
        Ok(Self::normalized(numerator_ps, denominator))
    }

    fn rounded_picoseconds(self) -> Result<u64, Ecp5FlowError> {
        let rounded = self
            .numerator_ps
            .checked_add(self.denominator / 2)
            .and_then(|value| value.checked_div(self.denominator))
            .ok_or(Ecp5FlowError::TimingDelayOverflow)?;
        u64::try_from(rounded)
            .ok()
            .filter(|period| *period != 0)
            .ok_or(Ecp5FlowError::TimingDelayOverflow)
    }

    fn normalized(numerator_ps: u128, denominator: u128) -> Self {
        let common = gcd_u128(numerator_ps, denominator);
        Self {
            numerator_ps: numerator_ps / common,
            denominator: denominator / common,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct GeneratedClockRelation {
    pub(super) source: NetId,
    pub(super) multiply_by: u32,
    pub(super) divide_by: u32,
    pub(super) phase_ps: i64,
}

pub(super) type GeneratedClockRelations = BTreeMap<NetId, GeneratedClockRelation>;

#[derive(Clone, Copy)]
struct PendingPll<'a> {
    cell: CellId,
    input_net: NetId,
    fabric_output: PllOutput,
    feedback_output: PllOutput,
    parameters: &'a BTreeMap<String, String>,
}

pub(super) fn constrain_pll_outputs(
    design: &Design,
    metadata: &BTreeMap<CellId, PrimitiveMetadata>,
    packing: &mut Ecp5Packing,
) -> Result<GeneratedClockRelations, Ecp5FlowError> {
    let mut relations = BTreeMap::new();
    let mut periods = known_clock_periods(design, packing)?;
    let mut pending = Vec::new();
    for (&cell, primitive) in metadata {
        let PrimitiveMetadata::Pll {
            fabric_output,
            feedback_output,
            parameters,
            ..
        } = primitive
        else {
            continue;
        };
        pending.push(PendingPll {
            cell,
            input_net: pll_input_net(design, cell)?,
            fabric_output: *fabric_output,
            feedback_output: *feedback_output,
            parameters,
        });
    }

    while !pending.is_empty() {
        propagate_global_clock_periods(packing, &mut periods)?;
        let mut deferred = Vec::new();
        let mut resolved_any = false;
        for pll in pending {
            let Some(&input_period) = periods.get(&pll.input_net) else {
                deferred.push(pll);
                continue;
            };
            let clocks = constrain_one_pll(design, pll, input_period, packing, &mut relations)?;
            for clock in clocks {
                insert_known_period(&mut periods, clock.net, clock.period)?;
            }
            propagate_global_clock_periods(packing, &mut periods)?;
            resolved_any = true;
        }
        if !resolved_any {
            let unresolved = deferred[0];
            return Err(Ecp5FlowError::MissingPllInputClockConstraint {
                cell: design.cells()[unresolved.cell.0].name.clone(),
                net: unresolved.input_net,
            });
        }
        pending = deferred;
    }
    Ok(relations)
}

fn known_clock_periods(
    design: &Design,
    packing: &Ecp5Packing,
) -> Result<BTreeMap<NetId, ClockPeriod>, Ecp5FlowError> {
    let mut periods = BTreeMap::new();
    for (&cell, &frequency_hz) in packing.clock_frequencies_hz() {
        let logical = &design.cells()[cell.0];
        let driven_nets = logical
            .pins()
            .iter()
            .filter_map(|&pin| {
                let pin = &design.pins()[pin.0];
                (pin.direction != PinDirection::Input)
                    .then(|| pin.net())
                    .flatten()
            })
            .collect::<BTreeSet<_>>();
        if driven_nets.len() != 1 {
            return Err(Ecp5FlowError::ClockIoNet {
                cell: logical.name.clone(),
            });
        }
        let period = ClockPeriod::from_frequency_hz(frequency_hz).ok_or_else(|| {
            Ecp5FlowError::ClockFrequencyOutOfRange {
                cell: logical.name.clone(),
                frequency_hz,
            }
        })?;
        insert_known_period(&mut periods, *driven_nets.first().unwrap(), period)?;
    }
    for (&net, &period_ps) in packing.generated_clock_periods_ps() {
        let period =
            ClockPeriod::from_picoseconds(period_ps).ok_or(Ecp5FlowError::TimingDelayOverflow)?;
        insert_known_period(&mut periods, net, period)?;
    }
    propagate_global_clock_periods(packing, &mut periods)?;
    Ok(periods)
}

fn propagate_global_clock_periods(
    packing: &Ecp5Packing,
    periods: &mut BTreeMap<NetId, ClockPeriod>,
) -> Result<(), Ecp5FlowError> {
    loop {
        let mut changed = false;
        for clock in packing.global_clocks() {
            let Some(&period) = periods.get(&clock.source_net) else {
                continue;
            };
            changed |= insert_known_period(periods, clock.global_net, period)?;
        }
        if !changed {
            return Ok(());
        }
    }
}

fn insert_known_period(
    periods: &mut BTreeMap<NetId, ClockPeriod>,
    net: NetId,
    period: ClockPeriod,
) -> Result<bool, Ecp5FlowError> {
    if let Some(previous) = periods.get(&net) {
        if previous.rounded_picoseconds()? != period.rounded_picoseconds()? {
            return Err(Ecp5FlowError::ConflictingClockPeriods { net });
        }
        return Ok(false);
    }
    periods.insert(net, period);
    Ok(true)
}

fn constrain_one_pll(
    design: &Design,
    pll: PendingPll<'_>,
    input_period: ClockPeriod,
    packing: &mut Ecp5Packing,
    relations: &mut GeneratedClockRelations,
) -> Result<Vec<OutputClock>, Ecp5FlowError> {
    let PendingPll {
        cell,
        input_net,
        fabric_output,
        feedback_output,
        parameters,
    } = pll;
    let cell_name = &design.cells()[cell.0].name;
    if fabric_output == PllOutput::Clkintfb {
        return Err(unsupported(
            cell_name,
            "CLKINTFB",
            "internal feedback cannot be a fabric clock",
        ));
    }
    if parameters
        .get("DPHASE_SOURCE")
        .is_some_and(|value| value != "DISABLED")
    {
        return Err(unsupported(
            cell_name,
            "dynamic outputs",
            "runtime phase adjustment has no static generated-clock waveform",
        ));
    }

    let input_divider = integer_parameter(parameters, "CLKI_DIV", 1, cell_name)?;
    let feedback_divider = integer_parameter(parameters, "CLKFB_DIV", 1, cell_name)?;
    let feedback_output_divider = feedback_path_divider(parameters, cell_name)?;
    let outputs = fabric_outputs(design, cell, fabric_output, feedback_output);
    let has_clkop = fabric_output == PllOutput::Clkop || feedback_output == PllOutput::Clkop;
    let clocks = outputs
        .into_iter()
        .map(|output| {
            output_clock(
                design,
                cell,
                output,
                parameters,
                cell_name,
                has_clkop,
                input_period,
                input_divider,
                feedback_divider,
                feedback_output_divider,
            )
        })
        .collect::<Result<Vec<_>, _>>()?;
    for clock in &clocks {
        let period_ps = clock.period.rounded_picoseconds()?;
        if let Some(previous) = packing.set_generated_clock_period_ps(clock.net, period_ps)
            && previous != period_ps
        {
            return Err(Ecp5FlowError::ConflictingClockPeriods { net: clock.net });
        }
    }
    record_relations(
        &clocks,
        input_net,
        input_divider,
        feedback_divider,
        feedback_output_divider,
        relations,
        cell_name,
    )?;
    Ok(clocks)
}

fn pll_input_net(design: &Design, cell: CellId) -> Result<NetId, Ecp5FlowError> {
    let cell_name = &design.cells()[cell.0].name;
    let pin =
        find_cell_pin(design, cell, "CLKI").ok_or_else(|| Ecp5FlowError::MissingPllInputClock {
            cell: cell_name.into(),
        })?;
    design.pins()[pin.0]
        .net()
        .ok_or_else(|| Ecp5FlowError::MissingPllInputClock {
            cell: cell_name.into(),
        })
}

fn feedback_path_divider(
    parameters: &BTreeMap<String, String>,
    cell_name: &str,
) -> Result<u64, Ecp5FlowError> {
    let path = parameters
        .get("FEEDBK_PATH")
        .map_or("CLKOP", String::as_str);
    let output = match path {
        "CLKOP" | "INT_OP" => "CLKOP",
        "CLKOS" | "INT_OS" => "CLKOS",
        "CLKOS2" | "INT_OS2" => "CLKOS2",
        "CLKOS3" | "INT_OS3" => "CLKOS3",
        _ => {
            return Err(unsupported(
                cell_name,
                path,
                "FEEDBK_PATH is not one of the four divider outputs",
            ));
        }
    };
    integer_parameter(
        parameters,
        &format!("{output}_DIV"),
        ECP5_PLL_OUTPUT_DIVIDER_DEFAULT,
        cell_name,
    )
}

fn fabric_outputs(
    design: &Design,
    cell: CellId,
    fabric_output: PllOutput,
    feedback_output: PllOutput,
) -> Vec<PllOutput> {
    let mut outputs = vec![fabric_output];
    for output in [
        PllOutput::Clkop,
        PllOutput::Clkos,
        PllOutput::Clkos2,
        PllOutput::Clkos3,
    ] {
        if output != fabric_output
            && output != feedback_output
            && find_cell_pin(design, cell, output.port()).is_some()
        {
            outputs.push(output);
        }
    }
    outputs
}

#[derive(Clone, Copy)]
struct OutputClock {
    output: PllOutput,
    net: NetId,
    divider: u64,
    coarse_phase: u64,
    fine_phase: u64,
    period: ClockPeriod,
}

#[allow(clippy::too_many_arguments)]
fn output_clock(
    design: &Design,
    cell: CellId,
    output: PllOutput,
    parameters: &BTreeMap<String, String>,
    cell_name: &str,
    has_clkop: bool,
    input_period: ClockPeriod,
    input_divider: u64,
    feedback_divider: u64,
    feedback_output_divider: u64,
) -> Result<OutputClock, Ecp5FlowError> {
    validate_output_mode(parameters, output, cell_name, has_clkop)?;
    let divider = integer_parameter(
        parameters,
        &format!("{}_DIV", output.port()),
        ECP5_PLL_OUTPUT_DIVIDER_DEFAULT,
        cell_name,
    )?;
    let coarse_phase = phase_parameter(parameters, output, "CPHASE", 127, cell_name)?;
    let fine_phase = phase_parameter(parameters, output, "FPHASE", 7, cell_name)?;
    let pin = find_cell_pin(design, cell, output.port()).ok_or_else(|| {
        Ecp5FlowError::MissingPllOutputPin {
            cell: cell_name.into(),
            pin: output.port().into(),
        }
    })?;
    let net = design.pins()[pin.0]
        .net()
        .ok_or_else(|| Ecp5FlowError::MissingPllOutputNet {
            cell: cell_name.into(),
            pin: output.port().into(),
        })?;
    let period = input_period
        .scaled(input_divider, feedback_divider)?
        .scaled(divider, feedback_output_divider)?;
    Ok(OutputClock {
        output,
        net,
        divider,
        coarse_phase,
        fine_phase,
        period,
    })
}

fn validate_output_mode(
    parameters: &BTreeMap<String, String>,
    output: PllOutput,
    cell_name: &str,
    has_clkop: bool,
) -> Result<(), Ecp5FlowError> {
    let (letter, divider_mux) = match output {
        PllOutput::Clkop => ('A', "DIVA"),
        PllOutput::Clkos => ('B', "DIVB"),
        PllOutput::Clkos2 => ('C', "DIVC"),
        PllOutput::Clkos3 => ('D', "DIVD"),
        PllOutput::Clkintfb => unreachable!("fabric CLKINTFB was rejected"),
    };
    let enable = format!("{}_ENABLE", output.port());
    if parameters.get(&enable).map_or("ENABLED", String::as_str) != "ENABLED" {
        return Err(unsupported(
            cell_name,
            output.port(),
            "output is not enabled",
        ));
    }
    let mux = format!("OUTDIVIDER_MUX{letter}");
    let default_mux = if has_clkop { divider_mux } else { "REFCLK" };
    if parameters.get(&mux).map_or(default_mux, String::as_str) != divider_mux {
        return Err(unsupported(
            cell_name,
            output.port(),
            "output divider mux is bypassed",
        ));
    }
    Ok(())
}

fn phase_parameter(
    parameters: &BTreeMap<String, String>,
    output: PllOutput,
    suffix: &str,
    maximum: u64,
    cell_name: &str,
) -> Result<u64, Ecp5FlowError> {
    let name = format!("{}_{suffix}", output.port());
    let value = parameters
        .get(&name)
        .map_or(Some(0), |raw| raw.parse::<u64>().ok())
        .filter(|value| *value <= maximum)
        .ok_or_else(|| Ecp5FlowError::InvalidPllParameter {
            cell: cell_name.into(),
            parameter: name.clone(),
            value: parameters.get(&name).cloned(),
        })?;
    Ok(value)
}

fn integer_parameter(
    parameters: &BTreeMap<String, String>,
    name: &str,
    default: u64,
    cell_name: &str,
) -> Result<u64, Ecp5FlowError> {
    parameters
        .get(name)
        .map_or(Some(default), |raw| raw.parse::<u64>().ok())
        .filter(|value| (1..=128).contains(value))
        .ok_or_else(|| Ecp5FlowError::InvalidPllParameter {
            cell: cell_name.into(),
            parameter: name.into(),
            value: parameters.get(name).cloned(),
        })
}

fn record_relations(
    clocks: &[OutputClock],
    input_net: NetId,
    input_divider: u64,
    feedback_divider: u64,
    feedback_output_divider: u64,
    relations: &mut GeneratedClockRelations,
    cell_name: &str,
) -> Result<(), Ecp5FlowError> {
    let root = clocks[0];
    if (root.coarse_phase, root.fine_phase) != (0, 0) {
        return Err(unsupported(
            cell_name,
            root.output.port(),
            "non-zero absolute output phase is not modeled",
        ));
    }
    for generated in clocks.iter().skip(1) {
        if (generated.coarse_phase, generated.fine_phase) != (root.coarse_phase, root.fine_phase) {
            return Err(unsupported(
                cell_name,
                generated.output.port(),
                "output phase differs from the selected fabric clock",
            ));
        }
    }
    let root_multiply = feedback_divider
        .checked_mul(feedback_output_divider)
        .ok_or(Ecp5FlowError::TimingDelayOverflow)?;
    let root_divide = input_divider
        .checked_mul(root.divider)
        .ok_or(Ecp5FlowError::TimingDelayOverflow)?;
    let common = gcd(root_multiply, root_divide);
    insert_relation(
        relations,
        root.net,
        GeneratedClockRelation {
            source: input_net,
            multiply_by: u32::try_from(root_multiply / common)
                .map_err(|_| Ecp5FlowError::TimingDelayOverflow)?,
            divide_by: u32::try_from(root_divide / common)
                .map_err(|_| Ecp5FlowError::TimingDelayOverflow)?,
            phase_ps: 0,
        },
    )?;
    for &generated in clocks.iter().skip(1) {
        let common = gcd(root.divider, generated.divider);
        let relation = GeneratedClockRelation {
            source: root.net,
            multiply_by: u32::try_from(root.divider / common)
                .map_err(|_| Ecp5FlowError::TimingDelayOverflow)?,
            divide_by: u32::try_from(generated.divider / common)
                .map_err(|_| Ecp5FlowError::TimingDelayOverflow)?,
            phase_ps: 0,
        };
        insert_relation(relations, generated.net, relation)?;
    }
    Ok(())
}

fn insert_relation(
    relations: &mut GeneratedClockRelations,
    net: NetId,
    relation: GeneratedClockRelation,
) -> Result<(), Ecp5FlowError> {
    if let Some(previous) = relations.insert(net, relation)
        && previous != relation
    {
        return Err(Ecp5FlowError::ConflictingClockRelations { net });
    }
    Ok(())
}

fn unsupported(cell: &str, output: &str, reason: &str) -> Ecp5FlowError {
    Ecp5FlowError::UnsupportedPllClockRelation {
        cell: cell.into(),
        output: output.into(),
        reason: reason.into(),
    }
}

const fn gcd(mut left: u64, mut right: u64) -> u64 {
    while right != 0 {
        (left, right) = (right, left % right);
    }
    left
}

const fn gcd_u128(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        (left, right) = (right, left % right);
    }
    left
}
