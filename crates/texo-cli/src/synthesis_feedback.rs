//! Synthesis proposals from measured, routed STA. No estimated delays or Fmax.
use serde::Deserialize;
use std::{collections::BTreeMap, error::Error, path::Path};
use struo_target_ecp5::{
    PhysicalFeedback, PhysicalLocation, PhysicalNetTiming, PhysicalTimingEndpoint,
};

#[derive(Deserialize)]
struct Observations {
    schema_version: u32,
    placement: Vec<Placed>,
    timing: Timing,
}
#[derive(Deserialize)]
struct Placed {
    cell: String,
    bel: String,
}
#[derive(Deserialize)]
struct Timing {
    delay_model: String,
    worst_slack_ps: Option<i64>,
    net_setup_slacks: Vec<EndpointSlack>,
    net_delays: Vec<Delay>,
}
#[derive(Deserialize)]
struct EndpointSlack {
    net_id: usize,
    sink_pin_id: usize,
    slack_ps: i64,
}
#[derive(Deserialize)]
struct Delay {
    net_id: usize,
    net: String,
    driver_cell: String,
    sink_cell: String,
    sink_pin: String,
    sink_pin_id: usize,
    max_delay_ps: u32,
}

/// Read actual net delays and allocate each sink's remaining routed setup slack
/// to that net. Replication is a proposal: fresh mapping equivalence and full
/// placement/routing/STA are required before an implementation is accepted.
///
/// # Errors
/// Rejects malformed checkpoints, unsupported schemas, and non-measured STA.
pub fn measured_synthesis_feedback(path: &Path) -> Result<PhysicalFeedback, Box<dyn Error>> {
    let observations: Observations = crate::read_checkpoint(path)?;
    convert(observations)
}

/// Reject a replica name collision before physical import. Older Struo versions
/// restart replica numbering per feedback pass; never accept an ambiguous name.
///
/// # Errors
/// Returns an error if any two LUT/FF cells share a physical replica name.
pub fn validate_feedback_replica_names(
    mapped: &struo_target_ecp5::Ecp5Netlist,
) -> Result<(), Box<dyn Error>> {
    let mut seen = std::collections::BTreeSet::new();
    for cell in mapped.cells() {
        let (struo_target_ecp5::Ecp5Cell::Lut4 { name, .. }
        | struo_target_ecp5::Ecp5Cell::FlipFlop { name, .. }) = cell
        else {
            continue;
        };
        if !seen.insert(name) {
            return Err(format!("duplicate feedback cell name: {name}").into());
        }
    }
    if !mapped.retiming().equivalence_signed_off {
        return Err("cumulative physical feedback equivalence failed".into());
    }
    Ok(())
}

fn convert(observations: Observations) -> Result<PhysicalFeedback, Box<dyn Error>> {
    if observations.schema_version != 3
        || observations.timing.delay_model != "measured_joint_cell_route_min_max_ps"
    {
        return Err("synthesis feedback requires a schema-3 measured-STA checkpoint".into());
    }
    if observations
        .timing
        .worst_slack_ps
        .is_none_or(|slack| slack >= 0)
    {
        return Ok(PhysicalFeedback::default());
    }
    let mut placements = BTreeMap::new();
    let mut bels = BTreeMap::new();
    for placed in observations.placement {
        let tile = placed.bel.split('/').next().ok_or("missing BEL tile")?;
        let (row, column) = tile
            .strip_prefix('R')
            .ok_or("non-ECP5 BEL tile")?
            .split_once('C')
            .ok_or("missing BEL column")?;
        placements.insert(
            placed.cell.clone(),
            PhysicalLocation {
                x: column.parse()?,
                y: row.parse()?,
            },
        );
        bels.insert(placed.cell, placed.bel);
    }
    let slacks = observations
        .timing
        .net_setup_slacks
        .into_iter()
        .map(|s| ((s.net_id, s.sink_pin_id), s.slack_ps))
        .collect::<BTreeMap<_, _>>();
    let mut nets = BTreeMap::<String, (i64, PhysicalNetTiming)>::new();
    for delay in observations.timing.net_delays {
        let Some(&slack) = slacks.get(&(delay.net_id, delay.sink_pin_id)) else {
            continue;
        };
        let budget =
            (i128::from(delay.max_delay_ps) + i128::from(slack)).clamp(0, i128::from(u32::MAX));
        let (worst, net) = nets.entry(delay.net.clone()).or_insert_with(|| {
            (
                slack,
                PhysicalNetTiming {
                    driver: delay.driver_cell.clone(),
                    net: delay.net,
                    endpoints: Vec::new(),
                },
            )
        });
        if net.driver != delay.driver_cell {
            return Err("inconsistent feedback net driver".into());
        }
        *worst = (*worst).min(slack);
        net.endpoints.push(PhysicalTimingEndpoint {
            cell: synthesis_cell_name(&delay.sink_cell).to_owned(),
            port: delay.sink_pin,
            delay_ps: delay.max_delay_ps,
            budget_ps: u32::try_from(budget)?,
        });
    }
    let mut nets = nets
        .into_values()
        .filter(|(slack, _)| *slack < 0)
        .collect::<Vec<_>>();
    nets.sort_by(|a, b| a.0.cmp(&b.0).then_with(|| a.1.net.cmp(&b.1.net)));
    // No invented critical paths or Fmax: only observed sink delays/slacks.
    // The empty clock map also disables speculative physical retiming.
    Ok(PhysicalFeedback::from_observations(
        placements,
        bels,
        nets.into_iter().map(|(_, net)| net).collect(),
        Vec::new(),
        BTreeMap::new(),
    ))
}

// Native CCU2C slices are physical members of one mapped arithmetic cell.
// Rewiring either member must address that original cell, not a missing alias.
fn synthesis_cell_name(name: &str) -> &str {
    for suffix in ["$slice0", "$slice1"] {
        if let Some(base) = name.strip_suffix(suffix)
            && base.starts_with("ccu_")
        {
            return base;
        }
    }
    name
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn matches_slack_by_net_and_normalizes_carry_sink_names() {
        let mut input = fixture("measured_joint_cell_route_min_max_ps", -300);
        input.timing.net_setup_slacks.push(EndpointSlack {
            net_id: 2,
            sink_pin_id: 7,
            slack_ps: 800,
        });
        input.timing.net_delays[0].sink_cell = "ccu_compare8_1$slice0".into();
        let f = convert(input).unwrap();
        assert_eq!(f.net_timings()[0].endpoints[0].cell, "ccu_compare8_1");
        assert_eq!(f.net_timings()[0].endpoints[0].budget_ps, 900);
        assert_eq!(synthesis_cell_name("ff$slice0"), "ff$slice0");
        assert_eq!(synthesis_cell_name("ccu_sum$slice1"), "ccu_sum");
    }

    fn fixture(model: &str, slack: i64) -> Observations {
        Observations {
            schema_version: 3,
            placement: vec![Placed {
                cell: "driver".into(),
                bel: "R12C34/SLICEA.K0".into(),
            }],
            timing: Timing {
                delay_model: model.into(),
                worst_slack_ps: Some(slack),
                net_setup_slacks: vec![EndpointSlack {
                    net_id: 1,
                    sink_pin_id: 7,
                    slack_ps: slack,
                }],
                net_delays: vec![Delay {
                    net_id: 1,
                    net: "n".into(),
                    driver_cell: "driver".into(),
                    sink_cell: "sink".into(),
                    sink_pin: "D".into(),
                    sink_pin_id: 7,
                    max_delay_ps: 1200,
                }],
            },
        }
    }
    #[test]
    fn uses_measured_delay_and_remaining_path_budget() {
        let f = convert(fixture("measured_joint_cell_route_min_max_ps", -300)).unwrap();
        assert_eq!(
            f.location("driver"),
            Some(PhysicalLocation { x: 34, y: 12 })
        );
        assert_eq!(f.net_timings()[0].endpoints[0].delay_ps, 1200);
        assert_eq!(f.net_timings()[0].endpoints[0].budget_ps, 900);
        assert!(f.critical_paths().is_empty());
        assert!(!f.is_near_timing_closure(1));
    }
    #[test]
    fn rejects_unmeasured_data_and_skips_passing_setup() {
        assert!(convert(fixture("estimated", -1)).is_err());
        assert!(
            convert(fixture("measured_joint_cell_route_min_max_ps", 1))
                .unwrap()
                .net_timings()
                .is_empty()
        );
        assert_eq!(
            convert(fixture("measured_joint_cell_route_min_max_ps", -2000))
                .unwrap()
                .net_timings()[0]
                .endpoints[0]
                .budget_ps,
            0
        );
    }
}
