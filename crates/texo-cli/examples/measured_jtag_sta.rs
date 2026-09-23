//! Supplemental JTCK-domain STA using the original Texo engine and frozen net
//! delays. Hard JTAGG timing and async reset recovery/removal remain external
//! boundaries, explicitly reported instead of inventing characterization.
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fs::File,
    io::BufReader,
};
use texo_model::{CellPinId, Design, PinDirection, ResourceKind};
use texo_target_ecp5::{DelayRangeRecord, read_architecture_cache};
use texo_timing::{
    ClockEdge, DelayRange, NetDelay, TimingConstraints, TimingModel, analyze_timing_from_net_delays,
};

fn u(v: &Value) -> u64 {
    v.as_u64().unwrap()
}
fn s(v: &Value) -> &str {
    v.as_str().unwrap()
}
fn delay(v: DelayRangeRecord) -> DelayRange {
    DelayRange::new(v.min_ps, v.max_ps).unwrap()
}

#[allow(clippy::too_many_lines)]
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    assert_eq!(args.len(), 3, "ARCH CHECKPOINT OUTPUT");
    let mut arch = read_architecture_cache(BufReader::new(File::open(&args[0])?))?;
    let cp: Value = serde_json::from_reader(BufReader::new(File::open(&args[1])?))?;
    let measured = cp
        .pointer("/target/measured_timing")
        .filter(|v| !v.is_null())
        .ok_or("measured checkpoint required")?;
    texo_cli::measured_timing::install(
        std::path::Path::new(s(&measured["path"])),
        &mut arch,
        s(&cp["target"]["package"]),
        Some(s(&measured["sha256"])),
    )?;

    let metadata: BTreeMap<_, _> = cp["primitive_metadata"]
        .as_array()
        .unwrap()
        .iter()
        .map(|m| (u(&m["cell_id"]), m))
        .collect();
    let delays = cp["timing"]["net_delays"].as_array().unwrap();
    let mut design = Design::new();
    let mut cells = BTreeMap::new();
    for cell in cp["placement"].as_array().unwrap() {
        cells.insert(
            u(&cell["cell_id"]),
            design.add_cell(s(&cell["cell"]), ResourceKind::Logic),
        );
    }
    let mut pin_info = BTreeMap::new();
    let mut by_net = BTreeMap::<u64, Vec<&Value>>::new();
    for d in delays {
        for (prefix, direction) in [
            ("driver", PinDirection::Output),
            ("sink", PinDirection::Input),
        ] {
            pin_info.insert(
                u(&d[format!("{prefix}_pin_id")]),
                (
                    u(&d[format!("{prefix}_cell_id")]),
                    s(&d[format!("{prefix}_pin")]),
                    direction,
                ),
            );
        }
        by_net.entry(u(&d["net_id"])).or_default().push(d);
    }
    let mut pins = BTreeMap::new();
    let mut named = BTreeMap::new();
    for (&old, &(cell, name, direction)) in &pin_info {
        let pin = design.add_pin(cells[&cell], name, direction)?;
        pins.insert(old, pin);
        named.insert((cell, name), pin);
    }
    let mut nets = BTreeMap::new();
    let mut net_delays = Vec::new();
    for (&old, ds) in &by_net {
        let driver = pins[&u(&ds[0]["driver_pin_id"])];
        assert!(ds.iter().all(|d| pins[&u(&d["driver_pin_id"])] == driver));
        let net = design.add_net(
            s(&ds[0]["net"]),
            driver,
            ds.iter().map(|d| pins[&u(&d["sink_pin_id"])]),
        )?;
        nets.insert(old, net);
        for d in ds {
            net_delays.push(NetDelay {
                net,
                sink: pins[&u(&d["sink_pin_id"])],
                delay: DelayRange::from_independent_corners(
                    u(&d["min_delay_ps"]),
                    u(&d["max_delay_ps"]),
                ),
            });
        }
    }
    let source = by_net
        .iter()
        .find(|(_, ds)| ds[0]["driver_pin"] == "JTCK")
        .ok_or("missing JTCK")?
        .0;
    let global = cp["packing"]["global_clocks"]
        .as_array()
        .unwrap()
        .iter()
        .find(|g| u(&g["source_net"]) == *source)
        .ok_or("missing JTCK global buffer")?;
    let clock_net = nets[&u(&global["global_net"])];
    let mut constraints = TimingConstraints::new();
    constraints.set_clock_period_ps(nets[source], 166_666);
    constraints.set_clock_period_ps(clock_net, 166_666);
    constraints.set_generated_clock(clock_net, nets[source], 1, 1, 0);
    let records: BTreeMap<_, _> = arch.speed_grades()["8_5G"]
        .cells
        .iter()
        .map(|r| (r.cell_type.as_str(), r))
        .collect();
    let general: BTreeSet<_> = cp["packing"]["general_routing_ffs"]
        .as_array()
        .unwrap()
        .iter()
        .map(u)
        .collect();
    let constants: BTreeSet<_> = by_net
        .values()
        .filter_map(|ds| {
            s(&ds[0]["driver_cell"])
                .starts_with("$PACKER_")
                .then_some(nets[&u(&ds[0]["net_id"])])
        })
        .collect();
    let mut incoming_inputs = BTreeMap::new();
    for route in cp["routes"].as_array().unwrap() {
        for pip in route["pips"].as_array().unwrap() {
            let flags = u(&pip["lutperm_flags"]);
            if flags & 0x4000 != 0 {
                incoming_inputs.insert(
                    u(&pip["to_wire_id"]),
                    ["A", "B", "C", "D"][(flags & 3) as usize],
                );
            }
        }
    }
    let mut physical_inputs = BTreeMap::new();
    for placement in cp["placement"].as_array().unwrap() {
        for pin in placement["bel_pins"].as_array().unwrap() {
            if let Some(&input) = incoming_inputs.get(&u(&pin["wire_id"])) {
                physical_inputs.insert((u(&placement["cell_id"]), s(&pin["name"])), input);
            }
        }
    }
    let mut model = TimingModel::new();
    let mut selected = BTreeSet::new();
    let mut other_tap_clocks = Vec::new();
    let mut rising = Vec::new();
    for (&old, &cell) in &cells {
        let pin = |name| named.get(&(old, name)).copied();
        let is_ff = metadata
            .get(&old)
            .is_some_and(|m| m["configuration"]["kind"] == "flip_flop");
        let is_jtag = design.cells()[cell.0]
            .name
            .starts_with("ff_board.jtag_debug_transport.");
        let on_jtck =
            is_ff && pin("CLK").is_some_and(|p| design.pins()[p.0].net() == Some(clock_net));
        if is_jtag && is_ff && !on_jtck {
            let name = &design.cells()[cell.0].name;
            if name.contains("mailbox_") || name.ends_with(".request_toggle") {
                other_tap_clocks.push(name.clone());
            }
        }
        let edge = if metadata
            .get(&old)
            .is_some_and(|m| m["configuration"]["edge"] == "falling")
        {
            ClockEdge::Falling
        } else {
            ClockEdge::Rising
        };
        if on_jtck {
            selected.insert(cell);
            if edge == ClockEdge::Rising {
                rising.push(design.cells()[cell.0].name.clone());
            }
        }
        let mut types = Vec::new();
        if on_jtck {
            types.push("TRELLIS_FF");
        } else if !is_ff {
            if pin("CLKI").is_some() && pin("CLKO").is_some() {
                types.push("DCCA");
            } else if ["A", "B", "C", "D"].iter().any(|&p| pin(p).is_some()) && pin("F").is_some() {
                types.push("TRELLIS_COMB");
            }
            if pin("OFX").is_some() {
                types.push(if pin("F1").is_some() {
                    "TRELLIS_PFUMX"
                } else {
                    "TRELLIS_L6MUX21"
                });
            }
        }
        for name in types {
            let record = records[name];
            for arc in &record.arcs {
                let (Some(from), Some(to)) = (pin(arc.from_pin.as_str()), pin(arc.to_pin.as_str()))
                else {
                    continue;
                };
                if on_jtck && arc.from_pin == "CLK" && arc.to_pin == "Q" {
                    model.add_clock_to_q(from, to, edge, delay(arc.delay))?;
                } else {
                    let physical_input = physical_inputs
                        .get(&(old, arc.from_pin.as_str()))
                        .copied()
                        .unwrap_or(arc.from_pin.as_str());
                    let physical_arc = record
                        .arcs
                        .iter()
                        .find(|a| a.from_pin == physical_input && a.to_pin == arc.to_pin)
                        .ok_or("missing physical input arc")?;
                    model.add_cell_arc(from, to, delay(physical_arc.delay))?;
                }
            }
            if on_jtck {
                for check in &record.setup_holds {
                    if check.signal_pin == "LSR"
                        || (check.signal_pin == "DI" && general.contains(&old))
                        || (check.signal_pin == "M" && !general.contains(&old))
                    {
                        continue;
                    }
                    let signal_name = if check.signal_pin == "M" {
                        "DI"
                    } else {
                        &check.signal_pin
                    };
                    let (Some(signal), Some(clock)) = (pin(signal_name), pin(&check.clock_pin))
                    else {
                        continue;
                    };
                    if design.pins()[signal.0]
                        .net()
                        .is_some_and(|n| constants.contains(&n))
                    {
                        continue;
                    }
                    model.add_setup_hold(
                        clock,
                        signal,
                        edge,
                        delay(check.setup),
                        delay(check.hold),
                    )?;
                }
            }
        }
    }
    let report = analyze_timing_from_net_delays(&design, &model, &constraints, net_delays)?;
    let describe = |p: CellPinId| json!({"cell":design.cells()[design.pins()[p.0].cell.0].name,"pin":design.pins()[p.0].name});
    let unchecked: Vec<_> = report
        .unchecked_endpoints
        .iter()
        .map(|e| json!({"endpoint":describe(e.data_pin),"reason":format!("{:?}",e.reason)}))
        .collect();
    let setup: Vec<_> = report.setup_checks.iter().map(|c| json!({"endpoint":describe(c.data_pin),"slack_ps":c.slack_ps,"launch_edge":c.launch_edge.as_str(),"capture_edge":c.capture_edge.as_str()})).collect();
    let hold: Vec<_> = report.hold_checks.iter().map(|c| json!({"endpoint":describe(c.data_pin),"slack_ps":c.slack_ps,"launch_edge":c.launch_edge.as_str(),"capture_edge":c.capture_edge.as_str()})).collect();
    // These are the three wholly external JTAGG input cones in this RTL.
    // A new missing endpoint is an error, even if all measured slacks are positive.
    let expected_boundaries: BTreeSet<_> = [
        "ff_board.jtag_debug_transport.shift_high[63]",
        "ff_board.jtag_debug_transport.shift_enable_q",
        "ff_board.jtag_debug_transport.update_q",
    ]
    .into_iter()
    .map(str::to_owned)
    .collect();
    let actual_boundaries: BTreeSet<_> = report
        .unchecked_endpoints
        .iter()
        .map(|e| {
            design.cells()[design.pins()[e.data_pin.0].cell.0]
                .name
                .clone()
        })
        .collect();
    let boundary_coverage_ok = actual_boundaries == expected_boundaries
        && report.unchecked_endpoints.len() == expected_boundaries.len()
        && report.unchecked_endpoints.iter().all(|e| {
            e.reason == texo_timing::UncheckedEndpointReason::NoSynchronousLaunch
                && design.pins()[e.data_pin.0].name == "DI"
        });
    let internal_coverage_ok = selected.len() == 261
        && report.setup_checks.len() == 515
        && report.hold_checks.len() == 515;
    let pass = report.met_timing()
        && rising.is_empty()
        && other_tap_clocks.is_empty()
        && boundary_coverage_ok
        && internal_coverage_ok;
    let result = json!({"scope":"JTCK register-to-register only; original pinned Texo STA and routed delays",
        "external_boundaries_not_characterized":["JTAGG launch/capture arcs", "JRSTN recovery/removal", "JTCK-to-CPU bundled-data CDC"],
        "jtag_tck_hz":6_000_000,"boundary_coverage_ok":boundary_coverage_ok,"internal_coverage_ok":internal_coverage_ok,"jtck_register_count":selected.len(),"register_paths_met_timing":report.met_timing(),
        "falling_edge_structure_ok":rising.is_empty(),"separate_update_clock_absent":other_tap_clocks.is_empty(),
        "rising_edge_registers":rising,"other_tap_clock_registers":other_tap_clocks,
        "worst_setup_slack_ps":report.worst_slack_ps,"worst_hold_slack_ps":report.worst_hold_slack_ps,
        "unchecked_endpoints":unchecked,"setup_checks":setup,"hold_checks":hold,"structural_and_internal_timing_gate":pass});
    serde_json::to_writer_pretty(File::create(&args[2])?, &result)?;
    println!(
        "JTCK FFs={} setup={} hold={} unchecked={} WNS={:?} WHS={:?} gate={pass}",
        selected.len(),
        report.setup_checks.len(),
        report.hold_checks.len(),
        report.unchecked_endpoints.len(),
        report.worst_slack_ps,
        report.worst_hold_slack_ps
    );
    if !pass {
        std::process::exit(1);
    }
    Ok(())
}
