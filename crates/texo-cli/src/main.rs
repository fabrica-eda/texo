//! Texo command-line entry point.

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs::File;
use std::io::{BufReader, BufWriter, Write};
use std::path::Path;
use std::process::ExitCode;
use std::time::Instant;

use celox::SimulatorBuilder;
use serde_json::{Value, json};
use struo_celox::ecp5_simulator;
use struo_example_axi4_smartconnect::{AXI4_CROSSBAR_SOURCE, axi4_crossbar_self_test};
use struo_ir::Netlist;
use struo_synth::synthesize;
use struo_target_ecp5::{Ecp5Cell, Ecp5Netlist, map_to_ecp5};
use texo_flow::{
    Ecp5FlowOptions, Ecp5FlowResult, Ecp5FlowStage, Evidence, Gate, RoutingProgress, implement,
    implement_struo_ecp5_with_progress, verify_post_map_with_celox,
};
use texo_model::{Design, Device, PinDirection, ResourceKind};
use texo_struo::{ActiveLevel, ClockEdge, PortDirection, PrimitiveMetadata, import_ecp5};
use texo_target_ecp5::{
    Ecp5Architecture, parse_lpf, read_architecture, read_architecture_cache,
    write_architecture_cache,
};

const USAGE: &str = "\
Texo FPGA place and route

Usage:
  texo demo                         run the deterministic abstract-grid PnR demo
  texo ecp5-demo <architecture> <package> <speed-grade> <constraints.lpf> [checkpoint.json]
                                    run a verified Struo/Celox ECP5 XOR flow
  texo axi4-pnr <architecture> <package> <speed-grade> <constraints.lpf> [checkpoint.json] [weight-exponent]
                                    run the Struo AXI4 self-test through native Texo PnR
  texo axi4-json <design.json>      export the same mapped AXI4 design for nextpnr
  texo target-info <architecture>   inspect an ECP5 architecture snapshot
  texo cache-architecture <architecture.json> <architecture.txdb>
                                    cache the expanded routing graph for fast reuse
  texo lpf-info <constraints.lpf>   inspect ECP5 pin, IO, and clock constraints
  texo help                         show this help
";

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("error: {error}");
            ExitCode::FAILURE
        }
    }
}

/// Parses the optional trailing placement weight exponent argument.
fn parse_weight_exponent(args: &mut impl Iterator<Item = String>) -> Result<u32, Box<dyn Error>> {
    match args.next() {
        Some(arg) => arg.parse::<u32>().map_err(|_| {
            format!("axi4-pnr placement weight exponent must be a positive integer\n\n{USAGE}")
                .into()
        }),
        None => Ok(1),
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    match args.next().as_deref() {
        Some("demo") if args.next().is_none() => demo(),
        Some("ecp5-demo") => {
            let architecture = args
                .next()
                .ok_or_else(|| format!("ecp5-demo requires an architecture path\n\n{USAGE}"))?;
            let package = args
                .next()
                .ok_or_else(|| format!("ecp5-demo requires a package name\n\n{USAGE}"))?;
            let speed_grade = args
                .next()
                .ok_or_else(|| format!("ecp5-demo requires a speed grade\n\n{USAGE}"))?;
            let lpf = args
                .next()
                .ok_or_else(|| format!("ecp5-demo requires an LPF path\n\n{USAGE}"))?;
            let checkpoint = args.next();
            if args.next().is_some() {
                return Err(format!("ecp5-demo accepts at most five arguments\n\n{USAGE}").into());
            }
            ecp5_demo(
                &architecture,
                &package,
                &speed_grade,
                &lpf,
                checkpoint.as_deref(),
            )
        }
        Some("axi4-pnr") => {
            let architecture = args
                .next()
                .ok_or_else(|| format!("axi4-pnr requires an architecture path\n\n{USAGE}"))?;
            let package = args
                .next()
                .ok_or_else(|| format!("axi4-pnr requires a package name\n\n{USAGE}"))?;
            let speed_grade = args
                .next()
                .ok_or_else(|| format!("axi4-pnr requires a speed grade\n\n{USAGE}"))?;
            let lpf = args
                .next()
                .ok_or_else(|| format!("axi4-pnr requires an LPF path\n\n{USAGE}"))?;
            let checkpoint = args.next();
            let exponent = parse_weight_exponent(&mut args)?;
            if args.next().is_some() {
                return Err(format!("axi4-pnr accepts at most six arguments\n\n{USAGE}").into());
            }
            axi4_pnr(
                &architecture,
                &package,
                &speed_grade,
                &lpf,
                checkpoint.as_deref(),
                exponent,
            )
        }
        Some("axi4-json") => parse_axi4_json(args),
        Some("target-info") => {
            let path = args
                .next()
                .ok_or_else(|| format!("target-info requires an architecture path\n\n{USAGE}"))?;
            if args.next().is_some() {
                return Err(format!("target-info accepts one architecture path\n\n{USAGE}").into());
            }
            target_info(&path)
        }
        Some("cache-architecture") => {
            let source = args.next().ok_or_else(|| {
                format!("cache-architecture requires a JSON architecture path\n\n{USAGE}")
            })?;
            let destination = args.next().ok_or_else(|| {
                format!("cache-architecture requires an output .txdb path\n\n{USAGE}")
            })?;
            if args.next().is_some() {
                return Err(format!(
                    "cache-architecture accepts a source and destination path\n\n{USAGE}"
                )
                .into());
            }
            cache_architecture(&source, &destination)
        }
        Some("lpf-info") => {
            let path = args
                .next()
                .ok_or_else(|| format!("lpf-info requires an LPF path\n\n{USAGE}"))?;
            if args.next().is_some() {
                return Err(format!("lpf-info accepts one LPF path\n\n{USAGE}").into());
            }
            lpf_info(&path)
        }
        None | Some("help" | "-h" | "--help") => {
            print!("{USAGE}");
            Ok(())
        }
        Some(command) => Err(format!("unknown command `{command}`\n\n{USAGE}").into()),
    }
}

fn parse_axi4_json(mut args: impl Iterator<Item = String>) -> Result<(), Box<dyn Error>> {
    let path = args
        .next()
        .ok_or_else(|| format!("axi4-json requires an output path\n\n{USAGE}"))?;
    if args.next().is_some() {
        return Err(format!("axi4-json accepts one argument\n\n{USAGE}").into());
    }
    axi4_json(&path)
}

fn axi4_json(path: &str) -> Result<(), Box<dyn Error>> {
    let rtl = axi4_crossbar_self_test()?;
    let synthesized = synthesize(&rtl)?;
    let mapped = map_to_ecp5(&synthesized.netlist)?;
    let mut output = BufWriter::new(File::create(path)?);
    output.write_all(lossless_nextpnr_json(&mapped)?.as_bytes())?;
    output.flush()?;
    println!(
        "nextpnr JSON: {path} ({} mapped cells)",
        mapped.cells().len()
    );
    Ok(())
}

fn lossless_nextpnr_json(mapped: &Ecp5Netlist) -> Result<String, Box<dyn Error>> {
    let mut document: Value = serde_json::from_str(&mapped.to_nextpnr_json()?)?;
    let mut totals = BTreeMap::<&str, usize>::new();
    for cell in mapped.cells() {
        *totals.entry(ecp5_cell_name(cell)).or_default() += 1;
    }

    let module = document["modules"]
        .get_mut(mapped.name())
        .and_then(Value::as_object_mut)
        .ok_or("Struo nextpnr JSON omitted its top module")?;
    let cells = module
        .get_mut("cells")
        .and_then(Value::as_object_mut)
        .ok_or("Struo nextpnr JSON omitted its cell map")?;
    let mut occurrences = BTreeMap::<&str, usize>::new();
    for cell in mapped.cells() {
        let name = ecp5_cell_name(cell);
        let total = totals[name];
        let occurrence = occurrences.entry(name).or_default();
        if total > 1 && *occurrence + 1 < total {
            let unique_name = format!("{name}$texo_duplicate{occurrence}");
            if cells
                .insert(unique_name.clone(), duplicate_nextpnr_cell(cell)?)
                .is_some()
            {
                return Err(format!("duplicate repair name `{unique_name}` already exists").into());
            }
        }
        *occurrence += 1;
    }
    if cells.len() != mapped.cells().len() {
        return Err(format!(
            "lossless nextpnr export contains {} cells, expected {}",
            cells.len(),
            mapped.cells().len()
        )
        .into());
    }
    Ok(serde_json::to_string_pretty(&document)?)
}

fn duplicate_nextpnr_cell(cell: &Ecp5Cell) -> Result<Value, Box<dyn Error>> {
    let Ecp5Cell::Lut4 {
        inputs,
        output,
        init,
        ..
    } = cell
    else {
        return Err(format!(
            "lossless nextpnr export does not yet support duplicate non-LUT cell `{}`",
            ecp5_cell_name(cell)
        )
        .into());
    };
    Ok(json!({
        "hide_name": 0,
        "type": "LUT4",
        "parameters": { "INIT": format!("{init:016b}") },
        "attributes": {},
        "port_directions": {
            "A": "input", "B": "input", "C": "input", "D": "input", "Z": "output"
        },
        "connections": {
            "A": [inputs[0]], "B": [inputs[1]], "C": [inputs[2]], "D": [inputs[3]],
            "Z": [output]
        }
    }))
}

fn ecp5_cell_name(cell: &Ecp5Cell) -> &str {
    match cell {
        Ecp5Cell::Lut4 { name, .. }
        | Ecp5Cell::Ccu2c { name, .. }
        | Ecp5Cell::FlipFlop { name, .. }
        | Ecp5Cell::BlockRam { name, .. } => name,
    }
}

fn lpf_info(path: &str) -> Result<(), Box<dyn Error>> {
    let constraints = parse_lpf(File::open(path)?)?;
    println!("locations: {}", constraints.locations().len());
    for (port, pin) in constraints.locations() {
        println!("  {port} -> {pin}");
    }
    println!("IOBUF ports: {}", constraints.io_attributes().len());
    for (port, attributes) in constraints.io_attributes() {
        let settings = attributes
            .iter()
            .map(|(key, value)| format!("{key}={value}"))
            .collect::<Vec<_>>()
            .join(" ");
        println!("  {port}: {settings}");
    }
    println!("clock ports: {}", constraints.frequencies_hz().len());
    for (port, frequency_hz) in constraints.frequencies_hz() {
        println!("  {port}: {frequency_hz} Hz");
    }
    println!(
        "unsupported commands: {}",
        constraints.unsupported_commands().len()
    );
    for command in constraints.unsupported_commands() {
        println!("  {command}");
    }
    Ok(())
}

fn target_info(path: &str) -> Result<(), Box<dyn Error>> {
    let architecture = load_architecture(path)?;
    let device = architecture.device();
    let fixed_pips = architecture
        .pip_metadata_iter()
        .filter(|(_, pip)| pip.fixed)
        .count();

    println!("device: {}", device.name());
    println!("grid: {} x {}", device.width(), device.height());
    println!("BELs: {}", device.bels().len());
    println!("BEL pins: {}", device.bel_pins().len());
    println!("wires: {}", device.wires().len());
    println!("PIPs: {} ({fixed_pips} fixed)", device.pips().len());
    let package_names = architecture
        .packages()
        .iter()
        .map(|package| package.name.as_str())
        .collect::<Vec<_>>()
        .join(", ");
    println!("packages: {package_names}");
    println!(
        "speed grades: {}",
        architecture
            .speed_grades()
            .keys()
            .map(String::as_str)
            .collect::<Vec<_>>()
            .join(", ")
    );
    println!(
        "Project Trellis revision: {}",
        architecture.provenance().project_trellis_revision
    );
    println!(
        "database revision: {}",
        architecture.provenance().database_revision
    );
    Ok(())
}

fn ecp5_demo(
    architecture_path: &str,
    package: &str,
    speed_grade: &str,
    lpf_path: &str,
    checkpoint_path: Option<&str>,
) -> Result<(), Box<dyn Error>> {
    let mut source = Netlist::new("xor");
    let lhs = source.add_input("lhs");
    let rhs = source.add_input("rhs");
    let value = source.add_xor(lhs, rhs);
    source.add_output("value", value);
    let mapped = map_to_ecp5(&source)?;
    let imported = import_ecp5(&mapped)?;
    let architecture = load_architecture(architecture_path)?;
    let lpf = parse_lpf(File::open(lpf_path)?)?;
    let mut evidence = Evidence::new();

    verify_post_map_with_celox(&mut evidence, || -> Result<(), Box<dyn Error>> {
        let mut simulator = ecp5_simulator(&mapped)?.build_native()?;
        let lhs_signal = simulator.signal("lhs");
        let rhs_signal = simulator.signal("rhs");
        let value_signal = simulator.signal("value");
        for (lhs, rhs, expected) in [(0_u8, 0_u8, 0_u8), (0, 1, 1), (1, 0, 1), (1, 1, 0)] {
            simulator.modify(|io| {
                io.set(lhs_signal, lhs);
                io.set(rhs_signal, rhs);
            })?;
            if simulator.get(value_signal) != expected.into() {
                return Err(format!(
                    "Celox XOR mismatch for lhs={lhs}, rhs={rhs}: expected {expected}"
                )
                .into());
            }
        }
        Ok(())
    })?;
    let mut phase_started = Instant::now();
    let result = implement_struo_ecp5_with_progress(
        &imported,
        &architecture,
        Ecp5FlowOptions {
            speed_grade: Some(speed_grade),
            package: Some(package),
            lpf: Some(&lpf),
            ..Ecp5FlowOptions::default()
        },
        &mut evidence,
        |stage| report_flow_stage(stage, &mut phase_started),
    )?;

    println!("Celox post-map XOR truth table: passed");
    println!(
        "device: {} ({package}, speed {speed_grade})",
        architecture.device().name()
    );
    println!(
        "packed: {} LUT/FF pairs, {} carry pairs, {} BRAMs, {} global clocks",
        result.packing.lut_ff_pairs().len(),
        result.packing.carry_pairs().len(),
        result.packing.block_rams().len(),
        result.packing.global_clocks().len()
    );
    println!(
        "placed {} cells and routed {} nets through {} PIPs",
        result.implementation.placement.bindings().len(),
        result.implementation.routes.len(),
        result.implementation.total_pips
    );
    match result.timing.worst_slack_ps {
        Some(slack_ps) => println!(
            "timing: {} setup/hold checks, worst setup {slack_ps} ps, worst hold {} ps ({})",
            result.timing.setup_checks.len(),
            result.timing.worst_hold_slack_ps.unwrap_or(0),
            if result.timing.met_timing() {
                "passed"
            } else {
                "failed"
            }
        ),
        None => println!("timing: no constrained sequential endpoints"),
    }
    for (cell_id, &bel_id) in result
        .implementation
        .placement
        .bindings()
        .iter()
        .enumerate()
    {
        let cell = &result.design.cells()[cell_id];
        let bel = &architecture.device().bels()[bel_id.0];
        println!(
            "  {:<20} -> {:<20} ({}, {})",
            cell.name, bel.name, bel.point.x, bel.point.y
        );
    }

    if let Some(path) = checkpoint_path {
        let checkpoint = ecp5_checkpoint("xor", &result, &architecture, package, &evidence);
        let destination = File::create(path)?;
        serde_json::to_writer_pretty(destination, &checkpoint)?;
        println!("checkpoint: {path}");
    }
    println!("physical implementation gate: passed");
    println!("bitstream release: blocked until RTL/equivalence/timing gates pass");
    Ok(())
}

fn axi4_pnr(
    architecture_path: &str,
    package: &str,
    speed_grade: &str,
    lpf_path: &str,
    checkpoint_path: Option<&str>,
    placement_weight_exponent: u32,
) -> Result<(), Box<dyn Error>> {
    let rtl = axi4_crossbar_self_test()?;
    let mut evidence = Evidence::new();
    verify_axi4_rtl(&mut evidence)?;

    let synthesized = synthesize(&rtl)?;
    let mapped = map_to_ecp5(&synthesized.netlist)?;
    if !mapped.retiming().equivalence_signed_off {
        return Err("Struo mapping/retiming equivalence sign-off failed".into());
    }
    evidence.record(Gate::SynthesisEquivalence);
    println!("Struo synthesis/mapping equivalence sign-off: passed");
    let imported = import_ecp5(&mapped)?;
    println!(
        "Struo AXI4 self-test: {} Boolean nodes, {} registers, {} mapped cells",
        synthesized.netlist.nodes().len(),
        synthesized.netlist.registers().len(),
        mapped.cells().len()
    );

    verify_post_map_with_celox(&mut evidence, || -> Result<(), Box<dyn Error>> {
        let mut simulator = ecp5_simulator(&mapped)?.build_native()?;
        let clock = simulator.event("clk");
        let reset = simulator.signal("rst_n");
        let passed = simulator.signal("passed");
        let failed = simulator.signal("failed");
        simulator.modify(|io| io.set(reset, 0_u8))?;
        simulator.tick(clock)?;
        simulator.modify(|io| io.set(reset, 1_u8))?;
        for _ in 0..12 {
            if simulator.get(passed) == 1_u8.into() {
                break;
            }
            simulator.tick(clock)?;
        }
        if simulator.get(passed) != 1_u8.into() || simulator.get(failed) != 0_u8.into() {
            return Err("Celox AXI4 self-test did not pass".into());
        }
        Ok(())
    })?;
    println!("Celox post-map AXI4 self-test: passed");

    let started = Instant::now();
    let architecture = load_architecture(architecture_path)?;
    println!("architecture loaded in {:.2?}", started.elapsed());
    let lpf = parse_lpf(File::open(lpf_path)?)?;
    let mut phase_started = Instant::now();
    let result = implement_struo_ecp5_with_progress(
        &imported,
        &architecture,
        Ecp5FlowOptions {
            speed_grade: Some(speed_grade),
            package: Some(package),
            lpf: Some(&lpf),
            placement_weight_exponent,
            ..Ecp5FlowOptions::default()
        },
        &mut evidence,
        |stage| report_flow_stage(stage, &mut phase_started),
    )?;

    println!(
        "native PnR: {} cells, {} routed nets, {} PIPs",
        result.implementation.placement.bindings().len(),
        result.implementation.routes.len(),
        result.implementation.total_pips
    );
    if let Some(slack_ps) = result.timing.worst_slack_ps {
        println!(
            "speed {speed_grade}: worst setup {slack_ps} ps, worst hold {} ps ({})",
            result.timing.worst_hold_slack_ps.unwrap_or(0),
            if result.timing.met_timing() {
                "passed"
            } else {
                "failed"
            }
        );
    }
    report_critical_setup_edges(&result, imported.design(), architecture.device());
    if let Some(path) = checkpoint_path {
        let checkpoint = ecp5_checkpoint(
            "axi4-crossbar-self-test",
            &result,
            &architecture,
            package,
            &evidence,
        );
        serde_json::to_writer_pretty(File::create(path)?, &checkpoint)?;
        println!("checkpoint: {path}");
    }
    Ok(())
}

fn verify_axi4_rtl(evidence: &mut Evidence) -> Result<(), Box<dyn Error>> {
    let mut rtl_simulator =
        SimulatorBuilder::new(AXI4_CROSSBAR_SOURCE, "Axi4CrossbarSelfTest").build_native()?;
    let rtl_clock = rtl_simulator.event("clk");
    let rtl_reset = rtl_simulator.signal("rst_n");
    let rtl_passed = rtl_simulator.signal("passed");
    let rtl_failed = rtl_simulator.signal("failed");
    rtl_simulator.modify(|io| io.set(rtl_reset, 0_u8))?;
    rtl_simulator.tick(rtl_clock)?;
    rtl_simulator.modify(|io| io.set(rtl_reset, 1_u8))?;
    for _ in 0..24 {
        if rtl_simulator.get(rtl_passed) == 1_u8.into() {
            break;
        }
        rtl_simulator.tick(rtl_clock)?;
    }
    if rtl_simulator.get(rtl_passed) != 1_u8.into() || rtl_simulator.get(rtl_failed) != 0_u8.into()
    {
        return Err("Celox RTL AXI4 self-test did not pass".into());
    }
    evidence.record(Gate::RtlSimulation);
    println!("Celox RTL AXI4 self-test: passed");
    Ok(())
}

fn report_critical_setup_edges(result: &Ecp5FlowResult, design: &Design, device: &Device) {
    let Some(worst_slack_ps) = result.timing.worst_slack_ps else {
        return;
    };
    for edge in result
        .timing
        .net_setup_slacks
        .iter()
        .filter(|edge| edge.slack_ps == worst_slack_ps)
    {
        let net = &design.nets()[edge.net.0];
        let driver_pin = &design.pins()[net.driver.0];
        let sink_pin = &design.pins()[edge.sink.0];
        let driver_cell = &design.cells()[driver_pin.cell.0];
        let sink_cell = &design.cells()[sink_pin.cell.0];
        let driver_bel = result.implementation.placement.bindings()[driver_pin.cell.0];
        let sink_bel = result.implementation.placement.bindings()[sink_pin.cell.0];
        let delay_ps = result
            .timing
            .net_delays
            .iter()
            .find(|delay| delay.net == edge.net && delay.sink == edge.sink)
            .map_or(0, |delay| delay.delay.max_ps);
        println!(
            "critical setup edge: net {} {}.{} @ {} -> {}.{} @ {}, delay {delay_ps} ps",
            net.name,
            driver_cell.name,
            driver_pin.name,
            device.bels()[driver_bel.0].name,
            sink_cell.name,
            sink_pin.name,
            device.bels()[sink_bel.0].name,
        );
    }
}

fn report_flow_stage(stage: Ecp5FlowStage, started: &mut Instant) {
    if let Ecp5FlowStage::CriticalPathMove { cell, from, to } = stage {
        println!(
            "critical-path placement trial: cell {}, BEL {} -> {}",
            cell.0, from.0, to.0
        );
        return;
    }
    if let Ecp5FlowStage::TimingTrialDecision { improves_objective } = stage {
        println!(
            "timing trial: {}",
            if improves_objective {
                "improves incumbent"
            } else {
                "rejected"
            }
        );
        return;
    }
    if let Ecp5FlowStage::TimingSnapshot {
        worst_setup_ps,
        setup_tns_ps,
        setup_violations,
        worst_hold_ps,
        hold_ths_ps,
        hold_violations,
    } = stage
    {
        println!(
            "timing trial: WNS {} ps, TNS {setup_tns_ps} ps ({setup_violations} violations), WHS {} ps, THS {hold_ths_ps} ps ({hold_violations} violations)",
            worst_setup_ps.map_or_else(|| "n/a".into(), |value| value.to_string()),
            worst_hold_ps.map_or_else(|| "n/a".into(), |value| value.to_string()),
        );
        return;
    }
    if let Ecp5FlowStage::Routing(event) | Ecp5FlowStage::TimingDrivenRouting(event) = stage {
        let label = match stage {
            Ecp5FlowStage::Routing(_) => "routing",
            Ecp5FlowStage::TimingDrivenRouting(_) => "timing-driven routing",
            _ => unreachable!("routing variants were matched above"),
        };
        match event {
            RoutingProgress::Iteration { iteration, nets } => {
                println!("{label} iteration {}: {nets} nets", iteration + 1);
            }
            RoutingProgress::Net {
                iteration,
                ordinal,
                total,
                net,
            } => {
                if env::var_os("TEXO_ROUTE_TRACE").is_some() {
                    println!(
                        "{label} iteration {} net {ordinal}/{total}: {}",
                        iteration + 1,
                        net.0
                    );
                }
            }
        }
        return;
    }
    let name = match stage {
        Ecp5FlowStage::Packed => "packing",
        Ecp5FlowStage::Placed => "placement",
        Ecp5FlowStage::GlobalClocksRouted => "global clock routing",
        Ecp5FlowStage::Routing(_) => unreachable!("routing progress returned above"),
        Ecp5FlowStage::Routed => "negotiated routing",
        Ecp5FlowStage::TimingDrivenPlaced => "timing-driven placement",
        Ecp5FlowStage::CriticalPathMove { .. } => {
            unreachable!("critical-path move returned above")
        }
        Ecp5FlowStage::TimingDrivenGlobalClocksRouted => "timing-driven global clock routing",
        Ecp5FlowStage::TimingDrivenRouting(_) => {
            unreachable!("routing progress returned above")
        }
        Ecp5FlowStage::TimingDrivenRouted => "timing-driven negotiated routing",
        Ecp5FlowStage::TimingSnapshot { .. } => {
            unreachable!("timing snapshot returned above")
        }
        Ecp5FlowStage::TimingTrialDecision { .. } => {
            unreachable!("timing decision returned above")
        }
        Ecp5FlowStage::Timed => "timing analysis",
    };
    println!("{name} completed in {:.2?}", started.elapsed());
    *started = Instant::now();
}

fn load_architecture(path: &str) -> Result<Ecp5Architecture, Box<dyn Error>> {
    let reader = BufReader::new(File::open(path)?);
    if Path::new(path)
        .extension()
        .and_then(|extension| extension.to_str())
        == Some("txdb")
    {
        Ok(read_architecture_cache(reader)?)
    } else {
        Ok(read_architecture(reader)?)
    }
}

fn cache_architecture(source: &str, destination: &str) -> Result<(), Box<dyn Error>> {
    let started = Instant::now();
    let architecture = read_architecture(BufReader::new(File::open(source)?))?;
    println!("architecture expanded in {:.2?}", started.elapsed());
    let started = Instant::now();
    write_architecture_cache(BufWriter::new(File::create(destination)?), &architecture)?;
    println!(
        "architecture cache written to {destination} in {:.2?}",
        started.elapsed()
    );
    Ok(())
}

fn ecp5_checkpoint(
    design_name: &str,
    result: &Ecp5FlowResult,
    architecture: &Ecp5Architecture,
    package: &str,
    evidence: &Evidence,
) -> Value {
    let device = architecture.device();
    json!({
        "schema_version": 1,
        "design": design_name,
        "target": {
            "family": "ECP5",
            "device": device.name(),
            "package": package,
            "speed_grade": result.speed_grade,
            "placement_weight_exponent": result.placement_weight_exponent,
            "project_trellis_revision": architecture.provenance().project_trellis_revision,
            "database_revision": architecture.provenance().database_revision,
        },
        "evidence": checkpoint_evidence(evidence),
        "metrics": {
            "cells": result.design.cells().len(),
            "nets": result.design.nets().len(),
            "routed_nets": result.implementation.routes.len(),
            "total_pips": result.implementation.total_pips,
        },
        "primitive_metadata": result.primitive_metadata.iter().map(|(cell, metadata)| {
            primitive_metadata_json(*cell, metadata, &result.design)
        }).collect::<Vec<_>>(),
        "absorbed_inputs": checkpoint_absorbed_inputs(result),
        "packing": checkpoint_packing(result),
        "placement": checkpoint_placement(result, architecture),
        "routes": checkpoint_routes(result, architecture),
        "timing": checkpoint_timing(result),
    })
}

fn checkpoint_placement(result: &Ecp5FlowResult, architecture: &Ecp5Architecture) -> Vec<Value> {
    result
        .implementation
        .placement
        .bindings()
        .iter()
        .enumerate()
        .map(|(cell_id, bel_id)| {
            let cell = &result.design.cells()[cell_id];
            let bel = &architecture.device().bels()[bel_id.0];
            json!({
                "cell_id": cell_id,
                "cell": cell.name,
                "bel_id": bel_id.0,
                "bel": bel.name,
                "x": bel.point.x,
                "y": bel.point.y,
            })
        })
        .collect()
}

fn checkpoint_routes(result: &Ecp5FlowResult, architecture: &Ecp5Architecture) -> Vec<Value> {
    let device = architecture.device();
    result
        .implementation
        .routes
        .iter()
        .map(|route| {
            let net = &result.design.nets()[route.net.0];
            let driver_pin = &result.design.pins()[net.driver.0];
            let driver_bel = result.implementation.placement.bindings()[driver_pin.cell.0];
            let driver_bel_pin = result
                .implementation
                .placement
                .pin_binding(net.driver)
                .or_else(|| {
                    device.bels()[driver_bel.0]
                        .pins()
                        .iter()
                        .copied()
                        .find(|bel_pin| {
                            let physical = &device.bel_pins()[bel_pin.0];
                            physical.name == driver_pin.name
                                && physical.direction == driver_pin.direction
                        })
                })
                .expect("a routed net driver has a physical BEL pin");
            let driver_wire = device.bel_pins()[driver_bel_pin.0].wire;
            let wires = route
                .wires()
                .map(|wire| json!({ "wire_id": wire.0, "wire": device.wires()[wire.0].name }))
                .collect::<Vec<_>>();
            let pips = route
                .pips()
                .map(|pip_id| {
                    let pip = &device.pips()[pip_id.0];
                    json!({
                        "pip_id": pip_id.0,
                        "from_wire_id": pip.from().0,
                        "from": device.wires()[pip.from().0].name,
                        "to_wire_id": pip.to().0,
                        "to": device.wires()[pip.to().0].name,
                        "bidirectional": pip.bidirectional(),
                        "fixed": architecture.pip_metadata(pip_id).fixed,
                    })
                })
                .collect::<Vec<_>>();
            json!({
                "net_id": route.net.0,
                "net": net.name,
                "driver_wire_id": driver_wire.0,
                "driver_wire": device.wires()[driver_wire.0].name,
                "wires": wires,
                "pips": pips,
            })
        })
        .collect()
}

fn checkpoint_absorbed_inputs(result: &Ecp5FlowResult) -> Vec<Value> {
    result
        .absorbed_inputs
        .iter()
        .map(|(cell, pins)| {
            json!({
                "cell_id": cell.0,
                "cell": result.design.cells()[cell.0].name,
                "pins": pins,
            })
        })
        .collect()
}

fn checkpoint_packing(result: &Ecp5FlowResult) -> Value {
    let lut_ff_pairs = result
        .packing
        .lut_ff_pairs()
        .iter()
        .map(|pair| json!({ "lut": pair.lut.0, "ff": pair.ff.0 }))
        .collect::<Vec<_>>();
    let carry_pairs = result
        .packing
        .carry_pairs()
        .iter()
        .map(|pair| json!({ "first": pair[0].0, "second": pair[1].0 }))
        .collect::<Vec<_>>();
    let block_rams = result
        .packing
        .block_rams()
        .iter()
        .map(|ram| {
            json!({
                "cell": ram.cell.0,
                "wid": ram.wid,
                "depth": ram.depth,
                "word_width": ram.word_width,
                "physical_width": ram.physical_width,
            })
        })
        .collect::<Vec<_>>();
    let global_clocks = result
        .packing
        .global_clocks()
        .iter()
        .map(|clock| {
            json!({
                "source_net": clock.source_net.0,
                "buffer": clock.buffer.0,
                "global_net": clock.global_net.0,
            })
        })
        .collect::<Vec<_>>();
    let io_attributes = result
        .packing
        .io_attributes()
        .iter()
        .map(|(cell, attributes)| {
            json!({
                "cell_id": cell.0,
                "cell": result.design.cells()[cell.0].name,
                "attributes": attributes,
            })
        })
        .collect::<Vec<_>>();
    let clock_frequencies_hz = result
        .packing
        .clock_frequencies_hz()
        .iter()
        .map(|(cell, frequency_hz)| {
            json!({
                "cell_id": cell.0,
                "cell": result.design.cells()[cell.0].name,
                "frequency_hz": frequency_hz,
            })
        })
        .collect::<Vec<_>>();
    json!({
        "lut_ff_pairs": lut_ff_pairs,
        "carry_pairs": carry_pairs,
        "general_routing_ffs": result.packing.general_routing_ffs().iter().map(|cell| cell.0).collect::<Vec<_>>(),
        "block_rams": block_rams,
        "global_clocks": global_clocks,
        "io_attributes": io_attributes,
        "clock_frequencies_hz": clock_frequencies_hz,
        "unsupported_lpf_commands": result.packing.unsupported_lpf_commands(),
    })
}

fn checkpoint_timing(result: &Ecp5FlowResult) -> Value {
    let net_delays = result
        .timing
        .net_delays
        .iter()
        .map(|delay| {
            let net = &result.design.nets()[delay.net.0];
            let driver = &result.design.pins()[net.driver.0];
            let sink = &result.design.pins()[delay.sink.0];
            json!({
                "net_id": delay.net.0,
                "net": net.name,
                "driver_pin_id": net.driver.0,
                "driver_pin": driver.name,
                "driver_cell_id": driver.cell.0,
                "driver_cell": result.design.cells()[driver.cell.0].name,
                "sink_pin_id": delay.sink.0,
                "sink_pin": sink.name,
                "sink_cell_id": sink.cell.0,
                "sink_cell": result.design.cells()[sink.cell.0].name,
                "min_delay_ps": delay.delay.min_ps,
                "max_delay_ps": delay.delay.max_ps,
            })
        })
        .collect::<Vec<_>>();
    let setup_checks = result
        .timing
        .setup_checks
        .iter()
        .map(|check| {
            json!({
                "cell_id": check.cell.0,
                "cell": result.design.cells()[check.cell.0].name,
                "data_pin_id": check.data_pin.0,
                "clock_net_id": check.clock_net.0,
                "arrival_ps": check.arrival_ps,
                "clock_arrival_ps": check.clock_arrival_ps,
                "setup_ps": check.setup_ps,
                "required_ps": check.required_ps,
                "slack_ps": check.slack_ps,
            })
        })
        .collect::<Vec<_>>();
    let net_setup_slacks = result
        .timing
        .net_setup_slacks
        .iter()
        .map(|edge| {
            json!({
                "net_id": edge.net.0,
                "net": result.design.nets()[edge.net.0].name,
                "sink_pin_id": edge.sink.0,
                "slack_ps": edge.slack_ps,
            })
        })
        .collect::<Vec<_>>();
    let hold_checks = result
        .timing
        .hold_checks
        .iter()
        .map(|check| {
            json!({
                "cell_id": check.cell.0,
                "cell": result.design.cells()[check.cell.0].name,
                "data_pin_id": check.data_pin.0,
                "clock_net_id": check.clock_net.0,
                "arrival_ps": check.arrival_ps,
                "clock_arrival_ps": check.clock_arrival_ps,
                "hold_ps": check.hold_ps,
                "required_ps": check.required_ps,
                "slack_ps": check.slack_ps,
            })
        })
        .collect::<Vec<_>>();
    json!({
        "delay_model": "project_trellis_speed_grade_min_max_ps",
        "net_delays": net_delays,
        "net_setup_slacks": net_setup_slacks,
        "setup_checks": setup_checks,
        "hold_checks": hold_checks,
        "worst_slack_ps": result.timing.worst_slack_ps,
        "worst_hold_slack_ps": result.timing.worst_hold_slack_ps,
        "met_timing": result.timing.met_timing(),
    })
}

fn checkpoint_evidence(evidence: &Evidence) -> Vec<&'static str> {
    [
        (Gate::RtlSimulation, "rtl_simulation"),
        (Gate::SynthesisEquivalence, "synthesis_equivalence"),
        (Gate::MappedNetlistComplete, "mapped_netlist_complete"),
        (Gate::PostMapSimulation, "post_map_simulation"),
        (Gate::PhysicalImplementation, "physical_implementation"),
        (Gate::TimingClosure, "timing_closure"),
    ]
    .into_iter()
    .filter_map(|(gate, name)| evidence.contains(gate).then_some(name))
    .collect()
}

fn primitive_metadata_json(
    cell: texo_model::CellId,
    metadata: &PrimitiveMetadata,
    design: &Design,
) -> Value {
    let configuration = match metadata {
        PrimitiveMetadata::Lut4 { init } => json!({ "kind": "lut4", "init": init }),
        PrimitiveMetadata::CarrySlice {
            init,
            inject,
            slice,
        } => json!({
            "kind": "carry_slice",
            "init": init,
            "inject": inject,
            "slice": slice,
        }),
        PrimitiveMetadata::FlipFlop {
            edge,
            enable,
            reset,
        } => json!({
            "kind": "flip_flop",
            "edge": clock_edge_name(*edge),
            "enable": enable.map(active_level_name),
            "reset": reset.as_ref().map(|reset| json!({
                "active": active_level_name(reset.active),
                "asynchronous": reset.asynchronous,
                "value": reset.value,
            })),
        }),
        PrimitiveMetadata::BlockRam {
            depth,
            word_width,
            physical_width,
            edge,
            write_enable,
            read_enable,
        } => json!({
            "kind": "block_ram",
            "depth": depth,
            "word_width": word_width,
            "physical_width": physical_width,
            "edge": clock_edge_name(*edge),
            "write_enable": active_level_name(*write_enable),
            "read_enable": read_enable.map(active_level_name),
        }),
        PrimitiveMetadata::Port {
            name,
            bit,
            direction,
        } => json!({
            "kind": "port",
            "name": name,
            "bit": bit,
            "direction": match direction {
                PortDirection::Input => "input",
                PortDirection::Output => "output",
            },
        }),
        PrimitiveMetadata::Constant { value } => {
            json!({ "kind": "constant", "value": value })
        }
    };
    json!({
        "cell_id": cell.0,
        "cell": design.cells()[cell.0].name,
        "configuration": configuration,
    })
}

const fn clock_edge_name(edge: ClockEdge) -> &'static str {
    match edge {
        ClockEdge::Rising => "rising",
        ClockEdge::Falling => "falling",
    }
}

const fn active_level_name(level: ActiveLevel) -> &'static str {
    match level {
        ActiveLevel::High => "high",
        ActiveLevel::Low => "low",
    }
}

fn demo() -> Result<(), Box<dyn Error>> {
    let mut design = Design::new();
    let input = design.add_cell("input_buffer", ResourceKind::Logic);
    let input_out = design.add_pin(input, "out", PinDirection::Output)?;
    let lut = design.add_cell("lut4", ResourceKind::Logic);
    let lut_in = design.add_pin(lut, "in", PinDirection::Input)?;
    let lut_out = design.add_pin(lut, "out", PinDirection::Output)?;
    let register = design.add_cell("trellis_ff", ResourceKind::Logic);
    let register_in = design.add_pin(register, "in", PinDirection::Input)?;
    let register_out = design.add_pin(register, "out", PinDirection::Output)?;
    let output = design.add_cell("output_buffer", ResourceKind::Logic);
    let output_in = design.add_pin(output, "in", PinDirection::Input)?;
    design.add_net("input_to_lut", input_out, [lut_in])?;
    design.add_net("lut_to_ff", lut_out, [register_in])?;
    design.add_net("ff_to_output", register_out, [output_in])?;

    let device = Device::rectangular_logic(8, 8)?;
    let mut evidence = Evidence::new();
    let result = implement(&design, &device, &mut evidence)?;

    println!("placed {} cells", result.placement.bindings().len());
    for (id, bel) in result.placement.bindings().iter().enumerate() {
        let physical = &device.bels()[bel.0];
        println!(
            "  {:<16} -> {:<14} ({}, {})",
            design.cells()[id].name,
            physical.name,
            physical.point.x,
            physical.point.y
        );
    }
    println!(
        "routed {} nets through {} PIPs",
        result.routes.len(),
        result.total_pips
    );
    println!("physical implementation gate: passed");
    println!("bitstream release: blocked until simulation and timing gates pass");
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;
    use std::fs;

    use serde_json::Value;
    use struo_example_axi4_smartconnect::axi4_crossbar_self_test;
    use struo_synth::synthesize;
    use struo_target_ecp5::map_to_ecp5;

    use super::{ecp5_cell_name, ecp5_demo, lossless_nextpnr_json};

    const ARCHITECTURE: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../texo-target-ecp5/fixtures/minimal-ecp5.json"
    );
    const LPF: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../texo-target-ecp5/fixtures/xor.lpf"
    );

    #[test]
    fn ecp5_demo_writes_a_deterministic_checkpoint() {
        let temporary = std::env::temp_dir();
        let process = std::process::id();
        let first = temporary.join(format!("texo-ecp5-demo-{process}-first.json"));
        let second = temporary.join(format!("texo-ecp5-demo-{process}-second.json"));
        let first_path = first.to_string_lossy();
        let second_path = second.to_string_lossy();

        ecp5_demo(ARCHITECTURE, "CABGA381", "6", LPF, Some(&first_path)).unwrap();
        ecp5_demo(ARCHITECTURE, "CABGA381", "6", LPF, Some(&second_path)).unwrap();
        let first_bytes = fs::read(&first).unwrap();
        let second_bytes = fs::read(&second).unwrap();
        let checkpoint: Value = serde_json::from_slice(&first_bytes).unwrap();

        assert_eq!(first_bytes, second_bytes);
        assert_eq!(checkpoint["schema_version"], 1);
        assert_eq!(checkpoint["target"]["package"], "CABGA381");
        assert_eq!(checkpoint["target"]["speed_grade"], "6");
        assert_eq!(checkpoint["metrics"]["cells"], 4);
        assert_eq!(checkpoint["metrics"]["routed_nets"], 3);
        assert_eq!(checkpoint["placement"].as_array().unwrap().len(), 4);
        assert_eq!(checkpoint["routes"].as_array().unwrap().len(), 3);
        assert_eq!(
            checkpoint["timing"]["setup_checks"]
                .as_array()
                .unwrap()
                .len(),
            0
        );
        assert_eq!(
            checkpoint["timing"]["hold_checks"]
                .as_array()
                .unwrap()
                .len(),
            0
        );
        assert_eq!(checkpoint["timing"]["met_timing"], false);
        assert!(
            checkpoint["evidence"]
                .as_array()
                .unwrap()
                .contains(&Value::String("post_map_simulation".into()))
        );

        fs::remove_file(first).unwrap();
        fs::remove_file(second).unwrap();
    }

    #[test]
    fn axi4_nextpnr_export_preserves_every_mapped_cell() {
        let rtl = axi4_crossbar_self_test().unwrap();
        let synthesized = synthesize(&rtl).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        let unique_names = mapped
            .cells()
            .iter()
            .map(ecp5_cell_name)
            .collect::<BTreeSet<_>>();
        // Struo keeps retimed replica names unique since PR #35, so the
        // lossless exporter must pass every cell through unchanged.
        assert_eq!(unique_names.len(), mapped.cells().len());

        let document: Value =
            serde_json::from_str(&lossless_nextpnr_json(&mapped).unwrap()).unwrap();
        let cells = document["modules"][mapped.name()]["cells"]
            .as_object()
            .unwrap();
        assert_eq!(cells.len(), mapped.cells().len());
        assert!(cells.keys().all(|name| !name.contains("$texo_duplicate")));
    }
}
