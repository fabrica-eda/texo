//! Design-specific Texo flows kept out of the general-purpose CLI.

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
use texo_cli::{ecp5_checkpoint_ref, write_checkpoint_visualizer};
use texo_flow::{
    Ecp5FlowOptions, Ecp5FlowResult, Ecp5FlowStage, Evidence, Gate, RoutingProgress, implement,
    implement_struo_ecp5_with_progress, verify_post_map_with_celox,
};
use texo_model::{Design, Device, PinDirection, ResourceKind};
use texo_struo::import_ecp5;
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
  texo axi4-route-nextpnr-placement <architecture> <package> <speed-grade> <constraints.lpf> <nextpnr-placed.json> [checkpoint.json]
                                    route and time a fixed nextpnr placement without closure
  texo axi4-json <design.json>      export the same mapped AXI4 design for nextpnr
  texo target-info <architecture>   inspect an ECP5 architecture snapshot
  texo cache-architecture <architecture.json> <architecture.txdb>
                                    cache the expanded routing graph for fast reuse
  texo visualize <checkpoint.json> [output.html]
                                    render placement and routes as interactive HTML/SVG
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
        None => Ok(4),
    }
}

fn parse_axi4_route_nextpnr_placement(
    mut args: impl Iterator<Item = String>,
) -> Result<(), Box<dyn Error>> {
    let architecture = args.next().ok_or_else(|| {
        format!("axi4-route-nextpnr-placement requires an architecture path\n\n{USAGE}")
    })?;
    let package = args.next().ok_or_else(|| {
        format!("axi4-route-nextpnr-placement requires a package name\n\n{USAGE}")
    })?;
    let speed_grade = args
        .next()
        .ok_or_else(|| format!("axi4-route-nextpnr-placement requires a speed grade\n\n{USAGE}"))?;
    let lpf = args
        .next()
        .ok_or_else(|| format!("axi4-route-nextpnr-placement requires an LPF path\n\n{USAGE}"))?;
    let placement = args.next().ok_or_else(|| {
        format!("axi4-route-nextpnr-placement requires a nextpnr JSON path\n\n{USAGE}")
    })?;
    let checkpoint = args.next();
    if args.next().is_some() {
        return Err(format!(
            "axi4-route-nextpnr-placement accepts at most six arguments\n\n{USAGE}"
        )
        .into());
    }
    axi4_route_nextpnr_placement(
        &architecture,
        &package,
        &speed_grade,
        &lpf,
        &placement,
        checkpoint.as_deref(),
    )
}

fn parse_visualize(mut args: impl Iterator<Item = String>) -> Result<(), Box<dyn Error>> {
    let checkpoint = args
        .next()
        .ok_or_else(|| format!("visualize requires a checkpoint path\n\n{USAGE}"))?;
    let output = args.next().unwrap_or_else(|| format!("{checkpoint}.html"));
    if args.next().is_some() {
        return Err(
            format!("visualize accepts a checkpoint and optional output path\n\n{USAGE}").into(),
        );
    }
    write_checkpoint_visualizer(&checkpoint, &output)?;
    println!("visualizer: {output}");
    Ok(())
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
        Some("axi4-route-nextpnr-placement") => parse_axi4_route_nextpnr_placement(args),
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
        Some("visualize") => parse_visualize(args),
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
        | Ecp5Cell::PfuMux { name, .. }
        | Ecp5Cell::L6Mux21 { name, .. }
        | Ecp5Cell::Ccu2c { name, .. }
        | Ecp5Cell::FlipFlop { name, .. }
        | Ecp5Cell::BlockRam { name, .. }
        | Ecp5Cell::TrellisIo { name, .. }
        | Ecp5Cell::Jtagg { name, .. }
        | Ecp5Cell::Pll { name, .. } => name,
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
            "timing: {}/{} endpoints checked, worst setup {slack_ps} ps, worst hold {} ps ({})",
            result.timing.setup_checks.len(),
            result.timing.modeled_endpoint_count(),
            result.timing.worst_hold_slack_ps.unwrap_or(0),
            if result.timing.met_timing() {
                "passed"
            } else {
                "failed"
            }
        ),
        None => println!(
            "timing: no constrained sequential endpoints (0/{} endpoints checked)",
            result.timing.modeled_endpoint_count()
        ),
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
        let checkpoint = ecp5_checkpoint_ref("xor", &result, &architecture, package, &evidence);
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
    axi4_pnr_with_initial_placement(
        architecture_path,
        package,
        speed_grade,
        lpf_path,
        checkpoint_path,
        placement_weight_exponent,
        None,
        None,
        true,
    )
}

fn axi4_route_nextpnr_placement(
    architecture_path: &str,
    package: &str,
    speed_grade: &str,
    lpf_path: &str,
    placement_path: &str,
    checkpoint_path: Option<&str>,
) -> Result<(), Box<dyn Error>> {
    let placement = read_nextpnr_placement(placement_path)?;
    println!(
        "nextpnr placement: {} shared logical cells, {} dedicated LUT/FF pairs (synthetic carry cells inferred by Texo)",
        placement.bindings.len(),
        placement.lut_ff_pairs.len()
    );
    axi4_pnr_with_initial_placement(
        architecture_path,
        package,
        speed_grade,
        lpf_path,
        checkpoint_path,
        1,
        Some(&placement.bindings),
        Some(&placement.lut_ff_pairs),
        false,
    )
}

#[allow(clippy::too_many_arguments)]
fn axi4_pnr_with_initial_placement(
    architecture_path: &str,
    package: &str,
    speed_grade: &str,
    lpf_path: &str,
    checkpoint_path: Option<&str>,
    placement_weight_exponent: u32,
    initial_placement: Option<&BTreeMap<String, String>>,
    lut_ff_pairs: Option<&BTreeMap<String, String>>,
    optimize_timing: bool,
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
            initial_placement,
            lut_ff_pairs,
            initial_timing_reroute: initial_placement.is_some(),
            optimize_timing,
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
        let checkpoint = ecp5_checkpoint_ref(
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

struct NextpnrPlacement {
    bindings: BTreeMap<String, String>,
    lut_ff_pairs: BTreeMap<String, String>,
}

fn read_nextpnr_placement(path: &str) -> Result<NextpnrPlacement, Box<dyn Error>> {
    let document: Value = serde_json::from_reader(BufReader::new(File::open(path)?))?;
    let modules = document["modules"]
        .as_object()
        .ok_or("nextpnr placement JSON omitted its module map")?;
    if modules.len() != 1 {
        return Err(format!(
            "nextpnr placement JSON contains {} modules, expected one",
            modules.len()
        )
        .into());
    }
    let cells = modules
        .values()
        .next()
        .and_then(|module| module["cells"].as_object())
        .ok_or("nextpnr placement JSON omitted its cell map")?;
    let mut comb_by_bel = BTreeMap::new();
    for (name, cell) in cells {
        if cell["type"] == "TRELLIS_COMB" {
            let bel = cell["attributes"]["NEXTPNR_BEL"]
                .as_str()
                .ok_or_else(|| format!("nextpnr cell `{name}` has no NEXTPNR_BEL"))?;
            comb_by_bel.insert(bel, name.as_str());
        }
    }
    let mut bindings = BTreeMap::new();
    let mut lut_ff_pairs = BTreeMap::new();
    for (nextpnr_name, cell) in cells {
        let cell_type = cell["type"]
            .as_str()
            .ok_or_else(|| format!("nextpnr cell `{nextpnr_name}` has no type"))?;
        if nextpnr_name.starts_with("$nextpnr_CCU2C_") || cell_type == "DCCA" {
            continue;
        }
        let nextpnr_bel = cell["attributes"]["NEXTPNR_BEL"]
            .as_str()
            .ok_or_else(|| format!("nextpnr cell `{nextpnr_name}` has no NEXTPNR_BEL"))?;
        let texo_name = nextpnr_cell_to_texo(nextpnr_name, cell_type)?;
        let texo_bel = nextpnr_bel_to_texo(nextpnr_bel)?;
        if bindings.insert(texo_name.clone(), texo_bel).is_some() {
            return Err(format!("nextpnr placement maps Texo cell `{texo_name}` twice").into());
        }
        if cell_type == "TRELLIS_FF"
            && cell["parameters"]["SD"]
                .as_str()
                .is_some_and(|value| value.trim() == "1")
        {
            let local_lut_bel = nextpnr_bel
                .strip_suffix(".FF0")
                .map(|site| format!("{site}.K0"))
                .or_else(|| {
                    nextpnr_bel
                        .strip_suffix(".FF1")
                        .map(|site| format!("{site}.K1"))
                })
                .ok_or_else(|| {
                    format!("paired nextpnr FF `{nextpnr_name}` has invalid BEL `{nextpnr_bel}`")
                })?;
            let nextpnr_lut = comb_by_bel.get(local_lut_bel.as_str()).ok_or_else(|| {
                format!("paired nextpnr FF `{nextpnr_name}` has no LUT at `{local_lut_bel}`")
            })?;
            let texo_lut = nextpnr_cell_to_texo(nextpnr_lut, "TRELLIS_COMB")?;
            if lut_ff_pairs
                .insert(texo_lut.clone(), texo_name.clone())
                .is_some()
            {
                return Err(format!("nextpnr pairs Texo LUT `{texo_lut}` twice").into());
            }
        }
    }
    Ok(NextpnrPlacement {
        bindings,
        lut_ff_pairs,
    })
}

fn nextpnr_cell_to_texo(name: &str, cell_type: &str) -> Result<String, Box<dyn Error>> {
    if let Some(base) = name.strip_suffix("$CCU2_COMB0") {
        return Ok(format!("{base}$slice0"));
    }
    if let Some(base) = name.strip_suffix("$CCU2_COMB1") {
        return Ok(format!("{base}$slice1"));
    }
    if cell_type == "TRELLIS_IO" {
        let port = name
            .strip_suffix("$tr_io")
            .ok_or_else(|| format!("nextpnr IO cell `{name}` lacks the `$tr_io` suffix"))?;
        return Ok(format!("${port}[0]"));
    }
    Ok(name.to_owned())
}

fn nextpnr_bel_to_texo(name: &str) -> Result<String, Box<dyn Error>> {
    let (x, rest) = name
        .strip_prefix('X')
        .and_then(|name| name.split_once("/Y"))
        .ok_or_else(|| format!("nextpnr BEL `{name}` does not begin with X#/Y#"))?;
    let (y, bel) = rest
        .split_once('/')
        .ok_or_else(|| format!("nextpnr BEL `{name}` has no site name"))?;
    let column = x
        .parse::<u32>()
        .map_err(|_| format!("nextpnr BEL `{name}` has an invalid X coordinate"))?;
    let row = y
        .parse::<u32>()
        .map_err(|_| format!("nextpnr BEL `{name}` has an invalid Y coordinate"))?;
    Ok(format!("R{row}C{column}/{bel}"))
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

    use serde_json::{Value, json};
    use sha2::{Digest, Sha256};
    use struo_example_axi4_smartconnect::axi4_crossbar_self_test;
    use struo_synth::synthesize;
    use struo_target_ecp5::map_to_ecp5;

    use super::{ecp5_cell_name, ecp5_demo, lossless_nextpnr_json, read_nextpnr_placement};

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
        assert_eq!(
            format!("{:x}", Sha256::digest(&first_bytes)),
            "675de668f6eb6a414386f19e10777b7a6d9588467b4647cafa03b65eda73e5af"
        );
        assert_eq!(checkpoint["schema_version"], 3);
        assert_eq!(checkpoint["target"]["package"], "CABGA381");
        assert_eq!(checkpoint["target"]["speed_grade"], "6");
        assert_eq!(checkpoint["target"]["placement_weight_exponent"], 4);
        assert_eq!(
            checkpoint["target"]["placement_model"]["initial_algorithm"],
            "ecp5_timing_routability_electrostatic_v1"
        );
        assert_eq!(
            checkpoint["target"]["placement_model"]["timing_weight_model"],
            "ecp5_1_plus_10_criticality_power_v1"
        );
        assert_eq!(
            checkpoint["target"]["placement_model"]["routability_model"],
            "directional_rudy_area_adjustment_v1"
        );
        assert_eq!(
            checkpoint["target"]["placement_model"]["initial_predicted_detail"],
            false
        );
        assert_eq!(checkpoint["metrics"]["cells"], 4);
        assert_eq!(checkpoint["metrics"]["routed_nets"], 3);
        assert_eq!(checkpoint["placement"].as_array().unwrap().len(), 4);
        assert!(
            checkpoint["placement"]
                .as_array()
                .unwrap()
                .iter()
                .all(|cell| cell["kind"].is_string()
                    && cell["bel_type"].is_string()
                    && cell["bel_z"].is_number()
                    && cell["bel_pins"].is_array()
                    && cell["configuration_tiles"].is_array())
        );
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
        assert_eq!(checkpoint["timing"]["modeled_endpoint_count"], 0);
        assert_eq!(checkpoint["timing"]["all_modeled_endpoints_checked"], true);
        assert!(
            checkpoint["timing"]["unchecked_endpoints"]
                .as_array()
                .unwrap()
                .is_empty()
        );
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

    #[test]
    fn nextpnr_import_preserves_dedicated_carry_lut_ff_pairs() {
        let path = std::env::temp_dir().join(format!(
            "texo-nextpnr-carry-pairs-{}.json",
            std::process::id()
        ));
        let document = json!({
            "modules": {
                "top": {
                    "cells": {
                        "sum$CCU2_COMB0": {
                            "type": "TRELLIS_COMB",
                            "attributes": {"NEXTPNR_BEL": "X3/Y4/SLICEA.K0"},
                            "parameters": {}
                        },
                        "sum$CCU2_COMB1": {
                            "type": "TRELLIS_COMB",
                            "attributes": {"NEXTPNR_BEL": "X3/Y4/SLICEA.K1"},
                            "parameters": {}
                        },
                        "sum_low": {
                            "type": "TRELLIS_FF",
                            "attributes": {"NEXTPNR_BEL": "X3/Y4/SLICEA.FF0"},
                            "parameters": {"SD": "1"}
                        },
                        "sum_high": {
                            "type": "TRELLIS_FF",
                            "attributes": {"NEXTPNR_BEL": "X3/Y4/SLICEA.FF1"},
                            "parameters": {"SD": "1"}
                        }
                    }
                }
            }
        });
        fs::write(&path, serde_json::to_vec(&document).unwrap()).unwrap();

        let placement = read_nextpnr_placement(path.to_str().unwrap()).unwrap();

        assert_eq!(
            placement.lut_ff_pairs,
            [
                ("sum$slice0".into(), "sum_low".into()),
                ("sum$slice1".into(), "sum_high".into()),
            ]
            .into_iter()
            .collect()
        );
        assert_eq!(placement.bindings["sum$slice0"], "R4C3/SLICEA.K0");
        assert_eq!(placement.bindings["sum$slice1"], "R4C3/SLICEA.K1");
        assert_eq!(placement.bindings["sum_low"], "R4C3/SLICEA.FF0");
        assert_eq!(placement.bindings["sum_high"], "R4C3/SLICEA.FF1");

        fs::remove_file(path).unwrap();
    }
}
