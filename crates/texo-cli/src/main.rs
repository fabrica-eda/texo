//! Texo command-line entry point.

use std::env;
use std::error::Error;
use std::fs::File;
use std::io::{BufReader, BufWriter};
use std::path::Path;
use std::process::ExitCode;
use std::time::Instant;

use serde_json::{Value, json};
use struo_celox::ecp5_simulator;
use struo_example_axi4_smartconnect::axi4_crossbar_self_test;
use struo_ir::Netlist;
use struo_synth::synthesize;
use struo_target_ecp5::map_to_ecp5;
use texo_flow::{
    Ecp5FlowOptions, Ecp5FlowResult, Evidence, Gate, implement, implement_struo_ecp5,
    verify_post_map_with_celox,
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
  texo axi4-pnr <architecture> <package> <speed-grade> <constraints.lpf> [checkpoint.json]
                                    run the Struo AXI4 self-test through native Texo PnR
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
            if args.next().is_some() {
                return Err(format!("axi4-pnr accepts at most five arguments\n\n{USAGE}").into());
            }
            axi4_pnr(
                &architecture,
                &package,
                &speed_grade,
                &lpf,
                checkpoint.as_deref(),
            )
        }
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
    let result = implement_struo_ecp5(
        &imported,
        &architecture,
        Ecp5FlowOptions {
            speed_grade: Some(speed_grade),
            package: Some(package),
            lpf: Some(&lpf),
            ..Ecp5FlowOptions::default()
        },
        &mut evidence,
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
) -> Result<(), Box<dyn Error>> {
    let rtl = axi4_crossbar_self_test()?;
    let synthesized = synthesize(&rtl)?;
    let mapped = map_to_ecp5(&synthesized.netlist)?;
    let imported = import_ecp5(&mapped)?;
    println!(
        "Struo AXI4 self-test: {} Boolean nodes, {} registers, {} mapped cells",
        synthesized.netlist.nodes().len(),
        synthesized.netlist.registers().len(),
        mapped.cells().len()
    );

    let mut evidence = Evidence::new();
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
    let result = implement_struo_ecp5(
        &imported,
        &architecture,
        Ecp5FlowOptions {
            speed_grade: Some(speed_grade),
            package: Some(package),
            lpf: Some(&lpf),
            ..Ecp5FlowOptions::default()
        },
        &mut evidence,
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
            let wires = route
                .wires
                .iter()
                .map(|wire| json!({ "wire_id": wire.0, "wire": device.wires()[wire.0].name }))
                .collect::<Vec<_>>();
            let pips = route
                .pips
                .iter()
                .map(|pip_id| {
                    let pip = &device.pips()[pip_id.0];
                    json!({
                        "pip_id": pip_id.0,
                        "from_wire_id": pip.from.0,
                        "from": device.wires()[pip.from.0].name,
                        "to_wire_id": pip.to.0,
                        "to": device.wires()[pip.to.0].name,
                    })
                })
                .collect::<Vec<_>>();
            json!({
                "net_id": route.net.0,
                "net": result.design.nets()[route.net.0].name,
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
            json!({
                "net_id": delay.net.0,
                "net": result.design.nets()[delay.net.0].name,
                "sink_pin_id": delay.sink.0,
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
    use std::fs;

    use serde_json::Value;

    use super::ecp5_demo;

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
}
