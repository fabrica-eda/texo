//! Texo command-line entry point.

use std::env;
use std::error::Error;
use std::fs::File;
use std::process::ExitCode;

use texo_flow::{Evidence, implement};
use texo_model::{Design, Device, PinDirection, ResourceKind};
use texo_target_ecp5::{parse_lpf, read_architecture};

const USAGE: &str = "\
Texo FPGA place and route

Usage:
  texo demo                         run the deterministic abstract-grid PnR demo
  texo target-info <architecture>   inspect an ECP5 architecture snapshot
  texo lpf-info <constraints.lpf>   inspect ECP5 pin and IOBUF constraints
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
        Some("target-info") => {
            let path = args
                .next()
                .ok_or_else(|| format!("target-info requires an architecture path\n\n{USAGE}"))?;
            if args.next().is_some() {
                return Err(format!("target-info accepts one architecture path\n\n{USAGE}").into());
            }
            target_info(&path)
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
    let architecture = read_architecture(File::open(path)?)?;
    let device = architecture.device();
    let fixed_pips = architecture
        .pip_metadata()
        .values()
        .filter(|pip| pip.fixed)
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
        "Project Trellis revision: {}",
        architecture.provenance().project_trellis_revision
    );
    println!(
        "database revision: {}",
        architecture.provenance().database_revision
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
