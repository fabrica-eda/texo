//! Texo command-line entry point.

use std::env;
use std::error::Error;
use std::process::ExitCode;

use texo_flow::{Evidence, implement};
use texo_model::{Design, Device, PinDirection, ResourceKind};

const USAGE: &str = "\
Texo FPGA place and route

Usage:
  texo demo     run the deterministic abstract-grid PnR demo
  texo help     show this help
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
    match env::args().nth(1).as_deref() {
        Some("demo") => demo(),
        None | Some("help" | "-h" | "--help") => {
            print!("{USAGE}");
            Ok(())
        }
        Some(command) => Err(format!("unknown command `{command}`\n\n{USAGE}").into()),
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
