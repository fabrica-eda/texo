//! Texo command-line entry point.

use std::env;
use std::error::Error;
use std::process::ExitCode;

use texo_flow::{Evidence, implement};
use texo_model::{Design, Device, ResourceKind};

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
    let lut = design.add_cell("lut4", ResourceKind::Logic);
    let register = design.add_cell("trellis_ff", ResourceKind::Logic);
    let output = design.add_cell("output_buffer", ResourceKind::Logic);
    design.add_net("input_to_lut", [input, lut])?;
    design.add_net("lut_to_ff", [lut, register])?;
    design.add_net("ff_to_output", [register, output])?;

    let device = Device::rectangular_logic(8, 8)?;
    let mut evidence = Evidence::new();
    let result = implement(&design, &device, &mut evidence)?;

    println!("placed {} cells", result.placement.locations().len());
    for (id, point) in result.placement.locations().iter().enumerate() {
        println!(
            "  {:<16} -> ({}, {})",
            design.cells()[id].name,
            point.x,
            point.y
        );
    }
    println!(
        "routed {} nets, abstract wire length {}",
        result.routes.len(),
        result.total_wire_length
    );
    println!("physical implementation gate: passed");
    println!("bitstream release: blocked until simulation and timing gates pass");
    Ok(())
}
