//! Structural regression for the frozen Core250 ECP5 `QoR` fixture.

use std::fs::File;
use std::io::BufReader;
use std::path::PathBuf;

use struo_synth::synthesize;
use struo_target_ecp5::{
    Ecp5Cell, JtaggBinding, MappingOptions, PllBinding, map_to_ecp5_with_options,
};
use texo_cli::load_veryl_project;

#[test]
fn core250_qor_fixture_preserves_its_mapped_shape() {
    let fixture = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../examples/core250-qor");
    let loaded = load_veryl_project(&fixture, None).expect("load Core250 fixture");
    assert_eq!(loaded.top, "Core250JtagTop");
    assert_eq!(loaded.project_sources, 6);

    let synthesized = synthesize(&loaded.design).expect("synthesize Core250 fixture");
    assert_eq!(synthesized.netlist.nodes().len(), 6_237);
    assert_eq!(synthesized.netlist.registers().len(), 1_167);
    assert_eq!(synthesized.netlist.memories().len(), 4);

    let mut mapped = map_to_ecp5_with_options(
        &synthesized.netlist,
        MappingOptions {
            timing_goal_mhz: 250,
            ..MappingOptions::default()
        },
    )
    .expect("map Core250 fixture");
    mapped
        .bind_jtagg(&JtaggBinding::with_prefix("jtag"))
        .expect("bind JTAGG");
    let pll: PllBinding = serde_json::from_reader(BufReader::new(
        File::open(fixture.join("pll-12-to-250.json")).expect("open PLL binding"),
    ))
    .expect("parse PLL binding");
    mapped.bind_pll(&pll).expect("bind PLL");

    assert!(mapped.retiming().equivalence_signed_off);
    assert_eq!(mapped.cells().len(), 3_508);
    assert_eq!(
        count_cells(mapped.cells(), |cell| matches!(
            cell,
            Ecp5Cell::BlockRam { .. }
        )),
        8
    );
    assert_eq!(
        count_cells(mapped.cells(), |cell| matches!(
            cell,
            Ecp5Cell::Jtagg { .. }
        )),
        1
    );
    assert_eq!(
        count_cells(mapped.cells(), |cell| matches!(cell, Ecp5Cell::Pll { .. })),
        1
    );
}

fn count_cells(cells: &[Ecp5Cell], predicate: impl Fn(&Ecp5Cell) -> bool) -> usize {
    cells.iter().filter(|cell| predicate(cell)).count()
}
