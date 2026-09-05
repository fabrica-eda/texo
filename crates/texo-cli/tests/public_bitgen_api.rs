//! Compile-time coverage for the public end-to-end bitgen API.

use std::path::Path;

use texo_cli::{Ecp5BitgenError, Ecp5BitgenOptions, Ecp5BitgenOutput, Ecp5BitgenRuntime, bitgen};

#[test]
fn end_to_end_bitgen_is_part_of_the_library_api() {
    let options = Ecp5BitgenOptions::new("closed.checkpoint.json", "design.bit");
    let entry_point: fn(&Ecp5BitgenOptions) -> Result<Ecp5BitgenOutput, Ecp5BitgenError> = bitgen;

    assert_eq!(options.checkpoint, Path::new("closed.checkpoint.json"));
    assert_eq!(options.bitstream, Path::new("design.bit"));
    assert_eq!(options.runtime, Ecp5BitgenRuntime::Auto);
    let _ = entry_point;
}

#[test]
fn uncovered_timing_is_rejected_before_runtime_lookup_or_output() {
    let temporary = std::env::temp_dir().join(format!(
        "texo-bitgen-unchecked-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos(),
    ));
    std::fs::create_dir(&temporary).unwrap();
    let options = Ecp5BitgenOptions {
        runtime: Ecp5BitgenRuntime::TargetPack(temporary.join("missing-target-pack")),
        ..Ecp5BitgenOptions::new(temporary.join("bad.json"), temporary.join("bad.bit"))
    };
    let checkpoint = serde_json::json!({
        "schema_version": 3,
        "evidence": [
            "synthesis_equivalence", "mapped_netlist_complete",
            "physical_implementation", "timing_closure"
        ],
        "target": {"family": "ECP5", "device": "LFE5UM5G-85F"},
        "timing": {
            "met_timing": true,
            "all_modeled_endpoints_checked": false,
            "unchecked_endpoints": [{
                "cell": "jtag_shift", "data_pin_id": 1, "reason": "unconstrained_clock"
            }]
        }
    });
    std::fs::write(
        &options.checkpoint,
        serde_json::to_vec(&checkpoint).unwrap(),
    )
    .unwrap();

    let error = bitgen(&options).unwrap_err().to_string();
    assert!(error.contains("unconstrained_clock"), "{error}");
    assert!(!options.bitstream.exists());
    assert_eq!(std::fs::read_dir(&temporary).unwrap().count(), 1);
    std::fs::remove_dir_all(temporary).unwrap();
}
