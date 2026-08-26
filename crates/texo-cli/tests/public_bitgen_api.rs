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
