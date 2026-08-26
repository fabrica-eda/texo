//! Reusable output support shared by the Texo CLI and design examples.

mod bitgen;
mod bitstream;
mod checkpoint;
mod target_pack;
mod veryl_project;
mod visualizer;

pub use bitgen::{
    Ecp5BitgenError, Ecp5BitgenOptions, Ecp5BitgenOutput, Ecp5BitgenPaths, Ecp5BitgenRuntime,
    bitgen,
};
pub use bitstream::{BitgenError, NativeEcp5Config, generate_ecp5_config};
pub use checkpoint::ecp5_checkpoint;
pub use target_pack::{
    Ecp5TargetPack, TargetPackError, install_ecp5_target_pack, resolve_ecp5_target,
    target_cache_root,
};
pub use veryl_project::{VerylProject, load_veryl_project};
pub use visualizer::write_checkpoint_visualizer;
