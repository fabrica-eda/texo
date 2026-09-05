//! End-to-end native ECP5 bitstream generation.

use std::env;
use std::error::Error;
use std::fmt;
use std::fs::{self, File};
use std::io::BufReader;
use std::path::{Path, PathBuf};
use std::process::Command;

use serde_json::Value;
use texo_target_ecp5::{read_architecture, read_architecture_cache};

use crate::{Ecp5TargetPack, generate_ecp5_config, resolve_ecp5_target};

/// Explicit runtime files used by ECP5 bitgen.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Ecp5BitgenPaths {
    /// Expanded ECP5 architecture JSON or `.txdb` cache.
    pub architecture: PathBuf,
    /// Project Trellis database root.
    pub database: PathBuf,
    /// Decompressed empty-device Project Trellis configuration.
    pub base_config: PathBuf,
    /// `ecppack` executable.
    pub ecppack: PathBuf,
}

/// Source of the architecture and Project Trellis runtime used by bitgen.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub enum Ecp5BitgenRuntime {
    /// Resolve and, when needed, download the target pack for the checkpoint device.
    #[default]
    Auto,
    /// Open an already installed target-pack directory.
    TargetPack(PathBuf),
    /// Use explicitly supplied architecture and Project Trellis paths.
    Explicit(Ecp5BitgenPaths),
}

/// Inputs for end-to-end native ECP5 bitstream generation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Ecp5BitgenOptions {
    /// Timing-closed checkpoint schema v3.
    pub checkpoint: PathBuf,
    /// ECP5 bitstream destination.
    pub bitstream: PathBuf,
    /// Text configuration destination, or `None` to append `.config` to `bitstream`.
    pub configuration: Option<PathBuf>,
    /// Runtime files used to encode the bitstream.
    pub runtime: Ecp5BitgenRuntime,
}

impl Ecp5BitgenOptions {
    /// Creates options that automatically resolve the checkpoint device's target pack.
    pub fn new(checkpoint: impl Into<PathBuf>, bitstream: impl Into<PathBuf>) -> Self {
        Self {
            checkpoint: checkpoint.into(),
            bitstream: bitstream.into(),
            configuration: None,
            runtime: Ecp5BitgenRuntime::Auto,
        }
    }
}

/// Files and route-accounting metrics produced by [`bitgen`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Ecp5BitgenOutput {
    /// Project Trellis text configuration written by bitgen.
    pub configuration: PathBuf,
    /// Encoded ECP5 bitstream written by `ecppack`.
    pub bitstream: PathBuf,
    /// Number of programmable PIPs emitted as configuration arcs.
    pub programmable_pips: usize,
    /// Number of fixed routing edges intentionally omitted from configuration.
    pub fixed_edges: usize,
}

/// Failure from end-to-end ECP5 bitstream generation.
#[derive(Debug)]
pub struct Ecp5BitgenError {
    source: Box<dyn Error>,
}

impl fmt::Display for Ecp5BitgenError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.source.fmt(formatter)
    }
}

impl Error for Ecp5BitgenError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        Some(self.source.as_ref())
    }
}

/// Generates an ECP5 bitstream from an implemented, timing-closed checkpoint.
///
/// This is the library entry point used by the `texo bitgen` command. It loads
/// or resolves the selected target runtime, writes the intermediate Project
/// Trellis configuration, and invokes `ecppack` to encode the final bitstream.
///
/// # Errors
///
/// Returns an error when an input cannot be read or validated, paths overlap,
/// target-pack resolution fails, configuration generation fails, or `ecppack`
/// cannot be run successfully.
pub fn bitgen(options: &Ecp5BitgenOptions) -> Result<Ecp5BitgenOutput, Ecp5BitgenError> {
    bitgen_inner(options).map_err(|source| Ecp5BitgenError { source })
}

#[allow(clippy::too_many_lines)]
fn bitgen_inner(options: &Ecp5BitgenOptions) -> Result<Ecp5BitgenOutput, Box<dyn Error>> {
    let checkpoint: Value =
        serde_json::from_reader(BufReader::new(File::open(&options.checkpoint)?))?;
    // Reject missing evidence/coverage before loading or fetching the runtime
    // and before writing a configuration or invoking the bitstream codec.
    crate::bitstream::validate_checkpoint(&checkpoint)?;
    let device = checkpoint
        .get("target")
        .and_then(|target| target.get("device"))
        .and_then(Value::as_str)
        .ok_or("checkpoint target device is absent")?;

    let pack = match &options.runtime {
        Ecp5BitgenRuntime::Auto => Some(resolve_ecp5_target(device)?),
        Ecp5BitgenRuntime::TargetPack(root) => Some(Ecp5TargetPack::open(root.clone())?),
        Ecp5BitgenRuntime::Explicit(_) => None,
    };
    if let Some(pack) = &pack
        && pack.device()? != device
    {
        return Err(format!(
            "target pack device {} does not match checkpoint {device}",
            pack.device()?
        )
        .into());
    }

    let explicit = match &options.runtime {
        Ecp5BitgenRuntime::Explicit(paths) => Some(paths),
        Ecp5BitgenRuntime::Auto | Ecp5BitgenRuntime::TargetPack(_) => None,
    };
    let architecture_path = explicit
        .map(|paths| &paths.architecture)
        .or_else(|| pack.as_ref().map(|pack| &pack.architecture))
        .expect("an explicit runtime or target pack was resolved");
    let database = explicit
        .map(|paths| &paths.database)
        .or_else(|| pack.as_ref().map(|pack| &pack.database))
        .expect("an explicit runtime or target pack was resolved");
    let base_config_path = explicit
        .map(|paths| &paths.base_config)
        .or_else(|| pack.as_ref().map(|pack| &pack.base_config))
        .expect("an explicit runtime or target pack was resolved");
    let ecppack = explicit
        .map(|paths| &paths.ecppack)
        .or_else(|| pack.as_ref().map(|pack| &pack.ecppack))
        .expect("an explicit runtime or target pack was resolved");

    let architecture = load_architecture(architecture_path)?;
    let base_config = fs::read_to_string(base_config_path)?;
    let iodb_path = pack.as_ref().map_or_else(
        || database.join("ECP5").join(device).join("iodb.json"),
        |pack| pack.iodb.clone(),
    );
    let iodb: Value = serde_json::from_reader(BufReader::new(File::open(iodb_path)?))?;
    let generated = generate_ecp5_config(&checkpoint, &architecture, &base_config, &iodb)?;
    let expected_pips = checkpoint
        .get("metrics")
        .and_then(|metrics| metrics.get("total_pips"))
        .and_then(Value::as_u64)
        .ok_or("checkpoint total PIP count is absent")?;
    if u64::try_from(generated.programmable_pips + generated.fixed_edges)? != expected_pips {
        return Err(format!(
            "configuration accounted for {} of {expected_pips} PIPs",
            generated.programmable_pips + generated.fixed_edges
        )
        .into());
    }

    let configuration = options
        .configuration
        .clone()
        .unwrap_or_else(|| bitstream_config_path(&options.bitstream));
    for (input, output, input_label, output_label) in [
        (
            options.checkpoint.as_path(),
            configuration.as_path(),
            "checkpoint",
            "configuration",
        ),
        (
            options.checkpoint.as_path(),
            options.bitstream.as_path(),
            "checkpoint",
            "bitstream",
        ),
        (
            architecture_path.as_path(),
            configuration.as_path(),
            "architecture",
            "configuration",
        ),
        (
            architecture_path.as_path(),
            options.bitstream.as_path(),
            "architecture",
            "bitstream",
        ),
    ] {
        ensure_distinct_paths(input, output, input_label, output_label)?;
    }
    if configuration == options.bitstream {
        return Err("configuration and bitstream outputs must differ".into());
    }
    for output in [&configuration, &options.bitstream] {
        if let Some(parent) = output
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
        {
            fs::create_dir_all(parent)?;
        }
    }
    fs::write(&configuration, generated.text)?;

    let mut command = Command::new(ecppack);
    if let Some(pack) = &pack {
        configure_pack_libraries(&mut command, pack)?;
    }
    let status = command
        .arg(&configuration)
        .arg(&options.bitstream)
        .arg("--db")
        .arg(database)
        .status()?;
    if !status.success() {
        return Err(format!("ecppack exited with {status}").into());
    }

    Ok(Ecp5BitgenOutput {
        configuration,
        bitstream: options.bitstream.clone(),
        programmable_pips: generated.programmable_pips,
        fixed_edges: generated.fixed_edges,
    })
}

fn load_architecture(path: &Path) -> Result<texo_target_ecp5::Ecp5Architecture, Box<dyn Error>> {
    let reader = BufReader::new(File::open(path)?);
    if path.extension().and_then(|extension| extension.to_str()) == Some("txdb") {
        Ok(read_architecture_cache(reader)?)
    } else {
        Ok(read_architecture(reader)?)
    }
}

fn bitstream_config_path(bitstream: &Path) -> PathBuf {
    let mut path = bitstream.as_os_str().to_owned();
    path.push(".config");
    path.into()
}

fn ensure_distinct_paths(
    input: &Path,
    output: &Path,
    input_label: &str,
    output_label: &str,
) -> Result<(), Box<dyn Error>> {
    let same_path = input == output
        || fs::canonicalize(input)
            .ok()
            .zip(fs::canonicalize(output).ok())
            .is_some_and(|(input, output)| input == output);
    if same_path {
        return Err(format!(
            "{output_label} must not overwrite {input_label}: {}",
            input.display()
        )
        .into());
    }
    Ok(())
}

fn configure_pack_libraries(
    command: &mut Command,
    pack: &Ecp5TargetPack,
) -> Result<(), Box<dyn Error>> {
    let mut paths = vec![pack.root.join("lib")];
    if let Some(existing) = env::var_os("LD_LIBRARY_PATH") {
        paths.extend(env::split_paths(&existing));
    }
    command.env("LD_LIBRARY_PATH", env::join_paths(paths)?);
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::{Ecp5BitgenOptions, Ecp5BitgenRuntime};

    #[test]
    fn options_default_to_automatic_target_pack_resolution() {
        let options = Ecp5BitgenOptions::new("closed.json", "design.bit");

        assert_eq!(options.checkpoint, Path::new("closed.json"));
        assert_eq!(options.bitstream, Path::new("design.bit"));
        assert_eq!(options.configuration, None);
        assert_eq!(options.runtime, Ecp5BitgenRuntime::Auto);
    }
}
