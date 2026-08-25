//! Texo command-line entry point.

use std::env;
use std::error::Error;
use std::fs::File;
use std::io::{BufReader, BufWriter, Write};
use std::num::NonZeroU32;
use std::path::{Path, PathBuf};
use std::process::{Command as ProcessCommand, ExitCode};
use std::time::Instant;

use clap::{Args, Parser, Subcommand};
use struo_synth::synthesize;
use struo_target_ecp5::{ECP5_QOR_TARGET_MHZ, MappingOptions, map_to_ecp5_with_options};
use texo_cli::{
    Ecp5TargetPack, VerylProject, ecp5_checkpoint, generate_ecp5_config, install_ecp5_target_pack,
    load_veryl_project, resolve_ecp5_target, write_checkpoint_visualizer,
};
use texo_flow::{
    Ecp5FlowOptions, Ecp5FlowResult, Ecp5FlowStage, Evidence, Gate, PostMapSimulationPolicy,
    RoutingProgress, implement_struo_ecp5_with_progress,
};
use texo_struo::import_ecp5;
use texo_target_ecp5::{
    Ecp5Architecture, parse_lpf, read_architecture, read_architecture_cache,
    write_architecture_cache,
};

/// Native ECP5 place and route.
#[derive(Debug, Parser)]
#[command(name = "texo", version, about, propagate_version = true)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Synthesize, place, route, and time a Veryl project.
    Pnr(PnrArgs),
    /// Cache an expanded JSON architecture for fast subsequent loads.
    CacheArchitecture {
        /// Project Trellis architecture JSON.
        source: PathBuf,
        /// Destination architecture cache (`.txdb`).
        destination: PathBuf,
    },
    /// Inspect an ECP5 architecture JSON or cache.
    TargetInfo {
        /// Architecture JSON or `.txdb` cache.
        architecture: PathBuf,
    },
    /// Inspect pin, IO, and clock constraints in an LPF.
    LpfInfo {
        /// LPF constraint file.
        constraints: PathBuf,
    },
    /// Generate an ECP5 bitstream from an implemented, timing-closed checkpoint.
    Bitgen(BitgenArgs),
    /// Install or fetch architecture and bitstream-codec target packs.
    Target(TargetArgs),
    /// Render a Texo checkpoint as self-contained interactive HTML/SVG.
    Visualize {
        /// P&R checkpoint written by `texo pnr`.
        checkpoint: PathBuf,
        /// HTML destination; defaults to `<checkpoint>.html`.
        #[arg(short, long)]
        output: Option<PathBuf>,
    },
}

#[derive(Debug, Args)]
struct BitgenArgs {
    /// Timing-closed checkpoint schema v3.
    checkpoint: PathBuf,
    /// ECP5 architecture `.txdb` used by the checkpoint.
    #[arg(short, long)]
    architecture: Option<PathBuf>,
    /// Target-pack Project Trellis database root.
    #[arg(long)]
    database: Option<PathBuf>,
    /// Decompressed empty-device Project Trellis configuration.
    #[arg(long)]
    base_config: Option<PathBuf>,
    /// Already installed target-pack directory.
    #[arg(long)]
    target_pack: Option<PathBuf>,
    /// Target-pack `ecppack` executable.
    #[arg(long)]
    ecppack: Option<PathBuf>,
    /// Text configuration destination; defaults beside the bitstream.
    #[arg(long)]
    config: Option<PathBuf>,
    /// ECP5 bitstream destination.
    #[arg(short, long)]
    bit: PathBuf,
}

#[derive(Debug, Args)]
struct TargetArgs {
    #[command(subcommand)]
    command: TargetCommand,
}

#[derive(Debug, Subcommand)]
enum TargetCommand {
    /// Install and verify a downloaded `.txpkg.zst`.
    Install {
        /// Target-pack archive.
        archive: PathBuf,
        /// Override the target cache root.
        #[arg(long)]
        cache_dir: Option<PathBuf>,
    },
    /// Download a pinned target pack when it is not cached.
    Fetch {
        /// Exact FPGA device name.
        #[arg(default_value = "LFE5UM5G-85F")]
        device: String,
    },
}

#[derive(Debug, Args)]
struct PnrArgs {
    /// Project directory or `Veryl.toml`.
    input: PathBuf,
    /// Top module; overrides `[synth].top`.
    #[arg(short, long)]
    top: Option<String>,
    /// ECP5 architecture JSON or `.txdb` cache.
    #[arg(short, long)]
    architecture: Option<PathBuf>,
    /// Exact FPGA device used to resolve an installed target pack.
    #[arg(long, default_value = "LFE5UM5G-85F")]
    device: String,
    /// Exact package name from the architecture database.
    #[arg(short, long)]
    package: String,
    /// Exact device speed grade (for example `6` or `8_5G`).
    #[arg(short, long)]
    speed: String,
    /// LPF pin, IO, and clock constraints.
    #[arg(short, long)]
    lpf: Option<PathBuf>,
    /// Checkpoint destination; defaults to `target/texo/<top>.json` for projects.
    #[arg(short, long)]
    output: Option<PathBuf>,
    /// Mapping/retiming optimization target in MHz.
    #[arg(long, default_value_t = default_synthesis_goal())]
    synthesis_goal_mhz: NonZeroU32,
    /// Sharpen timing-driven placement criticality weights.
    #[arg(long, default_value = "1")]
    placement_weight_exponent: NonZeroU32,
    /// Permit top-level IO bits without an LPF location.
    #[arg(long)]
    allow_unconstrained_io: bool,
    /// Override automatic global-clock promotion fanout.
    #[arg(long)]
    global_clock_fanout: Option<usize>,
    /// Keep the initial legal placement and route without timing closure.
    #[arg(long)]
    no_timing_optimization: bool,
}

const fn default_synthesis_goal() -> NonZeroU32 {
    NonZeroU32::new(ECP5_QOR_TARGET_MHZ).expect("the ECP5 QoR target is nonzero")
}

fn main() -> ExitCode {
    match run(Cli::parse()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("error: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run(cli: Cli) -> Result<(), Box<dyn Error>> {
    match cli.command {
        Command::Pnr(args) => pnr(&args),
        Command::CacheArchitecture {
            source,
            destination,
        } => cache_architecture(&source, &destination),
        Command::TargetInfo { architecture } => target_info(&architecture),
        Command::LpfInfo { constraints } => lpf_info(&constraints),
        Command::Bitgen(args) => bitgen(&args),
        Command::Target(args) => target(&args),
        Command::Visualize { checkpoint, output } => {
            let output = output.unwrap_or_else(|| checkpoint_html_path(&checkpoint));
            ensure_distinct_paths(&checkpoint, &output, "checkpoint", "HTML output")?;
            write_checkpoint_visualizer(path_text(&checkpoint)?, path_text(&output)?)?;
            println!("visualizer: {}", output.display());
            Ok(())
        }
    }
}

#[allow(clippy::too_many_lines)]
fn bitgen(args: &BitgenArgs) -> Result<(), Box<dyn Error>> {
    let checkpoint: serde_json::Value =
        serde_json::from_reader(BufReader::new(File::open(&args.checkpoint)?))?;
    let device = checkpoint
        .get("target")
        .and_then(|target| target.get("device"))
        .and_then(serde_json::Value::as_str)
        .ok_or("checkpoint target device is absent")?;
    let explicit = args.architecture.is_some()
        || args.database.is_some()
        || args.base_config.is_some()
        || args.ecppack.is_some();
    if explicit
        && (args.architecture.is_none()
            || args.database.is_none()
            || args.base_config.is_none()
            || args.ecppack.is_none())
    {
        return Err(
            "--architecture, --database, --base-config, and --ecppack must be supplied together"
                .into(),
        );
    }
    if args.target_pack.is_some() && explicit {
        return Err("--target-pack cannot be combined with individual runtime paths".into());
    }
    let pack = if let Some(root) = &args.target_pack {
        Some(Ecp5TargetPack::open(root.clone())?)
    } else if explicit {
        None
    } else {
        Some(resolve_ecp5_target(device)?)
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
    let architecture_path = args
        .architecture
        .as_ref()
        .or_else(|| pack.as_ref().map(|pack| &pack.architecture))
        .expect("explicit paths or a pack were resolved");
    let database = args
        .database
        .as_ref()
        .or_else(|| pack.as_ref().map(|pack| &pack.database))
        .expect("explicit paths or a pack were resolved");
    let base_config_path = args
        .base_config
        .as_ref()
        .or_else(|| pack.as_ref().map(|pack| &pack.base_config))
        .expect("explicit paths or a pack were resolved");
    let ecppack = args
        .ecppack
        .as_ref()
        .or_else(|| pack.as_ref().map(|pack| &pack.ecppack))
        .expect("explicit paths or a pack were resolved");
    let architecture = load_architecture(architecture_path)?;
    let base_config = std::fs::read_to_string(base_config_path)?;
    let iodb_path = pack.as_ref().map_or_else(
        || database.join("ECP5").join(device).join("iodb.json"),
        |pack| pack.iodb.clone(),
    );
    let iodb: serde_json::Value = serde_json::from_reader(BufReader::new(File::open(&iodb_path)?))?;
    let generated = generate_ecp5_config(&checkpoint, &architecture, &base_config, &iodb)?;
    let expected_pips = checkpoint
        .get("metrics")
        .and_then(|metrics| metrics.get("total_pips"))
        .and_then(serde_json::Value::as_u64)
        .ok_or("checkpoint total PIP count is absent")?;
    if u64::try_from(generated.programmable_pips + generated.fixed_edges)? != expected_pips {
        return Err(format!(
            "configuration accounted for {} of {expected_pips} PIPs",
            generated.programmable_pips + generated.fixed_edges
        )
        .into());
    }
    let config = args
        .config
        .clone()
        .unwrap_or_else(|| bitstream_config_path(&args.bit));
    for (input, output, input_label, output_label) in [
        (&args.checkpoint, &config, "checkpoint", "configuration"),
        (&args.checkpoint, &args.bit, "checkpoint", "bitstream"),
        (architecture_path, &config, "architecture", "configuration"),
        (architecture_path, &args.bit, "architecture", "bitstream"),
    ] {
        ensure_distinct_paths(input, output, input_label, output_label)?;
    }
    if config == args.bit {
        return Err("configuration and bitstream outputs must differ".into());
    }
    for output in [&config, &args.bit] {
        if let Some(parent) = output
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
        {
            std::fs::create_dir_all(parent)?;
        }
    }
    std::fs::write(&config, generated.text)?;
    let mut command = ProcessCommand::new(ecppack);
    if let Some(pack) = &pack {
        configure_pack_libraries(&mut command, pack)?;
    }
    let status = command
        .arg(&config)
        .arg(&args.bit)
        .arg("--db")
        .arg(database)
        .status()?;
    if !status.success() {
        return Err(format!("ecppack exited with {status}").into());
    }
    println!(
        "Texo native bitgen: {} programmable PIPs, {} fixed edges",
        generated.programmable_pips, generated.fixed_edges
    );
    println!("configuration: {}", config.display());
    println!("bitstream: {}", args.bit.display());
    Ok(())
}

fn target(args: &TargetArgs) -> Result<(), Box<dyn Error>> {
    let pack = match &args.command {
        TargetCommand::Install { archive, cache_dir } => {
            install_ecp5_target_pack(archive, cache_dir.as_deref())?
        }
        TargetCommand::Fetch { device } => resolve_ecp5_target(device)?,
    };
    println!("target: {}", pack.device()?);
    println!("installed: {}", pack.root.display());
    Ok(())
}

fn configure_pack_libraries(
    command: &mut ProcessCommand,
    pack: &Ecp5TargetPack,
) -> Result<(), Box<dyn Error>> {
    let mut paths = vec![pack.root.join("lib")];
    if let Some(existing) = env::var_os("LD_LIBRARY_PATH") {
        paths.extend(env::split_paths(&existing));
    }
    command.env("LD_LIBRARY_PATH", env::join_paths(paths)?);
    Ok(())
}

fn pnr(args: &PnrArgs) -> Result<(), Box<dyn Error>> {
    let flow_started = Instant::now();
    let (loaded, output) = prepare_veryl_input(args)?;

    let synthesized = synthesize(&loaded.design)?;
    for report in &synthesized.reports {
        println!("synthesis {}: {}", report.pass, report.message);
    }
    let mapped = map_to_ecp5_with_options(
        &synthesized.netlist,
        MappingOptions {
            timing_goal_mhz: args.synthesis_goal_mhz.get(),
            ..MappingOptions::default()
        },
    )?;
    if !mapped.retiming().equivalence_signed_off {
        return Err("Struo mapping/retiming equivalence sign-off failed".into());
    }
    let mut evidence = Evidence::new();
    evidence.record(Gate::SynthesisEquivalence);
    println!(
        "mapped: {} Boolean nodes, {} registers, {} ECP5 cells ({} MHz goal)",
        synthesized.netlist.nodes().len(),
        synthesized.netlist.registers().len(),
        mapped.cells().len(),
        args.synthesis_goal_mhz,
    );

    let imported = import_ecp5(&mapped)?;
    let architecture_started = Instant::now();
    let pack = args
        .architecture
        .is_none()
        .then(|| resolve_ecp5_target(&args.device))
        .transpose()?;
    let architecture_path = args
        .architecture
        .as_deref()
        .or_else(|| pack.as_ref().map(|pack| pack.architecture.as_path()))
        .expect("an explicit architecture or target pack was resolved");
    let architecture = load_architecture(architecture_path)?;
    println!(
        "architecture loaded in {:.2?}",
        architecture_started.elapsed()
    );
    let lpf = match &args.lpf {
        Some(path) => Some(parse_lpf(File::open(path)?)?),
        None => None,
    };

    let mut options = Ecp5FlowOptions {
        post_map_simulation: PostMapSimulationPolicy::AllowMissing,
        speed_grade: Some(&args.speed),
        package: Some(&args.package),
        lpf: lpf.as_ref(),
        allow_unconstrained_io: args.allow_unconstrained_io,
        placement_weight_exponent: args.placement_weight_exponent.get(),
        optimize_timing: !args.no_timing_optimization,
        ..Ecp5FlowOptions::default()
    };
    if let Some(fanout) = args.global_clock_fanout {
        options.global_clock_fanout = fanout;
    }
    let mut phase_started = Instant::now();
    let result = implement_struo_ecp5_with_progress(
        &imported,
        &architecture,
        options,
        &mut evidence,
        |stage| report_flow_stage(stage, &mut phase_started),
    )?;

    write_checkpoint(
        &output,
        &loaded.top,
        &result,
        &architecture,
        &args.package,
        &evidence,
    )?;

    println!(
        "implemented: {} cells, {} nets, {} PIPs in {:.2?}",
        result.design.cells().len(),
        result.implementation.routes.len(),
        result.implementation.total_pips,
        flow_started.elapsed(),
    );
    println!(
        "timing: WNS {}, WHS {}, {}",
        format_slack(result.timing.worst_slack_ps),
        format_slack(result.timing.worst_hold_slack_ps),
        if result.timing.met_timing() {
            "met"
        } else {
            "not met"
        },
    );
    println!("checkpoint: {}", output.display());
    println!(
        "verification: mapping equivalence passed; post-map functional simulation not supplied"
    );
    Ok(())
}

fn prepare_veryl_input(args: &PnrArgs) -> Result<(VerylProject, PathBuf), Box<dyn Error>> {
    let loaded = load_veryl_project(&args.input, args.top.as_deref())?;
    let output = args
        .output
        .clone()
        .unwrap_or_else(|| default_checkpoint_path(&loaded));
    ensure_distinct_paths(&args.input, &output, "Veryl input", "checkpoint")?;
    ensure_distinct_paths(&loaded.manifest, &output, "Veryl.toml", "checkpoint")?;
    for source in &loaded.source_paths {
        ensure_distinct_paths(source, &output, "Veryl source", "checkpoint")?;
    }
    for warning in &loaded.warnings {
        eprintln!("warning: {warning}");
    }
    println!(
        "Veryl project: {} top {} ({} project sources, {} total units)",
        loaded.project, loaded.top, loaded.project_sources, loaded.total_sources
    );
    Ok((loaded, output))
}

fn write_checkpoint(
    output: &Path,
    design_name: &str,
    result: &Ecp5FlowResult,
    architecture: &Ecp5Architecture,
    package: &str,
    evidence: &Evidence,
) -> Result<(), Box<dyn Error>> {
    if let Some(parent) = output
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        std::fs::create_dir_all(parent)?;
    }
    let checkpoint = ecp5_checkpoint(design_name, result, architecture, package, evidence);
    let mut writer = BufWriter::new(File::create(output)?);
    serde_json::to_writer_pretty(&mut writer, &checkpoint)?;
    writer.write_all(b"\n")?;
    writer.flush()?;
    Ok(())
}

fn report_flow_stage(stage: Ecp5FlowStage, started: &mut Instant) {
    match stage {
        Ecp5FlowStage::CriticalPathMove { cell, from, to } => {
            println!(
                "critical-path placement trial: cell {}, BEL {} -> {}",
                cell.0, from.0, to.0
            );
        }
        Ecp5FlowStage::TimingTrialDecision { improves_objective } => println!(
            "timing trial: {}",
            if improves_objective {
                "accepted"
            } else {
                "rejected"
            }
        ),
        Ecp5FlowStage::TimingSnapshot {
            worst_setup_ps,
            setup_tns_ps,
            setup_violations,
            worst_hold_ps,
            hold_ths_ps,
            hold_violations,
        } => println!(
            "timing trial: WNS {}, TNS {setup_tns_ps} ps ({setup_violations}), WHS {}, THS {hold_ths_ps} ps ({hold_violations})",
            format_slack(worst_setup_ps),
            format_slack(worst_hold_ps),
        ),
        Ecp5FlowStage::Routing(event) | Ecp5FlowStage::TimingDrivenRouting(event) => {
            let label = if matches!(stage, Ecp5FlowStage::Routing(_)) {
                "routing"
            } else {
                "timing routing"
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
                } if env::var_os("TEXO_ROUTE_TRACE").is_some() => println!(
                    "{label} iteration {} net {ordinal}/{total}: {}",
                    iteration + 1,
                    net.0
                ),
                RoutingProgress::Net { .. } => {}
            }
        }
        stage => {
            let label = match stage {
                Ecp5FlowStage::Packed => "packing",
                Ecp5FlowStage::Placed => "placement",
                Ecp5FlowStage::GlobalClocksRouted => "global clock routing",
                Ecp5FlowStage::Routed => "negotiated routing",
                Ecp5FlowStage::TimingDrivenPlaced => "timing-driven placement",
                Ecp5FlowStage::TimingDrivenGlobalClocksRouted => {
                    "timing-driven global clock routing"
                }
                Ecp5FlowStage::TimingDrivenRouted => "timing-driven negotiated routing",
                Ecp5FlowStage::Timed => "timing analysis",
                _ => unreachable!("handled progress stage"),
            };
            println!("{label} completed in {:.2?}", started.elapsed());
            *started = Instant::now();
        }
    }
}

fn format_slack(slack: Option<i128>) -> String {
    slack.map_or_else(|| "n/a".into(), |value| format!("{value} ps"))
}

fn load_architecture(path: &Path) -> Result<Ecp5Architecture, Box<dyn Error>> {
    let reader = BufReader::new(File::open(path)?);
    if path.extension().and_then(|extension| extension.to_str()) == Some("txdb") {
        Ok(read_architecture_cache(reader)?)
    } else {
        Ok(read_architecture(reader)?)
    }
}

fn cache_architecture(source: &Path, destination: &Path) -> Result<(), Box<dyn Error>> {
    ensure_distinct_paths(source, destination, "architecture source", "cache output")?;
    let started = Instant::now();
    let architecture = read_architecture(BufReader::new(File::open(source)?))?;
    println!("architecture expanded in {:.2?}", started.elapsed());
    if let Some(parent) = destination
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        std::fs::create_dir_all(parent)?;
    }
    let started = Instant::now();
    write_architecture_cache(BufWriter::new(File::create(destination)?), &architecture)?;
    println!(
        "architecture cache written to {} in {:.2?}",
        destination.display(),
        started.elapsed()
    );
    Ok(())
}

fn target_info(path: &Path) -> Result<(), Box<dyn Error>> {
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
    println!(
        "packages: {}",
        architecture
            .packages()
            .iter()
            .map(|package| package.name.as_str())
            .collect::<Vec<_>>()
            .join(", ")
    );
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

fn lpf_info(path: &Path) -> Result<(), Box<dyn Error>> {
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

fn default_checkpoint_path(loaded: &VerylProject) -> PathBuf {
    loaded
        .root
        .join("target")
        .join("texo")
        .join(format!("{}.json", loaded.top))
}

fn checkpoint_html_path(checkpoint: &Path) -> PathBuf {
    let mut path = checkpoint.as_os_str().to_owned();
    path.push(".html");
    path.into()
}

fn bitstream_config_path(bitstream: &Path) -> PathBuf {
    let mut path = bitstream.as_os_str().to_owned();
    path.push(".config");
    path.into()
}

fn path_text(path: &Path) -> Result<&str, Box<dyn Error>> {
    path.to_str()
        .ok_or_else(|| format!("path is not valid UTF-8: {}", path.display()).into())
}

fn ensure_distinct_paths(
    input: &Path,
    output: &Path,
    input_label: &str,
    output_label: &str,
) -> Result<(), Box<dyn Error>> {
    let same_path = input == output
        || std::fs::canonicalize(input)
            .ok()
            .zip(std::fs::canonicalize(output).ok())
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

#[cfg(test)]
mod tests {
    use std::path::Path;

    use clap::{CommandFactory, Parser as _};

    use super::{Cli, Command, ensure_distinct_paths};

    #[test]
    fn parses_documented_pnr_command() {
        Cli::command().debug_assert();
        let cli = Cli::try_parse_from([
            "texo",
            "pnr",
            "project",
            "--top",
            "Top",
            "--architecture",
            "device.txdb",
            "--package",
            "CABGA381",
            "--speed",
            "8_5G",
            "--lpf",
            "pins.lpf",
            "--output",
            "build/design.json",
            "--placement-weight-exponent",
            "2",
        ])
        .unwrap();
        let Command::Pnr(args) = cli.command else {
            panic!("expected pnr command");
        };
        assert_eq!(args.input, Path::new("project"));
        assert_eq!(args.top.as_deref(), Some("Top"));
        assert_eq!(args.speed, "8_5G");
        assert_eq!(args.placement_weight_exponent.get(), 2);
        assert!(!args.no_timing_optimization);
    }

    #[test]
    fn project_input_uses_manifest_top_by_default() {
        let cli = Cli::try_parse_from([
            "texo",
            "pnr",
            "project",
            "-a",
            "device.txdb",
            "-p",
            "CABGA381",
            "-s",
            "6",
        ])
        .unwrap();
        let Command::Pnr(args) = cli.command else {
            panic!("expected pnr command");
        };
        assert_eq!(args.input, Path::new("project"));
        assert_eq!(args.top, None);
    }

    #[test]
    fn pnr_resolves_the_default_target_pack_without_an_architecture_argument() {
        let cli = Cli::try_parse_from([
            "texo",
            "pnr",
            "project",
            "--package",
            "CABGA381",
            "--speed",
            "8",
        ])
        .unwrap();
        let Command::Pnr(args) = cli.command else {
            panic!("expected pnr command");
        };
        assert_eq!(args.architecture, None);
        assert_eq!(args.device, "LFE5UM5G-85F");
    }

    #[test]
    fn parses_target_pack_bitgen_command() {
        let cli = Cli::try_parse_from([
            "texo",
            "bitgen",
            "closed.checkpoint.json",
            "--bit",
            "design.bit",
        ])
        .unwrap();
        let Command::Bitgen(args) = cli.command else {
            panic!("expected bitgen command");
        };
        assert_eq!(args.checkpoint, Path::new("closed.checkpoint.json"));
        assert_eq!(args.bit, Path::new("design.bit"));
        assert_eq!(args.architecture, None);
        assert_eq!(args.target_pack, None);
    }

    #[test]
    fn rejects_zero_optimization_values() {
        let common = [
            "texo",
            "pnr",
            "project",
            "-t",
            "Top",
            "-a",
            "device.txdb",
            "-p",
            "CABGA381",
            "-s",
            "6",
        ];
        let mut with_zero_goal = common.to_vec();
        with_zero_goal.extend(["--synthesis-goal-mhz", "0"]);
        assert!(Cli::try_parse_from(with_zero_goal).is_err());

        let mut with_zero_weight = common.to_vec();
        with_zero_weight.extend(["--placement-weight-exponent", "0"]);
        assert!(Cli::try_parse_from(with_zero_weight).is_err());
    }

    #[test]
    fn rejects_an_output_that_overwrites_its_input() {
        let path = Path::new("Veryl.toml");
        assert!(ensure_distinct_paths(path, path, "source", "output").is_err());
    }
}
