//! Texo command-line entry point.

use std::env;
use std::error::Error;
use std::fs::File;
use std::io::{BufReader, BufWriter, Write};
use std::num::NonZeroU32;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::Instant;

use clap::{Args, Parser, Subcommand};
use struo_synth::synthesize;
use struo_target_ecp5::{ECP5_QOR_TARGET_MHZ, MappingOptions, map_to_ecp5_with_options};
use texo_cli::{VerylProject, ecp5_checkpoint, load_veryl_project, write_checkpoint_visualizer};
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
struct PnrArgs {
    /// Project directory or `Veryl.toml`.
    input: PathBuf,
    /// Top module; overrides `[synth].top`.
    #[arg(short, long)]
    top: Option<String>,
    /// ECP5 architecture JSON or `.txdb` cache.
    #[arg(short, long)]
    architecture: PathBuf,
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
        Command::Visualize { checkpoint, output } => {
            let output = output.unwrap_or_else(|| checkpoint_html_path(&checkpoint));
            ensure_distinct_paths(&checkpoint, &output, "checkpoint", "HTML output")?;
            write_checkpoint_visualizer(path_text(&checkpoint)?, path_text(&output)?)?;
            println!("visualizer: {}", output.display());
            Ok(())
        }
    }
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
    let architecture = load_architecture(&args.architecture)?;
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
