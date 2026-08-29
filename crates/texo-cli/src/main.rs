//! Texo command-line entry point.

use std::collections::{BTreeMap, BTreeSet};
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
use struo_target_ecp5::{
    ECP5_QOR_TARGET_MHZ, Ecp5Netlist, JtaggBinding, MappingOptions, OpenDrainIo, PhysicalFeedback,
    PhysicalLocation, PhysicalNetTiming, PhysicalTimingEndpoint, PllBinding,
    map_to_ecp5_with_options,
};
use texo_cli::{
    Ecp5BitgenOptions, Ecp5BitgenPaths, Ecp5BitgenRuntime, VerylProject, bitgen, ecp5_checkpoint,
    install_ecp5_target_pack, load_veryl_project, resolve_ecp5_target, write_checkpoint_visualizer,
};
use texo_flow::{
    Ecp5FlowError, Ecp5FlowOptions, Ecp5FlowResult, Ecp5FlowStage, Evidence, Gate,
    PhysicalFeedbackPolicy, PostMapSimulationPolicy, RoutingProgress,
    implement_struo_ecp5_with_progress,
};
use texo_pnr::PnrError;
use texo_struo::import_ecp5;
use texo_target_ecp5::{
    Ecp5Architecture, parse_lpf, read_architecture, read_architecture_cache,
    write_architecture_cache,
};

const MAX_PHYSICAL_SYNTHESIS_ROUNDS: usize = 3;

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
    /// Bind `PIN:INPUT:DRIVE_LOW` as one physical open-drain pad (repeatable).
    #[arg(
        long = "open-drain",
        value_name = "PIN:INPUT:DRIVE_LOW",
        value_parser = parse_open_drain
    )]
    open_drain: Vec<OpenDrainIo>,
    /// Bind `<PREFIX>_*` scalar ports to the dedicated ECP5 JTAG block.
    #[arg(long, value_name = "PREFIX")]
    jtagg_prefix: Option<String>,
    /// Disable JTAG extension register two.
    #[arg(long, requires = "jtagg_prefix")]
    jtagg_disable_er2: bool,
    /// Apply a user-owned EHXPLLL boundary binding from JSON (repeatable).
    #[arg(long, value_name = "JSON")]
    pll_binding: Vec<PathBuf>,
}

fn parse_open_drain(value: &str) -> Result<OpenDrainIo, String> {
    let mut fields = value.split(':');
    let pin = fields.next().unwrap_or_default();
    let input = fields.next().unwrap_or_default();
    let drive_low = fields.next().unwrap_or_default();
    if pin.is_empty() || input.is_empty() || drive_low.is_empty() || fields.next().is_some() {
        return Err("expected PIN:INPUT:DRIVE_LOW".into());
    }
    Ok(OpenDrainIo::new(pin, input, drive_low))
}

fn bind_target_primitives(mapped: &mut Ecp5Netlist, args: &PnrArgs) -> Result<(), Box<dyn Error>> {
    mapped.bind_open_drain_ios(&args.open_drain)?;
    if let Some(prefix) = args.jtagg_prefix.as_deref() {
        mapped.bind_jtagg(&jtagg_binding(prefix, args.jtagg_disable_er2))?;
    }
    for binding in load_pll_bindings(&args.pll_binding)? {
        mapped.bind_pll(&binding)?;
    }
    Ok(())
}

fn jtagg_binding(prefix: &str, disable_er2: bool) -> JtaggBinding {
    let mut binding = JtaggBinding::with_prefix(prefix);
    binding.extension_register_2 = !disable_er2;
    binding
}

fn load_pll_bindings(paths: &[PathBuf]) -> Result<Vec<PllBinding>, Box<dyn Error>> {
    paths
        .iter()
        .map(|path| {
            let file = File::open(path).map_err(|error| {
                format!("failed to read PLL binding `{}`: {error}", path.display())
            })?;
            serde_json::from_reader(BufReader::new(file)).map_err(|error| {
                format!("invalid PLL binding `{}`: {error}", path.display()).into()
            })
        })
        .collect()
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
        Command::Bitgen(args) => run_bitgen(&args),
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

fn run_bitgen(args: &BitgenArgs) -> Result<(), Box<dyn Error>> {
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
    let runtime = if let Some(root) = &args.target_pack {
        Ecp5BitgenRuntime::TargetPack(root.clone())
    } else if explicit {
        Ecp5BitgenRuntime::Explicit(Ecp5BitgenPaths {
            architecture: args.architecture.clone().expect("validated above"),
            database: args.database.clone().expect("validated above"),
            base_config: args.base_config.clone().expect("validated above"),
            ecppack: args.ecppack.clone().expect("validated above"),
        })
    } else {
        Ecp5BitgenRuntime::Auto
    };
    let output = bitgen(&Ecp5BitgenOptions {
        checkpoint: args.checkpoint.clone(),
        bitstream: args.bit.clone(),
        configuration: args.config.clone(),
        runtime,
    })?;
    println!(
        "Texo native bitgen: {} programmable PIPs, {} fixed edges",
        output.programmable_pips, output.fixed_edges
    );
    println!("configuration: {}", output.configuration.display());
    println!("bitstream: {}", output.bitstream.display());
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

#[allow(clippy::too_many_lines)]
fn pnr(args: &PnrArgs) -> Result<(), Box<dyn Error>> {
    let flow_started = Instant::now();
    let (loaded, output) = prepare_veryl_input(args)?;

    let synthesized = synthesize(&loaded.design)?;
    for report in &synthesized.reports {
        println!("synthesis {}: {}", report.pass, report.message);
    }
    let mut mapped = map_to_ecp5_with_options(
        &synthesized.netlist,
        MappingOptions {
            timing_goal_mhz: args.synthesis_goal_mhz.get(),
            ..MappingOptions::default()
        },
    )?;
    bind_target_primitives(&mut mapped, args)?;
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
        physical_feedback: PhysicalFeedbackPolicy::HandoffAfterLocal,
        ..Ecp5FlowOptions::default()
    };
    if let Some(fanout) = args.global_clock_fanout {
        options.global_clock_fanout = fanout;
    }
    let mut phase_started = Instant::now();
    let mut result = implement_struo_ecp5_with_progress(
        &imported,
        &architecture,
        options,
        &mut evidence,
        |stage| report_flow_stage(stage, &mut phase_started),
    )?;

    for round in 1..=MAX_PHYSICAL_SYNTHESIS_ROUNDS {
        if !options.optimize_timing || result.timing.met_timing() {
            break;
        }
        if physical_rewrite_is_coarser_than_remaining_miss(result.timing.worst_slack_ps) {
            println!(
                "physical synthesis: retained route-level near-closure result (WNS {})",
                format_slack(result.timing.worst_slack_ps)
            );
            break;
        }
        let feedback =
            native_physical_feedback(&result, &architecture, args.synthesis_goal_mhz.get());
        let mut physical_candidates = mapped.physical_feedback_candidates(&feedback).into_iter();
        if let Some(refined) = physical_candidates.next() {
            println!(
                "physical synthesis round {round}: {} equivalent logic replicas, {} physical rewires",
                refined
                    .retiming()
                    .equivalent_logic_replications
                    .saturating_sub(mapped.retiming().equivalent_logic_replications),
                refined
                    .retiming()
                    .equivalent_physical_rewires
                    .saturating_sub(mapped.retiming().equivalent_physical_rewires),
            );
            let refined_imported = import_ecp5(&refined)?;
            let refined_cell_names = refined_imported
                .design()
                .cells()
                .iter()
                .map(|cell| cell.name.as_str())
                .collect::<BTreeSet<_>>();
            let inherited_placement = result
                .design
                .cells()
                .iter()
                .enumerate()
                .filter_map(|(index, cell)| {
                    refined_cell_names.contains(cell.name.as_str()).then(|| {
                        let bel = result
                            .implementation
                            .placement
                            .bel(texo_model::CellId(index))?;
                        Some((
                            cell.name.clone(),
                            architecture.device().bels()[bel.0].name.clone(),
                        ))
                    })?
                })
                .collect::<BTreeMap<_, _>>();
            println!(
                "physical synthesis round {round}: inherited {} stable cell placements",
                inherited_placement.len()
            );
            let refined_options = Ecp5FlowOptions {
                initial_placement: Some(&inherited_placement),
                incremental_seed: Some(&result),
                physical_feedback: PhysicalFeedbackPolicy::CompleteClosure,
                ..options
            };
            phase_started = Instant::now();
            let refined_result = match implement_struo_ecp5_with_progress(
                &refined_imported,
                &architecture,
                refined_options,
                &mut evidence,
                |stage| report_flow_stage(stage, &mut phase_started),
            ) {
                Ok(result) => result,
                Err(Ecp5FlowError::Pnr(PnrError::InvalidPlacement { reason })) => {
                    eprintln!(
                        "physical synthesis round {round}: inherited placement rejected ({reason}); retrying native placement"
                    );
                    phase_started = Instant::now();
                    implement_struo_ecp5_with_progress(
                        &refined_imported,
                        &architecture,
                        options,
                        &mut evidence,
                        |stage| report_flow_stage(stage, &mut phase_started),
                    )?
                }
                Err(error) => return Err(error.into()),
            };
            let incumbent_wns = result.timing.worst_slack_ps.unwrap_or(i128::MIN);
            let refined_wns = refined_result.timing.worst_slack_ps.unwrap_or(i128::MIN);
            if refined_wns > incumbent_wns {
                mapped = refined;
                result = refined_result;
                println!(
                    "physical synthesis round {round}: accepted (WNS {incumbent_wns} ps -> {refined_wns} ps)"
                );
            } else {
                println!(
                    "physical synthesis round {round}: rejected (WNS {incumbent_wns} ps -> {refined_wns} ps)"
                );
                break;
            }
        } else {
            println!("physical synthesis round {round}: no equivalent rewrite candidate");
            break;
        }
    }

    if matches!(
        options.physical_feedback,
        PhysicalFeedbackPolicy::HandoffAfterLocal
    ) && !result.timing.met_timing()
    {
        println!("physical synthesis handoff did not close timing; resuming critical closure");
        let resumed_imported = import_ecp5(&mapped)?;
        let resumed_cell_names = resumed_imported
            .design()
            .cells()
            .iter()
            .map(|cell| cell.name.as_str())
            .collect::<BTreeSet<_>>();
        let resumed_placement = result
            .design
            .cells()
            .iter()
            .enumerate()
            .filter_map(|(index, cell)| {
                resumed_cell_names.contains(cell.name.as_str()).then(|| {
                    let bel = result
                        .implementation
                        .placement
                        .bel(texo_model::CellId(index))?;
                    Some((
                        cell.name.clone(),
                        architecture.device().bels()[bel.0].name.clone(),
                    ))
                })?
            })
            .collect::<BTreeMap<_, _>>();
        phase_started = Instant::now();
        result = implement_struo_ecp5_with_progress(
            &resumed_imported,
            &architecture,
            Ecp5FlowOptions {
                initial_placement: Some(&resumed_placement),
                incremental_seed: Some(&result),
                physical_feedback: PhysicalFeedbackPolicy::CompleteClosure,
                ..options
            },
            &mut evidence,
            |stage| report_flow_stage(stage, &mut phase_started),
        )?;
    }

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
        "timing: WNS {}, WHS {}, {}; endpoints {}/{} checked",
        format_slack(result.timing.worst_slack_ps),
        format_slack(result.timing.worst_hold_slack_ps),
        if result.timing.met_timing() {
            "met"
        } else {
            "not met"
        },
        result.timing.setup_checks.len(),
        result.timing.modeled_endpoint_count(),
    );
    println!("checkpoint: {}", output.display());
    println!(
        "verification: mapping equivalence passed; post-map functional simulation not supplied"
    );
    Ok(())
}

fn native_physical_feedback(
    result: &Ecp5FlowResult,
    architecture: &Ecp5Architecture,
    timing_goal_mhz: u32,
) -> PhysicalFeedback {
    let placement = &result.implementation.placement;
    let placements = result
        .design
        .cells()
        .iter()
        .enumerate()
        .filter_map(|(index, cell)| {
            let point = placement.point(texo_model::CellId(index), architecture.device())?;
            Some((
                cell.name.clone(),
                PhysicalLocation {
                    x: i32::try_from(point.x).ok()?,
                    y: i32::try_from(point.y).ok()?,
                },
            ))
        })
        .collect::<BTreeMap<_, _>>();
    let bels = result
        .design
        .cells()
        .iter()
        .enumerate()
        .filter_map(|(index, cell)| {
            let bel = placement.bel(texo_model::CellId(index))?;
            Some((
                cell.name.clone(),
                architecture.device().bels()[bel.0].name.clone(),
            ))
        })
        .collect::<BTreeMap<_, _>>();
    let slacks = result
        .timing
        .net_setup_slacks
        .iter()
        .map(|edge| ((edge.net, edge.sink), edge.slack_ps))
        .collect::<BTreeMap<_, _>>();
    let mut net_timings = BTreeMap::<texo_model::NetId, PhysicalNetTiming>::new();
    for delay in &result.timing.net_delays {
        let net = &result.design.nets()[delay.net.0];
        let driver = &result.design.pins()[net.driver.0];
        let sink = &result.design.pins()[delay.sink.0];
        let delay_ps = u32::try_from(delay.delay.max_ps).unwrap_or(u32::MAX);
        let slack_ps = slacks.get(&(delay.net, delay.sink)).copied().unwrap_or(0);
        let deficit_ps = u64::try_from(slack_ps.unsigned_abs()).unwrap_or(u64::MAX);
        let budget_ps = if slack_ps < 0 {
            delay
                .delay
                .max_ps
                .saturating_sub(deficit_ps)
                .max(delay.delay.max_ps / 2)
        } else {
            delay.delay.max_ps
        };
        net_timings
            .entry(delay.net)
            .or_insert_with(|| PhysicalNetTiming {
                driver: result.design.cells()[driver.cell.0].name.clone(),
                net: net.name.clone(),
                endpoints: Vec::new(),
            })
            .endpoints
            .push(PhysicalTimingEndpoint {
                cell: result.design.cells()[sink.cell.0].name.clone(),
                port: sink.name.clone(),
                delay_ps,
                budget_ps: u32::try_from(budget_ps).unwrap_or(u32::MAX),
            });
    }
    let goal_khz = timing_goal_mhz.saturating_mul(1_000).max(1);
    let goal_period_ps = 1_000_000_000_u64 / u64::from(goal_khz);
    let worst_slack_ps = result.timing.worst_slack_ps.unwrap_or(0);
    let achieved_period_ps = if worst_slack_ps < 0 {
        goal_period_ps
            .saturating_add(u64::try_from(worst_slack_ps.unsigned_abs()).unwrap_or(u64::MAX))
    } else {
        goal_period_ps.saturating_sub(u64::try_from(worst_slack_ps).unwrap_or(u64::MAX))
    }
    .max(1);
    let achieved_khz = u32::try_from(1_000_000_000_u64 / achieved_period_ps).unwrap_or(u32::MAX);
    PhysicalFeedback::from_observations(
        placements,
        bels,
        net_timings.into_values().collect(),
        Vec::new(),
        BTreeMap::from([("native".into(), (achieved_khz, goal_khz))]),
    )
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

fn physical_rewrite_is_coarser_than_remaining_miss(worst_slack_ps: Option<i128>) -> bool {
    let route_refinement_window_ps =
        i128::from(texo_pnr::ROUTING_DELAY_QUANTUM_PS.saturating_mul(2));
    worst_slack_ps.is_some_and(|slack| (-route_refinement_window_ps..0).contains(&slack))
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

    use super::{
        Cli, Command, ensure_distinct_paths, jtagg_binding,
        physical_rewrite_is_coarser_than_remaining_miss,
    };

    #[test]
    fn physical_rewrites_defer_to_near_closure_route_repair() {
        let window = i128::from(texo_pnr::ROUTING_DELAY_QUANTUM_PS * 2);

        assert!(!physical_rewrite_is_coarser_than_remaining_miss(None));
        assert!(physical_rewrite_is_coarser_than_remaining_miss(Some(
            -window
        )));
        assert!(physical_rewrite_is_coarser_than_remaining_miss(Some(
            -window + 1
        )));
        assert!(physical_rewrite_is_coarser_than_remaining_miss(Some(-1)));
        assert!(!physical_rewrite_is_coarser_than_remaining_miss(Some(0)));
    }

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
    fn parses_repeatable_open_drain_pad_bindings() {
        let cli = Cli::try_parse_from([
            "texo",
            "pnr",
            "project",
            "--package",
            "CABGA381",
            "--speed",
            "8",
            "--open-drain",
            "sda:sda_i:sda_drive_low",
            "--open-drain",
            "scl:scl_i:scl_drive_low",
        ])
        .unwrap();
        let Command::Pnr(args) = cli.command else {
            panic!("expected pnr command");
        };
        assert_eq!(args.open_drain.len(), 2);
        assert_eq!(args.open_drain[0].pin, "sda");
        assert_eq!(args.open_drain[0].input_port, "sda_i");
        assert_eq!(args.open_drain[0].drive_low_port, "sda_drive_low");
        assert_eq!(args.open_drain[1].pin, "scl");
        assert!(
            Cli::try_parse_from([
                "texo",
                "pnr",
                "project",
                "--package",
                "CABGA381",
                "--speed",
                "8",
                "--open-drain",
                "sda:sda_i",
            ])
            .is_err()
        );
    }

    #[test]
    fn parses_jtagg_prefix_binding() {
        let cli = Cli::try_parse_from([
            "texo",
            "pnr",
            "project",
            "--package",
            "CABGA381",
            "--speed",
            "8",
            "--jtagg-prefix",
            "debug",
            "--jtagg-disable-er2",
        ])
        .unwrap();
        let Command::Pnr(args) = cli.command else {
            panic!("expected pnr command");
        };

        assert_eq!(args.jtagg_prefix.as_deref(), Some("debug"));
        assert!(args.jtagg_disable_er2);
        assert!(!jtagg_binding("debug", args.jtagg_disable_er2).extension_register_2);
        assert!(jtagg_binding("debug", false).extension_register_2);
        assert!(
            Cli::try_parse_from([
                "texo",
                "pnr",
                "project",
                "--package",
                "CABGA381",
                "--speed",
                "8",
                "--jtagg-disable-er2",
            ])
            .is_err()
        );
    }

    #[test]
    fn parses_repeatable_pll_bindings() {
        let cli = Cli::try_parse_from([
            "texo",
            "pnr",
            "project",
            "--package",
            "CABGA381",
            "--speed",
            "8",
            "--pll-binding",
            "pll-a.json",
            "--pll-binding",
            "pll-b.json",
        ])
        .unwrap();
        let Command::Pnr(args) = cli.command else {
            panic!("expected pnr command");
        };

        assert_eq!(
            args.pll_binding,
            [Path::new("pll-a.json"), Path::new("pll-b.json")]
        );
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
