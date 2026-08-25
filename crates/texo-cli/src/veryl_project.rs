//! Whole-project Veryl analysis for the physical-design CLI.

use std::error::Error;
use std::fmt::{self, Display, Formatter};
use std::path::{Path, PathBuf};

use struo_frontend_veryl::{analyze_and_lower, lower_analyzed_ir};
use struo_rtl::Design;
use veryl_analyzer::ir::Ir;
use veryl_analyzer::{Analyzer, AnalyzerError, Context};
use veryl_metadata::Metadata;
use veryl_parser::Parser;

/// Fully analyzed and lowered Veryl input.
#[derive(Debug)]
pub struct VerylDesign {
    /// Frontend-independent RTL selected for synthesis.
    pub design: Design,
    /// Resolved top module, including a `[synth].top` default when present.
    pub top: String,
    /// Project namespace used by the Veryl analyzer.
    pub project: String,
    /// Project root, or the standalone source's parent directory.
    pub root: PathBuf,
    /// `Veryl.toml` for project input; absent for standalone source input.
    pub manifest: Option<PathBuf>,
    /// Number of project-owned source compilation units.
    pub project_sources: usize,
    /// Number of analyzed units including dependencies and the standard library.
    pub total_sources: usize,
    /// All analyzed sources protected from checkpoint overwrite.
    pub source_paths: Vec<PathBuf>,
    /// Non-fatal analyzer diagnostics retained for reporting.
    pub warnings: Vec<String>,
}

/// Loads either a Veryl project directory/manifest or one standalone source.
///
/// Project input expands `[build].sources`, the standard library, and all
/// dependencies through Veryl's metadata and lockfile implementation. The top
/// comes from `top_override` first and `[synth].top` second. A standalone source
/// requires an explicit top.
///
/// # Errors
///
/// Returns an error for an unsupported input path, missing project top,
/// metadata/dependency resolution failure, source IO, parser/analyzer
/// diagnostics, or unsupported Struo lowering.
pub fn load_veryl_design(
    input: &Path,
    top_override: Option<&str>,
) -> Result<VerylDesign, Box<dyn Error>> {
    if input.is_dir() || input.file_name().is_some_and(|name| name == "Veryl.toml") {
        load_project(input, top_override)
    } else if input
        .extension()
        .is_some_and(|extension| extension == "veryl")
    {
        load_standalone(input, top_override)
    } else {
        Err(ProjectInputError(format!(
            "Veryl input must be a project directory, Veryl.toml, or .veryl file: {}",
            input.display()
        ))
        .into())
    }
}

fn load_standalone(
    source_path: &Path,
    top_override: Option<&str>,
) -> Result<VerylDesign, Box<dyn Error>> {
    let top = top_override.ok_or_else(|| {
        ProjectInputError("--top is required for a standalone .veryl file".into())
    })?;
    let source = std::fs::read_to_string(source_path)?;
    let root = source_path.parent().unwrap_or_else(|| Path::new("."));
    let project = "standalone".to_owned();
    let design = analyze_and_lower(&source, &project, top)?;
    Ok(VerylDesign {
        design,
        top: top.into(),
        project,
        root: root.to_path_buf(),
        manifest: None,
        project_sources: 1,
        total_sources: 1,
        source_paths: vec![source_path.canonicalize()?],
        warnings: Vec::new(),
    })
}

fn load_project(input: &Path, top_override: Option<&str>) -> Result<VerylDesign, Box<dyn Error>> {
    clear_analyzer_tables();
    let manifest = if input.is_dir() {
        input.join("Veryl.toml")
    } else {
        input.to_path_buf()
    };
    let mut metadata = Metadata::load(&manifest)?;
    let top = top_override
        .map(str::to_owned)
        .or_else(|| metadata.synth.top.clone())
        .ok_or_else(|| {
            ProjectInputError(
                "no top selected; pass --top or set `top` in [synth] of Veryl.toml".into(),
            )
        })?;
    let paths = metadata.paths::<PathBuf>(&[], true, true)?;
    let project_sources = paths
        .iter()
        .filter(|path| path.prj == metadata.project.name)
        .count();
    let source_paths = paths
        .iter()
        .map(|path| path.src.clone())
        .collect::<Vec<_>>();
    if project_sources == 0 {
        return Err(ProjectInputError(format!(
            "Veryl project has no source files: {}",
            manifest.display()
        ))
        .into());
    }

    let mut parsed = Vec::with_capacity(paths.len());
    for path in &paths {
        let source = std::fs::read_to_string(&path.src)?;
        parsed.push((path, Parser::parse(&source, &path.src)?));
    }
    let analyzer = Analyzer::new(&metadata);
    let mut warnings = Vec::new();
    for (path, parser) in &parsed {
        require_no_errors(
            &format!("pass1 for {}", path.src.display()),
            analyzer.analyze_pass1(&path.prj, &parser.veryl),
            &mut warnings,
        )?;
    }
    require_no_errors("post-pass1", Analyzer::analyze_post_pass1(), &mut warnings)?;

    let mut context = Context::default();
    let mut ir = Ir::default();
    for (path, parser) in &parsed {
        context.set_project_name(&path.prj);
        require_no_errors(
            &format!("pass2 for {}", path.src.display()),
            analyzer.analyze_pass2(&parser.veryl, &mut context, Some(&mut ir)),
            &mut warnings,
        )?;
    }
    require_no_errors(
        "post-pass2",
        Analyzer::analyze_post_pass2(&ir),
        &mut warnings,
    )?;
    let design = lower_analyzed_ir(&ir, &top)?;

    Ok(VerylDesign {
        design,
        top,
        project: metadata.project.name.clone(),
        root: metadata.project_path(),
        manifest: Some(manifest.canonicalize()?),
        project_sources,
        total_sources: paths.len(),
        source_paths,
        warnings,
    })
}

fn clear_analyzer_tables() {
    veryl_analyzer::symbol_table::clear();
    veryl_analyzer::attribute_table::clear();
    veryl_parser::doc_comment_table::clear();
}

fn require_no_errors(
    stage: &str,
    diagnostics: Vec<AnalyzerError>,
    warnings: &mut Vec<String>,
) -> Result<(), ProjectInputError> {
    let (errors, non_fatal): (Vec<_>, Vec<_>) =
        diagnostics.into_iter().partition(AnalyzerError::is_error);
    warnings.extend(
        non_fatal
            .into_iter()
            .map(|diagnostic| format!("{stage}: {diagnostic:?}")),
    );
    if errors.is_empty() {
        Ok(())
    } else {
        Err(ProjectInputError(format!("{stage}: {errors:?}")))
    }
}

#[derive(Debug)]
struct ProjectInputError(String);

impl Display for ProjectInputError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.0)
    }
}

impl Error for ProjectInputError {}

#[cfg(test)]
mod tests {
    use std::fs;
    use std::path::PathBuf;

    use super::load_veryl_design;

    struct TemporaryProject(PathBuf);

    impl TemporaryProject {
        fn new() -> Self {
            let path = std::env::temp_dir().join(format!(
                "texo-veryl-project-{}-{:?}",
                std::process::id(),
                std::thread::current().id()
            ));
            let _ = fs::remove_dir_all(&path);
            fs::create_dir_all(path.join("src")).unwrap();
            fs::write(
                path.join("Veryl.toml"),
                r#"
[project]
name = "multi_file"
version = "0.1.0"
[build]
sources = ["src"]
exclude_std = true
target = { type = "directory", path = "target/veryl" }
[synth]
top = "Top"
"#,
            )
            .unwrap();
            fs::write(
                path.join("src/Gate.veryl"),
                r"
module Gate (
    lhs: input logic,
    rhs: input logic,
    result: output logic,
) {
    always_comb { result = lhs ^ rhs; }
}
",
            )
            .unwrap();
            fs::write(
                path.join("src/Top.veryl"),
                r"
module Top (
    lhs: input logic,
    rhs: input logic,
    value: output logic,
) {
    inst gate: Gate (
        lhs: lhs,
        rhs: rhs,
        result: value,
    );
}
",
            )
            .unwrap();
            Self(path)
        }
    }

    impl Drop for TemporaryProject {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn loads_all_project_units_and_uses_manifest_top() {
        let project = TemporaryProject::new();
        let loaded = load_veryl_design(&project.0, None).unwrap();

        assert_eq!(loaded.project, "multi_file");
        assert_eq!(loaded.top, "Top");
        assert_eq!(loaded.project_sources, 2);
        assert_eq!(loaded.total_sources, 2);
        assert!(loaded.design.top_module().is_some());
    }

    #[test]
    fn resolves_and_analyzes_local_project_dependencies() {
        let root = std::env::temp_dir().join(format!(
            "texo-veryl-dependency-{}-{:?}",
            std::process::id(),
            std::thread::current().id()
        ));
        let cleanup = TemporaryProject(root.clone());
        let main = root.join("main");
        let dependency = root.join("gate");
        fs::create_dir_all(main.join("src")).unwrap();
        fs::create_dir_all(dependency.join("src")).unwrap();
        fs::write(
            dependency.join("Veryl.toml"),
            r#"
[project]
name = "gate"
version = "0.1.0"
[build]
sources = ["src"]
exclude_std = true
target = { type = "directory", path = "target/veryl" }
"#,
        )
        .unwrap();
        fs::write(
            dependency.join("src/Gate.veryl"),
            r"
pub module Gate (
    input_value: input logic,
    output_value: output logic,
) {
    always_comb { output_value = ~input_value; }
}
",
        )
        .unwrap();
        fs::write(
            main.join("Veryl.toml"),
            r#"
[project]
name = "dependency_top"
version = "0.1.0"
[build]
sources = ["src"]
exclude_std = true
target = { type = "directory", path = "target/veryl" }
[synth]
top = "Top"
[dependencies]
gate = { path = "../gate" }
"#,
        )
        .unwrap();
        fs::write(
            main.join("src/Top.veryl"),
            r"
module Top (
    input_value: input logic,
    output_value: output logic,
) {
    inst inverter: gate::Gate (
        input_value : input_value,
        output_value: output_value,
    );
}
",
        )
        .unwrap();

        let loaded = load_veryl_design(&main, None).unwrap();

        assert_eq!(loaded.project_sources, 1);
        assert_eq!(loaded.total_sources, 2);
        assert!(loaded.design.top_module().is_some());
        drop(cleanup);
    }
}
