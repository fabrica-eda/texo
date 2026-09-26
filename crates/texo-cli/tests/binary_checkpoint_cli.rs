//! Exercise default binary output and its real CLI consumers on a small design.
use std::{fs, path::Path, process::Command};

fn successful(command: &mut Command) {
    let output = command.output().unwrap();
    assert!(
        output.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn default_checkpoint_resumes_and_visualizes_without_json_staging() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let fixture = root.join("examples/xor");
    let directory = tempfile::tempdir().unwrap();
    let project = directory.path();
    fs::create_dir(project.join("src")).unwrap();
    for name in ["Veryl.toml", "Veryl.lock", "xor.lpf"] {
        fs::copy(fixture.join(name), project.join(name)).unwrap();
    }
    for entry in fs::read_dir(fixture.join("src")).unwrap() {
        let entry = entry.unwrap();
        fs::copy(entry.path(), project.join("src").join(entry.file_name())).unwrap();
    }
    let pnr = || {
        let mut command = Command::new(env!("CARGO_BIN_EXE_texo"));
        command
            .arg("pnr")
            .arg(project)
            .arg("--architecture")
            .arg(root.join("crates/texo-target-ecp5/fixtures/minimal-ecp5.json"))
            .args(["--package", "CABGA381", "--speed", "6", "--lpf"])
            .arg(project.join("xor.lpf"));
        command
    };
    successful(&mut pnr());
    let checkpoint = project.join("target/texo/Xor.txcp");
    assert!(fs::read(&checkpoint).unwrap().starts_with(b"TEXOCP\x01\n"));
    assert!(!project.join("target/texo/Xor.json").exists());
    let first: serde_json::Value = texo_cli::read_checkpoint(&checkpoint).unwrap();
    let resumed = project.join("resumed.txcp");
    successful(
        pnr()
            .arg("--resume-checkpoint")
            .arg(&checkpoint)
            .arg("--output")
            .arg(&resumed),
    );
    let second: serde_json::Value = texo_cli::read_checkpoint(&resumed).unwrap();
    for key in ["placement", "packing", "routes", "timing"] {
        assert_eq!(first[key], second[key], "{key}");
    }
    let html = project.join("view.html");
    successful(
        Command::new(env!("CARGO_BIN_EXE_texo"))
            .arg("visualize")
            .arg(&checkpoint)
            .arg("--output")
            .arg(&html),
    );
    assert!(fs::metadata(html).unwrap().len() > 0);
}
