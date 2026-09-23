//! All-or-nothing installation of an empirically fitted STA timing library.
use serde::Deserialize;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::error::Error;
use std::path::Path;
use texo_target_ecp5::{Ecp5Architecture, SpeedGradeRecord};

#[derive(Deserialize)]
struct Library {
    schema: u32,
    kind: String,
    device: String,
    package: String,
    input_sha256: BTreeMap<String, String>,
    qualification: Qualification,
    timing: SpeedGradeRecord,
    entry_evidence: BTreeMap<String, Vec<String>>,
}
#[derive(Deserialize)]
struct Qualification {
    heldout_passed: bool,
    no_legacy_delay_labels: bool,
    independent_voltage_sweep: bool,
    temperature_fraction: f64,
    required_setup_hold_guard_ps: u64,
    validated_samples: BTreeMap<String, String>,
}

/// Verify measurements and atomically replace the complete selected timing grade.
///
/// # Errors
/// Rejects hash mismatches, failed holdouts, unsupported targets and missing entries.
pub fn install(
    path: &Path,
    architecture: &mut Ecp5Architecture,
    package: &str,
    expected_sha256: Option<&str>,
) -> Result<Value, Box<dyn Error>> {
    let bytes = std::fs::read(path)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    if expected_sha256.is_some_and(|expected| expected != digest) {
        return Err("STA library differs from the implemented checkpoint".into());
    }
    let library: Library = serde_json::from_slice(&bytes)?;
    if library.schema != 1
        || library.kind != "measured_joint_cell_route_sta_library"
        || library.device != architecture.device().name()
        || library.package != package
        || !library.qualification.heldout_passed
        || !library.qualification.no_legacy_delay_labels
        || library.qualification.validated_samples.is_empty()
        || library.input_sha256.is_empty()
        || !library.qualification.temperature_fraction.is_finite()
        || library.qualification.temperature_fraction < 0.20
        || library.qualification.required_setup_hold_guard_ps == 0
    {
        return Err("unqualified or incompatible measured STA library".into());
    }
    for (name, expected) in &library.input_sha256 {
        if format!("{:x}", Sha256::digest(std::fs::read(name)?)) != *expected {
            return Err(format!("measurement input changed: {name}").into());
        }
    }
    for path in library.qualification.validated_samples.values() {
        if !library.input_sha256.contains_key(path) {
            return Err("sample evidence is not hash-bound".into());
        }
    }
    let mut keys = Vec::new();
    for class in library.timing.pip_classes.keys() {
        keys.push(format!("pip:{class}"));
    }
    for cell in &library.timing.cells {
        for a in &cell.arcs {
            keys.push(format!(
                "arc:{}:{}:{}",
                cell.cell_type, a.from_pin, a.to_pin
            ));
        }
        for h in &cell.setup_holds {
            keys.push(format!(
                "check:{}:{}:{}",
                cell.cell_type, h.signal_pin, h.clock_pin
            ));
        }
    }
    if keys.len() != library.entry_evidence.len() {
        return Err("timing evidence surface differs from the table".into());
    }
    for key in &keys {
        let refs = library
            .entry_evidence
            .get(key)
            .ok_or_else(|| format!("unmeasured STA entry: {key}"))?;
        if refs.is_empty()
            || refs
                .iter()
                .any(|r| !library.qualification.validated_samples.contains_key(r))
        {
            return Err(format!("missing measurement support for {key}").into());
        }
    }
    let result = json!({"path":path.canonicalize()?.display().to_string(),"sha256":digest,
        "kind":library.kind,"timing_grade":library.timing.name,"entries":keys.len(),
        "legacy_table_fallback":false,"lut_input_timing":"routed_physical_pin","temperature_fraction":library.qualification.temperature_fraction,
        "voltage_sweep_measured":library.qualification.independent_voltage_sweep,
        "required_setup_hold_guard_ps":library.qualification.required_setup_hold_guard_ps});
    architecture
        .replace_timing_grade(library.timing)
        .map_err(|s| -> Box<dyn Error> { s.into() })?;
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn changed_measurement_and_incomplete_surface_never_install() {
        let mut architecture = texo_target_ecp5::read_architecture(
            include_str!("../../texo-target-ecp5/fixtures/minimal-ecp5.json").as_bytes(),
        )
        .unwrap();
        let original = architecture.speed_grades()["6"].clone();
        let directory = std::env::temp_dir().join(format!(
            "texo-measured-library-test-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir(&directory).unwrap();
        let evidence = directory.join("explicit-synthetic-unit-test.txt");
        std::fs::write(
            &evidence,
            b"synthetic parser fixture; never a production measurement",
        )
        .unwrap();
        let digest = format!("{:x}", Sha256::digest(std::fs::read(&evidence).unwrap()));
        let mut refs = BTreeMap::new();
        for name in original.pip_classes.keys() {
            refs.insert(format!("pip:{name}"), vec!["test"]);
        }
        for cell in &original.cells {
            for arc in &cell.arcs {
                refs.insert(
                    format!("arc:{}:{}:{}", cell.cell_type, arc.from_pin, arc.to_pin),
                    vec!["test"],
                );
            }
            for check in &cell.setup_holds {
                refs.insert(
                    format!(
                        "check:{}:{}:{}",
                        cell.cell_type, check.signal_pin, check.clock_pin
                    ),
                    vec!["test"],
                );
            }
        }
        let mut library = json!({"schema":1,"kind":"measured_joint_cell_route_sta_library",
            "device":architecture.device().name(),"package":"test-package",
            "input_sha256":{evidence.display().to_string():digest},
            "qualification":{"heldout_passed":true,"no_legacy_delay_labels":true,"independent_voltage_sweep":false,
                "temperature_fraction":0.2,"required_setup_hold_guard_ps":201,
                "validated_samples":{"test":evidence.display().to_string()}},
            "timing":original,"entry_evidence":refs});
        let file = directory.join("library.json");
        std::fs::write(&file, serde_json::to_vec(&library).unwrap()).unwrap();
        assert!(
            install(
                &file,
                &mut architecture,
                "test-package",
                Some("wrong digest")
            )
            .is_err()
        );
        std::fs::write(&evidence, b"changed measurement").unwrap();
        assert!(install(&file, &mut architecture, "test-package", None).is_err());
        assert_eq!(architecture.speed_grades()["6"], original);
        std::fs::write(
            &evidence,
            b"synthetic parser fixture; never a production measurement",
        )
        .unwrap();
        let complete = library.clone();
        library["timing"]["cells"][0]["arcs"]
            .as_array_mut()
            .unwrap()
            .clear();
        library["entry_evidence"]
            .as_object_mut()
            .unwrap()
            .remove("arc:DCCA:CLKI:CLKO");
        std::fs::write(&file, serde_json::to_vec(&library).unwrap()).unwrap();
        assert!(install(&file, &mut architecture, "test-package", None).is_err());
        assert_eq!(architecture.speed_grades()["6"], original);
        std::fs::write(&file, serde_json::to_vec(&complete).unwrap()).unwrap();
        let provenance = install(&file, &mut architecture, "test-package", None).unwrap();
        assert_eq!(provenance["legacy_table_fallback"], false);
        assert_eq!(provenance["required_setup_hold_guard_ps"], 201);
        std::fs::remove_dir_all(directory).unwrap();
    }
}
