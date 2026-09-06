//! Read placement, packing pairs and routes without accepting saved timing evidence.

use std::collections::BTreeMap;
use std::error::Error;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

use serde::Deserialize;
use texo_flow::Ecp5InitialRoute;
use texo_target_ecp5::Ecp5Architecture;

#[derive(Deserialize)]
pub(super) struct Checkpoint {
    schema_version: u32,
    target: Target,
    placement: Vec<SavedPlacement>,
    packing: Packing,
    pub routes: Vec<Ecp5InitialRoute>,
}

#[derive(Deserialize)]
struct Target {
    device: String,
    package: String,
    database_revision: String,
    project_trellis_revision: String,
}

#[derive(Deserialize)]
struct SavedPlacement {
    cell_id: usize,
    cell: String,
    bel: String,
}

#[derive(Deserialize)]
struct Packing {
    lut_ff_pairs: Vec<Pair>,
}

#[derive(Deserialize)]
struct Pair {
    lut: usize,
    ff: usize,
}

impl Checkpoint {
    pub fn read(path: &Path) -> Result<Self, Box<dyn Error>> {
        let saved: Self = serde_json::from_reader(BufReader::new(File::open(path)?))?;
        if saved.schema_version != 3 {
            return Err("resume requires checkpoint schema 3".into());
        }
        Ok(saved)
    }

    pub fn validate_target(
        &self,
        architecture: &Ecp5Architecture,
        package: &str,
    ) -> Result<(), Box<dyn Error>> {
        let provenance = architecture.provenance();
        if self.target.device != architecture.device().name()
            || self.target.package != package
            || self.target.database_revision != provenance.database_revision
            || self.target.project_trellis_revision != provenance.project_trellis_revision
        {
            return Err("resume checkpoint device/package/architecture revision mismatch".into());
        }
        Ok(())
    }

    pub fn placement(&self) -> Result<BTreeMap<String, String>, Box<dyn Error>> {
        let mut bindings = BTreeMap::new();
        for cell in &self.placement {
            if bindings
                .insert(cell.cell.clone(), cell.bel.clone())
                .is_some()
            {
                return Err(format!("duplicate resume cell {}", cell.cell).into());
            }
        }
        Ok(bindings)
    }

    pub fn pairs(&self) -> Result<BTreeMap<String, String>, Box<dyn Error>> {
        let mut names = BTreeMap::new();
        for cell in &self.placement {
            if names.insert(cell.cell_id, cell.cell.as_str()).is_some() {
                return Err("duplicate resume cell ID".into());
            }
        }
        let mut pairs = BTreeMap::new();
        for pair in &self.packing.lut_ff_pairs {
            let lut = *names.get(&pair.lut).ok_or("unknown resume LUT ID")?;
            let ff = *names.get(&pair.ff).ok_or("unknown resume FF ID")?;
            if pairs.insert(lut.to_owned(), ff.to_owned()).is_some() {
                return Err("duplicate resume LUT/FF pair".into());
            }
        }
        Ok(pairs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use texo_target_ecp5::read_architecture;

    fn saved() -> Checkpoint {
        serde_json::from_value(json!({
            "schema_version": 3,
            "target": {
                "device": "LFE5UM5G-85F-test", "package": "TEST",
                "database_revision": "015e0330630d7c238c0e4f2cdd9c8157eb78c54a",
                "project_trellis_revision": "3afe7b52b30f4b4417ee98f03016767a502006e3"
            },
            "placement": [
                {"cell_id": 18, "cell": "lut", "bel": "LUT0"},
                {"cell_id": 3, "cell": "ff", "bel": "FF0"}
            ],
            "packing": {"lut_ff_pairs": [{"lut": 18, "ff": 3}]},
            "routes": [],
            "timing": "saved timing is deliberately not consumed"
        }))
        .unwrap()
    }

    #[test]
    fn reconstructs_names_without_reusing_saved_numeric_cell_ids() {
        let saved = saved();
        assert_eq!(
            saved.placement().unwrap(),
            BTreeMap::from([("lut".into(), "LUT0".into()), ("ff".into(), "FF0".into()),])
        );
        assert_eq!(
            saved.pairs().unwrap(),
            BTreeMap::from([("lut".into(), "ff".into())])
        );
    }

    #[test]
    fn rejects_duplicate_or_missing_placement_and_pair_references() {
        let mut checkpoint = saved();
        checkpoint.placement[1].cell = "lut".into();
        assert!(checkpoint.placement().is_err());
        let mut checkpoint = saved();
        checkpoint.placement[1].cell_id = 18;
        assert!(checkpoint.pairs().is_err());
        let mut checkpoint = saved();
        checkpoint.packing.lut_ff_pairs[0].ff = 999;
        assert!(checkpoint.pairs().is_err());
        let mut checkpoint = saved();
        checkpoint
            .packing
            .lut_ff_pairs
            .push(Pair { lut: 18, ff: 3 });
        assert!(checkpoint.pairs().is_err());
    }

    #[test]
    fn rejects_foreign_device_package_and_architecture_revisions() {
        let architecture = read_architecture(
            include_bytes!("../../texo-target-ecp5/fixtures/minimal-ecp5.json").as_slice(),
        )
        .unwrap();
        saved().validate_target(&architecture, "TEST").unwrap();
        assert!(saved().validate_target(&architecture, "OTHER").is_err());
        let mut checkpoint = saved();
        checkpoint.target.device = "other".into();
        assert!(checkpoint.validate_target(&architecture, "TEST").is_err());
        let mut checkpoint = saved();
        checkpoint.target.database_revision = "old".into();
        assert!(checkpoint.validate_target(&architecture, "TEST").is_err());
        let mut checkpoint = saved();
        checkpoint.target.project_trellis_revision = "old".into();
        assert!(checkpoint.validate_target(&architecture, "TEST").is_err());
    }
}
