//! Export physical resource identities for measurements; delays are not labels.
use std::collections::HashMap;
use std::error::Error;
use std::fs::File;
use std::io::{BufReader, BufWriter};
use texo_model::PipId;
use texo_target_ecp5::read_architecture_cache;

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 4, "architecture.txdb requests.json output.json");
    let architecture = read_architecture_cache(BufReader::new(File::open(&args[1])?))?;
    let device = architecture.device();
    let requests: Vec<(String, String)> =
        serde_json::from_reader(BufReader::new(File::open(&args[2])?))?;
    let names: HashMap<_, _> = device
        .wires()
        .iter()
        .enumerate()
        .map(|(id, w)| (w.name.as_str(), id))
        .collect();
    let mut wanted: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for (index, (source, sink)) in requests.iter().enumerate() {
        if let (Some(&a), Some(&b)) = (names.get(source.as_str()), names.get(sink.as_str())) {
            wanted.entry((a, b)).or_default().push(index);
        }
    }
    drop(names);
    let mut matches = vec![Vec::new(); requests.len()];
    for (id, pip) in device.pips().iter().enumerate() {
        if let Some(indices) = wanted.get(&(pip.from().0, pip.to().0)) {
            let meta = architecture.pip_metadata(PipId(id));
            let value = serde_json::json!({"pip_id":id,"class":meta.timing_class,"fixed":meta.fixed,"lutperm_flags":meta.lutperm_flags});
            for &index in indices {
                matches[index].push(value.clone());
            }
        }
    }
    let covered = matches.iter().filter(|m| !m.is_empty()).count();
    serde_json::to_writer(
        BufWriter::new(File::create(&args[3])?),
        &serde_json::json!({
            "resource_identities_only":true,"request_count":requests.len(),"matched":covered,"matches":matches,
            "timing_surface_for_coverage_only":architecture.speed_grades()["8_5G"],
        }),
    )?;
    println!(
        "Mapped {covered}/{} requested physical arcs",
        requests.len()
    );
    Ok(())
}
