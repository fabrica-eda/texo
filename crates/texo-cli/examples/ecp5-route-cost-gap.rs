//! Quantify the static-fanout router cost versus final route timing fanout.

use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::error::Error;
use std::fs::File;
use std::io::BufReader;

use serde_json::Value;
use texo_model::{PipId, WireId};
use texo_target_ecp5::read_architecture_cache;

const USAGE: &str = "Usage: ecp5-route-cost-gap ARCH.txdb CHECKPOINT.json\n";

#[derive(Default)]
struct ClassStats {
    selected: u64,
    affected: u64,
    excess_ps: u64,
    maximum_excess_ps: u64,
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let architecture_path = args.next().ok_or(USAGE)?;
    let checkpoint_path = args.next().ok_or(USAGE)?;
    if args.next().is_some() {
        return Err(USAGE.into());
    }
    let architecture = read_architecture_cache(BufReader::new(File::open(architecture_path)?))?;
    let checkpoint: Value = serde_json::from_reader(BufReader::new(File::open(checkpoint_path)?))?;
    let mut selected = BTreeSet::<PipId>::new();
    for route in checkpoint["routes"]
        .as_array()
        .ok_or("checkpoint routes is not an array")?
    {
        for pip in route["pips"]
            .as_array()
            .ok_or("route pips is not an array")?
        {
            let id = pip["pip_id"].as_u64().ok_or("pip_id is not an integer")?;
            selected.insert(PipId(id.try_into()?));
        }
    }
    let mut fanout = BTreeMap::<WireId, u64>::new();
    for &pip in &selected {
        *fanout
            .entry(architecture.device().pips()[pip.0].from())
            .or_default() += 1;
    }
    let speed_name = checkpoint["target"]["speed_grade"]
        .as_str()
        .ok_or("checkpoint target speed_grade is not a string")?;
    let speed = architecture
        .speed_grades()
        .get(speed_name)
        .ok_or("checkpoint speed_grade is absent from architecture")?;
    println!("speed_grade={speed_name}");
    let nextpnr_witness_names = [
        "R38C80/Q6_SLICE",
        "R38C80/Q6",
        "R38C77/H06W0303",
        "R38C71/H06W0003",
        "R38C65/H06W0103",
        "R35C62/V06N0103",
        "R29C62/V06N0203",
        "R23C62/V06N0203",
        "R17C62/V06N0303",
        "R14C62/V01S0100",
        "R15C61/H02W0101",
        "R15C61/B3",
        "R15C61/C3_SLICE",
    ];
    let mut nextpnr_witness_wires = BTreeMap::<&str, WireId>::new();
    for (index, wire) in architecture.device().wires().iter().enumerate() {
        if nextpnr_witness_names.contains(&wire.name.as_str()) {
            nextpnr_witness_wires.insert(&wire.name, WireId(index));
        }
    }
    println!(
        "nextpnr_witness_named_wires={}/{}",
        nextpnr_witness_wires.len(),
        nextpnr_witness_names.len(),
    );
    for pair in nextpnr_witness_names.windows(2) {
        let from = nextpnr_witness_wires
            .get(pair[0])
            .copied()
            .ok_or("missing nextpnr witness source wire")?;
        let to = nextpnr_witness_wires
            .get(pair[1])
            .copied()
            .ok_or("missing nextpnr witness destination wire")?;
        let pip = architecture
            .device()
            .routing_neighbors(from)?
            .find_map(|(neighbor, pip)| (neighbor == to).then_some(pip))
            .ok_or("nextpnr witness edge is absent from Texo graph")?;
        let class_name = architecture.pip_metadata(pip).timing_class;
        let timing = &speed.pip_classes[class_name];
        println!(
            "nextpnr_witness_pip\t{}\t{}\t{}\t{}\t{}\t{}",
            pip.0, class_name, timing.base.max_ps, timing.fanout_adder.max_ps, pair[0], pair[1],
        );
    }
    let mut classes = BTreeMap::<String, ClassStats>::new();
    let mut affected = 0_u64;
    let mut total_excess_ps = 0_u64;
    let mut maximum_excess_ps = 0_u64;
    for &pip in &selected {
        let source = architecture.device().pips()[pip.0].from();
        let source_fanout = fanout[&source];
        let class_name = architecture.pip_metadata(pip).timing_class;
        let timing = &speed.pip_classes[class_name];
        let excess = timing
            .fanout_adder
            .max_ps
            .saturating_mul(source_fanout.saturating_sub(1));
        let stats = classes.entry(class_name.to_owned()).or_default();
        stats.selected += 1;
        if excess != 0 {
            affected += 1;
            stats.affected += 1;
        }
        stats.excess_ps += excess;
        stats.maximum_excess_ps = stats.maximum_excess_ps.max(excess);
        total_excess_ps += excess;
        maximum_excess_ps = maximum_excess_ps.max(excess);
    }
    let sources_gt_one = fanout.values().filter(|&&count| count > 1).count();
    let pips_from_sources_gt_one: u64 = fanout.values().filter(|&&count| count > 1).sum();
    println!(
        "selected_pips={} sources={} sources_gt1={} pips_from_sources_gt1={} max_fanout={} affected_nonzero_adder={} aggregate_excess_ps={} max_per_pip_excess_ps={}",
        selected.len(),
        fanout.len(),
        sources_gt_one,
        pips_from_sources_gt_one,
        fanout.values().copied().max().unwrap_or(0),
        affected,
        total_excess_ps,
        maximum_excess_ps,
    );
    println!("class\tselected\taffected\taggregate_excess_ps\tmax_per_pip_excess_ps");
    let mut ordered = classes.into_iter().collect::<Vec<_>>();
    ordered.sort_unstable_by_key(|(name, stats)| {
        (
            std::cmp::Reverse(stats.excess_ps),
            std::cmp::Reverse(stats.affected),
            name.clone(),
        )
    });
    for (name, stats) in ordered.into_iter().filter(|(_, stats)| stats.affected != 0) {
        println!(
            "{}\t{}\t{}\t{}\t{}",
            name, stats.selected, stats.affected, stats.excess_ps, stats.maximum_excess_ps
        );
    }
    // Fixed-placement witness for the CPU setup gap: this is the first routing
    // edge of nextpnr's reported critical path, using the same stable IDs in
    // the authoritative Texo checkpoint.
    let witness_route = checkpoint["routes"]
        .as_array()
        .and_then(|routes| routes.iter().find(|route| route["net_id"] == 1221))
        .ok_or("missing witness net 1221")?;
    let sink_wire = checkpoint["placement"]
        .as_array()
        .and_then(|placement| placement.iter().find(|cell| cell["cell_id"] == 3992))
        .and_then(|cell| cell["bel_pins"].as_array())
        .and_then(|pins| pins.iter().find(|pin| pin["name"] == "C"))
        .and_then(|pin| pin["wire_id"].as_u64())
        .ok_or("missing witness sink wire")?;
    let driver_wire = witness_route["driver_wire_id"]
        .as_u64()
        .ok_or("missing witness driver wire")?;
    let mut by_sink = BTreeMap::<u64, PipId>::new();
    for pip in witness_route["pips"]
        .as_array()
        .ok_or("witness route pips is not an array")?
    {
        by_sink.insert(
            pip["to_wire_id"].as_u64().ok_or("missing to_wire_id")?,
            PipId(pip["pip_id"].as_u64().ok_or("missing pip_id")?.try_into()?),
        );
    }
    let mut cursor = sink_wire;
    let mut witness_pips = Vec::new();
    while cursor != driver_wire {
        let pip = *by_sink
            .get(&cursor)
            .ok_or("witness route is disconnected")?;
        witness_pips.push(pip);
        cursor = architecture.device().pips()[pip.0].from().0.try_into()?;
    }
    let mut static_ps = 0_u64;
    let mut final_ps = 0_u64;
    let mut families = BTreeMap::<String, u64>::new();
    for &pip in &witness_pips {
        let model_pip = &architecture.device().pips()[pip.0];
        let class_name = architecture.pip_metadata(pip).timing_class;
        let timing = &speed.pip_classes[class_name];
        static_ps += timing.base.max_ps + timing.fanout_adder.max_ps;
        final_ps += timing.base.max_ps + timing.fanout_adder.max_ps * fanout[&model_pip.from()];
        let wire_name = &architecture.device().wires()[model_pip.to().0].name;
        let local = wire_name.rsplit('/').next().unwrap_or(wire_name);
        if local.len() >= 3 && matches!(local.as_bytes()[0], b'H' | b'V') {
            *families.entry(local[..3].to_owned()).or_default() += 1;
        }
        println!(
            "witness_pip\t{}\t{}\t{}\t{}\t{}\t{}\t{:?}\t{}\t{:?}",
            pip.0,
            class_name,
            timing.base.max_ps,
            timing.fanout_adder.max_ps,
            fanout[&model_pip.from()],
            architecture.device().wires()[model_pip.from().0].name,
            architecture.device().wires()[model_pip.from().0].point,
            architecture.device().wires()[model_pip.to().0].name,
            architecture.device().wires()[model_pip.to().0].point,
        );
    }
    println!(
        "witness net=1221 sink=lut10230.C pips={} static_fanout1_ps={} final_fanout_ps={} excess_ps={} families={:?}",
        witness_pips.len(),
        static_ps,
        final_ps,
        final_ps - static_ps,
        families,
    );
    Ok(())
}
