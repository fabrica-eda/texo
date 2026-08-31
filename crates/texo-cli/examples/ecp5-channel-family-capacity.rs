//! Read-only ECP5 general-channel capacity inventory for congestion diagnostics.

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs::File;
use std::io::BufReader;

use texo_model::WireId;
use texo_target_ecp5::read_architecture_cache;

const USAGE: &str = "Usage: ecp5-channel-family-capacity ARCH.txdb\n";

#[derive(Default)]
struct Capacity {
    global: u64,
    full: u64,
    tight: u64,
    wires: u64,
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let path = args.next().ok_or(USAGE)?;
    if args.next().is_some() {
        return Err(USAGE.into());
    }
    let architecture = read_architecture_cache(BufReader::new(File::open(path)?))?;
    let device = architecture.device();
    let mut usable = vec![false; device.wires().len()];
    for pip in device.pips() {
        usable[pip.from().0] = true;
        usable[pip.to().0] = true;
    }
    let mut families = BTreeMap::<String, Capacity>::new();
    for (index, wire) in device.wires().iter().enumerate() {
        if !usable[WireId(index).0] {
            continue;
        }
        let local = wire.name.rsplit('/').next().unwrap_or(&wire.name);
        let bytes = local.as_bytes();
        if bytes.len() < 3
            || !matches!(bytes[0], b'H' | b'V')
            || !bytes[1].is_ascii_digit()
            || !bytes[2].is_ascii_digit()
        {
            continue;
        }
        let family = local[..3].to_owned();
        let capacity = u64::from(wire.capacity);
        let entry = families.entry(family).or_default();
        entry.global += capacity;
        entry.wires += 1;
        if (59..=79).contains(&wire.point.x) && (32..=52).contains(&wire.point.y) {
            entry.full += capacity;
        }
        if (59..=79).contains(&wire.point.x) && (35..=45).contains(&wire.point.y) {
            entry.tight += capacity;
        }
    }
    println!("family\twires\tglobal_capacity\tfull_x59_79_y32_52\ttight_x59_79_y35_45");
    for (family, capacity) in families {
        println!(
            "{}\t{}\t{}\t{}\t{}",
            family, capacity.wires, capacity.global, capacity.full, capacity.tight
        );
    }
    println!("speed8_pip_class\tbase_max_ps\tfanout_adder_max_ps");
    for (name, timing) in &architecture.speed_grades()["8"].pip_classes {
        println!(
            "{}\t{}\t{}",
            name, timing.base.max_ps, timing.fanout_adder.max_ps
        );
    }
    Ok(())
}
