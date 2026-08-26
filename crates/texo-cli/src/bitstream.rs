//! Native checkpoint-to-Project-Trellis configuration generation.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::error::Error;
use std::fmt::{self, Write as _};

use serde_json::Value;
use texo_model::Point;
use texo_target_ecp5::Ecp5Architecture;

const REQUIRED_IMPLEMENTATION_EVIDENCE: [&str; 4] = [
    "synthesis_equivalence",
    "mapped_netlist_complete",
    "physical_implementation",
    "timing_closure",
];

/// Text configuration and route-accounting result ready for `ecppack`.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NativeEcp5Config {
    /// Project Trellis textual configuration.
    pub text: String,
    /// Number of programmable PIPs emitted as configuration arcs.
    pub programmable_pips: usize,
    /// Number of fixed routing edges intentionally omitted from configuration.
    pub fixed_edges: usize,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct TileConfig {
    arcs: Vec<(String, String)>,
    words: Vec<(String, Vec<bool>)>,
    enums: Vec<(String, String)>,
    unknowns: Vec<String>,
}

impl TileConfig {
    fn add_arc(&mut self, sink: impl Into<String>, source: impl Into<String>) {
        self.arcs.push((sink.into(), source.into()));
    }

    fn add_word(&mut self, name: impl Into<String>, value: Vec<bool>) {
        self.words.push((name.into(), value));
    }

    fn add_enum(&mut self, name: impl Into<String>, value: impl Into<String>) {
        self.enums.push((name.into(), value.into()));
    }

    fn write_to(&self, output: &mut String) {
        for (sink, source) in &self.arcs {
            writeln!(output, "arc: {sink} {source}").expect("writing to String cannot fail");
        }
        for (name, value) in &self.words {
            let bits = value
                .iter()
                .rev()
                .map(|bit| if *bit { '1' } else { '0' })
                .collect::<String>();
            writeln!(output, "word: {name} {bits}").expect("writing to String cannot fail");
        }
        for (name, value) in &self.enums {
            writeln!(output, "enum: {name} {value}").expect("writing to String cannot fail");
        }
        for unknown in &self.unknowns {
            writeln!(output, "unknown: {unknown}").expect("writing to String cannot fail");
        }
    }
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct ChipConfig {
    device: String,
    variant: Option<String>,
    comments: Vec<String>,
    sysconfig: BTreeMap<String, String>,
    tiles: BTreeMap<String, TileConfig>,
    tile_groups: Vec<(Vec<String>, TileConfig)>,
    bram_data: BTreeMap<u16, Vec<u16>>,
}

impl ChipConfig {
    #[allow(clippy::too_many_lines)]
    fn parse_base(text: &str) -> Result<Self, BitgenError> {
        let mut config = Self::default();
        let mut active_tile = None::<String>;
        for (line_number, raw) in text.lines().enumerate() {
            let line = raw.trim();
            if line.is_empty() {
                continue;
            }
            if let Some(directive) = line.strip_prefix('.') {
                active_tile = None;
                let (name, value) = directive.split_once(' ').unwrap_or((directive, ""));
                match name {
                    "device" => config.device = value.trim().into(),
                    "variant" => config.variant = Some(value.trim().into()),
                    "comment" => config.comments.push(value.into()),
                    "sysconfig" => {
                        let (key, value) = value.split_once(' ').ok_or_else(|| {
                            BitgenError::new(format!(
                                "base config line {} has malformed sysconfig",
                                line_number + 1
                            ))
                        })?;
                        config.sysconfig.insert(key.into(), value.trim().into());
                    }
                    "tile" => {
                        let tile = value.trim().to_owned();
                        config.tiles.entry(tile.clone()).or_default();
                        active_tile = Some(tile);
                    }
                    unsupported => {
                        return Err(BitgenError::new(format!(
                            "base config line {} uses unsupported directive .{unsupported}",
                            line_number + 1
                        )));
                    }
                }
                continue;
            }
            let tile = active_tile.as_ref().ok_or_else(|| {
                BitgenError::new(format!(
                    "base config line {} is outside a tile section",
                    line_number + 1
                ))
            })?;
            let (kind, body) = line.split_once(' ').ok_or_else(|| {
                BitgenError::new(format!("base config line {} is malformed", line_number + 1))
            })?;
            let tile = config.tiles.get_mut(tile).expect("active tile exists");
            match kind {
                "arc:" => {
                    let (sink, source) = body.split_once(' ').ok_or_else(|| {
                        BitgenError::new(format!(
                            "base config line {} has malformed arc",
                            line_number + 1
                        ))
                    })?;
                    tile.add_arc(sink, source.trim());
                }
                "word:" => {
                    let (name, bits) = body.split_once(' ').ok_or_else(|| {
                        BitgenError::new(format!(
                            "base config line {} has malformed word",
                            line_number + 1
                        ))
                    })?;
                    let value = bits
                        .trim()
                        .chars()
                        .rev()
                        .map(|bit| match bit {
                            '0' => Ok(false),
                            '1' => Ok(true),
                            _ => Err(BitgenError::new(format!(
                                "base config line {} has a non-binary word",
                                line_number + 1
                            ))),
                        })
                        .collect::<Result<Vec<_>, _>>()?;
                    tile.add_word(name, value);
                }
                "enum:" => {
                    let (name, value) = body.split_once(' ').ok_or_else(|| {
                        BitgenError::new(format!(
                            "base config line {} has malformed enum",
                            line_number + 1
                        ))
                    })?;
                    tile.add_enum(name, value.trim());
                }
                "unknown:" => tile.unknowns.push(body.trim().into()),
                unsupported => {
                    return Err(BitgenError::new(format!(
                        "base config line {} uses unsupported tile entry {unsupported}",
                        line_number + 1
                    )));
                }
            }
        }
        if config.device.is_empty() {
            return Err(BitgenError::new("base config has no .device"));
        }
        Ok(config)
    }

    fn tile_mut(&mut self, name: &str) -> &mut TileConfig {
        self.tiles.entry(name.into()).or_default()
    }

    fn serialize(&self) -> String {
        let mut output = format!(".device {}\n\n", self.device);
        if let Some(variant) = &self.variant {
            writeln!(output, ".variant {variant}\n").expect("writing to String cannot fail");
        }
        for comment in &self.comments {
            writeln!(output, ".comment {comment}").expect("writing to String cannot fail");
        }
        for (key, value) in &self.sysconfig {
            writeln!(output, ".sysconfig {key} {value}").expect("writing to String cannot fail");
        }
        output.push('\n');
        for (name, tile) in &self.tiles {
            if tile == &TileConfig::default() {
                continue;
            }
            writeln!(output, ".tile {name}").expect("writing to String cannot fail");
            tile.write_to(&mut output);
            output.push('\n');
        }
        for (wid, values) in &self.bram_data {
            writeln!(output, ".bram_init {wid}").expect("writing to String cannot fail");
            for chunk in values.chunks(8) {
                let line = chunk
                    .iter()
                    .map(|value| format!("{value:03x}"))
                    .collect::<Vec<_>>()
                    .join(" ");
                writeln!(output, "{line}").expect("writing to String cannot fail");
            }
            output.push('\n');
        }
        for (tiles, group) in &self.tile_groups {
            writeln!(output, ".tile_group {}", tiles.join(" "))
                .expect("writing to String cannot fail");
            group.write_to(&mut output);
            output.push('\n');
        }
        output
    }
}

/// Generates Project Trellis configuration text without importing `pytrellis`.
///
/// `base_config` is the decompressed empty-device configuration and `iodb` is
/// the selected device's Project Trellis `iodb.json` value.
///
/// # Errors
///
/// Returns an error when the checkpoint lacks implementation/timing evidence,
/// its target does not match the architecture/base configuration, or a
/// routed/configured resource cannot be represented by the native ECP5
/// configuration writer. RTL and post-map simulation evidence is optional.
pub fn generate_ecp5_config(
    checkpoint: &Value,
    architecture: &Ecp5Architecture,
    base_config: &str,
    iodb: &Value,
) -> Result<NativeEcp5Config, BitgenError> {
    validate_checkpoint(checkpoint)?;
    let target = member(checkpoint, "target")?;
    object(checkpoint, "target")?;
    let device = string(target, "device")?;
    if architecture.device().name() != device {
        return Err(BitgenError::new(format!(
            "checkpoint device {device} does not match architecture {}",
            architecture.device().name()
        )));
    }
    let mut config = ChipConfig::parse_base(base_config)?;
    if config.device != device {
        return Err(BitgenError::new(format!(
            "base config device {} does not match checkpoint {device}",
            config.device
        )));
    }
    config
        .comments
        .push(format!("Part: {device}-{}", string(target, "package")?));

    let (programmable, fixed, incoming_flags, routed_wires) = add_routes(&mut config, checkpoint)?;
    let metadata = keyed_objects(array(checkpoint, "primitive_metadata")?, "cell_id")?;
    let absorbed = keyed_objects(array(checkpoint, "absorbed_inputs")?, "cell_id")?;
    let packing = member(checkpoint, "packing")?;
    object(checkpoint, "packing")?;
    let attributes = keyed_objects(array(packing, "io_attributes")?, "cell_id")?;
    let packed_rams = keyed_objects(array(packing, "block_rams")?, "cell")?;
    let dedicated_ffs = array(packing, "lut_ff_pairs")?
        .iter()
        .map(|pair| usize_value(pair, "ff"))
        .collect::<Result<BTreeSet<_>, _>>()?;

    for placement in array(checkpoint, "placement")? {
        if string(placement, "kind")? == "global_clock" {
            continue;
        }
        let cell_id = usize_value(placement, "cell_id")?;
        let metadata = metadata.get(&cell_id).ok_or_else(|| {
            BitgenError::new(format!("cell {cell_id} has no primitive configuration"))
        })?;
        let configuration = member(metadata, "configuration")?;
        object(metadata, "configuration")?;
        let absorbed_inputs = absorbed
            .get(&cell_id)
            .map(|record| object(record, "pins"))
            .transpose()?
            .cloned()
            .unwrap_or_default();
        match string(configuration, "kind")? {
            "lut4" | "carry_slice" | "constant" => write_comb(
                &mut config,
                placement,
                configuration,
                &incoming_flags,
                &absorbed_inputs,
            )?,
            "flip_flop" => write_ff(
                &mut config,
                placement,
                configuration,
                &routed_wires,
                dedicated_ffs.contains(&cell_id),
            )?,
            "port" => write_io(
                &mut config,
                architecture,
                iodb,
                placement,
                configuration,
                attributes.get(&cell_id).copied(),
            )?,
            "block_ram" => {
                let packed = packed_rams.get(&cell_id).ok_or_else(|| {
                    BitgenError::new(format!("DP16KD cell {cell_id} is absent from packing"))
                })?;
                write_bram(
                    &mut config,
                    architecture,
                    placement,
                    configuration,
                    packed,
                    &absorbed_inputs,
                )?;
            }
            kind => return Err(BitgenError::new(format!("unsupported primitive {kind}"))),
        }
    }

    Ok(NativeEcp5Config {
        text: config.serialize(),
        programmable_pips: programmable,
        fixed_edges: fixed,
    })
}

fn validate_checkpoint(checkpoint: &Value) -> Result<(), BitgenError> {
    if u64_value(checkpoint, "schema_version")? != 3 {
        return Err(BitgenError::new(
            "native bitgen requires checkpoint schema version 3",
        ));
    }
    let evidence = array(checkpoint, "evidence")?
        .iter()
        .filter_map(Value::as_str)
        .collect::<BTreeSet<_>>();
    let missing = REQUIRED_IMPLEMENTATION_EVIDENCE
        .iter()
        .filter(|gate| !evidence.contains(**gate))
        .copied()
        .collect::<Vec<_>>();
    if !missing.is_empty() {
        return Err(BitgenError::new(format!(
            "bitstream generation is missing implementation evidence: {}",
            missing.join(", ")
        )));
    }
    if !bool_value(member(checkpoint, "timing")?, "met_timing")? {
        return Err(BitgenError::new(
            "bitstream generation requires a timing-closed checkpoint",
        ));
    }
    if string(member(checkpoint, "target")?, "family")? != "ECP5" {
        return Err(BitgenError::new(
            "native bitgen only accepts ECP5 checkpoints",
        ));
    }
    Ok(())
}

type IncomingFlags = HashMap<u64, Vec<u64>>;
type IoConfigurationTiles = (String, String, Option<(String, &'static str)>);

fn add_routes(
    config: &mut ChipConfig,
    checkpoint: &Value,
) -> Result<(usize, usize, IncomingFlags, BTreeSet<String>), BitgenError> {
    let mut programmable = 0;
    let mut fixed = 0;
    let mut incoming_flags = IncomingFlags::new();
    let mut routed_wires = BTreeSet::new();
    for route in array(checkpoint, "routes")? {
        for wire in array(route, "wires")? {
            routed_wires.insert(string(wire, "wire")?.into());
        }
        for pip in array(route, "pips")? {
            incoming_flags
                .entry(u64_value(pip, "to_wire_id")?)
                .or_default()
                .push(u64_value(pip, "lutperm_flags")?);
            if bool_value(pip, "fixed")? {
                fixed += 1;
                continue;
            }
            let tile = string(pip, "config_tile")?;
            let owner = tile_point(tile)?;
            let source = trellis_wire_name(owner, string(pip, "from")?)?;
            let sink = trellis_wire_name(owner, string(pip, "to")?)?;
            config.tile_mut(tile).add_arc(sink, source);
            programmable += 1;
        }
    }
    Ok((programmable, fixed, incoming_flags, routed_wires))
}

fn logic_tile(placement: &Value) -> Result<&str, BitgenError> {
    let matches = array(placement, "configuration_tiles")?
        .iter()
        .filter(|tile| string(tile, "tile_type") == Ok("PLC2"))
        .map(|tile| string(tile, "name"))
        .collect::<Result<Vec<_>, _>>()?;
    if matches.len() != 1 {
        return Err(BitgenError::new(format!(
            "{} has {} PLC2 configuration tiles",
            string(placement, "bel")?,
            matches.len()
        )));
    }
    Ok(matches[0])
}

fn slice_and_lc(placement: &Value) -> Result<(String, usize), BitgenError> {
    let z = usize_value(placement, "bel_z")? >> 2;
    let slice = *b"ABCD"
        .get(z / 2)
        .ok_or_else(|| BitgenError::new("logic BEL z is outside SLICEA-D"))?
        as char;
    Ok((format!("SLICE{slice}"), z % 2))
}

fn write_comb(
    config: &mut ChipConfig,
    placement: &Value,
    configuration: &Value,
    incoming_flags: &IncomingFlags,
    absorbed_inputs: &serde_json::Map<String, Value>,
) -> Result<(), BitgenError> {
    let tile = logic_tile(placement)?.to_owned();
    let (slice, lc) = slice_and_lc(placement)?;
    let kind = string(configuration, "kind")?;
    let mode = if kind == "carry_slice" {
        "CCU2"
    } else {
        "LOGIC"
    };
    let (init, used) = permute_lut(configuration, placement, incoming_flags, absorbed_inputs)?;
    let target = config.tile_mut(&tile);
    target.add_enum(format!("{slice}.MODE"), mode);
    target.add_word(
        format!("{slice}.K{lc}.INIT"),
        integer_bits(u64::from(init), 16),
    );
    let inject = if optional_bool(configuration, "inject")?.unwrap_or(false) {
        "YES"
    } else {
        "NO"
    };
    target.add_enum(
        format!("{slice}.CCU2.INJECT1_{lc}"),
        if mode == "CCU2" { inject } else { "_NONE_" },
    );
    for (physical, pin) in "ABCD".chars().enumerate() {
        if !used.contains(&physical) {
            target.add_enum(format!("{slice}.{pin}{lc}MUX"), "1");
        }
    }
    Ok(())
}

fn permute_lut(
    configuration: &Value,
    placement: &Value,
    incoming_flags: &IncomingFlags,
    absorbed_inputs: &serde_json::Map<String, Value>,
) -> Result<(u16, BTreeSet<usize>), BitgenError> {
    let kind = string(configuration, "kind")?;
    let mut original = optional_u64(configuration, "init")?.map_or_else(
        || {
            optional_bool(configuration, "value")
                .map(|value| if value.unwrap_or(false) { 0xffff } else { 0 })
        },
        |value| u16::try_from(value).map_err(|_| BitgenError::new("LUT INIT exceeds 16 bits")),
    )?;
    if kind == "carry_slice" {
        for (input, pin) in "ABCD".chars().enumerate() {
            if absorbed_inputs
                .get(&pin.to_string())
                .and_then(Value::as_bool)
                == Some(false)
            {
                original = fold_lut_input(original, input, false);
            }
        }
    }
    let pin_wires = array(placement, "bel_pins")?
        .iter()
        .filter_map(|pin| {
            let name = string(pin, "name").ok()?;
            "ABCD"
                .contains(name)
                .then(|| Ok((name.to_owned(), u64_value(pin, "wire_id")?)))
        })
        .collect::<Result<BTreeMap<_, _>, BitgenError>>()?;
    let mut physical_to_logical = vec![Vec::<usize>::new(); 4];
    for (physical, name) in "ABCD".chars().enumerate() {
        let wire = pin_wires.get(&name.to_string()).ok_or_else(|| {
            BitgenError::new(format!(
                "{} lacks LUT pin {name}",
                string(placement, "bel").unwrap_or("BEL")
            ))
        })?;
        for &flags in incoming_flags.get(wire).into_iter().flatten() {
            if flags & 0x4000 != 0 {
                let logical = (flags & 0x3) as usize;
                let destination = ((flags >> 2) & 0x3) as usize;
                if destination != physical {
                    return Err(BitgenError::new(format!(
                        "invalid LUT permutation flags 0x{flags:04x}"
                    )));
                }
                // Project Trellis LUT permutation flags name the logical
                // source first and physical destination second. Preserve the
                // same inverse mapping used by the architecture importer.
                physical_to_logical[logical].push(physical);
            } else {
                physical_to_logical[physical].push(physical);
            }
        }
    }
    if kind == "carry_slice" {
        for (physical, logicals) in physical_to_logical.iter_mut().enumerate() {
            if !logicals.is_empty() {
                continue;
            }
            for logical in 2 * (physical / 2)..2 * ((physical / 2) + 1) {
                let wire = pin_wires[&"ABCD".chars().nth(logical).expect("0..4").to_string()];
                if !incoming_flags.contains_key(&wire) {
                    logicals.push(logical);
                }
            }
        }
    }
    let mut permuted = 0_u16;
    for physical_value in 0..16_u16 {
        let mut logical_value = 0_u16;
        for (physical, logicals) in physical_to_logical.iter().enumerate() {
            if physical_value & (1 << physical) != 0 {
                for logical in logicals {
                    logical_value |= 1 << logical;
                }
            }
        }
        if original & (1 << logical_value) != 0 {
            permuted |= 1 << physical_value;
        }
    }
    let used = physical_to_logical
        .iter()
        .enumerate()
        .filter_map(|(index, mappings)| (!mappings.is_empty()).then_some(index))
        .collect();
    Ok((permuted, used))
}

fn write_ff(
    config: &mut ChipConfig,
    placement: &Value,
    configuration: &Value,
    routed_wires: &BTreeSet<String>,
    dedicated: bool,
) -> Result<(), BitgenError> {
    let tile = logic_tile(placement)?.to_owned();
    let (slice, lc) = slice_and_lc(placement)?;
    let reset = configuration.get("reset").filter(|value| !value.is_null());
    let enable = configuration
        .get("enable")
        .and_then(|value| (!value.is_null()).then_some(value))
        .and_then(Value::as_str);
    let target = config.tile_mut(&tile);
    target.add_enum(format!("{slice}.GSR"), "DISABLED");
    target.add_enum(
        format!("{slice}.REG{lc}.SD"),
        if dedicated { "1" } else { "0" },
    );
    target.add_enum(
        format!("{slice}.REG{lc}.REGSET"),
        if reset
            .and_then(|value| optional_bool(value, "value").ok().flatten())
            .unwrap_or(false)
        {
            "SET"
        } else {
            "RESET"
        },
    );
    target.add_enum(format!("{slice}.REG{lc}.LSRMODE"), "LSR");
    target.add_enum(
        format!("{slice}.CEMUX"),
        match enable {
            None => "1",
            Some("high") => "CE",
            Some("low") => "INV",
            Some(other) => {
                return Err(BitgenError::new(format!(
                    "unsupported flip-flop enable level {other}"
                )));
            }
        },
    );
    let point = (usize_value(placement, "x")?, usize_value(placement, "y")?);
    if let Some(reset) = reset {
        let srmode = if bool_value(reset, "asynchronous")? {
            "ASYNC"
        } else {
            "LSR_OVER_CE"
        };
        let lsrmux = if string(reset, "active")? == "high" {
            "LSR"
        } else {
            "INV"
        };
        for index in 0..2 {
            if route_uses_local_wire(routed_wires, point, &format!("LSR{index}")) {
                config
                    .tile_mut(&tile)
                    .add_enum(format!("LSR{index}.SRMODE"), srmode);
                config
                    .tile_mut(&tile)
                    .add_enum(format!("LSR{index}.LSRMUX"), lsrmux);
            }
        }
    }
    let clkmux = if string(configuration, "edge")? == "rising" {
        "CLK"
    } else {
        "INV"
    };
    for index in 0..2 {
        if route_uses_local_wire(routed_wires, point, &format!("CLK{index}")) {
            config
                .tile_mut(&tile)
                .add_enum(format!("CLK{index}.CLKMUX"), clkmux);
        }
    }
    Ok(())
}

fn write_io(
    config: &mut ChipConfig,
    architecture: &Ecp5Architecture,
    iodb: &Value,
    placement: &Value,
    configuration: &Value,
    attributes: Option<&Value>,
) -> Result<(), BitgenError> {
    let (base_tile, companion_tile, tristate) = io_tiles(architecture, placement)?;
    let pio = string(placement, "bel")?
        .rsplit('/')
        .next()
        .ok_or_else(|| BitgenError::new("PIO BEL has no basename"))?;
    let direction = io_base_direction(configuration)?;
    let attributes = attributes
        .map(|value| object(value, "attributes"))
        .transpose()?
        .cloned()
        .unwrap_or_default();
    let io_type = attributes
        .get("IO_TYPE")
        .and_then(Value::as_str)
        .unwrap_or("LVCMOS33");
    config
        .tile_mut(&base_tile)
        .add_enum(format!("{pio}.BASE_TYPE"), format!("{direction}_{io_type}"));
    config
        .tile_mut(&companion_tile)
        .add_enum(format!("{pio}.BASE_TYPE"), format!("{direction}_{io_type}"));
    if matches!(direction, "INPUT" | "BIDIR") {
        config.tile_mut(&base_tile).add_enum(
            format!("{pio}.HYSTERESIS"),
            attributes
                .get("HYSTERESIS")
                .and_then(Value::as_str)
                .unwrap_or("ON"),
        );
    } else if let Some((tile, wire)) = tristate {
        config
            .tile_mut(&tile)
            .add_enum(format!("CIB.{wire}MUX"), "0");
    }
    for (attribute, default) in [
        ("SLEWRATE", "SLOW"),
        ("PULLMODE", "NONE"),
        ("DIFFRESISTOR", "OFF"),
        ("CLAMP", "OFF"),
        ("DRIVE", "8"),
        ("OPENDRAIN", "OFF"),
    ] {
        if let Some(value) = attributes.get(attribute).and_then(Value::as_str) {
            config.tile_mut(&base_tile).add_enum(
                format!("{pio}.{attribute}"),
                value_or_default(value, default),
            );
        }
    }
    if matches!(direction, "OUTPUT" | "BIDIR") {
        let bank = io_bank(iodb, placement)?;
        let bank_type = format!("BANKREF{bank}");
        let bank_tile = unique_tile_of_type(architecture, &bank_type)?;
        config
            .tile_mut(&bank_tile)
            .add_enum("BANK.VCCIO", io_voltage(io_type)?);
    }
    Ok(())
}

fn io_base_direction(configuration: &Value) -> Result<&'static str, BitgenError> {
    Ok(match string(configuration, "direction")? {
        "input" => "INPUT",
        "output" => "OUTPUT",
        "inout" => "BIDIR",
        direction => {
            return Err(BitgenError::new(format!(
                "unsupported IO direction {direction}"
            )));
        }
    })
}

fn value_or_default<'a>(value: &'a str, _default: &'a str) -> &'a str {
    value
}

fn io_tiles(
    architecture: &Ecp5Architecture,
    placement: &Value,
) -> Result<IoConfigurationTiles, BitgenError> {
    let x = usize_value(placement, "x")?;
    let y = usize_value(placement, "y")?;
    let pio = string(placement, "bel")?
        .rsplit('/')
        .next()
        .ok_or_else(|| BitgenError::new("PIO BEL has no basename"))?;
    let max_x = architecture.device().width() as usize - 1;
    let max_y = architecture.device().height() as usize - 1;
    if y == 0 {
        let offset = usize::from(pio != "PIOA");
        return Ok((
            chip_tile(architecture, x + offset, 0, &[&format!("PIOT{offset}")])?,
            chip_tile(architecture, x + offset, 1, &[&format!("PICT{offset}")])?,
            Some((chip_tile(architecture, x + offset, 1, &["CIB"])?, "JB0")),
        ));
    }
    if y == max_y {
        let offset = usize::from(pio != "PIOA");
        let accepted = if offset == 0 {
            ["PICB0", "EFB0_PICB0", "EFB2_PICB0", "SPICB0"].as_slice()
        } else {
            ["PICB1", "EFB1_PICB1", "EFB3_PICB1"].as_slice()
        };
        let tile = chip_tile(architecture, x + offset, y, accepted)?;
        return Ok((tile.clone(), tile, None));
    }
    if x == 0 {
        let (base_y, companion_y, companion_types): (_, _, &[&str]) =
            if matches!(pio, "PIOA" | "PIOB") {
                (y + 1, y, &["PICL0", "PICL0_DQS2"])
            } else {
                (y + 1, y + 2, &["PICL2", "PICL2_DQS1", "MIB_CIB_LR"])
            };
        return Ok((
            chip_tile(
                architecture,
                0,
                base_y,
                &["PICL1", "PICL1_DQS0", "PICL1_DQS3"],
            )?,
            chip_tile(architecture, 0, companion_y, companion_types)?,
            None,
        ));
    }
    if x == max_x {
        let (base_y, companion_y, companion_types): (_, _, &[&str]) =
            if matches!(pio, "PIOA" | "PIOB") {
                (y + 1, y, &["PICR0", "PICR0_DQS2"])
            } else {
                (y + 1, y + 2, &["PICR2", "PICR2_DQS1", "MIB_CIB_LR_A"])
            };
        return Ok((
            chip_tile(
                architecture,
                x,
                base_y,
                &["PICR1", "PICR1_DQS0", "PICR1_DQS3"],
            )?,
            chip_tile(architecture, x, companion_y, companion_types)?,
            None,
        ));
    }
    Err(BitgenError::new(format!(
        "PIO is not on the device edge: {}",
        string(placement, "bel")?
    )))
}

fn chip_tile(
    architecture: &Ecp5Architecture,
    x: usize,
    y: usize,
    accepted: &[&str],
) -> Result<String, BitgenError> {
    let matches = architecture
        .configuration_tiles(Point::new(
            u32::try_from(x).map_err(|_| BitgenError::new("tile x exceeds u32"))?,
            u32::try_from(y).map_err(|_| BitgenError::new("tile y exceeds u32"))?,
        ))
        .filter(|(_, tile_type)| accepted.contains(tile_type))
        .map(|(name, _)| name.to_owned())
        .collect::<Vec<_>>();
    if matches.len() != 1 {
        return Err(BitgenError::new(format!(
            "R{y}C{x} has {} tiles of type {}",
            matches.len(),
            accepted.join("/")
        )));
    }
    Ok(matches[0].clone())
}

fn unique_tile_of_type(
    architecture: &Ecp5Architecture,
    accepted: &str,
) -> Result<String, BitgenError> {
    let mut matches = Vec::new();
    for y in 0..architecture.device().height() {
        for x in 0..architecture.device().width() {
            matches.extend(
                architecture
                    .configuration_tiles(Point::new(x, y))
                    .filter(|(_, tile_type)| *tile_type == accepted)
                    .map(|(name, _)| name.to_owned()),
            );
        }
    }
    matches.sort();
    matches.dedup();
    if matches.len() != 1 {
        return Err(BitgenError::new(format!(
            "device has {} {accepted} tiles",
            matches.len()
        )));
    }
    Ok(matches.remove(0))
}

fn io_bank(iodb: &Value, placement: &Value) -> Result<u64, BitgenError> {
    let pio = string(placement, "bel")?
        .rsplit('/')
        .next()
        .and_then(|name| name.strip_prefix("PIO"))
        .ok_or_else(|| BitgenError::new("invalid PIO BEL name"))?;
    let x = u64_value(placement, "x")?;
    let y = u64_value(placement, "y")?;
    for record in array(iodb, "pio_metadata")? {
        if u64_value(record, "col")? == x
            && u64_value(record, "row")? == y
            && string(record, "pio")? == pio
        {
            return u64_value(record, "bank");
        }
    }
    Err(BitgenError::new(format!(
        "IO bank metadata is absent for {}",
        string(placement, "bel")?
    )))
}

fn io_voltage(io_type: &str) -> Result<&'static str, BitgenError> {
    match io_type {
        "LVCMOS33" => Ok("3V3"),
        "LVCMOS25" => Ok("2V5"),
        "LVCMOS18" => Ok("1V8"),
        "LVCMOS15" => Ok("1V5"),
        "LVCMOS12" => Ok("1V2"),
        _ => Err(BitgenError::new(format!(
            "native bitgen does not classify IO_TYPE={io_type}"
        ))),
    }
}

#[allow(clippy::too_many_lines)]
fn write_bram(
    config: &mut ChipConfig,
    architecture: &Ecp5Architecture,
    placement: &Value,
    configuration: &Value,
    packed: &Value,
    absorbed_inputs: &serde_json::Map<String, Value>,
) -> Result<(), BitgenError> {
    let width = u64_value(configuration, "physical_width")?;
    if ![1, 2, 4, 9, 18].contains(&width) {
        return Err(BitgenError::new(format!(
            "DP16KD has unsupported physical width {width}"
        )));
    }
    for field in ["depth", "word_width", "physical_width"] {
        if u64_value(configuration, field)? != u64_value(packed, field)? {
            return Err(BitgenError::new(format!(
                "DP16KD packing disagrees on {field}: {}",
                string(placement, "cell")?
            )));
        }
    }
    let z = usize_value(placement, "bel_z")?;
    let ebr = format!("EBR{z}");
    let mut group = TileConfig::default();
    for (name, value) in [
        (format!("{ebr}.MODE"), "DP16KD".into()),
        (format!("{ebr}.DP16KD.DATA_WIDTH_A"), width.to_string()),
        (format!("{ebr}.DP16KD.DATA_WIDTH_B"), width.to_string()),
        (format!("{ebr}.DP16KD.WRITEMODE_A"), "NORMAL".into()),
        (format!("{ebr}.DP16KD.WRITEMODE_B"), "NORMAL".into()),
        (format!("{ebr}.REGMODE_A"), "NOREG".into()),
        (format!("{ebr}.REGMODE_B"), "NOREG".into()),
        (format!("{ebr}.RESETMODE"), "SYNC".into()),
        (format!("{ebr}.ASYNC_RESET_RELEASE"), "SYNC".into()),
        (format!("{ebr}.GSR"), "DISABLED".into()),
    ] {
        group.add_enum(name, value);
    }
    let wid = u64_value(packed, "wid")?;
    group.add_word(format!("{ebr}.WID"), integer_bits(reverse_bits(wid, 9), 9));

    for pin in array(placement, "bel_pins")? {
        let name = string(pin, "name")?;
        let Some(value) = absorbed_inputs.get(name).and_then(Value::as_bool) else {
            continue;
        };
        let tie = member(pin, "cib_tie")?;
        object(pin, "cib_tie")?;
        let forced_high = matches!(
            name,
            "CLKA"
                | "CLKB"
                | "WEA"
                | "WEB"
                | "RSTA"
                | "RSTB"
                | "CEA"
                | "CEB"
                | "OCEA"
                | "OCEB"
                | "CSA0"
                | "CSA1"
                | "CSA2"
                | "CSB0"
                | "CSB1"
                | "CSB2"
        );
        add_cib_tie(config, tie, forced_high || value)?;
    }

    let rising = string(configuration, "edge")? == "rising";
    for (name, value) in [
        (
            format!("{ebr}.CLKAMUX"),
            if rising { "CLKA" } else { "INV" },
        ),
        (
            format!("{ebr}.CLKBMUX"),
            if rising { "CLKB" } else { "INV" },
        ),
        (format!("{ebr}.RSTAMUX"), "INV"),
        (format!("{ebr}.RSTBMUX"), "INV"),
        (
            format!("{ebr}.WEAMUX"),
            if string(configuration, "write_enable")? == "high" {
                "WEA"
            } else {
                "INV"
            },
        ),
        (format!("{ebr}.WEBMUX"), "INV"),
        (format!("{ebr}.CEAMUX"), "CEA"),
        (
            format!("{ebr}.CEBMUX"),
            if configuration
                .get("read_enable")
                .and_then(Value::as_str)
                .is_none_or(|level| level == "high")
            {
                "CEB"
            } else {
                "INV"
            },
        ),
        (format!("{ebr}.OCEAMUX"), "OCEA"),
        (format!("{ebr}.OCEBMUX"), "OCEB"),
    ] {
        group.add_enum(name, value);
    }
    for port in ['A', 'B'] {
        let mut decode = vec![false; 3];
        for (bit, value) in decode.iter_mut().enumerate() {
            let name = format!("CS{port}{bit}");
            if absorbed_inputs.get(&name).and_then(Value::as_bool) == Some(false) {
                *value = !*value;
            }
        }
        decode.reverse();
        group.add_word(format!("{ebr}.CSDECODE_{port}"), decode);
    }
    let wid = u16::try_from(wid).map_err(|_| BitgenError::new("DP16KD WID exceeds u16"))?;
    if config.bram_data.insert(wid, vec![0; 2048]).is_some() {
        return Err(BitgenError::new(format!("duplicate DP16KD WID {wid}")));
    }
    config
        .tile_groups
        .push((bram_tiles(architecture, placement)?, group));
    Ok(())
}

fn bram_tiles(
    architecture: &Ecp5Architecture,
    placement: &Value,
) -> Result<Vec<String>, BitgenError> {
    const EBR0: &[&str] = &["MIB_EBR0", "EBR_CMUX_UR", "EBR_CMUX_LR", "EBR_CMUX_LR_25K"];
    const EBR8: &[&str] = &[
        "MIB_EBR8",
        "EBR_SPINE_UL1",
        "EBR_SPINE_UR1",
        "EBR_SPINE_LL1",
        "EBR_CMUX_UL",
        "EBR_SPINE_LL0",
        "EBR_CMUX_LL",
        "EBR_SPINE_LR0",
        "EBR_SPINE_LR1",
        "EBR_CMUX_LL_25K",
        "EBR_SPINE_UL2",
        "EBR_SPINE_UL0",
        "EBR_SPINE_UR2",
        "EBR_SPINE_LL2",
        "EBR_SPINE_LR2",
        "EBR_SPINE_UR0",
    ];
    let x = usize_value(placement, "x")?;
    let y = usize_value(placement, "y")?;
    let z = usize_value(placement, "bel_z")?;
    let groups: &[(usize, &[&str])] = match z {
        0 => &[(0, EBR0), (1, &["MIB_EBR1"])],
        1 => &[(0, &["MIB_EBR2"]), (1, &["MIB_EBR3"]), (2, &["MIB_EBR4"])],
        2 => &[(0, &["MIB_EBR4"]), (1, &["MIB_EBR5"]), (2, &["MIB_EBR6"])],
        3 => &[(0, &["MIB_EBR6"]), (1, &["MIB_EBR7"]), (2, EBR8)],
        _ => {
            return Err(BitgenError::new(format!(
                "DP16KD BEL has invalid z={z}: {}",
                string(placement, "bel")?
            )));
        }
    };
    groups
        .iter()
        .map(|(offset, accepted)| chip_tile(architecture, x + offset, y, accepted))
        .collect()
}

fn add_cib_tie(config: &mut ChipConfig, tie: &Value, value: bool) -> Result<(), BitgenError> {
    let mux = string(tie, "mux")?;
    if mux.starts_with("JCE") && !value {
        return Err(BitgenError::new(format!(
            "CIB {mux} can only provide a high constant"
        )));
    }
    let output = if mux.starts_with("JCLK") || mux.starts_with("JLSR") {
        if !value {
            return Err(BitgenError::new(format!(
                "CIB {mux} requires its inverted-zero encoding"
            )));
        }
        false
    } else {
        value
    };
    let config_tile = string(tie, "tile")?;
    config
        .tile_mut(config_tile)
        .add_enum(format!("CIB.{mux}MUX"), if output { "1" } else { "0" });
    Ok(())
}

fn route_uses_local_wire(
    routed_wires: &BTreeSet<String>,
    point: (usize, usize),
    basename: &str,
) -> bool {
    routed_wires.contains(&format!("R{}C{}/{basename}", point.1, point.0))
}

fn tile_point(name: &str) -> Result<(usize, usize), BitgenError> {
    let rest = name
        .split_once('R')
        .map(|(_, rest)| rest)
        .ok_or_else(|| BitgenError::new(format!("tile has no row coordinate: {name}")))?;
    let (row, rest) = rest
        .split_once('C')
        .ok_or_else(|| BitgenError::new(format!("tile has no column coordinate: {name}")))?;
    let column = rest
        .split_once(':')
        .map_or(rest, |(column, _)| column)
        .parse::<usize>()
        .map_err(|_| BitgenError::new(format!("invalid tile column: {name}")))?;
    let row = row
        .parse::<usize>()
        .map_err(|_| BitgenError::new(format!("invalid tile row: {name}")))?;
    Ok((column, row))
}

fn trellis_wire_name(owner: (usize, usize), qualified: &str) -> Result<String, BitgenError> {
    let qualified = qualified
        .strip_prefix('R')
        .ok_or_else(|| BitgenError::new(format!("invalid qualified wire: {qualified}")))?;
    let (row, rest) = qualified
        .split_once('C')
        .ok_or_else(|| BitgenError::new(format!("invalid qualified wire: R{qualified}")))?;
    let (column, basename) = rest
        .split_once('/')
        .ok_or_else(|| BitgenError::new(format!("invalid qualified wire: R{qualified}")))?;
    let wire_y = row
        .parse::<usize>()
        .map_err(|_| BitgenError::new("invalid wire row"))?;
    let wire_x = column
        .parse::<usize>()
        .map_err(|_| BitgenError::new("invalid wire column"))?;
    if basename.starts_with("G_")
        || basename.starts_with("L_")
        || basename.starts_with("R_")
        || (wire_x, wire_y) == owner
    {
        return Ok(basename.into());
    }
    let mut prefix = String::new();
    if wire_y < owner.1 {
        write!(prefix, "N{}", owner.1 - wire_y).expect("writing to String cannot fail");
    } else if wire_y > owner.1 {
        write!(prefix, "S{}", wire_y - owner.1).expect("writing to String cannot fail");
    }
    if wire_x > owner.0 {
        write!(prefix, "E{}", wire_x - owner.0).expect("writing to String cannot fail");
    } else if wire_x < owner.0 {
        write!(prefix, "W{}", owner.0 - wire_x).expect("writing to String cannot fail");
    }
    Ok(format!("{prefix}_{basename}"))
}

fn integer_bits(value: u64, width: usize) -> Vec<bool> {
    (0..width).map(|bit| value & (1 << bit) != 0).collect()
}

fn reverse_bits(value: u64, width: usize) -> u64 {
    (0..width).fold(0, |reversed, bit| {
        if value & (1 << bit) != 0 {
            reversed | (1 << (width - bit - 1))
        } else {
            reversed
        }
    })
}

fn fold_lut_input(initialization: u16, input: usize, value: bool) -> u16 {
    let mut folded = 0;
    for output in 0..16 {
        let source = (output & !(1 << input)) | (usize::from(value) << input);
        if initialization & (1 << source) != 0 {
            folded |= 1 << output;
        }
    }
    folded
}

fn keyed_objects<'a>(
    values: &'a [Value],
    key: &str,
) -> Result<BTreeMap<usize, &'a Value>, BitgenError> {
    values
        .iter()
        .map(|value| Ok((usize_value(value, key)?, value)))
        .collect()
}

fn member<'a>(value: &'a Value, key: &str) -> Result<&'a Value, BitgenError> {
    value
        .get(key)
        .ok_or_else(|| BitgenError::new(format!("{key} is absent")))
}

fn object<'a>(
    value: &'a Value,
    key: &str,
) -> Result<&'a serde_json::Map<String, Value>, BitgenError> {
    value
        .get(key)
        .and_then(Value::as_object)
        .ok_or_else(|| BitgenError::new(format!("{key} is absent or is not an object")))
}

fn array<'a>(value: &'a Value, key: &str) -> Result<&'a [Value], BitgenError> {
    value
        .get(key)
        .and_then(Value::as_array)
        .map(Vec::as_slice)
        .ok_or_else(|| BitgenError::new(format!("{key} is absent or is not an array")))
}

fn string<'a>(value: &'a Value, key: &str) -> Result<&'a str, BitgenError> {
    value
        .get(key)
        .and_then(Value::as_str)
        .ok_or_else(|| BitgenError::new(format!("{key} is absent or is not a string")))
}

fn u64_value(value: &Value, key: &str) -> Result<u64, BitgenError> {
    value
        .get(key)
        .and_then(Value::as_u64)
        .ok_or_else(|| BitgenError::new(format!("{key} is absent or is not an unsigned integer")))
}

fn usize_value(value: &Value, key: &str) -> Result<usize, BitgenError> {
    usize::try_from(u64_value(value, key)?)
        .map_err(|_| BitgenError::new(format!("{key} exceeds usize")))
}

fn bool_value(value: &Value, key: &str) -> Result<bool, BitgenError> {
    value
        .get(key)
        .and_then(Value::as_bool)
        .ok_or_else(|| BitgenError::new(format!("{key} is absent or is not a boolean")))
}

fn optional_bool(value: &Value, key: &str) -> Result<Option<bool>, BitgenError> {
    match value.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(value) => value
            .as_bool()
            .map(Some)
            .ok_or_else(|| BitgenError::new(format!("{key} is not a boolean"))),
    }
}

fn optional_u64(value: &Value, key: &str) -> Result<Option<u64>, BitgenError> {
    match value.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(value) => value
            .as_u64()
            .map(Some)
            .ok_or_else(|| BitgenError::new(format!("{key} is not an unsigned integer"))),
    }
}

/// Invalid checkpoint or unsupported ECP5 configuration request.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BitgenError {
    message: String,
}

impl BitgenError {
    fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }
}

impl fmt::Display for BitgenError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.message)
    }
}

impl Error for BitgenError {}

#[cfg(test)]
mod tests {
    use serde_json::json;
    use texo_target_ecp5::{ArchitectureFile, TileRecord, expand};

    use super::{
        ChipConfig, TileConfig, fold_lut_input, io_base_direction, reverse_bits, trellis_wire_name,
        validate_checkpoint, write_bram,
    };

    const ARCHITECTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../texo-target-ecp5/fixtures/minimal-ecp5.json"
    ));

    #[test]
    fn parses_and_canonically_serializes_an_empty_device_config() {
        let parsed = ChipConfig::parse_base(
            ".device TEST\n\n.tile R0C0:CIB\nunknown: F2B0\nenum: CIB.JA0MUX 0\n",
        )
        .unwrap();
        assert_eq!(
            parsed.serialize(),
            ".device TEST\n\n\n.tile R0C0:CIB\nenum: CIB.JA0MUX 0\nunknown: F2B0\n\n"
        );
    }

    #[test]
    fn bit_helpers_match_trellis_ordering() {
        assert_eq!(reverse_bits(0b000_000_011, 9), 0b110_000_000);
        assert_eq!(fold_lut_input(0xaaaa, 0, false), 0x0000);
        assert_eq!(fold_lut_input(0xaaaa, 0, true), 0xffff);
    }

    #[test]
    fn maps_bidirectional_ports_to_the_ecp5_bidir_base_type() {
        assert_eq!(
            io_base_direction(&json!({"direction": "input"})).unwrap(),
            "INPUT"
        );
        assert_eq!(
            io_base_direction(&json!({"direction": "output"})).unwrap(),
            "OUTPUT"
        );
        assert_eq!(
            io_base_direction(&json!({"direction": "inout"})).unwrap(),
            "BIDIR"
        );
        assert!(io_base_direction(&json!({"direction": "unknown"})).is_err());
    }

    #[test]
    fn trellis_wire_names_are_relative_to_the_configuration_tile() {
        assert_eq!(
            trellis_wire_name((5, 7), "R6C8/JLOCAL").unwrap(),
            "N1E3_JLOCAL"
        );
        assert_eq!(
            trellis_wire_name((5, 7), "R6C8/G_HPBX0000").unwrap(),
            "G_HPBX0000"
        );
    }

    #[test]
    fn bitgen_accepts_a_closed_ecp5_checkpoint_without_simulation_evidence() {
        let mut checkpoint = json!({
            "schema_version": 3,
            "evidence": [
                "synthesis_equivalence", "mapped_netlist_complete",
                "physical_implementation", "timing_closure"
            ],
            "timing": {"met_timing": true},
            "target": {"family": "ECP5"}
        });
        assert!(validate_checkpoint(&checkpoint).is_ok());
        checkpoint["schema_version"] = json!(2);
        assert!(
            validate_checkpoint(&checkpoint)
                .unwrap_err()
                .to_string()
                .contains("schema version 3")
        );
    }

    #[test]
    fn dp16kd_emits_tile_group_cib_ties_and_zero_initialization() {
        let mut file: ArchitectureFile = serde_json::from_str(ARCHITECTURE).unwrap();
        file.locations[0].tiles.push(TileRecord {
            name: "MIB_R0C0:MIB_EBR0".into(),
            tile_type: "MIB_EBR0".into(),
        });
        file.locations[1].tiles.push(TileRecord {
            name: "MIB_R0C1:MIB_EBR1".into(),
            tile_type: "MIB_EBR1".into(),
        });
        let architecture = expand(file).unwrap();
        let placement = json!({
            "cell": "words", "cell_id": 7, "bel": "R0C0/EBR0", "bel_z": 0,
            "x": 0, "y": 0,
            "bel_pins": [
                {"name": "WEB", "cib_tie": {"tile": "CIB_R0C0:CIB_EBR", "mux": "JLSR0"}},
                {"name": "DIA8", "cib_tie": {"tile": "CIB_R0C0:CIB_EBR", "mux": "JD0"}},
                {"name": "CSA0", "cib_tie": {"tile": "CIB_R0C0:CIB_EBR", "mux": "JCE0"}}
            ]
        });
        let configuration = json!({
            "kind": "block_ram", "depth": 256, "word_width": 8,
            "physical_width": 9, "edge": "rising", "write_enable": "high",
            "read_enable": null
        });
        let packed = json!({
            "cell": 7, "wid": 3, "depth": 256, "word_width": 8, "physical_width": 9
        });
        let absorbed = json!({"WEB": false, "DIA8": false, "CSA0": false});
        let mut config = ChipConfig::default();

        write_bram(
            &mut config,
            &architecture,
            &placement,
            &configuration,
            &packed,
            absorbed.as_object().unwrap(),
        )
        .unwrap();

        assert_eq!(
            config.tile_groups[0].0,
            ["MIB_R0C0:MIB_EBR0", "MIB_R0C1:MIB_EBR1"]
        );
        assert!(
            config.tile_groups[0]
                .1
                .enums
                .contains(&("EBR0.DP16KD.DATA_WIDTH_A".into(), "9".into()))
        );
        assert!(
            config.tiles["CIB_R0C0:CIB_EBR"]
                .enums
                .contains(&("CIB.JCE0MUX".into(), "1".into()))
        );
        assert_eq!(config.bram_data[&3], vec![0; 2048]);
        assert_ne!(config.tile_groups[0].1, TileConfig::default());
    }
}
