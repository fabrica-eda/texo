#!/usr/bin/env python3
"""Convert a timing-closed Texo ECP5 checkpoint without invoking nextpnr."""

import argparse
import gzip
import hashlib
import json
import re
import subprocess
import sys
import tempfile
from collections import defaultdict
from pathlib import Path

REQUIRED_EVIDENCE = {
    "rtl_simulation", "synthesis_equivalence", "mapped_netlist_complete",
    "post_map_simulation", "physical_implementation", "timing_closure",
}
DEFAULT_DATABASE = Path("/usr/share/trellis/database")
DEFAULT_PYTRELLIS_DIRS = (
    Path("/usr/lib/x86_64-linux-gnu/trellis"),
    Path("/usr/lib/aarch64-linux-gnu/trellis"),
    Path("/usr/local/lib/trellis"),
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--bit", required=True, type=Path)
    parser.add_argument("--database", default=DEFAULT_DATABASE, type=Path)
    parser.add_argument("--base-config", type=Path)
    parser.add_argument("--pytrellis-libdir", action="append", default=[], type=Path)
    parser.add_argument("--ecppack", default="ecppack")
    parser.add_argument("--ecpunpack", default="ecpunpack")
    return parser.parse_args()


def import_pytrellis(extra_directories):
    for directory in (*extra_directories, *DEFAULT_PYTRELLIS_DIRS):
        if directory.is_dir() and str(directory) not in sys.path:
            sys.path.insert(0, str(directory))
    try:
        import pytrellis
    except ImportError as error:
        raise RuntimeError(
            "pytrellis is required; install python3-pytrellis or pass --pytrellis-libdir"
        ) from error
    return pytrellis


def default_base_config(device):
    stem = f"empty_{device.lower()}.config"
    root = Path("/usr/share/doc/fpga-trellis/basecfgs")
    for path in (root / stem, root / f"{stem}.gz"):
        if path.is_file():
            return path
    raise RuntimeError(f"Project Trellis base config is absent for {device}")


def read_text(path):
    if path.suffix == ".gz":
        with gzip.open(path, "rt", encoding="utf-8") as source:
            return source.read()
    return path.read_text(encoding="utf-8")


def run(command):
    subprocess.run(command, check=True)


def bool_vector(pytrellis, values):
    result = pytrellis.BoolVector()
    for value in values:
        result.append(bool(value))
    return result


def integer_bits(value, width):
    return [(value & (1 << bit)) != 0 for bit in range(width)]


def fold_lut_input(initialization, input_index, value):
    folded = 0
    for output_index in range(16):
        source_index = (output_index & ~(1 << input_index)) | (int(value) << input_index)
        if initialization & (1 << source_index):
            folded |= 1 << output_index
    return folded


def tile_config(pytrellis, config, name):
    if name not in config.tiles:
        config.tiles[name] = pytrellis.TileConfig()
    return config.tiles[name]


def add_word(pytrellis, config, tile, name, values):
    tile_config(pytrellis, config, tile).add_word(name, bool_vector(pytrellis, values))


def add_enum(pytrellis, config, tile, name, value):
    tile_config(pytrellis, config, tile).add_enum(name, str(value))


def parse_qualified_resource(name):
    match = re.fullmatch(r"R(\d+)C(\d+)/(.*)", name)
    if match is None:
        raise RuntimeError(f"invalid qualified ECP5 resource: {name}")
    return int(match.group(2)), int(match.group(1)), match.group(3)


def tile_point(name):
    match = re.search(r"R(\d+)C(\d+)", name)
    if match is None:
        raise RuntimeError(f"configuration tile lacks coordinates: {name}")
    return int(match.group(2)), int(match.group(1))


def trellis_wire_name(owner, qualified):
    wire_x, wire_y, basename = parse_qualified_resource(qualified)
    if basename.startswith(("G_", "L_", "R_")) or (wire_x, wire_y) == owner:
        return basename
    owner_x, owner_y = owner
    prefix = ""
    if wire_y < owner_y:
        prefix += f"N{owner_y - wire_y}"
    if wire_y > owner_y:
        prefix += f"S{wire_y - owner_y}"
    if wire_x > owner_x:
        prefix += f"E{wire_x - owner_x}"
    if wire_x < owner_x:
        prefix += f"W{owner_x - wire_x}"
    return f"{prefix}_{basename}"


def add_routes(pytrellis, config, checkpoint):
    programmable = 0
    fixed = 0
    incoming_flags = defaultdict(list)
    routed_wires = set()
    for route in checkpoint["routes"]:
        routed_wires.update(wire["wire"] for wire in route["wires"])
        for pip in route["pips"]:
            incoming_flags[pip["to_wire_id"]].append(pip["lutperm_flags"])
            if pip["fixed"]:
                fixed += 1
                continue
            tile = pip.get("config_tile")
            if not tile:
                raise RuntimeError(f"programmable PIP {pip['pip_id']} has no config tile")
            owner = tile_point(tile)
            source = trellis_wire_name(owner, pip["from"])
            sink = trellis_wire_name(owner, pip["to"])
            tile_config(pytrellis, config, tile).add_arc(sink, source)
            programmable += 1
    return programmable, fixed, incoming_flags, routed_wires


def logic_tile(placement):
    matches = [tile["name"] for tile in placement["configuration_tiles"] if tile["tile_type"] == "PLC2"]
    if len(matches) != 1:
        raise RuntimeError(f"{placement['bel']} has {len(matches)} PLC2 config tiles")
    return matches[0]


def slice_and_lc(placement):
    z = placement["bel_z"] >> 2
    return f"SLICE{'ABCD'[z // 2]}", str(z % 2)


def permute_lut(configuration, placement, incoming_flags, absorbed_inputs):
    original = configuration.get("init", 0xFFFF if configuration.get("value") else 0)
    if configuration["kind"] == "carry_slice":
        # ECP5 carry inputs are physically disconnected at packing time and
        # consequently read as one. Fold logical zeroes into INIT first so the
        # physical tie-high remains Boolean-equivalent to the mapped CCU2C.
        for input_index, pin in enumerate("ABCD"):
            if pin in absorbed_inputs and not absorbed_inputs[pin]:
                original = fold_lut_input(original, input_index, False)
    pin_wires = {
        pin["name"]: pin["wire_id"] for pin in placement["bel_pins"] if pin["name"] in "ABCD"
    }
    physical_to_logical = [[] for _ in range(4)]
    for physical, name in enumerate("ABCD"):
        for flags in incoming_flags.get(pin_wires[name], []):
            if flags & 0x4000:
                logical = flags & 0x3
                destination = (flags >> 2) & 0x3
                if destination != physical:
                    raise RuntimeError(f"invalid LUT permutation flags 0x{flags:04x}")
                physical_to_logical[logical].append(physical)
            else:
                physical_to_logical[physical].append(physical)
    if configuration["kind"] == "carry_slice":
        for physical in range(4):
            if physical_to_logical[physical]:
                continue
            for logical in range(2 * (physical // 2), 2 * ((physical // 2) + 1)):
                if not incoming_flags.get(pin_wires["ABCD"[logical]]):
                    physical_to_logical[physical].append(logical)
    permuted = 0
    for physical_value in range(16):
        logical_value = 0
        for physical in range(4):
            if physical_value & (1 << physical):
                for logical in physical_to_logical[physical]:
                    logical_value |= 1 << logical
        if original & (1 << logical_value):
            permuted |= 1 << physical_value
    used = {index for index, mappings in enumerate(physical_to_logical) if mappings}
    return permuted, used


def write_comb(pytrellis, config, placement, configuration, incoming_flags, absorbed_inputs):
    tile = logic_tile(placement)
    slice_name, lc = slice_and_lc(placement)
    mode = "CCU2" if configuration["kind"] == "carry_slice" else "LOGIC"
    init, used = permute_lut(configuration, placement, incoming_flags, absorbed_inputs)
    add_enum(pytrellis, config, tile, f"{slice_name}.MODE", mode)
    add_word(pytrellis, config, tile, f"{slice_name}.K{lc}.INIT", integer_bits(init, 16))
    inject = "YES" if configuration.get("inject", False) else "NO"
    add_enum(pytrellis, config, tile, f"{slice_name}.CCU2.INJECT1_{lc}", inject if mode == "CCU2" else "_NONE_")
    for physical, pin in enumerate("ABCD"):
        if physical not in used:
            add_enum(pytrellis, config, tile, f"{slice_name}.{pin}{lc}MUX", "1")


def route_uses_local_wire(routed_wires, point, basename):
    return f"R{point[1]}C{point[0]}/{basename}" in routed_wires


def write_ff(pytrellis, config, routed_wires, placement, configuration, dedicated_ffs):
    tile = logic_tile(placement)
    slice_name, lc = slice_and_lc(placement)
    reset = configuration.get("reset")
    enable = configuration.get("enable")
    add_enum(pytrellis, config, tile, f"{slice_name}.GSR", "DISABLED")
    add_enum(pytrellis, config, tile, f"{slice_name}.REG{lc}.SD", "1" if placement["cell_id"] in dedicated_ffs else "0")
    add_enum(pytrellis, config, tile, f"{slice_name}.REG{lc}.REGSET", "SET" if reset and reset["value"] else "RESET")
    add_enum(pytrellis, config, tile, f"{slice_name}.REG{lc}.LSRMODE", "LSR")
    ce_mux = "1" if enable is None else ("CE" if enable == "high" else "INV")
    add_enum(pytrellis, config, tile, f"{slice_name}.CEMUX", ce_mux)
    point = (placement["x"], placement["y"])
    if reset:
        srmode = "ASYNC" if reset["asynchronous"] else "LSR_OVER_CE"
        lsrmux = "LSR" if reset["active"] == "high" else "INV"
        for index in range(2):
            if route_uses_local_wire(routed_wires, point, f"LSR{index}"):
                add_enum(pytrellis, config, tile, f"LSR{index}.SRMODE", srmode)
                add_enum(pytrellis, config, tile, f"LSR{index}.LSRMUX", lsrmux)
    clkmux = "CLK" if configuration["edge"] == "rising" else "INV"
    for index in range(2):
        if route_uses_local_wire(routed_wires, point, f"CLK{index}"):
            add_enum(pytrellis, config, tile, f"CLK{index}.CLKMUX", clkmux)


def chip_tile(chip, y, x, accepted):
    accepted = {accepted} if isinstance(accepted, str) else set(accepted)
    matches = [tile.info.name for tile in chip.get_tiles_by_position(y, x) if tile.info.type in accepted]
    if len(matches) != 1:
        raise RuntimeError(f"R{y}C{x} has {len(matches)} tiles of type {sorted(accepted)}")
    return matches[0]


def io_tiles(chip, placement):
    x, y = placement["x"], placement["y"]
    pio = placement["bel"].rsplit("/", 1)[1]
    max_x, max_y = chip.get_max_col(), chip.get_max_row()
    if y == 0:
        offset = 0 if pio == "PIOA" else 1
        return chip_tile(chip, 0, x + offset, f"PIOT{offset}"), chip_tile(chip, 1, x + offset, f"PICT{offset}"), chip_tile(chip, 1, x + offset, "CIB"), "JB0"
    if y == max_y:
        offset = 0 if pio == "PIOA" else 1
        types = {"PICB0", "EFB0_PICB0", "EFB2_PICB0", "SPICB0"} if offset == 0 else {"PICB1", "EFB1_PICB1", "EFB3_PICB1"}
        tile = chip_tile(chip, y, x + offset, types)
        return tile, tile, None, None
    if x == 0:
        if pio in {"PIOA", "PIOB"}:
            pio_y, pic_y, pic_types = y + 1, y, {"PICL0", "PICL0_DQS2"}
        else:
            pio_y, pic_y, pic_types = y + 1, y + 2, {"PICL2", "PICL2_DQS1", "MIB_CIB_LR"}
        return chip_tile(chip, pio_y, 0, {"PICL1", "PICL1_DQS0", "PICL1_DQS3"}), chip_tile(chip, pic_y, 0, pic_types), None, None
    if x == max_x:
        if pio in {"PIOA", "PIOB"}:
            pio_y, pic_y, pic_types = y + 1, y, {"PICR0", "PICR0_DQS2"}
        else:
            pio_y, pic_y, pic_types = y + 1, y + 2, {"PICR2", "PICR2_DQS1", "MIB_CIB_LR_A"}
        return chip_tile(chip, pio_y, x, {"PICR1", "PICR1_DQS0", "PICR1_DQS3"}), chip_tile(chip, pic_y, x, pic_types), None, None
    raise RuntimeError(f"PIO is not on the device edge: {placement['bel']}")


def io_bank(database, device, placement):
    with (database / "ECP5" / device / "iodb.json").open(encoding="utf-8") as source:
        records = json.load(source)["pio_metadata"]
    pio = placement["bel"].rsplit("/", 1)[1].removeprefix("PIO")
    for record in records:
        if record["col"] == placement["x"] and record["row"] == placement["y"] and record["pio"] == pio:
            return record["bank"]
    raise RuntimeError(f"IO bank metadata is absent for {placement['bel']}")


def io_voltage(io_type):
    voltage = {"LVCMOS33": "3V3", "LVCMOS25": "2V5", "LVCMOS18": "1V8", "LVCMOS15": "1V5", "LVCMOS12": "1V2"}.get(io_type)
    if voltage is None:
        raise RuntimeError(f"native bitgen does not yet classify IO_TYPE={io_type}")
    return voltage


def write_io(pytrellis, config, chip, database, device, placement, configuration, attributes):
    pio_tile, pic_tile, tristate_tile, tristate_wire = io_tiles(chip, placement)
    pio = placement["bel"].rsplit("/", 1)[1]
    direction = "INPUT" if configuration["direction"] == "input" else "OUTPUT"
    io_type = attributes.get("IO_TYPE", "LVCMOS33")
    add_enum(pytrellis, config, pio_tile, f"{pio}.BASE_TYPE", f"{direction}_{io_type}")
    add_enum(pytrellis, config, pic_tile, f"{pio}.BASE_TYPE", f"{direction}_{io_type}")
    if direction == "INPUT":
        add_enum(pytrellis, config, pio_tile, f"{pio}.HYSTERESIS", attributes.get("HYSTERESIS", "ON"))
    elif tristate_tile:
        add_enum(pytrellis, config, tristate_tile, f"CIB.{tristate_wire}MUX", "0")
    for attribute, default in (("SLEWRATE", "SLOW"), ("PULLMODE", "NONE"), ("DIFFRESISTOR", "OFF"), ("CLAMP", "OFF"), ("DRIVE", "8"), ("OPENDRAIN", "OFF")):
        if attribute in attributes:
            add_enum(pytrellis, config, pio_tile, f"{pio}.{attribute}", attributes.get(attribute, default))
    if direction == "OUTPUT":
        bank = io_bank(database, device, placement)
        bank_type = f"BANKREF{bank}"
        bank_tiles = [tile.info.name for tile in chip.tiles.values() if tile.info.type == bank_type]
        if len(bank_tiles) != 1:
            raise RuntimeError(f"device has {len(bank_tiles)} {bank_type} tiles")
        add_enum(pytrellis, config, bank_tiles[0], "BANK.VCCIO", io_voltage(io_type))


def build_config(pytrellis, checkpoint, database, base_config):
    pytrellis.load_database(str(database))
    device = checkpoint["target"]["device"]
    chip = pytrellis.Chip(device)
    config = pytrellis.ChipConfig.from_string(read_text(base_config))
    config.metadata.append(f"Part: {device}-{checkpoint['target']['package']}")
    programmable, fixed, incoming_flags, routed_wires = add_routes(
        pytrellis, config, checkpoint
    )
    metadata = {item["cell_id"]: item["configuration"] for item in checkpoint["primitive_metadata"]}
    attributes = {item["cell_id"]: item["attributes"] for item in checkpoint["packing"]["io_attributes"]}
    absorbed_inputs = {item["cell_id"]: item["pins"] for item in checkpoint["absorbed_inputs"]}
    dedicated_ffs = {item["ff"] for item in checkpoint["packing"]["lut_ff_pairs"]}
    for placement in checkpoint["placement"]:
        configuration = metadata.get(placement["cell_id"])
        if placement["kind"] == "global_clock":
            continue
        if configuration is None:
            raise RuntimeError(f"cell has no primitive configuration: {placement['cell']}")
        kind = configuration["kind"]
        if kind in {"lut4", "carry_slice", "constant"}:
            write_comb(
                pytrellis,
                config,
                placement,
                configuration,
                incoming_flags,
                absorbed_inputs.get(placement["cell_id"], {}),
            )
        elif kind == "flip_flop":
            write_ff(
                pytrellis,
                config,
                routed_wires,
                placement,
                configuration,
                dedicated_ffs,
            )
        elif kind == "port":
            write_io(pytrellis, config, chip, database, device, placement, configuration, attributes.get(placement["cell_id"], {}))
        elif kind == "block_ram":
            raise RuntimeError("native DP16KD configuration is not implemented yet")
        else:
            raise RuntimeError(f"unsupported native configuration primitive: {kind}")
    return config.to_string(), programmable, fixed


def validate_checkpoint(checkpoint):
    if checkpoint.get("schema_version") != 2:
        raise RuntimeError("native bitgen requires checkpoint schema version 2")
    missing = REQUIRED_EVIDENCE - set(checkpoint.get("evidence", []))
    if missing:
        raise RuntimeError(f"bitstream release is missing evidence: {', '.join(sorted(missing))}")
    if not checkpoint.get("timing", {}).get("met_timing"):
        raise RuntimeError("bitstream release requires a timing-closed checkpoint")
    if checkpoint["target"].get("family") != "ECP5":
        raise RuntimeError("native bitgen only accepts ECP5 checkpoints")


def main():
    args = parse_args()
    with args.checkpoint.open(encoding="utf-8") as source:
        checkpoint = json.load(source)
    validate_checkpoint(checkpoint)
    pytrellis = import_pytrellis(args.pytrellis_libdir)
    base_config = args.base_config or default_base_config(checkpoint["target"]["device"])
    config_text, programmable, fixed = build_config(pytrellis, checkpoint, args.database.resolve(), base_config.resolve())
    if programmable + fixed != checkpoint["metrics"]["total_pips"]:
        raise RuntimeError(f"configuration accounted for {programmable + fixed} of {checkpoint['metrics']['total_pips']} PIPs")
    args.config.parent.mkdir(parents=True, exist_ok=True)
    args.bit.parent.mkdir(parents=True, exist_ok=True)
    args.config.write_text(config_text, encoding="utf-8")
    with tempfile.TemporaryDirectory(prefix="texo-bitgen-") as temporary:
        temporary = Path(temporary)
        roundtrip_config = temporary / "roundtrip.config"
        roundtrip_bit = temporary / "roundtrip.bit"
        run([args.ecppack, str(args.config), str(args.bit)])
        run([args.ecpunpack, str(args.bit), str(roundtrip_config)])
        run([args.ecppack, str(roundtrip_config), str(roundtrip_bit)])
        bit_bytes = args.bit.read_bytes()
        if bit_bytes != roundtrip_bit.read_bytes():
            raise RuntimeError("ecpunpack/ecppack bitstream round trip changed the artifact")
    digest = hashlib.sha256(bit_bytes).hexdigest()
    print(f"Texo native bitgen: {checkpoint['metrics']['cells']} cells, {programmable} programmable PIPs, {fixed} fixed edges")
    print(f"bitstream: {args.bit} ({len(bit_bytes)} bytes, sha256 {digest})")
    print("bitstream/configuration round-trip: passed")


if __name__ == "__main__":
    try:
        main()
    except (ImportError, OSError, subprocess.CalledProcessError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1)
