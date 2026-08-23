#!/usr/bin/env python3
"""Convert a timing-closed Texo AXI4 checkpoint into an ECP5 bitstream."""

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile
from collections import Counter, defaultdict, deque
from pathlib import Path


REQUIRED_EVIDENCE = {
    "rtl_simulation",
    "synthesis_equivalence",
    "mapped_netlist_complete",
    "post_map_simulation",
    "physical_implementation",
    "timing_closure",
}


def resource_name(name):
    match = re.fullmatch(r"R(\d+)C(\d+)/(.*)", name)
    return f"X{match.group(2)}/Y{match.group(1)}/{match.group(3)}" if match else name


def placement_cell_names(checkpoint):
    totals = Counter(item["cell"] for item in checkpoint["placement"])
    occurrences = Counter()
    feedins = sum(
        1 for item in checkpoint["placement"] if re.fullmatch(r"\$carry_feedin\d+\$slice0", item["cell"])
    )
    feedouts = sum(
        1 for item in checkpoint["placement"] if re.fullmatch(r"\$carry_feedout\d+\$slice0", item["cell"])
    )
    result = []
    for item in checkpoint["placement"]:
        name = item["cell"]
        occurrence = occurrences[name]
        occurrences[name] += 1
        if totals[name] > 1 and occurrence + 1 < totals[name]:
            result.append(f"{name}$texo_duplicate{occurrence}")
            continue
        match = re.fullmatch(r"\$(clk|rst_n|failed|passed)\[0\]", name)
        if match:
            result.append(f"{match.group(1)}$tr_io")
            continue
        match = re.fullmatch(r"\$carry_feedin(\d+)\$slice([01])", name)
        if match:
            generated = 2 * (feedins - 1 - int(match.group(1)))
            result.append(f"$nextpnr_CCU2C_{generated}$CCU2_COMB{match.group(2)}")
            continue
        match = re.fullmatch(r"\$carry_feedout(\d+)\$slice([01])", name)
        if match:
            generated = 2 * (feedouts - 1 - int(match.group(1))) + 1
            result.append(f"$nextpnr_CCU2C_{generated}$CCU2_COMB{match.group(2)}")
            continue
        match = re.fullmatch(r"(.+)\$slice([01])", name)
        if match:
            result.append(f"{match.group(1)}$CCU2_COMB{match.group(2)}")
            continue
        if name == "$gbuf$$wire2":
            result.append("$gbuf$clk$TRELLIS_IO_IN")
            continue
        result.append(name)
    return result


def context_maps():
    return (
        {str(item.first): item.second for item in ctx.cells},
        {str(item.first): item.second for item in ctx.nets},
        {str(item.first): str(item.second) for item in ctx.net_aliases},
    )


def alias_net(alias, nets, aliases):
    actual = aliases.get(alias, alias)
    if actual not in nets:
        raise RuntimeError(f"packed net alias does not resolve: {alias} -> {actual}")
    return nets[actual]


def embedded_pack():
    ctx.pack()


def embedded_bridge():
    checkpoint_path = Path(os.environ["TEXO_CHECKPOINT"])
    config_path = Path(os.environ["TEXO_CONFIG"])
    with checkpoint_path.open(encoding="utf-8") as source:
        checkpoint = json.load(source)

    cells, nets, aliases = context_maps()
    bels = {str(bel): bel for bel in ctx.getBels()}
    mapped_cells = placement_cell_names(checkpoint)
    for item, cell_name in zip(checkpoint["placement"], mapped_cells):
        bel_name = resource_name(item["bel"])
        if cell_name not in cells or bel_name not in bels:
            raise RuntimeError(f"placement object is absent: {cell_name} -> {bel_name}")
        cell = cells[cell_name]
        if cell.bel:
            if str(cell.bel) != bel_name:
                raise RuntimeError(f"pre-bound BEL differs for {cell_name}: {cell.bel} != {bel_name}")
        else:
            ctx.bindBel(bels[bel_name], cell, STRENGTH_LOCKED)
    if sum(bool(item.second.bel) for item in ctx.cells) != len(ctx.cells):
        raise RuntimeError("not every packed cell was placed by the Texo checkpoint")

    original_ccus = []
    for item in sorted(checkpoint["placement"], key=lambda value: value["cell_id"]):
        match = re.fullmatch(r"(.+)\$slice0", item["cell"])
        if match and not match.group(1).startswith("$carry_"):
            original_ccus.append(match.group(1))
    feedins = sum(
        1 for item in checkpoint["placement"] if re.fullmatch(r"\$carry_feedin\d+\$slice0", item["cell"])
    )
    feedouts = sum(
        1 for item in checkpoint["placement"] if re.fullmatch(r"\$carry_feedout\d+\$slice0", item["cell"])
    )

    def route_net(name):
        if name == "$false":
            return alias_net("$PACKER_GND_NET", nets, aliases)
        if name == "$true":
            return alias_net("$PACKER_VCC_NET", nets, aliases)
        if name == "$glbnet$$wire2":
            return alias_net("$glbnet$clk$TRELLIS_IO_IN", nets, aliases)
        match = re.fullmatch(r"\$wire(\d+)", name)
        if match:
            bit = int(match.group(1))
            special = {
                2: "clk$TRELLIS_IO_IN",
                3: "rst_n$TRELLIS_IO_IN",
                1288: "passed$TRELLIS_IO_OUT",
                1289: "failed$TRELLIS_IO_OUT",
            }
            return alias_net(special.get(bit, f"$frontend${bit}"), nets, aliases)
        match = re.fullmatch(r"\$carry(\d+)", name)
        if not match:
            return alias_net(name, nets, aliases)
        signal = int(match.group(1))
        original_count = len(original_ccus)
        if signal < original_count:
            return alias_net(f"{original_ccus[signal]}$CCU2_FCI_INT", nets, aliases)
        feedin_limit = original_count + 2 * feedins
        if signal < feedin_limit:
            index = (signal - original_count) // 2
            generated = 2 * (feedins - 1 - index)
            suffix = "CCU2_FCI_INT" if (signal - original_count) % 2 == 0 else "COUT"
            return alias_net(f"$nextpnr_CCU2C_{generated}${suffix}", nets, aliases)
        offset = signal - feedin_limit
        feedout = offset // 4
        generated = 2 * (feedouts - 1 - feedout) + 1
        if offset % 4 == 0:
            port = {str(item.first): item.second for item in cells[f"$nextpnr_CCU2C_{generated}$CCU2_COMB0"].ports}["F"]
            if port.net is None:
                raise RuntimeError(f"feed-out {feedout} has no packed F net")
            return port.net
        if offset % 4 == 1:
            return alias_net(f"$nextpnr_CCU2C_{generated}$CCU2_FCI_INT", nets, aliases)
        raise RuntimeError(f"checkpoint contains an unexpected unrouted carry signal: {name}")

    route_nets = {route["net"]: route_net(route["net"]) for route in checkpoint["routes"]}
    needed = {
        resource_name(name)
        for route in checkpoint["routes"]
        for item in route["pips"]
        for name in (item["from"], item["to"])
    }
    needed.update(resource_name(route["driver_wire"]) for route in checkpoint["routes"])
    needed.update(resource_name(item["wire"]) for route in checkpoint["routes"] for item in route["wires"])
    wires = {}
    for wire in ctx.getWires():
        name = str(wire)
        if name in needed:
            wires[name] = wire
    missing_wires = needed - wires.keys()
    if missing_wires:
        raise RuntimeError(f"nextpnr lacks {len(missing_wires)} checkpoint wires: {sorted(missing_wires)[:5]}")

    programmable = 0
    fixed = 0
    for route in checkpoint["routes"]:
        net = route_nets[route["net"]]
        root = wires[resource_name(route["driver_wire"])]
        adjacency = defaultdict(list)
        for item in route["pips"]:
            source = wires[resource_name(item["from"])]
            target = wires[resource_name(item["to"])]
            direct = next(
                (pip for pip in ctx.getPipsDownhill(source) if ctx.getPipDstWire(pip) == target),
                None,
            )
            reverse = None
            if item["bidirectional"]:
                reverse = next(
                    (pip for pip in ctx.getPipsDownhill(target) if ctx.getPipDstWire(pip) == source),
                    None,
                )
                adjacency[target].append((source, reverse, item["fixed"]))
            adjacency[source].append((target, direct, item["fixed"]))
            if direct is None and reverse is None and not item["fixed"]:
                raise RuntimeError(f"programmable PIP is absent in nextpnr: {item['from']} -> {item['to']}")

        if ctx.getBoundWireNet(root) is None:
            ctx.bindWire(root, net, STRENGTH_LOCKED)
        visited = {root}
        queue = deque([root])
        while queue:
            source = queue.popleft()
            for target, pip, is_fixed in adjacency[source]:
                if target in visited:
                    continue
                if pip is None:
                    if not is_fixed:
                        continue
                    ctx.bindWire(target, net, STRENGTH_LOCKED)
                    fixed += 1
                else:
                    ctx.bindPip(pip, net, STRENGTH_LOCKED)
                    programmable += 1
                visited.add(target)
                queue.append(target)
        expected = {wires[resource_name(item["wire"])] for item in route["wires"]}
        if not expected.issubset(visited):
            raise RuntimeError(f"route {route['net']} left {len(expected - visited)} wires disconnected")

    if programmable + fixed != checkpoint["metrics"]["total_pips"]:
        raise RuntimeError(
            "checkpoint routes are not trees: bound "
            f"{programmable + fixed} of {checkpoint['metrics']['total_pips']} PIPs"
        )

    print(
        f"Texo bitstream bridge: {len(cells)} cells, {len(route_nets)} nets, "
        f"{programmable} programmable PIPs, {fixed} fixed edges"
    )
    write_bitstream(ctx, "", str(config_path))


def embedded_main():
    stage = os.environ.get("TEXO_BITGEN_STAGE")
    if stage == "pack":
        embedded_pack()
    elif stage == "bridge":
        embedded_bridge()
    else:
        raise RuntimeError(f"unknown embedded stage: {stage!r}")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--mapped-json", required=True, type=Path)
    parser.add_argument("--lpf", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--bit", required=True, type=Path)
    parser.add_argument("--nextpnr", default="nextpnr-ecp5")
    parser.add_argument("--ecppack", default="ecppack")
    parser.add_argument("--ecpunpack", default="ecpunpack")
    return parser.parse_args()


def run(command, environment=None):
    subprocess.run(command, check=True, env=environment)


def standalone_main():
    args = parse_args()
    with args.checkpoint.open(encoding="utf-8") as source:
        checkpoint = json.load(source)
    missing = REQUIRED_EVIDENCE - set(checkpoint.get("evidence", []))
    if missing:
        raise RuntimeError(f"bitstream release is missing evidence: {', '.join(sorted(missing))}")
    if not checkpoint.get("timing", {}).get("met_timing"):
        raise RuntimeError("bitstream release requires a timing-closed checkpoint")
    device = checkpoint["target"]["device"]
    if not device.endswith("85F"):
        raise RuntimeError(f"only the validated 85K AXI4 target is supported, got {device}")
    package = checkpoint["target"]["package"]
    script = Path(__file__).resolve()
    environment = os.environ.copy()

    with tempfile.TemporaryDirectory(prefix="texo-bitgen-") as temporary:
        temporary = Path(temporary)
        packed = temporary / "packed.json"
        trimmed = temporary / "trimmed.json"
        roundtrip_config = temporary / "roundtrip.config"
        roundtrip_bit = temporary / "roundtrip.bit"

        environment["TEXO_BITGEN_STAGE"] = "pack"
        run(
            [
                args.nextpnr,
                "--85k",
                "--package",
                package,
                "--json",
                str(args.mapped_json.resolve()),
                "--lpf",
                str(args.lpf.resolve()),
                "--run",
                str(script),
                "--write",
                str(packed),
            ],
            environment,
        )
        with packed.open(encoding="utf-8") as source:
            packed_design = json.load(source)
        module = next(iter(packed_design["modules"].values()))
        keep = set(placement_cell_names(checkpoint))
        extras = set(module["cells"]) - keep
        invalid = [name for name in extras if not re.fullmatch(r"\$nextpnr_CCU2C_\d+\$CCU2_COMB[01]", name)]
        if invalid:
            raise RuntimeError(f"packed design has unexpected cells absent from Texo: {invalid[:5]}")
        for name in extras:
            del module["cells"][name]
        if len(module["cells"]) != checkpoint["metrics"]["cells"]:
            raise RuntimeError("trimmed packed cell count differs from the checkpoint")
        with trimmed.open("w", encoding="utf-8") as destination:
            json.dump(packed_design, destination, indent=2)
            destination.write("\n")

        args.config.parent.mkdir(parents=True, exist_ok=True)
        args.bit.parent.mkdir(parents=True, exist_ok=True)
        environment.update(
            {
                "TEXO_BITGEN_STAGE": "bridge",
                "TEXO_CHECKPOINT": str(args.checkpoint.resolve()),
                "TEXO_CONFIG": str(args.config.resolve()),
            }
        )
        run(
            [
                args.nextpnr,
                "--85k",
                "--package",
                package,
                "--json",
                str(trimmed),
                "--no-pack",
                "--run",
                str(script),
            ],
            environment,
        )
        run([args.ecppack, str(args.config), str(args.bit)])
        run([args.ecpunpack, str(args.bit), str(roundtrip_config)])
        run([args.ecppack, str(roundtrip_config), str(roundtrip_bit)])
        bit_bytes = args.bit.read_bytes()
        if bit_bytes != roundtrip_bit.read_bytes():
            raise RuntimeError("ecpunpack/ecppack bitstream round trip changed the artifact")
        digest = hashlib.sha256(bit_bytes).hexdigest()
        print(f"bitstream: {args.bit} ({len(bit_bytes)} bytes, sha256 {digest})")
        print("bitstream/configuration round-trip: passed")


if "ctx" in globals():
    embedded_main()
else:
    try:
        standalone_main()
    except (OSError, subprocess.CalledProcessError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1)
