#!/usr/bin/env python3
"""Export Project Trellis's deduplicated ECP5 graph for texo-target-ecp5."""

import argparse
import json
import sys
from pathlib import Path


SCHEMA_VERSION = 1
DIRECTIONS = {0: "input", 1: "output", 2: "inout"}


def parse_args():
    parser = argparse.ArgumentParser(
        description="export a versioned ECP5 architecture snapshot for Texo"
    )
    parser.add_argument("--database", required=True, type=Path, help="prjtrellis-db root")
    parser.add_argument("--device", required=True, help="exact database device name")
    parser.add_argument("--output", required=True, type=Path, help="output JSON file")
    parser.add_argument(
        "--project-trellis-revision",
        required=True,
        help="Project Trellis source Git revision",
    )
    parser.add_argument(
        "--database-revision", required=True, help="prjtrellis-db Git revision"
    )
    parser.add_argument(
        "-L",
        "--libdir",
        action="append",
        default=[],
        help="directory containing the pytrellis module (repeatable)",
    )
    return parser.parse_args()


def relative(reference):
    return {
        "dx": reference.rel.x,
        "dy": reference.rel.y,
        "index": reference.id,
    }


def find_bel_index(graph, location, name):
    location_type = graph.locationTypes[graph.typeAtLocation[location]]
    for index, bel in enumerate(location_type.bels):
        if graph.to_str(bel.name) == name:
            return index
    return None


def export_packages(pytrellis, graph, database, device):
    with (database / "ECP5" / device / "iodb.json").open(encoding="utf-8") as source:
        io_database = json.load(source)

    packages = []
    for package_name, package_data in sorted(io_database["packages"].items()):
        pins = []
        for pin_name, pin_location in sorted(package_data.items()):
            x = pin_location["col"]
            y = pin_location["row"]
            location = pytrellis.Location(x, y)
            bel = find_bel_index(graph, location, "PIO" + pin_location["pio"])
            # Project Trellis intentionally omits some special-purpose bottom IO.
            if bel is not None:
                pins.append({"name": pin_name, "x": x, "y": y, "bel": bel})
        packages.append({"name": package_name, "pins": pins})
    return packages


def export_location_type(graph, location_type):
    wires = [{"name": graph.to_str(wire.name)} for wire in location_type.wires]
    bels = []
    for bel in location_type.bels:
        pins = [
            {
                "name": graph.to_str(pin.pin),
                "direction": DIRECTIONS[int(pin.dir)],
                "wire": relative(pin.wire),
            }
            for pin in bel.wires
        ]
        bels.append(
            {
                "name": graph.to_str(bel.name),
                "bel_type": graph.to_str(bel.type),
                "z": bel.z,
                "pins": pins,
            }
        )

    pips = []
    for arc in location_type.arcs:
        pips.append(
            {
                "from": relative(arc.srcWire),
                "to": relative(arc.sinkWire),
                "fixed": int(arc.cls) == 1,
                "tile_type": graph.to_str(arc.tiletype),
                "delay": arc.delay,
                "lutperm_flags": arc.lutperm_flags,
            }
        )
    return {"wires": wires, "bels": bels, "pips": pips}


def main():
    args = parse_args()
    sys.path.extend(args.libdir)
    try:
        import pytrellis
    except ImportError as error:
        raise SystemExit(
            "unable to import pytrellis; pass its build directory with -L"
        ) from error

    pytrellis.load_database(str(args.database))
    chip = pytrellis.Chip(args.device)
    graph = pytrellis.make_dedup_chipdb(
        chip, include_lutperm_pips=True, split_slice_mode=True
    )

    location_type_keys = [entry.key() for entry in graph.locationTypes]
    location_types = [
        export_location_type(graph, graph.locationTypes[key])
        for key in location_type_keys
    ]
    locations = []
    for y in range(chip.get_max_row() + 1):
        for x in range(chip.get_max_col() + 1):
            key = graph.typeAtLocation[pytrellis.Location(x, y)]
            locations.append(
                {"x": x, "y": y, "location_type": location_type_keys.index(key)}
            )

    output = {
        "schema_version": SCHEMA_VERSION,
        "provenance": {
            "project_trellis_revision": args.project_trellis_revision,
            "database_revision": args.database_revision,
            "include_lutperm_pips": True,
            "split_slice_mode": True,
        },
        "family": "ECP5",
        "device": args.device,
        "width": chip.get_max_col() + 1,
        "height": chip.get_max_row() + 1,
        "location_types": location_types,
        "locations": locations,
        "packages": export_packages(
            pytrellis, graph, args.database.resolve(), args.device
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as destination:
        json.dump(output, destination, separators=(",", ":"), sort_keys=True)
        destination.write("\n")


if __name__ == "__main__":
    main()
