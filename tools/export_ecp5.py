#!/usr/bin/env python3
"""Export Project Trellis's deduplicated ECP5 graph for texo-target-ecp5."""

import argparse
import json
import math
import sys
from pathlib import Path


SCHEMA_VERSION = 2
DIRECTIONS = {0: "input", 1: "output", 2: "inout"}
SPEED_GRADES = ["6", "7", "8", "8_5G"]


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
        help="Python module directory; pass libtrellis and timing/util (repeatable)",
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


def absolute_wire_name(pytrellis, graph, location, reference):
    x = location.x + reference.rel.x
    y = location.y + reference.rel.y
    target = pytrellis.Location(x, y)
    location_type = graph.locationTypes[graph.typeAtLocation[target]]
    wire = graph.to_str(location_type.wires[reference.id].name)
    return f"R{y}C{x}_{wire}"


def classify_pip(pip_classes, known_classes, source, sink):
    if "FCO" in source or "FCI" in sink:
        return "zero"
    if "F5" in source or "FX" in source or "FXA" in sink or "FXB" in sink:
        return "zero"
    timing_class = pip_classes.get_pip_class(source, sink)
    if timing_class is None or timing_class not in known_classes:
        return "default"
    return timing_class


def export_location_type(
    pytrellis, graph, location_type, representative, pip_classes, known_classes
):
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
        source = absolute_wire_name(pytrellis, graph, representative, arc.srcWire)
        sink = absolute_wire_name(pytrellis, graph, representative, arc.sinkWire)
        pips.append(
            {
                "from": relative(arc.srcWire),
                "to": relative(arc.sinkWire),
                "fixed": int(arc.cls) == 1,
                "tile_type": graph.to_str(arc.tiletype),
                "timing_class": classify_pip(
                    pip_classes, known_classes, source, sink
                ),
                "lutperm_flags": arc.lutperm_flags,
            }
        )
    return {"wires": wires, "bels": bels, "pips": pips}


def delay_range(entry):
    return {
        "min_ps": min(entry["rising"][0], entry["falling"][0]),
        "max_ps": max(entry["rising"][2], entry["falling"][2]),
    }


def merge_range(current, incoming):
    if current is None:
        return incoming
    return {
        "min_ps": min(current["min_ps"], incoming["min_ps"]),
        "max_ps": max(current["max_ps"], incoming["max_ps"]),
    }


def export_cell_timings(cell_database):
    slogic = cell_database["SLOGICB"]
    carry = cell_database["SCCU2C"]
    lut_arcs = {}
    carry_arcs = [{}, {}]
    ff_arcs = {}
    ff_checks = {}
    for entry in slogic:
        if entry["type"] == "IOPath":
            source = entry["from_pin"]
            destination = entry["to_pin"]
            if source in {"A0", "B0", "C0", "D0"} and destination == "F0":
                key = (source[0], "F")
                lut_arcs[key] = merge_range(lut_arcs.get(key), delay_range(entry))
            elif source == "CLK" and destination == "Q0":
                key = ("CLK", "Q")
                ff_arcs[key] = merge_range(ff_arcs.get(key), delay_range(entry))
        elif entry["type"] == "SetupHold" and not isinstance(entry["pin"], list):
            normalized = {"DI0": "DI", "M0": "M"}.get(entry["pin"], entry["pin"])
            if normalized not in {"DI", "M", "CE", "LSR"}:
                continue
            clock = entry["clock"][1]
            key = (normalized, clock)
            setup = {"min_ps": entry["setup"][0], "max_ps": entry["setup"][2]}
            hold = {"min_ps": entry["hold"][0], "max_ps": entry["hold"][2]}
            previous = ff_checks.get(key)
            ff_checks[key] = {
                "setup": merge_range(None if previous is None else previous["setup"], setup),
                "hold": merge_range(None if previous is None else previous["hold"], hold),
            }

    for entry in carry:
        if entry["type"] != "IOPath":
            continue
        source = entry["from_pin"]
        destination = entry["to_pin"]
        for slice_index in range(2):
            suffix = str(slice_index)
            if source in {f"A{suffix}", f"B{suffix}", f"C{suffix}", f"D{suffix}"}:
                normalized_source = source[0]
            elif source == "FCI":
                normalized_source = "FCI"
            else:
                continue
            if destination == f"F{suffix}":
                normalized_destination = "F"
            elif destination == "FCO":
                normalized_destination = "FCO"
            else:
                continue
            key = (normalized_source, normalized_destination)
            carry_arcs[slice_index][key] = merge_range(
                carry_arcs[slice_index].get(key), delay_range(entry)
            )

    def arcs(records):
        return [
            {"from_pin": source, "to_pin": sink, "delay": delay}
            for (source, sink), delay in sorted(records.items())
        ]

    checks = [
        {
            "signal_pin": signal,
            "clock_pin": clock,
            "setup": values["setup"],
            "hold": values["hold"],
        }
        for (signal, clock), values in sorted(ff_checks.items())
    ]
    return [
        {
            "cell_type": "DCCA",
            "arcs": [
                {
                    "from_pin": "CLKI",
                    "to_pin": "CLKO",
                    "delay": {"min_ps": 0, "max_ps": 0},
                }
            ],
            "setup_holds": [],
        },
        {
            "cell_type": "TRELLIS_COMB",
            "arcs": arcs(lut_arcs),
            "setup_holds": [],
        },
        {
            "cell_type": "TRELLIS_CARRY0",
            "arcs": arcs(carry_arcs[0]),
            "setup_holds": [],
        },
        {
            "cell_type": "TRELLIS_CARRY1",
            "arcs": arcs(carry_arcs[1]),
            "setup_holds": [],
        },
        {
            "cell_type": "TRELLIS_FF",
            "arcs": arcs(ff_arcs),
            "setup_holds": checks,
        },
    ]


def export_speed_grades(database):
    interconnect_by_grade = {}
    cells_by_grade = {}
    known_classes = {"default", "zero"}
    for grade in SPEED_GRADES:
        root = database / "ECP5" / "timing" / f"speed_{grade}"
        with (root / "interconnect.json").open(encoding="utf-8") as source:
            interconnect_by_grade[grade] = json.load(source)
        with (root / "cells.json").open(encoding="utf-8") as source:
            cells_by_grade[grade] = json.load(source)
    known_classes.update(interconnect_by_grade["6"])

    speed_grades = []
    for grade in SPEED_GRADES:
        classes = {}
        database_classes = interconnect_by_grade[grade]
        for name in sorted(known_classes):
            if name == "zero":
                values = (0, 0, 0, 0)
            elif name == "default" or name not in database_classes:
                values = (50, 50, 0, 0)
            else:
                timing = database_classes[name]
                values = (
                    math.floor(timing["delay"][0] * 1.1),
                    math.ceil(timing["delay"][2] * 1.1),
                    math.floor(timing["fanout"][0]),
                    math.ceil(timing["fanout"][2]),
                )
            classes[name] = {
                "min_base_ps": values[0],
                "max_base_ps": values[1],
                "min_fanout_adder_ps": values[2],
                "max_fanout_adder_ps": values[3],
            }
        speed_grades.append(
            {
                "name": grade,
                "pip_classes": classes,
                "cells": export_cell_timings(cells_by_grade[grade]),
            }
        )
    return speed_grades, known_classes


def main():
    args = parse_args()
    sys.path.extend(args.libdir)
    try:
        import pytrellis
    except ImportError as error:
        raise SystemExit(
            "unable to import pytrellis; pass its build directory with -L"
        ) from error
    try:
        import pip_classes
    except ImportError as error:
        raise SystemExit(
            "unable to import Project Trellis timing utilities; add "
            "-L /path/to/prjtrellis/timing/util"
        ) from error

    pytrellis.load_database(str(args.database))
    chip = pytrellis.Chip(args.device)
    graph = pytrellis.make_dedup_chipdb(
        chip, include_lutperm_pips=True, split_slice_mode=True
    )

    speed_grades, known_classes = export_speed_grades(args.database.resolve())
    location_type_keys = [entry.key() for entry in graph.locationTypes]
    representatives = {}
    for y in range(chip.get_max_row() + 1):
        for x in range(chip.get_max_col() + 1):
            location = pytrellis.Location(x, y)
            key = graph.typeAtLocation[location]
            representatives.setdefault(key, location)
    location_types = [
        export_location_type(
            pytrellis,
            graph,
            graph.locationTypes[key],
            representatives[key],
            pip_classes,
            known_classes,
        )
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
        "speed_grades": speed_grades,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as destination:
        json.dump(output, destination, separators=(",", ":"), sort_keys=True)
        destination.write("\n")


if __name__ == "__main__":
    main()
