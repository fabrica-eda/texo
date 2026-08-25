//! Stable JSON checkpoint serialization for implemented ECP5 designs.

use serde_json::{Value, json};
use texo_flow::{Ecp5FlowResult, Evidence, Gate};
use texo_model::{Design, ResourceKind};
use texo_struo::{ActiveLevel, ClockEdge, PortDirection, PrimitiveMetadata};
use texo_target_ecp5::Ecp5Architecture;

/// Builds the stable, schema-versioned JSON representation of one ECP5 run.
#[must_use]
pub fn ecp5_checkpoint(
    design_name: &str,
    result: &Ecp5FlowResult,
    architecture: &Ecp5Architecture,
    package: &str,
    evidence: &Evidence,
) -> Value {
    let device = architecture.device();
    json!({
        "schema_version": 1,
        "design": design_name,
        "target": {
            "family": "ECP5",
            "device": device.name(),
            "package": package,
            "speed_grade": result.speed_grade,
            "placement_weight_exponent": result.placement_weight_exponent,
            "project_trellis_revision": architecture.provenance().project_trellis_revision,
            "database_revision": architecture.provenance().database_revision,
        },
        "evidence": checkpoint_evidence(evidence),
        "metrics": {
            "cells": result.design.cells().len(),
            "nets": result.design.nets().len(),
            "routed_nets": result.implementation.routes.len(),
            "total_pips": result.implementation.total_pips,
        },
        "primitive_metadata": result.primitive_metadata.iter().map(|(cell, metadata)| {
            primitive_metadata_json(*cell, metadata, &result.design)
        }).collect::<Vec<_>>(),
        "absorbed_inputs": checkpoint_absorbed_inputs(result),
        "packing": checkpoint_packing(result),
        "placement": checkpoint_placement(result, architecture),
        "routes": checkpoint_routes(result, architecture),
        "timing": checkpoint_timing(result),
    })
}

fn checkpoint_placement(result: &Ecp5FlowResult, architecture: &Ecp5Architecture) -> Vec<Value> {
    result
        .implementation
        .placement
        .bindings()
        .iter()
        .enumerate()
        .map(|(cell_id, bel_id)| {
            let cell = &result.design.cells()[cell_id];
            let bel = &architecture.device().bels()[bel_id.0];
            json!({
                "cell_id": cell_id,
                "cell": cell.name,
                "kind": checkpoint_resource_kind(cell.kind),
                "bel_id": bel_id.0,
                "bel": bel.name,
                "x": bel.point.x,
                "y": bel.point.y,
            })
        })
        .collect()
}

const fn checkpoint_resource_kind(kind: ResourceKind) -> &'static str {
    match kind {
        ResourceKind::Logic => "logic",
        ResourceKind::Lut(4) => "lut4",
        ResourceKind::Lut(_) => "lut",
        ResourceKind::Register => "flip_flop",
        ResourceKind::Memory => "block_ram",
        ResourceKind::Clock => "global_clock",
        ResourceKind::Io => "port",
        ResourceKind::Constant => "constant",
    }
}

fn checkpoint_routes(result: &Ecp5FlowResult, architecture: &Ecp5Architecture) -> Vec<Value> {
    let device = architecture.device();
    result
        .implementation
        .routes
        .iter()
        .map(|route| {
            let net = &result.design.nets()[route.net.0];
            let driver_pin = &result.design.pins()[net.driver.0];
            let driver_bel = result.implementation.placement.bindings()[driver_pin.cell.0];
            let driver_bel_pin = result
                .implementation
                .placement
                .pin_binding(net.driver)
                .or_else(|| {
                    device.bels()[driver_bel.0]
                        .pins()
                        .iter()
                        .copied()
                        .find(|bel_pin| {
                            let physical = &device.bel_pins()[bel_pin.0];
                            physical.name == driver_pin.name
                                && physical.direction == driver_pin.direction
                        })
                })
                .expect("a routed net driver has a physical BEL pin");
            let driver_wire = device.bel_pins()[driver_bel_pin.0].wire;
            let wires = route
                .wires()
                .map(|wire| json!({ "wire_id": wire.0, "wire": device.wires()[wire.0].name }))
                .collect::<Vec<_>>();
            let pips = route
                .pips()
                .map(|pip_id| {
                    let pip = &device.pips()[pip_id.0];
                    json!({
                        "pip_id": pip_id.0,
                        "from_wire_id": pip.from().0,
                        "from": device.wires()[pip.from().0].name,
                        "to_wire_id": pip.to().0,
                        "to": device.wires()[pip.to().0].name,
                        "bidirectional": pip.bidirectional(),
                        "fixed": architecture.pip_metadata(pip_id).fixed,
                    })
                })
                .collect::<Vec<_>>();
            json!({
                "net_id": route.net.0,
                "net": net.name,
                "driver_wire_id": driver_wire.0,
                "driver_wire": device.wires()[driver_wire.0].name,
                "wires": wires,
                "pips": pips,
            })
        })
        .collect()
}

fn checkpoint_absorbed_inputs(result: &Ecp5FlowResult) -> Vec<Value> {
    result
        .absorbed_inputs
        .iter()
        .map(|(cell, pins)| {
            json!({
                "cell_id": cell.0,
                "cell": result.design.cells()[cell.0].name,
                "pins": pins,
            })
        })
        .collect()
}

fn checkpoint_packing(result: &Ecp5FlowResult) -> Value {
    let lut_ff_pairs = result
        .packing
        .lut_ff_pairs()
        .iter()
        .map(|pair| json!({ "lut": pair.lut.0, "ff": pair.ff.0 }))
        .collect::<Vec<_>>();
    let carry_pairs = result
        .packing
        .carry_pairs()
        .iter()
        .map(|pair| json!({ "first": pair[0].0, "second": pair[1].0 }))
        .collect::<Vec<_>>();
    let block_rams = result
        .packing
        .block_rams()
        .iter()
        .map(|ram| {
            json!({
                "cell": ram.cell.0,
                "wid": ram.wid,
                "depth": ram.depth,
                "word_width": ram.word_width,
                "physical_width": ram.physical_width,
            })
        })
        .collect::<Vec<_>>();
    let global_clocks = result
        .packing
        .global_clocks()
        .iter()
        .map(|clock| {
            json!({
                "source_net": clock.source_net.0,
                "buffer": clock.buffer.0,
                "global_net": clock.global_net.0,
            })
        })
        .collect::<Vec<_>>();
    let io_attributes = result
        .packing
        .io_attributes()
        .iter()
        .map(|(cell, attributes)| {
            json!({
                "cell_id": cell.0,
                "cell": result.design.cells()[cell.0].name,
                "attributes": attributes,
            })
        })
        .collect::<Vec<_>>();
    let clock_frequencies_hz = result
        .packing
        .clock_frequencies_hz()
        .iter()
        .map(|(cell, frequency_hz)| {
            json!({
                "cell_id": cell.0,
                "cell": result.design.cells()[cell.0].name,
                "frequency_hz": frequency_hz,
            })
        })
        .collect::<Vec<_>>();
    json!({
        "lut_ff_pairs": lut_ff_pairs,
        "carry_pairs": carry_pairs,
        "general_routing_ffs": result.packing.general_routing_ffs().iter().map(|cell| cell.0).collect::<Vec<_>>(),
        "block_rams": block_rams,
        "global_clocks": global_clocks,
        "io_attributes": io_attributes,
        "clock_frequencies_hz": clock_frequencies_hz,
        "unsupported_lpf_commands": result.packing.unsupported_lpf_commands(),
    })
}

fn checkpoint_timing(result: &Ecp5FlowResult) -> Value {
    let net_delays = result
        .timing
        .net_delays
        .iter()
        .map(|delay| {
            let net = &result.design.nets()[delay.net.0];
            let driver = &result.design.pins()[net.driver.0];
            let sink = &result.design.pins()[delay.sink.0];
            json!({
                "net_id": delay.net.0,
                "net": net.name,
                "driver_pin_id": net.driver.0,
                "driver_pin": driver.name,
                "driver_cell_id": driver.cell.0,
                "driver_cell": result.design.cells()[driver.cell.0].name,
                "sink_pin_id": delay.sink.0,
                "sink_pin": sink.name,
                "sink_cell_id": sink.cell.0,
                "sink_cell": result.design.cells()[sink.cell.0].name,
                "min_delay_ps": delay.delay.min_ps,
                "max_delay_ps": delay.delay.max_ps,
            })
        })
        .collect::<Vec<_>>();
    let setup_checks = result
        .timing
        .setup_checks
        .iter()
        .map(|check| {
            json!({
                "cell_id": check.cell.0,
                "cell": result.design.cells()[check.cell.0].name,
                "data_pin_id": check.data_pin.0,
                "clock_net_id": check.clock_net.0,
                "arrival_ps": check.arrival_ps,
                "clock_arrival_ps": check.clock_arrival_ps,
                "setup_ps": check.setup_ps,
                "required_ps": check.required_ps,
                "slack_ps": check.slack_ps,
            })
        })
        .collect::<Vec<_>>();
    let net_setup_slacks = result
        .timing
        .net_setup_slacks
        .iter()
        .map(|edge| {
            json!({
                "net_id": edge.net.0,
                "net": result.design.nets()[edge.net.0].name,
                "sink_pin_id": edge.sink.0,
                "slack_ps": edge.slack_ps,
            })
        })
        .collect::<Vec<_>>();
    let hold_checks = result
        .timing
        .hold_checks
        .iter()
        .map(|check| {
            json!({
                "cell_id": check.cell.0,
                "cell": result.design.cells()[check.cell.0].name,
                "data_pin_id": check.data_pin.0,
                "clock_net_id": check.clock_net.0,
                "arrival_ps": check.arrival_ps,
                "clock_arrival_ps": check.clock_arrival_ps,
                "hold_ps": check.hold_ps,
                "required_ps": check.required_ps,
                "slack_ps": check.slack_ps,
            })
        })
        .collect::<Vec<_>>();
    json!({
        "delay_model": "project_trellis_speed_grade_min_max_ps",
        "net_delays": net_delays,
        "net_setup_slacks": net_setup_slacks,
        "setup_checks": setup_checks,
        "hold_checks": hold_checks,
        "worst_slack_ps": result.timing.worst_slack_ps,
        "worst_hold_slack_ps": result.timing.worst_hold_slack_ps,
        "met_timing": result.timing.met_timing(),
    })
}

fn checkpoint_evidence(evidence: &Evidence) -> Vec<&'static str> {
    [
        (Gate::RtlSimulation, "rtl_simulation"),
        (Gate::SynthesisEquivalence, "synthesis_equivalence"),
        (Gate::MappedNetlistComplete, "mapped_netlist_complete"),
        (Gate::PostMapSimulation, "post_map_simulation"),
        (Gate::PhysicalImplementation, "physical_implementation"),
        (Gate::TimingClosure, "timing_closure"),
    ]
    .into_iter()
    .filter_map(|(gate, name)| evidence.contains(gate).then_some(name))
    .collect()
}

fn primitive_metadata_json(
    cell: texo_model::CellId,
    metadata: &PrimitiveMetadata,
    design: &Design,
) -> Value {
    let configuration = match metadata {
        PrimitiveMetadata::Lut4 { init } => json!({ "kind": "lut4", "init": init }),
        PrimitiveMetadata::CarrySlice {
            init,
            inject,
            slice,
        } => json!({
            "kind": "carry_slice",
            "init": init,
            "inject": inject,
            "slice": slice,
        }),
        PrimitiveMetadata::FlipFlop {
            edge,
            enable,
            reset,
        } => json!({
            "kind": "flip_flop",
            "edge": clock_edge_name(*edge),
            "enable": enable.map(active_level_name),
            "reset": reset.as_ref().map(|reset| json!({
                "active": active_level_name(reset.active),
                "asynchronous": reset.asynchronous,
                "value": reset.value,
            })),
        }),
        PrimitiveMetadata::BlockRam {
            depth,
            word_width,
            physical_width,
            edge,
            write_enable,
            read_enable,
        } => json!({
            "kind": "block_ram",
            "depth": depth,
            "word_width": word_width,
            "physical_width": physical_width,
            "edge": clock_edge_name(*edge),
            "write_enable": active_level_name(*write_enable),
            "read_enable": read_enable.map(active_level_name),
        }),
        PrimitiveMetadata::Port {
            name,
            bit,
            direction,
        } => json!({
            "kind": "port",
            "name": name,
            "bit": bit,
            "direction": match direction {
                PortDirection::Input => "input",
                PortDirection::Output => "output",
            },
        }),
        PrimitiveMetadata::Constant { value } => {
            json!({ "kind": "constant", "value": value })
        }
    };
    json!({
        "cell_id": cell.0,
        "cell": design.cells()[cell.0].name,
        "configuration": configuration,
    })
}

const fn clock_edge_name(edge: ClockEdge) -> &'static str {
    match edge {
        ClockEdge::Rising => "rising",
        ClockEdge::Falling => "falling",
    }
}

const fn active_level_name(level: ActiveLevel) -> &'static str {
    match level {
        ActiveLevel::High => "high",
        ActiveLevel::Low => "low",
    }
}
