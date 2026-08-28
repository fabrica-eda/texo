//! Stable JSON checkpoint serialization for implemented ECP5 designs.

use std::collections::{BTreeMap, BTreeSet};

use serde_json::{Value, json};
use texo_flow::{Ecp5FlowResult, Evidence, Gate};
use texo_model::{BelId, Design, PinDirection, PipId, ResourceKind, WireId};
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
        "schema_version": 3,
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
    let memory_bels = result
        .implementation
        .placement
        .bindings()
        .iter()
        .enumerate()
        .filter_map(|(cell, bel)| {
            (result.design.cells()[cell].kind == ResourceKind::Memory).then_some(*bel)
        });
    let cib_ties = cib_ties_for_bels(architecture, memory_bels);
    result
        .implementation
        .placement
        .bindings()
        .iter()
        .enumerate()
        .map(|(cell_id, bel_id)| {
            let cell = &result.design.cells()[cell_id];
            let bel = &architecture.device().bels()[bel_id.0];
            let metadata = architecture.bel_metadata(*bel_id);
            let bel_pins = bel
                .pins()
                .iter()
                .map(|pin_id| {
                    let pin = &architecture.device().bel_pins()[pin_id.0];
                    json!({
                        "name": pin.name,
                        "direction": match pin.direction {
                            texo_model::PinDirection::Input => "input",
                            texo_model::PinDirection::Output => "output",
                            texo_model::PinDirection::Inout => "inout",
                        },
                        "wire_id": pin.wire.0,
                        "wire": architecture.device().wires()[pin.wire.0].name,
                        "cib_tie": cib_ties.get(&pin.wire),
                    })
                })
                .collect::<Vec<_>>();
            let configuration_tiles = architecture
                .configuration_tiles(bel.point)
                .map(|(name, tile_type)| json!({ "name": name, "tile_type": tile_type }))
                .collect::<Vec<_>>();
            json!({
                "cell_id": cell_id,
                "cell": cell.name,
                "kind": checkpoint_resource_kind(cell.kind),
                "bel_id": bel_id.0,
                "bel": bel.name,
                "bel_type": metadata.bel_type,
                "bel_z": metadata.z,
                "bel_pins": bel_pins,
                "configuration_tiles": configuration_tiles,
                "x": bel.point.x,
                "y": bel.point.y,
            })
        })
        .collect()
}

fn cib_ties_for_bels(
    architecture: &Ecp5Architecture,
    bels: impl IntoIterator<Item = BelId>,
) -> BTreeMap<WireId, Value> {
    let device = architecture.device();
    let targets = bels
        .into_iter()
        .flat_map(|bel| device.bels()[bel.0].pins().iter().copied())
        .filter_map(|pin| {
            let pin = &device.bel_pins()[pin.0];
            (pin.direction == PinDirection::Input).then_some(pin.wire)
        })
        .collect::<BTreeSet<_>>();
    if targets.is_empty() {
        return BTreeMap::new();
    }
    let mut ties = BTreeMap::new();
    for (index, pip) in device.pips().iter().enumerate() {
        if !targets.contains(&pip.to()) || !architecture.pip_metadata(PipId(index)).fixed {
            continue;
        }
        let source = &device.wires()[pip.from().0];
        let Some(mux) = source.name.rsplit_once('/').map(|(_, basename)| basename) else {
            continue;
        };
        if !is_cib_tie_mux(mux) {
            continue;
        }
        let mut configuration_tiles = architecture
            .configuration_tiles(source.point)
            .filter(|(_, tile_type)| tile_type.starts_with("CIB") || tile_type.starts_with("VCIB"));
        let Some((tile, _)) = configuration_tiles.next() else {
            continue;
        };
        if configuration_tiles.next().is_some() {
            continue;
        }
        ties.insert(
            pip.to(),
            json!({
                "tile": tile,
                "mux": mux,
            }),
        );
    }
    ties
}

fn is_cib_tie_mux(name: &str) -> bool {
    let Some(index) = name.as_bytes().last().copied() else {
        return false;
    };
    index.is_ascii_digit()
        && index <= b'7'
        && ["JA", "JB", "JC", "JD", "JCE", "JLSR", "JCLK"]
            .iter()
            .any(|prefix| {
                name.strip_prefix(prefix)
                    .is_some_and(|suffix| suffix.len() == 1)
            })
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
                    let metadata = architecture.pip_metadata(pip_id);
                    json!({
                        "pip_id": pip_id.0,
                        "from_wire_id": pip.from().0,
                        "from": device.wires()[pip.from().0].name,
                        "to_wire_id": pip.to().0,
                        "to": device.wires()[pip.to().0].name,
                        "bidirectional": pip.bidirectional(),
                        "fixed": metadata.fixed,
                        "config_tile": metadata.config_tile,
                        "tile_type": metadata.tile_type,
                        "lutperm_flags": metadata.lutperm_flags,
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
    let wide_lut_clusters = result
        .packing
        .wide_lut_clusters()
        .iter()
        .map(|cluster| cluster.iter().map(|cell| cell.0).collect::<Vec<_>>())
        .collect::<Vec<_>>();
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
    let generated_clock_periods_ps = result
        .packing
        .generated_clock_periods_ps()
        .iter()
        .map(|(net, period_ps)| {
            json!({
                "net_id": net.0,
                "net": result.design.nets()[net.0].name,
                "period_ps": period_ps,
            })
        })
        .collect::<Vec<_>>();
    json!({
        "wide_lut_clusters": wide_lut_clusters,
        "lut_ff_pairs": lut_ff_pairs,
        "carry_pairs": carry_pairs,
        "general_routing_ffs": result.packing.general_routing_ffs().iter().map(|cell| cell.0).collect::<Vec<_>>(),
        "block_rams": block_rams,
        "global_clocks": global_clocks,
        "io_attributes": io_attributes,
        "clock_frequencies_hz": clock_frequencies_hz,
        "generated_clock_periods_ps": generated_clock_periods_ps,
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
                "uncertainty_ps": check.uncertainty_ps,
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
    let unchecked_endpoints = checkpoint_unchecked_endpoints(result);
    json!({
        "delay_model": "project_trellis_speed_grade_min_max_ps_with_setup_uncertainty",
        "net_delays": net_delays,
        "net_setup_slacks": net_setup_slacks,
        "setup_checks": setup_checks,
        "hold_checks": hold_checks,
        "unchecked_endpoints": unchecked_endpoints,
        "modeled_endpoint_count": result.timing.modeled_endpoint_count(),
        "all_modeled_endpoints_checked": result.timing.all_modeled_endpoints_checked(),
        "worst_slack_ps": result.timing.worst_slack_ps,
        "worst_hold_slack_ps": result.timing.worst_hold_slack_ps,
        "met_timing": result.timing.met_timing(),
    })
}

fn checkpoint_unchecked_endpoints(result: &Ecp5FlowResult) -> Vec<Value> {
    result
        .timing
        .unchecked_endpoints
        .iter()
        .map(|endpoint| {
            json!({
                "cell_id": endpoint.cell.0,
                "cell": result.design.cells()[endpoint.cell.0].name,
                "data_pin_id": endpoint.data_pin.0,
                "clock_pin_id": endpoint.clock_pin.0,
                "clock_net_id": endpoint.clock_net.map(|net| net.0),
                "reason": endpoint.reason.as_str(),
            })
        })
        .collect()
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
        PrimitiveMetadata::Jtagg {
            extension_register_1,
            extension_register_2,
        } => json!({
            "kind": "jtagg",
            "extension_register_1": extension_register_1,
            "extension_register_2": extension_register_2,
        }),
        PrimitiveMetadata::Pll {
            fabric_output,
            feedback_output,
            parameters,
            attributes,
        } => json!({
            "kind": "pll",
            "fabric_output": fabric_output.port(),
            "feedback_output": feedback_output.port(),
            "parameters": parameters,
            "attributes": attributes,
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
                PortDirection::Inout => "inout",
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

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use texo_model::{CellId, Design, PinDirection, ResourceKind};
    use texo_struo::{PllOutput, PortDirection, PrimitiveMetadata};
    use texo_target_ecp5::{
        ArchitectureFile, PipRecord, RelativeRef, TileRecord, WireRecord, expand,
    };

    use super::{cib_ties_for_bels, primitive_metadata_json};

    const ARCHITECTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../texo-target-ecp5/fixtures/minimal-ecp5.json"
    ));

    #[test]
    fn checkpoints_bidirectional_port_direction() {
        let mut design = Design::new();
        let cell = design.add_cell("$sda[0]", ResourceKind::Io);
        let configuration = primitive_metadata_json(
            cell,
            &PrimitiveMetadata::Port {
                name: "sda".into(),
                bit: 0,
                direction: PortDirection::Inout,
            },
            &design,
        );

        assert_eq!(cell, CellId(0));
        assert_eq!(configuration["configuration"]["direction"], "inout");
    }

    #[test]
    fn checkpoints_jtagg_extension_registers() {
        let mut design = Design::new();
        let cell = design.add_cell("jtagg", ResourceKind::Logic);
        let configuration = primitive_metadata_json(
            cell,
            &PrimitiveMetadata::Jtagg {
                extension_register_1: true,
                extension_register_2: false,
            },
            &design,
        );

        assert_eq!(configuration["configuration"]["kind"], "jtagg");
        assert_eq!(configuration["configuration"]["extension_register_1"], true);
        assert_eq!(
            configuration["configuration"]["extension_register_2"],
            false
        );
    }

    #[test]
    fn checkpoints_pll_configuration() {
        let mut design = Design::new();
        let cell = design.add_cell("pll", ResourceKind::Logic);
        let configuration = primitive_metadata_json(
            cell,
            &PrimitiveMetadata::Pll {
                fabric_output: PllOutput::Clkos,
                feedback_output: PllOutput::Clkop,
                parameters: BTreeMap::from([("CLKI_DIV".into(), "3".into())]),
                attributes: BTreeMap::from([("FREQUENCY_PIN_CLKOS".into(), "250".into())]),
            },
            &design,
        );

        assert_eq!(configuration["configuration"]["kind"], "pll");
        assert_eq!(configuration["configuration"]["fabric_output"], "CLKOS");
        assert_eq!(configuration["configuration"]["feedback_output"], "CLKOP");
        assert_eq!(
            configuration["configuration"]["parameters"]["CLKI_DIV"],
            "3"
        );
        assert_eq!(
            configuration["configuration"]["attributes"]["FREQUENCY_PIN_CLKOS"],
            "250"
        );
    }

    #[test]
    fn checkpoints_the_fixed_cib_tie_before_a_dp16kd_pin() {
        let mut file: ArchitectureFile = serde_json::from_str(ARCHITECTURE).unwrap();
        let source = file.location_types[0].wires.len();
        file.location_types[0].wires.push(WireRecord {
            name: "JCLK0".into(),
        });
        file.location_types[0].pips.push(PipRecord {
            from: RelativeRef {
                dx: 0,
                dy: 0,
                index: source,
            },
            to: RelativeRef {
                dx: 0,
                dy: 0,
                index: 15,
            },
            fixed: true,
            tile_type: "CIB_EBR".into(),
            timing_class: "zero".into(),
            lutperm_flags: 0,
        });
        file.locations[0].tiles.push(TileRecord {
            name: "CIB_R0C0:CIB_EBR".into(),
            tile_type: "CIB_EBR".into(),
        });
        let architecture = expand(file).unwrap();
        let device = architecture.device();
        let bel = device
            .bels()
            .iter()
            .enumerate()
            .find(|(index, bel)| {
                architecture
                    .bel_metadata(texo_model::BelId(*index))
                    .bel_type
                    == "DP16KD"
                    && bel.point.x == 0
            })
            .map(|(index, _)| texo_model::BelId(index))
            .unwrap();
        let clock = device.bels()[bel.0]
            .pins()
            .iter()
            .map(|pin| &device.bel_pins()[pin.0])
            .find(|pin| pin.name == "CLKA" && pin.direction == PinDirection::Input)
            .unwrap()
            .wire;

        let ties = cib_ties_for_bels(&architecture, [bel]);

        assert_eq!(ties[&clock]["tile"], "CIB_R0C0:CIB_EBR");
        assert_eq!(ties[&clock]["mux"], "JCLK0");
    }
}
