//! Stable JSON checkpoint serialization for implemented ECP5 designs.

use std::collections::BTreeMap;

use serde::ser::{SerializeMap, SerializeSeq};
use serde::{Serialize, Serializer};
use serde_json::{Value, json};
use texo_flow::{
    ECP5_PLACEMENT_ROUTABILITY_MODEL, ECP5_PLACEMENT_TIMING_WEIGHT_MODEL, Ecp5FlowResult,
    Ecp5InitialPlacementAlgorithm, Evidence, Gate,
};
use texo_model::{BelId, Design, PinDirection, PipId, ResourceKind, WireId};
use texo_struo::{ActiveLevel, ClockEdge, DistributedRamRole, PortDirection, PrimitiveMetadata};
use texo_target_ecp5::Ecp5Architecture;

/// Builds the stable, schema-versioned JSON representation of one ECP5 run.
///
/// # Panics
///
/// Panics only if an in-memory checkpoint record cannot be represented by
/// `serde_json::Value`.
#[must_use]
pub fn ecp5_checkpoint(
    design_name: &str,
    result: &Ecp5FlowResult,
    architecture: &Ecp5Architecture,
    package: &str,
    evidence: &Evidence,
) -> Value {
    serde_json::to_value(ecp5_checkpoint_ref(
        design_name,
        result,
        architecture,
        package,
        evidence,
    ))
    .expect("ECP5 checkpoint records are JSON-representable")
}

/// Borrowed schema-v3 checkpoint document that serializes without first
/// materializing the complete JSON value tree.
///
/// The emitted JSON is semantically identical to [`ecp5_checkpoint`]. This
/// form avoids cloning every architecture and design name and keeps only the
/// small block-RAM CIB-tie index alive while the document is written.
pub struct Ecp5CheckpointRef<'a> {
    design_name: &'a str,
    result: &'a Ecp5FlowResult,
    architecture: &'a Ecp5Architecture,
    package: &'a str,
    evidence: &'a Evidence,
    cib_ties: BTreeMap<WireId, Value>,
}

/// Builds a borrowed, streaming schema-v3 checkpoint document.
#[must_use]
pub fn ecp5_checkpoint_ref<'a>(
    design_name: &'a str,
    result: &'a Ecp5FlowResult,
    architecture: &'a Ecp5Architecture,
    package: &'a str,
    evidence: &'a Evidence,
) -> Ecp5CheckpointRef<'a> {
    let memory_bels = result
        .implementation
        .placement
        .bindings()
        .iter()
        .enumerate()
        .filter_map(|(cell, bel)| {
            (result.design.cells()[cell].kind == ResourceKind::Memory).then_some(*bel)
        });
    Ecp5CheckpointRef {
        design_name,
        result,
        architecture,
        package,
        evidence,
        cib_ties: cib_ties_for_bels(architecture, memory_bels),
    }
}

fn checkpoint_placement_model(
    algorithm: Ecp5InitialPlacementAlgorithm,
    weight_exponent: u32,
) -> Value {
    let timing_driven = algorithm.is_timing_driven_routability();
    json!({
        "initial_algorithm": algorithm.checkpoint_name(),
        "timing_weight_model": timing_driven.then_some(ECP5_PLACEMENT_TIMING_WEIGHT_MODEL),
        "criticality_exponent": timing_driven.then_some(weight_exponent),
        "routability_model": timing_driven.then_some(ECP5_PLACEMENT_ROUTABILITY_MODEL),
        "initial_predicted_detail": false,
    })
}

impl Serialize for Ecp5CheckpointRef<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let device = self.architecture.device();
        let mut checkpoint = serializer.serialize_map(Some(11))?;
        checkpoint.serialize_entry("absorbed_inputs", &checkpoint_absorbed_inputs(self.result))?;
        checkpoint.serialize_entry("design", self.design_name)?;
        checkpoint.serialize_entry("evidence", &checkpoint_evidence(self.evidence))?;
        checkpoint.serialize_entry(
            "metrics",
            &json!({
                "cells": self.result.design.cells().len(),
                "nets": self.result.design.nets().len(),
                "routed_nets": self.result.implementation.routes.len(),
                "total_pips": self.result.implementation.total_pips,
            }),
        )?;
        checkpoint.serialize_entry("packing", &checkpoint_packing(self.result))?;
        checkpoint.serialize_entry(
            "placement",
            &PlacementRecords {
                result: self.result,
                architecture: self.architecture,
                cib_ties: &self.cib_ties,
            },
        )?;
        checkpoint.serialize_entry(
            "primitive_metadata",
            &self
                .result
                .primitive_metadata
                .iter()
                .map(|(cell, metadata)| {
                    primitive_metadata_json(*cell, metadata, &self.result.design)
                })
                .collect::<Vec<_>>(),
        )?;
        checkpoint.serialize_entry(
            "routes",
            &RouteRecords {
                result: self.result,
                architecture: self.architecture,
            },
        )?;
        checkpoint.serialize_entry("schema_version", &3)?;
        checkpoint.serialize_entry(
            "target",
            &json!({
                "family": "ECP5",
                "device": device.name(),
                "package": self.package,
                "speed_grade": self.result.speed_grade,
                "placement_weight_exponent": self.result.placement_weight_exponent,
                "placement_model": checkpoint_placement_model(
                    self.result.initial_placement_algorithm,
                    self.result.placement_weight_exponent,
                ),
                "project_trellis_revision": self.architecture.provenance().project_trellis_revision,
                "database_revision": self.architecture.provenance().database_revision,
            }),
        )?;
        checkpoint.serialize_entry("timing", &TimingRecord(self.result))?;
        checkpoint.end()
    }
}

struct PlacementRecords<'a> {
    result: &'a Ecp5FlowResult,
    architecture: &'a Ecp5Architecture,
    cib_ties: &'a BTreeMap<WireId, Value>,
}

impl Serialize for PlacementRecords<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let bindings = self.result.implementation.placement.bindings();
        let mut records = serializer.serialize_seq(Some(bindings.len()))?;
        for cell_id in 0..bindings.len() {
            records.serialize_element(&PlacementRecord {
                result: self.result,
                architecture: self.architecture,
                cib_ties: self.cib_ties,
                cell_id,
            })?;
        }
        records.end()
    }
}

struct PlacementRecord<'a> {
    result: &'a Ecp5FlowResult,
    architecture: &'a Ecp5Architecture,
    cib_ties: &'a BTreeMap<WireId, Value>,
    cell_id: usize,
}

impl Serialize for PlacementRecord<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let cell = &self.result.design.cells()[self.cell_id];
        let bel_id = self.result.implementation.placement.bindings()[self.cell_id];
        let bel = &self.architecture.device().bels()[bel_id.0];
        let metadata = self.architecture.bel_metadata(bel_id);
        let mut record = serializer.serialize_map(Some(11))?;
        record.serialize_entry("bel", &bel.name)?;
        record.serialize_entry("bel_id", &bel_id.0)?;
        record.serialize_entry(
            "bel_pins",
            &BelPinRecords {
                architecture: self.architecture,
                cib_ties: self.cib_ties,
                bel: bel_id,
            },
        )?;
        record.serialize_entry("bel_type", metadata.bel_type)?;
        record.serialize_entry("bel_z", &metadata.z)?;
        record.serialize_entry("cell", &cell.name)?;
        record.serialize_entry("cell_id", &self.cell_id)?;
        record.serialize_entry(
            "configuration_tiles",
            &ConfigurationTileRecords {
                architecture: self.architecture,
                bel: bel_id,
            },
        )?;
        record.serialize_entry("kind", checkpoint_resource_kind(cell.kind))?;
        record.serialize_entry("x", &bel.point.x)?;
        record.serialize_entry("y", &bel.point.y)?;
        record.end()
    }
}

struct BelPinRecords<'a> {
    architecture: &'a Ecp5Architecture,
    cib_ties: &'a BTreeMap<WireId, Value>,
    bel: BelId,
}

impl Serialize for BelPinRecords<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let device = self.architecture.device();
        let bel = &device.bels()[self.bel.0];
        let mut records = serializer.serialize_seq(Some(bel.pins().len()))?;
        for pin_id in bel.pins() {
            let pin = &device.bel_pins()[pin_id.0];
            records.serialize_element(&BelPinRecord {
                name: &pin.name,
                direction: match pin.direction {
                    PinDirection::Input => "input",
                    PinDirection::Output => "output",
                    PinDirection::Inout => "inout",
                },
                wire_id: pin.wire.0,
                wire: &device.wires()[pin.wire.0].name,
                cib_tie: self.cib_ties.get(&pin.wire),
            })?;
        }
        records.end()
    }
}

#[derive(Serialize)]
struct BelPinRecord<'a> {
    cib_tie: Option<&'a Value>,
    direction: &'static str,
    name: &'a str,
    wire: &'a str,
    wire_id: usize,
}

struct ConfigurationTileRecords<'a> {
    architecture: &'a Ecp5Architecture,
    bel: BelId,
}

impl Serialize for ConfigurationTileRecords<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let point = self.architecture.device().bels()[self.bel.0].point;
        let tiles = self
            .architecture
            .configuration_tiles(point)
            .collect::<Vec<_>>();
        let mut records = serializer.serialize_seq(Some(tiles.len()))?;
        for (name, tile_type) in tiles {
            records.serialize_element(&ConfigurationTileRecord { name, tile_type })?;
        }
        records.end()
    }
}

#[derive(Serialize)]
struct ConfigurationTileRecord<'a> {
    name: &'a str,
    tile_type: &'a str,
}

struct RouteRecords<'a> {
    result: &'a Ecp5FlowResult,
    architecture: &'a Ecp5Architecture,
}

impl Serialize for RouteRecords<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let routes = &self.result.implementation.routes;
        let mut records = serializer.serialize_seq(Some(routes.len()))?;
        for route_index in 0..routes.len() {
            records.serialize_element(&RouteRecord {
                result: self.result,
                architecture: self.architecture,
                route_index,
            })?;
        }
        records.end()
    }
}

struct RouteRecord<'a> {
    result: &'a Ecp5FlowResult,
    architecture: &'a Ecp5Architecture,
    route_index: usize,
}

impl Serialize for RouteRecord<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let device = self.architecture.device();
        let route = &self.result.implementation.routes[self.route_index];
        let net = &self.result.design.nets()[route.net.0];
        let driver_pin = &self.result.design.pins()[net.driver.0];
        let driver_bel = self.result.implementation.placement.bindings()[driver_pin.cell.0];
        let driver_bel_pin = self
            .result
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
        let mut record = serializer.serialize_map(Some(6))?;
        record.serialize_entry("driver_wire", &device.wires()[driver_wire.0].name)?;
        record.serialize_entry("driver_wire_id", &driver_wire.0)?;
        record.serialize_entry("net", &net.name)?;
        record.serialize_entry("net_id", &route.net.0)?;
        record.serialize_entry(
            "pips",
            &RoutePipRecords {
                result: self.result,
                architecture: self.architecture,
                route_index: self.route_index,
            },
        )?;
        record.serialize_entry(
            "wires",
            &RouteWireRecords {
                result: self.result,
                architecture: self.architecture,
                route_index: self.route_index,
            },
        )?;
        record.end()
    }
}

struct RouteWireRecords<'a> {
    result: &'a Ecp5FlowResult,
    architecture: &'a Ecp5Architecture,
    route_index: usize,
}

impl Serialize for RouteWireRecords<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let route = &self.result.implementation.routes[self.route_index];
        let device = self.architecture.device();
        let mut records = serializer.serialize_seq(Some(route.wires().count()))?;
        for wire in route.wires() {
            records.serialize_element(&RouteWireRecord {
                id: wire.0,
                name: &device.wires()[wire.0].name,
            })?;
        }
        records.end()
    }
}

#[derive(Serialize)]
struct RouteWireRecord<'a> {
    #[serde(rename = "wire")]
    name: &'a str,
    #[serde(rename = "wire_id")]
    id: usize,
}

struct RoutePipRecords<'a> {
    result: &'a Ecp5FlowResult,
    architecture: &'a Ecp5Architecture,
    route_index: usize,
}

impl Serialize for RoutePipRecords<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let route = &self.result.implementation.routes[self.route_index];
        let mut records = serializer.serialize_seq(Some(route.pips().count()))?;
        for pip in route.pips() {
            records.serialize_element(&RoutePipRecord {
                architecture: self.architecture,
                pip,
            })?;
        }
        records.end()
    }
}

struct RoutePipRecord<'a> {
    architecture: &'a Ecp5Architecture,
    pip: PipId,
}

impl Serialize for RoutePipRecord<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let device = self.architecture.device();
        let pip = &device.pips()[self.pip.0];
        let metadata = self.architecture.pip_metadata(self.pip);
        let mut record = serializer.serialize_map(Some(10))?;
        record.serialize_entry("bidirectional", &pip.bidirectional())?;
        record.serialize_entry("config_tile", &metadata.config_tile)?;
        record.serialize_entry("fixed", &metadata.fixed)?;
        record.serialize_entry("from", &device.wires()[pip.from().0].name)?;
        record.serialize_entry("from_wire_id", &pip.from().0)?;
        record.serialize_entry("lutperm_flags", &metadata.lutperm_flags)?;
        record.serialize_entry("pip_id", &self.pip.0)?;
        record.serialize_entry("tile_type", metadata.tile_type)?;
        record.serialize_entry("to", &device.wires()[pip.to().0].name)?;
        record.serialize_entry("to_wire_id", &pip.to().0)?;
        record.end()
    }
}

struct TimingRecord<'a>(&'a Ecp5FlowResult);

impl Serialize for TimingRecord<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let result = self.0;
        let mut timing = serializer.serialize_map(Some(12))?;
        timing.serialize_entry(
            "all_modeled_endpoints_checked",
            &result.timing.all_modeled_endpoints_checked(),
        )?;
        timing.serialize_entry("delay_model", "nextpnr_ecp5_project_trellis_min_max_ps")?;
        timing.serialize_entry("hold_checks", &HoldCheckRecords(result))?;
        timing.serialize_entry("met_timing", &result.timing.met_timing())?;
        timing.serialize_entry(
            "modeled_endpoint_count",
            &result.timing.modeled_endpoint_count(),
        )?;
        timing.serialize_entry("net_delays", &NetDelayRecords(result))?;
        timing.serialize_entry(
            "net_setup_criticalities",
            &NetSetupCriticalityRecords(result),
        )?;
        timing.serialize_entry("net_setup_slacks", &NetSetupSlackRecords(result))?;
        timing.serialize_entry("setup_checks", &SetupCheckRecords(result))?;
        timing.serialize_entry("unchecked_endpoints", &UncheckedEndpointRecords(result))?;
        timing.serialize_entry("worst_hold_slack_ps", &result.timing.worst_hold_slack_ps)?;
        timing.serialize_entry("worst_slack_ps", &result.timing.worst_slack_ps)?;
        timing.end()
    }
}

struct NetDelayRecords<'a>(&'a Ecp5FlowResult);

impl Serialize for NetDelayRecords<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let result = self.0;
        let mut records = serializer.serialize_seq(Some(result.timing.net_delays.len()))?;
        for delay in &result.timing.net_delays {
            let net = &result.design.nets()[delay.net.0];
            let driver = &result.design.pins()[net.driver.0];
            let sink = &result.design.pins()[delay.sink.0];
            records.serialize_element(&NetDelayRecord {
                net_id: delay.net.0,
                net: &net.name,
                driver_pin_id: net.driver.0,
                driver_pin: &driver.name,
                driver_cell_id: driver.cell.0,
                driver_cell: &result.design.cells()[driver.cell.0].name,
                sink_pin_id: delay.sink.0,
                sink_pin: &sink.name,
                sink_cell_id: sink.cell.0,
                sink_cell: &result.design.cells()[sink.cell.0].name,
                min_delay_ps: delay.delay.min_ps,
                max_delay_ps: delay.delay.max_ps,
            })?;
        }
        records.end()
    }
}

#[derive(Serialize)]
struct NetDelayRecord<'a> {
    driver_cell: &'a str,
    driver_cell_id: usize,
    driver_pin: &'a str,
    driver_pin_id: usize,
    max_delay_ps: u64,
    min_delay_ps: u64,
    net: &'a str,
    net_id: usize,
    sink_cell: &'a str,
    sink_cell_id: usize,
    sink_pin: &'a str,
    sink_pin_id: usize,
}

struct NetSetupSlackRecords<'a>(&'a Ecp5FlowResult);

impl Serialize for NetSetupSlackRecords<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let result = self.0;
        let mut records = serializer.serialize_seq(Some(result.timing.net_setup_slacks.len()))?;
        for edge in &result.timing.net_setup_slacks {
            records.serialize_element(&NetSetupSlackRecord {
                net_id: edge.net.0,
                net: &result.design.nets()[edge.net.0].name,
                sink_pin_id: edge.sink.0,
                slack_ps: edge.slack_ps,
            })?;
        }
        records.end()
    }
}

#[derive(Serialize)]
struct NetSetupSlackRecord<'a> {
    net: &'a str,
    net_id: usize,
    sink_pin_id: usize,
    slack_ps: i128,
}

struct NetSetupCriticalityRecords<'a>(&'a Ecp5FlowResult);

impl Serialize for NetSetupCriticalityRecords<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let result = self.0;
        let mut records =
            serializer.serialize_seq(Some(result.timing.net_setup_criticalities.len()))?;
        for edge in &result.timing.net_setup_criticalities {
            records.serialize_element(&NetSetupCriticalityRecord {
                net_id: edge.net.0,
                net: &result.design.nets()[edge.net.0].name,
                sink_pin_id: edge.sink.0,
                path_delay_ps: edge.path_delay_ps,
                domain_worst_path_delay_ps: edge.domain_worst_path_delay_ps,
            })?;
        }
        records.end()
    }
}

#[derive(Serialize)]
struct NetSetupCriticalityRecord<'a> {
    domain_worst_path_delay_ps: u128,
    net: &'a str,
    net_id: usize,
    path_delay_ps: u128,
    sink_pin_id: usize,
}

struct SetupCheckRecords<'a>(&'a Ecp5FlowResult);

impl Serialize for SetupCheckRecords<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let result = self.0;
        let mut records = serializer.serialize_seq(Some(result.timing.setup_checks.len()))?;
        for check in &result.timing.setup_checks {
            records.serialize_element(&SetupCheckRecord {
                cell_id: check.cell.0,
                cell: &result.design.cells()[check.cell.0].name,
                data_pin_id: check.data_pin.0,
                clock_net_id: check.clock_net.0,
                launch_edge: check.launch_edge.as_str(),
                capture_edge: check.capture_edge.as_str(),
                arrival_ps: check.arrival_ps,
                clock_arrival_ps: check.clock_arrival_ps,
                setup_ps: check.setup_ps,
                uncertainty_ps: check.uncertainty_ps,
                required_ps: check.required_ps,
                slack_ps: check.slack_ps,
            })?;
        }
        records.end()
    }
}

#[derive(Serialize)]
struct SetupCheckRecord<'a> {
    arrival_ps: u64,
    capture_edge: &'static str,
    cell: &'a str,
    cell_id: usize,
    clock_arrival_ps: u64,
    clock_net_id: usize,
    data_pin_id: usize,
    launch_edge: &'static str,
    required_ps: i128,
    setup_ps: u64,
    slack_ps: i128,
    uncertainty_ps: u64,
}

struct HoldCheckRecords<'a>(&'a Ecp5FlowResult);

impl Serialize for HoldCheckRecords<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let result = self.0;
        let mut records = serializer.serialize_seq(Some(result.timing.hold_checks.len()))?;
        for check in &result.timing.hold_checks {
            records.serialize_element(&HoldCheckRecord {
                cell_id: check.cell.0,
                cell: &result.design.cells()[check.cell.0].name,
                data_pin_id: check.data_pin.0,
                clock_net_id: check.clock_net.0,
                launch_edge: check.launch_edge.as_str(),
                capture_edge: check.capture_edge.as_str(),
                arrival_ps: check.arrival_ps,
                clock_arrival_ps: check.clock_arrival_ps,
                hold_ps: check.hold_ps,
                required_ps: check.required_ps,
                slack_ps: check.slack_ps,
            })?;
        }
        records.end()
    }
}

#[derive(Serialize)]
struct HoldCheckRecord<'a> {
    arrival_ps: u64,
    capture_edge: &'static str,
    cell: &'a str,
    cell_id: usize,
    clock_arrival_ps: u64,
    clock_net_id: usize,
    data_pin_id: usize,
    hold_ps: u64,
    launch_edge: &'static str,
    required_ps: i128,
    slack_ps: i128,
}

struct UncheckedEndpointRecords<'a>(&'a Ecp5FlowResult);

impl Serialize for UncheckedEndpointRecords<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let result = self.0;
        let mut records =
            serializer.serialize_seq(Some(result.timing.unchecked_endpoints.len()))?;
        for endpoint in &result.timing.unchecked_endpoints {
            records.serialize_element(&UncheckedEndpointRecord {
                cell_id: endpoint.cell.0,
                cell: &result.design.cells()[endpoint.cell.0].name,
                data_pin_id: endpoint.data_pin.0,
                clock_pin_id: endpoint.clock_pin.0,
                clock_net_id: endpoint.clock_net.map(|net| net.0),
                reason: endpoint.reason.as_str(),
            })?;
        }
        records.end()
    }
}

#[derive(Serialize)]
struct UncheckedEndpointRecord<'a> {
    cell: &'a str,
    cell_id: usize,
    clock_net_id: Option<usize>,
    clock_pin_id: usize,
    data_pin_id: usize,
    reason: &'static str,
}

fn cib_ties_for_bels(
    architecture: &Ecp5Architecture,
    bels: impl IntoIterator<Item = BelId>,
) -> BTreeMap<WireId, Value> {
    let device = architecture.device();
    let mut targets = vec![false; device.wires().len()];
    let mut target_count = 0_usize;
    for wire in bels
        .into_iter()
        .flat_map(|bel| device.bels()[bel.0].pins().iter().copied())
        .filter_map(|pin| {
            let pin = &device.bel_pins()[pin.0];
            (pin.direction == PinDirection::Input).then_some(pin.wire)
        })
    {
        if !targets[wire.0] {
            targets[wire.0] = true;
            target_count += 1;
        }
    }
    if target_count == 0 {
        return BTreeMap::new();
    }
    let mut ties = BTreeMap::new();
    for (index, pip) in device.pips().iter().enumerate() {
        if !targets[pip.to().0] || !architecture.pip_metadata(PipId(index)).fixed {
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
    let distributed_rams = checkpoint_distributed_rams(result);
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
        "distributed_rams": distributed_rams,
        "global_clocks": global_clocks,
        "io_attributes": io_attributes,
        "clock_frequencies_hz": clock_frequencies_hz,
        "generated_clock_periods_ps": generated_clock_periods_ps,
        "unsupported_lpf_commands": result.packing.unsupported_lpf_commands(),
    })
}

fn checkpoint_distributed_rams(result: &Ecp5FlowResult) -> Vec<Value> {
    result
        .packing
        .distributed_rams()
        .iter()
        .map(|ram| {
            json!({
                "data": ram.data.map(|cell| cell.0),
                "blockers": ram.blockers.map(|cell| cell.0),
                "write_port": ram.write_port.0,
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
            second_port,
        } => json!({
            "kind": "block_ram",
            "depth": depth,
            "word_width": word_width,
            "physical_width": physical_width,
            "edge": clock_edge_name(*edge),
            "write_enable": active_level_name(*write_enable),
            "read_enable": read_enable.map(active_level_name),
            "second_port": second_port.map(|port| json!({
                "edge": clock_edge_name(port.edge),
                "write_enable": active_level_name(port.write_enable),
                "read_enable": port.read_enable.map(active_level_name),
            })),
        }),
        PrimitiveMetadata::DistributedRam {
            role,
            edge,
            write_enable,
        } => distributed_ram_metadata_json(*role, *edge, *write_enable),
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

fn distributed_ram_metadata_json(
    role: DistributedRamRole,
    edge: ClockEdge,
    write_enable: ActiveLevel,
) -> Value {
    json!({
        "kind": match role {
            DistributedRamRole::Data(_) => "distributed_ram_data",
            DistributedRamRole::WritePort => "distributed_ram_write_port",
            DistributedRamRole::WriteBlocker => "distributed_ram_blocker",
        },
        "bit": match role {
            DistributedRamRole::Data(bit) => Some(bit),
            DistributedRamRole::WritePort | DistributedRamRole::WriteBlocker => None,
        },
        "edge": clock_edge_name(edge),
        "write_enable": active_level_name(write_enable),
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

    use serde_json::json;
    use texo_flow::{
        ECP5_PLACEMENT_ROUTABILITY_MODEL, ECP5_PLACEMENT_TIMING_WEIGHT_MODEL,
        Ecp5InitialPlacementAlgorithm,
    };
    use texo_model::{CellId, Design, PinDirection, ResourceKind};
    use texo_struo::{
        ActiveLevel, ClockEdge, DistributedRamRole, PllOutput, PortDirection, PrimitiveMetadata,
    };
    use texo_target_ecp5::{
        ArchitectureFile, PipRecord, RelativeRef, TileRecord, WireRecord, expand,
    };

    use super::{checkpoint_placement_model, cib_ties_for_bels, primitive_metadata_json};

    const ARCHITECTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../texo-target-ecp5/fixtures/minimal-ecp5.json"
    ));

    #[test]
    fn checkpoints_distributed_ram_cell_roles() {
        let mut design = Design::new();
        let data = design.add_cell("ram$data0", ResourceKind::Lut(4));
        assert_eq!(
            primitive_metadata_json(
                data,
                &PrimitiveMetadata::DistributedRam {
                    role: DistributedRamRole::Data(0),
                    edge: ClockEdge::Falling,
                    write_enable: ActiveLevel::Low,
                },
                &design,
            )["configuration"],
            json!({
                "kind": "distributed_ram_data",
                "bit": 0,
                "edge": "falling",
                "write_enable": "low",
            })
        );
    }

    #[test]
    fn checkpoints_the_effective_timing_driven_placement_model() {
        assert_eq!(
            checkpoint_placement_model(
                Ecp5InitialPlacementAlgorithm::TimingDrivenRoutabilityElectrostatic,
                4,
            ),
            json!({
                "initial_algorithm": "ecp5_timing_routability_electrostatic_v1",
                "timing_weight_model": ECP5_PLACEMENT_TIMING_WEIGHT_MODEL,
                "criticality_exponent": 4,
                "routability_model": ECP5_PLACEMENT_ROUTABILITY_MODEL,
                "initial_predicted_detail": false,
            }),
        );
        assert_eq!(
            checkpoint_placement_model(Ecp5InitialPlacementAlgorithm::Imported, 4),
            json!({
                "initial_algorithm": "imported_v1",
                "timing_weight_model": null,
                "criticality_exponent": null,
                "routability_model": null,
                "initial_predicted_detail": false,
            }),
        );
    }

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
