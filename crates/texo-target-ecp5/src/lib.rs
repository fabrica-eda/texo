//! Versioned Project Trellis architecture import for ECP5.
//!
//! Project Trellis exposes its routing graph through C++/Python. The companion
//! `tools/export_ecp5.py` script snapshots that graph into the schema defined
//! here. Runtime placement and routing then use only Rust and [`texo_model`].

mod lpf;

pub use lpf::{
    LogicalPort, LpfConstraints, LpfError, ResolvedLpf, parse_lpf, resolve_lpf_port_cells,
    resolve_lpf_ports,
};

use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::error::Error;
use std::fmt;
use std::io::{Read, Write};
use std::sync::Arc;

use serde::{Deserialize, Serialize};
use texo_model::{
    BelId, BelPinId, BufferSpec, CellId, CellPinId, Design, Device, ModelError, NetId,
    PinDirection, PipId, Point, ResourceKind, UnifiedGraph, WireId,
};
use texo_pnr::{NetRoute, Placement, PlacementConstraints, RoutingConstraints};

/// Current on-disk architecture schema version.
pub const SCHEMA_VERSION: u32 = 4;

/// Version of the expanded binary architecture cache.
pub const ARCHITECTURE_CACHE_VERSION: u32 = 2;

/// Provenance required for every generated architecture snapshot.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct Provenance {
    /// Project Trellis source revision used by the exporter.
    pub project_trellis_revision: String,
    /// `prjtrellis-db` submodule revision.
    pub database_revision: String,
    /// Whether LUT permutation arcs were included.
    pub include_lutperm_pips: bool,
    /// Whether monolithic slices were split into fine-grained BELs.
    pub split_slice_mode: bool,
}

/// Relative resource reference used by a deduplicated location type.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct RelativeRef {
    /// Horizontal offset from the location instance.
    pub dx: i32,
    /// Vertical offset from the location instance.
    pub dy: i32,
    /// Resource index within the referenced location type.
    pub index: usize,
}

/// Direction encoded by the architecture snapshot.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum PinDirectionRecord {
    /// BEL input.
    Input,
    /// BEL output.
    Output,
    /// Bidirectional BEL pin.
    Inout,
}

impl From<PinDirectionRecord> for PinDirection {
    fn from(value: PinDirectionRecord) -> Self {
        match value {
            PinDirectionRecord::Input => Self::Input,
            PinDirectionRecord::Output => Self::Output,
            PinDirectionRecord::Inout => Self::Inout,
        }
    }
}

/// One BEL pin in a deduplicated location type.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct BelPinRecord {
    /// Project Trellis pin name.
    pub name: String,
    /// Signal direction.
    pub direction: PinDirectionRecord,
    /// Wire reached by this pin.
    pub wire: RelativeRef,
}

/// One BEL in a deduplicated location type.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct BelRecord {
    /// Location-local BEL name.
    pub name: String,
    /// Project Trellis BEL type.
    pub bel_type: String,
    /// Z-order within the location.
    pub z: i32,
    /// Physical pin surface.
    pub pins: Vec<BelPinRecord>,
}

/// One routing wire in a deduplicated location type.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct WireRecord {
    /// Project Trellis wire name.
    pub name: String,
}

/// One directed Project Trellis routing arc.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct PipRecord {
    /// Source wire.
    pub from: RelativeRef,
    /// Destination wire.
    pub to: RelativeRef,
    /// Whether this is an always-connected rather than configurable arc.
    pub fixed: bool,
    /// Project Trellis tile type owning the configuration bit.
    pub tile_type: String,
    /// Project Trellis interconnect timing class.
    pub timing_class: String,
    /// LUT permutation metadata used during configuration generation.
    pub lutperm_flags: u16,
}

/// Deduplicated resource layout shared by compatible grid locations.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct LocationTypeRecord {
    /// Routing wires.
    pub wires: Vec<WireRecord>,
    /// Placeable BELs.
    pub bels: Vec<BelRecord>,
    /// Directed routing arcs.
    pub pips: Vec<PipRecord>,
}

/// One physical grid location and its deduplicated type.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct LocationRecord {
    /// Horizontal device coordinate.
    pub x: u32,
    /// Vertical device coordinate.
    pub y: u32,
    /// Index into [`ArchitectureFile::location_types`].
    pub location_type: usize,
    /// ECP5 global-clock quadrant, tap, and spine mapping at this location.
    pub global: GlobalInfoRecord,
}

/// One of the four ECP5 primary-clock quadrants.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum GlobalQuadrant {
    /// Upper-left quadrant.
    Ul,
    /// Upper-right quadrant.
    Ur,
    /// Lower-left quadrant.
    Ll,
    /// Lower-right quadrant.
    Lr,
}

impl GlobalQuadrant {
    fn wire_prefix(self) -> &'static str {
        match self {
            Self::Ul => "UL",
            Self::Ur => "UR",
            Self::Ll => "LL",
            Self::Lr => "LR",
        }
    }
}

/// Direction from a logic tile toward its ECP5 global-clock tap column.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum GlobalTapDirection {
    /// Use the left-going tap segment.
    Left,
    /// Use the right-going tap segment.
    Right,
}

/// Target-specific global-clock topology attached to one grid location.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct GlobalInfoRecord {
    /// Clock quadrant containing this location.
    pub quadrant: GlobalQuadrant,
    /// Horizontal tap-segment direction.
    pub tap_direction: GlobalTapDirection,
    /// Column containing the tap driver.
    pub tap_column: u32,
    /// Spine driver coordinate when this location is itself a tap column.
    pub spine: Option<Point>,
}

/// One package pin bound to a PIO BEL.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct PackagePinRecord {
    /// Package ball or lead name.
    pub name: String,
    /// PIO location X coordinate.
    pub x: u32,
    /// PIO location Y coordinate.
    pub y: u32,
    /// BEL index within the location type.
    pub bel: usize,
}

/// Package pin table.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct PackageRecord {
    /// Project Trellis package name.
    pub name: String,
    /// Available package pins.
    pub pins: Vec<PackagePinRecord>,
}

/// Minimum and maximum characterized delay in picoseconds.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct DelayRangeRecord {
    /// Earliest characterized delay.
    pub min_ps: u64,
    /// Latest characterized delay.
    pub max_ps: u64,
}

/// One combinational or clock-to-output cell arc.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct CellArcTimingRecord {
    /// Source pin name on the split BEL surface.
    pub from_pin: String,
    /// Destination pin name on the split BEL surface.
    pub to_pin: String,
    /// Characterized delay range.
    pub delay: DelayRangeRecord,
}

/// One sequential input timing check against a clock pin.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct SetupHoldTimingRecord {
    /// Data or control pin name on the split BEL surface.
    pub signal_pin: String,
    /// Associated clock pin name.
    pub clock_pin: String,
    /// Setup requirement range.
    pub setup: DelayRangeRecord,
    /// Hold requirement range.
    pub hold: DelayRangeRecord,
}

/// Timing arcs for one split ECP5 cell type.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct CellTimingRecord {
    /// Project Trellis/nextpnr split cell type.
    pub cell_type: String,
    /// Combinational and clock-to-output arcs.
    pub arcs: Vec<CellArcTimingRecord>,
    /// Sequential input checks.
    pub setup_holds: Vec<SetupHoldTimingRecord>,
}

/// Independently characterized Project Trellis timing corners.
///
/// Project Trellis solves these corners independently, so their numeric values
/// are not required to be monotonic.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct TimingCornersRecord {
    /// Value fitted from the minimum-delay SDF corner.
    pub min_ps: u64,
    /// Value fitted from the typical-delay SDF corner.
    pub typ_ps: u64,
    /// Value fitted from the maximum-delay SDF corner.
    pub max_ps: u64,
}

/// Delay coefficients for one interconnect timing class.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct PipClassTimingRecord {
    /// Fixed delay component at each characterized corner.
    pub base: TimingCornersRecord,
    /// Delay added per enabled PIP leaving the same source wire.
    pub fanout_adder: TimingCornersRecord,
}

/// Cell and interconnect timing for one ECP5 speed grade.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct SpeedGradeRecord {
    /// ECP5 speed-grade name such as `6`, `7`, `8`, or `8_5G`.
    pub name: String,
    /// Interconnect coefficients indexed by timing class.
    pub pip_classes: BTreeMap<String, PipClassTimingRecord>,
    /// Split-cell timing records.
    pub cells: Vec<CellTimingRecord>,
}

/// Versioned output of the Project Trellis exporter.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct ArchitectureFile {
    /// Must equal [`SCHEMA_VERSION`].
    pub schema_version: u32,
    /// Source and database revisions.
    pub provenance: Provenance,
    /// Expected to be `ECP5`.
    pub family: String,
    /// Exact Project Trellis device name.
    pub device: String,
    /// Number of routing-grid columns.
    pub width: u32,
    /// Number of routing-grid rows.
    pub height: u32,
    /// Deduplicated resource layouts.
    pub location_types: Vec<LocationTypeRecord>,
    /// Type assignment for each physical location.
    pub locations: Vec<LocationRecord>,
    /// Package databases available for this device.
    pub packages: Vec<PackageRecord>,
    /// Characterized timing tables by speed grade.
    pub speed_grades: Vec<SpeedGradeRecord>,
}

/// ECP5-specific properties attached to a generic BEL.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BelMetadata<'a> {
    /// Project Trellis BEL type.
    pub bel_type: &'a str,
    /// Z-order within its grid location.
    pub z: i32,
}

/// ECP5-specific properties attached to a generic PIP.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PipMetadata<'a> {
    /// Whether the arc is always connected.
    pub fixed: bool,
    /// Project Trellis tile type.
    pub tile_type: &'a str,
    /// Interconnect timing class resolved through a speed-grade table.
    pub timing_class: &'a str,
    /// LUT permutation flags.
    pub lutperm_flags: u16,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
struct CompactBelMetadata {
    bel_type: u32,
    z: i32,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
struct CompactPipMetadata {
    tile_type: u32,
    timing_class: u32,
    lutperm_flags: u16,
    fixed: bool,
}

/// Resolved package pin table.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct Package {
    /// Package name.
    pub name: String,
    /// Package pin to PIO BEL mapping.
    pub pins: BTreeMap<String, BelId>,
}

/// Expanded ECP5 device ready for Texo placement and routing.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct Ecp5Architecture {
    provenance: Provenance,
    device: Device,
    metadata_strings: Vec<String>,
    bel_metadata: Vec<CompactBelMetadata>,
    pip_metadata: Vec<CompactPipMetadata>,
    global_info: Vec<GlobalInfoRecord>,
    packages: Vec<Package>,
    speed_grades: BTreeMap<String, SpeedGradeRecord>,
}

/// Reusable reverse-routing indexes for ECP5 global-clock construction.
///
/// Large devices contain millions of wires and PIPs. Building these immutable
/// indexes once per placement candidate dominates timing-driven `PnR`, so callers
/// that evaluate multiple placements should retain one cache for the full run.
#[derive(Debug)]
pub struct Ecp5GlobalRoutingCache<'a> {
    incoming: CompactIncomingPips,
    unique_roots: HashMap<&'a str, Option<WireId>>,
    forward_routes: HashMap<(WireId, WireId), (Vec<WireId>, Vec<PipId>)>,
    reverse_routes: HashMap<(usize, WireId), GlobalClockBranch>,
    reverse_search: GlobalReverseSearch,
}

type GlobalClockBranch = (WireId, Vec<WireId>, Vec<PipId>);

/// Compressed sparse-row reverse PIP index. PIP IDs within each wire retain
/// device order, matching the former `Vec<Vec<PipId>>` traversal exactly.
#[derive(Debug)]
struct CompactIncomingPips {
    offsets: Vec<u32>,
    pips: Vec<u32>,
}

impl CompactIncomingPips {
    fn new(device: &Device) -> Self {
        let mut offsets = vec![0_u32; device.wires().len() + 1];
        for pip in device.pips() {
            offsets[pip.to().0 + 1] += 1;
        }
        for index in 1..offsets.len() {
            offsets[index] += offsets[index - 1];
        }
        let mut cursors = offsets.clone();
        let mut pips = vec![0_u32; device.pips().len()];
        for (index, pip) in device.pips().iter().enumerate() {
            let cursor = &mut cursors[pip.to().0];
            pips[*cursor as usize] =
                u32::try_from(index).expect("ECP5 PIP IDs fit compact u32 storage");
            *cursor += 1;
        }
        Self { offsets, pips }
    }

    fn for_wire(&self, wire: WireId) -> &[u32] {
        &self.pips[self.offsets[wire.0] as usize..self.offsets[wire.0 + 1] as usize]
    }
}

#[derive(Deserialize, Serialize)]
struct ArchitectureCache {
    version: u32,
    architecture: Ecp5Architecture,
}

#[derive(Serialize)]
struct ArchitectureCacheRef<'a> {
    version: u32,
    architecture: &'a Ecp5Architecture,
}

impl Ecp5Architecture {
    /// Snapshot provenance.
    #[must_use]
    pub const fn provenance(&self) -> &Provenance {
        &self.provenance
    }

    /// Generic physical device.
    #[must_use]
    pub const fn device(&self) -> &Device {
        &self.device
    }

    /// Target metadata for a BEL.
    ///
    /// # Panics
    ///
    /// Panics if `bel` does not belong to this architecture.
    #[must_use]
    pub fn bel_metadata(&self, bel: BelId) -> BelMetadata<'_> {
        let metadata = &self.bel_metadata[bel.0];
        BelMetadata {
            bel_type: self.metadata_string(metadata.bel_type),
            z: metadata.z,
        }
    }

    /// Target metadata for a routing arc.
    ///
    /// # Panics
    ///
    /// Panics if `pip` does not belong to this architecture.
    #[must_use]
    pub fn pip_metadata(&self, pip: PipId) -> PipMetadata<'_> {
        let metadata = &self.pip_metadata[pip.0];
        PipMetadata {
            fixed: metadata.fixed,
            tile_type: self.metadata_string(metadata.tile_type),
            timing_class: self.metadata_string(metadata.timing_class),
            lutperm_flags: metadata.lutperm_flags,
        }
    }

    /// BEL metadata in stable BEL ID order.
    pub fn bel_metadata_iter(&self) -> impl Iterator<Item = (BelId, BelMetadata<'_>)> + '_ {
        (0..self.bel_metadata.len()).map(|index| {
            let bel = BelId(index);
            (bel, self.bel_metadata(bel))
        })
    }

    /// PIP metadata in stable PIP ID order.
    pub fn pip_metadata_iter(&self) -> impl Iterator<Item = (PipId, PipMetadata<'_>)> + '_ {
        (0..self.pip_metadata.len()).map(|index| {
            let pip = PipId(index);
            (pip, self.pip_metadata(pip))
        })
    }

    /// Compact timing-class IDs in stable PIP ID order.
    ///
    /// IDs index this architecture's metadata dictionary and are intended for
    /// dense per-speed-grade tables. They avoid resolving and comparing a
    /// timing-class string for every one of the device's millions of PIPs.
    #[must_use]
    pub fn pip_timing_class_ids(&self) -> impl ExactSizeIterator<Item = u32> + '_ {
        self.pip_metadata
            .iter()
            .map(|metadata| metadata.timing_class)
    }

    /// Number of entries in the compact metadata string dictionary.
    #[must_use]
    pub const fn metadata_string_count(&self) -> usize {
        self.metadata_strings.len()
    }

    /// Resolves one compact metadata string ID.
    #[must_use]
    pub fn metadata_string_by_id(&self, id: u32) -> Option<&str> {
        self.metadata_strings.get(id as usize).map(String::as_str)
    }

    /// ECP5 global-clock topology at one device coordinate.
    #[must_use]
    pub fn global_info(&self, point: Point) -> Option<GlobalInfoRecord> {
        if point.x >= self.device.width() || point.y >= self.device.height() {
            return None;
        }
        self.global_info
            .get((point.y * self.device.width() + point.x) as usize)
            .copied()
    }

    /// Resolved package pin tables.
    #[must_use]
    pub fn packages(&self) -> &[Package] {
        &self.packages
    }

    /// Available speed-grade timing tables.
    #[must_use]
    pub const fn speed_grades(&self) -> &BTreeMap<String, SpeedGradeRecord> {
        &self.speed_grades
    }

    fn metadata_string(&self, id: u32) -> &str {
        &self.metadata_strings[id as usize]
    }
}

/// One LUT and FF selected for the ECP5 dedicated `F → DI` path.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LutFfPair {
    /// Driving LUT4 cell.
    pub lut: CellId,
    /// Driven register cell.
    pub ff: CellId,
}

/// Structural information required to pack one logical memory into `DP16KD`.
///
/// Struo's ECP5 mapper exposes these values as immutable primitive metadata;
/// keeping this type structural avoids coupling the target crate to one
/// particular frontend adapter.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BlockRamRequirement {
    /// Logical memory cell.
    pub cell: CellId,
    /// Logical number of words.
    pub depth: u32,
    /// Logical word width.
    pub word_width: u8,
    /// Width selected for the physical ECP5 RAM ports.
    pub physical_width: u8,
}

/// One legal `DP16KD` packing decision.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PackedBlockRam {
    /// Logical memory cell.
    pub cell: CellId,
    /// Stable ECP5 write-ID configuration value.
    pub wid: u32,
    /// Logical number of words.
    pub depth: u32,
    /// Logical word width.
    pub word_width: u8,
    /// Width selected for the physical ECP5 RAM ports.
    pub physical_width: u8,
}

/// One logical net selected for promotion onto an ECP5 global clock network.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct GlobalClockRequirement {
    /// Source net before DCCA insertion.
    pub net: NetId,
}

/// One inserted and legally constrained ECP5 DCCA clock buffer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PackedGlobalClock {
    /// Original source net driving `CLKI`.
    pub source_net: NetId,
    /// Inserted DCCA logical cell.
    pub buffer: CellId,
    /// New global net driven by `CLKO`.
    pub global_net: NetId,
}

/// nextpnr-compatible minimum clock-pin fanout for automatic promotion.
pub const DEFAULT_GLOBAL_CLOCK_FANOUT: usize = 5;

/// Number of global primary clock networks in an ECP5 device.
pub const ECP5_GLOBAL_CLOCK_COUNT: usize = 16;

/// Target packing decisions consumed by grouped placement and configuration.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Ecp5Packing {
    constraints: PlacementConstraints,
    carry_pairs: Vec<[CellId; 2]>,
    carry_pairs_packed: bool,
    lut_ff_pairs: Vec<LutFfPair>,
    general_routing_ffs: Vec<CellId>,
    block_rams: Vec<PackedBlockRam>,
    block_rams_packed: bool,
    global_clocks: Vec<PackedGlobalClock>,
    global_clocks_packed: bool,
    io_attributes: BTreeMap<CellId, BTreeMap<String, String>>,
    clock_frequencies_hz: BTreeMap<CellId, u64>,
    unsupported_lpf_commands: Vec<String>,
}

/// One logical IO cell constrained to a package ball or lead.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PackagePinBinding {
    /// Logical IO cell.
    pub cell: CellId,
    /// Package ball or lead name.
    pub pin: String,
}

impl Ecp5Packing {
    /// Atomic placement groups and candidate-specific pin bindings.
    #[must_use]
    pub const fn constraints(&self) -> &PlacementConstraints {
        &self.constraints
    }

    /// LUT/FF pairs using the dedicated data path (`SD=1`).
    #[must_use]
    pub fn lut_ff_pairs(&self) -> &[LutFfPair] {
        &self.lut_ff_pairs
    }

    /// Transfers one LUT's dedicated data edge to another directly-driven FF.
    ///
    /// The replacement FF takes the same placement-group column as the old
    /// FF. The displaced FF is changed to its general-routing `M` input. This
    /// is the atomic packing mutation used by post-route timing ECOs.
    ///
    /// # Errors
    ///
    /// Returns an error when the LUT is not currently paired, the replacement
    /// is not a general-routed direct fanout, or the packing group is missing.
    pub fn reassign_lut_ff_pair(
        &mut self,
        design: &Design,
        lut: CellId,
        new_ff: CellId,
    ) -> Result<CellId, PackingError> {
        let lut_name = design
            .cells()
            .get(lut.0)
            .map_or_else(|| format!("cell#{}", lut.0), |cell| cell.name.clone());
        let new_ff_name = design
            .cells()
            .get(new_ff.0)
            .map_or_else(|| format!("cell#{}", new_ff.0), |cell| cell.name.clone());
        let Some(pair_index) = self.lut_ff_pairs.iter().position(|pair| pair.lut == lut) else {
            return Err(PackingError::InvalidLutFfPair {
                lut: lut_name,
                ff: new_ff_name,
                reason: "LUT has no current dedicated-path FF".into(),
            });
        };
        if !self.general_routing_ffs.contains(&new_ff) {
            return Err(PackingError::InvalidLutFfPair {
                lut: lut_name,
                ff: new_ff_name,
                reason: "replacement FF is not available on general routing".into(),
            });
        }
        let new_data_pin = design
            .cells()
            .get(new_ff.0)
            .and_then(|cell| {
                cell.pins()
                    .iter()
                    .copied()
                    .find(|pin| design.pins()[pin.0].name == "DI")
            })
            .ok_or_else(|| PackingError::MissingFfDataPin {
                cell: new_ff_name.clone(),
            })?;
        if lut_driver(design, new_data_pin) != Some(lut) {
            return Err(PackingError::InvalidLutFfPair {
                lut: lut_name,
                ff: new_ff_name,
                reason: "LUT does not directly drive the replacement FF data input".into(),
            });
        }
        let old_ff = self.lut_ff_pairs[pair_index].ff;
        let old_data_pin = design.cells()[old_ff.0]
            .pins()
            .iter()
            .copied()
            .find(|pin| design.pins()[pin.0].name == "DI")
            .ok_or_else(|| PackingError::MissingFfDataPin {
                cell: design.cells()[old_ff.0].name.clone(),
            })?;
        if !self.constraints.replace_group_cell(old_ff, new_ff) {
            return Err(PackingError::InvalidLutFfPair {
                lut: design.cells()[lut.0].name.clone(),
                ff: new_ff_name,
                reason: "dedicated-path placement group cannot be reassigned".into(),
            });
        }
        self.constraints.bind_pin_name(old_data_pin, "M");
        self.constraints.unbind_pin_name(new_data_pin);
        self.lut_ff_pairs[pair_index].ff = new_ff;
        self.general_routing_ffs.retain(|&ff| ff != new_ff);
        self.general_routing_ffs.push(old_ff);
        self.general_routing_ffs.sort_unstable();
        Ok(old_ff)
    }

    /// Releases one dedicated LUT-to-FF edge onto the FF's general-routing
    /// `M` input.
    ///
    /// The current placement remains legal, but the LUT and FF cease to be an
    /// atomic group so a post-route hold ECO may route extra minimum delay or
    /// move the FF independently.
    ///
    /// # Errors
    ///
    /// Returns an error when the exact pair is not currently packed, its data
    /// pin is missing, or the corresponding placement group is missing.
    pub fn release_lut_ff_pair(
        &mut self,
        design: &Design,
        lut: CellId,
        ff: CellId,
    ) -> Result<(), PackingError> {
        let lut_name = design
            .cells()
            .get(lut.0)
            .map_or_else(|| format!("cell#{}", lut.0), |cell| cell.name.clone());
        let ff_name = design
            .cells()
            .get(ff.0)
            .map_or_else(|| format!("cell#{}", ff.0), |cell| cell.name.clone());
        let Some(pair_index) = self
            .lut_ff_pairs
            .iter()
            .position(|pair| pair.lut == lut && pair.ff == ff)
        else {
            return Err(PackingError::InvalidLutFfPair {
                lut: lut_name,
                ff: ff_name,
                reason: "exact LUT/FF pair is not currently dedicated".into(),
            });
        };
        let data_pin = design.cells()[ff.0]
            .pins()
            .iter()
            .copied()
            .find(|pin| design.pins()[pin.0].name == "DI")
            .ok_or_else(|| PackingError::MissingFfDataPin {
                cell: design.cells()[ff.0].name.clone(),
            })?;
        if !self.constraints.remove_group(&[lut, ff]) {
            return Err(PackingError::InvalidLutFfPair {
                lut: design.cells()[lut.0].name.clone(),
                ff: design.cells()[ff.0].name.clone(),
                reason: "dedicated-path placement group cannot be released".into(),
            });
        }
        self.constraints.bind_pin_name(data_pin, "M");
        self.lut_ff_pairs.remove(pair_index);
        self.general_routing_ffs.push(ff);
        self.general_routing_ffs.sort_unstable();
        Ok(())
    }

    /// Split `CCU2C` pairs assigned to the two LUTs in one ECP5 slice.
    #[must_use]
    pub fn carry_pairs(&self) -> &[[CellId; 2]] {
        &self.carry_pairs
    }

    /// Constrains complete split `CCU2C` chains to dedicated carry-connected
    /// K0/K1 `TRELLIS_COMB` BEL sequences.
    ///
    /// # Errors
    ///
    /// Returns an error for duplicate/invalid cells, unavailable compatible
    /// slice pairs, or a second invocation.
    pub fn pack_carry_pairs(
        &mut self,
        design: &Design,
        architecture: &Ecp5Architecture,
        pairs: impl IntoIterator<Item = [CellId; 2]>,
    ) -> Result<(), PackingError> {
        if self.carry_pairs_packed {
            return Err(PackingError::CarryPairsAlreadyPacked);
        }
        let mut occupied = BTreeSet::new();
        let mut packed = Vec::new();
        for pair in pairs {
            for &cell in &pair {
                let Some(logical) = design.cells().get(cell.0) else {
                    return Err(PackingError::UnknownCarryCell(cell));
                };
                if !is_carry_slice(design, cell) {
                    return Err(PackingError::CellIsNotCarrySlice {
                        cell: logical.name.clone(),
                    });
                }
                if !occupied.insert(cell) {
                    return Err(PackingError::DuplicateCarryCell {
                        cell: logical.name.clone(),
                    });
                }
            }
            packed.push(pair);
        }
        let chains = logical_carry_chains(design, &packed)?;
        let mut assignments_by_length = BTreeMap::<usize, Arc<[Vec<BelId>]>>::new();
        for chain in chains {
            let assignments = assignments_by_length
                .entry(chain.len())
                .or_insert_with(|| Arc::from(carry_chain_assignments(architecture, chain.len())));
            if assignments.is_empty() {
                return Err(PackingError::MissingCarrySlicePair {
                    cell: design.cells()[packed[chain[0]][0].0].name.clone(),
                });
            }
            let cells = chain
                .iter()
                .flat_map(|&pair| packed[pair])
                .collect::<Vec<_>>();
            self.constraints
                .add_group_with_shared_assignments(cells, Arc::clone(assignments));
        }
        self.carry_pairs = packed;
        self.carry_pairs_packed = true;
        Ok(())
    }

    /// FFs rebound from logical `DI` to the general-routing `M` pin (`SD=0`).
    #[must_use]
    pub fn general_routing_ffs(&self) -> &[CellId] {
        &self.general_routing_ffs
    }

    /// Packed `DP16KD` memories in stable logical cell order.
    #[must_use]
    pub fn block_rams(&self) -> &[PackedBlockRam] {
        &self.block_rams
    }

    /// Inserted DCCA buffers in stable source-net order.
    #[must_use]
    pub fn global_clocks(&self) -> &[PackedGlobalClock] {
        &self.global_clocks
    }

    /// LPF `IOBUF` attributes resolved to logical IO cells.
    #[must_use]
    pub const fn io_attributes(&self) -> &BTreeMap<CellId, BTreeMap<String, String>> {
        &self.io_attributes
    }

    /// LPF clock frequencies resolved to logical IO cells.
    #[must_use]
    pub const fn clock_frequencies_hz(&self) -> &BTreeMap<CellId, u64> {
        &self.clock_frequencies_hz
    }

    /// LPF commands retained because this packing stage does not implement them.
    #[must_use]
    pub fn unsupported_lpf_commands(&self) -> &[String] {
        &self.unsupported_lpf_commands
    }

    /// Applies resolved LPF locations, IO attributes, and clock frequencies
    /// atomically.
    ///
    /// # Errors
    ///
    /// Returns package-binding errors, invalid IO-cell references, or an
    /// attribute conflict with a previously applied LPF set.
    pub fn apply_resolved_lpf(
        &mut self,
        design: &Design,
        architecture: &Ecp5Architecture,
        package_name: &str,
        resolved: &ResolvedLpf,
    ) -> Result<(), PackingError> {
        let mut io_attributes = self.io_attributes.clone();
        for (&cell_id, attributes) in &resolved.io_attributes {
            let Some(cell) = design.cells().get(cell_id.0) else {
                return Err(PackingError::UnknownIoCell(cell_id));
            };
            if cell.kind != ResourceKind::Io {
                return Err(PackingError::CellIsNotIo {
                    cell: cell.name.clone(),
                });
            }
            let target = io_attributes.entry(cell_id).or_default();
            for (key, value) in attributes {
                if let Some(previous) = target.get(key)
                    && previous != value
                {
                    return Err(PackingError::ConflictingIoAttribute {
                        cell: cell.name.clone(),
                        key: key.clone(),
                    });
                }
                target.insert(key.clone(), value.clone());
            }
        }

        let mut clock_frequencies_hz = self.clock_frequencies_hz.clone();
        for (&cell_id, &frequency_hz) in &resolved.clock_frequencies_hz {
            let Some(cell) = design.cells().get(cell_id.0) else {
                return Err(PackingError::UnknownIoCell(cell_id));
            };
            if cell.kind != ResourceKind::Io {
                return Err(PackingError::CellIsNotIo {
                    cell: cell.name.clone(),
                });
            }
            if let Some(previous) = clock_frequencies_hz.insert(cell_id, frequency_hz)
                && previous != frequency_hz
            {
                return Err(PackingError::ConflictingClockFrequency {
                    cell: cell.name.clone(),
                });
            }
        }

        self.bind_package_pins(
            design,
            architecture,
            package_name,
            resolved.package_pins.clone(),
        )?;
        self.io_attributes = io_attributes;
        self.clock_frequencies_hz = clock_frequencies_hz;
        self.unsupported_lpf_commands
            .extend(resolved.unsupported_commands.iter().cloned());
        Ok(())
    }

    /// Adds fixed IO BEL assignments for one exact package.
    ///
    /// # Errors
    ///
    /// Returns an error for an unknown package/pin, a non-IO logical cell,
    /// duplicate cell or package-pin bindings, or an incompatible PIO BEL.
    pub fn bind_package_pins(
        &mut self,
        design: &Design,
        architecture: &Ecp5Architecture,
        package_name: &str,
        bindings: impl IntoIterator<Item = PackagePinBinding>,
    ) -> Result<(), PackingError> {
        let package = architecture
            .packages()
            .iter()
            .find(|package| package.name == package_name)
            .ok_or_else(|| PackingError::UnknownPackage(package_name.into()))?;
        let graph = UnifiedGraph::new(design, architecture.device());
        let mut cells = BTreeSet::new();
        let mut pins = BTreeSet::new();
        let mut fixed_groups = Vec::new();
        for binding in bindings {
            let Some(cell) = design.cells().get(binding.cell.0) else {
                return Err(PackingError::UnknownIoCell(binding.cell));
            };
            if cell.kind != ResourceKind::Io {
                return Err(PackingError::CellIsNotIo {
                    cell: cell.name.clone(),
                });
            }
            if !cells.insert(binding.cell) {
                return Err(PackingError::DuplicateIoCell {
                    cell: cell.name.clone(),
                });
            }
            if !pins.insert(binding.pin.clone()) {
                return Err(PackingError::DuplicatePackagePin(binding.pin));
            }
            let bel = package.pins.get(&binding.pin).copied().ok_or_else(|| {
                PackingError::UnknownPackagePin {
                    package: package_name.into(),
                    pin: binding.pin.clone(),
                }
            })?;
            if !graph.placement_candidates(binding.cell)?.contains(&bel) {
                return Err(PackingError::IncompatiblePackagePin {
                    cell: cell.name.clone(),
                    package: package_name.into(),
                    pin: binding.pin,
                });
            }
            fixed_groups.push((binding.cell, bel));
        }
        for (cell, bel) in fixed_groups {
            self.constraints.add_group([cell], [vec![bel]]);
        }
        Ok(())
    }

    /// Validates and constrains every logical memory to an ECP5 `DP16KD` BEL.
    ///
    /// Requirements are matched by cell ID, independent of input order. The
    /// operation is transactional and assigns WID values starting at 3 in
    /// stable cell order, matching the convention used by nextpnr-ecp5.
    ///
    /// # Errors
    ///
    /// Returns an error for missing, duplicate, unknown, non-memory, or
    /// physically illegal requirements, an incompatible architecture, or a
    /// second invocation on the same packing result.
    pub fn pack_block_rams(
        &mut self,
        design: &Design,
        architecture: &Ecp5Architecture,
        requirements: impl IntoIterator<Item = BlockRamRequirement>,
    ) -> Result<(), PackingError> {
        if self.block_rams_packed {
            return Err(PackingError::BlockRamsAlreadyPacked);
        }

        let mut requirements_by_cell = BTreeMap::new();
        for requirement in requirements {
            let Some(cell) = design.cells().get(requirement.cell.0) else {
                return Err(PackingError::UnknownBlockRamCell(requirement.cell));
            };
            if cell.kind != ResourceKind::Memory {
                return Err(PackingError::CellIsNotBlockRam {
                    cell: cell.name.clone(),
                });
            }
            if requirements_by_cell
                .insert(requirement.cell, requirement)
                .is_some()
            {
                return Err(PackingError::DuplicateBlockRamRequirement {
                    cell: cell.name.clone(),
                });
            }
        }

        for (index, cell) in design.cells().iter().enumerate() {
            let cell_id = CellId(index);
            if cell.kind == ResourceKind::Memory && !requirements_by_cell.contains_key(&cell_id) {
                return Err(PackingError::MissingBlockRamRequirement {
                    cell: cell.name.clone(),
                });
            }
        }

        let graph = UnifiedGraph::new(design, architecture.device());
        let mut groups = Vec::new();
        let mut packed = Vec::new();
        for (index, requirement) in requirements_by_cell.values().copied().enumerate() {
            let cell_name = &design.cells()[requirement.cell.0].name;
            let Some(max_depth) = dp16kd_max_depth(requirement.physical_width) else {
                return Err(PackingError::InvalidBlockRamPhysicalWidth {
                    cell: cell_name.clone(),
                    physical_width: requirement.physical_width,
                });
            };
            if requirement.word_width == 0 || requirement.word_width > requirement.physical_width {
                return Err(PackingError::InvalidBlockRamWordWidth {
                    cell: cell_name.clone(),
                    word_width: requirement.word_width,
                    physical_width: requirement.physical_width,
                });
            }
            if requirement.depth == 0 || requirement.depth > max_depth {
                return Err(PackingError::InvalidBlockRamDepth {
                    cell: cell_name.clone(),
                    depth: requirement.depth,
                    max_depth,
                });
            }

            let assignments = graph
                .placement_candidates(requirement.cell)?
                .into_iter()
                .filter(|bel| architecture.bel_metadata(*bel).bel_type == "DP16KD")
                .map(|bel| vec![bel])
                .collect::<Vec<_>>();
            if assignments.is_empty() {
                return Err(PackingError::MissingBlockRamBel {
                    cell: cell_name.clone(),
                });
            }

            let wid = u32::try_from(index)
                .ok()
                .and_then(|index| index.checked_add(3))
                .ok_or(PackingError::TooManyBlockRams)?;
            groups.push((requirement.cell, assignments));
            packed.push(PackedBlockRam {
                cell: requirement.cell,
                wid,
                depth: requirement.depth,
                word_width: requirement.word_width,
                physical_width: requirement.physical_width,
            });
        }

        for (cell, assignments) in groups {
            self.constraints.add_group([cell], assignments);
        }
        self.block_rams = packed;
        self.block_rams_packed = true;
        Ok(())
    }

    /// Inserts and constrains DCCA buffers for selected clock nets.
    ///
    /// Only recognized clock sinks move behind `CLKO`; any data sinks remain
    /// directly connected to the original net. Requirements are processed in
    /// stable net-ID order, independent of input order. Both the design and
    /// packing constraints are updated transactionally.
    ///
    /// # Errors
    ///
    /// Returns an error for an unknown or duplicate net, a net without a
    /// recognized clock sink, insufficient compatible DCCA BELs, graph/model
    /// failures, or a second invocation on the same packing result.
    pub fn promote_global_clocks(
        &mut self,
        design: &mut Design,
        architecture: &Ecp5Architecture,
        requirements: impl IntoIterator<Item = GlobalClockRequirement>,
    ) -> Result<(), PackingError> {
        if self.global_clocks_packed {
            return Err(PackingError::GlobalClocksAlreadyPacked);
        }

        let mut requirements_by_net = BTreeMap::new();
        for requirement in requirements {
            let Some(net) = design.nets().get(requirement.net.0) else {
                return Err(PackingError::Model(ModelError::UnknownNet(requirement.net)));
            };
            if requirements_by_net
                .insert(requirement.net, requirement)
                .is_some()
            {
                return Err(PackingError::DuplicateGlobalClockRequirement {
                    net: net.name.clone(),
                });
            }
        }

        let pending = requirements_by_net
            .values()
            .map(|requirement| {
                let net = &design.nets()[requirement.net.0];
                let sinks = net
                    .sinks
                    .iter()
                    .copied()
                    .filter(|&pin| is_clock_sink(design, pin))
                    .collect::<Vec<_>>();
                if sinks.is_empty() {
                    Err(PackingError::GlobalClockHasNoClockSinks {
                        net: net.name.clone(),
                    })
                } else {
                    Ok((requirement.net, net.name.clone(), sinks))
                }
            })
            .collect::<Result<Vec<_>, _>>()?;
        let dcca_bels = compatible_dcca_bels(architecture);
        let available_global_clocks = dcca_bels.len().min(ECP5_GLOBAL_CLOCK_COUNT);
        if pending.len() > available_global_clocks {
            return Err(PackingError::InsufficientGlobalClockBels {
                required: pending.len(),
                available: available_global_clocks,
            });
        }

        let mut transformed = design.clone();
        let mut constraints = self.constraints.clone();
        let mut packed = Vec::new();
        for (source_net, net_name, sinks) in pending {
            let inserted = transformed.insert_buffer_on_net(
                source_net,
                sinks,
                BufferSpec {
                    cell_name: format!("$gbuf${net_name}"),
                    kind: ResourceKind::Clock,
                    input_pin: "CLKI".into(),
                    output_pin: "CLKO".into(),
                    output_net: format!("$glbnet${net_name}"),
                },
            )?;
            let assignments = UnifiedGraph::new(&transformed, architecture.device())
                .placement_candidates(inserted.cell)?
                .into_iter()
                .filter(|bel| architecture.bel_metadata(*bel).bel_type == "DCCA")
                .map(|bel| vec![bel])
                .collect::<Vec<_>>();
            if assignments.is_empty() {
                return Err(PackingError::InsufficientGlobalClockBels {
                    required: 1,
                    available: 0,
                });
            }
            constraints.add_group([inserted.cell], assignments);
            packed.push(PackedGlobalClock {
                source_net,
                buffer: inserted.cell,
                global_net: inserted.output_net,
            });
        }

        *design = transformed;
        self.constraints = constraints;
        self.global_clocks = packed;
        self.global_clocks_packed = true;
        Ok(())
    }

    /// Builds one locked ECP5 primary-clock tree per promoted DCCA net.
    ///
    /// Placement must already be complete because the local `HPBX → CLK`
    /// branches depend on sink BEL coordinates. Each clock receives a stable,
    /// distinct global network index. The result uses ordinary [`WireId`] and
    /// [`PipId`] resources, including the fixed spine/tap aliases expanded
    /// from the architecture's global metadata.
    ///
    /// # Errors
    ///
    /// Returns an error when placement is incomplete or the exported global
    /// topology cannot connect a DCCA to every placed clock sink.
    pub fn global_routing_constraints(
        &self,
        design: &Design,
        architecture: &Ecp5Architecture,
        placement: &Placement,
    ) -> Result<RoutingConstraints, PackingError> {
        let mut cache = architecture.global_routing_cache();
        self.global_routing_constraints_cached(design, architecture, placement, &mut cache)
    }

    /// Builds global-clock constraints using indexes retained by the caller.
    ///
    /// This is equivalent to [`Self::global_routing_constraints`] but avoids
    /// rebuilding device-wide reverse-routing indexes for every placement.
    ///
    /// # Errors
    ///
    /// Returns an error when placement is incomplete or the exported global
    /// topology cannot connect a DCCA to every placed clock sink.
    pub fn global_routing_constraints_cached(
        &self,
        design: &Design,
        architecture: &Ecp5Architecture,
        placement: &Placement,
        cache: &mut Ecp5GlobalRoutingCache<'_>,
    ) -> Result<RoutingConstraints, PackingError> {
        let device = architecture.device();
        let graph = UnifiedGraph::new(design, device);
        let mut constraints = RoutingConstraints::new();
        for (network, clock) in self.global_clocks.iter().enumerate() {
            let net = &design.nets()[clock.global_net.0];
            let source = placed_pin_wire(&graph, placement, net.driver)?;
            let mut wires = BTreeSet::from([source]);
            let mut pips = BTreeSet::new();

            for quadrant in [
                GlobalQuadrant::Ul,
                GlobalQuadrant::Ur,
                GlobalQuadrant::Ll,
                GlobalQuadrant::Lr,
            ] {
                let root_name = format!("G_{}PCLK{network}", quadrant.wire_prefix());
                let root = cache.unique_wire(&root_name).ok_or_else(|| {
                    global_route_error(net, format!("missing quadrant root `{root_name}`"))
                })?;
                let (path_wires, path_pips) =
                    cache.forward_route(device, source, root).ok_or_else(|| {
                        global_route_error(net, format!("DCCA cannot reach `{root_name}`"))
                    })?;
                wires.extend(path_wires);
                pips.extend(path_pips);
            }

            let tile_name = format!("G_HPBX{network:02}00");
            for &sink in &net.sinks {
                let sink_wire = placed_pin_wire(&graph, placement, sink)?;
                if wires.contains(&sink_wire) {
                    continue;
                }
                let (join, branch_wires, branch_pips) = cache
                    .reverse_route(device, network, sink_wire, &wires, &tile_name)
                    .ok_or_else(|| {
                        global_route_error(
                            net,
                            format!(
                                "cannot reach `{tile_name}` from sink wire `{}`",
                                device.wires()[sink_wire.0].name
                            ),
                        )
                    })?;
                wires.extend(branch_wires);
                pips.extend(branch_pips);
                if !wires.contains(&join) {
                    return Err(global_route_error(
                        net,
                        "local branch did not join its tree",
                    ));
                }
                if wire_basename(&device.wires()[join.0].name) == tile_name {
                    add_global_trunk(architecture, cache, network, join, &mut wires, &mut pips)
                        .map_err(|reason| global_route_error(net, reason))?;
                }
            }
            let sinks = net
                .sinks
                .iter()
                .copied()
                .map(|sink| Ok((sink, placed_pin_wire(&graph, placement, sink)?)))
                .collect::<Result<Vec<_>, PackingError>>()?;
            let route = NetRoute::from_tree(clock.global_net, source, sinks, pips, device)
                .map_err(|reason| global_route_error(net, reason))?;
            constraints.add_route(route);
        }
        Ok(constraints)
    }
}

impl Ecp5Architecture {
    /// Builds immutable indexes shared by repeated global-clock routes.
    #[must_use]
    pub fn global_routing_cache(&self) -> Ecp5GlobalRoutingCache<'_> {
        let device = self.device();
        let incoming = CompactIncomingPips::new(device);
        let mut unique_roots = HashMap::new();
        for (index, wire) in device.wires().iter().enumerate() {
            let name = wire_basename(&wire.name);
            if is_global_root_name(name) {
                unique_roots
                    .entry(name)
                    .and_modify(|known| *known = None)
                    .or_insert(Some(WireId(index)));
            }
        }
        Ecp5GlobalRoutingCache {
            incoming,
            unique_roots,
            forward_routes: HashMap::new(),
            reverse_routes: HashMap::new(),
            reverse_search: GlobalReverseSearch::new(device.wires().len()),
        }
    }
}

impl Ecp5GlobalRoutingCache<'_> {
    fn unique_wire(&self, name: &str) -> Option<WireId> {
        self.unique_roots.get(name).copied().flatten()
    }

    fn forward_route(
        &mut self,
        device: &Device,
        source: WireId,
        target: WireId,
    ) -> Option<(Vec<WireId>, Vec<PipId>)> {
        if let std::collections::hash_map::Entry::Vacant(entry) =
            self.forward_routes.entry((source, target))
        {
            entry.insert(forward_route(device, source, target)?);
        }
        self.forward_routes.get(&(source, target)).cloned()
    }

    fn reverse_route(
        &mut self,
        device: &Device,
        network: usize,
        sink: WireId,
        tree: &BTreeSet<WireId>,
        target_name: &str,
    ) -> Option<(WireId, Vec<WireId>, Vec<PipId>)> {
        let key = (network, sink);
        if let Some((join, wires, pips)) = self.reverse_routes.get(&key)
            && (tree.contains(join) || wire_basename(&device.wires()[join.0].name) == target_name)
        {
            return Some((*join, wires.clone(), pips.clone()));
        }
        let route = self
            .reverse_search
            .route(device, &self.incoming, sink, tree, target_name)?;
        self.reverse_routes.insert(key, route.clone());
        Some(route)
    }
}

fn global_route_error(net: &texo_model::Net, reason: impl Into<String>) -> PackingError {
    PackingError::GlobalClockRouting {
        net: net.name.clone(),
        reason: reason.into(),
    }
}

fn placed_pin_wire(
    graph: &UnifiedGraph<'_>,
    placement: &Placement,
    pin: CellPinId,
) -> Result<WireId, PackingError> {
    let cell = graph.design().pins()[pin.0].cell;
    let bel = placement
        .bel(cell)
        .ok_or_else(|| PackingError::GlobalClockRouting {
            net: graph.design().cells()[cell.0].name.clone(),
            reason: "cell is missing from placement".into(),
        })?;
    if let Some(bel_pin) = placement.pin_binding(pin) {
        Ok(graph.device().bel_pins()[bel_pin.0].wire)
    } else {
        Ok(graph.bound_wire(pin, bel)?)
    }
}

fn forward_route(
    device: &Device,
    source: WireId,
    target: WireId,
) -> Option<(Vec<WireId>, Vec<PipId>)> {
    let mut seen = vec![false; device.wires().len()];
    let mut previous = vec![None; device.wires().len()];
    let mut queue = VecDeque::from([source]);
    seen[source.0] = true;
    while let Some(mut wire) = queue.pop_front() {
        if wire == target {
            let mut wires = vec![wire];
            let mut pips = Vec::new();
            while let Some((prior, pip)) = previous[wire.0] {
                pips.push(pip);
                wire = prior;
                wires.push(wire);
            }
            return Some((wires, pips));
        }
        for (next, pip) in device.routing_neighbors(wire).ok()? {
            if !seen[next.0] {
                seen[next.0] = true;
                previous[next.0] = Some((wire, pip));
                queue.push_back(next);
            }
        }
    }
    None
}

#[derive(Debug)]
struct GlobalReverseSearch {
    epoch: u32,
    seen: Vec<u32>,
    next_wire: Vec<usize>,
    next_pip: Vec<usize>,
}

impl GlobalReverseSearch {
    fn new(wire_count: usize) -> Self {
        Self {
            epoch: 0,
            seen: vec![0; wire_count],
            next_wire: vec![usize::MAX; wire_count],
            next_pip: vec![usize::MAX; wire_count],
        }
    }

    fn route(
        &mut self,
        device: &Device,
        incoming: &CompactIncomingPips,
        sink: WireId,
        tree: &BTreeSet<WireId>,
        target_name: &str,
    ) -> Option<(WireId, Vec<WireId>, Vec<PipId>)> {
        self.epoch = self.epoch.wrapping_add(1);
        if self.epoch == 0 {
            self.seen.fill(0);
            self.epoch = 1;
        }
        let epoch = self.epoch;
        let mut queue = VecDeque::from([sink]);
        self.seen[sink.0] = epoch;
        self.next_wire[sink.0] = usize::MAX;
        self.next_pip[sink.0] = usize::MAX;
        while let Some(mut wire) = queue.pop_front() {
            if tree.contains(&wire) || wire_basename(&device.wires()[wire.0].name) == target_name {
                let join = wire;
                let mut wires = vec![wire];
                let mut pips = Vec::new();
                while self.next_wire[wire.0] != usize::MAX {
                    pips.push(PipId(self.next_pip[wire.0]));
                    wire = WireId(self.next_wire[wire.0]);
                    wires.push(wire);
                }
                return Some((join, wires, pips));
            }
            for &pip in incoming.for_wire(wire) {
                let pip = PipId(pip as usize);
                let prior = device.pips()[pip.0].from();
                if self.seen[prior.0] != epoch {
                    self.seen[prior.0] = epoch;
                    self.next_wire[prior.0] = wire.0;
                    self.next_pip[prior.0] = pip.0;
                    queue.push_back(prior);
                }
            }
        }
        None
    }
}

fn add_global_trunk(
    architecture: &Ecp5Architecture,
    cache: &Ecp5GlobalRoutingCache<'_>,
    network: usize,
    tile: WireId,
    wires: &mut BTreeSet<WireId>,
    pips: &mut BTreeSet<PipId>,
) -> Result<(), String> {
    let device = architecture.device();
    let tap_alias = only_global_alias_incoming(architecture, &cache.incoming, tile)?;
    add_pip_to_tree(device, tap_alias, wires, pips);
    let tap = device.pips()[tap_alias.0].from();

    let tap_pip = only_original_incoming(architecture, &cache.incoming, tap)?;
    add_pip_to_tree(device, tap_pip, wires, pips);
    let tap_source = device.pips()[tap_pip.0].from();
    let tap_info = architecture
        .global_info(device.wires()[tap_source.0].point)
        .ok_or_else(|| "tap source has no global metadata".to_owned())?;
    let spine_point = tap_info
        .spine
        .ok_or_else(|| "tap source has no spine coordinate".to_owned())?;
    let spine_alias = only_global_alias_incoming(architecture, &cache.incoming, tap_source)?;
    let spine = device.pips()[spine_alias.0].from();
    if device.wires()[spine.0].point != spine_point {
        return Err("global spine alias has the wrong coordinate".to_owned());
    }
    add_pip_to_tree(device, spine_alias, wires, pips);

    let spine_pip = only_original_incoming(architecture, &cache.incoming, spine)?;
    add_pip_to_tree(device, spine_pip, wires, pips);
    let spine_source = device.pips()[spine_pip.0].from();
    let root_name = format!("G_{}PCLK{network}", tap_info.quadrant.wire_prefix());
    let expected_root = cache
        .unique_roots
        .get(root_name.as_str())
        .copied()
        .flatten()
        .ok_or_else(|| format!("missing unique quadrant root `{root_name}`"))?;
    let root_alias = only_global_alias_incoming(architecture, &cache.incoming, spine_source)?;
    if device.pips()[root_alias.0].from() != expected_root {
        return Err(format!("global spine does not connect to `{root_name}`"));
    }
    add_pip_to_tree(device, root_alias, wires, pips);
    Ok(())
}

fn is_global_root_name(name: &str) -> bool {
    ["G_ULPCLK", "G_URPCLK", "G_LLPCLK", "G_LRPCLK"]
        .iter()
        .any(|prefix| {
            name.strip_prefix(prefix)
                .and_then(|network| network.parse::<usize>().ok())
                .is_some_and(|network| network < ECP5_GLOBAL_CLOCK_COUNT)
        })
}

fn only_global_alias_incoming(
    architecture: &Ecp5Architecture,
    incoming: &CompactIncomingPips,
    wire: WireId,
) -> Result<PipId, String> {
    only_incoming_by_alias_kind(architecture, incoming, wire, true)
}

fn only_original_incoming(
    architecture: &Ecp5Architecture,
    incoming: &CompactIncomingPips,
    wire: WireId,
) -> Result<PipId, String> {
    only_incoming_by_alias_kind(architecture, incoming, wire, false)
}

fn only_incoming_by_alias_kind(
    architecture: &Ecp5Architecture,
    incoming: &CompactIncomingPips,
    wire: WireId,
    alias: bool,
) -> Result<PipId, String> {
    let candidates = incoming
        .for_wire(wire)
        .iter()
        .map(|&pip| PipId(pip as usize))
        .filter(|&pip| (architecture.pip_metadata(pip).tile_type == "TEXO_GLOBAL_ALIAS") == alias)
        .collect::<Vec<_>>();
    match candidates.as_slice() {
        [pip] => Ok(*pip),
        _ => Err(format!(
            "wire `{}` has {} {} incoming PIPs instead of one",
            architecture.device().wires()[wire.0].name,
            candidates.len(),
            if alias { "global-alias" } else { "physical" },
        )),
    }
}

fn add_pip_to_tree(
    device: &Device,
    pip: PipId,
    wires: &mut BTreeSet<WireId>,
    pips: &mut BTreeSet<PipId>,
) {
    let pip_data = &device.pips()[pip.0];
    wires.insert(pip_data.from());
    wires.insert(pip_data.to());
    pips.insert(pip);
}

fn physical_carry_pairs(architecture: &Ecp5Architecture) -> Vec<[BelId; 2]> {
    let mut comb_by_slot = BTreeMap::new();
    for &bel in architecture.device().bels_of_kind(ResourceKind::Lut(4)) {
        let metadata = architecture.bel_metadata(bel);
        if metadata.bel_type == "TRELLIS_COMB" {
            comb_by_slot.insert((architecture.device().bels()[bel.0].point, metadata.z), bel);
        }
    }
    let mut assignments = Vec::new();
    for (&(point, z), &first) in &comb_by_slot {
        if z.rem_euclid(8) != 0 {
            continue;
        }
        if let Some(second_z) = z.checked_add(4)
            && let Some(&second) = comb_by_slot.get(&(point, second_z))
        {
            let Some(first_fco) = find_bel_pin(architecture.device(), first, "FCO") else {
                continue;
            };
            if find_bel_pin(architecture.device(), first, "FCI").is_none() {
                continue;
            }
            let Some(second_fci) = find_bel_pin(architecture.device(), second, "FCI") else {
                continue;
            };
            if find_bel_pin(architecture.device(), second, "FCO").is_none() {
                continue;
            }
            if architecture.device().bel_pins()[first_fco.0].wire
                == architecture.device().bel_pins()[second_fci.0].wire
            {
                assignments.push([first, second]);
            }
        }
    }
    assignments
}

fn carry_chain_assignments(architecture: &Ecp5Architecture, pair_count: usize) -> Vec<Vec<BelId>> {
    let pairs = physical_carry_pairs(architecture);
    if pair_count == 0 {
        return Vec::new();
    }
    let mut pairs_by_fci_wire = BTreeMap::<WireId, Vec<usize>>::new();
    let mut fco_wires = Vec::with_capacity(pairs.len());
    for (index, pair) in pairs.iter().enumerate() {
        let fci = find_bel_pin(architecture.device(), pair[0], "FCI")
            .expect("physical carry pairs require first FCI");
        let fco = find_bel_pin(architecture.device(), pair[1], "FCO")
            .expect("physical carry pairs require second FCO");
        pairs_by_fci_wire
            .entry(architecture.device().bel_pins()[fci.0].wire)
            .or_default()
            .push(index);
        fco_wires.push(architecture.device().bel_pins()[fco.0].wire);
    }
    let successors = fco_wires
        .iter()
        .map(|&wire| fixed_carry_successors(architecture, wire, &pairs_by_fci_wire))
        .collect::<Vec<_>>();

    let mut sequences = (0..pairs.len())
        .map(|index| vec![index])
        .collect::<Vec<_>>();
    for _ in 1..pair_count {
        let mut extended = Vec::new();
        for sequence in sequences {
            let Some(&last) = sequence.last() else {
                continue;
            };
            for &successor in &successors[last] {
                if sequence.contains(&successor) {
                    continue;
                }
                let mut next = sequence.clone();
                next.push(successor);
                extended.push(next);
            }
        }
        sequences = extended;
    }
    sequences
        .into_iter()
        .map(|sequence| sequence.into_iter().flat_map(|pair| pairs[pair]).collect())
        .collect()
}

fn fixed_carry_successors(
    architecture: &Ecp5Architecture,
    start: WireId,
    pairs_by_fci_wire: &BTreeMap<WireId, Vec<usize>>,
) -> Vec<usize> {
    let mut queue = VecDeque::from([start]);
    let mut visited = BTreeSet::from([start]);
    let mut successors = BTreeSet::new();
    while let Some(wire) = queue.pop_front() {
        if let Some(pairs) = pairs_by_fci_wire.get(&wire) {
            successors.extend(pairs.iter().copied());
            if wire != start {
                continue;
            }
        }
        for (neighbor, pip) in architecture
            .device()
            .routing_neighbors(wire)
            .expect("routing index contains every wire")
        {
            if architecture.pip_metadata(pip).fixed && visited.insert(neighbor) {
                queue.push_back(neighbor);
            }
        }
    }
    successors.into_iter().collect()
}

fn logical_carry_chains(
    design: &Design,
    pairs: &[[CellId; 2]],
) -> Result<Vec<Vec<usize>>, PackingError> {
    let pair_by_cell = pairs
        .iter()
        .enumerate()
        .flat_map(|(index, pair)| [(pair[0], index), (pair[1], index)])
        .collect::<BTreeMap<_, _>>();
    let mut successor = vec![None; pairs.len()];
    let mut predecessor = vec![None; pairs.len()];
    for (index, pair) in pairs.iter().enumerate() {
        let fco = design.cells()[pair[1].0]
            .pins()
            .iter()
            .copied()
            .find(|pin| design.pins()[pin.0].name == "FCO")
            .expect("carry surface was validated");
        let Some(net) = design.pins()[fco.0].net().map(|net| &design.nets()[net.0]) else {
            continue;
        };
        let mut successor_pair = None;
        for &sink in &net.sinks {
            let sink_pin = &design.pins()[sink.0];
            if sink_pin.name != "FCI" {
                return Err(PackingError::InvalidCarryConnection {
                    cell: design.cells()[pair[1].0].name.clone(),
                    reason: format!(
                        "FCO directly drives general-routing pin {}.{}; a carry feed-out is required",
                        design.cells()[sink_pin.cell.0].name,
                        sink_pin.name
                    ),
                });
            }
            let Some(&next_pair) = pair_by_cell.get(&sink_pin.cell) else {
                return Err(PackingError::InvalidCarryConnection {
                    cell: design.cells()[pair[1].0].name.clone(),
                    reason: format!(
                        "FCI sink cell ID {} is not in a carry pair",
                        sink_pin.cell.0
                    ),
                });
            };
            if successor_pair.replace(next_pair).is_some() {
                return Err(PackingError::InvalidCarryConnection {
                    cell: design.cells()[pair[1].0].name.clone(),
                    reason: "FCO drives more than one carry successor".into(),
                });
            }
        }
        if let Some(next_pair) = successor_pair {
            if predecessor[next_pair].replace(index).is_some() {
                return Err(PackingError::InvalidCarryConnection {
                    cell: design.cells()[pairs[next_pair][0].0].name.clone(),
                    reason: "carry pair has more than one predecessor".into(),
                });
            }
            successor[index] = Some(next_pair);
        }
    }

    let mut visited = BTreeSet::new();
    let mut chains = Vec::new();
    for root in (0..pairs.len()).filter(|&index| predecessor[index].is_none()) {
        let mut chain = Vec::new();
        let mut cursor = Some(root);
        while let Some(index) = cursor {
            if !visited.insert(index) {
                return Err(PackingError::InvalidCarryConnection {
                    cell: design.cells()[pairs[index][0].0].name.clone(),
                    reason: "carry chain contains a cycle".into(),
                });
            }
            chain.push(index);
            cursor = successor[index];
        }
        chains.push(chain);
    }
    if visited.len() != pairs.len() {
        let index = (0..pairs.len())
            .find(|index| !visited.contains(index))
            .expect("length mismatch guarantees an unvisited pair");
        return Err(PackingError::InvalidCarryConnection {
            cell: design.cells()[pairs[index][0].0].name.clone(),
            reason: "carry chain contains a cycle".into(),
        });
    }
    Ok(chains)
}

/// Selects nets with at least `minimum_clock_sinks` recognized clock pins.
///
/// A zero threshold is treated as one. Register `CLK` and block-RAM
/// `CLKA`/`CLKB` pins are recognized. At most 16 nets are returned, choosing
/// the highest fanout first with stable net-ID tie breaking; the returned set
/// itself is ordered by net ID.
#[must_use]
pub fn find_global_clock_requirements(
    design: &Design,
    minimum_clock_sinks: usize,
) -> Vec<GlobalClockRequirement> {
    let minimum_clock_sinks = minimum_clock_sinks.max(1);
    let mut candidates = design
        .nets()
        .iter()
        .enumerate()
        .filter_map(|(index, net)| {
            let clock_sinks = net
                .sinks
                .iter()
                .filter(|&&pin| is_clock_sink(design, pin))
                .count();
            (clock_sinks >= minimum_clock_sinks).then_some((NetId(index), clock_sinks))
        })
        .collect::<Vec<_>>();
    candidates.sort_by_key(|(net, clock_sinks)| (std::cmp::Reverse(*clock_sinks), *net));
    candidates.truncate(ECP5_GLOBAL_CLOCK_COUNT);
    candidates.sort_by_key(|(net, _)| *net);
    candidates
        .into_iter()
        .map(|(net, _)| GlobalClockRequirement { net })
        .collect()
}

fn is_clock_sink(design: &Design, pin: CellPinId) -> bool {
    let pin = &design.pins()[pin.0];
    let kind = design.cells()[pin.cell.0].kind;
    (kind == ResourceKind::Register && pin.name == "CLK")
        || (kind == ResourceKind::Memory && matches!(pin.name.as_str(), "CLKA" | "CLKB"))
}

fn compatible_dcca_bels(architecture: &Ecp5Architecture) -> Vec<BelId> {
    architecture
        .bel_metadata_iter()
        .filter_map(|(bel, metadata)| {
            let input = find_bel_pin(architecture.device(), bel, "CLKI")
                .map(|pin| architecture.device().bel_pins()[pin.0].direction);
            let output = find_bel_pin(architecture.device(), bel, "CLKO")
                .map(|pin| architecture.device().bel_pins()[pin.0].direction);
            (metadata.bel_type == "DCCA"
                && input == Some(PinDirection::Input)
                && output == Some(PinDirection::Output))
            .then_some(bel)
        })
        .collect()
}

fn dp16kd_max_depth(physical_width: u8) -> Option<u32> {
    match physical_width {
        1 => Some(16_384),
        2 => Some(8_192),
        4 => Some(4_096),
        9 => Some(2_048),
        18 => Some(1_024),
        _ => None,
    }
}

/// Packs LUT-driven FFs into matching ECP5 logic-cell BEL pairs.
///
/// For each LUT, the first FF whose `DI` net is driven by that LUT's `F` pin
/// receives an atomic `TRELLIS_COMB(z)` / `TRELLIS_FF(z + 1)` placement group.
/// Other FFs are rebound to the physical `M` pin for ordinary routing.
///
/// # Errors
///
/// Returns an error when the logical FF surface or physical general-routing
/// pin surface is incomplete, or graph candidate generation fails.
pub fn pack_lut_ffs(
    design: &Design,
    architecture: &Ecp5Architecture,
) -> Result<Ecp5Packing, PackingError> {
    pack_lut_ffs_excluding(design, architecture, [])
}

/// Packs LUT-driven FFs while keeping selected LUT outputs on general routing.
///
/// This is used for sources such as constant generators whose direct local FF
/// path cannot provide enough minimum delay for hold repair.
///
/// # Errors
///
/// Returns the same errors as [`pack_lut_ffs`].
pub fn pack_lut_ffs_excluding(
    design: &Design,
    architecture: &Ecp5Architecture,
    excluded_luts: impl IntoIterator<Item = CellId>,
) -> Result<Ecp5Packing, PackingError> {
    let excluded_luts = excluded_luts.into_iter().collect::<BTreeSet<_>>();
    let mut constraints = PlacementConstraints::new();
    let mut paired_luts = BTreeSet::new();
    let mut paired_ffs = BTreeSet::new();
    let mut lut_ff_pairs = Vec::new();
    let mut ff_data_pins = BTreeMap::new();
    let lut_ff_assignments: Arc<[Vec<BelId>]> = lut_ff_assignments(architecture).into();
    let has_general_data_pin = architecture
        .device()
        .bels_of_kind(ResourceKind::Register)
        .iter()
        .copied()
        .any(|bel| {
            architecture.bel_metadata(bel).bel_type == "TRELLIS_FF"
                && find_bel_pin(architecture.device(), bel, "M").is_some()
        });

    for (index, cell) in design.cells().iter().enumerate() {
        if cell.kind != ResourceKind::Register {
            continue;
        }
        let ff = CellId(index);
        let data_pin = cell
            .pins()
            .iter()
            .copied()
            .find(|pin| design.pins()[pin.0].name == "DI")
            .ok_or_else(|| PackingError::MissingFfDataPin {
                cell: cell.name.clone(),
            })?;
        ff_data_pins.insert(ff, data_pin);

        let Some(lut) = lut_driver(design, data_pin) else {
            continue;
        };
        if is_carry_slice(design, lut) || excluded_luts.contains(&lut) {
            continue;
        }
        if paired_luts.contains(&lut) {
            continue;
        }
        if lut_ff_assignments.is_empty() {
            continue;
        }
        constraints.add_group_with_shared_assignments([lut, ff], Arc::clone(&lut_ff_assignments));
        paired_luts.insert(lut);
        paired_ffs.insert(ff);
        lut_ff_pairs.push(LutFfPair { lut, ff });
    }

    let mut general_routing_ffs = Vec::new();
    for (ff, data_pin) in ff_data_pins {
        if paired_ffs.contains(&ff) {
            continue;
        }
        if !has_general_data_pin {
            return Err(PackingError::MissingGeneralDataPin {
                cell: design.cells()[ff.0].name.clone(),
            });
        }
        constraints.bind_pin_name(data_pin, "M");
        general_routing_ffs.push(ff);
    }

    Ok(Ecp5Packing {
        constraints,
        carry_pairs: Vec::new(),
        carry_pairs_packed: false,
        lut_ff_pairs,
        general_routing_ffs,
        block_rams: Vec::new(),
        block_rams_packed: false,
        global_clocks: Vec::new(),
        global_clocks_packed: false,
        io_attributes: BTreeMap::new(),
        clock_frequencies_hz: BTreeMap::new(),
        unsupported_lpf_commands: Vec::new(),
    })
}

/// Enumerates every structurally legal ordinary LUT-to-FF dedicated-path
/// candidate before one FF per LUT is selected.
#[must_use]
pub fn lut_ff_pair_candidates(
    design: &Design,
    excluded_luts: impl IntoIterator<Item = CellId>,
) -> Vec<LutFfPair> {
    let excluded_luts = excluded_luts.into_iter().collect::<BTreeSet<_>>();
    let mut candidates = Vec::new();
    for (index, cell) in design.cells().iter().enumerate() {
        if cell.kind != ResourceKind::Register {
            continue;
        }
        let ff = CellId(index);
        let Some(data_pin) = cell
            .pins()
            .iter()
            .copied()
            .find(|pin| design.pins()[pin.0].name == "DI")
        else {
            continue;
        };
        let Some(lut) = lut_driver(design, data_pin) else {
            continue;
        };
        if !is_carry_slice(design, lut) && !excluded_luts.contains(&lut) {
            candidates.push(LutFfPair { lut, ff });
        }
    }
    candidates
}

/// Packs an explicitly selected set of LUT/FF dedicated-path pairs.
///
/// This is primarily useful when importing a placement produced by another
/// ECP5 packer: the physical locations are only comparable if both tools agree
/// which FF inputs use the local `F → DI` path and which use general routing.
/// Every requested pair must be a real logical LUT-to-FF data connection.
///
/// # Errors
///
/// Returns an error for unknown, duplicate, or disconnected pair members, or
/// when the physical FF surface lacks the required local/general data pins.
#[allow(clippy::too_many_lines)]
pub fn pack_lut_ffs_with_pairs(
    design: &Design,
    architecture: &Ecp5Architecture,
    pairs: impl IntoIterator<Item = LutFfPair>,
) -> Result<Ecp5Packing, PackingError> {
    let mut constraints = PlacementConstraints::new();
    let mut paired_luts = BTreeSet::new();
    let mut paired_ffs = BTreeSet::new();
    let mut lut_ff_pairs = Vec::new();
    let mut ff_data_pins = BTreeMap::new();
    let lut_ff_assignments: Arc<[Vec<BelId>]> = lut_ff_assignments(architecture).into();
    let has_general_data_pin = architecture
        .device()
        .bels_of_kind(ResourceKind::Register)
        .iter()
        .copied()
        .any(|bel| {
            architecture.bel_metadata(bel).bel_type == "TRELLIS_FF"
                && find_bel_pin(architecture.device(), bel, "M").is_some()
        });

    for (index, cell) in design.cells().iter().enumerate() {
        if cell.kind != ResourceKind::Register {
            continue;
        }
        let ff = CellId(index);
        let data_pin = cell
            .pins()
            .iter()
            .copied()
            .find(|pin| design.pins()[pin.0].name == "DI")
            .ok_or_else(|| PackingError::MissingFfDataPin {
                cell: cell.name.clone(),
            })?;
        ff_data_pins.insert(ff, data_pin);
    }

    for pair in pairs {
        let lut_name = design
            .cells()
            .get(pair.lut.0)
            .map_or_else(|| format!("cell#{}", pair.lut.0), |cell| cell.name.clone());
        let ff_name = design
            .cells()
            .get(pair.ff.0)
            .map_or_else(|| format!("cell#{}", pair.ff.0), |cell| cell.name.clone());
        let Some(&data_pin) = ff_data_pins.get(&pair.ff) else {
            return Err(PackingError::InvalidLutFfPair {
                lut: lut_name,
                ff: ff_name,
                reason: "pair member is not a register with a DI pin".into(),
            });
        };
        if lut_driver(design, data_pin) != Some(pair.lut) {
            return Err(PackingError::InvalidLutFfPair {
                lut: lut_name,
                ff: ff_name,
                reason: "LUT does not directly drive the FF data input".into(),
            });
        }
        if is_carry_slice(design, pair.lut) {
            return Err(PackingError::InvalidLutFfPair {
                lut: lut_name,
                ff: ff_name,
                reason: "carry slices cannot use the ordinary LUT/FF pair".into(),
            });
        }
        if !paired_luts.insert(pair.lut) || !paired_ffs.insert(pair.ff) {
            return Err(PackingError::InvalidLutFfPair {
                lut: lut_name,
                ff: ff_name,
                reason: "LUT or FF occurs in more than one pair".into(),
            });
        }
        if lut_ff_assignments.is_empty() {
            return Err(PackingError::InvalidLutFfPair {
                lut: lut_name,
                ff: ff_name,
                reason: "architecture has no compatible dedicated-path BEL pairs".into(),
            });
        }
        constraints.add_group_with_shared_assignments(
            [pair.lut, pair.ff],
            Arc::clone(&lut_ff_assignments),
        );
        lut_ff_pairs.push(pair);
    }

    let mut general_routing_ffs = Vec::new();
    for (ff, data_pin) in ff_data_pins {
        if paired_ffs.contains(&ff) {
            continue;
        }
        if !has_general_data_pin {
            return Err(PackingError::MissingGeneralDataPin {
                cell: design.cells()[ff.0].name.clone(),
            });
        }
        constraints.bind_pin_name(data_pin, "M");
        general_routing_ffs.push(ff);
    }

    Ok(Ecp5Packing {
        constraints,
        carry_pairs: Vec::new(),
        carry_pairs_packed: false,
        lut_ff_pairs,
        general_routing_ffs,
        block_rams: Vec::new(),
        block_rams_packed: false,
        global_clocks: Vec::new(),
        global_clocks_packed: false,
        io_attributes: BTreeMap::new(),
        clock_frequencies_hz: BTreeMap::new(),
        unsupported_lpf_commands: Vec::new(),
    })
}

fn is_carry_slice(design: &Design, cell: CellId) -> bool {
    design.cells()[cell.0]
        .pins()
        .iter()
        .any(|pin| design.pins()[pin.0].name == "FCO")
}

fn lut_driver(design: &Design, data_pin: CellPinId) -> Option<CellId> {
    let net = &design.nets()[design.pins()[data_pin.0].net()?.0];
    let driver = &design.pins()[net.driver.0];
    (driver.name == "F" && design.cells()[driver.cell.0].kind == ResourceKind::Lut(4))
        .then_some(driver.cell)
}

fn lut_ff_assignments(architecture: &Ecp5Architecture) -> Vec<Vec<BelId>> {
    let mut ff_by_slot = BTreeMap::new();
    for &ff_bel in architecture.device().bels_of_kind(ResourceKind::Register) {
        let metadata = architecture.bel_metadata(ff_bel);
        if metadata.bel_type == "TRELLIS_FF" {
            ff_by_slot.insert(
                (architecture.device().bels()[ff_bel.0].point, metadata.z),
                ff_bel,
            );
        }
    }
    let mut assignments = Vec::new();
    for &lut_bel in architecture.device().bels_of_kind(ResourceKind::Lut(4)) {
        let lut_metadata = architecture.bel_metadata(lut_bel);
        if lut_metadata.bel_type != "TRELLIS_COMB" {
            continue;
        }
        if let Some(ff_z) = lut_metadata.z.checked_add(1)
            && let Some(&ff_bel) =
                ff_by_slot.get(&(architecture.device().bels()[lut_bel.0].point, ff_z))
        {
            assignments.push(vec![lut_bel, ff_bel]);
        }
    }
    assignments
}

fn find_bel_pin(device: &Device, bel: BelId, name: &str) -> Option<BelPinId> {
    device.bels()[bel.0]
        .pins()
        .iter()
        .copied()
        .find(|pin| device.bel_pins()[pin.0].name == name)
}

/// ECP5 logic packing failed before placement.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PackingError {
    /// Unified graph candidate generation failed.
    Model(ModelError),
    /// A register did not expose the expected logical `DI` pin.
    MissingFfDataPin {
        /// Register cell name.
        cell: String,
    },
    /// No compatible physical FF exposed the general-routing `M` pin.
    MissingGeneralDataPin {
        /// Register cell name.
        cell: String,
    },
    /// An explicitly selected local LUT/FF pair is structurally invalid.
    InvalidLutFfPair {
        /// Requested LUT cell name.
        lut: String,
        /// Requested FF cell name.
        ff: String,
        /// Structural reason.
        reason: String,
    },
    /// Carry-pair packing was invoked more than once.
    CarryPairsAlreadyPacked,
    /// A carry requirement referenced an unknown cell.
    UnknownCarryCell(CellId),
    /// A carry requirement referenced a cell without the carry pin surface.
    CellIsNotCarrySlice {
        /// Logical cell name.
        cell: String,
    },
    /// One carry slice occurred in more than one pair.
    DuplicateCarryCell {
        /// Logical cell name.
        cell: String,
    },
    /// Carry net topology cannot be implemented by the dedicated chain.
    InvalidCarryConnection {
        /// Logical carry slice where the invalid connection originates.
        cell: String,
        /// Structural reason.
        reason: String,
    },
    /// No adjacent K0/K1 physical pair can implement a carry primitive.
    MissingCarrySlicePair {
        /// First logical slice name.
        cell: String,
    },
    /// A requirement referenced an unknown logical cell.
    UnknownBlockRamCell(CellId),
    /// A requirement referenced a cell that is not a memory.
    CellIsNotBlockRam {
        /// Logical cell name.
        cell: String,
    },
    /// One logical memory had more than one requirement.
    DuplicateBlockRamRequirement {
        /// Logical cell name.
        cell: String,
    },
    /// A logical memory had no structural requirement.
    MissingBlockRamRequirement {
        /// Logical cell name.
        cell: String,
    },
    /// Physical port width is not supported by `DP16KD`.
    InvalidBlockRamPhysicalWidth {
        /// Logical cell name.
        cell: String,
        /// Requested physical port width.
        physical_width: u8,
    },
    /// Logical word width is zero or exceeds the selected physical width.
    InvalidBlockRamWordWidth {
        /// Logical cell name.
        cell: String,
        /// Requested logical word width.
        word_width: u8,
        /// Selected physical port width.
        physical_width: u8,
    },
    /// Logical depth is zero or exceeds `DP16KD` capacity at this width.
    InvalidBlockRamDepth {
        /// Logical cell name.
        cell: String,
        /// Requested logical depth.
        depth: u32,
        /// Maximum legal depth at the selected physical width.
        max_depth: u32,
    },
    /// No compatible physical `DP16KD` BEL exists.
    MissingBlockRamBel {
        /// Logical cell name.
        cell: String,
    },
    /// BRAM packing was requested twice for the same packing result.
    BlockRamsAlreadyPacked,
    /// Stable WID assignment exceeded its representable range.
    TooManyBlockRams,
    /// One source net was selected for global promotion more than once.
    DuplicateGlobalClockRequirement {
        /// Logical source net name.
        net: String,
    },
    /// A selected net did not drive any recognized clock terminal.
    GlobalClockHasNoClockSinks {
        /// Logical source net name.
        net: String,
    },
    /// The architecture has too few compatible DCCA BELs.
    InsufficientGlobalClockBels {
        /// Number of selected global clock nets.
        required: usize,
        /// Number of compatible DCCA BELs.
        available: usize,
    },
    /// Global clock promotion was requested twice for one packing result.
    GlobalClocksAlreadyPacked,
    /// A promoted clock could not be mapped onto the exported spine/tap graph.
    GlobalClockRouting {
        /// Promoted logical net name.
        net: String,
        /// Physical topology or placement reason.
        reason: String,
    },
    /// Selected package is not present in the architecture snapshot.
    UnknownPackage(String),
    /// Package pin is not present in the selected package.
    UnknownPackagePin {
        /// Package name.
        package: String,
        /// Pin name.
        pin: String,
    },
    /// A binding referenced an unknown logical cell.
    UnknownIoCell(CellId),
    /// A package binding referenced a non-IO logical cell.
    CellIsNotIo {
        /// Logical cell name.
        cell: String,
    },
    /// One logical IO cell was constrained more than once in one call.
    DuplicateIoCell {
        /// Logical cell name.
        cell: String,
    },
    /// One package pin was assigned more than once in one call.
    DuplicatePackagePin(String),
    /// Logical IO pin surface is incompatible with the package's PIO BEL.
    IncompatiblePackagePin {
        /// Logical cell name.
        cell: String,
        /// Package name.
        package: String,
        /// Pin name.
        pin: String,
    },
    /// A later LPF application changed an existing IO attribute value.
    ConflictingIoAttribute {
        /// Logical IO cell name.
        cell: String,
        /// Attribute key.
        key: String,
    },
    /// A later LPF application changed an existing clock frequency.
    ConflictingClockFrequency {
        /// Logical IO cell name.
        cell: String,
    },
}

impl fmt::Display for PackingError {
    #[allow(clippy::too_many_lines)]
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Model(error) => write!(f, "invalid packing graph: {error}"),
            Self::MissingFfDataPin { cell } => {
                write!(f, "register `{cell}` has no logical DI pin")
            }
            Self::MissingGeneralDataPin { cell } => {
                write!(
                    f,
                    "register `{cell}` has no compatible general-routing M pin"
                )
            }
            Self::InvalidLutFfPair { lut, ff, reason } => {
                write!(f, "invalid LUT/FF pair `{lut}` -> `{ff}`: {reason}")
            }
            Self::CarryPairsAlreadyPacked => write!(f, "carry pairs were already packed"),
            Self::UnknownCarryCell(cell) => write!(f, "unknown carry cell ID {}", cell.0),
            Self::CellIsNotCarrySlice { cell } => {
                write!(f, "cell `{cell}` does not expose an ECP5 carry slice")
            }
            Self::DuplicateCarryCell { cell } => {
                write!(f, "carry slice `{cell}` occurs in more than one pair")
            }
            Self::InvalidCarryConnection { cell, reason } => {
                write!(
                    f,
                    "carry slice `{cell}` has an invalid connection: {reason}"
                )
            }
            Self::MissingCarrySlicePair { cell } => {
                write!(f, "carry slice `{cell}` has no compatible K0/K1 BEL pair")
            }
            Self::UnknownBlockRamCell(cell) => {
                write!(f, "unknown block RAM cell ID {}", cell.0)
            }
            Self::CellIsNotBlockRam { cell } => {
                write!(f, "cell `{cell}` is not a block RAM")
            }
            Self::DuplicateBlockRamRequirement { cell } => {
                write!(f, "block RAM `{cell}` has more than one requirement")
            }
            Self::MissingBlockRamRequirement { cell } => {
                write!(f, "block RAM `{cell}` has no structural requirement")
            }
            Self::InvalidBlockRamPhysicalWidth {
                cell,
                physical_width,
            } => write!(
                f,
                "block RAM `{cell}` has unsupported physical width {physical_width}"
            ),
            Self::InvalidBlockRamWordWidth {
                cell,
                word_width,
                physical_width,
            } => write!(
                f,
                "block RAM `{cell}` word width {word_width} is invalid for physical width {physical_width}"
            ),
            Self::InvalidBlockRamDepth {
                cell,
                depth,
                max_depth,
            } => write!(
                f,
                "block RAM `{cell}` depth {depth} exceeds the legal range 1..={max_depth}"
            ),
            Self::MissingBlockRamBel { cell } => {
                write!(f, "block RAM `{cell}` has no compatible DP16KD BEL")
            }
            Self::BlockRamsAlreadyPacked => write!(f, "block RAMs were already packed"),
            Self::TooManyBlockRams => write!(f, "too many block RAMs for stable WID assignment"),
            Self::DuplicateGlobalClockRequirement { net } => {
                write!(f, "clock net `{net}` was selected more than once")
            }
            Self::GlobalClockHasNoClockSinks { net } => {
                write!(f, "net `{net}` has no recognized ECP5 clock sinks")
            }
            Self::InsufficientGlobalClockBels {
                required,
                available,
            } => write!(
                f,
                "global clock promotion requires {required} DCCA BELs but only {available} are compatible"
            ),
            Self::GlobalClocksAlreadyPacked => {
                write!(f, "global clocks were already promoted")
            }
            Self::GlobalClockRouting { net, reason } => {
                write!(f, "global clock `{net}` cannot be routed: {reason}")
            }
            Self::UnknownPackage(package) => write!(f, "unknown ECP5 package `{package}`"),
            Self::UnknownPackagePin { package, pin } => {
                write!(f, "package `{package}` has no pin `{pin}`")
            }
            Self::UnknownIoCell(cell) => write!(f, "unknown IO cell ID {}", cell.0),
            Self::CellIsNotIo { cell } => write!(f, "cell `{cell}` is not an IO cell"),
            Self::DuplicateIoCell { cell } => {
                write!(f, "IO cell `{cell}` has more than one package pin")
            }
            Self::DuplicatePackagePin(pin) => {
                write!(f, "package pin `{pin}` is assigned more than once")
            }
            Self::IncompatiblePackagePin { cell, package, pin } => write!(
                f,
                "IO cell `{cell}` is incompatible with package `{package}` pin `{pin}`"
            ),
            Self::ConflictingIoAttribute { cell, key } => {
                write!(f, "IO cell `{cell}` has a conflicting `{key}` attribute")
            }
            Self::ConflictingClockFrequency { cell } => {
                write!(f, "IO cell `{cell}` has a conflicting clock frequency")
            }
        }
    }
}

impl Error for PackingError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Model(error) => Some(error),
            Self::MissingFfDataPin { .. }
            | Self::MissingGeneralDataPin { .. }
            | Self::InvalidLutFfPair { .. }
            | Self::CarryPairsAlreadyPacked
            | Self::UnknownCarryCell(_)
            | Self::CellIsNotCarrySlice { .. }
            | Self::DuplicateCarryCell { .. }
            | Self::InvalidCarryConnection { .. }
            | Self::MissingCarrySlicePair { .. }
            | Self::UnknownBlockRamCell(_)
            | Self::CellIsNotBlockRam { .. }
            | Self::DuplicateBlockRamRequirement { .. }
            | Self::MissingBlockRamRequirement { .. }
            | Self::InvalidBlockRamPhysicalWidth { .. }
            | Self::InvalidBlockRamWordWidth { .. }
            | Self::InvalidBlockRamDepth { .. }
            | Self::MissingBlockRamBel { .. }
            | Self::BlockRamsAlreadyPacked
            | Self::TooManyBlockRams
            | Self::DuplicateGlobalClockRequirement { .. }
            | Self::GlobalClockHasNoClockSinks { .. }
            | Self::InsufficientGlobalClockBels { .. }
            | Self::GlobalClocksAlreadyPacked
            | Self::GlobalClockRouting { .. }
            | Self::UnknownPackage(_)
            | Self::UnknownPackagePin { .. }
            | Self::UnknownIoCell(_)
            | Self::CellIsNotIo { .. }
            | Self::DuplicateIoCell { .. }
            | Self::DuplicatePackagePin(_)
            | Self::IncompatiblePackagePin { .. }
            | Self::ConflictingIoAttribute { .. }
            | Self::ConflictingClockFrequency { .. } => None,
        }
    }
}

impl From<ModelError> for PackingError {
    fn from(value: ModelError) -> Self {
        Self::Model(value)
    }
}

/// Reads and expands one versioned architecture snapshot.
///
/// # Errors
///
/// Returns JSON, schema, reference, or generic model errors.
pub fn read_architecture(reader: impl Read) -> Result<Ecp5Architecture, ImportError> {
    let file: ArchitectureFile = serde_json::from_reader(reader)?;
    expand(file)
}

/// Writes an expanded architecture cache without rebuilding its routing graph.
///
/// The cache is a Postcard-encoded, versioned representation of the complete
/// [`Ecp5Architecture`], including the physical routing adjacency index.
///
/// # Errors
///
/// Returns an error when binary serialization or output fails.
pub fn write_architecture_cache(
    writer: impl Write,
    architecture: &Ecp5Architecture,
) -> Result<(), ImportError> {
    postcard::to_io(
        &ArchitectureCacheRef {
            version: ARCHITECTURE_CACHE_VERSION,
            architecture,
        },
        writer,
    )?;
    Ok(())
}

/// Reads a previously expanded binary architecture cache.
///
/// # Errors
///
/// Returns an error for malformed binary data or an unsupported cache version.
pub fn read_architecture_cache(reader: impl Read) -> Result<Ecp5Architecture, ImportError> {
    let mut scratch = [0_u8; 16 * 1024];
    let (cache, _) = postcard::from_io((reader, &mut scratch))?;
    let ArchitectureCache {
        version,
        architecture,
    } = cache;
    if version != ARCHITECTURE_CACHE_VERSION {
        return Err(ImportError::UnsupportedCacheVersion(version));
    }
    Ok(architecture)
}

/// Expands an already decoded architecture snapshot.
///
/// # Errors
///
/// Returns schema, reference, or generic model errors.
pub fn expand(file: ArchitectureFile) -> Result<Ecp5Architecture, ImportError> {
    validate_header(&file)?;
    let mut device = Device::new(&file.device, file.width, file.height)?;
    let mut wires = BTreeMap::new();
    let mut bels = BTreeMap::new();
    let mut metadata_strings = StringInterner::default();
    let mut bel_metadata = Vec::new();
    let mut pip_metadata = Vec::new();
    let locations = indexed_locations(&file)?;
    let mut global_info = vec![None; (file.width * file.height) as usize];
    for location in &file.locations {
        global_info[(location.y * file.width + location.x) as usize] = Some(location.global);
    }
    let global_info = global_info.into_iter().collect::<Option<Vec<_>>>().ok_or(
        ImportError::IncompleteLocationGrid {
            expected: (file.width * file.height) as usize,
            actual: file.locations.len(),
        },
    )?;

    for location in &file.locations {
        let location_type = &file.location_types[location.location_type];
        for (index, wire) in location_type.wires.iter().enumerate() {
            let id = device.add_wire(
                qualified_name(location.x, location.y, &wire.name),
                Point::new(location.x, location.y),
                1,
            )?;
            if wires.insert((location.x, location.y, index), id).is_some() {
                return Err(ImportError::DuplicateResource {
                    x: location.x,
                    y: location.y,
                    index,
                });
            }
        }
    }

    for location in &file.locations {
        let location_type = &file.location_types[location.location_type];
        for (index, bel) in location_type.bels.iter().enumerate() {
            let id = device.add_bel(
                qualified_name(location.x, location.y, &bel.name),
                resource_kind(&bel.bel_type),
                Point::new(location.x, location.y),
            )?;
            bels.insert((location.x, location.y, index), id);
            debug_assert_eq!(id.0, bel_metadata.len());
            bel_metadata.push(CompactBelMetadata {
                bel_type: metadata_strings.intern(&bel.bel_type)?,
                z: bel.z,
            });
            for pin in &bel.pins {
                let wire = resolve_wire(location, pin.wire, &locations, &file, &wires)?;
                device.add_bel_pin(id, &pin.name, pin.direction.into(), wire)?;
            }
        }
    }

    for location in &file.locations {
        let location_type = &file.location_types[location.location_type];
        for pip in &location_type.pips {
            let from = resolve_wire(location, pip.from, &locations, &file, &wires)?;
            let to = resolve_wire(location, pip.to, &locations, &file, &wires)?;
            let id = device.add_pip(from, to, false, 1)?;
            debug_assert_eq!(id.0, pip_metadata.len());
            pip_metadata.push(CompactPipMetadata {
                fixed: pip.fixed,
                tile_type: metadata_strings.intern(&pip.tile_type)?,
                timing_class: metadata_strings.intern(&pip.timing_class)?,
                lutperm_flags: pip.lutperm_flags,
            });
        }
    }

    add_global_clock_aliases(
        &mut device,
        &global_info,
        &mut metadata_strings,
        &mut pip_metadata,
    )?;

    let packages = resolve_packages(&file.packages, &bels, &device)?;
    let speed_grades = file
        .speed_grades
        .into_iter()
        .map(|grade| (grade.name.clone(), grade))
        .collect();
    Ok(Ecp5Architecture {
        provenance: file.provenance,
        device,
        metadata_strings: metadata_strings.into_values(),
        bel_metadata,
        pip_metadata,
        global_info,
        packages,
        speed_grades,
    })
}

fn add_global_clock_aliases(
    device: &mut Device,
    global_info: &[GlobalInfoRecord],
    metadata_strings: &mut StringInterner,
    pip_metadata: &mut Vec<CompactPipMetadata>,
) -> Result<(), ImportError> {
    let wire_by_location_and_name = device
        .wires()
        .iter()
        .enumerate()
        .map(|(index, wire)| {
            (
                (wire.point, wire_basename(&wire.name).to_owned()),
                WireId(index),
            )
        })
        .collect::<BTreeMap<_, _>>();
    let wire_by_name = device
        .wires()
        .iter()
        .enumerate()
        .map(|(index, wire)| (wire_basename(&wire.name).to_owned(), WireId(index)))
        .collect::<BTreeMap<_, _>>();
    let mut incoming = vec![Vec::new(); device.wires().len()];
    for (index, pip) in device.pips().iter().enumerate() {
        incoming[pip.to().0].push(PipId(index));
    }
    let mut aliases = BTreeSet::new();

    for network in 0..ECP5_GLOBAL_CLOCK_COUNT {
        let tile_name = format!("G_HPBX{network:02}00");
        for ((point, name), &tile_wire) in &wire_by_location_and_name {
            if name != &tile_name {
                continue;
            }
            let info = global_info_at(device, global_info, *point)?;
            let tap_point = Point::new(info.tap_column, point.y);
            let tap_side = match info.tap_direction {
                GlobalTapDirection::Left => 'L',
                GlobalTapDirection::Right => 'R',
            };
            let tap_name = format!("{tap_side}_HPBX{network:02}00");
            let tap_wire = required_global_wire(
                &wire_by_location_and_name,
                tap_point,
                &tap_name,
                "logic-tile tap",
            )?;
            aliases.insert((tap_wire, tile_wire));

            let tap_pip = single_incoming_pip(device, &incoming, tap_wire, "tap driver")?;
            let tap_source = device.pips()[tap_pip.0].from();
            let tap_source_info =
                global_info_at(device, global_info, device.wires()[tap_source.0].point)?;
            let spine_point =
                tap_source_info
                    .spine
                    .ok_or_else(|| ImportError::InvalidGlobalTopology {
                        point: device.wires()[tap_source.0].point,
                        reason: "tap driver has no spine coordinate".into(),
                    })?;
            let spine_name = wire_basename(&device.wires()[tap_source.0].name);
            let spine_wire = required_global_wire(
                &wire_by_location_and_name,
                spine_point,
                spine_name,
                "spine transmitter",
            )?;
            aliases.insert((spine_wire, tap_source));

            let spine_pip = single_incoming_pip(device, &incoming, spine_wire, "spine driver")?;
            let spine_source = device.pips()[spine_pip.0].from();
            let root_name = format!("G_{}PCLK{network}", tap_source_info.quadrant.wire_prefix());
            let root = wire_by_name.get(&root_name).copied().ok_or_else(|| {
                ImportError::InvalidGlobalTopology {
                    point: spine_point,
                    reason: format!("missing quadrant root `{root_name}`"),
                }
            })?;
            aliases.insert((root, spine_source));
        }
    }

    let tile_type = metadata_strings.intern("TEXO_GLOBAL_ALIAS")?;
    let timing_class = metadata_strings.intern("zero")?;
    for (from, to) in aliases {
        let id = device.add_pip(from, to, false, 1)?;
        debug_assert_eq!(id.0, pip_metadata.len());
        pip_metadata.push(CompactPipMetadata {
            fixed: true,
            tile_type,
            timing_class,
            lutperm_flags: 0,
        });
    }
    Ok(())
}

fn wire_basename(name: &str) -> &str {
    name.rsplit('/').next().unwrap_or(name)
}

fn global_info_at(
    device: &Device,
    global_info: &[GlobalInfoRecord],
    point: Point,
) -> Result<GlobalInfoRecord, ImportError> {
    global_info
        .get((point.y * device.width() + point.x) as usize)
        .copied()
        .ok_or_else(|| ImportError::InvalidGlobalTopology {
            point,
            reason: "coordinate is outside the global-information table".into(),
        })
}

fn required_global_wire(
    wires: &BTreeMap<(Point, String), WireId>,
    point: Point,
    name: &str,
    role: &str,
) -> Result<WireId, ImportError> {
    wires
        .get(&(point, name.to_owned()))
        .copied()
        .ok_or_else(|| ImportError::InvalidGlobalTopology {
            point,
            reason: format!("missing {role} wire `{name}`"),
        })
}

fn single_incoming_pip(
    device: &Device,
    incoming: &[Vec<PipId>],
    wire: WireId,
    role: &str,
) -> Result<PipId, ImportError> {
    match incoming[wire.0].as_slice() {
        [pip] => Ok(*pip),
        pips => Err(ImportError::InvalidGlobalTopology {
            point: device.wires()[wire.0].point,
            reason: format!(
                "{role} wire `{}` has {} incoming PIPs instead of one",
                device.wires()[wire.0].name,
                pips.len()
            ),
        }),
    }
}

#[derive(Default)]
struct StringInterner {
    ids: BTreeMap<String, u32>,
    values: Vec<String>,
}

impl StringInterner {
    fn intern(&mut self, value: &str) -> Result<u32, ImportError> {
        if let Some(&id) = self.ids.get(value) {
            return Ok(id);
        }
        let id =
            u32::try_from(self.values.len()).map_err(|_| ImportError::TooManyMetadataStrings)?;
        let value = value.to_owned();
        self.values.push(value.clone());
        self.ids.insert(value, id);
        Ok(id)
    }

    fn into_values(self) -> Vec<String> {
        self.values
    }
}

fn validate_header(file: &ArchitectureFile) -> Result<(), ImportError> {
    if file.schema_version != SCHEMA_VERSION {
        return Err(ImportError::UnsupportedSchema(file.schema_version));
    }
    if file.family != "ECP5" {
        return Err(ImportError::WrongFamily(file.family.clone()));
    }
    if file.provenance.project_trellis_revision.is_empty()
        || file.provenance.database_revision.is_empty()
    {
        return Err(ImportError::MissingProvenance);
    }
    if !file.provenance.split_slice_mode {
        return Err(ImportError::SplitSliceRequired);
    }
    validate_timing(file)?;
    for location in &file.locations {
        if location.location_type >= file.location_types.len() {
            return Err(ImportError::UnknownLocationType(location.location_type));
        }
        if location.x >= file.width || location.y >= file.height {
            return Err(ImportError::LocationOutsideDevice {
                x: location.x,
                y: location.y,
            });
        }
        if location.global.tap_column >= file.width
            || location
                .global
                .spine
                .is_some_and(|point| point.x >= file.width || point.y >= file.height)
        {
            return Err(ImportError::InvalidGlobalTopology {
                point: Point::new(location.x, location.y),
                reason: "tap or spine coordinate is outside the device".into(),
            });
        }
    }
    if file.locations.len() != (file.width * file.height) as usize {
        return Err(ImportError::IncompleteLocationGrid {
            expected: (file.width * file.height) as usize,
            actual: file.locations.len(),
        });
    }
    Ok(())
}

fn validate_timing(file: &ArchitectureFile) -> Result<(), ImportError> {
    if file.speed_grades.is_empty() {
        return Err(ImportError::MissingSpeedGrades);
    }
    let used_classes = file
        .location_types
        .iter()
        .flat_map(|location| location.pips.iter().map(|pip| pip.timing_class.as_str()))
        .collect::<BTreeSet<_>>();
    let mut grade_names = BTreeSet::new();
    for grade in &file.speed_grades {
        if grade.name.is_empty() || !grade_names.insert(grade.name.as_str()) {
            return Err(ImportError::DuplicateSpeedGrade(grade.name.clone()));
        }
        for class in &used_classes {
            if !grade.pip_classes.contains_key(*class) {
                return Err(ImportError::MissingPipTimingClass {
                    speed_grade: grade.name.clone(),
                    timing_class: (*class).into(),
                });
            }
        }
        let mut cell_types = BTreeSet::new();
        for cell in &grade.cells {
            if cell.cell_type.is_empty() || !cell_types.insert(cell.cell_type.as_str()) {
                return Err(ImportError::DuplicateCellTiming {
                    speed_grade: grade.name.clone(),
                    cell_type: cell.cell_type.clone(),
                });
            }
            for arc in &cell.arcs {
                validate_delay_range(&grade.name, &cell.cell_type, arc.delay)?;
            }
            for check in &cell.setup_holds {
                validate_delay_range(&grade.name, &cell.cell_type, check.setup)?;
                validate_delay_range(&grade.name, &cell.cell_type, check.hold)?;
            }
        }
    }
    Ok(())
}

fn validate_delay_range(
    speed_grade: &str,
    subject: &str,
    delay: DelayRangeRecord,
) -> Result<(), ImportError> {
    if delay.min_ps > delay.max_ps {
        Err(ImportError::InvalidTimingRange {
            speed_grade: speed_grade.into(),
            subject: subject.into(),
        })
    } else {
        Ok(())
    }
}

fn indexed_locations(file: &ArchitectureFile) -> Result<BTreeMap<(u32, u32), usize>, ImportError> {
    let mut locations = BTreeMap::new();
    for location in &file.locations {
        if locations
            .insert((location.x, location.y), location.location_type)
            .is_some()
        {
            return Err(ImportError::DuplicateLocation {
                x: location.x,
                y: location.y,
            });
        }
    }
    Ok(locations)
}

fn resolve_wire(
    base: &LocationRecord,
    relative: RelativeRef,
    locations: &BTreeMap<(u32, u32), usize>,
    file: &ArchitectureFile,
    wires: &BTreeMap<(u32, u32, usize), WireId>,
) -> Result<WireId, ImportError> {
    let x = i64::from(base.x) + i64::from(relative.dx);
    let y = i64::from(base.y) + i64::from(relative.dy);
    let x = u32::try_from(x).map_err(|_| ImportError::RelativeReferenceOutsideDevice)?;
    let y = u32::try_from(y).map_err(|_| ImportError::RelativeReferenceOutsideDevice)?;
    if x >= file.width || y >= file.height || !locations.contains_key(&(x, y)) {
        return Err(ImportError::RelativeReferenceOutsideDevice);
    }
    wires
        .get(&(x, y, relative.index))
        .copied()
        .ok_or(ImportError::UnknownWireReference {
            x,
            y,
            index: relative.index,
        })
}

fn resolve_packages(
    records: &[PackageRecord],
    bels: &BTreeMap<(u32, u32, usize), BelId>,
    device: &Device,
) -> Result<Vec<Package>, ImportError> {
    records
        .iter()
        .map(|package| {
            let mut pins = BTreeMap::new();
            for pin in &package.pins {
                let bel = bels.get(&(pin.x, pin.y, pin.bel)).copied().ok_or_else(|| {
                    ImportError::UnknownPackageBel {
                        package: package.name.clone(),
                        pin: pin.name.clone(),
                    }
                })?;
                if device.bels()[bel.0].kind != ResourceKind::Io {
                    return Err(ImportError::PackageBelIsNotIo {
                        package: package.name.clone(),
                        pin: pin.name.clone(),
                    });
                }
                if pins.insert(pin.name.clone(), bel).is_some() {
                    return Err(ImportError::DuplicatePackagePin {
                        package: package.name.clone(),
                        pin: pin.name.clone(),
                    });
                }
            }
            Ok(Package {
                name: package.name.clone(),
                pins,
            })
        })
        .collect()
}

fn resource_kind(bel_type: &str) -> ResourceKind {
    match bel_type {
        "TRELLIS_COMB" => ResourceKind::Lut(4),
        "TRELLIS_FF" => ResourceKind::Register,
        "DP16KD" => ResourceKind::Memory,
        "DCCA" => ResourceKind::Clock,
        "PIO" | "TRELLIS_IO" => ResourceKind::Io,
        _ => ResourceKind::Logic,
    }
}

fn qualified_name(x: u32, y: u32, local: &str) -> String {
    format!("R{y}C{x}/{local}")
}

/// Invalid versioned ECP5 architecture snapshot.
#[derive(Debug)]
pub enum ImportError {
    /// JSON decoding failed.
    Json(serde_json::Error),
    /// Expanded binary cache encoding or decoding failed.
    Binary(postcard::Error),
    /// Generic target model construction failed.
    Model(ModelError),
    /// File uses an unsupported schema version.
    UnsupportedSchema(u32),
    /// Binary cache uses an unsupported schema version.
    UnsupportedCacheVersion(u32),
    /// File describes another FPGA family.
    WrongFamily(String),
    /// One or both source revisions were omitted.
    MissingProvenance,
    /// Fine-grained BEL mode is mandatory for Texo.
    SplitSliceRequired,
    /// Architecture metadata contained more strings than a compact ID can address.
    TooManyMetadataStrings,
    /// Architecture snapshot did not contain any speed-grade timing table.
    MissingSpeedGrades,
    /// A speed-grade name was empty or repeated.
    DuplicateSpeedGrade(String),
    /// A routed PIP class had no timing coefficients in one speed grade.
    MissingPipTimingClass {
        /// Speed-grade name.
        speed_grade: String,
        /// Missing timing class.
        timing_class: String,
    },
    /// A split cell type had more than one timing record in one speed grade.
    DuplicateCellTiming {
        /// Speed-grade name.
        speed_grade: String,
        /// Repeated cell type.
        cell_type: String,
    },
    /// A timing range had a minimum greater than its maximum.
    InvalidTimingRange {
        /// Speed-grade name.
        speed_grade: String,
        /// Timing object being validated.
        subject: String,
    },
    /// A location referenced a missing location type.
    UnknownLocationType(usize),
    /// A location coordinate exceeded the declared dimensions.
    LocationOutsideDevice {
        /// X coordinate.
        x: u32,
        /// Y coordinate.
        y: u32,
    },
    /// The same physical coordinate appeared twice.
    DuplicateLocation {
        /// X coordinate.
        x: u32,
        /// Y coordinate.
        y: u32,
    },
    /// The dense device grid omitted one or more coordinates.
    IncompleteLocationGrid {
        /// Width multiplied by height.
        expected: usize,
        /// Number of unique location records supplied.
        actual: usize,
    },
    /// ECP5 quadrant, tap, or spine metadata was inconsistent with the graph.
    InvalidGlobalTopology {
        /// Coordinate at which reconstruction failed.
        point: Point,
        /// Specific topology invariant that failed.
        reason: String,
    },
    /// A resource was repeated within one coordinate.
    DuplicateResource {
        /// X coordinate.
        x: u32,
        /// Y coordinate.
        y: u32,
        /// Type-local resource index.
        index: usize,
    },
    /// A relative reference left the declared device or sparse location map.
    RelativeReferenceOutsideDevice,
    /// A relative wire index was invalid.
    UnknownWireReference {
        /// X coordinate.
        x: u32,
        /// Y coordinate.
        y: u32,
        /// Wire index.
        index: usize,
    },
    /// A package pin referenced a missing PIO BEL.
    UnknownPackageBel {
        /// Package name.
        package: String,
        /// Pin name.
        pin: String,
    },
    /// A package pin resolved to a BEL that is not an IO resource.
    PackageBelIsNotIo {
        /// Package name.
        package: String,
        /// Pin name.
        pin: String,
    },
    /// A package repeated a pin name.
    DuplicatePackagePin {
        /// Package name.
        package: String,
        /// Pin name.
        pin: String,
    },
}

impl fmt::Display for ImportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Json(error) => write!(f, "invalid architecture JSON: {error}"),
            Self::Binary(error) => write!(f, "invalid architecture cache: {error}"),
            Self::Model(error) => write!(f, "invalid physical model: {error}"),
            Self::UnsupportedSchema(version) => {
                write!(f, "unsupported architecture schema version {version}")
            }
            Self::UnsupportedCacheVersion(version) => {
                write!(f, "unsupported architecture cache version {version}")
            }
            Self::WrongFamily(family) => write!(f, "expected ECP5 family, found `{family}`"),
            Self::MissingProvenance => write!(f, "architecture provenance is incomplete"),
            Self::SplitSliceRequired => write!(f, "split-slice Project Trellis data is required"),
            Self::TooManyMetadataStrings => {
                write!(
                    f,
                    "architecture contains too many distinct metadata strings"
                )
            }
            Self::MissingSpeedGrades => {
                write!(f, "architecture contains no speed-grade timing tables")
            }
            Self::DuplicateSpeedGrade(speed_grade) => {
                write!(f, "duplicate or empty speed grade `{speed_grade}`")
            }
            Self::MissingPipTimingClass {
                speed_grade,
                timing_class,
            } => write!(
                f,
                "speed grade `{speed_grade}` has no PIP timing class `{timing_class}`"
            ),
            Self::DuplicateCellTiming {
                speed_grade,
                cell_type,
            } => write!(
                f,
                "speed grade `{speed_grade}` repeats cell timing `{cell_type}`"
            ),
            Self::InvalidTimingRange {
                speed_grade,
                subject,
            } => write!(
                f,
                "speed grade `{speed_grade}` has an invalid timing range for {subject}"
            ),
            Self::UnknownLocationType(index) => write!(f, "unknown location type {index}"),
            Self::LocationOutsideDevice { x, y } => {
                write!(f, "location ({x}, {y}) is outside the device")
            }
            Self::DuplicateLocation { x, y } => write!(f, "duplicate location ({x}, {y})"),
            Self::IncompleteLocationGrid { expected, actual } => write!(
                f,
                "architecture location grid has {actual} entries, expected {expected}"
            ),
            Self::InvalidGlobalTopology { point, reason } => write!(
                f,
                "invalid ECP5 global-clock topology at ({}, {}): {reason}",
                point.x, point.y
            ),
            Self::DuplicateResource { x, y, index } => {
                write!(f, "duplicate resource {index} at ({x}, {y})")
            }
            Self::RelativeReferenceOutsideDevice => {
                write!(f, "relative resource reference leaves the device")
            }
            Self::UnknownWireReference { x, y, index } => {
                write!(f, "unknown wire {index} at ({x}, {y})")
            }
            Self::UnknownPackageBel { package, pin } => {
                write!(
                    f,
                    "package `{package}` pin `{pin}` refers to an unknown BEL"
                )
            }
            Self::PackageBelIsNotIo { package, pin } => {
                write!(
                    f,
                    "package `{package}` pin `{pin}` does not refer to an IO BEL"
                )
            }
            Self::DuplicatePackagePin { package, pin } => {
                write!(f, "package `{package}` repeats pin `{pin}`")
            }
        }
    }
}

impl Error for ImportError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Json(error) => Some(error),
            Self::Binary(error) => Some(error),
            Self::Model(error) => Some(error),
            _ => None,
        }
    }
}

impl From<serde_json::Error> for ImportError {
    fn from(value: serde_json::Error) -> Self {
        Self::Json(value)
    }
}

impl From<postcard::Error> for ImportError {
    fn from(value: postcard::Error) -> Self {
        Self::Binary(value)
    }
}

impl From<ModelError> for ImportError {
    fn from(value: ModelError) -> Self {
        Self::Model(value)
    }
}

#[cfg(test)]
mod tests {
    use std::num::NonZeroU32;

    use struo_ir::{
        ActiveLevel as StruoActiveLevel, ArithmeticOp, ClockEdge as StruoClockEdge, EnableControl,
        MemoryCell, Netlist, RegisterCell,
    };
    use struo_target_ecp5::{
        ArithmeticMapping, MappingOptions, map_to_ecp5, map_to_ecp5_with_options,
    };
    use texo_model::{
        BelId, CellId, Design, PinDirection, PipId, ResourceKind, UnifiedGraph, WireId,
    };
    use texo_pnr::{
        place_and_route_with_constraints, place_with_constraints, swap_placement_cells,
    };
    use texo_struo::{PrimitiveMetadata, import_ecp5};

    use super::{
        ArchitectureFile, BlockRamRequirement, CompactIncomingPips, GlobalClockRequirement,
        ImportError, LogicalPort, LutFfPair, PackagePinBinding, PackedBlockRam, PackingError,
        PipMetadata, expand, find_bel_pin, find_global_clock_requirements, pack_lut_ffs,
        pack_lut_ffs_excluding, pack_lut_ffs_with_pairs, parse_lpf, read_architecture,
        read_architecture_cache, resolve_lpf_port_cells, resolve_lpf_ports,
        write_architecture_cache,
    };

    const FIXTURE: &str = include_str!("../fixtures/minimal-ecp5.json");

    #[test]
    fn expands_deduplicated_locations_and_package_pins() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();

        assert_eq!(architecture.device().name(), "LFE5UM5G-85F-test");
        assert_eq!(architecture.device().bels().len(), 12);
        assert_eq!(architecture.device().wires().len(), 63);
        assert_eq!(architecture.device().pips().len(), 14);
        assert_eq!(architecture.packages()[0].pins.len(), 3);
        assert_eq!(
            architecture.pip_metadata(PipId(0)),
            PipMetadata {
                fixed: false,
                tile_type: "PLC2",
                timing_class: "default",
                lutperm_flags: 0,
            }
        );
        assert!(architecture.speed_grades().contains_key("6"));
    }

    #[test]
    fn round_trips_the_expanded_architecture_cache() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut encoded = Vec::new();

        write_architecture_cache(&mut encoded, &architecture).unwrap();
        let decoded = read_architecture_cache(encoded.as_slice()).unwrap();

        assert_eq!(decoded, architecture);
    }

    #[test]
    fn rejects_a_speed_grade_missing_a_used_pip_class() {
        let mut file: ArchitectureFile = serde_json::from_str(FIXTURE).unwrap();
        file.speed_grades[0].pip_classes.remove("default");

        assert!(matches!(
            expand(file),
            Err(ImportError::MissingPipTimingClass { .. })
        ));
    }

    #[test]
    fn preserves_independently_fitted_non_monotonic_pip_corners() {
        let mut file: ArchitectureFile = serde_json::from_str(FIXTURE).unwrap();
        let timing = file.speed_grades[0].pip_classes.get_mut("default").unwrap();
        timing.base.min_ps = 59;
        timing.base.typ_ps = 54;
        timing.base.max_ps = 48;

        let architecture = expand(file).unwrap();
        let timing = &architecture.speed_grades()["6"].pip_classes["default"];
        assert_eq!(
            (timing.base.min_ps, timing.base.typ_ps, timing.base.max_ps),
            (59, 54, 48)
        );
    }

    #[test]
    fn compact_pip_timing_class_ids_match_resolved_metadata() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let compact = architecture.pip_timing_class_ids().collect::<Vec<_>>();
        let resolved = architecture
            .pip_metadata_iter()
            .map(|(_, metadata)| metadata.timing_class)
            .collect::<Vec<_>>();

        assert_eq!(compact.len(), resolved.len());
        assert!(
            compact
                .iter()
                .zip(resolved)
                .all(|(&id, name)| { architecture.metadata_string_by_id(id) == Some(name) })
        );
    }

    #[test]
    fn compact_incoming_pips_preserve_stable_device_order() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let device = architecture.device();
        let incoming = CompactIncomingPips::new(device);

        for wire in 0..device.wires().len() {
            let expected = device
                .pips()
                .iter()
                .enumerate()
                .filter(|(_, pip)| pip.to() == WireId(wire))
                .map(|(index, _)| u32::try_from(index).unwrap())
                .collect::<Vec<_>>();
            assert_eq!(incoming.for_wire(WireId(wire)), expected);
        }
    }

    #[test]
    fn imported_struo_lut_has_a_real_trellis_comb_candidate() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut source = Netlist::new("logic");
        let lhs = source.add_input("lhs");
        let rhs = source.add_input("rhs");
        let value = source.add_xor(lhs, rhs);
        source.add_output("value", value);
        let mapped = map_to_ecp5(&source).unwrap();
        let imported = import_ecp5(&mapped).unwrap();
        let lut = imported
            .design()
            .cells()
            .iter()
            .position(|cell| cell.kind == ResourceKind::Lut(4))
            .map(CellId)
            .unwrap();
        let graph = UnifiedGraph::new(imported.design(), architecture.device());

        assert_eq!(graph.placement_candidates(lut).unwrap().len(), 4);
        for (index, cell) in imported.design().cells().iter().enumerate() {
            if cell.kind == ResourceKind::Io {
                assert_eq!(graph.placement_candidates(CellId(index)).unwrap().len(), 3);
            }
        }

        let parsed = parse_lpf(b"LOCATE COMP lhs SITE A10;".as_slice()).unwrap();
        let resolved = resolve_lpf_port_cells(
            &parsed,
            imported
                .ports()
                .iter()
                .map(|port| (port.name.as_str(), port.bits.as_slice())),
            true,
        )
        .unwrap();
        let lhs_cell = imported
            .ports()
            .iter()
            .find(|port| port.name == "lhs")
            .unwrap()
            .bits[0];
        assert_eq!(resolved.package_pins[0].cell, lhs_cell);

        let mut packing = pack_lut_ffs(imported.design(), &architecture).unwrap();
        packing
            .apply_resolved_lpf(imported.design(), &architecture, "CABGA381", &resolved)
            .unwrap();
        assert_eq!(packing.constraints().groups().len(), 1);
    }

    #[test]
    fn packs_a_lut_driven_ff_into_matching_z_slots() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let lut = design.add_cell("lut", ResourceKind::Lut(4));
        for name in ["A", "B", "C", "D"] {
            design.add_pin(lut, name, PinDirection::Input).unwrap();
        }
        let lut_output = design.add_pin(lut, "F", PinDirection::Output).unwrap();
        let ff = add_ff(&mut design, "ff");
        let ff_data = design.cells()[ff.0]
            .pins()
            .iter()
            .copied()
            .find(|pin| design.pins()[pin.0].name == "DI")
            .unwrap();
        design.add_net("lut_to_ff", lut_output, [ff_data]).unwrap();

        let packing = pack_lut_ffs(&design, &architecture).unwrap();
        let placement =
            place_with_constraints(&design, architecture.device(), packing.constraints()).unwrap();
        let lut_bel = placement.bel(lut).unwrap();
        let ff_bel = placement.bel(ff).unwrap();

        assert_eq!(packing.lut_ff_pairs().len(), 1);
        assert!(packing.general_routing_ffs().is_empty());
        assert_eq!(
            architecture.device().bels()[lut_bel.0].point,
            architecture.device().bels()[ff_bel.0].point
        );
        assert_eq!(
            architecture.bel_metadata(lut_bel).z + 1,
            architecture.bel_metadata(ff_bel).z
        );
    }

    #[test]
    fn releases_a_dedicated_lut_ff_edge_for_general_routing() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let lut = design.add_cell("lut", ResourceKind::Lut(4));
        for name in ["A", "B", "C", "D"] {
            design.add_pin(lut, name, PinDirection::Input).unwrap();
        }
        let lut_output = design.add_pin(lut, "F", PinDirection::Output).unwrap();
        let ff = add_ff(&mut design, "ff");
        let ff_data = design.cells()[ff.0]
            .pins()
            .iter()
            .copied()
            .find(|pin| design.pins()[pin.0].name == "DI")
            .unwrap();
        design.add_net("lut_to_ff", lut_output, [ff_data]).unwrap();
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();
        let placement =
            place_with_constraints(&design, architecture.device(), packing.constraints()).unwrap();

        packing.release_lut_ff_pair(&design, lut, ff).unwrap();
        let rebound = texo_pnr::rebind_placement_pins(
            &design,
            architecture.device(),
            packing.constraints(),
            &placement,
        )
        .unwrap();
        let rebound_pin = rebound.pin_binding(ff_data).unwrap();

        assert!(packing.lut_ff_pairs().is_empty());
        assert_eq!(packing.general_routing_ffs(), &[ff]);
        assert!(packing.constraints().groups().is_empty());
        assert_eq!(
            packing.constraints().pin_name_bindings().get(&ff_data),
            Some(&"M".to_owned())
        );
        assert_eq!(architecture.device().bel_pins()[rebound_pin.0].name, "M");
    }

    #[test]
    fn excluded_lut_keeps_its_ff_on_general_routing() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let lut = design.add_cell("constant_lut", ResourceKind::Lut(4));
        for name in ["A", "B", "C", "D"] {
            design.add_pin(lut, name, PinDirection::Input).unwrap();
        }
        let lut_output = design.add_pin(lut, "F", PinDirection::Output).unwrap();
        let ff = add_ff(&mut design, "ff");
        let ff_data = design.cells()[ff.0]
            .pins()
            .iter()
            .copied()
            .find(|pin| design.pins()[pin.0].name == "DI")
            .unwrap();
        design
            .add_net("constant_to_ff", lut_output, [ff_data])
            .unwrap();

        let packing = pack_lut_ffs_excluding(&design, &architecture, [lut]).unwrap();

        assert!(packing.lut_ff_pairs().is_empty());
        assert_eq!(packing.general_routing_ffs(), &[ff]);
        assert_eq!(
            packing.constraints().pin_name_bindings().get(&ff_data),
            Some(&"M".to_owned())
        );
    }

    #[test]
    fn explicit_lut_ff_pair_selects_one_fanout_sink() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let lut = design.add_cell("lut", ResourceKind::Lut(4));
        for name in ["A", "B", "C", "D"] {
            design.add_pin(lut, name, PinDirection::Input).unwrap();
        }
        let lut_output = design.add_pin(lut, "F", PinDirection::Output).unwrap();
        let first = add_ff(&mut design, "first");
        let selected = add_ff(&mut design, "selected");
        let data_pins = [first, selected].map(|ff| {
            design.cells()[ff.0]
                .pins()
                .iter()
                .copied()
                .find(|pin| design.pins()[pin.0].name == "DI")
                .unwrap()
        });
        design.add_net("lut_to_ffs", lut_output, data_pins).unwrap();

        let packing =
            pack_lut_ffs_with_pairs(&design, &architecture, [LutFfPair { lut, ff: selected }])
                .unwrap();

        assert_eq!(packing.lut_ff_pairs(), &[LutFfPair { lut, ff: selected }]);
        assert_eq!(packing.general_routing_ffs(), &[first]);
        assert_eq!(
            packing.constraints().pin_name_bindings().get(&data_pins[0]),
            Some(&"M".to_owned())
        );
    }

    #[test]
    fn reassigns_a_dedicated_lut_ff_edge_atomically() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let lut = design.add_cell("lut", ResourceKind::Lut(4));
        for name in ["A", "B", "C", "D"] {
            design.add_pin(lut, name, PinDirection::Input).unwrap();
        }
        let lut_output = design.add_pin(lut, "F", PinDirection::Output).unwrap();
        let old_ff = add_ff(&mut design, "old");
        let new_ff = add_ff(&mut design, "new");
        let data_pins = [old_ff, new_ff].map(|ff| {
            design.cells()[ff.0]
                .pins()
                .iter()
                .copied()
                .find(|pin| design.pins()[pin.0].name == "DI")
                .unwrap()
        });
        design.add_net("lut_to_ffs", lut_output, data_pins).unwrap();
        let mut packing =
            pack_lut_ffs_with_pairs(&design, &architecture, [LutFfPair { lut, ff: old_ff }])
                .unwrap();
        let placement =
            place_with_constraints(&design, architecture.device(), packing.constraints()).unwrap();
        let old_bel = placement.bel(old_ff).unwrap();
        let new_bel = placement.bel(new_ff).unwrap();

        assert_eq!(
            packing.reassign_lut_ff_pair(&design, lut, new_ff),
            Ok(old_ff)
        );
        let swapped = swap_placement_cells(
            &design,
            architecture.device(),
            packing.constraints(),
            &placement,
            old_ff,
            new_ff,
        )
        .unwrap();
        assert_eq!(packing.lut_ff_pairs(), &[LutFfPair { lut, ff: new_ff }]);
        assert_eq!(packing.general_routing_ffs(), &[old_ff]);
        assert_eq!(packing.constraints().groups()[0].cells, [lut, new_ff]);
        assert_eq!(swapped.bel(old_ff), Some(new_bel));
        assert_eq!(swapped.bel(new_ff), Some(old_bel));
        assert_eq!(
            packing.constraints().pin_name_bindings().get(&data_pins[0]),
            Some(&"M".to_owned())
        );
        assert!(
            !packing
                .constraints()
                .pin_name_bindings()
                .contains_key(&data_pins[1])
        );
    }

    #[test]
    fn packs_a_split_ccu2c_into_one_physical_slice() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut source = Netlist::new("carry");
        let width = NonZeroU32::new(2).unwrap();
        let lhs = source.add_input_port("lhs", width);
        let rhs = source.add_input_port("rhs", width);
        let sum = source
            .add_arithmetic(ArithmeticOp::Add, &lhs, &rhs)
            .unwrap();
        source.add_output_port("sum", &sum).unwrap();
        let mapped = map_to_ecp5_with_options(
            &source,
            MappingOptions {
                arithmetic: ArithmeticMapping::CarryChain,
                ..MappingOptions::default()
            },
        )
        .unwrap();
        let imported = import_ecp5(&mapped).unwrap();
        let pair = imported.carry_pairs()[0];
        let mut packing = pack_lut_ffs(imported.design(), &architecture).unwrap();

        packing
            .pack_carry_pairs(
                imported.design(),
                &architecture,
                imported.carry_pairs().iter().take(1).copied(),
            )
            .unwrap();
        assert_eq!(packing.carry_pairs(), &[pair]);
        let group = packing
            .constraints()
            .groups()
            .iter()
            .find(|group| group.cells == pair)
            .unwrap();
        assert!(!group.assignments.is_empty());
        for assignment in group.assignments.iter() {
            let [first, second] = assignment.as_slice() else {
                panic!("carry assignment must contain two BELs")
            };
            assert_eq!(
                architecture.device().bels()[first.0].point,
                architecture.device().bels()[second.0].point
            );
            assert_eq!(
                architecture.bel_metadata(*first).z + 4,
                architecture.bel_metadata(*second).z
            );
        }
    }

    #[test]
    fn packs_connected_ccu2c_pairs_as_one_physical_carry_chain() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut source = Netlist::new("carry_chain");
        let width = NonZeroU32::new(4).unwrap();
        let lhs = source.add_input_port("lhs", width);
        let rhs = source.add_input_port("rhs", width);
        let sum = source
            .add_arithmetic(ArithmeticOp::Add, &lhs, &rhs)
            .unwrap();
        source.add_output_port("sum", &sum).unwrap();
        let mapped = map_to_ecp5_with_options(
            &source,
            MappingOptions {
                arithmetic: ArithmeticMapping::CarryChain,
                ..MappingOptions::default()
            },
        )
        .unwrap();
        let imported = import_ecp5(&mapped).unwrap();
        let mut packing = pack_lut_ffs(imported.design(), &architecture).unwrap();

        packing
            .pack_carry_pairs(
                imported.design(),
                &architecture,
                imported.carry_pairs().iter().take(2).copied(),
            )
            .unwrap();

        assert_eq!(imported.carry_pairs().len(), 3);
        let cells = imported
            .carry_pairs()
            .iter()
            .take(2)
            .flatten()
            .copied()
            .collect::<Vec<_>>();
        let group = packing
            .constraints()
            .groups()
            .iter()
            .find(|group| group.cells == cells)
            .unwrap();
        assert!(!group.assignments.is_empty());
        for assignment in group.assignments.iter() {
            let first_fco = find_bel_pin(architecture.device(), assignment[1], "FCO").unwrap();
            let second_fci = find_bel_pin(architecture.device(), assignment[2], "FCI").unwrap();
            assert_eq!(
                architecture.device().bel_pins()[first_fco.0].wire,
                architecture.device().bel_pins()[second_fci.0].wire
            );
        }
    }

    #[test]
    fn binds_an_unpaired_ff_to_the_general_routing_input() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let ff = add_ff(&mut design, "standalone_ff");
        let data_pin = design.cells()[ff.0]
            .pins()
            .iter()
            .copied()
            .find(|pin| design.pins()[pin.0].name == "DI")
            .unwrap();

        let packing = pack_lut_ffs(&design, &architecture).unwrap();
        let placement =
            place_with_constraints(&design, architecture.device(), packing.constraints()).unwrap();
        let physical_pin = placement.pin_binding(data_pin).unwrap();

        assert!(packing.lut_ff_pairs().is_empty());
        assert_eq!(packing.general_routing_ffs(), &[ff]);
        assert!(packing.constraints().pin_bindings().is_empty());
        assert_eq!(
            packing.constraints().pin_name_bindings().get(&data_pin),
            Some(&"M".to_owned())
        );
        assert_eq!(architecture.device().bel_pins()[physical_pin.0].name, "M");
    }

    #[test]
    fn fixes_an_io_cell_to_its_package_pin_bel() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let io = design.add_cell("input", ResourceKind::Io);
        design.add_pin(io, "O", PinDirection::Output).unwrap();
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();
        packing
            .bind_package_pins(
                &design,
                &architecture,
                "CABGA381",
                [PackagePinBinding {
                    cell: io,
                    pin: "A10".into(),
                }],
            )
            .unwrap();

        let placement =
            place_with_constraints(&design, architecture.device(), packing.constraints()).unwrap();

        assert_eq!(
            placement.bel(io),
            architecture.packages()[0].pins.get("A10").copied()
        );
    }

    #[test]
    fn package_binding_failure_is_transactional() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let first = design.add_cell("first", ResourceKind::Io);
        design.add_pin(first, "O", PinDirection::Output).unwrap();
        let second = design.add_cell("second", ResourceKind::Io);
        design.add_pin(second, "O", PinDirection::Output).unwrap();
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();

        assert!(
            packing
                .bind_package_pins(
                    &design,
                    &architecture,
                    "CABGA381",
                    [
                        PackagePinBinding {
                            cell: first,
                            pin: "A10".into(),
                        },
                        PackagePinBinding {
                            cell: second,
                            pin: "missing".into(),
                        },
                    ],
                )
                .is_err()
        );
        assert!(packing.constraints().groups().is_empty());
    }

    #[test]
    fn applies_resolved_lpf_location_and_iobuf_attributes() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let input = design.add_cell("input", ResourceKind::Io);
        design.add_pin(input, "O", PinDirection::Output).unwrap();
        let parsed = parse_lpf(
            br#"
                LOCATE COMP "input" SITE "A10";
                IOBUF PORT "input" IO_TYPE=LVCMOS33 PULLMODE=UP;
                FREQUENCY PORT "input" 25 MHZ;
            "#
            .as_slice(),
        )
        .unwrap();
        let resolved = resolve_lpf_ports(
            &parsed,
            &[LogicalPort {
                name: "input".into(),
                bits: vec![input],
            }],
            false,
        )
        .unwrap();
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();

        packing
            .apply_resolved_lpf(&design, &architecture, "CABGA381", &resolved)
            .unwrap();
        let placement =
            place_with_constraints(&design, architecture.device(), packing.constraints()).unwrap();

        assert_eq!(
            placement.bel(input),
            architecture.packages()[0].pins.get("A10").copied()
        );
        assert_eq!(packing.io_attributes()[&input]["IO_TYPE"], "LVCMOS33");
        assert_eq!(packing.io_attributes()[&input]["PULLMODE"], "UP");
        assert_eq!(packing.clock_frequencies_hz()[&input], 25_000_000);
    }

    #[test]
    fn packs_dp16kd_memories_in_stable_cell_order() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let first = add_block_ram(&mut design, "first");
        let second = add_block_ram(&mut design, "second");
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();

        packing
            .pack_block_rams(
                &design,
                &architecture,
                [
                    BlockRamRequirement {
                        cell: second,
                        depth: 1_024,
                        word_width: 18,
                        physical_width: 18,
                    },
                    BlockRamRequirement {
                        cell: first,
                        depth: 8_192,
                        word_width: 2,
                        physical_width: 2,
                    },
                ],
            )
            .unwrap();
        let placement =
            place_with_constraints(&design, architecture.device(), packing.constraints()).unwrap();

        assert_eq!(
            packing.block_rams(),
            [
                PackedBlockRam {
                    cell: first,
                    wid: 3,
                    depth: 8_192,
                    word_width: 2,
                    physical_width: 2,
                },
                PackedBlockRam {
                    cell: second,
                    wid: 4,
                    depth: 1_024,
                    word_width: 18,
                    physical_width: 18,
                },
            ]
        );
        for memory in [first, second] {
            let bel = placement.bel(memory).unwrap();
            assert_eq!(architecture.bel_metadata(bel).bel_type, "DP16KD");
        }
    }

    #[test]
    fn rejects_illegal_dp16kd_shape_without_mutating_constraints() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let memory = add_block_ram(&mut design, "words");
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();

        assert_eq!(
            packing.pack_block_rams(
                &design,
                &architecture,
                [BlockRamRequirement {
                    cell: memory,
                    depth: 2_049,
                    word_width: 9,
                    physical_width: 9,
                }]
            ),
            Err(PackingError::InvalidBlockRamDepth {
                cell: "words".into(),
                depth: 2_049,
                max_depth: 2_048,
            })
        );
        assert!(packing.constraints().groups().is_empty());
        assert!(packing.block_rams().is_empty());

        packing
            .pack_block_rams(
                &design,
                &architecture,
                [BlockRamRequirement {
                    cell: memory,
                    depth: 2_048,
                    word_width: 9,
                    physical_width: 9,
                }],
            )
            .unwrap();
        assert_eq!(packing.constraints().groups().len(), 1);
    }

    #[test]
    fn requires_structural_metadata_for_every_memory() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        add_block_ram(&mut design, "words");
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();

        assert_eq!(
            packing.pack_block_rams(&design, &architecture, []),
            Err(PackingError::MissingBlockRamRequirement {
                cell: "words".into(),
            })
        );
    }

    #[test]
    fn consumes_struo_block_ram_metadata_without_a_git_dependency() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut source = Netlist::new("memory");
        let clock = source.add_input("clock");
        let write_enable = source.add_input("write_enable");
        let read_address = (0..2)
            .map(|index| source.add_input(format!("read_address_{index}")))
            .collect();
        let write_address = (0..2)
            .map(|index| source.add_input(format!("write_address_{index}")))
            .collect();
        let write_data = (0..2)
            .map(|index| source.add_input(format!("write_data_{index}")))
            .collect();
        let read_data: Vec<_> = (0..2)
            .map(|index| source.add_memory_output(format!("read_data_{index}")))
            .collect();
        source.add_memory(MemoryCell::new(
            "words",
            4,
            read_address,
            read_data.clone(),
            None,
            write_address,
            write_data,
            EnableControl {
                signal: write_enable,
                active: StruoActiveLevel::High,
            },
            clock,
            StruoClockEdge::Rising,
        ));
        for (index, output) in read_data.into_iter().enumerate() {
            source.add_output(format!("read_data_{index}"), output);
        }
        let imported = import_ecp5(&map_to_ecp5(&source).unwrap()).unwrap();
        let requirements = imported
            .metadata()
            .iter()
            .filter_map(|(&cell, metadata)| match metadata {
                PrimitiveMetadata::BlockRam {
                    depth,
                    word_width,
                    physical_width,
                    ..
                } => Some(BlockRamRequirement {
                    cell,
                    depth: *depth,
                    word_width: *word_width,
                    physical_width: *physical_width,
                }),
                _ => None,
            })
            .collect::<Vec<_>>();
        let mut packing = pack_lut_ffs(imported.design(), &architecture).unwrap();

        packing
            .pack_block_rams(imported.design(), &architecture, requirements)
            .unwrap();

        assert_eq!(packing.block_rams().len(), 1);
        assert_eq!(packing.block_rams()[0].depth, 4);
        assert_eq!(packing.block_rams()[0].word_width, 2);
        assert_eq!(packing.block_rams()[0].physical_width, 2);
        assert_eq!(packing.constraints().groups().len(), 1);
    }

    #[test]
    fn inserts_places_and_routes_a_dcca_global_clock() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let input = design.add_cell("clock", ResourceKind::Io);
        let driver = design.add_pin(input, "O", PinDirection::Output).unwrap();
        let ff = add_ff(&mut design, "state");
        let clock = design.cells()[ff.0]
            .pins()
            .iter()
            .copied()
            .find(|pin| design.pins()[pin.0].name == "CLK")
            .unwrap();
        let source_net = design.add_net("clock", driver, [clock]).unwrap();
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();
        let local_ff = architecture
            .device()
            .bels()
            .iter()
            .position(|bel| bel.kind == ResourceKind::Register && bel.point.x == 0)
            .map(BelId)
            .unwrap();
        packing.constraints.add_group([ff], [vec![local_ff]]);
        packing
            .bind_package_pins(
                &design,
                &architecture,
                "CABGA381",
                [PackagePinBinding {
                    cell: input,
                    pin: "A10".into(),
                }],
            )
            .unwrap();

        let requirements = find_global_clock_requirements(&design, 1);
        packing
            .promote_global_clocks(&mut design, &architecture, requirements)
            .unwrap();
        let result =
            place_and_route_with_constraints(&design, architecture.device(), packing.constraints())
                .unwrap();

        assert_eq!(packing.global_clocks().len(), 1);
        let promoted = packing.global_clocks()[0];
        assert_eq!(promoted.source_net, source_net);
        assert_eq!(design.pins()[clock.0].net(), Some(promoted.global_net));
        assert_eq!(design.nets()[source_net.0].sinks.len(), 1);
        assert_eq!(
            architecture
                .bel_metadata(result.placement.bel(promoted.buffer).unwrap())
                .bel_type,
            "DCCA"
        );
        assert_eq!(result.routes.len(), 2);
        assert_eq!(result.total_pips, 2);
    }

    #[test]
    fn global_clock_failure_does_not_mutate_the_design_or_constraints() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let source = design.add_cell("source", ResourceKind::Logic);
        let driver = design.add_pin(source, "out", PinDirection::Output).unwrap();
        let sink = design.add_cell("sink", ResourceKind::Logic);
        let input = design.add_pin(sink, "in", PinDirection::Input).unwrap();
        let net = design.add_net("data", driver, [input]).unwrap();
        let original = design.clone();
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();
        let original_constraints = packing.constraints().clone();

        assert_eq!(
            packing.promote_global_clocks(
                &mut design,
                &architecture,
                [GlobalClockRequirement { net }]
            ),
            Err(PackingError::GlobalClockHasNoClockSinks { net: "data".into() })
        );
        assert_eq!(design, original);
        assert_eq!(packing.constraints(), &original_constraints);
        assert!(packing.global_clocks().is_empty());
    }

    #[test]
    fn rejects_more_global_clocks_than_the_architecture_can_place() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        for index in 0..2 {
            let source = design.add_cell(format!("source_{index}"), ResourceKind::Logic);
            let driver = design.add_pin(source, "out", PinDirection::Output).unwrap();
            let ff = add_ff(&mut design, &format!("state_{index}"));
            let clock = design.cells()[ff.0]
                .pins()
                .iter()
                .copied()
                .find(|pin| design.pins()[pin.0].name == "CLK")
                .unwrap();
            design
                .add_net(format!("clock_{index}"), driver, [clock])
                .unwrap();
        }
        let original = design.clone();
        let requirements = find_global_clock_requirements(&design, 1);
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();

        assert_eq!(
            packing.promote_global_clocks(&mut design, &architecture, requirements),
            Err(PackingError::InsufficientGlobalClockBels {
                required: 2,
                available: 1,
            })
        );
        assert_eq!(design, original);
        assert!(packing.global_clocks().is_empty());
    }

    #[test]
    fn promotes_a_clock_from_a_direct_struo_import() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut source = Netlist::new("state");
        let data = source.add_input("data");
        let clock = source.add_input("clock");
        let output = source.add_register_output("state");
        source.add_register(RegisterCell::new(
            "state",
            output,
            data,
            clock,
            StruoClockEdge::Rising,
            None,
            None,
        ));
        source.add_output("state", output);
        let imported = import_ecp5(&map_to_ecp5(&source).unwrap()).unwrap();
        let mut design = imported.into_design();
        let requirements = find_global_clock_requirements(&design, 1);
        let clock_net_name = design.nets()[requirements[0].net.0].name.clone();
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();

        assert_eq!(requirements.len(), 1);
        packing
            .promote_global_clocks(&mut design, &architecture, requirements)
            .unwrap();

        assert_eq!(packing.global_clocks().len(), 1);
        let buffer = packing.global_clocks()[0].buffer;
        assert_eq!(design.cells()[buffer.0].kind, ResourceKind::Clock);
        assert_eq!(
            design.cells()[buffer.0].name,
            format!("$gbuf${clock_net_name}")
        );
    }

    fn add_ff(design: &mut Design, name: &str) -> CellId {
        let ff = design.add_cell(name, ResourceKind::Register);
        for pin in ["DI", "CLK", "LSR", "CE"] {
            design.add_pin(ff, pin, PinDirection::Input).unwrap();
        }
        design.add_pin(ff, "Q", PinDirection::Output).unwrap();
        ff
    }

    fn add_block_ram(design: &mut Design, name: &str) -> CellId {
        let memory = design.add_cell(name, ResourceKind::Memory);
        design.add_pin(memory, "CLKA", PinDirection::Input).unwrap();
        memory
    }
}
