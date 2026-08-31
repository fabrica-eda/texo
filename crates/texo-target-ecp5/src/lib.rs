//! Versioned Project Trellis architecture import for ECP5.
//!
//! Project Trellis exposes its routing graph through C++/Python. The companion
//! `tools/export_ecp5.py` script snapshots that graph into the schema defined
//! here. Runtime placement and routing then use only Rust and [`texo_model`].

mod lpf;
mod placement_delay;

pub use lpf::{
    LogicalPort, LpfConstraints, LpfError, ResolvedLpf, parse_lpf, resolve_lpf_port_cells,
    resolve_lpf_ports,
};
pub use placement_delay::{Ecp5DelayPredictorError, Ecp5PlacementDelayPredictor};

use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::error::Error;
use std::fmt;
use std::io::{Read, Write};
use std::sync::Arc;

use serde::de::Error as _;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use texo_model::{
    BelId, BelPinId, BufferSpec, CellId, CellPinId, Design, Device, ModelError, NetId,
    PinDirection, PipId, Point, ResourceKind, UnifiedGraph, WireId,
};
use texo_pnr::{NetRoute, Placement, PlacementConstraints, RoutingConstraints};

/// Current on-disk architecture schema version.
pub const SCHEMA_VERSION: u32 = 7;

/// Version of the expanded binary architecture cache.
pub const ARCHITECTURE_CACHE_VERSION: u32 = 5;

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

/// One physical Project Trellis configuration tile at a grid location.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct TileRecord {
    /// Exact config-file tile name.
    pub name: String,
    /// Tile database type used by routing arcs and feature settings.
    pub tile_type: String,
}

/// One physical grid location and its deduplicated type.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct LocationRecord {
    /// Horizontal device coordinate.
    pub x: u32,
    /// Vertical device coordinate.
    pub y: u32,
    /// Index into [`ArchitectureFile::location_types`].
    pub location_type: usize,
    /// Configuration tiles physically present at this location.
    #[serde(default)]
    pub tiles: Vec<TileRecord>,
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
    /// Exact Project Trellis tile receiving the configurable routing arc.
    pub config_tile: Option<&'a str>,
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

/// Eight-byte runtime PIP metadata. Custom serde retains the cache-v5 field
/// widths and ordering so released architecture databases remain readable.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct CompactPipMetadata {
    tile_type_and_fixed: u16,
    config_tile: u16,
    timing_class: u16,
    lutperm_flags: u16,
}

const FIXED_PIP_METADATA_BIT: u16 = 1 << 15;
const NO_CONFIG_TILE_METADATA: u16 = u16::MAX;

#[derive(Deserialize, Serialize)]
struct SerializedCompactPipMetadata {
    tile_type: u32,
    config_tile: Option<u32>,
    timing_class: u32,
    lutperm_flags: u16,
    fixed: bool,
}

impl CompactPipMetadata {
    fn new(
        tile_type: u32,
        config_tile: Option<u32>,
        timing_class: u32,
        lutperm_flags: u16,
        fixed: bool,
    ) -> Result<Self, ImportError> {
        let tile_type =
            u16::try_from(tile_type).map_err(|_| ImportError::TooManyMetadataStrings)?;
        if tile_type >= FIXED_PIP_METADATA_BIT {
            return Err(ImportError::TooManyMetadataStrings);
        }
        let config_tile = config_tile
            .map(|id| {
                u16::try_from(id)
                    .ok()
                    .filter(|&id| id != NO_CONFIG_TILE_METADATA)
                    .ok_or(ImportError::TooManyMetadataStrings)
            })
            .transpose()?
            .unwrap_or(NO_CONFIG_TILE_METADATA);
        let timing_class = u16::try_from(timing_class)
            .ok()
            .filter(|&id| id != NO_CONFIG_TILE_METADATA)
            .ok_or(ImportError::TooManyMetadataStrings)?;
        Ok(Self {
            tile_type_and_fixed: tile_type | if fixed { FIXED_PIP_METADATA_BIT } else { 0 },
            config_tile,
            timing_class,
            lutperm_flags,
        })
    }

    const fn tile_type(self) -> u32 {
        (self.tile_type_and_fixed & !FIXED_PIP_METADATA_BIT) as u32
    }

    const fn config_tile(self) -> Option<u32> {
        if self.config_tile == NO_CONFIG_TILE_METADATA {
            None
        } else {
            Some(self.config_tile as u32)
        }
    }

    const fn timing_class(self) -> u32 {
        self.timing_class as u32
    }

    const fn fixed(self) -> bool {
        self.tile_type_and_fixed & FIXED_PIP_METADATA_BIT != 0
    }
}

impl Serialize for CompactPipMetadata {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        SerializedCompactPipMetadata {
            tile_type: self.tile_type(),
            config_tile: self.config_tile(),
            timing_class: self.timing_class(),
            lutperm_flags: self.lutperm_flags,
            fixed: self.fixed(),
        }
        .serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for CompactPipMetadata {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let metadata = SerializedCompactPipMetadata::deserialize(deserializer)?;
        Self::new(
            metadata.tile_type,
            metadata.config_tile,
            metadata.timing_class,
            metadata.lutperm_flags,
            metadata.fixed,
        )
        .map_err(|_| D::Error::custom("ECP5 PIP metadata string ID exceeds compact storage"))
    }
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
    configuration_tiles: Vec<Vec<(u32, u32)>>,
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

#[derive(Clone, Copy, Debug)]
struct BlockedGlobalResources<'a> {
    wires: &'a BTreeSet<WireId>,
    pips: &'a BTreeSet<PipId>,
}

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
            fixed: metadata.fixed(),
            tile_type: self.metadata_string(metadata.tile_type()),
            config_tile: metadata.config_tile().map(|id| self.metadata_string(id)),
            timing_class: self.metadata_string(metadata.timing_class()),
            lutperm_flags: metadata.lutperm_flags,
        }
    }

    /// Configuration tile names and types at one device coordinate.
    pub fn configuration_tiles(&self, point: Point) -> impl Iterator<Item = (&str, &str)> {
        let index = (point.y * self.device.width() + point.x) as usize;
        self.configuration_tiles
            .get(index)
            .into_iter()
            .flatten()
            .map(|&(name, tile_type)| (self.metadata_string(name), self.metadata_string(tile_type)))
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
            .map(|metadata| metadata.timing_class())
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

/// Placement-relevant ECP5 flip-flop control set.
///
/// Values are opaque stable identifiers supplied by the mapped-netlist
/// adapter. `slice_ce` is shared by the two FFs in one slice; `tile_clock`
/// and `tile_lsr` are shared by all eight FFs in one logic tile. Clock edge,
/// reset assertion polarity, and synchronous/asynchronous mode must be folded
/// into the corresponding value by the adapter.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct FfControlSet {
    /// Logical register cell.
    pub cell: CellId,
    /// CE net, constant state, and assertion polarity.
    pub slice_ce: u64,
    /// Clock net and active edge.
    pub tile_clock: u64,
    /// LSR net, assertion polarity, and sync/async mode.
    pub tile_lsr: u64,
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

/// Output-divider fallback used for emitted `EHXPLLL` configuration.
///
/// The nextpnr ECP5 bitstream writer and Texo's native bitgen emit divide-by-eight
/// for omitted `CLKOP_DIV`, `CLKOS_DIV`, `CLKOS2_DIV`, and `CLKOS3_DIV` parameters.
/// This differs from nextpnr 0.6's packing-time timing fallback of one (and the
/// zero reset encoding in Trellis `bits.db`), so STA must follow the configuration
/// actually emitted into the bitstream.
pub const ECP5_PLL_OUTPUT_DIVIDER_DEFAULT: u64 = 8;

/// Target packing decisions consumed by grouped placement and configuration.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Ecp5Packing {
    constraints: PlacementConstraints,
    wide_lut_clusters: Option<Vec<Vec<CellId>>>,
    carry_pairs: Vec<[CellId; 2]>,
    carry_state: CarryPackingState,
    lut_ff_pairs: Vec<LutFfPair>,
    general_routing_ffs: Vec<CellId>,
    block_rams: Vec<PackedBlockRam>,
    block_rams_packed: bool,
    global_clocks: Vec<PackedGlobalClock>,
    global_clocks_packed: bool,
    io_attributes: BTreeMap<CellId, BTreeMap<String, String>>,
    clock_frequencies_hz: BTreeMap<CellId, u64>,
    generated_clock_periods_ps: BTreeMap<NetId, u64>,
    unsupported_lpf_commands: Vec<String>,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
enum CarryPackingState {
    #[default]
    Unpacked,
    Pairs,
    PairsAndFfs,
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

    /// LUT4 cells packed into dedicated ECP5 LUT5/LUT6/LUT7 cascades.
    #[must_use]
    pub fn wide_lut_clusters(&self) -> &[Vec<CellId>] {
        self.wide_lut_clusters.as_deref().unwrap_or_default()
    }

    /// Constrains two-, four-, and eight-LUT clusters to the
    /// `PFUMX`/`L6MUX21` dedicated topology used by nextpnr-ecp5.
    ///
    /// # Errors
    ///
    /// Returns an error for repeated, unknown, overlapping, or structurally
    /// incompatible cells, an unavailable physical cluster, or a second call.
    pub fn pack_wide_luts(
        &mut self,
        design: &Design,
        architecture: &Ecp5Architecture,
        clusters: impl IntoIterator<Item = Vec<CellId>>,
    ) -> Result<(), PackingError> {
        if self.wide_lut_clusters.is_some() {
            return Err(PackingError::WideLutsAlreadyPacked);
        }
        let mut occupied = BTreeSet::new();
        let mut packed = Vec::new();
        let mut assignments_by_size = BTreeMap::<usize, Arc<[Vec<BelId>]>>::new();
        for cluster in clusters {
            if !matches!(cluster.len(), 2 | 4 | 8) {
                return Err(PackingError::InvalidWideLutClusterSize(cluster.len()));
            }
            for &cell in &cluster {
                let Some(logical) = design.cells().get(cell.0) else {
                    return Err(PackingError::UnknownWideLutCell(cell));
                };
                if logical.kind != ResourceKind::Lut(4) {
                    return Err(PackingError::CellIsNotWideLut {
                        cell: logical.name.clone(),
                    });
                }
                if !occupied.insert(cell) {
                    return Err(PackingError::DuplicateWideLutCell {
                        cell: logical.name.clone(),
                    });
                }
            }
            if !valid_wide_lut_cluster(design, &cluster) {
                return Err(PackingError::InvalidWideLutStructure {
                    cell: design.cells()[cluster[0].0].name.clone(),
                });
            }
            let assignments = assignments_by_size
                .entry(cluster.len())
                .or_insert_with(|| Arc::from(wide_lut_assignments(architecture, cluster.len())));
            if assignments.is_empty() {
                return Err(PackingError::MissingWideLutCluster {
                    cell: design.cells()[cluster[0].0].name.clone(),
                    size: cluster.len(),
                });
            }
            self.constraints.add_group_with_shared_assignments(
                cluster.iter().copied(),
                Arc::clone(assignments),
            );
            packed.push(cluster);
        }
        self.wide_lut_clusters = Some(packed);
        Ok(())
    }

    /// Constrains both FFs in each ECP5 slice to one compatible CE control set.
    ///
    /// Values are opaque identifiers chosen by the mapped-netlist adapter. Two
    /// registers may share a slice exactly when their identifiers are equal.
    ///
    /// # Panics
    ///
    /// Panics only if the architecture contains more ECP5 slices than fit in
    /// a `u64` resource identifier.
    pub fn constrain_ff_slice_ce_muxes(
        &mut self,
        architecture: &Ecp5Architecture,
        cell_values: impl IntoIterator<Item = (CellId, u64)>,
    ) {
        let cell_values = cell_values.into_iter().collect::<Vec<_>>();
        if cell_values.is_empty() {
            return;
        }
        let mut resource_ids = BTreeMap::new();
        let mut bel_resources = Vec::new();
        for &bel in architecture.device().bels_of_kind(ResourceKind::Register) {
            let metadata = architecture.bel_metadata(bel);
            if metadata.bel_type != "TRELLIS_FF" {
                continue;
            }
            let point = architecture.device().bels()[bel.0].point;
            let key = (point, metadata.z >> 3);
            let next = u64::try_from(resource_ids.len()).expect("ECP5 slice count fits u64");
            let resource = *resource_ids.entry(key).or_insert(next);
            bel_resources.push((bel, resource));
        }
        self.constraints
            .add_shared_resource(cell_values, bel_resources);
    }

    /// Constrains all eight FFs in one ECP5 logic tile to one compatible clock
    /// control set.
    ///
    /// Values identify the logical clock net and active edge. The split-BEL
    /// graph exposes a separate local clock wire at each slice,
    /// but they select one tile-wide clock net and polarity in configuration.
    ///
    /// # Panics
    ///
    /// Panics only if the architecture contains more shared ECP5 clock wires
    /// than fit in a `u64` resource identifier.
    pub fn constrain_ff_clock_muxes(
        &mut self,
        architecture: &Ecp5Architecture,
        cell_values: impl IntoIterator<Item = (CellId, u64)>,
    ) {
        let cell_values = cell_values.into_iter().collect::<Vec<_>>();
        if cell_values.is_empty() {
            return;
        }
        let mut resource_ids = BTreeMap::new();
        let mut bel_resources = Vec::new();
        for &bel in architecture.device().bels_of_kind(ResourceKind::Register) {
            let metadata = architecture.bel_metadata(bel);
            if metadata.bel_type != "TRELLIS_FF" {
                continue;
            }
            let point = architecture.device().bels()[bel.0].point;
            let next = u64::try_from(resource_ids.len()).expect("ECP5 logic tile count fits u64");
            let resource = *resource_ids.entry(point).or_insert(next);
            bel_resources.push((bel, resource));
        }
        self.constraints
            .add_shared_resource(cell_values, bel_resources);
    }

    /// Constrains all eight FFs in one ECP5 logic tile to one reset control
    /// set.
    ///
    /// Values must identify the LSR net together with assertion polarity and
    /// synchronous/asynchronous mode. A register with no reset therefore has
    /// a different value from a register using a routed reset: otherwise the
    /// shared tile LSR would reset both registers.
    ///
    /// # Panics
    ///
    /// Panics only if the architecture contains more shared ECP5 LSR wires
    /// than fit in a `u64` resource identifier.
    pub fn constrain_ff_lsr_muxes(
        &mut self,
        architecture: &Ecp5Architecture,
        cell_values: impl IntoIterator<Item = (CellId, u64)>,
    ) {
        let cell_values = cell_values.into_iter().collect::<Vec<_>>();
        if cell_values.is_empty() {
            return;
        }
        let mut resource_ids = BTreeMap::new();
        let mut bel_resources = Vec::new();
        for &bel in architecture.device().bels_of_kind(ResourceKind::Register) {
            let metadata = architecture.bel_metadata(bel);
            if metadata.bel_type != "TRELLIS_FF" {
                continue;
            }
            let point = architecture.device().bels()[bel.0].point;
            let next = u64::try_from(resource_ids.len()).expect("ECP5 logic tile count fits u64");
            let resource = *resource_ids.entry(point).or_insert(next);
            bel_resources.push((bel, resource));
        }
        self.constraints
            .add_shared_resource(cell_values, bel_resources);
    }

    /// Installs all placement-relevant ECP5 FF shared-resource constraints.
    pub fn constrain_ff_control_sets(
        &mut self,
        architecture: &Ecp5Architecture,
        control_sets: &[FfControlSet],
    ) {
        self.constrain_ff_slice_ce_muxes(
            architecture,
            control_sets.iter().map(|set| (set.cell, set.slice_ce)),
        );
        self.constrain_ff_clock_muxes(
            architecture,
            control_sets.iter().map(|set| (set.cell, set.tile_clock)),
        );
        self.constrain_ff_lsr_muxes(
            architecture,
            control_sets.iter().map(|set| (set.cell, set.tile_lsr)),
        );
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
        if !self.constraints.remove_group(&[lut, ff]) && !self.constraints.remove_group_cell(ff) {
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
        if self.carry_state != CarryPackingState::Unpacked {
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
        self.carry_state = CarryPackingState::Pairs;
        Ok(())
    }

    /// Packs the maximum deterministic set of carry-result FFs onto the
    /// dedicated local `F -> DI` paths of an already constructed carry macro.
    ///
    /// Carry LUTs remain one rigid atomic group. Selected FFs are appended as
    /// columns whose BEL is exactly `TRELLIS_FF(z + 1)` for the corresponding
    /// carry LUT in every legal assignment row. Selection is exact within each
    /// tile: all selected FFs share one tile clock/LSR control set, each slice
    /// shares one CE control set, and at most one FF is selected for each LUT
    /// even when its `F` net has duplicate fanout.
    ///
    /// # Errors
    ///
    /// Returns an error when carry pairs have not been packed, this operation
    /// was already performed, a direct candidate lacks a control set, or the
    /// architecture/group surface cannot represent the dedicated FF columns.
    pub fn pack_carry_lut_ffs(
        &mut self,
        design: &Design,
        architecture: &Ecp5Architecture,
        control_sets: impl IntoIterator<Item = FfControlSet>,
    ) -> Result<(), PackingError> {
        self.pack_carry_lut_ffs_impl(design, architecture, control_sets, None)
    }

    /// Packs an explicitly selected set of carry-result FFs.
    ///
    /// This restores packing from an external checkpoint. Requested pairs
    /// must obey the same tile-wide clock/LSR and slice-wide CE rules as the
    /// automatic selector.
    ///
    /// # Errors
    ///
    /// Returns an error for the same structural failures as
    /// [`Self::pack_carry_lut_ffs`], or for a requested pair that is not a
    /// direct carry `F -> DI` edge or violates a shared control set.
    pub fn pack_carry_lut_ffs_with_pairs(
        &mut self,
        design: &Design,
        architecture: &Ecp5Architecture,
        control_sets: impl IntoIterator<Item = FfControlSet>,
        pairs: impl IntoIterator<Item = LutFfPair>,
    ) -> Result<(), PackingError> {
        let pairs = pairs.into_iter().collect::<Vec<_>>();
        self.pack_carry_lut_ffs_impl(design, architecture, control_sets, Some(&pairs))
    }

    fn pack_carry_lut_ffs_impl(
        &mut self,
        design: &Design,
        architecture: &Ecp5Architecture,
        control_sets: impl IntoIterator<Item = FfControlSet>,
        requested_pairs: Option<&[LutFfPair]>,
    ) -> Result<(), PackingError> {
        if self.carry_state == CarryPackingState::Unpacked {
            return Err(PackingError::CarryPairsNotPacked);
        }
        if self.carry_state == CarryPackingState::PairsAndFfs {
            return Err(PackingError::CarryLutFfsAlreadyPacked);
        }
        let control_sets = collect_ff_control_sets(design, control_sets)?;
        let chains = logical_carry_chains(design, &self.carry_pairs)?;
        let locations = carry_lut_locations(&self.carry_pairs, &chains);
        let selected = if let Some(requested_pairs) = requested_pairs {
            validate_requested_carry_lut_ff_pairs(
                design,
                &self.general_routing_ffs,
                &locations,
                &control_sets,
                requested_pairs,
            )?
        } else {
            select_carry_lut_ff_pairs(design, &self.general_routing_ffs, &locations, &control_sets)?
        };
        let ff_by_slot = physical_ff_by_slot(architecture);
        let mut constraints = self.constraints.clone();
        let mut selected_ffs = BTreeSet::new();

        for (chain_index, chain) in chains.iter().enumerate() {
            let old_cells = chain
                .iter()
                .flat_map(|&pair| self.carry_pairs[pair])
                .collect::<Vec<_>>();
            let chain_pairs = selected
                .iter()
                .filter(|selected| selected.location.chain == chain_index)
                .collect::<Vec<_>>();
            if chain_pairs.is_empty() {
                continue;
            }
            let Some(group) = constraints
                .groups()
                .iter()
                .find(|group| group.cells == old_cells)
                .cloned()
            else {
                return Err(PackingError::CarryFfPacking {
                    cell: design.cells()[old_cells[0].0].name.clone(),
                    reason: "carry placement group is missing".into(),
                });
            };
            let mut cells = old_cells.clone();
            cells.extend(chain_pairs.iter().map(|selected| selected.pair.ff));
            let assignments = carry_group_assignments_with_ffs(
                architecture,
                &group.assignments,
                &chain_pairs,
                &control_sets,
                &ff_by_slot,
            );
            if assignments.is_empty() {
                return Err(PackingError::CarryFfPacking {
                    cell: design.cells()[old_cells[0].0].name.clone(),
                    reason: "no carry assignment has every matching dedicated FF BEL".into(),
                });
            }
            if !constraints.replace_group(&old_cells, cells, assignments) {
                return Err(PackingError::CarryFfPacking {
                    cell: design.cells()[old_cells[0].0].name.clone(),
                    reason: "carry placement group could not be extended transactionally".into(),
                });
            }
            for selected in chain_pairs {
                let data_pin = ff_data_pin(design, selected.pair.ff)?;
                constraints.unbind_pin_name(data_pin);
                selected_ffs.insert(selected.pair.ff);
            }
        }

        self.constraints = constraints;
        self.lut_ff_pairs
            .extend(selected.iter().map(|selected| selected.pair));
        self.lut_ff_pairs
            .sort_unstable_by_key(|pair| (pair.lut, pair.ff));
        self.general_routing_ffs
            .retain(|ff| !selected_ffs.contains(ff));
        self.carry_state = CarryPackingState::PairsAndFfs;
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

    /// Locks promoted clock buffers to the DCCA inputs with the shortest
    /// physical paths from their already placed sources.
    ///
    /// Coarse placement geometry does not describe the dedicated connectivity
    /// between PLL outputs and the regional DCCA inputs. This performs a
    /// small, exact bipartite assignment after source placement and narrows
    /// each buffer's legal placement group to the selected DCCA.
    ///
    /// # Errors
    ///
    /// Returns an error when a source or DCCA input is unavailable, or no
    /// injective source-to-DCCA assignment is routable.
    pub fn lock_global_clock_buffers_to_shortest_sources(
        &mut self,
        design: &Design,
        architecture: &Ecp5Architecture,
        placement: &Placement,
    ) -> Result<BTreeMap<CellId, BelId>, PackingError> {
        if self.global_clocks.is_empty() {
            return Ok(BTreeMap::new());
        }
        let device = architecture.device();
        let graph = UnifiedGraph::new(design, device);
        let candidates = compatible_dcca_bels(architecture);
        let mut target_wires = Vec::with_capacity(candidates.len());
        for &bel in &candidates {
            let Some(pin) = find_bel_pin(device, bel, "CLKI") else {
                return Err(global_route_error(
                    &design.nets()[self.global_clocks[0].source_net.0],
                    format!("compatible DCCA BEL {} has no CLKI pin", bel.0),
                ));
            };
            target_wires.push(device.bel_pins()[pin.0].wire);
        }
        let mut distance_search =
            ForwardRouteTargetDistances::new(device.wires().len(), &target_wires);
        let mut costs = Vec::with_capacity(self.global_clocks.len());
        for clock in &self.global_clocks {
            let net = &design.nets()[clock.source_net.0];
            let source = placed_pin_wire(&graph, placement, net.driver)?;
            costs.push(distance_search.distances(device, source).0.to_vec());
        }
        let reachable = costs
            .iter()
            .map(|row| row.iter().flatten().count())
            .collect::<Vec<_>>();
        let assignment = minimum_injective_assignment(&costs).ok_or_else(|| {
            global_route_error(
                &design.nets()[self.global_clocks[0].source_net.0],
                format!(
                    "no injective source-to-DCCA assignment is routable (reachable candidates per source: {reachable:?})"
                ),
            )
        })?;
        let selected = self
            .global_clocks
            .iter()
            .zip(assignment)
            .map(|(clock, candidate)| (clock.buffer, (candidates[candidate], clock.source_net)))
            .collect::<BTreeMap<_, _>>();
        for (&buffer, &(bel, source_net)) in &selected {
            if !self.constraints.remove_group(&[buffer]) {
                return Err(global_route_error(
                    &design.nets()[source_net.0],
                    "promoted clock buffer has no placement group",
                ));
            }
            self.constraints.add_group([buffer], [vec![bel]]);
        }
        Ok(selected
            .into_iter()
            .map(|(buffer, (bel, _))| (buffer, bel))
            .collect())
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

    /// User-configured primitive output clock periods indexed by logical net.
    #[must_use]
    pub const fn generated_clock_periods_ps(&self) -> &BTreeMap<NetId, u64> {
        &self.generated_clock_periods_ps
    }

    /// Records a generated-clock period and returns any previous value.
    pub fn set_generated_clock_period_ps(&mut self, net: NetId, period_ps: u64) -> Option<u64> {
        self.generated_clock_periods_ps.insert(net, period_ps)
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
        let mut constraints =
            self.routing_restrictions_cached(design, architecture, placement, cache)?;
        let mut reserved_wires = BTreeSet::new();
        let mut reserved_pips = BTreeSet::new();
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
                    .reverse_route(
                        device,
                        network,
                        sink_wire,
                        &wires,
                        BlockedGlobalResources {
                            wires: &reserved_wires,
                            pips: &reserved_pips,
                        },
                        &tile_name,
                    )
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
            ensure_global_route_disjoint(
                net,
                device,
                &wires,
                &pips,
                BlockedGlobalResources {
                    wires: &reserved_wires,
                    pips: &reserved_pips,
                },
            )?;
            let route = NetRoute::from_tree(
                clock.global_net,
                source,
                sinks,
                pips.iter().copied(),
                device,
            )
            .map_err(|reason| global_route_error(net, reason))?;
            reserved_wires.extend(wires.iter().copied());
            reserved_pips.extend(pips.iter().copied());
            constraints.add_route(route);
        }
        Ok(constraints)
    }

    /// Builds placement-specific ECP5 routing legality restrictions without
    /// constructing any immutable route trees.
    ///
    /// # Errors
    ///
    /// Returns an error when a packed carry cell is missing from placement or
    /// its placed BEL does not expose the required LUT inputs.
    pub fn routing_restrictions(
        &self,
        design: &Design,
        architecture: &Ecp5Architecture,
        placement: &Placement,
    ) -> Result<RoutingConstraints, PackingError> {
        let cache = architecture.global_routing_cache();
        self.routing_restrictions_cached(design, architecture, placement, &cache)
    }

    /// Cached equivalent of [`Self::routing_restrictions`].
    ///
    /// # Errors
    ///
    /// Returns an error under the same conditions as
    /// [`Self::routing_restrictions`].
    pub fn routing_restrictions_cached(
        &self,
        design: &Design,
        architecture: &Ecp5Architecture,
        placement: &Placement,
        cache: &Ecp5GlobalRoutingCache<'_>,
    ) -> Result<RoutingConstraints, PackingError> {
        let mut constraints = RoutingConstraints::new();
        block_illegal_carry_lut_permutations(
            self,
            design,
            architecture,
            placement,
            &cache.incoming,
            &mut constraints,
        )?;
        Ok(constraints)
    }
}

fn block_illegal_carry_lut_permutations(
    packing: &Ecp5Packing,
    design: &Design,
    architecture: &Ecp5Architecture,
    placement: &Placement,
    incoming: &CompactIncomingPips,
    constraints: &mut RoutingConstraints,
) -> Result<(), PackingError> {
    let device = architecture.device();
    for &cell in packing.carry_pairs.iter().flatten() {
        let bel = placement
            .bel(cell)
            .ok_or_else(|| PackingError::CarryRouting {
                cell: design.cells()[cell.0].name.clone(),
                reason: "cell is missing from placement".into(),
            })?;
        for pin_name in ["A", "B", "C", "D"] {
            let pin =
                find_bel_pin(device, bel, pin_name).ok_or_else(|| PackingError::CarryRouting {
                    cell: design.cells()[cell.0].name.clone(),
                    reason: format!("placed BEL has no `{pin_name}` input"),
                })?;
            let wire = device.bel_pins()[pin.0].wire;
            constraints.block_pips(incoming.for_wire(wire).iter().filter_map(|&raw_pip| {
                let pip = PipId(raw_pip as usize);
                let flags = architecture.pip_metadata(pip).lutperm_flags;
                let is_lut_permutation = flags & 0x4000 != 0;
                let source_input = flags & 0x3;
                let destination_input = (flags >> 2) & 0x3;
                (is_lut_permutation && source_input / 2 != destination_input / 2).then_some(pip)
            }));
        }
    }
    Ok(())
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
        blocked: BlockedGlobalResources<'_>,
        target_name: &str,
    ) -> Option<(WireId, Vec<WireId>, Vec<PipId>)> {
        let key = (network, sink);
        if let Some((join, wires, pips)) = self.reverse_routes.get(&key)
            && (tree.contains(join) || wire_basename(&device.wires()[join.0].name) == target_name)
            && wires.iter().all(|wire| !blocked.wires.contains(wire))
            && pips.iter().all(|pip| !blocked.pips.contains(pip))
        {
            return Some((*join, wires.clone(), pips.clone()));
        }
        let route =
            self.reverse_search
                .route(device, &self.incoming, sink, tree, blocked, target_name)?;
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

fn minimum_injective_assignment(costs: &[Vec<Option<usize>>]) -> Option<Vec<usize>> {
    let candidate_count = costs.first().map_or(0, Vec::len);
    if costs.is_empty() {
        return Some(Vec::new());
    }
    if costs.len() > candidate_count || costs.iter().any(|row| row.len() != candidate_count) {
        return None;
    }
    // Rectangular Hungarian assignment. ECP5 exposes more regional DCCA BELs
    // than global clocks, so a subset bitmask would scale with the wrong side
    // of the problem (56 candidates on the 85K device).
    let rows = costs.len();
    let columns = candidate_count;
    let infinity = i64::MAX / 8;
    let mut row_potential = vec![0_i64; rows + 1];
    let mut column_potential = vec![0_i64; columns + 1];
    let mut matched_row = vec![0_usize; columns + 1];
    let mut predecessor = vec![0_usize; columns + 1];
    for row in 1..=rows {
        matched_row[0] = row;
        let mut column = 0_usize;
        let mut minimum = vec![infinity; columns + 1];
        let mut used = vec![false; columns + 1];
        loop {
            used[column] = true;
            let active_row = matched_row[column];
            let mut delta = infinity;
            let mut next_column = 0_usize;
            for candidate in 1..=columns {
                if used[candidate] {
                    continue;
                }
                let edge = costs[active_row - 1][candidate - 1]
                    .and_then(|cost| i64::try_from(cost).ok())
                    .unwrap_or(infinity);
                let reduced = edge
                    .saturating_sub(row_potential[active_row])
                    .saturating_sub(column_potential[candidate]);
                if reduced < minimum[candidate] {
                    minimum[candidate] = reduced;
                    predecessor[candidate] = column;
                }
                if minimum[candidate] < delta {
                    delta = minimum[candidate];
                    next_column = candidate;
                }
            }
            if delta == infinity || next_column == 0 {
                return None;
            }
            for candidate in 0..=columns {
                if used[candidate] {
                    row_potential[matched_row[candidate]] =
                        row_potential[matched_row[candidate]].saturating_add(delta);
                    column_potential[candidate] = column_potential[candidate].saturating_sub(delta);
                } else {
                    minimum[candidate] = minimum[candidate].saturating_sub(delta);
                }
            }
            column = next_column;
            if matched_row[column] == 0 {
                break;
            }
        }
        loop {
            let previous = predecessor[column];
            matched_row[column] = matched_row[previous];
            column = previous;
            if column == 0 {
                break;
            }
        }
    }
    let mut assignment = vec![usize::MAX; rows];
    for (column, &row) in matched_row.iter().enumerate().skip(1) {
        if row != 0 {
            assignment[row - 1] = column - 1;
        }
    }
    assignment
        .iter()
        .enumerate()
        .all(|(row, &column)| column != usize::MAX && costs[row][column].is_some())
        .then_some(assignment)
}

fn ensure_global_route_disjoint(
    net: &texo_model::Net,
    device: &Device,
    wires: &BTreeSet<WireId>,
    pips: &BTreeSet<PipId>,
    blocked: BlockedGlobalResources<'_>,
) -> Result<(), PackingError> {
    if let Some(wire) = wires.intersection(blocked.wires).next() {
        return Err(global_route_error(
            net,
            format!(
                "global clock trees overlap at wire `{}`",
                device.wires()[wire.0].name
            ),
        ));
    }
    if let Some(pip) = pips.intersection(blocked.pips).next() {
        return Err(global_route_error(
            net,
            format!("global clock trees overlap at PIP {}", pip.0),
        ));
    }
    Ok(())
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
struct ForwardRouteTargetDistances {
    epoch: u32,
    seen: Vec<u32>,
    targets_by_wire: HashMap<WireId, Vec<usize>>,
    results: Vec<Option<usize>>,
    queue: VecDeque<(WireId, usize)>,
}

impl ForwardRouteTargetDistances {
    fn new(wire_count: usize, targets: &[WireId]) -> Self {
        let mut targets_by_wire = HashMap::<WireId, Vec<usize>>::new();
        for (index, &target) in targets.iter().enumerate() {
            targets_by_wire.entry(target).or_default().push(index);
        }
        Self {
            epoch: 0,
            seen: vec![0; wire_count],
            targets_by_wire,
            results: vec![None; targets.len()],
            queue: VecDeque::new(),
        }
    }

    /// Returns target distances in the same order passed to `new`, plus the
    /// number of popped wires. Search stops as soon as every target is found.
    fn distances(&mut self, device: &Device, source: WireId) -> (&[Option<usize>], usize) {
        self.results.fill(None);
        self.queue.clear();
        if self.results.is_empty() || source.0 >= self.seen.len() {
            return (&self.results, 0);
        }
        self.epoch = self.epoch.wrapping_add(1);
        if self.epoch == 0 {
            self.seen.fill(0);
            self.epoch = 1;
        }
        self.seen[source.0] = self.epoch;
        self.queue.push_back((source, 0));
        let mut unresolved = self.results.len();
        let mut visited = 0;
        while let Some((wire, distance)) = self.queue.pop_front() {
            visited += 1;
            if let Some(targets) = self.targets_by_wire.get(&wire) {
                for &target in targets {
                    self.results[target] = Some(distance);
                }
                unresolved -= targets.len();
                if unresolved == 0 {
                    break;
                }
            }
            let Ok(neighbors) = device.routing_neighbors(wire) else {
                continue;
            };
            let next_distance = distance.saturating_add(1);
            for (next, _) in neighbors {
                if self.seen[next.0] != self.epoch {
                    self.seen[next.0] = self.epoch;
                    self.queue.push_back((next, next_distance));
                }
            }
        }
        (&self.results, visited)
    }
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
        blocked: BlockedGlobalResources<'_>,
        target_name: &str,
    ) -> Option<(WireId, Vec<WireId>, Vec<PipId>)> {
        if blocked.wires.contains(&sink) {
            return None;
        }
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
                if blocked.pips.contains(&pip) {
                    continue;
                }
                let prior = device.pips()[pip.0].from();
                if !blocked.wires.contains(&prior) && self.seen[prior.0] != epoch {
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

fn wide_lut_assignments(architecture: &Ecp5Architecture, cluster_size: usize) -> Vec<Vec<BelId>> {
    let mut comb_by_slot = BTreeMap::new();
    for &bel in architecture.device().bels_of_kind(ResourceKind::Lut(4)) {
        let metadata = architecture.bel_metadata(bel);
        if metadata.bel_type == "TRELLIS_COMB" {
            comb_by_slot.insert((architecture.device().bels()[bel.0].point, metadata.z), bel);
        }
    }
    let (alignment, required_pins): (i32, &[&[&str]]) = match cluster_size {
        2 => (
            8,
            &[
                &["A", "B", "C", "D", "F", "F1", "M", "OFX"],
                &["A", "B", "C", "D", "F"],
            ],
        ),
        4 => (
            16,
            &[
                &["A", "B", "C", "D", "F", "F1", "M", "OFX"],
                &["A", "B", "C", "D", "F", "FXA", "FXB", "M", "OFX"],
                &["A", "B", "C", "D", "F", "F1", "M", "OFX"],
                &["A", "B", "C", "D", "F"],
            ],
        ),
        8 => (
            32,
            &[
                &["A", "B", "C", "D", "F", "F1", "M", "OFX"],
                &["A", "B", "C", "D", "F", "FXA", "FXB", "M", "OFX"],
                &["A", "B", "C", "D", "F", "F1", "M", "OFX"],
                &["A", "B", "C", "D", "F", "FXA", "FXB", "M", "OFX"],
                &["A", "B", "C", "D", "F", "F1", "M", "OFX"],
                &["A", "B", "C", "D", "F", "FXA", "FXB", "M", "OFX"],
                &["A", "B", "C", "D", "F", "F1", "M", "OFX"],
                &["A", "B", "C", "D", "F"],
            ],
        ),
        _ => return Vec::new(),
    };
    let mut assignments = Vec::new();
    for &(point, z) in comb_by_slot.keys() {
        if z.rem_euclid(alignment) != 0 {
            continue;
        }
        let Some(bels) = (0..cluster_size)
            .map(|index| {
                let offset = i32::try_from(index).ok()?.checked_mul(4)?;
                comb_by_slot.get(&(point, z.checked_add(offset)?)).copied()
            })
            .collect::<Option<Vec<_>>>()
        else {
            continue;
        };
        if bels.iter().zip(required_pins).all(|(&bel, pins)| {
            pins.iter()
                .all(|pin| find_bel_pin(architecture.device(), bel, pin).is_some())
        }) {
            assignments.push(bels);
        }
    }
    assignments
}

fn valid_wide_lut_cluster(design: &Design, cluster: &[CellId]) -> bool {
    let drives = |source: CellId, output: &str, sink: CellId, input: &str| {
        let Some(sink_pin) = design.cells()[sink.0]
            .pins()
            .iter()
            .copied()
            .find(|pin| design.pins()[pin.0].name == input)
        else {
            return false;
        };
        let Some(net) = design.pins()[sink_pin.0].net() else {
            return false;
        };
        let driver = &design.pins()[design.nets()[net.0].driver.0];
        driver.cell == source && driver.name == output
    };
    match cluster {
        [root, child] => drives(*child, "F", *root, "F1"),
        [one_root, l6_root, zero_root, zero_child] => {
            drives(*l6_root, "F", *one_root, "F1")
                && drives(*zero_child, "F", *zero_root, "F1")
                && drives(*zero_root, "OFX", *l6_root, "FXA")
                && drives(*one_root, "OFX", *l6_root, "FXB")
        }
        [
            one_one_root,
            one_l6_root,
            one_zero_root,
            l7_root,
            zero_one_root,
            zero_l6_root,
            zero_zero_root,
            zero_zero_child,
        ] => {
            valid_wide_lut_cluster(
                design,
                &[*one_one_root, *one_l6_root, *one_zero_root, *l7_root],
            ) && valid_wide_lut_cluster(
                design,
                &[
                    *zero_one_root,
                    *zero_l6_root,
                    *zero_zero_root,
                    *zero_zero_child,
                ],
            ) && drives(*zero_l6_root, "OFX", *l7_root, "FXA")
                && drives(*one_l6_root, "OFX", *l7_root, "FXB")
        }
        _ => false,
    }
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

    // A carry macro has one continuous origin and one set of member offsets.
    // Starting only at the first slice of a tile makes every legal assignment
    // a translation of that same shape, including across tile boundaries.
    // This matches the ECP5 carry-macro alignment used by nextpnr.
    let mut sequences = pairs
        .iter()
        .enumerate()
        .filter(|(_, pair)| architecture.bel_metadata(pair[0]).z == 0)
        .map(|(index, _)| vec![index])
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
    validate_logical_carry_pairs(design, pairs)?;
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
            if sink_pin.cell != pairs[next_pair][0] {
                return Err(PackingError::InvalidCarryConnection {
                    cell: design.cells()[pair[1].0].name.clone(),
                    reason: format!(
                        "FCO must drive the first half of its successor carry pair, not {}",
                        design.cells()[sink_pin.cell.0].name
                    ),
                });
            }
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

fn validate_logical_carry_pairs(
    design: &Design,
    pairs: &[[CellId; 2]],
) -> Result<(), PackingError> {
    for pair in pairs {
        let first = pair[0];
        let second = pair[1];
        let first_fco = design.cells()[first.0]
            .pins()
            .iter()
            .copied()
            .find(|pin| design.pins()[pin.0].name == "FCO")
            .ok_or_else(|| PackingError::InvalidCarryConnection {
                cell: design.cells()[first.0].name.clone(),
                reason: "first carry half has no FCO pin".into(),
            })?;
        let second_fci = design.cells()[second.0]
            .pins()
            .iter()
            .copied()
            .find(|pin| design.pins()[pin.0].name == "FCI")
            .ok_or_else(|| PackingError::InvalidCarryConnection {
                cell: design.cells()[second.0].name.clone(),
                reason: "second carry half has no FCI pin".into(),
            })?;
        let Some(internal_net) = design.pins()[first_fco.0].net() else {
            return Err(PackingError::InvalidCarryConnection {
                cell: design.cells()[first.0].name.clone(),
                reason: format!(
                    "FCO is not connected exclusively to {}.FCI",
                    design.cells()[second.0].name
                ),
            });
        };
        let net = &design.nets()[internal_net.0];
        if design.pins()[second_fci.0].net() != Some(internal_net)
            || net.driver != first_fco
            || net.sinks.as_slice() != [second_fci]
        {
            return Err(PackingError::InvalidCarryConnection {
                cell: design.cells()[first.0].name.clone(),
                reason: format!(
                    "FCO must drive only the paired second half {}.FCI",
                    design.cells()[second.0].name
                ),
            });
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct CarryLutLocation {
    chain: usize,
    pair: usize,
    half: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct SelectedCarryLutFf {
    location: CarryLutLocation,
    pair: LutFfPair,
}

fn carry_lut_locations(
    pairs: &[[CellId; 2]],
    chains: &[Vec<usize>],
) -> BTreeMap<CellId, CarryLutLocation> {
    let mut locations = BTreeMap::new();
    for (chain_index, chain) in chains.iter().enumerate() {
        for (pair_index, &pair) in chain.iter().enumerate() {
            for (half, &cell) in pairs[pair].iter().enumerate() {
                locations.insert(
                    cell,
                    CarryLutLocation {
                        chain: chain_index,
                        pair: pair_index,
                        half,
                    },
                );
            }
        }
    }
    locations
}

fn collect_ff_control_sets(
    design: &Design,
    control_sets: impl IntoIterator<Item = FfControlSet>,
) -> Result<BTreeMap<CellId, FfControlSet>, PackingError> {
    let mut collected = BTreeMap::new();
    for control_set in control_sets {
        let Some(cell) = design.cells().get(control_set.cell.0) else {
            return Err(PackingError::CarryFfPacking {
                cell: format!("cell#{}", control_set.cell.0),
                reason: "control set names an unknown cell".into(),
            });
        };
        if cell.kind != ResourceKind::Register {
            return Err(PackingError::CarryFfPacking {
                cell: cell.name.clone(),
                reason: "control set member is not a register".into(),
            });
        }
        if collected.insert(control_set.cell, control_set).is_some() {
            return Err(PackingError::CarryFfPacking {
                cell: cell.name.clone(),
                reason: "register has more than one control-set record".into(),
            });
        }
    }
    Ok(collected)
}

fn physical_ff_by_slot(architecture: &Ecp5Architecture) -> BTreeMap<(Point, i32), BelId> {
    architecture
        .device()
        .bels_of_kind(ResourceKind::Register)
        .iter()
        .copied()
        .filter_map(|bel| {
            let metadata = architecture.bel_metadata(bel);
            (metadata.bel_type == "TRELLIS_FF")
                .then_some(((architecture.device().bels()[bel.0].point, metadata.z), bel))
        })
        .collect()
}

fn carry_group_assignments_with_ffs(
    architecture: &Ecp5Architecture,
    rows: &[Vec<BelId>],
    selected: &[&SelectedCarryLutFf],
    control_sets: &BTreeMap<CellId, FfControlSet>,
    ff_by_slot: &BTreeMap<(Point, i32), BelId>,
) -> Vec<Vec<BelId>> {
    rows.iter()
        .filter_map(|row| {
            let mut extended = row.clone();
            let mut tile_controls = BTreeMap::new();
            let mut slice_controls = BTreeMap::new();
            let mut used_ff_bels = BTreeSet::new();
            for selected in selected {
                let lut_column = selected.location.pair * 2 + selected.location.half;
                let &lut_bel = row.get(lut_column)?;
                let metadata = architecture.bel_metadata(lut_bel);
                let point = architecture.device().bels()[lut_bel.0].point;
                let ff_z = metadata.z.checked_add(1)?;
                let &ff_bel = ff_by_slot.get(&(point, ff_z))?;
                let set = control_sets[&selected.pair.ff];
                let tile_control = (set.tile_clock, set.tile_lsr);
                if tile_controls
                    .insert(point, tile_control)
                    .is_some_and(|old| old != tile_control)
                    || slice_controls
                        .insert((point, metadata.z >> 3), set.slice_ce)
                        .is_some_and(|old| old != set.slice_ce)
                    || !used_ff_bels.insert(ff_bel)
                {
                    return None;
                }
                extended.push(ff_bel);
            }
            Some(extended)
        })
        .collect()
}

fn ff_data_pin(design: &Design, ff: CellId) -> Result<CellPinId, PackingError> {
    design.cells()[ff.0]
        .pins()
        .iter()
        .copied()
        .find(|pin| design.pins()[pin.0].name == "DI")
        .ok_or_else(|| PackingError::MissingFfDataPin {
            cell: design.cells()[ff.0].name.clone(),
        })
}

fn select_carry_lut_ff_pairs(
    design: &Design,
    general_routing_ffs: &[CellId],
    locations: &BTreeMap<CellId, CarryLutLocation>,
    control_sets: &BTreeMap<CellId, FfControlSet>,
) -> Result<Vec<SelectedCarryLutFf>, PackingError> {
    let mut candidates = BTreeMap::<CarryLutLocation, Vec<(LutFfPair, FfControlSet)>>::new();
    for &ff in general_routing_ffs {
        let data_pin = ff_data_pin(design, ff)?;
        let Some(lut) = lut_driver(design, data_pin) else {
            continue;
        };
        let Some(&location) = locations.get(&lut) else {
            continue;
        };
        let Some(&control_set) = control_sets.get(&ff) else {
            return Err(PackingError::CarryFfPacking {
                cell: design.cells()[ff.0].name.clone(),
                reason: "direct carry-result FF has no control-set record".into(),
            });
        };
        candidates
            .entry(location)
            .or_default()
            .push((LutFfPair { lut, ff }, control_set));
    }

    let tiles = candidates
        .keys()
        .map(|location| (location.chain, location.pair / 4))
        .collect::<BTreeSet<_>>();
    let mut selected = Vec::new();
    for (chain, tile) in tiles {
        let tile_classes = candidates
            .iter()
            .filter(|(location, _)| location.chain == chain && location.pair / 4 == tile)
            .flat_map(|(_, candidates)| {
                candidates
                    .iter()
                    .map(|(_, set)| (set.tile_clock, set.tile_lsr))
            })
            .collect::<BTreeSet<_>>();
        let mut best_tile = Vec::new();
        for tile_class in tile_classes {
            let mut tile_selection = Vec::new();
            for slice in 0..4 {
                let ce_classes = candidates
                    .iter()
                    .filter(|(location, _)| {
                        location.chain == chain
                            && location.pair / 4 == tile
                            && location.pair % 4 == slice
                    })
                    .flat_map(|(_, candidates)| {
                        candidates.iter().filter_map(|(_, set)| {
                            ((set.tile_clock, set.tile_lsr) == tile_class).then_some(set.slice_ce)
                        })
                    })
                    .collect::<BTreeSet<_>>();
                let mut best_slice = Vec::new();
                for ce in ce_classes {
                    let mut slice_selection = Vec::new();
                    for half in 0..2 {
                        let location = CarryLutLocation {
                            chain,
                            pair: tile * 4 + slice,
                            half,
                        };
                        let candidate = candidates.get(&location).and_then(|candidates| {
                            candidates
                                .iter()
                                .filter(|(_, set)| {
                                    (set.tile_clock, set.tile_lsr) == tile_class
                                        && set.slice_ce == ce
                                })
                                .min_by_key(|(pair, _)| pair.ff)
                        });
                        if let Some(&(pair, _)) = candidate {
                            slice_selection.push(SelectedCarryLutFf { location, pair });
                        }
                    }
                    if slice_selection.len() > best_slice.len() {
                        best_slice = slice_selection;
                    }
                }
                tile_selection.extend(best_slice);
            }
            if tile_selection.len() > best_tile.len() {
                best_tile = tile_selection;
            }
        }
        selected.extend(best_tile);
    }
    selected.sort_unstable_by_key(|selected| (selected.location, selected.pair.ff));
    Ok(selected)
}

fn validate_requested_carry_lut_ff_pairs(
    design: &Design,
    general_routing_ffs: &[CellId],
    locations: &BTreeMap<CellId, CarryLutLocation>,
    control_sets: &BTreeMap<CellId, FfControlSet>,
    requested_pairs: &[LutFfPair],
) -> Result<Vec<SelectedCarryLutFf>, PackingError> {
    let available_ffs = general_routing_ffs.iter().copied().collect::<BTreeSet<_>>();
    let mut used_luts = BTreeSet::new();
    let mut used_ffs = BTreeSet::new();
    let mut tile_classes = BTreeMap::new();
    let mut slice_classes = BTreeMap::new();
    let mut selected = Vec::new();
    for &pair in requested_pairs {
        let lut_name = design
            .cells()
            .get(pair.lut.0)
            .map_or_else(|| format!("cell#{}", pair.lut.0), |cell| cell.name.clone());
        let ff_name = design
            .cells()
            .get(pair.ff.0)
            .map_or_else(|| format!("cell#{}", pair.ff.0), |cell| cell.name.clone());
        let Some(&location) = locations.get(&pair.lut) else {
            return Err(PackingError::InvalidLutFfPair {
                lut: lut_name,
                ff: ff_name,
                reason: "requested LUT is not in a packed carry chain".into(),
            });
        };
        if !available_ffs.contains(&pair.ff) {
            return Err(PackingError::InvalidLutFfPair {
                lut: lut_name,
                ff: ff_name,
                reason: "requested FF is not available on general routing".into(),
            });
        }
        let data_pin = ff_data_pin(design, pair.ff)?;
        if lut_driver(design, data_pin) != Some(pair.lut) {
            return Err(PackingError::InvalidLutFfPair {
                lut: lut_name,
                ff: ff_name,
                reason: "carry LUT does not directly drive the FF data input".into(),
            });
        }
        if !used_luts.insert(pair.lut) || !used_ffs.insert(pair.ff) {
            return Err(PackingError::InvalidLutFfPair {
                lut: lut_name,
                ff: ff_name,
                reason: "carry LUT or FF occurs in more than one requested pair".into(),
            });
        }
        let Some(&set) = control_sets.get(&pair.ff) else {
            return Err(PackingError::CarryFfPacking {
                cell: ff_name,
                reason: "requested carry-result FF has no control-set record".into(),
            });
        };
        let tile = (location.chain, location.pair / 4);
        let tile_class = (set.tile_clock, set.tile_lsr);
        if tile_classes
            .insert(tile, tile_class)
            .is_some_and(|old| old != tile_class)
        {
            return Err(PackingError::InvalidLutFfPair {
                lut: lut_name,
                ff: ff_name,
                reason: "requested carry FF conflicts with the tile CLK/LSR control set".into(),
            });
        }
        let slice = (location.chain, location.pair / 4, location.pair % 4);
        if slice_classes
            .insert(slice, set.slice_ce)
            .is_some_and(|old| old != set.slice_ce)
        {
            return Err(PackingError::InvalidLutFfPair {
                lut: lut_name,
                ff: ff_name,
                reason: "requested carry FF conflicts with the slice CE control set".into(),
            });
        }
        selected.push(SelectedCarryLutFf { location, pair });
    }
    selected.sort_unstable_by_key(|selected| (selected.location, selected.pair.ff));
    Ok(selected)
}

/// Selects nets with at least `minimum_clock_sinks` recognized clock pins.
///
/// A zero threshold is treated as one. Register `CLK`, block-RAM `CLKA`/`CLKB`,
/// and PLL `CLKI` pins are recognized. At most 16 nets are returned, choosing
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
        || (kind == ResourceKind::Logic && pin.name == "CLKI")
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
        wide_lut_clusters: None,
        carry_pairs: Vec::new(),
        carry_state: CarryPackingState::Unpacked,
        lut_ff_pairs,
        general_routing_ffs,
        block_rams: Vec::new(),
        block_rams_packed: false,
        global_clocks: Vec::new(),
        global_clocks_packed: false,
        io_attributes: BTreeMap::new(),
        clock_frequencies_hz: BTreeMap::new(),
        generated_clock_periods_ps: BTreeMap::new(),
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
        wide_lut_clusters: None,
        carry_pairs: Vec::new(),
        carry_state: CarryPackingState::Unpacked,
        lut_ff_pairs,
        general_routing_ffs,
        block_rams: Vec::new(),
        block_rams_packed: false,
        global_clocks: Vec::new(),
        global_clocks_packed: false,
        io_attributes: BTreeMap::new(),
        clock_frequencies_hz: BTreeMap::new(),
        generated_clock_periods_ps: BTreeMap::new(),
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
    /// Wide-LUT packing was invoked more than once.
    WideLutsAlreadyPacked,
    /// A wide-LUT cluster contained neither two, four, nor eight LUT4 cells.
    InvalidWideLutClusterSize(usize),
    /// A wide-LUT requirement referenced an unknown cell.
    UnknownWideLutCell(CellId),
    /// A wide-LUT requirement referenced a non-LUT4 cell.
    CellIsNotWideLut {
        /// Logical cell name.
        cell: String,
    },
    /// One LUT4 occurred in more than one wide-LUT cluster.
    DuplicateWideLutCell {
        /// Logical cell name.
        cell: String,
    },
    /// A wide-LUT cluster did not contain the required F/F1/OFX/FX topology.
    InvalidWideLutStructure {
        /// First logical LUT name.
        cell: String,
    },
    /// No physical PFUMX/L6MUX21 cluster can implement the requirement.
    MissingWideLutCluster {
        /// First logical LUT name.
        cell: String,
        /// Required LUT4 count.
        size: usize,
    },
    /// Carry-pair packing was invoked more than once.
    CarryPairsAlreadyPacked,
    /// Carry-result FF packing was requested before the carry macros existed.
    CarryPairsNotPacked,
    /// Carry-result FF packing was invoked more than once.
    CarryLutFfsAlreadyPacked,
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
    /// A carry macro could not be extended with dedicated-path FFs.
    CarryFfPacking {
        /// Logical cell associated with the failure.
        cell: String,
        /// Structural or control-set reason.
        reason: String,
    },
    /// A placed carry slice cannot be routed using legal LUT input permutations.
    CarryRouting {
        /// Logical carry slice name.
        cell: String,
        /// Physical topology or placement reason.
        reason: String,
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
            Self::WideLutsAlreadyPacked => write!(f, "wide LUTs were already packed"),
            Self::InvalidWideLutClusterSize(size) => {
                write!(f, "wide-LUT cluster has unsupported size {size}")
            }
            Self::UnknownWideLutCell(cell) => {
                write!(f, "unknown wide-LUT cell ID {}", cell.0)
            }
            Self::CellIsNotWideLut { cell } => {
                write!(f, "cell `{cell}` is not a LUT4 in a wide-LUT cluster")
            }
            Self::DuplicateWideLutCell { cell } => {
                write!(f, "LUT4 `{cell}` occurs in more than one wide-LUT cluster")
            }
            Self::InvalidWideLutStructure { cell } => write!(
                f,
                "wide-LUT cluster beginning at `{cell}` has an invalid dedicated-mux topology"
            ),
            Self::MissingWideLutCluster { cell, size } => write!(
                f,
                "{size}-LUT cluster beginning at `{cell}` has no compatible PFUMX/L6MUX21 BEL sequence"
            ),
            Self::CarryPairsAlreadyPacked => write!(f, "carry pairs were already packed"),
            Self::CarryPairsNotPacked => {
                write!(f, "carry-result FFs require carry pairs to be packed first")
            }
            Self::CarryLutFfsAlreadyPacked => {
                write!(f, "carry-result FFs were already packed")
            }
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
            Self::CarryFfPacking { cell, reason } => {
                write!(
                    f,
                    "carry-result FF near `{cell}` cannot be packed: {reason}"
                )
            }
            Self::CarryRouting { cell, reason } => {
                write!(
                    f,
                    "carry slice `{cell}` has an invalid routing context: {reason}"
                )
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
            | Self::WideLutsAlreadyPacked
            | Self::InvalidWideLutClusterSize(_)
            | Self::UnknownWideLutCell(_)
            | Self::CellIsNotWideLut { .. }
            | Self::DuplicateWideLutCell { .. }
            | Self::InvalidWideLutStructure { .. }
            | Self::MissingWideLutCluster { .. }
            | Self::CarryPairsAlreadyPacked
            | Self::CarryPairsNotPacked
            | Self::CarryLutFfsAlreadyPacked
            | Self::UnknownCarryCell(_)
            | Self::CellIsNotCarrySlice { .. }
            | Self::DuplicateCarryCell { .. }
            | Self::InvalidCarryConnection { .. }
            | Self::MissingCarrySlicePair { .. }
            | Self::CarryFfPacking { .. }
            | Self::CarryRouting { .. }
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
pub fn read_architecture_cache(mut reader: impl Read) -> Result<Ecp5Architecture, ImportError> {
    // Postcard's streaming flavor obtains each scalar byte through `Read`.
    // An ECP5-85F cache contains hundreds of millions of scalar varints, so
    // decoding one contiguous slice avoids that per-byte I/O dispatch. Drop
    // the encoded input before validating or normalizing the decoded graph.
    let cache = {
        let mut encoded = Vec::new();
        reader.read_to_end(&mut encoded)?;
        postcard::from_bytes(&encoded)?
    };
    let ArchitectureCache {
        version,
        mut architecture,
    } = cache;
    if version != ARCHITECTURE_CACHE_VERSION {
        return Err(ImportError::UnsupportedCacheVersion(version));
    }
    architecture.device.compact_routing_graph()?;
    Ok(architecture)
}

type ConfigurationTileIds = Vec<Vec<(u32, u32)>>;

fn expand_location_metadata(
    file: &ArchitectureFile,
    metadata_strings: &mut StringInterner,
) -> Result<(Vec<GlobalInfoRecord>, ConfigurationTileIds), ImportError> {
    let size = (file.width * file.height) as usize;
    let mut global_info = vec![None; size];
    let mut configuration_tiles = vec![Vec::new(); size];
    for location in &file.locations {
        let index = (location.y * file.width + location.x) as usize;
        global_info[index] = Some(location.global);
        for tile in &location.tiles {
            configuration_tiles[index].push((
                metadata_strings.intern(&tile.name)?,
                metadata_strings.intern(&tile.tile_type)?,
            ));
        }
    }
    let global_info = global_info.into_iter().collect::<Option<Vec<_>>>().ok_or(
        ImportError::IncompleteLocationGrid {
            expected: size,
            actual: file.locations.len(),
        },
    )?;
    Ok((global_info, configuration_tiles))
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
    let (global_info, configuration_tiles) =
        expand_location_metadata(&file, &mut metadata_strings)?;

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
            let config_tile = location
                .tiles
                .iter()
                .find(|tile| tile.tile_type == pip.tile_type)
                .map(|tile| metadata_strings.intern(&tile.name))
                .transpose()?;
            debug_assert_eq!(id.0, pip_metadata.len());
            pip_metadata.push(CompactPipMetadata::new(
                metadata_strings.intern(&pip.tile_type)?,
                config_tile,
                metadata_strings.intern(&pip.timing_class)?,
                pip.lutperm_flags,
                pip.fixed,
            )?);
        }
    }

    add_global_clock_aliases(
        &mut device,
        &global_info,
        &mut metadata_strings,
        &mut pip_metadata,
    )?;
    device.compact_routing_graph()?;

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
        configuration_tiles,
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
        pip_metadata.push(CompactPipMetadata::new(
            tile_type,
            None,
            timing_class,
            0,
            true,
        )?);
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
    /// Reading a binary architecture cache failed.
    Io(std::io::Error),
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
            Self::Io(error) => write!(f, "cannot read architecture cache: {error}"),
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
            Self::Io(error) => Some(error),
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

impl From<std::io::Error> for ImportError {
    fn from(value: std::io::Error) -> Self {
        Self::Io(value)
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
    use std::collections::{BTreeMap, BTreeSet, VecDeque};
    use std::num::NonZeroU32;

    use struo_ir::{
        ActiveLevel as StruoActiveLevel, ArithmeticOp, ClockEdge as StruoClockEdge, EnableControl,
        MemoryCell, Netlist, RegisterCell,
    };
    use struo_target_ecp5::{
        ArithmeticMapping, IoTimingConstraints, MappingOptions, map_to_ecp5,
        map_to_ecp5_with_constraints, map_to_ecp5_with_options,
    };
    use texo_model::{
        BelId, CellId, CellPinId, Design, NetId, PinDirection, PipId, Point, ResourceKind,
        UnifiedGraph, WireId,
    };
    use texo_pnr::{
        RoutingConstraints, place_analytically_with_net_sink_weights,
        place_and_route_with_constraints, place_with_constraints, placement_from_partial_bindings,
        refine_placement_with_net_weights, route_with_placement, swap_placement_cells,
    };
    use texo_struo::{PrimitiveMetadata, import_ecp5};

    use super::{
        ArchitectureFile, BlockRamRequirement, BlockedGlobalResources, CompactIncomingPips,
        CompactPipMetadata, Ecp5Packing, FfControlSet, ForwardRouteTargetDistances,
        GlobalClockRequirement, GlobalReverseSearch, ImportError, LogicalPort, LutFfPair,
        PackagePinBinding, PackedBlockRam, PackingError, PipMetadata, SerializedCompactPipMetadata,
        TileRecord, expand, find_bel_pin, find_global_clock_requirements, logical_carry_chains,
        minimum_injective_assignment, pack_lut_ffs, pack_lut_ffs_excluding,
        pack_lut_ffs_with_pairs, parse_lpf, read_architecture, read_architecture_cache,
        resolve_lpf_port_cells, resolve_lpf_ports, valid_wide_lut_cluster,
        write_architecture_cache,
    };

    const FIXTURE: &str = include_str!("../fixtures/minimal-ecp5.json");

    #[derive(Clone, Copy)]
    struct TestCarryHalf {
        cell: CellId,
        fci: CellPinId,
        f: CellPinId,
        fco: CellPinId,
    }

    fn add_test_carry_half(design: &mut Design, name: impl Into<String>) -> TestCarryHalf {
        let cell = design.add_cell(name, ResourceKind::Lut(4));
        for name in ["A", "B", "C", "D"] {
            design.add_pin(cell, name, PinDirection::Input).unwrap();
        }
        let fci = design.add_pin(cell, "FCI", PinDirection::Input).unwrap();
        let f = design.add_pin(cell, "F", PinDirection::Output).unwrap();
        let fco = design.add_pin(cell, "FCO", PinDirection::Output).unwrap();
        TestCarryHalf { cell, fci, f, fco }
    }

    fn test_carry_chain(pair_count: usize) -> (Design, Vec<[CellId; 2]>, Vec<NetId>) {
        let mut design = Design::new();
        let halves = (0..pair_count)
            .map(|pair| {
                [
                    add_test_carry_half(&mut design, format!("carry{pair}_0")),
                    add_test_carry_half(&mut design, format!("carry{pair}_1")),
                ]
            })
            .collect::<Vec<_>>();
        let mut nets = Vec::new();
        for (pair, half) in halves.iter().enumerate() {
            nets.push(
                design
                    .add_net(format!("carry{pair}_internal"), half[0].fco, [half[1].fci])
                    .unwrap(),
            );
            if let Some(successor) = halves.get(pair + 1) {
                nets.push(
                    design
                        .add_net(format!("carry{pair}_next"), half[1].fco, [successor[0].fci])
                        .unwrap(),
                );
            }
        }
        let pairs = halves
            .iter()
            .map(|half| [half[0].cell, half[1].cell])
            .collect();
        (design, pairs, nets)
    }

    fn ensure_carry_ff_bels(architecture: &mut super::Ecp5Architecture) {
        let slots = architecture
            .device()
            .bels_of_kind(ResourceKind::Lut(4))
            .iter()
            .copied()
            .filter_map(|bel| {
                let metadata = architecture.bel_metadata(bel);
                (metadata.bel_type == "TRELLIS_COMB")
                    .then(|| metadata.z.checked_add(1))
                    .flatten()
                    .map(|z| (architecture.device().bels()[bel.0].point, z))
            })
            .collect::<BTreeSet<_>>();
        let mut existing = architecture
            .device()
            .bels_of_kind(ResourceKind::Register)
            .iter()
            .copied()
            .map(|bel| {
                (
                    architecture.device().bels()[bel.0].point,
                    architecture.bel_metadata(bel).z,
                )
            })
            .collect::<BTreeSet<_>>();
        let ff_type = u32::try_from(
            architecture
                .metadata_strings
                .iter()
                .position(|value| value == "TRELLIS_FF")
                .unwrap(),
        )
        .unwrap();
        for (point, z) in slots {
            if !existing.insert((point, z)) {
                continue;
            }
            let bel = architecture
                .device
                .add_bel(
                    format!("test_ff_x{}_y{}_z{z}", point.x, point.y),
                    ResourceKind::Register,
                    point,
                )
                .unwrap();
            for (name, direction) in [
                ("DI", PinDirection::Input),
                ("M", PinDirection::Input),
                ("CLK", PinDirection::Input),
                ("LSR", PinDirection::Input),
                ("CE", PinDirection::Input),
                ("Q", PinDirection::Output),
            ] {
                let wire = architecture
                    .device
                    .add_wire(
                        format!("test_ff_x{}_y{}_z{z}_{name}", point.x, point.y),
                        point,
                        1,
                    )
                    .unwrap();
                architecture
                    .device
                    .add_bel_pin(bel, name, direction, wire)
                    .unwrap();
            }
            architecture.bel_metadata.push(super::CompactBelMetadata {
                bel_type: ff_type,
                z,
            });
        }
    }

    fn carry_pair_with_ff_fanouts(
        first_fanout: usize,
        second_fanout: usize,
    ) -> (Design, [CellId; 2], Vec<CellId>) {
        let mut design = Design::new();
        let halves = [
            add_test_carry_half(&mut design, "carry0"),
            add_test_carry_half(&mut design, "carry1"),
        ];
        let pair = [halves[0].cell, halves[1].cell];
        design
            .add_net("carry_internal", halves[0].fco, [halves[1].fci])
            .unwrap();
        let mut ffs = Vec::new();
        for (half, fanout) in [first_fanout, second_fanout].into_iter().enumerate() {
            let mut sinks = Vec::new();
            for index in 0..fanout {
                let ff = design.add_cell(format!("ff{half}_{index}"), ResourceKind::Register);
                sinks.push(design.add_pin(ff, "DI", PinDirection::Input).unwrap());
                for name in ["CLK", "CE", "LSR"] {
                    design.add_pin(ff, name, PinDirection::Input).unwrap();
                }
                design.add_pin(ff, "Q", PinDirection::Output).unwrap();
                ffs.push(ff);
            }
            if !sinks.is_empty() {
                design
                    .add_net(format!("carry_f{half}"), halves[half].f, sinks)
                    .unwrap();
            }
        }
        (design, pair, ffs)
    }

    fn add_carry_ff_route_fixture(architecture: &mut super::Ecp5Architecture) {
        let zero = u32::try_from(
            architecture
                .metadata_strings
                .iter()
                .position(|value| value == "zero")
                .unwrap(),
        )
        .unwrap();
        let default = u32::try_from(
            architecture
                .metadata_strings
                .iter()
                .position(|value| value == "default")
                .unwrap(),
        )
        .unwrap();
        let plc2 = u32::try_from(
            architecture
                .metadata_strings
                .iter()
                .position(|value| value == "PLC2")
                .unwrap(),
        )
        .unwrap();
        let general = architecture
            .device
            .add_wire("test_carry_f_general", Point::new(0, 0), 1)
            .unwrap();
        let mut by_slot = BTreeMap::new();
        for &bel in architecture.device().bels_of_kind(ResourceKind::Register) {
            let metadata = architecture.bel_metadata(bel);
            if metadata.bel_type == "TRELLIS_FF" {
                by_slot.insert((architecture.device().bels()[bel.0].point, metadata.z), bel);
            }
        }
        let carry_luts = architecture
            .device()
            .bels_of_kind(ResourceKind::Lut(4))
            .iter()
            .copied()
            .filter(|&bel| architecture.bel_metadata(bel).z == 0)
            .collect::<Vec<_>>();
        for lut in carry_luts {
            let point = architecture.device().bels()[lut.0].point;
            let ff = by_slot[&(point, 1)];
            let f = find_bel_pin(architecture.device(), lut, "F")
                .map(|pin| architecture.device().bel_pins()[pin.0].wire)
                .unwrap();
            let di = find_bel_pin(architecture.device(), ff, "DI")
                .map(|pin| architecture.device().bel_pins()[pin.0].wire)
                .unwrap();
            let pip = architecture.device.add_pip(f, di, false, 1).unwrap();
            debug_assert_eq!(pip.0, architecture.pip_metadata.len());
            architecture
                .pip_metadata
                .push(super::CompactPipMetadata::new(plc2, None, zero, 0, true).unwrap());

            let pip = architecture.device.add_pip(f, general, false, 1).unwrap();
            debug_assert_eq!(pip.0, architecture.pip_metadata.len());
            architecture
                .pip_metadata
                .push(super::CompactPipMetadata::new(plc2, None, default, 0, false).unwrap());
        }
        for &ff in by_slot.values() {
            let m = find_bel_pin(architecture.device(), ff, "M")
                .map(|pin| architecture.device().bel_pins()[pin.0].wire)
                .unwrap();
            let pip = architecture.device.add_pip(general, m, false, 1).unwrap();
            debug_assert_eq!(pip.0, architecture.pip_metadata.len());
            architecture
                .pip_metadata
                .push(super::CompactPipMetadata::new(plc2, None, default, 0, false).unwrap());
        }
    }

    #[test]
    fn expands_deduplicated_locations_and_package_pins() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();

        assert_eq!(architecture.device().name(), "LFE5UM5G-85F-test");
        assert_eq!(architecture.device().bels().len(), 14);
        assert_eq!(architecture.device().wires().len(), 63);
        assert_eq!(architecture.device().pips().len(), 14);
        assert_eq!(architecture.packages()[0].pins.len(), 3);
        assert_eq!(
            architecture.pip_metadata(PipId(0)),
            PipMetadata {
                fixed: false,
                tile_type: "PLC2",
                config_tile: None,
                timing_class: "default",
                lutperm_flags: 0,
            }
        );
        assert!(architecture.speed_grades().contains_key("6"));
    }

    #[test]
    fn preserves_exact_configuration_tile_ownership() {
        let mut file: ArchitectureFile = serde_json::from_str(FIXTURE).unwrap();
        file.locations[0].tiles.push(TileRecord {
            name: "R0C0:PLC2".into(),
            tile_type: "PLC2".into(),
        });

        let architecture = expand(file).unwrap();

        assert_eq!(
            architecture.pip_metadata(PipId(0)).config_tile,
            Some("R0C0:PLC2")
        );
        assert_eq!(
            architecture
                .configuration_tiles(Point::new(0, 0))
                .collect::<Vec<_>>(),
            vec![("R0C0:PLC2", "PLC2")]
        );
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
    fn architecture_cache_read_preserves_io_errors() {
        struct FailingReader;

        impl std::io::Read for FailingReader {
            fn read(&mut self, _buffer: &mut [u8]) -> std::io::Result<usize> {
                Err(std::io::Error::other("cache read failed"))
            }
        }

        let error = read_architecture_cache(FailingReader).unwrap_err();
        assert!(matches!(
            error,
            ImportError::Io(error) if error.to_string() == "cache read failed"
        ));
    }

    #[test]
    #[ignore = "requires TEXO_ECP5_85F_TXDB pointing to the full target-pack architecture"]
    fn full_85f_slice_decode_matches_stream_decode() {
        let path = std::env::var_os("TEXO_ECP5_85F_TXDB")
            .expect("set TEXO_ECP5_85F_TXDB to architecture.txdb");
        let mut scratch = [0_u8; 16 * 1024];
        let (streamed, _) = postcard::from_io::<super::ArchitectureCache, _>((
            std::io::BufReader::new(std::fs::File::open(&path).unwrap()),
            &mut scratch,
        ))
        .unwrap();
        assert_eq!(streamed.version, super::ARCHITECTURE_CACHE_VERSION);
        let mut expected = streamed.architecture;
        expected.device.compact_routing_graph().unwrap();

        let decoded =
            read_architecture_cache(std::io::BufReader::new(std::fs::File::open(path).unwrap()))
                .unwrap();
        assert_eq!(decoded, expected);
    }

    #[test]
    fn compact_pip_metadata_preserves_the_cache_5_wire_format() {
        let serialized = SerializedCompactPipMetadata {
            tile_type: 12_345,
            config_tile: Some(14_000),
            timing_class: 14_001,
            lutperm_flags: 0x403b,
            fixed: true,
        };
        let compact = CompactPipMetadata::new(
            serialized.tile_type,
            serialized.config_tile,
            serialized.timing_class,
            serialized.lutperm_flags,
            serialized.fixed,
        )
        .unwrap();

        assert_eq!(std::mem::size_of::<CompactPipMetadata>(), 8);
        let historical = postcard::to_stdvec(&serialized).unwrap();
        assert_eq!(postcard::to_stdvec(&compact).unwrap(), historical);
        assert_eq!(
            postcard::from_bytes::<CompactPipMetadata>(&historical).unwrap(),
            compact
        );
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

        assert_eq!(graph.placement_candidates(lut).unwrap().len(), 6);
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
    fn packs_a_pfumx_pair_into_one_physical_slice() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let root = design.add_cell("wide0", ResourceKind::Lut(4));
        for name in ["A", "B", "C", "D"] {
            design.add_pin(root, name, PinDirection::Input).unwrap();
        }
        design.add_pin(root, "F", PinDirection::Output).unwrap();
        design.add_pin(root, "F1", PinDirection::Input).unwrap();
        design.add_pin(root, "M", PinDirection::Input).unwrap();
        design.add_pin(root, "OFX", PinDirection::Output).unwrap();
        let child = design.add_cell("wide1", ResourceKind::Lut(4));
        for name in ["A", "B", "C", "D"] {
            design.add_pin(child, name, PinDirection::Input).unwrap();
        }
        design.add_pin(child, "F", PinDirection::Output).unwrap();
        let child_f = design.cells()[child.0]
            .pins()
            .iter()
            .copied()
            .find(|pin| design.pins()[pin.0].name == "F")
            .unwrap();
        let root_f1 = design.cells()[root.0]
            .pins()
            .iter()
            .copied()
            .find(|pin| design.pins()[pin.0].name == "F1")
            .unwrap();
        design.add_net("wide_f1", child_f, [root_f1]).unwrap();

        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();
        packing
            .pack_wide_luts(&design, &architecture, [vec![root, child]])
            .unwrap();
        let placement =
            place_with_constraints(&design, architecture.device(), packing.constraints()).unwrap();
        let root_bel = placement.bel(root).unwrap();
        let child_bel = placement.bel(child).unwrap();

        assert_eq!(packing.wide_lut_clusters(), &[vec![root, child]]);
        assert_eq!(
            architecture.device().bels()[root_bel.0].point,
            architecture.device().bels()[child_bel.0].point
        );
        assert_eq!(
            architecture.bel_metadata(root_bel).z + 4,
            architecture.bel_metadata(child_bel).z
        );
    }

    #[test]
    fn packs_an_l6mux21_cluster_into_two_adjacent_slices() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut design = Design::new();
        let mut add_lut = |name: &str, extra_inputs: &[&str], ofx: bool| {
            let cell = design.add_cell(name, ResourceKind::Lut(4));
            for pin in ["A", "B", "C", "D"]
                .into_iter()
                .chain(extra_inputs.iter().copied())
            {
                design.add_pin(cell, pin, PinDirection::Input).unwrap();
            }
            design.add_pin(cell, "F", PinDirection::Output).unwrap();
            if ofx {
                design.add_pin(cell, "OFX", PinDirection::Output).unwrap();
            }
            cell
        };
        let one_root = add_lut("one0", &["F1", "M"], true);
        let l6_root = add_lut("one1", &["FXA", "FXB", "M"], true);
        let zero_root = add_lut("zero0", &["F1", "M"], true);
        let zero_child = add_lut("zero1", &[], false);
        let cluster = vec![one_root, l6_root, zero_root, zero_child];
        let pin = |cell: CellId, name: &str| {
            design.cells()[cell.0]
                .pins()
                .iter()
                .copied()
                .find(|pin| design.pins()[pin.0].name == name)
                .unwrap()
        };
        let one_f = pin(l6_root, "F");
        let one_f1 = pin(one_root, "F1");
        let zero_f = pin(zero_child, "F");
        let zero_f1 = pin(zero_root, "F1");
        let zero_ofx = pin(zero_root, "OFX");
        let fxa = pin(l6_root, "FXA");
        let one_ofx = pin(one_root, "OFX");
        let fxb = pin(l6_root, "FXB");
        design.add_net("one_f1", one_f, [one_f1]).unwrap();
        design.add_net("zero_f1", zero_f, [zero_f1]).unwrap();
        design.add_net("l6_fxa", zero_ofx, [fxa]).unwrap();
        design.add_net("l6_fxb", one_ofx, [fxb]).unwrap();

        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();
        packing
            .pack_wide_luts(&design, &architecture, [cluster.clone()])
            .unwrap();
        let placement =
            place_with_constraints(&design, architecture.device(), packing.constraints()).unwrap();
        let bels = cluster
            .iter()
            .map(|cell| placement.bel(*cell).unwrap())
            .collect::<Vec<_>>();
        let point = architecture.device().bels()[bels[0].0].point;

        assert!(
            bels.iter()
                .all(|bel| architecture.device().bels()[bel.0].point == point)
        );
        assert_eq!(
            bels.iter()
                .map(|bel| architecture.bel_metadata(*bel).z)
                .collect::<Vec<_>>(),
            [0, 4, 8, 12]
        );
    }

    #[test]
    fn accepts_a_nested_l6mux21_lut7_cluster() {
        let mut source = Netlist::new("seven_input_parity");
        let inputs = source.add_input_port("inputs", NonZeroU32::new(7).unwrap());
        let parity = inputs[1..]
            .iter()
            .fold(inputs[0], |value, input| source.add_xor(value, *input));
        source.add_output("result", parity);
        let constraints = IoTimingConstraints::new()
            .with_input_delay_ps("inputs", 0)
            .with_output_delay_ps("result", 0);
        let mapped = map_to_ecp5_with_constraints(
            &source,
            MappingOptions {
                timing_goal_mhz: 1_500,
                ..MappingOptions::default()
            },
            &constraints,
        )
        .unwrap();
        let imported = import_ecp5(&mapped).unwrap();
        let [cluster] = imported.wide_lut_clusters() else {
            panic!("expected one LUT7 cluster")
        };
        assert!(valid_wide_lut_cluster(imported.design(), cluster));

        // The compact fixture models only half a PLC. Extend its repeated
        // logic-cell pattern to the eight slots exposed by a complete PLC.
        let mut file: ArchitectureFile = serde_json::from_str(FIXTURE).unwrap();
        let tail = file.location_types[0].bels[..4].to_vec();
        let top_mux_pins = tail[1]
            .pins
            .iter()
            .filter(|pin| matches!(pin.name.as_str(), "FXA" | "FXB" | "M" | "OFX"))
            .cloned()
            .collect::<Vec<_>>();
        file.location_types[0].bels[3].pins.extend(top_mux_pins);
        for (index, mut bel) in tail.into_iter().enumerate() {
            bel.name = format!("LUT7.EXTRA{index}");
            bel.z += 16;
            file.location_types[0].bels.push(bel);
        }
        let architecture = expand(file).unwrap();
        let mut packing = Ecp5Packing::default();

        packing
            .pack_wide_luts(imported.design(), &architecture, [cluster.clone()])
            .unwrap();

        assert_eq!(packing.wide_lut_clusters(), std::slice::from_ref(cluster));
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
        let pair = *imported.carry_pairs().last().unwrap();
        let mut packing = pack_lut_ffs(imported.design(), &architecture).unwrap();

        packing
            .pack_carry_pairs(imported.design(), &architecture, [pair])
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
            assert_eq!(architecture.bel_metadata(*first).z, 0);
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
    fn rejects_a_malformed_or_reversed_split_ccu2c_pair_before_placement() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let mut disconnected = Design::new();
        let first = add_test_carry_half(&mut disconnected, "first");
        let second = add_test_carry_half(&mut disconnected, "second");
        let mut packing = Ecp5Packing::default();

        assert!(matches!(
            packing.pack_carry_pairs(&disconnected, &architecture, [[first.cell, second.cell]],),
            Err(PackingError::InvalidCarryConnection { .. })
        ));
        assert!(packing.carry_pairs().is_empty());
        assert!(packing.constraints().groups().is_empty());

        let (connected, pairs, _) = test_carry_chain(1);
        assert!(matches!(
            packing.pack_carry_pairs(&connected, &architecture, [[pairs[0][1], pairs[0][0]]],),
            Err(PackingError::InvalidCarryConnection { .. })
        ));
        assert!(packing.carry_pairs().is_empty());
        assert!(packing.constraints().groups().is_empty());
    }

    #[test]
    fn blocks_only_cross_half_lut_permutations_at_placed_carry_bels() {
        let mut architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let (design, pairs, _) = test_carry_chain(1);
        let pair = pairs[0];
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();
        packing
            .pack_carry_pairs(&design, &architecture, [pair])
            .unwrap();
        let placement =
            place_with_constraints(&design, architecture.device(), packing.constraints()).unwrap();
        let carry_bel = placement.bel(pair[0]).unwrap();
        let carry_wires = ["A", "B"].map(|name| {
            let pin = find_bel_pin(architecture.device(), carry_bel, name).unwrap();
            architecture.device().bel_pins()[pin.0].wire
        });
        let illegal_at_carry = architecture
            .device()
            .pips()
            .iter()
            .position(|pip| pip.to() == carry_wires[0])
            .map(PipId)
            .unwrap();
        let legal_at_carry = architecture
            .device()
            .pips()
            .iter()
            .position(|pip| pip.to() == carry_wires[1])
            .map(PipId)
            .unwrap();
        let other_bel = architecture
            .device()
            .bels_of_kind(ResourceKind::Lut(4))
            .iter()
            .copied()
            .find(|&bel| architecture.bel_metadata(bel).z == 0 && bel != carry_bel)
            .unwrap();
        let other_a = find_bel_pin(architecture.device(), other_bel, "A").unwrap();
        let other_a_wire = architecture.device().bel_pins()[other_a.0].wire;
        let illegal_at_other_lut = architecture
            .device()
            .pips()
            .iter()
            .position(|pip| pip.to() == other_a_wire)
            .map(PipId)
            .unwrap();
        // Physical C -> logical A crosses the carry-legal A/B and C/D halves.
        architecture.pip_metadata[illegal_at_carry.0].lutperm_flags = 0x4002;
        architecture.pip_metadata[illegal_at_other_lut.0].lutperm_flags = 0x4002;
        // Physical B -> logical A remains within the A/B half.
        architecture.pip_metadata[legal_at_carry.0].lutperm_flags = 0x4001;

        let constraints = packing
            .global_routing_constraints(&design, &architecture, &placement)
            .unwrap();

        assert!(constraints.blocked_pips().contains(&illegal_at_carry));
        assert!(!constraints.blocked_pips().contains(&legal_at_carry));
        assert!(!constraints.blocked_pips().contains(&illegal_at_other_lut));
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
        let pairs = [
            imported.carry_pairs()[1],
            *imported.carry_pairs().last().unwrap(),
        ];

        packing
            .pack_carry_pairs(imported.design(), &architecture, pairs)
            .unwrap();

        assert_eq!(imported.carry_pairs().len(), 4);
        let cells = pairs.into_iter().flatten().collect::<Vec<_>>();
        let group = packing
            .constraints()
            .groups()
            .iter()
            .find(|group| group.cells == cells)
            .unwrap();
        assert!(!group.assignments.is_empty());
        let reference_origin = architecture.device().bels()[group.assignments[0][0].0].point;
        let reference_offsets = group.assignments[0]
            .iter()
            .map(|&bel| {
                let point = architecture.device().bels()[bel.0].point;
                (
                    i64::from(point.x) - i64::from(reference_origin.x),
                    i64::from(point.y) - i64::from(reference_origin.y),
                )
            })
            .collect::<Vec<_>>();
        for assignment in group.assignments.iter() {
            assert_eq!(architecture.bel_metadata(assignment[0]).z, 0);
            let origin = architecture.device().bels()[assignment[0].0].point;
            let offsets = assignment
                .iter()
                .map(|&bel| {
                    let point = architecture.device().bels()[bel.0].point;
                    (
                        i64::from(point.x) - i64::from(origin.x),
                        i64::from(point.y) - i64::from(origin.y),
                    )
                })
                .collect::<Vec<_>>();
            assert_eq!(offsets, reference_offsets);
            let first_fco = find_bel_pin(architecture.device(), assignment[1], "FCO").unwrap();
            let second_fci = find_bel_pin(architecture.device(), assignment[2], "FCI").unwrap();
            assert_eq!(
                architecture.device().bel_pins()[first_fco.0].wire,
                architecture.device().bel_pins()[second_fci.0].wire
            );
        }
    }

    #[test]
    fn routes_a_complete_carry_chain_only_over_fixed_fco_fci_arcs() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let (design, pairs, carry_nets) = test_carry_chain(2);
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();
        packing
            .pack_carry_pairs(&design, &architecture, pairs)
            .unwrap();

        let result =
            place_and_route_with_constraints(&design, architecture.device(), packing.constraints())
                .unwrap();
        let mut saw_carry_arc = false;
        for net in carry_nets {
            let route = result.routes.iter().find(|route| route.net == net).unwrap();
            for arc in &route.arcs {
                saw_carry_arc = true;
                // Trellis may canonicalize a dedicated connection as one
                // shared wire, in which case this path legitimately has no
                // PIP. Any explicit PIP on the path must be a fixed zero-delay
                // carry alias rather than general routing.
                assert!(arc.pips.iter().all(|&pip| {
                    let metadata = architecture.pip_metadata(pip);
                    metadata.fixed && metadata.timing_class == "zero"
                }));
            }
        }
        assert!(saw_carry_arc);
    }

    #[test]
    #[ignore = "requires TEXO_ECP5_85F_TXDB pointing to the full target-pack architecture"]
    fn full_85f_carry_assignment_rows_are_one_translation_shape() {
        let path = std::env::var_os("TEXO_ECP5_85F_TXDB")
            .expect("set TEXO_ECP5_85F_TXDB to architecture.txdb");
        let architecture =
            read_architecture_cache(std::io::BufReader::new(std::fs::File::open(path).unwrap()))
                .unwrap();
        assert_eq!(architecture.device().name(), "LFE5UM5G-85F");
        let (design, pairs, _) = test_carry_chain(128);
        let mut packing = Ecp5Packing::default();
        packing
            .pack_carry_pairs(&design, &architecture, pairs)
            .unwrap();
        let group = packing.constraints().groups().first().unwrap();
        assert!(group.assignments.len() > 1_000);
        let reference_origin = architecture.device().bels()[group.assignments[0][0].0].point;
        let reference_offsets = group.assignments[0]
            .iter()
            .map(|&bel| {
                let point = architecture.device().bels()[bel.0].point;
                (
                    i64::from(point.x) - i64::from(reference_origin.x),
                    i64::from(point.y) - i64::from(reference_origin.y),
                )
            })
            .collect::<Vec<_>>();
        for assignment in group.assignments.iter() {
            let origin = architecture.device().bels()[assignment[0].0].point;
            let offsets = assignment
                .iter()
                .map(|&bel| {
                    let point = architecture.device().bels()[bel.0].point;
                    (
                        i64::from(point.x) - i64::from(origin.x),
                        i64::from(point.y) - i64::from(origin.y),
                    )
                })
                .collect::<Vec<_>>();
            assert_eq!(offsets, reference_offsets);
        }
    }

    #[test]
    fn packs_maximum_compatible_carry_ffs_and_leaves_duplicate_fanout_routed() {
        let mut architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        ensure_carry_ff_bels(&mut architecture);
        let (design, pair, ffs) = carry_pair_with_ff_fanouts(2, 1);
        let controls = ffs.iter().copied().map(|cell| FfControlSet {
            cell,
            slice_ce: 1,
            tile_clock: 2,
            tile_lsr: 3,
        });
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();
        packing
            .pack_carry_pairs(&design, &architecture, [pair])
            .unwrap();

        packing
            .pack_carry_lut_ffs(&design, &architecture, controls)
            .unwrap();

        assert_eq!(
            packing.lut_ff_pairs(),
            &[
                LutFfPair {
                    lut: pair[0],
                    ff: ffs[0],
                },
                LutFfPair {
                    lut: pair[1],
                    ff: ffs[2],
                },
            ]
        );
        assert_eq!(packing.general_routing_ffs(), &[ffs[1]]);
        let data_pins = ffs
            .iter()
            .map(|&ff| super::ff_data_pin(&design, ff).unwrap())
            .collect::<Vec<_>>();
        assert!(
            !packing
                .constraints()
                .pin_name_bindings()
                .contains_key(&data_pins[0])
        );
        assert_eq!(
            packing.constraints().pin_name_bindings()[&data_pins[1]],
            "M"
        );
        assert!(
            !packing
                .constraints()
                .pin_name_bindings()
                .contains_key(&data_pins[2])
        );
        let group = packing
            .constraints()
            .groups()
            .iter()
            .find(|group| group.cells == [pair[0], pair[1], ffs[0], ffs[2]])
            .unwrap();
        for row in group.assignments.iter() {
            assert_eq!(row.len(), 4);
            for (lut_bel, ff_bel) in [(row[0], row[2]), (row[1], row[3])] {
                assert_eq!(
                    architecture.device().bels()[lut_bel.0].point,
                    architecture.device().bels()[ff_bel.0].point
                );
                assert_eq!(
                    architecture.bel_metadata(lut_bel).z + 1,
                    architecture.bel_metadata(ff_bel).z
                );
            }
        }
    }

    #[test]
    fn eplace_legalization_and_detail_preserve_one_complete_carry_ff_row() {
        let mut architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        ensure_carry_ff_bels(&mut architecture);
        add_carry_ff_route_fixture(&mut architecture);
        let (design, pair, ffs) = carry_pair_with_ff_fanouts(1, 0);
        let controls = [FfControlSet {
            cell: ffs[0],
            slice_ce: 1,
            tile_clock: 2,
            tile_lsr: 3,
        }];
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();
        packing
            .pack_carry_pairs(&design, &architecture, [pair])
            .unwrap();
        packing
            .pack_carry_lut_ffs(&design, &architecture, controls)
            .unwrap();
        let group = packing
            .constraints()
            .groups()
            .iter()
            .find(|group| group.cells == [pair[0], pair[1], ffs[0]])
            .unwrap();

        let assert_complete_row = |placement: &texo_pnr::Placement| {
            let assignment = group
                .cells
                .iter()
                .map(|&cell| placement.bel(cell).unwrap())
                .collect::<Vec<_>>();
            assert!(
                group.assignments.contains(&assignment),
                "carry+FF placement must be one complete assignment row: {assignment:?}"
            );
        };
        let legalized = place_analytically_with_net_sink_weights(
            &design,
            architecture.device(),
            packing.constraints(),
            &BTreeMap::new(),
        )
        .unwrap();
        assert_complete_row(&legalized);

        let weights = design
            .nets()
            .iter()
            .enumerate()
            .map(|(index, _)| (NetId(index), 64))
            .collect::<BTreeMap<_, _>>();
        let detailed = refine_placement_with_net_weights(
            &design,
            architecture.device(),
            packing.constraints(),
            legalized,
            &weights,
        )
        .unwrap();
        assert_complete_row(&detailed);

        let result = route_with_placement(
            &design,
            architecture.device(),
            detailed,
            &RoutingConstraints::new(),
        )
        .unwrap();
        assert_complete_row(&result.placement);

        let direct_pin = super::ff_data_pin(&design, ffs[0]).unwrap();
        let source_pin = design.cells()[pair[0].0]
            .pins()
            .iter()
            .copied()
            .find(|&pin| design.pins()[pin.0].name == "F")
            .unwrap();
        let direct_net = design.pins()[source_pin.0].net().unwrap();
        let direct_route = result
            .routes
            .iter()
            .find(|route| route.net == direct_net)
            .unwrap()
            .arc(direct_pin)
            .unwrap();
        assert_eq!(
            direct_route.wires.last().copied(),
            find_bel_pin(
                architecture.device(),
                result.placement.bel(ffs[0]).unwrap(),
                "DI",
            )
            .map(|pin| architecture.device().bel_pins()[pin.0].wire)
        );
        assert!(!direct_route.pips.is_empty());
        assert!(direct_route.pips.iter().all(|&pip| {
            let metadata = architecture.pip_metadata(pip);
            metadata.fixed && metadata.timing_class == "zero"
        }));
    }

    #[test]
    fn places_and_routes_dedicated_and_general_carry_fanouts_together() {
        let mut architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        ensure_carry_ff_bels(&mut architecture);
        add_carry_ff_route_fixture(&mut architecture);
        let (design, pair, ffs) = carry_pair_with_ff_fanouts(2, 0);
        let controls = ffs.iter().copied().map(|cell| FfControlSet {
            cell,
            slice_ce: 1,
            tile_clock: 2,
            tile_lsr: 3,
        });
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();
        packing
            .pack_carry_pairs(&design, &architecture, [pair])
            .unwrap();
        packing
            .pack_carry_lut_ffs(&design, &architecture, controls)
            .unwrap();

        assert_eq!(
            packing.lut_ff_pairs(),
            &[LutFfPair {
                lut: pair[0],
                ff: ffs[0],
            }]
        );
        assert_eq!(packing.general_routing_ffs(), &[ffs[1]]);
        let result =
            place_and_route_with_constraints(&design, architecture.device(), packing.constraints())
                .unwrap();
        let direct_pin = super::ff_data_pin(&design, ffs[0]).unwrap();
        let general_pin = super::ff_data_pin(&design, ffs[1]).unwrap();
        let source_pin = design.cells()[pair[0].0]
            .pins()
            .iter()
            .copied()
            .find(|&pin| design.pins()[pin.0].name == "F")
            .unwrap();
        let net = design.pins()[source_pin.0].net().unwrap();
        let route = result.routes.iter().find(|route| route.net == net).unwrap();
        let direct = route.arc(direct_pin).unwrap();
        let general = route.arc(general_pin).unwrap();

        let lut_bel = result.placement.bel(pair[0]).unwrap();
        let direct_ff_bel = result.placement.bel(ffs[0]).unwrap();
        assert_eq!(
            architecture.device().bels()[lut_bel.0].point,
            architecture.device().bels()[direct_ff_bel.0].point
        );
        assert_eq!(
            architecture.bel_metadata(lut_bel).z + 1,
            architecture.bel_metadata(direct_ff_bel).z
        );
        assert_eq!(
            direct.wires.last().copied(),
            find_bel_pin(architecture.device(), direct_ff_bel, "DI")
                .map(|pin| architecture.device().bel_pins()[pin.0].wire)
        );
        let general_ff_bel = result.placement.bel(ffs[1]).unwrap();
        assert_eq!(
            general.wires.last().copied(),
            find_bel_pin(architecture.device(), general_ff_bel, "M")
                .map(|pin| architecture.device().bel_pins()[pin.0].wire)
        );
        assert!(!direct.pips.is_empty());
        assert!(direct.pips.iter().all(|&pip| {
            let metadata = architecture.pip_metadata(pip);
            metadata.fixed && metadata.timing_class == "zero"
        }));
        assert!(general.pips.iter().any(|&pip| {
            let metadata = architecture.pip_metadata(pip);
            !metadata.fixed && metadata.timing_class == "default"
        }));
    }

    #[test]
    fn carry_ff_selection_obeys_tile_clock_lsr_and_slice_ce_scopes() {
        for controls in [
            [(1, 10, 20), (1, 11, 20)],
            [(1, 10, 20), (1, 10, 21)],
            [(1, 10, 20), (2, 10, 20)],
        ] {
            let mut architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
            ensure_carry_ff_bels(&mut architecture);
            let (design, pair, ffs) = carry_pair_with_ff_fanouts(1, 1);
            let control_sets = ffs.iter().copied().zip(controls).map(
                |(cell, (slice_ce, tile_clock, tile_lsr))| FfControlSet {
                    cell,
                    slice_ce,
                    tile_clock,
                    tile_lsr,
                },
            );
            let mut packing = pack_lut_ffs(&design, &architecture).unwrap();
            packing
                .pack_carry_pairs(&design, &architecture, [pair])
                .unwrap();

            packing
                .pack_carry_lut_ffs(&design, &architecture, control_sets)
                .unwrap();

            assert_eq!(packing.lut_ff_pairs().len(), 1);
            assert_eq!(packing.lut_ff_pairs()[0].ff, ffs[0]);
            assert_eq!(packing.general_routing_ffs(), &[ffs[1]]);
        }
    }

    #[test]
    fn rejects_explicit_incompatible_carry_ff_control_sets() {
        let mut architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        ensure_carry_ff_bels(&mut architecture);
        let (design, pair, ffs) = carry_pair_with_ff_fanouts(1, 1);
        let controls = [
            FfControlSet {
                cell: ffs[0],
                slice_ce: 1,
                tile_clock: 2,
                tile_lsr: 3,
            },
            FfControlSet {
                cell: ffs[1],
                slice_ce: 1,
                tile_clock: 4,
                tile_lsr: 3,
            },
        ];
        let requested = [
            LutFfPair {
                lut: pair[0],
                ff: ffs[0],
            },
            LutFfPair {
                lut: pair[1],
                ff: ffs[1],
            },
        ];
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();
        packing
            .pack_carry_pairs(&design, &architecture, [pair])
            .unwrap();

        assert!(matches!(
            packing.pack_carry_lut_ffs_with_pairs(&design, &architecture, controls, requested,),
            Err(PackingError::InvalidLutFfPair { .. })
        ));
        assert!(packing.lut_ff_pairs().is_empty());
        assert_eq!(packing.general_routing_ffs(), ffs);
    }

    #[test]
    fn ff_shared_resources_use_tile_clock_lsr_and_slice_ce_scopes() {
        let mut architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        ensure_carry_ff_bels(&mut architecture);
        let mut by_slot = BTreeMap::new();
        for &bel in architecture.device().bels_of_kind(ResourceKind::Register) {
            let metadata = architecture.bel_metadata(bel);
            if metadata.bel_type == "TRELLIS_FF" {
                by_slot.insert((architecture.device().bels()[bel.0].point, metadata.z), bel);
            }
        }
        let point = by_slot.keys().find(|(_, z)| *z == 1).unwrap().0;
        let same_slice = [by_slot[&(point, 1)], by_slot[&(point, 5)]];
        let other_slice = by_slot[&(point, 9)];
        let mut design = Design::new();
        let cells = [
            design.add_cell("first", ResourceKind::Register),
            design.add_cell("second", ResourceKind::Register),
        ];

        let constraints_for = |sets: [FfControlSet; 2]| {
            let mut packing = Ecp5Packing::default();
            packing.constrain_ff_control_sets(&architecture, &sets);
            packing
        };
        let bindings = BTreeMap::from([(cells[0], same_slice[0]), (cells[1], other_slice)]);
        let incompatible_clock = constraints_for([
            FfControlSet {
                cell: cells[0],
                slice_ce: 1,
                tile_clock: 2,
                tile_lsr: 3,
            },
            FfControlSet {
                cell: cells[1],
                slice_ce: 4,
                tile_clock: 5,
                tile_lsr: 3,
            },
        ]);
        assert!(
            placement_from_partial_bindings(
                &design,
                architecture.device(),
                incompatible_clock.constraints(),
                &bindings,
            )
            .is_err()
        );

        let compatible_other_slice = constraints_for([
            FfControlSet {
                cell: cells[0],
                slice_ce: 1,
                tile_clock: 2,
                tile_lsr: 3,
            },
            FfControlSet {
                cell: cells[1],
                slice_ce: 4,
                tile_clock: 2,
                tile_lsr: 3,
            },
        ]);
        assert!(
            placement_from_partial_bindings(
                &design,
                architecture.device(),
                compatible_other_slice.constraints(),
                &bindings,
            )
            .is_ok()
        );

        let same_slice_bindings =
            BTreeMap::from([(cells[0], same_slice[0]), (cells[1], same_slice[1])]);
        assert!(
            placement_from_partial_bindings(
                &design,
                architecture.device(),
                compatible_other_slice.constraints(),
                &same_slice_bindings,
            )
            .is_err()
        );
    }

    #[test]
    fn releasing_a_carry_ff_preserves_the_rigid_carry_group() {
        let mut architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        ensure_carry_ff_bels(&mut architecture);
        let (design, pair, ffs) = carry_pair_with_ff_fanouts(1, 1);
        let controls = ffs.iter().copied().map(|cell| FfControlSet {
            cell,
            slice_ce: 1,
            tile_clock: 2,
            tile_lsr: 3,
        });
        let mut packing = pack_lut_ffs(&design, &architecture).unwrap();
        packing
            .pack_carry_pairs(&design, &architecture, [pair])
            .unwrap();
        packing
            .pack_carry_lut_ffs(&design, &architecture, controls)
            .unwrap();

        packing
            .release_lut_ff_pair(&design, pair[0], ffs[0])
            .unwrap();

        assert_eq!(
            packing.lut_ff_pairs(),
            &[LutFfPair {
                lut: pair[1],
                ff: ffs[1],
            }]
        );
        assert_eq!(packing.general_routing_ffs(), &[ffs[0]]);
        let group = packing
            .constraints()
            .groups()
            .iter()
            .find(|group| group.cells.contains(&pair[0]))
            .unwrap();
        assert_eq!(group.cells, [pair[0], pair[1], ffs[1]]);
        assert!(group.assignments.iter().all(|row| row.len() == 3));
        let released_di = super::ff_data_pin(&design, ffs[0]).unwrap();
        assert_eq!(packing.constraints().pin_name_bindings()[&released_di], "M");
    }

    #[test]
    fn recognizes_a_32_bit_counter_as_one_logical_carry_chain() {
        let mut source = Netlist::new("counter");
        let clock = source.add_input("clock");
        let state = (0..32)
            .map(|bit| source.add_register_output(format!("counter[{bit}]")))
            .collect::<Vec<_>>();
        let zero = source.add_constant(false);
        let one = source.add_constant(true);
        let increment = std::iter::once(one)
            .chain(std::iter::repeat_n(zero, 31))
            .collect::<Vec<_>>();
        let next = source
            .add_arithmetic(ArithmeticOp::Add, &state, &increment)
            .unwrap();
        for (bit, (&output, &data)) in state.iter().zip(&next).enumerate() {
            source.add_register(RegisterCell::new(
                format!("counter[{bit}]"),
                output,
                data,
                clock,
                StruoClockEdge::Rising,
                None,
                None,
            ));
        }
        source.add_output_port("counter", &state).unwrap();

        let mapped = map_to_ecp5_with_options(
            &source,
            MappingOptions {
                timing_goal_mhz: 250,
                arithmetic: ArithmeticMapping::CarryChain,
            },
        )
        .unwrap();
        let imported = import_ecp5(&mapped).unwrap();
        let chains = logical_carry_chains(imported.design(), imported.carry_pairs()).unwrap();

        // Sixteen CCU2C cells implement the 32 data bits. Struo adds one
        // feed-in and one feed-out pair, and Texo must keep all eighteen
        // pairs in one atomic physical FCI/FCO chain.
        assert_eq!(imported.carry_pairs().len(), 18);
        assert_eq!(
            chains,
            vec![
                std::iter::once(16)
                    .chain(0..16)
                    .chain(std::iter::once(17))
                    .collect::<Vec<_>>()
            ]
        );
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
    fn global_clock_assignment_minimizes_total_source_route_cost() {
        let costs = vec![
            vec![Some(20), Some(2), Some(8)],
            vec![Some(3), Some(30), Some(4)],
        ];

        assert_eq!(minimum_injective_assignment(&costs), Some(vec![1, 0]));
        assert_eq!(
            minimum_injective_assignment(&[vec![Some(1)], vec![Some(2)]]),
            None
        );
    }

    fn exhaustive_forward_target_distances(
        device: &texo_model::Device,
        source: WireId,
        targets: &[WireId],
    ) -> Vec<Option<usize>> {
        let mut distances = vec![usize::MAX; device.wires().len()];
        let mut queue = VecDeque::from([source]);
        distances[source.0] = 0;
        while let Some(wire) = queue.pop_front() {
            let next_distance = distances[wire.0] + 1;
            for (next, _) in device.routing_neighbors(wire).unwrap() {
                if distances[next.0] == usize::MAX {
                    distances[next.0] = next_distance;
                    queue.push_back(next);
                }
            }
        }
        targets
            .iter()
            .map(|target| (distances[target.0] != usize::MAX).then_some(distances[target.0]))
            .collect()
    }

    #[test]
    fn bounded_target_distances_preserve_dcca_assignment() {
        let mut device = texo_model::Device::new("dcca-costs", 1, 1).unwrap();
        let point = Point::new(0, 0);
        let source_a = device.add_wire("source-a", point, 1).unwrap();
        let source_b = device.add_wire("source-b", point, 1).unwrap();
        let target_a = device.add_wire("target-a", point, 1).unwrap();
        let target_b = device.add_wire("target-b", point, 1).unwrap();
        let target_c = device.add_wire("target-c", point, 1).unwrap();
        let unreachable = device.add_wire("unreachable", point, 1).unwrap();
        let a_to_b = device.add_wire("a-to-b", point, 1).unwrap();
        let a_to_c_0 = device.add_wire("a-to-c-0", point, 1).unwrap();
        let a_to_c_1 = device.add_wire("a-to-c-1", point, 1).unwrap();
        let b_to_a_0 = device.add_wire("b-to-a-0", point, 1).unwrap();
        let b_to_a_1 = device.add_wire("b-to-a-1", point, 1).unwrap();
        let b_to_c = device.add_wire("b-to-c", point, 1).unwrap();
        device.add_pip(source_a, target_a, false, 1).unwrap();
        device.add_pip(source_a, a_to_b, false, 1).unwrap();
        device.add_pip(a_to_b, target_b, false, 1).unwrap();
        device.add_pip(source_a, a_to_c_0, false, 1).unwrap();
        device.add_pip(a_to_c_0, a_to_c_1, false, 1).unwrap();
        device.add_pip(a_to_c_1, target_c, false, 1).unwrap();
        device.add_pip(source_b, b_to_a_0, false, 1).unwrap();
        device.add_pip(b_to_a_0, b_to_a_1, false, 1).unwrap();
        device.add_pip(b_to_a_1, target_a, false, 1).unwrap();
        device.add_pip(source_b, target_b, false, 1).unwrap();
        device.add_pip(source_b, b_to_c, false, 1).unwrap();
        device.add_pip(b_to_c, target_c, false, 1).unwrap();
        let targets = [target_a, target_b, target_c, unreachable];
        let mut search = ForwardRouteTargetDistances::new(device.wires().len(), &targets);
        let optimized = [source_a, source_b]
            .into_iter()
            .map(|source| search.distances(&device, source).0.to_vec())
            .collect::<Vec<_>>();
        let exhaustive = [source_a, source_b]
            .into_iter()
            .map(|source| exhaustive_forward_target_distances(&device, source, &targets))
            .collect::<Vec<_>>();

        assert_eq!(optimized, exhaustive);
        assert_eq!(optimized[0], [Some(1), Some(2), Some(3), None]);
        assert_eq!(optimized[1], [Some(3), Some(1), Some(2), None]);
        assert_eq!(minimum_injective_assignment(&optimized), Some(vec![0, 1]));
    }

    #[test]
    fn target_distance_search_stops_after_all_targets_and_reuses_scratch() {
        let mut device = texo_model::Device::new("bounded-dcca-search", 1, 1).unwrap();
        let point = Point::new(0, 0);
        let source = device.add_wire("source", point, 1).unwrap();
        let target = device.add_wire("target", point, 1).unwrap();
        let tail = (0..32)
            .map(|index| device.add_wire(format!("tail-{index}"), point, 1).unwrap())
            .collect::<Vec<_>>();
        device.add_pip(source, target, false, 1).unwrap();
        device.add_pip(source, tail[0], false, 1).unwrap();
        for pair in tail.windows(2) {
            device.add_pip(pair[0], pair[1], false, 1).unwrap();
        }
        let mut search = ForwardRouteTargetDistances::new(device.wires().len(), &[target, target]);

        let (distances, visited) = search.distances(&device, source);
        assert_eq!(distances, [Some(1), Some(1)]);
        assert_eq!(visited, 2);

        let (distances, visited) = search.distances(&device, target);
        assert_eq!(distances, [Some(0), Some(0)]);
        assert_eq!(visited, 1);
    }

    #[test]
    #[ignore = "release-only microbenchmark; run explicitly with --ignored --nocapture"]
    fn benchmark_bounded_target_distances_against_exhaustive_search() {
        const TAIL_LENGTH: u32 = 65_536;
        let mut device = texo_model::Device::new("dcca-distance-benchmark", 1, 1).unwrap();
        let point = Point::new(0, 0);
        let source = device.add_wire("source", point, 1).unwrap();
        let target = device.add_wire("target", point, 1).unwrap();
        device.add_pip(source, target, false, 1).unwrap();
        let mut previous_tail = None;
        for index in 0..TAIL_LENGTH {
            let wire = device.add_wire(format!("tail-{index}"), point, 1).unwrap();
            if let Some(previous) = previous_tail {
                device.add_pip(previous, wire, false, 1).unwrap();
            } else {
                device.add_pip(source, wire, false, 1).unwrap();
            }
            previous_tail = Some(wire);
        }

        let exhaustive_iterations = 20_u32;
        let exhaustive_started = std::time::Instant::now();
        for _ in 0..exhaustive_iterations {
            std::hint::black_box(exhaustive_forward_target_distances(
                &device,
                source,
                &[target],
            ));
        }
        let exhaustive_elapsed =
            exhaustive_started.elapsed().as_secs_f64() / f64::from(exhaustive_iterations);

        let mut search = ForwardRouteTargetDistances::new(device.wires().len(), &[target]);
        let bounded_iterations = 10_000_u32;
        let bounded_started = std::time::Instant::now();
        for _ in 0..bounded_iterations {
            std::hint::black_box(search.distances(&device, source));
        }
        let bounded_elapsed =
            bounded_started.elapsed().as_secs_f64() / f64::from(bounded_iterations);
        let (distances, visited) = search.distances(&device, source);

        eprintln!(
            "DCCA target distance wires={} exhaustive_ms={:.3} bounded_us={:.3} speedup={:.2}x",
            device.wires().len(),
            exhaustive_elapsed * 1.0e3,
            bounded_elapsed * 1.0e6,
            exhaustive_elapsed / bounded_elapsed,
        );
        assert_eq!(distances, [Some(1)]);
        assert_eq!(visited, 2);
        assert!(bounded_elapsed < exhaustive_elapsed);
    }

    #[test]
    fn global_clock_branch_avoids_a_reserved_clock_wire() {
        let mut device = texo_model::Device::new("clock-branches", 1, 1).unwrap();
        let point = Point::new(0, 0);
        let first_root = device.add_wire("first_root", point, 1).unwrap();
        let second_root = device.add_wire("second_root", point, 1).unwrap();
        let reserved = device.add_wire("reserved", point, 1).unwrap();
        let alternate = device.add_wire("alternate", point, 1).unwrap();
        let sink = device.add_wire("sink", point, 1).unwrap();
        device.add_pip(first_root, reserved, false, 1).unwrap();
        device.add_pip(reserved, sink, false, 1).unwrap();
        device.add_pip(second_root, alternate, false, 1).unwrap();
        device.add_pip(alternate, sink, false, 1).unwrap();
        let incoming = CompactIncomingPips::new(&device);
        let mut search = GlobalReverseSearch::new(device.wires().len());

        let (join, wires, _) = search
            .route(
                &device,
                &incoming,
                sink,
                &BTreeSet::from([first_root, second_root]),
                BlockedGlobalResources {
                    wires: &BTreeSet::from([reserved]),
                    pips: &BTreeSet::new(),
                },
                "unused_target",
            )
            .unwrap();

        assert_eq!(join, second_root);
        assert_eq!(wires, [second_root, alternate, sink]);
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
