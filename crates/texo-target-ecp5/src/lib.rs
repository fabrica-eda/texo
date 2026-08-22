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

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;
use std::io::Read;

use serde::{Deserialize, Serialize};
use texo_model::{
    BelId, BelPinId, BufferSpec, CellId, CellPinId, Design, Device, ModelError, NetId,
    PinDirection, PipId, Point, ResourceKind, UnifiedGraph, WireId,
};
use texo_pnr::PlacementConstraints;

/// Current on-disk architecture schema version.
pub const SCHEMA_VERSION: u32 = 1;

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
    /// Relative delay value from the routing graph.
    pub delay: i32,
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
}

/// ECP5-specific properties attached to a generic BEL.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BelMetadata {
    /// Project Trellis BEL type.
    pub bel_type: String,
    /// Z-order within its grid location.
    pub z: i32,
}

/// ECP5-specific properties attached to a generic PIP.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PipMetadata {
    /// Whether the arc is always connected.
    pub fixed: bool,
    /// Project Trellis tile type.
    pub tile_type: String,
    /// Relative routing-graph delay.
    pub delay: i32,
    /// LUT permutation flags.
    pub lutperm_flags: u16,
}

/// Resolved package pin table.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Package {
    /// Package name.
    pub name: String,
    /// Package pin to PIO BEL mapping.
    pub pins: BTreeMap<String, BelId>,
}

/// Expanded ECP5 device ready for Texo placement and routing.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Ecp5Architecture {
    provenance: Provenance,
    device: Device,
    bel_metadata: BTreeMap<BelId, BelMetadata>,
    pip_metadata: BTreeMap<PipId, PipMetadata>,
    packages: Vec<Package>,
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

    /// Target metadata for each BEL.
    #[must_use]
    pub const fn bel_metadata(&self) -> &BTreeMap<BelId, BelMetadata> {
        &self.bel_metadata
    }

    /// Target metadata for each routing arc.
    #[must_use]
    pub const fn pip_metadata(&self) -> &BTreeMap<PipId, PipMetadata> {
        &self.pip_metadata
    }

    /// Resolved package pin tables.
    #[must_use]
    pub fn packages(&self) -> &[Package] {
        &self.packages
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
    lut_ff_pairs: Vec<LutFfPair>,
    general_routing_ffs: Vec<CellId>,
    block_rams: Vec<PackedBlockRam>,
    block_rams_packed: bool,
    global_clocks: Vec<PackedGlobalClock>,
    global_clocks_packed: bool,
    io_attributes: BTreeMap<CellId, BTreeMap<String, String>>,
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

    /// LPF commands retained because this packing stage does not implement them.
    #[must_use]
    pub fn unsupported_lpf_commands(&self) -> &[String] {
        &self.unsupported_lpf_commands
    }

    /// Applies resolved LPF locations and IO attributes atomically.
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

        self.bind_package_pins(
            design,
            architecture,
            package_name,
            resolved.package_pins.clone(),
        )?;
        self.io_attributes = io_attributes;
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
                .filter(|bel| architecture.bel_metadata()[bel].bel_type == "DP16KD")
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
                .filter(|bel| architecture.bel_metadata()[bel].bel_type == "DCCA")
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
        .bel_metadata()
        .iter()
        .filter_map(|(&bel, metadata)| {
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
    let graph = UnifiedGraph::new(design, architecture.device());
    let mut constraints = PlacementConstraints::new();
    let mut paired_luts = BTreeSet::new();
    let mut paired_ffs = BTreeSet::new();
    let mut lut_ff_pairs = Vec::new();
    let mut ff_data_pins = BTreeMap::new();

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
        if paired_luts.contains(&lut) {
            continue;
        }
        let assignments = lut_ff_assignments(&graph, architecture, lut, ff)?;
        if assignments.is_empty() {
            continue;
        }
        constraints.add_group([lut, ff], assignments);
        paired_luts.insert(lut);
        paired_ffs.insert(ff);
        lut_ff_pairs.push(LutFfPair { lut, ff });
    }

    let mut general_routing_ffs = Vec::new();
    for (ff, data_pin) in ff_data_pins {
        if paired_ffs.contains(&ff) {
            continue;
        }
        let mut bound = false;
        for bel in graph.placement_candidates(ff)? {
            if architecture.bel_metadata()[&bel].bel_type != "TRELLIS_FF" {
                continue;
            }
            if let Some(m_pin) = find_bel_pin(architecture.device(), bel, "M") {
                constraints.bind_pin(data_pin, bel, m_pin);
                bound = true;
            }
        }
        if !bound {
            return Err(PackingError::MissingGeneralDataPin {
                cell: design.cells()[ff.0].name.clone(),
            });
        }
        general_routing_ffs.push(ff);
    }

    Ok(Ecp5Packing {
        constraints,
        lut_ff_pairs,
        general_routing_ffs,
        block_rams: Vec::new(),
        block_rams_packed: false,
        global_clocks: Vec::new(),
        global_clocks_packed: false,
        io_attributes: BTreeMap::new(),
        unsupported_lpf_commands: Vec::new(),
    })
}

fn lut_driver(design: &Design, data_pin: CellPinId) -> Option<CellId> {
    let net = &design.nets()[design.pins()[data_pin.0].net()?.0];
    let driver = &design.pins()[net.driver.0];
    (driver.name == "F" && design.cells()[driver.cell.0].kind == ResourceKind::Lut(4))
        .then_some(driver.cell)
}

fn lut_ff_assignments(
    graph: &UnifiedGraph<'_>,
    architecture: &Ecp5Architecture,
    lut: CellId,
    ff: CellId,
) -> Result<Vec<Vec<BelId>>, ModelError> {
    let lut_bels = graph.placement_candidates(lut)?;
    let ff_bels = graph.placement_candidates(ff)?;
    let mut assignments = Vec::new();
    for lut_bel in lut_bels {
        let lut_metadata = &architecture.bel_metadata()[&lut_bel];
        if lut_metadata.bel_type != "TRELLIS_COMB" {
            continue;
        }
        for &ff_bel in &ff_bels {
            let ff_metadata = &architecture.bel_metadata()[&ff_bel];
            if ff_metadata.bel_type == "TRELLIS_FF"
                && architecture.device().bels()[lut_bel.0].point
                    == architecture.device().bels()[ff_bel.0].point
                && lut_metadata.z.checked_add(1) == Some(ff_metadata.z)
            {
                assignments.push(vec![lut_bel, ff_bel]);
            }
        }
    }
    Ok(assignments)
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
}

impl fmt::Display for PackingError {
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
        }
    }
}

impl Error for PackingError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Model(error) => Some(error),
            Self::MissingFfDataPin { .. }
            | Self::MissingGeneralDataPin { .. }
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
            | Self::UnknownPackage(_)
            | Self::UnknownPackagePin { .. }
            | Self::UnknownIoCell(_)
            | Self::CellIsNotIo { .. }
            | Self::DuplicateIoCell { .. }
            | Self::DuplicatePackagePin(_)
            | Self::IncompatiblePackagePin { .. }
            | Self::ConflictingIoAttribute { .. } => None,
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
    let mut bel_metadata = BTreeMap::new();
    let mut pip_metadata = BTreeMap::new();
    let locations = indexed_locations(&file)?;

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
            bel_metadata.insert(
                id,
                BelMetadata {
                    bel_type: bel.bel_type.clone(),
                    z: bel.z,
                },
            );
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
            pip_metadata.insert(
                id,
                PipMetadata {
                    fixed: pip.fixed,
                    tile_type: pip.tile_type.clone(),
                    delay: pip.delay,
                    lutperm_flags: pip.lutperm_flags,
                },
            );
        }
    }

    let packages = resolve_packages(&file.packages, &bels, &device)?;
    Ok(Ecp5Architecture {
        provenance: file.provenance,
        device,
        bel_metadata,
        pip_metadata,
        packages,
    })
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
    }
    Ok(())
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
    /// Generic target model construction failed.
    Model(ModelError),
    /// File uses an unsupported schema version.
    UnsupportedSchema(u32),
    /// File describes another FPGA family.
    WrongFamily(String),
    /// One or both source revisions were omitted.
    MissingProvenance,
    /// Fine-grained BEL mode is mandatory for Texo.
    SplitSliceRequired,
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
            Self::Model(error) => write!(f, "invalid physical model: {error}"),
            Self::UnsupportedSchema(version) => {
                write!(f, "unsupported architecture schema version {version}")
            }
            Self::WrongFamily(family) => write!(f, "expected ECP5 family, found `{family}`"),
            Self::MissingProvenance => write!(f, "architecture provenance is incomplete"),
            Self::SplitSliceRequired => write!(f, "split-slice Project Trellis data is required"),
            Self::UnknownLocationType(index) => write!(f, "unknown location type {index}"),
            Self::LocationOutsideDevice { x, y } => {
                write!(f, "location ({x}, {y}) is outside the device")
            }
            Self::DuplicateLocation { x, y } => write!(f, "duplicate location ({x}, {y})"),
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

impl From<ModelError> for ImportError {
    fn from(value: ModelError) -> Self {
        Self::Model(value)
    }
}

#[cfg(test)]
mod tests {
    use struo_ir::{
        ActiveLevel as StruoActiveLevel, ClockEdge as StruoClockEdge, EnableControl, MemoryCell,
        Netlist, RegisterCell,
    };
    use struo_target_ecp5::map_to_ecp5;
    use texo_model::{CellId, Design, PinDirection, ResourceKind, UnifiedGraph};
    use texo_pnr::{place_and_route_with_constraints, place_with_constraints};
    use texo_struo::{PrimitiveMetadata, import_ecp5};

    use super::{
        BlockRamRequirement, GlobalClockRequirement, LogicalPort, PackagePinBinding,
        PackedBlockRam, PackingError, PipMetadata, find_global_clock_requirements, pack_lut_ffs,
        parse_lpf, read_architecture, resolve_lpf_port_cells, resolve_lpf_ports,
    };

    const FIXTURE: &str = include_str!("../fixtures/minimal-ecp5.json");

    #[test]
    fn expands_deduplicated_locations_and_package_pins() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();

        assert_eq!(architecture.device().name(), "LFE5UM5G-85F-test");
        assert_eq!(architecture.device().bels().len(), 8);
        assert_eq!(architecture.device().wires().len(), 41);
        assert_eq!(architecture.device().pips().len(), 3);
        assert_eq!(architecture.packages()[0].pins.len(), 1);
        assert_eq!(
            architecture.pip_metadata().values().next(),
            Some(&PipMetadata {
                fixed: false,
                tile_type: "PLC2".into(),
                delay: 1,
                lutperm_flags: 0,
            })
        );
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

        assert_eq!(graph.placement_candidates(lut).unwrap().len(), 2);
        for (index, cell) in imported.design().cells().iter().enumerate() {
            if cell.kind == ResourceKind::Io {
                assert_eq!(graph.placement_candidates(CellId(index)).unwrap().len(), 1);
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
            architecture.bel_metadata()[&lut_bel].z + 1,
            architecture.bel_metadata()[&ff_bel].z
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
            assert_eq!(architecture.bel_metadata()[&bel].bel_type, "DP16KD");
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
            architecture.bel_metadata()[&result.placement.bel(promoted.buffer).unwrap()].bel_type,
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
