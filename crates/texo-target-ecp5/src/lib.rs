//! Versioned Project Trellis architecture import for ECP5.
//!
//! Project Trellis exposes its routing graph through C++/Python. The companion
//! `tools/export_ecp5.py` script snapshots that graph into the schema defined
//! here. Runtime placement and routing then use only Rust and [`texo_model`].

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;
use std::io::Read;

use serde::{Deserialize, Serialize};
use texo_model::{
    BelId, BelPinId, CellId, CellPinId, Design, Device, ModelError, PinDirection, PipId, Point,
    ResourceKind, UnifiedGraph, WireId,
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

/// Target packing decisions consumed by grouped placement and configuration.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Ecp5Packing {
    constraints: PlacementConstraints,
    lut_ff_pairs: Vec<LutFfPair>,
    general_routing_ffs: Vec<CellId>,
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
        }
    }
}

impl Error for PackingError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Model(error) => Some(error),
            Self::MissingFfDataPin { .. }
            | Self::MissingGeneralDataPin { .. }
            | Self::UnknownPackage(_)
            | Self::UnknownPackagePin { .. }
            | Self::UnknownIoCell(_)
            | Self::CellIsNotIo { .. }
            | Self::DuplicateIoCell { .. }
            | Self::DuplicatePackagePin(_)
            | Self::IncompatiblePackagePin { .. } => None,
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
    use struo_ir::Netlist;
    use struo_target_ecp5::map_to_ecp5;
    use texo_model::{CellId, Design, PinDirection, ResourceKind, UnifiedGraph};
    use texo_pnr::place_with_constraints;
    use texo_struo::import_ecp5;

    use super::{PackagePinBinding, PipMetadata, pack_lut_ffs, read_architecture};

    const FIXTURE: &str = include_str!("../fixtures/minimal-ecp5.json");

    #[test]
    fn expands_deduplicated_locations_and_package_pins() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();

        assert_eq!(architecture.device().name(), "LFE5UM5G-85F-test");
        assert_eq!(architecture.device().bels().len(), 5);
        assert_eq!(architecture.device().wires().len(), 27);
        assert_eq!(architecture.device().pips().len(), 1);
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

    fn add_ff(design: &mut Design, name: &str) -> CellId {
        let ff = design.add_cell(name, ResourceKind::Register);
        for pin in ["DI", "CLK", "LSR", "CE"] {
            design.add_pin(ff, pin, PinDirection::Input).unwrap();
        }
        design.add_pin(ff, "Q", PinDirection::Output).unwrap();
        ff
    }
}
