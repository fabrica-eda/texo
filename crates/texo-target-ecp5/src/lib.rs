//! Versioned Project Trellis architecture import for ECP5.
//!
//! Project Trellis exposes its routing graph through C++/Python. The companion
//! `tools/export_ecp5.py` script snapshots that graph into the schema defined
//! here. Runtime placement and routing then use only Rust and [`texo_model`].

use std::collections::BTreeMap;
use std::error::Error;
use std::fmt;
use std::io::Read;

use serde::{Deserialize, Serialize};
use texo_model::{BelId, Device, ModelError, PinDirection, PipId, Point, ResourceKind, WireId};

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
    use texo_model::{CellId, ResourceKind, UnifiedGraph};
    use texo_struo::import_ecp5;

    use super::{PipMetadata, read_architecture};

    const FIXTURE: &str = include_str!("../fixtures/minimal-ecp5.json");

    #[test]
    fn expands_deduplicated_locations_and_package_pins() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();

        assert_eq!(architecture.device().name(), "LFE5UM5G-85F-test");
        assert_eq!(architecture.device().bels().len(), 3);
        assert_eq!(architecture.device().wires().len(), 15);
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
}
