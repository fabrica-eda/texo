//! Frontend-independent logical and physical design models.

use std::error::Error;
use std::fmt;

/// Stable index of a cell in a [`Design`].
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct CellId(pub usize);

/// Stable index of a net in a [`Design`].
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct NetId(pub usize);

/// Integer coordinate in the physical device model.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct Point {
    /// Horizontal coordinate.
    pub x: u32,
    /// Vertical coordinate.
    pub y: u32,
}

impl Point {
    /// Creates a point.
    #[must_use]
    pub const fn new(x: u32, y: u32) -> Self {
        Self { x, y }
    }

    /// Manhattan distance between two points.
    #[must_use]
    pub fn manhattan(self, other: Self) -> u64 {
        u64::from(self.x.abs_diff(other.x)) + u64::from(self.y.abs_diff(other.y))
    }
}

/// Coarse resource class shared by cells and compatible sites.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum ResourceKind {
    /// General combinational or sequential logic.
    Logic,
    /// Embedded memory.
    Memory,
    /// Package-facing input/output resource.
    Io,
}

/// A placeable physical site.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Site {
    /// Site coordinate.
    pub point: Point,
    /// Resource accepted by this site.
    pub kind: ResourceKind,
}

/// A technology-mapped cell.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Cell {
    /// Unique human-readable name.
    pub name: String,
    /// Required physical resource class.
    pub kind: ResourceKind,
}

/// A logical net connecting two or more cells.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Net {
    /// Unique human-readable name.
    pub name: String,
    /// Connected cells. The first terminal is treated as the source by the
    /// reference router, although direction belongs in target-specific data.
    pub terminals: Vec<CellId>,
}

/// Owned technology-mapped design consumed by `PnR`.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Design {
    cells: Vec<Cell>,
    nets: Vec<Net>,
}

impl Design {
    /// Creates an empty design.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            cells: Vec::new(),
            nets: Vec::new(),
        }
    }

    /// Appends a cell and returns its stable ID.
    pub fn add_cell(&mut self, name: impl Into<String>, kind: ResourceKind) -> CellId {
        let id = CellId(self.cells.len());
        self.cells.push(Cell {
            name: name.into(),
            kind,
        });
        id
    }

    /// Appends a net after validating its terminal IDs.
    ///
    /// # Errors
    ///
    /// Returns an error when fewer than two terminals are supplied or a cell
    /// ID does not exist.
    pub fn add_net(
        &mut self,
        name: impl Into<String>,
        terminals: impl IntoIterator<Item = CellId>,
    ) -> Result<NetId, ModelError> {
        let terminals: Vec<_> = terminals.into_iter().collect();
        if terminals.len() < 2 {
            return Err(ModelError::TooFewTerminals);
        }
        if let Some(id) = terminals.iter().find(|id| id.0 >= self.cells.len()) {
            return Err(ModelError::UnknownCell(*id));
        }
        let id = NetId(self.nets.len());
        self.nets.push(Net {
            name: name.into(),
            terminals,
        });
        Ok(id)
    }

    /// Cells in stable ID order.
    #[must_use]
    pub fn cells(&self) -> &[Cell] {
        &self.cells
    }

    /// Nets in stable ID order.
    #[must_use]
    pub fn nets(&self) -> &[Net] {
        &self.nets
    }
}

/// Physical target model.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Device {
    width: u32,
    height: u32,
    sites: Vec<Site>,
}

impl Device {
    /// Creates a rectangular grid containing one logic site per coordinate.
    ///
    /// # Errors
    ///
    /// Returns an error for a zero-sized dimension.
    pub fn rectangular_logic(width: u32, height: u32) -> Result<Self, ModelError> {
        if width == 0 || height == 0 {
            return Err(ModelError::EmptyDevice);
        }
        let mut sites = Vec::new();
        for y in 0..height {
            for x in 0..width {
                sites.push(Site {
                    point: Point::new(x, y),
                    kind: ResourceKind::Logic,
                });
            }
        }
        Ok(Self {
            width,
            height,
            sites,
        })
    }

    /// Device width in grid coordinates.
    #[must_use]
    pub const fn width(&self) -> u32 {
        self.width
    }

    /// Device height in grid coordinates.
    #[must_use]
    pub const fn height(&self) -> u32 {
        self.height
    }

    /// All placeable sites.
    #[must_use]
    pub fn sites(&self) -> &[Site] {
        &self.sites
    }

    /// Whether a point lies in the routing grid.
    #[must_use]
    pub const fn contains(&self, point: Point) -> bool {
        point.x < self.width && point.y < self.height
    }
}

/// Invalid logical or physical model.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ModelError {
    /// A net needs a source and at least one sink.
    TooFewTerminals,
    /// A net referred to a missing cell.
    UnknownCell(CellId),
    /// A physical device had no usable area.
    EmptyDevice,
}

impl fmt::Display for ModelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooFewTerminals => write!(f, "a net must have at least two terminals"),
            Self::UnknownCell(id) => write!(f, "unknown cell ID {}", id.0),
            Self::EmptyDevice => write!(f, "device dimensions must be non-zero"),
        }
    }
}

impl Error for ModelError {}

#[cfg(test)]
mod tests {
    use super::{CellId, Design, Device, ModelError, Point, ResourceKind};

    #[test]
    fn rejects_unknown_net_terminal() {
        let mut design = Design::new();
        let a = design.add_cell("a", ResourceKind::Logic);
        assert_eq!(
            design.add_net("bad", [a, CellId(99)]),
            Err(ModelError::UnknownCell(CellId(99)))
        );
    }

    #[test]
    fn builds_rectangular_grid_in_stable_order() {
        let device = Device::rectangular_logic(2, 2).unwrap();
        let points: Vec<_> = device.sites().iter().map(|site| site.point).collect();
        assert_eq!(
            points,
            [
                Point::new(0, 0),
                Point::new(1, 0),
                Point::new(0, 1),
                Point::new(1, 1)
            ]
        );
    }
}
