//! Typed logical and physical resources exposed as one problem graph.
//!
//! Logical objects and physical resources live in separate typed arenas. A
//! [`UnifiedGraph`] projects them into a single heterogeneous graph and creates
//! placement/binding candidate edges lazily, avoiding a materialized
//! `cells × BELs` cross product.

use std::error::Error;
use std::fmt;

/// Stable index of a cell in a [`Design`].
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct CellId(pub usize);

/// Stable index of a cell pin in a [`Design`].
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct CellPinId(pub usize);

/// Stable index of a net in a [`Design`].
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct NetId(pub usize);

/// Stable index of a basic element in a [`Device`].
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct BelId(pub usize);

/// Stable index of a basic-element pin in a [`Device`].
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct BelPinId(pub usize);

/// Stable index of a routing wire in a [`Device`].
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct WireId(pub usize);

/// Stable index of a programmable interconnect point in a [`Device`].
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct PipId(pub usize);

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

/// Coarse resource class shared by logical cells and compatible BELs.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum ResourceKind {
    /// General combinational or sequential logic.
    Logic,
    /// Embedded memory.
    Memory,
    /// Package-facing input/output resource.
    Io,
}

/// Signal direction at a logical or physical pin.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum PinDirection {
    /// Signal enters the owning object.
    Input,
    /// Signal leaves the owning object.
    Output,
    /// Bidirectional signal.
    Inout,
}

impl PinDirection {
    const fn can_drive(self) -> bool {
        matches!(self, Self::Output | Self::Inout)
    }

    const fn can_sink(self) -> bool {
        matches!(self, Self::Input | Self::Inout)
    }
}

/// A technology-mapped logical cell.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Cell {
    /// Unique human-readable name.
    pub name: String,
    /// Required physical resource class.
    pub kind: ResourceKind,
    pins: Vec<CellPinId>,
}

impl Cell {
    /// Pins owned by this cell.
    #[must_use]
    pub fn pins(&self) -> &[CellPinId] {
        &self.pins
    }
}

/// A logical cell pin connected to at most one net.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CellPin {
    /// Pin name used to match a physical BEL pin.
    pub name: String,
    /// Owning cell.
    pub cell: CellId,
    /// Logical direction.
    pub direction: PinDirection,
    net: Option<NetId>,
}

impl CellPin {
    /// Connected logical net, when assigned.
    #[must_use]
    pub const fn net(&self) -> Option<NetId> {
        self.net
    }
}

/// A directed logical net with one driver and one or more sinks.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Net {
    /// Unique human-readable name.
    pub name: String,
    /// Driving logical pin.
    pub driver: CellPinId,
    /// Sink logical pins.
    pub sinks: Vec<CellPinId>,
}

/// Owned technology-mapped logical design.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Design {
    cells: Vec<Cell>,
    pins: Vec<CellPin>,
    nets: Vec<Net>,
}

impl Design {
    /// Creates an empty logical design.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            cells: Vec::new(),
            pins: Vec::new(),
            nets: Vec::new(),
        }
    }

    /// Appends a cell and returns its stable ID.
    pub fn add_cell(&mut self, name: impl Into<String>, kind: ResourceKind) -> CellId {
        let id = CellId(self.cells.len());
        self.cells.push(Cell {
            name: name.into(),
            kind,
            pins: Vec::new(),
        });
        id
    }

    /// Adds a uniquely named pin to a cell.
    ///
    /// # Errors
    ///
    /// Returns an error for an unknown cell or duplicate pin name.
    pub fn add_pin(
        &mut self,
        cell: CellId,
        name: impl Into<String>,
        direction: PinDirection,
    ) -> Result<CellPinId, ModelError> {
        let name = name.into();
        let owner = self
            .cells
            .get(cell.0)
            .ok_or(ModelError::UnknownCell(cell))?;
        if owner.pins.iter().any(|pin| self.pins[pin.0].name == name) {
            return Err(ModelError::DuplicatePin(name));
        }
        let id = CellPinId(self.pins.len());
        self.pins.push(CellPin {
            name,
            cell,
            direction,
            net: None,
        });
        self.cells[cell.0].pins.push(id);
        Ok(id)
    }

    /// Adds a directed logical net and connects all supplied pins.
    ///
    /// # Errors
    ///
    /// Returns an error for missing pins, invalid directions, no sinks, a
    /// repeated terminal, or a pin already connected to another net.
    pub fn add_net(
        &mut self,
        name: impl Into<String>,
        driver: CellPinId,
        sinks: impl IntoIterator<Item = CellPinId>,
    ) -> Result<NetId, ModelError> {
        let sinks: Vec<_> = sinks.into_iter().collect();
        if sinks.is_empty() {
            return Err(ModelError::NoSinks);
        }
        let driver_pin = self.pin(driver)?;
        if !driver_pin.direction.can_drive() {
            return Err(ModelError::InvalidDriver(driver));
        }
        if driver_pin.net.is_some() {
            return Err(ModelError::PinAlreadyConnected(driver));
        }
        let mut terminals = vec![driver];
        for &sink in &sinks {
            let sink_pin = self.pin(sink)?;
            if !sink_pin.direction.can_sink() {
                return Err(ModelError::InvalidSink(sink));
            }
            if sink_pin.net.is_some() {
                return Err(ModelError::PinAlreadyConnected(sink));
            }
            if terminals.contains(&sink) {
                return Err(ModelError::DuplicateTerminal(sink));
            }
            terminals.push(sink);
        }

        let id = NetId(self.nets.len());
        self.nets.push(Net {
            name: name.into(),
            driver,
            sinks,
        });
        for terminal in terminals {
            self.pins[terminal.0].net = Some(id);
        }
        Ok(id)
    }

    /// Cells in stable ID order.
    #[must_use]
    pub fn cells(&self) -> &[Cell] {
        &self.cells
    }

    /// Cell pins in stable ID order.
    #[must_use]
    pub fn pins(&self) -> &[CellPin] {
        &self.pins
    }

    /// Nets in stable ID order.
    #[must_use]
    pub fn nets(&self) -> &[Net] {
        &self.nets
    }

    fn cell(&self, id: CellId) -> Result<&Cell, ModelError> {
        self.cells.get(id.0).ok_or(ModelError::UnknownCell(id))
    }

    fn pin(&self, id: CellPinId) -> Result<&CellPin, ModelError> {
        self.pins.get(id.0).ok_or(ModelError::UnknownCellPin(id))
    }

    fn net(&self, id: NetId) -> Result<&Net, ModelError> {
        self.nets.get(id.0).ok_or(ModelError::UnknownNet(id))
    }
}

/// A placeable physical basic element.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Bel {
    /// Unique architecture-local name.
    pub name: String,
    /// Resource class accepted by this BEL.
    pub kind: ResourceKind,
    /// Physical coordinate.
    pub point: Point,
    pins: Vec<BelPinId>,
}

impl Bel {
    /// Physical pins owned by this BEL.
    #[must_use]
    pub fn pins(&self) -> &[BelPinId] {
        &self.pins
    }
}

/// Physical pin connecting a BEL to one routing wire.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BelPin {
    /// Pin name matched against a logical cell pin.
    pub name: String,
    /// Owning BEL.
    pub bel: BelId,
    /// Physical direction.
    pub direction: PinDirection,
    /// Routing wire reached through this pin.
    pub wire: WireId,
}

/// A routing resource with finite sharing capacity.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Wire {
    /// Unique architecture-local name.
    pub name: String,
    /// Representative physical coordinate.
    pub point: Point,
    /// Maximum number of nets that may occupy this resource.
    pub capacity: u16,
}

/// A directed or bidirectional programmable connection between two wires.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Pip {
    /// Source wire.
    pub from: WireId,
    /// Destination wire.
    pub to: WireId,
    /// Whether the connection can also be traversed from `to` to `from`.
    pub bidirectional: bool,
    /// Maximum number of nets that may occupy this resource.
    pub capacity: u16,
}

/// Physical target database.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Device {
    name: String,
    width: u32,
    height: u32,
    bels: Vec<Bel>,
    bel_pins: Vec<BelPin>,
    wires: Vec<Wire>,
    pips: Vec<Pip>,
}

impl Device {
    /// Creates an empty, non-zero-sized physical device.
    ///
    /// # Errors
    ///
    /// Returns an error for a zero-sized dimension.
    pub fn new(name: impl Into<String>, width: u32, height: u32) -> Result<Self, ModelError> {
        if width == 0 || height == 0 {
            return Err(ModelError::EmptyDevice);
        }
        Ok(Self {
            name: name.into(),
            width,
            height,
            bels: Vec::new(),
            bel_pins: Vec::new(),
            wires: Vec::new(),
            pips: Vec::new(),
        })
    }

    /// Creates a reference grid with logic BELs and explicit pin/channel wires.
    ///
    /// # Errors
    ///
    /// Returns an error for a zero-sized dimension.
    pub fn rectangular_logic(width: u32, height: u32) -> Result<Self, ModelError> {
        let mut device = Self::new("reference-grid", width, height)?;
        let mut channels = Vec::new();
        for y in 0..height {
            for x in 0..width {
                let point = Point::new(x, y);
                let input = device.add_wire(format!("X{x}Y{y}/IN"), point, 1)?;
                let output = device.add_wire(format!("X{x}Y{y}/OUT"), point, 1)?;
                let channel = device.add_wire(format!("X{x}Y{y}/CHAN"), point, 4)?;
                let bel = device.add_bel(format!("X{x}Y{y}/LOGIC"), ResourceKind::Logic, point)?;
                device.add_bel_pin(bel, "in", PinDirection::Input, input)?;
                device.add_bel_pin(bel, "out", PinDirection::Output, output)?;
                device.add_pip(output, channel, false, 1)?;
                device.add_pip(channel, input, false, 1)?;
                channels.push(channel);
            }
        }
        for y in 0..height {
            for x in 0..width {
                let here = channels[(y * width + x) as usize];
                if x + 1 < width {
                    let right = channels[(y * width + x + 1) as usize];
                    device.add_pip(here, right, true, 1)?;
                }
                if y + 1 < height {
                    let down = channels[((y + 1) * width + x) as usize];
                    device.add_pip(here, down, true, 1)?;
                }
            }
        }
        Ok(device)
    }

    /// Adds a routing wire.
    ///
    /// # Errors
    ///
    /// Returns an error for an out-of-bounds point or zero capacity.
    pub fn add_wire(
        &mut self,
        name: impl Into<String>,
        point: Point,
        capacity: u16,
    ) -> Result<WireId, ModelError> {
        self.validate_point(point)?;
        if capacity == 0 {
            return Err(ModelError::ZeroCapacity);
        }
        let id = WireId(self.wires.len());
        self.wires.push(Wire {
            name: name.into(),
            point,
            capacity,
        });
        Ok(id)
    }

    /// Adds a placeable BEL.
    ///
    /// # Errors
    ///
    /// Returns an error for an out-of-bounds point.
    pub fn add_bel(
        &mut self,
        name: impl Into<String>,
        kind: ResourceKind,
        point: Point,
    ) -> Result<BelId, ModelError> {
        self.validate_point(point)?;
        let id = BelId(self.bels.len());
        self.bels.push(Bel {
            name: name.into(),
            kind,
            point,
            pins: Vec::new(),
        });
        Ok(id)
    }

    /// Adds a uniquely named pin to a BEL.
    ///
    /// # Errors
    ///
    /// Returns an error for an unknown BEL/wire or duplicate pin name.
    pub fn add_bel_pin(
        &mut self,
        bel: BelId,
        name: impl Into<String>,
        direction: PinDirection,
        wire: WireId,
    ) -> Result<BelPinId, ModelError> {
        let name = name.into();
        let owner = self.bel(bel)?;
        self.wire(wire)?;
        if owner
            .pins
            .iter()
            .any(|pin| self.bel_pins[pin.0].name == name)
        {
            return Err(ModelError::DuplicatePin(name));
        }
        let id = BelPinId(self.bel_pins.len());
        self.bel_pins.push(BelPin {
            name,
            bel,
            direction,
            wire,
        });
        self.bels[bel.0].pins.push(id);
        Ok(id)
    }

    /// Adds a programmable connection.
    ///
    /// # Errors
    ///
    /// Returns an error for an unknown wire or zero capacity.
    pub fn add_pip(
        &mut self,
        from: WireId,
        to: WireId,
        bidirectional: bool,
        capacity: u16,
    ) -> Result<PipId, ModelError> {
        self.wire(from)?;
        self.wire(to)?;
        if capacity == 0 {
            return Err(ModelError::ZeroCapacity);
        }
        let id = PipId(self.pips.len());
        self.pips.push(Pip {
            from,
            to,
            bidirectional,
            capacity,
        });
        Ok(id)
    }

    /// Device name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
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

    /// BELs in stable ID order.
    #[must_use]
    pub fn bels(&self) -> &[Bel] {
        &self.bels
    }

    /// BEL pins in stable ID order.
    #[must_use]
    pub fn bel_pins(&self) -> &[BelPin] {
        &self.bel_pins
    }

    /// Wires in stable ID order.
    #[must_use]
    pub fn wires(&self) -> &[Wire] {
        &self.wires
    }

    /// PIPs in stable ID order.
    #[must_use]
    pub fn pips(&self) -> &[Pip] {
        &self.pips
    }

    fn validate_point(&self, point: Point) -> Result<(), ModelError> {
        if point.x < self.width && point.y < self.height {
            Ok(())
        } else {
            Err(ModelError::PointOutsideDevice(point))
        }
    }

    fn bel(&self, id: BelId) -> Result<&Bel, ModelError> {
        self.bels.get(id.0).ok_or(ModelError::UnknownBel(id))
    }

    fn bel_pin(&self, id: BelPinId) -> Result<&BelPin, ModelError> {
        self.bel_pins.get(id.0).ok_or(ModelError::UnknownBelPin(id))
    }

    fn wire(&self, id: WireId) -> Result<&Wire, ModelError> {
        self.wires.get(id.0).ok_or(ModelError::UnknownWire(id))
    }

    fn pip(&self, id: PipId) -> Result<&Pip, ModelError> {
        self.pips.get(id.0).ok_or(ModelError::UnknownPip(id))
    }
}

/// A node in the unified logical/physical problem graph.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum GraphNode {
    /// Logical cell.
    Cell(CellId),
    /// Logical cell pin.
    CellPin(CellPinId),
    /// Logical net.
    Net(NetId),
    /// Physical basic element.
    Bel(BelId),
    /// Physical basic-element pin.
    BelPin(BelPinId),
    /// Physical routing wire.
    Wire(WireId),
}

/// Relationship represented by an edge in the unified graph.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum GraphEdgeKind {
    /// Object ownership, such as a cell owning a pin.
    Contains,
    /// Logical pin-to-net connection.
    LogicalConnection,
    /// Lazily generated compatible cell-to-BEL assignment.
    PlacementCandidate,
    /// Lazily generated logical-pin-to-BEL-pin binding.
    BindingCandidate,
    /// Fixed BEL-pin-to-wire access.
    PinAccess,
    /// Programmable wire-to-wire connection.
    Pip(PipId),
}

/// One outgoing edge from a unified graph node.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct GraphArc {
    /// Adjacent node.
    pub to: GraphNode,
    /// Relationship to the adjacent node.
    pub kind: GraphEdgeKind,
}

/// Read-only unified view over a logical design and physical device.
#[derive(Clone, Copy, Debug)]
pub struct UnifiedGraph<'a> {
    design: &'a Design,
    device: &'a Device,
}

impl<'a> UnifiedGraph<'a> {
    /// Creates a unified graph view without materializing candidate edges.
    #[must_use]
    pub const fn new(design: &'a Design, device: &'a Device) -> Self {
        Self { design, device }
    }

    /// Returns compatible BELs for a cell in stable ID order.
    ///
    /// # Errors
    ///
    /// Returns an error for an unknown cell ID.
    pub fn placement_candidates(&self, cell: CellId) -> Result<Vec<BelId>, ModelError> {
        let cell_data = self.design.cell(cell)?;
        Ok(self
            .device
            .bels
            .iter()
            .enumerate()
            .filter(|(_, bel)| {
                bel.kind == cell_data.kind
                    && cell_data.pins.iter().all(|cell_pin| {
                        let logical = &self.design.pins[cell_pin.0];
                        bel.pins.iter().any(|bel_pin| {
                            let physical = &self.device.bel_pins[bel_pin.0];
                            physical.name == logical.name && physical.direction == logical.direction
                        })
                    })
            })
            .map(|(index, _)| BelId(index))
            .collect())
    }

    /// Finds the physical pin bound to a logical pin under a cell-to-BEL choice.
    ///
    /// # Errors
    ///
    /// Returns an error when an ID is invalid, the BEL has the wrong resource
    /// class, or the required physical pin does not exist.
    pub fn bound_bel_pin(&self, cell_pin: CellPinId, bel: BelId) -> Result<BelPinId, ModelError> {
        let logical = self.design.pin(cell_pin)?;
        let physical_bel = self.device.bel(bel)?;
        let logical_cell = self.design.cell(logical.cell)?;
        if logical_cell.kind != physical_bel.kind {
            return Err(ModelError::IncompatibleBinding { cell_pin, bel });
        }
        physical_bel
            .pins
            .iter()
            .copied()
            .find(|bel_pin| {
                let physical = &self.device.bel_pins[bel_pin.0];
                physical.name == logical.name && physical.direction == logical.direction
            })
            .ok_or(ModelError::IncompatibleBinding { cell_pin, bel })
    }

    /// Finds the routing wire bound to a logical pin under a placement choice.
    ///
    /// # Errors
    ///
    /// Propagates invalid or incompatible binding errors.
    pub fn bound_wire(&self, cell_pin: CellPinId, bel: BelId) -> Result<WireId, ModelError> {
        let bel_pin = self.bound_bel_pin(cell_pin, bel)?;
        Ok(self.device.bel_pin(bel_pin)?.wire)
    }

    /// Returns outgoing routing arcs from one wire in stable PIP order.
    ///
    /// # Errors
    ///
    /// Returns an error for an unknown wire ID.
    pub fn routing_neighbors(&self, wire: WireId) -> Result<Vec<(WireId, PipId)>, ModelError> {
        self.device.wire(wire)?;
        let mut neighbors = Vec::new();
        for (index, pip) in self.device.pips.iter().enumerate() {
            let pip_id = PipId(index);
            if pip.from == wire {
                neighbors.push((pip.to, pip_id));
            }
            if pip.bidirectional && pip.to == wire {
                neighbors.push((pip.from, pip_id));
            }
        }
        neighbors.sort_unstable();
        Ok(neighbors)
    }

    /// Returns outgoing arcs for any logical or physical node.
    ///
    /// Placement and pin-binding candidates are generated on demand.
    ///
    /// # Errors
    ///
    /// Returns an error for an unknown typed ID.
    pub fn neighbors(&self, node: GraphNode) -> Result<Vec<GraphArc>, ModelError> {
        let mut arcs = match node {
            GraphNode::Cell(cell) => self.cell_arcs(cell)?,
            GraphNode::CellPin(cell_pin) => self.cell_pin_arcs(cell_pin)?,
            GraphNode::Net(net) => self.net_arcs(net)?,
            GraphNode::Bel(bel) => self.bel_arcs(bel)?,
            GraphNode::BelPin(bel_pin) => self.bel_pin_arcs(bel_pin)?,
            GraphNode::Wire(wire) => self.wire_arcs(wire)?,
        };
        arcs.sort_unstable();
        Ok(arcs)
    }

    fn cell_arcs(&self, cell: CellId) -> Result<Vec<GraphArc>, ModelError> {
        let data = self.design.cell(cell)?;
        let mut arcs: Vec<_> = data
            .pins
            .iter()
            .map(|pin| GraphArc {
                to: GraphNode::CellPin(*pin),
                kind: GraphEdgeKind::Contains,
            })
            .collect();
        arcs.extend(
            self.placement_candidates(cell)?
                .into_iter()
                .map(|bel| GraphArc {
                    to: GraphNode::Bel(bel),
                    kind: GraphEdgeKind::PlacementCandidate,
                }),
        );
        Ok(arcs)
    }

    fn cell_pin_arcs(&self, cell_pin: CellPinId) -> Result<Vec<GraphArc>, ModelError> {
        let data = self.design.pin(cell_pin)?;
        let mut arcs = vec![GraphArc {
            to: GraphNode::Cell(data.cell),
            kind: GraphEdgeKind::Contains,
        }];
        if let Some(net) = data.net {
            arcs.push(GraphArc {
                to: GraphNode::Net(net),
                kind: GraphEdgeKind::LogicalConnection,
            });
        }
        for bel in self.placement_candidates(data.cell)? {
            if let Ok(bel_pin) = self.bound_bel_pin(cell_pin, bel) {
                arcs.push(GraphArc {
                    to: GraphNode::BelPin(bel_pin),
                    kind: GraphEdgeKind::BindingCandidate,
                });
            }
        }
        Ok(arcs)
    }

    fn net_arcs(&self, net: NetId) -> Result<Vec<GraphArc>, ModelError> {
        let data = self.design.net(net)?;
        let mut arcs = vec![GraphArc {
            to: GraphNode::CellPin(data.driver),
            kind: GraphEdgeKind::LogicalConnection,
        }];
        arcs.extend(data.sinks.iter().map(|sink| GraphArc {
            to: GraphNode::CellPin(*sink),
            kind: GraphEdgeKind::LogicalConnection,
        }));
        Ok(arcs)
    }

    fn bel_arcs(&self, bel: BelId) -> Result<Vec<GraphArc>, ModelError> {
        let data = self.device.bel(bel)?;
        let mut arcs: Vec<_> = data
            .pins
            .iter()
            .map(|pin| GraphArc {
                to: GraphNode::BelPin(*pin),
                kind: GraphEdgeKind::Contains,
            })
            .collect();
        for (index, _) in self.design.cells.iter().enumerate() {
            let cell = CellId(index);
            if self.placement_candidates(cell)?.contains(&bel) {
                arcs.push(GraphArc {
                    to: GraphNode::Cell(cell),
                    kind: GraphEdgeKind::PlacementCandidate,
                });
            }
        }
        Ok(arcs)
    }

    fn bel_pin_arcs(&self, bel_pin: BelPinId) -> Result<Vec<GraphArc>, ModelError> {
        let data = self.device.bel_pin(bel_pin)?;
        let mut arcs = vec![
            GraphArc {
                to: GraphNode::Bel(data.bel),
                kind: GraphEdgeKind::Contains,
            },
            GraphArc {
                to: GraphNode::Wire(data.wire),
                kind: GraphEdgeKind::PinAccess,
            },
        ];
        for (index, logical) in self.design.pins.iter().enumerate() {
            if logical.name == data.name
                && logical.direction == data.direction
                && self.placement_candidates(logical.cell)?.contains(&data.bel)
            {
                arcs.push(GraphArc {
                    to: GraphNode::CellPin(CellPinId(index)),
                    kind: GraphEdgeKind::BindingCandidate,
                });
            }
        }
        Ok(arcs)
    }

    fn wire_arcs(&self, wire: WireId) -> Result<Vec<GraphArc>, ModelError> {
        self.device.wire(wire)?;
        let mut arcs: Vec<_> = self
            .device
            .bel_pins
            .iter()
            .enumerate()
            .filter(|(_, pin)| pin.wire == wire)
            .map(|(index, _)| GraphArc {
                to: GraphNode::BelPin(BelPinId(index)),
                kind: GraphEdgeKind::PinAccess,
            })
            .collect();
        arcs.extend(
            self.routing_neighbors(wire)?
                .into_iter()
                .map(|(neighbor, pip)| GraphArc {
                    to: GraphNode::Wire(neighbor),
                    kind: GraphEdgeKind::Pip(pip),
                }),
        );
        Ok(arcs)
    }

    /// Returns a validated PIP.
    ///
    /// # Errors
    ///
    /// Returns an error for an unknown PIP ID.
    pub fn pip(&self, id: PipId) -> Result<&'a Pip, ModelError> {
        self.device.pip(id)
    }

    /// Logical design backing this view.
    #[must_use]
    pub const fn design(&self) -> &'a Design {
        self.design
    }

    /// Physical device backing this view.
    #[must_use]
    pub const fn device(&self) -> &'a Device {
        self.device
    }
}

/// Invalid logical model, physical model, or logical/physical binding.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ModelError {
    /// A logical cell ID does not exist.
    UnknownCell(CellId),
    /// A logical pin ID does not exist.
    UnknownCellPin(CellPinId),
    /// A logical net ID does not exist.
    UnknownNet(NetId),
    /// A BEL ID does not exist.
    UnknownBel(BelId),
    /// A BEL pin ID does not exist.
    UnknownBelPin(BelPinId),
    /// A wire ID does not exist.
    UnknownWire(WireId),
    /// A PIP ID does not exist.
    UnknownPip(PipId),
    /// A cell or BEL already owns a pin with this name.
    DuplicatePin(String),
    /// A logical net has no sinks.
    NoSinks,
    /// The selected driver pin cannot drive a signal.
    InvalidDriver(CellPinId),
    /// The selected sink pin cannot receive a signal.
    InvalidSink(CellPinId),
    /// The same terminal appeared more than once on a net.
    DuplicateTerminal(CellPinId),
    /// A logical pin already belongs to another net.
    PinAlreadyConnected(CellPinId),
    /// A physical device had no usable area.
    EmptyDevice,
    /// A physical resource was outside the device bounds.
    PointOutsideDevice(Point),
    /// A physical resource had no capacity.
    ZeroCapacity,
    /// A logical pin cannot bind to the selected BEL.
    IncompatibleBinding {
        /// Logical pin.
        cell_pin: CellPinId,
        /// Selected BEL.
        bel: BelId,
    },
}

impl fmt::Display for ModelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownCell(id) => write!(f, "unknown cell ID {}", id.0),
            Self::UnknownCellPin(id) => write!(f, "unknown cell-pin ID {}", id.0),
            Self::UnknownNet(id) => write!(f, "unknown net ID {}", id.0),
            Self::UnknownBel(id) => write!(f, "unknown BEL ID {}", id.0),
            Self::UnknownBelPin(id) => write!(f, "unknown BEL-pin ID {}", id.0),
            Self::UnknownWire(id) => write!(f, "unknown wire ID {}", id.0),
            Self::UnknownPip(id) => write!(f, "unknown PIP ID {}", id.0),
            Self::DuplicatePin(name) => write!(f, "duplicate pin name `{name}`"),
            Self::NoSinks => write!(f, "a logical net must have at least one sink"),
            Self::InvalidDriver(id) => write!(f, "cell pin {} cannot drive a net", id.0),
            Self::InvalidSink(id) => write!(f, "cell pin {} cannot sink a net", id.0),
            Self::DuplicateTerminal(id) => write!(f, "cell pin {} is repeated on a net", id.0),
            Self::PinAlreadyConnected(id) => {
                write!(f, "cell pin {} is already connected", id.0)
            }
            Self::EmptyDevice => write!(f, "device dimensions must be non-zero"),
            Self::PointOutsideDevice(point) => {
                write!(f, "point ({}, {}) is outside the device", point.x, point.y)
            }
            Self::ZeroCapacity => write!(f, "physical resource capacity must be non-zero"),
            Self::IncompatibleBinding { cell_pin, bel } => {
                write!(f, "cell pin {} cannot bind to BEL {}", cell_pin.0, bel.0)
            }
        }
    }
}

impl Error for ModelError {}

#[cfg(test)]
mod tests {
    use super::{
        Design, Device, GraphEdgeKind, GraphNode, PinDirection, ResourceKind, UnifiedGraph,
    };

    #[test]
    fn rejects_a_net_with_reversed_pin_directions() {
        let mut design = Design::new();
        let cell = design.add_cell("cell", ResourceKind::Logic);
        let input = design.add_pin(cell, "in", PinDirection::Input).unwrap();
        let output = design.add_pin(cell, "out", PinDirection::Output).unwrap();

        assert!(design.add_net("bad", input, [output]).is_err());
    }

    #[test]
    fn unified_graph_crosses_logical_and_physical_resources() {
        let mut design = Design::new();
        let cell = design.add_cell("cell", ResourceKind::Logic);
        let output = design.add_pin(cell, "out", PinDirection::Output).unwrap();
        let device = Device::rectangular_logic(2, 1).unwrap();
        let graph = UnifiedGraph::new(&design, &device);

        let cell_edges = graph.neighbors(GraphNode::Cell(cell)).unwrap();
        assert_eq!(
            cell_edges
                .iter()
                .filter(|edge| edge.kind == GraphEdgeKind::PlacementCandidate)
                .count(),
            2
        );

        let pin_edges = graph.neighbors(GraphNode::CellPin(output)).unwrap();
        let bel_pin = pin_edges
            .iter()
            .find(|edge| edge.kind == GraphEdgeKind::BindingCandidate)
            .unwrap()
            .to;
        let physical_edges = graph.neighbors(bel_pin).unwrap();
        assert!(
            physical_edges
                .iter()
                .any(|edge| edge.kind == GraphEdgeKind::PinAccess)
        );
    }

    #[test]
    fn incompatible_pin_removes_placement_candidate() {
        let mut design = Design::new();
        let cell = design.add_cell("cell", ResourceKind::Logic);
        design
            .add_pin(cell, "clock_enable", PinDirection::Input)
            .unwrap();
        let device = Device::rectangular_logic(1, 1).unwrap();
        let graph = UnifiedGraph::new(&design, &device);

        assert!(graph.placement_candidates(cell).unwrap().is_empty());
    }

    #[test]
    fn directed_pip_is_not_visible_from_its_destination() {
        let design = Design::new();
        let mut device = Device::new("directed", 2, 1).unwrap();
        let from = device.add_wire("from", super::Point::new(0, 0), 1).unwrap();
        let to = device.add_wire("to", super::Point::new(1, 0), 1).unwrap();
        device.add_pip(from, to, false, 1).unwrap();
        let graph = UnifiedGraph::new(&design, &device);

        assert_eq!(graph.routing_neighbors(from).unwrap().len(), 1);
        assert!(graph.routing_neighbors(to).unwrap().is_empty());
    }
}
