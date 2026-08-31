//! Architecture-capacity and placement routing-demand maps.

use texo_model::{Design, Device, PipId, Point, ResourceKind};

use crate::{Placement, RoutingConstraints};

/// General routing-channel direction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RoutingChannelOrientation {
    /// General horizontal `H##` resources.
    Horizontal,
    /// General vertical `V##` resources.
    Vertical,
}

/// Usable general-channel capacity on the device tile grid.
///
/// Every `H##` or `V##` wire is counted at the representative point supplied
/// by [`Device`], weighted by its sharing capacity. A wire is usable only when
/// at least one incident PIP remains after target routing restrictions.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RoutingCapacityMap {
    width: u32,
    height: u32,
    horizontal: Vec<u32>,
    vertical: Vec<u32>,
}

/// Architecture capacity and bbox-uniform routing demand on the device grid.
///
/// For a non-clock net with bounding-box extents `dx` and `dy`, every tile in
/// its inclusive bounding box receives horizontal demand
/// `dx / ((dx + 1) * (dy + 1))` and corresponding vertical demand with `dy`.
/// Therefore map-wide directional sums exactly reproduce directional HPWL.
#[derive(Clone, Debug, PartialEq)]
pub struct RoutingDemandMap {
    capacity: RoutingCapacityMap,
    horizontal: Vec<f64>,
    vertical: Vec<f64>,
    included_nets: usize,
    excluded_clock_nets: usize,
}

/// One rectangular capacity/demand aggregation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RoutingDemandBin {
    /// Inclusive starting column.
    pub x: u32,
    /// Inclusive starting row.
    pub y: u32,
    /// Number of covered columns.
    pub width: u32,
    /// Number of covered rows.
    pub height: u32,
    /// Usable horizontal channel capacity.
    pub horizontal_capacity: u64,
    /// Usable vertical channel capacity.
    pub vertical_capacity: u64,
    /// Bbox-uniform horizontal demand.
    pub horizontal_demand: f64,
    /// Bbox-uniform vertical demand.
    pub vertical_demand: f64,
}

impl RoutingCapacityMap {
    /// Device-grid width.
    #[must_use]
    pub const fn width(&self) -> u32 {
        self.width
    }

    /// Device-grid height.
    #[must_use]
    pub const fn height(&self) -> u32 {
        self.height
    }

    fn index(&self, point: Point) -> Option<usize> {
        (point.x < self.width && point.y < self.height)
            .then_some((point.y * self.width + point.x) as usize)
    }

    /// Directional capacity at one tile.
    #[must_use]
    pub fn capacity(&self, point: Point, orientation: RoutingChannelOrientation) -> Option<u32> {
        let index = self.index(point)?;
        Some(match orientation {
            RoutingChannelOrientation::Horizontal => self.horizontal[index],
            RoutingChannelOrientation::Vertical => self.vertical[index],
        })
    }

    /// Total directional capacity across the device.
    #[must_use]
    pub fn total_capacity(&self, orientation: RoutingChannelOrientation) -> u64 {
        let values = match orientation {
            RoutingChannelOrientation::Horizontal => &self.horizontal,
            RoutingChannelOrientation::Vertical => &self.vertical,
        };
        values.iter().map(|&value| u64::from(value)).sum()
    }
}

impl RoutingDemandMap {
    /// Architecture capacity used to normalize this demand map.
    #[must_use]
    pub const fn capacity_map(&self) -> &RoutingCapacityMap {
        &self.capacity
    }

    /// Number of non-clock nets accumulated into the map.
    #[must_use]
    pub const fn included_nets(&self) -> usize {
        self.included_nets
    }

    /// Number of clock-driven nets excluded from the map.
    #[must_use]
    pub const fn excluded_clock_nets(&self) -> usize {
        self.excluded_clock_nets
    }

    /// Directional bbox-uniform demand at one tile.
    #[must_use]
    pub fn demand(&self, point: Point, orientation: RoutingChannelOrientation) -> Option<f64> {
        let index = self.capacity.index(point)?;
        Some(match orientation {
            RoutingChannelOrientation::Horizontal => self.horizontal[index],
            RoutingChannelOrientation::Vertical => self.vertical[index],
        })
    }

    /// Capacity-normalized directional demand at one tile.
    #[must_use]
    pub fn utilization(&self, point: Point, orientation: RoutingChannelOrientation) -> Option<f64> {
        let capacity = self.capacity.capacity(point, orientation)?;
        (capacity != 0)
            .then(|| self.demand(point, orientation).unwrap_or(0.0) / f64::from(capacity))
    }

    /// Total directional demand across the device.
    #[must_use]
    pub fn total_demand(&self, orientation: RoutingChannelOrientation) -> f64 {
        let values = match orientation {
            RoutingChannelOrientation::Horizontal => &self.horizontal,
            RoutingChannelOrientation::Vertical => &self.vertical,
        };
        values.iter().sum()
    }

    /// Rectangular bin sums in deterministic row-major order.
    #[must_use]
    pub fn bins(&self, bin_size: u32) -> Vec<RoutingDemandBin> {
        let bin_size = bin_size.max(1);
        let mut bins = Vec::new();
        for y in (0..self.capacity.height).step_by(bin_size as usize) {
            for x in (0..self.capacity.width).step_by(bin_size as usize) {
                let end_x = x.saturating_add(bin_size).min(self.capacity.width);
                let end_y = y.saturating_add(bin_size).min(self.capacity.height);
                let mut bin = RoutingDemandBin {
                    x,
                    y,
                    width: end_x - x,
                    height: end_y - y,
                    horizontal_capacity: 0,
                    vertical_capacity: 0,
                    horizontal_demand: 0.0,
                    vertical_demand: 0.0,
                };
                for tile_y in y..end_y {
                    for tile_x in x..end_x {
                        let index = (tile_y * self.capacity.width + tile_x) as usize;
                        bin.horizontal_capacity += u64::from(self.capacity.horizontal[index]);
                        bin.vertical_capacity += u64::from(self.capacity.vertical[index]);
                        bin.horizontal_demand += self.horizontal[index];
                        bin.vertical_demand += self.vertical[index];
                    }
                }
                bins.push(bin);
            }
        }
        bins
    }
}

fn channel_orientation(name: &str) -> Option<RoutingChannelOrientation> {
    let local_name = name.rsplit('/').next().unwrap_or(name);
    let bytes = local_name.as_bytes();
    if bytes.len() < 3 || !bytes[1].is_ascii_digit() || !bytes[2].is_ascii_digit() {
        return None;
    }
    match bytes[0] {
        b'H' => Some(RoutingChannelOrientation::Horizontal),
        b'V' => Some(RoutingChannelOrientation::Vertical),
        _ => None,
    }
}

/// Builds the reusable architecture-capacity map after routing restrictions.
#[must_use]
pub fn routing_capacity_map(
    device: &Device,
    constraints: &RoutingConstraints,
) -> RoutingCapacityMap {
    let tile_count = (device.width() * device.height()) as usize;
    let mut map = RoutingCapacityMap {
        width: device.width(),
        height: device.height(),
        horizontal: vec![0; tile_count],
        vertical: vec![0; tile_count],
    };
    let mut usable_wire = vec![false; device.wires().len()];
    for (index, pip) in device.pips().iter().enumerate() {
        if constraints.blocked_pips().contains(&PipId(index)) {
            continue;
        }
        usable_wire[pip.from().0] = true;
        usable_wire[pip.to().0] = true;
    }
    for (index, wire) in device.wires().iter().enumerate() {
        if !usable_wire[index] {
            continue;
        }
        let Some(orientation) = channel_orientation(&wire.name) else {
            continue;
        };
        let tile = (wire.point.y * map.width + wire.point.x) as usize;
        let capacity = u32::from(wire.capacity);
        match orientation {
            RoutingChannelOrientation::Horizontal => {
                map.horizontal[tile] = map.horizontal[tile].saturating_add(capacity);
            }
            RoutingChannelOrientation::Vertical => {
                map.vertical[tile] = map.vertical[tile].saturating_add(capacity);
            }
        }
    }
    map
}

/// Builds architecture capacity and directional RUDY for one legal placement.
#[must_use]
pub fn routing_demand_map(
    design: &Design,
    device: &Device,
    placement: &Placement,
    constraints: &RoutingConstraints,
) -> RoutingDemandMap {
    routing_demand_map_with_capacity(
        design,
        device,
        placement,
        routing_capacity_map(device, constraints),
    )
}

/// Deposits one legal placement onto a reusable architecture-capacity map.
///
/// This split lets iterative placers build channel capacity once, then compare
/// many placement candidates without rescanning the physical routing graph.
#[must_use]
pub fn routing_demand_map_with_capacity(
    design: &Design,
    device: &Device,
    placement: &Placement,
    capacity: RoutingCapacityMap,
) -> RoutingDemandMap {
    let tile_count = (capacity.width * capacity.height) as usize;
    let mut map = RoutingDemandMap {
        capacity,
        horizontal: vec![0.0; tile_count],
        vertical: vec![0.0; tile_count],
        included_nets: 0,
        excluded_clock_nets: 0,
    };
    for net in design.nets() {
        let driver_cell = design.pins()[net.driver.0].cell;
        if design.cells()[driver_cell.0].kind == ResourceKind::Clock {
            map.excluded_clock_nets += 1;
            continue;
        }
        let Some(driver) = placement.point(driver_cell, device) else {
            continue;
        };
        let mut minimum_x = driver.x;
        let mut maximum_x = driver.x;
        let mut minimum_y = driver.y;
        let mut maximum_y = driver.y;
        for &sink in &net.sinks {
            let sink_cell = design.pins()[sink.0].cell;
            if let Some(point) = placement.point(sink_cell, device) {
                minimum_x = minimum_x.min(point.x);
                maximum_x = maximum_x.max(point.x);
                minimum_y = minimum_y.min(point.y);
                maximum_y = maximum_y.max(point.y);
            }
        }
        map.included_nets += 1;
        let dx = maximum_x - minimum_x;
        let dy = maximum_y - minimum_y;
        let area = f64::from(dx + 1) * f64::from(dy + 1);
        let horizontal = f64::from(dx) / area;
        let vertical = f64::from(dy) / area;
        for y in minimum_y..=maximum_y {
            for x in minimum_x..=maximum_x {
                let tile = (y * map.capacity.width + x) as usize;
                map.horizontal[tile] += horizontal;
                map.vertical[tile] += vertical;
            }
        }
    }
    map
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use texo_model::{Design, Device, PinDirection, Point, ResourceKind};

    use super::{
        RoutingChannelOrientation, routing_capacity_map, routing_demand_map,
        routing_demand_map_with_capacity,
    };
    use crate::{Placement, RoutingConstraints};

    fn placed_net(
        driver_kind: ResourceKind,
        driver_point: Point,
        sink_point: Point,
    ) -> (Design, Device, Placement) {
        let mut design = Design::new();
        let driver = design.add_cell("driver", driver_kind);
        let sink = design.add_cell("sink", ResourceKind::Register);
        let output = design.add_pin(driver, "O", PinDirection::Output).unwrap();
        let input = design.add_pin(sink, "I", PinDirection::Input).unwrap();
        design.add_net("net", output, [input]).unwrap();

        let width = driver_point.x.max(sink_point.x) + 1;
        let height = driver_point.y.max(sink_point.y) + 1;
        let mut device = Device::new("demand", width, height).unwrap();
        let driver_bel = device.add_bel("DRIVER", driver_kind, driver_point).unwrap();
        let sink_bel = device
            .add_bel("SINK", ResourceKind::Register, sink_point)
            .unwrap();
        let placement = Placement {
            bindings: vec![driver_bel, sink_bel],
            pin_bindings: BTreeMap::new(),
        };
        (design, device, placement)
    }

    fn assert_close(actual: f64, expected: f64) {
        assert!((actual - expected).abs() < 1.0e-9, "{actual} != {expected}");
    }

    #[test]
    fn blocked_only_incident_pip_removes_channel_capacity() {
        let mut device = Device::new("capacity", 3, 2).unwrap();
        let active_h = device
            .add_wire("R0C0/H02E0001", Point::new(0, 0), 2)
            .unwrap();
        let blocked_h = device
            .add_wire("R0C1/H02E0001", Point::new(1, 0), 3)
            .unwrap();
        let active_v = device
            .add_wire("R1C2/V06N0003", Point::new(2, 1), 4)
            .unwrap();
        let endpoint_a = device.add_wire("R0C0/JA", Point::new(0, 0), 1).unwrap();
        let endpoint_b = device.add_wire("R0C1/JB", Point::new(1, 0), 1).unwrap();
        let endpoint_c = device.add_wire("R1C2/JC", Point::new(2, 1), 1).unwrap();
        device.add_pip(active_h, endpoint_a, false, 1).unwrap();
        let blocked = device.add_pip(blocked_h, endpoint_b, false, 1).unwrap();
        device.add_pip(active_v, endpoint_c, false, 1).unwrap();
        let mut constraints = RoutingConstraints::new();
        constraints.block_pips([blocked]);

        let map = routing_capacity_map(&device, &constraints);
        assert_eq!(
            map.capacity(Point::new(0, 0), RoutingChannelOrientation::Horizontal),
            Some(2)
        );
        assert_eq!(
            map.capacity(Point::new(1, 0), RoutingChannelOrientation::Horizontal),
            Some(0)
        );
        assert_eq!(
            map.capacity(Point::new(2, 1), RoutingChannelOrientation::Vertical),
            Some(4)
        );
        assert_eq!(map.total_capacity(RoutingChannelOrientation::Horizontal), 2);
        assert_eq!(map.total_capacity(RoutingChannelOrientation::Vertical), 4);
    }

    #[test]
    fn directional_demand_conserves_bbox_hpwl() {
        let (design, device, placement) =
            placed_net(ResourceKind::Logic, Point::new(0, 0), Point::new(3, 2));
        let map = routing_demand_map(&design, &device, &placement, &RoutingConstraints::new());

        assert_eq!(map.included_nets(), 1);
        assert_close(map.total_demand(RoutingChannelOrientation::Horizontal), 3.0);
        assert_close(map.total_demand(RoutingChannelOrientation::Vertical), 2.0);
        assert_close(
            map.demand(Point::new(1, 1), RoutingChannelOrientation::Horizontal)
                .unwrap(),
            0.25,
        );
        assert_close(
            map.demand(Point::new(1, 1), RoutingChannelOrientation::Vertical)
                .unwrap(),
            1.0 / 6.0,
        );
    }

    #[test]
    fn clock_driven_nets_are_excluded() {
        let (design, device, placement) =
            placed_net(ResourceKind::Clock, Point::new(0, 0), Point::new(3, 2));
        let map = routing_demand_map(&design, &device, &placement, &RoutingConstraints::new());

        assert_eq!(map.included_nets(), 0);
        assert_eq!(map.excluded_clock_nets(), 1);
        assert_close(map.total_demand(RoutingChannelOrientation::Horizontal), 0.0);
        assert_close(map.total_demand(RoutingChannelOrientation::Vertical), 0.0);
    }

    #[test]
    fn bin_aggregation_conserves_capacity_and_demand() {
        let (design, mut device, placement) =
            placed_net(ResourceKind::Logic, Point::new(0, 0), Point::new(3, 2));
        let horizontal = device
            .add_wire("R1C1/H02E0001", Point::new(1, 1), 2)
            .unwrap();
        let vertical = device
            .add_wire("R2C3/V02N0001", Point::new(3, 2), 3)
            .unwrap();
        let endpoint_h = device.add_wire("R1C1/JH", Point::new(1, 1), 1).unwrap();
        let endpoint_v = device.add_wire("R2C3/JV", Point::new(3, 2), 1).unwrap();
        device.add_pip(horizontal, endpoint_h, false, 1).unwrap();
        device.add_pip(vertical, endpoint_v, false, 1).unwrap();
        let capacity = routing_capacity_map(&device, &RoutingConstraints::new());
        let map = routing_demand_map_with_capacity(&design, &device, &placement, capacity);
        let bins = map.bins(2);

        assert_eq!(bins.len(), 4);
        assert_eq!(
            bins.iter().map(|bin| bin.horizontal_capacity).sum::<u64>(),
            2
        );
        assert_eq!(bins.iter().map(|bin| bin.vertical_capacity).sum::<u64>(), 3);
        assert_close(bins.iter().map(|bin| bin.horizontal_demand).sum(), 3.0);
        assert_close(bins.iter().map(|bin| bin.vertical_demand).sum(), 2.0);
        let lower_right = bins.iter().find(|bin| bin.x == 2 && bin.y == 2).unwrap();
        assert_eq!(lower_right.vertical_capacity, 3);
    }
}
