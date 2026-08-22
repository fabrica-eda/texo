//! Deterministic reference placement and routing.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::error::Error;
use std::fmt;

use texo_model::{CellId, Design, Device, NetId, Point};

/// Placement indexed by stable cell ID.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Placement {
    locations: Vec<Point>,
}

impl Placement {
    /// Location of a cell, if the ID exists.
    #[must_use]
    pub fn location(&self, cell: CellId) -> Option<Point> {
        self.locations.get(cell.0).copied()
    }

    /// Cell locations in stable ID order.
    #[must_use]
    pub fn locations(&self) -> &[Point] {
        &self.locations
    }
}

/// Routed tree for one logical net.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NetRoute {
    /// Logical net represented by this tree.
    pub net: NetId,
    /// Occupied grid points in deterministic discovery order.
    pub points: Vec<Point>,
}

/// Complete result of the reference `PnR` engine.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PnrResult {
    /// Legal cell placement.
    pub placement: Placement,
    /// One route tree per logical net.
    pub routes: Vec<NetRoute>,
    /// Sum of Manhattan edges across routed trees.
    pub total_wire_length: u64,
}

/// Places and routes a design on an abstract grid.
///
/// The placer orders cells by connectivity and chooses the compatible free site
/// with the lowest distance to already placed neighbors. The router grows a
/// rectilinear tree with breadth-first searches while preventing inter-net
/// resource conflicts.
///
/// # Errors
///
/// Returns a descriptive legality or routability error.
pub fn place_and_route(design: &Design, device: &Device) -> Result<PnrResult, PnrError> {
    let placement = place(design, device)?;
    let routes = route(design, device, &placement)?;
    let total_wire_length = routes
        .iter()
        .map(|route| route.points.len().saturating_sub(1) as u64)
        .sum();
    Ok(PnrResult {
        placement,
        routes,
        total_wire_length,
    })
}

fn place(design: &Design, device: &Device) -> Result<Placement, PnrError> {
    let mut degree = vec![0_usize; design.cells().len()];
    let mut neighbors = vec![Vec::new(); design.cells().len()];
    for net in design.nets() {
        for &cell in &net.terminals {
            degree[cell.0] += net.terminals.len() - 1;
            neighbors[cell.0].extend(net.terminals.iter().copied().filter(|other| *other != cell));
        }
    }

    let mut order: Vec<_> = (0..design.cells().len()).map(CellId).collect();
    order.sort_by_key(|id| (std::cmp::Reverse(degree[id.0]), *id));

    let mut placed = vec![None; design.cells().len()];
    let mut occupied = BTreeSet::new();
    for cell_id in order {
        let cell = &design.cells()[cell_id.0];
        let choice = device
            .sites()
            .iter()
            .filter(|site| site.kind == cell.kind && !occupied.contains(&site.point))
            .map(|site| {
                let cost: u64 = neighbors[cell_id.0]
                    .iter()
                    .filter_map(|neighbor| placed[neighbor.0])
                    .map(|point: Point| point.manhattan(site.point))
                    .sum();
                (cost, site.point)
            })
            .min();
        let (_, point) = choice.ok_or_else(|| PnrError::NoSite {
            cell: cell.name.clone(),
        })?;
        occupied.insert(point);
        placed[cell_id.0] = Some(point);
    }

    Ok(Placement {
        locations: placed
            .into_iter()
            .map(|point| point.expect("every ordered cell was placed"))
            .collect(),
    })
}

fn route(
    design: &Design,
    device: &Device,
    placement: &Placement,
) -> Result<Vec<NetRoute>, PnrError> {
    let cell_points: BTreeSet<_> = placement.locations().iter().copied().collect();
    let mut globally_used = BTreeSet::new();
    let mut routes = Vec::with_capacity(design.nets().len());

    for (index, net) in design.nets().iter().enumerate() {
        let net_id = NetId(index);
        let terminals: Vec<_> = net
            .terminals
            .iter()
            .map(|cell| {
                placement
                    .location(*cell)
                    .ok_or(PnrError::MissingPlacement { cell: *cell })
            })
            .collect::<Result<_, _>>()?;
        let terminal_set: BTreeSet<_> = terminals.iter().copied().collect();
        let mut tree = BTreeSet::from([terminals[0]]);
        let mut ordered_points = vec![terminals[0]];

        for &sink in &terminals[1..] {
            if tree.contains(&sink) {
                continue;
            }
            let path = shortest_path(sink, &tree, device, |point| {
                terminal_set.contains(&point)
                    || (!cell_points.contains(&point) && !globally_used.contains(&point))
                    || tree.contains(&point)
            })
            .ok_or_else(|| PnrError::Unroutable {
                net: net.name.clone(),
            })?;
            for point in path {
                if tree.insert(point) {
                    ordered_points.push(point);
                }
            }
        }

        globally_used.extend(
            tree.iter()
                .copied()
                .filter(|point| !terminal_set.contains(point)),
        );
        routes.push(NetRoute {
            net: net_id,
            points: ordered_points,
        });
    }
    Ok(routes)
}

fn shortest_path(
    start: Point,
    goals: &BTreeSet<Point>,
    device: &Device,
    passable: impl Fn(Point) -> bool,
) -> Option<Vec<Point>> {
    let mut queue = VecDeque::from([start]);
    let mut predecessor = BTreeMap::new();
    predecessor.insert(start, None);

    let reached = loop {
        let point = queue.pop_front()?;
        if goals.contains(&point) {
            break point;
        }
        for neighbor in neighbors(point, device) {
            if passable(neighbor) && !predecessor.contains_key(&neighbor) {
                predecessor.insert(neighbor, Some(point));
                queue.push_back(neighbor);
            }
        }
    };

    let mut path = Vec::new();
    let mut cursor = reached;
    path.push(cursor);
    while let Some(Some(previous)) = predecessor.get(&cursor) {
        cursor = *previous;
        path.push(cursor);
    }
    path.reverse();
    Some(path)
}

fn neighbors(point: Point, device: &Device) -> impl Iterator<Item = Point> {
    let candidates = [
        point.x.checked_sub(1).map(|x| Point::new(x, point.y)),
        point.y.checked_sub(1).map(|y| Point::new(point.x, y)),
        point.x.checked_add(1).map(|x| Point::new(x, point.y)),
        point.y.checked_add(1).map(|y| Point::new(point.x, y)),
    ];
    candidates
        .into_iter()
        .flatten()
        .filter(|candidate| device.contains(*candidate))
}

/// `PnR` failure with the responsible object identified.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PnrError {
    /// No compatible unoccupied site exists.
    NoSite {
        /// Cell that could not be placed.
        cell: String,
    },
    /// Internal or externally supplied placement omitted a cell.
    MissingPlacement {
        /// Missing cell ID.
        cell: CellId,
    },
    /// No conflict-free route exists in the current grid model.
    Unroutable {
        /// Net that could not be connected.
        net: String,
    },
}

impl fmt::Display for PnrError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoSite { cell } => write!(f, "no compatible free site for cell `{cell}`"),
            Self::MissingPlacement { cell } => {
                write!(f, "placement is missing cell ID {}", cell.0)
            }
            Self::Unroutable { net } => write!(f, "net `{net}` is unroutable"),
        }
    }
}

impl Error for PnrError {}

#[cfg(test)]
mod tests {
    use texo_model::{Design, Device, ResourceKind};

    use super::{PnrError, place_and_route};

    #[test]
    fn places_connected_cells_close_and_routes_them() {
        let mut design = Design::new();
        let a = design.add_cell("a", ResourceKind::Logic);
        let b = design.add_cell("b", ResourceKind::Logic);
        design.add_net("a_to_b", [a, b]).unwrap();
        let device = Device::rectangular_logic(4, 4).unwrap();

        let result = place_and_route(&design, &device).unwrap();

        assert_eq!(
            result
                .placement
                .location(a)
                .unwrap()
                .manhattan(result.placement.location(b).unwrap()),
            1
        );
        assert_eq!(result.routes.len(), 1);
        assert_eq!(result.total_wire_length, 1);
    }

    #[test]
    fn reports_site_exhaustion() {
        let mut design = Design::new();
        design.add_cell("a", ResourceKind::Logic);
        design.add_cell("b", ResourceKind::Logic);
        let device = Device::rectangular_logic(1, 1).unwrap();

        assert_eq!(
            place_and_route(&design, &device),
            Err(PnrError::NoSite { cell: "b".into() })
        );
    }
}
