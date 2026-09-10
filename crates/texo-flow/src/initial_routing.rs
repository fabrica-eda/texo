//! Import physical trees as checked initial routing constraints. Neither timing
//! nor evidence from the input checkpoint is trusted; the normal router and STA
//! verify the fresh design, placement, occupancy and constraints.

use std::collections::BTreeSet;

use serde::Deserialize;
use texo_model::{CellPinId, Design, Device, NetId, PipId, WireId};
use texo_pnr::{NetRoute, Placement, PnrError, RoutingConstraints};

/// A named physical tree from a schema-v3 checkpoint.
#[derive(Clone, Debug, Deserialize)]
pub struct Ecp5InitialRoute {
    net: String,
    driver_wire: String,
    driver_wire_id: usize,
    pips: Vec<InitialPip>,
}

#[derive(Clone, Debug, Deserialize)]
struct InitialPip {
    pip_id: usize,
    from: String,
    to: String,
    from_wire_id: usize,
    to_wire_id: usize,
    bidirectional: bool,
}

fn pin_wire(
    design: &Design,
    device: &Device,
    placement: &Placement,
    pin: CellPinId,
) -> Option<WireId> {
    let logical = &design.pins()[pin.0];
    let bel = placement.bel(logical.cell)?;
    let physical = placement.pin_binding(pin).or_else(|| {
        device.bels()[bel.0].pins().iter().copied().find(|id| {
            let p = &device.bel_pins()[id.0];
            p.name == logical.name && p.direction == logical.direction
        })
    })?;
    Some(device.bel_pins()[physical.0].wire)
}

pub(super) fn import_routes(
    design: &Design,
    device: &Device,
    placement: &Placement,
    records: &[Ecp5InitialRoute],
    routing: &mut RoutingConstraints,
) -> Result<(), PnrError> {
    let names = design
        .nets()
        .iter()
        .enumerate()
        .map(|(id, net)| (net.name.as_str(), NetId(id)))
        .collect::<std::collections::BTreeMap<_, _>>();
    let mut seen = BTreeSet::new();
    for record in records {
        let error = |reason: String| PnrError::InvalidRoutingRestriction { reason };
        let net_id = *names
            .get(record.net.as_str())
            .ok_or_else(|| error(format!("initial route has unknown net {}", record.net)))?;
        let invalid = |reason: String| PnrError::InvalidRoutingConstraint {
            net: net_id,
            reason,
        };
        if !seen.insert(net_id) {
            return Err(invalid("duplicate initial route".into()));
        }
        let net = &design.nets()[net_id.0];
        let driver = pin_wire(design, device, placement, net.driver)
            .ok_or_else(|| invalid("initial driver has no physical pin".into()))?;
        if driver.0 != record.driver_wire_id || device.wires()[driver.0].name != record.driver_wire
        {
            return Err(invalid(
                "initial route driver differs from the placed design".into(),
            ));
        }
        let mut pips = BTreeSet::new();
        for saved in &record.pips {
            let id = PipId(saved.pip_id);
            let pip = device
                .pips()
                .get(id.0)
                .ok_or_else(|| invalid(format!("unknown initial PIP {}", id.0)))?;
            if pip.from().0 != saved.from_wire_id
                || pip.to().0 != saved.to_wire_id
                || pip.bidirectional() != saved.bidirectional
                || device.wires()[pip.from().0].name != saved.from
                || device.wires()[pip.to().0].name != saved.to
                || !pips.insert(id)
            {
                return Err(invalid(format!(
                    "initial PIP {} differs from the architecture or is duplicated",
                    id.0
                )));
            }
        }
        let sinks = net
            .sinks
            .iter()
            .map(|&pin| {
                pin_wire(design, device, placement, pin)
                    .map(|wire| (pin, wire))
                    .ok_or_else(|| invalid(format!("initial sink {} has no physical pin", pin.0)))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let route = NetRoute::from_tree(net_id, driver, sinks, pips, device).map_err(invalid)?;
        if let Some(required) = routing.routes().get(&net_id) {
            let imported_pips = route.pips().collect::<BTreeSet<_>>();
            if required.pips().any(|pip| !imported_pips.contains(&pip)) {
                return Err(invalid(
                    "initial route omits mandatory target routing".into(),
                ));
            }
        }
        routing.add_route(route);
    }
    Ok(())
}

pub(super) fn preserve_routes(
    design: &Design,
    names: &[String],
    initial: &RoutingConstraints,
    immutable: &mut RoutingConstraints,
) -> Result<(), PnrError> {
    if names.is_empty() {
        return Ok(());
    }
    let by_name = initial
        .routes()
        .iter()
        .map(|(net, route)| (design.nets()[net.0].name.as_str(), route))
        .collect::<std::collections::BTreeMap<_, _>>();
    let mut seen = BTreeSet::new();
    let selected = names
        .iter()
        .map(|name| {
            let invalid = |reason| PnrError::InvalidRoutingRestriction { reason };
            if !seen.insert(name) {
                return Err(invalid(format!("duplicate preserved initial route {name}")));
            }
            by_name
                .get(name.as_str())
                .copied()
                .ok_or_else(|| invalid(format!("preserved route {name} has no initial tree")))
        })
        .collect::<Result<Vec<_>, _>>()?;
    for route in selected {
        immutable.add_route(std::sync::Arc::clone(route));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;
    use texo_model::{PinDirection, Point, ResourceKind};
    use texo_pnr::{PlacementConstraints, placement_from_partial_bindings, route_with_placement};

    fn fixture(extra_sink: bool) -> (Design, Device, Placement, Ecp5InitialRoute) {
        let mut device = Device::new("test", 3, 1).unwrap();
        let mut design = Design::new();
        let mut bindings = BTreeMap::new();
        let mut pins = Vec::new();
        for x in 0..if extra_sink { 3 } else { 2 } {
            let point = Point { x, y: 0 };
            let wire = device.add_wire(format!("w{x}"), point, 1).unwrap();
            let bel = device
                .add_bel(format!("b{x}"), ResourceKind::Logic, point)
                .unwrap();
            let direction = if x == 0 {
                PinDirection::Output
            } else {
                PinDirection::Input
            };
            device.add_bel_pin(bel, "P", direction, wire).unwrap();
            let cell = design.add_cell(format!("c{x}"), ResourceKind::Logic);
            pins.push(design.add_pin(cell, "P", direction).unwrap());
            bindings.insert(cell, bel);
        }
        device.add_pip(WireId(0), WireId(1), false, 1).unwrap();
        design
            .add_net("data", pins[0], pins[1..].iter().copied())
            .unwrap();
        let placement = placement_from_partial_bindings(
            &design,
            &device,
            &PlacementConstraints::new(),
            &bindings,
        )
        .unwrap();
        let record = Ecp5InitialRoute {
            net: "data".into(),
            driver_wire: "w0".into(),
            driver_wire_id: 0,
            pips: vec![InitialPip {
                pip_id: 0,
                from: "w0".into(),
                to: "w1".into(),
                from_wire_id: 0,
                to_wire_id: 1,
                bidirectional: false,
            }],
        };
        (design, device, placement, record)
    }

    #[test]
    fn preserved_names_select_imported_trees_and_reject_invalid_lists_atomically() {
        let (design, device, placement, record) = fixture(false);
        let mut initial = RoutingConstraints::new();
        import_routes(&design, &device, &placement, &[record], &mut initial).unwrap();
        let mut immutable = RoutingConstraints::new();
        for names in [
            vec!["data".into(), "missing".into()],
            vec!["data".into(), "data".into()],
        ] {
            assert!(preserve_routes(&design, &names, &initial, &mut immutable).is_err());
            assert!(immutable.routes().is_empty());
        }
        preserve_routes(&design, &[], &initial, &mut immutable).unwrap();
        assert!(immutable.routes().is_empty());
        preserve_routes(&design, &["data".into()], &initial, &mut immutable).unwrap();
        assert_eq!(immutable.routes(), initial.routes());
    }

    #[test]
    fn imports_a_tree_against_the_current_design_and_runs_normal_routing_checks() {
        let (design, device, placement, record) = fixture(false);
        let mut constraints = RoutingConstraints::new();
        import_routes(&design, &device, &placement, &[record], &mut constraints).unwrap();
        let result = route_with_placement(&design, &device, placement, &constraints).unwrap();
        assert_eq!(result.total_pips, 1);
        assert_eq!(result.routes.len(), 1);
    }

    #[test]
    fn rejects_stale_names_ids_directions_and_incomplete_trees() {
        let (design, device, placement, record) = fixture(false);
        let mut cases = Vec::new();
        let mut changed = record.clone();
        changed.net = "old_net".into();
        cases.push(changed);
        let mut changed = record.clone();
        changed.driver_wire_id = usize::MAX;
        cases.push(changed);
        let mut changed = record.clone();
        changed.driver_wire = "old_wire".into();
        cases.push(changed);
        let mut changed = record.clone();
        changed.pips[0].pip_id = usize::MAX;
        cases.push(changed);
        let mut changed = record.clone();
        changed.pips[0].from_wire_id = 1;
        cases.push(changed);
        let mut changed = record.clone();
        changed.pips[0].to = "old_sink".into();
        cases.push(changed);
        let mut changed = record.clone();
        changed.pips[0].bidirectional = true;
        cases.push(changed);
        let mut changed = record.clone();
        changed.pips.push(changed.pips[0].clone());
        cases.push(changed);
        let mut changed = record;
        changed.pips.clear();
        cases.push(changed);
        for changed in cases {
            assert!(
                import_routes(
                    &design,
                    &device,
                    &placement,
                    &[changed],
                    &mut RoutingConstraints::new()
                )
                .is_err()
            );
        }
    }

    #[test]
    fn rejects_duplicate_nets_and_a_new_unreached_sink() {
        let (design, device, placement, record) = fixture(false);
        assert!(
            import_routes(
                &design,
                &device,
                &placement,
                &[record.clone(), record],
                &mut RoutingConstraints::new()
            )
            .is_err()
        );
        let (design, device, placement, record) = fixture(true);
        assert!(
            import_routes(
                &design,
                &device,
                &placement,
                &[record],
                &mut RoutingConstraints::new()
            )
            .is_err()
        );
    }

    #[test]
    fn imported_routes_cannot_remove_mandatory_target_branches() {
        let (design, mut device, placement, record) = fixture(false);
        let leaf = device
            .add_wire("required_clock_branch", Point { x: 2, y: 0 }, 1)
            .unwrap();
        let branch = device.add_pip(WireId(0), leaf, false, 1).unwrap();
        let required = NetRoute::from_tree(
            NetId(0),
            WireId(0),
            [(CellPinId(1), WireId(1))],
            BTreeSet::from([PipId(0), branch]),
            &device,
        )
        .unwrap();
        let mut constraints = RoutingConstraints::new();
        constraints.add_route(required);
        assert!(import_routes(&design, &device, &placement, &[record], &mut constraints).is_err());
    }
}
