//! Import physical trees as checked initial routing constraints. Neither timing
//! nor evidence from the input checkpoint is trusted; the normal router and STA
//! verify the fresh design, placement, occupancy and constraints.

use std::collections::{BTreeMap, BTreeSet};

use serde::Deserialize;
use texo_model::{CellPinId, Design, Device, NetId, PipId, WireId};
use texo_pnr::{NetRoute, Placement, PnrError, RoutingConstraints, RoutingCosts};

/// A named physical tree from a schema-v3 checkpoint.
#[derive(Clone, Debug, Deserialize)]
pub struct Ecp5InitialRoute {
    /// Optional endpoints of an advisory subtree. Omitted design sinks must
    /// still be connected by ordinary routing before sign-off.
    #[serde(default)]
    advisory_sink_wire_ids: Option<Vec<usize>>,
    /// Search priorities only, keyed by current physical sink wire. These do
    /// not supply delays or timing evidence and are replaced after routed STA.
    #[serde(default)]
    advisory_sink_criticalities: BTreeMap<usize, u64>,
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

// Enforce the import contract again before emitting timing/bitgen evidence.
// An ECO must not silently replace a tree declared immutable by the caller.
pub(super) fn verify_preserved_routes(
    design: &Design,
    preserved: &[String],
    initial: &RoutingConstraints,
    implementation: &texo_pnr::PnrResult,
) -> Result<(), PnrError> {
    if preserved.is_empty() {
        return Ok(());
    }
    let names = design
        .nets()
        .iter()
        .enumerate()
        .map(|(id, net)| (net.name.as_str(), NetId(id)))
        .collect::<std::collections::BTreeMap<_, _>>();
    for name in preserved {
        let net = *names
            .get(name.as_str())
            .ok_or_else(|| PnrError::InvalidRoutingRestriction {
                reason: format!("unknown immutable net {name}"),
            })?;
        let expected = initial.routes().get(&net);
        let actual = implementation.routes.get(net.0);
        if expected.is_none() || actual != expected {
            return Err(PnrError::InvalidRoutingConstraint {
                net,
                reason: "timing feedback changed an immutable imported route".into(),
            });
        }
    }
    Ok(())
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
        let sinks = select_imported_sinks(
            record,
            net_id,
            sinks,
            routing.routes().contains_key(&net_id),
        )?;
        let mut route =
            NetRoute::from_tree(net_id, driver, sinks, pips, device).map_err(invalid)?;
        if record.advisory_sink_wire_ids.is_some() {
            // from_tree validates every supplied PIP before obsolete branches
            // are discarded. They must not survive as immutable topology arcs.
            route = NetRoute::new(
                net_id,
                route
                    .arcs
                    .into_iter()
                    .filter(|arc| arc.sink.is_some())
                    .collect(),
            );
        }
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
            let route = by_name
                .get(name.as_str())
                .copied()
                .ok_or_else(|| invalid(format!("preserved route {name} has no initial tree")))?;
            if design.nets()[route.net.0]
                .sinks
                .iter()
                .any(|&sink| route.arc(sink).is_none())
            {
                return Err(invalid(format!(
                    "preserved route {name} has incomplete endpoints"
                )));
            }
            Ok(route)
        })
        .collect::<Result<Vec<_>, _>>()?;
    for route in selected {
        immutable.add_route(std::sync::Arc::clone(route));
    }
    Ok(())
}

fn select_imported_sinks(
    record: &Ecp5InitialRoute,
    net: NetId,
    mut sinks: Vec<(CellPinId, WireId)>,
    mandatory: bool,
) -> Result<Vec<(CellPinId, WireId)>, PnrError> {
    let invalid = |reason| PnrError::InvalidRoutingConstraint { net, reason };
    if let Some(ids) = &record.advisory_sink_wire_ids {
        if mandatory {
            return Err(invalid(
                "partial endpoints require a non-mandatory advisory route".into(),
            ));
        }
        let requested = ids.iter().copied().collect::<BTreeSet<_>>();
        let available = sinks
            .iter()
            .map(|(_, wire)| wire.0)
            .collect::<BTreeSet<_>>();
        if requested.is_empty() || requested.len() != ids.len() || !requested.is_subset(&available)
        {
            return Err(invalid(
                "partial endpoints must name distinct current sink wires".into(),
            ));
        }
        sinks.retain(|(_, wire)| requested.contains(&wire.0));
    }
    Ok(sinks)
}

// Called only after import_routes has checked driver identity and topology.
// Merge by maximum so an advisory priority cannot weaken a freshly predicted
// criticality. Missing sinks and values outside the router's range are errors.
pub(super) fn apply_advisory_weights(
    design: &Design,
    device: &Device,
    placement: &Placement,
    records: &[Ecp5InitialRoute],
    costs: &mut RoutingCosts,
) -> Result<(), PnrError> {
    let names = design
        .nets()
        .iter()
        .enumerate()
        .map(|(id, net)| (net.name.as_str(), NetId(id)))
        .collect::<BTreeMap<_, _>>();
    let mut nets = costs.net_criticalities().clone();
    let mut sinks = costs.sink_criticalities().clone();
    for record in records {
        if record.advisory_sink_criticalities.is_empty() {
            continue;
        }
        let net =
            *names
                .get(record.net.as_str())
                .ok_or_else(|| PnrError::InvalidRoutingRestriction {
                    reason: "advisory weight names an unknown net".into(),
                })?;
        let invalid = || PnrError::InvalidRoutingConstraint {
            net,
            reason: "advisory criticalities require current sink wires and weights in 1..=64"
                .into(),
        };
        if record
            .advisory_sink_criticalities
            .values()
            .any(|weight| !(1..=64).contains(weight))
        {
            return Err(invalid());
        }
        let mut seen = BTreeSet::new();
        for &sink in &design.nets()[net.0].sinks {
            let wire = pin_wire(design, device, placement, sink).ok_or_else(invalid)?;
            if let Some(&weight) = record.advisory_sink_criticalities.get(&wire.0) {
                seen.insert(wire.0);
                let existing = costs
                    .sink_criticalities()
                    .get(&(net, sink))
                    .or_else(|| costs.net_criticalities().get(&net))
                    .copied()
                    .unwrap_or(1);
                let weight = weight.max(existing);
                sinks
                    .entry((net, sink))
                    .and_modify(|old| *old = (*old).max(weight))
                    .or_insert(weight);
                nets.entry(net)
                    .and_modify(|old| *old = (*old).max(weight))
                    .or_insert(weight);
            }
        }
        if seen.len() != record.advisory_sink_criticalities.len() {
            return Err(invalid());
        }
    }
    costs.set_net_criticalities(nets);
    costs.set_sink_criticalities(sinks);
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
            advisory_sink_wire_ids: None,
            advisory_sink_criticalities: BTreeMap::new(),
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

    fn route_seeded(
        design: &Design,
        device: &Device,
        placement: Placement,
        seeds: &RoutingConstraints,
    ) -> Result<texo_pnr::PnrResult, PnrError> {
        texo_pnr::route_with_initial_routes_workspace_and_progress(
            design,
            device,
            placement,
            &RoutingConstraints::new(),
            &seeds.routes().values().cloned().collect::<Vec<_>>(),
            None,
            &mut texo_pnr::RoutingWorkspace::new(device),
            |_| {},
        )
    }

    #[test]
    fn partial_advisory_tree_preserves_old_sink_and_routes_new_sink() {
        let (design, mut device, placement, mut record) = fixture(true);
        device.add_pip(WireId(1), WireId(2), false, 1).unwrap();
        record.advisory_sink_wire_ids = Some(vec![1]);
        let mut constraints = RoutingConstraints::new();
        import_routes(&design, &device, &placement, &[record], &mut constraints).unwrap();
        let old = constraints.routes()[&NetId(0)]
            .arc(CellPinId(1))
            .unwrap()
            .clone();
        assert!(constraints.routes()[&NetId(0)].arc(CellPinId(2)).is_none());
        let result = route_seeded(&design, &device, placement, &constraints).unwrap();
        assert_eq!(result.routes[0].arc(CellPinId(1)), Some(&old));
        assert!(result.routes[0].arc(CellPinId(2)).is_some());
        assert_eq!(result.total_pips, 2);
    }

    #[test]
    fn partial_advisory_endpoints_do_not_hide_invalid_or_unroutable_sinks() {
        let (design, device, placement, mut record) = fixture(true);
        for endpoints in [vec![], vec![1, 1], vec![99], vec![2]] {
            record.advisory_sink_wire_ids = Some(endpoints);
            assert!(
                import_routes(
                    &design,
                    &device,
                    &placement,
                    &[record.clone()],
                    &mut RoutingConstraints::new()
                )
                .is_err()
            );
        }
        record.advisory_sink_wire_ids = Some(vec![1]);
        let mut constraints = RoutingConstraints::new();
        import_routes(
            &design,
            &device,
            &placement,
            &[record.clone()],
            &mut constraints,
        )
        .unwrap();
        // The missing second sink has no physical path in this fixture.
        assert!(route_seeded(&design, &device, placement.clone(), &constraints).is_err());
        assert!(
            preserve_routes(
                &design,
                &["data".into()],
                &constraints,
                &mut RoutingConstraints::new()
            )
            .is_err()
        );
        let required = constraints.routes()[&NetId(0)].clone();
        let mut locked = RoutingConstraints::new();
        locked.add_route(required);
        assert!(import_routes(&design, &device, &placement, &[record], &mut locked).is_err());
    }

    #[test]
    fn partial_advisory_tree_discards_obsolete_topology_leaves() {
        let (design, mut device, placement, mut record) = fixture(false);
        let orphan = device
            .add_wire("old_sink", Point { x: 2, y: 0 }, 1)
            .unwrap();
        let extra = device.add_pip(WireId(0), orphan, false, 1).unwrap();
        record.pips.push(InitialPip {
            pip_id: extra.0,
            from: "w0".into(),
            to: "old_sink".into(),
            from_wire_id: 0,
            to_wire_id: orphan.0,
            bidirectional: false,
        });
        record.advisory_sink_wire_ids = Some(vec![1]);
        let mut constraints = RoutingConstraints::new();
        import_routes(&design, &device, &placement, &[record], &mut constraints).unwrap();
        let route = &constraints.routes()[&NetId(0)];
        assert_eq!(route.pips().collect::<Vec<_>>(), vec![texo_model::PipId(0)]);
        assert!(route.arcs.iter().all(|arc| arc.sink.is_some()));
        let result = route_seeded(&design, &device, placement, &constraints).unwrap();
        assert_eq!(result.total_pips, 1);
    }

    #[test]
    fn advisory_weights_only_raise_priorities_and_reject_stale_inputs() {
        let (design, device, placement, mut record) = fixture(false);
        record.advisory_sink_criticalities.insert(1, 17);
        let mut costs = texo_pnr::RoutingCosts::new(vec![23], BTreeMap::from([(NetId(0), 20)]));
        super::apply_advisory_weights(&design, &device, &placement, &[record.clone()], &mut costs)
            .unwrap();
        assert_eq!(costs.net_criticalities()[&NetId(0)], 20);
        assert_eq!(costs.sink_criticalities()[&(NetId(0), CellPinId(1))], 20);
        let before = costs.sink_criticalities().clone();
        for (wire, weight) in [(1, 0), (1, 65), (99, 17)] {
            record.advisory_sink_criticalities = BTreeMap::from([(wire, weight)]);
            assert!(
                super::apply_advisory_weights(
                    &design,
                    &device,
                    &placement,
                    &[record.clone()],
                    &mut costs
                )
                .is_err()
            );
            assert_eq!(costs.sink_criticalities(), &before);
        }
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
    fn final_check_rejects_a_legal_replacement_of_an_immutable_import() {
        let (design, mut device, placement, record) = fixture(false);
        let middle = device
            .add_wire("alternate", Point { x: 1, y: 0 }, 1)
            .unwrap();
        let first = device.add_pip(WireId(0), middle, false, 1).unwrap();
        let second = device.add_pip(middle, WireId(1), false, 1).unwrap();
        let mut constraints = RoutingConstraints::new();
        import_routes(
            &design,
            &device,
            &placement,
            std::slice::from_ref(&record),
            &mut constraints,
        )
        .unwrap();
        let mut result = route_with_placement(&design, &device, placement, &constraints).unwrap();
        verify_preserved_routes(&design, &["data".into()], &constraints, &result).unwrap();
        let alternate = NetRoute::from_tree(
            NetId(0),
            WireId(0),
            [(CellPinId(1), WireId(1))],
            [first, second],
            &device,
        )
        .unwrap();
        result.routes[0] = std::sync::Arc::new(alternate);
        result.total_pips = 2;
        assert!(verify_preserved_routes(&design, &["data".into()], &constraints, &result).is_err());
        verify_preserved_routes(&design, &[], &constraints, &result).unwrap();
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
