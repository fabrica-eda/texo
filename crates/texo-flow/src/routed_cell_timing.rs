//! Select cell input delays after physical LUT input permutation is known.
use super::{
    BTreeMap, Design, Ecp5Architecture, Ecp5FlowError, PipId, PnrError, PnrResult, ResourceKind,
    SpeedGradeRecord, TimingModel, find_cell_pin, timing_delay,
};
use std::borrow::Cow;

pub(super) fn has_input_asymmetry(grade: &SpeedGradeRecord) -> bool {
    grade.cells.iter().any(|cell| {
        let mut outputs = BTreeMap::new();
        cell.arcs
            .iter()
            .filter(|a| matches!(a.from_pin.as_str(), "A" | "B" | "C" | "D"))
            .any(|a| {
                outputs
                    .insert(&a.to_pin, a.delay)
                    .is_some_and(|old| old != a.delay)
            })
    })
}

pub(super) fn resolve<'a>(
    design: &Design,
    architecture: &Ecp5Architecture,
    grade: &SpeedGradeRecord,
    implementation: &PnrResult,
    model: &'a TimingModel,
) -> Result<Cow<'a, TimingModel>, Ecp5FlowError> {
    if !has_input_asymmetry(grade) {
        return Ok(Cow::Borrowed(model));
    }
    let records = grade
        .cells
        .iter()
        .map(|c| (c.cell_type.as_str(), c))
        .collect::<BTreeMap<_, _>>();
    let mut result = Cow::Borrowed(model);
    for route in &implementation.routes {
        for arc in &route.arcs {
            let Some(sink) = arc.sink else {
                continue;
            };
            let pin = &design.pins()[sink.0];
            if design.cells()[pin.cell.0].kind != ResourceKind::Lut(4)
                || !matches!(pin.name.as_str(), "A" | "B" | "C" | "D")
            {
                continue;
            }
            let bel = implementation
                .placement
                .bel(pin.cell)
                .expect("routed cell is placed");
            let bound = implementation
                .placement
                .pin_binding(sink)
                .or_else(|| {
                    architecture.device().bels()[bel.0]
                        .pins()
                        .iter()
                        .copied()
                        .find(|id| architecture.device().bel_pins()[id.0].name == pin.name)
                })
                .ok_or_else(|| PnrError::InvalidPlacement {
                    reason: format!(
                        "missing physical LUT input {}.{}",
                        design.cells()[pin.cell.0].name,
                        pin.name
                    ),
                })?;
            let physical = &architecture.device().bel_pins()[bound.0];
            let mut input = physical.name.as_str();
            for &pip in &arc.pips {
                if architecture.device().pips()[pip.0].to() != physical.wire {
                    continue;
                }
                let flags = architecture.pip_metadata(pip).lutperm_flags;
                if flags & 0x4000 != 0 {
                    // D0 -> B0_SLICE (0x4007) routes logical B through
                    // physical D. The bit generator permutes INIT accordingly.
                    input = ["A", "B", "C", "D"][usize::from(flags & 3)];
                }
            }
            if input == pin.name {
                continue;
            }
            let carry = find_cell_pin(design, pin.cell, "FCO").is_some();
            let ordinary = if carry {
                if architecture.device().bels()[bel.0].name.ends_with(".K0") {
                    "TRELLIS_CARRY0"
                } else {
                    "TRELLIS_CARRY1"
                }
            } else {
                "TRELLIS_COMB"
            };
            for output in ["F", "FCO", "OFX"] {
                let Some(to) = find_cell_pin(design, pin.cell, output) else {
                    continue;
                };
                if model.cell_arc(sink, to).is_none() {
                    continue;
                }
                let kind = if output == "OFX" {
                    "TRELLIS_PFUMX"
                } else {
                    ordinary
                };
                let record = records
                    .get(kind)
                    .ok_or_else(|| Ecp5FlowError::MissingCellTiming {
                        speed_grade: grade.name.clone(),
                        cell_type: kind.into(),
                    })?;
                let measured = record
                    .arcs
                    .iter()
                    .find(|a| a.from_pin == input && a.to_pin == output)
                    .ok_or_else(|| Ecp5FlowError::MissingCellTiming {
                        speed_grade: grade.name.clone(),
                        cell_type: format!("{kind}:{input}->{output}"),
                    })?;
                let changed =
                    result
                        .to_mut()
                        .update_cell_arc_delay(sink, to, timing_delay(measured.delay)?);
                debug_assert!(changed);
            }
        }
    }
    Ok(result)
}

/// Relative cell-input cost for route ranking, never a net-delay STA label.
pub(super) struct InputCosts([u32; 4]);
impl InputCosts {
    pub(super) fn new(grade: &SpeedGradeRecord) -> Result<Self, Ecp5FlowError> {
        let mut values = [0; 4];
        if let Some(record) = grade.cells.iter().find(|c| c.cell_type == "TRELLIS_COMB") {
            for (i, pin) in ["A", "B", "C", "D"].iter().enumerate() {
                if let Some(arc) = record
                    .arcs
                    .iter()
                    .find(|a| a.from_pin == *pin && a.to_pin == "F")
                {
                    values[i] = u32::try_from(arc.delay.max_ps)
                        .map_err(|_| Ecp5FlowError::TimingDelayOverflow)?;
                }
            }
        }
        let base = *values.iter().min().expect("four inputs");
        Ok(Self(values.map(|v| v - base)))
    }

    pub(super) fn pip_penalty(&self, architecture: &Ecp5Architecture, pip: PipId) -> u32 {
        if self.0 == [0; 4] {
            return 0;
        }
        let wire = &architecture.device().wires()[architecture.device().pips()[pip.0].to().0];
        let Some(stem) = wire
            .name
            .rsplit('/')
            .next()
            .and_then(|n| n.strip_suffix("_SLICE"))
        else {
            return 0;
        };
        let bytes = stem.as_bytes();
        if bytes.len() != 2
            || !(b'A'..=b'D').contains(&bytes[0])
            || !(b'0'..=b'7').contains(&bytes[1])
        {
            return 0;
        }
        let flags = architecture.pip_metadata(pip).lutperm_flags;
        let input = if flags & 0x4000 == 0 {
            usize::from(bytes[0] - b'A')
        } else {
            usize::from(flags & 3)
        };
        self.0[input]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Arc, BelId, DelayRange, Ecp5EcoTimingSession, NetId, NetRoute, PinDirection,
        PlacementConstraints, TimingConstraints, placement_from_complete_bindings,
    };
    use texo_pnr::RouteArc;
    use texo_target_ecp5::{DelayRangeRecord, read_architecture};

    #[test]
    fn routed_d_to_b_uses_physical_delay_and_does_not_mutate_the_logical_model() {
        let mut fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../../texo-target-ecp5/fixtures/minimal-ecp5.json"
        ))
        .unwrap();
        fixture["location_types"][0]["pips"]
            .as_array_mut()
            .unwrap()
            .push(serde_json::json!({
                "from":{"dx":0,"dy":0,"index":3},"to":{"dx":0,"dy":0,"index":1},
                "fixed":false,"tile_type":"PLC2","timing_class":"default","lutperm_flags":0x4007
            }));
        let arch = read_architecture(serde_json::to_vec(&fixture).unwrap().as_slice()).unwrap();
        let mut grade = arch.speed_grades()["6"].clone();
        let lut = grade
            .cells
            .iter_mut()
            .find(|c| c.cell_type == "TRELLIS_COMB")
            .unwrap();
        for arc in &mut lut.arcs {
            arc.delay = if arc.from_pin == "D" {
                DelayRangeRecord {
                    min_ps: 20,
                    max_ps: 40,
                }
            } else {
                DelayRangeRecord {
                    min_ps: 100,
                    max_ps: 200,
                }
            };
        }
        let mut design = Design::new();
        let cell = design.add_cell("lut", ResourceKind::Lut(4));
        let input = design.add_pin(cell, "B", PinDirection::Input).unwrap();
        let output = design.add_pin(cell, "F", PinDirection::Output).unwrap();
        let bel = arch
            .device()
            .bels()
            .iter()
            .position(|b| b.name == "R0C0/SLICEA.K0")
            .map(BelId)
            .unwrap();
        let placement = placement_from_complete_bindings(
            &design,
            arch.device(),
            &PlacementConstraints::default(),
            vec![bel],
        )
        .unwrap();
        let bound = arch.device().bels()[bel.0]
            .pins()
            .iter()
            .copied()
            .find(|id| arch.device().bel_pins()[id.0].name == "B")
            .unwrap();
        let wire = arch.device().bel_pins()[bound.0].wire;
        let pip = arch
            .device()
            .pips()
            .iter()
            .enumerate()
            .find(|(i, p)| p.to() == wire && arch.pip_metadata(PipId(*i)).lutperm_flags == 0x4007)
            .map(|(i, _)| PipId(i))
            .unwrap();
        // This synthetic unit fixture checks cell remapping only, not routing.
        let mut implementation = PnrResult {
            placement,
            routes: vec![Arc::new(NetRoute::new(
                NetId(0),
                vec![RouteArc {
                    sink: Some(input),
                    wires: vec![arch.device().pips()[pip.0].from(), wire],
                    pips: vec![pip],
                }],
            ))],
            total_pips: 1,
        };
        let mut model = TimingModel::new();
        model
            .add_cell_arc(input, output, DelayRange::new(100, 200).unwrap())
            .unwrap();
        let physical = resolve(&design, &arch, &grade, &implementation, &model).unwrap();
        assert_eq!(
            physical.cell_arc(input, output),
            Some(DelayRange::new(20, 40).unwrap())
        );
        assert_eq!(
            model.cell_arc(input, output),
            Some(DelayRange::new(100, 200).unwrap())
        );
        assert!(!model.update_cell_arc_delay(output, input, DelayRange::new(1, 2).unwrap()));
        implementation.routes.clear();
        let rerouted = resolve(&design, &arch, &grade, &implementation, &model).unwrap();
        assert_eq!(
            rerouted.cell_arc(input, output),
            Some(DelayRange::new(100, 200).unwrap())
        );
        let constraints = TimingConstraints::new();
        let session =
            Ecp5EcoTimingSession::new(&design, &arch, &grade, &model, &constraints).unwrap();
        assert!(session.routed_model_inputs.is_some());
        let costs = InputCosts::new(&grade).unwrap();
        assert_eq!(costs.pip_penalty(&arch, pip), 0); // Physical D is the fastest input.
        let direct = arch
            .device()
            .pips()
            .iter()
            .enumerate()
            .find(|(i, p)| p.to() == wire && arch.pip_metadata(PipId(*i)).lutperm_flags == 0)
            .map(|(i, _)| PipId(i))
            .unwrap();
        assert_eq!(costs.pip_penalty(&arch, direct), 160); // Direct B must pay B-D.
    }
}
