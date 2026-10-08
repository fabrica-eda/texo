//! Routed STA for successive setup ECO candidates.

use super::{
    Arc, BTreeMap, CellPinId, DelayRange, Design, Ecp5Architecture, Ecp5FlowError, HashMap,
    NetDelay, NetRoute, PipClassTimingRecord, PipId, Placement, PnrResult, SpeedGradeRecord,
    TimingAnalysisSession, TimingConstraints, TimingError, TimingModel, TimingReport, WireId,
    pip_class_delay, routed_cell_timing,
};
use texo_target_ecp5::CellTimingRecord;
use texo_timing::routed_sink_delays;

type InputDelay = (CellPinId, CellPinId, DelayRange);

/// Routed STA for ECO candidates that each change a few nets of one placed
/// design.
///
/// A net is re-timed only when its route tree changed or one of its PIPs
/// leaves a wire whose selected-PIP fanout changed. Every other net keeps
/// its previous sink delays and physical LUT-input cell delays, which depend
/// on nothing else. The report equals a complete
/// `analyze_ecp5_implementation` of the same implementation.
pub(crate) struct Ecp5EcoTimingSession<'a> {
    pub(crate) routed_model_inputs: Option<(&'a Design, &'a TimingModel)>,
    design: &'a Design,
    model: &'a TimingModel,
    timing: TimingAnalysisSession<'a>,
    architecture: &'a Ecp5Architecture,
    speed_grade: &'a SpeedGradeRecord,
    cell_records: BTreeMap<&'a str, &'a CellTimingRecord>,
    /// Speed-grade class by compact timing-class ID, resolved on first use.
    pip_classes: Vec<Option<&'a PipClassTimingRecord>>,
    /// Number of route trees using each PIP, exact while `exact_uses`.
    pip_uses: Vec<u8>,
    touched_pips: Vec<PipId>,
    exact_uses: bool,
    /// Distinct selected PIPs leaving each wire.
    source_fanout: Vec<u64>,
    touched_sources: Vec<WireId>,
    fanout_changed: Vec<bool>,
    changed_sources: Vec<WireId>,
    previous: Option<RoutedTiming>,
}

/// Per-net inputs of the last complete analysis.
struct RoutedTiming {
    placement: Placement,
    routes: Vec<Arc<NetRoute>>,
    net_delays: Vec<NetDelay>,
    /// First `net_delays` index of each net, followed by the total.
    offsets: Vec<usize>,
    /// Distinct source wires of each net's PIPs.
    sources: Vec<Vec<WireId>>,
    input_delays: Vec<Vec<InputDelay>>,
}

impl<'a> Ecp5EcoTimingSession<'a> {
    pub(crate) fn new(
        design: &'a Design,
        architecture: &'a Ecp5Architecture,
        speed_grade: &'a SpeedGradeRecord,
        model: &'a TimingModel,
        constraints: &'a TimingConstraints,
    ) -> Result<Self, Ecp5FlowError> {
        let device = architecture.device();
        Ok(Self {
            routed_model_inputs: routed_cell_timing::has_input_asymmetry(speed_grade)
                .then_some((design, model)),
            design,
            model,
            timing: TimingAnalysisSession::new(design, model, constraints)?,
            architecture,
            speed_grade,
            cell_records: routed_cell_timing::cell_records(speed_grade),
            pip_classes: vec![None; architecture.metadata_string_count()],
            pip_uses: vec![0; device.pips().len()],
            touched_pips: Vec::new(),
            exact_uses: true,
            source_fanout: vec![0; device.wires().len()],
            touched_sources: Vec::new(),
            fanout_changed: vec![false; device.wires().len()],
            changed_sources: Vec::new(),
            previous: None,
        })
    }

    pub(crate) fn analyze(
        &mut self,
        implementation: &PnrResult,
    ) -> Result<TimingReport, Ecp5FlowError> {
        let result = self.analyze_reusing(implementation);
        if result.is_err() {
            // Counters may be half-updated. Recount from scratch next time.
            self.previous = None;
            self.exact_uses = false;
        }
        result
    }

    #[allow(clippy::too_many_lines)]
    fn analyze_reusing(
        &mut self,
        implementation: &PnrResult,
    ) -> Result<TimingReport, Ecp5FlowError> {
        let nets = self.design.nets().len();
        let indexed = implementation.routes.len() == nets
            && implementation
                .routes
                .iter()
                .enumerate()
                .all(|(index, route)| route.net.0 == index);
        if !indexed {
            self.previous = None;
            return self.analyze_complete(implementation);
        }
        let previous = self
            .previous
            .take()
            .filter(|previous| self.exact_uses && previous.placement == implementation.placement);
        let reused = previous.is_some();
        let mut state = previous.unwrap_or_else(|| RoutedTiming {
            placement: implementation.placement.clone(),
            routes: implementation.routes.clone(),
            net_delays: Vec::new(),
            offsets: Vec::new(),
            sources: vec![Vec::new(); nets],
            input_delays: vec![Vec::new(); nets],
        });
        let changed = if reused {
            (0..nets)
                .filter(|&index| !Arc::ptr_eq(&state.routes[index], &implementation.routes[index]))
                .collect::<Vec<_>>()
        } else {
            (0..nets).collect()
        };

        if self.routed_model_inputs.is_some() {
            for &index in &changed {
                let delays = &mut state.input_delays[index];
                delays.clear();
                routed_cell_timing::route_physical_input_delays(
                    self.design,
                    self.architecture,
                    self.speed_grade,
                    &self.cell_records,
                    &implementation.placement,
                    &implementation.routes[index],
                    self.model,
                    delays,
                )?;
            }
        }

        let mut dirty = vec![false; nets];
        if reused {
            // Add before removing so a PIP kept by a rebuilt tree never
            // passes through zero uses.
            for &index in &changed {
                for pip in implementation.routes[index].pips() {
                    self.add_pip(pip, true)?;
                }
            }
            for &index in &changed {
                for pip in state.routes[index].pips() {
                    self.remove_pip(pip);
                }
                dirty[index] = true;
            }
            for (index, sources) in state.sources.iter().enumerate() {
                if !dirty[index] && sources.iter().any(|wire| self.fanout_changed[wire.0]) {
                    dirty[index] = true;
                }
            }
        } else {
            self.recount(&implementation.routes)?;
            dirty.fill(true);
        }
        for wire in self.changed_sources.drain(..) {
            self.fanout_changed[wire.0] = false;
        }

        let mut net_delays = Vec::new();
        let mut offsets = Vec::with_capacity(nets + 1);
        let mut pip_delays = HashMap::new();
        for (index, route) in implementation.routes.iter().enumerate() {
            offsets.push(net_delays.len());
            if !dirty[index] {
                let start = state.offsets[index];
                net_delays.extend_from_slice(&state.net_delays[start..state.offsets[index + 1]]);
                continue;
            }
            pip_delays.clear();
            let mut sources = Vec::new();
            for pip in route.pips() {
                pip_delays.insert(pip, self.pip_delay(pip)?);
                sources.push(self.architecture.device().pips()[pip.0].from());
            }
            sources.sort_unstable();
            sources.dedup();
            state.sources[index] = sources;
            routed_sink_delays(
                self.design,
                self.architecture.device(),
                &implementation.placement,
                route,
                &mut |pip| pip_delays.get(&pip).copied(),
                &mut net_delays,
            )?;
        }
        offsets.push(net_delays.len());
        state.net_delays = net_delays;
        state.offsets = offsets;
        state.routes.clone_from(&implementation.routes);

        if self.routed_model_inputs.is_some() {
            let applied = self
                .timing
                .set_cell_arc_delays(state.input_delays.iter().flatten().copied());
            debug_assert!(applied, "physical input delays replace model arcs");
        }
        let report = self.timing.analyze(state.net_delays.clone())?;
        self.previous = Some(state);
        Ok(report)
    }

    /// Complete analysis without reuse, for route lists that are not one
    /// tree per net in net order. Errors match the routed STA checks.
    fn analyze_complete(
        &mut self,
        implementation: &PnrResult,
    ) -> Result<TimingReport, Ecp5FlowError> {
        if let Some((design, model)) = self.routed_model_inputs {
            let delays = routed_cell_timing::physical_input_delays(
                design,
                self.architecture,
                self.speed_grade,
                implementation,
                model,
            )?;
            let applied = self.timing.set_cell_arc_delays(delays);
            debug_assert!(applied, "physical input delays replace model arcs");
        }
        self.recount(&implementation.routes)?;
        let mut pip_delays = HashMap::new();
        for index in 0..self.touched_pips.len() {
            let pip = self.touched_pips[index];
            pip_delays.insert(pip, self.pip_delay(pip)?);
        }
        Ok(self
            .timing
            .analyze_routed(self.architecture.device(), implementation, |pip| {
                pip_delays.get(&pip).copied()
            })?)
    }

    /// Resets every counter and counts `routes` from scratch.
    fn recount(&mut self, routes: &[Arc<NetRoute>]) -> Result<(), Ecp5FlowError> {
        for pip in self.touched_pips.drain(..) {
            self.pip_uses[pip.0] = 0;
        }
        for wire in self.touched_sources.drain(..) {
            self.source_fanout[wire.0] = 0;
        }
        self.exact_uses = true;
        for route in routes {
            for pip in route.pips() {
                self.add_pip(pip, false)?;
            }
        }
        Ok(())
    }

    fn add_pip(&mut self, pip: PipId, track_fanout: bool) -> Result<(), Ecp5FlowError> {
        let Some(&uses) = self.pip_uses.get(pip.0) else {
            return Err(TimingError::UnknownRoutedPip(pip).into());
        };
        if uses == 0 {
            self.touched_pips.push(pip);
            let source = self.architecture.device().pips()[pip.0].from();
            if self.source_fanout[source.0] == 0 {
                self.touched_sources.push(source);
            }
            self.source_fanout[source.0] += 1;
            if track_fanout {
                self.mark_fanout_changed(source);
            }
        }
        match uses.checked_add(1) {
            Some(uses) => self.pip_uses[pip.0] = uses,
            None => self.exact_uses = false,
        }
        Ok(())
    }

    fn remove_pip(&mut self, pip: PipId) {
        self.pip_uses[pip.0] -= 1;
        if self.pip_uses[pip.0] == 0 {
            let source = self.architecture.device().pips()[pip.0].from();
            self.source_fanout[source.0] -= 1;
            self.mark_fanout_changed(source);
        }
    }

    fn mark_fanout_changed(&mut self, source: WireId) {
        if !self.fanout_changed[source.0] {
            self.fanout_changed[source.0] = true;
            self.changed_sources.push(source);
        }
    }

    fn pip_delay(&mut self, pip: PipId) -> Result<DelayRange, Ecp5FlowError> {
        let id = self.architecture.pip_timing_class_id(pip) as usize;
        let class = if let Some(class) = self.pip_classes[id] {
            class
        } else {
            let timing_class = self.architecture.pip_metadata(pip).timing_class;
            let class = self
                .speed_grade
                .pip_classes
                .get(timing_class)
                .ok_or_else(|| Ecp5FlowError::MissingPipTimingClass {
                    speed_grade: self.speed_grade.name.clone(),
                    timing_class: timing_class.to_owned(),
                })?;
            self.pip_classes[id] = Some(class);
            class
        };
        let source = self.architecture.device().pips()[pip.0].from();
        pip_class_delay(class, self.source_fanout[source.0])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        BelId, PinDirection, PlacementConstraints, ResourceKind, analyze_ecp5_implementation,
        placement_from_complete_bindings,
    };
    use texo_pnr::RouteArc;
    use texo_target_ecp5::{DelayRangeRecord, read_architecture};

    #[test]
    #[allow(clippy::too_many_lines)]
    fn reused_net_delays_match_complete_sta_across_route_changes() {
        let mut fixture: serde_json::Value =
            serde_json::from_str(include_str!("../fixtures/minimal-ecp5.json")).unwrap();
        // PADDI -> D0_SLICE, and D0_SLICE -> B0_SLICE through the LUT
        // permutation, so logical B can use physical D.
        let pips = fixture["location_types"][0]["pips"].as_array_mut().unwrap();
        for (from, to, flags) in [(6, 3, 0), (3, 1, 0x4007)] {
            pips.push(serde_json::json!({
                "from":{"dx":0,"dy":0,"index":from},"to":{"dx":0,"dy":0,"index":to},
                "fixed":false,"tile_type":"PLC2","timing_class":"default","lutperm_flags":flags
            }));
        }
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
        let device = arch.device();
        let bel = |name: &str| {
            BelId(
                device
                    .bels()
                    .iter()
                    .position(|b| b.name == format!("R0C0/{name}"))
                    .unwrap(),
            )
        };
        let wire = |name: &str| {
            texo_model::WireId(
                device
                    .wires()
                    .iter()
                    .position(|w| w.name == format!("R0C0/{name}"))
                    .unwrap(),
            )
        };
        let pip = |from: &str, to: &str| {
            let (from, to) = (wire(from), wire(to));
            PipId(
                device
                    .pips()
                    .iter()
                    .position(|p| p.from() == from && p.to() == to)
                    .unwrap(),
            )
        };

        let mut design = Design::new();
        let pad = design.add_cell("pad", ResourceKind::Io);
        let pad_o = design.add_pin(pad, "O", PinDirection::Output).unwrap();
        let lut = design.add_cell("lut", ResourceKind::Lut(4));
        let lut_a = design.add_pin(lut, "A", PinDirection::Input).unwrap();
        let lut_b = design.add_pin(lut, "B", PinDirection::Input).unwrap();
        let lut_f1 = design.add_pin(lut, "F1", PinDirection::Input).unwrap();
        let lut_f = design.add_pin(lut, "F", PinDirection::Output).unwrap();
        let feeder = design.add_cell("feeder", ResourceKind::Lut(4));
        let feeder_f = design.add_pin(feeder, "F", PinDirection::Output).unwrap();
        let pad_net = design.add_net("pad", pad_o, [lut_a, lut_b]).unwrap();
        let feeder_net = design.add_net("feeder", feeder_f, [lut_f1]).unwrap();
        let placement = placement_from_complete_bindings(
            &design,
            device,
            &PlacementConstraints::default(),
            vec![bel("PIOA"), bel("SLICEA.K0"), bel("SLICEA.K1")],
        )
        .unwrap();
        let mut model = TimingModel::new();
        for input in [lut_a, lut_b] {
            model
                .add_cell_arc(input, lut_f, DelayRange::new(100, 200).unwrap())
                .unwrap();
        }
        let constraints = TimingConstraints::new();

        let a_arc = RouteArc {
            sink: Some(lut_a),
            wires: vec![wire("PADDI_A"), wire("A0_SLICE")],
            pips: vec![pip("PADDI_A", "A0_SLICE")],
        };
        let direct = Arc::new(NetRoute::new(
            pad_net,
            vec![
                a_arc.clone(),
                RouteArc {
                    sink: Some(lut_b),
                    wires: vec![wire("PADDI_A"), wire("B0_SLICE")],
                    pips: vec![pip("PADDI_A", "B0_SLICE")],
                },
            ],
        ));
        let permuted = Arc::new(NetRoute::new(
            pad_net,
            vec![
                a_arc,
                RouteArc {
                    sink: Some(lut_b),
                    wires: vec![wire("PADDI_A"), wire("D0_SLICE"), wire("B0_SLICE")],
                    pips: vec![pip("PADDI_A", "D0_SLICE"), pip("D0_SLICE", "B0_SLICE")],
                },
            ],
        ));
        let dedicated = Arc::new(NetRoute::new(
            feeder_net,
            vec![RouteArc {
                sink: Some(lut_f1),
                wires: vec![wire("F1_SLICE")],
                pips: Vec::new(),
            }],
        ));
        let implementation = |pad_route: &Arc<NetRoute>| PnrResult {
            placement: placement.clone(),
            routes: vec![pad_route.clone(), dedicated.clone()],
            total_pips: pad_route.pips().len(),
        };

        let mut session =
            Ecp5EcoTimingSession::new(&design, &arch, &grade, &model, &constraints).unwrap();
        assert!(session.routed_model_inputs.is_some());
        let mut reports = Vec::new();
        // A rebuilt but identical tree is a changed pointer with equal contents.
        let rebuilt = Arc::new((*direct).clone());
        for pad_route in [&direct, &permuted, &rebuilt, &permuted, &direct] {
            let candidate = implementation(pad_route);
            let complete = analyze_ecp5_implementation(
                &design,
                &arch,
                &grade,
                &candidate,
                &model,
                &constraints,
            )
            .unwrap();
            assert_eq!(session.analyze(&candidate).unwrap(), complete);
            reports.push(complete);
        }
        assert_ne!(reports[0].net_delays, reports[1].net_delays);

        // Route lists outside net order still receive the complete checks.
        let mut reversed = implementation(&direct);
        reversed.routes.reverse();
        assert_eq!(session.analyze(&reversed).unwrap(), reports[0]);
        let mut missing = implementation(&direct);
        missing.routes.pop();
        assert!(matches!(
            session.analyze(&missing),
            Err(Ecp5FlowError::Timing(TimingError::MissingRoute(net))) if net == feeder_net
        ));
        assert_eq!(
            session.analyze(&implementation(&permuted)).unwrap(),
            reports[1]
        );
    }
}
