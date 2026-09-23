//! Measured LUT-hop ranking. Combined LUT/route measurements are never inserted
//! into a net-only STA arc. Unmeasured edges receive no synthetic delay.
// BEL names are case-sensitive architecture identifiers, not file extensions.
#![allow(clippy::case_sensitive_file_extension_comparisons)]

use serde::Deserialize;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;
use texo_model::{BelId, CellId, CellPinId, Design, NetId, ResourceKind};
use texo_pnr::{Placement, PlacementConstraints, placement_from_complete_bindings};
use texo_target_ecp5::Ecp5Architecture;
use texo_timing::TimingReport;

/// Validated measured ranking data with immutable source provenance.
#[derive(Clone, Debug)]
pub struct MeasuredPlacementModel {
    coefficients: BTreeMap<String, [f64; 6]>,
    holdout_error: f64,
    sha256: String,
    source: String,
}

#[derive(Deserialize)]
struct Record {
    schema: u32,
    kind: String,
    device: String,
    package: String,
    features: Vec<String>,
    coefficients_by_input_pin_ps: BTreeMap<String, [f64; 6]>,
    holdout_max_abs_relative_error: f64,
    input_sha256: BTreeMap<String, String>,
    sta_qualified: bool,
}

impl MeasuredPlacementModel {
    /// Load and verify every measured input hash. No guessed-model fallback.
    ///
    /// # Errors
    /// Rejects missing/tampered inputs, incompatible target or invalid fit.
    pub fn load(path: &Path) -> Result<Self, Box<dyn std::error::Error>> {
        let bytes = std::fs::read(path)?;
        let r: Record = serde_json::from_slice(&bytes)?;
        if r.schema != 1
            || r.kind != "measured_loop_average_lut_plus_route_cost"
            || r.device != "LFE5UM5G-85F"
            || r.package != "CABGA381"
            || r.sta_qualified
            || r.features
                != [
                    "lut_hops",
                    "x_near_tiles",
                    "x_far_tiles",
                    "y_near_tiles",
                    "y_far_tiles",
                    "loop_offset",
                ]
            || r.coefficients_by_input_pin_ps
                .keys()
                .map(String::as_str)
                .collect::<Vec<_>>()
                != ["A", "B", "C", "D"]
            || !r.holdout_max_abs_relative_error.is_finite()
            || !(0.0..=0.10).contains(&r.holdout_max_abs_relative_error)
            || r.coefficients_by_input_pin_ps
                .values()
                .flatten()
                .any(|v| !v.is_finite() || *v < 0.0)
            || r.input_sha256.is_empty()
        {
            return Err("invalid measured placement model or failed independent holdout".into());
        }
        for (file, expected) in &r.input_sha256 {
            let actual = format!("{:x}", Sha256::digest(std::fs::read(file)?));
            if actual != *expected {
                return Err(format!("measured input hash mismatch: {file}").into());
            }
        }
        Ok(Self {
            coefficients: r.coefficients_by_input_pin_ps,
            holdout_error: r.holdout_max_abs_relative_error,
            sha256: format!("{:x}", Sha256::digest(&bytes)),
            source: path.canonicalize()?.display().to_string(),
        })
    }

    pub(super) fn provenance(&self) -> Value {
        json!({"kind":"measured_lut_hop_ranking_v1", "model_path":self.source,
            "model_sha256":self.sha256, "holdout_max_abs_relative_error":self.holdout_error,
            "guessed_placement_predictor_enabled":false, "sta_model_replaced":false,
            "scope":"K0 F to K0 A/B/C/D, one logical sink, nonzero displacement, major axis <=12 tiles and minor axis <=2",
            "acceptance":"fresh whole-design routed STA, no worse setup or hold slack",
            "uncovered_edges":"not scored; all affected nets rerouted and timed"})
    }

    fn hop(
        &self,
        source: &str,
        sink: &str,
        pin: &str,
        dx: u32,
        dy: u32,
        fanout: usize,
    ) -> Option<f64> {
        if !source.ends_with(".K0")
            || !sink.ends_with(".K0")
            || fanout != 1
            || dx + dy == 0
            || dx.max(dy) > 12
            || dx.min(dy) > 2
        {
            return None;
        }
        let c = self.coefficients.get(pin)?;
        // Ring-specific gate/observer offset is deliberately not charged per hop.
        let f = [
            1.0,
            f64::from(dx.min(5)),
            f64::from(dx.saturating_sub(5)),
            f64::from(dy.min(5)),
            f64::from(dy.saturating_sub(5)),
            0.0,
        ];
        Some(c.iter().zip(f).map(|(a, b)| a * b).sum())
    }
}

pub(super) struct Proposal {
    pub placement: Placement,
    pub nets: BTreeSet<NetId>,
    pub report: Value,
}

struct Scored {
    cell: CellId,
    target: BelId,
    nets: BTreeSet<NetId>,
    before: f64,
    after: f64,
    slack: i128,
    covered: usize,
    total: usize,
}

#[allow(clippy::too_many_arguments)]
fn edge_score(
    model: &MeasuredPlacementModel,
    design: &Design,
    arch: &Ecp5Architecture,
    place: &Placement,
    net_id: NetId,
    sink: CellPinId,
    moved: CellId,
    target: BelId,
) -> Option<f64> {
    let net = &design.nets()[net_id.0];
    let driver = &design.pins()[net.driver.0];
    let sink = &design.pins()[sink.0];
    if driver.name != "F"
        || design.cells()[driver.cell.0].kind != ResourceKind::Lut(4)
        || design.cells()[sink.cell.0].kind != ResourceKind::Lut(4)
    {
        return None;
    }
    let sb = if driver.cell == moved {
        target
    } else {
        place.bel(driver.cell)?
    };
    let tb = if sink.cell == moved {
        target
    } else {
        place.bel(sink.cell)?
    };
    let source = &arch.device().bels()[sb.0];
    let target = &arch.device().bels()[tb.0];
    model.hop(
        &source.name,
        &target.name,
        &sink.name,
        source.point.x.abs_diff(target.point.x),
        source.point.y.abs_diff(target.point.y),
        net.sinks.len(),
    )
}

/// Rank legal local moves using measurements; routed STA only orders urgency.
#[allow(clippy::too_many_lines)]
pub(super) fn propose(
    model: &MeasuredPlacementModel,
    design: &Design,
    arch: &Ecp5Architecture,
    constraints: &PlacementConstraints,
    place: &Placement,
    timing: &TimingReport,
) -> Vec<Proposal> {
    let occupied: BTreeSet<_> = place.bindings().iter().copied().collect();
    let grouped: BTreeSet<_> = constraints
        .groups()
        .iter()
        .flat_map(|g| g.cells.iter().copied())
        .collect();
    let mut free = BTreeMap::<(u32, u32), Vec<BelId>>::new();
    for (i, b) in arch.device().bels().iter().enumerate() {
        if b.kind == ResourceKind::Lut(4)
            && b.name.ends_with(".K0")
            && !occupied.contains(&BelId(i))
        {
            free.entry((b.point.x, b.point.y))
                .or_default()
                .push(BelId(i));
        }
    }
    let mut incidents = vec![Vec::new(); design.cells().len()];
    for (i, n) in design.nets().iter().enumerate() {
        let source = design.pins()[n.driver.0].cell;
        for &sink in &n.sinks {
            incidents[source.0].push((NetId(i), sink));
            let target = design.pins()[sink.0].cell;
            if target != source {
                incidents[target.0].push((NetId(i), sink));
            }
        }
    }
    let mut slacks = BTreeMap::<NetId, i128>::new();
    for s in &timing.net_setup_slacks {
        slacks
            .entry(s.net)
            .and_modify(|v| *v = (*v).min(s.slack_ps))
            .or_insert(s.slack_ps);
    }
    let mut scored = Vec::new();
    for (i, c) in design.cells().iter().enumerate() {
        let cell = CellId(i);
        let Some(old) = place.bel(cell) else {
            continue;
        };
        if c.kind != ResourceKind::Lut(4)
            || grouped.contains(&cell)
            || !arch.device().bels()[old.0].name.ends_with(".K0")
        {
            continue;
        }
        let incident = &incidents[i];
        let covered = incident
            .iter()
            .filter_map(|&(n, s)| {
                edge_score(model, design, arch, place, n, s, cell, old).map(|v| (n, s, v))
            })
            .collect::<Vec<_>>();
        if covered.is_empty() || covered.len() * 2 < incident.len() {
            continue;
        }
        let slack = covered
            .iter()
            .filter_map(|(n, _, _)| slacks.get(n))
            .copied()
            .min()
            .unwrap_or(i128::MAX);
        if slack > 2000 {
            continue;
        }
        let before: f64 = covered.iter().map(|(_, _, v)| v).sum();
        let point = arch.device().bels()[old.0].point;
        for dx in -3..=3 {
            for dy in -3..=3 {
                let (Some(x), Some(y)) = (
                    point.x.checked_add_signed(dx),
                    point.y.checked_add_signed(dy),
                ) else {
                    continue;
                };
                for &target in free.get(&(x, y)).into_iter().flatten() {
                    let Some(after) = covered
                        .iter()
                        .map(|&(n, s, _)| {
                            edge_score(model, design, arch, place, n, s, cell, target)
                        })
                        .sum::<Option<f64>>()
                    else {
                        continue;
                    };
                    if before - after <= (2.0 * model.holdout_error * before).max(100.0) {
                        continue;
                    }
                    scored.push(Scored {
                        cell,
                        target,
                        nets: incident.iter().map(|(n, _)| *n).collect(),
                        before,
                        after,
                        slack,
                        covered: covered.len(),
                        total: incident.len(),
                    });
                }
            }
        }
    }
    scored.sort_by(|a, b| {
        a.slack
            .cmp(&b.slack)
            .then_with(|| (b.before - b.after).total_cmp(&(a.before - a.after)))
            .then_with(|| {
                design.cells()[a.cell.0]
                    .name
                    .cmp(&design.cells()[b.cell.0].name)
            })
            .then_with(|| {
                arch.device().bels()[a.target.0]
                    .name
                    .cmp(&arch.device().bels()[b.target.0].name)
            })
    });
    let mut proposals = Vec::new();
    for s in scored {
        let mut bindings = place.bindings().to_vec();
        let old = bindings[s.cell.0];
        bindings[s.cell.0] = s.target;
        // Enforces carry occupancy, packed FFs, pin bindings and shared resources.
        let Ok(placement) =
            placement_from_complete_bindings(design, arch.device(), constraints, bindings)
        else {
            continue;
        };
        let report = json!({"cell":design.cells()[s.cell.0].name,"old":arch.device().bels()[old.0].name,
            "new":arch.device().bels()[s.target.0].name,"score_before_ps":s.before,"score_after_ps":s.after,
            "modeled_incident_hops":s.covered,"unmodeled_incident_hops":s.total-s.covered,
            "minimum_incident_native_slack_ps":s.slack,"incident_nets":s.nets.iter().map(|n|&design.nets()[n.0].name).collect::<Vec<_>>()});
        proposals.push(Proposal {
            placement,
            nets: s.nets,
            report,
        });
        if proposals.len() == 4 {
            break;
        }
    }
    proposals
}

pub(super) fn accepts(incumbent: &TimingReport, candidate: &TimingReport) -> bool {
    candidate.met_timing()
        && candidate.worst_slack_ps >= incumbent.worst_slack_ps
        && candidate.worst_hold_slack_ps >= incumbent.worst_hold_slack_ps
        && candidate.setup_checks.len() == incumbent.setup_checks.len()
        && candidate.hold_checks.len() == incumbent.hold_checks.len()
        && candidate.unchecked_endpoints == incumbent.unchecked_endpoints
}

#[cfg(test)]
mod tests {
    use super::*;
    fn model() -> MeasuredPlacementModel {
        MeasuredPlacementModel {
            coefficients: ["A", "B", "C", "D"]
                .into_iter()
                .map(|p| (p.into(), [400.0, 30.0, 20.0, 40.0, 25.0, 900.0]))
                .collect(),
            holdout_error: 0.04,
            sha256: String::new(),
            source: String::new(),
        }
    }
    #[test]
    fn unsupported_edges_never_receive_fabricated_costs() {
        let m = model();
        for (source, sink, pin, dx, dy, fanout) in [
            ("a.K1", "b.K0", "A", 1, 0, 1),
            ("a.K0", "b.FF0", "DI", 1, 0, 1),
            ("a.K0", "b.K0", "A", 1, 0, 2),
            ("a.K0", "b.K0", "A", 13, 0, 1),
            ("a.K0", "b.K0", "A", 3, 3, 1),
            ("a.K0", "b.K0", "A", 0, 0, 1),
        ] {
            assert_eq!(m.hop(source, sink, pin, dx, dy, fanout), None);
        }
    }
    #[test]
    fn loop_offset_is_not_double_counted_per_hop() {
        assert_eq!(model().hop("a.K0", "b.K0", "A", 6, 1, 1), Some(610.0));
    }

    #[test]
    fn rejected_timing_cannot_trade_hold_for_setup_or_lose_constraints() {
        let baseline = TimingReport {
            net_delays: vec![],
            net_setup_slacks: vec![],
            net_setup_criticalities: vec![],
            setup_checks: vec![],
            hold_checks: vec![],
            unchecked_endpoints: vec![],
            worst_slack_ps: Some(0),
            worst_hold_slack_ps: Some(90),
        };
        assert!(accepts(&baseline, &baseline));
        for (setup, hold) in [(Some(-1), Some(90)), (Some(100), Some(89)), (None, None)] {
            let mut next = baseline.clone();
            next.worst_slack_ps = setup;
            next.worst_hold_slack_ps = hold;
            assert!(!accepts(&baseline, &next));
        }
    }

    #[test]
    fn tampered_measurement_is_rejected_before_use() {
        let dir = std::env::temp_dir().join(format!("texo-measured-input-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let sample = dir.join("sample.json");
        let model_path = dir.join("model.json");
        std::fs::write(&sample, b"measured counts").unwrap();
        let value = json!({"schema":1,"kind":"measured_loop_average_lut_plus_route_cost",
            "device":"LFE5UM5G-85F","package":"CABGA381","sta_qualified":false,
            "features":["lut_hops","x_near_tiles","x_far_tiles","y_near_tiles","y_far_tiles","loop_offset"],
            "coefficients_by_input_pin_ps":model().coefficients,"holdout_max_abs_relative_error":0.04,
            "input_sha256":{sample.display().to_string():format!("{:x}",Sha256::digest(b"measured counts"))}});
        std::fs::write(&model_path, serde_json::to_vec(&value).unwrap()).unwrap();
        assert!(MeasuredPlacementModel::load(&model_path).is_ok());
        std::fs::write(&sample, b"different counts").unwrap();
        assert!(MeasuredPlacementModel::load(&model_path).is_err());
        std::fs::remove_dir_all(dir).unwrap();
    }
}
