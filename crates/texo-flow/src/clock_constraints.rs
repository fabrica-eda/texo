//! Explicit period constraints for clock sources inside the mapped design.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use texo_model::{CellId, Design, NetId, PinDirection};
use texo_target_ecp5::Ecp5Packing;

use super::{Ecp5FlowError, find_cell_pin};

/// A primary clock period specified on an exact mapped cell output pin.
///
/// This supports internal clock sources such as `JTAGG.JTCK`, whose external
/// port disappears during target binding. It supplies a waveform period, not
/// clock-to-Q characterization for the source primitive. PLL and DCCA clock
/// relationships remain derived from the physical implementation.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ClockConstraint {
    /// Exact mapped source cell name (no patterns).
    pub cell: String,
    /// Exact connected output pin on the source cell.
    pub pin: String,
    /// Minimum clock period in integer picoseconds; must be nonzero.
    pub period_ps: u64,
}

pub(crate) fn apply_clock_constraints(
    design: &Design,
    packing: &mut Ecp5Packing,
    clocks: &[ClockConstraint],
) -> Result<(), Ecp5FlowError> {
    let mut resolved = BTreeMap::<NetId, u64>::new();
    for clock in clocks {
        let invalid = |reason: &str| Ecp5FlowError::InvalidClockConstraint {
            cell: clock.cell.clone(),
            pin: clock.pin.clone(),
            reason: reason.into(),
        };
        if clock.period_ps == 0 {
            return Err(invalid("period_ps must be nonzero"));
        }
        let mut cells = design
            .cells()
            .iter()
            .enumerate()
            .filter(|(_, c)| c.name == clock.cell);
        let Some((index, _)) = cells.next() else {
            return Err(invalid("source cell does not exist"));
        };
        if cells.next().is_some() {
            return Err(invalid("source cell name is ambiguous"));
        }
        let pin = find_cell_pin(design, CellId(index), &clock.pin)
            .ok_or_else(|| invalid("source pin does not exist"))?;
        let source = &design.pins()[pin.0];
        if source.direction != PinDirection::Output {
            return Err(invalid("source pin must be an output"));
        }
        let mut net = source
            .net()
            .ok_or_else(|| invalid("source pin is unconnected"))?;
        if !design.nets()[net.0].sinks.iter().any(|pin| {
            matches!(
                design.pins()[pin.0].name.as_str(),
                "CLK" | "CLKA" | "CLKB" | "CLKI"
            )
        }) {
            return Err(invalid(
                "source must directly drive a register, RAM, PLL, or DCCA clock pin",
            ));
        }
        // An assertion on a promoted output is an assertion on its source,
        // not a new unrelated clock. Keep all DCCA relationships intact.
        let mut seen = BTreeSet::new();
        while let Some(clock_buffer) = packing.global_clocks().iter().find(|c| c.global_net == net)
        {
            if !seen.insert(net) {
                return Err(invalid("cycle in global clock sources"));
            }
            net = clock_buffer.source_net;
        }
        if resolved.insert(net, clock.period_ps).is_some() {
            return Err(invalid("clock source is constrained more than once"));
        }
        if packing
            .generated_clock_periods_ps()
            .get(&net)
            .is_some_and(|&p| p != clock.period_ps)
        {
            return Err(Ecp5FlowError::ConflictingClockPeriods { net });
        }
    }
    // Resolve the entire list before mutating packing. The existing period
    // table is also the input to PLL derivation and DCCA propagation; a period
    // entry alone never makes the clock related to another source.
    for (net, period) in resolved {
        packing.set_generated_clock_period_ps(net, period);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ecp5_pll::GeneratedClockRelations, ecp5_timing_constraints, ecp5_timing_model};
    use texo_model::ResourceKind;
    use texo_struo::{ClockEdge, PrimitiveMetadata};
    use texo_target_ecp5::{Ecp5Architecture, GlobalClockRequirement, read_architecture};
    use texo_timing::{
        ClockEdge as TimingEdge, DelayRange, NetDelay, TimingReport, analyze_timing_from_net_delays,
    };

    struct Fixture {
        design: Design,
        packing: Ecp5Packing,
        architecture: Ecp5Architecture,
        metadata: BTreeMap<CellId, PrimitiveMetadata>,
        root: NetId,
    }

    impl Fixture {
        fn new() -> Self {
            let architecture = read_architecture(
                include_bytes!("../../texo-target-ecp5/fixtures/minimal-ecp5.json").as_slice(),
            )
            .unwrap();
            let mut design = Design::new();
            let jtagg = design.add_cell("tap", ResourceKind::Logic);
            let jtck = design.add_pin(jtagg, "JTCK", PinDirection::Output).unwrap();
            let jtdi = design.add_pin(jtagg, "JTDI", PinDirection::Output).unwrap();
            let mut metadata = BTreeMap::from([(
                jtagg,
                PrimitiveMetadata::Jtagg {
                    extension_register_1: true,
                    extension_register_2: true,
                },
            )]);
            let mut clocks = Vec::new();
            let mut data = Vec::new();
            let mut q = Vec::new();
            for name in ["launch", "capture", "boundary"] {
                let cell = design.add_cell(name, ResourceKind::Register);
                clocks.push(design.add_pin(cell, "CLK", PinDirection::Input).unwrap());
                data.push(design.add_pin(cell, "DI", PinDirection::Input).unwrap());
                q.push(design.add_pin(cell, "Q", PinDirection::Output).unwrap());
                metadata.insert(
                    cell,
                    PrimitiveMetadata::FlipFlop {
                        edge: ClockEdge::Falling,
                        enable: None,
                        reset: None,
                    },
                );
            }
            let root = design
                .add_net("unstable_generated_name", jtck, clocks)
                .unwrap();
            design
                .add_net("registered_data", q[0], [data[0], data[1]])
                .unwrap();
            design.add_net("external_data", jtdi, [data[2]]).unwrap();
            let mut packing = Ecp5Packing::default();
            packing
                .promote_global_clocks(
                    &mut design,
                    &architecture,
                    [GlobalClockRequirement { net: root }],
                )
                .unwrap();
            Self {
                design,
                packing,
                architecture,
                metadata,
                root,
            }
        }

        fn analyze(&self) -> TimingReport {
            self.analyze_with_margin(0)
        }

        fn analyze_with_margin(&self, uncertainty_ps: u64) -> TimingReport {
            let model = ecp5_timing_model(
                &self.design,
                &self.packing,
                &self.architecture.speed_grades()["6"],
                &BTreeSet::new(),
                &self.metadata,
            )
            .unwrap();
            let mut constraints = ecp5_timing_constraints(
                &self.design,
                &self.packing,
                &GeneratedClockRelations::new(),
            )
            .unwrap();
            crate::apply_setup_uncertainty(&mut constraints, uncertainty_ps);
            let delays = self
                .design
                .nets()
                .iter()
                .enumerate()
                .flat_map(|(index, net)| {
                    net.sinks.iter().map(move |&sink| NetDelay {
                        net: NetId(index),
                        sink,
                        delay: DelayRange::new(500, 500).unwrap(),
                    })
                })
                .collect();
            analyze_timing_from_net_delays(&self.design, &model, &constraints, delays).unwrap()
        }
    }

    #[test]
    fn setup_margin_preserves_periods_edges_and_hold_in_promoted_domain() {
        let mut fixture = Fixture::new();
        apply_clock_constraints(
            &fixture.design,
            &mut fixture.packing,
            &[ClockConstraint {
                cell: "tap".into(),
                pin: "JTCK".into(),
                period_ps: 166_666,
            }],
        )
        .unwrap();
        let nominal = fixture.analyze();
        let guarded = fixture.analyze_with_margin(250);
        assert_eq!(nominal.hold_checks, guarded.hold_checks);
        assert_eq!(nominal.unchecked_endpoints, guarded.unchecked_endpoints);
        for (before, after) in nominal.setup_checks.iter().zip(&guarded.setup_checks) {
            assert_eq!(before.slack_ps - 250, after.slack_ps);
            assert_eq!(after.uncertainty_ps, 250);
            assert_eq!(before.launch_edge, after.launch_edge);
            assert_eq!(before.capture_edge, after.capture_edge);
        }
    }

    fn jtck() -> ClockConstraint {
        ClockConstraint {
            cell: "tap".into(),
            pin: "JTCK".into(),
            period_ps: 166_666,
        }
    }

    #[test]
    fn internal_jtck_checks_falling_edge_registers_but_keeps_macro_boundary_unchecked() {
        let mut fixture = Fixture::new();
        let before = fixture.analyze();
        assert_eq!(before.setup_checks.len(), 0);
        assert!(
            before
                .unchecked_endpoints
                .iter()
                .all(|e| e.reason.as_str() == "unconstrained_clock")
        );
        apply_clock_constraints(&fixture.design, &mut fixture.packing, &[jtck()]).unwrap();
        let after = fixture.analyze();
        assert!(after.met_timing());
        assert_eq!(after.setup_checks.len(), 2);
        assert_eq!(after.hold_checks.len(), 2);
        assert!(
            after
                .setup_checks
                .iter()
                .all(|c| c.launch_edge == TimingEdge::Falling
                    && c.capture_edge == TimingEdge::Falling)
        );
        assert_eq!(after.unchecked_endpoints.len(), 1);
        assert_eq!(
            after.unchecked_endpoints[0].reason.as_str(),
            "no_synchronous_launch"
        );
        let endpoint = after.unchecked_endpoints[0];
        assert_eq!(fixture.design.cells()[endpoint.cell.0].name, "boundary");
        let constraints = ecp5_timing_constraints(
            &fixture.design,
            &fixture.packing,
            &GeneratedClockRelations::new(),
        )
        .unwrap();
        assert_eq!(constraints.clock_periods_ps()[&fixture.root], 166_666);
        assert_eq!(
            constraints.clock_periods_ps()[&fixture.packing.global_clocks()[0].global_net],
            166_666
        );

        // A rising capture sees only half the period from a falling launch.
        let capture = fixture
            .design
            .cells()
            .iter()
            .position(|c| c.name == "capture")
            .unwrap();
        if let PrimitiveMetadata::FlipFlop { edge, .. } =
            fixture.metadata.get_mut(&CellId(capture)).unwrap()
        {
            *edge = ClockEdge::Rising;
        }
        let opposite = fixture.analyze();
        let slack = |r: &TimingReport| {
            r.setup_checks
                .iter()
                .find(|c| c.cell == CellId(capture))
                .unwrap()
                .slack_ps
        };
        assert_eq!(slack(&after) - slack(&opposite), 83_333);
    }

    #[test]
    fn promoted_output_constraints_preserve_the_source_clock() {
        let mut fixture = Fixture::new();
        let buffer = fixture.packing.global_clocks()[0].buffer;
        let alias = ClockConstraint {
            cell: fixture.design.cells()[buffer.0].name.clone(),
            pin: "CLKO".into(),
            period_ps: 166_666,
        };
        apply_clock_constraints(
            &fixture.design,
            &mut fixture.packing,
            std::slice::from_ref(&alias),
        )
        .unwrap();
        assert_eq!(
            fixture.packing.generated_clock_periods_ps(),
            &BTreeMap::from([(fixture.root, 166_666)])
        );
        assert!(fixture.analyze().met_timing());
        assert!(
            apply_clock_constraints(&fixture.design, &mut fixture.packing, &[jtck(), alias])
                .is_err()
        );
    }

    #[test]
    fn invalid_or_stale_source_constraints_fail_without_mutating_packing() {
        for bad in [
            ClockConstraint {
                period_ps: 0,
                ..jtck()
            },
            ClockConstraint {
                cell: "missing".into(),
                ..jtck()
            },
            ClockConstraint {
                pin: "missing".into(),
                ..jtck()
            },
            ClockConstraint {
                pin: "JTDI".into(),
                ..jtck()
            },
            ClockConstraint {
                cell: "launch".into(),
                pin: "CLK".into(),
                ..jtck()
            },
            ClockConstraint {
                cell: "capture".into(),
                pin: "Q".into(),
                ..jtck()
            },
        ] {
            let mut fixture = Fixture::new();
            let original = fixture.packing.clone();
            assert!(
                apply_clock_constraints(&fixture.design, &mut fixture.packing, &[jtck(), bad])
                    .is_err()
            );
            assert_eq!(fixture.packing, original);
        }
        let mut fixture = Fixture::new();
        fixture
            .packing
            .set_generated_clock_period_ps(fixture.root, 100_000);
        assert!(matches!(
            apply_clock_constraints(&fixture.design, &mut fixture.packing, &[jtck()]),
            Err(Ecp5FlowError::ConflictingClockPeriods { .. })
        ));
        assert_eq!(
            fixture.packing.generated_clock_periods_ps()[&fixture.root],
            100_000
        );
    }

    #[test]
    fn ambiguous_sources_and_unknown_json_fields_are_rejected() {
        let mut fixture = Fixture::new();
        fixture.design.add_cell("tap", ResourceKind::Logic);
        assert!(apply_clock_constraints(&fixture.design, &mut fixture.packing, &[jtck()]).is_err());
        assert!(
            serde_json::from_str::<ClockConstraint>(
                r#"{"cell":"tap","pin":"JTCK","period_ps":1000,"period":2000}"#
            )
            .is_err()
        );
    }
}
