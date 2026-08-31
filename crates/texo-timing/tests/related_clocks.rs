//! Integration tests for synchronous paths between related clock domains.

use texo_model::{CellPinId, Design, NetId, PinDirection, ResourceKind};
use texo_timing::{
    ClockEdge, DelayRange, NetDelay, TimingConstraints, TimingError, TimingModel,
    UncheckedEndpointReason, analyze_timing_from_net_delays,
};

#[test]
fn checks_both_directions_between_related_integer_ratio_clocks() {
    let fixture = RelatedClockFixture::new();
    let mut constraints = fixture.constraints();

    let unchecked = analyze_timing_from_net_delays(
        &fixture.design,
        &fixture.model,
        &constraints,
        fixture.net_delays.clone(),
    )
    .unwrap();
    assert_eq!(unchecked.unchecked_endpoints.len(), 2);
    assert!(
        unchecked
            .unchecked_endpoints
            .iter()
            .all(|endpoint| endpoint.reason == UncheckedEndpointReason::NoSynchronousLaunch)
    );

    constraints.set_generated_clock(fixture.fast_clock, fixture.reference, 2, 1, 0);
    constraints.set_generated_clock(fixture.slow_clock, fixture.reference, 1, 1, 25);
    let report = analyze_timing_from_net_delays(
        &fixture.design,
        &fixture.model,
        &constraints,
        fixture.net_delays,
    )
    .unwrap();

    assert!(report.all_modeled_endpoints_checked());
    let setup = |capture| {
        report
            .setup_checks
            .iter()
            .find(|check| check.clock_net == capture)
            .unwrap()
    };
    let hold = |capture| {
        report
            .hold_checks
            .iter()
            .find(|check| check.clock_net == capture)
            .unwrap()
    };

    // Slow rising edges are phase shifted by 25 ps. Across the repeating 2:1
    // schedule, setup separation is 25 ps fast->slow and 75 ps slow->fast;
    // the preceding hold-edge separations are complementary.
    assert_eq!(setup(fixture.slow_clock).required_ps, 15);
    assert_eq!(setup(fixture.slow_clock).slack_ps, 10);
    assert_eq!(setup(fixture.fast_clock).required_ps, 65);
    assert_eq!(setup(fixture.fast_clock).slack_ps, 60);
    assert_eq!(hold(fixture.slow_clock).arrival_ps, 80);
    assert_eq!(hold(fixture.slow_clock).slack_ps, 70);
    assert_eq!(hold(fixture.fast_clock).arrival_ps, 30);
    assert_eq!(hold(fixture.fast_clock).slack_ps, 20);
    assert_eq!(report.net_setup_slacks.len(), 2);
    assert_eq!(report.net_setup_criticalities.len(), 2);
    assert!(
        report
            .net_setup_criticalities
            .iter()
            .all(|edge| { edge.path_delay_ps == 15 && edge.domain_worst_path_delay_ps == 15 })
    );
}

#[test]
fn rejects_generated_clock_cycles() {
    let fixture = RelatedClockFixture::new();
    let mut constraints = fixture.constraints();
    constraints.set_generated_clock(fixture.fast_clock, fixture.reference, 2, 1, 0);
    constraints.set_generated_clock(fixture.reference, fixture.fast_clock, 1, 2, 0);

    let error = analyze_timing_from_net_delays(
        &fixture.design,
        &fixture.model,
        &constraints,
        fixture.net_delays,
    )
    .unwrap_err();
    assert!(matches!(error, TimingError::GeneratedClockCycle(_)));
}

#[test]
fn rejects_inconsistent_sibling_periods_with_unconstrained_root() {
    let fixture = RelatedClockFixture::new();
    let mut constraints = fixture.constraints();
    constraints.set_generated_clock(fixture.fast_clock, fixture.reference, 2, 1, 0);
    constraints.set_generated_clock(fixture.slow_clock, fixture.reference, 1, 1, 0);

    // Both independently rounded endpoints may be within one picosecond of
    // an exact common-source waveform, so the intervals still meet at 202 ps.
    constraints.set_clock_period_ps(fixture.slow_clock, 203);
    analyze_timing_from_net_delays(
        &fixture.design,
        &fixture.model,
        &constraints,
        fixture.net_delays.clone(),
    )
    .unwrap();

    constraints.set_clock_period_ps(fixture.slow_clock, 204);
    let error = analyze_timing_from_net_delays(
        &fixture.design,
        &fixture.model,
        &constraints,
        fixture.net_delays,
    )
    .unwrap_err();
    assert!(matches!(
        error,
        TimingError::InconsistentRelatedClockPeriods { .. }
    ));
}

struct RegisterPins {
    clock: CellPinId,
    data: CellPinId,
    q: CellPinId,
}

struct RelatedClockFixture {
    design: Design,
    reference: NetId,
    fast_clock: NetId,
    slow_clock: NetId,
    model: TimingModel,
    net_delays: Vec<NetDelay>,
}

impl RelatedClockFixture {
    fn new() -> Self {
        let mut design = Design::new();
        let source = design.add_cell("clock-source", ResourceKind::Io);
        let reference_o = design
            .add_pin(source, "REFERENCE", PinDirection::Output)
            .unwrap();
        let fast_o = design
            .add_pin(source, "FAST", PinDirection::Output)
            .unwrap();
        let slow_o = design
            .add_pin(source, "SLOW", PinDirection::Output)
            .unwrap();
        let sink_cell = design.add_cell("reference-sink", ResourceKind::Io);
        let reference_sink = design.add_pin(sink_cell, "I", PinDirection::Input).unwrap();
        let fast_launch = add_register(&mut design, "fast-launch");
        let slow_capture = add_register(&mut design, "slow-capture");
        let slow_launch = add_register(&mut design, "slow-launch");
        let fast_capture = add_register(&mut design, "fast-capture");

        let reference = design
            .add_net("reference", reference_o, [reference_sink])
            .unwrap();
        let fast_clock = design
            .add_net(
                "fast-clock",
                fast_o,
                [fast_launch.clock, fast_capture.clock],
            )
            .unwrap();
        let slow_clock = design
            .add_net(
                "slow-clock",
                slow_o,
                [slow_launch.clock, slow_capture.clock],
            )
            .unwrap();
        design
            .add_net("fast-to-slow", fast_launch.q, [slow_capture.data])
            .unwrap();
        design
            .add_net("slow-to-fast", slow_launch.q, [fast_capture.data])
            .unwrap();

        let mut model = TimingModel::new();
        let clock_to_q = DelayRange::new(5, 5).unwrap();
        let setup_hold = DelayRange::new(10, 10).unwrap();
        for register in [&fast_launch, &slow_launch] {
            model
                .add_clock_to_q(register.clock, register.q, ClockEdge::Rising, clock_to_q)
                .unwrap();
        }
        for register in [&slow_capture, &fast_capture] {
            model
                .add_setup_hold(
                    register.clock,
                    register.data,
                    ClockEdge::Rising,
                    setup_hold,
                    setup_hold,
                )
                .unwrap();
        }
        let net_delays = design
            .nets()
            .iter()
            .enumerate()
            .flat_map(|(index, net)| {
                net.sinks.iter().map(move |&sink| NetDelay {
                    net: NetId(index),
                    sink,
                    delay: DelayRange::zero(),
                })
            })
            .collect();
        Self {
            design,
            reference,
            fast_clock,
            slow_clock,
            model,
            net_delays,
        }
    }

    fn constraints(&self) -> TimingConstraints {
        let mut constraints = TimingConstraints::new();
        constraints.set_clock_period_ps(self.fast_clock, 100);
        constraints.set_clock_period_ps(self.slow_clock, 200);
        constraints
    }
}

fn add_register(design: &mut Design, name: &str) -> RegisterPins {
    let cell = design.add_cell(name, ResourceKind::Register);
    RegisterPins {
        clock: design.add_pin(cell, "CLK", PinDirection::Input).unwrap(),
        data: design.add_pin(cell, "DI", PinDirection::Input).unwrap(),
        q: design.add_pin(cell, "Q", PinDirection::Output).unwrap(),
    }
}
