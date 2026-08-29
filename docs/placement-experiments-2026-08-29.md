# Placement algorithm experiments (2026-08-29)

## Question

The frozen Core250 mapping still showed a placement-sensitive QoR gap: the
saved Texo run closed 250 MHz with +7 ps setup slack, while the saved nextpnr
seed reported 258.93 MHz after routing. This study isolates analytical
placement changes while keeping the RTL, 250 MHz synthesis goal, device,
package, speed grade, router, and timing model fixed.

The fixture PLL now fails Texo's newer phase-detector jitter guard because its
12 MHz input is divided by three. Experiments therefore used a temporary
benchmark-only binding with `CLKI_DIV=1`. The generated-clock attribute, logic
mapping, device, package, and all placement comparisons were otherwise held
constant. This binding is not a valid replacement for the shipped bitstream
configuration and is not committed.

## Literature and implementation comparison

- FastPlace explains the central weakness of a plain quadratic objective: it
  is only an indirect approximation of linear wirelength. Its remedy combines
  quadratic solves, density spreading/cell shifting, and HPWL-oriented local
  refinement: <https://home.engineering.iastate.edu/~cnchu/pubs/c20.pdf>.
- nextpnr's HeAP source uses a bound-to-bound net model and divides analytical
  edge weight by current coordinate separation, repeatedly rebuilding and
  solving each axis around spreading/legalization iterations:
  <https://github.com/YosysHQ/nextpnr/blob/main/common/place/placer_heap.cc>.
- elfPlace and DREAMPlaceFPGA represent the nonlinear/electrostatic family.
  They jointly optimize smooth wirelength and resource-specific density, then
  legalize and detail-place. This is attractive for much larger, denser
  devices, but is a substantially larger replacement than the missing update
  in Texo's existing quadratic solver:
  <https://yibolin.com/publications/papers/FPGA_TCAD2021_Meng.pdf> and
  <https://github.com/rachelselinar/DREAMPlaceFPGA>.
- VPR's alternative is timing-driven simulated annealing. It is useful as a
  basin-escape reference, but Texo already spends routed STA feedback on
  deterministic critical-cell moves; adding a second broad stochastic engine
  would cost much more than fixing the global analytical objective:
  <https://docs.verilogtorouting.org/en/latest/vpr/command_line_usage/>.
- Recent timing-driven placement work distinguishes net-weighting from direct
  path objectives and reports gains from fine-grained critical-path attraction.
  Texo already has routed per-sink criticalities and path-vertex refinement, so
  the first missing mechanism was wirelength linearization rather than another
  copy of net weighting:
  <https://arxiv.org/abs/2503.11674>.

## Implemented experiment

The analytical solver now performs iteratively reweighted least squares
(IRLS) during its four density-spreading rounds. For one coordinate, an edge
with logical/timing weight `w` and current separation `d` receives quadratic
weight `w / max(1, |d|)`. Its quadratic gradient is therefore approximately a
constant-magnitude L1 pull instead of growing linearly with the length of an
already-long edge.

The implementation preserves:

- fixed BEL boundary conditions;
- placement-group member offsets;
- per-sink timing weights;
- soft anchors used by iterative timing placement;
- deterministic solve, legalization, and tie-breaking.

## Core250 results

All 270 MHz rows use the same 250 MHz synthesized mapping and disable CLI
physical-synthesis feedback only for the experiment, so each number is the
first P&R result for the same netlist. Runtime includes the complete CLI flow.

| analytical variant | initial HPWL | initial WNS | final WNS @270 | runtime | result |
|---|---:|---:|---:|---:|---|
| original fixed quadratic | 160,411 | -5,229 ps | -518 ps | 74.3 s | baseline |
| IRLS only in 4 density rounds | 159,689 | -3,770 ps | **-62 ps** | **61.8 s** | kept |
| 4 extra IRLS solves before density (8 total) | 146,488 | -2,663 ps | -199 ps | 43.4 s first P&R | rejected: lower HPWL, worse final WNS |
| bound-to-bound net model plus IRLS | 165,801 | -4,794 ps | below -1,856 ps when stopped | >38 s partial | rejected early |

At a 3,704 ps target period, -62 ps corresponds to an approximately 3,766 ps
critical period, or 265.5 MHz. This is above the saved nextpnr result of
258.93 MHz on the fixture, though the tools' independent runs and the temporary
PLL validation workaround mean it should be treated as a regression result,
not a general cross-tool performance claim.

The rejected variants are informative. Minimizing initial HPWL more
aggressively did not select the best routed timing basin. The nextpnr-style
bound-to-bound model also cannot simply replace Texo's driver-to-sink model:
it diluted the existing per-sink timing forces and badly degraded both WNS and
TNS. The useful transferable mechanism was inverse-distance reweighting,
coupled specifically to density spreading.

A final nominal-constraint regression with the kept code closed 250 MHz at
+1 ps WNS, +290 ps hold slack, and 43,176 PIPs in 46.0 s. The adjacent
original run closed at +4 ps and 43,418 PIPs in 31.1 s. IRLS therefore buys
substantial high-effort Fmax headroom but can spend more search time when the
old placement already barely meets the requested constraint.

## Follow-up

The next placement work should add at least one unrelated dense/high-utilization
fixture before changing the global model again. If a larger residual gap is
found there, the best-supported next experiment is resource-specific
electrostatic density (logic, RAM, DSP independently), not more IRLS passes.
Path-count or explicit critical-path attraction is a secondary experiment once
the benchmark shows that current per-sink routed feedback is insufficient.
