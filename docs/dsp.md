# ECP5 combinational DSP multipliers

Struo wrapping multiplication can map to `MULT18X18D` partial products. Texo
imports these as a separate DSP resource, places them on the four legal site
positions (`z = 0, 1, 4, 5`), routes A/B and the unsigned/direct-input controls,
and emits the matching Trellis tile-group settings. Internal registers and
cascade are not supported yet. Pipeline registers remain ordinary fabric FFs.

STA uses `MULT18X18D:REGS=NONE` from the selected Project Trellis speed grade.
The exporter expands the characterized A/B and signedness bus arcs to all
scalar pins, including the high product bits. Missing, duplicate or incomplete
records fail instead of making a DSP path appear delay-free. Native bitgen
accepts only this modeled DSP configuration. The unused C input pins receive
CIB tie-offs, following all fixed wire aliases and rejecting ambiguous or
programmable alternatives; SIGNEDA/B and SOURCEA/B remain explicit routed zeros.

New architecture caches use format 6. Format 5 remains readable for existing
designs without DSPs, preserving compatibility with the current target-pack
catalog. DSP designs require a newly exported architecture with DSP BEL
classification and timing; the existing downloadable cache 5 is insufficient.
Generate it with `tools/export_ecp5.py` and `texo cache-architecture`, then use
`--architecture` explicitly. `architectures/ecp5/manifest.json` describes the
format-6 asset; publishing a replacement target pack is a separate release step.

`examples/dsp-pipeline` exercises every A/B/P bit in a registered multiplier.
For example, with an exported cache:

```sh
texo pnr examples/dsp-pipeline --architecture dsp.txdb \
  --package CABGA381 --speed 8 --lpf examples/dsp-pipeline/timing.lpf \
  --synthesis-goal-mhz 175 --setup-uncertainty-ps 200 --output dsp.json
```

Its LPF is an STA fixture with a 124.8 MHz input-clock constraint. It does not
create that frequency from the evaluation board's 12 MHz reference. A board
application needs a PLL binding for the intended operating clock.

Source references: [Lattice sysDSP guide](https://www.latticesemi.com/view_document?document_id=50469),
[Project Trellis](https://github.com/YosysHQ/prjtrellis), and
[nextpnr's ECP5 configuration writer](https://github.com/YosysHQ/nextpnr/blob/main/ecp5/bitstream.cc).
