# Constraining an internal clock source

An ECP5 JTAG binding removes the external TCK port and replaces it with
`JTAGG.JTCK`. An LPF `FREQUENCY PORT` command can no longer select that source.
Use a clock-constraint JSON file to constrain the exact mapped cell output:

```json
[
  {"cell": "jtagg", "pin": "JTCK", "period_ps": 166666}
]
```

```sh
texo pnr path/to/project --package CABGA381 --speed 8 \
  --lpf board.lpf --jtagg-prefix jtag \
  --clock-constraints clocks.json --timing-exceptions timing-review.json \
  --output design.json
```

Here 166,666 ps conservatively represents the minimum period at 6 MHz TCK.
Use the fastest clock allowed by the board/host contract. The names refer to
the mapped cell and connected output pin, not a potentially unstable generated
wire name. Library callers set `Ecp5FlowOptions::clock_constraints`.

The source must drive a clock input. The flow rejects missing/ambiguous cells,
missing/input/unconnected pins, zero periods, unknown JSON fields, duplicate
source constraints, and conflicts with LPF or PLL-derived periods. An
assertion on a DCCA output resolves back to its source instead of declaring an
unrelated primary clock. This also catches duplicate source/output aliases.
Explicit periods enter the same period table used by PLL derivation and global
clock propagation; matching assertions retain the physical PLL ratios/phases.

The period reaches promoted DCCA nets with their 1:1 relationship. Register
launch/capture edges come from primitive configuration: falling-edge JTAG
registers are analyzed on falling edges, and opposite-edge paths get a
half-period budget. JTCK remains unrelated to CPU clocks. A period constraint
does not fabricate a synchronous relationship or a launch arc from JTAGG.

Checkpoints retain the user input as `timing.clock_constraints` and resolved
source periods in `packing.generated_clock_periods_ps` (the existing table
also holds explicit internal source periods). The normal timing report now
includes the constrained domain's register-to-register checks. Source
constraints affect placement/routing timing objectives, so adding one can
change the physical implementation even with otherwise identical RTL/options.

## Boundary coverage

`timing.unmodeled_boundaries` lists each connected JTAGG interface and marks
its launch/capture characterization as absent. The CLI also prints that
limitation. A JTAGG output data/control cone with no modeled launch still
reports `no_synchronous_launch`, requiring an exact justified exception under
the [timing coverage policy](timing-coverage.md). The report does not silently
create a zero-delay JTAGG arc or count an exception as a timing check.

This list currently describes JTAGG boundaries, not a complete library-wide
coverage audit. Asynchronous reset recovery/removal and external/CDC timing
contracts remain separate verification work. A bitgen coverage gate validates
the supplied model and review records; it cannot prove an uncharacterized
hard-macro interface safe.

Regression tests use the real ECP5 timing model and engine to check JTCK
propagation, falling/opposite-edge budgets, retained external boundaries,
DCCA aliases, rejected stale inputs, and LPF/PLL conflict handling.

## Reserving setup margin

`pnr --setup-uncertainty-ps 250` reserves 250 ps on every constrained capture
clock, including PLL outputs and DCCA-promoted clocks. The API field is
`Ecp5FlowOptions::setup_uncertainty_ps`; the default is zero. Nominal periods,
PLL phase/ratios and hold checks remain unchanged. All placement, routing and
hold-repair trials use the same guarded setup budget. The checkpoint records
`timing.setup_uncertainty_ps` as well as each setup check's `uncertainty_ps`.
A guarded setup slack of +1 ps with a 250 ps reserve is +251 ps against the
nominal period. This is a user-selected engineering reserve; it does not
characterize PLL jitter, clock-tree skew or an unmodeled primitive boundary.
