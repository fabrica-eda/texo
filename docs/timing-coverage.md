# Timing coverage before bit generation

`TimingReport::met_timing()` and checkpoint `timing.met_timing` report the
numerical result for **checked** setup/hold paths. Positive slacks do not
establish coverage of another clock domain. The physical flow now records
`Gate::TimingClosure` only if those checks pass and every modeled endpoint
omitted from them satisfies the coverage policy. A timing gate inherited from
a previous run is removed before recording the current result.

PnR still saves an incomplete checkpoint for diagnosis. Its console output
distinguishes checked-path timing from timing closure and reports the first
coverage error. `timing.meets_timing_closure` records the combined result.

## Coverage policy

- `unconnected_clock` and `unconstrained_clock` always block release. Connect
  and constrain the capture clock; these reasons cannot be excepted.
- `no_synchronous_launch` also blocks release unless the user supplies an
  exact cell name, data pin name, expected reason, and nonempty justification.
  This can describe a separately reviewed CDC first stage or external input.
- Unknown reasons, duplicate endpoints/exceptions, unused exceptions, and new
  unreviewed endpoints block release. Names are literal, without wildcards.
  Changing an endpoint from `no_synchronous_launch` to an unconstrained clock
  cannot silently reuse an exception.

Example `timing-review.json` (use the names from **your** checkpoint):

```json
[
  {
    "cell": "ff_top.request_toggle_meta",
    "data_pin": "DI",
    "reason": "no_synchronous_launch",
    "justification": "First stage of a toggle synchronizer; CDC review verifies its second stage and source stability."
  }
]
```

```sh
texo pnr path/to/project --package CABGA381 --speed 8 \
  --lpf board.lpf --timing-exceptions timing-review.json \
  --output design.json
texo bitgen design.json --bit design.bit
```

Library callers use `Ecp5FlowOptions::timing_exceptions`. The result owns a copy
and exposes `validate_timing_coverage()` and `meets_timing_closure()`. The shared
`validate_timing_coverage` validator is also used by bitgen. The policy is
applied at sign-off, not to the timing objective used during PnR optimization.

## Checkpoint compatibility

Schema v3 now includes `timing.coverage_exceptions`,
`timing.meets_timing_closure`, and the human-readable `data_pin` on unchecked
endpoint records. An exception does not turn an omitted endpoint into a
check: `all_modeled_endpoints_checked` stays false when exceptions are used.

Bitgen revalidates endpoint records and exceptions instead of trusting the
saved closure flag. It requires the coverage fields already present in old
v3 checkpoints (`unchecked_endpoints` and `all_modeled_endpoints_checked`)
and verifies that the flag agrees with the list. An absent exception list
means no exceptions. Old fully checked checkpoints can still pass. Old
incomplete checkpoints must be regenerated with constraints and any explicit
reviews, including named data pins; a saved `timing_closure` gate is insufficient.
`--allow-unconstrained-io` only controls LPF pin locations and does not bypass
timing coverage.

The public `bitgen()` entry point rejects invalid evidence/coverage before
target-pack lookup/download, configuration output, or the bitstream codec.
`generate_ecp5_config()` validates the same policy for direct API callers.

## Reproducer and hardware motivation

The flow regression
`timing_gate_rejects_an_unconstrained_domain_even_when_checked_paths_pass`
constructs two independent clock domains and runs the real timing engine.
Only the CPU clock is initially constrained: numerical timing passes while
the JTAG register is omitted. Closure is rejected, including when the caller
supplies an old closure gate. Constraining both clocks permits closure;
tightening a period to introduce a setup violation removes it again.

The CLI regressions exercise old checkpoint records, reason changes, new
endpoints, stale/missing exceptions, contradictory coverage flags, and
rejection before runtime lookup or any output. They need no target pack or
FPGA. Run:

```sh
cargo test --workspace --locked
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo test -p texo-cli --example design-specific-flows --locked
```

The motivating Rica RV64 run used Texo
`7b372da4997c924b48cc09327ef006b7af753e65` on an
LFE5UM5G-85F/CABGA381, speed `8_5G`, with 124.8 MHz CPU and 62.4 MHz memory.
Its checkpoint recorded:

| Field | Original failing implementation |
| --- | ---: |
| `met_timing` | true |
| `all_modeled_endpoints_checked` | false |
| Setup / hold checks | 13,902 each |
| Setup / hold worst slack | +14 / +290 ps |
| `unconstrained_clock` endpoints | 519 |
| `no_synchronous_launch` endpoints | 1 |

It carried `timing_closure`, and native bitgen produced an image whose JTAG
ER1 responses stayed zero at 6 MHz, 1 MHz, and 100 kHz TCK. IDCODE remained
readable. With CPU placement/routing and PLL configuration fixed, changing
the JTAG receive shifter to the falling edge restored responses in a
diagnostic framing experiment. The downstream RTL fix also aligned the
JTAG control/data pipeline and removed UPDATE as a separate fabric clock;
normal 128-bit scans then passed 100 distinct hardware ELFs / 134 executions.
Those hardware results validate the downstream RTL repair, not this generic
coverage policy or a new numerical timing model.

Retained artifact SHA-256 identifiers:

- Original checkpoint:
  `2968727d267284680a1f22ada914f8b259d7bfca1d61ed04a805b23cba34198e`
- Original bitstream:
  `f76f09071540e8ffcf637e53f48b061324fcdcb9b87f1fae2ea09afb3af3473e`
- RTL-fixed checkpoint (main STA still omits the JTAG domain):
  `73418831502025fb155a71760eab3365e7d5cddfa3e56e7661757e077c61921c`

Both retained checkpoints are rejected by the new preflight for
`unconstrained_clock`, before runtime lookup and with no output bitstream.
The downstream fixed image additionally uses a project-specific JTCK STA
and hardware qualification. That supplemental analysis is not implicitly
imported by this PR's generic gate.

## What this does not characterize

Coverage here means coverage of **modeled** endpoints. JTAGG launch/capture
arcs, asynchronous reset recovery/removal, and external/CDC timing contracts
still need separate modeling or verification. An explicit exception records
a scope decision; it is not a false-path proof or a new timing check.
The exact JTAGG internal arc and physical hold violation in the motivating
failure were not measured. This failure does not establish that the
nextpnr/Project Trellis delay numbers or Texo's STA arithmetic were wrong:
the failing boundary was outside the reported checks.

No JTAGG delay constants, PLL uncertainty values, clock constraints, or PnR
search behavior are changed here. A characterized JTAGG model and a general
internal-clock constraint interface remain separate follow-up work.
