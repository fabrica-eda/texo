# Reuse routed physical implementation

`texo pnr --resume-checkpoint previous.json` accepts the placement, LUT/FF
pairs and routed trees of a schema-3 checkpoint as starting constraints.
Pass the same RTL, top, mapping settings, LPF, PLL, clock constraints and
timing exceptions as for the original run. Use a different output path.
The saved timing report is not an input to timing sign-off.

The importer checks device/package and architecture revisions, reconstructs
packing pairs by cell name, and compares every driver's placed physical pin
and each PIP's identity, endpoints and direction with the current architecture.
It rebuilds sink paths from the current netlist. Duplicate or stale records,
disconnected sinks and removal of mandatory target routing are errors. The
normal router checks occupancy/connectivity and full setup/hold STA runs again.
This is for compatible mapped designs; it is not a general RTL-change ECO API.

Resumed runs skip global placement feedback and preserve routes for local
setup repair. After a strictly improving local placement change, route ECOs
are reopened against the changed physical state. These passes alternate
until closure or a fixed point, instead of requiring another complete
synthesis/checkpoint round trip to reconsider routes.

## Fixed-clock ECP5 measurement

The RV64 compact/store design retained CPU 124.8 MHz, memory 62.4 MHz,
6 MHz JTCK and its four explicit boundary exceptions:

| Result | Qualified input | Repaired output |
| --- | ---: | ---: |
| Nominal setup WNS | +6 ps | +209 ps |
| Additional setup uncertainty | 0 ps | 200 ps |
| Reported guarded setup WNS | +6 ps | +9 ps |
| Hold WHS | +290 ps | +290 ps |
| Setup / hold checks | 14,437 each | 14,437 each |

The ordinary bitgen output SHA-256 is
`f035a971936594e5c67d0c25ae14869e5daf6b878afdc0200a52cea42c44727f`.
It passed 105 physical ELF programs and 28 diagnostic repeats (133 executions).
The final import guards reproduced the identical bitstream. Native and
supplemental JTCK STA agree on all 515 setup and 515 hold slacks/edges.
Existing JTAGG/CDC characterization limits remain; no clock, delay model or
timing coverage rule was relaxed.

These measurements use Struo `eff404d` / Veryl 0.20.3 with the resume changes.
Rica records complete commands, input hashes, checkpoints, bitstreams and
logs under `toolchain/measurements/2026-09-06-rv64-load-margin-m/`.
The additional reserve is a design target, not a physical PVT guarantee.
