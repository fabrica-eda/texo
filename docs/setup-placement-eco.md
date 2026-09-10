# Local setup placement repair

After route ECOs and global placement feedback stall on negative setup slack,
Texo proposes moves for cells in the worst failing cones. It ranks cells by
slack and realized incident route delay, tries local characterized-delay moves
before broad span reductions, and returns to route repair after each accepted
move. The current [incremental closure flow](incremental-closure.md) also uses
measured incumbent delays and distinct-tile broad shortlists. The historical
RV64 store-latency result below stopped with WNS -706 ps before these passes.

Each trial retains routes whose endpoints and physical pin bindings remain
unchanged. Moving a driver releases its net; moving a sink releases its branch
and any sibling branch sharing the released PIPs. Global clocks are re-resolved.
A legal route and full setup/hold STA are required before the existing strict
objective can commit a trial. Rejected trials retain the incumbent physical
implementation. Repeated complete placement proposals are skipped. This pass
runs only when timing optimization is enabled and setup still fails.

The ECP5 LFE5UM5G-85F/CABGA381/speed-8 RV64 candidate closed at 124.8 MHz CPU,
62.4 MHz memory and 6 MHz JTCK after 29 accepted local moves:

| Result | Before local moves | After local moves |
| --- | ---: | ---: |
| Setup WNS | -706 ps | +6 ps |
| Hold WHS | +290 ps | +290 ps |
| Checked setup/hold endpoints | 14,437 each | 14,437 each |
| Explicit endpoint exceptions | 4 | 4 |

Ordinary bitgen produced SHA-256
`e325997fb3b7c4d283d4e2d2ad192b2c0a550639ce8acd4cfc953f210136b942`.
That exact image passed 100 distinct physical RV64I/Zca/basic/diagnostic ELFs,
118 executions with diagnostic repeats. A build from the finalized source
reproduced the same bitstream. Native and supplemental JTCK STA agreed on all
515 setup and 515 hold slacks and clock edges; the existing JTAGG/CDC boundary
limitations remain. No timing model, clock constraint or exception was relaxed.

This adds search time after the earlier flow exhausts its candidates. Recorded
end-to-end wall time was approximately 495 seconds for the finalized run; the
original run was approximately 192 seconds. Other validation ran concurrently,
so these times are provenance, not a controlled runtime benchmark.

Validation: 68 `texo-flow` library tests, including route retention, coupled-branch
release and incumbent immutability; package Clippy with `--no-deps` and all targets;
workspace formatting. Rica retains the full PNR arguments, sources, trial logs,
checkpoint, bitstream and physical samples in
`toolchain/measurements/2026-09-06-rv64-lsu-refill/`.
