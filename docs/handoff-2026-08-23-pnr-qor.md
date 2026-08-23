# Handoff: PnR QoR work against nextpnr (2026-08-23)

## Objective

Close the Fmax gap between Texo and nextpnr on the AXI4 self-test design
(LFE5UM5G-85F, CABGA381, speed grade 8), while keeping every step measured
and deterministic. All numbers below are from this machine; treat them as
relative indicators, not absolute claims.

## Benchmark environment

```sh
# architecture cache (748 MB, regenerate only if prjtrellis changes)
ls /tmp/texo-LFE5UM5G-85F-final.txdb

# Texo P&R at a given target (LPF FREQUENCY drives STA targets)
TEXO_METRICS=1 cargo run --release -- axi4-pnr \
  /tmp/texo-LFE5UM5G-85F-final.txdb CABGA381 8 <lpf> [checkpoint.json] [weight-exp]

# nextpnr reference (flags mirror struo CI; timingweight raised to 40 there)
nextpnr-ecp5 --um5g-85k --package CABGA381 --speed 8 \
  --json <mapped.json> --lpf <lpf> --freq 300 \
  --placer-budgets --placer-heap-timingweight 40 --tmg-ripup \
  --timing-allow-fail --seed N

# current-synthesis netlists (struo @ 62a57f3f45935829320fd66e533597c9a146e452)
/tmp/opencode/struo-build/design-pulled.json   # lossless since PR #35
```

Gotchas learned the hard way:

- `--um-85k` is LFE5UM; UM5G needs `--um5g-85k`. Wrong device silently
  produces ~210 MHz results.
- Without `--timing-allow-fail`, nextpnr aborts after placement and prints a
  placement estimate, not a routed result.
- An LPF `FREQUENCY PORT` overrides `--freq` for the target check and
  therefore for router effort. Compare tools under identical constraints.
- `perf` does not work on this WSL kernel; use `TEXO_METRICS=1` stage lines
  and `/usr/bin/time -p` instead.
- The old `/tmp/texo-axi4.json` predates Struo PR #35 and is missing four
  replicated LUTs; do not use it for QoR comparisons any more.

## Scoreboard (identical current netlist, 300 MHz target)

| Tool | Fmax | Notes |
|---|---|---|
| Texo (budgets + global ripup) | ≈271 MHz | WNS −352 ps, deterministic |
| nextpnr (CI flags, tw=40) | ≈308–310 MHz | seeds 1–3 within ±1 MHz |

At the 250 MHz target Texo closes (+7/+9 ps) in about 112 s, down from
160 s at session start.

## Session update (2026-08-24, uncommitted)

| Config | WNS @300 | Runtime |
|---|---|---|
| baseline (handoff) | −352 ps | ~290 s |
| drop second analytical seed + stalled-ripup memo | −352 ps | 269 s |
| budget-excess scoring for local vertex moves | −342 ps | 243 s |
| + single-connection endpoint cells join vertex moves (**kept**) | **−287 ps** | 344 s |

Fmax ≈ 276 MHz with the kept configuration. Remaining critical path:
decoder FF → carry cluster feed (~890 ps over 3–4 tiles), carry hops free,
cluster → LUT → retimed FF (~650+390 ps).

## Commits (oldest first)

| Commit | Content |
|---|---|
| `450c214` | Routing/placement constant factors: pin→wire cache, exact ring scan, precomputed routing order, shared tree-arrival scratch, epoch-stamped tree membership in the A* loop (160 s → 135 s) |
| `d9b3f41` | Deterministic basin escape (`escape_placement_basin`) when all phases stall with negative setup slack |
| `0b92d50` | Placement weight exponent as `Ecp5FlowOptions` field + optional CLI arg, recorded in checkpoints; TEXO_METRICS stage lines |
| `ed9db57` | Struo pin → c58e76c (no output change) |
| `68a79eb` | Routed-delay budgets driving placement refinement (see below) |
| `2210946` | Global data-route ripup during detailed rerouting (−451 → −352 ps) |
| `0c3d548` | Seed-winner diagnostics under TEXO_METRICS |
| `e2b148c` | Struo pin → 62a57f3 (PR #35 unique retimed names) + test invariant update |

## Established findings

1. **Wirelength proxies do not predict final WNS here.** Sweeping the
   placement weight exponent changed initial HPWL monotonically but final
   timing quality was uncorrelated (best-HPWL variant failed, worst passed).
2. **Delay-objective changes do transfer**: routed-delay budgets (+31 ps) and
   global route ripup (+99 ps) both moved the 300 MHz WNS.
3. **The timing-driven analytical replacement never wins archive selection**
   (initial −6123 vs timed −7248 ps). Sharpening its weights did nothing
   downstream because the candidate loses before refinement. Clustering by
   criticality hurts at 1.6% utilization; wirelength-dominant solves win
   after routing. This kills the "just tune analytic weights" path and
   motivates blending criticality into one solve instead of two competing
   seeds.
4. **Struo's JSON export used to drop replica cells** behind name collisions
   (fixed upstream in PR #35). Always verify `JSON cells == mapped.cells()`.
5. Critical path structure at 300 MHz: two general-routing nets into/out of a
   carry cluster (~712/855 ps over 2–3 tiles) dominate; carry hops are free.

## Failed experiments (patches kept, not merged)

- Bound2bound reweighting alone: `/tmp/opencode/b2b-experiment.patch`
  (WNS +5 ps vs +26 ps baseline, 297 s).
- HeAP-style anchored spreading: `/tmp/opencode/heap-anchor-experiment.patch`
  (WNS −453 ps; uniform-box anchors fight connectivity at low utilization).
- Threshold acceptance inside descent loops: reverted same-day; accepting
  regressions derailed the greedy trajectory (−15 ps / 287 s). Kept instead:
  gated basin escape after all phases stall.
- Budget-excess scoring for *broad* (distance-16) vertex moves: unbounded A*
  per candidate blew past 15 min; two-stage span-pre-ranking variant ran at
  −342 ps / 276 s — no QoR over span-only, +33 s. Reverted.
- `MAX_CRITICAL_PATH_CELLS` 6→12: same WNS (−287 ps), 436 s. The extra cells'
  route trials are rejected; coverage is not the limiter.
- Batched endpoint pulls (`refine_connection_delay` per worst edge, one route
  trial for the batch): same WNS (−287 ps), 445 s; greedy estimated-delay
  acceptance moves cells whose full-route outcome regresses TNS elsewhere.
- Static moderate weights on carry-adjacent nets in the initial analytical
  solve: WNS −509 ps / 337 s. Confirms finding 3 a third time: keep the
  initial solve connectivity-only at this utilization.
- Replace-style iterative timed re-solves before refinement (weights from the
  routed incumbent, contrast capped at 8, iterate while routed WNS improves,
  up to 4 rounds): WNS −329 ps / 391 s. The phase-0 gate compares against the
  *unrefined* initial candidate, so a mediocre early improvement replaces the
  good basin that plain-seed refinement would have descended from. Every
  weighting-based seed variant has now lost to the connectivity-only descent;
  the structural difference vs nextpnr is its unified placer applying timing
  weight continuously during spread/legalization, not a solve we can bolt on.
- Estimated-STA feedback inside the solve (new `estimate_placement_timing` in
  texo-timing: Manhattan×250 ps + 300 ps per net edge, shared STA machinery),
  iterating while *estimated* score improves and gating the final weighted
  placement by real routed score: WNS −287 ps (identical basin) but +205 s.
  Estimated-timing ranking does not transfer to routed outcome either —
  finding 1 extends to placement-based delay estimates at this utilization.
  Reverted; the gate itself worked (no QoR regression).

**Verdict after five weighting variants (routed weights as competing seed,
routed weights as replacement, static carry weights, capped blend iteration,
estimated-STA iteration): on this netlist at 1.6% utilization every
criticality-weighted analytical solve lands in a worse basin than pure-HPWL
descent. Closing the remaining −287 ps needs a genuinely unified iterative
placer (spread/legalization interwoven with timing weight) or denser-design
evidence before more density-side work — not another reweighting of the
existing solve→refine pipeline.**

## Next steps (priority order)

1. **Blend criticality into one analytical solve** instead of two competing
   placements. Concretely: fold moderate sink weights into the single
   initial solve (or drop `optimize`'s second seed entirely and save ~20 s),
   mirroring nextpnr's single-solve timingweight blend. Code:
   `TimingDrivenContext::optimize` / `timing_driven_placement` /
   `timing_placement_weights` in crates/texo-flow/src/lib.rs;
   `analytical_place` in crates/texo-pnr/src/lib.rs.
2. **Ripup cost control**: the global ripup stage costs ~200 extra seconds.
   Gate later rounds (skip when the previous round's trial regressed), or
   collapse `DETAILED_ROUTING_QUANTA_PS` to a single quantum.
3. **Hold repair gating**: hold repair only runs for setup-clean archive
   entries, so −517 ps hold remains whenever setup fails. Decide whether
   hold-critical closure needs it earlier.
4. **More benchmarks** via the new `struo qor <veryl> <top>` command
   (uncommitted in ~/develop/struo working tree): include one high-utilization
   design before judging density-side work (ePlace-style) — density was
   irrelevant at 1.6% utilization.
5. Optional: port the budget objective into `refine_critical_path_cells`
   proposal scoring so vertex moves also respect allowances.

## Reproducing today's key measurements

```sh
# budgets/ripup effect at 300 MHz (expect WNS −352 ps, ~290 s)
TEXO_METRICS=1 target/release/texo axi4-pnr /tmp/texo-LFE5UM5G-85F-final.txdb \
  CABGA381 8 /tmp/opencode/lpf-300.lpf

# seed-selection diagnostic (expect seed=initial to always win)
... 2>&1 | grep 'seed='
```

`/tmp/opencode/lpf-300.lpf` is the 250 MHz LPF with the frequency edited to
300 MHZ. Regenerate it from examples/axi4-self-test/ if missing.
