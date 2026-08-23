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

## Session update (2026-08-24)

| Config | WNS @300 | Runtime |
|---|---|---|
| baseline (handoff) | −352 ps | ~290 s |
| drop second analytical seed + stalled-ripup memo | −352 ps | 269 s |
| budget-excess scoring for local vertex moves | −342 ps | 243 s |
| + single-connection endpoint cells join vertex moves | −287 ps | 344 s |
| + margin-gated route prescreening (commit 49efc28) | −277 ps | ~310 s |
| + incremental congestion tracking, single ripup quantum (**kept**) | **−277 ps** | **245 s** |

Fmax ≈ 276.6 MHz. Runtime profile that drove the last two changes:
98% of the flow is route trials; per-trial breakdown showed `route_net` A*
dominating (~1.3 s even for 14-net releases), full-scan congestion history
~130 ms per trial, and five successful global data-route ripups at 26–38 s
each. The tracker keeps overuse sets incrementally (identical history values,
verified bit-identical final placement), and collapsing
`DETAILED_ROUTING_QUANTA_PS` to `[10]` removed one full renegotiation per
multiresolution round with a bit-identical result — the 1 ps pass contributed
nothing on this design.

Remaining known costs, in priority order for future sessions:
1. Unbounded A* for criticality-1 nets (no corridor) — the long tail of
   0.7–1.6 s small trials.
2. Detailed-quantum transition searches turn arrival into part of the A*
   state, multiplying visited states for the released critical nets.
3. Global ripups renegotiate all 2350 nets when only the failing region
   contends; targeted ripup would trade QoR risk for most of the remaining
   ~110 s.

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

## Why nextpnr feels fundamentally different (source-verified 2026-08-24)

Cloned nextpnr master (`/tmp/opencode/nextpnr-src`, sparse: common+ecp5).
The intuition is correct; the difference is one architectural idea Texo lacks,
not tuning:

1. **Anchored iterative re-solve** (`common/place/placer_heap.cc:961-971`):
   every HeAP iteration re-solves the QP with an extra arc pulling each cell
   toward its previous *legalised* position, weight `alpha*iter/dist`
   (alpha default 0.1). The placement evolves continuously across
   iterations, so updated timing weights never jump basins.
   `while (stalled < 5 && solved_hpwl <= legal_hpwl * 0.8)` (line 249) runs
   solve → CutSpreader → strict legalisation → `tmg.run()` per iteration.
   Our v9 replace-style re-solve failed precisely because it had **no anchor**
   and jumped basins; this is the missing piece, not weights themselves.
2. **Estimated STA every iteration** (`tmg.run()` → TimingAnalyser over
   `ctx->predictArcDelay`): slacks come from `Arch::estimateDelay`
   (`ecp5/arch.cc:465`): `(80-9*speed) * (6 + max(dx-5,0) + max(dy-5,0)
   + 2*(min(dx,5)+min(dy,5)))` — fixed overhead plus *double rate for the
   first five tiles*, i.e. long lines make far tiles relatively cheap. Our
   linear Manhattan prescreen model has the wrong shape, which is why the
   unguarded filter lost 388 ps (v11).
3. **Normalised criticality** (`common/kernel/timing.cc:778-786`):
   `crit = clamp(1 - (slack - worst)/(-worst), 0, 1)` per port, fed back as
   multiplicative weight `(1 + timingWeight * crit^exponent)`
   (placer_heap.cc:946). Bounded [1, 1+tw], unlike our urgency weights that
   span 1..64+.
4. **Estimate-driven detail moves**: `timing_opt.cc:51` runs 30 rounds of
   tmg.run() → find_crit_paths(0.98, 50000) → cell swaps scored purely by
   estimated delays, no routing. Thousands of free evaluations.
5. **router2 feedback** (`common/route/router2.cc:1735-1777`): after every
   `do_route()`, real routed arc delays are pushed into the same
   TimingAnalyser (`update_route_delays` → `tmg.set_route_delay`), nets are
   re-sorted by criticality, and arcs failing slack get ripped up next round.

Net effect: nextpnr evaluates timing tens of thousands of times per run at
~microsecond cost (geometry estimates), while Texo evaluates it ~170 times at
~1.5 s cost (full route trials). Every weighting experiment we ran bolted
weights onto the expensive pipeline instead of building the cheap loop.

### Port plan (priority order)

a. Anchor term in `analytical_place`: accept an optional previous legal
   placement; add diagonal/rhs contributions `alpha*iter/dist` toward it.
b. Iterate solve→spread→legalise→`estimate_placement_timing`(rebuild with the
   arch.cc-shaped model)→normalised-criticality weights until HPWL stalls;
   descend refinement from the result once. This replaces the failed
   replace-style experiment with its missing ingredient.
c. Reshape `estimate_edge_delay` to the double-rate-near/far form before any
   further prescreen tuning.
d. Optional later: router-level slack-failure ripup using real delays.

### Port result (same day): experiment 7, negative

Implemented a–c faithfully (AnalyticalAnchor in texo-pnr stamped after the
plain solve with drift-weighted attraction; DelayEstimator with
overhead/near-knee/far shape; normalised criticality weights `1 + 10·crit²`;
loop of up to 8 anchored rounds, stall-2 stop; single route trial gated by
real score). Result: **WNS −441 ps vs −277 ps baseline** at ~230 s. The
anchored candidate won the early routed comparison and refinement descended
from it into a worse final state — the same failure signature as the v9
replace-style run, now with all nextpnr ingredients present.

Interpretation: on this design class (1.6% utilisation, carry-dominated,
many parallel decoder paths) the connectivity-only global placement plus our
greedy refinement is genuinely better than any criticality-informed global
solve we have built — including one structurally faithful to HeAP. What
nextpnr has that we still do not is the *estimate-driven detail* stage
(timing_opt.cc: 30 rounds of crit-path cell swaps scored by estimates) and
router2's per-round real-delay feedback; those act on an already-good
placement instead of replacing its seed. Any further work should target that
layer, not the analytical solve.

### Experiment 8 (detail-layer turnover): negative, reverted

Tried exactly that detail layer: rank each cell's vertex proposals by the
criticality-weighted geometric estimate and route only the top-N (N=2/3,
candidates widened 4→8), plus first-improvement early exit in local-
connection refinement. Result: WNS −293 ps (−16 ps vs baseline), runtime
roughly unchanged under load; disabling either half still gave −293. The
geometric estimate does not reliably identify which proposal wins after full
routing — consistent with every other estimate-based result on this design.
The committed 25%-margin prescreen remains the right amount of estimation:
it only removes clearly-hopeless trials and never reorders or substitutes.

Conclusion for future sessions: on the AXI4 self-test, estimates can veto but
not rank. A real turnover gain needs a faster *routed* trial (router-level
work: bounded search for criticality-1 nets, targeted ripup), not smarter
pre-route scoring.

### Experiment 9 (plain-net search bounding): neutral, reverted

Extended the corridor to criticality-1 nets (`PLAIN_ROUTE_CORRIDOR_MARGIN`).
Margin 4: WNS −301 ps and slower — frequent fallbacks double-searched, and
the old costs-less retry lost delay awareness. Margin 10 with a
cost-preserving unbounded retry: final placement bit-identical to baseline,
no measurable speedup. Two findings: (1) plain nets were never the small-
trial bottleneck — their Manhattan-guided sweeps already stay local at this
utilization; (2) the real cost center of ~1 s small trials is the
detailed-quantum transition search on released critical nets, where arrival
becomes part of the A* state (`routing_transition_cost`, state =
(wire, distance, arrival)) and multiplies visited states. Future router work
should target that state space (coarser arrival buckets on non-failing
sinks, or per-sink quantum escalation) and targeted ripup; plain-net
bounding is a dead end here.

### Experiment 10 (detailed-quantum state space): positive, kept (commit 45de64b)

Confirmed the diagnosis by ablation: running vertex-pass detailed searches
at the default 50 ps quantum instead of 10 ps left WNS and hold identical
(−277/−381) while cutting median trial routing from 0.92 s to 0.67 s.
Vertex passes now use `ROUTING_DELAY_QUANTUM_PS` (now public); the
multiresolution ripup keeps its fine quantum since it touches only failing
sinks' nets. Flow: **−277 ps @ 232 s** (from −352 ps @ ~290 s at session
start). Next router targets, in order: per-sink quantum escalation for the
multiresolution ripup's detailed nets, and targeted ripup scoped to nets
contending with the failing region.

### Experiment 11 (targeted ripup): negative, reverted

Scoped the multiresolution ripup to nets whose routes pass within the
failing endpoints' bounding rectangle (+4, then +12 tiles): WNS −346 ps
(−69 vs baseline) at ~30 s faster, identical for both margins. The failing
endpoints span the whole decoder array, so widening the region changes
nothing — the value of the global renegotiation is renegotiating *outside*
the failing region too, under fresh criticalities and history, not local
congestion relief. Global data-route ripup stays.

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
