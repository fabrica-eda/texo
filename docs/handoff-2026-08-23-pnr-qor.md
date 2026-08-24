# Handoff: PnR QoR work against nextpnr (2026-08-23)

## Objective

Close the Fmax gap between Texo and nextpnr on the AXI4 self-test design
(LFE5UM5G-85F, CABGA381, speed grade 8), while keeping every step measured
and deterministic. All numbers below are from this machine; treat them as
relative indicators, not absolute claims.

## 2026-08-24 arc-owned routing redesign

The aggregate `NetRoute { wires, pips }` representation was replaced outright
with ordered driver-to-sink `RouteArc`s. Compatibility was intentionally not
preserved. `NetRoute` now derives sorted wire/PIP reference counts from its
arcs, so removing one sink decrements shared resources and releases them only
when their last arc reference disappears.

Negotiated routing now builds sparse wire/PIP reverse-owner indexes for
overused resources. Capacity arbitration ranks the actual owner arcs by
per-sink timing criticality, keeps locked architecture arcs, and requeues only
the lower-criticality conflicting arcs. If a released arc shares a PIP trunk
with sibling arcs of the same net, those structurally coupled siblings are
released transitively; otherwise the rest of the net stays bound. STA no
longer reconstructs sink paths from an aggregate undirected tree and instead
sums each canonical ordered arc directly.

Measured on the same 300 MHz AXI4 benchmark:

| router | runtime | setup WNS | hold WHS | PIPs |
|---|---:|---:|---:|---:|
| aggregate-tree baseline (`103ad88`) | 124.46 s | -277 ps | -381 ps | 29,488 |
| strict conflicting-arc-only | 51.24 s | -496 ps | -300 ps | 29,656 |
| arc owner + shared-PIP coupled victims | 102.28 s | **-266 ps** | -533 ps | 29,651 |

The strict branch-only policy proves that arc ownership removes most of the
runtime, but freezing a sibling's shared parent trunk can trap the critical arc
on a slow topology. Releasing only the transitive shared-PIP component recovers
setup QoR while remaining about 18% faster than the aggregate-tree baseline.
Hold QoR regressed and remains separate follow-up work; the flow currently
gates hold repair on setup closure.

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

## Runtime update (2026-08-24, full-ripup circuit breaker)

Re-ran the current 300 MHz AXI4 benchmark while comparing the installed
nextpnr ECP5 default router against both router1 and router2 source. The
dominant remaining structural difference is that nextpnr owns a persistent
arc queue and routing state: it routes 6536 connections in 0.99 s and rips up
the seven negative-slack arcs in place. Texo instead evaluates placement moves
by rebuilding routed candidates and periodically renegotiates all 2350 data
nets.

One attempted router2-style fast path retained satisfied branches and released
only negative-slack sinks. It cost 2--5 s per closure round, regressed timing
every time on this sparse design, and was reverted. The useful measurement was
that after the first successful full-chip ripup, three later full-chip ripups
all regressed. Each local placement improvement had re-enabled the same
18--19 s failed search.

Kept change: a failed global ripup now stays stalled until placement WNS has
improved by 250 ps (approximately one measured general-routing tile), instead
of re-arming after any placement change. On the deterministic benchmark this
removes two redundant full negotiations:

| Build | Runtime | WNS / hold | PIPs |
|---|---:|---:|---:|
| preceding measured trial | 214.32 s | −277 / −381 ps | 29488 |
| ripup re-arm threshold | **159.20 s** | **−277 / −381 ps** | **29488** |

The immediately preceding trial included about 12 s from the rejected
connection-only experiment, so the isolated circuit-breaker gain is roughly
38 s (about 19%); the observed end-to-end reduction is 25.7%. The same
nextpnr command completed in 3.14 s and routed at 309.60 MHz, confirming that
this change removes waste but does not close the architectural runtime gap.
The next large step remains a persistent connection router with uphill and
downhill indexes, an arc priority queue, in-place conflict ripup, and routed
delay feedback; more full-route placement trials cannot approach nextpnr's
cost model.

## Runtime update (2026-08-24, reusable routing workspace)

The timing-driven loop previously rebuilt device-sized scratch storage for
every placement trial. On the 85K device that meant repeatedly allocating and
zeroing occupancy/history arrays for 3,798,913 wires and 29,116,611 PIPs, plus
the A* search arrays and route-tree arrival array. This work was independent of
the handful of nets released by most incremental trials.

Kept change: `RoutingWorkspace` now owns those arrays for the full flow. It
clears occupancy and history through touched-index lists, resets the existing
generation-stamped A* workspace in place, and is passed through every initial,
incremental, and full-ripup route call. The original public routing entry
points remain as allocating compatibility wrappers; workspace-aware entry
points serve repeated callers. A regression test routes the same placement
twice through one workspace and verifies that no occupancy leaks between
trials.

| Build | Runtime | user / sys | WNS / hold | PIPs |
|---|---:|---:|---:|---:|
| ripup re-arm threshold | 159.20 s | 148.57 / 10.44 s | -277 / -381 ps | 29488 |
| + reusable routing workspace | 149.84 s | 147.77 / 2.39 s | -277 / -381 ps | 29488 |
| + persistent A* frontier | 148.76 s | 146.75 / 2.30 s | -277 / -381 ps | 29488 |
| + criticality-1 routing corridor | 147.52 s | 145.48 / 2.37 s | -277 / -381 ps | 29488 |
| + 32-bit downhill adjacency | 145.22 s | 143.75 / 1.77 s | -277 / -381 ps | 29488 |
| + 12-byte physical PIPs | 136.62 s | 135.03 / 1.84 s | -277 / -381 ps | 29488 |
| + SoA routing hot metadata | **122.32 s** | **120.81 / 1.79 s** | **-277 / -381 ps** | **29488** |

This removes another 9.36 s (5.9%) end to end and cuts kernel/system time by
77%, with identical deterministic QoR and route size. Small incremental route
trials that previously took roughly 57--90 ms now commonly take 15--25 ms;
the remaining 0.5--0.9 s trials are dominated by A* search rather than scratch
initialization. Relative to the 214.32 s measured trial before the two kept
runtime changes, the workspace alone gives a 30.1% cumulative reduction.

Two follow-up constant-factor changes also stayed bit-identical. The A*
`BinaryHeap` now lives in `RouteSearch`, so a large critical search retains
its frontier allocation for later connections instead of freeing and
regrowing it. Timing nets with the minimum nonzero criticality now get the
same bounded corridor as all other timing nets; an unsuccessful corridor
still falls back to the original unbounded search. Together these remove a
further 2.32 s (1.5%), for a 31.2% cumulative reduction from 214.32 s.

The next kept series ports another important nextpnr/chipdb property: dense
physical IDs use compact storage, and the router's hot numeric data is
separate from names and other cold metadata. `Device::routing_neighbors`
previously stored every one of the 29,116,611 downhill arcs as two 64-bit
Rust IDs (16 bytes); it now stores two 32-bit IDs (8 bytes). `Pip` similarly
shrinks from 24 to 12 bytes. Custom serde continues to encode the historical
`(usize, usize)` adjacency tuples and named PIP fields, verified by loading the
existing 715 MiB architecture cache unchanged. The serialized file therefore
does not shrink yet, but the in-memory graph loses about 555 MiB.

Finally, A* no longer randomly fetches coordinates and capacities from large
`Wire` records that also contain names, or from the full PIP records. The
flow-persistent `RoutingWorkspace` owns structure-of-arrays copies of wire
points, wire capacities, and PIP capacities (about 92 MiB), leaving a net
runtime-memory reduction around 464 MiB while making the search working set
contiguous. This last change alone removes 14.30 s (10.5%). The complete kept
series is 122.32 s: 23.2% faster than the 159.20 s circuit-breaker build and
42.9% faster than the 214.32 s preceding measured trial, with bit-identical
placement, route, WNS, hold, and PIP count.

Attempted and reverted in between: a compact uphill CSR plus reverse A* from
each sink. Some individual searches became faster, but choosing a different
equal/near-equal connection path changed the negotiated-routing basin. WNS
progress lagged badly and small trials grew to 2.3 s after global ripup. A
useful nextpnr-style bidirectional router therefore cannot be bolted onto the
current stateless net rebuild: it needs persistent per-connection route state,
conflict ownership, and in-place arc ripup as one coherent change.

## QoR-first follow-up (2026-08-24)

The objective is not to freeze QoR while optimizing runtime. Runtime is search
budget that must ultimately raise Fmax/QoR. Several nextpnr-inspired ways of
spending that budget were measured after the 122.32 s router build. None beat
the incumbent, so all were reverted:

| Experiment | Runtime | WNS @300 | Result |
|---|---:|---:|---|
| aggregate score reordered WNS-first | 140.00 s | -470 ps | worse basin and slower |
| per-sink criticality + critical-first tree + fine sinks | 119.99 s | -381 ps | faster trials, worse tree |
| fine quantum only on failing sinks | 100.82 s | -361 ps | 21 s saved, 84 ps lost |
| failing sinks first, then coarse fanout | 90.27 s | -346 ps | recovered 15 ps, still worse |
| preceding build + two closure rounds | 95.65 s | -346 ps | extra search was neutral |
| aggregate improvement with a 25 ps WNS-regression gate | 94.55 s | -346 ps | blocked necessary turnover |
| fully restored incumbent (confirmation run) | 124.46 s | **-277 ps** | -10,235 ps TNS, 29,488 PIPs |
| deterministic net-order portfolio after a failed ripup | 144.93 s | -277 ps | alternate order added ~20 s, identical route |
| retained-tree setup/min-corner arrival reconstruction | 158.86 s | -277 ps | inactive on setup's whole-net releases |
| exact worst-arc-only ripup | 129.06 s | -277 ps | frozen resource owners made the retry identical |
| early critical-corridor victims (128 nets) | 136.56 s | -278 ps | changed basin; hold -372 ps, no Fmax gain |
| corridor victims gated to WNS >= -400 ps | 144.13 s | -277 ps | preserved basin; -323 ps candidate rejected by STA |

The router2 comparison remains useful, but arc criticality, critical-first
ordering, and per-arc ripup form one coupled design. Texo's net-level route
tree shares earlier sink branches with later sinks; changing only arc order or
delay resolution therefore changes the placement-refinement basin rather than
reproducing router2. The next QoR attempt should introduce persistent
per-connection ownership and in-place conflict/timing ripup first, then add
bidirectional arc search and per-arc criticality on that state model. The
current aggregate timing objective must also remain able to accept temporary
WNS regressions: both strict WNS ordering and a small Pareto-style gate lost
the deterministic -277 ps basin.

The arc/victim experiments sharpen that conclusion. Arc-only ripup cannot
evict a fast-resource owner, while preselecting victims by a geometric
corridor either perturbs the placement trajectory too early or produces a
worse late candidate. The missing router2 mechanism is dynamic resource
ownership: route the critical arc, discover the actual conflicts, rip up
those owning arcs in place, and iterate without rebuilding unrelated net
trees. Static victim selection is not an adequate substitute.

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
1. Forward-only A* remains the long tail: some 14--40-net trials still take
   0.4--0.7 s even with bounded timing corridors. Do not reintroduce reverse
   A* alone; the next router-level step is persistent per-connection state,
   resource ownership, and in-place conflict ripup, then bidirectional search.
2. Detailed-quantum transition searches make arrival affect the A*
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

## Initial-placement A/B and density correction (2026-08-24)

The nextpnr ECP5 default remains the flat `router1`; hierarchy is not the
explanation for its roughly 3.1 s result. Its no-route run places this design
in about 1.7 s with an estimated 386.25 MHz result, so placement was isolated
from routing directly instead of inferred from end-to-end timings.

`axi4-route-nextpnr-placement` now imports `NEXTPNR_BEL` assignments from a
nextpnr placed JSON, translates ECP5 coordinates and split carry names, imports
its selected dedicated LUT/FF pairs, completes Texo-only synthetic carry
members, and routes the fixed placement without placement closure. This is an
A/B diagnostic command rather than a production dependency on nextpnr.

On the same AXI4 design:

| Placement routed by Texo | HPWL | initial WNS | timing-rerouted WNS |
|---|---:|---:|---:|
| old native analytical placement | 262,967 | -5,808 ps | -3,196 ps |
| fixed nextpnr placement | 233,107 | -2,809 ps | -1,022 ps |

Thus weak initial placement accounts for roughly 2.17 ns of the initial
timing-route gap. It is not the whole QoR gap: even with nextpnr's placement,
Texo still leaves same-tile connections on 999--1,292 ps routes in some
trials, so route-topology choice and architecture timing remain independently
material.

The concrete placer defect was that analytical spreading treated one
placement unit as a complete physical coordinate. An ECP5 logic tile has
multiple compatible LUT/FF slots. The new spread derives per-coordinate
capacity from legal assignments and targets 3/8 occupancy, while legalization
stops after the first legal distance ring. The quadratic equations now also
retain every atomic macro member's offset; carry chains no longer collapse to
the representative cell during the solve.

Measured density sweep (all use the macro-offset correction):

| target units/tile | initial HPWL | final WNS / WHS | PIPs | runtime |
|---:|---:|---:|---:|---:|
| 1 | 254,656 | -461 / -524 ps | 29,720 | ~100 s |
| 2 | 242,588 | -372 / -375 ps | 26,700 | slower than 3 |
| **3 (selected)** | **236,937** | **-318 / -397 ps** | **25,734** | **70.76 s** |
| 4 | 233,432 | -456 / -562 ps | 24,594 | ~50 s |
| 6 | 230,761 | -598 / -544 ps | 23,820 | ~40--50 s |

The selected density reduces runtime about 31% and route size about 13%
relative to the arc-router baseline (102.28 s, 29,651 PIPs), while setup WNS
regresses 52 ps from -266 ps. This is therefore a speed/structure improvement,
not yet the requested setup-QoR win. More compaction monotonically improves
HPWL but worsens routed WNS, proving HPWL is not a sufficient placement graph
objective.

Finally, disposable local ECO route trials now stop after eight negotiated
iterations while whole-design routing keeps the 32-iteration budget. Accepted
candidates on this benchmark converge in three to five iterations; the old
32-iteration tails were infeasible candidates that were rejected afterward.
The bounded run reproduced the exact -318/-397 ps, 25,734-PIP result. Its
isolated gain was small here, but it prevents pathological failed candidates
from consuming the search budget needed for future QoR work.

## Routed-topology-aware dedicated-edge placement ECO (2026-08-24)

HPWL is no longer used to choose among competing ordinary LUT-to-FF dedicated
edges. After the first route and STA, the flow constructs the relevant part of
the timing-aware placement graph from per-sink routed delay and propagated
setup slack. A vertex is a placement unit; an edge is a logical sink arc with
its current physical route. The most critical unselected LUT-to-FF edge is a
bounded discrete proposal:

1. transfer the LUT's dedicated `F -> DI` edge to that FF;
2. move the displaced FF to the candidate's old BEL and rebind it to `M`;
3. freeze unaffected sink arcs, preserving their actual occupied topology;
4. reroute only nets incident to the two swapped FFs under timing costs; and
5. keep the packing/placement mutation only if full setup/hold STA improves.

This is intentionally one actual-routing trial, not a broad unloaded-shortest-
path legalizer. The latter concentrated critical cells onto the same fast
resources without modeling interference and was both slower and worse. The
same-kind detailed-placement swap rebuilds only target pin bindings and checks
the affected atomic group; it reduced the AXI4 proposal construction from
about 3 seconds per candidate to about 16 ms after the first cached check.

Measured on the identical 300 MHz AXI4 input:

| flow | final WNS / WHS | PIPs | runtime |
|---|---:|---:|---:|
| density baseline | -318 / -397 ps | 25,734 | 70.76 s |
| topology-aware dedicated-edge ECO | **-254 / -397 ps** | **25,660** | 84.00 s |

The selected `lut251: ff2437 -> ff2439` transfer improved initial TNS by
990 ps with unchanged initial WNS, then led timing closure to a 64 ps better
setup endpoint and 74 fewer PIPs. The ECO trial itself cost about 1.05 seconds;
the remaining runtime increase comes from the subsequent closure trajectory
exploring the improved packing basin more deeply. A post-closure-only variant
finished in 74.30 seconds but rejected all four candidates and retained
-318/-397 ps, so it was not selected.

## Persistent routing transactions (2026-08-24)

`RoutingWorkspace` now retains the last successfully routed net trees together
with the occupancy they contribute. Before a new local placement/packing
trial, its frozen routes are compared with that resident snapshot per net:

- unchanged trees retain their occupancy without being rebuilt;
- removed or replaced trees decrement only their resource-set difference;
- new frozen trees increment only their resource-set difference; and
- negotiation history is cleared for the touched resources, so a rejected
  search cannot poison the next transaction.

A successful route atomically becomes the next resident snapshot. A failed
negotiation invalidates it and deliberately falls back to the prior sparse
full reset on the following call. Rejection needs no special rollback API:
the next trial's frozen incumbent trees are the transaction target, and the
same difference synchronizer restores them. Thus placement/packing search can
keep its stateless `PnrResult` contract while the expensive physical occupancy
state remains persistent underneath it.

The ECP5-wide PIP delay table is also constructed once for timing closure and
shared by the dedicated-edge ECO and later placement refinements. Local ECOs
temporarily use eight negotiated-congestion iterations, then restore the full
32-iteration budget. The saved runtime was spent on widening the routed
dedicated-edge search from one candidate to four rather than merely reducing
the flow's work.

Measured on the same AXI4 300 MHz input:

| flow | routed packing candidates | final WNS / WHS | PIPs | runtime |
|---|---:|---:|---:|---:|
| rebuild baseline | 1 | -254 / -397 ps | 25,660 | 84.00 s |
| persistent routes | 1 | -254 / -397 ps | 25,660 | 73.75 s |
| persistent routes, wider search | 4 | **-254 / -397 ps** | **25,660** | **78.90 s** |

The four packing trials themselves route in roughly 160--180 ms each after
the first cached placement check. The wider run is still 5.10 seconds faster
than the one-candidate rebuild baseline with identical selected QoR, and it
evaluates three additional real-route alternatives. This is why speed and QoR
are coupled here: actual route+STA is the reliable objective, so cheaper
transactions buy more objective evaluations before the runtime budget is
exhausted.

This does not close the nextpnr gap: its reference run remains about 3.14
seconds and reaches roughly 309.6 MHz. The next structural target is the inner
A*/negotiation cost and a cheap topology/capacity projection capable of
screening thousands of placement moves; persistent transactions remove the
state-reconstruction tax but do not yet provide that candidate-generation
layer.

## Architecture-scaled A* estimate (2026-08-24)

The router previously added raw Manhattan tile distance to a path score whose
units are a criticality blend of quantized picosecond delay and congestion.
That dimensional mismatch made the heuristic too weak for nearby critical
arcs and inconsistently strong for long-line hops. nextpnr instead converts an
architecture delay prediction into the same units as its accumulated route
score.

Texo now predicts the remaining critical-route delay as `100 ps + 100
ps/tile`, converts it through the existing criticality/quantum function, and
adds only the noncritical hop fraction. Congestion-only routing retains raw
Manhattan distance, so the fast initial route is unchanged. The coefficients
are intentionally larger than nextpnr's ECP5 formula because Texo folds wire
delay into PIP timing classes whereas nextpnr accounts for wire and PIP delay
separately.

Measured on the same AXI4 300 MHz input:

| flow | final WNS / WHS | PIPs | runtime |
|---|---:|---:|---:|
| persistent-route baseline | -254 / -397 ps | 25,660 | 78.90 s |
| architecture-scaled A* | **-240 / -397 ps** | 25,667 | **66.51 s** |

This improves setup by 14 ps while reducing wall time by 12.39 seconds
(15.7%). Two rejected calibrations established the useful range: directly
copying nextpnr's small residual-delay formula finished near 57 seconds but
regressed WNS to -481 ps, while the fully realized `300 + 250 ps/tile` model
finished in 60.50 seconds at -377 ps. Both over-constrained Texo's differently
normalized route graph and were reverted.

## Arc-critical capacity projection and hierarchical placement nodes (2026-08-24)

Broad critical-path placement no longer sends the four smallest-HPWL moves
directly to negotiated routing. The incumbent route is projected into sparse
`wire/PIP -> [(net, arc criticality)]` ownership. Shared resources carry the
maximum criticality of the sink arcs that actually use them, rather than one
net-wide value. A local bounded A* then ranks the best 16 geometric moves by
characterized PIP delay plus the cost of the weakest conflicting owner that
would have to retreat. Capacity greater than current occupancy costs nothing;
an actual conflict costs `150 ps + 10 ps * victim criticality`. The moving
net's own incumbent tree is excluded.

This is the placement-level version of "pass the critical arc and retreat only
the conflicting noncritical arc." It is a projection, not a substitute for
negotiated routing: trying only its first candidate was fast but discarded a
better topology basin.

Broad search now also has an explicit first hierarchy. Physical tile
coordinates are the coarse graph nodes; BEL slots inside a tile are retained
for packing and local refinement but equivalent coarse candidates are routed
only once. Logs showed that the old flat candidate list repeatedly routed
different slots in one tile and received nearly identical STA results.

Measured on the same AXI4 300 MHz input:

| broad candidate policy | final WNS / WHS | PIPs | runtime |
|---|---:|---:|---:|
| architecture-scaled A* baseline | -240 / -397 ps | 25,667 | 66.51 s |
| projection, first candidate only | -508 / -399 ps | 25,650 | 43.32 s |
| projection, first two candidates | -404 / -399 ps | 25,589 | 49.64 s |
| projection, four flat candidates | **-214 / -399 ps** | 25,645 | 68.21 s |
| **projection, four candidates then tile collapse** | **-214 / -399 ps** | **25,665** | **59.78 s** |

The selected form improves setup WNS by 26 ps and removes two PIPs while
cutting 6.73 seconds (10.1%) from the preceding baseline. Hold WNS moves by
2 ps in the wrong direction and is effectively unchanged. The one- and
two-candidate results are important: the projection graph is useful for
shortlisting, but it is not accurate enough to replace real route topology
selection yet.

The next structural waste visible in the trace is round-level duplication:
when a closure seed does not change, the same cell/BEL move is routed and
analyzed again in later rounds. Candidate identity plus seed route identity
should become a transposition table before widening the projected search.

## Exact closure-move transpositions (2026-08-24)

Critical-path closure now fingerprints the complete seed placement and route
arcs together with each proposed placement. A proposal already routed from
that exact physical topology is skipped in later closure rounds. Placement
alone is deliberately insufficient: after a global ripup changes the route
tree, the same cell move remains eligible because its interference can change.

The selected capacity-projection result is bit-identical at -214/-399 ps and
25,665 PIPs. Runtime fell from 59.78 to 58.31 seconds, another 1.47 seconds
(2.5%). This is a conservative transposition table rather than approximate
memoization; it cannot reuse a result across genuinely different topologies.

## Persistent placement hierarchy and bounded local transactions (2026-08-24)

`PlacementRefiner` used to cache legal assignments but rebuild their spatial
index on every timing refinement, then linearly scan every legal BEL choice
for each critical cell. The index now lives with the reusable refiner. A move
of radius 16 enumerates only the Manhattan rings around the current coarse
tile (about 545 coordinates on this device) and then validates the indexed
assignments exactly. Packing groups retain their atomic assignments; the
coarse lookup does not weaken legality.

Two smaller transaction changes are also exact:

- a combinational-only placement move reuses the incumbent immutable global
  clock routes when every global-net endpoint and pin binding is unchanged;
- disposable local routing trials stop after five negotiated iterations,
  while whole-design routing retains its 32-iteration budget. Selected local
  candidates on this benchmark converge within five iterations.

Each step reproduced -214/-399 ps and 25,665 PIPs. From the exact-move cache's
58.31-second result, global-route reuse reached 57.91 seconds, the local trial
cap reached 57.57 seconds, and persistent spatial indexing reached **56.45
seconds**. Relative to the 66.51-second architecture-scaled A* baseline, the
combined capacity/topology work is 10.06 seconds (15.1%) faster while setup
WNS is 26 ps better.

## Shared immutable route DAGs (2026-08-24)

Local placement trials previously copied every frozen `NetRoute` three times:
from the incumbent result into constraints, from constraints into the router's
working vector, and from the successful working vector into the persistent
occupancy snapshot. The route topology is immutable for all but the released
nets, so `PnrResult`, `RoutingConstraints`, and `RoutingWorkspace` now share
those trees with `Arc`. Negotiation replaces only dirty net entries. Occupancy
synchronization first tests pointer identity and falls back to structural
equality only for independently constructed but equivalent trees.

Constraint validation and negotiated routing also share one placement-specific
pin-to-wire cache. Resource-reference validation now derives counts directly
from borrowed arcs instead of cloning and re-sorting the route. These changes
preserve route ordering, congestion decisions, and timing costs.

Alternating identical AXI4 runs measured the committed baseline at 61.95 s and
the shared-tree form at 60.72 s; another adjacent pair measured 64.08 s and
61.45 s. A final combined run completed in 60.63 s. Every run reproduced
**-214/-399 ps and 25,665 PIPs** exactly. Runtime is noisy on this host, but the
direction held in both adjacent comparisons (about 2--4%).

The more important profile result is structural: one 57.41-second metrics run
spent 2.64 seconds loading the architecture, 16.70 seconds in 113 routing
transactions, and 1.48 seconds in their STA passes. At least 36.6 seconds lies
outside negotiated routing and STA. The next optimization target must therefore
be split at whole-flow boundaries before further tuning A*: graph/packing,
initial placement, and candidate construction are now more plausible dominant
costs than the local router itself.

## Phase-local capacity projection (2026-08-24)

Whole-flow metrics now report packing, initial placement, initial global
routing, initial route/timing, dedicated-edge search, and timing closure as
separate durations. Closure further reports refiner construction, monotonic
refinement, local connections, critical vertices, basin escape, and hold
repair. On AXI4, the 51.33-second implementation split was:

| phase | time |
|---|---:|
| packing | 0.19 s |
| initial placement | 2.52 s |
| initial global routing | 1.17 s |
| initial route and timing | 0.62 s |
| dedicated-edge search | 1.51 s |
| timing closure | **45.32 s** |

Inside closure, critical-vertex refinement alone took 30.56 seconds. The
capacity projection over every routed arc and its 25,665 PIPs was being rebuilt
for every radius-1 and radius-2 local move even though only the radius-16 broad
topology search reads it. Projection construction is now conditional on the
broad hierarchy level. This changes no candidate score or routing decision.

With host load elevated, an adjacent saved-binary comparison measured the
pre-change shared-route version at 66.42 s wall / 54.34 s user CPU and the lazy
projection version at **62.10 s wall / 51.27 s user CPU**. Both produced
**-214/-399 ps and 25,665 PIPs**. The 4.32-second wall and 3.07-second user-CPU
reductions confirm that graph construction scope, not just A* implementation,
was a material part of the remaining runtime.

## Packing-generation placement graph workspace (2026-08-24)

The analytical placer and timing-closure refiner used to validate the complete
legal-assignment table independently. More importantly, a shared physical
assignment table was traversed once for every logical packing group even when
all groups had the same candidate domains. On AXI4 this repeated the device-wide
LUT/FF legality proof thousands of times.

`PlacementRefinementWorkspace` now survives packing generations and owns three
architecture-level caches:

- compatible BEL tables keyed by logical cell shape;
- spatial indexes keyed by the shared assignment table; and
- validated group shapes keyed by strong `Arc` identities for the assignment
  table and each column's candidate table.

The strong references make the identity proof safe against allocator address
reuse. Candidate-specific pin bindings produce distinct candidate tables and
therefore still receive their own complete validation. A failed validation is
never entered into the cache. The initial analytical solve and the refiner
rebuilt after the dedicated-edge packing ECO share this workspace, while each
still receives the correct current `PlacementConstraints`.

In adjacent metrics runs, initial placement fell from 2.59 s to 0.41 s and the
post-packing closure-refiner build from 2.26 s to 0.00048 s. Total flow time
fell from 52.31 s to 45.72 s; wall time fell from 58.51 s to **51.86 s**. A
second non-metrics run completed in **51.98 s**. Both reproduced **-214/-399
ps and 25,665 PIPs** exactly. This is the clearest evidence so far that graph
construction lifetime and hierarchy were a larger problem than the low-level
search implementation.

The same hierarchy is now used when validating the starting point of every
refinement transaction. Previously each logical group linearly searched its
entire shared assignment table on every proposal; single-cell units likewise
linearly searched all compatible BELs. A grouped assignment now selects its
physical-tile bucket from the cached spatial index and compares only entries in
that bucket, while sorted single-cell BEL IDs use binary search. This is an
exact membership test, not a geometric approximation.

The next identical non-metrics run fell from 51.98 s to **48.59 s** (user CPU
40.83 s to **37.96 s**) with unchanged -214/-399 ps and 25,665 PIPs. A metrics
run put timing closure at 39.96 seconds and the whole implementation at 44.02
seconds. The remaining dominant phase is still critical-vertex refinement at
29.74 seconds; its 37 proposal passes now total 4.64 seconds, so most of that
phase is the deliberate route-and-STA portfolio rather than graph membership
or projection construction.

## Pareto-filtered topology trials (2026-08-24)

Stage-wise archive metrics exposed an important non-monotonic dependency. The
7.67-second setup refinement moves the initial -3119 ps / -519139 ps TNS seed
to -1139 ps / -40688 ps TNS before critical closure. Removing that apparent
detour expanded critical-vertex refinement from 28.7 to 46.9 seconds and ended
at -524 ps WNS. It is therefore a basin-conditioning pass, not removable
overhead.

The broad critical-cell search had a different redundancy. Its route-topology
projection ranks the failing path, but up to four placements were then routed
even when a deeper placement was worse both on that topology score and on the
criticality-weighted estimate over every net moved with the cell. The flow now
keeps the topology winner and routes a deeper candidate only when it establishes
a new lower whole-cell timing estimate. This is the Pareto frontier of two
independent graph views; it does not impose a fixed candidate-rank cutoff.

A fixed two-candidate experiment was faster but regressed to -490 ps WNS and
was rejected. The Pareto filter completed the identical 300 MHz case in
**44.34 seconds** (34.55 s user, 10.10 s system), down from the best 48.59-second
baseline. Setup WNS and hold WHS remain **-214/-399 ps**, while setup TNS
improved from -4325 to **-3905 ps**. The selected implementation uses 25,675
PIPs, ten more than the baseline. This demonstrates that the route graph and
the timing placement graph are complementary: pruning is safe only when a
candidate is dominated in both.

The radius-1/2 placement scorer also repeated the same bounded 16-hop A*
delay query for multiple legal assignments that resolve to identical physical
pin-wire endpoints. A transaction-local `(driver wire, sink wire)` cache now
reuses those exact results while evaluating one cell's candidates. Extending
the cache across the complete refiner lifetime was measured and rejected: the
larger hash table and delay-table identity checks made critical closure slower.

Adjacent saved-binary A/B runs measured 44.79 s wall / 34.93 s user without
the local cache and **44.08 s wall / 34.30 s user** with it. Both produced
-214/-399 ps and 25,675 PIPs exactly. The cache lifetime therefore matches the
placement-graph transaction rather than the architecture or closure lifetime.

## Recovery-only speculative placement checkpoints (2026-08-25)

Replacing the local critical-cell portfolio with unconditional speculative
batches exposed why the remaining route trials matter. Six-cell and two-cell
batches reached 32.30 and 30.68 seconds respectively but both ended at -369 ps
WNS; a one-cell first-improvement variant reached 25.74 seconds but fell to
-555 ps. The established search is a best-improvement method over as many as
six routed cell alternatives, not merely an inefficient way to accumulate
locally attractive moves.

A guarded fast path now handles only the distinct recovery case after a global
multiresolution reroute has pushed WNS below -800 ps. It speculatively combines
the first two legal local moves and accepts the checkpoint only if the complete
route and STA objective improves and WNS gains at least 32 ps. Rejected
checkpoint fingerprints are removed from the trial cache so the original
per-cell best-improvement search remains a true fallback. On AXI4 exactly one
checkpoint qualified: it recovered 313 ps at once.

The final implementation remains identical at the reported level: **-214 ps
WNS, -3905 ps setup TNS, -399 ps WHS, and 25,675 PIPs**. Critical closure fell
from about 25.0 to 24.1 seconds. Adjacent non-metrics saved-binary runs measured
44.65 s wall / 34.14 s user for the prior implementation and **44.03 s wall /
34.07 s user** for the recovery checkpoint path. The modest wall reduction is
primarily lower system work; treat the phase reduction as the stronger signal
until more benchmarks exercise severe post-ripup recovery.

## Shared immutable routing cost tables (2026-08-25)

Every local route trial cloned `RoutingCosts` solely to lower its negotiation
iteration limit. The clone also duplicated both device-wide PIP delay vectors,
even though characterized maximum/minimum delays are immutable across every
placement and routing trial. On the full ECP5 device this repeated a large
allocation and copy for essentially every critical candidate.

The maximum and minimum PIP tables are now `Arc<[u32]>`; a cost clone shares
them while retaining independent criticality maps, hold constraints, detailed
net selections, and iteration limits. Pointer-sharing has a unit test, and the
router's indexed reads are unchanged.

In adjacent saved-binary non-metrics runs, the `Vec` version measured 43.67 s
wall / 33.97 s user / 10.02 s system with 3,208,944 KiB maximum RSS. The shared
version measured **33.43 s wall / 32.00 s user / 1.72 s system** with 3,013,476
KiB RSS. A metrics run placed critical refinement at 16.12 seconds and the flow
itself at 27.92 seconds, down from approximately 24.1 and 40.1 seconds. Final
QoR remains exactly **-214 ps WNS, -3905 ps setup TNS, -399 ps WHS, and 25,675
PIPs**. Eliminating the map clone after this change had no measurable benefit
and was reverted; the device-wide arrays were the structural problem.

## Incremental congestion-cost workspace (2026-08-25)

Route-trial instrumentation separated the reusable router transaction into
pin-wire resolution, constraint/cost validation, occupancy synchronization,
and negotiation. Across 112 AXI4 route trials, those stages totaled 0.045,
0.425, 0.072, and **17.333 seconds** respectively: rebuilding pin and
constraint views was only 3% of the transaction cost. Search counters then
showed 137,371 sink searches expanding 520.8 million physical edges and
pushing 374.9 million frontier entries. The remaining cost is inside repeated
edge relaxation, not transaction setup.

Each relaxation previously fetched occupancy, capacity, and history for both
its destination wire and PIP, then recomputed the same prospective congestion
penalty. `RoutingWorkspace` now stores dense `u32` wire/PIP congestion views.
They are recomputed only for touched resources when the negotiation factor or
history changes and updated immediately when occupancy changes. The A* and
hold searches consume the cached values; their accumulated costs, ordering,
and selected routes are unchanged. Although the full vectors represent about
126 MiB virtually on the 85K device, zero pages remain uncommitted and the
measured peak RSS rose by only about 7 MiB.

The metrics run reduced total negotiation from 17.320 to **16.797 seconds**,
critical-vertex closure from 15.925 to **15.625 seconds**, flow time from
27.624 to **27.214 seconds**, and wall time from 33.57 to **33.03 seconds**.
Final QoR stayed exactly **-214 ps WNS, -3905 ps setup TNS, -399 ps WHS, and
25,675 PIPs**. Two exact-order hot-loop experiments were rejected alongside
this result: delay/heuristic lookup tables were neutral (33.64 vs 33.70 s),
and a four-way frontier heap slowed negotiation to 19.834 seconds.

## Compact route-search state (2026-08-25)

The device graph already guarantees 32-bit physical wire and PIP IDs, but A*
still stored its distance, arrival, predecessor IDs, and four-field frontier
tuple as 64-bit values. On the 85K graph this made the persistent search
scratch 40 bytes per wire and each of the 374.9 million pushed frontier
entries 32 bytes. Physical FPGA path delays and negotiated scores are far
below the 32-bit limit.

`RouteSearch` now uses checked 32-bit values for its hot state and converts to
64-bit only while evaluating a transition. The frontier tuple is 16 bytes and
has a size regression test; tuple ordering and path reconstruction are
unchanged. On top of the incremental congestion cache, the AXI4 metrics run
kept exactly **-214 ps WNS, -3905 ps setup TNS, -399 ps WHS, and 25,675
PIPs**. Flow time fell from 27.214 to **27.033 seconds**, critical refinement
from 15.625 to **15.559 seconds**, wall time from 33.03 to **32.95 seconds**,
and maximum RSS from 3,023,520 to **2,991,864 KiB**.

`Device::routing_neighbors` also used to validate a wire by randomly reading
the cold `Wire` table before indexing the same-ID adjacency table. Those
tables are constructed with identical lengths, so a checked adjacency lookup
provides the same unknown-wire error without touching names and other cold
wire metadata on every expanded node. Removing that second lookup kept QoR
identical and reduced the next metrics run to **26.954 seconds** of flow,
**15.502 seconds** of critical refinement, **32.83 seconds** wall, and 31.12
seconds user CPU.

## Retained-tree topology in the placement projection (2026-08-25)

The broad placement projection previously evaluated every moved sink as an
independent driver-to-sink route. That did not match the incremental router:
when one fanout sink moves, all other sink arcs remain frozen and their shared
tree is a legal set of connection sources. The resulting projection both
double-counted shared trunks and preferred placements that looked cheap only
after discarding useful incumbent topology.

`RouteCapacityProjection` now retains the incumbent net trees. For a moved
sink, its bounded capacity-aware search is seeded from every wire belonging to
the other sink arcs; a moved driver still receives a complete driver-to-sink
projection. A regression test verifies that the projected connection grows
from the cheaper retained-tree source. The local radius-1/2 hierarchy keeps
its routed-delay-excess ordering: applying even the improved projection there
still changed the final result to -266 ps WNS, so topology projection remains
the broad-search layer rather than replacing the allowance objective.

On AXI4, the broad shared-tree projection preserved **-214 ps WNS, -3905 ps
setup TNS, and -399 ps WHS**, while reducing the selected implementation from
25,675 to **25,655 PIPs**. Critical refinement fell from 15.502 to **15.179
seconds**, flow time from 26.954 to **26.494 seconds**, wall time from 32.83 to
**32.32 seconds**, and user CPU from 31.12 to **30.78 seconds**. This is a QoR
and runtime gain from increasing placement-graph topology fidelity, not from
reducing the candidate portfolio.

Three related shortcuts were measured and rejected. Immediate arc-victim
removal lost the shared present-congestion price and ended at -325 ps; fixed
timing corridors of 6 tiles ran faster but ended at -474 ps (10 tiles restored
QoR but was slower than 12); and old-route soft upper bounds reached 8.92
seconds of critical refinement but trapped closure at -379 ps or worse. The
remaining router redesign must preserve shared-tree state and shared conflict
prices while allowing critical topology to change.

The retained source set depends only on the incumbent `(net, sink)` topology,
not on a BEL candidate. It was initially rebuilt as a `BTreeSet` inside every
projected candidate evaluation. The broad-placement transaction now builds
each sorted source slice once and shares it across all candidate searches;
membership is a binary search with no per-candidate tree allocation. Output
remained exactly -214/-399 ps with -3905 ps setup TNS and 25,655 PIPs. Critical
refinement fell to **15.074 seconds**, flow time to **26.197 seconds**, wall
time to **31.99 seconds**, and user CPU to **30.47 seconds**.

A subsequent two-layer owner/pricing-occupancy prototype confirmed that
retaining conflict prices is necessary but not sufficient for in-place
rip-up. Immediate eviction of every selected victim reached 26.006 seconds
but ended at -301 ps WNS; mixing event-driven eviction only for
criticality-1 arcs was worse at -457 ps and 31.107 seconds. Resource prices,
arc ordering, and victim ownership must therefore be replaced coherently
rather than mixed with the round-based negotiator.

## Pass-local exact placement-delay workspace (2026-08-25)

The local radius-1/2 placement objective runs a bounded 16-hop route search
for each physical endpoint pair. Its result depends on the two endpoint wires
and the PIP delay table, not on the rest of the placement. The critical-path
loop nevertheless discarded both completed queries and the search heap/hash
allocations after every cell. Near the final solution, one radius-2 pass that
found no proposal spent 943 ms almost entirely repeating this work.

`PlacementConnectionDelayWorkspace` now lives for one critical refinement
pass. It memoizes exact endpoint-pair results across adjacent path cells and
reuses the allocation behind the bounded-search heap and visited-cost table
for every new pair. The lifetime is deliberately shorter than the full timing
closure: an earlier refiner-lifetime cache paid table-identity and oversized
hash costs after the useful working set had changed.

The deterministic AXI4 result remained exactly **-214 ps WNS, -3905 ps setup
TNS, -399 ps WHS, and 25,655 PIPs**. The final empty radius-2 proposal pass
fell from 943 to **591 ms**, critical refinement from 15.074 to **14.256
seconds**, flow time from 26.197 to **25.351 seconds**, wall time from 31.99 to
**31.21 seconds**, and user CPU from 30.47 to **29.55 seconds**. This is an
exact graph-query optimization: it changes neither candidate order nor route
or timing decisions.

The remaining local-search table originally keyed `(WireId, hop)` and endpoint
pairs through Rust's general-purpose tuple hash. Both IDs are guaranteed to
fit 32 bits by the physical model, so these states are now packed into one
`u64` and hashed with a single SplitMix64 finalizer. A raw identity hasher was
measured and rejected: regular ECP5 wire IDs collided badly in hashbrown and
grew the final empty pass to 1.107 seconds. Mixing the packed integer reduced
that pass further to **410 and 407 ms** in consecutive runs, and critical
refinement to **14.046 and 14.037 seconds**. The second complete run measured
**25.325 seconds** flow, **31.13 seconds** wall, and **29.63 seconds** user CPU,
with the same -214/-3905/-399 ps result and 25,655 PIPs.

Two topology representations were then measured and rejected. First, a
pass-persistent `resource -> (net, sink arc)` index kept every owner in nested
hash/tree containers and updated it after each arc change. Although output was
identical, it made critical refinement **16.326 seconds**, flow **28.182
seconds**, and raised RSS by about 32 MiB. The current round-end index is
already sparse because it records only overused resources; an event-driven
router needs a compact flat owner table, not a general persistent map.

Second, monotonic placement anchored each weighted sink to the single
incumbent sibling-tree wire currently nearest that sink. It reduced the final
route to 25,645 PIPs but trapped placement on the old branch structure:
**-375 ps WNS**, 14.952 seconds of critical refinement, and 27.417 seconds of
flow. A retained route tree must therefore remain a candidate-dependent
source *set*. The next coarse placement layer should expose a distance field
to that complete set, then let the physical graph projection choose the exact
source for a shortlist; collapsing the set to one point loses necessary
topology freedom.

## Placement-hierarchy transition on WNS regressions (2026-08-25)

Per-trial metrics exposed a deterministic 2.805-second tail in monotonic
placement. The first four accepted global refinements produced the winning
seed at -1139 ps WNS / -40688 ps TNS. A fifth round then routed 256-, 128-,
64-, and 32-unit portfolios (997, 972, 529, and 307 ms) solely to reject all
four; critical refinement subsequently started from exactly the round-four
seed.

The last two accepted global moves both improved aggregate timing while
worsening WNS (-930 to -1044 to -1139 ps). That is a hierarchy boundary: the
global placer is still reducing TNS, but it is now fighting the worst path
that the next critical-vertex layer is designed to repair. Monotonic placement
now transitions after two consecutive accepted WNS regressions instead of
routing another complete large-move portfolio to discover the same boundary.

The AXI4 monotonic result stayed bit-for-bit at -1139/-40688 ps, 25,754 PIPs,
and 236,661 HPWL. Final output also remained exactly **-214 ps WNS, -3905 ps
setup TNS, -399 ps WHS, and 25,655 PIPs**. Monotonic refinement fell from
6.351 to **3.414 seconds**, complete timing closure to **19.143 seconds**, flow
to **23.209 seconds**, wall to **29.31 seconds**, and user CPU to **27.68
seconds**.

## One global route layer per placement topology (2026-08-25)

Critical closure still invoked full-chip multiresolution routing after every
outer placement round. The first 2350-net renegotiation was useful: it took
1.26 seconds and improved aggregate timing. The next full pass took 1.20
seconds and was rejected. A moved-cell rearm prototype skipped that pass but
eventually accumulated enough local moves to launch a third full route; it
failed to resolve one conflict after all 32 iterations and invalidated the
resident workspace, making the run slower.

The routing hierarchy is now explicit. Global timing placement receives one
full data-route renegotiation. Every later critical placement move freezes the
unchanged topology and reroutes all affected nets, so reopening all 2350 nets
cannot add information about untouched placement. The old WNS-based rearm
state was removed rather than retained as a second, conflicting policy.

Final AXI4 output remained exactly **-214 ps WNS, -3905 ps setup TNS, -399 ps
WHS, and 25,655 PIPs**. Critical closure fell from 14.451 to **12.969
seconds**, complete timing closure from 19.143 to **17.669 seconds**, flow from
23.209 to **21.709 seconds**, wall from 29.31 to **27.84 seconds**, and user CPU
from 27.68 to **26.12 seconds**.

## Critical-search hierarchy and dominated local routes (2026-08-25)

The radius schedule is a real search hierarchy, not an arbitrary portfolio.
Four radius-1 rounds shape local slots, four radius-2 rounds escape the local
tile, and four radius-16 rounds change coarse path topology. Doubling the
per-radius depth ran in 14.877 seconds but regressed WNS to -482 ps; depth five
ended at -395 ps; interleaving `[1, 2, 16]` ran in 14.026 seconds but ended at
-746 ps. Simple occupied-BEL swaps likewise either produced -848/-431 ps or,
when restricted enough to preserve QoR, found no usable move. The unresolved
blockers are packing groups and routing ownership, not singleton occupancy.

The exact local delay search also retained every `(wire, hop-count)` state,
even when the same wire had already been reached in fewer hops and no more
delay. It now stores a per-wire cumulative hop Pareto frontier and never
enqueues such dominated states. Two expensive proposal misses fell from 201
to 89 ms and from 175 to 73 ms. This pruning is exact: the final result stayed
at **-214 ps WNS, -3905 ps TNS, -399 ps WHS, and 25,655 PIPs**.

Two-cell local batches were previously enabled only for severe recovery below
-800 ps. Metrics showed a second useful regime near closure: one batch moved
WNS from -305 to -245 ps. Batches now run in the severe (<= -800 ps) and
near-closure (>= -400 ps) bands, and their cumulative criticality-weighted
geometry must not regress before routing. This retained only the +313 ps and
+60 ps batches while suppressing routed batches that produced -289 ps or only
+12 ps. Final timing stayed **-214/-3905/-399 ps**; the selected route grew by
three PIPs to 25,658. Consecutive runs measured 20.88--21.62 seconds of flow,
12.87--12.98 seconds of critical closure, and 25.84--26.08 seconds user CPU.

## Dedicated-edge portfolio bound (2026-08-25)

Disabling post-route LUT-to-FF reassignment changed the trajectory and ended
at -225 ps WNS with 25,842 PIPs, so packing topology is necessary. On this
deterministic design, however, only the first ranked candidate was accepted;
later candidates could not affect the winning descent. Capping the initial
portfolio from four trials to one preserved the exact final timing trajectory.
Candidate preparation, packing clone/reassignment, and placement swap together
measured below 1 ms; the one incremental route and STA add about 0.18 seconds.
Repeating this search after final critical placement found three legal
candidates and none improved WNS, so no final packing ECO is retained.

The final critical path is already geographically compact
(`R20C67 -> R20C68 -> R20C69 -> R20C71 -> R20C70`) but its first adjacent
hop costs 841 ps. The next design target is therefore an owner-aware route
transaction: project the unloaded fast path, identify the exact lower-critical
sink arcs occupying it, and release those victims together with the critical
arc. Releasing every corridor or victim immediately was previously shown to
lose QoR; ownership, path choice, and rip-up must be one coherent operation.

The owner-aware projection exposed a missing dimension immediately. Pricing
owners only by criticality selected a nominally cheap fast path whose victim
was one 268-sink shared trunk; moving one logical owner therefore meant
releasing 268 coupled arcs, not one noncritical arc. Limiting the final
transaction to eight blockers found no small victim set for any of the three
largest critical connections (841, 504, and 297 ps). Rerouting each connection
alone also reproduced the incumbent exactly in 7--13 ms. The final bottleneck
is not a missed singleton collision.

`RouteCapacityProjection` now records the number of sink arcs through each
`(resource, owner net)` and adds that release scope to broad-placement path
cost. Up to eight coupled arcs pay 25 ps each; a larger shared-trunk rewrite is
priced out of this local hierarchy and must be handled by a future coarse
topology transaction. This graph-fidelity change preserved **-214 ps WNS,
-3905 ps TNS, -399 ps WHS, and 25,658 PIPs**. The measured run took **20.608
seconds** of flow, **12.293 seconds** of critical closure, **26.55 seconds**
wall, and 24.85 seconds user CPU. The experimental final blocker/reroute pass
is not retained; only the placement projection's transaction-size model is.

## Alternating in-place critical routing (2026-08-25)

Within one critical-cell pass, placement already advanced to the accepted
candidate but the next trial still froze routes from the pass-start
implementation. It therefore rerouted every previously moved cell again.
Keeping the accepted `PnrResult` as a rolling incumbent reduced critical
closure from about 12.3 to 7.85 seconds, but permanently froze collateral
topology and regressed WNS to -431 ps. Rebasing only after two rolling moves
had the same failure because most pass-local chains contain at most two moves.

The retained policy alternates hierarchy levels: one accepted move routes
in-place from the rolling implementation; the next starts from the pass seed
and reopens the cumulative moved-net topology. This restores the exact final
timing (**-214 ps WNS, -3905 ps TNS, -399 ps WHS**) while reducing the route
from 25,658 to **25,649 PIPs**. Consecutive runs measured **12.077/12.098
seconds** of critical closure, **20.227/19.953 seconds** of flow,
**26.03/25.88 seconds** wall, and 24.70/24.53 seconds user CPU. The result is a
two-level transaction rather than a fixed compatibility rule: local route
state is reused once, then coarse topology freedom is deliberately restored.

## Corridor-entry pruning for retained route trees (2026-08-25)

The multi-source A* used every wire in a retained route tree as a heap seed.
That is especially wasteful for a local edit to a high-fanout net: the AXI4
reset tree contains about 2,671 wires, while the timing search is restricted
to a small driver/sink corridor. A source outside the corridor whose outgoing
neighbors are also all outside cannot relax any state in the permitted search
graph, but it still paid a heap push and pop for every sink search and every
negotiation iteration.

`RouteSearch` now marks every retained source as a tree member, preserving the
old cycle/tie behavior, but seeds only sources already in the corridor or with
a direct edge into it. This is exact graph pruning: a removed source had no
legal first transition under the existing corridor rule, so the reachable
paths and route ordering are unchanged.

Two AXI4 runs retained **-214 ps WNS, -399 ps WHS, and 25,649 PIPs**. Critical
closure measured **11.965/11.989 seconds**, flow **20.079/19.743 seconds**, and
wall **26.01/25.62 seconds**, compared with 12.077/12.098, 20.227/19.953, and
26.03/25.88 seconds for the preceding alternating-rebase baseline.

Three adjacent hierarchy experiments were rejected. Adding radius 32 to every
critical-placement round ran faster but regressed WNS to -395 ps; running it
only after normal closure generated no candidate and added about 0.17 seconds.
An adaptive rebase threshold based on cumulative changed route arcs preserved
QoR only at the eight-arc local-transaction bound and was still slower than
fixed alternation; a 128-arc threshold regressed WNS to -298 ps. Finally, a
rolling-route preflight perfectly predicted the measured rebase accept/reject
decisions, but the difficult preflights themselves took 150--200 ms, nearly
doubling the relevant routing work. These results keep rebase policy fixed and
move optimization into the retained-tree graph traversal itself.

The same high-fanout path also checked whether each logical sink was already
routed with a linear scan of every retained arc. For the reset net this was a
quadratic scan of roughly 1,361 sinks per routing iteration. Canonical
`NetRoute` objects now use their existing sink ordering for binary lookup;
the mutable arc list inside `route_net` uses a separate `BTreeSet` because new
arcs are appended in criticality order until the completed route is
canonicalized again.

This second exact indexing change again retained **-214 ps WNS, -399 ps WHS,
and 25,649 PIPs**. Two runs measured **11.868/11.848 seconds** of critical
closure, **20.069/19.741 seconds** of flow, and **25.88/25.59 seconds** wall.
