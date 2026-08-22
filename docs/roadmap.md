# Roadmap

## M0 — workspace and executable reference model

- [x] typed Cell/Pin/Net and BEL/BelPin/Wire/PIP arenas
- [x] unified graph queries with lazily generated placement/binding candidates
- [x] deterministic connectivity-aware Cell-to-BEL reference placer
- [x] directed, capacity-aware Wire/PIP reference router
- [x] flow-level verification gates and CLI demo
- [x] unit and integration tests

The M0 grid is intentionally abstract. It has separate BEL-pin and channel
wires plus directed PIPs, but does not model real ECP5 tiles. Its purpose is to
validate graph APIs and occupancy invariants before importing silicon data.

## M1 — Struo/Celox adapter

- [x] Add a revision-pinned adapter crate for Struo's `Ecp5Netlist`.
- [x] Consume Celox from crates.io at the workspace-pinned exact version; do not
  introduce a Celox Git dependency.
- [x] Preserve LUT4 equations, `TRELLIS_FF` controls, `DP16KD`, constants, ports,
  and clocks.
- [ ] Import user constraints alongside the mapped object.
- [x] Add programmatic mapped-netlist fixtures for LUT4, `TRELLIS_FF`, and
  `DP16KD` import plus crates.io Celox artifact validation.
- [x] Run caller-provided Celox post-map testbenches transactionally and require
  their evidence before ECP5 physical implementation.
- [ ] Add a complete mapped blinky fixture so PnR work does not require a
  frontend rebuild on every test.

Exit criterion: a Struo blinky enters Texo without a JSON round trip and the
same mapped design still passes Celox simulation.

## M2 — ECP5 architecture database and packing

- [x] Define a provenance-bearing, versioned, deduplicated Project Trellis
  snapshot format and Rust importer.
- [x] Import BELs, BEL pins, wires, directed/fixed PIPs, package-to-PIO
  bindings, and target-specific PIP metadata.
- [x] Verify direct Struo LUT4 and IO compatibility against imported
  `TRELLIS_COMB` and `PIO` BELs.
- [x] Atomically place LUT-driven FFs in matching `TRELLIS_COMB(z)` /
  `TRELLIS_FF(z+1)` slots; bind unpaired FF data through the `M` input.
- [x] Convert resolved package-pin bindings into fixed PIO placement groups.
- [x] Parse LPF `LOCATE COMP` and `IOBUF PORT` constraints, resolve scalar and
  vector Struo port cells, and retain unsupported commands for diagnostics.
- [x] Parse LPF `FREQUENCY PORT`, normalize exact decimal units to Hz, and
  resolve clock ports to logical IO cells.
- [ ] Generate and characterize a complete production device snapshot using a
  locally built `pytrellis` and `prjtrellis-db` checkout.
- [x] Import speed-grade PIP classes, LUT arcs, FF clock-to-Q/setup/hold, and
  DCCA arcs from Project Trellis timing data.
- [x] Validate DP16KD width/depth modes, constrain each BRAM to compatible
  BELs, and assign stable WID configuration values with explicit errors.
- [x] Rank clock nets by FF/BRAM clock-pin fanout, insert DCCA cells for at most
  16 global networks, and constrain them to compatible BELs transactionally.
- [x] Fold LUT constants into INIT, absorb FF/BRAM constants into input-mux
  metadata, and synthesize shared constant LUTs only for residual nets.
- [x] Orchestrate Struo metadata, optional LPF resolution, all target packing,
  placement, routing, and verification evidence as one transactional API.

Exit criterion: every packed primitive has one legal BEL on the selected exact
device/package and a checker can reconstruct all occupancy.

## M3 — placement

- [ ] Add timing/congestion cost models and incremental bounding-box updates.
- [ ] Implement simulated annealing first; evaluate analytical placement later.
- [ ] Extend the existing DCCA, fixed IO, and DP16KD placement groups as new target
  rules require.
- [x] Emit schema-versioned deterministic JSON implementation checkpoints and
  machine-readable occupancy metrics from the verified CLI flow.

Exit criterion: the blinky and AXI4 fixture place legally, repeatably, and with
quality measured against nextpnr.

## M4 — routing

- Separate global routing estimates from detailed routing resources.
- Implement negotiated-congestion routing (PathFinder-style), timing criticality,
  rip-up/reroute, and dedicated-resource handling.
- Verify shorts, opens, illegal PIPs, and directionality independently of the
  router.

Exit criterion: all fixture nets are connected with zero resource conflicts and
the route can be imported into Project Trellis configuration tooling.

## M5 — timing and bitstream release

- [x] Build a post-route timing graph from logical pins, selected PIPs, register
  boundaries, and LPF clock constraints.
- [x] Evaluate Project Trellis PIP class/fanout delays as min/max picoseconds
  and emit deterministic per-sink delays and worst setup/hold slack.
- [x] Import speed-grade LUT arcs, FF clock-to-Q/setup/hold, and DCCA timing.
- [x] Implement conservative early/late hold analysis.
- [ ] Implement multicycle/false paths, generated clocks, and BRAM timing.
- Generate the Project Trellis textual configuration; use `ecppack` initially.
- Require all functional, physical, and timing gates before bitstream release.
- Add hardware smoke tests and nextpnr differential tests.

## Later targets

Keep the physical model target-neutral, but do not generalize prematurely.
After ECP5 closes end to end, add another FPGA architecture. An ASIC flow would
require a separate LEF/DEF/Liberty/SPEF adapter and physical-effects model; it is
not part of the initial FPGA milestones.
