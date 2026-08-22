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
- [ ] Run Celox post-map simulation automatically before physical
  implementation.
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
- [ ] Generate and characterize a complete production device snapshot using a
  locally built `pytrellis` and `prjtrellis-db` checkout.
- [ ] Model clock networks and timing arcs beyond the routing-graph delay
  metadata already preserved by the importer.
- [x] Validate DP16KD width/depth modes, constrain each BRAM to compatible
  BELs, and assign stable WID configuration values with explicit errors.
- [ ] Pack global clocks with explicit legality errors.
- [x] Fold LUT constants into INIT, absorb FF/BRAM constants into input-mux
  metadata, and synthesize shared constant LUTs only for residual nets.

Exit criterion: every packed primitive has one legal BEL on the selected exact
device/package and a checker can reconstruct all occupancy.

## M3 — placement

- Add timing/congestion cost models and incremental bounding-box updates.
- Implement simulated annealing first; evaluate analytical placement later.
- Add dedicated clock placement constraints; extend the existing fixed IO and
  DP16KD groups as new target rules require.
- Emit deterministic checkpoints and machine-readable quality metrics.

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

- Build a timing graph from cells, wires, PIPs, clocks, and constraints.
- Implement setup/hold analysis, multicycle/false paths, and timing reports.
- Generate the Project Trellis textual configuration; use `ecppack` initially.
- Require all functional, physical, and timing gates before bitstream release.
- Add hardware smoke tests and nextpnr differential tests.

## Later targets

Keep the physical model target-neutral, but do not generalize prematurely.
After ECP5 closes end to end, add another FPGA architecture. An ASIC flow would
require a separate LEF/DEF/Liberty/SPEF adapter and physical-effects model; it is
not part of the initial FPGA milestones.
