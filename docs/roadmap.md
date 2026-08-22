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

- Add a revision-pinned adapter crate for Struo's `Ecp5Netlist`.
- Consume Celox from crates.io at the workspace-pinned exact version; do not
  introduce a Celox Git dependency.
- Preserve LUT4 equations, `TRELLIS_FF` controls, `DP16KD`, constants, ports,
  clocks, and user constraints.
- Run Celox post-map simulation before physical implementation.
- Add committed mapped-netlist fixtures so PnR work does not require a frontend
  rebuild on every test.

Exit criterion: a Struo blinky enters Texo without a JSON round trip and the
same mapped design still passes Celox simulation.

## M2 — ECP5 architecture database and packing

- Import Project Trellis chip database data into a versioned compact format.
- Model tiles, BELs, wires, PIPs, packages, clock networks, and timing arcs.
- Pack LUT/FF pairs, IO buffers, BRAMs, and global clocks with explicit legality
  errors.
- Parse LPF constraints and bind package pins.

Exit criterion: every packed primitive has one legal BEL on the selected exact
device/package and a checker can reconstruct all occupancy.

## M3 — placement

- Add timing/congestion cost models and incremental bounding-box updates.
- Implement simulated annealing first; evaluate analytical placement later.
- Add dedicated clock/IO/BRAM placement constraints.
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
