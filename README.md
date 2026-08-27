# Texo

Texo is an FPGA-first place-and-route project written in Rust. The initial
target is Lattice ECP5. It is designed to complete this open EDA flow:

```text
Veryl project (`Veryl.toml`, sources, and dependencies)
    -> Struo analysis and synthesis
    -> Struo ECP5 technology-mapped netlist
    -> Texo packing, placement, routing, and timing
    -> Texo-native configuration generation + bundled ECP5 bitstream codec
```

Celox functional and post-map verification can be attached by API clients that
have a testbench. The general-purpose file CLI does not claim that evidence
when no testbench was supplied.

The workspace currently contains a small, deterministic reference PnR engine.
It establishes the data ownership and error boundaries before ECP5-specific
architecture data is introduced; it is not yet a production FPGA router.

## Workspace

| Crate | Responsibility |
|---|---|
| `texo-model` | Typed logical/physical arenas and their unified graph view |
| `texo-pnr` | Atomic-group Cell-to-BEL placement and capacity-aware Wire/PIP routing |
| `texo-timing` | Min/max post-route timing graph with setup and hold analysis |
| `texo-flow` | Verified Struo-to-ECP5 orchestration and release gates |
| `texo-struo` | Direct Struo ECP5 import and crates.io Celox verification boundary |
| `texo-target-ecp5` | Project Trellis import, LUT/FF, DP16KD and DCCA packing, package-to-PIO binding |
| `texo-cli` | Command-line entry point |

The PnR crates do not depend on Veryl, Struo, Celox, or a particular FPGA.
Cells, nets, BELs, BEL pins, wires, and PIPs are visible through one typed
problem graph. Candidate binding edges are generated lazily, while the backing
storage remains split into compact type-specific arenas. Adapters and target
databases stay at the boundary so upstream API changes do not leak into the
algorithms.

`texo-struo` pins Struo to one exact Git revision because it is not published
on crates.io. Celox is pinned to `=0.3.1` from crates.io and is never replaced
with a Git dependency. The adapter accepts current Struo `CCU2C` output by
splitting each primitive into two atomically packed ECP5 carry slices.

## Try it

```sh
cargo run --release -- pnr examples/xor \
  --architecture crates/texo-target-ecp5/fixtures/minimal-ecp5.json \
  --package CABGA381 \
  --speed 6 \
  --lpf examples/xor/xor.lpf \
  --output /tmp/texo-xor-checkpoint.json
cargo run -- target-info crates/texo-target-ecp5/fixtures/minimal-ecp5.json
cargo run -- lpf-info examples/xor/xor.lpf
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
```

`texo pnr` accepts a project directory or `Veryl.toml`. It follows
`[build].sources`, resolves the standard library and local/Git dependencies
through `Veryl.lock`, analyzes all compilation units together, and uses
`[synth].top` unless `--top` overrides it. Run `texo pnr --help` for
synthesis-goal, placement-weight, unconstrained-IO, global-clock, and
timing-closure controls. Without `--output`, project checkpoints go to
`target/texo/<top>.json`.

## PnR search model

Texo is intended to support two entry points into the same deterministic,
anytime PnR search:

- a **scratch flow** that constructs an implementation from the mapped design
  and target database alone; and
- an **incremental/ECO flow** that starts from a compatible implementation
  checkpoint, preserves unaffected placement and routing, and repairs the
  physical consequences of a design or constraint change.

Both flows keep the best legal implementation found so far while candidate
placement and routing state is allowed to move through worse or temporarily
congested states. Additional deterministic work should improve the incumbent
instead of being required to recover an earlier result. The search must serve
both useful QoR latency--how quickly Fmax rises--and QoR ceiling--the Fmax it
can reach with more effort.

A checkpoint is an optimization seed, not a memoized answer. PnR is
trajectory-dependent: retained topology changes routing ownership, placement
neighborhoods, and the later order of otherwise deterministic decisions.
Consequently, a checkpoint-guided run can finish with either better or worse
QoR than a scratch run, even when both are reproducible and both pass the same
legality and timing gates. Scratch and incremental results are therefore not
required to be physically identical, and benchmarks must identify the entry
mode and compare QoR at fixed work budgets rather than treating warm-start
time as cold-start time.

Incremental preservation is a search bias, not a permanent restriction. The
flow should first invalidate only changed cells, sink arcs, constraints, and
their dependent timing state; route and place the smallest affected conflict
component transactionally; and retain the previous implementation on a failed
trial. If local repair stalls, it should progressively reopen shared route
subtrees, whole nets, placement regions, and finally full-chip topology. A
scratch rebase remains an available search branch. Reopening the chip restores
search freedom, but cannot promise the identical basin or result of a separate
scratch run.

The current CLI writes complete physical checkpoints, but does not yet expose
checkpoint-guided PnR as a general project command. Until that interface and
its compatibility checks are implemented, checkpoints are output artifacts
for inspection, bit generation, and future incremental reuse.

Board-level open-drain buses keep a two-state verification interface in the
Veryl design and are fused into one physical bidirectional ECP5 pad at the
mapping boundary. For example, this binds the scalar input `sda_i` and
active-high pull-down request `sda_drive_low` to the LPF port `sda`:

```sh
texo pnr path/to/veryl-project --package CABGA381 --speed 8 \
  --lpf board.lpf --open-drain sda:sda_i:sda_drive_low
```

Repeat `--open-drain` for additional pads. The LPF constrains only the fused
pad name, such as `LOCATE COMP sda SITE A10;`; the generated PIO drives zero
when requested, otherwise enters high impedance, and continuously feeds pad
readback to the input port.

## Releases

Release-plz maintains a release PR from conventional commits on `main`.
Merging that PR creates a `vX.Y.Z` GitHub Release without publishing the
workspace to crates.io. The release workflow builds the `texo` CLI for GNU and
static-musl x86-64 Linux, Apple Silicon and Intel macOS, and x86-64 Windows. It
uploads a platform archive and SHA-256 checksum for every binary.

`tools/export_ecp5.py` generates a deduplicated architecture snapshot from a
local Project Trellis build and database. Schema v6 includes exact physical
configuration-tile ownership in addition to PIP timing classes with their
independently fitted `min/typ/max` corners and the `6/7/8/8_5G` speed-grade
cell/interconnect tables. Production device
snapshots are generated artifacts; the repository keeps a small schema fixture
for fast, deterministic tests.

Production `.txdb` inputs are reproducible release artifacts. The tracked
architecture manifest pins their Project Trellis environment, schema, cache
format, device set, and artifact names; the builder emits compressed caches,
provenance, and SHA-256 checksums. See
[`docs/architecture-databases.md`](docs/architecture-databases.md) for the
build, release, download, and verification procedure.

The P&R checkpoint contains provenance, evidence, primitive configuration, absorbed constants,
packing decisions, IO/clock constraints, placement, Wire/PIP routes, and the
post-route timing report.

Render any such checkpoint as a self-contained interactive physical-design
view (no server or architecture database is required):

```sh
cargo run --release -- visualize artifacts/axi4.checkpoint.json \
  --output artifacts/axi4.html
```

The viewer draws cells and per-net PIP topology, supports pan/zoom and search,
and can isolate routes below a configurable setup/hold slack threshold. See
[`docs/visualizer.md`](docs/visualizer.md) for controls and rendering details.

Normal users do not install Project Trellis or manage an architecture file.
The first `pnr` or `bitgen` for a device downloads one pinned, checksummed
target pack into the platform cache; later runs are offline. A pack contains
the Texo architecture database and the small Trellis codec/runtime subset
needed to serialize an ECP5 bitstream. It does not contain Python.

```sh
texo target fetch LFE5UM5G-85F                 # optional eager download
texo pnr path/to/veryl-project --package CABGA381 --speed 8 \
  --lpf board.lpf --output design.checkpoint.json
texo bitgen design.checkpoint.json --bit design.bit
```

To use the dedicated ECP5 JTAG block, expose the scalar
`jtag_tdo1`, `jtag_tdo2`, `jtag_tdi`, `jtag_tck`, `jtag_rti1`,
`jtag_rti2`, `jtag_shift`, `jtag_update`, `jtag_rst_n`, `jtag_ce1`, and
`jtag_ce2` ports and pass `--jtagg-prefix jtag` to `pnr`. Add
`--jtagg-disable-er2` when extension register two is not used. Texo binds those
ports to `JTAGG`, routes its fabric interface, and carries the ER1/ER2 settings
through the checkpoint into native bit generation.

To insert a user-configured ECP5 PLL, expose the physical reference clock and
the logical generated clock/lock signals as scalar inputs, then pass Struo's
`PllBinding` JSON to `pnr`:

```sh
texo pnr path/to/veryl-project --package CABGA381 --speed 8 \
  --lpf board.lpf --pll-binding pll-12-to-250.json \
  --output design.checkpoint.json
```

The binding keeps `reference_clock_port` as a package input, removes
`output_clock_port` and `locked_port` from the package boundary, and connects
them to the selected `EHXPLLL` outputs. Divider parameters and analog
attributes remain user-owned (normally generated by `ecppll`). An LPF
`FREQUENCY PORT` command constrains the physical reference clock, while
`FREQUENCY_PIN_<output>` in the PLL attributes constrains the generated clock
for Texo STA. Repeat `--pll-binding` for independent PLLs. PLL placement,
feedback/control routing, configuration words, and tile groups are retained in
the checkpoint and emitted by native bitgen.

For an air-gapped machine, download the release `.txpkg.zst` elsewhere and run
`texo target install <pack.txpkg.zst>`. `TEXO_TARGET_DIR` overrides the cache
location. Supplying individual architecture/database/codec paths remains a
developer diagnostic mode, not a user prerequisite.

`bitgen` requires synthesis/mapping completeness, legal physical
implementation, and timing closure. RTL and post-map simulation evidence is
accepted when an API client supplies a testbench, but it is not required to
emit a bitstream from the general project CLI. The AXI4 self-test additionally
exercises the fully simulation-signed-off path:

```sh
cargo build --release --locked -p texo-cli
/usr/bin/python3 tools/build_ecp5_txdb.py --device LFE5UM5G-85F
cargo run --release -p texo-cli --example design-specific-flows -- axi4-pnr \
  artifacts/architecture/texo-LFE5UM5G-85F-schema6-cache5.txdb CABGA381 8 \
  examples/axi4-self-test/lfe5um5g-85f-evn-250mhz.lpf \
  artifacts/axi4.checkpoint.json
cargo run --release -- bitgen artifacts/axi4.checkpoint.json \
  --bit artifacts/axi4.bit
```

`texo bitgen` consumes checkpoint schema v3 and writes Trellis configuration
features in Rust; neither nextpnr nor pytrellis is a runtime dependency. It
selects the exact checkpoint device, configures every non-fixed Texo route edge
and placed primitive, and refuses generation without implementation and timing
evidence. LUT/carry, FF, input/output and open-drain bidirectional IO, JTAGG,
EHXPLLL, DCCA routing, and DP16KD configuration are emitted without reconstructing a
second PnR context. The bundled `ecppack` is only the final binary codec.

See [docs/architecture.md](docs/architecture.md) for the integration boundary
and [docs/roadmap.md](docs/roadmap.md) for the implementation sequence.
