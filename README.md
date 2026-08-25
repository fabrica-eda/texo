# Texo

Texo is an FPGA-first place-and-route project written in Rust. The initial
target is Lattice ECP5. It is designed to complete this open EDA flow:

```text
Veryl source
    -> Struo analysis and synthesis
    -> Struo ECP5 technology-mapped netlist
    -> Texo packing, placement, routing, and timing
    -> Project Trellis configuration/bitstream tooling
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
cargo run --release -- pnr examples/xor/xor.veryl \
  --top Xor \
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

`texo pnr` accepts any path to one self-contained Veryl compilation unit. Run
`texo pnr --help` for synthesis-goal, placement-weight, unconstrained-IO,
global-clock, and timing-closure controls. If `--output` is omitted, the
checkpoint is written beside the source as `<source>.texo.json`.

`tools/export_ecp5.py` generates a deduplicated architecture snapshot from a
local Project Trellis build and database. Schema v4 includes PIP timing classes
with their independently fitted `min/typ/max` corners and the `6/7/8/8_5G`
speed-grade cell/interconnect tables. Production device
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

The AXI4 self-test has a gated path from Struo through a Texo-owned placement
and route to an ECP5 bitstream. Cache a production architecture snapshot, export
the lossless mapped netlist, run PnR, and emit the bitstream as follows:

```sh
cargo build --release --locked -p texo-cli
/usr/bin/python3 tools/build_ecp5_txdb.py --device LFE5UM5G-85F
cargo run --release -p texo-cli --example design-specific-flows -- axi4-json artifacts/axi4.json
cargo run --release -p texo-cli --example design-specific-flows -- axi4-pnr \
  artifacts/architecture/texo-LFE5UM5G-85F-schema4-cache2.txdb CABGA381 8 \
  examples/axi4-self-test/lfe5um5g-85f-evn-250mhz.lpf \
  artifacts/axi4.checkpoint.json
tools/axi4_bitstream.py \
  --checkpoint artifacts/axi4.checkpoint.json \
  --mapped-json artifacts/axi4.json \
  --lpf examples/axi4-self-test/lfe5um5g-85f-evn-250mhz.lpf \
  --config artifacts/axi4.config \
  --bit artifacts/axi4.bit
```

`axi4_bitstream.py` uses nextpnr only for ECP5 packing and configuration
writing: it locks every Texo placement and route edge and does not invoke the
nextpnr placer or router. It refuses checkpoints without all six functional,
physical, and timing evidence gates, without nonnegative setup and hold slack,
or with a route that is not a tree. It then requires every checkpoint edge to
be represented in the configuration and byte-compares an `ecpunpack`/`ecppack`
round trip. The command requires `nextpnr-ecp5`, `ecppack`, and `ecpunpack` on
`PATH`.

See [docs/architecture.md](docs/architecture.md) for the integration boundary
and [docs/roadmap.md](docs/roadmap.md) for the implementation sequence.
