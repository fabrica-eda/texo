# Texo

Texo is an FPGA-first place-and-route project written in Rust. The initial
target is Lattice ECP5. It is designed to complete this open EDA flow:

```text
Veryl source
    -> Struo analysis and synthesis
    -> Struo ECP5 technology-mapped netlist
    -> Celox functional and post-map verification
    -> Texo packing, placement, routing, and timing
    -> Project Trellis configuration/bitstream tooling
```

The workspace currently contains a small, deterministic reference PnR engine.
It establishes the data ownership and error boundaries before ECP5-specific
architecture data is introduced; it is not yet a production FPGA router.

## Workspace

| Crate | Responsibility |
|---|---|
| `texo-model` | Typed logical/physical arenas and their unified graph view |
| `texo-pnr` | Atomic-group Cell-to-BEL placement and capacity-aware Wire/PIP routing |
| `texo-flow` | End-to-end stage orchestration and verification gates |
| `texo-struo` | Direct Struo ECP5 import and crates.io Celox verification boundary |
| `texo-target-ecp5` | Project Trellis import, LUT/FF packing, package-to-PIO binding |
| `texo-cli` | Command-line entry point |

The PnR crates do not depend on Veryl, Struo, Celox, or a particular FPGA.
Cells, nets, BELs, BEL pins, wires, and PIPs are visible through one typed
problem graph. Candidate binding edges are generated lazily, while the backing
storage remains split into compact type-specific arenas. Adapters and target
databases stay at the boundary so upstream API changes do not leak into the
algorithms.

`texo-struo` pins Struo to one exact Git revision because it is not published
on crates.io. Celox is pinned to `=0.3.1` from crates.io and is never replaced
with a Git dependency.

## Try it

```sh
cargo run -- demo
cargo run -- target-info crates/texo-target-ecp5/fixtures/minimal-ecp5.json
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
```

`tools/export_ecp5.py` generates a deduplicated architecture snapshot from a
local Project Trellis build and database. Production device snapshots are
generated artifacts; the repository keeps a small schema fixture for fast,
deterministic tests.

See [docs/architecture.md](docs/architecture.md) for the integration boundary
and [docs/roadmap.md](docs/roadmap.md) for the implementation sequence.
