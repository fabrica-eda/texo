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
| `texo-pnr` | Cell-to-BEL placement and capacity-aware Wire/PIP routing |
| `texo-flow` | End-to-end stage orchestration and verification gates |
| `texo-cli` | Command-line entry point |

The PnR crates do not depend on Veryl, Struo, Celox, or a particular FPGA.
Cells, nets, BELs, BEL pins, wires, and PIPs are visible through one typed
problem graph. Candidate binding edges are generated lazily, while the backing
storage remains split into compact type-specific arenas. Adapters and target
databases stay at the boundary so upstream API changes do not leak into the
algorithms.

## Try it

```sh
cargo run -- demo
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
```

See [docs/architecture.md](docs/architecture.md) for the integration boundary
and [docs/roadmap.md](docs/roadmap.md) for the implementation sequence.
