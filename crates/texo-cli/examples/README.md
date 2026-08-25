# Design-specific CLI examples

The installed `texo` binary accepts arbitrary self-contained Veryl input.
Historical hard-coded XOR and AXI4 experiments remain available only as a
Cargo example so benchmark and interchange diagnostics do not become part of
the user-facing CLI contract:

```sh
cargo run -p texo-cli --example design-specific-flows -- help
```

New reusable behavior belongs in `texo pnr`; design-specific experiments and
testbenches belong here.
