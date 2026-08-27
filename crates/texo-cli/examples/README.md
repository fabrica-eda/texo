# Design-specific CLI examples

The installed `texo` binary accepts a complete Veryl project rooted at
`Veryl.toml`; `examples/xor` is a two-compilation-unit reference project.
Historical hard-coded XOR and AXI4 experiments remain available only as a
Cargo example so benchmark and interchange diagnostics do not become part of
the user-facing CLI contract:

```sh
cargo run -p texo-cli --example design-specific-flows -- help
```

New reusable behavior belongs in `texo pnr`; design-specific experiments and
testbenches belong here. The frozen `examples/core250-qor` Veryl project is the
larger ECP5 timing-closure regression used to exercise BRAM, JTAGG, PLL, carry,
setup, and hold behavior together.
