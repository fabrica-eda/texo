# Core250 ECP5 QoR regression

This frozen design is a physical-design stress test, not Texo's recommended
CPU microarchitecture. It combines a non-overlapped fifteen-cycle RV32I
subset core, four inferred `DP16KD` blocks, a JTAG-to-I2C debug path, and a
12 MHz to 250 MHz `EHXPLLL` binding.

The regular workspace test loads the whole Veryl project, synthesizes and maps
it, and checks its structural fingerprint. Run the full ECP5-85F speed-8 PnR
regression with:

```sh
cargo run --release -- pnr examples/core250-qor \
  --top Core250JtagTop \
  --package CABGA381 \
  --speed 8 \
  --lpf examples/core250-qor/core250-soc-250.lpf \
  --output /tmp/texo-core250-qor.json \
  --synthesis-goal-mhz 250 \
  --jtagg-prefix jtag \
  --pll-binding examples/core250-qor/pll-12-to-250.json \
  --allow-unconstrained-io \
  --placement-weight-exponent 2
```

The baseline for Texo commit `2966730` is 3,711 implemented cells, 3,936
nets, 42,077 PIPs, setup slack +7 ps, and hold slack +290 ps. Two independent
runs produced byte-identical checkpoints with SHA-256
`ed94e94d2577d2bbb6004ab981150c00a1da84d6f4baec65947b9cac24cfbf65`.
