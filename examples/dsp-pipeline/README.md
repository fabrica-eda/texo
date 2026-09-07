# DSP placement and timing fixture

An unsigned 18-by-18 multiplication sits between ordinary input and output
registers. Two counters exercise all input bits; the LED XOR uses every bit of
the 36-bit product. The expected mapping contains one `MULT18X18D`.

See [DSP support](../../docs/dsp.md) for the supported mode, architecture cache
requirements and invocation. `timing.lpf` constrains the input to 124.8 MHz for
STA; this fixture has no PLL and has not been qualified on hardware.
