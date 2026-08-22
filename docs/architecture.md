# Architecture

## Scope

Texo starts as an ECP5 FPGA implementation tool. "The whole toolchain" is
assembled from independently testable stages rather than reimplementing every
frontend at once:

1. Celox supplies RTL simulation and a source-independent simulator frontend.
2. Struo supplies the Veryl frontend, synthesis IR, synthesis passes, ECP5
   technology mapping, and Celox adapter.
3. Texo owns physical architecture import, packing, placement, routing, static
   timing analysis, and implementation artifacts.
4. Project Trellis is initially used to pack the routed configuration into a
   bitstream. Native bitstream generation can be added after the PnR result is
   stable and independently checked.

The inspected upstream versions are:

| Project | Dependency policy | Relevant contract |
|---|---|---|
| `fabrica-eda/struo` | `fd994db45f792fb4a019d57575fbb1239eae21ae` | `Ecp5Netlist`, `Ecp5Cell`, mapped ports, nextpnr-compatible JSON, verification policy |
| `celox` | crates.io exact version `=0.3.1` | `FrontendArtifact` and native post-map simulation |

Struo is not currently published on crates.io, so its first adapter will pin an
exact Git revision. Celox must be consumed from crates.io and pinned to an exact
release version; it must not be added as a Git dependency or overridden with a
Git `[patch.crates-io]` entry. Both adapters require fixture tests before an
upgrade.

## Boundaries

```text
            logical domain                         physical domain

 Veryl -> Struo RTL -> Struo IR -> ECP5 cells -> Texo target model
                  |                    |               |
                  |                    +-> Celox       +-> pack/place/route
                  +-> Celox reference simulation              |
                                                              v
                                            route + timing + configuration
```

`texo-model` deliberately uses stable IDs and owned data. A Struo adapter will
copy mapped cells, nets, port directions, constants, clocks, and constraints
into this model. No Struo or Celox type appears in the PnR API.

## Verification policy

Artifacts advance only when the evidence for the preceding stage exists. The
intended release gates are:

1. RTL simulation
2. RTL-to-synthesized equivalence
3. mapped-netlist completeness
4. Celox post-map simulation
5. placement legality
6. routing legality and connectivity
7. timing closure
8. bitstream/configuration round-trip check

Celox is a functional simulator, not a gate-level timing simulator. Static
timing and post-route delay validation therefore remain Texo responsibilities.

## Reproducibility

- Pin tool versions, crates.io package versions, and unavoidable Git revisions;
  never follow a moving Git branch in a release flow.
- Store the target device/package/speed grade and constraints in every artifact
  manifest.
- Seed randomized algorithms explicitly and record the seed.
- Keep a human-readable route artifact and compare it against nextpnr during
  bring-up.
