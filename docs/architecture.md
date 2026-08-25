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
4. Texo generates configuration features natively. A versioned target pack
   supplies the imported configuration database and bundled final ECP5 codec;
   users do not install Project Trellis.

The inspected upstream versions are:

| Project | Dependency policy | Relevant contract |
|---|---|---|
| `fabrica-eda/struo` | `8dad8c2e27f4dacd6283bb4015ad99208618c228` | `Ecp5Netlist`, `Ecp5Cell`, `CCU2C`, mapped ports, nextpnr-compatible JSON, verification policy |
| `celox` | crates.io exact version `=0.3.1` | `FrontendArtifact` and native post-map simulation |
| `YosysHQ/prjtrellis` | exporter inspected at `3afe7b52b30f4b4417ee98f03016767a502006e3` | deduplicated chip database, relative resource references, package IO database |
| `prjtrellis-db` | snapshot records the exact revision; fixture uses `015e0330630d7c238c0e4f2cdd9c8157eb78c54a` | ECP5 routing, package, cell timing, and interconnect timing data |

Struo is not currently published on crates.io, so its first adapter will pin an
exact Git revision. Celox must be consumed from crates.io and pinned to an exact
release version; it must not be added as a Git dependency or overridden with a
Git `[patch.crates-io]` entry. Both adapters require fixture tests before an
upgrade.

`texo-struo` implements that boundary. It consumes Struo's in-memory
`Ecp5Netlist`, creates explicit logical pins for LUT4, CCU2C carry slices,
TRELLIS_FF, DP16KD, constant networks, and every top-level port bit, then
connects them through Texo nets without serializing JSON. Each compound CCU2C
is split into two cells joined by an adapter-local carry net while INIT and
INJECT configuration remains in metadata. The same mapped object can be turned
into a crates.io Celox `FrontendArtifact` for post-map verification.

`texo-flow::verify_post_map_with_celox` records simulation evidence only after
a caller-provided Celox testbench succeeds. `implement_struo_ecp5` requires
that evidence by default; the arbitrary-file CLI explicitly permits it to be
absent without recording the gate. The flow clones the imported design, derives DP16KD requirements from
Struo metadata, runs LUT/FF, BRAM, DCCA, and LPF packing, then invokes generic
placement and routing. Failures commit neither new evidence nor a partially
transformed design. The owned success result retains primitive metadata,
absorbed configuration inputs, packing decisions, placement, and routes for
later timing and bitstream stages; the original mapped object remains available
for further Celox verification.

The `texo pnr` command exercises that boundary for a complete Veryl project,
including its compilation units and dependencies, without intermediate netlist
serialization. Its schema-versioned JSON checkpoint is deterministic
and records architecture/database provenance, verification evidence, mapped
primitive configuration, absorbed inputs, target packing, final Cell-to-BEL
bindings, selected speed grade, every routed Wire/PIP ID and name, exact
configuration-tile ownership, and min/max timing checks. It is the complete
input to the native configuration stage. `texo bitgen` translates those
records directly into configuration features in Rust and invokes the
target-pack-local `ecppack` only as the final binary codec. The release path
has no nextpnr, Python, pytrellis, or system Project Trellis dependency.
Design-specific
Celox and AXI4 flows live in the
`design-specific-flows` Cargo example rather than the installed CLI.

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

## Unified problem graph

Placement and routing operate on one heterogeneous graph:

```text
Cell --placement candidate--> BEL
  |                            |
CellPin --binding candidate--> BelPin --pin access--> Wire
  |                                                      |
 Net                                      Wire <--PIP--> Wire
```

Logical cells and nets are demand; BELs, wires, and PIPs are finite-capacity
physical supply. A solution selects cell-to-BEL bindings and one connected
wire/PIP tree for each net. This exposes placement legality, routing capacity,
pin reachability, congestion, and eventually timing to the same optimizer.

The graph is unified at the query and solver level, not stored as one boxed
node array. Each kind has a typed stable ID (`CellId`, `BelId`, `WireId`, and so
on) and a compact arena. `UnifiedGraph` returns logical, fixed physical, and
programmable adjacency directly, while `Cell -> BEL` and `CellPin -> BelPin`
candidate edges are generated lazily. This avoids storing the potentially huge
Cartesian product of cells and compatible BELs.

The M0 router already observes wire and PIP capacities and PIP direction. A
production router will replace its breadth-first search with negotiated
congestion without changing the graph contract.

Target packing emits atomic placement groups rather than replacing the unified
graph. A group lists its logical cells and every legal BEL tuple; the generic
placer selects a tuple as one indivisible unit. Candidate-specific CellPin to
BelPin overrides handle physical port selection without mutating the logical
netlist. For ECP5, a LUT-driven FF is grouped with `TRELLIS_COMB(z)` and
`TRELLIS_FF(z+1)` and keeps the dedicated `DI` path. An unpaired FF maps its
logical `DI` terminal to the general-routing `M` pin. Package constraints use
the same mechanism as a one-cell group with one PIO BEL assignment.
The two cells from a split CCU2C form another atomic group: K0 and K1 share one
physical slice and their `TRELLIS_COMB` z values differ by four. Their FCI/FCO
arcs use the speed-grade `SCCU2C` characterization split the same way as the
nextpnr ECP5 chip database: K0 owns the characterized FCI-to-FCO delay, while
K1 has a zero-delay continuation and only the remaining FCI-to-F1 delay.

Each logical memory must also supply its Struo-derived depth, logical word
width, and physical port width. The ECP5 packer accepts only DP16KD modes
`1/2/4/9/18`, checks the corresponding `16384/8192/4096/2048/1024` depth
limits, constrains the cell to compatible `DP16KD` BELs, and assigns stable
WID values from 3 in CellId order. Missing metadata and illegal geometries are
reported before any packing constraint is mutated. Checkpoint schema v3 also
records the fixed BEL-pin-to-CIB predecessor for every absorbed DP16KD input.
The native configuration writer uses those records to emit CIB constants,
clock/enable polarity, chip-select decode, WID, zero initialization, and the
multi-tile EBR feature group without rebuilding the routing graph.
Global-clock promotion uses the same graph instead of a separate clock model.
The packer ranks nets by recognized FF/BRAM clock-pin fanout (five sinks by
default), selects at most the 16 ECP5 primary clock networks, and inserts a
logical DCCA cell with `CLKI` and `CLKO` pins. A target-neutral net-splitting
operation moves only clock sinks behind `CLKO`; mixed data sinks remain on the
original net. The DCCA cell is then constrained to compatible physical DCCA
BELs, so both sides are routed through the ordinary Wire/PIP graph. Unknown
nets, duplicate requests, non-clock nets, and insufficient DCCA resources are
transactional packing errors.
`texo-flow::implement_struo_ecp5` carries these packing decisions through
placement and routing before recording mapped-netlist and physical evidence.

The LPF boundary parses nextpnr-compatible `LOCATE COMP <port> SITE <pin>` and
`IOBUF PORT <port> key=value...`, and `FREQUENCY PORT <port> <value> <unit>`
commands. It supports quoted names, comments, multiline commands, exact
Hz/kHz/MHz/GHz normalization, scalar ports, and indexed vector bits. Resolution accepts a
borrowed `(port name, &[CellId])` iterator, so Struo's `ImportedPort` remains in
the adapter crate. Locations become fixed package/PIO groups; IOBUF attributes
remain indexed by logical IO CellId for configuration generation. Unknown port
names, duplicate sites/attributes, and unconstrained bits in strict mode are
errors. Other LPF verbs are retained and shown by `texo lpf-info`.

Constant handling occurs while the Struo mapped object is copied into the
logical graph. Constant LUT inputs select and replicate the corresponding INIT
truth-table plane, so those logical pins and nets disappear. Constant FF
controls and DP16KD inputs that have configuration muxes are recorded in an
`absorbed_inputs` table keyed by CellId and physical pin name. If a constant is
still required by a routable terminal—such as a constant top-level output—the
adapter lazily creates one shared LUT source for that value. Consequently no
abstract `Constant` cell reaches ECP5 placement, while the original mapped
object remains unchanged for Celox post-map verification.

## ECP5 architecture snapshots

`texo-target-ecp5` expands a schema-versioned snapshot into the target-neutral
`Device`. Project Trellis location types remain deduplicated on disk: each grid
location names one type, while BEL pins and PIPs use relative location/resource
references. Import occurs in three passes—wires, BELs/pins, then PIPs—so every
relative reference is validated before it reaches the solver. Package records
must resolve to an IO BEL. ECP5-only data such as BEL type/Z, fixed-arc status,
tile type, PIP timing class, and LUT permutation flags remains in side metadata
keyed by the same stable IDs. Schema v3 also contains timing tables for speed
grades `6`, `7`, `8`, and `8_5G`: interconnect base/fanout coefficients plus
split `TRELLIS_COMB`, `TRELLIS_FF`, and DCCA cell arcs. Interconnect
coefficients retain Project Trellis's independently fitted `min/typ/max`
corners without sorting them. Those fitted values are not necessarily
monotonic; setup propagation selects `max`, while hold propagation selects
`min`, matching the ECP5 timing database semantics used by nextpnr.

Generate a snapshot from local, revision-controlled Project Trellis source and
database checkouts:

```sh
python3 tools/export_ecp5.py \
  -L /path/to/prjtrellis/libtrellis/build/libtrellis \
  -L /path/to/prjtrellis/timing/util \
  --database /path/to/prjtrellis-db \
  --device LFE5UM5G-85F \
  --project-trellis-revision <source-commit> \
  --database-revision <database-commit> \
  --output artifacts/LFE5UM5G-85F.json

cargo run -- target-info artifacts/LFE5UM5G-85F.json
```

The exporter deliberately enables Project Trellis's LUT-permutation PIPs and
split-slice mode. It classifies PIPs using Project Trellis's timing utilities,
normalizes the characterized timing tables to integer picoseconds, and retains
conservative min/max corners. The importer rejects snapshots that omit
split-slice mode or speed-grade data, use another schema/family, have incomplete
provenance, contain invalid timing ranges/classes, or contain dangling
relative/package references.

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
`texo-timing` computes early/minimum and late/maximum arrival ranges through the
selected PIP tree and characterized cell arcs. ECP5 STA includes PIP timing
class and enabled-source fanout, LUT input-to-output delay, DCCA propagation,
FF clock-to-Q, setup, and hold. Setup uses late data versus early capture clock;
hold uses early data versus late capture clock. The speed grade is mandatory
and recorded in the checkpoint. A report with no constrained sequential
endpoint, negative setup slack, or negative hold slack does not satisfy the
timing-closure gate. BRAM timing, generated clocks, multicycle paths, and false
paths remain future work.

## Reproducibility

- Pin tool versions, crates.io package versions, and unavoidable Git revisions;
  never follow a moving Git branch in a release flow.
- Store the target device/package/speed grade and constraints in every artifact
  manifest.
- Seed randomized algorithms explicitly and record the seed.
- Keep a human-readable route artifact and compare it against nextpnr during
  bring-up.
