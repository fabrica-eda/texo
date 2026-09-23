# Measured ECP5 placement and complete STA libraries

`texo pnr --measured-placement-model model.json` uses a board-fitted route/LUT
model to propose placement changes. Each candidate is routed and checked by
whole-design STA; it is accepted only when setup and hold do not regress. The
heuristic placement delay predictor is disabled in this mode. With a full
measured library, routed setup ECOs, physical-PIP placement refinement and hold
repair remain enabled. Their candidates use the installed table and fresh STA,
without constructing the heuristic predictor. Repeated local moves are memoized
within an improving search epoch, then reconsidered once at a fixed point.
All critical cells receive nearby-move trials before broad relocations, so a
large BRAM trial cannot starve small LUT repairs. Two-, four- and eight-tile
moves are ranked by actual PIP costs, including equal-span alternatives; only
broad relocations use the coarse span filter. Local scoring runs Dijkstra within
a finite spatial corridor, without the old 16-hop cutoff that could reject
reachable paths. Zero-cost cycles terminate with one best label per wire. BRAM relocations retain the normal retry budget because they affect wide data
and address/control ports; small ordinary moves keep the shorter trial budget. Imported trees
must be advisory, with no explicitly preserved initial routes.

`texo pnr --measured-timing-library library.json` additionally replaces the
selected speed grade's entire PIP, cell-arc and setup/hold surface. This option
is independent of placement model selection. The JSON schema is version 1,
kind `measured_joint_cell_route_sta_library`; `timing` is a complete
`SpeedGradeRecord`. The loader checks device/package, successful heldout
validation, a declared temperature margin of at least 20%, required setup/hold
uncertainty, hashes of all measurement inputs, and evidence references for every
entry. It never copies missing numeric entries from the architecture cache.

LUT timing follows the physical input selected by the routed input-permutation
PIP, rather than the logical input name before routing. Full STA and ECO STA
both resolve these arcs again after route changes. Route ranking includes each
physical input's excess delay above the fastest input; reported wire delays
remain unchanged, and STA counts the selected cell arc exactly once. Equal-input
legacy libraries retain their previous routing costs.

Both CLI uncertainties must be at least the library's
`qualification.required_setup_hold_guard_ps`. The checkpoint stores the exact
library path and SHA-256. Checked bitgen reloads that exact library and rejects
changed or missing evidence. Resuming a measured checkpoint requires an explicit
library selection and fresh STA. These checks enforce provenance and declared
coverage; they do not independently establish that the physical experiment or
fitting assumptions are correct.

For an 18x18 combinational multiplier, input bit i cannot affect product bits
below i; signed correction cannot affect bits below 18. A complete measured
library therefore contains 1,026 real dependencies and omits the 342 impossible
arcs in historical dense tables. General STA still accepts complete legacy dense
tables for old checkpoints, but never accepts a missing real dependency.

The hardware model used during development is a joint effective cell/route
model, not an independent measurement of every transistor or pin parameter.
Fixed access delays can be absorbed into adjacent cell arcs and common clock
latency. Such reference choices, shared feature projections, capture-boundary
checks, holdout partitions and board/PVT scope belong in the measurement
artifact. Synthetic parser fixtures in unit tests are explicitly identified and
must never be used as board timing libraries.

`measurement_identities` exports physical PIP classes and BEL/pin identities
without numeric delay labels. `measured_surface` exports table structure for
coverage auditing. `measured_jtag_sta` reconstructs the internal JTCK graph using
the exact checkpoint library, allowing comparison against integrated STA; it
does not characterize the external JTAGG boundary.
