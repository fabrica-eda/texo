# Incremental setup closure

The normal `texo pnr` flow combines connection-aware partial placement,
local placement repair and transactional route ECOs. Every accepted change
must pass legal routing and strictly improve the existing full setup/hold
objective. Clock constraints, uncertainty and timing coverage still determine
whether the result has timing-closure evidence.

## Placement and route inputs

`--initial-placement placement.json` fixes the named cells to their BELs.
Omitted cells are placed using connectivity to the fixed cells, while atomic
placement groups and shared-resource legality are preserved. A complete set
of bindings therefore keeps the supplied physical placement.

`--initial-routes routes.json` can accompany that placement. The file contains
an array of named physical trees in the checkpoint `routes` format. A subset
of complete net trees is accepted. Driver wires, PIP identities/directions,
every current sink and mandatory target routes are checked against the newly
mapped design. Missing nets are routed normally. The importer accepts neither
old timing evidence nor disconnected fragments as a shortcut to sign-off.

Use `--resume-checkpoint` for an unchanged compatible mapped design. It
conflicts with independent placement, route and LUT/FF-pair files; use a new
output path. See [checked resume](resume-routing.md).

## Setup search

Local placement uses actual routed delays to decide whether the incumbent
misses its connection targets. Empty-device shortest paths rank proposed
locations but do not replace the incumbent measurement. Broad shortlists
count distinct destination tiles before truncation. Local moves precede broad
moves; an accepted placement gives route repair another turn immediately.

Whole-net ECOs consider failing cones within 50 ps of the current worst slack.
Each net is represented by a sink in its own worst cone, including zero-delay
connections. Local placement considers a wider 250 ps window. Both searches
still require improvement of the full STA objective to commit a result.

ECO routing preserves 1 ps delay distinctions. An occupancy probe may discover
up to 32 additional owners of the target's preferred track; the probe itself
is never an accepted route. Every affected net is rebuilt together with its
immutable branches preserved. Among equal criticalities the primary target
goes first, so its displaced owner cannot immediately reacquire the track.
Higher-criticality owners retain precedence. Failed probes, illegal rebuilds
and rejected STA trials keep the incumbent. Routing cost modes and workspace
occupancy are restored before returning to the caller.

The negotiated router retains a finite 128-iteration ceiling, increased from
32 for congested designs. This can spend more time on designs that previously
exhausted their routing limit; it does not guarantee timing closure.

## Per-invocation setup budget

`--setup-optimization-budget-seconds 900` limits elapsed setup-search time.
The library equivalent is `Ecp5FlowOptions::setup_optimization_budget`, an
`Option<Duration>`. The default is unlimited; zero skips setup search. The
option conflicts with `--no-timing-optimization` in the CLI.

The timer starts at the first setup-search step, after initial implementation
and STA. It is owned by one flow invocation, so another invocation has a fresh
budget. A candidate already in progress finishes routing and STA before the
limit is checked again. Initial routing, final reporting and hold repair are
outside this soft limit. The last fully checked incumbent is returned, including
its actual negative slack if closure was not reached. Budget expiry never
grants timing evidence or permits bitgen of an unclosed implementation.

Wall time is not a deterministic search seed. Save the command, inputs and
checkpoint; replay its placement/routes with timing optimization disabled when
exact physical reproduction is required.
