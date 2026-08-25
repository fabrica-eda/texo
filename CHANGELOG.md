# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0](https://github.com/fabrica-eda/texo/releases/tag/v0.1.0) - 2026-08-25

### Added

- *(cli)* add arbitrary Veryl PnR command
- *(visualizer)* explain route colors
- *(cli)* add physical design visualizer

### Fixed

- *(cli)* load complete Veryl projects
- *(visualizer)* classify global clock cells

### Other

- *(cli)* require Veryl project input
- *(pnr)* model physical placement density
- *(pnr)* route and rip up individual sink arcs
- *(pnr)* accelerate timing-driven routing
- Track Struo PR 35 unique retimed cell names
- Expose the placement weight exponent as flow configuration
- Emit verified AXI4 bitstreams from tree routes
- Close AXI4 timing with deterministic critical refinement
- Add deterministic analytical timing refinement
- Add deterministic timing-driven native ECP5 PnR
- Add native AXI4 PnR benchmark path
- Compact ECP5 resource metadata
- Add Struo CCU2C carry packing
- Add ECP5 speed grade timing
- Add post-route ECP5 timing analysis
- Add verified ECP5 CLI checkpoint flow
- Parse ECP5 LPF constraints
- Import Project Trellis ECP5 architecture
- Unify logical and physical PnR graph
- Initial Texo workspace
