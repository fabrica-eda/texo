# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.3](https://github.com/fabrica-eda/texo/compare/v0.1.2...v0.1.3) (2026-08-26)


### Features

* support native ECP5 open-drain IO ([902334e](https://github.com/fabrica-eda/texo/commit/902334ecc63d441354c85bebe876054ace4e1ead))

## [0.1.2](https://github.com/fabrica-eda/texo/compare/v0.1.1...v0.1.2) (2026-08-25)


### Bug Fixes

* **bitgen:** accept project CLI checkpoints ([c0f2cc8](https://github.com/fabrica-eda/texo/commit/c0f2cc8be2a612bedf11819f885a8baf9412cf4d))

## [0.1.1](https://github.com/fabrica-eda/texo/compare/v0.1.0...v0.1.1) (2026-08-25)


### Features

* **ecp5:** emit native DP16KD configuration ([cb4e1e7](https://github.com/fabrica-eda/texo/commit/cb4e1e7568d92618110b6c1d2ad32db38d593eaa))
* **ecp5:** ship install-free target packs ([987deb9](https://github.com/fabrica-eda/texo/commit/987deb9afaf040fd20c3e2e6fcc4b332e1a2e302))
* **flow:** remove nextpnr bitstream dependency ([868b40f](https://github.com/fabrica-eda/texo/commit/868b40f7240344eacfd33e0c2e29e9b70254f357))
* **flow:** remove nextpnr runtime dependency ([e153f96](https://github.com/fabrica-eda/texo/commit/e153f9668cd3276499a7c011e9c8ef68e74e7b9d))
* **timing:** characterize DP16KD in STA ([e414b4a](https://github.com/fabrica-eda/texo/commit/e414b4a6853a83a8b5be35d6239600d304ba3af0))


### Bug Fixes

* **ci:** generate git-only release PRs without packaging ([6205ebf](https://github.com/fabrica-eda/texo/commit/6205ebf79718779af88467b2e39666b0f3a448e3))
* **ci:** release GitHub artifacts without crates.io ([7a9359d](https://github.com/fabrica-eda/texo/commit/7a9359dde73aadd28b364c19299bd57313ed46e9))
* **release:** pin canonical ECP5 target pack ([be58ce7](https://github.com/fabrica-eda/texo/commit/be58ce707206c385070cfe532579dc7b1a32420d))

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
