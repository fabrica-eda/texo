# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.6](https://github.com/fabrica-eda/texo/compare/v0.1.5...v0.1.6) (2026-08-31)


### Features

* **ecp5:** import true dual-port Struo memories ([#50](https://github.com/fabrica-eda/texo/issues/50)) ([b7fb068](https://github.com/fabrica-eda/texo/commit/b7fb068895760116304e59e4783a49173bb847c9))
* **ecp5:** support Struo LUT7 clusters ([#39](https://github.com/fabrica-eda/texo/issues/39)) ([efbb541](https://github.com/fabrica-eda/texo/commit/efbb5417534275874d8199c14fde9956a80da557))
* **ecp5:** support Struo wide LUTs ([#37](https://github.com/fabrica-eda/texo/issues/37)) ([04f14cf](https://github.com/fabrica-eda/texo/commit/04f14cf4458d9693bce068b04cb9c146b37ad74b))


### Bug Fixes

* **ecp5:** export wide LUT timing records ([#38](https://github.com/fabrica-eda/texo/issues/38)) ([35cb8aa](https://github.com/fabrica-eda/texo/commit/35cb8aa615a4aab46453bcc4fbf09c584084583f))
* **ecp5:** import true-dual-port clock enables ([#51](https://github.com/fabrica-eda/texo/issues/51)) ([6ea8909](https://github.com/fabrica-eda/texo/commit/6ea890922e32102c82f3734f8571bc041802219c))
* **ecp5:** model clock edges in STA ([#45](https://github.com/fabrica-eda/texo/issues/45)) ([2136749](https://github.com/fabrica-eda/texo/commit/2136749d557829cd8fa23795bc5afcee6b9fbf16))
* **pnr:** close routed placement without unsafe rewrites ([#55](https://github.com/fabrica-eda/texo/issues/55)) ([12c395a](https://github.com/fabrica-eda/texo/commit/12c395ad65f3843a9faed69d11eefdfedd77d591))
* **pnr:** escape repeated routing victim stalls ([#40](https://github.com/fabrica-eda/texo/issues/40)) ([10606c7](https://github.com/fabrica-eda/texo/commit/10606c7fd9384affd3f6b7df20b40ff84146e455))
* **pnr:** iterate anchored timing placement ([#43](https://github.com/fabrica-eda/texo/issues/43)) ([ae71d30](https://github.com/fabrica-eda/texo/commit/ae71d30502ddfed17517326bad402033762b88c8))
* **pnr:** preserve critical high-fanout placement edges ([#49](https://github.com/fabrica-eda/texo/issues/49)) ([ac29f25](https://github.com/fabrica-eda/texo/commit/ac29f25917bdd2649a8f60861b3bc37b6e95773e))
* **pnr:** preserve WNS gains during setup closure ([#35](https://github.com/fabrica-eda/texo/issues/35)) ([6542359](https://github.com/fabrica-eda/texo/commit/6542359f76b22da2194fab0222f405d2c26a684d))
* **pnr:** reweight analytical placement wirelength ([#47](https://github.com/fabrica-eda/texo/issues/47)) ([0d350f7](https://github.com/fabrica-eda/texo/commit/0d350f7de2242c9efdca186a2d6a0e22cb51992b))
* **release:** pin ECP5 v4 target pack ([#41](https://github.com/fabrica-eda/texo/issues/41)) ([18b839e](https://github.com/fabrica-eda/texo/commit/18b839e086a12dec6b653d7c04d821fe78a45968))
* **sta:** match nextpnr ECP5 timing margin ([#46](https://github.com/fabrica-eda/texo/issues/46)) ([bcc53c6](https://github.com/fabrica-eda/texo/commit/bcc53c632a3dc79e0baf30205e30e8078ff1c3ef))
* **sta:** reserve calibrated ECP5 setup uncertainty ([#44](https://github.com/fabrica-eda/texo/issues/44)) ([4023fa2](https://github.com/fabrica-eda/texo/commit/4023fa2d220c903eaab4ac0b4177a96cffed6be2))


### Performance Improvements

* **pnr:** add routability-aware ECP5 timing closure ([#56](https://github.com/fabrica-eda/texo/issues/56)) ([946a684](https://github.com/fabrica-eda/texo/commit/946a684cd744c8843ce58b844842effa25d35dc5))
* **pnr:** drive placement with pre-route timing ([#54](https://github.com/fabrica-eda/texo/issues/54)) ([18b4932](https://github.com/fabrica-eda/texo/commit/18b4932f54d263d912fe7da65d9edee0f36d566e))
* **pnr:** stop redundant timing closure trials ([#52](https://github.com/fabrica-eda/texo/issues/52)) ([0142ffd](https://github.com/fabrica-eda/texo/commit/0142ffd08c26976a3f1a224de98bc6c62a34f721))

## [0.1.5](https://github.com/fabrica-eda/texo/compare/v0.1.4...v0.1.5) (2026-08-27)


### Features

* **cli:** expose end-to-end bitgen API ([#21](https://github.com/fabrica-eda/texo/issues/21)) ([c3b1cb3](https://github.com/fabrica-eda/texo/commit/c3b1cb3ff24a42d9d04e971b85e8a23d930563e5))
* **ecp5:** support user-configured PLL bindings ([#24](https://github.com/fabrica-eda/texo/issues/24)) ([430a3a8](https://github.com/fabrica-eda/texo/commit/430a3a84d51579d58cc0807b0502d7527991ffa0))


### Bug Fixes

* **ecp5:** emit valid JTAGG configuration ([#20](https://github.com/fabrica-eda/texo/issues/20)) ([a59f787](https://github.com/fabrica-eda/texo/commit/a59f787de1a7608cdbeb8b652a71c017e3bce001))
* **ecp5:** insert routed carry feed-ins ([#27](https://github.com/fabrica-eda/texo/issues/27)) ([3c01ca5](https://github.com/fabrica-eda/texo/commit/3c01ca568f7184b6700b0ce9aefa72fe249647ea))
* **ecp5:** preserve dedicated routing across clocks and ECOs ([#25](https://github.com/fabrica-eda/texo/issues/25)) ([299836d](https://github.com/fabrica-eda/texo/commit/299836dd8ee35afe2ce616d8dddb77c9973d3918))
* **ecp5:** preserve legal JTAG and carry configuration ([#23](https://github.com/fabrica-eda/texo/issues/23)) ([522d7ca](https://github.com/fabrica-eda/texo/commit/522d7cafc0af9ed4b082925b84f0b418a7aa70d8))
* **pnr:** stabilize placement across processes ([#29](https://github.com/fabrica-eda/texo/issues/29)) ([077453c](https://github.com/fabrica-eda/texo/commit/077453cf065902af8b1e56a3a472e4ce9a3fd4b8))
* **timing:** harden CPPR and endpoint coverage ([#34](https://github.com/fabrica-eda/texo/issues/34)) ([b452fb7](https://github.com/fabrica-eda/texo/commit/b452fb7ac60c22a43a9f1dc40b0d17e94c9e258f))
* **timing:** remove common clock path pessimism ([#32](https://github.com/fabrica-eda/texo/issues/32)) ([22825cd](https://github.com/fabrica-eda/texo/commit/22825cd47e3ffc1beea8de8fc07f65d02803d033))


### Performance Improvements

* **pnr:** add timing-driven placement portfolio ([#28](https://github.com/fabrica-eda/texo/issues/28)) ([bdfb4ac](https://github.com/fabrica-eda/texo/commit/bdfb4acb5aad08e91e3dea3a5fcbc84efd3c3ae7))
* **pnr:** bound speculative timing closure search ([#31](https://github.com/fabrica-eda/texo/issues/31)) ([d776ddc](https://github.com/fabrica-eda/texo/commit/d776ddc0692257024a81ffb532526a3d80bffa74))

## [0.1.4](https://github.com/fabrica-eda/texo/compare/v0.1.3...v0.1.4) (2026-08-26)


### Features

* **ecp5:** support dedicated JTAGG flow ([1fff703](https://github.com/fabrica-eda/texo/commit/1fff70389e89f98fbab79d43d62840412b698d4d))
* **ecp5:** support dedicated JTAGG flow ([c3f2adf](https://github.com/fabrica-eda/texo/commit/c3f2adfc8aff0adde0276f1938f449243ba2f0ae))


### Bug Fixes

* **ci:** serialize release changelog generation ([20a0189](https://github.com/fabrica-eda/texo/commit/20a01890425a15f354fd7363fdaac28df543015c))
* **ci:** serialize release changelog generation ([58d7de8](https://github.com/fabrica-eda/texo/commit/58d7de8963101d2244301d0b4da918ad3ad4ef9a))

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
