# Architecture database lifecycle

Texo's `.txdb` files are versioned build artifacts, not disposable local
caches. They contain the expanded routing graph and timing tables, so the
inputs and the cache format must be identifiable before a file can be used or
distributed.

Schema 6 includes the Project Trellis `DP16KD` `REGMODE_A/B=NOREG`
clock-to-output and setup/hold characterization used by Texo's block-RAM STA.

## Source of truth

[`architectures/ecp5/manifest.json`](../architectures/ecp5/manifest.json) pins:

- the architecture JSON schema and binary cache format;
- the Project Trellis Python and database package versions;
- the provenance strings embedded in the cache;
- every device built for a release, its stable artifact name, and the expected
  uncompressed byte size and SHA-256 digest.

Changing a source package, exporter schema, or binary cache format requires a
manifest change. A cache-format change also requires a new artifact name. Do
not commit production `.json` or `.txdb` files to Git.

## Reproduce a cache

The release build currently uses Ubuntu 24.04 and its pinned Project Trellis
packages:

```sh
sudo apt-get install \
  fpga-trellis=1.4-2build4 \
  fpga-trellis-database=1.4-2build4 \
  python3-pytrellis=1.4-2build4 \
  libboost-filesystem1.83.0=1.83.0-2.1ubuntu3.2 \
  libboost-program-options1.83.0=1.83.0-2.1ubuntu3.2 \
  libboost-thread1.83.0=1.83.0-2.1ubuntu3.2 \
  zstd=1.5.5+dfsg2-2build1.1
cargo build --release --locked -p texo-cli
/usr/bin/python3 tools/build_ecp5_txdb.py --device LFE5UM5G-85F
/usr/bin/python3 tools/build_ecp5_target_pack.py --device LFE5UM5G-85F
```

The builder checks the installed package versions and checks that the schema
and cache constants still match the tracked manifest. It then exports the
deduplicated Project Trellis graph, creates the Postcard `.txdb`, reads it back
with `texo target-info`, and rejects it unless its byte size and SHA-256 match
the tracked expected result. A successful build emits:

```text
texo-LFE5UM5G-85F-schema6-cache5.txdb
texo-LFE5UM5G-85F-schema6-cache5.txdb.zst
texo-LFE5UM5G-85F-schema6-cache5.release.json
texo-LFE5UM5G-85F-schema6-cache5.SHA256SUMS
texo-LFE5UM5G-85F-schema6-cache5-x86_64-unknown-linux-gnu.txpkg.zst
```

The release manifest records the uncompressed cache digest and size as well as
the exact Texo revision and Project Trellis provenance. `--keep-json` retains
the intermediate architecture JSON for debugging. `--skip-package-check` is
only for development against a non-release Project Trellis build; artifacts
made with it must not be published.

## Publish and consume

Pushing a `txdb-ecp5-v*` tag runs the architecture release workflow.
The workflow builds in the pinned environment, uploads an Actions artifact,
and creates a GitHub Release containing the compressed cache, target pack,
release manifest, and checksums. The workflow refuses to replace an existing
release or its assets. Runtime `texo pnr` and `texo bitgen` resolve the target
pack from the embedded catalog, verify its release SHA-256, safely unpack it
once, and use the cache thereafter. Project Trellis is a release-build input,
not something an end user installs.

Download and verify a release in an empty directory:

```sh
gh release download txdb-ecp5-v3 \
  -p 'texo-LFE5UM5G-85F-schema6-cache5.*'
sha256sum -c texo-LFE5UM5G-85F-schema6-cache5.SHA256SUMS
zstd -d texo-LFE5UM5G-85F-schema6-cache5.txdb.zst
cargo run --release -- target-info \
  texo-LFE5UM5G-85F-schema6-cache5.txdb
```

Both checks matter: SHA-256 confirms that the downloaded bytes match the
release checksums, while `target-info` rejects an unsupported binary cache
version and displays the embedded device and source provenance.

Normal users instead run `texo target fetch LFE5UM5G-85F`, or simply let the
first `texo pnr`/`texo bitgen` fetch it. For offline installation use
`texo target install <archive.txpkg.zst>`. Set `TEXO_TARGET_DIR` to override the
platform cache root.
