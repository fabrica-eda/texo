# Release policy

Texo's public Rust packages are released as one lockstep distribution. The
root `texo-release-metadata` version, every `texo-*` package, exact internal
dependency requirements, `Cargo.lock`, and `.release-please-manifest.json` must
carry the same version. `./scripts/check-release-version.py` enforces this.

The public entry point is `texo-cli`, which installs the `texo` binary with
`cargo install texo-cli`. The remaining published `texo-*` crates preserve the
internal dependency graph and are implementation details unless their
documentation says otherwise. Published crates may depend only on crates.io
packages; Struo and Celox are consumed from their crates.io releases.

## Publication workflow

Stable releases are published from `.github/workflows/publish-crates.yml` using
crates.io Trusted Publishing. The workflow must be dispatched from a stable tag
such as `v0.1.0`, and that tag must have a non-draft, non-prerelease GitHub
Release. It validates every package, runs the workspace tests and Rustdoc, then
publishes in dependency order. Retrying is safe because versions already visible
on crates.io are skipped.

The `crates-io` GitHub environment owns the deployment boundary. Restrict it to
protected tags matching `v*` and add required reviewers if publication should
require explicit approval. The publish job exchanges its GitHub OIDC identity
for a temporary crates.io token; do not store a long-lived
`CARGO_REGISTRY_TOKEN` in GitHub.

For each release:

1. Merge the release-please PR. It bumps every crate, the marked internal
   requirements in `Cargo.toml`, and `.release-please-manifest.json`.
2. On that push, release-plz creates the matching `vX.Y.Z` tag and GitHub
   Release, and the release workflow uploads the CLI binaries.
3. Run **Publish Rust Crates** from that tag.

## First crates.io release

crates.io cannot configure a Trusted Publisher for an unused crate name. The
first release therefore bootstraps each name with an intentionally empty
`0.0.0` placeholder. This is a one-time local operation and requires a
short-expiry crates.io API token with `publish-new` and **Manage trusted
publishing configurations** permissions.

After this release setup is merged and the `crates-io` GitHub environment
exists, validate the generated placeholders:

```bash
./scripts/bootstrap-crates-io.sh package
```

Pass the token without writing it to shell history, then perform the permanent
bootstrap:

```bash
read -rsp 'crates.io bootstrap token: ' CARGO_REGISTRY_TOKEN
echo
export CARGO_REGISTRY_TOKEN
./scripts/bootstrap-crates-io.sh publish BOOTSTRAP-TEXO-CRATES
unset CARGO_REGISTRY_TOKEN
```

For every public crate, the script publishes `0.0.0` and registers this exact
Trusted Publisher identity:

| Field | Value |
| --- | --- |
| GitHub owner | `fabrica-eda` |
| GitHub repository | `texo` |
| Workflow filename | `publish-crates.yml` |
| Environment | `crates-io` |

Re-running the bootstrap skips completed names and matching configurations. It
stops on an existing crate without the expected placeholder or on an unexpected
Trusted Publisher configuration. Revoke the bootstrap token immediately after
completion and add any intended backup owners on crates.io.

Once all names are bootstrapped, create the first stable tag and GitHub Release,
then run the normal Trusted Publishing workflow. Optionally enable **Trusted
Publishing Only** for each crate after the stable release succeeds.
