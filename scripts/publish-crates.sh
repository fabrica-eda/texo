#!/usr/bin/env bash
set -euo pipefail

mode="${1:-package}"
if [[ "$mode" != "list" && "$mode" != "package" && "$mode" != "publish" ]]; then
  echo "usage: $0 [list|package|publish]" >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/.." && pwd)"
cd "$repo_root"

version="$("$script_dir/check-release-version.py" --print-version)"
if [[ ! "$version" =~ ^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)$ ]]; then
  echo "the workspace version must be a stable SemVer version, got $version" >&2
  exit 1
fi

# crates.io resolves path dependencies from the registry during packaging, so
# every crate must follow all of its normal and development dependencies.
crates=(
  texo-model
  texo-pnr
  texo-timing
  texo-struo
  texo-target-ecp5
  texo-flow
  texo-cli
)

if [[ "$mode" == "list" ]]; then
  printf '%s\n' "${crates[@]}"
  exit 0
fi

"$script_dir/check-release-version.py"

# Fail before uploading anything if a crate needs an internal crate (of any
# dependency kind) that the order above publishes later.
cargo metadata --locked --no-deps --format-version 1 | python3 -c '
import json, sys
order = sys.argv[1:]
packages = {p["name"]: p for p in json.load(sys.stdin)["packages"]}
for index, name in enumerate(order):
    for dependency in packages[name]["dependencies"]:
        dep = dependency["name"]
        if dep in order and order.index(dep) >= index:
            kind = dependency["kind"] or "normal"
            sys.exit(f"{name} has a {kind} dependency on {dep}, which is not published before it")
' "${crates[@]}"

if [[ "$mode" == "publish" ]]; then
  : "${CARGO_REGISTRY_TOKEN:?CARGO_REGISTRY_TOKEN is required for publication}"
fi

crate_exists() {
  local crate="$1"
  curl --fail --silent --show-error \
    --user-agent "texo-release-workflow/$version (https://github.com/fabrica-eda/texo)" \
    "https://crates.io/api/v1/crates/$crate/$version" \
    >/dev/null 2>&1
}

wait_for_crate() {
  local crate="$1"
  for _ in {1..12}; do
    if crate_exists "$crate"; then
      return 0
    fi
    sleep 5
  done
  echo "$crate@$version was published but did not become visible on crates.io" >&2
  return 1
}

if [[ "$mode" == "package" ]]; then
  # crates.io may not have this version of the sibling crates yet, so package
  # them all together, then type-check every target of each archive with the
  # unpublished siblings patched to their own archives. Siblings already on
  # crates.io at this version come from there, as they will when publishing.
  # This catches files a crate reads from outside its package, or API it needs
  # from a sibling that was published without it, before anything is uploaded.
  package_args=()
  for crate in "${crates[@]}"; do
    package_args+=(-p "$crate")
  done
  cargo package --locked --allow-dirty --no-verify "${package_args[@]}"
  # Outside the repository, so Cargo does not treat the archives as members of
  # this workspace.
  sources="$(mktemp -d)"
  trap 'rm -rf "$sources"' EXIT
  patches=()
  unpublished=()
  for crate in "${crates[@]}"; do
    if crate_exists "$crate"; then
      echo "$crate@$version is already published; checking dependents against it"
      continue
    fi
    tar -xzf "$repo_root/target/package/$crate-$version.crate" -C "$sources"
    # The packaged lockfile records checksums from Cargo's temporary registry.
    rm -f "$sources/$crate-$version/Cargo.lock"
    patches+=(--config "patch.crates-io.$crate.path=\"$sources/$crate-$version\"")
    unpublished+=("$crate")
  done
  for crate in "${unpublished[@]}"; do
    echo "checking all targets of the $crate@$version archive"
    cargo check \
      --quiet \
      --all-targets \
      --manifest-path "$sources/$crate-$version/Cargo.toml" \
      --target-dir "$repo_root/target/package-checks" \
      "${patches[@]}"
  done
  exit 0
fi

for crate in "${crates[@]}"; do

  if crate_exists "$crate"; then
    echo "$crate@$version is already published; skipping"
    continue
  fi

  echo "building and checking package archive for $crate@$version"
  cargo package --locked -p "$crate"
  package_dir="$repo_root/target/package/$crate-$version"
  if [[ ! -f "$package_dir/Cargo.toml" || -L "$package_dir" ]]; then
    echo "cargo did not create the expected package directory: $package_dir" >&2
    exit 1
  fi
  cargo check \
    --locked \
    --all-targets \
    --manifest-path "$package_dir/Cargo.toml" \
    --target-dir "$repo_root/target/package-checks"
  cargo publish --locked -p "$crate"
  wait_for_crate "$crate"
done
