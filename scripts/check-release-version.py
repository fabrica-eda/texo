#!/usr/bin/env python3
"""Validate Texo's lockstep crates.io release version.

The workspace root package (`texo-release-metadata`) carries the release
version. With `--print-version`, print it and exit without validating.
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
ANCHOR = "texo-release-metadata"
PUBLIC_CRATES = {
    "texo-cli",
    "texo-flow",
    "texo-model",
    "texo-pnr",
    "texo-struo",
    "texo-target-ecp5",
    "texo-timing",
}
SEMVER = re.compile(r"^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)$")


def fail(message: str) -> None:
    raise SystemExit(message)


metadata = json.loads(
    subprocess.check_output(
        ["cargo", "metadata", "--locked", "--no-deps", "--format-version", "1"],
        cwd=ROOT,
        text=True,
    )
)
packages = {package["name"]: package for package in metadata["packages"]}
version = packages[ANCHOR]["version"]
if "--print-version" in sys.argv[1:]:
    print(version)
    raise SystemExit(0)
if not SEMVER.fullmatch(version):
    fail(f"{ANCHOR} must carry a stable SemVer version, got {version!r}")

published = {
    name for name, package in packages.items() if package.get("publish") == ["crates-io"]
}
if published != PUBLIC_CRATES:
    fail(
        "public crate set differs from the release policy: "
        f"missing={sorted(PUBLIC_CRATES - published)}, "
        f"unexpected={sorted(published - PUBLIC_CRATES)}"
    )

for name in sorted(PUBLIC_CRATES):
    package = packages[name]
    if package["version"] != version:
        fail(f"{name} has version {package['version']}; expected {version}")
    for dependency in package["dependencies"]:
        dependency_name = dependency["name"]
        if dependency_name in PUBLIC_CRATES and dependency["req"] != f"={version}":
            fail(
                f"{name} requires {dependency_name} {dependency['req']}; "
                f"expected ={version}"
            )
        if (dependency.get("source") or "").startswith("git+"):
            fail(f"{name} depends on {dependency_name} from Git; crates.io cannot resolve it")

manifest = json.loads((ROOT / ".release-please-manifest.json").read_text(encoding="utf-8"))
if manifest.get(".") != version:
    fail(f".release-please-manifest.json has {manifest.get('.')}; expected {version}")

lockfile = (ROOT / "Cargo.lock").read_text(encoding="utf-8")
for block in lockfile.split("[[package]]")[1:]:
    name_match = re.search(r'^\s*name = "([^"]+)"', block, re.MULTILINE)
    version_match = re.search(r'^\s*version = "([^"]+)"', block, re.MULTILINE)
    if name_match and name_match.group(1) in PUBLIC_CRATES | {ANCHOR}:
        if version_match is None or version_match.group(1) != version:
            candidate = version_match.group(1) if version_match else "missing"
            fail(f"Cargo.lock has {name_match.group(1)} {candidate}; expected {version}")

print(f"Validated stable Texo version {version} across {len(PUBLIC_CRATES)} crates")
