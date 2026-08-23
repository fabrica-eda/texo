#!/usr/bin/env python3
"""Build reproducible, checksummed Texo ECP5 architecture release assets."""

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = REPOSITORY / "architectures" / "ecp5" / "manifest.json"


def parse_args():
    parser = argparse.ArgumentParser(
        description="build and package versioned ECP5 .txdb architecture caches"
    )
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument(
        "--device",
        action="append",
        default=[],
        help="build only this device (repeatable; defaults to every manifest device)",
    )
    parser.add_argument(
        "--output-dir", type=Path, default=REPOSITORY / "artifacts" / "architecture"
    )
    parser.add_argument(
        "--texo-bin", type=Path, default=REPOSITORY / "target" / "release" / "texo"
    )
    parser.add_argument(
        "--keep-json", action="store_true", help="retain the intermediate JSON snapshot"
    )
    parser.add_argument(
        "--no-compress", action="store_true", help="do not produce the .txdb.zst asset"
    )
    parser.add_argument(
        "--skip-package-check",
        action="store_true",
        help="allow a non-release Project Trellis installation (development only)",
    )
    parser.add_argument(
        "--force", action="store_true", help="replace matching local output files"
    )
    parser.add_argument(
        "--print-plan", action="store_true", help="validate and print the build plan only"
    )
    return parser.parse_args()


def load_manifest(path):
    with path.open(encoding="utf-8") as source:
        manifest = json.load(source)
    if manifest.get("manifest_version") != 1:
        raise ValueError("unsupported architecture manifest version")
    required = {"architecture_schema_version", "cache_format_version", "source", "devices"}
    missing = sorted(required - manifest.keys())
    if missing:
        raise ValueError(f"architecture manifest is missing: {', '.join(missing)}")
    return manifest


def source_constant(path, pattern, label):
    match = re.search(pattern, path.read_text(encoding="utf-8"))
    if match is None:
        raise ValueError(f"could not find {label} in {path}")
    return int(match.group(1))


def validate_format_versions(manifest):
    exporter_schema = source_constant(
        REPOSITORY / "tools" / "export_ecp5.py",
        r"SCHEMA_VERSION\s*=\s*(\d+)",
        "SCHEMA_VERSION",
    )
    rust_source = REPOSITORY / "crates" / "texo-target-ecp5" / "src" / "lib.rs"
    rust_schema = source_constant(
        rust_source, r"SCHEMA_VERSION:\s*u32\s*=\s*(\d+)", "SCHEMA_VERSION"
    )
    cache_version = source_constant(
        rust_source,
        r"ARCHITECTURE_CACHE_VERSION:\s*u32\s*=\s*(\d+)",
        "ARCHITECTURE_CACHE_VERSION",
    )
    declared_schema = manifest["architecture_schema_version"]
    declared_cache = manifest["cache_format_version"]
    if exporter_schema != declared_schema or rust_schema != declared_schema:
        raise ValueError(
            "manifest architecture_schema_version does not match the exporter and Rust importer"
        )
    if cache_version != declared_cache:
        raise ValueError(
            "manifest cache_format_version does not match ARCHITECTURE_CACHE_VERSION"
        )


def selected_devices(manifest, requested):
    devices = manifest["devices"]
    required = {
        "device",
        "artifact_stem",
        "expected_txdb_bytes",
        "expected_txdb_sha256",
    }
    for entry in devices:
        missing = sorted(required - entry.keys())
        if missing:
            raise ValueError(
                f"device entry is missing {', '.join(missing)}: {entry.get('device', '<unknown>')}"
            )
    names = [entry["device"] for entry in devices]
    if len(names) != len(set(names)):
        raise ValueError("architecture manifest contains a duplicate device")
    if not requested:
        return devices
    unknown = sorted(set(requested) - set(names))
    if unknown:
        raise ValueError(f"device is not in architecture manifest: {', '.join(unknown)}")
    requested_set = set(requested)
    return [entry for entry in devices if entry["device"] in requested_set]


def installed_package_version(package):
    result = subprocess.run(
        ["dpkg-query", "-W", "-f=${Version}", package],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def validate_packages(source):
    for package, expected in source["packages"].items():
        try:
            actual = installed_package_version(package)
        except (FileNotFoundError, subprocess.CalledProcessError) as error:
            raise RuntimeError(f"required package is not installed: {package}={expected}") from error
        if actual != expected:
            raise RuntimeError(
                f"package version mismatch: {package}={actual}, expected {expected}"
            )


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run(command, *, capture=False):
    print("+", " ".join(str(part) for part in command), flush=True)
    return subprocess.run(
        [str(part) for part in command],
        check=True,
        capture_output=capture,
        text=capture,
    )


def texo_revision():
    try:
        revision = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=REPOSITORY,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=REPOSITORY,
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        return revision + ("-dirty" if dirty else "")
    except (FileNotFoundError, subprocess.CalledProcessError):
        return "unknown"


def refuse_existing(paths, force):
    existing = [path for path in paths if path.exists()]
    if existing and not force:
        names = ", ".join(str(path) for path in existing)
        raise FileExistsError(f"output already exists (pass --force to replace): {names}")


def install_output(source, destination, force):
    if destination.exists():
        if not force:
            raise FileExistsError(destination)
        destination.unlink()
    source.replace(destination)


def build_device(args, manifest, entry):
    source = manifest["source"]
    stem = entry["artifact_stem"]
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    final_txdb = output_dir / f"{stem}.txdb"
    final_json = output_dir / f"{stem}.json"
    final_zstd = output_dir / f"{stem}.txdb.zst"
    final_release = output_dir / f"{stem}.release.json"
    final_sums = output_dir / f"{stem}.SHA256SUMS"
    outputs = [final_txdb, final_release, final_sums]
    if args.keep_json:
        outputs.append(final_json)
    if not args.no_compress:
        outputs.append(final_zstd)
    refuse_existing(outputs, args.force)

    with tempfile.TemporaryDirectory(prefix=f".{stem}-", dir=output_dir) as temporary:
        temporary = Path(temporary)
        json_path = temporary / final_json.name
        txdb_path = temporary / final_txdb.name
        zstd_path = temporary / final_zstd.name
        release_path = temporary / final_release.name
        sums_path = temporary / final_sums.name

        export_command = [
            sys.executable,
            REPOSITORY / "tools" / "export_ecp5.py",
            "--database",
            source["database_root"],
            "--device",
            entry["device"],
            "--output",
            json_path,
            "--project-trellis-revision",
            source["project_trellis_revision"],
            "--database-revision",
            source["database_revision"],
        ]
        for directory in source["python_libdirs"]:
            export_command.extend(["-L", directory])
        run(export_command)
        run([args.texo_bin, "cache-architecture", json_path, txdb_path])
        inspection = run([args.texo_bin, "target-info", txdb_path], capture=True).stdout
        print(inspection, end="")
        for expected in (
            f"device: {entry['device']}",
            f"Project Trellis revision: {source['project_trellis_revision']}",
            f"database revision: {source['database_revision']}",
        ):
            if expected not in inspection:
                raise RuntimeError(f"generated cache inspection omitted: {expected}")

        txdb_digest = sha256(txdb_path)
        txdb_bytes = txdb_path.stat().st_size
        if txdb_bytes != entry["expected_txdb_bytes"]:
            raise RuntimeError(
                f"generated cache size is {txdb_bytes}, expected {entry['expected_txdb_bytes']}"
            )
        if txdb_digest != entry["expected_txdb_sha256"]:
            raise RuntimeError(
                f"generated cache SHA-256 is {txdb_digest}, "
                f"expected {entry['expected_txdb_sha256']}"
            )
        release_files = []
        if not args.no_compress:
            run(["zstd", "-10", "-T0", "--force", txdb_path, "-o", zstd_path])
            release_files.append(
                {
                    "name": zstd_path.name,
                    "bytes": zstd_path.stat().st_size,
                    "sha256": sha256(zstd_path),
                }
            )

        release = {
            "release_manifest_version": 1,
            "device": entry["device"],
            "architecture_schema_version": manifest["architecture_schema_version"],
            "cache_format_version": manifest["cache_format_version"],
            "provenance": {
                "project_trellis_revision": source["project_trellis_revision"],
                "database_revision": source["database_revision"],
                "distribution": source["distribution"],
                "packages": source["packages"],
            },
            "producer": {"texo_revision": texo_revision()},
            "cache": {
                "name": txdb_path.name,
                "bytes": txdb_bytes,
                "sha256": txdb_digest,
            },
            "release_files": release_files,
        }
        with release_path.open("w", encoding="utf-8") as destination:
            json.dump(release, destination, indent=2, sort_keys=True)
            destination.write("\n")

        distributed = [zstd_path] if not args.no_compress else [txdb_path]
        distributed.append(release_path)
        with sums_path.open("w", encoding="utf-8") as destination:
            for path in distributed:
                destination.write(f"{sha256(path)}  {path.name}\n")

        install_output(txdb_path, final_txdb, args.force)
        if args.keep_json:
            install_output(json_path, final_json, args.force)
        if not args.no_compress:
            install_output(zstd_path, final_zstd, args.force)
        install_output(release_path, final_release, args.force)
        install_output(sums_path, final_sums, args.force)
    print(f"built {final_txdb}")


def main():
    args = parse_args()
    manifest = load_manifest(args.manifest)
    validate_format_versions(manifest)
    devices = selected_devices(manifest, args.device)
    if args.print_plan:
        plan = {
            "manifest": str(args.manifest.resolve()),
            "output_dir": str(args.output_dir.resolve()),
            "devices": devices,
            "source": manifest["source"],
        }
        print(json.dumps(plan, indent=2, sort_keys=True))
        return
    if not args.skip_package_check:
        validate_packages(manifest["source"])
    if not args.texo_bin.is_file():
        raise FileNotFoundError(
            f"Texo binary not found: {args.texo_bin}; run cargo build --release -p texo-cli"
        )
    if not args.no_compress and shutil.which("zstd") is None:
        raise FileNotFoundError("zstd is required to build the release asset")
    for entry in devices:
        build_device(args, manifest, entry)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1) from error
