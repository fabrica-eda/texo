#!/usr/bin/env python3
"""Package an install-free Texo ECP5 target runtime as a zstd archive."""

import argparse
import gzip
import hashlib
import json
import os
import platform
import shutil
import struct
import subprocess
import tempfile
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = REPOSITORY / "architectures" / "ecp5" / "manifest.json"
DEFAULT_CATALOG = REPOSITORY / "architectures" / "ecp5" / "catalog.json"
MAGIC = b"TEXO_TARGET_PACK\n"
FORMAT_VERSION = 1


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--device", default="LFE5UM5G-85F")
    parser.add_argument("--architecture", type=Path)
    parser.add_argument("--database", type=Path, default=Path("/usr/share/trellis/database"))
    parser.add_argument(
        "--base-config-root", type=Path, default=Path("/usr/share/doc/fpga-trellis/basecfgs")
    )
    parser.add_argument("--ecppack", type=Path, default=Path(shutil.which("ecppack") or "ecppack"))
    parser.add_argument("--project-trellis-license", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def platform_name():
    machine = platform.machine()
    if platform.system() != "Linux" or machine not in {"x86_64", "aarch64"}:
        raise RuntimeError(f"unsupported target-pack host: {platform.system()} {machine}")
    return f"{machine}-unknown-linux-gnu"


def device_entry(manifest, device):
    for entry in manifest["devices"]:
        if entry["device"] == device:
            return entry
    raise RuntimeError(f"device is absent from architecture manifest: {device}")


def catalog_entry(catalog, device, platform_id):
    for entry in catalog["targets"]:
        if entry["device"] == device and entry["platform"] == platform_id:
            return entry
    raise RuntimeError(f"target is absent from catalog: {device} {platform_id}")


def find_base_config(root, device):
    stem = f"empty_{device.lower()}.config"
    for path in (root / stem, root / f"{stem}.gz"):
        if path.is_file():
            return path
    raise RuntimeError(f"base config is absent for {device}")


def dynamic_libraries(executable):
    result = subprocess.run(
        ["ldd", executable], check=True, capture_output=True, text=True
    )
    libraries = []
    for line in result.stdout.splitlines():
        stripped = line.strip()
        if " => " not in stripped:
            continue
        name, remainder = stripped.split(" => ", 1)
        path = Path(remainder.split(" ", 1)[0])
        if name == "libtrellis.so" or name.startswith("libboost_"):
            libraries.append(path.resolve())
    if not any(path.name == "libtrellis.so" for path in libraries):
        raise RuntimeError("ecppack does not resolve libtrellis.so")
    return sorted(set(libraries))


def copy_file(source, root, relative, executable=False):
    destination = root / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, destination)
    destination.chmod(0o755 if executable else 0o644)


def stage_pack(args, manifest, architecture, root):
    entry = device_entry(manifest, args.device)
    copy_file(architecture, root, "architecture.txdb")
    base = find_base_config(args.base_config_root, args.device)
    destination = root / "base.config"
    if base.suffix == ".gz":
        with gzip.open(base, "rb") as source, destination.open("wb") as output:
            shutil.copyfileobj(source, output)
    else:
        shutil.copyfile(base, destination)
    destination.chmod(0o644)
    device_root = args.database / "ECP5" / args.device
    copy_file(device_root / "iodb.json", root, "iodb.json")
    copy_file(args.database / "devices.json", root, "database/devices.json")
    copy_file(device_root / "tilegrid.json", root, f"database/ECP5/{args.device}/tilegrid.json")
    copy_file(device_root / "globals.json", root, f"database/ECP5/{args.device}/globals.json")
    for bits in sorted((args.database / "ECP5" / "tiledata").glob("*/bits.db")):
        copy_file(bits, root, f"database/ECP5/tiledata/{bits.parent.name}/bits.db")
    copy_file(args.database / "COPYING", root, "licenses/project-trellis-database-CC0.txt")
    trellis_license = args.project_trellis_license
    if trellis_license is None:
        candidate = Path("/usr/share/doc/fpga-trellis/copyright")
        if candidate.is_file():
            trellis_license = candidate
    if trellis_license is not None and trellis_license.is_file():
        copy_file(trellis_license, root, "licenses/project-trellis.txt")
    boost_license = Path("/usr/share/doc/libboost-filesystem1.83.0/copyright")
    if boost_license.is_file():
        copy_file(boost_license, root, "licenses/boost.txt")
    copy_file(args.ecppack.resolve(), root, "bin/ecppack", executable=True)
    for library in dynamic_libraries(str(args.ecppack.resolve())):
        copy_file(library, root, f"lib/{library.name}", executable=True)

    metadata = {
        "format_version": FORMAT_VERSION,
        "device": args.device,
        "platform": platform_name(),
        "architecture_schema_version": manifest["architecture_schema_version"],
        "cache_format_version": manifest["cache_format_version"],
        "architecture": {
            "bytes": architecture.stat().st_size,
            "sha256": sha256(architecture),
        },
        "project_trellis_revision": manifest["source"]["project_trellis_revision"],
        "database_revision": manifest["source"]["database_revision"],
        "artifact_stem": entry["artifact_stem"],
    }
    (root / "manifest.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def write_archive(root, output):
    files = sorted(path for path in root.rglob("*") if path.is_file())
    with output.open("wb") as archive:
        archive.write(MAGIC)
        archive.write(struct.pack("<I", FORMAT_VERSION))
        for path in files:
            relative = path.relative_to(root).as_posix().encode()
            mode = path.stat().st_mode & 0o777
            archive.write(struct.pack("<I", len(relative)))
            archive.write(relative)
            archive.write(struct.pack("<IQ", mode, path.stat().st_size))
            archive.write(bytes.fromhex(sha256(path)))
            with path.open("rb") as source:
                shutil.copyfileobj(source, archive, 1024 * 1024)
        archive.write(struct.pack("<I", 0))


def main():
    args = parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    catalog = json.loads(args.catalog.read_text(encoding="utf-8"))
    entry = device_entry(manifest, args.device)
    architecture = args.architecture or (
        REPOSITORY / "artifacts" / "architecture" / f"{entry['artifact_stem']}.txdb"
    )
    if not architecture.is_file():
        raise RuntimeError(f"architecture cache is absent: {architecture}")
    if architecture.stat().st_size != entry["expected_txdb_bytes"] or sha256(architecture) != entry["expected_txdb_sha256"]:
        raise RuntimeError("architecture cache does not match the pinned manifest")
    platform_id = platform_name()
    published = catalog_entry(catalog, args.device, platform_id)
    output = args.output or (
        REPOSITORY
        / "artifacts"
        / "architecture"
        / f"texo-{args.device}-schema{manifest['architecture_schema_version']}-cache{manifest['cache_format_version']}-{platform_id}.txpkg.zst"
    )
    if output.exists() and not args.force:
        raise RuntimeError(f"target pack already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="texo-target-pack-") as temporary:
        temporary = Path(temporary)
        stage = temporary / "stage"
        stage.mkdir()
        stage_pack(args, manifest, architecture, stage)
        raw = temporary / "target.txpkg"
        write_archive(stage, raw)
        subprocess.run(
            ["zstd", "-10", "-T0", "--force", str(raw), "-o", str(output)],
            check=True,
        )
    digest = sha256(output)
    if output.name != published["asset"] and args.output is None:
        raise RuntimeError(
            f"generated asset name {output.name} does not match catalog {published['asset']}"
        )
    if output.stat().st_size != published["bytes"] or digest != published["sha256"]:
        raise RuntimeError(
            "target pack does not match the pinned catalog: "
            f"bytes={output.stat().st_size}, sha256={digest}"
        )
    sums = output.with_name(f"{output.name}.SHA256SUMS")
    sums.write_text(f"{digest}  {output.name}\n", encoding="utf-8")
    print(f"target pack: {output}")
    print(f"bytes: {output.stat().st_size}")
    print(f"sha256: {digest}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, subprocess.CalledProcessError, ValueError) as error:
        print(f"error: {error}", file=os.sys.stderr)
        raise SystemExit(1)
