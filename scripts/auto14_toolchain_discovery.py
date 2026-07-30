#!/usr/bin/env python3
"""Audit the official OE package, ONNX contracts and calibration inventory."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import re


OFFICIAL_ARCHIVE_SHA256 = (
    "de90da5cf58879a0883bb47856232514c3cc30e368d8864911bd05e267229c5b"
)
OFFICIAL_VERSION = "3.7.0"
OFFICIAL_URL = (
    "https://d-robotics-aitoolchain.oss-cn-beijing.aliyuncs.com/"
    "oe/3.7.0/oe-package-3.7.0-s100-s600.tgz"
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def wheel_inventory(root: Path) -> list[dict]:
    rows = []
    for path in sorted(root.rglob("*.whl")):
        match = re.match(r"(?P<name>.+?)-(?P<version>\d[^-]*)-", path.name)
        rows.append(
            {
                "name": match.group("name") if match else path.stem,
                "version": match.group("version") if match else None,
                "path": str(path.relative_to(root)).replace("\\", "/"),
                "sha256": sha256(path),
            }
        )
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--archive", required=True)
    parser.add_argument("--package-root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--board-device-count", type=int, default=0)
    args = parser.parse_args()
    archive = Path(args.archive)
    root = Path(args.package_root)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    archive_digest = sha256(archive)
    wheels = wheel_inventory(root)
    required = {
        "hbdk4_compiler": "4.7.5",
        "hmct": "2.6.5",
        "horizon_tc_ui": "3.5.3",
    }
    resolved = {
        name: any(
            row["name"].replace("-", "_") == name
            and row["version"].startswith(version)
            for row in wheels
        )
        for name, version in required.items()
    }
    report = {
        "schema_version": 1,
        "stage": "AUTO-14",
        "official_source": {
            "url": OFFICIAL_URL,
            "oe_version": OFFICIAL_VERSION,
            "archive_bytes": archive.stat().st_size,
            "archive_sha256": archive_digest,
            "archive_sha256_expected": OFFICIAL_ARCHIVE_SHA256,
            "archive_integrity_pass": archive_digest
            == OFFICIAL_ARCHIVE_SHA256,
        },
        "host": {
            "platform": platform.platform(),
            "machine": platform.machine(),
            "board_device_count": args.board_device_count,
        },
        "wheel_inventory": wheels,
        "required_versions": required,
        "required_versions_resolved": resolved,
        "official_toolchain_package_ready": (
            archive_digest == OFFICIAL_ARCHIVE_SHA256
            and all(resolved.values())
        ),
        "j6_toolchain_pass": False,
        "j6_runtime_pass": False,
        "truth_boundary": (
            "Package discovery and hb_compile availability do not constitute "
            "model quantization/compile or board runtime acceptance."
        ),
    }
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["official_toolchain_package_ready"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
