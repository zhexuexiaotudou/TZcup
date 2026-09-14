#!/usr/bin/env python3
"""Prepare a fixed-start runtime map from the frozen offline raycast source."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "starter_ws/src/sanitation_formal_campus_integration"
LF_TEXT_SUFFIXES = frozenset({".json", ".jsonl", ".yaml", ".yml"})
sys.path.insert(0, str(PACKAGE_ROOT))

from sanitation_formal_campus_integration.offline_map_source import (  # noqa: E402
    OFFLINE_MAP_SOURCE_LABEL,
    OFFLINE_RAYCAST_MAPPING,
    RUNTIME_BINDING_FILE,
    sha256_bytes,
    validate_frozen_offline_raycast_map_source,
)


def _read_json(path: Path, label: str) -> dict:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{label} must be a regular file: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{label} is not valid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{label} JSON root must be an object")
    return value


def _fixed_start(episode_manifest: Path) -> tuple[float, float, float]:
    manifest = _read_json(episode_manifest, "episode manifest")
    raw = manifest.get("vehicle_start_pose_source_world")
    if not isinstance(raw, dict):
        raise ValueError("episode manifest has no source-world fixed start")
    values = (
        float(raw.get("x_m")),
        float(raw.get("y_m")),
        float(raw.get("yaw_rad", 0.0)),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("episode fixed start must be finite")
    if not math.isclose(values[2], 0.0, rel_tol=0.0, abs_tol=1e-12):
        raise ValueError(
            "offline runtime map currently requires a zero-yaw fixed start"
        )
    return values


def _write_json(path: Path, value: dict) -> None:
    pending = path.with_suffix(path.suffix + f".pending.{os.getpid()}")
    pending.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    pending.replace(path)


def normalize_offline_map_text_artifacts(source_root: Path) -> tuple[Path, ...]:
    """Normalize only text artifact line endings before provenance hashing."""

    if source_root.is_symlink() or not source_root.is_dir():
        raise ValueError(f"offline map source must be a real directory: {source_root}")
    root = source_root.resolve()
    normalized: list[Path] = []
    for path in sorted(root.rglob("*")):
        if path.is_symlink() or not path.is_file():
            continue
        if path.suffix.lower() not in LF_TEXT_SUFFIXES:
            continue
        original = path.read_bytes()
        lf_bytes = original.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
        if lf_bytes == original:
            continue
        pending = path.with_name(f".{path.name}.lf.pending.{os.getpid()}")
        try:
            pending.write_bytes(lf_bytes)
            pending.chmod(path.stat().st_mode)
            pending.replace(path)
        finally:
            if pending.exists():
                pending.unlink()
        normalized.append(path)
    return tuple(normalized)


def materialize_map(
    *,
    source_root: Path,
    episode_manifest: Path,
    output_root: Path,
    source_revision: str,
) -> dict:
    normalize_offline_map_text_artifacts(source_root)
    evidence = validate_frozen_offline_raycast_map_source(source_root)
    start_x, start_y, start_yaw = _fixed_start(episode_manifest)
    if not math.isclose(start_yaw, 0.0, rel_tol=0.0, abs_tol=1e-12):
        raise ValueError("offline runtime map requires a zero-yaw episode start")
    if output_root.exists():
        raise ValueError(f"refusing stale offline runtime map root: {output_root}")
    output_root.mkdir(parents=True)

    for name in (
        "occupancy.pgm",
        "offline_raycast_manifest.json",
        "offline_raycast_seal.json",
        "SOURCE_PROVENANCE.json",
        "map_area_verification.json",
    ):
        shutil.copyfile(source_root / name, output_root / name)

    source_origin = [float(value) for value in evidence["origin_m"]]
    runtime_origin = [source_origin[0] - start_x, source_origin[1] - start_y, 0.0]
    source_yaml = yaml.safe_load(
        (source_root / "occupancy.yaml").read_text(encoding="utf-8")
    )
    if not isinstance(source_yaml, dict):
        raise ValueError("offline occupancy YAML root must be a mapping")
    runtime_yaml = {**source_yaml, "origin": runtime_origin}
    runtime_yaml_path = output_root / "occupancy.yaml"
    runtime_yaml_path.write_text(
        yaml.safe_dump(runtime_yaml, sort_keys=False), encoding="utf-8"
    )

    source_pgm_sha = sha256_bytes((source_root / "occupancy.pgm").read_bytes())
    runtime_pgm_sha = sha256_bytes((output_root / "occupancy.pgm").read_bytes())
    runtime_yaml_sha = sha256_bytes(runtime_yaml_path.read_bytes())
    if runtime_pgm_sha != source_pgm_sha:
        raise ValueError("runtime offline map PGM is not byte-identical")
    binding = {
        "schema_version": 1,
        "map_source_mode": OFFLINE_RAYCAST_MAPPING,
        "label": OFFLINE_MAP_SOURCE_LABEL,
        "live_slam_claimed": False,
        "source_revision": source_revision,
        "source_manifest_sha256": evidence["source_manifest_sha256"],
        "source_provenance_sha256": evidence["source_provenance_sha256"],
        "source_pgm_sha256": source_pgm_sha,
        "runtime_pgm_sha256": runtime_pgm_sha,
        "runtime_yaml_sha256": runtime_yaml_sha,
        "source_fixed_start_pose": [start_x, start_y, start_yaw],
        "source_origin_m": source_origin,
        "runtime_origin_m": runtime_origin,
        "alignment_rule": "runtime_origin = source_origin - source_fixed_start_xy",
        "claim_boundary": (
            "The runtime map is byte-identical to the frozen offline raycast "
            "PGM. Only the YAML origin is translated so the episode fixed start "
            "is map (0, 0). This is OFFLINE_MAP_SOURCE, not live SLAM."
        ),
    }
    _write_json(output_root / RUNTIME_BINDING_FILE, binding)
    return binding


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--episode-manifest", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--source-revision", required=True)
    args = parser.parse_args()
    if args.output_root.exists():
        raise SystemExit(f"fresh offline runtime map root required: {args.output_root}")
    materialize_map(
        source_root=args.source_root.resolve(),
        episode_manifest=args.episode_manifest.resolve(),
        output_root=args.output_root.resolve(),
        source_revision=args.source_revision,
    )
    print(args.output_root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
