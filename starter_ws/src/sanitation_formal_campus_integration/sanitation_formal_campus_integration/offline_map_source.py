"""Fail-closed admission for a frozen offline raycast occupancy map."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import yaml


OFFLINE_RAYCAST_MAPPING = "OFFLINE_RAYCAST_MAPPING"
OFFLINE_MAP_SOURCE_LABEL = "OFFLINE_MAP_SOURCE"
LIVE_SLAM_MAP_SOURCE = "LIVE_SLAM"
LIVE_SLAM_MAP_SOURCE_LABEL = "LIVE_SLAM"
OFFLINE_MAP_SOURCE_STATUS = (
    "OFFLINE_RAYCAST_MAPPING_PASS_LIVE_SLAM_NOT_CLAIMED"
)
OFFLINE_MAP_SOURCE_COMMIT = "4ab39bb839e92fe50e22b1efe3891c5915f39bf0"
MINIMUM_KNOWN_AREA_M2 = 20000.0
MAXIMUM_UNKNOWN_FRACTION = 0.05
EXPECTED_RESOLUTION_M = 0.05
EXPECTED_WIDTH_CELLS = 4200
EXPECTED_HEIGHT_CELLS = 2200
REQUIRED_FILES = frozenset(
    {
        "occupancy.yaml",
        "occupancy.pgm",
        "offline_raycast_manifest.json",
        "offline_raycast_seal.json",
        "SOURCE_PROVENANCE.json",
        "map_area_verification.json",
    }
)
RUNTIME_BINDING_FILE = "offline_map_source_binding.json"


class OfflineMapSourceError(RuntimeError):
    """Raised when a frozen offline map source cannot be admitted."""


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: str | Path) -> str:
    return sha256_bytes(Path(path).read_bytes())


def _legacy_text_provenance_sha256(snapshot: bytes) -> str:
    if b"\r" in snapshot:
        return sha256_bytes(snapshot)
    return sha256_bytes(snapshot.replace(b"\n", b"\r\n"))


def _matches_text_provenance_sha256(snapshot: bytes, expected: Any) -> bool:
    """Accept the frozen CRLF digest while the working artifact is LF."""

    if not isinstance(expected, str):
        return False
    return expected in {
        sha256_bytes(snapshot),
        _legacy_text_provenance_sha256(snapshot),
    }


def _read_regular(path: Path, label: str) -> bytes:
    if path.is_symlink() or not path.is_file():
        raise OfflineMapSourceError(f"{label} must be a regular file: {path}")
    try:
        return path.read_bytes()
    except OSError as exc:
        raise OfflineMapSourceError(f"{label} is unreadable: {path}") from exc


def _read_json_snapshot(snapshot: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(snapshot)
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise OfflineMapSourceError(f"{label} is not valid JSON") from exc
    if not isinstance(value, dict):
        raise OfflineMapSourceError(f"{label} JSON root must be an object")
    return value


def _read_yaml_snapshot(snapshot: bytes, label: str) -> dict[str, Any]:
    try:
        value = yaml.safe_load(snapshot)
    except yaml.YAMLError as exc:
        raise OfflineMapSourceError(f"{label} is not valid YAML") from exc
    if not isinstance(value, dict):
        raise OfflineMapSourceError(f"{label} YAML root must be a mapping")
    return value


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise OfflineMapSourceError(f"{label} must be numeric")
    number = float(value)
    if not math.isfinite(number):
        raise OfflineMapSourceError(f"{label} must be finite")
    return number


def _parse_p5_pgm(snapshot: bytes, label: str) -> tuple[int, int, int, bytes]:
    """Parse a binary PGM without accepting comments or non-P5 encodings."""

    tokens: list[bytes] = []
    position = 0
    while len(tokens) < 4 and position < len(snapshot):
        while position < len(snapshot) and snapshot[position] in b" \t\r\n":
            position += 1
        if position < len(snapshot) and snapshot[position:position + 1] == b"#":
            newline = snapshot.find(b"\n", position)
            if newline < 0:
                break
            position = newline + 1
            continue
        start = position
        while (
            position < len(snapshot)
            and snapshot[position] not in b" \t\r\n"
        ):
            position += 1
        if start == position:
            break
        tokens.append(snapshot[start:position])
    if len(tokens) != 4 or tokens[0] != b"P5":
        raise OfflineMapSourceError(f"{label} must be a binary P5 PGM")
    try:
        width, height, maximum = (int(token) for token in tokens[1:])
    except ValueError as exc:
        raise OfflineMapSourceError(f"{label} PGM header is invalid") from exc
    if (
        width <= 0
        or height <= 0
        or maximum <= 0
        or maximum > 255
    ):
        raise OfflineMapSourceError(f"{label} PGM header values are invalid")
    while position < len(snapshot) and snapshot[position] in b" \t\r\n":
        position += 1
    pixels = snapshot[position:]
    if len(pixels) != width * height:
        raise OfflineMapSourceError(f"{label} PGM pixel payload length is invalid")
    return width, height, maximum, pixels


def validate_frozen_offline_raycast_map_source(
    artifact_directory: str | Path,
    *,
    allow_episode_aligned_yaml: bool = False,
) -> dict[str, Any]:
    """Validate the compact frozen-map bundle before any runtime binding."""

    root = Path(artifact_directory).resolve()
    if not root.is_dir() or root.is_symlink():
        raise OfflineMapSourceError(
            f"offline raycast map source must be a real directory: {root}"
        )

    snapshots: dict[str, bytes] = {}
    for name in sorted(REQUIRED_FILES):
        snapshots[name] = _read_regular(root / name, f"offline map artifact {name}")

    manifest = _read_json_snapshot(
        snapshots["offline_raycast_manifest.json"],
        "offline raycast provenance manifest",
    )
    seal = _read_json_snapshot(
        snapshots["offline_raycast_seal.json"],
        "offline raycast provenance seal",
    )
    source_provenance = _read_json_snapshot(
        snapshots["SOURCE_PROVENANCE.json"],
        "offline map source provenance",
    )
    area = _read_json_snapshot(
        snapshots["map_area_verification.json"],
        "offline raycast area verification",
    )
    occupancy = _read_yaml_snapshot(
        snapshots["occupancy.yaml"], "offline raycast occupancy YAML"
    )

    map_metadata = manifest.get("map")
    quality = manifest.get("quality_metrics")
    area_metadata = manifest.get("map_area_verifier")
    if not all(
        isinstance(value, dict)
        for value in (map_metadata, quality, area_metadata)
    ):
        raise OfflineMapSourceError(
            "offline raycast provenance map/quality/area metadata is incomplete"
        )
    if (
        manifest.get("schema_version") != 1
        or manifest.get("mapping_kind") != OFFLINE_RAYCAST_MAPPING
        or manifest.get("status") != OFFLINE_MAP_SOURCE_STATUS
        or "not Gazebo live SLAM"
        not in str(manifest.get("claim_boundary", ""))
    ):
        raise OfflineMapSourceError(
            "offline raycast provenance does not preserve the not-live-SLAM boundary"
        )
    if (
        source_provenance.get("schema_version") != 1
        or source_provenance.get("map_source_mode") != OFFLINE_RAYCAST_MAPPING
        or source_provenance.get("label") != OFFLINE_MAP_SOURCE_LABEL
        or source_provenance.get("live_slam_claimed") is not False
        or source_provenance.get("source_commit") != OFFLINE_MAP_SOURCE_COMMIT
        or not _matches_text_provenance_sha256(
            snapshots["offline_raycast_manifest.json"],
            source_provenance.get("source_manifest_sha256"),
        )
        or source_provenance.get("occupancy_yaml_sha256")
        != map_metadata.get("occupancy_yaml_sha256")
        or (
            not allow_episode_aligned_yaml
            and not _matches_text_provenance_sha256(
                snapshots["occupancy.yaml"],
                source_provenance.get("occupancy_yaml_sha256"),
            )
        )
        or source_provenance.get("occupancy_pgm_sha256")
        != sha256_bytes(snapshots["occupancy.pgm"])
        or not _matches_text_provenance_sha256(
            snapshots["map_area_verification.json"],
            source_provenance.get("area_verification_sha256"),
        )
    ):
        raise OfflineMapSourceError("offline map source commit provenance is invalid")

    resolution = _finite_number(
        map_metadata.get("resolution_m"), "offline map resolution"
    )
    width = map_metadata.get("width_cells")
    height = map_metadata.get("height_cells")
    if (
        not math.isclose(
            resolution, EXPECTED_RESOLUTION_M, rel_tol=0.0, abs_tol=1e-12
        )
        or width != EXPECTED_WIDTH_CELLS
        or height != EXPECTED_HEIGHT_CELLS
    ):
        raise OfflineMapSourceError(
            "offline raycast map geometry differs from the frozen 0.05 m map"
        )
    if map_metadata.get("occupancy_pgm_sha256") != sha256_bytes(
        snapshots["occupancy.pgm"]
    ):
        raise OfflineMapSourceError("offline raycast PGM hash differs from provenance")
    occupancy_yaml_sha256 = sha256_bytes(snapshots["occupancy.yaml"])
    if (
        not allow_episode_aligned_yaml
        and not _matches_text_provenance_sha256(
            snapshots["occupancy.yaml"],
            map_metadata.get("occupancy_yaml_sha256"),
        )
    ):
        raise OfflineMapSourceError("offline raycast YAML hash differs from provenance")
    if (
        seal.get("mapping_kind") != OFFLINE_RAYCAST_MAPPING
        or seal.get("status") != OFFLINE_MAP_SOURCE_STATUS
        or not _matches_text_provenance_sha256(
            snapshots["offline_raycast_manifest.json"],
            seal.get("manifest_sha256"),
        )
        or seal.get("occupancy_yaml_sha256")
        != map_metadata.get("occupancy_yaml_sha256")
        or (
            not allow_episode_aligned_yaml
            and not _matches_text_provenance_sha256(
                snapshots["occupancy.yaml"],
                seal.get("occupancy_yaml_sha256"),
            )
        )
        or seal.get("occupancy_pgm_sha256")
        != sha256_bytes(snapshots["occupancy.pgm"])
    ):
        raise OfflineMapSourceError("offline raycast provenance seal is invalid")
    if (
        area.get("pass") is not True
        or _finite_number(area.get("largest_known_area_m2"), "area verification")
        < MINIMUM_KNOWN_AREA_M2
    ):
        raise OfflineMapSourceError("offline raycast area verification did not pass")
    if (
        area_metadata.get("report_path") != "map_area_verification.json"
        or not _matches_text_provenance_sha256(
            snapshots["map_area_verification.json"],
            area_metadata.get("report_sha256"),
        )
        or area_metadata.get("area_gate") is not True
    ):
        raise OfflineMapSourceError("offline raycast area report is not provenance-bound")

    width_pgm, height_pgm, maximum, pixels = _parse_p5_pgm(
        snapshots["occupancy.pgm"], "offline raycast occupancy PGM"
    )
    if (
        width_pgm != width
        or height_pgm != height
        or maximum != 255
    ):
        raise OfflineMapSourceError("offline raycast PGM geometry is inconsistent")
    known_cells = sum(pixel != 205 for pixel in pixels)
    unknown_cells = len(pixels) - known_cells
    computed_known_area = known_cells * resolution * resolution
    computed_unknown_fraction = unknown_cells / len(pixels)
    recorded_known_area = _finite_number(
        quality.get("known_area_m2"), "offline map known area"
    )
    recorded_unknown_fraction = _finite_number(
        quality.get("unknown_fraction"), "offline map unknown fraction"
    )
    if (
        quality.get("quality_gate") is not True
        or quality.get("area_gate") is not True
        or quality.get("resolution_gate") is not True
        or quality.get("unknown_fraction_gate") is not True
        or recorded_known_area < MINIMUM_KNOWN_AREA_M2
        or recorded_unknown_fraction > MAXIMUM_UNKNOWN_FRACTION
        or not math.isclose(
            computed_known_area, recorded_known_area, rel_tol=0.0, abs_tol=1e-6
        )
        or not math.isclose(
            computed_unknown_fraction,
            recorded_unknown_fraction,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
    ):
        raise OfflineMapSourceError("offline raycast quality metrics did not pass")

    if (
        occupancy.get("image") != "occupancy.pgm"
        or occupancy.get("mode") != "trinary"
        or occupancy.get("negate") != 0
        or not math.isclose(
            _finite_number(
                occupancy.get("resolution"), "occupancy YAML resolution"
            ),
            resolution,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
        or not isinstance(occupancy.get("origin"), list)
        or len(occupancy["origin"]) != 3
    ):
        raise OfflineMapSourceError("offline raycast occupancy YAML is invalid")

    return {
        "schema_version": 1,
        "map_source_mode": OFFLINE_RAYCAST_MAPPING,
        "label": OFFLINE_MAP_SOURCE_LABEL,
        "live_slam_claimed": False,
        "status": OFFLINE_MAP_SOURCE_STATUS,
        "claim_boundary": manifest["claim_boundary"],
        "source_manifest_sha256": _legacy_text_provenance_sha256(
            snapshots["offline_raycast_manifest.json"]
        ),
        "source_seal_sha256": sha256_bytes(snapshots["offline_raycast_seal.json"]),
        "source_provenance_sha256": sha256_bytes(
            snapshots["SOURCE_PROVENANCE.json"]
        ),
        "area_verification_sha256": sha256_bytes(
            snapshots["map_area_verification.json"]
        ),
        "occupancy_yaml_sha256": occupancy_yaml_sha256,
        "frozen_occupancy_yaml_sha256": map_metadata.get(
            "occupancy_yaml_sha256"
        ),
        "occupancy_pgm_sha256": sha256_bytes(snapshots["occupancy.pgm"]),
        "source_world_sha256": manifest.get("source", {}).get("world_sha256"),
        "resolution_m": resolution,
        "width_cells": width,
        "height_cells": height,
        "known_area_m2": recorded_known_area,
        "unknown_fraction": recorded_unknown_fraction,
        "origin_m": [float(value) for value in occupancy["origin"]],
    }


def validate_offline_raycast_map_source(
    artifact_directory: str | Path,
    contract: Any,
) -> dict[str, Any]:
    """Validate the runtime-aligned offline map and its episode binding."""

    root = Path(artifact_directory).resolve()
    evidence = validate_frozen_offline_raycast_map_source(
        root, allow_episode_aligned_yaml=True
    )
    binding_snapshot = _read_regular(
        root / RUNTIME_BINDING_FILE, "offline runtime map binding"
    )
    binding = _read_json_snapshot(
        binding_snapshot, "offline runtime map binding"
    )
    if (
        binding.get("schema_version") != 1
        or binding.get("map_source_mode") != OFFLINE_RAYCAST_MAPPING
        or binding.get("label") != OFFLINE_MAP_SOURCE_LABEL
        or binding.get("live_slam_claimed") is not False
    ):
        raise OfflineMapSourceError("offline runtime map binding is not explicitly offline")
    start = tuple(float(value) for value in contract.fixed_start_source)
    binding_start = binding.get("source_fixed_start_pose")
    if (
        not isinstance(binding_start, list)
        or len(binding_start) != 3
        or any(
            not math.isclose(
                float(actual), expected, rel_tol=0.0, abs_tol=1e-9
            )
            for actual, expected in zip(binding_start, start)
        )
    ):
        raise OfflineMapSourceError("offline map binding start pose differs from episode")
    if not math.isclose(
        float(binding_start[2]), 0.0, rel_tol=0.0, abs_tol=1e-12
    ):
        raise OfflineMapSourceError(
            "offline runtime map currently requires a zero-yaw fixed start"
        )
    source_origin = binding.get("source_origin_m")
    runtime_origin = binding.get("runtime_origin_m")
    if (
        not isinstance(source_origin, list)
        or len(source_origin) != 3
        or not isinstance(runtime_origin, list)
        or len(runtime_origin) != 3
    ):
        raise OfflineMapSourceError("offline runtime map origin binding is invalid")
    expected_origin = [
        float(source_origin[0]) - start[0],
        float(source_origin[1]) - start[1],
        0.0,
    ]
    if any(
        not math.isclose(
            float(actual), expected, rel_tol=0.0, abs_tol=1e-9
        )
        for actual, expected in zip(runtime_origin, expected_origin)
    ):
        raise OfflineMapSourceError("offline runtime map is not fixed at the episode start")
    if (
        binding.get("source_manifest_sha256")
        != evidence["source_manifest_sha256"]
        or binding.get("source_provenance_sha256")
        != evidence["source_provenance_sha256"]
        or binding.get("source_pgm_sha256") != evidence["occupancy_pgm_sha256"]
        or binding.get("runtime_pgm_sha256") != evidence["occupancy_pgm_sha256"]
        or binding.get("runtime_yaml_sha256") != evidence["occupancy_yaml_sha256"]
    ):
        raise OfflineMapSourceError("offline runtime map binding hashes have drifted")
    runtime_yaml = _read_yaml_snapshot(
        _read_regular(root / "occupancy.yaml", "runtime occupancy YAML"),
        "runtime occupancy YAML",
    )
    if any(
        not math.isclose(
            float(actual), expected, rel_tol=0.0, abs_tol=1e-9
        )
        for actual, expected in zip(runtime_yaml["origin"], expected_origin)
    ):
        raise OfflineMapSourceError("runtime occupancy YAML origin is not episode-aligned")

    return {
        **evidence,
        "binding_sha256": sha256_bytes(binding_snapshot),
        "source_fixed_start_pose": list(start),
        "source_origin_m": [float(value) for value in source_origin],
        "runtime_origin_m": [float(value) for value in runtime_origin],
        "runtime_aligned_to_episode_start": True,
    }
