from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from prepare_offline_raycast_map_runtime import materialize_map
from sanitation_formal_campus_integration.offline_map_source import (
    validate_frozen_offline_raycast_map_source,
    validate_offline_raycast_map_source,
)


ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = (
    ROOT
    / "artifacts"
    / "day1_dynamic_avoidance_offline_map_20260914"
    / "offline_map_source"
)


def _episode_manifest(tmp_path: Path) -> Path:
    path = tmp_path / "episode_manifest.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "vehicle_start_pose_source_world": {
                    "x_m": -98.0,
                    "y_m": 0.0,
                    "yaw_rad": 0.0,
                },
            }
        ),
        encoding="utf-8",
    )
    return path


def test_frozen_offline_source_preserves_commit_provenance() -> None:
    evidence = validate_frozen_offline_raycast_map_source(SOURCE_ROOT)
    assert evidence["map_source_mode"] == "OFFLINE_RAYCAST_MAPPING"
    assert evidence["label"] == "OFFLINE_MAP_SOURCE"
    assert evidence["live_slam_claimed"] is False
    assert evidence["known_area_m2"] == pytest.approx(22399.99)
    assert evidence["unknown_fraction"] == pytest.approx(0.030303463203463204)
    assert evidence["resolution_m"] == pytest.approx(0.05)


def test_runtime_map_is_copied_byte_for_byte_and_aligned_to_start(
    tmp_path: Path,
) -> None:
    output_root = tmp_path / "offline_map_source"
    binding = materialize_map(
        source_root=SOURCE_ROOT,
        episode_manifest=_episode_manifest(tmp_path),
        output_root=output_root,
        source_revision="4ab39bb839e92fe50e22b1efe3891c5915f39bf0",
    )
    evidence = validate_offline_raycast_map_source(
        output_root,
        SimpleNamespace(fixed_start_source=(-98.0, 0.0, 0.0)),
    )
    assert binding["runtime_origin_m"] == [-7.0, -55.0, 0.0]
    assert evidence["runtime_origin_m"] == [-7.0, -55.0, 0.0]
    assert (
        evidence["occupancy_pgm_sha256"]
        == binding["source_pgm_sha256"]
        == binding["runtime_pgm_sha256"]
    )


def test_runtime_map_fails_if_pgm_bytes_change(tmp_path: Path) -> None:
    output_root = tmp_path / "offline_map_source"
    materialize_map(
        source_root=SOURCE_ROOT,
        episode_manifest=_episode_manifest(tmp_path),
        output_root=output_root,
        source_revision="4ab39bb839e92fe50e22b1efe3891c5915f39bf0",
    )
    pgm = output_root / "occupancy.pgm"
    value = bytearray(pgm.read_bytes())
    value[-1] ^= 1
    pgm.write_bytes(value)
    with pytest.raises(Exception, match="PGM hash|provenance"):
        validate_offline_raycast_map_source(
            output_root,
            SimpleNamespace(fixed_start_source=(-98.0, 0.0, 0.0)),
        )
