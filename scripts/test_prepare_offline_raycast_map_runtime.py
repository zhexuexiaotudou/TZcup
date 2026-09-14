from __future__ import annotations

import json
import shutil
from pathlib import Path
from types import SimpleNamespace

import pytest

from prepare_offline_raycast_map_runtime import (
    materialize_map,
    normalize_offline_map_text_artifacts,
)
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


def test_text_line_endings_normalize_to_lf_and_pgm_stays_binary(
    tmp_path: Path,
) -> None:
    source = tmp_path / "source"
    source.mkdir()
    text_files = {
        "artifact.yaml": b"a: 1\r\nb: 2\r\n",
        "artifact.json": b'{"a": 1}\r\n',
        "artifact.jsonl": b'{"a": 1}\r{"a": 2}\n',
    }
    for name, value in text_files.items():
        (source / name).write_bytes(value)
    pgm_bytes = b"P5\r\n1 1\r\n255\r\n\x00"
    (source / "occupancy.pgm").write_bytes(pgm_bytes)

    normalized = normalize_offline_map_text_artifacts(source)

    assert {path.name for path in normalized} == set(text_files)
    assert (source / "artifact.yaml").read_bytes() == b"a: 1\nb: 2\n"
    assert (source / "artifact.json").read_bytes() == b'{"a": 1}\n'
    assert (source / "artifact.jsonl").read_bytes() == (
        b'{"a": 1}\n{"a": 2}\n'
    )
    assert (source / "occupancy.pgm").read_bytes() == pgm_bytes


def test_materialize_normalizes_crlf_source_before_provenance_validation(
    tmp_path: Path,
) -> None:
    source = tmp_path / "source"
    shutil.copytree(SOURCE_ROOT, source)
    expected = {}
    for path in sorted(source.iterdir()):
        if path.suffix.lower() not in {".json", ".yaml"}:
            continue
        expected[path.name] = path.read_bytes()
        path.write_bytes(expected[path.name].replace(b"\n", b"\r\n"))
    pgm = source / "occupancy.pgm"
    expected_pgm = pgm.read_bytes()

    output_root = tmp_path / "offline_map_source"
    materialize_map(
        source_root=source,
        episode_manifest=_episode_manifest(tmp_path),
        output_root=output_root,
        source_revision="4ab39bb839e92fe50e22b1efe3891c5915f39bf0",
    )

    for name, original_lf in expected.items():
        assert (source / name).read_bytes() == original_lf
        if name != "occupancy.yaml":
            assert (output_root / name).read_bytes() == original_lf
    assert pgm.read_bytes() == expected_pgm
    assert (output_root / "occupancy.pgm").read_bytes() == expected_pgm
