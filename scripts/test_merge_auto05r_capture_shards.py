from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "merge_auto05r_capture_shards",
    ROOT / "scripts" / "merge_auto05r_capture_shards.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _shard(root: Path, scene: str, world: str) -> Path:
    for name in ("models", "worlds", "scenes"):
        (root / name).mkdir(parents=True, exist_ok=True)
    (root / "models" / "asset.bin").write_bytes(b"same")
    (root / "worlds" / "manifest.json").write_text(
        json.dumps({"schema_version": 1, "worlds": ["all"]})
    )
    scene_root = root / "scenes" / scene
    scene_root.mkdir()
    (scene_root / "scene_manifest.json").write_text(json.dumps({"world_id": world}))
    (scene_root / "capture_report.json").write_text(
        json.dumps({"capture_pass": True, "records": [{}] * 10})
    )
    return root


def test_merge_requires_identical_static_payload_and_disjoint_scenes(tmp_path: Path):
    first = _shard(tmp_path / "first", "scene_0000", "world_a")
    second = _shard(tmp_path / "second", "scene_0025", "world_b")
    output = tmp_path / "merged"
    report = MODULE.merge_shards(
        [first, second], output, Path("worlds/manifest.json")
    )
    assert report["scene_count"] == 2
    assert report["world_ids"] == ["world_a", "world_b"]
    assert (output / "scenes" / "scene_0025" / "capture_report.json").is_file()
    assert (tmp_path / "merged_merge_report.json").is_file()


def test_merge_rejects_payload_drift_and_existing_output(tmp_path: Path):
    first = _shard(tmp_path / "first", "scene_0000", "world_a")
    second = _shard(tmp_path / "second", "scene_0025", "world_b")
    (second / "models" / "asset.bin").write_bytes(b"different")
    with pytest.raises(RuntimeError, match="byte-identical"):
        MODULE.merge_shards(
            [first, second], tmp_path / "merged", Path("worlds/manifest.json")
        )
    existing = tmp_path / "existing"
    existing.mkdir()
    with pytest.raises(FileExistsError):
        MODULE.merge_shards(
            [first, second], existing, Path("worlds/manifest.json")
        )
