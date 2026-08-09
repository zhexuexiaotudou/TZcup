from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "overlay_auto05r_capture_scenes",
    ROOT / "scripts" / "overlay_auto05r_capture_scenes.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _dataset(root: Path, *, frame_count: int = 10, passed: bool = True) -> Path:
    (root / "worlds").mkdir(parents=True)
    (root / "worlds" / "g4_world_manifest.json").write_text(
        json.dumps({"schema_version": 1, "worlds": ["same"]})
    )
    scene = root / "scenes" / "scene_0001"
    scene.mkdir(parents=True)
    (scene / "scene_manifest.json").write_text(
        json.dumps({"scene_seed": 1, "world_id": "world_a", "split": "train"})
    )
    (scene / "capture_report.json").write_text(
        json.dumps({"capture_pass": passed, "records": [{}] * frame_count})
    )
    (scene / "payload.txt").write_text(root.name)
    return root


def test_overlay_preserves_base_and_replaces_valid_scene(tmp_path: Path) -> None:
    base = _dataset(tmp_path / "base")
    overlay = _dataset(tmp_path / "overlay")
    output = tmp_path / "output"
    report = MODULE.overlay_scenes(base, [overlay], output)
    assert report["replacement_count"] == 1
    assert (base / "scenes" / "scene_0001" / "payload.txt").read_text() == "base"
    assert (output / "scenes" / "scene_0001" / "payload.txt").read_text() == "overlay"


def test_overlay_fails_closed_on_bad_capture_or_existing_output(tmp_path: Path) -> None:
    base = _dataset(tmp_path / "base")
    bad = _dataset(tmp_path / "bad", frame_count=9, passed=False)
    with pytest.raises(RuntimeError, match="capture failed"):
        MODULE.overlay_scenes(base, [bad], tmp_path / "output")
    output = tmp_path / "existing"
    output.mkdir()
    with pytest.raises(FileExistsError):
        MODULE.overlay_scenes(base, [base], output)
