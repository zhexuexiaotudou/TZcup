from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[4]


def _load_screening():
    pytest.importorskip("torch")
    path = ROOT / "scripts" / "auto05r_screening.py"
    spec = importlib.util.spec_from_file_location("auto05r_screening", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _row(index: int, world: str = "world") -> dict:
    return {
        "world_id": world,
        "scene_seed": index // 10,
        "frame_index": index,
        "split": "train",
        "negative_only": False,
    }


def test_small_object_frames_are_retained_before_stratified_fill() -> None:
    screening = _load_screening()
    rows = [_row(index) for index in range(20)]
    small_keys = {
        screening._row_identity(rows[index]) for index in (2, 5, 9)
    }
    selected = screening._prioritized_discovery_row_sample(
        rows, small_keys, 8, seed=17
    )
    selected_keys = {screening._row_identity(row) for row in selected}
    assert len(selected) == 8
    assert small_keys <= selected_keys
    assert len(selected_keys) == len(selected)


def test_small_object_sampling_is_deterministic_when_over_capacity() -> None:
    screening = _load_screening()
    rows = [_row(index, world=f"world_{index % 2}") for index in range(12)]
    small_keys = {screening._row_identity(row) for row in rows}
    first = screening._prioritized_discovery_row_sample(
        rows, small_keys, 5, seed=19
    )
    second = screening._prioritized_discovery_row_sample(
        rows, small_keys, 5, seed=19
    )
    assert [screening._row_identity(row) for row in first] == [
        screening._row_identity(row) for row in second
    ]
