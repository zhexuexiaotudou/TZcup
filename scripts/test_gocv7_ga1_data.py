import importlib.util
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def load_module():
    path = ROOT / "scripts/prepare_gocv7_ga1.py"
    spec = importlib.util.spec_from_file_location(path.stem, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_bbox_is_native_xyxy():
    module = load_module()
    mask = np.zeros((8, 10), dtype=bool)
    mask[2:6, 3:9] = True
    assert module.bbox(mask) == [3, 2, 9, 6]
    assert module.bbox(np.zeros_like(mask)) is None


def test_world_isolated_split_contract():
    module = load_module()
    assert module.SPLIT_BY_WORLD_INDEX == {
        0: "GA1_TRAIN",
        1: "GA1_TRAIN",
        2: "GA1_TRAIN",
        3: "GA1_HOLDOUT",
    }
    manifest = {"worlds": [{"world_id": f"world_{index}"} for index in range(4)]}
    assert module.world_indices(manifest)["world_3"] == 3


def test_ga1_uses_prompt_minimum_representative_frame_gate():
    source = (ROOT / "scripts" / "prepare_gocv7_ga1.py").read_text(
        encoding="utf-8"
    )
    assert "minimum_frames_per_mission: int = 20" in source
    assert '"minimum_representative_frame_gate": 300' in source


def test_ga1_dedupes_within_split_but_fails_cross_split():
    source = (ROOT / "scripts" / "prepare_gocv7_ga1.py").read_text(
        encoding="utf-8"
    )
    assert "if prior[0] != split:" in source
    assert "exact RGB duplicate across GA1 TRAIN/HOLDOUT" in source
    assert "within_split_duplicates.append" in source
