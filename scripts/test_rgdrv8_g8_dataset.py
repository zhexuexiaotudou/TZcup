#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("prepare_rgdrv8_g8", ROOT / "scripts/prepare_rgdrv8_g8.py")
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader
SPEC.loader.exec_module(MODULE)


def test_phash_is_deterministic_and_sensitive(tmp_path: Path) -> None:
    first, second = tmp_path / "first.png", tmp_path / "second.png"
    a = np.zeros((64, 64, 3), dtype=np.uint8)
    b = a.copy()
    a[:, :32] = 255
    b[:32, :] = 255
    assert cv2.imwrite(str(first), a)
    assert cv2.imwrite(str(second), b)
    assert MODULE.phash(first) == MODULE.phash(first)
    assert MODULE.phash(first) != MODULE.phash(second)


def test_bbox_uses_coco_xywh_and_short_side() -> None:
    mask = np.zeros((20, 30), dtype=bool)
    mask[2:8, 4:14] = True
    box, short, area = MODULE.bbox(mask)
    assert box == [4, 2, 10, 6]
    assert short == 6
    assert area == 60


def test_split_intersections_cover_every_pair() -> None:
    parts = {"TRAIN_NEW": {"a", "x"}, "HOLDOUT_NEW": {"b", "x"}, "VAL_NEW": {"a", "c"}}
    assert MODULE.intersections(parts) == ["a", "x"]
