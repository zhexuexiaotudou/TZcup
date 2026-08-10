from __future__ import annotations

import random
import json
import sys
from pathlib import Path

import pytest
import numpy as np


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


from sanitation_learning.g4_geometry import (  # noqa: E402
    bbox_is_bounded,
    bbox_model_to_native,
    bbox_native_to_model,
    flip_bbox_horizontal,
    max_coordinate_error,
    remap_flipped_box,
    validate_bbox,
)
from sanitation_learning.g4_data import (  # noqa: E402
    DISCOVERY_MODEL_SIZE,
    G4AreaDataset,
    G4DiscoveryDataset,
    encode_discovery_pyramid_targets,
    encode_teacher_quality_pyramid,
    load_frame_rows,
    load_instance_records,
    _read_rgb_bgr,
)


def test_a3_assigns_each_box_to_exactly_one_pyramid_level() -> None:
    boxes = [
        {"class_index": 0, "bbox_xyxy": [0, 0, 40, 30]},
        {"class_index": 0, "bbox_xyxy": [0, 0, 70, 50]},
        {"class_index": 0, "bbox_xyxy": [0, 0, 100, 60]},
    ]
    targets = encode_discovery_pyramid_targets(boxes, assign_by_scale=True)
    assert float(targets["regression_mask_s4"].sum()) == 1.0
    assert float(targets["regression_mask_s8"].sum()) == 1.0
    assert float(targets["regression_mask_s16"].sum()) == 1.0


def test_a3_teacher_quality_preserves_frozen_soft_scores() -> None:
    targets = encode_teacher_quality_pyramid(
        [
            {"score": 0.73, "bbox_xyxy": [10, 20, 50, 60]},
            {"score": 0.91, "bbox_xyxy": [100, 100, 170, 140]},
        ]
    )
    assert float(targets["teacher_quality_s4"].max()) == pytest.approx(0.73)
    assert float(targets["teacher_quality_s8"].max()) == pytest.approx(0.91)
    assert float(targets["teacher_quality_s16"].max()) == 0.0


def test_discovery_dataset_does_not_read_unused_modalities(monkeypatch) -> None:
    pytest.importorskip("torch")
    from sanitation_learning import g4_data

    calls = {"rgb": 0}

    def fake_read_rgb(_row):
        calls["rgb"] += 1
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def forbidden_read_frame(_row):
        raise AssertionError("RGB-only discovery loaded unused modalities")

    monkeypatch.setattr(g4_data, "read_rgb", fake_read_rgb)
    monkeypatch.setattr(g4_data, "read_frame", forbidden_read_frame)
    dataset = G4DiscoveryDataset(
        [{"scene_seed": 1, "frame_index": 2}], {}, augment=False
    )
    image, targets = dataset[0]
    assert tuple(image.shape) == (3, 480, 640)
    assert tuple(targets["heatmap"].shape) == (1, 120, 160)
    assert tuple(targets["heatmap_s4"].shape) == (1, 120, 160)
    assert tuple(targets["heatmap_s8"].shape) == (1, 60, 80)
    assert tuple(targets["heatmap_s16"].shape) == (1, 30, 40)
    assert calls == {"rgb": 1}


def test_rgb_reader_retries_transient_bind_mount_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sanitation_learning import g4_data

    expected = np.zeros((2, 3, 3), dtype=np.uint8)
    attempts = iter((None, None, expected))
    sleeps = []
    monkeypatch.setattr(g4_data.cv2, "imread", lambda _path: next(attempts))
    monkeypatch.setattr(g4_data.time, "sleep", sleeps.append)
    assert _read_rgb_bgr("frame.png", attempts=3) is expected
    assert sleeps == [0.05, 0.1]


def test_area_cache_is_compressed_but_batches_remain_float32(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    pytest.importorskip("torch")
    from sanitation_learning import g4_data

    monkeypatch.setattr(
        g4_data,
        "read_frame",
        lambda _row: (
            np.zeros((8, 8, 3), dtype=np.uint8),
            np.ones((8, 8), dtype=np.float32),
            np.zeros((8, 8), dtype=np.uint8),
            np.zeros((8, 8), dtype=np.uint16),
        ),
    )
    monkeypatch.setattr(g4_data, "load_camera_info", lambda _row: {})
    monkeypatch.setattr(
        g4_data,
        "build_area_input",
        lambda *_args, **_kwargs: np.full((6, 7, 10), 0.25, dtype=np.float32),
    )
    dataset = G4AreaDataset(
        [{"negative_only": True}], channel=0, cache_frames=True
    )
    inputs, targets, boundaries = dataset[0]
    cached = dataset._frame_cache[0]
    assert cached[0].dtype == np.float16
    assert cached[1].dtype == np.uint8
    assert cached[2].dtype == np.uint8
    assert str(inputs.dtype) == "torch.float32"
    assert str(targets.dtype) == "torch.float32"
    assert str(boundaries.dtype) == "torch.float32"


def _box(x1, y1, x2, y2):
    return {"native_bbox_xyxy": [x1, y1, x2, y2]}


def test_round_trip_corners_non_square() -> None:
    native = (800, 600)
    model = (640, 480)
    corners = (
        (0.0, 0.0, 10.0, 10.0),
        (790.0, 0.0, 800.0, 10.0),
        (0.0, 590.0, 10.0, 600.0),
        (790.0, 590.0, 800.0, 600.0),
        (400.0, 300.0, 410.0, 310.0),
    )
    for corner in corners:
        model_box = bbox_native_to_model(corner, native, model)
        restored = bbox_model_to_native(model_box, native, model)
        assert max_coordinate_error(restored, corner) <= 0.5
        assert bbox_is_bounded(model_box, model)
        assert bbox_is_bounded(restored, native)


def test_round_trip_non_square_model_scales_correctly() -> None:
    # Regression against the historical hard-coded 384/512 bug: y scale must
    # come from the actual native/model heights, never from a fixed 512/480.
    native = (640, 480)
    model = (384, 512)
    box = (320.0, 240.0, 340.0, 260.0)
    model_box = bbox_native_to_model(box, native, model)
    assert model_box[1] == pytest.approx(240.0 * 512.0 / 480.0)
    assert model_box[3] == pytest.approx(260.0 * 512.0 / 480.0)
    restored = bbox_model_to_native(model_box, native, model)
    assert max_coordinate_error(restored, box) <= 0.5


def test_discovery_model_size_is_640x480() -> None:
    assert DISCOVERY_MODEL_SIZE == (640, 480)


def test_flip_once_and_twice_round_trip() -> None:
    native = (640, 480)
    model = DISCOVERY_MODEL_SIZE
    boxes = [
        _box(10.0, 20.0, 100.0, 120.0),
        _box(540.0, 30.0, 630.0, 200.0),
        _box(0.0, 0.0, 320.0, 240.0),
    ]
    flipped = [
        remap_flipped_box(box, native_size=native, model_size=model)
        for box in boxes
    ]
    for original, flipped_box in zip(boxes, flipped):
        expected_native = flip_bbox_horizontal(
            original["native_bbox_xyxy"], native[0]
        )
        assert max_coordinate_error(
            flipped_box["native_bbox_xyxy"], expected_native
        ) <= 1e-9
        expected_model = bbox_native_to_model(
            expected_native, native, model
        )
        assert max_coordinate_error(
            flipped_box["bbox_xyxy"], expected_model
        ) <= 1e-9
        # The model bbox is regenerated from the unified native->model utility.
        assert flipped_box["bbox_xyxy"] == flipped_box["model_bbox_xyxy"]
        assert bbox_is_bounded(flipped_box["bbox_xyxy"], model)
    flipped_twice = [
        remap_flipped_box(box, native_size=native, model_size=model)
        for box in flipped
    ]
    for original, twice in zip(boxes, flipped_twice):
        assert max_coordinate_error(
            twice["native_bbox_xyxy"], original["native_bbox_xyxy"]
        ) <= 1e-9
        assert max_coordinate_error(
            twice["bbox_xyxy"],
            bbox_native_to_model(
                original["native_bbox_xyxy"], native, model
            ),
        ) <= 1e-9


def test_no_flip_keeps_bbox_identical() -> None:
    native = (800, 600)
    model = (640, 480)
    box = (100.0, 80.0, 300.0, 260.0)
    model_box = bbox_native_to_model(box, native, model)
    assert model_box == pytest.approx(
        (80.0, 64.0, 240.0, 208.0)
    )
    assert max_coordinate_error(
        bbox_model_to_native(model_box, native, model), box
    ) <= 0.5


def test_random_1000_boxes_round_trip_error_under_half_pixel() -> None:
    rng = random.Random(20260808)
    native = (800, 600)
    model = DISCOVERY_MODEL_SIZE
    worst = 0.0
    for _ in range(1000):
        width = rng.uniform(1.0, 300.0)
        height = rng.uniform(1.0, 300.0)
        x1 = rng.uniform(0.0, native[0] - width)
        y1 = rng.uniform(0.0, native[1] - height)
        box = (x1, y1, x1 + width, y1 + height)
        validate_bbox(box)
        model_box = bbox_native_to_model(box, native, model)
        restored = bbox_model_to_native(model_box, native, model)
        error = max_coordinate_error(restored, box)
        worst = max(worst, error)
        assert error <= 0.5
        assert bbox_is_bounded(model_box, model)
        assert bbox_is_bounded(restored, native)
    assert worst <= 0.5


def test_random_flipped_boxes_round_trip_error_under_half_pixel() -> None:
    rng = random.Random(7)
    native = (640, 480)
    model = DISCOVERY_MODEL_SIZE
    worst = 0.0
    for _ in range(1000):
        width = rng.uniform(1.0, 200.0)
        height = rng.uniform(1.0, 200.0)
        x1 = rng.uniform(0.0, native[0] - width)
        y1 = rng.uniform(0.0, native[1] - height)
        box = _box(x1, y1, x1 + width, y1 + height)
        flipped = remap_flipped_box(
            box, native_size=native, model_size=model
        )
        expected_native = flip_bbox_horizontal(
            box["native_bbox_xyxy"], native[0]
        )
        assert max_coordinate_error(
            flipped["native_bbox_xyxy"], expected_native
        ) <= 1e-9
        restored = bbox_model_to_native(
            flipped["bbox_xyxy"], native, model
        )
        error = max_coordinate_error(restored, expected_native)
        worst = max(worst, error)
        assert error <= 0.5
        assert bbox_is_bounded(flipped["bbox_xyxy"], model)
    assert worst <= 0.5


def test_invalid_sizes_and_boxes_fail_closed() -> None:
    with pytest.raises(ValueError):
        bbox_native_to_model((0, 0, 10, 10), (0, 480), (640, 480))
    with pytest.raises(ValueError):
        bbox_native_to_model((0, 0, 10, 10), (640, 480), (0, 480))
    with pytest.raises(ValueError):
        validate_bbox((10, 10, 0, 0))
    with pytest.raises(ValueError):
        validate_bbox((0, 0, 10, 20, 30))
    with pytest.raises(ValueError, match="outside"):
        bbox_native_to_model((639, 10, 641, 20), (640, 480), (640, 480))
    with pytest.raises(ValueError, match="outside"):
        bbox_model_to_native((0, 0, 641, 20), (640, 480), (640, 480))


def test_manifest_loaders_filter_forbidden_roles_before_annotations(tmp_path) -> None:
    frame_manifest = tmp_path / "frames.jsonl"
    rows = [
        {
            "scene_seed": 1,
            "frame_index": 0,
            "split": "train",
            "rgb_path": "train/rgb.png",
            "depth_path": "train/depth.exr",
            "semantic_path": "train/semantic.png",
            "instance_path": "train/instance.png",
        },
        {
            "scene_seed": 2,
            "frame_index": 0,
            "split": "test",
            "rgb_path": "legacy/rgb.png",
            "depth_path": "legacy/depth.exr",
            "semantic_path": "legacy/semantic.png",
            "instance_path": "legacy/instance.png",
        },
        {
            "scene_seed": 3,
            "frame_index": 0,
            "split": "G5_SEALED_FINAL",
            "rgb_path": "sealed/rgb.png",
            "depth_path": "sealed/depth.exr",
            "semantic_path": "sealed/semantic.png",
            "instance_path": "sealed/instance.png",
        },
    ]
    frame_manifest.write_text(
        "\n".join(json.dumps(row) for row in rows) + "\n",
        encoding="utf-8",
    )
    loaded = load_frame_rows(
        frame_manifest,
        tmp_path,
        allowed_splits={"train", "val"},
    )
    assert [(row["scene_seed"], row["split"]) for row in loaded] == [
        (1, "train")
    ]
    assert "legacy" not in str(loaded[0]["rgb_path"])
    assert "sealed" not in str(loaded[0]["rgb_path"])

    instance_manifest = tmp_path / "instances.jsonl"
    records = [
        {"scene_seed": seed, "frame_index": 0, "semantic_class": "x"}
        for seed in (1, 2, 3)
    ]
    instance_manifest.write_text(
        "\n".join(json.dumps(record) for record in records) + "\n",
        encoding="utf-8",
    )
    filtered_records = load_instance_records(
        instance_manifest, allowed_frame_keys={(1, 0)}
    )
    assert [record["scene_seed"] for record in filtered_records] == [1]
