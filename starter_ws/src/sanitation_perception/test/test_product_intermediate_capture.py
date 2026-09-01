import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from sanitation_perception.pc_open_vocab_adapter import (
    _ground_dirt_prompt_decisions,
    _ground_dirt_prompt_indices,
)
from sanitation_perception.product_intermediate_capture import (
    FORBIDDEN_ARTIFACT_TOKENS,
    ProductIntermediateCapture,
)
from sanitation_perception.product_projection import (
    CameraIntrinsics,
    PublicGrid,
    project_rgbd_observation,
)


PACKAGE = Path(__file__).resolve().parents[1]


def _capture_payload(shape=(12, 16)) -> dict:
    height, width = shape
    projection = {
        "sample_stride": 4,
        "ground_z_m": 0.0,
        "ground_tolerance_m": 0.25,
        "valid_depth_pixels_uv": np.asarray([[1, 2], [3, 4]], dtype=np.int32),
        "valid_depth_m": np.asarray([1.0, 1.2], dtype=np.float32),
        "map_points_xyz": np.asarray([[0.1, 0.2, 0.0], [0.2, 0.3, 0.4]]),
        "ground_mask": np.asarray([True, False]),
        "in_grid_mask": np.asarray([True, False]),
        "public_free_mask": np.asarray([True, False]),
        "map_rows_cols": np.asarray([[2, 3], [-1, -1]], dtype=np.int32),
        "public_free_applied_to_product_output": False,
        "per_class_rasters": {
            "puddle": np.zeros((4, 5), dtype=np.uint8),
            "dust_or_soil": np.ones((4, 5), dtype=np.uint8) * 3,
        },
        "final_union_raster": np.ones((4, 5), dtype=np.uint8),
    }
    return {
        "sensor": "front",
        "rgb_stamp_s": 1.0,
        "depth_stamp_s": 0.98,
        "rgb": np.zeros((height, width, 3), dtype=np.uint8),
        "depth": np.ones((height, width), dtype=np.float32),
        "camera_info": {
            "frame_id": "front_rgbd_depth_optical_frame",
            "stamp_s": 1.0,
            "width": width,
            "height": height,
            "distortion_model": "plumb_bob",
            "k": [10.0, 0.0, 8.0, 0.0, 10.0, 6.0, 0.0, 0.0, 1.0],
            "d": [],
        },
        "map_from_camera": np.eye(4),
        "detections": [
            {
                "detection_index": 0,
                "class_id": "puddle",
                "confidence": 0.01,
                "xyxy": [1.0, 2.0, 8.0, 9.0],
            }
        ],
        "prompt_decisions": [
            {
                "detection_index": 0,
                "class_id": "puddle",
                "accepted": True,
                "reason": "accepted",
                "area_fraction": 0.1,
            }
        ],
        "prompt_detection_indices": np.asarray([0]),
        "prompt_masks": [np.ones((height, width), dtype=bool)],
        "prompt_qualities": [0.8],
        "projection_diagnostics": projection,
        "map_occupancy": np.zeros((4, 5), dtype=np.int8),
        "map_metadata": {
            "frame_id": "map",
            "stamp_s": 0.9,
            "width": 5,
            "height": 4,
            "resolution": 0.1,
            "origin_x": -1.0,
            "origin_y": -2.0,
        },
    }


def test_capture_policy_is_fixed_rate_front_only_and_bounded(tmp_path):
    capture = ProductIntermediateCapture(
        tmp_path / "capture", max_frames=2, minimum_interval_s=1.0
    )
    assert capture.wants_frame("front", 1.0)
    assert not capture.wants_frame("wrist", 1.0)
    payload = _capture_payload()
    assert capture.capture_frame(**payload)
    assert not capture.wants_frame("front", 1.5)
    payload["rgb_stamp_s"] = 2.0
    payload["depth_stamp_s"] = 1.98
    assert capture.capture_frame(**payload)
    assert not capture.wants_frame("front", 3.0)


def test_capture_is_atomic_hashed_deduplicated_and_contains_no_private_tokens(tmp_path):
    root = tmp_path / "product_capture"
    capture = ProductIntermediateCapture(root, max_frames=2, minimum_interval_s=1.0)
    payload = _capture_payload()
    assert capture.capture_frame(**payload)
    payload["rgb_stamp_s"] = 2.0
    payload["depth_stamp_s"] = 1.98
    assert capture.capture_frame(**payload)

    frames = sorted((root / "frames").iterdir())
    maps = sorted((root / "maps").iterdir())
    assert [path.name for path in frames] == ["frame-0000", "frame-0001"]
    assert len(maps) == 1
    assert not list(root.rglob(".*"))
    for bundle in [*frames, *maps]:
        manifest = json.loads((bundle / "manifest.json").read_text(encoding="utf-8"))
        for name, expected_hash in manifest["files"].items():
            actual = hashlib.sha256((bundle / name).read_bytes()).hexdigest()
            assert actual == expected_hash
        for json_path in bundle.glob("*.json"):
            encoded = json_path.read_text(encoding="utf-8").lower()
            assert all(token not in encoded for token in FORBIDDEN_ARTIFACT_TOKENS)
    with np.load(frames[0] / "arrays.npz") as arrays:
        for name in (
            "rgb",
            "depth",
            "camera_k",
            "map_from_camera",
            "prompt_mask_000",
            "ground_mask",
            "in_grid_mask",
            "public_free_mask",
            "class_raster_puddle",
            "final_union_raster",
        ):
            assert name in arrays


def test_capture_disk_limit_fails_before_frame_commit_and_removes_staging(tmp_path):
    capture = ProductIntermediateCapture(
        tmp_path / "bounded", max_frames=1, minimum_interval_s=1.0, max_bytes=1024 * 1024
    )
    payload = _capture_payload((1024, 1024))
    rng = np.random.default_rng(7)
    payload["rgb"] = rng.integers(0, 256, (1024, 1024, 3), dtype=np.uint8)
    payload["depth"] = rng.random((1024, 1024), dtype=np.float32)
    payload["prompt_masks"] = [rng.integers(0, 2, (1024, 1024), dtype=np.uint8).astype(bool)]
    with pytest.raises(RuntimeError, match="disk limit exceeded"):
        capture.capture_frame(**payload)
    assert not list(capture.frames_root.iterdir())
    assert not [path for path in capture.root.rglob("*") if path.name.startswith(".")]
    assert sum(path.stat().st_size for path in capture.root.rglob("*") if path.is_file()) <= capture.max_bytes


def test_capture_failure_can_be_latched_off_without_retrying_expensive_io(tmp_path):
    capture = ProductIntermediateCapture(
        tmp_path / "bounded", max_frames=2, minimum_interval_s=1.0
    )
    assert capture.wants_frame("front", 1.0)
    capture.disable("disk limit exceeded")
    assert capture.disabled_reason == "disk limit exceeded"
    assert not capture.wants_frame("front", 1.0)
    assert not capture.wants_frame("front", 2.0)


def test_projection_diagnostics_record_public_free_without_changing_output():
    depth = np.full((8, 8), 2.0, dtype=np.float32)
    transform = np.eye(4)
    transform[2, 3] = -2.0
    masks = [np.ones((8, 8), dtype=bool)]
    grid = PublicGrid(
        20,
        20,
        0.25,
        -2.5,
        -2.5,
        occupancy=np.full((20, 20), 100, dtype=np.int8),
    )
    baseline, baseline_targets = project_rgbd_observation(
        depth,
        CameraIntrinsics(4.0, 4.0, 3.5, 3.5),
        transform,
        grid,
        boxes_xyxy=np.asarray([[0, 0, 8, 8]], dtype=np.float32),
        class_ids=["puddle"],
        masks=masks,
        confidences=[0.8],
        sample_stride=1,
    )
    diagnostics = {}
    observed, observed_targets = project_rgbd_observation(
        depth,
        CameraIntrinsics(4.0, 4.0, 3.5, 3.5),
        transform,
        grid,
        boxes_xyxy=np.asarray([[0, 0, 8, 8]], dtype=np.float32),
        class_ids=["puddle"],
        masks=masks,
        confidences=[0.8],
        sample_stride=1,
        diagnostics_out=diagnostics,
    )
    assert np.array_equal(observed, baseline)
    assert observed_targets == baseline_targets
    assert not diagnostics["public_free_mask"].any()
    assert diagnostics["public_free_applied_to_product_output"] is False
    assert np.array_equal(diagnostics["final_union_raster"], baseline)


def test_prompt_audit_reproduces_indices_and_records_every_reason():
    class_ids = ["litter_cube", "puddle", "puddle", "puddle", "puddle"]
    boxes = np.asarray(
        [
            [0, 0, 10, 10],
            [0, 0, 100, 100],
            [0, 0, 20, 20],
            [20, 20, 40, 40],
            [40, 40, 60, 60],
        ],
        dtype=np.float32,
    )
    expected = _ground_dirt_prompt_indices(class_ids, boxes, (100, 100))
    selected, decisions = _ground_dirt_prompt_decisions(class_ids, boxes, (100, 100))
    assert selected.tolist() == expected.tolist() == [2, 3, 4]
    assert [row["reason"] for row in decisions] == [
        "not_ground_dirt_class",
        "box_area_above_limit",
        "accepted",
        "accepted",
        "accepted",
    ]


def test_adapter_enables_bounded_capture_after_product_publish_without_accuracy_changes():
    source = (PACKAGE / "sanitation_perception/pc_open_vocab_adapter.py").read_text(
        encoding="utf-8"
    )
    runner = (PACKAGE.parents[2] / "scripts/run_formal_random_scene_perception.sh").read_text(
        encoding="utf-8"
    )
    assert 'declare_parameter("score_threshold", 0.005)' in source
    assert 'declare_parameter("nms_threshold", 0.65)' in source
    assert 'declare_parameter("intermediate_capture_max_frames", 12)' in source
    capture_call = source.index("self.intermediate_capture.capture_frame")
    assert source.index("self.target_publisher.publish(target_array)") < capture_call
    assert "self.intermediate_capture.disable(capture_exc)" in source
    assert 'intermediate_capture_root:="${episode_root}/product_intermediates"' in runner
    assert "intermediate_capture_max_bytes:=268435456" in runner
