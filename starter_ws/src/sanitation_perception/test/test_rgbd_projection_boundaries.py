"""Offline input-boundary regressions; these are not recognition benchmarks."""

from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sanitation_perception.pc_open_vocab_adapter import (
    _projection_masks,
    validate_rgbd_context,
)
from sanitation_perception.product_projection import (
    CameraIntrinsics, PublicGrid, project_rgbd_observation, require_valid_depth,
)


def project(box, **overrides):
    arguments = dict(
        depth=np.ones((8, 8), dtype=np.float32),
        camera=CameraIntrinsics(4, 4, 3.5, 3.5),
        map_from_camera=np.eye(4),
        grid=PublicGrid(20, 20, 0.25, -2.5, -2.5),
        boxes_xyxy=np.asarray([box]), class_ids=["litter_cube"],
        masks=[np.ones((8, 8), dtype=bool)], confidences=[0.9],
    )
    arguments.update(overrides)
    return project_rgbd_observation(**arguments)


@pytest.mark.parametrize("box", [[-8, 0, -2, 6], [0, -8, 6, -2], [10, 0, 12, 6], [6, 1, 2, 4]])
def test_outside_or_reversed_boxes_do_not_project_targets(box):
    assert project(box)[1] == []


def test_depth_units_are_equivalent_and_invalid_depth_does_not_project():
    box = [0, 0, 8, 8]
    assert project(box)[1] == project(box, depth=np.full((8, 8), 1000, dtype=np.uint16))[1]
    for invalid in (0.0, -1.0, float("nan"), float("inf"), 100.0):
        raster, targets = project(box, depth=np.full((8, 8), invalid, dtype=np.float32))
        assert not raster.any()
        assert targets == []


@pytest.mark.parametrize("overrides", [
    {"camera": CameraIntrinsics(float("nan"), 4, 3.5, 3.5)},
    {"camera": CameraIntrinsics(4, 4, float("inf"), 3.5)},
    {"map_from_camera": np.eye(3)},
    {"map_from_camera": np.zeros((4, 4))},
    {"map_from_camera": np.diag([2.0, 1.0, 1.0, 1.0])},
    {"map_from_camera": np.diag([-1.0, 1.0, 1.0, 1.0])},
    {"depth": np.ones((8, 8), dtype=np.int16)},
    {"masks": [np.ones((4, 4))]},
    {"masks": [np.full((8, 8), float("nan"))]},
    {"confidences": [float("nan")]},
    {"sample_stride": 0},
])
def test_invalid_projection_context_is_rejected(overrides):
    with pytest.raises(ValueError):
        project([0, 0, 8, 8], **overrides)


@pytest.mark.parametrize("boxes, mask_count", [
    ([[0, 0, 10, 10], [1, 1, 3, 3]], 1),  # area rejected
    ([[1, 1, 3, 3]] * 4, 3),  # prompt budget rejected
])
def test_skipped_edgesam_prompts_reject_frame_instead_of_claiming_clean_ground(boxes, mask_count):
    with pytest.raises(ValueError, match="no EdgeSAM segmentation"):
        _projection_masks(
            (10, 10), np.asarray(boxes), ["puddle"] * len(boxes),
            [np.eye(10, dtype=bool)] * mask_count, [0.8] * mask_count,
        )


@pytest.mark.parametrize("invalid", [float("nan"), float("inf"), float("-inf")])
def test_nonfinite_edgesam_mask_rejected_before_boolean_conversion(invalid):
    mask = np.zeros((10, 10), dtype=np.float32)
    mask[1, 1] = invalid
    with pytest.raises(ValueError, match="finite before boolean"):
        _projection_masks((10, 10), np.asarray([[1, 1, 3, 3]]), ["puddle"], [mask], [0.8])


def context():
    def message():
        return SimpleNamespace(
            header=SimpleNamespace(frame_id="front_optical", stamp=SimpleNamespace(sec=1, nanosec=0)),
            width=8, height=8, encoding="32FC1", k=[4, 0, 3.5, 0, 4, 3.5, 0, 0, 1],
        )
    return message(), message(), message()


@pytest.mark.parametrize("mutation", [
    lambda rgb, depth, info: setattr(depth.header, "frame_id", "wrist_optical"),
    lambda rgb, depth, info: setattr(info.header, "frame_id", "wrist_optical"),
    lambda rgb, depth, info: setattr(info, "width", 16),
    lambda rgb, depth, info: setattr(depth, "encoding", "mono16"),
    lambda rgb, depth, info: setattr(depth.header.stamp, "sec", 2),
    lambda rgb, depth, info: setattr(info.header.stamp, "sec", 2),
    lambda rgb, depth, info: setattr(rgb.header.stamp, "nanosec", -1),
    lambda rgb, depth, info: setattr(info, "k", [float("nan")] * 9),
])
def test_rgbd_context_rejects_misaligned_source_metadata(mutation):
    messages = context()
    mutation(*messages)
    with pytest.raises(ValueError):
        validate_rgbd_context(*messages, maximum_age_s=0.5)


def test_rgbd_context_accepts_registered_and_latched_calibration():
    rgb, depth, info = context()
    validate_rgbd_context(rgb, depth, info, maximum_age_s=0.5)
    info.header.stamp.sec = 0
    validate_rgbd_context(rgb, depth, info, maximum_age_s=0.5)


def test_ros_rgbd_callback_validates_context_before_inference():
    source = (Path(__file__).resolve().parents[1] / "sanitation_perception" / "pc_open_vocab_adapter.py").read_text(encoding="utf-8")
    callback = source.split("def _on_rgbd(", 1)[1].split("def _diagnostic(", 1)[0]
    assert callback.index("validate_rgbd_context(") < callback.index("self.detector.infer(")
    assert callback.index("require_valid_depth(depth)") < callback.index("self.detector.infer(")


@pytest.mark.parametrize("invalid", [0.0, -1.0, float("nan"), float("inf"), 100.0])
def test_live_depth_gate_rejects_frames_without_usable_range(invalid):
    with pytest.raises(ValueError, match="no valid depth"):
        require_valid_depth(np.full((8, 8), invalid, dtype=np.float32))
    depth = np.full((8, 8), invalid, dtype=np.float32)
    depth[3, 3] = 1.0
    require_valid_depth(depth)


def test_nominal_cube_dimensions_cannot_publish_wrist_grasp_recheck():
    source = (Path(__file__).resolve().parents[1] / "sanitation_perception" / "pc_open_vocab_adapter.py").read_text(encoding="utf-8")
    assert '"wrist_grasp_recheck_not_ready"' in source
    assert '"measured_cube_geometry_missing"' in source
    assert "self.wrist_recheck_publisher.publish(" not in source
