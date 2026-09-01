import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from sanitation_perception.formal_random_scene_evaluator import (
    _camera_from_map_at_staging,
    _lookup_pose_pair_with_retry,
    _pose2d_error,
    _project_cube,
)


PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]


def test_live_evaluator_is_truth_isolated_and_requires_real_camera_topics():
    source = (PACKAGE / "sanitation_perception/formal_random_scene_evaluator.py").read_text(encoding="utf-8")
    assert "load_evaluator_truth" in source
    assert "SetEntityPose" in source
    assert "synthetic" not in source.lower()
    for topic in (
        "/sensors/front_rgbd/depth/image_rect_raw/image",
        "/sensors/front_rgbd/depth/image_rect_raw/depth_image",
        "/sensors/wrist_rgbd/depth/image_rect_raw/image",
        "/sensors/rear_left_fisheye/image_raw",
        "/sensors/rear_right_fisheye/image_raw",
        "/perception/garbage/detections_2d",
        "/perception/ground_dirt/masks",
        "/perception/garbage/targets",
    ):
        assert topic in source
    assert "create_publisher" not in source
    assert "/evaluation/scenario_ground_truth" not in source
    # Missing product messages are empty predictions, not missing evaluator
    # samples: real front-camera frames establish the evaluation denominator.
    assert 'sensor == "front"' in source
    assert "front_truth_by_stamp" in source
    assert "self.evaluated_frame_count += 1" in source
    assert "self.acceptance_sampling_active = True" in source
    assert 'missing_startup_inputs.append("product_liveness_diagnostic")' in source
    assert "diagnostic_qos" in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert '"product_detection_message_count"' in source
    assert '"product_mask_message_count"' in source
    assert '"product_target_message_count"' in source
    assert "self._stamp_in_acceptance_window(message.header.stamp)" in source
    assert "real_gazebo_camera_frame" in source
    assert "diagnostic_frame_path" in source
    assert "public_manifest_path" in source
    assert "public_episode_manifest_vehicle_start_pose_source_world" in source
    assert "vehicle_start_pose_localization_map" in source
    assert "world_base_x, world_base_y, world_yaw = self.public_start_pose" in source
    # Evaluator truth is map-frame data and pinhole projection consumes
    # camera-frame points.  Guard the tf2 target/source order explicitly.
    assert 'frame_id, "map", Time()' in source
    assert "camera_from_map @ corners" in source
    assert "np.linalg.inv(camera_from_map)" not in source
    assert "self.staged_front_camera_from_map" in source
    assert "_camera_from_map_at_staging(" in source
    assert '"base_footprint"' in source
    assert '"z_m": self.map_ground_z_m + edge / 2.0' in source


def test_project_cube_consumes_camera_from_map_without_double_inverse():
    camera_from_map = np.eye(4, dtype=np.float64)
    camera_from_map[2, 3] = 2.0
    info = SimpleNamespace(
        k=[400.0, 0.0, 100.0, 0.0, 400.0, 50.0, 0.0, 0.0, 1.0],
        width=200,
        height=100,
    )
    cube = {
        "object_id": "cube-1",
        "edge_m": 0.03,
        "pose": {"x_m": 0.0, "y_m": 0.0},
    }
    projected = _project_cube(cube, camera_from_map, info)
    assert projected is not None
    box, depth = projected
    assert box.object_id == "cube-1"
    assert box.xyxy[0] < 100.0 < box.xyxy[2]
    assert box.xyxy[1] < 50.0 < box.xyxy[3]
    assert 2.0 < depth < 2.1


def test_pose2d_error_keeps_localization_and_source_frames_separate():
    position_error, yaw_error = _pose2d_error(
        (5.3, -2.6, -math.pi + 0.05),
        (5.0, -3.0, math.pi - 0.05),
    )
    assert position_error == pytest.approx(0.5)
    assert yaw_error == pytest.approx(0.1)


def test_project_cube_honors_public_tf_ground_plane_offset():
    camera_from_map = np.eye(4, dtype=np.float64)
    camera_from_map[2, 3] = 2.0
    info = SimpleNamespace(
        k=[400.0, 0.0, 100.0, 0.0, 400.0, 50.0, 0.0, 0.0, 1.0],
        width=200,
        height=100,
    )
    default_cube = {
        "object_id": "default",
        "edge_m": 0.03,
        "pose": {"x_m": 0.0, "y_m": 0.0},
    }
    offset_cube = {
        "object_id": "offset",
        "edge_m": 0.03,
        "pose": {"x_m": 0.0, "y_m": 0.0, "z_m": -0.15},
    }
    default_projection = _project_cube(default_cube, camera_from_map, info)
    offset_projection = _project_cube(offset_cube, camera_from_map, info)
    assert default_projection is not None and offset_projection is not None
    assert offset_projection[1] < default_projection[1]


def test_project_cube_matches_real_d435_pixel_geometry_with_base_footprint_height():
    # Captured public TF for the formal front D435: base_link<-optical frame.
    base_from_camera = np.asarray(
        [
            [0.0, -0.423, 0.906, 0.570],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -0.906, -0.423, 0.447],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    camera_from_map = np.linalg.inv(base_from_camera)
    info = SimpleNamespace(
        k=[447.2, 0.0, 424.0, 0.0, 433.0, 240.0, 0.0, 0.0, 1.0],
        width=848,
        height=480,
    )
    cube = {
        "object_id": "near-left",
        "edge_m": 0.03,
        # base_footprint->base_link is +0.1651 m; map->base_link is z=0.
        "pose": {"x_m": 1.15, "y_m": -0.54, "z_m": -0.1651 + 0.015},
    }
    projected = _project_cube(cube, camera_from_map, info)
    assert projected is not None
    box, _ = projected
    assert box.xyxy == pytest.approx(
        (718.2, 392.0, 751.4, 417.3), abs=1.0
    )


def test_staged_camera_projection_is_not_shifted_by_later_odom_drift():
    map_from_base = np.eye(4, dtype=np.float64)
    map_from_base[:2, 3] = [3.0, -2.0]
    yaw = math.radians(31.0)
    map_from_base[:2, :2] = [
        [math.cos(yaw), -math.sin(yaw)],
        [math.sin(yaw), math.cos(yaw)],
    ]
    camera_from_base = np.eye(4, dtype=np.float64)
    camera_from_base[:3, 3] = [0.2, -0.1, 0.4]

    frozen = _camera_from_map_at_staging(map_from_base, camera_from_base)
    staged_map_point = map_from_base @ np.asarray([1.15, -0.54, 0.015, 1.0])
    expected_camera_point = camera_from_base @ np.asarray(
        [1.15, -0.54, 0.015, 1.0]
    )
    assert frozen @ staged_map_point == pytest.approx(expected_camera_point)

    # A later localization drift is a TF/map-projection failure, not a change
    # to the fixed staged cube's 2D truth box.
    drifted_map_from_base = map_from_base.copy()
    drifted_map_from_base[0, 3] += 0.063
    dynamic_camera_from_map = _camera_from_map_at_staging(
        drifted_map_from_base, camera_from_base
    )
    assert not np.allclose(
        dynamic_camera_from_map @ staged_map_point, expected_camera_point
    )


def test_pose_pair_lookup_retries_early_history_extrapolation_as_one_pair():
    calls = []
    spins = []
    attempts = {"map": 0, "odom": 0}
    clock_values = iter((0.0, 0.0, 0.1, 0.2))

    def lookup(target):
        calls.append(target)
        attempts[target] += 1
        if target == "odom" and attempts[target] == 1:
            raise RuntimeError("earliest data is newer than requested time")
        return f"{target}-from-base-{attempts[target]}"

    result = _lookup_pose_pair_with_retry(
        lookup,
        lambda: spins.append("spin"),
        timeout_s=1.0,
        monotonic=lambda: next(clock_values),
    )
    assert result == ("map-from-base-2", "odom-from-base-2")
    assert calls == ["map", "odom", "map", "odom"]
    assert spins == ["spin"]


def test_pose_pair_lookup_fails_closed_after_bounded_retries():
    clock_values = iter((0.0, 0.0, 0.4, 1.1))

    def unavailable(_target):
        raise RuntimeError("TF unavailable")

    with pytest.raises(RuntimeError, match="within 1.000s: TF unavailable"):
        _lookup_pose_pair_with_retry(
            unavailable,
            lambda: None,
            timeout_s=1.0,
            monotonic=lambda: next(clock_values),
        )


def test_acceptance_config_freezes_thresholds_and_claim_boundaries():
    config = yaml.safe_load((PACKAGE / "config/formal_random_scene_acceptance.yaml").read_text(encoding="utf-8"))
    assert config["minimum_episode_count"] == 30
    assert config["statistical_scope"] == {
        "tier": "formal_pc_gazebo_validation_matrix",
        "required_split": "val",
        "required_validation_map_indices": list(range(8)),
        "minimum_unique_validation_map_count": 8,
        "minimum_episodes_per_validation_map": 3,
        "smoke_episode_count": 3,
        "smoke_eligible_for_final_product_evidence": False,
        "statistical_generalization_claimed": False,
    }
    assert config["staged_scene"]["pose_pair_timeout_s"] >= 30.0
    assert config["staged_scene"]["fixed_start_localization_error_m_max"] <= 0.50
    assert config["metrics"]["cube_precision_min"] >= 0.8
    assert config["metrics"]["cube_recall_min"] >= 0.8
    assert config["metrics"]["cube_f1_min"] >= 0.8
    assert config["metrics"]["ground_dirt_iou_min"] >= 0.65
    assert config["metrics"]["ground_dirt_recall_min"] >= 0.85
    assert config["metrics"]["map_projection_rmse_m_max"] <= 0.20
    assert config["metrics"]["map_projection_p95_m_max"] <= 0.35
    assert config["runtime"]["depth_rgb_skew_s_max"] <= 0.50
    assert config["runtime"]["tf_success_ratio_min"] >= 0.95
    assert config["runtime"]["tf_max_age_s"] <= 0.50
    assert config["truth_boundary"]["publish_truth_to_ros"] is False
    assert config["truth_boundary"]["product_truth_input_allowed"] is False
    assert config["truth_boundary"]["synthetic_offline_image_eligible"] is False
    assert config["claim_boundary"]["s100_board_accepted"] is False


def test_console_entry_and_runtime_dependency_are_declared():
    setup = (PACKAGE / "setup.py").read_text(encoding="utf-8")
    package_xml = (PACKAGE / "package.xml").read_text(encoding="utf-8")
    pc_requirements = (PACKAGE / "requirements-pc.txt").read_text(encoding="utf-8")
    assert "formal_random_scene_perception_evaluator" in setup
    assert "<exec_depend>ros_gz_interfaces</exec_depend>" in package_xml
    assert "opencv-python-headless" in pc_requirements
    assert "<exec_depend>python3-opencv</exec_depend>" in package_xml
    assert "Python 3.10-3.12" in pc_requirements
