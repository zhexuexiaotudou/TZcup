from pathlib import Path

from sanitation_active_cleaning import formal_observation_bridge
from sanitation_active_cleaning import formal_cleaning_coordinator
from sanitation_active_cleaning import formal_policy_planner
from sanitation_active_cleaning import formal_trajectory_executor
from sanitation_active_cleaning.formal_policy_planner import build_grasp_request
import json
import pytest


PACKAGE = Path(__file__).parents[1]


def test_ros_adapters_declare_only_product_map_path_and_safety_inputs():
    assert formal_observation_bridge.CONTROL_INPUT_TOPICS == (
        "/perception/ground_dirt/masks",
        "/perception/garbage/targets",
    )
    assert formal_trajectory_executor.CONTROL_INPUT_TOPICS == (
        "/active_cleaning/trajectory",
        "/active_cleaning/cancel",
        "/safety/actuators_enabled",
    )
    assert formal_trajectory_executor.NAVIGATION_ACTION == "/follow_path"
    assert formal_policy_planner.CONTROL_INPUT_TOPICS == (
        "/active_cleaning/ground_dirt_belief",
        "/active_cleaning/garbage_targets",
        "/active_cleaning/observation_ready",
        "/active_cleaning/executor_status",
        "/active_cleaning/grasp_result",
    )
    assert formal_cleaning_coordinator.CONTROL_INPUT_TOPICS == (
        "/active_cleaning/cleaning_requested",
        "/safety/actuators_enabled",
    )


def test_executor_uses_follow_path_cancel_and_safety_chain_without_direct_drive_output():
    source = (
        PACKAGE
        / "sanitation_active_cleaning"
        / "formal_trajectory_executor.py"
    ).read_text(encoding="utf-8")
    assert "ActionClient" in source
    assert "FollowPath" in source
    assert "cancel_goal_async" in source
    assert "safety_not_permitted_or_stale" in source
    assert "Twist" not in source
    assert "/base_controller/cmd_vel" not in source


def test_formal_launch_starts_product_adapters_and_frozen_policy_process():
    source = (PACKAGE / "launch/formal_active_cleaning.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'executable="formal_observation_bridge"' in source
    assert 'executable="formal_trajectory_executor"' in source
    assert 'executable="formal_policy_planner"' in source
    assert 'executable="formal_cleaning_coordinator"' in source
    assert 'DeclareLaunchArgument(\n                "policy_checkpoint"' in source
    assert 'DeclareLaunchArgument(\n                "maximum_task_distance_m"' in source
    assert 'DeclareLaunchArgument(\n                "episode_seed"' in source
    assert '"episode_seed": ParameterValue(episode_seed, value_type=int)' in source
    assert "active_cleaning_train" not in source


def test_policy_planner_emits_only_path_cleaning_and_grasp_requests_not_drive_commands():
    source = (
        PACKAGE / "sanitation_active_cleaning" / "formal_policy_planner.py"
    ).read_text(encoding="utf-8")
    assert "FormalRuntimePolicyCore" in source
    assert "truth_used_for_control" in source
    assert "nav_msgs.msg import OccupancyGrid, Odometry, Path" in source
    assert "/active_cleaning/grasp_request" in source
    assert "RETURNING_HOME" in source
    assert "task_distance_m_excluding_return" in source
    assert "Twist" not in source
    assert "/base_controller/cmd_vel" not in source


def test_policy_planner_forwards_complete_truth_free_3d_grasp_observation():
    payload = json.loads(
        build_grasp_request(
            target_id="track-7",
            frame_id="map",
            pose=(1.0, 2.0, 0.015, 0.0, 0.0, 0.0, 1.0),
            size_m=(0.03, 0.03, 0.03),
            confidence=0.91,
        )
    )
    assert payload["schema_version"] == 2
    assert payload["pose"]["z_m"] == 0.015
    assert payload["size_m"] == [0.03, 0.03, 0.03]
    assert payload["material"] == "unknown"
    assert payload["truth_used"] is False


def test_policy_planner_rejects_incomplete_or_nonphysical_grasp_observation():
    with pytest.raises(ValueError, match="dimensions"):
        build_grasp_request(
            target_id="track-8",
            frame_id="map",
            pose=(1.0, 2.0, 0.015, 0.0, 0.0, 0.0, 1.0),
            size_m=(0.0, 0.03, 0.03),
            confidence=0.91,
        )


def test_cleaning_coordinator_uses_safety_inputs_and_watchdogs_all_outputs():
    source = (
        PACKAGE / "sanitation_active_cleaning" / "formal_cleaning_coordinator.py"
    ).read_text(encoding="utf-8")
    assert "/safety/command/brush" in source
    assert "/safety/command/pump" in source
    assert "request_timeout_sec" in source
    assert "safety_timeout_sec" in source
    assert "joint_state_timeout_sec" in source
    assert "FormalCleaningCore" in source
    assert "JointState" in source
    assert "water_recovery/command/enable" in source
    assert "[0.0, 0.0, 0.0]" in source
    assert "[0.0]" in source
