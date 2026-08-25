from __future__ import annotations

import copy

from formal_vehicle_mobility_metrics import WHEEL_JOINTS, evaluate_motion


def _evidence() -> dict:
    return {
        "controller_state": "active",
        "command_subscription_count": 1,
        "ground_truth": {
            "start": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "forward_end": {"x": 0.82, "y": 0.01, "yaw": 0.01},
            "stopped_end": {"x": 0.84, "y": 0.01, "yaw": 0.01},
        },
        "controller_odom": {
            "start": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "forward_end": {"x": 0.80, "y": 0.0, "yaw": 0.0},
            "stopped_end": {"x": 0.82, "y": 0.0, "yaw": 0.0},
            "stopped_linear_velocity_mps": {"x": 0.0, "y": 0.0},
        },
        "wheel_state": {
            "observed_names": list(WHEEL_JOINTS),
            "start_positions_rad": {name: 0.0 for name in WHEEL_JOINTS},
            "forward_end_positions_rad": {name: 5.0 for name in WHEEL_JOINTS},
            "stopped_velocities_rad_s": {name: 0.0 for name in WHEEL_JOINTS},
        },
    }


def test_accepts_consistent_forward_motion_and_stop() -> None:
    result = evaluate_motion(_evidence())
    assert result["passed"] is True
    assert all(result["checks"].values())
    assert result["metrics"]["ground_truth_forward_delta_m"] == 0.82


def test_rejects_stationary_vehicle_even_when_controller_is_active() -> None:
    evidence = _evidence()
    evidence["ground_truth"]["forward_end"]["x"] = 0.0
    evidence["controller_odom"]["forward_end"]["x"] = 0.0
    evidence["wheel_state"]["forward_end_positions_rad"] = {name: 0.0 for name in WHEEL_JOINTS}
    result = evaluate_motion(evidence)
    assert result["passed"] is False
    assert result["checks"]["ground_truth_forward_motion"] is False


def test_rejects_odometry_that_disagrees_with_gazebo_truth() -> None:
    evidence = _evidence()
    evidence["controller_odom"]["forward_end"]["x"] = 1.25
    result = evaluate_motion(evidence)
    assert result["passed"] is False
    assert result["checks"]["odometry_matches_ground_truth"] is False


def test_rejects_vehicle_that_keeps_rolling_after_zero_command() -> None:
    evidence = copy.deepcopy(_evidence())
    evidence["ground_truth"]["stopped_end"]["x"] = 1.10
    evidence["controller_odom"]["stopped_end"]["x"] = 1.05
    evidence["controller_odom"]["stopped_linear_velocity_mps"]["x"] = 0.12
    evidence["wheel_state"]["stopped_velocities_rad_s"]["front_left_wheel_joint"] = 1.0
    result = evaluate_motion(evidence)
    assert result["passed"] is False
    assert result["checks"]["vehicle_stopped_after_zero_command"] is False
    assert result["checks"]["odometry_stopped_after_zero_command"] is False
    assert result["checks"]["wheel_joints_stopped_after_zero_command"] is False
