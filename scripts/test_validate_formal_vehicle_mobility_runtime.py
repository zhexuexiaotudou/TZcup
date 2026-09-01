from __future__ import annotations

import ast
import copy
from pathlib import Path

import pytest

from formal_vehicle_mobility_metrics import WHEEL_JOINTS, evaluate_motion


def _watchdog_type() -> type:
    """Load the ROS-independent watchdog class without importing rclpy on Windows."""

    validator = Path(__file__).with_name("validate_formal_vehicle_mobility_runtime.py")
    tree = ast.parse(validator.read_text(encoding="utf-8"), filename=str(validator))
    watchdog = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "SimulationClockProgressWatchdog"
    )
    module = ast.Module(body=[watchdog], type_ignores=[])
    namespace: dict[str, object] = {}
    exec(compile(module, str(validator), "exec"), namespace)
    return namespace["SimulationClockProgressWatchdog"]  # type: ignore[return-value]


def _evidence() -> dict:
    return {
        "joint_state_broadcaster_state": "active",
        "command_subscription_count": 1,
        "actuator_enabled_sample_count": 1,
        "ground_truth": {
            "start": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "forward_end": {"x": 0.82, "y": 0.01, "yaw": 0.01},
            "stopped_end": {"x": 0.84, "y": 0.01, "yaw": 0.01},
        },
        "plant_odom": {
            "start": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "forward_end": {"x": 0.80, "y": 0.0, "yaw": 0.0},
            "stopped_end": {"x": 0.82, "y": 0.0, "yaw": 0.0},
            "stopped_linear_velocity_mps": {"x": 0.0, "y": 0.0},
            "stopped_angular_velocity_rad_s": 0.0,
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


def test_rejects_stationary_vehicle_even_when_plant_is_enabled() -> None:
    evidence = _evidence()
    evidence["ground_truth"]["forward_end"]["x"] = 0.0
    evidence["plant_odom"]["forward_end"]["x"] = 0.0
    evidence["wheel_state"]["forward_end_positions_rad"] = {name: 0.0 for name in WHEEL_JOINTS}
    result = evaluate_motion(evidence)
    assert result["passed"] is False
    assert result["checks"]["ground_truth_forward_motion"] is False


def test_rejects_odometry_that_disagrees_with_gazebo_truth() -> None:
    evidence = _evidence()
    evidence["plant_odom"]["forward_end"]["x"] = 1.25
    result = evaluate_motion(evidence)
    assert result["passed"] is False
    assert result["checks"]["odometry_matches_ground_truth"] is False


def test_rejects_vehicle_that_keeps_rolling_after_zero_command() -> None:
    evidence = copy.deepcopy(_evidence())
    evidence["ground_truth"]["stopped_end"]["x"] = 1.10
    evidence["plant_odom"]["stopped_end"]["x"] = 1.05
    evidence["plant_odom"]["stopped_linear_velocity_mps"]["x"] = 0.12
    evidence["wheel_state"]["stopped_velocities_rad_s"]["front_left_wheel_joint"] = 1.0
    result = evaluate_motion(evidence)
    assert result["passed"] is False
    assert result["checks"]["vehicle_stopped_after_zero_command"] is False
    assert result["checks"]["plant_odometry_stopped_after_zero_command"] is False
    assert result["checks"]["wheel_joints_stopped_after_zero_command"] is False


def test_rejects_vehicle_that_rotates_after_zero_command() -> None:
    evidence = copy.deepcopy(_evidence())
    evidence["ground_truth"]["stopped_end"]["yaw"] = 0.12
    evidence["plant_odom"]["stopped_end"]["yaw"] = 0.15
    evidence["plant_odom"]["stopped_angular_velocity_rad_s"] = 0.04

    result = evaluate_motion(evidence)

    assert result["passed"] is False
    assert result["checks"]["ground_truth_heading_stopped_after_zero_command"] is False
    assert result["checks"]["plant_odometry_heading_stopped_after_zero_command"] is False


def test_accepts_bounded_skid_steer_wheel_odom_coast_after_physical_stop() -> None:
    evidence = copy.deepcopy(_evidence())
    evidence["ground_truth"]["stopped_end"]["x"] = 0.86
    evidence["plant_odom"]["stopped_end"]["x"] = 0.92
    result = evaluate_motion(evidence)
    assert result["checks"]["vehicle_stopped_after_zero_command"] is True
    assert result["checks"]["plant_odometry_stopped_after_zero_command"] is True
    assert result["metrics"]["stop_coast_disagreement_m"] <= 0.10


def test_slow_simulation_with_continuous_clock_progress_does_not_time_out() -> None:
    watchdog_type = _watchdog_type()
    watchdog = watchdog_type(
        initial_sim_ns=0,
        initial_wall_s=0.0,
        stall_timeout_s=5.0,
        hard_timeout_s=100.0,
    )
    sim_ns = 0
    # RTF 0.0025 is much slower than the measured formal-vehicle RTF ~0.06,
    # yet each four-wall-second observation advances simulation time.
    for wall_s in range(4, 81, 4):
        sim_ns += 10_000_000
        watchdog.observe(sim_ns, float(wall_s))


def test_stalled_simulation_clock_trips_no_progress_watchdog() -> None:
    watchdog_type = _watchdog_type()
    watchdog = watchdog_type(
        initial_sim_ns=1_000_000,
        initial_wall_s=10.0,
        stall_timeout_s=5.0,
        hard_timeout_s=100.0,
    )
    watchdog.observe(1_000_000, 14.99)
    with pytest.raises(TimeoutError, match="made no progress"):
        watchdog.observe(1_000_000, 15.0)


def test_progressing_clock_still_has_configurable_hard_wall_limit() -> None:
    watchdog_type = _watchdog_type()
    watchdog = watchdog_type(
        initial_sim_ns=0,
        initial_wall_s=0.0,
        stall_timeout_s=2.0,
        hard_timeout_s=10.0,
    )
    for wall_s in range(1, 10):
        watchdog.observe(wall_s * 1_000_000, float(wall_s))
    with pytest.raises(TimeoutError, match="hard wall limit"):
        watchdog.observe(10_000_000, 10.0)


def test_spin_completion_is_sim_time_based_not_duration_multiplier() -> None:
    source = Path(__file__).with_name("validate_formal_vehicle_mobility_runtime.py").read_text(
        encoding="utf-8"
    )
    assert "while simulated < duration:" in source
    assert "duration * 15.0" not in source
    assert "--clock-stall-timeout" in source
    assert "--phase-hard-timeout" in source
