from __future__ import annotations

import copy

from formal_vehicle_mobility_metrics import WHEEL_JOINTS, evaluate_estop_stop


def _raw() -> dict:
    zeros = {joint: 0.0 for joint in WHEEL_JOINTS}
    return {
        "estop": {
            "trigger_sim_time_ns": 1_000_000_000,
            "final_command_trace": [
                {"sim_time_ns": 900_000_000, "linear_x_mps": 1.0, "angular_z_rad_s": 0.0},
                {"sim_time_ns": 1_600_000_000, "linear_x_mps": 0.0, "angular_z_rad_s": 0.0},
            ],
            "final_command_writer": {
                "writers": ["/whole_vehicle_safety_manager"],
                "expected_sole_writer": "/whole_vehicle_safety_manager",
                "input_subscription_count": 1,
            },
            "emergency_stop_feedback_trace": [{"sim_time_ns": 1_100_000_000, "active": True}],
            "safety_status_trace": [],
            "ground_truth": {
                "motion_start": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                "trigger": {"x": 0.2, "y": 0.0, "yaw": 0.0},
                "stopped_end": {"x": 0.25, "y": 0.0, "yaw": 0.0},
            },
            "plant_odom": {
                "trigger": {"x": 0.2, "y": 0.0, "yaw": 0.0},
                "stopped_end": {"x": 0.25, "y": 0.0, "yaw": 0.0},
                "stopped_linear_velocity_mps": {"x": 0.0, "y": 0.0},
            },
            "wheel_state": {"stopped_velocities_rad_s": zeros},
        }
    }


def test_estop_requires_pretrigger_final_speed_writer_and_feedback() -> None:
    assert evaluate_estop_stop(_raw())["passed"] is True
    raw = copy.deepcopy(_raw())
    raw["estop"]["final_command_trace"][0]["linear_x_mps"] = 0.45
    result = evaluate_estop_stop(raw)
    assert result["checks"]["final_safety_command_reached_one_mps_before_estop"] is False

    raw = copy.deepcopy(_raw())
    raw["estop"]["final_command_writer"]["writers"] = ["/unexpected_writer"]
    result = evaluate_estop_stop(raw)
    assert result["checks"]["final_safety_command_has_one_expected_writer_and_input_subscriber"] is False
