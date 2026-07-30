#!/usr/bin/env python3
"""Run AUTO-09 micro, closed-loop and fail-closed acceptance matrices."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(
    0, str(ROOT / "starter_ws" / "src" / "sanitation_manipulation")
)
from sanitation_manipulation.core import (  # noqa: E402
    BinState,
    ManipulationController,
    PICK_CLASSES,
    Target,
    generate_grasps,
    simulate_trial,
)


def rate(rows: list[dict], key: str) -> float:
    return sum(bool(row[key]) for row in rows) / len(rows)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--implementation-commit", required=True)
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    micro = {
        class_id: [
            simulate_trial(class_id, seed, True) for seed in range(20)
        ]
        for class_id in PICK_CLASSES
    }
    formal = {
        class_id: [
            simulate_trial(class_id, seed, False) for seed in range(30)
        ]
        for class_id in PICK_CLASSES
    }
    unreachable = []
    for class_id in PICK_CLASSES:
        for seed in range(30):
            controller = ManipulationController()
            target = Target(
                f"unreachable-{class_id}-{seed}",
                class_id,
                1.1 + seed * 0.01,
                0.0,
                0.05,
                0.002,
            )
            result = controller.execute(target, BinState(), 0.5)
            result.update({"class_id": class_id, "seed": seed})
            unreachable.append(result)

    bin_state = BinState()
    bin_state.reserve(39.8)
    bin_controller = ManipulationController()
    bin_full_result = bin_controller.execute(
        Target("bin-full", "plastic_bottle", 0.4, 0.0, 0.05, 0.001),
        bin_state,
        0.5,
    )
    estop_controller = ManipulationController()
    estop_controller.emergency_stop()
    estop_result = estop_controller.execute(
        Target("estop", "metal_can", 0.4, 0.0, 0.05, 0.001),
        BinState(),
        0.33,
    )
    brush_routing = {
        class_id: len(
            generate_grasps(
                Target(class_id, class_id, 0.4, 0.0, 0.05, 0.001)
            )
        )
        == 0
        for class_id in ("leaf_pile", "puddle")
    }

    per_class = {}
    for class_id in PICK_CLASSES:
        micro_rows = micro[class_id]
        formal_rows = formal[class_id]
        per_class[class_id] = {
            "micro_trials": len(micro_rows),
            "micro_grasp_success": rate(micro_rows, "pick_success"),
            "micro_lift_success": rate(micro_rows, "lift_success"),
            "formal_trials": len(formal_rows),
            "pick_success": rate(formal_rows, "pick_success"),
            "transport_success": rate(formal_rows, "transport_success"),
            "bin_placement_success": rate(
                formal_rows, "bin_placement_success"
            ),
        }
    all_rows = [
        row for class_rows in formal.values() for row in class_rows
    ]
    checks = {
        "system_moveit2_config_present": (
            ROOT
            / "starter_ws/src/sanitation_manipulation/config/sanitation_arm.srdf"
        ).is_file(),
        "system_ros2_control_present": (
            "SanitationManipulatorSystem"
            in (
                ROOT
                / "starter_ws/src/sanitation_vehicle_description/urdf/"
                "sanitation_manipulator.urdf.xacro"
            ).read_text(encoding="utf-8")
        ),
        "micro_at_least_20_per_class": all(
            row["micro_trials"] >= 20 for row in per_class.values()
        ),
        "micro_grasp_success_at_least_0_95": all(
            row["micro_grasp_success"] >= 0.95 for row in per_class.values()
        ),
        "micro_lift_success_at_least_0_95": all(
            row["micro_lift_success"] >= 0.95 for row in per_class.values()
        ),
        "formal_at_least_30_per_class": all(
            row["formal_trials"] >= 30 for row in per_class.values()
        ),
        "formal_pick_at_least_0_90": all(
            row["pick_success"] >= 0.90 for row in per_class.values()
        ),
        "formal_transport_at_least_0_90": all(
            row["transport_success"] >= 0.90 for row in per_class.values()
        ),
        "formal_bin_placement_at_least_0_90": all(
            row["bin_placement_success"] >= 0.90
            for row in per_class.values()
        ),
        "wrong_object_grasp_zero": not any(
            row["wrong_object_grasp"] for row in all_rows
        ),
        "drop_outside_safe_zone_zero": not any(
            row["drop_outside_safe_zone"] for row in all_rows
        ),
        "collision_zero": sum(row["collision_count"] for row in all_rows) == 0,
        "joint_limit_violation_zero": sum(
            row["joint_limit_violation_count"] for row in all_rows
        )
        == 0,
        "unreachable_fail_closed_100_percent": all(
            not row["success"] and row["reason"] == "unreachable_fail_closed"
            for row in unreachable
        ),
        "bin_nominal_capacity_at_least_40_l": bin_state.capacity_l >= 40.0,
        "bin_fill_state_observable": set(bin_state.observable)
        == {"capacity_l", "fill_l", "fill_ratio", "full"},
        "overfill_rejected_100_percent": (
            not bin_full_result["success"]
            and bin_state.fill_l == 39.8
        ),
        "bin_full_routing_tested": bin_full_result["bin_full_route"],
        "estop_fail_closed": (
            not estop_result["success"]
            and estop_result["reason"] == "estop_active"
        ),
        "leaf_puddle_brush_routing_only": all(brush_routing.values()),
        "truth_not_used_for_control": all(
            not row["truth_used_for_control"] for row in all_rows
        ),
    }
    report = {
        "schema_version": 1,
        "stage": "AUTO-09",
        "attempt_id": "AUTO-09-MANIPULATION-V1",
        "implementation_commit": args.implementation_commit,
        "source_level": "OFFLINE_KINEMATIC_PERCEPTION_LOOP_SIMULATION",
        "truth_boundary": (
            "MoveIt2 and ros2_control artifacts are delivered and statically "
            "audited. Trial metrics come from a truth-separated offline "
            "kinematic simulator, not Gazebo or a physical arm."
        ),
        "per_class": per_class,
        "micro_trials": micro,
        "formal_trials": formal,
        "unreachable_trials": unreachable,
        "bin_full_result": bin_full_result,
        "estop_result": estop_result,
        "brush_routing": brush_routing,
        "checks": checks,
        "auto09_gate_pass": all(checks.values()),
    }
    report_path = output / "formal_report.json"
    report_path.write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    summary = {
        "stage": report["stage"],
        "attempt_id": report["attempt_id"],
        "source_level": report["source_level"],
        "per_class": per_class,
        "checks": checks,
        "auto09_gate_pass": report["auto09_gate_pass"],
        "formal_report_sha256": hashlib.sha256(
            report_path.read_bytes()
        ).hexdigest(),
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))
    return 0 if report["auto09_gate_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
