"""Deterministic URDF-independent cube perception/manipulation smoke demo."""

from __future__ import annotations

import json
import math

from .cube_geometry import CubePointCloudDetector, generate_top_grasps
from .cube_task import CubeTaskController


def _fixture_cloud() -> list[tuple[float, float, float]]:
    cloud = [
        (x * 0.04, y * 0.04, 0.0)
        for x in range(-12, 13)
        for y in range(-8, 9)
    ]
    yaw = math.radians(23.0)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    for row in range(7):
        for column in range(7):
            u = -0.015 + row * 0.005
            v = -0.015 + column * 0.005
            cloud.append(
                (
                    0.35 + cosine * u - sine * v,
                    0.08 + sine * u + cosine * v,
                    0.030,
                )
            )
    return cloud


def run_placeholder_demo() -> dict:
    detection = CubePointCloudDetector().detect(_fixture_cloud())
    if len(detection.candidates) != 1:
        raise RuntimeError("placeholder fixture must yield exactly one cube")
    target_id = "placeholder-cube-0"
    grasps = generate_top_grasps(target_id, detection.candidates[0])
    controller = CubeTaskController()
    outcome = controller.execute(target_id, grasps)
    return {
        "schema_version": 1,
        "profile": "urdf_independent_mobile_manipulator_placeholder_v1",
        "placeholder_evidence_only": True,
        "real_robot_evidence": False,
        "gazebo_truth_used_for_control": False,
        "detected_cube_count": len(detection.candidates),
        "grasp_candidate_count": len(grasps),
        "task_state": outcome.state.value,
        "attempts": outcome.attempts,
        "placed_in_bin": outcome.placed_in_bin,
        "success": outcome.success,
    }


def main() -> int:
    summary = run_placeholder_demo()
    print(json.dumps(summary, sort_keys=True))
    return 0 if summary["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
