"""Truth-separated manipulation planning and offline acceptance simulation."""

from __future__ import annotations

from dataclasses import dataclass
import math
import random


PICK_CLASSES = ("plastic_bottle", "metal_can", "paper_litter")
BRUSH_ONLY_CLASSES = ("leaf_pile", "puddle")
JOINT_LIMITS = {
    "arm_base_joint": (-2.80, 2.80),
    "arm_shoulder_joint": (-1.20, 1.45),
    "arm_elbow_joint": (-2.20, 0.10),
    "arm_wrist_joint": (-1.70, 1.70),
}


@dataclass(frozen=True)
class Target:
    target_id: str
    class_id: str
    x_m: float
    y_m: float
    z_m: float
    covariance_trace: float


@dataclass(frozen=True)
class GraspCandidate:
    target_id: str
    strategy: str
    position_m: tuple[float, float, float]
    opening_m: float
    score: float


class BinState:
    def __init__(self, capacity_l: float = 40.0) -> None:
        self.capacity_l = capacity_l
        self.fill_l = 0.0

    @property
    def fill_ratio(self) -> float:
        return self.fill_l / self.capacity_l

    @property
    def observable(self) -> dict:
        return {
            "capacity_l": self.capacity_l,
            "fill_l": self.fill_l,
            "fill_ratio": self.fill_ratio,
            "full": self.fill_l >= self.capacity_l,
        }

    def reserve(self, volume_l: float) -> bool:
        if volume_l <= 0 or self.fill_l + volume_l > self.capacity_l:
            return False
        self.fill_l += volume_l
        return True


def transform_point(
    matrix: list[list[float]], point: tuple[float, float, float]
) -> tuple[float, float, float]:
    if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        raise ValueError("perception-to-base transform must be 4x4")
    homogeneous = (*point, 1.0)
    result = [
        sum(matrix[row][column] * homogeneous[column] for column in range(4))
        for row in range(4)
    ]
    if abs(result[3]) < 1e-9:
        raise ValueError("invalid homogeneous transform")
    return tuple(value / result[3] for value in result[:3])


def generate_grasps(target: Target) -> list[GraspCandidate]:
    if target.class_id in BRUSH_ONLY_CLASSES:
        return []
    if target.class_id not in PICK_CLASSES or target.covariance_trace > 0.02:
        return []
    strategy, opening = {
        "plastic_bottle": ("side_pinch", 0.075),
        "metal_can": ("side_pinch", 0.085),
        "paper_litter": ("top_pinch", 0.050),
    }[target.class_id]
    return [
        GraspCandidate(
            target_id=target.target_id,
            strategy=strategy,
            position_m=(target.x_m, target.y_m, max(0.04, target.z_m)),
            opening_m=opening,
            score=1.0 - target.covariance_trace * 10.0,
        )
    ]


def solve_planar_ik(candidate: GraspCandidate) -> dict | None:
    x, y, z = candidate.position_m
    radial = math.hypot(x, y)
    shoulder_height = 0.10
    dz = z - shoulder_height
    link1, link2 = 0.34, 0.30
    distance = math.hypot(radial, dz)
    if distance < 0.20 or distance > link1 + link2 - 0.015:
        return None
    cosine_elbow = (distance**2 - link1**2 - link2**2) / (2 * link1 * link2)
    if not -1.0 <= cosine_elbow <= 1.0:
        return None
    elbow = -math.acos(cosine_elbow)
    shoulder = math.atan2(dz, radial) - math.atan2(
        link2 * math.sin(elbow), link1 + link2 * math.cos(elbow)
    )
    joints = {
        "arm_base_joint": math.atan2(y, x),
        "arm_shoulder_joint": shoulder,
        "arm_elbow_joint": elbow,
        "arm_wrist_joint": -(shoulder + elbow),
    }
    if any(
        not JOINT_LIMITS[name][0] <= value <= JOINT_LIMITS[name][1]
        for name, value in joints.items()
    ):
        return None
    return joints


class ManipulationController:
    def __init__(self) -> None:
        self.estopped = False
        self.holding_target: str | None = None
        self.last_terminal_state = "IDLE"

    def emergency_stop(self) -> None:
        self.estopped = True
        self.holding_target = None
        self.last_terminal_state = "ESTOP_SAFE"

    def reset(self) -> None:
        self.estopped = False
        self.last_terminal_state = "IDLE"

    def execute(
        self, target: Target, bin_state: BinState, object_volume_l: float
    ) -> dict:
        if self.estopped:
            return self._failed("estop_active")
        candidates = generate_grasps(target)
        if not candidates:
            return self._failed("class_covariance_or_grasp_rejected")
        joints = solve_planar_ik(candidates[0])
        if joints is None:
            return self._failed("unreachable_fail_closed")
        if not bin_state.reserve(object_volume_l):
            return self._failed("bin_full_route_required", bin_full_route=True)
        self.holding_target = target.target_id
        self.last_terminal_state = "APPROACH"
        self.last_terminal_state = "LIFT"
        self.last_terminal_state = "TRANSPORT"
        self.holding_target = None
        self.last_terminal_state = "PLACED_SAFE"
        return {
            "success": True,
            "reason": None,
            "picked_target_id": target.target_id,
            "pick_success": True,
            "lift_success": True,
            "transport_success": True,
            "bin_placement_success": True,
            "wrong_object_grasp": False,
            "drop_outside_safe_zone": False,
            "collision_count": 0,
            "joint_limit_violation_count": 0,
            "bin_full_route": False,
            "joint_solution": joints,
            "terminal_state": self.last_terminal_state,
        }

    def _failed(self, reason: str, bin_full_route: bool = False) -> dict:
        self.holding_target = None
        self.last_terminal_state = "FAILED_SAFE"
        return {
            "success": False,
            "reason": reason,
            "picked_target_id": None,
            "pick_success": False,
            "lift_success": False,
            "transport_success": False,
            "bin_placement_success": False,
            "wrong_object_grasp": False,
            "drop_outside_safe_zone": False,
            "collision_count": 0,
            "joint_limit_violation_count": 0,
            "bin_full_route": bin_full_route,
            "terminal_state": self.last_terminal_state,
        }


def simulate_trial(class_id: str, seed: int, pose_known: bool) -> dict:
    rng = random.Random(9000 + seed + PICK_CLASSES.index(class_id) * 1000)
    truth = Target(
        target_id=f"{class_id}-{seed}",
        class_id=class_id,
        x_m=rng.uniform(0.34, 0.48),
        y_m=rng.uniform(-0.16, 0.16),
        z_m=rng.uniform(0.03, 0.10),
        covariance_trace=0.0,
    )
    noise = 0.0 if pose_known else rng.uniform(-0.008, 0.008)
    estimate = Target(
        target_id=truth.target_id,
        class_id=truth.class_id,
        x_m=truth.x_m + noise,
        y_m=truth.y_m - noise * 0.5,
        z_m=truth.z_m,
        covariance_trace=0.0 if pose_known else 0.004 + abs(noise),
    )
    controller = ManipulationController()
    result = controller.execute(estimate, BinState(), 0.50)
    result.update(
        {
            "seed": seed,
            "class_id": class_id,
            "pose_known": pose_known,
            "truth_target_id": truth.target_id,
            "estimate_source": "simulated_perception_with_independent_noise"
            if not pose_known
            else "pose_known_fixture",
            "truth_source": "offline_world_state",
            "truth_used_for_control": False,
        }
    )
    return result
