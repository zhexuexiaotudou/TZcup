"""Evaluation harness joining scenario, planning, and mock manipulation.

The harness is intentionally the only component that reads evaluator files.
The policy receives ``AgentObservation`` only.  The grasp callback receives a
currently observed target id and a synthetic perceived 30 mm geometry record;
it never reads Gazebo or scenario truth.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

from sanitation_active_cleaning.environment import (
    ActiveCleaningEnv,
    GraspVerificationResult,
    create_evaluation_token,
)
from sanitation_active_cleaning.models import Pose2D, RoleSeeds, TaskConfig, TaskLayout
from sanitation_active_cleaning.policies import SensingGreedyPolicy
from sanitation_manipulation.active_cleaning_adapter import (
    ActiveCleaningManipulationAdapter,
    SingleTargetGraspRequest,
)
from sanitation_manipulation.cube_geometry import CubeCandidate


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _rotated_box(asset: Mapping[str, Any]) -> tuple[tuple[float, float], ...]:
    pose = asset["pose"]
    width, height = (float(value) for value in asset["size_m"][:2])
    x = float(pose["x_m"])
    y = float(pose["y_m"])
    yaw = float(pose["yaw_rad"])
    cosine, sine = math.cos(yaw), math.sin(yaw)
    result = []
    for local_x, local_y in (
        (-width / 2.0, -height / 2.0),
        (width / 2.0, -height / 2.0),
        (width / 2.0, height / 2.0),
        (-width / 2.0, height / 2.0),
    ):
        result.append(
            (
                x + cosine * local_x - sine * local_y,
                y + sine * local_x + cosine * local_y,
            )
        )
    return tuple(result)


@dataclass(frozen=True)
class Bundle:
    root: Path
    public_manifest: Mapping[str, Any]
    evaluator_manifest: Mapping[str, Any]
    truth: Mapping[str, Any]
    public_manifest_sha256: str
    world_sha256: str

    @classmethod
    def load(cls, root: str | Path) -> "Bundle":
        base = Path(root).resolve()
        public_path = base / "public" / "episode_manifest.json"
        evaluator_path = base / "evaluator" / "episode_manifest.json"
        truth_path = base / "evaluator" / "ground_truth.json"
        world_path = base / "public" / "world.sdf"
        public = _load_json(public_path)
        evaluator = _load_json(evaluator_path)
        truth = _load_json(truth_path)
        identities = {
            (str(item["episode_id"]), str(item["map_id"]))
            for item in (public, evaluator, truth)
        }
        if len(identities) != 1:
            raise ValueError("public/evaluator/truth episode identity mismatch")
        if truth.get("control_use_prohibited") is not True:
            raise ValueError("evaluator truth must prohibit controller use")
        expected_world_hash = str(evaluator.get("world_sha256", ""))
        actual_world_hash = _sha256(world_path)
        if not expected_world_hash or actual_world_hash != expected_world_hash:
            raise ValueError("world hash mismatch")
        return cls(
            root=base,
            public_manifest=public,
            evaluator_manifest=evaluator,
            truth=truth,
            public_manifest_sha256=_sha256(public_path),
            world_sha256=actual_world_hash,
        )


def build_active_task(bundle: Bundle) -> tuple[TaskConfig, TaskLayout]:
    public = bundle.public_manifest
    truth = bundle.truth
    field = public["field"]
    start = public["vehicle_start_pose_map"]
    geofence = tuple(
        (float(point[0]), float(point[1])) for point in field["geofence_polygon_m"]
    )
    obstacles = tuple(_rotated_box(asset) for asset in truth["static_assets"])
    dirt_regions = tuple(
        (
            float(item["pose"]["x_m"]),
            float(item["pose"]["y_m"]),
            math.sqrt(float(item["area_m2"]) / math.pi),
        )
        for item in truth["dirt_patches"]
    )
    targets = tuple(
        (
            str(item["object_id"]),
            float(item["pose"]["x_m"]),
            float(item["pose"]["y_m"]),
        )
        for item in truth["discrete_cubes"]
    )
    pedestrians = tuple(
        (
            float(item["waypoints"][0][1]),
            float(item["waypoints"][0][2]),
            0.0,
        )
        for item in truth["pedestrians"]
    )
    config = TaskConfig.from_mapping(
        {
            "geofence": geofence,
            "static_obstacles": obstacles,
            "start": {
                "x": float(start["x_m"]),
                "y": float(start["y_m"]),
                "yaw": float(start["yaw_rad"]),
            },
            "grid_resolution": 1.0,
            "sensing_radius": 15.0,
            "sensing_fov_rad": math.tau,
            "cleaning_width": 0.70,
            "vehicle_radius": 0.36,
            "grasp_radius": 0.60,
            "min_turn_radius": 0.30,
            "path_sample_spacing": 0.20,
            "observation_threshold": 0.95,
            "ground_clear_threshold": 0.95,
            "discrete_clear_threshold": 0.95,
            "ground_dirt_count": 0,
            "discrete_target_count": 0,
            "pedestrian_count": 0,
            "max_grasp_attempts": 2,
            "grasp_success_probability": 0.0,
            "max_steps": 80,
        }
    )
    return config, TaskLayout(
        ground_dirt_regions=dirt_regions,
        discrete_targets=targets,
        pedestrians=pedestrians,
    )


def _ratio(numerator: int, denominator: int) -> float:
    return 1.0 if denominator == 0 else numerator / denominator


def run_bundle(root: str | Path) -> dict[str, Any]:
    bundle = Bundle.load(root)
    config, layout = build_active_task(bundle)
    manipulation = ActiveCleaningManipulationAdapter()
    decisions: list[Mapping[str, Any]] = []

    def verify_grasp(target_id, _target_position, _observation):  # type: ignore[no-untyped-def]
        perceived_cube = CubeCandidate(
            center_m=(0.35, 0.0, 0.015),
            size_m=(0.030, 0.030, 0.030),
            yaw_rad=0.0,
            point_count=49,
            dimension_error_m=0.0,
        )
        decision = manipulation.execute(
            SingleTargetGraspRequest(target_id, perceived_cube)
        )
        decisions.append(decision.evidence)
        return GraspVerificationResult(
            decision.verified_in_bin,
            source="urdf_independent_manipulation_adapter",
        )

    token = create_evaluation_token()
    env = ActiveCleaningEnv(
        config,
        evaluation_token=token,
        task_layout=layout,
        grasp_verifier=verify_grasp,
    )
    seed = int(bundle.evaluator_manifest["seeds"]["sensor"])
    observation = env.reset(seed=seed)
    policy = SensingGreedyPolicy(config)
    policy.reset(episode_seed=RoleSeeds.from_master(seed).policy)
    while True:
        truth = env.evaluation_snapshot(token)
        if truth.terminated or truth.truncated:
            break
        observation = env.step(policy.act(observation)).observation
    truth = env.evaluation_snapshot(token)
    observed_free = sum(
        observed and free for observed, free in zip(truth.observed, env.grid.traversable)
    )
    observed_ratio = observed_free / sum(env.grid.traversable)
    observed_ground = {
        index for index in truth.initial_ground_dirt_cells if truth.observed[index]
    }
    cleared_ground = observed_ground - truth.remaining_ground_dirt_cells
    observed_targets = {
        target_id
        for target_id, x, y in truth.initial_targets
        if truth.observed[env.grid.nearest_index((x, y))]
    }
    cleared_targets = observed_targets & truth.cleared_target_ids
    ground_ratio = _ratio(len(cleared_ground), len(observed_ground))
    target_ratio = _ratio(len(cleared_targets), len(observed_targets))
    success = bool(
        truth.terminated
        and not truth.truncated
        and observed_ratio >= config.observation_threshold
        and ground_ratio >= config.ground_clear_threshold
        and target_ratio >= config.discrete_clear_threshold
        and truth.collisions == 0
        and truth.boundary_violations == 0
        and truth.invalid_actions == 0
    )
    return {
        "schema_version": 1,
        "episode_identity": {
            "split": bundle.public_manifest["split"],
            "map_id": bundle.public_manifest["map_id"],
            "episode_id": bundle.public_manifest["episode_id"],
            "public_manifest_sha256": bundle.public_manifest_sha256,
            "world_sha256": bundle.world_sha256,
        },
        "policy": "sensing_greedy",
        "policy_input": "serialized_public_belief_only",
        "truth_boundary": "evaluation_harness_only_not_policy_or_manipulation",
        "grasp_verification": "external_urdf_independent_adapter",
        "metrics": {
            "success": success,
            "terminated": truth.terminated,
            "truncated": truth.truncated,
            "steps": truth.step_index,
            "observed_ratio": observed_ratio,
            "ground_clear_ratio": ground_ratio,
            "discrete_clear_ratio": target_ratio,
            "task_distance_m": truth.task_distance,
            "collisions": truth.collisions,
            "boundary_violations": truth.boundary_violations,
            "invalid_actions": truth.invalid_actions,
            "return_distance_included": False,
            "time_or_energy_scored": False,
        },
        "manipulation": {
            "requested_targets": len(decisions),
            "verified_in_bin": sum(
                bool(row["decision"]["verified_in_bin"]) for row in decisions
            ),
            "evidence_sha256": [row["evidence_sha256"] for row in decisions],
        },
        "authority": {
            "placeholder_evidence_only": True,
            "real_robot_evidence": False,
            "measured_urdf_used": False,
            "s100_runtime_used": False,
            "journey6_evidence": False,
            "gazebo_truth_used_for_control": False,
        },
    }
