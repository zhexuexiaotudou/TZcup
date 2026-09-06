"""Reproducible multi-map training over the frozen formal campus split.

The evaluator-side scenario truth initializes the simulator and computes final
metrics.  It is never included in :class:`AgentObservation` and is never passed
to a policy.  Reports remain explicitly research-only until a held-out run is
fed by the product DOSOD+EdgeSAM ROS topics and a real manipulation verifier.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any, Iterable, Sequence

import yaml

from .environment import ActiveCleaningEnv, StepResult
from .evaluation import run_episode
from .formal_budget import load_formal_rl_budget
from .models import Point2D, Pose2D, RoleSeeds, TaskConfig, TaskLayout
from .policies import FullCoveragePolicy
from .rl import CoverageBackstoppedQLearningPolicy, QLearningPolicy


FORMAL_FULL_MAP_COUNTS: dict[str, int] = {
    "train": 32,
    "validation": 8,
    "hidden": 12,
}


@dataclass(frozen=True)
class FormalSplitManifest:
    """The complete frozen multi-map task split used by the formal gate.

    One mission per map is a topology smoke check only.  Formal generalized
    training consumes every frozen map/mission pair (6400/800/1200 tasks).
    """

    scenario_config: Path
    map_counts: dict[str, int]
    selections: dict[str, tuple[tuple[int, int], ...]]

    def report(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "scenario_config": str(self.scenario_config),
            "required_map_counts": dict(self.map_counts),
            "selections": {
                name: [f"{map_index}:{mission_index}" for map_index, mission_index in rows]
                for name, rows in self.selections.items()
            },
            "all_frozen_missions_per_declared_map": True,
            "smoke_subset_accepted_as_generalization": False,
        }


def load_full_formal_split_manifest(
    scenario_config: str | Path, budget_contract: str | Path | None = None
) -> FormalSplitManifest:
    """Validate the frozen formal scenario and build its non-reducible split.

    A different YAML file or a smaller map count is not a substitute for the
    frozen 32/8/12 contract.  Refuse before materialization so an undersized
    run cannot later be labelled a multi-map generalization result.
    """
    path = Path(scenario_config)
    config = _yaml(path)
    split = config.get("split")
    if not isinstance(split, dict):
        raise ValueError("frozen scenario lacks a split mapping")
    source_keys = {"train": "train", "validation": "val", "hidden": "hidden"}
    actual: dict[str, int] = {}
    for name, source_key in source_keys.items():
        row = split.get(source_key)
        if not isinstance(row, dict):
            raise ValueError(f"frozen scenario lacks split.{source_key}")
        count = row.get("map_count")
        if isinstance(count, bool) or not isinstance(count, int):
            raise ValueError(f"frozen scenario split.{source_key}.map_count is invalid")
        actual[name] = count
    if actual != FORMAL_FULL_MAP_COUNTS:
        raise ValueError(
            "formal multi-map generalization requires frozen map counts "
            f"{FORMAL_FULL_MAP_COUNTS}, got {actual}"
        )
    budget = load_formal_rl_budget(budget_contract) if budget_contract else None
    if budget is not None:
        if actual != budget.multimap_map_counts:
            raise ValueError("scenario map counts disagree with formal RL budget")
        expected_missions = budget.multimap_missions_per_map
        for name, source_key in source_keys.items():
            if split[source_key].get("missions_per_map") != expected_missions[name]:
                raise ValueError("scenario missions_per_map disagrees with formal RL budget")
    missions = {
        name: int(split[source_key]["missions_per_map"])
        for name, source_key in source_keys.items()
    }
    return FormalSplitManifest(
        scenario_config=path.resolve(),
        map_counts=actual,
        selections={
            name: tuple(
                (map_index, mission_index)
                for map_index in range(count)
                for mission_index in range(missions[name])
            )
            for name, count in actual.items()
        },
    )


def _require_full_map_selection(
    name: str,
    supplied: Sequence[tuple[int, int]] | None,
    manifest: FormalSplitManifest,
) -> tuple[tuple[int, int], ...]:
    expected = manifest.selections[name]
    if supplied is None:
        return expected
    actual = tuple(supplied)
    if actual != expected:
        raise ValueError(
            f"{name} selection is not the frozen full-map manifest; "
            f"expected {len(expected)} frozen map/mission selections"
        )
    return actual


def _validate_full_map_generalization(
    train: Sequence[FormalEpisode],
    validation: Sequence[FormalEpisode],
    test: Sequence[FormalEpisode],
) -> dict[str, Any]:
    """Reject incomplete map coverage even for direct Python callers."""
    expected = (
        ("train", "train", train),
        ("validation", "val", validation),
        ("hidden", "hidden", test),
    )
    actual_counts: dict[str, int] = {}
    map_indices: dict[str, list[int]] = {}
    for name, episode_split, rows in expected:
        by_index: dict[int, set[str]] = {}
        for episode in rows:
            if episode.split != episode_split:
                raise ValueError(
                    f"{name} contains an episode declared as {episode.split!r}"
                )
            by_index.setdefault(episode.map_index, set()).add(episode.map_id)
        if any(len(map_ids) != 1 for map_ids in by_index.values()):
            raise ValueError(f"{name} map index does not identify exactly one map")
        indices = sorted(by_index)
        if indices != list(range(FORMAL_FULL_MAP_COUNTS[name])):
            raise ValueError(
                f"{name} must cover every frozen map index 0.."
                f"{FORMAL_FULL_MAP_COUNTS[name] - 1}; got {indices}"
            )
        actual_counts[name] = len(by_index)
        map_indices[name] = indices
    return {
        "required_map_counts": dict(FORMAL_FULL_MAP_COUNTS),
        "actual_distinct_map_counts": actual_counts,
        "map_indices": map_indices,
        "full_map_coverage": True,
        "smoke_subset_accepted_as_generalization": False,
    }


@dataclass(frozen=True)
class FormalEpisode:
    split: str
    map_index: int
    mission_index: int
    map_id: str
    episode_id: str
    mission_seed: int
    area_m2: float
    aspect_ratio: float
    config: TaskConfig
    layout: TaskLayout
    artifact_root: Path


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON mapping: {path}")
    return value


def _yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected YAML mapping: {path}")
    return value


def _rotated_box(center: Point2D, size: Point2D, yaw: float) -> tuple[Point2D, ...]:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    result = []
    for local_x, local_y in (
        (-size[0] / 2.0, -size[1] / 2.0),
        (size[0] / 2.0, -size[1] / 2.0),
        (size[0] / 2.0, size[1] / 2.0),
        (-size[0] / 2.0, size[1] / 2.0),
    ):
        result.append(
            (
                center[0] + cosine * local_x - sine * local_y,
                center[1] + sine * local_x + cosine * local_y,
            )
        )
    return tuple(result)


def _pedestrian_pose(row: dict[str, Any]) -> tuple[float, float, float]:
    waypoints = row.get("waypoints")
    if not isinstance(waypoints, list) or len(waypoints) < 2:
        raise ValueError("formal pedestrian needs at least two waypoints")
    first, second = waypoints[0], waypoints[1]
    x0, y0 = float(first[1]), float(first[2])
    yaw = math.atan2(float(second[2]) - y0, float(second[1]) - x0)
    return x0, y0, yaw


def _cube_grasp_reach_radius(public: dict[str, Any]) -> float:
    contract = public.get("cube_contract")
    if not isinstance(contract, dict) or "grasp_reach_radius_m" not in contract:
        raise ValueError("cube contract lacks physical grasp_reach_radius_m")
    reach = float(contract["grasp_reach_radius_m"])
    clearance = float(contract.get("grasp_clearance_m", 0.0))
    if reach <= 0.0 or clearance < reach:
        raise ValueError("cube placement clearance must contain the grasp reach window")
    return reach


def load_materialized_episode(
    artifact_root: str | Path,
    *,
    split: str,
    map_index: int,
    mission_index: int,
    planning_resolution_m: float = 1.0,
    sensing_radius_m: float = 10.0,
    sensing_fov_rad: float = math.radians(87.0),
    max_steps: int = 400,
) -> FormalEpisode:
    root = Path(artifact_root)
    public = _json(root / "episode/public/episode_manifest.json")
    evaluator = _json(root / "episode/evaluator/episode_manifest.json")
    truth = _json(root / "episode/evaluator/ground_truth.json")
    schedule = _json(root / "episode/environment/pedestrian_schedule.json")
    mission = _yaml(root / "maps/mission_geometry.yaml")
    materialization = _yaml(root / "maps/materialization_contract.yaml")

    if public.get("split") != split:
        raise ValueError("public episode split mismatch")
    if int(public.get("map_index", -1)) != map_index:
        raise ValueError("public map index mismatch")
    if int(public.get("mission_index", -1)) != mission_index:
        raise ValueError("public mission index mismatch")
    if truth.get("control_use_prohibited") is not True:
        raise ValueError("evaluator truth boundary is not fail-closed")
    if materialization.get("evaluator_truth_used") is not False:
        raise ValueError("public map materialization used evaluator truth")

    static_obstacles = []
    for row in mission.get("materialized_static_obstacles", []):
        center = tuple(float(value) for value in row["center_map_m"])
        size = tuple(float(value) for value in row["size_xy_m"])
        static_obstacles.append(
            _rotated_box(center, size, float(row.get("yaw_rad", 0.0)))
        )

    dirt_polygons = []
    for row in truth.get("dirt_patches", []):
        pose = row["pose"]
        dirt_polygons.append(
            _rotated_box(
                (float(pose["x_m"]), float(pose["y_m"])),
                tuple(float(value) for value in row["size_m"]),
                float(pose["yaw_rad"]),
            )
        )
    targets = tuple(
        (
            str(row["object_id"]),
            float(row["pose"]["x_m"]),
            float(row["pose"]["y_m"]),
        )
        for row in truth.get("discrete_cubes", [])
    )
    pedestrians = tuple(
        _pedestrian_pose(row) for row in schedule.get("pedestrians", [])
    )
    pedestrian_radius = max(
        (float(row.get("radius_m", 0.25)) for row in schedule.get("pedestrians", [])),
        default=0.25,
    )
    # This materialized mission remains in source-world coordinates.  Prefer
    # the explicit coordinate contract; the legacy key is source-world only.
    start = public.get("vehicle_start_pose_source_world") or public["vehicle_start_pose_map"]
    navigation_radius = float(materialization["formal_navigation_footprint_radius_m"])
    task = TaskConfig.from_mapping(
        {
            "geofence": mission["outer_polygon"],
            "static_obstacles": static_obstacles,
            "start": {
                "x": start["x_m"],
                "y": start["y_m"],
                "yaw": start["yaw_rad"],
            },
            "grid_resolution": planning_resolution_m,
            "sensing_radius": sensing_radius_m,
            "sensing_fov_rad": sensing_fov_rad,
            "cleaning_width": float(mission["operation_width_m"]),
            "vehicle_radius": navigation_radius,
            # Placement clearance is a scenario-generation parking envelope;
            # it is not the manipulator reach.  The two semantics are
            # intentionally separate in generator v0.3.0.
            "grasp_radius": _cube_grasp_reach_radius(public),
            "min_turn_radius": 0.70,
            "path_sample_spacing": min(0.20, planning_resolution_m / 2.0),
            "observation_threshold": 0.95,
            "ground_clear_threshold": 0.95,
            "discrete_clear_threshold": 0.95,
            "ground_dirt_count": 0,
            "discrete_target_count": 0,
            "pedestrian_count": 0,
            "pedestrian": {"radius": pedestrian_radius, "step_distance": 0.25},
            "max_grasp_attempts": 2,
            "grasp_success_probability": 1.0,
            "max_steps": max_steps,
        }
    )
    seeds = evaluator.get("seeds", {})
    mission_seed = int(seeds.get("dirt", 0))
    if mission_seed <= 0:
        mission_seed = int.from_bytes(
            hashlib.sha256(str(public["episode_id"]).encode()).digest()[:8],
            "big",
        ) & 0x7FFFFFFFFFFFFFFF
    return FormalEpisode(
        split=split,
        map_index=map_index,
        mission_index=mission_index,
        map_id=str(public["map_id"]),
        episode_id=str(public["episode_id"]),
        mission_seed=mission_seed,
        area_m2=float(public["field"]["area_m2"]),
        aspect_ratio=float(public["field"]["aspect_ratio"]),
        config=task,
        layout=TaskLayout(
            ground_dirt_polygons=tuple(dirt_polygons),
            discrete_targets=targets,
            pedestrians=pedestrians,
        ),
        artifact_root=root,
    )


def materialize_episode(
    scenario_config: str | Path,
    motion_profile: str | Path,
    output_root: str | Path,
    *,
    split: str,
    map_index: int,
    mission_index: int,
    map_resolution_m: float = 0.50,
    planning_resolution_m: float = 1.0,
    max_steps: int = 400,
    snapshot_path: str | Path | None = None,
    session_path: str | Path | None = None,
    hidden_receipt_root: str | Path | None = None,
    freeze_receipt_path: str | Path | None = None,
) -> FormalEpisode:
    from sanitation_campus_scenario.generator import generate_episode, load_config
    from sanitation_campus_scenario.hidden_materializer import materialize_hidden_episode
    from sanitation_campus_scenario.io import write_episode
    from sanitation_formal_campus_integration.campus_materializer import (
        materialize_campus_artifacts,
    )

    root = Path(output_root) / f"{split}-map-{map_index:03d}-mission-{mission_index:03d}"
    episode_root = root / "episode"
    maps_root = root / "maps"
    if split == "hidden":
        if snapshot_path is None or session_path is None or hidden_receipt_root is None:
            raise ValueError("hidden RL tasks require snapshot, session and hidden receipt root")
        materialize_hidden_episode(
            scenario_config=Path(scenario_config), snapshot_path=Path(snapshot_path),
            session_path=Path(session_path),
            run_root=Path(hidden_receipt_root),
            output=episode_root, map_index=map_index, mission_index=mission_index,
            freeze_producer="formal_rl_multimap",
        )
    else:
        files = generate_episode(
            load_config(scenario_config), "formal", split, map_index, mission_index,
            include_proxy=False,
        )
        write_episode(episode_root, files)
    materialize_campus_artifacts(
        episode_root / "public/episode_manifest.json",
        episode_root / "public/world.sdf",
        motion_profile,
        maps_root,
        resolution=map_resolution_m,
    )
    return load_materialized_episode(
        root,
        split=split,
        map_index=map_index,
        mission_index=mission_index,
        planning_resolution_m=planning_resolution_m,
        max_steps=max_steps,
    )


def _episode_split_rows(episodes: Iterable[FormalEpisode]) -> list[dict[str, Any]]:
    by_map: dict[str, dict[str, Any]] = {}
    for episode in episodes:
        row = by_map.setdefault(
            episode.map_id,
            {
                "map_id": episode.map_id,
                "map_index": episode.map_index,
                "area_m2": episode.area_m2,
                "aspect_ratio": episode.aspect_ratio,
                "mission_seeds": [],
            },
        )
        if row["map_index"] != episode.map_index:
            raise ValueError("one map ID cannot identify multiple map indices")
        row["mission_seeds"].append(episode.mission_seed)
    return list(by_map.values())


def _train_episode(
    episode: FormalEpisode,
    q_table: dict[str, dict[str, float]],
    *,
    policy_seed: int,
) -> dict[str, Any]:
    policy = QLearningPolicy(
        episode.config,
        epsilon=0.20,
        seed=policy_seed,
    )
    policy.q_table = q_table
    env = ActiveCleaningEnv(
        episode.config,
        task_layout=episode.layout,
    )
    observation = env.reset(seed=episode.mission_seed)
    policy.reset(episode_seed=RoleSeeds.from_master(episode.mission_seed).policy)
    total_reward = 0.0
    total_environment_reward = 0.0
    started = time.perf_counter()
    while True:
        state, label, action = policy.act_with_label(observation, explore=True)
        result = env.step(action)
        shaped_reward = _belief_only_training_reward(observation, result)
        total_environment_reward += result.reward
        total_reward += shaped_reward
        policy.update(
            state,
            label,
            shaped_reward,
            result.observation,
            done=result.terminated or result.truncated,
        )
        observation = result.observation
        if result.terminated or result.truncated:
            break
    row = {
        "episode_id": episode.episode_id,
        "map_id": episode.map_id,
        "mission_seed": episode.mission_seed,
        "terminated": result.terminated,
        "truncated": result.truncated,
        "steps": observation.step_index,
        "observed_ratio": observation.observed_ratio,
        "task_distance": observation.task_distance,
        "belief_only_shaped_reward": total_reward,
        "environment_reward_for_diagnostics": total_environment_reward,
        "runtime_s": round(time.perf_counter() - started, 6),
    }
    print(json.dumps({"event": "formal_train_episode", **row}, sort_keys=True), flush=True)
    return row


def _belief_only_training_reward(
    previous: Any,
    result: StepResult,
) -> float:
    """Shape Q updates from public belief deltas and executed trajectory only."""
    current = result.observation
    observed_progress = max(0.0, current.observed_ratio - previous.observed_ratio)
    cleared_ground = sum(
        before > 0.0 and after == 0.0
        for before, after in zip(
            previous.belief.known_ground_dirt,
            current.belief.known_ground_dirt,
        )
    )
    before_targets = {
        target.target_id: target
        for target in previous.belief.known_targets
    }
    cleared_targets = sum(
        target.cleared
        and target.target_id in before_targets
        and not before_targets[target.target_id].cleared
        for target in current.belief.known_targets
    )
    executed_distance = float(result.info.get("executed_distance", 0.0))
    reward = (
        25.0 * observed_progress
        + 0.20 * cleared_ground
        + 2.0 * cleared_targets
        - 0.0005 * executed_distance
    )
    if result.terminated:
        reward += 10.0
    if result.truncated:
        reward -= 2.0
    if result.info.get("accepted") is not True:
        reward -= 5.0
    elif executed_distance <= 1.0e-9 and cleared_targets == 0:
        reward -= 0.25
    return reward


def train_and_evaluate(
    train: Sequence[FormalEpisode],
    validation: Sequence[FormalEpisode],
    test: Sequence[FormalEpisode],
    *,
    policy_seed: int = 7,
    epochs: int = 1,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any]]:
    if not train or not validation or not test:
        raise ValueError("formal train/validation/test episode sets must be non-empty")
    generalization_contract = _validate_full_map_generalization(
        train, validation, test
    )
    map_sets = [set(item.map_id for item in rows) for rows in (train, validation, test)]
    if any(map_sets[i] & map_sets[j] for i in range(3) for j in range(i + 1, 3)):
        raise ValueError("formal map IDs must be disjoint")
    seed_sets = [set(item.mission_seed for item in rows) for rows in (train, validation, test)]
    if any(seed_sets[i] & seed_sets[j] for i in range(3) for j in range(i + 1, 3)):
        raise ValueError("formal mission seeds must be disjoint")

    q_table: dict[str, dict[str, float]] = {}
    train_rows = []
    for _ in range(epochs):
        for episode in train:
            train_rows.append(
                _train_episode(episode, q_table, policy_seed=policy_seed)
            )

    checkpoint_policy = QLearningPolicy(train[0].config, seed=policy_seed)
    checkpoint_policy.q_table = q_table
    checkpoint = checkpoint_policy.checkpoint()
    checkpoint.update(
        {
            "formal_multi_map": True,
            "product_perception_used_for_training": False,
            "environment_truth_used_only_for_simulator_initialization": True,
            "formal_multimap_contract": generalization_contract,
        }
    )

    baseline_rows = []
    validation_rows = []
    hybrid_rows = []
    for episode in (*validation, *test):
        baseline_started = time.perf_counter()
        baseline = run_episode(
            episode.config,
            seed=episode.mission_seed,
            policy=FullCoveragePolicy(episode.config),
            baseline_distance=None,
            task_layout=episode.layout,
        )
        baseline.update({"episode_id": episode.episode_id, "map_id": episode.map_id})
        baseline["split"] = "validation" if episode in validation else "hidden"
        baseline["runtime_s"] = round(time.perf_counter() - baseline_started, 6)
        print(
            json.dumps(
                {
                    "event": "formal_baseline_episode",
                    "episode_id": episode.episode_id,
                    "success": baseline["success"],
                    "observed_ratio": baseline["observed_ratio"],
                    "task_distance": baseline["task_distance"],
                    "runtime_s": baseline["runtime_s"],
                },
                sort_keys=True,
            ),
            flush=True,
        )
        baseline_rows.append(baseline)
        policy = QLearningPolicy(episode.config, epsilon=0.0, seed=policy_seed)
        policy.q_table = q_table
        active_started = time.perf_counter()
        active = run_episode(
            episode.config,
            seed=episode.mission_seed,
            policy=policy,
            baseline_distance=(
                float(baseline["task_distance"]) if baseline["success"] else None
            ),
            task_layout=episode.layout,
        )
        active.update(
            {
                "episode_id": episode.episode_id,
                "map_id": episode.map_id,
                "split": "validation" if episode in validation else "hidden",
                "baseline_valid": bool(baseline["success"]),
                "baseline_distance": float(baseline["task_distance"]),
                "path_ratio_to_full_coverage": (
                    float(active["task_distance"]) / float(baseline["task_distance"])
                    if float(baseline["task_distance"]) > 0.0
                    else None
                ),
                "runtime_s": round(time.perf_counter() - active_started, 6),
            }
        )
        active["formal_success"] = bool(
            active["success"]
            and baseline["success"]
            and active["path_ratio_to_full_coverage"] is not None
            and active["path_ratio_to_full_coverage"] <= 1.0 + 1.0e-9
        )
        validation_rows.append(active)
        print(
            json.dumps(
                {
                    "event": "formal_q_episode",
                    "episode_id": episode.episode_id,
                    "formal_success": active["formal_success"],
                    "observed_ratio": active["observed_ratio"],
                    "ground_clear_ratio": active["ground_clear_ratio"],
                    "discrete_clear_ratio": active["discrete_clear_ratio"],
                    "task_distance": active["task_distance"],
                    "path_ratio_to_full_coverage": active[
                        "path_ratio_to_full_coverage"
                    ],
                    "runtime_s": active["runtime_s"],
                },
                sort_keys=True,
            ),
            flush=True,
        )

        hybrid_policy = CoverageBackstoppedQLearningPolicy(
            episode.config,
            q_table=q_table,
            seed=policy_seed,
        )
        hybrid_started = time.perf_counter()
        hybrid = run_episode(
            episode.config,
            seed=episode.mission_seed,
            policy=hybrid_policy,
            baseline_distance=(
                float(baseline["task_distance"]) if baseline["success"] else None
            ),
            task_layout=episode.layout,
        )
        hybrid.update(
            {
                "episode_id": episode.episode_id,
                "map_id": episode.map_id,
                "split": "validation" if episode in validation else "hidden",
                "baseline_valid": bool(baseline["success"]),
                "baseline_distance": float(baseline["task_distance"]),
                "path_ratio_to_full_coverage": (
                    float(hybrid["task_distance"]) / float(baseline["task_distance"])
                    if float(baseline["task_distance"]) > 0.0
                    else None
                ),
                "runtime_s": round(time.perf_counter() - hybrid_started, 6),
                "systematic_coverage_backstop_activated": bool(
                    hybrid_policy._coverage_activated
                ),
            }
        )
        hybrid["formal_success"] = bool(
            hybrid["success"]
            and baseline["success"]
            and hybrid["path_ratio_to_full_coverage"] is not None
            and hybrid["path_ratio_to_full_coverage"] <= 1.0 + 1.0e-9
        )
        hybrid_rows.append(hybrid)
        print(
            json.dumps(
                {
                    "event": "formal_q_with_coverage_backstop_episode",
                    "episode_id": episode.episode_id,
                    "formal_success": hybrid["formal_success"],
                    "observed_ratio": hybrid["observed_ratio"],
                    "ground_clear_ratio": hybrid["ground_clear_ratio"],
                    "discrete_clear_ratio": hybrid["discrete_clear_ratio"],
                    "task_distance": hybrid["task_distance"],
                    "path_ratio_to_full_coverage": hybrid[
                        "path_ratio_to_full_coverage"
                    ],
                    "systematic_coverage_backstop_activated": hybrid[
                        "systematic_coverage_backstop_activated"
                    ],
                    "runtime_s": hybrid["runtime_s"],
                },
                sort_keys=True,
            ),
            flush=True,
        )

    training_report = {
        "schema_version": 1,
        "policy": "q_learning",
        "truth_access_used": False,
        "truth_used_for_control": False,
        "evaluator_truth_used_for_metrics": True,
        "product_perception_used": False,
        "formal_map_used": True,
        "planning_resolution_is_downsampled_from_product_map": True,
        "map_splits": {
            "train": _episode_split_rows(train),
            "validation": _episode_split_rows(validation),
            "test": _episode_split_rows(test),
        },
        "formal_multimap_contract": generalization_contract,
        "epochs": epochs,
        "episodes": train_rows,
        "q_state_count": len(q_table),
        "reward_contract": "public_belief_delta_plus_executed_trajectory_v1",
    }
    baseline_report = {
        "schema_version": 1,
        "mode": "full_coverage",
        "formal_map_used": True,
        "truth_used_for_control": False,
        "return_distance_included": False,
        "time_energy_ignored": True,
        "episodes": baseline_rows,
    }
    validation_report = {
        "schema_version": 1,
        "status": "research_only_not_product_acceptance",
        "formal_map_used": True,
        "product_perception_used": False,
        "truth_used_for_control": False,
        "return_distance_included": False,
        "time_energy_ignored": True,
        "policy_output": "global_reference_trajectory",
        "episodes": validation_rows,
        "pure_q_episodes": validation_rows,
        "hybrid_policy": "q_learning_with_systematic_coverage_backstop",
        "hybrid_episodes": hybrid_rows,
        "gate_policy": "q_learning_with_systematic_coverage_backstop",
        "formal_multimap_contract": generalization_contract,
        "hidden_gate_passed": bool(
            [row for row in hybrid_rows if row["split"] == "hidden"]
        )
        and generalization_contract["full_map_coverage"] is True
        and all(
            row["formal_success"]
            for row in hybrid_rows
            if row["split"] == "hidden"
        ),
    }
    return checkpoint, training_report, baseline_report, validation_report


def _selection(value: str) -> list[tuple[int, int]]:
    result = []
    for item in value.split(","):
        map_index, mission_index = item.split(":", 1)
        result.append((int(map_index), int(mission_index)))
    if not result:
        raise argparse.ArgumentTypeError("selection cannot be empty")
    return result


def _policy_seeds(value: str) -> tuple[int, ...]:
    try:
        seeds = tuple(int(item) for item in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("policy seeds must be comma-separated integers") from exc
    if not seeds or len(set(seeds)) != len(seeds):
        raise argparse.ArgumentTypeError("policy seeds must be non-empty and unique")
    return seeds


def _write(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-config", required=True, type=Path)
    parser.add_argument("--motion-profile", required=True, type=Path)
    parser.add_argument("--work-root", required=True, type=Path)
    parser.add_argument("--evidence-root", required=True, type=Path)
    parser.add_argument("--snapshot", type=Path)
    parser.add_argument("--session", type=Path)
    parser.add_argument("--hidden-receipt-root", type=Path)
    parser.add_argument(
        "--train",
        type=_selection,
        help="must equal the frozen 32-map train manifest when supplied",
    )
    parser.add_argument(
        "--validation",
        type=_selection,
        help="must equal the frozen 8-map validation manifest when supplied",
    )
    parser.add_argument(
        "--test",
        type=_selection,
        help="must equal the frozen 12-map hidden manifest when supplied",
    )
    parser.add_argument("--map-resolution", type=float, default=0.50)
    parser.add_argument("--planning-resolution", type=float, default=1.0)
    parser.add_argument("--epochs", type=int, default=1)
    parser.add_argument("--max-steps", type=int, default=400)
    parser.add_argument("--policy-seed", type=int, default=7)
    parser.add_argument(
        "--policy-seeds",
        type=_policy_seeds,
        help="formal budget mode: run every frozen policy seed serially",
    )
    parser.add_argument(
        "--budget-contract",
        type=Path,
        help="required for a formal multi-map budget claim; smoke runs remain research-only",
    )
    args = parser.parse_args(argv)
    budget = load_formal_rl_budget(args.budget_contract) if args.budget_contract else None
    if budget is not None:
        if args.epochs != 1:
            raise ValueError("multi-map frozen task budget requires exactly one pass over every task")
        if args.max_steps != budget.max_steps_per_episode:
            raise ValueError("max_steps must equal the formal per-episode truncation guard")
        if args.policy_seeds is None:
            raise ValueError("formal budget mode requires all frozen --policy-seeds")
        if args.policy_seeds != budget.policy_seeds:
            raise ValueError("--policy-seeds must equal the frozen formal policy seed list")
    split_manifest = load_full_formal_split_manifest(args.scenario_config, args.budget_contract)
    selections = {
        "train": _require_full_map_selection("train", args.train, split_manifest),
        "validation": _require_full_map_selection(
            "validation", args.validation, split_manifest
        ),
        "hidden": _require_full_map_selection("hidden", args.test, split_manifest),
    }
    if any(value is None for value in (args.snapshot, args.session, args.hidden_receipt_root)):
        raise ValueError("formal multi-map training requires --snapshot, --session and --hidden-receipt-root")
    from sanitation_campus_scenario.hidden_materializer import require_canonical_formal_inputs
    require_canonical_formal_inputs(
        snapshot_path=args.snapshot, session_path=args.session, scenario_config=args.scenario_config,
    )

    def prepare(
        split: str, rows: Sequence[tuple[int, int]], freeze_receipt_path: Path | None = None,
    ) -> list[FormalEpisode]:
        return [
            materialize_episode(
                args.scenario_config,
                args.motion_profile,
                args.work_root,
                split=split,
                map_index=map_index,
                mission_index=mission_index,
                map_resolution_m=args.map_resolution,
                planning_resolution_m=args.planning_resolution,
                max_steps=args.max_steps,
                snapshot_path=args.snapshot,
                session_path=args.session,
                hidden_receipt_root=args.hidden_receipt_root,
                freeze_receipt_path=freeze_receipt_path,
            )
            for map_index, mission_index in rows
        ]

    prepared = {
        "train": prepare("train", selections["train"]),
        "validation": prepare("val", selections["validation"]),
    }
    policy_seeds = args.policy_seeds or (args.policy_seed,)
    seed_runs = []
    primary: tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any]] | None = None
    # All algorithm/configuration choices and the complete seed list are
    # frozen before any hidden rollout.  Hidden metrics are never used to
    # choose a checkpoint; the first frozen seed is the downstream artifact.
    configuration_freeze = {
        "frozen_before_hidden": True,
        "selection_source": "validation_only_before_hidden",
        "policy_seeds": list(policy_seeds),
    }
    from sanitation_campus_scenario.hidden_materializer import (
        commit_hidden_configuration_freeze, verify_hidden_consumption_records,
    )
    freeze_receipt = commit_hidden_configuration_freeze(
        run_root=args.hidden_receipt_root,
        snapshot_path=args.snapshot, session_path=args.session,
        scenario_config=args.scenario_config, producer="formal_rl_multimap",
        frozen_configuration=configuration_freeze,
    )
    # Keep the final map/mission files absent until the immutable algorithm,
    # seed list and checkpoint-selection rule have been recorded.  A formal
    # run does not adapt this configuration from hidden metrics.
    prepared["hidden"] = prepare("hidden", selections["hidden"], freeze_receipt)
    hidden_consumption = verify_hidden_consumption_records(
        run_root=args.hidden_receipt_root, snapshot_path=args.snapshot, session_path=args.session,
        scenario_config=args.scenario_config,
        records=[{
            "producer": "formal_hidden_episode",
            "request": {"profile": "formal", "split": "hidden", "map_index": row[0], "mission_index": row[1]},
            "output": args.work_root / f"hidden-map-{row[0]:03d}-mission-{row[1]:03d}" / "episode",
        } for row in selections["hidden"]],
    )
    for policy_seed in policy_seeds:
        run = train_and_evaluate(
            prepared["train"], prepared["validation"], prepared["hidden"],
            policy_seed=policy_seed, epochs=args.epochs,
        )
        if primary is None:
            primary = run
        checkpoint_run, training_run, _baseline_run, validation_run = run
        seed_runs.append({
            "policy_seed": policy_seed,
            "training_rollout_count": len(training_run.get("episodes", [])),
            "validation_episode_count": sum(
                row.get("split") == "validation"
                for row in validation_run.get("hybrid_episodes", [])
            ),
            "hidden_episode_count": sum(
                row.get("split") == "hidden"
                for row in validation_run.get("hybrid_episodes", [])
            ),
            "hidden_gate_passed": validation_run.get("hidden_gate_passed") is True,
            "checkpoint_q_state_count": len(checkpoint_run.get("q_table", {})),
        })
    assert primary is not None
    checkpoint, training, baseline, validation = primary
    evidence = args.evidence_root / "formal_planning"
    split_manifest_report = split_manifest.report()
    for report in (checkpoint, training, baseline, validation):
        report["frozen_split_manifest"] = split_manifest_report
        report["formal_budget_execution"] = {
            "contract": budget.report() if budget is not None else None,
            "formal_budget_claim": budget is not None,
            "task_counts": {name: len(selections[name]) for name in ("train", "validation", "hidden")},
            "training_rollout_count": len(selections["train"]) * args.epochs,
            "max_steps_per_episode": args.max_steps,
            "max_steps_is_episode_truncation_guard_not_task_or_episode_budget": True,
            "policy_seed": args.policy_seed,
            "policy_seeds": list(policy_seeds),
            "policy_seed_runs": seed_runs,
            "configuration_freeze": configuration_freeze,
            "configuration_freeze_receipt_sha256": hashlib.sha256(
                freeze_receipt.read_bytes()
            ).hexdigest(),
            "hidden_consumption": hidden_consumption,
        }
    _write(evidence / "q_policy.json", checkpoint)
    _write(evidence / "training_report.json", training)
    _write(evidence / "baseline_report.json", baseline)
    _write(evidence / "validation_report.json", validation)
    print(evidence)
    return 0 if validation["hidden_gate_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
