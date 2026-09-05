#!/usr/bin/env python3
"""Materialize disjoint formal campuses and evaluate belief-only planners.

This is a research-harness evaluation.  It intentionally does not claim that
the generated belief state came from the product perception stack.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Iterable, Sequence


ROOT = Path(__file__).resolve().parents[1]
ACTIVE_PACKAGE = ROOT / "starter_ws/src/sanitation_active_cleaning"
SCENARIO_PACKAGE = ROOT / "starter_ws/src/sanitation_campus_scenario"
for package in (ACTIVE_PACKAGE, SCENARIO_PACKAGE):
    sys.path.insert(0, str(package))

from sanitation_active_cleaning.environment import (  # noqa: E402
    ActiveCleaningEnv,
    create_evaluation_token,
)
from sanitation_active_cleaning.evaluation import _episode_metrics  # noqa: E402
from sanitation_active_cleaning.models import (  # noqa: E402
    RoleSeeds,
    TaskConfig,
    TaskLayout,
)
from sanitation_active_cleaning.policies import (  # noqa: E402
    FullCoveragePolicy,
    SensingGreedyPolicy,
    TrajectoryPolicy,
)
from sanitation_active_cleaning.rl import QLearningPolicy  # noqa: E402
from sanitation_campus_scenario.generator import (  # noqa: E402
    generate_episode,
    load_config,
)


SPLITS = ("train", "val", "hidden")


@dataclass(frozen=True)
class MaterializedTask:
    split: str
    directory: Path
    public_manifest: dict[str, Any]
    evaluator_manifest: dict[str, Any]
    truth: dict[str, Any]
    config: TaskConfig
    layout: TaskLayout
    file_sha256: dict[str, str]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _rotated_rectangle(
    x: float, y: float, width: float, height: float, yaw: float
) -> tuple[tuple[float, float], ...]:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    result = []
    for local_x, local_y in (
        (-width / 2.0, -height / 2.0),
        (width / 2.0, -height / 2.0),
        (width / 2.0, height / 2.0),
        (-width / 2.0, height / 2.0),
    ):
        result.append(
            (
                x + local_x * cosine - local_y * sine,
                y + local_x * sine + local_y * cosine,
            )
        )
    return tuple(result)


def _circle_polygon(
    x: float, y: float, radius: float, *, vertices: int = 12
) -> tuple[tuple[float, float], ...]:
    return tuple(
        (
            x + radius * math.cos(2.0 * math.pi * index / vertices),
            y + radius * math.sin(2.0 * math.pi * index / vertices),
        )
        for index in range(vertices)
    )


def _asset_polygon(asset: dict[str, Any]) -> tuple[tuple[float, float], ...]:
    pose = asset["pose"]
    x = float(pose["x_m"])
    y = float(pose["y_m"])
    yaw = float(pose["yaw_rad"])
    size = tuple(float(value) for value in asset["size_m"])
    if asset["kind"] == "pole":
        return _circle_polygon(x, y, size[0] / 2.0)
    if asset["kind"] == "tree":
        # The generator SDF uses a 0.28 m trunk collision.  The 1.2 m canopy
        # is visual-only and must not inflate the navigation obstacle.
        return _circle_polygon(x, y, 0.28)
    return _rotated_rectangle(x, y, size[0], size[1], yaw)


def _task_from_generated(
    split: str,
    directory: Path,
    *,
    grid_resolution: float,
    sensing_radius: float,
    sensing_fov_rad: float,
    max_steps: int,
) -> MaterializedTask:
    public_path = directory / "public/episode_manifest.json"
    evaluator_path = directory / "evaluator/episode_manifest.json"
    truth_path = directory / "evaluator/ground_truth.json"
    public = json.loads(public_path.read_text(encoding="utf-8"))
    evaluator = json.loads(evaluator_path.read_text(encoding="utf-8"))
    truth = json.loads(truth_path.read_text(encoding="utf-8"))
    if public.get("split") != split or evaluator.get("split") != split:
        raise ValueError(f"materialized split mismatch: expected {split}")
    if int(public["counts"]["pedestrians"]) != 8:
        raise ValueError(f"formal sample {split} does not contain 8 pedestrians")

    start = public["vehicle_start_pose_map"]
    config = TaskConfig.from_mapping(
        {
            "geofence": public["field"]["geofence_polygon_m"],
            "static_obstacles": [
                _asset_polygon(asset) for asset in truth["static_assets"]
            ],
            "start": {
                "x": start["x_m"],
                "y": start["y_m"],
                "yaw": start["yaw_rad"],
            },
            "grid_resolution": grid_resolution,
            "sensing_radius": sensing_radius,
            "sensing_fov_rad": sensing_fov_rad,
            "cleaning_width": 1.32,
            "vehicle_radius": math.hypot(0.620, 0.695),
            "grasp_radius": 1.05,
            "min_turn_radius": 0.75,
            "path_sample_spacing": min(0.40, grid_resolution * 0.5),
            "observation_threshold": 0.95,
            "ground_clear_threshold": 0.95,
            "discrete_clear_threshold": 0.95,
            "ground_dirt_count": len(truth["dirt_patches"]),
            "ground_dirt_radius_range": [math.sqrt(1.0 / math.pi)] * 2,
            "discrete_target_count": len(truth["discrete_cubes"]),
            "pedestrian_count": len(truth["pedestrians"]),
            "pedestrian": {"radius": 0.25, "step_distance": 0.20},
            "max_grasp_attempts": 2,
            "grasp_success_probability": 1.0,
            "max_steps": max_steps,
        }
    )
    layout = TaskLayout(
        ground_dirt_polygons=tuple(
            _rotated_rectangle(
                float(item["pose"]["x_m"]),
                float(item["pose"]["y_m"]),
                float(item["size_m"][0]),
                float(item["size_m"][1]),
                float(item["pose"]["yaw_rad"]),
            )
            for item in truth["dirt_patches"]
        ),
        discrete_targets=tuple(
            (
                str(item["object_id"]),
                float(item["pose"]["x_m"]),
                float(item["pose"]["y_m"]),
            )
            for item in truth["discrete_cubes"]
        ),
        pedestrians=tuple(_initial_pedestrian(item) for item in truth["pedestrians"]),
    )
    files = sorted(path for path in directory.rglob("*") if path.is_file())
    return MaterializedTask(
        split=split,
        directory=directory,
        public_manifest=public,
        evaluator_manifest=evaluator,
        truth=truth,
        config=config,
        layout=layout,
        file_sha256={str(path.relative_to(directory)): _sha256(path) for path in files},
    )


def _initial_pedestrian(item: dict[str, Any]) -> tuple[float, float, float]:
    first, second = item["waypoints"][:2]
    x = float(first[1])
    y = float(first[2])
    yaw = math.atan2(float(second[2]) - y, float(second[1]) - x)
    return x, y, yaw


def materialize_tasks(
    scenario_config: Path,
    output_root: Path,
    *,
    grid_resolution: float,
    sensing_radius: float,
    sensing_fov_rad: float,
    max_steps: int,
) -> list[MaterializedTask]:
    scenario = load_config(scenario_config)
    tasks = []
    for split in SPLITS:
        directory = output_root / "samples" / split / "map-000" / "mission-000"
        generated = generate_episode(scenario, "formal", split, 0, 0)
        for relative, content in generated.items():
            destination = directory / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(content, encoding="utf-8")
        tasks.append(
            _task_from_generated(
                split,
                directory,
                grid_resolution=grid_resolution,
                sensing_radius=sensing_radius,
                sensing_fov_rad=sensing_fov_rad,
                max_steps=max_steps,
            )
        )
    _assert_disjoint(tasks)
    return tasks


def _assert_disjoint(tasks: Sequence[MaterializedTask]) -> None:
    if tuple(task.split for task in tasks) != SPLITS:
        raise ValueError("train/val/hidden tasks are incomplete or out of order")
    map_ids = {task.public_manifest["map_id"] for task in tasks}
    if len(map_ids) != len(tasks):
        raise ValueError("map IDs overlap across splits")
    seed_rows = [set(int(value) for value in task.evaluator_manifest["seeds"].values()) for task in tasks]
    if any(seed_rows[left] & seed_rows[right] for left in range(3) for right in range(left + 1, 3)):
        raise ValueError("role seeds overlap across splits")


def _run_policy(
    task: MaterializedTask,
    policy: TrajectoryPolicy,
    *,
    seed: int,
    baseline_distance: float | None,
) -> dict[str, Any]:
    token = create_evaluation_token()
    env = ActiveCleaningEnv(
        task.config,
        evaluation_token=token,
        task_layout=task.layout,
        # Preserve useful failure metrics even when the baseline itself is
        # blocked; distance remains a separate fail-closed gate below.
        max_task_distance=None,
    )
    observation = env.reset(seed=seed)
    policy.reset(episode_seed=RoleSeeds.from_master(seed).policy)
    started = time.perf_counter()
    while True:
        truth = env.evaluation_snapshot(token)
        if truth.terminated or truth.truncated:
            break
        observation = env.step(policy.act(observation)).observation
    runtime = time.perf_counter() - started
    truth = env.evaluation_snapshot(token)
    metrics = _episode_metrics(env, truth, baseline_distance=baseline_distance)
    metrics.update(
        {
            "split": task.split,
            "map_id": task.public_manifest["map_id"],
            "episode_id": task.public_manifest["episode_id"],
            "policy": policy.name,
            "runtime_s": round(runtime, 6),
            "pedestrian_count": len(task.layout.pedestrians),
            "product_perception_used": False,
            "belief_source": "evaluation_harness_geometric_sensing_surrogate",
        }
    )
    print(
        json.dumps(
            {
                "event": "episode_complete",
                "split": task.split,
                "policy": policy.name,
                "runtime_s": metrics["runtime_s"],
                "observed_ratio": metrics["observed_ratio"],
                "ground_clear_ratio": metrics["ground_clear_ratio"],
                "discrete_clear_ratio": metrics["discrete_clear_ratio"],
            },
            sort_keys=True,
        ),
        flush=True,
    )
    return metrics


def _train_q_policy(
    task: MaterializedTask,
    *,
    seed: int,
    epochs: int,
) -> tuple[QLearningPolicy, dict[str, Any]]:
    policy = QLearningPolicy(task.config, epsilon=0.20, seed=73012026)
    started = time.perf_counter()
    episode_rows = []
    for epoch in range(epochs):
        episode_seed = seed + epoch
        env = ActiveCleaningEnv(task.config, task_layout=task.layout)
        observation = env.reset(seed=episode_seed)
        policy.reset(episode_seed=RoleSeeds.from_master(episode_seed).policy)
        reward = 0.0
        terminated = False
        truncated = False
        for _ in range(task.config.max_steps):
            state, label, action = policy.act_with_label(observation, explore=True)
            result = env.step(action)
            reward += result.reward
            policy.update(
                state,
                label,
                result.reward,
                result.observation,
                done=result.terminated or result.truncated,
            )
            observation = result.observation
            terminated = result.terminated
            truncated = result.truncated
            if terminated or truncated:
                break
        episode_rows.append(
            {
                "epoch": epoch,
                "seed": episode_seed,
                "terminated": terminated,
                "truncated": truncated,
                "steps": observation.step_index,
                "observed_ratio": observation.observed_ratio,
                "task_distance": observation.task_distance,
                "reward": reward,
            }
        )
        print(
            json.dumps(
                {
                    "event": "q_training_epoch_complete",
                    **episode_rows[-1],
                },
                sort_keys=True,
            ),
            flush=True,
        )
    return policy, {
        "epochs": epochs,
        "runtime_s": round(time.perf_counter() - started, 6),
        "q_state_count": len(policy.q_table),
        "truth_access_used": False,
        "episodes": episode_rows,
    }


def _policy_summary(rows: Iterable[dict[str, Any]]) -> dict[str, Any]:
    selected = list(rows)
    count = len(selected)
    return {
        "episodes": count,
        "success_rate": (
            sum(bool(row["formal_success"]) for row in selected) / count if count else 0.0
        ),
        "minimum_observed_ratio": min((row["observed_ratio"] for row in selected), default=0.0),
        "minimum_ground_clear_ratio": min((row["ground_clear_ratio"] for row in selected), default=0.0),
        "minimum_discrete_clear_ratio": min((row["discrete_clear_ratio"] for row in selected), default=0.0),
        "mean_task_distance": (
            sum(row["task_distance"] for row in selected) / count if count else 0.0
        ),
        "total_runtime_s": sum(row["runtime_s"] for row in selected),
    }


def evaluate(
    tasks: Sequence[MaterializedTask],
    *,
    q_epochs: int,
    output_root: Path,
) -> dict[str, Any]:
    seeds = {
        task.split: int(task.evaluator_manifest["seeds"]["sensor"])
        for task in tasks
    }
    q_policy, training = _train_q_policy(
        tasks[0], seed=seeds["train"], epochs=q_epochs
    )
    checkpoint = output_root / "q_policy.json"
    q_policy.save(checkpoint)
    episodes = []
    for task in tasks:
        seed = seeds[task.split]
        baseline = _run_policy(
            task,
            FullCoveragePolicy(task.config),
            seed=seed,
            baseline_distance=None,
        )
        baseline_valid = bool(baseline["success"])
        baseline["baseline_valid"] = baseline_valid
        baseline["formal_success"] = baseline_valid
        episodes.append(baseline)
        baseline_distance = float(baseline["task_distance"])
        for policy in (SensingGreedyPolicy(task.config), q_policy):
            row = _run_policy(
                task,
                policy,
                seed=seed,
                baseline_distance=baseline_distance,
            )
            row["baseline_valid"] = baseline_valid
            row["formal_success"] = bool(row["success"] and baseline_valid)
            episodes.append(row)

    summaries = {
        policy: _policy_summary(row for row in episodes if row["policy"] == policy)
        for policy in ("full_coverage", "sensing_greedy", "q_learning")
    }
    research_gates = {
        "split_maps_and_role_seeds_disjoint": True,
        "one_map_one_mission_each_split_materialized": len(tasks) == 3,
        "eight_random_pedestrians_each_episode": all(
            len(task.layout.pedestrians) == 8 for task in tasks
        ),
        "sensing_greedy_all_splits_success": summaries["sensing_greedy"]["success_rate"] == 1.0,
        "q_learning_all_splits_success": summaries["q_learning"]["success_rate"] == 1.0,
    }
    product_gates = {
        "product_perception_used": False,
        "gazebo_sensor_streams_used": False,
        "nav2_execution_used": False,
    }
    blockers = [
        key for key, passed in research_gates.items() if not passed
    ] + [
        f"all_policy_episodes_reached_{tasks[0].config.max_steps}_step_compute_cap",
        "product_perception_not_exercised",
        "gazebo_and_nav2_not_exercised",
        "grid_cell_dirt_metric_is_a_discretized_rectangle_approximation",
        "pedestrian_schedule_reduced_to_stepwise_current_position",
    ]
    return {
        "schema_version": 1,
        "report_id": "formal_active_cleaning_disjoint_split_research_eval_v1",
        "status": "BLOCKED_RESEARCH_ONLY",
        "research_policy_gates_passed": all(research_gates.values()),
        "formal_product_ready": False,
        "research_gates": research_gates,
        "product_gates": product_gates,
        "blockers": blockers,
        "claim_boundary": (
            "Pure-Python belief-only planning surrogate over generated formal maps; "
            "not product perception, Gazebo dynamics, Nav2 execution, or competition acceptance."
        ),
        "thresholds": {
            "observed_ratio": 0.95,
            "ground_clear_ratio": 0.95,
            "discrete_clear_ratio": 0.95,
            "distance_upper_bound": "same_episode_full_coverage_distance",
            "return_distance_included": False,
            "time_energy_scored": False,
        },
        "harness_assumptions": {
            "grid_resolution_m": tasks[0].config.grid_resolution,
            "sensing_radius_m": tasks[0].config.sensing_radius,
            "sensing_fov_rad": tasks[0].config.sensing_fov_rad,
            "cleaning_width_m": tasks[0].config.cleaning_width,
            "vehicle_radius_m": tasks[0].config.vehicle_radius,
            "grasp_radius_m": tasks[0].config.grasp_radius,
            "minimum_turn_radius_m": tasks[0].config.min_turn_radius,
            "max_steps_per_episode": tasks[0].config.max_steps,
            "pedestrian_step_distance_m_per_policy_step": tasks[0].config.pedestrian.step_distance,
            "runtime_definition": "policy_loop_wall_clock_excludes_materialization_and_environment_construction",
        },
        "map_splits": {
            task.split: {
                "map_id": task.public_manifest["map_id"],
                "episode_id": task.public_manifest["episode_id"],
                "field": task.public_manifest["field"],
                "counts": task.public_manifest["counts"],
                "role_seeds": task.evaluator_manifest["seeds"],
                "sample_directory": str(task.directory),
                "file_sha256": task.file_sha256,
            }
            for task in tasks
        },
        "q_learning_training": training,
        "q_checkpoint": {
            "path": str(checkpoint),
            "sha256": _sha256(checkpoint),
            "truth_access_used": False,
        },
        "episodes": episodes,
        "summaries": summaries,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario-config",
        type=Path,
        default=ROOT / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml",
    )
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--grid-resolution", type=float, default=1.0)
    parser.add_argument("--sensing-radius", type=float, default=8.0)
    parser.add_argument("--sensing-fov-rad", type=float, default=math.radians(86.0))
    parser.add_argument("--max-steps", type=int, default=2500)
    parser.add_argument("--q-epochs", type=int, default=1)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if (
        args.grid_resolution <= 0.0
        or args.sensing_radius <= 0.0
        or not 0.0 < args.sensing_fov_rad <= 2.0 * math.pi
        or args.max_steps <= 0
        or args.q_epochs <= 0
    ):
        raise SystemExit("all numeric runtime arguments must be positive and FOV <= 2*pi")
    args.output_root.mkdir(parents=True, exist_ok=True)
    tasks = materialize_tasks(
        args.scenario_config,
        args.output_root,
        grid_resolution=args.grid_resolution,
        sensing_radius=args.sensing_radius,
        sensing_fov_rad=args.sensing_fov_rad,
        max_steps=args.max_steps,
    )
    report = evaluate(tasks, q_epochs=args.q_epochs, output_root=args.output_root)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({"report": str(args.report), "status": report["status"]}))
    # Product readiness is deliberately fail-closed; research policy failures
    # also remain visible in the JSON rather than being turned into a claim.
    return 0 if report["research_policy_gates_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
