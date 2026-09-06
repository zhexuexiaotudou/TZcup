"""Executable fixed-map Stage-A RL budget runner.

This runner is intentionally separate from multi-map training.  It materializes
the 10,000/500/1,000 frozen task indices on one stable formal layout, trains
five policy seeds serially, freezes each configured policy before the hidden
rollouts, and emits provenance/count evidence.  It is expensive by design and
must never be replaced by a smoke subset in final acceptance.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time
from typing import Any, Sequence

from .environment import ActiveCleaningEnv
from .evaluation import run_episode
from .formal_budget import all_stage_a_task_indices, load_formal_rl_budget
from .formal_training import (
    FormalEpisode,
    _belief_only_training_reward,
    _cube_grasp_reach_radius,
    _pedestrian_pose,
    _rotated_box,
)
from .models import RoleSeeds, TaskConfig, TaskLayout
from .policies import FullCoveragePolicy
from .rl import CoverageBackstoppedQLearningPolicy, QLearningPolicy


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON mapping: {path}")
    return value


def _yaml(path: Path) -> dict[str, Any]:
    import yaml

    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected YAML mapping: {path}")
    return value


def materialize_stage_a_episode(
    scenario_config: str | Path,
    motion_profile: str | Path,
    output_root: str | Path,
    *,
    phase: str,
    task_index: int,
    map_resolution_m: float,
    planning_resolution_m: float,
    max_steps: int,
    snapshot_path: str | Path | None = None,
    session_path: str | Path | None = None,
    hidden_receipt_root: str | Path | None = None,
    freeze_receipt_path: str | Path | None = None,
) -> FormalEpisode:
    from sanitation_campus_scenario.generator import generate_stage_a_episode, load_config
    from sanitation_campus_scenario.hidden_materializer import materialize_hidden_stage_a_episode
    from sanitation_campus_scenario.io import write_episode
    from sanitation_formal_campus_integration.campus_materializer import materialize_campus_artifacts

    root = Path(output_root) / f"stage-a-{phase}-task-{task_index:05d}"
    episode_root = root / "episode"
    maps_root = root / "maps"
    if phase == "hidden":
        if snapshot_path is None or session_path is None or hidden_receipt_root is None:
            raise ValueError("hidden Stage-A tasks require snapshot, session and hidden receipt root")
        materialize_hidden_stage_a_episode(
            scenario_config=Path(scenario_config), snapshot_path=Path(snapshot_path),
            session_path=Path(session_path),
            run_root=Path(hidden_receipt_root),
            output=episode_root, task_index=task_index,
            freeze_producer="formal_rl_stage_a",
        )
    else:
        files = generate_stage_a_episode(load_config(scenario_config), "formal", phase, task_index)
        write_episode(episode_root, files)
    materialize_campus_artifacts(
        episode_root / "public/episode_manifest.json",
        episode_root / "public/world.sdf",
        motion_profile,
        maps_root,
        resolution=map_resolution_m,
    )
    public = _json(episode_root / "public/episode_manifest.json")
    evaluator = _json(episode_root / "evaluator/episode_manifest.json")
    truth = _json(episode_root / "evaluator/ground_truth.json")
    schedule = _json(episode_root / "environment/pedestrian_schedule.json")
    mission = _yaml(maps_root / "mission_geometry.yaml")
    materialization = _yaml(maps_root / "materialization_contract.yaml")
    if public.get("split") != f"stage_a_{phase}" or truth.get("control_use_prohibited") is not True:
        raise ValueError("Stage-A public/truth boundary is invalid")
    static_obstacles = [
        _rotated_box(
            tuple(float(value) for value in row["center_map_m"]),
            tuple(float(value) for value in row["size_xy_m"]),
            float(row.get("yaw_rad", 0.0)),
        )
        for row in mission.get("materialized_static_obstacles", [])
    ]
    dirt = [
        _rotated_box(
            (float(row["pose"]["x_m"]), float(row["pose"]["y_m"])),
            tuple(float(value) for value in row["size_m"]),
            float(row["pose"]["yaw_rad"]),
        )
        for row in truth.get("dirt_patches", [])
    ]
    pedestrians = tuple(_pedestrian_pose(row) for row in schedule.get("pedestrians", []))
    pedestrian_radius = max((float(row.get("radius_m", 0.25)) for row in schedule.get("pedestrians", [])), default=0.25)
    start = public["vehicle_start_pose_map"]
    config = TaskConfig.from_mapping({
        "geofence": mission["outer_polygon"],
        "static_obstacles": static_obstacles,
        "start": {"x": start["x_m"], "y": start["y_m"], "yaw": start["yaw_rad"]},
        "grid_resolution": planning_resolution_m,
        "sensing_radius": 10.0,
        "sensing_fov_rad": 1.5184364492350666,
        "cleaning_width": float(mission["operation_width_m"]),
        "vehicle_radius": float(materialization["formal_navigation_footprint_radius_m"]),
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
    })
    seeds = evaluator["seeds"]
    return FormalEpisode(
        split=f"stage_a_{phase}", map_index=0, mission_index=task_index,
        map_id=str(public["map_id"]), episode_id=str(public["episode_id"]),
        mission_seed=int(seeds["dirt"]), area_m2=float(public["field"]["area_m2"]),
        aspect_ratio=float(public["field"]["aspect_ratio"]), config=config,
        layout=TaskLayout(
            ground_dirt_polygons=tuple(dirt),
            discrete_targets=tuple((str(row["object_id"]), float(row["pose"]["x_m"]), float(row["pose"]["y_m"])) for row in truth.get("discrete_cubes", [])),
            pedestrians=pedestrians,
        ), artifact_root=root,
    )


def _train(episodes: Sequence[FormalEpisode], policy_seed: int) -> dict[str, dict[str, float]]:
    table: dict[str, dict[str, float]] = {}
    for episode in episodes:
        policy = QLearningPolicy(episode.config, epsilon=0.20, seed=policy_seed)
        policy.q_table = table
        env = ActiveCleaningEnv(episode.config, task_layout=episode.layout)
        observation = env.reset(seed=episode.mission_seed)
        policy.reset(episode_seed=RoleSeeds.from_master(episode.mission_seed).policy)
        while True:
            state, label, action = policy.act_with_label(observation, explore=True)
            result = env.step(action)
            policy.update(state, label, _belief_only_training_reward(observation, result), result.observation, done=result.terminated or result.truncated)
            observation = result.observation
            if result.terminated or result.truncated:
                break
    return table


def _evaluate(episodes: Sequence[FormalEpisode], table: dict[str, dict[str, float]], policy_seed: int) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for episode in episodes:
        baseline = run_episode(episode.config, seed=episode.mission_seed, policy=FullCoveragePolicy(episode.config), task_layout=episode.layout)
        policy = CoverageBackstoppedQLearningPolicy(episode.config, q_table=table, seed=policy_seed)
        result = run_episode(episode.config, seed=episode.mission_seed, policy=policy, baseline_distance=float(baseline["task_distance"]) if baseline["success"] else None, task_layout=episode.layout)
        ratio = float(result["task_distance"]) / float(baseline["task_distance"]) if float(baseline["task_distance"]) > 0 else None
        rows.append({
            "episode_id": episode.episode_id, "mission_index": episode.mission_index,
            "formal_success": bool(result["success"] and baseline["success"] and ratio is not None and ratio <= 1.0 + 1e-9),
            "observed_ratio": result["observed_ratio"], "ground_clear_ratio": result["ground_clear_ratio"],
            "discrete_clear_ratio": result["discrete_clear_ratio"], "task_distance": result["task_distance"],
            "baseline_distance": baseline["task_distance"], "path_ratio_to_full_coverage": ratio,
            "collisions": result["collisions"], "boundary_violations": result["boundary_violations"],
            "invalid_actions": result["invalid_actions"], "systematic_coverage_backstop_activated": bool(policy._coverage_activated),
        })
    return rows


def execute(args: argparse.Namespace) -> dict[str, Any]:
    from sanitation_campus_scenario.hidden_materializer import commit_hidden_configuration_freeze

    budget = load_formal_rl_budget(args.budget_contract)
    if any(value is None for value in (args.snapshot, args.session, args.hidden_receipt_root)):
        raise ValueError("formal Stage-A training requires --snapshot, --session and --hidden-receipt-root")

    def prepare(phase: str, freeze_receipt_path: Path | None = None) -> list[FormalEpisode]:
        return [materialize_stage_a_episode(args.scenario_config, args.motion_profile, args.work_root, phase=phase, task_index=index, map_resolution_m=args.map_resolution, planning_resolution_m=args.planning_resolution, max_steps=budget.max_steps_per_episode, snapshot_path=args.snapshot, session_path=args.session, hidden_receipt_root=args.hidden_receipt_root, freeze_receipt_path=freeze_receipt_path) for index in all_stage_a_task_indices(budget, phase)]
    started = time.time_ns()
    train, validation = prepare("train"), prepare("validation")
    # The hidden list is intentionally not materialized until all configured
    # train/validation runs finish and the configuration-freeze record exists.
    runs = []
    trained_tables: dict[int, dict[str, dict[str, float]]] = {}
    for seed in budget.policy_seeds:
        table = _train(train, seed)
        trained_tables[seed] = table
        validation_rows = _evaluate(validation, table, seed)
        runs.append({"policy_seed": seed, "q_state_count": len(table), "train_episode_count": len(train), "validation_episode_count": len(validation), "validation_all_formal_success": all(row["formal_success"] for row in validation_rows), "validation_rows": validation_rows})
    freeze = {"frozen_after_validation": True, "selection_source": "validation_only_before_hidden", "policy_seeds": list(budget.policy_seeds), "frozen_epoch_ns": time.time_ns()}
    freeze_receipt = commit_hidden_configuration_freeze(
        run_root=args.hidden_receipt_root,
        snapshot_path=args.snapshot, session_path=args.session,
        scenario_config=args.scenario_config, producer="formal_rl_stage_a",
        frozen_configuration=freeze,
    )
    hidden = prepare("hidden", freeze_receipt)
    for run in runs:
        run["hidden_episode_count"] = len(hidden)
        seed = int(run["policy_seed"])
        run["hidden_rows"] = _evaluate(hidden, trained_tables[seed], seed)
        run["hidden_all_formal_success"] = all(row["formal_success"] for row in run["hidden_rows"])
    return {"schema_version": 1, "status": "FORMAL_RL_STAGE_A_FIXED_MAP_COMPLETE" if all(run["hidden_all_formal_success"] for run in runs) else "FORMAL_RL_STAGE_A_FIXED_MAP_FAILED", "budget_contract": budget.report(), "fixed_map_id": budget.payload["stage_a_fixed_map"]["fixed_map_id"], "hidden_tasks_materialized_after_freeze": True, "configuration_freeze": freeze, "configuration_freeze_receipt_sha256": hashlib.sha256(freeze_receipt.read_bytes()).hexdigest(), "policy_runs": runs, "started_epoch_ns": started, "finished_epoch_ns": time.time_ns()}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-config", type=Path, required=True)
    parser.add_argument("--motion-profile", type=Path, required=True)
    parser.add_argument("--budget-contract", type=Path, required=True)
    parser.add_argument("--work-root", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path)
    parser.add_argument("--session", type=Path)
    parser.add_argument("--hidden-receipt-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--map-resolution", type=float, default=0.5)
    parser.add_argument("--planning-resolution", type=float, default=2.0)
    args = parser.parse_args(argv)
    result = execute(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if result["status"].endswith("_COMPLETE") else 2


if __name__ == "__main__":
    raise SystemExit(main())
