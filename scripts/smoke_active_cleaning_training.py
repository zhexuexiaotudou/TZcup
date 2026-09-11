"""Bounded CPU train/validation throughput probe; never consumes hidden tasks."""

from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
for package in (ROOT / "starter_ws/src").iterdir():
    if package.is_dir():
        sys.path.insert(0, str(package))

from sanitation_active_cleaning.formal_training import materialize_episode, _train_episode
from sanitation_active_cleaning.evaluation import run_episode
from sanitation_active_cleaning.policies import FullCoveragePolicy
from sanitation_active_cleaning.rl import CoverageBackstoppedQLearningPolicy


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--max-steps", type=int, default=8)
    args = parser.parse_args(argv)
    if not 1 <= args.max_steps <= 40:
        parser.error("smoke max-steps must be between 1 and 40")
    args.output_root.mkdir(parents=True, exist_ok=False)
    report = {
        "status": "RUNNING", "formal_acceptance": False,
        "hidden_consumed": False, "product_perception_used": False,
        "gazebo_runtime_verified": False, "max_steps": args.max_steps,
        "timing_scope": "single_cpu_process_public_train_validation_smoke",
        "rows": [],
        "source_sha256": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in sorted((ROOT / "starter_ws/src/sanitation_active_cleaning/sanitation_active_cleaning").glob("*.py"))},
    }
    destination = args.output_root / "report.json"

    def save():
        destination.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    save()
    table = {}
    try:
        for split in ("train", "val"):
            started = time.perf_counter()
            episode = materialize_episode(
                ROOT / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml",
                ROOT / "config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml",
                args.output_root / "episodes", split=split, map_index=0, mission_index=0,
                map_resolution_m=0.5, planning_resolution_m=2.0, max_steps=args.max_steps,
            )
            materialization_s = time.perf_counter() - started
            started = time.perf_counter()
            if split == "train":
                result = _train_episode(episode, table, policy_seed=7)
            else:
                baseline = run_episode(episode.config, seed=episode.mission_seed,
                    policy=FullCoveragePolicy(episode.config), baseline_distance=None,
                    task_layout=episode.layout)
                result = run_episode(episode.config, seed=episode.mission_seed,
                    policy=CoverageBackstoppedQLearningPolicy(episode.config, q_table=deepcopy(table), seed=7),
                    baseline_distance=float(baseline["task_distance"]) if baseline["success"] else None,
                    task_layout=episode.layout)
                result["baseline"] = baseline
            elapsed = time.perf_counter() - started
            steps = result["steps"] + result.get("baseline", {}).get("steps", 0)
            row = {"split": split, "materialization_s": materialization_s,
                   "rollout_wall_s": elapsed, "completed_action_steps": steps,
                   "action_steps_per_second": steps / elapsed, "result": result}
            report["rows"].append(row)
            save()
            print(json.dumps({"split": split, "rollout_wall_s": elapsed}), flush=True)
        report["status"] = "SMOKE_COMPLETED_NOT_FORMAL"
        report["q_table_sha256"] = hashlib.sha256(json.dumps(table, sort_keys=True).encode()).hexdigest()
        report["q_state_count"] = len(table)
        report["formal_duration_estimate"] = None
        report["estimate_limitation"] = "Truncated smoke cannot predict completed 400-step episode runtime or formal success."
    except BaseException as error:
        report["status"] = "SMOKE_FAILED"
        report["failure"] = {"type": type(error).__name__, "message": str(error)}
        raise
    finally:
        save()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
