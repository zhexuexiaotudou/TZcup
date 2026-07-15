#!/usr/bin/env python3
"""Exercise 20 deterministic Gazebo SetPose seeds without overstating coverage."""

import argparse
import json
import random
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--output", required=True)
    args = parser.parse_args(); randomizer = random.Random(20260715)
    trials = []
    for seed in range(20):
        # World x=-4 corresponds to map x=4, across the Stage4R coverage area.
        y = randomizer.uniform(-3.5, 3.5)
        completed = subprocess.run(
            ["bash", "/work/scripts/gz_set_dynamic_obstacle.sh", "-4.0", f"{y:.6f}"],
            text=True, capture_output=True, check=False,
        )
        trials.append({"seed": seed, "world_pose": [-4.0, y, 0.55], "set_pose_success": completed.returncode == 0 and "true" in completed.stdout.lower(), "collision_count": None, "minimum_distance_m": None, "coverage_completion": False})
    report = {
        "schema_version": 1, "requested_seed_count": 20, "executed_set_pose_seed_count": len(trials),
        "coverage_interaction_trial_count": 0,
        "blocked_by": "same-frame localization failure prevented a valid coverage mission; service-only obstacle motion is not counted as an avoidance trial",
        "success": False, "trials": trials,
    }
    Path(args.output).write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__": main()
