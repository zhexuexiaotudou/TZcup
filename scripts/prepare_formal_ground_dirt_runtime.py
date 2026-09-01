#!/usr/bin/env python3
"""Create a one-patch random-campus episode for formal dirt runtime acceptance."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "starter_ws/src/sanitation_campus_scenario"
sys.path.insert(0, str(PACKAGE))

from sanitation_campus_scenario.generator import generate_episode, load_config  # noqa: E402
from sanitation_campus_scenario.io import write_episode  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    config = copy.deepcopy(
        load_config(PACKAGE / "config/default_scenario.yaml")
    )
    config["episode"]["dirt_patch_count"] = 1
    config["episode"]["pedestrian_count"] = 0
    files = generate_episode(config, "formal", "train", 0, 0)
    write_episode(args.output_dir, files)
    truth = json.loads(files["evaluator/ground_truth.json"])
    patch = truth["dirt_patches"][0]
    if patch["size_m"] != [2.0, 0.5]:
        raise RuntimeError(
            f"frozen acceptance seed no longer yields the 2.0 x 0.5 m patch: {patch}"
        )
    setup = {
        "schema_version": 1,
        "world_name": "campus_formal",
        "world_path": str((args.output_dir / "public/world.sdf").resolve()),
        "patch": patch,
        "initial_area_m2": truth["dirt_union_area_m2"],
        "cell_contract": truth["dirt_cell_contract"],
        "rigid_litter_ids": [item["object_id"] for item in truth["discrete_cubes"]],
        "truth_use": "evaluator_initialization_only_not_product_ros",
    }
    setup_path = args.output_dir / "evaluator/runtime_setup.json"
    setup_path.write_text(json.dumps(setup, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(setup, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
