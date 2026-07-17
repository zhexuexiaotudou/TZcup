from __future__ import annotations

import argparse
import json

from .evaluate import evaluate_model
from .onnx_model import build_color_prototype_model
from .synthetic import write_dataset


def generate_main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--start-seed", type=int, default=0)
    parser.add_argument("--scene-count", type=int, default=20)
    args = parser.parse_args()
    manifest = write_dataset(args.output, list(range(args.start_seed, args.start_seed + args.scene_count)))
    print(json.dumps({"scene_count": manifest["scene_count"], "split_hash": manifest["split_hash"]}))
    return 0


def build_model_main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    print(build_color_prototype_model(args.output))
    return 0


def evaluate_main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--seeds", default="16,17,18,19")
    args = parser.parse_args()
    seeds = [int(value) for value in args.seeds.split(",") if value]
    report = evaluate_model(args.model, seeds, args.output)
    print(json.dumps({"synthetic_perception_pass": report["synthetic_perception_pass"]}))
    return 0 if report["synthetic_perception_pass"] else 2
