from __future__ import annotations

import argparse
import json
from pathlib import Path

from .assets import write_gazebo_assets
from .evaluation import evaluate_model
from .j6_preflight import run_preflight
from .models import train_candidates
from .rendered import validate_annotations, write_dataset


def generate_assets_main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--registry", required=True); parser.add_argument("--output", required=True); args = parser.parse_args()
    report = write_gazebo_assets(args.registry, args.output); print(json.dumps({"target_variant_count": report["target_variant_count"]})); return 0


def generate_dataset_main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--registry", required=True); parser.add_argument("--output", required=True); parser.add_argument("--scene-count", type=int, default=500); parser.add_argument("--frames-per-scene", type=int, default=10); args = parser.parse_args()
    manifest = write_dataset(args.output, args.registry, list(range(args.scene_count)), args.frames_per_scene)
    qa = validate_annotations(args.output); (Path(args.output) / "annotation_qa.json").write_text(json.dumps(qa, indent=2) + "\n", encoding="utf-8")
    calibration = {"schema_version": 1, "dataset_id": manifest["dataset_id"], "frame_count": min(500, manifest["frame_count"]), "records": [{"image": item["image"], "sha256": item["image_sha256"]} for item in manifest["records"] if item["split"] == "train"][:500], "j6_quantization_executed": False}
    (Path(args.output) / "calibration_dataset_manifest.json").write_text(json.dumps(calibration, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"scene_count": manifest["scene_count"], "frame_count": manifest["frame_count"], "annotation_qa_pass": qa["annotation_qa_pass"]})); return 0 if qa["annotation_qa_pass"] else 2


def train_models_main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--registry", required=True); parser.add_argument("--config", required=True); parser.add_argument("--output", required=True); args = parser.parse_args()
    report = train_candidates(args.registry, args.config, args.output); print(json.dumps({"selected_candidate": report["selected_candidate"], "model_sha256": report["selected_model_sha256"]})); return 0


def evaluate_models_main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--model", required=True); parser.add_argument("--registry", required=True); parser.add_argument("--output", required=True); parser.add_argument("--test-scenes", type=int, default=100); parser.add_argument("--frames-per-scene", type=int, default=10); args = parser.parse_args()
    report = evaluate_model(args.model, args.registry, args.output, args.test_scenes, args.frames_per_scene); print(json.dumps({"rendered_synthetic_perception_pass": report["rendered_synthetic_perception_pass"], "color_shortcut_pass": report["color_shortcut"]["color_shortcut_pass"]})); return 0 if report["rendered_synthetic_perception_pass"] else 2


def j6_preflight_main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--model", required=True); parser.add_argument("--calibration-manifest"); parser.add_argument("--output", required=True); args = parser.parse_args()
    report = run_preflight(args.model, args.calibration_manifest, args.output); print(json.dumps({"j6_toolchain_available": report["j6_toolchain_available"], "j6_model_precheck_pass": report["j6_model_precheck_pass"]})); return 0
