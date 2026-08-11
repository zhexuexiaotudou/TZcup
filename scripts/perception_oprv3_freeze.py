#!/usr/bin/env python3
"""Create a fail-closed OPRV3-08 x86 product-pipeline freeze.

The generator does not run inference or alter model output. It verifies that
the passed OPRV3-07 evidence, exact model files, runtime manifest, immutable
container and redistribution notices agree, then creates one atomic freeze
directory. A failed check leaves no partially valid freeze behind.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

import yaml


FULL_SHA = re.compile(r"[0-9a-f]{40}")
OCI_DIGEST = re.compile(r".+@sha256:[0-9a-f]{64}")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def artifact(path: Path) -> dict:
    require(path.is_file(), f"artifact missing: {path}")
    return {"path": path.as_posix(), "sha256": sha256(path), "bytes": path.stat().st_size}


def git_head(repo_root: Path) -> str:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=repo_root, text=True,
        capture_output=True, check=True,
    )
    return result.stdout.strip()


def _model_record(path: Path, expected: dict, role: str) -> dict:
    record = artifact(path)
    require(record["sha256"] == expected["sha256"], f"{role} SHA-256 mismatch")
    return {**record, "role": role}


def build_freeze(args: argparse.Namespace, timestamp: str) -> tuple[dict, dict, dict]:
    dev = load_json(args.dev_report)
    require(dev.get("protocol") == "OPRV3-07", "wrong development protocol")
    require(dev.get("OPRV3_X86_DEV_PASS") is True, "OPRV3-07 did not pass")
    require(dev.get("MODEL_BLOCKED_INTERNAL") is False, "model remains internally blocked")
    require(dev.get("freeze_allowed") is True, "development report forbids freeze")
    require(not dev.get("failed_sections"), "development report has failed sections")

    moving = load_json(args.moving_gate)
    area = load_json(args.area_gate)
    product_map = load_json(args.product_map_gate)
    performance = load_json(args.performance_gate)
    require(moving.get("OPRV3_02_pass") is True, "moving gate did not pass")
    require(area.get("OPRV3_06_AREA_PASS") is True, "area gate did not pass")
    require(product_map.get("GT_used_by_product_pipeline") is False, "product pipeline used GT")
    require(product_map.get("metrics", {}).get("pre_fov_target_creation") == 0, "pre-FOV target creation detected")
    require(performance.get("pass") is True, "performance gate did not pass")
    require(performance.get("metrics", {}).get("formal_product_pipeline_executed") is True, "formal pipeline was not executed")

    source_commit = dev.get("source_commit")
    require(bool(FULL_SHA.fullmatch(str(source_commit))), "development source commit is not full SHA")
    require(moving.get("source_commits") == [source_commit], "moving source commit mismatch")
    require(product_map.get("source_commit") == source_commit, "product-map source commit mismatch")
    require(performance.get("source_commit") == source_commit, "performance source commit mismatch")
    for name, report in (("development", dev), ("moving", moving), ("area", area), ("product map", product_map), ("performance", performance)):
        require(report.get("G5_SEALED_FINAL_read") is False, f"{name} evidence crossed sealed-final boundary")
        require(report.get("legacy_G4_D6_read") is False, f"{name} evidence read legacy D6")

    models = performance["models"]
    model_files = {
        "detector_checkpoint": _model_record(args.detector_checkpoint, models["detector"], "detector_checkpoint"),
        "detector_onnx": _model_record(args.detector_onnx, models["onnx"]["detector"], "detector_onnx"),
        "leaf_checkpoint": _model_record(args.leaf_checkpoint, models["leaf"], "leaf_checkpoint"),
        "leaf_onnx": _model_record(args.leaf_onnx, models["onnx"]["leaf"], "leaf_onnx"),
        "puddle_checkpoint": _model_record(args.puddle_checkpoint, models["puddle"], "puddle_checkpoint"),
        "puddle_onnx": _model_record(args.puddle_onnx, models["onnx"]["puddle"], "puddle_onnx"),
    }
    require(models["onnx"].get("provider") == "CUDAExecutionProvider", "performance did not activate CUDAExecutionProvider")
    require(OCI_DIGEST.fullmatch(args.container_image) is not None, "container image must use immutable repo@sha256 digest")

    pipeline = yaml.safe_load(args.pipeline_config.read_text(encoding="utf-8"))
    runtime = pipeline["runtime"]
    require(runtime.get("backend") == "onnxruntime", "runtime backend is not onnxruntime")
    require(runtime.get("required_provider") == "CUDAExecutionProvider", "runtime does not require CUDA")
    require(runtime.get("cpu_fallback_forbidden") is True, "CPU fallback is not forbidden")
    require(runtime.get("minimum_area_region_m2_by_class") == {"leaf_pile": 0.02, "puddle": 0.05}, "class area thresholds drifted")
    require(runtime.get("performance", {}).get("minimum_effective_hz") == 10.0, "throughput contract drifted")
    require(runtime.get("performance", {}).get("maximum_drop_rate") == 0.01, "drop contract drifted")

    selected = area["selected_config"]["by_class"]
    evidence = {
        "development": artifact(args.dev_report),
        "moving": artifact(args.moving_gate),
        "area": artifact(args.area_gate),
        "product_map": artifact(args.product_map_gate),
        "performance": artifact(args.performance_gate),
    }
    repo_files = {
        "pipeline_config": artifact(args.pipeline_config),
        "dockerfile": artifact(args.dockerfile),
        "third_party_notices": artifact(args.third_party_notices),
        "asset_license_manifest": artifact(args.asset_license_manifest),
        "tracker": artifact(args.repo_root / "starter_ws/src/sanitation_perception/sanitation_perception/tracker_v2.py"),
        "dynamic_trash_map": artifact(args.repo_root / "starter_ws/src/sanitation_perception/sanitation_perception/dynamic_trash_map.py"),
        "scheduler": artifact(args.repo_root / "starter_ws/src/sanitation_spot_cleaning/sanitation_spot_cleaning/cleaning_task_scheduler.py"),
        "post_clean_verification": artifact(args.repo_root / "starter_ws/src/sanitation_spot_cleaning/sanitation_spot_cleaning/post_clean_verification.py"),
        "spot_cleaning_config": artifact(args.repo_root / "starter_ws/src/sanitation_spot_cleaning/config/spot_cleaning.yaml"),
        "sealed_evaluator": artifact(args.sealed_evaluator),
        "sealed_evaluator_policy": artifact(args.sealed_policy),
        "sealed_geometry_audit": artifact(args.geometry_audit),
        "sealed_development_world_manifest": artifact(args.development_world_manifest),
        "sealed_evaluator_moving_dependency": artifact(args.repo_root / "scripts/perception_oprv3_moving_benchmark.py"),
        "sealed_evaluator_dataset_dependency": artifact(args.repo_root / "starter_ws/src/sanitation_learning/sanitation_learning/g5_dataset.py"),
    }
    tool_commit = git_head(args.repo_root)
    require(bool(FULL_SHA.fullmatch(tool_commit)), "freeze tool revision is not a full SHA")

    freeze = {
        "schema_version": 1,
        "protocol": "OPRV3-08",
        "freeze_id": args.freeze_id,
        "created_at_utc": timestamp,
        "status": "FROZEN_X86",
        "evaluated_source_commit": source_commit,
        "freeze_tool_revision": tool_commit,
        "candidate": "MRV2-A_online_first_product_pipeline",
        "models": {
            "detector": {
                "architecture": models["detector"]["architecture"],
                "checkpoint": model_files["detector_checkpoint"],
                "onnx": model_files["detector_onnx"],
                "input_size_wh": models["detector"]["input_size"],
                "action_threshold": models["detector"]["action_threshold"],
                "classes": ["plastic_bottle", "metal_can", "paper_litter"],
            },
            "classifier": {"mode": "integrated_detector_classes", "artifact": None},
            "small_object_specialist": {"mode": "not_used", "artifact": None},
            "leaf": {
                "checkpoint": model_files["leaf_checkpoint"], "onnx": model_files["leaf_onnx"],
                "threshold": selected["leaf_pile"]["threshold"], "morphology": selected["leaf_pile"]["morphology"],
            },
            "puddle": {
                "checkpoint": model_files["puddle_checkpoint"], "onnx": model_files["puddle_onnx"],
                "threshold": selected["puddle"]["threshold"], "morphology": selected["puddle"]["morphology"],
            },
        },
        "sensor_and_preprocess": {
            "inputs": performance["input_contract"]["product_inputs"],
            "detector_input_size_wh": models["detector"]["input_size"],
            "camera_frustum": runtime["camera_frustum"],
            "roi_or_tile": "full_frame_resize_no_tiling",
            "prompts": "not_used",
            "preprocess_spec": runtime["preprocess_spec"],
            "postprocess_spec": runtime["postprocess_spec"],
        },
        "online_contract": {
            "cadence_hz": {"source": performance["input_contract"]["source_hz"], "detector": performance["input_contract"]["detector_hz"], "leaf": performance["input_contract"]["leaf_hz"], "puddle": performance["input_contract"]["puddle_hz"]},
            "queue": performance["input_contract"]["queue"],
            "tracker_v2": runtime["tracker_v2"],
            "dynamic_trash_map": runtime["dynamic_trash_map"],
            "minimum_area_region_m2_by_class": runtime["minimum_area_region_m2_by_class"],
            "cleaning_task_scheduler": repo_files["scheduler"],
            "post_clean_verification": {"implementation": repo_files["post_clean_verification"], "config": repo_files["spot_cleaning_config"]},
        },
        "runtime": {
            "backend": runtime["backend"],
            "provider": models["onnx"]["provider"],
            "io_binding_required": runtime["io_binding_required"],
            "cpu_fallback_forbidden": runtime["cpu_fallback_forbidden"],
            "container_image": args.container_image,
            "environment": performance["environment"],
            "performance_contract": runtime["performance"],
        },
        "evidence": evidence,
        "implementation": repo_files,
        "sealed_final": {
            "dataset": "G5_SEALED_FINAL or G5_V2", "maximum_accesses": 1,
            "accesses_at_freeze": 0, "evaluator": repo_files["sealed_evaluator"],
            "policy": repo_files["sealed_evaluator_policy"],
            "geometry_audit": repo_files["sealed_geometry_audit"],
            "development_world_manifest": repo_files["sealed_development_world_manifest"],
        },
        "release_boundary": {"x86_development_pass": True, "sealed_final_pass": False, "j6_pass": False, "field_pass": False},
    }
    dependency_lock = {
        "schema_version": 1,
        "lock_type": "immutable_oci_runtime",
        "container_image": args.container_image,
        "dockerfile": repo_files["dockerfile"],
        "onnxruntime_gpu_version": "1.20.2",
        "required_provider": "CUDAExecutionProvider",
        "ros_distro": "jazzy",
        "python": performance["environment"].get("python", "container-defined"),
        "model_onnx_sha256": {key: value["sha256"] for key, value in model_files.items() if key.endswith("_onnx")},
    }
    frozen_pipeline = pipeline
    frozen_pipeline["pipeline_id"] = f"oprv3_mrv2a_x86_{source_commit[:12]}"
    frozen_pipeline["model_manifests"] = {
        "detector": model_files["detector_onnx"],
        "classifier": {"mode": "integrated_detector_classes"},
        "leaf_segmenter": model_files["leaf_onnx"],
        "puddle_segmenter": model_files["puddle_onnx"],
    }
    frozen_pipeline["frozen_postprocess"] = {
        "detector_action_threshold": models["detector"]["action_threshold"],
        "leaf": {"threshold": selected["leaf_pile"]["threshold"], "morphology": selected["leaf_pile"]["morphology"]},
        "puddle": {"threshold": selected["puddle"]["threshold"], "morphology": selected["puddle"]["morphology"]},
    }
    frozen_pipeline["freeze"] = {"protocol": "OPRV3-08", "freeze_id": args.freeze_id, "evaluated_source_commit": source_commit, "container_image": args.container_image}
    frozen_pipeline["status"] = {"formal_pipeline_pass": True, "oprv3_x86_dev_pass": True, "frozen": True, "sealed_final_pass": False, "competition_pipeline_claim_allowed": False}
    return freeze, dependency_lock, frozen_pipeline


def write_freeze(args: argparse.Namespace, freeze: dict, dependency_lock: dict, pipeline: dict) -> Path:
    output = args.output_dir.resolve()
    require(not output.exists(), f"output already exists: {output}")
    temporary = output.parent / f".{output.name}.tmp"
    if temporary.exists():
        shutil.rmtree(temporary)
    temporary.mkdir(parents=True)
    try:
        model_path = temporary / "MODEL_FREEZE_X86.json"
        dep_path = temporary / "PERCEPTION_X86_DEPENDENCY_LOCK.json"
        pipe_path = temporary / "perception_pipeline_x86_frozen.yaml"
        model_path.write_text(json.dumps(freeze, indent=2) + "\n", encoding="utf-8")
        dep_path.write_text(json.dumps(dependency_lock, indent=2) + "\n", encoding="utf-8")
        pipe_path.write_text(yaml.safe_dump(pipeline, sort_keys=False), encoding="utf-8")
        manifest = {
            "schema_version": 1, "protocol": "OPRV3-08", "freeze_id": freeze["freeze_id"],
            "status": "FROZEN_X86", "evaluated_source_commit": freeze["evaluated_source_commit"],
            "freeze_tool_revision": freeze["freeze_tool_revision"],
            "container_image": freeze["runtime"]["container_image"],
            "files": {p.name: artifact(p) for p in (model_path, dep_path, pipe_path)},
            "G5_SEALED_FINAL_read": False, "legacy_G4_D6_read": False,
        }
        manifest_path = temporary / "PERCEPTION_X86_FREEZE_MANIFEST.json"
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        checksum_paths = [model_path, manifest_path, dep_path, pipe_path]
        (temporary / "SHA256SUMS").write_text("".join(f"{sha256(path)}  {path.name}\n" for path in checksum_paths), encoding="utf-8")
        temporary.replace(output)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dev-report", required=True, type=Path)
    parser.add_argument("--moving-gate", required=True, type=Path)
    parser.add_argument("--area-gate", required=True, type=Path)
    parser.add_argument("--product-map-gate", required=True, type=Path)
    parser.add_argument("--performance-gate", required=True, type=Path)
    for name in ("detector-checkpoint", "detector-onnx", "leaf-checkpoint", "leaf-onnx", "puddle-checkpoint", "puddle-onnx"):
        parser.add_argument(f"--{name}", required=True, type=Path)
    parser.add_argument("--pipeline-config", required=True, type=Path)
    parser.add_argument("--dockerfile", required=True, type=Path)
    parser.add_argument("--third-party-notices", required=True, type=Path)
    parser.add_argument("--asset-license-manifest", required=True, type=Path)
    parser.add_argument("--sealed-evaluator", required=True, type=Path)
    parser.add_argument("--sealed-policy", required=True, type=Path)
    parser.add_argument("--geometry-audit", required=True, type=Path)
    parser.add_argument("--development-world-manifest", required=True, type=Path)
    parser.add_argument("--container-image", required=True)
    parser.add_argument("--freeze-id", required=True)
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    timestamp = datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
    freeze, lock, pipeline = build_freeze(args, timestamp)
    output = write_freeze(args, freeze, lock, pipeline)
    print(json.dumps({"OPRV3_08_PASS": True, "freeze_id": args.freeze_id, "output": output.as_posix()}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
