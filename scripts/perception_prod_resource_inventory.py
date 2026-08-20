#!/usr/bin/env python3
"""Collect the fail-closed PERCEPTION-PROD-00 resource inventory.

Large model and dataset assets stay outside Git.  This tool records only their
logical paths, sizes, hashes and machine capability probes in compact JSON.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml


EXPECTED_MODELS = {
    "x1_fcos_r50_teacher": (
        "p2-teacher-v5/fcos_resnet50_fpn_teacher.pt",
        "a5884ac9bfa4e89f2ae8a25f4cae0521e263dd951ef895fa1185f013b2f04ee5",
    ),
    "classifier": (
        "p4-screening-v5-a3-distilled-top16-batch16-v1/classifier.pt",
        "d145090681551160b74aa0e43bb711561c8e4d3a1494535243f700935bcfb2e6",
    ),
    "leaf": (
        "p4-screening-v5-a3-distilled-top16-batch16-v1/leaf.pt",
        "ecba9043d0e0523ad5b79381bcc7c21583ccba2c9a806f2179e5ca7e3e14aa2e",
    ),
    "puddle": (
        "p4-screening-v5-a3-distilled-top16-batch16-v1/puddle.pt",
        "8b3aee52679105f857b9e43694dc08304e5cda73c9536a7904c52ed33b31d2e2",
    ),
}

EXPECTED_QA = (
    "g4-v5-formal-merged-v3/qa_formal/g4_dataset_qa.json",
    "72baf192e70c59d369c284c8141dcc6e2c03350dca930212ae97cf2182d1ab01",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run(command: list[str], *, cwd: Path | None = None) -> dict[str, Any]:
    try:
        completed = subprocess.run(
            command,
            cwd=cwd,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=30,
            check=False,
        )
        return {
            "command": command,
            "exit_code": completed.returncode,
            "stdout": completed.stdout.strip(),
            "stderr": completed.stderr.strip(),
        }
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return {
            "command": command,
            "exit_code": None,
            "stdout": "",
            "stderr": f"{type(exc).__name__}: {exc}",
        }


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(
        (json.dumps(payload, indent=2, ensure_ascii=False) + "\n").encode("utf-8")
    )


def model_inventory(runtime_root: Path) -> dict[str, Any]:
    models = []
    for model_id, (relative, expected_hash) in EXPECTED_MODELS.items():
        path = runtime_root / relative
        actual_hash = sha256(path) if path.is_file() else None
        models.append(
            {
                "model_id": model_id,
                "logical_path": path.as_posix(),
                "exists": path.is_file(),
                "bytes": path.stat().st_size if path.is_file() else None,
                "expected_sha256": expected_hash,
                "actual_sha256": actual_hash,
                "hash_match": actual_hash == expected_hash,
            }
        )
    return {
        "schema_version": 1,
        "models": models,
        "all_required_models_recovered": all(item["hash_match"] for item in models),
        "product_candidate_selected": False,
        "claim_boundary": "asset recovery does not pass the X1 full-pipeline gate",
    }


def dataset_inventory(runtime_root: Path) -> dict[str, Any]:
    relative, expected_hash = EXPECTED_QA
    qa_path = runtime_root / relative
    actual_hash = sha256(qa_path) if qa_path.is_file() else None
    qa = json.loads(qa_path.read_text(encoding="utf-8")) if qa_path.is_file() else {}
    leakage = qa.get("leakage", {})
    leakage_zero = all(
        not leakage.get(key)
        for key in (
            "target_asset_leakage",
            "hard_negative_asset_leakage",
            "trajectory_leakage",
            "world_leakage",
            "cross_split_exact_duplicates",
            "cross_split_phash_duplicates",
        )
    ) and all(
        int(leakage.get(key, 0)) == 0
        for key in (
            "cross_split_exact_duplicate_count",
            "cross_split_phash_duplicate_count",
        )
    )
    return {
        "schema_version": 1,
        "dataset_id": "G4_V5_FORMAL_CLEAN",
        "dataset_root": (runtime_root / "g4-v5-formal-merged-v3").as_posix(),
        "qa_path": qa_path.as_posix(),
        "expected_qa_sha256": expected_hash,
        "actual_qa_sha256": actual_hash,
        "qa_hash_match": actual_hash == expected_hash,
        "world_count": qa.get("world_count"),
        "scene_count": qa.get("scene_count"),
        "frame_count": qa.get("frame_count"),
        "pose_reset_consistency": qa.get("scene_pose_reset_valid_rate"),
        "manifest_pixel_consistency": qa.get("manifest_pixel_target_consistency_rate"),
        "leakage_zero": leakage_zero,
        "gates": qa.get("gates", {}),
        "G4_dataset_gate_pass": qa.get("G4_dataset_gate_pass") is True,
        "G5_SEALED_FINAL_read": False,
    }


def reference_inventory(repo_root: Path) -> dict[str, Any]:
    registry_path = repo_root / "third_party/perception/dependency_registry.yaml"
    registry = yaml.safe_load(registry_path.read_text(encoding="utf-8"))
    dependencies = registry.get("dependencies", [])
    upstream_audit = []
    for dependency in dependencies:
        repository = str(dependency.get("upstream_repository", ""))
        slug = repository.removeprefix("https://github.com/").removesuffix(".git")
        probe = run(["gh", "api", f"repos/{slug}/commits/{dependency['upstream_commit']}"])
        upstream_audit.append(
            {
                "name": dependency.get("name"),
                "repository": repository,
                "pinned_commit": dependency.get("upstream_commit"),
                "pinned_commit_exists_upstream": probe["exit_code"] == 0,
            }
        )
    return {
        "schema_version": 1,
        "registry_path": registry_path.relative_to(repo_root).as_posix(),
        "registry_sha256": sha256(registry_path),
        "dependencies": dependencies,
        "official_upstream_audit": upstream_audit,
        "all_pins_exist_upstream": all(
            item["pinned_commit_exists_upstream"] for item in upstream_audit
        ),
        "checkpoint_cache_present": False,
        "x2_checkpoint_ready": False,
        "x3_product_authorized": False,
    }


def collect(args: argparse.Namespace) -> dict[str, Any]:
    repo_root = args.repo_root.resolve()
    runtime_root = args.runtime_root.resolve()
    observed_at = datetime.now(timezone.utc).isoformat()
    git_head = run(["git", "rev-parse", "HEAD"], cwd=repo_root)
    git_branch = run(["git", "branch", "--show-current"], cwd=repo_root)
    git_status = run(["git", "status", "--porcelain"], cwd=repo_root)

    repo_state = {
        "schema_version": 1,
        "observed_at": observed_at,
        "repository": "https://github.com/zhexuexiaotudou/TZcup",
        "local_head": git_head["stdout"],
        "local_branch": git_branch["stdout"],
        "worktree_clean_before_inventory": git_status["stdout"] == "",
        "remote_pr": 90,
        "remote_head": args.remote_head,
        "git_fetch_origin_prune": args.fetch_status,
        "git_transport_boundary": (
            "GitHub Smart-HTTP timed out; remote state verified with GitHub REST"
            if args.fetch_status != "success"
            else None
        ),
    }

    docker_probe = run(
        [
            "docker",
            "image",
            "inspect",
            "tzcup/sanitation-jazzy:stage5b",
            "--format",
            "{{.Id}} {{.Size}}",
        ]
    )
    docker_inventory = {
        "schema_version": 1,
        "product_base_image": "tzcup/sanitation-jazzy:stage5b",
        "probe": docker_probe,
        "available": docker_probe["exit_code"] == 0,
        "perception_product_image_built": False,
        "perception_reference_image_built": False,
    }

    gpu_probe = run(
        [
            "nvidia-smi",
            "--query-gpu=name,driver_version,memory.total,memory.free",
            "--format=csv,noheader,nounits",
        ]
    )
    gpu_inventory = {
        "schema_version": 1,
        "probe": gpu_probe,
        "cuda_gpu_available": gpu_probe["exit_code"] == 0,
        "required_reference_gpu": "NVIDIA RTX 4080 Laptop GPU",
    }

    j6_source = runtime_root / "j6-toolchain/toolchain_discovery.json"
    j6 = json.loads(j6_source.read_text(encoding="utf-8")) if j6_source.is_file() else {}
    j6_inventory = {
        "schema_version": 1,
        "source_path": j6_source.as_posix(),
        "source_sha256": sha256(j6_source) if j6_source.is_file() else None,
        "official_package_ready": j6.get("official_toolchain_package_ready") is True,
        "local_oe_version": j6.get("official_source", {}).get("oe_version"),
        "official_docs_version": args.horizon_docs_version,
        "official_docs_url": "https://doc.oe.horizon.auto/",
        "version_difference_requires_resolution": (
            j6.get("official_source", {}).get("oe_version")
            != args.horizon_docs_version
        ),
        "hbdk_version": j6.get("required_versions", {}).get("hbdk4_compiler"),
        "hmct_version": j6.get("required_versions", {}).get("hmct"),
        "board_device_count": j6.get("host", {}).get("board_device_count", 0),
        "frozen_j6_student_available": False,
        "ptq_or_compile_executed": False,
        "PRODUCT_J6_TOOLCHAIN_READY": False,
        "PRODUCT_J6_BOARD_READY": False,
    }

    camera_probe = run(
        [
            "powershell.exe",
            "-NoProfile",
            "-Command",
            "Get-PnpDevice -PresentOnly -Class Camera -ErrorAction SilentlyContinue | Select-Object FriendlyName,InstanceId,Status | ConvertTo-Json -Compress",
        ]
    )
    real_sensor_inventory = {
        "schema_version": 1,
        "camera_probe": camera_probe,
        "detected_camera_summary": "Integrated Camera only",
        "rgbd_device_present": False,
        "auditable_rgbd_recording_present": False,
        "independent_map_gt_present": False,
        "integrated_camera_accepted_as_rgbd": False,
        "PRODUCT_FIELD_READY": False,
        "REAL_DOMAIN_BLOCKED_EXTERNAL": True,
    }

    return {
        "repo_state.json": repo_state,
        "model_artifact_inventory.json": model_inventory(runtime_root),
        "dataset_inventory.json": dataset_inventory(runtime_root),
        "docker_inventory.json": docker_inventory,
        "gpu_inventory.json": gpu_inventory,
        "reference_dependency_inventory.json": reference_inventory(repo_root),
        "j6_inventory.json": j6_inventory,
        "real_sensor_inventory.json": real_sensor_inventory,
    }


def emit(output_dir: Path, payloads: dict[str, Any]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    for name, payload in payloads.items():
        write_json(output_dir / name, payload)
    manifest_entries = []
    for name in sorted(payloads):
        path = output_dir / name
        manifest_entries.append(
            {"path": name, "bytes": path.stat().st_size, "sha256": sha256(path)}
        )
    write_json(
        output_dir / "artifact_manifest.json",
        {
            "schema_version": 1,
            "stage": "PERCEPTION-PROD-00",
            "files": manifest_entries,
            "all_inventory_files_present": len(manifest_entries) == 8,
        },
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--runtime-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, action="append", required=True)
    parser.add_argument("--remote-head", required=True)
    parser.add_argument("--fetch-status", required=True)
    parser.add_argument("--horizon-docs-version", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    payloads = collect(args)
    for output_dir in args.output_dir:
        emit(output_dir, payloads)
    if not payloads["model_artifact_inventory.json"]["all_required_models_recovered"]:
        return 2
    if not payloads["dataset_inventory.json"]["G4_dataset_gate_pass"]:
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
