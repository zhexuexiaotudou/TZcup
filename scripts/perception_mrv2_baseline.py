#!/usr/bin/env python3
"""Create the immutable MODEL-RECOVERY-V2 baseline without reading sealed data."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
from typing import Any


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(
        (json.dumps(payload, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    )


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def command_record(command: list[str], timeout: int = 20) -> dict:
    try:
        completed = subprocess.run(
            command, cwd=ROOT, capture_output=True, text=True,
            timeout=timeout, check=False,
        )
        return {
            "command": command,
            "exit_code": completed.returncode,
            "stdout": completed.stdout.strip(),
            "stderr": completed.stderr.strip(),
        }
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return {
            "command": command, "exit_code": None, "stdout": "",
            "stderr": f"{type(exc).__name__}: {exc}",
        }


def asset(path: Path, role: str) -> dict:
    present = path.is_file()
    size = path.stat().st_size if present else None
    return {
        "role": role,
        "logical_external_path": path.as_posix(),
        "present": present,
        "bytes": size,
        "sha256": sha256(path) if present and size else None,
        "valid_nonempty_file": bool(present and size and size > 0),
        "repository_tracked": False,
    }


def build_payloads(args) -> dict[str, dict]:
    prior = args.prior_evidence.resolve()
    runtime = args.runtime_root.resolve()
    dataset = load_json(prior / "prod00_resources/dataset_inventory.json")
    old_models = load_json(prior / "prod00_resources/model_artifact_inventory.json")
    docker = load_json(prior / "prod00_resources/docker_inventory.json")
    gpu = load_json(prior / "prod00_resources/gpu_inventory.json")
    sensor = load_json(prior / "prod00_resources/real_sensor_inventory.json")
    j6 = load_json(prior / "prod00_resources/j6_inventory.json")
    x3_checkpoint = asset(
        runtime / "x3-direct-fcos-3class-v1/x3_fcos_r50_direct_3class.pt",
        "historical X3 failed-static checkpoint",
    )
    grounding = asset(
        runtime
        / "reference-cache/grounding-dino/groundingdino_swint_ogc.pth",
        "MRV2 official Grounding DINO acquisition candidate",
    )
    baseline = {
        "schema_version": 1,
        "stage": "MRV2-00-BASELINE",
        "protocol": "MODEL-RECOVERY-V2 / PRODUCT-FINALIZATION",
        "source_commit": args.source_commit,
        "source_tree": args.source_tree,
        "remote_pr": {
            "number": 90,
            "head": args.remote_head,
            "tree": args.remote_tree,
            "local_remote_tree_match": args.source_tree == args.remote_tree,
            "draft": True,
            "ci": "SUCCESS",
        },
        "HISTORICAL_X1_PASS": False,
        "HISTORICAL_X2_STATUS": "BLOCKED_EXTERNAL_NETWORK_ASSET",
        "HISTORICAL_X3_PASS": False,
        "NEW_MODEL_RECOVERY_V2_AUTHORIZED": True,
        "MRV2_is_historical_X4": False,
        "MODEL_FREEZE_X86_created": False,
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
        "AUTONOMOUS_EXECUTION": True,
        "ASK_USER_DURING_RUN": False,
        "fetch_attempt": {
            "exit_code": 1,
            "failure": "fatal bad object / remote did not send all necessary objects",
            "fallback": "GitHub API commit/tree verification",
        },
    }
    blockers = {
        "schema_version": 1,
        "stage": "MRV2-00-BLOCKER-BASELINE",
        "historical_routes": {
            "X1": "FAILED_STATIC_FULL_PIPELINE",
            "X2": "BLOCKED_EXTERNAL_NETWORK_ASSET",
            "X3": "FAILED_STATIC_FULL_PIPELINE",
            "immutable_history": True,
        },
        "new_recovery_targets": {
            "small_object_recall_lt_18px": 0.3076923076923077,
            "cross_world_metal_can_recall": 0.44565217391304346,
            "VAL_boundary_f1": 0.6880077913444143,
            "cross_world_negative_area_fp_per_frame": 0.13043478260869565,
        },
        "MRV2_routes_started": [],
        "MRV2_routes_exhausted": False,
        "MODEL_BLOCKED_INTERNAL": False,
        "claim_boundary": "The historical internal blocker is reopened only under the new MRV2 protocol; no MRV2 route has run yet.",
    }
    assets = {
        "schema_version": 1,
        "stage": "MRV2-00-ASSET-INVENTORY",
        "assets": [x3_checkpoint, grounding],
        "grounding_dino_newly_nonempty": grounding["valid_nonempty_file"],
        "grounding_dino_provenance_verified": False,
        "G5_SEALED_FINAL_read": False,
    }
    dataset_inventory = {
        **dataset,
        "stage": "MRV2-00-DATASET-INVENTORY",
        "development_partitions_authorized": [
            "train", "train_world_holdout", "VAL", "D1", "D2", "D3", "D4", "D5"
        ],
        "legacy_G4_D6_read": False,
        "G5_SEALED_FINAL_read": False,
        "sealed_final_access_authorized_now": False,
    }
    model_inventory = {
        "schema_version": 1,
        "stage": "MRV2-00-MODEL-INVENTORY",
        "historical_models": old_models["models"],
        "x3_checkpoint": x3_checkpoint,
        "grounding_dino_checkpoint_candidate": grounding,
        "selected_MRV2_product_candidate": None,
        "MODEL_FREEZE_X86_created": False,
    }
    environment = {
        "schema_version": 1,
        "stage": "MRV2-00-ENVIRONMENT",
        "platform": platform.platform(),
        "python": platform.python_version(),
        "docker_prior_inventory": docker,
        "gpu_prior_inventory": gpu,
        "sensor_prior_inventory": sensor,
        "j6_prior_inventory": j6,
        "docker_live": command_record(["docker", "version", "--format", "{{.Server.Version}}"]),
        "gpu_live": command_record([
            "nvidia-smi", "--query-gpu=name,memory.total,driver_version",
            "--format=csv,noheader",
        ]),
        "original_dirty_workspace_modified": False,
        "isolated_worktree": str(ROOT),
    }
    return {
        "BASELINE.json": baseline,
        "BLOCKER_BASELINE.json": blockers,
        "ASSET_INVENTORY.json": assets,
        "DATASET_INVENTORY.json": dataset_inventory,
        "MODEL_INVENTORY.json": model_inventory,
        "ENVIRONMENT.json": environment,
    }


def emit(output: Path, payloads: dict[str, dict]) -> None:
    output.mkdir(parents=True, exist_ok=True)
    for name, payload in payloads.items():
        write_json(output / name, payload)
    records = []
    for name in sorted(payloads):
        path = output / name
        records.append({"path": name, "bytes": path.stat().st_size, "sha256": sha256(path)})
    write_json(
        output / "artifact_manifest.json",
        {"schema_version": 1, "stage": "MRV2-00", "files": records},
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--prior-evidence", type=Path, required=True)
    parser.add_argument("--runtime-root", type=Path, required=True)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--source-tree", required=True)
    parser.add_argument("--remote-head", required=True)
    parser.add_argument("--remote-tree", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    emit(args.output, build_payloads(args))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
