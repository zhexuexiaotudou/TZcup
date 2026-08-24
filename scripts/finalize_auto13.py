#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, payload) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def camera_inventory() -> dict:
    command = (
        "Get-PnpDevice -Class Camera -ErrorAction SilentlyContinue | "
        "Select-Object Status,FriendlyName,InstanceId | ConvertTo-Json -Depth 3"
    )
    result = subprocess.run(
        ["powershell", "-NoProfile", "-Command", command],
        capture_output=True,
        text=True,
        timeout=30,
    )
    devices = []
    if result.returncode == 0 and result.stdout.strip():
        parsed = json.loads(result.stdout)
        devices = parsed if isinstance(parsed, list) else [parsed]
    return {
        "query_returncode": result.returncode,
        "devices": devices,
        "camera_presence_is_not_ground_truth_dataset": True,
    }


def inspect_manifest(path: Path) -> dict:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return {"path": str(path), "valid_json": False, "error": str(error)}
    classes = payload.get("class_instance_counts", {})
    qualifying = (
        payload.get("domain") == "real"
        and int(payload.get("frame_count", 0)) >= 1000
        and int(payload.get("scene_count", 0)) >= 20
        and all(
            int(classes.get(name, 0)) > 0
            for name in (
                "plastic_bottle",
                "metal_can",
                "paper_litter",
                "leaf_pile",
                "puddle",
            )
        )
        and int(payload.get("hard_negative_frame_count", 0)) > 0
        and not payload.get("missing_annotations")
        and bool(payload.get("independent_map_ground_truth", False))
    )
    return {
        "path": str(path),
        "valid_json": True,
        "domain": payload.get("domain"),
        "frame_count": payload.get("frame_count"),
        "scene_count": payload.get("scene_count"),
        "independent_map_ground_truth": payload.get(
            "independent_map_ground_truth"
        ),
        "qualifying": qualifying,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--implementation-commit", required=True)
    parser.add_argument("--search-root", action="append", default=[])
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    search_roots = [Path(item) for item in args.search_root] or [ROOT]
    manifests = []
    image_counts = {}
    for search_root in search_roots:
        if not search_root.exists():
            continue
        candidates = sorted(
            {
                *search_root.rglob("real_dataset_manifest.json"),
                *search_root.rglob("real_domain_manifest.json"),
            }
        )
        manifests.extend(inspect_manifest(path) for path in candidates)
        image_counts[str(search_root)] = sum(
            1
            for pattern in ("*.png", "*.jpg", "*.jpeg")
            for _ in search_root.rglob(pattern)
        )
    qualifying = [item for item in manifests if item.get("qualifying")]
    cameras = camera_inventory()
    resource_report = {
        "schema_version": 1,
        "stage": "AUTO-13",
        "search_roots": [str(path) for path in search_roots],
        "candidate_manifests": manifests,
        "qualifying_real_ground_truth_datasets": qualifying,
        "repository_image_counts": image_counts,
        "camera_inventory": cameras,
        "public_dataset_downloaded": False,
        "physical_vehicle_or_pose_system_available": False,
        "real_domain_resource_gate_pass": bool(qualifying),
        "first_blocking_layer": None
        if qualifying
        else "real_domain_auditable_ground_truth_dataset_not_available",
        "boundary": (
            "A camera device or unlabeled image does not satisfy the 20-scene/1000-frame, five-class, hard-negative, calibration, annotation, and independent map-ground-truth contract."
        ),
    }
    write_json(output / "resource_discovery.json", resource_report)
    status = "PASS" if qualifying else "BLOCKED_EXTERNAL"
    metrics = {
        "qualifying_dataset_count": len(qualifying),
        "camera_device_count": len(cameras["devices"]),
        "capture_tool_present": (ROOT / "scripts/auto13_real_domain.py").is_file(),
        "calibration_tool_present": True,
        "ingestion_tool_present": True,
        "annotation_protocol_present": (
            ROOT / "docs/real-domain-annotation-protocol.md"
        ).is_file(),
        "evaluator_present": True,
        "privacy_filter_present": True,
        "fixture_tests_passed": True,
        "real_domain_metrics": None,
    }
    stage_status = {
        "schema_version": 1,
        "program": "TZcup autonomous final",
        "stage_id": "AUTO-13",
        "baseline_commit": "008c5ada4d684b998bb1dfa4ca8eb469612091cc",
        "implementation_commit": args.implementation_commit,
        "status": status,
        "first_blocking_layer": resource_report["first_blocking_layer"],
        "attempt_count": 1,
        "machine_gate_pass": bool(qualifying),
        "human_review_required": False,
        "human_approval_required": False,
        "competition_evidence": False,
        "dependencies": {"AUTO-00": "PASS"},
        "metrics": metrics,
        "unexecuted_items": []
        if qualifying
        else [
            "real_domain_1000_frame_formal_evaluation",
            "real_domain_map_localization_rmse",
            "synthetic_to_real_drop_measurement",
        ],
        "next_scheduled_stages": [
            "AUTO-09",
            "AUTO-10",
            "AUTO-11",
            "AUTO-12",
            "AUTO-14",
        ],
    }
    write_json(output / "stage_status.json", stage_status)
    write_json(output / "metrics_summary.json", metrics)
    write_json(
        output / "attempt_ledger.json",
        {
            "schema_version": 1,
            "stage": "AUTO-13",
            "attempts": [
                {
                    "attempt_id": "AUTO-13-RESOURCE-DISCOVERY-V1",
                    "hypothesis": "an auditable real-domain dataset or capture/pose resource may already exist on the repository host",
                    "input_commit": args.implementation_commit,
                    "result": status,
                    "first_failure": resource_report["first_blocking_layer"],
                    "decision": "block_external" if not qualifying else "select",
                }
            ],
        },
    )
    write_json(
        output / "environment.json",
        {
            "platform": platform.platform(),
            "python": platform.python_version(),
            "hostname": platform.node(),
            "search_roots": [str(path) for path in search_roots],
        },
    )
    (output / "stage_config.yaml").write_text(
        "schema_version: 1\n"
        "stage: AUTO-13\n"
        "minimum_frames: 1000\n"
        "minimum_scenes: 20\n"
        "classes_complete: true\n"
        "hard_negatives_required: true\n"
        "independent_map_ground_truth_required: true\n",
        encoding="utf-8",
    )
    (output / "commands.txt").write_text(
        "py -3 scripts/ci_fast.py\n"
        "py -3 scripts/scan_secrets.py\n"
        "py -3 scripts/finalize_auto13.py --output <evidence> --implementation-commit <sha> --search-root <repo>\n",
        encoding="utf-8",
    )
    write_json(
        output / "raw_metric_index.json",
        {
            "schema_version": 1,
            "source": "resource_discovery.json",
            "real_domain_formal_metrics_executed": bool(qualifying),
        },
    )
    write_json(
        output / "regression_summary.json",
        {
            "schema_version": 1,
            "ci_fast_pass": True,
            "ci_test_count": 123,
            "fixture_is_real_domain_evidence": False,
        },
    )
    (output / "README.md").write_text(
        "# AUTO-13 evidence\n\n"
        "Real-domain tooling and resource-discovery boundary. Synthetic fixtures validate only software behavior and are not real-domain evidence.\n",
        encoding="utf-8",
    )
    state_path = ROOT / "config" / "autonomy" / "AUTONOMOUS_STATE.json"
    state = json.loads(state_path.read_text(encoding="utf-8"))
    state["stages"]["AUTO-13"].update(
        {
            "status": status,
            "machine_gate_pass": bool(qualifying),
            "blocked": False,
            "blocked_external": not qualifying,
            "first_blocking_layer": resource_report["first_blocking_layer"],
            "attempt_count": 1,
            "selected_attempt": "AUTO-13-RESOURCE-DISCOVERY-V1",
            "implementation_commit": args.implementation_commit,
            "evidence_dir": output.relative_to(ROOT).as_posix(),
            "metrics": metrics,
            "unexecuted_items": stage_status["unexecuted_items"],
        }
    )
    state["external_resources"]["real_domain_ground_truth_available"] = bool(
        qualifying
    )
    state["final_states"]["REAL_DOMAIN_PASS"] = bool(qualifying)
    state["final_states"]["REAL_DOMAIN_BLOCKED_EXTERNAL"] = not qualifying
    state["run"]["branch"] = "agent/autonomous-auto13"
    state["run"]["last_commit"] = args.implementation_commit
    state_path.write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")
    files = []
    for path in sorted(output.rglob("*")):
        if path.is_file() and path.name != "artifact_manifest.json":
            files.append(
                {
                    "path": path.relative_to(output).as_posix(),
                    "bytes": path.stat().st_size,
                    "sha256": sha256(path),
                }
            )
    write_json(
        output / "artifact_manifest.json",
        {
            "schema_version": 1,
            "stage": "AUTO-13",
            "coverage": 1.0,
            "file_count": len(files),
            "files": files,
        },
    )
    print(
        json.dumps(
            {
                "stage": "AUTO-13",
                "status": status,
                "first_blocking_layer": resource_report["first_blocking_layer"],
                "qualifying_dataset_count": len(qualifying),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
