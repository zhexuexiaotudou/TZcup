#!/usr/bin/env python3
"""Create the fail-closed DetectorDataRecoveryV4 final evidence package."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes((json.dumps(payload, indent=2) + "\n").encode("utf-8"))


def write_text(path: Path, payload: str) -> None:
    """Write deterministic UTF-8/LF bytes on Windows and Linux."""
    path.write_bytes(payload.replace("\r\n", "\n").encode("utf-8"))


def record(path: Path, *, logical_path: str | None = None) -> dict:
    return {
        "path": logical_path or path.as_posix(),
        "bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def build_status(d1: dict, online: dict, performance: dict, j6: dict, board: dict, field: dict, source: dict) -> dict:
    return {
        "schema_version": 1,
        "stage": "DDRV4-14-FINAL-STATUS",
        "source": source,
        "DDRV4_D1_PASS": bool(d1["DDRV4_D1_PASS"]),
        "DDRV4_D2_PASS": False,
        "DDRV4_D2_STATE": "NOT_EXECUTED_D1_STATIC_PASSED",
        "DDRV4_D3_PASS": False,
        "DDRV4_D3_STATE": "NOT_EXECUTED_D1_STATIC_PASSED",
        "DDRV4_X86_DEV_PASS": bool(online["DDRV4_X86_DEV_PASS"]),
        "DDRV4_PRODUCT_PERFORMANCE_PASS": bool(performance["pass"]),
        "G5_V2_PASS": False,
        "ONLINE_DYNAMIC_DISCOVERY_PASS": False,
        "DYNAMIC_TRASH_MAP_PASS": False,
        "SPOT_CLEAN_PRODUCT_PASS": False,
        "POST_CLEAN_VERIFICATION_PASS": False,
        "SOAK_2H_PASS": False,
        "MCAP_REPLAY_PASS": False,
        "RELEASE_BUNDLE_PASS": False,
        "PRODUCT_X86_PERCEPTION_READY": False,
        "PRODUCT_J6_TOOLCHAIN_READY": bool(j6["PRODUCT_J6_TOOLCHAIN_READY"]),
        "PRODUCT_J6_BOARD_READY": bool(board["PRODUCT_J6_BOARD_READY"]),
        "PRODUCT_FIELD_READY": bool(field["PRODUCT_FIELD_READY"]),
        "COMPETITION_PERCEPTION_PASS": False,
        "MODEL_FREEZE_X86_CREATED": False,
        "G5_SEALED_FINAL_read": False,
        "G5_V2_SEALED_FINAL_read": False,
        "NEAT_FREAK_SYNC_STATUS": "NOT_RUN_PRODUCTION_GATE_NOT_REACHED",
        "PR_90_READY_ALLOWED": False,
        "merge_allowed": False,
        "deployment_executed": False,
        "blocked_follow_on_gates": [
            "DDRV4-07 x86 freeze and G5_V2",
            "DDRV4-08 30-seed moving-camera",
            "DDRV4-09 spot-clean and post-clean",
            "DDRV4-10 dynamic-map formal acceptance",
            "DDRV4-11 soak, replay and x86 release",
            "DDRV4-12 J6 student, PTQ, compile and board",
            "DDRV4-13 field acceptance",
            "DDRV4-14 production knowledge synchronization, Ready, merge and deploy",
        ],
        "claim_boundary": (
            "D1 passed the one-time static G7 VAL gate, but the existing 24-mission "
            "moving-camera product regression failed. No freeze or later product gate "
            "is unlocked, and no deployment-ready claim is permitted."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--performance", type=Path, required=True)
    parser.add_argument("--resource-scan", type=Path, required=True)
    parser.add_argument("--external-preflight", type=Path, required=True)
    parser.add_argument("--d1-a-train-report", type=Path, required=True)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--source-tree", required=True)
    args = parser.parse_args()

    root = args.root
    final = root / "final"
    final.mkdir(parents=True, exist_ok=True)
    d1_path = root / "d1/D1_STATIC_SUMMARY.json"
    online_path = root / "online_dev/DDRV4_ONLINE_DEV_SUMMARY.json"
    d1 = load(d1_path)
    online = load(online_path)
    performance = load(args.performance)
    j6_path = args.resource_scan / "j6_inventory.json"
    sensor_path = args.resource_scan / "real_sensor_inventory.json"
    board_path = args.external_preflight / "j6_board/J6_BOARD_STATUS.json"
    field_path = args.external_preflight / "field/FIELD_RESOURCE_AND_SOFTWARE_STATUS.json"
    lock_path = args.external_preflight / "j6_toolchain/J6_TOOLCHAIN_LOCK.json"
    j6, board, field = load(j6_path), load(board_path), load(field_path)
    source = {"commit": args.source_commit, "tree": args.source_tree, "worktree_isolated": True}

    status = build_status(d1, online, performance, j6, board, field, source)
    status_path = final / "PERCEPTION_DDRV4_FINAL_STATUS.json"
    write_json(status_path, status)

    compact_copies = {
        root / "performance/DDRV4_PRODUCT_PERFORMANCE.json": args.performance,
        root / "j6/J6_INVENTORY.json": j6_path,
        root / "j6/J6_TOOLCHAIN_LOCK.json": lock_path,
        root / "j6/J6_BOARD_STATUS.json": board_path,
        root / "field/REAL_SENSOR_INVENTORY.json": sensor_path,
        root / "field/FIELD_RESOURCE_AND_SOFTWARE_STATUS.json": field_path,
    }
    for destination, source_path in compact_copies.items():
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source_path, destination)

    blockers = {
        "schema_version": 1,
        "stage": "DDRV4-14-FINAL-BLOCKERS",
        "primary_blocker": "MODEL_BLOCKED_INTERNAL_ONLINE_DOMAIN",
        "internal_blockers": [
            {
                "id": "moving_camera_eventual_recall",
                "gate": 0.95,
                "actual": online["base_regression"]["eventual_discrete_recall"],
                "eligible_targets": online["base_regression"]["eligible_discrete_targets"],
            },
            {
                "id": "moving_camera_correct_class_recall",
                "gate": 0.95,
                "actual": online["base_regression"]["eventual_correct_class_recall_all_targets"],
            },
            {
                "id": "moving_camera_small_recall",
                "gate": 0.90,
                "actual": online["base_regression"]["small_object_eventual_recall"],
            },
            {
                "id": "product_target_precision",
                "gate": 0.95,
                "actual": online["product_map"]["product_target_precision"],
            },
            {
                "id": "formal_special_coverage_missing",
                "required": ["behind_vehicle_fov_entry", "turning", "occlusion", "reflection"],
                "actual_missing": online["base_regression"]["missing_coverage"],
            },
            {
                "id": "strict_performance_throughput",
                "gate_hz": performance["thresholds"]["effective_hz"],
                "actual_hz": performance["metrics"]["effective_hz"],
                "p95_ms": performance["metrics"]["end_to_end_p95_ms"],
                "drop_rate": performance["metrics"]["drop_rate"],
            },
        ],
        "minimum_next_data_requirement": {
            "authorization": "new development protocol required; D2/D3 are not unlocked by a D1 static pass",
            "training": (
                "A new G7-derived moving-camera TRAIN pack matching product RGB/TF/depth rendering, "
                "with at least 20 missions and balanced actionable encounters for all three discrete classes."
            ),
            "independent_evaluation": (
                "A disjoint moving-camera development evaluation of at least 20 missions covering "
                "behind-FOV entry, turns, occlusion, reflections, negative-only motion and <18 px targets."
            ),
            "sealed_data": "G5 and G5_V2 remain forbidden for recovery or selection.",
        },
        "minimum_next_architecture_hypothesis": {
            "first": (
                "Retain RTMDet-s capacity but recover the train-to-online input distribution and calibrate "
                "scores only on a moving-camera holdout; static G7 accuracy shows capacity is not the first blocker."
            ),
            "fallback_if_new_data_fails": (
                "Authorize a new protocol for a P2/small-object route or temporal proposal-plus-classifier "
                "with tracker-aware calibration; do not silently reinterpret D2/D3."
            ),
        },
        "minimum_external_hardware_requirement": {
            "x86_internal_blocker": "none; the current failure is model/data quality, not missing x86 hardware",
            "j6": "one identified Horizon J6E or J6M board plus a target-matched current OpenExplorer toolchain",
            "field": "one synchronized RGB-D device on the moving platform plus independent map/placement ground truth",
        },
        "external_blockers": {
            "J6_board": bool(board["BLOCKED_EXTERNAL_J6_BOARD"]),
            "real_RGBD_and_independent_GT": bool(field["REAL_DOMAIN_BLOCKED_EXTERNAL"]),
            "verified_official_sanitation_competition_rule": False,
        },
    }
    blockers_path = final / "PERCEPTION_DDRV4_FINAL_BLOCKERS.json"
    write_json(blockers_path, blockers)

    registry = {
        "schema_version": 1,
        "selected_development_model": "D1-B",
        "frozen_product_model": None,
        "freeze_id": None,
        "models": [
            {"route": "D1-A", "status": "STATIC_PASS_NOT_SELECTED", "checkpoint_sha256": d1["training"]["D1_A_checkpoint_sha256"]},
            {
                "route": "D1-B",
                "status": "STATIC_PASS_SELECTED_ONLINE_FAIL",
                "architecture": d1["architecture"],
                "threshold": d1["selection"]["threshold"],
                "checkpoint_sha256": d1["training"]["D1_B_checkpoint_sha256"],
                "selection_data": d1["selection"]["data"],
                "G5_read": False,
                "G5_V2_read": False,
            },
        ],
    }
    registry_path = final / "PERCEPTION_DDRV4_MODEL_REGISTRY.json"
    write_json(registry_path, registry)

    release = {
        "schema_version": 1,
        "release_created": False,
        "release_path": None,
        "release_sha256": None,
        "deployed": False,
        "deployed_commit": None,
        "rollback_point": None,
        "reason": "DDRV4_X86_DEV_PASS=false; freeze, G5_V2 and downstream release gates are locked",
        "source": source,
    }
    release_path = final / "PERCEPTION_DDRV4_RELEASE_MANIFEST.json"
    write_json(release_path, release)

    competition = {
        "schema_version": 1,
        "stage": "DDRV4-COMPETITION-GATE-MAPPING",
        "official_rule_source_verified": False,
        "competition_perception_pass": False,
        "mapping_status": "BLOCKED_UNVERIFIED_OFFICIAL_SANITATION_RULE_DEFINITION",
        "audit": {
            "searched_on_utc_date": "2026-08-12",
            "official_site_checked": "https://www.tianzhibei.com/",
            "site_identity": "Tianzhibei artificial-intelligence competition platform",
            "sanitation_vehicle_rule_found": False,
            "decision": "No internal DDRV4 threshold is promoted to an official competition requirement.",
        },
        "metrics": [],
        "next_required_external_input": (
            "Primary sanitation-task rules defining scoring unit, sequences, accuracy formula and deployment target."
        ),
    }
    competition_path = final / "COMPETITION_GATE_MAPPING.json"
    write_json(competition_path, competition)

    notices_path = final / "PERCEPTION_DDRV4_THIRD_PARTY_NOTICES.md"
    write_text(
        notices_path,
        f"""# PERCEPTION DDRV4 third-party notices

No DDRV4 product release bundle was created.

## MMDetection / RTMDet

- Upstream: OpenMMLab MMDetection
- Version: `3.3.0`
- Code license: Apache-2.0
- Architecture: official RTMDet-s
- Official initialization checkpoint SHA256: `{load(args.d1_a_train_report)['initial_checkpoint_sha256']}`
- Local modification: project dataset/config integration and existing CUDA NMS compatibility patch
- Shipped in product: no
- Redistribution: not attempted; exact release packaging remains gated

## ONNX Runtime GPU

- Version: `1.20.2`
- Role: CUDA execution of the two existing Area ONNX heads
- Shipped in product: no

## G7

- Origin: project-generated Gazebo detector development data
- External public dataset ingested: no
- G5/G5_V2 used for training or selection: no
""",
    )

    report_path = final / "DETECTOR_DATA_RECOVERY_V4_REPORT.md"
    base = online["base_regression"]
    perf = performance["metrics"]
    write_text(
        report_path,
        f"""# Detector Data Recovery V4 final report

## Outcome

DDRV4 recovered the detector static gate but did not recover the online product gate. D1-B passed one-time G7 VAL, while the 24-mission, 2160-frame moving-camera regression achieved only `{base['eventual_discrete_recall']:.4f}` eventual recall and `{online['product_map']['product_target_precision']:.4f}` product-target precision. No x86 freeze, G5_V2 access, release, merge or deployment was authorized.

## Completed evidence

- G7 detector development pack: 3200 frames, 13 worlds, 2810 instances, generator and independent reread QA passed.
- D1 static: recall/precision/macro-F1 `{d1['one_time_G7_VAL']['recall']:.4f}/{d1['one_time_G7_VAL']['precision']:.4f}/{d1['one_time_G7_VAL']['macro_f1']:.4f}`; D1-B selected at threshold `{d1['selection']['threshold']}`.
- Online compatibility regression: `{base['missions']}` missions, `{base['frames']}` frames, metal recall `{base['per_class_eventual_detection_recall']['metal_can']:.4f}`, small recall `{base['small_object_eventual_recall']:.4f}`.
- Product performance: 300 submitted/processed frames, `{perf['effective_hz']:.4f}` Hz, p95 `{perf['end_to_end_p95_ms']:.2f}` ms, drop rate `{perf['drop_rate']:.4f}`. The strict 10 Hz gate failed.
- J6/field preflight: current official documentation was rechecked; no frozen student, installed current toolchain, board, RGB-D recording or independent map GT exists. Field software preparation is present.

## Locked work

D2/D3 were not executed because the authorized protocol sends a static D1 pass directly to DDRV4-06. Online failure blocks freeze, G5_V2, 30-seed dynamic-map/spot-clean runs, post-clean verification, soak, replay, release, J6 training/PTQ/compile and field acceptance. The neat-freak production sync gate was not run because production verification was never reached.

PR #90 remains Draft. Historical A1/A2/A3, X1/X2/X3, MRV2, OPR-A/B/C and the original G5 failure remain preserved in its body.
""",
    )

    evidence_sources = [
        (d1_path, "d1/D1_STATIC_SUMMARY.json"),
        (online_path, "online_dev/DDRV4_ONLINE_DEV_SUMMARY.json"),
        (root / "performance/DDRV4_PRODUCT_PERFORMANCE.json", "performance/DDRV4_PRODUCT_PERFORMANCE.json"),
        (root / "j6/J6_INVENTORY.json", "j6/J6_INVENTORY.json"),
        (root / "j6/J6_TOOLCHAIN_LOCK.json", "j6/J6_TOOLCHAIN_LOCK.json"),
        (root / "j6/J6_BOARD_STATUS.json", "j6/J6_BOARD_STATUS.json"),
        (root / "field/REAL_SENSOR_INVENTORY.json", "field/REAL_SENSOR_INVENTORY.json"),
        (root / "field/FIELD_RESOURCE_AND_SOFTWARE_STATUS.json", "field/FIELD_RESOURCE_AND_SOFTWARE_STATUS.json"),
        (status_path, "final/PERCEPTION_DDRV4_FINAL_STATUS.json"),
        (blockers_path, "final/PERCEPTION_DDRV4_FINAL_BLOCKERS.json"),
        (registry_path, "final/PERCEPTION_DDRV4_MODEL_REGISTRY.json"),
        (release_path, "final/PERCEPTION_DDRV4_RELEASE_MANIFEST.json"),
        (competition_path, "final/COMPETITION_GATE_MAPPING.json"),
        (notices_path, "final/PERCEPTION_DDRV4_THIRD_PARTY_NOTICES.md"),
        (report_path, "final/DETECTOR_DATA_RECOVERY_V4_REPORT.md"),
    ]
    index_path = final / "PERCEPTION_DDRV4_EVIDENCE_INDEX.md"
    lines = ["# PERCEPTION DDRV4 evidence index", "", f"Source commit: `{args.source_commit}`", ""]
    for path, logical in evidence_sources:
        item = record(path, logical_path=logical)
        lines.append(f"- `{item['path']}` - {item['bytes']} bytes - `{item['sha256']}`")
    write_text(index_path, "\n".join(lines) + "\n")
    manifest_path = root / "artifact_manifest.json"
    manifest_records = []
    for path in sorted(root.rglob("*")):
        if not path.is_file() or path == manifest_path or path.suffix not in (".json", ".md"):
            continue
        manifest_records.append(record(path, logical_path=path.relative_to(root).as_posix()))
    write_json(
        manifest_path,
        {
            "schema_version": 1,
            "stage": "DDRV4-EVIDENCE-MANIFEST",
            "root": ".",
            "files": manifest_records,
        },
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
