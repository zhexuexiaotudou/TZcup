#!/usr/bin/env python3
"""Create the mandatory fail-closed CRV6 final evidence bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8-sig"))


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def artifact(path: Path) -> dict:
    return {
        "path": path.as_posix(),
        "bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--online-report", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    online = load(args.online_report)
    recovery = load(
        args.root / "checkpoint_recovery/CHECKPOINT_RECOVERY_FINAL_AUDIT.json"
    )
    recon = load(
        args.root / "reconstitution/provenance/CHECKPOINT_RECONSTITUTION_REPORT.json"
    )
    parity = load(args.root / "parity/CRV6_GOLDEN_PARITY_REPORT.json")
    native = load(args.root / "native_moving/CRV6_NATIVE_MOVING_REPORT.json")
    ma1 = load(args.root / "adaptation/MA1_GATE/CRV6_NATIVE_MOVING_REPORT.json")
    checkpoint_path = args.root / "adaptation/MA1/best_coco_bbox_mAP_epoch_6.pth"
    candidate_hash = ma1["candidate_sha256"]
    historical_hash = recovery["target"]["historical_D1_B_sha256"]
    reconstituted_hash = recon["candidate_sha256"]
    if sha256(checkpoint_path) != candidate_hash:
        raise ValueError("MA1 checkpoint hash does not match its gate report")
    if online.get("candidate_sha256") != candidate_hash:
        raise ValueError("online report is not bound to the MA1 checkpoint")
    if online.get("source_commit") != args.source_commit:
        raise ValueError("online report is not bound to the requested source commit")

    x86_pass = online.get("CRV6_X86_DEV_PASS") is True
    status = {
        "schema_version": 1,
        "protocol": "CHECKPOINT-RECONSTITUTION-V6",
        "source_commit": args.source_commit,
        "historical_D1B_sha256": historical_hash,
        "HISTORICAL_D1B_CHECKPOINT_LOST": recovery[
            "HISTORICAL_D1B_CHECKPOINT_LOST"
        ],
        "reconstitution_route": recon["route"],
        "reconstituted_candidate_sha256": reconstituted_hash,
        "historical_hash_rewritten": False,
        "CRV6_GOLDEN_PARITY_PASS": parity["CRV6_GOLDEN_PARITY_PASS"],
        "native_unadapted_moving_pass": native["MOVING_NATIVE_DETECTOR_PASS"],
        "moving_adaptation_route": "MA1",
        "moving_candidate_sha256": candidate_hash,
        "MOVING_NATIVE_DETECTOR_PASS": ma1["MOVING_NATIVE_DETECTOR_PASS"],
        "CRV6_PROJECTION_TRACKER_MAP_PASS": online[
            "CRV6_PROJECTION_TRACKER_MAP_PASS"
        ],
        "CRV6_X86_DEV_PASS": x86_pass,
        "CRV6_PERFORMANCE_PASS": False,
        "MODEL_FREEZE_X86_CREATED": False,
        "G5_V2_read": False,
        "G5_V2_PASS": False,
        "ONLINE_DYNAMIC_DISCOVERY_PASS": False,
        "SPOT_CLEAN_PRODUCT_PASS": False,
        "SOAK_2H_PASS": False,
        "MCAP_REPLAY_PASS": False,
        "RELEASE_BUNDLE_PASS": False,
        "PRODUCT_X86_PERCEPTION_READY": False,
        "PRODUCT_J6_TOOLCHAIN_READY": False,
        "PRODUCT_J6_BOARD_READY": False,
        "PRODUCT_FIELD_READY": False,
        "MODEL_BLOCKED_INTERNAL": not x86_pass,
        "REAL_DOMAIN_BLOCKED_EXTERNAL": True,
        "PR_READY_ALLOWED": False,
        "deployment_allowed": False,
    }
    blockers = {
        "schema_version": 1,
        "protocol": "CHECKPOINT-RECONSTITUTION-V6",
        "source_commit": args.source_commit,
        "internal": [
            {
                "stage": "CRV6-07",
                "failed_sections": online["failed_sections"],
                "failed_gates": {
                    name: [
                        key
                        for key, value in section["gates"].items()
                        if not value
                    ]
                    for name, section in online["sections"].items()
                    if not section["pass"]
                },
            }
        ],
        "external": [
            "no authorized real RGB-D field dataset with independent map GT",
            "no verified current J6 toolchain, frozen student, or board acceptance evidence",
        ],
        "NEXT_RESEARCH_REQUIRED": True,
        "minimum_new_data_requirement": (
            "A new, non-consumed TRAIN/HOLDOUT/VAL moving dataset aligned to the "
            "real Gazebo target assets and camera rendering, with all three discrete "
            "classes, small/distant targets, hard negatives, and independently frozen "
            "selection and validation splits. The already-read 24-mission replay must "
            "remain evaluation-only."
        ),
        "minimum_new_architecture_hypothesis": (
            "Before changing architecture, test whether the measured synthetic-to-real "
            "asset/rendering shift can be closed by bounded data-domain adaptation of "
            "the same official RTMDet-s. Any architecture change requires a new protocol."
        ),
        "minimum_external_hardware_requirement": (
            "For later gates only: an authorized RGB-D device/recording with calibrated "
            "CameraInfo and TF plus independent placement/map GT; J6 board and official "
            "toolchain are additionally required for board claims."
        ),
    }
    registry = {
        "schema_version": 1,
        "protocol": "CHECKPOINT-RECONSTITUTION-V6",
        "models": [
            {
                "id": "historical_D1B",
                "sha256": historical_hash,
                "bytes_available": False,
                "status": "historical_pass_preserved_checkpoint_lost",
            },
            {
                "id": "D1B_RECON_R1",
                "sha256": reconstituted_hash,
                "historical_D1B": False,
                "status": "static_and_parity_pass_native_moving_failed",
            },
            {
                "id": "MA1",
                "sha256": candidate_hash,
                "checkpoint": artifact(checkpoint_path),
                "selection_boundary": "G7_MOVING_HOLDOUT_ONLY",
                "G7_MOVING_VAL_pass": True,
                "real_gazebo_x86_dev_pass": False,
                "frozen_for_product": False,
            },
        ],
    }
    release = {
        "schema_version": 1,
        "protocol": "CHECKPOINT-RECONSTITUTION-V6",
        "source_commit": args.source_commit,
        "release_created": False,
        "release_zip": None,
        "freeze_manifest": None,
        "reason": "CRV6_X86_DEV_PASS is false",
        "G5_V2_read": False,
        "rollback_point": args.source_commit,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    paths = {
        "status": args.output / "PERCEPTION_CRV6_FINAL_STATUS.json",
        "blockers": args.output / "PERCEPTION_CRV6_FINAL_BLOCKERS.json",
        "registry": args.output / "PERCEPTION_CRV6_MODEL_REGISTRY.json",
        "release": args.output / "PERCEPTION_CRV6_RELEASE_MANIFEST.json",
    }
    write_json(paths["status"], status)
    write_json(paths["blockers"], blockers)
    write_json(paths["registry"], registry)
    write_json(paths["release"], release)
    notices = """# PERCEPTION CRV6 THIRD-PARTY NOTICES

No CRV6 release bundle was created. The development/evaluation path uses the
repository's audited MMDetection 3.3.0 / official RTMDet-s dependency and
PyTorch CUDA environment. Existing dependency registry and licenses remain the
authority; this blocked result does not authorize model redistribution.
"""
    notices_path = args.output / "PERCEPTION_CRV6_THIRD_PARTY_NOTICES.md"
    notices_path.write_text(notices, encoding="utf-8")
    summary = f"""# CHECKPOINT RECONSTITUTION V6 REPORT

Historical D1-B `{historical_hash}` remains lost. R1 created a
hash-new candidate and passed static regression plus golden parity. The bounded
MA1 adaptation `{candidate_hash}` passed independent G7-MOVING validation but
failed the physically consistent 24-mission Gazebo online development gate.

CRV6-07 failed sections: {', '.join(online['failed_sections'])}.
Projection passed; detector/map domain transfer, required coverage, and the
strict G6 Area integration thresholds did not all pass. The workflow therefore
stopped before performance, freeze, G5_V2, 30-seed, cleaning, soak, replay,
release, J6 student, field validation, merge, or deployment.
"""
    summary_path = args.output / "CHECKPOINT_RECONSTITUTION_V6_REPORT.md"
    summary_path.write_text(summary, encoding="utf-8")
    evidence_sources = [
        args.root / "checkpoint_recovery/CHECKPOINT_RECOVERY_FINAL_AUDIT.json",
        args.root / "reconstitution/provenance/CHECKPOINT_RECONSTITUTION_REPORT.json",
        args.root / "static_regression/CRV6_STATIC_REGRESSION_REPORT.json",
        args.root / "parity/CRV6_GOLDEN_PARITY_REPORT.json",
        args.root / "native_moving/CRV6_NATIVE_MOVING_REPORT.json",
        args.root / "adaptation/MA1_GATE/CRV6_NATIVE_MOVING_REPORT.json",
        args.online_report,
    ]
    generated = [*paths.values(), notices_path, summary_path]
    index_lines = [
        "# PERCEPTION CRV6 EVIDENCE INDEX",
        "",
        f"Source commit: `{args.source_commit}`",
        "",
        "| Artifact | Bytes | SHA-256 |",
        "|---|---:|---|",
    ]
    for path in [*evidence_sources, *generated]:
        item = artifact(path)
        index_lines.append(
            f"| `{item['path']}` | {item['bytes']} | `{item['sha256']}` |"
        )
    index_path = args.output / "PERCEPTION_CRV6_EVIDENCE_INDEX.md"
    index_path.write_text("\n".join(index_lines) + "\n", encoding="utf-8")
    print(json.dumps({"output": args.output.as_posix(), "x86_pass": x86_pass, "files": 7}, indent=2))
    return 0 if x86_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
