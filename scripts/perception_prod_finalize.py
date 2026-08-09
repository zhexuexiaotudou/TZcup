#!/usr/bin/env python3
"""Build the PERCEPTION-PROD-12 fail-closed final evidence package."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def git_head() -> str:
    return subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, check=True,
        capture_output=True, text=True,
    ).stdout.strip()


def build_status(source_commit: str) -> dict:
    return {
        "schema_version": 1,
        "stage": "PERCEPTION-PROD-12",
        "evidence_source_commit": source_commit,
        "statuses": {
            "ONLINE_DYNAMIC_DISCOVERY_PASS": False,
            "DYNAMIC_TRASH_MAP_PASS": False,
            "POST_CLEAN_VERIFICATION_PASS": False,
            "SPOT_CLEAN_PRODUCT_PASS": False,
            "PRODUCT_X86_PERCEPTION_READY": False,
            "PRODUCT_J6_TOOLCHAIN_READY": False,
            "PRODUCT_J6_BOARD_READY": False,
            "PRODUCT_FIELD_READY": False,
            "COMPETITION_PERCEPTION_PASS": False,
        },
        "model_route_status": {
            "X1": "FAILED_STATIC_FULL_PIPELINE",
            "X2": "BLOCKED_EXTERNAL_NETWORK_ASSET",
            "X3": "FAILED_STATIC_FULL_PIPELINE",
            "routes_exhausted": True,
            "MODEL_BLOCKED_INTERNAL": True,
        },
        "sealed_data": {
            "G5_SEALED_FINAL_read": False,
            "legacy_G4_D6_read": False,
            "freeze_performed": False,
        },
        "downstream_gates": {
            "G5_final": "not_run_no_frozen_candidate",
            "moving_camera_30_seed": "not_run_no_frozen_candidate",
            "dynamic_trash_map": "not_run_no_frozen_candidate",
            "spot_clean_post_clean": "not_run_no_frozen_candidate",
            "x86_performance_2h_soak_replay_release": "not_run_no_frozen_candidate",
            "J6_student_PTQ_compile": "not_run_no_frozen_x86_teacher",
            "J6_board": "blocked_external_no_board",
            "real_RGBD_field": "blocked_external_no_resources",
        },
        "product_deployment_complete": False,
        "claim_boundary": (
            "All three authorized model routes were exhausted without a static-pass "
            "candidate. No later gate is inferred from software-only evidence."
        ),
    }


def build_blockers(x3: dict, j6: dict, board: dict, field: dict) -> dict:
    return {
        "schema_version": 1,
        "stage": "PERCEPTION-PROD-12",
        "blockers": [
            {
                "id": "X86_MODEL_STATIC_GATE",
                "classification": "internal_model",
                "blocking": [
                    "PRODUCT_X86_PERCEPTION_READY",
                    "G5/freeze/moving-camera/map/spot-clean/soak/release",
                ],
                "evidence": "x3/X3_STATIC_FAILURE.json",
                "facts": {
                    "routes_exhausted": x3["routes_exhausted"],
                    "failed_subgates": x3["failed_subgates"],
                },
            },
            {
                "id": "X2_OFFICIAL_CHECKPOINT_BYTES",
                "classification": "external_network_asset",
                "blocking": ["ONLINE-X2 evaluation"],
                "evidence": "x2/X2_EXTERNAL_ASSET_BLOCKED.json",
            },
            {
                "id": "J6_STUDENT_AND_TOOLCHAIN",
                "classification": "internal_dependency",
                "blocking": ["PRODUCT_J6_TOOLCHAIN_READY"],
                "evidence": "j6_toolchain/J6_TOOLCHAIN_LOCK.json",
                "facts": {
                    "frozen_j6_student_available": j6["frozen_j6_student_available"],
                    "version_compatibility_resolved": j6["version_compatibility_resolved"],
                    "installation_root": j6["installation_root"],
                },
            },
            {
                "id": "PHYSICAL_J6_BOARD",
                "classification": "external_resource",
                "blocking": ["PRODUCT_J6_BOARD_READY"],
                "evidence": "j6_board/J6_BOARD_STATUS.json",
                "facts": {"board_device_count": board["board_device_count"]},
            },
            {
                "id": "REAL_RGBD_AND_INDEPENDENT_GT",
                "classification": "external_resource",
                "blocking": ["PRODUCT_FIELD_READY"],
                "evidence": "field/FIELD_RESOURCE_AND_SOFTWARE_STATUS.json",
                "facts": {
                    "software_preparation_complete": field["software_preparation_complete"],
                    "rgbd_device_present": field["resource_scan"]["rgbd_device_present"],
                    "qualifying_frames": field["actual_qualifying_frames"],
                    "independent_map_gt_present": field["resource_scan"]["independent_map_gt_present"],
                },
            },
        ],
        "external_only": False,
        "product_deployment_complete": False,
    }


def evidence_index(evidence_root: Path, source_commit: str) -> str:
    relative_paths = [
        "prod00_resources/artifact_manifest.json",
        "x1/X1_STATIC_FAILURE.json",
        "x2/X2_EXTERNAL_ASSET_BLOCKED.json",
        "x3/X3_HYPOTHESIS.json",
        "x3/X3_TRAIN_SUMMARY.json",
        "x3/x3_full_static_report.json",
        "x3/X3_STATIC_FAILURE.json",
        "j6_toolchain/J6_TOOLCHAIN_LOCK.json",
        "j6_board/J6_BOARD_STATUS.json",
        "field/FIELD_RESOURCE_AND_SOFTWARE_STATUS.json",
    ]
    lines = [
        "# PERCEPTION PRODUCT FINAL EVIDENCE INDEX", "",
        f"Evidence source commit: `{source_commit}`", "",
        "| Stage | SHA-256 | Bytes | Evidence |", "|---|---|---:|---|",
    ]
    for relative in relative_paths:
        path = evidence_root / relative
        if not path.is_file():
            raise FileNotFoundError(path)
        lines.append(
            f"| {relative.split('/', 1)[0]} | `{sha256(path)}` | {path.stat().st_size} | `{relative}` |"
        )
    lines.extend(
        [
            "", "G5 sealed final and legacy G4 D6 were not read. Large model and raw-data "
            "artifacts remain outside Git; only compact evidence is indexed here.",
        ]
    )
    return "\n".join(lines) + "\n"


def model_registry(x3: dict, source_commit: str) -> dict:
    return {
        "schema_version": 1,
        "evidence_source_commit": source_commit,
        "selected_product_model": None,
        "models": [
            {
                "route": "X1", "model": "FCOS-R50 proposal + crop classifier",
                "status": "FAILED_STATIC_FULL_PIPELINE",
                "checkpoint_sha256": "a5884ac9bfa4e89f2ae8a25f4cae0521e263dd951ef895fa1185f013b2f04ee5",
                "shipped": False,
            },
            {
                "route": "X2", "model": "Grounding DINO + crop classifier",
                "status": "BLOCKED_EXTERNAL_NETWORK_ASSET",
                "checkpoint_sha256": None, "shipped": False,
            },
            {
                "route": "X3", "model": "Torchvision FCOS-R50 direct three-class",
                "status": x3["decision"],
                "checkpoint_sha256": "02869d3677a999a0d8cd0a73114a60fbc803c447717d129313bcf3dbfe68507b",
                "checkpoint_repository_tracked": False, "shipped": False,
            },
        ],
        "PRODUCT_X86_PERCEPTION_READY": False,
        "release_model_count": 0,
    }


def third_party_notices() -> str:
    return """# PERCEPTION PRODUCT FINAL THIRD-PARTY NOTICES

No third-party model checkpoint is shipped because no product model qualified.

| Dependency | Pinned/audited version | License | Product disposition |
|---|---|---|---|
| Torchvision FCOS ResNet50-FPN | COCO_V1, SHA-256 `99b0c9b7...b9e7` | BSD-3-Clause code; reference-weight dataset terms also apply | Used for X1/X3 development only; trained checkpoints external and not shipped |
| Grounding DINO | commit `856dde20aee659246248e20734ef9ba5214f5e44` | Apache-2.0 | X2 checkpoint unavailable; not loaded or shipped |
| SAM 2 | commit `2b90b9f5ceec907a1c18123530e92e794ad901a4` | Apache-2.0 | Reference tooling only; no checkpoint shipped |
| YOLO-World | commit `4f70adbaacf5685bd9ec5bea85f1f91057f6fc0b` | GPL-3.0 | Rejected as the product X3 route; not shipped |

Exact dependency commit and URL records are in
`prod00_resources/reference_dependency_inventory.json`. Runtime redistribution requires a
fresh legal review if a future qualified model changes this no-shipment state.
"""


def release_manifest(source_commit: str) -> dict:
    return {
        "schema_version": 1,
        "stage": "PERCEPTION-PROD-12",
        "evidence_source_commit": source_commit,
        "release_id": None,
        "selected_model": None,
        "container_digest": None,
        "deployment_target": None,
        "deployed_commit": None,
        "rollback_point": source_commit,
        "release_ready": False,
        "release_blocked_at": "static_x86_model_qualification",
        "SBOM_generated": False,
        "signature_generated": False,
        "production_verification_run": False,
        "truth_boundary": "A release manifest records the blocked state; it is not a product release.",
    }


def write_json(path: Path, payload: dict):
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", type=Path, required=True)
    args = parser.parse_args()
    root = args.evidence_root.resolve()
    source = git_head()
    x3 = load(root / "x3/X3_STATIC_FAILURE.json")
    j6 = load(root / "j6_toolchain/J6_TOOLCHAIN_LOCK.json")
    board = load(root / "j6_board/J6_BOARD_STATUS.json")
    field = load(root / "field/FIELD_RESOURCE_AND_SOFTWARE_STATUS.json")
    write_json(root / "PERCEPTION_PRODUCT_FINAL_STATUS.json", build_status(source))
    write_json(root / "PERCEPTION_PRODUCT_FINAL_BLOCKERS.json", build_blockers(x3, j6, board, field))
    (root / "PERCEPTION_PRODUCT_FINAL_EVIDENCE_INDEX.md").write_text(
        evidence_index(root, source), encoding="utf-8"
    )
    write_json(root / "PERCEPTION_PRODUCT_FINAL_MODEL_REGISTRY.json", model_registry(x3, source))
    (root / "PERCEPTION_PRODUCT_FINAL_THIRD_PARTY_NOTICES.md").write_text(
        third_party_notices(), encoding="utf-8"
    )
    write_json(root / "PERCEPTION_PRODUCT_RELEASE_MANIFEST.json", release_manifest(source))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
