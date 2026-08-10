#!/usr/bin/env python3
"""Generate OPRV3-00/01 gate provenance and analytic geometry evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
LEARNING = ROOT / "starter_ws/src/sanitation_learning"
if str(LEARNING) not in sys.path:
    sys.path.insert(0, str(LEARNING))

from sanitation_learning.oprv3_geometry import derive_product_geometry  # noqa: E402
from sanitation_learning.oprv3_online import validate_gate_provenance  # noqa: E402


DEFAULT_MRV2 = ROOT / "artifacts/model_recovery_v2_20260810T004459Z"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def git(*args: str) -> str:
    completed = subprocess.run(
        ["git", *args], cwd=ROOT, text=True, capture_output=True, check=True,
    )
    return completed.stdout.strip()


def prior_truth(mrv2_root: Path) -> tuple[dict, dict, list[dict]]:
    final_status_path = mrv2_root / "final/PERCEPTION_MRV2_FINAL_STATUS.json"
    static_path = mrv2_root / "mrv2_c/MRV2_C_FULL_STATIC.json"
    if not final_status_path.is_file() or not static_path.is_file():
        raise FileNotFoundError("tracked MRV2 final status and MRV2-C static report are required")
    status = load_json(final_status_path)
    static = load_json(static_path)
    return status, static, [
        {"path": final_status_path.relative_to(ROOT).as_posix(), "sha256": sha256(final_status_path)},
        {"path": static_path.relative_to(ROOT).as_posix(), "sha256": sha256(static_path)},
    ]


def build_baseline(source_commit: str, status: dict, static: dict, prior_files: list[dict]) -> dict:
    val = static["splits"]["VAL"]
    cross = static["cross_world_aggregate"]
    d4 = static["splits"]["D4"]
    return {
        "schema_version": 1,
        "protocol": "ONLINE-FIRST-PRODUCT-RECOVERY-V3",
        "stage": "OPRV3-00",
        "source_commit": source_commit,
        "historical_truth_immutable": True,
        "historical_routes": {
            "X1": "FAILED_STATIC_FULL_PIPELINE",
            "X2": "HISTORICALLY_NETWORK_BLOCKED_THEN_GROUNDING_DINO_REFERENCE_FAILED",
            "X3": "FAILED_STATIC_FULL_PIPELINE",
            "MRV2_A": "FAILED_STATIC_FULL_PIPELINE",
            "MRV2_B": "FAILED_STATIC_FULL_PIPELINE",
            "MRV2_C": "FAILED_STATIC_FULL_PIPELINE",
        },
        "historical_status": {
            "MRV2_X86_STATIC_PASS": False,
            "MODEL_BLOCKED_INTERNAL": True,
            "MODEL_FREEZE_X86_created": False,
            "G5_SEALED_FINAL_read": False,
            "legacy_G4_D6_read": False,
        },
        "mrv2_c_static_metrics": {
            "VAL_candidate_recall": val["candidate"]["all_gt_candidate_recall"],
            "VAL_macro_f1": val["discrete"]["macro_f1"],
            "VAL_small_object_recall": val["discrete"]["small_object_recall"],
            "VAL_metal_can_recall": val["discrete"]["per_class"]["metal_can"]["recall"],
            "VAL_false_candidates_per_min": val["candidate"]["false_candidates_per_min"],
            "cross_world_small_object_recall": cross["discrete"]["small_object_recall"],
            "D1_D2_D3_D4_metal_can_recall": [
                static["splits"][name]["discrete"]["per_class"]["metal_can"]["recall"]
                for name in ("D1", "D2", "D3", "D4")
            ],
            "VAL_boundary_f1": val["area"]["boundary_f1"],
            "D4_boundary_f1": d4["area"]["boundary_f1"],
            "D4_negative_area_fp_per_frame": d4["area"]["negative_area_fp_per_frame"],
        },
        "new_status": {
            "OPRV3_AUTHORIZED": True,
            "OPRV3_X86_DEV_PASS": False,
            "ONLINE_DYNAMIC_DISCOVERY_PASS": False,
            "PRODUCT_X86_PERCEPTION_READY": False,
            "COMPETITION_PERCEPTION_PASS": False,
        },
        "prior_evidence_files": prior_files,
        "prior_final_status_sha_matches_loaded": status.get("MODEL_BLOCKED_INTERNAL") is True,
        "claim_boundary": "OPRV3 changes the product measurement question; it does not rewrite any historical static result or pass a downstream gate.",
    }


def competition_mapping(provenance: dict) -> dict:
    audit = provenance["competition_material_audit"]
    return {
        "schema_version": 1,
        "stage": "OPRV3-00",
        "official_rule_source_verified": False,
        "competition_perception_pass": False,
        "mapping_status": "BLOCKED_UNVERIFIED_OFFICIAL_RULE_DEFINITION",
        "repository_internal_thresholds_promoted_to_official": False,
        "audit": audit,
        "metrics": [],
        "next_required_external_input": "A primary competition rule document that defines the sanitation task, scoring unit, dataset/sequence conditions, and accuracy formula.",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--mrv2-root", type=Path, default=DEFAULT_MRV2)
    args = parser.parse_args()
    output = args.output_root.resolve()
    source_commit = git("rev-parse", "HEAD")
    source_tree = git("rev-parse", "HEAD^{tree}")
    gate_config_path = ROOT / "starter_ws/src/sanitation_learning/config/oprv3_gate_provenance.yaml"
    provenance = yaml.safe_load(gate_config_path.read_text(encoding="utf-8"))
    validate_gate_provenance(provenance)
    status, static, prior_files = prior_truth(args.mrv2_root.resolve())
    geometry = derive_product_geometry(ROOT)
    if not geometry["all_classes_have_nonempty_window"]:
        raise RuntimeError("frozen camera/action geometry has an empty class window")

    baseline_path = output / "baseline/OPRV3_BASELINE.json"
    provenance_path = output / "gate_provenance/GATE_PROVENANCE.json"
    competition_path = output / "gate_provenance/COMPETITION_GATE_MAPPING.json"
    geometry_path = output / "geometry/CAMERA_PRODUCT_GEOMETRY_AUDIT.json"
    write_json(baseline_path, build_baseline(source_commit, status, static, prior_files))
    write_json(provenance_path, {
        **provenance,
        "generated_from": gate_config_path.relative_to(ROOT).as_posix(),
        "generated_from_sha256": sha256(gate_config_path),
        "source_commit": source_commit,
        "source_tree": source_tree,
    })
    write_json(competition_path, competition_mapping(provenance))
    write_json(geometry_path, {**geometry, "source_commit": source_commit, "source_tree": source_tree})

    evidence_files = [baseline_path, provenance_path, competition_path, geometry_path]
    manifest_path = output / "OPRV3_00_01_EVIDENCE_MANIFEST.json"
    manifest = {
        "schema_version": 1,
        "stages": ["OPRV3-00", "OPRV3-01_ANALYTIC_ONLY"],
        "source_commit": source_commit,
        "source_tree": source_tree,
        "command": [sys.executable, str(Path(__file__).relative_to(ROOT)), "--output-root", str(args.output_root)],
        "exit_code": 0,
        "environment": {"platform": platform.platform(), "python": platform.python_version()},
        "gt_boundary": "No production target is created here. Geometry uses repository configuration and target dimensions only; prior GT metrics are read solely as immutable audit history.",
        "empirical_moving_camera_probe_executed": False,
        "OPRV3_01_pass": False,
        "OPRV3_01_blocked_by": "PIXEL_DISTANCE_EMPIRICAL_REPORT.json requires a real moving Gazebo probe with at least 20 targets per class.",
        "files": [
            {"path": path.relative_to(output).as_posix(), "bytes": path.stat().st_size, "sha256": sha256(path)}
            for path in evidence_files
        ],
    }
    write_json(manifest_path, manifest)
    print(json.dumps({
        "output_root": str(output),
        "source_commit": source_commit,
        "all_classes_have_nonempty_window": geometry["all_classes_have_nonempty_window"],
        "empirical_probe_executed": False,
        "manifest_sha256": sha256(manifest_path),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
