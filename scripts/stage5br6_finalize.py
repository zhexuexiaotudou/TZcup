from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys


ROOT = Path(__file__).resolve().parents[1]
LEARNING = ROOT / "starter_ws" / "src" / "sanitation_learning"
sys.path.insert(0, str(LEARNING))

from sanitation_learning.human_review_handoff import audit_handoff  # noqa: E402


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--handoff-root", required=True)
    parser.add_argument("--review-dir", required=True)
    parser.add_argument("--negative-capture-root", required=True)
    args = parser.parse_args()
    handoff_root, review = Path(args.handoff_root), Path(args.review_dir)
    negative_capture_root = Path(args.negative_capture_root)
    if review.exists():
        raise RuntimeError(f"review directory already exists: {review}")
    review.mkdir(parents=True)
    handoff = json.loads((handoff_root / "handoff_manifest.json").read_text(encoding="utf-8"))
    audit = audit_handoff(handoff_root)
    redacted = {
        key: value for key, value in handoff.items()
        if key not in {"sealed_truth"}
    }
    redacted["sealed_truth"] = {
        "sha256": handoff["sealed_truth"]["sha256"],
        "stored_in_git_ignored_handoff_only": True,
        "not_for_reviewers": True,
    }
    write_json(review / "human_handoff_manifest_redacted.json", redacted)
    write_json(review / "human_handoff_integrity_report.json", audit)
    capture = json.loads((negative_capture_root / "capture_report.json").read_text(encoding="utf-8"))
    capture_records = capture["records"]
    capture_summary = {
        "schema_version": 1,
        "stage": "Stage5BR6-A",
        "capture_source": "actual Gazebo Harmonic V4 verification camera",
        "training_only_label_zero_world": True,
        "production_world_modified": False,
        "camera_contract": capture["camera_contract"],
        "capture_count": len(capture_records),
        "exact_four_sensor_timestamp_count": sum(bool(item["exact_four_sensor_timestamp"]) for item in capture_records),
        "semantic_target_pixel_count_total": sum(int(item["semantic_target_pixel_count"]) for item in capture_records),
        "negative_category_counts": capture.get("negative_category_counts", {
            category: sum(item["negative_category"] == category for item in capture_records)
            for category in sorted({item["negative_category"] for item in capture_records})
        }),
        "capture_report_sha256": sha256(negative_capture_root / "capture_report.json"),
    }
    write_json(review / "v4_negative_capture_summary.json", capture_summary)

    status = {
        "schema_version": 1,
        "stage": "Stage5BR6-A",
        "predecessor_main_commit": "c345445dc6f09a13e77e376d51d67052eb185e16",
        "historical_stage5br5_conclusion_preserved": {
            "first_blocking_layer": "G2_camera_selection_blocked_two_independent_human_manual_reviewers_not_available",
            "READY_FOR_GPT_REVIEW_STAGE5B": False,
            "READY_FOR_STAGE5C": False,
        },
        "human_review_handoff": {
            "pre_registered_camera_candidate": "V4",
            "camera_selected": False,
            "positive_sample_count": handoff["positive_sample_count"],
            "negative_sample_count": handoff["negative_sample_count"],
            "total_sample_count_per_reviewer": handoff["total_sample_count"],
            "negative_category_counts": handoff["negative_category_counts"],
            "negative_camera_contracts": handoff["negative_camera_contracts"],
            "semantic_target_pixels_in_negative_crops": handoff["semantic_target_pixels_in_negative_crops"],
            "reviewer_package_count": len(handoff["reviewer_packages"]),
            "reviewer_packages_crc_pass": all(item["zip_crc_pass"] for item in handoff["reviewer_packages"]),
            "reviewer_ids_disjoint": audit["reviewer_ids_disjoint"],
            "reviewer_orders_independent": audit["reviewer_orders_independent"],
            "truth_mapping_absent_from_reviewer_packages": audit["truth_mapping_absent_from_reviewer_packages"],
            "metadata_leakage_count": 0,
            "reviewer_response_count": 0,
            "required_independent_reviewers": 2,
        },
        "manual_audit": {
            "completed": False,
            "integrity_pass": False,
            "scoring_executed": False,
            "metrics": None,
            "scripts_or_llm_substitute_for_human_review": False,
        },
        "downstream_stop_boundary": {
            "camera_contract_frozen": False,
            "policy_v2_frozen": False,
            "candidate_production_footprint_changed": False,
            "candidate_footprint_stage4w_regression_executed": False,
            "oracle_active_observation_executed": False,
            "detector_or_area_model_training_executed": False,
            "j6_gate_executed": False,
        },
        "competition_perception_pass": False,
        "real_domain_evaluation_executed": False,
        "j6_runtime_pass": False,
        "competition_efficiency_pass": False,
        "theoretical_efficiency_m2_h": 1053,
        "target_efficiency_m2_h": 3500,
        "first_blocking_layer": "G2_camera_selection_blocked_awaiting_two_independent_human_reviews",
        "AWAITING_HUMAN_REVIEW": True,
        "READY_FOR_STAGE5BR6_ORACLE": False,
        "REVIEW_PACKET_COMPLETE": True,
        "READY_FOR_GPT_REVIEW_STAGE5BR6": False,
        "READY_FOR_STAGE5BR7": False,
        "READY_FOR_GPT_REVIEW_STAGE5B": False,
        "READY_FOR_STAGE5C": False,
    }
    write_json(review / "stage5br6_status.json", status)
    shutil.copy2(ROOT / "starter_ws" / "src" / "sanitation_learning" / "test" / "test_stage5br6_handoff.py", review / "test_stage5br6_handoff.py")

    files = []
    for path in sorted(review.rglob("*")):
        if path.is_file() and path.name != "artifact_manifest.json":
            files.append({"path": path.relative_to(review).as_posix(), "bytes": path.stat().st_size, "sha256": sha256(path)})
    write_json(review / "artifact_manifest.json", {
        "schema_version": 1,
        "stage": "Stage5BR6-A",
        "file_count_excluding_manifest": len(files),
        "files": files,
        "REVIEW_PACKET_COMPLETE": True,
        "AWAITING_HUMAN_REVIEW": True,
        "READY_FOR_GPT_REVIEW_STAGE5BR6": False,
        "READY_FOR_STAGE5BR7": False,
    })
    print(json.dumps({"review_dir": str(review), "files": len(files), "first_blocking_layer": status["first_blocking_layer"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
