from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--review-dir", required=True)
    args = parser.parse_args()
    root = Path(args.review_dir)
    status = load(root / "stage5br4_status.json")
    observability = load(root / "perception_observability_report.json")
    ablation = load(root / "camera_ablation_report.json")
    manual = load(root / "manual_recognizability_audit.json")
    production = load(root / "production_isolation" / "production_isolation_report.json")
    regression = load(root / "stage5br4_regression_summary.json")
    checks = {
        "historical_attempts_unchanged": status["historical_stage5br3_failure_preserved"]["attempts_modified"] is False,
        "all_visible_retained": observability["partitions"]["all_visible"]["count"] == 3370,
        "ready_and_non_ready_partition_exact": observability["partitions"]["recognition_ready"]["count"] + observability["partitions"]["non_ready"]["count"] == 3370,
        "camera_runtime_pass": ablation["camera_ablation_runtime_pass"] is True,
        "active_conversion_gate_failed": ablation["active_observation"]["gate_pass"] is False,
        "manual_audit_failed": manual["manual_audit_pass"] is False,
        "no_camera_selected": ablation["camera_selected_for_model_training"] is None,
        "model_training_not_executed": status["model_recovery"]["detector_micro_overfit_executed"] is False,
        "dataset_expansion_not_executed": status["dataset_restructure"]["executed"] is False,
        "production_isolation_pass": production["production_isolation_pass"] is True,
        "regression_gate_pass": regression["regression_gate_pass"] is True,
        "review_complete": status["REVIEW_PACKET_COMPLETE"] is True,
        "stage5b_not_ready": status["READY_FOR_GPT_REVIEW_STAGE5B"] is False,
        "stage5c_not_ready": status["READY_FOR_STAGE5C"] is False,
    }
    if not all(checks.values()):
        raise RuntimeError(f"Stage5BR4 finalization failed: {checks}")
    files = []
    for path in sorted(root.rglob("*")):
        if not path.is_file() or path.name == "artifact_manifest.json":
            continue
        files.append({
            "path": path.relative_to(root).as_posix(),
            "bytes": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        })
    manifest = {
        "schema_version": 1,
        "stage": "Stage5BR4",
        "checks": checks,
        "file_count_excluding_manifest": len(files),
        "files": files,
        "REVIEW_PACKET_COMPLETE": True,
        "READY_FOR_GPT_REVIEW_STAGE5B": False,
        "READY_FOR_STAGE5C": False,
    }
    (root / "artifact_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"checks": checks, "file_count": len(files)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
