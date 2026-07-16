#!/usr/bin/env python3
"""Create the compact, checksum-verifiable Stage4U review evidence set."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "artifacts" / "stage4u_20260716_review"
FORMAL = ROOT / "artifacts" / "stage4t_localization_20260716_063729" / "oracle_localization_report.json"
MAP_LANES = ROOT / "artifacts" / "stage4u_20260716_map_lanes"


def load(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def dump(path: Path, payload) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def compact_trial(trial):
    return {
        "trial_path": trial["trial_path"],
        "sample_count": trial.get("sample_count"),
        "estimate_sample_count": trial.get("estimate_sample_count"),
        "truth_sample_count": trial.get("truth_sample_count"),
        "map_relative_xy_rmse_m": trial.get("map_relative_localization_error", {}).get("xy_m", {}).get("rmse"),
        "map_relative_yaw_rmse_rad": trial.get("map_relative_localization_error", {}).get("yaw_rad", {}).get("rmse"),
        "particle_valid_update_count": trial.get("particle_filter", {}).get("valid_update_count"),
        "particle_count_min": trial.get("particle_filter", {}).get("particle_count_min"),
        "particle_count_max": trial.get("particle_filter", {}).get("particle_count_max"),
        "particle_instrumentation_pass": trial.get("particle_filter", {}).get("particle_instrumentation_pass"),
        "particle_degenerate_update_count": trial.get("particle_filter", {}).get("degenerate_update_count"),
        "tf_continuous": trial.get("tf_continuity", {}).get("continuous"),
        "navigation_success": trial.get("navigation", {}).get("success"),
        "navigation_recoveries": trial.get("navigation", {}).get("recoveries_max"),
        "navigation_exit_code": trial.get("navigation_exit_code"),
        "evaluator_exit_code": trial.get("evaluator_exit_code"),
    }


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    formal = load(FORMAL)
    compact = {key: value for key, value in formal.items() if key != "trials"}
    compact["trials"] = [compact_trial(trial) for trial in formal["trials"]]
    dump(OUTPUT / "oracle_10seed_compact.json", compact)

    for relative in (
        "map_lane_summary.json",
        "M1_slam_raw/map_geometry.json",
        "M1_slam_raw/map_quality.json",
        "M2_slam_refined/lane_status.json",
        "M2_slam_refined/map_geometry.json",
        "M2_slam_refined/map_quality.json",
        "M3_structured_reference_v2_002/surveyed_reference_report.json",
        "structured_world_v2/structured_world_report.json",
    ):
        source = MAP_LANES / relative
        target = OUTPUT / relative.replace("/", "__")
        target.write_bytes(source.read_bytes())

    summary = {
        "schema_version": 1,
        "stage": "Stage4U",
        "baseline_remote_main": "de5106cdaf0948888c0225a1076cad790280efa3",
        "map_frame_contract": "explicit frozen T_target_source; no per-seed fitting",
        "particle_topic_type": "nav2_msgs/msg/ParticleCloud",
        "map_lanes": {
            "M1_slam_raw": "localization geometry failed; low-quality rigid calibration rejected",
            "M2_slam_refined": "posegraph serialized; offline optimization not executed; localization geometry failed",
            "M3_surveyed_reference": "localization-only reference; not mapping evidence",
        },
        "backend_screening_map_relative_xy_rmse_m": {
            "M1_amcl": 0.199260,
            "M2_amcl": 0.0874376,
            "M2_slam_toolbox": 0.297881,
            "M3_sparse_amcl_baseline": 0.124271,
            "M3_sparse_amcl_tuned_360x10": 0.112926,
            "M3_sparse_amcl_tuned_720x20": 0.122323,
            "M3_structured_v2_005_amcl": 0.0647310,
            "M3_structured_v2_002_amcl": 0.0628141,
        },
        "formal_oracle_10seed": {
            "candidate": "M3 structured v2, 0.02 m, AMCL tuned profile, 360 samples at 10 Hz",
            "completed_seed_count": formal["completed_seed_count"],
            "required_seed_count": formal["required_seed_count"],
            "xy_rmse_m": formal["map_relative_xy_rmse_m"],
            "yaw_rmse_rad": formal["map_relative_yaw_rmse_rad"],
            "particle_instrumentation_pass_all_trials": formal["particle_instrumentation_pass_all_trials"],
            "tf_continuous_all_trials": formal["tf_continuous_all_trials"],
            "navigation_success_all_trials": formal["navigation_success_all_trials"],
            "recovery_count": formal["recovery_count"],
            "worst_seed_trial": formal["worst_seed_trial"],
            "oracle_localization_pass": formal["oracle_localization_pass"],
        },
        "stop_condition": "Oracle 10-seed max RMSE exceeds 0.05 m; realistic and full Coverage not executed",
        "theoretical_efficiency_m2_per_h": 1053,
        "competition_efficiency_target_m2_per_h": 3500,
        "competition_efficiency_pass": False,
        "READY_FOR_GPT_REVIEW_STAGE4U": False,
        "READY_FOR_STAGE5A": False,
    }
    dump(OUTPUT / "stage4u_summary.json", summary)

    entries = []
    for path in sorted(OUTPUT.glob("*")):
        if path.name == "MANIFEST.json" or not path.is_file():
            continue
        data = path.read_bytes()
        entries.append({"path": path.name, "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()})
    dump(OUTPUT / "MANIFEST.json", {"schema_version": 1, "entries": entries})


if __name__ == "__main__":
    main()
