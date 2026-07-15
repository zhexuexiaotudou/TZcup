#!/usr/bin/env python3
"""Create a compact, traceable Stage4T GPT-review evidence directory."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import shutil
from pathlib import Path


def copy_file(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(source, destination)
    content = destination.read_bytes()
    if b"\x00" not in content:
        try:
            text = content.decode("utf-8")
        except UnicodeDecodeError:
            pass
        else:
            lines = text.replace("\r\n", "\n").splitlines()
            destination.write_text("\n".join(line.rstrip() for line in lines).rstrip() + "\n", encoding="utf-8")


def copy_if(source, destination):
    if source and source.is_file(): copy_file(source, destination)


def load(path): return json.loads(path.read_text(encoding="utf-8")) if path and path.is_file() else None


def git_archive_bytes(path):
    """Match Git's text normalization so hashes survive cross-platform archives."""
    content = path.read_bytes()
    if b"\x00" not in content:
        try:
            content.decode("utf-8")
        except UnicodeDecodeError:
            pass
        else:
            content = content.replace(b"\r\n", b"\n")
    return content


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--output", required=True, type=Path); parser.add_argument("--core", required=True, type=Path); parser.add_argument("--transient", required=True, type=Path); parser.add_argument("--ablation", required=True, type=Path); parser.add_argument("--mapping", type=Path); parser.add_argument("--localization", type=Path); parser.add_argument("--oracle-localization", type=Path); parser.add_argument("--coverage", type=Path)
    args = parser.parse_args(); out = args.output; out.mkdir(parents=True, exist_ok=True)
    core_files = ("measurement_covariance_report.json", "operational_envelope_report.json", "rosbag_info.txt", "stage4t_core_smoke_summary.json")
    for name in core_files: copy_if(args.core / name, out / name)
    for name in ("transient_response_report.json", "closed_loop_heading_report.json"):
        copy_if(args.transient / name, out / name)
    trial_out = out / "angular_rate_trials"; trial_out.mkdir(exist_ok=True)
    for path in sorted((args.transient / "angular_rate_trials").glob("*.json")): copy_file(path, trial_out / path.name)
    for name in ("ekf_ablation_report.json", "selected_ekf.yaml"):
        copy_if(args.ablation / name, out / name)
    for path in sorted((args.ablation / "ekf_trials").glob("*/seed_*/motion_calibration_report.json")):
        copy_file(path, out / "ekf_trials" / path.relative_to(args.ablation / "ekf_trials"))
    if args.mapping:
        for name in ("map_geometry_report.json", "map_truth_overlay.png", "selected_map.yaml", "selected_map.pgm", "selected_map_alignment.env"):
            copy_if(args.mapping / name, out / name)
        for pattern in ("map_geometry.json", "map_quality.json", "mapping_probe.json", "rosbag_info.txt", "gazebo_mapping.png", "map_truth_overlay.png", "map_preview.png"):
            for path in sorted(args.mapping.glob(f"map_*/{pattern}")): copy_file(path, out / "map_trials" / path.parent.name / path.name)
    if args.localization:
        copy_if(args.localization / "realistic_localization_report.json", out / "realistic_localization_report.json")
        for path in sorted((args.localization / "localization_trials").glob("seed_*/localization_report.json")): copy_file(path, out / "localization_trials" / path.relative_to(args.localization / "localization_trials"))
    if args.oracle_localization:
        copy_if(args.oracle_localization / "oracle_localization_report.json", out / "oracle_localization_report.json")
        for path in sorted((args.oracle_localization / "localization_trials").glob("seed_*/localization_report.json")):
            copy_file(path, out / "oracle_localization_trials" / path.relative_to(args.oracle_localization / "localization_trials"))
    if args.coverage:
        for name in ("stage4t_coverage_report.json", "coverage_report.json", "dynamic_obstacle_report.json", "filter_report.json", "safety_latency_report.json", "rosbag_info.txt", "replay_ground_truth.txt", "gazebo_initial.png", "gazebo_coverage.png"):
            destination = out / ("coverage_rosbag_info.txt" if name == "rosbag_info.txt" else name)
            copy_if(args.coverage / name, destination)
    if not args.localization:
        write = {"schema_version": 1, "executed": False, "competition_evidence": False, "competition_localization_pass": False, "blocked_by": "oracle_localization_pass", "reason": "Stage4T stop condition: Oracle localization did not pass the 0.05 m XY RMSE gate."}
        (out / "realistic_localization_report.json").write_text(json.dumps(write, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if not args.coverage:
        write = {"schema_version": 1, "executed": False, "competition_evidence": False, "success": False, "blocked_by": "realistic_localization_pass", "reason": "Full Coverage, dynamic-obstacle, filter, emergency-stop, and replay gates were not run after localization failure."}
        (out / "stage4t_coverage_report.json").write_text(json.dumps(write, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    transient = load(out / "transient_response_report.json"); heading = load(out / "closed_loop_heading_report.json"); covariance = load(out / "measurement_covariance_report.json"); envelope = load(out / "operational_envelope_report.json"); ablation = load(out / "ekf_ablation_report.json"); mapping = load(out / "map_geometry_report.json"); realistic = load(out / "realistic_localization_report.json"); oracle = load(out / "oracle_localization_report.json"); coverage = load(out / "stage4t_coverage_report.json")
    gates = {
        "transient_matrix_complete": bool(transient and transient.get("matrix_complete")),
        "closed_loop_matrix_complete": bool(heading and heading.get("matrix_complete") and heading.get("closed_loop_tracking_pass")),
        "operational_envelope_pass": bool(envelope and envelope.get("pass")),
        "measurement_covariance_pass": bool(covariance and covariance.get("pass")),
        "ekf_ablation_pass": bool(ablation and ablation.get("ablation_complete") and ablation.get("selected_candidate")),
        "map_geometry_pass": bool(mapping and mapping.get("pass")),
        "oracle_localization_pass": bool(oracle and oracle.get("oracle_localization_pass")),
        "realistic_localization_pass": bool(realistic and realistic.get("competition_localization_pass")),
        "coverage_and_safety_pass": bool(coverage and coverage.get("success")),
        "complete_rosbag_replay": bool(args.coverage and (args.coverage / "full_mission_bag" / "metadata.yaml").is_file() and (out / "replay_ground_truth.txt").is_file() and (out / "replay_ground_truth.txt").stat().st_size > 0),
    }
    first_failed = next((name for name, passed in gates.items() if not passed), None)
    readiness = bool(gates["realistic_localization_pass"] and gates["coverage_and_safety_pass"] and all(gates.values()))
    summary = {
        "schema_version": 1, "generated_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "baseline_commit": "b7734801d775740dccf6ce16a12f6e739b2e8136",
        "stage4s_high_speed_failure_preserved": True,
        "high_speed_open_loop_stress_pass": bool(transient and transient.get("high_speed_open_loop_stress_pass")),
        "stress_profile_default_enabled": False,
        "selected_ekf_candidate": ablation.get("selected_candidate") if ablation else None,
        "optional_chassis_controller": "not_needed" if heading and heading.get("closed_loop_tracking_pass") else "not_implemented_gate_not_met",
        "oracle_lane": oracle,
        "readiness_gates": gates, "first_failed_layer": first_failed,
        "ready_for_gpt_review_stage4t": readiness, "ready_for_stage5a": readiness,
        "competition_efficiency": {"theoretical_m2_h": 1053.0, "target_m2_h": 3500.0, "competition_efficiency_pass": False},
    }
    (out / "stage4t_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    entries = []
    for path in sorted(item for item in out.rglob("*") if item.is_file() and item.name != "MANIFEST.json"):
        content = git_archive_bytes(path)
        entries.append({"path": path.relative_to(out).as_posix(), "size_bytes": len(content), "sha256": hashlib.sha256(content).hexdigest()})
    manifest = {"schema_version": 1, "generated_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(), "hash_basis": "git_archive_bytes_lf_normalized_for_utf8_text", "self_excluded": "MANIFEST.json", "file_count": len(entries), "files": entries}
    (out / "MANIFEST.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__": main()
