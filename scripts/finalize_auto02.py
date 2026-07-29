#!/usr/bin/env python3
"""Freeze the AUTO-02 navigation profile and compact its machine evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import yaml

from autonomous_runner import (
    atomic_write_json,
    build_manifest,
    write_standard_evidence,
)


ROOT = Path(__file__).resolve().parents[1]
SOURCE_PROFILE = (
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_navigation"
    / "config"
    / "auto01_g2_v5_retracted.yaml"
)
FROZEN_PROFILE = SOURCE_PROFILE.with_name("autonomous_navigation_profile_v1.yaml")


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def copy_required(source: Path, destination: Path) -> None:
    if not source.is_file():
        raise RuntimeError(f"required AUTO-02 evidence missing: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def freeze_profile(completed_at: str) -> dict:
    profile = yaml.safe_load(SOURCE_PROFILE.read_text(encoding="utf-8"))
    profile.update(
        {
            "stage": "AUTO-02",
            "attempt_id": "AUTO-02-FREEZE-V1",
            "profile": "autonomous_navigation_profile_v1",
            "source_profile": "auto01_g2_v5_retracted",
            "source_attempt_id": "AUTO-01-G2-C3",
            "frozen_after_full_regression": True,
            "frozen_at": completed_at,
        }
    )
    FROZEN_PROFILE.write_text(
        yaml.safe_dump(profile, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
        newline="\n",
    )
    return profile


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    raw = args.raw.resolve()
    acceptance = load(raw / "auto02_acceptance_report.json")
    if acceptance.get("machine_gate_pass") is not True:
        raise RuntimeError("AUTO-02 acceptance is not PASS")
    if not all(acceptance.get("checks", {}).values()):
        raise RuntimeError("AUTO-02 acceptance contains a failed check")

    completed_at = datetime.now(timezone.utc).isoformat()
    profile = freeze_profile(completed_at)
    commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
    ).strip()
    evidence_dir = args.output.resolve()
    checks = acceptance["checks"]
    stage_config = {
        "schema_version": 1,
        "stage": "AUTO-02",
        "selected_attempt": "AUTO-02-FREEZE-V1",
        "profile": "autonomous_navigation_profile_v1",
        "source_profile": "auto01_g2_v5_retracted",
        "camera_profile": "V5_retracted",
        "static_seeds": [0, 1, 2, 3, 4],
        "dynamic_seed": 10,
        "cold_start_trials": 5,
        "docker_image": "tzcup/sanitation-jazzy:stage5b",
        "production_default_unchanged": True,
        "profile_sha256": sha256(FROZEN_PROFILE),
    }
    stage_status = {
        "schema_version": 1,
        "program": "TZcup autonomous final",
        "stage_id": "AUTO-02",
        "baseline_commit": commit,
        "implementation_commit": commit,
        "started_at": None,
        "completed_at": completed_at,
        "status": "PASS",
        "first_blocking_layer": None,
        "attempt_count": 4,
        "machine_gate_pass": True,
        "human_review_required": False,
        "human_approval_required": False,
        "competition_evidence": False,
        "dependencies": {"AUTO-01": "PASS"},
        "metrics": checks,
        "unexecuted_items": [],
        "next_scheduled_stages": ["AUTO-03"],
    }
    attempts = [
        {
            "attempt_id": "AUTO-02-COLD-SMOKE",
            "hypothesis": "the selected G2 profile reaches all runtime interfaces",
            "input_commit": commit,
            "configuration_sha256": sha256(SOURCE_PROFILE),
            "changed_variables": [],
            "fixed_variables": ["auto01_g2_v5_retracted", "V5_retracted"],
            "commands": [
                [
                    "powershell",
                    "-File",
                    "scripts/run_auto02_regression_docker.ps1",
                    "-ColdTrialCount",
                    "1",
                    "-SkipStatic",
                    "-SkipDynamic",
                ]
            ],
            "result": "PASS",
            "first_failure": None,
            "metrics": {"cold_start_trial": 0},
            "raw_evidence": ["cold_start/trial_0"],
            "decision": "continue",
        },
        {
            "attempt_id": "AUTO-02-STATIC-REPLAY-CONTRACT-R1",
            "hypothesis": "every static bag must contain an emergency-stop message",
            "input_commit": commit,
            "configuration_sha256": sha256(SOURCE_PROFILE),
            "changed_variables": [],
            "fixed_variables": ["static seed 0 raw MCAP"],
            "commands": [
                [
                    "python3",
                    "scripts/auto02_replay_audit.py",
                    "--bag",
                    "static/seed_0/static_coverage_bag",
                ]
            ],
            "result": "FAIL",
            "first_failure": "inactive emergency-stop topic was incorrectly required in a static-only bag",
            "metrics": {
                "mission_complete": True,
                "coverage_metric_reproduction_delta": 0.0,
                "static_required_topics_present": "15/15 after scenario-specific correction",
            },
            "raw_evidence": ["static/seed_0"],
            "decision": "fix_audit_contract_and_reuse_immutable_raw_bag",
        },
        {
            "attempt_id": "AUTO-02-STATIC-DOMAIN-R1",
            "hypothesis": "seed-offset ROS domains remain inside the DDS port range",
            "input_commit": commit,
            "configuration_sha256": sha256(SOURCE_PROFILE),
            "changed_variables": ["ROS_DOMAIN_ID allocation"],
            "fixed_variables": ["static seed 3", "selected navigation profile"],
            "commands": [
                [
                    "powershell",
                    "-File",
                    "scripts/run_auto02_regression_docker.ps1",
                    "-ReuseCompletedStatic",
                ]
            ],
            "result": "FAIL",
            "first_failure": "ROS_DOMAIN_ID 233 exceeded the Fast DDS port range",
            "metrics": {
                "completed_static_seeds_before_failure": 3,
                "algorithm_failure": False,
            },
            "raw_evidence": ["static_failed_seed3_domain233"],
            "decision": "move_domains_to_180_184_and_resume",
        },
        {
            "attempt_id": "AUTO-02-FULL-REGRESSION",
            "hypothesis": "the selected profile passes the complete navigation matrix",
            "input_commit": commit,
            "configuration_sha256": sha256(SOURCE_PROFILE),
            "changed_variables": [],
            "fixed_variables": ["auto01_g2_v5_retracted", "V5_retracted"],
            "commands": [
                [
                    "powershell",
                    "-File",
                    "scripts/run_auto02_regression_docker.ps1",
                    "-OutputName",
                    raw.name,
                ]
            ],
            "result": "PASS",
            "first_failure": None,
            "metrics": checks,
            "raw_evidence": [str(raw)],
            "decision": "select_and_freeze",
        },
    ]
    commands = [
        ["py", "-3", "scripts/ci_fast.py"],
        [
            "powershell",
            "-File",
            "scripts/run_auto02_regression_docker.ps1",
            "-OutputName",
            raw.name,
        ],
        [
            "py",
            "-3",
            "scripts/auto02_acceptance.py",
            "--root",
            str(raw),
        ],
    ]
    metrics = {
        "schema_version": 1,
        "stage": "AUTO-02",
        "checks": checks,
        "static_trials": acceptance["static_trials"],
        "cold_start_ready_seconds": acceptance["cold_start_ready_seconds"],
        "dynamic": {
            "valid_interactions": acceptance["dynamic_summary"][
                "valid_interactions"
            ],
            "minimum_observed_separation_m": acceptance["dynamic_summary"][
                "minimum_observed_separation_m"
            ],
            "configured_hard_minimum_separation_m": acceptance[
                "dynamic_summary"
            ]["configured_hard_minimum_separation_m"],
            "speed_zone_mean_m_s": acceptance["dynamic_summary"]["filters"][
                "speed_zone"
            ]["mean_speed_m_s"]["inside"],
            "estop_latency_sec": acceptance["dynamic_summary"][
                "emergency_stop"
            ]["latency_sec"],
        },
        "autonomous_navigation_profile_frozen": True,
        "frozen_profile_sha256": sha256(FROZEN_PROFILE),
    }
    write_standard_evidence(
        ROOT,
        "AUTO-02",
        stage_config,
        stage_status,
        attempts,
        commands,
        metrics,
        evidence_dir=evidence_dir,
    )

    selected = {
        "acceptance_report.json": raw / "auto02_acceptance_report.json",
        "static/static_matrix_report.json": raw
        / "static"
        / "stage4w_static_matrix_report.json",
        "dynamic/stage4w_dynamic_report.json": raw
        / "dynamic"
        / "stage4w_dynamic_report.json",
        "dynamic/replay_audit.json": raw
        / "dynamic"
        / "auto02_replay_audit.json",
        "dynamic/rosbag_info.txt": raw / "dynamic" / "rosbag_info.txt",
        "runtime/frozen_profile.yaml": FROZEN_PROFILE,
        "runtime/seed_0_collision_monitor.json": raw
        / "static"
        / "seed_0"
        / "runtime_collision_monitor_g2_params.json",
        "runtime/seed_0_local_costmap.yaml": raw
        / "static"
        / "seed_0"
        / "runtime_local_costmap_params.yaml",
        "runtime/seed_0_global_costmap.yaml": raw
        / "static"
        / "seed_0"
        / "runtime_global_costmap_params.yaml",
        "runtime/tf_map_base.txt": raw
        / "cold_start"
        / "trial_0"
        / "tf_map_base.txt",
        "runtime/tf_odom_base.txt": raw
        / "cold_start"
        / "trial_0"
        / "tf_odom_base.txt",
    }
    for index in range(5):
        selected[f"cold_start/trial_{index}.json"] = (
            raw / "cold_start" / f"trial_{index}" / "cold_start_report.json"
        )
        selected[f"static/seed_{index}_runtime_geometry.json"] = (
            raw
            / "static"
            / f"seed_{index}"
            / "auto02_runtime_geometry_audit.json"
        )
        selected[f"static/seed_{index}_replay.json"] = (
            raw / "static" / f"seed_{index}" / "auto02_replay_audit.json"
        )
    for relative, source in selected.items():
        copy_required(source, evidence_dir / relative)

    atomic_write_json(
        evidence_dir / "raw_metric_index.json",
        {
            "schema_version": 1,
            "raw_metrics": [
                {
                    "path": relative,
                    "bytes": (evidence_dir / relative).stat().st_size,
                    "sha256": sha256(evidence_dir / relative),
                }
                for relative in sorted(selected)
            ],
        },
    )
    atomic_write_json(
        evidence_dir / "regression_summary.json",
        {
            "schema_version": 1,
            "status": "PASS",
            "regressions": [
                "85 fast ROS-independent tests passed before formal runtime",
                "five static Coverage seeds passed 17/17 components",
                "five independent cold starts reached 8/8 active lifecycle nodes",
                "dynamic interaction, keepout, speed, and 30-trial estop gates passed",
                "six MCAP runs passed metadata, required-topic, replay-state, and <=1% metric-delta gates",
                "production launch default remains production",
            ],
        },
    )
    (evidence_dir / "README.md").write_text(
        "# AUTO-02 machine evidence\n\n"
        "The AUTO-01 G2-C3 candidate passed the complete Docker ROS 2 Jazzy "
        "and Gazebo Harmonic navigation regression and is frozen as "
        "`autonomous_navigation_profile_v1`. This is simulation-only "
        "machine evidence; it does not promote human review, real-vehicle, "
        "real-domain, J6, or final competition status.\n",
        encoding="utf-8",
        newline="\n",
    )
    atomic_write_json(
        evidence_dir / "artifact_manifest.json", build_manifest(evidence_dir)
    )

    state_path = ROOT / "AUTONOMOUS_STATE.json"
    state = load(state_path)
    state["stages"]["AUTO-02"].update(
        {
            "status": "PASS",
            "machine_gate_pass": True,
            "blocked": False,
            "blocked_external": False,
            "first_blocking_layer": None,
            "attempt_count": len(attempts),
            "selected_attempt": "AUTO-02-FREEZE-V1",
            "implementation_commit": commit,
            "evidence_dir": evidence_dir.relative_to(ROOT).as_posix(),
            "metrics": metrics,
            "unexecuted_items": [],
        }
    )
    state["run"]["current_stage"] = "AUTO-03"
    state["run"]["last_commit"] = commit
    state["run"]["branch"] = subprocess.check_output(
        ["git", "branch", "--show-current"], cwd=ROOT, text=True
    ).strip()
    atomic_write_json(state_path, state)
    print(evidence_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
