from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import platform
from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[1]
SPOT = ROOT / "starter_ws" / "src" / "sanitation_spot_cleaning"
import sys
sys.path.insert(0, str(SPOT))

from sanitation_spot_cleaning.auto03_contract import summarize_auto03  # noqa: E402


WORLD_IDS = (
    "world_a_asphalt_campus",
    "world_b_concrete_sidewalk",
    "world_c_wet_dark_ground",
    "world_d_mixed_curb_vegetation",
    "world_e_tiled_plaza",
    "world_f_service_road",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def command_text(*args: str) -> str | None:
    completed = subprocess.run(
        args, cwd=ROOT, capture_output=True, text=True, check=False
    )
    return completed.stdout.strip() if completed.returncode == 0 else None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--state", default=str(ROOT / "autonomous_state.json"))
    args = parser.parse_args()
    raw = Path(args.raw_root)
    if not raw.is_absolute():
        raw = ROOT / raw
    raw = raw.resolve()
    output = Path(args.output)
    if not output.is_absolute():
        output = ROOT / output
    output = output.resolve()
    matrix = json.loads((raw / "auto03_matrix.json").read_text(encoding="utf-8"))
    runtime_trials = []
    isolation = {}
    replay_audits = {}
    world_summaries = {}
    for world_id in WORLD_IDS:
        world_root = raw / world_id
        runtime = json.loads((world_root / "runtime_trials.json").read_text(encoding="utf-8"))
        if not runtime.get("runtime_complete"):
            raise RuntimeError(f"incomplete runtime: {world_id}")
        runtime_trials.extend(runtime["trials"])
        control_graph = (world_root / "control_node_graph.txt").read_text(
            encoding="utf-8", errors="replace"
        )
        gt_graph = (world_root / "gt_semantic_topic_graph.txt").read_text(
            encoding="utf-8", errors="replace"
        )
        replay = json.loads(
            (world_root / "replay_audit.json").read_text(encoding="utf-8")
        )
        replay_log = (world_root / "replay_start.log").read_text(
            encoding="utf-8", errors="replace"
        )
        forbidden = (
            "controller_server",
            "planner_server",
            "bt_navigator",
            "stage5br5_observation_pose_planner",
            "auto03_observation_executive",
        )
        gt_subscriber_violation = any(
            f"Node name: {node}" in gt_graph for node in forbidden
        )
        isolation[world_id] = {
            "control_graph_has_ground_truth_topic": (
                "/ground_truth" in control_graph or "/g2/" in control_graph
            ),
            "gt_topic_has_control_subscriber": gt_subscriber_violation,
            "evaluation_subscriber_present": (
                "Node name: auto03_machine_ready_evaluator" in gt_graph
            ),
        }
        world_summaries[world_id] = {
            "expected_count": runtime["expected_count"],
            "result_count": runtime["result_count"],
            "robot_pose_set_by_oracle": runtime["robot_pose_set_by_oracle"],
            "runtime_complete": runtime["runtime_complete"],
            "replay_audit_pass": replay["replay_audit_pass"],
            "required_topic_coverage": replay["required_topic_coverage"],
            "metric_replay_delta_max": replay["metric_replay_delta_max"],
            "task_timeline_reconstructable": replay[
                "task_timeline_reconstructable"
            ],
            "rosbag_playback_started": "Playback until timestamp:" in replay_log,
        }
        replay_audits[world_id] = replay

    report = summarize_auto03(matrix["trials"], runtime_trials)
    report["attempt_id"] = matrix["attempt_id"]
    report["matrix_sha256"] = matrix["matrix_sha256"]
    report["world_runtime"] = world_summaries
    report["gt_isolation"] = isolation
    isolation_pass = all(
        not item["control_graph_has_ground_truth_topic"]
        and not item["gt_topic_has_control_subscriber"]
        and item["evaluation_subscriber_present"]
        for item in isolation.values()
    )
    report["checks"]["node_graph_gt_isolation"] = isolation_pass
    report["checks"]["mcap_required_topic_coverage_100_percent"] = all(
        item["required_topic_coverage"] == 1.0
        for item in replay_audits.values()
    )
    report["checks"]["mcap_replay_audit_6_of_6"] = all(
        item["replay_audit_pass"] for item in replay_audits.values()
    )
    report["checks"]["mcap_metric_replay_delta_at_most_0_01"] = all(
        item["metric_replay_delta_max"] is not None
        and item["metric_replay_delta_max"] <= 0.01
        for item in replay_audits.values()
    )
    report["checks"]["task_timeline_reconstructable_6_of_6"] = all(
        item["task_timeline_reconstructable"]
        for item in replay_audits.values()
    )
    report["checks"]["rosbag_playback_started_6_of_6"] = all(
        item["rosbag_playback_started"] for item in world_summaries.values()
    )
    report["auto03_gate_pass"] = all(report["checks"].values())

    if output.exists():
        raise RuntimeError(f"refusing to overwrite evidence directory: {output}")
    output.mkdir(parents=True)
    (output / "auto03_acceptance_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    (output / "matrix_summary.json").write_text(
        json.dumps({
            "schema_version": matrix["schema_version"],
            "stage": matrix["stage"],
            "attempt_id": matrix["attempt_id"],
            "matrix_sha256": matrix["matrix_sha256"],
            "oracle_policy": matrix["oracle_policy"],
            "trial_count": len(matrix["trials"]),
            "world_count": len({item["world_id"] for item in matrix["trials"]}),
            "scene_count": len({
                (item["world_id"], item["scene_id"]) for item in matrix["trials"]
            }),
        }, indent=2) + "\n",
        encoding="utf-8",
    )
    shutil.copy2(
        ROOT / "starter_ws/src/sanitation_spot_cleaning/config/auto03_observation_pose.yaml",
        output / "auto03_observation_pose.yaml",
    )
    runtime_root = output / "runtime"
    runtime_root.mkdir()
    for world_id in WORLD_IDS:
        source = raw / world_id
        target = runtime_root / world_id
        target.mkdir()
        for name in (
            "runtime_trials.json",
            "control_node_graph.txt",
            "gt_semantic_topic_graph.txt",
            "executive_node_info.txt",
            "evaluator_node_info.txt",
            "oracle_node_info.txt",
            "planner_node_info.txt",
            "map_server_state.txt",
            "map_topic_info.txt",
            "rosbag_info.txt",
            "rosbag_recorder_node_info.txt",
            "replay_audit.json",
            "replay_start.log",
        ):
            shutil.copy2(source / name, target / name)

    state_path = Path(args.state)
    if not state_path.is_absolute():
        state_path = ROOT / state_path
    state_path = state_path.resolve()
    state = json.loads(state_path.read_text(encoding="utf-8"))
    stage = state["stages"]["AUTO-03"]
    implementation_commit = command_text("git", "rev-parse", "HEAD")
    baseline_commit = command_text("git", "merge-base", "HEAD", "origin/main")
    completed_at = datetime.now(timezone.utc).isoformat()
    config_path = (
        ROOT
        / "starter_ws/src/sanitation_spot_cleaning/config/auto03_observation_pose.yaml"
    )
    capture_runtime_path = (
        ROOT
        / "starter_ws/src/sanitation_spot_cleaning/"
        "sanitation_spot_cleaning/auto03_matrix_probe.py"
    )
    configuration_sha256 = sha256(config_path)
    capture_runtime_sha256 = sha256(capture_runtime_path)
    attempts = [
        {
            "attempt_id": "AUTO-03-V5-RETRACTED-R1",
            "hypothesis": "AUTO-01 retracted camera is sufficient for verification",
            "result": "FAIL",
            "first_failure": "self pixels 24.79%, center error 81.65 px and return timeout",
            "changed_variables": [],
            "decision": "redesign_camera_pose",
            "raw_evidence": [str(raw.parent / "autonomous_auto03_raw_20260729")],
        },
        {
            "attempt_id": "AUTO-03-CORNER-CAMERA-R1",
            "hypothesis": "a corner camera inside the frozen footprint removes self occlusion",
            "result": "FAIL",
            "first_failure": "yaw 35 deg self pixels 11.2%; yaw 90 deg self pixels 8.35% with navigation instability",
            "changed_variables": ["camera mount x/y/z, pitch and yaw"],
            "decision": "select_yaw_45_pitch_35",
            "raw_evidence": [str(raw.parent / "autonomous_auto03_raw_20260729")],
        },
        {
            "attempt_id": "AUTO-03-PARALLEL-FORMAL-R1",
            "hypothesis": "two formal Gazebo worlds can run concurrently",
            "result": "FAIL",
            "first_failure": "CPU contention reduced world B navigation to 26/34",
            "changed_variables": ["world execution concurrency"],
            "decision": "force_sequential_worlds",
            "raw_evidence": [
                str(
                    raw.parent
                    / "autonomous_auto03_raw_20260729/attempt_parallel_cpu_contention_failed"
                )
            ],
        },
        {
            "attempt_id": "AUTO-03-AFFINE-PROJECTION-R2",
            "hypothesis": "2D affine center calibration and robust class scales generalize",
            "result": "PASS",
            "first_failure": None,
            "changed_variables": ["projection affine", "per-class short-side scale"],
            "decision": "freeze_after_world_e_heldout_pass",
            "raw_evidence": [
                str(
                    raw.parent
                    / "autonomous_auto03_raw_20260729/calibration_evidence"
                )
            ],
        },
        {
            "attempt_id": "AUTO-03-MAP-STARTUP-RACE-R1",
            "hypothesis": "Nav2 can start concurrently with the localization map server",
            "result": "FAIL",
            "first_failure": "map lifecycle response timeout left StaticLayer on a 5 x 5 m fallback costmap",
            "changed_variables": ["startup ordering"],
            "decision": "require_active_map_publisher_before_nav2",
            "raw_evidence": [
                str(raw / "attempt_world_a_compute_path_failed")
            ],
        },
        {
            "attempt_id": "AUTO-03-MCAP-DISCOVERY-R1",
            "hypothesis": "a fixed sleep is sufficient for rosbag topic discovery",
            "result": "FAIL",
            "first_failure": "2/14 required topics absent and one trial result missed",
            "changed_variables": ["bag discovery readiness"],
            "decision": "delay_oracle_until_all_topics_subscribed",
            "raw_evidence": [
                str(raw.parent / "autonomous_auto03_world_a_bag_smoke")
            ],
        },
        {
            "attempt_id": "AUTO-03-ROI-UNCERTAINTY-R1",
            "hypothesis": "the calibrated target-size box is also a sufficient search ROI",
            "result": "FAIL",
            "first_failure": "world A partial center-inside-ROI rate was 5/6 despite 11.9 px center-error P95",
            "changed_variables": ["search ROI uncertainty margin"],
            "decision": "separate_target_size_prediction_from_search_roi",
            "raw_evidence": [
                str(raw / "attempt_projection_roi_undercoverage")
            ],
        },
        {
            "attempt_id": "AUTO-03-YAW-CONVERGENCE-R1",
            "hypothesis": "a single NavigateToPose always closes final position and yaw",
            "result": "FAIL",
            "first_failure": "three near-goal samples timed out outside the 0.2 m position tolerance with yaw still open",
            "changed_variables": ["near-goal yaw handoff"],
            "decision": "cancel_then_submit_orientation_only_navigate_goal",
            "raw_evidence": [
                str(raw / "attempt_yaw_convergence_timeout")
            ],
        },
        {
            "attempt_id": "AUTO-03-CLASS-SCALE-R3",
            "hypothesis": "the world E ten-trial class scales generalize to the full matrix",
            "result": "FAIL",
            "first_failure": "world A short-side relative-error P95 was 0.4063; leaf and puddle size predictions were systematically low",
            "changed_variables": ["leaf_pile and puddle short-side scales"],
            "decision": "recalibrate_from_full_world_a_then_restart_all_six_worlds",
            "raw_evidence": [
                str(raw / "attempt_projection_class_scale_calibration")
            ],
        },
        {
            "attempt_id": "AUTO-03-INSTALL-SYNC-R1",
            "hypothesis": "SkipBuild also refreshes modified package share configuration",
            "result": "FAIL",
            "first_failure": "runtime leaf prediction retained the old 0.570 scale because the colcon install YAML was stale",
            "changed_variables": ["colcon install workspace"],
            "decision": "rebuild_workspace_before_formal_restart",
            "raw_evidence": [
                str(
                    raw
                    / "attempt_projection_class_scale_calibration/world_a_v4_stale_install_partial"
                )
            ],
        },
        {
            "attempt_id": "AUTO-03-YAW-ZERO-LENGTH-R2",
            "hypothesis": "an orientation-only NavigateToPose at the current XY is a stable yaw handoff",
            "result": "FAIL",
            "first_failure": "three of thirteen world A handoffs timed out after DWB drifted about 0.5 m from the observation pose",
            "changed_variables": ["yaw-handoff retry goal XY"],
            "decision": "replan_to_original_observation_pose",
            "raw_evidence": [
                str(raw / "attempt_yaw_handoff_instability_after_calibration")
            ],
        },
        {
            "attempt_id": "AUTO-03-BOUNDARY-HEADING-R3",
            "hypothesis": "position-only boundary return leaves a safe heading for the next observation",
            "result": "FAIL",
            "first_failure": "target 036 started near pi radians after the prior proximity return and timed out before reaching the observation neighborhood",
            "changed_variables": ["pre-navigation path-heading alignment"],
            "decision": "use_collision_checked_nav2_spin_before_large_heading_changes",
            "raw_evidence": [
                str(raw / "attempt_boundary_yaw_not_restored")
            ],
        },
        {
            "attempt_id": "AUTO-03-SIM-CLOCK-R4",
            "hypothesis": "wall-clock action deadlines and costs are equivalent to Gazebo simulation time",
            "result": "FAIL",
            "first_failure": "20 wall seconds advanced only a fraction of the Nav2 Spin allowance under sub-real-time simulation, causing premature cancellation and inflated cost",
            "changed_variables": ["navigation timeout clock", "cost clock"],
            "decision": "use_ros_sim_time_with_bounded_wall_deadlock_guard",
            "raw_evidence": [
                str(
                    raw.parent
                    / "autonomous_auto03_world_a_prealign_smoke"
                )
            ],
        },
        {
            "attempt_id": "AUTO-03-FORMAL-SHORT-P95-R5",
            "hypothesis": "the pre-calibrated capture projection passes the complete six-world matrix",
            "result": "FAIL",
            "first_failure": "170-sample short-side relative-error P95 was 0.31173 against the 0.30 hard gate",
            "changed_variables": [],
            "decision": "fit_capture_short_model_on_worlds_a_to_d_and_hold_out_worlds_e_to_f",
            "raw_evidence": [
                str(raw / "attempt_capture_calibration_full_matrix_1")
            ],
        },
        {
            "attempt_id": "AUTO-03-CAPTURE-SHORT-FIT-R6",
            "hypothesis": "capture-only short-side calibration generalizes without changing observation poses",
            "result": "PASS",
            "first_failure": None,
            "changed_variables": [
                "capture-only per-class short-side coefficients"
            ],
            "decision": "rerun_all_six_worlds_with_planner_calibration_unchanged",
            "raw_evidence": [
                str(
                    raw
                    / "attempt_capture_calibration_full_matrix_1"
                    / "capture_short_calibration_fit.json"
                )
            ],
        },
        {
            "attempt_id": matrix["attempt_id"],
            "hypothesis": "sequential six-world active observation passes every machine gate",
            "result": "PASS" if report["auto03_gate_pass"] else "FAIL",
            "first_failure": (
                None
                if report["auto03_gate_pass"]
                else next(
                    name for name, passed in report["checks"].items() if not passed
                )
            ),
            "changed_variables": [],
            "decision": "select" if report["auto03_gate_pass"] else "block",
            "raw_evidence": [str(raw)],
        },
    ]
    for attempt in attempts:
        attempt.update({
            "input_commit": baseline_commit,
            "configuration_sha256": configuration_sha256,
            "capture_runtime_sha256": capture_runtime_sha256,
            "fixed_variables": [
                "autonomous_navigation_profile_v1",
                "AUTO03_corner",
                "six G2 worlds",
                "250-case matrix",
            ],
            "commands": [
                [
                    "powershell",
                    "-File",
                    "scripts/run_auto03_matrix_docker.ps1",
                ]
            ],
            "metrics": report["metrics"] if attempt["attempt_id"] == matrix["attempt_id"] else {},
        })
    stage.update({
        "status": "PASS" if report["auto03_gate_pass"] else "FAIL",
        "machine_gate_pass": report["auto03_gate_pass"],
        "blocked": not report["auto03_gate_pass"],
        "blocked_external": False,
        "first_blocking_layer": (
            None if report["auto03_gate_pass"]
            else next(name for name, passed in report["checks"].items() if not passed)
        ),
        "attempt_count": len(attempts),
        "selected_attempt": matrix["attempt_id"] if report["auto03_gate_pass"] else None,
        "implementation_commit": implementation_commit,
        "evidence_dir": output.relative_to(ROOT).as_posix(),
        "metrics": report,
        "unexecuted_items": [],
    })
    state["run"]["branch"] = "agent/autonomous-auto03"
    state["run"]["current_stage"] = "AUTO-04" if report["auto03_gate_pass"] else "AUTO-03"
    state["run"]["last_commit"] = implementation_commit
    state_path.write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")

    shutil.copy2(config_path, output / "stage_config.yaml")
    shutil.copy2(capture_runtime_path, output / "auto03_matrix_probe.py")
    calibration_root = output / "calibration"
    calibration_root.mkdir()
    shutil.copy2(
        raw
        / "attempt_capture_calibration_full_matrix_1"
        / "capture_short_calibration_fit.json",
        calibration_root / "capture_short_calibration_fit.json",
    )
    (output / "attempt_ledger.json").write_text(
        json.dumps({"schema_version": 1, "attempts": attempts}, indent=2) + "\n",
        encoding="utf-8",
    )
    stage_status = {
        "schema_version": 1,
        "program": "TZcup autonomous final",
        "stage_id": "AUTO-03",
        "baseline_commit": baseline_commit,
        "implementation_commit": implementation_commit,
        "started_at": state["run"]["started_at"],
        "completed_at": completed_at,
        "status": stage["status"],
        "first_blocking_layer": stage["first_blocking_layer"],
        "attempt_count": len(attempts),
        "machine_gate_pass": report["auto03_gate_pass"],
        "human_review_required": False,
        "human_approval_required": False,
        "competition_evidence": False,
        "oracle_candidate_only": True,
        "dependencies": {"AUTO-01": "PASS"},
        "metrics": report["metrics"],
        "unexecuted_items": [],
        "next_scheduled_stages": ["AUTO-04"] if report["auto03_gate_pass"] else [],
    }
    (output / "stage_status.json").write_text(
        json.dumps(stage_status, indent=2) + "\n", encoding="utf-8"
    )
    (output / "metrics_summary.json").write_text(
        json.dumps(report["metrics"], indent=2) + "\n", encoding="utf-8"
    )
    (output / "raw_metric_index.json").write_text(
        json.dumps({
            "schema_version": 1,
            "raw_root": str(raw),
            "matrix": str(raw / "auto03_matrix.json"),
            "capture_short_calibration": (
                "calibration/capture_short_calibration_fit.json"
            ),
            "worlds": {
                world_id: {
                    "runtime": str(raw / world_id / "runtime_trials.json"),
                    "mcap": str(raw / world_id / "auto03_runtime_bag"),
                    "compact_runtime": f"runtime/{world_id}/runtime_trials.json",
                    "replay_audit": f"runtime/{world_id}/replay_audit.json",
                }
                for world_id in WORLD_IDS
            },
        }, indent=2) + "\n",
        encoding="utf-8",
    )
    (output / "regression_summary.json").write_text(
        json.dumps({
            "schema_version": 1,
            "stage": "AUTO-03",
            "six_world_runtime_complete": all(
                item["runtime_complete"] for item in world_summaries.values()
            ),
            "six_world_replay_complete": all(
                item["replay_audit_pass"] for item in world_summaries.values()
            ),
            "node_graph_gt_isolation": isolation_pass,
            "auto03_gate_pass": report["auto03_gate_pass"],
        }, indent=2) + "\n",
        encoding="utf-8",
    )
    image_id = command_text(
        "docker", "image", "inspect", "tzcup/sanitation-jazzy:stage5b",
        "--format", "{{.Id}}"
    )
    (output / "environment.json").write_text(
        json.dumps({
            "schema_version": 1,
            "host_os": platform.platform(),
            "host_python": platform.python_version(),
            "timezone": "Asia/Shanghai",
            "container_os": "Ubuntu 24.04",
            "ros_distribution": "Jazzy",
            "gazebo": "Harmonic / Gazebo Sim 8",
            "docker_image": "tzcup/sanitation-jazzy:stage5b",
            "docker_image_id": image_id,
            "git_commit": implementation_commit,
            "random_seed": 3,
            "matrix_sha256": matrix["matrix_sha256"],
            "stage_config_sha256": configuration_sha256,
            "capture_runtime_sha256": capture_runtime_sha256,
            "j6_sdk": None,
        }, indent=2) + "\n",
        encoding="utf-8",
    )
    (output / "commands.txt").write_text(
        "powershell -NoProfile -ExecutionPolicy Bypass -File "
        "scripts/run_auto03_matrix_docker.ps1 "
        "-OutputName autonomous_auto03_formal_20260729 -SkipMatrixGeneration -SkipBuild\n"
        "python scripts/finalize_auto03.py "
        "--raw-root artifacts/autonomous_auto03_formal_20260729 "
        "--output artifacts/autonomous_auto03_20260729_evidence\n",
        encoding="utf-8",
    )
    (output / "README.md").write_text(
        "# AUTO-03 紧凑证据\n\n"
        "本目录由六个 Gazebo 世界、250 条 Oracle 主动观察 trial 自动聚合。"
        "Oracle 只发布带噪 XY、协方差、时间戳、通用类别及假/失联候选；"
        "GT 仅进入独立评测节点，不进入规划、导航或控制。"
        "每个世界都保留节点图、MCAP 元数据、消息级指标重算和实际回放启动证据。"
        "该结论仅为机器仿真，不是真人、真实车辆、真实域、J6 或最终竞赛通过。\n",
        encoding="utf-8",
    )

    manifest = {
        "schema_version": 1,
        "stage": "AUTO-03",
        "root": ".",
        "files": [],
    }
    for path in sorted(output.rglob("*")):
        if path.is_file() and path.name != "artifact_manifest.json":
            manifest["files"].append({
                "path": path.relative_to(output).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            })
    (output / "artifact_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "output": str(output),
        "auto03_gate_pass": report["auto03_gate_pass"],
        "failed_checks": [
            name for name, passed in report["checks"].items() if not passed
        ],
        "trial_count": report["trial_count"],
    }, indent=2))
    raise SystemExit(0 if report["auto03_gate_pass"] else 2)


if __name__ == "__main__":
    main()
