#!/usr/bin/env python3
"""Fail-closed audit for known Gazebo water-runtime startup warnings."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from dataclasses import dataclass
from pathlib import Path


ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
# The safety architecture deliberately loads brush / recovery controllers
# inactive; only whole_vehicle_safety_manager may activate them.  This loader
# message is the deterministic point after both velocity controllers exist and
# all currently allowlisted Gazebo / ros2_control startup warnings have passed.
STABLE_MARKER = "[spawner_brush_controller]: Loaded recovery_controller"
DIAGNOSTIC_RE = re.compile(
    r"\[WARN\]|\[ERROR\]|\bWarning \[|\bError:|Traceback|"
    r"publish_thread_failed|join_fatal",
    re.IGNORECASE,
)
PREEMBEDDED_SENSOR_WORLD_PATH_RE = re.compile(
    r"[._]preembedded_sensor_world\.sdf"
)


@dataclass(frozen=True)
class WarningRule:
    identifier: str
    pattern: re.Pattern[str]
    source_prefix: str = "[gazebo-1]"
    expected_count: int = 1
    allowed_counts: tuple[int, ...] | None = None


RULES = (
    WarningRule(
        "sdformat_vn100_gz_frame_id_extension",
        re.compile(r'sensor\[@name="vn100"\]/gz_frame_id:.*XML Element\[gz_frame_id\].*not defined in SDF'),
    ),
    WarningRule(
        "sdformat_zed_f9p_gz_frame_id_extension",
        re.compile(r'sensor\[@name="zed_f9p"\]/gz_frame_id:.*XML Element\[gz_frame_id\].*not defined in SDF'),
    ),
    WarningRule(
        "sdformat_utm30lx_gz_frame_id_extension",
        re.compile(r'sensor\[@name="utm30lx"\]/gz_frame_id:.*XML Element\[gz_frame_id\].*not defined in SDF'),
    ),
    WarningRule(
        "gz_ros_control_waiting_resource_manager",
        re.compile(r"\[gz_ros_control\]: Waiting RM to load and initialize hardware\.\.\."),
    ),
    WarningRule(
        "gz_ros_control_vn100_not_in_hardware_info",
        re.compile(r"\[gz_ros_control\]: IMU sensor 'vn100' not found in hardware_info, skipping\."),
    ),
    WarningRule(
        "controller_hardware_initialization_executor_unavailable",
        re.compile(r"\[controller_manager\.hardware_component\.system\.formal_vehicle_system\]: Executor is not available during hardware component initialization.*Skipping node creation!"),
    ),
    WarningRule(
        "controller_statistics_not_initialized",
        re.compile(r"\[controller_manager\]: Component 'formal_vehicle_system' does not have read or write statistics initialized, skipping registration\."),
    ),
    WarningRule(
        "controller_waiting_for_robot_description",
        re.compile(r"\[controller_manager\]: Waiting for data on 'robot_description' topic to finish initialization"),
        allowed_counts=(0, 1),
    ),
    WarningRule(
        "controller_update_period_slower_than_simulation",
        re.compile(r"\[gz_ros_control\]:\s+Desired controller update period \(0\.004 s\) is slower than the gazebo simulation period \(0\.001 s\)\."),
    ),
)

# The generated preembedded world retains the same three known warnings above
# and adds these exact nine sensor declarations.  Keep this list separate from
# the normal runtime profile: accepting these names on a dynamic launch would
# hide an unexpected generated-world dependency.
PREEMBEDDED_SENSOR_WORLD_RULES = (
    WarningRule(
        "sdformat_front_rgbd_d435_infra1_gz_frame_id_extension",
        re.compile(
            r'sensor\[@name="front_rgbd_d435_infra1"\]/gz_frame_id:.*'
            r'[._]preembedded_sensor_world\.sdf:.*XML Element\[gz_frame_id\].*not defined in SDF'
        ),
    ),
    WarningRule(
        "sdformat_front_rgbd_d435_infra2_gz_frame_id_extension",
        re.compile(
            r'sensor\[@name="front_rgbd_d435_infra2"\]/gz_frame_id:.*'
            r'[._]preembedded_sensor_world\.sdf:.*XML Element\[gz_frame_id\].*not defined in SDF'
        ),
    ),
    WarningRule(
        "sdformat_front_rgbd_d435_rgbd_gz_frame_id_extension",
        re.compile(
            r'sensor\[@name="front_rgbd_d435_rgbd"\]/gz_frame_id:.*'
            r'[._]preembedded_sensor_world\.sdf:.*XML Element\[gz_frame_id\].*not defined in SDF'
        ),
    ),
    WarningRule(
        "sdformat_mid360_gz_frame_id_extension",
        re.compile(
            r'sensor\[@name="mid360"\]/gz_frame_id:.*[._]preembedded_sensor_world\.sdf:.*'
            r'XML Element\[gz_frame_id\].*not defined in SDF'
        ),
    ),
    WarningRule(
        "sdformat_rear_left_fisheye_imx291_gz_frame_id_extension",
        re.compile(
            r'sensor\[@name="rear_left_fisheye_imx291"\]/gz_frame_id:.*'
            r'[._]preembedded_sensor_world\.sdf:.*XML Element\[gz_frame_id\].*not defined in SDF'
        ),
    ),
    WarningRule(
        "sdformat_rear_right_fisheye_imx291_gz_frame_id_extension",
        re.compile(
            r'sensor\[@name="rear_right_fisheye_imx291"\]/gz_frame_id:.*'
            r'[._]preembedded_sensor_world\.sdf:.*XML Element\[gz_frame_id\].*not defined in SDF'
        ),
    ),
    WarningRule(
        "sdformat_wrist_rgbd_d435_infra1_gz_frame_id_extension",
        re.compile(
            r'sensor\[@name="wrist_rgbd_d435_infra1"\]/gz_frame_id:.*'
            r'[._]preembedded_sensor_world\.sdf:.*XML Element\[gz_frame_id\].*not defined in SDF'
        ),
    ),
    WarningRule(
        "sdformat_wrist_rgbd_d435_infra2_gz_frame_id_extension",
        re.compile(
            r'sensor\[@name="wrist_rgbd_d435_infra2"\]/gz_frame_id:.*'
            r'[._]preembedded_sensor_world\.sdf:.*XML Element\[gz_frame_id\].*not defined in SDF'
        ),
    ),
    WarningRule(
        "sdformat_wrist_rgbd_d435_rgbd_gz_frame_id_extension",
        re.compile(
            r'sensor\[@name="wrist_rgbd_d435_rgbd"\]/gz_frame_id:.*'
            r'[._]preembedded_sensor_world\.sdf:.*XML Element\[gz_frame_id\].*not defined in SDF'
        ),
    ),
)


def audit(
    log_path: Path,
    *,
    stable_marker: str = STABLE_MARKER,
    expected_stable_marker_count: int = 1,
    required_clean_exit_processes: tuple[str, ...] = (),
    required_clean_exit_process_counts: dict[str, int] | None = None,
) -> dict:
    if expected_stable_marker_count < 1:
        raise ValueError("expected_stable_marker_count must be at least 1")
    raw = log_path.read_bytes()
    lines = [ANSI_RE.sub("", line) for line in raw.decode("utf-8", errors="replace").splitlines()]
    preembedded_sensor_world = any(
        PREEMBEDDED_SENSOR_WORLD_PATH_RE.search(line) for line in lines
    )
    active_rules = RULES + (
        PREEMBEDDED_SENSOR_WORLD_RULES if preembedded_sensor_world else ()
    )
    marker_lines = [
        index for index, line in enumerate(lines, start=1) if stable_marker in line
    ]
    # The preflight and physical validator attach to the same launch.  A normal
    # or full scenario therefore has one controller-loader marker.  The stable
    # runtime window begins at the final marker, while the exact expected count
    # remains a separate fail-closed check for callers that combine logs.
    marker_line = marker_lines[-1] if marker_lines else None
    matches: dict[str, list[int]] = {rule.identifier: [] for rule in active_rules}
    unexpected: list[dict[str, object]] = []
    runtime_diagnostics: list[dict[str, object]] = []

    for line_number, line in enumerate(lines, start=1):
        if not DIAGNOSTIC_RE.search(line):
            continue
        matched = None
        for rule in active_rules:
            if line.startswith(rule.source_prefix) and rule.pattern.search(line):
                matched = rule
                break
        if matched is None:
            diagnostic = {"line": line_number, "text": line}
            unexpected.append(diagnostic)
            if marker_line is None or line_number >= marker_line:
                runtime_diagnostics.append(diagnostic)
            continue
        matches[matched.identifier].append(line_number)
        if marker_line is None or line_number >= marker_line:
            runtime_diagnostics.append(
                {
                    "line": line_number,
                    "signature": matched.identifier,
                    "text": line,
                }
            )

    rule_results = {
        rule.identifier: {
            "source_prefix": rule.source_prefix,
            "expected_count": rule.expected_count,
            "allowed_counts": list(rule.allowed_counts or (rule.expected_count,)),
            "observed_count": len(matches[rule.identifier]),
            "line_numbers": matches[rule.identifier],
            "startup_only": True,
            "passed": len(matches[rule.identifier]) in (rule.allowed_counts or (rule.expected_count,)),
        }
        for rule in active_rules
    }
    required_clean_exit_counts = {
        process: 1 for process in required_clean_exit_processes
    }
    for process, expected_count in (required_clean_exit_process_counts or {}).items():
        if not process or expected_count < 1:
            raise ValueError("required clean-exit process counts must be positive")
        required_clean_exit_counts[process] = expected_count
    clean_exit_counts = {
        process: sum(
            bool(
                re.search(
                    rf"\[INFO\] \[{re.escape(process)}-\d+\]: "
                    r"process has finished cleanly \[pid \d+\]",
                    line,
                )
            )
            for line in lines
        )
        for process in required_clean_exit_counts
    }
    checks = {
        "expected_stable_window_marker_count": (
            len(marker_lines) == expected_stable_marker_count
        ),
        "all_expected_startup_warning_counts_exact": all(
            result["passed"] for result in rule_results.values()
        ),
        "zero_unexpected_warning_or_error_lines": not unexpected,
        "zero_warning_or_error_at_or_after_stable_window": not runtime_diagnostics,
        "required_processes_finished_cleanly_exactly": all(
            clean_exit_counts[process] == expected_count
            for process, expected_count in required_clean_exit_counts.items()
        ),
    }
    passed = all(checks.values())
    return {
        "schema_version": 3,
        "status": "FORMAL_WATER_LAUNCH_LOG_AUDIT_PASSED" if passed else "FAILED",
        "passed": passed,
        "log": str(log_path),
        "log_sha256": hashlib.sha256(raw).hexdigest(),
        "warning_profile": (
            "preembedded_sensor_world" if preembedded_sensor_world else "dynamic"
        ),
        "stable_window_marker": stable_marker,
        "expected_stable_window_marker_count": expected_stable_marker_count,
        "stable_window_marker_lines": marker_lines,
        "stable_window_start_line": marker_line,
        "checks": checks,
        "rules": rule_results,
        "unexpected_diagnostics": unexpected,
        "runtime_diagnostics": runtime_diagnostics,
        "required_clean_exit_processes": list(required_clean_exit_counts),
        "required_clean_exit_process_counts": required_clean_exit_counts,
        "clean_exit_counts": clean_exit_counts,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--stable-marker",
        default=STABLE_MARKER,
        help="Exact substring that starts the warning-free stable window.",
    )
    parser.add_argument(
        "--expected-stable-marker-count",
        type=int,
        default=1,
        help=(
            "Exact number of stable marker occurrences required; the final "
            "occurrence starts the warning-free stable window."
        ),
    )
    parser.add_argument(
        "--required-clean-exit-process",
        action="append",
        default=[],
        help="Launch executable that must report exactly one clean exit.",
    )
    parser.add_argument(
        "--required-clean-exit-process-count",
        action="append",
        default=[],
        metavar="PROCESS=COUNT",
        help="Launch executable and its exact required clean-exit count.",
    )
    args = parser.parse_args()
    required_counts: dict[str, int] = {}
    for item in args.required_clean_exit_process_count:
        process, separator, raw_count = item.rpartition("=")
        if not separator or not process:
            parser.error("--required-clean-exit-process-count must be PROCESS=COUNT")
        try:
            count = int(raw_count)
        except ValueError:
            parser.error("--required-clean-exit-process-count COUNT must be an integer")
        if count < 1:
            parser.error("--required-clean-exit-process-count COUNT must be positive")
        required_counts[process] = count
    report = audit(
        args.log,
        stable_marker=args.stable_marker,
        expected_stable_marker_count=args.expected_stable_marker_count,
        required_clean_exit_processes=tuple(args.required_clean_exit_process),
        required_clean_exit_process_counts=required_counts,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
