#!/usr/bin/env python3
"""Fail-closed finalizer for one high-bandwidth formal sensor transport probe.

The runner deliberately keeps the Windows start gates, watchdog and sensor
collector as separate processes.  This finalizer turns their *fresh*,
attempt-local records into one auditable decision without starting any runtime
process itself.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping


REPORT_ID = "tzcup_formal_sensor_transport_probe_v1"
SUCCESS = "FORMAL_SENSOR_TRANSPORT_PROBE_PASSED"
BLOCKED = "FORMAL_SENSOR_TRANSPORT_PROBE_BLOCKED"
WINDOWS_START_REPORT_ID = "tzcup_formal_windows_memory_start_gate_v1"
WINDOWS_START_PASS = "FORMAL_WINDOWS_MEMORY_START_GATE_PASSED"
WATCHDOG_PASS = {
    "FORMAL_MEMORY_WATCHDOG_COMPLETED",
    "FORMAL_MEMORY_WATCHDOG_STOPPED",
}
SENSOR_PASS = "FORMAL_GAZEBO_CONTROL_AND_SENSOR_RUNTIME_PASSED_EXTERNAL_FIDELITY_GATES_PENDING"
CLEANUP_PASS = "FORMAL_SENSOR_TRANSPORT_CLEANUP_PASSED"
LOOPBACK_PASS = "FORMAL_SENSOR_LOOPBACK_TRANSPORT_ATTESTED"

# The payloads which exercise Gazebo Transport and DDS most heavily.  The
# surrounding 25-topic contract is also required below; this explicit list
# makes the high-bandwidth portion reviewable in the final record.
PAYLOAD_TOPICS = (
    "/sensors/front_rgbd/depth/image_rect_raw/image",
    "/sensors/front_rgbd/depth/image_rect_raw/depth_image",
    "/sensors/front_rgbd/infra1/image_rect_raw",
    "/sensors/front_rgbd/infra2/image_rect_raw",
    "/sensors/wrist_rgbd/depth/image_rect_raw/image",
    "/sensors/wrist_rgbd/depth/image_rect_raw/depth_image",
    "/sensors/wrist_rgbd/infra1/image_rect_raw",
    "/sensors/wrist_rgbd/infra2/image_rect_raw",
    "/sensors/lidar_2d/scan",
    "/sensors/lidar_3d/points",
    "/sensors/rear_left_fisheye/image_raw",
    "/sensors/rear_right_fisheye/image_raw",
)

_WATCHDOG_SAMPLE = re.compile(
    r"^(?P<time>\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z) sample "
    r".*?nonpaged_pool_bytes=(?P<nonpaged>\d+) "
    r".*?ndis_tag_bytes=(?P<ndis>\d+) "
    r".*?suspected_ndis_pool_leak=(?P<suspect>true|false)\s*$"
)


class ProbeError(RuntimeError):
    pass


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _strict_bool(value: Any, label: str) -> bool:
    if type(value) is not bool:
        raise ProbeError(f"{label} must be a boolean")
    return value


def _strict_int(value: Any, label: str, *, minimum: int = 0) -> int:
    if type(value) is not int or value < minimum:
        raise ProbeError(f"{label} must be an integer >= {minimum}")
    return value


def _regular_file(path: Path, label: str) -> Path:
    if path.is_symlink() or not path.is_file():
        raise ProbeError(f"{label} must be a regular file: {path}")
    return path.resolve()


def _json_file(path: Path, label: str) -> dict[str, Any]:
    path = _regular_file(path, label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ProbeError(f"cannot parse {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise ProbeError(f"{label} JSON root must be an object")
    return value


def _reference(path: Path) -> dict[str, str]:
    path = _regular_file(path, "evidence")
    return {"path": str(path), "sha256": _sha256(path)}


def _epoch_from_utc(value: str, label: str) -> int:
    try:
        parsed = datetime.strptime(value, "%Y-%m-%dT%H:%M:%SZ").replace(
            tzinfo=timezone.utc
        )
    except ValueError as exc:
        raise ProbeError(f"invalid watchdog UTC timestamp {label}: {value}") from exc
    return int(parsed.timestamp() * 1_000_000_000)


def _validate_start_gate(
    path: Path, label: str, *, must_be_cold: bool
) -> tuple[dict[str, Any], dict[str, str]]:
    value = _json_file(path, label)
    if value.get("report_id") != WINDOWS_START_REPORT_ID:
        raise ProbeError(f"{label} has the wrong report_id")
    if value.get("status") != WINDOWS_START_PASS or _strict_bool(value.get("passed"), f"{label}.passed") is not True:
        raise ProbeError(f"{label} is not a passed Windows start gate")
    requires_stopped = _strict_bool(
        value.get("require_wsl_stopped"), f"{label}.require_wsl_stopped"
    )
    requires_running = _strict_bool(
        value.get("require_wsl_running"), f"{label}.require_wsl_running"
    )
    if requires_stopped == requires_running:
        raise ProbeError(f"{label} has an inconsistent WSL-state requirement")
    if must_be_cold and not requires_stopped:
        raise ProbeError(f"{label} does not prove a cold WSL boundary")
    recorded = _strict_int(value.get("recorded_epoch_ns"), f"{label}.recorded_epoch_ns", minimum=1)
    sample = value.get("sample")
    checks = value.get("checks")
    thresholds = value.get("thresholds_bytes")
    if not isinstance(sample, dict) or not isinstance(checks, dict) or not isinstance(thresholds, dict):
        raise ProbeError(f"{label} is missing sample/checks/thresholds")
    for name in (
        "windows_commit_available_at_least_configured_minimum",
        "docker_private_at_most_configured_maximum",
        "wsl_vm_stopped_when_required",
        "wsl_vm_running_when_required",
        "no_suspected_ndis_nonpaged_pool_leak",
    ):
        if _strict_bool(checks.get(name), f"{label}.checks.{name}") is not True:
            raise ProbeError(f"{label} check is not passing: {name}")
    for name in ("min_commit_available", "max_docker_private"):
        _strict_int(thresholds.get(name), f"{label}.thresholds_bytes.{name}")
    vmmem = _strict_int(
        sample.get("vmmem_wsl_private_bytes"),
        f"{label}.sample.vmmem_wsl_private_bytes",
    )
    if (requires_stopped and vmmem != 0) or (requires_running and vmmem == 0):
        raise ProbeError(f"{label} does not match its required WSL state")
    diagnostics = sample.get("pool_tag_diagnostics")
    if not isinstance(diagnostics, dict) or diagnostics.get("status") != "available":
        raise ProbeError(f"{label} lacks available Windows pool-tag evidence")
    tags = diagnostics.get("tracked_nonpaged_bytes")
    if not isinstance(tags, dict):
        raise ProbeError(f"{label} lacks tracked NDIS pool tags")
    tag_total = _strict_int(
        diagnostics.get("tracked_nonpaged_bytes_total"),
        f"{label}.sample.pool_tag_diagnostics.tracked_nonpaged_bytes_total",
    )
    if sum(_strict_int(tags.get(tag), f"{label}.sample.pool_tag_diagnostics.{tag}") for tag in ("Nbuf", "Nnbl", "Nnbf")) != tag_total:
        raise ProbeError(f"{label} NDIS pool-tag total is inconsistent")
    if _strict_bool(diagnostics.get("suspected_ndis_nonpaged_pool_leak"), f"{label}.sample.pool_tag_diagnostics.suspected_ndis_nonpaged_pool_leak"):
        raise ProbeError(f"{label} reports a suspected NDIS nonpaged-pool leak")
    value["_recorded_epoch_ns"] = recorded
    return value, _reference(path)


def _find_one(root: Path, pattern: str, label: str) -> Path:
    matches = sorted(path for path in root.rglob(pattern) if path.is_file() and not path.is_symlink())
    if len(matches) != 1:
        raise ProbeError(f"{label} requires exactly one {pattern} beneath attempt root; found {len(matches)}")
    return matches[0].resolve()


def _watchdog_summary(log_path: Path) -> dict[str, Any]:
    log_path = _regular_file(log_path, "memory watchdog log")
    samples: list[dict[str, Any]] = []
    probe_failures: list[str] = []
    for raw in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "Windows memory probe" in raw and ("failure" in raw.lower() or "exhausted" in raw.lower()):
            probe_failures.append(raw)
        if " sample " not in raw:
            continue
        match = _WATCHDOG_SAMPLE.match(raw)
        if match is None:
            raise ProbeError("memory watchdog contains a malformed sample record")
        row = {
            "epoch_ns": _epoch_from_utc(match.group("time"), "sample"),
            "utc": match.group("time"),
            "nonpaged_pool_bytes": int(match.group("nonpaged")),
            "ndis_tag_bytes": int(match.group("ndis")),
            "suspected_ndis_pool_leak": match.group("suspect") == "true",
        }
        # Watchdog log timestamps are deliberately whole-second UTC strings;
        # two accepted samples may therefore share a printed timestamp.  They
        # may not move backwards, while the enclosing before/after gates stay
        # strictly ordered below.
        if samples and row["epoch_ns"] < samples[-1]["epoch_ns"]:
            raise ProbeError("memory watchdog samples move backwards in time")
        samples.append(row)
    if not samples:
        raise ProbeError("memory watchdog contains no parseable Windows samples")
    if probe_failures:
        raise ProbeError("memory watchdog records a Windows probe failure")
    if any(row["suspected_ndis_pool_leak"] for row in samples):
        raise ProbeError("memory watchdog records a suspected NDIS nonpaged-pool leak")
    return {
        "sample_count": len(samples),
        "first": samples[0],
        "peak_nonpaged_pool": max(samples, key=lambda row: row["nonpaged_pool_bytes"]),
        "peak_ndis_tag": max(samples, key=lambda row: row["ndis_tag_bytes"]),
        "last": samples[-1],
    }


def _validate_watchdog(path: Path) -> tuple[dict[str, Any], dict[str, str]]:
    value = _json_file(path, "memory watchdog JSON")
    if value.get("report_id") != "tzcup_formal_memory_watchdog_v1":
        raise ProbeError("memory watchdog JSON has the wrong report_id")
    if value.get("status") not in WATCHDOG_PASS:
        raise ProbeError("memory watchdog did not complete cleanly")
    if _strict_int(value.get("surviving_group_processes"), "memory watchdog surviving_group_processes") != 0:
        raise ProbeError("memory watchdog reports surviving target-group processes")
    reasons = value.get("reasons")
    if not isinstance(reasons, dict):
        raise ProbeError("memory watchdog lacks reason flags")
    for name in (
        "low_mem_available",
        "excessive_swap_used",
        "excessive_group_rss",
        "low_windows_commit_available",
        "excessive_docker_private",
        "windows_probe_failed",
        "suspected_ndis_nonpaged_pool_leak",
    ):
        if _strict_bool(reasons.get(name), f"memory watchdog reasons.{name}"):
            raise ProbeError(f"memory watchdog reason is true: {name}")
    return value, _reference(path)


def _require_equal(left: Any, right: Any, label: str) -> None:
    if left != right:
        raise ProbeError(f"identity mismatch: {label}")


def _validate_sensor_report(path: Path) -> tuple[dict[str, Any], dict[str, str]]:
    report = _json_file(path, "sensor runtime report")
    if report.get("status") != SENSOR_PASS or _strict_bool(report.get("passed"), "sensor report passed") is not True:
        raise ProbeError("sensor runtime report is not formally passing")
    for name in ("passed_checks", "runtime_sensor_contract", "acceptance_session_binding", "runtime_gate_binding", "runtime_closure_binding"):
        if not isinstance(report.get(name), dict):
            raise ProbeError(f"sensor runtime report lacks {name}")
    checks = report["passed_checks"]
    for name, value in checks.items():
        if type(value) is bool and value is not True:
            raise ProbeError(f"sensor report failed check: {name}")
    contract = report["runtime_sensor_contract"]
    if _strict_bool(contract.get("passed"), "runtime sensor contract passed") is not True:
        raise ProbeError("runtime sensor contract is not passing")
    if contract.get("missing_topics") != [] or contract.get("frame_errors") != {} or contract.get("dimension_errors") != {} or contract.get("frequency_errors") != {}:
        raise ProbeError("runtime sensor contract retains missing/frame/dimension/frequency errors")
    contract_checks = contract.get("passed_checks")
    if not isinstance(contract_checks, dict) or not contract_checks:
        raise ProbeError("runtime sensor contract lacks checks")
    for name, value in contract_checks.items():
        if _strict_bool(value, f"runtime sensor contract {name}") is not True:
            raise ProbeError(f"runtime sensor contract failed check: {name}")
    counts = report.get("sample_counts")
    if not isinstance(counts, dict):
        raise ProbeError("sensor runtime report lacks sample_counts")
    for topic in PAYLOAD_TOPICS:
        if _strict_int(counts.get(topic), f"payload stream {topic}") < 3:
            raise ProbeError(f"payload stream lacks three bounded samples: {topic}")
    groups = contract.get("formal_sensor_group_observation")
    if not isinstance(groups, dict) or len(groups) != 8:
        raise ProbeError("runtime sensor contract does not retain all eight sensor groups")
    for group, topic_counts in groups.items():
        if not isinstance(topic_counts, dict) or not topic_counts:
            raise ProbeError(f"sensor group is malformed: {group}")
        for topic, count in topic_counts.items():
            if _strict_int(count, f"sensor group {group} {topic}") < 3:
                raise ProbeError(f"sensor group lacks three samples: {group}/{topic}")

    session_binding = report["acceptance_session_binding"]
    runtime_binding = report["runtime_gate_binding"]
    closure = report["runtime_closure_binding"]
    if _strict_bool(report.get("session_bound"), "sensor report session_bound") is not True:
        raise ProbeError("sensor report is not session-bound")
    session_path = session_binding.get("session_manifest_path")
    if not isinstance(session_path, str) or not session_path:
        raise ProbeError("sensor report lacks session manifest path")
    session = _json_file(Path(session_path), "bound acceptance session")
    if session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise ProbeError("bound acceptance session is not RUNNING")
    _require_equal(session_binding.get("session_manifest_sha256"), _sha256(Path(session_path)), "sensor/session manifest SHA256")
    _require_equal(session_binding.get("snapshot"), session.get("snapshot"), "sensor/session snapshot")
    runtime_session_binding = runtime_binding.get("acceptance_session_binding")
    runtime_closure = runtime_binding.get("runtime_closure_binding")
    if not isinstance(runtime_session_binding, dict) or not isinstance(runtime_closure, dict):
        raise ProbeError("runtime binding is incomplete")
    _require_equal(runtime_session_binding.get("snapshot"), session_binding.get("snapshot"), "runtime/sensor snapshot")
    _require_equal(runtime_session_binding.get("session_manifest_sha256"), session_binding.get("session_manifest_sha256"), "runtime/sensor session SHA256")
    _require_equal(runtime_closure, closure, "runtime-binding/report closure")
    _require_equal(session.get("runtime_closure_binding"), closure, "session/report closure")
    for name in ("manifest", "manifest_sha256", "closure_sha256", "runtime_install_root", "status"):
        if name not in closure:
            raise ProbeError(f"runtime closure lacks {name}")
    manifest = closure["manifest"]
    if not isinstance(manifest, str) or not manifest:
        raise ProbeError("runtime closure manifest path is invalid")
    manifest_path = _regular_file(Path(manifest), "runtime closure manifest")
    _require_equal(closure.get("manifest_sha256"), _sha256(manifest_path), "runtime closure manifest SHA256")
    if closure.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED":
        raise ProbeError("runtime closure is not verified")
    sidecar_path = path.with_name(path.name + ".runtime_binding.json")
    sidecar = _json_file(sidecar_path, "sensor runtime binding sidecar")
    _require_equal(sidecar, runtime_binding, "sensor report/runtime-binding sidecar")
    return report, _reference(path)


def _validate_cleanup(path: Path) -> tuple[dict[str, Any], dict[str, str]]:
    value = _json_file(path, "cleanup attestation")
    if value.get("status") != CLEANUP_PASS or _strict_bool(value.get("passed"), "cleanup attestation passed") is not True:
        raise ProbeError("cleanup attestation is not passing")
    if value.get("partition_survivors") != []:
        raise ProbeError("cleanup attestation retains GZ_PARTITION survivors")
    if _strict_int(value.get("surviving_group_processes"), "cleanup attestation surviving_group_processes") != 0:
        raise ProbeError("cleanup attestation retains process-group survivors")
    if _strict_bool(value.get("lock_released"), "cleanup attestation lock_released") is not True:
        raise ProbeError("cleanup attestation does not prove the global lock was released")
    return value, _reference(path)


def _validate_loopback(path: Path) -> tuple[dict[str, Any], dict[str, str]]:
    value = _json_file(path, "sensor loopback attestation")
    if (
        value.get("report_id") != "tzcup_formal_sensor_loopback_attestation_v1"
        or value.get("status") != LOOPBACK_PASS
        or _strict_bool(value.get("passed"), "loopback attestation passed") is not True
    ):
        raise ProbeError("sensor loopback transport is not attested")
    if value.get("blockers") != []:
        raise ProbeError("sensor loopback attestation retains blockers")
    processes = value.get("processes")
    if not isinstance(processes, list):
        raise ProbeError("sensor loopback attestation lacks process evidence")
    roles = {row.get("role") for row in processes if isinstance(row, dict)}
    if roles != {"gazebo_sim", "ros_gz_parameter_bridge"}:
        raise ProbeError("sensor loopback attestation lacks required process roles")
    for name in ("acceptance_session", "runtime_closure_manifest"):
        binding = value.get(name)
        if (
            not isinstance(binding, dict)
            or not isinstance(binding.get("path"), str)
            or not re.fullmatch(r"[0-9a-f]{64}", str(binding.get("sha256", "")))
        ):
            raise ProbeError(f"sensor loopback attestation lacks {name} binding")
    return value, _reference(path)


def finalize(attempt_root: Path, windows_before: Path, windows_after: Path) -> dict[str, Any]:
    if attempt_root.is_symlink() or not attempt_root.is_dir():
        raise ProbeError("attempt root must be a regular existing directory")
    root = attempt_root.resolve()
    before, before_ref = _validate_start_gate(
        windows_before, "Windows before gate", must_be_cold=True
    )
    after, after_ref = _validate_start_gate(
        windows_after, "Windows after gate", must_be_cold=False
    )
    watchdog_log = _find_one(root, "*.memory_watchdog.log", "memory watchdog log")
    watchdog_json = _find_one(root, "*.memory_watchdog.json", "memory watchdog JSON")
    sensor_report = _find_one(root, "formal_vehicle_runtime_report.json", "sensor runtime report")
    cleanup = _find_one(root, "cleanup_attestation.json", "cleanup attestation")
    loopback = _find_one(root, "*.loopback_attestation.json", "sensor loopback attestation")
    watchdog_summary = _watchdog_summary(watchdog_log)
    _, watchdog_ref = _validate_watchdog(watchdog_json)
    report, sensor_ref = _validate_sensor_report(sensor_report)
    _, cleanup_ref = _validate_cleanup(cleanup)
    loopback_value, loopback_ref = _validate_loopback(loopback)
    _require_equal(
        loopback_value["acceptance_session"]["sha256"],
        report["acceptance_session_binding"]["session_manifest_sha256"],
        "loopback/sensor session SHA256",
    )
    _require_equal(
        loopback_value["runtime_closure_manifest"]["sha256"],
        report["runtime_closure_binding"]["manifest_sha256"],
        "loopback/sensor closure manifest SHA256",
    )
    first_ns = watchdog_summary["first"]["epoch_ns"]
    last_ns = watchdog_summary["last"]["epoch_ns"]
    if not (before["_recorded_epoch_ns"] < first_ns <= last_ns < after["_recorded_epoch_ns"]):
        raise ProbeError("Windows before/watchdog/after evidence is not strictly ordered")
    return {
        "schema_version": 1,
        "report_id": REPORT_ID,
        "status": SUCCESS,
        "passed": True,
        "attempt_root": str(root),
        "high_bandwidth_payload_topics": list(PAYLOAD_TOPICS),
        "windows_memory": {
            "before": {**before_ref, "recorded_epoch_ns": before["_recorded_epoch_ns"]},
            "watchdog_log": _reference(watchdog_log),
            "watchdog": watchdog_ref,
            "summary": watchdog_summary,
            "after": {**after_ref, "recorded_epoch_ns": after["_recorded_epoch_ns"]},
        },
        "sensor_runtime": {
            **sensor_ref,
            "runtime_binding_sidecar": _reference(sensor_report.with_name(sensor_report.name + ".runtime_binding.json")),
            "session_manifest_sha256": report["acceptance_session_binding"]["session_manifest_sha256"],
            "snapshot": report["acceptance_session_binding"]["snapshot"],
            "runtime_closure_manifest_sha256": report["runtime_closure_binding"]["manifest_sha256"],
            "runtime_closure_sha256": report["runtime_closure_binding"]["closure_sha256"],
        },
        "cleanup": cleanup_ref,
        "loopback_transport": loopback_ref,
    }


def _write_exclusive(path: Path, value: Mapping[str, Any]) -> None:
    if path.exists() or path.is_symlink():
        raise ProbeError(f"refusing stale finalizer output: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open("x", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True)
            stream.write("\n")
    except FileExistsError as exc:
        raise ProbeError(f"refusing stale finalizer output: {path}") from exc


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt-root", type=Path, required=True)
    parser.add_argument("--windows-before", type=Path, required=True)
    parser.add_argument("--windows-after", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = finalize(args.attempt_root, args.windows_before, args.windows_after)
    except ProbeError as exc:
        result = {
            "schema_version": 1,
            "report_id": REPORT_ID,
            "status": BLOCKED,
            "passed": False,
            "attempt_root": str(args.attempt_root),
            "error": str(exc),
        }
        try:
            _write_exclusive(args.output, result)
        except ProbeError as output_error:
            print(json.dumps({**result, "output_error": str(output_error)}, indent=2, sort_keys=True))
            return 2
        print(json.dumps(result, indent=2, sort_keys=True))
        return 2
    try:
        _write_exclusive(args.output, result)
    except ProbeError as exc:
        print(json.dumps({"status": BLOCKED, "passed": False, "error": str(exc)}, indent=2, sort_keys=True))
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
