"""Fixture tests for the fail-closed sensor transport probe finalizer."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts" / "finalize_formal_sensor_transport_probe.py"
SPEC = importlib.util.spec_from_file_location("sensor_transport_finalizer", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _start_gate(
    recorded_ns: int,
    *,
    suspected: bool = False,
    pool_available: bool = True,
    wsl_running: bool = False,
) -> dict[str, object]:
    tags = {"Nbuf": 10, "Nnbl": 20, "Nnbf": 30}
    return {
        "report_id": "tzcup_formal_windows_memory_start_gate_v1",
        "status": "FORMAL_WINDOWS_MEMORY_START_GATE_PASSED",
        "passed": True,
        "recorded_epoch_ns": recorded_ns,
        "thresholds_bytes": {"min_commit_available": 1, "max_docker_private": 2},
        "require_wsl_stopped": not wsl_running,
        "require_wsl_running": wsl_running,
        "sample": {
            "vmmem_wsl_private_bytes": 1 if wsl_running else 0,
            "pool_tag_diagnostics": {
                "status": "available" if pool_available else "unavailable",
                "tracked_nonpaged_bytes": tags,
                "tracked_nonpaged_bytes_total": sum(tags.values()),
                "suspected_ndis_nonpaged_pool_leak": suspected,
            },
        },
        "checks": {
            "windows_commit_available_at_least_configured_minimum": True,
            "docker_private_at_most_configured_maximum": True,
            "wsl_vm_stopped_when_required": True,
            "wsl_vm_running_when_required": True,
            "no_suspected_ndis_nonpaged_pool_leak": not suspected,
        },
    }


def _watchdog(*, status: str = "FORMAL_MEMORY_WATCHDOG_COMPLETED", reason: bool = False) -> dict[str, object]:
    return {
        "report_id": "tzcup_formal_memory_watchdog_v1",
        "status": status,
        "surviving_group_processes": 0,
        "reasons": {
            "low_mem_available": False,
            "excessive_swap_used": False,
            "excessive_group_rss": False,
            "low_windows_commit_available": False,
            "excessive_docker_private": False,
            "windows_probe_failed": reason,
            "suspected_ndis_nonpaged_pool_leak": False,
        },
    }


def _watchdog_log(*, suspect: bool = False, probe_failure: bool = False) -> str:
    lines = [
        "2026-09-01T00:00:01Z sample mem_available_kib=9000000 swap_used_kib=0 group_rss_kib=100 windows_commit_available_bytes=50000000000 docker_private_bytes=0 vmmem_wsl_private_bytes=4000000000 nonpaged_pool_bytes=100 pool_tags_available=true ndis_tag_bytes=20 suspected_ndis_pool_leak=false",
        "2026-09-01T00:00:02Z sample mem_available_kib=9000000 swap_used_kib=0 group_rss_kib=100 windows_commit_available_bytes=50000000000 docker_private_bytes=0 vmmem_wsl_private_bytes=4000000000 nonpaged_pool_bytes=300 pool_tags_available=true ndis_tag_bytes=80 suspected_ndis_pool_leak=false",
        "2026-09-01T00:00:03Z sample mem_available_kib=9000000 swap_used_kib=0 group_rss_kib=100 windows_commit_available_bytes=50000000000 docker_private_bytes=0 vmmem_wsl_private_bytes=4000000000 nonpaged_pool_bytes=200 pool_tags_available=true ndis_tag_bytes=40 suspected_ndis_pool_leak=" + ("true" if suspect else "false"),
    ]
    if probe_failure:
        lines.append("2026-09-01T00:00:04Z Windows memory probe transient failure kind=stdout_timeout")
    return "\n".join(lines) + "\n"


def _fixture(tmp_path: Path) -> dict[str, Path]:
    attempt = tmp_path / "attempt"
    sensor_dir = attempt / "sensor"
    external = tmp_path / "bound"
    manifest = external / "final_runtime_closure_manifest.json"
    _write(manifest, {"kind": "fixture"})
    closure = {
        "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
        "manifest": str(manifest),
        "manifest_sha256": _sha(manifest),
        "closure_sha256": "c" * 64,
        "runtime_install_root": str(external / "install"),
    }
    snapshot = {
        "snapshot_manifest_sha256": "a" * 64,
        "source_inventory_sha256": "b" * 64,
        "expanded_urdf_sha256": "d" * 64,
    }
    session = external / "formal_final_acceptance_session.json"
    _write(session, {
        "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        "snapshot": snapshot,
        "runtime_closure_binding": closure,
    })
    session_sha = _sha(session)
    acceptance = {
        "session_manifest_path": str(session),
        "session_manifest_sha256": session_sha,
        "snapshot": snapshot,
    }
    binding = {
        "acceptance_session_binding": acceptance,
        "runtime_closure_binding": closure,
    }
    groups = {f"group_{index}": {f"/topic/{index}": 3} for index in range(8)}
    report = {
        "report_id": "tzcup_formal_vehicle_headless_runtime_v5",
        "status": "FORMAL_GAZEBO_CONTROL_AND_SENSOR_RUNTIME_PASSED_EXTERNAL_FIDELITY_GATES_PENDING",
        "passed": True,
        "session_bound": True,
        "passed_checks": {"collector": True, "payload_limit": 1.512},
        "sample_counts": {topic: 3 for topic in MODULE.PAYLOAD_TOPICS},
        "runtime_sensor_contract": {
            "passed": True,
            "passed_checks": {"streams": True, "frames": True},
            "missing_topics": [],
            "frame_errors": {},
            "dimension_errors": {},
            "frequency_errors": {},
            "formal_sensor_group_observation": groups,
        },
        "acceptance_session_binding": acceptance,
        "runtime_gate_binding": binding,
        "runtime_closure_binding": closure,
    }
    report_path = sensor_dir / "formal_vehicle_runtime_report.json"
    _write(report_path, report)
    _write(report_path.with_name(report_path.name + ".runtime_binding.json"), binding)
    _write(sensor_dir / "formal_vehicle_runtime_report.memory_watchdog.json", _watchdog())
    (sensor_dir / "formal_vehicle_runtime_report.memory_watchdog.log").parent.mkdir(parents=True, exist_ok=True)
    (sensor_dir / "formal_vehicle_runtime_report.memory_watchdog.log").write_text(_watchdog_log(), encoding="utf-8")
    _write(attempt / "cleanup_attestation.json", {
        "status": "FORMAL_SENSOR_TRANSPORT_CLEANUP_PASSED",
        "passed": True,
        "partition_survivors": [],
        "surviving_group_processes": 0,
        "lock_released": True,
    })
    _write(sensor_dir / "formal_vehicle_runtime_report.loopback_attestation.json", {
        "report_id": "tzcup_formal_sensor_loopback_attestation_v1",
        "status": "FORMAL_SENSOR_LOOPBACK_TRANSPORT_ATTESTED",
        "passed": True,
        "blockers": [],
        "processes": [
            {"role": "gazebo_sim"},
            {"role": "ros_gz_parameter_bridge"},
        ],
        "acceptance_session": {"path": str(session), "sha256": session_sha},
        "runtime_closure_manifest": {
            "path": str(manifest),
            "sha256": _sha(manifest),
        },
    })
    before = tmp_path / "windows_before.json"
    after = tmp_path / "windows_after.json"
    _write(before, _start_gate(1))
    _write(after, _start_gate(9_000_000_000_000_000_000))
    return {
        "attempt": attempt,
        "before": before,
        "after": after,
        "report": report_path,
        "watchdog": sensor_dir / "formal_vehicle_runtime_report.memory_watchdog.json",
        "watchdog_log": sensor_dir / "formal_vehicle_runtime_report.memory_watchdog.log",
        "cleanup": attempt / "cleanup_attestation.json",
        "loopback": sensor_dir / "formal_vehicle_runtime_report.loopback_attestation.json",
        "session": session,
    }


def test_finalizer_accepts_complete_fresh_bound_probe(tmp_path: Path) -> None:
    paths = _fixture(tmp_path)
    result = MODULE.finalize(paths["attempt"], paths["before"], paths["after"])
    assert result["status"] == MODULE.SUCCESS
    assert result["passed"] is True
    assert result["windows_memory"]["summary"]["sample_count"] == 3
    assert result["windows_memory"]["summary"]["peak_nonpaged_pool"]["nonpaged_pool_bytes"] == 300
    assert result["windows_memory"]["summary"]["peak_ndis_tag"]["ndis_tag_bytes"] == 80
    assert result["sensor_runtime"]["runtime_closure_manifest_sha256"] == _sha(
        Path(json.loads(paths["report"].read_text(encoding="utf-8"))["runtime_closure_binding"]["manifest"])
    )


@pytest.mark.parametrize("suspected,pool_available", [(True, True), (False, False)])
def test_start_gate_requires_available_non_suspect_pool_tags(
    tmp_path: Path, suspected: bool, pool_available: bool
) -> None:
    paths = _fixture(tmp_path)
    _write(paths["before"], _start_gate(1, suspected=suspected, pool_available=pool_available))
    with pytest.raises(MODULE.ProbeError):
        MODULE.finalize(paths["attempt"], paths["before"], paths["after"])


def test_after_gate_may_be_warm_when_its_wsl_state_is_consistent(tmp_path: Path) -> None:
    paths = _fixture(tmp_path)
    _write(paths["after"], _start_gate(9_000_000_000_000_000_000, wsl_running=True))
    assert MODULE.finalize(paths["attempt"], paths["before"], paths["after"])["passed"] is True


def test_watchdog_rejects_any_ndis_suspect_or_probe_failure(tmp_path: Path) -> None:
    paths = _fixture(tmp_path)
    paths["watchdog_log"].write_text(_watchdog_log(suspect=True), encoding="utf-8")
    with pytest.raises(MODULE.ProbeError, match="NDIS"):
        MODULE.finalize(paths["attempt"], paths["before"], paths["after"])
    paths["watchdog_log"].write_text(_watchdog_log(probe_failure=True), encoding="utf-8")
    with pytest.raises(MODULE.ProbeError, match="probe failure"):
        MODULE.finalize(paths["attempt"], paths["before"], paths["after"])


@pytest.mark.parametrize("status,reason", [
    ("FORMAL_MEMORY_WATCHDOG_COMPLETED", True),
    ("FORMAL_MEMORY_WATCHDOG_STOPPED", True),
    ("UNEXPECTED", False),
])
def test_watchdog_requires_clean_terminal_status_and_no_reason(
    tmp_path: Path, status: str, reason: bool
) -> None:
    paths = _fixture(tmp_path)
    _write(paths["watchdog"], _watchdog(status=status, reason=reason))
    with pytest.raises(MODULE.ProbeError):
        MODULE.finalize(paths["attempt"], paths["before"], paths["after"])


def test_watchdog_accepts_normal_parent_requested_stop_after_clean_runner(
    tmp_path: Path,
) -> None:
    paths = _fixture(tmp_path)
    _write(paths["watchdog"], _watchdog(status="FORMAL_MEMORY_WATCHDOG_STOPPED"))
    assert MODULE.finalize(paths["attempt"], paths["before"], paths["after"])["passed"] is True


def test_sensor_requires_all_twelve_payloads_and_runtime_contract(tmp_path: Path) -> None:
    paths = _fixture(tmp_path)
    report = json.loads(paths["report"].read_text(encoding="utf-8"))
    report["sample_counts"][MODULE.PAYLOAD_TOPICS[0]] = 2
    _write(paths["report"], report)
    with pytest.raises(MODULE.ProbeError, match="payload stream"):
        MODULE.finalize(paths["attempt"], paths["before"], paths["after"])
    paths = _fixture(tmp_path / "contract")
    report = json.loads(paths["report"].read_text(encoding="utf-8"))
    report["runtime_sensor_contract"]["frequency_errors"] = {"/sensor": {"observed_hz": 0}}
    _write(paths["report"], report)
    with pytest.raises(MODULE.ProbeError, match="missing/frame/dimension/frequency"):
        MODULE.finalize(paths["attempt"], paths["before"], paths["after"])


def test_sensor_rejects_session_closure_or_sidecar_drift(tmp_path: Path) -> None:
    paths = _fixture(tmp_path)
    session = json.loads(paths["session"].read_text(encoding="utf-8"))
    session["runtime_closure_binding"]["closure_sha256"] = "e" * 64
    _write(paths["session"], session)
    with pytest.raises(MODULE.ProbeError, match="sensor/session manifest SHA256"):
        MODULE.finalize(paths["attempt"], paths["before"], paths["after"])
    paths = _fixture(tmp_path / "sidecar")
    sidecar = paths["report"].with_name(paths["report"].name + ".runtime_binding.json")
    _write(sidecar, {"unexpected": True})
    with pytest.raises(MODULE.ProbeError, match="sidecar"):
        MODULE.finalize(paths["attempt"], paths["before"], paths["after"])


@pytest.mark.parametrize("field,value", [
    ("partition_survivors", [123]),
    ("surviving_group_processes", 1),
    ("lock_released", False),
])
def test_cleanup_requires_no_partition_or_group_survivors(
    tmp_path: Path, field: str, value: object
) -> None:
    paths = _fixture(tmp_path)
    cleanup = json.loads(paths["cleanup"].read_text(encoding="utf-8"))
    cleanup[field] = value
    _write(paths["cleanup"], cleanup)
    with pytest.raises(MODULE.ProbeError):
        MODULE.finalize(paths["attempt"], paths["before"], paths["after"])


def test_loopback_attestation_requires_both_live_roles_and_matching_bindings(
    tmp_path: Path,
) -> None:
    paths = _fixture(tmp_path)
    value = json.loads(paths["loopback"].read_text(encoding="utf-8"))
    value["processes"] = [{"role": "gazebo_sim"}]
    _write(paths["loopback"], value)
    with pytest.raises(MODULE.ProbeError, match="process roles"):
        MODULE.finalize(paths["attempt"], paths["before"], paths["after"])
    paths = _fixture(tmp_path / "binding")
    value = json.loads(paths["loopback"].read_text(encoding="utf-8"))
    value["acceptance_session"]["sha256"] = "0" * 64
    _write(paths["loopback"], value)
    with pytest.raises(MODULE.ProbeError, match="loopback/sensor session"):
        MODULE.finalize(paths["attempt"], paths["before"], paths["after"])


def test_evidence_timeline_and_watchdog_sample_syntax_fail_closed(tmp_path: Path) -> None:
    paths = _fixture(tmp_path)
    _write(paths["after"], _start_gate(2))
    with pytest.raises(MODULE.ProbeError, match="strictly ordered"):
        MODULE.finalize(paths["attempt"], paths["before"], paths["after"])
    paths = _fixture(tmp_path / "syntax")
    paths["watchdog_log"].write_text("2026-09-01T00:00:01Z sample not-a-valid-record\n", encoding="utf-8")
    with pytest.raises(MODULE.ProbeError, match="malformed"):
        MODULE.finalize(paths["attempt"], paths["before"], paths["after"])


def test_watchdog_allows_equal_whole_second_sample_timestamps(tmp_path: Path) -> None:
    paths = _fixture(tmp_path)
    lines = _watchdog_log().splitlines()
    lines[1] = lines[1].replace("2026-09-01T00:00:02Z", "2026-09-01T00:00:01Z")
    paths["watchdog_log"].write_text("\n".join(lines) + "\n", encoding="utf-8")
    assert MODULE.finalize(paths["attempt"], paths["before"], paths["after"])["passed"] is True


def test_exclusive_success_and_blocked_outputs_are_never_overwritten(tmp_path: Path) -> None:
    paths = _fixture(tmp_path)
    output = tmp_path / "result.json"
    MODULE._write_exclusive(output, MODULE.finalize(paths["attempt"], paths["before"], paths["after"]))
    with pytest.raises(MODULE.ProbeError, match="stale"):
        MODULE._write_exclusive(output, {"replacement": True})
