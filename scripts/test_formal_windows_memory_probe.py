from __future__ import annotations

import json
import inspect
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

import formal_windows_memory_probe as probe
from formal_windows_memory_probe import ProbeError, parse_powershell_sample


FIXTURE = {
    "epoch_ns": 1_777_000_000_000_000_000,
    "commit_limit_bytes": 75_311_267_840,
    "commit_charge_bytes": 69_793_267_840,
    "commit_available_bytes": 5_518_000_000,
    "docker_private_bytes": 56_000_000_000,
    "vmmem_wsl_private_bytes": 5_000_000_000,
}

POOL_TAG_FIXTURE = {
    **FIXTURE,
    "nonpaged_pool_bytes": 23_429_971_456,
    "nonpaged_pool_status": "available",
    "pool_tag_diagnostics": {
        "status": "available",
        "tracked_nonpaged_bytes": {
            "Nbuf": 14_667_153_408,
            "Nnbl": 3_414_327_296,
            "Nnbf": 1_771_675_648,
        },
        "tracked_nonpaged_bytes_total": 19_853_156_352,
        "suspected_ndis_nonpaged_pool_leak": True,
    },
}


def test_parses_realistic_powershell_commit_and_docker_fixture() -> None:
    assert parse_powershell_sample(json.dumps(FIXTURE)) == FIXTURE


def test_parser_preserves_readonly_nonpaged_pool_and_ndis_suspicion_fixture() -> None:
    assert parse_powershell_sample(json.dumps(POOL_TAG_FIXTURE)) == POOL_TAG_FIXTURE


@pytest.mark.parametrize(
    "mutation",
    (
        {"nonpaged_pool_status": "broken"},
        {
            "pool_tag_diagnostics": {
                **POOL_TAG_FIXTURE["pool_tag_diagnostics"],
                "tracked_nonpaged_bytes_total": 1,
            }
        },
    ),
)
def test_parser_rejects_malformed_pool_tag_diagnostics(
    mutation: dict[str, object],
) -> None:
    payload = {**POOL_TAG_FIXTURE, **mutation}
    with pytest.raises(ProbeError):
        parse_powershell_sample(json.dumps(payload))


def test_parser_preserves_structured_pool_tag_query_failure() -> None:
    payload = {
        **POOL_TAG_FIXTURE,
        "pool_tag_diagnostics": {
            "status": "unavailable",
            "failure": {"kind": "ntquery_failed", "nt_status": "0xC0000001"},
        },
    }
    parsed = parse_powershell_sample(json.dumps(payload))
    assert parsed["pool_tag_diagnostics"] == payload["pool_tag_diagnostics"]


@pytest.mark.parametrize(
    "mutation",
    (
        {"commit_available_bytes": 1},
        {"commit_charge_bytes": 80_000_000_000},
        {"docker_private_bytes": -1},
        {"vmmem_wsl_private_bytes": -1},
        {"epoch_ns": "1777"},
    ),
)
def test_parser_rejects_inconsistent_or_non_integer_powershell_data(
    mutation: dict[str, object],
) -> None:
    payload = {**FIXTURE, **mutation}
    with pytest.raises(ProbeError):
        parse_powershell_sample(json.dumps(payload))


def test_parser_accepts_utf8_bom_from_windows_powershell() -> None:
    assert parse_powershell_sample("\ufeff" + json.dumps(FIXTURE)) == FIXTURE


def test_parser_rejects_values_outside_uint64_domain() -> None:
    payload = {**FIXTURE, "docker_private_bytes": 1 << 64}
    with pytest.raises(ProbeError, match="invalid docker_private_bytes"):
        parse_powershell_sample(json.dumps(payload))


def test_probe_uses_one_native_memory_query_process_without_cim() -> None:
    command = probe.powershell_command(stream=True, interval_s=1.0)
    script = command[-1]
    assert "GlobalMemoryStatusEx" in script
    assert "ullTotalPageFile" in script
    assert "ullAvailPageFile" in script
    assert "GetProcessesByName('com.docker.backend')" in script
    assert "GetProcessesByName('vmmemWSL')" in script
    assert "GetPerformanceInfo" in script
    assert "NtQuerySystemInformation" in script
    assert "SystemPoolTagInformation = 22" in script
    assert "Nbuf" in script and "Nnbl" in script and "Nnbf" in script
    assert "MaxPoolTagBufferBytes" in script
    assert "Get-CimInstance" not in script
    assert "Win32_PerfRawData_PerfOS_Memory" not in script
    assert "Restart-NetAdapter" not in script
    assert "Disable-NetAdapter" not in script
    assert script.count("Add-Type -TypeDefinition") == 1
    assert "while ($true) { Emit-Sample; Start-Sleep -Milliseconds 1000 }" in script


def test_stream_has_one_bounded_restart_for_transient_windows_query_exit() -> None:
    source = Path(probe.__file__).read_text(encoding="utf-8")
    assert 'env_uint("FORMAL_WINDOWS_STREAM_MAX_RESTARTS", 1)' in source
    assert "bounded restart" in source
    assert "exhausted bounded restarts" in source


def test_stream_record_is_one_ascii_ten_field_write(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    writes: list[tuple[int, bytes]] = []

    def record_write(fd: int, payload: bytes) -> int:
        writes.append((fd, payload))
        return len(payload)

    monkeypatch.setattr(probe.os, "write", record_write)
    probe._write_stream_record(POOL_TAG_FIXTURE)
    assert len(writes) == 1
    assert writes[0][0] == sys.stdout.fileno()
    payload = writes[0][1]
    assert payload.endswith(b"\n")
    assert len(payload.decode("ascii").split()) == 10


def test_stream_record_short_write_fails_closed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(probe.os, "write", lambda unused_fd, unused_payload: 1)
    with pytest.raises(ProbeError, match="write was short"):
        probe._write_stream_record(FIXTURE)


def test_stream_sequence_is_strictly_increasing_across_clock_rollback() -> None:
    first, first_sequence = probe._with_next_stream_sequence(FIXTURE, 0)
    wall_clock_rolled_back = {
        **FIXTURE,
        "epoch_ns": FIXTURE["epoch_ns"] - 5_000_000_000,
    }
    second, second_sequence = probe._with_next_stream_sequence(
        wall_clock_rolled_back,
        first_sequence,
    )
    assert first["epoch_ns"] == 1
    assert second["epoch_ns"] == 2
    assert second_sequence > first_sequence


def test_one_shot_query_retries_once_then_returns_valid_sample(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[int] = []
    results = iter(
        (
            subprocess.CompletedProcess([], 1, stdout="detail on stdout", stderr=""),
            subprocess.CompletedProcess([], 0, stdout=json.dumps(FIXTURE), stderr=""),
        )
    )

    def run_once(*unused_args: object, **unused_kwargs: object) -> subprocess.CompletedProcess[str]:
        calls.append(1)
        return next(results)

    sleeps: list[float] = []
    monkeypatch.setattr(probe.subprocess, "run", run_once)
    monkeypatch.setattr(probe.time, "sleep", sleeps.append)
    assert probe.read_once() == FIXTURE
    assert len(calls) == 2
    assert sleeps == [probe.READ_ONCE_RETRY_DELAY_S]


def test_one_shot_query_fails_after_exactly_one_bounded_retry(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[int] = []

    def fail(*unused_args: object, **unused_kwargs: object) -> subprocess.CompletedProcess[str]:
        calls.append(1)
        return subprocess.CompletedProcess([], 1, stdout="native failure", stderr="")

    monkeypatch.setattr(probe.subprocess, "run", fail)
    monkeypatch.setattr(probe.time, "sleep", lambda unused_seconds: None)
    with pytest.raises(ProbeError, match="stdout=native failure"):
        probe.read_once()
    assert len(calls) == probe.READ_ONCE_MAX_ATTEMPTS


def test_stream_inherits_stderr_and_never_uses_multi_argument_print_for_records() -> None:
    source = inspect.getsource(probe.stream)
    assert "stderr=None" in source
    assert "_write_stream_record(stream_sample)" in source
    assert "stderr=subprocess.PIPE" not in source


def test_native_probe_fails_closed_on_impossible_available_commit() -> None:
    script = probe.powershell_command(stream=False, interval_s=1.0)[-1]
    assert "if ($available -gt $limit)" in script
    assert "throw 'GlobalMemoryStatusEx reported available page file above total page file'" in script
    assert "$charge = $limit - $available" in script


def test_start_gate_records_threshold_breach_without_touching_docker(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(probe, "read_once", lambda: dict(FIXTURE))
    output = tmp_path / "start_gate.json"
    assert probe.check_start(output) == 86
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "FORMAL_WINDOWS_MEMORY_START_REFUSED"
    assert payload["passed"] is False
    assert payload["checks"] == {
        "windows_commit_available_at_least_configured_minimum": False,
        "docker_private_at_most_configured_maximum": False,
        "wsl_vm_stopped_when_required": True,
        "wsl_vm_running_when_required": True,
        "no_suspected_ndis_nonpaged_pool_leak": True,
    }
    assert payload["docker_was_signalled_or_stopped"] is False
    assert payload["commit_recovery"]["attempted"] is False


def test_start_gate_refuses_a_suspected_ndis_nonpaged_pool_leak_without_retry(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    sample = {
        **POOL_TAG_FIXTURE,
        "commit_charge_bytes": 30_000_000_000,
        "commit_available_bytes": 45_311_267_840,
        "docker_private_bytes": 0,
        "vmmem_wsl_private_bytes": 0,
    }
    monkeypatch.setattr(probe, "read_once", lambda: sample)
    output = tmp_path / "ndis_refused_start_gate.json"
    assert probe.check_start(output) == 86
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["passed"] is False
    assert payload["checks"]["no_suspected_ndis_nonpaged_pool_leak"] is False
    assert payload["commit_recovery"]["attempted"] is False


def test_start_gate_retries_only_a_transient_commit_shortfall_and_recovers(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    low_commit = {
        **FIXTURE,
        "commit_charge_bytes": 70_000_000_000,
        "commit_available_bytes": 5_311_267_840,
        "docker_private_bytes": 0,
        "vmmem_wsl_private_bytes": 0,
    }
    recovered = {
        **low_commit,
        "commit_charge_bytes": 50_000_000_000,
        "commit_available_bytes": 25_311_267_840,
    }
    samples = iter((low_commit, recovered))
    sleeps: list[float] = []
    clock = [0.0]

    monkeypatch.setattr(probe, "read_once", lambda: next(samples))

    def advance(seconds: float) -> None:
        sleeps.append(seconds)
        clock[0] += seconds

    output = tmp_path / "recovered_start_gate.json"
    assert probe.check_start(output, monotonic=lambda: clock[0], sleep=advance) == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert sleeps == [probe.START_COMMIT_RECOVERY_INTERVAL_S]
    assert payload["sample"] == recovered
    assert payload["commit_recovery"] == {
        "timeout_s": probe.START_COMMIT_RECOVERY_TIMEOUT_S,
        "interval_s": probe.START_COMMIT_RECOVERY_INTERVAL_S,
        "sample_count": 2,
        "attempted": True,
        "recovered": True,
    }


def test_start_gate_fails_closed_after_bounded_commit_recovery_wait(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    low_commit = {
        **FIXTURE,
        "commit_charge_bytes": 70_000_000_000,
        "commit_available_bytes": 5_311_267_840,
        "docker_private_bytes": 0,
        "vmmem_wsl_private_bytes": 0,
    }
    samples = iter((low_commit,) * 100)
    sleeps: list[float] = []
    clock = [0.0]

    monkeypatch.setattr(probe, "read_once", lambda: next(samples))

    def advance(seconds: float) -> None:
        sleeps.append(seconds)
        clock[0] += seconds

    output = tmp_path / "refused_start_gate.json"
    assert probe.check_start(output, monotonic=lambda: clock[0], sleep=advance) == 86
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert sum(sleeps) == probe.START_COMMIT_RECOVERY_TIMEOUT_S
    assert len(sleeps) == int(
        probe.START_COMMIT_RECOVERY_TIMEOUT_S
        / probe.START_COMMIT_RECOVERY_INTERVAL_S
    )
    assert payload["status"] == "FORMAL_WINDOWS_MEMORY_START_REFUSED"
    assert payload["checks"]["windows_commit_available_at_least_configured_minimum"] is False
    assert payload["commit_recovery"]["sample_count"] == len(sleeps) + 1
    assert payload["commit_recovery"]["attempted"] is True
    assert payload["commit_recovery"]["recovered"] is False


def test_cold_start_gate_rejects_a_running_wsl_vm(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    sample = {
        **FIXTURE,
        "commit_charge_bytes": 40_000_000_000,
        "commit_available_bytes": 35_311_267_840,
        "docker_private_bytes": 0,
    }
    monkeypatch.setattr(probe, "read_once", lambda: sample)
    monkeypatch.setenv("FORMAL_WINDOWS_START_REQUIRE_WSL_STOPPED", "1")
    output = tmp_path / "cold_start_gate.json"
    assert probe.check_start(output) == 86
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["require_wsl_stopped"] is True
    assert payload["require_wsl_running"] is False
    assert payload["checks"]["wsl_vm_stopped_when_required"] is False


def test_warm_start_gate_requires_the_wsl_vm_to_still_be_running(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    sample = {
        **FIXTURE,
        "commit_charge_bytes": 40_000_000_000,
        "commit_available_bytes": 35_311_267_840,
        "docker_private_bytes": 0,
        "vmmem_wsl_private_bytes": 0,
    }
    monkeypatch.setattr(probe, "read_once", lambda: sample)
    monkeypatch.setenv("FORMAL_WINDOWS_START_REQUIRE_WSL_RUNNING", "1")
    output = tmp_path / "warm_start_gate.json"
    assert probe.check_start(output) == 86
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["require_wsl_running"] is True
    assert payload["checks"]["wsl_vm_running_when_required"] is False


def test_start_gate_rejects_contradictory_wsl_state_requirements(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(probe, "read_once", lambda: dict(FIXTURE))
    monkeypatch.setenv("FORMAL_WINDOWS_START_REQUIRE_WSL_STOPPED", "1")
    monkeypatch.setenv("FORMAL_WINDOWS_START_REQUIRE_WSL_RUNNING", "1")
    with pytest.raises(ProbeError, match="both stopped and running"):
        probe.check_start(tmp_path / "invalid.json")


def test_start_gate_rejects_non_boolean_wsl_requirement(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(probe, "read_once", lambda: dict(FIXTURE))
    monkeypatch.setenv("FORMAL_WINDOWS_START_REQUIRE_WSL_STOPPED", "2")
    with pytest.raises(ProbeError, match="must be 0 or 1"):
        probe.check_start(tmp_path / "invalid.json")


def test_cli_query_failure_returns_125_and_emits_no_false_pass(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "start_gate.json"

    def fail_query() -> dict[str, int]:
        raise ProbeError("injected query failure")

    monkeypatch.setattr(probe, "read_once", fail_query)
    monkeypatch.setattr(
        sys,
        "argv",
        ["formal_windows_memory_probe.py", "--check-start", "--output", str(output)],
    )
    assert probe.main() == 125
    assert not output.exists()


@pytest.mark.skipif(os.name != "posix", reason="requires POSIX signal semantics")
def test_stream_sigterm_reaps_its_exact_query_child(tmp_path: Path) -> None:
    fake = tmp_path / "fake-powershell"
    fake.write_text(
        """#!/usr/bin/env bash
set -eu
printf '%s\\n' \"$$\" >\"${FAKE_PS_PID_FILE}"
trap 'exit 0' INT TERM HUP
while :; do
  printf '%s\\n' '{"epoch_ns":1777000000000000000,"commit_limit_bytes":80000000000,"commit_charge_bytes":20000000000,"commit_available_bytes":60000000000,"docker_private_bytes":0,"vmmem_wsl_private_bytes":0}'
  sleep 0.1
done
""",
        encoding="utf-8",
    )
    fake.chmod(0o755)
    pid_file = tmp_path / "fake-powershell.pid"
    environment = {
        **os.environ,
        "FORMAL_WINDOWS_POWERSHELL": str(fake),
        "FAKE_PS_PID_FILE": str(pid_file),
    }
    process = subprocess.Popen(
        [
            sys.executable,
            str(probe.__file__),
            "--stream",
            "--interval-s",
            "0.1",
        ],
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        deadline = time.monotonic() + 3.0
        while not pid_file.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert pid_file.is_file()
        assert process.stdout is not None
        assert len(process.stdout.readline().split()) == 10
        child_pid = int(pid_file.read_text(encoding="utf-8").strip())
        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=5) == 0
        with pytest.raises(ProcessLookupError):
            os.kill(child_pid, 0)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3)
