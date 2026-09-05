from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
WATCHDOG = ROOT / "scripts/formal_memory_watchdog.sh"
BUILD = ROOT / "scripts/build_formal_final_runtime.sh"
ISOLATION = ROOT / "scripts/run_formal_runtime_isolation.sh"


def test_watchdog_defaults_and_exact_group_signal_contract() -> None:
    source = WATCHDOG.read_text(encoding="utf-8")
    assert "FORMAL_MEMORY_MIN_AVAILABLE_KIB:-3145728" in source
    assert "FORMAL_MEMORY_MAX_SWAP_USED_KIB:-1048576" in source
    assert "FORMAL_MEMORY_MAX_GROUP_RSS_KIB:-9437184" in source
    assert "FORMAL_WINDOWS_RUNTIME_MIN_COMMIT_AVAILABLE_BYTES:-6442450944" in source
    assert "FORMAL_WINDOWS_RUNTIME_MAX_DOCKER_PRIVATE_BYTES:-8589934592" in source
    assert "FORMAL_WINDOWS_PROBE_TIMEOUT_S:-5" in source
    assert "FORMAL_WINDOWS_PROBE_TRANSIENT_RETRIES:-1" in source
    assert "windows_probe_failure_streak > windows_probe_transient_retries" in source
    assert "Windows memory probe transient failure kind=" in source
    assert 'sample_failure_kind=stdout_timeout' in source
    assert 'sample_failure_kind=field_count' in source
    assert 'sample_failure_kind=non_uint' in source
    assert 'sample_failure_kind=non_monotonic_epoch' in source
    assert 'sample_failure_kind=commit_invariant' in source
    assert 'sample_failure_kind=pool_diagnostic_invariant' in source
    assert 'sample_failure_kind=post_timeout_duplicate' in source
    assert '"${previous_windows_probe_attempt_failure_kind}" == stdout_timeout' in source
    assert 'post_timeout_duplicate_grace=true' in source
    assert '"last_probe_failure"' in source
    assert "Six fields are accepted only for older probe binaries" in source
    assert '"nonpaged_pool"' in source
    assert '"suspected_ndis_nonpaged_pool_leak"' in source
    assert '"suspected_ndis_nonpaged_pool_leak": %s' in source
    assert '"${suspected_ndis_pool_leak}" == true' in source
    assert 'kill -INT -- "-${target_pgid}"' in source
    assert 'kill -TERM -- "-${target_pgid}"' in source
    assert "docker_signalled_or_stopped\": false" in source
    assert "pkill" not in source
    assert "killall" not in source
    isolation = ISOLATION.read_text(encoding="utf-8")
    assert "formal_windows_memory_probe.py\" --check-start" in isolation
    assert "FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT=125" in isolation


def test_final_copy_overlay_build_is_fresh_and_defaults_to_one_worker() -> None:
    source = BUILD.read_text(encoding="utf-8")
    assert "FORMAL_COLCON_PARALLEL_WORKERS:-1" in source
    assert '--executor parallel --parallel-workers "${parallel_workers}"' in source
    assert 'export CMAKE_BUILD_PARALLEL_LEVEL="${parallel_workers}"' in source
    assert 'export MAKEFLAGS="-j${parallel_workers}"' in source
    assert "--merge-install" in source
    assert "--symlink-install" not in source
    assert "refusing non-fresh final runtime path" in source


@pytest.mark.skipif(os.name != "posix", reason="requires Linux /proc process groups")
def test_low_test_threshold_stops_only_target_group(tmp_path: Path) -> None:
    unrelated = subprocess.Popen(
        ["bash", "-c", "trap 'exit 0' INT TERM; while :; do sleep 1; done"],
        start_new_session=True,
    )
    target = subprocess.Popen(
        ["bash", "-c", "trap 'exit 0' INT TERM; while :; do sleep 1; done"],
        start_new_session=True,
    )
    evidence = tmp_path / "memory.json"
    log = tmp_path / "memory.log"
    environment = {
        **os.environ,
        "FORMAL_WINDOWS_MEMORY_GUARD_ENABLED": "0",
        "FORMAL_MEMORY_MIN_AVAILABLE_KIB": "0",
        "FORMAL_MEMORY_MAX_SWAP_USED_KIB": str(2**63 - 1),
        "FORMAL_MEMORY_MAX_GROUP_RSS_KIB": "1",
        "FORMAL_MEMORY_POLL_S": "0.1",
        "FORMAL_MEMORY_INT_GRACE_S": "0.2",
        "FORMAL_MEMORY_TERM_GRACE_S": "0.2",
    }
    try:
        result = subprocess.run(
            [
                "bash",
                str(WATCHDOG),
                "--leader-pid",
                str(target.pid),
                "--pgid",
                str(target.pid),
                "--json",
                str(evidence),
                "--log",
                str(log),
            ],
            env=environment,
            timeout=10,
            check=False,
        )
        assert result.returncode == 86
        assert target.wait(timeout=3) == 0
        assert unrelated.poll() is None
        payload = json.loads(evidence.read_text(encoding="utf-8"))
        assert payload["status"] == "FORMAL_MEMORY_LIMIT_BREACHED"
        assert payload["reasons"]["excessive_group_rss"] is True
        assert payload["signals"]["exact_pgid_only"] is True
        assert payload["signals"]["docker_signalled_or_stopped"] is False
    finally:
        if unrelated.poll() is None:
            os.killpg(unrelated.pid, signal.SIGTERM)
            unrelated.wait(timeout=3)
        if target.poll() is None:
            os.killpg(target.pid, signal.SIGTERM)
            target.wait(timeout=3)


@pytest.mark.skipif(os.name != "posix", reason="requires Linux /proc process groups")
def test_windows_probe_failure_fails_closed_and_stops_only_target_group(
    tmp_path: Path,
) -> None:
    unrelated = subprocess.Popen(
        ["bash", "-c", "trap 'exit 0' INT TERM; while :; do sleep 1; done"],
        start_new_session=True,
    )
    target = subprocess.Popen(
        ["bash", "-c", "trap 'exit 0' INT TERM; while :; do sleep 1; done"],
        start_new_session=True,
    )
    evidence = tmp_path / "memory.json"
    log = tmp_path / "memory.log"
    environment = {
        **os.environ,
        "FORMAL_WINDOWS_POWERSHELL": "/bin/false",
        "FORMAL_MEMORY_MIN_AVAILABLE_KIB": "0",
        "FORMAL_MEMORY_MAX_SWAP_USED_KIB": str(2**63 - 1),
        "FORMAL_MEMORY_MAX_GROUP_RSS_KIB": str(2**63 - 1),
        "FORMAL_WINDOWS_PROBE_TIMEOUT_S": "0.5",
        "FORMAL_MEMORY_INT_GRACE_S": "0.2",
        "FORMAL_MEMORY_TERM_GRACE_S": "0.2",
    }
    try:
        result = subprocess.run(
            [
                "bash",
                str(WATCHDOG),
                "--leader-pid",
                str(target.pid),
                "--pgid",
                str(target.pid),
                "--json",
                str(evidence),
                "--log",
                str(log),
            ],
            env=environment,
            timeout=10,
            check=False,
        )
        assert result.returncode == 125
        assert target.wait(timeout=3) == 0
        assert unrelated.poll() is None
        payload = json.loads(evidence.read_text(encoding="utf-8"))
        assert payload["status"] == "FORMAL_WINDOWS_MEMORY_PROBE_FAILED_CLOSED"
        assert payload["breach_exit_code"] == 125
        assert payload["reasons"]["windows_probe_failed"] is True
        assert payload["signals"]["exact_pgid_only"] is True
        assert payload["signals"]["docker_signalled_or_stopped"] is False
    finally:
        if unrelated.poll() is None:
            os.killpg(unrelated.pid, signal.SIGTERM)
            unrelated.wait(timeout=3)
        if target.poll() is None:
            os.killpg(target.pid, signal.SIGTERM)
            target.wait(timeout=3)


@pytest.mark.skipif(os.name != "posix", reason="requires Linux /proc process groups")
def test_suspected_ndis_pool_leak_stops_only_target_group(tmp_path: Path) -> None:
    fake = tmp_path / "fake-powershell"
    fake.write_text(
        """#!/usr/bin/env bash
set -eu
while :; do
  printf '%s\n' '{"epoch_ns":1777000000000000000,"commit_limit_bytes":80000000000,"commit_charge_bytes":20000000000,"commit_available_bytes":60000000000,"docker_private_bytes":0,"vmmem_wsl_private_bytes":5000000000,"nonpaged_pool_bytes":3000000000,"nonpaged_pool_status":"available","pool_tag_diagnostics":{"status":"available","tracked_nonpaged_bytes":{"Nbuf":1100000000,"Nnbl":100000000,"Nnbf":50000000},"tracked_nonpaged_bytes_total":1250000000,"suspected_ndis_nonpaged_pool_leak":true}}'
  sleep 0.1
done
""",
        encoding="utf-8",
    )
    fake.chmod(0o755)
    unrelated = subprocess.Popen(
        ["bash", "-c", "trap 'exit 0' INT TERM; while :; do sleep 1; done"],
        start_new_session=True,
    )
    target = subprocess.Popen(
        ["bash", "-c", "trap 'exit 0' INT TERM; while :; do sleep 1; done"],
        start_new_session=True,
    )
    evidence = tmp_path / "memory.json"
    log = tmp_path / "memory.log"
    environment = {
        **os.environ,
        "FORMAL_WINDOWS_POWERSHELL": str(fake),
        "FORMAL_MEMORY_MIN_AVAILABLE_KIB": "0",
        "FORMAL_MEMORY_MAX_SWAP_USED_KIB": str(2**63 - 1),
        "FORMAL_MEMORY_MAX_GROUP_RSS_KIB": str(2**63 - 1),
        "FORMAL_WINDOWS_PROBE_TIMEOUT_S": "1",
        "FORMAL_MEMORY_INT_GRACE_S": "0.2",
        "FORMAL_MEMORY_TERM_GRACE_S": "0.2",
    }
    try:
        result = subprocess.run(
            [
                "bash",
                str(WATCHDOG),
                "--leader-pid",
                str(target.pid),
                "--pgid",
                str(target.pid),
                "--json",
                str(evidence),
                "--log",
                str(log),
            ],
            env=environment,
            timeout=10,
            check=False,
        )
        assert result.returncode == 86
        assert target.wait(timeout=3) == 0
        assert unrelated.poll() is None
        payload = json.loads(evidence.read_text(encoding="utf-8"))
        assert payload["status"] == "FORMAL_MEMORY_LIMIT_BREACHED"
        assert payload["reasons"]["suspected_ndis_nonpaged_pool_leak"] is True
        assert payload["windows_diagnostics"]["suspected_ndis_nonpaged_pool_leak"] is True
        assert payload["signals"]["exact_pgid_only"] is True
        assert payload["signals"]["docker_signalled_or_stopped"] is False
    finally:
        if unrelated.poll() is None:
            os.killpg(unrelated.pid, signal.SIGTERM)
            unrelated.wait(timeout=3)
        if target.poll() is None:
            os.killpg(target.pid, signal.SIGTERM)
            target.wait(timeout=3)


@pytest.mark.skipif(os.name != "posix", reason="requires Linux /proc process groups")
def test_one_windows_probe_timeout_recovers_without_signalling_target(
    tmp_path: Path,
) -> None:
    fake = tmp_path / "fake-powershell"
    fake.write_text(
        """#!/usr/bin/env bash
set -eu
sleep 0.35
epoch=1777000000000000000
while :; do
  printf '{"epoch_ns":%s,"commit_limit_bytes":80000000000,"commit_charge_bytes":20000000000,"commit_available_bytes":60000000000,"docker_private_bytes":0,"vmmem_wsl_private_bytes":0}\\n' "${epoch}"
  epoch=$((epoch + 100000000))
  sleep 0.1
done
""",
        encoding="utf-8",
    )
    fake.chmod(0o755)
    target = subprocess.Popen(
        ["bash", "-c", "trap 'exit 0' INT TERM; while :; do sleep 1; done"],
        start_new_session=True,
    )
    evidence = tmp_path / "memory.json"
    log = tmp_path / "memory.log"
    environment = {
        **os.environ,
        "FORMAL_WINDOWS_POWERSHELL": str(fake),
        "FORMAL_MEMORY_MIN_AVAILABLE_KIB": "0",
        "FORMAL_MEMORY_MAX_SWAP_USED_KIB": str(2**63 - 1),
        "FORMAL_MEMORY_MAX_GROUP_RSS_KIB": str(2**63 - 1),
        "FORMAL_WINDOWS_PROBE_TIMEOUT_S": "0.2",
        "FORMAL_WINDOWS_PROBE_TRANSIENT_RETRIES": "1",
        "FORMAL_MEMORY_POLL_S": "0.1",
    }
    watchdog = subprocess.Popen(
        [
            "bash",
            str(WATCHDOG),
            "--leader-pid",
            str(target.pid),
            "--pgid",
            str(target.pid),
            "--json",
            str(evidence),
            "--log",
            str(log),
        ],
        env=environment,
    )
    try:
        deadline = time.monotonic() + 5.0
        recovered = False
        while time.monotonic() < deadline:
            text = log.read_text(encoding="utf-8") if log.exists() else ""
            recovered = (
                "streak=1/2" in text
                and "windows_commit_available_bytes=60000000000" in text
            )
            if recovered:
                break
            time.sleep(0.05)
        assert recovered
        watchdog.send_signal(signal.SIGTERM)
        assert watchdog.wait(timeout=5) == 0
        assert target.poll() is None
        payload = json.loads(evidence.read_text(encoding="utf-8"))
        assert payload["status"] == "FORMAL_MEMORY_WATCHDOG_STOPPED"
        assert payload["signals"]["sigint_sent"] is False
        assert payload["windows_observed_bytes"]["nonpaged_pool"] == 0
        assert payload["windows_diagnostics"]["pool_tag_query_available"] is False
        assert payload["windows_diagnostics"]["last_probe_failure"] == {
            "kind": "stdout_timeout",
            "read_rc": 142,
            "field_count": 0,
            "failure_streak": 1,
            "probe_alive_at_check": True,
            "had_prior_valid_sample": False,
            "rejected_sequence": 0,
            "previous_accepted_sequence": 0,
        }
    finally:
        if watchdog.poll() is None:
            watchdog.terminate()
            watchdog.wait(timeout=3)
        if target.poll() is None:
            os.killpg(target.pid, signal.SIGTERM)
            target.wait(timeout=3)


@pytest.mark.skipif(os.name != "posix", reason="requires Linux /proc process groups")
def test_malformed_partial_probe_record_reports_field_count_and_fails_closed(
    tmp_path: Path,
) -> None:
    wrapper = tmp_path / "partial-probe.py"
    wrapper.write_text(
        """#!/usr/bin/env python3
import os, sys, time
os.write(sys.stdout.fileno(), b'1777000000000000000 80000000000 20000000000')
time.sleep(0.3)
os.write(sys.stdout.fileno(), b' 60000000000 0 0 0 0 0 0\\n')
time.sleep(5)
""",
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    target = subprocess.Popen(
        ["bash", "-c", "trap 'exit 0' INT TERM; while :; do sleep 1; done"],
        start_new_session=True,
    )
    evidence = tmp_path / "memory.json"
    log = tmp_path / "memory.log"
    # Replace the watchdog's Python command while leaving the target group and
    # all resource thresholds unchanged.  The wrapper reproduces the r36
    # half-record timeout sequence.
    helper = tmp_path / "scripts"
    helper.mkdir()
    watchdog_copy = helper / WATCHDOG.name
    watchdog_copy.write_text(
        WATCHDOG.read_text(encoding="utf-8").replace(
            'exec python3 "${helper_dir}/formal_windows_memory_probe.py" --stream --interval-s "${poll_s}"',
            f'exec "{wrapper}"',
        ),
        encoding="utf-8",
    )
    watchdog_copy.chmod(0o755)
    environment = {
        **os.environ,
        "FORMAL_MEMORY_MIN_AVAILABLE_KIB": "0",
        "FORMAL_MEMORY_MAX_SWAP_USED_KIB": str(2**63 - 1),
        "FORMAL_MEMORY_MAX_GROUP_RSS_KIB": str(2**63 - 1),
        "FORMAL_WINDOWS_PROBE_TIMEOUT_S": "0.2",
        "FORMAL_WINDOWS_PROBE_TRANSIENT_RETRIES": "1",
        "FORMAL_MEMORY_INT_GRACE_S": "0.2",
        "FORMAL_MEMORY_TERM_GRACE_S": "0.2",
    }
    try:
        result = subprocess.run(
            [
                "bash",
                str(watchdog_copy),
                "--leader-pid",
                str(target.pid),
                "--pgid",
                str(target.pid),
                "--json",
                str(evidence),
                "--log",
                str(log),
            ],
            env=environment,
            timeout=10,
            check=False,
        )
        assert result.returncode == 125
        assert target.wait(timeout=3) == 0
        payload = json.loads(evidence.read_text(encoding="utf-8"))
        assert payload["status"] == "FORMAL_WINDOWS_MEMORY_PROBE_FAILED_CLOSED"
        assert payload["reasons"] == {
            "low_mem_available": False,
            "excessive_swap_used": False,
            "excessive_group_rss": False,
            "low_windows_commit_available": False,
            "excessive_docker_private": False,
            "windows_probe_failed": True,
            "suspected_ndis_nonpaged_pool_leak": False,
        }
        failure = payload["windows_diagnostics"]["last_probe_failure"]
        assert failure["kind"] == "field_count"
        assert failure["read_rc"] == 125
        assert failure["field_count"] == 7
        assert failure["failure_streak"] == 2
    finally:
        if target.poll() is None:
            os.killpg(target.pid, signal.SIGTERM)
            target.wait(timeout=3)


@pytest.mark.skipif(os.name != "posix", reason="requires Linux /proc process groups")
def test_one_duplicate_sequence_after_timeout_gets_exactly_one_grace(
    tmp_path: Path,
) -> None:
    wrapper = tmp_path / "timeout-duplicate-probe.py"
    wrapper.write_text(
        """#!/usr/bin/env python3
import os, sys, time
def emit(sequence):
    row = f"{sequence} 80000000000 20000000000 60000000000 0 0 0 0 0 0\\n"
    os.write(sys.stdout.fileno(), row.encode("ascii"))
emit(1)
time.sleep(0.35)
emit(1)
time.sleep(0.05)
emit(2)
sequence = 2
while True:
    time.sleep(0.1)
    sequence += 1
    emit(sequence)
""",
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    helper = tmp_path / "scripts"
    helper.mkdir()
    watchdog_copy = helper / WATCHDOG.name
    watchdog_copy.write_text(
        WATCHDOG.read_text(encoding="utf-8").replace(
            'exec python3 "${helper_dir}/formal_windows_memory_probe.py" --stream --interval-s "${poll_s}"',
            f'exec "{wrapper}"',
        ),
        encoding="utf-8",
    )
    watchdog_copy.chmod(0o755)
    target = subprocess.Popen(
        ["bash", "-c", "trap 'exit 0' INT TERM; while :; do sleep 1; done"],
        start_new_session=True,
    )
    evidence = tmp_path / "memory.json"
    log = tmp_path / "memory.log"
    environment = {
        **os.environ,
        "FORMAL_MEMORY_MIN_AVAILABLE_KIB": "0",
        "FORMAL_MEMORY_MAX_SWAP_USED_KIB": str(2**63 - 1),
        "FORMAL_MEMORY_MAX_GROUP_RSS_KIB": str(2**63 - 1),
        "FORMAL_WINDOWS_PROBE_TIMEOUT_S": "0.2",
        "FORMAL_WINDOWS_PROBE_TRANSIENT_RETRIES": "1",
        "FORMAL_MEMORY_POLL_S": "0.1",
    }
    watchdog = subprocess.Popen(
        [
            "bash",
            str(watchdog_copy),
            "--leader-pid",
            str(target.pid),
            "--pgid",
            str(target.pid),
            "--json",
            str(evidence),
            "--log",
            str(log),
        ],
        env=environment,
    )
    try:
        deadline = time.monotonic() + 5.0
        recovered = False
        while time.monotonic() < deadline:
            content = log.read_text(encoding="utf-8") if log.exists() else ""
            recovered = (
                "kind=stdout_timeout" in content
                and "kind=post_timeout_duplicate" in content
                and "previous_accepted_sequence=1" in content
                and "windows_commit_available_bytes=60000000000" in content
            )
            if recovered:
                break
            time.sleep(0.05)
        assert recovered
        watchdog.send_signal(signal.SIGTERM)
        assert watchdog.wait(timeout=5) == 0
        assert target.poll() is None
        payload = json.loads(evidence.read_text(encoding="utf-8"))
        assert payload["status"] == "FORMAL_MEMORY_WATCHDOG_STOPPED"
        failure = payload["windows_diagnostics"]["last_probe_failure"]
        assert failure["kind"] == "post_timeout_duplicate"
        assert failure["rejected_sequence"] == 1
        assert failure["previous_accepted_sequence"] == 1
        assert failure["failure_streak"] == 1
    finally:
        if watchdog.poll() is None:
            watchdog.terminate()
            watchdog.wait(timeout=3)
        if target.poll() is None:
            os.killpg(target.pid, signal.SIGTERM)
            target.wait(timeout=3)
