"""Shell-level lifecycle regression tests without sourcing ROS or Gazebo setup."""
from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "scripts" / "run_public_mobile_gazebo_dosod_calibration.sh"


def _function(name: str, source: str) -> str:
    start = source.index(f"{name}() {{")
    # Functions in this runner are separated by a top-level newline and a name.
    tail = source[start:]
    for marker in ("\ntrap ", "\nformal_runtime_register", "\nscene_rows()"):
        if marker in tail:
            tail = tail[:tail.index(marker)]
    return tail


def _run_cleanup_fixture(*, stop_rc: int) -> tuple[int, list[str]]:
    source = RUNNER.read_text(encoding="utf-8")
    cleanup = _function("cleanup", source)
    work = ROOT / ".work"
    work.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="mobile-runner-test-", dir=work) as raw:
        tmp_path = Path(raw)
        fixture = tmp_path / "fixture.sh"
        fixture.write_bytes((
        "#!/usr/bin/env bash\nset -Eeuo pipefail\n"
        "events=()\n"
        f"stop_verified(){{ events+=(stop_verified); scene_stop_verified=true; return {stop_rc}; }}\n"
        "stop_estop_publisher(){ events+=(stop_estop_publisher); return 0; }\n"
        "formal_runtime_cleanup_groups(){ events+=(cleanup_groups); return 0; }\n"
        "write_receipt(){ events+=(receipt:$1:$2:$3); return 0; }\n"
        "stop_private_group(){ return 0; }; stop_deadline(){ return 0; }; binding_digest(){ printf fixture; }\n"
        "kill_checks=0; kill(){ if [[ \"$1\" == -0 && \"$2\" == 777 ]]; then ((kill_checks+=1)); (( kill_checks == 1 )); else command kill \"$@\"; fi; }\n"
        "RUN_ROOT=\"$(cd -- \"$(dirname -- \"${BASH_SOURCE[0]}\")\" && pwd -P)\"; SELECTOR=\"$RUN_ROOT/no-selector\"; DESIRED_STATE=BLOCKED; RUNNER_EXIT_CODE=4\n"
        "launch_pid=777; operator_pid=999999; collector_pid=''; stop_estop_pid=''; QUOTA_PID=''; DEADLINE_PID=''; PREFLIGHT_PID=''; READINESS_PID=''; FINAL_ORACLE_PID=''; ADMISSION_BINDING_SHA256=fixture\n"
        "scene_operator_started=true; scene_stop_attempted=false; scene_stop_verified=false\n"
        + cleanup
        + "\nset +e\ncleanup\nrc=$?\nset -e\nprintf '%s\\n' \"$rc\" \"${events[*]}\"\n"
        ).encode("utf-8"))
        fixture.chmod(0o700)
        result = subprocess.run(
            ["bash", fixture.relative_to(ROOT).as_posix()],
            cwd=ROOT,
            text=True,
            capture_output=True,
            check=True,
            timeout=20,
        )
    rows = result.stdout.strip().splitlines()
    return int(rows[0]), rows[1].split()


def test_dead_operator_does_not_bypass_verified_stop_while_launch_is_alive() -> None:
    rc, events = _run_cleanup_fixture(stop_rc=0)
    assert rc == 0
    assert events[:2] == ["stop_verified", "cleanup_groups"]
    assert events[-1] == "receipt:BLOCKED:4:true"


def test_failed_verified_stop_forces_cleanup_rc_and_receipt_125() -> None:
    rc, events = _run_cleanup_fixture(stop_rc=1)
    assert rc == 1
    assert events[:2] == ["stop_verified", "cleanup_groups"]
    assert events[-1] == "receipt:BLOCKED:125:true"


def test_runner_exit_and_signal_traps_capture_the_real_status() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    assert 'trap \'RUNNER_EXIT_CODE=$?; formal_runtime_exit_trap "$RUNNER_EXIT_CODE"\' EXIT' in source
    assert "RUNNER_EXIT_CODE=130; PRIMARY_ERROR=signal:INT" in source
    assert "RUNNER_EXIT_CODE=143; PRIMARY_ERROR=signal:TERM" in source
