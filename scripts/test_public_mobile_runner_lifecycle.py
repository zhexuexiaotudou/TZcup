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


def _run_cleanup_fixture(*, stop_rc: int, watchdog_breach: bool = False, git_drift: bool = False) -> tuple[int, list[str]]:
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
            "formal_runtime_stop_memory_watchdog(){ return 0; }; "
            f"formal_runtime_memory_watchdog_tripped(){{ return {0 if watchdog_breach else 1}; }}; "
            "FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT=0; FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE=86\n"
        "write_receipt(){ events+=(receipt:$1:$2:$3); return 0; }\n"
        "stop_private_group(){ return 0; }; stop_deadline(){ return 0; }; revalidate_final_inputs(){ return 0; }; binding_digest(){ printf fixture; }\n"
        "kill_checks=0; kill(){ if [[ \"$1\" == -0 && \"$2\" == 777 ]]; then ((kill_checks+=1)); (( kill_checks == 1 )); else command kill \"$@\"; fi; }\n"
            "RUN_ROOT=\"$(cd -- \"$(dirname -- \"${BASH_SOURCE[0]}\")\" && pwd -P)\"; ROOT=\"$RUN_ROOT\"; "
            + ("git(){ [[ \"$*\" == *status* ]] || printf drifted; }; " if git_drift else "git(){ [[ \"$*\" == *status* ]] || printf fixture; }; ")
            + "SELECTOR=\"$RUN_ROOT/no-selector\"; DESIRED_STATE=BLOCKED; RUNNER_EXIT_CODE=4\n"
            "launch_pid=777; operator_pid=999999; collector_pid=''; stop_estop_pid=''; QUOTA_PID=''; DEADLINE_PID=''; SETUP_PID=''; STOP_PID=''; PREFLIGHT_PID=''; PREFLIGHT_PGID=''; READINESS_PID=''; FINAL_ORACLE_PID=''; ADMISSION_BINDING_SHA256=fixture; ADMISSION_GIT_HEAD=fixture; ADMISSION_GIT_TREE=fixture; GIT_STATUS_AT_ADMISSION=''\n"
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
    assert len(rows) == 2, result.stderr
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


def test_memory_breach_precedes_simultaneous_cleanup_failure_in_the_receipt() -> None:
    rc, events = _run_cleanup_fixture(stop_rc=1, watchdog_breach=True)
    assert rc == 1
    assert events[-1] == "receipt:BLOCKED:86:true"


def test_head_or_clean_tree_drift_blocks_the_receipt() -> None:
    rc, events = _run_cleanup_fixture(stop_rc=0, git_drift=True)
    assert rc == 1
    assert events[-1] == "receipt:BLOCKED:125:true"


def test_normal_watcher_completion_is_preserved_in_scene_runtime_index() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    start = source.index("record_scene_runtime() {")
    record = source[start:source.index("\nwrite_receipt()", start)]
    work = ROOT / ".work"
    work.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=work) as raw:
        raw_path = Path(raw)
        scene_root = raw_path / "scenes" / "map-0-mission-0"
        scene_root.mkdir(parents=True)
        (scene_root / "memory_watchdog.json").write_text('{"target_pgid": 4242, "surviving_group_processes": 0, "status": "FORMAL_MEMORY_WATCHDOG_COMPLETED"}', encoding="utf-8")
        (scene_root / "memory_watchdog.log").write_text("completed\n", encoding="utf-8")
        fixture = raw_path / "fixture.sh"
        fixture.write_bytes((
            "#!/usr/bin/env bash\nset -Eeuo pipefail\n"
            "RUN_ROOT=\"$(cd -- \"$(dirname -- \"${BASH_SOURCE[0]}\")\" && pwd -P)\"\n"
            "SCENE_RUNTIME_INDEX=\"$RUN_ROOT/scene_runtime_index.json\"\nscene=map-0-mission-0\nGZ_PGID=4242\nscene_root=\"$RUN_ROOT/scenes/map-0-mission-0\"\n"
            + record + "\nrecord_scene_runtime\n"
        ).encode("utf-8"))
        result = subprocess.run(["bash", fixture.relative_to(ROOT).as_posix()], cwd=ROOT, text=True, capture_output=True, timeout=10)
        assert result.returncode == 0, result.stderr
        import json
        row = json.loads((raw_path / "scene_runtime_index.json").read_text(encoding="utf-8"))[0]
    assert row["gazebo_pgid"] == 4242
    assert row["watchdog_status"] == "FORMAL_MEMORY_WATCHDOG_COMPLETED"


def test_hung_foreground_child_is_taken_over_by_the_one_absolute_deadline() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    remaining = source[source.index("remaining_seconds() {"):source.index("\ndeadline_run() {")]
    deadline = source[source.index("deadline_run() {"):source.index("\nstop_private_group() {")]
    stopper = source[source.index("stop_private_group() {"):source.index("\nstop_deadline()", source.index("stop_private_group() {"))]
    live = source[source.index("group_has_live_processes() {"):source.index("\n# Even an early setup")]
    work = ROOT / ".work"
    work.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=work) as raw:
        fixture = Path(raw) / "fixture.sh"
        fixture.write_bytes((
            "#!/usr/bin/env bash\nset -Eeuo pipefail\n"
            + live + "\n" + remaining + "\n" + deadline + "\n" + stopper
            + "\nDEADLINE_EPOCH=$((SECONDS + 1)); setsid sleep 1 & DEADLINE_PID=$!\n"
            + "if deadline_run 20 /dev/null bash -c 'sleep 20'; then rc=0; else rc=$?; fi\n"
            + "kill -0 -- \"-$DEADLINE_PID\" 2>/dev/null && exit 99\nprintf '%s\\n' \"$rc\"\n"
        ).encode("utf-8"))
        result = subprocess.run(["bash", fixture.relative_to(ROOT).as_posix()], cwd=ROOT, text=True, capture_output=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "124"


def test_setup_leader_exit_with_background_child_fails_closed_and_reaps_group() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    remaining = source[source.index("remaining_seconds() {"):source.index("\nrevalidate_final_inputs() {")]
    deadline = source[source.index("deadline_run() {"):source.index("\nstop_private_group() {")]
    stopper = source[source.index("stop_private_group() {"):source.index("\nstop_deadline()", source.index("stop_private_group() {"))]
    live = source[source.index("group_has_live_processes() {"):source.index("\n# Even an early setup")]
    work = ROOT / ".work"
    with tempfile.TemporaryDirectory(dir=work) as raw:
        fixture = Path(raw) / "fixture.sh"
        fixture.write_bytes((
            "#!/usr/bin/env bash\nset -Eeuo pipefail\n"
            + live + "\n" + remaining + "\n" + deadline + "\n" + stopper
            + "\nDEADLINE_EPOCH=$((SECONDS + 20)); setsid sleep 20 & DEADLINE_PID=$!; CURRENT_PHASE=setup\n"
            + "if deadline_run 5 /dev/null bash -c 'sleep 20 & exit 0'; then rc=0; else rc=$?; fi\n"
            + "[[ -z \"${SETUP_PID:-}\" ]] || exit 98\nkill -0 -- \"-$DEADLINE_PID\" 2>/dev/null && kill -TERM -- \"-$DEADLINE_PID\" || true\nprintf '%s\\n' \"$rc\"\n"
        ).encode("utf-8"))
        result = subprocess.run(["bash", fixture.relative_to(ROOT).as_posix()], cwd=ROOT, text=True, capture_output=True, timeout=20)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "125"


def test_signal_during_setup_reaps_setup_and_deadline_groups() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    early = source[source.index("early_deadline_exit() {"):source.index("\ntrap early_deadline_exit EXIT")]
    live = source[source.index("group_has_live_processes() {"):source.index("\n# Even an early setup")]
    work = ROOT / ".work"
    with tempfile.TemporaryDirectory(dir=work) as raw:
        raw_path = Path(raw)
        pid_file = raw_path / "pids"
        fixture = raw_path / "fixture.sh"
        fixture.write_bytes((
            "#!/usr/bin/env bash\nset -Eeuo pipefail\ncd -- \"$(dirname -- \"${BASH_SOURCE[0]}\")\"\n"
            + live + "\n" + early + "\n"
            + "setsid bash -c 'sleep 20' & SETUP_PID=$!; setsid sleep 20 & DEADLINE_PID=$!\n"
            + "printf '%s %s\\n' \"$SETUP_PID\" \"$DEADLINE_PID\" >pids\n"
            + "trap early_deadline_exit EXIT\ntrap 'early_deadline_exit 143' TERM\nkill -TERM \"$$\"\n"
        ).encode("utf-8"))
        result = subprocess.run(["bash", fixture.relative_to(ROOT).as_posix()], cwd=ROOT, text=True, capture_output=True, timeout=15)
        setup, deadline = pid_file.read_text(encoding="utf-8").split()
        setup_alive = subprocess.run(["bash", "-c", "kill -0 -- \"-$1\" 2>/dev/null", "bash", setup], cwd=ROOT, timeout=5).returncode == 0
        deadline_alive = subprocess.run(["bash", "-c", "kill -0 -- \"-$1\" 2>/dev/null", "bash", deadline], cwd=ROOT, timeout=5).returncode == 0
    assert result.returncode != 0
    assert not setup_alive and not deadline_alive


def test_final_oracle_reserve_blocks_start_and_propagates_124() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    remaining = source[source.index("remaining_seconds() {"):source.index("\nrevalidate_final_inputs() {")]
    reserve = source[source.index("require_final_oracle_reserve() {"):source.index("\ndeadline_run() {")]
    fixture = "#!/usr/bin/env bash\nset -Eeuo pipefail\n" + remaining + "\n" + reserve + "\nFINAL_VALIDATION_DEADLINE_SEC=120; DEADLINE_EPOCH=$((SECONDS + 119)); FINAL_ORACLE_PID=''; oracle_started=false\nif final_oracle_budget=$(require_final_oracle_reserve); then oracle_started=true; wrapper=0; else wrapper=$?; fi\n[[ \"$oracle_started\" == false ]] || exit 99\nprintf '%s:%s\\n' \"$wrapper\" \"${FINAL_ORACLE_PID:-empty}\"\n"
    work = ROOT / ".work"
    with tempfile.TemporaryDirectory(dir=work) as raw:
        script = Path(raw) / "fixture.sh"
        script.write_bytes(fixture.encode("utf-8"))
        result = subprocess.run(["bash", script.relative_to(ROOT).as_posix()], cwd=ROOT, text=True, capture_output=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "124:empty"


def test_final_revalidation_rejects_same_content_setup_symlink_swap() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    has = source[source.index("has_symlink_ancestor() {"):source.index("\nreal_regular() {")]
    regular = source[source.index("real_regular() {"):source.index("\nwithin_root() {")]
    within = source[source.index("within_root() {"):source.index("\nhas_symlink_ancestor \"$PUBLIC_GAZEBO_CALIBRATION_OUTPUT\"")]
    revalidate = source[source.index("revalidate_final_inputs() {"):source.index("\nrequire_final_oracle_reserve() {")]
    work = ROOT / ".work"
    with tempfile.TemporaryDirectory(dir=work) as raw:
        root = Path(raw)
        for name in ("plan.json", "stage.sh", "runtime.sh", "campus.sh"):
            (root / name).write_text("same\n", encoding="utf-8")
        fixture = root / "fixture.sh"
        fixture.write_bytes((
            "#!/usr/bin/env bash\nset -Eeuo pipefail\n"
            "ROOT=\"$(cd -- \"$(dirname -- \"${BASH_SOURCE[0]}\")\" && pwd -P)\"; MODE=pilot\n"
            "PUBLIC_GAZEBO_CALIBRATION_PLAN=\"$ROOT/plan.json\"; CANONICAL_PLAN=\"$ROOT/plan.json\"; PUBLIC_GAZEBO_CALIBRATION_STAGE1_SETUP=\"$ROOT/stage.sh\"; PUBLIC_GAZEBO_CALIBRATION_RUNTIME_SETUP=\"$ROOT/runtime.sh\"; PUBLIC_GAZEBO_CALIBRATION_CAMPUS_SETUP=\"$ROOT/campus.sh\"\n"
            + has + "\n" + regular + "\n" + within + "\n" + revalidate
            + "\nrevalidate_final_inputs\nrm \"$ROOT/stage.sh\"; ln -s \"$ROOT/runtime.sh\" \"$ROOT/stage.sh\"\nif revalidate_final_inputs; then exit 99; fi\nprintf '%s\\n' rejected\n"
        ).encode("utf-8"))
        result = subprocess.run(["bash", fixture.relative_to(ROOT).as_posix()], cwd=ROOT, text=True, capture_output=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "rejected"


def test_runner_exit_and_signal_traps_capture_the_real_status() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    assert 'trap \'RUNNER_EXIT_CODE=$?; formal_runtime_exit_trap "$RUNNER_EXIT_CODE"\' EXIT' in source
    assert "RUNNER_EXIT_CODE=130; PRIMARY_ERROR=signal:INT" in source
    assert "RUNNER_EXIT_CODE=143; PRIMARY_ERROR=signal:TERM" in source
