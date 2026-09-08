"""Route and guard regressions; fixtures never invoke ROS or Gazebo."""
from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SHIM = ROOT / "scripts" / "run_public_gazebo_dosod_calibration.sh"
MOBILE = ROOT / "scripts" / "run_public_mobile_gazebo_dosod_calibration.sh"


def test_legacy_entrypoint_exec_delegates_to_the_only_mobile_owner() -> None:
    work = ROOT / ".work"
    work.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=work) as raw:
        repo = Path(raw) / "repo"
        scripts = repo / "scripts"
        scripts.mkdir(parents=True)
        target = scripts / "run_public_mobile_gazebo_dosod_calibration.sh"
        target.write_bytes(b"#!/usr/bin/env bash\nprintf '%s\\n' \"$*\" >\"$(dirname \"$0\")/argv\"\nexit 37\n")
        target.chmod(0o700)
        shim = scripts / SHIM.name
        shim.write_bytes(SHIM.read_bytes())
        shim.chmod(0o700)
        run = subprocess.run(["bash", shim.relative_to(ROOT).as_posix(), "pilot", "--fixture"], cwd=ROOT, text=True, capture_output=True, timeout=10)
        assert run.returncode == 37
        assert (scripts / "argv").read_text(encoding="utf-8").strip() == "pilot --fixture"


def test_mobile_has_one_route_with_fixed_modes_quotas_and_resource_guards() -> None:
    source = MOBILE.read_text(encoding="utf-8")
    shim = SHIM.read_text(encoding="utf-8")
    assert 'exec "$ROOT/scripts/run_public_mobile_gazebo_dosod_calibration.sh" "$@"' in shim
    assert "ros2 " not in shim and shim.count("exec ") == 1
    for required in (
        'MODE="${PUBLIC_GAZEBO_CALIBRATION_MODE:-full}"',
        'pilot quota must be 25',
        'full quota must be 25 (20+4 scenes = 500+100)',
        'PUBLIC_GAZEBO_CALIBRATION_TOTAL_TIMEOUT_SEC',
        'FORMAL_MEMORY_MAX_GROUP_RSS_KIB=9437184',
        'formal_runtime_start_memory_watchdog',
        'wait -n -p finished',
        'stop_private_group',
        'formal_runtime_cleanup_groups',
        'binding_digest',
        'binding_or_git_drift',
        'zero_survivor_check',
        'CANONICAL_PLAN="$(real_regular "$ROOT/config/public_gazebo_dosod_train_scene_plan.json")"',
        'pilot collector and total deadlines must be at least 900 seconds',
        'full collector and total deadlines must be at least 14400 seconds',
        'PREFLIGHT_PGID',
        'record_scene_runtime',
        'preprocessing_oracle_final_validation_sha256',
    ):
        assert required in source
    assert source.index('validate_dosod_single_frame_preprocessing_oracle.py') < source.index('ros2 launch sanitation_formal_campus_integration')
    assert source.count('validate_dosod_single_frame_preprocessing_oracle.py') == 2


def test_deadline_helper_uses_term_then_kill_for_an_exact_private_group() -> None:
    source = MOBILE.read_text(encoding="utf-8")
    start = source.index("stop_private_group() {")
    end = source.index("\nstop_deadline()", start)
    helper = source[start:end]
    live = source[source.index("group_has_live_processes() {"):source.index("\n# Even an early setup")]
    fixture = "#!/usr/bin/env bash\nset -Eeuo pipefail\n" + live + "\n" + helper + "\nsetsid bash -c 'trap \"\" TERM; while :; do sleep 1; done' & pid=$!\nsleep .05\nset +e\nstop_private_group \"$pid\"\nrc=$?\nset -e\nkill -0 -- \"-$pid\" 2>/dev/null && exit 99\nprintf '%s\\n' \"$rc\"\n"
    work = ROOT / ".work"
    work.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=work) as raw:
        script = Path(raw) / "fixture.sh"
        script.write_bytes(fixture.encode("utf-8"))
        result = subprocess.run(["bash", script.relative_to(ROOT).as_posix()], cwd=ROOT, text=True, capture_output=True, timeout=20)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "0"
