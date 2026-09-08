from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT / ".work"


def _bash(path: Path) -> str:
    """Map a Windows worktree into WSL, while leaving native POSIX paths intact."""
    if os.name != "nt":
        return path.as_posix()
    drive = path.drive.rstrip(":").lower()
    if len(drive) != 1 or not drive.isalpha():
        raise ValueError(f"expected a drive-qualified Windows path: {path}")
    return f"/mnt/{drive}" + str(path)[2:].replace("\\", "/")


BASH_ROOT = _bash(ROOT)
LIB = f"{BASH_ROOT}/scripts/public_gazebo_mobile_readiness.sh"
PARSER = f"{BASH_ROOT}/scripts/parse_public_gazebo_topic_info.py"


def test_bash_path_keeps_native_posix_paths(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(os, "name", "posix")
    assert _bash(Path("/home/runner/work/TZcup/TZcup")) == "/home/runner/work/TZcup/TZcup"


def _run(script_path: Path) -> subprocess.CompletedProcess[str]:
    command = ["bash", "-lc", f"timeout --kill-after=2 15 bash {_bash(script_path)}"]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        out, err = process.communicate(timeout=20)
    except subprocess.TimeoutExpired:
        process.kill()
        out, err = process.communicate()
        pytest.fail(f"outer readiness fixture timeout; stdout={out!r} stderr={err!r}")
    return subprocess.CompletedProcess(command, process.returncode, out, err)


def test_matrix_fake_ros2_cases() -> None:
    readiness = (ROOT / "scripts/public_gazebo_mobile_readiness.sh").read_text(encoding="utf-8")
    assert "required_nodes=(/formal_map_lifecycle_manager /formal_legacy_topic_adapter /formal_vehicle_training_gt_bridge)" in readiness
    assert '"$ros2bin" node list --no-daemon' in readiness
    WORK.mkdir(exist_ok=True)
    raw = Path(tempfile.mkdtemp(prefix="test-readiness-", dir=WORK))
    try:
        fake = raw / "ros2"
        fake.write_text(
            r'''#!/usr/bin/env bash
mode=${FAKE_MODE:-ok}
printf '%s\n' "$*" >> "${FAKE_ARGS_LOG:?}"
if [[ "$1 $2" == "node list" ]]; then
  echo /formal_map_lifecycle_manager
  echo /formal_legacy_topic_adapter
  echo /formal_vehicle_training_gt_bridge
  [[ $mode == duplicate ]] && echo /formal_legacy_topic_adapter
  exit 0
fi
if [[ "$1 $2" == "node info" ]]; then
  [[ $mode == missing && $3 == /formal_legacy_topic_adapter ]] && exit 2
  exit 0
fi
if [[ "$1 $2" == "topic info" ]]; then
  [[ $mode == hung ]] && { trap '' TERM; sleep 70 & wait; }
  t=$3; owner=formal_vehicle_training_gt_bridge
  [[ $t == /camera/* ]] && owner=formal_legacy_topic_adapter
  typ=sensor_msgs/msg/Image
  [[ $t == */camera_info ]] && typ=sensor_msgs/msg/CameraInfo
  [[ $mode == wrong_owner ]] && owner=wrong
  [[ $mode == wrong_type ]] && typ=wrong/msg/Type
  [[ $mode == multi ]] && count=2 || count=1
  [[ $mode == bad_gid ]] && gid=00000000000000000000000000000000 || gid=aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
  printf 'Type: %s\nPublisher count: %s\nPublisher GID:\n  %s\nNode name: %s\nNode namespace: /\n' "$typ" "$count" "$gid" "$owner"
  [[ $mode == cumulative_bad ]] && printf 'Publisher GID:\n  bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n'
  exit 0
fi
if [[ "$1 $2" == "topic echo" ]]; then
  topic=$4
  [[ $mode == no_message && $topic != /clock ]] && exit 3
  if [[ $topic == /clock ]]; then
    if [[ $mode == clock_static ]]; then printf 'sec: 7\nnanosec: 0\n'; else printf 'sec: %s\nnanosec: 0\n' "$(date +%s%N)"; fi
  else
    printf 'msg\n'
  fi
  exit 0
fi
exit 2
''',
            newline="\n",
        )
        fake.chmod(0o755)
        arguments_log = raw / "ros2-arguments.log"
        cases = [
            ("ok", 0, "independent"),
            ("missing", 2, "independent"),
            ("duplicate", 2, "independent"),
            ("wrong_owner", 2, "independent"),
            ("wrong_type", 2, "independent"),
            ("bad_gid", 2, "independent"),
            ("multi", 2, "independent"),
            ("cumulative_bad", 2, "independent"),
            ("no_message", 3, "independent"),
            ("clock_static", 2, "independent"),
            ("hung", 124, "independent"),
            ("ok", 0, "orchestrated"),
            ("ok", 125, "leader_exit_child"),
        ]
        for index, (mode, expected_rc, topology) in enumerate(cases):
            log = raw / f"{index}-{mode}.log"
            leader = "setsid sleep 80 & p=$!"
            session = ""
            expected = ""
            cleanup = 'kill -KILL -- -"$p" 2>/dev/null || true; wait "$p" 2>/dev/null || true'
            if topology == "orchestrated":
                # The leader shares the caller's outer PGID; it is not a private session leader.
                leader = "sleep 80 & p=$!; outer=$(ps -o pgid= -p $$ | tr -d '[:space:]')"
                session = "export FORMAL_ORCHESTRATED_STEP_SESSION=1"
                expected = ' "$outer"'
                cleanup = 'kill -KILL "$p" 2>/dev/null || true; wait "$p" 2>/dev/null || true'
            elif topology == "leader_exit_child":
                leader = "setsid bash -c 'sleep 80 & exit 0' & p=$!; sleep 0.2"
            script = f'''set +e
source "{LIB}"
export PUBLIC_GAZEBO_CALIBRATION_PARSER="{PARSER}"
export PUBLIC_GAZEBO_CALIBRATION_ROS2_BIN="{_bash(fake)}"
export FAKE_ARGS_LOG="{_bash(arguments_log)}"
export FAKE_MODE="{mode}"
{session}
{leader}
cleanup() {{ {cleanup}; }}
trap cleanup EXIT INT TERM
public_mobile_mapping_readiness "$p" "{_bash(log)}" {2 if mode == "hung" else 8}{expected}
r=$?
exit "$r"
'''
            case_script = raw / f"{index}-{mode}.sh"
            case_script.write_text(script, encoding="utf-8", newline="\n")
            case_script.chmod(0o755)
            result = _run(case_script)
            assert result.returncode == expected_rc, (mode, topology, script, result.stdout, result.stderr)
            receipt = json.loads((Path(f"{log}.receipt.json")).read_text(encoding="utf-8"))
            assert receipt["returncode"] == expected_rc
            assert receipt["status"] == ("READY" if expected_rc == 0 else "BLOCKED")
            assert receipt["operations"] or topology == "leader_exit_child"
            assert all({"role", "path", "sha256", "elapsed_ms", "returncode", "zero_survivor"} <= row.keys() for row in receipt["operations"])
            assert all(row["zero_survivor"] for row in receipt["operations"])
        commands = arguments_log.read_text(encoding="utf-8").splitlines()
        node_lists = [command for command in commands if command.startswith("node list")]
        assert node_lists
        assert all(command == "node list --no-daemon" for command in node_lists)
    finally:
        shutil.rmtree(raw)
