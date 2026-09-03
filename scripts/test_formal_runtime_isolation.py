from __future__ import annotations

import json
import os
import re
import subprocess
import textwrap
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
HELPER = SCRIPTS / "run_formal_runtime_isolation.sh"

ISOLATED_RUNNERS = (
    "run_formal_20_cube_grasp_acceptance.sh",
    "run_formal_auxiliary_runtime.sh",
    "run_formal_campus_runtime.sh",
    "run_formal_cleaning_actuator_motor_runtime.sh",
    "run_formal_cube_pick_place_runtime.sh",
    "run_formal_dynamic_obstacle_avoidance.sh",
    "run_formal_first_map_dynamic_prerequisite.sh",
    "run_formal_function_positions_runtime.sh",
    "run_formal_grasp_executor_runtime.sh",
    "run_formal_ground_dirt_cleaning_runtime.sh",
    "run_formal_manipulator_trajectory_runtime.sh",
    "run_formal_random_scene_perception.sh",
    "run_formal_same_map_full_coverage_baseline.sh",
    "run_formal_saved_map_cleaning_lifecycle.sh",
    "run_formal_service_door_runtime.sh",
    "run_formal_service_interface_acceptance.sh",
    "run_formal_single_episode_cleaning_mission.sh",
    "run_formal_typed_cleaning_motor_diagnostic.sh",
    "run_formal_vehicle_mobility_runtime.sh",
    "run_formal_vehicle_sensor_runtime.sh",
    "run_formal_vehicle_visual_acceptance.sh",
    "run_formal_water_recovery_runtime.sh",
    "run_whole_vehicle_actuator_interlock_runtime.sh",
)

DEFAULT_DOMAINS = {
    "run_formal_20_cube_grasp_acceptance.sh": (96, 1),
    "run_formal_auxiliary_runtime.sh": (221, 1),
    "run_formal_campus_runtime.sh": (89, 1),
    "run_formal_cleaning_actuator_motor_runtime.sh": (85, 1),
    "run_formal_cube_pick_place_runtime.sh": (84, 1),
    "run_formal_dynamic_obstacle_avoidance.sh": (73, 1),
    "run_formal_first_map_dynamic_prerequisite.sh": (99, 1),
    "run_formal_function_positions_runtime.sh": (87, 1),
    "run_formal_grasp_executor_runtime.sh": (218, 1),
    "run_formal_ground_dirt_cleaning_runtime.sh": (95, 1),
    "run_formal_manipulator_trajectory_runtime.sh": (86, 1),
    "run_formal_random_scene_perception.sh": (70, 3),
    "run_formal_same_map_full_coverage_baseline.sh": (61, 1),
    "run_formal_saved_map_cleaning_lifecycle.sh": (60, 1),
    "run_formal_service_door_runtime.sh": (79, 1),
    "run_formal_service_interface_acceptance.sh": (87, 8),
    "run_formal_single_episode_cleaning_mission.sh": (62, 1),
    "run_formal_typed_cleaning_motor_diagnostic.sh": (97, 1),
    "run_formal_vehicle_mobility_runtime.sh": (82, 1),
    "run_formal_vehicle_sensor_runtime.sh": (81, 1),
    "run_formal_vehicle_visual_acceptance.sh": (100, 2),
    "run_formal_water_recovery_runtime.sh": (96, 1),
    "run_whole_vehicle_actuator_interlock_runtime.sh": (220, 1),
}


def linux_safe(domain: int) -> bool:
    return 0 <= domain <= 101 or 215 <= domain <= 231


def shell_logical_lines(source: str) -> list[str]:
    logical_lines: list[str] = []
    pending: list[str] = []
    for raw in source.splitlines():
        stripped = raw.strip()
        pending.append(stripped[:-1].strip() if stripped.endswith("\\") else stripped)
        if stripped.endswith("\\"):
            continue
        logical_lines.append(" ".join(pending))
        pending = []
    return logical_lines


def test_shared_helper_owns_the_complete_isolation_policy() -> None:
    source = HELPER.read_text(encoding="utf-8")
    for token in (
        "formal ROS domain must be a decimal integer",
        "FORMAL_RUNTIME_MAX_AUTO_PARTICIPANT_INDEX=120",
        "formal_runtime_max_dds_unicast_port",
        "max_dds_port < 65535",
        "0 && domain <= 101",
        "domain >= 215 && domain <= 231",
        "ROS2CLI_DISABLE_DAEMON=1",
        "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
        "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST",
        "unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS",
        "GZ_IP=127.0.0.1",
        "IGN_IP=127.0.0.1",
        "unset GZ_RELAY IGN_RELAY",
        "cyclonedds_localhost.xml",
        'repo_root="$(cd -- "${helper_dir}/.." && pwd)"',
        'CYCLONEDDS_URI="file://${repo_root}/config/cyclonedds_localhost.xml"',
        "FORMAL_GAZEBO_LOCK_FILE:-/tmp/tzcup_formal_gazebo.lock",
        "flock -n 9",
        'FORMAL_RUNTIME_SESSION_PREFIX=(setsid)',
        'FORMAL_ORCHESTRATED_STEP_SESSION:-0',
        'kill -INT "${pid}"',
        "for signal in TERM KILL",
        'kill -0 -- "-${pid}"',
        'wait "${pid}"',
        'pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null || true)"',
        'if [[ -n "${pgid}" && "${pgid}" != "${pid}" ]]; then',
        'needle = ("GZ_PARTITION=" + sys.argv[1]).encode()',
        "formal_runtime_contain_rejected_leader",
        "formal partition cleanup survivors",
        "formal_runtime_quarantine_evidence",
        ".cleanup_failed.$$",
        "status=125",
    ):
        assert token in source
    assert "FORMAL_MEMORY_WATCHDOG_ENABLED cannot be disabled for formal runtime" in source
    assert "FORMAL_WINDOWS_MEMORY_GUARD_ENABLED cannot be disabled for formal runtime" in source
    assert 'RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-' not in source
    assert 'CYCLONEDDS_URI:-' not in source
    assert 'CYCLONEDDS_URI="file://${helper_dir}/../config/' not in source


def test_shared_helper_replaces_legacy_ros_discovery_environment() -> None:
    harness = """
set -euo pipefail
export ROS_LOCALHOST_ONLY=1
export ROS_STATIC_PEERS=192.0.2.10
formal_runtime_configure 81
[[ "${ROS_AUTOMATIC_DISCOVERY_RANGE}" == "LOCALHOST" ]]
[[ -z "${ROS_LOCALHOST_ONLY+x}" ]]
[[ -z "${ROS_STATIC_PEERS+x}" ]]
"""
    result = subprocess.run(
        ["bash"],
        check=False,
        capture_output=True,
        input=(HELPER.read_text(encoding="utf-8").replace("\r", "") + harness).encode(
            "utf-8"
        ),
        timeout=10,
    )
    assert result.returncode == 0, result.stderr.decode("utf-8", errors="replace")


def test_formal_vehicle_launch_pins_gazebo_transport_before_simulator_actions() -> None:
    launch_path = (
        ROOT
        / "starter_ws"
        / "src"
        / "sanitation_vehicle_description"
        / "launch"
        / "formal_vehicle_sim.launch.py"
    )
    source = launch_path.read_text(encoding="utf-8")
    gz_ip = source.index('SetEnvironmentVariable("GZ_IP", "127.0.0.1")')
    ign_ip = source.index('SetEnvironmentVariable("IGN_IP", "127.0.0.1")')
    simulator = source.index("IncludeLaunchDescription(", gz_ip)
    assert gz_ip < ign_ip < simulator


def test_orchestrated_shared_group_cleanup_is_bounded_and_partition_scoped() -> None:
    source = HELPER.read_text(encoding="utf-8")
    shared_group_guard = source.index(
        'if [[ -n "${pgid}" && "${pgid}" != "${pid}" ]]; then'
    )
    partition_cleanup = source.index('formal_runtime_cleanup_partition "${partition}"')
    reap_after_partition = source.index('wait "${pid}" 2>/dev/null || true', partition_cleanup)
    assert 'for attempt in {1..40}; do' in source[shared_group_guard:partition_cleanup]
    assert "return 0" in source[shared_group_guard:partition_cleanup]
    assert partition_cleanup < reap_after_partition


def test_localhost_cyclonedds_capacity_supports_the_complete_campus_graph() -> None:
    config = ROOT / "config" / "cyclonedds_localhost.xml"
    root = ET.parse(config).getroot()
    namespace = {"c": "https://cdds.io/config"}
    interface = root.find(".//c:NetworkInterface", namespace)
    maximum = root.find(".//c:MaxAutoParticipantIndex", namespace)
    assert interface is not None
    assert interface.attrib == {"name": "lo", "multicast": "true"}
    assert maximum is not None
    assert maximum.text == "120"
    for domain in range(0, 102):
        assert 7400 + 250 * domain + 11 + 2 * int(maximum.text) < 65535
    for domain in range(215, 232):
        assert 7400 + 250 * domain + 11 + 2 * int(maximum.text) < 65535


def test_every_non_visual_gazebo_runner_uses_shared_policy_and_fail_closed_traps() -> None:
    for name in ISOLATED_RUNNERS:
        source = (SCRIPTS / name).read_text(encoding="utf-8")
        assert "run_formal_runtime_isolation.sh" in source, name
        assert "formal_runtime_configure" in source, name
        assert "formal_runtime_install_traps" in source, name
        assert "formal_runtime_register_evidence_paths" in source, name
        assert '"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ' in source, name
        assert "GZ_PARTITION" in source, name
        assert not re.search(r"trap\s+cleanup\s+EXIT", source), name


def test_orchestrated_runners_do_not_create_nested_process_groups() -> None:
    for name in ISOLATED_RUNNERS:
        source = (SCRIPTS / name).read_text(encoding="utf-8")
        assert not re.search(r"(?m)^\s*setsid\s+", source), name

    integrated = (SCRIPTS / "run_integrated_functional_acceptance.sh").read_text(
        encoding="utf-8"
    )
    assert 'FORMAL_ORCHESTRATED_STEP_SESSION:-0' in integrated
    assert "integrated_session_prefix=(setsid)" in integrated
    assert '"${integrated_session_prefix[@]}"' in integrated
    assert integrated.count("FORMAL_ORCHESTRATED_STEP_SESSION=1") == 2
    assert not re.search(r"(?m)^\s*setsid\s+", integrated)

    service = (SCRIPTS / "run_formal_service_interface_acceptance.sh").read_text(
        encoding="utf-8"
    )
    assert "timeout --foreground" in service


@pytest.mark.skipif(os.name != "posix", reason="requires native POSIX process groups")
def test_orchestrated_inner_watchdog_binds_to_the_outer_session_and_stops(
    tmp_path: Path,
) -> None:
    prefix = tmp_path / "subrunner_outer_session_watchdog"
    harness = """
set -euo pipefail
source "$1"
runner_pid="$$"
session_leader="$(ps -o sid= -p "${runner_pid}" | tr -d '[:space:]')"
[[ "${session_leader}" =~ ^[0-9]+$ && "${session_leader}" != "${runner_pid}" ]]
sleep 30 &
launch_pid=$!
trap 'formal_runtime_stop_memory_watchdog || true; kill "${launch_pid}" 2>/dev/null || true; wait "${launch_pid}" 2>/dev/null || true' EXIT
formal_runtime_start_memory_watchdog "${launch_pid}" "$2"
watchdog_pid="${FORMAL_RUNTIME_MEMORY_WATCHDOG_PID}"
[[ "${watchdog_pid}" =~ ^[0-9]+$ ]]
kill -0 "${watchdog_pid}"
for attempt in {1..100}; do
  grep -q 'watchdog started' "$2.log" 2>/dev/null && break
  sleep 0.05
done
grep -q 'watchdog started' "$2.log"
formal_runtime_stop_memory_watchdog
[[ -z "${FORMAL_RUNTIME_MEMORY_WATCHDOG_PID}" ]]
python3 - "$2.json" "${session_leader}" "${runner_pid}" <<'PY'
import json, sys
payload = json.load(open(sys.argv[1], encoding="utf-8"))
assert payload["status"] == "FORMAL_MEMORY_WATCHDOG_STOPPED"
assert payload["leader_pid"] == payload["target_pgid"] == int(sys.argv[2])
assert payload["leader_pid"] != int(sys.argv[3])
PY
printf 'watchdog=%s\\n' "${watchdog_pid}"
"""
    # The inner shell intentionally shares the outer setsid group's PGID/SID.
    # Integrated mobility/grasp child runners use this shape.
    wrapper = 'bash -c "$1" bash "$2" "$3" & child=$!; wait "${child}"'
    result = subprocess.run(
        [
            "setsid",
            "bash",
            "-c",
            wrapper,
            "bash",
            harness,
            str(HELPER),
            str(prefix),
        ],
        check=False,
        capture_output=True,
        text=True,
        env={
            **os.environ,
            "FORMAL_ORCHESTRATED_STEP_SESSION": "1",
            "FORMAL_ORCHESTRATED_STEP_SESSION_TOKEN": "a" * 64,
            "FORMAL_WINDOWS_MEMORY_GUARD_ENABLED": "0",
            "FORMAL_MEMORY_MIN_AVAILABLE_KIB": "0",
            "FORMAL_MEMORY_MAX_SWAP_USED_KIB": str(2**63 - 1),
            "FORMAL_MEMORY_MAX_GROUP_RSS_KIB": str(2**63 - 1),
            "FORMAL_MEMORY_POLL_S": "0.1",
        },
        timeout=10,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.startswith("watchdog=")
    payload = json.loads(prefix.with_suffix(".json").read_text(encoding="utf-8"))
    assert payload["status"] == "FORMAL_MEMORY_WATCHDOG_STOPPED"
    assert payload["leader_pid"] == payload["target_pgid"]
    assert prefix.with_suffix(".log").is_file()


def test_orchestrated_inner_watchdog_fails_closed_when_pgid_and_sid_do_not_match() -> None:
    harness = """
set -uo pipefail
ps() {
  case "${2:-}" in
    pgid=) printf ' 700\\n' ;;
    sid=) printf ' 701\\n' ;;
    *) return 2 ;;
  esac
}
setsid() { echo 'unexpected nested watchdog' >&2; return 99; }
formal_runtime_start_memory_watchdog 4242 /tmp/tzcup_unattested_watchdog
rc=$?
printf 'rc=%s\\n' "${rc}"
exit "${rc}"
"""
    result = subprocess.run(
        ["bash"],
        check=False,
        capture_output=True,
        input=(
            HELPER.read_text(encoding="utf-8")
            + "\n"
            + "export FORMAL_ORCHESTRATED_STEP_SESSION=1\n"
            + "export FORMAL_ORCHESTRATED_STEP_SESSION_TOKEN=" + "a" * 64 + "\n"
            + harness
        ).encode("utf-8"),
        timeout=10,
    )
    assert result.returncode == 125
    assert result.stdout.decode("utf-8", errors="replace") == "rc=125\n"
    assert "must share one dedicated PGID/SID" in result.stderr.decode(
        "utf-8", errors="replace"
    )


@pytest.mark.skipif(os.name != "posix", reason="requires native POSIX process groups")
@pytest.mark.parametrize(
    ("mode", "outer_token", "child_token", "expected"),
    (
        ("missing", None, None, "requires a valid capability token"),
        ("malformed", "a" * 64, "not-a-token", "requires a valid capability token"),
        ("mismatch", "a" * 64, "b" * 64, "lacks the matching capability token"),
    ),
)
def test_orchestrated_source_rejects_unattested_session(
    tmp_path: Path,
    mode: str,
    outer_token: str | None,
    child_token: str | None,
    expected: str,
) -> None:
    prefix = tmp_path / mode
    harness = """
set -uo pipefail
source "$1"
rc=$?
printf 'rc=%s\\n' "${rc}"
exit "${rc}"
"""
    child_environment = ""
    if child_token is not None:
        child_environment = f"FORMAL_ORCHESTRATED_STEP_SESSION_TOKEN={child_token} "
    wrapper = f'{child_environment}bash -c "$1" bash "$2" "$3" & child=$!; wait "${{child}}"'
    environment = {
        **os.environ,
        "FORMAL_ORCHESTRATED_STEP_SESSION": "1",
        "FORMAL_WINDOWS_MEMORY_GUARD_ENABLED": "0",
    }
    if outer_token is not None:
        environment["FORMAL_ORCHESTRATED_STEP_SESSION_TOKEN"] = outer_token
    result = subprocess.run(
        ["setsid", "bash", "-c", wrapper, "bash", harness, str(HELPER), str(prefix)],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
        timeout=10,
    )
    assert result.returncode == 125
    assert result.stdout == "rc=125\n"
    assert expected in result.stderr
    assert not prefix.with_suffix(".json").exists()
    assert not prefix.with_suffix(".log").exists()


@pytest.mark.skipif(os.name != "posix", reason="requires native POSIX process groups")
def test_orchestrated_watchdog_rejects_launch_outside_attested_session(tmp_path: Path) -> None:
    prefix = tmp_path / "launch_group_mismatch"
    harness = """
set -uo pipefail
source "$1"
setsid sleep 30 &
launch_pid=$!
trap 'kill -TERM -- "-${launch_pid}" 2>/dev/null || kill -TERM "${launch_pid}" 2>/dev/null || true; wait "${launch_pid}" 2>/dev/null || true' EXIT
formal_runtime_start_memory_watchdog "${launch_pid}" "$2"
rc=$?
printf 'rc=%s\\n' "${rc}"
exit "${rc}"
"""
    result = subprocess.run(
        ["setsid", "bash", "-c", 'bash -c "$1" bash "$2" "$3"', "bash", harness, str(HELPER), str(prefix)],
        check=False,
        capture_output=True,
        text=True,
        env={
            **os.environ,
            "FORMAL_ORCHESTRATED_STEP_SESSION": "1",
            "FORMAL_ORCHESTRATED_STEP_SESSION_TOKEN": "a" * 64,
            "FORMAL_WINDOWS_MEMORY_GUARD_ENABLED": "0",
        },
        timeout=10,
    )
    assert result.returncode == 125
    assert result.stdout == "rc=125\n"
    assert "launch must share the runner PGID/SID" in result.stderr
    assert not prefix.with_suffix(".json").exists()
    assert not prefix.with_suffix(".log").exists()


def test_all_default_domain_spans_avoid_linux_ephemeral_ports() -> None:
    for name, (base, count) in DEFAULT_DOMAINS.items():
        assert all(linux_safe(base + offset) for offset in range(count)), name
        source = (SCRIPTS / name).read_text(encoding="utf-8")
        assert str(base) in source, name


def test_every_background_process_is_its_own_killable_session() -> None:
    background = re.compile(r"(?:^|\s)&(?:\s|$)")
    for name in ISOLATED_RUNNERS:
        source = (SCRIPTS / name).read_text(encoding="utf-8")
        for command in shell_logical_lines(source):
            if background.search(command):
                assert command.startswith('"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" '), (
                    f"{name}: {command}"
                )


def test_visual_runner_uses_the_same_fail_closed_runtime_isolation() -> None:
    assert "run_formal_vehicle_visual_acceptance.sh" in ISOLATED_RUNNERS


def run_pgid_readiness_harness(mode: str) -> subprocess.CompletedProcess[str]:
    helper_source = HELPER.read_text(encoding="utf-8")
    harness = textwrap.dedent(
        f"""
        set -uo pipefail
        mode={mode!r}
        leader_pid=4242
        state="/tmp/tzcup_pgid_readiness_calls_${{BASHPID}}"
        capture="/tmp/tzcup_pgid_readiness_capture_${{BASHPID}}"
        kill_log="/tmp/tzcup_pgid_readiness_kills_${{BASHPID}}"
        printf '0\n' >"${{state}}"
        : >"${{kill_log}}"
        trap 'rm -f -- "${{state}}" "${{capture}}" "${{kill_log}}"' EXIT
        late_group_alive=0
        leader_alive=1

        ps() {{
          local calls
          calls="$(<"${{state}}")"
          calls=$((calls + 1))
          printf '%s\n' "${{calls}}" >"${{state}}"
          case "${{mode}}" in
            transition)
              if (( calls < 3 )); then printf ' 111\n'; else printf ' %s\n' "${{leader_pid}}"; fi
              ;;
            disappear)
              if (( calls == 1 )); then printf ' 111\n'; else return 1; fi
              ;;
            timeout)
              printf ' 111\n'
              ;;
            *) return 2 ;;
          esac
        }}
        kill() {{
          printf '%s\n' "$*" >>"${{kill_log}}"
          if [[ "${{1:-}}" == "-0" && "${{2:-}}" == "${{leader_pid}}" ]]; then
            if [[ "${{mode}}" == disappear && "$(<"${{state}}")" -ge 2 ]]; then
              return 1
            fi
            (( leader_alive == 1 ))
            return
          fi
          if [[ "${{1:-}}" == "-INT" && "${{2:-}}" == "${{leader_pid}}" ]]; then
            [[ "${{mode}}" != timeout ]] || late_group_alive=1
            leader_alive=0
            return 0
          fi
          if [[ ( "${{1:-}}" == "-TERM" || "${{1:-}}" == "-KILL" ) && "${{2:-}}" == "${{leader_pid}}" ]]; then
            leader_alive=0
            return 0
          fi
          if [[ "${{1:-}}" == "-0" && "${{2:-}}" == "--" && "${{3:-}}" == "-${{leader_pid}}" ]]; then
            (( late_group_alive == 1 ))
            return
          fi
          if [[ "${{1:-}}" == "-TERM" && "${{2:-}}" == "--" && "${{3:-}}" == "-${{leader_pid}}" ]]; then
            late_group_alive=0
            return 0
          fi
          return 1
        }}
        sleep() {{ :; }}
        setsid() {{ printf '%s\n' "$*" >"${{capture}}"; }}

        if formal_runtime_start_memory_watchdog "${{leader_pid}}" "/tmp/tzcup_pgid_readiness_${{BASHPID}}"; then
          rc=0
        else
          rc=$?
        fi
        if [[ -n "${{FORMAL_RUNTIME_MEMORY_WATCHDOG_PID}}" ]]; then
          wait "${{FORMAL_RUNTIME_MEMORY_WATCHDOG_PID}}" || true
        fi
        printf 'RESULT rc=%s calls=%s launched=%s\n' \
          "${{rc}}" "$(<"${{state}}")" "$([[ -e "${{capture}}" ]] && echo yes || echo no)"
        printf 'CLEANUP exact_pid=%s exact_group=%s wrong_group=%s\n' \
          "$(grep -Fqx -- '-INT 4242' "${{kill_log}}" && echo yes || echo no)" \
          "$(grep -Fqx -- '-TERM -- -4242' "${{kill_log}}" && echo yes || echo no)" \
          "$(grep -Eq -- '(^| )-111($| )' "${{kill_log}}" && echo yes || echo no)"
        if [[ -e "${{capture}}" ]]; then
          printf 'CAPTURE %s\n' "$(<"${{capture}}")"
        fi
        exit "${{rc}}"
        """
    )
    raw = subprocess.run(
        ["bash"],
        check=False,
        capture_output=True,
        input=(helper_source + "\n" + harness).encode("utf-8"),
        timeout=10,
    )
    return subprocess.CompletedProcess(
        raw.args,
        raw.returncode,
        raw.stdout.decode("utf-8", errors="replace"),
        raw.stderr.decode("utf-8", errors="replace"),
    )


@pytest.mark.skipif(os.name != "posix", reason="requires native POSIX process groups")
def test_memory_watchdog_waits_for_exact_setsid_pgid_transition() -> None:
    result = run_pgid_readiness_harness("transition")
    assert result.returncode == 0, result.stderr
    assert "RESULT rc=0 calls=3 launched=yes" in result.stdout
    assert "CLEANUP exact_pid=no exact_group=no wrong_group=no" in result.stdout
    assert "--leader-pid 4242 --pgid 4242" in result.stdout
    assert "--pgid 111" not in result.stdout


@pytest.mark.skipif(os.name != "posix", reason="requires native POSIX process groups")
def test_memory_watchdog_fails_closed_when_leader_disappears_during_pgid_wait() -> None:
    result = run_pgid_readiness_harness("disappear")
    assert result.returncode == 2
    assert "RESULT rc=2 calls=2 launched=no" in result.stdout
    assert "CLEANUP exact_pid=no exact_group=no wrong_group=no" in result.stdout
    assert "disappeared before creating its own process group" in result.stderr


@pytest.mark.skipif(os.name != "posix", reason="requires native POSIX process groups")
def test_memory_watchdog_fails_closed_when_pgid_never_becomes_exact() -> None:
    result = run_pgid_readiness_harness("timeout")
    assert result.returncode == 2
    assert "RESULT rc=2 calls=100 launched=no" in result.stdout
    assert "CLEANUP exact_pid=yes exact_group=yes wrong_group=no" in result.stdout
    assert "did not create its own process group within the bounded readiness window" in result.stderr
    assert "last_pgid=111" in result.stderr
