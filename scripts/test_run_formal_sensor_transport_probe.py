from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PROBE = ROOT / "scripts" / "run_formal_sensor_transport_probe.sh"


def test_probe_requires_fresh_repo_work_attempt_and_frozen_runtime() -> None:
    source = PROBE.read_text(encoding="utf-8")

    for token in (
        "--runtime-ws",
        "--attempt-root",
        "--domain",
        'work_root="${repo_root}/.work"',
        '"${work_root}"/*',
        '[[ ! -e "${attempt_root}" && ! -L "${attempt_root}" ]]',
        'runtime_setup="${runtime_ws}/install/setup.bash"',
        'closure_manifest="${runtime_ws}/final_runtime_closure_manifest.json"',
        'mkdir -- "${attempt_root}"',
    ):
        assert token in source


def test_probe_keeps_canonical_snapshot_read_only_and_isolates_every_output() -> None:
    source = PROBE.read_text(encoding="utf-8")

    assert 'canonical_snapshot="${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json"' in source
    assert 'generate_formal_vehicle_snapshot.py" \\' in source
    assert '--check --output "${canonical_snapshot}"' in source
    assert 'formal_acceptance_session.py" start' in source
    assert '--snapshot "${canonical_snapshot}" --output "${session}"' in source
    for token in (
        'session="${attempt_root}/formal_sensor_probe_session.json"',
        'sensor_output="${attempt_root}/formal_vehicle_runtime_report.json"',
        'sensor_log="${attempt_root}/formal_vehicle_sensor_runtime.launch.log"',
        'fov_output="${attempt_root}/formal_vehicle_fov_occlusion_report.json"',
        'preembedded_world="${attempt_root}/preembedded_sensor_world.sdf"',
        'preembedded_report="${attempt_root}/preembedded_sensor_world.json"',
        'runtime_binding="${sensor_output}.runtime_binding.json"',
        'memory_preflight_json="${memory_base}.windows_memory_preflight.json"',
        'memory_watchdog_json="${memory_base}.memory_watchdog.json"',
        'export PYTHONDONTWRITEBYTECODE=1',
    ):
        assert token in source
    assert "FORMAL_SENSOR_SNAPSHOT" in source
    assert "unset FORMAL_SENSOR_SNAPSHOT" in source
    assert "domain intersects the Linux ephemeral UDP port range" in source


def test_probe_preserves_full_spec_runner_and_standalone_memory_guards() -> None:
    source = PROBE.read_text(encoding="utf-8")

    assert '"${repo_root}/scripts/run_formal_vehicle_sensor_runtime.sh"' in source
    assert "high_bandwidth_sensor_runtime:=false" not in source
    assert "start_high_bandwidth_sensor_bridges:=false" not in source
    assert "FORMAL_ORCHESTRATED_STEP_SESSION=0" in source
    assert "FORMAL_MEMORY_WATCHDOG_ENABLED=1" in source
    assert "FORMAL_WINDOWS_MEMORY_GUARD_ENABLED=1" in source
    assert 'partition="tzcup_formal_sensor_transport_probe_${domain}_$$_$(date +%s)"' in source
    assert 'lock_path="/tmp/tzcup_formal_sensor_transport_probe_${domain}_$$_lock"' in source


def test_probe_attests_without_deleting_or_reaping_partition_evidence() -> None:
    source = PROBE.read_text(encoding="utf-8")

    assert 'attestation="${attempt_root}/cleanup_attestation.json"' in source
    assert 'needle = ("GZ_PARTITION=" + sys.argv[1]).encode()' in source
    assert "partition_survivor_pids" in source
    assert "partition_survivors" in source
    assert "surviving_group_processes" in source
    assert "lock_released" in source
    assert "partition_survivor_count" in source
    assert "lock_reacquirable" in source
    assert "evidence_deleted_by_probe" in source
    assert "\nformal_runtime_cleanup_partition " not in source
    assert "rm -rf" not in source
    assert "rm -f" not in source
    assert "exit 125" in source
