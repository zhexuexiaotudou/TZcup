from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"


RUNNERS = {
    name: (SCRIPTS / name).read_text(encoding="utf-8")
    for name in (
        "run_formal_service_door_runtime.sh",
        "run_formal_vehicle_sensor_runtime.sh",
        "run_formal_vehicle_mobility_runtime.sh",
        "run_formal_vehicle_visual_acceptance.sh",
        "run_formal_service_interface_acceptance.sh",
        "run_formal_single_episode_cleaning_mission.sh",
        "run_formal_ground_dirt_cleaning_runtime.sh",
        "run_formal_water_recovery_runtime.sh",
        "run_formal_first_map_dynamic_prerequisite.sh",
        "run_formal_saved_map_cleaning_lifecycle.sh",
        "run_formal_random_scene_perception.sh",
    )
}
ISOLATION = (SCRIPTS / "run_formal_runtime_isolation.sh").read_text(encoding="utf-8")


def test_every_runner_validates_the_dds_domain_range() -> None:
    for name, source in RUNNERS.items():
        assert "run_formal_runtime_isolation.sh" in source, name
        assert "formal_runtime_configure" in source, name
        assert re.search(r"ROS domain|ROS_DOMAIN|base_domain|domain", source), name
    assert "domain <= 101" in ISOLATION
    assert "domain <= 231" in ISOLATION


def test_multifield_runners_reserve_the_complete_domain_range() -> None:
    visual = RUNNERS["run_formal_vehicle_visual_acceptance.sh"]
    service = RUNNERS["run_formal_service_interface_acceptance.sh"]
    perception = RUNNERS["run_formal_random_scene_perception.sh"]
    assert 'formal_runtime_configure "${base_domain}" 2' in visual
    assert 'formal_runtime_configure "${base_domain}" "${#scenarios[@]}"' in service
    assert 'formal_runtime_configure "${base_domain}" "${episode_count}"' in perception


def test_contract_outputs_are_default_or_explicitly_published() -> None:
    assert "artifacts/formal_service_door_runtime.json" in RUNNERS[
        "run_formal_service_door_runtime.sh"
    ]
    assert "reports/engineering/formal_vehicle_runtime_report.json" in RUNNERS[
        "run_formal_vehicle_sensor_runtime.sh"
    ]
    assert "artifacts/formal_service_interface_acceptance.json" in RUNNERS[
        "run_formal_service_interface_acceptance.sh"
    ]
    assert "artifacts/formal_ground_dirt_cleaning_final_retry" in RUNNERS[
        "run_formal_ground_dirt_cleaning_runtime.sh"
    ]
    assert "artifacts/formal_water_recovery_acceptance.json" in RUNNERS[
        "run_formal_water_recovery_runtime.sh"
    ]
    assert "artifacts/formal_map_lifecycle_acceptance.json" in RUNNERS[
        "run_formal_saved_map_cleaning_lifecycle.sh"
    ]
    assert "artifacts/formal_random_scene_perception_acceptance.json" in RUNNERS[
        "run_formal_random_scene_perception.sh"
    ]
    assert "artifacts/formal_end_to_end_cleaning_mission_acceptance.json" in RUNNERS[
        "run_formal_single_episode_cleaning_mission.sh"
    ]


def test_map_runners_share_one_default_map_root() -> None:
    expected = ".work/formal_first_map_acceptance"
    assert expected in RUNNERS["run_formal_first_map_dynamic_prerequisite.sh"]
    assert expected in RUNNERS["run_formal_saved_map_cleaning_lifecycle.sh"]


def test_gazebo_runners_use_process_groups_partitions_and_waited_cleanup() -> None:
    for name, source in RUNNERS.items():
        assert "GZ_PARTITION" in source, name
        assert '"${FORMAL_RUNTIME_SESSION_PREFIX[@]}"' in source, name
        assert "formal_runtime_install_traps" in source, name
    assert 'wait "${pid}"' in ISOLATION
    # Let ros2 launch perform its ordered child shutdown first.  Escalate only
    # surviving process groups, while the partition sweep remains the final
    # INT/TERM/KILL backstop for processes which escaped the launch group.
    assert 'kill -INT "${pid}"' in ISOLATION
    assert "for signal in TERM KILL" in ISOLATION
    assert "for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL)" in ISOLATION
    assert "/proc" in ISOLATION


def test_e2e_uses_one_explicit_frozen_overlay_and_legal_default_domain() -> None:
    source = RUNNERS["run_formal_single_episode_cleaning_mission.sh"]
    assert "FORMAL_E2E_RUNTIME_OVERLAY" in source
    assert 'source "${RUNTIME_OVERLAY}/setup.bash"' in source
    assert 'ROS_DOMAIN="62"' in source
    assert 'formal_runtime_configure "${ROS_DOMAIN}"' in source


def test_mobility_runner_starts_real_safety_inputs_without_render_load() -> None:
    source = RUNNERS["run_formal_vehicle_mobility_runtime.sh"]
    assert "start_simulation_safety_inputs:=true" in source
    assert "high_bandwidth_sensor_runtime:=false" in source
    assert "start_localization:=false" in source


def test_fresh_evidence_roots_are_not_silently_reused() -> None:
    for name, source in RUNNERS.items():
        assert "Refusing stale" in source or "refusing to" in source, name
