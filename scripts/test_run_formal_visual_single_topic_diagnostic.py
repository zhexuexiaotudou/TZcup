from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_single_topic_diagnostic_is_memory_guarded_and_bounded() -> None:
    source = (
        ROOT / "scripts/run_formal_visual_single_topic_diagnostic.sh"
    ).read_text(encoding="utf-8")
    assert "formal_runtime_memory_preflight" in source
    assert "formal_runtime_start_memory_watchdog" in source
    assert "formal_runtime_memory_watchdog_tripped" in source
    assert "formal_runtime_cleanup_partition" in source
    assert "standalone-only; refusing nested orchestration" in source
    assert "gz-physics-dartsim-plugin" in source
    assert "timeout -k 5 45 ros2 topic echo" in source
    assert 'domain_id=225' in source
    assert '"${domain_id}" -le 231' in source
    assert '[0, 231]' in source
    assert 'topic="/formal_visual/front_left"' in source
    assert "ros2 run ros_gz_image image_bridge" in source
    assert "gz topic -e" in source
    assert "extract_formal_gz_image_metadata.py" in source
    assert "image_bridge_ldd.txt" in source
    assert "capture_formal_transport_process_maps.py" in source
    assert "transport_process_maps.json" in source
    assert "wait \"${bridge_pid}\"" not in source
    assert "wait \"${gz_pid}\"" not in source
    assert "expected_uncompressed_data_bytes_from_step" in source
    assert "Subscription count: 1" in source
    assert "for attempt in $(seq 1 20)" in source
    assert "finalize_formal_visual_single_topic_diagnostic.py" in source
    inner = source.split("setsid bash -c '", 1)[1].split("' bash", 1)[0]
    assert "'" not in inner
    assert "--once --field width" in source
    assert "grep -qx \"1600\"" in source


def test_single_topic_diagnostic_does_not_start_full_vehicle_launch() -> None:
    source = (
        ROOT / "scripts/run_formal_visual_single_topic_diagnostic.sh"
    ).read_text(encoding="utf-8")
    assert "ros2 launch sanitation_vehicle_description" not in source
    assert "formal_vehicle_visual_acceptance.sdf" in source
    assert "prepare_formal_visual_single_topic_world.py" in source
    assert '"${single_world}"' in source
    assert source.count('ros2 run ros_gz_image image_bridge') == 1
