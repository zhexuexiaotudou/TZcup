from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_runner_uses_gazebo_camera_product_graph_and_evaluator_truth_only():
    source = (ROOT / "scripts/run_formal_random_scene_perception.sh").read_text(encoding="utf-8")
    assert "ros2 run sanitation_campus_scenario sanitation-campus-scenario generate" in source
    assert "formal_campus.launch.py" in source
    assert "pc_open_vocab_product_adapter" in source
    assert "formal_random_scene_perception_evaluator" in source
    assert 'public_manifest_path:="${episode_root}/scenario/public/episode_manifest.json"' in source
    assert "evaluator/ground_truth.json" in source
    assert "cv2.imwrite" not in source
    assert "synthetic/offline image is eligible" in source
    assert "start_pedestrians:=true" in source
    assert "FORMAL_PERCEPTION_EPISODE_COUNT:-30" in source
    assert "formal_minimum_episode_count=30" in source
    assert "smoke-scale" in source
    assert "map_index=$((index % formal_validation_map_count))" in source
    assert "mission_index=$((index / formal_validation_map_count))" in source
    assert '--minimum-episodes "${formal_minimum_episode_count}"' in source
    assert "tf_static_once.yaml" in source
    assert "tf_once.yaml" in source
    assert "odom_once.yaml" in source
    assert "joint_states_once.yaml" in source
    assert "product_score_threshold.txt" in source
    assert "timeout 5 ros2 param dump" in source
    assert "ros2 param get" not in source
    assert "ros2 topic list" not in source
    assert "ros2 node list" not in source
    assert "wait_for_ros_graph.py" in source
    assert "campus_graph_readiness.json" in source
    assert "product_graph_readiness.json" in source
    assert "product_startup_diagnostic.yaml" in source
    assert "ros2 topic echo --once --qos-durability transient_local" in source
    assert "/perception/open_vocab/diagnostics diagnostic_msgs/msg/DiagnosticArray" in source
    assert "overlay_preflight.log" in source
    assert "timeout 15 ros2 pkg prefix" not in source
    assert "AMENT_PREFIX_PATH" in source
    assert "CYCLONEDDS_MAX_AUTO_PARTICIPANT_INDEX" in source
    assert "CYCLONEDDS_MAX_UNICAST_PORT" in source
    assert "CYCLONEDDS_CONFIG_RESOLVED" in source
    assert 'active_uri.removeprefix("file://")' in source
    assert "cyclonedds_udp_port_bound" in source
    assert "share/ament_index/resource_index/packages" in source
    assert "source_install_hash" in source
    assert "FILE_CLOSURE OK" in source
    assert "get_package_prefix(package)" in source
    assert "timeout_drvfs_file_closure_authoritative" in source
    assert "formal_physical_grasp.launch.py" in source
    assert "formal_campus.launch.py" in source
    assert "formal_vehicle_sim.launch.py" in source
    assert "IMPORT sanitation_perception_interfaces OK" in source
    assert "status=READY" in source
    assert "product_source_manifest.sha256" in source
    assert "-p score_threshold:=0.005" in source
    assert "-p fallen_leaves_score_threshold:=0.0025" in source
    assert "-p dust_or_soil_score_threshold:=0.002" in source
    assert "-p puddle_score_threshold:=0.003" in source
    assert "diagnose_formal_dosod_frame.py" in source
    assert "tf2_echo" in source
    assert "formal_source_bound_preflight.sh" in source
    assert "formal_source_bound_preflight" in source
    assert "formal_source_bound_verify_overlay" in source
    assert "formal_source_bound_perception_roots" in source
    assert "FORMAL_VEHICLE_RUNTIME_WS" in source
    assert "FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST" in source
    assert "FORMAL_ACCEPTANCE_SESSION" in source
    assert 'runtime_binding="${output_root}/runtime_gate_binding.json"' in source
    assert '--runtime-binding "${runtime_binding}"' in source
    assert source.index("formal_source_bound_preflight") < source.index("ros2 launch")
    assert "FORMAL_CAMPUS_STAGE1_SETUP" not in source
    assert "FORMAL_PERCEPTION_ACCEPTANCE_SETUP" not in source
    assert "/home/zhexu/tzcup_integrated_build" not in source


def test_cyclonedds_preflight_compares_resolved_file_uri_paths():
    canonical = ROOT / "config" / "cyclonedds_localhost.xml"
    helper_spelling = ROOT / "scripts" / ".." / "config" / "cyclonedds_localhost.xml"
    unrelated = ROOT / "config" / "formal_acceptance_thresholds.yaml"
    assert helper_spelling.resolve() == canonical.resolve()
    assert unrelated.resolve() != canonical.resolve()
