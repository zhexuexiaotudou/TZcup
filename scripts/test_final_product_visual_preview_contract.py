from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER = (ROOT / "scripts/run_final_product_visual_preview.sh").read_text(encoding="utf-8")
WINDOWS = (ROOT / "scripts/run_final_product_visual_preview.ps1").read_text(encoding="utf-8")
HMI = (ROOT / "starter_ws/src/sanitation_hmi/web/demo.html").read_text(encoding="utf-8")
HMI_SERVER = (ROOT / "starter_ws/src/sanitation_hmi/sanitation_hmi/live_server.py").read_text(encoding="utf-8")
HMI_STATE = (ROOT / "starter_ws/src/sanitation_hmi/sanitation_hmi/live_state.py").read_text(encoding="utf-8")


def test_preview_uses_final_a300_map_then_hard_restart_topology() -> None:
    assert "FINAL_VISUAL_PREVIEW_NOT_PRODUCT_PASS" in RUNNER
    assert '"product_pass": False' in RUNNER
    assert '"field_dimensions_m": [200, 100]' in RUNNER
    assert '"vehicle": "A300"' in RUNNER
    assert RUNNER.count("formal_campus_map_lifecycle.launch.py") == 2
    assert RUNNER.count("lidar_bridge_ready_timeout_sec:=600") == 1
    assert 'gazebo_gui=false' in RUNNER
    assert '--gazebo-gui true|false' in RUNNER
    assert 'mission_mode:=mapping gui:="${gazebo_gui}"' in RUNNER
    assert 'mission_mode:=cleaning cleaning_planner:=full_coverage gui:="${gazebo_gui}"' in RUNNER
    assert RUNNER.index("mission_mode:=mapping") < RUNNER.index("mission_mode:=cleaning")
    assert "write_state HARD_RESTART" in RUNNER
    assert 'formal_runtime_cleanup_groups "${mapping_partition}" "${mapping_launch_pid}"' in RUNNER


def test_preview_reuses_live_hmi_and_refuses_formal_acceptance_claims() -> None:
    assert "python3 -m sanitation_hmi.live_server" in RUNNER
    assert "publish_final_product_visual_state.py" in RUNNER
    assert "perception_provider\": \"unavailable\"" in RUNNER
    assert "formal_product_acceptance\": False" in RUNNER
    assert "run_formal_first_map_dynamic_prerequisite.sh" not in RUNNER
    assert "run_formal_saved_map_cleaning_lifecycle.sh" not in RUNNER
    assert "AUTO-17" not in RUNNER and "Ackermann" not in RUNNER
    assert RUNNER.count("simulation_initial_estop_active:=false") == 2
    assert 'mission_config:="${preview_hmi_mission}"' in RUNNER
    assert "preview HMI requires the declared formal 200x100m episode geofence" in RUNNER
    assert "首次建图中；清扫组件将在重启后的清扫阶段开始统计" in HMI
    assert "公开 geofence" in HMI
    assert 'Odometry, "/odom", self._on_formal_odometry' in HMI_SERVER
    assert 'TwistStamped,\n            "/base_controller/cmd_vel"' in HMI_SERVER
    assert 'lookup_transform(\n                "map", "base_footprint", Time()' in HMI_SERVER
    assert "odometry_preview_pose_odom" in HMI_STATE
    assert "/base_controller/cmd_vel" in HMI_STATE
    assert "Odom 预演轨迹（非真值/非地图定位）" in HMI


def test_preview_is_fresh_scoped_and_has_a_windows_preflight_entry() -> None:
    assert "refusing stale preview run root" in RUNNER
    assert "preview run root must remain below TZcup project root" in RUNNER
    assert "preview runtime workspace must remain below TZcup project root" in RUNNER
    assert "preview episode root must remain below TZcup project root" in RUNNER
    assert "--preflight" in RUNNER
    assert '"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch' in RUNNER
    assert 'coverage_report="${cleaning_root}/coverage_execution.json"' in RUNNER
    assert 'report.get("terminal_state") != "COMPLETED"' in RUNNER
    assert 'kill -KILL -- "-${pid}"' in RUNNER
    assert "MappingRosDomain and CleaningRosDomain must differ" in WINDOWS
    assert "run_final_product_visual_preview.sh" in WINDOWS
    assert "Replace('\\', '/')" in WINDOWS
    assert "require_dashboard_port_available" in RUNNER
    assert 'sock.bind(("127.0.0.1", port))' in RUNNER
    assert RUNNER.index("require_dashboard_port_available") < RUNNER.index('if "${preflight_only}"')
    assert RUNNER.index('formal_runtime_memory_preflight "${run_root}/windows_memory_preflight"') < RUNNER.index(
        'if "${preflight_only}"'
    )
    assert RUNNER.index("require_runtime_package_provenance") < RUNNER.index('if "${preflight_only}"')
    assert "check_runtime_overlay_freshness.py" in RUNNER
    assert "runtime_overlay_freshness.json" in RUNNER
    assert RUNNER.index("require_runtime_overlay_freshness") < RUNNER.index('if "${preflight_only}"')
    assert RUNNER.index("require_expanded_wheel_surface") < RUNNER.index('if "${preflight_only}"')
    assert "A300_EXPANDED_WHEEL_SURFACE_PREFLIGHT_PASSED" in RUNNER


def test_preview_uses_formal_dds_gazebo_isolation_with_safe_distinct_domains() -> None:
    assert 'source "${repo_root}/scripts/run_formal_runtime_isolation.sh"' in RUNNER
    assert 'formal_runtime_domain_is_linux_safe "${mapping_ros_domain}"' in RUNNER
    assert 'formal_runtime_domain_is_linux_safe "${cleaning_ros_domain}"' in RUNNER
    assert 'formal_runtime_configure "${mapping_ros_domain}"' in RUNNER
    assert 'mapping and cleaning ROS domains must be distinct Linux-safe domains' in RUNNER
    assert 'mapping_partition="tzcup_final_visual_mapping_${mapping_ros_domain}_$$"' in RUNNER
    assert 'cleaning_partition="tzcup_final_visual_cleaning_${cleaning_ros_domain}_$$"' in RUNNER
    assert 'formal_runtime_cleanup_groups "${mapping_partition}" "${mapping_launch_pid}"' in RUNNER
    assert 'formal_runtime_cleanup_groups "${cleaning_partition}" "${cleaning_launch_pid}"' in RUNNER
    assert 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp' not in RUNNER
    assert "Test-LinuxSafeRosDomain" in WINDOWS
    assert "Linux-safe ROS domains: 0..101 or 215..231" in WINDOWS


def test_preview_fails_closed_on_memory_or_process_cleanup_faults() -> None:
    assert 'formal_runtime_memory_preflight "${run_root}/windows_memory_preflight"' in RUNNER
    assert RUNNER.count("formal_runtime_start_memory_watchdog") == 2
    assert RUNNER.count("finish_memory_watchdog") == 3
    assert "formal_runtime_memory_watchdog_tripped" in RUNNER
    assert 'return "${FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE}"' in RUNNER
    assert "formal_runtime_install_traps cleanup" in RUNNER
    guarded_mapping_cleanup = (
        'formal_runtime_cleanup_groups "${mapping_partition}" "${mapping_launch_pid}"'
    )
    assert guarded_mapping_cleanup in RUNNER
    assert '"mapping process cleanup failed closed" "mapping_cleanup"' in RUNNER
    assert RUNNER.index(guarded_mapping_cleanup, RUNNER.index('write_state MAP_SAVED')) < RUNNER.index(
        'write_state HARD_RESTART "${map_sha256}"'
    )
    assert 'echo "process group ${pid} survived TERM then KILL"' in RUNNER
    assert "require_phase_processes mapping" in RUNNER
    assert "require_phase_processes cleaning" in RUNNER
    assert "except PermissionError" in HMI_SERVER


def test_preview_requires_observable_phase_progress_and_hmi_stage_receipts() -> None:
    assert "phase_progress_timeout_sec=600" in RUNNER
    assert 'canonical scan ready; starting standard autostart SLAM lifecycle' in RUNNER
    assert "dashboard_has_first_map" in RUNNER
    assert "mapping progress watchdog timed out" in RUNNER
    assert "mapping explorer progress watchdog timed out after first map" in RUNNER
    assert "dashboard_live_map_tf_pose" in RUNNER
    assert 'source.get("status") != "live"' in RUNNER
    assert 'source.get("topic") != "/tf map->base_footprint"' in RUNNER
    assert "evaluation-only ground-truth overlay" in RUNNER
    assert "math.hypot(x1 - x0, y1 - y0) >= 0.05" in RUNNER
    assert "mapping frontier motion watchdog timed out" in RUNNER
    assert "dashboard_observed_lifecycle_fraction" in RUNNER
    assert 'lifecycle.get("status") != "observed"' in RUNNER
    assert 'value.get("observed_fraction")' in RUNNER
    assert "current >= previous + 0.001" in RUNNER
    assert "mapping lifecycle stagnation watchdog timed out" in RUNNER
    assert "initial_scan_sweep_blocked" in RUNNER
    assert "blocked_excessive_nav2_failures" in RUNNER
    assert 'value.get("terminal") is True' in RUNNER
    assert 'reason = json.dumps(value.get("reason"), ensure_ascii=False)' in RUNNER
    assert "publish_preview_terminal_with_hmi" in RUNNER
    assert "guard_or_publish_terminal" in RUNNER
    assert "mapping explorer terminal: state=${explorer_state}; reason=${explorer_reason}" in RUNNER
    assert '"failure_source_state"' in RUNNER
    assert '"failure_reason"' in RUNNER
    assert "initial_scan_sweep_complete" in RUNNER
    assert 'value.get("initial_scan_sweep_state")' in RUNNER
    assert '"${initial_scan_sweep_state}" == "complete"' in RUNNER
    assert "frontier_goal_requested|frontier_goal_reached|frontier_goal_failed|navigating_frontier" in RUNNER
    assert "same planner turn as initial_scan_sweep_complete" in RUNNER
    assert "periodic SLAM publication alone cannot prove either claim" in RUNNER
    assert 'http.client.HTTPConnection("127.0.0.1", int(port), timeout=3.0)' in RUNNER
    assert "urllib.request" not in RUNNER
    assert 'fetch_json("/api/v1/telemetry")' in RUNNER
    assert "os.replace(temporary, path)" in RUNNER
    assert "for attempt in {1..5}" in RUNNER
    assert 'wait_for_hmi_receipt MAPPING ""' in RUNNER
    assert 'wait_for_hmi_receipt MAP_SAVED "${map_sha256}"' in RUNNER
    assert 'wait_for_hmi_receipt HARD_RESTART "${map_sha256}"' in RUNNER
    assert 'wait_for_hmi_receipt RELOAD_LOCALIZE "${map_sha256}"' in RUNNER
    assert "dashboard_live_coverage_state" in RUNNER
    assert 'coverage.get("status") != "live"' in RUNNER
    assert '"PLANNING", "TRANSIT", "CLEANING", "FAILED", "COMPLETED"' in RUNNER
    assert "PLANNING|TRANSIT|CLEANING" in RUNNER
    assert 'write_state COVERAGE "${map_sha256}"' in RUNNER
    assert 'wait_for_hmi_receipt COVERAGE "${map_sha256}"' in RUNNER
    assert 'save_hmi_phase_snapshot cleaning COVERAGE "${map_sha256}"' in RUNNER
    assert 'save_hmi_phase_snapshot cleaning RELOAD_LOCALIZE "${map_sha256}"' not in RUNNER
    assert "coverage report appeared without a verified live coverage HMI receipt and snapshot" in RUNNER
    assert "FAILED/COMPLETED remain governed by the" in RUNNER
    assert 'wait_for_hmi_receipt PRODUCT_TERMINAL "${active_map_sha256}"' in RUNNER
    assert 'write_state PRODUCT_TERMINAL "${active_map_sha256}"' in RUNNER
    assert 'save_hmi_phase_snapshot terminal PRODUCT_TERMINAL "${active_map_sha256}"' in RUNNER
    assert "hmi_terminal_telemetry.json" in RUNNER
    assert "HMI PRODUCT_TERMINAL receipt was unavailable and no terminal snapshot was saved" in RUNNER
    assert RUNNER.index('write_state HARD_RESTART "${map_sha256}"') < RUNNER.rindex(
        'stop_pid "${state_publisher_pid}"'
    )
    assert RUNNER.rindex("publish_preview_terminal_with_hmi") < RUNNER.index(
        "FINAL_VISUAL_PREVIEW_TERMINAL="
    )
    assert '"hmi_terminal_telemetry_sha256"' in RUNNER
    assert '"hmi_phase_telemetry_sha256"' in RUNNER
    assert 'save_hmi_phase_snapshot mapping MAP_SAVED "${map_sha256}"' in RUNNER
    assert 'save_hmi_phase_snapshot hard_restart HARD_RESTART "${map_sha256}"' in RUNNER
    assert "hmi_mapping_telemetry.json" in RUNNER
    assert "hmi_hard_restart_telemetry.json" in RUNNER
    assert "hmi_cleaning_telemetry.json" in RUNNER
    assert "cleaning_progress_deadline" in RUNNER
    assert "coverage_stage_not_observed" in RUNNER
    assert 'dashboard["live_inputs"]["cleaning_motor_status"]' in RUNNER
    assert '"${dashboard_output}/dashboard_telemetry.json"' in RUNNER
    assert '"${dashboard_telemetry}"' not in RUNNER
    assert "mapping_cleaning_motor_nonhealthy_deadline=$((SECONDS + 90))" in RUNNER
    assert 'cleaning_fault_code="persistent_cleaning_motor_${cleaning_motor_state}"' in RUNNER
    assert '"mapping_cleaning_motor_${cleaning_motor_state}"' in RUNNER
    assert '[[ "${cleaning_motor_state}" == "healthy" ]]' in RUNNER
    assert "Unknown, malformed, stale, and faulted telemetry" in RUNNER
    assert RUNNER.count("publish_preview_terminal_with_hmi") >= 15
