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
    assert 'formal_runtime_cleanup_groups "${mapping_partition}" "${mapping_launch_pid}" || exit 125' in RUNNER
    assert RUNNER.index(
        'formal_runtime_cleanup_groups "${mapping_partition}" "${mapping_launch_pid}" || exit 125'
    ) < RUNNER.index('write_state HARD_RESTART "${map_sha256}"')
    assert 'echo "process group ${pid} survived TERM then KILL"' in RUNNER
    assert "require_phase_processes mapping" in RUNNER
    assert "require_phase_processes cleaning" in RUNNER
    assert "except PermissionError" in HMI_SERVER


def test_preview_requires_observable_phase_progress_and_hmi_stage_receipts() -> None:
    assert "phase_progress_timeout_sec=600" in RUNNER
    assert 'canonical scan ready; starting standard autostart SLAM lifecycle' in RUNNER
    assert "dashboard_has_first_map" in RUNNER
    assert "mapping progress watchdog timed out" in RUNNER
    assert "cleaning progress watchdog timed out before hard-restart receipt" in RUNNER
    assert 'http.client.HTTPConnection("127.0.0.1", int(port), timeout=1.0)' in RUNNER
    assert "urllib.request" not in RUNNER
    assert 'wait_for_hmi_receipt MAPPING ""' in RUNNER
    assert 'wait_for_hmi_receipt MAP_SAVED "${map_sha256}"' in RUNNER
    assert 'wait_for_hmi_receipt HARD_RESTART "${map_sha256}"' in RUNNER
    assert 'wait_for_hmi_receipt RELOAD_LOCALIZE "${map_sha256}"' in RUNNER
    assert 'wait_for_hmi_receipt COVERAGE "${map_sha256}"' in RUNNER
    assert 'wait_for_hmi_receipt PRODUCT_TERMINAL "${map_sha256}"' in RUNNER
    assert RUNNER.index('write_state HARD_RESTART "${map_sha256}"') < RUNNER.rindex(
        'stop_pid "${state_publisher_pid}"'
    )
    assert RUNNER.index('wait_for_hmi_receipt PRODUCT_TERMINAL "${map_sha256}"') < RUNNER.index(
        'write_terminal "live mapping and same-map FullCoverage preview'
    )
    assert '"hmi_terminal_telemetry_sha256"' in RUNNER
