from pathlib import Path

import pytest

from prepare_manual_gazebo_gui import remove_camera_tracking


ROOT = Path(__file__).resolve().parents[1]


def test_representative_frame_is_taken_from_the_completed_end_of_video():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")

    assert 'ffmpeg -nostdin -y -sseof -5 -i "${OUTPUT_DIR}/visual_demo.mp4"' in launcher
    assert '-frames:v 1 -update 1 "${OUTPUT_DIR}/visual_demo_frame.png"' in launcher
    assert 'ffmpeg -nostdin -y -ss 5 -i "${OUTPUT_DIR}/visual_demo.mp4"' not in launcher


def test_launcher_exposes_optimized_and_legacy_coverage_profiles():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")
    wrapper = (ROOT / "scripts" / "run_visual_demo.ps1").read_text(encoding="utf-8")
    frozen = (ROOT / "scripts" / "run_frozen_coverage_trial.ps1").read_text(
        encoding="utf-8"
    )

    assert 'COVERAGE_PROFILE="optimized"' in launcher
    assert '--coverage-profile) COVERAGE_PROFILE="$2"' in launcher
    assert '"${COVERAGE_PROFILE}" == "legacy"' in launcher
    assert 'competition_demo_area.yaml' in launcher
    assert 'coverage_demo_overlap.yaml' in launcher
    assert '[ValidateSet("optimized", "legacy")]' in wrapper
    assert '"--coverage-profile", $CoverageProfile' in wrapper
    assert '[ValidateSet("optimized", "legacy")]' in frozen


def test_launcher_can_run_a_bounded_physical_dynamic_matrix():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")
    wrapper = (ROOT / "scripts" / "run_visual_demo.ps1").read_text(encoding="utf-8")
    world = (
        ROOT / "starter_ws/src/sanitation_worlds/worlds/sanitation_competition_demo.sdf"
    ).read_text(encoding="utf-8")

    assert 'DYNAMIC_OBSTACLE_TRIALS=0' in launcher
    assert '--dynamic-obstacle-trials) DYNAMIC_OBSTACLE_TRIALS="$2"' in launcher
    assert 'dynamic_probe_executable="$(ros2 pkg prefix sanitation_tasks)' in launcher
    assert '"${dynamic_probe_executable}" --ros-args' in launcher
    assert 'model_name:="dynamic_pedestrian_box"' in launcher
    assert 'dynamic_obstacle_report.json' in launcher
    assert 'dynamic_probe_code' in launcher
    assert '[int]$DynamicObstacleTrials = 0' in wrapper
    assert '"--dynamic-obstacle-trials", "$DynamicObstacleTrials"' in wrapper
    assert '<model name="dynamic_pedestrian_box">' in world


def test_headless_matrix_can_select_ogre_without_changing_gui_default():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")
    wrapper = (ROOT / "scripts" / "run_visual_demo.ps1").read_text(encoding="utf-8")

    assert 'SIMULATION_RENDER_ENGINE="ogre2"' in launcher
    assert '--simulation-render-engine) SIMULATION_RENDER_ENGINE="$2"' in launcher
    assert 'case "${SIMULATION_RENDER_ENGINE}" in ogre2|ogre)' in launcher
    assert 'f"<render_engine>{render_engine}</render_engine>"' in launcher
    assert 'server_headless_rendering="false"' in launcher
    assert 'headless_rendering:="${server_headless_rendering}"' in launcher
    assert '[ValidateSet("ogre2", "ogre")]' in wrapper
    assert '[string]$SimulationRenderEngine = "ogre2"' in wrapper
    stage4v = (
        ROOT / "starter_ws/src/sanitation_bringup/launch/stage4v_localization.launch.py"
    ).read_text(encoding="utf-8")
    assert "DeclareLaunchArgument('headless_rendering', default_value='true')" in stage4v
    assert "'headless_rendering': LaunchConfiguration('headless_rendering')" in stage4v


def test_headless_formal_matrix_still_scores_all_cleaning_targets():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")

    assert 'if [[ "${GAZEBO_TRAIL}" -eq 1 ]]; then' in launcher
    assert '[[ "${GAZEBO_TRAIL}" -eq 1 ]] && summary_args+=(--targets-required)' in launcher
    assert '"${GUI}" -eq 1 && "${GAZEBO_TRAIL}" -eq 1' not in launcher


def test_optimized_connectors_use_nav2_behaviors_and_dedicated_controllers():
    probe = (
        ROOT
        / "starter_ws/src/sanitation_coverage/sanitation_coverage/coverage_probe.py"
    ).read_text(encoding="utf-8")
    nav2 = (
        ROOT / "starter_ws/src/sanitation_navigation/config/nav2.yaml"
    ).read_text(encoding="utf-8")

    assert 'ActionClient(self, Spin, "/spin")' in probe
    assert 'ActionClient(\n            self, DriveOnHeading, "/drive_on_heading"' in probe
    assert 'ActionClient(self, BackUp, "/backup")' in probe
    assert 'create_publisher(\n            Twist, "/cmd_vel_nav"' not in probe
    assert '"CLEAN": "CleanPath"' in probe
    assert '"REPAIR": "RepairPath"' in probe
    assert "controller_plugins: [FollowPath, CleanPath, RepairPath]" in nav2
    assert "desired_linear_vel: 0.65" in nav2
    assert "use_velocity_scaled_lookahead_dist: false" in nav2


def test_formal_matrix_preserves_five_seed_a_b_and_one_mcap_replay_source():
    matrix = (
        ROOT / "scripts" / "run_coverage_optimizer_matrix.sh"
    ).read_text(encoding="utf-8")

    assert 'OPTIMIZED_SEEDS="132,133,134,135,136"' in matrix
    assert 'LEGACY_SEEDS="140,141,142,143,144"' in matrix
    assert 'MCAP_SEED="132"' in matrix
    assert 'run_profile optimized "${OPTIMIZED_SEEDS}" selected' in matrix
    assert 'run_profile legacy "${LEGACY_SEEDS}" baseline' in matrix
    assert 'Refusing to overwrite retained matrix status' in matrix
    assert 'args+=(--no-mcap)' in matrix


def test_readiness_bypasses_stale_ros_daemon():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")
    readiness = (ROOT / "scripts" / "ros_runtime_readiness.py").read_text(
        encoding="utf-8"
    )

    assert "ros2 node list --no-daemon --spin-time 3" in launcher
    assert 'timeout 155 python3 "${ROOT}/scripts/ros_runtime_readiness.py"' in launcher
    assert '"${OUTPUT_DIR}/runtime_readiness.json"' in launcher
    assert "node.get_topic_names_and_types()" in readiness
    assert "node.get_service_names_and_types()" in readiness
    for action in ("compute_coverage_path", "follow_path", "navigate_to_pose"):
        assert f'"/{action}/_action/send_goal"' in readiness
    assert '"/controller_server/get_state"' in readiness
    assert '"/planner_server/get_state"' in readiness
    assert "controller_state == 3" in readiness
    assert "planner_state == 3" in readiness
    assert "ros2 action list" not in launcher


def test_nav2_waits_for_localization_tf_and_exact_lifecycle_state():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")
    readiness = (ROOT / "scripts" / "ros_runtime_readiness.py").read_text(
        encoding="utf-8"
    )

    localization_gate = launcher.index("tf2_echo odom base_footprint")
    navigation_start = launcher.index(
        "setsid ros2 launch sanitation_navigation navigation.launch.py"
    )
    assert localization_gate < navigation_start
    assert "grep -Fq 'Translation:'" in launcher
    assert "grep -Fq 'Rotation:'" in launcher
    assert '"/controller_server/get_state"' in readiness
    assert '"/planner_server/get_state"' in readiness
    assert "controller_state == 3" in readiness
    assert "planner_state == 3" in readiness


def test_camera_follow_discovery_has_a_hard_timeout():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")

    assert "for _ in $(seq 1 10)" in launcher
    assert 'gz_topics="$(timeout 3 gz topic -l 2>/dev/null || true)"' in launcher
    assert 'gz_topics="$(gz topic -l 2>/dev/null || true)"' not in launcher


def test_manual_control_removes_camera_tracking_instead_of_trying_to_release_it():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")
    config = (ROOT / "starter_ws" / "src" / "sanitation_gazebo_control" / "config" / "mission_control_demo.config").read_text(encoding="utf-8")

    free_config = remove_camera_tracking(config)
    assert 'filename="CameraTracking"' in config
    assert 'filename="CameraTracking"' not in free_config
    assert 'filename="InteractiveViewControl"' in free_config
    gui_launch = launcher.index('setsid "${gazebo_gui_env[@]}" gz sim -g')
    follow_guard = launcher.index(
        'if [[ "${GUI}" -eq 1 && "${MANUAL_CONTROL}" -eq 0 ]]'
    )
    assert gui_launch < follow_guard
    assert '[[ "${GUI}" -eq 0 || "${MANUAL_CONTROL}" -eq 1 ]]' in launcher


def test_manual_gui_preparation_fails_closed_without_one_tracking_plugin():
    with pytest.raises(ValueError, match="exactly one CameraTracking"):
        remove_camera_tracking("<window></window>\n")


def test_emergency_stop_availability_waits_for_dashboard_but_stays_bounded():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")
    availability = (ROOT / "scripts" / "emergency_stop_availability.py").read_text(
        encoding="utf-8"
    )

    assert 'timeout 20 python3 "${ROOT}/scripts/emergency_stop_availability.py"' in launcher
    assert "publisher.get_subscription_count()" in availability
    assert "publish_count < 5" in availability
    assert '"/emergency_stop" in payload.get("topics_seen", [])' in availability
    assert "Unable to publish the bounded emergency-stop availability pulse." in launcher


def test_wslg_gui_is_launched_directly_for_native_plugin_backend():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")

    assert 'gui:=false random_seed:="${RANDOM_SEED}"' in launcher
    assert 'setsid "${gazebo_gui_env[@]}" gz sim -g --gui-config "${gui_config}"' in launcher
    assert '> "${OUTPUT_DIR}/gazebo_gui.log" 2>&1 &' in launcher
    assert 'gui:="${gui_value}"' not in launcher
    assert launcher.index('timeout 155 python3 "${ROOT}/scripts/ros_runtime_readiness.py"') < launcher.index(
        'setsid "${gazebo_gui_env[@]}" gz sim -g --gui-config "${gui_config}"'
    )


def test_wslg_gui_uses_a_visible_software_viewport_and_pixel_probe():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")
    wrapper = (ROOT / "scripts" / "run_visual_demo.ps1").read_text(encoding="utf-8")

    assert 'GAZEBO_GUI_RENDERER="auto"' in launcher
    assert 'gazebo_gui_renderer="software"' in launcher
    assert 'GALLIUM_DRIVER=llvmpipe' in launcher
    assert 'LIBGL_ALWAYS_SOFTWARE=1' in launcher
    assert 'QT_QPA_PLATFORM=xcb' in launcher
    assert 'gazebo_viewport_probe.py' in launcher
    assert 'Gazebo 3D viewport is black' in launcher
    assert 'gazebo_gui_renderer.json' in launcher
    assert '[ValidateSet("auto", "d3d12", "software")]' in wrapper
    assert '"--gazebo-gui-renderer", $GazeboGuiRenderer' in wrapper


def test_wslg_gui_is_health_checked_and_supervised_during_the_mission():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")

    assert 'gazebo_gui_pid="$!"' in launcher
    assert "'/sanitation_gazebo_mission_control'" in launcher
    assert 'kill -0 "${gazebo_gui_pid}"' in launcher
    assert 'gui_closed_during_mission=1' in launcher
    assert 'runtime_termination_status="OPERATOR_GUI_CLOSED"' in launcher
    assert '"${OUTPUT_DIR}/launcher_termination.json"' in launcher
    assert "pgrep -f 'gz sim.*-g'" not in launcher


def test_coverage_supervisor_waits_for_the_real_setsid_child():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")

    assert (
        'PYTHONUNBUFFERED=1 setsid --wait timeout "${MISSION_TIMEOUT_SEC}"'
    ) in launcher
    assert "ros2 pkg prefix sanitation_coverage" in launcher
    assert '"${coverage_executable}" --ros-args' in launcher
    assert "ros2 run sanitation_coverage coverage_probe" not in launcher
    assert 'setsid timeout "${MISSION_TIMEOUT_SEC}" ros2 run' not in launcher
    assert '"${OUTPUT_DIR}/coverage_process_exit_code.txt"' in launcher


def test_windows_wrapper_runs_the_wslg_window_guard():
    wrapper = (ROOT / "scripts" / "run_visual_demo.ps1").read_text(encoding="utf-8")
    guard = (ROOT / "scripts" / "wslg_window_guard.ps1").read_text(encoding="utf-8")

    assert '"wslg_window_guard.ps1"' in wrapper
    assert '"wslg_window_guard.jsonl"' in wrapper
    assert '"wslg_window_guard.stop"' in wrapper
    assert '"wslg_window_guard.failed"' in wrapper
    assert '"$wslRoot/scripts/prepare_wslg_runtime.sh"' in wrapper
    assert '"Preparing the WSLg shared-memory transport..."' in wrapper
    assert "$prepareExitCode -eq 10" in wrapper
    assert 'wsl.exe --shutdown' in wrapper
    assert 'wsl.exe --list --running --quiet' in wrapper
    assert 'other WSL distributions are running' in wrapper
    assert "$wslExitCode -in @(4, 7)" in wrapper
    assert "$wslgRecoveryAttempted" in wrapper
    assert '"COPY MODE detected"' in wrapper
    assert "restarting WSLg once and retrying the demo..." in wrapper
    assert "Gazebo GUI exited before native controls loaded" in wrapper
    assert "launcher_termination_early_gui_exit_attempt.json" in wrapper
    assert 'launcher_termination_copy_mode_attempt.json' in wrapper
    assert '"-WindowTitle", \'"Gazebo Sim"\'' in wrapper
    assert 'Start-Process -FilePath "powershell.exe"' in wrapper
    assert "ShowWindowAsync" in guard
    assert "SetForegroundWindow" in guard
    assert 'StartsWith("[WARN:COPY MODE]"' in guard
    assert '"copy_mode_timeout"' in guard
    assert 'Found multiple WSLg windows' in guard
    assert '"-CloseWindowOnStop"' in wrapper
    assert "WM_CLOSE" in guard
    assert "PostMessage" in guard
    assert '"window_close_requested"' in guard
    assert "close_request_accepted" in guard


def test_wslg_shared_memory_preflight_is_idempotent_and_persistent():
    preflight = (ROOT / "scripts" / "prepare_wslg_runtime.sh").read_text(
        encoding="utf-8"
    )

    assert 'SHARED_MEMORY_DIR="/mnt/shared_memory"' in preflight
    assert 'mountpoint -q "${SHARED_MEMORY_DIR}"' in preflight
    assert 'mount -t tmpfs' in preflight
    assert "/etc/fstab" in preflight
    assert 'filesystem_type' in preflight
    assert 'refusing to replace it' in preflight
    assert 'rdp_allocate_shared_memory: Failed to open' in preflight
    assert 'wslg_restart_required' in preflight
    assert 'exit 10' in preflight
