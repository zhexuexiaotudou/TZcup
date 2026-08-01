from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_representative_frame_is_taken_from_the_completed_end_of_video():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")

    assert 'ffmpeg -nostdin -y -sseof -5 -i "${OUTPUT_DIR}/visual_demo.mp4"' in launcher
    assert '-frames:v 1 -update 1 "${OUTPUT_DIR}/visual_demo_frame.png"' in launcher
    assert 'ffmpeg -nostdin -y -ss 5 -i "${OUTPUT_DIR}/visual_demo.mp4"' not in launcher


def test_readiness_bypasses_stale_ros_daemon():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")

    assert "ros2 node list --no-daemon --spin-time 3" in launcher
    assert "ros2 topic list --no-daemon --spin-time 3" in launcher
    assert "ros2 service list --no-daemon --spin-time 3" in launcher
    assert "--include-hidden-services" in launcher
    for action in ("compute_coverage_path", "follow_path", "navigate_to_pose"):
        assert f"/{action}/_action/send_goal" in launcher
    assert "ros2 action list" not in launcher


def test_camera_follow_discovery_has_a_hard_timeout():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")

    assert "for _ in $(seq 1 10)" in launcher
    assert 'gz_topics="$(timeout 3 gz topic -l 2>/dev/null || true)"' in launcher
    assert 'gz_topics="$(gz topic -l 2>/dev/null || true)"' not in launcher


def test_emergency_stop_availability_publish_cannot_block_gui_supervision():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")

    assert "timeout 10 ros2 topic pub --once --wait-matching-subscriptions 0" in launcher
    assert "Unable to publish the bounded emergency-stop availability pulse." in launcher


def test_wslg_gui_is_launched_directly_for_native_plugin_backend():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")

    assert 'gui:=false random_seed:="${RANDOM_SEED}"' in launcher
    assert 'setsid gz sim -g --gui-config "${gui_config}"' in launcher
    assert '> "${OUTPUT_DIR}/gazebo_gui.log" 2>&1 &' in launcher
    assert 'gui:="${gui_value}"' not in launcher
    assert launcher.index('if [[ "${ready}" -ne 1 ]]') < launcher.index(
        'setsid gz sim -g --gui-config "${gui_config}"'
    )


def test_wslg_gui_is_health_checked_and_supervised_during_the_mission():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")

    assert 'gazebo_gui_pid="$!"' in launcher
    assert "'/sanitation_gazebo_mission_control'" in launcher
    assert 'kill -0 "${gazebo_gui_pid}"' in launcher
    assert 'gui_closed_during_mission=1' in launcher
    assert 'runtime_termination_status="OPERATOR_GUI_CLOSED"' in launcher
    assert '"${OUTPUT_DIR}/launcher_termination.json"' in launcher
    assert "pgrep -f 'gz sim.*-g'" not in launcher


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
