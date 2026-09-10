from pathlib import Path


SCRIPT = Path(__file__).with_name("run_visual_demo.sh")


def test_rosbag_is_not_in_generic_cleanup_and_has_its_own_pid():
    text = SCRIPT.read_text(encoding="utf-8")
    recorder = text.index("setsid ros2 bag record --storage mcap")
    assignment = text.index('rosbag_pid="$!"', recorder)
    next_section = text.index("effective_video_mode=", recorder)

    assert assignment < next_section
    assert 'pids+=("$!")' not in text[recorder:next_section]


def test_recorder_finalization_precedes_generic_runtime_cleanup():
    text = SCRIPT.read_text(encoding="utf-8")
    coverage_terminal = text.index("if [[ \"${RECORD_MCAP}\" -eq 1 ]]; then", text.index("coverage_process_exit_code.txt"))
    finalizer = text.index("finalize_rosbag || true", coverage_terminal)
    generic_cleanup = text.index("stop_all", finalizer)

    assert finalizer < generic_cleanup
    assert "ROSBAG_SIGINT_TIMEOUT_SEC" in text
    assert "ROSBAG_SIGTERM_TIMEOUT_SEC" in text
    assert "ROSBAG_SEAL_TIMEOUT_SEC" in text
    assert 'kill -INT -- "-${rosbag_pid}"' in text
    assert 'kill -TERM -- "-${rosbag_pid}"' in text
    assert 'kill -KILL -- "-${rosbag_pid}"' in text
    assert "KILL_AFTER_TIMEOUT" in text
    assert "PGID_MISMATCH" in text
    assert "--require-sealed" in text
