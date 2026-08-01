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
