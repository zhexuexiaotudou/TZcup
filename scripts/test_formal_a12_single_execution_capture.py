from __future__ import annotations

import importlib.util
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest


SCRIPT = Path(__file__).with_name("formal_a12_single_execution_capture.py")
SPEC = importlib.util.spec_from_file_location("a12_capture", SCRIPT)
assert SPEC and SPEC.loader
capture = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = capture
SPEC.loader.exec_module(capture)


RGB_INFO = """Type: sensor_msgs/msg/Image
Publisher count: 1
Subscription count: 2
Node name: camera
QoS profile:
  Reliability: BEST_EFFORT
  Durability: VOLATILE
"""
BOOL_INFO = """Type: std_msgs/msg/Bool
Publisher count: 0
Subscription count: 1
Node name: gate
QoS profile:
  Reliability: RELIABLE
  Durability: TRANSIENT_LOCAL
"""

# This fixture is deliberately independent from the producer table.  A wrong
# fixed type in production must make metadata validation fail rather than
# regenerating a matching expectation from that same wrong constant.
RECORDER_TOPIC_FIXTURE = {
    "/brush_controller/commands": "std_msgs/msg/Float64MultiArray",
    "/localization/fused_odom": "nav_msgs/msg/Odometry",
    "/ground_truth/odom": "nav_msgs/msg/Odometry",
    "/perception/open_vocab/dosod_boxes": "vision_msgs/msg/Detection2DArray",
    "/perception/garbage/targets": "sanitation_perception_interfaces/msg/GarbageTargetArray",
    "/active_cleaning/planner_status": "diagnostic_msgs/msg/DiagnosticArray",
    "/active_cleaning/grasp_result": "std_msgs/msg/String",
    "/product_demo/operator_start": "std_msgs/msg/Bool",
    "/active_cleaning/mission_complete": "std_msgs/msg/Bool",
}


def test_runtime_topic_contract_accepts_observed_expected_qos() -> None:
    row = capture._topic_info_contract(RGB_INFO, capture.TOPIC_REQUIREMENTS[0])
    assert row["publisher_count"] == 1
    assert row["matching_qos"] == {"reliability": "BEST_EFFORT", "durability": "VOLATILE"}
    start = capture._topic_info_contract(BOOL_INFO, capture.TOPIC_REQUIREMENTS[1])
    assert start["type"] == "std_msgs/msg/Bool"


def test_runtime_topic_contract_rejects_wrong_type_qos_or_missing_rgb_publisher() -> None:
    with pytest.raises(capture.CaptureError, match="type mismatch"):
        capture._topic_info_contract(RGB_INFO.replace("sensor_msgs/msg/Image", "std_msgs/msg/Bool"), capture.TOPIC_REQUIREMENTS[0])
    with pytest.raises(capture.CaptureError, match="QoS mismatch"):
        capture._topic_info_contract(RGB_INFO.replace("BEST_EFFORT", "RELIABLE"), capture.TOPIC_REQUIREMENTS[0])
    with pytest.raises(capture.CaptureError, match="no live publisher"):
        capture._topic_info_contract(RGB_INFO.replace("Publisher count: 1", "Publisher count: 0"), capture.TOPIC_REQUIREMENTS[0])


def test_window_strictly_excludes_frames_outside_start_complete_and_counts_drops() -> None:
    state = capture.WindowState(queue_size=1)
    state.on_image(object(), 1)
    state.on_operator_start(True, 2)
    state.on_image(object(), 3)
    state.on_image(object(), 4)
    state.on_mission_complete(True, 5)
    state.on_image(object(), 6)
    queued = state.frames.get_nowait()
    assert queued.received_epoch_ns == 3
    assert state.ignored_before_start_count == 1
    assert state.dropped_frame_count == 1
    assert state.ignored_after_complete_count == 1
    assert state.accepted_frame_count == 0
    assert state.started_epoch_ns == 2 and state.completed_epoch_ns == 5


def test_timestamp_resampling_is_fixed_fps_and_does_not_treat_expected_downsampling_as_queue_loss() -> None:
    start = 1_000_000_000
    assert capture._resample_index(start, start, 15.0) == 0
    assert capture._resample_index(start, start + 1_000_000_000, 15.0) == 15
    with pytest.raises(capture.CaptureError, match="predates"):
        capture._resample_index(start, start - 1, 15.0)
    state = capture.WindowState(queue_size=2)
    state.on_operator_start(True, start)
    state.on_image(object(), start + 1)
    state.on_image(object(), start + 2)
    state.on_image(object(), start + 3)
    assert state.dropped_frame_count == 1
    assert state.resample_skipped_frame_count == 0


def _source_quality_args(tmp_path: Path):
    return capture.parse_args([
        "--run-root", str(tmp_path), "--output", "video.mp4", "--ready-file", "ready.json",
        "--session-status", str(tmp_path / "session.json"), "--runtime-id", "run-1",
    ])


def _quality_state(frame_count: int = 20, step_ns: int = 100_000_000, complete_lag_ns: int = 50_000_000):
    state = capture.WindowState(queue_size=max(1, frame_count))
    start = 1_000_000_000
    state.on_operator_start(True, start)
    for index in range(frame_count):
        stamp = start + 1 + index * step_ns
        state.accepted(capture.FrameItem(object(), stamp, stamp), 500_000_000)
    state.on_mission_complete(True, start + (frame_count - 1) * step_ns + complete_lag_ns)
    return state


def test_source_frame_quality_requires_real_rate_unique_stamps_and_fresh_edges(tmp_path: Path) -> None:
    args = _source_quality_args(tmp_path)
    quality = capture._validate_source_frame_quality(_quality_state(), args)
    assert quality["nominal_rate_hz"] == 30.0
    assert quality["accepted_unique_source_frames"] == 20
    assert quality["observed_rate_hz"] >= 10.0

    frozen = _quality_state(frame_count=1, complete_lag_ns=6 * 60 * 60 * 1_000_000_000)
    with pytest.raises(capture.CaptureError, match="not fresh|count/rate"):
        capture._validate_source_frame_quality(frozen, args)

    long_gap = capture.WindowState(queue_size=2)
    long_gap.on_operator_start(True, 1)
    long_gap.accepted(capture.FrameItem(object(), 1, 1), 500_000_000)
    with pytest.raises(capture.CaptureError, match="gap"):
        long_gap.accepted(capture.FrameItem(object(), 600_000_001, 600_000_001), 500_000_000)

    stale_terminal = _quality_state(complete_lag_ns=600_000_000)
    with pytest.raises(capture.CaptureError, match="mission_complete"):
        capture._validate_source_frame_quality(stale_terminal, args)

    repeated = capture.WindowState(queue_size=2)
    repeated.on_operator_start(True, 1)
    repeated.accepted(capture.FrameItem(object(), 10, 10), 500_000_000)
    with pytest.raises(capture.CaptureError, match="repeated or moved backwards"):
        repeated.accepted(capture.FrameItem(object(), 20, 10), 500_000_000)


def test_formal_front_d435_contract_requires_unique_fixed_topic_and_30hz(tmp_path: Path) -> None:
    import yaml

    contract = tmp_path / "pre_urdf_contract.yaml"

    def write(rows) -> None:
        contract.write_text(yaml.safe_dump({"sensor_contracts": rows}), encoding="utf-8")

    expected = {
        "id": "front_d435_depth",
        "topic": "/sensors/front_rgbd/depth/image_rect_raw",
        "update_rate_hz": 30.0,
    }
    write([expected])
    loaded = capture._load_formal_rgb_source_contract(contract)
    assert loaded["id"] == "front_d435_depth"
    assert loaded["topic"] == "/sensors/front_rgbd/depth/image_rect_raw"
    assert loaded["update_rate_hz"] == 30.0

    write([{**expected, "update_rate_hz": 15.0}])
    with pytest.raises(capture.CaptureError, match="update rate"):
        capture._load_formal_rgb_source_contract(contract)
    write([{**expected, "topic": "/camera/not-the-formal-topic"}])
    with pytest.raises(capture.CaptureError, match="topic differs"):
        capture._load_formal_rgb_source_contract(contract)
    write([expected, dict(expected)])
    with pytest.raises(capture.CaptureError, match="exactly one"):
        capture._load_formal_rgb_source_contract(contract)


def test_complete_before_start_fails_closed() -> None:
    with pytest.raises(capture.CaptureError, match="before operator_start"):
        capture.WindowState(queue_size=1).on_mission_complete(True, 1)


def test_safe_outputs_require_existing_non_link_root_relative_mp4_and_no_replace(tmp_path: Path) -> None:
    with pytest.raises(capture.CaptureError, match="relative"):
        capture._safe_outputs(tmp_path, tmp_path / "outside.mp4")
    with pytest.raises(capture.CaptureError, match=".mp4"):
        capture._safe_outputs(tmp_path, Path("capture.avi"))
    video, manifest = capture._safe_outputs(tmp_path, Path("capture.mp4"))
    assert video == tmp_path / "capture.mp4" and manifest.name == "capture.mp4.json"
    video.write_bytes(b"existing")
    with pytest.raises(capture.CaptureError, match="already exists"):
        capture._safe_outputs(tmp_path, Path("capture.mp4"))


def test_session_and_readiness_are_fresh_and_bound_to_a_running_formal_session(tmp_path: Path) -> None:
    session = tmp_path / "session.json"
    session.write_text(
        '{"report_id":"tzcup_formal_final_acceptance_session_v1","status":"FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING","started_epoch_ns":9}',
        encoding="utf-8",
    )
    binding = capture._session_binding(session)
    assert binding["started_epoch_ns"] == 9 and len(binding["sha256_at_capture_start"]) == 64
    ready = capture._safe_fresh_relative_file(tmp_path, Path("capture_ready.json"), "capture readiness")
    assert ready == tmp_path / "capture_ready.json"
    ready.write_text("retained", encoding="utf-8")
    with pytest.raises(capture.CaptureError, match="already exists"):
        capture._safe_fresh_relative_file(tmp_path, Path("capture_ready.json"), "capture readiness")
    session.write_text('{"report_id":"tzcup_formal_final_acceptance_session_v1","status":"FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE","started_epoch_ns":9}', encoding="utf-8")
    with pytest.raises(capture.CaptureError, match="RUNNING"):
        capture._session_binding(session)


def test_supervisor_interface_uses_fixed_same_pgid_children_and_finalization_inputs(tmp_path: Path) -> None:
    args = capture.parse_args([
        "--run-root", str(tmp_path), "--output", "video.mp4", "--ready-file", "ready.json",
        "--session-status", str(tmp_path / "session.json"), "--runtime-id", "run-1",
    ])
    command = capture._worker_command(args)
    assert command[2] == "--worker-video"
    assert command[command.index("--video-ready-file") + 1] == "a12_video_worker_ready.json"
    source = SCRIPT.read_text(encoding="utf-8")
    assert '"setsid", sys.executable' in source
    assert '"--node-name", "a12_trusted_gt_recorder"' in source
    assert "start_new_session=False" in source
    assert '"collector_raw": args.run_root.resolve() / args.collector_raw' in source
    assert '"source_metrics": args.run_root.resolve() / args.source_metrics' in source
    assert "A12_CAPTURE_SUPERVISOR_BLOCKED" in source
    assert "raw.rfind(\")\")" in source
    assert "_cleanup_private_pgid(os.getpgrp())" in source
    assert "refusing to clean a non-private A12 process group" in source
    assert '"argv": command' in source
    assert '"timeout_seconds": timeout_seconds' in source
    assert '"source_metrics_child"' in source


def test_recorder_metadata_requires_every_actual_topic_type_and_positive_count(tmp_path: Path) -> None:
    import yaml

    bag = tmp_path / "a12_execution.mcap"
    bag.mkdir()
    metadata = {
        "rosbag2_bagfile_information": {
            "topics_with_message_count": [
                {"topic_metadata": {"name": topic, "type": message_type}, "message_count": 1}
                for topic, message_type in RECORDER_TOPIC_FIXTURE.items()
            ]
        }
    }
    (bag / "metadata.yaml").write_text(yaml.safe_dump(metadata), encoding="utf-8")
    observation = capture._inspect_bag_metadata(bag)
    assert observation["topics"]["/ground_truth/odom"]["count"] == 1
    assert capture.EXPECTED_BAG_TOPICS == RECORDER_TOPIC_FIXTURE
    metadata["rosbag2_bagfile_information"]["topics_with_message_count"][0]["message_count"] = 0
    (bag / "metadata.yaml").write_text(yaml.safe_dump(metadata), encoding="utf-8")
    with pytest.raises(capture.CaptureError, match="topic/type/count"):
        capture._inspect_bag_metadata(bag)


def test_recorder_readiness_requires_subscription_names_and_types() -> None:
    node_info = "Subscribers:\n" + "".join(
        f"  {topic}: {message_type}\n" for topic, message_type in RECORDER_TOPIC_FIXTURE.items()
    ) + "Publishers:\n  /ignored: std_msgs/msg/String\n"
    assert capture._recorder_subscription_contract(node_info) == RECORDER_TOPIC_FIXTURE
    with pytest.raises(capture.CaptureError, match="Subscribers"):
        capture._recorder_subscription_contract("Publishers:\n  /x: std_msgs/msg/String\n")


def test_ready_storage_snapshot_rejects_metadata_only_and_reports_nonempty_mcap(tmp_path: Path) -> None:
    bag = tmp_path / "a12_execution.mcap"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("not a final metadata contract", encoding="utf-8")
    assert capture._bag_storage_snapshot(bag) == {}
    (bag / "a12_execution_0.mcap").write_bytes(b"mcap-sample-bytes")
    assert capture._bag_storage_snapshot(bag) == {"a12_execution_0.mcap": len(b"mcap-sample-bytes")}


@pytest.mark.skipif(os.name != "posix", reason="private process groups and /proc are Linux runtime contracts")
def test_private_pgid_cleanup_reaps_direct_child_and_ignores_dead_zombie() -> None:
    script = """
import importlib.util, os, subprocess, sys, time
spec = importlib.util.spec_from_file_location('a12_test_capture', sys.argv[1])
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
os.setsid()
child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
deadline = time.monotonic() + 3
while child.pid not in module._same_pgid_survivors(os.getpgrp()) and time.monotonic() < deadline:
    time.sleep(.02)
assert child.pid in module._same_pgid_survivors(os.getpgrp())
row = module._stop_child(child, 'fixture_direct_child', ['fixture-child'], 3.0)
assert row['returncode'] is not None
assert child.pid not in module._same_pgid_survivors(os.getpgrp())
assert module._cleanup_private_pgid(os.getpgrp()) == []
"""
    completed = subprocess.run([sys.executable, "-c", script, str(SCRIPT)], capture_output=True, text=True, timeout=15)
    assert completed.returncode == 0, completed.stderr


def test_encoder_probe_creates_and_ffprobe_reads_a_real_local_mp4(tmp_path: Path) -> None:
    ffprobe = shutil.which("ffprobe")
    if ffprobe is None:
        pytest.skip("ffprobe is not installed in this test runtime")
    observation = capture.verify_encoder(tmp_path, ffprobe)
    assert observation["frame_count"] >= 12
    assert observation["duration_seconds"] > 0
    assert not list(tmp_path.glob(".a12-encoder-probe-*.mp4"))


def test_video_window_default_covers_a_full_runner_timeout_and_drop_limit_is_bounded(tmp_path: Path) -> None:
    args = capture.parse_args([
        "--run-root", str(tmp_path), "--output", "video.mp4", "--ready-file", "ready.json",
        "--session-status", str(tmp_path / "session.json"), "--runtime-id", "run-1",
    ])
    assert args.window_timeout_seconds >= 21_600
    assert args.max_drop_ratio == 0.10
    with pytest.raises(SystemExit):
        capture.parse_args([
            "--run-root", str(tmp_path), "--output", "video.mp4", "--ready-file", "ready.json",
            "--session-status", str(tmp_path / "session.json"), "--runtime-id", "run-1",
            "--window-timeout-seconds", "21599",
        ])


def test_atomic_no_replace_preserves_existing_destination(tmp_path: Path) -> None:
    source = tmp_path / "source.mp4"
    destination = tmp_path / "published.mp4"
    source.write_bytes(b"first")
    capture._atomic_no_replace(source, destination)
    assert destination.read_bytes() == b"first"
    replacement = tmp_path / "replacement.mp4"
    replacement.write_bytes(b"replacement")
    with pytest.raises(capture.CaptureError, match="concurrently created"):
        capture._atomic_no_replace(replacement, destination)
    assert destination.read_bytes() == b"first"


def test_ffprobe_validation_rejects_incomplete_metadata(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    video = tmp_path / "video.mp4"
    video.write_bytes(b"not inspected by mocked probe")
    monkeypatch.setattr(capture.shutil, "which", lambda _: "ffprobe")

    class Completed:
        returncode = 0
        stderr = ""
        stdout = '{"streams":[{"width":320,"height":240,"nb_read_frames":"0"}],"format":{"duration":"0"}}'

    monkeypatch.setattr(capture.subprocess, "run", lambda *args, **kwargs: Completed())
    with pytest.raises(capture.CaptureError, match="non-positive"):
        capture._ffprobe_video(video, "ffprobe")


def test_script_never_invokes_gazebo_or_publishes_a_product_receipt() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    assert "create_publisher(" not in source
    assert "gz service" not in source.lower()
    assert "ros2 launch" not in source
    assert "AUTO15_PRODUCT_EXECUTION_EVIDENCE_SEALED" not in source
