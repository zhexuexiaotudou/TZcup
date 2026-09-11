from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import generate_demo_artifact_manifest as subject


IDENTITY = {"session_id": "session-a", "runtime_id": "runtime-a", "episode_id": "episode-a", "session_start_epoch_ns": 100}


def put_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value), encoding="utf-8")


def fixture_run(tmp_path: Path) -> tuple[Path, Path, Path, Path]:
    root = tmp_path / "run"; root.mkdir()
    raw = root / "raw.json"
    put_json(raw, {"artifact_kind": "single_live_episode_raw_collection", "run_identity": IDENTITY,
                   "timed_out": False,
                   "replay_metric_capture": {"complete": True, "capture_window": {"operator_start_ros_time_ns": 10, "task_complete_ros_time_ns": 20}},
                   "product": {"grasp_results": [{"verified_in_bin": True, "collector_received_epoch_ns": 12}],
                               "planner_status_samples": [{"returning_home": "true", "collector_received_epoch_ns": 18}]}})
    video = root / "video.mp4"; video.write_bytes(b"synthetic-video")
    session = root / "formal_session.json"
    put_json(session, {"report_id": "tzcup_formal_final_acceptance_session_v1", "started_epoch_ns": 100})
    observation = root / "video.mp4.json"
    put_json(observation, {"schema_version": "tzcup.a12.video_observation.v1", "status": "A12_VIDEO_OBSERVED", "run_root": str(root), "runtime_id": "runtime-a",
                           "formal_session": {"path": str(session), "sha256_at_capture_start": hashlib.sha256(session.read_bytes()).hexdigest()},
                           "video": {"path": str(video), "sha256": hashlib.sha256(video.read_bytes()).hexdigest(), "size_bytes": video.stat().st_size},
                           "window": {"operator_start_epoch_ns": 1000, "mission_complete_epoch_ns": 2000}})
    mcap = root / "bag"; mcap.mkdir(); (mcap / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8"); (mcap / "bag_0.mcap").write_bytes(b"mcap")
    return root, raw, observation, mcap


def fake_probe(path: Path) -> dict:
    return {"ffprobe_readable": True, "duration_seconds": 5.0}


def test_manifest_marks_unobserved_required_events_missing(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    root, raw, observation, mcap = fixture_run(tmp_path)
    monkeypatch.setattr(subject, "ffprobe", fake_probe)
    manifest, timeline = subject.build_manifest(root, raw, observation, mcap, [])
    assert manifest["delivery_status"] == "BLOCKED_MISSING_REQUIRED_EVIDENCE"
    assert manifest["missing_required_events"] == ["map_creation", "hard_restart", "obstacle_avoidance"]
    assert {row["event"] for row in timeline if row["status"] == "observed"} == {"cleaning", "grasp_drop", "return_home"}


def test_manifest_accepts_only_same_identity_explicit_events(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    root, raw, observation, mcap = fixture_run(tmp_path)
    monkeypatch.setattr(subject, "ffprobe", fake_probe)
    reports = []
    for index, event in enumerate(("map_creation", "hard_restart", "obstacle_avoidance")):
        report = root / f"{event}.json"; put_json(report, {"run_identity": IDENTITY, "event": event, "event_epoch_ns": 1001 + index, "status": "OBSERVED"}); reports.append(report)
    manifest, timeline = subject.build_manifest(root, raw, observation, mcap, reports)
    assert manifest["delivery_status"] == "READY_FOR_REVIEW"
    subject.publish(root / "delivery", manifest, timeline)
    assert (root / "delivery" / "checksums.sha256").is_file()
    assert (root / "delivery" / "timeline.csv").read_text(encoding="utf-8").count("observed") == 6


def test_manifest_rejects_video_from_another_runtime(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    root, raw, observation, mcap = fixture_run(tmp_path)
    monkeypatch.setattr(subject, "ffprobe", fake_probe)
    payload = json.loads(observation.read_text(encoding="utf-8")); payload["runtime_id"] = "other"; put_json(observation, payload)
    with pytest.raises(subject.ManifestError, match="runtime_id"):
        subject.build_manifest(root, raw, observation, mcap, [])


def test_manifest_rejects_incomplete_raw_capture(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    root, raw, observation, mcap = fixture_run(tmp_path)
    monkeypatch.setattr(subject, "ffprobe", fake_probe)
    payload = json.loads(raw.read_text(encoding="utf-8")); payload["replay_metric_capture"]["complete"] = False; put_json(raw, payload)
    with pytest.raises(subject.ManifestError, match="complete replay-metric"):
        subject.build_manifest(root, raw, observation, mcap, [])
