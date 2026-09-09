from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


SCRIPT = Path(__file__).with_name("collect_s100p_g1_g3_read_only.py")
SPEC = importlib.util.spec_from_file_location("s100p_g1_g3_read_only", SCRIPT)
assert SPEC and SPEC.loader
SUBJECT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SUBJECT)


def _observed(topics: list[str], _: float) -> dict[str, object]:
    return {
        "collector_node": "read_only",
        "use_sim_time": False,
        "topic_graph": {topic: ["sensor_msgs/msg/Image"] for topic in topics},
        "topics": {
            topic: {"status": "OBSERVED", "messages_received": 2, "header_stamp_monotonic": True, "frame_ids": ["camera_link"], "observed_rate_hz": 10.0}
            for topic in topics
        },
    }


def test_collect_is_not_an_acceptance_and_observes_requested_topics(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    setup = tmp_path / "setup.bash"
    overlay = tmp_path / "overlay.bash"
    setup.write_text("setup", encoding="utf-8")
    overlay.write_text("overlay", encoding="utf-8")
    monkeypatch.setattr(SUBJECT, "collect_devices", lambda: {"camera": {"status": "PRESENT"}, "can": {"status": "ABSENT"}, "tty": {"status": "PRESENT"}, "i2c": {"status": "PRESENT"}, "spi": {"status": "PRESENT"}, "network_interfaces": {"status": "PRESENT"}})
    report = SUBJECT.collect(tros_setup=setup, overlay_setup=overlay, topics=["/camera/color/image_raw"], duration_sec=1.0, required_devices=["camera"], ros_observer=_observed)
    assert report["status"] == "READ_ONLY_EVIDENCE_COLLECTED_NOT_ACCEPTED"
    assert report["acceptance"] == {"g1_accepted": False, "g2_accepted": False, "g3_accepted": False, "ready_for_g4": False}
    assert report["ros"]["topics"]["/camera/color/image_raw"]["observed_rate_hz"] == 10.0


def test_collect_blocks_missing_overlay_required_device_and_bad_topic(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    setup = tmp_path / "setup.bash"
    setup.write_text("setup", encoding="utf-8")
    monkeypatch.setattr(SUBJECT, "collect_devices", lambda: {"camera": {"status": "ABSENT"}, "can": {"status": "ABSENT"}, "tty": {"status": "PRESENT"}, "i2c": {"status": "PRESENT"}, "spi": {"status": "PRESENT"}, "network_interfaces": {"status": "PRESENT"}})
    report = SUBJECT.collect(tros_setup=setup, overlay_setup=tmp_path / "missing.bash", topics=["/camera/color/image_raw"], duration_sec=1.0, required_devices=["camera"], ros_observer=_observed)
    assert report["status"] == "BLOCKED"
    assert "overlay_setup_absent" in report["blockers"]
    assert "required_device_category_absent:camera" in report["blockers"]
    with pytest.raises(SUBJECT.ReadOnlyEvidenceError, match="unsafe_or_invalid_topic"):
        SUBJECT.collect(tros_setup=setup, overlay_setup=setup, topics=["/gazebo/camera"], duration_sec=1.0, required_devices=[], ros_observer=_observed)


def test_collect_fails_closed_without_a_requested_g3_topic(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    setup = tmp_path / "setup.bash"
    setup.write_text("setup", encoding="utf-8")
    monkeypatch.setattr(SUBJECT, "collect_devices", lambda: {"camera": {"status": "PRESENT"}, "can": {"status": "PRESENT"}, "tty": {"status": "PRESENT"}, "i2c": {"status": "PRESENT"}, "spi": {"status": "PRESENT"}, "network_interfaces": {"status": "PRESENT"}})
    report = SUBJECT.collect(tros_setup=setup, overlay_setup=setup, topics=[], duration_sec=1.0, required_devices=[], ros_observer=_observed)
    assert report["status"] == "BLOCKED"
    assert report["blockers"] == ["no_topics_requested_g3_observation_incomplete"]


def test_fresh_output_rejects_existing_or_symlinked_paths(tmp_path: Path) -> None:
    existing = tmp_path / "existing.json"
    existing.write_text("old", encoding="utf-8")
    with pytest.raises(SUBJECT.ReadOnlyEvidenceError, match="fresh"):
        SUBJECT._safe_fresh_output(existing.absolute())
    link_parent = tmp_path / "link-parent"
    try:
        link_parent.symlink_to(tmp_path, target_is_directory=True)
    except OSError as exc:
        pytest.skip(f"symbolic links unavailable on this test host: {exc.winerror}")
    with pytest.raises(SUBJECT.ReadOnlyEvidenceError, match="symlink"):
        SUBJECT._safe_fresh_output((link_parent / "new.json").absolute())


def test_atomic_output_is_new_json(tmp_path: Path) -> None:
    output = (tmp_path / "fresh.json").absolute()
    SUBJECT._atomic_write_new(output, {"status": "BLOCKED"})
    assert json.loads(output.read_text(encoding="utf-8")) == {"status": "BLOCKED"}
    with pytest.raises(SUBJECT.ReadOnlyEvidenceError, match="fresh"):
        SUBJECT._atomic_write_new(output, {"status": "BLOCKED"})


def test_socketcan_is_enumerated_from_sysfs_metadata(tmp_path: Path) -> None:
    net = tmp_path / "net"
    (net / "can0").mkdir(parents=True)
    (net / "can0" / "type").write_text("280\n", encoding="ascii")
    (net / "eth0").mkdir()
    (net / "eth0" / "type").write_text("1\n", encoding="ascii")
    observed = SUBJECT._network_interfaces(net)
    assert observed["names"] == ["can0", "eth0"]
    assert observed["socketcan_interfaces"] == ["can0"]
