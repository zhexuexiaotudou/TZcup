import importlib.util
import json
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import yaml
import pytest


SCRIPT = Path(__file__).with_name("capture_formal_first_map_early_recording_audit.py")
SPEC = importlib.util.spec_from_file_location("early_recording_audit", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _clock(value: int):
    return SimpleNamespace(clock=SimpleNamespace(sec=value // 1_000_000_000, nanosec=value % 1_000_000_000))


def _zero_twist():
    vector = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    return SimpleNamespace(twist=SimpleNamespace(linear=vector, angular=vector))


def _ready_tracker(monkeypatch):
    tracker = MODULE.Tracker("early", MODULE.EARLY_TYPES, "/session", "s" * 64, "token", "/bag")
    base = 1_000_000_000
    for offset, value in enumerate((100, 200, 300)):
        tracker.written(MODULE.CLOCK, _clock(value), base + offset)
    from sanitation_formal_campus_integration import lifecycle_health_protocol as protocol
    monkeypatch.setattr(protocol, 'producer_alive', lambda *args, **kwargs: True)
    business = {'status': 'waiting_for_fresh_mapping_inputs', 'ready': False}
    packet = dict(business, lifecycle_health={
        'version': 1, 'session': '/session', 'session_sha256': 's' * 64,
        'run_token': 'token', 'producer_identity': {'boot_id':'fixture', 'pid':123, 'process_starttime_ticks':1},
        'health_seq': 1, 'health_monotonic_ns': base + 10, 'eval_seq': 1,
        'eval_monotonic_ns': base + 10, 'business_sha256': protocol.digest(business),
        'validity_kind': 'evaluation_lease', 'business_valid_until_monotonic_ns': base + 15_000_000_000})
    for topic in MODULE.REQUIRED:
        message = SimpleNamespace(data=json.dumps(packet)) if topic == '/formal_mapping/lifecycle_status' else SimpleNamespace()
        tracker.written(topic, message, base + 10)
    tracker.written("/base_controller/cmd_vel", _zero_twist(), base + 10)
    tracker.written("/safety/relay_cycle_diagnostic_json", SimpleNamespace(), base + 10)
    tracker.written("/formal_vehicle/auxiliary/critical_safety_relay_diagnostic_json", SimpleNamespace(), base + 10)
    safety = {
        "state": "ENABLED", "safety_inputs_permit_actuators": True,
        "actuators_enabled": True, "managed_controllers_active": True,
        "active_reasons": "", "status_publish_count": 42,
    }
    tracker.written("/safety/status_json", SimpleNamespace(data=json.dumps(safety)), base + 10)
    return tracker, base + 11


def test_arm_requires_actual_writes_and_strict_clock_advances(monkeypatch):
    tracker = MODULE.Tracker("early", MODULE.EARLY_TYPES, "/session", "s" * 64, "token", "/bag")
    assert "clock did not strictly advance at least twice" in tracker.blockers(1)
    tracker, now = _ready_tracker(monkeypatch)
    assert tracker.blockers(now) == []
    assert tracker.clock_advances == 2
    assert tracker.pre_arm_status_count == 42
    assert tracker.pre_arm_status_messages_received == tracker.counts["/safety/status_json"] == 1


def test_non_safe_bootstrap_is_audit_only_but_active_is_a_permanent_arm_blocker(monkeypatch):
    tracker, now = _ready_tracker(monkeypatch)
    inhibited = {"state": "INHIBITED", "active_reasons": "manual_estop", "status_publish_count": 43}
    tracker.written("/safety/status_json", SimpleNamespace(data=json.dumps(inhibited)), now)
    assert tracker.pre_arm_unsafe_count == 1
    assert tracker.blockers(now) == []
    active = {"state": "ACTIVE", "active_reasons": "fault", "status_publish_count": 44}
    tracker.written("/safety/status_json", SimpleNamespace(data=json.dumps(active)), now + 1)
    assert tracker.pre_arm_active_seen is True
    assert "pre-arm ACTIVE safety evidence observed" in tracker.blockers(now + 1)


def test_clock_stall_and_rollback_fail_closed_without_duplicate_clock_refresh(monkeypatch):
    tracker, now = _ready_tracker(monkeypatch)
    tracker.written(MODULE.CLOCK, _clock(300), now)
    assert "clock evidence is stalled" in tracker.blockers(now + MODULE.FRESH_NS + 1)
    tracker.written(MODULE.CLOCK, _clock(299), now + 1)
    assert tracker.clock_rollback is True
    assert "clock rollback observed" in tracker.blockers(now + 1)


def test_closed_bag_summary_requires_exact_counts_and_mcap_framing(tmp_path):
    bag = tmp_path / "early_recording_audit"
    bag.mkdir()
    counts = {topic: 1 for topic in MODULE.EARLY_TYPES}
    (bag / "audit_0.mcap").write_bytes(b"\x89MCAP0\r\npayload\x89MCAP0\r\n")
    metadata = {"rosbag2_bagfile_information": {
        "relative_file_paths": ["audit_0.mcap"],
        "message_count": sum(counts.values()),
        "topics_with_message_count": [
            {"topic_metadata": {"name": topic}, "message_count": count}
            for topic, count in counts.items()
        ],
    }}
    (bag / "metadata.yaml").write_text(yaml.safe_dump(metadata), encoding="utf-8")
    assert MODULE.closed_summary(bag, counts)["metadata_message_counts"] == counts


def _install_fake_ros(monkeypatch, failure: str | None, *, deliver_message: bool,
                      deliveries: list[tuple[int, object]] | None = None):
    """Exercise main() error paths without a ROS graph or a real writer."""
    state = {"delivered": False, "writers": [], "writes": [], "serialized": []}

    class FakeClockMessage:
        def __init__(self):
            self.clock = SimpleNamespace(sec=1, nanosec=0)

    class FakeNode:
        def __init__(self, *_):
            self.callbacks = []
        def create_subscription(self, _type, _topic, callback, _qos):
            self.callbacks.append(callback)
            return object()
        def destroy_node(self):
            return None
        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1))
        def get_logger(self):
            return SimpleNamespace(error=lambda *_: None)

    class FakeWriter:
        def __init__(self):
            self.closed = False
            state["writers"].append(self)
        def open(self, *_):
            return None
        def create_topic(self, *_):
            return None
        def write(self, *_):
            if failure == "write":
                raise RuntimeError("write failure")
            state["writes"].append(_)
        def close(self):
            self.closed = True
            if failure == "close":
                raise RuntimeError("close failure")

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: None
    rclpy.shutdown = lambda: None
    pending = list(deliveries or [])
    def spin_once(node, **_):
        if pending:
            index, message = pending.pop(0)
            node.callbacks[index](message)
        elif deliver_message and not state["delivered"]:
            state["delivered"] = True
            node.callbacks[0](FakeClockMessage())
    rclpy.spin_once = spin_once
    node_mod = types.ModuleType("rclpy.node"); node_mod.Node = FakeNode
    qos_mod = types.ModuleType("rclpy.qos"); qos_mod.qos_profile_sensor_data = object()
    serialization_mod = types.ModuleType("rclpy.serialization")
    def serialize(message):
        state["serialized"].append(message)
        if failure == "serialize":
            raise RuntimeError("serialize failure")
        return b"cdr"
    serialization_mod.serialize_message = serialize
    bag = types.ModuleType("rosbag2_py")
    bag.SequentialWriter = FakeWriter
    bag.StorageOptions = lambda **kwargs: kwargs
    bag.ConverterOptions = lambda *_: object()
    bag.TopicMetadata = lambda **kwargs: kwargs
    geometry = types.ModuleType("geometry_msgs.msg"); geometry.TwistStamped = type("TwistStamped", (), {})
    nav = types.ModuleType("nav_msgs.msg"); nav.Odometry = type("Odometry", (), {})
    clock = types.ModuleType("rosgraph_msgs.msg"); clock.Clock = FakeClockMessage
    sensor = types.ModuleType("sensor_msgs.msg"); sensor.NavSatFix = type("NavSatFix", (), {})
    std = types.ModuleType("std_msgs.msg"); std.String = type("String", (), {})
    for name, module in {
        "rclpy": rclpy, "rclpy.node": node_mod, "rclpy.qos": qos_mod,
        "rclpy.serialization": serialization_mod, "rosbag2_py": bag,
        "geometry_msgs.msg": geometry, "nav_msgs.msg": nav,
        "rosgraph_msgs.msg": clock, "sensor_msgs.msg": sensor, "std_msgs.msg": std,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(MODULE, "boot_id", lambda: "fake-boot")
    monkeypatch.setattr(MODULE, "start_ticks", lambda _pid: 1)
    monkeypatch.setattr(MODULE.os, "getpgid", lambda _pid: 1, raising=False)
    return state


def test_writer_uses_receive_epoch_not_clock_or_header_stamp(monkeypatch, tmp_path):
    clock = _clock(987_654_321_000)
    payload = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=123, nanosec=456)),
        marker="payload must remain unchanged",
    )
    state = _install_fake_ros(
        monkeypatch, None, deliver_message=False,
        deliveries=[(0, clock), (1, payload)],
    )
    session = tmp_path / "session.json"; session.write_text("{}", encoding="utf-8")
    monkeypatch.setenv("FORMAL_ACCEPTANCE_SESSION", str(session))
    monkeypatch.setenv("FORMAL_RECORDING_RUN_TOKEN", "token")
    monkeypatch.setenv("FORMAL_OBSERVATION_DEADLINE_MONOTONIC_NS", str(MODULE.time.monotonic_ns() + 100_000_000))
    receive_epoch_ns = 1_777_777_777_123_456_789
    monkeypatch.setattr(MODULE.time, "time_ns", lambda: receive_epoch_ns)
    monkeypatch.setattr(MODULE, "closed_summary", lambda *_: {})
    assert MODULE.main(["--output", str(tmp_path), "--role", "early", "--timeout", "1"]) == 2
    assert [entry[2] for entry in state["writes"]] == [receive_epoch_ns, receive_epoch_ns]
    assert receive_epoch_ns != MODULE.clock_value(clock)
    assert receive_epoch_ns != 123_000_000_456
    assert state["serialized"] == [clock, payload]
    assert payload.header.stamp.sec == 123 and payload.header.stamp.nanosec == 456
    assert payload.marker == "payload must remain unchanged"


@pytest.mark.parametrize("failure", ["serialize", "write"])
def test_main_writer_or_serializer_failure_invalidates_without_counting(monkeypatch, tmp_path, failure):
    state = _install_fake_ros(monkeypatch, failure, deliver_message=True)
    session = tmp_path / "session.json"; session.write_text("{}", encoding="utf-8")
    monkeypatch.setenv("FORMAL_ACCEPTANCE_SESSION", str(session))
    monkeypatch.setenv("FORMAL_RECORDING_RUN_TOKEN", "token")
    monkeypatch.setenv("FORMAL_OBSERVATION_DEADLINE_MONOTONIC_NS", str(MODULE.time.monotonic_ns() + 1_000_000_000))
    assert MODULE.main(["--output", str(tmp_path), "--role", "early", "--timeout", "1"]) == 1
    invalid = json.loads((tmp_path / "formal_recording_invalid.json").read_text())
    closed = json.loads((tmp_path / "formal_recording_closed.json").read_text())
    assert invalid["writer_error"] == f"{failure} failure"
    assert closed["actual_writer_message_counts"][MODULE.CLOCK] == 0
    assert closed["close_rc"] == 1
    assert state["writers"][0].closed is True


def test_main_close_failure_invalidates_and_records_closed_receipt(monkeypatch, tmp_path):
    _install_fake_ros(monkeypatch, "close", deliver_message=False)
    session = tmp_path / "session.json"; session.write_text("{}", encoding="utf-8")
    monkeypatch.setenv("FORMAL_ACCEPTANCE_SESSION", str(session))
    monkeypatch.setenv("FORMAL_RECORDING_RUN_TOKEN", "token")
    monkeypatch.setenv("FORMAL_OBSERVATION_DEADLINE_MONOTONIC_NS", str(MODULE.time.monotonic_ns() + 100_000_000))
    assert MODULE.main(["--output", str(tmp_path), "--role", "early", "--timeout", "1"]) == 1
    assert "close failure" in json.loads((tmp_path / "formal_recording_invalid.json").read_text())["writer_error"]
    assert json.loads((tmp_path / "formal_recording_closed.json").read_text())["close_rc"] == 1


def test_valid_local_ready_rejects_negative_age(monkeypatch, tmp_path):
    tracker = MODULE.Tracker("early", MODULE.EARLY_TYPES, "/session", "s" * 64, "token", str(tmp_path / "early"))
    future = 100
    payload = {
        "status": "FORMAL_LOCALIZATION_RECORDING_READY", "ready": True,
        "formal_acceptance_session": "/session", "formal_acceptance_session_sha256": "s" * 64,
        "run_token": "token", "bag_dir": str(tmp_path / "mapping_localization_diagnostic"),
        "owner_identity": {"owner_token": "token:localization"}, "ready_monotonic_ns": future,
        "clock_ns": 1, "clock_advances": 2, "clock_rollback": False,
        "last_clock_advance_monotonic_ns": future,
        "writer_message_counts": {topic: 1 for topic in MODULE.REQUIRED},
        "last_message_monotonic_ns_by_topic": {topic: future for topic in MODULE.REQUIRED},
    }
    path = tmp_path / "formal_localization_ready.json"; path.write_text(json.dumps(payload), encoding="utf-8")
    monkeypatch.setattr(MODULE, "owner_live", lambda _: True)
    monkeypatch.setattr(MODULE.time, "monotonic_ns", lambda: future - 1)
    assert MODULE.valid_local_ready(path, tracker) is None


def test_main_early_role_refuses_a_localization_invalid_receipt(monkeypatch, tmp_path):
    _install_fake_ros(monkeypatch, None, deliver_message=False)
    session = tmp_path / "session.json"; session.write_text("{}", encoding="utf-8")
    (tmp_path / "formal_localization_recording_invalid.json").write_text("{}", encoding="utf-8")
    monkeypatch.setenv("FORMAL_ACCEPTANCE_SESSION", str(session))
    monkeypatch.setenv("FORMAL_RECORDING_RUN_TOKEN", "token")
    monkeypatch.setenv("FORMAL_OBSERVATION_DEADLINE_MONOTONIC_NS", str(MODULE.time.monotonic_ns() + 1_000_000_000))
    assert MODULE.main(["--output", str(tmp_path), "--role", "early", "--timeout", "1"]) == 1
    assert (tmp_path / "formal_recording_invalid.json").is_file()


def test_startup_markers_sealed_before_callbacks_and_clock_rollback_rejected(monkeypatch, tmp_path):
    state = _install_fake_ros(monkeypatch, None, deliver_message=True)
    session = tmp_path / 'session.json'; session.write_text('{}')
    monkeypatch.setenv('FORMAL_ACCEPTANCE_SESSION', str(session))
    monkeypatch.setenv('FORMAL_RECORDING_RUN_TOKEN', 'token')
    monkeypatch.setenv('FORMAL_OBSERVATION_DEADLINE_MONOTONIC_NS', str(MODULE.time.monotonic_ns()+100_000_000))
    monkeypatch.setattr(MODULE.time, 'time_ns', lambda: 100 if state['delivered'] else 200)
    monkeypatch.setattr(MODULE, 'closed_summary', lambda *_: {})
    spin = sys.modules['rclpy'].spin_once
    def checked_spin(node, **kwargs):
        startup = json.loads((tmp_path/'formal_recording_timing_start.json').read_bytes())
        assert startup['acceptance_window_start_epoch_ns'] == 200
        assert (tmp_path/'formal_recording_writer_open.json').is_file()
        assert startup['writer_ready_sample']['monotonic_ns'] <= startup['subscription_created_sample']['monotonic_ns'] <= startup['spin_start_sample']['monotonic_ns']
        return spin(node, **kwargs)
    monkeypatch.setattr(sys.modules['rclpy'], 'spin_once', checked_spin)
    assert MODULE.main(['--output',str(tmp_path),'--role','early','--timeout','1']) == 1
    assert state['writes'] == []
    invalid = json.loads((tmp_path/'formal_recording_invalid.json').read_bytes())
    assert invalid['writer_error'] == 'receive epoch precedes sealed writer-ready window'
    trace = json.loads((tmp_path/'formal_recording_timing_closed.json').read_bytes())
    assert trace['callbacks'][0]['receive_epoch_ns'] == 100
    assert trace['callbacks'][0]['write_completed'] is False
