"""Isolated ROS transport tests; mock permit input is not physical acceptance."""
import json
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from sanitation_product_demo_integration.simulation_operator_gate import node_class


@pytest.fixture
def harness(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "97")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    rclpy.init(args=[])
    observer = Node("isolated_operator_lease_probe")
    gate = node_class()()
    gate.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    executor = SingleThreadedExecutor()
    executor.add_node(observer)
    executor.add_node(gate)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class Harness:
        def __init__(self):
            self.gate = gate
            self.power, self.estop, self.armed, self.status = [], [], [], []
            self.instances = {"planner": "planner-1", "executor": "executor-1"}
            self.last = {"planner": 0.0, "executor": 0.0, "permit": 0.0}
            self.sequences = {"planner": 0, "executor": 0}
            self.metadata = {}
            self.start = observer.create_publisher(Bool, "/product_demo/operator_start", latched)
            self.planner = observer.create_publisher(String, "/active_cleaning/control_health", 1)
            self.action = observer.create_publisher(DiagnosticArray, "/active_cleaning/executor_status", 1)
            self.permit = observer.create_publisher(Bool, "/safety/actuators_enabled", latched)
            def record(target):
                def callback(message):
                    target.append(message.data)
                return callback
            for topic, target in (
                ("/formal_vehicle/simulation/command/main_power", self.power),
                ("/formal_vehicle/simulation/command/emergency_stop", self.estop),
                ("/product_demo/operator_armed", self.armed),
            ):
                observer.create_subscription(Bool, topic, record(target), 10)
            observer.create_subscription(String, "/product_demo/operator_gate_status",
                                         lambda msg: self.status.append(json.loads(msg.data)), 10)

        def publish_owner(self, role, *, sequence=None, source_time_ns=None, record=True):
            if sequence is None:
                self.sequences[role] += 1
                sequence = self.sequences[role]
            source_time_ns = time.monotonic_ns() if source_time_ns is None else source_time_ns
            if record:
                self.metadata[role] = {
                    "sequence": sequence,
                    "source_time_ns": source_time_ns,
                }
            if role == "planner":
                self.planner.publish(String(data=json.dumps({
                    "healthy": True,
                    "instance_id": self.instances[role],
                    "published_at_monotonic_ns": source_time_ns,
                    "sequence": sequence,
                })))
                return
            row = DiagnosticStatus(name="formal_active_cleaning_trajectory_executor", message="IDLE")
            row.values = [
                KeyValue(key="restart_required", value="false"),
                KeyValue(key="instance_id", value=self.instances[role]),
                KeyValue(key="published_at_monotonic_ns", value=str(source_time_ns)),
                KeyValue(key="sequence", value=str(sequence)),
            ]
            self.action.publish(DiagnosticArray(status=[row]))

        def replay_owner(self, role, failure):
            metadata = self.metadata[role]
            if failure == "sequence":
                self.publish_owner(role, sequence=metadata["sequence"],
                                   source_time_ns=metadata["source_time_ns"], record=False)
            elif failure == "timestamp":
                self.publish_owner(role, sequence=metadata["sequence"] + 1,
                                   source_time_ns=metadata["source_time_ns"] - 1, record=False)
            else:
                raise ValueError(f"unknown replay failure: {failure}")

        def step(self, seconds, *, planner=True, action=True, permit=True):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                now = time.monotonic()
                if planner and now - self.last["planner"] >= 0.5:
                    self.publish_owner("planner")
                    self.last["planner"] = now
                if action and now - self.last["executor"] >= 0.1:
                    self.publish_owner("executor")
                    self.last["executor"] = now
                if permit is not None and now - self.last["permit"] >= 0.05:
                    self.permit.publish(Bool(data=permit))
                    self.last["permit"] = now
                executor.spin_once(timeout_sec=0.01)

        def arm(self):
            self.start.publish(Bool(data=True))
            self.step(1.4)
            assert self.power[-1] is True and self.armed[-1] is True

        def restart_gate(self):
            executor.remove_node(self.gate)
            self.gate.destroy_node()
            self.gate = node_class()()
            self.gate.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
            executor.add_node(self.gate)

    result = Harness()
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            result.step(0.1)
            if all(pub.get_subscription_count() for pub in
                   (result.start, result.planner, result.action, result.permit)) and result.power:
                break
        assert result.power and result.start.get_subscription_count() > 0
        yield result
    finally:
        executor.shutdown()
        result.gate.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()


def test_arm_waits_for_physical_permit_and_stopped_clock_does_not_freeze_gate(harness):
    harness.step(0.2, permit=False)
    assert harness.power[-1] is False
    harness.start.publish(Bool(data=True))
    harness.step(1.3, permit=False)
    assert harness.power[-1] is True and harness.armed[-1] is False
    harness.step(0.2)
    assert harness.armed[-1] is True
    assert harness.status[-1]["control_owners_ready"] is True
    assert isinstance(harness.status[-1]["instance_id"], str)
    assert harness.status[-1]["sequence"] > 0
    assert harness.status[-1]["published_at_unix_ns"] <= time.time_ns()
    assert harness.gate.get_clock().now().nanoseconds == 0
    harness.step(0.8, action=False)
    assert harness.power[-1] is False and harness.estop[-1] is True
    harness.start.publish(Bool(data=True))
    harness.step(0.3)
    assert harness.power[-1] is False  # a repeated high does not rearm
    harness.start.publish(Bool(data=False))
    harness.step(0.1)
    harness.arm()


@pytest.mark.parametrize(("permit", "wait"), [(False, 0.2), (None, 0.7)])
def test_lost_or_stale_physical_permit_reasserts_safe_commands_and_requires_a_new_edge(
        harness, permit, wait):
    harness.arm()
    harness.step(wait, permit=permit)
    assert harness.power[-1] is False and harness.estop[-1] is True
    assert harness.armed[-1] is False
    assert "physical_safety_permit_lost" in harness.status[-1]["reason"]
    harness.step(0.2, permit=True)
    harness.start.publish(Bool(data=True))
    harness.step(0.3)
    assert harness.power[-1] is False and harness.estop[-1] is True
    assert harness.armed[-1] is False  # A repeated high cannot restore motion.


def test_replayed_owner_sequence_before_start_cannot_make_owners_ready_or_arm(harness):
    harness.step(0.6)
    harness.replay_owner("planner", "sequence")
    harness.step(0.2, planner=False, action=False)
    assert harness.status[-1]["control_owners_ready"] is False
    harness.start.publish(Bool(data=True))
    harness.step(0.2, planner=False, action=False)
    assert harness.power[-1] is False and harness.estop[-1] is True
    assert harness.armed[-1] is False


def test_armed_replayed_owner_timestamp_stops_and_fresh_owner_does_not_auto_rearm(harness):
    harness.arm()
    harness.replay_owner("executor", "timestamp")
    harness.step(0.2, planner=False, action=False)
    assert harness.power[-1] is False and harness.estop[-1] is True
    assert harness.armed[-1] is False
    assert "executor_source_timestamp_regressed" in harness.status[-1]["reason"]
    harness.step(0.7)
    assert harness.power[-1] is False and harness.estop[-1] is True
    assert harness.armed[-1] is False


@pytest.mark.parametrize("all_traffic_lost", [False, True])
def test_independent_watchdog_stops_on_control_owner_loss(harness, all_traffic_lost):
    harness.arm()
    harness.step(1.8, planner=False, action=not all_traffic_lost,
                 permit=None if all_traffic_lost else True)
    assert harness.power[-1] is False and harness.estop[-1] is True
    assert harness.armed[-1] is False


def test_restarted_gate_does_not_replay_retained_operator_start(harness):
    harness.arm()
    harness.restart_gate()
    harness.step(1.6)
    assert harness.power[-1] is False and harness.armed[-1] is False


@pytest.mark.parametrize("role", ["planner", "executor"])
def test_process_replacement_before_heartbeat_timeout_stops_motion(harness, role):
    harness.arm()
    harness.instances[role] = "replacement-process"
    harness.last[role] = 0.0
    harness.step(0.15)
    assert harness.power[-1] is False and harness.armed[-1] is False
    assert "instance_changed" in harness.status[-1]["reason"]
