"""Exercise actual ROS callback bodies with deterministic futures and clocks.

These tests cover transport races, not ROS/Gazebo runtime acceptance.
"""
import ast
import json
from pathlib import Path
from types import SimpleNamespace as NS
import math

import pytest

from sanitation_active_cleaning import formal_policy_planner, formal_trajectory_executor


class Future:
    def __init__(self):
        self.callbacks = []
        self.value = None
        self.error = None

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def result(self):
        if self.error:
            raise self.error
        return self.value

    def resolve(self, value=None, error=None):
        self.value, self.error = value, error
        for callback in self.callbacks:
            callback(self)


class Handle:
    accepted = True

    def __init__(self):
        self.goal_id = NS(uuid=bytes([7] * 16))
        self.result_future = Future()
        self.cancel_future = Future()
        self.cancel_calls = 0
        self.result_calls = 0

    def get_result_async(self):
        self.result_calls += 1
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future


def load_class(module, name, clock):
    # main() nests the node to keep importing the algorithms ROS-independent.
    tree = ast.parse(Path(module.__file__).read_text(encoding="utf-8"))
    cls = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == name)
    namespace = dict(vars(module))
    namespace.update(
        Node=object, Bool=lambda **kw: NS(**kw), String=lambda **kw: NS(**kw),
        GoalStatus=NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6),
        time=NS(monotonic=lambda: clock[0], monotonic_ns=lambda: int(clock[0] * 1_000_000_000)),
    )
    exec(compile(ast.Module(body=[cls], type_ignores=[]), module.__file__, "exec", flags=0x1000000), namespace)
    return namespace[name]


def executor():
    clock = [0.0]
    cls = load_class(formal_trajectory_executor, "FormalTrajectoryExecutor", clock)
    node = cls.__new__(cls)
    node._goal_handle = None
    node._goal_pending = True
    node._cancel_requested = False
    node._cancel_sent = False
    node._goal_started = 0.0
    node._cancel_started = None
    node._fatal_reason = ""
    node._terminal_confirmed = False
    node._goal_response_timeout_sec = 5.0
    node._execution_timeout_sec = 20.0
    node._cancel_timeout_sec = 3.0
    node._safety_permitted = True
    node._last_safety_time = 0.0
    node._max_safety_age = 1000.0
    node._state, node._reason = "SUBMITTING", "test"
    node.states = []
    node._publish_status = lambda: node.states.append(node._state)
    node.get_logger = lambda: NS(error=lambda message: None)
    return node, clock


def accept(node, handle):
    future = Future()
    future.value = handle
    node._on_goal_response(future)


def finish(handle, status=5):
    handle.result_future.resolve(NS(status=status, result=NS(error_code=0)))


def test_cancel_before_acceptance_observes_terminal_and_drains_handle():
    node, _ = executor()
    node._on_cancel(NS(data=True))
    handle = Handle()
    accept(node, handle)
    assert handle.result_calls == handle.cancel_calls == 1
    assert node._goal_handle is handle
    handle.cancel_future.resolve(NS(return_code=0, goals_canceling=[NS(goal_id=handle.goal_id)]))
    assert not node._terminal_confirmed
    finish(handle)
    assert node._state == "CANCELED"
    assert node._terminal_confirmed and node._goal_handle is None


def test_lost_goal_response_latches_and_late_acceptance_is_canceled():
    node, clock = executor()
    clock[0] = 6.0
    node._watchdog()
    assert node._fatal_reason == "goal_response_timeout"
    handle = Handle()
    accept(node, handle)
    assert handle.cancel_calls == handle.result_calls == 1
    finish(handle, status=4)
    assert node._state == "FAILED"
    node._on_safety(NS(data=True))
    assert node._state == "FAILED"  # Fresh permit cannot erase timed-out task.


def test_repeated_safety_loss_does_not_flood_cancel_or_reset_deadline():
    node, clock = executor()
    handle = Handle()
    accept(node, handle)
    node._on_safety(NS(data=False))
    for stamp in (1.0, 2.0, 4.0):
        clock[0] = stamp
        node._watchdog()
    assert handle.cancel_calls == 1
    assert node._cancel_started == 0.0
    assert node._fatal_reason == "cancel_terminal_timeout"
    assert not node._terminal_confirmed and node._goal_handle is handle


@pytest.mark.parametrize("result", [
    NS(return_code=1, goals_canceling=[]),
    NS(return_code=0, goals_canceling=[NS(goal_id=NS(uuid=bytes(16)))]),
])
def test_cancel_ack_must_match_own_goal_and_not_release_ownership(result):
    node, _ = executor()
    handle = Handle()
    accept(node, handle)
    node._on_cancel(NS(data=True))
    handle.cancel_future.resolve(result)
    assert node._fatal_reason == "cancel_not_acknowledged_for_active_goal"
    assert node._goal_handle is handle
    finish(handle)
    assert node._terminal_confirmed and node._state == "FAILED"


def test_execution_deadline_cancels_without_claiming_terminal():
    node, clock = executor()
    handle = Handle()
    accept(node, handle)
    clock[0] = 21.0
    node._watchdog()
    assert node._fatal_reason == "execution_timeout"
    assert handle.cancel_calls == 1 and not node._terminal_confirmed


def test_late_cancel_ack_cannot_poison_the_next_goal():
    node, _ = executor()
    old = Handle()
    accept(node, old)
    node._on_cancel(NS(data=True))
    finish(old)
    current = Handle()
    current.goal_id = NS(uuid=bytes([8] * 16))
    node._goal_pending = True
    node._cancel_sent = False
    node._terminal_confirmed = False
    accept(node, current)
    old.cancel_future.resolve(NS(return_code=0, goals_canceling=[NS(goal_id=old.goal_id)]))
    assert node._fatal_reason == ""
    assert node._goal_handle is current and node._state == "EXECUTING"


def test_result_transport_error_retains_ownership_and_cancels():
    node, _ = executor()
    handle = Handle()
    accept(node, handle)
    handle.result_future.resolve(error=RuntimeError("transport failed"))
    assert node._goal_handle is handle and not node._terminal_confirmed
    assert handle.cancel_calls == 1


def test_busy_duplicate_does_not_publish_false_rejected_terminal():
    node, _ = executor()
    handle = Handle()
    accept(node, handle)
    node._on_path(NS())
    assert node._state == "EXECUTING"
    assert node._goal_handle is handle


def test_success_survives_next_permit_heartbeat():
    node, _ = executor()
    handle = Handle()
    accept(node, handle)
    finish(handle, status=4)
    node._on_safety(NS(data=True))
    assert node._state == "SUCCEEDED"


def planner():
    clock = [10.0]
    cls = load_class(formal_policy_planner, "FormalPolicyPlanner", clock)
    node = cls.__new__(cls)
    node._fatal_reason = ""
    node._mission_complete = False
    node._busy = True
    node._pending_grasp = None
    node._pending_since = 9.0
    node._executor_status_time = 9.5
    node._executor_status_timeout_sec = 5.0
    node._grasp_result_timeout_sec = 900.0
    node._cancel_sent = False
    node._cleaning_requested = True
    node._request_stamp = "2:3"
    node._executor_seen_active = True
    node._maximum_age = 1.5
    node._odom_time = None
    node._stationary_since = None
    node._last_odom_xy = None
    node._task_distance = 0.0
    node.cleaning, node.cancels = [], []
    node._cleaning_publisher = NS(publish=lambda msg: node.cleaning.append(msg.data))
    node._cancel_publisher = NS(publish=lambda msg: node.cancels.append(msg.data))
    node._publish_status = lambda *args: None
    node._inputs_fresh = lambda: False
    return node, clock


def status(state, *, request="2:3", terminal="true", restart="false"):
    return NS(status=[NS(name="formal_active_cleaning_trajectory_executor", message=state,
        values=[NS(key=k, value=v) for k, v in {
            "request_stamp": request, "terminal_confirmed": terminal,
            "restart_required": restart,
        }.items()])])


def test_busy_planner_stops_cleaning_and_cancels_on_stale_input():
    node, _ = planner()
    node._plan()
    node._plan()
    assert node.cleaning == [False, False]
    assert node.cancels == [True]
    assert node._busy  # Must still wait for real terminal evidence.


def armed_planner():
    node, clock = planner()
    node._operator_armed = False
    node._operator_armed_time = None
    node._ever_armed = False
    node._inputs_fresh = lambda: True
    node._map_pose = lambda: object()
    node._odom_time = clock[0]
    node.health, node.plans = [], []
    node._instance_id = "planner-test-instance"
    node._control_health_sequence = 0
    node._control_health_publisher = NS(publish=lambda msg: node.health.append(json.loads(msg.data)))
    node._plan = lambda: node.plans.append(True)
    node._busy = False
    return node, clock


def test_planner_announces_startup_health_but_waits_for_confirmed_arming():
    node, _ = armed_planner()
    node._plan_tick()
    assert node.health[-1]["healthy"] is True and node.health[-1]["sequence"] == 1 and not node.plans
    node._on_operator_armed(NS(data=True))
    node._plan_tick()
    assert node.plans == [True]


def test_lost_arm_confirmation_stops_and_latches_restart():
    node, clock = armed_planner()
    node._on_operator_armed(NS(data=True))
    node._busy = True
    clock[0] += 2.0
    node._plan_tick()
    assert not node.plans and node.health[-1]["healthy"] is False
    assert node.cancels == [True]
    assert "requires_restart" in node._fatal_reason


def test_planner_exception_publishes_unhealthy_before_propagating():
    node, _ = armed_planner()
    node._on_operator_armed(NS(data=True))
    def fail():
        raise RuntimeError("planner died")
    node._plan = fail
    with pytest.raises(RuntimeError):
        node._plan_tick()
    assert node.health[-1]["healthy"] is False


def test_stale_odometry_cannot_dispatch_a_path_or_keep_control_healthy():
    node, clock = armed_planner()
    node._on_operator_armed(NS(data=True))
    node._odom_time = clock[0] - 3.0
    node._plan_tick()
    assert not node.plans and node.health[-1]["healthy"] is False


def test_planner_health_has_instance_monotonic_source_time_and_strict_sequence():
    node, clock = armed_planner()
    node._plan_tick()
    clock[0] += 0.5
    node._plan_tick()
    assert [row["sequence"] for row in node.health] == [1, 2]
    assert all(row["instance_id"] == "planner-test-instance" for row in node.health)
    assert [row["published_at_monotonic_ns"] for row in node.health] == [10_000_000_000, 10_500_000_000]


def test_paused_or_rewound_clock_cannot_reuse_previous_request_identity():
    node, _ = planner()
    node._busy = False
    for sec, nano in ((2, 3), (1, 999)):
        assert not node._begin_path(NS(header=NS(stamp=NS(sec=sec, nanosec=nano))))
        assert node._request_stamp == "2:3"
        assert not node._cleaning_requested
    assert node._begin_path(NS(header=NS(stamp=NS(sec=2, nanosec=4))))
    assert node._request_stamp == "2:4"


def test_old_or_nonterminal_executor_status_cannot_release_planner():
    node, _ = planner()
    node._on_executor_status(status("SUCCEEDED", request="1:2"))
    node._on_executor_status(status("FAILED", terminal="false"))
    node._on_executor_status(status("IDLE"))
    assert node._busy
    node._on_executor_status(status("CANCELED"))
    assert not node._busy and not node._cleaning_requested


def test_executor_disappearance_latches_no_automatic_resubmit():
    node, clock = planner()
    clock[0] = 16.0
    node._plan()
    assert node._fatal_reason == "executor_status_timeout_requires_restart"
    node._inputs_fresh = lambda: True
    node._on_executor_status(status("SUCCEEDED"))
    node._plan()
    assert node._state == "FAILED" and node.cleaning[-1] is False


def test_grasp_timeout_does_not_release_base_for_navigation():
    node, clock = planner()
    node._pending_grasp = "target-1"
    clock[0] = 910.0
    node._plan()
    assert node._fatal_reason == "grasp_result_timeout_requires_restart"
    assert node._busy and node._pending_grasp == "target-1"
    assert node.cleaning[-1] is False


def odom(speed):
    return NS(pose=NS(pose=NS(position=NS(x=0.0, y=0.0))),
              twist=NS(twist=NS(linear=NS(x=speed, y=0.0), angular=NS(z=0.0))))


def test_stationary_dwell_restarts_after_odom_gap_or_motion():
    node, clock = planner()
    node._on_odometry(odom(0.0))
    assert node._stationary_since == 10.0
    clock[0] = 11.0
    node._on_odometry(odom(0.0))
    assert node._stationary_since == 10.0
    clock[0] = 20.0
    node._on_odometry(odom(0.0))
    assert node._stationary_since == 20.0
    node._on_odometry(odom(0.1))
    assert node._stationary_since is None


def test_odometry_jump_fails_instead_of_silently_reducing_task_distance():
    node, _ = planner()
    node._on_odometry(odom(0.1))
    jumped = odom(0.1)
    jumped.pose.pose.position.x = 3.0
    node._on_odometry(jumped)
    assert node._fatal_reason == "odometry_discontinuity_requires_restart"
    assert node._busy and node.cancels == [True]


def test_equal_size_shifted_belief_is_rejected_and_invalidates_previous_input():
    node, _ = planner()
    node._map_frame = "map"
    node._belief_time = 10.0
    node._core = NS(public_map=NS(width=2, height=2, resolution=0.1, origin_x=0.0, origin_y=0.0))
    message = NS(header=NS(frame_id="map"), data=[0, 0, 0, 0],
        info=NS(width=2, height=2, resolution=0.1, origin=NS(
            position=NS(x=1.0, y=0.0), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))))
    node._on_belief(message)
    assert node._belief_time is None
    assert node._reason == "belief_geometry_or_values_mismatch"
    assert node.cancels == [True]
