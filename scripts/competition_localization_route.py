#!/usr/bin/env python3
"""One public-map Nav2 route; ground truth is recorded only by the external bag."""
import argparse
import json
import signal
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Empty


LIFECYCLE_NODES = ("bt_navigator", "controller_server", "planner_server")
ACTIVE_STATE = State.PRIMARY_STATE_ACTIVE


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=120)
    parser.add_argument("--prepare-seconds", type=float, default=420)
    parser.add_argument("--goal-x", type=float, default=6)
    parser.add_argument("--estop-distance", type=float, default=0)
    args = parser.parse_args()
    assert args.estop_distance == 0

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("competition_localization_route")
    state = {"sim": None, "permit": False, "stop": False}
    events = []
    started = time.monotonic()
    first_sim = None
    motion_start = None
    readiness_error = None
    goal_future = None
    goal_handle = None
    result_future = None
    stage = 0
    results = []
    last_command = progress = -1e9
    lifecycle_clients = {
        name: node.create_client(GetState, f"/{name}/get_state")
        for name in LIFECYCLE_NODES
    }
    lifecycle_states = {name: None for name in LIFECYCLE_NODES}
    lifecycle_futures = {name: None for name in LIFECYCLE_NODES}
    lifecycle_future_started = {name: None for name in LIFECYCLE_NODES}
    next_lifecycle_poll = 0.0

    def log(kind, data):
        events.append(
            {
                "wall_s": time.monotonic(),
                "sim_s": state["sim"],
                "kind": kind,
                "data": data,
            }
        )

    def stop(*_):
        state["stop"] = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node.create_subscription(
        Clock,
        "/clock",
        lambda msg: state.update(
            sim=msg.clock.sec + msg.clock.nanosec * 1e-9
        ),
        qos_profile_sensor_data,
    )
    node.create_subscription(
        Bool,
        "/safety/actuators_enabled",
        lambda msg: state.update(permit=msg.data),
        10,
    )
    publishers = {
        key: node.create_publisher(topic_type, topic, 10)
        for key, topic_type, topic in (
            (
                "power",
                Bool,
                "/formal_vehicle/simulation/command/main_power",
            ),
            (
                "estop",
                Bool,
                "/formal_vehicle/simulation/command/emergency_stop",
            ),
            (
                "reset",
                Bool,
                "/formal_vehicle/simulation/command/emergency_stop_reset",
            ),
            ("heartbeat", Empty, "/safety/control_heartbeat"),
        )
    }
    zero = node.create_publisher(Twist, "/cmd_vel_gate", 10)
    action = ActionClient(node, NavigateToPose, "/navigate_to_pose")

    (args.output / "route_protocol.json").write_text(
        json.dumps(
            {
                "scope": "localization only",
                "public_map_goals_x_m": [args.goal_x, 0.0],
                "return_not_before_sim_s": 40.0,
                "observation_duration_sim_s": 60.0,
                "ground_truth_subscribed_by_driver": False,
                "brush_commands": False,
                "estop_test": False,
                "requires_lifecycle_active_nodes": list(LIFECYCLE_NODES),
                "requires_action_server_ready": True,
                "requires_actuator_permit": True,
                "readiness_timeout_s": args.prepare_seconds,
            },
            indent=2,
        )
    )

    def poll_lifecycle(now):
        nonlocal next_lifecycle_poll
        if now >= next_lifecycle_poll:
            for name, client in lifecycle_clients.items():
                if lifecycle_futures[name] is not None:
                    continue
                if client.service_is_ready():
                    lifecycle_futures[name] = client.call_async(
                        GetState.Request()
                    )
                    lifecycle_future_started[name] = now
            next_lifecycle_poll = now + 0.25
        for name, future in lifecycle_futures.items():
            if future is None:
                continue
            if future.done():
                try:
                    lifecycle_states[name] = (
                        future.result().current_state.id
                    )
                except Exception as exc:
                    lifecycle_states[name] = None
                    log(
                        "lifecycle_state_error",
                        {"node": name, "error": str(exc)},
                    )
                lifecycle_futures[name] = None
                lifecycle_future_started[name] = None
            elif (
                lifecycle_future_started[name] is not None
                and now - lifecycle_future_started[name] > 2.0
            ):
                future.cancel()
                lifecycle_futures[name] = None
                lifecycle_future_started[name] = None
                lifecycle_states[name] = None

    def readiness_snapshot():
        return {
            "permit": state["permit"],
            "action_server_ready": action.server_is_ready(),
            "lifecycle_states": {
                name: lifecycle_states[name] for name in LIFECYCLE_NODES
            },
        }

    def ready():
        return (
            state["sim"] is not None
            and state["permit"]
            and action.server_is_ready()
            and all(
                lifecycle_states[name] == ACTIVE_STATE
                for name in LIFECYCLE_NODES
            )
        )

    try:
        while not state["stop"]:
            now = time.monotonic()
            if motion_start is None:
                if now - started >= args.prepare_seconds:
                    readiness_error = "nav2_lifecycle_action_or_permit_timeout"
                    log("readiness_timeout", readiness_snapshot())
                    break
            elif now - motion_start >= args.seconds:
                break

            rclpy.spin_once(node, timeout_sec=0.005)
            now = time.monotonic()
            if now - last_command >= 0.04:
                for key, value in (
                    ("power", True),
                    ("estop", False),
                    ("reset", True),
                ):
                    publishers[key].publish(Bool(data=value))
                publishers["heartbeat"].publish(Empty())
                if zero is not None:
                    zero.publish(Twist())
                last_command = now

            if motion_start is None:
                poll_lifecycle(now)
                if ready():
                    motion_start = now
                    log("readiness_ready", readiness_snapshot())
                if now - progress >= 20:
                    print(
                        json.dumps(
                            {
                                "wall_s": now - started,
                                "sim_s": state["sim"],
                                "stage": "waiting_for_nav2_lifecycle",
                                **readiness_snapshot(),
                            }
                        ),
                        flush=True,
                    )
                    progress = now
                continue

            if state["sim"] is not None and first_sim is None:
                first_sim = state["sim"]

            if (
                goal_future is None
                and state["sim"] is not None
                and (stage == 0 or (stage == 1 and state["sim"] >= 40.0))
            ):
                goal = NavigateToPose.Goal()
                goal.pose.header.frame_id = "map"
                goal.pose.header.stamp.sec = int(state["sim"])
                goal.pose.header.stamp.nanosec = int(
                    (state["sim"] - int(state["sim"])) * 1e9
                )
                goal.pose.pose.position.x = (
                    args.goal_x if stage == 0 else 0.0
                )
                goal.pose.pose.orientation.w = 1.0
                goal_future = action.send_goal_async(goal)
                log(
                    "goal_sent",
                    {
                        "stage": stage,
                        "x": goal.pose.pose.position.x,
                    },
                )
            if (
                goal_future is not None
                and goal_future.done()
                and goal_handle is None
            ):
                goal_handle = goal_future.result()
                log("goal_accepted", goal_handle.accepted)
                if not goal_handle.accepted:
                    break
                if zero is not None:
                    node.destroy_publisher(zero)
                    zero = None
                result_future = goal_handle.get_result_async()
            if result_future is not None and result_future.done():
                status = result_future.result().status
                results.append(status)
                log("goal_result", {"stage": stage, "status": status})
                stage += 1
                goal_future = result_future = goal_handle = None
                zero = node.create_publisher(Twist, "/cmd_vel_gate", 10)
                if status != 4:
                    break
            if (
                first_sim is not None
                and state["sim"] - first_sim >= 60
                and stage >= 2
            ):
                break
            if now - progress >= 20:
                print(
                    json.dumps(
                        {
                            "wall_s": now - started,
                            "sim_s": state["sim"],
                            "permit": state["permit"],
                            "stage": stage,
                            "results": results,
                        }
                    ),
                    flush=True,
                )
                progress = now
    finally:
        if goal_handle is not None and goal_handle.accepted:
            goal_handle.cancel_goal_async()
        if zero is None:
            zero = node.create_publisher(Twist, "/cmd_vel_gate", 10)
        for _ in range(10):
            zero.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.03)
        summary = {
            "nav_results": results,
            "passed": results == [4, 4],
            "first_sim_s": first_sim,
            "last_sim_s": state["sim"],
            "wall_duration_s": time.monotonic() - started,
            "ground_truth_used_for_control": False,
            "stopped_by_signal": state["stop"],
            "readiness_reached": motion_start is not None,
            "readiness_wall_s": (
                None if motion_start is None else motion_start - started
            ),
            "readiness_timeout_s": args.prepare_seconds,
            "readiness_error": readiness_error,
            "lifecycle_states": lifecycle_states,
            "action_server_ready": action.server_is_ready(),
            "permit": state["permit"],
        }
        (args.output / "route.json").write_text(
            json.dumps(summary, indent=2)
        )
        (args.output / "route_timeline.jsonl").write_text(
            "".join(json.dumps(row) + "\n" for row in events)
        )
        print(json.dumps(summary), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
