#!/usr/bin/env python3
"""Real ROS transport probe using a test FollowPath server, never Gazebo.

Runs only in an explicitly isolated domain. No cmd_vel or real actuator topics
are published. It is NOT a formal vehicle/safety/mission acceptance producer.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import threading
import time


def run(root: Path) -> dict:
    import rclpy
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from nav2_msgs.action import FollowPath
    from nav_msgs.msg import Path as NavPath
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool
    from sanitation_active_cleaning import formal_trajectory_executor as module

    geometry = root / "geometry.yaml"
    geometry.write_text("frame_id: map\nouter_polygon: [[0,0],[5,0],[5,5],[0,5]]\nkeepout_polygons: []\n")
    rclpy.init(args=["--ros-args", "-p", f"mission_geometry:={geometry}",
        "-p", "use_sim_time:=true", "-p", "goal_response_timeout_sec:=5.0",
        "-p", "execution_timeout_sec:=10.0", "-p", "cancel_timeout_sec:=3.0",
        "-p", "path_topic:=/probe/trajectory", "-p", "cancel_topic:=/probe/cancel",
        "-p", "safety_permit_topic:=/probe/permit", "-p", "status_topic:=/probe/status",
        "-p", "follow_path_action:=/probe/follow_path"])
    server = Node("action_lifecycle_test_server", use_global_arguments=False)
    client = module.node_class()()
    group = ReentrantCallbackGroup()
    receiving = threading.Event()
    records = []
    mode = {"value": "cancel_before_acceptance"}

    def goal_callback(request):
        receiving.set()
        time.sleep(0.5)  # Allow cancellation to arrive before acceptance.
        return GoalResponse.ACCEPT

    def execute(handle):
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            if mode["value"] == "success":
                handle.succeed()
                return FollowPath.Result()
            if handle.is_cancel_requested:
                handle.canceled()
                return FollowPath.Result()
            time.sleep(0.02)
        handle.abort()
        return FollowPath.Result()

    action = ActionServer(server, FollowPath, "/probe/follow_path", execute_callback=execute,
        goal_callback=goal_callback, cancel_callback=lambda _: CancelResponse.ACCEPT,
        callback_group=group)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    permit = server.create_publisher(Bool, "/probe/permit", qos)
    cancel = server.create_publisher(Bool, "/probe/cancel", 10)
    paths = server.create_publisher(NavPath, "/probe/trajectory", 10)
    server.create_timer(0.05, lambda: permit.publish(Bool(data=True)))
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(client)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()

    def wait_for(predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.02)
        raise RuntimeError(f"timeout: state={client._state}, reason={client._reason}")

    def submit(index):
        message = NavPath()
        message.header.frame_id = "map"
        message.header.stamp.sec = index
        for x in (1.0, 1.25, 1.5):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y = x, 1.0
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        paths.publish(message)

    try:
        wait_for(lambda: client._safety_is_fresh_and_permitted()
                 and client._action_client.server_is_ready()
                 and paths.get_subscription_count() > 0
                 and cancel.get_subscription_count() > 0)
        submit(1)
        if not receiving.wait(5.0):
            raise RuntimeError("server never received first goal")
        cancel.publish(Bool(data=True))
        wait_for(lambda: client._state == "CANCELED" and client._terminal_confirmed)
        records.append({"case": "cancel_before_acceptance", "passed": True,
                        "terminal": client._state, "simulation_clock_ns": client.get_clock().now().nanoseconds})
        mode["value"] = "success"
        receiving.clear()
        submit(2)
        wait_for(lambda: client._state == "SUCCEEDED")
        time.sleep(0.2)
        if client._state != "SUCCEEDED":
            raise RuntimeError("permit heartbeat overwrote successful terminal")
        records.append({"case": "next_goal_after_confirmed_cancel", "passed": True})
        mode["value"] = "execution_timeout"
        receiving.clear()
        submit(3)
        wait_for(lambda: client._fatal_reason == "execution_timeout", timeout=15.0)
        wait_for(lambda: client._terminal_confirmed)
        if client._state != "FAILED":
            raise RuntimeError("deadline failure was promoted by late terminal")
        if client.get_clock().now().nanoseconds != 0:
            raise RuntimeError("test requires stopped simulation clock")
        records.append({"case": "wall_watchdog_with_stopped_sim_clock", "passed": True})
        return {"status": "ROS_ACTION_TRANSPORT_PROBE_PASSED", "formal_acceptance": False,
                "cases": records, "source_sha256": hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()}
    finally:
        client._on_cancel(Bool(data=True))
        time.sleep(0.2)
        executor.shutdown(timeout_sec=5.0)
        worker.join(timeout=5.0)
        client.destroy_node()
        action.destroy()
        server.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get("TZCUP_ISOLATED_ACTION_PROBE") != "1":
        parser.error("requires TZCUP_ISOLATED_ACTION_PROBE=1 and a dedicated ROS domain")
    if not os.environ.get("ROS_DOMAIN_ID", "").isdigit() or not 1 <= int(os.environ["ROS_DOMAIN_ID"]) <= 101:
        parser.error("requires explicit isolated ROS_DOMAIN_ID in 1..101")
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    try:
        report = run(root)
    except Exception as exc:
        report = {"status": "ROS_ACTION_TRANSPORT_PROBE_FAILED", "formal_acceptance": False,
                  "error": f"{type(exc).__name__}: {exc}"}
    (root / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "ROS_ACTION_TRANSPORT_PROBE_PASSED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
