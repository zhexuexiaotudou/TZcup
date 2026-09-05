"""Place the formal vehicle at the episode start before task execution."""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose

from .contract import resolve_spawn_pose


class SpawnInitializer(Node):
    """Perform exactly one initialization-only Gazebo pose request, then exit."""

    def __init__(self) -> None:
        super().__init__("formal_spawn_initializer")
        self.declare_parameter("episode_manifest_path", "")
        self.declare_parameter("world_name", "campus_formal")
        self.declare_parameter("entity_name", "tzcup_formal_sanitation_vehicle")
        self.declare_parameter("spawn_x", float("nan"))
        self.declare_parameter("spawn_y", float("nan"))
        self.declare_parameter("spawn_yaw", float("nan"))
        self.declare_parameter("spawn_z", 0.005)
        self.declare_parameter("service_timeout_sec", 30.0)
        self.succeeded = False

    def execute(self) -> None:
        path = str(self.get_parameter("episode_manifest_path").value)
        overrides = []
        for name in ("spawn_x", "spawn_y", "spawn_yaw"):
            value = float(self.get_parameter(name).value)
            overrides.append(None if math.isnan(value) else value)
        x, y, yaw = resolve_spawn_pose(
            path,
            spawn_x=overrides[0],
            spawn_y=overrides[1],
            spawn_yaw=overrides[2],
        )
        world = str(self.get_parameter("world_name").value)
        client = self.create_client(SetEntityPose, f"/world/{world}/set_pose")
        timeout = float(self.get_parameter("service_timeout_sec").value)
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("Gazebo SetEntityPose service unavailable")
        deadline = time.monotonic() + timeout
        attempts = 0
        while rclpy.ok() and time.monotonic() < deadline:
            request = SetEntityPose.Request()
            request.entity.name = str(self.get_parameter("entity_name").value)
            request.pose = Pose()
            request.pose.position.x = x
            request.pose.position.y = y
            request.pose.position.z = float(self.get_parameter("spawn_z").value)
            request.pose.orientation.z = math.sin(yaw / 2.0)
            request.pose.orientation.w = math.cos(yaw / 2.0)
            attempts += 1
            future = client.call_async(request)
            attempt_deadline = min(deadline, time.monotonic() + 2.0)
            while (
                rclpy.ok()
                and not future.done()
                and time.monotonic() < attempt_deadline
            ):
                rclpy.spin_once(self, timeout_sec=0.05)
            if future.done() and future.result() is not None and future.result().success:
                self.succeeded = True
                break
            rclpy.spin_once(self, timeout_sec=0.2)
        if not self.succeeded:
            raise RuntimeError("initialization-only formal vehicle placement failed")
        self.get_logger().info(
            f"formal vehicle initialized at x={x:.3f}, y={y:.3f}, "
            f"yaw={yaw:.3f} after {attempts} attempt(s)"
        )


def main() -> None:
    rclpy.init()
    node = SpawnInitializer()
    try:
        node.execute()
    finally:
        node.destroy_node()
        rclpy.shutdown()
