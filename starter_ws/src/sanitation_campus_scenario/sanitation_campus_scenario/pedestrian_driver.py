"""Environment-only ROS 2 driver for generated pedestrian schedules."""

from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import String

from .generator import GenerationError
from .motion import interpolate_loop, load_schedule


class PedestrianDriver(Node):
    def __init__(self) -> None:
        super().__init__("campus_pedestrian_environment_driver")
        self.declare_parameter("schedule_path", "")
        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("service_wait_timeout_sec", 15.0)
        schedule_path = str(self.get_parameter("schedule_path").value)
        if not schedule_path:
            raise GenerationError("schedule_path parameter is required")
        self.schedule = load_schedule(schedule_path)
        rate = float(self.get_parameter("update_rate_hz").value)
        if rate <= 0.0 or rate > 50.0:
            raise GenerationError("update_rate_hz must be in (0, 50]")
        self.service_wait_timeout_sec = float(
            self.get_parameter("service_wait_timeout_sec").value
        )
        if self.service_wait_timeout_sec <= 0.0:
            raise GenerationError("service_wait_timeout_sec must be positive")
        service = f"/world/{self.schedule['world_name']}/set_pose"
        self.client = self.create_client(SetEntityPose, service)
        self.status = self.create_publisher(
            String, "/scenario/environment/pedestrian_driver/status", 10
        )
        self.start_ns = self.get_clock().now().nanoseconds
        self.service_wait_started = time.monotonic()
        self.pending = []
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.publish_status("WAITING_FOR_SET_POSE")

    def publish_status(self, state: str, **details: object) -> None:
        message = String()
        message.data = json.dumps({"state": state, **details}, sort_keys=True)
        self.status.publish(message)

    def on_timer(self) -> None:
        finished = [future for future in self.pending if future.done()]
        for future in finished:
            try:
                response = future.result()
            except Exception as exc:  # ROS middleware errors are runtime-specific.
                self.publish_status("ERROR_SET_POSE_EXCEPTION", error=str(exc))
                self.timer.cancel()
                return
            if response is None or not response.success:
                self.publish_status("ERROR_SET_POSE_REJECTED")
                self.timer.cancel()
                return
        self.pending = [future for future in self.pending if not future.done()]
        if self.pending:
            return
        if not self.client.service_is_ready():
            if time.monotonic() - self.service_wait_started > self.service_wait_timeout_sec:
                self.publish_status("ERROR_SET_POSE_UNAVAILABLE")
                self.timer.cancel()
                return
            self.publish_status("WAITING_FOR_SET_POSE")
            return
        elapsed_s = (self.get_clock().now().nanoseconds - self.start_ns) / 1e9
        for pedestrian in self.schedule["pedestrians"]:
            x, y, yaw = interpolate_loop(pedestrian["waypoints"], elapsed_s)
            request = SetEntityPose.Request()
            request.entity.name = pedestrian["object_id"]
            request.entity.type = Entity.MODEL
            request.pose.position.x = x
            request.pose.position.y = y
            request.pose.position.z = 0.0
            request.pose.orientation.z = math.sin(yaw / 2.0)
            request.pose.orientation.w = math.cos(yaw / 2.0)
            self.pending.append(self.client.call_async(request))
        self.publish_status(
            "ACTIVE", pedestrian_count=len(self.schedule["pedestrians"])
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PedestrianDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
