#!/usr/bin/env python3
"""Publish ROS calibration matching the nominal Gazebo equisolid lens.

This is a simulation projection contract, not a calibration result for a
physical Arducam serial number.  SDFormat's ``equisolid_angle`` mapping is
``r = 2 f sin(theta / 2)``.  ROS ``CameraInfo`` represents that mapping with
the ``equidistant`` / Kannala-Brandt polynomial, whose coefficients below are
the Taylor expansion through theta**9.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo


WIDTH = 1920
HEIGHT = 1080
HORIZONTAL_FOV_RAD = math.radians(150.0)
# Gazebo scale_to_hfov makes r(theta=HFOV/2) equal to half the image width.
FOCAL_LENGTH_PX = (WIDTH / 2.0) / (
    2.0 * math.sin(HORIZONTAL_FOV_RAD / 4.0)
)
CX = WIDTH / 2.0
CY = HEIGHT / 2.0
# 2*sin(theta/2) / theta = 1 - theta^2/24 + theta^4/1920
#                               - theta^6/322560 + theta^8/92897280 + ...
DISTORTION_COEFFICIENTS = [
    -1.0 / 24.0,
    1.0 / 1920.0,
    -1.0 / 322560.0,
    1.0 / 92897280.0,
]


class FormalFisheyeCameraInfoPublisher(Node):
    """Publish the nominal Gazebo-to-ROS fisheye projection contract."""

    def __init__(self) -> None:
        super().__init__("formal_fisheye_camera_info_publisher")
        # Do not shadow rclpy.node.Node._publishers: Node owns that internal
        # list and destroy_node() iterates it during launch teardown.
        self._camera_info_publishers = {
            "rear_left_fisheye_optical_frame": self.create_publisher(
                CameraInfo,
                "/sensors/rear_left_fisheye/camera_info",
                qos_profile_sensor_data,
            ),
            "rear_right_fisheye_optical_frame": self.create_publisher(
                CameraInfo,
                "/sensors/rear_right_fisheye/camera_info",
                qos_profile_sensor_data,
            ),
        }
        self._timer = self.create_timer(0.1, self._publish)

    @staticmethod
    def _message(frame_id: str) -> CameraInfo:
        message = CameraInfo()
        message.header.frame_id = frame_id
        message.width = WIDTH
        message.height = HEIGHT
        message.distortion_model = "equidistant"
        message.d = DISTORTION_COEFFICIENTS.copy()
        message.k = [
            FOCAL_LENGTH_PX,
            0.0,
            CX,
            0.0,
            FOCAL_LENGTH_PX,
            CY,
            0.0,
            0.0,
            1.0,
        ]
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.p = [
            FOCAL_LENGTH_PX,
            0.0,
            CX,
            0.0,
            0.0,
            FOCAL_LENGTH_PX,
            CY,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return message

    def _publish(self) -> None:
        if not rclpy.ok():
            return
        stamp = self.get_clock().now().to_msg()
        for frame_id, publisher in self._camera_info_publishers.items():
            message = self._message(frame_id)
            message.header.stamp = stamp
            try:
                publisher.publish(message)
            except Exception:
                # The launch supervisor can invalidate the context between the
                # preflight check and publish during ordered teardown.  Ignore
                # only that shutdown race; a live-context publish error remains
                # a real runtime failure and is re-raised into the launch log.
                if rclpy.ok():
                    raise
                return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FormalFisheyeCameraInfoPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
