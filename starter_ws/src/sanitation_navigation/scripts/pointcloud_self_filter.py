#!/usr/bin/env python3
# Copyright 2026 Sanitation Vehicle Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Transform a PointCloud2 into base_footprint and remove known robot points."""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Return the 3x3 rotation matrix for an xyzw quaternion."""
    norm = x * x + y * y + z * z + w * w
    if norm <= np.finfo(float).eps:
        return np.identity(3)
    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=np.float64,
    )


class PointCloudSelfFilter(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_self_filter')
        self.declare_parameter(
            'input_topic', '/verification_camera/depth/color/points'
        )
        self.declare_parameter(
            'output_topic',
            '/verification_camera/depth/color/points/navigation',
        )
        self.declare_parameter('output_frame', 'base_footprint')
        self.declare_parameter('mask_min_xyz_m', [-0.60, -0.43, -0.20])
        self.declare_parameter('mask_max_xyz_m', [0.72, 0.43, 0.75])
        self.declare_parameter('sampling_stride', 4)

        self._output_frame = str(self.get_parameter('output_frame').value)
        self._mask_min = np.asarray(
            self.get_parameter('mask_min_xyz_m').value, dtype=np.float64
        )
        self._mask_max = np.asarray(
            self.get_parameter('mask_max_xyz_m').value, dtype=np.float64
        )
        self._stride = max(
            1, int(self.get_parameter('sampling_stride').value)
        )
        if self._mask_min.shape != (3,) or self._mask_max.shape != (3,):
            raise ValueError('self mask bounds must contain exactly three axes')
        if np.any(self._mask_min >= self._mask_max):
            raise ValueError('self mask minimum must be below maximum')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('output_topic').value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('input_topic').value),
            self._callback,
            qos_profile_sensor_data,
        )

    def _callback(self, message: PointCloud2) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._output_frame,
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'pointcloud transform unavailable: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        points = np.asarray(
            point_cloud2.read_points_numpy(
                message,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
            ),
            dtype=np.float64,
        ).reshape(-1, 3)
        points = points[:: self._stride]
        if points.size:
            rotation = transform.transform.rotation
            translation = transform.transform.translation
            matrix = quaternion_matrix(
                rotation.x, rotation.y, rotation.z, rotation.w
            )
            points = points @ matrix.T
            points += np.array(
                [translation.x, translation.y, translation.z],
                dtype=np.float64,
            )
            finite = np.isfinite(points).all(axis=1)
            inside_self = np.logical_and(
                points >= self._mask_min, points <= self._mask_max
            ).all(axis=1)
            points = points[finite & ~inside_self]

        header = Header()
        header.stamp = message.header.stamp
        header.frame_id = self._output_frame
        self._publisher.publish(
            point_cloud2.create_cloud_xyz32(header, points.tolist())
        )


def main() -> None:
    rclpy.init()
    node = PointCloudSelfFilter()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        # Launch shutdown can invalidate the context before this finally block
        # runs; cleanup must stay quiet and idempotent.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
