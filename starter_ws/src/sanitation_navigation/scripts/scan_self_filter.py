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
"""Remove laser returns caused by the opt-in V4 camera body itself."""

from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanSelfFilter(Node):
    def __init__(self) -> None:
        super().__init__('scan_self_filter')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan/navigation')
        self.declare_parameter('laser_origin_x_m', 0.12)
        self.declare_parameter('mask_min_x_m', 0.6047791986798621)
        self.declare_parameter('mask_max_x_m', 0.735220801320138)
        self.declare_parameter('mask_min_y_m', 0.24000000000000002)
        self.declare_parameter('mask_max_y_m', 0.44000000000000006)
        self.declare_parameter('replace_infinite_ranges_with_max', False)
        self.declare_parameter('maximum_range_margin_m', 0.01)
        self._laser_x = float(self.get_parameter('laser_origin_x_m').value)
        self._bounds = tuple(
            float(self.get_parameter(name).value)
            for name in (
                'mask_min_x_m',
                'mask_max_x_m',
                'mask_min_y_m',
                'mask_max_y_m',
            )
        )
        self._replace_infinite = bool(
            self.get_parameter('replace_infinite_ranges_with_max').value
        )
        self._maximum_range_margin = float(
            self.get_parameter('maximum_range_margin_m').value
        )
        output = str(self.get_parameter('output_topic').value)
        self._publisher = self.create_publisher(
            LaserScan, output, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('input_topic').value),
            self._callback,
            qos_profile_sensor_data,
        )

    def _callback(self, message: LaserScan) -> None:
        filtered = LaserScan()
        filtered.header = message.header
        filtered.angle_min = message.angle_min
        filtered.angle_max = message.angle_max
        filtered.angle_increment = message.angle_increment
        filtered.time_increment = message.time_increment
        filtered.scan_time = message.scan_time
        filtered.range_min = message.range_min
        filtered.range_max = message.range_max
        filtered.ranges = list(message.ranges)
        filtered.intensities = list(message.intensities)
        min_x, max_x, min_y, max_y = self._bounds
        for index, distance in enumerate(filtered.ranges):
            if not math.isfinite(distance):
                if self._replace_infinite:
                    filtered.ranges[index] = max(
                        float(message.range_min),
                        float(message.range_max) - self._maximum_range_margin,
                    )
                continue
            angle = message.angle_min + index * message.angle_increment
            x = self._laser_x + distance * math.cos(angle)
            y = distance * math.sin(angle)
            if min_x <= x <= max_x and min_y <= y <= max_y:
                filtered.ranges[index] = math.inf
        self._publisher.publish(filtered)


def main() -> None:
    rclpy.init()
    node = ScanSelfFilter()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        # The launch context can already be invalid when the executor unwinds.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
