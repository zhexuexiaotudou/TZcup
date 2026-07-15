"""Preserve raw Gazebo measurements and publish covariance-corrected copies."""

import copy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def diagonal_covariance(size, diagonal):
    values = [0.0] * (size * size)
    for index, value in enumerate(diagonal):
        values[index * size + index] = float(value)
    return values


class MeasurementAdapter(Node):
    def __init__(self):
        super().__init__("measurement_adapter")
        self.declare_parameter("wheel_input_topic", "/odom/unfiltered")
        self.declare_parameter("wheel_output_topic", "/measurements/wheel_odom")
        self.declare_parameter("wheel_frame_id", "odom")
        self.declare_parameter("wheel_child_frame_id", "base_footprint")
        self.declare_parameter(
            "wheel_pose_diagonal",
            [0.0025, 0.0025, 1.0e6, 1.0e6, 1.0e6, 0.0076],
        )
        self.declare_parameter(
            "wheel_twist_diagonal",
            [0.0025, 0.0400, 1.0e6, 1.0e6, 1.0e6, 0.0076],
        )
        self.declare_parameter("imu_input_topic", "/imu/data")
        self.declare_parameter("imu_output_topic", "/measurements/imu")
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("imu_orientation_diagonal", [1.0e6, 1.0e6, 0.0004])
        self.declare_parameter(
            "imu_angular_velocity_diagonal", [1.0e6, 1.0e6, 0.0004]
        )
        self.declare_parameter(
            "imu_linear_acceleration_diagonal", [0.0400, 0.0400, 0.0900]
        )
        self.wheel_publisher = self.create_publisher(
            Odometry, str(self.get_parameter("wheel_output_topic").value), 50
        )
        self.imu_publisher = self.create_publisher(
            Imu, str(self.get_parameter("imu_output_topic").value), qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("wheel_input_topic").value),
            self.on_wheel,
            50,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_input_topic").value),
            self.on_imu,
            qos_profile_sensor_data,
        )

    def on_wheel(self, message):
        output = copy.deepcopy(message)
        output.header.frame_id = str(self.get_parameter("wheel_frame_id").value)
        output.child_frame_id = str(self.get_parameter("wheel_child_frame_id").value)
        output.pose.covariance = diagonal_covariance(
            6, self.get_parameter("wheel_pose_diagonal").value
        )
        output.twist.covariance = diagonal_covariance(
            6, self.get_parameter("wheel_twist_diagonal").value
        )
        self.wheel_publisher.publish(output)

    def on_imu(self, message):
        output = copy.deepcopy(message)
        output.header.frame_id = str(self.get_parameter("imu_frame_id").value)
        output.orientation_covariance = diagonal_covariance(
            3, self.get_parameter("imu_orientation_diagonal").value
        )
        output.angular_velocity_covariance = diagonal_covariance(
            3, self.get_parameter("imu_angular_velocity_diagonal").value
        )
        output.linear_acceleration_covariance = diagonal_covariance(
            3, self.get_parameter("imu_linear_acceleration_diagonal").value
        )
        self.imu_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = MeasurementAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
