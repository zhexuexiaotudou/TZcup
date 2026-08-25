"""Spawn the formal vehicle in a dedicated Gazebo Harmonic validation world."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    gui = LaunchConfiguration("gui")
    start_controllers = LaunchConfiguration("start_controllers")
    use_sim_time = LaunchConfiguration("use_sim_time")
    physics_engine = LaunchConfiguration("physics_engine")
    bodywork_visible = LaunchConfiguration("bodywork_visible")
    world = LaunchConfiguration("world")
    model = PathJoinSubstitution(
        [FindPackageShare("sanitation_vehicle_description"), "urdf", "formal_competition_vehicle.urdf.xacro"]
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare("sanitation_vehicle_description"), "worlds", "formal_vehicle_validation.sdf"]
    )
    gz_launch = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
    robot_description = ParameterValue(
        Command([
            "xacro ", model,
            " use_sim:=true dry_load_mass_kg:=0.0 wastewater_load_mass_kg:=0.0 bodywork_visible:=",
            bodywork_visible,
        ]),
        value_type=str,
    )
    # sdformat rewrites package:// URIs to model:// URIs. Gazebo therefore
    # needs the parent of the installed package share directory on its resource
    # path so every vendored and project-generated mesh resolves offline.
    package_share_parent = str(Path(get_package_share_directory("sanitation_vehicle_description")).parent)
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = os.pathsep.join(
        item for item in (package_share_parent, existing_resource_path) if item
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "base_controller", "arm_controller",
            "cleaning_controller", "storage_controller", "brush_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
            "--service-call-timeout", "30",
            "--switch-timeout", "30",
            "--activate-as-group",
        ],
        output="screen",
        condition=IfCondition(start_controllers),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("start_controllers", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("bodywork_visible", default_value="true"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument(
                "physics_engine",
                default_value="gz-physics-bullet-featherstone-plugin",
                description="Physics engine; Bullet Featherstone is required for Robotiq mimic constraints.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={
                    "gz_args": [" -r -s ", world, " --physics-engine ", physics_engine]
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                condition=IfCondition(gui),
                launch_arguments={"gz_args": " -g"}.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                parameters=[{"robot_description": robot_description}],
                # base_footprint is the wheel-ground projection; use only a
                # 5 mm contact-settling clearance instead of lifting the car.
                arguments=["-param", "robot_description", "-name", "tzcup_formal_sanitation_vehicle", "-z", "0.005"],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/sensors/lidar_2d/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/sensors/lidar_2d/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                    "/sensors/lidar_3d/points@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/sensors/lidar_3d/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                    "/sensors/gnss/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
                    "/sensors/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    "/sensors/front_rgbd/depth/image_rect_raw/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/sensors/front_rgbd/depth/image_rect_raw/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/sensors/front_rgbd/depth/image_rect_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                    "/sensors/front_rgbd/depth/image_rect_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    "/sensors/wrist_rgbd/depth/image_rect_raw/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/sensors/wrist_rgbd/depth/image_rect_raw/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/sensors/wrist_rgbd/depth/image_rect_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                    "/sensors/wrist_rgbd/depth/image_rect_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    "/sensors/rear_left_fisheye/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/sensors/rear_left_fisheye/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    "/sensors/rear_right_fisheye/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/sensors/rear_right_fisheye/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    "/model/tzcup_formal_sanitation_vehicle/payload/dry_mass_kg@std_msgs/msg/Float64@gz.msgs.Double",
                    "/model/tzcup_formal_sanitation_vehicle/payload/dry_mass_kg/applied@std_msgs/msg/Float64[gz.msgs.Double",
                    "/model/tzcup_formal_sanitation_vehicle/payload/wastewater_mass_kg@std_msgs/msg/Float64@gz.msgs.Double",
                    "/model/tzcup_formal_sanitation_vehicle/payload/wastewater_mass_kg/applied@std_msgs/msg/Float64[gz.msgs.Double",
                    "/formal_visual/front_left@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/formal_visual/rear_right@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/formal_visual/top_cleaning@sensor_msgs/msg/Image[gz.msgs.Image",
                ],
                output="screen",
            ),
            TimerAction(period=6.0, actions=[controller_spawner]),
        ]
    )
