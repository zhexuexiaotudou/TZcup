from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    world_file = LaunchConfiguration("world_file")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    urdf = os.path.join(
        get_package_share_directory("sanitation_vehicle_description"),
        "urdf", "sanitation_vehicle.urdf.xacro",
    )
    robot_description = xacro.process_file(
        urdf, mappings={"enable_training_gt": "true"}
    ).toxml()
    gz_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )
    return LaunchDescription([
        DeclareLaunchArgument("world_file"),
        DeclareLaunchArgument("spawn_x", default_value="-8.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={"gz_args": [" -r -s --headless-rendering ", world_file]}.items(),
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{"use_sim_time": True, "robot_description": robot_description}],
        ),
        Node(
            package="ros_gz_sim", executable="create",
            arguments=["-topic", "robot_description", "-name", "sanitation_vehicle",
                       "-x", spawn_x, "-y", spawn_y, "-z", "0.18", "-Y", spawn_yaw],
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                "/ground_truth/model_odom_raw@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/g2/semantic_gt/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/g2/semantic_gt/labels_map@sensor_msgs/msg/Image[gz.msgs.Image",
                "/g2/instance_gt/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/g2/instance_gt/labels_map@sensor_msgs/msg/Image[gz.msgs.Image",
            ],
            remappings=[
                ("/camera/camera_info", "/camera/color/camera_info"),
                ("/camera/image", "/camera/color/image_raw"),
                ("/camera/depth_image", "/camera/depth/image_rect_raw"),
                ("/g2/semantic_gt/labels_map", "/ground_truth/semantic/image"),
                ("/g2/instance_gt/labels_map", "/ground_truth/instance/image"),
            ],
        ),
    ])
