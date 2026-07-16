from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless_rendering")
    use_sim_time = LaunchConfiguration("use_sim_time")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    enable_command_timeout = LaunchConfiguration("enable_command_timeout")
    enable_ekf = LaunchConfiguration("enable_ekf")
    enable_measurement_adapter = LaunchConfiguration("enable_measurement_adapter")
    ekf_config = LaunchConfiguration("ekf_config")
    physical_wheel_radius = LaunchConfiguration("physical_wheel_radius")
    physical_track_width = LaunchConfiguration("physical_track_width")
    drive_wheel_radius = LaunchConfiguration("drive_wheel_radius")
    drive_wheel_separation = LaunchConfiguration("drive_wheel_separation")
    wheel_mu_longitudinal = LaunchConfiguration("wheel_mu_longitudinal")
    wheel_mu_lateral = LaunchConfiguration("wheel_mu_lateral")
    slip_compliance_longitudinal = LaunchConfiguration("slip_compliance_longitudinal")
    slip_compliance_lateral = LaunchConfiguration("slip_compliance_lateral")
    enable_wheel_slip = LaunchConfiguration("enable_wheel_slip")
    lidar_samples = LaunchConfiguration("lidar_samples")
    lidar_update_rate = LaunchConfiguration("lidar_update_rate")
    world_file = LaunchConfiguration("world_file")
    random_seed = LaunchConfiguration("random_seed")
    world_to_map_x = LaunchConfiguration("world_to_map_x")
    world_to_map_y = LaunchConfiguration("world_to_map_y")
    world_to_map_yaw = LaunchConfiguration("world_to_map_yaw")

    gz_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )
    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("sanitation_vehicle_description"),
            "urdf",
            "sanitation_vehicle.urdf.xacro",
        ]
    )
    world_path = PathJoinSubstitution(
        [FindPackageShare("sanitation_worlds"), "worlds", "sanitation_test_world.sdf"]
    )
    default_ekf_config = PathJoinSubstitution(
        [FindPackageShare("sanitation_bringup"), "config", "selected_ekf.yaml"]
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                urdf_path,
                " physical_wheel_radius:=", physical_wheel_radius,
                " physical_track_width:=", physical_track_width,
                " drive_wheel_radius:=", drive_wheel_radius,
                " drive_wheel_separation:=", drive_wheel_separation,
                " wheel_mu_longitudinal:=", wheel_mu_longitudinal,
                " wheel_mu_lateral:=", wheel_mu_lateral,
                " slip_compliance_longitudinal:=", slip_compliance_longitudinal,
                " slip_compliance_lateral:=", slip_compliance_lateral,
                " enable_wheel_slip:=", enable_wheel_slip,
                " lidar_samples:=", lidar_samples,
                " lidar_update_rate:=", lidar_update_rate,
            ]
        ),
        value_type=str,
    )
    sim_time_parameter = ParameterValue(use_sim_time, value_type=bool)
    headless_rendering = PythonExpression(
        ["'--headless-rendering' if '", headless, "'.lower() == 'true' else ''"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("headless_rendering", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("spawn_x", default_value="-8.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("enable_command_timeout", default_value="true"),
            DeclareLaunchArgument("enable_ekf", default_value="true"),
            DeclareLaunchArgument("enable_measurement_adapter", default_value="true"),
            DeclareLaunchArgument("ekf_config", default_value=default_ekf_config),
            DeclareLaunchArgument("physical_wheel_radius", default_value="0.14"),
            DeclareLaunchArgument("physical_track_width", default_value="0.80"),
            DeclareLaunchArgument("drive_wheel_radius", default_value="0.14"),
            DeclareLaunchArgument("drive_wheel_separation", default_value="0.80"),
            DeclareLaunchArgument("wheel_mu_longitudinal", default_value="1.0"),
            DeclareLaunchArgument("wheel_mu_lateral", default_value="1.0"),
            DeclareLaunchArgument("slip_compliance_longitudinal", default_value="0.0"),
            DeclareLaunchArgument("slip_compliance_lateral", default_value="0.0"),
            DeclareLaunchArgument("enable_wheel_slip", default_value="false"),
            DeclareLaunchArgument("lidar_samples", default_value="360"),
            DeclareLaunchArgument("lidar_update_rate", default_value="10"),
            DeclareLaunchArgument("world_file", default_value=world_path),
            DeclareLaunchArgument("random_seed", default_value="0"),
            DeclareLaunchArgument("world_to_map_x", default_value="8.0"),
            DeclareLaunchArgument("world_to_map_y", default_value="0.0"),
            DeclareLaunchArgument("world_to_map_yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={
                    "gz_args": [" -r -s --seed ", random_seed, " ", headless_rendering, " ", world_file]
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                condition=IfCondition(gui),
                launch_arguments={"gz_args": [" -g"]}.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": sim_time_parameter,
                        "robot_description": robot_description,
                    }
                ],
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-name",
                    "sanitation_vehicle",
                    "-x",
                    spawn_x,
                    "-y",
                    spawn_y,
                    "-z",
                    "0.18",
                    "-Y",
                    spawn_yaw,
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                output="screen",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                    "/odom/unfiltered@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/ground_truth/model_odom_raw@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                    "/world/sanitation_test_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                    "/world_overview/image@sensor_msgs/msg/Image[gz.msgs.Image",
                ],
                remappings=[
                    ("/camera/camera_info", "/camera/color/camera_info"),
                    ("/camera/image", "/camera/color/image_raw"),
                    ("/camera/depth_image", "/camera/depth/image_rect_raw"),
                    ("/camera/points", "/camera/depth/color/points"),
                    (
                        "/world/sanitation_test_world/dynamic_pose/info",
                        "/ground_truth/dynamic_pose",
                    ),
                ],
            ),
            Node(
                package="sanitation_tasks",
                executable="sanitation_ground_truth_adapter",
                name="ground_truth_adapter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": sim_time_parameter,
                        "world_to_map_x": ParameterValue(world_to_map_x, value_type=float),
                        "world_to_map_y": ParameterValue(world_to_map_y, value_type=float),
                        "world_to_map_yaw": ParameterValue(world_to_map_yaw, value_type=float),
                        "expected_source_frame": "world",
                        "expected_child_frame": "sanitation_vehicle/base_footprint",
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="lidar_scoped_frame_bridge",
                output="screen",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "laser",
                    "--child-frame-id",
                    "sanitation_vehicle/base_footprint/sanitation_gpu_lidar",
                ],
            ),
            Node(
                package="linorobot2_gazebo",
                executable="command_timeout",
                name="command_timeout",
                output="screen",
                condition=IfCondition(enable_command_timeout),
            ),
            Node(
                package="sanitation_tasks",
                executable="sanitation_measurement_adapter",
                name="measurement_adapter",
                output="screen",
                parameters=[{"use_sim_time": sim_time_parameter}],
                condition=IfCondition(enable_measurement_adapter),
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[{"use_sim_time": sim_time_parameter}, ekf_config],
                remappings=[("odometry/filtered", "/odom")],
                condition=IfCondition(enable_ekf),
            ),
        ]
    )
