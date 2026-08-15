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
    drive_model = LaunchConfiguration("drive_model")
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
    cleaning_width = LaunchConfiguration("cleaning_width")
    brush_center_y = LaunchConfiguration("brush_center_y")
    world_file = LaunchConfiguration("world_file")
    world_name = LaunchConfiguration("world_name")
    random_seed = LaunchConfiguration("random_seed")
    world_to_map_x = LaunchConfiguration("world_to_map_x")
    world_to_map_y = LaunchConfiguration("world_to_map_y")
    world_to_map_yaw = LaunchConfiguration("world_to_map_yaw")
    camera_profile = LaunchConfiguration("camera_profile")
    enable_training_gt = LaunchConfiguration("enable_training_gt")
    gui_config = LaunchConfiguration("gui_config")
    gui_config_arg = PythonExpression(
        ["'--gui-config ' + '", gui_config, "' if '", gui_config, "' else ''"]
    )
    engineering_camera = PythonExpression(
        [
            "'",
            camera_profile,
            "' in ('V4_engineering', 'V5_retracted', 'AUTO03_corner')",
        ]
    )

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
    ackermann_ekf_config = PathJoinSubstitution(
        [FindPackageShare("sanitation_bringup"), "config", "ekf_ackermann.yaml"]
    )
    selected_ekf_by_drive = PythonExpression(
        [
            "'",
            ackermann_ekf_config,
            "' if '",
            drive_model,
            "' == 'ackermann' else '",
            default_ekf_config,
            "'",
        ]
    )
    ackermann_condition = IfCondition(
        PythonExpression(["'", drive_model, "' == 'ackermann'"])
    )
    legacy_condition = IfCondition(
        PythonExpression(["'", drive_model, "' != 'ackermann'"])
    )
    brush_forward_x = PythonExpression(
        ["'0.68' if '", drive_model, "' == 'ackermann' else '0.58'"]
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                urdf_path,
                " drive_model:=", drive_model,
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
                " cleaning_width:=", cleaning_width,
                " brush_center_y:=", brush_center_y,
                " brush_forward_x:=", brush_forward_x,
                " enable_verification_camera:=", PythonExpression(
                    [
                        "'true' if '",
                        camera_profile,
                        "' in ('V4_engineering', 'V5_retracted', 'AUTO03_corner') else 'false'",
                    ]
                ),
                " verification_camera_x:=", PythonExpression(
                    [
                        "'0.32' if '",
                        camera_profile,
                        "' == 'AUTO03_corner' else ('0.36' if '",
                        camera_profile,
                        "' == 'V5_retracted' else ('0.67' if '",
                        camera_profile,
                        "' == 'V4_engineering' else '0.30'))",
                    ]
                ),
                " verification_camera_y:=", PythonExpression(
                    [
                        "'0.28' if '",
                        camera_profile,
                        "' == 'AUTO03_corner' else ('0.0' if '",
                        camera_profile,
                        "' == 'V5_retracted' else ('0.34' if '",
                        camera_profile,
                        "' == 'V4_engineering' else '0.0'))",
                    ]
                ),
                " verification_camera_z:=", PythonExpression(
                    [
                        "'0.66' if '",
                        camera_profile,
                        "' in ('V5_retracted', 'AUTO03_corner') else ('0.48' if '",
                        camera_profile,
                        "' == 'V4_engineering' else '0.70')",
                    ]
                ),
                " verification_camera_pitch_rad:=", PythonExpression(
                    [
                        "'0.6108652382' if '",
                        camera_profile,
                        "' == 'AUTO03_corner' else ('0.8726646260' if '",
                        camera_profile,
                        "' in ('V4_engineering', 'V5_retracted') else '0.7853981634')",
                    ]
                ),
                " verification_camera_yaw_rad:=", PythonExpression(
                    [
                        "'0.7853981634' if '",
                        camera_profile,
                        "' == 'AUTO03_corner' else '0.0'",
                    ]
                ),
                " enable_training_gt:=", enable_training_gt,
                " enable_self_mask_gt:=", enable_training_gt,
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
            DeclareLaunchArgument(
                "drive_model",
                default_value="ackermann",
                description=(
                    "ackermann (physical front steering + rear traction) or "
                    "skid_steer_legacy (explicit legacy regression only); "
                    "the product default is Ackermann"
                ),
            ),
            DeclareLaunchArgument("ekf_config", default_value=selected_ekf_by_drive),
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
            DeclareLaunchArgument("cleaning_width", default_value="1.32"),
            DeclareLaunchArgument("brush_center_y", default_value="0.52"),
            DeclareLaunchArgument("world_file", default_value=world_path),
            DeclareLaunchArgument("world_name", default_value="sanitation_test_world"),
            DeclareLaunchArgument("gui_config", default_value=""),
            DeclareLaunchArgument("random_seed", default_value="0"),
            DeclareLaunchArgument("world_to_map_x", default_value="8.0"),
            DeclareLaunchArgument("world_to_map_y", default_value="0.0"),
            DeclareLaunchArgument("world_to_map_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "camera_profile",
                default_value="production",
                description="production or opt-in engineering cameras; production remains unchanged",
            ),
            DeclareLaunchArgument(
                "enable_training_gt",
                default_value="false",
                description="evaluation-only semantic/instance labels; never enabled by production defaults",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={
                    "gz_args": [" -r -s --seed ", random_seed, " ", headless_rendering, " ", world_file]
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                condition=IfCondition(gui),
                launch_arguments={"gz_args": [" -g ", gui_config_arg]}.items(),
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
                name="ackermann_wheel_odom_bridge",
                output="screen",
                condition=ackermann_condition,
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                    "/wheel/odom_raw@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/ground_truth/model_odom_raw@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                    ["/world/", world_name, "/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
                    "/world_overview/image@sensor_msgs/msg/Image[gz.msgs.Image",
                ],
                remappings=[
                    ("/camera/camera_info", "/camera/color/camera_info"),
                    ("/camera/image", "/camera/color/image_raw"),
                    ("/camera/depth_image", "/camera/depth/image_rect_raw"),
                    ("/camera/points", "/camera/depth/color/points"),
                    (
                        ["/world/", world_name, "/dynamic_pose/info"],
                        "/ground_truth/dynamic_pose",
                    ),
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="legacy_wheel_odom_bridge",
                output="screen",
                condition=legacy_condition,
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
                    ["/world/", world_name, "/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
                    "/world_overview/image@sensor_msgs/msg/Image[gz.msgs.Image",
                ],
                remappings=[
                    ("/camera/camera_info", "/camera/color/camera_info"),
                    ("/camera/image", "/camera/color/image_raw"),
                    ("/camera/depth_image", "/camera/depth/image_rect_raw"),
                    ("/camera/points", "/camera/depth/color/points"),
                    (
                        ["/world/", world_name, "/dynamic_pose/info"],
                        "/ground_truth/dynamic_pose",
                    ),
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="auto03_evaluation_only_gt_bridge",
                output="screen",
                condition=IfCondition(enable_training_gt),
                arguments=[
                    "/g2/verification_semantic_gt/labels_map@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/g2/verification_instance_gt/labels_map@sensor_msgs/msg/Image[gz.msgs.Image",
                ],
                remappings=[
                    (
                        "/g2/verification_semantic_gt/labels_map",
                        "/ground_truth/verification_semantic/image",
                    ),
                    (
                        "/g2/verification_instance_gt/labels_map",
                        "/ground_truth/verification_instance/image",
                    ),
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="engineering_verification_camera_bridge",
                output="screen",
                condition=IfCondition(engineering_camera),
                arguments=[
                    "/verification_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    "/verification_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/verification_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/verification_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                ],
                remappings=[
                    ("/verification_camera/camera_info", "/verification_camera/color/camera_info"),
                    ("/verification_camera/image", "/verification_camera/color/image_raw"),
                    ("/verification_camera/depth_image", "/verification_camera/depth/image_rect_raw"),
                    ("/verification_camera/points", "/verification_camera/depth/color/points"),
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
                parameters=[
                    {
                        "use_sim_time": sim_time_parameter,
                        "wheel_input_topic": PythonExpression(
                            [
                                "'/wheel/odom_raw' if '",
                                drive_model,
                                "' == 'ackermann' else '/odom/unfiltered'",
                            ]
                        ),
                    }
                ],
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
