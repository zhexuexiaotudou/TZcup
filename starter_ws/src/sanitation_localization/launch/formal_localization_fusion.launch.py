"""Start the formal wheel/IMU/GNSS/lidar-map localization fusion chain."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _validate_stabilizer_contract(context) -> list:
    stabilizer = context.perform_substitution(
        LaunchConfiguration("map_odom_stabilizer")
    ).strip()
    start_global_fusion = context.perform_substitution(
        LaunchConfiguration("start_global_fusion")
    ).strip()
    if stabilizer not in {"true", "false"}:
        raise RuntimeError("map_odom_stabilizer must be true or false")
    if start_global_fusion not in {"true", "false"}:
        raise RuntimeError("start_global_fusion must be true or false")
    if stabilizer == "true" and start_global_fusion != "true":
        raise RuntimeError(
            "map_odom_stabilizer requires start_global_fusion:=true"
        )
    if stabilizer == "true":
        tau = float(
            context.perform_substitution(
                LaunchConfiguration("map_odom_stabilizer_tau_sec")
            )
        )
        max_dt = float(
            context.perform_substitution(
                LaunchConfiguration("map_odom_stabilizer_max_dt_sec")
            )
        )
        max_gap = float(
            context.perform_substitution(
                LaunchConfiguration("map_odom_stabilizer_max_gap_sec")
            )
        )
        if not 0.75 <= tau <= 3.0:
            raise RuntimeError("stabilizer tau_sec must be in [0.75, 3.0]")
        if not 0.02 <= max_dt <= 0.2:
            raise RuntimeError("stabilizer max_dt_sec must be in [0.02, 0.2]")
        if not 0.1 <= max_gap <= 1.0:
            raise RuntimeError("stabilizer max_gap_sec must be in [0.1, 1.0]")
    return []


def generate_launch_description() -> LaunchDescription:
    parameters = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_local_fusion = LaunchConfiguration("start_local_fusion")
    start_navsat_transform = LaunchConfiguration("start_navsat_transform")
    start_global_fusion = LaunchConfiguration("start_global_fusion")
    map_odom_stabilizer = LaunchConfiguration("map_odom_stabilizer")
    map_odom_stabilizer_tau_sec = LaunchConfiguration(
        "map_odom_stabilizer_tau_sec"
    )
    map_odom_stabilizer_max_dt_sec = LaunchConfiguration(
        "map_odom_stabilizer_max_dt_sec"
    )
    map_odom_stabilizer_max_gap_sec = LaunchConfiguration(
        "map_odom_stabilizer_max_gap_sec"
    )
    raw_map_odom_topic = LaunchConfiguration("raw_map_odom_topic")
    map_odom_stabilizer_status_topic = LaunchConfiguration(
        "map_odom_stabilizer_status_topic"
    )
    navsat_odometry_input = LaunchConfiguration("navsat_odometry_input")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_local_fusion", default_value="true"),
            DeclareLaunchArgument("start_navsat_transform", default_value="true"),
            DeclareLaunchArgument("start_global_fusion", default_value="true"),
            DeclareLaunchArgument("map_odom_stabilizer", default_value="false"),
            DeclareLaunchArgument(
                "map_odom_stabilizer_tau_sec", default_value="1.5"
            ),
            DeclareLaunchArgument(
                "map_odom_stabilizer_max_dt_sec", default_value="0.1"
            ),
            DeclareLaunchArgument(
                "map_odom_stabilizer_max_gap_sec", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "raw_map_odom_topic",
                default_value="/localization/raw_map_odom",
            ),
            DeclareLaunchArgument(
                "map_odom_stabilizer_status_topic",
                default_value="/localization/map_odom_stabilizer/status",
            ),
            DeclareLaunchArgument(
                "navsat_odometry_input",
                default_value="/localization/fused_odom",
                description=(
                    "navsat_transform derives its measurement/output frame from "
                    "this Odometry source."
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("sanitation_localization"),
                        "config",
                        "formal_fusion.yaml",
                    ]
                ),
            ),
            OpaqueFunction(function=_validate_stabilizer_contract),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="local_ekf",
                condition=IfCondition(start_local_fusion),
                parameters=[parameters, {"use_sim_time": use_sim_time}],
                remappings=[("odometry/filtered", "/odom")],
                output="screen",
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                condition=IfCondition(start_navsat_transform),
                parameters=[parameters, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("imu", "/imu/data"),
                    ("gps/fix", "/gnss/fix"),
                    ("odometry/filtered", navsat_odometry_input),
                    ("odometry/gps", "/odometry/gps"),
                ],
                output="screen",
            ),
            GroupAction(
                condition=IfCondition(start_global_fusion),
                actions=[
                    Node(
                        package="robot_localization",
                        executable="ekf_node",
                        name="global_ekf",
                        condition=UnlessCondition(map_odom_stabilizer),
                        parameters=[parameters, {"use_sim_time": use_sim_time}],
                        remappings=[
                            ("odometry/filtered", "/localization/fused_odom"),
                        ],
                        output="screen",
                    ),
                    Node(
                        package="robot_localization",
                        executable="ekf_node",
                        name="global_ekf",
                        condition=IfCondition(map_odom_stabilizer),
                        parameters=[parameters, {"use_sim_time": use_sim_time}],
                        remappings=[
                            ("odometry/filtered", "/localization/fused_odom"),
                            ("/tf", raw_map_odom_topic),
                        ],
                        output="screen",
                    ),
                ],
            ),
            Node(
                package="sanitation_localization",
                executable="map_odom_stabilizer",
                name="map_odom_stabilizer",
                condition=IfCondition(map_odom_stabilizer),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_tf_topic": raw_map_odom_topic,
                        "status_topic": map_odom_stabilizer_status_topic,
                        "tau_sec": map_odom_stabilizer_tau_sec,
                        "max_filter_dt_sec": map_odom_stabilizer_max_dt_sec,
                        "max_gap_sec": map_odom_stabilizer_max_gap_sec,
                    }
                ],
                output="screen",
            ),
        ]
    )
