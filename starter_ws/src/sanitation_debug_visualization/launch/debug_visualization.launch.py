from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    fixed_frame = LaunchConfiguration("fixed_frame")
    show_static_targets = LaunchConfiguration("show_static_targets")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    default_rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("sanitation_debug_visualization"),
            "config",
            "debug_visualization.rviz",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "fixed_frame",
                default_value="base_link",
                description="Use base_link for baseline follow view or map with Nav2/SLAM.",
            ),
            DeclareLaunchArgument("show_static_targets", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            Node(
                package="sanitation_debug_visualization",
                executable="debug_visualization_node",
                name="sanitation_debug_visualization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "frame_id": fixed_frame,
                        "show_static_targets": show_static_targets,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="sanitation_debug_rviz",
                output="screen",
                arguments=["-d", rviz_config, "-f", fixed_frame],
                condition=IfCondition(rviz),
            ),
        ]
    )
