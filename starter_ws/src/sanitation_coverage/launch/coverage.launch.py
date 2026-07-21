from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare("sanitation_coverage"), "config", "coverage.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('footprint_profile', default_value='production'),
            Node(
                package="opennav_coverage",
                executable="opennav_coverage",
                name="coverage_server",
                output="screen",
                parameters=[params, {
                    'robot_width': ParameterValue(
                        PythonExpression(["0.83 if '", LaunchConfiguration('footprint_profile'), "' == 'stage5br6w_v4' else 0.72"]),
                        value_type=float,
                    ),
                }],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="coverage_lifecycle_manager",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": True,
                        "node_names": ["coverage_server"],
                    }
                ],
            ),
        ]
    )
