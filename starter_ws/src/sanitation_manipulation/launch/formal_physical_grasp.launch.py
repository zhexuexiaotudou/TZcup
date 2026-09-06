"""Start MoveIt planning and the truth-free physical grasp orchestrator.

The enclosing formal vehicle launch owns robot_state_publisher and
ros2_control. This launch adds one move_group instance plus the product
executor; it never exposes evaluator reset, teleport or entity selection.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package = FindPackageShare("sanitation_manipulation")
    config = PathJoinSubstitution([package, "config", "formal_grasp_executor.yaml"])
    scene_config = PathJoinSubstitution([package, "config", "bin_and_scene.yaml"])
    moveit_launch = PathJoinSubstitution([package, "launch", "manipulation.launch.py"])
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch),
                launch_arguments={
                    "use_sim_time": "true",
                    "start_control_stack": "false",
                    "start_state_publisher": "false",
                    # The formal vehicle publisher is the sole global
                    # /robot_description writer. MoveIt keeps its planning
                    # description as a private parameter only.
                    "publish_robot_description": "false",
                }.items(),
            ),
            Node(
                package="sanitation_manipulation",
                executable="moveit_planning_scene_bootstrap",
                name="moveit_planning_scene_bootstrap",
                parameters=[{"config_file": scene_config, "use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="sanitation_manipulation",
                executable="formal_physical_grasp_executor",
                name="formal_physical_grasp_executor",
                parameters=[config, {"planning_scene_config_file": scene_config}],
                output="screen",
            ),
        ]
    )
