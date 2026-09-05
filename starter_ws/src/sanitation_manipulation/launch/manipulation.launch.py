"""Launch the formal UR5e/2F-85 MoveIt planning service.

``start_control_stack:=false`` reuses the controller manager and state
publisher already owned by the formal Gazebo vehicle launch. The default true
is a standalone planning / fake-control path backed by
``mock_components/GenericSystem``. It is not the real-hardware bring-up path.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _yaml(path: Path):
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _flatten(prefix: str, value):
    """Emit explicit dotted ROS parameters for MoveIt 2.12/Jazzy."""
    flattened = {}
    for key, item in value.items():
        name = f"{prefix}.{key}" if prefix else key
        if isinstance(item, dict):
            flattened.update(_flatten(name, item))
        else:
            flattened[name] = item
    return flattened


def generate_launch_description() -> LaunchDescription:
    package = Path(get_package_share_directory("sanitation_manipulation"))
    vehicle = Path(get_package_share_directory("sanitation_vehicle_description"))
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_control_stack = LaunchConfiguration("start_control_stack")
    start_state_publisher = LaunchConfiguration("start_state_publisher")
    publish_robot_description = LaunchConfiguration("publish_robot_description")
    xacro_path = vehicle / "urdf" / "formal_competition_vehicle.urdf.xacro"
    controller_path = vehicle / "config" / "formal_vehicle_controllers.yaml"

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    str(xacro_path),
                    # MoveIt needs the URDF mimic graph so its predicted
                    # follower-link collision geometry tracks the single
                    # commanded 2F-85 knuckle.  Gazebo is spawned from a
                    # separate use_sim:=true description whose custom effort
                    # plugin owns those five physical follower joints.
                    " use_sim:=false bodywork_visible:=true ",
                    "high_bandwidth_sensor_runtime:=true controller_config_path:=",
                    str(controller_path),
                ]
            ),
            value_type=str,
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": (package / "config" / "formal_vehicle.srdf").read_text(
            encoding="utf-8"
        )
    }
    robot_description_kinematics = {
        "robot_description_kinematics": _yaml(package / "config" / "kinematics.yaml")
    }
    robot_description_planning = {
        "robot_description_planning": _yaml(package / "config" / "joint_limits.yaml")
    }
    ompl = _yaml(package / "config" / "ompl_planning.yaml")
    # MoveIt 2.12/Jazzy's single-pipeline compatibility path reads OMPL at the
    # move_group root. It avoids the known 2.12 multi-pipeline namespace bug.
    planning_pipeline = _flatten("move_group", ompl)
    trajectory_execution = _yaml(package / "config" / "moveit_controllers.yaml")
    move_group_parameters = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        planning_pipeline,
        trajectory_execution,
        {
            "use_sim_time": use_sim_time,
            "allow_trajectory_execution": True,
            "trajectory_execution.allowed_execution_duration_scaling": 1.5,
            "trajectory_execution.allowed_goal_duration_margin": 1.0,
            "planning_scene_monitor.publish_planning_scene": True,
            "planning_scene_monitor.publish_geometry_updates": True,
            "planning_scene_monitor.publish_state_updates": True,
            "planning_scene_monitor.publish_transforms_updates": True,
            "publish_robot_description": ParameterValue(
                publish_robot_description, value_type=bool
            ),
            "publish_robot_description_semantic": True,
        },
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_control_stack", default_value="true"),
            DeclareLaunchArgument("start_state_publisher", default_value="true"),
            DeclareLaunchArgument("publish_robot_description", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                condition=IfCondition(start_state_publisher),
                parameters=[robot_description, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                condition=IfCondition(start_control_stack),
                parameters=[robot_description, str(controller_path)],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                condition=IfCondition(start_control_stack),
                arguments=[
                    "joint_state_broadcaster",
                    "arm_controller",
                    "gripper_controller",
                    "storage_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--activate-as-group",
                ],
                output="screen",
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                parameters=move_group_parameters,
            ),
        ]
    )
