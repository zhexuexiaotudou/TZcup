from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution


def generate_launch_description():
    vehicle_xacro = PathJoinSubstitution(
        [
            FindPackageShare("sanitation_vehicle_description"),
            "urdf",
            "sanitation_vehicle.urdf.xacro",
        ]
    )
    config_root = FindPackageShare("sanitation_manipulation")
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", vehicle_xacro, " enable_manipulator:=true"]),
            value_type=str,
        )
    }
    semantic_path = PathJoinSubstitution(
        [config_root, "config", "sanitation_arm.srdf"]
    )
    kinematics = PathJoinSubstitution(
        [config_root, "config", "kinematics.yaml"]
    )
    joint_limits = PathJoinSubstitution(
        [config_root, "config", "joint_limits.yaml"]
    )
    ompl = PathJoinSubstitution(
        [config_root, "config", "ompl_planning.yaml"]
    )
    moveit_controllers = PathJoinSubstitution(
        [config_root, "config", "moveit_controllers.yaml"]
    )
    ros2_controllers = PathJoinSubstitution(
        [config_root, "config", "ros2_controllers.yaml"]
    )
    semantic = {
        "robot_description_semantic": ParameterValue(
            Command(["cat ", semantic_path]), value_type=str
        )
    }
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, ros2_controllers],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "arm_controller", "gripper_controller"],
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    robot_description,
                    semantic,
                    kinematics,
                    joint_limits,
                    ompl,
                    moveit_controllers,
                    {"planning_scene_monitor.publish_planning_scene": True},
                ],
            ),
        ]
    )
