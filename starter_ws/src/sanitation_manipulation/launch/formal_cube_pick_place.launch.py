"""Launch the formal vehicle and one physical 30 mm material cube in Gazebo."""

from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    gui = LaunchConfiguration("gui")
    material = LaunchConfiguration("material")
    use_sim_time = LaunchConfiguration("use_sim_time")
    physics_engine = LaunchConfiguration("physics_engine")
    model = PathJoinSubstitution(
        [FindPackageShare("sanitation_manipulation"), "urdf", "formal_manipulation_acceptance.urdf.xacro"]
    )
    cube = PathJoinSubstitution(
        [FindPackageShare("sanitation_manipulation"), "urdf", "material_cube.urdf.xacro"]
    )
    world = PathJoinSubstitution(
        [FindPackageShare("sanitation_manipulation"), "worlds", "formal_cube_manipulation.sdf"]
    )
    gz_launch = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
    robot_description = ParameterValue(
        Command(["xacro ", model, " use_sim:=true bodywork_visible:=true"]), value_type=str
    )
    cube_description = ParameterValue(
        Command(["xacro ", cube, " material:=", material]), value_type=str
    )
    share_parent = str(Path(get_package_share_directory("sanitation_vehicle_description")).parent)
    resource_path = os.pathsep.join(
        value for value in (share_parent, os.environ.get("GZ_SIM_RESOURCE_PATH", "")) if value
    )
    contact_bridges = []
    for source, destination in (
        (
            "/world/formal_cube_manipulation/model/material_cube/link/cube_link/sensor/cube_contact/contact",
            "/manipulation/cube/contact",
        ),
        (
            "/world/formal_cube_manipulation/model/tzcup_formal_sanitation_vehicle/link/robotiq_85_left_finger_tip_link/sensor/left_finger_tip_contact/contact",
            "/manipulation/gripper/left_contact",
        ),
        (
            "/world/formal_cube_manipulation/model/tzcup_formal_sanitation_vehicle/link/robotiq_85_right_finger_tip_link/sensor/right_finger_tip_contact/contact",
            "/manipulation/gripper/right_contact",
        ),
        (
            "/world/formal_cube_manipulation/model/tzcup_formal_sanitation_vehicle/link/dry_bin_lid_link/sensor/dry_deposit_contact/contact",
            "/storage/dry_deposit/contact",
        ),
        (
            "/world/formal_cube_manipulation/model/tzcup_formal_sanitation_vehicle/link/dry_bin_link/sensor/dry_bin_floor_contact/contact",
            "/storage/dry_bin/floor_contact",
        ),
    ):
        contact_bridges.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[source + "@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"],
                remappings=[(source, destination)],
                output="screen",
            )
        )

    return LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("material", default_value="PET"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("physics_engine", default_value="gz-physics-bullet-featherstone-plugin"),
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
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        parameters=[{"robot_description": cube_description}],
                        arguments=[
                            "-param", "robot_description", "-name", "material_cube",
                            # Work outside the body envelope so the fingers can
                            # reach the ground cube without hitting a cowl.
                            "-x", "0.650", "-y", "-0.450", "-z", "0.017",
                        ],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=4.0,
                actions=[
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
                        arguments=[
                            "-param", "robot_description", "-name", "tzcup_formal_sanitation_vehicle", "-z", "0.005"
                        ],
                        output="screen",
                    ),
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/world/formal_cube_manipulation/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                    "/manipulation/grasp/detach@std_msgs/msg/Empty]gz.msgs.Empty",
                    "/manipulation/grasp/attach@std_msgs/msg/Empty]gz.msgs.Empty",
                    "/manipulation/grasp/state@std_msgs/msg/Bool[gz.msgs.Boolean",
                    "/world/formal_cube_manipulation/set_pose@ros_gz_interfaces/srv/SetEntityPose",
                ],
                output="screen",
            ),
            *contact_bridges,
            # DetachableJoint starts attached by design.  Release before any
            # arm motion, then reset the cube pose in the runtime verifier.
            TimerAction(
                period=10.0,
                actions=[
                    ExecuteProcess(
                        cmd=["gz", "topic", "-t", "/manipulation/grasp/detach", "-m", "gz.msgs.Empty", "-p", ""],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "joint_state_broadcaster", "arm_controller", "gripper_controller", "storage_controller",
                            "--controller-manager", "/controller_manager",
                            "--controller-manager-timeout", "40", "--service-call-timeout", "40",
                            "--switch-timeout", "40", "--activate-as-group",
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
