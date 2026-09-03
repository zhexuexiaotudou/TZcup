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
    cube_name = LaunchConfiguration("cube_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    physics_engine = LaunchConfiguration("physics_engine")
    spawn_single_cube = LaunchConfiguration("spawn_single_cube")
    dry_accounting_mode = LaunchConfiguration("dry_accounting_mode")
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
    localization_launch = PathJoinSubstitution(
        [
            FindPackageShare("sanitation_localization"),
            "launch",
            "formal_localization_fusion.launch.py",
        ]
    )
    robot_description = ParameterValue(
        Command([
            "xacro ", model, " use_sim:=true bodywork_visible:=true",
            " dry_accounting_mode:=", dry_accounting_mode,
            # The grasp probe explicitly performs the operator reset through
            # the mechanical E-stop command bridge.  Starting this isolated
            # scene latched would leave its safety manager permanently
            # fail-closed before that product command can take effect.
            " initial_estop_latched:=false",
        ]),
        value_type=str,
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
        (
            "/safety/front_bumper/contact",
            "/formal_vehicle/simulation/raw/front_bumper/contact",
        ),
        (
            "/safety/rear_bumper/contact",
            "/formal_vehicle/simulation/raw/rear_bumper/contact",
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

    active_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "arm_controller",
            "gripper_controller", "cleaning_controller", "storage_controller",
            "service_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "40", "--service-call-timeout", "40",
            "--switch-timeout", "40", "--activate-as-group",
        ],
        output="screen",
    )
    velocity_controller_loader = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "brush_controller", "recovery_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "40", "--service-call-timeout", "40",
            "--switch-timeout", "40", "--inactive",
        ],
        output="screen",
    )
    safety_manager = Node(
        package="sanitation_safety",
        executable="whole_vehicle_safety_manager",
        parameters=[{"use_sim_time": False}],
        output="screen",
    )
    # Keep the grasp scene on the same safety path as the formal vehicle.  In
    # particular, do not synthesize an actuator permit here: these bridges and
    # simulators provide the BMS, traction, cleaning-fault and mechanical
    # E-stop inputs that the whole-vehicle safety manager requires.
    a300_drivetrain_adapter = Node(
        package="sanitation_gazebo_control",
        executable="a300_drivetrain_command_adapter",
        name="a300_drivetrain_command_adapter",
        parameters=[{"use_sim_time": False}],
        output="screen",
    )
    cleaning_actuator_command_mirror = Node(
        package="sanitation_gazebo_control",
        executable="cleaning_actuator_command_mirror",
        name="cleaning_actuator_command_mirror",
        parameters=[{"use_sim_time": False}],
        output="screen",
    )
    cleaning_actuator_scalar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="cleaning_actuator_scalar_bridge",
        arguments=[
            "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/command/lift_position@std_msgs/msg/Float64]gz.msgs.Double",
            "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/command/enable@std_msgs/msg/Bool]gz.msgs.Boolean",
            "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/command/reset_faults@std_msgs/msg/Bool]gz.msgs.Boolean",
            "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/fault_active@std_msgs/msg/Bool[gz.msgs.Boolean",
        ],
        output="screen",
    )
    cleaning_actuator_motor_bridge = Node(
        package="sanitation_gazebo_control",
        executable="cleaning_actuator_vector_bridge",
        name="cleaning_actuator_motor_bridge",
        output="screen",
    )
    a300_drivetrain_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="a300_drivetrain_bridge",
        arguments=[
            "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/actuator_enable@std_msgs/msg/Bool]gz.msgs.Boolean",
            "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/emergency_stop@std_msgs/msg/Bool[gz.msgs.Boolean",
            "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/status@std_msgs/msg/String[gz.msgs.StringMsg",
        ],
        remappings=[
            (
                "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/emergency_stop",
                "/emergency_stop",
            ),
            (
                "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/odom",
                "/odom/unfiltered",
            ),
        ],
        output="screen",
    )
    formal_auxiliary_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="formal_auxiliary_bridge",
        arguments=[
            "/formal_vehicle/simulation/command/emergency_stop@std_msgs/msg/Bool]gz.msgs.Boolean",
            "/formal_vehicle/simulation/command/emergency_stop_reset@std_msgs/msg/Bool]gz.msgs.Boolean",
            "/formal_vehicle/power/branches/safety/enabled@std_msgs/msg/Bool]gz.msgs.Boolean",
            "/formal_vehicle/simulation/command/main_power@std_msgs/msg/Bool]gz.msgs.Boolean",
            "/formal_vehicle/power/main_contactor_command@std_msgs/msg/Bool]gz.msgs.Boolean",
            "/formal_vehicle/power/main_isolator_closed@std_msgs/msg/Bool[gz.msgs.Boolean",
            "/formal_vehicle/power/main_contactor_closed@std_msgs/msg/Bool[gz.msgs.Boolean",
        ],
        output="screen",
    )
    a300_bms = Node(
        package="sanitation_power_system",
        executable="a300_bms_simulator",
        name="a300_bms_simulator",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("sanitation_power_system"),
                    "config",
                    "a300_40ah_bms.yaml",
                ]
            ),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("material", default_value="PET"),
            DeclareLaunchArgument("cube_name", default_value="material_cube"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("physics_engine", default_value="gz-physics-bullet-featherstone-plugin"),
            DeclareLaunchArgument("spawn_single_cube", default_value="true"),
            DeclareLaunchArgument(
                "dry_accounting_mode", default_value="physical_resident"
            ),
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
                condition=IfCondition(spawn_single_cube),
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        parameters=[{"robot_description": cube_description}],
                        arguments=[
                            "-param", "robot_description", "-name", cube_name,
                            # Work outside the body envelope so the fingers can
                            # reach the ground cube without hitting a cowl.
                            "-x", "0.300", "-y", "-0.950", "-z", "0.017",
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
                        # The formal publisher below is the single global
                        # String /robot_description writer.  Keep the state
                        # publisher's private model parameter out of that
                        # public product topic, as in formal_vehicle_sim.
                        remappings=[
                            (
                                "robot_description",
                                "/formal_vehicle/internal/robot_description_from_state_publisher",
                            )
                        ],
                        output="screen",
                    ),
                    Node(
                        package="sanitation_vehicle_description",
                        executable="formal_robot_description_publisher.py",
                        name="formal_robot_description_publisher",
                        parameters=[{"robot_description": robot_description}],
                        output="screen",
                    ),
                    Node(
                        package="sanitation_vehicle_description",
                        executable="formal_encoder_feedback_publisher.py",
                        name="formal_encoder_feedback_publisher",
                        parameters=[{"use_sim_time": use_sim_time}],
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
            a300_drivetrain_bridge,
            formal_auxiliary_bridge,
            a300_drivetrain_adapter,
            cleaning_actuator_command_mirror,
            cleaning_actuator_scalar_bridge,
            cleaning_actuator_motor_bridge,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "start_local_fusion": "true",
                    "start_navsat_transform": "false",
                    "start_global_fusion": "false",
                }.items(),
            ),
            Node(
                package="sanitation_safety",
                executable="simulation_safety_inputs",
                parameters=[{"use_sim_time": False, "initial_estop_active": False}],
                output="screen",
            ),
            a300_bms,
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
                actions=[active_controller_spawner],
            ),
            # Controller spawners can remain alive for several seconds after a
            # successful group switch on a low-real-time-factor Gazebo run.
            # Start the inactive brush/recovery loaders and the safety manager
            # from explicit startup deadlines rather than treating process
            # exit as proof that safety authority exists.
            TimerAction(period=16.0, actions=[velocity_controller_loader]),
            TimerAction(period=20.0, actions=[safety_manager]),
        ]
    )
