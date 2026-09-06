"""Spawn the formal vehicle in a dedicated Gazebo Harmonic validation world."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def _start_actions_unless_shutdown(context, *actions):
    """Do not let controller-exit events start nodes during launch teardown."""
    return [] if context.is_shutdown else list(actions)


def generate_launch_description() -> LaunchDescription:
    gui = LaunchConfiguration("gui")
    headless_rendering = LaunchConfiguration("headless_rendering")
    start_controllers = LaunchConfiguration("start_controllers")
    enable_safety_manager = LaunchConfiguration("enable_safety_manager")
    start_service_drain_safety_manager = LaunchConfiguration(
        "start_service_drain_safety_manager"
    )
    start_simulation_safety_inputs = LaunchConfiguration(
        "start_simulation_safety_inputs"
    )
    start_power_system_simulators = LaunchConfiguration(
        "start_power_system_simulators"
    )
    start_localization = LaunchConfiguration("start_localization")
    simulation_initial_estop_active = LaunchConfiguration(
        "simulation_initial_estop_active"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    physics_engine = LaunchConfiguration("physics_engine")
    bodywork_visible = LaunchConfiguration("bodywork_visible")
    high_bandwidth_sensor_runtime = LaunchConfiguration(
        "high_bandwidth_sensor_runtime"
    )
    start_high_bandwidth_sensor_bridges = LaunchConfiguration(
        "start_high_bandwidth_sensor_bridges"
    )
    high_bandwidth_bridges_enabled = PythonExpression(
        [
            "'",
            high_bandwidth_sensor_runtime,
            "'.lower() == 'true' and '",
            start_high_bandwidth_sensor_bridges,
            "'.lower() == 'true'",
        ]
    )
    visual_acceptance_runtime = LaunchConfiguration("visual_acceptance_runtime")
    start_product_bridge = LaunchConfiguration("start_product_bridge")
    start_product_support_parameter_bridges = LaunchConfiguration(
        "start_product_support_parameter_bridges"
    )
    start_a300_transport_bridge = LaunchConfiguration(
        "start_a300_transport_bridge"
    )
    start_cleaning_actuator_scalar_bridge = LaunchConfiguration(
        "start_cleaning_actuator_scalar_bridge"
    )
    cleaning_realtime_telemetry_enabled = LaunchConfiguration(
        "cleaning_realtime_telemetry_enabled"
    )
    cleaning_status_json_enabled = LaunchConfiguration(
        "cleaning_status_json_enabled"
    )
    cleaning_status_json_publish_rate_hz = LaunchConfiguration(
        "cleaning_status_json_publish_rate_hz"
    )
    dry_load_mass_kg = LaunchConfiguration("dry_load_mass_kg")
    dry_accounting_mode = LaunchConfiguration("dry_accounting_mode")
    wastewater_load_mass_kg = LaunchConfiguration("wastewater_load_mass_kg")
    water_evaluation_interfaces = LaunchConfiguration("water_evaluation_interfaces")
    dry_bin_evaluation_interfaces = LaunchConfiguration("dry_bin_evaluation_interfaces")
    service_door_evaluation_interfaces = LaunchConfiguration(
        "service_door_evaluation_interfaces"
    )
    squeegee_evaluation_interfaces = LaunchConfiguration(
        "squeegee_evaluation_interfaces"
    )
    manipulation_sim_interfaces = LaunchConfiguration("manipulation_sim_interfaces")
    spawn_robot = LaunchConfiguration("spawn_robot")
    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")
    default_model = PathJoinSubstitution(
        [FindPackageShare("sanitation_vehicle_description"), "urdf", "formal_competition_vehicle.urdf.xacro"]
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare("sanitation_vehicle_description"), "worlds", "formal_vehicle_validation.sdf"]
    )
    visual_bridge_contract_path = (
        Path(get_package_share_directory("sanitation_vehicle_description"))
        / "config"
        / "formal_visual_sensor_bridge.yaml"
    )
    visual_bridge_contract = yaml.safe_load(
        visual_bridge_contract_path.read_text(encoding="utf-8")
    )
    visual_image_topics = [row["gz_topic_name"] for row in visual_bridge_contract]
    if (
        len(visual_image_topics) != 19
        or len(set(visual_image_topics)) != len(visual_image_topics)
        or any(
            row["ros_topic_name"] != row["gz_topic_name"]
            for row in visual_bridge_contract
        )
        or any(
            row["ros_type_name"] != "sensor_msgs/msg/Image"
            or row["gz_type_name"] != "gz.msgs.Image"
            for row in visual_bridge_contract
        )
        or any(row["direction"] != "GZ_TO_ROS" for row in visual_bridge_contract)
        or any(
            row["qos_profile"] != "DEFAULT_RELIABLE"
            for row in visual_bridge_contract
        )
    ):
        raise RuntimeError(
            f"invalid formal visual image bridge contract: {visual_bridge_contract_path}"
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
            "xacro ", model,
            " use_sim:=true dry_load_mass_kg:=", dry_load_mass_kg,
            " dry_accounting_mode:=", dry_accounting_mode,
            " wastewater_load_mass_kg:=", wastewater_load_mass_kg,
            " bodywork_visible:=",
            bodywork_visible,
            " high_bandwidth_sensor_runtime:=",
            high_bandwidth_sensor_runtime,
            " service_door_evaluation_interfaces:=",
            service_door_evaluation_interfaces,
            " initial_estop_latched:=",
            simulation_initial_estop_active,
            " cleaning_realtime_telemetry_enabled:=",
            cleaning_realtime_telemetry_enabled,
            " cleaning_status_json_enabled:=",
            cleaning_status_json_enabled,
            " cleaning_status_json_publish_rate_hz:=",
            cleaning_status_json_publish_rate_hz,
        ]),
        value_type=str,
    )
    # sdformat rewrites package:// URIs to model:// URIs. Gazebo therefore
    # needs the parent of the installed package share directory on its resource
    # path so every vendored and project-generated mesh resolves offline.
    package_share_parent = str(Path(get_package_share_directory("sanitation_vehicle_description")).parent)
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = os.pathsep.join(
        item for item in (package_share_parent, existing_resource_path) if item
    )
    # Model plugins are installed by a separate ament package. Gazebo does not
    # search every ament prefix automatically, so make that package's lib
    # directory explicit instead of relying on a developer shell's inherited
    # GZ_SIM_SYSTEM_PLUGIN_PATH.
    gazebo_control_lib = str(
        Path(get_package_prefix("sanitation_gazebo_control")) / "lib"
    )
    gazebo_auxiliary_lib = str(
        Path(get_package_prefix("sanitation_gazebo_auxiliary")) / "lib"
    )
    existing_system_plugin_path = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    system_plugin_path = os.pathsep.join(
        item
        for item in (
            gazebo_control_lib,
            gazebo_auxiliary_lib,
            existing_system_plugin_path,
        )
        if item
    )

    legacy_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "arm_controller", "gripper_controller",
            "cleaning_controller", "storage_controller", "service_controller",
            "brush_controller", "recovery_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
            "--service-call-timeout", "30",
            "--switch-timeout", "30",
            "--activate-as-group",
        ],
        output="screen",
        condition=UnlessCondition(enable_safety_manager),
    )

    # Position controllers stay active so an inhibit can cancel goals and
    # hold gravity-loaded joints. Brush and recovery are loaded inactive; only
    # the fail-closed safety manager may activate those velocity controllers.
    safe_active_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "arm_controller",
            "gripper_controller", "cleaning_controller", "storage_controller",
            "service_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
            "--service-call-timeout", "30",
            "--switch-timeout", "30",
            "--activate-as-group",
        ],
        output="screen",
        condition=IfCondition(enable_safety_manager),
    )
    safe_velocity_controller_loader = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "brush_controller", "recovery_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
            "--service-call-timeout", "30",
            "--switch-timeout", "30",
            "--inactive",
        ],
        output="screen",
    )
    safety_manager = Node(
        package="sanitation_safety",
        executable="whole_vehicle_safety_manager",
        parameters=[{"use_sim_time": False}],
        output="screen",
    )
    service_drain_safety_manager = Node(
        package="sanitation_safety",
        executable="service_drain_safety_manager",
        parameters=[{"use_sim_time": False}],
        output="screen",
        condition=IfCondition(start_service_drain_safety_manager),
    )
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
        package="sanitation_gazebo_control",
        executable="cleaning_actuator_scalar_native_bridge",
        name="cleaning_actuator_scalar_bridge",
        output="screen",
        condition=IfCondition(start_cleaning_actuator_scalar_bridge),
    )
    cleaning_actuator_motor_bridge = Node(
        package="sanitation_gazebo_control",
        executable="cleaning_actuator_vector_bridge",
        name="cleaning_actuator_motor_bridge",
        # Sole /clock owner for both product and bounded diagnostic launches.
        output="screen",
    )
    a300_drivetrain_bridge = Node(
        package="sanitation_gazebo_control",
        executable="a300_drivetrain_native_bridge",
        name="a300_drivetrain_bridge",
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
        condition=IfCondition(start_a300_transport_bridge),
    )
    # This bridge is the only formal ROS writer of /emergency_stop. Commands
    # travel ROS -> Gazebo, while applied lighting and the physical latch travel
    # Gazebo -> ROS. Keeping command and state topics separate prevents an
    # operator publisher from bypassing the mechanical latch.
    formal_auxiliary_bridge = Node(
        package="sanitation_gazebo_control",
        executable="formal_auxiliary_native_bridge",
        name="formal_auxiliary_bridge",
        output="screen",
        condition=IfCondition(start_product_support_parameter_bridges),
    )
    service_door_evaluation_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="formal_service_door_evaluation_bridge",
        arguments=[
            "/formal_vehicle/evaluation/bodywork_service/power/hinge_target_rad@std_msgs/msg/Float64]gz.msgs.Double",
            "/formal_vehicle/evaluation/bodywork_service/power/latch_target_rad@std_msgs/msg/Float64]gz.msgs.Double",
            "/formal_vehicle/evaluation/bodywork_service/compute/hinge_target_rad@std_msgs/msg/Float64]gz.msgs.Double",
            "/formal_vehicle/evaluation/bodywork_service/compute/latch_target_rad@std_msgs/msg/Float64]gz.msgs.Double",
            "/formal_vehicle/evaluation/bodywork_service/wet/hinge_target_rad@std_msgs/msg/Float64]gz.msgs.Double",
            "/formal_vehicle/evaluation/bodywork_service/wet/latch_target_rad@std_msgs/msg/Float64]gz.msgs.Double",
            "/formal_vehicle/evaluation/bodywork_service/rear_dry/hinge_target_rad@std_msgs/msg/Float64]gz.msgs.Double",
            "/formal_vehicle/evaluation/bodywork_service/rear_dry/latch_target_rad@std_msgs/msg/Float64]gz.msgs.Double",
        ],
        output="screen",
        condition=IfCondition(service_door_evaluation_interfaces),
    )
    # The service-door evaluator must observe Gazebo's physical model state,
    # not the controller-owned /joint_states aggregate.  Keep this topic
    # dedicated so it cannot become a second writer for product joint state.
    service_door_physical_state_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="formal_service_door_physical_state_bridge",
        arguments=[
            "/formal_vehicle/evaluation/bodywork_service/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        remappings=[
            (
                "/formal_vehicle/evaluation/bodywork_service/joint_states",
                "/formal/service_door_joint_states",
            ),
        ],
        output="screen",
        condition=IfCondition(service_door_evaluation_interfaces),
    )
    squeegee_evaluation_bridge = Node(
        package="sanitation_gazebo_control",
        executable="formal_contact_evaluation_native_bridge",
        name="formal_squeegee_evaluation_bridge",
        parameters=[{"endpoint_group": "squeegee"}],
        remappings=[
            (
                "/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link/squeegee_link/sensor/squeegee_blade_ground_contact/contact",
                "/cleaning/squeegee/contact",
            ),
        ],
        output="screen",
        condition=IfCondition(squeegee_evaluation_interfaces),
    )
    brush_contact_evaluation_bridge = Node(
        package="sanitation_gazebo_control",
        executable="formal_contact_evaluation_native_bridge",
        name="formal_brush_contact_evaluation_bridge",
        parameters=[{"endpoint_group": "brushes"}],
        remappings=[
            (
                "/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link/left_side_brush_link/sensor/left_side_brush_ground_contact/contact",
                "/cleaning/left_side_brush/contact",
            ),
            (
                "/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link/right_side_brush_link/sensor/right_side_brush_ground_contact/contact",
                "/cleaning/right_side_brush/contact",
            ),
            (
                "/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link/central_roller_link/sensor/central_roller_ground_contact/contact",
                "/cleaning/central_roller/contact",
            ),
        ],
        output="screen",
        condition=IfCondition(squeegee_evaluation_interfaces),
    )
    simulation_safety_inputs = Node(
        package="sanitation_safety",
        executable="simulation_safety_inputs",
        parameters=[
            {
                "use_sim_time": False,
                "initial_estop_active": ParameterValue(
                    simulation_initial_estop_active, value_type=bool
                ),
            }
        ],
        output="screen",
        condition=IfCondition(start_simulation_safety_inputs),
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
        condition=IfCondition(start_power_system_simulators),
    )
    charge_interface_manager = Node(
        package="sanitation_power_system",
        executable="charge_interface_manager",
        name="charge_interface_manager",
        parameters=[{"use_sim_time": False}],
        output="screen",
        condition=IfCondition(start_power_system_simulators),
    )
    charge_receptacle_contact_bridge = Node(
        package="sanitation_gazebo_control",
        executable="formal_contact_evaluation_native_bridge",
        name="charge_receptacle_contact_bridge",
        parameters=[{"endpoint_group": "charge_receptacle"}],
        remappings=[
            (
                "/formal_vehicle/gazebo/charge_receptacle/contact",
                "/formal_vehicle/service/raw/charge_plug_contact",
            )
        ],
        output="screen",
        condition=IfCondition(start_product_support_parameter_bridges),
    )
    wastewater_drain_contact_bridge = Node(
        package="sanitation_gazebo_control",
        executable="formal_contact_evaluation_native_bridge",
        name="wastewater_drain_contact_bridge",
        parameters=[{"endpoint_group": "wastewater_drain"}],
        remappings=[
            (
                "/formal_vehicle/gazebo/wastewater_drain_coupling/contact",
                "/formal_vehicle/service/raw/drain_hose_contact",
            )
        ],
        output="screen",
        condition=IfCondition(start_product_support_parameter_bridges),
    )

    # Ground-water truth and episode reset commands belong to the evaluator,
    # not the product ROS graph.  They are opt-in so the deployed launch cannot
    # spoof water removal or mass-conservation acceptance through ROS topics.
    water_evaluation_bridge = Node(
        package="sanitation_gazebo_control",
        executable="water_evaluation_bridge",
        name="water_evaluation_bridge",
        output="screen",
        condition=IfCondition(water_evaluation_interfaces),
    )

    # Dry payload truth is also evaluator-only. The product graph receives no
    # interface that can inject, delete or rewrite contained rigid bodies.
    dry_bin_evaluation_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="dry_bin_evaluation_bridge",
        arguments=[
            "/model/tzcup_formal_sanitation_vehicle/dry_bin/contained_object_count@std_msgs/msg/Int32[gz.msgs.Int32",
            "/model/tzcup_formal_sanitation_vehicle/dry_bin/contained_mass_kg@std_msgs/msg/Float64[gz.msgs.Double",
            "/model/tzcup_formal_sanitation_vehicle/dry_bin/status_json@std_msgs/msg/String[gz.msgs.StringMsg",
        ],
        output="screen",
        condition=IfCondition(dry_bin_evaluation_interfaces),
    )
    manipulation_sim_bridge = Node(
        package="sanitation_gazebo_control",
        executable="manipulation_sim_bridge",
        name="manipulation_sim_bridge",
        output="screen",
        condition=IfCondition(manipulation_sim_interfaces),
    )

    return LaunchDescription(
        [
            # Formal simulation is a single-host acceptance.  Gazebo
            # Transport does not inherit the ROS 2 DDS interface policy, so
            # pin its discovery/data plane to loopback before any simulator or
            # bridge action is created.
            SetEnvironmentVariable("GZ_IP", "127.0.0.1"),
            SetEnvironmentVariable("IGN_IP", "127.0.0.1"),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", system_plugin_path),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument(
                "headless_rendering",
                default_value="false",
                description=(
                    "Enable Gazebo's offscreen Ogre2 renderer for server-only camera "
                    "workloads such as formal visual acceptance."
                ),
            ),
            DeclareLaunchArgument("start_controllers", default_value="true"),
            DeclareLaunchArgument(
                "enable_safety_manager",
                default_value="true",
                description=(
                    "Fail-closed command ownership for base, cleaning, arm, "
                    "gripper, brush and recovery actuators."
                ),
            ),
            DeclareLaunchArgument(
                "start_service_drain_safety_manager",
                default_value="true",
                description=(
                    "Start the product service-drain command owner. Disable only "
                    "for the evaluator-isolated water plant/mass-ledger gate; the "
                    "separate service-interface gate validates the full physical "
                    "cap, hose, power and safety-manager chain."
                ),
            ),
            DeclareLaunchArgument(
                "start_simulation_safety_inputs",
                default_value="false",
                description=(
                    "Start the engineering-only relay, heartbeat, E-stop, "
                    "battery and lighting simulator."
                ),
            ),
            DeclareLaunchArgument(
                "start_power_system_simulators",
                default_value=start_simulation_safety_inputs,
                description=(
                    "Start the A300 BMS and charge-interface simulators. "
                    "Defaults to the simulation-safety-input setting but can "
                    "be enabled independently for an in-process acceptance probe."
                ),
            ),
            DeclareLaunchArgument(
                "start_localization",
                default_value="true",
                description=(
                    "Start the local EKF as the sole /odom and "
                    "odom-to-base_footprint authority."
                ),
            ),
            DeclareLaunchArgument(
                "simulation_initial_estop_active",
                default_value="true",
                description="Power-up E-stop state for simulation inputs.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("bodywork_visible", default_value="true"),
            DeclareLaunchArgument(
                "high_bandwidth_sensor_runtime",
                default_value="true",
                description=(
                    "Enable MID360 and camera simulation. Physical links stay "
                    "in the URDF when disabled for the 2D-lidar mapping pass."
                ),
            ),
            DeclareLaunchArgument(
                "start_high_bandwidth_sensor_bridges",
                default_value="true",
                description=(
                    "Start ROS conversion for high-bandwidth image and point-cloud "
                    "streams. Diagnostic Gazebo-native probes may disable these "
                    "bridges while keeping the physical sensors enabled."
                ),
            ),
            DeclareLaunchArgument(
                "visual_acceptance_runtime",
                default_value="false",
                description=(
                    "Start the dedicated nineteen-topic ros_gz_image bridge only "
                    "for the low-rate visual-acceptance studio."
                ),
            ),
            DeclareLaunchArgument(
                "start_product_bridge",
                default_value="true",
                description="Start the default product ROS-Gazebo parameter bridge.",
            ),
            DeclareLaunchArgument(
                "start_product_support_parameter_bridges",
                default_value="true",
                description=(
                    "Start auxiliary, service-contact, and bumper parameter bridges. "
                    "Bounded native-transport diagnostics may disable them."
                ),
            ),
            DeclareLaunchArgument(
                "start_a300_transport_bridge",
                default_value="true",
                description="Start the default A300 ROS-Gazebo drivetrain bridge.",
            ),
            DeclareLaunchArgument(
                "start_cleaning_actuator_scalar_bridge",
                default_value="true",
                description="Start scalar cleaning-actuator ROS-Gazebo interfaces.",
            ),
            DeclareLaunchArgument("dry_load_mass_kg", default_value="0.0"),
            DeclareLaunchArgument(
                "dry_accounting_mode",
                default_value="physical_resident",
                description=(
                    "Exclusive dry ledger: physical_resident retains independent "
                    "bin rigid bodies and rejects non-zero aggregate dry mass; "
                    "aggregate is for legacy bulk loads without rigid bodies."
                ),
            ),
            DeclareLaunchArgument("wastewater_load_mass_kg", default_value="0.0"),
            DeclareLaunchArgument(
                "cleaning_realtime_telemetry_enabled",
                default_value="true",
                description="Diagnostic isolation switch; product default keeps typed 20 Hz telemetry enabled.",
            ),
            DeclareLaunchArgument(
                "cleaning_status_json_enabled",
                default_value="true",
                description="Diagnostic isolation switch for the Gazebo-only JSON stream.",
            ),
            DeclareLaunchArgument(
                "cleaning_status_json_publish_rate_hz",
                default_value="20.0",
                description="Gazebo-only status JSON rate in (0, 20] Hz.",
            ),
            DeclareLaunchArgument(
                "water_evaluation_interfaces",
                default_value="false",
                description="Opt-in evaluator-only water truth/reset ROS bridge.",
            ),
            DeclareLaunchArgument(
                "dry_bin_evaluation_interfaces",
                default_value="false",
                description="Opt-in evaluator-only dry-bin observation ROS bridge.",
            ),
            DeclareLaunchArgument(
                "service_door_evaluation_interfaces",
                default_value="false",
                description=(
                    "Opt-in evaluator-only bodywork service-door target bridge."
                ),
            ),
            DeclareLaunchArgument(
                "squeegee_evaluation_interfaces",
                default_value="false",
                description=(
                    "Opt-in evaluator-only squeegee compliance and cleaning-contact bridges."
                ),
            ),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("model", default_value=default_model),
            DeclareLaunchArgument(
                "spawn_robot",
                default_value="true",
                description=(
                    "Spawn from robot_description through UserCommands. Set false only "
                    "when a source-bound world already embeds the same formal vehicle."
                ),
            ),
            DeclareLaunchArgument(
                "manipulation_sim_interfaces",
                default_value="false",
                description=(
                    "Expose identity-free contact, attachment-state and dry-bin "
                    "sensor bridges for the formal grasp executor."
                ),
            ),
            DeclareLaunchArgument(
                "physics_engine",
                default_value="gz-physics-dartsim-plugin",
                description=(
                    "Physics engine; DART is the validated whole-vehicle engine. "
                    "The Gazebo Harmonic Bullet backends inject unbounded energy "
                    "into this articulated four-wheel model and are not accepted."
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={
                    "gz_args": [
                        " -r -s ",
                        world,
                        " --physics-engine ",
                        physics_engine,
                        IfElseSubstitution(
                            headless_rendering,
                            " --headless-rendering",
                            "",
                        ),
                    ]
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                condition=IfCondition(gui),
                launch_arguments={"gz_args": " -g"}.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
                remappings=[
                    ("robot_description", "/formal_vehicle/internal/robot_description_from_state_publisher")
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
                # base_footprint is the wheel-ground projection; use only a
                # 5 mm contact-settling clearance instead of lifting the car.
                arguments=["-param", "robot_description", "-name", "tzcup_formal_sanitation_vehicle", "-z", "0.005"],
                output="screen",
                condition=IfCondition(spawn_robot),
            ),
            # Payload mass remains owned by physical simulation, and water
            # service-drain commands remain fail-closed through the safety
            # manager and plugin watchdog.  The native product bridge exposes
            # operational telemetry only: evaluator truth/reset interfaces stay
            # behind their opt-in evaluator launch arguments.
            Node(
                package="sanitation_gazebo_control",
                executable="formal_vehicle_product_native_bridge",
                name="formal_vehicle_product_bridge",
                output="screen",
                condition=IfCondition(start_product_bridge),
            ),
            # Raw images and point clouds dwarf the control-plane traffic.  A
            # dedicated lazy bridge preserves every product topic, resolution
            # and source update rate while avoiding conversion / DDS queues for
            # streams with no ROS subscriber (for example D435 point clouds in
            # the metadata-only formal sensor gate).
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="formal_vehicle_high_bandwidth_sensor_bridge",
                parameters=[
                    {
                        "config_file": PathJoinSubstitution(
                            [
                                FindPackageShare("sanitation_vehicle_description"),
                                "config",
                                "formal_high_bandwidth_sensor_bridge.yaml",
                            ]
                        ),
                        "subscription_heartbeat": 100,
                    }
                ],
                output="screen",
                condition=IfCondition(high_bandwidth_bridges_enabled),
            ),
            # ros_gz_bridge parameter_bridge discovers the image endpoints but
            # did not forward these 1600x1000 frames in the pinned Jazzy stack.
            # The ROS-supported image-specific bridge consumes the same topic
            # contract in one process. Reliable QoS is intentional: a raw frame
            # is 4.8 MB and BEST_EFFORT dropped fragmented loopback samples in
            # the pinned CycloneDDS runtime during the formal visual gate.
            Node(
                package="ros_gz_image",
                executable="image_bridge",
                name="formal_vehicle_visual_bridge",
                arguments=visual_image_topics,
                parameters=[{"qos": "default"}],
                output="screen",
                condition=IfCondition(visual_acceptance_runtime),
            ),
            Node(
                package="sanitation_vehicle_description",
                executable="formal_fisheye_camera_info_publisher.py",
                name="formal_fisheye_camera_info_publisher",
                output="screen",
                condition=IfCondition(high_bandwidth_bridges_enabled),
            ),
            water_evaluation_bridge,
            dry_bin_evaluation_bridge,
            manipulation_sim_bridge,
            a300_drivetrain_bridge,
            formal_auxiliary_bridge,
            service_door_evaluation_bridge,
            service_door_physical_state_bridge,
            squeegee_evaluation_bridge,
            brush_contact_evaluation_bridge,
            a300_drivetrain_adapter,
            cleaning_actuator_command_mirror,
            cleaning_actuator_scalar_bridge,
            cleaning_actuator_motor_bridge,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch),
                condition=IfCondition(start_localization),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "start_local_fusion": "true",
                    "start_global_fusion": "false",
                }.items(),
            ),
            charge_receptacle_contact_bridge,
            wastewater_drain_contact_bridge,
            Node(
                package="sanitation_gazebo_control",
                executable="formal_contact_evaluation_native_bridge",
                name="front_bumper_contact_bridge",
                parameters=[{"endpoint_group": "front_bumper"}],
                remappings=[
                    (
                        "/safety/front_bumper/contact",
                        "/formal_vehicle/simulation/raw/front_bumper/contact",
                    )
                ],
                output="screen",
                condition=IfCondition(start_product_support_parameter_bridges),
            ),
            Node(
                package="sanitation_gazebo_control",
                executable="formal_contact_evaluation_native_bridge",
                name="rear_bumper_contact_bridge",
                parameters=[{"endpoint_group": "rear_bumper"}],
                remappings=[
                    (
                        "/safety/rear_bumper/contact",
                        "/formal_vehicle/simulation/raw/rear_bumper/contact",
                    )
                ],
                output="screen",
                condition=IfCondition(start_product_support_parameter_bridges),
            ),
            simulation_safety_inputs,
            a300_bms,
            charge_interface_manager,
            TimerAction(
                period=6.0,
                actions=[legacy_controller_spawner, safe_active_controller_spawner],
                condition=IfCondition(start_controllers),
                cancel_on_shutdown=True,
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=safe_active_controller_spawner,
                    on_exit=[
                        OpaqueFunction(
                            function=_start_actions_unless_shutdown,
                            args=[safe_velocity_controller_loader],
                        )
                    ],
                ),
                condition=IfCondition(start_controllers),
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=safe_velocity_controller_loader,
                    on_exit=[
                        OpaqueFunction(
                            function=_start_actions_unless_shutdown,
                            args=[safety_manager, service_drain_safety_manager],
                        )
                    ],
                ),
                condition=IfCondition(start_controllers),
            ),
        ]
    )
