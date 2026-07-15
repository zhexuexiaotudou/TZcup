from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless_rendering")
    operational_profile = LaunchConfiguration("operational_profile")
    max_linear_velocity = LaunchConfiguration("max_linear_velocity")
    max_angular_velocity = LaunchConfiguration("max_angular_velocity")
    random_seed = LaunchConfiguration("random_seed")
    ekf_config = LaunchConfiguration("ekf_config")
    dynamics_arguments = (
        "physical_wheel_radius",
        "physical_track_width",
        "drive_wheel_radius",
        "drive_wheel_separation",
        "wheel_mu_longitudinal",
        "wheel_mu_lateral",
        "slip_compliance_longitudinal",
        "slip_compliance_lateral",
        "enable_wheel_slip",
    )
    sim_launch = PathJoinSubstitution(
        [FindPackageShare("sanitation_bringup"), "launch", "sim.launch.py"]
    )
    calibration_world = PathJoinSubstitution(
        [FindPackageShare("sanitation_worlds"), "worlds", "motion_calibration_world.sdf"]
    )
    default_ekf_config = PathJoinSubstitution(
        [FindPackageShare("sanitation_bringup"), "config", "selected_ekf.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("headless_rendering", default_value="true"),
            DeclareLaunchArgument("operational_profile", default_value="stress"),
            DeclareLaunchArgument("max_linear_velocity", default_value="0.45"),
            DeclareLaunchArgument("max_angular_velocity", default_value="0.60"),
            DeclareLaunchArgument("random_seed", default_value="0"),
            DeclareLaunchArgument("ekf_config", default_value=default_ekf_config),
            *[
                DeclareLaunchArgument(name, default_value=default)
                for name, default in (
                    ("physical_wheel_radius", "0.14"),
                    ("physical_track_width", "0.80"),
                    ("drive_wheel_radius", "0.14"),
                    ("drive_wheel_separation", "0.80"),
                    ("wheel_mu_longitudinal", "1.0"),
                    ("wheel_mu_lateral", "1.0"),
                    ("slip_compliance_longitudinal", "0.0"),
                    ("slip_compliance_lateral", "0.0"),
                    ("enable_wheel_slip", "false"),
                )
            ],
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    "gui": gui,
                    "headless_rendering": headless,
                    "enable_command_timeout": "false",
                    "enable_ekf": "true",
                    "world_file": calibration_world,
                    "random_seed": random_seed,
                    "ekf_config": ekf_config,
                    **{name: LaunchConfiguration(name) for name in dynamics_arguments},
                }.items(),
            ),
            Node(
                package="sanitation_safety",
                executable="velocity_gate",
                name="calibration_velocity_gate",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": "/cmd_vel_gate",
                        "command_timeout_sec": 0.5,
                        "profile_name": operational_profile,
                        "max_linear_velocity": ParameterValue(max_linear_velocity, value_type=float),
                        "max_angular_velocity": ParameterValue(max_angular_velocity, value_type=float),
                    }
                ],
            ),
        ]
    )
