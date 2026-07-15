from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless_rendering")
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
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("headless_rendering", default_value="true"),
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
                    }
                ],
            ),
        ]
    )
