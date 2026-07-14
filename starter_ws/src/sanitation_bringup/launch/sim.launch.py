from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    upstream_launch = PathJoinSubstitution(
        [FindPackageShare("linorobot2_gazebo"), "launch", "gazebo.launch.py"]
    )
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("sanitation_vehicle_description"), "urdf", "sanitation_vehicle.urdf.xacro"]
    )
    world_path = PathJoinSubstitution(
        [FindPackageShare("sanitation_worlds"), "worlds", "sanitation_test_world.sdf"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("spawn_x", default_value="-8.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),

        # Linorobot2 launch evaluates this environment variable.
        SetEnvironmentVariable("LINOROBOT2_BASE", "4wd"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upstream_launch),
            launch_arguments={
                "gui": gui,
                "urdf": urdf_path,
                "world_path": world_path,
                "spawn_x": spawn_x,
                "spawn_y": spawn_y,
                "spawn_z": "0.05",
                "spawn_yaw": spawn_yaw,
                "odom_topic": "/odom",
            }.items(),
        ),
    ])
