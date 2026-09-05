"""Launch one truth-free physical charge/drain service acceptance episode."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    scenario = LaunchConfiguration('scenario')
    output = LaunchConfiguration('output')
    vehicle_model = LaunchConfiguration('vehicle_model')
    station_x_offset = LaunchConfiguration('station_x_offset')
    vehicle_share = get_package_share_directory('sanitation_vehicle_description')
    acceptance_share = get_package_share_directory('sanitation_service_acceptance')

    formal_vehicle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{vehicle_share}/launch/formal_vehicle_sim.launch.py'
        ),
        launch_arguments={
            'gui': 'false',
            'start_controllers': 'true',
            'enable_safety_manager': 'true',
            'start_service_drain_safety_manager': 'true',
            'start_simulation_safety_inputs': 'true',
            # The service gate requires both nodes explicitly; do not rely on
            # the vehicle launch file coupling their defaults.
            'start_power_system_simulators': 'true',
            'high_bandwidth_sensor_runtime': 'false',
            'model': vehicle_model,
        }.items(),
    )
    station = Node(
        package='ros_gz_sim',
        executable='create',
        name='formal_physical_service_station_spawner',
        arguments=[
            '-world',
            'formal_vehicle_validation',
            '-file',
            f'{acceptance_share}/models/formal_service_station.sdf',
            '-name',
            'formal_physical_service_station',
            '-x',
            station_x_offset,
        ],
        output='screen',
    )
    evaluation_joint_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='service_acceptance_joint_command_bridge',
        arguments=[
            '/formal_vehicle/evaluation/service/charge_door_position_rad'
            '@std_msgs/msg/Float64]gz.msgs.Double',
            '/formal_vehicle/evaluation/service/charge_lock_position_m'
            '@std_msgs/msg/Float64]gz.msgs.Double',
            '/formal_vehicle/evaluation/service/drain_cap_position_rad'
            '@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen',
    )
    collector = Node(
        package='sanitation_service_acceptance',
        executable='formal_service_acceptance_collector',
        name='formal_service_acceptance_collector',
        arguments=['--scenario', scenario, '--output', output],
        output='screen',
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('scenario'),
            DeclareLaunchArgument('output'),
            DeclareLaunchArgument('vehicle_model'),
            DeclareLaunchArgument('station_x_offset', default_value='0.0'),
            formal_vehicle,
            evaluation_joint_bridge,
            TimerAction(period=6.0, actions=[station]),
            TimerAction(period=12.0, actions=[collector]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=collector,
                    on_exit=[EmitEvent(event=Shutdown(reason='service episode complete'))],
                )
            ),
        ]
    )
