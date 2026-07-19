from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("model_path"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="sanitation_ground_truth",
                executable="garbage_ground_truth_node",
                name="garbage_ground_truth",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="sanitation_perception",
                executable="garbage_perception_node",
                name="garbage_perception",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"backend": "onnxruntime"},
                    {"model_path": model_path},
                    {"model_id": "stage5b_learned_perception_v1"},
                    {"model_scope": "D1_procedural_rendered_not_gazebo_camera"},
                    {"learned_weights": True},
                ],
            ),
            Node(
                package="sanitation_spot_cleaning",
                executable="spot_cleaning_node",
                name="spot_cleaning_coordinator",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, {"mode": "deferred"}],
            ),
        ]
    )
