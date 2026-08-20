"""Ground-truth-isolated product perception and spot-cleaning control stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    pipeline_manifest = LaunchConfiguration("pipeline_manifest")
    artifact_root = LaunchConfiguration("artifact_root")
    mission_id = LaunchConfiguration("mission_id")
    dynamic_map_path = LaunchConfiguration("dynamic_map_path")
    resume_same_mission = LaunchConfiguration("resume_same_mission")
    cleanable_polygon_json = LaunchConfiguration("cleanable_polygon_json")
    observation_config = PathJoinSubstitution([
        FindPackageShare("sanitation_spot_cleaning"),
        "config",
        "product_observation_pose.yaml",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("pipeline_manifest"),
        DeclareLaunchArgument("artifact_root"),
        DeclareLaunchArgument("mission_id"),
        DeclareLaunchArgument("dynamic_map_path"),
        DeclareLaunchArgument("resume_same_mission", default_value="false"),
        DeclareLaunchArgument("cleanable_polygon_json"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="sanitation_perception",
            executable="product_perception_node",
            name="product_perception",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "pipeline_manifest": pipeline_manifest,
                "artifact_root": artifact_root,
                "mission_id": mission_id,
                "dynamic_map_path": dynamic_map_path,
                "resume_same_mission": resume_same_mission,
                "autostart": True,
                "keepout_mask_topic": "/keepout_filter_mask",
            }],
        ),
        Node(
            package="sanitation_spot_cleaning",
            executable="stage5br5_observation_pose_node",
            name="stage5br5_observation_pose_planner",
            output="screen",
            parameters=[
                observation_config,
                {
                    "use_sim_time": use_sim_time,
                    "cleanable_polygon_json": ParameterValue(
                        cleanable_polygon_json, value_type=str
                    ),
                },
            ],
        ),
        Node(
            package="sanitation_spot_cleaning",
            executable="product_reobservation_node",
            name="product_reobservation",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="sanitation_spot_cleaning",
            executable="spot_cleaning_node",
            name="product_spot_cleaning",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
