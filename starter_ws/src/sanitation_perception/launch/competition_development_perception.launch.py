"""Camera-only Stage5B development smoke; no truth publisher or accuracy claim."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path'),
        DeclareLaunchArgument('policy_file', default_value=PathJoinSubstitution([
            FindPackageShare('sanitation_perception'), 'config', 'competition_candidate.yaml'])),
        Node(
            package='sanitation_perception', executable='garbage_perception_node',
            name='competition_development_perception', output='screen',
            parameters=[LaunchConfiguration('policy_file'), {
                'use_sim_time': True,
                'backend': 'onnxruntime',
                'model_path': LaunchConfiguration('model_path'),
                'model_id': 'stage5b_learned_perception_v1',
                'model_scope': 'D1_development_model_on_live_Gazebo_RGBD_no_accuracy_claim',
                'learned_weights': True,
            }],
            remappings=[
                ('/camera/color/image_raw', '/sensors/front_rgbd/depth/image_rect_raw/image'),
                ('/camera/depth/image_rect_raw', '/sensors/front_rgbd/depth/image_rect_raw/depth_image'),
                ('/camera/color/camera_info', '/sensors/front_rgbd/depth/image_rect_raw/camera_info'),
            ],
        ),
    ])
