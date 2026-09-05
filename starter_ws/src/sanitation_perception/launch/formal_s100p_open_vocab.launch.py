"""Launch the fail-closed RDK S100P DOSOD + EdgeSAM product graph.

The BPU packages exchange ``ai_msgs/PerceptionTargets`` internally.  The
project adapter deliberately consumes only that public ROS message plus the
raw (non shared-memory) camera/depth/map/TF inputs, and is the sole publisher
of the product-facing ``vision_msgs`` output.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    front_rgb_topic = LaunchConfiguration("front_rgb_topic")
    front_depth_topic = LaunchConfiguration("front_depth_topic")
    front_camera_info_topic = LaunchConfiguration("front_camera_info_topic")
    front_nv12_topic = LaunchConfiguration("front_nv12_topic")
    map_topic = LaunchConfiguration("map_topic")
    dosod_model_path = LaunchConfiguration("dosod_model_path")
    dosod_vocabulary_path = LaunchConfiguration("dosod_vocabulary_path")
    artifact_manifest_path = LaunchConfiguration("artifact_manifest_path")
    edgesam_encoder_model_path = LaunchConfiguration("edgesam_encoder_model_path")
    edgesam_decoder_model_path = LaunchConfiguration("edgesam_decoder_model_path")
    dosod_targets_topic = LaunchConfiguration("dosod_targets_topic")
    edgesam_targets_topic = LaunchConfiguration("edgesam_targets_topic")
    edgesam_prompts_topic = LaunchConfiguration("edgesam_prompts_topic")
    product_detections_topic = LaunchConfiguration("product_detections_topic")
    product_boxes_topic = LaunchConfiguration("product_boxes_topic")
    product_masks_topic = LaunchConfiguration("product_masks_topic")
    product_targets_topic = LaunchConfiguration("product_targets_topic")
    diagnostics_topic = LaunchConfiguration("diagnostics_topic")
    edgesam_capture_width = LaunchConfiguration("edgesam_capture_width")
    edgesam_capture_height = LaunchConfiguration("edgesam_capture_height")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "front_rgb_topic",
                default_value="/sensors/front_rgbd/depth/image_rect_raw/image",
                description="Formal ROS sensor_msgs/Image RGB topic; shared-memory transport is forbidden.",
            ),
            DeclareLaunchArgument(
                "front_depth_topic",
                default_value="/sensors/front_rgbd/depth/image_rect_raw/depth_image",
                description="Formal ROS sensor_msgs/Image depth topic.",
            ),
            DeclareLaunchArgument(
                "front_camera_info_topic",
                default_value="/sensors/front_rgbd/depth/image_rect_raw/camera_info",
                description="Formal ROS sensor_msgs/CameraInfo topic.",
            ),
            DeclareLaunchArgument(
                "front_nv12_topic",
                default_value="/perception/open_vocab/front_nv12",
                description="Validated NV12 sensor_msgs/Image topic consumed only by mono_edgesam.",
            ),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument(
                "dosod_model_path",
                description="Absolute verified project DOSOD .hbm model path on the S100P.",
            ),
            DeclareLaunchArgument(
                "dosod_vocabulary_path",
                description="Absolute verified project DOSOD vocabulary path on the S100P.",
            ),
            DeclareLaunchArgument(
                "artifact_manifest_path",
                description="Absolute frozen artifact_manifest.json bound to all four S100P board artifacts.",
            ),
            DeclareLaunchArgument(
                "edgesam_encoder_model_path",
                description="Absolute verified project EdgeSAM encoder .hbm model path on the S100P.",
            ),
            DeclareLaunchArgument(
                "edgesam_decoder_model_path",
                description="Absolute verified project EdgeSAM decoder .hbm model path on the S100P.",
            ),
            DeclareLaunchArgument(
                "dosod_targets_topic",
                default_value="/perception/open_vocab/dosod_raw",
                description="Raw ai_msgs/PerceptionTargets from hobot_dosod.",
            ),
            DeclareLaunchArgument(
                "edgesam_prompts_topic",
                default_value="/perception/open_vocab/edgesam_prompts",
                description="Validated ground-dirt prompts from the product adapter.",
            ),
            DeclareLaunchArgument(
                "edgesam_targets_topic",
                default_value="/perception/open_vocab/edgesam_raw",
                description="Raw ai_msgs/PerceptionTargets from mono_edgesam.",
            ),
            DeclareLaunchArgument(
                "product_detections_topic",
                default_value="/perception/garbage/detections_2d",
                description="Product vision_msgs/Detection2DArray topic.",
            ),
            DeclareLaunchArgument(
                "product_boxes_topic",
                default_value="/perception/open_vocab/dosod_boxes",
            ),
            DeclareLaunchArgument(
                "product_masks_topic",
                default_value="/perception/ground_dirt/masks",
            ),
            DeclareLaunchArgument(
                "product_targets_topic",
                default_value="/perception/garbage/targets",
            ),
            DeclareLaunchArgument(
                "diagnostics_topic",
                default_value="/perception/open_vocab/diagnostics",
            ),
            DeclareLaunchArgument("edgesam_capture_width", default_value="512"),
            DeclareLaunchArgument("edgesam_capture_height", default_value="288"),
            Node(
                package="sanitation_perception",
                executable="rgb_to_nv12_adapter",
                name="rgb_to_nv12_adapter",
                output="screen",
                parameters=[
                    {
                        "input_topic": front_rgb_topic,
                        "output_topic": front_nv12_topic,
                        "diagnostics_topic": diagnostics_topic,
                    }
                ],
            ),
            Node(
                package="hobot_dosod",
                executable="hobot_dosod",
                name="hobot_dosod",
                output="screen",
                parameters=[
                    {
                        "feed_type": 1,
                        "is_shared_mem_sub": 0,
                        "ros_img_sub_topic_name": front_nv12_topic,
                        "ai_msg_pub_topic_name": dosod_targets_topic,
                        "model_file_name": dosod_model_path,
                        "vocabulary_file_name": dosod_vocabulary_path,
                        "roi": False,
                        "trigger_mode": 0,
                        "class_mode": 0,
                    }
                ],
            ),
            Node(
                package="mono_edgesam",
                executable="mono_edgesam",
                name="mono_edgesam",
                output="screen",
                parameters=[
                    {
                        "feed_type": 1,
                        "is_regular_box": 0,
                        "is_shared_mem_sub": 0,
                        "ros_img_sub_topic_name": front_nv12_topic,
                        "ai_msg_sub_topic_name": edgesam_prompts_topic,
                        "ai_msg_pub_topic_name": edgesam_targets_topic,
                        "encoder_model_file_name": edgesam_encoder_model_path,
                        "decoder_model_file_name": edgesam_decoder_model_path,
                    }
                ],
            ),
            Node(
                package="sanitation_perception",
                executable="open_vocab_product_adapter",
                name="open_vocab_product_adapter",
                output="screen",
                parameters=[
                    {
                        "dosod_raw_topic": dosod_targets_topic,
                        "edgesam_prompts_topic": edgesam_prompts_topic,
                        "edgesam_raw_topic": edgesam_targets_topic,
                        "front_rgb_topic": front_rgb_topic,
                        "front_depth_topic": front_depth_topic,
                        "front_camera_info_topic": front_camera_info_topic,
                        "map_topic": map_topic,
                        "product_detections_topic": product_detections_topic,
                        "product_boxes_topic": product_boxes_topic,
                        "product_masks_topic": product_masks_topic,
                        "product_targets_topic": product_targets_topic,
                        "diagnostics_topic": diagnostics_topic,
                        "dosod_model_path": dosod_model_path,
                        "dosod_vocabulary_path": dosod_vocabulary_path,
                        "artifact_manifest_path": artifact_manifest_path,
                        "edgesam_encoder_model_path": edgesam_encoder_model_path,
                        "edgesam_decoder_model_path": edgesam_decoder_model_path,
                        "edgesam_capture_width": edgesam_capture_width,
                        "edgesam_capture_height": edgesam_capture_height,
                        "use_sim_time": False,
                    }
                ],
            ),
        ]
    )
