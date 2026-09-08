"""Launch the explicit non-formal RDK S100P development smoke graph.

The BPU packages exchange ``ai_msgs/PerceptionTargets`` internally.  The
development manifest classified ``NON_FORMAL_ABI_DEVELOPMENT``. It is
CHAIN_LIVENESS-only, never formal, semantic, or board acceptance. Inputs must
be an exact prepared 848x480 ``DERIVED_LIVENESS_INPUT`` RGB-D tuple,
CameraInfo, map, and static map-to-camera TF traffic.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    front_rgb_topic = LaunchConfiguration("front_rgb_topic")
    front_depth_topic = LaunchConfiguration("front_depth_topic")
    front_camera_info_topic = LaunchConfiguration("front_camera_info_topic")
    front_dosod_nv12_topic = LaunchConfiguration("front_dosod_nv12_topic")
    front_edgesam_nv12_topic = LaunchConfiguration("front_edgesam_nv12_topic")
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
                "front_dosod_nv12_topic",
                default_value="/perception/open_vocab/front_dosod_nv12",
                description="Selected original 848x480 packed NV12 input; official hobot_dosod owns pyramid preprocessing.",
            ),
            DeclareLaunchArgument(
                "front_edgesam_nv12_topic",
                default_value="/perception/open_vocab/front_edgesam_nv12",
                description="Selected original 848x480 packed NV12 input; official mono_edgesam ResizeNV12 produces its 512x288 network input.",
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
                        "dosod_output_topic": front_dosod_nv12_topic,
                        "edgesam_output_topic": front_edgesam_nv12_topic,
                        "diagnostics_topic": diagnostics_topic,
                    }
                ],
            ),
            Node(
                package="hobot_dosod",
                executable="hobot_dosod",
                name="hobot_dosod",
                output="screen",
                arguments=["--ros-args", "--log-level", "warn"],
                parameters=[
                    {
                        "feed_type": 1,
                        "is_shared_mem_sub": 0,
                        "ros_img_sub_topic_name": front_dosod_nv12_topic,
                        "ai_msg_pub_topic_name": dosod_targets_topic,
                        "model_file_name": dosod_model_path,
                        "vocabulary_file_name": dosod_vocabulary_path,
                        "roi": False,
                        "trigger_mode": 0,
                        "class_mode": 0,
                        "score_threshold": 0.002,
                        "iou_threshold": 0.65,
                        "nms_top_k": 300,
                    }
                ],
            ),
            Node(
                package="mono_edgesam",
                executable="mono_edgesam",
                name="mono_edgesam",
                output="screen",
                arguments=["--ros-args", "--log-level", "warn"],
                parameters=[
                    {
                        "feed_type": 1,
                        "is_regular_box": 0,
                        "is_padding_seg": 0,
                        "is_shared_mem_sub": 0,
                        "ros_img_sub_topic_name": front_edgesam_nv12_topic,
                        "ai_msg_sub_topic_name": edgesam_prompts_topic,
                        "ai_msg_pub_topic_name": edgesam_targets_topic,
                        "encoder_model_file_name": edgesam_encoder_model_path,
                        "decoder_model_file_name": edgesam_decoder_model_path,
                        "is_sync_mode": 0,
                        "cache_len_limit": 8,
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
                        "artifact_mode": "development",
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
