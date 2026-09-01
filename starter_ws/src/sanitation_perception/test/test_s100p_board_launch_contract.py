"""Static contract for the real S100P product launch graph."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / "launch" / "formal_s100p_open_vocab.launch.py"


def test_s100p_launch_uses_real_official_parameter_names_and_project_topics():
    source = LAUNCH.read_text(encoding="utf-8")
    for node_name in (
        "rgb_to_nv12_adapter",
        "hobot_dosod",
        "mono_edgesam",
        "open_vocab_product_adapter",
    ):
        assert f'name="{node_name}"' in source
    for parameter in (
        "model_file_name",
        "vocabulary_file_name",
        "artifact_manifest_path",
        "ros_img_sub_topic_name",
        "ai_msg_pub_topic_name",
        "ai_msg_sub_topic_name",
        "encoder_model_file_name",
        "decoder_model_file_name",
        "is_shared_mem_sub",
    ):
        assert f'"{parameter}"' in source
    for invalid_placeholder in (
        '"image_topic"',
        '"model_path"',
        '"input_targets_topic"',
        '"encoder_model_path"',
        '"decoder_model_path"',
    ):
        assert invalid_placeholder not in source
    assert '"is_shared_mem_sub": 0' in source
    for topic in (
        "/perception/open_vocab/dosod_raw",
        "/perception/open_vocab/edgesam_prompts",
        "/perception/open_vocab/edgesam_raw",
        "/perception/garbage/detections_2d",
        "/perception/ground_dirt/masks",
        "/perception/garbage/targets",
        "/perception/open_vocab/diagnostics",
        "/perception/open_vocab/front_nv12",
    ):
        assert topic in source


def test_s100p_adapter_is_installed_with_ai_msgs_runtime_dependency():
    setup = (PACKAGE / "setup.py").read_text(encoding="utf-8")
    package_xml = (PACKAGE / "package.xml").read_text(encoding="utf-8")
    assert (
        "open_vocab_product_adapter = "
        "sanitation_perception.s100p_product_adapter:main"
    ) in setup
    assert "rgb_to_nv12_adapter = sanitation_perception.rgb_to_nv12_adapter:main" in setup
    assert "<exec_depend>ai_msgs</exec_depend>" in package_xml


def test_s100p_bpu_nodes_consume_the_validated_nv12_topic():
    source = LAUNCH.read_text(encoding="utf-8")
    assert source.count('"ros_img_sub_topic_name": front_nv12_topic') == 2
    assert '"input_topic": front_rgb_topic' in source
    assert '"output_topic": front_nv12_topic' in source


def test_s100p_adapter_receives_the_frozen_board_artifact_manifest():
    source = LAUNCH.read_text(encoding="utf-8")
    assert 'LaunchConfiguration("artifact_manifest_path")' in source
    assert 'DeclareLaunchArgument(\n                "artifact_manifest_path"' in source
    assert '"artifact_manifest_path": artifact_manifest_path' in source
