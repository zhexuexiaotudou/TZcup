from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "starter_ws" / "src" / "sanitation_bringup" / "launch" / "product_cleaning.launch.py"


def test_product_launch_uses_only_product_control_nodes() -> None:
    text = SOURCE.read_text(encoding="utf-8")
    assert 'executable="product_perception_node"' in text
    assert 'executable="spot_cleaning_node"' in text
    assert 'executable="stage5br5_observation_pose_node"' in text
    assert 'executable="product_reobservation_node"' in text
    assert "sanitation_ground_truth" not in text
    assert "garbage_ground_truth_node" not in text


def test_product_launch_requires_frozen_artifacts_and_mission_identity() -> None:
    text = SOURCE.read_text(encoding="utf-8")
    for required in (
        'DeclareLaunchArgument("pipeline_manifest")',
        'DeclareLaunchArgument("artifact_root")',
        'DeclareLaunchArgument("mission_id")',
        'DeclareLaunchArgument("dynamic_map_path")',
        'DeclareLaunchArgument("cleanable_polygon_json")',
    ):
        assert required in text
    assert '"autostart": True' in text
    assert '"keepout_mask_topic": "/keepout_filter_mask"' in text
