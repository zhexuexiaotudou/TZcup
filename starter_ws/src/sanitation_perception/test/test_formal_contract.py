import hashlib
import json
from pathlib import Path

import pytest
import yaml

import sanitation_perception.formal_contract as formal_contract
from sanitation_perception.formal_contract import (
    _launch_node_binds_config,
    audit_formal_perception,
)


PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parents[2]
CONTRACT = PACKAGE / "config" / "formal_open_vocab_perception.yaml"
FORMAL_LAUNCH = (
    REPOSITORY
    / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
)
HIGH_BANDWIDTH_CONFIG = (
    REPOSITORY
    / "starter_ws/src/sanitation_vehicle_description/config/"
    / "formal_high_bandwidth_sensor_bridge.yaml"
)
FISHEYE_PUBLISHER = (
    REPOSITORY
    / "starter_ws/src/sanitation_vehicle_description/scripts/"
    / "formal_fisheye_camera_info_publisher.py"
)
S100_PROFILE = PACKAGE / "config" / "open_vocab_s100_profile.yaml"
S100_PREFLIGHT_LAUNCH = PACKAGE / "launch" / "formal_perception_preflight.launch.py"


def _write_artifacts(root: Path, platform: str) -> None:
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    rows = {}
    for component in contract["platforms"][platform]["components"].values():
        for relative in component["artifacts"]:
            artifact = root / relative
            artifact.parent.mkdir(parents=True, exist_ok=True)
            artifact.write_bytes(relative.encode("utf-8"))
            rows[relative] = {
                "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
                "byte_size": artifact.stat().st_size,
                "source_revision": component["revision"],
                "model_role": relative.split("/", 1)[0],
            }
    (root / "artifact_manifest.json").write_text(
        json.dumps({"artifacts": rows}), encoding="utf-8"
    )


def _write_minimal_sensor_repository(
    root: Path,
    *,
    bridge_rows: list[dict],
    launch_suffix: str = "",
    launch_text: str | None = None,
) -> None:
    launch = (
        root
        / "starter_ws/src/sanitation_vehicle_description/launch/"
        / "formal_vehicle_sim.launch.py"
    )
    bridge = (
        root
        / "starter_ws/src/sanitation_vehicle_description/config/"
        / "formal_high_bandwidth_sensor_bridge.yaml"
    )
    publisher = (
        root
        / "starter_ws/src/sanitation_vehicle_description/scripts/"
        / "formal_fisheye_camera_info_publisher.py"
    )
    for target in (launch, bridge, publisher):
        target.parent.mkdir(parents=True, exist_ok=True)
    launch.write_text(
        (FORMAL_LAUNCH.read_text(encoding="utf-8") if launch_text is None else launch_text)
        + launch_suffix,
        encoding="utf-8",
    )
    bridge.write_text(yaml.safe_dump(bridge_rows, sort_keys=False), encoding="utf-8")
    publisher.write_text(FISHEYE_PUBLISHER.read_text(encoding="utf-8"), encoding="utf-8")


def test_real_repository_is_blocked_without_models_but_has_product_adapters():
    report = audit_formal_perception(
        CONTRACT, repository_root=REPOSITORY, platform="pc", artifact_root=None
    )
    assert report["ready"] is False
    assert report["ground_truth_input_used"] is False
    assert report["checks"]["formal_high_bandwidth_bridge_config_valid"] is True
    assert report["checks"]["formal_vehicle_sensor_bridges_declared"] is True
    assert "artifact_root_missing" in report["blockers"]
    assert "pc_dosod_runtime_adapter_missing" not in report["blockers"]
    assert "pc_edgesam_runtime_adapter_missing" not in report["blockers"]
    assert report["checks"]["pc_product_adapter_present"] is True
    assert report["checks"]["pc_product_adapter_excludes_truth_inputs"] is True
    assert report["checks"]["pc_python_runtime_contract_valid"] is True
    assert report["checks"]["pc_python_version_supported"] is False
    assert "pc_python_version_unsupported" in report["blockers"]
    assert report["checks"]["pc_opencv_headless_declared"] is True
    assert isinstance(report["checks"]["pc_opencv_importable"], bool)
    assert ("pc_opencv_unavailable" in report["blockers"]) is (
        not report["checks"]["pc_opencv_importable"]
    )
    assert report["checks"]["area_target_planner_consumer_implemented"] is True
    assert "area_target_planner_consumer_missing" not in report["blockers"]
    assert report["legacy_repository_models"]


def test_s100_records_official_support_but_reports_project_artifact_blockers():
    report = audit_formal_perception(
        CONTRACT, repository_root=REPOSITORY, platform="rdk_s100", artifact_root=None
    )
    assert report["ready"] is False
    assert report["checks"]["dosod_official_s100_support"] is True
    assert "dosod_official_s100_support_unverified" not in report["blockers"]
    assert report["checks"]["dosod_runtime_adapter_present"] is True
    assert report["checks"]["edgesam_runtime_adapter_present"] is True
    assert "rdk_s100_dosod_runtime_adapter_missing" not in report["blockers"]
    assert "rdk_s100_edgesam_runtime_adapter_missing" not in report["blockers"]
    assert "artifact_missing:dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm" in report["blockers"]
    assert "artifact_missing:edgesam/edgesam_encoder_512.hbm" in report["blockers"]


def test_s100_artifact_provenance_uses_model_source_not_ros_wrapper_revision(tmp_path):
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    vocabulary = tmp_path / "dosod/tzcup_offline_vocabulary.json"
    vocabulary.parent.mkdir(parents=True)
    vocabulary.write_text('{"labels": ["bottle"]}\n', encoding="utf-8")
    row = {
        "sha256": __import__("hashlib").sha256(vocabulary.read_bytes()).hexdigest(),
        "byte_size": vocabulary.stat().st_size,
        "source_revision": contract["platforms"]["rdk_s100"]["components"]["dosod"]
        ["artifact_source_revisions"]["dosod/tzcup_offline_vocabulary.json"],
        "model_role": "frozen_project_prompt_vocabulary",
    }
    (tmp_path / "artifact_manifest.json").write_text(
        __import__("json").dumps({"schema_version": 1, "artifacts": {"dosod/tzcup_offline_vocabulary.json": row}}),
        encoding="utf-8",
    )
    report = audit_formal_perception(
        CONTRACT, repository_root=REPOSITORY, platform="rdk_s100", artifact_root=tmp_path
    )
    assert report["artifact_results"]["dosod/tzcup_offline_vocabulary.json"][
        "source_revision_matches"
    ] is True
    assert "artifact_provenance_mismatch:dosod/tzcup_offline_vocabulary.json" not in report["blockers"]


def test_s100p_board_first_contract_keeps_official_examples_and_four_project_roles_blocked():
    profile = yaml.safe_load(S100_PROFILE.read_text(encoding="utf-8"))
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    profile_board_first = profile["board_first_execution"]
    graph = contract["s100p_product_adapter_graph_contract"]

    assert profile_board_first == {
        "target_board": "RDK S100P",
        "target_soc": "Journey 6P",
        "pc_is_not_a_deployment_prerequisite": True,
        "real_board_execution_required_before_deployment_claim": True,
        "actuator_control_allowed": False,
        "note": profile_board_first["note"],
    }
    assert contract["hardware_identity"]["board_first_execution"] is True
    assert contract["s100p_board_first_contract"]["board_preflight_launch"] == {
        "launch_file": "formal_perception_preflight.launch.py",
        "required_platform_argument": "rdk_s100",
        "preflight_only": True,
    }
    assert graph["status"] == (
        "blocked_until_project_artifacts_adapter_and_live_board_graph_exist"
    )
    assert graph["official_examples_are_linkage_evidence_only"] is True
    assert graph["selected_edgesam_variant"] == "edgesam_512"
    assert graph["required_nodes"] == [
        "hobot_dosod", "mono_edgesam", "open_vocab_product_adapter"
    ]
    assert graph["required_internal_ai_msgs_edge"] == {
        "from_node": "hobot_dosod",
        "to_node": "mono_edgesam",
        "message_type": "ai_msgs/msg/PerceptionTargets",
    }
    assert graph["required_model_roles"] == {
        "dosod_hbm": "dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm",
        "dosod_vocabulary": "dosod/tzcup_offline_vocabulary.json",
        "edgesam_encoder_hbm": "edgesam/edgesam_encoder_512.hbm",
        "edgesam_decoder_hbm": "edgesam/edgesam_decoder_512.hbm",
    }
    assert profile["project_board_artifacts"]["required_model_roles"] == graph[
        "required_model_roles"
    ]


def test_s100p_preflight_launch_exposes_platform_but_cannot_masquerade_as_product_graph():
    launch = S100_PREFLIGHT_LAUNCH.read_text(encoding="utf-8")
    graph = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))["s100p_product_adapter_graph_contract"]

    assert 'DeclareLaunchArgument("platform", default_value="pc")' in launch
    assert '"--platform", LaunchConfiguration("platform")' in launch
    assert "formal_perception_preflight" in launch
    assert "open_vocab_product_adapter" not in launch
    assert graph["status"].startswith("blocked_until_")


def test_artifacts_must_have_matching_provenance(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr(formal_contract, "import_module", lambda name: object())
    monkeypatch.setattr(formal_contract, "_runtime_python_version", lambda: (3, 12))
    _write_artifacts(tmp_path, "pc")
    report = audit_formal_perception(
        CONTRACT, repository_root=REPOSITORY, platform="pc", artifact_root=tmp_path
    )
    assert report["checks"]["artifact_manifest_valid"] is True
    assert report["checks"]["formal_high_bandwidth_bridge_config_valid"] is True
    assert all(
        row["sha256_matches"] and row["byte_size_matches"] and row["source_revision_matches"]
        for row in report["artifact_results"].values()
    )
    assert report["ready"] is True


def test_pc_python_contract_allows_3_12_and_rejects_3_13(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(formal_contract, "_runtime_python_version", lambda: (3, 12))
    allowed = audit_formal_perception(
        CONTRACT, repository_root=REPOSITORY, platform="pc", artifact_root=None
    )
    assert allowed["checks"]["pc_python_runtime_contract_valid"] is True
    assert allowed["checks"]["pc_python_version_supported"] is True
    assert "pc_python_version_unsupported" not in allowed["blockers"]

    monkeypatch.setattr(formal_contract, "_runtime_python_version", lambda: (3, 13))
    rejected = audit_formal_perception(
        CONTRACT, repository_root=REPOSITORY, platform="pc", artifact_root=None
    )
    assert rejected["checks"]["pc_python_version_supported"] is False
    assert "pc_python_version_unsupported" in rejected["blockers"]


def test_pc_preflight_fails_closed_when_opencv_cannot_import(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def unavailable(name: str):
        assert name == "cv2"
        raise ImportError("cv2 is unavailable")

    monkeypatch.setattr(formal_contract, "import_module", unavailable)
    report = audit_formal_perception(
        CONTRACT, repository_root=REPOSITORY, platform="pc", artifact_root=None
    )
    assert report["checks"]["pc_opencv_headless_declared"] is True
    assert report["checks"]["pc_opencv_importable"] is False
    assert "pc_opencv_unavailable" in report["blockers"]


@pytest.mark.parametrize(
    ("field", "invalid_value"),
    (
        ("lazy", False),
        ("qos_profile", "DEFAULT"),
        ("subscriber_queue", 10),
        ("publisher_queue", 10),
        ("ros_type_name", ""),
    ),
)
def test_high_bandwidth_bridge_config_fails_closed(
    tmp_path: Path, field: str, invalid_value: object
):
    rows = yaml.safe_load(HIGH_BANDWIDTH_CONFIG.read_text(encoding="utf-8"))
    rows[0][field] = invalid_value
    _write_minimal_sensor_repository(tmp_path, bridge_rows=rows)
    report = audit_formal_perception(
        CONTRACT, repository_root=tmp_path, platform="pc", artifact_root=None
    )
    assert report["checks"]["formal_high_bandwidth_bridge_config_valid"] is False
    assert "formal_high_bandwidth_bridge_config_invalid" in report["blockers"]


@pytest.mark.parametrize(
    ("topic", "qos_profile"),
    (
        ("/sensors/lidar_3d/points", "SENSOR_DATA"),
        ("/sensors/front_rgbd/depth/image_rect_raw/image", "SYSTEM_DEFAULT"),
    ),
)
def test_high_bandwidth_bridge_qos_is_exactly_topic_bound(
    tmp_path: Path, topic: str, qos_profile: str
):
    rows = yaml.safe_load(HIGH_BANDWIDTH_CONFIG.read_text(encoding="utf-8"))
    for row in rows:
        if row["ros_topic_name"] == topic:
            row["qos_profile"] = qos_profile
            break
    else:
        pytest.fail(f"missing high-bandwidth topic fixture: {topic}")
    _write_minimal_sensor_repository(tmp_path, bridge_rows=rows)
    report = audit_formal_perception(
        CONTRACT, repository_root=tmp_path, platform="pc", artifact_root=None
    )
    assert report["checks"]["formal_high_bandwidth_bridge_config_valid"] is False
    assert "formal_high_bandwidth_bridge_config_invalid" in report["blockers"]


def test_sensor_topic_prefix_cannot_masquerade_as_exact_bridge(tmp_path: Path):
    rows = yaml.safe_load(HIGH_BANDWIDTH_CONFIG.read_text(encoding="utf-8"))
    required_topic = "/sensors/front_rgbd/depth/image_rect_raw/image"
    for row in rows:
        if row["ros_topic_name"] == required_topic:
            row["ros_topic_name"] += "_bad"
            break
    _write_minimal_sensor_repository(tmp_path, bridge_rows=rows)
    report = audit_formal_perception(
        CONTRACT, repository_root=tmp_path, platform="pc", artifact_root=None
    )
    assert required_topic in report["missing_lazy_formal_sensor_topics"]
    assert required_topic in report["missing_formal_sensor_bridges"]


def test_required_sensor_topic_cannot_move_back_to_eager_bridge(tmp_path: Path):
    rows = yaml.safe_load(HIGH_BANDWIDTH_CONFIG.read_text(encoding="utf-8"))
    required_topic = "/sensors/front_rgbd/depth/image_rect_raw/image"
    rows = [row for row in rows if row["ros_topic_name"] != required_topic]
    _write_minimal_sensor_repository(
        tmp_path,
        bridge_rows=rows,
        launch_suffix=(
            f'\nEAGER_REGRESSION = "{required_topic}'
            '@sensor_msgs/msg/Image[gz.msgs.Image"\n'
        ),
    )
    report = audit_formal_perception(
        CONTRACT, repository_root=tmp_path, platform="pc", artifact_root=None
    )
    assert required_topic in report["missing_lazy_formal_sensor_topics"]
    assert required_topic in report["eager_high_bandwidth_sensor_topics"]
    assert "formal_high_bandwidth_sensor_topic_eagerly_duplicated" in report["blockers"]


def test_duplicate_high_bandwidth_topics_fail_closed(tmp_path: Path):
    rows = yaml.safe_load(HIGH_BANDWIDTH_CONFIG.read_text(encoding="utf-8"))
    rows.append(dict(rows[0]))
    _write_minimal_sensor_repository(tmp_path, bridge_rows=rows)
    report = audit_formal_perception(
        CONTRACT, repository_root=tmp_path, platform="pc", artifact_root=None
    )
    assert report["checks"]["formal_high_bandwidth_bridge_config_valid"] is False
    assert report["high_bandwidth_duplicate_ros_topics"] == [rows[0]["ros_topic_name"]]
    assert report["high_bandwidth_duplicate_gz_topics"] == [rows[0]["gz_topic_name"]]


def test_bridge_config_filename_in_comment_cannot_masquerade_as_launch_binding(
    tmp_path: Path,
):
    rows = yaml.safe_load(HIGH_BANDWIDTH_CONFIG.read_text(encoding="utf-8"))
    _write_minimal_sensor_repository(
        tmp_path,
        bridge_rows=rows,
        launch_text=(
            "# formal_high_bandwidth_sensor_bridge.yaml\n"
            "# formal_vehicle_high_bandwidth_sensor_bridge\n"
            "# formal_fisheye_camera_info_publisher.py\n"
        ),
    )
    report = audit_formal_perception(
        CONTRACT, repository_root=tmp_path, platform="pc", artifact_root=None
    )
    assert (
        report["checks"]["formal_high_bandwidth_bridge_config_bound_in_launch"]
        is False
    )
    assert "formal_high_bandwidth_bridge_config_not_bound_in_launch" in report["blockers"]


def test_bridge_filename_outside_config_file_value_cannot_masquerade_as_binding():
    launch_text = """
Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    name="formal_vehicle_high_bandwidth_sensor_bridge",
    arguments=["formal_high_bandwidth_sensor_bridge.yaml"],
    parameters=[{"config_file": "wrong.yaml"}],
)
"""
    assert _launch_node_binds_config(
        launch_text,
        node_name="formal_vehicle_high_bandwidth_sensor_bridge",
        config_name="formal_high_bandwidth_sensor_bridge.yaml",
    ) is False


def test_all_formal_product_topics_exclude_evaluator_truth():
    report = audit_formal_perception(
        CONTRACT, repository_root=REPOSITORY, platform="pc", artifact_root=None
    )
    assert report["checks"]["product_topics_exclude_evaluator_truth"] is True
    assert report["checks"]["edgesam_not_misclaimed_as_detector"] is True
    assert report["checks"]["unobservable_material_class_not_inferred"] is True
    assert report["checks"]["project_prompt_vocabulary_frozen"] is True
    assert report["checks"]["all_formal_cameras_have_compute_schedule"] is True
    assert report["checks"]["product_output_types_complete"] is True
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    assert contract["product_outputs"]["wrist_grasp_recheck"] == (
        "/perception/wrist/grasp_recheck"
    )
    assert report["checks"]["s100_dosod_edgesam_ai_msgs_boundary_explicit"] is True
    assert report["checks"]["s100_and_journey6_same_project_platform"] is True


def test_ground_dirt_product_mask_requires_public_map_projection():
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    semantics = contract["ground_dirt_mask_semantics"]
    assert semantics["frame_id"] == "map"
    assert semantics["dimensions"] == "exact_public_occupancy_grid"
    assert semantics["values"] == {
        0: "unobserved",
        1: "observed_clean",
        "2_to_255": "dirty_confidence",
    }


def test_formal_launch_is_preflight_only_and_never_starts_truth_or_legacy_model():
    launch = (PACKAGE / "launch" / "formal_perception_preflight.launch.py").read_text(
        encoding="utf-8"
    )
    assert "formal_perception_preflight" in launch
    assert "garbage_ground_truth" not in launch
    assert "garbage_perception_node" not in launch
    assert "/ground_truth/" not in launch


def test_locked_source_manifest_matches_pc_and_s100_contracts():
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    sources = yaml.safe_load(
        (REPOSITORY / "repos" / "perception.repos").read_text(encoding="utf-8")
    )["repositories"]
    expected = {
        "dosod_pc": contract["platforms"]["pc"]["components"]["dosod"],
        "edgesam_pc": contract["platforms"]["pc"]["components"]["edgesam"],
        "hobot_dosod_s100": contract["platforms"]["rdk_s100"]["components"]["dosod"],
        "mono_edgesam_s100": contract["platforms"]["rdk_s100"]["components"]["edgesam"],
    }
    for name, component in expected.items():
        assert sources[name]["url"].removesuffix(".git") == component["source_url"]
        assert sources[name]["version"] == component["revision"]
