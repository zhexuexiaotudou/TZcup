"""Static contract for the real S100P product launch graph."""

import ast
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE.parents[2]
LAUNCH = PACKAGE / "launch" / "formal_s100p_open_vocab.launch.py"
DEVELOPMENT_LAUNCH = PACKAGE / "launch" / "development_s100p_open_vocab.launch.py"


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
        "/perception/open_vocab/front_dosod_nv12",
        "/perception/open_vocab/front_edgesam_nv12",
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
    assert '"numpy>=1.21.5,<2"' in setup
    assert '"numpy==1.26.4"' not in setup


def test_fast_validation_installs_setuptools_for_sdist_contract():
    workflow = (
        REPOSITORY_ROOT / ".github/workflows/development-workflow.yml"
    ).read_text(encoding="utf-8")
    assert "setuptools==80.9.0" in workflow


def test_s100p_bpu_nodes_consume_the_validated_nv12_topic():
    source = LAUNCH.read_text(encoding="utf-8")
    assert '"ros_img_sub_topic_name": front_dosod_nv12_topic' in source
    assert '"ros_img_sub_topic_name": front_edgesam_nv12_topic' in source
    assert '"input_topic": front_rgb_topic' in source
    assert '"dosod_output_topic": front_dosod_nv12_topic' in source
    assert '"edgesam_output_topic": front_edgesam_nv12_topic' in source


def test_s100p_launch_uses_frozen_official_prefilter_and_async_parameters():
    source = LAUNCH.read_text(encoding="utf-8")
    for value in (
        '"score_threshold": 0.002',
        '"iou_threshold": 0.65',
        '"nms_top_k": 300',
        '"is_sync_mode": 0',
        '"cache_len_limit": 8',
        '"is_padding_seg": 0',
        'arguments=["--ros-args", "--log-level", "warn"]',
    ):
        assert value in source


def test_s100p_projection_queries_tf_at_the_exact_rgb_header_stamp():
    source = (
        PACKAGE / "sanitation_perception" / "s100p_product_adapter.py"
    ).read_text(encoding="utf-8")
    assert "Time.from_msg(image.header.stamp)" in source
    assert "validate_exact_tf_binding(" in source
    assert 'lookup_transform(\n                str(self.get_parameter("map_frame").value),' in source


def test_only_the_nv12_adapter_selects_source_frames_and_product_cache_is_bounded():
    adapter = (PACKAGE / "sanitation_perception" / "rgb_to_nv12_adapter.py").read_text(encoding="utf-8")
    product = (PACKAGE / "sanitation_perception" / "s100p_product_adapter.py").read_text(encoding="utf-8")
    assert "SourceStampSelector" in adapter
    assert "SourceStampSelector" not in product
    assert 'self.declare_parameter("rgb_frame_cache_limit", 96)' in product
    assert "ExactStampRgbdCache" in product
    assert "DosodFrameTransform" not in product
    assert "inverse_s100p_dosod_detections" not in product
    assert "black-square" not in adapter


def test_s100p_short_diagnostic_contract_binds_dual_nv12_and_same_stamp_chain():
    profile = yaml.safe_load((PACKAGE / "config" / "open_vocab_s100_profile.yaml").read_text(encoding="utf-8"))
    contract = profile["board_short_diagnostic_contract"]
    assert contract["source_selector_hz"] == 2.0
    assert contract["required_nv12_outputs"] == {
        "dosod": {"width": 848, "height": 480, "encoding": "nv12", "packed": True},
        "edgesam": {"width": 848, "height": 480, "encoding": "nv12", "packed": True},
    }
    assert contract["required_same_stamp_chain"] == ["rgb", "dosod", "edgesam"]
    assert contract["required_processing_counters"] == [
        "selected_frames", "dosod_raw_frames", "product_box_frames", "product_target_frames", "reject_reasons"
    ]
    assert contract["official_preprocessing_contract"] == {
        "dosod": {"owner": "hobot_dosod", "operation": "GetNV12Pyramid", "source_dimensions": [848, 480]},
        "edgesam": {"owner": "mono_edgesam", "operation": "ResizeNV12Img", "source_dimensions": [848, 480], "network_dimensions": [512, 288]},
    }
    assert contract["dosod_raw_output_shape"] == [1, 8400, 4]
    diagnostics = contract["diagnostics_observation"]
    assert diagnostics["topic"] == "/perception/open_vocab/diagnostics"
    assert diagnostics["must_subscribe_and_receive"] is True
    assert diagnostics["node_liveness_only_is_insufficient"] is True
    assert diagnostics["nodes"] == {
        "formal_open_vocab_perception/rgb_to_nv12_adapter": {
            "required_healthy_level": 0,
            "required_error_propagation_level": 2,
        },
        "formal_open_vocab_perception/product_adapter": {
            "required_healthy_level": 0,
            "required_error_propagation_level": 2,
        },
    }


def test_s100p_diagnostic_status_level_supports_rclpy_uint8_generators():
    """Both deployed Python generators must accept the diagnostic-level helper."""
    for module in ("rgb_to_nv12_adapter.py", "s100p_product_adapter.py"):
        source = (PACKAGE / "sanitation_perception" / module).read_text(encoding="utf-8")
        assert "from .diagnostic_compat import set_diagnostic_level" in source
        assert "set_diagnostic_level(status, level)" in source
    compat = (PACKAGE / "sanitation_perception" / "diagnostic_compat.py").read_text(
        encoding="utf-8"
    )
    assert "status.level = value" in compat
    assert "status.level = bytes([value])" in compat


def test_s100p_adapter_receives_the_frozen_board_artifact_manifest():
    source = LAUNCH.read_text(encoding="utf-8")
    assert 'LaunchConfiguration("artifact_manifest_path")' in source
    assert 'DeclareLaunchArgument(\n                "artifact_manifest_path"' in source
    assert '"artifact_manifest_path": artifact_manifest_path' in source


def test_development_artifact_mode_is_explicit_and_cannot_leak_into_formal_launch():
    formal = LAUNCH.read_text(encoding="utf-8")
    development = DEVELOPMENT_LAUNCH.read_text(encoding="utf-8")
    adapter = (PACKAGE / "sanitation_perception" / "s100p_product_adapter.py").read_text(encoding="utf-8")
    assert "development" not in formal
    assert '"artifact_mode": "development"' in development
    assert "NON_FORMAL" in development
    assert '"artifact_mode": "formal"' in adapter
    assert "NON_FORMAL_ABI_DEVELOPMENT" in adapter
    assert "load_verified_board_artifact_contract(**artifact_kwargs)" in adapter
    assert "from .s100p_development_artifact_contract import" in adapter
    core = PACKAGE / "sanitation_perception" / "s100p_product_adapter_core.py"
    assert hashlib.sha256(core.read_bytes()).hexdigest() == "57c52aead86ebff5d10b4257cd08eb4e2a2ba64b50ebd39dab8ac55f613ce3fd"


def test_development_launch_is_ast_identical_to_formal_except_for_its_mode():
    class StripDevelopmentMode(ast.NodeTransformer):
        def visit_Dict(self, node):
            node = self.generic_visit(node)
            pairs = [
                (key, value)
                for key, value in zip(node.keys, node.values, strict=True)
                if not (isinstance(key, ast.Constant) and key.value == "artifact_mode")
            ]
            node.keys, node.values = [pair[0] for pair in pairs], [pair[1] for pair in pairs]
            return node

    formal, development = ast.parse(LAUNCH.read_text(encoding="utf-8")), ast.parse(DEVELOPMENT_LAUNCH.read_text(encoding="utf-8"))
    for tree in (formal, development):
        if isinstance(tree.body[0], ast.Expr) and isinstance(tree.body[0].value, ast.Constant):
            tree.body.pop(0)
    development = StripDevelopmentMode().visit(development)
    assert ast.dump(formal, include_attributes=False) == ast.dump(development, include_attributes=False)


def test_formal_bundle_rebind_is_limited_to_the_product_adapter_source_row():
    manifest = json.loads((REPOSITORY_ROOT / "config/s100p_formal_board_bundle_manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "COPYABLE_MANIFEST_ONLY_BLOCKED_DEPLOYMENT"
    assert manifest["copy_boundary"]["payload_copy_authorized"] is False
    rows = {row["role"]: row for row in manifest["bound_sources"]}
    assert set(rows) == {
        "dosod_hbm_compile_contract", "offline_predeploy_product_bundle", "board_launch_parameter_record",
        "board_overlay_package_contract", "dosod_edgesam_s100_profile", "formal_ros2_launch",
        "project_perception_package_manifest", "diagnostic_compat_source", "nv12_adapter_source",
        "product_adapter_source", "project_perception_entry_points", "perception_interfaces_package_manifest",
    }
    adapter = PACKAGE / "sanitation_perception" / "s100p_product_adapter.py"
    assert rows["product_adapter_source"] == {
        "path": "starter_ws/src/sanitation_perception/sanitation_perception/s100p_product_adapter.py",
        "byte_size": adapter.stat().st_size,
        "sha256": hashlib.sha256(adapter.read_bytes()).hexdigest(),
        "role": "product_adapter_source",
    }
    assert rows["formal_ros2_launch"]["sha256"] == "cbe73e72bb3dbb76131766ed6d241602a74b4dc1ccc4467c1ebc26454507a7c5"
    assert rows["dosod_hbm_compile_contract"]["sha256"] == "05a10c9d427a7836366f355e455dda96f3c7a1aa6d5aad2ca087d21065a3dd7d"
    assert rows["offline_predeploy_product_bundle"]["sha256"] == "b9719f78d0947cd6a834e111ecc9b3c3a3d33f4dde8915ca13885208bd62689e"


def test_s100p_packaged_board_configs_match_the_authoritative_root_records():
    for name in (
        "s100p_product_overlay_packages.json",
        "s100p_product_board_launch_parameters.json",
    ):
        authoritative = REPOSITORY_ROOT / "config" / name
        packaged = PACKAGE / "config" / name
        assert json.loads(packaged.read_text(encoding="utf-8")) == json.loads(
            authoritative.read_text(encoding="utf-8")
        )
        assert packaged.read_bytes() == authoritative.read_bytes()


def test_s100p_board_configs_survive_sdist_and_install_data(tmp_path: Path):
    """Exercise the install path that a board overlay receives, not source text."""
    def run_packaging(command: list[str], *, cwd: Path) -> None:
        result = subprocess.run(
            command,
            cwd=cwd,
            check=False,
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, (
            f"packaging command failed: {command!r}\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )

    source = tmp_path / "sanitation_perception"
    shutil.copytree(PACKAGE, source)
    dist = tmp_path / "dist"
    run_packaging(
        [sys.executable, "setup.py", "sdist", "--dist-dir", str(dist)],
        cwd=source,
    )
    archive = dist / "sanitation_perception-0.1.0.tar.gz"
    with tarfile.open(archive) as bundle:
        members = set(bundle.getnames())
        for name in (
            "s100p_product_overlay_packages.json",
            "s100p_product_board_launch_parameters.json",
        ):
            assert f"sanitation_perception-0.1.0/config/{name}" in members
        bundle.extractall(tmp_path / "unpacked", filter="data")
    unpacked = tmp_path / "unpacked" / "sanitation_perception-0.1.0"
    install_root = tmp_path / "installed"
    run_packaging(
        [
            sys.executable,
            "setup.py",
            "install",
            "--root",
            str(install_root),
            "--prefix",
            "/usr",
            "--single-version-externally-managed",
            "--record",
            str(tmp_path / "installed-files.txt"),
        ],
        cwd=unpacked,
    )
    installed_configs = list(install_root.glob("**/share/sanitation_perception/config"))
    assert len(installed_configs) == 1
    installed_config = installed_configs[0]
    for name in (
        "s100p_product_overlay_packages.json",
        "s100p_product_board_launch_parameters.json",
    ):
        assert (installed_config / name).read_bytes() == (PACKAGE / "config" / name).read_bytes()
