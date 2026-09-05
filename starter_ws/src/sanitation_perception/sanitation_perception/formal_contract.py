"""Fail-closed audit for formal DOSOD + EdgeSAM perception integration."""

from __future__ import annotations

import argparse
import ast
import hashlib
from importlib import import_module
import json
import re
import sys
from pathlib import Path
from typing import Any, Mapping

import yaml


FORBIDDEN_TOPIC_TOKENS = ("ground_truth", "evaluator", "evaluation/")
LEGACY_MODEL_IDS = (
    "stage5a_synthetic_color_prototype",
    "stage5b_learned_perception",
    "auto04_direct_detector",
    "auto05_direct_detector",
)
ROS_NATIVE_SENSOR_TOPICS = {
    "/sensors/rear_left_fisheye/camera_info",
    "/sensors/rear_right_fisheye/camera_info",
}

# These three high-bandwidth streams are explicitly bound to the bridge's
# reliable SYSTEM_DEFAULT profile.  Every other product sensor bridge remains
# on the bounded SENSOR_DATA profile.
RELIABLE_HIGH_BANDWIDTH_TOPICS = frozenset(
    {
        "/sensors/lidar_3d/points",
        "/sensors/rear_left_fisheye/image_raw",
        "/sensors/rear_right_fisheye/image_raw",
    }
)


class FormalPerceptionContractError(RuntimeError):
    """Raised when the product perception graph violates its frozen boundary."""


def _runtime_python_version() -> tuple[int, int]:
    """Return only the interpreter components governed by the PC contract."""

    return sys.version_info.major, sys.version_info.minor


def _version_pair(value: Any) -> tuple[int, int] | None:
    if (
        not isinstance(value, list)
        or len(value) != 2
        or any(isinstance(item, bool) or not isinstance(item, int) for item in value)
    ):
        return None
    return value[0], value[1]


def _load_yaml(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise FormalPerceptionContractError(f"contract is not a mapping: {path}")
    if payload.get("schema_version") != 1:
        raise FormalPerceptionContractError("formal perception schema_version must equal 1")
    return payload


def _topic_values(payload: Mapping[str, Any]) -> list[str]:
    values: list[str] = []
    for key, value in payload.items():
        if isinstance(value, Mapping):
            values.extend(_topic_values(value))
        elif isinstance(value, str) and ("topic" in key or value.startswith("/")):
            values.append(value)
    return values


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _launch_node_binds_config(
    launch_text: str, *, node_name: str, config_name: str
) -> bool:
    """Require the bridge name and config inside one real launch Node call."""

    try:
        tree = ast.parse(launch_text)
    except SyntaxError:
        return False
    for call in (
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
    ):
        keywords = {
            item.arg: item.value for item in call.keywords if item.arg is not None
        }
        expected_keywords = {
            "package": "ros_gz_bridge",
            "executable": "parameter_bridge",
            "name": node_name,
        }
        if any(
            not isinstance(keywords.get(key), ast.Constant)
            or keywords[key].value != expected
            for key, expected in expected_keywords.items()
        ):
            continue
        parameters = keywords.get("parameters")
        if parameters is None:
            continue
        for mapping in (
            node for node in ast.walk(parameters) if isinstance(node, ast.Dict)
        ):
            for key_node, value_node in zip(mapping.keys, mapping.values, strict=True):
                if not (
                    isinstance(key_node, ast.Constant)
                    and key_node.value == "config_file"
                ):
                    continue
                config_literals = {
                    node.value
                    for node in ast.walk(value_node)
                    if isinstance(node, ast.Constant)
                    and isinstance(node.value, str)
                }
                if config_name in config_literals:
                    return True
    return False


def _artifact_manifest(path: Path) -> tuple[dict[str, Any], str | None]:
    if not path.is_file():
        return {}, "artifact_manifest_missing"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {}, f"artifact_manifest_invalid:{exc.__class__.__name__}"
    if not isinstance(payload, dict) or not isinstance(payload.get("artifacts"), dict):
        return {}, "artifact_manifest_invalid_schema"
    return payload["artifacts"], None


def audit_formal_perception(
    contract_path: str | Path,
    *,
    repository_root: str | Path,
    platform: str,
    artifact_root: str | Path | None,
) -> dict[str, Any]:
    """Audit only real product sources and artifacts; never infer from truth files."""

    contract_path = Path(contract_path).resolve()
    repository_root = Path(repository_root).resolve()
    contract = _load_yaml(contract_path)
    platforms = contract.get("platforms", {})
    if platform not in platforms:
        raise FormalPerceptionContractError(f"unknown formal perception platform: {platform}")

    checks: dict[str, bool] = {}
    blockers: list[str] = []

    claim = contract.get("claim_boundary", {})
    checks["truth_control_disabled"] = claim.get("production_control_truth_allowed") is False
    checks["evaluator_topics_disabled"] = claim.get("evaluator_topics_allowed") is False
    checks["legacy_stage5_model_ineligible"] = claim.get("legacy_stage5_model_eligible") is False
    hardware_identity = contract.get("hardware_identity", {})
    checks["s100_and_journey6_same_project_platform"] = (
        hardware_identity.get("board") == "RDK S100P"
        and hardware_identity.get("soc") == "Journey 6P"
        and hardware_identity.get("same_project_platform") is True
    )

    topics = _topic_values(
        {
            "sensor_inputs": contract.get("sensor_inputs", {}),
            "product_outputs": contract.get("product_outputs", {}),
        }
    )
    forbidden_topics = sorted(
        topic for topic in topics if any(token in topic.lower() for token in FORBIDDEN_TOPIC_TOKENS)
    )
    checks["product_topics_exclude_evaluator_truth"] = not forbidden_topics
    if forbidden_topics:
        blockers.append("product_topic_truth_boundary_violation")

    formal_launch = repository_root / (
        "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    )
    formal_launch_text = formal_launch.read_text(encoding="utf-8") if formal_launch.is_file() else ""
    sensor_topics = _topic_values(contract.get("sensor_inputs", {}))
    required_lazy_sensor_topics = set(sensor_topics) - ROS_NATIVE_SENSOR_TOPICS
    inline_bridge_topics = set(
        re.findall(r"[\"'](/[^\"'@]+)@", formal_launch_text)
    )
    high_bandwidth_bridge_config = repository_root / (
        "starter_ws/src/sanitation_vehicle_description/config/"
        "formal_high_bandwidth_sensor_bridge.yaml"
    )
    high_bandwidth_bridge_config_valid = False
    high_bandwidth_bridge_topics: set[str] = set()
    high_bandwidth_duplicate_ros_topics: set[str] = set()
    high_bandwidth_duplicate_gz_topics: set[str] = set()
    high_bandwidth_bridge_bound = _launch_node_binds_config(
        formal_launch_text,
        node_name="formal_vehicle_high_bandwidth_sensor_bridge",
        config_name=high_bandwidth_bridge_config.name,
    )
    checks["formal_high_bandwidth_bridge_config_bound_in_launch"] = (
        high_bandwidth_bridge_bound
    )
    if not high_bandwidth_bridge_bound:
        blockers.append("formal_high_bandwidth_bridge_config_not_bound_in_launch")
    if high_bandwidth_bridge_bound and high_bandwidth_bridge_config.is_file():
        try:
            bridge_rows = yaml.safe_load(
                high_bandwidth_bridge_config.read_text(encoding="utf-8")
            )
        except (OSError, yaml.YAMLError):
            bridge_rows = None
        rows_have_valid_schema = isinstance(bridge_rows, list) and bool(bridge_rows) and all(
            isinstance(row, Mapping)
            and row.get("direction") == "GZ_TO_ROS"
            and row.get("lazy") is True
            and row.get("qos_profile")
            == (
                "SYSTEM_DEFAULT"
                if row.get("ros_topic_name") in RELIABLE_HIGH_BANDWIDTH_TOPICS
                else "SENSOR_DATA"
            )
            and type(row.get("subscriber_queue")) is int
            and row.get("subscriber_queue") == 1
            and type(row.get("publisher_queue")) is int
            and row.get("publisher_queue") == 1
            and all(
                isinstance(row.get(key), str) and bool(row[key].strip())
                for key in (
                    "ros_topic_name",
                    "gz_topic_name",
                    "ros_type_name",
                    "gz_type_name",
                )
            )
            for row in bridge_rows
        )
        if rows_have_valid_schema:
            ros_topic_rows = [str(row["ros_topic_name"]) for row in bridge_rows]
            gz_topic_rows = [str(row["gz_topic_name"]) for row in bridge_rows]
            high_bandwidth_bridge_topics = set(ros_topic_rows)
            high_bandwidth_duplicate_ros_topics = {
                topic for topic in ros_topic_rows if ros_topic_rows.count(topic) > 1
            }
            high_bandwidth_duplicate_gz_topics = {
                topic for topic in gz_topic_rows if gz_topic_rows.count(topic) > 1
            }
            high_bandwidth_bridge_config_valid = not (
                high_bandwidth_duplicate_ros_topics
                or high_bandwidth_duplicate_gz_topics
            )
    checks["formal_high_bandwidth_bridge_config_valid"] = (
        high_bandwidth_bridge_config_valid
    )
    if not high_bandwidth_bridge_config_valid:
        blockers.append("formal_high_bandwidth_bridge_config_invalid")
    missing_lazy_sensor_topics = sorted(
        required_lazy_sensor_topics - high_bandwidth_bridge_topics
    )
    eager_high_bandwidth_sensor_topics = sorted(
        required_lazy_sensor_topics & inline_bridge_topics
    )
    checks["formal_high_bandwidth_sensor_topics_complete"] = not (
        missing_lazy_sensor_topics
    )
    checks["formal_high_bandwidth_sensor_topics_not_eagerly_duplicated"] = not (
        eager_high_bandwidth_sensor_topics
    )
    if missing_lazy_sensor_topics:
        blockers.append("formal_high_bandwidth_sensor_topic_missing")
    if eager_high_bandwidth_sensor_topics:
        blockers.append("formal_high_bandwidth_sensor_topic_eagerly_duplicated")

    # Rear fisheye CameraInfo has no Gazebo Transport source: it is a
    # deliberately single-writer, ROS-native calibration contract.  Count the
    # publisher's declared topics only when the formal launch actually starts
    # that executable, rather than requiring a non-existent GZ bridge.
    fisheye_camera_info_publisher = repository_root / (
        "starter_ws/src/sanitation_vehicle_description/scripts/"
        "formal_fisheye_camera_info_publisher.py"
    )
    ros_native_sensor_topics: set[str] = set()
    if (
        "formal_fisheye_camera_info_publisher.py" in formal_launch_text
        and fisheye_camera_info_publisher.is_file()
    ):
        publisher_text = fisheye_camera_info_publisher.read_text(encoding="utf-8")
        ros_native_sensor_topics = set(
            re.findall(r"[\"'](/[^\"']+)[\"']", publisher_text)
        )
    missing_sensor_bridges = sorted(
        set(sensor_topics)
        - high_bandwidth_bridge_topics
        - ros_native_sensor_topics
    )
    checks["formal_vehicle_sensor_bridges_declared"] = not missing_sensor_bridges
    if missing_sensor_bridges:
        blockers.append("formal_vehicle_sensor_bridge_missing")

    outputs = contract.get("product_outputs", {})
    required_outputs = {
        "detections_2d",
        "masks",
        "unified_targets",
        "wrist_grasp_recheck",
        "diagnostics",
        "detector_boxes_for_edgesam",
    }
    checks["product_output_contract_complete"] = required_outputs <= set(outputs)
    output_types = contract.get("product_output_types", {})
    checks["product_output_types_complete"] = (
        set(outputs.values()) == set(output_types)
        and all("/msg/" in str(message_type) for message_type in output_types.values())
    )

    schedule = contract.get("camera_schedule", {})
    checks["all_formal_cameras_have_compute_schedule"] = all(
        isinstance(schedule.get(sensor_name), Mapping)
        and float(schedule[sensor_name].get("target_rate_hz", 0.0)) > 0.0
        for sensor_name in contract.get("sensor_inputs", {})
    )

    s100_internal = contract.get("s100_internal_contract", {})
    checks["s100_dosod_edgesam_ai_msgs_boundary_explicit"] = (
        s100_internal.get("detector_package") == "hobot_dosod"
        and s100_internal.get("segmenter_package") == "mono_edgesam"
        and s100_internal.get("detector_output_type")
        == s100_internal.get("segmenter_input_type")
        == "ai_msgs/msg/PerceptionTargets"
        and s100_internal.get("product_adapter_required") is True
    )

    graph = contract.get("inference_graph", {})
    discrete_litter = graph.get("discrete_litter", {})
    ground_dirt = graph.get("ground_dirt", {})
    checks["unobservable_material_class_not_inferred"] = (
        discrete_litter.get("classes") == ["litter_cube"]
        and discrete_litter.get("material_class_inference_allowed") is False
    )
    checks["edgesam_not_misclaimed_as_detector"] = (
        ground_dirt.get("edge_sam_discovery_claim") is False
        and "dosod_region_box" in ground_dirt.get("stages", [])
    )
    prompt_vocabulary = graph.get("frozen_prompt_vocabulary", {})
    checks["project_prompt_vocabulary_frozen"] = all(
        isinstance(prompt_vocabulary.get(class_id), list)
        and bool(prompt_vocabulary[class_id])
        for class_id in ("litter_cube", "fallen_leaves", "dust_or_soil", "puddle")
    )

    consumers = contract.get("consumer_contracts", {})
    for name, consumer in consumers.items():
        relative = consumer.get("repository_relative_path")
        topic = consumer.get("required_topic")
        if not relative:
            checks[f"{name}_implemented"] = False
            blockers.append(f"{name}_missing")
            continue
        source = repository_root / relative
        text = source.read_text(encoding="utf-8") if source.is_file() else ""
        ok = bool(source.is_file() and isinstance(topic, str) and topic in text)
        checks[f"{name}_implemented"] = ok
        if not ok:
            blockers.append(f"{name}_missing")

    artifact_base = Path(artifact_root).resolve() if artifact_root else None
    manifest_relative = contract.get("artifact_manifest", {}).get(
        "relative_path", "artifact_manifest.json"
    )
    manifest_rows: dict[str, Any] = {}
    manifest_error = "artifact_root_missing"
    if artifact_base is not None:
        manifest_rows, manifest_error = _artifact_manifest(artifact_base / manifest_relative)
    checks["artifact_manifest_valid"] = manifest_error is None
    if manifest_error:
        blockers.append(manifest_error)

    platform_contract = platforms[platform]
    product_adapter = str(platform_contract.get("product_adapter", ""))
    if platform == "pc":
        python_runtime = platform_contract.get("python_runtime")
        if not isinstance(python_runtime, dict):
            checks["pc_python_runtime_contract_valid"] = False
            checks["pc_python_version_supported"] = False
            blockers.append("pc_python_runtime_contract_invalid")
        else:
            minimum = _version_pair(python_runtime.get("minimum_inclusive"))
            maximum = _version_pair(python_runtime.get("maximum_exclusive"))
            runtime_valid = bool(
                python_runtime.get("formal_environment") == "ubuntu_24_04_ros_jazzy"
                and minimum is not None
                and maximum is not None
                and minimum < maximum
            )
            checks["pc_python_runtime_contract_valid"] = runtime_valid
            version = _runtime_python_version()
            checks["pc_python_version_supported"] = bool(
                runtime_valid and minimum <= version < maximum
            )
            if not runtime_valid:
                blockers.append("pc_python_runtime_contract_invalid")
            elif not checks["pc_python_version_supported"]:
                blockers.append("pc_python_version_unsupported")
        requirements_path = repository_root / "starter_ws/src/sanitation_perception/requirements-pc.txt"
        requirements_text = (
            requirements_path.read_text(encoding="utf-8")
            if requirements_path.is_file()
            else ""
        )
        checks["pc_opencv_headless_declared"] = any(
            line.strip().lower().startswith("opencv-python-headless")
            for line in requirements_text.splitlines()
        )
        try:
            import_module("cv2")
        except Exception:  # fail closed for broken native imports as well as absence
            checks["pc_opencv_importable"] = False
        else:
            checks["pc_opencv_importable"] = True
        if not checks["pc_opencv_headless_declared"]:
            blockers.append("pc_opencv_headless_not_declared")
        if not checks["pc_opencv_importable"]:
            blockers.append("pc_opencv_unavailable")
        product_adapter_source = repository_root / (
            f"starter_ws/src/sanitation_perception/sanitation_perception/{product_adapter}.py"
        )
        product_adapter_text = (
            product_adapter_source.read_text(encoding="utf-8")
            if product_adapter_source.is_file()
            else ""
        )
        checks["pc_product_adapter_present"] = bool(product_adapter and product_adapter_source.is_file())
        checks["pc_product_adapter_excludes_truth_inputs"] = not any(
            literal in product_adapter_text.lower()
            for literal in ('"/ground_truth', '"/evaluator', '"/evaluation/')
        )
        if not checks["pc_product_adapter_present"]:
            blockers.append("pc_product_adapter_missing")
        if not checks["pc_product_adapter_excludes_truth_inputs"]:
            blockers.append("pc_product_adapter_truth_boundary_violation")
    artifact_results: dict[str, Any] = {}
    s100p_launch_source = repository_root / (
        "starter_ws/src/sanitation_perception/launch/formal_s100p_open_vocab.launch.py"
    )
    s100p_launch_text = (
        s100p_launch_source.read_text(encoding="utf-8")
        if s100p_launch_source.is_file()
        else ""
    )
    for component_name, component in platform_contract.get("components", {}).items():
        adapter = str(component.get("runtime_adapter", ""))
        if platform == "rdk_s100":
            executable = str(component.get("runtime_executable", ""))
            adapter_ok = bool(
                executable
                and s100p_launch_source.is_file()
                and f'package="{adapter}"' in s100p_launch_text
                and f'executable="{executable}"' in s100p_launch_text
            )
        else:
            adapter_source = repository_root / (
                f"starter_ws/src/sanitation_perception/sanitation_perception/{adapter}.py"
            )
            adapter_ok = adapter_source.is_file()
        checks[f"{component_name}_runtime_adapter_present"] = adapter_ok
        if not adapter_ok:
            blockers.append(f"{platform}_{component_name}_runtime_adapter_missing")

        if platform == "rdk_s100" and component_name == "dosod":
            supported = component.get("official_platform_support") == "rdk_s100"
            checks["dosod_official_s100_support"] = supported
            if not supported:
                blockers.append("dosod_official_s100_support_unverified")
            reference = component.get("upstream_coco80_reference_only", {})
            checks["upstream_coco80_hbm_not_accepted_as_project_model"] = bool(
                reference.get("artifact")
                and reference.get("artifact") not in component.get("artifacts", [])
            )

        for relative in component.get("artifacts", []):
            artifact = artifact_base / relative if artifact_base is not None else None
            exists = bool(artifact and artifact.is_file())
            row = manifest_rows.get(relative) if isinstance(manifest_rows, dict) else None
            row_ok = isinstance(row, dict)
            required_fields = contract.get("artifact_manifest", {}).get("required_fields", [])
            metadata_ok = bool(row_ok and all(row.get(field) not in (None, "") for field in required_fields))
            digest_ok = bool(exists and metadata_ok and _sha256(artifact) == str(row["sha256"]).lower())
            size_ok = bool(exists and metadata_ok and artifact.stat().st_size == int(row["byte_size"]))
            expected_source_revision = component.get(
                "artifact_source_revisions", {}
            ).get(relative, component.get("revision"))
            source_ok = bool(
                metadata_ok and row["source_revision"] == expected_source_revision
            )
            artifact_results[relative] = {
                "exists": exists,
                "manifest_metadata_complete": metadata_ok,
                "sha256_matches": digest_ok,
                "byte_size_matches": size_ok,
                "source_revision_matches": source_ok,
                "expected_source_revision": expected_source_revision,
            }
            checks[f"artifact:{relative}"] = all(
                (exists, metadata_ok, digest_ok, size_ok, source_ok)
            )
            if not exists:
                blockers.append(f"artifact_missing:{relative}")
            elif not metadata_ok:
                blockers.append(f"artifact_metadata_missing:{relative}")
            elif not all((digest_ok, size_ok, source_ok)):
                blockers.append(f"artifact_provenance_mismatch:{relative}")

    # Historical ONNX evidence lives under artifacts/.  Do not recurse through
    # build, install, log or task workspaces: those trees can contain symlink
    # cycles and are prohibitively slow over a Windows-mounted WSL filesystem.
    evidence_root = repository_root / "artifacts"
    repository_legacy_models = [
        str(path.relative_to(repository_root)).replace("\\", "/")
        for path in (evidence_root.rglob("*.onnx") if evidence_root.is_dir() else ())
        if any(token in path.name for token in LEGACY_MODEL_IDS)
    ]
    checks["legacy_models_not_accepted_as_formal_artifacts"] = not any(
        relative in artifact_results for relative in repository_legacy_models
    )

    blockers = list(dict.fromkeys(blockers))
    ready = bool(checks and all(checks.values()) and not blockers)
    return {
        "schema_version": 1,
        "contract_id": contract.get("contract_id"),
        "platform": platform,
        "ready": ready,
        "status": "ready" if ready else "blocked_fail_closed",
        "checks": checks,
        "blockers": blockers,
        "formal_sensor_topics": sensor_topics,
        "missing_formal_sensor_bridges": missing_sensor_bridges,
        "missing_lazy_formal_sensor_topics": missing_lazy_sensor_topics,
        "eager_high_bandwidth_sensor_topics": eager_high_bandwidth_sensor_topics,
        "high_bandwidth_duplicate_ros_topics": sorted(
            high_bandwidth_duplicate_ros_topics
        ),
        "high_bandwidth_duplicate_gz_topics": sorted(
            high_bandwidth_duplicate_gz_topics
        ),
        "product_outputs": outputs,
        "product_output_types": output_types,
        "artifact_results": artifact_results,
        "legacy_repository_models": repository_legacy_models,
        "ground_truth_input_used": False,
        "claim_boundary": (
            "A passing preflight proves only source/artifact/topic contract readiness. "
            "It does not prove model accuracy, PC real-time performance, S100 board execution, "
            "or downstream closed-loop cleaning."
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", required=True)
    parser.add_argument("--repository-root", required=True)
    parser.add_argument("--platform", choices=("pc", "rdk_s100"), required=True)
    parser.add_argument("--artifact-root")
    parser.add_argument("--output")
    args = parser.parse_args()
    report = audit_formal_perception(
        args.contract,
        repository_root=args.repository_root,
        platform=args.platform,
        artifact_root=args.artifact_root,
    )
    rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    raise SystemExit(0 if report["ready"] else 2)


if __name__ == "__main__":
    main()
