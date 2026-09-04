"""Dry-run the S100P pre-deployment contract without touching a board.

This Windows-safe validator only reads repository files and optional local model
artifacts.  It deliberately has no SSH, copy, process-launch, ROS, or data
collection implementation.  Its report distinguishes a historical G0/BPU smoke
from the current formal S100P acceptance gate.
"""

from __future__ import annotations

import argparse
import ast
import importlib.util
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PLAN = ROOT / "config" / "s100p_offline_predeploy_plan.json"
EXPECTED_BOUNDARY = "no_board_copy_no_ssh_no_node_start_no_data_collection"
OPTIONAL_INPUTS = {"historical_board_smoke"}
REQUIRED_READY_CHECKS = (
    "operation_boundary_exact",
    "optional_input_policy_valid",
    "overlay_package_sources_valid",
    "overlay_runtime_package_set_valid",
    "launch_parameter_record_identity_valid",
    "launch_parameter_path_roles_valid",
    "launch_source_contract_valid",
    "formal_resource_gate_valid",
    "future_operator_plan_recorded",
    "rollback_plan_recorded",
    "central_acceptance_unchanged",
    "validator_has_no_board_or_network_implementation",
)
EXPECTED_NODES = (
    ("sanitation_perception", "rgb_to_nv12_adapter"),
    ("hobot_dosod", "hobot_dosod"),
    ("mono_edgesam", "mono_edgesam"),
    ("sanitation_perception", "open_vocab_product_adapter"),
)
HISTORICAL_G0_REFERENCE_PACKAGES = {
    "hobot_dosod", "mono_edgesam", "ai_msgs", "vision_msgs", "diagnostic_msgs", "cv_bridge", "tf2_ros",
}
EXPECTED_LAUNCH_ARGUMENTS = frozenset(
    {
        "artifact_manifest_path",
        "dosod_model_path",
        "dosod_vocabulary_path",
        "edgesam_encoder_model_path",
        "edgesam_decoder_model_path",
        "front_rgb_topic",
        "front_depth_topic",
        "front_camera_info_topic",
        "map_topic",
    }
)


def _load_json(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    if not path.is_file():
        return None, "missing"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        return None, f"invalid:{type(exc).__name__}"
    if not isinstance(payload, dict):
        return None, "not_mapping"
    return payload, None


def _relative_path(value: Any) -> str | None:
    if not isinstance(value, str) or not value.strip():
        return None
    path = PurePosixPath(value.replace("\\", "/"))
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        return None
    return path.as_posix()


def _append(blockers: list[str], value: str) -> None:
    if value not in blockers:
        blockers.append(value)


def _offline_ready(
    bundle_report: Mapping[str, Any], checks: Mapping[str, bool], blockers: list[str]
) -> bool:
    return not blockers and bool(bundle_report.get("ready")) and all(
        checks.get(key, False) for key in REQUIRED_READY_CHECKS
    )


def _parse_meminfo_kib(value: Any, field: str) -> int | None:
    if not isinstance(value, str):
        return None
    match = re.search(rf"^{re.escape(field)}:\s*(\d+)\s+kB$", value, re.MULTILINE)
    return int(match.group(1)) if match else None


def _extract_ros_packages(g0: Mapping[str, Any]) -> set[str]:
    graph = g0.get("ros_graph_read_only")
    if not isinstance(graph, Mapping):
        return set()
    packages = graph.get("packages")
    if not isinstance(packages, Mapping):
        return set()
    output = packages.get("stdout")
    if not isinstance(output, str):
        return set()
    return {line.strip() for line in output.splitlines() if re.fullmatch(r"[a-z0-9_]+", line.strip())}


def _validator_has_no_board_or_network_implementation(source: str) -> bool:
    """Reject imports or direct calls used for transport/process-side effects."""

    try:
        tree = ast.parse(source)
    except SyntaxError:
        return False
    forbidden_modules = {"subprocess", "socket", "paramiko", "asyncssh", "shutil"}
    for node in ast.walk(tree):
        if isinstance(node, ast.Import) and any(
            alias.name.split(".")[0] in forbidden_modules for alias in node.names
        ):
            return False
        if isinstance(node, ast.ImportFrom) and node.module and node.module.split(".")[0] in forbidden_modules:
            return False
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            if isinstance(node.func.value, ast.Name) and (node.func.value.id, node.func.attr) in {
                ("os", "system"),
                ("shutil", "copy"),
                ("shutil", "copy2"),
            }:
                return False
    return True


def _load_bundle_module(repository_root: Path):
    script = repository_root / "scripts" / "validate_s100p_product_artifact_bundle.py"
    spec = importlib.util.spec_from_file_location("s100p_product_artifact_bundle", script)
    if spec is None or spec.loader is None:
        raise RuntimeError("bundle_validator_import_failed")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _validate_overlay(
    overlay: Mapping[str, Any], repository_root: Path, checks: dict[str, bool], blockers: list[str]
) -> dict[str, Any]:
    packages = overlay.get("packages")
    # The inventory names this collection "base runtime exemptions": these are
    # exactly the packages that must exist in the board image rather than the
    # two packages supplied by the project overlay.
    required_runtime = overlay.get("board_base_runtime_package_exemptions")
    result: dict[str, Any] = {"packages": [], "required_runtime_packages": required_runtime}
    checks["overlay_inventory_identity_valid"] = (
        overlay.get("schema_version") == 1
        and overlay.get("inventory_id") == "tzcup_s100p_product_overlay_packages_v1"
        and overlay.get("operation_boundary") == "preparation_only_no_build_install_or_board_copy"
        and overlay.get("workspace_relative_path") == "starter_ws"
        and overlay.get("required_setup_relative_path") == "install/setup.bash"
    )
    if not checks["overlay_inventory_identity_valid"]:
        _append(blockers, "overlay_inventory_identity_invalid")
    if not isinstance(packages, list) or not packages:
        checks["overlay_package_sources_valid"] = False
        _append(blockers, "overlay_packages_missing")
        return result
    package_ok = True
    for row in packages:
        entry: dict[str, Any] = {"valid": False}
        result["packages"].append(entry)
        if not isinstance(row, Mapping):
            package_ok = False
            continue
        name = row.get("name")
        source_rel = _relative_path(row.get("source_relative_path"))
        xml_rel = _relative_path(row.get("package_xml_relative_path"))
        target_rel = _relative_path(row.get("board_overlay_relative_path"))
        entry.update({"name": name, "source_relative_path": source_rel, "board_overlay_relative_path": target_rel})
        if not isinstance(name, str) or not source_rel or not xml_rel or not target_rel:
            package_ok = False
            _append(blockers, f"overlay_package_definition_invalid:{name}")
            continue
        source_path = repository_root / source_rel
        xml_path = repository_root / xml_rel
        try:
            xml_name = ET.parse(xml_path).getroot().findtext("name")
        except (OSError, ET.ParseError):
            xml_name = None
        entry["source_exists"] = source_path.is_dir()
        entry["package_xml_name"] = xml_name
        entry["valid"] = source_path.is_dir() and xml_name == name
        if not entry["valid"]:
            package_ok = False
            _append(blockers, f"overlay_package_source_or_xml_invalid:{name}")
    checks["overlay_package_sources_valid"] = package_ok
    # Keep this list in parity with the full package.xml closure declared in
    # s100p_product_overlay_packages.json.  A shortened hand-picked subset can
    # make an incomplete base image look predeploy-ready.
    expected_runtime = {
        "ai_msgs", "cv_bridge", "diagnostic_msgs", "geometry_msgs", "hobot_dosod",
        "launch", "launch_ros", "mono_edgesam", "nav_msgs", "python3-numpy",
        "python3-opencv", "python3-pip", "python3-yaml", "rclpy",
        "ros_gz_interfaces", "sensor_msgs", "std_msgs", "tf2_ros", "vision_msgs",
    }
    checks["overlay_runtime_package_set_valid"] = isinstance(required_runtime, list) and set(required_runtime) == expected_runtime
    if not checks["overlay_runtime_package_set_valid"]:
        _append(blockers, "overlay_required_runtime_packages_invalid")
    return result


def _validate_launch(
    launch_record: Mapping[str, Any], launch_text: str | None, checks: dict[str, bool], blockers: list[str]
) -> dict[str, Any]:
    result: dict[str, Any] = {"expected_nodes": list(EXPECTED_NODES)}
    checks["launch_parameter_record_identity_valid"] = (
        launch_record.get("schema_version") == 1
        and launch_record.get("record_id") == "tzcup_s100p_product_board_launch_parameters_v1"
        and launch_record.get("operation_boundary") == "record_only_no_ssh_copy_install_node_start_or_actuator_command"
        and launch_record.get("platform") == "rdk_s100"
        and launch_record.get("board") == "RDK S100P"
        and launch_record.get("soc") == "Journey 6P"
        and launch_record.get("march") == "nash-m"
    )
    if not checks["launch_parameter_record_identity_valid"]:
        _append(blockers, "launch_parameter_record_identity_invalid")
    required = launch_record.get("required_absolute_parameters")
    required_names = set(required) if isinstance(required, Mapping) else set()
    expected_paths = {
        "artifact_manifest_path": "artifact_manifest.json",
        "dosod_model_path": "dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm",
        "dosod_vocabulary_path": "dosod/tzcup_offline_vocabulary.json",
        "edgesam_encoder_model_path": "edgesam/edgesam_encoder_512.hbm",
        "edgesam_decoder_model_path": "edgesam/edgesam_decoder_512.hbm",
    }
    checks["launch_parameter_path_roles_valid"] = required_names == set(expected_paths) and all(
        isinstance(required, Mapping)
        and isinstance(required.get(name), Mapping)
        and required[name].get("target_root") == "artifact_root"
        and required[name].get("relative_path") == relative
        for name, relative in expected_paths.items()
    )
    if not checks["launch_parameter_path_roles_valid"]:
        _append(blockers, "launch_parameter_path_roles_invalid")
    if launch_text is None:
        checks["launch_source_contract_valid"] = False
        _append(blockers, "launch_source_missing")
        return result
    arguments = set(re.findall(r'DeclareLaunchArgument\(\s*"([^"]+)"', launch_text))
    nodes_ok = all(
        f'package="{package}"' in launch_text and f'executable="{executable}"' in launch_text
        for package, executable in EXPECTED_NODES
    )
    product_topics = {
        "/perception/garbage/detections_2d",
        "/perception/ground_dirt/masks",
        "/perception/garbage/targets",
        "/perception/open_vocab/diagnostics",
    }
    source_safe = not any(token in launch_text for token in ("ExecuteProcess", "subprocess", "/cmd_vel", "/cleaning/enable"))
    checks["launch_source_contract_valid"] = (
        EXPECTED_LAUNCH_ARGUMENTS.issubset(arguments)
        and nodes_ok
        and all(topic in launch_text for topic in product_topics)
        and source_safe
    )
    result.update({"declared_arguments": sorted(arguments), "product_topics": sorted(product_topics)})
    if not checks["launch_source_contract_valid"]:
        _append(blockers, "launch_source_interface_or_safety_contract_invalid")
    return result


def _validate_historical_g0(
    g0: Mapping[str, Any], required_runtime_packages: list[str] | None, checks: dict[str, bool], blockers: list[str]
) -> dict[str, Any]:
    identity = g0.get("identity")
    safety = g0.get("safety")
    result: dict[str, Any] = {"status": g0.get("status"), "evidence_class": "historical_read_only_reference_not_current_acceptance"}
    checks["historical_g0_identity_valid"] = (
        g0.get("status") == "G0_READ_ONLY_INVENTORY_COLLECTED"
        and isinstance(identity, Mapping)
        and isinstance(identity.get("model"), Mapping)
        and identity["model"].get("value") == "D-Robotics RDK S100P V1P0"
        and isinstance(identity.get("compatible"), Mapping)
        and identity["compatible"].get("value") == "drobot,s100-rdk"
        and identity.get("architecture") == "aarch64"
    )
    if not checks["historical_g0_identity_valid"]:
        _append(blockers, "historical_g0_identity_invalid")
    checks["historical_g0_read_only_safety_valid"] = isinstance(safety, Mapping) and all(
        safety.get(name) is False
        for name in (
            "actuator_commands_sent",
            "ros_publish_or_service_calls_sent",
            "can_gpio_or_actuator_access_attempted",
        )
    )
    if not checks["historical_g0_read_only_safety_valid"]:
        _append(blockers, "historical_g0_read_only_safety_invalid")
    packages = _extract_ros_packages(g0)
    # G0 is a retained, deliberately narrow read-only baseline.  It can only
    # prove the package subset observed then; requiring the future full overlay
    # closure here would relabel a historical reference as a current install.
    reference = HISTORICAL_G0_REFERENCE_PACKAGES
    planned = set(required_runtime_packages or [])
    checks["historical_g0_runtime_package_reference_valid"] = reference.issubset(packages)
    result["runtime_packages_missing_from_historical_inventory"] = sorted(reference - packages)
    result["future_predeploy_packages_not_proven_by_historical_inventory"] = sorted(planned - packages)
    if not checks["historical_g0_runtime_package_reference_valid"]:
        _append(blockers, "historical_g0_runtime_package_inventory_incomplete")
    overlays = g0.get("project_overlay_files")
    checks["historical_g0_project_overlay_absent"] = isinstance(overlays, list) and bool(overlays) and all(
        isinstance(row, Mapping) and row.get("status") == "ABSENT" for row in overlays
    )
    if not checks["historical_g0_project_overlay_absent"]:
        _append(blockers, "historical_g0_project_overlay_status_unknown")
    models = g0.get("model_files_path_and_sha256_only")
    checks["historical_g0_project_models_absent"] = models == []
    if not checks["historical_g0_project_models_absent"]:
        _append(blockers, "historical_g0_project_model_status_unknown")
    memory = g0.get("memory")
    meminfo = memory.get("meminfo", {}) if isinstance(memory, Mapping) else {}
    memtext = meminfo.get("value") if isinstance(meminfo, Mapping) else None
    total_kib = _parse_meminfo_kib(memtext, "MemTotal")
    available_kib = _parse_meminfo_kib(memtext, "MemAvailable")
    result["historical_mem_total_kib"] = total_kib
    result["historical_mem_available_kib"] = available_kib
    result["historical_available_memory_percent"] = (available_kib * 100.0 / total_kib) if total_kib and available_kib is not None else None
    checks["historical_g0_resource_reference_meets_5_percent"] = bool(
        total_kib and available_kib is not None and available_kib * 20 >= total_kib
    )
    return result


def _validate_historical_smoke(smoke: Mapping[str, Any], checks: dict[str, bool], blockers: list[str]) -> dict[str, Any]:
    runtime = smoke.get("runtime")
    safety = smoke.get("safety")
    nodes = smoke.get("nodes_observed")
    expected_nodes = {"rgb_to_nv12_adapter", "hobot_dosod", "mono_edgesam", "open_vocab_product_adapter"}
    checks["historical_smoke_reference_available"] = True
    checks["historical_smoke_reference_valid"] = (
        smoke.get("status") == "SMOKE_PASSED_FORMAL_ACCEPTANCE_BLOCKED"
        and smoke.get("formal_acceptance") is False
        and isinstance(runtime, Mapping)
        and runtime.get("backend") == "bpu"
        and isinstance(nodes, list)
        and expected_nodes.issubset(nodes)
        and isinstance(safety, Mapping)
        and safety.get("actuator_commands_sent") is False
    )
    if not checks["historical_smoke_reference_valid"]:
        _append(blockers, "historical_board_smoke_invalid_or_overclaimed")
    checks["historical_smoke_remains_nonformal"] = smoke.get("formal_acceptance") is False and bool(smoke.get("formal_blockers"))
    checks["historical_smoke_does_not_grant_acceptance"] = smoke.get("formal_acceptance") is False
    if not checks["historical_smoke_remains_nonformal"]:
        _append(blockers, "historical_board_smoke_formal_boundary_missing")
    return {
        "available": True,
        "status": smoke.get("status"),
        "formal_acceptance": smoke.get("formal_acceptance"),
        "formal_blockers": smoke.get("formal_blockers"),
        "evidence_class": "historical_bpu_smoke_not_formal_acceptance",
    }


def validate_offline_predeploy(
    plan_path: str | Path = DEFAULT_PLAN,
    *,
    repository_root: str | Path = ROOT,
    artifact_root: str | Path | None = None,
) -> dict[str, Any]:
    """Return a low-memory, local-only pre-deployment audit report."""

    repository_root = Path(repository_root).resolve()
    plan_path = Path(plan_path).resolve()
    plan, error = _load_json(plan_path)
    blockers: list[str] = []
    checks: dict[str, bool] = {}
    if error:
        return {
            "schema_version": 1,
            "report_id": "tzcup_s100p_offline_predeploy_validation_v1",
            "operation_boundary": EXPECTED_BOUNDARY,
            "status": "BLOCKED",
            "ready": False,
            "blockers": [f"offline_predeploy_plan_{error}"],
            "checks": {"offline_predeploy_plan_parseable": False},
        }
    assert plan is not None
    checks["offline_predeploy_plan_parseable"] = True
    checks["operation_boundary_exact"] = (
        plan.get("schema_version") == 1
        and plan.get("plan_id") == "tzcup_s100p_offline_predeploy_v1"
        and plan.get("operation_boundary") == EXPECTED_BOUNDARY
        and plan.get("scope") == "windows_low_memory_offline_predeployment_audit_only"
    )
    if not checks["operation_boundary_exact"]:
        _append(blockers, "operation_boundary_invalid")
    inputs = plan.get("inputs")
    if not isinstance(inputs, Mapping):
        inputs = {}
        _append(blockers, "plan_inputs_missing")
    resolved: dict[str, Path] = {}
    expected_inputs = {
        "artifact_bundle_manifest",
        "overlay_package_inventory",
        "board_launch_parameter_record",
        "board_launch_source",
        "historical_g0_inventory",
        "historical_board_smoke",
    }
    optional_inputs = plan.get("optional_inputs")
    checks["optional_input_policy_valid"] = (
        isinstance(optional_inputs, list) and set(optional_inputs) == OPTIONAL_INPUTS
    )
    if not checks["optional_input_policy_valid"]:
        _append(blockers, "optional_input_policy_invalid")
    checks["plan_input_keys_exact"] = set(inputs) == expected_inputs
    if not checks["plan_input_keys_exact"]:
        _append(blockers, "plan_input_keys_invalid")
    for name in expected_inputs:
        relative = _relative_path(inputs.get(name))
        if not relative:
            _append(blockers, f"plan_input_path_invalid:{name}")
            continue
        path = repository_root / relative
        resolved[name] = path
        if not path.is_file() and name not in OPTIONAL_INPUTS:
            _append(blockers, f"plan_input_missing:{name}")

    bundle, bundle_error = _load_json(resolved.get("artifact_bundle_manifest", Path(".")))
    overlay, overlay_error = _load_json(resolved.get("overlay_package_inventory", Path(".")))
    launch_record, launch_record_error = _load_json(resolved.get("board_launch_parameter_record", Path(".")))
    g0, g0_error = _load_json(resolved.get("historical_g0_inventory", Path(".")))
    smoke, smoke_error = _load_json(resolved.get("historical_board_smoke", Path(".")))
    for name, load_error in {
        "bundle": bundle_error,
        "overlay": overlay_error,
        "launch_record": launch_record_error,
        "historical_g0": g0_error,
        "historical_smoke": smoke_error if smoke_error != "missing" else None,
    }.items():
        if load_error:
            _append(blockers, f"{name}_{load_error}")

    artifact_root = Path(artifact_root).resolve() if artifact_root else repository_root / ".work" / "formal_perception_assets"
    bundle_report: dict[str, Any] = {}
    if bundle and not bundle_error:
        try:
            module = _load_bundle_module(repository_root)
            bundle_report = module.validate_bundle(
                resolved["artifact_bundle_manifest"], artifact_root=artifact_root, repository_root=repository_root
            )
            checks["bundle_validator_completed"] = True
            for item in bundle_report.get("blockers", []):
                _append(blockers, f"bundle:{item}")
        except (OSError, RuntimeError, AttributeError) as exc:
            checks["bundle_validator_completed"] = False
            _append(blockers, f"bundle_validator_failed:{type(exc).__name__}")
    else:
        checks["bundle_validator_completed"] = False

    overlay_result = _validate_overlay(overlay, repository_root, checks, blockers) if overlay and not overlay_error else {}
    launch_text = None
    launch_path = resolved.get("board_launch_source")
    if launch_path and launch_path.is_file():
        launch_text = launch_path.read_text(encoding="utf-8")
    launch_result = _validate_launch(launch_record, launch_text, checks, blockers) if launch_record and not launch_record_error else {}
    g0_result = _validate_historical_g0(
        g0, overlay_result.get("required_runtime_packages"), checks, blockers
    ) if g0 and not g0_error else {}
    if smoke and not smoke_error:
        smoke_result = _validate_historical_smoke(smoke, checks, blockers)
    elif smoke_error == "missing":
        checks["historical_smoke_reference_available"] = False
        checks["historical_smoke_reference_valid"] = False
        checks["historical_smoke_remains_nonformal"] = False
        checks["historical_smoke_does_not_grant_acceptance"] = True
        smoke_result = {
            "available": False,
            "status": "MISSING_OPTIONAL_REFERENCE",
            "formal_acceptance": False,
            "formal_blockers": ["historical_bpu_smoke_source_not_provided"],
            "evidence_class": "missing_historical_bpu_smoke_not_current_acceptance",
        }
    else:
        checks["historical_smoke_reference_available"] = False
        checks["historical_smoke_reference_valid"] = False
        checks["historical_smoke_remains_nonformal"] = False
        checks["historical_smoke_does_not_grant_acceptance"] = True
        smoke_result = {
            "available": False,
            "status": "INVALID_OPTIONAL_REFERENCE",
            "formal_acceptance": False,
            "formal_blockers": ["historical_bpu_smoke_source_invalid"],
            "evidence_class": "invalid_historical_bpu_smoke_not_current_acceptance",
        }

    resource_gate = plan.get("formal_resource_gate")
    checks["formal_resource_gate_valid"] = isinstance(resource_gate, Mapping) and (
        resource_gate.get("duration_sec") == 1800
        and resource_gate.get("minimum_available_memory_percent") == 5.0
        and resource_gate.get("maximum_temperature_c") == 85.0
        and resource_gate.get("required_backends") == ["bpu"]
        and resource_gate.get("minimum_hz") == {"dosod": 2.0, "edgesam": 1.0}
        and resource_gate.get("maximum_p95_latency_ms") == 1000.0
        and resource_gate.get("historical_g0_is_not_a_current_resource_pass") is True
    )
    if not checks["formal_resource_gate_valid"]:
        _append(blockers, "formal_resource_gate_invalid")
    checks["future_operator_plan_recorded"] = isinstance(plan.get("future_operator_plan"), list) and len(plan["future_operator_plan"]) == 4
    checks["rollback_plan_recorded"] = isinstance(plan.get("rollback_plan"), Mapping) and plan["rollback_plan"].get("current_task_board_mutation") is False
    checks["central_acceptance_unchanged"] = isinstance(plan.get("claim_boundary"), Mapping) and plan["claim_boundary"].get("central_acceptance_unchanged") is True
    if not checks["future_operator_plan_recorded"]:
        _append(blockers, "future_operator_plan_missing")
    if not checks["rollback_plan_recorded"]:
        _append(blockers, "rollback_plan_missing_or_mutating")
    if not checks["central_acceptance_unchanged"]:
        _append(blockers, "central_acceptance_boundary_missing")

    source = Path(__file__).read_text(encoding="utf-8")
    checks["validator_has_no_board_or_network_implementation"] = _validator_has_no_board_or_network_implementation(source)
    if not checks["validator_has_no_board_or_network_implementation"]:
        _append(blockers, "validator_contains_forbidden_board_or_network_implementation")
    offline_ready = _offline_ready(bundle_report, checks, blockers)
    return {
        "schema_version": 1,
        "report_id": "tzcup_s100p_offline_predeploy_validation_v1",
        "operation_boundary": EXPECTED_BOUNDARY,
        "operation_performed": "local_read_only_dry_run",
        "status": "PREDEPLOY_READY_NOT_DEPLOYED" if offline_ready else "BLOCKED",
        "ready": offline_ready,
        "board_interaction_performed": False,
        "data_collection_performed": False,
        "formal_board_acceptance": False,
        "plan_path": str(plan_path),
        "artifact_root": str(artifact_root),
        "checks": checks,
        "blockers": blockers,
        "bundle_report": bundle_report,
        "overlay": overlay_result,
        "launch": launch_result,
        "historical_g0": g0_result,
        "historical_board_smoke": smoke_result,
        "formal_resource_gate": resource_gate,
        "future_operator_plan": plan.get("future_operator_plan"),
        "rollback_plan": plan.get("rollback_plan"),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, default=DEFAULT_PLAN)
    parser.add_argument("--repository-root", type=Path, default=ROOT)
    parser.add_argument("--artifact-root", type=Path)
    parser.add_argument("--report", type=Path, help="Optional local JSON report path.")
    parser.add_argument("--allow-blocked-exit-zero", action="store_true")
    args = parser.parse_args()
    report = validate_offline_predeploy(args.plan, repository_root=args.repository_root, artifact_root=args.artifact_root)
    encoded = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if report["ready"] or args.allow_blocked_exit_zero else 2


if __name__ == "__main__":
    raise SystemExit(main())
