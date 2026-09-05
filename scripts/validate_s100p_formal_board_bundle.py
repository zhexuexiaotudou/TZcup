"""Fail-closed static audit for the S100P snapshot-bound board bundle.

This module only hashes local files in 1 MiB blocks and reads JSON/text.  It
does not copy a payload, contact a board, invoke ROS, or create model/HBM data.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
from typing import Any, Mapping
from xml.etree import ElementTree


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = ROOT / "config" / "s100p_formal_board_bundle_manifest.json"
EXPECTED_BOUND_ROLES = {
    "dosod_hbm_compile_contract",
    "offline_predeploy_product_bundle",
    "board_launch_parameter_record",
    "board_overlay_package_contract",
    "dosod_edgesam_s100_profile",
    "formal_ros2_launch",
    "project_perception_package_manifest",
    "nv12_adapter_source",
    "product_adapter_source",
    "project_perception_entry_points",
    "perception_interfaces_package_manifest",
}
REQUIRED_MODEL_ASSETS = {
    "dosod_hbm",
    "dosod_vocabulary",
    "edgesam_encoder_hbm",
    "edgesam_decoder_hbm",
}
REQUIRED_PAYLOAD_ROLE_BINDINGS = {
    "dosod_hbm": {
        "target_relative_path": "dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm",
        "launch_parameter": "dosod_model_path",
        "launch_binding": '"model_file_name": dosod_model_path',
    },
    "dosod_vocabulary": {
        "target_relative_path": "dosod/tzcup_offline_vocabulary.json",
        "launch_parameter": "dosod_vocabulary_path",
        "launch_binding": '"vocabulary_file_name": dosod_vocabulary_path',
    },
    "edgesam_encoder_hbm": {
        "target_relative_path": "edgesam/edgesam_encoder_512.hbm",
        "launch_parameter": "edgesam_encoder_model_path",
        "launch_binding": '"encoder_model_file_name": edgesam_encoder_model_path',
    },
    "edgesam_decoder_hbm": {
        "target_relative_path": "edgesam/edgesam_decoder_512.hbm",
        "launch_parameter": "edgesam_decoder_model_path",
        "launch_binding": '"decoder_model_file_name": edgesam_decoder_model_path',
    },
}
SANITATION_PERCEPTION_EXEC_DEPENDENCIES = {
    "ai_msgs", "cv_bridge", "diagnostic_msgs", "geometry_msgs", "launch",
    "launch_ros", "nav_msgs", "python3-numpy", "python3-opencv", "python3-pip",
    "python3-yaml", "rclpy", "ros_gz_interfaces", "sanitation_perception_interfaces",
    "sensor_msgs", "std_msgs", "tf2_ros", "vision_msgs",
}
MANDATORY_BLOCKERS = {
    "project_dosod_hbm_missing_or_unhashed",
    "required_board_model_payloads_not_all_receipted",
    "board_tzcup_overlay_unbuilt_or_unverified",
    "board_runtime_dependencies_unverified",
    "thermal_and_power_measurement_not_collected",
    "board_runtime_evidence_not_collected",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_json(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    try:
        is_file = path.is_file()
    except OSError as exc:
        return None, f"unreadable:{type(exc).__name__}"
    if not is_file:
        return None, "missing"
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        return None, f"invalid:{type(exc).__name__}"
    return (value, None) if isinstance(value, dict) else (None, "not_mapping")


def _read_text(path: Path) -> tuple[str | None, str | None]:
    try:
        return path.read_text(encoding="utf-8"), None
    except FileNotFoundError:
        return None, "missing"
    except (OSError, UnicodeDecodeError) as exc:
        return None, f"unreadable:{type(exc).__name__}"


def _relative(value: Any) -> str | None:
    if not isinstance(value, str) or not value.strip():
        return None
    path = PurePosixPath(value.replace("\\", "/"))
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        return None
    return path.as_posix()


def _append(blockers: list[str], value: str) -> None:
    if value not in blockers:
        blockers.append(value)


def _source_bindings(
    root: Path, rows: Any, blockers: list[str]
) -> tuple[dict[str, bool], dict[str, Any]]:
    checks: dict[str, bool] = {}
    results: dict[str, Any] = {}
    if not isinstance(rows, list):
        _append(blockers, "bound_sources_not_a_list")
        return {"bound_source_roles_exact": False, "bound_source_digests_valid": False}, results
    roles = {row.get("role") for row in rows if isinstance(row, Mapping)}
    checks["bound_source_roles_exact"] = len(rows) == len(EXPECTED_BOUND_ROLES) and roles == EXPECTED_BOUND_ROLES
    if not checks["bound_source_roles_exact"]:
        _append(blockers, "bound_source_roles_not_exact")
    all_valid = True
    for row in rows:
        if not isinstance(row, Mapping):
            all_valid = False
            _append(blockers, "bound_source_row_invalid")
            continue
        role = row.get("role")
        relative = _relative(row.get("path"))
        valid_declared = (
            isinstance(role, str)
            and isinstance(row.get("sha256"), str)
            and len(row["sha256"]) == 64
            and isinstance(row.get("byte_size"), int)
            and row["byte_size"] > 0
            and relative is not None
        )
        if not valid_declared:
            all_valid = False
            _append(blockers, f"bound_source_declaration_invalid:{role}")
            continue
        source = root / relative
        try:
            source.resolve().relative_to(root.resolve())
        except (OSError, ValueError):
            all_valid = False
            _append(blockers, f"bound_source_escape:{role}")
            continue
        try:
            if not source.is_file():
                all_valid = False
                _append(blockers, f"bound_source_missing:{role}")
                continue
            actual_size = source.stat().st_size
            actual_sha = _sha256(source)
        except (OSError, ValueError) as exc:
            all_valid = False
            _append(blockers, f"bound_source_unreadable:{role}:{type(exc).__name__}")
            continue
        valid = actual_size == row["byte_size"] and actual_sha == row["sha256"]
        results[str(role)] = {
            "path": relative,
            "byte_size": actual_size,
            "sha256": actual_sha,
            "valid": valid,
        }
        if not valid:
            all_valid = False
            _append(blockers, f"bound_source_digest_mismatch:{role}")
    checks["bound_source_digests_valid"] = all_valid
    return checks, results


def _payload_role_checks(
    payload_roles: Any, product: Mapping[str, Any], parameters: Mapping[str, Any],
    launch: str, blockers: list[str],
) -> dict[str, bool]:
    checks: dict[str, bool] = {}
    rows_by_key: dict[str, Mapping[str, Any]] = {}
    if isinstance(payload_roles, list):
        for row in payload_roles:
            if isinstance(row, Mapping) and isinstance(row.get("asset_key"), str):
                rows_by_key[row["asset_key"]] = row
    checks["required_board_payload_roles_exact"] = (
        isinstance(payload_roles, list)
        and len(payload_roles) == len(REQUIRED_PAYLOAD_ROLE_BINDINGS)
        and set(rows_by_key) == set(REQUIRED_PAYLOAD_ROLE_BINDINGS)
        and all(
            row.get("target_relative_path") == binding["target_relative_path"]
            and row.get("required") is True
            and row.get("source_receipt_required") is True
            for key, binding in REQUIRED_PAYLOAD_ROLE_BINDINGS.items()
            for row in (rows_by_key[key],)
        )
    )
    if not checks["required_board_payload_roles_exact"]:
        _append(blockers, "required_board_payload_roles_not_exact")
    assets = product.get("assets")
    checks["payload_roles_match_product_artifact_bundle"] = isinstance(assets, Mapping) and all(
        isinstance(assets.get(key), Mapping)
        and assets[key].get("target_root") == "artifact_root"
        and assets[key].get("target_relative_path") == binding["target_relative_path"]
        for key, binding in REQUIRED_PAYLOAD_ROLE_BINDINGS.items()
    )
    if not checks["payload_roles_match_product_artifact_bundle"]:
        _append(blockers, "required_board_payload_roles_do_not_match_product_bundle")
    required_parameters = parameters.get("required_absolute_parameters")
    checks["payload_roles_match_launch_parameter_record"] = isinstance(required_parameters, Mapping) and all(
        isinstance(required_parameters.get(binding["launch_parameter"]), Mapping)
        and required_parameters[binding["launch_parameter"]].get("target_root") == "artifact_root"
        and required_parameters[binding["launch_parameter"]].get("relative_path") == binding["target_relative_path"]
        for binding in REQUIRED_PAYLOAD_ROLE_BINDINGS.values()
    )
    if not checks["payload_roles_match_launch_parameter_record"]:
        _append(blockers, "required_board_payload_roles_do_not_match_launch_record")
    checks["launch_binds_each_required_board_payload_role"] = all(
        f'LaunchConfiguration("{binding["launch_parameter"]}")' in launch
        and f'DeclareLaunchArgument(\n                "{binding["launch_parameter"]}"' in launch
        and binding["launch_binding"] in launch
        for binding in REQUIRED_PAYLOAD_ROLE_BINDINGS.values()
    )
    if not checks["launch_binds_each_required_board_payload_role"]:
        _append(blockers, "launch_does_not_bind_each_required_board_payload_role")
    return checks


def _runtime_dependency_closure_checks(
    overlay: Mapping[str, Any], package_xml: str, launch: str, blockers: list[str]
) -> dict[str, bool]:
    checks: dict[str, bool] = {}
    try:
        root = ElementTree.fromstring(package_xml)
        actual_exec_dependencies = {
            (node.text or "").strip() for node in root.findall("exec_depend") if (node.text or "").strip()
        }
    except ElementTree.ParseError:
        _append(blockers, "sanitation_perception_package_xml_unparseable")
        return {"sanitation_perception_package_xml_parseable": False}
    checks["sanitation_perception_package_xml_parseable"] = True
    checks["sanitation_perception_exec_dependencies_exact"] = actual_exec_dependencies == SANITATION_PERCEPTION_EXEC_DEPENDENCIES
    if not checks["sanitation_perception_exec_dependencies_exact"]:
        _append(blockers, "sanitation_perception_exec_dependencies_unexpected")
    declared_exec = overlay.get("sanitation_perception_package_xml_exec_dependencies")
    provided = overlay.get("overlay_provided_runtime_packages")
    exemptions = overlay.get("board_base_runtime_package_exemptions")
    launch_base = overlay.get("launch_required_board_base_packages")
    def _string_set(value: Any) -> tuple[set[str], bool]:
        if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
            return set(), False
        values = set(value)
        return values, len(values) == len(value)

    declared_exec_set, declared_exec_unique = _string_set(declared_exec)
    provided_set, provided_unique = _string_set(provided)
    exemptions_set, exemptions_unique = _string_set(exemptions)
    launch_base_set, launch_base_unique = _string_set(launch_base)
    package_rows = overlay.get("packages")
    package_names = {
        row.get("name") for row in package_rows if isinstance(row, Mapping) and isinstance(row.get("name"), str)
    } if isinstance(package_rows, list) else set()
    checks["overlay_runtime_inventory_matches_package_xml"] = (
        declared_exec_unique and declared_exec_set == actual_exec_dependencies
    )
    checks["overlay_runtime_dependency_closure_classified"] = (
        provided_unique and exemptions_unique
        and provided_set == package_names == {"sanitation_perception", "sanitation_perception_interfaces"}
        and exemptions_set == ((actual_exec_dependencies - provided_set) | launch_base_set)
        and not (provided_set & exemptions_set)
    )
    checks["launch_required_base_packages_explicitly_exempted"] = (
        launch_base_unique and launch_base_set == {"hobot_dosod", "mono_edgesam"}
        and launch_base_set <= exemptions_set
        and all(f'package="{name}"' in launch for name in launch_base_set)
    )
    if not checks["overlay_runtime_inventory_matches_package_xml"]:
        _append(blockers, "overlay_runtime_inventory_does_not_match_package_xml")
    if not checks["overlay_runtime_dependency_closure_classified"]:
        _append(blockers, "overlay_runtime_dependency_closure_unclassified")
    if not checks["launch_required_base_packages_explicitly_exempted"]:
        _append(blockers, "launch_required_board_base_packages_not_explicitly_exempted")
    return checks


def _semantic_checks(root: Path, payload_roles: Any, blockers: list[str]) -> dict[str, bool]:
    checks: dict[str, bool] = {}
    product, product_error = _load_json(root / "config/s100p_product_artifact_bundle.json")
    compile_contract, compile_error = _load_json(root / "config/dosod_s100p_hbm_compile_contract.json")
    overlay, overlay_error = _load_json(root / "config/s100p_product_overlay_packages.json")
    parameters, parameter_error = _load_json(root / "config/s100p_product_board_launch_parameters.json")
    snapshot, snapshot_error = _load_json(root / "reports/engineering/formal_vehicle_snapshot_manifest.json")
    if product_error or compile_error or overlay_error or parameter_error or snapshot_error:
        _append(blockers, "required_s100p_or_snapshot_configuration_unparseable")
        return {"semantic_configuration_parseable": False}
    assert product and compile_contract and overlay and parameters and snapshot
    checks["semantic_configuration_parseable"] = True
    assets = product.get("assets")
    checks["product_bundle_declared_preparation_only"] = product.get("status") == "PREPARATION_ONLY_BLOCKED"
    checks["product_bundle_model_role_set_valid"] = isinstance(assets, Mapping) and REQUIRED_MODEL_ASSETS <= set(assets)
    overlay_asset = assets.get("overlay_package_inventory") if isinstance(assets, Mapping) else None
    overlay_path = root / "config/s100p_product_overlay_packages.json"
    try:
        overlay_size = overlay_path.stat().st_size
        overlay_sha = _sha256(overlay_path)
    except (OSError, ValueError):
        overlay_size = -1
        overlay_sha = ""
    checks["product_bundle_overlay_inventory_digest_matches_contract"] = (
        isinstance(overlay_asset, Mapping)
        and overlay_asset.get("relative_path")
        == "config/s100p_product_overlay_packages.json"
        and overlay_asset.get("byte_size") == overlay_size
        and overlay_asset.get("sha256") == overlay_sha
    )
    if not checks["product_bundle_overlay_inventory_digest_matches_contract"]:
        _append(blockers, "product_bundle_overlay_inventory_digest_mismatch")
    dosod = assets.get("dosod_hbm") if isinstance(assets, Mapping) else None
    checks["dosod_hbm_is_not_claimed_available"] = isinstance(dosod, Mapping) and dosod.get("sha256") is None and dosod.get("byte_size") is None
    checks["dosod_hbm_compile_output_not_produced"] = (
        isinstance(compile_contract.get("output"), Mapping)
        and compile_contract["output"].get("status") == "HBM_NOT_PRODUCED"
    )
    checks["overlay_is_not_claimed_built_or_copied"] = (
        isinstance(overlay.get("claim_boundary"), Mapping)
        and overlay["claim_boundary"].get("overlay_built") is False
        and overlay["claim_boundary"].get("overlay_copied_to_board") is False
        and overlay["claim_boundary"].get("dependencies_installed") is False
    )
    checks["snapshot_identity_present"] = (
        snapshot.get("kind") == "tzcup_formal_vehicle_snapshot_manifest"
        and snapshot.get("schema_version") == 1
        and isinstance(snapshot.get("source_inventory_sha256"), str)
        and isinstance(snapshot.get("output_inventory_sha256"), str)
    )
    launch, launch_error = _read_text(root / "starter_ws/src/sanitation_perception/launch/formal_s100p_open_vocab.launch.py")
    profile, profile_error = _read_text(root / "starter_ws/src/sanitation_perception/config/open_vocab_s100_profile.yaml")
    package, package_error = _read_text(root / "starter_ws/src/sanitation_perception/package.xml")
    interfaces, interfaces_error = _read_text(root / "starter_ws/src/sanitation_perception_interfaces/package.xml")
    setup, setup_error = _read_text(root / "starter_ws/src/sanitation_perception/setup.py")
    source_errors = {
        name: error for name, error in (
            ("launch", launch_error), ("profile", profile_error), ("package_xml", package_error),
            ("interfaces_package_xml", interfaces_error), ("setup", setup_error),
        ) if error
    }
    if source_errors:
        _append(blockers, "semantic_bound_source_unreadable")
        for name, source_error in source_errors.items():
            _append(blockers, f"semantic_bound_source_{source_error}:{name}")
        checks["semantic_bound_sources_readable"] = False
        return checks
    assert launch is not None and profile is not None and package is not None and interfaces is not None and setup is not None
    checks["semantic_bound_sources_readable"] = True
    checks.update(_payload_role_checks(payload_roles, product, parameters, launch, blockers))
    checks.update(_runtime_dependency_closure_checks(overlay, package, launch, blockers))
    checks["launch_binds_dosod_edgesam_and_project_adapters"] = all(
        token in launch
        for token in (
            'package="hobot_dosod"', 'executable="hobot_dosod"',
            'package="mono_edgesam"', 'executable="mono_edgesam"',
            'executable="rgb_to_nv12_adapter"', 'executable="open_vocab_product_adapter"',
            '"dosod_model_path"', '"edgesam_encoder_model_path"', '"edgesam_decoder_model_path"',
        )
    )
    checks["profile_keeps_four_artifact_roles_blocked"] = all(
        token in profile
        for token in (
            "blocked_until_four_project_model_roles_are_present_and_hashed",
            "dosod_hbm:", "dosod_vocabulary:", "edgesam_encoder_hbm:", "edgesam_decoder_hbm:",
        )
    )
    checks["ros_package_and_entrypoint_bindings_present"] = all(
        token in package + interfaces + setup
        for token in (
            "sanitation_perception", "sanitation_perception_interfaces", "ai_msgs",
            "open_vocab_product_adapter = sanitation_perception.s100p_product_adapter:main",
            "rgb_to_nv12_adapter = sanitation_perception.rgb_to_nv12_adapter:main",
        )
    )
    return checks


def _snapshot_binding(
    root: Path, declaration: Any, blockers: list[str]
) -> dict[str, bool]:
    checks = {"formal_snapshot_file_matches_declaration": False, "formal_snapshot_content_matches_declaration": False}
    if not isinstance(declaration, Mapping):
        _append(blockers, "formal_snapshot_binding_invalid")
        return checks
    relative = _relative(declaration.get("path"))
    if relative != "reports/engineering/formal_vehicle_snapshot_manifest.json":
        _append(blockers, "formal_snapshot_path_invalid")
        return checks
    path = root / relative
    try:
        if not path.is_file():
            _append(blockers, "formal_snapshot_missing")
            return checks
        checks["formal_snapshot_file_matches_declaration"] = (
            path.stat().st_size == declaration.get("byte_size")
            and _sha256(path) == declaration.get("sha256")
        )
    except (OSError, ValueError) as exc:
        _append(blockers, f"formal_snapshot_unreadable:{type(exc).__name__}")
        return checks
    if not checks["formal_snapshot_file_matches_declaration"]:
        _append(blockers, "formal_snapshot_digest_or_size_mismatch")
        return checks
    snapshot, error = _load_json(path)
    if error:
        _append(blockers, f"formal_snapshot_{error}")
        return checks
    assert snapshot is not None
    outputs = snapshot.get("outputs")
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf") if isinstance(outputs, Mapping) else None
    checks["formal_snapshot_content_matches_declaration"] = (
        snapshot.get("source_inventory_sha256") == declaration.get("source_inventory_sha256")
        and snapshot.get("output_inventory_sha256") == declaration.get("output_inventory_sha256")
        and isinstance(urdf, Mapping)
        and urdf.get("sha256") == declaration.get("formal_urdf_sha256")
    )
    if not checks["formal_snapshot_content_matches_declaration"]:
        _append(blockers, "formal_snapshot_identity_content_mismatch")
    return checks


def validate_manifest(
    manifest_path: str | Path = DEFAULT_MANIFEST, *, repository_root: str | Path = ROOT
) -> dict[str, Any]:
    """Validate a copyable manifest and return BLOCKED until real board gates exist."""
    root = Path(repository_root).resolve()
    payload, error = _load_json(Path(manifest_path).resolve())
    blockers: list[str] = []
    if error:
        return {
            "schema_version": 1, "report_id": "tzcup_s100p_formal_board_bundle_validation_v1",
            "status": "BLOCKED", "ready_to_deploy": False,
            "blockers": [f"bundle_manifest_{error}"], "checks": {"manifest_parseable": False},
        }
    assert payload is not None
    checks: dict[str, bool] = {"manifest_parseable": True}
    checks["manifest_identity_valid"] = (
        payload.get("schema_version") == 1
        and payload.get("bundle_id") == "tzcup_s100p_formal_snapshot_bound_board_bundle_v1"
        and payload.get("status") == "COPYABLE_MANIFEST_ONLY_BLOCKED_DEPLOYMENT"
        and payload.get("operation_boundary") == "manifest_and_sha_validation_only_no_board_copy_ssh_install_node_start_data_collection_or_model_generation"
    )
    if not checks["manifest_identity_valid"]:
        _append(blockers, "bundle_identity_or_operation_boundary_invalid")
    copy = payload.get("copy_boundary")
    checks["copy_boundary_fail_closed"] = isinstance(copy, Mapping) and (
        copy.get("manifest_is_copyable") is True and copy.get("payload_copy_authorized") is False
        and copy.get("board_destination_is_a_plan_only") is True
    )
    if not checks["copy_boundary_fail_closed"]:
        _append(blockers, "copy_boundary_invalid_or_payload_copy_authorized")
    snapshot = payload.get("formal_snapshot")
    checks["snapshot_binding_declared"] = isinstance(snapshot, Mapping) and (
        snapshot.get("path") == "reports/engineering/formal_vehicle_snapshot_manifest.json"
        and snapshot.get("live_source_revalidation_performed") is False
        and isinstance(snapshot.get("sha256"), str) and len(snapshot["sha256"]) == 64
    )
    if not checks["snapshot_binding_declared"]:
        _append(blockers, "formal_snapshot_binding_invalid")
    checks.update(_snapshot_binding(root, snapshot, blockers))
    binding_checks, source_results = _source_bindings(root, payload.get("bound_sources"), blockers)
    checks.update(binding_checks)
    semantic = _semantic_checks(root, payload.get("required_board_payload_roles"), blockers)
    checks.update(semantic)
    gates = payload.get("deployment_gates")
    checks["deployment_gates_explicitly_unaccepted"] = isinstance(gates, Mapping) and all(
        gates.get(name) is False
        for name in (
            "project_model_weights_and_hbm_present_and_hashed", "board_tzcup_overlay_built_and_verified",
            "board_runtime_dependencies_verified", "board_thermal_and_power_measured",
            "board_runtime_evidence_collected", "board_runtime_accepted",
        )
    )
    if not checks["deployment_gates_explicitly_unaccepted"]:
        _append(blockers, "deployment_gates_not_explicitly_fail_closed")
    declared = payload.get("blocked_reasons")
    checks["mandatory_blockers_declared"] = isinstance(declared, list) and MANDATORY_BLOCKERS <= set(declared)
    if not checks["mandatory_blockers_declared"]:
        _append(blockers, "mandatory_board_blockers_missing_from_manifest")
    for blocker in sorted(MANDATORY_BLOCKERS):
        _append(blockers, blocker)
    return {
        "schema_version": 1,
        "report_id": "tzcup_s100p_formal_board_bundle_validation_v1",
        "operation": "static_manifest_and_sha_validation_only",
        "status": "BLOCKED",
        "ready_to_deploy": False,
        "manifest_copyable": checks["copy_boundary_fail_closed"],
        "payload_copy_authorized": False,
        "board_operations_performed": False,
        "formal_snapshot_live_source_revalidated": False,
        "checks": checks,
        "bound_source_results": source_results,
        "blockers": blockers,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--repository-root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--allow-blocked-exit-zero", action="store_true")
    args = parser.parse_args()
    report = validate_manifest(args.manifest, repository_root=args.repository_root)
    rendered = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        output = args.output
        if not output.is_absolute():
            output = args.repository_root.resolve() / output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0 if args.allow_blocked_exit_zero else 2


if __name__ == "__main__":
    raise SystemExit(main())
