#!/usr/bin/env python3
"""Fail-closed, Windows-safe freshness audit for four formal Gazebo chains.

This audit performs no build and launches no runtime process.  It only checks
the source-level runner/validator/aggregate wiring and whether the current
formal-vehicle snapshot is bound to four *fresh*, session-bound formal reports:
forward/stop, physical grasp-and-bin, ground-dirt cleaning, and water recovery.
Retained artifacts are historical references; they cannot become current merely
because their JSON says PASS.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path
import re
import shlex
from typing import Any

import yaml

from formal_final_runtime_closure import (
    FINAL_RUNTIME_PACKAGES,
    FORMAL_RUNTIME_CONTRACT_REVISION,
)
from generate_formal_vehicle_snapshot import SnapshotError, verify_snapshot


ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT_RELATIVE = Path("reports/engineering/formal_vehicle_snapshot_manifest.json")
SESSION_RELATIVE = Path("artifacts/formal_final_acceptance_session.json")
CONTRACT_RELATIVE = Path("config/high_fidelity_vehicle/formal_functional_acceptance_contract.yaml")
DEFAULT_OUTPUT = ROOT / "reports/engineering/formal_four_chain_runtime_readiness.json"

CHAIN_SPECS = {
    "forward_brake": {
        "gate": "a300_drivetrain_runtime",
        "runner": "scripts/run_formal_vehicle_mobility_runtime.sh",
        "validator": "scripts/validate_formal_vehicle_mobility_runtime.py",
        "launches": ("starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py",),
        "runner_mode": "unconditional",
        "validator_mode": "live",
    },
    "physical_grasp_and_bin": {
        "gate": "physical_grasp_and_bin",
        "runner": "scripts/run_formal_grasp_executor_runtime.sh",
        "validator": "scripts/validate_formal_grasp_executor_runtime.py",
        "launches": (
            "starter_ws/src/sanitation_manipulation/launch/formal_cube_pick_place.launch.py",
            "starter_ws/src/sanitation_manipulation/launch/formal_physical_grasp.launch.py",
        ),
        "runner_mode": "unconditional",
        "validator_mode": "live",
    },
    "ground_dirt_cleaning": {
        "gate": "ground_dirt_cleaning",
        "runner": "scripts/run_formal_ground_dirt_cleaning_runtime.sh",
        "validator": "scripts/validate_formal_ground_dirt_cleaning_runtime.py",
        "launches": ("starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py",),
        "runner_mode": "unconditional",
        "validator_mode": "live",
    },
    "water_recovery": {
        "gate": "water_recovery",
        "runner": "scripts/run_formal_water_recovery_runtime.sh",
        "validator": "scripts/finalize_formal_water_recovery_acceptance.py",
        "launches": ("starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py",),
        "runner_mode": "canonical_all",
        "validator_mode": "water_finalizer",
    },
}
AGGREGATE_RELATIVE = Path("scripts/run_formal_final_acceptance.py")


def _scheduled_runner_names(path: Path) -> tuple[set[str], str | None]:
    """Read runner literals only from the authoritative STEP_SPECS assignment."""

    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except (OSError, UnicodeError, SyntaxError) as exc:
        return set(), str(exc)
    for node in tree.body:
        value: ast.AST | None = None
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "STEP_SPECS"
            for target in node.targets
        ):
            value = node.value
        elif (
            isinstance(node, ast.AnnAssign)
            and isinstance(node.target, ast.Name)
            and node.target.id == "STEP_SPECS"
        ):
            value = node.value
        if value is None:
            continue
        if not isinstance(value, (ast.Tuple, ast.List)):
            return set(), "STEP_SPECS must be a direct tuple or list literal"
        runners: set[str] = set()
        for call in value.elts:
            if not isinstance(call, ast.Call) or not (
                isinstance(call.func, ast.Name) and call.func.id == "StepSpec"
            ):
                return set(), "STEP_SPECS contains a non-StepSpec entry"
            runner_node: ast.AST | None = call.args[3] if len(call.args) >= 4 else None
            if runner_node is None:
                for keyword in call.keywords:
                    if keyword.arg == "runner":
                        runner_node = keyword.value
                        break
            if runner_node is None or (
                isinstance(runner_node, ast.Constant) and runner_node.value is None
            ):
                continue
            if isinstance(runner_node, ast.Constant) and isinstance(
                runner_node.value, str
            ):
                runners.add(runner_node.value)
            else:
                return set(), "STEP_SPECS runner must be a string literal or omitted"
        return runners, None
    return set(), "STEP_SPECS assignment is missing"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1_048_576), b""):
            digest.update(block)
    return digest.hexdigest()


def _json_digest(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _read_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _nested(payload: dict[str, Any], dotted_path: str) -> Any:
    value: Any = payload
    for key in dotted_path.split("."):
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


def snapshot_identity(snapshot_path: Path) -> dict[str, str]:
    """Return every snapshot digest an acceptance report must bind."""

    snapshot = _read_json(snapshot_path)
    if snapshot is None:
        raise ValueError("snapshot manifest is not a JSON object")
    outputs = snapshot.get("outputs")
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf") if isinstance(outputs, dict) else None
    source_hash = snapshot.get("source_inventory_sha256")
    output_hash = snapshot.get("output_inventory_sha256")
    urdf_hash = urdf.get("sha256") if isinstance(urdf, dict) else None
    if not all(isinstance(value, str) and value for value in (source_hash, output_hash, urdf_hash)):
        raise ValueError("snapshot lacks source, output, or expanded-URDF hashes")
    return {
        "snapshot_manifest_sha256": _sha256(snapshot_path),
        "source_inventory_sha256": source_hash,
        "output_inventory_sha256": output_hash,
        "expanded_urdf_sha256": urdf_hash,
    }


def _load_contract(root: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load((root / CONTRACT_RELATIVE).read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot read formal functional acceptance contract: {exc}") from exc
    if not isinstance(value, dict) or not isinstance(value.get("evidence_gates"), dict):
        raise ValueError("formal functional acceptance contract lacks evidence_gates")
    return value


def _shell_executable_text(text: str) -> str:
    """Remove shell comments while preserving quoted executable content."""

    rows: list[str] = []
    for line in text.splitlines():
        output: list[str] = []
        quote: str | None = None
        escaped = False
        for character in line:
            if escaped:
                output.append(character)
                escaped = False
                continue
            if character == "\\" and quote != "'":
                output.append(character)
                escaped = True
                continue
            if quote is not None:
                output.append(character)
                if character == quote:
                    quote = None
                continue
            if character in {"'", '"'}:
                quote = character
                output.append(character)
            elif character == "#":
                break
            else:
                output.append(character)
        rows.append("".join(output))
    return "\n".join(rows)


def _shell_logical_lines(text: str) -> list[tuple[int, str]]:
    """Return comment-free, backslash-joined shell lines with source locations."""

    logical: list[tuple[int, str]] = []
    start_line: int | None = None
    pending = ""
    for line_no, row in enumerate(_shell_executable_text(text).splitlines(), start=1):
        stripped = row.rstrip()
        if start_line is None:
            start_line = line_no
        if stripped.endswith("\\"):
            pending += stripped[:-1] + " "
            continue
        logical.append((start_line, pending + stripped))
        start_line = None
        pending = ""
    if start_line is not None:
        logical.append((start_line, pending))
    return logical


def _is_function_start(line: str) -> bool:
    return bool(re.match(r"^\s*(?:function\s+)?[A-Za-z_][A-Za-z0-9_]*\s*\(\)\s*\{\s*$", line))


def _is_canonical_all_guard(line: str) -> bool:
    return line.strip() == 'if [[ "${scenario}" == "all" ]]; then'


def _shell_direct_python_commands(text: str) -> list[tuple[int, list[str], tuple[str, ...]]]:
    """Read only direct Python commands on the canonical shell control path.

    This deliberately recognizes a narrow shell subset.  A token inside an
    assignment, echo, function body, compound command, comment, or unsupported
    conditional is not evidence that the runner executes the binding tool.
    """

    commands: list[tuple[int, list[str], tuple[str, ...]]] = []
    guards: list[str] = []
    function_depth = 0
    for line_no, line in _shell_logical_lines(text):
        stripped = line.strip()
        if not stripped:
            continue
        if _is_function_start(stripped):
            function_depth += 1
            continue
        if stripped == "}" and function_depth:
            function_depth -= 1
            continue
        if stripped.startswith("if ") and stripped.endswith("then"):
            guards.append("canonical_all" if _is_canonical_all_guard(stripped) else "other")
            continue
        if stripped == "fi" and guards:
            guards.pop()
            continue
        if function_depth or not stripped.startswith("python3 "):
            continue
        try:
            argv = shlex.split(stripped, posix=True)
        except ValueError:
            continue
        if argv and argv[0] == "python3":
            commands.append((line_no, argv, tuple(guards)))
    return commands


def _shell_top_level_fail_fast_lines(text: str) -> set[int]:
    """Return unconditional, top-level set -e lines only."""

    lines: set[int] = set()
    guards: list[str] = []
    function_depth = 0
    for line_no, line in _shell_logical_lines(text):
        stripped = line.strip()
        if _is_function_start(stripped):
            function_depth += 1
            continue
        if stripped == "}" and function_depth:
            function_depth -= 1
            continue
        if stripped.startswith("if ") and stripped.endswith("then"):
            guards.append("guard")
            continue
        if stripped == "fi" and guards:
            guards.pop()
            continue
        if (
            not function_depth
            and not guards
            and re.match(r"^set\s+-[A-Za-z]*e[A-Za-z]*\b", stripped)
        ):
            lines.add(line_no)
    return lines


def _has_flag(argv: list[str], flag: str, value: str | None = None) -> bool:
    try:
        index = argv.index(flag)
    except ValueError:
        return False
    return value is None or index + 1 < len(argv) and argv[index + 1] == value


def _runner_wiring_errors(text: str, mode: str) -> list[str]:
    """Require ordered, direct execution of the snapshot and binding commands."""

    commands = _shell_direct_python_commands(text)
    allowed_guard = () if mode == "unconditional" else ("canonical_all",)
    snapshot_candidates = [
        (line_no, argv)
        for line_no, argv, guards in commands
        if guards == allowed_guard
        and len(argv) >= 2
        and argv[1].endswith("/scripts/generate_formal_vehicle_snapshot.py")
        and _has_flag(argv, "--check")
        and _has_flag(argv, "--output", "${snapshot}")
    ]
    binding_candidates = [
        (line_no, argv)
        for line_no, argv, guards in commands
        if guards == allowed_guard
        and len(argv) >= 2
        and argv[1].endswith("/scripts/formal_runtime_gate_binding.py")
        and all(
            (
                _has_flag(argv, "--repository-root"),
                _has_flag(argv, "--install-root"),
                _has_flag(argv, "--closure-manifest"),
                _has_flag(argv, "--session", "${session}"),
                _has_flag(argv, "--snapshot", "${snapshot}"),
                _has_flag(argv, "--output", "${runtime_binding}"),
            )
        )
    ]
    errors: list[str] = []
    if not snapshot_candidates:
        errors.append("runner lacks a direct snapshot --check command")
    if not binding_candidates:
        errors.append("runner lacks a direct runtime-binding command")
    if snapshot_candidates and binding_candidates:
        if min(line_no for line_no, _ in binding_candidates) < min(
            line_no for line_no, _ in snapshot_candidates
        ):
            errors.append("runner binds runtime before the snapshot --check")
    first_required_line = min(
        [line_no for line_no, _ in snapshot_candidates + binding_candidates], default=None
    )
    if first_required_line is not None:
        if not any(
            line_no < first_required_line
            for line_no in _shell_top_level_fail_fast_lines(text)
        ):
            errors.append("runner lacks fail-fast set -e before binding commands")
    return errors


def _direct_calls(statements: list[ast.stmt]) -> list[tuple[int, ast.Call]]:
    """Calls in direct statements only; branches and nested functions are excluded."""

    calls: list[tuple[int, ast.Call]] = []
    for index, statement in enumerate(statements):
        if not isinstance(statement, (ast.Assign, ast.AnnAssign, ast.Expr, ast.Return)):
            continue
        for node in ast.walk(statement):
            if isinstance(node, ast.Call):
                calls.append((index, node))
    return calls


def _call_name(call: ast.Call) -> str | None:
    if isinstance(call.func, ast.Name):
        return call.func.id
    if isinstance(call.func, ast.Attribute):
        if isinstance(call.func.value, ast.Name):
            return f"{call.func.value.id}.{call.func.attr}"
        return call.func.attr
    return None


def _contains_call_not_under_literal_false(node: ast.AST, name: str) -> bool:
    """Find a call while rejecting code hidden beneath ``if False``."""

    if isinstance(node, ast.If):
        if _is_literal_false(node.test):
            return any(_contains_call_not_under_literal_false(item, name) for item in node.orelse)
        return any(
            _contains_call_not_under_literal_false(item, name)
            for item in (*node.body, *node.orelse)
        )
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
        return False
    if isinstance(node, ast.Call) and _call_name(node) == name:
        return True
    return any(_contains_call_not_under_literal_false(child, name) for child in ast.iter_child_nodes(node))


def _is_literal_false(node: ast.AST) -> bool:
    if isinstance(node, ast.Constant):
        return node.value in (False, None, 0, "")
    if isinstance(node, (ast.Tuple, ast.List, ast.Set)):
        return not node.elts
    return isinstance(node, ast.Dict) and not node.keys


def _is_args_runtime_binding(node: ast.AST) -> bool:
    return (
        isinstance(node, ast.Attribute)
        and node.attr == "runtime_binding"
        and isinstance(node.value, ast.Name)
        and node.value.id == "args"
    )


def _validator_wiring_errors(text: str, path: Path, mode: str) -> list[str]:
    """Verify the validator's executed entrypoint binds evidence before use."""

    try:
        tree = ast.parse(text, filename=str(path))
    except SyntaxError as exc:
        return [f"validator Python parse failed: {exc}"]
    functions = {
        node.name: node
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    main = functions.get("main")
    if main is None:
        return ["validator has no module-level main entrypoint"]
    main_calls = _direct_calls(main.body)
    errors: list[str] = []
    if mode == "live":
        bound_positions = [
            index for index, call in main_calls if _call_name(call) == "_bound_runtime_evidence"
        ]
        runtime_positions = [
            index for index, call in main_calls if _call_name(call) in {"rclpy.init", "run"}
        ]
        if not bound_positions:
            errors.append("validator main does not directly call _bound_runtime_evidence")
        if not runtime_positions:
            errors.append("validator main has no direct runtime execution boundary")
        elif bound_positions and min(bound_positions) > min(runtime_positions):
            errors.append("validator binds runtime evidence after runtime execution begins")
        helper = functions.get("_bound_runtime_evidence")
        if helper is None or not any(
            _call_name(call) == "load_binding" for _, call in _direct_calls(helper.body)
        ):
            errors.append("_bound_runtime_evidence does not directly load runtime binding")
    elif mode == "water_finalizer":
        argument_calls = [
            call for _, call in main_calls if _call_name(call) == "parser.add_argument"
        ]
        has_required_argument = any(
            call.args
            and isinstance(call.args[0], ast.Constant)
            and call.args[0].value == "--runtime-binding"
            and any(
                keyword.arg == "required"
                and isinstance(keyword.value, ast.Constant)
                and keyword.value.value is True
                for keyword in call.keywords
            )
            for call in argument_calls
        )
        combine_calls = [
            call for _, call in main_calls if _call_name(call) == "combine"
        ]
        passes_runtime_binding = any(
            any(_is_args_runtime_binding(argument) for argument in call.args)
            or any(
                keyword.arg == "runtime_binding_path"
                and _is_args_runtime_binding(keyword.value)
                for keyword in call.keywords
            )
            for call in combine_calls
        )
        if not has_required_argument:
            errors.append("water finalizer main does not require --runtime-binding")
        if not passes_runtime_binding:
            errors.append("water finalizer main does not pass args.runtime_binding to combine")
        combine = functions.get("combine")
        if combine is None or not any(
            _contains_call_not_under_literal_false(statement, "load_binding")
            for statement in combine.body
        ):
            errors.append("water finalizer combine does not execute load_binding")
    else:
        errors.append(f"unknown validator wiring mode: {mode}")
    return errors


def _source_wiring(root: Path, spec: dict[str, Any]) -> tuple[bool, list[str], dict[str, str]]:
    errors: list[str] = []
    hashes: dict[str, str] = {}
    for field in ("runner", "validator"):
        relative = Path(str(spec[field]))
        path = root / relative
        if not path.is_file():
            errors.append(f"missing {field}: {relative.as_posix()}")
            continue
        text = path.read_text(encoding="utf-8")
        if field == "runner":
            errors.extend(_runner_wiring_errors(text, str(spec["runner_mode"])))
        else:
            errors.extend(_validator_wiring_errors(text, path, str(spec["validator_mode"])))
        hashes[relative.as_posix()] = _sha256(path)
    for relative_text in spec["launches"]:
        relative = Path(relative_text)
        path = root / relative
        if not path.is_file():
            errors.append(f"missing launch: {relative.as_posix()}")
        else:
            hashes[relative.as_posix()] = _sha256(path)
    return not errors, errors, hashes


def _expected_sidecar(report_path: Path, gate: dict[str, Any]) -> Path | None:
    binding = gate.get("runtime_binding")
    if not isinstance(binding, dict) or not isinstance(binding.get("sidecar_suffix"), str):
        return None
    return report_path.with_name(report_path.name + binding["sidecar_suffix"])


def _validate_runtime_closure_binding(closure: Any) -> tuple[list[str], dict[str, str] | None]:
    """Bind a sidecar to one readable, canonical frozen-runtime manifest."""

    blockers: list[str] = []
    if not isinstance(closure, dict):
        return ["RUNTIME_CLOSURE_BINDING_INVALID"], None
    if closure.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED":
        blockers.append("RUNTIME_CLOSURE_STATUS_INVALID")
    if closure.get("symbolic_link_count") != 0:
        blockers.append("RUNTIME_CLOSURE_SYMBOLIC_LINKS_PRESENT")

    manifest_text = closure.get("manifest")
    install_text = closure.get("runtime_install_root")
    manifest_hash = closure.get("manifest_sha256")
    closure_hash = closure.get("closure_sha256")
    if not all(
        isinstance(value, str) and value
        for value in (manifest_text, install_text, manifest_hash, closure_hash)
    ):
        blockers.append("RUNTIME_CLOSURE_IDENTITY_INCOMPLETE")
        return blockers, None

    manifest_path = Path(manifest_text)
    install_root = Path(install_text)
    if not manifest_path.is_absolute() or str(manifest_path.resolve()) != manifest_text:
        blockers.append("RUNTIME_CLOSURE_MANIFEST_PATH_NOT_CANONICAL")
    if not install_root.is_absolute() or str(install_root.resolve()) != install_text:
        blockers.append("RUNTIME_INSTALL_ROOT_NOT_CANONICAL")
    if manifest_path.is_symlink() or not manifest_path.is_file():
        blockers.append("RUNTIME_CLOSURE_MANIFEST_MISSING_OR_SYMLINK")
        manifest = None
    else:
        manifest = _read_json(manifest_path)
        if _sha256(manifest_path) != manifest_hash:
            blockers.append("RUNTIME_CLOSURE_MANIFEST_HASH_MISMATCH")
    if install_root.is_symlink() or not install_root.is_dir():
        blockers.append("RUNTIME_INSTALL_ROOT_MISSING_OR_SYMLINK")
    if manifest is None:
        blockers.append("RUNTIME_CLOSURE_MANIFEST_INVALID")
    else:
        stored_closure = manifest.get("closure")
        if (
            manifest.get("schema_version") != 6
            or manifest.get("kind") != "tzcup_formal_final_runtime_closure"
            or manifest.get("status")
            != "FORMAL_FINAL_RUNTIME_CLOSURE_FROZEN"
            or manifest.get("runtime_contract_revision")
            != FORMAL_RUNTIME_CONTRACT_REVISION
            or not isinstance(stored_closure, dict)
        ):
            blockers.append("RUNTIME_CLOSURE_MANIFEST_SCHEMA_INVALID")
        elif (
            stored_closure.get("runtime_packages")
            != list(FINAL_RUNTIME_PACKAGES)
            or stored_closure.get("symbolic_link_count") != 0
            or _json_digest(stored_closure) != manifest.get("closure_sha256")
            or manifest.get("closure_sha256") != closure_hash
        ):
            blockers.append("RUNTIME_CLOSURE_MANIFEST_IDENTITY_MISMATCH")
        else:
            runtime_ws_text = stored_closure.get("runtime_ws")
            if not isinstance(runtime_ws_text, str) or not runtime_ws_text:
                blockers.append("RUNTIME_CLOSURE_RUNTIME_WS_MISSING")
            else:
                runtime_ws = Path(runtime_ws_text)
                if (
                    not runtime_ws.is_absolute()
                    or str(runtime_ws.resolve()) != runtime_ws_text
                    or (runtime_ws / "install").resolve() != install_root.resolve()
                ):
                    blockers.append("RUNTIME_CLOSURE_RUNTIME_WS_MISMATCH")

    identity = None
    if not blockers:
        identity = {
            "manifest": manifest_text,
            "manifest_sha256": manifest_hash,
            "closure_sha256": closure_hash,
            "runtime_install_root": install_text,
        }
    return blockers, identity


def evaluate_fresh_report(
    *,
    report_path: Path,
    gate: dict[str, Any],
    expected_snapshot: dict[str, str],
    session_path: Path,
) -> dict[str, Any]:
    """Assess one report as a fresh formal report, never as historical proof."""

    blockers: list[str] = []
    report = _read_json(report_path)
    if report is None:
        return {"fresh": False, "classification": "MISSING", "blockers": ["FRESH_FORMAL_REPORT_MISSING"], "report_path": str(report_path)}
    session = _read_json(session_path)
    if session is None:
        blockers.append("ACTIVE_ACCEPTANCE_SESSION_MISSING")
        started_ns = None
    else:
        started_ns = session.get("started_epoch_ns")
        if session.get("status") == "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE":
            blockers.append("SESSION_FINALIZED_NOT_ACTIVE")
        elif session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
            blockers.append("ACTIVE_ACCEPTANCE_SESSION_INVALID")
        if not isinstance(started_ns, int) or started_ns <= 0:
            blockers.append("ACTIVE_ACCEPTANCE_SESSION_INVALID")
        elif session.get("snapshot") != {
            key: value
            for key, value in expected_snapshot.items()
            if key != "output_inventory_sha256"
        }:
            # A report-side binding is never enough on its own: the live
            # session must still name the exact current snapshot.  Otherwise
            # an older session/sidecar pair can be copied next to a report and
            # look fresh merely because its files were touched recently.
            blockers.append("ACTIVE_ACCEPTANCE_SESSION_SNAPSHOT_MISMATCH")
    if report_path.is_symlink():
        blockers.append("REPORT_SYMBOLIC_LINK_FORBIDDEN")
    if report.get("report_id") != gate.get("report_id"):
        blockers.append("REPORT_ID_MISMATCH")
    if report.get("status") not in gate.get("success_statuses", []):
        blockers.append("FORMAL_RUNTIME_STATUS_NOT_PASSED")
    for dotted_path, expected in gate.get("required_values", {}).items():
        if _nested(report, dotted_path) != expected:
            blockers.append(f"REQUIRED_VALUE_MISMATCH:{dotted_path}")
    for field, expected in expected_snapshot.items():
        report_field = {
            "snapshot_manifest_sha256": gate.get("snapshot_manifest_hash_field"),
            "expanded_urdf_sha256": gate.get("snapshot_urdf_hash_field"),
            "source_inventory_sha256": gate.get("snapshot_source_hash_field"),
            # Every new freshness audit also requires the current output
            # inventory digest even if the legacy contract predates this field.
            "output_inventory_sha256": "runtime_gate_binding.acceptance_session_binding.snapshot_output_inventory_sha256",
        }[field]
        if not isinstance(report_field, str) or _nested(report, report_field) != expected:
            blockers.append(f"SNAPSHOT_BINDING_MISMATCH:{field}")
    sidecar_path = _expected_sidecar(report_path, gate)
    sidecar = _read_json(sidecar_path) if sidecar_path is not None else None
    if sidecar is None:
        blockers.append("RUNTIME_BINDING_SIDECAR_MISSING")
    else:
        if sidecar_path is not None and sidecar_path.is_symlink():
            blockers.append("RUNTIME_BINDING_SYMBOLIC_LINK_FORBIDDEN")
        report_binding_field = gate.get("runtime_binding", {}).get("report_field")
        if not isinstance(report_binding_field, str) or _nested(report, report_binding_field) != sidecar:
            blockers.append("RUNTIME_BINDING_SIDECAR_MISMATCH")
        if sidecar.get("status") != "FORMAL_RUNTIME_GATE_BOUND":
            blockers.append("RUNTIME_BINDING_NOT_BOUND")
        expected_runtime_snapshot = {
            key: value
            for key, value in expected_snapshot.items()
            if key != "output_inventory_sha256"
        }
        session_binding = _nested(sidecar, "acceptance_session_binding")
        if not isinstance(session_binding, dict):
            blockers.append("RUNTIME_BINDING_SESSION_MISSING")
        elif (
            session_binding.get("snapshot") != expected_runtime_snapshot
            or session_binding.get("snapshot_output_inventory_sha256")
            != expected_snapshot["output_inventory_sha256"]
            or session_binding.get("session_status_at_gate")
            != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
            or session_binding.get("snapshot_current_source_verified") is not True
        ):
            blockers.append("RUNTIME_BINDING_SNAPSHOT_MISMATCH")
        elif (
            session is None
            or session_binding.get("session_manifest_sha256")
            != _sha256(session_path)
            or session_binding.get("session_started_epoch_ns") != started_ns
        ):
            blockers.append("RUNTIME_BINDING_SESSION_MANIFEST_MISMATCH")
        closure_binding = _nested(sidecar, "runtime_closure_binding")
        closure_blockers, closure_identity = _validate_runtime_closure_binding(
            closure_binding
        )
        blockers.extend(closure_blockers)
        session_closure = (
            session.get("runtime_closure_binding")
            if isinstance(session, dict)
            else None
        )
        if not isinstance(session_closure, dict):
            blockers.append("SESSION_RUNTIME_CLOSURE_BINDING_MISSING")
        elif isinstance(closure_binding, dict):
            closure_session_fields = (
                "status",
                "manifest",
                "manifest_sha256",
                "closure_sha256",
                "runtime_install_root",
                "symbolic_link_count",
            )
            if any(
                session_closure.get(field) != closure_binding.get(field)
                for field in closure_session_fields
            ):
                blockers.append("SESSION_RUNTIME_CLOSURE_BINDING_MISMATCH")
        verified_ns = sidecar.get("verified_epoch_ns")
        if not isinstance(verified_ns, int) or (
            isinstance(started_ns, int)
            and (verified_ns < started_ns or verified_ns > report_path.stat().st_mtime_ns)
        ):
            blockers.append("RUNTIME_BINDING_VERIFICATION_TIME_INVALID")
    if isinstance(started_ns, int):
        for path, label in ((report_path, "REPORT"), (sidecar_path, "RUNTIME_BINDING")):
            if path is None or not path.is_file() or path.stat().st_mtime_ns < started_ns:
                blockers.append(f"{label}_PREDATES_ACTIVE_SESSION")
    return {
        "fresh": not blockers,
        "classification": "FRESH" if not blockers else "HISTORICAL_OR_INVALID",
        "blockers": blockers,
        "report_path": str(report_path),
        "runtime_binding_path": str(sidecar_path) if sidecar_path else None,
        "runtime_closure_identity": closure_identity if sidecar is not None else None,
    }


def audit(root: Path = ROOT) -> dict[str, Any]:
    """Audit four chains without starting Gazebo or reading historical PASS as current."""

    root = root.resolve()
    snapshot_path = root / SNAPSHOT_RELATIVE
    session_path = root / SESSION_RELATIVE
    try:
        identity = snapshot_identity(snapshot_path)
        try:
            verify_snapshot(root, snapshot_path)
            snapshot_current = True
            snapshot_error = None
        except (SnapshotError, OSError, UnicodeError, json.JSONDecodeError) as exc:
            snapshot_current = False
            snapshot_error = str(exc)
    except (OSError, ValueError) as exc:
        identity = {}
        snapshot_current = False
        snapshot_error = str(exc)
    try:
        contract = _load_contract(root)
        gates = contract["evidence_gates"]
        contract_error = None
    except ValueError as exc:
        gates = {}
        contract_error = str(exc)

    aggregate_path = root / AGGREGATE_RELATIVE
    scheduled_runners, aggregate_error = _scheduled_runner_names(aggregate_path)
    chains: dict[str, Any] = {}
    all_source_wiring = aggregate_error is None
    for chain_id, spec in CHAIN_SPECS.items():
        source_ready, source_errors, source_hashes = _source_wiring(root, spec)
        gate = gates.get(spec["gate"])
        if not isinstance(gate, dict):
            source_ready = False
            source_errors.append(f"missing acceptance contract gate: {spec['gate']}")
            evidence = {"fresh": False, "classification": "MISSING", "blockers": ["FORMAL_ACCEPTANCE_GATE_MISSING"]}
        else:
            if not isinstance(gate.get("report_id"), str) or not gate["report_id"]:
                source_ready = False
                source_errors.append("acceptance contract gate lacks a stable report_id")
            report_relative = gate.get("path")
            if not isinstance(report_relative, str) or Path(report_relative).is_absolute() or ".." in Path(report_relative).parts:
                source_ready = False
                source_errors.append("acceptance report path is invalid")
                evidence = {"fresh": False, "classification": "MISSING", "blockers": ["FORMAL_ACCEPTANCE_REPORT_PATH_INVALID"]}
            elif not identity:
                evidence = {"fresh": False, "classification": "BLOCKED", "blockers": ["CURRENT_SNAPSHOT_IDENTITY_UNAVAILABLE"]}
            else:
                evidence = evaluate_fresh_report(report_path=root / report_relative, gate=gate, expected_snapshot=identity, session_path=session_path)
        if spec["runner"].split("/")[-1] not in scheduled_runners:
            source_ready = False
            source_errors.append("unified final aggregate does not schedule this runner")
        all_source_wiring = all_source_wiring and source_ready
        chains[chain_id] = {"source_wiring_ready": source_ready, "source_wiring_errors": source_errors, "source_sha256": source_hashes, "formal_report": evidence}

    closure_identities = {
        json.dumps(chain["formal_report"].get("runtime_closure_identity"), sort_keys=True)
        for chain in chains.values()
        if chain["formal_report"].get("runtime_closure_identity") is not None
    }
    if len(closure_identities) > 1:
        for chain in chains.values():
            evidence = chain["formal_report"]
            if evidence.get("fresh"):
                evidence["fresh"] = False
                evidence["classification"] = "HISTORICAL_OR_INVALID"
                evidence["blockers"].append(
                    "RUNTIME_CLOSURE_BINDING_DIFFERS_ACROSS_CHAINS"
                )
    all_fresh = snapshot_current and all(chain["formal_report"]["fresh"] for chain in chains.values())
    return {
        "schema_version": 1,
        "report_id": "tzcup_formal_four_chain_runtime_readiness_v1",
        "status": "FORMAL_FOUR_CHAIN_RUNTIME_READY" if all_source_wiring and all_fresh else "FORMAL_FOUR_CHAIN_RUNTIME_BLOCKED",
        "ready_for_formal_runtime_acceptance": all_source_wiring and all_fresh,
        "execution_scope": {"static_read_only": True, "started_gazebo": False, "started_wsl": False, "started_docker": False, "generated_runtime_report": False},
        "snapshot": {"path": str(snapshot_path), "current": snapshot_current, "error": snapshot_error, "identity": identity},
        "aggregate": {
            "path": str(aggregate_path),
            "present": aggregate_path.is_file(),
            "sha256": _sha256(aggregate_path) if aggregate_path.is_file() else None,
            "parse_error": aggregate_error,
            "scheduled_runners": sorted(scheduled_runners),
        },
        "contract_error": contract_error,
        "chains": chains,
        "historical_evidence_policy": "A missing or session/snapshot-stale report remains BLOCKED; retained artifacts are historical reference only.",
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    report = audit(args.root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if report["ready_for_formal_runtime_acceptance"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
