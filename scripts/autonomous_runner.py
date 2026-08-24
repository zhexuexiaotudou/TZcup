#!/usr/bin/env python3
"""Resumable, evidence-first controller for the TZcup AUTO-00..AUTO-16 DAG.

The controller deliberately separates orchestration from stage implementation.
Only stages with an explicit argv-list ``command`` in the registry are
executable. A missing command remains pending and is never reported as passed.
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import os
import platform
import subprocess
import sys
import tempfile
from collections import defaultdict, deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterator, Sequence

import yaml


ROOT = Path(__file__).resolve().parents[1]
REGISTRY_PATH = ROOT / "config" / "autonomous_stage_registry.yaml"
AUTONOMY_CONFIG_DIR = ROOT / "config" / "autonomy"
STATE_PATH = AUTONOMY_CONFIG_DIR / "AUTONOMOUS_STATE.json"
PLAN_PATH = AUTONOMY_CONFIG_DIR / "AUTONOMOUS_RUN_PLAN.json"
LOCK_PATH = ROOT / ".git" / "autonomous-runner.lock"
TERMINAL_STATUSES = {"PASS", "BLOCKED", "BLOCKED_EXTERNAL"}
VALID_STATUSES = {"PENDING", "RUNNING", "PASS", "FAIL", "BLOCKED", "BLOCKED_EXTERNAL"}
HISTORICAL_REVIEW_GLOBS = (
    "artifacts/stage4w_*_review/**",
    "artifacts/stage5a_*_review/**",
    "artifacts/stage5b_*_review/**",
    "artifacts/stage5br*_review/**",
)
EVIDENCE_REQUIRED = (
    "stage_status.json",
    "stage_config.yaml",
    "attempt_ledger.json",
    "environment.json",
    "commands.txt",
    "metrics_summary.json",
    "raw_metric_index.json",
    "regression_summary.json",
    "README.md",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def load_registry(path: Path = REGISTRY_PATH) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return value


def atomic_write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(value, ensure_ascii=False, indent=2) + "\n"
    fd, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as handle:
            handle.write(payload)
        os.replace(temp_name, path)
    finally:
        with contextlib.suppress(FileNotFoundError):
            os.unlink(temp_name)


@contextlib.contextmanager
def exclusive_lock(path: Path = LOCK_PATH) -> Iterator[None]:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    except FileExistsError as exc:
        raise RuntimeError(f"runner lock already exists: {path}") from exc
    try:
        os.write(fd, f"pid={os.getpid()}\nstarted_at={utc_now()}\n".encode())
        os.close(fd)
        yield
    finally:
        with contextlib.suppress(FileNotFoundError):
            path.unlink()


def _graph(registry: dict[str, Any]) -> tuple[dict[str, set[str]], dict[str, set[str]]]:
    stages = registry.get("stages")
    if not isinstance(stages, dict) or not stages:
        raise ValueError("registry.stages must be a non-empty mapping")
    deps: dict[str, set[str]] = {}
    reverse: dict[str, set[str]] = defaultdict(set)
    for stage_id, spec in stages.items():
        if not isinstance(spec, dict):
            raise ValueError(f"{stage_id} registry entry must be a mapping")
        required = spec.get("dependencies", [])
        optional = spec.get("optional_dependencies", [])
        if not isinstance(required, list) or not isinstance(optional, list):
            raise ValueError(f"{stage_id} dependencies must be lists")
        unknown = (set(required) | set(optional)) - set(stages)
        if unknown:
            raise ValueError(f"{stage_id} references unknown stages: {sorted(unknown)}")
        if stage_id in required or stage_id in optional:
            raise ValueError(f"{stage_id} cannot depend on itself")
        deps[stage_id] = set(required)
        for dependency in required:
            reverse[dependency].add(stage_id)
    return deps, reverse


def topological_levels(registry: dict[str, Any]) -> list[list[str]]:
    deps, reverse = _graph(registry)
    remaining = {stage: len(required) for stage, required in deps.items()}
    ready = sorted(stage for stage, count in remaining.items() if count == 0)
    levels: list[list[str]] = []
    seen = 0
    while ready:
        level = ready
        levels.append(level)
        next_ready: list[str] = []
        for stage in level:
            seen += 1
            for child in sorted(reverse.get(stage, ())):
                remaining[child] -= 1
                if remaining[child] == 0:
                    next_ready.append(child)
        ready = sorted(next_ready)
    if seen != len(deps):
        cycle_nodes = sorted(stage for stage, count in remaining.items() if count > 0)
        raise ValueError(f"dependency cycle detected: {cycle_nodes}")
    return levels


def validate_registry(registry: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if registry.get("schema_version") != 1:
        errors.append("registry schema_version must equal 1")
    try:
        topological_levels(registry)
    except ValueError as exc:
        errors.append(str(exc))
    for stage_id, spec in registry.get("stages", {}).items():
        command = spec.get("command")
        if command is not None and (
            not isinstance(command, list)
            or not command
            or not all(isinstance(item, str) and item for item in command)
        ):
            errors.append(f"{stage_id}.command must be null or a non-empty argv list")
    return errors


def validate_state(state: dict[str, Any], registry: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if state.get("schema_version") != 1:
        errors.append("state schema_version must equal 1")
    state_stages = state.get("stages")
    if not isinstance(state_stages, dict):
        return errors + ["state.stages must be a mapping"]
    expected = set(registry.get("stages", {}))
    actual = set(state_stages)
    if actual != expected:
        errors.append(f"state stage set mismatch: missing={sorted(expected-actual)}, extra={sorted(actual-expected)}")
    boundaries = state.get("historical_boundaries", {})
    if boundaries.get("historical_evidence_modified") is not False:
        errors.append("historical_evidence_modified must remain false")
    if boundaries.get("stage5br6a_human_review_completed") is not False:
        errors.append("historical human-review flag must remain false")
    if boundaries.get("stage5br6a_manual_audit_pass") is not False:
        errors.append("historical manual-audit flag must remain false")
    for stage_id, stage in state_stages.items():
        if not isinstance(stage, dict):
            errors.append(f"{stage_id} state must be a mapping")
            continue
        status = stage.get("status")
        if status not in VALID_STATUSES:
            errors.append(f"{stage_id} has invalid status {status!r}")
        if status == "PASS" and stage.get("machine_gate_pass") is not True:
            errors.append(f"{stage_id} PASS requires machine_gate_pass=true")
        expected_deps = registry.get("stages", {}).get(stage_id, {}).get("dependencies", [])
        if stage.get("dependencies") != expected_deps:
            errors.append(f"{stage_id} dependency snapshot differs from registry")
    return errors


def build_plan(registry: dict[str, Any]) -> dict[str, Any]:
    levels = topological_levels(registry)
    lanes = sorted(
        {
            spec.get("lane")
            for spec in registry["stages"].values()
            if spec.get("lane") not in {None, "control", "integration", "release"}
        }
    )
    return {
        "schema_version": 1,
        "program": registry.get("program"),
        "baseline_commit": registry.get("baseline_commit"),
        "generated_from": "config/autonomous_stage_registry.yaml",
        "dependency_cycle_count": 0,
        "topological_levels": levels,
        "independent_lanes": lanes,
        "rules": {
            "pass_is_terminal": True,
            "valid_pass_evidence_is_reused": True,
            "failed_dependency_blocks_only_dependents": True,
            "optional_dependencies_do_not_block_simulation_status": True,
            "historical_evidence_is_read_only": True,
        },
    }


def ready_stages(state: dict[str, Any], registry: dict[str, Any]) -> list[str]:
    ready: list[str] = []
    for stage_id in sum(topological_levels(registry), []):
        stage = state["stages"][stage_id]
        if stage["status"] in TERMINAL_STATUSES or stage["status"] == "RUNNING":
            continue
        dependencies = registry["stages"][stage_id].get("dependencies", [])
        if all(state["stages"][dependency]["status"] == "PASS" for dependency in dependencies):
            ready.append(stage_id)
    return ready


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_manifest(evidence_dir: Path) -> dict[str, Any]:
    files = []
    for path in sorted(evidence_dir.rglob("*")):
        if not path.is_file() or path.name == "artifact_manifest.json":
            continue
        files.append(
            {
                "path": path.relative_to(evidence_dir).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
        )
    return {"schema_version": 1, "root": ".", "files": files}


def verify_manifest(evidence_dir: Path) -> list[str]:
    manifest_path = evidence_dir / "artifact_manifest.json"
    if not manifest_path.is_file():
        return ["artifact_manifest.json is missing"]
    manifest = load_json(manifest_path)
    errors: list[str] = []
    entries = manifest.get("files")
    if not isinstance(entries, list):
        return ["manifest.files must be a list"]
    listed: set[str] = set()
    for entry in entries:
        relative = entry.get("path")
        if not isinstance(relative, str) or relative.startswith(("/", "\\")) or ".." in Path(relative).parts:
            errors.append(f"unsafe manifest path: {relative!r}")
            continue
        listed.add(Path(relative).as_posix())
        path = evidence_dir / relative
        if not path.is_file():
            errors.append(f"missing file: {relative}")
            continue
        if path.stat().st_size != entry.get("bytes"):
            errors.append(f"byte count mismatch: {relative}")
        if sha256_file(path) != entry.get("sha256"):
            errors.append(f"sha256 mismatch: {relative}")
    actual = {
        path.relative_to(evidence_dir).as_posix()
        for path in evidence_dir.rglob("*")
        if path.is_file() and path.name != "artifact_manifest.json"
    }
    if listed != actual:
        errors.append(f"manifest coverage mismatch: missing={sorted(actual-listed)}, extra={sorted(listed-actual)}")
    return errors


def git_output(args: Sequence[str], cwd: Path = ROOT) -> str:
    completed = subprocess.run(
        ["git", *args],
        cwd=cwd,
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
    )
    return completed.stdout.strip()


def historical_hash_audit(root: Path = ROOT) -> dict[str, Any]:
    tracked = git_output(["ls-files", "artifacts"], root).splitlines()
    historical = [
        path
        for path in tracked
        if "_review/" in path
        and any(token in path for token in ("stage4w_", "stage5a_", "stage5b_", "stage5br"))
    ]
    files = []
    for relative in sorted(historical):
        path = root / relative
        files.append({"path": relative, "bytes": path.stat().st_size, "sha256": sha256_file(path)})
    aggregate = hashlib.sha256(
        "\n".join(f"{entry['sha256']}  {entry['path']}" for entry in files).encode()
    ).hexdigest()
    changed = git_output(["diff", "--name-only", "--", "artifacts"], root).splitlines()
    return {
        "schema_version": 1,
        "baseline_commit": git_output(["rev-parse", "HEAD"], root),
        "historical_evidence_modified": bool(changed),
        "changed_historical_paths": changed,
        "file_count": len(files),
        "aggregate_sha256": aggregate,
        "files": files,
    }


def _environment(root: Path) -> dict[str, Any]:
    def probe(argv: list[str]) -> dict[str, Any]:
        try:
            result = subprocess.run(argv, capture_output=True, text=True, timeout=5)
            return {
                "available": result.returncode == 0,
                "returncode": result.returncode,
                "output": (result.stdout or result.stderr).strip()[:4000],
            }
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            return {"available": False, "error": type(exc).__name__}

    return {
        "captured_at": utc_now(),
        "os": platform.platform(),
        "machine": platform.machine(),
        "python": sys.version,
        "git_commit": git_output(["rev-parse", "HEAD"], root),
        "docker": probe(["docker", "info", "--format", "{{json .ServerVersion}}"]),
        "ros2": probe(["ros2", "--version"]),
        "gz": probe(["gz", "--version"]),
        "j6_sdk": {"available": False, "basis": "no official SDK path configured in registry or environment"},
        "random_seeds": [],
    }


def write_standard_evidence(
    root: Path,
    stage_id: str,
    stage_config: dict[str, Any],
    stage_status: dict[str, Any],
    attempts: list[dict[str, Any]],
    commands: list[list[str]],
    metrics: dict[str, Any],
    evidence_dir: Path | None = None,
) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    evidence_dir = evidence_dir or root / "artifacts" / f"autonomous_{stage_id.lower().replace('-', '')}_{stamp}_evidence"
    evidence_dir.mkdir(parents=True, exist_ok=False)
    atomic_write_json(evidence_dir / "stage_status.json", stage_status)
    (evidence_dir / "stage_config.yaml").write_text(
        yaml.safe_dump(stage_config, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
        newline="\n",
    )
    atomic_write_json(evidence_dir / "attempt_ledger.json", {"schema_version": 1, "attempts": attempts})
    atomic_write_json(evidence_dir / "environment.json", _environment(root))
    (evidence_dir / "commands.txt").write_text(
        "\n".join(" ".join(command) for command in commands) + ("\n" if commands else ""),
        encoding="utf-8",
        newline="\n",
    )
    atomic_write_json(evidence_dir / "metrics_summary.json", metrics)
    atomic_write_json(evidence_dir / "raw_metric_index.json", {"schema_version": 1, "raw_metrics": []})
    atomic_write_json(
        evidence_dir / "regression_summary.json",
        {"schema_version": 1, "status": "NOT_RUN" if stage_id == "AUTO-00" else "PENDING", "regressions": []},
    )
    (evidence_dir / "README.md").write_text(
        f"# {stage_id} machine evidence\n\n"
        "This compact directory was generated by `scripts/autonomous_runner.py`.\n"
        "The manifest covers every file except the manifest itself.\n",
        encoding="utf-8",
        newline="\n",
    )
    atomic_write_json(evidence_dir / "artifact_manifest.json", build_manifest(evidence_dir))
    return evidence_dir


Executor = Callable[[list[str], Path], subprocess.CompletedProcess[str]]


def execute_stage(
    stage_id: str,
    root: Path = ROOT,
    registry_path: Path = REGISTRY_PATH,
    state_path: Path = STATE_PATH,
    executor: Executor | None = None,
) -> str:
    registry = load_registry(registry_path)
    state = load_json(state_path)
    if stage_id not in registry["stages"]:
        raise ValueError(f"unknown stage: {stage_id}")
    stage = state["stages"][stage_id]
    if stage["status"] == "PASS":
        evidence = stage.get("evidence_dir")
        if evidence and not verify_manifest(root / evidence):
            return "SKIPPED_EXISTING_PASS"
        raise RuntimeError(f"{stage_id} claims PASS but its evidence is missing or invalid")
    dependencies = registry["stages"][stage_id].get("dependencies", [])
    if not all(state["stages"][dependency]["status"] == "PASS" for dependency in dependencies):
        return "DEPENDENCY_BLOCKED"
    command = registry["stages"][stage_id].get("command")
    if command is None:
        return "NO_COMMAND"
    run = executor or (
        lambda argv, cwd: subprocess.run(argv, cwd=cwd, capture_output=True, text=True)
    )
    started_at = utc_now()
    stage["status"] = "RUNNING"
    stage["attempt_count"] += 1
    state["run"]["current_stage"] = stage_id
    atomic_write_json(state_path, state)
    completed = run(command, root)
    completed_at = utc_now()
    stage["status"] = "PASS" if completed.returncode == 0 else "FAIL"
    stage["machine_gate_pass"] = completed.returncode == 0
    stage["first_blocking_layer"] = None if completed.returncode == 0 else "stage_command"
    attempt = {
        "attempt_id": f"{stage_id}-A{stage['attempt_count']}",
        "hypothesis": "registered stage command satisfies its machine gate",
        "input_commit": git_output(["rev-parse", "HEAD"], root),
        "configuration_sha256": hashlib.sha256(
            json.dumps(registry["stages"][stage_id], sort_keys=True).encode()
        ).hexdigest(),
        "changed_variables": [],
        "fixed_variables": [],
        "commands": [command],
        "returncode": completed.returncode,
        "stdout": completed.stdout[-8000:],
        "stderr": completed.stderr[-8000:],
        "result": stage["status"],
        "first_failure": stage["first_blocking_layer"],
        "metrics": {},
        "raw_evidence": [],
        "decision": "select" if completed.returncode == 0 else "retry",
    }
    stage_status = {
        "schema_version": 1,
        "program": registry.get("program"),
        "stage_id": stage_id,
        "baseline_commit": registry.get("baseline_commit"),
        "implementation_commit": git_output(["rev-parse", "HEAD"], root),
        "started_at": started_at,
        "completed_at": completed_at,
        "status": stage["status"],
        "first_blocking_layer": stage["first_blocking_layer"],
        "attempt_count": stage["attempt_count"],
        "machine_gate_pass": stage["machine_gate_pass"],
        "human_review_required": False,
        "human_approval_required": False,
        "competition_evidence": False,
        "dependencies": {
            dependency: state["stages"][dependency]["status"] for dependency in dependencies
        },
        "metrics": {},
        "unexecuted_items": [],
        "next_scheduled_stages": [],
    }
    evidence_dir = write_standard_evidence(
        root,
        stage_id,
        registry["stages"][stage_id],
        stage_status,
        [attempt],
        [command],
        {"schema_version": 1, "metrics": {}},
    )
    stage["evidence_dir"] = evidence_dir.relative_to(root).as_posix()
    atomic_write_json(state_path, state)
    return stage["status"]


def cmd_validate(args: argparse.Namespace) -> int:
    registry = load_registry(args.registry)
    state = load_json(args.state)
    registry_errors = validate_registry(registry)
    state_errors = validate_state(state, registry)
    plan_matches = load_json(args.plan) == build_plan(registry)
    result = {
        "baseline_commit_resolved": git_output(["cat-file", "-e", f"{registry['baseline_commit']}^{{commit}}"]) == "",
        "stage_registry_valid": not registry_errors,
        "dependency_cycle_count": 0 if not registry_errors else None,
        "state_schema_valid": not state_errors,
        "run_plan_matches_registry": plan_matches,
        "registry_errors": registry_errors,
        "state_errors": state_errors,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if all(
        result[key]
        for key in ("baseline_commit_resolved", "stage_registry_valid", "state_schema_valid", "run_plan_matches_registry")
    ) else 1


def cmd_plan(args: argparse.Namespace) -> int:
    plan = build_plan(load_registry(args.registry))
    if args.write:
        atomic_write_json(args.plan, plan)
    print(json.dumps(plan, ensure_ascii=False, indent=2))
    return 0


def cmd_status(args: argparse.Namespace) -> int:
    state = load_json(args.state)
    registry = load_registry(args.registry)
    print(json.dumps({"run": state["run"], "ready_stages": ready_stages(state, registry)}, ensure_ascii=False, indent=2))
    return 0


def cmd_run_stage(args: argparse.Namespace) -> int:
    with exclusive_lock(args.lock):
        result = execute_stage(args.stage, ROOT, args.registry, args.state)
    print(json.dumps({"stage": args.stage, "result": result}, ensure_ascii=False))
    return 0 if result in {"PASS", "SKIPPED_EXISTING_PASS", "NO_COMMAND"} else 1


def cmd_baseline_audit(args: argparse.Namespace) -> int:
    report = historical_hash_audit(ROOT)
    if args.output:
        atomic_write_json(args.output, report)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 1 if report["historical_evidence_modified"] else 0


def cmd_finalize_auto00(args: argparse.Namespace) -> int:
    registry = load_registry(args.registry)
    state = load_json(args.state)
    current = state["stages"]["AUTO-00"]
    if current["status"] == "PASS":
        evidence = current.get("evidence_dir")
        errors = verify_manifest(ROOT / evidence) if evidence else ["missing evidence_dir"]
        print(json.dumps({"result": "SKIPPED_EXISTING_PASS", "errors": errors}, indent=2))
        return 1 if errors else 0

    gate_commands = [
        [sys.executable, "scripts/ci_fast.py"],
        [sys.executable, "scripts/verify_state_invariants.py"],
        [sys.executable, "scripts/scan_secrets.py"],
        ["git", "diff", "--check"],
    ]
    attempts: list[dict[str, Any]] = []
    gate_results: dict[str, bool] = {}
    for index, command in enumerate(gate_commands, start=1):
        completed = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        gate_name = Path(command[1]).stem if len(command) > 1 and command[0] == sys.executable else "git_diff_check"
        gate_results[gate_name] = completed.returncode == 0
        attempts.append(
            {
                "attempt_id": f"AUTO-00-GATE-{index}",
                "hypothesis": f"{gate_name} passes on the implementation worktree",
                "input_commit": git_output(["rev-parse", "HEAD"]),
                "configuration_sha256": hashlib.sha256(" ".join(command).encode()).hexdigest(),
                "changed_variables": [],
                "fixed_variables": [],
                "commands": [command],
                "returncode": completed.returncode,
                "stdout": completed.stdout[-12000:],
                "stderr": completed.stderr[-12000:],
                "result": "PASS" if completed.returncode == 0 else "FAIL",
                "first_failure": None if completed.returncode == 0 else gate_name,
                "metrics": {},
                "raw_evidence": [],
                "decision": "select" if completed.returncode == 0 else "block",
            }
        )

    audit = historical_hash_audit(ROOT)
    registry_errors = validate_registry(registry)
    state_errors = validate_state(state, registry)
    plan_matches = load_json(args.plan) == build_plan(registry)
    criteria = {
        "baseline_commit_resolved": subprocess.run(
            ["git", "cat-file", "-e", f"{registry['baseline_commit']}^{{commit}}"],
            cwd=ROOT,
        ).returncode
        == 0,
        "historical_evidence_modified": audit["historical_evidence_modified"],
        "stage_registry_valid": not registry_errors,
        "dependency_cycle_count": 0 if not registry_errors else None,
        "state_schema_valid": not state_errors,
        "run_plan_matches_registry": plan_matches,
        "resume_from_interruption_test": gate_results.get("ci_fast", False),
        "idempotent_rerun_test": gate_results.get("ci_fast", False),
        "ci_fast": gate_results.get("ci_fast", False),
        "state_invariants": gate_results.get("verify_state_invariants", False),
        "secret_scan": gate_results.get("scan_secrets", False),
        "git_diff_check": gate_results.get("git_diff_check", False),
    }
    passed = (
        criteria["baseline_commit_resolved"]
        and criteria["historical_evidence_modified"] is False
        and criteria["stage_registry_valid"]
        and criteria["dependency_cycle_count"] == 0
        and criteria["state_schema_valid"]
        and criteria["run_plan_matches_registry"]
        and all(gate_results.values())
    )
    started_at = state["run"].get("started_at") or utc_now()
    completed_at = utc_now()
    commit = git_output(["rev-parse", "HEAD"])
    stage_status = {
        "schema_version": 1,
        "program": registry.get("program"),
        "stage_id": "AUTO-00",
        "baseline_commit": registry.get("baseline_commit"),
        "implementation_commit": commit,
        "started_at": started_at,
        "completed_at": completed_at,
        "status": "PASS" if passed else "FAIL",
        "first_blocking_layer": None if passed else "auto00_machine_gate",
        "attempt_count": len(attempts),
        "machine_gate_pass": passed,
        "human_review_required": False,
        "human_approval_required": False,
        "competition_evidence": False,
        "dependencies": {},
        "metrics": criteria,
        "unexecuted_items": [],
        "next_scheduled_stages": (
            ["AUTO-01", "AUTO-04", "AUTO-09", "AUTO-10", "AUTO-11", "AUTO-12", "AUTO-13", "AUTO-14"]
            if passed
            else []
        ),
    }
    evidence_dir = write_standard_evidence(
        ROOT,
        "AUTO-00",
        registry["stages"]["AUTO-00"],
        stage_status,
        attempts,
        gate_commands,
        {"schema_version": 1, **criteria},
    )
    atomic_write_json(evidence_dir / "baseline_hash_report.json", audit)
    atomic_write_json(
        evidence_dir / "dependency_dag.json",
        {
            "schema_version": 1,
            "topological_levels": topological_levels(registry),
            "dependency_cycle_count": criteria["dependency_cycle_count"],
        },
    )
    atomic_write_json(evidence_dir / "artifact_manifest.json", build_manifest(evidence_dir))

    current.update(
        {
            "status": "PASS" if passed else "FAIL",
            "machine_gate_pass": passed,
            "first_blocking_layer": None if passed else "auto00_machine_gate",
            "attempt_count": len(attempts),
            "selected_attempt": "python_orchestrator",
            "implementation_commit": commit,
            "evidence_dir": evidence_dir.relative_to(ROOT).as_posix(),
            "metrics": criteria,
        }
    )
    state["historical_boundaries"]["historical_evidence_modified"] = audit[
        "historical_evidence_modified"
    ]
    state["run"]["current_stage"] = "AUTO-01" if passed else "AUTO-00"
    state["run"]["last_commit"] = commit
    atomic_write_json(args.state, state)
    print(
        json.dumps(
            {
                "result": current["status"],
                "evidence_dir": current["evidence_dir"],
                "metrics": criteria,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0 if passed else 1


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.set_defaults(registry=REGISTRY_PATH, state=STATE_PATH, plan=PLAN_PATH, lock=LOCK_PATH)
    parser.add_argument("--registry", type=Path, default=REGISTRY_PATH)
    parser.add_argument("--state", type=Path, default=STATE_PATH)
    parser.add_argument("--plan", type=Path, default=PLAN_PATH)
    parser.add_argument("--lock", type=Path, default=LOCK_PATH)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("validate").set_defaults(func=cmd_validate)
    plan = subparsers.add_parser("plan")
    plan.add_argument("--write", action="store_true")
    plan.set_defaults(func=cmd_plan)
    subparsers.add_parser("status").set_defaults(func=cmd_status)
    run_stage = subparsers.add_parser("run-stage")
    run_stage.add_argument("stage")
    run_stage.set_defaults(func=cmd_run_stage)
    baseline = subparsers.add_parser("baseline-audit")
    baseline.add_argument("--output", type=Path)
    baseline.set_defaults(func=cmd_baseline_audit)
    subparsers.add_parser("finalize-auto00").set_defaults(func=cmd_finalize_auto00)
    return parser


def main() -> int:
    args = make_parser().parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
