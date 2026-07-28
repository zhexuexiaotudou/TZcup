from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import yaml

import autonomous_runner as runner


def _stage(dependencies: list[str], command: list[str] | None) -> dict:
    return {
        "title": "test stage",
        "lane": "test",
        "dependencies": dependencies,
        "optional_dependencies": [],
        "command": command,
    }


def _state(stage_ids: list[str], dependencies: dict[str, list[str]]) -> dict:
    return {
        "schema_version": 1,
        "historical_boundaries": {
            "historical_evidence_modified": False,
            "stage5br6a_human_review_completed": False,
            "stage5br6a_manual_audit_pass": False,
        },
        "run": {"current_stage": "AUTO-00"},
        "stages": {
            stage_id: {
                "status": "PENDING",
                "machine_gate_pass": False,
                "attempt_count": 0,
                "first_blocking_layer": None,
                "evidence_dir": None,
                "dependencies": dependencies[stage_id],
            }
            for stage_id in stage_ids
        },
    }


def _init_repo(path: Path) -> None:
    subprocess.run(["git", "init", "-q"], cwd=path, check=True)
    subprocess.run(["git", "config", "user.email", "ci@example.invalid"], cwd=path, check=True)
    subprocess.run(["git", "config", "user.name", "CI"], cwd=path, check=True)
    (path / "README.md").write_text("test\n", encoding="utf-8")
    subprocess.run(["git", "add", "README.md"], cwd=path, check=True)
    subprocess.run(["git", "commit", "-qm", "test baseline"], cwd=path, check=True)


def test_registry_has_complete_acyclic_program() -> None:
    registry = runner.load_registry()
    assert runner.validate_registry(registry) == []
    levels = runner.topological_levels(registry)
    assert levels[0] == ["AUTO-00"]
    assert {stage for level in levels for stage in level} == {
        f"AUTO-{index:02d}" for index in range(17)
    }


def test_resume_and_idempotent_rerun(tmp_path: Path) -> None:
    _init_repo(tmp_path)
    marker = tmp_path / "marker.txt"
    command = [
        sys.executable,
        "-c",
        (
            "from pathlib import Path; "
            f"p=Path({str(marker)!r}); "
            "p.write_text((p.read_text() if p.exists() else '') + 'run\\n')"
        ),
    ]
    registry = {
        "schema_version": 1,
        "program": "test",
        "baseline_commit": subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=tmp_path, check=True, capture_output=True, text=True
        ).stdout.strip(),
        "stages": {
            "AUTO-00": _stage([], command),
            "AUTO-01": _stage(["AUTO-00"], [sys.executable, "-c", "raise SystemExit(0)"]),
        },
    }
    state = _state(["AUTO-00", "AUTO-01"], {"AUTO-00": [], "AUTO-01": ["AUTO-00"]})
    registry_path = tmp_path / "registry.yaml"
    state_path = tmp_path / "state.json"
    registry_path.write_text(yaml.safe_dump(registry), encoding="utf-8")
    runner.atomic_write_json(state_path, state)

    assert runner.execute_stage("AUTO-01", tmp_path, registry_path, state_path) == "DEPENDENCY_BLOCKED"
    assert runner.execute_stage("AUTO-00", tmp_path, registry_path, state_path) == "PASS"
    resumed = runner.load_json(state_path)
    assert runner.ready_stages(resumed, registry) == ["AUTO-01"]
    assert marker.read_text(encoding="utf-8") == "run\n"

    assert runner.execute_stage("AUTO-00", tmp_path, registry_path, state_path) == "SKIPPED_EXISTING_PASS"
    assert marker.read_text(encoding="utf-8") == "run\n"
    evidence_dir = tmp_path / runner.load_json(state_path)["stages"]["AUTO-00"]["evidence_dir"]
    assert runner.verify_manifest(evidence_dir) == []


def test_state_invariants_reject_forged_human_flags() -> None:
    registry = runner.load_registry()
    state = runner.load_json(runner.STATE_PATH)
    state["historical_boundaries"]["stage5br6a_human_review_completed"] = True
    errors = runner.validate_state(state, registry)
    assert "historical human-review flag must remain false" in errors
