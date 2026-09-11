from __future__ import annotations

import importlib.util
import os
import subprocess
import sys
from pathlib import Path

import pytest


SCRIPT = Path(__file__).with_name("collect_formal_single_episode_cleaning_mission.py")
SPEC = importlib.util.spec_from_file_location("single_episode_collector_trusted_args", SCRIPT)
assert SPEC and SPEC.loader
collector = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = collector
SPEC.loader.exec_module(collector)


def test_trusted_recorder_accepts_only_the_fixed_node_and_live_pid_pgid(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(collector.os, "getpgid", lambda pid: 444, raising=False)
    collector.validate_trusted_gt_recorder_args("/a12_trusted_gt_recorder", 333, 444)
    with pytest.raises(SystemExit, match="identity is invalid"):
        collector.validate_trusted_gt_recorder_args("/other_recorder", 333, 444)
    with pytest.raises(SystemExit, match="required together"):
        collector.validate_trusted_gt_recorder_args("/a12_trusted_gt_recorder", None, None)
    with pytest.raises(SystemExit, match="does not belong"):
        collector.validate_trusted_gt_recorder_args("/a12_trusted_gt_recorder", 333, 445)


def test_collector_cli_rejects_bad_trusted_identity_before_loading_inputs(tmp_path: Path) -> None:
    """Exercise argparse/main rather than only a helper or static source text."""
    command = [
        sys.executable, str(SCRIPT),
        "--session-id", "session", "--episode-id", "episode", "--episode-seed", "1",
        "--runtime-id", "runtime", "--gazebo-process-id", "2", "--session-start-epoch-ns", "3",
        "--input-binding", str(tmp_path / "missing-binding.json"),
        "--ready-file", str(tmp_path / "ready.json"), "--output", str(tmp_path / "raw.json"),
        "--replay-metric-stream-dir", str(tmp_path / "replay-metric-streams"),
        "--episode-manifest", str(tmp_path / "episode.json"),
        "--evaluator-episode-manifest", str(tmp_path / "evaluator-episode.json"),
        "--evaluator-ground-truth", str(tmp_path / "truth.json"),
        "--world", str(tmp_path / "world.sdf"), "--pedestrian-schedule", str(tmp_path / "schedule.json"),
        "--session-status", str(tmp_path / "session.json"), "--same-map-baseline", str(tmp_path / "baseline.json"),
        "--policy-checkpoint", str(tmp_path / "checkpoint"), "--runtime-binding", str(tmp_path / "runtime.json"),
        "--saved-map", str(tmp_path / "map"), "--perception-artifacts", str(tmp_path / "perception"),
        "--trusted-gt-recorder-node", "/not-authorized", "--trusted-gt-recorder-pid", "3",
        "--trusted-gt-recorder-pgid", "3",
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    assert completed.returncode != 0
    assert "trusted GT recorder identity is invalid" in completed.stderr
    assert not (tmp_path / "ready.json").exists()
