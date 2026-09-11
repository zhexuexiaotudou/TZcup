#!/usr/bin/env python3
"""Bind clean source, a live ADB probe, and a live simulation-host probe."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import subprocess
from datetime import datetime, timedelta, timezone
from pathlib import Path


COMPONENTS = (
    "2395736b4919a95e85296f43e75ce44c063e6ce9",
    "a877df06d78f5abe323e0b5df3f8412af60d900a",
    "d3f9de916180f90625562ac75c88c834fbb8a028",
    "aedfdd4b2eb5e2852acca5c9b4f61d5bfcca1619",
    "6fe52a7e0d4f2ffd7a6b356c8b2e76b9461b71ac",
)
REQUIREMENTS = (
    ("SIM-01", "user_q5", "competition_core", "mapping", "Golden Mission raw evidence"),
    ("SIM-02", "user_q5", "competition_core", "route_or_coverage_following", "Golden Mission raw evidence"),
    ("SIM-03", "user_q5", "competition_core", "garbage_detection_and_localization", "Perception outputs and independent scoring"),
    ("SIM-04", "user_q5", "competition_core", "grasp_or_cleaning", "Physical contact or cleaning-state evidence"),
    ("SIM-05", "user_q5", "competition_core", "dynamic_obstacle_avoidance", "Collision and minimum-distance evidence"),
    ("SIM-06", "user_q5", "competition_core", "emergency_braking", "Fresh controlled command-to-stop evidence"),
    ("SIM-07", "simulation_only_spec", "engineering", "three_fresh_complete_runs", "Fresh run receipts"),
    ("BOARD-01", "user_q5", "competition_core", "perception", "Board PID, hashes, I/O and metrics"),
    ("BOARD-02", "user_q5", "competition_core", "mapping_localization", "Board PID, hashes, I/O and metrics"),
    ("BOARD-03", "user_q5", "competition_core", "planning_decision", "Board PID, hashes, I/O and metrics"),
    ("BOARD-04", "user_q5", "competition_core", "control", "Board PID, hashes, I/O and metrics"),
    ("METRIC-01", "COMPETITION_REQUIREMENTS.md", "project_adopted_pending_official_check", "localization_xy_m<=0.05", "Source-bound aggregate"),
    ("METRIC-02", "COMPETITION_REQUIREMENTS.md", "project_adopted_pending_official_check", "mapped_effective_area_m2>=20000", "Map evidence"),
    ("METRIC-03", "COMPETITION_REQUIREMENTS.md", "project_adopted_pending_official_check", "net_cleaning_efficiency_m2_h>=3500", "Actual swept union / total sim time"),
    ("METRIC-04", "COMPETITION_REQUIREMENTS.md", "project_adopted_pending_official_check", "effective_cleaning_width_m>=0.6", "Geometry and sweep readback"),
    ("METRIC-05", "COMPETITION_REQUIREMENTS.md", "project_adopted_pending_official_check", "emergency_braking_s<=1.0", "Timestamped stop evidence"),
    ("PACK-01", "simulation_only_spec", "competition_core", "reproduction_package", "Manifest, commands, metrics, logs, limitations"),
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git(repo: Path, *args: str) -> str:
    return subprocess.check_output(["git", "-C", str(repo), *args], text=True, encoding="utf-8").strip()


def load_probe(path: Path, expected_id: str) -> dict:
    if path.is_symlink() or not path.is_file():
        raise SystemExit(f"probe must be a regular non-symlink file: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise SystemExit(f"probe is unreadable: {path}: {exc}") from exc
    if not isinstance(value, dict) or value.get("report_id") != expected_id:
        raise SystemExit(f"wrong probe identity: {path}")
    if value.get("live_probe") is not True:
        raise SystemExit(f"probe is not marked live: {path}")
    try:
        generated = datetime.fromisoformat(str(value["generated_utc"]))
    except (KeyError, TypeError, ValueError) as exc:
        raise SystemExit(f"probe timestamp is invalid: {path}") from exc
    now = datetime.now(timezone.utc)
    if generated.tzinfo is None or not now - timedelta(minutes=15) <= generated <= now + timedelta(minutes=1):
        raise SystemExit(f"probe is stale or from the future: {path}")
    return value


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--board-probe", type=Path, required=True)
    parser.add_argument("--remote-probe", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    repo = args.repo.resolve()
    output = args.output.resolve()
    if output.exists():
        raise SystemExit("output must not exist")
    if not repo.joinpath(".git").exists():
        raise SystemExit("repo is not a Git worktree")
    status = git(repo, "status", "--porcelain=v1")
    if status:
        raise SystemExit("source worktree must be clean before evidence freeze")

    board_path = args.board_probe.resolve()
    remote_path = args.remote_probe.resolve()
    board = load_probe(board_path, "tzcup_competition_sim_only_board_probe_v1")
    remote = load_probe(remote_path, "tzcup_competition_sim_only_remote_probe_v1")
    if (
        "S100P" not in str(board.get("model", ""))
        or board.get("architecture") != "aarch64"
        or "/dev/bpu_core0" not in board.get("bpu_nodes", [])
        or len(str(board.get("overlay_setup_sha256", ""))) != 64
    ):
        raise SystemExit("board probe does not prove the required S100P/BPU/overlay identity")
    head = git(repo, "rev-parse", "HEAD")
    tree = git(repo, "rev-parse", "HEAD^{tree}")
    remote_source = remote.get("source", {})
    if len(str(remote_source.get("head", ""))) != 40 or len(str(remote_source.get("tree", ""))) != 40:
        raise SystemExit("remote probe Git identity is malformed")
    if not str(remote.get("gpu", "")).strip():
        raise SystemExit("remote probe GPU inventory is missing")
    if remote_source.get("head") != head or remote_source.get("tree") != tree:
        raise SystemExit("remote probe source does not match the clean local source")
    if remote_source.get("status_porcelain") != "":
        raise SystemExit("remote probe source is dirty")

    component_rows = []
    for commit in COMPONENTS:
        result = subprocess.run(["git", "-C", str(repo), "merge-base", "--is-ancestor", commit, head])
        component_rows.append({"commit": commit, "is_ancestor_of_head": result.returncode == 0})
    if not all(row["is_ancestor_of_head"] for row in component_rows):
        raise SystemExit("one or more integration components are not ancestors of HEAD")

    output.mkdir(parents=True)
    generated = datetime.now(timezone.utc).isoformat()
    write_json(output / "source_manifest.json", {
        "schema_version": 2,
        "report_id": "tzcup_competition_sim_only_source_manifest_v2",
        "generated_utc": generated,
        "repo": repo.as_posix(),
        "branch": git(repo, "branch", "--show-current"),
        "head": head,
        "tree": tree,
        "parents": git(repo, "show", "-s", "--format=%P", "HEAD").split(),
        "status_porcelain": status,
        "integration_components": component_rows,
    })
    write_json(output / "runtime_inventory.json", {
        "schema_version": 2,
        "report_id": "tzcup_competition_sim_only_runtime_inventory_v2",
        "generated_utc": generated,
        "board_probe": {"path": board_path.as_posix(), "sha256": sha256(board_path), "data": board},
        "remote_probe": {"path": remote_path.as_posix(), "sha256": sha256(remote_path), "data": remote},
        "claim_boundary": "Live identity probes only; no algorithm or mission acceptance.",
    })
    with (output / "scope_matrix.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(("requirement_id", "source", "level", "requirement", "required_evidence", "status"))
        for row in REQUIREMENTS:
            writer.writerow((*row, "PENDING_FRESH_EVIDENCE"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
