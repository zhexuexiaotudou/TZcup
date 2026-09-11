#!/usr/bin/env python3
"""Seal one competition-demo delivery inventory without backfilling evidence."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any


EVENTS = (
    "map_creation", "hard_restart", "cleaning", "grasp_drop",
    "obstacle_avoidance", "return_home",
)


class ManifestError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def regular_file(path: Path, run_root: Path) -> Path:
    resolved = path.resolve(strict=True)
    try:
        resolved.relative_to(run_root)
    except ValueError as exc:
        raise ManifestError(f"source is outside run root: {path}") from exc
    if path.is_symlink() or not resolved.is_file():
        raise ManifestError(f"source must be a regular, non-symlink file: {path}")
    if resolved.stat().st_size <= 0:
        raise ManifestError(f"source is empty: {path}")
    return resolved


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ManifestError(f"{label} is not readable JSON: {path}") from exc
    if not isinstance(payload, dict):
        raise ManifestError(f"{label} must be a JSON object: {path}")
    return payload


def identity_from(raw: dict[str, Any], label: str) -> dict[str, Any]:
    identity = raw.get("run_identity")
    if not isinstance(identity, dict):
        raise ManifestError(f"{label} has no run_identity")
    required = ("session_id", "runtime_id", "episode_id", "session_start_epoch_ns")
    if any(identity.get(key) in (None, "") for key in required):
        raise ManifestError(f"{label} has incomplete run_identity")
    if isinstance(identity["session_start_epoch_ns"], bool) or not isinstance(identity["session_start_epoch_ns"], int) or identity["session_start_epoch_ns"] <= 0:
        raise ManifestError(f"{label} has invalid session_start_epoch_ns")
    return {key: identity[key] for key in required}


def require_identity(actual: dict[str, Any], expected: dict[str, Any], label: str) -> None:
    if actual != expected:
        raise ManifestError(f"{label} is not bound to the raw collection run_identity")


def fingerprint(path: Path, kind: str) -> dict[str, Any]:
    # Opening and hashing the complete byte stream is the portable readability
    # check.  Semantic MCAP replay remains a separate ROS-runtime gate.
    return {
        "kind": kind,
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
        "readable": True,
    }


def fingerprint_mcap(directory: Path, run_root: Path) -> dict[str, Any]:
    resolved = directory.resolve(strict=True)
    try:
        resolved.relative_to(run_root)
    except ValueError as exc:
        raise ManifestError(f"MCAP is outside run root: {directory}") from exc
    if directory.is_symlink() or not resolved.is_dir():
        raise ManifestError("MCAP source must be a non-symlink directory")
    metadata = resolved / "metadata.yaml"
    if not metadata.is_file() or metadata.is_symlink():
        raise ManifestError("MCAP directory has no regular metadata.yaml")
    files = sorted(path for path in resolved.rglob("*") if path.is_file())
    if not files:
        raise ManifestError("MCAP directory is empty")
    entries: list[dict[str, Any]] = []
    for file in files:
        if file.is_symlink():
            raise ManifestError(f"MCAP contains symlink: {file}")
        entries.append(fingerprint(file, "mcap_storage_file") | {
            "relative_path": str(file.relative_to(resolved)).replace("\\", "/")
        })
    has_payload = any(item["relative_path"].endswith(".mcap") for item in entries)
    if not has_payload:
        raise ManifestError("MCAP directory has no .mcap payload")
    tree = hashlib.sha256("\n".join(
        f"{item['relative_path']} {item['sha256']}" for item in entries
    ).encode("utf-8")).hexdigest()
    return {
        "kind": "rosbag2_mcap_directory",
        "path": str(resolved),
        "readable": True,
        "tree_sha256": tree,
        "files": entries,
        "semantic_replay": "not_performed_by_inventory_tool",
    }


def ffprobe(path: Path) -> dict[str, Any]:
    command = ["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "json", str(path)]
    try:
        result = subprocess.run(command, text=True, capture_output=True, check=True, timeout=30)
        parsed = json.loads(result.stdout)
        duration = float(parsed["format"]["duration"])
    except (OSError, subprocess.SubprocessError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ManifestError(f"video is not readable by ffprobe: {path}") from exc
    if duration <= 0:
        raise ManifestError(f"video has non-positive duration: {path}")
    return {"ffprobe_readable": True, "duration_seconds": duration}


def event_row(name: str, evidence: dict[str, Any] | None, reason: str) -> dict[str, Any]:
    if evidence is None:
        return {"event": name, "status": "missing", "epoch_ns": "", "source": "", "reason": reason}
    return {"event": name, "status": "observed", "epoch_ns": evidence["epoch_ns"], "source": evidence["source"], "reason": ""}


def explicit_events(paths: list[Path], identity: dict[str, Any]) -> dict[str, dict[str, Any]]:
    found: dict[str, dict[str, Any]] = {}
    for path in paths:
        report = read_json(path, "event report")
        require_identity(identity_from(report, "event report"), identity, "event report")
        name = report.get("event")
        stamp = report.get("event_epoch_ns")
        if name not in EVENTS or isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
            raise ManifestError(f"event report must name a required event and positive event_epoch_ns: {path}")
        if report.get("status") != "OBSERVED":
            raise ManifestError(f"event report must have OBSERVED status: {path}")
        if name in found:
            raise ManifestError(f"duplicate explicit event report: {name}")
        found[name] = {"epoch_ns": stamp, "source": str(path)}
    return found


def build_manifest(run_root: Path, raw_path: Path, video_manifest_path: Path,
                   mcap_path: Path, event_paths: list[Path]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    root = run_root.resolve(strict=True)
    if run_root.is_symlink() or not root.is_dir():
        raise ManifestError("run root must be a non-symlink directory")
    raw_path = regular_file(raw_path, root)
    video_manifest_path = regular_file(video_manifest_path, root)
    raw = read_json(raw_path, "raw collection")
    if raw.get("artifact_kind") != "single_live_episode_raw_collection":
        raise ManifestError("raw collection has unexpected artifact_kind")
    identity = identity_from(raw, "raw collection")
    video_manifest = read_json(video_manifest_path, "video manifest")
    if video_manifest.get("schema_version") != "tzcup.a12.video_observation.v1" or video_manifest.get("status") != "A12_VIDEO_OBSERVED":
        raise ManifestError("video manifest is not an observed A12 video")
    if Path(str(video_manifest.get("run_root", ""))).resolve() != root:
        raise ManifestError("video manifest has a different run_root")
    runtime_id = video_manifest.get("runtime_id")
    if runtime_id != identity["runtime_id"]:
        raise ManifestError("video manifest runtime_id differs from raw collection")
    session = video_manifest.get("formal_session")
    if not isinstance(session, dict):
        raise ManifestError("video manifest has no formal session binding")
    session_path = Path(str(session.get("path", "")))
    if session_path.is_symlink() or not session_path.is_file():
        raise ManifestError("video manifest formal session is not a regular file")
    session_payload = read_json(session_path, "formal session")
    if not isinstance(session.get("sha256_at_capture_start"), str) or len(session["sha256_at_capture_start"]) != 64:
        raise ManifestError("video manifest has no formal-session capture hash")
    if session_payload.get("report_id") != "tzcup_formal_final_acceptance_session_v1":
        raise ManifestError("video manifest formal session has unexpected report identity")
    if session_payload.get("started_epoch_ns") != identity["session_start_epoch_ns"]:
        raise ManifestError("video manifest formal session differs from raw collection")
    video = video_manifest.get("video")
    if not isinstance(video, dict):
        raise ManifestError("video manifest has no video descriptor")
    video_path = regular_file(Path(str(video.get("path", ""))), root)
    video_fingerprint = fingerprint(video_path, "mp4_video") | ffprobe(video_path)
    if video.get("sha256") != video_fingerprint["sha256"] or video.get("size_bytes") != video_fingerprint["size_bytes"]:
        raise ManifestError("video bytes differ from its observation manifest")
    window = video_manifest.get("window")
    if not isinstance(window, dict):
        raise ManifestError("video manifest has no recording window")
    start = window.get("operator_start_epoch_ns")
    end = window.get("mission_complete_epoch_ns")
    if any(isinstance(value, bool) or not isinstance(value, int) or value <= 0 for value in (start, end)) or start >= end:
        raise ManifestError("video manifest recording window is invalid")
    replay_capture = raw.get("replay_metric_capture")
    if not isinstance(replay_capture, dict):
        raise ManifestError("raw collection has no replay-metric capture")
    raw_window = replay_capture.get("capture_window", {})
    raw_start = raw_window.get("operator_start_ros_time_ns")
    raw_end = raw_window.get("task_complete_ros_time_ns")
    if any(isinstance(value, bool) or not isinstance(value, int) or value <= 0 for value in (raw_start, raw_end)) or raw_start >= raw_end:
        raise ManifestError("raw collection has no valid recording event window")
    if raw.get("timed_out") is not False:
        raise ManifestError("raw collection is not a completed mission")
    if replay_capture.get("complete") is not True:
        raise ManifestError("raw collection has no complete replay-metric capture")
    mcap = fingerprint_mcap(mcap_path, root)
    bound_event_paths = [regular_file(path, root) for path in event_paths]
    explicit = explicit_events(bound_event_paths, identity)
    product = raw.get("product") if isinstance(raw.get("product"), dict) else {}
    planner_samples = product.get("planner_status_samples") if isinstance(product.get("planner_status_samples"), list) else []
    returning = next((row for row in planner_samples if isinstance(row, dict) and row.get("returning_home") == "true" and isinstance(row.get("collector_received_epoch_ns"), int)), None)
    grasp = next((row for row in product.get("grasp_results", []) if isinstance(row, dict) and row.get("verified_in_bin") is True and isinstance(row.get("collector_received_epoch_ns"), int)), None)
    events = {
        "cleaning": {"epoch_ns": start, "source": str(video_manifest_path)},
        "grasp_drop": ({"epoch_ns": grasp["collector_received_epoch_ns"], "source": str(raw_path)} if grasp else None),
        "return_home": ({"epoch_ns": returning["collector_received_epoch_ns"], "source": str(raw_path)} if returning else None),
    }
    events.update(explicit)
    timeline = [event_row(name, events.get(name), "no same-run explicit event observation") for name in EVENTS]
    missing = [row["event"] for row in timeline if row["status"] == "missing"]
    manifest = {
        "schema_version": "tzcup.competition_demo_artifact_manifest.v1",
        "run_root": str(root), "run_identity": identity,
        "recording_window": {"operator_start_epoch_ns": start, "mission_complete_epoch_ns": end,
                             "raw_operator_start_ros_time_ns": raw_start, "raw_task_complete_ros_time_ns": raw_end},
        "artifacts": {"raw_collection": fingerprint(raw_path, "raw_collection_json"),
                      "video_observation": fingerprint(video_manifest_path, "video_observation_manifest_json"),
                      "formal_session_current": fingerprint(session_path.resolve(), "formal_session_json") | {"sha256_at_video_capture_start": session["sha256_at_capture_start"]},
                      "event_reports": [fingerprint(path, "same_run_event_report_json") for path in bound_event_paths],
                      "video": video_fingerprint, "mcap": mcap},
        "timeline_status": "complete" if not missing else "incomplete",
        "missing_required_events": missing,
        "delivery_status": "READY_FOR_REVIEW" if not missing else "BLOCKED_MISSING_REQUIRED_EVIDENCE",
        "limitations": ["Inventory hashes and reads files but does not substitute historical evidence.",
                        "MCAP semantic replay is not performed by this offline inventory tool."],
    }
    return manifest, timeline


def publish(output_dir: Path, manifest: dict[str, Any], timeline: list[dict[str, Any]]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    targets = {"manifest.json": json.dumps(manifest, indent=2, sort_keys=True) + "\n"}
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", newline="", delete=False, dir=output_dir) as handle:
        writer = csv.DictWriter(handle, fieldnames=("event", "status", "epoch_ns", "source", "reason"))
        writer.writeheader(); writer.writerows(timeline); timeline_temp = Path(handle.name)
    try:
        targets["timeline.csv"] = timeline_temp.read_text(encoding="utf-8")
    finally:
        timeline_temp.unlink(missing_ok=True)
    hashes = "".join(f"{hashlib.sha256(text.encode('utf-8')).hexdigest()}  {name}\n" for name, text in sorted(targets.items()))
    targets["checksums.sha256"] = hashes
    for name in targets:
        if (output_dir / name).exists():
            raise ManifestError(f"refusing to overwrite existing delivery output: {output_dir / name}")
    for name, text in targets.items():
        temporary = output_dir / f".{name}.{os.getpid()}.tmp"
        temporary.write_text(text, encoding="utf-8", newline="")
        try:
            os.link(temporary, output_dir / name)
        except FileExistsError as exc:
            raise ManifestError(f"refusing to overwrite existing delivery output: {output_dir / name}") from exc
        finally:
            temporary.unlink(missing_ok=True)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--raw-collection", type=Path, required=True)
    parser.add_argument("--video-manifest", type=Path, required=True)
    parser.add_argument("--mcap", type=Path, required=True)
    parser.add_argument("--event-report", type=Path, action="append", default=[])
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        manifest, timeline = build_manifest(args.run_root, args.raw_collection, args.video_manifest, args.mcap, args.event_report)
        output = args.output_dir.resolve()
        try:
            output.relative_to(args.run_root.resolve())
        except ValueError as exc:
            raise ManifestError("output directory must be inside run root") from exc
        publish(output, manifest, timeline)
    except ManifestError as exc:
        print(f"demo artifact manifest: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"delivery_status": manifest["delivery_status"], "output_dir": str(output)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
