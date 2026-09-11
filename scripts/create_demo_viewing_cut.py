#!/usr/bin/env python3
"""Build a source-bound 5--8 minute competition-demo viewing cut.

The program deliberately consumes only one T1/T2 delivery folder.  It refuses
to join clips, add titles, or generate a movie when the raw video, evidence
identity, checksums, or any required event is missing.  The EDL and subtitles
are derived exclusively from the verified timeline timestamps.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
import subprocess
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any


REQUIRED_EVENTS = (
    "map_creation", "hard_restart", "cleaning", "grasp_drop",
    "obstacle_avoidance", "return_home",
)
MIN_DURATION_SEC = 300.0
MAX_DURATION_SEC = 480.0
SEGMENT_DURATION_SEC = 50.0
SCHEMA = "tzcup.competition_demo_viewing_cut.v1"


class ContractError(ValueError):
    """A source or identity condition prevents a viewing cut."""


@dataclass(frozen=True)
class Segment:
    event: str
    source: str
    reason: str
    event_epoch_ns: int
    start_sec: float
    end_sec: float
    speed: float = 1.0


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_under(root: Path, raw: object, label: str) -> Path:
    if isinstance(raw, dict):
        raw = raw.get("path")
    if not isinstance(raw, str) or not raw.strip():
        raise ContractError(f"{label}: missing path")
    candidate = Path(raw)
    candidate = candidate if candidate.is_absolute() else root / candidate
    resolved = candidate.resolve()
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise ContractError(f"{label}: path is outside run_root") from exc
    if candidate.is_symlink() or not resolved.is_file():
        raise ContractError(f"{label}: requires a non-link regular file")
    return resolved


def artifact_descriptor(manifest: dict[str, Any], name: str) -> object:
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise ContractError("artifacts: missing object")
    if name not in artifacts:
        raise ContractError(f"artifacts.{name}: missing")
    return artifacts[name]


def verify_file_artifact(manifest: dict[str, Any], root: Path, name: str) -> Path:
    descriptor = artifact_descriptor(manifest, name)
    if not isinstance(descriptor, dict):
        raise ContractError(f"artifacts.{name}: requires an object descriptor")
    path = resolve_under(root, descriptor, f"artifacts.{name}")
    expected_hash, expected_bytes = descriptor.get("sha256"), descriptor.get("size_bytes")
    if expected_hash != sha256(path) or expected_bytes != path.stat().st_size:
        raise ContractError(f"artifacts.{name}: bytes differ from manifest")
    return path


def verify_mcap_artifact(manifest: dict[str, Any], root: Path) -> Path:
    descriptor = artifact_descriptor(manifest, "mcap")
    if not isinstance(descriptor, dict) or not isinstance(descriptor.get("path"), str):
        raise ContractError("artifacts.mcap: requires a directory descriptor")
    candidate = Path(descriptor["path"])
    candidate = candidate if candidate.is_absolute() else root / candidate
    path = candidate.resolve()
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ContractError("artifacts.mcap: path is outside run_root") from exc
    if candidate.is_symlink() or not path.is_dir():
        raise ContractError("artifacts.mcap: requires a non-link directory")
    files = descriptor.get("files")
    if not isinstance(files, list) or not files:
        raise ContractError("artifacts.mcap: file inventory is missing")
    inventory = []
    declared_paths: set[str] = set()
    for item in files:
        if not isinstance(item, dict) or not isinstance(item.get("relative_path"), str):
            raise ContractError("artifacts.mcap: invalid file inventory")
        relative_path = item["relative_path"].replace("\\", "/")
        if relative_path in declared_paths:
            raise ContractError("artifacts.mcap: duplicate inventory path")
        declared_paths.add(relative_path)
        file = (path / relative_path).resolve()
        try:
            file.relative_to(path)
        except ValueError as exc:
            raise ContractError("artifacts.mcap: inventory escapes directory") from exc
        if (file.is_symlink() or not file.is_file()
                or item.get("sha256") != sha256(file)
                or item.get("size_bytes") != file.stat().st_size):
            raise ContractError("artifacts.mcap: file inventory differs from manifest")
        inventory.append(f"{relative_path} {item['sha256']}")
    actual_paths = {
        str(file.relative_to(path)).replace("\\", "/")
        for file in path.rglob("*") if file.is_file() and not file.is_symlink()
    }
    if actual_paths != declared_paths:
        raise ContractError("artifacts.mcap: on-disk inventory differs from manifest")
    if descriptor.get("tree_sha256") != hashlib.sha256("\n".join(inventory).encode("utf-8")).hexdigest():
        raise ContractError("artifacts.mcap: tree hash differs from manifest")
    return path


def verify_event_reports(manifest: dict[str, Any], root: Path) -> list[Path]:
    descriptors = artifact_descriptor(manifest, "event_reports")
    if not isinstance(descriptors, list) or not descriptors:
        raise ContractError("artifacts.event_reports: requires a non-empty array")
    paths: list[Path] = []
    for index, descriptor in enumerate(descriptors):
        if not isinstance(descriptor, dict):
            raise ContractError(f"artifacts.event_reports[{index}]: invalid descriptor")
        path = resolve_under(root, descriptor, f"artifacts.event_reports[{index}]")
        if descriptor.get("sha256") != sha256(path) or descriptor.get("size_bytes") != path.stat().st_size:
            raise ContractError(f"artifacts.event_reports[{index}]: bytes differ from manifest")
        paths.append(path)
    return paths


def parse_checksum_file(path: Path) -> dict[str, str]:
    expected: dict[str, str] = {}
    for row in path.read_text(encoding="utf-8").splitlines():
        row = row.strip()
        if not row:
            continue
        parts = row.split(maxsplit=1)
        if len(parts) != 2 or len(parts[0]) != 64:
            raise ContractError("checksums: malformed SHA-256 row")
        expected[parts[1].lstrip("*").replace("\\", "/")] = parts[0].lower()
    return expected


def verify_checksums(manifest_path: Path, timeline_path: Path, checksum_path: Path) -> None:
    expected = parse_checksum_file(checksum_path)
    for path in (manifest_path, timeline_path):
        actual = sha256(path)
        candidates = {path.name, path.as_posix()}
        recorded = next((expected[item] for item in candidates if item in expected), None)
        if recorded != actual:
            raise ContractError(f"checksums: {path.name} is missing or does not match")


def require_identity(manifest: dict[str, Any]) -> dict[str, Any]:
    identity = manifest.get("run_identity")
    required = ("session_id", "runtime_id", "episode_id", "session_start_epoch_ns")
    if not isinstance(identity, dict) or any(not identity.get(key) for key in required):
        raise ContractError("run_identity: missing required identity fields")
    try:
        int(identity["session_start_epoch_ns"])
    except (TypeError, ValueError) as exc:
        raise ContractError("run_identity.session_start_epoch_ns must be an integer") from exc
    return identity


def recording_window(manifest: dict[str, Any]) -> tuple[int, int]:
    window = manifest.get("recording_window")
    if not isinstance(window, dict):
        raise ContractError("recording_window: missing object")
    starts = ("start_epoch_ns", "video_start_epoch_ns", "recording_start_epoch_ns", "operator_start_epoch_ns")
    ends = ("end_epoch_ns", "video_end_epoch_ns", "recording_end_epoch_ns", "mission_complete_epoch_ns")
    start = next((window.get(key) for key in starts if window.get(key) is not None), None)
    end = next((window.get(key) for key in ends if window.get(key) is not None), None)
    try:
        start_ns, end_ns = int(start), int(end)
    except (TypeError, ValueError) as exc:
        raise ContractError("recording_window: requires start and end epoch nanoseconds") from exc
    if end_ns <= start_ns:
        raise ContractError("recording_window: end must follow start")
    return start_ns, end_ns


def require_delivery_ready(manifest: dict[str, Any]) -> None:
    if manifest.get("delivery_status") != "READY_FOR_REVIEW":
        raise ContractError("manifest delivery_status is not READY_FOR_REVIEW")
    if manifest.get("timeline_status") != "complete" or manifest.get("missing_required_events"):
        raise ContractError("manifest reports an incomplete required-event timeline")


def read_timeline(path: Path) -> dict[str, dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        expected = ["event", "status", "epoch_ns", "source", "reason"]
        if reader.fieldnames != expected:
            raise ContractError("timeline.csv: columns must be event,status,epoch_ns,source,reason")
        rows = list(reader)
    by_event: dict[str, dict[str, str]] = {}
    for row in rows:
        event = row.get("event", "")
        if event in by_event:
            raise ContractError(f"timeline.csv: duplicate event {event}")
        by_event[event] = row
    return by_event


def video_duration(video: Path, ffprobe: str) -> float:
    completed = subprocess.run(
        [ffprobe, "-v", "error", "-show_entries", "format=duration", "-of", "json", str(video)],
        capture_output=True, text=True, check=False,
    )
    if completed.returncode:
        raise ContractError(f"ffprobe failed for video: {completed.stderr.strip() or completed.returncode}")
    try:
        duration = float(json.loads(completed.stdout)["format"]["duration"])
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ContractError("ffprobe did not return a usable MP4 duration") from exc
    if duration <= 0:
        raise ContractError("ffprobe returned a non-positive MP4 duration")
    return duration


def make_segments(rows: dict[str, dict[str, str]], start_ns: int, duration: float) -> list[Segment]:
    segments: list[Segment] = []
    missing: list[str] = []
    for event in REQUIRED_EVENTS:
        row = rows.get(event)
        if not row or row.get("status") != "observed":
            missing.append(event)
            continue
        if not row.get("source") or not row.get("reason"):
            missing.append(f"{event}:missing_source_or_reason")
            continue
        try:
            event_ns = int(row["epoch_ns"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ContractError(f"timeline.csv: {event} has invalid epoch_ns") from exc
        center = (event_ns - start_ns) / 1_000_000_000
        cut_start = max(0.0, center - SEGMENT_DURATION_SEC / 2)
        cut_end = min(duration, cut_start + SEGMENT_DURATION_SEC)
        cut_start = max(0.0, cut_end - SEGMENT_DURATION_SEC)
        if center < 0 or center > duration or cut_end - cut_start < SEGMENT_DURATION_SEC:
            missing.append(f"{event}:outside_or_insufficient_video_window")
            continue
        segments.append(Segment(event, row["source"], row["reason"], event_ns, round(cut_start, 3), round(cut_end, 3)))
    if missing:
        raise ContractError("required events missing: " + ", ".join(missing))
    segments.sort(key=lambda item: item.start_sec)
    for previous, current in zip(segments, segments[1:], strict=False):
        if current.start_sec < previous.end_sec:
            raise ContractError("event windows overlap; cannot create a truthful 5-minute cut")
    total = sum(item.end_sec - item.start_sec for item in segments)
    if not MIN_DURATION_SEC <= total <= MAX_DURATION_SEC:
        raise ContractError(f"EDL duration {total:.3f}s is outside {MIN_DURATION_SEC:.0f}-{MAX_DURATION_SEC:.0f}s")
    return segments


def write_edl(segments: list[Segment], manifest: dict[str, Any], output: Path) -> tuple[Path, Path]:
    output.mkdir(parents=True, exist_ok=True)
    edl_path = output / "viewing_cut.edl.json"
    srt_path = output / "viewing_cut.srt"
    edl_path.write_text(json.dumps({
        "schema_version": SCHEMA,
        "kind": "source_bound_viewing_cut_edl",
        "run_identity": manifest["run_identity"],
        "source_video": manifest["artifacts"]["video"],
        "duration_sec": round(sum(item.end_sec - item.start_sec for item in segments), 3),
        "segments": [asdict(item) for item in segments],
        "claim_boundary": "Cuts, timing, subtitles, and speed are derived only from one verified episode timeline.",
    }, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    cursor = 0.0
    rows: list[str] = []
    for index, segment in enumerate(segments, 1):
        end = cursor + segment.end_sec - segment.start_sec
        def stamp(value: float) -> str:
            millis = round(value * 1000)
            hours, millis = divmod(millis, 3_600_000)
            minutes, millis = divmod(millis, 60_000)
            seconds, millis = divmod(millis, 1000)
            return f"{hours:02}:{minutes:02}:{seconds:02},{millis:03}"
        rows.extend([str(index), f"{stamp(cursor)} --> {stamp(end)}", f"{segment.event} | {segment.source}", segment.reason, ""])
        cursor = end
    srt_path.write_text("\n".join(rows), encoding="utf-8")
    return edl_path, srt_path


def render(video: Path, segments: list[Segment], srt: Path, output: Path, ffmpeg: str) -> Path:
    clip_dir = output / "clips"
    clip_dir.mkdir(exist_ok=True)
    clip_paths: list[Path] = []
    for index, segment in enumerate(segments, 1):
        clip = clip_dir / f"{index:02d}_{segment.event}.mp4"
        command = [ffmpeg, "-y", "-ss", str(segment.start_sec), "-to", str(segment.end_sec), "-i", str(video), "-map", "0:v:0", "-map", "0:a?", "-c:v", "libx264", "-c:a", "aac", str(clip)]
        if subprocess.run(command, capture_output=True, text=True).returncode:
            raise ContractError(f"ffmpeg failed while rendering {segment.event}")
        clip_paths.append(clip)
    concat = output / "viewing_cut.concat.txt"
    concat.write_text("".join(f"file '{path.as_posix()}'\n" for path in clip_paths), encoding="utf-8")
    target = output / "competition_demo_viewing_cut.mp4"
    command = [ffmpeg, "-y", "-f", "concat", "-safe", "0", "-i", str(concat), "-i", str(srt), "-map", "0:v:0", "-map", "0:a?", "-map", "1:0", "-c:v", "copy", "-c:a", "copy", "-c:s", "mov_text", str(target)]
    if subprocess.run(command, capture_output=True, text=True).returncode:
        raise ContractError("ffmpeg failed while muxing the viewing cut")
    return target


def build(manifest_path: Path, timeline_path: Path, checksum_path: Path, output: Path, ffprobe: str, ffmpeg: str | None, execute: bool) -> dict[str, Any]:
    receipt: dict[str, Any] = {"schema_version": SCHEMA, "status": "BLOCKED_MISSING_REQUIRED_EVIDENCE", "generated_from": {"manifest": str(manifest_path), "timeline": str(timeline_path), "checksums": str(checksum_path)}, "missing": [], "outputs": {}}
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest.get("schema_version") != "tzcup.competition_demo_artifact_manifest.v1":
            raise ContractError("manifest: unsupported schema_version")
        root = Path(manifest.get("run_root", "")).resolve()
        if not root.is_dir():
            raise ContractError("run_root: missing directory")
        verify_checksums(manifest_path, timeline_path, checksum_path)
        identity = require_identity(manifest)
        require_delivery_ready(manifest)
        start_ns, end_ns = recording_window(manifest)
        for name in ("raw_collection", "video_observation", "formal_session_current", "video"):
            verify_file_artifact(manifest, root, name)
        verify_mcap_artifact(manifest, root)
        verify_event_reports(manifest, root)
        video = verify_file_artifact(manifest, root, "video")
        duration = video_duration(video, ffprobe)
        window_duration = (end_ns - start_ns) / 1_000_000_000
        if abs(duration - window_duration) > 2.0:
            raise ContractError("MP4 duration does not match the signed recording window")
        segments = make_segments(read_timeline(timeline_path), start_ns, duration)
        edl, srt = write_edl(segments, manifest, output)
        receipt.update({"status": "READY_FOR_REVIEW", "run_identity": identity, "source_video": {"path": str(video), "sha256": sha256(video), "duration_sec": duration}, "outputs": {"edl": str(edl), "subtitles": str(srt)}, "missing": []})
        if execute:
            if not ffmpeg:
                raise ContractError("ffmpeg is required with --execute")
            movie = render(video, segments, srt, output, ffmpeg)
            receipt["outputs"]["video"] = str(movie)
            receipt["outputs"]["video_sha256"] = sha256(movie)
    except (OSError, json.JSONDecodeError, ContractError) as exc:
        receipt["missing"] = [str(exc)]
    output.mkdir(parents=True, exist_ok=True)
    (output / "viewing_cut_receipt.json").write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--timeline", type=Path, required=True)
    parser.add_argument("--checksums", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--ffprobe", default=shutil.which("ffprobe") or "ffprobe")
    parser.add_argument("--ffmpeg", default=shutil.which("ffmpeg"))
    parser.add_argument("--execute", action="store_true", help="render the MP4 after all gates pass")
    args = parser.parse_args()
    receipt = build(args.manifest.resolve(), args.timeline.resolve(), args.checksums.resolve(), args.output_dir.resolve(), args.ffprobe, args.ffmpeg, args.execute)
    print(json.dumps(receipt, ensure_ascii=False, indent=2))
    return 0 if receipt["status"] == "READY_FOR_REVIEW" else 2


if __name__ == "__main__":
    raise SystemExit(main())
