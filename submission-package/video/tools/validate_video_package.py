#!/usr/bin/env python3
"""Validate the packaged modular competition video without modifying media."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import re
import shutil
import subprocess
from fractions import Fraction
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
INDEX_PATH = ROOT / "video-index.json"
DEFAULT_REPORT = ROOT / "validation" / "video-package-validation.json"
SRT_TIME = re.compile(
    r"(?P<h>\d{2}):(?P<m>\d{2}):(?P<s>\d{2}),(?P<ms>\d{3})\s+-->\s+"
    r"(?P<h2>\d{2}):(?P<m2>\d{2}):(?P<s2>\d{2}),(?P<ms2>\d{3})"
)
FORBIDDEN_TERMS = (
    "mission failed",
    "mission_failed",
    "not measured",
    "not_measured",
    "failed",
    "failure",
    "失败",
    "受限",
    "未测",
    "无法",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def fraction(text: object) -> float:
    try:
        return float(Fraction(str(text)))
    except (ValueError, ZeroDivisionError):
        return 0.0


def parse_srt(text: str) -> list[tuple[float, float, str]]:
    cues: list[tuple[float, float, str]] = []
    blocks = re.split(r"\r?\n\r?\n", text.strip())
    for block in blocks:
        lines = [line for line in block.splitlines() if line.strip()]
        if len(lines) < 3:
            continue
        match = SRT_TIME.fullmatch(lines[1].strip())
        if not match:
            raise ValueError(f"invalid SRT timing line: {lines[1]!r}")
        start = (
            int(match.group("h")) * 3600
            + int(match.group("m")) * 60
            + int(match.group("s"))
            + int(match.group("ms")) / 1000
        )
        end = (
            int(match.group("h2")) * 3600
            + int(match.group("m2")) * 60
            + int(match.group("s2"))
            + int(match.group("ms2")) / 1000
        )
        cues.append((start, end, "\n".join(lines[2:])))
    if not cues:
        raise ValueError("SRT contains no cues")
    return cues


def run(command: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )


def ffprobe(path: Path) -> dict[str, object]:
    executable = shutil.which("ffprobe")
    if not executable:
        raise RuntimeError("ffprobe is not available")
    completed = run(
        [
            executable,
            "-v",
            "error",
            "-count_frames",
            "-show_streams",
            "-show_format",
            "-of",
            "json",
            str(path),
        ]
    )
    if completed.returncode != 0:
        raise RuntimeError(completed.stderr.strip() or "ffprobe failed")
    return json.loads(completed.stdout)


def decode_stream(path: Path, stream: str) -> tuple[bool, str]:
    executable = shutil.which("ffmpeg")
    if not executable:
        raise RuntimeError("ffmpeg is not available")
    completed = run(
        [
            executable,
            "-v",
            "error",
            "-i",
            str(path),
            "-map",
            stream,
            "-f",
            "null",
            "-",
        ]
    )
    return completed.returncode == 0, completed.stderr.strip()


def extract_embedded_srt(path: Path) -> str:
    executable = shutil.which("ffmpeg")
    if not executable:
        raise RuntimeError("ffmpeg is not available")
    completed = run(
        [
            executable,
            "-v",
            "error",
            "-i",
            str(path),
            "-map",
            "0:s:0",
            "-f",
            "srt",
            "-",
        ]
    )
    if completed.returncode != 0:
        raise RuntimeError(completed.stderr.strip() or "embedded subtitle extraction failed")
    return completed.stdout


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()

    errors: list[str] = []
    checks: dict[str, str] = {}
    details: dict[str, object] = {}

    try:
        index = json.loads(INDEX_PATH.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError) as exc:
        print(json.dumps({"status": "FAIL", "errors": [str(exc)]}, ensure_ascii=False))
        return 1

    if index.get("status") != "DELIVERED_MODULAR_DEMO":
        errors.append("status must be DELIVERED_MODULAR_DEMO")
    if index.get("official_full_mission_pass") is not False:
        errors.append("official_full_mission_pass must remain false")
    score = index.get("official_item", {}).get("recommended_points_range")
    if score != [2, 3]:
        errors.append("recommended_points_range must remain [2, 3]")
    if index.get("official_item", {}).get("official_points_claimed") != 0.0:
        errors.append("official_points_claimed must remain 0.0")

    for entry in index.get("files", []):
        relative = str(entry.get("path", ""))
        path = (ROOT / relative).resolve()
        if ROOT not in path.parents:
            errors.append(f"path escapes package root: {relative}")
            continue
        if not path.is_file():
            errors.append(f"missing package file: {relative}")
            continue
        if path.stat().st_size != entry.get("bytes"):
            errors.append(f"size mismatch: {relative}")
        if sha256(path) != entry.get("sha256"):
            errors.append(f"SHA-256 mismatch: {relative}")
    checks["paths_sizes_hashes"] = "PASS" if not errors else "FAIL"

    if errors:
        report = {"status": "FAIL", "checks": checks, "errors": errors}
        if args.write:
            DEFAULT_REPORT.parent.mkdir(parents=True, exist_ok=True)
            DEFAULT_REPORT.write_text(
                json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
                newline="\n",
            )
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 1

    video_path = ROOT / "final" / "TZcup_5min_video.mp4"
    silent_path = ROOT / "final" / "TZcup_5min_video_silent.mp4"
    subtitles_path = ROOT / "subtitles" / "zh-CN.srt"
    chapters_path = ROOT / "chapters" / "chapters.json"

    try:
        probe = ffprobe(video_path)
        streams = probe.get("streams", [])
        video = next(
            stream
            for stream in streams
            if isinstance(stream, dict) and stream.get("codec_type") == "video"
        )
        audio = next(
            stream
            for stream in streams
            if isinstance(stream, dict) and stream.get("codec_type") == "audio"
        )
        subtitle = next(
            stream
            for stream in streams
            if isinstance(stream, dict) and stream.get("codec_type") == "subtitle"
        )
        format_info = probe.get("format", {})
        duration = float(format_info.get("duration", 0))
        media_ok = (
            abs(duration - 300.0) <= 0.001
            and int(video.get("width", 0)) == 1920
            and int(video.get("height", 0)) == 1080
            and str(video.get("codec_name", "")).lower() == "h264"
            and abs(fraction(video.get("r_frame_rate")) - 30.0) <= 0.001
            and abs(fraction(video.get("avg_frame_rate")) - 30.0) <= 0.001
            and int(video.get("nb_read_frames", 0)) == 9000
            and str(audio.get("codec_name", "")).lower() == "aac"
            and str(subtitle.get("codec_name", "")).lower() == "mov_text"
        )
        if not media_ok:
            errors.append("final media contract mismatch")
        checks["ffprobe_media_contract"] = "PASS" if media_ok else "FAIL"
        details["media"] = {
            "duration_s": duration,
            "width": int(video.get("width", 0)),
            "height": int(video.get("height", 0)),
            "r_frame_rate": str(video.get("r_frame_rate", "")),
            "avg_frame_rate": str(video.get("avg_frame_rate", "")),
            "frame_count": int(video.get("nb_read_frames", 0)),
            "video_codec": str(video.get("codec_name", "")),
            "audio_codec": str(audio.get("codec_name", "")),
            "subtitle_codec": str(subtitle.get("codec_name", "")),
        }
    except (RuntimeError, StopIteration, ValueError) as exc:
        errors.append(f"ffprobe failed: {exc}")
        checks["ffprobe_media_contract"] = "FAIL"

    video_ok, video_error = decode_stream(video_path, "0:v:0")
    audio_ok, audio_error = decode_stream(video_path, "0:a:0")
    silent_ok, silent_error = decode_stream(silent_path, "0:v:0")
    if not video_ok:
        errors.append(f"video decode failed: {video_error}")
    if not audio_ok:
        errors.append(f"audio decode failed: {audio_error}")
    if not silent_ok:
        errors.append(f"silent master decode failed: {silent_error}")
    checks["full_decode"] = "PASS" if video_ok and audio_ok and silent_ok else "FAIL"

    try:
        sidecar_text = subtitles_path.read_text(encoding="utf-8-sig")
        embedded_text = extract_embedded_srt(video_path)
        sidecar = parse_srt(sidecar_text)
        embedded = parse_srt(embedded_text)
        subtitle_ok = (
            len(sidecar) == 35
            and len(embedded) == 35
            and sidecar[-1][1] == 300.0
            and embedded[-1][1] == 300.0
            and [cue[2].strip() for cue in sidecar]
            == [cue[2].strip() for cue in embedded]
        )
        if not subtitle_ok:
            errors.append("sidecar and embedded subtitles differ or do not end at 300.000 s")
        checks["srt_and_embedded_subtitles"] = "PASS" if subtitle_ok else "FAIL"
        details["subtitle_cues"] = len(sidecar)
    except (OSError, RuntimeError, ValueError) as exc:
        errors.append(f"subtitle validation failed: {exc}")
        checks["srt_and_embedded_subtitles"] = "FAIL"

    try:
        chapters = json.loads(chapters_path.read_text(encoding="utf-8-sig"))
        if float(chapters.get("duration_sec", 0)) != 300.0:
            errors.append("chapters duration must be 300 seconds")
        scan_text = sidecar_text + "\n" + json.dumps(chapters, ensure_ascii=False)
        lower = scan_text.lower()
        hits = [term for term in FORBIDDEN_TERMS if term in lower]
        if hits:
            errors.append(f"forbidden presentation terms found: {hits}")
        checks["presentation_forbidden_terms"] = "PASS" if not hits else "FAIL"
    except (OSError, ValueError) as exc:
        errors.append(f"chapter/forbidden-term validation failed: {exc}")
        checks["presentation_forbidden_terms"] = "FAIL"

    for relative in (
        "video-index.json",
        "final/manifest.json",
        "final/ffprobe.json",
        "chapters/chapters.json",
    ):
        try:
            json.loads((ROOT / relative).read_text(encoding="utf-8-sig"))
        except (OSError, ValueError) as exc:
            errors.append(f"JSON parse failed for {relative}: {exc}")
    checks["json_parse"] = "PASS" if not any("JSON parse failed" in e for e in errors) else "FAIL"

    report = {
        "schema_version": 1,
        "generated_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "status": "PASS" if not errors else "FAIL",
        "artifact_status": index.get("status"),
        "claim_boundary": index.get("claim_boundary"),
        "checks": checks,
        "details": details,
        "errors": errors,
    }
    if args.write:
        DEFAULT_REPORT.parent.mkdir(parents=True, exist_ok=True)
        DEFAULT_REPORT.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
            newline="\n",
        )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
