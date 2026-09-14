#!/usr/bin/env python3
"""Validate a candidate 5-10 minute competition demo with ffprobe."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from fractions import Fraction
from pathlib import Path


MIN_DURATION_S = 300.0
MAX_DURATION_S = 600.0
MIN_WIDTH = 1280
MIN_HEIGHT = 720
MIN_FPS = 24.0
ALLOWED_VIDEO_CODECS = {"h264", "hevc"}


def parse_fraction(value: str | None) -> float:
    if not value or value in {"0/0", "N/A"}:
        return 0.0
    try:
        return float(Fraction(value))
    except (ValueError, ZeroDivisionError):
        return 0.0


def extract_media_info(probe: dict[str, object]) -> dict[str, object]:
    streams = probe.get("streams")
    if not isinstance(streams, list):
        raise ValueError("ffprobe output has no streams list")
    video = next(
        (
            stream
            for stream in streams
            if isinstance(stream, dict) and stream.get("codec_type") == "video"
        ),
        None,
    )
    if not isinstance(video, dict):
        raise ValueError("ffprobe output has no video stream")

    format_info = probe.get("format")
    duration = 0.0
    if isinstance(format_info, dict):
        try:
            duration = float(format_info.get("duration", 0.0))
        except (TypeError, ValueError):
            duration = 0.0

    has_audio = any(
        isinstance(stream, dict) and stream.get("codec_type") == "audio"
        for stream in streams
    )
    return {
        "duration_s": duration,
        "width": int(video.get("width", 0)),
        "height": int(video.get("height", 0)),
        "avg_fps": parse_fraction(video.get("avg_frame_rate")),
        "video_codec": str(video.get("codec_name", "")).lower(),
        "has_audio": has_audio,
    }


def validate_media(
    info: dict[str, object],
    captions_present: bool = False,
) -> list[str]:
    errors: list[str] = []
    duration = float(info.get("duration_s", 0.0))
    if not MIN_DURATION_S <= duration <= MAX_DURATION_S:
        errors.append(
            f"duration must be {MIN_DURATION_S:.0f}-{MAX_DURATION_S:.0f}s; got {duration:.3f}s"
        )
    if int(info.get("width", 0)) < MIN_WIDTH or int(info.get("height", 0)) < MIN_HEIGHT:
        errors.append(f"resolution must be at least {MIN_WIDTH}x{MIN_HEIGHT}")
    if float(info.get("avg_fps", 0.0)) < MIN_FPS:
        errors.append(f"frame rate must be at least {MIN_FPS:.0f} fps")
    if str(info.get("video_codec", "")).lower() not in ALLOWED_VIDEO_CODECS:
        errors.append("video codec must be h264 or hevc")
    if not bool(info.get("has_audio")) and not captions_present:
        errors.append("audio stream or sidecar captions are required")
    return errors


def ffprobe(path: Path) -> dict[str, object]:
    executable = shutil.which("ffprobe")
    if not executable:
        raise RuntimeError("ffprobe is not available")
    completed = subprocess.run(
        [
            executable,
            "-v",
            "error",
            "-show_streams",
            "-show_format",
            "-of",
            "json",
            str(path),
        ],
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
    )
    return json.loads(completed.stdout)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("video", type=Path)
    parser.add_argument("--captions", type=Path)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    try:
        info = extract_media_info(ffprobe(args.video))
        captions_present = bool(args.captions and args.captions.is_file())
        if args.captions and not captions_present:
            errors = [f"captions file does not exist: {args.captions}"]
            status = "FAIL"
        else:
            errors = validate_media(info, captions_present)
            status = "PASS" if not errors else "FAIL"
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as exc:
        info = {}
        captions_present = False
        errors = [str(exc)]
        status = "ERROR"

    report = {
        "status": status,
        "video": str(args.video),
        "captions_present": captions_present,
        "media": info,
        "requirements": {
            "duration_s": [MIN_DURATION_S, MAX_DURATION_S],
            "minimum_resolution": [MIN_WIDTH, MIN_HEIGHT],
            "minimum_fps": MIN_FPS,
            "allowed_video_codecs": sorted(ALLOWED_VIDEO_CODECS),
            "audio_or_captions": True,
        },
        "errors": errors,
        "claim_policy": "A FAIL or ERROR cannot be reported as a completed official demo.",
    }
    payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.write:
        evidence = root / "evidence"
        evidence.mkdir(exist_ok=True)
        output = evidence / "demo-media-validation.json"
        output.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
