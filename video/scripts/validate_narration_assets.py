#!/usr/bin/env python3
"""Validate narration timing, text policy, and generated audio."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TIMELINE = Path(__file__).with_name("narration_timeline.json")


def run(command: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, capture_output=True, text=True, check=False)


def parse_srt(path: Path) -> list[dict[str, Any]]:
    text = path.read_text(encoding="utf-8")
    pattern = re.compile(
        r"(?ms)^(?P<index>\d+)\s*\n"
        r"(?P<start>\d{2}:\d{2}:\d{2},\d{3}) --> "
        r"(?P<end>\d{2}:\d{2}:\d{2},\d{3})\s*\n"
        r"(?P<text>.+?)(?=\n\s*\n|\Z)"
    )
    rows: list[dict[str, Any]] = []
    for match in pattern.finditer(text):
        rows.append(
            {
                "index": int(match.group("index")),
                "start": match.group("start"),
                "end": match.group("end"),
                "text": match.group("text").strip(),
            }
        )
    return rows


def srt_timestamp(seconds: float) -> str:
    total_ms = round(seconds * 1000)
    hours, remainder = divmod(total_ms, 3_600_000)
    minutes, remainder = divmod(remainder, 60_000)
    secs, millis = divmod(remainder, 1000)
    return f"{hours:02d}:{minutes:02d}:{secs:02d},{millis:03d}"


def ffprobe_audio(path: Path) -> dict[str, Any]:
    completed = run(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "a:0",
            "-show_entries",
            "stream=codec_name,sample_rate,channels:format=duration,size",
            "-of",
            "json",
            str(path),
        ]
    )
    if completed.returncode != 0:
        raise ValueError(completed.stderr.strip() or "ffprobe failed")
    payload = json.loads(completed.stdout)
    stream = (payload.get("streams") or [{}])[0]
    return {
        "codec": stream.get("codec_name"),
        "sample_rate": int(stream.get("sample_rate", 0)),
        "channels": int(stream.get("channels", 0)),
        "duration_sec": float(payload["format"]["duration"]),
        "size_bytes": int(payload["format"]["size"]),
    }


def loudness(path: Path) -> dict[str, Any]:
    completed = run(
        [
            "ffmpeg",
            "-hide_banner",
            "-nostats",
            "-i",
            str(path),
            "-af",
            "loudnorm=I=-16:TP=-1.5:LRA=11:print_format=json",
            "-f",
            "null",
            "-",
        ]
    )
    if completed.returncode != 0:
        raise ValueError(completed.stderr.strip() or "ffmpeg loudness scan failed")
    match = re.search(
        r"\[Parsed_loudnorm[^\]]*\]\s*(\{.*?\})",
        completed.stderr,
        re.DOTALL,
    )
    if not match:
        raise ValueError("loudness scan did not return JSON")
    return json.loads(match.group(1))


def silence(path: Path) -> dict[str, Any]:
    completed = run(
        [
            "ffmpeg",
            "-hide_banner",
            "-nostats",
            "-i",
            str(path),
            "-af",
            "silencedetect=noise=-45dB:d=0.6",
            "-f",
            "null",
            "-",
        ]
    )
    if completed.returncode != 0:
        raise ValueError(completed.stderr.strip() or "ffmpeg silence scan failed")
    durations = [
        float(value)
        for value in re.findall(r"silence_duration:\s*([0-9.]+)", completed.stderr)
    ]
    return {
        "threshold_db": -45.0,
        "minimum_duration_sec": 0.6,
        "detected_count": len(durations),
        "longest_sec": max(durations, default=0.0),
    }


def validate_text(
    timeline: dict[str, Any], srt_rows: list[dict[str, Any]]
) -> dict[str, Any]:
    expected = timeline["segments"]
    if len(expected) != len(srt_rows):
        raise ValueError(
            f"SRT has {len(srt_rows)} cues, timeline has {len(expected)} segments"
        )
    forbidden = timeline.get("forbidden_terms", [])
    combined_parts = []
    for segment, row in zip(expected, srt_rows, strict=True):
        expected_start = srt_timestamp(float(segment["start_sec"]))
        expected_end = srt_timestamp(float(segment["end_sec"]))
        if row["start"] != expected_start or row["end"] != expected_end:
            raise ValueError(
                f"SRT cue {row['index']} timing differs from timeline segment {segment['id']}"
            )
        if row["text"] != str(segment["speech"]).strip():
            raise ValueError(f"SRT cue {row['index']} differs from timeline segment {segment['id']}")
        combined_parts.extend([str(segment["speech"]), str(segment["screen_text"])])
    for chapter in timeline["chapters"]:
        combined_parts.extend(
            [str(chapter["title"]), str(chapter["subtitle"]), str(chapter["source_kind"])]
        )
    narration_path = ROOT / "video" / "scripts" / "narration.md"
    if narration_path.is_file():
        combined_parts.append(narration_path.read_text(encoding="utf-8"))
    combined = "\n".join(combined_parts).casefold()
    hits = [term for term in forbidden if term.casefold() in combined]
    if hits:
        raise ValueError(f"forbidden terms in video text: {hits}")

    srt_text = "\n".join(row["text"] for row in srt_rows)
    if not srt_text.strip():
        raise ValueError("SRT text is empty")
    return {
        "cue_count": len(srt_rows),
        "first_cue_start": srt_rows[0]["start"],
        "last_cue_end": srt_rows[-1]["end"],
        "forbidden_term_hits": hits,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeline", type=Path, default=DEFAULT_TIMELINE)
    parser.add_argument("--video-root", type=Path, default=ROOT / "video")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    timeline = json.loads(args.timeline.read_text(encoding="utf-8"))
    srt_rows = parse_srt(args.video_root / "subtitles" / "zh-CN.srt")
    result: dict[str, Any] = {
        "schema_version": 1,
        "status": "PASS",
        "text": validate_text(timeline, srt_rows),
        "audio": None,
    }

    wav_path = args.video_root / "audio" / "narration.wav"
    mp3_path = args.video_root / "audio" / "narration.mp3"
    available_audio = [path for path in (wav_path, mp3_path) if path.is_file()]
    if available_audio:
        audio_results: dict[str, Any] = {}
        for path in available_audio:
            metadata = ffprobe_audio(path)
            tolerance = 0.12 if path.suffix == ".wav" else 0.35
            if abs(metadata["duration_sec"] - float(timeline["duration_sec"])) > tolerance:
                raise ValueError(f"{path.name} duration differs from the 300 second timeline")
            audio_results[path.suffix.lstrip(".")] = metadata
        loudness_path = wav_path if wav_path.is_file() else mp3_path
        loudness_stats = loudness(loudness_path)
        silence_stats = silence(loudness_path)
        input_i = float(loudness_stats["input_i"])
        input_tp = float(loudness_stats["input_tp"])
        if abs(input_i + 16.0) > 1.5:
            raise ValueError(f"integrated loudness is outside target: {input_i}")
        if input_tp > -1.0:
            raise ValueError(f"true peak is too high: {input_tp}")
        result["audio"] = {
            **audio_results,
            "integrated_lufs": input_i,
            "true_peak_dbtp": input_tp,
            "loudness_range_lu": float(loudness_stats["input_lra"]),
            "silence": silence_stats,
        }

    output = args.output or args.video_root / "validation" / "narration-assets.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
