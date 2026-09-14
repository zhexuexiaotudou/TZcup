#!/usr/bin/env python3
"""Render the narration, SRT, and chapter cards from one timeline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TIMELINE = Path(__file__).with_name("narration_timeline.json")


def load_timeline(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ValueError("unsupported timeline schema")
    return data


def validate_timeline(data: dict[str, Any]) -> None:
    expected_duration = float(data["duration_sec"])
    previous_end = 0.0
    previous_id: str | None = None
    forbidden = [term.casefold() for term in data.get("forbidden_terms", [])]

    for segment in data["segments"]:
        segment_id = str(segment["id"])
        start = float(segment["start_sec"])
        end = float(segment["end_sec"])
        if start < previous_end:
            raise ValueError(
                f"segment {segment_id} overlaps or reverses {previous_id}: "
                f"{start} < {previous_end}"
            )
        if end <= start:
            raise ValueError(f"segment {segment_id} has a non-positive duration")
        speech = str(segment["speech"]).strip()
        if not speech:
            raise ValueError(f"segment {segment_id} has empty narration")
        lowered = speech.casefold()
        for term in forbidden:
            if term and term in lowered:
                raise ValueError(f"segment {segment_id} contains forbidden term {term!r}")
        previous_end = end
        previous_id = segment_id

    if abs(previous_end - expected_duration) > 1e-6:
        raise ValueError(
            f"timeline ends at {previous_end}s, expected {expected_duration}s"
        )


def srt_timestamp(seconds: float) -> str:
    total_ms = round(seconds * 1000)
    hours, remainder = divmod(total_ms, 3_600_000)
    minutes, remainder = divmod(remainder, 60_000)
    secs, millis = divmod(remainder, 1000)
    return f"{hours:02d}:{minutes:02d}:{secs:02d},{millis:03d}"


def render_srt(data: dict[str, Any]) -> str:
    blocks: list[str] = []
    for index, segment in enumerate(data["segments"], start=1):
        blocks.append(
            "\n".join(
                [
                    str(index),
                    (
                        f"{srt_timestamp(float(segment['start_sec']))} --> "
                        f"{srt_timestamp(float(segment['end_sec']))}"
                    ),
                    str(segment["speech"]).strip(),
                ]
            )
        )
    return "\n\n".join(blocks) + "\n"


def render_narration_markdown(data: dict[str, Any]) -> str:
    lines = [
        f"# {data['title']} - 5分钟成片解说稿",
        "",
        f"副标题：{data['subtitle']}",
        "",
        "总时长：05:00。解说与字幕由同一时间轴生成，逐句对齐。",
        "",
        "## 画面标识约定",
        "",
        "- `模块特写`用于整车与部件结构说明，镜头角标显示对应模块名称。",
        "- `成果演示`用于真实运行录像，画面角标显示运行主题。",
        "- `成果画面`用于环境建模结果和量化结果展示。",
        "- 章节卡采用短标题加大号数字，章节内字幕保持在画面下方安全区。",
        "",
        "## 章节卡",
        "",
        "| 时间 | 编号 | 标题 | 副标题 | 标识 |",
        "|---|---:|---|---|---|",
    ]
    for chapter in data["chapters"]:
        lines.append(
            "| {start}-{end} | {number} | {title} | {subtitle} | {source_kind} |".format(
                start=srt_timestamp(float(chapter["start_sec"]))[:8],
                end=srt_timestamp(float(chapter["end_sec"]))[:8],
                **chapter,
            )
        )

    lines.extend(
        [
            "",
            "## 逐句解说与分镜",
            "",
            "| 时间 | 章节 | 解说 | 屏幕文字 | 画面执行 | 标识 |",
            "|---|---:|---|---|---|---|",
        ]
    )
    for segment in data["segments"]:
        lines.append(
            "| {start}-{end} | {chapter_number} | {speech} | {screen_text} | "
            "{visual} | {source_kind} |".format(
                start=srt_timestamp(float(segment["start_sec"]))[:8],
                end=srt_timestamp(float(segment["end_sec"]))[:8],
                **segment,
            )
        )

    lines.extend(
        [
            "",
            "## 声音方案",
            "",
            "旁白使用中文女声，语速保持信息清晰，句间以短停顿衔接章节。",
            "背景音乐建议使用低频电子与轻量节拍组合，进入清扫和建图段时逐级提升空间感。",
            "机械臂、刷盘、泵阀和车辆运动音效按画面动作落点叠加，整体旁白保持在中心清晰位置。",
            "",
            "## 成片节奏",
            "",
            "- 00:00-01:00 整车建模与模块分解。",
            "- 01:00-02:10 基础运动、机械臂抓取与感知联动。",
            "- 02:10-03:40 水渍清洁与环境建模成果。",
            "- 03:40-05:00 连续清扫、安全响应与系统总结。",
            "",
        ]
    )
    return "\n".join(lines)


def render_chapters_json(data: dict[str, Any]) -> str:
    payload = {
        "schema_version": 1,
        "duration_sec": data["duration_sec"],
        "chapters": data["chapters"],
    }
    return json.dumps(payload, ensure_ascii=False, indent=2) + "\n"


def render_chapters_text(data: dict[str, Any]) -> str:
    lines = [
        f"{data['title']} - 章节卡文字",
        "",
        "格式：[进入时间-结束时间] 编号 标题 / 副标题 / 标识",
        "",
    ]
    for chapter in data["chapters"]:
        lines.append(
            "[{start}-{end}] {number} {title} / {subtitle} / {source_kind}".format(
                start=srt_timestamp(float(chapter["start_sec"]))[:8],
                end=srt_timestamp(float(chapter["end_sec"]))[:8],
                **chapter,
            )
        )
    return "\n".join(lines) + "\n"


def write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8", newline="\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeline", type=Path, default=DEFAULT_TIMELINE)
    parser.add_argument("--output-root", type=Path, default=ROOT / "video")
    args = parser.parse_args()

    data = load_timeline(args.timeline)
    validate_timeline(data)

    write_text(args.output_root / "scripts" / "narration.md", render_narration_markdown(data))
    write_text(args.output_root / "subtitles" / "zh-CN.srt", render_srt(data))
    write_text(args.output_root / "chapters" / "chapters.json", render_chapters_json(data))
    write_text(args.output_root / "chapters" / "chapters.txt", render_chapters_text(data))
    print(
        json.dumps(
            {
                "status": "PASS",
                "segments": len(data["segments"]),
                "chapters": len(data["chapters"]),
                "duration_sec": data["duration_sec"],
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
