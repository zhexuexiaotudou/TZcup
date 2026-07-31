#!/usr/bin/env python3
"""Render the read-only live dashboard telemetry into deterministic video frames."""

from __future__ import annotations

import argparse
import json
import math
import time
import urllib.request
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


WIDTH = 1600
HEIGHT = 900
BACKGROUND = "#07100e"
PANEL = "#0b1714"
LINE = "#253b34"
TEXT = "#effff8"
MUTED = "#8da79d"
MINT = "#66f1b5"
CYAN = "#69d7ff"
AMBER = "#ffc66d"
RED = "#ff7b7b"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--stop-file", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=2.0)
    return parser.parse_args()


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    candidates = [
        Path("C:/Windows/Fonts/msyhbd.ttc" if bold else "C:/Windows/Fonts/msyh.ttc"),
        Path("C:/Windows/Fonts/segoeuib.ttf" if bold else "C:/Windows/Fonts/segoeui.ttf"),
    ]
    for candidate in candidates:
        if candidate.is_file():
            return ImageFont.truetype(str(candidate), size)
    return ImageFont.load_default()


FONTS = {size: font(size, size >= 28) for size in (18, 22, 28, 38, 54)}


def fetch(url: str) -> dict:
    with urllib.request.urlopen(url, timeout=2.0) as response:
        return json.load(response)


def draw_text(draw: ImageDraw.ImageDraw, xy: tuple[int, int], value: object, size: int, fill: str = TEXT) -> None:
    draw.text(xy, str(value), font=FONTS[size], fill=fill)


def map_bounds(data: dict) -> tuple[float, float, float, float]:
    visual = data.get("visualization", {})
    geometry = visual.get("geometry", {})
    points = list(geometry.get("outer_polygon", []))
    points += list(visual.get("planned_path", []))
    points += list(visual.get("evaluation_only_trajectory", []))
    if not points:
        return -3.0, 7.0, -5.0, 5.0
    xs = [float(point[0]) for point in points]
    ys = [float(point[1]) for point in points]
    padding = 0.8
    return min(xs) - padding, max(xs) + padding, min(ys) - padding, max(ys) + padding


def render_polyline(
    draw: ImageDraw.ImageDraw,
    points: list,
    transform,
    color: str,
    width: int,
) -> None:
    if len(points) >= 2:
        draw.line([transform(point) for point in points], fill=color, width=width, joint="curve")


def render_frame(data: dict, frame_index: int) -> Image.Image:
    image = Image.new("RGB", (WIDTH, HEIGHT), BACKGROUND)
    draw = ImageDraw.Draw(image)
    draw_text(draw, (44, 28), "AUTO-17 · LIVE GAZEBO NAVIGATION COVERAGE", 18, MINT)
    draw_text(draw, (44, 64), "清扫任务正在发生。", 54)
    draw_text(draw, (44, 132), "真值仅用于评估与绘图，不参与车辆控制", 18, AMBER)

    map_box = (44, 180, 1120, 850)
    draw.rounded_rectangle(map_box, radius=8, fill=PANEL, outline=LINE, width=2)
    draw_text(draw, (70, 204), "任务地图与车辆轨迹", 28)
    plot = (70, 252, 1094, 824)
    draw.rectangle(plot, fill="#06100e", outline=LINE, width=2)

    min_x, max_x, min_y, max_y = map_bounds(data)
    scale = min((plot[2] - plot[0] - 48) / max(0.1, max_x - min_x), (plot[3] - plot[1] - 48) / max(0.1, max_y - min_y))
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2

    def transform(point) -> tuple[int, int]:
        x = plot[0] + (plot[2] - plot[0]) / 2 + (float(point[0]) - center_x) * scale
        y = plot[1] + (plot[3] - plot[1]) / 2 - (float(point[1]) - center_y) * scale
        return round(x), round(y)

    visual = data.get("visualization", {})
    geometry = visual.get("geometry", {})
    outer = geometry.get("outer_polygon", [])
    if len(outer) >= 3:
        polygon = [transform(point) for point in outer]
        draw.polygon(polygon, fill="#10261f", outline="#527f6f", width=3)
    for keepout in geometry.get("keepout_polygons", []):
        if len(keepout) >= 3:
            draw.polygon([transform(point) for point in keepout], fill="#482a29", outline=RED, width=3)

    render_polyline(draw, visual.get("planned_path", []), transform, CYAN, 5)
    render_polyline(draw, visual.get("evaluation_only_trajectory", []), transform, "#607b72", 5)
    render_polyline(draw, visual.get("evaluation_only_cleaned_trajectory", []), transform, MINT, 7)

    pose = data.get("vehicle", {}).get("evaluation_only_pose_map") or data.get("vehicle", {}).get("estimated_pose_map")
    if pose and len(pose) >= 3:
        x, y = transform(pose)
        yaw = float(pose[2])
        tip = (x + round(24 * math.cos(yaw)), y - round(24 * math.sin(yaw)))
        left = (x + round(12 * math.cos(yaw + 2.5)), y - round(12 * math.sin(yaw + 2.5)))
        right = (x + round(12 * math.cos(yaw - 2.5)), y - round(12 * math.sin(yaw - 2.5)))
        draw.polygon([tip, left, right], fill=AMBER, outline="#fff2ce")

    side = (1142, 180, 1556, 850)
    draw.rounded_rectangle(side, radius=8, fill=PANEL, outline=LINE, width=2)
    progress = data.get("progress", {})
    status = data.get("status", "BOOTING")
    draw_text(draw, (1170, 208), "MISSION STATE", 18, MUTED)
    draw_text(draw, (1170, 242), status, 38, MINT)
    completed = int(progress.get("completed_components", 0))
    expected = int(progress.get("expected_components", 0))
    ratio = float(progress.get("ratio", 0.0))
    draw_text(draw, (1170, 306), f"组件 {completed}/{expected}", 22)
    draw.rounded_rectangle((1170, 348, 1528, 366), radius=9, fill="#172a24")
    draw.rounded_rectangle((1170, 348, 1170 + round(358 * min(1.0, ratio)), 366), radius=9, fill=MINT)
    draw_text(draw, (1170, 388), f"完成度 {round(ratio * 100)}%", 22)
    vehicle = data.get("vehicle", {})
    cleaning = data.get("cleaning", {})
    draw_text(draw, (1170, 452), "线速度", 18, MUTED)
    draw_text(draw, (1170, 480), f"{float(vehicle.get('linear_speed_m_s', 0.0)):.2f} m/s", 28)
    draw_text(draw, (1170, 540), "当前组件", 18, MUTED)
    draw_text(draw, (1170, 568), progress.get("current_component") or "—", 28)
    draw_text(draw, (1170, 628), "清扫刷盘", 18, MUTED)
    draw_text(draw, (1170, 656), "开启" if cleaning.get("brush_enabled") else "关闭", 28, MINT if cleaning.get("brush_enabled") else AMBER)
    draw_text(draw, (1170, 716), "安全状态", 18, MUTED)
    draw_text(draw, (1170, 744), "急停" if cleaning.get("emergency_stop") else "正常", 28, RED if cleaning.get("emergency_stop") else MINT)
    draw_text(draw, (1170, 802), f"Frame {frame_index:06d} · {float(data.get('elapsed_sec', 0.0)):.1f}s", 18, MUTED)
    return image


def main() -> int:
    args = parse_args()
    if args.fps <= 0:
        raise ValueError("--fps must be positive")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.stop_file.unlink(missing_ok=True)
    interval = 1.0 / args.fps
    next_frame_at = time.monotonic()
    frame_index = 0
    last_data: dict = {}
    while not args.stop_file.exists():
        try:
            last_data = fetch(args.url)
        except Exception:
            pass
        render_frame(last_data, frame_index).save(
            args.output_dir / f"frame_{frame_index:06d}.jpg",
            "JPEG",
            quality=88,
            optimize=True,
        )
        frame_index += 1
        next_frame_at += interval
        time.sleep(max(0.0, next_frame_at - time.monotonic()))
    return 0 if frame_index else 1


if __name__ == "__main__":
    raise SystemExit(main())
