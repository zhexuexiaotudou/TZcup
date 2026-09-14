#!/usr/bin/env python3
"""Render deterministic map and module-mosaic assets for the day1 video."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


FONT_REGULAR = Path(r"C:\Windows\Fonts\msyh.ttc")
FONT_BOLD = Path(r"C:\Windows\Fonts\msyhbd.ttc")


def font(size: int, *, bold: bool = False) -> ImageFont.FreeTypeFont:
    path = FONT_BOLD if bold else FONT_REGULAR
    return ImageFont.truetype(str(path), size=size)


def fit_image(image: Image.Image, size: tuple[int, int]) -> Image.Image:
    image = image.convert("RGB")
    scale = max(size[0] / image.width, size[1] / image.height)
    resized = image.resize(
        (math.ceil(image.width * scale), math.ceil(image.height * scale)),
        Image.Resampling.LANCZOS,
    )
    left = (resized.width - size[0]) // 2
    top = (resized.height - size[1]) // 2
    return resized.crop((left, top, left + size[0], top + size[1]))


def render_module_mosaic(source_root: Path, output_path: Path) -> None:
    canvas = Image.new("RGB", (1920, 1080), "#f3f1eb")
    draw = ImageDraw.Draw(canvas)
    draw.rounded_rectangle(
        (36, 28, 1884, 1052),
        radius=18,
        fill="#f8f7f2",
        outline="#d5d9d3",
        width=3,
    )
    draw.text((72, 52), "模块分解示意", font=font(46, bold=True), fill="#18221e")
    draw.text(
        (72, 112),
        "底盘与车身  /  清扫系统  /  机械臂  /  感知与收纳",
        font=font(25),
        fill="#50605a",
    )

    cards = [
        (
            source_root / "report" / "figures" / "formal_front_left.png",
            "底盘与车身",
            (72, 158, 930, 570),
        ),
        (
            source_root / "report" / "figures" / "formal_top_cleaning.png",
            "清扫系统",
            (990, 158, 1848, 570),
        ),
        (
            source_root / "report" / "figures" / "formal_rear_right.png",
            "机械臂",
            (72, 604, 930, 1016),
        ),
        (
            source_root / "report" / "figures" / "formal_sensor_tower.png",
            "感知与收纳",
            (990, 604, 1848, 1016),
        ),
    ]

    for source_path, label, box in cards:
        image = fit_image(Image.open(source_path), (box[2] - box[0], box[3] - box[1]))
        canvas.paste(image, (box[0], box[1]))
        draw.rounded_rectangle(
            (box[0], box[3] - 58, box[2], box[3]),
            radius=10,
            fill="#101714",
        )
        draw.text(
            (box[0] + 24, box[3] - 51),
            label,
            font=font(28, bold=True),
            fill="#f5f7f4",
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output_path, quality=96)


def render_map(source_root: Path, output_path: Path) -> None:
    manifest_path = (
        source_root
        / "reports"
        / "mapping"
        / "offline_raycast_mapping"
        / "offline_raycast_manifest.json"
    )
    if not manifest_path.is_file():
        manifest_path = (
            Path(__file__).resolve().parents[1]
            / "reports"
            / "mapping"
            / "offline_raycast_mapping"
            / "offline_raycast_manifest.json"
        )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    canvas = Image.new("RGB", (1920, 1080), "#eef1ed")
    draw = ImageDraw.Draw(canvas)
    draw.rectangle((0, 0, 1920, 132), fill="#15201c")
    draw.text(
        (64, 28),
        "园区二维地图重建",
        font=font(48, bold=True),
        fill="#f7faf7",
    )
    draw.text(
        (66, 88),
        "离线一致性重建  ·  0.05 米栅格  ·  仿真环境",
        font=font(24),
        fill="#c8d6cf",
    )

    field = (-100.0, -50.0, 100.0, 50.0)
    margin = 96
    plot_left, plot_top, plot_right, plot_bottom = 64, 174, 1450, 1010
    scale = min(
        (plot_right - plot_left) / (field[2] - field[0]),
        (plot_bottom - plot_top) / (field[3] - field[1]),
    )

    def project(x: float, y: float) -> tuple[float, float]:
        cx = (plot_left + plot_right) / 2
        cy = (plot_top + plot_bottom) / 2
        return cx + x * scale, cy - y * scale

    draw.rounded_rectangle(
        (plot_left - 16, plot_top - 16, plot_right + 16, plot_bottom + 16),
        radius=18,
        fill="#f9faf7",
        outline="#aeb9b2",
        width=3,
    )

    grid_step_m = 10
    for x in range(-100, 101, grid_step_m):
        x0, y0 = project(x, -50)
        x1, y1 = project(x, 50)
        draw.line((x0, y0, x1, y1), fill="#dde3de", width=2)
    for y in range(-50, 51, grid_step_m):
        x0, y0 = project(-100, y)
        x1, y1 = project(100, y)
        draw.line((x0, y0, x1, y1), fill="#dde3de", width=2)

    draw.rectangle(
        (
            project(field[0], field[3])[0],
            project(field[0], field[3])[1],
            project(field[2], field[1])[0],
            project(field[2], field[1])[1],
        ),
        fill="#e4eae3",
        outline="#6d7e75",
        width=3,
    )

    path = manifest["pose_path"]["points_source_world_m"]
    points = [project(float(x), float(y)) for x, y in path]
    draw.line(points, fill="#149a70", width=7, joint="curve")
    for point in (points[0], points[-1]):
        px, py = point
        draw.ellipse((px - 10, py - 10, px + 10, py + 10), fill="#0d6d50")

    buildings: list[tuple[float, float, float, float, str]] = []
    circles: list[tuple[float, float, float, str]] = []
    for obstacle in manifest["source"]["scan_plane_collisions"]:
        name = obstacle["name"]
        x = float(obstacle["center_x_m"])
        y = float(obstacle["center_y_m"])
        if obstacle["shape"] == "box":
            half_x = float(obstacle["half_x_m"])
            half_y = float(obstacle["half_y_m"])
            if name.startswith("building") or name.startswith("service_building"):
                buildings.append((x - half_x, y - half_y, x + half_x, y + half_y, name))
            else:
                x0, y1 = project(x - half_x, y - half_y)
                x1, y0 = project(x + half_x, y + half_y)
                draw.rectangle((x0, y0, x1, y1), fill="#3c4841")
        elif obstacle["shape"] == "cylinder":
            radius = float(obstacle["half_x_m"])
            circles.append((x, y, radius, name))

    for x0w, y0w, x1w, y1w, _ in buildings:
        x0, y1 = project(x0w, y0w)
        x1, y0 = project(x1w, y1w)
        draw.rounded_rectangle((x0, y0, x1, y1), radius=8, fill="#26322c")

    for x, y, radius, name in circles:
        px, py = project(x, y)
        radius_px = max(4, radius * scale)
        if name.startswith("tree"):
            draw.ellipse(
                (px - radius_px, py - radius_px, px + radius_px, py + radius_px),
                fill="#348465",
            )
        else:
            draw.ellipse(
                (px - radius_px, py - radius_px, px + radius_px, py + radius_px),
                fill="#5d6a63",
            )

    panel = (1492, 174, 1856, 836)
    draw.rounded_rectangle(panel, radius=18, fill="#1a2520")
    draw.text((1526, 206), "重建地图指标", font=font(34, bold=True), fill="#ffffff")
    draw.text((1526, 282), "已知区域", font=font(24), fill="#bfd0c7")
    draw.text((1526, 318), "约 2.24 万平方米", font=font(38, bold=True), fill="#7be0b5")
    draw.text((1526, 414), "栅格分辨率", font=font(24), fill="#bfd0c7")
    draw.text((1526, 450), "0.05 米", font=font(38, bold=True), fill="#ffffff")
    draw.text((1526, 546), "重建路径", font=font(24), fill="#bfd0c7")
    draw.text((1526, 582), f"{manifest['pose_path']['route_length_m']:.1f} 米", font=font(38, bold=True), fill="#ffffff")
    draw.text((1526, 678), "已重建障碍", font=font(24), fill="#bfd0c7")
    draw.text(
        (1526, 714),
        f"{len(manifest['source']['scan_plane_collisions'])} 个",
        font=font(38, bold=True),
        fill="#ffffff",
    )
    draw.text(
        (1526, 790),
        "道路、建筑与主要边界",
        font=font(21),
        fill="#c9d6d0",
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output_path, quality=96)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    args = parser.parse_args()

    render_module_mosaic(args.source_root, args.output_root / "module-mosaic.png")
    render_map(args.source_root, args.output_root / "map-reconstruction.png")
    print(args.output_root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
