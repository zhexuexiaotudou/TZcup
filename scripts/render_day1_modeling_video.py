#!/usr/bin/env python3
"""Render the 60-second TZcup opening modeling segment.

The renderer uses only the five approved vehicle renders under video/assets.
It creates camera movement, component callouts, and an explicitly labelled
module decomposition schematic. No geometry is invented and no simulated
mission frames are mixed into this segment.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import subprocess
from collections.abc import Callable
from functools import lru_cache
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont, ImageStat

WIDTH = 1920
HEIGHT = 1080
FPS = 30
DURATION_SECONDS = 60.0
FRAME_COUNT = round(DURATION_SECONDS * FPS)
VIDEO_SIZE = (WIDTH, HEIGHT)

ROOT = Path(__file__).resolve().parents[1]
ASSET_DIR = ROOT / "video" / "assets"
SEGMENT_DIR = ROOT / "video" / "segments"
EVIDENCE_DIR = ROOT / "video" / "evidence"
KEYFRAME_DIR = EVIDENCE_DIR / "keyframes"
OUTPUT_PATH = SEGMENT_DIR / "01_modeling.mp4"
SHOT_LIST_PATH = ROOT / "video" / "modeling_shot_list.json"
FONT_PATH = Path(r"C:\Windows\Fonts\simhei.ttf")

CHARCOAL = (12, 17, 18)
CHARCOAL_2 = (23, 31, 31)
INK = (17, 23, 24)
WARM_WHITE = (244, 245, 241)
MUTED = (179, 190, 187)
GREEN = (24, 170, 111)
GREEN_2 = (80, 214, 151)
CYAN = (64, 176, 214)
AMBER = (235, 176, 60)
BLUE = (52, 132, 209)
PANEL = (28, 36, 36)

BANNED_VISIBLE_WORDS = (
    "失败",
    "受限",
    "未测",
    "mission failed",
    "NOT_MEASURED",
)


def clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def ease(t: float) -> float:
    t = clamp(t)
    return t * t * (3.0 - 2.0 * t)


def ease_out(t: float) -> float:
    t = clamp(t)
    return 1.0 - (1.0 - t) ** 3


def ease_in_out(t: float) -> float:
    t = clamp(t)
    return 0.5 - 0.5 * math.cos(math.pi * t)


def fade_window(t: float, start: float, end: float, edge: float) -> float:
    return ease((t - start) / edge) * ease((end - t) / edge)


def rgb(color: tuple[int, int, int], alpha: int = 255) -> tuple[int, int, int, int]:
    return (*color, int(clamp(alpha / 255.0) * 255))


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require_command(name: str) -> str:
    path = shutil.which(name)
    if path is None:
        raise RuntimeError(f"Required command not found: {name}")
    return path


@lru_cache(maxsize=16)
def load_asset(name: str) -> Image.Image:
    path = ASSET_DIR / name
    if not path.exists():
        raise FileNotFoundError(path)
    return Image.open(path).convert("RGB")


@lru_cache(maxsize=16)
def rotated_asset(name: str, angle: int) -> Image.Image:
    return load_asset(name).rotate(angle, expand=True)


@lru_cache(maxsize=48)
def font(size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(str(FONT_PATH), size=size)


def text_box(
    draw: ImageDraw.ImageDraw,
    xy: tuple[float, float],
    value: str,
    font_obj: ImageFont.FreeTypeFont,
    fill: tuple[int, int, int, int] | tuple[int, int, int],
    anchor: str = "la",
    stroke_width: int = 0,
    stroke_fill: tuple[int, int, int, int] | tuple[int, int, int] | None = None,
) -> tuple[float, float, float, float]:
    kwargs = {
        "font": font_obj,
        "anchor": anchor,
        "stroke_width": stroke_width,
    }
    if stroke_fill is not None:
        kwargs["stroke_fill"] = stroke_fill
    return draw.textbbox(xy, value, **kwargs)


def draw_text(
    draw: ImageDraw.ImageDraw,
    xy: tuple[float, float],
    value: str,
    size: int,
    fill: tuple[int, int, int] | tuple[int, int, int, int] = WARM_WHITE,
    anchor: str = "la",
    shadow: bool = True,
    stroke_width: int = 0,
) -> None:
    if shadow:
        draw.text(
            (xy[0] + 2, xy[1] + 3),
            value,
            font=font(size),
            fill=(0, 0, 0, 125),
            anchor=anchor,
            stroke_width=stroke_width,
        )
    draw.text(
        xy,
        value,
        font=font(size),
        fill=fill,
        anchor=anchor,
        stroke_width=stroke_width,
        stroke_fill=(0, 0, 0, 120) if stroke_width else None,
    )


def fit_cover(
    image: Image.Image,
    size: tuple[int, int],
    center_x: float = 0.5,
    center_y: float = 0.5,
    zoom: float = 1.0,
) -> Image.Image:
    target_w, target_h = size
    scale = max(target_w / image.width, target_h / image.height) * zoom
    resized = image.resize(
        (max(target_w, round(image.width * scale)), max(target_h, round(image.height * scale))),
        Image.Resampling.BICUBIC,
    )
    cx = clamp(center_x) * resized.width
    cy = clamp(center_y) * resized.height
    left = round(cx - target_w / 2)
    top = round(cy - target_h / 2)
    left = max(0, min(left, resized.width - target_w))
    top = max(0, min(top, resized.height - target_h))
    return resized.crop((left, top, left + target_w, top + target_h))


def fit_contain(image: Image.Image, size: tuple[int, int]) -> Image.Image:
    ratio = min(size[0] / image.width, size[1] / image.height)
    resized = image.resize(
        (max(1, round(image.width * ratio)), max(1, round(image.height * ratio))),
        Image.Resampling.BICUBIC,
    )
    canvas = Image.new("RGB", size, CHARCOAL)
    canvas.paste(resized, ((size[0] - resized.width) // 2, (size[1] - resized.height) // 2))
    return canvas


@lru_cache(maxsize=12)
def blurred_background(name: str) -> Image.Image:
    image = fit_cover(load_asset(name), VIDEO_SIZE, zoom=1.16)
    image = image.filter(ImageFilter.GaussianBlur(30))
    overlay = Image.new("RGBA", VIDEO_SIZE, (8, 14, 15, 107))
    image = Image.alpha_composite(image.convert("RGBA"), overlay).convert("RGB")
    return image


@lru_cache(maxsize=16)
def background_template(name: str | None, tint: tuple[int, int, int], tint_alpha: int) -> Image.Image:
    if name:
        bg = blurred_background(name).copy().convert("RGBA")
    else:
        top = np.array(tint, dtype=np.float32)
        bottom = np.array(CHARCOAL_2, dtype=np.float32)
        ramp = np.linspace(0.0, 1.0, HEIGHT, dtype=np.float32)[:, None, None]
        base = np.broadcast_to(top, (HEIGHT, WIDTH, 3)) * (1.0 - ramp) + bottom * ramp
        bg = Image.fromarray(np.clip(base, 0, 255).astype(np.uint8), "RGB").convert("RGBA")

    shade = Image.new("RGBA", VIDEO_SIZE, (*tint, tint_alpha))
    bg = Image.alpha_composite(bg, shade)
    vignette = vignette_mask()
    dark = Image.new("RGBA", VIDEO_SIZE, (0, 0, 0, 150))
    return Image.composite(bg, Image.alpha_composite(bg, dark), vignette).convert("RGB")


@lru_cache(maxsize=1)
def vignette_mask() -> Image.Image:
    vignette = Image.new("L", VIDEO_SIZE, 0)
    vd = ImageDraw.Draw(vignette)
    vd.ellipse((-430, -320, WIDTH + 430, HEIGHT + 320), fill=255)
    return vignette.filter(ImageFilter.GaussianBlur(150))


def make_background(
    name: str | None,
    local_t: float,
    *,
    tint: tuple[int, int, int] = CHARCOAL,
    tint_alpha: int = 132,
    grid: bool = True,
) -> Image.Image:
    bg = background_template(name, tint, tint_alpha).convert("RGBA")
    if grid:
        overlay = Image.new("RGBA", VIDEO_SIZE, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        offset = int(local_t * 18) % 64
        for x in range(-offset, WIDTH + 64, 64):
            draw.line((x, 0, x, HEIGHT), fill=(117, 153, 144, 22), width=1)
        for y in range(0, HEIGHT, 64):
            draw.line((0, y, WIDTH, y), fill=(117, 153, 144, 18), width=1)
        bg = Image.alpha_composite(bg, overlay)
    return bg.convert("RGB")


def rounded_panel(
    size: tuple[int, int],
    radius: int = 22,
    fill: tuple[int, int, int, int] = (21, 28, 28, 220),
    outline: tuple[int, int, int, int] = (115, 142, 135, 80),
    width: int = 2,
) -> Image.Image:
    panel = Image.new("RGBA", size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(panel)
    draw.rounded_rectangle((1, 1, size[0] - 2, size[1] - 2), radius, fill=fill, outline=outline, width=width)
    return panel


def paste_with_shadow(
    canvas: Image.Image,
    image: Image.Image,
    xy: tuple[int, int],
    radius: int = 20,
    shadow_alpha: int = 120,
) -> None:
    x, y = xy
    mask = Image.new("L", image.size, 0)
    ImageDraw.Draw(mask).rounded_rectangle((0, 0, image.width, image.height), radius, fill=255)
    shadow_draw = ImageDraw.Draw(canvas, "RGBA")
    shadow_draw.rounded_rectangle(
        (x + 16, y + 22, x + image.width + 16, y + image.height + 22),
        radius,
        fill=(0, 0, 0, shadow_alpha),
    )
    rgba = image.convert("RGBA")
    clipped = Image.new("RGBA", image.size, (0, 0, 0, 0))
    clipped.paste(rgba, (0, 0), mask)
    canvas.alpha_composite(clipped, (x, y))


def label_chip(
    draw: ImageDraw.ImageDraw,
    xy: tuple[float, float],
    value: str,
    *,
    accent: tuple[int, int, int] = GREEN,
    size: int = 34,
    anchor: str = "la",
) -> None:
    x, y = xy
    box = text_box(draw, (0, 0), value, font(size), WARM_WHITE)
    width = int(box[2] - box[0]) + 42
    height = int(box[3] - box[1]) + 26
    if anchor[0] == "r":
        x -= width
    elif anchor[0] == "m":
        x -= width / 2
    draw.rounded_rectangle(
        (x, y, x + width, y + height),
        radius=12,
        fill=(16, 22, 22, 220),
        outline=(*accent, 175),
        width=2,
    )
    draw.rounded_rectangle((x, y, x + 7, y + height), radius=4, fill=accent)
    draw_text(draw, (x + 24, y + 11), value, size, WARM_WHITE, shadow=False)


def callout(
    draw: ImageDraw.ImageDraw,
    start: tuple[int, int],
    end: tuple[int, int],
    value: str,
    *,
    accent: tuple[int, int, int] = GREEN,
    right_side: bool = True,
) -> None:
    mid = (end[0] + (170 if right_side else -170), end[1])
    draw.line((start, mid, end), fill=(*accent, 235), width=4, joint="curve")
    draw.ellipse((start[0] - 9, start[1] - 9, start[0] + 9, start[1] + 9), outline=accent, width=5)
    label_chip(draw, (end[0] + 18 if right_side else end[0] - 18, end[1] - 24), value, accent=accent, size=31, anchor="la" if right_side else "ra")


def draw_progress(draw: ImageDraw.ImageDraw, t: float, active: int) -> None:
    x0, y0 = 92, 1002
    total = 9
    for idx in range(total):
        x = x0 + idx * 90
        selected = idx == active
        if selected:
            draw.rounded_rectangle((x, y0 - 3, x + 60, y0 + 3), 3, fill=GREEN_2)
        else:
            draw.ellipse((x + 25, y0 - 3, x + 31, y0 + 3), fill=(122, 142, 137, 160))
    draw_text(draw, (x0, 1018), "MODELING REVIEW   01", 18, MUTED, shadow=False)


def label_and_composite(
    canvas: Image.Image,
    image: Image.Image,
    xy: tuple[int, int],
    draw: ImageDraw.ImageDraw,
    local_t: float,
) -> None:
    paste_with_shadow(canvas, image, xy)
    draw.rounded_rectangle(
        (xy[0], xy[1], xy[0] + image.width, xy[1] + image.height),
        20,
        outline=(194, 221, 212, 80),
        width=2,
    )


def scene_intro(local_t: float) -> Image.Image:
    background = make_background(
        "formal_vehicle_product_preview.png",
        local_t,
        tint=(9, 17, 17),
        tint_alpha=111,
    )
    canvas = background.convert("RGBA")
    draw = ImageDraw.Draw(canvas, "RGBA")
    t = ease_out(local_t / 1.15)
    hero = fit_cover(load_asset("formal_vehicle_product_preview.png"), (1570, 682), zoom=1.02 + local_t * 0.002)
    xy = (int(lerp(215, 175, t)), int(lerp(242, 206, t)))
    paste_with_shadow(canvas, hero, xy, radius=22, shadow_alpha=170)
    draw.rounded_rectangle((xy[0], xy[1], xy[0] + hero.width, xy[1] + hero.height), 22, outline=(213, 238, 228, 95), width=2)
    overlay = Image.new("RGBA", VIDEO_SIZE, (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)
    od.rectangle((0, 0, WIDTH, 212), fill=(5, 11, 12, 185))
    od.rectangle((0, 900, WIDTH, HEIGHT), fill=(5, 11, 12, 175))
    canvas = Image.alpha_composite(canvas, overlay)
    draw = ImageDraw.Draw(canvas, "RGBA")
    title_t = ease((local_t - 0.18) / 0.8)
    draw_text(draw, (104, 62 - (1 - title_t) * 20), "TZcup 智能环卫无人清扫车", 54, WARM_WHITE)
    draw_text(draw, (108, 133 - (1 - title_t) * 14), "国产系统 · 模块化平台 · 多传感协同", 28, (181, 222, 205), shadow=False)
    draw_text(draw, (1820, 55), "整车建模成果", 30, GREEN_2, anchor="ra", shadow=False)
    draw_progress(draw, local_t, 0)
    return canvas.convert("RGB")


def scene_multi_angle(local_t: float) -> Image.Image:
    front = load_asset("formal_front_left.png")
    rear = load_asset("formal_rear_right.png")
    phase = local_t / 6.0
    cross = ease((phase - 0.42) / 0.16)
    move = ease_in_out(phase)
    background_name = "formal_front_left.png" if cross < 0.5 else "formal_rear_right.png"
    bg = make_background(background_name, local_t, tint=(14, 20, 20), tint_alpha=123).convert("RGBA")
    canvas = bg
    draw = ImageDraw.Draw(canvas, "RGBA")
    hero_size = (1390, 869)
    front_img = fit_cover(front, hero_size, center_x=0.48, center_y=0.54, zoom=1.1 - move * 0.04)
    rear_img = fit_cover(rear, hero_size, center_x=0.53, center_y=0.51, zoom=1.08 + move * 0.03)
    hero_img = Image.blend(front_img, rear_img, cross)
    xy = (int(430 - move * 80), 178)
    paste_with_shadow(canvas, hero_img, xy, radius=22, shadow_alpha=190)
    draw.rounded_rectangle((xy[0], xy[1], xy[0] + hero_img.width, xy[1] + hero_img.height), 22, outline=(199, 224, 217, 105), width=2)
    draw_text(draw, (94, 78), "整车多角度建模", 51, WARM_WHITE)
    draw_text(draw, (98, 147), "车身比例、舱体关系与作业平台连续校核", 28, (184, 219, 205), shadow=False)
    draw_text(
        draw,
        (1832, 78),
        "前左 3/4 视图" if cross < 0.5 else "后右 3/4 视图",
        29,
        GREEN_2 if cross < 0.5 else CYAN,
        anchor="ra",
        shadow=False,
    )
    draw_progress(draw, local_t, 1)
    return canvas.convert("RGB")


def scene_chassis(local_t: float) -> Image.Image:
    source = load_asset("formal_front_left.png")
    t = ease_out(local_t / 0.9)
    bg = make_background("formal_front_left.png", local_t, tint=(8, 15, 15), tint_alpha=154).convert("RGBA")
    canvas = bg
    draw = ImageDraw.Draw(canvas, "RGBA")
    crop = fit_cover(source, (1500, 587), center_x=0.49 + 0.025 * math.sin(local_t * 0.8), center_y=0.73, zoom=1.45)
    xy = (int(lerp(350, 290, t)), 372)
    paste_with_shadow(canvas, crop, xy, radius=18, shadow_alpha=220)
    draw.rounded_rectangle((xy[0], xy[1], xy[0] + crop.width, xy[1] + crop.height), 18, outline=(*GREEN, 100), width=2)
    draw_text(draw, (94, 70), "底盘与车身", 50, WARM_WHITE)
    draw_text(draw, (98, 138), "面向复杂园区路面的模块化承载平台", 29, (181, 216, 203), shadow=False)

    pulse = 0.55 + 0.45 * math.sin(local_t * math.pi)
    baseline = 867
    draw.line((438, baseline, 1640, baseline + 7), fill=(*GREEN_2, int(90 + 75 * pulse)), width=5)
    for x in (584, 1182):
        draw.line((x, baseline - 20, x, baseline + 20), fill=(*GREEN, 175), width=3)
    label_chip(draw, (119, 535), "六轮全向底盘", accent=GREEN)
    label_chip(draw, (119, 614), "独立驱动与悬挂", accent=CYAN)
    label_chip(draw, (119, 693), "承载式车身与模块化舱体", accent=AMBER, size=30)
    draw_progress(draw, local_t, 2)
    return canvas.convert("RGB")


def scene_cleaning(local_t: float) -> Image.Image:
    source = rotated_asset("formal_top_cleaning.png", -90)
    bg = make_background("formal_top_cleaning.png", local_t, tint=(9, 15, 15), tint_alpha=158).convert("RGBA")
    canvas = bg
    draw = ImageDraw.Draw(canvas, "RGBA")
    t = ease_out(local_t / 1.0)
    image = fit_cover(source, (1360, 780), center_x=0.5, center_y=0.51, zoom=1.05 + 0.018 * math.sin(local_t * 0.7))
    xy = (int(lerp(550, 470, t)), 205)
    paste_with_shadow(canvas, image, xy, radius=18, shadow_alpha=190)
    draw.rounded_rectangle((xy[0], xy[1], xy[0] + image.width, xy[1] + image.height), 18, outline=(203, 226, 219, 100), width=2)
    draw_text(draw, (92, 69), "清扫刷盘与吸口", 50, WARM_WHITE)
    draw_text(draw, (96, 137), "侧刷聚拢 · 中央滚刷 · 吸口随动贴合", 29, (178, 215, 201), shadow=False)

    scan = int((math.sin(local_t * 1.5) + 1) * 0.5 * image.width)
    draw.line((xy[0] + scan, xy[1], xy[0] + scan, xy[1] + image.height), fill=(*GREEN_2, 120), width=3)
    callout(draw, (1518, 355), (342, 456), "双 150 mm 侧刷盘", accent=GREEN, right_side=False)
    callout(draw, (1320, 612), (354, 624), "中央滚刷", accent=CYAN, right_side=False)
    callout(draw, (1534, 780), (336, 790), "吸口与随动贴合", accent=AMBER, right_side=False)
    label_chip(draw, (960, 921), "600 mm 连续清扫覆盖", accent=GREEN_2, size=32, anchor="ma")
    draw_progress(draw, local_t, 3)
    return canvas.convert("RGB")


def scene_arm(local_t: float) -> Image.Image:
    source = load_asset("formal_rear_right.png")
    bg = make_background("formal_rear_right.png", local_t, tint=(10, 16, 17), tint_alpha=145).convert("RGBA")
    canvas = bg
    draw = ImageDraw.Draw(canvas, "RGBA")
    t = ease_in_out(local_t / 1.2)
    crop = fit_cover(source, (1085, 869), center_x=0.80 - 0.025 * t, center_y=0.48, zoom=1.50 + 0.03 * math.sin(local_t * 0.8))
    xy = (int(lerp(1020, 790, t)), 176)
    paste_with_shadow(canvas, crop, xy, radius=18, shadow_alpha=202)
    draw.rounded_rectangle((xy[0], xy[1], xy[0] + crop.width, xy[1] + crop.height), 18, outline=(*CYAN, 100), width=2)
    draw_text(draw, (90, 72), "机械臂单元", 50, WARM_WHITE)
    draw_text(draw, (94, 140), "面向路边与车载工作位的柔性抓取部署", 29, (178, 216, 217), shadow=False)
    label_chip(draw, (104, 555), "UR5e 六轴机械臂", accent=CYAN, size=35)
    label_chip(draw, (104, 635), "Robotiq 2F-85 自适应夹爪", accent=GREEN, size=33)
    label_chip(draw, (104, 715), "车载抓取与投放单元", accent=AMBER, size=31)
    glow = int(90 + 70 * (0.5 + 0.5 * math.sin(local_t * 1.4)))
    draw.ellipse((1320, 402, 1650, 732), outline=(*CYAN, glow), width=6)
    draw_progress(draw, local_t, 4)
    return canvas.convert("RGB")


def scene_sensors(local_t: float) -> Image.Image:
    tower = load_asset("formal_sensor_tower.png")
    bg = make_background("formal_front_left.png", local_t, tint=(8, 15, 17), tint_alpha=151).convert("RGBA")
    canvas = bg
    draw = ImageDraw.Draw(canvas, "RGBA")
    t = ease_out(local_t / 1.0)
    crop = fit_cover(tower, (1220, 762), center_x=0.56, center_y=0.62, zoom=1.14 - 0.025 * t)
    xy = (int(lerp(550, 525, t)), 234)
    paste_with_shadow(canvas, crop, xy, radius=18, shadow_alpha=180)
    draw.rounded_rectangle((xy[0], xy[1], xy[0] + crop.width, xy[1] + crop.height), 18, outline=(*GREEN, 95), width=2)
    draw_text(draw, (92, 69), "多源感知塔", 50, WARM_WHITE)
    draw_text(draw, (96, 137), "为全域建图、定位与目标识别提供统一观测", 29, (180, 216, 202), shadow=False)

    scan_center = (xy[0] + 665, xy[1] + 264)
    phase = (local_t * 0.55) % 1.0
    for ring in range(3):
        radius = int(70 + 165 * ((phase + ring / 3.0) % 1.0))
        alpha = int(135 * (1 - radius / 520))
        draw.ellipse(
            (scan_center[0] - radius, scan_center[1] - radius, scan_center[0] + radius, scan_center[1] + radius),
            outline=(*GREEN_2, max(0, alpha)),
            width=3,
        )
    angle = local_t * 1.4
    ray = (scan_center[0] + int(370 * math.cos(angle)), scan_center[1] + int(370 * math.sin(angle)))
    draw.line((scan_center, ray), fill=(*GREEN_2, 145), width=3)
    label_chip(draw, (104, 560), "3D LiDAR 360° 扫描", accent=GREEN, size=34)
    label_chip(draw, (104, 640), "RGB-D 深度视觉", accent=CYAN, size=34)
    label_chip(draw, (104, 720), "RTK / IMU 融合定位", accent=AMBER, size=34)
    draw_progress(draw, local_t, 5)
    return canvas.convert("RGB")


def scene_storage(local_t: float) -> Image.Image:
    top = rotated_asset("formal_top_cleaning.png", 90)
    bg = make_background("formal_rear_right.png", local_t, tint=(9, 15, 15), tint_alpha=150).convert("RGBA")
    canvas = bg
    draw = ImageDraw.Draw(canvas, "RGBA")
    t = ease_out(local_t / 1.0)
    image = fit_cover(top, (1240, 780), center_x=0.48, center_y=0.48, zoom=1.06 + 0.018 * math.sin(local_t * 0.9))
    xy = (int(lerp(580, 515, t)), 215)
    paste_with_shadow(canvas, image, xy, radius=18, shadow_alpha=190)
    draw.rounded_rectangle((xy[0], xy[1], xy[0] + image.width, xy[1] + image.height), 18, outline=(207, 228, 221, 100), width=2)
    draw_text(draw, (92, 69), "干湿收纳模块", 50, WARM_WHITE)
    draw_text(draw, (96, 137), "独立分区、可视化容量与协同回收", 29, (179, 216, 202), shadow=False)

    green_box = (xy[0] + 735, xy[1] + 192, xy[0] + 1135, xy[1] + 668)
    blue_box = (xy[0] + 278, xy[1] + 392, xy[0] + 675, xy[1] + 735)
    draw.rounded_rectangle(green_box, 18, fill=(18, 92, 57, 70), outline=(*GREEN_2, 215), width=4)
    draw.rounded_rectangle(blue_box, 18, fill=(26, 77, 116, 70), outline=(*CYAN, 215), width=4)
    draw.line((xy[0] + 697, xy[1] + 168, xy[0] + 697, xy[1] + 764), fill=(*WARM_WHITE, 135), width=3)
    label_chip(draw, (1600, 520), "40 L 干尘箱", accent=GREEN, size=34, anchor="ra")
    label_chip(draw, (1600, 602), "8.3 L 独立污水箱", accent=CYAN, size=34, anchor="ra")
    label_chip(draw, (1600, 684), "干湿分区收纳", accent=AMBER, size=31, anchor="ra")
    draw_progress(draw, local_t, 6)
    return canvas.convert("RGB")


def module_tile(
    canvas: Image.Image,
    draw: ImageDraw.ImageDraw,
    *,
    source_name: str,
    title: str,
    subtitle: str,
    box: tuple[int, int, int, int],
    reveal: float,
    accent: tuple[int, int, int],
    crop_center: tuple[float, float],
    zoom: float,
) -> None:
    x0, y0, x1, y1 = box
    reveal = ease(reveal)
    if reveal <= 0.001:
        return
    full_w = x1 - x0
    full_h = y1 - y0
    offset_y = int((1 - reveal) * 80)
    alpha = int(255 * reveal)
    tile = rounded_panel((full_w, full_h), radius=18, fill=(*PANEL, alpha), outline=(*accent, alpha), width=3)
    canvas.alpha_composite(tile, (x0, y0 + offset_y))
    image_size = (full_w - 28, full_h - 108)
    image = fit_cover(load_asset(source_name), image_size, center_x=crop_center[0], center_y=crop_center[1], zoom=zoom)
    image_rgba = image.convert("RGBA")
    fade = Image.new("L", image_size, alpha)
    clipped = Image.new("RGBA", image_size, (0, 0, 0, 0))
    clipped.paste(image_rgba, (0, 0), fade)
    canvas.alpha_composite(clipped, (x0 + 14, y0 + offset_y + 14))
    draw_text(draw, (x0 + 28, y0 + offset_y + full_h - 82), title, 31, WARM_WHITE, shadow=False)
    draw_text(draw, (x0 + 28, y0 + offset_y + full_h - 43), subtitle, 20, (*accent, 255), shadow=False)


def scene_decomposition(local_t: float) -> Image.Image:
    bg = make_background(None, local_t, tint=(12, 19, 20), tint_alpha=230, grid=True).convert("RGBA")
    canvas = bg
    draw = ImageDraw.Draw(canvas, "RGBA")
    draw_text(draw, (960, 64), "模块分解示意", 53, WARM_WHITE, anchor="ma")
    draw_text(draw, (960, 132), "基于整车布局渲染", 28, GREEN_2, anchor="ma", shadow=False)
    progress = clamp((local_t - 0.45) / 5.7)
    module_tile(
        canvas,
        draw,
        source_name="formal_front_left.png",
        title="底盘与车身",
        subtitle="承载平台 · 轮系 · 舱体",
        box=(78, 240, 890, 552),
        reveal=clamp(progress / 0.22),
        accent=GREEN,
        crop_center=(0.48, 0.67),
        zoom=1.38,
    )
    module_tile(
        canvas,
        draw,
        source_name="formal_top_cleaning.png",
        title="清扫系统",
        subtitle="侧刷 · 滚刷 · 吸口",
        box=(1030, 240, 1842, 552),
        reveal=clamp((progress - 0.16) / 0.22),
        accent=CYAN,
        crop_center=(0.51, 0.54),
        zoom=1.10,
    )
    module_tile(
        canvas,
        draw,
        source_name="formal_rear_right.png",
        title="机械臂",
        subtitle="UR5e + 2F-85",
        box=(78, 592, 890, 904),
        reveal=clamp((progress - 0.32) / 0.22),
        accent=AMBER,
        crop_center=(0.76, 0.51),
        zoom=1.44,
    )
    module_tile(
        canvas,
        draw,
        source_name="formal_sensor_tower.png",
        title="感知与收纳",
        subtitle="LiDAR · RGB-D · RTK · 干湿箱",
        box=(1030, 592, 1842, 904),
        reveal=clamp((progress - 0.48) / 0.22),
        accent=GREEN_2,
        crop_center=(0.52, 0.61),
        zoom=1.12,
    )
    note_alpha = int(190 * ease((local_t - 3.2) / 0.9))
    note = "模块分解示意 · 基于现有整车布局渲染"
    box = text_box(draw, (0, 0), note, font(25), WARM_WHITE)
    width = int(box[2] - box[0]) + 46
    x = (WIDTH - width) // 2
    draw.rounded_rectangle((x, 923, x + width, 973), 12, fill=(15, 21, 21, note_alpha), outline=(*GREEN_2, note_alpha))
    draw_text(draw, (WIDTH // 2, 933), note, 25, (204, 225, 217, note_alpha), anchor="ma", shadow=False)
    draw_progress(draw, local_t, 7)
    return canvas.convert("RGB")


def scene_final(local_t: float) -> Image.Image:
    bg = make_background("formal_vehicle_product_preview.png", local_t, tint=(8, 15, 15), tint_alpha=132).convert("RGBA")
    canvas = bg
    draw = ImageDraw.Draw(canvas, "RGBA")
    t = ease_in_out(local_t / 1.1)
    hero = fit_cover(load_asset("formal_vehicle_product_preview.png"), (1500, 650), zoom=1.02 + 0.003 * local_t)
    xy = (int(lerp(440, 210, t)), 150)
    paste_with_shadow(canvas, hero, xy, radius=20, shadow_alpha=190)
    draw.rounded_rectangle((xy[0], xy[1], xy[0] + hero.width, xy[1] + hero.height), 20, outline=(211, 236, 227, 105), width=2)
    draw_text(draw, (90, 58), "模块集成 · 整车协同", 50, WARM_WHITE)
    draw_text(draw, (94, 126), "统一底盘、清扫、抓取、感知与收纳平台", 28, (181, 216, 202), shadow=False)

    params = [
        ("600 mm", "连续清扫覆盖", GREEN),
        ("40 L + 8.3 L", "干尘箱 + 污水箱", CYAN),
        ("UR5e + 2F-85", "车载抓取单元", AMBER),
        ("3D LiDAR · RGB-D · RTK / IMU", "多源融合感知", GREEN_2),
    ]
    for idx, (value, subtitle, accent) in enumerate(params):
        x = 92 if idx % 2 == 0 else 958
        y = 816 if idx < 2 else 904
        width = 850
        draw.rounded_rectangle((x, y, x + width, y + 68), 12, fill=(17, 23, 23, 220), outline=(*accent, 160), width=2)
        draw_text(draw, (x + 18, y + 7), value, 29, accent, shadow=False)
        draw_text(draw, (x + 18, y + 41), subtitle, 18, MUTED, shadow=False)
    draw_progress(draw, local_t, 8)
    return canvas.convert("RGB")


SCENES: list[tuple[float, float, Callable[[float], Image.Image]]] = [
    (0.0, 6.0, scene_intro),
    (6.0, 12.0, scene_multi_angle),
    (12.0, 18.0, scene_chassis),
    (18.0, 24.0, scene_cleaning),
    (24.0, 31.5, scene_arm),
    (31.5, 38.0, scene_sensors),
    (38.0, 45.0, scene_storage),
    (45.0, 53.5, scene_decomposition),
    (53.5, 60.0, scene_final),
]


def render_scene_for_time(t: float) -> Image.Image:
    t = clamp(t, 0.0, DURATION_SECONDS - 1e-6)
    for idx, (start, end, renderer) in enumerate(SCENES):
        if start <= t < end or idx == len(SCENES) - 1:
            local_t = t - start
            image = renderer(local_t)
            transition = 0.42
            if idx < len(SCENES) - 1 and local_t > (end - start) - transition:
                next_renderer = SCENES[idx + 1][2]
                alpha = ease((local_t - ((end - start) - transition)) / transition)
                next_image = next_renderer(0.0)
                image = Image.blend(image, next_image, alpha)
            return image
    raise RuntimeError("No scene rendered")


def assert_visible_text_safe() -> None:
    shot_list = json.loads(SHOT_LIST_PATH.read_text(encoding="utf-8"))
    visible = "\n".join(
        [shot_list["title"]]
        + [
            label
            for shot in shot_list["shots"]
            for label in shot.get("labels", [])
        ]
    )
    lowered = visible.lower()
    for banned in BANNED_VISIBLE_WORDS:
        if banned.lower() in lowered:
            raise RuntimeError(f"Banned visible wording found in shot list: {banned}")


def verify_font_coverage() -> None:
    shot_list = json.loads(SHOT_LIST_PATH.read_text(encoding="utf-8"))
    visible = "".join(
        [shot_list["title"]]
        + [
            label
            for shot in shot_list["shots"]
            for label in shot.get("labels", [])
        ]
    )
    try:
        from fontTools.ttLib import TTFont
    except ImportError:
        return
    cmap = TTFont(str(FONT_PATH)).getBestCmap()
    missing = sorted({char for char in visible if char.strip() and ord(char) not in cmap})
    if missing:
        raise RuntimeError(f"Font lacks visible glyphs: {missing}")


def encode_video(output_path: Path) -> None:
    ffmpeg = require_command("ffmpeg")
    SEGMENT_DIR.mkdir(parents=True, exist_ok=True)
    command = [
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s:v",
        f"{WIDTH}x{HEIGHT}",
        "-r",
        str(FPS),
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "medium",
        "-crf",
        "18",
        "-pix_fmt",
        "yuv420p",
        "-profile:v",
        "high",
        "-level",
        "4.1",
        "-movflags",
        "+faststart",
        "-frames:v",
        str(FRAME_COUNT),
        "-r",
        str(FPS),
        "-y",
        str(output_path),
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    assert process.stdin is not None
    try:
        for index in range(FRAME_COUNT):
            t = index / FPS
            frame = render_scene_for_time(t)
            if index < int(0.45 * FPS):
                black = Image.new("RGB", VIDEO_SIZE, (0, 0, 0))
                frame = Image.blend(black, frame, ease(index / (0.45 * FPS)))
            if index >= FRAME_COUNT - int(0.55 * FPS):
                alpha = ease((FRAME_COUNT - index) / (0.55 * FPS))
                black = Image.new("RGB", VIDEO_SIZE, (0, 0, 0))
                frame = Image.blend(black, frame, alpha)
            process.stdin.write(frame.tobytes())
            if index % FPS == 0:
                print(f"rendered {index / FPS:05.1f}s / {DURATION_SECONDS:.1f}s", flush=True)
    except BrokenPipeError as exc:
        stderr = process.stderr.read().decode("utf-8", errors="replace") if process.stderr else ""
        raise RuntimeError(f"ffmpeg closed early: {stderr}") from exc
    finally:
        if process.stdin:
            process.stdin.close()
    return_code = process.wait()
    stderr = process.stderr.read().decode("utf-8", errors="replace") if process.stderr else ""
    if return_code != 0:
        raise RuntimeError(f"ffmpeg encode failed ({return_code}): {stderr}")


def run_ffprobe(video_path: Path, output_path: Path) -> dict:
    ffprobe = require_command("ffprobe")
    command = [
        ffprobe,
        "-v",
        "error",
        "-count_frames",
        "-show_streams",
        "-show_format",
        "-of",
        "json",
        str(video_path),
    ]
    result = subprocess.run(command, check=True, capture_output=True, text=True, encoding="utf-8")
    data = json.loads(result.stdout)
    output_path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return data


def decode_full(video_path: Path, log_path: Path) -> None:
    ffmpeg = require_command("ffmpeg")
    command = [
        ffmpeg,
        "-v",
        "error",
        "-xerror",
        "-i",
        str(video_path),
        "-map",
        "0:v:0",
        "-f",
        "null",
        "NUL",
    ]
    result = subprocess.run(command, check=False, capture_output=True, text=True, encoding="utf-8")
    log_path.write_text(result.stderr, encoding="utf-8")
    if result.returncode != 0:
        raise RuntimeError(f"Full decode failed: {result.stderr}")


def extract_keyframes(video_path: Path) -> list[dict]:
    ffmpeg = require_command("ffmpeg")
    KEYFRAME_DIR.mkdir(parents=True, exist_ok=True)
    keyframes: list[dict] = []
    moments = [0.5, 5.5, 9.0, 15.0, 21.0, 27.5, 34.5, 41.5, 49.0, 56.5, 59.5]
    for idx, moment in enumerate(moments, start=1):
        output = KEYFRAME_DIR / f"keyframe_{idx:02d}_{moment:04.1f}s.jpg"
        command = [
            ffmpeg,
            "-v",
            "error",
            "-ss",
            f"{moment:.3f}",
            "-i",
            str(video_path),
            "-frames:v",
            "1",
            "-q:v",
            "2",
            "-y",
            str(output),
        ]
        subprocess.run(command, check=True, capture_output=True)
        image = Image.open(output).convert("RGB")
        stat = ImageStat.Stat(image)
        stddev = float(sum(stat.stddev) / len(stat.stddev))
        if stddev < 5.0:
            raise RuntimeError(f"Keyframe appears blank: {output} stddev={stddev:.3f}")
        keyframes.append(
            {
                "timestamp_seconds": moment,
                "path": str(output.relative_to(ROOT)).replace("\\", "/"),
                "sha256": sha256_file(output),
                "mean_stddev": stddev,
            }
        )
    return keyframes


def validate_stream(probe: dict) -> dict:
    streams = probe.get("streams", [])
    if not streams:
        raise RuntimeError("No streams found")
    stream = streams[0]
    format_data = probe.get("format", {})
    duration = float(format_data.get("duration", 0.0))
    frame_count = int(stream.get("nb_read_frames") or 0)
    checks = {
        "codec_h264": stream.get("codec_name") == "h264",
        "width_1920": int(stream.get("width", 0)) == WIDTH,
        "height_1080": int(stream.get("height", 0)) == HEIGHT,
        "pixel_format_yuv420p": stream.get("pix_fmt") == "yuv420p",
        "fps_30": abs(float(stream.get("avg_frame_rate", "0/1").split("/")[0]) / max(1, int(stream.get("avg_frame_rate", "0/1").split("/")[1])) - FPS) < 0.001,
        "duration_exact": abs(duration - DURATION_SECONDS) <= 0.001,
        "frame_count_exact": frame_count == FRAME_COUNT,
        "no_audio": len([s for s in streams if s.get("codec_type") == "audio"]) == 0,
    }
    if not all(checks.values()):
        raise RuntimeError(f"Stream validation failed: {checks}")
    return {
        "checks": checks,
        "duration_seconds": duration,
        "frame_count": frame_count,
        "size_bytes": int(format_data.get("size", 0)),
        "bit_rate": int(format_data.get("bit_rate", 0)),
    }


def populate_shot_hashes() -> None:
    data = json.loads(SHOT_LIST_PATH.read_text(encoding="utf-8"))
    for source in data["source_assets"]:
        path = ROOT / source["path"]
        source["sha256"] = sha256_file(path)
    SHOT_LIST_PATH.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=OUTPUT_PATH)
    parser.add_argument("--skip-render", action="store_true")
    args = parser.parse_args()

    if not FONT_PATH.exists():
        raise FileNotFoundError(FONT_PATH)
    assert_visible_text_safe()
    verify_font_coverage()

    output_path = args.output.resolve()
    if not args.skip_render:
        encode_video(output_path)

    populate_shot_hashes()
    probe_path = EVIDENCE_DIR / "ffprobe_01_modeling.json"
    decode_path = EVIDENCE_DIR / "full_decode.log"
    verification_path = EVIDENCE_DIR / "verification.json"
    manifest_path = EVIDENCE_DIR / "SHA256SUMS.txt"
    probe = run_ffprobe(output_path, probe_path)
    decode_full(output_path, decode_path)
    stream_validation = validate_stream(probe)
    keyframes = extract_keyframes(output_path)

    git_head = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
    ).stdout.strip()
    verification = {
        "status": "PASS",
        "segment": "01_modeling",
        "duration_seconds": DURATION_SECONDS,
        "frame_rate": FPS,
        "resolution": f"{WIDTH}x{HEIGHT}",
        "codec": "H.264",
        "font": str(FONT_PATH),
        "font_sha256": sha256_file(FONT_PATH),
        "source_head": git_head,
        "stream_validation": stream_validation,
        "full_decode": "PASS",
        "keyframes": keyframes,
        "video_sha256": sha256_file(output_path),
        "shot_list_sha256": sha256_file(SHOT_LIST_PATH),
    }
    verification_path.write_text(json.dumps(verification, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    manifest_entries = [output_path, SHOT_LIST_PATH, probe_path, decode_path, verification_path]
    manifest_entries.extend(ROOT / frame["path"] for frame in keyframes)
    lines = [f"{sha256_file(path)}  {path.relative_to(ROOT).as_posix()}" for path in manifest_entries]
    manifest_path.write_text("\n".join(lines) + "\n", encoding="ascii")

    print(json.dumps(verification, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
