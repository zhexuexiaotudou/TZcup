#!/usr/bin/env python3
"""Capture the Gazebo X11 window and reject an all-black 3D viewport."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import time
from pathlib import Path

from PIL import Image, ImageChops, ImageStat


WINDOW_RE = re.compile(
    r'^\s*(0x[0-9a-fA-F]+)\s+"(?P<title>[^"]+)".*?'
    r'(?P<width>\d+)x(?P<height>\d+)\+',
)


def find_window_id(title: str) -> tuple[str, int, int]:
    listing = subprocess.run(
        ["xwininfo", "-root", "-tree"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    candidates: list[tuple[int, str, int, int]] = []
    for line in listing.splitlines():
        match = WINDOW_RE.search(line)
        if not match or title.casefold() not in match.group("title").casefold():
            continue
        width = int(match.group("width"))
        height = int(match.group("height"))
        candidates.append((width * height, match.group(1), width, height))
    if not candidates:
        raise RuntimeError(f"X11 window containing {title!r} was not found")
    _, window_id, width, height = max(candidates)
    return window_id, width, height


def capture_window(window_id: str, xwd_path: Path, png_path: Path) -> None:
    subprocess.run(
        ["xwd", "-silent", "-id", window_id, "-out", str(xwd_path)],
        check=True,
    )
    subprocess.run(
        [
            "ffmpeg",
            "-nostdin",
            "-y",
            "-loglevel",
            "error",
            "-i",
            str(xwd_path),
            "-frames:v",
            "1",
            str(png_path),
        ],
        check=True,
    )


def analyze_viewport(image_path: Path) -> dict[str, object]:
    with Image.open(image_path) as source:
        image = source.convert("RGB")
    width, height = image.size
    # Both the TZcup mission layout and Gazebo's default layout keep the 3D
    # viewport in this central-left region. Excluding the toolbar and right
    # panels prevents healthy Qt chrome from hiding a black render surface.
    crop_box = (
        max(0, round(width * 0.02)),
        max(0, round(height * 0.15)),
        max(1, round(width * 0.55)),
        max(1, round(height * 0.94)),
    )
    viewport = image.crop(crop_box)
    red, green, blue = viewport.split()
    brightest_channel = ImageChops.lighter(ImageChops.lighter(red, green), blue)
    histogram = brightest_channel.histogram()
    near_black = sum(histogram[:13])
    black_ratio = near_black / max(1, viewport.width * viewport.height)
    luminance = viewport.convert("L")
    stats = ImageStat.Stat(luminance)
    mean_luminance = float(stats.mean[0])
    stddev_luminance = float(stats.stddev[0])
    is_black = black_ratio >= 0.90 or mean_luminance <= 5.0
    return {
        "image_size": [width, height],
        "viewport_crop": list(crop_box),
        "near_black_ratio": round(black_ratio, 6),
        "mean_luminance": round(mean_luminance, 3),
        "stddev_luminance": round(stddev_luminance, 3),
        "render_visible": not is_black,
    }


def run_probe(output_dir: Path, title: str, timeout_sec: float) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    xwd_path = output_dir / "gazebo_viewport.xwd"
    png_path = output_dir / "gazebo_viewport.png"
    deadline = time.monotonic() + timeout_sec
    last_error = "probe did not run"
    last_result: dict[str, object] | None = None
    while time.monotonic() < deadline:
        try:
            window_id, window_width, window_height = find_window_id(title)
            capture_window(window_id, xwd_path, png_path)
            last_result = analyze_viewport(png_path)
            last_result.update(
                {
                    "window_id": window_id,
                    "reported_window_size": [window_width, window_height],
                    "capture": str(png_path),
                }
            )
            if last_result["render_visible"]:
                return last_result
            last_error = "3D viewport remained black"
        except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
            last_error = str(exc)
        time.sleep(1.0)
    if last_result is not None:
        last_result["error"] = last_error
        return last_result
    return {"render_visible": False, "error": last_error}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--title", default="Gazebo Sim")
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()
    result = run_probe(args.output_dir, args.title, args.timeout)
    result_path = args.output_dir / "gazebo_viewport_probe.json"
    result_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result.get("render_visible") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
