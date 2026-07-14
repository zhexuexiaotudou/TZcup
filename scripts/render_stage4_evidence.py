#!/usr/bin/env python3
"""Render headless review figures from Stage 3/4 machine-readable evidence."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image


def render_coverage(stage4_dir: Path) -> None:
    path_data = json.loads(
        (stage4_dir / "coverage_path.json").read_text(encoding="utf-8")
    )
    metrics = json.loads(
        (stage4_dir / "coverage_metrics.json").read_text(encoding="utf-8")
    )
    nav_path = path_data["nav_path"]
    start = metrics["nav2_handoff"]["handoff_start_pose_index"]
    count = metrics["nav2_handoff"]["handoff_path_pose_count"]

    figure, axis = plt.subplots(figsize=(12, 6), constrained_layout=True)
    x, y = zip(*nav_path)
    axis.plot(x, y, color="#94a3b8", linewidth=1.0, label="Full Dubins path")
    for index, (swath_start, swath_end) in enumerate(path_data["swaths"]):
        label = "Reconstructed swaths" if index == 0 else None
        axis.plot(
            [swath_start[0], swath_end[0]],
            [swath_start[1], swath_end[1]],
            color="#0f766e",
            linewidth=3.0,
            alpha=0.75,
            label=label,
        )
    execution = nav_path[start : start + count]
    ex, ey = zip(*execution)
    axis.plot(ex, ey, color="#dc2626", linewidth=3.0, label="Nav2 execution window")
    axis.scatter([ex[0]], [ey[0]], color="#111827", s=45, zorder=5, label="Handoff start")
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("map x (m)")
    axis.set_ylabel("map y (m)")
    axis.set_title(
        "Stage 4 Coverage Evidence | 97.5% planned coverage | "
        "12 swaths / 11 turns / 2140 poses"
    )
    axis.grid(True, color="#e2e8f0", linewidth=0.6)
    axis.legend(loc="upper center", ncol=4, frameon=False)
    figure.savefig(stage4_dir / "coverage_plan.png", dpi=180)
    plt.close(figure)


def render_slam(stage3_dir: Path, stage4_dir: Path) -> None:
    image = Image.open(stage3_dir / "slam_map.pgm")
    figure, axis = plt.subplots(figsize=(10, 5), constrained_layout=True)
    axis.imshow(image, cmap="gray", origin="lower")
    axis.set_title("Stage 3 SLAM map evidence | 194 x 64 cells | 0.05 m/cell")
    axis.set_xlabel("map cell x")
    axis.set_ylabel("map cell y")
    figure.savefig(stage4_dir / "slam_map.png", dpi=180)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("stage3_dir", type=Path)
    parser.add_argument("stage4_dir", type=Path)
    args = parser.parse_args()
    render_coverage(args.stage4_dir)
    render_slam(args.stage3_dir, args.stage4_dir)


if __name__ == "__main__":
    main()
