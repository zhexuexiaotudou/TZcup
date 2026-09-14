#!/usr/bin/env python3
"""Render a success-only, data-driven localization/tracking video segment."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
from typing import Any, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
import yaml


DEFAULT_MCAP_SHA256 = (
    "8e64c0b7dd6a29247ad0d2f61ac719ba7f8c4f6b1957edf8bffc88aa38308047"
)
EXPECTED_CANDIDATE_METRICS = {
    "rmse_m": 0.02919189697400822,
    "p95_m": 0.040285014375691416,
    "max_m": 0.04756582158794771,
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_rows(path: Path) -> list[dict[str, float]]:
    with path.open(encoding="utf-8", newline="") as stream:
        rows = []
        for raw in csv.DictReader(stream):
            row = {
                key: float(value)
                for key, value in raw.items()
                if key != "motion_class"
            }
            row["sim_s"] = float(raw["sim_s"])
            rows.append(row)
    rows.sort(key=lambda row: row["sim_s"])
    if len(rows) < 100:
        raise ValueError("candidate CSV has fewer than 100 samples")
    return rows


def load_route_events(path: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def verify_inputs(
    *,
    mcap: Path | None,
    expected_mcap_sha256: str,
    focus_result: Path,
    candidate_csv: Path,
    receipt: Path,
) -> dict[str, str]:
    hashes = {
        "focus_result": sha256_file(focus_result),
        "candidate_csv": sha256_file(candidate_csv),
        "receipt": sha256_file(receipt),
    }
    focus = json.loads(focus_result.read_text(encoding="utf-8"))
    accepted_mcap_hashes = set(focus.get("files", {}).values())
    if expected_mcap_sha256 not in accepted_mcap_hashes:
        raise ValueError("expected MCAP hash is not bound by the focus result")
    if mcap is not None:
        actual = sha256_file(mcap)
        if actual != expected_mcap_sha256:
            raise ValueError(
                f"MCAP hash mismatch: {actual} != {expected_mcap_sha256}"
            )
        hashes["raw_mcap"] = actual
    receipt_data = json.loads(receipt.read_text(encoding="utf-8"))
    if receipt_data.get("status") != "OFFLINE_CANDIDATE_PASS":
        raise ValueError("the causal candidate receipt is not successful")
    if receipt_data.get("metrics", {}).get("candidate", {}).get(
        "rmse_m"
    ) != EXPECTED_CANDIDATE_METRICS["rmse_m"]:
        raise ValueError("receipt does not contain the expected candidate metrics")
    if receipt_data.get("evidence", {}).get(
        "candidate_csv_sha256"
    ) != hashes["candidate_csv"]:
        raise ValueError("receipt candidate CSV hash does not match input")
    return hashes


def interpolate_rows(
    rows: Sequence[dict[str, float]], key: str, sim_s: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    source_t = np.array([row["sim_s"] for row in rows], dtype=float)
    source_x = np.array([row[f"{key}_x_m"] for row in rows], dtype=float)
    source_y = np.array([row[f"{key}_y_m"] for row in rows], dtype=float)
    return (
        np.interp(sim_s, source_t, source_x),
        np.interp(sim_s, source_t, source_y),
    )


def route_state(route_events: Sequence[dict[str, Any]], sim_s: float) -> str:
    goals = [
        event
        for event in route_events
        if event.get("kind") in {"goal_sent", "goal_result"}
    ]
    if not goals:
        return "Tracking active"
    completed = [
        event
        for event in goals
        if event.get("kind") == "goal_result"
        and int(event.get("data", {}).get("status", -1)) == 4
        and float(event["sim_s"]) <= sim_s
    ]
    sent = [
        event
        for event in goals
        if event.get("kind") == "goal_sent" and float(event["sim_s"]) <= sim_s
    ]
    if len(completed) >= 2:
        return "2 / 2 goals completed"
    if len(completed) == 1 and len(sent) >= 2:
        return "Goal 2 / 2 in progress"
    if len(completed) == 1:
        return "Goal 1 complete"
    if sent:
        return "Goal 1 / 2 in progress"
    return "Tracking active"


def load_map(map_pgm: Path, map_yaml: Path) -> tuple[np.ndarray, tuple[float, ...]]:
    image = mpimg.imread(map_pgm)
    metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    resolution = float(metadata["resolution"])
    origin = [float(value) for value in metadata["origin"]]
    height, width = image.shape[:2]
    extent = (
        origin[0],
        origin[0] + width * resolution,
        origin[1],
        origin[1] + height * resolution,
    )
    return np.flipud(image), extent


def build_figure(
    rows: Sequence[dict[str, float]],
    map_image: np.ndarray,
    map_extent: tuple[float, ...],
) -> tuple[Any, dict[str, Any]]:
    figure = plt.Figure(figsize=(19.2, 10.8), dpi=100, facecolor="#0b1117")
    canvas = FigureCanvasAgg(figure)
    grid = figure.add_gridspec(
        1, 2, width_ratios=(1.45, 1.0), left=0.035, right=0.975,
        bottom=0.08, top=0.83, wspace=0.12
    )
    map_axis = figure.add_subplot(grid[0, 0])
    chart_axis = figure.add_subplot(grid[0, 1])
    figure.text(
        0.035, 0.955, "Localization + Tracking Demonstration",
        color="white", fontsize=30, fontweight="bold", va="top"
    )
    figure.text(
        0.035, 0.905,
        "Sealed navigation run | two goals completed",
        color="#8ea7b5", fontsize=17, va="top"
    )
    figure.text(
        0.965, 0.95,
        "2 / 2 GOALS",
        color="#64e6a5", fontsize=19, fontweight="bold",
        ha="right", va="top"
    )
    figure.text(
        0.965, 0.885,
        "RMSE 29.2 mm   P95 40.3 mm   max 47.6 mm",
        color="white", fontsize=15, ha="right", va="top"
    )

    map_axis.imshow(
        map_image, cmap="gray", extent=map_extent, origin="lower",
        vmin=0.0, vmax=1.0, alpha=0.72
    )
    map_axis.set_facecolor("#0f1921")
    map_axis.grid(color="#29404e", linewidth=0.6, alpha=0.35)
    map_axis.tick_params(colors="#7893a2", labelsize=10)
    for spine in map_axis.spines.values():
        spine.set_color("#29404e")
    map_axis.set_xlabel("map x (m)", color="#8ea7b5", fontsize=12)
    map_axis.set_ylabel("map y (m)", color="#8ea7b5", fontsize=12)

    reference_x, reference_y = interpolate_rows(
        rows, "gt_map", np.array([row["sim_s"] for row in rows])
    )
    estimate_x, estimate_y = interpolate_rows(
        rows, "candidate", np.array([row["sim_s"] for row in rows])
    )
    map_axis.plot(
        reference_x, reference_y, color="#51d7ff", linewidth=5.0,
        alpha=0.34, label="Reference trajectory"
    )
    map_axis.plot(
        estimate_x, estimate_y, color="#64e6a5", linewidth=2.4,
        alpha=0.95, label="Localization trajectory"
    )
    goal_x = [6.0, 0.0]
    goal_y = [0.0, 0.0]
    map_axis.scatter(
        goal_x, goal_y, s=100, marker="s", facecolors="none",
        edgecolors="#ffd166", linewidths=2.0, label="Navigation goals", zorder=6
    )
    for index, (x_value, y_value) in enumerate(zip(goal_x, goal_y), start=1):
        map_axis.text(
            x_value, y_value + 0.22, f"G{index}", color="#ffd166",
            fontsize=12, ha="center", fontweight="bold"
        )
    map_axis.scatter(
        estimate_x[0], estimate_y[0], s=75, color="#ff7b72",
        edgecolor="white", linewidth=0.8, zorder=7
    )
    map_axis.scatter(
        estimate_x[-1], estimate_y[-1], s=75, color="#f2cc60",
        edgecolor="white", linewidth=0.8, zorder=7
    )
    trajectory_line, = map_axis.plot(
        [], [], color="#ffffff", linewidth=2.4, alpha=0.98,
        marker="o", markersize=7, markerfacecolor="#64e6a5",
        markeredgecolor="white", zorder=8
    )
    x_min_value = min(float(np.min(reference_x)), float(np.min(estimate_x)))
    x_max_value = max(float(np.max(reference_x)), float(np.max(estimate_x)))
    y_min_value = min(float(np.min(reference_y)), float(np.min(estimate_y)))
    y_max_value = max(float(np.max(reference_y)), float(np.max(estimate_y)))
    x_margin = max(0.5, 0.12 * (x_max_value - x_min_value))
    y_margin = max(0.2, 0.25 * (y_max_value - y_min_value))
    x_min = x_min_value - x_margin
    x_max = x_max_value + x_margin
    y_min = y_min_value - y_margin
    y_max = y_max_value + y_margin
    map_axis.set_xlim(x_min, x_max)
    map_axis.set_ylim(y_min, y_max)
    map_axis.set_aspect("auto")
    legend = map_axis.legend(
        loc="lower right", frameon=True, facecolor="#15232d",
        edgecolor="#29404e", labelcolor="white", fontsize=10
    )
    legend.get_frame().set_alpha(0.9)

    sim_s = np.array([row["sim_s"] for row in rows], dtype=float)
    errors_mm = np.array(
        [row["candidate_error_m"] * 1000.0 for row in rows], dtype=float
    )
    chart_axis.set_facecolor("#0f1921")
    chart_axis.plot(sim_s, errors_mm, color="#64e6a5", linewidth=2.0)
    chart_axis.fill_between(sim_s, 0.0, errors_mm, color="#64e6a5", alpha=0.13)
    chart_axis.axhline(
        50.0, color="#ffd166", linewidth=1.8, linestyle="--",
        label="50 mm success envelope"
    )
    chart_axis.set_ylim(0.0, 50.0)
    chart_axis.set_xlim(float(sim_s[0]), float(sim_s[-1]))
    chart_axis.set_xlabel("simulation time (s)", color="#8ea7b5", fontsize=12)
    chart_axis.set_ylabel("localization error (mm)", color="#8ea7b5", fontsize=12)
    chart_axis.text(
        0.02, 0.97, "Causal localization error",
        transform=chart_axis.transAxes, color="white", fontsize=18,
        ha="left", va="top"
    )
    chart_axis.grid(color="#29404e", linewidth=0.7, alpha=0.42)
    chart_axis.tick_params(colors="#7893a2", labelsize=10)
    for spine in chart_axis.spines.values():
        spine.set_color("#29404e")
    chart_axis.legend(
        loc="upper left", frameon=True, facecolor="#15232d",
        edgecolor="#29404e", labelcolor="white", fontsize=10
    )
    cursor_line = chart_axis.axvline(
        float(sim_s[0]), color="#ff7b72", linewidth=1.5, alpha=0.9
    )
    cursor_dot, = chart_axis.plot(
        [], [], marker="o", markersize=7, color="#ff7b72", zorder=7
    )
    figure.text(
        0.035, 0.03, "LOCALIZATION", color="#51d7ff",
        fontsize=13, fontweight="bold"
    )
    figure.text(
        0.25, 0.03, "TRACKING", color="#64e6a5",
        fontsize=13, fontweight="bold"
    )
    figure.text(
        0.43, 0.03, "CAUSAL FILTER", color="#ffd166",
        fontsize=13, fontweight="bold"
    )
    state_text = figure.text(
        0.965, 0.06, "Tracking active", color="#d9e6ed",
        fontsize=17, fontweight="bold", ha="right"
    )
    time_text = figure.text(
        0.965, 0.03, "0.0 s", color="#8ea7b5",
        fontsize=12, ha="right"
    )
    artists = {
        "canvas": canvas,
        "figure": figure,
        "trajectory_line": trajectory_line,
        "cursor_line": cursor_line,
        "cursor_dot": cursor_dot,
        "state_text": state_text,
        "time_text": time_text,
        "reference_x": reference_x,
        "reference_y": reference_y,
        "estimate_x": estimate_x,
        "estimate_y": estimate_y,
        "errors_mm": errors_mm,
    }
    return figure, artists


def render_stills(
    *,
    rows: Sequence[dict[str, float]],
    map_image: np.ndarray,
    map_extent: tuple[float, ...],
    output_dir: Path,
    route_events: Sequence[dict[str, Any]],
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    figure, artists = build_figure(rows, map_image, map_extent)
    reference_x = artists["reference_x"]
    reference_y = artists["reference_y"]
    estimate_x = artists["estimate_x"]
    estimate_y = artists["estimate_y"]
    artist_line = artists["trajectory_line"]
    artist_line.set_data(estimate_x, estimate_y)
    artists["cursor_line"].set_xdata([float(rows[-1]["sim_s"])])
    artists["cursor_dot"].set_data(
        [rows[-1]["sim_s"]], [artists["errors_mm"][-1]]
    )
    artists["state_text"].set_text("2 / 2 goals completed")
    artists["time_text"].set_text(f"{rows[-1]['sim_s']:.1f} s")
    trajectory_path = output_dir / "localization_trajectory_success.png"
    figure.savefig(trajectory_path, dpi=100, facecolor=figure.get_facecolor())

    error_figure, error_axis = plt.subplots(figsize=(12, 6), dpi=160)
    error_figure.patch.set_facecolor("#0b1117")
    error_axis.set_facecolor("#0f1921")
    sim_s = [row["sim_s"] for row in rows]
    error_axis.plot(sim_s, artists["errors_mm"], color="#64e6a5", linewidth=2.2)
    error_axis.fill_between(
        sim_s, 0.0, artists["errors_mm"], color="#64e6a5", alpha=0.15
    )
    error_axis.axhline(
        50.0, color="#ffd166", linewidth=2.0, linestyle="--",
        label="50 mm success envelope"
    )
    error_axis.set_ylim(0.0, 50.0)
    error_axis.set_xlim(sim_s[0], sim_s[-1])
    error_axis.set_xlabel("simulation time (s)", color="#8ea7b5")
    error_axis.set_ylabel("localization error (mm)", color="#8ea7b5")
    error_axis.set_title(
        "Localization error remains inside the success envelope",
        color="white", fontsize=16
    )
    error_axis.grid(color="#29404e", alpha=0.45)
    error_axis.tick_params(colors="#7893a2")
    for spine in error_axis.spines.values():
        spine.set_color("#29404e")
    error_axis.legend(
        facecolor="#15232d", edgecolor="#29404e", labelcolor="white"
    )
    error_path = output_dir / "localization_error_success.png"
    error_figure.savefig(
        error_path, dpi=160, bbox_inches="tight", facecolor=error_figure.get_facecolor()
    )
    plt.close(error_figure)

    event_figure, event_axis = plt.subplots(figsize=(12, 6), dpi=160)
    event_figure.patch.set_facecolor("#0b1117")
    event_axis.set_facecolor("#0f1921")
    goal_events = [
        event
        for event in route_events
        if event.get("kind") in {"goal_sent", "goal_result"}
    ]
    for index, event in enumerate(goal_events):
        event_axis.barh(
            index, 1.0,
            color="#64e6a5" if event.get("kind") == "goal_result" else "#51d7ff",
            alpha=0.85
        )
        event_axis.text(
            0.03, index,
            f"{event['kind'].replace('_', ' ')} | sim {float(event['sim_s']):.1f} s",
            color="white", va="center", fontsize=11
        )
    event_axis.set_yticks([])
    event_axis.set_xticks([])
    event_axis.set_xlim(0.0, 1.0)
    event_axis.set_title(
        "Navigation goals completed", color="white", fontsize=16
    )
    for spine in event_axis.spines.values():
        spine.set_visible(False)
    event_path = output_dir / "tracking_goals_success.png"
    event_figure.savefig(
        event_path, dpi=160, bbox_inches="tight", facecolor=event_figure.get_facecolor()
    )
    plt.close(event_figure)
    plt.close(figure)
    return {
        "trajectory": trajectory_path,
        "error": error_path,
        "goals": event_path,
    }


def render_video(
    *,
    rows: Sequence[dict[str, float]],
    map_image: np.ndarray,
    map_extent: tuple[float, ...],
    route_events: Sequence[dict[str, Any]],
    output_path: Path,
    duration_sec: float,
    fps: int,
    ffmpeg: str,
) -> None:
    if output_path.exists():
        output_path.unlink()
    figure, artists = build_figure(rows, map_image, map_extent)
    sim_s = np.array([row["sim_s"] for row in rows], dtype=float)
    start_s = float(sim_s[0])
    end_s = float(sim_s[-1])
    frame_count = int(round(duration_sec * fps))
    command = [
        ffmpeg,
        "-y",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgba",
        "-s",
        "1920x1080",
        "-r",
        str(fps),
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "medium",
        "-crf",
        "24",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output_path),
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    if process.stdin is None:
        raise RuntimeError("ffmpeg stdin was not created")
    canvas = artists["canvas"]
    trajectory_line = artists["trajectory_line"]
    cursor_line = artists["cursor_line"]
    cursor_dot = artists["cursor_dot"]
    state_text = artists["state_text"]
    time_text = artists["time_text"]
    reference_x = artists["reference_x"]
    reference_y = artists["reference_y"]
    estimate_x = artists["estimate_x"]
    estimate_y = artists["estimate_y"]
    errors_mm = artists["errors_mm"]
    hold_frames = min(fps * 3, frame_count // 8)
    moving_frames = frame_count - hold_frames
    try:
        for frame_index in range(frame_count):
            progress = min(1.0, frame_index / max(1, moving_frames - 1))
            current_s = start_s + (end_s - start_s) * progress
            upto = max(1, int(np.searchsorted(sim_s, current_s, side="right")))
            trajectory_line.set_data(estimate_x[:upto], estimate_y[:upto])
            cursor_line.set_xdata([current_s])
            cursor_dot.set_data([current_s], [np.interp(current_s, sim_s, errors_mm)])
            state_text.set_text(route_state(route_events, current_s))
            time_text.set_text(f"{current_s:.1f} s")
            canvas.draw()
            process.stdin.write(np.asarray(canvas.buffer_rgba()).tobytes())
    finally:
        process.stdin.close()
        returncode = process.wait()
    if returncode != 0:
        raise RuntimeError(f"ffmpeg failed with return code {returncode}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mcap", type=Path)
    parser.add_argument("--expected-mcap-sha256", default=DEFAULT_MCAP_SHA256)
    parser.add_argument("--focus-result", required=True, type=Path)
    parser.add_argument("--candidate-csv", required=True, type=Path)
    parser.add_argument("--receipt", required=True, type=Path)
    parser.add_argument("--map-pgm", required=True, type=Path)
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--route-timeline", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--duration-sec", type=float, default=60.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--skip-video", action="store_true")
    parser.add_argument("--ffmpeg", default=shutil.which("ffmpeg") or "ffmpeg")
    args = parser.parse_args()
    if args.output_dir.exists():
        raise SystemExit(f"fresh output directory required: {args.output_dir}")
    if args.duration_sec <= 0.0 or args.fps <= 0:
        raise SystemExit("duration and fps must be positive")
    args.output_dir.mkdir(parents=True)
    input_hashes = verify_inputs(
        mcap=args.mcap,
        expected_mcap_sha256=args.expected_mcap_sha256,
        focus_result=args.focus_result,
        candidate_csv=args.candidate_csv,
        receipt=args.receipt,
    )
    rows = load_rows(args.candidate_csv)
    route_events = load_route_events(args.route_timeline)
    completed_goals = [
        event
        for event in route_events
        if event.get("kind") == "goal_result"
        and int(event.get("data", {}).get("status", -1)) == 4
    ]
    if len(completed_goals) < 2:
        raise ValueError("route timeline does not contain two completed goals")
    map_image, map_extent = load_map(args.map_pgm, args.map_yaml)
    stills = render_stills(
        rows=rows,
        map_image=map_image,
        map_extent=map_extent,
        output_dir=args.output_dir,
        route_events=route_events,
    )
    video_path = args.output_dir / "localization_tracking_success_1080p.mp4"
    if not args.skip_video:
        render_video(
            rows=rows,
            map_image=map_image,
            map_extent=map_extent,
            route_events=route_events,
            output_path=video_path,
            duration_sec=args.duration_sec,
            fps=args.fps,
            ffmpeg=args.ffmpeg,
        )
    outputs = {name: path for name, path in stills.items()}
    if video_path.is_file():
        outputs["video"] = video_path
    manifest = {
        "schema_version": 1,
        "status": "VIDEO_PACK_READY",
        "scope": "success-only localization and tracking segment for a five-minute edit",
        "source": {
            "focus_result_sha256": input_hashes["focus_result"],
            "candidate_csv_sha256": input_hashes["candidate_csv"],
            "receipt_sha256": input_hashes["receipt"],
            "raw_mcap_sha256": input_hashes.get("raw_mcap", args.expected_mcap_sha256),
            "route_timeline_sha256": sha256_file(args.route_timeline),
            "map_pgm_sha256": sha256_file(args.map_pgm),
            "map_yaml_sha256": sha256_file(args.map_yaml),
        },
        "conclusion": {
            "navigation_results": [4, 4],
            "goals_completed": 2,
            "candidate_metrics_mm": {
                "rmse": EXPECTED_CANDIDATE_METRICS["rmse_m"] * 1000.0,
                "p95": EXPECTED_CANDIDATE_METRICS["p95_m"] * 1000.0,
                "max": EXPECTED_CANDIDATE_METRICS["max_m"] * 1000.0,
            },
        },
        "outputs": {
            name: {
                "path": str(path.resolve()),
                "sha256": sha256_file(path),
                "bytes": path.stat().st_size,
            }
            for name, path in outputs.items()
        },
    }
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(manifest["conclusion"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
