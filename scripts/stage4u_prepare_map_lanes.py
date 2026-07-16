#!/usr/bin/env python3
"""Prepare honest M1/M2/M3 localization lanes from Stage4T evidence."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from stage4u_map_calibration import build_calibration  # noqa: E402


def run(command):
    subprocess.run([str(item) for item in command], check=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage4t-review", required=True, type=Path)
    parser.add_argument("--world-sdf", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--posegraph-root", type=Path)
    args = parser.parse_args()

    report = json.loads(
        (args.stage4t_review / "map_geometry_report.json").read_text(encoding="utf-8")
    )
    selected = str(report["selected_map"])
    source_trial = args.stage4t_review / "map_trials" / f"map_{selected}"
    output = args.output_dir
    m1 = output / "M1_slam_raw"
    m2 = output / "M2_slam_refined"
    m3 = output / "M3_surveyed_reference"
    for lane in (m1, m2, m3):
        lane.mkdir(parents=True, exist_ok=True)

    for source, destination in (
        (args.stage4t_review / "selected_map.yaml", m1 / "selected_map.yaml"),
        (args.stage4t_review / "selected_map.pgm", m1 / "selected_map.pgm"),
        (source_trial / "map_geometry.json", m1 / "map_geometry.json"),
        (source_trial / "map_quality.json", m1 / "map_quality.json"),
        (source_trial / "mapping_probe.json", m1 / "mapping_probe.json"),
    ):
        shutil.copy2(source, destination)
    metadata = yaml.safe_load((m1 / "selected_map.yaml").read_text(encoding="utf-8"))
    metadata["image"] = "selected_map.pgm"
    (m1 / "selected_map.yaml").write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
    )
    calibration = build_calibration(
        m1 / "selected_map.yaml", m1 / "map_geometry.json", "M1_slam_raw"
    )
    (m1 / "map_frame_calibration.yaml").write_text(
        yaml.safe_dump(calibration, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    run(
        [
            sys.executable,
            ROOT / "scripts" / "generate_stage4r_masks.py",
            "--map-yaml",
            m1 / "selected_map.yaml",
            "--output-dir",
            m1 / "filters",
        ]
    )

    posegraph_search_root = args.posegraph_root or args.stage4t_review
    posegraphs = sorted(posegraph_search_root.rglob("*.posegraph"))
    data_files = sorted(posegraph_search_root.rglob("*.data"))
    m2_available = bool(posegraphs and data_files)
    if m2_available:
        m2_source = args.posegraph_root
        if m2_source is None:
            m2_source = posegraphs[0].parent
        for source, destination in (
            (m2_source / "slam_map.yaml", m2 / "selected_map.yaml"),
            (m2_source / "slam_map.pgm", m2 / "selected_map.pgm"),
            (m2_source / "map_geometry.json", m2 / "map_geometry.json"),
            (m2_source / "map_quality.json", m2 / "map_quality.json"),
            (m2_source / "mapping_probe.json", m2 / "mapping_probe.json"),
            (posegraphs[0], m2 / "slam_posegraph.posegraph"),
            (data_files[0], m2 / "slam_posegraph.data"),
        ):
            shutil.copy2(source, destination)
        m2_metadata = yaml.safe_load((m2 / "selected_map.yaml").read_text(encoding="utf-8"))
        m2_metadata["image"] = "selected_map.pgm"
        (m2 / "selected_map.yaml").write_text(
            yaml.safe_dump(m2_metadata, sort_keys=False), encoding="utf-8"
        )
        m2_calibration = build_calibration(
            m2 / "selected_map.yaml", m2 / "map_geometry.json", "M2_slam_refined"
        )
        (m2 / "map_frame_calibration.yaml").write_text(
            yaml.safe_dump(m2_calibration, allow_unicode=True, sort_keys=False), encoding="utf-8"
        )
        run(
            [
                sys.executable,
                ROOT / "scripts" / "generate_stage4r_masks.py",
                "--map-yaml",
                m2 / "selected_map.yaml",
                "--output-dir",
                m2 / "filters",
            ]
        )
    m2_report = {
        "schema_version": 1,
        "lane": "M2_slam_refined",
        "available": m2_available,
        "mapping_evidence": True,
        "localization_reference": True,
        "offline_optimization_executed": False,
        "serialized_post_loop_closure_graph": m2_available,
        "map_rendered_from_serialized_graph": False,
        "source_posegraph_files": [str(path) for path in posegraphs + data_files],
        "reason": (
            "Serialized post-loop-closure graph is available for localization mode, but a separate offline optimization/render pass has not yet been executed"
            if m2_available
            else "Stage4T did not serialize a SLAM Toolbox pose graph; a fresh mapping run is required"
        ),
    }
    (m2 / "lane_status.json").write_text(
        json.dumps(m2_report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    run(
        [
            sys.executable,
            ROOT / "scripts" / "stage4u_reference_map.py",
            "--template-map",
            m1 / "selected_map.yaml",
            "--world-sdf",
            args.world_sdf,
            "--output-dir",
            m3,
        ]
    )
    run(
        [
            sys.executable,
            ROOT / "scripts" / "generate_stage4r_masks.py",
            "--map-yaml",
            m3 / "surveyed_reference.yaml",
            "--output-dir",
            m3 / "filters",
        ]
    )
    summary = {
        "schema_version": 1,
        "lanes": {
            "M1_slam_raw": {
                "available": True,
                "mapping_evidence": True,
                "localization_reference": True,
                "map_yaml": "M1_slam_raw/selected_map.yaml",
                "calibration": "M1_slam_raw/map_frame_calibration.yaml",
            },
            "M2_slam_refined": {
                **m2_report,
                "map_yaml": "M2_slam_refined/selected_map.yaml" if m2_available else None,
                "calibration": "M2_slam_refined/map_frame_calibration.yaml" if m2_available else None,
                "posegraph_base": "M2_slam_refined/slam_posegraph" if m2_available else None,
            },
            "M3_surveyed_reference": {
                "available": True,
                "mapping_evidence": False,
                "localization_reference": True,
                "map_yaml": "M3_surveyed_reference/surveyed_reference.yaml",
                "calibration": "M3_surveyed_reference/map_frame_calibration.yaml",
            },
        },
        "first_blocked_lane": (
            "M2_offline_optimization" if m2_available else "M2_slam_refined"
        ),
    }
    (output / "map_lane_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
