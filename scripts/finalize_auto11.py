#!/usr/bin/env python3
"""Create compact AUTO-11 evidence from the formal large-map report."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write(path: Path, payload) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--formal-root", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    formal_root = Path(args.formal_root)
    output = Path(args.output)
    report = json.loads(
        (formal_root / "formal_report.json").read_text(encoding="utf-8")
    )
    if not report["auto11_gate_pass"]:
        raise RuntimeError("AUTO-11 formal gate did not pass")
    if output.exists():
        shutil.rmtree(output)
    output.mkdir(parents=True)
    compact = {
        "schema_version": 1,
        "stage": "AUTO-11",
        "attempt_id": report["attempt_id"],
        "implementation_commit": report["implementation_commit"],
        "source_level": report["source_level"],
        "map": {
            key: report["map"][key]
            for key in (
                "resolution",
                "width_cells",
                "height_cells",
                "width_m",
                "height_m",
                "area_m2",
                "pgm_sha256",
                "metadata_sha256",
            )
        },
        "trajectory_count": len(report["localization"]),
        "trajectory_rmse_m": [
            row["rmse_m"] for row in report["localization"]
        ],
        "coverage_mission_count": len(report["coverage_missions"]),
        "scheduled_route_count": len(report["scheduled_routes"]),
        "aggregate": report["aggregate"],
        "checks": report["checks"],
        "auto11_gate_pass": True,
    }
    write(output / "metrics_summary.json", compact)
    write(
        output / "stage_status.json",
        {
            "schema_version": 1,
            "stage": "AUTO-11",
            "status": "PASS",
            "machine_gate_pass": True,
            "blocked": False,
            "blocked_external": False,
            "first_blocking_layer": None,
            "selected_attempt": report["attempt_id"],
            "implementation_commit": report["implementation_commit"],
            "evidence_level": report["source_level"],
            "gazebo_claimed": False,
            "real_vehicle_claimed": False,
        },
    )
    write(
        output / "environment.json",
        {
            "schema_version": 1,
            "host": platform.platform(),
            "python": platform.python_version(),
            "simulation": "truth-separated deterministic large-map motion model",
        },
    )
    raw_files = [
        formal_root / "formal_report.json",
        formal_root / "map" / "large_map.pgm",
        formal_root / "map" / "large_map.json",
    ]
    write(
        output / "raw_metric_index.json",
        {
            "schema_version": 1,
            "external_raw_files": [
                {
                    "path": str(path),
                    "bytes": path.stat().st_size,
                    "sha256": sha256(path),
                }
                for path in raw_files
            ],
        },
    )
    write(
        output / "attempt_ledger.json",
        {
            "schema_version": 1,
            "attempts": [
                {
                    "attempt_id": report["attempt_id"],
                    "selected": True,
                    "gate_pass": True,
                }
            ],
        },
    )
    (output / "commands.txt").write_text(
        "py -3 scripts/auto11_large_map_formal.py --output <formal-root> "
        f"--implementation-commit {report['implementation_commit']}\n"
        "py -3 scripts/ci_fast.py\n",
        encoding="utf-8",
    )
    (output / "README.md").write_text(
        "# AUTO-11 compact evidence\n\n"
        "The committed evidence records a truth-separated offline large-map "
        "simulation. It does not claim Gazebo, real-vehicle, or J6 execution. "
        "The 2 MB map and full trajectory report stay outside Git and are SHA indexed.\n",
        encoding="utf-8",
    )
    regression = subprocess.run(
        ["py", "-3", "scripts/ci_fast.py"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    write(
        output / "regression_summary.json",
        {
            "schema_version": 1,
            "returncode": regression.returncode,
            "stdout_tail": regression.stdout[-2000:],
            "stderr_tail": regression.stderr[-2000:],
            "pass": regression.returncode == 0,
        },
    )
    if regression.returncode:
        raise RuntimeError("ci_fast regression failed")
    files = [
        {"path": path.name, "bytes": path.stat().st_size, "sha256": sha256(path)}
        for path in sorted(output.iterdir())
        if path.name != "artifact_manifest.json"
    ]
    write(
        output / "artifact_manifest.json",
        {
            "schema_version": 1,
            "stage": "AUTO-11",
            "coverage": 1.0,
            "file_count": len(files),
            "files": files,
        },
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
