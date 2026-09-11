#!/usr/bin/env python3
"""Offline sanity gate for a sealed saved-map coverage-planner input."""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_formal_campus_integration"))

from sanitation_formal_campus_integration.map_lifecycle_core import (
    MapLifecycleError, load_campus_map_contract,
    validate_saved_map_cleaning_consumer_bundle,
)
from sanitation_formal_campus_integration.saved_map_coverage_core import (
    FORMAL_OPERATION_WIDTH_M, SavedMapCoverageError, load_product_mission_geometry,
)

RECOMMENDED_LANE_SPACING_M = 1.056  # 80% of the declared 1.32 m effective width.


def _runs(columns: list[int]) -> list[tuple[int, int]]:
    result: list[tuple[int, int]] = []
    for column in sorted(columns):
        if not result or column > result[-1][1] + 1:
            result.append((column, column))
        else:
            result[-1] = (result[-1][0], column)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-root", required=True, type=Path)
    parser.add_argument("--episode-manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        contract = load_campus_map_contract(args.episode_manifest)
        validate_saved_map_cleaning_consumer_bundle(args.map_root, contract)
        geometry = load_product_mission_geometry(args.map_root / "mission_geometry.yaml")
    except (MapLifecycleError, SavedMapCoverageError, OSError, ValueError) as exc:
        report = {"passed": False, "blockers": [f"sealed_map_reload: {exc}"]}
    else:
        by_row: dict[int, list[int]] = defaultdict(list)
        for column, row in geometry.free_cells:
            by_row[row].append(column)
        rows = sorted(by_row)
        stride = max(1, math.floor(RECOMMENDED_LANE_SPACING_M / geometry.raster_resolution_m))
        selected = rows[::stride]
        if rows and selected[-1] != rows[-1]:
            selected.append(rows[-1])
        lanes = [_runs(by_row[row]) for row in selected]
        realizable = [any((end - start + 1) * geometry.raster_resolution_m >= FORMAL_OPERATION_WIDTH_M for start, end in lane) for lane in lanes]
        transition_blockers = [
            row for row, before, after in zip(selected[1:], lanes, lanes[1:])
            if not any(max(a0, b0) <= min(a1, b1) for a0, a1 in before for b0, b1 in after)
        ]
        report = {
            "passed": not transition_blockers and all(realizable),
            "operation_width_m": FORMAL_OPERATION_WIDTH_M,
            "recommended_max_lane_spacing_m": RECOMMENDED_LANE_SPACING_M,
            "realized_lane_spacing_m": stride * geometry.raster_resolution_m,
            "coverage_raster_resolution_m": geometry.raster_resolution_m,
            "free_cells": len(geometry.free_cells),
            "candidate_lane_count": len(selected),
            "candidate_lanes": [
                {
                    "row": row,
                    "y_m": geometry.origin_y + (row + 0.5) * geometry.raster_resolution_m,
                    "free_segments_m": [
                        [
                            geometry.origin_x + start * geometry.raster_resolution_m,
                            geometry.origin_x + (end + 1) * geometry.raster_resolution_m,
                        ]
                        for start, end in lane
                    ],
                }
                for row, lane in zip(selected, lanes)
            ],
            "narrow_or_empty_lane_rows": [row for row, ok in zip(selected, realizable) if not ok],
            "disconnected_turn_rows": transition_blockers,
            "planner_path_authority": "opennav_coverage_compute_coverage_path",
            "candidate_lanes_summary_only": True,
            "blockers": [],
        }
        if not report["passed"]:
            report["blockers"] = [
                *( ["narrow_or_empty_coverage_lane"] if not all(realizable) else [] ),
                *( ["adjacent_coverage_lanes_have_no_free_space_turn_connection"] if transition_blockers else [] ),
            ]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, sort_keys=True))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
