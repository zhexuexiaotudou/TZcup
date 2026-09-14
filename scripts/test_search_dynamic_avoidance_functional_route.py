from __future__ import annotations

import json
import math
from pathlib import Path

from prepare_dynamic_avoidance_single_run import _sha256
from search_dynamic_avoidance_functional_route import (
    corridor_capacity,
    search_route,
)
from test_prepare_formal_dynamic_obstacle_schedule import _write_inputs


ROOT = Path(__file__).resolve().parents[1]
SMOKE_PROTOCOL_PATH = (
    ROOT / "config/dynamic_avoidance_functional_smoke_protocol.json"
)


def test_corridor_capacity_exposes_short_leg_three_crossing_impossibility() -> None:
    proof = corridor_capacity(
        nominal_leg_m=6.0,
        fraction_start=0.20,
        fraction_end=0.80,
        crossing_count=3,
    )
    assert math.isclose(proof["available_centerline_span_m"], 3.6)
    assert proof["required_center_span_m"] == 8.0
    assert proof["geometrically_feasible"] is False


def test_search_materializes_single_crossing_smoke_route(tmp_path: Path) -> None:
    manifest, world, base_schedule = _write_inputs(tmp_path)
    protocol = json.loads(SMOKE_PROTOCOL_PATH.read_text(encoding="utf-8"))
    protocol["schedule"]["seed"] = 1
    result = search_route(
        episode_manifest=manifest,
        public_world=world,
        base_schedule=base_schedule,
        protocol=protocol,
        protocol_sha256="a" * 64,
        input_sha256={
            "episode_manifest": _sha256(manifest),
            "public_world": _sha256(world),
            "base_schedule": _sha256(base_schedule),
        },
        max_seed_candidates=128,
        corridor_ranges=[("declared", 0.20, 0.80)],
    )
    assert result["status"] == "FUNCTIONAL_SMOKE_ROUTE_MATERIALIZED", json.dumps(
        result["retained_failure_examples"],
        indent=2,
    )
    assert result["selected_crossing_count"] == 1
    assert (
        result["predeclared_route_manifest"]["selected_obstacle"]["object_id"]
        == "walker_0"
    )
    proof = result["route_materialization"]["static_interaction_materialization"]
    sample = proof["selected_interaction_sample"]
    assert proof["status"] == "STATIC_OBSTACLE_INTERACTION_SAMPLEABLE"
    assert proof["passed"] is True
    assert sample["interaction_sample_trigger_delta_s"] <= proof["trigger_window_s"]
    assert sample["interaction_sample_surface_gap_m"] >= 0.12
    assert sample["interaction_sample_status_alignment_s"] <= 0.5
