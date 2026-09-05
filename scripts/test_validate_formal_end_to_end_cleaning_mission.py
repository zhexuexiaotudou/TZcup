import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from aggregate_formal_single_episode_cleaning_mission import aggregate
from test_aggregate_formal_single_episode_cleaning_mission import build_raw
import validate_formal_end_to_end_cleaning_mission as end_to_end
from validate_formal_end_to_end_cleaning_mission import validate


def test_complete_single_live_episode_passes_after_raw_recomputation(tmp_path: Path) -> None:
    payload = aggregate(build_raw(tmp_path))
    result = validate(payload)
    assert result["passed"] is True
    assert result["validated_closed_loop"]["same_map_full_coverage_efficiency_at_least_3500"] is True
    assert result["runtime_gate_binding"] == payload["evidence"]["runtime_gate_binding"]


def test_runtime_gate_binding_must_match_the_final_sidecar(tmp_path: Path) -> None:
    payload = aggregate(build_raw(tmp_path))
    result = validate(payload, runtime_gate_binding={"status": "FORMAL_RUNTIME_GATE_BOUND"})
    assert result["passed"] is False
    assert "differs from the final sidecar" in "\n".join(result["errors"])


def test_final_sidecar_is_rechecked_against_current_snapshot_and_session(
    tmp_path: Path,
) -> None:
    raw_path = build_raw(tmp_path)
    raw = json.loads(raw_path.read_text(encoding="utf-8"))
    snapshot = tmp_path / "snapshot.json"
    session = tmp_path / "session.json"
    sidecar = Path(raw["input_binding"]["artifacts"]["runtime_binding"]["path"])
    binding = json.loads(sidecar.read_text(encoding="utf-8"))
    binding["acceptance_session_binding"].update(
        {
            "session_manifest": str(session.resolve()),
            "session_manifest_sha256": end_to_end._sha256(session),
            "snapshot_current_source_verified": True,
        }
    )
    sidecar.write_text(json.dumps(binding), encoding="utf-8")

    assert end_to_end._current_runtime_binding(snapshot, session, sidecar) == binding

    binding["acceptance_session_binding"]["snapshot"]["source_inventory_sha256"] = "x" * 64
    sidecar.write_text(json.dumps(binding), encoding="utf-8")
    with pytest.raises(end_to_end.EndToEndMissionError, match="current snapshot/session"):
        end_to_end._current_runtime_binding(snapshot, session, sidecar)


def test_tampered_aggregate_fails_deterministic_raw_recomputation(tmp_path: Path) -> None:
    payload = aggregate(build_raw(tmp_path))
    payload["field"]["width_m"] = 999.0
    result = validate(payload)
    assert not result["passed"]
    assert any("deterministic recomputation" in row for row in result["errors"])


def test_cross_episode_metric_splice_fails_raw_reaggregation(tmp_path: Path) -> None:
    raw_path = build_raw(tmp_path)
    payload = aggregate(raw_path)
    raw = json.loads(raw_path.read_text())
    raw["metric_sources"][3]["episode_id"] = "old-episode"
    raw_path.write_text(json.dumps(raw), encoding="utf-8")
    result = validate(payload)
    assert not result["passed"]
    assert any("hash mismatch" in row or "identity mismatch" in row for row in result["errors"])


def test_historical_source_and_truth_control_fail_closed(tmp_path: Path) -> None:
    payload = aggregate(build_raw(tmp_path))
    payload["evidence"]["metric_sources"][5]["source_class"] = "historical_artifact"
    payload["evidence"]["truth_boundary"]["control_truth_topics_subscribed"] = ["/world/model/info"]
    result = validate(payload)
    assert not result["passed"]
    assert any("historical" in row for row in result["errors"])
    assert any("truth entered" in row for row in result["errors"])


def test_boolean_mass_claim_without_increment_fails_recomputation(tmp_path: Path) -> None:
    payload = aggregate(build_raw(tmp_path))
    payload["water_recovery"]["tank_mass_increment_kg"] = 0.0
    payload["water_recovery"]["dynamic_tank_mass_increment_verified"] = True
    result = validate(payload)
    assert not result["passed"]
    assert any("deterministic recomputation" in row or "increments missing" in row for row in result["errors"])


def test_same_map_competition_efficiency_below_3500_fails_closed(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    payload = aggregate(build_raw(tmp_path))
    efficiency = payload["planning"]["same_map_full_coverage_efficiency"]
    efficiency["actual_duration_sec"] = 20000.0 / 3499.0 * 3600.0
    efficiency["measured_net_efficiency_m2_h"] = 3499.0
    efficiency["recomputed_net_efficiency_m2_h"] = 3499.0
    monkeypatch.setattr(end_to_end, "aggregate", lambda _: payload)
    result = end_to_end.validate(payload)
    assert not result["passed"]
    assert "below 3500" in "\n".join(result["errors"])


@pytest.mark.parametrize(
    ("key", "value", "message"),
    [
        ("covered_area_m2", True, "finite numeric"),
        ("measured_net_efficiency_m2_h", 3601.0, "formula mismatch"),
        ("return_distance_included", True, "includes return-home"),
    ],
)
def test_same_map_competition_efficiency_type_formula_and_return_gates(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    key: str,
    value: object,
    message: str,
) -> None:
    payload = aggregate(build_raw(tmp_path))
    payload["planning"]["same_map_full_coverage_efficiency"][key] = value
    monkeypatch.setattr(end_to_end, "aggregate", lambda _: payload)
    result = end_to_end.validate(payload)
    assert not result["passed"]
    assert message in "\n".join(result["errors"])
