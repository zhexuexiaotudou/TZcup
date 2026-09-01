from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from validate_electrical_harness_thermal_readiness import DEFAULT_CONFIG, ROOT, validate


def _payload() -> dict:
    return yaml.safe_load(DEFAULT_CONFIG.read_text(encoding="utf-8"))


def _write(tmp_path: Path, payload: dict) -> Path:
    path = tmp_path / "electrical_harness_thermal_readiness.yaml"
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return path


def test_source_bound_electrical_readiness_is_valid_but_not_ready() -> None:
    result = validate()
    assert result["valid"] is True
    assert result["ready"] is False
    assert result["computed"] == {
        "source_total_continuous_w": 649.9,
        "source_total_peak_w": 1026.0,
        "branch_total_continuous_w": 649.9,
        "branch_total_peak_w": 1026.0,
    }


@pytest.mark.parametrize(
    ("branch", "field"),
    [
        ("ur5e_control", "continuous_w"),
        ("s100_compute", "peak_w"),
        ("sensors_24v", "voltage_v"),
    ],
)
@pytest.mark.parametrize("value", [True, float("nan"), float("inf")])
def test_rejects_boolean_and_nonfinite_branch_numbers(
    tmp_path: Path, branch: str, field: str, value: object
) -> None:
    payload = _payload()
    payload["branches"][branch][field] = value
    result = validate(_write(tmp_path, payload), root=ROOT)
    assert result["valid"] is False
    assert f"branches.{branch}.{field} must be a finite number" in result["errors"]


def test_rejects_power_budget_total_that_does_not_match_source(tmp_path: Path) -> None:
    payload = _payload()
    payload["power_budget_binding"]["declared_total_peak_w"] = 1.0
    result = validate(_write(tmp_path, payload), root=ROOT)
    assert result["valid"] is False
    assert "power_budget_binding.declared_total_peak_w does not match pre_urdf power_budget" in result["errors"]


def test_rejects_known_voltage_branch_above_upstream_capacity(tmp_path: Path) -> None:
    payload = _payload()
    payload["branches"]["sensors_12v"]["peak_w"] = 121.0
    result = validate(_write(tmp_path, payload), root=ROOT)
    assert result["valid"] is False
    assert "branches.sensors_12v.peak_w exceeds its upstream rail capacity" in result["errors"]


def test_rejects_unverified_branch_or_top_level_declared_ready(tmp_path: Path) -> None:
    payload = _payload()
    payload["ready"] = True
    payload["branches"]["s100_compute"]["branch_ready"] = True
    result = validate(_write(tmp_path, payload), root=ROOT)
    assert result["valid"] is False
    assert "ready must be boolean false" in result["errors"]
    assert "branches.s100_compute.branch_ready must be boolean false" in result["errors"]


def test_rejects_missing_thermal_or_safety_blocker(tmp_path: Path) -> None:
    payload = _payload()
    payload["branches"]["recovery_pump"]["thermal_status"] = "VERIFIED"
    payload["blocking_categories"].pop("thermal_design_and_validation")
    result = validate(_write(tmp_path, payload), root=ROOT)
    assert result["valid"] is False
    assert any("branches.recovery_pump.thermal_status must remain NOT_READY" in error for error in result["errors"])
    assert "blocking_categories must contain exactly every blocked electrical release category" in result["errors"]
