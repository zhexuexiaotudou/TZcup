from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from validate_formal_operation_speed_profiles import (
    PROFILES,
    FormalOperationSpeedProfileError,
    validate,
)


def _mutated_profiles(tmp_path: Path, mutate) -> Path:
    data = yaml.safe_load(PROFILES.read_text(encoding="utf-8"))
    mutate(data)
    path = tmp_path / "profiles.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


def test_current_final_runner_is_explicitly_not_ready_for_competition_efficiency() -> None:
    result = validate()
    assert result["status"] == "NOT_READY_FOR_COMPETITION_EFFICIENCY"
    assert result["static_theory_is_not_acceptance_evidence"] is True
    assert result["current_final_runner_uses_mapping_safe_0_45_m_s"] is True
    assert result["dry_candidate_enabled_for_formal_runtime"] is False
    assert result["not_ready_reasons"] == [
        "final_runner_still_uses_mapping_safe_0_45_m_s",
        "dry_candidate_has_no_passed_source_bound_measured_coverage_gate",
    ]
    assert result["mapping_can_meet_competition_efficiency"] is False
    assert result["mapping_theoretical_area_m2_h"] == pytest.approx(2138.4)
    assert result["mapping_effective_area_m2_h"] == pytest.approx(1603.8)


def test_dry_candidate_recomputes_3564_design_area_with_only_64_margin() -> None:
    result = validate()
    assert result["dry_candidate_design_area_m2_h"] == pytest.approx(3564.0)
    assert result["dry_candidate_margin_m2_h"] == pytest.approx(64.0)
    assert result["dry_candidate_exact_minimum_theoretical_speed_m_s"] == pytest.approx(
        3500.0 / (1.32 * 0.75 * 3600.0)
    )


def test_rejects_unverified_dry_candidate_being_enabled(tmp_path: Path) -> None:
    path = _mutated_profiles(
        tmp_path,
        lambda data: data["profiles"]["dry_cleaning_competition_candidate"].update(
            {"enabled_for_formal_runtime": True}
        ),
    )
    with pytest.raises(FormalOperationSpeedProfileError, match="cannot be enabled"):
        validate(profiles_path=path)


def test_rejects_nonfinite_or_boolean_speed_values(tmp_path: Path) -> None:
    path = _mutated_profiles(
        tmp_path,
        lambda data: data["profiles"]["mapping_safe"].update(
            {"maximum_linear_speed_m_s": True}
        ),
    )
    with pytest.raises(FormalOperationSpeedProfileError, match="must be a number"):
        validate(profiles_path=path)


def test_rejects_static_theory_being_relabelled_as_acceptance_evidence(tmp_path: Path) -> None:
    path = _mutated_profiles(
        tmp_path,
        lambda data: data["competition_efficiency_contract"].update(
            {"static_theory_is_not_acceptance_evidence": False}
        ),
    )
    with pytest.raises(FormalOperationSpeedProfileError, match="must remain non-acceptance evidence"):
        validate(profiles_path=path)
