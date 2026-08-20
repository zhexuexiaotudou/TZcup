from pathlib import Path

import finalize_trcrv10_integrated_holdout as final


def target(encounter, class_id, small=False, **updates):
    row = {"encounter_id": encounter, "truth_kind": "target", "truth_class": class_id,
           "confirmed_class": class_id, "confirmed_actionable": True, "clean_opportunity": True,
           "clean_now": True, "first_visible_short_side_px": 10 if small else 30,
           "reobserve_count": 1, "extra_distance_m": 1, "extra_time_s": 2}
    row.update(updates)
    return row


def test_perfect_integrated_trace_passes() -> None:
    rows = [target("b", "plastic_bottle", True), target("c", "metal_can"), target("p", "paper_litter")]
    rows.append({"encounter_id": "n", "truth_kind": "negative", "truth_class": None,
                 "confirmed_class": None, "confirmed_actionable": False, "clean_opportunity": False,
                 "clean_now": False, "first_visible_short_side_px": 0, "reobserve_count": 1})
    assert final.compute(rows)["pass"]


def test_small_cohort_is_first_visible_and_wrong_clean_fails() -> None:
    rows = [target("b", "plastic_bottle", True, confirmed_class="metal_can"),
            target("c", "metal_can"), target("p", "paper_litter")]
    result = final.compute(rows)
    assert result["metrics"]["small_target_eventual_correct_class_recall"] == 0.0
    assert result["metrics"]["wrong_class_CLEAN_NOW"] == 1
    assert not result["pass"]


def test_sealed_access_depends_on_complete_gate() -> None:
    source = Path(final.__file__).read_text(encoding="utf-8")
    assert '"sealed_access_authorized_next": passed' in source
    assert '"G10_DEV_VAL_SEALED_read": False' in source
