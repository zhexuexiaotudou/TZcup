import finalize_trcrv10_identifiability as final


def perfect(support: int = 10) -> dict:
    return {
        "macro_f1": 1.0,
        "per_class": {name: {"precision": 1.0, "recall": 1.0, "f1": 1.0, "support": support} for name in final.CLASSES},
        "confusion": [[support, 0, 0], [0, support, 0], [0, 0, support]],
    }


def test_passes_applies_macro_and_every_class() -> None:
    row = perfect()
    assert final.passes(row, .97, .95)
    row["per_class"]["metal_can"]["recall"] = .94
    assert not final.passes(row, .97, .95)


def test_reliable_bucket_is_conservative_across_both_models() -> None:
    rows = []
    for model in ("convnext_tiny", "resnet18"):
        by_size = {bucket: perfect() for bucket in final.BUCKETS}
        rows.append({"model": model, "view": "tight", "by_size": by_size})
    assert final.reliable_bucket(rows, "tight") == "lt12"
    rows[1]["by_size"]["lt12"]["macro_f1"] = .9
    assert final.reliable_bucket(rows, "tight") == "12_18"


def test_missing_bucket_cannot_pass_vacuously() -> None:
    rows = [
        {"model": model, "view": "tight", "by_size": {"64_96": perfect(), "ge96": perfect()}}
        for model in ("convnext_tiny", "resnet18")
    ]
    assert final.reliable_bucket(rows, "tight") == "64_96"
    del rows[1]["by_size"]["64_96"]
    assert final.reliable_bucket(rows, "tight") == "ge96"


def test_combined_confusion_recomputes_metrics() -> None:
    result = final.combine_confusions([perfect(2), perfect(3)])
    assert result["macro_f1"] == 1.0
    assert all(row["support"] == 5 for row in result["per_class"].values())


def test_sample_recommendation_is_disclosed_not_promoted_to_hard_gate() -> None:
    source = open(final.__file__, encoding="utf-8").read()
    assert "holdout_support_by_run" in source
    assert '"sample_recommendation_is_not_a_hard_gate": True' in source
