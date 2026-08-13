import build_trcrv10_holdout_traces as traces


def test_classifier_rows_group_by_scene_frame_and_proposal() -> None:
    report = {"evaluated_rows": [
        {"scene": "s", "frame_index": 1, "proposal_index": 2, "view": "tight"},
        {"scene": "s", "frame_index": 1, "proposal_index": 2, "view": "context"},
    ]}
    assert set(traces.group_classifier_rows(report)[("s", 1, 2)]) == {"tight", "context"}


def test_trace_builder_is_holdout_and_sealed_safe() -> None:
    source = open(traces.__file__, encoding="utf-8").read()
    assert '"split": "G10_HOLDOUT"' in source
    assert '"production_runtime_gt_used": False' in source
    assert '"G10_DEV_VAL_SEALED_read": False' in source
    assert '"VAL_NEW_read": False' in source
    assert '"G5_V2_read": False' in source
    assert 'tight.get("projection_covariance_m2"), float("inf")' in source


def test_persistence_is_derived_from_adjacent_bbox_association() -> None:
    def row(box):
        return {"tight": {"source_bbox_xyxy": box}}
    groups = {
        ("s", 0, 0): row([0, 0, 10, 10]),
        ("s", 1, 0): row([1, 0, 11, 10]),
        ("s", 2, 0): row([2, 0, 12, 10]),
        ("s", 4, 0): row([2, 0, 12, 10]),
    }
    result = traces.persistence_by_proposal(groups)
    assert result[("s", 2, 0)] == 3
    assert result[("s", 4, 0)] == 1


def test_missing_covariance_fails_closed() -> None:
    assert traces.value_or_default(None, float("inf")) == float("inf")
    assert traces.value_or_default(0.01, float("inf")) == 0.01
