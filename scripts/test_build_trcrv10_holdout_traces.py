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
