import select_trcrv10_proposal as selection


def test_rank_prefers_pass_then_lower_flood() -> None:
    base = {
        "metrics": {
            "eventual_proposal_recall": 1.0,
            "small_eventual_proposal_recall": 1.0,
            "proposal_fp_per_frame": 0.4,
            "persistence_frames": 3,
            "threshold": 0.5,
        },
        "pass": True,
    }
    cleaner = {"metrics": {**base["metrics"], "proposal_fp_per_frame": 0.2}, "pass": True}
    failed = {"metrics": {**base["metrics"], "proposal_fp_per_frame": 0.0}, "pass": False}
    assert selection.rank(cleaner) > selection.rank(base)
    assert selection.rank(base) > selection.rank(failed)


def test_search_space_is_protocol_bounded() -> None:
    assert selection.PERSISTENCE == (2, 3, 4, 5)
    assert min(selection.THRESHOLDS) == 0.05
    assert max(selection.THRESHOLDS) == 0.95
