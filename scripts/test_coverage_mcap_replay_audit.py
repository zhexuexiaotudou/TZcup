from coverage_mcap_replay_audit import REQUIRED_TOPICS, summarize_replay


def test_replay_requires_topics_timeline_brush_transitions_and_real_play():
    report = summarize_replay(
        topic_counts={topic: 2 for topic in REQUIRED_TOPICS},
        states=["READY", "EXECUTING_SWATH", "COMPLETED"],
        component_payloads=[{"component_id": "swath-000"}],
        brush_values=[False, True, False],
        first_timestamp_ns=1_000,
        last_timestamp_ns=2_000,
        play_exit_code=0,
    )

    assert report["ordered_component_ids"] == ["swath-000"]
    assert report["gates"]["ros2_bag_play_succeeded"] is True
    assert report["pass"] is True


def test_replay_fails_when_ros2_bag_play_did_not_complete():
    report = summarize_replay(
        topic_counts={topic: 1 for topic in REQUIRED_TOPICS},
        states=["COMPLETED"],
        component_payloads=[{"component_id": "swath-000"}],
        brush_values=[False, True],
        first_timestamp_ns=1,
        last_timestamp_ns=2,
        play_exit_code=124,
    )

    assert report["gates"]["ros2_bag_play_succeeded"] is False
    assert report["pass"] is False
