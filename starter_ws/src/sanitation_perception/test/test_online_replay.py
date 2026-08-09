from sanitation_perception.dynamic_trash_map import DynamicTrashMap

from online_map_test_support import observation, record_sweep


def test_replay_reconstructs_same_online_map():
    original = DynamicTrashMap.start_new("mission-replay")
    for stamp, x_m in (
        (1_000_000_000, 2.00),
        (1_100_000_000, 2.04),
        (1_200_000_000, 1.98),
    ):
        record_sweep(original, stamp)
        original.ingest(observation(original, stamp, x_m=x_m))

    replayed = DynamicTrashMap.replay(
        original.observation_log,
        original.observed_regions.to_records(),
        original.mission_id,
    )
    source_target = next(iter(original.targets.values()))
    replay_target = next(iter(replayed.targets.values()))
    assert replayed.count == original.count
    assert replay_target.current_class == source_target.current_class
    assert replay_target.track_state == source_target.track_state
    assert abs(replay_target.map_x_m - source_target.map_x_m) < 1e-12
    assert replayed.observation_log == original.observation_log


def test_explicit_same_mission_resume_restores_targets(tmp_path):
    original = DynamicTrashMap.start_new("mission-resume")
    stamp = 1_000_000_000
    record_sweep(original, stamp)
    original.ingest(observation(original, stamp))
    path = tmp_path / "dynamic-map.json"
    original.persist(path)

    resumed = DynamicTrashMap.resume_same_mission(path, "mission-resume")
    assert resumed.count == 1
    assert resumed.snapshot()["mission_id"] == "mission-resume"
