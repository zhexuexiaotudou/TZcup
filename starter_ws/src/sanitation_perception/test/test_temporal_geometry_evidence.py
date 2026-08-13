from sanitation_perception.temporal_geometry_evidence import (
    TemporalGeometryConfig,
    TemporalGeometryTrack,
)


def observation(cls="plastic_bottle", *, distance=2.0, short=24, probability=0.98, position=(1.0, 2.0)):
    rest = (1.0 - probability) / 3.0
    probabilities = {name: rest for name in ("plastic_bottle", "metal_can", "paper_litter", "background")}
    probabilities[cls] = probability
    return {"class_probabilities": probabilities, "distance_m": distance, "short_side_px": short, "depth_valid_ratio": 1.0, "map_xy_m": position, "physical_plausible": True, "clean_opportunity_exists": True}


def test_far_tiny_evidence_cannot_trigger_clean_now() -> None:
    track = TemporalGeometryTrack(TemporalGeometryConfig(minimum_observations=2))
    for _ in range(3):
        track.update(observation(distance=5.0, short=6))
    assert not track.clean_action_allowed
    assert track.reobserve_count <= 2


def test_close_consistent_evidence_confirms_class() -> None:
    track = TemporalGeometryTrack(TemporalGeometryConfig(minimum_observations=3, confirmation_probability=0.90))
    for index in range(3):
        track.update(observation(position=(1.0 + index * 0.01, 2.0)))
    assert track.state == "CONFIRMED"
    assert track.final_class == "plastic_bottle"
    assert track.clean_action_allowed


def test_first_frame_class_is_not_permanently_locked() -> None:
    track = TemporalGeometryTrack(TemporalGeometryConfig(minimum_observations=3, confirmation_probability=0.70))
    track.update(observation(cls="metal_can", probability=0.80))
    for _ in range(4):
        track.update(observation(cls="paper_litter", probability=0.95))
    assert track.final_class == "paper_litter"


def test_geometry_scatter_defers_even_high_confidence() -> None:
    track = TemporalGeometryTrack(TemporalGeometryConfig(minimum_observations=3, confirmation_probability=0.80))
    for position in ((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)):
        track.update(observation(position=position))
    assert not track.clean_action_allowed


def test_reobserve_is_bounded() -> None:
    track = TemporalGeometryTrack(TemporalGeometryConfig(minimum_observations=10, maximum_reobserve_count=2))
    for _ in range(8):
        track.update(observation(probability=0.60))
    assert track.reobserve_count == 2
