from sanitation_manipulation.cube_geometry import CubeCandidate
from sanitation_manipulation.scenario_adapter_contract import (
    PerceivedScenarioCube,
    PublicEpisodeIdentity,
    TruthFreeScenarioGraspRequestBuilder,
)


def test_scenario_draft_uses_public_identity_and_perceived_geometry_only():
    episode = PublicEpisodeIdentity(
        split="train",
        map_id="train-map-000",
        mission_id="train-map-000-mission-001",
        public_manifest_sha256="a" * 64,
        world_sha256="b" * 64,
    )
    observation = PerceivedScenarioCube(
        episode=episode,
        target_id="perceived-cube-1",
        frame_id="camera_depth_optical_frame",
        observation_stamp_ns=123,
        cube=CubeCandidate((0.4, 0.1, 0.015), (0.03, 0.03, 0.03), 0.0, 40, 0.0),
    )
    request = TruthFreeScenarioGraspRequestBuilder().build_request(observation)
    assert request.target_id == observation.target_id
    assert request.cube == observation.cube
    assert "truth" not in repr(observation).lower()
