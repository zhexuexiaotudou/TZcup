"""Draft public scenario-to-manipulation contract with no truth dependency.

This module intentionally imports neither ``sanitation_campus_scenario`` nor
``sanitation_active_cleaning``.  A future adapter may populate the identity from
a controller-facing episode manifest and the target from perceived geometry;
evaluation ground truth and environment-driver schedules are not accepted.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from .active_cleaning_adapter import SingleTargetGraspRequest
from .cube_geometry import CubeCandidate


@dataclass(frozen=True)
class PublicEpisodeIdentity:
    split: str
    map_id: str
    mission_id: str
    public_manifest_sha256: str
    world_sha256: str

    def __post_init__(self) -> None:
        if self.split not in {"train", "val", "hidden"}:
            raise ValueError("split must be train, val, or hidden")
        values = (self.map_id, self.mission_id, self.public_manifest_sha256, self.world_sha256)
        if any(not value for value in values):
            raise ValueError("public episode identity fields must be non-empty")


@dataclass(frozen=True)
class PerceivedScenarioCube:
    episode: PublicEpisodeIdentity
    target_id: str
    frame_id: str
    observation_stamp_ns: int
    cube: CubeCandidate

    def __post_init__(self) -> None:
        if not self.target_id or not self.frame_id:
            raise ValueError("perceived target and frame identifiers must be non-empty")
        if self.observation_stamp_ns < 0:
            raise ValueError("observation_stamp_ns cannot be negative")


class ScenarioGraspRequestAdapter(Protocol):
    def build_request(self, observation: PerceivedScenarioCube) -> SingleTargetGraspRequest:
        """Build one request from public identity plus perceived geometry."""


class TruthFreeScenarioGraspRequestBuilder:
    def build_request(self, observation: PerceivedScenarioCube) -> SingleTargetGraspRequest:
        return SingleTargetGraspRequest(observation.target_id, observation.cube)
