"""Truth-free active-cleaning to placeholder manipulation clearance gate.

The adapter accepts exactly one perceived 30 mm cube target.  It delegates the
pick/place sequence to :class:`CubeTaskController` and reports ``cleared`` only
after the controller has both verified placement and registered the target in
the count-limited rear bin.  The default backend is deterministic and mock-only;
its JSON evidence is deliberately ineligible as Gazebo or real-robot evidence.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

from .cube_geometry import CubeCandidate, generate_top_grasps
from .cube_task import (
    BIN_INTERNAL_SIZE_M,
    MAX_GRASP_ATTEMPTS,
    MAX_TARGETS_PER_EPISODE,
    CubeTaskController,
    MockGripperBackend,
    MockNavigationBackend,
    MockPlanningBackend,
    MockSafetyBackend,
    TargetTaskState,
    VerificationEvidence,
)


ADAPTER_ID = "active_cleaning_manipulation_placeholder_v1"
EVIDENCE_LEVEL = "MOCK_TASK_SEMANTICS_ONLY"


def _canonical_json(value: Mapping[str, Any]) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def _sha256(value: Mapping[str, Any]) -> str:
    return hashlib.sha256(_canonical_json(value).encode("utf-8")).hexdigest()


def _finite_vector(raw: Any, name: str, *, positive: bool = False) -> tuple[float, float, float]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        raise ValueError(f"{name} must contain exactly three numbers")
    result = tuple(float(value) for value in raw)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must be finite")
    if positive and any(value <= 0.0 for value in result):
        raise ValueError(f"{name} must be positive")
    return result  # type: ignore[return-value]


@dataclass(frozen=True)
class SingleTargetGraspRequest:
    """URDF-independent request built from a perceived cube, never truth."""

    target_id: str
    cube: CubeCandidate

    def __post_init__(self) -> None:
        if not self.target_id or not self.target_id.strip():
            raise ValueError("target_id must be non-empty")
        if any(abs(size - 0.030) > 0.010 + 1.0e-9 for size in self.cube.size_m):
            raise ValueError("cube dimensions must remain within the 30 mm contract")
        if self.cube.point_count <= 0:
            raise ValueError("cube point_count must be positive")
        values = (*self.cube.center_m, *self.cube.size_m, self.cube.yaw_rad, self.cube.dimension_error_m)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("cube geometry must be finite")
        if self.cube.dimension_error_m < 0.0:
            raise ValueError("dimension_error_m cannot be negative")

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "SingleTargetGraspRequest":
        if not isinstance(data, Mapping):
            raise ValueError("request must be a JSON object")
        expected = {"schema_version", "target_id", "cube"}
        if set(data) != expected:
            raise ValueError(f"request keys must equal {sorted(expected)}")
        if data["schema_version"] != 1:
            raise ValueError("schema_version must equal 1")
        cube = data["cube"]
        if not isinstance(cube, Mapping):
            raise ValueError("cube must be a JSON object")
        cube_keys = {
            "center_m",
            "size_m",
            "yaw_rad",
            "point_count",
            "dimension_error_m",
        }
        if set(cube) != cube_keys:
            raise ValueError(f"cube keys must equal {sorted(cube_keys)}")
        target_id = data["target_id"]
        if not isinstance(target_id, str):
            raise ValueError("target_id must be a string")
        point_count = cube["point_count"]
        if isinstance(point_count, bool) or not isinstance(point_count, int):
            raise ValueError("cube.point_count must be an integer")
        candidate = CubeCandidate(
            center_m=_finite_vector(cube["center_m"], "cube.center_m"),
            size_m=_finite_vector(cube["size_m"], "cube.size_m", positive=True),
            yaw_rad=float(cube["yaw_rad"]),
            point_count=point_count,
            dimension_error_m=float(cube["dimension_error_m"]),
        )
        return cls(target_id, candidate)

    def to_mapping(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "target_id": self.target_id,
            "cube": {
                "center_m": list(self.cube.center_m),
                "size_m": list(self.cube.size_m),
                "yaw_rad": self.cube.yaw_rad,
                "point_count": self.cube.point_count,
                "dimension_error_m": self.cube.dimension_error_m,
            },
        }


@dataclass(frozen=True)
class MockExecutionProfile:
    """Deterministic fault-injection profile for the placeholder backend."""

    navigation_outcomes: tuple[bool, ...] = (True,)
    planning_outcomes: tuple[bool, ...] = (True,)
    grasp_evidence: tuple[VerificationEvidence, ...] | None = None
    place_outcomes: tuple[bool, ...] = (True,)
    safe: bool = True


class _RecordingMockGripperBackend(MockGripperBackend):
    def __init__(self, profile: MockExecutionProfile) -> None:
        super().__init__(profile.grasp_evidence, profile.place_outcomes)
        self.observed_evidence: list[VerificationEvidence] = []

    def grasp_and_lift(self, candidate):  # type: ignore[no-untyped-def]
        evidence = super().grasp_and_lift(candidate)
        self.observed_evidence.append(evidence)
        return evidence


@dataclass(frozen=True)
class ClearanceDecision:
    target_id: str
    cleared: bool
    verified_in_bin: bool
    state: str
    attempts: int
    reason: str | None
    evidence: Mapping[str, Any]

    def to_mapping(self) -> dict[str, Any]:
        return dict(self.evidence)


class ActiveCleaningManipulationAdapter:
    """Stateful one-target clearance gate with a shared 20-target mock bin."""

    def __init__(self, profile: MockExecutionProfile | None = None) -> None:
        self.profile = profile or MockExecutionProfile()
        self.navigation = MockNavigationBackend(self.profile.navigation_outcomes)
        self.planning = MockPlanningBackend(self.profile.planning_outcomes)
        self.gripper = _RecordingMockGripperBackend(self.profile)
        self.safety = MockSafetyBackend(self.profile.safe)
        self.controller = CubeTaskController(
            navigation=self.navigation,
            planning=self.planning,
            gripper=self.gripper,
            safety=self.safety,
        )
        self._attempt_evidence: dict[str, tuple[VerificationEvidence, ...]] = {}

    def execute(self, request: SingleTargetGraspRequest) -> ClearanceDecision:
        before_evidence = len(self.gripper.observed_evidence)
        candidates = generate_top_grasps(request.target_id, request.cube)
        outcome = self.controller.execute(request.target_id, candidates)
        newly_observed = tuple(self.gripper.observed_evidence[before_evidence:])
        if newly_observed:
            self._attempt_evidence[request.target_id] = (
                *self._attempt_evidence.get(request.target_id, ()),
                *newly_observed,
            )

        target_registered_in_bin = self.controller.collection_bin.contains(request.target_id)
        verified_in_bin = bool(
            outcome.state is TargetTaskState.CLEARED
            and outcome.success
            and outcome.placed_in_bin
            and target_registered_in_bin
        )
        cleared = verified_in_bin
        request_mapping = request.to_mapping()
        evidence: dict[str, Any] = {
            "schema_version": 1,
            "adapter_id": ADAPTER_ID,
            "request_sha256": _sha256(request_mapping),
            "target_id": request.target_id,
            "decision": {
                "cleared": cleared,
                "verified_in_bin": verified_in_bin,
                "task_state": outcome.state.value,
                "is_terminal": outcome.state in (TargetTaskState.CLEARED, TargetTaskState.DEFERRED),
                "attempts": outcome.attempts,
                "maximum_attempts": MAX_GRASP_ATTEMPTS,
                "reason": outcome.reason,
            },
            "verification": {
                "controller_success": outcome.success,
                "controller_placed_in_bin": outcome.placed_in_bin,
                "target_registered_in_bin": target_registered_in_bin,
                "attempt_evidence": [
                    {**asdict(row), "independent_categories": row.independent_categories, "accepted": row.accepted}
                    for row in self._attempt_evidence.get(request.target_id, ())
                ],
                "state_history": list(outcome.history),
            },
            "backend": {
                "profile": "deterministic_mock",
                "navigation_calls": self.navigation.calls,
                "planning_calls": self.planning.calls,
                "grasp_calls": self.gripper.grasp_calls,
                "place_calls": self.gripper.place_calls,
                "recovery_calls": self.gripper.recovery_calls,
            },
            "bin_contract": {
                "internal_size_m": list(BIN_INTERNAL_SIZE_M),
                "single_layer": True,
                "stacked": False,
                "maximum_targets": MAX_TARGETS_PER_EPISODE,
                "current_count": self.controller.collection_bin.count,
                "packing_proof": False,
            },
            "authority": {
                "evidence_level": EVIDENCE_LEVEL,
                "evidence_authority": False,
                "placeholder_evidence_only": True,
                "real_robot_evidence": False,
                "gazebo_runtime_evidence": False,
                "measured_urdf_used": False,
                "moveit_or_hardware_execution_used": False,
                "truth_used_for_control": False,
            },
        }
        evidence["evidence_sha256"] = _sha256(evidence)
        return ClearanceDecision(
            target_id=request.target_id,
            cleared=cleared,
            verified_in_bin=verified_in_bin,
            state=outcome.state.value,
            attempts=outcome.attempts,
            reason=outcome.reason,
            evidence=evidence,
        )


def _profile(name: str) -> MockExecutionProfile:
    accepted = VerificationEvidence(gripper_width_ok=True, target_follows_tool=True)
    rejected = VerificationEvidence(gripper_width_ok=True)
    profiles = {
        "success": MockExecutionProfile(),
        "fail_then_success": MockExecutionProfile(grasp_evidence=(rejected, accepted)),
        "grasp_fail": MockExecutionProfile(grasp_evidence=(rejected, rejected)),
        "place_fail": MockExecutionProfile(grasp_evidence=(accepted,), place_outcomes=(False,)),
        "navigation_fail": MockExecutionProfile(navigation_outcomes=(False,)),
    }
    return profiles[name]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--mock-profile",
        choices=("success", "fail_then_success", "grasp_fail", "place_fail", "navigation_fail"),
        default="success",
    )
    return parser


def _emit(payload: Mapping[str, Any], output: Path | None) -> None:
    encoded = json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if output is None:
        print(encoded, end="")
        return
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(encoded, encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        raw = json.loads(args.request.read_text(encoding="utf-8"))
        request = SingleTargetGraspRequest.from_mapping(raw)
        decision = ActiveCleaningManipulationAdapter(_profile(args.mock_profile)).execute(request)
        payload = decision.to_mapping()
        code = 0 if decision.cleared else 2
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
        payload = {
            "schema_version": 1,
            "adapter_id": ADAPTER_ID,
            "decision": {"cleared": False, "verified_in_bin": False},
            "error": {"type": type(exc).__name__, "message": str(exc)},
            "authority": {
                "evidence_level": EVIDENCE_LEVEL,
                "evidence_authority": False,
                "placeholder_evidence_only": True,
                "real_robot_evidence": False,
            },
        }
        code = 2
    _emit(payload, args.output)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
