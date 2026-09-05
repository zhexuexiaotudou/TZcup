"""URDF-independent DOSOD + EdgeSAM runtime and fusion contracts.

The module deliberately does not pretend that model artifacts are bundled.  It
validates the external PC/S100 runtime boundary and fuses only production-side
detector, segmenter and depth-geometry proposals.  Evaluation/Gazebo truth is
rejected at the API boundary.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping, Any

import yaml


class OpenVocabularyContractError(RuntimeError):
    """Raised when the runtime or proposal boundary is not safe to use."""


FORBIDDEN_CONTROL_SOURCES = {
    "gazebo_truth",
    "ground_truth",
    "evaluation",
    "evaluation_ground_truth",
    "semantic_truth",
    "instance_truth",
}


@dataclass(frozen=True)
class RuntimeComponent:
    name: str
    backend: str
    artifact_relative_path: str | None
    source_revision_required: bool
    platform_support: str


@dataclass(frozen=True)
class OpenVocabularyProfile:
    profile_id: str
    platform: str
    soc_identity: str | None
    detector: RuntimeComponent
    segmenter: RuntimeComponent
    development_only: bool
    journey6_evidence_allowed: bool
    ground_truth_control_allowed: bool


@dataclass(frozen=True)
class RuntimeReadiness:
    ready: bool
    missing: tuple[str, ...]
    detail: str


@dataclass(frozen=True)
class Proposal2D:
    proposal_id: str
    source: str
    label: str
    score: float
    bbox_xyxy: tuple[float, float, float, float]
    mask_area_px: int | None = None
    depth_xyz_m: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class FusedProposal:
    proposal_id: str
    label: str
    score: float
    bbox_xyxy: tuple[float, float, float, float]
    sources: tuple[str, ...]
    mask_area_px: int | None
    depth_xyz_m: tuple[float, float, float] | None


def _component(raw: Mapping[str, Any], path: str) -> RuntimeComponent:
    name = raw.get("name")
    backend = raw.get("backend")
    artifact = raw.get("artifact_relative_path")
    if not isinstance(name, str) or not name:
        raise OpenVocabularyContractError(f"{path}.name must be non-empty")
    if not isinstance(backend, str) or not backend:
        raise OpenVocabularyContractError(f"{path}.backend must be non-empty")
    if artifact is not None and (not isinstance(artifact, str) or not artifact):
        raise OpenVocabularyContractError(
            f"{path}.artifact_relative_path must be null or non-empty"
        )
    return RuntimeComponent(
        name=name,
        backend=backend,
        artifact_relative_path=artifact,
        source_revision_required=bool(raw.get("source_revision_required", True)),
        platform_support=str(raw.get("platform_support", "unverified")),
    )


def load_open_vocabulary_profile(path: str | Path) -> OpenVocabularyProfile:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise OpenVocabularyContractError("profile schema_version must equal 1")
    profile_id = raw.get("profile_id")
    platform = raw.get("platform")
    if not isinstance(profile_id, str) or not profile_id:
        raise OpenVocabularyContractError("profile_id must be non-empty")
    if platform not in {"pc", "rdk_s100"}:
        raise OpenVocabularyContractError("platform must be pc or rdk_s100")
    profile = OpenVocabularyProfile(
        profile_id=profile_id,
        platform=platform,
        soc_identity=(str(raw["soc_identity"]) if raw.get("soc_identity") else None),
        detector=_component(raw.get("detector", {}), "detector"),
        segmenter=_component(raw.get("segmenter", {}), "segmenter"),
        development_only=bool(raw.get("development_only", True)),
        journey6_evidence_allowed=bool(raw.get("journey6_evidence_allowed", False)),
        ground_truth_control_allowed=bool(raw.get("ground_truth_control_allowed", False)),
    )
    if profile.ground_truth_control_allowed:
        raise OpenVocabularyContractError("ground truth control must remain disabled")
    if profile.platform == "rdk_s100" and profile.soc_identity != "journey6p":
        raise OpenVocabularyContractError("RDK S100P must identify the Journey 6P SoC")
    if profile.development_only and profile.journey6_evidence_allowed:
        raise OpenVocabularyContractError(
            "development-only profile cannot claim Journey 6P board evidence"
        )
    return profile


def check_runtime_readiness(
    profile: OpenVocabularyProfile,
    *,
    artifact_root: str | Path | None,
    runtime_available: bool,
    source_revisions_locked: bool,
) -> RuntimeReadiness:
    missing: list[str] = []
    if not runtime_available:
        missing.append(f"{profile.platform}_runtime")
    if not source_revisions_locked and (
        profile.detector.source_revision_required
        or profile.segmenter.source_revision_required
    ):
        missing.append("source_revisions")
    root = Path(artifact_root) if artifact_root is not None else None
    for role, component in (
        ("detector", profile.detector),
        ("segmenter", profile.segmenter),
    ):
        if profile.platform == "rdk_s100" and component.platform_support != "official_s100":
            missing.append(f"{role}_s100_platform_support")
        if component.artifact_relative_path is None:
            missing.append(f"{role}_artifact_contract")
        elif root is None or not (root / component.artifact_relative_path).is_file():
            missing.append(f"{role}_artifact")
    unique = tuple(dict.fromkeys(missing))
    return RuntimeReadiness(
        ready=not unique,
        missing=unique,
        detail="ready" if not unique else "fail_closed_external_runtime_or_artifact_missing",
    )


def _validate_proposal(proposal: Proposal2D) -> None:
    if not proposal.proposal_id:
        raise OpenVocabularyContractError("proposal_id must be non-empty")
    normalized_source = proposal.source.strip().lower()
    if normalized_source in FORBIDDEN_CONTROL_SOURCES or any(
        token in normalized_source for token in ("ground_truth", "gazebo_truth")
    ):
        raise OpenVocabularyContractError(
            f"evaluation truth source is forbidden in control fusion: {proposal.source}"
        )
    if not 0.0 <= proposal.score <= 1.0:
        raise OpenVocabularyContractError("proposal score must be in [0, 1]")
    x1, y1, x2, y2 = proposal.bbox_xyxy
    if not (x2 > x1 and y2 > y1):
        raise OpenVocabularyContractError("proposal bbox must have positive area")
    if proposal.mask_area_px is not None and proposal.mask_area_px <= 0:
        raise OpenVocabularyContractError("mask_area_px must be positive")


def fuse_proposals(proposals: Iterable[Proposal2D]) -> tuple[FusedProposal, ...]:
    """Fuse rows sharing a proposal id without consulting hidden truth.

    DOSOD usually contributes label/box, EdgeSAM contributes mask area, and the
    RGB-D geometry path contributes depth.  The result remains actionable only
    as a proposal; downstream tracking and safety gates retain authority.
    """

    grouped: dict[str, list[Proposal2D]] = {}
    for proposal in proposals:
        _validate_proposal(proposal)
        grouped.setdefault(proposal.proposal_id, []).append(proposal)
    output: list[FusedProposal] = []
    for proposal_id in sorted(grouped):
        rows = grouped[proposal_id]
        primary = max(rows, key=lambda row: (row.score, row.source))
        mask_rows = [row for row in rows if row.mask_area_px is not None]
        depth_rows = [row for row in rows if row.depth_xyz_m is not None]
        output.append(
            FusedProposal(
                proposal_id=proposal_id,
                label=primary.label,
                score=primary.score,
                bbox_xyxy=primary.bbox_xyxy,
                sources=tuple(sorted({row.source for row in rows})),
                mask_area_px=(max(row.mask_area_px for row in mask_rows) if mask_rows else None),
                depth_xyz_m=(max(depth_rows, key=lambda row: row.score).depth_xyz_m if depth_rows else None),
            )
        )
    return tuple(output)
