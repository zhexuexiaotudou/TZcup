"""Fail-closed data-boundary policy for Detector Data Recovery V4.

The module intentionally contains no dataset reader.  Callers must pass the
declared dataset identifiers through these guards before opening annotations,
metrics, manifests, or pixels.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping


PROTOCOL = "DETECTOR-DATA-RECOVERY-V4"
G7_DATASET_ID = "G7_DETECTOR_DEVELOPMENT"
G5_DATASET_ID = "G5_SEALED_FINAL"
G5_V2_DATASET_ID = "G5_V2_SEALED_FINAL"
G6_DATASET_ID = "G6_DEVELOPMENT_OPRV3_V1"


class DDRV4BoundaryError(ValueError):
    """Raised before a prohibited DDRV4 data access can occur."""


def require_ddrv4_selection_inputs(dataset_ids: Iterable[str]) -> tuple[str, ...]:
    """Validate that route selection uses only the new G7 development pack."""

    declared = tuple(str(item) for item in dataset_ids)
    if not declared:
        raise DDRV4BoundaryError("DDRV4 selection requires an explicit dataset id")
    forbidden = {G5_DATASET_ID, G5_V2_DATASET_ID, G6_DATASET_ID}
    overlap = sorted(set(declared) & forbidden)
    if overlap:
        raise DDRV4BoundaryError(
            "DDRV4 route/checkpoint/threshold selection cannot use: "
            + ", ".join(overlap)
        )
    unexpected = sorted(set(declared) - {G7_DATASET_ID})
    if unexpected:
        raise DDRV4BoundaryError(
            "DDRV4 selection input is not the registered G7 pack: "
            + ", ".join(unexpected)
        )
    return declared


def require_sealed_access(dataset_id: str, freeze: Mapping[str, object] | None) -> None:
    """Authorize a sealed-final access without reading any sealed content."""

    if dataset_id == G5_DATASET_ID:
        raise DDRV4BoundaryError("G5_SEALED_FINAL is consumed and can never reopen")
    if dataset_id != G5_V2_DATASET_ID:
        raise DDRV4BoundaryError(f"unknown DDRV4 sealed dataset: {dataset_id}")
    if not isinstance(freeze, Mapping):
        raise DDRV4BoundaryError("G5_V2 access denied before a DDRV4 freeze")
    release = freeze.get("release_boundary")
    sealed = freeze.get("sealed_final")
    valid = (
        freeze.get("protocol") == "DDRV4-07"
        and freeze.get("status") == "FROZEN_X86_DDR_V4"
        and isinstance(release, Mapping)
        and release.get("DDRV4_X86_DEV_PASS") is True
        and isinstance(sealed, Mapping)
        and sealed.get("dataset") == G5_V2_DATASET_ID
        and sealed.get("maximum_accesses") == 1
    )
    if not valid:
        raise DDRV4BoundaryError("G5_V2 access denied before a valid DDRV4-07 freeze")


def validate_data_boundary(payload: Mapping[str, object]) -> None:
    """Validate the compact DDRV4-00 data-boundary evidence."""

    expected = {
        "G5_STATUS": "CONSUMED_FINAL",
        "G5_CAN_BE_USED_FOR_TUNING": False,
        "G6_STATUS": "DEVELOPMENT_HISTORY",
        "G6_CAN_BE_USED_FOR_NEW_ROUTE_TUNING": False,
        "G5_V2_STATUS": "SEALED_NOT_OPENED",
        "G5_V2_CAN_BE_USED_FOR_TUNING": False,
        "DDRV4_G7_DEVELOPMENT_AUTHORIZED": True,
    }
    for key, value in expected.items():
        if payload.get(key) != value:
            raise DDRV4BoundaryError(f"invalid DDRV4 data boundary: {key}")
    require_ddrv4_selection_inputs(payload.get("selection_dataset_ids", ()))


__all__ = [
    "DDRV4BoundaryError",
    "G5_DATASET_ID",
    "G5_V2_DATASET_ID",
    "G6_DATASET_ID",
    "G7_DATASET_ID",
    "PROTOCOL",
    "require_ddrv4_selection_inputs",
    "require_sealed_access",
    "validate_data_boundary",
]
