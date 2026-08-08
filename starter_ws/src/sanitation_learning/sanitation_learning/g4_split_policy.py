"""Explicit G4 split-role policy.

Development may only read:

- ``train``
- ``train_world_holdout`` (deterministic in-domain holdout)
- ``val`` (cross-world validation)
- ``D1`` .. ``D5`` (shift diagnostics)

The old ``test`` role is contaminated diagnostic evidence only and is
represented in every report and CLI as ``legacy_G4_D6_diagnostic``.  It can
never contribute to training, threshold selection, checkpoint selection,
hard-negative mining or screening pass/fail.  ``G5_SEALED_FINAL`` is a sealed
final set that development code must never load.
"""

from __future__ import annotations

import warnings


DEVELOPMENT_ROLES = (
    "train",
    "train_world_holdout",
    "val",
    "D1",
    "D2",
    "D3",
    "D4",
    "D5",
)
SHIFT_DIAGNOSTIC_ROLES = ("D1", "D2", "D3", "D4", "D5")
LEGACY_DIAGNOSTIC_ROLE = "legacy_G4_D6_diagnostic"
SEALED_FINAL_ROLE = "G5_SEALED_FINAL"
ALL_ROLES = (
    DEVELOPMENT_ROLES + (LEGACY_DIAGNOSTIC_ROLE, SEALED_FINAL_ROLE)
)

_LEGACY_WARNING = (
    "split role 'test' is contaminated diagnostic evidence only; it is "
    "renamed to legacy_G4_D6_diagnostic and must never be used for training, "
    "threshold selection, checkpoint selection, hard-negative mining, or "
    "screening pass/fail"
)


def canonical_role_name(split: str) -> str:
    """Return the canonical role for a raw split name.

    ``test`` is renamed (with a mandatory warning) to
    ``legacy_G4_D6_diagnostic``.  Unknown splits raise ``ValueError`` so a
    misspelled role can never silently join development.
    """
    if not isinstance(split, str):
        raise ValueError(f"split must be a string, got {split!r}")
    name = split.strip()
    if name == "test":
        warnings.warn(_LEGACY_WARNING, stacklevel=2)
        return LEGACY_DIAGNOSTIC_ROLE
    if name in ALL_ROLES:
        return name
    raise ValueError(f"unknown G4 split role: {split!r}")


def is_development_role(split: str) -> bool:
    return canonical_role_name(split) in DEVELOPMENT_ROLES


def is_legacy_diagnostic(split: str) -> bool:
    return canonical_role_name(split) == LEGACY_DIAGNOSTIC_ROLE


def is_sealed_final(split: str) -> bool:
    return canonical_role_name(split) == SEALED_FINAL_ROLE


def partition_rows(rows) -> dict[str, list[dict]]:
    """Partition rows into canonical roles, renaming ``test`` with a warning."""
    result: dict[str, list[dict]] = {role: [] for role in ALL_ROLES}
    for row in rows:
        role = canonical_role_name(row.get("split", ""))
        result[role].append(row)
    return result


def assert_development_rows(rows, purpose: str) -> None:
    """Fail closed when rows intended for development contain forbidden roles.

    Raises ``ValueError`` on legacy diagnostic or sealed final rows (including
    raw ``test`` labels before renaming) so they can never influence
    training, thresholding, checkpoint selection, hard-negative mining or
    screening decisions.
    """
    forbidden: list[dict] = []
    for row in rows:
        raw = row.get("split", "")
        role = (
            LEGACY_DIAGNOSTIC_ROLE
            if raw == "test"
            else raw
        )
        if role in (LEGACY_DIAGNOSTIC_ROLE, SEALED_FINAL_ROLE):
            forbidden.append(row)
        elif role not in DEVELOPMENT_ROLES:
            raise ValueError(
                f"{purpose} received row with undeclared split role {raw!r}"
            )
    if forbidden:
        raise ValueError(
            f"{purpose} must never read legacy_G4_D6_diagnostic or "
            f"G5_SEALED_FINAL data; found {len(forbidden)} forbidden rows"
        )


def screening_decision(
    gates: dict[str, bool],
    *,
    legacy_gates: dict[str, bool] | None = None,
) -> dict:
    """Compute a screening decision that is invariant to legacy diagnostics.

    Only ``gates`` (development evidence) may influence pass/fail.  A separate
    ``legacy_gates`` dictionary is recorded for transparency but changing it
    cannot change the decision.
    """
    passed = bool(gates) and all(gates.values())
    return {
        "P4_SCREENING_PASS": passed,
        "AUTO_05R_PASS": passed,
        "AUTO_05R_BLOCKED": not passed,
        "legacy_G4_D6_diagnostic_included_in_decision": False,
        "G5_sealed_final_included_in_decision": False,
        "gates": dict(gates),
        "legacy_diagnostic_gates": dict(legacy_gates or {}),
    }


__all__ = [
    "ALL_ROLES",
    "DEVELOPMENT_ROLES",
    "LEGACY_DIAGNOSTIC_ROLE",
    "SEALED_FINAL_ROLE",
    "SHIFT_DIAGNOSTIC_ROLES",
    "assert_development_rows",
    "canonical_role_name",
    "is_development_role",
    "is_legacy_diagnostic",
    "is_sealed_final",
    "partition_rows",
    "screening_decision",
]
