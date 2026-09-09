"""Journey 6 target selection and fail-closed runtime contracts."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


TARGET_FAMILY = "journey6"
AUTO = "auto"
SUPPORTED_MARCHES = frozenset({"nash-e", "nash-m", "nash-p"})
SUPPORTED_PROFILES = frozenset(
    {
        "journey6_generic",
        "journey6_nash_e",
        "journey6_nash_m",
        "journey6_nash_p",
    }
)
MARCH_TO_PROFILE = {
    "nash-e": "journey6_nash_e",
    "nash-m": "journey6_nash_m",
    "nash-p": "journey6_nash_p",
}


@dataclass(frozen=True)
class Journey6Target:
    target_family: str = TARGET_FAMILY
    target_sku: str = AUTO
    target_march: str = AUTO
    profile: str = "journey6_generic"

    @classmethod
    def from_mapping(cls, payload: Mapping[str, object]) -> "Journey6Target":
        target = cls(
            target_family=str(payload.get("target_family", TARGET_FAMILY)),
            target_sku=str(payload.get("target_sku", AUTO)),
            target_march=str(payload.get("target_march", AUTO)),
            profile=str(payload.get("profile", "journey6_generic")),
        )
        target.validate()
        return target

    def validate(self) -> None:
        if self.target_family != TARGET_FAMILY:
            raise ValueError("Journey 6 bundle cannot target an RDK/J5 family")
        if self.target_march != AUTO and self.target_march not in SUPPORTED_MARCHES:
            raise ValueError(f"unsupported or unverified Journey 6 march: {self.target_march}")
        if self.profile not in SUPPORTED_PROFILES:
            raise ValueError(f"unsupported Journey 6 profile: {self.profile}")
        if self.profile != "journey6_generic":
            expected = MARCH_TO_PROFILE.get(self.target_march)
            if expected != self.profile:
                raise ValueError("profile and target_march do not match")

    def resolve(self, inventory: Mapping[str, object]) -> "Journey6Target":
        """Resolve auto fields only from a real board/official-SDK inventory."""
        family = str(inventory.get("target_family", ""))
        sku = str(inventory.get("target_sku", ""))
        march = str(inventory.get("target_march", ""))
        source = str(inventory.get("fact_source", ""))
        if family != TARGET_FAMILY:
            raise ValueError("inventory does not identify a Journey 6 target")
        if not sku or sku == AUTO:
            raise ValueError("inventory has no resolved Journey 6 SKU")
        if march not in SUPPORTED_MARCHES:
            raise ValueError("inventory has no supported official Journey 6 march")
        if source not in {"board_inventory", "official_j6_sdk"}:
            raise ValueError("SKU/march must come from the board or official J6 SDK")
        if self.target_sku != AUTO and self.target_sku != sku:
            raise ValueError("configured SKU does not match inventory")
        if self.target_march != AUTO and self.target_march != march:
            raise ValueError("configured march does not match inventory")
        resolved = Journey6Target(
            target_family=TARGET_FAMILY,
            target_sku=sku,
            target_march=march,
            profile=MARCH_TO_PROFILE[march],
        )
        resolved.validate()
        return resolved


__all__ = [
    "AUTO",
    "MARCH_TO_PROFILE",
    "SUPPORTED_MARCHES",
    "SUPPORTED_PROFILES",
    "TARGET_FAMILY",
    "Journey6Target",
]
