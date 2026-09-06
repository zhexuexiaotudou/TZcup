"""Mission-mode-specific Nav2 safety-source materialization."""

from __future__ import annotations

from typing import Any


class Nav2ModeConfigError(ValueError):
    """Raised when the base Nav2 safety configuration cannot be narrowed safely."""


def configure_collision_monitor_sources(
    nav2: dict[str, Any], *, mission_mode: str
) -> None:
    """Keep mapping collision monitoring bound to its available 2D scan.

    The formal mapping launch deliberately disables the high-bandwidth sensor
    runtime.  Its Collision Monitor must therefore not retain a MID360 source
    that has no publisher.  Saved-map cleaning retains the source set from the
    formal high-bandwidth base configuration unchanged.
    """
    if mission_mode not in {"mapping", "cleaning"}:
        raise Nav2ModeConfigError(f"unsupported mission mode: {mission_mode!r}")
    try:
        parameters = nav2["collision_monitor"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise Nav2ModeConfigError("collision_monitor parameters are required") from exc
    if not isinstance(parameters, dict):
        raise Nav2ModeConfigError("collision_monitor parameters must be a mapping")
    sources = parameters.get("observation_sources")
    if not isinstance(sources, list) or "scan" not in sources:
        raise Nav2ModeConfigError("collision_monitor requires an enabled scan source")
    scan = parameters.get("scan")
    if not isinstance(scan, dict) or scan.get("enabled") is not True:
        raise Nav2ModeConfigError("collision_monitor scan source must remain enabled")
    if mission_mode == "mapping":
        # Do not merely disable the dead source: removing it prevents Nav2
        # from waiting on a 3D topic that this launch intentionally omits.
        parameters["observation_sources"] = ["scan"]
        parameters.pop("mid360", None)
