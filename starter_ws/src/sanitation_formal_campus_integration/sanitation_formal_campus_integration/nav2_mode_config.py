"""Mission-mode-specific Nav2 safety-source materialization."""

from __future__ import annotations

from typing import Any


class Nav2ModeConfigError(ValueError):
    """Raised when the base Nav2 safety configuration cannot be narrowed safely."""


def configure_collision_monitor_sources(
    nav2: dict[str, Any], *, mission_mode: str, high_bandwidth_sensor_runtime: bool
) -> None:
    """Keep collision monitoring bound to sources enabled by this lifecycle.

    Scan-only mapping must remove MID360 because it has no publisher. Mapping
    with the explicit high-bandwidth opt-in must retain and verify MID360;
    saved-map cleaning retains its existing formal source set.
    """
    if mission_mode not in {"mapping", "cleaning"}:
        raise Nav2ModeConfigError(f"unsupported mission mode: {mission_mode!r}")
    if not isinstance(high_bandwidth_sensor_runtime, bool):
        raise Nav2ModeConfigError("high_bandwidth_sensor_runtime must be bool")
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
    if mission_mode == "mapping" and not high_bandwidth_sensor_runtime:
        # Do not merely disable the dead source: removing it prevents Nav2
        # from waiting on a 3D topic omitted by scan-only mapping.
        parameters["observation_sources"] = ["scan"]
        parameters.pop("mid360", None)
    elif mission_mode == "mapping":
        mid360 = parameters.get("mid360")
        if "mid360" not in sources or not isinstance(mid360, dict) or mid360.get("enabled") is not True:
            raise Nav2ModeConfigError(
                "high-bandwidth mapping requires an enabled mid360 source"
            )
