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
    if mission_mode != "mapping":
        return
    if scan.get("topic") != "/scan/navigation" or scan.get("type") != "scan":
        raise Nav2ModeConfigError(
            "mapping collision_monitor requires the canonical enabled scan source"
        )
    costmap_layers: list[tuple[str, dict[str, Any]]] = []
    for costmap_name in ("local_costmap", "global_costmap"):
        try:
            layer = nav2[costmap_name][costmap_name]["ros__parameters"][
                "obstacle_layer"
            ]
        except (KeyError, TypeError) as exc:
            raise Nav2ModeConfigError(
                f"{costmap_name} obstacle_layer parameters are required"
            ) from exc
        if not isinstance(layer, dict):
            raise Nav2ModeConfigError(
                f"{costmap_name} obstacle_layer parameters must be a mapping"
            )
        costmap_layers.append((costmap_name, layer))
    if not high_bandwidth_sensor_runtime:
        # Do not merely disable the dead source: removing it prevents Nav2
        # from waiting on a 3D topic omitted by scan-only mapping.
        parameters["observation_sources"] = ["scan"]
        parameters.pop("mid360", None)
        for _, layer in costmap_layers:
            layer["observation_sources"] = "scan"
            layer.pop("mid360", None)
    else:
        mid360 = parameters.get("mid360")
        if (
            "mid360" not in sources
            or not isinstance(mid360, dict)
            or mid360.get("topic") != "/sensors/lidar_3d/points"
            or mid360.get("type") != "pointcloud"
            or mid360.get("enabled") is not True
        ):
            raise Nav2ModeConfigError(
                "high-bandwidth mapping requires an enabled mid360 source"
            )
        for costmap_name, layer in costmap_layers:
            scan_source, mid360_source = layer.get("scan"), layer.get("mid360")
            if (
                layer.get("enabled") is not True
                or layer.get("observation_sources") != "scan mid360"
                or not isinstance(scan_source, dict)
                or scan_source.get("topic") != "/scan/navigation"
                or scan_source.get("data_type") != "LaserScan"
                or not isinstance(mid360_source, dict)
                or mid360_source.get("topic") != "/sensors/lidar_3d/points"
                or mid360_source.get("data_type") != "PointCloud2"
            ):
                raise Nav2ModeConfigError(
                    f"high-bandwidth mapping requires valid {costmap_name} obstacle sources"
                )
