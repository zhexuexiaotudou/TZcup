from copy import deepcopy

import pytest

from sanitation_formal_campus_integration.nav2_mode_config import (
    Nav2ModeConfigError,
    configure_collision_monitor_sources,
)


def _nav2() -> dict:
    return {
        "collision_monitor": {
            "ros__parameters": {
                "observation_sources": ["scan", "mid360"],
                "scan": {"topic": "/scan/navigation", "enabled": True},
                "mid360": {
                    "topic": "/sensors/lidar_3d/points",
                    "enabled": True,
                },
            }
        }
    }


def test_scan_only_mapping_removes_unavailable_mid360_from_collision_monitor():
    nav2 = _nav2()

    configure_collision_monitor_sources(
        nav2, mission_mode="mapping", high_bandwidth_sensor_runtime=False
    )

    parameters = nav2["collision_monitor"]["ros__parameters"]
    assert parameters["observation_sources"] == ["scan"]
    assert parameters["scan"]["topic"] == "/scan/navigation"
    assert parameters["scan"]["enabled"] is True
    assert "mid360" not in parameters


def test_high_bandwidth_mapping_retains_and_requires_mid360_collision_source():
    nav2 = _nav2()
    expected = deepcopy(nav2)

    configure_collision_monitor_sources(
        nav2, mission_mode="mapping", high_bandwidth_sensor_runtime=True
    )

    assert nav2 == expected
    broken = _nav2()
    broken["collision_monitor"]["ros__parameters"]["mid360"]["enabled"] = False
    with pytest.raises(Nav2ModeConfigError, match="enabled mid360"):
        configure_collision_monitor_sources(
            broken, mission_mode="mapping", high_bandwidth_sensor_runtime=True
        )
    missing = _nav2()
    missing["collision_monitor"]["ros__parameters"]["observation_sources"] = ["scan"]
    with pytest.raises(Nav2ModeConfigError, match="enabled mid360"):
        configure_collision_monitor_sources(
            missing, mission_mode="mapping", high_bandwidth_sensor_runtime=True
        )


@pytest.mark.parametrize("high_bandwidth_sensor_runtime", (False, True))
def test_cleaning_mode_retains_the_high_bandwidth_collision_sources_exactly(
    high_bandwidth_sensor_runtime: bool,
):
    nav2 = _nav2()
    expected = deepcopy(nav2)

    configure_collision_monitor_sources(
        nav2,
        mission_mode="cleaning",
        high_bandwidth_sensor_runtime=high_bandwidth_sensor_runtime,
    )

    assert nav2 == expected


@pytest.mark.parametrize("mode", ("", "mapping_then_cleaning", "hidden"))
def test_unknown_mode_fails_closed(mode: str):
    with pytest.raises(Nav2ModeConfigError, match="unsupported mission mode"):
        configure_collision_monitor_sources(
            _nav2(), mission_mode=mode, high_bandwidth_sensor_runtime=False
        )


def test_high_bandwidth_runtime_must_be_bool():
    with pytest.raises(Nav2ModeConfigError, match="must be bool"):
        configure_collision_monitor_sources(  # type: ignore[arg-type]
            _nav2(), mission_mode="mapping", high_bandwidth_sensor_runtime="true"
        )
