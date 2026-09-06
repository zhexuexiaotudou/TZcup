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


def test_mapping_mode_removes_unavailable_mid360_from_collision_monitor():
    nav2 = _nav2()

    configure_collision_monitor_sources(nav2, mission_mode="mapping")

    parameters = nav2["collision_monitor"]["ros__parameters"]
    assert parameters["observation_sources"] == ["scan"]
    assert parameters["scan"]["topic"] == "/scan/navigation"
    assert parameters["scan"]["enabled"] is True
    assert "mid360" not in parameters


def test_cleaning_mode_retains_the_high_bandwidth_collision_sources_exactly():
    nav2 = _nav2()
    expected = deepcopy(nav2)

    configure_collision_monitor_sources(nav2, mission_mode="cleaning")

    assert nav2 == expected


@pytest.mark.parametrize("mode", ("", "mapping_then_cleaning", "hidden"))
def test_unknown_mode_fails_closed(mode: str):
    with pytest.raises(Nav2ModeConfigError, match="unsupported mission mode"):
        configure_collision_monitor_sources(_nav2(), mission_mode=mode)
