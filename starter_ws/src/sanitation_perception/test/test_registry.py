from pathlib import Path

import pytest
import yaml

from sanitation_perception.registry import GarbageRegistry, RegistryError


REGISTRY = Path(__file__).parents[1] / "config" / "garbage_registry.yaml"


def test_registry_exact_identity_and_coverage():
    registry = GarbageRegistry.load(REGISTRY)
    assert len(registry.entries) == 5
    assert len(registry.sha256) == 64
    assert registry.resolve("trash_bottle_01").class_id == "plastic_bottle"
    assert registry.resolve("trash_bin_obstacle") is None
    assert registry.is_negative("trash_bin_obstacle")
    assert registry.resolve("prefix_trash_bottle_01_suffix") is None


def test_registry_rejects_unknown_scene_and_unstable_uuid():
    registry = GarbageRegistry.load(REGISTRY)
    with pytest.raises(RegistryError, match="unregistered"):
        registry.require_complete_scene({"trash_bottle_01", "mystery_trash"})
    payload = yaml.safe_load(REGISTRY.read_text(encoding="utf-8"))
    payload["models"]["trash_bottle_01"]["uuid"] = "00000000-0000-0000-0000-000000000000"
    with pytest.raises(RegistryError, match="stable UUIDv5"):
        GarbageRegistry(payload)
