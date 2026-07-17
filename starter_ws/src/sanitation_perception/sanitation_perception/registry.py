from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import uuid

import yaml


TARGET_TYPES = {"discrete", "area"}
POLICIES = {"spot_clean", "local_coverage"}
REQUIRED_CLASSES = {
    "plastic_bottle",
    "metal_can",
    "paper_litter",
    "leaf_pile",
    "puddle",
}


@dataclass(frozen=True)
class RegistryEntry:
    model_name: str
    uuid: str
    class_id: str
    target_type: str
    policy: str
    pickable: bool
    size_m: tuple[float, float, float]
    is_target: bool


class RegistryError(ValueError):
    pass


class GarbageRegistry:
    def __init__(self, payload: dict):
        self.payload = payload
        self._validate()
        self.namespace = uuid.UUID(payload["uuid_namespace"])
        self.class_order = tuple(payload["class_order"])
        self.entries = {
            name: RegistryEntry(
                model_name=name,
                uuid=str(spec["uuid"]),
                class_id=str(spec["class_id"]),
                target_type=str(spec["target_type"]),
                policy=str(spec["policy"]),
                pickable=bool(spec["pickable"]),
                size_m=tuple(float(value) for value in spec["size_m"]),
                is_target=True,
            )
            for name, spec in payload["models"].items()
        }
        self.negative_models = frozenset(payload["negative_models"])
        canonical = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        self.sha256 = hashlib.sha256(canonical.encode("utf-8")).hexdigest()

    @classmethod
    def load(cls, path: str | Path) -> "GarbageRegistry":
        payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise RegistryError("registry root must be a mapping")
        return cls(payload)

    def _validate(self) -> None:
        required = {
            "schema_version",
            "registry_version",
            "uuid_namespace",
            "class_order",
            "models",
            "negative_models",
        }
        missing = required - set(self.payload)
        if missing:
            raise RegistryError(f"registry missing fields: {sorted(missing)}")
        if self.payload["schema_version"] != 1:
            raise RegistryError("unsupported registry schema_version")
        try:
            namespace = uuid.UUID(str(self.payload["uuid_namespace"]))
        except ValueError as exc:
            raise RegistryError("invalid uuid_namespace") from exc
        class_order = self.payload["class_order"]
        if not isinstance(class_order, list) or class_order[0] != "background":
            raise RegistryError("class_order must start with background")
        if len(class_order) != len(set(class_order)):
            raise RegistryError("class_order contains duplicates")
        if not REQUIRED_CLASSES.issubset(class_order):
            raise RegistryError("class_order does not cover all Stage5A classes")
        models = self.payload["models"]
        negatives = self.payload["negative_models"]
        if set(models) & set(negatives):
            raise RegistryError("models and negative_models must be disjoint")
        seen_uuids: set[str] = set()
        seen_classes: set[str] = set()
        for name, spec in models.items():
            for field in ("uuid", "class_id", "target_type", "policy", "pickable", "size_m"):
                if field not in spec:
                    raise RegistryError(f"{name} missing {field}")
            expected_uuid = str(uuid.uuid5(namespace, name))
            if str(spec["uuid"]) != expected_uuid:
                raise RegistryError(f"{name} UUID is not stable UUIDv5")
            if spec["target_type"] not in TARGET_TYPES:
                raise RegistryError(f"{name} has invalid target_type")
            if spec["policy"] not in POLICIES:
                raise RegistryError(f"{name} has invalid policy")
            if spec["class_id"] not in class_order:
                raise RegistryError(f"{name} class is absent from class_order")
            if len(spec["size_m"]) != 3 or any(float(v) <= 0 for v in spec["size_m"]):
                raise RegistryError(f"{name} size_m must contain three positive values")
            if spec["uuid"] in seen_uuids:
                raise RegistryError("registry UUID collision")
            seen_uuids.add(spec["uuid"])
            seen_classes.add(spec["class_id"])
        if seen_classes != REQUIRED_CLASSES:
            raise RegistryError("target models must cover each Stage5A class exactly")
        for name, spec in negatives.items():
            if set(spec) != {"uuid", "class_id"}:
                raise RegistryError(f"negative model {name} may only define uuid and class_id")
            if str(spec["uuid"]) != str(uuid.uuid5(namespace, name)):
                raise RegistryError(f"negative model {name} UUID is not stable UUIDv5")
            if spec["uuid"] in seen_uuids:
                raise RegistryError("registry UUID collision")
            seen_uuids.add(spec["uuid"])

    def resolve(self, model_name: str) -> RegistryEntry | None:
        """Resolve exact simulator identity. Prefix/substring inference is forbidden."""
        return self.entries.get(model_name)

    def is_negative(self, model_name: str) -> bool:
        return model_name in self.negative_models

    def require_complete_scene(self, model_names: set[str]) -> None:
        unknown = model_names - set(self.entries) - self.negative_models
        if unknown:
            raise RegistryError(f"scene contains unregistered models: {sorted(unknown)}")
