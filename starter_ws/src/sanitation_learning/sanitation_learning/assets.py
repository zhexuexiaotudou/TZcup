from __future__ import annotations

import json
from pathlib import Path

import yaml


REQUIRED_CLASSES = {
    "plastic_bottle",
    "metal_can",
    "paper_litter",
    "leaf_pile",
    "puddle",
}


def load_asset_registry(path: str | Path) -> dict:
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if payload.get("schema_version") != 1:
        raise ValueError("unsupported asset registry schema")
    if set(payload.get("classes", {})) != REQUIRED_CLASSES:
        raise ValueError("asset registry must define exactly the five Stage5B classes")
    variant_ids: set[str] = set()
    palette_usage: dict[str, set[str]] = {}
    for class_id, spec in payload["classes"].items():
        variants = spec.get("variants", [])
        if len(variants) < 6:
            raise ValueError(f"{class_id} needs at least six variants")
        for variant in variants:
            required = {"id", "geometry", "texture", "palette"}
            if required - set(variant):
                raise ValueError(f"incomplete variant in {class_id}")
            if variant["id"] in variant_ids:
                raise ValueError(f"duplicate asset id {variant['id']}")
            variant_ids.add(variant["id"])
            for color in variant["palette"]:
                if color not in payload["palette_rgb"]:
                    raise ValueError(f"unknown palette {color}")
                palette_usage.setdefault(color, set()).add(class_id)
    if not any(len(classes) >= 3 for classes in palette_usage.values()):
        raise ValueError("palettes must overlap across classes to prevent color coding")
    negatives = set(payload.get("negative_assets", []))
    required_negatives = {
        "bottle_like_reusable_cone", "can_like_bollard", "red_obstacle",
        "green_obstacle", "blue_obstacle", "cardboard", "fixed_bin",
        "reflective_patch", "shadow", "wet_asphalt",
        "leaves_outside_target_region", "robot_self_pixels",
    }
    if not required_negatives.issubset(negatives):
        raise ValueError("hard-negative registry is incomplete")
    return payload


def registry_summary(payload: dict) -> dict:
    return {
        "schema_version": 1,
        "class_variant_counts": {
            class_id: len(spec["variants"])
            for class_id, spec in payload["classes"].items()
        },
        "target_variant_count": sum(
            len(spec["variants"]) for spec in payload["classes"].values()
        ),
        "negative_asset_count": len(payload["negative_assets"]),
        "fixed_class_color_encoding": False,
        "license": "Apache-2.0",
        "external_files_used": False,
    }


def _geometry_for(class_id: str, variant_index: int) -> tuple[str, tuple[float, ...]]:
    if class_id == "plastic_bottle":
        return "cylinder", (0.035 + 0.003 * (variant_index % 3), 0.16 + 0.02 * (variant_index % 2))
    if class_id == "metal_can":
        return "cylinder", (0.04 + 0.004 * (variant_index % 2), 0.10 + 0.015 * (variant_index % 3))
    if class_id == "paper_litter":
        return "box", (0.18 + 0.02 * (variant_index % 2), 0.11 + 0.01 * (variant_index % 3), 0.008)
    if class_id == "leaf_pile":
        return "sphere", (0.25 + 0.03 * (variant_index % 3), 0.16 + 0.02 * (variant_index % 2), 0.025)
    return "sphere", (0.36 + 0.04 * (variant_index % 3), 0.22 + 0.02 * (variant_index % 2), 0.006)


def _geometry_xml(kind: str, values: tuple[float, ...]) -> str:
    if kind == "cylinder":
        return f"<cylinder><radius>{values[0]:.4f}</radius><length>{values[1]:.4f}</length></cylinder>"
    if kind == "box":
        return f"<box><size>{values[0]:.4f} {values[1]:.4f} {values[2]:.4f}</size></box>"
    return f"<ellipsoid><radii>{values[0]:.4f} {values[1]:.4f} {values[2]:.4f}</radii></ellipsoid>"


def write_gazebo_assets(registry_path: str | Path, output: str | Path) -> dict:
    registry = load_asset_registry(registry_path)
    root = Path(output)
    root.mkdir(parents=True, exist_ok=True)
    generated = []
    for class_id, spec in registry["classes"].items():
        for index, variant in enumerate(spec["variants"]):
            color = registry["palette_rgb"][variant["palette"][0]]
            rgba = " ".join(f"{channel / 255.0:.4f}" for channel in color) + " 1"
            kind, values = _geometry_for(class_id, index)
            geometry = _geometry_xml(kind, values)
            model_dir = root / variant["id"]
            model_dir.mkdir(parents=True, exist_ok=True)
            sdf = (
                "<?xml version=\"1.0\"?>\n"
                "<sdf version=\"1.9\">\n"
                f"  <model name=\"{variant['id']}\">\n"
                "    <static>true</static>\n"
                "    <link name=\"body\">\n"
                f"      <visual name=\"visual\"><geometry>{geometry}</geometry>"
                f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>"
                "<roughness>0.65</roughness></material></visual>\n"
                f"      <collision name=\"collision\"><geometry>{geometry}</geometry></collision>\n"
                "    </link>\n"
                "  </model>\n"
                "</sdf>\n"
            )
            path = model_dir / "model.sdf"
            path.write_text(sdf, encoding="utf-8")
            generated.append({"class_id": class_id, "asset_id": variant["id"], "path": path.as_posix()})
    summary = {**registry_summary(registry), "generated_assets": generated}
    (root / "generated_asset_manifest.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    return summary
