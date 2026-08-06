"""G4 asset registry, procedural model generation, and summary helpers.

The G4 domain replaces the G3 single-color primitives with per-variant
programmatic texture PNGs referenced from real Gazebo model SDFs.  All
generation is deterministic: the same registry bytes produce the same model
directories, textures, and SHA-256 values, so the committed registry and the
generator can be verified byte-for-byte.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


SCHEMA_VERSION = 2
CLASS_ORDER = (
    "plastic_bottle",
    "metal_can",
    "paper_litter",
    "leaf_pile",
    "puddle",
)
SPLITS = ("train", "val", "test")

TARGET_MIN_COUNTS = {
    "plastic_bottle": {"total": 30, "train": 20, "val": 5, "test": 5},
    "metal_can": {"total": 30, "train": 20, "val": 5, "test": 5},
    "paper_litter": {"total": 46, "train": 30, "val": 8, "test": 8},
    "leaf_pile": {"total": 30, "train": 20, "val": 5, "test": 5},
    "puddle": {"total": 30, "train": 20, "val": 5, "test": 5},
}
HARD_NEGATIVE_MIN_COUNTS = {"total": 80, "train": 50, "val": 15, "test": 15}

REQUIRED_PAPER_TAXONOMIES = (
    "road_marking_fragment",
    "light_paver",
    "paver_joint",
    "paper_like_road_patch",
    "crack",
    "shadow_edge",
    "plastic_label",
    "packaging_graphic",
    "flat_stone",
    "light_leaf_litter",
    "reflective_area",
    "vehicle_white_gray_structure",
    "curb_corner",
    "truncated_object_edge",
)

LEAF_REQUIRED_AREA_ATTRIBUTES = (
    "thickness_high",
    "thickness_low",
    "density_sparse",
    "density_dense",
    "condition_wet",
    "condition_dry",
    "leaf_shape_round",
    "leaf_shape_pointed",
    "leaf_shape_lobed",
    "shadow_coverage",
    "partial_occlusion",
    "background_leaf_discrimination",
)

PUDDLE_REQUIRED_AREA_ATTRIBUTES = (
    "contour_irregular",
    "contour_round",
    "reflectivity_shallow",
    "reflectivity_deep",
    "ground_asphalt",
    "ground_concrete",
    "ground_paving",
    "ground_soil",
    "shadow_coverage",
    "wet_non_puddle",
    "specular_highlight",
    "partial_occlusion",
    "boundary_blur",
)

# geometry_family -> (kind, values) for every variant referenced by the
# registry.  True physical sizes in metres; no G1-style scale factor.
GEOMETRY_PARAMS = {
    "bottle_cylinder_smooth": ("cylinder", (0.035, 0.17)),
    "bottle_cylinder_ribbed": ("cylinder", (0.038, 0.19)),
    "bottle_dented": ("cylinder", (0.036, 0.16)),
    "bottle_crushed": ("cylinder", (0.045, 0.12)),
    "bottle_flattened": ("box", (0.10, 0.05, 0.16)),
    "bottle_labeled": ("cylinder", (0.037, 0.18)),
    "can_cylinder_short": ("cylinder", (0.04, 0.10)),
    "can_cylinder_tall": ("cylinder", (0.038, 0.14)),
    "can_dented": ("cylinder", (0.041, 0.11)),
    "can_crushed": ("cylinder", (0.048, 0.09)),
    "can_flattened": ("box", (0.10, 0.04, 0.12)),
    "can_labeled": ("cylinder", (0.039, 0.13)),
    "paper_sheet_folded": ("box", (0.20, 0.13, 0.008)),
    "paper_sheet_torn": ("box", (0.18, 0.12, 0.006)),
    "paper_strip_long": ("box", (0.24, 0.09, 0.006)),
    "paper_crumpled": ("ellipsoid", (0.09, 0.08, 0.05)),
    "paper_sheet_wide": ("box", (0.22, 0.16, 0.007)),
    "paper_wad": ("ellipsoid", (0.07, 0.07, 0.045)),
    "leaf_pile_sparse": ("ellipsoid", (0.22, 0.13, 0.015)),
    "leaf_pile_dense": ("ellipsoid", (0.30, 0.19, 0.035)),
    "leaf_pile_windrow": ("box", (0.35, 0.12, 0.025)),
    "leaf_pile_round": ("ellipsoid", (0.26, 0.22, 0.03)),
    "leaf_pile_long": ("box", (0.30, 0.10, 0.02)),
    "leaf_pile_mixed": ("ellipsoid", (0.28, 0.18, 0.028)),
    "puddle_round": ("ellipsoid", (0.42, 0.30, 0.004)),
    "puddle_wide": ("ellipsoid", (0.55, 0.34, 0.005)),
    "puddle_irregular": ("ellipsoid", (0.45, 0.28, 0.004)),
    "puddle_long": ("box", (0.60, 0.22, 0.004)),
    "puddle_split": ("ellipsoid", (0.40, 0.26, 0.004)),
    "puddle_shallow": ("ellipsoid", (0.34, 0.24, 0.003)),
    "flat_patch": ("ellipsoid", (0.18, 0.14, 0.005)),
    "thin_slab": ("box", (0.20, 0.14, 0.006)),
    "stripe_strip": ("box", (0.28, 0.08, 0.004)),
    "paver_block": ("box", (0.22, 0.16, 0.03)),
    "paver_joint_line": ("box", (0.30, 0.04, 0.003)),
    "crack_line": ("box", (0.26, 0.03, 0.002)),
    "curb_corner_block": ("box", (0.18, 0.18, 0.12)),
    "object_edge_block": ("box", (0.16, 0.16, 0.10)),
    "bollard_cone": ("cylinder", (0.06, 0.25)),
    "obstacle_box": ("box", (0.20, 0.14, 0.10)),
    "cardboard_box": ("box", (0.22, 0.16, 0.10)),
    "metal_grill_patch": ("box", (0.24, 0.18, 0.015)),
    "gravel_cluster": ("ellipsoid", (0.20, 0.15, 0.02)),
    "reflective_patch": ("ellipsoid", (0.22, 0.16, 0.003)),
    "shadow_patch": ("ellipsoid", (0.40, 0.25, 0.001)),
}

# material_family -> (roughness, metalness, specular_scale)
MATERIAL_FAMILIES = {
    "pet_plastic_glossy": (0.28, 0.0, 0.32),
    "pet_plastic_matte": (0.66, 0.0, 0.06),
    "hdpe_opaque": (0.72, 0.0, 0.05),
    "aluminum_can_glossy": (0.34, 0.85, 0.28),
    "steel_can_matte": (0.58, 0.62, 0.10),
    "paper_glossy": (0.45, 0.0, 0.18),
    "paper_matte": (0.85, 0.0, 0.05),
    "kraft_cardboard": (0.90, 0.0, 0.0),
    "dry_leaf": (0.80, 0.0, 0.05),
    "wet_leaf": (0.48, 0.0, 0.34),
    "water_shallow": (0.14, 0.0, 0.48),
    "water_deep": (0.08, 0.0, 0.68),
    "asphalt": (0.92, 0.0, 0.02),
    "concrete": (0.88, 0.0, 0.02),
    "paint_mark": (0.55, 0.0, 0.10),
    "paver_stone": (0.85, 0.0, 0.05),
    "metal_grill": (0.50, 0.70, 0.15),
    "obstacle_plastic": (0.65, 0.0, 0.08),
    "cardboard": (0.90, 0.0, 0.0),
    "rubber_shadow": (0.95, 0.0, 0.0),
    "gravel": (0.93, 0.0, 0.02),
}

TEXTURE_FAMILIES = {
    "ribbed_bands",
    "rings",
    "grid_lines",
    "crosshatch",
    "speckle",
    "veins",
    "label_patch",
    "crushed_folds",
    "stripe_pattern",
    "crack_network",
    "paver_grid",
    "gradient_shadow",
    "mottled",
    "patchwork",
    "reflective_ripple",
    "dashed_line",
    "text_blocks",
    "edge_fade",
    "leaf_mosaic",
    "puddle_ripple",
    "clean_stone",
}


def _geometry_xml(kind: str, values: tuple[float, ...]) -> str:
    if kind == "cylinder":
        return (
            f"<cylinder><radius>{values[0]:.4f}</radius>"
            f"<length>{values[1]:.4f}</length></cylinder>"
        )
    if kind == "box":
        return (
            f"<box><size>{values[0]:.4f} {values[1]:.4f} "
            f"{values[2]:.4f}</size></box>"
        )
    return (
        f"<ellipsoid><radii>{values[0]:.4f} {values[1]:.4f} "
        f"{values[2]:.4f}</radii></ellipsoid>"
    )


def _variant_sha256(class_id: str, variant: dict) -> str:
    canonical = {
        "class_id": class_id,
        "id": variant["id"],
        "geometry_family": variant["geometry_family"],
        "material_family": variant["material_family"],
        "texture_family": variant["texture_family"],
        "palette": list(variant["palette"]),
        "split_eligibility": list(variant["split_eligibility"]),
        "source": variant["source"],
        "license": variant["license"],
    }
    if "area_attributes" in variant:
        canonical["area_attributes"] = sorted(variant["area_attributes"])
    payload = json.dumps(canonical, sort_keys=True, ensure_ascii=True)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _negative_sha256(negative: dict) -> str:
    canonical = {
        "id": negative["id"],
        "taxonomy": negative["taxonomy"],
        "geometry_family": negative["geometry_family"],
        "material_family": negative["material_family"],
        "texture_family": negative["texture_family"],
        "split_eligibility": list(negative["split_eligibility"]),
        "source": negative["source"],
        "license": negative["license"],
    }
    payload = json.dumps(canonical, sort_keys=True, ensure_ascii=True)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _rgb(palette_rgb: dict, name: str) -> tuple[int, int, int]:
    value = palette_rgb[name]
    return (int(value[0]), int(value[1]), int(value[2]))


def _class_variants(registry: dict, class_id: str) -> list[dict]:
    spec = registry["classes"][class_id]
    if isinstance(spec, dict):
        return spec.get("variants", [])
    return spec


def _texture_png(
    texture_family: str,
    palette_rgb: dict,
    palette_names: list[str],
    seed: int,
    size: tuple[int, int] = (256, 256),
) -> np.ndarray:
    """Deterministically render a visible texture for one model variant."""
    width, height = size
    base = np.asarray(_rgb(palette_rgb, palette_names[0]), dtype=np.uint8)
    accent = np.asarray(_rgb(palette_rgb, palette_names[1 % len(palette_names)]), dtype=np.uint8)
    third = np.asarray(_rgb(palette_rgb, palette_names[2 % len(palette_names)]), dtype=np.uint8)
    rng = np.random.default_rng(seed)
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    canvas[:] = base
    yy, xx = np.mgrid[0:height, 0:width]
    if texture_family == "ribbed_bands":
        bands = (xx // 18 + (yy // 14) % 2) % 3
        canvas[bands == 1] = accent
        canvas[bands == 2] = third
    elif texture_family == "rings":
        radius = np.hypot(xx - width / 2, yy - height / 2)
        canvas[(radius.astype(int) // 14) % 2 == 1] = accent
        canvas[(radius.astype(int) // 14) % 3 == 2] = third
    elif texture_family == "grid_lines":
        canvas[((xx % 42) < 4) | ((yy % 42) < 4)] = accent
    elif texture_family == "crosshatch":
        canvas[((xx + yy) % 38 < 5) | ((xx - yy) % 38 < 5)] = accent
    elif texture_family == "speckle":
        mask = rng.random((height, width)) > 0.78
        canvas[mask] = accent
        mask2 = rng.random((height, width)) > 0.94
        canvas[mask2] = third
    elif texture_family == "veins":
        for _ in range(9):
            start = (int(rng.integers(0, width)), int(rng.integers(0, height)))
            end = (int(rng.integers(0, width)), int(rng.integers(0, height)))
            cv2.line(canvas, start, end, tuple(accent.tolist()), 1, cv2.LINE_AA)
            cv2.line(canvas, start, end, tuple(third.tolist()), 1, cv2.LINE_AA)
    elif texture_family == "label_patch":
        label_x, label_y = width // 5, height // 3
        label_w, label_h = width - 2 * label_x, height // 3
        canvas[label_y : label_y + label_h, label_x : label_x + label_w] = accent
        for row in range(4):
            y = label_y + 14 + row * 22
            blocks = rng.integers(18, 40, size=6)
            x = label_x + 12
            for block in blocks:
                cv2.rectangle(
                    canvas, (int(x), y), (int(x + block), y + 8), tuple(third.tolist()), -1
                )
                x += block + 8
    elif texture_family == "crushed_folds":
        for _ in range(14):
            cx = int(rng.integers(0, width))
            cy = int(rng.integers(0, height))
            pts = np.column_stack(
                (
                    cx + rng.normal(0, 24, 5),
                    cy + rng.normal(0, 24, 5),
                )
            ).astype(np.int32)
            cv2.fillPoly(canvas, [pts], tuple(accent.tolist()))
    elif texture_family == "stripe_pattern":
        stripes = ((xx * 3 + yy) // 22) % 4
        canvas[stripes == 1] = accent
        canvas[stripes == 3] = third
    elif texture_family == "crack_network":
        for _ in range(16):
            x, y = int(rng.integers(0, width)), int(rng.integers(0, height))
            points = [(x, y)]
            for _ in range(5):
                x += int(rng.integers(-18, 19))
                y += int(rng.integers(-14, 15))
                points.append((x, y))
            cv2.polylines(
                canvas,
                [np.asarray(points, dtype=np.int32)],
                False,
                tuple(accent.tolist()),
                1,
                cv2.LINE_AA,
            )
    elif texture_family == "paver_grid":
        canvas[((xx % 34) < 3) | ((yy % 34) < 3)] = accent
        canvas[(rng.random((height, width)) > 0.86) & ((xx % 34) > 3) & ((yy % 34) > 3)] = third
    elif texture_family == "gradient_shadow":
        gradient = np.linspace(0.35, 1.0, height, dtype=np.float32)[:, None]
        canvas = np.clip(canvas.astype(np.float32) * gradient, 0, 255).astype(np.uint8)
        canvas[(rng.random((height, width)) > 0.92)] = accent
    elif texture_family == "mottled":
        for _ in range(26):
            cx = int(rng.integers(0, width))
            cy = int(rng.integers(0, height))
            axes = (int(rng.integers(5, 20)), int(rng.integers(5, 20)))
            color = tuple(accent.tolist()) if rng.random() < 0.55 else tuple(third.tolist())
            cv2.ellipse(canvas, (cx, cy), axes, 0, 0, 360, color, -1)
    elif texture_family == "patchwork":
        for _ in range(22):
            x0 = int(rng.integers(0, width - 30))
            y0 = int(rng.integers(0, height - 30))
            w = int(rng.integers(10, 34))
            h = int(rng.integers(10, 34))
            color = (
                tuple(accent.tolist())
                if rng.random() < 0.5
                else tuple(third.tolist())
            )
            cv2.rectangle(canvas, (x0, y0), (x0 + w, y0 + h), color, -1)
    elif texture_family == "reflective_ripple":
        wave = np.sin(xx * 0.28 + np.sin(yy * 0.22) * 2.4)
        canvas[wave > 0.45] = accent
        highlight = np.hypot(xx - width * 0.38, yy - height * 0.42) < 30
        canvas[highlight] = np.clip(
            canvas[highlight].astype(np.int16) + 55, 0, 255
        ).astype(np.uint8)
    elif texture_family == "dashed_line":
        for y in range(0, height, 22):
            for x in range(8, width, 34):
                cv2.rectangle(
                    canvas, (x, y), (x + 14, y + 9), tuple(accent.tolist()), -1
                )
    elif texture_family == "text_blocks":
        for row in range(7):
            y = 12 + row * 34
            x = 8
            while x < width - 24:
                block = int(rng.integers(8, 30))
                cv2.rectangle(
                    canvas, (x, y), (x + block, y + 14), tuple(accent.tolist()), -1
                )
                cv2.rectangle(
                    canvas, (x, y), (x + block, y + 14), tuple(third.tolist()), 1
                )
                x += block + 9
    elif texture_family == "edge_fade":
        dist = np.minimum(np.minimum(xx, width - xx), np.minimum(yy, height - yy))
        factor = np.clip(dist / 55.0, 0.25, 1.0).astype(np.float32)
        canvas = np.clip(canvas.astype(np.float32) * factor[..., None], 0, 255).astype(np.uint8)
    elif texture_family == "leaf_mosaic":
        for _ in range(70):
            cx = int(rng.integers(0, width))
            cy = int(rng.integers(0, height))
            axes = (int(rng.integers(6, 16)), int(rng.integers(3, 8)))
            angle = int(rng.integers(0, 180))
            color = (
                tuple(accent.tolist())
                if rng.random() < 0.6
                else tuple(third.tolist())
            )
            cv2.ellipse(canvas, (cx, cy), axes, angle, 0, 360, color, -1)
    elif texture_family == "puddle_ripple":
        radius = np.hypot(xx - width / 2, yy - height / 2)
        canvas[(radius.astype(int) // 9) % 3 == 1] = accent
        highlight = (radius.astype(int) % 13) < 2
        canvas[highlight] = np.clip(
            canvas[highlight].astype(np.int16) + 40, 0, 255
        ).astype(np.uint8)
    else:  # clean_stone and any fallback
        noise = rng.normal(0, 12, (height, width, 1)).astype(np.float32)
        canvas = np.clip(canvas.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        canvas[(rng.random((height, width)) > 0.9)] = accent
    noise = rng.normal(0, 4.0, canvas.shape).astype(np.float32)
    canvas = np.clip(canvas.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return canvas


def load_g4_asset_registry(path: str | Path) -> dict:
    """Validate the G4 registry schema, counts, metadata, and SHA fields."""
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if payload.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"unsupported G4 registry schema: {payload.get('schema_version')}")
    classes = payload.get("classes", {})
    if set(classes) != set(CLASS_ORDER):
        raise ValueError("G4 registry must define exactly the five target classes")
    palette_rgb = payload.get("palette_rgb", {})
    if not palette_rgb:
        raise ValueError("G4 registry palette_rgb is empty")
    seen_ids: set[str] = set()
    for class_id in CLASS_ORDER:
        variants = _class_variants(payload, class_id)
        minimum = TARGET_MIN_COUNTS[class_id]
        counts = {split: 0 for split in SPLITS}
        for variant in variants:
            required = {
                "id",
                "geometry_family",
                "material_family",
                "texture_family",
                "palette",
                "split_eligibility",
                "source",
                "license",
                "sha256",
            }
            missing = required - set(variant)
            if missing:
                raise ValueError(f"incomplete G4 target variant in {class_id}: {sorted(missing)}")
            if variant["id"] in seen_ids:
                raise ValueError(f"duplicate G4 asset id {variant['id']}")
            seen_ids.add(variant["id"])
            split = variant["split_eligibility"]
            if split != [split[0]] or split[0] not in SPLITS:
                raise ValueError(f"invalid split_eligibility {split} for {variant['id']}")
            counts[split[0]] += 1
            if variant["geometry_family"] not in GEOMETRY_PARAMS:
                raise ValueError(f"unknown geometry_family {variant['geometry_family']}")
            if variant["material_family"] not in MATERIAL_FAMILIES:
                raise ValueError(f"unknown material_family {variant['material_family']}")
            if variant["texture_family"] not in TEXTURE_FAMILIES:
                raise ValueError(f"unknown texture_family {variant['texture_family']}")
            if not variant["palette"] or not all(name in palette_rgb for name in variant["palette"]):
                raise ValueError(f"invalid palette in {variant['id']}")
            expected = _variant_sha256(class_id, variant)
            if variant["sha256"] != expected:
                raise ValueError(f"sha256 mismatch for {variant['id']}")
            if class_id in {"leaf_pile", "puddle"}:
                attributes = variant.get("area_attributes", [])
                if not attributes:
                    raise ValueError(f"area variant {variant['id']} missing area_attributes")
        if len(variants) < minimum["total"]:
            raise ValueError(f"{class_id} needs at least {minimum['total']} variants")
        for split in SPLITS:
            if counts[split] < minimum[split]:
                raise ValueError(
                    f"{class_id} needs at least {minimum[split]} {split} variants, got {counts[split]}"
                )
    _validate_area_coverage(payload)
    _validate_hard_negatives(payload)
    return payload


def _validate_area_coverage(payload: dict) -> None:
    attributes_by_class = {
        "leaf_pile": LEAF_REQUIRED_AREA_ATTRIBUTES,
        "puddle": PUDDLE_REQUIRED_AREA_ATTRIBUTES,
    }
    for class_id, required in attributes_by_class.items():
        covered: dict[str, int] = {}
        for variant in _class_variants(payload, class_id):
            if variant["split_eligibility"][0] != "train":
                continue
            for attribute in variant.get("area_attributes", []):
                covered[attribute] = covered.get(attribute, 0) + 1
        missing = [attribute for attribute in required if covered.get(attribute, 0) < 4]
        if missing:
            raise ValueError(
                f"{class_id} train area coverage missing attributes with <4 variants: {missing}"
            )


def _validate_hard_negatives(payload: dict) -> None:
    negatives = payload.get("hard_negatives", [])
    minimum = HARD_NEGATIVE_MIN_COUNTS
    if len(negatives) < minimum["total"]:
        raise ValueError(
            f"hard negatives need at least {minimum['total']} families, got {len(negatives)}"
        )
    counts = {split: 0 for split in SPLITS}
    taxonomy_by_split: dict[str, set[str]] = {split: set() for split in SPLITS}
    seen_ids: set[str] = set()
    for negative in negatives:
        required = {
            "id",
            "taxonomy",
            "geometry_family",
            "material_family",
            "texture_family",
            "split_eligibility",
            "source",
            "license",
            "sha256",
        }
        missing = required - set(negative)
        if missing:
            raise ValueError(f"incomplete hard negative: {sorted(missing)}")
        if negative["id"] in seen_ids:
            raise ValueError(f"duplicate hard negative id {negative['id']}")
        seen_ids.add(negative["id"])
        split = negative["split_eligibility"]
        if split != [split[0]] or split[0] not in SPLITS:
            raise ValueError(f"invalid split_eligibility {split} for {negative['id']}")
        counts[split[0]] += 1
        taxonomy_by_split[split[0]].add(negative["taxonomy"])
        if negative["geometry_family"] not in GEOMETRY_PARAMS:
            raise ValueError(f"unknown negative geometry_family {negative['geometry_family']}")
        if negative["material_family"] not in MATERIAL_FAMILIES:
            raise ValueError(f"unknown negative material_family {negative['material_family']}")
        if negative["texture_family"] not in TEXTURE_FAMILIES:
            raise ValueError(f"unknown negative texture_family {negative['texture_family']}")
        expected = _negative_sha256(negative)
        if negative["sha256"] != expected:
            raise ValueError(f"sha256 mismatch for hard negative {negative['id']}")
    for split in SPLITS:
        if counts[split] < minimum[split]:
            raise ValueError(
                f"hard negatives need at least {minimum[split]} {split} families, got {counts[split]}"
            )
    missing_taxonomies = sorted(set(REQUIRED_PAPER_TAXONOMIES) - taxonomy_by_split["train"])
    if missing_taxonomies:
        raise ValueError(
            "train hard negatives must cover every required paper taxonomy; missing: "
            + ", ".join(missing_taxonomies)
        )


def g4_registry_summary(registry: dict) -> dict:
    """Per-split variant/family counts and the SHA-256 manifest."""
    target_counts = {
        class_id: {
            split: sum(
                variant["split_eligibility"][0] == split
                for variant in _class_variants(registry, class_id)
            )
            for split in SPLITS
        }
        for class_id in CLASS_ORDER
    }
    negatives = registry.get("hard_negatives", [])
    negative_split_counts = {
        split: sum(item["split_eligibility"][0] == split for item in negatives)
        for split in SPLITS
    }
    shas = {
        **{
            variant["id"]: variant["sha256"]
            for class_id in CLASS_ORDER
            for variant in _class_variants(registry, class_id)
        },
        **{item["id"]: item["sha256"] for item in negatives},
    }
    geometry_families = {
        variant["geometry_family"]
        for class_id in CLASS_ORDER
        for variant in _class_variants(registry, class_id)
    } | {item["geometry_family"] for item in negatives}
    texture_families = {
        variant["texture_family"]
        for class_id in CLASS_ORDER
        for variant in _class_variants(registry, class_id)
    } | {item["texture_family"] for item in negatives}
    material_families = {
        variant["material_family"]
        for class_id in CLASS_ORDER
        for variant in _class_variants(registry, class_id)
    } | {item["material_family"] for item in negatives}
    area_attributes = {
        class_id: sorted(
            {
                attribute
                for variant in _class_variants(registry, class_id)
                for attribute in variant.get("area_attributes", [])
            }
        )
        for class_id in ("leaf_pile", "puddle")
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "target_variant_counts_by_class": target_counts,
        "target_variant_total": sum(
            sum(counts.values()) for counts in target_counts.values()
        ),
        "hard_negative_family_counts_by_split": negative_split_counts,
        "hard_negative_family_total": len(negatives),
        "geometry_family_count": len(geometry_families),
        "material_family_count": len(material_families),
        "texture_family_count": len(texture_families),
        "area_attributes_by_class": area_attributes,
        "required_paper_taxonomy_count": len(REQUIRED_PAPER_TAXONOMIES),
        "sha256_manifest_count": len(shas),
        "sha256_manifest": shas,
    }


def _model_sdf(
    name: str,
    kind: str,
    values: tuple[float, ...],
    texture_name: str,
    material_family: str,
    label: int,
    texture_uri: str | None = None,
    rgba: str = "0.50 0.50 0.50 1",
) -> str:
    geometry = _geometry_xml(kind, values)
    roughness, metalness, _ = MATERIAL_FAMILIES[material_family]
    albedo = texture_uri or f"textures/{texture_name}"
    return (
        '<?xml version="1.0"?>\n'
        f'<sdf version="1.9">\n'
        f'  <model name="{name}">\n'
        "    <static>true</static>\n"
        '    <link name="body">\n'
        '      <visual name="visual">\n'
        f"        <geometry>{geometry}</geometry>\n"
        f"        <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>"
        "<pbr><metal>"
        f"<albedo_map>{albedo}</albedo_map>"
        f"<roughness>{roughness}</roughness><metalness>{metalness}</metalness>"
        "</metal></pbr></material></visual>\n"
        f'      <collision name="collision"><geometry>{geometry}</geometry></collision>\n'
        "    </link>\n"
        f'    <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">'
        f"<label>{label}</label></plugin>\n"
        "  </model>\n"
        "</sdf>\n"
    )


def _model_config(name: str, description: str) -> str:
    return (
        '<?xml version="1.0"?>\n'
        "<model>\n"
        f"  <name>{name}</name>\n"
        "  <version>1.0</version>\n"
        '  <sdf version="1.9">model.sdf</sdf>\n'
        "  <author><name>TZcup Project</name></author>\n"
        f"  <description>{description}</description>\n"
        "</model>\n"
    )


def _seed_for(registry_path: Path, asset_id: str) -> int:
    digest = hashlib.sha256(
        f"{registry_path.name}:{asset_id}".encode("utf-8")
    ).hexdigest()
    return int(digest[:16], 16)


def write_g4_assets(registry_path: str | Path, output_dir: str | Path) -> dict:
    """Write model.sdf, model.config, and a procedural texture PNG per variant."""
    registry = load_g4_asset_registry(registry_path)
    root = Path(output_dir)
    root.mkdir(parents=True, exist_ok=True)
    registry_path = Path(registry_path)
    generated: list[dict] = []
    for class_id, semantic_label in zip(CLASS_ORDER, range(1, len(CLASS_ORDER) + 1)):
        for variant in _class_variants(registry, class_id):
            kind, values = GEOMETRY_PARAMS[variant["geometry_family"]]
            model_dir = root / variant["id"]
            textures_dir = model_dir / "textures"
            textures_dir.mkdir(parents=True, exist_ok=True)
            texture_name = f"texture_{variant['id']}.png"
            texture_path = textures_dir / texture_name
            texture = _texture_png(
                variant["texture_family"],
                registry["palette_rgb"],
                variant["palette"],
                _seed_for(registry_path, variant["id"]),
            )
            cv2.imwrite(str(texture_path), cv2.cvtColor(texture, cv2.COLOR_RGB2BGR))
            color = registry["palette_rgb"][variant["palette"][0]]
            rgba = " ".join(f"{channel / 255.0:.4f}" for channel in color) + " 1"
            sdf = _model_sdf(
                variant["id"],
                kind,
                values,
                texture_name,
                variant["material_family"],
                semantic_label,
                rgba=rgba,
            )
            (model_dir / "model.sdf").write_text(sdf, encoding="utf-8")
            (model_dir / "model.config").write_text(
                _model_config(variant["id"], f"G4 {class_id} target variant"),
                encoding="utf-8",
            )
            generated.append(
                {
                    "asset_id": variant["id"],
                    "class_id": class_id,
                    "semantic_label": semantic_label,
                    "split": variant["split_eligibility"][0],
                    "geometry_family": variant["geometry_family"],
                    "material_family": variant["material_family"],
                    "texture_family": variant["texture_family"],
                    "texture_png_sha256": hashlib.sha256(
                        texture_path.read_bytes()
                    ).hexdigest(),
                    "model_sdf_sha256": hashlib.sha256(
                        (model_dir / "model.sdf").read_bytes()
                    ).hexdigest(),
                    "registry_sha256": variant["sha256"],
                }
            )
    for negative in registry["hard_negatives"]:
        kind, values = GEOMETRY_PARAMS[negative["geometry_family"]]
        model_dir = root / negative["id"]
        textures_dir = model_dir / "textures"
        textures_dir.mkdir(parents=True, exist_ok=True)
        texture_name = f"texture_{negative['id']}.png"
        texture_path = textures_dir / texture_name
        texture = _texture_png(
            negative["texture_family"],
            registry["palette_rgb"],
            ["gray", "stone", "dark_gray"],
            _seed_for(registry_path, negative["id"]),
        )
        cv2.imwrite(str(texture_path), cv2.cvtColor(texture, cv2.COLOR_RGB2BGR))
        palette_names = ["red", "green", "blue", "yellow", "teal", "orange", "brown", "white"]
        palette_name = palette_names[_seed_for(registry_path, negative["id"]) % len(palette_names)]
        color = registry["palette_rgb"][palette_name]
        rgba = " ".join(f"{channel / 255.0:.4f}" for channel in color) + " 1"
        sdf = _model_sdf(
            negative["id"],
            kind,
            values,
            texture_name,
            negative["material_family"],
            0,
            rgba=rgba,
        )
        (model_dir / "model.sdf").write_text(sdf, encoding="utf-8")
        (model_dir / "model.config").write_text(
            _model_config(
                negative["id"], f"G4 hard negative ({negative['taxonomy']})"
            ),
            encoding="utf-8",
        )
        generated.append(
            {
                "asset_id": negative["id"],
                "taxonomy": negative["taxonomy"],
                "semantic_label": 0,
                "split": negative["split_eligibility"][0],
                "geometry_family": negative["geometry_family"],
                "material_family": negative["material_family"],
                "texture_family": negative["texture_family"],
                "texture_png_sha256": hashlib.sha256(
                    texture_path.read_bytes()
                ).hexdigest(),
                "model_sdf_sha256": hashlib.sha256(
                    (model_dir / "model.sdf").read_bytes()
                ).hexdigest(),
                "registry_sha256": negative["sha256"],
            }
        )
    summary = {
        **g4_registry_summary(registry),
        "registry_sha256": hashlib.sha256(
            registry_path.read_bytes()
        ).hexdigest(),
        "generated_assets": generated,
    }
    (root / "g4_generated_asset_manifest.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    return summary


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    summary = write_g4_assets(args.registry, args.output_dir)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
