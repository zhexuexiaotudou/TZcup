#!/usr/bin/env python3
"""Deterministically generate the committed G4 asset registry YAML.

The generated file must match `starter_ws/src/sanitation_learning/config/
g4_asset_registry.yaml` byte-for-byte.  SHA-256 values are content hashes of
the canonical variant descriptors, so a modified registry fails validation.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))

from sanitation_learning.g4_assets import (  # noqa: E402
    REQUIRED_PAPER_TAXONOMIES,
    _negative_sha256,
    _variant_sha256,
)


PALETTE_RGB = {
    "teal": [38, 142, 146],
    "blue": [54, 94, 168],
    "red": [176, 54, 58],
    "green": [68, 132, 74],
    "yellow": [190, 157, 48],
    "rust": [154, 76, 48],
    "stone": [112, 118, 126],
    "white": [222, 222, 214],
    "gray": [128, 130, 128],
    "dark_gray": [72, 74, 76],
    "orange": [214, 122, 38],
    "brown": [122, 82, 46],
    "leaf_green": [96, 122, 52],
    "leaf_amber": [168, 122, 38],
    "leaf_brown": [140, 92, 40],
    "water_blue": [58, 108, 140],
    "water_dark": [42, 72, 96],
    "water_light": [92, 140, 156],
    "asphalt": [58, 60, 62],
    "concrete": [148, 148, 144],
    "paver": [168, 158, 148],
}


CLASS_TEMPLATES = {
    "plastic_bottle": {
        "split_sizes": [20, 5, 5],
        "combos": [
            ("bottle_cylinder_smooth", "pet_plastic_glossy", "ribbed_bands"),
            ("bottle_cylinder_ribbed", "pet_plastic_matte", "grid_lines"),
            ("bottle_dented", "hdpe_opaque", "speckle"),
            ("bottle_crushed", "pet_plastic_matte", "crushed_folds"),
            ("bottle_flattened", "hdpe_opaque", "crosshatch"),
            ("bottle_labeled", "pet_plastic_glossy", "label_patch"),
        ],
        "palettes": [
            ["teal", "white"],
            ["red", "stone"],
            ["blue", "yellow"],
            ["green", "rust"],
            ["white", "blue"],
            ["yellow", "teal"],
            ["orange", "gray"],
            ["brown", "white"],
        ],
    },
    "metal_can": {
        "split_sizes": [20, 5, 5],
        "combos": [
            ("can_cylinder_short", "aluminum_can_glossy", "rings"),
            ("can_cylinder_tall", "steel_can_matte", "label_patch"),
            ("can_dented", "aluminum_can_glossy", "speckle"),
            ("can_crushed", "steel_can_matte", "crushed_folds"),
            ("can_flattened", "aluminum_can_glossy", "stripe_pattern"),
            ("can_labeled", "steel_can_matte", "text_blocks"),
        ],
        "palettes": [
            ["teal", "rust"],
            ["red", "white"],
            ["green", "blue"],
            ["yellow", "stone"],
            ["white", "green"],
            ["blue", "rust"],
            ["gray", "orange"],
            ["stone", "red"],
        ],
    },
    "paper_litter": {
        "split_sizes": [30, 8, 8],
        "combos": [
            ("paper_sheet_folded", "paper_matte", "grid_lines"),
            ("paper_sheet_torn", "paper_glossy", "text_blocks"),
            ("paper_strip_long", "paper_matte", "dashed_line"),
            ("paper_crumpled", "paper_glossy", "crushed_folds"),
            ("paper_sheet_wide", "paper_matte", "patchwork"),
            ("paper_wad", "kraft_cardboard", "speckle"),
        ],
        "palettes": [
            ["white", "blue"],
            ["stone", "red"],
            ["white", "green"],
            ["yellow", "rust"],
            ["gray", "teal"],
            ["white", "dark_gray"],
            ["stone", "brown"],
            ["white", "orange"],
        ],
    },
    "leaf_pile": {
        "split_sizes": [20, 5, 5],
        "combos": [
            ("leaf_pile_sparse", "dry_leaf", "leaf_mosaic"),
            ("leaf_pile_dense", "wet_leaf", "veins"),
            ("leaf_pile_windrow", "dry_leaf", "mottled"),
            ("leaf_pile_round", "wet_leaf", "leaf_mosaic"),
            ("leaf_pile_long", "dry_leaf", "speckle"),
            ("leaf_pile_mixed", "wet_leaf", "mottled"),
        ],
        "palettes": [
            ["leaf_green", "leaf_amber"],
            ["leaf_amber", "leaf_brown"],
            ["leaf_brown", "leaf_green"],
            ["leaf_green", "brown"],
            ["leaf_amber", "rust"],
            ["leaf_brown", "yellow"],
            ["leaf_green", "yellow"],
            ["leaf_amber", "green"],
        ],
        "area_attribute_pools": {
            "thickness": ["thickness_high", "thickness_low"],
            "density": ["density_sparse", "density_dense"],
            "shape": ["leaf_shape_round", "leaf_shape_pointed", "leaf_shape_lobed"],
            "condition": ["condition_wet", "condition_dry"],
            "scene": [
                "shadow_coverage",
                "partial_occlusion",
                "background_leaf_discrimination",
            ],
        },
    },
    "puddle": {
        "split_sizes": [20, 5, 5],
        "combos": [
            ("puddle_round", "water_shallow", "puddle_ripple"),
            ("puddle_wide", "water_deep", "reflective_ripple"),
            ("puddle_irregular", "water_shallow", "puddle_ripple"),
            ("puddle_long", "water_deep", "reflective_ripple"),
            ("puddle_split", "water_shallow", "edge_fade"),
            ("puddle_shallow", "water_deep", "puddle_ripple"),
        ],
        "palettes": [
            ["water_blue", "water_light"],
            ["water_dark", "water_blue"],
            ["water_light", "water_dark"],
            ["asphalt", "water_dark"],
            ["concrete", "water_light"],
            ["water_blue", "white"],
            ["water_dark", "stone"],
            ["water_light", "blue"],
        ],
        "area_attribute_pools": {
            "contour": ["contour_irregular", "contour_round"],
            "reflectivity": ["reflectivity_shallow", "reflectivity_deep"],
            "ground": [
                "ground_asphalt",
                "ground_concrete",
                "ground_paving",
                "ground_soil",
            ],
            "scene": [
                "shadow_coverage",
                "wet_non_puddle",
                "specular_highlight",
                "partial_occlusion",
                "boundary_blur",
            ],
        },
    },
}


# Each required paper taxonomy gets 3 train + 1 val + 1 test family.  Extra
# taxonomies add one train family each; two of them additionally contribute a
# val and a test family.  Total: 14*5 + 8 + 2*3 = 84 families with
# 52 train / 16 val / 16 test, meeting the 50/15/15 hard minimums.
NEGATIVE_SPECS = {
    "road_marking_fragment": (
        ["flat_patch", "thin_slab", "stripe_strip"],
        ["paint_mark", "concrete", "asphalt"],
        ["dashed_line", "edge_fade", "crack_network"],
    ),
    "light_paver": (
        ["paver_block", "flat_patch", "thin_slab"],
        ["paver_stone", "concrete", "paint_mark"],
        ["paver_grid", "clean_stone", "patchwork"],
    ),
    "paver_joint": (
        ["paver_joint_line", "crack_line", "stripe_strip"],
        ["paver_stone", "asphalt", "concrete"],
        ["gradient_shadow", "edge_fade", "mottled"],
    ),
    "paper_like_road_patch": (
        ["thin_slab", "flat_patch", "paper_sheet_wide"],
        ["paper_matte", "concrete", "paint_mark"],
        ["text_blocks", "grid_lines", "patchwork"],
    ),
    "crack": (
        ["crack_line", "stripe_strip", "paver_joint_line"],
        ["asphalt", "concrete", "rubber_shadow"],
        ["crack_network", "gradient_shadow", "mottled"],
    ),
    "shadow_edge": (
        ["shadow_patch", "flat_patch", "thin_slab"],
        ["rubber_shadow", "asphalt", "concrete"],
        ["gradient_shadow", "edge_fade", "mottled"],
    ),
    "plastic_label": (
        ["thin_slab", "flat_patch", "paper_sheet_folded"],
        ["pet_plastic_glossy", "pet_plastic_matte", "hdpe_opaque"],
        ["label_patch", "text_blocks", "grid_lines"],
    ),
    "packaging_graphic": (
        ["paper_sheet_wide", "thin_slab", "cardboard_box"],
        ["paper_glossy", "kraft_cardboard", "paper_matte"],
        ["patchwork", "text_blocks", "label_patch"],
    ),
    "flat_stone": (
        ["flat_patch", "paver_block", "thin_slab"],
        ["paver_stone", "concrete", "gravel"],
        ["clean_stone", "mottled", "paver_grid"],
    ),
    "light_leaf_litter": (
        ["flat_patch", "shadow_patch", "leaf_pile_sparse"],
        ["dry_leaf", "wet_leaf", "gravel"],
        ["leaf_mosaic", "speckle", "mottled"],
    ),
    "reflective_area": (
        ["reflective_patch", "flat_patch", "shadow_patch"],
        ["water_deep", "water_shallow", "aluminum_can_glossy"],
        ["reflective_ripple", "puddle_ripple", "gradient_shadow"],
    ),
    "vehicle_white_gray_structure": (
        ["object_edge_block", "bollard_cone", "obstacle_box"],
        ["obstacle_plastic", "steel_can_matte", "concrete"],
        ["clean_stone", "grid_lines", "mottled"],
    ),
    "curb_corner": (
        ["curb_corner_block", "object_edge_block", "paver_block"],
        ["concrete", "paver_stone", "asphalt"],
        ["clean_stone", "paver_grid", "edge_fade"],
    ),
    "truncated_object_edge": (
        ["object_edge_block", "obstacle_box", "curb_corner_block"],
        ["obstacle_plastic", "steel_can_matte", "concrete"],
        ["edge_fade", "mottled", "grid_lines"],
    ),
    "bottle_like_cone": (
        ["bollard_cone", "obstacle_box", "flat_patch"],
        ["obstacle_plastic", "hdpe_opaque", "pet_plastic_matte"],
        ["ribbed_bands", "stripe_pattern", "speckle"],
    ),
    "can_like_bollard": (
        ["bollard_cone", "obstacle_box", "object_edge_block"],
        ["steel_can_matte", "aluminum_can_glossy", "obstacle_plastic"],
        ["rings", "stripe_pattern", "label_patch"],
    ),
    "red_obstacle": (
        ["obstacle_box", "bollard_cone", "object_edge_block"],
        ["obstacle_plastic", "hdpe_opaque", "paint_mark"],
        ["speckle", "ribbed_bands", "patchwork"],
    ),
    "green_obstacle": (
        ["obstacle_box", "bollard_cone", "object_edge_block"],
        ["obstacle_plastic", "hdpe_opaque", "paint_mark"],
        ["speckle", "grid_lines", "patchwork"],
    ),
    "blue_obstacle": (
        ["obstacle_box", "bollard_cone", "object_edge_block"],
        ["obstacle_plastic", "hdpe_opaque", "paint_mark"],
        ["speckle", "crosshatch", "patchwork"],
    ),
    "cardboard_box": (
        ["cardboard_box", "paper_sheet_wide", "thin_slab"],
        ["kraft_cardboard", "cardboard", "paper_matte"],
        ["patchwork", "text_blocks", "speckle"],
    ),
    "fixed_bin": (
        ["obstacle_box", "object_edge_block", "bollard_cone"],
        ["steel_can_matte", "obstacle_plastic", "concrete"],
        ["mottled", "grid_lines", "clean_stone"],
    ),
    "dark_metal_grill": (
        ["metal_grill_patch", "flat_patch", "thin_slab"],
        ["metal_grill", "steel_can_matte", "asphalt"],
        ["crosshatch", "grid_lines", "crack_network"],
    ),
    "gravel_patch": (
        ["gravel_cluster", "flat_patch", "shadow_patch"],
        ["gravel", "asphalt", "concrete"],
        ["mottled", "speckle", "clean_stone"],
    ),
    "water_stain_patch": (
        ["shadow_patch", "flat_patch", "reflective_patch"],
        ["water_shallow", "water_deep", "concrete"],
        ["puddle_ripple", "edge_fade", "reflective_ripple"],
    ),
}

EXTRA_TAXONOMY_SPLITS = {
    "bottle_like_cone": ["train"],
    "can_like_bollard": ["train"],
    "red_obstacle": ["train"],
    "green_obstacle": ["train"],
    "blue_obstacle": ["train"],
    "cardboard_box": ["train"],
    "fixed_bin": ["train"],
    "dark_metal_grill": ["train"],
    "gravel_patch": ["train", "val", "test"],
    "water_stain_patch": ["train", "val", "test"],
}

AREA_FIELD_KEYS = {
    "leaf_pile": ("thickness", "density", "shape", "condition", "scene"),
    "puddle": ("contour", "reflectivity", "ground", "scene"),
}


def _area_attributes(class_id: str, index: int) -> list[str]:
    if class_id not in ("leaf_pile", "puddle"):
        return []
    pools = CLASS_TEMPLATES[class_id]["area_attribute_pools"]
    return [
        pools[key][index % len(pools[key])]
        for key in AREA_FIELD_KEYS[class_id]
    ]


def build_target_variants() -> dict[str, list[dict]]:
    classes: dict[str, list[dict]] = {}
    for class_id, template in CLASS_TEMPLATES.items():
        split_sizes = template["split_sizes"]
        combos = template["combos"]
        palettes = template["palettes"]
        total = sum(split_sizes)
        boundaries = []
        cumulative = 0
        for size in split_sizes:
            cumulative += size
            boundaries.append(cumulative)
        split_names = ("train", "val", "test")
        variants = []
        for index in range(total):
            split = split_names[next(
                slot for slot, boundary in enumerate(boundaries) if index < boundary
            )]
            geometry, material, texture = combos[index % len(combos)]
            variant = {
                "id": f"g4_{class_id}_{index + 1:03d}",
                "geometry_family": geometry,
                "material_family": material,
                "texture_family": texture,
                "palette": palettes[index % len(palettes)],
                "split_eligibility": [split],
                "source": "project-authored procedural G4 model",
                "license": "Apache-2.0",
            }
            if class_id in ("leaf_pile", "puddle"):
                variant["area_attributes"] = _area_attributes(class_id, index)
            variant["sha256"] = _variant_sha256(class_id, variant)
            variants.append(variant)
        classes[class_id] = variants
    return classes


def build_hard_negatives() -> list[dict]:
    negatives = []
    for taxonomy in REQUIRED_PAPER_TAXONOMIES:
        geometry_pool, material_pool, texture_pool = NEGATIVE_SPECS[taxonomy]
        for index in range(5):
            split = ["train", "train", "train", "val", "test"][index]
            geometry = geometry_pool[index % len(geometry_pool)]
            material = material_pool[index % len(material_pool)]
            texture = texture_pool[index % len(texture_pool)]
            negative = {
                "id": f"g4_neg_{taxonomy}_{index + 1:02d}",
                "taxonomy": taxonomy,
                "geometry_family": geometry,
                "material_family": material,
                "texture_family": texture,
                "split_eligibility": [split],
                "source": "project-authored procedural G4 hard negative",
                "license": "Apache-2.0",
            }
            negative["sha256"] = _negative_sha256(negative)
            negatives.append(negative)
    for taxonomy, splits in EXTRA_TAXONOMY_SPLITS.items():
        geometry_pool, material_pool, texture_pool = NEGATIVE_SPECS[taxonomy]
        for slot, split in enumerate(splits):
            negative = {
                "id": f"g4_neg_{taxonomy}_{split}",
                "taxonomy": taxonomy,
                "geometry_family": geometry_pool[slot % len(geometry_pool)],
                "material_family": material_pool[slot % len(material_pool)],
                "texture_family": texture_pool[slot % len(texture_pool)],
                "split_eligibility": [split],
                "source": "project-authored procedural G4 hard negative",
                "license": "Apache-2.0",
            }
            negative["sha256"] = _negative_sha256(negative)
            negatives.append(negative)
    return negatives


def build_registry() -> dict:
    return {
        "schema_version": 2,
        "registry_version": "2026.08-g4-d1",
        "dataset_domain": "G4_deployment_aligned_vehicle_camera_gazebo",
        "license_manifest": "asset_license_manifest.json",
        "palette_rgb": PALETTE_RGB,
        "classes": build_target_variants(),
        "hard_negatives": build_hard_negatives(),
        "area_coverage": {
            "leaf_pile": {
                "required_attributes": sorted(
                    {
                        attribute
                        for pool in CLASS_TEMPLATES["leaf_pile"][
                            "area_attribute_pools"
                        ].values()
                        for attribute in pool
                    }
                ),
                "minimum_train_variants_per_attribute": 4,
            },
            "puddle": {
                "required_attributes": sorted(
                    {
                        attribute
                        for pool in CLASS_TEMPLATES["puddle"][
                            "area_attribute_pools"
                        ].values()
                        for attribute in pool
                    }
                ),
                "minimum_train_variants_per_attribute": 4,
            },
        },
        "split_contract": {
            "worlds": 12,
            "world_split_counts": {"train": 8, "val": 2, "test": 2},
            "scenes": 300,
            "scenes_per_world": 25,
            "frames": 3000,
            "frames_per_scene": 10,
            "test_used_for_model_selection": False,
            "G4_dataset_gate_pass": False,
            "full_capture_executed": False,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT
        / "starter_ws"
        / "src"
        / "sanitation_learning"
        / "config"
        / "g4_asset_registry.yaml",
    )
    args = parser.parse_args()
    payload = build_registry()
    text = yaml.safe_dump(
        payload,
        sort_keys=False,
        allow_unicode=True,
        default_flow_style=False,
        width=100,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(text)
    print(f"wrote {args.output} ({len(text.encode('utf-8'))} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
