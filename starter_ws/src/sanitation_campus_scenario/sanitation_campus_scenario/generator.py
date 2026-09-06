"""Pure-Python scenario generator with explicit truth separation."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import random
from typing import Any, Iterable
import xml.etree.ElementTree as ET

import yaml


GENERATOR_VERSION = "0.4.0"
EVALUATOR_NAMESPACE = "/evaluation/scenario_ground_truth"
SPLITS = ("train", "val", "hidden")
DIRT_CELL_COLUMNS = 10
DIRT_CELL_ROWS = 10
LEAF_VISUAL_COUNT = DIRT_CELL_COLUMNS * DIRT_CELL_ROWS
DUST_VISUAL_COUNT = DIRT_CELL_COLUMNS * DIRT_CELL_ROWS
PUDDLE_VISUAL_COUNT = DIRT_CELL_COLUMNS * DIRT_CELL_ROWS
CUBE_MATERIAL_DENSITY_KG_M3 = {
    "paperboard": 700.0,
    "PP": 900.0,
    "PET": 1380.0,
    "aluminum": 2700.0,
}


class GenerationError(RuntimeError):
    """Raised when a scenario cannot be generated without violating its contract."""


@dataclass(frozen=True)
class Pose2D:
    x_m: float
    y_m: float
    yaw_rad: float = 0.0


@dataclass(frozen=True)
class Asset:
    asset_id: str
    kind: str
    pose: Pose2D
    size_m: tuple[float, float, float]

    @property
    def half_extent(self) -> tuple[float, float]:
        return self.size_m[0] / 2.0, self.size_m[1] / 2.0


@dataclass(frozen=True)
class DirtPatch:
    object_id: str
    kind: str
    pose: Pose2D
    size_m: tuple[float, float]
    area_m2: float
    color_rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class Cube:
    object_id: str
    pose: Pose2D
    edge_m: float
    color_rgba: tuple[float, float, float, float]
    material: str
    density_kg_m3: float
    mass_kg: float


@dataclass(frozen=True)
class Pedestrian:
    object_id: str
    radius_m: float
    height_m: float
    speed_mps: float
    waypoints: tuple[tuple[float, float, float], ...]


@dataclass(frozen=True)
class EpisodeSeeds:
    layout: int
    dirt: int
    cubes: int
    pedestrians: int
    sensor: int


def _canonical_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def _sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def _dirt_visual_rng(patch: DirtPatch) -> random.Random:
    """Return a stable, public-appearance-only RNG for a dirt model."""

    digest = hashlib.sha256(f"dirt-visual-v2:{patch.object_id}".encode("utf-8")).digest()
    return random.Random(int.from_bytes(digest[:8], "big"))


def _set_visual_material(visual: ET.Element, rgba: tuple[float, float, float, float]) -> None:
    text = " ".join(_fmt(value) for value in rgba)
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = text
    ET.SubElement(material, "diffuse").text = text


def _add_dirt_visuals(link: ET.Element, patch: DirtPatch) -> None:
    """Render deterministic, redistributable dirt without truth-colour shortcuts.

    Geometry is procedural so formal episodes do not depend on proprietary
    textures.  The product still receives camera pixels only; patch kind and
    region truth remain evaluator-private.
    """

    rng = _dirt_visual_rng(patch)
    width, height = patch.size_m
    if patch.kind == "leaf":
        palette = (
            (0.24, 0.10, 0.018, 1.0),
            (0.42, 0.19, 0.025, 1.0),
            (0.68, 0.34, 0.035, 1.0),
            (0.78, 0.52, 0.06, 1.0),
            (0.22, 0.27, 0.035, 1.0),
        )
        for index in range(LEAF_VISUAL_COUNT):
            leaf_length = rng.uniform(0.055, 0.105)
            leaf_width = leaf_length * rng.uniform(0.28, 0.48)
            visual = ET.SubElement(link, "visual", {"name": f"leaf_{index:03d}"})
            ET.SubElement(visual, "pose").text = (
                f"{_fmt(rng.uniform(-0.47 * width, 0.47 * width))} "
                f"{_fmt(rng.uniform(-0.47 * height, 0.47 * height))} "
                f"{_fmt(0.0015 + 0.00035 * (index % 5))} 0 0 {_fmt(rng.uniform(-math.pi, math.pi))}"
            )
            geometry = ET.SubElement(visual, "geometry")
            polyline = ET.SubElement(geometry, "polyline")
            ET.SubElement(polyline, "height").text = "0.0025"
            for x, y in (
                (-0.5 * leaf_length, 0.0),
                (-0.28 * leaf_length, -0.38 * leaf_width),
                (0.0, -0.5 * leaf_width),
                (0.30 * leaf_length, -0.32 * leaf_width),
                (0.5 * leaf_length, 0.0),
                (0.30 * leaf_length, 0.32 * leaf_width),
                (0.0, 0.5 * leaf_width),
                (-0.28 * leaf_length, 0.38 * leaf_width),
            ):
                ET.SubElement(polyline, "point").text = f"{_fmt(x)} {_fmt(y)}"
            base = palette[rng.randrange(len(palette))]
            jitter = rng.uniform(0.88, 1.12)
            _set_visual_material(
                visual,
                (min(base[0] * jitter, 1.0), min(base[1] * jitter, 1.0), min(base[2] * jitter, 1.0), 1.0),
            )
        return

    if patch.kind == "dust":
        cell_x = width / DIRT_CELL_COLUMNS
        cell_y = height / DIRT_CELL_ROWS
        for iy in range(DIRT_CELL_ROWS):
            for ix in range(DIRT_CELL_COLUMNS):
                index = iy * DIRT_CELL_COLUMNS + ix
                visual = ET.SubElement(link, "visual", {"name": f"dust_mottle_{index:03d}"})
                local_x = -width / 2.0 + (ix + 0.5) * cell_x
                local_y = -height / 2.0 + (iy + 0.5) * cell_y
                ET.SubElement(visual, "pose").text = (
                    f"{_fmt(local_x)} {_fmt(local_y)} {_fmt(0.001 + 0.00015 * ((ix + 2 * iy) % 3))} 0 0 0"
                )
                geometry = ET.SubElement(visual, "geometry")
                box = ET.SubElement(geometry, "box")
                ET.SubElement(box, "size").text = f"{_fmt(cell_x * 1.01)} {_fmt(cell_y * 1.01)} 0.002"
                tone = rng.uniform(0.23, 0.52)
                warm = rng.uniform(-0.025, 0.035)
                _set_visual_material(visual, (tone + warm, tone, max(0.0, tone - 0.035), rng.uniform(0.68, 0.9)))
        return

    if patch.kind == "puddle":
        for index in range(PUDDLE_VISUAL_COUNT):
            angle = rng.uniform(0.0, 2.0 * math.pi)
            radial = math.sqrt(rng.random())
            local_x = math.cos(angle) * radial * width * 0.46
            local_y = math.sin(angle) * radial * height * 0.46
            radius = rng.uniform(0.055, 0.12) * min(width, height)
            visual = ET.SubElement(link, "visual", {"name": f"puddle_lobe_{index:03d}"})
            ET.SubElement(visual, "pose").text = f"{_fmt(local_x)} {_fmt(local_y)} 0.001 0 0 0"
            geometry = ET.SubElement(visual, "geometry")
            cylinder = ET.SubElement(geometry, "cylinder")
            ET.SubElement(cylinder, "radius").text = _fmt(radius)
            ET.SubElement(cylinder, "length").text = "0.0015"
            blue = rng.uniform(0.16, 0.34)
            highlight = 0.09 if index % 11 == 0 else 0.0
            _set_visual_material(
                visual,
                (0.035 + highlight, 0.10 + 0.35 * blue + highlight, blue + highlight, rng.uniform(0.48, 0.72)),
            )
        return

    raise GenerationError(f"unsupported dirt visual kind: {patch.kind}")


def _opaque_id(prefix: str, seed: int, index: int) -> str:
    token = _sha256_text(f"{prefix}:{seed}:{index}")[:12]
    return f"{prefix}_{token}"


def load_config(path: str | Path) -> dict[str, Any]:
    config_path = Path(path)
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise GenerationError(f"cannot load config: {exc}") from exc
    if not isinstance(raw, dict):
        raise GenerationError("config root must be a mapping")
    validate_config(raw)
    return raw


def _positive_number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise GenerationError(f"{path} must be a positive number")
    return float(value)


def _nonnegative_integer(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise GenerationError(f"{path} must be a non-negative integer")
    return value


def _require_exact_keys(value: Any, expected: set[str], path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise GenerationError(f"{path} must be a mapping")
    actual = set(value)
    if actual != expected:
        missing = sorted(expected - actual)
        unexpected = sorted(actual - expected)
        raise GenerationError(
            f"{path} keys invalid; missing={missing}, unexpected={unexpected}"
        )
    return value


def validate_config(config: dict[str, Any]) -> None:
    _require_exact_keys(
        config, {"schema_version", "profiles", "episode", "split", "truth"}, "config"
    )
    if config.get("schema_version") != 1:
        raise GenerationError("schema_version must equal 1")
    profiles = _require_exact_keys(
        config.get("profiles"), {"research", "formal"}, "profiles"
    )
    expected = {"research": (106.0, 53.0), "formal": (200.0, 100.0)}
    for name, dimensions in expected.items():
        profile = _require_exact_keys(
            profiles[name],
            {
                "width_m",
                "height_m",
                "building_count",
                "pole_count",
                "bin_count",
                "tree_count",
                "bench_count",
            },
            f"profiles.{name}",
        )
        width = _positive_number(profile.get("width_m"), f"profiles.{name}.width_m")
        height = _positive_number(profile.get("height_m"), f"profiles.{name}.height_m")
        if (width, height) != dimensions:
            raise GenerationError(f"profiles.{name} dimensions must be {dimensions}")
        for key in ("building_count", "pole_count", "bin_count", "tree_count", "bench_count"):
            _nonnegative_integer(profile.get(key), f"profiles.{name}.{key}")
    episode = _require_exact_keys(
        config.get("episode"),
        {
            "cube_count",
            "cube_edge_m",
            "grasp_clearance_m",
            "grasp_reach_radius_m",
            "dirt_patch_count",
            "dirt_patch_area_m2",
            "dirt_spacing_m",
            "pedestrian_count",
        },
        "episode",
    )
    cube_count = _nonnegative_integer(episode.get("cube_count"), "episode.cube_count")
    if cube_count > 20:
        raise GenerationError("episode.cube_count cannot exceed 20")
    if abs(_positive_number(episode.get("cube_edge_m"), "episode.cube_edge_m") - 0.03) > 1e-9:
        raise GenerationError("episode.cube_edge_m must equal 0.03")
    for key in ("dirt_patch_count", "pedestrian_count"):
        _nonnegative_integer(episode.get(key), f"episode.{key}")
    if _positive_number(
        episode.get("dirt_patch_area_m2"), "episode.dirt_patch_area_m2"
    ) != 1.0:
        raise GenerationError("episode.dirt_patch_area_m2 must equal 1.0")
    _positive_number(episode.get("dirt_spacing_m"), "episode.dirt_spacing_m")
    _positive_number(episode.get("grasp_clearance_m"), "episode.grasp_clearance_m")
    reach_radius = _positive_number(
        episode.get("grasp_reach_radius_m"), "episode.grasp_reach_radius_m"
    )
    placement_clearance = float(episode["grasp_clearance_m"])
    if placement_clearance <= reach_radius:
        raise GenerationError(
            "episode.grasp_clearance_m must exceed grasp_reach_radius_m so "
            "whole-vehicle parking clearance is not confused with arm reach"
        )
    split = _require_exact_keys(
        config.get("split"), {"master_seed", "train", "val", "hidden"}, "split"
    )
    expected_splits = {
        "train": (32, 200),
        "val": (8, 100),
        "hidden": (12, 100),
    }
    for name, (map_count, missions_per_map) in expected_splits.items():
        section = _require_exact_keys(
            split.get(name), {"map_count", "missions_per_map"}, f"split.{name}"
        )
        if section.get("map_count") != map_count:
            raise GenerationError(f"split.{name}.map_count must equal {map_count}")
        if section.get("missions_per_map") != missions_per_map:
            raise GenerationError(
                f"split.{name}.missions_per_map must equal {missions_per_map}"
            )
    _nonnegative_integer(split.get("master_seed"), "split.master_seed")
    truth = _require_exact_keys(
        config.get("truth"), {"evaluator_namespace"}, "truth"
    )
    namespace = truth.get("evaluator_namespace")
    if namespace != EVALUATOR_NAMESPACE:
        raise GenerationError(f"truth.evaluator_namespace must equal {EVALUATOR_NAMESPACE}")


def split_index(config: dict[str, Any]) -> dict[str, Any]:
    validate_config(config)
    master = config["split"]["master_seed"]
    maps: list[dict[str, Any]] = []
    mission_total = 0
    for split in SPLITS:
        section = config["split"][split]
        for map_index in range(section["map_count"]):
            layout_seed = _derived_seed(master, split, map_index, "layout")
            missions = []
            for mission_index in range(section["missions_per_map"]):
                seeds = _mission_seeds(master, split, map_index, mission_index)
                mission_id = f"{split}-map-{map_index:03d}-mission-{mission_index:03d}"
                missions.append(
                    {
                        "mission_id": mission_id,
                        "mission_index": mission_index,
                        "seeds": {
                            "dirt": seeds.dirt,
                            "cubes": seeds.cubes,
                            "pedestrians": seeds.pedestrians,
                            "sensor": seeds.sensor,
                        },
                    }
                )
                mission_total += 1
            maps.append(
                {
                    "map_id": f"{split}-map-{map_index:03d}",
                    "split": split,
                    "map_index": map_index,
                    "layout_seed": layout_seed,
                    "missions": missions,
                }
            )
    return {
        "schema_version": 1,
        "generator_version": GENERATOR_VERSION,
        "counts": {
            name: {
                "map_count": config["split"][name]["map_count"],
                "missions_per_map": config["split"][name]["missions_per_map"],
                "mission_count": config["split"][name]["map_count"]
                * config["split"][name]["missions_per_map"],
            }
            for name in SPLITS
        },
        "total_map_count": sum(config["split"][name]["map_count"] for name in SPLITS),
        "total_mission_count": mission_total,
        "maps": maps,
    }


def _derived_seed(master: int, *parts: object) -> int:
    digest = hashlib.sha256(_canonical_json([master, *parts]).encode("utf-8")).digest()
    return int.from_bytes(digest[:8], "big") % (2**31 - 2) + 1


def _mission_seeds(
    master: int, split: str, map_index: int, mission_index: int
) -> EpisodeSeeds:
    return EpisodeSeeds(
        layout=_derived_seed(master, split, map_index, "layout"),
        dirt=_derived_seed(master, split, map_index, mission_index, "dirt"),
        cubes=_derived_seed(master, split, map_index, mission_index, "cubes"),
        pedestrians=_derived_seed(
            master, split, map_index, mission_index, "pedestrians"
        ),
        sensor=_derived_seed(master, split, map_index, mission_index, "sensor"),
    )


def derive_field_dimensions(
    profile: dict[str, Any], map_index: int, layout_seed: int
) -> tuple[float, float]:
    """Derive a fixed-area rectangular field from the map layout seed.

    Map zero remains the frozen baseline for each split. Other maps use one of
    four bounded aspect multipliers, keeping the nominal 2:1 campus character
    while exercising materially different geometry.
    """
    base_width = float(profile["width_m"])
    base_height = float(profile["height_m"])
    if map_index == 0:
        return base_width, base_height
    base_area = base_width * base_height
    base_aspect = base_width / base_height
    multipliers = (0.75, 0.875, 1.125, 1.25)
    selector = _derived_seed(layout_seed, "field_aspect") % len(multipliers)
    aspect = base_aspect * multipliers[selector]
    width = math.sqrt(base_area * aspect)
    height = base_area / width
    return width, height


def seeds_for(
    config: dict[str, Any], split: str, map_index: int, mission_index: int
) -> EpisodeSeeds:
    if split not in SPLITS:
        raise GenerationError(f"unknown split: {split}")
    section = config["split"][split]
    if map_index < 0 or map_index >= section["map_count"]:
        raise GenerationError(f"map_index out of range for {split}: {map_index}")
    if mission_index < 0 or mission_index >= section["missions_per_map"]:
        raise GenerationError(
            f"mission_index out of range for {split}: {mission_index}"
        )
    return _mission_seeds(
        config["split"]["master_seed"], split, map_index, mission_index
    )


def _inside(x: float, y: float, width: float, height: float, margin: float) -> bool:
    return abs(x) <= width / 2.0 - margin and abs(y) <= height / 2.0 - margin


def _overlaps_asset(x: float, y: float, radius: float, asset: Asset, margin: float = 0.0) -> bool:
    # Rotation-invariant and intentionally conservative for arbitrary asset yaw.
    asset_radius = math.hypot(*asset.half_extent)
    return math.hypot(x - asset.pose.x_m, y - asset.pose.y_m) < asset_radius + radius + margin


_GEOMETRY_EPSILON = 1e-9


def _point_segment_distance(point: tuple[float, float], start: tuple[float, float], end: tuple[float, float]) -> float:
    """Return the deterministic Euclidean distance from a point to a segment."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= _GEOMETRY_EPSILON:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    fraction = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared))
    return math.hypot(point[0] - (start[0] + fraction * dx), point[1] - (start[1] + fraction * dy))


def _point_box_distance(point: tuple[float, float], half_extent: tuple[float, float]) -> float:
    return math.hypot(max(abs(point[0]) - half_extent[0], 0.0), max(abs(point[1]) - half_extent[1], 0.0))


def _segments_intersect(start: tuple[float, float], end: tuple[float, float], left: tuple[float, float], right: tuple[float, float]) -> bool:
    """Inclusive segment intersection, including tangencies and zero-length cases."""
    def cross(origin: tuple[float, float], first: tuple[float, float], second: tuple[float, float]) -> float:
        return (first[0] - origin[0]) * (second[1] - origin[1]) - (first[1] - origin[1]) * (second[0] - origin[0])

    def on_segment(origin: tuple[float, float], point: tuple[float, float], target: tuple[float, float]) -> bool:
        return abs(cross(origin, target, point)) <= _GEOMETRY_EPSILON and (
            min(origin[0], target[0]) - _GEOMETRY_EPSILON <= point[0] <= max(origin[0], target[0]) + _GEOMETRY_EPSILON
            and min(origin[1], target[1]) - _GEOMETRY_EPSILON <= point[1] <= max(origin[1], target[1]) + _GEOMETRY_EPSILON
        )

    first, second = cross(start, end, left), cross(start, end, right)
    third, fourth = cross(left, right, start), cross(left, right, end)
    if ((first > _GEOMETRY_EPSILON and second < -_GEOMETRY_EPSILON) or (first < -_GEOMETRY_EPSILON and second > _GEOMETRY_EPSILON)) and ((third > _GEOMETRY_EPSILON and fourth < -_GEOMETRY_EPSILON) or (third < -_GEOMETRY_EPSILON and fourth > _GEOMETRY_EPSILON)):
        return True
    return any((on_segment(start, left, end), on_segment(start, right, end), on_segment(left, start, right), on_segment(left, end, right)))


def _segment_box_distance(start: tuple[float, float], end: tuple[float, float], center: tuple[float, float], half_extent: tuple[float, float], yaw_rad: float) -> float:
    """Exact 2-D distance between a segment and a rotated box footprint."""
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    def local(point: tuple[float, float]) -> tuple[float, float]:
        dx, dy = point[0] - center[0], point[1] - center[1]
        return cosine * dx + sine * dy, -sine * dx + cosine * dy

    local_start, local_end = local(start), local(end)
    hx, hy = half_extent
    corners = ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))
    if _point_box_distance(local_start, half_extent) <= _GEOMETRY_EPSILON or _point_box_distance(local_end, half_extent) <= _GEOMETRY_EPSILON:
        return 0.0
    if any(_segments_intersect(local_start, local_end, corners[index], corners[(index + 1) % 4]) for index in range(4)):
        return 0.0
    return min(
        _point_box_distance(local_start, half_extent),
        _point_box_distance(local_end, half_extent),
        *(_point_segment_distance(corner, local_start, local_end) for corner in corners),
    )


def segment_clearance_to_asset(start: tuple[float, float], end: tuple[float, float], radius_m: float, asset: Asset, clearance_m: float = 0.0) -> bool:
    """Return whether a pedestrian capsule is clear of one true SDF collision.

    Boxes retain their yaw; poles and trees use their cylinder collision radius.
    Tangency is unsafe by contract, so equality returns ``False``.
    """
    if radius_m < 0.0 or clearance_m < 0.0:
        raise GenerationError("segment clearance radii must be non-negative")
    required = radius_m + clearance_m
    if asset.kind in {"pole", "tree"}:
        obstacle_radius = asset.size_m[0] / 2.0 if asset.kind == "pole" else 0.28
        distance = _point_segment_distance((asset.pose.x_m, asset.pose.y_m), start, end)
        required += obstacle_radius
    else:
        distance = _segment_box_distance(start, end, (asset.pose.x_m, asset.pose.y_m), asset.half_extent, asset.pose.yaw_rad)
    return distance > required + _GEOMETRY_EPSILON


def segment_clearance_to_cube(start: tuple[float, float], end: tuple[float, float], radius_m: float, cube: Cube, clearance_m: float = 0.0) -> bool:
    if radius_m < 0.0 or clearance_m < 0.0:
        raise GenerationError("segment clearance radii must be non-negative")
    distance = _segment_box_distance(start, end, (cube.pose.x_m, cube.pose.y_m), (cube.edge_m / 2.0, cube.edge_m / 2.0), cube.pose.yaw_rad)
    return distance > radius_m + clearance_m + _GEOMETRY_EPSILON


def segment_clearance_to_pedestrian(
    start: tuple[float, float],
    end: tuple[float, float],
    radius_m: float,
    other_start: tuple[float, float],
    other_end: tuple[float, float],
    other_radius_m: float,
    clearance_m: float = 0.0,
) -> bool:
    """Return whether two pedestrian-path capsules are strictly separated.

    This intentionally rejects spatially overlapping paths even if a particular
    schedule phase happens to avoid an immediate encounter.  The generated
    schedules loop forever with independently rounded durations, so this
    conservative geometry rule prevents collisions at any later phase too.
    """
    if min(radius_m, other_radius_m, clearance_m) < 0.0:
        raise GenerationError("segment clearance radii must be non-negative")
    distance = 0.0 if _segments_intersect(start, end, other_start, other_end) else min(
        _point_segment_distance(start, other_start, other_end),
        _point_segment_distance(end, other_start, other_end),
        _point_segment_distance(other_start, start, end),
        _point_segment_distance(other_end, start, end),
    )
    return distance > radius_m + other_radius_m + clearance_m + _GEOMETRY_EPSILON


def pedestrian_paths_clear(
    pedestrian: Pedestrian, other: Pedestrian, *, clearance_m: float = 0.0
) -> bool:
    """Return whether every segment of two looping pedestrian paths is clear."""
    return all(
        segment_clearance_to_pedestrian(
            (x1, y1), (x2, y2), pedestrian.radius_m,
            (other_x1, other_y1), (other_x2, other_y2), other.radius_m,
            clearance_m,
        )
        for (_, x1, y1), (_, x2, y2) in zip(
            pedestrian.waypoints, pedestrian.waypoints[1:]
        )
        for (_, other_x1, other_y1), (_, other_x2, other_y2) in zip(
            other.waypoints, other.waypoints[1:]
        )
    )


def validate_episode_geometry(profile: dict[str, Any], assets: Iterable[Asset], cubes: Iterable[Cube], pedestrians: Iterable[Pedestrian], *, pedestrian_clearance_m: float = 0.75) -> None:
    """Fail closed unless every pedestrian motion segment clears public collisions.

    This deliberately consumes only collision-bearing static assets and cubes.
    Dirt, puddles and leaves remain surface semantics, not hard obstacles.
    """
    asset_list, cube_list, pedestrian_list = tuple(assets), tuple(cubes), tuple(pedestrians)
    for pedestrian in pedestrian_list:
        if pedestrian.radius_m <= 0.0 or pedestrian_clearance_m < 0.0:
            raise GenerationError("pedestrian geometry contains an invalid radius or clearance")
        if len(pedestrian.waypoints) < 2:
            raise GenerationError("pedestrian geometry requires at least two waypoints")
        for (_, x1, y1), (_, x2, y2) in zip(pedestrian.waypoints, pedestrian.waypoints[1:]):
            start, end = (x1, y1), (x2, y2)
            if not _inside(x1, y1, profile["width_m"], profile["height_m"], pedestrian.radius_m + pedestrian_clearance_m) or not _inside(x2, y2, profile["width_m"], profile["height_m"], pedestrian.radius_m + pedestrian_clearance_m):
                raise GenerationError(f"pedestrian {pedestrian.object_id} leaves the geofence")
            if any(not segment_clearance_to_asset(start, end, pedestrian.radius_m, asset, pedestrian_clearance_m) for asset in asset_list):
                raise GenerationError(f"pedestrian {pedestrian.object_id} intersects a static collision")
            if any(not segment_clearance_to_cube(start, end, pedestrian.radius_m, cube) for cube in cube_list):
                raise GenerationError(f"pedestrian {pedestrian.object_id} intersects a cube")
    for index, pedestrian in enumerate(pedestrian_list):
        if any(
            not pedestrian_paths_clear(pedestrian, other)
            for other in pedestrian_list[:index]
        ):
            raise GenerationError(
                f"pedestrian {pedestrian.object_id} intersects another pedestrian path"
            )


def _sample_free_point(
    rng: random.Random,
    width: float,
    height: float,
    assets: Iterable[Asset],
    *,
    radius: float,
    clearance: float,
    existing: Iterable[tuple[float, float, float]] = (),
    max_attempts: int = 5000,
) -> tuple[float, float]:
    boundary_margin = radius + clearance
    for _ in range(max_attempts):
        x = rng.uniform(-width / 2 + boundary_margin, width / 2 - boundary_margin)
        y = rng.uniform(-height / 2 + boundary_margin, height / 2 - boundary_margin)
        if any(_overlaps_asset(x, y, radius, asset, clearance) for asset in assets):
            continue
        if any(math.hypot(x - px, y - py) < radius + pr + clearance for px, py, pr in existing):
            continue
        return round(x, 6), round(y, 6)
    raise GenerationError("unable to place object within bounded attempts")


def _asset_spec(kind: str, rng: random.Random) -> tuple[float, float, float]:
    if kind == "building":
        return (rng.uniform(8.0, 14.0), rng.uniform(6.0, 10.0), rng.uniform(4.0, 7.0))
    return {
        "pole": (0.24, 0.24, 6.0),
        "bin": (0.70, 0.60, 1.05),
        "tree": (1.20, 1.20, 5.0),
        "bench": (1.80, 0.65, 0.85),
    }[kind]


def generate_assets(profile: dict[str, Any], seed: int) -> list[Asset]:
    rng = random.Random(seed)
    width, height = profile["width_m"], profile["height_m"]
    assets: list[Asset] = []
    start = _vehicle_start_pose(profile)
    existing: list[tuple[float, float, float]] = [(start.x_m, start.y_m, 1.5)]
    for kind, count_key in (
        ("building", "building_count"),
        ("pole", "pole_count"),
        ("bin", "bin_count"),
        ("tree", "tree_count"),
        ("bench", "bench_count"),
    ):
        for index in range(profile[count_key]):
            size = _asset_spec(kind, rng)
            radius = math.hypot(size[0], size[1]) / 2.0
            x, y = _sample_free_point(
                rng, width, height, (), radius=radius, clearance=2.0, existing=existing
            )
            yaw = round(rng.uniform(-math.pi, math.pi), 6)
            assets.append(Asset(_opaque_id("asset", seed, len(assets)), kind, Pose2D(x, y, yaw), tuple(round(v, 4) for v in size)))
            existing.append((x, y, radius))
    return assets


def generate_cubes(profile: dict[str, Any], episode: dict[str, Any], assets: list[Asset], seed: int) -> list[Cube]:
    rng = random.Random(seed)
    edge = episode["cube_edge_m"]
    clearance = episode["grasp_clearance_m"]
    existing: list[tuple[float, float, float]] = []
    cubes: list[Cube] = []
    for index in range(episode["cube_count"]):
        x, y = _sample_free_point(
            rng,
            profile["width_m"],
            profile["height_m"],
            assets,
            radius=edge / 2.0,
            clearance=clearance,
            existing=existing,
        )
        color = tuple(round(rng.uniform(0.05, 0.95), 6) for _ in range(3)) + (1.0,)
        material = rng.choice(tuple(CUBE_MATERIAL_DENSITY_KG_M3))
        density = CUBE_MATERIAL_DENSITY_KG_M3[material]
        mass = density * edge**3
        cubes.append(
            Cube(
                _opaque_id("object", seed, index),
                Pose2D(x, y, round(rng.uniform(-math.pi, math.pi), 6)),
                edge,
                color,
                material,
                density,
                round(mass, 9),
            )
        )
        existing.append((x, y, edge / 2.0))
    return cubes


def generate_dirt(profile: dict[str, Any], episode: dict[str, Any], assets: list[Asset], seed: int) -> list[DirtPatch]:
    rng = random.Random(seed)
    dirt: list[DirtPatch] = []
    occupied: list[tuple[float, float, float]] = []
    # Every template is exactly 1.0 m2, so aspect ratio can change without
    # changing the evaluated dirty area. Conservative bounding circles make
    # rotated rectangles mutually exclusive.
    shape_templates = ((0.5, 2.0), (0.8, 1.25), (1.0, 1.0), (1.25, 0.8), (2.0, 0.5))
    for index in range(episode["dirt_patch_count"]):
        kind = ("leaf", "dust", "puddle")[index % 3]
        sx, sy = rng.choice(shape_templates)
        radius = math.hypot(sx, sy) / 2.0
        x, y = _sample_free_point(
            rng, profile["width_m"], profile["height_m"], assets,
            radius=radius,
            clearance=episode["dirt_spacing_m"],
            existing=occupied,
        )
        color = {
            "leaf": (0.30, 0.18, 0.04, 0.92),
            "dust": (0.42, 0.38, 0.32, 0.72),
            "puddle": (0.05, 0.18, 0.30, 0.58),
        }[kind]
        dirt.append(
            DirtPatch(
                _opaque_id("surface", seed, index),
                kind,
                Pose2D(x, y, round(rng.uniform(-math.pi, math.pi), 6)),
                (sx, sy),
                episode["dirt_patch_area_m2"],
                color,
            )
        )
        occupied.append((x, y, radius))
    return dirt


def generate_pedestrians(profile: dict[str, Any], episode: dict[str, Any], assets: list[Asset], cubes: list[Cube], seed: int) -> list[Pedestrian]:
    rng = random.Random(seed)
    pedestrians: list[Pedestrian] = []
    occupied: list[tuple[float, float, float]] = []
    for index in range(episode["pedestrian_count"]):
        radius, height = 0.25, rng.uniform(1.55, 1.90)
        for _ in range(1000):
            x1, y1 = _sample_free_point(
                rng, profile["width_m"], profile["height_m"], assets,
                radius=radius, clearance=0.75, existing=occupied,
            )
            if not all(
                segment_clearance_to_cube((x1, y1), (x1, y1), radius, cube)
                for cube in cubes
            ):
                continue
            for _ in range(100):
                angle = rng.uniform(-math.pi, math.pi)
                distance = rng.uniform(
                    8.0, min(25.0, max(8.1, profile["height_m"] * 0.45))
                )
                x2, y2 = x1 + math.cos(angle) * distance, y1 + math.sin(angle) * distance
                candidate = Pedestrian(
                    "candidate", radius, height, 0.0,
                    ((0.0, x1, y1), (1.0, x2, y2), (2.0, x1, y1)),
                )
                if (
                    _inside(x2, y2, profile["width_m"], profile["height_m"], radius + 0.75)
                    and all(segment_clearance_to_asset((x1, y1), (x2, y2), radius, asset, 0.75) for asset in assets)
                    and all(segment_clearance_to_cube((x1, y1), (x2, y2), radius, cube) for cube in cubes)
                    and all(pedestrian_paths_clear(candidate, other) for other in pedestrians)
                ):
                    break
            else:
                continue
            break
        else:
            raise GenerationError("unable to construct pedestrian trajectory")
        speed = round(rng.uniform(0.45, 1.35), 4)
        duration = round(math.hypot(x2 - x1, y2 - y1) / speed, 4)
        pedestrians.append(Pedestrian(_opaque_id("walker", seed, index), radius, round(height, 4), speed, ((0.0, x1, y1), (duration, round(x2, 6), round(y2, 6)), (2 * duration, x1, y1))))
        occupied.append((x1, y1, radius))
    return pedestrians


def _fmt(value: float) -> str:
    return f"{value:.6f}".rstrip("0").rstrip(".") or "0"


def _add_asset_model(world: ET.Element, asset: Asset) -> None:
    model = ET.SubElement(world, "model", {"name": asset.asset_id})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{_fmt(asset.pose.x_m)} {_fmt(asset.pose.y_m)} {_fmt(asset.size_m[2] / 2)} 0 0 {_fmt(asset.pose.yaw_rad)}"
    link = ET.SubElement(model, "link", {"name": "link"})
    if asset.kind in ("pole", "tree"):
        radius = asset.size_m[0] / 2.0 if asset.kind == "pole" else 0.28
        for role in ("collision", "visual"):
            node = ET.SubElement(link, role, {"name": role})
            geometry = ET.SubElement(node, "geometry")
            cylinder = ET.SubElement(geometry, "cylinder")
            ET.SubElement(cylinder, "radius").text = _fmt(radius)
            ET.SubElement(cylinder, "length").text = _fmt(asset.size_m[2])
            if role == "visual":
                material = ET.SubElement(node, "material")
                color = "0.25 0.27 0.29 1" if asset.kind == "pole" else "0.28 0.16 0.07 1"
                ET.SubElement(material, "ambient").text = color
                ET.SubElement(material, "diffuse").text = color
        if asset.kind == "tree":
            canopy = ET.SubElement(link, "visual", {"name": "canopy"})
            ET.SubElement(canopy, "pose").text = f"0 0 {_fmt(asset.size_m[2] * 0.32)} 0 0 0"
            geometry = ET.SubElement(canopy, "geometry")
            sphere = ET.SubElement(geometry, "sphere")
            ET.SubElement(sphere, "radius").text = _fmt(asset.size_m[0] / 2.0)
            material = ET.SubElement(canopy, "material")
            ET.SubElement(material, "ambient").text = "0.12 0.43 0.10 1"
            ET.SubElement(material, "diffuse").text = "0.12 0.43 0.10 1"
        return
    for role in ("collision", "visual"):
        node = ET.SubElement(link, role, {"name": role})
        geometry = ET.SubElement(node, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = " ".join(_fmt(v) for v in asset.size_m)
        if role == "visual":
            material = ET.SubElement(node, "material")
            color = {"building": "0.62 0.65 0.68 1", "pole": "0.25 0.27 0.29 1", "bin": "0.08 0.42 0.20 1", "tree": "0.18 0.48 0.14 1", "bench": "0.42 0.24 0.10 1"}[asset.kind]
            ET.SubElement(material, "ambient").text = color
            ET.SubElement(material, "diffuse").text = color


def render_sdf(profile_name: str, profile: dict[str, Any], assets: list[Asset], dirt: list[DirtPatch], cubes: list[Cube], pedestrians: list[Pedestrian], include_proxy: bool) -> str:
    root = ET.Element("sdf", {"version": "1.10"})
    world = ET.SubElement(root, "world", {"name": f"campus_{profile_name}"})
    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
        ("gz-sim-imu-system", "gz::sim::systems::Imu"),
        ("gz-sim-navsat-system", "gz::sim::systems::NavSat"),
    ):
        ET.SubElement(world, "plugin", {"filename": filename, "name": name})
    sensors = ET.SubElement(
        world,
        "plugin",
        {
            "filename": "gz-sim-sensors-system",
            "name": "gz::sim::systems::Sensors",
        },
    )
    ET.SubElement(sensors, "render_engine").text = "ogre2"
    spherical = ET.SubElement(world, "spherical_coordinates")
    for key, value in (
        ("surface_model", "EARTH_WGS84"),
        ("world_frame_orientation", "ENU"),
        ("latitude_deg", "39.9042"),
        ("longitude_deg", "116.4074"),
        ("elevation", "43.5"),
        ("heading_deg", "0"),
    ):
        ET.SubElement(spherical, key).text = value
    light = ET.SubElement(world, "light", {"type": "directional", "name": "sun"})
    ET.SubElement(light, "pose").text = "0 0 50 0 0 0"
    ET.SubElement(light, "diffuse").text = "0.9 0.9 0.86 1"
    ET.SubElement(light, "direction").text = "-0.4 0.2 -0.9"
    ground = ET.SubElement(world, "model", {"name": "ground_plane"})
    ET.SubElement(ground, "static").text = "true"
    ground_link = ET.SubElement(ground, "link", {"name": "link"})
    for role in ("collision", "visual"):
        node = ET.SubElement(ground_link, role, {"name": role})
        geometry = ET.SubElement(node, "geometry")
        plane = ET.SubElement(geometry, "plane")
        ET.SubElement(plane, "normal").text = "0 0 1"
        ET.SubElement(plane, "size").text = f"{_fmt(profile['width_m'] + 10.0)} {_fmt(profile['height_m'] + 10.0)}"
        if role == "visual":
            material = ET.SubElement(node, "material")
            ET.SubElement(material, "ambient").text = "0.34 0.36 0.35 1"
            ET.SubElement(material, "diffuse").text = "0.34 0.36 0.35 1"
    for asset in assets:
        _add_asset_model(world, asset)
    for patch in dirt:
        model = ET.SubElement(world, "model", {"name": patch.object_id})
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = f"{_fmt(patch.pose.x_m)} {_fmt(patch.pose.y_m)} 0.002 0 0 {_fmt(patch.pose.yaw_rad)}"
        link = ET.SubElement(model, "link", {"name": "link"})
        _add_dirt_visuals(link, patch)
    for cube in cubes:
        asset = Asset(cube.object_id, "bin", cube.pose, (cube.edge_m, cube.edge_m, cube.edge_m))
        model = ET.SubElement(world, "model", {"name": asset.asset_id})
        ET.SubElement(model, "static").text = "false"
        ET.SubElement(model, "pose").text = f"{_fmt(cube.pose.x_m)} {_fmt(cube.pose.y_m)} {_fmt(cube.edge_m / 2)} 0 0 {_fmt(cube.pose.yaw_rad)}"
        link = ET.SubElement(model, "link", {"name": "link"})
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "mass").text = _fmt(cube.mass_kg)
        inertia = ET.SubElement(inertial, "inertia")
        diagonal = cube.mass_kg * cube.edge_m**2 / 6.0
        for axis in ("ixx", "iyy", "izz"):
            ET.SubElement(inertia, axis).text = f"{diagonal:.12g}"
        for cross in ("ixy", "ixz", "iyz"):
            ET.SubElement(inertia, cross).text = "0"
        for role in ("collision", "visual"):
            node = ET.SubElement(link, role, {"name": role})
            geometry = ET.SubElement(node, "geometry")
            box = ET.SubElement(geometry, "box")
            ET.SubElement(box, "size").text = f"{cube.edge_m} {cube.edge_m} {cube.edge_m}"
            if role == "visual":
                material = ET.SubElement(node, "material")
                rgba = " ".join(_fmt(v) for v in cube.color_rgba)
                ET.SubElement(material, "ambient").text = rgba
                ET.SubElement(material, "diffuse").text = rgba
    for pedestrian in pedestrians:
        t0, x, y = pedestrian.waypoints[0]
        model = ET.SubElement(world, "model", {"name": pedestrian.object_id})
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = f"{_fmt(x)} {_fmt(y)} 0 0 0 0"
        link = ET.SubElement(model, "link", {"name": "link"})
        collision = ET.SubElement(link, "collision", {"name": "collision"})
        ET.SubElement(collision, "pose").text = f"0 0 {_fmt(pedestrian.height_m / 2)} 0 0 0"
        geometry = ET.SubElement(collision, "geometry")
        cylinder = ET.SubElement(geometry, "cylinder")
        ET.SubElement(cylinder, "radius").text = _fmt(pedestrian.radius_m)
        ET.SubElement(cylinder, "length").text = _fmt(pedestrian.height_m)
        visual = ET.SubElement(link, "visual", {"name": "body"})
        ET.SubElement(visual, "pose").text = f"0 0 {_fmt(pedestrian.height_m / 2)} 0 0 0"
        visual_geometry = ET.SubElement(visual, "geometry")
        visual_cylinder = ET.SubElement(visual_geometry, "cylinder")
        ET.SubElement(visual_cylinder, "radius").text = _fmt(pedestrian.radius_m * 0.96)
        ET.SubElement(visual_cylinder, "length").text = _fmt(pedestrian.height_m)
        material = ET.SubElement(visual, "material")
        ET.SubElement(material, "ambient").text = "0.92 0.32 0.08 1"
        ET.SubElement(material, "diffuse").text = "0.92 0.32 0.08 1"
    if include_proxy:
        proxy = Asset(
            "proxy_chassis_not_urdf",
            "building",
            _vehicle_start_pose(profile),
            (0.60, 0.40, 0.30),
        )
        _add_asset_model(world, proxy)
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True) + "\n"


def _geofence(profile: dict[str, Any]) -> list[list[float]]:
    w, h = profile["width_m"] / 2.0, profile["height_m"] / 2.0
    return [[-w, -h], [w, -h], [w, h], [-w, h]]


def _localize_geofence_point(point: list[float], start: Pose2D) -> tuple[float, float]:
    """Apply the one permitted source-world -> localization-map transform."""
    dx, dy = point[0] - start.x_m, point[1] - start.y_m
    cosine, sine = math.cos(start.yaw_rad), math.sin(start.yaw_rad)
    return (cosine * dx + sine * dy, -sine * dx + cosine * dy)


def _vehicle_start_pose(profile: dict[str, Any]) -> Pose2D:
    return Pose2D(-profile["width_m"] / 2.0 + 2.0, 0.0, 0.0)


def _start_exclusion(profile: dict[str, Any]) -> Asset:
    return Asset(
        "reserved_vehicle_start_not_world_entity",
        "building",
        _vehicle_start_pose(profile),
        (3.0, 3.0, 0.01),
    )


def generate_episode(
    config: dict[str, Any],
    profile_name: str,
    split: str,
    map_index: int,
    mission_index: int,
    *,
    include_proxy: bool = False,
    seed_namespace: str | None = None,
    seed_mission_index: int | None = None,
    layout_seed_override: int | None = None,
    map_id_override: str | None = None,
    reported_split: str | None = None,
    reported_mission_index: int | None = None,
    episode_id_mission_width: int = 3,
) -> dict[str, str]:
    """Generate one standard episode or a separately namespaced frozen task.

    The optional overrides are deliberately narrow: Stage-A uses the exact
    formal map-0 layout but needs 11,500 independently seeded task instances,
    whereas the multi-map generator freezes only 200/100/100 missions per
    split.  The caller cannot alter geometry through these arguments.
    """
    validate_config(config)
    if profile_name not in config["profiles"]:
        raise GenerationError(f"unknown profile: {profile_name}")
    base_profile = config["profiles"][profile_name]
    episode = config["episode"]
    if seed_namespace is None:
        seeds = seeds_for(config, split, map_index, mission_index)
    else:
        if not seed_namespace or seed_mission_index is None or seed_mission_index < 0:
            raise GenerationError("namespaced episode requires a non-negative task index")
        master = int(config["split"]["master_seed"])
        seeds = EpisodeSeeds(
            layout=(
                int(layout_seed_override)
                if layout_seed_override is not None
                else _derived_seed(master, seed_namespace, "layout")
            ),
            dirt=_derived_seed(master, seed_namespace, seed_mission_index, "dirt"),
            cubes=_derived_seed(master, seed_namespace, seed_mission_index, "cubes"),
            pedestrians=_derived_seed(
                master, seed_namespace, seed_mission_index, "pedestrians"
            ),
            sensor=_derived_seed(master, seed_namespace, seed_mission_index, "sensor"),
        )
    derived_width, derived_height = derive_field_dimensions(
        base_profile, map_index, seeds.layout
    )
    profile = {
        **base_profile,
        "width_m": derived_width,
        "height_m": derived_height,
    }
    assets = generate_assets(profile, seeds.layout)
    sampling_exclusions = [*assets, _start_exclusion(profile)]
    cubes = generate_cubes(profile, episode, sampling_exclusions, seeds.cubes)
    dirt = generate_dirt(profile, episode, sampling_exclusions, seeds.dirt)
    pedestrians = generate_pedestrians(
        profile, episode, sampling_exclusions, cubes, seeds.pedestrians
    )
    validate_episode_geometry(profile, assets, cubes, pedestrians)
    sdf = render_sdf(profile_name, profile, assets, dirt, cubes, pedestrians, include_proxy)
    ET.fromstring(sdf)
    map_id = map_id_override or f"{split}-map-{map_index:03d}"
    report_split = reported_split or split
    report_mission_index = (
        mission_index if reported_mission_index is None else reported_mission_index
    )
    if episode_id_mission_width not in {3, 5}:
        raise GenerationError("episode ID mission width must be 3 or 5")
    episode_id = (
        f"{map_id}-mission-{report_mission_index:0{episode_id_mission_width}d}"
    )
    runtime_schedule = {
        "schema_version": 1,
        "access": "environment_driver_only_not_robot_control",
        "world_name": f"campus_{profile_name}",
        "loop": True,
        "pedestrians": [asdict(item) for item in pedestrians],
    }
    truth = {
        "schema_version": 1,
        "namespace": EVALUATOR_NAMESPACE,
        "control_use_prohibited": True,
        "episode_id": episode_id,
        "map_id": map_id,
        "static_assets": [asdict(item) for item in assets],
        "dirt_patches": [asdict(item) for item in dirt],
        "discrete_cubes": [asdict(item) for item in cubes],
        "pedestrians": [asdict(item) for item in pedestrians],
    }
    config_sha = _sha256_text(_canonical_json(config))
    dirt_union_area_m2 = round(sum(item.area_m2 for item in dirt), 6)
    field_area_m2 = base_profile["width_m"] * base_profile["height_m"]
    start_pose = _vehicle_start_pose(profile)
    public_manifest = {
        "schema_version": 1,
        "episode_id": episode_id,
        "map_id": map_id,
        "profile": profile_name,
        "split": report_split,
        "map_index": map_index,
        "mission_index": report_mission_index,
        "field": {
            "width_m": profile["width_m"],
            "height_m": profile["height_m"],
            "area_m2": field_area_m2,
            "aspect_ratio": profile["width_m"] / profile["height_m"],
            "dimension_policy": "per_map_aspect_fixed_area",
            "physical_boundary_walls": False,
            "source_world_geofence": {
                "frame_id": "source_world",
                "polygon_m": _geofence(profile),
            },
            "localization_map_geofence": {
                "frame_id": "map",
                "polygon_m": [
                    list(_localize_geofence_point(point, start_pose))
                    for point in _geofence(profile)
                ],
                "transform": "source_world_to_localization_map_at_fixed_start",
            },
            "legacy_geofence": {
                "field": "geofence_polygon_m",
                "frame_id": "source_world",
                "deprecation": "use source_world_geofence or localization_map_geofence",
            },
            # Legacy machine semantics are fixed: this is source-world data,
            # never a localization-map polygon despite the historical name.
            "geofence_frame": "source_world",
            "geofence_polygon_m": _geofence(profile),
        },
        "counts": {"static_assets": len(assets), "dirt_patches": len(dirt), "discrete_cubes": len(cubes), "pedestrians": len(pedestrians)},
        "cube_contract": {
            "edge_m": 0.03,
            "single_layer": True,
            "maximum_count": 20,
            "grasp_clearance_m": episode["grasp_clearance_m"],
            "grasp_reach_radius_m": episode["grasp_reach_radius_m"],
            "clearance_semantics": "whole_vehicle_side_pick_parking_envelope",
        },
        "dirt_contract": {
            "patch_area_m2": episode["dirt_patch_area_m2"],
            "total_union_area_m2": dirt_union_area_m2,
            "overlap_policy": "mutually_exclusive_conservative_bounds",
            "shape_randomization": "fixed_area_rectangle_aspect_ratio",
            "visual_representation": "deterministic_procedural_realistic_v2",
            "visual_assets_redistributable": True,
            "visual_counts_by_kind": {
                "leaf": LEAF_VISUAL_COUNT,
                "dust": DUST_VISUAL_COUNT,
                "puddle": PUDDLE_VISUAL_COUNT,
            },
        },
        "vehicle": {"included": include_proxy, "profile": "proxy_chassis_not_urdf" if include_proxy else None, "urdf_claim": False},
        # Keep the historical field for consumers that materialize the Gazebo
        # source world, but make the frame boundary explicit for localization
        # and evaluator consumers.  The product localization map is reset at
        # the fixed vehicle start; it is not the Gazebo source-world frame.
        "vehicle_start_pose_map": asdict(start_pose),
        "vehicle_start_pose_source_world": asdict(start_pose),
        "vehicle_start_pose_localization_map": {
            "x_m": 0.0,
            "y_m": 0.0,
            "yaw_rad": 0.0,
        },
        "map_resolution_contract": {
            # This public bundle does not own product-runtime defaults. Their
            # executing components record actual values in their own reports.
            "static_materializer": {"value_source": "formal_campus.launch.py:map_resolution", "purpose": "public_world_static_collision_raster"},
            "lifecycle_support_mask": {"value_source": "prepare_public_lifecycle_artifacts(resolution)", "purpose": "public_geofence_support_mask"},
            "slam_occupancy": {"value_source": "saved_map_metadata", "maximum_accepted_resolution_value_source": "map_lifecycle_core.MAXIMUM_SAVED_MAP_RESOLUTION_M", "purpose": "runtime_lidar_slam_occupancy"},
            "coverage_planning": {"value_source": "ProductCoverageTelemetry(raster_resolution_m)", "purpose": "saved_map_coverage_raster_planning"},
        },
        "dynamic_pedestrians_present": bool(pedestrians),
    }
    evaluator_manifest = {
        "schema_version": 1,
        "generator_version": GENERATOR_VERSION,
        "episode_id": episode_id,
        "map_id": map_id,
        "profile": profile_name,
        "split": report_split,
        "map_index": map_index,
        "mission_index": report_mission_index,
        "seeds": asdict(seeds),
        "truth_boundary": {"evaluator_namespace": EVALUATOR_NAMESPACE, "control_use_prohibited": True, "truth_file": "evaluator/ground_truth.json"},
        "runtime_environment": {"pedestrian_schedule": "environment/pedestrian_schedule.json", "driver_required_for_motion": bool(pedestrians)},
        "sensor_randomization": {
            "seed": seeds.sensor,
            "parameters_pending_real_sensor_profile": True,
        },
        "config_sha256": config_sha,
        "world_sha256": _sha256_text(sdf),
    }
    truth["dirt_union_area_m2"] = dirt_union_area_m2
    truth["dirt_overlap_policy"] = "mutually_exclusive_conservative_bounds"
    truth["dirt_cell_contract"] = {
        "columns_per_patch": DIRT_CELL_COLUMNS,
        "rows_per_patch": DIRT_CELL_ROWS,
        "cell_area_m2": episode["dirt_patch_area_m2"]
        / (DIRT_CELL_COLUMNS * DIRT_CELL_ROWS),
        "total_cell_count": len(dirt) * DIRT_CELL_COLUMNS * DIRT_CELL_ROWS,
        "state_owner": "gazebo_evaluator_only",
        "product_ros_truth_exported": False,
    }
    return {
        "public/world.sdf": sdf,
        "public/episode_manifest.json": json.dumps(public_manifest, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
        "evaluator/episode_manifest.json": json.dumps(evaluator_manifest, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
        "evaluator/ground_truth.json": json.dumps(truth, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
        "environment/pedestrian_schedule.json": json.dumps(runtime_schedule, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
    }


def generate_stage_a_episode(
    config: dict[str, Any],
    profile_name: str,
    phase: str,
    task_index: int,
    *,
    include_proxy: bool = False,
) -> dict[str, str]:
    """Generate a Stage-A task on one fixed formal layout with a unique seed.

    Stage-A is intentionally distinct from the 32/8/12 multi-map split.  It
    shares one layout across train/validation/hidden tasks, while all task
    randomization uses an immutable phase-qualified seed namespace.
    """
    phases = {"train", "validation", "hidden"}
    if phase not in phases:
        raise GenerationError(f"unknown Stage-A phase: {phase}")
    if task_index < 0:
        raise GenerationError("Stage-A task index must be non-negative")
    master = int(config["split"]["master_seed"])
    return generate_episode(
        config,
        profile_name,
        "train",
        0,
        0,
        include_proxy=include_proxy,
        seed_namespace=f"stage_a_fixed_formal/{phase}",
        seed_mission_index=task_index,
        layout_seed_override=_derived_seed(master, "stage_a_fixed_formal", "layout"),
        map_id_override="stage-a-fixed-formal-map-000",
        reported_split=f"stage_a_{phase}",
        reported_mission_index=task_index,
        episode_id_mission_width=5,
    )
