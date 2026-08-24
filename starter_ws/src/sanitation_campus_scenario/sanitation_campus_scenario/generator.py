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


GENERATOR_VERSION = "0.1.0"
EVALUATOR_NAMESPACE = "/evaluation/scenario_ground_truth"
SPLITS = ("train", "val", "hidden")


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
        cubes.append(Cube(_opaque_id("object", seed, index), Pose2D(x, y, round(rng.uniform(-math.pi, math.pi), 6)), edge, color))
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


def generate_pedestrians(profile: dict[str, Any], episode: dict[str, Any], assets: list[Asset], seed: int) -> list[Pedestrian]:
    rng = random.Random(seed)
    pedestrians: list[Pedestrian] = []
    occupied: list[tuple[float, float, float]] = []
    for index in range(episode["pedestrian_count"]):
        radius, height = 0.25, rng.uniform(1.55, 1.90)
        x1, y1 = _sample_free_point(rng, profile["width_m"], profile["height_m"], assets, radius=radius, clearance=0.75, existing=occupied)
        for _ in range(1000):
            angle = rng.uniform(-math.pi, math.pi)
            distance = rng.uniform(8.0, min(25.0, max(8.1, profile["height_m"] * 0.45)))
            x2, y2 = x1 + math.cos(angle) * distance, y1 + math.sin(angle) * distance
            if _inside(x2, y2, profile["width_m"], profile["height_m"], 1.0) and not any(_overlaps_asset(x2, y2, radius, a, 0.75) for a in assets):
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
    ):
        ET.SubElement(world, "plugin", {"filename": filename, "name": name})
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
        visual = ET.SubElement(link, "visual", {"name": "visual"})
        geometry = ET.SubElement(visual, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = f"{_fmt(patch.size_m[0])} {_fmt(patch.size_m[1])} 0.004"
        material = ET.SubElement(visual, "material")
        rgba = " ".join(_fmt(v) for v in patch.color_rgba)
        ET.SubElement(material, "ambient").text = rgba
        ET.SubElement(material, "diffuse").text = rgba
    for cube in cubes:
        asset = Asset(cube.object_id, "bin", cube.pose, (cube.edge_m, cube.edge_m, cube.edge_m))
        model = ET.SubElement(world, "model", {"name": asset.asset_id})
        ET.SubElement(model, "static").text = "false"
        ET.SubElement(model, "pose").text = f"{_fmt(cube.pose.x_m)} {_fmt(cube.pose.y_m)} {_fmt(cube.edge_m / 2)} 0 0 {_fmt(cube.pose.yaw_rad)}"
        link = ET.SubElement(model, "link", {"name": "link"})
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "mass").text = "0.02"
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
) -> dict[str, str]:
    validate_config(config)
    if profile_name not in config["profiles"]:
        raise GenerationError(f"unknown profile: {profile_name}")
    base_profile = config["profiles"][profile_name]
    episode = config["episode"]
    seeds = seeds_for(config, split, map_index, mission_index)
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
        profile, episode, sampling_exclusions, seeds.pedestrians
    )
    sdf = render_sdf(profile_name, profile, assets, dirt, cubes, pedestrians, include_proxy)
    ET.fromstring(sdf)
    map_id = f"{split}-map-{map_index:03d}"
    episode_id = f"{map_id}-mission-{mission_index:03d}"
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
        "split": split,
        "map_index": map_index,
        "mission_index": mission_index,
        "field": {
            "width_m": profile["width_m"],
            "height_m": profile["height_m"],
            "area_m2": field_area_m2,
            "aspect_ratio": profile["width_m"] / profile["height_m"],
            "dimension_policy": "per_map_aspect_fixed_area",
            "physical_boundary_walls": False,
            "geofence_frame": "map",
            "geofence_polygon_m": _geofence(profile),
        },
        "counts": {"static_assets": len(assets), "dirt_patches": len(dirt), "discrete_cubes": len(cubes), "pedestrians": len(pedestrians)},
        "cube_contract": {"edge_m": 0.03, "single_layer": True, "maximum_count": 20, "grasp_clearance_m": episode["grasp_clearance_m"]},
        "dirt_contract": {
            "patch_area_m2": episode["dirt_patch_area_m2"],
            "total_union_area_m2": dirt_union_area_m2,
            "overlap_policy": "mutually_exclusive_conservative_bounds",
            "shape_randomization": "fixed_area_rectangle_aspect_ratio",
        },
        "vehicle": {"included": include_proxy, "profile": "proxy_chassis_not_urdf" if include_proxy else None, "urdf_claim": False},
        "vehicle_start_pose_map": asdict(start_pose),
        "dynamic_pedestrians_present": bool(pedestrians),
    }
    evaluator_manifest = {
        "schema_version": 1,
        "generator_version": GENERATOR_VERSION,
        "episode_id": episode_id,
        "map_id": map_id,
        "profile": profile_name,
        "split": split,
        "map_index": map_index,
        "mission_index": mission_index,
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
    return {
        "public/world.sdf": sdf,
        "public/episode_manifest.json": json.dumps(public_manifest, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
        "evaluator/episode_manifest.json": json.dumps(evaluator_manifest, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
        "evaluator/ground_truth.json": json.dumps(truth, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
        "environment/pedestrian_schedule.json": json.dumps(runtime_schedule, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
    }
