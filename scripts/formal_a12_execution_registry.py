#!/usr/bin/env python3
"""Canonical, fail-closed AUTO-15 18x10 execution identity registry.

This registry deliberately separates the fixed AUTO-15 matrix coordinate
(``seed`` 0..9) from the public scenario generator's derived dirt seed.  The
two are currently incompatible with the legacy AUTO-15 receipt schema, which
is a pre-start blocker, not something that may be silently normalized.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile
import uuid
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
REGISTRY_SCHEMA = "tzcup.a12.execution_registry.v1"
MANIFEST_SCHEMA = "tzcup.a12.execution_manifest.v1"
CONTRACT_PATH = Path("config/high_fidelity_vehicle/product_acceptance_contract.json")
GENERATOR_PATH = Path("starter_ws/src/sanitation_campus_scenario/sanitation_campus_scenario/generator.py")
SCENARIO_CONFIG_PATH = Path("starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml")
PRODUCT_LAUNCH_PATH = Path("starter_ws/src/sanitation_product_demo_integration/launch/product_demo.launch.py")
RUNNER_PATH = Path("scripts/run_formal_single_episode_cleaning_mission.sh")


class RegistryError(RuntimeError):
    """A canonical execution cannot be admitted safely."""


# This is an inventory, not a claim that the listed component evidence can be
# replayed as a product-demo scenario.  Every row remains blocked until there
# is a scenario-selecting launch input that the product-demo runner consumes.
FEATURE_AUDIT: dict[str, dict[str, Any]] = {
    "mapping": {"entrypoints": ("scripts/run_formal_first_map_dynamic_prerequisite.sh",), "feature": "first-map lifecycle launch"},
    "full_coverage": {"entrypoints": ("scripts/run_formal_same_map_full_coverage_baseline.sh",), "feature": "saved-map FullCoverage baseline"},
    "timed_trajectory": {"entrypoints": (), "feature": "timed trajectory scenario selector"},
    "discrete_pick": {"entrypoints": (), "feature": "discrete-pick product scenario selector"},
    "leaf_pile": {"entrypoints": (), "feature": "leaf-pile product target injector"},
    "puddle": {"entrypoints": (), "feature": "wet-puddle product target injector"},
    "spot_cleaning": {"entrypoints": (), "feature": "spot-clean product target injector"},
    "dynamic_avoidance": {"entrypoints": ("scripts/run_formal_dynamic_obstacle_avoidance.sh", "scripts/prepare_formal_dynamic_obstacle_schedule.py"), "feature": "dedicated Nav2 dynamic-obstacle runner"},
    "narrow_corridor": {"entrypoints": (), "feature": "narrow-corridor product scenario selector"},
    "boundary_protection": {"entrypoints": (), "feature": "boundary-protection product scenario selector"},
    "emergency_stop": {"entrypoints": ("scripts/collect_formal_dynamic_obstacle_avoidance_runtime.py",), "feature": "synchronized emergency-stop scenario injector"},
    "app": {"entrypoints": ("scripts/auto10_formal.py",), "feature": "live product APP injection and result binding"},
    "speech": {"entrypoints": ("scripts/auto10_speech.py",), "feature": "live product speech injection and result binding"},
    "llm_dsl": {"entrypoints": ("scripts/auto10_formal.py",), "feature": "live product DSL injection and result binding"},
    "bin_full": {"entrypoints": ("starter_ws/src/sanitation_manipulation/sanitation_manipulation/formal_grasp_executor.py",), "feature": "full-bin product state injector"},
    "recovery_replay": {"entrypoints": ("scripts/auto02_replay_audit.py",), "feature": "product recovery/replay scenario injector"},
    "efficiency": {"entrypoints": ("scripts/run_formal_same_map_full_coverage_baseline.sh",), "feature": "qualified 3500 m2/h product launch profile"},
    "j6_runtime": {"entrypoints": (), "feature": "J6 runtime product launch profile"},
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RegistryError(f"cannot read JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise RegistryError(f"JSON root must be an object: {path}")
    return value


def _read_scenario_config(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise RegistryError(f"cannot read scenario config {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise RegistryError("scenario config root must be an object")
    return value


def derived_seed(master_seed: int, *parts: object) -> int:
    """Exact copy of sanitation_campus_scenario.generator._derived_seed."""
    encoded = json.dumps([master_seed, *parts], ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return int.from_bytes(hashlib.sha256(encoded.encode("utf-8")).digest()[:8], "big") % (2**31 - 2) + 1


def _source_hashes(root: Path, extra: tuple[str, ...]) -> dict[str, str]:
    paths = (CONTRACT_PATH.as_posix(), GENERATOR_PATH.as_posix(), SCENARIO_CONFIG_PATH.as_posix(), PRODUCT_LAUNCH_PATH.as_posix(), RUNNER_PATH.as_posix(), *extra)
    hashes: dict[str, str] = {}
    for relative in paths:
        candidate = (root / relative).resolve()
        if not candidate.is_file() or candidate.is_symlink():
            raise RegistryError(f"required registry source is missing or linked: {relative}")
        hashes[relative] = sha256(candidate)
    return hashes


def build_registry(repository_root: Path = ROOT) -> dict[str, Any]:
    root = repository_root.resolve()
    contract = _read_json(root / CONTRACT_PATH)
    accounting = contract.get("auto15_execution_accounting")
    if not isinstance(accounting, dict):
        raise RegistryError("product contract has no AUTO-15 execution accounting")
    scenarios, seeds = accounting.get("scenario_ids"), accounting.get("seeds")
    if not isinstance(scenarios, list) or not isinstance(seeds, list):
        raise RegistryError("product contract has malformed scenario IDs or seeds")
    if tuple(scenarios) != tuple(FEATURE_AUDIT) or seeds != list(range(10)):
        raise RegistryError("registry feature audit has drifted from the fixed product contract")
    scenario_config = _read_scenario_config(root / SCENARIO_CONFIG_PATH)
    master_seed = scenario_config.get("split", {}).get("master_seed")
    if type(master_seed) is not int:
        raise RegistryError("scenario generator master seed is malformed")

    entries: list[dict[str, Any]] = []
    for scenario_index, scenario_id in enumerate(scenarios):
        audit = FEATURE_AUDIT[scenario_id]
        for matrix_seed in seeds:
            ordinal = scenario_index * len(seeds) + matrix_seed
            map_index, mission_index = ordinal % 12, ordinal // 12
            split = "hidden"
            mission_id = f"{split}-map-{map_index:03d}-mission-{mission_index:03d}"
            entries.append({
                "scenario_id": scenario_id,
                "seed": matrix_seed,
                "split": split,
                "map_index": map_index,
                "mission_index": mission_index,
                "mission_id": mission_id,
                "mission_group_id": f"a12-group-{ordinal // 6 + 1:02d}",
                "launch_profile": "formal_single_episode_product_demo",
                "generator_dirt_seed": derived_seed(master_seed, split, map_index, mission_index, "dirt"),
                "expected_injection": {
                    "status": "BLOCKED_MISSING_A12_PRODUCT_INJECTION",
                    "required_feature": audit["feature"],
                    "existing_component_entrypoints": list(audit["entrypoints"]),
                    "reason": "No existing product_demo launch argument selects and proves this feature inside the A12 capture window.",
                },
                "source_config_hashes": _source_hashes(root, audit["entrypoints"]),
            })
    return {
        "schema": REGISTRY_SCHEMA,
        "registry_version": 1,
        "contract": {"path": CONTRACT_PATH.as_posix(), "sha256": sha256(root / CONTRACT_PATH)},
        "generator": {"path": GENERATOR_PATH.as_posix(), "sha256": sha256(root / GENERATOR_PATH), "seed_derivation": "sha256(canonical_json([master_seed, split, map_index, mission_index, role]))[:8] mod (2**31-2) + 1"},
        "entries": entries,
    }


def validate_registry(registry: dict[str, Any], repository_root: Path = ROOT) -> None:
    if registry.get("schema") != REGISTRY_SCHEMA or registry.get("registry_version") != 1:
        raise RegistryError("unsupported A12 execution registry schema/version")
    expected = build_registry(repository_root)
    if registry.get("contract") != expected["contract"] or registry.get("generator") != expected["generator"]:
        raise RegistryError("registry contract or generator provenance drifted")
    entries = registry.get("entries")
    if not isinstance(entries, list) or len(entries) != 180:
        raise RegistryError("registry must contain exactly 180 execution rows")
    expected_rows = {(row["scenario_id"], row["seed"]): row for row in expected["entries"]}
    actual_rows: dict[tuple[object, object], dict[str, Any]] = {}
    for row in entries:
        if not isinstance(row, dict):
            raise RegistryError("registry row must be an object")
        key = (row.get("scenario_id"), row.get("seed"))
        if key in actual_rows:
            raise RegistryError(f"registry collision for scenario/seed: {key}")
        actual_rows[key] = row
        if key not in expected_rows:
            raise RegistryError(f"registry row is outside the fixed contract: {key}")
        if row != expected_rows[key]:
            raise RegistryError(f"registry row drifted from canonical generator/injection audit: {key}")
    if set(actual_rows) != set(expected_rows):
        raise RegistryError("registry does not cover the complete fixed 18x10 matrix")
    groups = {row["mission_group_id"] for row in entries}
    if len(groups) < 30:
        raise RegistryError("registry needs at least 30 planned independent mission groups")
    if len({row["mission_id"] for row in entries}) != 180:
        raise RegistryError("registry mission IDs collide")


def _write_json_no_replace(path: Path, payload: dict[str, Any]) -> None:
    path = path.resolve()
    if path.exists() or path.is_symlink():
        raise RegistryError(f"refusing to overwrite retained A12 execution manifest: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("xb") as stream:
            stream.write((json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"))
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)
    except FileExistsError as exc:
        raise RegistryError(f"refusing to replace raced A12 execution manifest: {path}") from exc
    finally:
        if temporary.exists():
            temporary.unlink()


def _inside(root: Path, path: Path, label: str) -> Path:
    resolved_root, resolved = root.resolve(), path.resolve()
    try:
        resolved.relative_to(resolved_root)
    except ValueError as exc:
        raise RegistryError(f"{label} escapes A12 run root") from exc
    if any(part.is_symlink() for part in (resolved, *resolved.parents)):
        raise RegistryError(f"{label} traverses a symlink")
    return resolved


def _find_entry(registry: dict[str, Any], scenario_id: str, seed: int) -> dict[str, Any]:
    matches = [row for row in registry["entries"] if row["scenario_id"] == scenario_id and row["seed"] == seed]
    if len(matches) != 1:
        raise RegistryError("scenario/seed is not uniquely represented in the registry")
    return matches[0]


def write_execution_manifest(*, registry: dict[str, Any], repository_root: Path, run_root: Path, output: Path, scenario_id: str, seed: int, episode_manifest: Path, evaluator_manifest: Path) -> dict[str, Any]:
    validate_registry(registry, repository_root)
    run_root = run_root.resolve()
    row = _find_entry(registry, scenario_id, seed)
    if row["expected_injection"]["status"] != "AVAILABLE":
        raise RegistryError(f"{scenario_id}:seed-{seed} blocked before operator_start: {row['expected_injection']['required_feature']}")
    episode = _read_json(episode_manifest)
    evaluator = _read_json(evaluator_manifest)
    if evaluator.get("seeds", {}).get("dirt") != row["generator_dirt_seed"]:
        raise RegistryError("episode evaluator dirt seed does not match the canonical generator derivation")
    identity = episode.get("a12_execution")
    expected_identity = {key: row[key] for key in ("scenario_id", "seed", "mission_id", "mission_group_id")}
    if identity != expected_identity:
        raise RegistryError("episode does not carry the exact immutable A12 execution identity")
    # formal_product_mcap_replay currently requires a12_execution.seed == dirt.
    # Keep that incompatible historical schema as an admission boundary instead
    # of relabeling generated randomness as a matrix coordinate.
    if row["seed"] != row["generator_dirt_seed"]:
        raise RegistryError("AUTO-15 matrix seed and generator dirt seed are incompatible with the current replay receipt schema")
    destination = _inside(run_root, output, "A12 execution manifest output")
    payload = {
        "schema": MANIFEST_SCHEMA,
        "status": "A12_EXECUTION_IDENTITY_ADMITTED",
        "registry": {"schema": registry["schema"], "version": registry["registry_version"], "sha256": hashlib.sha256(json.dumps(registry, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()},
        "execution": row,
        "episode_manifest": {"path": str(episode_manifest.resolve()), "sha256": sha256(episode_manifest)},
        "evaluator_manifest": {"path": str(evaluator_manifest.resolve()), "sha256": sha256(evaluator_manifest)},
        "publication": {"method": "atomic_link_no_replace", "overwrote_existing": False},
        "run_root": str(run_root),
    }
    _write_json_no_replace(destination, payload)
    return payload


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, default=ROOT)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("validate")
    registry_out = sub.add_parser("print-registry")
    registry_out.add_argument("--output", type=Path)
    write = sub.add_parser("write-manifest")
    write.add_argument("--run-root", type=Path, required=True)
    write.add_argument("--output", type=Path, required=True)
    write.add_argument("--scenario", required=True)
    write.add_argument("--seed", type=int, required=True)
    write.add_argument("--episode-manifest", type=Path, required=True)
    write.add_argument("--evaluator-manifest", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        registry = build_registry(args.repository_root)
        validate_registry(registry, args.repository_root)
        if args.command == "validate":
            print(json.dumps({"status": "A12_EXECUTION_REGISTRY_VALID", "execution_count": len(registry["entries"]), "planned_mission_group_count": len({row["mission_group_id"] for row in registry["entries"]})}, sort_keys=True))
        elif args.command == "print-registry":
            if args.output:
                _write_json_no_replace(args.output, registry)
            else:
                print(json.dumps(registry, indent=2, sort_keys=True))
        else:
            result = write_execution_manifest(registry=registry, repository_root=args.repository_root, run_root=args.run_root, output=args.output, scenario_id=args.scenario, seed=args.seed, episode_manifest=args.episode_manifest, evaluator_manifest=args.evaluator_manifest)
            print(json.dumps(result, sort_keys=True))
    except (RegistryError, OSError, ValueError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
