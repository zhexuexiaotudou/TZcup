#!/usr/bin/env python3
"""Fail closed unless expanded SDF serializes the side-brush surface contract."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


EXPECTED = {
    "pose_z_m": -0.065,
    "radius_m": 0.150,
    "length_m": 0.026,
    "mu": 0.08,
    "mu2": 0.08,
    "kp": 1500.0,
    "kd": 50.0,
    "max_vel": 0.20,
    "min_depth": 0.003,
}

CENTRAL_EXPECTED = {
    "pose_z_m": 0.0,
    "radius_m": 0.100,
    "length_m": 0.620,
    "mu": 0.08,
    "mu2": 0.08,
    "kp": 1500.0,
    "kd": 50.0,
    "max_vel": 0.20,
    "min_depth": 0.003,
}


class SideBrushSdfSurfaceError(RuntimeError):
    """Raised when xacro expansion or the expanded-SDF contract fails."""


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _float_text(element: ET.Element | None, path: str) -> float:
    if element is None or element.text is None:
        raise SideBrushSdfSurfaceError(f"missing expanded-SDF value: {path}")
    try:
        value = float(element.text.strip())
    except ValueError as exc:
        raise SideBrushSdfSurfaceError(
            f"expanded-SDF value is not numeric: {path}={element.text!r}"
        ) from exc
    if not math.isfinite(value):
        raise SideBrushSdfSurfaceError(f"expanded-SDF value is not finite: {path}")
    return value


def _same(actual: float, expected: float, label: str) -> None:
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9):
        raise SideBrushSdfSurfaceError(
            f"{label} differs: expected {expected:.9f}, found {actual:.9f}"
        )


def _one(root: ET.Element, path: str, label: str) -> ET.Element:
    matches = root.findall(path)
    if len(matches) != 1:
        raise SideBrushSdfSurfaceError(
            f"expected exactly one {label}, found {len(matches)}"
        )
    return matches[0]


def validate_expanded_sdf_text(sdf_text: str) -> dict[str, Any]:
    try:
        root = ET.fromstring(sdf_text)
    except ET.ParseError as exc:
        raise SideBrushSdfSurfaceError(f"expanded SDF is not valid XML: {exc}") from exc

    sides: dict[str, Any] = {}
    for side in ("left", "right"):
        link_name = f"{side}_side_brush_link"
        collision_name = f"{link_name}_collision"
        link = _one(root, f".//link[@name='{link_name}']", f"link {link_name}")
        collisions = link.findall("collision")
        collision = _one(
            link,
            f"collision[@name='{collision_name}']",
            f"default-named direct sweep collision {collision_name}",
        )
        if not collisions or collisions[0] is not collision:
            raise SideBrushSdfSurfaceError(
                f"{collision_name} must be the first direct collision on {link_name}"
            )

        pose = (collision.findtext("pose") or "").split()
        if len(pose) != 6:
            raise SideBrushSdfSurfaceError(
                f"{collision_name} must have a six-value SDF pose"
            )
        try:
            pose_values = [float(value) for value in pose]
        except ValueError as exc:
            raise SideBrushSdfSurfaceError(
                f"{collision_name} pose contains a non-numeric value"
            ) from exc
        _same(pose_values[2], EXPECTED["pose_z_m"], f"{collision_name} pose z")

        radius = _float_text(collision.find("geometry/cylinder/radius"), f"{collision_name}/radius")
        length = _float_text(collision.find("geometry/cylinder/length"), f"{collision_name}/length")
        _same(radius, EXPECTED["radius_m"], f"{collision_name} radius")
        _same(length, EXPECTED["length_m"], f"{collision_name} length")

        surface = collision.find("surface")
        if surface is None:
            raise SideBrushSdfSurfaceError(
                f"{collision_name} has no surface; URDF Gazebo contact parameters were dropped"
            )
        values = {
            "mu": _float_text(surface.find("friction/ode/mu"), f"{collision_name}/surface/mu"),
            "mu2": _float_text(surface.find("friction/ode/mu2"), f"{collision_name}/surface/mu2"),
            "kp": _float_text(surface.find("contact/ode/kp"), f"{collision_name}/surface/kp"),
            "kd": _float_text(surface.find("contact/ode/kd"), f"{collision_name}/surface/kd"),
            "max_vel": _float_text(
                surface.find("contact/ode/max_vel"), f"{collision_name}/surface/max_vel"
            ),
            "min_depth": _float_text(
                surface.find("contact/ode/min_depth"), f"{collision_name}/surface/min_depth"
            ),
        }
        for key, value in values.items():
            _same(value, EXPECTED[key], f"{collision_name} {key}")

        joint = _one(root, f".//joint[@name='{side}_side_brush_joint']", f"joint {side}_side_brush_joint")
        if (joint.findtext("child") or "").strip() != link_name:
            raise SideBrushSdfSurfaceError(
                f"{side}_side_brush_joint must drive {link_name} in expanded SDF"
            )
        sides[side] = {
            "link": link_name,
            "joint": f"{side}_side_brush_joint",
            "collision": collision_name,
            "collision_index": collisions.index(collision),
            "collision_count_on_link": len(collisions),
            "pose_z_m": pose_values[2],
            "radius_m": radius,
            "length_m": length,
            "surface": values,
        }

    central_link_name = "central_roller_link"
    central_collision_name = f"{central_link_name}_collision"
    central_link = _one(
        root, f".//link[@name='{central_link_name}']", f"link {central_link_name}"
    )
    central_collisions = central_link.findall("collision")
    central_collision = _one(
        central_link,
        f"collision[@name='{central_collision_name}']",
        f"default-named direct sweep collision {central_collision_name}",
    )
    if not central_collisions or central_collisions[0] is not central_collision:
        raise SideBrushSdfSurfaceError(
            f"{central_collision_name} must be the first direct collision on {central_link_name}"
        )
    central_pose = (central_collision.findtext("pose") or "").split()
    if len(central_pose) != 6:
        raise SideBrushSdfSurfaceError(
            f"{central_collision_name} must have a six-value SDF pose"
        )
    try:
        central_pose_values = [float(value) for value in central_pose]
    except ValueError as exc:
        raise SideBrushSdfSurfaceError(
            f"{central_collision_name} pose contains a non-numeric value"
        ) from exc
    _same(
        central_pose_values[2],
        CENTRAL_EXPECTED["pose_z_m"],
        f"{central_collision_name} pose z",
    )
    central_radius = _float_text(
        central_collision.find("geometry/cylinder/radius"),
        f"{central_collision_name}/radius",
    )
    central_length = _float_text(
        central_collision.find("geometry/cylinder/length"),
        f"{central_collision_name}/length",
    )
    _same(
        central_radius,
        CENTRAL_EXPECTED["radius_m"],
        f"{central_collision_name} radius",
    )
    _same(
        central_length,
        CENTRAL_EXPECTED["length_m"],
        f"{central_collision_name} length",
    )
    central_surface = central_collision.find("surface")
    if central_surface is None:
        raise SideBrushSdfSurfaceError(
            f"{central_collision_name} has no surface; URDF Gazebo contact parameters were dropped"
        )
    central_values = {
        "mu": _float_text(
            central_surface.find("friction/ode/mu"),
            f"{central_collision_name}/surface/mu",
        ),
        "mu2": _float_text(
            central_surface.find("friction/ode/mu2"),
            f"{central_collision_name}/surface/mu2",
        ),
        "kp": _float_text(
            central_surface.find("contact/ode/kp"),
            f"{central_collision_name}/surface/kp",
        ),
        "kd": _float_text(
            central_surface.find("contact/ode/kd"),
            f"{central_collision_name}/surface/kd",
        ),
        "max_vel": _float_text(
            central_surface.find("contact/ode/max_vel"),
            f"{central_collision_name}/surface/max_vel",
        ),
        "min_depth": _float_text(
            central_surface.find("contact/ode/min_depth"),
            f"{central_collision_name}/surface/min_depth",
        ),
    }
    for key, value in central_values.items():
        _same(value, CENTRAL_EXPECTED[key], f"{central_collision_name} {key}")
    central_joint = _one(
        root, ".//joint[@name='central_roller_joint']", "joint central_roller_joint"
    )
    if (central_joint.findtext("child") or "").strip() != central_link_name:
        raise SideBrushSdfSurfaceError(
            f"central_roller_joint must drive {central_link_name} in expanded SDF"
        )
    central = {
        "link": central_link_name,
        "joint": "central_roller_joint",
        "collision": central_collision_name,
        "collision_index": central_collisions.index(central_collision),
        "collision_count_on_link": len(central_collisions),
        "pose_z_m": central_pose_values[2],
        "radius_m": central_radius,
        "length_m": central_length,
        "surface": central_values,
    }

    return {
        "schema_version": 2,
        "status": "FORMAL_SIDE_BRUSH_EXPANDED_SDF_SURFACE_PASSED",
        "expected": dict(EXPECTED),
        "central_expected": dict(CENTRAL_EXPECTED),
        "sides": sides,
        "central_roller": central,
        "runtime_effectiveness": {
            "dart_effective_from_surface_friction_ode": ["mu", "mu2"],
            "serialized_but_not_consumed_by_gz_physics_7_dart": [
                "kp",
                "kd",
                "max_vel",
                "min_depth",
            ],
            "dynamic_acceptance_required": (
                "same-update signed measured_speed/current/temperature/fault telemetry for side brushes and central roller"
            ),
        },
        "claim_boundary": (
            "This gate proves xacro-to-sdformat serialization. With gz-physics 7 "
            "DART, only friction ode mu/mu2 from these side-brush and central-roller "
            "bristle-sweep proxies are runtime-effective; "
            "contact ode kp/kd/max_vel/min_depth are preserved evidence, not a "
            "claim of DART compliance."
        ),
    }


def expand_vehicle_xacro(vehicle_xacro: Path) -> tuple[str, str]:
    if not vehicle_xacro.is_file():
        raise SideBrushSdfSurfaceError(f"vehicle xacro does not exist: {vehicle_xacro}")
    try:
        expanded_urdf = subprocess.run(
            ["xacro", str(vehicle_xacro)],
            check=True,
            capture_output=True,
            text=True,
            timeout=30.0,
        ).stdout
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".urdf", encoding="utf-8"
        ) as temporary_urdf:
            temporary_urdf.write(expanded_urdf)
            temporary_urdf.flush()
            expanded_sdf = subprocess.run(
                ["gz", "sdf", "-p", temporary_urdf.name],
                check=True,
                capture_output=True,
                text=True,
                timeout=30.0,
            ).stdout
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        raise SideBrushSdfSurfaceError(f"xacro -> gz sdf -p expansion failed: {exc}") from exc
    return expanded_urdf, expanded_sdf


def build_report(
    *, vehicle_xacro: Path | None = None, expanded_sdf_path: Path | None = None
) -> dict[str, Any]:
    if (vehicle_xacro is None) == (expanded_sdf_path is None):
        raise SideBrushSdfSurfaceError(
            "provide exactly one of vehicle_xacro or expanded_sdf_path"
        )
    if vehicle_xacro is not None:
        source_bytes = vehicle_xacro.read_bytes()
        expanded_urdf, expanded_sdf = expand_vehicle_xacro(vehicle_xacro)
        source = {
            "mode": "xacro_to_gz_sdf",
            "path": str(vehicle_xacro.resolve()),
            "sha256": _sha256_bytes(source_bytes),
            "expanded_urdf_sha256": _sha256_bytes(expanded_urdf.encode("utf-8")),
        }
    else:
        assert expanded_sdf_path is not None
        expanded_sdf = expanded_sdf_path.read_text(encoding="utf-8")
        source = {
            "mode": "preexpanded_sdf",
            "path": str(expanded_sdf_path.resolve()),
            "sha256": _sha256_bytes(expanded_sdf.encode("utf-8")),
        }
    report = validate_expanded_sdf_text(expanded_sdf)
    report["source"] = source
    report["expanded_sdf_sha256"] = _sha256_bytes(expanded_sdf.encode("utf-8"))
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--vehicle-xacro", type=Path)
    source.add_argument("--expanded-sdf", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    try:
        report = build_report(
            vehicle_xacro=args.vehicle_xacro,
            expanded_sdf_path=args.expanded_sdf,
        )
        exit_code = 0
    except (OSError, SideBrushSdfSurfaceError) as exc:
        report = {
            "schema_version": 1,
            "status": "FORMAL_SIDE_BRUSH_EXPANDED_SDF_SURFACE_FAILED",
            "error": str(exc),
        }
        exit_code = 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
