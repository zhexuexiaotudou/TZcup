#!/usr/bin/env python3
"""Generate the project-owned exterior bodywork for the formal vehicle.

The existing formal model intentionally keeps every functional mechanism as a
separate rigid body.  This file adds the missing product-design layer: moulded
composite fairings, service doors, wheel arches, bumpers, lamps and guarded
openings.  It is dependency-free and emits deterministic binary STL files in
metres.  Collision and inertia remain authoritative in ``bodywork.xacro``.

The surfaces are lofts and extruded profiles rather than primitive URDF boxes.
That distinction is important: simple collision boxes are still desirable for
real-time simulation, while the visible vehicle must read as a coherent
manufactured outdoor machine.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable, Sequence

from generate_cleaning_storage_meshes import (
    Mesh,
    Vec3,
    add,
    box,
    cylinder,
    ring,
    sweep_tube,
    write_binary_stl,
)


def _signed_power(value: float, power: float) -> float:
    return math.copysign(abs(value) ** power, value)


def superellipse_ring(
    x: float,
    half_y: float,
    center_z: float,
    half_z: float,
    *,
    center_y: float = 0.0,
    exponent: float = 3.6,
    samples: int = 64,
) -> list[Vec3]:
    """Rounded-rectangle section in the Y-Z plane."""
    power = 2.0 / exponent
    result: list[Vec3] = []
    for index in range(samples):
        angle = 2.0 * math.pi * index / samples
        y = center_y + half_y * _signed_power(math.cos(angle), power)
        z = center_z + half_z * _signed_power(math.sin(angle), power)
        result.append((x, y, z))
    return result


def lofted_body(
    sections: Sequence[tuple[float, float, float, float, float]],
    *,
    exponent: float = 3.6,
    samples: int = 64,
) -> Mesh:
    """Loft closed rounded sections.

    Section values are ``(x, center_y, half_y, center_z, half_z)``.
    """
    rings = [
        superellipse_ring(
            x,
            half_y,
            center_z,
            half_z,
            center_y=center_y,
            exponent=exponent,
            samples=samples,
        )
        for x, center_y, half_y, center_z, half_z in sections
    ]
    mesh = Mesh()
    for row in range(len(rings) - 1):
        for index in range(samples):
            nxt = (index + 1) % samples
            # The section walks counter-clockwise when viewed from +X.  Keep
            # the loft triangles outward-facing so Ogre2's back-face culling
            # does not remove the roof and upper shoulders.  The previous
            # winding made the shell look like disconnected side plates even
            # though the STL was geometrically closed.
            mesh.tri(rings[row][index], rings[row + 1][nxt], rings[row + 1][index])
            mesh.tri(rings[row][index], rings[row][nxt], rings[row + 1][nxt])
    front_center = (
        sections[0][0],
        sections[0][1],
        sections[0][3],
    )
    rear_center = (
        sections[-1][0],
        sections[-1][1],
        sections[-1][3],
    )
    for index in range(samples):
        nxt = (index + 1) % samples
        mesh.tri(front_center, rings[0][nxt], rings[0][index])
        mesh.tri(rear_center, rings[-1][index], rings[-1][nxt])
    return mesh


def extruded_polygon_xz(points: Sequence[tuple[float, float]], y_center: float, thickness: float) -> Mesh:
    """Extrude a convex X-Z service-panel outline along Y."""
    mesh = Mesh()
    low = [(x, y_center - thickness * 0.5, z) for x, z in points]
    high = [(x, y_center + thickness * 0.5, z) for x, z in points]
    for index in range(1, len(points) - 1):
        mesh.tri(low[0], low[index + 1], low[index])
        mesh.tri(high[0], high[index], high[index + 1])
    for index in range(len(points)):
        nxt = (index + 1) % len(points)
        mesh.tri(low[index], low[nxt], high[nxt])
        mesh.tri(low[index], high[nxt], high[index])
    return mesh


def arch_band(center_x: float, y_center: float, *, outer: float = 0.206, inner: float = 0.184) -> Mesh:
    """Top wheel arch with true radial tyre clearance."""
    mesh = Mesh()
    segments = 48
    y0, y1 = y_center - 0.025, y_center + 0.025
    angles = [math.pi * index / segments for index in range(segments + 1)]
    for index in range(segments):
        a0, a1 = angles[index], angles[index + 1]
        points = {}
        for key, radius, y in (
            ("o0", outer, y0), ("o1", outer, y1),
            ("i0", inner, y0), ("i1", inner, y1),
        ):
            points[(key, 0)] = (center_x + radius * math.cos(a0), y, radius * math.sin(a0))
            points[(key, 1)] = (center_x + radius * math.cos(a1), y, radius * math.sin(a1))
        # Outer and inner curved faces.
        mesh.tri(points[("o0", 0)], points[("o0", 1)], points[("o1", 1)])
        mesh.tri(points[("o0", 0)], points[("o1", 1)], points[("o1", 0)])
        mesh.tri(points[("i0", 0)], points[("i1", 1)], points[("i0", 1)])
        mesh.tri(points[("i0", 0)], points[("i1", 0)], points[("i1", 1)])
        # Visible side ribbons.
        mesh.tri(points[("i0", 0)], points[("o0", 1)], points[("o0", 0)])
        mesh.tri(points[("i0", 0)], points[("i0", 1)], points[("o0", 1)])
        mesh.tri(points[("i1", 0)], points[("o1", 0)], points[("o1", 1)])
        mesh.tri(points[("i1", 0)], points[("o1", 1)], points[("i1", 1)])
    # Close the two ends.
    for angle in (0.0, math.pi):
        outer_low = (center_x + outer * math.cos(angle), y0, outer * math.sin(angle))
        outer_high = (center_x + outer * math.cos(angle), y1, outer * math.sin(angle))
        inner_low = (center_x + inner * math.cos(angle), y0, inner * math.sin(angle))
        inner_high = (center_x + inner * math.cos(angle), y1, inner * math.sin(angle))
        mesh.tri(outer_low, outer_high, inner_high)
        mesh.tri(outer_low, inner_high, inner_low)
    return mesh


def rounded_bar_y(length: float, depth: float, height: float, x: float, z: float) -> Mesh:
    """A bumper/light bar spanning Y with rounded plan-view ends."""
    radius = min(depth * 0.5, length * 0.22)
    result = box((depth, length - 2.0 * radius, height), (x, 0.0, z))
    for sign in (-1.0, 1.0):
        result.extend(cylinder(radius, height, (x, sign * (length * 0.5 - radius), z)))
    return result


def badge_text_strokes() -> Mesh:
    """Abstract three-stroke TZ mark; it is geometry, not a texture decal."""
    mesh = Mesh()
    for z in (0.0, 0.040):
        mesh.extend(box((0.006, 0.090, 0.012), (0.0, -0.050, z)).rotated((0, 0, math.radians(24))))
    mesh.extend(box((0.006, 0.105, 0.012), (0.0, 0.050, 0.020)).rotated((0, 0, math.radians(-24))))
    return mesh


def brush_guard(sign: float) -> Mesh:
    """Water-drop motor fairing that leaves the outer brush sector open."""
    y = sign * 0.500
    return lofted_body(
        [
            (0.315, y, 0.060, 0.055, 0.040),
            (0.380, y, 0.070, 0.065, 0.050),
            (0.445, y, 0.050, 0.060, 0.042),
        ],
        exponent=3.0,
        samples=28,
    )


def bodywork_parts() -> dict[str, Mesh]:
    parts: dict[str, Mesh] = {}

    # The split lower tub leaves the centre underside open for the roller and
    # makes service-mode inspection possible without deleting mechanisms.
    parts["lower_tub_left.stl"] = lofted_body([
        (-0.52, 0.245, 0.125, 0.135, 0.115),
        (-0.36, 0.250, 0.130, 0.145, 0.125),
        (0.30, 0.250, 0.130, 0.145, 0.125),
        (0.52, 0.235, 0.115, 0.155, 0.105),
    ])
    parts["lower_tub_right.stl"] = lofted_body([
        (-0.52, -0.245, 0.125, 0.135, 0.115),
        (-0.36, -0.250, 0.130, 0.145, 0.125),
        (0.30, -0.250, 0.130, 0.145, 0.125),
        (0.52, -0.235, 0.115, 0.155, 0.105),
    ])
    parts["front_center_nose.stl"] = lofted_body([
        (0.08, 0.0, 0.285, 0.285, 0.105),
        (0.28, 0.0, 0.345, 0.285, 0.115),
        (0.47, 0.0, 0.355, 0.275, 0.105),
        (0.555, 0.0, 0.290, 0.255, 0.075),
    ], exponent=4.2)
    parts["rear_bin_outer_shell.stl"] = lofted_body([
        (-0.555, 0.0, 0.305, 0.485, 0.245),
        (-0.490, 0.0, 0.380, 0.500, 0.275),
        (-0.310, 0.0, 0.392, 0.510, 0.290),
        (-0.075, 0.0, 0.380, 0.495, 0.275),
        (-0.025, 0.0, 0.315, 0.455, 0.225),
    ], exponent=4.0)
    parts["front_left_power_cowl.stl"] = lofted_body([
        (0.025, 0.235, 0.145, 0.505, 0.245),
        (0.120, 0.240, 0.150, 0.545, 0.285),
        (0.430, 0.235, 0.150, 0.540, 0.275),
        (0.550, 0.205, 0.120, 0.475, 0.205),
    ], exponent=3.9)
    parts["front_right_compute_cowl.stl"] = lofted_body([
        (0.110, -0.285, 0.095, 0.360, 0.115),
        (0.220, -0.290, 0.100, 0.370, 0.125),
        (0.450, -0.280, 0.100, 0.355, 0.110),
        (0.515, -0.250, 0.070, 0.335, 0.085),
    ], exponent=3.6)
    parts["sensor_pylon_fairing.stl"] = lofted_body([
        (-0.205, 0.0, 0.090, 0.690, 0.070),
        (-0.135, 0.0, 0.075, 0.820, 0.210),
        (-0.075, 0.0, 0.060, 0.755, 0.120),
    ], exponent=3.2, samples=32)

    # Wheel arches and continuous side skirts.
    for name, x, y in (
        ("front_left", 0.256, 0.355), ("front_right", 0.256, -0.355),
        ("rear_left", -0.256, 0.355), ("rear_right", -0.256, -0.355),
    ):
        parts[f"wheel_arch_{name}.stl"] = arch_band(x, y)
    parts["side_skirt_left.stl"] = extruded_polygon_xz(
        [(-0.49, -0.015), (0.49, -0.015), (0.46, 0.075), (-0.46, 0.075)], 0.365, 0.035
    )
    parts["side_skirt_right.stl"] = extruded_polygon_xz(
        [(-0.49, -0.015), (0.49, -0.015), (0.46, 0.075), (-0.46, 0.075)], -0.365, 0.035
    )

    parts["front_bumper.stl"] = rounded_bar_y(0.660, 0.055, 0.070, 0.555, 0.105)
    parts["rear_bumper.stl"] = rounded_bar_y(0.700, 0.060, 0.075, -0.555, 0.105)

    # Flush service panels and the black sensor/character line are independent
    # parts so they retain realistic seams and material contrast.
    parts["power_service_door.stl"] = extruded_polygon_xz(
        [(0.055, 0.365), (0.485, 0.365), (0.505, 0.715), (0.095, 0.790)], 0.391, 0.010
    )
    parts["compute_service_door.stl"] = extruded_polygon_xz(
        [(0.165, 0.275), (0.455, 0.275), (0.470, 0.435), (0.185, 0.455)], -0.391, 0.010
    )
    parts["wet_service_door.stl"] = extruded_polygon_xz(
        [(-0.430, 0.360), (-0.070, 0.360), (-0.060, 0.620), (-0.420, 0.650)], -0.401, 0.010
    )
    parts["rear_dry_service_door.stl"] = lofted_body([
        (-0.568, 0.0, 0.220, 0.500, 0.175),
        (-0.556, 0.0, 0.220, 0.500, 0.175),
    ], exponent=4.8, samples=32)
    left_belt = extruded_polygon_xz(
        [(-0.500, 0.570), (-0.060, 0.550), (-0.055, 0.595), (-0.490, 0.620)], 0.397, 0.010
    )
    left_belt.extend(extruded_polygon_xz(
        [(-0.070, 0.550), (0.485, 0.515), (0.490, 0.555), (-0.060, 0.595)], 0.354, 0.010
    ))
    parts["belt_line_left.stl"] = left_belt
    right_belt = extruded_polygon_xz(
        [(-0.500, 0.570), (-0.060, 0.550), (-0.055, 0.595), (-0.490, 0.620)], -0.397, 0.010
    )
    right_belt.extend(extruded_polygon_xz(
        [(-0.070, 0.550), (0.485, 0.515), (0.490, 0.555), (-0.060, 0.595)], -0.354, 0.010
    ))
    parts["belt_line_right.stl"] = right_belt
    # Keep the front sensor/light belt seated on the centre nose.  The former
    # z=0.49 belt sat above the nose crown (z~=0.365 at x=0.55), which was
    # mechanically unsupported and read as a floating black bar in Gazebo.
    parts["front_sensor_band.stl"] = extruded_polygon_xz(
        [(0.548, 0.325), (0.563, 0.325), (0.563, 0.370), (0.548, 0.382)], 0.0, 0.500
    )
    parts["rear_sensor_band.stl"] = extruded_polygon_xz(
        [(-0.560, 0.485), (-0.548, 0.485), (-0.540, 0.560), (-0.560, 0.575)], 0.0, 0.380
    )
    parts["front_green_apron.stl"] = extruded_polygon_xz(
        [(0.552, 0.275), (0.564, 0.275), (0.564, 0.315), (0.550, 0.325)], 0.0, 0.430
    )

    # Product lighting and safety hardware.
    for side, y in (("left", 0.175), ("right", -0.175)):
        parts[f"front_work_light_{side}.stl"] = box((0.012, 0.105, 0.025), (0.566, y, 0.348))
        parts[f"rear_tail_light_{side}.stl"] = box((0.012, 0.105, 0.030), (-0.563, y, 0.500))
    beacons = Mesh()
    # Each beacon follows the local roof height.  A uniform z value made the
    # front-right lamp float above the intentionally low compute cowl.
    for x, y, z in (
        (-0.420, -0.330, 0.675),
        (-0.420, 0.330, 0.675),
        (0.410, 0.330, 0.835),
        (0.410, -0.270, 0.475),
    ):
        beacons.extend(cylinder(0.018, 0.040, (x, y, z)))
        beacons.extend(ring(0.019, 0.0025, (x, y, z - 0.020)))
    parts["corner_beacons.stl"] = beacons
    parts["emergency_stop.stl"] = cylinder(0.026, 0.040, (-0.405, -0.405, 0.680), axis="y")
    parts["charge_port.stl"] = cylinder(0.032, 0.020, (0.250, 0.402, 0.330), axis="y")
    parts["drain_coupling.stl"] = cylinder(0.025, 0.065, (-0.490, -0.305, 0.315), axis="x")
    tow = Mesh()
    for y in (-0.220, 0.220):
        tow.extend(ring(0.025, 0.005, (0.570, y, 0.075), axis="y"))
        tow.extend(ring(0.025, 0.005, (-0.570, y, 0.075), axis="y"))
    parts["tow_eyes.stl"] = tow

    parts["left_side_brush_motor_guard.stl"] = brush_guard(1.0)
    parts["right_side_brush_motor_guard.stl"] = brush_guard(-1.0)
    parts["rear_squeegee_valance.stl"] = extruded_polygon_xz(
        [(-0.525, 0.050), (-0.420, 0.050), (-0.405, 0.155), (-0.525, 0.185)], 0.0, 0.780
    )
    parts["arm_turret_shoulder.stl"] = lofted_body([
        (-0.015, -0.200, 0.145, 0.305, 0.070),
        (0.100, -0.200, 0.155, 0.315, 0.080),
        (0.220, -0.200, 0.125, 0.305, 0.070),
    ], exponent=4.0, samples=32)

    # A recessed, U-shaped work bay makes the manipulator read as an integrated
    # machine module.  Its four moulded members deliberately leave the arm's
    # swept top volume open while hiding the raw payload deck and compute box.
    parts["arm_bay_floor.stl"] = lofted_body([
        (-0.135, -0.200, 0.190, 0.330, 0.050),
        (0.080, -0.200, 0.205, 0.335, 0.055),
        (0.485, -0.200, 0.185, 0.330, 0.050),
    ], exponent=4.4, samples=36)
    parts["arm_bay_outer_sill.stl"] = extruded_polygon_xz(
        [(-0.135, 0.365), (0.485, 0.365), (0.465, 0.465), (-0.115, 0.465)], -0.405, 0.040
    )
    parts["arm_bay_inner_sill.stl"] = extruded_polygon_xz(
        [(-0.120, 0.365), (0.455, 0.365), (0.435, 0.430), (-0.100, 0.430)], 0.020, 0.035
    )
    parts["arm_bay_front_sill.stl"] = rounded_bar_y(0.420, 0.045, 0.080, 0.480, 0.410).translated((0, -0.190, 0))
    parts["arm_bay_rear_sill.stl"] = rounded_bar_y(0.420, 0.045, 0.080, -0.125, 0.410).translated((0, -0.190, 0))

    # A geometric brand badge and narrow green character lines prevent the
    # vehicle from reading as one undifferentiated white volume.
    parts["front_brand_badge.stl"] = cylinder(0.040, 0.008, (0.568, 0.0, 0.335), axis="x", segments=48)
    accent = Mesh()
    accent.extend(sweep_tube([(-0.48, 0.404, 0.420), (-0.10, 0.404, 0.405), (0.42, 0.398, 0.385)], 0.007, 12))
    accent.extend(sweep_tube([(-0.48, -0.404, 0.420), (-0.10, -0.404, 0.405), (0.42, -0.398, 0.385)], 0.007, 12))
    parts["sanitation_green_accent.stl"] = accent

    return parts


def main() -> None:
    package = Path(__file__).resolve().parents[2]
    output = package / "meshes" / "project" / "bodywork"
    parts = bodywork_parts()
    for name, mesh in sorted(parts.items()):
        write_binary_stl(output / name, mesh)
    print(f"generated {len(parts)} product-bodywork meshes in {output}")


if __name__ == "__main__":
    main()
