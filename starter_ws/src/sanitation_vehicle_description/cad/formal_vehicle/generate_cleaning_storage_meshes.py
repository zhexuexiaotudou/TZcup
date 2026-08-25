#!/usr/bin/env python3
"""Generate project-owned cleaning and split-bin visual meshes.

The dimensions are in metres and follow the nominal Xacro envelopes.  These
meshes intentionally model observable housings, brackets and service hardware;
they do not claim manufacturer-internal gears, windings or pump diaphragms.
Collision and inertia remain authoritative in the Xacro files.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Sequence

Vec3 = tuple[float, float, float]


def add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def mul(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(a: Vec3) -> float:
    return math.sqrt(dot(a, a))


def unit(a: Vec3) -> Vec3:
    n = norm(a)
    if n < 1e-12:
        return (1.0, 0.0, 0.0)
    return mul(a, 1.0 / n)


@dataclass
class Mesh:
    triangles: list[tuple[Vec3, Vec3, Vec3]] = field(default_factory=list)

    def tri(self, a: Vec3, b: Vec3, c: Vec3) -> None:
        self.triangles.append((a, b, c))

    def extend(self, other: "Mesh") -> "Mesh":
        self.triangles.extend(other.triangles)
        return self

    def translated(self, xyz: Vec3) -> "Mesh":
        return Mesh([tuple(add(v, xyz) for v in tri) for tri in self.triangles])

    def rotated(self, rpy: Vec3) -> "Mesh":
        cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
        cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
        cy, sy = math.cos(rpy[2]), math.sin(rpy[2])

        def apply(v: Vec3) -> Vec3:
            x, y, z = v
            y, z = cr * y - sr * z, sr * y + cr * z
            x, z = cp * x + sp * z, -sp * x + cp * z
            x, y = cy * x - sy * y, sy * x + cy * y
            return (x, y, z)

        return Mesh([tuple(apply(v) for v in tri) for tri in self.triangles])


def box(size: Vec3, center: Vec3 = (0.0, 0.0, 0.0)) -> Mesh:
    hx, hy, hz = (s * 0.5 for s in size)
    pts = [
        add(center, (sx * hx, sy * hy, sz * hz))
        for sx, sy, sz in (
            (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
            (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1),
        )
    ]
    faces = ((0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
             (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
             (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7))
    m = Mesh()
    for a, b, c in faces:
        m.tri(pts[a], pts[b], pts[c])
    return m


def cylinder(radius: float, length: float, center: Vec3 = (0, 0, 0),
             axis: str = "z", segments: int = 32) -> Mesh:
    m = Mesh()
    z0, z1 = -length * 0.5, length * 0.5
    rings = [[(radius * math.cos(2 * math.pi * i / segments),
               radius * math.sin(2 * math.pi * i / segments), z)
              for i in range(segments)] for z in (z0, z1)]
    for i in range(segments):
        j = (i + 1) % segments
        m.tri(rings[0][i], rings[0][j], rings[1][j])
        m.tri(rings[0][i], rings[1][j], rings[1][i])
        m.tri((0, 0, z0), rings[0][j], rings[0][i])
        m.tri((0, 0, z1), rings[1][i], rings[1][j])
    if axis == "x":
        m = m.rotated((0, math.pi / 2, 0))
    elif axis == "y":
        m = m.rotated((math.pi / 2, 0, 0))
    return m.translated(center)


def ring(major_radius: float, minor_radius: float, center: Vec3 = (0, 0, 0),
         axis: str = "z", major_segments: int = 36, minor_segments: int = 10) -> Mesh:
    m = Mesh()
    points: list[list[Vec3]] = []
    for i in range(major_segments):
        a = 2 * math.pi * i / major_segments
        row = []
        for j in range(minor_segments):
            b = 2 * math.pi * j / minor_segments
            rr = major_radius + minor_radius * math.cos(b)
            row.append((rr * math.cos(a), rr * math.sin(a), minor_radius * math.sin(b)))
        points.append(row)
    for i in range(major_segments):
        ni = (i + 1) % major_segments
        for j in range(minor_segments):
            nj = (j + 1) % minor_segments
            m.tri(points[i][j], points[ni][j], points[ni][nj])
            m.tri(points[i][j], points[ni][nj], points[i][nj])
    if axis == "x":
        m = m.rotated((0, math.pi / 2, 0))
    elif axis == "y":
        m = m.rotated((math.pi / 2, 0, 0))
    return m.translated(center)


def rectangular_frustum_shell(bottom: tuple[float, float], top: tuple[float, float],
                              height: float, wall: float) -> Mesh:
    """Open rectangular hopper with sloped walls and a real through aperture."""
    m = Mesh()
    bx, by = bottom[0] * 0.5, bottom[1] * 0.5
    tx, ty = top[0] * 0.5, top[1] * 0.5
    z0, z1 = -height * 0.5, height * 0.5
    outer_bottom = [(-bx, -by, z0), (bx, -by, z0), (bx, by, z0), (-bx, by, z0)]
    outer_top = [(-tx, -ty, z1), (tx, -ty, z1), (tx, ty, z1), (-tx, ty, z1)]
    ibx, iby = bx - wall, by - wall
    itx, ity = tx - wall, ty - wall
    inner_bottom = [(-ibx, -iby, z0), (ibx, -iby, z0), (ibx, iby, z0), (-ibx, iby, z0)]
    inner_top = [(-itx, -ity, z1), (itx, -ity, z1), (itx, ity, z1), (-itx, ity, z1)]
    for i in range(4):
        j = (i + 1) % 4
        m.tri(outer_bottom[i], outer_bottom[j], outer_top[j])
        m.tri(outer_bottom[i], outer_top[j], outer_top[i])
        m.tri(inner_bottom[i], inner_top[j], inner_bottom[j])
        m.tri(inner_bottom[i], inner_top[i], inner_top[j])
        m.tri(outer_top[i], outer_top[j], inner_top[j])
        m.tri(outer_top[i], inner_top[j], inner_top[i])
        m.tri(outer_bottom[i], inner_bottom[j], outer_bottom[j])
        m.tri(outer_bottom[i], inner_bottom[i], inner_bottom[j])
    return m


def sweep_tube(points: Sequence[Vec3], radius: float, segments: int = 10) -> Mesh:
    m = Mesh()
    rings: list[list[Vec3]] = []
    previous_n = (0.0, 0.0, 1.0)
    for i, p in enumerate(points):
        if i == 0:
            tangent = unit(sub(points[1], p))
        elif i == len(points) - 1:
            tangent = unit(sub(p, points[i - 1]))
        else:
            tangent = unit(sub(points[i + 1], points[i - 1]))
        reference = previous_n if abs(dot(previous_n, tangent)) < 0.92 else (0.0, 1.0, 0.0)
        n = unit(cross(tangent, reference))
        b = unit(cross(tangent, n))
        previous_n = n
        rings.append([add(p, add(mul(n, radius * math.cos(2 * math.pi * j / segments)),
                                  mul(b, radius * math.sin(2 * math.pi * j / segments))))
                      for j in range(segments)])
    for i in range(len(rings) - 1):
        for j in range(segments):
            nj = (j + 1) % segments
            m.tri(rings[i][j], rings[i + 1][j], rings[i + 1][nj])
            m.tri(rings[i][j], rings[i + 1][nj], rings[i][nj])
    c0, c1 = points[0], points[-1]
    for j in range(segments):
        nj = (j + 1) % segments
        m.tri(c0, rings[0][nj], rings[0][j])
        m.tri(c1, rings[-1][j], rings[-1][nj])
    return m


def curved_rect_beam(length_y: float, width_x: float, height_z: float,
                     bow_x: float, segments: int = 24, z: float = 0.0) -> Mesh:
    m = Mesh()
    ys = [-length_y / 2 + length_y * i / segments for i in range(segments + 1)]
    sections: list[list[Vec3]] = []
    for y in ys:
        q = y / (length_y / 2)
        x = bow_x * (1 - q * q)
        sections.append([(x - width_x / 2, y, z - height_z / 2),
                         (x + width_x / 2, y, z - height_z / 2),
                         (x + width_x / 2, y, z + height_z / 2),
                         (x - width_x / 2, y, z + height_z / 2)])
    for i in range(segments):
        for j in range(4):
            nj = (j + 1) % 4
            m.tri(sections[i][j], sections[i + 1][j], sections[i + 1][nj])
            m.tri(sections[i][j], sections[i + 1][nj], sections[i][nj])
    for a, flip in ((sections[0], True), (sections[-1], False)):
        if flip:
            m.tri(a[0], a[2], a[1]); m.tri(a[0], a[3], a[2])
        else:
            m.tri(a[0], a[1], a[2]); m.tri(a[0], a[2], a[3])
    return m


def cylindrical_sector_y(radius_inner: float, radius_outer: float, length_y: float,
                         angle0: float, angle1: float, segments: int = 32) -> Mesh:
    m = Mesh()
    pts: dict[tuple[int, int, int], Vec3] = {}
    for i in range(segments + 1):
        a = angle0 + (angle1 - angle0) * i / segments
        for r_i, r in enumerate((radius_inner, radius_outer)):
            for y_i, y in enumerate((-length_y / 2, length_y / 2)):
                pts[i, r_i, y_i] = (r * math.cos(a), y, r * math.sin(a))
    for i in range(segments):
        for r_i, reverse in ((0, True), (1, False)):
            a, b = pts[i, r_i, 0], pts[i + 1, r_i, 0]
            c, d = pts[i + 1, r_i, 1], pts[i, r_i, 1]
            if reverse:
                m.tri(a, c, b); m.tri(a, d, c)
            else:
                m.tri(a, b, c); m.tri(a, c, d)
        for y_i, reverse in ((0, False), (1, True)):
            a, b = pts[i, 0, y_i], pts[i + 1, 0, y_i]
            c, d = pts[i + 1, 1, y_i], pts[i, 1, y_i]
            if reverse:
                m.tri(a, c, b); m.tri(a, d, c)
            else:
                m.tri(a, b, c); m.tri(a, c, d)
    for i in (0, segments):
        a, b, c, d = pts[i, 0, 0], pts[i, 0, 1], pts[i, 1, 1], pts[i, 1, 0]
        m.tri(a, b, c); m.tri(a, c, d)
    return m


def panel_with_ribs(size: Vec3, rib_axis: str = "z", rib_count: int = 4) -> Mesh:
    sx, sy, sz = size
    m = box(size)
    if sx <= min(sy, sz):
        for i in range(rib_count):
            z = -sz * 0.38 + sz * 0.76 * i / max(1, rib_count - 1)
            m.extend(box((sx + 0.006, sy * 0.92, 0.008), (0, 0, z)))
    elif sy <= min(sx, sz):
        for i in range(rib_count):
            z = -sz * 0.38 + sz * 0.76 * i / max(1, rib_count - 1)
            m.extend(box((sx * 0.92, sy + 0.006, 0.008), (0, 0, z)))
    else:
        for i in range(rib_count):
            y = -sy * 0.38 + sy * 0.76 * i / max(1, rib_count - 1)
            m.extend(box((sx * 0.92, 0.008, sz + 0.006), (0, y, 0)))
    return m


def motor_body() -> Mesh:
    m = cylinder(0.0185, 0.057)
    m.extend(cylinder(0.0195, 0.011, (0, 0, 0.034)))
    m.extend(cylinder(0.014, 0.006, (0, 0, -0.0315)))
    m.extend(box((0.020, 0.012, 0.010), (0, 0.018, 0.022)))
    for a in range(0, 360, 90):
        r = math.radians(a)
        m.extend(cylinder(0.0018, 0.004, (0.015 * math.cos(r), 0.015 * math.sin(r), -0.032)))
    return m


def gearbox() -> Mesh:
    m = cylinder(0.0185, 0.028)
    m.extend(cylinder(0.0205, 0.004, (0, 0, 0.014)))
    m.extend(cylinder(0.013, 0.004, (0, 0, -0.014)))
    for a in range(0, 360, 90):
        r = math.radians(a)
        m.extend(cylinder(0.0016, 0.006, (0.014 * math.cos(r), 0.014 * math.sin(r), 0.015)))
    return m


def shaft() -> Mesh:
    m = cylinder(0.006, 0.028, (0, 0, 0.014))
    m.extend(cylinder(0.004, 0.046, (0, 0, -0.023)))
    m.extend(box((0.006, 0.002, 0.020), (0.002, 0, -0.030)))
    return m


def side_brush_disk() -> Mesh:
    m = cylinder(0.072, 0.012)
    m.extend(cylinder(0.020, 0.028, (0, 0, 0.014)))
    m.extend(ring(0.054, 0.003, (0, 0, 0.008)))
    for i in range(8):
        a = i * math.pi / 4
        rib = box((0.050, 0.008, 0.008), (0.043, 0, 0.008)).rotated((0, 0, a))
        m.extend(rib)
    return m


def side_brush_bristles() -> Mesh:
    m = Mesh()
    for i in range(48):
        a = 2 * math.pi * i / 48
        points = []
        for j in range(6):
            t = j / 5
            # Keep the physical bundle, including its tube radius, inside the
            # conservative 150 mm sweep collision used by the Xacro.
            rr = 0.058 + 0.090 * t
            aa = a + 0.18 * t
            points.append((rr * math.cos(aa), rr * math.sin(aa), -0.004 - 0.024 * t * t))
        m.extend(sweep_tube(points, 0.0017, 7))
    return m


def cleaning_mount_frame() -> Mesh:
    m = box((0.34, 0.055, 0.045))
    for y in (-0.275, 0.275):
        m.extend(box((0.26, 0.018, 0.10), (0, y, 0.035)))
        m.extend(box((0.05, 0.055, 0.07), (-0.13, y, 0.02)))
        for x in (-0.09, 0.09):
            m.extend(cylinder(0.007, 0.022, (x, y, 0.055), axis="y"))
    return m


def guide_column() -> Mesh:
    m = cylinder(0.010, 0.180)
    m.extend(cylinder(0.016, 0.012, (0, 0, -0.084)))
    m.extend(cylinder(0.016, 0.012, (0, 0, 0.084)))
    return m


def lift_carriage() -> Mesh:
    m = box((0.46, 0.62, 0.018))
    for x in (-0.205, 0.205):
        m.extend(box((0.035, 0.56, 0.07), (x, 0, -0.035)))
    for y in (-0.285, 0.285):
        m.extend(box((0.38, 0.025, 0.055), (0, y, -0.025)))
    for x in (-0.18, 0.18):
        for y in (-0.25, 0.25):
            m.extend(cylinder(0.016, 0.035, (x, y, 0), axis="z"))
    return m


def p16_body() -> Mesh:
    m = cylinder(0.009, 0.120, (0, 0, -0.030))
    m.extend(box((0.024, 0.039, 0.055), (0, 0, 0.058)))
    m.extend(cylinder(0.012, 0.025, (0, 0, 0.094), axis="y"))
    m.extend(cylinder(0.004, 0.032, (0, 0, 0.094), axis="y"))
    m.extend(box((0.010, 0.008, 0.012), (0.014, 0, 0.060)))
    return m


def p16_rod() -> Mesh:
    m = cylinder(0.0045, 0.100, (0, 0, 0.050))
    m.extend(cylinder(0.008, 0.008, (0, 0, 0.101)))
    return m


def clevis() -> Mesh:
    m = box((0.024, 0.004, 0.020), (0, -0.006, 0))
    m.extend(box((0.024, 0.004, 0.020), (0, 0.006, 0)))
    m.extend(box((0.006, 0.016, 0.020), (-0.010, 0, 0)))
    m.extend(cylinder(0.003, 0.020, (0.006, 0, 0), axis="y"))
    return m


def lift_linkage() -> Mesh:
    m = box((0.220, 0.018, 0.025))
    for x in (-0.110, 0.110):
        m.extend(cylinder(0.015, 0.020, (x, 0, 0), axis="y"))
        m.extend(cylinder(0.006, 0.023, (x, 0, 0), axis="y"))
    return m


def bearing_housing() -> Mesh:
    m = box((0.070, 0.025, 0.022), (0, 0, -0.018))
    m.extend(cylinder(0.030, 0.025, (0, 0, 0), axis="y"))
    m.extend(cylinder(0.014, 0.028, (0, 0, 0), axis="y"))
    for x in (-0.025, 0.025):
        m.extend(cylinder(0.004, 0.028, (x, 0, -0.018), axis="y"))
    return m


def central_roller() -> Mesh:
    m = cylinder(0.060, 0.620, axis="y")
    m.extend(cylinder(0.010, 0.680, axis="y"))
    for phase in (0, math.pi / 2, math.pi, 3 * math.pi / 2):
        pts = []
        for i in range(101):
            y = -0.300 + 0.600 * i / 100
            a = phase + 4 * math.pi * i / 100
            pts.append((0.080 * math.cos(a), y, 0.080 * math.sin(a)))
        m.extend(sweep_tube(pts, 0.006, 8))
    for y in (-0.315, 0.315):
        m.extend(cylinder(0.066, 0.018, (0, y, 0), axis="y"))
    return m


def roller_guard() -> Mesh:
    m = cylindrical_sector_y(0.112, 0.120, 0.680, 0.10 * math.pi, 0.90 * math.pi)
    for y in (-0.340, 0.340):
        m.extend(box((0.24, 0.008, 0.10), (-0.02, y, 0.035)))
    m.extend(box((0.018, 0.680, 0.120), (0.105, 0, -0.025)))
    return m


def squeegee_springs() -> Mesh:
    m = Mesh()
    for y in (-0.30, 0.30):
        pts = []
        for i in range(121):
            t = i / 120
            a = 16 * math.pi * t
            pts.append((0.012 * math.cos(a), y + 0.012 * math.sin(a), 0.008 + 0.074 * t))
        m.extend(sweep_tube(pts, 0.0018, 7))
        m.extend(cylinder(0.008, 0.008, (0, y, 0.004)))
        m.extend(cylinder(0.008, 0.008, (0, y, 0.086)))
    return m


def squeegee_backing() -> Mesh:
    m = curved_rect_beam(0.800, 0.032, 0.055, 0.045, z=0.015)
    for y in (-0.36, -0.18, 0, 0.18, 0.36):
        q = y / 0.4
        x = 0.045 * (1 - q * q)
        m.extend(cylinder(0.004, 0.040, (x, y, 0.040), axis="x"))
    return m


def squeegee_blades() -> Mesh:
    m = curved_rect_beam(0.800, 0.010, 0.080, 0.045, z=-0.032).translated((0.025, 0, 0))
    m.extend(curved_rect_beam(0.800, 0.010, 0.080, 0.045, z=-0.032).translated((-0.025, 0, 0)))
    return m


def suction_nozzle() -> Mesh:
    m = curved_rect_beam(0.650, 0.100, 0.025, 0.038, z=0.010)
    m.extend(curved_rect_beam(0.650, 0.014, 0.070, 0.038, z=-0.022).translated((0.045, 0, 0)))
    m.extend(curved_rect_beam(0.650, 0.014, 0.070, 0.038, z=-0.022).translated((-0.045, 0, 0)))
    m.extend(cylinder(0.035, 0.060, (0.040, 0, 0.050)))
    m.extend(ring(0.035, 0.004, (0.040, 0, 0.080)))
    return m


def corrugated_hose(length: float) -> Mesh:
    m = cylinder(0.027, length, axis="x")
    pitch = 0.012
    count = max(2, int(length / pitch))
    for i in range(count + 1):
        x = -length / 2 + length * i / count
        m.extend(ring(0.030, 0.0028, (x, 0, 0), axis="x", major_segments=24, minor_segments=7))
    for x in (-length / 2 + 0.012, length / 2 - 0.012):
        m.extend(cylinder(0.034, 0.020, (x, 0, 0), axis="x"))
    return m


def filter_bowl() -> Mesh:
    m = cylinder(0.060, 0.090, (0, 0, -0.045))
    m.extend(ring(0.056, 0.004, (0, 0, -0.088)))
    m.extend(cylinder(0.042, 0.075, (0, 0, -0.045)))
    return m


def filter_head() -> Mesh:
    m = box((0.145, 0.095, 0.040), (0, 0, 0.015))
    m.extend(cylinder(0.018, 0.045, (0, 0.062, 0.015), axis="y"))
    m.extend(cylinder(0.018, 0.045, (0, -0.062, 0.015), axis="y"))
    for x in (-0.055, 0.055):
        for y in (-0.032, 0.032):
            m.extend(cylinder(0.003, 0.045, (x, y, 0.015)))
    return m


def pump_motor() -> Mesh:
    m = cylinder(0.055, 0.150, axis="x")
    m.extend(cylinder(0.060, 0.012, (-0.075, 0, 0), axis="x"))
    m.extend(cylinder(0.060, 0.012, (0.075, 0, 0), axis="x"))
    m.extend(box((0.060, 0.040, 0.025), (-0.015, 0, 0.060)))
    for x in (-0.050, -0.025, 0, 0.025, 0.050):
        m.extend(ring(0.056, 0.002, (x, 0, 0), axis="x", major_segments=28, minor_segments=6))
    return m


def pump_head() -> Mesh:
    m = cylinder(0.072, 0.065)
    for a in range(0, 360, 90):
        r = math.radians(a)
        m.extend(cylinder(0.030, 0.070, (0.045 * math.cos(r), 0.045 * math.sin(r), 0)))
        m.extend(cylinder(0.003, 0.074, (0.061 * math.cos(r + 0.4), 0.061 * math.sin(r + 0.4), 0)))
    for y in (-0.065, 0.065):
        m.extend(cylinder(0.017, 0.055, (0, y, 0), axis="y"))
        m.extend(ring(0.017, 0.0025, (0, y + math.copysign(0.025, y), 0), axis="y"))
    return m


def pump_rotor() -> Mesh:
    m = cylinder(0.020, 0.018)
    for a in range(0, 360, 60):
        r = math.radians(a)
        m.extend(cylinder(0.012, 0.016, (0.026 * math.cos(r), 0.026 * math.sin(r), 0)))
    m.extend(cylinder(0.006, 0.040))
    return m


def inline_flow_sensor() -> Mesh:
    m = box((0.070, 0.050, 0.045))
    m.extend(cylinder(0.016, 0.105, axis="y"))
    m.extend(box((0.030, 0.026, 0.028), (0, 0, 0.035)))
    m.extend(cylinder(0.004, 0.055, (0.024, 0, 0), axis="y"))
    m.extend(cylinder(0.004, 0.055, (-0.024, 0, 0), axis="y"))
    return m


def pump_mount() -> Mesh:
    m = box((0.225, 0.165, 0.008))
    for x in (-0.080, 0.080):
        for y in (-0.060, 0.060):
            m.extend(cylinder(0.012, 0.024, (x, y, -0.012)))
            m.extend(cylinder(0.004, 0.030, (x, y, -0.012)))
    return m


def coupling() -> Mesh:
    m = cylinder(0.018, 0.060, axis="y")
    m.extend(ring(0.020, 0.0025, (0, -0.022, 0), axis="y"))
    m.extend(ring(0.020, 0.0025, (0, 0.022, 0), axis="y"))
    return m


def storage_mount() -> Mesh:
    m = box((0.570, 0.620, 0.012))
    for x in (-0.265, 0.265):
        m.extend(box((0.040, 0.620, 0.055), (x, 0, -0.022)))
    for y in (-0.285, 0.285):
        m.extend(box((0.520, 0.026, 0.035), (0, y, -0.018)))
    for x in (-0.24, 0, 0.24):
        for y in (-0.275, 0.275):
            m.extend(cylinder(0.005, 0.018, (x, y, 0)))
    return m


def bin_floor(size: Vec3) -> Mesh:
    m = box(size)
    sx, sy, sz = size
    for y in (-sy * 0.30, 0, sy * 0.30):
        m.extend(box((sx * 0.90, 0.012, 0.012), (0, y, sz * 0.25)))
    return m


def dry_lid() -> Mesh:
    # Four panels leave a 150 x 130 mm robot-deposition aperture in the lid.
    m = box((0.508, 0.105, 0.010), (0.254, 0.143, 0))
    m.extend(box((0.508, 0.105, 0.010), (0.254, -0.143, 0)))
    m.extend(box((0.179, 0.181, 0.010), (0.0895, 0, 0)))
    m.extend(box((0.179, 0.181, 0.010), (0.4185, 0, 0)))
    m.extend(box((0.120, 0.025, 0.025), (0.300, 0, 0.035)))
    m.extend(box((0.496, 0.012, 0.012), (0.254, 0.184, -0.007)))
    m.extend(box((0.496, 0.012, 0.012), (0.254, -0.184, -0.007)))
    for x in (0.05, 0.20, 0.35, 0.48):
        m.extend(box((0.010, 0.350, 0.012), (x, 0, 0.008)))
    return m


def dry_deposit_hopper() -> Mesh:
    m = rectangular_frustum_shell((0.115, 0.095), (0.190, 0.165), 0.080, 0.006)
    for x in (-0.082, 0.082):
        for y in (-0.069, 0.069):
            m.extend(cylinder(0.004, 0.015, (x, y, -0.047)))
    return m


def dry_deposit_gate() -> Mesh:
    m = box((0.184, 0.158, 0.008), (0.092, 0, 0))
    m.extend(box((0.020, 0.120, 0.020), (0.172, 0, 0.014)))
    m.extend(cylinder(0.006, 0.170, (0.004, 0, 0), axis="y"))
    return m


def dry_deposit_chute() -> Mesh:
    m = rectangular_frustum_shell((0.130, 0.105), (0.105, 0.085), 0.350, 0.004)
    m.extend(box((0.018, 0.115, 0.355), (0.060, 0, -0.002)))
    return m


def power_distribution_box() -> Mesh:
    m = box((0.180, 0.115, 0.060))
    m.extend(box((0.190, 0.125, 0.008), (0, 0, 0.034)))
    for i in range(5):
        m.extend(box((0.018, 0.012, 0.010), (-0.060 + 0.030 * i, -0.052, 0.034)))
    for x in (-0.050, 0.050):
        m.extend(cylinder(0.008, 0.018, (x, 0.063, 0), axis="y"))
    for y in (-0.035, 0, 0.035):
        m.extend(cylinder(0.007, 0.016, (0.092, y, -0.006), axis="x"))
    return m


def isolated_dc_dc_module() -> Mesh:
    m = box((0.135, 0.095, 0.042))
    for i in range(9):
        m.extend(box((0.004, 0.075, 0.012), (-0.052 + 0.013 * i, 0, 0.026)))
    for y in (-0.030, 0.030):
        m.extend(cylinder(0.009, 0.016, (0.070, y, 0), axis="x"))
    for x in (-0.050, 0.050):
        for y in (-0.030, 0.030):
            m.extend(cylinder(0.0035, 0.008, (x, y, -0.025)))
    return m


def safety_relay() -> Mesh:
    m = box((0.105, 0.075, 0.055))
    m.extend(cylinder(0.018, 0.010, (0, 0, 0.032)))
    for x in (-0.032, 0.032):
        m.extend(cylinder(0.006, 0.014, (x, 0.040, 0), axis="y"))
    return m


def wet_lid() -> Mesh:
    m = box((0.358, 0.266, 0.010), (0.179, 0, 0))
    m.extend(cylinder(0.046, 0.028, (0.210, 0, 0.024)))
    m.extend(ring(0.046, 0.004, (0.210, 0, 0.038)))
    for x in (0.06, 0.18, 0.30):
        m.extend(box((0.010, 0.230, 0.012), (x, 0, 0.008)))
    return m


def level_sensor() -> Mesh:
    m = box((0.040, 0.055, 0.022))
    for y in (-0.014, 0.014):
        m.extend(cylinder(0.008, 0.010, (0, y, -0.015)))
        m.extend(ring(0.008, 0.0015, (0, y, -0.020)))
    m.extend(box((0.012, 0.018, 0.010), (0.026, 0, 0)))
    return m


def level_probe() -> Mesh:
    m = cylinder(0.007, 0.075)
    m.extend(cylinder(0.011, 0.015, (0, 0, 0.045)))
    m.extend(box((0.010, 0.008, 0.012), (0.012, 0, 0.047)))
    return m


def vent_filter() -> Mesh:
    m = cylinder(0.020, 0.045)
    m.extend(cylinder(0.015, 0.020, (0, 0, 0.020)))
    for z in (-0.018, -0.008, 0.002, 0.012):
        m.extend(ring(0.019, 0.0015, (0, 0, z)))
    return m


def partition() -> Mesh:
    m = panel_with_ribs((0.520, 0.012, 0.280), rib_count=5)
    m.extend(box((0.520, 0.020, 0.020), (0, 0, -0.130)))
    return m


def inlet_coupling() -> Mesh:
    m = cylinder(0.020, 0.065, axis="y")
    m.extend(cylinder(0.026, 0.045, (0, 0.045, 0), axis="y"))
    m.extend(ring(0.025, 0.003, (0, 0.060, 0), axis="y"))
    m.extend(box((0.026, 0.018, 0.036), (0.022, 0.045, 0)))
    return m


def latch() -> Mesh:
    m = box((0.035, 0.050, 0.010), (0, 0, -0.005))
    m.extend(box((0.012, 0.035, 0.030), (0.012, 0, 0.008)))
    m.extend(cylinder(0.004, 0.055, (-0.010, 0, 0), axis="y"))
    return m


def write_binary_stl(path: Path, mesh: Mesh) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        header = b"TZcup project-owned parametric visual mesh"
        f.write(header + b" " * (80 - len(header)))
        f.write(struct.pack("<I", len(mesh.triangles)))
        for a, b, c in mesh.triangles:
            n = unit(cross(sub(b, a), sub(c, a)))
            f.write(struct.pack("<12fH", *(n + a + b + c), 0))


def main() -> None:
    package = Path(__file__).resolve().parents[2]
    out = package / "meshes" / "project"
    cleaning = out / "cleaning"
    storage = out / "storage"
    platform = package / "meshes" / "generated" / "platform"
    cleaning_parts: dict[str, Mesh] = {
        "pololu_37d_motor_body.stl": motor_body(),
        "pololu_37d_gearbox.stl": gearbox(),
        "side_brush_rotor_shaft.stl": shaft(),
        "side_brush_disk.stl": side_brush_disk(),
        "side_brush_bristles.stl": side_brush_bristles(),
        "cleaning_mount_frame.stl": cleaning_mount_frame(),
        "lift_guide_column.stl": guide_column(),
        "cleaning_lift_carriage.stl": lift_carriage(),
        "actuonix_p16_body.stl": p16_body(),
        "actuonix_p16_rod.stl": p16_rod(),
        "actuonix_p16_clevis.stl": clevis(),
        "lift_linkage.stl": lift_linkage(),
        "central_roller_bearing.stl": bearing_housing(),
        "central_roller.stl": central_roller(),
        "central_roller_guard.stl": roller_guard(),
        "squeegee_float_carrier.stl": curved_rect_beam(0.840, 0.075, 0.045, 0.040),
        "squeegee_springs.stl": squeegee_springs(),
        "squeegee_backing.stl": squeegee_backing(),
        "squeegee_blades.stl": squeegee_blades(),
        "suction_nozzle.stl": suction_nozzle(),
        "recovery_hose_220.stl": corrugated_hose(0.220),
        "recovery_hose_250.stl": corrugated_hose(0.250),
        "recovery_hose_200.stl": corrugated_hose(0.200),
        "strainer_filter_bowl.stl": filter_bowl(),
        "strainer_filter_head.stl": filter_head(),
        "jabsco_pump_motor.stl": pump_motor(),
        "jabsco_pump_head.stl": pump_head(),
        "jabsco_pump_rotor.stl": pump_rotor(),
        "inline_flow_sensor.stl": inline_flow_sensor(),
        "pump_isolator_mount.stl": pump_mount(),
        "quick_coupling.stl": coupling(),
    }
    storage_parts: dict[str, Mesh] = {
        "storage_mount_tray.stl": storage_mount(),
        "dry_bin_floor.stl": bin_floor((0.508, 0.383, 0.008)),
        "dry_bin_front_panel.stl": panel_with_ribs((0.004, 0.383, 0.248)),
        "dry_bin_rear_panel.stl": panel_with_ribs((0.004, 0.383, 0.248)),
        "dry_bin_side_panel.stl": panel_with_ribs((0.500, 0.004, 0.248)),
        "dry_bin_lid.stl": dry_lid(),
        "dry_deposit_hopper.stl": dry_deposit_hopper(),
        "dry_deposit_gate.stl": dry_deposit_gate(),
        "dry_deposit_chute.stl": dry_deposit_chute(),
        "dry_bin_latch.stl": latch(),
        "dry_bin_level_sensor.stl": level_sensor(),
        "wastewater_tank_floor.stl": bin_floor((0.358, 0.258, 0.008)),
        "wastewater_front_panel.stl": panel_with_ribs((0.004, 0.258, 0.168)),
        "wastewater_rear_panel.stl": panel_with_ribs((0.004, 0.258, 0.168)),
        "wastewater_side_panel.stl": panel_with_ribs((0.350, 0.004, 0.168)),
        "wastewater_baffle.stl": panel_with_ribs((0.004, 0.230, 0.125), rib_count=3),
        "wastewater_lid.stl": wet_lid(),
        "level_probe.stl": level_probe(),
        "wastewater_vent_filter.stl": vent_filter(),
        "dry_wet_partition.stl": partition(),
        "wastewater_inlet_coupling.stl": inlet_coupling(),
    }
    platform_parts: dict[str, Mesh] = {
        "power_distribution_box.stl": power_distribution_box(),
        "isolated_dc_dc_module.stl": isolated_dc_dc_module(),
        "safety_relay.stl": safety_relay(),
    }
    for name, mesh in cleaning_parts.items():
        write_binary_stl(cleaning / name, mesh)
    for name, mesh in storage_parts.items():
        write_binary_stl(storage / name, mesh)
    for name, mesh in platform_parts.items():
        write_binary_stl(platform / name, mesh)
    print(f"generated {len(cleaning_parts) + len(storage_parts) + len(platform_parts)} meshes in {out} and {platform}")


if __name__ == "__main__":
    main()
