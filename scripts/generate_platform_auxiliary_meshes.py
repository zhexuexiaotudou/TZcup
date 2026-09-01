#!/usr/bin/env python3
"""Generate project-owned mechanical-detail meshes for platform auxiliaries.

The official A300, UR5e and 2F-85 geometry stays vendor supplied.  This script
only owns the vehicle-specific mast, adapter plate, control cabinet and compute
enclosure.  Dimensions are metres and match the corresponding URDF envelopes.
Meshes represent externally visible interface-level CAD; they do not pretend
to reproduce proprietary motor, PCB or cabinet internals.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import trimesh


def moved(mesh: trimesh.Trimesh, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> trimesh.Trimesh:
    result = mesh.copy()
    matrix = trimesh.transformations.euler_matrix(*rpy, axes="sxyz")
    matrix[:3, 3] = xyz
    result.apply_transform(matrix)
    return result


def box(extents, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> trimesh.Trimesh:
    return moved(trimesh.creation.box(extents=extents), xyz, rpy)


def cylinder(radius, height, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0), sections=48):
    return moved(trimesh.creation.cylinder(radius=radius, height=height, sections=sections), xyz, rpy)


def rounded_box(extents, radius, xyz=(0.0, 0.0, 0.0)) -> trimesh.Trimesh:
    """Closed rounded planform enclosure assembled into a single mesh asset."""
    x, y, z = extents
    parts = [box((x - 2 * radius, y, z)), box((x, y - 2 * radius, z))]
    for sx in (-1, 1):
        for sy in (-1, 1):
            parts.append(cylinder(radius, z, (sx * (x / 2 - radius), sy * (y / 2 - radius), 0)))
    mesh = trimesh.util.concatenate(parts)
    mesh.apply_translation(xyz)
    return mesh


def triangular_gusset(length, height, thickness, xyz, flip=False):
    sign = -1.0 if flip else 1.0
    vertices = np.array([
        [0, -thickness / 2, 0], [sign * length, -thickness / 2, 0], [0, -thickness / 2, height],
        [0, thickness / 2, 0], [sign * length, thickness / 2, 0], [0, thickness / 2, height],
    ], dtype=float)
    faces = np.array([
        [0, 2, 1], [3, 4, 5], [0, 1, 4], [0, 4, 3],
        [1, 2, 5], [1, 5, 4], [2, 0, 3], [2, 3, 5],
    ])
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
    mesh.apply_translation(xyz)
    return mesh


def sensor_mast() -> trimesh.Trimesh:
    """Bolted twin-column sensor tower, without any sensor-specific bracket."""
    parts = [
        # 8 mm pedestal, twin 30 x 30 mm aluminium uprights and a central
        # service spine.  This replaces the visually ambiguous T-shaped slab.
        rounded_box((0.190, 0.150, 0.016), 0.012, (0, 0, 0.008)),
        rounded_box((0.030, 0.030, 0.760), 0.004, (0, -0.050, 0.396)),
        rounded_box((0.030, 0.030, 0.760), 0.004, (0, 0.050, 0.396)),
        rounded_box((0.036, 0.062, 0.690), 0.006, (-0.022, 0, 0.375)),
        rounded_box((0.150, 0.120, 0.018), 0.008, (0, 0, 0.778)),
    ]
    # Cross ties, pedestal bolts and gussets make the load path explicit.
    for z in (0.115, 0.365, 0.615):
        parts.append(rounded_box((0.040, 0.130, 0.018), 0.004, (0, 0, z)))
    for x in (-0.070, 0.070):
        for y in (-0.052, 0.052):
            parts.append(cylinder(0.006, 0.010, (x, y, 0.021), sections=24))
    parts.extend([
        triangular_gusset(0.080, 0.105, 0.010, (0, 0.055, 0.018)),
        triangular_gusset(0.080, 0.105, 0.010, (0, -0.055, 0.018), flip=True),
    ])
    return trimesh.util.concatenate(parts)


def utm30lx_mount() -> trimesh.Trimesh:
    """Front cantilever shelf, vibration pad and side clamps for UTM-30LX."""
    parts = [
        rounded_box((0.150, 0.120, 0.014), 0.008, (0.020, 0, 0.006)),
        rounded_box((0.100, 0.090, 0.008), 0.006, (0.055, 0, 0.019)),
        box((0.014, 0.105, 0.090), (-0.048, 0, -0.030)),
    ]
    for y in (-0.047, 0.047):
        parts.append(box((0.078, 0.008, 0.030), (0.060, y, 0.038)))
    for x in (0.025, 0.085):
        for y in (-0.035, 0.035):
            parts.append(cylinder(0.004, 0.008, (x, y, 0.026), sections=20))
    return trimesh.util.concatenate(parts)


def mid360_mount() -> trimesh.Trimesh:
    """Isolated circular top plate with four vibration standoffs."""
    parts = [cylinder(0.070, 0.010, (0, 0, 0.005), sections=64)]
    # Livox mechanical drawing: four M3 points on a 48 x 36 mm rectangle.
    for x in (-0.024, 0.024):
        for y in (-0.018, 0.018):
            parts.append(cylinder(0.006, 0.026, (x, y, 0.018), sections=24))
            parts.append(cylinder(0.0035, 0.005, (x, y, 0.0335), sections=20))
    return trimesh.util.concatenate(parts)


def gnss_mount() -> trimesh.Trimesh:
    """Lateral GNSS boom and sky-view plate, separated from the LiDAR."""
    parts = [
        rounded_box((0.100, 0.230, 0.018), 0.008, (0, 0.085, 0)),
        # The ANN-MB published performance reference uses a 150 mm ground plane.
        cylinder(0.075, 0.002, (-0.020, 0.180, 0.010), sections=72),
        triangular_gusset(0.070, 0.055, 0.010, (-0.035, 0.035, -0.045)),
    ]
    for x in (-0.050, 0.020):
        for y in (0.145, 0.215):
            parts.append(cylinder(0.0035, 0.006, (x, y, 0.015), sections=20))
    return trimesh.util.concatenate(parts)


def zed_f9p_receiver_enclosure() -> trimesh.Trimesh:
    """Open service tray for one ZED-F9P module and its antenna interface.

    The 17 x 22 x 2.4 mm child module is generated as a separate public-envelope
    reference.  This project enclosure therefore models only the carrier,
    splash lip, mounting ears, cable gland and SMA bulkhead that are needed to
    install the bare LGA receiver on the vehicle.  It is not a u-blox reference
    design and does not claim an undocumented PCB or connector placement.
    """

    parts = [
        # Open-bottom tray and four perimeter lips keep the child receiver
        # visible in service renders instead of hiding it in a black box.
        rounded_box((0.062, 0.045, 0.003), 0.004, (0, 0, -0.0075)),
        box((0.062, 0.003, 0.014), (0, 0.021, -0.002)),
        box((0.062, 0.003, 0.014), (0, -0.021, -0.002)),
        box((0.003, 0.039, 0.014), (0.0295, 0, -0.002)),
        box((0.003, 0.039, 0.014), (-0.0295, 0, -0.002)),
        # Two external mounting ears and their fastener heads.
        rounded_box((0.016, 0.060, 0.003), 0.003, (0, 0, -0.0095)),
        # Coax bulkhead and sealed power/data cable gland.
        cylinder(0.0055, 0.008, (0.033, 0.012, -0.001), (0, np.pi / 2, 0), 28),
        cylinder(0.0065, 0.008, (-0.033, -0.012, -0.001), (0, np.pi / 2, 0), 28),
    ]
    for y in (-0.026, 0.026):
        parts.append(cylinder(0.0035, 0.004, (0, y, -0.007), sections=20))
    return trimesh.util.concatenate(parts)


def zed_f9p_module_reference() -> trimesh.Trimesh:
    """Recognisable ZED-F9P-04B reference inside its official package envelope.

    u-blox publishes only the 17 x 22 x 2.4 mm LGA package envelope used here.
    The top shield, pin-one mark and underside land cues stay inside that
    envelope and do not reproduce proprietary silicon or an evaluation board.
    """

    parts = [
        rounded_box((0.017, 0.022, 0.0024), 0.0010),
        rounded_box((0.0135, 0.0175, 0.00035), 0.0007, (0, 0, 0.00105)),
        cylinder(0.00075, 0.00038, (-0.0064, 0.0089, 0.00115), sections=18),
    ]
    # Edge land cues provide scale without claiming the exact 54-pad pattern.
    for y in (-0.0092, 0.0092):
        for x in np.linspace(-0.0060, 0.0060, 5):
            parts.append(box((0.0010, 0.0011, 0.00025), (x, y, -0.00120)))
    return trimesh.util.concatenate(parts)


def a300_encoder_cap() -> trimesh.Trimesh:
    """Project reference for the protected wheel-encoder end cap.

    Clearpath's public A300 description exposes wheel state but does not publish
    encoder metrology or a separable encoder CAD.  The keyed cap, connector and
    cable strain relief therefore make the physical feedback location explicit
    while remaining an engineering integration reference rather than a vendor
    dimensional claim.
    """

    parts = [
        cylinder(0.030, 0.028, sections=56),
        cylinder(0.024, 0.006, (0, 0, 0.017), sections=48),
        rounded_box((0.025, 0.018, 0.012), 0.003, (0.018, 0, 0.006)),
        cylinder(0.006, 0.020, (0.032, 0, 0.006), (0, np.pi / 2, 0), 24),
    ]
    for angle in np.linspace(0, 2 * np.pi, 4, endpoint=False):
        parts.append(cylinder(
            0.0024,
            0.004,
            (0.022 * np.cos(angle), 0.022 * np.sin(angle), 0.016),
            sections=18,
        ))
    # A shallow key rib makes left/right orientation visible after mirroring.
    parts.append(box((0.004, 0.020, 0.005), (-0.027, 0, 0.010)))
    return trimesh.util.concatenate(parts)


def front_rgbd_mount() -> trimesh.Trimesh:
    """Recessed camera bezel with a short glare hood and four fasteners."""
    parts = [
        rounded_box((0.016, 0.132, 0.067), 0.010, (-0.005, 0, 0)),
        box((0.045, 0.132, 0.008), (0.012, 0, 0.035)),
    ]
    for y in (-0.052, 0.052):
        for z in (-0.022, 0.022):
            parts.append(cylinder(0.003, 0.006, (0.006, y, z), (0, np.pi / 2, 0), 18))
    return trimesh.util.concatenate(parts)


def fisheye_mount() -> trimesh.Trimesh:
    """Rugged wedge pod that keys the rear fisheye to the body panel."""
    parts = [
        rounded_box((0.028, 0.074, 0.070), 0.010),
        box((0.020, 0.090, 0.084), (-0.018, 0, 0)),
    ]
    for y in (-0.034, 0.034):
        for z in (-0.030, 0.030):
            parts.append(cylinder(0.003, 0.006, (-0.030, y, z), (0, np.pi / 2, 0), 18))
    return trimesh.util.concatenate(parts)


def imu_mount_tray() -> trimesh.Trimesh:
    """Internal machined IMU tray with four isolators."""
    parts = [rounded_box((0.090, 0.080, 0.006), 0.008)]
    for x in (-0.033, 0.033):
        for y in (-0.028, 0.028):
            parts.append(cylinder(0.005, 0.016, (x, y, 0.011), sections=20))
    return trimesh.util.concatenate(parts)


def arm_mount() -> trimesh.Trimesh:
    # Backing plate contacts the A300 payload deck at link-local z=-50 mm;
    # the ribbed pedestal closes the former 35 mm air gap under the UR adapter.
    parts = [
        rounded_box((0.280, 0.220, 0.012), 0.015, (0, 0, -0.044)),
        rounded_box((0.180, 0.180, 0.040), 0.012, (0, 0, -0.018)),
        cylinder(0.120, 0.030),
        cylinder(0.076, 0.046, (0, 0, 0.018)),
    ]
    for angle in np.linspace(0, 2 * np.pi, 4, endpoint=False):
        gusset = triangular_gusset(0.050, 0.050, 0.010, (0, 0, -0.038))
        gusset.apply_transform(trimesh.transformations.rotation_matrix(angle, [0, 0, 1]))
        parts.append(gusset)
    for angle in np.linspace(0, 2 * np.pi, 6, endpoint=False):
        x, y = 0.096 * np.cos(angle), 0.096 * np.sin(angle)
        parts.append(cylinder(0.009, 0.008, (x, y, 0.019), sections=24))
        parts.append(box((0.060, 0.012, 0.020), (x * 0.55, y * 0.55, 0.015), (0, 0, angle)))
    return trimesh.util.concatenate(parts)


def control_box() -> trimesh.Trimesh:
    parts = [
        rounded_box((0.438, 0.240, 0.425), 0.018),
        rounded_box((0.010, 0.212, 0.382), 0.008, (0.224, 0, 0)),
        box((0.012, 0.040, 0.140), (0.228, -0.075, 0.020)),
    ]
    # Door hinges, latch, cable glands and rear heat-sink ribs.  The vehicle-
    # specific isolation base is a separate asset / rigid body so that its
    # load path is visible in URDF instead of being baked into this cabinet.
    for z in (-0.145, 0.145):
        parts.append(cylinder(0.008, 0.050, (0.222, 0.105, z), (np.pi / 2, 0, 0), 24))
    for y in (-0.075, -0.025, 0.025, 0.075):
        parts.append(cylinder(0.010, 0.014, (-0.222, y, -0.155), (0, np.pi / 2, 0), 24))
    for z in np.linspace(-0.150, 0.150, 9):
        parts.append(box((0.012, 0.200, 0.006), (-0.224, 0, z)))
    return trimesh.util.concatenate(parts)


def control_box_isolation_base() -> trimesh.Trimesh:
    """Project-designed deck rails, vibration mounts and cabinet clamps.

    The clamp footprint is deliberately selected for this virtual vehicle; it
    is not represented as a Universal Robots factory hole pattern.  Two lower
    rails bolt to the A300 deck, four elastomer cylinders isolate the cabinet,
    and four upper clamp brackets close the visible vertical load path.
    """

    parts = []
    # Two deck rails: z = 0.000 .. 0.004 m.
    for y in (-0.080, 0.080):
        parts.append(rounded_box((0.340, 0.045, 0.004), 0.006, (0, y, 0.002)))
        for x in (-0.145, 0.145):
            parts.append(cylinder(0.006, 0.004, (x, y, 0.004), sections=24))
    # Four 26 mm project-selected elastomer isolation feet.
    for x in (-0.150, 0.150):
        for y in (-0.080, 0.080):
            parts.append(cylinder(0.018, 0.026, (x, y, 0.017), sections=32))
    # Cross rails distribute the cabinet load above the isolators.
    for x in (-0.170, 0.170):
        parts.append(rounded_box((0.050, 0.220, 0.006), 0.006, (x, 0, 0.033)))
    # Four bolted clamp pedestals reach the cabinet floor at z = 0.077 m.
    for x in (-0.170, 0.170):
        for y in (-0.090, 0.090):
            parts.append(box((0.050, 0.035, 0.041), (x, y, 0.0565)))
            parts.append(cylinder(0.005, 0.004, (x, y, 0.075), sections=24))
    return trimesh.util.concatenate(parts)


def compute_enclosure() -> trimesh.Trimesh:
    parts = [
        rounded_box((0.198, 0.148, 0.078), 0.012),
        rounded_box((0.228, 0.178, 0.010), 0.008, (0, 0, -0.045)),
    ]
    # External heat-sink fins, connector bosses, fan guard and fasteners.
    for y in np.linspace(-0.058, 0.058, 9):
        parts.append(box((0.155, 0.004, 0.010), (-0.010, y, 0.044)))
    for y in (-0.045, 0, 0.045):
        parts.append(cylinder(0.010, 0.018, (0.108, y, 0), (0, np.pi / 2, 0), 24))
    parts.append(cylinder(0.036, 0.006, (-0.102, 0, 0), (0, np.pi / 2, 0), 48))
    for angle in np.linspace(0, np.pi, 5):
        parts.append(box((0.005, 0.065, 0.005), (-0.106, 0, 0), (angle, 0, 0)))
    for x in (-0.085, 0.085):
        for y in (-0.064, 0.064):
            parts.append(cylinder(0.0045, 0.006, (x, y, 0.042), sections=20))
    return trimesh.util.concatenate(parts)


def s100_cabinet_roof_mount() -> trimesh.Trimesh:
    """Ventilated roof plate and four project-defined S100 stand-offs.

    Only the public S100 external envelope is authoritative.  The stand-off
    pattern is an integration choice within the larger enclosure mounting
    plate and must not be read as a D-Robotics board-hole specification.
    """

    parts = [rounded_box((0.230, 0.180, 0.002), 0.010, (0, 0, 0.001))]
    # The 24.9 mm stand-offs bridge the remaining cabinet-roof clearance and
    # contact the underside of the compute enclosure plate at z = 0.0269 m.
    for x in (-0.085, 0.085):
        for y in (-0.065, 0.065):
            parts.append(cylinder(0.0075, 0.0249, (x, y, 0.01445), sections=32))
            parts.append(cylinder(0.0045, 0.003, (x, y, 0.0254), sections=24))
    return trimesh.util.concatenate(parts)


def board_reference() -> trimesh.Trimesh:
    """Project-authored RDK S100 exterior reference within its published envelope.

    D-Robotics publishes an overall 121 x 120 x 52.4 mm envelope.  The vendor
    STEP is deliberately not copied because its redistribution licence is not
    explicit.  These details make the installed item recognisable without
    claiming undocumented internal geometry; every primitive stays inside the
    published outer dimensions used by the URDF collision.
    """

    parts = [
        rounded_box((0.121, 0.120, 0.002), 0.002, (0, 0, -0.0252)),
        rounded_box((0.117, 0.116, 0.045), 0.004, (-0.001, 0, -0.0012)),
    ]
    # Top heat-sink crown: the final fin tips define the published 52.4 mm Z
    # extent rather than extending it with decorative geometry.
    for y in np.linspace(-0.052, 0.052, 12):
        parts.append(box((0.108, 0.0032, 0.0052), (-0.003, y, 0.0236)))
    # Recognisable I/O connector faces remain within the official X/Y outline.
    for y, sy, z, sz in [
        (-0.045, 0.020, -0.012, 0.012),
        (-0.020, 0.016, -0.012, 0.012),
        (0.005, 0.018, -0.010, 0.016),
        (0.032, 0.022, -0.011, 0.014),
    ]:
        parts.append(box((0.003, sy, sz), (0.0590, y, z)))
    # Four mounting bosses and two short antenna-I/O bosses are external-form
    # cues only; their centre locations are not mounting-hole specifications.
    for x in (-0.050, 0.050):
        for y in (-0.050, 0.050):
            parts.append(cylinder(0.0035, 0.003, (x, y, -0.0237), sections=20))
    for y in (-0.040, 0.040):
        parts.append(cylinder(0.006, 0.003, (-0.0590, y, -0.010), (0, np.pi / 2, 0), 24))
    return trimesh.util.concatenate(parts)


GENERATORS = {
    "sensor_mast.stl": sensor_mast,
    "utm30lx_mount.stl": utm30lx_mount,
    "mid360_mount.stl": mid360_mount,
    "gnss_mount.stl": gnss_mount,
    "zed_f9p_receiver_enclosure.stl": zed_f9p_receiver_enclosure,
    "zed_f9p_module_reference.stl": zed_f9p_module_reference,
    "a300_encoder_cap.stl": a300_encoder_cap,
    "front_rgbd_mount.stl": front_rgbd_mount,
    "rear_fisheye_mount.stl": fisheye_mount,
    "imu_mount_tray.stl": imu_mount_tray,
    "arm_mount_adapter.stl": arm_mount,
    "ur5e_control_cabinet_isolation_base.stl": control_box_isolation_base,
    "ur5e_control_cabinet.stl": control_box,
    "s100_cabinet_roof_mount.stl": s100_cabinet_roof_mount,
    "s100_compute_enclosure.stl": compute_enclosure,
    "s100_board_reference.stl": board_reference,
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for filename, factory in GENERATORS.items():
        mesh = factory()
        mesh.remove_unreferenced_vertices()
        mesh.export(args.output / filename)
        print(f"{filename}: {len(mesh.vertices)} vertices, {len(mesh.faces)} faces")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
