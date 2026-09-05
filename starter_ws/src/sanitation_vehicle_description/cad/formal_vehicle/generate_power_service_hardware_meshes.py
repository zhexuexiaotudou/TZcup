#!/usr/bin/env python3
"""Generate deterministic product CAD for power and service mechanisms.

The shapes expose the parts a technician or simulator must interact with:
battery case ribs and terminals, BMS connector face, hinged charge door and
lock pin, latching E-stop plunger, and the complete wastewater drain train.
URDF keeps simple collision solids for real-time physics; these meshes are the
manufactured visual layer and do not claim hidden vendor-internal geometry.
"""

from __future__ import annotations

import math
from pathlib import Path

from generate_cleaning_storage_meshes import Mesh, box, cylinder, ring, sweep_tube, write_binary_stl


def battery_case() -> Mesh:
    mesh = box((0.330, 0.135, 0.135))
    for x in (-0.135, -0.090, -0.045, 0.0, 0.045, 0.090, 0.135):
        mesh.extend(box((0.008, 0.143, 0.010), (x, 0, 0.0725)))
    mesh.extend(box((0.100, 0.030, 0.018), (-0.070, 0, 0.0765)))
    for y in (-0.035, 0.035):
        mesh.extend(cylinder(0.010, 0.018, (0.120, y, 0.076), segments=24))
    return mesh


def bms_module() -> Mesh:
    mesh = box((0.065, 0.105, 0.025))
    mesh.extend(box((0.028, 0.112, 0.012), (0.020, 0, 0.017)))
    for y in (-0.035, 0.0, 0.035):
        mesh.extend(cylinder(0.004, 0.010, (-0.030, y, 0.016), segments=16))
    return mesh


def charge_housing() -> Mesh:
    mesh = cylinder(0.0375, 0.020, axis="y", segments=48)
    mesh.extend(ring(0.031, 0.004, (0, -0.012, 0), axis="y", major_segments=40))
    mesh.extend(box((0.018, 0.028, 0.018), (0, 0, 0.041)))
    return mesh


def charge_door() -> Mesh:
    mesh = cylinder(0.036, 0.006, axis="y", segments=48)
    mesh.extend(box((0.014, 0.012, 0.018), (0, 0, 0.041)))
    mesh.extend(box((0.022, 0.010, 0.010), (0, -0.005, -0.031)))
    return mesh


def charge_receptacle() -> Mesh:
    mesh = cylinder(0.029, 0.025, axis="y", segments=48)
    mesh.extend(ring(0.022, 0.003, (0, -0.015, 0), axis="y", major_segments=36))
    for angle in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0):
        mesh.extend(cylinder(0.004, 0.010,
                             (0.014 * math.cos(angle), -0.018, 0.014 * math.sin(angle)),
                             axis="y", segments=16))
    return mesh


def estop_housing() -> Mesh:
    mesh = cylinder(0.024, 0.032, axis="y", segments=40)
    mesh.extend(ring(0.025, 0.003, (0, -0.018, 0), axis="y"))
    mesh.extend(box((0.040, 0.010, 0.050), (0, 0.018, 0)))
    return mesh


def estop_plunger() -> Mesh:
    mesh = cylinder(0.026, 0.026, axis="y", segments=48)
    mesh.extend(cylinder(0.020, 0.012, (0, -0.018, 0), axis="y", segments=48))
    mesh.extend(ring(0.020, 0.003, (0, -0.025, 0), axis="y", major_segments=40))
    return mesh


def main_power_isolator_housing() -> Mesh:
    """IP65 rotary-isolator body with panel flange and cable glands."""

    mesh = box((0.092, 0.050, 0.106))
    mesh.extend(box((0.106, 0.012, 0.120), (0, 0.031, 0)))
    mesh.extend(cylinder(0.020, 0.020, (0, 0.047, 0), axis="y", segments=36))
    for x in (-0.031, 0.031):
        mesh.extend(cylinder(0.009, 0.018, (x, 0.033, -0.060), axis="z", segments=24))
    return mesh


def main_power_isolator_handle() -> Mesh:
    """Red/yellow lockable rotary handle and steel shaft."""

    mesh = cylinder(0.014, 0.042, axis="y", segments=32)
    mesh.extend(box((0.018, 0.018, 0.082), (0, 0.027, 0.020)))
    mesh.extend(box((0.036, 0.022, 0.022), (0, 0.027, 0.055)))
    mesh.extend(cylinder(0.006, 0.024, (0.018, 0.027, 0.055), axis="x", segments=20))
    return mesh


def main_contactor_housing() -> Mesh:
    """Sealed traction contactor body with bus studs and mounting feet."""

    mesh = box((0.116, 0.080, 0.092))
    mesh.extend(box((0.138, 0.094, 0.008), (0, 0, -0.050)))
    for x in (-0.035, 0.035):
        mesh.extend(cylinder(0.009, 0.024, (x, 0, 0.058), axis="z", segments=24))
        mesh.extend(cylinder(0.014, 0.006, (x, 0, 0.070), axis="z", segments=24))
    mesh.extend(box((0.052, 0.020, 0.018), (0, -0.050, -0.015)))
    return mesh


def main_contactor_armature() -> Mesh:
    """Visible plunger/armature witness for the normally-open contactor."""

    mesh = cylinder(0.013, 0.034, axis="z", segments=28)
    mesh.extend(box((0.050, 0.018, 0.010), (0, 0, 0.021)))
    return mesh


def drain_pipe() -> Mesh:
    return sweep_tube([(0, 0, 0), (-0.045, 0, 0), (-0.085, 0, 0)], 0.015, 20)


def drain_valve_body() -> Mesh:
    mesh = cylinder(0.028, 0.070, axis="x", segments=48)
    mesh.extend(ring(0.031, 0.004, (-0.035, 0, 0), axis="x"))
    mesh.extend(ring(0.031, 0.004, (0.035, 0, 0), axis="x"))
    mesh.extend(box((0.050, 0.045, 0.022), (0, 0, 0.033)))
    return mesh


def drain_ball() -> Mesh:
    # Externally visible stem/indicator; the sealed ball remains inside body.
    mesh = cylinder(0.008, 0.050, (0, 0, 0.020), segments=24)
    mesh.extend(box((0.050, 0.012, 0.009), (0.020, 0, 0.046)))
    return mesh


def drain_actuator() -> Mesh:
    mesh = box((0.095, 0.060, 0.070))
    mesh.extend(cylinder(0.018, 0.022, (-0.050, 0, 0), axis="x", segments=28))
    mesh.extend(box((0.055, 0.050, 0.014), (0, 0, -0.042)))
    return mesh


def drain_cap() -> Mesh:
    mesh = cylinder(0.029, 0.018, axis="x", segments=40)
    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        mesh.extend(box((0.020, 0.010, 0.010), (0, 0.027 * math.cos(angle), 0.027 * math.sin(angle))))
    return mesh


def drain_coupling() -> Mesh:
    mesh = cylinder(0.025, 0.060, axis="x", segments=40)
    mesh.extend(ring(0.027, 0.003, (0.020, 0, 0), axis="x"))
    mesh.extend(ring(0.027, 0.003, (-0.020, 0, 0), axis="x"))
    return mesh


def main() -> None:
    package = Path(__file__).resolve().parents[2]
    output = package / "meshes" / "project" / "service"
    parts = {
        "a300_battery_pack.stl": battery_case(),
        "a300_battery_bms.stl": bms_module(),
        "charge_port_housing.stl": charge_housing(),
        "charge_port_door.stl": charge_door(),
        "charge_receptacle.stl": charge_receptacle(),
        "charge_connector_lock.stl": box((0.018, 0.012, 0.012)),
        "emergency_stop_housing.stl": estop_housing(),
        "emergency_stop_plunger.stl": estop_plunger(),
        "main_power_isolator_housing.stl": main_power_isolator_housing(),
        "main_power_isolator_handle.stl": main_power_isolator_handle(),
        "main_contactor_housing.stl": main_contactor_housing(),
        "main_contactor_armature.stl": main_contactor_armature(),
        "wastewater_drain_pipe.stl": drain_pipe(),
        "wastewater_drain_valve_body.stl": drain_valve_body(),
        "wastewater_drain_valve_ball.stl": drain_ball(),
        "wastewater_drain_actuator.stl": drain_actuator(),
        "wastewater_drain_service_cap.stl": drain_cap(),
        "wastewater_drain_coupling.stl": drain_coupling(),
    }
    for name, mesh in sorted(parts.items()):
        write_binary_stl(output / name, mesh)
    print(f"generated {len(parts)} power/service meshes in {output}")


if __name__ == "__main__":
    main()
