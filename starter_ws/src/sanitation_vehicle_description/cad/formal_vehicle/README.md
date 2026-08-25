# Formal vehicle CAD source

`formal_vehicle_layout.scad` is the project-owned, millimetre-based packaging
assembly used to review service clearances and sensor placement. It deliberately
mirrors the link groups in `formal_competition_vehicle.urdf.xacro`; the URDF
remains the dynamics authority.

`generate_cleaning_storage_meshes.py` is the dependency-free parametric source
for the first detailed cleaning, recovery and split-bin visual set. It generates
binary STL files under `meshes/project/{cleaning,storage}` in metre units:

```powershell
py -3 starter_ws/src/sanitation_vehicle_description/cad/formal_vehicle/generate_cleaning_storage_meshes.py
```

The generated set includes service details absent from the packaging model:
37D motor housings and flanges, gearboxes, keyed shafts, ribbed brush discs,
48 curved side-brush bundles, a four-start helical central roller, bearing
housings, a curved guard, real helical suspension springs, curved twin squeegee
blades, a shaped suction nozzle, corrugated hoses, filter and coupling hardware,
an externally detailed pump assembly, and ribbed dry/wet storage panels.

Open the file in OpenSCAD and set `$explode` to a non-zero value for an exploded
packaging view. Exported STL files are visual-only and must not replace the URDF
collision primitives or inertial data.

This is a nominal digital design, not metrology from a purchased vehicle. The
external component envelopes use published/open reference dimensions; brackets,
cleaning hardware, tanks and cabinet are project-designed parts. In particular,
the Pololu, Actuonix and Jabsco meshes model observable housings, interfaces and
mounting features only. They do not assert exact hidden gears, windings,
diaphragms, manufacturing tolerances or unit-specific calibration.
