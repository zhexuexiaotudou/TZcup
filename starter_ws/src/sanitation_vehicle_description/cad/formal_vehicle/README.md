# Formal vehicle CAD source

`formal_vehicle_layout.scad` is the project-owned, millimetre-based parametric
assembly used to review packaging, service clearances and sensor placement before
vendor mesh licensing is resolved. It deliberately mirrors the link groups in
`formal_competition_vehicle.urdf.xacro`; the URDF remains the dynamics authority.

Open the file in OpenSCAD and set `$explode` to a non-zero value for an exploded
packaging view. Exported STL files are visual-only and must not replace the URDF
collision primitives or inertial data.

This is a nominal digital design, not metrology from a purchased vehicle. The
external component envelopes use published/open reference dimensions; brackets,
cleaning hardware, tanks and cabinet are project-designed parts.
