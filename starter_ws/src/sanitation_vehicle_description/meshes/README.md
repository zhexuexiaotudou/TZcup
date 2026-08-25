# Formal vehicle mesh policy

Visible geometry in the formal vehicle uses either a revision-locked upstream
mesh under `vendor/` or project-owned parametric CAD under `generated/` and
`project/`.
Primitive boxes and cylinders remain permitted for collision and inertia only;
they are not accepted as the sole visible representation of an exposed
mechanical component.

`vendor/SOURCES.yaml` records upstream revisions, roles, licenses and known
fidelity boundaries. Each vendor directory carries the corresponding upstream
license. Project-owned generated meshes keep their editable generator source in
`cad/formal_vehicle/` or `scripts/generate_*mesh*.py` and must be reproducible
from that source.
