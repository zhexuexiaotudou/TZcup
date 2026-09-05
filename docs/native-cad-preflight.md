# Native CAD export preflight

The formal vehicle retains a packaging OpenSCAD model and dependency-free
Python programs that deliberately generate visual STL meshes. They support
packaging review and simulation appearance, not editable manufacturing CAD.
The project must not relabel an STL/OBJ/DAE/glTF mesh (or a mesh converted to
STEP) as a native STEP part or assembly.

The audit can separately inventory a project-owned CadQuery parametric source
as an *editable native B-rep source input*. This is deliberately narrower than
a CAD release: the source must be a `.py` directly under
`cad/native_brep/formal_vehicle/`, import CadQuery lazily inside a callable
path, contain actual CadQuery solid/feature construction, contain no STL write
or triangulated-mesh markers, and be bound by a co-located source-manifest
SHA-256 entry. A generic `.py`, a mesh generator disguised as `.py`, or a
CadQuery source without a valid manifest/hash is excluded.

Run this Windows-only, low-memory audit without starting WSL, Docker, Gazebo,
FreeCAD, or OpenSCAD:

```powershell
py -3 scripts/audit_native_cad_readiness.py --output reports/engineering/native_cad_preflight.json
py -3 scripts/test_native_cad_readiness.py
```

The audit is intentionally fail-closed. It can declare readiness only after all
of the following evidence exists:

1. Project-owned editable B-rep source documents for the individual parts and
   the assembly. A statically qualified CadQuery source is source evidence
   only; it does not replace the following delivery gates.
2. `cad/formal_vehicle/native_cad_assembly_manifest.json`, which maps each
   assembly component to its editable native source. The checked-in
   `config/high_fidelity_vehicle/component_addressable_native_cad_assembly_manifest_draft.json`
   is intentionally only a static 105-project-part / 21-supplier-exclusion
   design-input crosswalk. When its validator passes, the audit inventories it
   and reports `NATIVE_ASSEMBLY_MANIFEST_DRAFT_NOT_RELEASED`; it never treats
   that draft as the released manifest.
3. An ISO-10303 STEP artifact exported from those sources, with components
   retained as an assembly rather than flattened into an unproven monolith.
4. A Windows-native B-rep exporter (such as `FreeCADCmd`) that the audit can
   discover, plus `cad/formal_vehicle/native_cad_export_receipt.json` binding
   that real export to the SHA-256 values of the native sources. Merely
   installing an exporter never passes the gate. The preflight rejects common
   tessellated STEP entities such as `FACETED_BREP` and
   `TRIANGULATED_FACE_SET`.

Existing STL generators remain listed as an explicit warning: they are
visual/simulation assets and can never satisfy native CAD, but their continued
presence does not permanently invalidate a separately proven native B-rep
reconstruction and export closure. All of the manifest, non-tessellated STEP,
receipt, Windows exporter, and receipt source-hash gates above remain
fail-closed.

Until every hard gate is satisfied, including the released assembly manifest
after the recognised component-addressable draft, the outcome remains `blocked`.
The source/contract validators only establish design-input and SHA-256
consistency: they do not satisfy the assembly manifest, real non-tessellated
STEP, Windows exporter, or export-receipt gates. URDF remains the
simulation/dynamics authority and the STL set remains visual-only. This
preflight is independent of the mechanical release-readiness contract, so it
records a manufacturing-CAD gap without upgrading or downgrading any current
mechanical readiness gate.
