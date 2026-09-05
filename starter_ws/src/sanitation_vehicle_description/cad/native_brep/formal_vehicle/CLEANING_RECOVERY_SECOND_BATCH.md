# Cleaning/recovery native B-rep second batch

`native_brep_cleaning_recovery_second_batch.py` is an independent CadQuery
source package for project-authored cleaning and recovery parts.  It obtains
all numerical inputs from
`config/high_fidelity_vehicle/native_brep_cleaning_recovery_second_batch_contract.json`.
The package is deliberately static-only at this stage: CadQuery is lazily
loaded and the current contract rejects export before a CAD kernel can load.
The source exposes both `build_design_input_shape()` for a named part and
`build_design_input_assembly()` for the named part-local package assembly;
the future export targets for both are recorded in each contract item.
`native_brep_cleaning_recovery_second_batch_source_manifest.json` binds this
editable source and the contract with SHA-256 for static drift detection.

Covered work packages are:

- side-brush disk and rotor shaft (left/right by assembly placement);
- central roller rigid core, shaft and end hubs; its bristle bars stay a
  flexible runtime/sweep model, not a manufactured B-rep claim;
- squeegee backing rail and mounting-pad envelopes;
- suction-nozzle housing envelope and outlet boss;
- recovery quick-coupling outer envelope;
- dry-deposit gate, hinge envelope and open-walled chute; and
- wastewater tank floor/pan, wall envelopes and anti-slosh baffle.

The contract cites `generate_cleaning_storage_meshes.py`, the two applicable
Xacro files, layout and BOM entries as design sources.  Their visual meshes
are evidence of existing simulation geometry only.  This source does not
read, reconstruct from, or convert those meshes.

For every package, holes, threads, material/finish, tolerances, controlled
purchased interfaces, seals, pressure/liquid performance, mass properties and
as-built inspection are explicit pending inputs.  No FCStd or STEP artifact is
created or implied.  Future release must close each component's listed export
preconditions, replace the pending state through normal review, then use a
separately approved CadQuery environment to build and export native parts and
assemblies.

Static verification only:

```powershell
py -3 -m py_compile starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_cleaning_recovery_second_batch.py
py -3 scripts/test_native_brep_cleaning_recovery_second_batch_sources.py
```
