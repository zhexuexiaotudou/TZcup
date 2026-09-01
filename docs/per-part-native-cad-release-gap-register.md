# Per-part native CAD release-gap register

`config/high_fidelity_vehicle/per_part_native_cad_release_gap_register.json` turns the eight native-B-rep contracts plus the 105-part component-addressable assembly draft into evidence-collection work items. Every project-authored part references its exact source mesh, native source, builder, primary contract, and at least two unresolved gates: the pending-export authorization and a contract-stated input gap. The 21 supplier references remain excluded and require supplier-native CAD or controlled interface evidence.

The shared gate catalog classifies only text already present in the contracts: material/finish, holes/threads/fasteners/GD&T, sealing/IP/fluid pressure, pump/isolator performance, electrical creepage/ratings, mass/inertia/CoG, FEA/fatigue/vibration, supplier interfaces, and service/design validation. Categories are work-routing labels, not asserted values or analysis results.

Run `py -3 scripts/validate_per_part_native_cad_release_gap_register.py`. A valid result remains `STATIC_PER_PART_RELEASE_GAPS_VALID_NATIVE_RELEASE_BLOCKED`: it neither creates nor proves CadQuery execution, FCStd/STEP, export receipt, material release, supplier approval, manufacturing release, or runtime acceptance.
