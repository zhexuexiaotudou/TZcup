# Component-addressable native CAD assembly draft

`config/high_fidelity_vehicle/component_addressable_native_cad_assembly_manifest_draft.json` is a static crosswalk for all 105 project-authored reconstruction-manifest parts. Each row names its exact source mesh, registered batch contract, CadQuery source file, and AST-verifiable builder symbol. The 21 vendor reference rows remain explicitly excluded and require supplier-native CAD or controlled supplier interface data; they are not reconstructed as project B-rep.

Run `py -3 scripts/validate_component_addressable_native_cad_assembly_draft.py` for a JSON-only/hash/AST check. A valid result is deliberately `STATIC_COMPONENT_ADDRESSABLE_DRAFT_VALID_NATIVE_EXPORT_BLOCKED`, not an assembly release. Every row stays `design_input_pending_native_export`; no CadQuery execution, FCStd/STEP artifact, Windows exporter evidence, or assembly/export receipt exists. Those omissions remain hard blockers for native CAD delivery.
