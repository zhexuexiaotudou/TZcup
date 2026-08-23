# Journey 6 source deployment bundle

This directory defines a reference-only bundle. The builder hashes prerequisites in place and writes compact manifest/status/checksum evidence; it never copies SDK archives, model weights, ONNX, HBM, BC, HBO, calibration images, or golden tensor payloads into Git.

`source_bundle_ready` is true only when all required paths exist and the machine-readable model selection, license audit, non-sealed calibration, golden tensor lock, Journey 6 toolchain lock, and nash profile matrix pass their semantic gates. The template now locks the D1 E1 canonical ONNX, the development-only Area ONNX, the C++ graph-external postprocess, and a real TRAIN-image golden tensor reference. Those structural references are not model or Area functional acceptance. Frozen model selection, release-clear licensing, calibration, nash profiles, and the official toolchain remain blocked.

The allowed TRAIN-root inventory currently exposes only 471 RGB PNG candidates and no ROI/crop files. None count as accepted calibration records without an explicit record inventory, per-file SHA-256, and complete class/scene/lighting/distance strata. Consequently `J6_CALIBRATION_PACK_READY=false` and `J6_SOURCE_DEPLOYMENT_BUNDLE_READY=false`.

Generate external evidence with:

```bash
python3 scripts/build_journey6_source_bundle.py --output-dir /tmp/tzcup-j6-source-bundle
```

Exit code 2 and `blocked_external` are expected until every source prerequisite is real and SHA-lockable.

The full calibration record contract, current evidence, commands, and readiness boundary are documented in [`docs/journey6-calibration-source-bundle.md`](../../../docs/journey6-calibration-source-bundle.md).
