# AUTO-05R-2/3 current status

Last updated: 2026-08-08

## P0 trustworthiness foundation (2026-08-08)

- P0-1..P0-12 implemented in code, tests, canonical policies and compact
  evidence; see [docs/auto05r-p0-trustworthiness.md](auto05r-p0-trustworthiness.md).
- The old G4 `test` split is only `legacy_G4_D6_diagnostic` (contaminated,
  non-gating); G5 remains sealed/not created.
- No new product model has been trained or frozen; `MODEL_FREEZE.json` does
  not exist; AUTO-05R/P4/P5/formal/live/J6/field claims remain false.

## Real evidence on repaired G4 data

- `G4_dataset_gate_pass=true`, `quality_gates_pass=true`
- negative-only frames with nonzero semantic targets: `0 / 860`
- discovery micro: `artifacts/auto05r_micro_discovery_crop_v15/micro_overfit_report.json`, pass
- classifier micro: `artifacts/auto05r_micro_classifier_v3/micro_overfit_report.json`, pass
- leaf micro: **passed** with AUTO-04-style square-crop 256 model, official IoU `0.986519`
- puddle micro: **passed** with AUTO-04-style square-crop 256 model, official IoU `0.979927`
- full-frame leaf/puddle area models still need screening-level tuning; they are not claimed as final runtime models yet
- DeepLabV3-ResNet50 300-epoch leaf run: IoU `0.923044`, still below `0.95`
- AUTO-04-style square-crop 256 leaf micro (300 epochs): IoU `0.890994`, still below `0.95`
- Final square-crop RGB AreaUNet leaf/puddle micro: passed, negative FP `0.0`

## Model and training changes

- Area segmenters use 10 input channels: RGB, depth, valid mask, height, gradient/normal (leaf) or HSV/texture (puddle)
- Area training uses deterministic balanced positive/negative batching
- Area encoder is ResNet18 when torchvision is available, with independent decoder and boundary head
- DeepLabV3-ResNet50 area segmenter is used when torchvision is available; older stage5b image falls back to the internal U-Net
- `scripts/auto05r_screening.py` implements the G4 screening train/evaluate/ONNX/report flow
- Screening smoke report: `artifacts/auto05r_screening_smoke2/auto05r_screening_report.json` (`AUTO_05R_BLOCKED=true`)

## Not claimed

No screening, formal, live, ROS, or spot-clean gate is claimed until the corresponding real gate report passes. No push/PR/CI was performed for this recovery work.

## Screening iteration evidence

- `artifacts/auto05r_screening_full_attempt1/auto05r_screening_report.json`: discovery candidate recall ~0.56, early-stopped at epoch 12.
- `artifacts/auto05r_screening_full_attempt2/auto05r_screening_report.json`: 60-epoch run still `AUTO_05R_BLOCKED=true`; val candidate recall `0.570`, test `0.608`, but false candidates/min remains tens of thousands because the full-frame objectness head produces dense high-score peaks.
- These two diagnostic runs read the G4 test split before the architecture and thresholds were frozen, and attempt 2 was informed by attempt 1. Their test metrics are therefore contaminated diagnostic evidence, not an AUTO-05R final-test result. Further model selection must use train/validation and D1-D5 only; a replacement/resealed D6 final test is required before any formal screening claim.
- A decode scan on the attempt-2 checkpoint showed threshold `0.9` cuts false positives but drops recall to ~0.07, so calibration alone cannot pass discovery.
- A grid+crop classifier proposal experiment on attempt-2 classifier produced zero candidate matches and very high false candidate counts; the current full classifier does not provide a screening-ready proposal chain either.
- Area: val/test leaf IoU already above `0.75`, but puddle and boundary/negative-area gates still fail.
