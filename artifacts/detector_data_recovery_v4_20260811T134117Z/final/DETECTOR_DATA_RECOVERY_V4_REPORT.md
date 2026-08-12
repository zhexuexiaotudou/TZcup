# Detector Data Recovery V4 final report

## Outcome

DDRV4 recovered the detector static gate but did not recover the online product gate. D1-B passed one-time G7 VAL, while the 24-mission, 2160-frame moving-camera regression achieved only `0.3898` eventual recall and `0.2111` product-target precision. No x86 freeze, G5_V2 access, release, merge or deployment was authorized.

## Completed evidence

- G7 detector development pack: 3200 frames, 13 worlds, 2810 instances, generator and independent reread QA passed.
- D1 static: recall/precision/macro-F1 `0.9778/0.9778/0.9777`; D1-B selected at threshold `0.53`.
- Online compatibility regression: `24` missions, `2160` frames, metal recall `0.1053`, small recall `0.3529`.
- Product performance: 300 submitted/processed frames, `9.9974` Hz, p95 `155.83` ms, drop rate `0.0000`. The strict 10 Hz gate failed.
- J6/field preflight: current official documentation was rechecked; no frozen student, installed current toolchain, board, RGB-D recording or independent map GT exists. Field software preparation is present.

## Locked work

D2/D3 were not executed because the authorized protocol sends a static D1 pass directly to DDRV4-06. Online failure blocks freeze, G5_V2, 30-seed dynamic-map/spot-clean runs, post-clean verification, soak, replay, release, J6 training/PTQ/compile and field acceptance. The neat-freak production sync gate was not run because production verification was never reached.

PR #90 remains Draft. Historical A1/A2/A3, X1/X2/X3, MRV2, OPR-A/B/C and the original G5 failure remain preserved in its body.
