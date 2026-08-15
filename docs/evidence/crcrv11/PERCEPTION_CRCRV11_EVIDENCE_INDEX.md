# CRCRV11 Final Evidence Index

This directory contains the compact final blocker package required by CRCRV11. It intentionally excludes failed checkpoints, generated crops, training logs, and run-by-run ledgers from the cleaned active repository.

| Evidence | Purpose |
|---|---|
| `PERCEPTION_CRCRV11_FINAL_STATUS.json` | Machine-readable stop condition B and sealed-data boundary |
| `PERCEPTION_CRCRV11_FINAL_BLOCKERS.json` | Aggregate R1/R2/R3 metrics and forbidden next actions |
| `PERCEPTION_CRCRV11_MODEL_REGISTRY.json` | Failed route registry; no selected or frozen model |
| `PERCEPTION_CRCRV11_RELEASE_MANIFEST.json` | Explicit no-release result |
| `PERCEPTION_CRCRV11_THIRD_PARTY_NOTICES.md` | Relevant framework notice |
| `CLOSE_RANGE_CLASSIFIER_CONTRACT_RECOVERY_V11_REPORT.md` | Required final questions and downstream status |

Primary provenance is closed Draft PR [#91](https://github.com/zhexuexiaotudou/TZcup/pull/91), final source commit `261f0d62d9e7bf6f844c0faf1fa72fe02486e0ce`, and its owner-authored Stop B comment. The final commit remains fetchable by exact SHA; its branch and failed training artifacts are not restored.

`G10_DEV_VAL_SEALED`, `VAL_NEW`, `G5_V2`, formal 30-seed cleaning, soak, replay, freeze, and release were not executed.
