# PERCEPTION-ONLINE evidence index

## Frozen baseline

- `PERCEPTION_ONLINE_BASELINE.json`
- `artifacts/perception_online_inventory/`
- Historical `artifacts/auto05r_p*` remains read-only.
- Compact software report: `artifacts/perception_online_software_20260809T230000Z/`.

## Software verification on 2026-08-09

- Windows fast gate: `py -3 -u scripts/ci_fast.py` → `496 passed, 23 skipped`, exit 0.
- Focused online contracts: FOV, insertion, fusion, replay, scheduler, and verification →
  `35 passed`, exit 0.
- Reference/product isolation suite: `py -3 -m pytest -q reference_vision/test` →
  `9 passed`, exit 0.
- ROS container build used `tzcup/sanitation-jazzy:stage5b` image
  `sha256:418550f48916d794bc0aff144c60a3b1353d0bb0bb1dcf086cda0ec8e2a5aadc`.
  The literal `--packages-select` build first failed because the declared
  `sanitation_dataset` dependency was absent from the selected build set. Re-running with
  `--packages-up-to` built six packages and tested the requested four packages:
  `413 tests, 0 errors, 0 failures, 2 skipped`.

## Evidence boundary

These results establish software and ROS package contracts only. No frozen x86 model,
G5 access, learned moving-camera matrix, formal spot-clean matrix, two-hour soak, J6 board,
or real RGB-D field evidence was produced. Corresponding product flags remain false.
