# Task ID

`tzcup-perception-product-deploy-20260808`

<!-- HYBRID_TASK_METADATA_BEGIN
{
  "task_id": "tzcup-perception-product-deploy-20260808",
  "validation_commands": [
    {
      "executable": "git",
      "arguments": ["diff", "--check"],
      "working_directory": ".",
      "timeout_seconds": 60,
      "permission_pattern": "git diff --check"
    },
    {
      "executable": "py",
      "arguments": ["-3", "-m", "pytest", "starter_ws/src/sanitation_learning/test/test_g4_data.py", "starter_ws/src/sanitation_learning/test/test_g4_training_protocol.py", "starter_ws/src/sanitation_learning/test/test_g4_models.py", "-q"],
      "working_directory": ".",
      "timeout_seconds": 1200,
      "permission_pattern": "py -3 -m pytest"
    },
    {
      "executable": "py",
      "arguments": ["-3", "scripts/ci_fast.py"],
      "working_directory": ".",
      "timeout_seconds": 1800,
      "permission_pattern": "py -3 scripts/ci_fast.py"
    }
  ]
}
HYBRID_TASK_METADATA_END -->

# Objective

Implement the bounded P0 trustworthiness foundation for the AUTO-05R perception product-grade recovery. This task fixes data leakage and augmentation defects, makes screening validation-only and constraint-aware, adds sealed-final-set controls, provenance/freeze/manifests, task-specific ONNX parity, and machine-evaluable hard gates. It must not run long training or claim any model gate has passed.

# Relevant context

- Product worktree: `F:\Project\TZcup-perception-product`, branch `codex/perception-product-grade`, based on recovery commit `ae80e735e6e8877fc020df992970628e3dfd2c90`.
- Authoritative user specification: `C:\Users\zhexu\Downloads\TZcup_Codex提示词_感知产品级部署总推进.md`, especially P0-1 through P0-12 and the P4/P5 fixed thresholds.
- Existing recovery code lives mainly in `starter_ws/src/sanitation_learning/sanitation_learning/g4_*.py`, its tests/config, `scripts/auto05r_*.py`, and `starter_ws/src/sanitation_perception` manifests.
- The legacy G4 test split has already been observed during two failed screening attempts. It may remain available only as explicitly labelled `legacy_G4_D6_diagnostic`; it must not influence model selection, threshold selection, early stopping, hard-negative mining, or pass/fail of development screening.
- Existing micro-overfit successes prove capacity only. Preserve that interpretation and do not turn them into screening/formal/live claims.
- Every code/config change requires checking whether root `README.md` needs a truthful capability-boundary update.

# Current architecture

- `G4DiscoveryDataset` resizes full RGB frames to `DISCOVERY_MODEL_SIZE=(640,480)` and encodes CenterNet-like class-agnostic targets. Its flip branch currently contains incorrect hard-coded `384/512` output scaling.
- `g4_training.Trainer` already has validation, EMA, early-stop, checkpoint scaffolding, but the operational `scripts/auto05r_screening.py` bypasses it by configuring zero patience/load-best false and evaluates legacy test as a screening gate.
- `g4_models.py` exposes ONNX export plus generic tensor parity. The product requirement needs task-aware decoded parity.
- Model manifests are fail-closed but still have null artifacts because no product model is frozen.

# Requirements

1. Replace the discovery horizontal-flip hard-coded dimensions with reusable native-to-model and model-to-native bbox helpers derived from actual frame shape and `DISCOVERY_MODEL_SIZE`. Add deterministic round-trip/property-style tests covering corners, non-square resolutions, flipped/unflipped cases, and random valid boxes. Maximum coordinate error must be at most 0.5 px and boxes must remain ordered and bounded. Do not add a silently incorrect fixed-resolution fallback.
2. Introduce an explicit split-role policy. Development may read `train`, a deterministic train-world holdout/in-domain validation subset, `val`, and shift subsets D1-D5. The old `test` role must be renamed/represented in reports and CLI as `legacy_G4_D6_diagnostic`, with a mandatory warning and a hard assertion that it cannot contribute to training, thresholding, checkpoint selection, hard-negative mining, or screening pass/fail. Add regression tests demonstrating that changing legacy diagnostic metrics cannot change the screening decision.
3. Add a sealed G5 final-set contract and fail-closed CLI scaffolding: require at least 4 unseen worlds, 100 scenes, 1000 frames, unseen target/hard-negative assets, and a `MODEL_FREEZE.json` before any G5 annotations/manifests can be opened. The final evaluator must be one-shot: atomically record first access/evaluation and refuse rerun or partial probing. Tests may use temporary synthetic metadata and must not create or expose a real final dataset.
4. Make operational screening train with per-epoch validation, EMA, positive early stopping patience, checkpoint persistence, and `load_best=True` for discovery, classifier, leaf, and puddle. Classifier validation must use a held-out development sample set rather than the training samples. Never pass legacy/G5 data to training functions.
5. Implement constraint-aware checkpoint/model selection. A candidate that violates a hard false-positive/specificity constraint cannot win solely by lower validation loss. Persist selected epoch, selection score, violated constraints, and validation metrics. Keep selection deterministic and test it with small pure-Python cases.
6. Add task-specific ONNX parity evaluators and wire them into screening evidence: discovery decoded candidate count/box/score agreement; classifier top-1 agreement and maximum probability error; segmenter binary-mask IoU/pixel agreement plus boundary-mask agreement. Require fixed shapes, opset 17, operator inventory, and zero custom ops. Generic tensor maximum error alone is insufficient. Tests may skip only when optional ONNX dependencies are unavailable.
7. Add deterministic train-world holdout semantics and report it explicitly as in-domain validation; report `val` as cross-world validation. No metric may call the contaminated legacy diagnostic set `test` or `final`.
8. Replace hard-coded false/missing gates with real computations or fail-closed `not_evaluated` evidence for same-color specificity, D1-D5 shift suite completeness, legacy D6 diagnostic (non-gating), and G5 final (separately gated). Encode all P4 fixed thresholds in one canonical machine-readable policy and all P5 thresholds in a separate final policy. Do not lower any threshold.
9. Generate artifact manifests from an immutable frozen model configuration. Include preprocessing/postprocessing, thresholds, class map, input/output shapes, operator inventory, artifact SHA-256, config hash, freeze timestamp/id, provenance, and acceptance status. Reject missing or mismatched fields/hashes fail-closed.
10. Add required pretrained-backbone provenance contracts using official torchvision weight enums/APIs (no deprecated boolean API): source URL or identifier, exact weight enum/version, SHA-256 of acquired artifact/cache file when available, license reference, and architecture. Production candidates must fail closed when official pretrained weights cannot be acquired or verified. Explicit `from_scratch_control` may exist only as a labelled ablation and can never produce product-ready status. Do not download large weights during unit tests.
11. Add a compact committed G4 data-gate evidence directory containing only schemas, hashes, counts, split/world/asset registries, and the existing `G4_dataset_gate_pass` decision; never add raw simulator frames, bags, or model binaries. Evidence generation must be deterministic and verifiable.
12. Strengthen micro gate reporting without falsely changing historical results: discovery adds AP50, precision, and FP rate; classifier adds background/hard-negative specificity; area adds boundary F1 and task-specific negative-frame FP. Make the distinction between capacity-only micro gates and development screening explicit.
13. Add focused tests for every fail-closed boundary above and register them in `scripts/ci_fast.py` if discovery is not automatic.
14. Synchronize `README.md`, `docs/progress.md`, and a concise new P0 document with the true current boundary: P0 infrastructure implemented; no new product model trained; legacy G4 is diagnostic only; G5 remains sealed/not created; AUTO-05R/P4/P5/formal/live/J6/field claims remain false.

# Explicit non-goals

- Do not perform long model training, generate a G5 dataset, access hidden annotations, tune thresholds on legacy/G5, or claim P4/P5 passed.
- Do not implement the P2 replacement architecture, P3 full product area architecture, P6 ROS runtime chain, live simulation, J6 conversion, deployment, merge, commit, push, or PR work in this task.
- Do not modify hybrid workflow/provider configuration, invoke another AI, expose credentials, delete historical evidence, or repair the shared corrupt Git object database.
- Do not commit raw datasets, bags, model binaries, caches, checkpoints, or large artifacts.

# Existing patterns to follow

- Fail-closed manifests: `starter_ws/src/sanitation_perception/sanitation_perception/pipeline_manifest.py` and `backends.py`.
- Train/test isolation and trainer scaffolding: `starter_ws/src/sanitation_learning/sanitation_learning/g4_training.py` and `test/test_g4_training_protocol.py`.
- Dataset QA and compact evidence: `g4_qa.py`, `g4_assets.py`, `scripts/auto05r_g4_finalize_dataset.py`, and `artifacts/auto05r_g4_data_gate/` if present.
- Existing model/export contracts: `g4_models.py`, `g4_evaluation.py`, and corresponding tests.
- Repository documentation style: `docs/auto05r-*.md`, root `README.md`, `docs/progress.md`.

# Validation commands

Run exactly the commands declared in metadata. If `test_g4_data.py` does not exist yet, create it as part of this task. Also run any focused tests added for P0 before the full fast CI.

# Acceptance criteria

- All 14 requirements above are observable in code, tests, canonical policies, and docs.
- Flip transform round-trip error is <=0.5 px for tested valid boxes and dimensions.
- Development screening decision is invariant to legacy D6 diagnostic values and cannot read G5.
- Operational screening uses validation every epoch, EMA, early stopping, best checkpoint loading, held-out classifier validation, and constraint-aware selection.
- Every model type has task-specific ONNX parity evidence and zero-custom-op enforcement.
- P4 and P5 thresholds are centralized, exact, and never weakened.
- Freeze/manifest/pretrained provenance paths fail closed on missing or mismatched evidence.
- Fast CI passes (optional dependency skips are acceptable only where already policy-compliant).
- Documentation states no new gate success and no product deployment.
- No secrets, raw data, bags, model binaries, or checkpoints are added.

# Required completion report

Report changed files, implemented behavior mapped to P0-1..P0-12, exact validation results, any skips, remaining risks, and incomplete items. Include `git status --short`, `git diff --stat`, and confirm that no commit/push was performed.

# Stop conditions

Stop only when the task exceeds the user's authorization, cannot be completed safely, requires another AI, or requires changing the hybrid workflow itself. Do not stop merely because the implementation is large; complete the bounded P0 foundation and return control to GPT for inspection.
