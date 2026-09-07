# Public Gazebo DOSOD calibration to S100P compile handoff

## W6 evidence boundary (draft)

The W6 evaluation contract is an explicit `DRAFT_BLOCKED_UNTIL_PUBLIC_500_PLUS_100_EVIDENCE`, not a
competition or product acceptance result.  It permits only a NON_FORMAL engineering-model gate after a
source-disjoint public 500/100 corpus, exact sidecar labels, an identified official input adapter, and
separate raw/behaviour evaluator receipts.  The current local utilities deliberately block capture until a
ten-frame native Gazebo-to-ROS probe records the GT topic/type/frame/stamp/rate/dimensions/encoding and
publisher identity; the `hrt_model_exec` executable/CLI/output binding likewise remains an external
pre-compile prerequisite.  Neither missing fact is replaced by a fallback byte layout or by `hbrt4`.

This is an executable, fail-closed preparation specification. The scripts in
this change do not themselves launch Gazebo, install OpenExplorer, invoke
`hb_compile`, copy to a board, SSH, or control a robot. The current user goal
does authorize the later protected remote verification/compilation and approved
deployment sequence; those actions remain gated by a fresh W1 source closure,
exclusive Gazebo scheduling, and their live evidence checks. Evaluator access,
hidden split access, replay, and control remain prohibited.

## Public train scene plan

[`config/public_gazebo_dosod_train_scene_plan.json`](../config/public_gazebo_dosod_train_scene_plan.json)
contains 20 calibration scenes and four holdout scenes. With the frozen common
quota of 25 unique tensors per scene it has exactly 500 calibration and 100
holdout samples. The sets are disjoint by scene ID. All IDs are valid `train`
indices under `default_scenario.yaml` (32 maps, 200 missions each), and the
spread over maps 0--23 and missions 0--191 is deliberate background/layout and
randomization diversity; it is not a claim that a target is visible.

The runner must receive that plan verbatim and the explicit bridge-backed pair:

```bash
PUBLIC_GAZEBO_CALIBRATION_PLAN=config/public_gazebo_dosod_train_scene_plan.json
PUBLIC_GAZEBO_CALIBRATION_IMAGE_TOPIC=/camera/color/image_raw
PUBLIC_GAZEBO_CALIBRATION_CAMERA_INFO_TOPIC=/camera/color/camera_info
PUBLIC_GAZEBO_CALIBRATION_PER_SCENE_QUOTA=25
```

The pair is authorized by
`formal_campus_integration.yaml`: native
`/sensors/front_rgbd/depth/image_rect_raw/image` maps to the RGB topic and its
native CameraInfo maps to the CameraInfo topic. The collector rejects any other
pair and requires equal frame IDs and timestamps. The source runner additionally
requires a fresh empty run root, a shared lock path, frozen setup files, a
whole-run timeout, and an isolated `ROS_DOMAIN_ID`; see
[`scripts/run_public_gazebo_dosod_calibration.sh`](../scripts/run_public_gazebo_dosod_calibration.sh).

Before committing to all 24 scenes, run the explicit `pilot` mode on the fixed
first calibration scene `map-0-mission-0` with quota exactly 25. It writes only
`pilot_manifest.json` and `pilot_contact_sheet.png`, with status
`NON_FORMAL_PILOT_CAPTURED` and `formal_passed=false`; it never writes
`calibration_manifest.json` or a full `FROZEN` result. Its 25 accepted frames must have distinct source and
tensor hashes, valid fresh Image/CameraInfo pairings, and a public-RGB contact
sheet review that demonstrates material scene/view change. Sensor noise or
timestamp-only changes do not establish viewpoint coverage. A stationary view
or ambiguous review blocks expansion; do not manufacture diversity by adding
noise or altering tensors.

After collection, inspect only the frozen public RGB tensors and their public
provenance/scene records. Produce a review receipt with per-class visible-frame
counts for `litter_cube`, `fallen_leaves`, `dust_or_soil`, and `puddle`, plus
the scene/background and viewpoint buckets actually observed. The full collector
requires an explicit approved receipt bound by SHA-256 to `pilot_manifest.json`:
all four class counts must be positive, background and material-view review must
be true, and at least one named manual or agent reviewer must explicitly approve. `class_ids` in a plan or manifest is
not visibility evidence. A zero count, unknown class, missing provenance, or
unreviewed tensor blocks compiler admission. No evaluator, ground-truth, hidden
file, replay, or control topic may supply this review.

## Current collector-to-compiler boundary

The output of PR #137 is **not** a direct input to the canonical S100P compile
validator. This is intentional fail-closed evidence, not a workaround:

- The collector manifest at `RUN_ROOT/dataset/calibration_manifest.json` has
  public provenance, selector nonces, episode-manifest hashes, calibration
  `records`, and retained `holdout_records`; it does not provide required
  `model_sha256`, `vocabulary_sha256`, or `records_sha256`.
- It deliberately writes holdout tensors under `holdout_samples/`. Canonical
  `audit_calibration()` inventories every `.npy` under its calibration directory
  and rejects those unregistered holdout tensors as
  `calibration_directory_manifest_set_mismatch`.
- It also writes public provenance JSON. That is not itself rejected, but the
  consumer expects the compiler calibration directory to contain exactly the
  manifest-registered calibration `.npy` set.

Do not point `--calibration-dir` at this dataset and do not move/delete the
holdout files to bypass the check. Use
`scripts/stage_public_gazebo_dosod_calibration_for_compile.py` to make the
fresh compiler-only directory: it binds the fixed model/vocabulary hashes,
computes canonical `records_sha256`, copies only registered calibration tensors,
and re-audits them. It never alters the original collector or retained holdout
tree. A failure leaves the compiler stage `BLOCKED`; a successful staging
receipt is still not an OE compile or board acceptance.

## Exact future compile queue

### Preprocess sensitivity is a live-oracle blocker

The W7 state machine is `RAW_BOUND -> NON_FORMAL_ORACLE_CANDIDATE_COMPILED ->
OFFICIAL_PREPROCESS_CAPTURED -> ORACLE_VERIFIED -> DATASET_FROZEN_500_100 ->
FORMAL_HBM_COMPILED`.  The candidate receipt is explicitly non-formal and is
rejected by formal compile, parity, metric and S100 admission consumers.  The
official-capture producer only seals externally generated official Y/UV and
identity/log evidence; it never generates, approximates or simulates Y/UV.
Until a real official capture, the oracle remains `BLOCKED`; test fixtures can
only be `TEST_FIXTURE_BLOCKED`.

Candidate calibration reuses the existing public-Gazebo pilot collector's
canonical 25 `samples/*.npy` tensors and its raw/provenance/contact closure;
there is no new materializer and no synthetic tensor substitute. Its fixed
`BOOTSTRAP_SYMMETRIC_BLACK_V1` route is non-final. The candidate only runs when
that closure and the current canonical collector producer identity re-audit;
its exact `cal_data_dir` is the pilot `samples` directory.

A local CPU ONNX Runtime 1.24.4 synthetic probe (fixed 848x480 input; model
SHA-256 prefix `30e4...e516`) found that preprocessing choice is material:
symmetrical-black versus centre-114 produced scores cosine/nRMSE
`0.714642`/`0.699508` and boxes cosine/nRMSE/MAE `0.996771`/`0.080315`/`14.015`
model pixels; the top-left RGB-zero proxy produced `0.303557`/`1.215239` and
`0.994137`/`0.108312`/`24.539`, respectively. This synthetic result proves
sensitivity only. It is neither an accuracy result nor a preprocessing-route
decision: the official hobotcv/NV12/color-range/HBM single-frame oracle remains
required. Raw sensor bytes and their source metadata are retained so every
derived preprocessing candidate can be recomputed after that oracle is frozen.

All paths below are bindings, not a command to execute from this local
worktree. Before protected remote compilation, read-only verify the selected
remote OpenExplorer environment and its assets: OE 3.7.0 candidates are
`.work/toolchains/oe-3.7.0-r055-20260905T203200Z/{venv,venv-cpu-r2,extracted}`
and the NVIDIA/cuDNN help-probed environment is
`.work/toolchains/oe-3.7.0-r055-r3-nvidia-cudnn-20260906T050900Z/venv-hbhelp-r3`.
The selected Linux x86_64 environment must expose `hb_compile` on `PATH` and
match `hbdk4_compiler==4.7.5`, `hmct==2.6.5`, and
`horizon_tc_ui==3.5.3`. Frozen repository discovery/help artifacts are a
binding baseline, not proof that this local isolation tree or the remote asset
roots are unavailable. Verify remote model/upstream assets through the
protected helper; do not install or guess a toolchain path.

After a fresh OE environment has been independently admitted, stage the frozen
collector tree, then execute in order:

```bash
python3 scripts/run_dosod_single_frame_preprocessing_oracle_supervised.py \
  --candidate-receipt <fresh_candidate_receipt> \
  --official-capture-receipt <fresh_official_capture_receipt> \
  --onnx-model .work/formal_perception_assets/dosod/dosod_mlp3x_s_tzcup_rep.onnx \
  --hrt-model-exec <absolute_verified_oe_hrt_model_exec> \
  --output <fresh_oracle_supervision_output>

python3 scripts/collect_dosod_s100p_compiler_identity.py \
  --toolchain-discovery artifacts/autonomous_auto14_20260730_evidence/toolchain_discovery.json \
  --output .work/dosod_s100p_compiler_identity.json

python3 scripts/stage_public_gazebo_dosod_calibration_for_compile.py \
  --source-dataset <fresh_public_collector_run>/dataset \
  --output <compiler_calibration_dir> \
  --contract config/dosod_s100p_hbm_compile_contract.json

python3 scripts/validate_dosod_s100p_hbm_compile_contract.py \
  --repository-root . \
  --artifact-root .work/formal_perception_assets \
  --upstream-root .work/perception_upstreams/dosod_pc \
  --calibration-dir <compiler_calibration_dir> \
  --compiler-identity .work/dosod_s100p_compiler_identity.json \
  --preprocessing-oracle <fresh_oracle_finalizer>

python3 scripts/auto14_onnx_preflight.py \
  --model .work/formal_perception_assets/dosod/dosod_mlp3x_s_tzcup_rep.onnx \
  --calibration-dir <compiler_calibration_dir> \
  --output-dir .work/dosod_s100p_compile \
  --model-name dosod_mlp3x_s_tzcup_rep \
  --repository-root . \
  --artifact-root .work/formal_perception_assets \
  --upstream-root .work/perception_upstreams/dosod_pc \
  --compiler-identity .work/dosod_s100p_compiler_identity.json \
  --preprocessing-oracle <fresh_oracle_finalizer> \
  --march nash-m --jobs 1

python3 scripts/execute_dosod_hbm_compile.py \
  --contract config/dosod_s100p_hbm_compile_contract.json \
  --preflight-report .work/dosod_s100p_compile/dosod_mlp3x_s_tzcup_rep_preflight.json \
  --compile-config .work/dosod_s100p_compile/dosod_mlp3x_s_tzcup_rep_config.yaml \
  --compiler-identity .work/dosod_s100p_compiler_identity.json \
  --calibration-manifest <compiler_calibration_dir>/calibration_manifest.json \
  --preprocessing-oracle <fresh_oracle_finalizer> \
  --output <fresh_compile_evidence_dir> \
  --compiler hb_compile
```

`<fresh_oracle_finalizer>` is exactly
`<fresh_oracle_supervision_output>/dosod_single_frame_preprocessing_oracle_supervision_receipt.json`.
It is the outer-watchdog-supervised finalizer accepted by the canonical
preprocessing-oracle validator. The supervisor owns one 180-second outer
deadline and the 9 GiB group-RSS watchdog; its retained raw child under
`<fresh_oracle_supervision_output>/collector/` is never a formal compile input.
Each preflight and compile step revalidates only the finalizer through the
canonical validator; the preflight records its SHA-256, and the compile receipt
binds the supplied finalizer path and SHA-256 exactly.

`<absolute_verified_oe_hrt_model_exec>` is the absolute resolved executable
built from the admitted OE/x86 bundle. Its identity, hash, and `model_info`
evidence must match this oracle contract. `/usr/hobot/bin/hrt_model_exec` is
board-equivalent evidence only and must not be substituted for the OE/x86
single-frame oracle.
After `hb_compile` returns, the compile step revalidates the same finalizer and
rejects any SHA-256 or canonical-validation drift while retaining the generated
HBM only as blocked evidence.

The contract locks the four-class ONNX/vocabulary/reparameterization hashes,
float32 `[1,3,640,640]` RGB NCHW calibration tensor format, `nash-m`, RGB
training input to NV12/NHWC runtime input, scale `1/255`, and int16 output
prefix. A successful compiler receipt is only `COMPILED_NOT_BOARD_ACCEPTED`;
x86 Nash parity, quantized metric regression, and board-runtime acceptance
remain separate required gates.
