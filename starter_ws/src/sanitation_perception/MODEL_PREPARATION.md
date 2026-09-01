# Formal PC perception artifacts (diagnostic/reference path)

This document prepares the PC comparison path only. It is not an S100P
deployment prerequisite. Board-first execution, the selected EdgeSAM 512 HBM
roles, and the real-board evidence boundary are defined in
`docs/s100p-board-first-execution-boundary.md` and
`docs/formal-s100-live-acceptance.md`.

This package uses a four-class, no-NMS DOSOD ONNX detector followed by a
box-prompted EdgeSAM-3x encoder/decoder.  The artifacts are not committed.  A
runtime directory must contain `artifact_manifest.json` and these files:

- `dosod/dosod_mlp3x_s_tzcup_rep.onnx`
- `dosod/tzcup_offline_vocabulary.json`
- `edgesam/edge_sam_3x_encoder.onnx`
- `edgesam/edge_sam_3x_decoder.onnx`

The frozen DOSOD prompt order is `small litter cube`, `fallen leaves`, `dust
patch`, `puddle`.  Synonyms remain in the JSON for provenance, but the locked
upstream script embeds the first text in each row.

## Locked sources and licenses

- DOSOD: `D-Robotics-AI-Lab/DOSOD@c50129b5badf6ed7bb85e692ab493d8bdb58da6a`, GPL-3.0.
- EdgeSAM: `chongzhou96/EdgeSAM@d24d99671f41a9c0003061248bded64a481e9059`, NTU S-Lab License 1.0.  It is a non-commercial research license; a commercial vehicle needs separate permission or a replacement segmenter.
- DOSOD-S checkpoint: `https://huggingface.co/D-Robotics/DOSOD/resolve/main/dosod_mlp3x_s.pth`; the successful mirror download was `https://modelscope.cn/models/D-Robotics/DOSOD/resolve/master/dosod_mlp3x_s.pth`, whose response ETag matched the SHA-256 below.
- EdgeSAM-3x ONNX: `https://huggingface.co/spaces/chongzhou/EdgeSAM/resolve/main/weights/edge_sam_3x_{encoder,decoder}.onnx`.

The verified source hashes from the 2026-08-26 preparation are:

| Artifact | Bytes | SHA-256 |
|---|---:|---|
| `dosod_mlp3x_s.pth` | 865127910 | `6878367c584d50306fc39bdb5bac6cbf2abd4d4b91d48624cc5797cf289ee4a3` |
| DOSOD project ONNX | 45430396 | `30e4da2516b7a18cc3dbb4b20572e99f07c28a0c08111055a8c14265a992e516` |
| EdgeSAM encoder | 22098300 | `719a498cf5b3fe9be9f01ee513e13d3915f9028aa4f23dfd30eaaa0a17143159` |
| EdgeSAM decoder | 15937006 | `83a2174d54571596913dcb7455d021e713623c3dca30a31c8c41ab98c9fb0863` |

## PC runtime dependencies

The formal PC target is **Ubuntu 24.04 with ROS Jazzy and Python 3.12**.  The
formal contract accepts Python 3.10 through 3.12 only and rejects Python 3.13
or later before it attempts model loading.  Keep the frozen dependency pins
below; do not replace them with NumPy 2 or a newer ONNX Runtime merely to make
a Python 3.13 workstation installable.

Install the PC adapter dependencies into an isolated environment before the
formal preflight:

```bash
python -m pip install -r starter_ws/src/sanitation_perception/requirements-pc.txt
```

The PC path uses the headless OpenCV wheel because DOSOD/EdgeSAM resize camera
rasters and masks through `cv2`.  The ROS package keeps the separate
`python3-opencv` rosdep/system dependency; do not install the PC wheel as a
substitute on the RDK S100P image.  The formal preflight imports `cv2` and fails
closed with `pc_opencv_unavailable` when the selected PC interpreter is not
ready.

## DOSOD reproduction

Use Python 3.10, CPU PyTorch 2.0.0, torchvision 0.15.1, the official CPU
`mmcv-2.0.0` wheel, mmdet 3.0.0, mmyolo 0.6.0 and mmengine 0.10.3.  Clone the
locked source and CLIP `openai/clip-vit-base-patch32`, change only the source
config's `model_name` to the local CLIP directory, and set the rep config's
`num_training_classes = 4`.  Then run from the DOSOD repository:

```bash
export PYTHONPATH="$PWD:$PWD/deploy"
python tools/generate_text_prompts_dosod.py \
  configs/dosod/dosod_mlp3x_s_100e_1x8gpus_obj365v1_goldg_train_lvis_minival.py \
  "$ARTIFACT_ROOT/dosod/dosod_mlp3x_s.pth" \
  --text "$ARTIFACT_ROOT/dosod/tzcup_offline_vocabulary.json" \
  --device cpu --out-dir "$ARTIFACT_ROOT/dosod"
python tools/reparameterize_dosod.py \
  --model "$ARTIFACT_ROOT/dosod/dosod_mlp3x_s.pth" \
  --text-embed "$ARTIFACT_ROOT/dosod/tzcup_offline_vocabulary_dosod_mlp3x_s.npy" \
  --out-dir "$ARTIFACT_ROOT/dosod"
python deploy/export_onnx.py \
  configs/dosod/rep_dosod_mlp3x_s_100e_1x8gpus_obj365v1_goldg_train_lvis_minival.py \
  "$ARTIFACT_ROOT/dosod/dosod_mlp3x_s_rep.pth" \
  --without-nms --device cpu --work-dir "$ARTIFACT_ROOT/dosod"
```

Rename the produced ONNX to `dosod_mlp3x_s_tzcup_rep.onnx`, calculate hashes,
and write the manifest with the exact source revisions above.  Run the formal
preflight before launch.  The product executable refuses an empty/missing
artifact root and validates ONNX input/output signatures on startup.

## Random Gazebo-camera accuracy acceptance

The reproducible evaluator uses three disjoint formal `val` episodes. Each
episode contains 20 randomly coloured 3 cm cubes whose cardboard/PP/PET/
aluminium mass is not visually exposed, 18 rotated fixed-area leaf/dust/puddle
patches, eight moving pedestrians and 120 static campus assets. The
evaluator-only process reads `evaluator/ground_truth.json` to score results.
Before sampling it places the already-randomized episode objects in a fixed
vehicle-relative visibility ROI; it never teleports the vehicle, so the Gazebo
camera pose and `map -> odom -> base_link` chain remain physically consistent.
It publishes no truth topic. The product node still receives only the public
occupancy map, TF, RGB, depth and CameraInfo topics.

```bash
TZCUP_REPOSITORY_ROOT=/mnt/f/Project/TZcup-integrated-functional-acceptance \
bash scripts/run_formal_random_scene_perception.sh
```

Frozen per-episode gates are cube precision/recall/F1 >= 0.80 at 0.50 box IoU,
false positives <= 0.20 per evaluated frame, ground-dirt IoU >= 0.65 and recall
>= 0.85, map projection RMSE <= 0.20 m and P95 <= 0.35 m, plus all four RGB,
both depth, all four CameraInfo topics, depth/RGB skew <= 0.50 s and map TF
success >= 95% with TF age <= 0.50 s. Fewer than three disjoint live Gazebo
episodes, any missing runtime input, or any metric miss is `BLOCKED`; offline
images and evaluator truth are never eligible substitutes. No fine-tuning is
performed by this gate.

## Claim boundary

PC ONNX loading and CPU execution are proven, not accuracy.  An unrelated bus
image produced a `litter_cube` score of 0.3388 at the current frozen model, so
threshold selection and a formal randomized scene dataset remain mandatory.
The official S100P COCO-80 HBM is compatibility evidence only; it is not the
four-class project model. The selected EdgeSAM board roles are the 512 encoder
and decoder HBM files. Project DOSOD conversion and the formal four-role real-board run remain a
separate blocked acceptance gate.

## S100P four-class HBM compile authorization

The board HBM path is governed by
`config/dosod_s100p_hbm_compile_contract.json`. Before `hb_compile`, run the
repository-level `validate_dosod_s100p_hbm_compile_contract.py` and then the
contract-bound `auto14_onnx_preflight.py`. The first stage is deliberately
read-only and returns `BLOCKED` unless the exact ONNX/vocabulary/reparameterized
inputs, frozen calibration manifest, official OE 3.7.0 evidence, and a live
compiler identity all agree. The second stage re-parses ONNX and requires the
exact static `images [1,3,640,640] -> scores [1,8400,4], boxes [1,8400,4]`
signature before it can emit YAML. Neither stage is evidence that an HBM was
compiled or accepted on the S100P.

The live identity file must be produced inside that same OE environment with
`collect_dosod_s100p_compiler_identity.py`; a historical wheel inventory or a
hand-written JSON file is not a compiler-runtime identity.
