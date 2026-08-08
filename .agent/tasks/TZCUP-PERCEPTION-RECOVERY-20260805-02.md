# Task ID

`TZCUP-PERCEPTION-RECOVERY-20260805-02`

<!-- HYBRID_TASK_METADATA_BEGIN
{
  "task_id": "TZCUP-PERCEPTION-RECOVERY-20260805-02",
  "validation_commands": [
    {
      "executable": "git",
      "arguments": ["diff", "--check"],
      "working_directory": ".",
      "timeout_seconds": 180,
      "permission_pattern": "git diff --check"
    },
    {
      "executable": "py",
      "arguments": ["-3", "scripts/ci_fast.py"],
      "working_directory": ".",
      "timeout_seconds": 600,
      "permission_pattern": "py -3 scripts/ci_fast.py"
    },
    {
      "executable": "py",
      "arguments": ["-3", "-m", "pytest", "-q",
        "starter_ws/src/sanitation_learning/test/test_g4_assets.py",
        "starter_ws/src/sanitation_learning/test/test_g4_scene_negative_prior.py",
        "starter_ws/src/sanitation_learning/test/test_g4_qa.py"],
      "working_directory": ".",
      "timeout_seconds": 300,
      "permission_pattern": "py -3 -m pytest targeted G4 tests"
    }
  ]
}
HYBRID_TASK_METADATA_END -->

# Objective

实现 `AUTO-05R-1` 的 G4 数据域重构代码、配置、QA 合同与测试。本任务只实现可复现的数据生成/采集/QA 基础设施，不执行完整 300 scene 真实采集，不训练模型，不伪造任何 G4 指标。

# Relevant context

当前仓库已有：

- G3 原生数据链：`gazebo_g3.py`、`g3_scene.py`、`scripts/auto05_capture_all.sh`、`scripts/run_auto05_g3_capture_docker.ps1`。
- G2 原生采集器：`g2_capture.py`、`gazebo_g2.py`。
- 生产相机合同：`starter_ws/src/sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro`，分辨率 640x480，生产话题 `/camera/color/image_raw`、`/camera/depth/image_rect_raw`、`/camera/color/camera_info`。
- AUTO-05R-0 已建立 `metric_scale.py`、`factorized_diagnostics.py`、manifest v2 和 backend fail-closed 合同。
- 历史 `AUTO-05=BLOCKED`，旧 G3 test 只能作为 legacy benchmark，不得作为新模型选择集。

必须先阅读：

```text
starter_ws/src/sanitation_learning/sanitation_learning/assets.py
starter_ws/src/sanitation_learning/sanitation_learning/gazebo_g3.py
starter_ws/src/sanitation_learning/sanitation_learning/g3_scene.py
starter_ws/src/sanitation_learning/sanitation_learning/g2_capture.py
starter_ws/src/sanitation_learning/sanitation_learning/gazebo_g2.py
scripts/auto05_capture_all.sh
scripts/run_auto05_g3_capture_docker.ps1
scripts/auto05_finalize_dataset.py
starter_ws/src/sanitation_learning/config/asset_registry.yaml
starter_ws/src/sanitation_learning/config/auto05r_factorized_diagnostics.yaml
```

# Current architecture

现有 G3 资产注册表只有 6 个变体/类、12 个负样本；世界固定为 8 个；场景生成器通过 `g3_scene.py` 固定每 world 15 scene、train/val/test 负样本比例不满足 G4 要求。

`assets.py` 当前生成简单纯色 primitive，`write_gazebo_assets` 只写模型 SDF，没有为 G4 的材质/纹理家族生成真实可见纹理差异。G4 必须解决这一点，否则不能声称 asset/texture 变体。

# Requirements

## 1. G4 asset registry

新增：

```text
scripts/generate_g4_asset_registry.py
starter_ws/src/sanitation_learning/config/g4_asset_registry.yaml
starter_ws/src/sanitation_learning/sanitation_learning/g4_assets.py
```

`g4_asset_registry.yaml` 必须满足：

- `schema_version: 2`
- `plastic_bottle` 总变体 >= 30，其中 train >= 20、val >= 5、test >= 5。
- `metal_can` 总变体 >= 30，其中 train >= 20、val >= 5、test >= 5。
- `paper_litter` 总变体 >= 46，其中 train >= 30、val >= 8、test >= 8。
- `leaf_pile` 总变体 >= 30，其中 train >= 20、val >= 5、test >= 5。
- `puddle` 总变体 >= 30，其中 train >= 20、val >= 5、test >= 5。
- hard negatives 总家族 >= 80，其中 train 家族 >= 50、val >= 15、test >= 15。
- 每个 target variant 至少含 `id`、`geometry_family`、`material_family`、`texture_family`、`palette`、`split_eligibility`、`source`、`license`、`sha256`。
- 每个 hard negative 至少含 `id`、`taxonomy`、`geometry_family`、`material_family`、`texture_family`、`split_eligibility`、`source`、`license`、`sha256`。

paper hard-negative taxonomy 必须真实包含：

```text
road_marking_fragment
light_paver
paver_joint
paper_like_road_patch
crack
shadow_edge
plastic_label
packaging_graphic
flat_stone
light_leaf_litter
reflective_area
vehicle_white_gray_structure
curb_corner
truncated_object_edge
```

area 数据覆盖至少：

- leaf：不同厚度、稀疏/密集、湿/干、不同叶形、阴影覆盖、部分遮挡、普通叶片背景区分。
- puddle：不同轮廓、浅/深反射、不同地面材质、阴影、湿地面非积水、镜面高光、局部遮挡、边界模糊。

`g4_assets.py` 提供：

- `load_g4_asset_registry(path)`：校验计数、split eligibility、metadata 完整性。
- `write_g4_assets(registry_path, output_dir)`：为每个变体生成真实可见的模型目录，至少包含 `model.sdf` 和程序化纹理 PNG；纹理由 NumPy/OpenCV 确定性生成，记录 `sha256`。
- `g4_registry_summary(registry)`：输出各 split 的变体数、家族数、SHA 清单。

`generate_g4_asset_registry.py` 必须能确定性生成上述 YAML，并保证生成结果与已提交 YAML 一致（可通过 `git diff` 或内容比较验证）。

## 2. G4 worlds

新增：

```text
starter_ws/src/sanitation_learning/sanitation_learning/gazebo_g4.py
```

要求：

- 至少 12 个不同 world，train/val/test = 8/2/2。
- 每个 world 的 `material_id`、`layout_family`、`geometry_family`、`lighting_family` 不同组合，world SHA 全部不同。
- 使用生产车辆相机合同与 G3 相同 topics。
- 生成 `g4_world_manifest.json`，记录 world、asset、negative、split、trajectory、sensor topics、SHA。
- 使用 `g4_assets.py` 的模型目录和纹理，不再只靠纯色 primitive。

新增 `scripts/auto05r_g4_capture_all.sh` 与 `scripts/run_auto05r_g4_capture_docker.ps1`：

- 可复用现有 G3 capture 流程，但使用 `gazebo_g4.py`、`g4_scene.py`、`g4_world_manifest.json`。
- 支持 resume-skip 已通过 scene。
- 每个 scene 至少 10 帧，真实同步 RGB/depth/semantic/instance/CameraInfo/TF。
- 默认总规模可配置；正式参数为 300 scene / 3000 frame，但本任务只验证脚本能启动和 smoke 流程，不执行完整采集。

## 3. G4 scene contract and negative prior

新增：

```text
starter_ws/src/sanitation_learning/sanitation_learning/g4_scene.py
starter_ws/src/sanitation_learning/config/auto05r_g4_contract.yaml
```

`g4_scene.py` 至少实现：

- `randomize(manifest_path, world_id, scene_seed, scene_index, output)`。
- 每个 split、每个 world 的 negative-only scene 比例固定在 `25%–35%`。
- train/val/test 的 negative-only 比例彼此差 <= 10 个百分点。
- train 集 negative-only frames >= 500、paper-like hard-negative frames >= 300。
- 每个 scene 保存 `native_gazebo_applied` 与 `offline_sensor_augmentation` 分开；本任务只生成 native plan，offline augmentation 明确为 `requested_only: false`。
- 所有 distance/size/occlusion/visible_fraction 分桶元数据写入 scene manifest。

`auto05r_g4_contract.yaml` 必须包含 G4 全部冻结规模、资产数量、负样本比例、QA 门和 `test_used_for_model_selection=false`。

## 4. G4 QA and finalize

新增：

```text
starter_ws/src/sanitation_learning/sanitation_learning/g4_qa.py
scripts/auto05r_g4_finalize_dataset.py
```

`g4_qa.py` 必须实现：

- `finalize_g4_dataset(data_root, output_dir)`：生成 frame manifest、instance records、split manifest、QA report。
- 校验 12 worlds、300 scenes、3000 frames 的正式目标（对 smoke 小样本也输出 `expected`/`actual`）。
- 校验负样本比例、asset 数量、taxonomy、annotation completeness、四传感器 sync、CameraInfo、TF、semantic-instance error、world/asset/trajectory leakage、exact/pHash duplicate、distance/size bucket。
- `test_used_for_model_selection=false` 强制。

`auto05r_g4_finalize_dataset.py` 提供 CLI：

```text
--data-root
--output-dir
--contract
--strict
```

## 5. Tests and CI

新增测试：

```text
starter_ws/src/sanitation_learning/test/test_g4_assets.py
starter_ws/src/sanitation_learning/test/test_g4_scene_negative_prior.py
starter_ws/src/sanitation_learning/test/test_g4_qa.py
```

覆盖：

- registry 计数与 metadata 校验；
- `write_g4_assets` 输出模型 SDF、纹理 PNG 和 SHA；
- 负样本比例 25%-35% 与跨 split 差 <= 10pp；
- 12 world / 8/2/2 split contract；
- QA report schema 与 leakage/sync gate；
- 旧 G3 test 不被用作新选择集。

把新测试加入 `scripts/ci_fast.py`。

## 6. Docs

新增 `docs/auto05r-1-g4-data.md`，说明：

- G4 规模与 split；
- 负样本/资产/taxonomy 合同；
- native vs offline augmentation 边界；
- 采集命令与 resume 方式；
- 当前尚未执行完整采集，`G4_dataset_gate_pass` 仍为 false。

在 `docs/progress.md` 顶部增加 `## 2026-08-05：AUTO-05R-1 G4 数据域重构代码` 小节，并在根目录中文 `README.md` 的“学习感知”状态中补充 G4 生成器已就绪、完整采集未执行。README 仍不得新增 `## AUTO-` 或 `## Stage` 进度标题，总行数保持 <= 180。

# Explicit non-goals

- 不执行完整 300 scene / 3000 frame Gazebo 采集；本任务只提供可复现脚本与 smoke 验证。
- 不训练模型、不导出 ONNX、不运行 screening/formal/live。
- 不改历史 evidence、`AUTONOMOUS_STATE.json`、`FINAL_AUTONOMOUS_STATUS.json`、`FINAL_BLOCKER_REGISTER.json`。
- 不把旧 G3 test 当新选择集；不伪造 `G4_dataset_gate_pass=true`。
- 不修改 hybrid workflow 或 `.agent` 之外的 workflow 配置。

# Existing patterns to follow

- 复用 `g2_capture.py`/`auto05_capture_all.sh` 的真实采集语义，不写 mock 采集。
- 复用 `g3_scene.py` 的确定性随机种子、scene manifest 和动态负样本执行方式。
- 新 G4 模块放在 `sanitation_learning/sanitation_learning/`，与旧模块并存，不破坏 G2/G3 回归。
- YAML/JSON 使用仓库现有风格；Python 使用 `from __future__ import annotations`。
- 所有生成物写入 Git 忽略的仓库外数据根，仓库只收代码、配置、紧凑证据和测试。

# Validation commands

见 metadata；执行：

```powershell
git diff --check
py -3 scripts/ci_fast.py
py -3 -m pytest -q `
  starter_ws/src/sanitation_learning/test/test_g4_assets.py `
  starter_ws/src/sanitation_learning/test/test_g4_scene_negative_prior.py `
  starter_ws/src/sanitation_learning/test/test_g4_qa.py
```

# Acceptance criteria

- `g4_asset_registry.yaml` 与 `generate_g4_asset_registry.py` 可重复一致，计数满足要求。
- `write_g4_assets` 可生成带纹理 PNG 和 SHA 的模型目录，测试通过。
- `gazebo_g4.py` 可生成 12 world manifest，train/val/test=8/2/2，world SHA 全部不同。
- `g4_scene.py` 可生成满足 25%-35% negative-only 比例和 paper hard-negative 要求的 scene manifest。
- `g4_qa.py` 与 finalize CLI 能对 smoke 数据输出结构化 QA，并对缺失/泄漏/比例错误 fail。
- 新测试全部通过，`ci_fast.py` 包含新测试并全绿。
- README 与 docs 已同步，且没有伪造任何 G4 采集完成状态。

# Required completion report

报告必须列出：

- changed/new files；
- G4 asset/world/scene/QA 设计；
- 各合同计数；
- 验证命令结果；
- 尚未执行项：真实 300 scene 采集、G4 dataset gate、模型训练/screening/formal/live/spot-clean；
- 残余风险：生成代码必须经过真实 Gazebo 采集验证后才能声明数据门通过。

# Stop conditions

只有任务超出用户授权、无法安全完成、需要另一个 AI、或必须修改 hybrid workflow 本身时才停止。
