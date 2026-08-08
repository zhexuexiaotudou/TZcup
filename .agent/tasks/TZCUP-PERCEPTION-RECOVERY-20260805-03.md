# Task ID

`TZCUP-PERCEPTION-RECOVERY-20260805-03`

<!-- HYBRID_TASK_METADATA_BEGIN
{
  "task_id": "TZCUP-PERCEPTION-RECOVERY-20260805-03",
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
        "starter_ws/src/sanitation_learning/test/test_g4_models.py",
        "starter_ws/src/sanitation_learning/test/test_ground_geometry.py",
        "starter_ws/src/sanitation_learning/test/test_g4_training_protocol.py"],
      "working_directory": ".",
      "timeout_seconds": 300,
      "permission_pattern": "py -3 -m pytest targeted AUTO-05R-2/3 tests"
    }
  ]
}
HYBRID_TASK_METADATA_END -->

# Objective

实现 `AUTO-05R-2/3` 的模型架构、ground geometry、训练协议、平衡采样、hard-negative mining 脚手架与 micro-overfit 合同。本任务只实现代码、配置和测试，不执行完整 G4 训练，不导出最终正式模型，不伪造 screening 指标。

# Relevant context

当前仓库已有：

- G4 资产、world、scene、QA 和真实 Gazebo 采集脚本（`gazebo_g4.py`、`g4_scene.py`、`g4_qa.py`、`scripts/auto05r_g4_capture_all.sh`）。
- AUTO-05R-0 的尺度合同、因子化诊断和 manifest v2。
- 旧 Attempt 3 是 52.8 万参数 from-scratch direct detector / 浅层 RGB-D area U-Net，已确认不能继续作为新正式候选。
- G4 正式采集目标为 300 scene / 3000 frame，train/val/test world split 8/2/2，负样本比例 25%-35%。

必须先阅读：

```text
starter_ws/src/sanitation_learning/sanitation_learning/g4_scene.py
starter_ws/src/sanitation_learning/sanitation_learning/g4_qa.py
starter_ws/src/sanitation_learning/config/auto05r_g4_contract.yaml
starter_ws/src/sanitation_learning/sanitation_learning/auto04_contract.py
starter_ws/src/sanitation_learning/sanitation_learning/models.py
starter_ws/src/sanitation_learning/sanitation_learning/evaluation.py
scripts/auto05_screening.py
```

# Requirements

## 1. Model architecture

新增：

```text
starter_ws/src/sanitation_learning/sanitation_learning/g4_models.py
```

至少实现：

- `DiscoveryDetector`：class-agnostic `litter_candidate` 检测器，输出 objectness heatmap、center offset、bbox regression；使用 stride 4/8 多尺度 FPN 风格特征；支持固定输入 `[1, 3, 512, 384]`。
- `CandidateCropClassifier`：crop classifier，输出 `background / plastic_bottle / metal_can / paper_litter` 四类；输入 `[1, 3, 192, 192]`。
- `LeafSegmenter` / `PuddleSegmenter`：两个独立二进制分割模型，共享预训练 encoder 时也必须有独立 decoder 和边界 head；输入 `[1, 4, 384, 512]`，输出 logits 和 boundary logits。
- `build_g4_models()` / `model_summary()`：返回模型卡，包含参数数量、输入输出 names/shapes/dtypes、状态 `not_trained`。
- 不依赖 Ultralytics；使用标准 PyTorch Conv/BN/ReLU/Add/Resize 算子。
- `torch` 和 `torchvision` 必须在函数内或 `importorskip` 下使用，保证本机无 torch 时 CI 不崩。

## 2. Ground geometry

新增：

```text
starter_ws/src/sanitation_learning/sanitation_learning/ground_geometry.py
```

至少实现：

- `GroundGeometryEstimator`，使用 CameraInfo、depth、相机外参计算：
  - valid depth mask
  - ground plane fit（RANSAC 或确定性最小二乘）
  - height above ground
  - local surface normal / depth gradient proxy
- 输入输出均为 NumPy；`fit_ground_plane` 对退化输入抛 `ValueError`。
- 同一实现必须可被训练和 ROS live 复用；不得在训练中使用 GT plane 而 live 使用估计 plane 的旁路。

## 3. Training protocol

新增：

```text
starter_ws/src/sanitation_learning/sanitation_learning/g4_training.py
starter_ws/src/sanitation_learning/config/auto05r_training_protocol.yaml
scripts/auto05r_micro_overfit.py
```

`g4_training.py` 至少实现：

- `BalancedBatchSampler`：按 positive/negative-only/paper-like hard negative/三类离散/leaf/puddle 预设比例采样；禁止只用 WeightedRandomSampler 重复少量负样本。
- `Trainer`：每 epoch validation、best checkpoint、EMA、early stopping、AMP 开关、deterministic seed、完整 curve。
- `HardNegativeMining`：最多 3 轮；只能从 train/val background 收集 top false positives；禁止读取 G4 final test。
- `MicroOverfitGate`：按提示词门槛计算 discovery recall、negative FP、classifier macro F1、paper precision、leaf/puddle IoU；未过门则返回 fail。

`auto05r_training_protocol.yaml` 必须冻结：

- 每模型 seed；
- micro-overfit 样本量和门槛；
- batch 比例；
- optimizer/scheduler；
- EMA decay、early stopping patience；
- 模型选择只允许 train/validation/D1-D5；
- `test_split_readable_during_training=false`；
- `hard_negative_mining_from_test=false`。

`scripts/auto05r_micro_overfit.py` 提供 CLI：

```text
--model-type discovery|classifier|leaf|puddle
--data-root
--output-dir
--config
```

本任务只验证 CLI 与代码路径，不要求真实训练通过。

## 4. ONNX contract

在 `g4_models.py` 或 `g4_training.py` 中实现：

- `export_fixed_onnx(model, dummy_input, output_path, opset=17)`；
- `operator_inventory(onnx_path)`；
- `torch_onnx_parity(model, onnx_session, inputs)`，返回 max numeric error 与 argmax/decoded agreement。
- 导出必须 `dynamic_axes=None`，不允许自定义 op。

## 5. Tests

新增测试：

```text
starter_ws/src/sanitation_learning/test/test_g4_models.py
starter_ws/src/sanitation_learning/test/test_ground_geometry.py
starter_ws/src/sanitation_learning/test/test_g4_training_protocol.py
```

覆盖：

- model output shapes（torch 可用时；不可用时 skip）；
- ground plane valid/invalid input；
- balanced sampler 比例；
- best checkpoint selection 不读 test；
- hard-negative mining 排除 test；
- ONNX export/parity（torch+onnx 可用时；不可用时 skip）。

把新测试加入 `scripts/ci_fast.py`。

## 6. Docs

新增 `docs/auto05r-2-3-models-training.md`，说明：

- 新模型架构；
- ground geometry 训练/live 同实现；
- 训练协议和门槛；
- 当前尚未真实训练，`micro_overfit_pass=false`，`AUTO-05R-2/3` 未完成。

在 `docs/progress.md` 顶部增加 `## 2026-08-06：AUTO-05R-2/3 模型与训练协议代码` 小节，并在 README 当前状态中注明模型代码已就绪、真实训练未执行。

# Explicit non-goals

- 不执行真实 G4 训练或 micro-overfit 正式运行。
- 不导出正式 ONNX、不生成 checkpoint。
- 不改历史 evidence、`AUTONOMOUS_STATE.json`、`FINAL_AUTONOMOUS_STATUS.json`。
- 不把旧 G3 test 当新选择集。
- 不修改 hybrid workflow 或 `.agent` 之外的 workflow 配置。

# Existing patterns to follow

- 复用 `auto04_contract.py` 的检测解码/NMS 风格。
- 复用 `g4_qa.py` 的 schema 和 scale 语义。
- 新模块放在 `sanitation_learning/sanitation_learning/`，与旧模块并存。
- Python 使用 `from __future__ import annotations`。

# Validation commands

见 metadata；执行：

```powershell
git diff --check
py -3 scripts/ci_fast.py
py -3 -m pytest -q `
  starter_ws/src/sanitation_learning/test/test_g4_models.py `
  starter_ws/src/sanitation_learning/test/test_ground_geometry.py `
  starter_ws/src/sanitation_learning/test/test_g4_training_protocol.py
```

# Acceptance criteria

- 新模块、配置、CLI、测试和文档均存在；
- `build_g4_models()` 可生成四类模型卡，输出 shape 正确；
- `GroundGeometryEstimator` 对有效深度输出非空估计，对退化输入抛错；
- `BalancedBatchSampler`、`Trainer`、`HardNegativeMining`、`MicroOverfitGate` 都有可测试实现；
- `ci_fast.py` 全绿且包含新测试；
- `AUTO-05R-2/3` 当前状态明确为 `not_trained`，不伪造任何模型通过。

# Required completion report

报告必须列出：

- changed/new files；
- 架构选择；
- 训练协议设计；
- ONNX 合同；
- 验证命令结果；
- 尚未执行项：G4 正式训练、micro-overfit、screening/formal/live/spot-clean。

# Stop conditions

只有任务超出用户授权、无法安全完成、需要另一个 AI、或必须修改 hybrid workflow 本身时才停止。
