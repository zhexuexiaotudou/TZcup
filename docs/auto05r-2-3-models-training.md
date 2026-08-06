# AUTO-05R-2/3 模型与训练协议代码
## 目的与范围

AUTO-05R-2/3 只实现**模型架构、ground geometry、训练协议、平衡采样、
hard-negative mining 脚手架与 micro-overfit 合同**。本任务不执行完整 G4
训练、不运行 micro-overfit 正式流程、不导出最终正式 ONNX、不生成
checkpoint，也不伪造任何 screening 指标。

旧 Attempt 3（52.8 万参数 from-scratch direct detector / 浅层 RGB-D area
U-Net）不再作为新正式候选；本任务提供替换它的新模型族与受控训练协议。

## 模型架构（`g4_models.py`）

| 模型 | 输入 | 输出 |
|---|---|---|
| `DiscoveryDetector`（class-agnostic `litter_candidate`） | `[1, 3, 512, 384]` RGB | objectness heatmap `[1, 1, 128, 96]`、center offset `[1, 2, 128, 96]`、bbox regression `[1, 2, 128, 96]`（stride 4） |
| `CandidateCropClassifier` | `[1, 3, 192, 192]` crop | `[1, 4]` logits（background / plastic_bottle / metal_can / paper_litter） |
| `LeafSegmenter` | `[1, 4, 384, 512]` RGB-D | logits `[1, 1, 384, 512]` + boundary logits `[1, 1, 384, 512]` |
| `PuddleSegmenter` | `[1, 4, 384, 512]` RGB-D | logits `[1, 1, 384, 512]` + boundary logits `[1, 1, 384, 512]` |

- `DiscoveryDetector` 使用 stride 4/8 FPN 风格特征（自底向上 stride 2/4/8，
  顶层 top-down 融合回 stride 4），head 只放在 stride 4，输出中心热图 +
  offset + bbox size；解码复用 `auto04_contract.decode_centernet_outputs`/
  NMS 风格。
- `LeafSegmenter` / `PuddleSegmenter` 是两个独立二进制分割模型；支持共享
  encoder（`build_g4_models(shared_encoder=True)`），但每个模型**必须拥有
  独立 decoder 与独立 boundary head**（测试强制校验）。
- 全部使用标准 PyTorch Conv/BN/ReLU/Add/Resize 算子，不依赖 Ultralytics；
  `torch`/`torchvision` 只在函数内或模块 `__getattr__` 中加载，无 torch
  主机上 import 本模块不会失败。
- `build_g4_models()` / `model_summary()` 返回四类模型卡：参数数量、输入
  输出 names/shapes/dtypes、`state: not_trained`。

## Ground geometry（`ground_geometry.py`）

`GroundGeometryEstimator` 是训练与 ROS live 共用的**同一个 NumPy 实现**：

- 输入 CameraInfo（fx/fy/cx/cy + 分辨率）、depth，可选相机外参
  （`base_to_camera_xyz_m` 纯平移约定）；
- 输出 valid depth mask、确定性最小二乘（SVD）地面平面拟合、逐像素离地
  高度、相机/基座离地高度、局部表面法线代理、深度梯度代理；
- `fit_ground_plane` 对退化输入（有效点 < 3、共线/恒定点云、fx/fy 非法）
  抛 `ValueError`；
- **没有训练期 GT plane 旁路**：`estimate` 要么拟合与 live 完全相同的
  depth 输入，要么复用之前**估计出的**平面（`ground_plane.provided=true`），
  绝不允许把 GT 注释当作平面来源。

## 训练协议（`g4_training.py` + `auto05r_training_protocol.yaml`）

### 平衡采样（`BalancedBatchSampler`）

按 `batch_proportions` 冻结比例从 positive / negative-only /
paper-like hard negative / plastic_bottle / metal_can / paper_litter /
leaf_pile / puddle 桶采样。每个桶**全量轮换后再洗牌**，不会像
`WeightedRandomSampler` 那样反复重复少量负样本（测试对仅 4 条 negative
样本的小数据集断言出现次数均衡）。同一 batch 内不允许重复索引。

### Trainer

- 每 epoch 训练 + 验证，记录完整 curve（`training_loss` + 验证指标）；
- best checkpoint 只在 train/val 上选择，`fit` 只接受 train/val loader，
  暴露 `split == "test"` 的数据集会直接 `ValueError`；
- EMA（验证时切换到 EMA 权重并恢复）、early stopping、AMP 开关、
  确定性 seed（`torch.use_deterministic_algorithms` + cudnn deterministic）；
- optimizer/scheduler 由协议冻结（AdamW + CosineAnnealingLR）。

### Hard-negative mining（`HardNegativeMining`）

- 最多 3 轮；只从 train/val background 收集 top false positives；
- 任何 `split == "test"` 的帧或缺少 split 声明的帧直接 `ValueError`，
  G4 final test 在代码层面不可读；
- 输出每轮 candidate/mined 计数、top scores 与 mined frame indices。

### Micro-overfit 门槛（`MicroOverfitGate`）

按 `micro_overfit.gates` 冻结阈值计算：discovery recall、negative FP、
classifier macro F1、paper precision、leaf/puddle IoU。任一指标缺失或
未达门槛即返回 `pass=false` / `micro_overfit_pass=false`。

## 协议冻结项（`auto05r_training_protocol.yaml`）

- 每模型 seed（discovery/classifier/leaf/puddle = 20260805/06/07/08）；
- micro-overfit 样本量（discovery 32 帧、classifier 64 crop、leaf/puddle
  各 16 帧）与六项门槛；
- batch 比例（合计 1.0）；optimizer（AdamW）/ scheduler（CosineAnnealingLR）；
- EMA decay `0.999`、early stopping patience `8`、epochs `40`；
- 模型选择只允许 train/validation + D1-D5；
- `test_split_readable_during_training=false`、
  `hard_negative_mining_from_test=false`；
- ONNX：opset 17、`dynamic_axes=none`、无自定义 op。

## ONNX 合同（`g4_models.py`）

- `export_fixed_onnx(model, dummy_input, output_path, opset=17)`：导出固定
  shape（`dynamic_axes=None`）、单一 `outputs` 张量（dict 输出按
  `output_names` 顺序拼接），导出后校验 `onnx.checker` 与动态维度；
- `operator_inventory(onnx_path)`：返回算子类型计数；
- `torch_onnx_parity(model, onnx_session, inputs)`：返回最大数值误差、
  argmax 一致率与解码一致率（detector 解码对比 / 分类与分割 argmax）；
- 无 torch/onnx 的主机自动 skip 对应测试。

## CLI（`scripts/auto05r_micro_overfit.py`）

```powershell
py -3 scripts/auto05r_micro_overfit.py `
  --model-type discovery|classifier|leaf|puddle `
  --data-root <g4_dataset_root> --output-dir <out> [--config <yaml>]
```

只验证 CLI 与代码路径：校验冻结协议、构建模型卡、写
`micro_overfit_report.json`，报告恒为 `executed=false`、
`micro_overfit_pass=false`、`status=not_trained`。

## 当前状态

- `AUTO-05R-2/3` 当前为 **`not_trained`**；
- `micro_overfit_pass=false`，未执行 G4 正式训练、micro-overfit 正式运行、
  screening/formal/live/spot-clean；
- 尚未执行项：G4 300 scene / 3000 frame 正式采集与 QA、四模型正式训练、
  micro-overfit 门禁运行、screening 与模型选择、formal/live/spot-clean
  验收；这些只能由后续真实运行产生证据。
