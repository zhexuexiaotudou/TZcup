# AUTO-05R-1 G4 数据域重构代码

## 目的与范围

AUTO-05R-1 只实现 G4 数据域的**可复现数据生成/采集/QA 基础设施**：

- 带程序化纹理 PNG 与 SHA-256 的 G4 资产注册表与模型生成器；
- 12 个使用生产相机合同的 G4 世界与 `g4_world_manifest.json`；
- 冻结 25%–35% negative-only 先验的场景规划器 `g4_scene.py`；
- G4 数据集 finalize 与 QA 门 `g4_qa.py` 及 CLI；
- 复用 G3 真实采集语义的 capture 脚本（真实同步 RGB/depth/semantic/instance/CameraInfo/TF）。

本任务**不执行**完整 300 scene / 3000 frame 的 Gazebo 采集，**不训练模型**、
不导出 ONNX，也不把旧 G3 test 当作新模型选择集。`G4_dataset_gate_pass`
在完整采集与 QA 全绿之前保持 `false`。

## G4 规模与 split

正式冻结规模（`starter_ws/src/sanitation_learning/config/auto05r_g4_contract.yaml`）：

| 项目 | 值 |
|---|---|
| worlds | 12（train 8 / val 2 / test 2） |
| scenes | 300（每 world 25） |
| frames | 3000（每 scene 10） |
| target variants | 166（bottle/can/paper/leaf/puddle = 30/30/46/30/30） |
| hard negative families | 84（train 52 / val 16 / test 16） |

每个 world 的 `material_id`、`layout_family`、`geometry_family`、
`lighting_family` 两两不同，12 个 world SHA 全部不同；asset 只出现在各自
split 的世界中，杜绝 asset/world/trajectory 跨 split 泄漏。

## 负样本 / 资产 / taxonomy 合同

- 每个 split、每个 world 的 negative-only scene 比例固定在 25%–35%：
  train 7/25 = 28%、val 8/25 = 32%、test 7/25 = 28%，跨 split 差 ≤ 10pp。
- train negative-only frames ≥ 500（当前计划 560 帧）、
  paper-like hard-negative frames ≥ 300（当前计划 720 帧）。
- 任务要求的 paper hard-negative taxonomy（road_marking_fragment、light_paver、
  paver_joint、paper_like_road_patch、crack、shadow_edge、plastic_label、
  packaging_graphic、flat_stone、light_leaf_litter、reflective_area、
  vehicle_white_gray_structure、curb_corner、truncated_object_edge）全部进入
  train hard-negative 家族，并由 QA 校验在 train 帧中真实出现。
- leaf area 覆盖：厚度（高/低）、稀疏/密集、湿/干、三种叶形、阴影覆盖、
  部分遮挡、普通叶片背景区分；puddle area 覆盖：不同轮廓、浅/深反射、
  四种地面材质、阴影、湿地面非积水、镜面高光、局部遮挡、边界模糊。
  每个属性在 train 变体中的出现次数 ≥ 4。
- 每个 target variant / hard negative 都带 `id`、几何/材质/纹理家族、
  split_eligibility、source、license 与内容哈希 `sha256`；`g4_assets.py`
  加载时强制校验计数与哈希，篡改注册表立即失败。

## native vs offline augmentation 边界

每个 scene manifest 分开记录：

```yaml
native_gazebo_applied: true
offline_sensor_augmentation:
  requested_only: false
  applied: false
  plan: null
```

本任务只生成 native Gazebo 执行计划（世界光照/地面材质/资产纹理全部由
Gazebo 原生渲染），离线传感器增强（曝光、噪声、模糊、雨滴等）明确为
`requested_only: false`，不生成也不应用。QA 对任何
`offline_sensor_augmentation.applied=true` 或 `requested_only=true` 的 scene
直接判错。

## 采集命令与 resume 方式

正式采集（300 scene / 3000 frame）：

```bash
bash scripts/auto05r_g4_capture_all.sh
```

Windows Docker 入口（默认 25 scene/world，可 `-ScenesPerWorld` 调整）：

```powershell
.\scripts\run_auto05r_g4_capture_docker.ps1 -ScenesPerWorld 25
```

smoke 采集（每 world 1 scene）：

```powershell
.\scripts\run_auto05r_g4_capture_docker.ps1 -ScenesPerWorld 1
```

采集脚本与 G3 共用真实语义：`gz sim` + `parameter_bridge` 直连真实进程、
每 scene 10 帧、真实同步 RGB/depth/semantic/instance/CameraInfo/TF；
world/asset 使用 `gazebo_g4.py` 与 `g4_scene.py`，话题与 G3 一致
（生产 `/camera/color/image_raw`、`/camera/depth/image_rect_raw`、
`/camera/color/camera_info`，GT `/ground_truth/semantic/image`、
`/ground_truth/instance/image`）。已通过的 scene 通过
`capture_report.json` 的 `capture_pass` 做 resume-skip，每个 scene 最多
重试 3 次。

## QA 与 finalize

```bash
py -3 scripts/auto05r_g4_finalize_dataset.py \
  --data-root <data_root> --output-dir <qa_out> \
  --contract starter_ws/src/sanitation_learning/config/auto05r_g4_contract.yaml \
  [--strict]
```

QA 输出 frame manifest、instance records、split manifest、leakage report 与
`g4_dataset_qa.json`，校验 12 worlds / 300 scenes / 3000 frames 正式目标
（smoke 小样本输出 expected/actual）、负样本比例、taxonomy、标注完整性、
四传感器 sync、CameraInfo、TF、semantic-instance 一致性、跨 split 泄漏、
exact/pHash 重复与 distance/size 分桶；`test_used_for_model_selection=false`
被强制。`--strict` 要求全部冻结门通过，否则退出码 2；非 strict 模式对
数据质量违规同样失败，只对未完成规模输出 expected/actual。

## 当前状态

- 完整 300 scene / 3000 frame 采集**尚未执行**；现有验证为生成器/QA 的
  smoke 与 pytest 证据。
- `G4_dataset_gate_pass` 保持 `false`，`full_capture_executed=false`，
  正式模型、screening/formal/live/spot-clean 均未执行。
- 生成代码必须经过真实 Gazebo 采集验证后才能声明数据门通过。
