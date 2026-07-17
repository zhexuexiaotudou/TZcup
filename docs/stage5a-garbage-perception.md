# Stage5A 垃圾感知、数据集与定点清扫

## 交付范围

Stage5A 在 Stage4W 的定位、完整 Coverage 与安全基线上增加五类垃圾的语义契约、仿真真值、合成 RGB-D 数据集、ONNX Runtime 感知、多帧目标跟踪和定点清扫状态机。五类目标为 `plastic_bottle`、`metal_can`、`paper_litter`、`leaf_pile` 和 `puddle`；固定垃圾桶、纸箱、动态行人盒、墙、灯、树和车辆结构均为显式负样本，不按模型名子串推断类别。

新增 ROS 2 包：

- `sanitation_perception_interfaces`：`GarbageTarget`、`GarbageTargetArray`、`CleaningEvent`；
- `sanitation_perception`：registry、后端抽象、RGB-D 投影、ONNX Runtime 实时节点和多帧 tracker；
- `sanitation_ground_truth`：仅用于标注/评估的 registry 真值和几何遮挡 fallback；
- `sanitation_dataset`：确定性场景生成、COCO detection/segmentation、scene-seed split、ONNX 模型和评测；
- `sanitation_spot_cleaning`：`deferred` 默认策略、预检、状态转换、事件和 30-seed 合成闭环评测。

## 后端与控制边界

统一后端包含 `ground_truth`、`mock`、`onnxruntime` 和 `horizon_j6`。正式 x86 仿真只允许 `onnxruntime`；`ground_truth` 进入 tracker 会直接报错，实时诊断与 30-seed 报告均审计 `ground_truth_control_violation_count=0`。J6 工具链不存在时后端 fail-closed，不允许回退到 GT 或 mock。

实时节点使用实际 RGB、depth、camera_info 和 TF：ONNX 输出分割后提取连通域，在 mask 内做鲁棒深度估计，反投影并转换到 `map`，发布非空 `vision_msgs/Detection2DArray`、`Detection3DArray`、分割图和多帧 `GarbageTargetArray`。缺 camera_info、depth 或 TF 时 map 目标 fail-closed，并在 diagnostics 中记录原因。

真值节点按精确 registry identity 输出稳定 UUID、map pose、尺寸、策略、可见度与遮挡率。几何 fallback 将目标和显式负样本建模为角区间；被近物体完全覆盖的目标不发布。该 fallback 是可审计的仿真标注路径，不是赛事真实感知。

## 数据与模型契约

20-scene smoke dataset 按 scene seed 固定分为 train 12、val 4、test 4，保存 RGB、depth、camera、TF、对象标注、scene manifest、COCO detection/segmentation、split hash 和重复图像检查。正式大数据与原始 MCAP 保留在本机，不进入 Git；复核目录只保存 manifest、COCO 标注、split、模型和紧凑报告。

当前 ONNX 是 batch=1、`1x3x96x128`、opset 13 的确定性色彩原型模型，没有学习权重，只验证端到端接口、评测和部署契约。类别顺序、预处理、后处理、算子清单、权重来源和许可证均版本化。它只能产生 `synthetic_perception_pass`，不能产生赛事真实精度结论。

## 复现

快速门：

```powershell
py scripts/ci_fast.py
```

完整 Stage5A 门（包含构建、ROS 测试、20-scene 数据、held-out ONNX、30-seed 状态闭环、Gazebo RGB-D 实时链路和压缩 rosbag）：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/run_stage5a_docker.ps1 `
  -OutputName stage5a_formal3 -RecordBag
```

Stage4W 单 seed 完整回归：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/run_stage4w_static_coverage_docker.ps1 `
  -Seed 0 -OutputName stage5a_stage4w_regression_seed0
```

## 结论边界

- Stage5A 指标仅适用于确定性 synthetic color domain；
- 30-seed 定点清扫是 ONNX + tracker + coordinator 的合成任务状态闭环，不等同于 30 次真实车辆或完整 Gazebo Nav2 定点任务；
- 几何遮挡 fallback 已测试完全遮挡抑制，但尚未替代真实数据标注审计；
- `competition_perception_pass=false`；
- `j6_toolchain_available=false`、`j6_quantization_pass=false`、`j6_runtime_pass=false`；
- `competition_efficiency_pass=false`，理论效率仍为 `1053 m²/h < 3500 m²/h`；
- 未执行原生 Ubuntu/WSLg GUI、J6 实板、实车或机械臂验收。
