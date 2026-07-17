# GPT 独立复核入口：Stage5A

## 复核结论入口

以 `artifacts/stage5a_20260717_review/stage5a_summary.json` 为机器可读总入口，以同目录 `MANIFEST.json` 校验逐字节完整性。不要仅依据本文中的布尔值，应复算 summary 的每个 gate，并抽查源报告。

当前生成结论为 `READY_FOR_GPT_REVIEW_STAGE5A=true`、`READY_FOR_STAGE5B=true`；它只表示本文列出的 synthetic-domain 内部门通过，不改变真实数据、J6、实车和竞赛效率的 false 状态。

## 建议复核顺序

1. 校验 `MANIFEST.json` 的 bytes 与 SHA-256；
2. 审查 `contracts/garbage_registry.yaml` 是否使用精确 identity、稳定 UUID 和显式负样本；
3. 审查 dataset manifest、scene-seed split、重复图像计数与 COCO detection/segmentation；
4. 审查 ONNX 模型 SHA、固定输入、类别顺序、预/后处理和算子清单；
5. 复算 held-out 离散类、区域类和 map 定位门；
6. 复算 30-seed spot-clean 状态闭环，确认 wrong-target、collision、keepout、brush final、GT control 和 Coverage resume；
7. 检查 live smoke 是否包含真实 RGB/depth/camera_info、非空 2D/3D/map 输出、ONNX backend 与 GT 隔离；
8. 检查 `rosbag_info.txt` 是否包含 RGB、depth、推理、定位、TF 与 GT；
9. 检查 Stage4W 单 seed 回归的完整任务、经验覆盖率、定位 RMSE、碰撞、keepout、刷盘最终状态与回放；
10. 确认所有真实数据、J6、实板、实车和竞赛效率声明保持 false。

## 内部门槛

- discrete macro precision/recall/F1 均不低于 0.95，每类 recall 不低于 0.90；
- leaf/puddle 区域 macro F1 不低于 0.95，mIoU 均不低于 0.80；
- map 定位 RMSE 不高于 0.10 m；
- spot-clean 有效 seed 不少于 30，任务完成和 Coverage 恢复均不低于 90%；
- wrong-target、collision、keepout、GT control 均为 0，brush final 为 false；
- Stage4W 最小回归通过；
- 正式 rosbag 必须包含 RGB、depth 和 perception 消息。

## 必须保持的保守表述

当前模型是确定性 synthetic color prototype，30-seed 闭环是软件级 synthetic task-state E2E。复核通过只能说明 Stage5A 内部接口、合成数据和仿真闭环可以进入下一轮研究，不能写成真实垃圾识别精度、J6 板端性能、机械臂抓取能力或赛事达标。

固定边界：

```text
competition_perception_pass=false
j6_toolchain_available=false
j6_quantization_pass=false
j6_runtime_pass=false
competition_efficiency_pass=false
theoretical_efficiency_m2_h=1053
target_efficiency_m2_h=3500
```

完成复核后再决定是否进入 Stage5B；本轮不自动启动 J6 量化、机械臂或大模型任务分解。
