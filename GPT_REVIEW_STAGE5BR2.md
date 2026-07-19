# TZcup Stage5BR2 独立复核入口

## 结论

本轮完成了 G2 车载相机数据基础恢复，但没有越过 G2 screening 数据门。四个世界已由 Gazebo Harmonic 实际启动，RGB、深度、semantic GT 和 instance GT 话题烟测通过；相机契约来自当前车辆 Xacro/launch，资产使用真实物理尺寸。尚未采集并质检 80 scene/800 frame，因此没有执行分辨率实测、detector/area segmenter 训练筛选、500 scene/5000 frame 扩量、live、真实 Nav2 或 J6 门。

```text
first_blocking_layer = G2_screening_dataset_80_scene_800_frame_not_executed
READY_FOR_GPT_REVIEW_STAGE5B = false
READY_FOR_STAGE5C = false
```

## 请优先复核

1. `artifacts/stage5br2_20260720_review/stage5br2_status.json` 的 fail-closed 边界是否准确。
2. `g2_worlds/g2_world_manifest.json` 是否正确继承生产相机外参、FOV、频率、原生分辨率与 ROS 话题。
3. 四个 world SHA、材料和 2/1/1 world-isolated split 是否满足 G2 screening 的基础条件。
4. `g2_metrics.py` 是否真正按 instance-id 计算 bbox、最短边、mask area、距离和遮挡，并把零像素物体记为 `not_visible`。
5. GT 传感器是否保持 training-only，且生产 launch 未被加入任何 GT 输入。
6. 下一轮是否应先补齐“车辆运动随机化 + 80/800 采集/QA + 四分辨率实测”，而不是提前训练或扩大到 500/5000。

## 指标语义勘误

Stage5BR 历史字段 `cross_asset_world` 实际是在同一个 G1 世界内隔离资产，应解释为 `cross_asset_same_world`。由于 G1 只有一个 world，真正的 `cross_world` 必须为 `null`；`cross_material` 也没有独立证据。为保持历史证据哈希，旧 JSON 不原地改写，本轮状态文件提供规范化语义。

## 证据索引

- `artifacts/stage5br2_20260720_review/g2_worlds/g2_world_manifest.json`
- `artifacts/stage5br2_20260720_review/g2_world_smoke/g2_world_smoke.json`
- `artifacts/stage5br2_20260720_review/stage5br2_status.json`
- `starter_ws/src/sanitation_learning/config/stage5br2_g2_screening.yaml`
- `docs/stage5br2-g2-vehicle-camera.md`

## 明确未完成

- G2 80 scene/800 frame 数据采集及逐实例 QA；
- 256×192、384×288、512×384、640×384 的完整实测权衡；
- 独立离散目标 detector 与叶片/积水 area segmenter；
- confidence-ranked AP50/AP50:95、small-object recall、same-color-negative FP；
- 500 scene/5000 frame、live、真实 Nav2、J6 实机链。
