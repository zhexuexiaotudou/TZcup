# Stage5BR：Gazebo-camera 数据恢复与模型筛查边界

## 结论

Stage5BR 已把 Stage5B 的数据真实性阻断项从“程序化 P1 renderer”推进为可复现的真实 Gazebo Harmonic camera 数据链。训练链 micro-overfit 与 PyTorch/ONNX/ROS 预处理一致性均通过；G1 smoke 采集了 50 个独立 scene、500 帧共视场 RGB-D、semantic 与 instance 数据，标注、同步和 split QA 通过。但三次学习模型筛查均未同时达到 in-domain、跨资产/世界和颜色压力门，因此本轮停在 `G1_model_recovery_in_domain_cross_asset_world_and_color_stress`，没有启动 500 scene/5000 帧正式数据、正式 live 门或真实 Nav2 spot-clean。

```text
REVIEW_PACKET_COMPLETE=true
READY_FOR_GPT_REVIEW_STAGE5B=false
READY_FOR_STAGE5C=false
competition_perception_pass=false
```

## 数据域命名

- `D0`：Stage5A deterministic color regression，只用于历史回归。
- `P1`：Stage5B NumPy/OpenCV 程序化筛查，不是 Gazebo camera 数据。
- `G1`：Gazebo Harmonic Ogre2 实际相机渲染的 RGB-D + semantic + instance synthetic 数据。
- `R1`：真实数据；本轮仍为空。

## Phase A：训练链自证

固定 12 帧，覆盖五类目标和至少四类 hard negatives，关闭增强训练轻量 U-Net：

| 门 | 结果 | 阈值 | 判定 |
|---|---:|---:|---|
| micro train macro F1 | 0.98124 | ≥0.98 | 通过 |
| micro foreground mIoU | 0.96333 | ≥0.95 | 通过 |
| PyTorch/ONNX 最大 logit 误差 | 0.00006866 | ≤0.0001 | 通过 |
| PyTorch/ONNX argmax agreement | 1.00000 | ≥0.9999 | 通过 |

类顺序、RGB 顺序、`float32/255` 归一化、128×96 resize、nearest mask 插值和 checkpoint 加载均显式检查。P1 分层瀑布在 `same_assets_unseen_seeds` 首次跌破 macro F1 0.70，确认旧失败不是 ONNX 或 ROS 预处理错位，而是 P1 泛化能力不足。

## G1 数据链

`sanitation_learning` 新增：

- 30 个 Gazebo 可渲染目标变体和 12 个 hard negatives；
- Gazebo `Label` system；
- 同 pose、同分辨率、同 FOV、同 10 Hz 的 RGB-D、semantic 和 instance cameras；
- `set_pose_vector` scene 随机化及 `light_config` 光照随机化；
- exact timestamp 采集、CameraInfo、固定 camera transform 和逐 scene manifest；
- scene/asset split、跨 split hash/pHash、类别/尺寸/遮挡估计和负样本分布 QA。

50-scene smoke：

| 项目 | 结果 |
|---|---:|
| scene / frame | 50 / 500 |
| annotation completeness | 100% |
| semantic-instance consistency error | 0 |
| exact RGB/depth/semantic/instance timestamp match | true |
| asset leakage | 0 |
| cross-split exact/perceptual duplicate | 0 / 0 |
| G1 smoke pass | true |

该结果只证明数据管线达到 screening 规模，不等于 500-scene 正式 G1 门。

## 三次模型筛查

| 尝试 | 数据/改动 | in-domain F1 | cross asset/world F1 | cross leaf/puddle mIoU | color stress F1 | 通过 |
|---|---|---:|---:|---:|---:|---|
| 1 | 4 m camera，基础增强 | 0.84511 | 0.65804 | 0.86313 | 0.47647 | 否 |
| 2 | 2.6 m camera，重颜色增强 | 0.69869 | 0.53370 | 0.66737 | 0.40960 | 否 |
| 3 | 2.6 m camera，48-base U-Net、中增强/稀有类加权 | 0.71019 | 0.37575 | 0.45005 | 0.27521 | 否 |

筛查门为 in-domain F1 ≥0.90、leaf/puddle mIoU ≥0.75、cross asset/world F1 ≥0.70、color stress F1 ≥0.60。三次均未全部通过；最佳跨资产/世界结果为尝试 1，但也不能进入正式扩量。尝试 2/3 的退化被完整保留，没有用后验阈值修改宣布通过。

## 回归

- `py scripts/ci_fast.py`：64 passed。
- ROS `sanitation_perception` + `sanitation_learning`：colcon build/test 通过，0 errors、0 failures。
- Stage5A：离线 perception 与 30 次 synthetic spot-clean 通过；真实 Gazebo live 处理 186 帧，MCAP 已记录，GT control violation 0。
- Stage4W seed 0：17/17 组件、经验覆盖率 93.6%、coverage localization RMSE 0.03260 m，碰撞/keepout/brush 违规均 0，退出时刷盘关闭。

## 复现

```powershell
py scripts/ci_fast.py

# 50 scene / 500 frame G1 smoke
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage5br_g1_docker.ps1 `
  -OutputName stage5br_g1_smoke50_scaled -SceneCount 50 -FramesPerScene 10 -RosDomainId 220

# G1 model screening
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage5br_g1_training_docker.ps1 `
  -DatasetName stage5br_g1_smoke50_scaled -OutputName stage5br_g1_model_screening3
```

## 下一步建议

先扩大几何、材质和 ground/world 多样性，并把离散小目标改为 detector + crop segmenter 或高分辨率分支；用验证集进行结构选择，保留同样的 cross-asset/world 和颜色压力门。只有 screening 四门均过，才允许生成 500 scene/5000 帧正式 G1 数据。
