# 当前项目状态

本页只描述当前有效状态，不记录按日期或尝试次数排列的开发过程。历史变更由 Git、PR 和 `artifacts/` 中的紧凑证据承担。

## 产品成熟度

| 领域 | 当前结论 | 主要边界 |
|---|---|---|
| 仿真与车辆模型 | 可运行 | 已具备 Gazebo 园区、清扫车、传感器和基础清扫演示 |
| 定位、导航与覆盖清扫 | 可运行 | 已具备 SLAM、Nav2、安全控制、覆盖规划和补扫链路 |
| 人机监督与可视化 | 可运行 | 支持 Gazebo、RViz 和浏览器看板；不替代真实场地验收 |
| 学习感知 | 内部阻断 | 静态检测候选通过，但运动相机在线质量和严格性能门未通过 |
| J6 端侧交付 | 外部阻断 | 缺少当前可用工具链、冻结 student 和实体板验证 |
| 真实场地交付 | 外部阻断 | 缺少正式 RGB-D 录制、独立地图真值和现场验收 |

因此，仓库当前不能表述为已经完成实车产品部署。有效产品标志仍为：

- `MODEL_BLOCKED_INTERNAL=true`
- `PRODUCT_X86_PERCEPTION_READY=false`
- `PRODUCT_J6_TOOLCHAIN_READY=false`
- `PRODUCT_J6_BOARD_READY=false`
- `PRODUCT_FIELD_READY=false`

## 当前感知结论

Detector Data Recovery V4 的 G7 静态候选达到 recall/precision `0.9778/0.9778`，但运动相机兼容回归的 eventual recall 为 `0.3898`，metal recall 为 `0.1053`，产品地图 precision 为 `0.2111`。性能回放达到 `9.9974 Hz`、P95 `155.83 ms`、掉帧率 `0`，仍未满足严格 `>=10 Hz` 门。

这些结果只证明静态候选具备继续研究的价值，不能解锁 freeze、G5_V2、Spot Cleaning、J6 或现场发布。完整边界与复现入口见 [Detector Data Recovery V4](detector-data-recovery-v4.md)。

## 权威入口

- 系统目标与接口：[`PROJECT_SPEC.md`](../PROJECT_SPEC.md)
- 验收门定义：[`STAGE_GATES.md`](../STAGE_GATES.md)
- 当前机器可读状态：[`FINAL_AUTONOMOUS_STATUS.json`](../FINAL_AUTONOMOUS_STATUS.json)
- 当前阻塞项：[`FINAL_BLOCKER_REGISTER.json`](../FINAL_BLOCKER_REGISTER.json)
- DDRV4 最终证据：[`artifacts/detector_data_recovery_v4_20260811T134117Z/final/`](../artifacts/detector_data_recovery_v4_20260811T134117Z/final/)
- 开发和交付规则：[开发工作流](development-workflow.md)

## 下一步解锁条件

后续工作只有在获得相应资源后继续：

1. 获得覆盖 behind-FOV、转弯、遮挡和反光场景的独立运动相机开发数据，重新执行在线产品质量门；
2. 获得当前 J6 工具链、冻结 student 和授权实体板，执行可追溯转换与实板验收；
3. 获得正式 RGB-D 录制、独立地图真值和现场授权，执行真实场地验收。

更新本页时应直接替换已经失效的结论，不追加日期、轮次或提交日志。
