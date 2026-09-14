# 硬件/软件功能完整度矩阵

## 1. 使用方式

本矩阵用于帮助评委快速定位“模块存在、证据在哪里、还缺什么”。它不能替代正式报告中的 FAIL、PARTIAL 和 NOT_MEASURED 状态。

| 模块 | 当前能力 | 状态 | 主要证据 | 阻断完整任务的原因 |
|---|---|---|---|---|
| 车辆与底盘 | URDF、轮系、传感器、清洗部件接口 | `PARTIAL` | `starter_ws/src/sanitation_vehicle_description` | 维护件和完整执行机构未实物冻结 |
| 场景与污物 | 测试世界、污物状态、积水/落叶代理 | `PARTIAL` | `starter_ws/src/sanitation_worlds` | 代理对象不等于真实域 |
| 感知 | RGB-D/2D 候选链、GPU 离线校准 | `FAIL` | `submission-package/evidence/day1/competition-perception-gpu-recovery.json` | 官方 95% 未测，工程策略召回不足 |
| 建图 | 离线射线重建、历史板端回放 | `PARTIAL` | `submission-package/evidence/day1/offline-raycast-mapping` | 实时 SLAM 未闭环 |
| 定位 | 严格配对、RMSE、Nav2 路线 | `PARTIAL` | `submission-package/evidence/day1/localization-rental-final` | 官方统计口径未冻结，max 失效 |
| 规划与控制 | Nav2 目标、速度安全门、急停 | `PARTIAL` | `submission-package/evidence/day1/competition-width-efficiency-estop.md` | 动态避障和完整任务未闭环 |
| 清扫执行 | 刷体接触、600 mm 连续清除带 | `PASS_MEASURED` | 同上 | 只覆盖受控固定试验 |
| 任务分解 | 冻结 UTF-8 到 DSL 回归 | `PASS_INTERNAL_FROZEN_REGRESSION` | `submission-package/evidence/day1/competition-task-decomposition.json` | 不是公开盲测或语音端到端 |
| 板端部署 | 控制/定位/建图有限回放 | `PARTIAL` | `submission-package/evidence/board` | 感知 BLOCKED，无联合负载和 HIL |
| 运维与用户 | 培训、SOP、按钮/维护模式设计 | `DESIGN_ONLY` | `codex/day1-value-package/application-value-package` | 无用户队列和实件维护试验 |
| 文档与回滚 | 报告、索引、失败留存、哈希 | `DELIVERED` | `submission-package/docs` 和本包 | 不改变技术未达标事实 |

## 2. 完成度结论

可以主张“模块化方案完整、证据链可追踪、失败保留”，但不能主张“硬件/软件端到端功能完整”。硬件/软件 6 分应按模块完成度和任务闭环分别评审。

## 3. 最短端到端验收

最终演示至少需要同一冻结版本下：

1. 传感器或回放输入；
2. 地图、定位和任务接受；
3. 车辆真实运动或完整仿真运动；
4. 刷体/清扫执行；
5. 障碍或急停事件；
6. 覆盖或任务完成；
7. 终态、日志、版本和哈希。
