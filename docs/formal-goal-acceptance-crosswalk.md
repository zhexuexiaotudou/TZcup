# 目标—验收—执行交叉表

机器可读入口为
[`config/high_fidelity_vehicle/formal_goal_acceptance_crosswalk.json`](../config/high_fidelity_vehicle/formal_goal_acceptance_crosswalk.json)。
它把 A01–A21 目标路由到当前 31 步正式编排器和 26 个功能证据门，解决“组件通过、正式编排完成、比赛仿真通过、S100P 通过”被混成一个完成度的问题。

## 使用边界

- 本表只证明目标有明确执行入口，不是 PASS 证据。
- 只有同一新鲜源码快照、同一 formal session 下产生并通过校验的证据，才能改变正式验收状态。
- `HISTORICAL_COMPONENT_PASS_NOT_FORMAL_INTEGRATED` 只保留历史组件能力；不得拼接成综合比赛通过。
- AutoDL 只跑原生 ROS/Gazebo/CUDA/HBM 工具链，不在容器里嵌套 Docker。
- S100P 门只接受真实板端证据；PC、仿真、离线 schema PASS 均不能替代。
- 真实执行器、车辆场地、相机标定数据和功率仪等外部输入未具备时，保持 BLOCKED，不造假输入。

## 状态层级

| 层级 | 当前值 | 完成条件 |
|---|---:|---|
| 功能系统闭合 | false | 38 个功能位置及其依赖门均由当前快照证据支持 |
| 正式编排闭合 | false | 31 步完成，26 个证据门全绿，session 完成密封 |
| 比赛仿真通过 | false | 综合比赛任务、效率、安全、感知和泛化门满足赛题阈值 |
| 仿真产品完成 | false | 产品级详细指标、长稳、故障恢复和发布证据齐全 |
| S100 计算验收 | false | 真实 S100P 上 DOSOD/EdgeSAM/ROS 2 长稳及热功耗通过 |
| 产品集成就绪 | false | 仿真和 S100 输出接口、版本与模型产物绑定一致 |
| 产品实地就绪 | false | 实车执行器和真实场地安全验收完成（当前外部范围） |

## 当前硬缺口

1. A02 服务舱门修复需要在新鲜 r062 运行根上重跑物理门。
2. A08 的 3500 m²/h 指标与 0.45 m/s 速度上限冲突，需要分档复核并保留安全余量；不是简单改大速度常量。
3. A12 历史文档提到的 `product_acceptance_v1` 文件在当前仓库历史中不存在，不能假装仍由旧脚本覆盖。
4. A19 当前 26 门没有独立的全时长 soak/故障注入门；必须补齐或明确由哪些正式报告等价覆盖。
5. A21 仍缺项目 DOSOD HBM、真实非仿真图像/标定与 holdout 数据、真实功率测量。板卡在线本身不等于验收通过。

## 执行顺序

先完成静态预检和当前阻断修复，再在 AutoDL 上保持单 Gazebo 世界串行跑 31 步；CPU-only、文档、板端只读和 S100P HBM/输入准备可与其并行。S100P 长稳采集在真实输入与四角色产物齐备后启动，最终通过 `--resume-s100` 回填同一 formal session，禁止跨历史运行拼接。
