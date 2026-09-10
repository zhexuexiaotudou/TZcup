# 目标—验收—执行交叉表

机器可读入口为
[`config/high_fidelity_vehicle/formal_goal_acceptance_crosswalk.json`](../config/high_fidelity_vehicle/formal_goal_acceptance_crosswalk.json)。
它把 A01–A21 目标路由到当前 32 步正式编排器和 27 个功能证据门，解决“组件通过、正式编排完成、比赛仿真通过、S100P 通过”被混成一个完成度的问题。

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
| 正式编排闭合 | false | 32 步完成，27 个证据门全绿，session 完成密封 |
| 比赛仿真通过 | false | 综合比赛任务、效率、安全、感知和泛化门满足赛题阈值 |
| 仿真产品完成 | false | 产品级详细指标、长稳、故障恢复和发布证据齐全 |
| S100 计算验收 | false | 真实 S100P 上 DOSOD/EdgeSAM/ROS 2 长稳及热功耗通过 |
| 产品集成就绪 | false | 仿真和 S100 输出接口、版本与模型产物绑定一致 |
| 产品实地就绪 | false | 实车执行器和真实场地安全验收完成（当前外部范围） |

## 当前硬缺口

1. A02 服务舱门修复需要在新鲜 r062 运行根上重跑物理门。
2. A08 的 3500 m²/h 指标与 0.45 m/s 速度上限冲突，需要分档复核并保留安全余量；不是简单改大速度常量。
3. A12 的固定产品验收基准已版本化为 [`a12-product-acceptance-specification.md`](a12-product-acceptance-specification.md)，并由 [`product_acceptance_contract.json`](../config/high_fidelity_vehicle/product_acceptance_contract.json) 约束 AUTO-15 的 18×10 执行账本、至少 30 个 mission group 及每个执行的 video/MCAP 保留。canonical replay/evidence producer 已实现并绑定当前 formal session/snapshot/run root、源码 commit/tree、模型/配置/数据集/容器/依赖身份，实际读取 MCAP 重算并审计视频；A12 因尚未在当前快照产生 180+30 份真实回执而保持 `NOT_RUN_CURRENT_SNAPSHOT`。producer 或静态合同不能使任何产品运行时状态变为 true。
4. A15 的一次性 hidden materializer、consumed/freeze/output-summary 账本和最终 12 站点重验合同已闭合；当前状态仅是尚未在新鲜快照上执行，不是通过。
5. A18 的 S100 性能部分与 A21 共用外部硬缺口，不能只标成“尚未运行”。
6. A19 已有 canonical producer、独立 validator、正式 runner 与 18 故障固定时间表，并作为 `a19_reliability` 接入总编排。当前没有新鲜 7200 秒 receipt，故状态为 `READY_FOR_FRESH_TWO_HOUR_RUNTIME` 而不是 PASS；fixture、预检、短跑或手写 JSON 均不能提升状态。
7. A20 的 canonical product replay producer 与严格 receipt validator 已实现；因当前快照尚无至少五个不同产品 MCAP 的回放重算、发布包与 rollback 演练，A20 保持 `NOT_RUN_CURRENT_SNAPSHOT`。AUTO-02/AUTO-03、coverage-only、嵌入 JSON 和历史 AUTO-16 报告均不可替代。
8. A21 仍缺项目 DOSOD HBM、真实非仿真图像/标定与 holdout 数据、真实功率测量。板卡在线本身不等于验收通过。

AUTO-05 G3/G4 不在当前 32 步或 S100 关键路径：未发现正式感知 runner、冻结模型或 HBM
直接消费其 dataset/screening 产物。它只与正式仿真共用 Gazebo 资源锁，因此当前不消耗租卡执行；若后续出现具体消费者，再按独立支线恢复。

## 执行顺序

先完成静态预检和当前阻断修复，再在 AutoDL 上保持单 Gazebo 世界串行跑 32 步；CPU-only、文档、板端只读和 S100P HBM/输入准备可与其并行。S100P 长稳采集在真实输入与四角色产物齐备后启动，最终通过 `--resume-s100` 回填同一 formal session，禁止跨历史运行拼接。
