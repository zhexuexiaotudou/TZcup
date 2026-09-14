# TZcup 2026-09-15 首轮提交最终记分板（视频纳入版）

更新时间：2026-09-15（Asia/Shanghai）

刷新基线：`codex/day1-evidence-integration@05481fc55e6ef89b9396070381797f760c6b72c4`

上一评分刷新：`codex/day1-score-refresh@c1c254ec27bcc9f746f8215533016d171ba1b2fa`

视频归档基线：`codex/day1-video-final@b663ca9871d75caf26b83c8f8fd40b0d906b1539`

本页记录当前交付、证据、边界和下一步。数值区间见 `docs/day1-score-estimate.md`，机器可读副本见 `docs/day1-score-refresh.json`。

## 状态变化

| 项目 | 旧状态 | 当前状态 | 结论 |
|---|---|---|---|
| 感知 | GPU 诊断 raw 41/0/35、policy 33/0/43 | 受控夹具全 30 帧、76 实例，raw/policy 五类 P/R=1.0、TP/FP/FN=76/0/0 | 状态为 `reviewable controlled-domain candidate`；官方 R01、真实域、S100P 板端仍 `NOT_MEASURED_DEFINITION_UNSPECIFIED` |
| 定位 | 原 run max 126.327803 mm，严格判据 FAIL | 同 run 因果低通离线候选，tau=1.5 s 时 RMSE/P95/max=29.192/40.285/47.566 mm | 严格 max 仅在离线因果重放通过；不是 live/official PASS，仍需 Gazebo 正式重跑 |
| 避障 | run-19b/21/22 交互存在但 goal 失败 | root cause 定位为 global EKF 将 `navsat_transform` 生成的 `/odometry/gps` 作为自身反馈，导致 `map->odom` 漂移 | 最小修复和 single-trial harness 已提交；官方 95% 仍 `NOT_MEASURED`，下一次成功最多 `1/1 functional` |
| Coverage | run-05 卡在 `/ground_truth/odom` | run-10 `headland=0.75` clearance 拒绝；run-11 `headland=1.35` clearance 通过，但 `default_headland_width=0` 导致 11 条 swath 越界 | run-12 未开始，coverage、清扫面积和完整任务效率均不能升级 |
| 演示视频 | 只有 6.333 s GUI 片段 | 300.000 s 成片、1080p、30 fps、中文旁白/字幕、章节和完整校验包 | `DELIVERED_MODULAR_DEMO`；可主张 5-10 分钟、清晰流畅和操作说明，按 2-3/4 准备，评委接受模块化综合演示时可裁量至 4/4 |
| Full mission | 只有 wrapper 设计 | fail-closed wrapper 已存在；模块化视频另包交付 | `official_points_claimed=0`；视频不是同一连续任务，完整 mission、return-home 和终态仍缺 |
| Live SLAM | run-06 超时 | fail-closed replay 审计已存在 | run-06 缺 `/scan`、`/tf`、`/tf_static`，不能离线补成 live SLAM；`official_points_claimed=0` |

## 已可审阅或已计分

| 官方项 | 分值 | 状态 | 可提交结论 | 不能提交的结论 |
|---|---:|---|---|---|
| 紧急制动功能 | 5 | `PASS_MEASURED` | 行驶中触发后，仿真时间 0.489 s 达到低速阈值并保持 1.5 s | 不能替代实车法规验证 |
| 紧急制动响应 <=1 s | 5 | `PASS_MEASURED` | 仿真 0.489 s；墙钟 4.809955 s 单独披露 | 不能只写 0.489 s 而省略墙钟边界 |
| 清扫宽度 >=600 mm | 5 | `PASS_MEASURED` | 最大连续清除带 600 mm | 不能写成 1400 mm 连续刷宽 |
| 一种清扫方式 | - | `PASS_MEASURED` | 刷体接触、污物计数从 606 到 1806、前后状态齐全 | 只覆盖固定受控试验 |
| 任务分解 >=90% | 5 | `PASS_INTERNAL_FROZEN_REGRESSION` | 开发集 36/37，内部 holdout 32/32 | 不是公开盲测、语音识别或任务执行 |
| 文档质量 | 5 | `DELIVERED` | 23 页报告、14 页补充包、证据索引、哈希和复算工具 | 文档完整不代表技术项通过 |
| 尘箱 >=40 L | 5 | `PARTIAL_DESIGN_STAGE` | 设计可用 48.347384 L，10 mm 敏感度 41.208 L | 不是实物填充或量产容量 |
| 演示视频清晰流畅 | 4 | `DELIVERED_MODULAR_DEMO` | 300.000 s、1920x1080、30 fps、H.264/AAC/mov_text；旁白、字幕、章节和完整解码均通过；另附 450 s 未剪辑连续 Gazebo 运动/安全片段；按 2-3/4 准备，评委接受综合演示时可争取 4/4 | 不是同一连续完整清扫任务、live SLAM、95% 避障或实车验证 |

## 部分得分与待补测

| 官方项 | 分值 | 当前状态 | 下一步最短路径 | 主要证据 |
|---|---:|---|---|---|
| 效率 >=3500 m2/h | 5 | `PARTIAL_MEASURED` | 在完整任务内计算有效清除面积并集除以总仿真时间 | `submission-package/evidence/day1/competition-width-efficiency-estop.md` |
| 定位 <=50 mm | 5 | `OFFLINE_CANDIDATE_PASS`；live `NOT_MEASURED` | 把因果低通接入实时 TF 链，重跑 Gazebo 并报告 RMSE/P95/max | `F:/Project/TZcup/.workspace/worktrees/TZcup-day1-localization-50mm-recovery/artifacts/day1_localization_50mm_recovery_20260914/recovery_receipt.json` |
| 建图 >=20000 m2 | 5 | `OFFLINE_ALGORITHM_PARTIAL` | fresh run 同时记录 SLAM 必需话题，再做 live 或离线 replay | `submission-package/evidence/day1/competition-mapping-offline-raycast.md` |
| 避障 >=95% | 5 | `ROOT_CAUSE_FIXED`；率 `NOT_MEASURED` | 先做单次 functional，再做独立重复矩阵 | `F:/Project/TZcup/.workspace/worktrees/TZcup-day1-avoidance-recovery/docs/day1-avoidance-recovery-root-cause.md` |
| 识别 >=95% | 10 | `CONTROLLED_CANDIDATE`；官方 `NOT_MEASURED` | 冻结官方定义，再做真实域和 S100P 板端 | `submission-package/evidence/day1/perception-score95/artifacts/perception_score95_20260914/receipt.json` |
| 复杂工况 | 5 | `DESIGN_OR_CONTROLLED_ONLY` | 浅积水和落叶各做一次受控复现 | `submission-package/supplements/nontechnical-score-closure/system-completeness.md` |
| 硬件/软件完整 | 6 | `PARTIAL` | 同一任务串起输入、清扫、安全、coverage、返航和终态 | `submission-package/evidence/day1/tooling/full-mission-demo/docs/day1-full-mission-demo-wrapper.md` |

## 未测或暂不可得分

| 官方项 | 分值 | 状态 | 原因 |
|---|---:|---|---|
| 3 天掌握操作 | 2 | `PLANNED_NOT_MEASURED` | 无真实新手队列 |
| 易损件更换 <=3 步 | 5 | `DESIGN_ONLY` | 缺少物理步骤、工具托盘和锁止测试 |
| 日均故障率 <1% | 3 | `NOT_MEASURED` | 无车辆日分母 |
| 社会效益 | 5 | `MODEL_ONLY` | 无同区域同班次现场对照 |
| 同一连续完整任务视频 | - | `NOT_MEASURED` | 300 s 成片是模块化成功素材综合剪辑，不证明所有镜头属于同一连续任务 |
| 产业化加分 | 5 | `REVIEW_READY_NOT_FIELD_VALIDATED` | 无真实订单、报价、SLA 或试点 |
| 专利或论文加分 | 5 | `NOT_ELIGIBLE` | 无申请号、受理回执、DOI 或发表证明 |

## Coverage 资源状态

- run-10：`headland.width_m=0.75`，规划前 clearance 拒绝。证据归档在 `codex/day1-coverage-run` 的 `artifacts/day1_bounded_coverage_20260914/day1-coverage-run10-d115c89-evidence.tar.gz`，提交 `31e9e3a`。
- run-11：`headland.width_m=1.35`，clearance 通过；planner 仍是 `default_headland_width=0`，11 条 swath 越界。本地 ops 证据为 `F:/Project/TZcup/.workspace/worktrees/TZcup-day1-coverage-run/.work/day1-coverage-run11-d2335f3/run11-d2335f3-ops.tar.gz`。
- run-12：`NOT_STARTED`。planner 参数修复未形成正式运行回执前，不能计算 coverage、dirt、brush、效率或完整任务得分。

## 一票否决与提交边界

1. 识别官方 ≥95% 仍没有定义域、分母和真实域结果，这是最高风险项。
2. live 定位和 live SLAM 均未通过；离线候选只能放在候选章节。
3. 避障官方 95% 无独立分母；单次 functional 通过也不能升级官方状态。
4. 40 L、维护、培训和可靠性都没有实物或现场数据。
5. 模块化 Demo 已交付，但完整连续任务未闭环；不能把模块化综合演示当作完整 mission PASS。

因此，本材料包可以审阅，但不能宣称为整包达标、关键技术全部通过或已进入稳定 70+；乐观 `76` 取决于评委是否接受模块化综合演示并给出视频 4/4。

## 交付入口

| 交付物 | 路径或提交 | 状态 |
|---|---|---|
| 合并技术报告 | `submission-package/docs/技术方案报告.pdf`；SHA-256 `3cc8158de1d57f6e5b692fae0571e862f6ce4b7c6f06647af1d16447ac37cc8b` | 23 页，已冻结 |
| 非技术补充包 | `submission-package/supplements/nontechnical-score-closure/pdf/TZcup-nontechnical-score-closure.pdf` | 14 页，设计/计划/未测边界明确 |
| 感知候选 | `3882dc34e756e1c361852027d49e55acac2b2a74` | 受控候选，不计官方 10 分 |
| 定位候选 | `a7841c9e024175901065d2c583f32a4a2d89f50f` | 离线候选，不计 official PASS |
| 避障修复 | `99ee7f0d01544d65443b68f189edb28fc21d0459` | 修复已提交，运行未完成 |
| Coverage | `d2335f3061146fedef8b9f731eb54760b0f82386`；run-12 `NOT_STARTED` | 不计分 |
| 5 分钟演示视频 | `codex/day1-video-final@b663ca9871d75caf26b83c8f8fd40b0d906b1539`；`F:/Project/TZcup/.workspace/worktrees/TZcup-day1-video-final/submission-package/video/final/TZcup_5min_video.mp4`；SHA-256 `631c2cfbf0e459bbcf74d7c3f9ddb6c1fc5a6519dbb6794bd4a6ea148d94e097` | `DELIVERED_MODULAR_DEMO`，建议 2-3/4，最高裁量 4/4；不是完整 mission PASS |

## 回滚与保留

- 旧评分状态：`codex/day1-scoring@9501d1c8d6f6b4f4ec95c1eea31fb3a4ad351391`
- 上一评分刷新：`codex/day1-score-refresh@c1c254ec27bcc9f746f8215533016d171ba1b2fa`，父提交 `05481fc55e6ef89b9396070381797f760c6b72c4`
- 视频归档提交：`b663ca9871d75caf26b83c8f8fd40b0d906b1539`
- 视频回滚点：`e6caf27`
- 定位回滚点：`47e3cb3a7ecc01edd82aa23a3b54cbeaffc418bc`
- 避障回滚点：`b74c702777b9974932f959800076071663255752`
- 感知回滚点：`906ea4f`
- Coverage run-10/11 和失败日志保留；未清理、未迁移、未覆盖。
