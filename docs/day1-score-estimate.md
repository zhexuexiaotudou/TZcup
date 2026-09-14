# 2026-09-15 首轮作品当前得分估算（刷新版）

更新时间：2026-09-14（Asia/Shanghai）

刷新基线：`codex/day1-evidence-integration@05481fc55e6ef89b9396070381797f760c6b72c4`

旧状态参考：`codex/day1-scoring@9501d1c8d6f6b4f4ec95c1eea31fb3a4ad351391`

本页不是官方评分。它根据官方 100 分表、当前可复核证据和未闭合边界给出评审情景区间，不把离线候选、受控夹具、设计材料或未启动运行升级为 official PASS。

## 本次刷新结论

| 口径 | 更新后 | 旧版 | 变化原因 |
|---|---:|---:|---|
| 保守分 | 42-50 / 100 | 42-52 / 100 | 没有新的 live/official 硬门通过；离线定位和受控感知不进入保守分 |
| 合理分 | 52-62 / 100 | 52-62 / 100 | 定位离线候选和感知受控候选提高解释空间，但被 live、真实域和板端缺口抵消 |
| 乐观分 | 64-72 / 100 | 62-70 / 100 | 若评委接受同 run 因果离线定位和受控 95 候选，可争取更高上限 |
| 硬上限判断 | 不建议宣称超过 72 | 不宜宣称超过 70 | 仍需至少一个 live 端到端闭环才能稳定进入 70+ 区间 |

本页的区间是评审情景，不是把分项相加得到的官方分数。当前唯一验收矩阵仍为 `PASS=6`、`FAIL=1`、`PARTIAL=5`、`NOT_MEASURED=2`；离线候选没有修改正式 PASS/FAIL/NOT_MEASURED 状态。

## 最容易得分到最难的官方项排序

| 顺序 | 官方项 | 分值 | 当前状态 | 最短下一动作 | 证据路径 | 仍不能声称 |
|---:|---|---:|---|---|---|---|
| 1 | 紧急制动响应 <=1 s | 5 | `PASS_MEASURED` | 冻结触发、速度归零和墙钟边界，不再重跑 | `submission-package/evidence/day1/competition-width-efficiency-estop.md` | 不能写成实车法规级制动验证 |
| 2 | 清扫宽度 >=600 mm | 5 | `PASS_MEASURED` | 冻结 600 mm 连续清除带和 100 mm 栅格 | `submission-package/evidence/day1/competition-width-efficiency-estop.md` | 不能写成 1400 mm 连续刷宽 |
| 3 | 任务分解正确率 >=90% | 5 | `PASS_INTERNAL_FROZEN_REGRESSION` | 可补外部盲测；否则冻结开发集 36/37 和 holdout 32/32 | `submission-package/evidence/day1/competition-task-decomposition.json` | 不能写成公开盲测、语音识别或执行成功 |
| 4 | 报告逻辑清晰、数据详实 | 5 | `DELIVERED` | 冻结 23 页报告、补充包和哈希 | `submission-package/docs/技术方案报告.pdf`；`submission-package/supplements/nontechnical-score-closure/pdf/TZcup-nontechnical-score-closure.pdf` | 文档完整不等于技术指标通过 |
| 5 | 尘箱承载 >=40 L | 5 | `PARTIAL_DESIGN_STAGE` | 保留设计可用 48.347384 L；实物填充需制造和密封验证 | `submission-package/evidence/day1/application-value-dust-bin-design.json` | 不能写成实物容量或量产验证 |
| 6 | 跨学科融合 | 5 | `PARTIAL_SUBJECTIVE` | 把 ROS、控制、感知、嵌入式、仿真、环卫业务和材料串成模块证据图 | `submission-package/evidence/day1/integration-manifest.json`；`submission-package/docs/技术方案报告.pdf` | 不能用模块清单替代端到端闭环 |
| 7 | 算法优化 | 10 | `PARTIAL` | 用同一 run 对照展示因果低通定位、安全门控和任务 DSL；不要冒充实车泛化 | `F:/Project/TZcup/.workspace/worktrees/TZcup-day1-localization-50mm-recovery/artifacts/day1_localization_50mm_recovery_20260914/recovery_receipt.json` | 不能写成完整任务或外部公开验证 |
| 8 | 产业化方案加分 | 5 | `REVIEW_READY_NOT_FIELD_VALIDATED` | 提交 H0-H5、成本公式、风险和 IP 边界；真实试点需另行开展 | `submission-package/supplements/nontechnical-score-closure/industrialization-readiness.md`；`submission-package/supplements/nontechnical-score-closure/official-score-closure.md` | 不能写成订单、报价、SLA 或 ROI 已验证 |
| 9 | 复杂工况适配创新 | 5 | `DESIGN_OR_CONTROLLED_ONLY` | 对浅积水和落叶代理各做一次受控可复现试验 | `submission-package/supplements/nontechnical-score-closure/system-completeness.md`；`docs/competition-mapping-day1-run-06.md` | 不能外推真实光照、真实路面和长期泛化 |
| 10 | 易损件更换 <=3 步 | 5 | `DESIGN_ONLY_NOT_PHYSICALLY_MEASURED` | 先在实物或冻结机构上做非维修人员步骤测试 | `submission-package/supplements/nontechnical-score-closure/data/maintenance-procedure.json`；`submission-package/supplements/nontechnical-score-closure/maintenance-three-step.md` | 不能写成实件无需专业工具已通过 |
| 11 | 3 天掌握操作 | 2 | `PLANNED_NOT_MEASURED` | 招募真实新手队列，记录安全项、提示次数和退训 | `submission-package/supplements/nontechnical-score-closure/data/operator-certification-template.csv`；`submission-package/supplements/nontechnical-score-closure/operator-usability.md` | 不能写成真实用户已掌握 |
| 12 | 提升城市/园区清扫质量和效率 | 5 | `MODEL_ONLY_NOT_FIELD_MEASURED` | 做同区域、同班次、同质量口径的前后对照 | `submission-package/supplements/nontechnical-score-closure/social-benefit.md`；`submission-package/supplements/nontechnical-score-closure/evidence/social-benefit-model.json` | 不能写成已产生现场社会效益 |
| 13 | 硬件/软件功能完整 | 6 | `PARTIAL` | 用一次完整 mission 串起任务、地图、清扫、安全、返航、终态和 MCAP | `submission-package/evidence/day1/tooling/full-mission-demo/docs/day1-full-mission-demo-wrapper.md` | 不能用分模块 replay 替代完整任务 |
| 14 | 清扫效率 >=3500 m2/h | 5 | `PARTIAL_MEASURED` | 同一完整任务内计算有效清除面积并集除以总仿真时间 | `submission-package/evidence/day1/competition-width-efficiency-estop.md`；`submission-package/evidence/day1/tooling/full-mission-demo/scripts/day1_full_mission_acceptance.py` | 不能把固定 10 s 的 4320 m2/h 写成整任务效率 |
| 15 | 建图面积 >=20000 m2 | 5 | `OFFLINE_ALGORITHM_PARTIAL` | 在同一 fresh run 中记录 `/scan`、`/tf`、`/tf_static`、`/clock`、`/odom`，再做 live SLAM 或离线重放 | `submission-package/evidence/day1/competition-mapping-offline-raycast.md`；`submission-package/evidence/day1/tooling/live-slam-replay-fail-closed/reports/mapping/day1_live_slam_replay_preflight_20260914.json` | 不能把 22399.99 m2 离线射线重建写成 live SLAM |
| 16 | 物体识别准确率 >=95% | 10 | `CONTROLLED_CANDIDATE`；官方 `NOT_MEASURED_DEFINITION_UNSPECIFIED` | 冻结官方类别、样本域和匹配规则，再做真实域及 S100P 复算 | `submission-package/evidence/day1/perception-score95/docs/competition-perception-score95.md`；`submission-package/evidence/day1/perception-score95/artifacts/perception_score95_20260914/receipt.json` | 30 帧/76 实例五类 P/R=1.0 只是受控候选，不能写成官方或真实域 95% |
| 17 | 定位精度 <=50 mm | 5 | `OFFLINE_CANDIDATE_PASS`；live official `NOT_MEASURED` | 把 tau=1.5 s 因果低通等价接入实时 TF 链，再跑一次 Gazebo 正式验收 | `F:/Project/TZcup/.workspace/worktrees/TZcup-day1-localization-50mm-recovery/docs/day1-localization-50mm-recovery.md`；`F:/Project/TZcup/.workspace/worktrees/TZcup-day1-localization-50mm-recovery/artifacts/day1_localization_50mm_recovery_20260914/recovery_receipt.json` | 离线候选不是 live 或 official PASS；原 run 的 max 126.327803 mm 失败记录仍保留 |
| 18 | 避障成功率 >=95% | 5 | `ROOT_CAUSE_FIXED`；官方率 `NOT_MEASURED` | 先做唯一一次 `FUNCTIONAL_PASS_1_OF_1_NOT_OFFICIAL_95`，再做独立重复矩阵 | `F:/Project/TZcup/.workspace/worktrees/TZcup-day1-avoidance-recovery/docs/day1-avoidance-recovery-root-cause.md`；`F:/Project/TZcup/.workspace/worktrees/TZcup-day1-avoidance-recovery/artifacts/day1_dynamic_avoidance_recovery_20260914/failure_receipt.json` | 单次功能通过仍不等于 95%，更不能把历史 run-19b/21/22 改成成功 |
| 19 | 演示视频清晰流畅 | 4 | `KIT_ONLY_VIDEO_NOT_MEASURED` | 先完成完整任务，再录 5-10 分钟连续视频和版本叠层 | `submission-package/video/视频索引.md`；`submission-package/supplements/nontechnical-score-closure/demo-video-production.md` | 现有 6.333 s GUI 片段不能当正式 Demo |
| 20 | 日均故障率 <1% | 3 | `INSTRUMENTED_NOT_MEASURED` | 按事件字典积累车辆日；零主故障需至少 299 车辆日 | `submission-package/supplements/nontechnical-score-closure/reliability-case.md`；`submission-package/supplements/nontechnical-score-closure/data/reliability-event-dictionary.json` | 短仿真或一次任务不能推出日均故障率 |
| 21 | 已申请专利或发表论文 | 5 | `NOT_ELIGIBLE_NO_FILING_OR_PUBLICATION` | 只能提交披露模板和提纲；取得申请号或 DOI 后才可计分 | `submission-package/supplements/nontechnical-score-closure/ip-research-boundary.md`；`submission-package/supplements/nontechnical-score-closure/invention-disclosure-template.md` | 模板、检索清单和论文提纲不是加分证据 |

`循迹`不是独立 100 分项，但已由 `47e3cb3` 的两个 Nav2 goal `[4,4]` 提供局部证据。它只能支撑完整任务链路，不能单独补分。

## Coverage 运行状态

| 运行 | 参数 | 结果 | 是否计分 |
|---|---|---|---|
| run-10 | `headland.width_m=0.75` | coverage 规划前因 clearance 拒绝 | 否 |
| run-11 | `headland.width_m=1.35` | headland clearance 通过，但 planner `default_headland_width=0`，11 条 swath 越界 | 否 |
| run-12 | 参数修复后待运行 | `NOT_STARTED` | 否，不得预填结果 |

run-10 的 39355-byte evidence archive 位于 `codex/day1-coverage-run` 的
`artifacts/day1_bounded_coverage_20260914/day1-coverage-run10-d115c89-evidence.tar.gz`，对应提交 `31e9e3a`。
run-11 的本地 ops 证据位于
`F:/Project/TZcup/.workspace/worktrees/TZcup-day1-coverage-run/.work/day1-coverage-run11-d2335f3/run11-d2335f3-ops.tar.gz`，
`headland.width_m=1.35` 由提交 `d2335f3` 固定。planner 参数修复尚未形成 run-12 结果，因此 coverage、清扫面积和完整任务效率仍是 `NOT_MEASURED`。

当前边界：

- `run-10` 和 `run-11` 都不能提升 `清扫效率`、`建图` 或 `硬件/软件完整` 状态。
- `full-mission` wrapper 只提供 fail-closed 验收入口，`official_points_claimed=0`。
- `live-SLAM` 审计工具只证明 run-06 缺少 SLAM 必需话题，`official_points_claimed=0`。

## 当前一票否决风险

1. 官方识别 `R01` 的定义、分母和匹配规则未冻结，真实域和 S100P 板端未测。
2. live 定位仍是 `NOT_MEASURED`；离线候选不能替代 Gazebo 正式重跑。
3. 20000 m2 只有离线射线重建；live SLAM、地图生命周期和 map reuse 未测。
4. 动态避障只有 root cause、修复和单次功能验证入口；官方 95% 无分母。
5. 完整清扫任务、coverage、返航和 5-10 分钟 Demo 未闭环。
6. 40 L 尘箱、维护步骤、3 天培训和日均故障率都没有物理或现场验收。

## 如果只剩最后一个资源窗口

1. 先让唯一 Gazebo owner 完成参数修复后的 run-12，保留 path、report、dirt、brush、final state 和 MCAP；有完整证据才升级 coverage 或效率状态。
2. 若 run-12 不能产生完整任务数据，优先执行定位 live 因果低通重跑，因为定位会影响规划、避障和整任务可信度。
3. 定位恢复后再做一次动态避障 `1/1 functional`；不得把单次结果写成 95%。
4. 不并行启动第二套 Gazebo，不迁移旧 PASS，不降低 coverage、95% 或 50 mm 门槛。

## 证据与回滚

- 报告合并：`codex/day1-evidence-integration@05481fc55e6ef89b9396070381797f760c6b72c4`
- 报告 PDF SHA-256：`3cc8158de1d57f6e5b692fae0571e862f6ce4b7c6f06647af1d16447ac37cc8b`
- 评分刷新基线：`codex/day1-score-refresh`，父提交为 `05481fc55e6ef89b9396070381797f760c6b72c4`
- 定位候选提交：`a7841c9e024175901065d2c583f32a4a2d89f50f`，回滚点 `47e3cb3a7ecc01edd82aa23a3b54cbeaffc418bc`
- 避障修复提交：`99ee7f0d01544d65443b68f189edb28fc21d0459`，回滚点 `b74c702777b9974932f959800076071663255752`
- 感知候选提交：`3882dc34e756e1c361852027d49e55acac2b2a74`，回滚点 `906ea4f`
- Coverage run-11 配置提交：`d2335f3061146fedef8b9f731eb54760b0f82386`
- Coverage run-12 结果：`NOT_STARTED`
