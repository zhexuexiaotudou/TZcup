# 2026-09-15 首轮作品当前得分估算（视频纳入版）

更新时间：2026-09-15（Asia/Shanghai）

刷新基线：`codex/day1-evidence-integration@05481fc55e6ef89b9396070381797f760c6b72c4`

旧状态参考：`codex/day1-scoring@9501d1c8d6f6b4f4ec95c1eea31fb3a4ad351391`

上一评分刷新：`codex/day1-score-refresh@c1c254ec27bcc9f746f8215533016d171ba1b2fa`

视频归档基线：`codex/day1-video-final@b663ca9871d75caf26b83c8f8fd40b0d906b1539`

本页不是官方评分。它根据官方 100 分表、当前可复核证据和未闭合边界给出评审情景区间，不把离线候选、受控夹具、设计材料或未启动运行升级为 official PASS。

## 本次刷新结论

| 口径 | 视频纳入后 | 上一刷新 | 最初版本 | 变化原因 |
|---|---:|---:|---:|---|
| 保守分 | 44-52 / 100 | 42-50 / 100 | 42-52 / 100 | 300 秒模块化演示视频按保守 2/4 计入；完整任务连续闭环仍未测 |
| 合理分 | 55-65 / 100 | 52-62 / 100 | 52-62 / 100 | 视频材料证据完整，按 3/4 准备；受控感知和离线定位只能提供有限解释空间 |
| 乐观分 | 67-76 / 100 | 64-72 / 100 | 62-70 / 100 | 若评委接受章节化操作说明和模块化综合演示并给出 4/4，可争取上限 |
| 硬上限判断 | 暂不建议宣称超过 76 | 不建议宣称超过 72 | 不宜宣称超过 70 | 76 上限依赖评委裁量，不代表完整任务、live SLAM 或实车验收通过 |

本页的区间是评审情景，不是把分项相加得到的官方分数。视频新增分值按保守 `+2`、合理 `+3`、乐观 `+4` 计入。当前唯一验收矩阵仍为 `PASS=6`、`FAIL=1`、`PARTIAL=5`、`NOT_MEASURED=2`；离线候选、受控感知和模块化视频都没有修改正式 PASS/FAIL/NOT_MEASURED 状态。

视频归档状态为 `DELIVERED_MODULAR_DEMO`，不是 `official full-mission PASS`。可证明官方要求的 5-10 分钟时长、清晰度、流畅播放、中文旁白/字幕和章节化操作说明；不能证明所有镜头属于同一连续任务。

## 最容易得分到最难的官方项排序

| 顺序 | 官方项 | 分值 | 当前状态 | 最短下一动作 | 证据路径 | 仍不能声称 |
|---:|---|---:|---|---|---|---|
| 1 | 紧急制动响应 <=1 s | 5 | `PASS_MEASURED` | 冻结触发、速度归零和墙钟边界，不再重跑 | `submission-package/evidence/day1/competition-width-efficiency-estop.md` | 不能写成实车法规级制动验证 |
| 2 | 清扫宽度 >=600 mm | 5 | `PASS_MEASURED` | 冻结 600 mm 连续清除带和 100 mm 栅格 | `submission-package/evidence/day1/competition-width-efficiency-estop.md` | 不能写成 1400 mm 连续刷宽 |
| 3 | 任务分解正确率 >=90% | 5 | `PASS_INTERNAL_FROZEN_REGRESSION` | 可补外部盲测；否则冻结开发集 36/37 和 holdout 32/32 | `submission-package/evidence/day1/competition-task-decomposition.json` | 不能写成公开盲测、语音识别或执行成功 |
| 4 | 报告逻辑清晰、数据详实 | 5 | `DELIVERED` | 冻结 24 页报告、补充包和哈希 | `submission-package/docs/技术方案报告.pdf`；`submission-package/supplements/nontechnical-score-closure/pdf/TZcup-nontechnical-score-closure.pdf` | 文档完整不等于技术指标通过 |
| 5 | 演示视频清晰流畅 | 4 | `DELIVERED_MODULAR_DEMO` | 冻结 300 s 成片、旁白/字幕、章节和哈希；另附 450 s 未剪辑连续 Gazebo 运动/安全片段，按 2-3/4 准备，评委接受综合演示时争取 4/4 | `F:/Project/TZcup/.workspace/worktrees/TZcup-day1-video-report-sync/submission-package/video/final/TZcup_5min_video.mp4`；`.../video/evidence/continuous-cleaning-safety-450s/gazebo-cleaning-raw.mp4`；`.../video/validation/video-package-validation.json` | 不能写成同一连续完整清扫任务、live SLAM、避障 95% 或实车验证通过 |
| 6 | 尘箱承载 >=40 L | 5 | `PARTIAL_DESIGN_STAGE` | 保留设计可用 48.347384 L；实物填充需制造和密封验证 | `submission-package/evidence/day1/application-value-dust-bin-design.json` | 不能写成实物容量或量产验证 |
| 7 | 跨学科融合 | 5 | `PARTIAL_SUBJECTIVE` | 把 ROS、控制、感知、嵌入式、仿真、环卫业务和材料串成模块证据图 | `submission-package/evidence/day1/integration-manifest.json`；`submission-package/docs/技术方案报告.pdf` | 不能用模块清单替代端到端闭环 |
| 8 | 算法优化 | 10 | `PARTIAL` | 用同一 run 对照展示因果低通定位、安全门控和任务 DSL；60 秒定位可视化已入正式包，不复跑 | `F:/Project/TZcup/.workspace/worktrees/TZcup-day1-video-report-sync/submission-package/video/evidence/localization-causal-filter-60s/localization_tracking_success_1080p.mp4` | 不能写成完整任务、live PASS 或外部公开验证 |
| 9 | 产业化方案加分 | 5 | `REVIEW_READY_NOT_FIELD_VALIDATED` | 提交 H0-H5、成本公式、风险和 IP 边界；真实试点需另行开展 | `submission-package/supplements/nontechnical-score-closure/industrialization-readiness.md`；`submission-package/supplements/nontechnical-score-closure/official-score-closure.md` | 不能写成订单、报价、SLA 或 ROI 已验证 |
| 10 | 复杂工况适配创新 | 5 | `CONTROLLED_SIMULATION_CLIP` | 浅积水清洁与落叶/颗粒物清扫各完成受控45s画面；真实场景不补宣称 | `submission-package/video/evidence/complex-conditions-90s/complex_conditions_demo_90s.mp4`；`submission-package/supplements/nontechnical-score-closure/system-completeness.md` | 不能外推真实光照、真实路面和长期泛化 |
| 11 | 易损件更换 <=3 步 | 5 | `DESIGN_ONLY_NOT_PHYSICALLY_MEASURED` | 先在实物或冻结机构上做非维修人员步骤测试 | `submission-package/supplements/nontechnical-score-closure/data/maintenance-procedure.json`；`submission-package/supplements/nontechnical-score-closure/maintenance-three-step.md` | 不能写成实件无需专业工具已通过 |
| 12 | 3 天掌握操作 | 2 | `PLANNED_NOT_MEASURED` | 招募真实新手队列，记录安全项、提示次数和退训 | `submission-package/supplements/nontechnical-score-closure/data/operator-certification-template.csv`；`submission-package/supplements/nontechnical-score-closure/operator-usability.md` | 不能写成真实用户已掌握 |
| 13 | 提升城市/园区清扫质量和效率 | 5 | `MODEL_ONLY_NOT_FIELD_MEASURED` | 做同区域、同班次、同质量口径的前后对照 | `submission-package/supplements/nontechnical-score-closure/social-benefit.md`；`submission-package/supplements/nontechnical-score-closure/evidence/social-benefit-model.json` | 不能写成已产生现场社会效益 |
| 14 | 硬件/软件功能完整 | 6 | `PARTIAL` | 用一次完整 mission 串起任务、地图、清扫、安全、返航、终态和 MCAP | `submission-package/evidence/day1/tooling/full-mission-demo/docs/day1-full-mission-demo-wrapper.md` | 不能用分模块 replay 或模块化视频替代完整任务 |
| 15 | 清扫效率 >=3500 m2/h | 5 | `PARTIAL_MEASURED` | 同一完整任务内计算有效清除面积并集除以总仿真时间 | `submission-package/evidence/day1/competition-width-efficiency-estop.md`；`submission-package/evidence/day1/tooling/full-mission-demo/scripts/day1_full_mission_acceptance.py` | 不能把固定 10 s 的 4320 m2/h 写成整任务效率 |
| 16 | 建图面积 >=20000 m2 | 5 | `OFFLINE_ALGORITHM_PARTIAL` | 在同一 fresh run 中记录 `/scan`、`/tf`、`/tf_static`、`/clock`、`/odom`，再做 live SLAM 或离线重放 | `submission-package/evidence/day1/competition-mapping-offline-raycast.md`；`submission-package/evidence/day1/tooling/live-slam-replay-fail-closed/reports/mapping/day1_live_slam_replay_preflight_20260914.json` | 不能把 22399.99 m2 离线射线重建写成 live SLAM |
| 17 | 物体识别准确率 >=95% | 10 | `CONTROLLED_CANDIDATE`；官方 `NOT_MEASURED_DEFINITION_UNSPECIFIED` | 冻结官方类别、样本域和匹配规则，再做真实域及 S100P 复算 | `submission-package/evidence/day1/perception-score95/docs/competition-perception-score95.md`；`submission-package/evidence/day1/perception-score95/artifacts/perception_score95_20260914/receipt.json` | 30 帧/76 实例五类 P/R=1.0 只是受控候选，不能写成官方或真实域 95% |
| 18 | 定位精度 <=50 mm | 5 | `OFFLINE_CANDIDATE_PASS`；live official `NOT_MEASURED` | 把 tau=1.5 s 因果低通等价接入实时 TF 链，再跑一次 Gazebo 正式验收 | `F:/Project/TZcup/.workspace/worktrees/TZcup-day1-localization-50mm-recovery/docs/day1-localization-50mm-recovery.md`；`F:/Project/TZcup/.workspace/worktrees/TZcup-day1-localization-50mm-recovery/artifacts/day1_localization_50mm_recovery_20260914/recovery_receipt.json` | 离线候选不是 live 或 official PASS；原 run 的 max 126.327803 mm 失败记录仍保留 |
| 19 | 避障成功率 >=95% | 5 | `ROOT_CAUSE_FIXED`；官方率 `NOT_MEASURED` | 先做唯一一次 `FUNCTIONAL_PASS_1_OF_1_NOT_OFFICIAL_95`，再做独立重复矩阵 | `F:/Project/TZcup/.workspace/worktrees/TZcup-day1-video-report-sync/submission-package/evidence/day1/avoidance-recovery/day1-avoidance-recovery-root-cause.md`；`.../failure_receipt.json` | 单次功能通过仍不等于 95%，更不能把历史 run-19b/21/22 改成成功 |
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
5. 5-10 分钟 Demo 已以 `DELIVERED_MODULAR_DEMO` 交付；完整清扫任务、coverage、返航和同一连续任务视频仍未闭环。
6. 40 L 尘箱、维护步骤、3 天培训和日均故障率都没有物理或现场验收。

## 如果只剩最后一个资源窗口

1. 视频已冻结，不再为视频 4 分项启动 Gazebo 或重录。
2. 先让唯一 Gazebo owner 完成参数修复后的 coverage 运行，保留 path、report、dirt、brush、final state 和 MCAP；有完整证据才升级 coverage 或效率状态。
3. 若 coverage 不能产生完整任务数据，优先执行定位 live 因果低通重跑，因为定位会影响规划、避障和整任务可信度。
4. 定位恢复后再做一次动态避障 `1/1 functional`；不得把单次结果写成 95%，也不得并行启动第二套 Gazebo。

## 证据与回滚

- 报告合并：`codex/day1-evidence-integration@05481fc55e6ef89b9396070381797f760c6b72c4`
- 报告 PDF SHA-256：`08bfa8d2eb5ef60a8c8eeb1464ea3968df77f0a4aa5e3248aae2196b5e2e8592`
- 上一评分刷新：`codex/day1-score-refresh@c1c254ec27bcc9f746f8215533016d171ba1b2fa`，父提交为 `05481fc55e6ef89b9396070381797f760c6b72c4`
- 视频归档：`codex/day1-video-final@b663ca9871d75caf26b83c8f8fd40b0d906b1539`
- 视频成片 SHA-256：`631c2cfbf0e459bbcf74d7c3f9ddb6c1fc5a6519dbb6794bd4a6ea148d94e097`
- 定位候选提交：`a7841c9e024175901065d2c583f32a4a2d89f50f`，回滚点 `47e3cb3a7ecc01edd82aa23a3b54cbeaffc418bc`
- 避障修复提交：`99ee7f0d01544d65443b68f189edb28fc21d0459`，回滚点 `b74c702777b9974932f959800076071663255752`
- 感知候选提交：`3882dc34e756e1c361852027d49e55acac2b2a74`，回滚点 `906ea4f`
- Coverage run-11 配置提交：`d2335f3061146fedef8b9f731eb54760b0f82386`
- Coverage run-12 结果：`NOT_STARTED`
