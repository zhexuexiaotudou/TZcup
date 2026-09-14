# TZcup 比赛最小材料包

当前整体仍未达到完整提交条件，但已正式归档 5 分钟模块化综合展示视频；技术报告、板端证据与结果接入工具已成稿。47e3cb3单次封存路线、3882dc34受控感知候选、6b1a7dd非技术补充包，以及ff42430/c33c75a动态避障run-09部分功能证据均已入包。受控夹具30帧/76实例的raw与policy五类P/R=1.0只属`reviewable_95_candidate`，真实域、S100P板端和官方R01仍为`NOT_MEASURED_DEFINITION_UNSPECIFIED`。run-09接受目标并产生部分运动，但Nav2未成功、单次0/1，不能升级官方避障状态。5 分钟视频是不同运行的成功分项剪辑，不表示所有镜头属于同一连续任务，也不升级完整任务、MCAP或未测技术状态。

- `docs/技术方案报告.pdf`：23页中文报告，页数由最终构建读取。
- `metrics/验收矩阵.md` / `results.json`：唯一结果矩阵，明确区分measured_simulation、internal_frozen_regression、offline_replay、offline_raycast_mapping、synthetic_replay、design_calculation与not_measured。
- `metrics/video-score-status.json` / `演示视频评分状态.md`：视频项`DELIVERED_MODULAR_DEMO`、建议`2-3/4`、评委裁量上限`4/4`与不可升级边界。
- `metrics/板端证明.md`：独立于仿真的板端证据。
- `提交清单.md`：必交格式、三张短表、缺件和命名。
- `video/`：300.000 s、1920x1080、恒定30 fps、H.264/AAC、中文旁白和内嵌字幕的模块化综合演示；`video/video-index.json` 记录 `DELIVERED_MODULAR_DEMO`、建议分值区间和不可主张边界。
- `evidence/index.json`：原件来源与复制时间；相对路径内证据可随包搬运。
- `evidence/day1/raw-evidence-sources.json`：宽度、定位、感知原始运行根目录、精确路径和哈希定位表；不声称复制原始 MCAP/timeline。
- `evidence/day1/competition-perception-gpu-recovery.json`、`competition-perception-gpu-parity.json`：`4ed19d2`候选与`906ea4f`回执的compact metrics；仅复制JSON，不复制ONNX。
- `evidence/day1/perception-score95/`：`3882dc34`的受控候选文档、receipt、合成校准/独立测试以及30帧raw/policy复算JSON；`33/50/43→41/0/35`、`0/0/76→33/0/43`保留为旧运行时9帧presence mask历史诊断，不是全帧能力。
- `supplements/nontechnical-score-closure/`：14页非技术补充PDF、状态矩阵、产业化/维护/社会效益材料和原始SHA清单；除已交付的视频包外，其余仍按设计、计划或模板阶段披露，正式主张分值为0。
- `evidence/day1/tooling/full-mission-demo/`、`live-slam-replay-fail-closed/`：完整任务效率/Demo验收wrapper及live-SLAM输入未闭合的fail-closed回执，仅作工具和失败边界，不宣称得分。
- `evidence/day1/competition-mapping-offline-raycast.md`、`offline-raycast-mapping/`：`4ab39bb`离线射线映射的compact方法、面积、manifest和hash seal；已知22399.99m²但明确不是Gazebo live SLAM，不复制PGM。
- `evidence/day1/competition-mapping-day1-run-06.md`、`mapping-day1-run-06/`：`ab6252d`记录run-06在3600s超时且无最终地图，live SLAM面积与质量仍为`NOT_MEASURED`。
- `evidence/day1/demo-failure-retention.json`：两次预Gazebo失败和三次有界 full-task failed attempt的提交、回滚点与哈希台账；不声称包含可用MCAP。
- `evidence/day1/demo/`：6.333s主部分clip、2.867s后续失败attempt的compact MP4/PNG及attempt记录；两次均未启动任务，不是完整Demo。
- `evidence/day1/demo/heartbeat-hardened-verify-failure/attempt.json`：`339b549`最终验证失败及Demo路线永久停止的compact记录。
- `evidence/day1/competition-localization-rental-day1-final-run.md`、`localization-rental-final/`：`47e3cb3`最终定位run的compact doc与JSON；strict max≤50mm为`FAIL`，RMSE替代口径为`PASS_PARTIAL`，未复制129.9MiB MCAP。
- `evidence/day1/dynamic-avoidance-rental-day1/`：`c9a27ab`/`b9e1c71` Gazebo前失败，以及`ff42430`修复后`c33c75a` run-09的compact receipt/metrics；run-09 `goal_accepted=true`、`nav2_goal_succeeded=false`、单次0/1，官方≥95%仍为`NOT_MEASURED`。部分运动不是collision-free完整试验。
- `接入说明.md`：补入结果后刷新报告。不要把当前旧main源码当最新运行代码提交。
- `docs/report-baseline.json`、`evidence/day1/integration-manifest.json`：冻结旧23页PDF SHA，并记录本次增量来源提交、包内路径、SHA和声明边界，含新增`DELIVERED_MODULAR_DEMO`视频项。

所有文中引用历史报告仅作证据，历史报告原有PASS/工程门禁不覆盖本包唯一矩阵。未清理根工作区、云卡或任何任务证据。
