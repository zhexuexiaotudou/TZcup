# TZcup 比赛最小材料包

当前整体未达到完整提交条件。技术报告、板端证据与结果接入工具已成稿；新增47e3cb3单次封存路线、906ea4f/4ed19d2 GPU离线感知候选，以及ff42430/c33c75a动态避障run-09部分功能证据。run-09接受目标并产生部分运动，但Nav2未成功、单次0/1且官方≥95%仍为`NOT_MEASURED`。Demo路线已在`339b549`永久停止，已有真实Gazebo GUI partial clip但没有有效5-10分钟完整任务Demo或可用MCAP，且没有伪造视频或未测PASS。

- `docs/技术方案报告.pdf`：23页中文报告，页数由最终构建读取。
- `metrics/验收矩阵.md` / `results.json`：唯一结果矩阵，明确区分measured_simulation、internal_frozen_regression、offline_replay、offline_raycast_mapping、synthetic_replay、design_calculation与not_measured。
- `metrics/板端证明.md`：独立于仿真的板端证据。
- `提交清单.md`：必交格式、三张短表、缺件和命名。
- `evidence/index.json`：原件来源与复制时间；相对路径内证据可随包搬运。
- `evidence/day1/raw-evidence-sources.json`：宽度、定位、感知原始运行根目录、精确路径和哈希定位表；不声称复制原始 MCAP/timeline。
- `evidence/day1/competition-perception-gpu-recovery.json`、`competition-perception-gpu-parity.json`：`4ed19d2`候选与`906ea4f`回执的compact metrics；仅复制JSON，不复制ONNX。
- `evidence/day1/competition-mapping-offline-raycast.md`、`offline-raycast-mapping/`：`4ab39bb`离线射线映射的compact方法、面积、manifest和hash seal；已知22399.99m²但明确不是Gazebo live SLAM，不复制PGM。
- `evidence/day1/competition-mapping-day1-run-06.md`、`mapping-day1-run-06/`：`ab6252d`记录run-06在3600s超时且无最终地图，live SLAM面积与质量仍为`NOT_MEASURED`。
- `evidence/day1/demo-failure-retention.json`：两次预Gazebo失败和三次有界 full-task failed attempt的提交、回滚点与哈希台账；不声称包含可用MCAP。
- `evidence/day1/demo/`：6.333s主部分clip、2.867s后续失败attempt的compact MP4/PNG及attempt记录；两次均未启动任务，不是完整Demo。
- `evidence/day1/demo/heartbeat-hardened-verify-failure/attempt.json`：`339b549`最终验证失败及Demo路线永久停止的compact记录。
- `evidence/day1/competition-localization-rental-day1-final-run.md`、`localization-rental-final/`：`47e3cb3`最终定位run的compact doc与JSON；strict max≤50mm为`FAIL`，RMSE替代口径为`PASS_PARTIAL`，未复制129.9MiB MCAP。
- `evidence/day1/dynamic-avoidance-rental-day1/`：`c9a27ab`/`b9e1c71` Gazebo前失败，以及`ff42430`修复后`c33c75a` run-09的compact receipt/metrics；run-09 `goal_accepted=true`、`nav2_goal_succeeded=false`、单次0/1，官方≥95%仍为`NOT_MEASURED`。部分运动不是collision-free完整试验。
- `接入说明.md`：补入结果后刷新报告。不要把当前旧main源码当最新运行代码提交。

所有文中引用历史报告仅作证据，历史报告原有PASS/工程门禁不覆盖本包唯一矩阵。未清理根工作区、云卡或任何任务证据。
