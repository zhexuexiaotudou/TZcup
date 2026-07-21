# Stage5BR6W GPT 独立复核入口

## 结论先行

Stage5BR6W 已建立不污染 Stage5BR6-A 的 engineering-only waiver，完成 V4 工程 profile、工程 policy、candidate footprint、运行时 footprint 一致性审计和 observation planner 加固。真实 Stage4W candidate-footprint seed 0 在 `no_reachable_clean_route` 失败，故按前置硬门停止，没有执行剩余静态 seed、dynamic/estop 或多世界 Oracle。

```text
first_blocking_layer=stage5br6w_phase4_candidate_footprint_static_seed0_no_reachable_clean_route
READY_FOR_STAGE5BR6W_ORACLE_ENGINEERING=false
READY_FOR_STAGE5BR7_ENGINEERING=false
AWAITING_HUMAN_REVIEW=true
MANUAL_AUDIT_PASS=false
READY_FOR_STAGE5BR6_ORACLE=false
READY_FOR_STAGE5BR7=false
READY_FOR_GPT_REVIEW_STAGE5B=false
READY_FOR_STAGE5C=false
```

## 优先复核

1. `ENGINEERING_WAIVER_STAGE5BR6W.md` 与 `config/engineering_waiver_stage5br6w.json`：正式状态和工程状态严格分离。
2. `artifacts/stage5br6w_20260721_review/stage5br6w_status.json`：机器可读总状态与停止边界。
3. `runtime_footprint_audit.json`：local/global costmap、Collision Monitor、Coverage 使用同一候选 footprint；production 默认未改变。
4. `stage4w_candidate_footprint_regression.json` 与 `seed0_coverage_report.json`：真实 Phase 4 首次失败。
5. `planner_hardening_report.json` 与 `observation_pose_planner.py`：整多边形、SE(3)、CameraInfo、costmap cost 和 fail-closed 行为。
6. `artifact_manifest.json`：紧凑证据的大小和 SHA-256。

## Phase 4 结果

candidate footprint 由 V4 AABB、production footprint 与 0.03 m 裕量推导，半径为 `0.856825 m`。运行时审计通过，但更大 footprint 把统一几何下的 cleanable area 压缩为 `6.89 m²`，9 条 swath 全部与膨胀 exclusion 相交。正向 staging 的 Nav2 path 可用但 staging 位于 operation polygon 外；反向 staging cost 为 99 且 `NO_VALID_PATH`。因此完整组件为 0、经验覆盖率为 0。定位 RMSE `0.04333 m`、碰撞 0、keepout 0 和刷盘最终关闭都不能替代完整任务门。

PR #26 合并后独立复验再次得到相同 candidate footprint、`6.89 m²`、9 条 swath 冲突和 `no_reachable_clean_route`；该次定位 RMSE 为 `0.05342 m > 0.05 m`。首次 post-merge 尝试还出现 Nav2 参数服务慢启动超时。因此工程门不仅没有通过，还需在下一轮同时处理可达几何和运行稳定性；post-merge 原始日志未纳入本紧凑证据目录。

本次 MCAP 已记录且元数据可读，但失败发生在 Coverage state 进入 bag 前，故 replay=false。按协议不补跑 dynamic 20、estop 30，也不启动 Oracle。

## 固定边界

V4 不是 competition camera，工程 policy 不是 human-validated policy，本轮没有人工答案、模型训练、真实域评测、J6 或竞赛效率证据。Reviewer A/B 原包与 sealed truth 均未修改；未来仍可原样恢复正式真人门。
