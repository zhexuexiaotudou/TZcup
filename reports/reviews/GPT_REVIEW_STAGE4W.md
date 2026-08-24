# TZcup Stage4W 独立复核说明

## 结论

`READY_FOR_GPT_REVIEW_STAGE4W=true`，`READY_FOR_STAGE5A=true`。

Stage4W 的定位、可达入口、统一清扫几何、完整 Coverage、动态障碍、过滤器、急停和 MCAP 回放硬门均已通过。竞赛效率门仍为 false：`0.65 × 0.45 × 3600 = 1053 m²/h < 3500 m²/h`。本结论不包含垃圾感知训练、J6 量化、实板部署或原生 GUI 验收。

## 建议优先复核的材料

- `artifacts/stage4w_20260717_review/stage4w_summary.json`：最终门禁与 READY 标志。
- `artifacts/stage4w_20260717_review/stage4w_localization_report.json`：正式 hybrid 10-seed。
- `artifacts/stage4w_20260717_review/stage4w_static_matrix_report.json`：正式静态 5-seed。
- `artifacts/stage4w_20260717_review/stage4w_dynamic_report.json`：动态、过滤器、急停和回放综合门。
- `artifacts/stage4w_20260717_review/MANIFEST.json`：紧凑证据的大小与 SHA-256。
- `docs/stage4w-full-coverage.md`：实现语义、几何口径和边界。

## 正式结果

| 门禁 | 结果 | 关键证据 |
|---|---:|---|
| Hybrid 定位 | 10/10 | XY RMSE P50/P95/max `0.02825/0.03726/0.03778 m` |
| GT 控制隔离 | 通过 | 控制违规 0；GT 仅用于评测 |
| 静态完整 Coverage | 5/5 | 每 seed `17/17`，transit 5/5 |
| 静态经验覆盖率 | 通过 | `92.93%–94.53%`，阈值 90% |
| 覆盖期定位 | 通过 | RMSE `0.02930–0.04620 m`，每 seed ≤0.05 m |
| 动态障碍 | 20/20 | 有效交互 20、碰撞 0 |
| 动态完整任务 | 通过 | `17/17`，覆盖率 93.53%，RMSE 0.03014 m |
| keepout / speed filter | 通过 | keepout 违规 0；限速区均速 0.288 m/s |
| 急停 | 30/30 | P50/P95/max `0.151/0.188/0.338 s` |
| 上游失联归零 | 通过 | 1.694 s 达到连续 5 帧稳定零输出 |
| MCAP 回放 | 通过 | 静态 5/5 + 动态完整回放 |
| 竞赛理论效率 | 失败 | `1053 m²/h < 3500 m²/h` |

静态逐 seed：

| seed | 组件 | 覆盖率 | 覆盖期 RMSE (m) | 回放 |
|---:|---:|---:|---:|---:|
| 0 | 17/17 | 94.20% | 0.03572 | 通过 |
| 1 | 17/17 | 93.40% | 0.04620 | 通过 |
| 2 | 17/17 | 92.93% | 0.03173 | 通过 |
| 3 | 17/17 | 94.53% | 0.03340 | 通过 |
| 4 | 17/17 | 94.00% | 0.02930 | 通过 |

## 关键修复与可审计语义

1. GNSS covariance 包含基础噪声、固定偏差和累计随机游走；hybrid fuser 对 refined/GNSS 方差与年龄设边界，并把最新全局锚点传播到当前局部里程计位姿。
2. mission geometry 统一编译 outer、headland、keepout、exclusion、固定障碍、footprint 与安全裕量。当前合法几何生成 9 swath + 8 turn = 17 组件。
3. Stage4V 的固定 23 组件来自无 headland、无 inner cutout 的旧几何，Stage4W 明确标记为 `not_applicable`，门禁是“统一几何生成的全部组件成功”，没有伪造 23/23。
4. staging 同时评估正/反路线；全局 costmap 必须实际覆盖候选点后才调用 ComputePath。transit 使用显式 approach yaw，首条 swath 前通过 brush-off 稠密 entry 对正。
5. NavigateToPose、ComputePathToPose 和 FollowPath 使用各自动作错误码；FollowPath 104 正确解释为 `PATIENCE_EXCEEDED`。运行时为 `PoseProgressChecker`、`failure_tolerance=5.0 s`。
6. 动态障碍使用持久 ROS–Gazebo SetEntityPose 服务桥；同一组件相邻注入至少间隔 0.5 m。局部/全局 obstacle layer 均启用 `inf_is_valid=true`，确保障碍移走后清除旧标记。
7. stale-command 门等待连续 5 帧零输出，并保留受控减速尾部样本，不再使用固定 sleep 后的瞬时判断。

## 复现命令

```powershell
py scripts/ci_fast.py
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage4w_static_matrix_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage4w_dynamic_docker.ps1
py scripts/stage4w_finalize.py --localization artifacts/stage4w_localization_regression --static artifacts/stage4w_static_formal --dynamic artifacts/stage4w_dynamic_formal --output artifacts/stage4w_20260717_review
```

原始 MCAP 很大且不进入 Git 历史；紧凑目录包含每次静态 summary、rosbag info、动态 rosbag info 和最终报告，可用 `MANIFEST.json` 校验。

## 请 GPT 重点判断

1. 是否认可“当前统一几何生成的全部 17 个组件成功”替代已经失效的历史固定 23 组件字面门。
2. 是否认可 Stage4V-compatible 的每 seed XY RMSE ≤0.05 m 为覆盖期定位硬门，而逐点 P95/max 继续作为诊断值。
3. 在竞赛效率仍失败、GUI/实板未验收的前提下，是否允许进入 Stage5A 的感知方案设计；如允许，应优先垃圾检测真值闭环、J6 工具链预检还是大场景效率重构。
