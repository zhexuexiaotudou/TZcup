# AUTO-02 完整导航回归与配置冻结

## 结论

AUTO-01 选出的 opt-in `G2-C3 / V5_retracted` 已通过 Docker 中 ROS 2 Jazzy + Gazebo Harmonic 的完整机器回归，冻结为 `starter_ws/src/sanitation_navigation/config/autonomous_navigation_profile_v1.yaml`。本阶段只证明该仿真配置在约定矩阵内可重复工作；production 默认 profile、真人审计、真实车辆、真实域、J6 和最终竞赛状态均未改变。

紧凑证据目录为 `artifacts/autonomous_auto02_20260729_evidence/`。大型原始 MCAP、日志和失败尝试保留在本地 Git 忽略目录 `artifacts/autonomous_auto02_raw_20260729/`，不生成中间复核 ZIP。

## 验收矩阵

| 门 | 实际结果 | 判定 |
| --- | --- | --- |
| 静态五 seed | `5/5`，每次 `17/17` | PASS |
| 经验覆盖率 | `0.92733–0.94467`，阈值 `>=0.90` | PASS |
| 计划覆盖率 | 五次均 `0.986`，阈值 `>=0.95` | PASS |
| 定位 XY RMSE | `0.03153–0.04050 m`，阈值 `<=0.05 m` | PASS |
| 静态安全 | 碰撞/keepout/刷盘状态违规均为 `0` | PASS |
| 动态障碍 | `20/20`，碰撞 `0`，全部恢复 | PASS |
| 动态分离距离 | `0.60439 m >= 0.12 m` 硬阈值 | PASS |
| keepout | 违规采样 `0` | PASS |
| 限速区 | `0.28613 m/s <= 0.3135 m/s` | PASS |
| 急停 | `30/30`，P95/max `0.13994/0.14063 s` | PASS |
| 停止后输出 | 每个 trial 持续为零，刷盘最终关闭 | PASS |
| 冷启动 | `5/5`，`24/24/24/24/25 s` | PASS |
| MCAP | 静态 `15/15`、动态 `16/16` 必需主题 | PASS |
| 回放重算 | 任务终态可回放，覆盖率相对误差均为 `0` | PASS |

限速门使用明确公式：

```text
0.45 m/s × 63% + 0.03 m/s = 0.3135 m/s
```

## 实现变化

- Coverage probe 新增 `/coverage/evaluation_sample`，把内部评测网格的真实采样流写入 bag，使经验覆盖率能够从 MCAP 独立重算。
- `auto02_replay_audit.py` 直接读取 MCAP 元数据和消息，验证场景必需主题、Coverage 终态和实际 replay 观察；经验覆盖率从评测采样流重算，定位 RMSE 限定在同一个 evaluation 时间窗重算，两者相对误差均不得超过 `1%`。
- 静态脚本拆出可复用 finalizer，并新增五 seed 聚合中的计划覆盖率门。
- 动态探针显式保存障碍最小分离硬阈值、每次任务恢复、限速计算参数和每次急停后的零输出状态。
- 冷启动门扩展为八个 lifecycle 节点、完整 TF 与 pointcloud self-filter 参数检查。
- Windows Docker 驱动器支持已完成 seed 的断点复用，并拒绝超出 Fast DDS `0–232` 范围的 ROS domain。
- `finalize_auto02.py` 仅在综合 acceptance 通过后冻结 profile、生成紧凑证据和逐文件 SHA-256 manifest，并把自主状态推进到 AUTO-03。

## 失败尝试与修正

1. 首次 seed0 回放审计错误地要求静态场景中未激活的 `/emergency_stop`。原始 MCAP 的任务和覆盖率重算均通过，因此修正为按场景定义必需主题，再复用同一个未修改 bag。
2. seed3 首次被分配 `ROS_DOMAIN_ID=233`，超出 Fast DDS 有效范围。该失败目录原样保留；驱动器加入范围保护并改用 `180–184` 后断点续跑。

这两个问题都记录在 `attempt_ledger.json`，没有覆盖失败事实，也没有把不完整运行计为通过。

## 复现

在 Windows PowerShell、Docker Desktop 可用且已有 `tzcup/sanitation-jazzy:stage5b` 镜像时：

```powershell
powershell -ExecutionPolicy Bypass -File scripts/run_auto02_regression_docker.ps1 `
  -OutputName autonomous_auto02_raw_YYYYMMDD
py -3 scripts/finalize_auto02.py `
  --raw artifacts/autonomous_auto02_raw_YYYYMMDD `
  --output artifacts/autonomous_auto02_YYYYMMDD_evidence
py -3 scripts/verify_evidence_manifest.py `
  artifacts/autonomous_auto02_YYYYMMDD_evidence
```

不要把本机原始 bag/log 或中间 ZIP 提交到仓库；最终发布压缩包只允许由 AUTO-16 生成。
