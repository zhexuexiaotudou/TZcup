# 定位稳定器实时接线与有界重跑

状态：`READY_FOR_ONE_BOUNDED_LIVE_REPLAY`

本版本没有启动 Gazebo。当前唯一 Gazebo 先由 coverage run-12 使用，之后由
Copernicus 执行 avoidance。下面的 harness 通过正式 `flock` 获取单 Gazebo
所有权；两个前序任务未释放锁或进程时，它会在启动前返回 `75`。

## 实时运行链路

默认 launch 行为不变：

```text
global_ekf -> /tf map -> odom
```

启用候选时：

```text
global_ekf -> /localization/raw_map_odom
map_odom_stabilizer -> /tf map -> odom
```

接线只改 global EKF 的 TF 输出 topic，不改变：

- EKF 输入；
- AMCL、GNSS、轮速和 IMU 的融合配置；
- `/localization/fused_odom`；
- Nav2、控制器或安全门。

稳定器只订阅 `/localization/raw_map_odom`。每次更新只接受当前消息及其
已有滤波状态，不调用 TF lookup，不读取未来样本，不订阅任何 ground truth
或 Gazebo world/model topic。

## 因果算法

```text
alpha = 1 - exp(-dt / tau)
output = previous + alpha * (current - previous)
```

默认参数：

| 参数 | 默认值 | 允许边界 |
|---|---:|---:|
| `tau_sec` | 1.5 s | 0.75--3.0 s |
| `max_filter_dt_sec` | 0.1 s | 0.02--0.2 s |
| `max_gap_sec` | 0.5 s | 0.1--1.0 s |

同时间戳重复值由幂等更新吸收；同时间戳冲突、时间倒退或输入 gap 超限会
latch 为 `BLOCKED`，停止继续发布，不能靠后续猜测恢复。

## 所需排他窗口

- `ROS_DOMAIN_ID=83`
- `GZ_PARTITION=tzcup_localization_stabilizer_20260915_01`
- 单次窗口：约 `11--15 min`
  - 启动与准备：约 40 s + 420 s
  - 动态验收：120 s
  - 收尾、MCAP 封存、复算和资源释放：约 90 s
- 必须等 coverage run-12 和 Copernicus avoidance 都释放
  `/tmp/tzcup_formal_gazebo.lock` 后再执行。

## 一键执行

先构建一次隔离 overlay，使新节点和 collector 出现在 `ros2 pkg executables`
中；harness 会在获取 Gazebo 锁之前做这项检查，缺包时直接失败，不会启动
Gazebo。

```bash
export SOURCE=/workspace/tzcup-competition-sim-only-20260912/source/TZcup-<revision>
export RUNTIME=/workspace/tzcup-competition-sim-only-20260912/runtime/runtime-ws-<runtime-revision>
export OUTPUT=/workspace/tzcup-competition-sim-only-20260912/evidence/day1-localization-stabilizer-20260915-01/run-01
export EPISODE=/workspace/tzcup-competition-sim-only-20260912/evidence/motion-cleaning-continuous-fixture-01/episode
export MAP_SOURCE=/workspace/tzcup-competition-sim-only-20260912/evidence/competition-integrated-20260913-01
export DRIVER=/workspace/tzcup-competition-sim-only-20260912/source/TZcup-<revision>/scripts/run_competition_localization_driver.py
export CANDIDATE_REVISION="$(git -C "$SOURCE" rev-parse HEAD)"
export COMPETITION_RUNTIME_OVERLAY=/workspace/tzcup-competition-sim-only-20260912/overlays/day1-localization-stabilizer
export LOCALIZATION_COLLECTOR_OVERLAY=/workspace/tzcup-competition-sim-only-20260912/overlays/day1-localization-stabilizer
export PROBE_DOMAIN=83
export PROBE_PARTITION=tzcup_localization_stabilizer_20260915_01

bash "$SOURCE/scripts/run_day1_localization_stabilizer_live.sh"
```

## 验收门

harness 只有在以下条件全部满足时写
`live_candidate_receipt.json: LIVE_CANDIDATE_PASS`：

1. effective parameters 精确显示
   `/map_odom_stabilizer` 的 `tau_sec=1.5`、
   `max_filter_dt_sec=0.1`、`max_gap_sec=0.5`；
2. 标准 `/tf` 的 `map->odom` 只有一个 publisher，归因到
   `/map_odom_stabilizer`；
3. `/localization/raw_map_odom` 由 `/global_ekf` 发布、由
   `/map_odom_stabilizer` 消费，且消息数不少于 100；
4. stabilizer 状态为 `READY`，`rejected_updates=0`，
   `accepted_updates>=100`，明确声明不使用 GT 和未来数据；
5. 同 session focus scorer 的 RMSE、P95、max 全部 `<=50 mm`，
   且配对样本不少于 100；
6. driver 返回 0；
7. Gazebo、ROS runtime 无残留，正式锁重新可用。

任何 gate 失败都会保留现场证据并生成 `FAIL` receipt；不删除失败尾部。

## 回滚和资源释放

- 行为回滚：launch 默认 `map_odom_stabilizer:=false`，恢复原
  `global_ekf -> /tf` 链路。
- 代码回滚点：`a7841c9e024175901065d2c583f32a4a2d89f50f`；
  原始定位证据基线：`47e3cb3a7ecc01edd82aa23a3b54cbeaffc418bc`。
- 清理统一使用 exact `GZ_PARTITION` 和已登记的 setsid PID groups。
- 清理后必须写 `resource_release.json`；残留任一进程时 receipt 固定为
  `FAIL`，不能手工改状态。
