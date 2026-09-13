# TZcup 仅仿真路线冲刺执行评估

日期：2026-09-12
结论：**当前仍不能认定为完整、有效的参赛仿真作品（NO-GO）**。本轮已经把环境、源码、板端模块和仿真入口固定下来，并取得可复核的真实运行证据；剩余缺口集中在正式板端感知与首图建图闭环，尚未形成 Golden Mission、三次重复运行或性能指标闭环。

## 1. 固定对象

- PC/云端源码：`8a0a8b46924dd65da6d1dbe58458888d152dc1bc`，tree `ece06f80c4e92b1fb17195d6c4532e1624f5d71e`。
- S100P 证据源码：`6731784c03261652bce70a1161fee921c9738291`，tree `6bad835f4316a5976d2d2c6c0ce7e852a234f0cb`。与 PC 最终提交之间的后续差异只涉及 PC 组合运行器以 `bash` 调用无执行位脚本及其测试，不改变板端执行模块。
- 统一云端运行时：20 个包、1803 个安装文件、0 个软链接，closure id `36972d9d834c44ea9108742ed6d1ecbf3052f616117c8f67bdbf5463d6e0b2f8`；NVIDIA EGL 绑定通过。
- 板端：S100P/aarch64，存在 `/dev/bpu_core0`，使用 `/opt/tzcup/s100p/releases/competition-sim-only-6731784-20260912` 隔离 release；最终报告不记录设备序列号。

## 2. 门禁结果

| 门禁 | 状态 | 本轮证据及边界 |
|---|---|---|
| T0 源码、板端、云端身份冻结 | PASS | 板端与云端探针相差约 15 秒；源码、tree、overlay 和依赖哈希已记录。 |
| 云端完整编译 | PASS | 8a0a8b4 的双 worker 构建在 PRoot 中崩溃，失败日志保留；单 worker 重建成功，20 包完成，rc=0。 |
| 感知资产启动预检 | PASS | DOSOD、EdgeSAM 与冻结词表均存在、可加载、哈希固定；这只证明 PC 资产可启动。 |
| 正式总验收预检 | PASS | 源码、运行时 closure、EGL、命令、输入和输出隔离检查全部通过；预检不等于任务运行通过。 |
| BOARD-01 感知 | BLOCKED | 仅有 `NON_FORMAL_ABI_DEVELOPMENT` HBM；产品适配器拒绝非 `nash-m` 目标。无语义精度、产品图活性和 BPU 时延样本。 |
| BOARD-02 建图定位 | PASS（synthetic replay） | slam_toolbox 输出有效 OccupancyGrid；robot_localization 有输出和运动响应。不能替代仿真整链闭环。 |
| BOARD-03 规划决策 | PASS（simulation replay） | Q-learning train/validation/test 成功率均为 1.0，无 split overlap、无真值访问；覆盖规划报告已生成。 |
| BOARD-04 控制 | PASS（replay） | 61 个输出样本，限幅、急停归零、恢复和输出变化检查通过。 |
| PC 首图建图 | BLOCKED | 真实 Gazebo 运行 45 分钟，推进到 checkpoint 144，但没有封图或生成生命周期 manifest。GNSS/odom 差异从 checkpoint 76 的 2.219 m 增至 checkpoint 140 的 5.389 m，超过 2.0 m 门槛；checkpoint 36、64 发生 Nav2 `progress_timeout` 后恢复。外层超时终止，rc=143。`444b6cf` 已把后续门禁改为最大 100 ms 的时间配对，`7195d63` 已提供只读定位 MCAP 采集/终结能力；二者均还没有这次冻结版本的新鲜真实运行结果。 |
| 保存地图清扫、GroundDirt、动态避障 | NOT RUN | 依赖首图封图，未跨越前置门禁，不能把组件历史证据拼成当前整链通过。 |
| Golden Mission 与三次重复 | NOT RUN | 没有一次完整闭环，因此没有成功率、端到端时延或三次重复性结论。 |
| 3500 m2/h | BLOCKED（未实测） | GroundDirt 的边刷圆扫与中央滚刷确实构成物理清扫链，不能只取 0.620 m 中央滚刷宽度。按 1.32 m 有效扫宽、0.45 m/s 和 0.75 覆盖效率，`1.32*0.45*3600*0.75=1603.8 m2/h`；1.0 m/s 候选为 `3564 m2/h` 理论值。因此 3500 不是物理不可能，但尚未通过 GroundDirt 前后状态、有效面积并集/遗漏率、安全和动力学实测；题面数值来源仍待复核。 |
| 视频 | DEFERRED | 按用户要求本轮不录制；没有用动画或视频代替原始运行证据。 |
| GitHub/PR/CI/合并 | DEFERRED | 按用户要求暂缓上传。分支、提交、工作树和证据全部保留。 |

## 3. 首图失败的只读根因排序

第一嫌疑是里程计几何不一致。A300 接触轮半径在 `a300_platform.xacro` 中为 0.1651 m，而 `A300DrivetrainPlantCore.hh` 的 control radius 为 0.1625 m，`A300DrivetrainPlantSystem.cc` 又用 control radius 积分 `/odom/unfiltered`。二者相差约 1.6%，在 200 m × 100 m 场地的长距离扫掠中可累积到数米，量级与本轮曲线一致。轮距的近似误差可能在转弯中继续放大。

第二个缺口是比较口径。`map_lifecycle_manager.py` 比较 local EKF `/odom` 与 `navsat_transform` 的 `/odometry/gps`，当前取两个 latest 样本计算距离，没有记录时间偏差、原始坐标或协方差。因此必须先排除异步样本造成的假差异。mapping 模式刻意关闭 global EKF，由 slam_toolbox 持有 `map -> odom`；本轮差异不是“SLAM 位姿与 GNSS”的直接比较。

初值或固定场景偏移的可能性较低：运行早期一致，差异在 checkpoint 76 后持续增长。Nav2 两次 `progress_timeout` 可恢复，单独保留为导航性能事件，目前没有证据表明它造成单调定位漂移。

## 4. 下一次最小闭环

1. 保持同一源码、场景和阈值，使用 `444b6cf` 的最大 100 ms 时间配对门；通过 `7195d63` 的只读 MCAP 诊断记录 `/odom`、`/odom/unfiltered`、`/odometry/gps`、`/gnss/fix`、`/formal_mapping/lifecycle_status`，并仅在已有图暴露时附加 `/ground_truth/odom`。终结器必须验证 MCAP、话题、绑定、时间窗口和哈希，再记录 `delta_t`、坐标、协方差、`e_raw` 与 `e_ekf`。这两项能力尚无新鲜运行 receipt，不能回填为通过。
2. 若 `e_raw` 随里程按约 1.6% 增长且 GNSS 与 Pose_V 一致，将里程计积分半径与 0.1651 m 物理接触轮统一；控制命令半径是否继续保留 0.1625 m 单独验证。不得放宽 2 m 门槛来掩盖误差。
3. 若仅 `e_ekf` 增长，检查 local EKF 与 IMU；若时间配对后不再增长，修复生命周期门禁为时间配对、skew 超限 fail-closed，并记录明细。
4. 首图封图后，才依次运行保存地图清扫、实际 GroundDirt 清除、动态避障/急停、板端正式感知，以及同一冻结配置的三次 Golden Mission。
5. 3500 m2/h 必须通过 GroundDirt 前后状态、有效面积并集、实测遗漏率、可验证安全速度和转弯/避障开销共同闭环。1.0 m/s 的 3564 m2/h 只是基于 1.32 m 有效扫宽和 0.75 覆盖效率的理论候选，未取得 GroundDirt、安全或动力学通过前不得宣称达标。

## 5. 验证与交付状态

- 本地完整验证：`2469 passed, 55 skipped, 26 subtests passed`，耗时 601.04 s；development workflow fast validation 通过；所有 shell 脚本 `bash -n` 通过；`git diff --check` 通过。
- 本地分支：`codex/simulation-only-completion`；HEAD `8a0a8b4`；工作树干净。最近一项修改的回退点为 `6731784`，旧运行时与全部失败日志仍保留。
- neat-freak 审核已完成：README、项目规则、仿真执行说明与当前证据边界一致，没有需要单独提交的文档漂移；未获授权写入平台 memory。
- 清理未执行。任务工作树、分支、板端 release、云端运行时、失败日志和验收证据继续作为复核与回退依据保留。

## 6. 主要证据

- `../competition-sim-only-20260912-final-board/evidence/board-module-summary.json`
- `../competition-sim-only-20260912-final-probes/board_probe.json`
- `../competition-sim-only-20260912-final-probes/remote_probe.json`
- `../competition-sim-only-20260912-final-t0/runtime_inventory.json`
- `../competition-sim-only-20260912-final-remote-extracted/evidence/build-8a0a8b4-02.log`
- `../competition-sim-only-20260912-final-remote-extracted/evidence/perception-preflight-8a0a8b4-01.json`
- `../competition-sim-only-20260912-final-remote-extracted/evidence/formal-preflight-8a0a8b4-01.json`
- `../competition-sim-only-20260912-final-remote-extracted/evidence/composite-8a0a8b4-02.log`
- `final_runtime_closure_manifest.json`
- `integrated_build_manifest.json`
- `../competition-sim-only-20260912-final-remote.tar.gz`
