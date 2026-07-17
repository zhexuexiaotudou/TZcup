# Stage4W 可达清扫域、完整覆盖与动态交互闭环

## 目标与边界

Stage4W 以修复 Stage4V 已定位到的完整 Coverage 执行链为主；正式 10-seed 回归暴露出 GNSS 协方差与全局锚点未随局部里程计传播的问题，因此只在既有 hybrid 架构内做了有界协方差和锚点传播修复，没有更换定位架构。垃圾感知、J6 量化和实板部署均未启动。

## 统一任务几何

`mission_geometry.py` 将 mission 配置编译为唯一的规划与评测几何：外边界、keepout、显式 exclusion、经过 `world_to_map_translation` 变换的固定障碍、机器人 footprint、安全裕量、作业宽度和 headland。OpenNav 的第一个 polygon 是外边界，后续 polygon 是膨胀后的 inner cutout；计划覆盖率与经验覆盖率使用同一份 headland 后的 cleanable geometry。

当前结构化场景的 `trash_bin_obstacle` 在 world→map 变换后位于任务外，报告为 ignored；`structured_waste_bin` 位于任务内并被膨胀为 cutout。配置 headland 为 1.00 m，编译得到的最小值约为 0.963 m。任何配置小于编译最小值都会在车辆移动前 fail-closed。

启用真实 cutout 与 headland 后，OpenNav 生成 9 条 swath 和 8 条 turn，即 17 个应执行组件。Stage4V 的 23 个组件来自“无 headland、未传 inner cutout”的旧几何，不能继续作为新任务的固定数量。Stage4W 以“当前统一几何生成的全部组件终态 SUCCEEDED”为门禁，并同时报告历史 23 组件口径不再适用，禁止伪造 23/23。

## 可达入口与 action 诊断

执行器同时构造正向和反向路线，按当前 fused pose 为两端 staging 候选调用 `ComputePathToPose`，等待并采样全局 costmap、keepout mask 与 speed mask，剔除无路径、lethal cost（`>=99`）、keepout 非零或 clearance 不足的候选，再选择最短可行路径。路线反转会同步反转 component 顺序、每个 component 的点序和刷盘调度；诊断同时记录 costmap 是否收到、边界和目标栅格值，避免把“未收到图”和“目标越界”混为一谈。

入口使用两阶段动作：先以明确的 approach yaw 执行 `NavigateToPose` 到 pre-staging，再用稠密 brush-off `FollowPath` 沿首条 swath 延长线完成对正和 0.20 m lead-in，之后才开启刷盘。单点或退化点不得隐式生成 yaw=0。

每个候选和 action 都记录 fused pose、仅评测用 ground truth、目标 x/y/yaw、首条 swath 航向、路线方向、预规划长度与端点、footprint clearance、全局 costmap/keepout/speed 栅格值、1 Hz 有界 feedback、distance remaining、恢复数、错误码与符号名、checker ID、终端位姿、位置/航向误差以及 timeout/cancel 响应。Nav2 使用有界 15 s 的 `PoseProgressChecker`，同时认可 0.20 m 平移或 0.20 rad 有效转向；Stage4V 的 Simple checker error 105 证据保留用于对照。

## 状态、动态障碍与证据

Coverage 发布 `/coverage/state`、`/coverage/component_state`、`/coverage/current_path` 和 `/coverage/diagnostics`。动态 probe 只有在 `EXECUTING_SWATH` 且剩余路径不少于 0.8 m 时，才把 `dynamic_pedestrian_box` 沿当前路径前方的横穿轨迹移动；同一组件的相邻注入点必须至少相距 0.5 m。服务通过持久 `ros_gz_interfaces/srv/SetEntityPose` 桥调用，使用参数化 world/model/timeout，并先执行 park→test→park 预检，避免每个轨迹点重新启动 Gazebo CLI。

Nav2 的局部和全局 obstacle layer 显式启用 `inf_is_valid=true`，使障碍移走后的无限量程激光束能够清除旧标记；controller `failure_tolerance=5.0 s`，允许有界横穿暂停但仍早于 15 s progress checker。动态探针会记录每个 set-pose 调用、LiDAR 最小距离、恢复位移、恢复时间、组件和相邻注入间距。

一次动态试验只有同时满足 set-pose 全成功、目标进入路径走廊、LiDAR 观测到交互、零碰撞和任务恢复推进才有效。服务超时或模型错误立即 fail-closed，不计入试验数。

静态门先运行 5 个 seed；只有 5/5 通过后，动态/过滤器/30 次急停/完整 MCAP 回放门才允许启动。覆盖期定位回归沿用 Stage4V 正式口径：每个 seed 的时间戳同步 XY RMSE 必须不超过 0.05 m；任务内逐点 P95/max 保留为诊断，不能偷换成一个从未在 Stage4V 使用的新硬门。原始 MCAP 与筛查输出保留在本地，仓库只提交可审计的紧凑复核证据。

## 效率边界

竞赛理论效率口径保持不变：`0.65 × 0.45 × 3600 = 1053 m²/h`，低于 `3500 m²/h`，因此 `competition_efficiency_pass=false`。Stage4W 只报告实际执行效率、转弯损失和重复率，不通过修改统计口径宣称效率达标。

## 正式结果

正式 hybrid 定位回归 10/10 通过，XY RMSE P50/P95/max 为 `0.02825/0.03726/0.03778 m`。静态覆盖 5/5 通过，每次均为 17/17，经验覆盖率 `92.93%–94.53%`、覆盖期 RMSE `0.02930–0.04620 m`。动态任务为 20/20 有效交互、碰撞 0，完整任务 17/17、覆盖率 93.53%、覆盖期 RMSE 0.03014 m；30 次急停 P95 0.188 s，stale command 在 1.694 s 达到连续 5 帧稳定零输出。

全部 Stage4W 硬门已通过，因此 `READY_FOR_GPT_REVIEW_STAGE4W=true`、`READY_FOR_STAGE5A=true`。紧凑证据写入 `GPT_REVIEW_STAGE4W.md` 和 `artifacts/stage4w_20260717_review/`，不复制原始 MCAP；原始运行证据保留在本机 artifact 目录直到用户确认。
