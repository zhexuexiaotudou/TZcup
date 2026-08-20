# Ackermann 导航与 Coverage

Ackermann Nav2 profile 使用 `SmacPlannerHybrid`、`REEDS_SHEPP`、72 航向 bins、冻结最小半径 `1.429352 m`。四个 RPP controller 按用途拆分：`FollowPath/CleanPath/RepairPath` 只允许前进，`ReversePath` 只服务显式倒车段；它们都开启碰撞/曲率限速并关闭 rotate-to-heading，行为插件与 BT 不含 Spin。返回库位同样走 Hybrid/Reeds-Shepp，不允许在车位前原地对齐。

未知栅格正式建图的短 frontier waypoint 使用宽松终点航向，避免把观察点误当停车位；蛇形扫图发生横向到纵向换带时，从当前融合位姿、在线 costmap 与扫带方向选择短距已知自由区 staging pose。在线净空允许直接前进时跳过倒车；否则最多执行两次 Nav2 原生碰撞检查 BackUp，耗尽后失效关闭。换带优先构造解析 forward Dubins 并按曲率反转点拆成曲线/直线/曲线原语；解析路径不可用时才使用 Smac Hybrid/Reeds-Shepp 规划一次，再按前进/倒车 cusp 分段。所有段在整路径在线 costmap 净空复核后分别交给 forward-only `DubinsPath` 与 `ReversePath` 完成动作终态交接；禁止用连续宽松 frontier waypoint 暗示车辆已经改变航向，也禁止每秒重规划或允许倒车的通用控制器在静止车位改变起始档位。

前沿排序保留原始栅格坐标，同时将远端前沿转换为满足 Ackermann 航向变化上限的短距本地圆弧端点。若代价图拒绝或 Nav2 执行失败，冷却集合必须同时包含原始前沿与实际下发端点；只排除本地端点会让同一远端前沿以另一个短圆弧重复入选，形成“动作成功但地图零增益”的循环。报告通过 `raw_frontier_exclusion_count` 显式记录该恢复分支。

短距本地圆弧端点被在线 SLAM 栅格或 Nav2 costmap 阻挡，或者已下发的普通短 frontier 无路、超时或中止时，探索器不会沿视线盲目延长目标。它为该原始前沿排队一次 fallback，先请求 `ComputePathToPose` 规划到已知侧接近点，校验路径起终点、有限坐标，并按两张在线栅格的较细分辨率加密采样整段车辆净空；只有全部通过，才沿返回路线截取最长 `30 m` 的前视点交给 `NavigateToPose`。空路径、过短路径、端点不符、任一点不安全、规划拒绝或超时都会失败关闭，同时冷却实际端点和原始前沿。

水平 sweep 的 frontier 全部暂时不可用时，等待不是无限状态：连续 5 次等待后，从在线 SLAM 已知自由栅格中按扫带方向、前向进度和横向偏差选择不超过 30 m 的 staging 候选，再走同一全局路径校验链。route 失败会冷却该候选并立即尝试下一个，不受普通 frontier TTL 阻塞。连续 30 次仍无安全候选时只允许碰撞检查倒车；倒车也不可用则以 `horizontal_sweep_frontier_deadlock_no_safe_recovery` 失败关闭。

Coverage 保留 Fields2Cover 弓字条带生成，产品刷宽为 `1.32 m`，规划间距候选为 `1.06/1.10/1.15/1.20 m`。连接器优先使用曲率和 footprint 均受检的 forward Dubins 路径，再尝试 forward U-turn、forward teardrop、Reeds-Shepp-like three-point 与 Smac Hybrid；仍不可行则延期下一条带。forward Dubins 在执行时按曲线/直线/曲线原语拆分，并在原语边界由 Nav2 goal checker 完成闭环交接。计划只含 `FORWARD/REVERSE/CUSP_STOP/DEFERRED_SWATH`，每个方向切换必须停车到实测 `|vx|<0.03 m/s`。不得出现 `ROTATE/SHIFT/Spin` 或瞬时换挡。

专用世界把外部 turning apron 扩展到 `15.6 m × 10.0 m`（`156.0 m²`）；青色 `x=[-2,2], y=[-3,0]` 清扫区和 12 m² 面积保持不变。额外空间用于容纳物理最小中心半径 `1.429 m`、执行保守半径 `1.8 m` 的前进式连接和直线引入段，不计入覆盖率。启动方式：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_gazebo_cleaning_demo.ps1 -DriveModel ackermann -CoverageProfile ackermann
```

```bash
./scripts/run_visual_demo.sh --drive-model ackermann --coverage-profile ackermann
```

机器状态由九个证据门的逻辑与产生。产品运行入口默认使用 Ackermann，legacy 只保留显式回归；但完整 5 m/圆周/零速转向/三点掉头、轮式里程计、10-seed 定位、5-seed Coverage、20 次动态交互、30 次急停和 MCAP replay 未全部通过时，不得把默认 profile 等同于产品验收通过，也不得手动把 `ACKERMANN_DEFAULT_PROFILE_READY` 改成 true。
