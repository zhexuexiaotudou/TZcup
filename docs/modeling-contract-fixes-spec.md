# R065 建模合同修复 Spec

## 1. 目标与治理

本变更修复当前正式仿真中已经由源码和留存运行证据确认的建模合同漏洞，同时保持既定技术路线：正式车辆仍为 A300 四轮滑移转向底盘，首次建图仍使用 `200 m x 100 m`、无实体外墙的 map-frame geofence，正式控制链不得消费 Gazebo/evaluator 真值，S100P 和真实整车状态继续与 PC/Gazebo 仿真状态分开报告。

执行层级固定为：Astra 负责本 Spec、边界和最终技术裁决；Sol 负责拆解、并行调度 Terra、审查集成、验证和交付；Terra 负责互不重叠的实现与测试。只有下列问题需要由 Sol 上升给 Astra：

- 需要改变赛题指标、正式技术路线、truth boundary 或 hidden-task 冻结规则；
- 无法在不伪造实物参数的前提下满足验收；
- 两个验收条件在当前代码或运行环境中不可同时成立；
- 需要真实 S100P、A300、传感器、机械安装或标定输入才能继续。

## 2. 已验证问题

### MC-01：动态 Nav2 footprint 的 ROS 消息类型不匹配

`formal_dynamic_footprint_manager` 向 `/local_costmap/footprint` 和 `/global_costmap/footprint` 发布 `geometry_msgs/msg/PolygonStamped`，而留存 ROS 图证明 Nav2 的 footprint 输入为 `geometry_msgs/msg/Polygon`；`PolygonStamped` 是 `/.../published_footprint` 输出类型。DDS 端点因此不能匹配。正式 Nav2 初值是 `transport_stowed`，清扫机构展开后的 `cleaning_deployed` 以及机械臂状态的 `arm_deployed` footprint 没有得到运行证据证明已进入 costmap。

### MC-02：MoveIt 缺少持久地面碰撞体和场景就绪门

`bin_and_scene.yaml` 声明 ground 等必需对象，但没有启动链或执行器消费它。MoveIt 只加载机器人 URDF/SRDF，grasp executor 只把当前感知 cube 加入 world/attached collision objects。Gazebo 地面不会自动进入 MoveIt planning scene。现有 `avoid_collisions=true` 能检查机器人自碰撞和车体碰撞，但不能证明规划轨迹不穿过地面。首版修复草案又暴露出一个运行时合同错误：它把 `4 m x 4 m` 地面 patch 以 `base_footprint` frame 注入，但 MoveIt 会把 world object 变换后固定存入 planning frame，并在 `GetPlanningScene` 回读时报告 planning frame；因此该 patch 既无法覆盖车辆跨场地移动后的机械臂位置，严格按 `base_footprint` frame 回读也会失败。

### MC-03：行人整段轨迹未做静态/实体/相互碰撞校验

场景生成器只校验行人起点和终点，随后用直线往返。整段线段可能穿过建筑、树、垃圾桶等静态碰撞体，也可能穿过实体 cube；此外它只避开既有行人的起点，不检查两条完整路径。公开 `40 map x 20 mission` 审计在 `22,400` 对路径中检出 `251` 对中心线最小距离 `<=0.50 m`，影响 `216/800=27.00%` episode，最强反例距离仅 `0.000040799 m`。当前行人 SDF 为 `static=true`，10 Hz driver 使用绝对位姿更新，不会由物理碰撞响应自动分离。修复必须检查实际 2D collision 几何，不得只用会产生大量误报的统一外接圆；本变更不扩张为动态刚体/物理驱动重构。

### MC-04：建模声明与可证明能力边界需要同步

校园 `puddle` 是感知/覆盖用的无碰撞视觉表面，不是水深、水量或水动力模型；水回收结论只能来自独立 water runtime。静态 FOV、离散机械臂 anchor、URDF 质量/惯量检查也不能改写为 S100P、连续机械臂空间或真实整车通过。`closed-campus-first-map-then-clean-plan.md` 中“正式速度始终 0.45 m/s”的旧表述还需与当前“mapping=0.45 m/s、隔离 dry candidate=1.0 m/s、通过四门后仍需实测效率”的正式速度合同对齐。

### MC-05：source-world 与 localization-map 的 geofence 字段语义含混

public manifest 的 `geofence_frame: map`/`geofence_polygon_m` 数值实际位于 Gazebo source-world 坐标；map lifecycle 再以固定车辆起点把它平移到 local localization map。当前数值链自洽，但字段名容易让新 consumer 跳过或重复平移。地图还有静态 materializer、lifecycle support mask、SLAM occupancy 和 coverage planning 等不同分辨率；这些分层用途不能被描述成“同一张地图的一个分辨率”。

### MC-06：首次建图的 collision monitor 依赖未启动的 3D 点云源

首次建图 launch 显式设置 `high_bandwidth_sensor_runtime=false`，但生成的 Nav2 collision monitor 仍保留 `scan + mid360` 两个 observation sources 且启用 `mid360`。r064 真实 ROS 图只有约 `6 Hz` 的 `/scan/navigation`，`/sensors/lidar_3d/points` publisher 为 `0`；因此 collision monitor 收到 `/cmd_vel_nav=0.45 m/s` 后没有形成 `/cmd_vel_gate`，whole-vehicle safety manager 最终以 `command_timeout` 保持 `BASE_COMMAND_STOPPED`，覆盖只达到 `1.24145%`。这是正式首次建图链的确定性配置/运行合同错误，不是降低碰撞安全阈值的理由。

## 3. 实现范围

### W1：修复动态 footprint 输入

- 将 costmap footprint 输入 publisher 改为 `geometry_msgs/msg/Polygon`，不得向输入 topic 发布 stamped 消息。
- 保留 `/formal_vehicle/navigation/footprint_status`，状态必须报告所选 profile、是否允许导航和触发原因。
- 不改变三个冻结 polygon 数值，不扩大或缩小物理声明。
- 增加可复用的 exact-polygon 比较/规范化逻辑，避免 local/global 和测试各自实现不同精度规则。
- 增加静态合同测试和 ROS 运行门：manager publisher 与 costmap subscriber 类型必须匹配；依次验证 transport、cleaning、arm 状态后，local/global `/published_footprint` 必须分别等于冻结 polygon。运行门不得向生产共享 `/joint_states` 注入伪造状态；若使用测试覆盖接口，该接口必须由默认关闭的显式 launch 参数启用、只影响 footprint 选择、在 base motion inhibited 且无执行器命令的隔离验收中运行，并在状态 reason/evidence 中明确标为 test override。机械臂展开时独立 base-motion inhibit 仍是硬门，动态 footprint 不是其替代品。
- 运行门不能只凭 safety manager 出现在订阅端点就声称底盘已锁止；每次 override 都必须读取 fresh `/safety/status_json`，证明 `whole_vehicle_safety_manager` 的发布序号前进、状态为生产定义的 base-stopped 状态、原因包含 `manipulator_base_inhibit` 且 publish thread 健康。ROS endpoint 的 node name 与 namespace 必须同时按真实 Nav2 图校验。

### W2：建立 MoveIt planning-scene bootstrap

- 将 `bin_and_scene.yaml` 从死配置改为正式输入；区分 `required_robot_links` 与 `required_world_objects`，不得把 URDF robot link 伪装成 world object。
- 启动/首次抓取前向 MoveIt world 注入有厚度、有限且覆盖完整 `200 m x 100 m` 正式 geofence（包含明确余量）的 `ground` box，frame、尺寸、顶面高度和 ID 必须来自配置并可验证。该 world object 必须直接存于 MoveIt 实际 planning frame（当前正式 SRDF 为 `map`），不得用会在应用时冻结为一次性世界位姿的 `base_footprint` 局部 patch 冒充全场地地面；`GetPlanningScene` 会以 planning frame 回读 world object，bootstrap 必须按该真实语义校验。正式 source-world geofence `x=[-100,100], y=[-50,50]` 以起点 `(-98,0)` 做唯一一次定位变换后，在 `map` 中为 `x=[-2,198], y=[-50,50]`；地面中心/边界必须从这条来源链推导并有覆盖测试，不能把 source-world 中心 `0` 直接当作 map 中心。地面高度仍须服从正式 URDF datum：`base_footprint` 是地面投影，`base_link` 相对它为 `+0.1651 m`；不得把 `base_link z=0` 错当物理地面。
- 正式 `formal_vehicle.srdf` 必须声明 `map -> base_footprint` 的 planar virtual joint，使 MoveIt model/planning frame 与正式定位 TF 链一致；机械臂 group 不得因此包含或规划底盘自由度。MoveIt CurrentStateMonitor 应只从正式 TF 更新该 multi-DOF joint，不得向 `/joint_states` 或执行器端点注入伪造底盘状态。
- 规划场景必须显式处理轮胎与地面的正常支撑接触：只允许配置列出的 wheel-support links 与 `ground` 接触，并回读/验证相应 allowed-collision matrix；不得为方便而放开机械臂、车体或全部 robot-world 碰撞。
- bootstrap 必须通过 `GetPlanningScene` 或等价只读接口回读 ground 的 ID、frame、shape、pose 和 scene readiness；未知 revision、缺对象、TF 不可用或 ground-only removal 后对象未恢复均 fail closed。
- 持久性测试只能用带已知 revision 的 scene diff 精确 `REMOVE ground` 来模拟对象丢失；不得发送 robot state、ACM、transforms 均为空的 `is_diff=false` 全量 scene，因为 MoveIt 会清空现有 ACM/world 并破坏 SRDF 相邻碰撞豁免。若未来确需全量 reset，必须完整回读并原子重放非 ground 状态后另立验收。
- bootstrap 在 timer callback 内等待异步 MoveIt service 时，Apply/Get response 和 TF subscriptions 必须位于可并发调度的 callback group，timer 本身不得重入；仅创建 `MultiThreadedExecutor` 而仍把全部实体留在同一默认 mutually-exclusive group 不算满足。
- live gate 必须回读 planning frame=`map`，并证明正式 virtual joint 存在于 scene robot state 的 `multi_dof_joint_state`，其 fresh 位姿与同一时刻的 `map -> base_footprint` TF 在容差内一致；TF 缺失、陈旧或 frame 冲突一律 fail closed。独立 fake-control 启动若没有合法 map TF，只能报告未就绪，不能默认零位姿冒充正式定位。
- 现有 perceived-cube world/attached object 生命周期保持不变。不得将 Gazebo truth/model identity 引入抓取请求或控制链。
- 增加负向测试：低于 ground 顶面的目标必须先在关闭碰撞过滤时得到可用的 IK/robot state，或使用冻结扫描中已位于关节限位内且穿越地面的确定性关节样本，再由只读碰撞查询（优先 `GetStateValidity` contacts，或等价接口）证明拒绝原因明确包含 world object `ground` 与非豁免机械臂 link；仅凭 collision-aware IK/Cartesian path 的失败码不得判定地面生效，因为它也可能来自不可达、关节限位、自碰撞或车体碰撞。正常 pregrasp/pick/lift/deposit 路径仍须通过既有 MoveIt 碰撞检查。若本机无 ROS/MoveIt，只能把该项报告为待 Linux runtime gate，不能用 mock 冒充通过。

### W3：修复场景几何有效性

- 为旋转 box 和 cylinder 提供依赖最小、确定性的 2D segment-clearance 检查，并复用现有 bounded sampling。
- 行人路径必须在整段上避开静态 collision（按行人半径膨胀）和实体 cube；起点、终点及整段使用同一几何语义。
- 每条新行人路径还必须与全部既有行人的完整 2D segment 保持严格大于两个行人半径之和的中心线间距；当前半径均为 `0.25 m`，所以 exact segment-to-segment distance `<=0.50 m` 必须拒绝。生成器和 validator 必须复用同一语义，并覆盖相交、平行近距、端点近距及恰好阈值四类边界；采用任意相位均安全的保守空间门，不修改既有 `static` SDF、10 Hz driver 或赛题路线。
- dirt/puddle/leaf 是地表语义，不作为行人硬碰撞物；cube 与 dirt 的重叠也不在本次一刀切禁止，避免把合理复合任务错误删掉。
- 生成失败保持 bounded/fail-closed；不得改 split 数量、seed 派生、public/environment/evaluator 分层、hidden 一次性消费或对象计数。
- 新增 `validate_episode_geometry` 一类可复用校验入口，并在 SDF 输出前调用。测试覆盖相切、端点、旋转 box、cylinder、反向线段、零长度和不可放置场景。
- 以向后兼容方式在 public manifest 中明确 source-world geofence 与 localization-map geofence；既有 legacy 字段在迁移期必须有单一、机器可读的语义和弃用说明。所有 consumer 必须优先读取显式字段，并有“未平移/重复平移”负向测试。
- manifest/report 分别记录静态栅格、lifecycle support mask、SLAM occupancy 与 coverage planning resolution 的用途和值；不得强行把不同用途改成同一分辨率。值必须来自正式配置、函数实参或运行时保存地图 metadata；没有代码来源的常见默认值不得升级成冻结合同。

### W4：声明与文档同步

- 明确校园 puddle 仅是 perception/coverage surface；水深与回收验收指向独立 water runner。
- 明确静态 mesh-ray FOV、离散 anchor 和 URDF deterministic PASS 的边界。
- 同步速度文档：首次建图/普通安全配置为 `0.45 m/s`；`1.0 m/s` 仅是显式 opt-in 的 dry requalification candidate，必须依次通过 mobility、interlock、dynamic-obstacle、ground-dirt 和最终 measured-efficiency 门，且本变更不授权实机提速。
- 更新 `docs/progress.md`；仅当项目入口或使用方式改变时才修改根 `README.md`。

### W5：收口首次建图 collision-monitor 输入

- 首次建图且 `high_bandwidth_sensor_runtime=false` 时，生成的 collision monitor 必须只声明实际存在的 2D `scan` observation source；不得保留没有 publisher 的 `mid360` source。
- 普通显式启用高带宽传感器的正式配置继续保留既有 3D source；不得通过放宽 scan 的碰撞半径、TTC、source timeout 或 collision-monitor 安全动作来绕过问题。
- 配置回归必须分别绑定 mapping scan-only 与 high-bandwidth 双源模式。ROS live gate 必须由真实 Nav2 导航命令证明 fresh `/cmd_vel_gate` 进入 whole-vehicle safety manager，状态不再因 `command_timeout` 停车且里程/odom 证明车辆实际移动；gate 不得向导航或底盘注入伪造非零速度。证据必须对整条命令链逐 topic 记录 sole publisher identity（正式预期依次为 controller server、velocity smoother、collision monitor、whole-vehicle safety manager），不能只核对后两级后便把任意来源的上游非零样本归因给 Nav2。

## 4. 明确非目标

- 不伪造 S100P 板框、孔位、质量、热、连接器或功耗数据；这些保持 external blocked。
- 不通过直接修改 URDF 质量数字制造 A300 载荷余量；当前距 5 kg 工程余量仍需实物减重约 `4.969583 kg`。
- 不声称解决连续机械臂全空间、液体晃动/CFD、刷毛法向力、真实有效清扫宽度、轮胎打滑或实车标定。
- 不访问或提前物化 hidden truth，不根据 hidden 结果调参。
- 不改变 r064 GNSS/odom origin 修复和正在保留的租卡失败证据。

## 5. 完成标准

### 自动化

- 受影响的纯 Python/合同/场景生成测试全部通过。
- `py -3 scripts/ci_fast.py` 全绿；Linux/CI 使用 `python scripts/ci_fast.py`。
- 任何改动的 Bash 脚本通过 `bash -n`。
- 生成器在全部公开 train/val map 的固定非 hidden 样本上，行人-静态 collision、行人-cube collision 和行人-行人完整路径间距违规均为 0；固定 `40 map x 20 mission` 的公开审计须报告 `0/22,400` pair violations。对象计数、field area、seed determinism 和 public truth boundary 不变，审计不得访问 hidden。
- source-world 与 localization-map geofence 的显式坐标经过一次且仅一次固定起点变换后完全一致；旧字段兼容读取测试通过。
- 首次建图的 materialized Nav2 参数只启用实际在线的 `scan` collision-monitor source；高带宽正式模式仍保留既有 `scan + mid360` 合同。

### ROS/Gazebo/MoveIt 运行证据

- live ROS graph 证明 local/global footprint 输入端点是 `Polygon` 且 manager publisher 与 costmap subscriber 匹配。
- transport、cleaning、arm 三次状态切换均从 local/global `/published_footprint` 读回 exact polygon；collision monitor 继续消费 local published footprint。
- 三次 footprint 切换期间，fresh safety status 均证明独立底盘 inhibit 已被实际采纳；运行门的 endpoint node namespace 与真实 costmap graph 一致。
- MoveIt planning scene 回读 ground；低于地面的可达 robot state 被拒绝且碰撞 contacts 明确归因到 `ground`，正常抓取路径通过，失败时无执行器命令绕过。
- 至少一个公开 formal train/val episode 在 Gazebo 中证明行人运动不穿静态碰撞体或其他行人，且动态避障证据仍能形成。
- 首次建图 live 链必须证明真实 Nav2 command 产生 fresh collision-monitor gated command，whole-vehicle safety manager 不再报告 `command_timeout`，并由实际 odom/里程证明车辆运动；不得由测试节点发布非零速度冒充该链。

### 交付门

- Sol 完成 diff/secret 审查、提交、推送和 PR；CI 必须全绿后合并。
- 合并后的 exact `origin/main` 修订部署到批准的 ROS 2/Gazebo 环境，重跑上述 runtime gates。纯 Windows 静态测试不能替代此门。
- 运行 `neat-freak` 做知识同步；若产生版本化改动，走补充 PR。
- 最终报告必须分别列出：已修并实测、已修但待 Linux runtime、仍需实物输入、与 S100P 上机的关系和回滚点。

## 6. 工作拆分建议

- Terra A：W1 动态 footprint 类型、状态与 ROS 测试。
- Terra B：W2 MoveIt planning-scene bootstrap、配置与负向测试。
- Terra C：W3 场景几何校验、公开集量化与生成器回归。
- Terra D：W5 首次建图 collision-monitor source 配置与 live 链 gate。
- Sol：W4、跨模块集成、全量 CI、PR/CI/部署、证据汇总；根据槽位分批调度 Terra。

W1、W2、W3 文件域独立，可并行实现；Sol 在合并前负责处理测试与配置交叉。任何 Terra 不得直接改变本 Spec 的赛题/实物边界。
