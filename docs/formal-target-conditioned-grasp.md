# 正式目标相关抓取闭环

正式抓取不再用固定 `PICK` 关节角代表随机目标。`/active_cleaning/grasp_request`
采用 v2 合同，必须来自产品感知链并同时携带 track id、完整 3D pose、实测三轴尺寸、
置信度和 `truth_used=false`。随机颜色无法揭示纸板/PP/PET/铝，因此产品请求中的可选
`material` 只能为 `unknown`，不得从颜色推断或读取仿真真值。模型名、Gazebo entity id
和世界真值接口均被拒绝。

`formal_physical_grasp.launch.py` 复用 `manipulation.launch.py` 启动一个
`move_group`；既有正式整车 launch 继续独占 robot_state_publisher 和
controller_manager。当前后端明确是 MoveGroup action、`/compute_ik`、
`/compute_cartesian_path`、`/apply_planning_scene` 和 `/execute_trajectory`，
没有安装或声称使用 MoveIt Task Constructor。

同一 launch 会并行启动独立 planning-scene bootstrap，而执行器在发出任何命令前
必须等待其就绪门。bootstrap 从
`config/bin_and_scene.yaml` 注入有限厚度 `0.50 m`、顶面 `z=0` 的配置化
`ground` box。它直接位于 MoveIt 的实际 planning frame `map`，不是随车的
`base_footprint` patch：公开 source-world geofence 是
`x=[-100,100], y=[-50,50]`，以唯一固定起点 `(-98,0)` 转入 `map` 后为
`x=[-2,198], y=[-50,50]`；四侧各留 `10 m` 后，ground 的尺寸和中心固定为
`220 x 120 x 0.50 m` 与 `(98,0,-0.25)`。这条 source bounds → 一次变换 →
map bounds → margin 的 provenance 写入配置并由纯函数测试四个 geofence 角点，
禁止把 source-world 中心 `0` 误当 map 中心。`formal_vehicle.srdf` 的
`formal_world_joint` 是 `map → base_footprint` 的 planar virtual joint；机械臂
group 仍只含机械臂，不规划或执行底盘。bootstrap 每轮从 live `move_group`
语义参数读取这个 joint，frame/name/child 任一与配置不符即 fail closed。只在
allowed-collision matrix 中放行四个轮支撑 link 与地面的正常接触。bootstrap 必须以
`GetPlanningScene` 回读 scene revision、ground ID/frame/box 尺寸/位姿和该
wheel-only ACM，未知 revision、TF 缺失或 ground 丢失会先锁存
`planning_scene_ready=false`；抓取执行器在这一硬门恢复并完成回读前不得发送
机械臂、夹爪、存储或 planning-scene 命令。持久性 gate 只发送带已知 revision 的
`is_diff=true + REMOVE ground`，并在恢复后实时比对非 ground world objects 与非
ground ACM；禁止用空的 full-scene reset 清除 robot state、SRDF 相邻碰撞豁免或
其他对象。每次 perceived-cube 的 world/
attached diff 也必须递增已知 revision，并在写入后重新证明 ground 与 ACM
仍存在。

Linux/MoveIt 运行门使用冻结且处于关节限位内的 `halton_00772` 负态，通过
`GetStateValidity` 响应 contacts 明确要求 `ground` 与非豁免机械臂 link 接触；ROS 2
请求本身没有 contacts 开关，MoveIt state-validation capability 在服务端启用并返回该证据；
正常 pregrasp/pick/lift/deposit 则必须继续通过 collision-aware IK、state
validity 与 Cartesian 检查。纯 Python 对配置和消息结构的测试不构成这项 live
证据，本机无 ROS/MoveIt 时该门必须保持 `PENDING`。live gate 还必须从
`GetPlanningScene.robot_state.multi_dof_joint_state` 回读 `formal_world_joint`，
并与 fresh `map → base_footprint` TF 的位姿一致；缺失、陈旧或不一致都失败关闭。
因此 standalone fake-control 若没有定位或明确的静态 `map` TF，不能报告 scene ready。

该 live gate 同时只读订阅 `/rosout`：JSON 会记录八个 bodywork service-door
hinge/latch joint 的已知 missing-joint warning 是否观察到、计数和去重摘要。这些
被动物理门铰链/锁扣只允许经 evaluator-only `/formal/service_door_joint_states`
反馈，绝不补写 shared `/joint_states`，也不因该记录改写 SRDF passive 定义。其他
MoveIt `ERROR/FATAL` 不会被此分类吞掉，会原样记录在输出中供归因，但日志采集本身不增加
新的通过条件；无论 warning 是否出现，正常 IK/state-validity/ground-contact 和
scene-completeness 服务门都仍是硬门。

任务序列为：停车与安全检查、MoveIt 回运输姿态、将感知立方体加入规划场景、目标相关
预抓、开夹爪、等待腕部 RGB-D 在 `/perception/wrist/grasp_recheck` 复测同一 track、
更新目标 pose/尺寸、直线接近、双指物理接触后夹持、把目标加入 attached collision
object、直线抬升、碰撞检查投箱、物理释放、材料质量和数量稳定增量验证、规划回撤、回运输
姿态和关投料门。为了允许真实接触，仅在最后短距离接近期间从 world collision set 移除
目标本体；整车和环境碰撞检查仍保持开启，夹持后立即改为 attached collision object。

PC 产品感知适配器只在腕部 RGB-D 帧中发布该复测话题，复用同一个 map-frame track id，
并再次输出三维 pose、尺寸、置信度和 `material=unknown`；无效四元数、尺寸或低置信度
复测会在发布前失败关闭。

机械臂任务开始前发布锁存的 `/manipulation/base_motion_inhibited=true`。只有机械臂恢复
`TRANSPORT` 且投料门关闭后才发布 `false`；预释放失败会保持 inhibit，要求人工恢复。
整车 safety manager 必须把该 topic 接入底盘速度门禁，但不得因此切断机械臂和存储执行器。

20 块连续验收入口为：

```bash
python3 scripts/prepare_formal_20_cube_grasp_acceptance.py \
  --output artifacts/formal_20_cube_grasp_manifest.json
bash scripts/run_formal_20_cube_grasp_acceptance.sh
```

该入口现在是自包含正式 runner：它在独立的 ROS domain/Gazebo partition 中启动正式整车、
MoveIt、安全管理、接触门、干垃圾箱监视器以及 20 个物理刚体，并在退出时按进程组清理。
runner 必须绑定当前 RUNNING 的正式验收 session 和冻结 snapshot；缺失、过期或哈希不一致
均失败关闭。清单把 20 个边长 3 cm 的方块放在唯一的 5×4 单层槽位中，槽距
分别为 6 cm 和 7 cm；即使方块采用任意平面朝向，实体包络之间仍大于 5 mm。全部槽位
相对 `arm_mount_link` 的最大平面距离为 0.824621125 m，小于 0.85 m，并仍须逐块通过运行时 MoveIt IK 与碰撞
检查，静态几何条件不替代运动学真值。

验收要求 `paperboard`、PP、PET、铝四种材料各 5 块并按种子随机分配，产品请求始终只见
`material=unknown`，且每块外观颜色独立随机、不编码材料类别；20 次必须全部物理落箱。每次均核对投箱后的材料质量类别、质量增量、
箱内件数 `+1` 和累计动态载荷；按冻结材料质量，20 块总质量为 0.7668 kg，最终累计质量
必须闭合，同时通过与产品控制隔离的 evaluator-only 状态交叉核对：场景始终存在 20 个刚体、
箱内惯性刚体逐次从 0 增至 20、最终物理质量为 0.7668 kg，且每一抓均观测到底盘 inhibit 的
true/false 生命周期。evaluator 状态不进入抓取执行器输入。总功能验收合同将该门绑定到腕部抓取观察、六轴臂、夹爪、投放口、
干垃圾箱和料位监视六个功能位置；单方块通过不能替代 20 块门。
正式 live 场景仍必须逐块完成后才能生成 PASS 报告；静态/单元合同不能替代该门。

每块最多执行两次。只有第一次在进入物理夹持阶段之前失败、机械臂和投料门已自动恢复到
安全运输状态，并且 evaluator 复核箱内件数和物理质量均未变化时，才允许第二次；一旦进入
夹持、发出释放命令、质量/件数发生变化或需要人工恢复，立即失败关闭，禁止靠重复请求产生
重复计数。runner 启动前还会把已有的 canonical 报告改名保留，并检查当前源文件仍与冻结
snapshot 一致，因此缺少 session、源代码漂移或启动失败时不会残留一个可误认成当前结果的
旧 PASS。

需要特别区分：本门当前使用清单生成的、符合产品 v2 schema 的目标位姿来验收“目标相关抓取
接口及机械闭环”，并不单独证明 DOSOD/EdgeSAM 从相机图像产生了这 20 个目标。真实视觉来源
必须由随机场景感知门和最终单 episode 端到端门另行绑定；这里不得把 schema 隔离写成感知
模型精度已经通过。
