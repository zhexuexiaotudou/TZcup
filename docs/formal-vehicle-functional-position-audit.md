# 正式整车功能位置独立审计

审计日期：2026-08-28  
审计范围：正式整车部件台账、当前 Xacro 展开结果、控制器配置、正式启动入口、功能验收合同，以及从 URDF 完成后到最终技术方案闭环的已有证据。  
审计方式：只做仓库文件、报告字段和 SHA-256 绑定核对，不启动 Gazebo，不重跑任何运行门，也不把历史或未绑定报告当作当前快照验收。

本文使用三种证据等级：

- **最终接受**：证据属于同一个有效 `formal_final_acceptance_session.json`，绑定当前未漂移 snapshot，并满足当前合同；
- **独立单项证据**：报告本身通过，但没有绑定最终 session、当前 snapshot 或完整产品运行链；
- **静态/替身证据**：只证明结构、接口、纯 Python 规划或历史回归，不能替代 Gazebo/Nav2/真板/端到端验收。

## 结论

当前整车已经从“几何占位”进入了可核对的产品结构。最近一次已生成的部件台账报告登记 9 个传感器安装、18 个机械子总成、38 个功能位置、63 个显式子部件、86 个话题合同和 29 个单写者合同，展开 URDF 哈希为 `d553a07367bc44980cb38bb3396ad0b1fc5d396949173a3c87576deb21397128`，静态台账报告状态为 `COMPONENT_REGISTER_URDF_FK_AND_INTERFACES_VALID`。

但该快照现在不能作为冻结验收基线。对 `formal_vehicle_snapshot_manifest.json` 逐文件复算发现 2 个 source inventory 条目已经漂移：

- `starter_ws/src/sanitation_safety/sanitation_safety/simulation_safety_inputs.py`；
- `starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro`。

同时，`artifacts/formal_final_acceptance_session.json` 不存在。审计后已用当前合同重新生成 `reports/engineering/formal_functional_acceptance_audit.json`：它现在登记完整 38 个位置（含 `bodywork_service_access`），但因缺少有效最终 session，结果仍是 0 个最终通过、38 个 pending。按当前 fail-closed 合同，没有任何功能位置可以在本轮审计中被提升为“最终接受”。

正式整车启动默认使用 DART，原因是 Bullet 在该整车模型上注入非物理能量。DART 原生不建立 URDF mimic 约束，因此当前实现由项目内 `GripperMimicEffortSystem` 对五个从动关节施加受限动力学联动；最新机械臂运行证据已观察到五个从动状态，最大 mimic 误差约 `9.3e-12 rad`。Gazebo 仍会打印原生 mimic 不支持警告，但夹爪联动不是依赖该原生约束；最终 session 仍需重新绑定这项证据。

## URDF 完成后到最终方案闭环总表

| 功能/验收 | 已证实 | 当前缺失或阻断 | 证据路径 | 下一动作 |
| --- | --- | --- | --- | --- |
| 正式 URDF、产品/检修外观与冻结基线 | 视觉合同已扩为 19 视图，并为干投口、电池/BMS/PDB/安全继电器/接触器/LV 配电、充电口和排污硬件设置专用相机；静态交叉表完整覆盖 38 个功能位置、9 个传感器安装和 18 个机械子总成。产品外观检查 156 个物理 link（110 个由台账直接推导）；检修外观移除车身 skin 后检查 142 个仍可见的物理 link（96 个由台账直接推导）。每个被检查 link 必须同时有 visual/collision 且在折叠机械臂、张开夹爪、开启投放闸门的实际采集姿态投进所分配相机；每帧还要求 PNG hash、字节数和尺寸绑定 | 旧 15 视图报告为 schema v3，缺少逐图绑定，已由 v4 合同明确判失效；必须 fresh 重渲染后才能恢复通过 | `reports/engineering/formal_vehicle_snapshot_manifest.json`；`reports/engineering/formal_vehicle_visual_acceptance/manifest.json`；`reports/engineering/formal_vehicle_service_visual_acceptance/manifest.json` | 冻结源码并生成 fresh snapshot；在新 session 中重跑产品/检修 19 视图，逐图复核 hash、尺寸、38/9/18 ID 交叉表和逐 link 投影门 |
| A300 底盘、编码器、负载与整车安全 | 当前独立直行/停车报告通过：Gazebo 真值前进 `0.9424 m`，四轮均被观测，零指令后轮速归零；整车 ROS 命令路径互锁和辅助电源/灯光也各有单项通过报告；DART 下的夹爪从动关节由专用受限动力学插件闭环 | 只证明直行/停车；未证明当前冻结快照下的转向、负载、路径跟踪、避障、硬件相关牵引/制动。直行 runner 现已要求冻结 runtime closure、RUNNING session 和 snapshot/source hash 三重绑定，但尚无新的 Gazebo 证据 | `artifacts/formal_a300_drivetrain_runtime.json`；`artifacts/formal_vehicle_safety/whole_vehicle_actuator_interlock.json`；`artifacts/formal_auxiliary_power_lighting_runtime.json`；`config/high_fidelity_vehicle/a300_drivetrain_realism_contract.yaml` | 在同一 session 重跑空载/满载直行、停止、左右转向、四编码器/里程计一致性、急停和恢复，再由 saved-map Nav2 真实走路径 |
| 传感器、首次建图、固图定位与保存地图清扫 | 静态安装链覆盖 UTM-30LX、MID-360、GNSS、IMU、前/腕 D435 与双鱼眼；随机感知运行中确实收到 4 路 RGB、2 路深度和 4 路 CameraInfo；历史 Stage4W/AUTO-11 只可作旧车/离线回归参考 | 当前 `formal_vehicle_runtime_report.json` 与 FOV/遮挡报告缺失；`formal_map_lifecycle_acceptance.json` 明确 BLOCKED，质量门地图、真实首次建图和独立 AMCL 固图清扫三项均未通过；不能用历史 Stage4W 或离线大地图替代当前正式车闭环 | `artifacts/formal_map_lifecycle_acceptance.json`；`scripts/run_formal_first_map_dynamic_prerequisite.sh`；`scripts/run_formal_saved_map_cleaning_lifecycle.sh`；`scripts/collect_formal_vehicle_sensor_runtime.py` | 在冻结整车上先关闭全部话题/频率/TF/FOV/自遮挡，再完成真实 `200 m × 100 m` 探索、已观测面积 `>=95%`、地图哈希封存、独立进程硬重启 AMCL 和 saved-map FullCoverage |
| DOSOD + EdgeSAM 产品感知 | 已有 3 个互相分离的正式 Gazebo 相机 episode；真实相机消息和 truth 隔离成立；真实 S100P 上官方参考 DOSOD 已以约 3 Hz 输入、15 ms BPU infer 完成 smoke，正式 adapter 与 RGB→NV12 桥可启动 | 三个 Gazebo episode 的方块 precision/recall/F1、地污 IoU/recall 和地图投影仍为 0 或空；板端 smoke 使用上游参考模型且输出为空，不是项目四类验收；真实 RGB-D/TF/map 与 EdgeSAM 非空产品输出未闭合 | `artifacts/formal_random_scene_perception_acceptance.json`；`artifacts/formal_s100p_board_smoke_20260830.json`；`scripts/run_formal_random_scene_perception.sh` | 冻结 session 后优先在 S100P 完成同版本四类 DOSOD+EdgeSAM/adapter/BPU 图与遥测门；PC 真实相机推理仅作对照。Gazebo 三个分离 episode 仍独立重跑，要求 20 块、18 个地污区域与 map 投影全部过门后才能进入端到端 |
| RL 规划双模式与跨地图泛化 | 纯 Python、belief-only 评测中，全覆盖基线 4/4 通过；“Q-learning + 六次低增益后切系统覆盖”双模式在验证/隐藏地图 4/4 通过，零碰撞/越界/非法动作，且 `truth_used_for_control=false` | 纯 Q 本身只有 1/4 正式成功，不能单独称为可用策略；当前通过依赖系统覆盖 backstop；报告未使用产品感知、Gazebo 动力学、Nav2 或 S100，也未绑定当前 snapshot/session | `reports/engineering/formal_rl_multimap_v7_evaluation.json`；`scripts/generate_formal_rl_multimap_report.py`；`docs/formal-single-episode-cleaning-acceptance.md` | 冻结带非空 Q 表的 checkpoint 和切换条件；先完成同 saved-map 的真实 FullCoverage 长跑基线，再用相同 episode/seed、真实 DOSOD+EdgeSAM belief 和 Nav2/Gazebo 执行双模式，对路径、覆盖、效率和失败样本做配对比较 |
| 动态避障与行人安全 | 合同、8 行人 schedule、collector/validator 和真值隔离边界已经实现 | 当前报告为 BLOCKED；首次前置错误为 `No module named 'sanitation_formal_campus_integration'`，随后 saved-map、当前 checkout 绑定、8 行人、Nav2 完成、实际绕行、Collision Monitor 干预和零碰撞等全部运行门均未形成证据 | `artifacts/formal_dynamic_obstacle_avoidance_acceptance.json`；`scripts/run_formal_dynamic_obstacle_avoidance.sh`；`scripts/prepare_formal_dynamic_obstacle_schedule.py` | 修复冻结 overlay 的包安装/来源绑定；依赖已通过的 saved-map lifecycle，在正式运输收纳姿态下完成 8 行人随机横穿、实际绕行、零物理碰撞、零越界和安全发布者单写者审计 |
| UR5e、腕部重观测、2F-85 与目标相关抓取 | 六轴/夹爪轨迹、五个 mimic 从动关节、单个 PET 方块的双指接触、抬升、释放、落箱和箱内复核均有独立通过证据；单块箱内实测质量增量为 `0.03726 kg`，控制请求未携带 simulator entity identity | 轨迹和单块报告未绑定当前 session；未证明连续全关节空间；正式 20 块报告不存在；尚未逐目标证明 DOSOD/EdgeSAM → 腕部重观测 → IK/碰撞检查 → MoveIt → 夹持/投箱 | `reports/engineering/formal_manipulator_runtime_report.json`；`artifacts/formal_grasp_executor_runtime.json`；`docs/formal-target-conditioned-grasp.md`；缺失的 `artifacts/formal_20_cube_grasp_runtime.json` | 在冻结 session 中按 5×4 单层布置逐块运行 20 次目标相关抓取，每块最多两次，保留腕部复核、IK、全场景碰撞、双指接触、抬升、释放和落箱证据 |
| 干垃圾投放与动态质量增长 | 单 PET 块落箱后 8 个稳定样本确认 1 块、`0.03726 kg` 增量；投放闸门开闭和实体留箱可观察 | 当前只是一块；20 块纸板/PP/PET/铝各 5 块的逐步质量阶跃、最终总和、重复计数防护、满箱闭锁和组合重心/惯量回绑均缺失；`formal_20_cube_grasp_runtime.json` 不存在 | `artifacts/formal_grasp_executor_runtime.json`；`scripts/run_formal_20_cube_grasp_acceptance.sh`；`scripts/validate_formal_20_cube_grasp_runtime.py` | 用 episode truth 质量逐块核对 20 次落箱增量，最终干箱增量必须等于 20 块质量之和，实体持续留箱且不与 aggregate payload 重复计入 |
| 地面脏污、刷盘/滚刷/升降与电机真实性 | 独立 1 m² 物理清扫报告通过：严格部分清扫为 `50%`、终态 `100%`、面积守恒误差 0，任务开始后无 set-pose，20 个刚体垃圾未被清扫插件篡改 | 报告未绑定当前 session；当前合同新增的 `formal_cleaning_actuator_motor_runtime.json` 不存在；旧执行器报告状态是 `...STORAGE_AND_RECOVERY_ACTUATORS_PASSED`，不满足新合同要求的 `...STORAGE_SERVICE_AND_RECOVERY_ACTUATORS_PASSED`；随机场景地污感知 IoU/recall 仍为 0 | `artifacts/formal_ground_dirt_cleaning_final_retry/ground_dirt_acceptance.json`；`reports/engineering/formal_function_positions_runtime_report.json`；缺失的 `artifacts/formal_cleaning_actuator_motor_runtime.json` | 先补齐升降、双侧刷、滚刷的转速/编码器/电流/温度/堵转保护和刮条预载，再由真实 EdgeSAM 地污 mask 驱动随机 18 区域清扫，并在同一任务中验证面积守恒 |
| 积水识别、刮吸回收、污水质量与排空 | L1 2.5D 单项报告验证正常回收率 100%、质量误差 0、24 列覆盖、满箱后地面扣减为 0，并包含一次受驻车互锁许可的排水质量下降 | 该报告未绑定当前 session，而且满箱证据使用 `9.42 kg`，与当前正式布局/Xacro 的 `8.30 kg` 有效上限冲突，说明它来自不同冻结状态，不能接受为当前车证据。正式 runner 现已要求冻结 runtime/session/source binding，并额外闭合“服务排出量 = 污水箱减质量 = DynamicPayload 减质量”；但没有新的 Gazebo 实跑，EdgeSAM 积水识别和同 episode 质量守恒仍未闭合 | `artifacts/formal_water_recovery_acceptance.json`；`artifacts/formal_water_recovery_final_current/water_full.json`；`config/high_fidelity_vehicle/formal_vehicle_layout.yaml`；`starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro` | 在当前 `8.30 kg` 合同上重跑正常、满箱零流量、滤堵、非法回收、排空和动态惯量回退；端到端中必须同时满足地面减量 = 累计回收量 = 污水箱质量增量 |
| 检修门、排污服务、充电、电源与灯光 | 新 19 视图合同为充电口、排污硬件和内部配电/安全链提供独立视角；辅助电源/灯光状态机另有单项证据 | 新视图尚未重渲染；动态检修门、充电和排污联锁仍须在冻结 session 重跑 | `reports/engineering/formal_vehicle_service_visual_acceptance/manifest.json`；`artifacts/formal_auxiliary_power_lighting_runtime.json`；`scripts/run_formal_service_door_runtime.sh`；`scripts/run_formal_service_interface_acceptance.sh` | fresh 重渲染 19 视图后，继续跑门锁、充电、排污和质量守恒矩阵 |
| RDK S100P / Journey 6P 真板部署 | 历史说明称真板曾完成身份、TROS、官方 HBM、项目 overlay、节点图、RGB→NV12 与 DOSOD BPU smoke；当前已核验的可读取 Git refs 仅能验证 G0 只读盘点，未找到 smoke 原始 JSON；collector/schema/validator 已要求实际模型及词表哈希和非空产品输出 | 尚无项目 DOSOD HBM、冻结词表、EdgeSAM encoder/decoder HBM、板端 manifest、真实传感器输入和 1800 秒正式 acceptance；未归档的官方 COCO/reference smoke 不能关闭语义门 | `docs/formal-s100-live-acceptance.md`；`docs/s100p-offline-predeploy.md`；缺失的 `artifacts/formal_s100_live_acceptance.json` | session/snapshot 后优先在真实 S100P 运行项目四类 DOSOD/EdgeSAM/adapter，采集至少 1800 秒；要求 BPU 后端、非空产品、DOSOD `>=2 Hz`、EdgeSAM `>=1 Hz`、p95 `<=1000 ms`、零重启/失败、可用内存 `>=5%`、峰值温度 `<=85 °C` |
| 单一随机 episode 与最终技术方案闭环 | runner、collector、baseline validator、hash 树和最终 aggregator 已具备源码入口 | `formal_end_to_end_cleaning_mission_acceptance.json` 与最终 session 均不存在；独立感知/抓取/积水/避障 JSON 不能拼接成通过 | `docs/formal-single-episode-cleaning-acceptance.md`；`scripts/run_formal_same_map_full_coverage_baseline.sh`；`scripts/run_formal_single_episode_cleaning_mission.sh`；缺失的 `artifacts/formal_end_to_end_cleaning_mission_acceptance.json` | S100P 计算门可在 session/snapshot 后优先并行采集；Gazebo 仍须在同一 episode ID/seed/cleaning 进程完成 saved-map、真实感知、双模式 RL/Nav2、动态避障、20 块抓投/质量、18 个地污区、积水守恒、服务/安全与返航，最终聚合时复核板端证据未漂移 |

## 功能—位置—验收矩阵

| 功能组 | 已确定的实体/安装位置 | 静态状态 | 当前动态状态 | 主要缺口 |
| --- | --- | --- | --- | --- |
| A300 底盘、四驱电机、四轮编码器 | `base_link`；四轮 joint；四个独立电机 link；四个独立编码器端盖 link；左右固定承载梁和垫块 | 已覆盖 | 直行/停车和安全互锁各有独立通过证据，但均未绑定最终 session；转向、负载和路径跟踪仍缺 | 先解决物理后端和 session 绑定，再完成前进、停止、转向、负载与编码器/里程计一致性；当前不能声称可驾驶闭环通过 |
| 单线 LiDAR | Hokuyo UTM-30LX，`[0.535, 0, 1.1621] m`，塔体前悬臂 | 已覆盖，职责限定为二维建图/定位/平面避障 | 当前传感器/FOV报告缺失；首次建图门失败 | 重跑 LaserScan、遮挡、slam_toolbox 首次建图和固图定位；不能用 MID-360 替代其二维建图证据 |
| MID-360 | ` [0.420, 0, 1.2731] m`，塔顶四点隔振板 | 已覆盖，职责限定为三维障碍点云 | 当前传感器/FOV报告缺失；动态避障门失败 | 重跑点云、盲区/自遮挡、局部障碍层和行人绕障；现有模型是扫描近似，不是 Livox 非重复扫描/测量级误差模型 |
| GNSS 天线与接收机 | ANN-MB 天线 `[0.365, 0.160, 1.1801] m`；ZED-F9P 模块和检修盒 `[0.360, -0.095, 0.9801] m` | 已分开建模并明确同轴连接语义 | 当前传感器与首次建图/定位门未通过 | 缺 RTK 改正链、遮挡/多路径和实物天线相位中心标定；模块质量仍是公开资料缺失后的工程分配 |
| IMU | VN-100，底盘内部隔振托盘 `[0, 0, 0.4091] m` | 已覆盖 | 当前 200 Hz 运行证据和融合闭环缺失 | 缺实物安装误差、温漂/偏置标定及当前快照 EKF 证据 |
| 前向 RGB-D | D435，前脸凹入式支架，俯视 25°；RGB/Depth/双 IR/CameraInfo | 已覆盖 | 当前传感器/FOV报告缺失；DOSOD+EdgeSAM 随机场景门失败 | 需实测 3 cm 方块/脏污有效识别距离和遮挡；Gazebo 内参不能替代实物标定 |
| 左右侧后鱼眼 | 两套 Arducam IMX291 + M27195H15；`[-0.565, ±0.280, 0.6001] m` | 已覆盖，150° equisolid 与独立 CameraInfo 合同已登记 | 当前图像、标定和动态障碍场景证据缺失 | 仅作周边可视化/补充态势，不是深度、定位或安全权威；实物 Kannala-Brandt 标定仍缺 |
| 六轴机械臂 | A300 载荷平台—背板—pedestal—转接盘—UR5e；六个受控关节 | 六轴链、惯量、碰撞和控制器已覆盖 | 轨迹报告为未绑定历史证据；当前惯量/扫掠报告缺失 | 默认 DART 下必须重新证明整车稳定、六轴轨迹、底盘锁止、全链碰撞；连续全关节空间尚未验收 |
| 腕部双目/RGB-D | `tool0`—机加工侧支架—D435；RGB/Depth/双 IR/CameraInfo | 已覆盖且随末端运动 | 当前传感器、抓取和 20 块验收缺失 | 必须证明腕部近距重观测进入真实抓取控制，而不是只发布图像或使用 Gazebo 真值 |
| 2F-85 夹爪 | UR 转接件—Robotiq 2F-85，多连杆和五个 mimic 关节 | 几何/关节已覆盖；DART 缺失的原生 mimic 已由 `GripperMimicEffortSystem` 补齐受限动力学联动 | 单块抓投仅有未绑定历史证据；20 块门缺失 | 还缺当前 session 下的真实夹持力/电流、目标随末端抬升和双证据抓取复核；Gazebo 的原生 mimic 警告不再等同于功能阻断 |
| 干垃圾投放链 | 漏斗—XW540 执行器/输出盘—闸门—导槽—存在传感器—干箱开口 | 已覆盖，闸门控制器和物理接触话题已登记 | 清扫/存储执行器报告状态不再满足新合同；20 块报告缺失 | 需在稳定物理后端证明每块真实落入、不是穿模/attach 评分捷径，且箱满闭锁生效 |
| 干垃圾箱与质量增长 | 独立 40 L 可用干箱、盖、三件式手动锁扣、料位；纸板/PP/PET/铝实体质量 | 已覆盖；实体垃圾保留在箱内，由箱内监测读取实际惯量质量 | 单块结果未绑定；20 块逐块动态质量门缺失 | 必须验证 20 块逐块质量阶跃、材料质量、保持在箱内和不重复计入 aggregate payload；现阶段不能声称质量增长闭环通过 |
| 双侧刷 | 清扫升降小车两侧，两个 Pololu 4694 电机外形、两个编码器、两刷 joint | 已覆盖 | 电机电流/温度/负载门缺失；地污清扫仅有未绑定历史证据 | 需证明落地后真实旋转、扫掠覆盖、编码器量化、堵转/过温/失电保护 |
| 中央滚刷 | 升降小车中央螺旋滚刷，Pololu 电机外形和编码器 | 已覆盖 | 同上 | 需证明旋转方向、地面接触、实际覆盖清除和故障保护，不只是关节转动 |
| 清扫头升降 | `cleaning_lift_joint`，零位抬起，正向向下，工作位 0.1 m | 已覆盖且零位已改为运输安全位 | 执行器和地污/积水证据未在当前快照闭合 | 需验证抬起不穿地、落下形成刷/刮吸预载、再次抬起回弹，并受整车安全互锁控制 |
| 浮动刮条 | 刮条浮动和平摆两个被动 joint、弹簧阻尼包、接触和施力遥测 | 已覆盖 | 新合同要求的预载弹簧动态证据未通过 | 需用真实接触对证明自由态—预载—回弹；不能用名义零位代替 |
| 吸口、软管、过滤器、泵、流量监测 | 地面吸口—三段软管—过滤器—Jabsco HD4 泵外形/旋转转子—流量位—污水箱 | 已覆盖 | 水回收仅有未绑定历史证据；清扫电机门缺失 | 重跑接触、泵转速/电流、流量、滤堵保护、满箱零流量、质量守恒 |
| 污水箱与质量增长 | 独立湿箱，14 L 安装空间、8.30 L 有效容量、盖/锁扣、低高液位 | 已覆盖；WaterRecovery 向 DynamicPayload 发布污水质量并更新组合惯量 | 水回收单项报告未绑定最终 session，且其 9.42 kg 满箱证据与当前 8.30 kg 合同不一致 | 在当前容量上重跑地面水等质量转移、8.30 kg 满箱限流和排空后的惯量回退；现有 L1 2.5D 水模型不包含自由液面、飞溅和 CFD |
| 排污服务链 | 排污管—常闭球阀—阀球/阀杆—24 V 执行器—服务盖—软管接头 | 已覆盖 | 服务接口报告缺失，安全互锁证据未绑定 | 需证明只有驻车、软管接入、服务盖状态和许可满足时才能开阀，失电回零且排出质量守恒 |
| 电源/BMS | 两个 40 Ah 包、两个 BMS、主隔离器、接触器、熔断配电、隔离 DC/DC、安全继电器 | 已覆盖 | 电源灯光和整车互锁仅有未绑定历史证据 | BMS 是线性 OCV + 工程内阻的系统级模型，不是单体电芯/热失控模型；需重跑欠压、过流、充电、接触器和执行器 fail-closed |
| 急停、前后碰撞条 | 6 mm 急停柱塞；前后低位碰撞接触体 | 已覆盖 | 互锁证据未绑定；动态避障门失败 | 需当前快照证明急停锁存/复位、碰撞立即撤销底盘/机械臂/刷泵许可以及恢复顺序 |
| 充电接口 | 壳体、插座、铰接门、6 mm 锁销和接触检测 | 已覆盖 | 服务接口缺失，电源证据未绑定 | 需证明门/插头/锁销/充电请求的互锁和充电时牵引禁止 |
| S100P/征程 6P | 载荷平台内部 S100P 计算盒 | 位置和供电分支已覆盖；真板已连接并完成官方参考 BPU smoke | 项目模型、真实传感器、1800 秒与功耗/温度正式报告缺失 | PC 不能替代 S100P；DOSOD、EdgeSAM、ARM CPU 规划/抓取链仍须按项目模型与真实输入完成帧率、延迟、内存、温度和 BPU 进程映射验收 |
| 车身、检修门与灯光 | 产品外壳；四扇限位门和独立锁舌；工作灯、尾灯、四角警示灯；灯光 datum 显式映射到 `bodywork_lighting_link`，前后碰撞 datum 映射到 `bodywork_lower_tub_link` 的命名碰撞体 | 已覆盖 | 旧 15 视图不满足 v4 逐图绑定；检修门动态和惯量扫掠仍缺失 | 冻结后重跑 Ogre2 19 视图、门锁顺序和惯量扫掠 |

## 与用户方案的覆盖关系

已覆盖的核心配置是：单线 LiDAR + MID-360、前向 RGB-D、两个侧后鱼眼、六轴机械臂、末端双目/RGB-D、平行夹爪、双侧刷、中央滚刷、浮动刮吸、污水泵/过滤/流量、独立干湿箱、逐步增加的干垃圾/污水质量、A300 四轮底盘/编码器、S100P、BMS、急停、前后碰撞条、充电/排污/检修接口。

正式竞赛车已经明确取代早期 `0.60 m × 0.40 m` 麦克纳姆研发车，底盘改为开源资料较完整的 Clearpath A300 四轮滑移转向平台。早期文档中的“保留真实麦克纳姆动力学 + 虚拟 Ackermann”只适用于旧研发 profile，不能再用于描述当前正式整车。上层规划仍可施加非完整/曲率约束，但需要以 A300 实际可执行的滑移转向动力学重新验收。

## 尚未达到“无限接近实车”的内部细节

以下不是功能位置遗漏，但与用户要求的测量级数字孪生仍有明确距离：

1. UR5e 六轴目前采用公开外形、刚体惯量和关节，不包含六套电机转子/定子、谐波减速器或齿轮箱、制动器和电流/温升的独立刚体及电气模型。
2. 2F-85 采用公开连杆和项目内 `GripperMimicEffortSystem` 补齐 DART 的受限动力学联动，但仍不包含内部电机、齿轮/丝杠、柔性、夹持力闭环和磨损模型；因此现状是系统级可运动模型，不是内部传动数字孪生。
3. A300 有四电机/编码器实体和工程动力模型，但没有轮胎可变形、胎压、复杂地面剪切、传动间隙、轴承和电机热模型；A300 编码器 4096 counts/rev 是待实物替换的工程参数。
4. MID-360 是量程/FOV/频率近似，不是非重复扫描和厂家测量误差数字孪生；鱼眼、D435、GNSS、IMU 都还需要实物序列号级标定。
5. BMS、电源、线束和连接器是功能级模型；没有电芯级等效电路、热耦合、线束压降、接插件温升、防水和 EMC 认证模型。
6. 刷毛弯曲、磨损、地面颗粒运动以及水的自由液面/飞溅/气液两相流没有建模；当前积水方案是可守恒验收的 L1 2.5D 模型。

这些边界在只有开源模型、且不购买真实部件的条件下不能靠继续堆几何体消除；能做到的是把功能、外形、安装链、质量惯量、运动约束和可观测接口做到可验证，并将无法从公开资料获得的内部参数明确标为待实物标定，而不是虚构精度。

## 建议的关闭顺序

1. 先消除两个 source inventory 漂移，冻结源码，重生成 snapshot/URDF/部件台账，并把功能聚合更新到当前 38 位置合同；随后创建新的正式验收 session。
2. 保持 DART 作为整车物理后端，并在最终 session 重验 `GripperMimicEffortSystem` 的五个从动关节、双指接触和失电安全；底盘与夹爪共用后端的源码冲突已解除，剩余的是最终证据绑定。
3. 在同一 session 关闭 A300 空/满载直行、停车、转向、编码器/里程计、安全互锁、惯量/扫掠和产品/检修视觉；同时补齐检修门、充电、排污和辅助电源服务矩阵。
4. 关闭全部传感器运行与 FOV/遮挡，再完成 `200 m × 100 m` 首次建图、`>=95%` 有效观测、地图冻结和独立 AMCL saved-map 清扫硬重启。
5. 在 S100P 产品链直接关闭 DOSOD+EdgeSAM 的模型、adapter、ROS 图和性能/稳定性门；PC 只作对照。与此同时，Gazebo 必须用真实正式车相机话题重跑三个分离随机 episode，关闭真实相机感知和 map 投影后才允许进入动态避障、目标抓取和端到端任务。
6. 在同 saved-map、同 episode/seed 下先跑真实 FullCoverage 长跑基线，再跑冻结的 Q+系统覆盖双模式；纯 Python 4/4 结果只保留为算法先验，不作为 Nav2/Gazebo 通过。
7. 关闭 8 行人动态绕障和整车安全发布链；随后关闭清扫头升降、三刷电机、刮条预载、吸口/泵/过滤/流量，再在当前 `8.30 kg` 合同上重跑随机地污和积水守恒。
8. 关闭六轴轨迹、夹爪联动和单块目标相关抓投，再完成 20 块逐块抓投、材料质量阶跃、重复计数防护、箱满与行人进入机械臂安全区时的暂停/回撤。
9. 运行单一随机 episode：同一 Gazebo cleaning 进程同时完成真实感知、双模式 RL/Nav2、动态避障、20 块抓投/质量、18 个地污区、积水守恒、安全/服务与返航；不得拼接历史独立 JSON。
10. 聚合当前 session 的 38 个功能位置和全部 mission gates；S100P 的同版本模型、词表和 1800 秒 BPU 证据应在 session/snapshot 就绪后优先采集，并在聚合时复核其仍未漂移。S100、端到端和底层物理位置验收互相不能替代；真实执行器测试还必须满足 [S100P板端优先边界](s100p-board-first-execution-boundary.md)的安全硬门。
