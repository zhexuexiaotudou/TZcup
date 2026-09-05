# 正式整车部件与机械连接架构

## 顶部凸出结构是什么

顶部结构统一命名为“模块化传感器塔”，不是造型件，也不是一个未定义的
黑色盒子。它承担三类设备，并把载荷逐级传回 A300 载荷平台：

`payload_deck_link → sensor_mast_link → 独立设备支架 → 传感器 link`

承力塔由 `190 × 150 × 16 mm` 螺栓底座、两根 `30 × 30 mm` 立柱、
`36 × 62 mm` 线缆服务脊、三道横撑、四个底座螺栓和角部加强筋组成。
黑色圆角件是非承力检修罩；其右侧独立小盖板表示线缆检修口。

| 从下到上 | 确定型号/功能 | 机械连接 | `base_footprint` 坐标 |
| --- | --- | --- | --- |
| 前侧蓝色设备 | Hokuyo UTM-30LX；二维占据栅格建图、固图定位与平面避障 | 前悬臂板、减振垫、两侧夹具 | `[0.535, 0, 1.1621] m` |
| 顶部深灰设备 | Livox MID-360；三维障碍物点云感知，不承担现有建图 | 直径 140 mm 散热板、48 × 36 mm 四点 M3 隔振柱 | `[0.420, 0, 1.2731] m` |
| 侧面白色设备 | u-blox ANN-MB GNSS 天线 | 侧置悬臂、直径 150 mm 接地板、紧固件 | `[0.365, 0.160, 1.1801] m` |
| 塔体右后服务件 | u-blox ZED-F9P-04B 接收机 | 开口式防溅托架、17 × 22 × 2.4 mm模块、同轴与供电接口；通过线缆连接ANN-MB | `[0.360, -0.095, 0.9801] m` |

MID-360 保持最高且无遮挡；GNSS 被放到侧下方，避免进入 MID-360 的
`-7°～+52°` 垂直扫描范围；UTM 后方盲区朝向承力塔，塔柱不占用其主要扫描区。
ZED-F9P接收机与ANN-MB天线是两个明确部件：接收机固定在塔侧检修托架中，
天线通过远置同轴连接保持开阔天空视野；Gazebo定位测量坐标仍位于天线相位中心，
不能把天线外壳误当作接收机电子模块。

现有地图流程是 `UTM-30LX LaserScan → slam_toolbox → 二维占据栅格`。
MID-360 的 `PointCloud2` 只进入三维障碍物感知/避障语义；当前没有三维 SLAM、
三维体素地图或点云地图闭环，因此不得把现有流程称为“三维建图”。

## 其它传感器

| 设备 | 安装链 | 可见性与仿真接口 |
| --- | --- | --- |
| 前向 Intel D435 | 前骨架 → 四边框凹入式基座 → D435 | 外露镜头，向下俯 25°；RGBD 30 Hz |
| 左/右后鱼眼 | 后车身骨架 → 密封楔形相机舱 → 相机 | 外露镜头，各自独立 30 Hz 图像话题 |
| VectorNav VN-100 | 底盘 → 内部水平托盘 → 四个隔振点 → IMU | 正常外观不可见，检修模式可见；IMU 200 Hz |
| 腕部 Intel D435 | UR `tool0` → 独立 `wrist_rgbd_mount_link` 机加工侧支架 → 独立 `wrist_rgbd_link` 相机壳体 → optical frame | 支架、相机本体和坐标帧不混用；随末端运动；RGBD 30 Hz |

正式接口不是“有图像即可”：前向和腕部 D435 都要求 RGB、Depth、CameraInfo，
以及 50 mm 基线的左右红外 Image/CameraInfo，每台共 7 个话题合同；左右后鱼眼
分别要求 `image_raw` 和 `camera_info`。基础运行门会检查全部 CameraInfo 的非零
分辨率和正的 `fx/fy`；任何 RGB、深度、红外或内参流缺失都按失败处理。
左右后鱼眼直接使用 SDFormat `wideanglecamera` 的 `equidistant` 镜头投影和 150°
水平视场，不再依赖仓库中并不存在的 ROS 畸变后处理节点；实物序列号级内参仍需标定。

四个橙色凸起是低矮 LED 警示灯，不是雷达；红色凸起是急停；前后灯具也不计入
环境传感器。完整机器可读台账位于
`config/high_fidelity_vehicle/formal_vehicle_component_register.yaml`。

## 机械臂与夹爪

机械链为：

`A300 载荷平台 → 280 × 220 mm 背板 → 加强 pedestal → 六螺栓转接盘 → UR5e → UR tool0 → Robotiq 转接件 → 2F-85 → 腕部 D435`

机械臂和夹爪使用独立控制器：六轴由 `arm_controller` 控制，夹爪主关节由
`gripper_controller` 控制，二者都禁止部分关节目标，并设置轨迹/终点误差门。
正式整车 Gazebo 启动固定使用 DART。DART 不原生建立 URDF mimic 约束，因此
2F-85 的五个从动关节由项目内 `GripperMimicEffortSystem` 读取主关节状态并施加
有界动力学联动；夹爪仍由真实关节力、接触和 `/joint_states` 证明运动，不能把
Gazebo 的原生 mimic 警告解释成“夹爪未建模”，也不能退回 Bullet Featherstone
来规避整车稳定性问题。

## 编码器实体与量化反馈

A300四个驱动电机的内侧均有独立的`*_encoder_link`、固定安装关节、碰撞体和
质量分配，不再把编码器含糊地藏在电机mesh里。清扫机构的左右侧刷和中央滚刷
也各有独立Pololu编码器帽。统一只读节点订阅`/joint_states`，发布四轮和三刷的
整数计数及严格由计数差分得到的量化`JointState`；它不写关节命令，也不伪造
编码器测得的effort。

Pololu 4694按厂家`64 CPR`与名义`70:1`得到`4480 counts/output rev`。A300公开
资料没有给出编码器型号、封装CAD或分辨率，因此仿真使用`4096 counts/wheel rev`
的工程量化参数并明确标记为非Clearpath规格；实车相关验收前必须识别实际型号、
测量分辨率/极性并完成轮径和轮距标定。

运行态验收不再采用“发一次 topic 后等待”的方式，而是等待
`FollowJointTrajectory` Action 的接受和成功结果，同时从 `/joint_states` 计算每个
关节的实际运动范围及终点误差。报告位于
`reports/engineering/formal_manipulator_runtime_report.json`。正式入口是
`scripts/run_formal_manipulator_trajectory_runtime.sh`：它只接受显式冻结的 colcon overlay，
独占 ROS domain/Gazebo partition，启动单一整车实例，并把报告绑定到冻结快照的展开 URDF
哈希；旧报告或手工执行 validator 不能关闭 session-bound 门。

## 清扫、投放与污水回收链

离散垃圾不是直接穿过车身进入垃圾箱。机械臂投放链固定为：

`夹爪释放 → 车顶投放漏斗 → XW540 防水伺服/同轴输出盘 → dry_deposit_gate_joint 闸门 → 导槽 → 干垃圾箱开孔`

漏斗、XW540执行器、输出盘、闸门、导槽和投放存在传感器都有独立 link；闸门由 `storage_controller`
控制，接触话题为 `/storage/dry_deposit/contact`。后车身碰撞体在相同位置留出实体开口，
不会再由一整块碰撞盒把投放路径封死。

干垃圾箱和污水箱的维护盖均由真实铰链关节连接，并各自补齐三件式手动过中心锁扣：
箱体固定底座、可绕横向销轴转动 70° 的手柄，以及固定在箱盖上的 keeper。箱盖和锁扣
手柄均只输出关节状态，不提供电动命令接口；零位代表箱盖关闭且锁扣锁止，维护顺序固定为
先释放锁扣、再抬起箱盖。URDF 树不能表达锁钩与 keeper 的封闭运动链，因此不使用虚假的
mimic 关节冒充锁紧力；该限制保留为实物锁扣刚度和密封压缩量标定边界。湿箱排空继续复用
独立的排污管、常闭球阀、可拆服务盖和软管接头，不新增不存在的电动翻箱能力。

地面清扫与积水回收链固定为：

`双侧刷/中央滚刷 → 浮动刮条 → 吸口 → 三段回收软管 → 过滤器 → 泵头/旋转泵转子 → 流量检测位置 → 污水箱入口`

升降、三把刷和回收泵由 ros2_control 驱动；刮条俯仰/浮动是被动机构，不提供命令
接口，由 Gazebo 中唯一的弹簧阻尼插件向真实关节施加有界力/力矩。刀片接触话题为
`/cleaning/squeegee/contact`，吸口接触话题为 `/cleaning/suction_nozzle/contact`；泵转子
使用独立连续关节，不再以固定装饰件代表运行。正式 function-position 验收按“抬起
自由态→落地预载接触→再次抬起回弹”测量两关节状态、刀片—地面碰撞对以及实际施加
的力/力矩，不能用 Xacro token 或名义零位代替接触证据。
当前 L1 水模型把 2.88 L 地面积水离散为 24 列有限 2.5D 水单元；只有刷盘、刮条、
吸口、泵和容量门同时成立时才将地面体积等质量转移到污水箱。历史证据达到 24/24 列、
100% 回收和零质量误差；新增服务硬件后有效容量改为 8.30 kg，最终满箱后余水保留、
零流量和停止扣减需在当前冻结快照上重跑确认。
该结果不是粒子流体、飞溅、自由液面晃动或 CFD 证据。

## 电气与安全位置

载荷平台内部显式装配 S100 计算盒、UR 控制箱、熔断配电盒、隔离 DC/DC 和硬接线
安全继电器。A300 底盘内部的左右 40 Ah 电池包与各自 BMS 均为独立 link；四个驱动
电机、左右固定承载梁和垫块也分别登记，不再只由一个 `mobility` 标签代指。

外部充电接口由壳体、插座、铰接门和 6 mm 行程锁销组成；急停由黄色壳体和明确
`0.006 m` 行程的红色柱塞组成。低位排污链逐件登记排污管、球阀阀体、球芯/阀杆、
执行器、服务盖和软管接头。四扇车身检修门也不是固定贴片：每扇均有底盘固定铰链座、
限位转轴和独立旋转锁舌，锁舌零位代表运输锁止，解锁后才允许在约 100° 机械范围内开门。
铰链与锁舌均作为只读状态关节进入 `/joint_states`，不虚构电动门执行器。
机器可读台账对 38 个功能位置和其中明确登记的子部件同时
检查 link、直接连接 joint、载荷祖先、零位 FK、可见性、控制器和必需话题，缺一项即失败。
正式整车启动默认不暴露服务门 evaluator bridge；仅
`run_formal_service_door_runtime.sh` 显式设置
`service_door_evaluation_interfaces:=true`，再通过独立 evaluator 话题驱动
`ServiceDoorSystem` 的有限 PD 力；插件只写真实关节力，不直接重置位姿。每个仿真秒
还会写出 `SERVICE_DOOR_DIAGNOSTIC`，包含 bridge 已送达的目标计数/值、互锁后的有效
目标、实测位置、力矩、force-write 计数及 `PostUpdate` 对 `JointForceCmd` 的只读回显，供
failed run 区分消息、互锁与动力学链路。该回显没有 ECM writer identity，且物理引擎可能
已消费或清零命令；它只能定位下一步，不能单独证明最终写者或物理受力。
采集器从
`/joint_states` 记录七阶段原始样本：运输锁止、锁止拒绝开门、解锁、开门、解锁闭门、
回零锁止和再次拒绝开门。`artifacts/formal_service_door_runtime.json` 只有在四门均按正确
方向打开至少 0.9 rad、全程未越限、锁舌回零后门轴再次拒动，且证据绑定当前正式
snapshot/session 时才关闭 `bodywork_service_access`。
其中球阀不是被动铰链：`wastewater_drain_valve_joint` 是 24 V 常闭执行器驱动的
位置关节，由 `service_controller` 在 `0～π/2 rad` 范围内控制；可拆服务盖仍是
独立被动互锁关节。失电、超时或安全许可撤销时，产品安全链必须将阀位命令归零。

## 最终源码冻结与证据绑定

`reports/engineering/formal_vehicle_snapshot_manifest.json` 是整车结构、网格和接口的
唯一冻结清单。最终运行验收开始前，必须执行
`python3 scripts/formal_acceptance_session.py start`；该命令拒绝覆盖已有 session，旧文件须先
转存，避免新旧证据共享时间边界。之后生成的部件台账、传感器、底盘、
互锁、灯光、20 块实体抓投与逐块动态质量、地面清扫、积水、服务、建图、随机感知、
动态避障和跨地图 RL 报告，只有在
`finalize` 后路径、状态与 SHA-256 均匹配本次会话时才有效。旧车型上的历史通过
报告会显示为 `unbound`，源码或展开 URDF 漂移会显示为 `stale`，均不能关闭任何
功能位置。S100 门仍只能由真实 RDK S100P/征程 6P 板端运行产生；它必须晚于本次 session
开始，并携带与冻结 snapshot 相同的 source-inventory SHA-256，本机不能生成替代证据。
传感器视场/自遮挡报告和整车惯量、重心、碰撞扫掠报告也属于同一会话，并分别把
`urdf_sha256` 与 `inputs.expanded_urdf_sha256` 回绑到冻结快照中的展开 URDF 散列。
因此改动支架、门铰链、执行器或质量参数后，即使旧报告仍写着通过，也必须重新扫描。
惯量扫描保持 1024 个 Halton 样本、64 个关节极限角点和生产动作锚点不变，但只用固定
50 项小顶堆保留报告所需的最严重碰撞候选，避免把全部事件字典常驻内存；正式报告同时
绑定 URDF、布局和扫描器 SHA-256。FOV 扫描同样保持全部射线域和阈值不变，按 transport、
pregrasp、pick、deposit 逐姿态构建网格/BVH、汇总后释放，不再同时保留四套几何。
功能验收合同的 `current_file_hashes` 会在 session 绑定之前重新计算布局、惯量扫描器和
FOV 校验器散列；任一实现漂移都会把报告标为 `stale`，不能只凭历史 `PASSED` 状态过门。
正式 runtime runner 的默认 DDS domain 均位于 `0..232`，多场景 runner 会在启动前检查
整个连续 domain 区间；每次 Gazebo 使用唯一 `GZ_PARTITION` 和独立进程组，退出时依次
INT/TERM/KILL、wait，并按 partition 清除遗留进程。runner 不复用旧 episode/raw 目录；
地面脏污、积水、服务接口、地图生命周期、随机感知和端到端任务仅在 fresh run 成功后
写入合同规定的最终路径。首次建图和硬重启清扫统一使用
`.work/formal_first_map_acceptance`，mapping-only 的失败关闭边界只保存在 map root 内，
不会提前占用正式 lifecycle 报告。

## 真实度边界

当前最终快照的静态证据可证明公开网格、刚体质量/惯量、关节约束、控制器和传感器
接口完整；旧快照上的 Gazebo 动力学、前进/停车、单方块抓投、有限 2.5D 积水守恒回收
和满箱闭锁仅作为历史回归证据，必须在当前正式会话中重跑后才能关闭对应位置。它不冒充
以下尚未具备实物参数的内容：UR 电机电流/温升和制动器
内部模型、线缆寿命、制造公差、整车防水认证、MoveIt 全空间自碰撞扫描以及实车标定。
这些边界不影响本轮“部件可识别、安装链明确、清扫/存储/回收机构及六轴夹爪能实际运动”的验收。

## 最终物理细节台账对账

前向与腕部 D435 的左右红外光学 frame、50 mm 基线和合计八路红外图像/标定话题已分别
绑定；连同 RGB、Depth、CameraInfo，每台 D435 共 7 个话题合同，并关联
`forward_perception`、`grasp_observation` 与 `sensor_runtime`。主隔离器壳体/手柄和主接触器
壳体/衔铁的实体 link/joint、命令及测量反馈已绑定原有 `fused_power_distribution` 与
`auxiliary_power_lighting`；0.4 kg 由原配电盒质量中等量拆分，整车总质量不变。刮吸弹簧包、
浮动/俯仰状态及真实施力遥测已绑定原有 `water_gathering` 与 `cleaning_actuators`，不新造
功能 ID，也不以名义参数替代运行证据。当前正式展开 URDF 为 196 links、195 joints、
160.007583 kg，当前 SHA-256 为
`d553a07367bc44980cb38bb3396ad0b1fc5d396949173a3c87576deb21397128`。19 视角静态门
现在逐项覆盖 38 个功能位置、9 个传感器安装和 18 个机械子总成，并在折叠机械臂、
夹爪张开、投放闸门开启的实际采集姿态下，产品外观对 156 个有 visual/collision 的物理
link 执行相机视锥投影（110 个直接来自机器可读部件台账）；检修外观主动移除车身 skin
后仍对 142 个可见物理 link 投影（96 个来自台账），不能拿本来就被隐藏的车壳冒充检修内容。
相机只对准一个名义 datum、
部件 link 没有碰撞体、或把部件分配给拍不到它的相机，均不能生成通过 manifest。
所有运行证据仍须
在主流程统一重生成 snapshot manifest 后，于同一正式会话内重新生成并回绑最终哈希。
