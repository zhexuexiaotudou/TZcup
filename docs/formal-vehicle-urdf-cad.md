# 正式整车 URDF / CAD 设计

## 交付范围

正式入口为 `sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro`，
机器人名固定为 `tzcup_formal_sanitation_vehicle`。模型不再复用缩比占位车，按已冻结的
开源/公开型号和项目自研清扫机构进行多刚体装配：

| 子系统 | 正式模型 |
|---|---|
| 移动底盘 | Clearpath Husky A300 40Ah，四轮独立关节，底盘总质量 78.5 kg |
| 机械臂/夹爪 | UR5e 六轴链 + Robotiq 2F-85 多连杆 mimic 夹爪 |
| 定位与感知 | UTM-30LX 负责二维建图/固图定位；MID-360 负责三维障碍感知；另有前/腕 D435、双侧后鱼眼、ZED-F9P/ANN-MB-00、VN-100 |
| 清扫机构 | 双侧刷、中央滚刷、P16 升降、带约 12.5 N 名义预载的双自由度弹簧阻尼浮动刮条、吸口、三段软管、过滤器 |
| 污水回收 | Jabsco HD4 电机/泵头/减振座/接头、14 L 安装空间、8.30 L 满足最终整车载荷约束的独立污水箱有效容量 |
| 干垃圾 | 45 L 几何容积、40 L 可用容积独立干箱、机械臂投放漏斗/闸门/导槽及料位传感器 |
| 计算与控制 | UR e-Series 12 kg 控制箱、S100 参考外壳、熔断配电盒、90°物理主隔离器、主接触器、隔离 DC/DC 与硬接线安全继电器 |
| 产品车身 | 项目参数化连续曲面外壳、右前机械臂工作舱、检修门、灯组、保险杠、轮眉、刷盘护罩和功能接口 |

相机运行合同完整覆盖前向/腕部 D435 各自的 RGB、Depth、CameraInfo、左右红外
Image/CameraInfo（每台 7 个话题合同），以及左右后鱼眼各自的 Image、CameraInfo；
这些话题与 Gazebo bridge 的精确 ROS/GZ 类型均由整车部件台账校验，不能用单独一幅
图像替代完整标定输入。两台 D435 的左右红外光学帧使用 50 mm 物理基线。

最终正式快照展开为 196 个 link，其中 194 个物理 link 都有质量和正定惯量；
`base_footprint` 与 UR 官方坐标链中的 `ur5e_base_link` 是无质量数学帧。195 个 joint
形成单根树；活动关节有轴、力矩、
速度和范围。碰撞体采用稳定的简化几何；外观采用锁定 commit 且允许再分发的
A300/UR5e/2F-85/传感器 mesh，以及项目参数化生成的清扫、干湿分仓和安装件 CAD。
项目车身不是 primitive 外观：18 个车身 link 共使用 42 个项目自有 mesh visual，全部具有
碰撞体和正定惯量；`bodywork_visible:=false` 只隐藏外观用于检修教学。当前展开 URDF 合计
216 个 mesh visual 引用、169 个不同 mesh URI；传感器光学体、动态载荷状态体等仍保留
20 个有明确物理或显示用途的 primitive visual。上游来源、许可证和 194 项 SHA-256 清单
位于 `meshes/`，冻结 snapshot 另绑定 231 项源码/配置/资产输入与 5 个正式输出。
完整的开源复用与自研外壳取舍见[正式整车外观资产决策](formal-vehicle-bodywork-source-decision.md)。

## CAD 与安装坐标

`config/high_fidelity_vehicle/formal_vehicle_layout.yaml` 是整车安装基准，坐标统一以
`base_footprint` 表示。校验器会在零关节位姿下正向求解整棵树，并要求 42 个安装帧
在 5 mm / 0.02 rad 内与布局一致；腕部相机等动态设备另通过任务姿态的视场与扫掠门约束。

`sanitation_vehicle_description/cad/formal_vehicle/formal_vehicle_layout.scad` 提供毫米制
参数化包装模型，适合做安装、维修空间和爆炸图评审。其导出网格只负责外观，动力学、
碰撞和质量仍以 Xacro 为权威。确定性零位预览由下列命令直接从展开 URDF 生成：

```powershell
py -3 scripts/render_formal_vehicle_preview.py
```

产品外壳源为 `generate_product_bodywork_meshes.py`。它输出 loft 曲面、轮眉、检修门、
灯具和工作舱；Gazebo 正式截图由 `formal_vehicle_visual_acceptance.launch.py` 启动，
再用 `capture_formal_vehicle_visual_acceptance.py` 自动把机械臂送到视觉收纳候选、打开
垃圾投放闸门，并采集前左、后右、正俯、传感器塔、前相机、机械臂安装位、投放口、
清扫头、后部服务接口、配电/计算舱和干湿存储/回收链共十一张 1600×1000 Ogre2 图像。
成品与检修模式不再只依赖清单标签：启动器实际转发 `bodywork_visible`，截图器还会读取
运行中的 `/robot_description`，要求成品模式含不少于 40 个车身 mesh visual（当前快照为
52 个引用、42 个不同资产）、检修模式为 0。
顶部结构和各设备载荷路径见[正式整车部件与机械连接架构](formal-vehicle-component-architecture.md)。

## 动态载荷

干垃圾和污水分别位于 `dry_bin_payload_reserve_link` 与
`wastewater_payload_reserve_link`。`DynamicPayloadSystem` 在 Gazebo PreUpdate 阶段
更新质量、惯量和重心：干垃圾按箱内均布长方体处理；污水按密度 1000 kg/m³ 的液柱
处理，液位和重心随质量上升。输入和实际应用确认话题为：

```text
/model/tzcup_formal_sanitation_vehicle/payload/dry_mass_kg
/model/tzcup_formal_sanitation_vehicle/payload/dry_mass_kg/applied
/model/tzcup_formal_sanitation_vehicle/payload/wastewater_mass_kg
/model/tzcup_formal_sanitation_vehicle/payload/wastewater_mass_kg/applied
```

插件强制限制干垃圾 1.512 kg、污水 8.30 kg。当前 L1 水模型不计算自由液面晃动、
飞溅或 CFD；防晃板以显式几何保留，复杂流体属于后续增强而不是当前通过项。

## 构建与验证

提交到 `reports/engineering/` 的正式 URDF 是启用仿真插件的确定性快照，必须在已 source
ROS 的 WSL 环境中由唯一入口同步生成；不要手工编辑 URDF 或四份派生报告：

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/generate_formal_vehicle_snapshot.py
```

生成器从仓库根目录用相对 Xacro 路径展开 `use_sim:=true`，并通过
`controller_config_path:=package://sanitation_vehicle_description/config/formal_vehicle_controllers.yaml`
消除 ament 安装前缀造成的机器绝对路径。布局、URDF、产品外观、部件台账报告和
`formal_vehicle_snapshot_manifest.json` 只有在全部确定性门通过后才一并替换。manifest
记录权威 Xacro/配置/校验器的 SHA-256 以及五个提交产物的 SHA-256；Windows fast CI
仅执行以下纯 Python 一致性检查，不需要安装 ROS 或 Xacro：

```powershell
py -3 scripts/generate_formal_vehicle_snapshot.py --check
```

临时展开、调试无仿真插件的模型仍可使用下列 `use_sim:=false` 命令，但该文件不是正式
提交快照，也不能用来更新工程报告：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sanitation_vehicle_description sanitation_gazebo_control
xacro starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro \
  use_sim:=false -o /tmp/formal_vehicle.urdf
check_urdf /tmp/formal_vehicle.urdf
python scripts/validate_formal_vehicle_urdf.py --urdf /tmp/formal_vehicle.urdf
python scripts/validate_formal_vehicle_visual_fidelity.py --urdf /tmp/formal_vehicle.urdf
python scripts/validate_formal_vehicle_product_design.py --urdf /tmp/formal_vehicle.urdf
python scripts/validate_formal_vehicle_component_register.py --urdf /tmp/formal_vehicle.urdf
```

最终快照的确定性结果为 196 links、195 joints、160.007583 kg；两个动态载荷状态 link
各保留 0.001 kg 数值稳定质量，装载时由实际载荷质量替换。装入 1.512 kg 干垃圾和
8.30 kg 污水后的名义总质量为 169.819583 kg，A300 设计载荷仅剩约 0.030417 kg 余量。视觉门要求所有关键外露 link
使用 mesh；当前模型有 214 个 mesh visual，只有两个透明载荷状态体保留 primitive。
产品门另验证 18 个车身 link、42 个不同车身 mesh 资产、4 个检修面板/铰链/锁舌、
多色材料、碰撞和正定惯量。动态插件已在 Gazebo Harmonic 中加载；完整 ros2_control
运行仍要求环境安装 `gz_ros2_control`，且运行证据必须与当前快照哈希绑定。

正式控制器分组为 `joint_state_broadcaster`、底盘物理 plant，以及机械臂、夹爪、
清扫升降、储存、服务、刷盘和回收控制链。十九视图 Ogre2 工作室入口用于证明产品/
检修两种外观、视觉收纳候选、机械臂投放接口与内部功能链能真实渲染；静态交叉表覆盖
38 个功能位置、9 个传感器安装和 18 个机械子总成；产品外观检查 156 个物理 link，
检修外观移除车身 skin 后检查 142 个仍可见物理 link，并在实际采集关节姿态验证逐 link
相机投影。正式启动器使用 DART；DART 不原生建立 URDF mimic 约束，
因此项目内 `GripperMimicEffortSystem` 对 2F-85 五个从动关节施加有界动力学联动，
`/joint_states` 仍保留这些从动关节用于误差验收。

仓库中的旧无头运行报告和旧 Stage 证据仍可用于回归诊断，但当前最终快照的运行结论只由
同一次正式 acceptance session 内重新生成、哈希匹配的报告决定。在最终会话完成前，传感器
出数、十九视图、底盘运动、机械臂轨迹、清扫/服务关节和综合任务均不得因为历史报告写着
`PASSED` 就在本文宣称为当前通过。

正式传感器运行入口保留两套 D435、双后鱼眼、MID-360、UTM-30LX、GNSS、IMU 与编码器
的原始尺寸、帧率、坐标系和话题。为避免超过 1 GB/s 的未压缩数据在 WSL/DDS 中形成提交
内存高水位，高带宽 GZ→ROS 接口改为 lazy、`SENSOR_DATA`、双侧 queue=1；采集器只保留
元数据和有界源时间戳，并在单流证据充分后退订。该修改不属于降分辨率或降帧率，且完整
同时运行能力仍必须由新 runtime 的 Gazebo 门证明。

功能位置台账当前登记并校验 38 个位置及 63 个明确子部件，覆盖移动、定位感知、机械臂抓取、干垃圾
投放、干湿分仓、清扫/刮吸/过滤/泵送、计算配电、安全触边、急停、照明、充电和排污。
子部件门单独核对 A300 四电机、左右固定梁/垫块、两电池/两 BMS、充电壳体/插座/门/锁、
急停壳体/6 mm 柱塞，以及排污管/球阀/执行器/服务盖/接头的直接 joint 与零位 FK。
四扇车身检修门另逐件核对固定铰链座、限位门轴、门板和零位锁止旋转锁舌；腕部 D435
则强制采用 `tool0 -> 独立支架 -> 相机壳体 -> optical frame` 层级，禁止支架与相机共用 link。
腕部支架采用 236.709 g 后置折弯狗腿结构，所有承力件均位于 D435 后平面
`x <= -12.5 mm`，再从侧向连接工具法兰，不允许任何板件穿过深度相机锥形视场。
最终支架设计的完整 mesh-ray 历史复核中，预抓姿态有效视场从 49.0196078% 提升至
96.6946779%，3 cm 方块功能区从 0/9 提升至 9/9；支架仍作为遮挡实体参与射线计算。
服务门运行证据不能由 URDF 字符串或手工 JSON 关闭：
`scripts/run_formal_service_door_runtime.sh` 启动车辆后，以有限关节力执行“锁止拒动 →
解锁开门 → 解锁闭门 → 锁舌回零 → 再次拒动”，采集七阶段
`/formal/service_door_joint_states` 原始样本并交由
`validate_formal_service_door_runtime.py` 重算限位、方向、开度和锁止结果。产物
`artifacts/formal_service_door_runtime.json` 是 session-bound gate，并与惯量/扫掠门及部件台账
共同约束 `bodywork_service_access`；本轮源码编写不等于该 Gazebo 门已经通过。
排污球阀由 `service_controller` 对 `wastewater_drain_valve_joint` 进行位置控制，
`wastewater_drain_service_cap_joint` 才是被动服务互锁；台账会拒绝把电动球阀误登记为
被动关节，并要求明确的执行器 link。
地图语义固定为 UTM-30LX + slam_toolbox 的二维占据栅格；MID-360 点云仅用于三维障碍感知，
当前交付不包含三维 SLAM 或三维地图，文档和台账均禁止宣称“三维建图”。
清扫/存储/回收活动关节的正式自包含入口为
`scripts/run_formal_function_positions_runtime.sh`。它在独占 ROS domain/Gazebo partition 中
启动冻结 overlay，驱动升降、投放闸门、电动排污球阀、双侧刷、中央滚刷和回收泵，并把
`reports/engineering/formal_function_positions_runtime_report.json` 绑定到同一展开 URDF 哈希。
该证据只证明机构和控制链，不把污水流量或垃圾转移效果误写成已通过。

正式车前进/停车、20 块材料方块逐块双指接触抓取、实体落箱及动态质量闭合、有限积水
守恒回收和满箱闭锁必须在
新增服务硬件后的同一全新构建上重跑；机器可复现入口为
`scripts/run_integrated_functional_acceptance.sh`。每次运行先保留在以 `run_id` 唯一命名的目录，
再由 `publish_integrated_basic_functional_acceptance.py` 验证四个场景、结果哈希、所选真实材料
质量和冻结 URDF 哈希，原子发布合同固定摘要
`reports/engineering/integrated_basic_functional_acceptance_summary.json`。固定摘要不是另一份手工
证据，唯一 run manifest 才是其不可变证据来源。

## 尚未通过的高保真门

- S100 实际 SKU 的板框、孔位、连接器和质量仍需对用户自有板实测；
- 最终机构设计曾完成 1113 个机械臂采样姿态和运输/预抓/抓取/投箱任务锚点的载荷、重心、离地和碰撞扫描，任务锚点通过；该报告仍需回绑当前正式快照，且任意连续关节空间仍存在已记录的自碰撞/触地排除区，不能把采样通过写成全空间证明；
- L1 有限 2.5D 水层的历史门已通过刮条/吸口几何、守恒回收和满箱闭锁；当前快照仍须正式会话重跑。被动刮条已加入有界弹簧阻尼、约 12.5 N 名义预载、刀片接触传感器，以及模型施加的弹簧阻尼 effort/运动遥测；以 1.38 kg、1800 N/m 与 g=9.81 m/s² 计算的自由平衡为约 -14.441 mm，距既有 -15 mm 限位约 0.559 mm。该解析余量不是未贴 hard-stop 的证明，不放宽既有行程/限位门，下一次 fresh function-position 必须实测。后者不是实测关节反力或直接接地证据，直接刀片-地面接触只由正常积水回收的 ContactSystem 门判定。刷毛连续接触力、软管柔性、污水自由液面和飞溅仍需后续增强；
- MID-360/VN100/GNSS 外壳是开源 ROS 近似而非厂家计量 CAD；MID-360 扫描仍为密集栅格近似。双后鱼眼已使用 SDFormat `wideanglecamera` 的 `equidistant` 投影，静态审计同时核对该类型、投影、HFOV 缩放和截止角，但实物序列号级内参、畸变系数和曝光响应仍待标定；
- 清扫电机、升降器和泵已建外壳、法兰、轴、接口与运动链，回收泵转子已由独立连续关节驱动；但不声称隐藏绕组、齿轮或泵膜片达到制造级精确；
- 8 个传感器安装位的确定性网格射线遮挡、安装方向和功能区覆盖已有预最终历史通过；当前快照仍须正式会话重跑。真实镜头标定、Livox 非重复扫描模式和实物 GNSS 多路径仍是外部校准门。
- A300 载荷门在当前名义模型上只剩约 0.030417 kg 余量，工程裕度非常薄；任何后续线束、紧固件或实物质量增加都必须重新核算，不能据此宣称实车载荷已经验收。完整实物建造边界见 [正式整车实物建造就绪度](formal-vehicle-real-world-build-readiness.md)。

因此本交付是可构建、可校验、可继续做动力学闭环的正式名义整车，不宣称已经达到
购置实物后的测量级数字孪生或通过完整比赛运行门。
