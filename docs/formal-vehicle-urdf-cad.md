# 正式整车 URDF / CAD 设计

## 交付范围

正式入口为 `sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro`，
机器人名固定为 `tzcup_formal_sanitation_vehicle`。模型不再复用缩比占位车，按已冻结的
开源/公开型号和项目自研清扫机构进行多刚体装配：

| 子系统 | 正式模型 |
|---|---|
| 移动底盘 | Clearpath Husky A300 40Ah，四轮独立关节，底盘总质量 78.5 kg |
| 机械臂/夹爪 | UR5e 六轴链 + Robotiq 2F-85 多连杆 mimic 夹爪 |
| 定位与感知 | UTM-30LX、MID-360、前/腕 D435、双侧后鱼眼、ZED-F9P/ANN-MB-00、VN-100 |
| 清扫机构 | 双侧刷、中央滚刷、P16 升降、双自由度浮动刮条、吸口、三段软管、过滤器 |
| 污水回收 | Jabsco HD4 电机/泵头/减振座/接头、14 L 安装空间独立污水箱 |
| 干垃圾 | 45 L 几何容积、40 L 可用容积独立干箱及料位传感器 |
| 计算与控制 | UR e-Series 12 kg 控制箱、S100 参考外壳与独立板卡基准 |
| 产品车身 | 项目参数化连续曲面外壳、右前机械臂工作舱、检修门、灯组、保险杠、轮眉、刷盘护罩和功能接口 |

113 个物理 link 都有质量和正定惯量；`base_footprint` 与 UR 官方坐标链中的
`ur5e_base_link` 是无质量数学帧。114 个 joint 形成单根树，活动关节有轴、力矩、
速度和范围。碰撞体采用稳定的简化几何；外观采用锁定 commit 且允许再分发的
A300/UR5e/2F-85/传感器 mesh，以及项目参数化生成的清扫、干湿分仓和安装件 CAD。
项目车身不是 primitive 外观：正常模式引用 42 个独立车身 mesh，`bodywork_visible:=false`
只隐藏外观用于检修教学。上游来源、许可证和 148 项 SHA-256 清单位于 `meshes/`。
完整的开源复用与自研外壳取舍见[正式整车外观资产决策](formal-vehicle-bodywork-source-decision.md)。

## CAD 与安装坐标

`config/high_fidelity_vehicle/formal_vehicle_layout.yaml` 是整车安装基准，坐标统一以
`base_footprint` 表示。校验器会在零关节位姿下正向求解整棵树，并要求 22 个非 pending
安装帧在 5 mm / 0.02 rad 内与布局一致；腕部相机的两个动态帧保留为机械臂扫掠门。

`sanitation_vehicle_description/cad/formal_vehicle/formal_vehicle_layout.scad` 提供毫米制
参数化包装模型，适合做安装、维修空间和爆炸图评审。其导出网格只负责外观，动力学、
碰撞和质量仍以 Xacro 为权威。确定性零位预览由下列命令直接从展开 URDF 生成：

```powershell
py -3 scripts/render_formal_vehicle_preview.py
```

产品外壳源为 `generate_product_bodywork_meshes.py`。它输出 loft 曲面、轮眉、检修门、
灯具和工作舱；Gazebo 正式截图由 `formal_vehicle_visual_acceptance.launch.py` 启动，
再用 `capture_formal_vehicle_visual_acceptance.py` 自动把机械臂送到视觉收纳候选并采集
前左、后右和正俯三张 1600×1000 Ogre2 图像。

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

插件强制限制干垃圾 1.512 kg、污水 9.7064 kg。当前 L1 水模型不计算自由液面晃动、
飞溅或 CFD；防晃板以显式几何保留，复杂流体属于后续增强而不是当前通过项。

## 构建与验证

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sanitation_vehicle_description sanitation_gazebo_control
xacro starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro \
  use_sim:=false -o /tmp/formal_vehicle.urdf
check_urdf /tmp/formal_vehicle.urdf
python scripts/validate_formal_vehicle_urdf.py --urdf /tmp/formal_vehicle.urdf
python scripts/validate_formal_vehicle_visual_fidelity.py --urdf /tmp/formal_vehicle.urdf
python scripts/validate_formal_vehicle_product_design.py --urdf /tmp/formal_vehicle.urdf
```

空载确定性结果为 115 links、114 joints、150.462866 kg（其中两个动态载荷保留 link
各含 0.001 kg 数值稳定质量）。装入 1.512 kg 干垃圾和 9.7064 kg 污水后为
161.681266 kg。视觉门要求 61 个关键外露 link 全部使用 mesh；当前展开结果有
142 个 mesh visual，仅两个透明载荷状态体保留 primitive。产品门另验证 10 个车身 link、
42 个正式车身 mesh、4 个检修面板、多色材料、碰撞和正定惯量。动态插件已在 Gazebo Harmonic 中加载并验证上下限
确认话题；完整 ros2_control 运行需要环境安装 `gz_ros2_control`。

`reports/engineering/formal_vehicle_runtime_report.json` 记录本次无界面运行证据：2D/3D
雷达、前/腕 RGB-D、双鱼眼、GNSS、IMU 和动态载荷桥接均已观察到真实消息；六个控制器
通过单次分组启动全部 active。V3 的三相机 Ogre2 工作室验收已证明产品外壳和视觉收纳候选能真实渲染；底盘累计 4 s 的
0.25 m/s 指令产生 0.537127 m 地面真值位移，刷盘、机械臂主关节和夹爪
驱动关节均有实测响应。正式启动器使用 Gazebo 官方 mimic 示例要求的 Bullet Featherstone
引擎，四个夹爪联动关节及左右指尖位置变化均观察到对应响应。Gazebo 服务端在 SIGINT、
SIGTERM 后仍需对本次任务的精确 PID 执行 SIGKILL，因此干净停机仍明确标为未通过。
此外，项目通用 Stage 1 两轮均为 629 tests、0 errors、0 failures，Stage 2 兼容回归收到
12/12 类必需话题并在 5 s 内产生 1.200000 m 位移；它验证旧运行链未回归，不替代上述
正式 mesh 整车的专用 Gazebo 证据。

## 尚未通过的高保真门

- S100 实际 SKU 的板框、孔位、连接器和质量仍需对用户自有板实测；
- 最终整车重心扫描和由重心确定的污水最终容量尚未冻结；
- UR5e/夹爪官方网格的全关节扫掠、自碰撞和投箱轨迹尚未运行；
- 刷毛、刮条地面接触、软管柔性、污水自由液面仍需 Gazebo 调参与验证；
- MID-360/VN100/GNSS 外壳是开源 ROS 近似而非厂家计量 CAD；MID-360 扫描为密集栅格近似，鱼眼镜头畸变由标定层处理；
- 清扫电机、升降器和泵已建外壳、法兰、轴、接口与运动链，但不声称隐藏绕组、齿轮或泵膜片达到制造级精确；
- 精确视场遮挡和全部 Gazebo 传感器可见性扫描尚未完成。
- 150.462866 kg 空载值暴露了前期 A300 包装预算不足；载荷、重心和底盘选型必须重新做正式工程门，当前不声称 A300 实车可安全承载。

因此本交付是可构建、可校验、可继续做动力学闭环的正式名义整车，不宣称已经达到
购置实物后的测量级数字孪生或通过完整比赛运行门。
