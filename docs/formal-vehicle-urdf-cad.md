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

103 个物理 link 都有质量和正定惯量；`base_footprint` 是唯一无质量的 REP-105 虚拟根帧。
103 个 joint 形成单根树，活动关节有轴、力矩、速度和范围。碰撞体采用稳定的凸基本体，外观几何由项目自有 Xacro 与 OpenSCAD 参数化
源描述，避免依赖不可再分发的厂商网格。

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
```

空载确定性结果为 104 links、103 joints、134.252001 kg（其中两个动态载荷保留 link
各含 0.001 kg 数值稳定质量）。装入 1.512 kg 干垃圾和 9.7064 kg 污水后为
145.468401 kg。动态插件已在 Gazebo Harmonic 中加载并验证上下限
确认话题；完整 ros2_control 运行需要环境安装 `gz_ros2_control`。

`reports/engineering/formal_vehicle_runtime_report.json` 记录本次无界面运行证据：2D/3D
雷达、前/腕 RGB-D、双鱼眼、GNSS、IMU 和动态载荷桥接均已观察到真实消息；六个控制器
全部 active，底盘 0.25 m/s 指令产生 0.344 m 地面真值位移，刷盘、机械臂主关节和夹爪
驱动关节均有实测响应。正式启动器使用 Gazebo 官方 mimic 示例要求的 Bullet Featherstone
引擎，四个夹爪联动关节均观察到对应响应。Gazebo 服务端仍在 SIGINT 后需要 SIGTERM
才退出，因此干净停机仍明确标为未通过。

## 尚未通过的高保真门

- S100 实际 SKU 的板框、孔位、连接器和质量仍需对用户自有板实测；
- 最终整车重心扫描和由重心确定的污水最终容量尚未冻结；
- UR5e/夹爪精确网格全关节扫掠、自碰撞和投箱轨迹尚未运行；
- 刷毛、刮条地面接触、软管柔性、污水自由液面仍需 Gazebo 调参与验证；
- MID-360 当前为密集栅格扫描近似，鱼眼镜头畸变由标定层处理；
- 精确视场遮挡和全部 Gazebo 传感器可见性扫描尚未完成。

因此本交付是可构建、可校验、可继续做动力学闭环的正式名义整车，不宣称已经达到
购置实物后的测量级数字孪生或通过完整比赛运行门。
