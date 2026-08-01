# 清扫车模型部件说明

车辆前进方向为 `base_link` 的 `+X`，左侧为 `+Y`，车顶为 `+Z`。本次只增强可视外观，冻结的二维 footprint、轮距、车轮动力学、传感器外参、碰撞包络和 ROS 话题没有改变。

| 区域 | URDF 部件 | 作用 |
|---|---|---|
| 底盘 | `base_footprint` / `base_link`、`lower_chassis_visual` | 坐标基准和承载结构；底盘碰撞体决定真实物理外形 |
| 上车体 | `upper_body_visual` | 电池、电控和水路设备舱外罩 |
| 前检修面板 | `front_service_panel_visual` | 前部检修和识别面板 |
| 前后防护 | `front_bumper_visual`、`rear_suction_intake_visual` | 前部防撞外观和后部吸尘入口 |
| 行车灯 | 前灯、尾灯和左右 marker visuals | 前照、尾灯和示宽/转向提示 |
| 安全灯 | `roof_safety_beacon_visual` | 车顶琥珀色作业警示灯 |
| 检修门 | 左右 access door 与 handle visuals | 电池、电控、水路设备维护入口 |
| 充电口 | `charging_port_visual` | 右侧充电接口外观标识 |
| 四轮底盘 | 四个 `wheel_link` | 4WD 差速/滑移转向；轮胎负责接地，轮毂显示转动方向 |
| 激光雷达 | `laser`、雷达外壳/顶盖/桅杆 | 360° 二维测距与导航障碍检测 |
| 前置 RGB-D | `camera_link` / `camera_depth_link` | 前方彩色、深度和点云输入 |
| 验证相机 | `verification_camera_link`（按工程配置启用） | 独立复核视角，不属于默认生产配置 |
| IMU | `imu_link` | 车体角速度与线加速度测量 |
| 40 L 尘箱 | `dust_bin_link` | 0.50 m × 0.40 m × 0.20 m，含箱盖和状态条，容积 0.040 m³ |
| 双侧刷 | `left_brush_link` / `right_brush_link` | 把道路垃圾汇聚到吸口；支臂显示安装关系 |
| 清扫宽度 | `cleaning_footprint_link` | 半透明、无碰撞的 0.65 m 名义清扫带，仅用于可视化 |
| 机械臂预留 | `arm_mount_link` | 后续定点拾取机械臂安装坐标，默认不启用 |

模型仍完全使用可离线分发的 URDF/Xacro 基础几何，没有引入未知许可的外部网格。刷盘和清扫带的可视对象不是覆盖率真值；覆盖率仍由 Gazebo ground-truth 位姿、刷盘状态和任务几何在评估链中计算。
