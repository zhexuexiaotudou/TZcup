# Gazebo 数字孪生场景与车辆模型

## 目标与启动

本场景把 `sanitation_structured_world.sdf` 从工程方盒测试场改造成可直接在 Gazebo
中理解的园区道路：保留已经冻结的定位、导航和 Coverage 几何，同时增加道路标线、
斑马线、路缘、人行道、绿化带、建筑立面、树木、路灯、长椅、垃圾桶、纸箱、行人
外观以及瓶、罐、纸盒、落叶和积水等清扫目标。它不依赖浏览器监督台。

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/sanitation_ws/install/setup.bash"
ros2 launch sanitation_bringup gazebo_scene.launch.py
```

上面的入口只启动场景和车辆。要在同一个 Gazebo 窗口观看自动清扫的完整过程：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_gazebo_cleaning_demo.ps1
```

专用入口默认加载单独制作的 `16 m × 12 m` 竞赛功能演示世界，而不是在大地图中限制一块区域。
场内包含专用作业面、充电位、路缘、护栏、行人假人、积水以及瓶、罐、纸张、纸箱和落叶五类
目标；Gazebo 同窗用橙色大框表示 30 m² 全部外任务区，用青色小框与半透明填充表示 12 m²
实际清扫区，并显示车辆出发点、Coverage 路径、实际轨迹、已清扫栅格、目标状态、
面积、覆盖率和效率。俯视镜头可同时看到车辆与完整作业区，不会打开浏览器控制台或 RViz。
全局黑色网格已关闭；覆盖率只统计青色区域，主路径结束后低于 `99.5%` 会按未覆盖栅格自动补扫。
增加 `-FullArea` 会切换到中型 `80 m × 50 m` 场景的原 17 组件任务。

小场清扫目标按 `1.15 m × 0.72 m` 车辆外形尺度设计，不再为了远距离可见性放大：

| 目标 | Gazebo 视觉尺寸 | 语义 |
|---|---:|---|
| 500 mL 级塑料瓶 | 直径 `0.066 m`，瓶身长 `0.22 m` | 横躺在地面 |
| 饮料罐 | 直径 `0.066 m`，长 `0.115 m` | 横躺在地面 |
| 纸张 | `0.21 m × 0.15 m × 0.004 m` | 约 A5 纸张 |
| 小纸盒 | `0.24 m × 0.17 m × 0.06 m` | 压扁包装盒 |
| 落叶 | 四片 `0.08–0.11 m` 薄片 | 不再使用球体冒充叶堆 |
| 积水 | 三个半径 `0.12–0.18 m` 的超薄不规则水斑 | 总外形约 `0.50 m × 0.39 m` |

起点与回库标记也改为 `0.14 m` 的贴地薄片，不使用悬浮大球，避免被误认为清扫物。

可用键盘控制验证车辆运动：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 场景语义

| 类别 | Gazebo 对象 | 说明 |
|---|---|---|
| 道路与边界 | `asphalt_ground`、`north_sidewalk`、`south_sidewalk`、`structured_curb_south` | 沥青路面、抬高人行道和实体路缘 |
| 人类可读标线 | `campus_road_markings`、`campus_crosswalk_west` | 车道边线、中心虚线和斑马线 |
| 绿化与参照物 | `campus_green_verges`、`structured_tree_*`、`campus_tree_*`、`structured_lamp_*`、`campus_bench_north` | 用于尺度、方向和道路语义识别 |
| 静态障碍 | `trash_bin_obstacle`、`cardboard_box_obstacle`、`structured_waste_bin`、`campus_safety_cones` | 具有碰撞几何，不伪装成感知结果 |
| 动态障碍 | `dynamic_pedestrian_box` | 具有人形外观，仍由既有 `SetEntityPose` 动态测试链移动 |
| 清扫目标 | `trash_bottle_01`、`trash_can_01`、`trash_paper_01`、`leaf_pile_01`、`puddle_zone` | 保留原对象名、真值注册和评测语义 |

`dynamic_pedestrian_box` 的动态行为沿用 `scripts/stage4w_dynamic_ci.sh` 和
`scripts/gz_set_dynamic_obstacle.sh`，不会用预制动画冒充避障闭环。

## 车辆外观与几何

`sanitation_vehicle.urdf.xacro` 在原 4WD 底盘、双刷、40 L 尘箱、LiDAR、RGB-D 和 IMU
基础上增加：

- 深色轮胎、独立轮毂和前保险杠；
- 白色上车体、绿色检修面板、左右示廓灯；
- LiDAR 顶盖与支柱、相机壳体与镜头；
- 尘箱盖、状态条和可辨识刷盘刷毛。

上车体视觉和碰撞使用同一 `0.72 × 0.64 × 0.22 m` 盒体，且完全位于冻结的
`1.15 × 0.72 m` 平面底盘外包络内；因此不会改变 Nav2、Collision Monitor 或 Coverage
使用的二维 footprint。轮位、轮径、轮距、质量、惯量、传感器外参、话题和控制插件
均未改变。

## 视觉、碰撞和导航几何边界

| 对象 | 视觉与物理口径 |
|---|---|
| 道路标线、斑马线 | 仅为薄涂料视觉，不产生占用或碰撞 |
| 绿化带表面 | 位于已有抬高人行道上，真实边界仍由人行道碰撞体提供 |
| 冻结结构化锚点 | 名称、位姿和碰撞尺寸逐项保持不变，避免让旧地图和导航几何漂移 |
| 新增长椅、树木、锥桶 | 有碰撞体，且全部位于原作业多边形之外或已有不可通行人行道内 |
| 树冠、路灯灯头 | 是高于底盘工作高度的非导航视觉细节；地面占用由树干/灯杆碰撞体提供 |
| 落叶堆 | 多片贴地薄片表示可跨越清扫目标，不作为刚性障碍 |
| 车辆上车体 | 视觉与碰撞一致，二维外包络不超过冻结底盘 |

新增场景和车辆资产全部由项目内 SDF/URDF primitive 自建，随仓库按 Apache-2.0
许可发布；未引入网格、贴图、Fuel URI 或第三方在线资源。

## 验证

```powershell
py -3 -m pytest -q scripts/test_gazebo_scene_contract.py
py -3 scripts/ci_fast.py
```

```bash
gz sdf -k starter_ws/src/sanitation_worlds/worlds/sanitation_structured_world.sdf
xacro starter_ws/src/sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro \
  > /tmp/sanitation_vehicle.urdf
check_urdf /tmp/sanitation_vehicle.urdf
gz sdf -k /tmp/sanitation_vehicle.urdf
```

URDF/SDF 改动还必须执行 Stage 2 和真实 WSLg Gazebo 渲染、话题与车辆运动验收；
静态解析不能代替运行时证据。
