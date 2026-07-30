# 调试可视化

## 目标

`sanitation_debug_visualization` 是独立的工程调试层，不改变 Gazebo 物理模型、
感知输出、导航控制或清扫状态机。它把机器可读的场景配置和 ROS 话题转换为
RViz 图层，让操作者直接看到“场景中有什么、车辆看到了什么、系统正在做什么”。

默认界面包含：

- 五类清扫目标的稳定英文标签和类别颜色（避免精简容器缺少 CJK 字体时乱码）；
- 固定垃圾桶、纸箱、动态行人等显式负样本/障碍标记；
- 清扫区域边界与禁行区；
- `/perception/garbage/targets` 的感知目标、置信度和三维尺寸；
- `/garbage/cleaning_events` 驱动的已清扫状态；
- `/brush_enabled`、`/coverage/state` 和 `/spot_clean/state` 状态摘要；
- `/odom` 驱动的车辆轮廓、`/coverage/current_path`；
- `/scan` 默认开启；有完整 TF 时可按需开启 RobotModel；
- 可选 `/map`、RGB-D 点云和车载 RGB 画面；
- “全场俯视”和“跟车视角”两个 RViz 视角。

真值只进入 `/debug/markers` 的显示路径，不发布速度、导航目标或清扫命令，
不得把调试真值接入控制链。

## 附着到已经运行的 Gazebo

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/sanitation_ws/install/setup.bash"
ros2 launch sanitation_debug_visualization debug_visualization.launch.py
```

基础仿真默认使用 `base_link` 作为 RViz 固定坐标系，并按 `/odom` 实时换算
场地标记，因此视图会跟随车辆且不依赖缺失的 `odom→base_footprint` TF。
启动 Nav2 或 SLAM 后可改用：

```bash
ros2 launch sanitation_debug_visualization debug_visualization.launch.py fixed_frame:=map
```

## 同时启动 Gazebo 与调试界面

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/sanitation_ws/install/setup.bash"
ros2 launch sanitation_debug_visualization debug_sim.launch.py
```

## 图例

- 青色边界：清扫作业区；
- 红色边界：禁行区；
- 紫红色底圈与文字：障碍或显式负样本；
- 半透明类别色：配置/真值目标；
- 带 `PRED` 和置信度的标记：实时感知输出；
- 绿色目标：收到 `result=cleaned` 清扫事件。

## 操作

- 鼠标滚轮：缩放；
- 鼠标中键拖动：平移俯视图；
- 左键拖动：旋转三维视角；
- `F`：聚焦选中对象；
- 左侧 `Debug Layers` 可以独立开关 LiDAR、路径、地图、点云和相机；
- 基础仿真中 RobotModel 默认关闭；启动提供完整 TF 的 Nav2/SLAM 后再开启。

调试节点使用 transient-local MarkerArray，RViz 晚启动仍能立即收到当前状态。
