# 第三方仓库选择与边界

## 1. 主选：Linorobot2

仓库：`linorobot/linorobot2`

用途：

- 4WD 移动底盘基线；
- 参数化 Xacro；
- Gazebo 仿真；
- LiDAR、RGB-D、IMU；
- Nav2、SLAM Toolbox、robot_localization；
- 仿真/实车接口保持一致。

采用方式：

- 锁定 `jazzy` 分支或具体 commit；
- 本项目只通过依赖、include、launch 参数和 overlay 包使用；
- 不直接修改第三方源码。

许可证：Apache-2.0。

## 2. 主选：OpenNav Coverage

仓库：`open-navigation/opennav_coverage`

用途：

- Nav2 兼容的 Complete Coverage Task Server；
- 输入多边形或 GML；
- 输出 swath、route 和 `nav_msgs/Path`；
- 支持 Boustrophedon、Snake、Spiral、Custom；
- 可配置机器人宽度、作业宽度和最小转弯半径。

优先分支：

1. `jazzy-v2`
2. 若不存在或构建不兼容，尝试 `v1.2.1-devel`
3. 最后才评估 `main`

Codex 必须记录最终 commit。

许可证：Apache-2.0。

## 3. 底层覆盖库：Fields2Cover

仓库：`Fields2Cover/Fields2Cover`

用途：

- swath 生成；
- headland；
- route 排序；
- Dubins/Reeds-Shepp 等可行路径；
- 非凸区域和带障碍区域支持。

优先使用 ROS Jazzy 二进制包 `ros-jazzy-fields2cover`；只有在版本冲突时才从源码构建。

许可证：BSD-3-Clause。

## 4. 参考但不作为主线：OpenPodcar

仓库：`OpenPodcar/OpenPodcar`

优点：

- 车辆形态接近低速作业车；
- 有 URDF/Gazebo、激光雷达、导航和实车资料。

不作为主线的原因：

- ROS Kinetic；
- Gazebo 7；
- ROS 1 move_base；
- 直接迁移会引入大量旧依赖。

只允许参考尺寸、结构和节点设计。若复制任何 GPL 代码，必须先评估许可证传染边界；默认不复制其代码或网格。

## 5. 可选后续组件

- `ros-navigation/navigation2`：导航核心；
- `PlanSys2/ros2_planning_system`：后续结构化任务规划；
- MoveIt 2：后续抓取；
- `vision_msgs`：统一检测输出；
- rosbag2：证据记录；
- `launch_testing`：自动化系统测试。

## 6. 资源许可规则

- 每个第三方模型必须记录来源、作者、许可证和修改内容；
- 禁止从商业清扫车产品页面直接抓取模型；
- 未明确许可的网格不进入仓库；
- 第一阶段优先使用自建 primitive，避免版权和离线加载问题。
