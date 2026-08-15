# Ackermann 清扫车车辆模型

旧模型的四个轮子方向固定，Gazebo `DiffDrive` 通过左右轮差速产生 yaw；当 `linear.x=0`、`angular.z≠0` 时左右轮反向旋转，因此可以原地掉头。这适合 skid-steer 底盘，但不符合道路式清扫车。

新 `ackermann` profile 使用前轮物理转向、后轮驱动。轴距 `L=0.76 m`，主销距 `T=0.80 m`，轮胎半径 `0.14 m`。冻结自行车等效转角为 `28°`：中心最小半径 `L/tan(28°)=1.429352 m`，内外轮角约 `36.44°/22.56°`，物理关节限位 `±38.5°` 留出至少 2° 余量。Gazebo 插件的 clamp 采用其角速度/正弦约定，因此配置为 `asin(L/R)=0.560618 rad`，不能直接写 28°。外部扫掠半径约 `2.013 m`。

Ackermann 展开包含两个 z 轴转向关节、两个自由滚动前轮关节、两个后轮牵引关节和唯一一个 `gz::sim::systems::AckermannSteering`。零线速度时后轮牵引为零，前轮可以预置角度，但车身不得产生 yaw。底盘保持 1.15 m × 0.72 m 外观，碰撞体为轮舱留出实际空间；刷盘前移到 0.66 m 以保持全转角轮刷净距。legacy 保持原 0.58 m 刷盘位置。

`/wheel/odom_raw` 由后轮位置和实际前轮转角积分；measurement adapter 加协方差后发布 `/measurements/wheel_odom`，EKF 独占 `odom→base_footprint`。Gazebo ground truth 只进入显示与验收。`artifacts/ackermann_inventory/` 由源码解析脚本生成，不能手填估计值。

当前证据边界是 SIL。真实轮胎侧偏、回正、执行器温升/电流、刷盘接触与道路制动需要 HIL 和封闭道路标定，不能由 Gazebo 通过外推。
