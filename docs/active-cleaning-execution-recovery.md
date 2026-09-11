# 主动清扫执行与恢复合同

本页记录任务执行中的所有权、失鲜、终态与恢复规则。它不改变 A12/A19/A20 的正式通过门槛。

## Nav2 Action 所有权

规划器一次只提交一个路径；路径时间戳与执行器诊断中的 `request_stamp` 一致，旧路径的 latched 状态不能释放新路径的忙碌锁。时钟未前进或回退时暂停派发，避免复用旧身份。执行器忽略忙碌时的重复请求，不把正在执行的目标状态改成 REJECTED；迟到的取消响应只作用于原目标句柄。

执行器在目标被接受后先订阅最终结果，再处理可能提前到达的取消。取消响应只表示请求被接受，不能证明车辆已经停下或 Action 已终止。只有 SUCCEEDED、CANCELED 或 ABORTED 的真实结果可以释放目标句柄。

三个墙钟期限默认分别为 `goal_response_timeout_sec=5`、`execution_timeout_sec=1800`、`cancel_timeout_sec=5`。执行和规划 watchdog 使用 STEADY_TIME 定时器，Gazebo `/clock` 暂停时仍会触发。

目标响应超时、结果传输异常、取消拒绝/错误目标或取消后始终无终态时，执行器锁存失败并发布 `restart_required=true`。迟到的 accepted goal 仍会取消并等待结果；不能把本地 future 超时视为远端动作不存在。新的 permit 心跳不会清除失败或覆盖成功终态。操作员需先核对底盘/机械臂已安全停止及服务端目标状态，再按新运行实例恢复；不能只清除布尔值继续。

## 运动所有者租约和重新启动

`simulation_operator_gate` 使用独立的墙钟看守 planner（默认 1.5 s）和 executor（默认 0.5 s）。两个进程各自在启动时生成 UUID `instance_id`。planner 的 `/active_cleaning/control_health` JSON 和 executor 的诊断行都必须携带 `instance_id`、`published_at_monotonic_ns` 与从 1 开始严格递增的 `sequence`；两者使用同一主机的 `time.monotonic_ns()`，不依赖 ROS `/clock` 或 Unix 校时。看守不把任一方的持续心跳当成另一方存活；UUID 变化、明确的 `healthy=false`、格式不合法或超时都会立即使租约失效。

gate 只接受源时间年龄在 0–0.5 s 的收据；同一 instance 的 sequence 不严格递增、源时间倒退或未来时间都会拒绝，并清除该 owner 的上一张收据。已 arm 时这种拒绝立即断主电、断言急停，且后续新鲜心跳也不能自动恢复，仍需要新的 `false → true` 操作员边沿。arrival-time TTL 保留为第二道死进程/失鲜看守，不能由延迟或重传的历史 DDS 帧刷新。

gate 在未接收操作员低到高的 `/product_demo/operator_start` 前保持主电断开、急停断言。它的 transient-local 状态 JSON 含 `control_owners_ready`、gate `instance_id`、每次发布递增的 `sequence` 与 `published_at_unix_ns`。`control_owners_ready=true` 只表示两个新鲜、彼此独立的控制所有者已经出现，供启动编排在发送唯一的 start 边沿前等待；这不是已允许运动。编排器必须只接受本次订阅后新收到、墙钟年龄在 0–0.5 s 内、同一 instance 且 sequence 不回退的状态，不能读取 retained 历史 true。启动边沿后的三秒窗口只用于等待两个所有者，不能靠重复 `true` 重放。完成任务、gate 重启、任一所有者失联/更换都会要求先发送 `false`、再发送新的 `true`。

gate 通过真实的仿真命令话题重复发送主电、急停和急停复位请求。复位阶段结束后，只有新鲜的 true `/safety/actuators_enabled` 才发布 `/product_demo/operator_armed=true`；这会让 planner 允许派发路径。已经确认过许可的运行中，许可变 false 或超过 0.5 s 未刷新会立即断主电、断言急停并作废租约。首次复位期间尚未收到许可时，gate 只是在等待安全管理器的确认，planner 仍被阻止。`operator_armed`、本地 watch dog 和 ROS topic mock 只证明软件接口行为，绝不能作为实体急停、接触器、制动距离或其他 physical acceptance 的证据。

任一 `restart_required=true`、目标终态缺失、控制所有者替换或运行期 arm/permit 丢失都必须结束当前 episode。恢复时应停止旧 launch/ROS domain 和相关 Nav2 action server，保留日志与目标状态证据，然后用新的 episode ID、ROS domain、Gazebo partition、planner/executor UUID 和 operator low-to-high 边沿启动整套链。不得仅重启 Nav2 client、复用旧 gate 或切换 permit/布尔值后，把未知的旧 goal 当作已终态；局部重开 Nav2 不能证明该 goal 已取消、底盘已停止或任务可安全续接。

## 规划器的停止与完成

- 忙碌期间继续检查输入时效，失鲜时关闭清扫请求，并通过既有 `/active_cleaning/cancel` 取消导航；仍等真实终态，不马上派发下一条轨迹。
- 与当前路径匹配的执行器心跳超过5秒未到，或抓取结果超过900秒未到，锁存失败。抓取超时时不释放底盘用于继续行驶，不伪造机械臂取消成功。
- 目标从相机当前结果数组消失，不从任务记忆删除。只有本任务匹配的抓投验证能将它标为清除；感知的 CLEARED 标签本身不能证明物理投箱。
- 返航开始后清扫距离冻结，后续行驶单列返航距离。到起点0.25 m内还需连续至少1秒的有效静止里程计反馈（线速度≤0.02 m/s、角速度≤0.03 rad/s）才能完成。该位置门不会随规划栅格变粗而放宽，且不是充电桩对接或实车标定通过。
- 同数据长度但不同地图宽高、分辨率、原点或旋转的 belief 会被拒绝；过期 TF 不用于决策。单步里程计跳变≥2 m锁存失败，不静默丢掉里程以改善效率。

底层安全管理器、Collision Monitor 和机械臂互锁仍是独立安全链。任务超时与ROS取消测试不能证明物理制动距离或实车安全。

## 验证入口

Windows先运行项目 `py -3 scripts/ci_fast.py`。聚焦测试位于 `starter_ws/src/sanitation_active_cleaning/test/test_formal_async_lifecycle.py`，直接执行生产节点的回调方法，注入可控的future、时钟和状态消息，覆盖迟到接受、取消拒绝、断联、重复请求和错误完成判定。

Linux/ROS Jazzy上的真实通信探针：

```bash
source /opt/ros/jazzy/setup.bash
source <本次构建的install>/setup.bash
export ROS_DOMAIN_ID=<本次专用且空闲的1至101域>
export ROS_LOCALHOST_ONLY=1
export TZCUP_ISOLATED_ACTION_PROBE=1
python3 scripts/probe_active_cleaning_action_lifecycle.py --output <全新输出目录>
```

探针只使用 `/probe/*`，提供测试 FollowPath server，不向实际底盘或安全总线发布命令。它在 `use_sim_time=true` 且无 `/clock` 时验证提前取消、取消后下一目标成功和执行墙钟超时。报告固定 `formal_acceptance=false`，不替代受影响Stage、Gazebo全过程或正式任务矩阵。
