# 主动清扫执行与恢复合同

本页记录任务执行中的所有权、失鲜、终态与恢复规则。它不改变 A12/A19/A20 的正式通过门槛。

## Nav2 Action 所有权

规划器一次只提交一个路径；路径时间戳与执行器诊断中的 `request_stamp` 一致，旧路径的 latched 状态不能释放新路径的忙碌锁。时钟未前进或回退时暂停派发，避免复用旧身份。执行器忽略忙碌时的重复请求，不把正在执行的目标状态改成 REJECTED；迟到的取消响应只作用于原目标句柄。

执行器在目标被接受后先订阅最终结果，再处理可能提前到达的取消。取消响应只表示请求被接受，不能证明车辆已经停下或 Action 已终止。只有 SUCCEEDED、CANCELED 或 ABORTED 的真实结果可以释放目标句柄。

三个墙钟期限默认分别为 `goal_response_timeout_sec=5`、`execution_timeout_sec=1800`、`cancel_timeout_sec=5`。执行和规划 watchdog 使用 STEADY_TIME 定时器，Gazebo `/clock` 暂停时仍会触发。

目标响应超时、结果传输异常、取消拒绝/错误目标或取消后始终无终态时，执行器锁存失败并发布 `restart_required=true`。迟到的 accepted goal 仍会取消并等待结果；不能把本地 future 超时视为远端动作不存在。新的 permit 心跳不会清除失败或覆盖成功终态。操作员需先核对底盘/机械臂已安全停止及服务端目标状态，再按新运行实例恢复；不能只清除布尔值继续。

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
