# 人类可读地图与数字孪生监督台

## 定位

该界面是本地浏览器中的人类监督层，不是新的控制器，也不是比赛成绩生成器。它把
项目已有的 Gazebo、SLAM、定位、规划、感知、清扫和安全数据按来源分开呈现，使操作者
能够回答四个问题：场景中配置了什么，车辆当前看到了什么，系统计划做什么，车辆实际
做了什么。

参考真值只来自 Gazebo world 与项目配置，用于显示和评测。SLAM 地图只来自 `/map`，
车辆轨迹只来自 `/odom`，感知目标只来自感知话题。任何实时来源缺失、错误或超时都会
显示为不可用、错误或过期，界面不会用参考配置把它补成“正常”。

## 一键启动

在本机 `TZcup-Ubuntu-24.04` WSL2 中构建当前仓库后执行：

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/sanitation_ws/install/setup.bash"
source "$HOME/tzcup_human_visualization_ws/install/setup.bash"
ros2 launch sanitation_hmi human_visualization_demo.launch.py \
  operator_token:=replace-with-a-local-token
```

其中 `sanitation_ws` 提供项目既有的 `linorobot2_*` 第三方 underlay；本任务工作空间作为
overlay 加载，不修改 underlay。该入口启动 Gazebo 仿真、异步 SLAM、安全速度门和浏览器监督服务。Windows 浏览器打开
`http://127.0.0.1:8765`。令牌只用于本机操作请求，不应提交到仓库；不传令牌时服务会
生成随机令牌并写入启动日志。

若 Gazebo、SLAM 和安全门已由其他启动文件运行，只附着监督台：

```bash
ros2 launch sanitation_hmi human_visualization.launch.py \
  operator_token:=replace-with-a-local-token
```

## 数据来源

| 界面对象 | 实际来源 | 事实边界 |
|---|---|---|
| 参考地图、目标、障碍、禁行区 | 项目 YAML / Gazebo 配置 | 仿真参考真值，不进入控制 |
| SLAM 占据栅格 | `/map` | 传感器建图输出，缺失时不替代 |
| 车辆位置和实际轨迹 | `/odom` | 估计里程计，不是 ground truth |
| 全局/局部路径 | `/coverage/current_path`、`/plan`、`/local_plan` | 规划结果，不是执行结果 |
| 感知目标 | `/perception/garbage/targets` | 模型输出，虚线标识 |
| 清扫真值与事件 | `/garbage/ground_truth`、`/garbage/cleaning_events` | 显示/评测支路 |
| 清扫覆盖网格 | `/odom` 轨迹与 `/brush_enabled` | 经验推导；无刷盘状态不计已清扫 |
| Gazebo 全场画面 | `/world_overview/image` | 真实 Gazebo 相机话题 |
| 车载画面 | `/camera/color/image_raw` | 真实车载仿真相机话题 |
| 安全控制 | `/emergency_stop` | 必须检测到外部速度门订阅者 |

绿色、橙色和红色覆盖格分别表示一次覆盖、重复覆盖和当前轨迹未覆盖。只有带
`brush_enabled=true` 的实际轨迹样本参与计算。规划覆盖率与经验覆盖率分栏显示，不能
互相替代。

## 界面与操作

- `评委`：保留地图、关键状态、三维/车载画面和事实边界；
- `学习`：增加路线依据、地图差异和数据口径说明；
- `工程`：在学习模式基础上显示全部数据源健康状态；
- `作业地图 / 参考地图 / SLAM 地图 / 对比`：切换地图语义；对比模式左右分屏；
- 地图左键拖动平移，滚轮缩放，`适配全图` 恢复全场视野；
- `图层` 可以独立开关语义区、真值、预测、障碍、规划、轨迹、车辆和栅格；
- `历史回放` 只在装载真实记录或当前会话收到至少两个里程计样本后启用；
- `导出摘要` 下载当前状态 JSON，并保留所有不可用字段和事实边界。

## 安全控制边界

操作请求继续经过 AUTO-10 token、角色、严格 schema、幂等键和受限任务 DSL。浏览器不
直接发送 `/cmd_vel`、关节或电机命令。

急停和解除急停可通过现有 `/emergency_stop` 接口派发，但只有 ROS 图中存在 HMI 自身
之外的安全订阅者时按钮才启用。Coverage、暂停、恢复和返航需要可审计的任务编排器；
当前仓库没有通用安全编排服务，因此这些按钮保持禁用，对 API 的同类请求返回 503
`safe_task_orchestrator_unavailable`。这是明确的工程边界，不是 UI 故障。

## 验收

ROS 无关单元与回归检查：

```bash
python scripts/ci_fast.py
python scripts/human_visualization_gate.py
```

连接 live 服务并生成机器报告：

```bash
python scripts/human_visualization_gate.py \
  --url http://127.0.0.1:8765 \
  --output artifacts/human_visualization_live_report.json
```

`software_contract_pass=true` 证明地图、来源分离、交互、回放、导出和失败关闭合同存在。
完整 `human_visualization_ready=true` 还要求 live `/odom`、`/map`、两路图像、外部安全门
和安全任务执行链全部在线。静态截图、测试 fixture 或成功的启动命令不能替代该 live 门。

浏览器人工复核至少覆盖 1920×1080 和 390×844：无水平溢出，地图仍为主视觉区，模式、
视图、图层、回放和控制禁用状态与来源能力一致，浏览器控制台无错误。
