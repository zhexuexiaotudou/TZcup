# 项目推进记录

## 2026-08-25：正式整车传感器塔与机械臂运行闭环

- 删除含义不清且会吞没 MID-360 的宽 T 形顶梁，重构为具名模块化传感器塔：`190×150×16 mm` 螺栓底座、双 `30×30 mm` 立柱、线缆服务脊、横撑和加强筋；UTM-30LX、MID-360 与 ANN-MB 分别使用独立悬臂、四点隔振散热板和侧下置 `150 mm` 接地板。前 D435、双后鱼眼和内部 VN-100 也增加独立物理支架，不再直接悬挂到底盘 link。
- 修正 MID-360 为公开的 `-7°～+52°` 垂直视场、`48×36 mm` 四点安装，并把 GNSS 放到 MID-360 下视边界之外；前 D435 下俯 `8°`。组件台账对 8 套传感器和 9 个主要机械总成逐项绑定型号、父 link、支架 link、设备 link、坐标、连接方式和话题。
- 机械臂承力链补齐载荷平台背板、加强 pedestal 和六螺栓转接盘。六轴 UR5e 与 2F-85 主驱动拆分为独立 `arm_controller` / `gripper_controller`，禁止部分目标并设置轨迹/终点误差；Bullet Featherstone 中实际执行四段六轴轨迹和夹爪 `开→闭→开→半闭`，Action 均返回成功，六轴运动范围 `0.2175～0.3989 rad`、夹爪 `0.6443 rad`，最大终点误差分别 `2.34e-5 rad` 与 `4.25e-5 rad`。
- 展开模型现为 122 links / 121 joints / 空载 156.205866 kg，120 个物理 link 的质量与正定惯量通过，29 个静态安装帧通过 5 mm / 0.02 rad 一致性门；150 个 mesh visual 中只有两个透明动态载荷状态体使用 primitive。产品和检修 profile 各完成六路 1600×1000 Gazebo Ogre2 采集，新增传感器塔、前 D435 和机械臂安装近景。
- 快速回归为 `341 passed`。本轮证明具名安装链、传感器可见性、控制器激活和六轴/夹爪刚体运动；MoveIt 全空间自碰撞、鱼眼等距投影、线缆柔性、电机电流/温升、精确视场光线追踪和实车标定仍保持 fail-closed。
- 全新 WSL/ROS 2 Jazzy 工作区的 Stage 1 完成两次构建，`629 tests / 0 errors / 0 failures`；Stage 2 通过 URDF/SDF、GPU/Ogre2 与 12 类必需话题检查，5 s 运动探针实测底盘位移 `1.07 m`。同时修复 Stage 1 复制工作区无法读取仓库级脚本、配置和历史工程证据的可移植性，使门禁不再依赖源码恰好位于仓库根目录。

## 2026-08-25：正式整车产品外观 V3 与 Gazebo 视觉验收

- 把 V2 的“机构网格完整但上装裸露”继续升级为产品车身：新增依赖无关的参数化外壳生成器，生成 44 个连续曲面/功能件 STL，正式 profile 引用 42 个车身 mesh，包含封闭前后舱、右前机械臂 U 形工作舱、检修门、灯组、保险杠、轮眉、刷盘护罩、接口和多色工业设计；`bodywork_visible:=false` 保留检修教学模式。
- 修复 loft 网格面法线反向导致 Ogre2 背面剔除车顶的根因；不是用颜色掩盖几何缺陷。机械臂安装位移至右前 `(0.100,-0.200,0.4341) m` 并使用 2.8 kg 显式安装座；前 RGB-D、后鱼眼、UTM、MID-360 与 GNSS 重新落到壳体/桅杆实际安装面，三段回收软管改为连续上升路线。
- 展开模型为 115 links / 114 joints / 空载 150.462866 kg；113 个物理 link 均有正质量和正定惯量，22 个非 pending 安装帧通过 5 mm / 0.02 rad 一致性门。当前有 142 个 mesh visual，仅两个透明动态载荷状态体保留 primitive。
- 新增产品车身门、Gazebo Ogre2 三相机工作室世界、自动机械臂视觉收纳命令与 1600×1000 三视角采集器；三路画面均通过非黑屏/分辨率检查。该证据证明正式外壳能在 Gazebo 渲染，不替代机械臂全空间扫掠、精确传感器自遮挡、清扫接触或实车验证。
- 资产审查没有找到许可证和尺寸同时适合本整车的开源环卫车完整外壳；继续直接复用官方 A300、UR5e、Robotiq 与 Robotnik 传感器资产，外壳由项目参数化生成。Robotnik RB-VOGUI（BSD-3-Clause）仅作合法形态参考，因轴距、轮距和上装接口不匹配而未拼接进正式车。
- 当前快速回归 `336 passed`；项目通用 Stage 1 为 629 tests、0 errors、0 failures（51 项上游 cppcheck 策略跳过），Stage 2 GPU/Ogre2 兼容回归收到 12/12 类必需话题并在 5 s 内位移 1.191250 m。Stage 2 只证明旧运行链未回归，不替代正式整车三相机证据。正式空载总质量已显著高于前期包装预算，A300 载荷/重心与运输宽度、机械臂扫掠、传感器遮挡和清扫接触仍是 fail-closed 外部门，不得把视觉完成度表述为实车数字孪生完成。

## 2026-08-25：竞赛级高保真整车规划与条件可行性评估

- 新增[高保真竞赛清扫车虚拟样机规划](high-fidelity-competition-vehicle-plan.md)，把Husky A300定义为通过载荷、重心、供电、效率和通道门后才能冻结的条件候选；正式profile包含UTM-30LX、MID-360、前向RGB-D、两个侧后鱼眼、末端相机、UR5e、2F-85、征程6/S100、几何约45 L且可用容积`>=40 L`的干垃圾舱和完整刷盘/刮吸/污水舱总成。
- 冻结积水回收的质量守恒口径：刷盘只负责汇聚，刮水胶条和受数据表额定流量/有界分段效率限制的吸口回收水；地面体积扣减必须等量增加污水舱液位和整车质量。20,000 m²主场采用可运行的稀疏水层，微型场才使用细粒度流体；视觉隐藏或直接删除积水不计为完成。
- 冻结3 cm立方体的纸板、PP、PET、铝材料随机质量方案；方块在投箱后仍保留为刚体，使垃圾箱质量和组合重心随任务增长。密度范围是规划初值，实施前仍须以材料数据表锁定。
- 复核赛题映射后采用保守分仓：赛题明确的是40 L尘箱，因此正式干舱保证可用容积`>=40 L`，污水舱不得侵占并按A300剩余载荷、重心和空间反算，设计上限不超过20 L；正常episode积水总量由污水舱可用容量自动约束。
- 积水首版冻结为L1：清扫/刮吸机构精确展开，主场采用稀疏2.5D水层、有界分段回收和质量守恒，不做全场粒子/CFD；赛题自身只要求`<=1 cm`低摩擦积水工况和通过/横向误差/打滑证据，实际清洁效果原本就不由初期仿真证明。项目额外要求可达积水回收`>=95%`、质量误差`<=1%`和非法回收为零。
- 方案冻结时的结论为“高保真名义数字样机可行、A300条件可行、全场粒子流体和未经实物标定的单位级数字孪生不可承诺”。用户已确认允许依据公开数据表自建并开源缺失CAD/Xacro；该时点只完成方案文档，随后才进入下一条所述的实施前准备。
- 随后完成[URDF实施前准备包](pre-urdf-readiness.md)：5个开源仓库锁定commit，冻结A300/UR5e/2F-85、8个传感器、两套独立Arducam鱼眼、带编码器Pololu刷盘电机、Actuonix推杆和Jabsco回收泵，并建立24 frame、8 joint、许可证契约及质量/功耗/容量/吞吐四张CSV账表。预载荷为`77.705 kg`，20个最坏铝块为`1.512 kg`，仅按载荷反算的污水名义/可用/正常episode上限为`12.133/9.706/8.493 L`；干扫预目标`3564 m²/h`，2 mm/10 mm积水流量限速约`0.116/0.023 m/s`。重心、安装空间、S100实板包络和VBAT拓扑仍是正式URDF阶段硬门。本轮没有制作正式Xacro/mesh或运行新仿真。

## 2026-08-24：无精确URDF的园区主动清扫与3 cm立方体抓取占位闭环

- 新增ROS无关的地面RANSAC、高度筛选、空间哈希聚类、30 mm包围盒筛选和规则顶抓Pose生成；新增最多两次尝试、安全暂停、两类证据验证、20目标后箱与mock导航/规划/夹爪闭环。
- 新增参数化`0.60 m × 0.40 m`麦克纳姆小车、通用六轴臂、夹爪、传感器frame和`0.20 m × 0.20 m × 0.10 m`后箱的placeholder Xacro/SRDF/profile，以及立方体抓取ROS消息/action合同。
- 新增固定面积多长宽比园区生成器、public/environment/evaluator隔离、随机移动行人、主动清扫环境、全覆盖/感知贪心/Oracle、belief-only Q-learning、单目标投箱清除适配和统一命令；ROS Jazzy构建23包，变更包87项ROS测试通过，完整研发场占位任务达到观测`97.43%`、地污`100%`、离散垃圾`95%`和零安全违规。
- 以上能力明确为`evidence_authority=false`，不证明实车几何、模型精度、机械臂可达性、板端性能或sim-to-real。真实闭环等待实测URDF、`ros2_control`、MoveIt 2、传感器/手眼标定和DOSOD/EdgeSAM实际运行，详见[`urdf-independent-research-demo.md`](urdf-independent-research-demo.md)。

## 2026-08-04：skid-steer 覆盖路径优化与语义可视化

- 历史基线完整保留：OpenNav Coverage + Fields2Cover 的 `BOUSTROPHEDON / DUBIN / CONTINUOUS`、`0.35 m` 条带间距、约 46% 重叠、最多两轮补扫、固定 17 组件和全局 turbo 速度仍可用 `-CoverageProfile legacy` 回归。该汽车式大圆弧基线不适合可原地旋转的 skid-steer，且正式 5-seed 基线的重复率和横向误差不满足新门，因此不再是小场默认值。
- 小场默认改为 Hybrid Area-Fill CPP：Fields2Cover 只负责区域/条带几何，优化器在 0–175° 与 `0.42/0.46/0.48/0.50/0.52 m` 候选中选择方向和间距；最终选择 `0° + 0.52 m`，形成 6 条相邻弓字主条带。主连接器为 Nav2 `Spin → DriveOnHeading → Spin`，失败时依次尝试后退、Nav2 绕行或延后条带，不直接发布 `/cmd_vel`。
- `CoveragePlan` 现在显式区分 `TRANSIT/SWATH/ROTATE/SHIFT/BACKUP/OBSTACLE_BYPASS/REPAIR_SWATH/RETURN_HOME`，完整计划由 transient-local `/coverage/full_plan` 持久发布；当前组件、规划条带/连接/补扫和实际清扫/转场/补扫各有独立话题，旧 `/coverage/current_path` 只保留 alias。`expected_components` 从实际 `ordered_components` 动态计算。
- 残余补扫按 8 邻域连通域生成局部短条带，进入和航向对齐期间刷盘关闭；每个任务最多一轮，实际下发补扫长度不超过主条带的 10%。10 个正式注入 seed 全部观察到漏扫、执行一次补扫并恢复到 ≥99.5%，重复率为 16.08%–19.50%，且没有使用 Gazebo 真值控制车辆。
- 正式 optimized 5-seed 全部通过：实际覆盖率 100%、10/10 目标、重复率 14.25%–17.83%、零碰撞/零禁行区违规，定位 RMSE 3.47–3.94 cm，直线规整度 P95 3.49–5.65 cm。相对 legacy 中位数，实际总里程下降 48.97%、刷盘关闭里程下降 61.46%、连接里程下降 68.00%、任务时间下降 49.57%。
- seed 132 的 MCAP 经真实 `ros2 bag play` 和顺序重建双门通过：124,094 条消息、14 个必需话题、125.303 s，包含完整语义计划、组件序列、刷盘切换和 `COMPLETED` 终态。原始 MCAP、运行日志和所有失败尝试保留在仓库外，Git 只接收紧凑摘要。
- Gazebo 原生地图固定使用紫色主条带、灰色虚线连接、黄色补扫、绿色实际清扫、深灰实际转场、白色当前组件、红色受阻区间和绿色已清扫栅格；完整计划不会再被每次当前路径更新清空。动态障碍矩阵分为三个独立任务，每轮要求 8 个有效交互且最多额外尝试 2 次，保留 3.0 m 路径余量、0.5 m 同条带间隔和 1.5–1.8 m 横穿距离。
- 正式动态矩阵 3 个独立任务全部通过：24/24 次注入均形成有效 LiDAR 交互，恢复率和任务继续率均为 100%，碰撞 0、最小观测净距 0.467 m、重复振荡 0；三次任务均完整结束、满足覆盖质量、无禁行区违规，并在退出时关闭刷盘。静态直线规整度由独立 5-seed 矩阵验收，避免把动态停车/恢复产生的瞬态误算成基础路径摆动。
- 任务 schema 同时支持 `AREA_FILL / TAUGHT_ROUTE / POINT_CLEAN`。教学路线必须带版本和 SHA-256，逐段保留速度、刷盘、方向、禁扫区、交互点和恢复点并由 Nav2 执行；篡改或越界 fail-closed。定点清扫继续委派 `sanitation_spot_cleaning`，不会被面积覆盖执行器误吞。
- 边界：以上是 ROS 2/Nav2/Gazebo SIL 证据，不证明垃圾视觉泛化、真实吸扫效果、真实 RTK/轮滑参数、J6 板端、真实行人制动或 20,000 m² 全场耐久。实车前必须重标刷宽/前置偏移、轮胎侧滑和旋转超调，加入电流/温升限制，用真实刷盘接触与定位替代 Gazebo 真值覆盖，并依次通过 HIL、封闭场低速和操作员回滚验收。

> 仓库清理说明：当前树只保留源码、配置、最终状态和紧凑评审证据。早期 Stage 0–4S 的原始构建日志、MCAP、逐点轨迹 CSV 和重复标定运行已从当前树移除；历史结论仍记录在本页及 `reports/reviews/GPT_REVIEW_STAGE*.md`，原始字节可从清理前 Git 历史恢复。新的原始运行数据必须留在 Git 忽略目录，详见 [`artifact-policy.md`](artifact-policy.md)。

## 2026-08-03：小场目标密度

- 小场可清扫目标从 5 个增加到 10 个，瓶、罐、纸张、纸盒和落叶各 2 个，全部注册进同一任务配置并参与清扫计数和场景移除。
- 新目标分布在实际清扫区的不同覆盖条带，保持真实物品尺寸、类别语义和清扫判定半径，不增加导航碰撞。
- 现场完整运行发现外任务区中的目标不会被青色内框覆盖路径遍历；现将 10 个目标中心全部约束在实际清扫区内部，并用场景契约阻止越界配置回归。
- 录制前复核发现，虽然右侧地图为 10/10，Ogre2 斜俯视三维画面中的超薄纸张和落叶会发生单面/深度消隐，沿西南视线排列的目标也会互相遮挡。纸张因此补充同尺寸超薄实体底层，落叶堆增加到六片真实尺度、可双面观察的薄实体叶片并使用不规则贴地阴影；合并后的 WSLg 复核又将相机收近并提高俯角，仍不使用圆形高亮或碰撞体。后续录制复核认为两行五列过于规则，目标现改为覆盖清扫区四个象限、最小间距 `0.70 m` 且横纵坐标不成行列的错落均匀分布，同时保持十个目标均可见、可遍历。
- 修复演示速度参数只更新控制器和安全速度门、却没有同步 velocity smoother 的问题；小场 `fast/turbo` 现分别使用 `0.70/0.90 m/s` 直线速度与 `0.60/0.75 rad/s` 转向速度。首次 0.90 m/s 实跑虽把执行时间降至约 234 秒且零碰撞，但实际覆盖率仅 95.83%，因此未作为通过结果；高速配置进一步把条带间距收紧到 `0.35 m`、形成约 46% 重叠与 17 个主组件，并继续以实际覆盖率 99.5%、目标全清和零碰撞为门禁。

## 2026-08-03：小场清扫物品可辨识外形

- 保持上一轮真实尺寸和目标真值不变，将小场瓶、罐、纸张、纸盒、落叶和积水从单一基础体升级为具体物品外形。
- 塑料瓶增加瓶底、标签、瓶肩、瓶颈与瓶盖；易拉罐增加标签、上下罐沿与拉环；纸张增加不规则轮廓、印刷线和折角；纸盒增加翻盖与胶带。
- 四片落叶改用带叶尖和弧形叶缘的 SDF polyline，每片增加主叶脉，代表叶增加叶柄；积水改为十二边不规则轮廓和高光，不再用圆盘拼接。
- 所有形状均为仓库内文本几何，不引入第三方模型；视觉细化不进入导航碰撞、规划或清扫判定链。

## 2026-08-02：独立小场、同窗清扫遥测与冷启动就绪门

- 新增独立 `16 m × 12 m` 竞赛功能演示世界、五类可移除目标和 30 m² 指定作业区；小场不再复用大场景或只裁剪任务多边形。Gazebo 右栏实时显示规划路径、实际轨迹、已清扫栅格、面积、覆盖率、目标数、效率、里程、速度、仿真时间和组件进度。
- 新增 `normal/fast/turbo` 三档仿真速度、硬件接口契约、Sim-to-Real 故障档案和 SIL/HIL/封闭场/实车准入说明；仿真目标清除与覆盖统计保持 `evaluation-only`，不冒充实车识别、吸入或称重证据。
- 冷启动现在先等待定位话题和 `odom→base_footprint` TF，再精确要求 Nav2 controller/planner 为 `active [3]` 后打开界面；修复了 `inactive` 包含 `active` 导致的假阳性。
- `main@9f6b788b64974b12c5b49e46055a7c305d16362a` 在本机 WSLg 从全新 overlay 实跑完成：9/9 组件、经验覆盖率 92.0%、5/5 目标、0 碰撞、0 禁行区违规、定位 XY RMSE 0.0478 m；暂停时刷盘关闭且速度为 0，继续后终态 `COMPLETED`，3D 视口 `near_black_ratio=0.0`。运行时代码经 [PR #72](https://github.com/zhexuexiaotudou/TZcup/pull/72) 与 [PR #73](https://github.com/zhexuexiaotudou/TZcup/pull/73) 合并。

## 2026-08-02：Gazebo 3D 视口真实黑屏修复

- 用户截图证明 Qt 外壳、World Control 和原生任务卡正常时，`3D Scene` 仍可能全黑；原有
  “窗口响应 + ROS READY + D3D12 renderer”验收不足。默认 Gazebo GUI 对照窗口使用
  X11/llvmpipe 后显示出真实场景，确认世界、模型和服务端数据本身正常。
- 大、中、小三档自定义 GUI 配置补齐 Gazebo 默认场景管理、交互视图、相机跟踪、Marker
  和实体选择插件；WSLg 的 AUTO 模式仅把 GUI 改用软件渲染，headless 服务端和传感器仍使用
  D3D12/NVIDIA。
- 新增 X11 原生像素探针：首轮仍黑时正确返回退出码 `8`，没有误报 READY；补齐插件并在独立
  overlay 重建后，比赛大图捕获 `near_black_ratio=0.0`、`render_visible=true`，控制节点和
  start/pause/resume/stop 四个服务同时在线。

## 2026-08-02：WSLg 黑色残留窗口收尾

- 复验发现 Linux `gz sim -g` 已退出时，WSLg 偶尔仍保留无标题黑色 RemoteApp 外壳；窗口
  守护器现保存已确认的 Gazebo HWND，并在 GUI 消失或守护停止时只向该句柄发送 `WM_CLOSE`，
  防止任务栏残留无渲染内容的黑窗口，同时避免终止整个 `msrdc` 或其他 WSLg 应用。
- 本机故意终止 Gazebo GUI 后，ROS/Gazebo 子进程与 `8877` 全部释放，Windows RemoteApp
  窗口枚举为 0，任务终态保持 `OPERATOR_GUI_CLOSED`，没有把异常退出记成任务完成。
- 连续关闭与重开回归还捕获到 WSLg 冷启动后 GUI 在原生控制加载前提前退出；Windows
  启动器现对退出码 `4`（早退）和 `7`（COPY MODE）共用一次有界安全重启，第二次失败即
  明确报错，避免把空窗口或无限重试留给操作者。

## 2026-08-01：Gazebo 原生任务控制、三档场景与车辆细化

- 修复 WSLg Gazebo 窗口关闭/最小化后的不可恢复问题：启动前自动挂载并持久化
  `/mnt/shared_memory`，避免 WSL 2.7.3 的 `[WARN:COPY MODE]` 回退；Windows 守护器恢复隐藏和
  异常最小化窗口，Linux 任务监督器在 GUI 关闭后立即停止运行链并
  释放看板端口，保留 `wslg_window_guard.jsonl` 和 `launcher_termination.json` 作为真实终态证据。
- 新增 Gazebo 原生“清扫任务控制”卡片，通过 ROS 2 Trigger 服务提供开始、暂停、继续、停止和关闭；
  暂停会关闭刷盘、取消当前 Nav2 goal 并进入 `PAUSED`，控制卡本身从不发布 `/cmd_vel`。
- 新增 `30 m × 20 m`、`80 m × 50 m`、`200 m × 100 m` 三档园区场景；大场景物理地面精确为
  `20,000 m²`，三档均包含道路、建筑、绿化、停车、公交站、街具和清扫目标，且只使用离线 SDF 基础几何。
- 车辆增加前灯、尾灯、警示灯、检修门、把手、充电口、后吸口、安全条和刷盘支臂等可读外观；
  二维 footprint、轮距、动力学、碰撞包络、传感器外参与 ROS 话题保持冻结。
- 本机 WSLg 已实际执行 `READY → 开始 → 暂停 → 继续 → 停止`：暂停后刷盘关闭且速度为零，
  停止报告包含 `stopped_by_operator=true`；大场景尺寸正确不等于 20,000 m² 全场清扫已验收。
- 操作入口、三档地图和车辆部件表分别见
  [`gazebo-multiscale-control.md`](gazebo-multiscale-control.md) 与
  [`vehicle-model-guide.md`](vehicle-model-guide.md)。

## 2026-08-01：有界小范围完整清扫演示

- `run_gazebo_cleaning_demo.ps1` 默认改为约 `6 m × 5 m` 的真实 Coverage 任务，保留
  `-FullArea` 切换原 17 段任务；缩小范围只用于一个镜头看清完整功能，不替代正式范围验收。
- Gazebo 增加指定清扫区、实际出发点、第一条作业带起点以及人类可读状态，俯视跟随镜头同时
  保留车辆和完整任务边界；规划、控制和安全链不订阅这些 marker。
- 收尾审计发现 Gazebo 会把动态 marker 的 `id=0` 解释为自动分配；当前路径和车顶状态已改为
  固定非零 ID 原位更新，并增加合同测试，防止长时间演示累积旧路径和文字残影。
- WSLg 正式运行完成 5 条作业带、4 次转弯（9/9 组件），经验覆盖率 92.42%，碰撞和禁入区
  违规均为 0，可视化 warning 与请求丢弃均为 0。`4 m × 4 m` 和最初位置的 `6 m × 5 m`
  候选分别暴露短作业带和实体垃圾桶阻挡，最终区域通过平移避障且未关闭任何安全门。

## 2026-08-01：Gazebo 单窗口完整清扫过程

- 新增 `sanitation_gazebo_visualization`，只读订阅真实 Coverage 任务的当前路径、状态、刷盘和
  evaluation-only 真值里程计，通过 Gazebo MarkerManager 显示当前路径、实际已清扫带和车顶状态。
- 新增 `scripts/run_gazebo_cleaning_demo.ps1`，复用 AUTO-17 的 Stage4V 定位、Nav2、Coverage
  和证据链，但默认不打开浏览器与 RViz，并在完成后保留 Gazebo 供人工检查。
- 显示层不发布 `/cmd_vel`、导航 goal 或安全状态；`world_to_map` 变换显式冻结为当前任务几何，
  真值只用于显示和评估，不能进入规划、控制或安全决策。
- 本机 WSLg 正式验收完成 17/17 组件，经验覆盖率 92.47%，碰撞与禁入区违规均为 0；
  Gazebo 中确认三个 marker namespace，限频后的显示节点警告与丢弃请求均为 0。紧凑证据见
  `artifacts/auto17_gazebo_cleaning_process_20260801_evidence/acceptance_summary.json`。

## 2026-07-31：Gazebo 场地、车辆模型与数字孪生场景

- 本轮按需求把重点收回 Gazebo 本体，没有继续扩展网页控制台；新增
  `gazebo_scene.launch.py`，可只启动结构化场景和车辆。
- 将原有基础方块世界完善为可读的校园/园区道路：增加道路标线、人行横道、路缘、绿化带、
  建筑立面、树木、路灯、垃圾箱、长椅、安全锥、行人，以及瓶、罐、纸盒、落叶和积水等
  清扫语义；全部资产为仓库内程序化基础几何，无在线模型依赖。
- 完善车辆 Xacro 的上车身、作业舱、保险杠、灯组、轮毂、传感器壳体、箱盖和刷盘视觉，
  同时保留传感器外参、轮子尺寸、基础碰撞包络与控制话题。
- 新增场景合同测试，冻结既有锚点和碰撞尺寸，并检查语义对象、离线资产、家具碰撞位置以及
  车辆视觉/碰撞一致性；顺带修复 3 个只能在源码原位运行、无法适应干净 Stage 1 工作区的
  历史测试脚本路径夹具。
- 验收结果：快速回归 `184 passed`；干净 Docker Stage 1 双轮均为 513 项、0 失败；
  Stage 2 为 43/43，URDF/SDF 均有效，运行时 12/12 类话题齐全，5 秒位移 `1.1775 m`。
  WSLg 独立场景另测位移 `1.2425 m`、实时因子 `1.0002`，并人工检查总览与车辆近景。
- [PR #56](https://github.com/zhexuexiaotudou/TZcup/pull/56) 的 `fast-validation` 通过后，
  已按 merge-commit 策略合入 `main@1d8a45eac1eca7e1e39990211efab2cd453f53d4`。从该精确
  合并提交建立的 `/home/zhexu/tzcup_gazebo_scene_deploy_1d8a45e_ws` overlay 构建 3 包
  成功，独立启动结构化世界后 12/12 类话题齐全，5 秒位移 `1.2050 m`、实时因子
  `1.0011`，合并版总览图人工复核通过；回滚点为 `8ec902e`。
- 操作、资产许可、几何一致性和验证说明见
  [`gazebo-digital-twin-scene.md`](gazebo-digital-twin-scene.md)。

## 2026-07-31：AUTO-17 可视化演示层

- 新增一条命令入口 `scripts/run_visual_demo.ps1` / `scripts/run_visual_demo.sh`，在专用 overlay 中启动 Stage4V 混合定位、外部定位 Nav2、Coverage、Gazebo GUI、RViz 和实时看板。
- 新增 `sanitation_live_dashboard` 只读 ROS/HTTP 节点、响应式任务地图、实时轨迹/进度/刷盘/安全状态、Gazebo 跟随相机和 RViz 跟随视图。
- 录像由只读遥测专用渲染器生成，不录制 Windows 桌面，不采集无关窗口；默认同时记录 18 个关键 ROS 话题到 MCAP。
- 本机 WSLg 正式任务：planning/transit/full execution/empirical coverage/safety 全部成功，17/17 组件，经验覆盖率 `0.9366667`，实际路径 `42.3215 m`，任务时长 `362.023 s`，定位 XY RMSE `0.035878 m`，碰撞/keepout/刷盘违规均为 `0`。
- MCAP `205528` 条消息、18 个话题、持续 `397.705 s`；看板终态 `COMPLETED`，专用 MP4 `1.49 MB`，机器验收汇总 `PASS`，单命令启动器自行返回 `0`。
- 边界不变：真值只用于评估与绘图；learned perception、real domain、J6 runtime、simulation competition matrix 仍为 false。

## 2026-07-31：地图优先的人类监督台

- 将原有单页 DSL 表单重构为高密度工业监督台：二维作业地图占首屏主区域，增加
  参考/SLAM/作业/对比视图、图层控制、车辆姿态、规划与实际轨迹、真值/预测分离、
  障碍/禁行区、刷盘轨迹推导覆盖网格和地图交互。
- 新增线程安全运行状态和可选 ROS 2 适配器，接入 `/odom`、`/map`、规划路径、
  两路图像、感知/真值、清扫事件、Coverage、刷盘和急停；每个来源单独计算
  live/stale/error/unavailable，不以静态配置填充缺失的实时来源。
- 新增 Gazebo 全场与车载相机面板、事件时间线、评委/学习/工程模式、当前会话真实
  里程计回放、JSON 摘要导出和 `human_visualization_gate.py`。
- 保留 AUTO-10 的鉴权、授权、幂等和受限 DSL；急停可派发至现有
  `/emergency_stop`，但必须检测到外部安全订阅者。Coverage、暂停/恢复、返航等任务
  在缺少安全任务编排器时 API 返回 503、UI 禁用，绝不声称执行成功。
- 新增一键 ROS 入口 `human_visualization_demo.launch.py`，用于启动结构化 Gazebo 场景、
  SLAM、安全门和浏览器监督服务。本机 WSLg 冷启动已确认 `/odom`、`/map`、生产车载
  相机、Gazebo 总览相机和外部安全门全部为 `live`，监督链
  `visual_monitoring_ready=true`；1920×1080 与 390×844 浏览器布局均无水平溢出。
- 合入同期 AUTO-17 主分支能力后，Windows 快速回归为 180 项通过，WSL ROS 包测试为
  15 项通过、0 失败；两套浏览器/RViz 可视化入口和测试均保留。完整
  `human_visualization_ready` 仍因仓库没有安全任务编排器而保持 false；这是已记录的
  执行链硬边界，不能用 UI 或 DSL 成功代替。

## 2026-07-30：Docker/WSLg 调试可视化

- 新增 `sanitation_debug_visualization` 包，将现有目标注册表、Stage5A 场景和任务
  区域配置转换为 `/debug/markers`，同时订阅感知、真值、清扫事件、刷盘、
  Coverage、定点清扫和里程计状态。
- RViz 已在 `tzcup-gazebo-x11` 容器中连接正在运行的 Gazebo 实际渲染；可见
  清扫区、禁行区、五类目标、负样本障碍、车辆方向、LiDAR 和运行状态；
  容器重启后 `/scan` 实测约 9 Hz。
- 基础仿真缺少完整 `odom→base_footprint` TF 时，节点用 `/odom` 将全局标记
  换算到 `base_link` 跟车坐标系；Nav2/SLAM 环境仍可使用 `fixed_frame:=map`。
- 调试层采用可靠、transient-local 的 `MarkerArray`，晚启动 RViz 也能收到当前
  状态；真值只用于显示，不发布任何控制命令，不提升感知或竞赛通过状态。
- 操作、图例和启动命令见 `docs/debug-visualization.md`。

## AUTO-16：最终发布工程（2026-07-30）

已建立最终状态、阻断注册表、18 类竞赛矩阵、证据索引、最终 manifest、SPDX SBOM、模型/资产/第三方许可、中文操作员指南、竞赛演示边界和回滚说明，并提供 Validate/Build/Simulation/Matrix/Package 一键入口。最终 clean clone `8549422` 的 `ci_fast` 为 154 passed；首次全 ROS build 发现 `sanitation_manipulation` 缺少 `ament_python` build type，随后又发现 HMI 未声明 pytest discovery。两项均修复并加入合同测试，重跑为 17 packages、220 tests、0 errors、0 failures、5 skipped。`AUTO-16=PASS`、`AUTONOMOUS_SOFTWARE_COMPLETE=true`；综合矩阵、真实域和 J6 最终状态保持 false。最终 ZIP 只从合并后的精确 main 生成。

## AUTO-15：18 类竞赛需求矩阵完成、综合任务依赖阻断（2026-07-30）

已将建图、全覆盖、定时轨迹、离散垃圾、落叶堆、积水、定点清扫、动态避障、窄通道、边界、急停、APP、语音、LLM DSL、满箱、恢复回放、效率和 J6 共 18 类场景逐项绑定到 AUTO 阶段状态。现有 PASS 阶段只记为组件证据。由于 AUTO-08 学习感知/定点清扫被 AUTO-05 模型门阻断，正式综合任务没有启动；每场景 seeds、integrated missions、视频和 MCAP 均为 0。故 `AUTO-15=BLOCKED`、`SIMULATION_COMPETITION_MATRIX_PASS=false`。证据位于 `artifacts/autonomous_auto15_20260730_evidence/`。

## AUTO-14：官方 J6 工具链就绪、正式编译依赖阻断（2026-07-30）

D-Robotics 官方 OpenExplorer `3.7.0` 的 2.85 GB S100/S600 包已完成 SHA-256 校验，`hbdk4_compiler 4.7.5`、`hmct 2.6.5`、`horizon_tc_ui 3.5.3` 已解析，隔离 CUDA/cuDNN 环境中的 `hb_compile --help` 成功。仓库具备固定 batch/shape、operator/custom-op、500 帧校准集预检、官方配置生成和 HBM fail-closed runtime adapter。AUTO-06 正式模型未产出，故不得执行正式量化/编译；本机无 J6 板卡。`AUTO-14=BLOCKED`，`J6_TOOLCHAIN_PASS=false`、`J6_RUNTIME_PASS=false`；证据位于 `artifacts/autonomous_auto14_20260730_evidence/`。

## AUTO-05：G3 数据门通过、模型 screening 阻断（2026-07-30）

已建立 8 个实际 Gazebo 世界和 `4/2/2` world split；每世界 15 scene、每 scene 10 个原生同步 frame。val/test 每世界固定 5 个 negative-only scene，target/hard-negative variant、world 和 trajectory 按 split 隔离。新增 world-level 实际材质/光照、实际重叠、主动接近前后角色，并让动态 hard-negative 在采集期间通过 Gazebo service 实际移动、逐帧记录，而不是只写请求字段。

数据 QA、防泄漏审计、direct detector、独立 RGB-D area heads、validation-only 阈值选择、test 冻结评估和 ONNX parity 均已执行。三次有界方案的最佳结果仍有 7 个门失败，故 `AUTO-05=BLOCKED`；AUTO-06/07/08 依赖阻断。紧凑证据见 `artifacts/autonomous_auto05_20260730_evidence/`。

采集期间暴露的车辆状态继承、桥接进程泄漏、动态对象碰撞走廊和 odom 位移假通过均已修复并保留失败证据；最终 120/120 场景、1200/1200 帧通过严格 QA。
## AUTO-11：20,000 m² 大地图与定时任务（2026-07-30）

新增 `sanitation_tasks.large_map`：生成 200 m × 100 m、0.1 m resolution 的 PGM/metadata、20 个 zone/submap 索引、可重载地图和定时任务矩阵。定位评测用 simulator world-state 生成 truth，另用带噪声/丢失事件的 observation model 生成 estimate，并在代码和报告中固定 `self_comparison_used=false`。

首轮正式矩阵通过：10 条轨迹 RMSE max `0.03004 m`，lost recovery `0.95`，TF continuity `0.99998`；5 次 full coverage 和 20 次 scheduled route 的 zone accuracy `1.0`、boundary/collision `0`、resume `0.96`。`AUTO-11=PASS`。紧凑证据位于 `artifacts/autonomous_auto11_20260730_evidence/`，2 MB map 和完整逐轨迹报告在 Git 外以 SHA 索引。证据级别为离线大地图仿真，不声称 Gazebo 或实车。

## AUTO-10：APP、语音与受限任务 DSL（2026-07-30）

新增 `sanitation_hmi` 包，提供标准库 HTTP 服务、本地响应式控制台、token/role/idempotency 网关以及固定 schema 的任务 DSL。语言层仅允许 coverage、spot-clean、schedule、pause/resume、return-home、status 和 emergency-stop 等任务工具，直接底盘/关节/电机控制被拒绝，HTTP 验证阶段不派发真实动作。

正式矩阵已通过：APP/API/UI `288` cases，合法成功率与非法拒绝率均为 `1.0`，P95 `16.03 ms`；Windows System.Speech 生成的 `500` 条语音，经 3 voices、3 rates、4 noise levels、3 reverb profiles 后由 GPU faster-whisper small 识别，intent accuracy `0.9911`、unsafe rejection `1.0`、P95 `171.94 ms`；DSL `1200` cases 的 semantic/tool/argument accuracy 均为 `1.0`，unsafe execution 和 direct actuator access 均为 `0`。真实浏览器首轮发现的桌面溢出、grid 宽度折叠和令牌入口缺失均修复后复验通过。紧凑证据位于 `artifacts/autonomous_auto10_20260730_evidence/`；500 个音频和逐 case 原始文件保留在 Git 外部并由 SHA 索引。`AUTO-10=PASS`，当前主依赖 stage 仍为 AUTO-05。

## AUTO-13：真实域机器评测（2026-07-30，独立 lane 资源发现）

已实现显式 `--consent` 的相机/视频采集、落盘前隐私区域模糊、棋盘格标定、capture/annotation/calibration 三方 SHA 接入 manifest，以及离散 bbox、区域 mask、hard-negative specificity、map localization 和 synthetic-to-real drop 的统一评测器。程序化 fixture 只验证工具数学和停止边界，不计为真实域数据。

资源发现识别到 1 个 Integrated Camera 和仓库内 249 个图像文件，但没有合格真实 dataset manifest，也没有 20 scene/1000 frame、完整五类/hard-negative、标定和独立 map GT 的组合资源。故 `AUTO-13=BLOCKED_EXTERNAL`、`REAL_DOMAIN_BLOCKED_EXTERNAL=true`、`REAL_DOMAIN_PASS=false`；未执行指标保持 null。紧凑证据为 `artifacts/autonomous_auto13_20260730_evidence/`，其他独立 lane 继续。

## AUTO-04：双模型 micro-overfit（2026-07-30，机器门通过）

已新增直接 anchor-free object detector、独立 leaf/puddle area segmenter、真实 Gazebo micro 数据选择、固定门禁、PyTorch/ONNX parity 和 Docker GPU 执行器。detector 监督目标为 instance bbox 的中心 heatmap、offset 和宽高，输出经 confidence 排序及 class-wise NMS 解码；不使用 segmentation connected-components 作为主 detector。area model 单独计算 leaf/puddle IoU 与 negative-only area FP。

第一轮正式 GPU 运行未通过：direct detector 的 AP50 为 `0.99670`、negative-only FP/frame 为 `0`、ONNX decoded parity 为 `1.0`，但固定 `0.5` 阈值下 macro recall 只有 `0.86087`；三分类 area head 的 leaf/puddle IoU 为 `0.63573/0.29043`、macro mIoU `0.46308`。该失败保留在紧凑证据的 `prior_attempts/` 和 Git 忽略的原始目录。第二轮遵循预定义 fallback，只冻结 detector 阈值为 `0.20`，并把 area 改为独立 leaf/puddle 二值 heads、收紧目标 crop 和负样本候选门。

第二轮正式运行通过：detector AP50 `0.9966997`、三类 recall 均为 `1.0`、negative-only FP/frame `0`、ONNX 最大误差 `1.1444e-05`、decoded agreement `1.0`；leaf/puddle IoU `0.9810641/0.9691405`、macro mIoU `0.9751023`、negative-only area FP/frame `0`、ONNX 最大误差 `9.1553e-05`、argmax agreement `1.0`。紧凑证据位于 `artifacts/autonomous_auto04_20260730_evidence/`，`AUTO-04=PASS`，自主状态推进到 AUTO-05；本结论不外推到跨世界、真实域、J6 或最终竞赛感知。复现和边界见 `docs/auto04-micro-overfit.md`。

## AUTO-03：Oracle 主动观察闭环（2026-07-30，机器门通过）

本阶段在 AUTO-01 的 opt-in `G2-C3` 几何和 AUTO-02 冻结导航配置上实现真实主动观察任务链。Oracle 只发布带噪 XY、协方差、时间戳、通用类别/尺度以及 false/stale 状态，不输出观察位姿、路径或成功状态，也不设置车辆位姿；语义 GT 只进入独立 `auto03_machine_ready_evaluator`，节点图审计确认 planner、Nav2、控制器和执行器均无 GT 订阅。production 默认相机、footprint 和 `enable_training_gt=false` 均未改变。

确定性正式矩阵包含 6 个 G2 Gazebo 世界、60 个 scene、250 条 trial：有效目标 200 条且五类各 40，其中 reachable 170、unreachable/keepout 30；另有 false 30、stale 20。每条可达候选均真实执行 Coverage 边界暂停、观察位姿采样、`ComputePathToPose`、`NavigateToPose`、同步捕获、机器可判定评测、返回边界和 Coverage 恢复。最终路径预检、可达导航、机器可判定和 Coverage 恢复均为 100%；三类拒绝用例均 100% fail-closed，碰撞、keepout 和 GT 控制违规均为 0。

首轮完整矩阵的中心误差 P50/P95 为 `8.53655/20.16039 px`，但目标短边相对误差 P95 为 `0.31173`，未达到 `≤0.30`，因此整轮作为失败证据保留。捕获侧短边模型只在 A–D 世界拟合，E–F 作为留出验证，且规划投影、观察位姿和导航参数不变。完整重跑后 170 个投影样本的中心误差 P50/P95 为 `7.89398/18.43392 px`，短边相对误差 P50/P95 为 `0.09270/0.29771`，中心落入搜索 ROI 和预测/实际 ready 一致率均为 100%；自车像素 P95 为 `0.005495`，目标/自车重叠 P95 为 0。

每个确认目标的中位额外距离/时间为 `0.000128 m/19.502 s`，按 AUTO-02 实测基线计算的吞吐损失为 `19.598%`，低于 25% 硬门。六个世界均记录 19/19 必需话题的 MCAP，候选身份与顺序、任务时间线、消息级指标重算和实际 `ros2 bag play` 全部通过。紧凑证据为 `artifacts/autonomous_auto03_20260729_evidence/`，manifest 和状态不变量均有效；大型原始 MCAP、日志和失败尝试保留在 Git 忽略目录。`AUTO-03=PASS`，自主状态推进到 AUTO-04；真人审计、真实车辆、真实域、J6 和最终竞赛状态未提升。

[PR #35](https://github.com/zhexuexiaotudou/TZcup/pull/35) 的 `fast-validation` 通过后已 squash 合入 `main@c49122113583cf17015989740239128f6341ec41`，主分支 CI run `30488608555` 通过。合并后的远端 main 已复核状态字段、92 个 manifest 管理文件及对应 Git blob，均与本轮通过证据一致。AUTO-03 没有常驻线上服务，部署门以远端发布和既有 Docker/ROS/Gazebo 正式运行证据标记为 `not_applicable`；回滚点为 `82c85c0`。

## 2026-07-29：本机 Ubuntu 24.04 WSLg 图形环境与基础运行链验收

- 在本地 `F:\WSL\TZcup-Ubuntu-24.04` 新建 Ubuntu 24.04.4 WSL2 发行版，安装 ROS 2 Jazzy Desktop、Gazebo Sim 8.11.0、`ros_gz`、Nav2、SLAM Toolbox、Fields2Cover 和项目依赖；环境不依赖 NAS。
- 在干净克隆 `main@11ee369590f543d78eab66b7e790ba27c82cc0d5` 上导入锁定第三方源码并完成全工作空间构建。最终测试为 `449 tests / 0 errors / 0 failures / 49 skipped`。
- WSLg 使用 D3D12/NVIDIA renderer，`glxinfo -B` 为 RTX 4080 Laptop GPU、OpenGL 4.6、`Accelerated: yes`。
- 实际打开并复核 Gazebo 三维清扫场景和 Nav2 默认 RViz 布局；RViz 中可见地图、RobotModel、TF、LaserScan 与 Navigation2 面板。
- 运行中 `sanitation_smoke_check` 返回 `success=true`，11/11 必需 topic 均存在，`missing_topics=[]`。紧凑证据见 `artifacts/wslg_gui_20260729_evidence/`。
- 本轮只补齐本机 WSLg 图形运行与基础 ROS topic 证据；不提升真人审计、真实车辆、真实域、J6 或竞赛效率状态。RViz 的一次 GLSL sampler warning 和 rosdep 的两项上游元数据 warning 保留为非阻塞边界。

## AUTO-02：完整导航回归与配置冻结（2026-07-29，机器门通过）

本阶段没有重新选择 AUTO-01 候选，也没有放宽任何门槛。`auto01_g2_v5_retracted / V5_retracted` 在隔离 worktree 和专用 Docker overlay 中完成五个静态 seed、动态障碍、keepout/限速区、急停、冷启动及 MCAP 回放验证，随后冻结为 `autonomous_navigation_profile_v1`。production 默认配置未改变。

验收结果：

- 静态 `5/5`：每个 seed 均 `17/17`，经验覆盖率分别为 `0.93933/0.92733/0.93467/0.94467/0.93733`，计划覆盖率均为 `0.986`；定位 XY RMSE 分别为 `0.03373/0.04050/0.03153/0.03590/0.03366 m`；碰撞、keepout 和刷盘状态违规均为 `0`。
- 动态障碍 `20/20`，障碍真实移动，碰撞 `0`，最小观测分离距离 `0.60439 m`，高于配置硬阈值 `0.12 m`，Coverage 每次均恢复并最终完成 `17/17`。
- keepout 违规 `0`；限速区均速 `0.28613 m/s`，配置限值 `0.2835 m/s`、允许容差 `0.03 m/s`，故低于验收上限 `0.3135 m/s`。
- 急停 `30/30`，P50/P95/max 为 `0.12142/0.13994/0.14063 s`，每次停止后的命令输出均持续为零，最终刷盘关闭。
- 冷启动 `5/5`，完整 lifecycle、TF、Nav2 和 pointcloud self-filter 参数服务分别在 `24/24/24/24/25 s` 内就绪。
- 五个静态 MCAP 的必需主题为 `15/15`，动态 MCAP 为 `16/16`；Coverage 状态实际录制并回放，使用新增 `/coverage/evaluation_sample` 重算的经验覆盖率与源报告相对误差均为 `0`，同一 evaluation 时间窗内重算的定位 RMSE 相对误差为 `0.280%–0.792%`。

尝试账本保留了两个真实夹具问题：首次静态回放审计错误地要求静态场景中未激活的 `/emergency_stop`，修正为按场景定义契约后复用未修改的 seed0 bag；seed3 首次分配到无效的 `ROS_DOMAIN_ID=233`，失败目录原样保留，随后加入 `0–232` 防护并将静态域移到 `180–184` 后断点续跑。两者都没有被计为算法通过。

紧凑证据为 `artifacts/autonomous_auto02_20260729_evidence/`，包含 acceptance、attempt ledger、冻结配置、运行时参数、六次 replay audit 与逐文件 SHA-256 manifest；大型原始 MCAP 和日志保留在 Git 忽略的 `artifacts/autonomous_auto02_raw_20260729/`。`AUTO-02=PASS`，下一阶段为 AUTO-03。真人审计、真实车辆、真实域、J6 和最终竞赛状态全部未提升。

[PR #32](https://github.com/zhexuexiaotudou/TZcup/pull/32) 的 `fast-validation` 通过后已 squash 合入 `main@6d09e1972526373e1ffdad97ec06c28e02a36e7c`。合并后的远端 main 已再次通过 evidence manifest、Git blob 与 git archive 精确字节校验。AUTO-02 没有常驻线上服务，部署门以远端发布和合并修订的 Docker/ROS/Gazebo 运行证据标记为 `not_applicable`。

## AUTO-01：几何/传感器候选冻结（2026-07-29，机器门通过）

从历史 Stage5BR6W 的 `no_reachable_clean_route` 出发，本阶段没有放宽 Stage4W 覆盖、定位或安全门槛。G1-C1 因联合外包络持续接收 LiDAR 自回波拒绝；G1-C2 因原始 scan 不具备高度语义而在正式运行前拒绝；G1-C3 完成两次 17/17 覆盖，但定位 RMSE 分别为 `0.11727 m` 和 `0.10863 m`，超过 `0.05 m` 硬门。G2-C1 的水平相机无法保护低障碍；G2-C2 的未滤波向下相机在空场看见车体自身，导致 transit 无法推进。

最终 G2-C3 使用固定回收态 `V5_retracted` 相机（安装中心 `[0.36, 0, 0.66] m`、俯角 `-50°`），相机碰撞 AABB 位于冻结导航 footprint 内且高于 LiDAR 平面。新增 `pointcloud_self_filter` 将验证相机点云转换到 `base_footprint`，以 `[-0.60,-0.43,-0.20]` 至 `[0.72,0.43,0.75] m` 的已知车体包围盒做自滤波，再发布 `/verification_camera/depth/color/points/navigation`。该话题和未掩膜 `/scan` 由单个 Collision Monitor 融合；生产默认配置不启用这条 opt-in 链。

验收结果：

- 快速回归 `81/81`；离线几何与物化配置审计全项通过。
- 冷启动 `3/3`，参数服务分别在 `25/25/26 s` 内就绪，点云、验证相机和运行时参数均实际到达。
- seed0 正式覆盖 `17/17`，经验覆盖率 `0.932`，碰撞 `0`，keepout 违规 `0`，定位 RMSE `0.0339096 m`，swath 冲突 `0`，合法 staging 与 MCAP 回放通过。
- 低障碍 `30/30`、高障碍 `30/30`，保护触发 `60/60`，碰撞 `0`，误判安全 `0`。首轮 30+30 因 Gazebo `set_pose` 单次服务超时未形成结果，夹具加入最多三次有限重试后从头完整重跑；没有把未完成运行计为通过。

紧凑证据为 `artifacts/autonomous_auto01_20260729_evidence/`，原始 bag、日志和所有拒绝尝试保留在本地工作树外的 Git 忽略路径。`AUTO_STAGE_01_PASS=true`；下一阶段为 AUTO-02。真人审计、真实车辆、真实域与 J6 状态均未被提升。

[PR #30](https://github.com/zhexuexiaotudou/TZcup/pull/30) 的 `fast-validation` 通过后已 squash 合入 `main@4e6c490df5a99b57645aec3cd8defc785e9dcd88`。合并后的远端 main 已再次通过 evidence manifest、Git blob 与 git archive 精确字节校验。AUTO-01 不是常驻服务，部署门以该远端发布和合并修订的 Docker/ROS/Gazebo 运行证据标记为 `not_applicable`；不虚构在线生产部署。

## AUTO-00：自主控制面（2026-07-28，机器门通过）

基线冻结为远端 `main@ac6d5697427425c438ff0f42780ff6ab772226f9`，独立分支为 `agent/autonomous-final`。规划包的 11 个文件已完成逐文件 SHA-256/字节校验。当前已实现 17 阶段 registry、DAG 计划、原子状态文件、执行锁、依赖调度、断点续跑、幂等证据复用、统一 evidence manifest、状态防伪、secret scan 和带显式执行保护的 GitHub 适配器。

AUTO-00 的机器门已通过：`ci_fast` 为 76/76，registry/state/plan 一致，依赖环为 0，断点续跑与幂等重跑测试通过，状态防伪、secret scan、diff 检查和逐文件 evidence manifest 均通过。证据目录为 `artifacts/autonomous_auto00_20260728T161119Z_evidence/`，其中 baseline hash audit 覆盖 475 个历史 review 文件。历史 Stage4W–Stage5BR6W evidence 未修改；Stage5BR6-A 人工完成/人工审计标志保持 false，Stage5BR6W 首个阻断层保持 `no_reachable_clean_route`。

[PR #28](https://github.com/zhexuexiaotudou/TZcup/pull/28) 的 `fast-validation` 通过后已按 merge-commit 策略合入 `main@14dc0ecaa0b2cd21d7a7359b4bcd8db62dd2b40b`。合并后的远端 main 再次通过 evidence manifest、Git blob 和 git archive 精确字节校验。AUTO-00 是离线控制面，运行时部署门不适用；该远端仓库复验是本阶段发布验收。下一阶段为 AUTO-01，独立 AUTO-04/09/10/11/12/13/14 lane 可并行调度。

## Stage5BR6W：人工门豁免工程支线（2026-07-21）

状态：完成 Phase 0–3 和 observation planner 加固；真实 Phase 4 seed 0 失败后按协议停止，未进入 Oracle。

已实现独立 engineering waiver，保留 `AWAITING_HUMAN_REVIEW=true` 与全部正式 false 状态；Reviewer A/B 包和 sealed truth 未修改。V4 只冻结为 engineering verification candidate；工程 policy 使用新 ID 与 SHA，明确 `human_validated=false`、`competition_metric_eligible=false`。candidate footprint 由 V4 AABB、production footprint 与 0.03 m 安全裕量推导，并通过双 opt-in profile 接入实际 V4 相机、local/global costmap、Collision Monitor、Coverage mission geometry 和 planner。运行时 footprint audit 全部通过，production 默认未改变。

Observation planner 已增加完整 camera SE(3)、V4 侧向偏置、实际 CameraInfo、整多边形边界/keepout 相交、global costmap footprint cost、位姿相关 target/self overlap、ROI/short-side、路径长度/转向/clearance 代价；工程输入缺失和无可行 pose 均 fail-closed。快速门为 73/73，针对性 planner 测试为 6/6。

真实 Stage4W seed 0 使用 candidate footprint 半径 `0.856825 m`、headland `1.35 m`。运行时 local/global costmap 与 Coverage 加载同一多边形；规划本身成功且计划覆盖率 `0.96226`，但 cleanable area 仅 `6.89 m²`，9 条 swath 全部与膨胀 exclusion 相交。正向 staging 可达却位于 operation polygon 外，反向 staging footprint cost 99 且 `NO_VALID_PATH`，最终 `no_reachable_clean_route`、组件 0、经验覆盖率 0。碰撞/keepout 为 0、刷盘最终关闭、定位 RMSE `0.04333 m`，但不能补偿完整任务失败；Coverage state 未进入 bag，replay=false。

PR #26 合并后使用同一代码树做了两次独立容器复验：首次在 Nav2 参数服务慢启动处超时，第二次完整复现 candidate footprint 一致性、`6.89 m²` cleanable area、9 条 swath 冲突和 `no_reachable_clean_route`，随后仍因 Coverage state 不存在而在 replay 等待处退出 124。第二次覆盖期定位 RMSE 为 `0.05342 m`，超过 `0.05 m` 门槛；这说明工程支线除几何不可达外还存在运行间定位波动，不改变所有 readiness=false 的结论。两次 post-merge 原始日志和 MCAP 只在本地保留，不进入 Git。

停止边界：static 仅执行 1/5 且 0/1 通过；dynamic 0/20、estop 0/30、Oracle 0 world/0 scene/0 candidate。`READY_FOR_STAGE5BR6W_ORACLE_ENGINEERING=false`、`READY_FOR_STAGE5BR7_ENGINEERING=false`，正式人工与 Stage5B readiness 全部保持 false。紧凑证据见 `artifacts/stage5br6w_20260721_review/`，完整失败日志与 bag 保留在本机 `artifacts/stage5br6w_20260721_runtime/footprint_regression_retry2/static/seed_0/`。

## Stage5BR6-A：双人盲审交付准备

状态：交付包已准备，等待两名独立真人评审；后续门禁未启动。

已完成：

- 将 Stage5BR5 预注册候选固定为 V4，但保持 `camera_selected=false`。
- 保留五类各 40 张的 200 个正样本，并从 Stage5BR6 训练专用 label=0 几何世界通过真实 V4 Gazebo RGB/depth/semantic/instance 精确同步链采集 70 个 no-target/hard-negative；同色非垃圾、瓶/罐形障碍、非积水湿地面、阴影、非目标落叶背景、车辆自身结构和裁剪边界伪影各 10 张，生产世界未修改。
- 每个负样本裁剪均以 semantic mask 检查，目标像素总数为 0。
- 生成 Reviewer A/B 两个独立 ZIP；随机顺序、opaque ID 与 package ID 均不同，无 Git 路径、world、camera 或 truth 映射泄漏。
- ZIP CRC、逐文件 SHA、PNG 元数据、response 模板空值和 sample ID 集合校验通过。
- 提供真人回收完整性校验脚本；脚本不会生成、补全或修正人工回答。

当前边界：

- `AWAITING_HUMAN_REVIEW=true`，真人 response 为 `0/2`。
- `READY_FOR_STAGE5BR6_ORACLE=false`、`READY_FOR_GPT_REVIEW_STAGE5BR6=false`、`READY_FOR_STAGE5BR7=false`。
- V4 相机契约与 policy v2 未冻结；production footprint 未修改。
- candidate-footprint Stage4W 回归、Oracle 主动观察、detector/area model 训练和 J6 均未执行。

证据：

- `artifacts/stage5br6_20260721_review/stage5br6_status.json`
- `artifacts/stage5br6_20260721_review/human_handoff_manifest_redacted.json`
- `artifacts/stage5br6_20260721_review/human_handoff_integrity_report.json`
- 本机 Git 忽略交付目录：`external_review_handoff/stage5br6/`

## Stage5BR5：相机机械重构、平衡盲审与主动观察基础（2026-07-20）

ActiveObservation 已把 `first_seen_s`、`last_seen_s`、排队、preflight、approach、动态 deadline 与最近 observation 时间分开；重复 discovery 刷新末见时间，sensor stale 和 queue timeout 独立，空间合并允许模型 ID 变化，旧记录可迁移。几何 planner 以 cleanable/keepout、footprint clearance、协方差、预期像素/ROI、自遮挡、视角、路径长度和转向代价选择候选；ROS 2 wrapper 实际调用 `/compute_path_to_pose` 且不使用 GT 输出位姿。

机械网格中 V1/V2/V4 通过，V3 因旋转相机 AABB 超出 trial footprint 剔除。V1/V2/V4 各在 6 world × 2 role × 10 frame 完成真实 Gazebo 采集，共 360 帧，精确四传感器时间戳、自像素 P95、target/self overlap 与物理门通过。v2 ready fraction 为 `0.13450/0.13636/0.30508`；这些只是 view-level 结果，不是主动观察闭环转换。

盲审数据经多轮固定 seed 补样后达到 200 张、五类各 40 张并覆盖六世界。当前没有两名独立人工评审，manual accuracy、Cohen kappa 与 self-occlusion failure 均为 null，相机没有选择，policy v2 为未冻结且 training disabled 的草案。首个阻断层为 `G2_camera_selection_blocked_two_independent_human_manual_reviewers_not_available`。正式 oracle active-observation、detector/area micro-overfit、120/1200、formal/live/J6 按门禁未执行；Stage5BR4 和更早结论未改写。

回归已重新执行：`ci_fast` 68/68；`sanitation_learning` 与 `sanitation_spot_cleaning` colcon build 通过、29 tests/0 failures；Stage5A 为 30/30 spot-clean、119 帧 live、GT control violation 0 且 rosbag 已录制；Stage4W seed0 为 17/17 组件、经验覆盖率 `0.944`、定位 RMSE `0.03737 m`、碰撞/keepout/brush violation 0 且 replay 通过；生产默认运行时 GT 隔离通过。

## Stage5BR4：可观测性、相机消融与主动观察（2026-07-20）

状态：复核材料完整，Stage5B/Stage5C readiness 均为 false。C0 原始 3370 可见实例只有 875 个满足冻结的 recognition-ready；C0–C3 五段真实采集均完成 10/10 同步帧。C3 verification 的 ready 比例为 29.63%，但 discovery non-ready 到 verification ready 的实例转换仅 50%，低于 90% 门，且 self-pixel P50 为 21.11%。人工可辨识审计失败，首个阻断层为 `G2_camera_selection_blocked_active_observation_ready_conversion_below_0.90_and_manual_audit_failed`。

已实现相机 mount 参数化、默认关闭的物理 C3 verification RGB-D/GT、冻结策略哈希、all-visible/ready/non-ready 报告，以及包含去重、路径/keepout/footprint/visibility preflight、stale/timeout/最大接近和代价记录的主动观察状态机。生产默认隔离通过。因相机选择失败，真正 detector、area 模型、micro-overfit、120/1200、screening、formal、正式 live、真实 active Nav2 和 J6 均未执行；Stage5BR3 三次旧模型结果未改写。

## Stage5BR3：真实车辆 G2 数据、逐实例 QA 与 split-model screening

Stage5BR3 将 `artifacts/stage5br2_*_review/**` 改为 binary，避免 Git blob 与 Windows 导出包发生 LF/CRLF 证据字节漂移；同时废弃独立静态相机 rig，训练 GT 传感器只在显式 `enable_training_gt:=true` 时挂到生产车辆 `camera_link`，生产默认渲染和运行时均无 semantic/instance GT。

最终 G2 有 6 个不同 SHA、材料和几何布局的世界，按 3 train / 1 val / 2 test 隔离。六世界真实消息门全部通过：640×480 RGB/depth/semantic/instance 非空、CameraInfo 有效、四传感器精确同时间戳、光学帧统一为 `camera_depth_link`、深度为 32FC1 且有限值处于 0.3–100 m、base→camera 外参为 `[0.53, 0, 0.22] m`，实际车辆 2 秒移动约 0.70 m。

原生数据一次采集 80 scene/800 frame、约 2.225 GB。第一次 QA 因 12 个 hard-negative 资产跨 split 复用且 negative-only 场景数为 0 而失败；将 hard negatives 固定拆为 8/2/2 并强制 5 个 negative-only 种子后重采。第二次 QA 为 80/800、标注完整率 100%、target/negative/trajectory leakage 0、跨 split exact/pHash duplicate 0、semantic-instance 错误率 0，hard-negative 数覆盖 0–8，最终通过。

四档离线扫描选择 640×384 与 512×384；实际在 512×384 执行 3 次 split-model 尝试。最佳 detector cross-world F1/AP50/AP50:95/small recall 为 `0.1311/0.3484/0.1075/0.4512`，最佳颜色压力 F1 `0.1018`，最低 negative-only FP `8.7/帧`；area cross-world mIoU `0.02346`。未达到 screening 门，故停在 `G2_split_model_screening_gates_failed_after_3_attempts`。没有执行 500/5000、live、真实 Nav2、真实域、J6 或竞赛效率门；`1053 m²/h < 3500 m²/h` 不变。

回归方面，`ci_fast.py` 68 项、`sanitation_learning` 11 项和三包 colcon build/test 通过；Stage5A 为 30/30 spot-clean、132 帧 live 且 GT control violation 0；Stage4W seed 0 完成 17/17 组件、覆盖率 `0.93533`、定位 RMSE `0.03572 m`、碰撞/keepout/brush violation 均为 0。完整运行日志留在本地，紧凑机器摘要为 `artifacts/stage5br3_20260720_review/stage5br3_regression_summary.json`。

## Stage5BR2：G2 车载相机基础恢复与 fail-closed 边界

- 从当前车辆 Xacro 提取 `camera_link` 相对 `base_link` 外参 `[0.53, 0, 0.22] m`、`camera_depth_link`、640×480、水平 FOV `1.50098 rad`、15 Hz，并校验 `sim.launch.py` 的生产 ROS 话题映射。
- 建立四个 G2 世界与 2/1/1 train/val/test world-isolated split；4 个 world SHA 和 4 种材料均不同，资产为项目自制 Apache-2.0 程序化几何、scale 1.0。
- Gazebo Harmonic 逐世界实际启动通过，RGB、深度、semantic GT、instance GT 四类话题齐全；GT 为 training-only，生产 launch 未修改。
- 修正指标语义：历史 G1 `cross_asset_world` 规范化为 `cross_asset_same_world`，单世界 `cross_world=null`；新增逐 instance-id bbox、最短边、mask area、距离、遮挡和 `not_visible` 统计。
- ROS-independent 快速门通过：68 tests。当前尚未采集 G2 80 scene/800 frame，故分辨率实测、detector/area segmenter、500/5000、live、真实 Nav2 和 J6 均未执行。
- 首个阻断层：`G2_screening_dataset_80_scene_800_frame_not_executed`；`READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`。
- 证据：`reports/reviews/GPT_REVIEW_STAGE5BR2.md`、`artifacts/stage5br2_20260720_review/`、`docs/stage5br2-g2-vehicle-camera.md`。

## Stage5BR：Gazebo-camera 数据恢复、训练链审计与泛化修复

状态：G1 数据 smoke 通过，学习模型 screening 失败并按停止条件冻结。

已完成：

- 12 帧 micro-overfit 达到 macro F1 `0.98124`、foreground mIoU `0.96333`。
- PyTorch/ONNX/ROS parity 达到最大 logit error `6.866e-05`、argmax agreement `1.0`。
- 新增 Gazebo Label system、共视场 RGB-D/semantic/instance cameras、scene/lighting 随机化和 exact timestamp collector。
- G1 50 scene/500 frame：annotation completeness `1.0`、label consistency error `0`、asset leakage `0`、跨 split exact/pHash duplicate `0/0`。
- 三次 G1 model screening 均失败；最佳 cross asset/world F1 为 `0.65804`，最佳 color stress F1 为 `0.47647`。
- Stage5A 回归通过：30/30 synthetic spot-clean，live 186 帧、MCAP true、GT control violation 0。
- Stage4W seed 0 回归通过：17/17、coverage `0.936`、RMSE `0.03260 m`、零碰撞/keepout/brush violation。

停止边界：

- 不生成 500 scene/5000 frame formal G1；
- 不运行 30 seed/10 min formal live；
- 不运行真实 Nav2 spot-clean；
- R1、J6 实板、竞赛感知和 `1053 < 3500 m²/h` 效率门保持 false。

证据：`reports/reviews/GPT_REVIEW_STAGE5BR.md`、`docs/stage5br-gazebo-camera-recovery.md`、`artifacts/stage5br_20260719_review/`。

## Stage5B：学习型感知、域隔离与颜色捷径筛查

状态：已形成可复核的失败边界，未通过 Stage5B，未进入 Stage5C。紧凑证据包完整，但 `READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`。

已完成：

- 新增 `sanitation_learning`，含五类各六变体、12 个硬负样本、许可清单、scene/asset/texture/world 隔离、RGB-D/semantic/instance/map-pose/COCO 生成、标注 QA、训练、ONNX 评测、颜色压力和 J6 预检。
- 候选 A 为已训练 1×1 Conv 基线；候选 B 为 137,078 参数、6 Conv + 5 ReLU 的上下文模型；候选 C 因 ONNX/J6 算子风险明确 deferred。选择未使用测试集。
- 三次结构性筛查后冻结：最佳验证 macro F1 `0.38637`，100 个未见 scene / 1000 帧离散 macro P/R/F1 `0.00752/0.00784/0.00768`，leaf/puddle IoU `0.00376/0.2494`，颜色压力 aggregate macro F1 `0.05192`；map RMSE `0.09731 m` 是唯一主要精度通过项。
- 修正评测命名：无置信度排序 PR 曲线时，`ap50`/`ap50_95` 为 null；实际 IoU 匹配分数使用独立字段，禁止冒充 AP。
- 训练模型真实接入 Gazebo RGB-D/TF/ONNX Runtime 链，处理 161 帧并发布分割与 map targets，且 `ground_truth_input_used=false`；该运行只作为接口诊断，正式 30 seed/10 分钟门为 false。
- 回归通过：`py scripts/ci_fast.py` 为 57 passed；Stage5A 固定颜色离线/30 次状态评测/实时 Gazebo 通过；Stage4W seed 0 为 17/17、经验覆盖率 94.2%、碰撞/keepout/刷盘违规 0。

停止边界：

- 当前 D1 数据是程序化 renderer，不是 Gazebo camera 实际渲染；500 seed/5000 帧正式集未执行。
- 颜色捷径和未见泛化失败后，按规划包停止条件不执行 30-seed 正式实时门与 30 次真实 Nav2 spot-clean，避免用运行可达性替代精度。
- D2 无授权真实数据；J6 官方工具链、转换/量化和实板 FPS 均无证据；理论效率 `1053 m²/h < 3500 m²/h`。

复核入口：`reports/reviews/GPT_REVIEW_STAGE5B.md`、`docs/stage5b-learned-perception.md` 与 `artifacts/stage5b_20260719_review/`。原始三次筛查、Docker workspace、数据卷与 rosbag 在用户确认前保留本机。

## Stage5A：垃圾感知真值闭环、数据集与定点清扫

状态：正式实现已覆盖 registry、GT、20-scene 数据、ONNX Runtime、RGB-D 到 map、多帧 tracker、30-seed synthetic task-state E2E 和 Stage4W 回归。紧凑复核目录的 9 个机器 gate 全部通过，`READY_FOR_GPT_REVIEW_STAGE5A=true`、`READY_FOR_STAGE5B=true`。

已验证边界：Stage5A 仅为 synthetic-domain 工程证据。30-seed 状态闭环不等于 30 次真实车辆/Nav2 定点任务；J6 工具链/量化/运行、真实数据精度、原生 GUI、实车、机械臂与竞赛效率仍未通过。详细复现与结果见 `docs/stage5a-garbage-perception.md` 和 `reports/reviews/GPT_REVIEW_STAGE5A.md`。

## Stage4W：可达清扫域、完整覆盖与动态交互闭环

状态：Stage4W 正式门禁全部通过并已作为 Stage5A 回归基线；当前阶段状态见上方 Stage5A 条目。

已完成：

- 修复 GNSS 协方差建模、refined/GNSS 有界权重和全局锚点随局部里程计传播；正式 hybrid 10-seed 为 10/10，XY RMSE P50/P95/max `0.02825/0.03726/0.03778 m`，导航、TF 单所有者、扫描精化参与均为 10/10，GT 控制违规 0。
- 建立唯一 mission geometry：outer、headland、keepout、显式 exclusion、world→map 固定障碍、footprint 和安全裕量共同编译。当前生成 9 swath + 8 turn = 17 组件；Stage4V 的固定 23 组件来自旧几何，Stage4W 标记为不适用。
- 同时预规划正/反 staging，等待全局 costmap 覆盖候选点，核对 cost/keepout/speed mask 与 footprint clearance，再以明确 approach yaw 执行 transit 和 brush-off 稠密 entry。
- 为 NavigateToPose、ComputePathToPose 和 FollowPath 使用各自动作错误码语义；FollowPath 104 被正确识别为 `PATIENCE_EXCEEDED`。Nav2 使用 `PoseProgressChecker`，controller 有界容忍 5 s。
- 动态障碍通过持久 ROS–Gazebo SetEntityPose 服务桥横穿；同一组件注入间距至少 0.5 m。局部/全局 obstacle layer 启用无限量程清障，消除障碍移走后的旧标记。
- 正式静态 5-seed 全部通过：每次 17/17、经验覆盖率 `92.93%–94.53%`、覆盖期 RMSE `0.02930–0.04620 m`、碰撞/keepout/刷盘违规均为 0、刷盘最终关闭、回放 5/5。
- 正式动态任务通过：20/20 有效交互、碰撞 0，完整任务 17/17、覆盖率 93.53%、覆盖期 RMSE 0.03014 m；keepout 违规 0、限速区平均 0.288 m/s。
- 30 次急停全部归零与释放恢复，P95 `0.188 s`；停止上游命令后 `1.694 s` 达到连续 5 帧稳定零输出。动态 MCAP 完整回放通过。

边界：

- `READY_FOR_GPT_REVIEW_STAGE4W=true`、`READY_FOR_STAGE5A=true` 只表示 Stage4W 技术门满足；Stage5A 已在后续独立阶段实施并保留新的合成域边界。
- 竞赛理论效率仍为 `1053 m²/h < 3500 m²/h`，`competition_efficiency_pass=false`；不得以经验覆盖率替换效率门。
- 垃圾感知训练、J6 量化和实板部署未执行；原生 Ubuntu/WSLg GUI 的历史缺口已由 2026-07-29 本机 WSLg 基础图形验收补齐。
- 紧凑证据位于 `artifacts/stage4w_20260717_review/`；原始 MCAP、筛查和失败诊断在用户确认前保留本机。

复核入口：`reports/reviews/GPT_REVIEW_STAGE4W.md`、`artifacts/stage4w_20260717_review/stage4w_summary.json` 与 `MANIFEST.json`。

## Stage4V：混合定位与完整任务复核

状态：正式混合定位门禁通过，完整 Coverage 门禁失败；未进入 Stage5A。

新增 `sanitation_scan_refiner`、`sanitation_gnss_sim`、混合全局融合器和 TF 所有权审计。正式 10-seed 的 XY RMSE P50/P95/max 为 `0.033438/0.037916/0.038717 m`；定位、导航、TF 单所有者及扫描参与均为 10/10，GT 控制违规 0。完整任务随后真实运行：规划覆盖率 97.5%，但 transit-to-start 超时/终止，完整执行 false、经验覆盖率 0%；动态障碍有效交互 0/20、碰撞 0；keepout 违规 0、速度区通过；30 次急停 P95 `0.1705 s`；MCAP 融合位姿回放通过。理论效率 `1053 m²/h` 未达 `3500 m²/h`。最终 `READY_FOR_GPT_REVIEW_STAGE4V=false`、`READY_FOR_STAGE5A=false`。

证据入口：`GPT_REVIEW_STAGE4V.md`、`artifacts/stage4v_20260716_review/`；原始 10-seed、Coverage 和 MCAP 在用户确认前保留。该根目录兼容入口由历史二进制清单固定引用，不随其他评审包迁移。

## Stage4U：坐标标定、定位地图与 5 cm 定位闭环

状态：达到可复核失败边界；未通过 Stage4U，未进入 Stage5A。

已完成：显式冻结 SE(2) 坐标标定；map-relative/地理配准/absolute 误差解耦；Jazzy `nav2_msgs/msg/ParticleCloud` 与 best-effort QoS 修复；加权粒子统计；地图生成/基础质量/定位几何三级门；M1/M2/M3 与 AMCL/SLAM Toolbox 对照；360@10 与 720@20、AMCL profile 灵敏度；结构化 v2 场景；正式串行 Oracle 10-seed。

正式最优候选为结构化 v2、0.02 m surveyed reference、AMCL 精度 profile、360@10 Hz。10/10 seed 完整，10/10 导航成功，TF 全连续，粒子仪器全有效，恢复 0 次；XY RMSE P50/P95/max 为 `0.067669/0.079833/0.080218 m`，worst 为 seed 7。首个真实失败层仍是 `oracle_localization_pass`。

边界：M2 posegraph 已序列化，但没有执行独立离线优化/重渲染；M3 是定位参考图，不是 SLAM 建图成绩；realistic、完整 Coverage、动态障碍和急停按停止条件未执行；理论效率仍为 `1053 m²/h`，`READY_FOR_GPT_REVIEW_STAGE4U=false`、`READY_FOR_STAGE5A=false`。

复核入口：`reports/reviews/GPT_REVIEW_STAGE4U.md`、`artifacts/stage4u_20260716_review/stage4u_summary.json`、`oracle_10seed_compact.json` 与 `MANIFEST.json`。

发布与合并后验证：[PR #9](https://github.com/zhexuexiaotudou/TZcup/pull/9) 的 `fast-validation` 通过，已按 merge-commit 策略合入 `main@efd5e34cbb3c8ba1016118c63a6e35402704e787`。远端 main tree `00f2b33c5866025421bc5e9bea224945b58eafbd` 与本地验证树一致；合并树真实 Gazebo core smoke 再验通过 covariance 与 operational envelope，MCAP 17.5 MiB、48,255 条消息且元数据可读。回滚点为 `de5106cdaf0948888c0225a1076cad790280efa3`。

## Stage4T：转向瞬态、EKF 融合与定位恢复

状态：到达可复核失败边界；未通过 Stage4T，未进入 Stage5A。

已完成：

- 固定时长瞬态 `200/200`、闭环航向 `120/120`；实际 `/cmd_vel` 积分、完整逐 trial 指标和重复性均保留，GT 控制违规为 0。
- precision/coverage 运行包络真实输出越界为 0；0.60 rad/s stress 失败原样保留且默认禁用。
- 原始全零 covariance topic 保留；项目 measurement adapter 发布非零 YAML 化 wheel/IMU covariance，真实 core smoke 通过。
- A/B/C/D 各 5 次同动作集消融完成，选择 EKF-B；可选 chassis yaw-rate controller 记录为 `not_needed`。
- 0.05/0.02 m 地图均以 selected EKF 自动闭环路线重建，0.05 m 质量门通过并选中；SDF 刚体配准几何指标、overlay、keepout/speed masks 和建图 MCAP 均保留。
- Oracle 正式 10-seed 达到 10/10 导航成功、TF 全连续、粒子退化 0，但 XY RMSE P50/P95/max 为 `0.08397/0.14848/0.16972 m`，超过 `0.05 m` 硬门。

当前边界：

- 第一真实失败层为 `oracle_localization_pass`，根因指向 SLAM 地图的非刚性几何误差与稀疏场景 AMCL 匹配精度。
- 按 Stage4T 停止条件，没有执行 realistic 全量 10-seed、完整 Coverage、20-seed 动态障碍、30 次急停或完整任务 rosbag replay。
- `READY_FOR_GPT_REVIEW_STAGE4T=false`，`READY_FOR_STAGE5A=false`；`competition_efficiency_pass=false`，理论效率仍为 `1053 m²/h`。

复核入口：

- `reports/reviews/GPT_REVIEW_STAGE4T.md`
- `artifacts/stage4t_20260715_review/stage4t_summary.json`
- `artifacts/stage4t_20260715_review/MANIFEST.json`

发布与合并后验证：

- [PR #7](https://github.com/zhexuexiaotudou/TZcup/pull/7) 的最新 `fast-validation` 已通过，随后按仓库 merge-commit 策略合入 `main@2412300192d6f4204e0049e55c06ba69353377ba`；回滚点为 `b7734801d775740dccf6ce16a12f6e739b2e8136`。
- 远端 main tree `cc9698b3167b37999592613db73f3e08af79cbcc` 已在独立部署副本中执行真实 Gazebo core smoke：covariance 与运行包络均通过，实际速度越界为 0，MCAP 为 17.9 MiB、49,437 条消息且元数据可读。
- 合并后 core smoke 不改变 Stage4T 停止结论：0.60 rad/s stress 仍失败，完整瞬态/EKF/地图/Oracle 证据继续以复核目录为准。

## Stage4S：运动模型标定与定位闭环

状态：已到达可复核失败边界，未通过 Stage4S，未进入 Stage5A。

已完成：

- 新增模型级 Gazebo `OdometryPublisher` 真值源 `/ground_truth/model_odom_raw`，严格校验 `world` 与 `sanitation_vehicle/base_footprint`，移除生产路径对匿名 `Pose_V.transforms[0]` 的依赖。
- 通过出生点、静止 20 s、前进 1 m、正负 90°、world→map_gt 变换和实体稳定性自证。
- 建立使用仿真时钟、无障碍专用世界的 13 段开环实验台，并记录命令、关节、raw odom、IMU、EKF、真值、TF、段标记和完整 MCAP。
- 解耦 physical 与 DiffDrive 参数，完成轮半径 5 点、轮距 9 点粗细网格；选择 `drive_wheel_radius=0.14 m`、`drive_wheel_separation=1.22 m`。
- 完成 5 点摩擦/WheelSlip 最小网格。降低横向摩擦或启用 WheelSlip 均显著恶化高速转向，默认接触为网格最优。

当前边界：

- 首个失败层为 `layer_1_body_command_tracking`。
- 5 m 直线、低速正反整圈和四个圆弧半径通过；高速 `0.60 rad/s` 正转整圈车体 yaw 误差为 `19.1825°`，门槛为 `≤18°`。
- raw wheel odom 与 IMU 初步门槛通过，但不能跳过 Layer 1 直接做 EKF 消融。
- Stage4S-5 至 Stage4S-9 未执行；`READY_FOR_GPT_REVIEW_STAGE4S=false`、`READY_FOR_STAGE5A=false`。
- 垃圾感知训练、J6 量化和实板部署均未开始。

复核入口：

- `reports/reviews/GPT_REVIEW_STAGE4S.md`
- `artifacts/stage4s_20260715_review/stage4s_summary.json`
- `artifacts/stage4s_20260715_review/manifest.sha256`

## Stage 0：预检与基线锁定

状态：已通过（容器 headless 预检门）。

已完成：

- 将用户提供的完整推进包扁平导入仓库根目录；原工作区为空，没有覆盖既有成果。
- 校验 `MANIFEST.json` 的 35 个条目均存在且字节数一致。
- 初始化 Git，并以独立基线提交保存原始推进包。
- 完成 Windows 宿主、GPU、磁盘、WSL、Docker 与本机 ROS 工具 inventory。
- 实时核查并锁定 Linorobot2、OpenNav Coverage 和 Fields2Cover 上游版本。
- 修复预检脚本，使其输出结构化检查、阻塞项、告警、命令路径与原始探针结果。
- 修复第三方导入策略：使用精确 commit、拒绝覆盖 dirty checkout、校验最终 SHA。
- 构建 `tzcup/sanitation-jazzy:stage0` 验证镜像并执行预检；脚本返回 0。
- 验证 Ubuntu 24.04.4、ROS 2 Jazzy、Gazebo Sim 8.11.0、`ros_gz`、colcon、rosdep、vcs 全部可用。
- 验证 `ros-jazzy-fields2cover` 2.0.0 二进制包可安装。

当前边界：

- Windows 宿主不满足直接运行 ROS 2 Jazzy/Gazebo Harmonic 的要求。
- Docker 可作为 Ubuntu 24.04/Jazzy headless 构建通道；GUI 已由 2026-07-29 本机 Ubuntu 24.04 WSLg 复核，动力学与算法门仍以对应历史运行证据为准。

证据：

- `artifacts/preflight.json`
- `artifacts/stage0_20260714_*/preflight.json`
- `artifacts/stage0_20260714_*/preflight_run.log`
- `artifacts/stage0_20260714_*/host_inventory.json`

复现命令：

```powershell
docker desktop start
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_docker_preflight.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\collect_stage0_evidence.ps1
```

## Stage 1：工作空间可重复构建

状态：已通过。

已完成：

- 在全新隔离工作空间中导入 starter 包、Linorobot2 和 OpenNav Coverage。
- 完成 rosdep 安装；`micro_ros_agent` 仅用于真实硬件路径且 Jazzy rosdep 无对应键，因此在仿真构建中显式跳过。
- 连续执行两次 `colcon build --symlink-install` 和两次测试。
- 增加 `sanitation_tasks` 的项目自有 pytest，验证冒烟检查所需的运动、传感器、相机与 TF topic 集合。
- 上游 `linorobot2_gazebo` 没有 pytest 用例（pytest code 5），因此从测试 lane 明确排除；上游 CMake `xmllint` 依赖在线 ROS schema，改由离线 XML well-formedness 检查覆盖。其余上游 lint、GTest 和项目测试均执行。
- 两次测试结果均为 275 tests、0 errors、0 failures、44 skipped；跳过项来自 cppcheck 对当前 2.13.0 慢版本的上游保护逻辑。
- 构建前后第三方仓库 SHA 一致且 `dirty_files=0`。

证据：

- `artifacts/stage1_20260714_154523/stage1_summary.json`
- `artifacts/stage1_20260714_154523/build_1.log`
- `artifacts/stage1_20260714_154523/build_2.log`
- `artifacts/stage1_20260714_154523/test_results.txt`
- `artifacts/stage1_20260714_154523/third_party_status_after.txt`

复现命令：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage1_docker.ps1
```

## Stage 2：车辆 URDF、场景与运行闭环

历史状态：headless GPU 验收已通过；当时 GUI 截图仍需原生 Ubuntu 24.04 或 WSLg 复核。该图形缺口已于 2026-07-29 在本机 WSLg 补齐。

已完成：

- 重写本项目 `sim.launch.py`，在 Jazzy 上以字符串参数加载 `robot_description`，组合 Gazebo server、可选 GUI、实体生成、ROS-Gazebo bridge、EKF 与命令超时保护。
- 建立参数化 4WD 清扫车：0.65 m 清扫 footprint、40 L 尘箱、四轮、双刷、LiDAR、RGB-D、IMU 与 `arm_mount_link`。
- 移除上游模型级重复 Sensors system，消除同一场景被创建两次导致的 Ogre2 重复材质和崩溃。
- 使用 Gazebo Harmonic Ogre2 headless rendering 和 Docker NVIDIA GPU passthrough 实际运行仿真。
- 静态验证 URDF、由 URDF 转换的 SDF 和场景 SDF。
- 新增运行探针，订阅时钟、TF、双路里程计、关节、IMU、LiDAR、RGB、深度和点云，并发送 5 秒速度指令验证实际动力学位移。
- Stage 2 实测 12/12 类话题均有消息；车辆位移 1.18725 m，阈值 0.01 m；仿真在证据采集期间保持存活。
- 给 launch 清理增加有上限的 INT/TERM/KILL 阶梯，避免 Gazebo 子进程造成 CI 假卡死。

证据：

- `artifacts/stage2_20260714_163402/stage2_summary.json`
- `artifacts/stage2_20260714_163402/runtime_probe.json`
- `artifacts/stage2_20260714_163402/simulation.log`
- `artifacts/stage2_20260714_163402/nodes.txt`
- `artifacts/stage2_20260714_163402/topics.txt`
- `artifacts/stage2_20260714_163402/gz_topics.txt`

复现命令：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage2_docker.ps1
```

## Stage 3：SLAM、定位、Nav2 与安全

状态：运行门已通过；定位精度仍是进入 Stage 4 前必须显式携带的风险。

已完成：

- 新增 `sanitation_navigation`，提供 SLAM Toolbox、地图保存、AMCL/Nav2、Regulated Pure Pursuit、车辆 footprint、keepout filter 与 speed filter 配置。
- 解决 Gazebo LiDAR 作用域帧与 URDF `laser` 帧不一致的问题，SLAM 能持续消费真实 `/scan`。
- 实际生成并保存 194×64、0.05 m/px 的 SLAM 地图。
- 新增 `sanitation_safety` 高优先级速度门：Nav2 统一输出到 `/cmd_vel_nav`，仅速度门可向车辆发布 `/cmd_vel`。
- 实际执行 10 点 `NavigateThroughPoses`，action 状态为 `SUCCEEDED`，并记录 node/topic/action/service、TF、AMCL、里程计与 rosbag。
- 隔离验证急停：正常指令放行、急停归零、释放后恢复、上游失联 0.5 秒后归零全部通过。
- 构建与新增测试通过；导航包 lint、XML 和 3 个速度门单元测试均通过。

证据与边界：

- `artifacts/stage3_20260714_172155/stage3_summary.json`
- `artifacts/stage3_20260714_172155/navigation_probe.json`
- `artifacts/stage3_20260714_172155/safety_probe.json`
- `artifacts/stage3_20260714_172155/slam_map.yaml`
- `artifacts/stage3_20260714_172155/navigation_bag/metadata.yaml`
- action 虽成功，但终点 AMCL 与里程计平面距离相差 1.806 m，且 controller 日志出现 2 次 progress failure；该结果只能证明导航闭环可运行，不能证明定位精度达标。

复现命令：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage3_docker.ps1
```

## Stage 4：覆盖规划、指标与受控执行交接

状态：评审门已通过；项目按主提示词停在 Stage 4，不进入感知训练或 J6 量化。

已完成：

- 新增 `sanitation_coverage`，集成 OpenNav Coverage 与 Fields2Cover，使用 Boustrophedon 路由、Dubins 转弯和 0.65 m 作业宽度。
- 对 16 m × 8 m、128 m² 示例区域生成 12 条作业带、11 个转弯和 2140 个稠密 Nav2 路径点；总路径长度 213.494 m。
- 以 0.10 m 栅格审计计划覆盖：覆盖 124.80 m²，覆盖率 97.5%，漏扫率 2.5%，重复率 2.492%。这些是规划几何指标，不是实车经验覆盖率。
- 发现并兼容 OpenNav `PathComponents` 中退化的 swath end point；兼容层只用相邻 turn 首点及最终路径点重建端点，原始与修复后数据均写入证据。
- 根据 AMCL 当前位姿选择完整覆盖路径的最近点，从 2140 点计划中截取 180 点执行窗交给 Nav2；action 被接受并持续执行，20 秒内里程计位移 7.393 m，随后主动取消。
- 清扫刷在执行窗内开启、退出时关闭；完整路径的作业带/转弯刷控计划记录为 12 个开启段和 11 个关闭段。
- 记录 coverage server、Nav2、Gazebo、node/topic/action/service、rosbag、完整路径 JSON 与指标 JSON；Stage 4 新增测试 3/3 通过，累计 293 tests、0 errors、0 failures、44 skipped。

证据与边界：

- `artifacts/stage4_20260714_174914/stage4_summary.json`
- `artifacts/stage4_20260714_174914/coverage_metrics.json`
- `artifacts/stage4_20260714_174914/coverage_path.json`
- `artifacts/stage4_20260714_174914/coverage_bag/metadata.yaml`
- 受 Stage 3 终点定位差 1.806 m 影响，只执行与取消局部路径窗以验证接口和物理运动；97.5% 覆盖率不能解释为完整覆盖任务已经实跑完成。
- 历史执行时当前主机没有 Ubuntu 24.04/WSLg 图形环境，因此没有伪造 Gazebo/RViz GUI 截图；该缺口已由 2026-07-29 本机 WSLg 实机复核补齐，原轮次的 headless Ogre2、ROS 图谱、JSON 与 rosbag 证据保持不变。

复现命令：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage4_docker.ps1
```

评审边界：优先修正定位一致性并完整回放覆盖任务；是否进入感知与 J6 阶段由人工评审后另行决定。
# AUTO-12 自主推进（2026-07-30）

新增 opt-in `auto12_efficiency_v1`，同步 `1.32 m` 展开刷组的物理/清扫/
碰撞/成本地图/Coverage/动力学/能耗消费者，保留 `0.65 m` 生产默认。
10 次时间步进与栅格正式任务的平均有效效率 `4205.81 m²/h`、95% CI
下界 `4193.52 m²/h`、单次最低 `4181.12 m²/h`；覆盖、漏扫、重复、
定位、安全和终态门全部通过。`AUTO-12=PASS`、
`competition_efficiency_pass=true`。证据等级仅为
`OFFLINE_TIME_STEP_DYNAMICS_AND_RASTER_SIMULATION`，尚未形成 Gazebo
或真实车辆效率证据。
# AUTO-09 自主推进（2026-07-30）

新增 opt-in arm/hand URDF、ros2_control、MoveIt2、抓取候选、感知坐标变换、
规划场景、40 L bin 和安全恢复。瓶/罐/纸分别完成 20 次 micro 与 30 次
正式离线运动学闭环，逐类抓取/运输/入箱成功率均为 `1.0`，90/90 不可达
目标 fail-closed，错误目标、掉落、碰撞和关节越界均为 0。
`AUTO-09=PASS`。证据等级为
`OFFLINE_KINEMATIC_PERCEPTION_LOOP_SIMULATION`，尚未形成 Gazebo 动态
抓取或实体机械臂证据。
# Coverage path optimization（2026-08-03）

- 新增版本化 `CoveragePlan` 与 8 类语义组件，规划、执行和 Gazebo 面板使用同一组件身份与刷盘语义。
- 新增 5 度步进方向搜索、经验间距选择、相邻往复式清扫带路由、履带底盘 RTR 转接、阻塞条带有界重试和连通残余区域补扫。
- 独立小场默认启用 0.48 m / DISCONTINUOUS 优化配置；保留 0.35 m / Dubins 连续旧配置作为显式回退。
- 遥测升级为 v2 并分离规划清扫带、转接、补扫、当前组件及三类实际轨迹；保留旧字段和 `/coverage/current_path`。
- Windows 快速门禁 202 项通过；ROS 选定包构建通过，coverage 与 Gazebo visualization 共 34 项 ROS 测试通过。真实多种子 Gazebo 结果以本任务验收报告为准，不用静态测试替代。

# 正式竞赛整车 URDF/CAD V2（2026-08-25，历史基线）

- 新增 A300、UR5e、2F-85、八路传感器、UR 控制箱、S100 参考箱、完整清扫/刮吸机构和独立干湿箱的正式多刚体 Xacro。
- V2 将原 primitive 外观基线升级为开源官方网格与项目参数化 CAD：A300/UR5e/2F-85/主要传感器采用锁定许可 mesh，清扫/回收/分仓生成 47 个机构 STL，鱼眼、支架、控制柜和安装件由项目生成；共 100 个 mesh visual，51 个关键外露 link 全部通过非 primitive 门。
- 展开模型为 105 links / 104 joints / 空载 135.283866 kg；`base_footprint` 与 UR 官方数学基准 `ur5e_base_link` 为无质量虚拟帧，其余 103 个物理 link 均有正定惯量；22 个非 pending 安装帧均通过 5 mm / 0.02 rad 一致性门。
- 新增 Gazebo `DynamicPayloadSystem`，实跑确认干垃圾和污水动态质量分别强制限于 1.512 kg 与 9.7064 kg，并同步更新惯量和污水重心。
- 新增 OpenSCAD 包装源、可重复 CAD 生成器、104 资产 SHA-256 清单、mesh-aware 工程预览、布局/URDF/视觉/运行时分层报告及 20 项确定性测试。
- Headless Gazebo Harmonic 已观察到全部雷达、相机、GNSS 和 IMU 消息；100 个 mesh visual 无资源加载失败，六个控制器分组启动后全部 active；底盘累计 4 s 的 0.25 m/s 指令产生 0.537127 m 地面真值位移，刷盘、机械臂、夹爪完整 mimic 链和储运接口均有运行响应。
- 同一提交候选的 Windows 快速门为 332/332；Stage 1 两轮均为 629 tests、0 errors、0 failures（51 项按上游 cppcheck 策略跳过）；Stage 2 回归收到 12/12 类必需话题，5 s 地面位移 1.200000 m。
- 保持 pending：S100 实物测量、最终 CoG/污水容量、机械臂全空间、清扫接触/柔性和精确遮挡；Gazebo 服务端仍需针对任务 PID 强制结束，不得据此宣称实车或综合竞赛通过。
