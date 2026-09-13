from pathlib import Path
import json, shutil, csv, datetime

ROOT = Path(__file__).resolve().parent
P = ROOT / 'submission-package'
E = Path(r'F:\Project\TZcup\.workspace\evidence')
STAMP = datetime.datetime.now(datetime.timezone.utc).isoformat()
for name in ['docs','evidence/board','evidence/reference','metrics','dependencies','video','tools']:
    (P/name).mkdir(parents=True, exist_ok=True)
index=[]
def copy(src, dst):
    src=Path(src); target=P/dst; target.parent.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(src,target)
    index.append({'file':dst,'source':str(src),'copied_utc':STAMP})
board=E/'competition-sim-only-20260912-final-board/evidence'
for folder in ['mapping-replay-01','localization-replay-01','planning-replay-01','planning-replay-02','qlearning-replay-01','control-replay-01']:
    for f in (board/folder).iterdir():
        if f.is_file() and f.suffix in ['.json','.log','.rc','.txt']:
            copy(f,'evidence/board/'+folder+'/'+f.name)
for name in ['board-module-summary.json','full-board-inventory.json']:
    copy(board/name,'evidence/board/'+name)
copy(E/'s100p-r14d-product-liveness-template-20260908T114500Z-28bbe2e/board_results/witness.json','evidence/board/development-liveness.json')
copy(E/'competition-sim-only-20260912-final-board/evidence/perception-replay-01/graph.log','evidence/board/perception-rejected.log')
copy(E/'mobility-turn-e999e53-02-closed/RESULT.md','evidence/reference/turn-diagnostic.md')
copy(E/'competition-sim-only-20260912-final-assessment/ASSESSMENT.md','evidence/reference/historical-assessment.md')
copy(Path(r'F:\Project\TZcup\.workspace\plans\simulation-only-completion-20260912\EXECUTION_STATUS.md'),'evidence/reference/execution-status-snapshot.md')
copy(Path(r'F:\Project\TZcup\_contest_brief.txt'),'evidence/reference/contest-brief.txt')
copy(E/'competition-sim-only-t7-turn-handoff-20260912T102409Z/final-package/sources/docs/formal-final-acceptance-orchestration.md','dependencies/historical-runtime-commands.md')
copy(E/'competition-sim-only-t7-turn-handoff-20260912T102409Z/final-package/inputs/model_and_asset_licenses.md','dependencies/model-and-asset-licenses.md')
(P/'evidence/index.json').write_text(json.dumps(index,ensure_ascii=False,indent=2),encoding='utf-8')

items=[
('mapping','建图','NOT_MEASURED','完整首图尚未封存；历史45分钟运行未完成。','保存地图、分辨率、已知栅格数和建图日志；重新加载后输出定位。'),
('following','循迹','NOT_MEASURED','短程左右转不等于自主路线完成。','一次任务的路线点、实际轨迹、终态和操作视频；记录中止事件。'),
('perception','垃圾识别与定位','NOT_MEASURED','板端开发HBM仅有消息活性；当前完整仿真识别定位未验收。','同帧图像、检测类别框、深度/标定/TF、预测地图坐标及独立标签对照。'),
('cleaning','一种清扫方式','NOT_MEASURED','采用边刷与中央滚刷；历史规划概率清除不证明本次动力学清扫。','同次任务GroundDirt前后状态、刷启停和清除事件；一段前后截图。'),
('avoidance','避障','NOT_MEASURED','历史分模块避障不能外推本次整链。','任务中插入一次动态障碍；记录无碰撞、绕行或等待及恢复任务。'),
('estop_function','应急制动功能','NOT_MEASURED','历史急停触发前已停稳；控制回放归零不是运动中车辆急停。','行驶中触发急停，保留触发事件和实际线/角速度，不只记录速度命令。'),
('localization_mm','定位≤50mm','FAIL','历史短程656/656配对，15ms最大跨度；EKF最大误差259.544mm。仅历史局部失败，当前全程未测。','同坐标系、同仿真时间配对估计与真值，导出RMSE/P95/max；预先采用max≤50mm的保守口径。'),
('mapped_area_m2','建图≥20000m²','NOT_MEASURED','200m×100m场景边界不是已建图面积。','有效已知栅格数×resolution²；未知区排除，保留地图文件与面积口径。'),
('cleaning_m2_h','效率≥3500m²/h','NOT_MEASURED','4205.81为历史离线估计；1.32×1×3600×0.75=3564亦是理论估计。','实际清除有效面积并集÷完整作业仿真秒数×3600；计入转弯、等待、避障，另报墙钟与RTF。'),
('width_mm','清扫宽度≥600mm','NOT_MEASURED','0.65m模型刷宽、1.32m展开设计值不作为有效清扫实测。','直线短段运行，取实际污物清除带横向宽度；保存刷状态、轨迹与前后图。'),
('braking_s','制动≤1s','NOT_MEASURED','历史174个静止GT样本未覆盖行驶到停车过程。','同一时钟下从急停触发至实际|v|≤0.01m/s且|ω|≤0.01rad/s，持续0.2s；报告采样频率与上界。')]
rows=[]
for key,label,status,note,method in items:
    ref='evidence/reference/turn-diagnostic.md' if key in ['localization_mm','braking_s','estop_function'] else 'evidence/reference/execution-status-snapshot.md'
    rows.append(dict(id=key,item=label,status=status,value=259.544 if key=='localization_mm' else None,basis='historical_limited_measurement' if key=='localization_mm' else 'not_measured',evidence=[ref],note=note,minimum_measurement=method,as_of_utc=STAMP))
(P/'metrics/results.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')

sections=[]
def section(title,*paragraphs): sections.append((title,list(paragraphs)))
section('01 项目背景与交付边界','面向智慧环卫场景的国产系统无人清扫车关键技术攻关；赛题DG-202604。本报告针对一天内的最小交付，采用独立软件仿真演示与本地S100P算法执行证据两条路线。当前材料具备可审阅正文，已有真实GUI partial clip但完整5-10分钟任务Demo仍缺，不能宣称全部达标。','需求来自包内 contest-brief.txt 第2–4页与第6页。六项功能与五项数字是本次收口工作范围，不能替代题面全部要求。报告还披露识别准确率≥95%、尘箱≥40L、两种交互、任务分解等未测条目。','证据截至 '+STAMP+'。本包由旧main建立的独立工作树生成，未构建或运行机器人；本工作树源码不是最新仿真运行源码。历史板端版本6731784；最新状态页候选1a211400，仅本地候选就绪。')
section('02 系统架构与PC/板端职责','独立仿真链：Gazebo场景/车辆/传感器 → 感知与SLAM/EKF → Nav2/覆盖规划/任务决策 → 速度安全门/刷控制 → Gazebo车辆和污物状态。真值进入独立评分，不进入感知或控制算法。','板端证明链：合成或仿真回放输入 → S100P实际运行算法 → 地图/定位/路径/控制输出 → 日志与报告。PC负责输入和独立评分，不得提前计算结果再由板端转发。两条链互不构成实时硬件闭环。','PC基线：Ubuntu24.04、ROS2 Jazzy、Gazebo Harmonic；板端历史环境：aarch64、S100P V1P0、TROS/ROS Humble，存在/dev/bpu_core0。不能将Jazzy工作空间直接当作Humble二进制使用。')
for key,label,status,note,method in items:
    section(f'{len(sections)+1:02d} {label}', '当前状态：'+status+'。'+note, '实现与最短测量：'+method, '判定边界：仅接受有对应运行记录的数据。未执行写NOT_MEASURED；观测失败写FAIL。单次演示不证明长期可靠性或跨场景成功率。仿真任务完成后只更新metrics/results.json，通过接入工具重建唯一矩阵与报告；不得修改历史原始报告。', '证据：'+next(r['evidence'][0] for r in rows if r['id']==key))
section('14 S100P连接与感知开发证明','环境证据：evidence/board/full-board-inventory.json；设备S100P V1P0，架构aarch64，存在BPU设备节点；release版本6731784。这里只证明历史环境与部署，不是当前连线探测。','感知证据：development-liveness.json来自R14d；NON_FORMAL_ABI_DEVELOPMENT，semantic_status=NOT_EVALUATED。DOSOD/EdgeSAM开发链活性不得称准确率通过，CPU/BPU完整性能分解NOT_MEASURED。','后来perception-rejected.log记录产品适配器拒绝非nash-m开发目标。两份证据日期和模式不同，可同时成立；本次不补正式HBM、500图或长稳。板端感知语义正确率、每类P/R、端到端时延和功耗均NOT_MEASURED。')
section('15 S100P建图、定位与规划回放','mapping-replay-01/report.json证明slam_toolbox产生含已知和占用栅格的OccupancyGrid；输入为synthetic_replay。localization-replay-01/report.json有69个输出，位置推进、速度与角速度响应变化。该证据不提供20000m²建图或50mm定位证明。','规划采用planning-replay-02/report.json与qlearning-replay-01/report.json。full_coverage 3/3，sensing_greedy 2/3，oracle 1/3且evaluation_only。保留全部9次及失败；oracle不能当产品策略。','Q-learning的训练/验证/测试成功率均1.0，报告无split overlap、无真值访问；该结果不能覆盖规划报告失败。板端规划为有限回放执行证明，抓取验证模式为simulated_probability，不是Gazebo接触或实车抓取。')
section('16 S100P控制、性能与局限','control-replay-01/report.json：61个输出，正向限幅、急停归零、恢复后反向限幅、输出变化检查通过。控制输出保留在测试路径，没有连接实体执行器。','这些输出证明板端程序响应输入变化；不能证明车辆制动距离、行驶中响应≤1s或安全继电器动作。输入/输出时间偏移可用于检查消息序列，不应直接替代端到端实时性能。','板端联合负载、持续运行、峰值内存、功耗、温度、BPU利用率、P95端到端时延：截至本报告NOT_MEASURED。已有进程状态文件随证据保留，只能反映对应采样时刻，不能推广为峰值或稳定性结果。')
section('17 关键技术与创新边界','关键技术是将感知、覆盖规划、刷清扫和高优先级安全门通过ROS接口连接，并在独立仿真中对任务前后环境变化评分。与仅给出覆盖路径相比，方案希望用实际污物清除并集衡量净效率；当前尚待同次完整运行证明。','机械臂控制、人机交互和任务分解在历史技术报告中已有设计描述，但本次最小闭环优先一种清扫方式。题面原文含“抓取等核心功能”，以清扫替代抓取来自当前用户缩限，并非已取得主办方书面豁免。','本报告不主张算法全球首创或全栈国产软件已实现。Nav2、SLAM Toolbox等属于第三方软件；采用国产计算板不能推导操作系统及全部依赖国产化。模型与第三方许可见dependencies/model-and-asset-licenses.md，未核清资产不附二进制。')
section('18 依赖、启动与复现说明','仿真依赖基线为ROS2 Jazzy/Gazebo Harmonic及Nav2、SLAM Toolbox、robot_localization、OpenNav Coverage/Fields2Cover和项目包。历史正式启动参数完整保留在dependencies/historical-runtime-commands.md；它是参考，不是当前候选验收命令。','基础入口命令：source /opt/ros/jazzy/setup.bash；bash scripts/run_baseline.sh。仅适用于README_FIRST骨架环境，不能用于宣称当前比赛任务成功。当前运行者应交付实际成功命令、工作目录、source commit、overlay路径和所用配置；本任务不执行这些命令。','板端已验证历史命令：/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node --ros-args --params-file /opt/tzcup/s100p/releases/competition-sim-only-6731784-20260912/source/slam_replay.yaml。其余模块实际cmdline随证据保存。重跑需原release和输入；本提交包不假装包含可执行完整源码与模型。')
section('19 失败、限制与其他题面要求','保留首图45分钟未封图、progress_timeout恢复、历史转向定位偏差与当前生命周期/录包问题。execution-status-snapshot.md是历史状态的快照，不表示后台后续工作停止。当前完整任务尚无成功分母，不能合并不同版本的组件PASS。','额外题面要求：识别准确率≥95%、尘箱承载≥40L、两种交互及指令准确率≥95%、任务分解正文≥95%而评分≥90%、新手3天、日均故障率<1%、三步更换、积水/落叶、边界防护。此次未新增测试，均披露NOT_MEASURED，不归类为自动取消的加分项。','当前提交风险：缺有效5-10分钟完整任务Demo，已有短GUI partial clip但任务未启动；报名表、申报人命名信息及当前仿真结果仍缺。技术报告可供审阅，但不得把整包称为已符合全部比赛最低要求。题面有关键技术指标不满足的一票否决说明。')
section('20 提交格式、证据接入与封包','项目保存题面要求：技术方案PDF 20–50页；软件仿真或实物二选一；演示视频5–10分钟并含操作说明；审核通过的参赛报名表；压缩包名为“申报人所在单位-申报人姓名-作品名称-联系电话”。未查到单个PDF或视频的强制文件名，当前采用项目自定描述名。','可立即提交的内容材料：本20页技术方案、唯一矩阵、板端回放证明与失败限制。这表示材料可取用，不表示整包可提交通过。等待：完整任务关键JSON/日志、地图/轨迹与测量数据、有效5–10分钟完整任务视频、报名表及最终身份信息；短GUI partial clip和代表截图不可替代。','不再做的加分或工程增强：选交源代码/数据集打包、用户调研/产业化方案、专利论文补充、500图与正式HBM补制、长稳、三次任务、GitHub/PR/部署/复杂取证。已有真实证据仍保留。取得结果后预计5–10分钟刷新矩阵与PDF、5–10分钟复核与压缩；不含补测、视频制作与报名表取得时间。')
assert len(sections)==20
(P/'docs/sections.json').write_text(json.dumps(sections,ensure_ascii=False,indent=2),encoding='utf-8')
(P/'README.md').write_text('''# TZcup 比赛最小材料包

当前整体未达到完整提交条件。技术报告、板端证据与结果接入工具已成稿；没有伪造视频或未测PASS。

- `docs/技术方案报告.pdf`：20页中文报告。
- `metrics/验收矩阵.md` / `results.json`：唯一仿真六功能、五指标状态源。
- `metrics/板端证明.md`：独立于仿真的板端证据。
- `提交清单.md`：必交格式、三张短表、缺件和命名。
- `evidence/index.json`：原件来源与复制时间；相对路径内证据可随包搬运。
- `接入说明.md`：补入结果后刷新报告。不要把当前旧main源码当最新运行代码提交。

所有文中引用历史报告仅作证据，历史报告原有PASS/工程门禁不覆盖本包唯一矩阵。未清理根工作区、云卡或任何任务证据。
''',encoding='utf-8')
print(P)
