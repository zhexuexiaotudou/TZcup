# 竞赛级整车 URDF 实施前准备包

> 状态：`READY_FOR_URDF_IMPLEMENTATION_WITH_LAYOUT_GATES`，2026-08-25。本文和配套配置只冻结正式 CAD/Xacro 的输入，不表示正式 URDF、网格、Gazebo 执行器或高保真闭环已经完成。

## 1. 已完成的准备

正式实现可以直接从 [`pre_urdf_contract.yaml`](../config/high_fidelity_vehicle/pre_urdf_contract.yaml) 开始，不再临时搜索或凭印象选择部件：

- 5 个 ROS 2/Jazzy 开源来源已锁到不可变 commit，覆盖 A300、Gazebo Harmonic、UR5e、2F-85、UTM-30LX、MID-360、D435、u-blox 和 VectorNav 描述；导入文件为 [`high_fidelity_vehicle.repos`](../repos/high_fidelity_vehicle.repos)。
- 两个侧后相机冻结为两套独立 `Arducam B0202 IMX291 UVC + M27195H15`，不会用单个双目设备冒充两个相机。
- 清扫执行器冻结为 3 个带64 CPR编码器的 `Pololu 4694` 24 V 减速电机、`Actuonix P16-100-256-12-P` 反馈推杆和 `Jabsco Q402J-118S-3A` 24 V 自吸泵。双侧刷、中央滚刷、浮动刮水胶条、吸口、箱体、管路和支架按这些真实部件自行建立开放几何。
- 24 个必需 frame、8 个清扫机构关节、8 个传感器话题/FOV/频率/量程以及 REP-103/105 坐标约定已冻结；安装坐标故意留到碰撞、机械臂包络和 FOV 联合布局时求解，避免在没有 CAD 的情况下编造外参。
- 干垃圾舱保持 `45 L`几何容积、`>=40 L`可用容积并与污水舱水密分隔；3 cm 方块继续按纸板、PP、PET、铝密度随机化。
- 功耗分为 A300 的 12 V/24 V 传感器支路和独立 VBAT DC 母线。UR5e、泵和刷盘电机不得挤占额定仅 120 W 的传感器支路。

仓库不复制厂商网页中的 CAD、图纸或图片。上游 BSD-3-Clause 模型按许可证使用；其他部件只引用公开参数，随后由项目根据尺寸和功能重新建模。这样既保持可追溯，也不把“公开可查看”误写成“允许再分发”。

## 2. 载荷与箱体预核算

校验器按 40 Ah A300 的 `101.5 kg`允许载荷和 10% 工程裕量计算：

| 项目 | 预核算值 | 口径 |
|---|---:|---|
| 设计载荷上限 | `91.350 kg` | `0.90 × 101.5` |
| 已选部件质量 | `37.705 kg` | 厂商值和已注明的保守上界 |
| 自研结构工程额度 | `40.000 kg` | 箱体、清扫件、支架、线束、转换器和S100等七项额度 |
| 固定载荷预算 | `77.705 kg` | 已选部件 + 工程额度 |
| 20个最坏铝块 | `1.512 kg` | `20 × 0.03³ × 2800` |
| 仅按载荷可给出的污水名义上限 | `12.133 L` | 尚未取重心/安装空间最小值 |
| 80%可用容积预值 | `9.706 L` | 保留20%液位/晃动余量 |
| 正常episode积水预上限 | `8.493 L` | 可用容积的87.5% |
| 此时剩余载荷裕量 | `2.427 kg` | 不冒充最终稳定性裕量 |

这证明 A300 仍有进入详细布局的载荷空间，但没有证明最终通过。正式污水容量必须在 URDF/CAD 阶段取以下四者的最小值：

```text
V_wet_nominal = min(V_by_mass, V_by_CoG, V_by_installation_space, 20 L)
V_wet_usable = 0.80 * V_wet_nominal
V_puddle_episode_max = 0.875 * V_wet_usable
```

若详细结构超过某项工程额度，校验器会直接压缩可用水量；若固定载荷加最坏垃圾已没有正的水量，则 A300 候选失败，不能靠把质量从 URDF 中删除来通过。

## 3. 正式 URDF 开始后的六个硬门

“准备完成”表示可以开始画车，不表示以下结果可以预先写成通过：

1. 读取用户实际 S100/Journey 6 板卡 SKU，并测量外壳、连接器、安装孔和质量；当前只给它保留 `2 kg`工程额度。
2. 完成全部部件质量/惯量和机械臂姿态、污水液位组合重心扫描。
3. 通过 UTM、MID-360、前向/腕部 RGB-D、两侧后鱼眼的遮挡、盲区和3 cm目标可见性图。
4. 写入污水舱重心上限和安装空间上限后重新冻结最终容量。
5. 对每个进入 Git 的网格逐项确认允许再分发；否则只提交项目自建几何。
6. 从所选 A300 手册冻结 VBAT 电压、DC/DC、保险丝、线径、连续/峰值电流和回滚方案。

这六项均已作为机器可读的 `layout_gates_during_urdf` 留在契约中。正式实现不得删除门禁后宣称完成。

## 4. 使用与验证

只验证准备包，不需要 ROS 或 Gazebo：

```powershell
py -3 scripts/validate_pre_urdf_readiness.py --expect-report reports/engineering/pre_urdf_readiness.json
py -3 -m pytest -q scripts/test_pre_urdf_readiness.py
```

校验会拒绝未锁 commit、未知许可证、缺失部件角色、重复 topic/frame、不足40 L的干舱、无污水载荷空间、传感器支路过载和缺失布局门。紧凑结果保存在 [`pre_urdf_readiness.json`](../reports/engineering/pre_urdf_readiness.json)。

## 5. 当前明确未做

- 未建立 A300 承载框、机械臂座、箱柜、刷盘、刮吸或泵的正式 Xacro/网格；
- 未下载或提交厂商 STEP/IGES，未编写 `ros2_control`/Gazebo 插件；
- 未冻结任何传感器安装外参，未运行碰撞、FOV、重心或稳定性扫描；
- 未启动新的 Gazebo 高保真车辆仿真，也未改变现有 AUTO 阶段证据状态。

因此本准备包是下一轮 URDF 设计的输入基线，不是完成证明。
