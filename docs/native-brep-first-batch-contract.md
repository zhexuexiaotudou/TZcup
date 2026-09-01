# 原生 B-rep 第一批承力与接口件参数合同

[`native_brep_first_batch_contract.json`](../config/high_fidelity_vehicle/native_brep_first_batch_contract.json)
固定首批四个项目自研承力/接口工作包的可追溯设计输入：
`arm_pedestal_adapter`、`sensor_tower`、`cleaning_head_brackets` 和
`storage_frame`。它与既有整车 B-rep 重建总清单并列，不修改任何 readiness
或集中验收结论。

合同将每件的生成器、布局坐标、Xacro 连接/惯量、可见网格、材料候选、BOM
参考质量和依赖关系写成机器可读字段。所有行的状态均固定为
`design_input_pending_native_export`：这些是原生特征建模之前的输入，不是已
完成的 FCStd/STEP、制造图、结构计算、材料放行或实物测量。

| 工作包 | 当前可用的精确输入 | 仍不能由网格/URDF 推断的输入 |
| --- | --- | --- |
| `arm_pedestal_adapter` | 280 × 220 × 12 mm 背板、180 × 180 × 40 mm pedestal、Ø240 mm 法兰外包络、192 mm 六点视觉基准圆、`arm_mount_link=[0.100,-0.200,0.4341] m` | A300 甲板与 UR5e OEM 孔图、孔径/螺纹/定位销、预紧、载荷/疲劳与 GD&T |
| `sensor_tower` | 190 × 150 × 16 mm 底座、双 30 × 30 × 760 mm 立柱、36 × 62 mm 服务脊、三横撑、四个底座位置基准、`sensor_mast_link=[0.420,0,0.3891] m` | 底座孔径（M8 只是草案候选）、传感器供应商孔图、挠度/振动、线束接地与公差 |
| `cleaning_head_brackets` | 340 mm 主轨、四个 `(+/-0.180,+/-0.250)` m 导轨中心、180 mm 导轨输入、100 mm 向下行程、升降/滑板安装基准 | 底盘孔图、导轨/轴承配合、P16 clevis/pin、磨损/防腐和负载/防护 |
| `storage_frame` | 570 × 620 × 12 mm 托盘、两纵梁/两横梁、六个托盘位置基准、干/湿箱安装变换 | 甲板、箱体孔图、满载/液体晃动、排水/开盖检修间隙和密封定义 |

四件均使用 `base_footprint` 的米/弧度坐标；零件本地 datum 与 URDF 连接
变换分开记录。源网格只可作为外部包络和尺寸输入，不能导入或转换成原生
B-rep。尤其要注意：生成器里的圆柱只表达**视觉紧固件包络或位置 datum**，
不是孔、螺纹、沉孔或 OEM 法兰规格。

计划中的 FCStd/STEP 路径特意保持不存在。静态验证器会在这些路径出现时失败，
防止占位文件被误当成交付。真实建模前至少要关闭受控接口、材料/紧固件、载荷
与结构、质量/惯量、DFM/GD&T 等输入；之后还必须完成原生特征模型审查和非镶嵌
STEP 预检。

在低内存 Windows 上可安全运行：

```powershell
py -3 scripts/validate_native_brep_first_batch_contract.py
py -3 scripts/test_native_brep_first_batch_contract.py
```

以上命令只读取 JSON 与源码文本；不会启动 WSL、Gazebo、Docker、FreeCAD、CAD
内核、网格转换器或 STEP 导出器。
