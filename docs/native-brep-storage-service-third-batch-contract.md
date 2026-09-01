# 原生 B-rep 第三批储箱与服务机构参数合同

[`native_brep_storage_service_third_batch_contract.json`](../config/high_fidelity_vehicle/native_brep_storage_service_third_batch_contract.json)
把尚未由第二批完整覆盖的储箱及服务机构整理为六个可追溯、可编辑的
CadQuery 设计输入工作包：40 L 干箱壳体/盖/加强筋、湿箱盖/通气/入口、干箱
锁扣及三件式 over-centre toggle latch、液位传感器/探针安装包络、排污阀服务链
和充电口接口。

所有尺寸都取自现有项目生成器、Xacro 安装变换、车辆 layout 与 BOM；第二批的
`wastewater_tank_pan_baffles` 被列为湿箱盖工作包的依赖而不是重复建模。源网格
从不被读取、导入或转换，因而不从三角面反推孔、螺纹、密封、材料或公差。

| 工作包 | 当前参数化 B-rep 源覆盖 | 必须保持 pending 的受控输入 |
| --- | --- | --- |
| `dry_bin_shell_lid_ribs` | 508×383 mm floor、四块壳板、四根加强筋、四片盖板和三根盖加强筋；40 L usable-volume 安装语境 | 壁体工艺/材料、铰链、密封、孔/紧固件、载荷 |
| `wastewater_lid_vent_inlet` | 358×266 mm 湿箱盖、凸台/加强筋、vent 和 inlet 外包络；依赖既有 pan/baffle | 罐体连接、滤芯、流道/配对件、螺纹/密封、耐化学/压力 |
| `dry_bin_latch_and_toggle_triplet` | 干箱锁扣的 handle，以及 base/handle/keeper 三件 latch 外形与安装 datum | 受力/啮合、销轴衬套、孔/紧固件、密封载荷、防腐 |
| `level_sensor_and_probe_mounts` | 干箱 sensor 外壳与低/高液位 probe 外包络 | 供应商接口、线束/防护、穿罐孔/螺纹/密封、标定 |
| `wastewater_drain_service_train` | pipe、阀体、可见 stem/indicator、actuator mount、cap 和 coupling 外形 | 内部球/流道/阀座、O-ring/螺纹、压力温度化学等级、锁定 |
| `charge_port_interface` | housing、receptacle 外包络、door 与 lock-pin 外包络 | 充电标准/触点、电气额定、铰链锁定、IP 密封、线束 |

权威的可编辑源码为
[`native_brep_storage_service_third_batch.py`](../starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_storage_service_third_batch.py)。
它仅在明确获准的路径中惰性导入 CadQuery；当前合同状态会在导入前拒绝任何
导出。每个可能的 FCStd 路径仅为可选的未来路径，所有 FCStd/STEP 均必须保持
不存在，直到供应商接口、孔/螺纹、密封、材料、公差及压力/电气条件受控放行。

源码和合同由
[`native_brep_storage_service_third_batch_source_manifest.json`](../starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_storage_service_third_batch_source_manifest.json)
以 SHA-256 绑定。它们不改变 native-CAD readiness 审计、CI、preflight 或集中
验收结论，也不安装 CadQuery、不生成 FCStd/STEP。

仅运行低内存静态检查：

```powershell
py -3 -m py_compile starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_storage_service_third_batch.py
py -3 scripts/test_native_brep_storage_service_third_batch_sources.py
```

这些命令不启动 WSL、Gazebo、Docker、FreeCAD 或数据采集。
