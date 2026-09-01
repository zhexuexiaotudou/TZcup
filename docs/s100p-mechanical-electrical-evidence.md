# S100P 安装与供电证据合同

[`s100p_mechanical_electrical_evidence.json`](../config/high_fidelity_vehicle/s100p_mechanical_electrical_evidence.json)
把 RDK S100P 安装/供电的已知官方资料和未关闭的实物门分开记录。它不是 URDF 更新、
机械设计、线束图、上电许可或验收报告；关联校验器只读取 JSON 和既有只读身份 artifact：

```powershell
py -3 .\scripts\validate_s100p_mechanical_electrical_evidence.py
py -3 -m pytest .\scripts\test_s100p_mechanical_electrical_evidence.py -q
```

官方一手资料为 [RDK S100 hardware introduction](https://d-robotics.github.io/rdk_doc/rdk_s/Quick_start/hardware_introduction/rdk_s100/)
和 [official archive directory](https://archive.d-robotics.cc/downloads/hardware/rdk_s100/rdk_s100/)。合同记录公开的
KS1P75Y / S100P、2.0 GHz、24 GB、128 TOPS、带亚克力外壳的 120×121×51 mm 标称尺寸、
12–20 VDC 输入、J1 20 V/10 A 额定、典型 12 V@5.5 A/70 W、最大 20 V@7.5 A/150 W、
0–45 °C 和连接器型号表存在性。

这些事实不能冻结当前 URDF 的板卡外形或安装：120×121×51 mm 是带亚克力外壳的开发板
外形，不是已实测的裸板边界、孔位或线束 keepout。所有证据项都有 URL/source ID、
evidence level 和 `can_freeze_urdf: false`。本地
`g0_read_only_inventory.json` 只证明已连接设备身份为 `D-Robotics RDK S100P V1P0`，不提供
机械尺寸、重量、针脚、极性或功耗测量。

目录中虽然可见约 69 MB 的官方 STEP，但其产品简介声明 Confidential、Proprietary、All rights
reserved。项目的仅开源约束因此禁止下载、提交、派生、导入或宣称该资产可复用；合同只记录
目录元数据和禁止动作，未获取任何二进制或 CAD 文件。

以下硬门保持 `BLOCKED`：实物尺寸/边界、孔位 datum、质量/重心、连接器坐标和 keepout、
散热器/风扇/风道包络、J1 针脚和极性、真实线束/保险/DC-DC/接地/浪涌、热降额与温度实测，
以及已装车上电运行验证。因此 `urdf_update_authorized`、机械安装、电气安装和运行时接受均固定为
`false`。

该校验器已纳入 `scripts/ci_fast.py` 的纯 Python 门禁。静态功能链审计也会调用同一验证器，并把
其 `BLOCKED_MECHANICAL_ELECTRICAL_INTEGRATION` 结论、未决门和四个 `false` acceptance 字段写入
`s100p_mechanical_electrical_evidence`。这只证明证据记录完整且仍然 fail-closed，绝不把孔位、
质量、热/连接器坐标或实际上电标为 ready。
