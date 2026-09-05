# 第四批车身、传感器与电源安装件 CadQuery 原生参数化源码

[`native_brep_body_sensor_power_fourth_batch.py`](../starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_body_sensor_power_fourth_batch.py) 是第四批项目自研 B-rep 的可编辑源码；[`native_brep_body_sensor_power_fourth_batch_contract.json`](../config/high_fidelity_vehicle/native_brep_body_sensor_power_fourth_batch_contract.json) 记录每一个米制参数及其来源。它涵盖：车身前鼻、分体下舱、顶/侧板和挡泥罩包络，四扇检修门与铰链/锁扣包络，传感器桅杆以及 UTM、MID360、前 RGBD、双后鱼眼、GNSS、IMU 的项目安装件，S100P 防护外壳/柜顶安装板、UR5e 控制柜隔振底座，以及 PDU、DC-DC、安全继电器的项目支撑/外壳包络。

外购或用户自有的传感器、计算板、继电器和转换器本体没有被重建；源码只表达其周边的项目结构。所有孔型、螺纹、材料、成形、公差、门缝、密封、热设计、EMC、接地、爬电/电气间隙、供应商接口和载荷验证均保持 `pending`，不得从视觉网格、碰撞体或紧固件外观推断。

源码的 CadQuery 导入是惰性的。当前合同状态为 `design_input_pending_native_export`，所以任何 `--export` 请求均在导入 CadQuery、创建输出目录或写入文件前失败；本批不安装 CadQuery，也不生成 FCStd、STEP 或 STP。源码不读取、导入、反推或转换任何网格。

[`native_brep_body_sensor_power_fourth_batch_source_manifest.json`](../starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_body_sensor_power_fourth_batch_source_manifest.json) 以 SHA-256 同时绑定源码和合同。哈希匹配只证明静态输入一致，不是制造或导出授权。

低内存静态检查只读取 Python/JSON 和哈希：

```powershell
py -3 scripts/test_native_brep_body_sensor_power_fourth_batch_sources.py
py -3 starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_body_sensor_power_fourth_batch.py --summary
```

将来的原生导出必须先通过单独审查：受控项目接口/孔图，S100P 与供应商硬件实测接口，车身门缝与密封定义，结构/振动/疲劳，热、EMC、接地与电气安全评审，以及非 tessellated STEP 预检。届时应新增经评审的 release-evidence schema 与实现，而不是解除当前 fail-closed 门禁。
