# 第六批：逐件清扫与回收机构原生 B-rep 设计输入

`native_brep_cleaning_mechanisms_sixth_batch_contract.json` 为 reconstruction manifest 的 23 个 `cleaning` 项各提供一个唯一 `source_mesh` 条目；源码把每一件作为命名 Assembly 成员保留，覆盖刷盘/刷毛、滚刷与护罩、升降架/导柱/连杆、刮吸组件、泵隔振座、软管、过滤器和快接头。它不是将清扫机构替换为一个融合盒体。

该批仅为静态 CadQuery 参数设计输入，且源码不会导入 STL、加载 CadQuery 或创建 FCStd/STEP。未来生成前仍须逐件关闭材料、防腐、孔/螺纹/公差、刷毛保持与平衡、导向/销轴/行程、泵曲线和隔振动态、软管弯曲半径、流道/密封/耐压/化学兼容性、过滤介质和可维护性等输入，并留下原生导出及装配 receipt。

`scripts/audit_native_brep_source_coverage.py` 已正式注册第六批；
`scripts/test_native_brep_cleaning_mechanisms_sixth_batch.py` 验证其哈希绑定、23 个 builder
和 23/23 cleaning 精确映射。即使静态映射完整，`native_cad_delivery_accepted` 仍为
`false`，直至有真实 CadQuery 原生构建、STEP/FCStd 和装配/导出凭据。
