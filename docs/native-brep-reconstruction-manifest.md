# 原生 B-rep 重建清单

[`native_brep_reconstruction_manifest.json`](../config/high_fidelity_vehicle/native_brep_reconstruction_manifest.json) 是正式车辆项目自研几何的逐件原生 B-rep 重建计划；它不是 CAD 交付物、制造放行或 STEP 导出凭证。Schema 位于同目录，静态校验器为 `scripts/validate_native_brep_reconstruction_manifest.py`。

清单把当前三份 Python 网格生成器的 126 个 STL 输出全部计入：105 个项目自研件逐件登记了生成器、源网格、目标零件 FCStd、目标零件 STEP、目标装配 STEP、尺寸/材料/质量/BOM、安装接口和坐标基准；六个装配条目同时保留目标装配 FCStd。21 个第三方参考外形（电机、执行器、电池、继电器、接触器等）单独列为排除项，必须取得供应商原生 CAD 或受控接口数据，不能被重建成“项目自研件”。

所有重建行的状态固定为 `pending_native_brep_reconstruction`，目标路径也必须不存在。清单不生成 FCStd 或 STEP 占位文件，因此不会把计划误读为已完成的 B-rep 交付。后续实际建模必须：

- 从列出的 Python 参数、布局、部件台账和 BOM 重新建立可编辑的特征/草图/装配约束；不能导入或转换 STL。
- 按清单的部件本地坐标系建模，以 Xacro visual origin 和 `base_footprint` 安装帧装配；不能从三角形顶点倒推出安装偏移。
- 为每一项补齐受控材料、实物称重、接口尺寸、GD&T 和供应商/试验依据后，才可使用 Windows 原生 B-rep 工具导出非镶嵌 ISO-10303 STEP，并另行形成哈希绑定的导出回执。

在低内存 Windows 主机上可安全运行下列只读校验；它不会启动 WSL、Docker、Gazebo、FreeCAD 或任何 mesh converter：

```powershell
py -3 scripts/validate_native_brep_reconstruction_manifest.py
py -3 scripts/test_native_brep_reconstruction_manifest.py
```

该清单不修改 `mechanical_release_readiness.yaml`、native-CAD readiness 审计或现有 CAD/bootstrap 文件；这些门仍应在真实原生源文件和非网格 STEP 交付之后独立判定。
