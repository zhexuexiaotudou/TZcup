# 正式整车外观资产决策

## 结论

正式车保留已有且许可证明确的开源功能组件，不拼接来源不清或尺寸不匹配的整车外壳：

- Clearpath A300 底盘 mesh；
- Universal Robots UR5e 六轴机械臂 mesh；
- Robotiq 2F-85 夹爪 mesh；
- Robotnik 传感器 mesh。

这些来源、锁定 commit、许可证和文件哈希以
`starter_ws/src/sanitation_vehicle_description/meshes/vendor/SOURCES.yaml` 与
`meshes/MANIFEST.sha256` 为权威。

未找到同时满足“许可证允许再分发、轴距/轮距匹配、可容纳 40 L 干箱和污水系统、
可安装 UR5e 与全部传感器”的开源完整环卫车外壳。Robotnik `robotnik_description`
仓库中的 RB-VOGUI（BSD-3-Clause，审查 commit
`4bc73425d090ead4591a7091e7ef7e7dc4fe862a`）可作为合法工业造型参考，但其约
1.040×0.650×0.235 m 底盘、轮位和接口与当前 A300/清扫机构不匹配，因此没有把它的
车体几何硬套到正式模型。

外壳改为项目自有的确定性参数化 CAD：连续曲面车舱、机械臂工作口、检修门、灯组、
保险杠、轮眉、刷盘护罩和功能接口均由
`generate_product_bodywork_meshes.py` 生成。这样保留开源真实部件的运动学与外观，
同时避免未知许可证、错误比例和无法维护的第三方整车拼接。

## 声明边界

该选择提高的是仿真产品完整度和可维护性，不是购置实物后的计量级逆向工程。
隐藏齿轮、线束、钣金厚度、注塑分型、密封和制造公差仍需真实硬件设计与测量。
