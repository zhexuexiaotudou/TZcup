# TZcup 许可证清单

本仓库不是单一许可证覆盖的代码集合，具体声明以各 ROS 2 包的
`package.xml` 为准：

- `starter_ws/src/sanitation_hmi`：MIT；
- 其余 `starter_ws/src/sanitation_*` 包：Apache-2.0；
- 自动生成的 Gazebo 世界、配置与本项目训练得到的模型随其所属 ROS 2 包采用相同许可证；
- 第三方依赖不复制进仓库，其许可证由上游项目保留。

SPDX 全量清单见 [`reports/release/SBOM.spdx.json`](reports/release/SBOM.spdx.json)，模型与资产来源见
[`MODEL_AND_ASSET_LICENSES.md`](MODEL_AND_ASSET_LICENSES.md)。Apache-2.0
与 MIT 的标准条款可从 [SPDX License List](https://spdx.org/licenses/)
获取。本文件是工程清单，不替代各上游许可证正文。
