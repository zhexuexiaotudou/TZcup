# TZcup 非技术得分补齐包

本包用于补强 DG-202604 官方评分表中不依赖 Gazebo、S100P 或实车运行的评分证据。它不覆盖或修改 23 页正式报告，也不重复 `codex/day1-value-package` 中已经完成的培训计划、三步维护设计、尘箱设计和客户试点材料。

## 基线

| 基线 | 提交 | 本包如何使用 |
|---|---|---|
| 23 页正式报告与证据索引 | `codex/day1-submission-report` / `43f7136` | 作为技术事实与运行证据的唯一入口 |
| 用户体验、维护、产业化设计包 | `codex/day1-value-package` / `2aefb12` | 作为培训、SOP、BOM/试点和 IP 清单来源 |
| 官方赛题 PDF | SHA-256 `5FA80F74B7A1F36E5D0D63592282A5CC5F26394E562350407497CFF27BDCCC9E` | 作为分值与文字要求来源 |

## 本包解决的问题

已有材料主要是说明性文档。本包把最容易生成材料分的项目进一步做成评审可操作的四件套：

1. 冻结的通过判据与证据边界；
2. 机器可读的数据契约；
3. 可重复运行的校验工具和测试；
4. 能装入报告或附件的状态表、分镜、模板和 PDF。

## 评分状态

| 官方项 | 分值 | 当前可主张 | 当前状态 |
|---|---:|---|---|
| 3 天掌握操作 | 2 | 三日课程、验收台账和认证算法已定义 | `PLANNED_NOT_MEASURED` |
| 易损件更换 <=3 步 | 5 | 三步 SOP、数字程序模型和验收协议已冻结 | `DESIGN_ONLY_NOT_PHYSICALLY_MEASURED` |
| 日均故障率 <1% | 3 | 事件字典、分母和 95% 置信资格算法已定义 | `INSTRUMENTED_NOT_MEASURED` |
| 社会效益 | 5 | 可审计的社会效益参数模型已定义 | `MODEL_ONLY_NOT_FIELD_MEASURED` |
| 硬件/软件功能完整 | 6 | 模块-证据-缺口矩阵已建立 | `PARTIAL` |
| 演示视频 | 4 | 300 秒模块化综合演示、中文旁白、字幕、章节、操作说明和媒体校验器已交付 | `DELIVERED_MODULAR_DEMO` |
| 文档质量 | 5 | 正式报告、证据索引和独立补充 PDF 可交叉复核 | `DELIVERED` |
| 可落地产业化方案 | 5 | 客户、试点、成本公式、Go/No-Go 和 IP 边界已整理成评审包 | `REVIEW_READY_NOT_FIELD_VALIDATED` |
| 已申请专利或发表论文 | 5 | 只提供发明披露和论文提纲模板 | `NOT_ELIGIBLE_NO_FILING_OR_PUBLICATION` |

上述状态是材料成熟度，不是官方得分。`PLANNED`、`DESIGN_ONLY`、`MODEL_ONLY` 和 `KIT_ONLY` 都不能改写成实测或已经通过。

## 快速验收

在本目录运行：

```powershell
py -3 -m unittest discover -s tests -v
py -3 tools\verify_package.py --write
py -3 tools\reliability_qualification.py --plan --write
py -3 tools\validate_maintenance_procedure.py
py -3 tools\social_benefit_model.py --scenario data\social-benefit-scenarios.json
py -3 tools\build_supplement_pdf.py
```

正式 5 分钟成片及视频包位于提交包的 `video/` 目录。可在提交包根目录复核：

```powershell
py -3 video\tools\validate_video_package.py --write
```

视频项建议按 `2-3 / 4` 分准备评审说明，评委接受模块化综合演示和章节化操作说明时最高可裁量 `4/4`；它不表示所有镜头来自同一次连续任务。历史 6.333 秒局部片段仍保留在 `evidence/day1/demo/`，只作为早期录屏链路证据。

## 目录

| 路径 | 用途 |
|---|---|
| `official-score-closure.md` | 官方分项、优先级、可主张/不可主张边界 |
| `operator-usability.md` | 三日认证的证据接口与留痕方式 |
| `maintenance-three-step.md` | 三步维护的冻结程序和物理验收协议 |
| `reliability-case.md` | 故障事件字典和统计资格计划 |
| `social-benefit.md` | 社会效益参数模型及来源边界 |
| `system-completeness.md` | 硬件/软件模块完整度矩阵 |
| `demo-video-production.md` | 5-10 分钟演示脚本、分镜和验收 |
| `industrialization-readiness.md` | 产业化评审摘要 |
| `ip-research-boundary.md` | 专利/论文材料边界和模板 |
| `data/` | 机器可读状态、程序、分镜、清单和场景 |
| `tools/` | 校验、计算和 PDF 生成工具 |
| `tests/` | 纯离线回归测试 |
| `pdf/` | 独立补充 PDF |
| `evidence/` | 自动生成的验证回执和局部视频失败回执 |

## 真实性规则

- 不把用户研究计划写成用户研究结果。
- 不把数字维护程序写成实物三步拆卸结果。
- 不把故障率计算工具写成长期运行结果。
- 不把社会效益参数算例写成园区实测效益。
- 不把已交付的模块化综合演示写成同一连续任务；旧的 6.333 秒 GUI 片段只作历史局部证据。
- 不把产业化路线写成订单、报价、ROI 或量产验证。
- 不把披露模板写成已申请专利或已发表论文。
