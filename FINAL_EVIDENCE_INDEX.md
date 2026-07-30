# FINAL_EVIDENCE_INDEX

本索引只列出状态机中已登记的证据，不把低等级证据外推为高等级结论。

| Stage | 状态 | 证据目录 | 首个阻断层 |
|---|---|---|---|
| AUTO-00 | PASS | `artifacts/autonomous_auto00_20260728T161119Z_evidence` | `—` |
| AUTO-01 | PASS | `artifacts/autonomous_auto01_20260729_evidence` | `—` |
| AUTO-02 | PASS | `artifacts/autonomous_auto02_20260729_evidence` | `—` |
| AUTO-03 | PASS | `artifacts/autonomous_auto03_20260729_evidence` | `—` |
| AUTO-04 | PASS | `artifacts/autonomous_auto04_20260730_evidence` | `—` |
| AUTO-05 | BLOCKED | `artifacts/autonomous_auto05_20260730_evidence` | `G3_split_model_screening_gates_failed_after_3_attempts` |
| AUTO-06 | BLOCKED | `无（未执行/依赖阻断）` | `dependency_AUTO-05_blocked` |
| AUTO-07 | BLOCKED | `无（未执行/依赖阻断）` | `dependency_AUTO-05_blocked` |
| AUTO-08 | BLOCKED | `无（未执行/依赖阻断）` | `dependency_AUTO-05_blocked` |
| AUTO-09 | PASS | `artifacts/autonomous_auto09_20260730_evidence` | `—` |
| AUTO-10 | PASS | `artifacts/autonomous_auto10_20260730_evidence` | `—` |
| AUTO-11 | PASS | `artifacts/autonomous_auto11_20260730_evidence` | `—` |
| AUTO-12 | PASS | `artifacts/autonomous_auto12_20260730_evidence` | `—` |
| AUTO-13 | BLOCKED_EXTERNAL | `artifacts/autonomous_auto13_20260730_evidence` | `real_domain_auditable_ground_truth_dataset_not_available` |
| AUTO-14 | BLOCKED | `artifacts/autonomous_auto14_20260730_evidence` | `dependency_AUTO-06_formal_model_not_selected` |
| AUTO-15 | BLOCKED | `artifacts/autonomous_auto15_20260730_evidence` | `dependency_AUTO-08_learned_spot_cleaning_blocked` |
| AUTO-16 | PASS | `artifacts/autonomous_auto16_20260730_evidence` | `—` |

最终边界：软件与发布工程可完成；AUTO-15 仿真综合矩阵、真实域和 J6 实体门未通过。
