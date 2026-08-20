# Close-Range Classifier Contract Recovery V11 Report

Final state: **Stop condition B — R1/R2/R3 all failed.** `CLOSE_RANGE_CLASSIFIER_CONTRACT_BLOCKED=true`, `MODEL_BLOCKED_INTERNAL=true`, and `SIMULATION_PRODUCT_COMPLETE=false`.

| # | Required question | Final answer |
|---:|---|---|
| 1 | TRAIN/HOLDOUT four-class unique crop counts | Per-class counts are not retained in the cleaned active repository; the retained aggregate shows only 9 unique V10 TRAIN background crops. |
| 2 | Background unique sources and sampler repeat | 9 unique V10 TRAIN background crops; expected replacement-sampler repeat factor about 186.64. |
| 3 | Near-miss background label noise | No evidence supported it as the primary failure cause; ambiguous C11 proposals were ignored rather than labeled background. Exact row count is not retained here. |
| 4 | TRAIN runtime-faithful fraction | 0.40. |
| 5 | Five-view A/B/C/D/E metrics | The compact retained package preserves the conclusion, not the full diagnostic matrix; GT/proposal tight macro-F1 differed by only 0.0064. |
| 6 | Background F/G/H/I specificity | Full matrix is not retained here; frozen V10 background controls failed, while R1 recovered aggregate HOLDOUT background specificity to 1.0. |
| 7 | Root cause | Supported: background scarcity/repetition, TRAIN-runtime view mismatch, augmentation/context-contract problems, and target-class confusion. Unsupported as primary cause: proposal crop geometry or RGB channel parity. |
| 8 | C11 background bank | 6,576 unique tight background crops, with zero exact/pHash overlap across splits. |
| 9 | R1 | Combined macro-F1 0.6075; target macro-F1 0.5397; background specificity 1.0000; failed. |
| 10 | R2 | Combined macro-F1 0.6311; target macro-F1 0.5829; background specificity 0.6333; failed. |
| 11 | R3 complementary evidence | Yes, 22.76% tight/context complementary correctness; R3 still failed. |
| 12 | Final classifier route | None. |
| 13 | Candidate-level macro-F1 | R1 0.6075; R2 0.6311; R3 0.4561; none passed. |
| 14 | ActionVerifier wrong actionable | NOT_EXECUTED — dependency blocked. |
| 15 | False/wrong CLEAN_NOW | NOT_EXECUTED — dependency blocked. |
| 16 | DEV_VAL one-shot | Not accessed. |
| 17 | VAL_NEW used for training | No; not accessed. |
| 18 | Tracker far-to-close continuity | NOT_EXECUTED. |
| 19 | DynamicTrashMap precision/coverage/RMSE | NOT_EXECUTED. |
| 20 | Full Gazebo Online | NOT_EXECUTED. |
| 21 | x86 Hz/P95/drop | NOT_EXECUTED. |
| 22 | Freeze ID/hash | Not created. |
| 23 | G5_V2 one-shot | Not accessed. |
| 24 | Formal 30-seed result | NOT_EXECUTED. |
| 25 | Spot Cleaning zero wrong cleans | NOT_EXECUTED. |
| 26 | Camera-backed CLEANED | NOT_EXECUTED. |
| 27 | 2 h soak | NOT_EXECUTED. |
| 28 | MCAP replay | NOT_EXECUTED. |
| 29 | Release ZIP/hash | Not created. |
| 30 | Simulation product state | `SIMULATION_PRODUCT_COMPLETE=false`. |
| 31 | PR state | PR #91 is closed and superseded by Draft PR #92; the failed classifier lane was not merged or deployed. |

The protocol forbids R4/R5, a new detector search, sealed-data tuning, or lowering product gates. All downstream perception-product stages remain dependency blocked.
