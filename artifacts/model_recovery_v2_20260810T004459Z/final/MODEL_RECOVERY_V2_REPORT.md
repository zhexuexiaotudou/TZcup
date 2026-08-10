# MODEL-RECOVERY-V2 final report

## Outcome

MODEL-RECOVERY-V2 did not produce a freeze-eligible x86 model. MRV2-A, MRV2-B and MRV2-C all failed the unchanged static development gate, so `MODEL_BLOCKED_INTERNAL=true`. G5 and legacy D6 remained unread; no freeze, deployment, soak, replay, J6 or field performance claim was made.

## Required answers

1. Selected route: none. A/B/C all failed.
2. Small-object recall: historical X3 `0.3077`; best formal MRV2 VAL result was MRV2-C `0.4615`; cross-world `0.5263`, below `0.70`.
3. MRV2-C metal_can recall: VAL `0.8125`; D1/D2/D3/D4 `0.6731/0.6471/0.8409/0.5741`. It does not satisfy VAL `0.90` and every-domain `0.70`.
4. Area: VAL boundary F1 `0.6880`; D4 boundary `0.5097` and negative-area FP/frame `0.8667`. Not all gates passed.
5. Grounding DINO: official checkpoint obtained and executed over holdout+VAL+D1-D5. VAL candidate recall `0.0219`, small recall `0.0000`; reference gate failed. Historical X2 remains unchanged.
6. Sealed final: not opened; static prerequisite failed.
7. 30-seed moving-camera: not run; no valid freeze/sealed-final pass.
8. DynamicTrashMap: software remains implemented, but no MRV2 formal product pass was unlocked.
9. Spot Cleaning/post-clean: not run under MRV2 because prerequisites failed; no CLEANED claim.
10. 2h soak/MCAP replay: not run; no frozen product pipeline existed.
11. x86 release: none; path/hash are null.
12. J6: not started under MRV2 because the x86 teacher never froze; board remains absent.
13. Field: no qualifying RGB-D/independent GT; no field metric was fabricated.
14. PR #90: remains Draft and open.
15. Remaining blocker: internal model quality first; physical J6, real RGB-D and independent GT also remain external resources but are not the reason execution stopped.

## Route evidence

- MRV2-A: VAL macro F1 `0.9450`, small recall `0.4103`, metal_can `0.9062`, FP/min `21.6`.
- MRV2-B: bounded tiling added no accepted small candidates and remained failed.
- MRV2-C: 28/102 eligible TRAIN small truths received teacher-refined geometry; P2 training completed 6x600 frames. VAL macro F1 `0.9299`, small recall `0.4615`, FP/min `20.4`.

No later gate is inferred from these development results.
