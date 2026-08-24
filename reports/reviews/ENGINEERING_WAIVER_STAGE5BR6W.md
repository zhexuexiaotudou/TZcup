# Stage5BR6W engineering-only waiver

Stage5BR6W is an isolated engineering lane. Two independent human reviewers are unavailable, so the sealed Stage5BR6-A packages remain untouched and can still be used to resume the formal human gate later. No script, AI system, or project member response is treated as human blind-review evidence.

The lane may freeze V4 only as `engineering_verification_camera=V4`, derive an opt-in engineering footprint, and test geometry, Nav2, Coverage, and active observation. It does not select a competition camera, validate human recognizability, authorize model training, or create competition evidence.

The formal status remains fail-closed:

```text
AWAITING_HUMAN_REVIEW=true
HUMAN_REVIEW_COMPLETED=false
MANUAL_AUDIT_PASS=false
READY_FOR_STAGE5BR6_ORACLE=false
READY_FOR_GPT_REVIEW_STAGE5BR6=false
READY_FOR_STAGE5BR7=false
READY_FOR_GPT_REVIEW_STAGE5B=false
READY_FOR_STAGE5C=false
```

Engineering readiness uses only `READY_FOR_STAGE5BR6W_ORACLE_ENGINEERING` and `READY_FOR_STAGE5BR7_ENGINEERING`. Neither field may be copied into the formal readiness fields.
