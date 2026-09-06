# S100P DOSOD HBM evidence producers

The three producer tools below are deliberately separate from board deployment
and final S100P acceptance.  They use only caller-supplied paths and a fresh,
empty evidence output directory.  None contains board copy, SSH, ROS launch,
actuator, calibration capture, or receipt-synthesis behavior.

1. `scripts/execute_dosod_hbm_compile.py` executes one verified `hb_compile`
   command after the contract preflight and live compiler identity agree.  Its
   only success status is `COMPILED_NOT_BOARD_ACCEPTED`.
2. `scripts/run_dosod_hbm_x86_parity.py` compares an actual runner-produced
   HBM result with ONNX on a frozen, non-calibration holdout.  It requires an
   absolute, hashed and versioned runner identity, its exact command template,
   a matching verified input adapter and output map, plus an explicit approved
   tolerance; unknown raw binary layouts are blocked rather than guessed.
3. `scripts/validate_dosod_quantized_metric_regression.py` compares retained
   ONNX and HBM evaluator reports over the same frozen holdout.  It binds the
   full passed parity record, calibration/holdout digests, runner identity and
   absolute/hash/version-bound evaluator identity; hand-written metric JSON is
   insufficient.  A metric pass does not claim board deployment or the
   1800-second runtime gate.

The required order is compile-contract validator, ONNX/toolchain preflight,
actual compile receipt, x86 or board-equivalent parity, then metric regression.
Every producer requires a previously nonexistent, non-symlink evidence root;
the compiler also rejects a pre-existing or symlinked working directory and
expected HBM, so an older HBM can never be relabelled as this run's output.
The four-role payload manifest and all five real board receipts remain required
by `scripts/validate_s100p_final_predeploy.py`.

Current external blockers are not softened by these tools: an authorized real
camera graph, at least 500 unique calibration tensors, an independent frozen
holdout, approved metric thresholds, a verified HBM runner serialization
contract, a project DOSOD HBM, and a measured positive input-power source.
