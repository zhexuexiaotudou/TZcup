# Formal runtime source-cache hygiene

`scripts/prepare_formal_runtime_source_cache_hygiene.ps1` is the prebuild cache gate for the fixed `starter_ws/src` tree. Its default is a dry run: it inventories only `__pycache__`, `.pytest_cache`, and `.pyc` files outside `__pycache__`, then writes `reports/engineering/formal_runtime_source_cache_hygiene_dry_run.json`.

Before either reporting or deleting, it verifies that its own repository root matches `git rev-parse --show-toplevel`; refuses out-of-root paths, reparse points/symlinks, and all paths reported by `git ls-files`; and records an auditable candidate manifest. Deletion is possible only with the explicit `-Execute` switch. A dry run never removes files.

The report is planning evidence only. It does not claim a ROS build, WSL session, Gazebo run, or runtime acceptance.
