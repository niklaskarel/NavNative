# Editing

Use `apply_patch` for every repository file edit, including file creation, modification, renaming, and deletion. Do not use shell commands, scripts, or formatters to write repository files directly. Run formatters in check mode or against temporary copies, then apply any formatting changes with `apply_patch`.
