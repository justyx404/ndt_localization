# Archived localization validation

The executable benchmark harness was removed from the production package after
Phases 0–4 and the post-Phase 4 refactor passed their recorded-bag regression
runs. The retained Markdown reports are historical validation evidence; they
are not installed and add no runtime dependency.

The production package now installs only:

- `config/localization.yaml`;
- the localization executable and its C++ libraries and headers.

The production `spot_navigation/launch/lio_localization.launch.py` starts the
localization executable directly and loads that YAML.

The removed replay harness can be recovered from Git commit `2588308` if a
future algorithm change requires the same bag-validation workflow.

Phase 5 temporarily recovered that archived harness outside the source tree
and used odometry-synchronized artificial priors because recorded
`/initialpose` is incomplete across the held-out bags. No replay Python or
YAML file was restored to the production package.

Validation records:

- `BASELINE_OBSERVATIONS.md`
- `PHASE1_VALIDATION.md`
- `PHASE2_VALIDATION.md`
- `PHASE3_VALIDATION.md`
- `PHASE4_CONFIG_MINIMIZATION.md`
- `PHASE4_REFACTOR_CLEANUP.md`
- `PRODUCTION_DIAGNOSTIC_REMOVAL.md`
- `PHASE5_VALIDATION.md`
