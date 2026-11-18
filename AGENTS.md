# Repository maintenance

Read `docs/ARCHITECTURE.md` for runtime ownership and module boundaries before
changing cross-project behavior. Each project's README owns its operating steps.

- Public projects are registered in `docs/projects.json`. When changing names,
  directories or entrypoints, update the registry and run
  `python3 tools/check_repository.py --write-index`; do not edit the generated
  README project tables by hand.
- Update the corresponding architecture/algorithm document with changes to
  process ownership, clock handling, coordinates, calibration or data layout.
  Refer to parameter files instead of duplicating long numeric tables.
- Keep original bags, image datasets, weights and generated runs in their
  existing local locations. Configure `semantic_mapping/paths.local.json`;
  never move data as part of a source-only reorganization or commit that file.
- Keep recording protocol identifiers and deployed robot scripts stable during
  display-name or documentation changes. Robot hardware and ROS 1 Melodic
  belong to the robot/control-PC split described in the architecture guide.
- Stage the intended changes, then run `python3 tools/check_repository.py` and
  `git diff --cached --check`. The checker validates published files, links and
  entrypoints, not robot accuracy or the complete meaning of algorithm prose.
- Run affected offline tests. Full checks are listed in
  `.github/workflows/repository.yml`; ROS GUI and integration tests need the
  control-PC environment and must be reported separately from hosted CI.
- Demonstration assets belong in the relevant project's `docs/images/` (or
  existing `screenshots/`). Keep full-size captures locally and publish a
  readable web version; the repository checker enforces a per-file size budget.
