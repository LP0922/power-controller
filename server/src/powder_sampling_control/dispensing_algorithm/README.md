# Dispensing Algorithm

Pure algorithm layer for powder dispensing — nothing here opens a serial port
or issues device commands; the executor and task state machine own all
hardware writes.

The package is consolidated into two modules:

## `core.py` — production chain

1. **Product requirements & contracts** — `load_product_requirements`,
   `validate_target_mass`, `acceptance_limits`, `FeedAction`
   (the live parts of the former `models.py`).
2. **Mass-motion observer** — `MassMotionEstimator` applies a
   variable-time alpha-beta-gamma observer to the 1 mg balance measurements,
   estimating mass, flow, acceleration, and filtered jerk, and predicting
   mass/flow 0.75 s ahead. Raw balance values are never differentiated
   directly.
3. **Profile seeds & control profiles** — software seed presets
   (`DISPENSE_PRESETS`), per-target profile interpolation, powder
   classification parameters, and the `data/control_profiles/*.json`
   resolver that piecewise-linearly resolves continuous controller
   parameters for every integer target from 100 through 1000 mg
   (the former `profiles.py`).
4. **Continuous feedback controller** — `ContinuousFeedbackController`,
   the production two-loop controller: a fast bounded duty loop plus a
   slower persistent-error window-position loop, with predictive stop,
   tail taper, stall recovery, and flow-hold logic
   (the former `feedback_controller.py`).

The example L0 data in `config/algorithm/l0_profiles.example.json` was part
of the removed four-stage planner and is no longer consumed.

## `calibration.py` — calibration & experiment chain

1. **Constant-rate controller** — `ConstantRateController` for
   constant-flow tests with duty-first, frequency-second, window-last
   actuator priority.
2. **Continuous taper planner** — `ContinuousTaperPlanner` for the 500 mg
   tail with fixed stages and predictive stop.
3. **Window PID planner** — `WindowPidPlanner` combining window
   feedforward with PID trim.
4. **Powder fingerprint probe** — `execute_probe` plus feature
   extraction, classification, and fingerprint storage
   (`data/powder_fingerprints/`).
5. **Rate-band searcher** — `execute_rate_search` finds (frequency,
   window, duty) points that hold a steady 10–30 mg/s flow.
6. **Experiment grid orchestrator** — `GridSpec`/`expand_grid`/
   `collect_scan_results` for declarative parameter-grid experiments and
   powder-library write-back.

## Powder control profiles

`data/control_profiles/{powder_id}.json` is the runtime bridge between a
fixed powder workspace and the feedback controller. Each file stores common
controller settings plus a small set of target-mass anchors. The resolver in
`core.py` accepts every integer target from 100 through 1000 mg and
piecewise-linearly resolves continuous target parameters. JSON anchor keys
are normalized to integers when loaded.

The current files are `draft` configurations derived from incomplete
workspaces. The dashboard exposes their warnings and each submitted run logs
the profile status and whether its target was exact, interpolated, or
extrapolated. Use `scripts/plan_cli.py profile POWDER_ID` to preview a new
draft after updating a workspace.

Use `scripts/device_maintenance.py calibrate-yield` to collect a physical
stable mass-delta sample. It is preview-only unless `--execute` is supplied.

## Removed legacy modules

The four-stage planner (`four_stage_planner.py`), L0 repository and yield
estimator (the bulk of the old `models.py`), constrained ILC (`ilc.py`),
dynamic rate planner (`dynamic_rate_planner.py`), auto rate controller
(`auto_rate_controller.py`), and the legacy `scripts/dispense_500mg.py`
workflow were removed. They remain reachable through git history.
