# Upstream comparison and full-run driving validation — round 15

Branch: `fix/driving-native-validation`; rollback: `ced1ba7`.
User asks to make driving work while keeping the simulator/backend. Preserve
model-generated paths and keep all changes reviewable/revertible.

## Upstream evidence

Fetched `upstream/main` on 2026-11-02; its latest commit is still `4914558`.
There is no newer upstream driving fix to merge. Our original controller/config
match that revision. The original README demo was introduced at `578170c` for
Alpamayo R1, before the current 1.5 setup. Code shipped with that demo uses a
minimum 10 km/h speed target based on trajectory extent, and projects its steering
target onto a driving-lane map waypoint. This is not the same control policy as
following native 1.5 speeds and paths. The demo alone therefore does not establish
that our current configuration works. No change to CARLA or model backend is
needed to investigate this difference; no map-projected executed path is added.

References:
- https://github.com/aveeslab/Carlamayo/blob/578170cfbcb75222ff1fb6cd9e2d4c7d6167a4f9/carla_alpamayo_closed_loop.py
- https://github.com/aveeslab/Carlamayo/blob/4914558/module/trajectory_follower.py

## Predeclared full fresh checks

`presets/native-driving-phase-speed.json` combines the existing opt-in phase
transition and bounded low-speed resampling changes with the prior native-path
validation profile. It does not modify the controller, model, rendering or
validation thresholds. `presets/native-driving-full-checks.json` declares two
previously failing initial conditions: offset/faster straight (previous failure
4.5 s) and centered left (previous failure 5.7 s). Each has a 15 s cap and must
reach the original route endpoint. All inference begins at time zero; no history
is replayed. These are targeted full checks, not a replacement five-case score.

First run: `02-straight-offset-faster`. Second run is queued manually after the
first is inspected, so implementation revisions cannot silently change a running
batch. All outcomes, source hashes, inputs, proposals and audits are retained
under `outputs/driving-investigation-15/`. Results pending at this checkpoint.


## Separate bounded planning-interval experiment

While the unchanged combined-policy full run executes, a separate
`plan_interval_probe` entrypoint adds optional 0.3 s planning intervals. Every
0.1 s control tick revalidates the last executed native plan against actual pose,
unchanged body/age/distance/heading bounds and original-controller tracking.
Reuse additionally requires source/current green, a still-current visual check,
no instruction-phase or endpoint transition, and the configured speed/clearance
preferences. Any failed gate immediately invokes the existing fresh sampler.
Unknown/red/yellow signals therefore continue normal fresh planning. The default
interval is 0.1 s. No native path/speed or controller parameter is edited.

This targets unnecessary per-tick plan changes and inference cost. It is not
assumed to improve driving until physically tested. Source clocks are never reset;
new per-frame evidence explicitly records zero new inference, and the audit
recomputes native geometry and scheduling eligibility in addition to original
controller forecasts. Requested raw input captures remain available on skipped
inference steps. Summary counts separate executed commands, scheduled native
execution, fresh planning and actual sampled pools. The complete 179-test suite passes in 4.09 s, including wrapper restoration,
requested input capture, native identity, audit tampering and failed-gate cases.

Predeclared next comparison: the same offset/faster straight initial condition,
with fresh inference from time zero and the same 15 s cap, changing only the
0.3 s interval and its explicit entrypoint. Preserve both full-run outcomes.
Preset: `presets/native-driving-interval.json`. This new code is not imported by
the already running combined-policy baseline, whose recorded source hashes and
launch commit remain `2d512a2`.


## Footprint-horizon diagnostic (rejected lead)

`screen_body_horizon.py` screens all 96 saved candidates from the original
centered/offset straight terminal failures with native footprint limited to
1.5 seconds, while retaining center/heading/reverse checks over all required
controller inputs and the unchanged tracking forecast. It admits zero candidates
in either case. This does not support shortening native footprint checks; no
validation threshold or horizon was changed. No candidate was executed.
Evidence: `body-horizon-screening.json` and its log.

## Reproducible launcher

`bash local/run_driving.sh --route straight` (or `--route left`) resolves the
predeclared full-run settings. It generates lane-reference geometry from the
configured CARLA map, removing the dependency on ignored investigation files.
`--print-config` previews settings without connecting; `--duration`, `--seed`,
`--flow-seed`, and a one-rollout `--config` provide explicit overrides. The caller
starts/stops the existing server normally. Recordings and exact resolved settings
are saved under the requested `--output`, which must not already exist.

The default replan interval remains 0.1 seconds. `--replan-interval 0.3` selects
the separate interval experiment. The 190-test suite passes in 4.13 seconds,
including launcher validation and unchanged user-config input. Physical launcher
validation remains pending here; the running baseline still uses its original
entrypoint and source snapshot. Expanded source capture includes the launcher,
route exporter, candidate reserve, original runner, adapter, and inference code.
