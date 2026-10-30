# Native model-plan retention — round 7

Branch: `experiment/driving-native-plan-retention`, starting checkpoint `0a2eb4a`.
Evidence root: `outputs/driving-investigation-7/`.
Previous results: [round 6](RESULTS_ROUND6.md).

## Scope and hypothesis

The user requires executed paths to remain model-generated. No map planner,
waypoint repair, artificial speed profile, model patch or production change is
introduced. This separate wrapper tests whether an unchanged, recently accepted
model path can bridge a transient failure to sample a new admissible path.
All production files and defaults remain unchanged.

At the previous four-camera, 0.10 m margin left-turn run's 5.0 s failure, the
native plan produced at 4.6 s passes the new full current-state check: its age is
0.4 s, current rear-axle separation is 0.2291 m, heading difference is 6.4361°,
and its checked remaining corners retain 0.2195 m predicted clearance. Current
actual corner clearance is 0.4287 m. Remaining native speed-preview mean is
5.3045 m/s. Evidence is the round-6 artifact
`previous-native-plan-full-validation.json`. This is offline availability,
not evidence of successful live reuse or completion of a left turn.

## Experimental policy

`retained_probe.py` first performs the existing fresh sampling and validation
(8 samples per pool, up to 6 pools). Only the specific `NoValidatedPlan` sampling
exhaustion is eligible for reuse. Inference errors and route completion propagate.

Each accepted fresh plan stores a cloned native prediction tensor, associated
text, exact original world-coordinate path, native orientation and original
speed array. Reuse never reanchors or edits these arrays, refreshes their source
time, or adds a reused plan to the cache. Maximum original age is 0.5 s.

Admission requires both original and current visual green signals, unchanged
navigation phase, finite current state, current actual body inside the route,
rear-axle distance at most 0.5 m from the old path and heading difference at most
20°. The remaining path must meet the same lane, native body clearance, reverse,
route-heading and optional green-progress checks. Checks cover at least 1.5 s
from the current time, the unchanged follower's actual spatial steering target
and its age-adjusted speed-preview window. This is an empty-intersection lane
check, not collision or swept-volume certification.

The original follower receives the exact stored world path and speeds, using
its existing `elapsed_sec` argument for real prediction age. The video uses the
original path origin/time. Each reused record names its source and age and
preserves all rejected fresh pools in `replan-rejections/`. A controller-input
log independently records actual arrays, age, control and controller diagnostics.
`audit_retention.py` checks native provenance, source freshness, exact array
identity, world anchor, unchanged speeds, real age and explicit rejection logs.

The entrypoint temporarily wraps only references in the experimental process.
No normal launcher imports it. Return to `experiment/driving-plan-validation`
at `0a2eb4a` to discard this experiment after saving any work; outputs stay local.

## Verification

Combined controller, adapter, dataset, VerifAI, native-validation and retention
tests: **90 passed in 4.75 s**. Retention cases cover expiry, changed conditions,
nonfinite state, actual body departure, fixed native arrays and world anchors,
failure-log preservation, propagation of inference errors, and forwarding the
exact original arrays plus real age to the original controller interface.

## Live execution

Pending. Reproduction command (local simulator must already be running):

```bash
source local/env.sh
.venv/bin/python -m local.experiments.junction_routes \
  --config local/configs/demo.yaml --output outputs/driving-validation-routes.json
.venv/bin/python -m local.experiments.retained_probe capture \
  --config local/configs/demo.yaml --duration 16 --capture-frames 0 19 45 \
  --options local/experiments/presets/retained-native-four-left-margin-43.json \
  --output outputs/driving-investigation-7/capture-retained-native-four-left-margin-43
```

This uses the round-6 four-camera margin preset plus the three retention limits.
No longer-forecast preference or measured-speed decoder substitution is enabled.

The first startup failed before any driving inference because the preset's
`outputs/driving-validation-routes.json` had not been generated. The original
attempt and log are preserved with suffix `-setup-failure`. Exported the route
with the command above and verified its geometry equals the round-6 capture's
saved reference exactly before relaunching. No path-policy change was needed.
