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

The full retrospective audit now covers the last five accepted paths at three
terminal states. Only frame 046 passes at the four-camera margin failure; none
passes the front-only margin failure. All five are rejected at the older 7.9 s
no-margin failure because the actual body is already outside the lane (three
also fail remaining-path clearance). Evidence:
`historical-failure-cache-audit.json`. Retention cannot hide that prior departure.

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

Reproduction command (local simulator must already be running):

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

The live run first exhausted all six fresh pools at 4.3 s. It reused native
frame 040, originally generated at 4.0 s, with age 0.3 s. At reuse, current
rear-axle distance to the fixed path is 0.0040 m, heading difference 1.7867°,
actual corner clearance 0.4048 m and remaining predicted clearance 0.1670 m.
Remaining native target speed is 3.9864 m/s. The first 44 controller-input
records, including this reuse, pass exact source/array/anchor/speed/age checks.
The fresh failure remains in `replan-rejections/frame_043.json`. This is direct
evidence of bridging one failed planning step, not a completed maneuver.

The same original plan is reused at 4.4 s (age 0.4 s), then fresh sampling
resumes at 4.5 s. At **4.6 s**, all 48 fresh candidates fail and all three
eligible cached plans fail the required 0.10 m remaining clearance. Their
best remaining clearances are 0.0940 and 0.0923 m; the third leaves the lane.
The previously reused frame 040 is now 0.6 s old and is expired. The closest
fresh candidate has 0.0888 m clearance. Bounds are not relaxed to continue.

Final result: **incomplete left turn**, 11.460° rotation, 4.6 s duration,
minimum actual speed 2.7832 m/s, final 3.2755 m/s, maximum 9.5427 m/s,
no near-stop, no recorded box-corner departure, minimum actual corner clearance
**0.3192 m**. There are 87 proposal calls, 46 executed native arrays and two
explicit reused steps. All source/anchor/speed/age and raw controller-input
identity checks pass; zero applied-control, snapshot-position or frame mismatch.
Ten visual signal checks report green. Neither route exit nor a complete turn
is reached. Different rendered inputs and different selected approach paths
prevent treating duration versus the prior 5.0 s run as a controlled A/B result.

Evidence: `native-left-retention-{control,footprint,route,retention}-audit.json`,
capture `retention-failure.json`, preserved `replan-rejections/`, and video
`native-left-retention-incomplete.mp4`. The overhead video keeps the actual
prediction age. Its yellow line is the full native 6.4 s forecast, whose tail
can still leave the lane; admission checks only the covered near-term segment.
The generic `000_verifai/predictions.jsonl` timestamps execution. For retained
paths, use the explicit original source context in `controller-inputs.jsonl`
or each frame's `retention` field to reconstruct the original world anchor.

## Earlier speed-quality resampling

The live approach offered only high-speed, body-admitted candidates in several
first pools. At 2.0 s their target speeds were 9.29 and 10.07 m/s; the existing
5 m/s preference ranks available candidates but does not trigger another pool
while any candidate passes. Preparing a frozen six-pool comparison at 1.9 s
to test whether earlier retries can offer slower native plans. No speed-profile
editing or controller change is proposed.
