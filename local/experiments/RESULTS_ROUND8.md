# Native sampling with controller-aware validation — round 8

Branch: `experiment/driving-tracking-validation`, starting checkpoint `d9a7281`.
Evidence root: `outputs/driving-investigation-8/`.
Previous evidence and paired native replay: [round 7](RESULTS_ROUND7.md).

## Scope

Every executed path and its speed array must remain an original model sample.
No waypoint repair, map-generated fallback, forced speed or controller change
is permitted. Production launchers, defaults and external model weights/code
remain unchanged. The separate `tracking_probe.py` entrypoint adds an optional
approximate tracking check to the existing native validation/resampling and
proactive recent-plan comparison. Return to `d9a7281` to discard this experiment.

## Motivation and policy

In round 7, geometrically valid native paths still caused actual body departures
when followed by the unchanged controller. Faithful replay reproduced the
complete source prefix exactly. At the same 6.0 s state, original candidate 4
departed at 7.4 s, while candidate 7 from the same model pool remained inside
through 8.0 s. An offline tracking forecast preferred candidate 7. This is a
paired short replay, not proof of closed-loop completion or broad reliability.

The new check copies the real stateful follower and invokes the unchanged base
controller on a constant-speed bicycle forecast, with effective steering-angle
gain **0.8448538582454754**, horizon **1.5 s**, and required predicted corner
reserve **0.20 m**. The gain is the median measured from 40 turning steps in
three older round-7 captures, excluding the proactive capture used for the
paired replay. See `tracking-forecast-heldout-audit.json` in round 7.

Forecast poses are used only to reject samples. They are never passed to the
real controller as an executed trajectory. Forecasting copies the follower's
integrator, steering history and target-speed filter, and bypasses experimental
logging/retention wrappers when calling its unchanged base implementation.
The actual follower state is not advanced by forecasts. Constant speed is a
forecast assumption, not an applied speed command. The approximation omits
longitudinal response, tire slip and varying roll/pitch; it overestimated
clearance by up to 0.186 m in the earlier held-plan comparisons and is not a
certificate of safe tracking or obstacle avoidance.

All existing native body, reverse, route, format and green-progress checks
remain required. Failed fresh candidates trigger the existing sampling budget
(up to six pools of eight). Existing cached paths must also pass the additional
tracking check, using their original fixed world anchor, speed array and real
age. The 0.5 s cache limit and all other cache checks remain unchanged. Ranking
among admitted candidates remains unchanged. No low-level actuation is edited.

`tracking_forecast` metrics record the minimum forecast clearance and its time,
gain, horizon, required reserve, assumed speed and original plan age. Rejected
samples receive `insufficient_controller_tracking_clearance`. Native proposals,
chosen arrays and rejected pools continue to be saved by the existing probes.

## Verification

Combined original controller, adapter, dataset, VerifAI, native validation,
retention, proactive comparison and tracking tests: **104 passed in 5.20 s**.
New tests check copied follower state, exact unchanged path/speed arrays and
real age; reject a predicted departure; prevent readmission of a natively
invalid sample; fail without an original follower; and restore process-local
wrappers after exceptions. Invalid/nonfinite configuration is rejected.

## Live test

Preset: `presets/tracking-native-four-left-quality-margin20-43.json`.
Relative to the previous proactive run, the only policy change is the added
tracking screen. Original four native pinhole cameras, text seed 43, motion
seed 42, native decoder, original controller, empty all-green Town03 scene.
Rendered images and later history may differ, so this is not an exact-input
whole-rollout A/B comparison.

```bash
source local/env.sh
.venv/bin/python -m local.experiments.tracking_probe capture \
  --config local/configs/demo.yaml --duration 16 \
  --capture-frames 0 19 40 50 60 64 68 74 80 90 \
  --options local/experiments/presets/tracking-native-four-left-quality-margin20-43.json \
  --output outputs/driving-investigation-8/capture-tracking-native-four-left-margin20-43
```

Status: pending full-inference result and actual-footprint/native-identity audits.
