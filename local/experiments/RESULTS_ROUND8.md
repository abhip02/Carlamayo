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

Completed: **7.7 s**, **50.7883°** left rotation, no route exit. Speed remains
between **2.78562 and 5.17466 m/s**, ending at **4.07124 m/s**, with no near-stop.
Minimum actual corner clearance is **0.24448 m**, at termination. All 77 native
arrays, fixed world anchors, speeds and ages match; all applied controls and
snapshot frames match. Every executed tracking check and original controller
command independently recomputes exactly. Maximum observed one-step forecast
clearance overestimate is **0.003549 m**, and maximum one-step position error
is **0.02459 m**. These are observed errors in this capture, not future bounds.

There are **121** fresh proposal pools, **546** geometrically admitted candidates
screened, and **122** tracking rejections. All **55** cached executions beat a
fresh admitted choice; none bridges complete sampling exhaustion. There are
77 accepted fresh proposals, 16 green VQA checks, and zero control/snapshot
mismatches. Speed-quality retries add 20 pools across seven steps, change six
choices, and leave two optional speed preferences unmet. Wall time is
**1350.4 s (22.5 minutes)**. Evidence:
`tracking-left-margin20-{control,footprint,route,retention,quality,tracking}-audit.json`.
Review video: `tracking-left-margin20-incomplete.mp4` (H.264, labeled incomplete).

At 7.7 s, **16 of 48** new samples pass native geometry but all fail the added
0.20 m tracking reserve. Best forecast reserve is **0.170819 m** (pool 3,
candidate 4); pool 0 candidate 2 has **0.158152 m**. Both minima occur 0.5 s
ahead. The eligible cached sources 072–076 fail native remaining-body reserve,
so the experiment ends explicitly while the actual car is still inside.

## Testing whether the reserve is too conservative

The replay helper now accepts an original rejected `--proposal` plus
`--candidate`. It independently requires the original native format, route,
reverse, body-margin and green-progress checks before executing the untouched
sample. It saves exact candidate arrays/tokens and original context. Three
additional tests check native provenance, immutable arrays, invalid geometry
and nonexistent candidates. Combined regression suite: **107 passed in 5.09 s**.

Two paired diagnostics reproduce the entire new 7.7 s prefix **exactly**, with
zero state and computed-control errors, then hold a rejected native plan for
1.5 s through the unchanged controller:

| Original sample | Forecast minimum | Actual minimum | Endpoint |
| --- | --- | --- | --- |
| `frame_077_pool_3`, candidate 4 | 0.170819 m | **0.153042 m** | 9.2 s, 76.14° total left rotation, 6.066 m/s |
| `frame_077_pool_0`, candidate 2 | 0.158152 m | **0.155646 m** | 9.2 s, 82.90° total left rotation, 7.136 m/s |

Both remain inside throughout the diagnostic, without new inference. They do
not establish a full closed-loop turn or validate long cache retention. The
second accelerates substantially as its original speed preview ages, another
reason to distinguish a held-plan replay from normal replanning. Artifacts:
`replay-terminal-pool3-candidate4/`, `replay-terminal-pool0-candidate2/`.

## Smaller hard-reserve comparison

The separate preset, `tracking-native-four-left-margin10-43.json`, changes
the native-body and tracking reserve requirements from **0.20 to 0.10 m**.
All other options, including 1.5 s forecasts, measured steering gain, original
controller and 0.5 s cache limit, remain unchanged. Both checks use the same
smaller reserve so a car passing through the observed 15 cm clearance region
can still propose fresh native paths. This is explicitly a two-threshold
change, not an isolated attribution to either individual check. The actual
pass criterion remains full lane-corridor containment and reaching the exit.
The earlier 20 cm result and all failed samples remain preserved.

Completed capture: `capture-tracking-native-four-left-margin10-43`.
It ends at **7.2 s**, **46.57394°** left rotation, without reaching the exit.
Minimum actual corner clearance is **0.203714 m**. Actual speed ranges from
**2.78541 to 6.38413 m/s**, ending at **5.49457 m/s**.
All 72 executed native arrays, fixed anchors, speed arrays and original ages
match. All 72 tracking forecasts and unchanged controller commands independently
recompute exactly. There are 148 proposal pools, 572 geometrically admitted
candidates screened and 146 tracking rejections. Of 47 cached executions, 44
beat a fresh admitted choice and three bridge exhausted fresh sampling.

At termination, all 48 fresh candidates fail; 14 pass native geometry but
fail the 0.10 m tracking reserve. An offline diagnostic removes the signal
gate only for analysis: source 067 has 0.111707 m remaining native clearance
but only 0.091748 m forecast clearance; source 070 has 0.040167 m native
corner overhang; source 071 has only 0.091661 m native clearance. Thus no
remaining cached plan passes the physical checks, independently of the
current unknown-signal result. Signal rules were not changed or bypassed
in execution. Evidence: `margin10-terminal-signal-gate-diagnostic.json`.

Maximum observed one-step clearance overestimate is **0.015839 m**, and
maximum one-step position error is **0.061451 m**. These are measured errors,
not future bounds. Speed retries add 38 pools across ten steps, change five
choices, and leave seven optional speed preferences unmet. Wall time is
**1646.4 s (27.4 minutes)**. Full audit files:
`tracking-left-margin10-{control,footprint,route,retention,quality,tracking}-audit.json`.

## Prefer 20 cm, retain a native 10 cm alternative

The next isolated preset, `tracking-native-four-left-preferred20-min10-43.json`,
keeps both hard minima at 0.10 m and adds a 0.20 m preference for both native
body geometry and controller-tracking forecast. Candidate ranking first
prefers samples meeting both preferred margins, then uses the existing
horizon preference and numeric score. If a pool provides only hard-admitted
samples below the preference, later pools are tried within the original
six-pool budget. The best admitted sample and its exact native tensor/text
are retained if subsequent pools fail. Fresh versus cached comparison uses
the same ranking. No extra sampling budget, signal change, cache extension,
path repair or speed override is introduced.

This tests whether the stricter policy's earlier choices can be preserved
while allowing native continuations in the 10–20 cm region when necessary.
The two rejected native continuations above justify investigating this region,
but do not establish its general reliability. The new preference is opt-in;
without it, ranking and retry behavior remain unchanged.

Pending capture: `capture-tracking-native-four-left-preferred20-min10-43`.
Return to `434d6c1` to remove this preference experiment alone.

## Additional offline checks during the live run

`audit_tracking.py` reconstructs the original follower state from recorded
actual poses/speeds and controller inputs. It recomputes each executed plan's
tracking forecast and independently checks the original control command,
forecast clearance, current measured speed and real plan age. The first 11
completed live steps match exactly, with zero forecast and controller error.
This checks implementation fidelity, not the physical model's accuracy.
The final full audit is required after the rollout ends.

The added screen alone was also evaluated on both previously successful
front-camera straight captures. All 85/79 original control commands reconstruct
exactly. With a 0.20 m forecast reserve, 48/44 of their executed samples would
be rejected; two in each run have a negative approximate forecast clearance.
Those earlier captures required zero additional native body reserve, and their
actual minimum clearances were only 0.112/0.068 m. This is not a controlled
alternate rollout: later replanning can correct a forecast departure, and a
20 cm margin is intentionally stricter than those earlier settings. It does
show why this screen should not become a general default based on one paired
left-turn replay. Evidence:
`tracking-screen-successful-straight-retrospective.json`.

Round-7 paired comparison artifacts are now available as
`native-paired-replay-6-to-8s.mp4` and `native-paired-replay-clearance.png/.pdf`.
Both video and plot were visually checked. The video freezes the original
candidate's frame after its 7.4 s departure while the alternative continues to
8.0 s; labels and frame timestamps distinguish the endpoints. The plot compares
native predicted corners, approximate controller tracking and actual CARLA
corners. The alternative's maximum forecast-clearance error over its two-second
replay is 0.1224 m, versus 0.3078 m for ideal geometry. These errors are measured
on this specific replay, not bounds for future scenes.


Preference verification: **157 passed in 5.21 s**, covering all `tests/` and
`local/experiments/` tests. Import the installed `carla` module before pytest
collection because an upstream test installs an incomplete CARLA stub when
it is absent; collecting both suites without that import fails unrelated
geometry tests. The experiment suite alone passes **51 tests**. New cases
check preference across retry pools, retaining the exact best tensor/text
when later samples fail, fresh/cache preference consistency, and separation
of hard admission from preference. Both completed captures still pass the
updated native retention audit (72/77 steps). Production source and external
model source remain unchanged.

```bash
source local/env.sh
.venv/bin/python -m local.experiments.tracking_probe capture \
  --config local/configs/demo.yaml --duration 16 \
  --capture-frames 0 19 40 50 60 64 68 74 80 90 100 110 \
  --options local/experiments/presets/tracking-native-four-left-preferred20-min10-43.json \
  --output outputs/driving-investigation-8/capture-tracking-native-four-left-preferred20-min10-43
```


## Frozen-input preference checks and audit additions

Re-evaluating all **77** strict-run fresh decisions with the preferred-20/minimum-10
policy reproduces **77/77** proposal/candidate choices using the same archived
model samples and independently reconstructed original controller state.
At the strict run's terminal frame 077, **six of 48** archived native candidates
pass the new hard checks; none meets the preferred reserve. The top-ranked
candidate is pool 1 candidate 2, with native clearance 0.237840 m, forecast
clearance 0.144397 m and native target 5.326040 m/s. This candidate differs from
the two already replayed alternatives and has not itself been physically replayed.
These checks do not reproduce new rendered inputs or a counterfactual live run.
Evidence and exact scripts: `preferred-reserve-frozen-prefix-comparison.json`,
`preferred-reserve-frozen-terminal77.json`, `check_preferred_reserve_prefix.py`,
`check_preferred_reserve_frozen.py` under the round-8 evidence root.

The quality audit now independently checks the best admitted candidate across
all observed pools and the reserve annotations against archived native/forecast
clearances. The tracking audit also checks each executed preference flag against
its recomputed forecast. Both completed prior runs pass the added selection
checks. The new capture requires a full final audit after completion.
The smaller-hard-reserve video is `tracking-left-margin10-incomplete.mp4`
(H.264, labeled incomplete, visually checked).


## Offline acceleration-aware validation diagnostic (not adopted)

While the preference rollout runs, `check_longitudinal_forecast.py` tests a
simple alternative forecast, using only saved data and no simulator ticks.
It fits acceleration from prior acceleration, original throttle, original
brake and speed on the same three older round-7 captures used for steering
calibration, excluding the evaluated captures. The first second is excluded
because startup/gear engagement is not represented. Coefficients are bounded
with nonnegative throttle/braking/drag effects and acceleration memory in [0,1].
The original controller is copied for prediction; its path/speed inputs remain
the untouched native sample. Forecast poses are never executed.

Fit: **152 steps**, acceleration RMSE **1.654 m/s²**. The fitted coefficients
(previous acceleration, throttle, negative brake, negative speed) are
`[0.5760644, 2.7896488, 3.0336509, 0.2132257]`. Seven already observed native
replays are evaluated. These are diagnostic comparisons, not a new independent
prospective test set. The running policy is unchanged.

| Saved native replay | Max position error, constant → fitted (m) | Max clearance error, constant → fitted (m) |
| --- | --- | --- |
| `replay-native-tracking-earlier060` | 0.404 → 0.245 | 0.147 → 0.055 |
| `replay-native-tracking-earlier064` | 0.550 → 0.198 | 0.186 → 0.102 |
| `replay-native-tracking-earlier060-candidate7` | 0.403 → 0.322 | 0.122 → 0.074 |
| `replay-native-tracking-source064` | 0.027 → 0.025 | 0.023 → 0.022 |
| `replay-native-tracking-source068` | 0.027 → 0.025 | 0.021 → 0.020 |
| `replay-terminal-pool3-candidate4` | 1.245 → 0.177 | 0.052 → 0.035 |
| `replay-terminal-pool0-candidate2` | 2.091 → 0.192 | 0.083 → 0.124 |

Position prediction improves in all seven comparisons, but clearance prediction
gets worse for terminal pool 0 candidate 2 (0.083 → 0.124 m maximum error).
The earlier-064 replay still has an actual corner departure while the fitted
forecast's minimum remains slightly positive (0.005 m). Therefore this simple
model is not sufficient to certify tracking and is **not adopted**. Its results
are retained for a later validation study, not presented as a driving fix.
Evidence: `longitudinal-forecast-diagnostic.json` and the exact diagnostic script.


## Map-corridor footprint sanity check (no path execution)

`route-ideal-footprint-diagnostic.json` checks the recorded Lincoln box at map
route poses, excluding the two truncated reference ends. Aligning the actor
center with the route leaves at least **0.568518 m** clearance. Aligning the
rear axle with the route instead leaves at least **0.204921 m**. This rules
out the simple explanation that the route's corridor is inherently narrower
than the aligned vehicle footprint everywhere in the turn. It does not prove
dynamic feasibility or controller trackability. These map poses are used only
for this geometry calculation and are never passed to execution.
