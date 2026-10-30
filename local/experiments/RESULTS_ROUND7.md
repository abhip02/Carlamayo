# Native model-plan retention — round 7

Branch: `experiment/driving-native-plan-retention`, starting checkpoint `0a2eb4a`.
Evidence root: `outputs/driving-investigation-7/`.
Previous results: [round 6](RESULTS_ROUND6.md).

## Current status

The left turn is still incomplete. Every executed path remains model-generated.
The completed tests are:

| Test | Outcome | Actual speed range | Minimum actual corner clearance |
| --- | --- | --- | --- |
| Retention, 0.10 m predicted margin | Ends 4.6 s, 11.5° left; 2 reused steps | 2.783–9.543 m/s | 0.319 m |
| Retention + speed-quality retries | Ends 8.3 s, 59.8° left; 7 reused steps | 1.759–6.468 m/s | 0.029 m |
| Same, 0.20 m predicted margin | Ends 5.3 s, 5.2° left; 4 reused steps | 2.788–6.031 m/s | 0.243 m |

All remain inside the recorded lane corridor and have no near-stop below
0.25 m/s, but none reaches the exit. The second has a much steadier approach
followed by a mid-turn slowdown and very small clearance. Whole-rollout
comparisons are descriptive because rendered inputs differ. Exact frozen-input
and same-input pool comparisons are recorded below. Review video:
`native-left-quality-incomplete.mp4`; plots: `round7-speed-and-clearance.png/.pdf`.

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
The 0.5 s limit applies when admitting a control step; the post-step video can
show age 0.6 s after that step advances simulation by 0.1 s. No new control is
admitted at age 0.6 s under these presets.
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

An additional offline diagnostic permits ages up to 1.0 s at this same terminal
state, without changing the live limit. Original frames 039 (age 0.7 s) and
040 (age 0.6 s) pass the complete current-state checks, with predicted corner
clearances 0.1089 and 0.1670 m. Older frames 036–038 fail clearance. This shows
that the chosen age cap contributes to this particular termination; it does
not establish live success with a longer limit. Evidence:
`terminal-longer-cache-diagnostic.json`. The quality comparison below retains
the 0.5 s cap so its setting change remains isolated.

## Earlier speed-quality resampling

The live approach offered only high-speed, body-admitted candidates in several
first pools. At 2.0 s their target speeds were 9.29 and 10.07 m/s; the existing
5 m/s preference ranks available candidates but does not trigger another pool
while any candidate passes.

Completed six frozen pools (48 samples) at 1.9 s. The baseline candidate arrays
match the live capture **exactly**. The unchanged selector chooses candidate 2
in pool 0: target 6.8594 m/s, predicted corner clearance 0.3114 m, score 1.324.
Pool 2 contains a better admitted candidate 4: target **5.407 m/s**, clearance
**0.479 m**, score **0.285**. It would never be requested by the old policy
because pool 0 already contains admitted samples. Evidence:
`replay-approach-speed-quality/`, `approach-speed-quality-audit.json` and
`speed-quality-replay-variants.json`.

Added optional `validation_resample_above_target_mps`, set to **6 m/s** in
`retained-native-four-left-quality-43.json`. With a visual green signal, retry
while the best admitted candidate still exceeds this target preference, up to
the existing pool budget. Compare admitted candidates using the existing
ranking (including any explicit longer-horizon preference). Store a cloned
native tensor and matching text from the best pool; later rejected or inferior
pools cannot overwrite it. Stop retrying once the best candidate meets the
preference. If no better candidate is found, use the best already-admitted
native candidate and explicitly record that the preference was unmet. This is
not a new speed cap, modified speed profile or requirement to discard a valid
path. Red/unknown and presets without this option retain their prior behavior.

Four new cases cover better later sampling, retaining the earlier exact array
and text when a later sample is worse or rejected, and unchanged red-stop
selection. Combined verification: **94 passed in 5.01 s**. The new live preset
changes only this preference relative to the completed retention run. The
comparison capture is `capture-retained-native-four-left-quality-43`.

### Quality-retry live result

The comparison ends at **8.3 s**, with **59.7846°** left rotation and no route
exit. Actual speed ranges **1.7594–6.4679 m/s**, final 2.7903 m/s, with no
near-stop below 0.25 m/s. All recorded vehicle corners remain inside the mapped
lane, but minimum clearance is only **0.0289 m**; this is fragile and incomplete.
It avoids the previous run's 9.5427 m/s approach peak, but different images and
diverging histories prevent a controlled whole-rollout causal claim.

There are **196 proposal calls** and **83 executed paths**: 76 newly sampled,
7 retained. Reuse occurs at 7.3 s (source 068, age 0.5 s), 7.6 s (source 075,
age 0.1 s), and 7.8–8.2 s (source 077, ages 0.1–0.5 s). All native-source,
array, world-anchor, speed-array and age checks pass, with zero control,
snapshot-position or frame mismatches. Seventeen image signal checks are green.
At 8.3 s all 48 fresh candidates fail and no unexpired cached plan remains.
The finish-turn phase has not activated: observed rotation is just below its
existing 60° threshold. Execution costs **2191.9 wall seconds (36.5 minutes)**.

`audit_quality.py` compares each fresh selection with the first admitted pool
on the same saved input. Quality retries spend **45 extra pools across 17
steps**, changing the selected native proposal on **15 steps**. Five steps
exhaust the budget with the preference unmet. Every selected array and generated
token sequence matches its original proposal file. These are same-input
selection comparisons, not simulated alternative whole-rollout outcomes.

Evidence: `native-left-quality-{control,footprint,route,retention,quality}-audit.json`
and video `native-left-quality-incomplete.mp4`.

The slowdown remains: at 7.0 and 7.1 s only one first-pool candidate passes,
targeting 1.6469 and 1.2669 m/s. The upper-speed retry rule accepts these. Later
the final source plan 077 approaches the hard 0.10 m predicted clearance limit.
An offline 1.0 s cache check at 8.3 s admits sources 075 (age 0.8 s, predicted
clearance 0.2239 m) and 077 (age 0.6 s, clearance 0.1598 m), but actual clearance
is just 0.0289 m. Evidence: `quality-terminal-longer-cache-diagnostic.json`.
Longer reuse has **not** been tested live. A roughly 2.1° actual heading lag
relative to source 077's nearest native pose can matter substantially at the
front of this long vehicle; small rear-axle separation alone is insufficient
evidence that the full body will track the predicted clearance.

### Frozen finish-turn instruction check

Replayed saved 7.6 and 8.3 s states with their baseline and three sampling pools
using the existing text, “Finish the left turn, then continue straight along
the exit lane.” Both baseline arrays reproduce exactly. All **48 finish-text
samples fail** the unchanged 1.5 s/controller-coverage, 0.10 m body margin,
and green-progress checks. Earlier phase text does not solve these two states;
no live phase threshold is changed. Artifacts: `replay-midturn-phase/`,
`midturn-phase-variants.json`, `midturn-phase-body-audit.json`.

### Frozen low-speed quality check

Replayed six pools (48 samples) at the earlier retention run's saved 4.5 s
state, where the admitted target is 2.0304 m/s. The baseline arrays reproduce
exactly. Pools 1–4 admit none; pool 5 admits one 1.945 m/s sample with a worse
ranking score. Thus these extra pools do not improve this particular slow
choice. No lower-speed retry rule is implemented on this evidence. Artifacts:
`replay-low-speed-quality/`, `low-speed-quality-audit.json`.

### Larger body-clearance margin comparison

At the quality run's final state, actual heading is 29.8529° while the nearest
source-077 native pose has heading 27.7433°. Rear-axle separation from that
vertex is 0.0742 m (polyline distance 0.0319 m), but native predicted corner
clearance at the vertex is 0.1697 m versus actual 0.0289 m. The front of this
vehicle extends about 3.83 m ahead of the rear axle, so a small heading error
can have a substantial footprint effect. This is a diagnostic comparison,
not a calibrated tracking-error bound.
The spatial comparison is saved in `cached-pose-tracking-comparison.json`.

Prepare `retained-native-four-left-quality-margin20-43.json`, changing only
the required predicted body clearance from **0.10 to 0.20 m**. It retains
the same 6 m/s retry preference, 0.5 s cache limit and original controller.
In the existing 6.8 s pool, the selected 3.045 m/s path has only 0.136 m
predicted clearance, while another native path at 2.564 m/s has 0.269 m.
The new requirement will change such selections or request more samples;
it will not alter any path. Capture extra mid-turn inputs at frames 65, 68,
70, 76 and 83 to make the next diagnostic reproducible. The comparison
`capture-retained-native-four-left-quality-margin20-43` ends at **5.3 s**,
with **5.2228°** left rotation and no route exit. Minimum speed is 2.7880 m/s,
maximum 6.0312 m/s and final 4.5696 m/s; no near-stop. Minimum actual corner
clearance is **0.2427 m**. All 53 executed native arrays, fixed anchors, speed
arrays and ages match; no control or snapshot mismatch. There are 131 proposal
calls, 11 green VQA checks and four reused steps (4.2, 4.5, 5.0, 5.1 s).
The three eligible cached plans at termination all fail remaining clearance;
all 48 fresh samples also fail admission. Wall time is 1374.7 s (22.9 minutes).
Larger margin does not complete this turn. Different rendered inputs and the
changed history prevent attributing the earlier endpoint solely to the margin.
Evidence: `native-left-margin20-{control,footprint,route,retention,quality}-audit.json`.
Retrospectively, 20 of the previous run's fresh selections have less than
0.20 m predicted clearance. At 6.6 s, already observed pools contain five
alternatives meeting 0.20 m, the best targeting 4.761 m/s. At 6.9 s and several
later steps, none of the already observed candidates meets it. This describes
sample availability only; a different earlier selection changes later inputs.
Evidence: `quality-retrospective-margin20-availability.json`.

## Proactive comparison of native plans

On the recorded 0.10 m quality run, 61 of its 76 fresh execution states also
have an older plan that passes the unchanged current-state cache checks and
has a better existing 1.5 s ranking score. At 6.7–7.1 s, source 066 remains
admitted with targets declining from 4.572 to 4.308 m/s and 0.195 m predicted
clearance. At 7.0 and 7.1 s, fresh choices instead target 1.647 and 1.267 m/s.
The exhaustion-only policy never considers this alternative because a fresh
sample passes. Evidence: `proactive-cache-retrospective-audit.json`. These are
recorded-state availability comparisons, not alternative live outcomes.

Added a separate `proactive_probe.py` entrypoint. It still samples and validates
fresh candidates, but then compares the accepted fresh plan with revalidated
recent native plans using the same existing score. A strictly better cached
plan may be selected before fresh sampling exhausts. The existing 0.5 s age,
green signal, phase, distance, heading, full-body and remaining-horizon checks
remain unchanged. There is no new switching penalty or path modification.
Longer-forecast preference is explicitly unsupported in this comparison so
the fresh and cached rankings remain comparable.

Every accepted fresh proposal is saved separately in `accepted_frame_*.json`,
even if it is not executed. Such a proposal can remain in the cache until its
original age expires. Root `frame_*.json` always describes the executed sample;
`retention.reason = cached_plan_ranked_better` distinguishes comparison from
fresh-sampling exhaustion. The fresh proposal's original arrays, text and
score are preserved, and no false rejection is logged. Controller records
point to the immutable source file. The retention audit verifies both source
identity and the preserved comparison; the quality audit explicitly identifies
its population as accepted fresh proposals, including superseded ones.

Three new tests cover a better cached choice, a better fresh choice and an
expired cache, including distinct native source files, associated text and
unchanged original age. Combined verification: **97 passed in 4.93 s**.
The next live comparison uses the same margin-20 preset via `proactive_probe`,
changing only the final fresh-versus-cached selection rule. Pending capture:
`capture-proactive-native-four-left-quality-margin20-43`.
