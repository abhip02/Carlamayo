# Model-only validation and resampling — round 6

Branch: `experiment/driving-plan-validation`, starting checkpoint `5b94733`.
Evidence root: `outputs/driving-investigation-6/`.
Previous results: [round 5](RESULTS_ROUND5.md).

## Current status

Every executed path remains a native model sample. Normal launchers and the
production controller are unchanged. The current evidence is:

| Live run | Outcome | Minimum speed | Recorded box corners |
| --- | --- | ---: | --- |
| Front, straight, seed 43; earlier ranking | Exit at 7.9 s | 2.650 m/s | Inside, minimum clearance 0.068 m |
| Front, straight, seed 42; current ranking | Exit at 8.5 s | 2.695 m/s | Inside, minimum clearance 0.112 m |
| Four cameras, left, 8 samples × 3 pools | Ends at 4.5 s; no valid next path | 2.788 m/s | Inside until termination, minimum clearance 0.374 m |

The two straight successes use different language seeds **and ranking
revisions**; initial poses match but rendered images differ. They are not a
controlled A/B pair or proof of general robustness. The left turn is not fixed.
Red/mixed-light behavior and obstacles are not validated by these green tests.
The body audit checks recorded corners, not swept volumes or collisions.

Current straight video: `validated-front-straight-body-42.mp4`.
Incomplete left video: `validated-native-left-short-incomplete.mp4`.
Source/config manifests live beside each capture; all artifact paths below are
relative to the evidence root. Read the final sections for the latest retries.

The user explicitly chose to keep executed paths model-generated. There is no
map-based fallback planner, trajectory correction, forced cruise speed, or
production controller change. This round tests a map-assisted **selection**
layer: CARLA lane geometry is used to evaluate model samples, and the chosen
waypoints are an exact slice of the original model output. This is a different
policy from unmodified first-sample inference; the map is now selection input,
not just evaluation data as in round 5.

## Frozen candidate availability

Completed 25 inference calls, four samples each: **100 proposed trajectories**.
All use equidistant preprocessing, language seed 43 and the existing format
guard. Motion seeds 42, 43 and 44 vary the model's native sampling noise.

- Left: four variants on the native round-5 left run's frames 019, 045, 065,
  095. Three left-command noise pools plus finish-turn/then-straight with seed
  42. At approach frame 045, the original four samples all exceed the 3 s lane
  corridor; one candidate each under motion seeds 43 and 44 remains inside it
  (maximum rear-axle distances 1.660 m and 1.399 m). On frame 019, all targets
  are still low. At 065/095 the actual car is already far off-route and none
  passes; resampling is not a guaranteed recovery method.
- Straight: three noise pools on round-4 `capture-ack-green-43` frames 019,
  045, 057. At 019, the original first target is 0.015 m/s, but another valid
  sample requests 0.842 m/s. At 045, each pool contains valid straight samples.
  At the later bad state 057, none passes; best maximum lane deviation remains
  10.48–11.39 m over the next 3 s.

Evidence: `replay-left-pools/`, `left-pool-audit.json`,
`replay-straight-pools/`, `straight-pool-audit.json`.

## Implemented experimental wrapper

`validated_probe.py` wraps the existing capture process. The normal launcher
does not import it. `validated_selection.py` rejects missing trajectory markers,
nonfinite points, signed reverse velocity below -0.1 m/s, predicted rear-axle
positions outside the mapped lane half-width, or moving-path headings more
than 45 degrees from the route. The initial horizon is 3 s.

The lane checks cover the rear axle, not the full vehicle footprint, and there
is no obstacle model. They do not certify safety or general traffic behavior.
Tests use the existing isolated empty intersection.

Among admitted samples, the score combines lane deviation and progress/speed
agreement. A separate image-based signal question supplies red/yellow/green/
unknown; simulator signal states are logged but not read by the selector. Green
uses a 5 m/s preference, unknown uses the current speed, and red/yellow prefer
shorter sampled paths. This preference only ranks model samples and never
modifies their speed profiles or the controller. Red behavior still needs live
validation.

If no sample passes, request another pool of four using the next motion seed,
up to three pools. If an entire pool lacks markers, also vary the language seed.
If all pools fail, end the experiment explicitly and record every rejection.
Do not execute the least-bad invalid sample or substitute a fallback path.

Left instructions advance to finish-turn/then-straight after 60 degrees of
observed rotation, and to lane following after reaching the exit. A configured
exit position/heading check can end capture at the target exit, avoiding an
unrequested next maneuver or exhaustion of the finite map reference. This is
reported as `route_exit_reached`, and does not erase earlier lane/stopping errors.
All other errors remain errors.

Every proposal and raw candidate is saved under `proposals/`. Root `frame_*.json`
records the selected candidate, signal result, instruction phase, rejection
metrics, and original candidate array, so the executed path can be checked for
exact identity. Full input captures are under `proposals/inputs/`. After the
first live process started, failure logging was extended to save the full input
under `inputs/` when all pools fail; this logging-only addition applies to later
launches. The first process's exact earlier sources are preserved by manifest
hash in its `source_snapshot/`.

## Live execution log

- `capture-validated-left-43`: up to 16 s, or earlier explicit route completion/
  validation failure. Four samples, seed 43, initial motion seed 42, original
  controller, acknowledged initialization/controls, snapshot waiting, ego-green
  layout. Ends explicitly at 3.0 s after all 12 proposed paths leave the lane
  somewhere in the fixed 3 s future window. No near-stop occurs: minimum actual
  speed 1.1628 m/s, maximum actual center deviation 0.0427 m. This improves the
  early stop but is an incomplete rollout, not a driving fix. All 30 executed
  trajectories remain native samples. The exact failure input was not saved by
  this initial process; all 12 rejected arrays/metrics and preceding saved inputs
  are preserved. Later launches include the added full-input failure save.
- Inspecting the actual original-controller steering targets on those rejected
  paths places them 1.5–1.8 s ahead, at 4.555 m lookahead. All paths stay inside
  the lane through those targets (maxima 0.56–1.35 m), before drifting later.
  This motivates a separate adaptive-window test, not accepting the full 3 s
  plans as valid.
- The revised validator checks at least the 1.0–1.5 s speed-preview window and
  the exact original pure-pursuit steering target, including all preceding
  waypoints. It extends farther for slow/short trajectories where the 4 m
  minimum lookahead reaches later points. The nearest-point starting index is
  also logged. Lane-width, marker, reverse and heading rejection bounds stay
  the same. `left-v1-short-horizon-audit.json` preserves the re-evaluation.
- `capture-validated-left-short-43`: ends explicitly at 2.3 s with no admitted
  plan after 12 samples. Actual speed reaches 0.000639 m/s; the native decoder
  estimates -0.6607 m/s after braking. All proposals contain reverse motion and
  heading violations. The full failure input is saved at `inputs/frame_023.pt`.
  All 23 consumed trajectories are exact native samples, with zero control or
  snapshot mismatches. This test does not solve the early stop.

## Format-guard / VQA interaction

Found and fixed an experimental integration bug: the trajectory-format guard
also applied to the separate signal question, where it suppressed the VQA
answer-ending token. This produced 256-token contradictory answers, often
parsed as unknown. The guard now recognizes the VQA answer prefix and leaves
that protocol alone. It still guards trajectory generation.

`vqa_guard_check.py` compares the archived old probe, corrected probe and
unguarded generation on identical saved inputs and seed. On both initial and
failure images from `capture-validated-left-short-43`, the corrected and
unguarded full-camera answers are identical: three tokens rather than 256.
However, both say red despite the configured ego-green scene. Front-only says
none initially and red at the failure frame; front plus telephoto says red.
The protocol fix is verified, but signal perception remains unresolved. This
does not explain the original unmodified CarlaMayo failure or earlier VQA
experiments which did not enable this guard.

Evidence: `vqa-guard-check/results.json`, source hashes in `sources.json`, and
the exact archived old probe under the failed capture's `source_snapshot/`.
`signal-geometry.json` records a later read-only map query, **after** rollout
cleanup. Its signal states are not the states during those recorded images.

## Front-camera candidate availability

Fifteen frozen calls on round-2 `capture-front-only` frames 015, 022 and 038:
original seed-42 single sample; guarded seed-43 single sample; and three pools
of four with motion seeds 42–44. These use the original pinhole front camera
only, without the equidistant projection. There are 42 proposed trajectories.

At slowdown frame 022, single-sample targets are 0.677 and 0.484 m/s. The pools
include valid native targets up to 1.218, 1.321 and 1.534 m/s respectively. At
frames 015 and 038 all tested candidates pass the adaptive controller-coverage
checks. This establishes candidate availability, not live driving success.
Older inputs lack world poses; for this **audit only**, poses were joined from
the original ego CSV by exact simulation time and matching frame ID. Original
inputs and replay records are untouched; augmented records are separately
stored in `front-pools-with-csv-pose/` with the pose-source path.

Evidence: `replay-front-pools/`, `front-pool-audit.json`. The subsequent
`capture-validated-front-straight-43` uses corrected VQA, front-only native
pinhole input, and the launcher's default all-green empty scene. It is a
separate controlled comparison, not a claim of mixed-signal robustness.

## Completed front-camera live tests

- `capture-validated-front-straight-43` reaches the straight exit at **7.3 s**,
  with minimum actual speed **2.6232 m/s**, final speed 9.8465 m/s and no
  near-stops. All 73 selected paths and controller inputs exactly match original
  model samples; no control or snapshot mismatch. The first pool suffices on
  every frame, so this run tests sample selection, not live retry recovery.
  Image signal checks return green 13 times and unknown twice near the exit.
- Maximum actor-center lane distance is **1.2969 m**. A stricter audit using
  the actual Lincoln bounding box finds up to **0.4874 m of corner overhang**,
  beginning at 4.7 s. Therefore this is a completed crossing without the early
  stop, **not clean lane containment**. Geometry was queried read-only from the
  same vehicle blueprint in the subsequent live test, after waiting for a
  simulator snapshot; the initial query before a snapshot found no actor.
  Evidence: `front-straight-audit.json`, `front-straight-route.json`,
  `front-straight-body-audit.json`, `vehicle-bounds.json`, and video
  `validated-front-straight.mp4`.
- `capture-validated-front-left-43` ends explicitly at **4.2 s**: all 12
  candidates miss the left route, with predicted maximum rear-axle lane
  distances 1.82–3.51 m. Actual minimum speed is 2.6589 m/s and actual maximum
  center distance 0.6813 m; no near-stops, but the turn is not completed.
  All 42 executed arrays are native with zero control/snapshot mismatch.
  Evidence: `front-left-audit.json`, `front-left-route.json`, and full failure
  input `capture-validated-front-left-43/inputs/frame_042.pt`.

The body audit motivates an opt-in `validation_check_body` extension. It
uses the model's native predicted orientations and the recorded CARLA vehicle
bounds to project box corners through the same checked horizon. Candidate
waypoints are still unchanged. This is a lane-corner check, not an obstacle or
swept-volume guarantee. Sources, options and native orientation traces are now
saved automatically by new probe launches. The existing center-only presets
remain separate comparison configurations.

The first frozen front-camera guidance batch completed four calls on approach
frame 019, then native guidance weight 2 with four samples hit an illegal CUDA
access in the unguided VLM forward pass despite the math-attention context.
The process was allowed to exit and release its GPUs. Its partial results and
failure remain in `replay-front-left-guidance/`. A fresh-process retry tests
single-sample guidance (previously working in round 5), plus the remaining
language/prompt ablations. That retry completes three ordinary calls, then
fails at the first guided call with CUBLAS execution failure, again in the
unguided continuation forward. Both processes release their GPUs. These
runtime failures are not driving outcomes; the unexecuted ablations are not
counted as tests.

A CPU retrospective body check re-decodes the archived actions with the
cached decoder configuration solely to recover missing orientations. The
original predicted positions stay untouched. Its maximum CPU/GPU re-decode
position discrepancy is 0.0957 m, so it is approximate diagnostic evidence;
live body validation uses the actual returned native rotations instead.
Evidence and the exact analysis script: `front-straight-retrospective-body-validation.json`
and `audit_native_body.py`.

The initial pose, velocity histories and rotation histories of the two straight
captures match exactly, but rendered images do not. Initial model predictions
therefore differ (maximum candidate-coordinate difference 0.0880 m). This is
not a pixel-identical A/B comparison. `straight-initial-repeatability.json`
records the comparison; frozen-input tests remain the stronger causal check.

## Straight crossing with body validation

`capture-validated-front-straight-body-43` reaches the straight exit at **7.9 s**.
Minimum measured speed **2.6500 m/s**, final speed 4.6619 m/s, no near-stops.
Maximum centerline distance is **0.7567 m**. All recorded vehicle-box corners
remain inside the selected lane corridor: minimum corner clearance **0.0683 m**.
This is a narrow margin, and the recorded-corner check is not a swept-volume
or obstacle guarantee. No traffic is present.

All **79** selected trajectories and controller inputs exactly match native
model samples, with zero control/snapshot mismatch. At frames 070 and 072,
the first two pools fail the body checks; a native sample from pool 3 passes
and is executed. This demonstrates actual live retry recovery. The other 77
frames use the first pool. Signal questions return green 12 times and unknown
four times. Rendering differs from the earlier run, as documented above.

Evidence: `front-straight-body-control-audit.json`,
`front-straight-body-route.json`, `front-straight-body-footprint.json`, and
`validated-front-straight-body.mp4`. Repeat with
`presets/validated-front-straight-body-43.json` after exporting the configured
map reference as shown in the README. This successful straight run does not
resolve left turns, mixed signals, different initial states or repeatability.

## Further left-turn sampling

`replay-left-camera-pools/` completes 12 calls / 48 native trajectories on the
front-left run's approach frame 019 and failure frame 042. Variants: all four
original pinhole cameras; another language seed; another motion seed; front
plus telephoto; a more explicit left command; and front with trajectory-only
prompting. The latter still enables the format guard and does not demonstrate
faithful removal of model reasoning.

With actual native orientations and the same recorded vehicle dimensions,
all 24 early candidates pass body checks through 3 s, but none passes the full
6.4 s plan. At the late failure state none of the 24 passes even the adaptive
1.5 s horizon. Evidence: `left-camera-body-audit.json`. No variant from this
batch establishes left-turn recovery.

Added opt-in `validation_resample_language`: after a rejected pool, increment
both the language seed and motion seed. Previously only motion noise changed
unless every trajectory marker was absent. The first pool stays the same.
An additional regression confirms a second-pool recovery returns the exact
candidate and records both changed seeds. All 11 focused tests pass.

`capture-validated-four-left-body-43` uses all four cameras with
the existing equidistant rendering experiment, default all-green empty scene,
body checks, 1.5 s minimum controller-coverage horizon, and this broader retry.
It is a new camera configuration relative to the front-only run; defaults in
normal CarlaMayo are unchanged. It terminates at **2.1 s** after near-stopping
(actual speed 0.00862 m/s); all 12 retries contain reverse motion. All five
image signal checks say green, and recorded simulator signals are also green.
All 21 executed arrays remain native, with no control/snapshot mismatch and
no recorded corner overhang. Audits: `four-left-body-control-audit.json` and
`four-left-body-footprint.json`.

## Correcting the experimental ranking horizon

The adaptive admission horizon introduced a **ranking bug**: progress error
was evaluated over a different duration for each candidate. A plan that stopped
now and moved later could beat a steadier plan because its checked horizon was
shorter. This is a flaw in the experimental selector, not a diagnosis of the
original CarlaMayo model.

On the exact saved pool at full-camera frame 019, the old selector chooses
candidate 2, target **0.1805 m/s** (5.6 s admission horizon), over candidate 1,
target **1.0530 m/s** (6.4 s admission horizon). The new ranking compares lane
RMS and progress over the same **1.5 s** for every admitted sample. Admission
still extends through the actual controller lookahead and checks native body
corners. With identical arrays, new ranking chooses candidate 1. Evidence:
`ranking-horizon-audit.json`; the regression covers unequal steering horizons.

An optional `validation_green_min_target_mps: 1.0` adds a simple progress
requirement for this **empty green-light test**. Low-progress samples trigger
resampling; paths are never sped up or repaired. Red and unknown signals do
not enable this requirement. It is not an obstacle-aware rule for arbitrary
roads. Exhausting the sampling budget still ends the test explicitly.

`replay-left-progress-pools/` completes six calls / 24 samples at frame 019:
five language/motion seed pairs and one explicitly supplied green-intent
prompt diagnostic. The initial pool already contains the valid 1.053 m/s
candidate. None of the four additional ordinary pools has a body-valid target
above 1 m/s. The supplied-intent diagnostic has two (1.258 and 1.394 m/s), but
this is supplied conditioning, not a model-generated explanation, and is not
used in the next live test. Evidence: `left-progress-audit.json`.

`capture-validated-four-left-progress-43` tests corrected ranking plus the
1 m/s requirement, keeping full-camera equidistant input, native sampling,
body validation, three pools and varied language on rejection. It ends explicitly at **2.3 s**: all 12 samples fail the 1 m/s
progress requirement (maximum target 0.914 m/s). Actual speed stays at least
**1.4620 m/s**, so it avoids the prior full stop but still cannot continue the
turn. All 23 controller inputs are native and all controls/snapshots match.
Evidence: `four-left-progress-control-audit.json` and the saved failure input.
All 13 focused regressions pass before launch; the combined suite subsequently
passes **76 tests in 4.37 s**.

The next controlled left test, `capture-validated-native-four-left-progress-43`,
uses the **original four pinhole cameras** instead of the equidistant rendering
experiment, plus a **3 s** minimum admission horizon. The longer horizon is
intended to reject a missed turn before the vehicle arrives too fast. Ranking
still uses the same 1.5 s interval for every candidate. It retains body checks,
the green-progress requirement and three native pools. It ends at **3.8 s**
when all 12 proposals fail predicted lane/body checks. Actual minimum speed
is **1.3441 m/s**, final speed 4.7756 m/s, no near-stop, maximum center
deviation 0.1231 m, and minimum recorded corner clearance 0.6581 m. All 38
executed arrays are native, with exact control/snapshot matching. Evidence:
`native-four-left-progress-control-audit.json` and
`native-four-left-progress-footprint.json`.

## Larger native pools at the turn failure

`replay-left-expanded-pools/` uses the exact native-four-camera failure input
at frame 038. Six successful calls produce 44 native proposals: 4, 8 and 12
ordinary samples; 8 each at native motion-noise temperatures 1.5 and 2; and
4 with an explicitly supplied green/left intent. CUDA execution is synchronous
for this diagnostic (`CUDA_LAUNCH_BLOCKING=1`), recorded separately in
`left-expanded-pools-environment.json`. No model source or weights are changed.

- Four ordinary samples: none passes the same 3 s body and progress checks.
- Eight or twelve ordinary samples: candidate 5 passes, target about 7.50 m/s,
  predicted corner clearance 0.096 / 0.102 m respectively.
- Temperature 1.5, eight samples: candidate 5 passes, but only 0.003 m of
  predicted corner clearance.
- Temperature 2, eight samples: candidate 4 passes, target **5.694 m/s**,
  predicted corner clearance **0.298 m**.
- Supplied green/left intent: none passes the body checks.

A subsequent four-sample guided call exhausts GPU memory; the next guided
variant is explicitly skipped. Maximum allocation for ordinary twelve samples
is 20.33 / 14.52 GiB across the two model GPUs. These trials demonstrate native
candidate availability at a failed state, not a completed turn.

`replay-left-single-guidance-sync/` then completes three memory-bounded native
guidance calls at weights 1.5, 2 and 3 with one sample and synchronous CUDA.
All three still fail body/lane checks; stronger guidance increases their worst
centerline error from 2.698 to 3.192 m. This successful execution does not prove
the earlier asynchronous failures' cause, since input/camera/batch settings
also differ. Evidence: `left-expanded-body-audit.json` and
`left-guidance-sync-body-audit.json`.

The next live configuration uses eight samples per pool, with noise
temperatures 1, 1.5 and 2 across rejected pools. Language and motion seeds also
vary as before. The first pool uses ordinary native sampling, and all admission
bounds remain unchanged. `capture-validated-native-four-left-expanded-43` is
ends explicitly at **2.2 s**: all 24 samples fail the 3 s admission checks.
Minimum actual speed is **2.7857 m/s**, final speed 5.9106 m/s, no near-stop,
maximum center deviation 0.0844 m, and all 22 executed arrays/controls/snapshots
match. The shorter adaptive 1.5 s/controller-coverage audit admits two samples
from its final high-noise pool, but none from the first two pools; this does
not make their full 3 s forecasts valid. Evidence:
`native-four-left-expanded-control-audit.json` and
`expanded-left-short-coverage-audit.json`. The retry regression also checks
the native temperature sequence; all 13 focused tests pass.

## Turn-distance diagnostic

The native helper documents navigation instructions with a distance. The next
frozen batch tests the expanded-pool failure frame 022 with the same eight
samples, varying only navigation text: no distance, distance to junction entry,
distance to the route's first 5-degree heading change, distance to its first
30-degree heading change, and an explicit straight-then-left instruction.
Distances are derived from map route arc length and recorded in
`left-distance-geometry.json`; this supplies navigation text, never map-generated
executed waypoints. All five calls / 40 native samples complete, but none
passes the unchanged 3 s body checks. Distances are 10.671 m to entry, 15.844 m
to the first 5-degree heading change, and 20.640 m to the first 30-degree change
(rounded to 11, 16 and 21 in text). Best rear-axle errors remain 1.370–1.441 m,
with body overhang. Evidence: `left-distance-body-audit.json`. No dynamic
distance-prompt feature is implemented on this evidence.

## Current-code straight verification

`capture-validated-front-straight-body-42` repeats the front-only body-checked
straight test with **language seed 42**, motion seed 42 and the corrected
fixed-window ranking. It keeps four samples, three noise-seed retries, the
1.5 s controller-coverage horizon and default all-green scene. It does not use
the added green-progress floor or temperature escalation. It reaches the
straight exit at **8.5 s**, with minimum speed **2.6945 m/s**, final speed
4.1157 m/s and no near-stop. Maximum centerline distance is **0.7016 m**;
all recorded box corners stay inside, with minimum clearance **0.1121 m**.
All 85 selected arrays and controller inputs remain exact native candidates,
with zero control/snapshot mismatch. Evidence:
`front-straight-body-42-control-audit.json`, `front-straight-body-42-route.json`,
`front-straight-body-42-footprint.json`, and video
`validated-front-straight-body-42.mp4`.

The sampling audit records **87 proposal calls for 85 executed steps** in the
seed-42 crossing. The first candidate of the first pool fails body validation
on 11 frames; frames 4.6 and 4.9 s require a second pool before proceeding.
The previous seed-43 crossing needs third pools at 7.0 and 7.2 s. Therefore,
these successes involve actual sample rejection/resampling, not simply accepting
the original first sample. Frozen/native identity audits establish exact
selection, but do not claim what an independent first-sample live run would do.
Evidence: `validation-sampling-summary.json`.

## Left test with controller-coverage horizon

`capture-validated-native-four-left-expanded-short-43` repeats eight samples
with the 1, 1.5, 2 noise sequence, original four pinhole cameras and progress
requirement, using the same **1.5 s minimum/controller-target coverage** as the
successful straight tests. Body, lane, direction and marker bounds are unchanged;
this tests a shorter forecast window, not validity of the rejected 3 s plans.
Ends explicitly at **4.5 s**, after all 24 proposals fail the unchanged body
checks. Minimum actual speed is **2.7880 m/s**, final speed 6.2122 m/s, and no
near-stop occurs. All recorded box corners stay in the lane, with minimum
clearance **0.3740 m**. All 45 executed trajectories and controller inputs are
exact native candidates, with zero control/snapshot mismatch. The car has
only started the turn; this remains incomplete. Evidence:
`native-four-left-expanded-short-control-audit.json`,
`native-four-left-expanded-short-footprint.json`, and the full failure input
`capture-validated-native-four-left-expanded-short-43/inputs/frame_045.pt`.

## Frozen turn-entry alternatives

At that exact 4.5 s input, five calls / 40 samples compare the original left
instruction, holding the current lane, following the current lane, and native
motion-noise temperatures 0.3 and 0.5. All fail the same 1.5 s/controller-coverage
body checks. The best original candidate's predicted corner overhang is
**0.1239 m**, versus 0.1255/0.1270 m for the lane-following texts and
0.1992/0.1771 m for reduced noise. All eight original rear-axle prefixes remain
inside the lane; the vehicle-box checks expose the tighter entry.

The route instruction is present inside the actual prompt's native route tags.
The eight generated intent texts are identical across these navigation wording
changes and include straight, left and right intentions. The paths differ only
slightly here. This is weak local instruction response, not evidence that the
adapter dropped the instruction, and generated prose does not establish the
model's internal cause. No held-lane retry policy is added on this evidence.
Artifacts: `left-entry-alternatives.json`, `replay-left-entry-alternatives/`,
`left-entry-alternatives-body-audit.json`, and `left-entry-failure-paths.png`.

A follow-up batch tests 96 proposals: four new independent language/motion-seed
pairs, three broader-noise pairs, front-only, front plus telephoto, warmer
language sampling, route repeated in user text, and trajectory-only prompting.
The first eight calls complete (64 trajectories), then the diagnostic stops
because its front/telephoto variant mistakenly requests camera ID 3; the actual
telephoto ID is 6. The error is preserved. The corrected front/telephoto call
and remaining three calls complete separately (32 trajectories) under
`replay-left-entry-diversity-rest/`; no omitted call is counted as a result.

At this exact input, the four new ordinary-noise pools have no valid sample.
Each of the three temperature-2 pools has at least one: five admitted samples
in total, with target speeds 1.111–6.428 m/s. Front-only admits one sample,
target 5.850 m/s, predicted minimum corner clearance 0.295 m. Front/telephoto,
warmer text, repeated user-text route and trajectory-only prompting admit none.
This supports trying broader native motion noise, not relaxing the bounds.
These are frozen prefixes, not completed turns. Evidence:
`left-entry-diversity-body-audit-partial.json` (all eight completed original calls
plus the explicitly recorded camera-ID error) and
`left-entry-diversity-rest-body-audit.json`.

Before increasing the live retry limit, `left-entry-reserve.json` tests the
exact next three pools in the existing schedule: language seeds 46–48, motion
seeds 45–47, temperature 2, eight samples each. The first three live pools will
be unchanged. An early reserve launch was rejected by the process lock while
the last diagnostic was still exiting; no overlapping model load occurred.
The rejected-launch log and completed-launch log are kept separately.

## Live six-pool retry check

`capture-validated-native-four-left-reserve-43` repeats the full-camera,
eight-sample left test with only `validation_max_pools` increased from 3 to 6.
The first three pool settings remain unchanged; later pools use the existing
language/motion seed progression and final temperature 2. No wrapper code or
selection bound changes are needed. The frozen exact-schedule test finds one
admitted candidate in pool 3 and one in pool 4 (zero-based), none in pool 5.
Evidence: `left-entry-reserve-body-audit.json`. Live result pending.

## Longer-forecast selection experiment

A CPU audit of the completed 4.5 s left run finds five frames where the selected
sample passes the required controller-coverage window but fails a 3 s body
check, while another sample in the **same pool** passes 3 s. At 1.5, 2.6, 3.0
and 3.1 s the longer-valid alternative has a similar target speed; at 1.8 s it
is faster (7.928 versus 6.424 m/s). Evidence:
`left-longer-horizon-selection-opportunities.json`. This identifies a selection
opportunity, not proof that the alternative would complete a live turn.

Added opt-in `validation_prefer_horizon_s`. On image-reported green, first
apply the unchanged controller-coverage admission checks and optional progress
floor. If any admitted sample also passes the longer forecast, choose among
those using the existing fixed-window score. Otherwise, use the ordinarily
admitted samples. This does not request a map path, relax an admission bound,
change a native speed, or apply the new preference on red/yellow/unknown.
Each sample logs the additional horizon and its rejection reasons. Normal
launchers and the tested straight preset do not enable the option.

Three regression cases verify selection of the exact longer-valid native array,
continued use of normally admitted samples when all fail the longer forecast,
and preservation of red-stop ranking. All 16 focused cases pass in 2.84 s; the combined controller, adapter, dataset,
VerifAI and selector suites pass **79 tests in 4.27 s**.
The running six-pool full-camera capture was launched **before** this edit and
uses its archived sources without the new option. A prepared front-camera
comparison enables the 3 s preference, eight samples and six pools; it changes
both camera input and selection policy and is not a controlled attribution test.
Live result pending under `capture-validated-front-left-forecast-43`.

## Upstream curvature inspection

Read upstream model commit `7a8f1c7` without modifying it. Its stabilization
changes curvature labels in `traj_to_action` and the default curvature bound.
The driving decoder `action_to_traj` does not call that estimator or apply those
bounds; its initial-state estimator returns velocity only. This diff does not
establish a cause for the observed turn-entry bias. No upstream model patch or
rollback is applied.

## Verification and rollback

Five focused tests cover rejection of wrong-lane/reverse/malformed plans,
green versus red ranking, exact original tensor/associated-text selection,
explicit failure without a fallback, and correct exit recognition. Combined
with existing controller, adapter, dataset and VerifAI tests: **68 passed in
3.37 s** before the failure-input logging addition. Follow-up verification will
be recorded with final results. After the logger addition all five focused tests
pass again (2.33 s); after adding controller-target coverage, all six focused
tests pass (2.45 s). The new test rejects a path that is initially in-lane but
leaves it before reaching the controller's actual distant steering target.

After the VQA protocol fix, all seven focused tests plus the existing suites
pass: **70 passed in 4.13 s**. The added regression verifies that VQA can emit
its answer ending while driving still has to emit its trajectory marker.

After body checks and their CARLA geometry regressions, **73 tests passed in
4.60 s**. The later retry regression brings the focused suite to 11 passing
tests. After fixed-window ranking and optional green-progress coverage, the
final suite contains 13 focused cases. Combined verification: **76 passed in
4.56 s**, covering controller, adapters, dataset, VerifAI and the experimental
selector. No normal driving source changed in this round.

Production defaults, controller, VerifAI launchers, model weights and external
model sources remain unchanged. Return to `experiment/driving-turn-instructions`
or checkpoint `5b94733` to discard this round after saving it. Outputs stay local.
