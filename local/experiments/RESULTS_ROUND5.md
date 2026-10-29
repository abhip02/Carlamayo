# Turn instruction investigation — round 5

Branch: `experiment/driving-turn-instructions`, starting checkpoint `6eca11d`.
Evidence root: `outputs/driving-investigation-5/`.
Previous results: [round 4](RESULTS_ROUND4.md).

This round tests the requested left turn and whether navigation wording,
navigation guidance, or a supplied intent improves the learned trajectories.
Production driving code, defaults, model weights and external model sources
remain unchanged. All changes are isolated experiment tools and documentation.

**Outcome:** neither left-turn test is a clean driving solution. Stronger native
navigation guidance can turn a straight prediction into a left prediction on
identical inputs, but does not remove the early stop or lane departure in the
live test. Keeping the left command active also contributes to an extra curling
prediction near the exit; updating it helps one saved state, but not a later bad
state. Completed work: **two live episodes (16 s and 12 s), 59 frozen inference
evaluations**, plus CPU/map audits. No experimental setting was promoted to the
normal launcher.

Watch `outputs/driving-investigation-5/left-turn-comparison.mp4` for both runs'
first 12 s, `left-native.mp4` for the full 16 s native run, or
`left-guidance15.mp4` for the full 12 s guided run. `left-turn-paths.png` shows
actual paths against the map. All videos use H.264 and include the front camera.

## Evaluation method

CARLA Town03 junction 103 has a left connection from the configured approach
lane: road 213, lane -1, ending at (21.2431, 134.2892), heading -0.8172 degrees.
The straight connection is road 184, lane -1. The left connector changes heading
by -90.4547 degrees in CARLA's coordinates (positive left in model coordinates).
`junction_routes.py` exports both lane centerlines, approach and exit extensions
from the running map. See `junction-routes.json`. Map geometry is used only to
evaluate recorded driving; it is not supplied to inference or the controller.

`audit_turn.py` measures the recorded vehicle actor center against the nearest
lane segment. Exit completion requires crossing the exit plane, reaching its
path arc distance, being within the lane half-width, and heading error at most
20 degrees. Report maximum/RMS center distance through first completion; a later
junction should not redefine the result of the requested turn. This is not a
full vehicle-body lane-containment or safety check. A turn angle alone does not
establish successful lane following.

Read-only exit-lane checks confirm a single outgoing lane at x=30 m, centered
near y=134.164 m, heading -0.817 degrees. The adjacent lane near y=130.664 m heads
in the opposite direction; y=137.8 m is not another driving lane. These queries
rule out treating the native test's outer exit departure as an alternate valid
lane (`exit-lane-check.json`). Inside a junction, overlapping map connectors make
nearest-driving-lane queries ambiguous; the specified connection is the reference.

Cross-checks on existing captures: the favorable round-2 straight run reaches
the straight exit at 9.9 s with maximum center deviation 0.7135 m, never the left
exit. The round-4 failed straight run does not reach either exit; its deviation
from the left route is 3.2678 m. An unwanted left curve is not automatically a
successful instructed left turn.

All new driving tests use one native model trajectory, fixed language seed 43,
independent motion seed 42, equidistant camera remapping, acknowledged pre-roll
and control delivery, snapshot waiting, and an ego-green signal layout. The
original controller is unchanged. The previous **straight-only candidate
selector is disabled**, as is the separate signal question. These settings are
experimental; this is not a claim about an unmodified default run.

## Frozen instruction comparison

Completed 21 evaluations on round-4 `capture-ack-green-43` inputs 000, 019, 045.
Each input is held identical across straight, left, no navigation, straight
repeat, supplied left intent, and four-sample straight/left variants. Full arrays
and raw generation are in `replay-instructions/`; summaries and plots are in
`instruction-comparison/`. Straight repeats match bitwise on all three inputs.

| Input | Native straight target, m/s | Native left target, m/s | Mean waypoint difference, m |
| --- | ---: | ---: | ---: |
| 000 | 2.236 | 2.254 | 0.0375 |
| 019 | 0.015 | 0.016 | 0.0028 |
| 045 | 8.123 | 7.698 | 2.504 |

Native straight, left and no-navigation variants generate the same straight
acceleration sentence at these three points. The arrays show almost no change
early, and mainly a speed change at frame 045: both native paths remain nearly
straight for the first sample. With four samples at frame 045, three left-prompt
candidates turn left, versus two turning candidates under the straight prompt
(one only partially). Sampling diversity matters: the first sample is not a
description of all possible outputs. Model-generated prose is not a faithful
explanation of internal causes. The terminal direction near 180 degrees on stopped frame 019 describes
predicted reverse motion, not a U-turn.

Supplying the completed assistant intent `Turn left at the intersection.` gives
a clearly turning frame-045 trajectory: endpoint 35.44 m forward / 27.56 m left,
terminal motion direction +86.75 degrees. It does not cure the near-stop input
019 (target 0.024 m/s). The supplied intent bypasses language-generated reasoning;
it is not the model's own perception or explanation. It changes conditioning,
not returned waypoints or controller commands. Frozen success at one frame is
not proof of a usable closed-loop turn.

## Stronger navigation guidance

Completed another 22 frozen evaluations: eleven variants each on frames 019 and
045. Same independent motion seed, one native sample, reference math attention
throughout. Using the math backend avoids the default-attention guidance failure
already recorded in round 1; comparisons include a matched math baseline. The
NVIDIA navigation CFG path is used unchanged, with weight 1.5 or 2.0. It amplifies
the motion prediction's navigation conditioning and does not replace generated
reasoning. Exact straight repeats match at both inputs.

| Frame 045 variant | Target, m/s | Endpoint forward / left, m | Terminal motion direction, degrees left |
| --- | ---: | ---: | ---: |
| Straight, weight 1.0 | 8.113 | 66.51 / -1.91 | -3.07 |
| Left, weight 1.0 | 7.679 | 60.18 / -1.14 | -0.63 |
| Left, weight 1.5 | 7.197 | 37.78 / 26.55 | 83.60 |
| Left, weight 2.0 | 7.048 | 34.43 / 26.73 | 89.49 |
| `Turn left in 10m`, weight 1.0 | 7.959 | 64.14 / -1.66 | -2.16 |
| Supplied completed left intent | 7.144 | 35.41 / 27.56 | 86.75 |

This is a positive **local** navigation response: changing guidance alone makes
the left instruction produce a left curve on this input, while stronger straight
guidance preserves a straight prediction. The generated acceleration sentence is
identical across these guidance variants, so the motion change does not depend
on a different generated explanation. The effect is not just a different random
noise draw. However, all eleven variants still request near-stop speeds on frame
019 (0.0117–0.0277 m/s). Shorter/distance-specific navigation text alone does not
repair either issue. Native CFG calls take about 8 seconds versus 4–6 seconds
for the matched math baseline on these saved inputs.

Full evidence: `replay-guidance/`, `guidance-comparison/`. The stronger-guidance
left rollout below is chosen for live follow-up because it retains native
language generation. `left-completed-43.json` remains a prepared diagnostic
option; it has **not** been run live in this round.

## Exit-instruction lifecycle

Twelve frozen calls compare left, straight, finish-turn/then-straight, no
navigation, supplied lane-following intent, and repeated left on the native
left run's frames 095 and 125. Native left exactly reproduces both the live
capture and repeated replay. This tests whether leaving the same left command
active after rotating contributes to excessive turning, without changing the
controller or simulator state.

At frame 095, the unchanged left command predicts a curling path ending 9.44 m
behind / 16.02 m left. Switching to straight predicts 39.02 m forward / 4.89 m
left, but that output **lacks the trajectory marker** and is not accepted as a
valid repair. Finish-turn/then-straight and no navigation also remove the curl,
with valid markers, but their predicted exit headings remain imperfect. The
supplied completed lane-following intent predicts 38.64 m forward / 6.82 m left.
Its first sampled rear-axle point beyond the exit plane is 1.66 m from the lane
center with 12.52-degree heading error. This is close to the 1.75 m lane
half-width and says nothing about full-body containment or actual future driving.

At the later frame 125, every variant still curves badly; endpoints remain
approximately 1–2 m forward / 11 m left. Updating the instruction is therefore
a local effect, not a general recovery mechanism. Endpoint motion direction
can flip by 180 degrees for almost identical paths when final motion reverses;
inspect full curves and signed velocities rather than heading alone.

Four final calls on frame 095 check the already existing format guard. The
unguarded straight failure repeats exactly. The guarded straight output has a
valid marker and ends 37.56 m forward / 15.60 m left; restoring the marker changes
the path and does **not** certify lane following. The guarded finish-turn variant
matches its already valid unguarded output. No instruction switching or guard
was introduced into normal driving, and these exit variants were not tested live.

Evidence: `replay-exit-instructions/`, `exit-instruction-comparison/`,
`exit-prediction-route-audit.json`, `replay-exit-marker/`, and
`exit-marker-comparison/`. The predicted-exit audit uses the original rear-axle
world transformation and reports the first sampled point past the map exit
plane, its nearest lane distance, and motion-heading error. It is distinct from
the live actor-center completion audit.

## Execution log

- `capture-left-native-43`: CARLA exited 139 during camera/pre-roll setup,
  before any model inference. Simulator crash state and logs are preserved;
  the waiting Python process was terminated and the owned container restarted.
- `capture-left-native-43-v2`: server-version RPC timed out during slow simulator
  startup, before loading the model. A later readiness check passed. These two
  starts are infrastructure failures, not failed driving episodes.
- `capture-left-native-43-v3`: 16 s native left instruction test. Completed
  analysis: the car does turn left, but never reaches the correct
  exit within the lane/heading criteria. Maximum left-route center deviation is
  5.2694 m (RMS 2.1544 m), final heading change +174.31 degrees, final speed
  0.2991 m/s. Near-stop onsets occur at 2.0, 10.4, 10.7, 14.0 and 15.8 s. The
  instruction remains active throughout; this is not a route manager that clears
  a completed maneuver. The car is already 3.93 m off the mapped turn centerline
  before reaching the exit, so repeated instruction alone does not explain away
  the initial lane-following failure. Duration reached normally after 160 calls;
  wall time 600 s. One generation lacks the trajectory marker at 9.7 s.
- `left-native-audit.json`: zero requested/reported control mismatches, exact
  snapshot/CSV poses and frames, and all 160 controller input arrays match the
  recorded model trajectory. See `left-native-route.json`, `left-native-route.png`
  and the browser-compatible video `left-native.mp4`.
- `left-native-plans.png/json` overlays raw predictions using the production
  rear-axle transformation. At 8.5 s the current rear axle is 0.3095 m from the
  left-route centerline, while its predicted next 1.5 s reaches 3.9119 m away.
  At 9.5 s the prediction curls back into the junction. Poor model plans clearly
  contribute; this is not solely an observed vehicle-vs-plan tracking discrepancy.
  These checks do not establish that controller tuning has no effect.
- `capture-left-guidance15-43`: 12 s native left instruction with guidance 1.5
  and math attention. Completes its requested duration after 120 calls in
  1009.3 wall seconds. Near-stop onsets are 1.8 and 2.0 s, both during the early
  braking episode; there are no later near-stops within this horizon. Final
  speed 1.5554 m/s; heading change +60.18 degrees. Maximum center deviation
  3.3908 m, final deviation 2.0961 m. The vehicle is still 9.6114 m of route arc
  short of the left exit at 12 s. A longer run would be needed to establish
  eventual completion; the stop and lane departure are already observed failures.
  One generation lacks the trajectory marker at 8.9 s. No collision is recorded.
- `left-guidance-audit.json`: all 120 controller input arrays match model output,
  with zero requested/reported command, snapshot or frame mismatches. Actual
  handover position, rotation and speed exactly match the native run
  (`left-initial-state-comparison.json`). Later closed-loop inputs differ.

Compare the **same first 12 seconds** rather than the full 16 s native run against
12 s guided. Both runs leave the turn lane. Smaller average deviation in the
guided run also comes with less progress; it is not proof of a better completed
maneuver (`left-common-12s-comparison.json`).

| First 12 seconds | Native left | Left guidance 1.5 + math |
| --- | ---: | ---: |
| Maximum center distance from left route, m | 3.9327 | 3.3908 |
| RMS center distance, m | 1.8036 | 1.2547 |
| Remaining route arc to exit, m | 1.3884 | 9.6114 |
| Final heading change left, degrees | 110.305 | 60.180 |
| Final speed, m/s | 2.0042 | 1.5554 |

The native and guided live settings differ in guidance and attention backend.
The frozen guidance comparison isolates weight with a matched math baseline;
do not attribute every live-run difference solely to guidance weight.

## Reproduction

Start the owned simulator with `bash local/server.sh start`, wait for a successful
CARLA RPC, then run sequentially (never overlap probes):

```bash
source local/env.sh
.venv/bin/python -m local.experiments.inference_probe capture \
  --config local/configs/demo.yaml --duration 16 --capture-frames 0 19 45 65 95 125 \
  --options local/experiments/presets/left-native-43.json \
  --output outputs/driving-investigation-5/my-left-native
.venv/bin/python -m local.experiments.inference_probe capture \
  --config local/configs/demo.yaml --duration 12 --capture-frames 0 19 45 65 95 \
  --options local/experiments/presets/left-guidance15-43.json \
  --output outputs/driving-investigation-5/my-left-guidance
bash local/server.sh stop
```

The native instruction replay uses `presets/turn-instruction-variants.json` with
round-4 `capture-ack-green-43/inputs/frame_{000,019,045}.pt`; the guidance replay
uses `presets/turn-guidance-variants.json` with inputs 019 and 045. Frozen inputs
are local artifacts, not repository files. Readiness must be checked before
capture after a restart; container-running alone was insufficient this round.
The exit replay uses `presets/exit-instruction-variants.json` and native-left
inputs 095/125; the marker follow-up uses `presets/exit-marker-variants.json`
and native-left input 095. Each replay has its own output directory to avoid
collisions between saved inputs with the same filename.

Exact probe source snapshots and manifest hashes are preserved with each run.
Outputs and recordings remain local and untracked. Return to the previous round
with `git switch experiment/driving-input-contract`; return to the original
VerifAI integration with `git switch feature/verifai-server-client`.

Existing controller, adapter, dataset and VerifAI regression suite: **63 passed
in 3.81 s**. No new environment packages or checkpoint changes were required.
All analysis entrypoints were exercised on recorded data; syntax, preset JSON,
source hashes and Git whitespace checks pass. The simulator was verified back
in asynchronous mode with zero sensors and zero owned ego vehicles, then the
owned container was stopped (`simulator-cleanup.json`). Production controller
and configuration still match upstream `4914558`; external model sources are
clean. Return to `6eca11d` / `experiment/driving-input-contract` to discard this
round's diagnostic additions. Outputs remain available locally after switching.
