# Ten distinct driving scenes — round 19

Storage update (2026-11-04): [cleanup inventory and replay limits](STORAGE_CLEANUP_20261104.md).
All successful and failed videos, predictions, commands and audits below remain.
All accepted captures retain their saved input tensors; older/nonselected input
tensors were removed to reclaim space. The historical results below are unchanged.

Branch `experiment/ten-driving-scenes`; rollback `experiment/projected-speed-selection`
at `0b044ac`. The user requests ten videos, one start per scenario, including
straight/left/right at the existing intersection, different road layouts, and a
red-light or stop-sign case. This suite uses a red light.

**Completed: ten accepted videos.** [Open the collection](../../outputs/driving-investigation-19-showcase/VIDEOS.md).
The final profile uses gain 1.40; the red case adds a model-visual stop-hold guard.
The initial settings, failures and adjustments are retained below.

## Declared cases and criteria

| ID | Scenario | Duration cap |
| --- | --- | --- |
| 01 | Original junction, straight | 15 s |
| 02 | Original junction, left | 15 s |
| 03 | Original junction, right, from its legal approach lane | 15 s |
| 04 | Curved road, left bend | 22 s |
| 05 | Curved road, right bend | 22 s |
| 06 | Roundabout, first exit | 30 s |
| 07 | Roundabout, second exit | 40 s |
| 08 | Tight curved road | 28 s |
| 09 | Elevated T-junction, right turn | 22 s |
| 10 | Original junction, red-light stop | 12 s |

All are in Town03 with no other vehicles or pedestrians. Moving cases set all
simulator lights green. The red case sets them red and records the actual CARLA
stop waypoint on the ego approach, 1.87 m before the junction entry. One initial
condition per case; no offset repeats. The exact road/lane/s anchors, instructions,
seeds, poses and duration caps are in `presets/ten-driving-scenes.json`.

The initial trial retains the round-18 target ceiling (4.5 m/s), steering gain
(1.18), six single-candidate maximum, 3–5 m/s preference, and original-plan reuse
up to 3 s. The accepted profile changes the gain to 1.40 as documented below.
All nine moving cases require the declared endpoint, minimum measured speed >=1 m/s,
maximum <=5.5 m/s, no recorded collision or corner departure, and passing
source/world-anchor/time/controller audits. Map paths use connected forward CARLA
lanes; no lateral lane changes or geometric jumps are invented by the exporter.

The red case executes one fresh native prediction per step, with no selection
for motion and no cached substitute. The same steering projection and speed
ceiling apply. It must keep its entire front footprint before the stop line,
finish within 8 m of it, and hold speed <=0.2 m/s for the final 2 s. The normal
moving-case minimum-speed/endpoint criteria do not apply to this stopping task.
Simulator red state configures/checks the scene; it does not inject braking or
replace the model's prediction. Visual signal answers remain recorded. The final
red profile also holds an achieved native stop using the model-perceived signal,
as documented in the isolated follow-up below.

All cases check live scene scope, including curved roads without nearby traffic
lights. Absence of a visible light is not relabeled green. Every candidate and
failed run is retained. The video collection identifies retries
and the implementation used; a failed first attempt is not erased.

## Implementation and preflight

- `scene_routes.py`: forward lane route search and live empty-scene checks.
- `projected_probe.py`: right-turn and explicit route instructions; original
  prediction anchors/times remain intact.
- `stop_mission.py`: independent front-footprint stop-line and hold checks.
- `run_projection_suite.py`: optional manifest, frozen sources/manifest, and
  optional stop after a failed capture.
- `render_scene_suite.py`: annotated H.264 videos, measured plots and video index.
- `local/run_scene_suite.sh`: opt-in ten-case launcher; original launchers retain
  their existing behavior.

All ten routes pass map preflight: minimum lane width 3.5 m, maximum adjacent
sample distance <0.61 m. The new right-turn instruction, route connectivity,
red-stop sampling/hold rules and scene-scope checks pass with the existing tests:
268 tests pass in 5.40 s. Simulator/model backend files are unchanged.

Run the final settings (choose available GPU UUIDs first; this launcher starts/stops its owned server):

```bash
export CARLAMAYO_MODEL_GPUS=GPU-ce1bea0e-c85a-f48a-a1ad-5c2bc153e10b
export CARLAMAYO_RENDER_GPU=GPU-3f4e4372-21d9-bac9-f577-755a624eb99d
export CARLAMAYO_PROJECTION_CONTAINER=apomalap-carlamayo-ten-scenes
bash local/run_scene_suite.sh --output outputs/ten-scenes-new \
  --stop-on-failure --cases 3 10 4 6 7 8 9 5 1 2
```

Use a new output directory for every attempt. The initial order exercises newly
added maneuvers first. Rendering uses a fresh owned container with the same pinned
CARLA 0.9.16 image. Other users' GPU jobs remain untouched; hardware availability
can change between attempts and each allocation is recorded.

Final outcome: **10 accepted videos**, with every declared driving and execution check passing. See the final collection table below; earlier trials are retained.

## First right-turn result and isolated follow-up

Launch `51d41d1`, `outputs/driving-investigation-19-ten/`: the right turn reaches
its endpoint at 6.3 s, with speed 2.710–4.620 m/s and all 63 execution checks
passing. It **fails driving geometry**: the worst recorded corner is 0.195 m
outside the corridor during the late turn/exit. There are 98 trajectory samples and 17
plan reuses. The suite stops after this failure; the other nine cases have not
run at this checkpoint. The failed video and complete audit are retained.

At 5.4–5.8 s, actor heading lags the local route by 16.6–18.0 degrees while speed
rises from 4.43 to 4.58 m/s. Rear-axle motion gives an effective steering response
ratio median 0.853 (10th–90th percentiles 0.840–0.869), consistent with previous
calibration. The issue is tight-turn tracking/body clearance rather than a new
unexplained steering actuator mismatch.

Predeclared isolated follow-up: repeat case 03 fresh with only steering gain
increased from 1.18 to **1.30**. This is empirical tracking compensation, not a
claim that the measured actuator ratio changed. Keep the same speed ceiling,
plan selection/reuse, initial state, seeds, route and 15 s cap. Evidence:
`outputs/driving-investigation-19-right-gain130/`.

## Gain 1.30 result and clearance follow-up

The fresh right-turn repeat with gain 1.30 formally passes: exit 6.2 s, speed
2.736–4.628 m/s, all 62 execution checks pass, and minimum corner clearance
**+0.0021 m**. It uses 89 calls and 19 plan reuses. This removes the recorded
departure but leaves only about 2 mm margin, which is too marginal for the
requested demonstration set. The pass remains recorded; it is not described as
robust clearance.

Predeclare a further gain **1.40** trial, with all other profile/case settings
unchanged, starting again with right turn and then the rest of the ten cases.
The gain is tracking compensation around the same map target and existing
actuator/rate limits. The original criterion (no recorded corner departure)
is unchanged, while a visible positive margin is preferred for the showcase.
Output: `outputs/driving-investigation-19-ten-gain140/`. All cases in this attempt
will be fresh; no earlier right-turn pass is substituted into it.

## Gain 1.40: right turn accepted

The fresh gain-1.40 right turn passes at **6.2 s**, speed **2.726–4.622 m/s**,
with minimum recorded corner clearance **+0.168 m**. All 62 execution audits pass;
87 calls, 19 reuses, maximum reuse age 0.8 s. This is the first accepted video
in the new ten-case attempt. The other nine are queued under the same gain;
the red-light case runs next. No gain is changed between cases in this attempt.

[Accepted right-turn video](../../outputs/driving-investigation-19-ten-gain140/03-intersection-right-route-targets.mp4).
Earlier gain-1.18 failure and gain-1.30 marginal pass remain in their original
folders. These fresh runs are not bit-for-bit matched model-input replays.


## Red-light failure: native stop followed by the launch floor

The gain-1.40 batch stops after red fails (12 s, 120 single samples, no reuse,
all execution integrity checks pass). At 2.0–2.1 s, the native controller plans a
stop and brakes. At 2.2 s the car has stopped (0.006 m/s), but the upstream launch
floor raises a raw 0.818 m/s preview to 2 m/s. At 2.4 s it raises even a raw
0.011 m/s preview to 2 m/s. The visual answer remains red. The car restarts and
subsequent native targets also rise. It ends 16.824 m past the front-footprint
stop line, moving 2.112 m/s. This is a behavioral failure, not an audit failure.
The failed video and all original predictions are preserved in
`outputs/driving-investigation-19-ten-gain140/10-red-light-stop-route/`.

Predeclare a scoped controller-assisted stop-hold follow-up: only the red case
sets `hold_visual_stop`. After a native planned stop, measured speed <=0.2 m/s
and a model-perceived red/yellow latch a zero executed speed target, preventing
the upstream launch rule from restarting the car. Unknown does not release;
only an explicit fresh model-perceived green releases. Simulator signal truth
is used for scene setup/audit, never to drive this latch. Native arrays, anchors,
times, seeds, initial condition, 12 s cap and pass criteria remain unchanged.
This demonstrates assisted stop-and-hold, not native prediction alone. The
actual navigation instruction remains "Continue straight through the intersection."
(the earlier configured text was already overridden by route instruction).

The accepted right turn remains from the prior batch. Final collection will
identify the sources of accepted videos and all retries; it will not be
presented as a single untouched ten-case benchmark.

Validation: **274 tests pass in 4.56 s**. The new tests cover launch suppression,
raw-array preservation, unknown/stale-green hold, fresh-green release, and
no activation without a native stop. Follow-up output:
`outputs/driving-investigation-19-stop-hold/`, red first, then the eight remaining
moving cases at unchanged gain 1.40, stopping on any failed case.


## Red-light stop-hold result

Fresh run `36d8b37` in `outputs/driving-investigation-19-stop-hold/` passes the
unchanged red mission for all 12 s. The latch activates at 2.2 s after the native
planned stop. Minimum front-footprint gap is 4.722 m; final gap 4.734 m, final
speed approximately zero. No recorded collision/corner departure; minimum
corner clearance 0.828 m. All 120 execution audits pass. There are 120 native
samples, no resampling/reuse, 284.23 s inference and 342.83 s capture wall time.
This is a controller-assisted held stop; stopping accuracy closer to the line
and live red-to-green release are not demonstrated by this fixed-red clip.
Fresh-green release is covered by a unit test.

The eight remaining moving scenarios now run with guard disabled and gain 1.40.


## First curved-road result

Case04 (left bend) passes fresh at 13.3 s: speed 2.667–4.609 m/s,
minimum corner clearance +0.647 m, all 133 execution checks pass. There are
173 trajectory samples and 53 plan reuses (maximum age 2.0 s); seven decisions miss the
preferred speed band but remain above the declared moving threshold. Inference
464.55 s, capture wall 530.03 s. No per-case tuning was applied. Source:
`outputs/driving-investigation-19-stop-hold/04-curve-left-route/`.


## First roundabout exit

Case06 passes fresh at 17.6 s: speed 2.632–4.603 m/s, minimum corner
clearance +0.459 m. All 176 execution checks pass. The car takes the declared
first exit, with 230 trajectory samples and 30 reuses (max age 1.0 s); ten decisions
miss the preferred speed band. Inference 586.47 s, capture wall 676.24 s.
The initial approach repeatedly uses the full six-sample budget for predictions
just below the 3 m/s preference, before sampling settles to one per step. This
is an observed efficiency cost, not a new change to the policy. Source:
`outputs/driving-investigation-19-stop-hold/06-round-first-route/`.


## Second roundabout exit

Case07 passes fresh at **28.7 s**: speed 2.602–4.561 m/s, minimum corner
clearance +0.594 m. All 287 execution checks pass. It continues past the first
exit and reaches the declared second-exit endpoint. There are 380 calls and
142 plan reuses (max age 2.1 s); 18 decisions miss the preferred speed band.
Maximum native-target projection distance is 0.411 m. Inference 1044.60 s,
capture wall 1194.59 s. The same fixed profile was used throughout; no settings
were changed to obtain this result. Source:
`outputs/driving-investigation-19-stop-hold/07-round-second-route/`.


## Sampling-cost observation during the tight bend (run still active)

At 11.2 s, the tight-bend run remains within the lane, speed 2.943 m/s.
Six fresh raw targets span 2.451–2.696 m/s; a still-valid 2.5 s old prediction
supplies 2.892 m/s. None meets the fixed 3–5 m/s preference, so every decision
uses all six samples even though these are plausible moving speeds for a tight
curve. This is an efficiency limitation of the fixed preference. The profile
is kept unchanged for this run; a later experiment could test a more tolerant
or curvature-aware preference. That has not been implemented here.

Reported sample counts count trajectory generations; separate visual signal
questions are not additional entries in that counter. Their elapsed time is
included in the measured inference timing.


## Tight-bend result

Case08 passes fresh at **20.4 s**: speed 2.438–4.624 m/s, minimum corner
clearance +0.438 m. All 204 execution checks pass. There are **759 trajectory
samples**, 143 reuses (max age 3.0 s), and 111 missed speed-band preferences.
Inference 1871.31 s; capture wall **2063.52 s (34.4 min)**. The recorded car keeps
moving and follows the tight curve, but this is an expensive case to generate.
The model's saved explanation describes adjusting speed for the left curve;
this is consistent with the observed lower targets, not proof of its internal
cause. All raw candidates and commands remain available. No thresholds,
endpoints or sampling settings were changed during the run. Source:
`outputs/driving-investigation-19-stop-hold/08-tight-bend-route/`.


## Elevated T-junction result

Case09 passes fresh at **11.2 s**: speed 2.605–4.604 m/s, minimum recorded
corner clearance **+0.091 m**. All 112 execution checks pass; 187 trajectory
samples, ten reuses (max age 0.4 s), 15 speed-preference misses. Inference
471.14 s; capture wall 536.72 s. This is the narrowest accepted margin so far:
it is a recorded containment pass, not generous clearance or evidence of
robustness to perturbations. The same gain 1.40 and 4.5 m/s ceiling are retained.
Source: `outputs/driving-investigation-19-stop-hold/09-t-junction-route/`.


## Right-bending road result

Case05 passes fresh at **11.9 s**: speed 2.657–4.595 m/s, minimum corner
clearance +0.637 m. All 119 execution checks pass; 164 trajectory samples,
36 reuses (max age 1.3 s), eight speed-preference misses. Inference 418.96 s;
capture wall 473.44 s. Source:
`outputs/driving-investigation-19-stop-hold/05-curve-right-route/`.


## Original intersection: straight

Case01 passes fresh at **9.2 s**: speed 2.708–4.625 m/s, minimum corner
clearance +0.832 m. All 92 execution checks pass; 112 trajectory samples,
53 reuses (max age 2.0 s), four speed-preference misses. Inference 295.51 s;
capture wall 345.06 s. The original approach and seed are retained at gain 1.40.
Source: `outputs/driving-investigation-19-stop-hold/01-intersection-straight-route/`.


## Final collection

All ten declared maneuvers have an accepted complete clip, using one fixed
starting condition per scenario. The final batch contains nine fresh passes
under runtime `36d8b37`; the gain-1.40 right turn comes from `51d41d1`. The
stop-hold feature is enabled only for the red case. This is a selected video
collection after documented development, not an untouched 10/10 benchmark.

The final left turn passes at 10.4 s, speed 2.677–4.612 m/s, corner clearance +0.392 m, with all 104 execution checks passing. It uses 114 trajectory samples and 32 reuses.

[Ten-video index](../../outputs/driving-investigation-19-showcase/VIDEOS.md) · [Download all ten](../../outputs/driving-investigation-19-showcase/ten-driving-videos.zip) · [Measured plots](../../outputs/driving-investigation-19-showcase/metrics.png) · [Provenance and audit results](../../outputs/driving-investigation-19-showcase/results.json).

| Scene | Driving time | Speed range (m/s) | Minimum corner clearance | Samples / reuses | Capture wall time | Video |
| --- | --- | --- | --- | --- | --- | --- |
| Intersection: straight | 9.2 s | 2.71–4.63 | 0.832 m | 112 / 53 | 5.8 min | [Watch](../../outputs/driving-investigation-19-showcase/01-intersection-straight-route.mp4) |
| Intersection: left | 10.4 s | 2.68–4.61 | 0.392 m | 114 / 32 | 5.5 min | [Watch](../../outputs/driving-investigation-19-showcase/02-intersection-left-route.mp4) |
| Intersection: right | 6.2 s | 2.73–4.62 | 0.168 m | 87 / 19 | 4.2 min | [Watch](../../outputs/driving-investigation-19-showcase/03-intersection-right-route.mp4) |
| Curved road: left bend | 13.3 s | 2.67–4.61 | 0.647 m | 173 / 53 | 8.8 min | [Watch](../../outputs/driving-investigation-19-showcase/04-curve-left-route.mp4) |
| Curved road: right bend | 11.9 s | 2.66–4.60 | 0.637 m | 164 / 36 | 7.9 min | [Watch](../../outputs/driving-investigation-19-showcase/05-curve-right-route.mp4) |
| Roundabout: first exit | 17.6 s | 2.63–4.60 | 0.459 m | 230 / 30 | 11.3 min | [Watch](../../outputs/driving-investigation-19-showcase/06-round-first-route.mp4) |
| Roundabout: second exit | 28.7 s | 2.60–4.56 | 0.594 m | 380 / 142 | 19.9 min | [Watch](../../outputs/driving-investigation-19-showcase/07-round-second-route.mp4) |
| Tight curved road | 20.4 s | 2.44–4.62 | 0.438 m | 759 / 143 | 34.4 min | [Watch](../../outputs/driving-investigation-19-showcase/08-tight-bend-route.mp4) |
| T-junction: right turn | 11.2 s | 2.60–4.60 | 0.091 m | 187 / 10 | 8.9 min | [Watch](../../outputs/driving-investigation-19-showcase/09-t-junction-route.mp4) |
| Red light: stop and hold | 12.0 s | 0.00–2.95 | 0.828 m | 120 / 0 | 5.7 min | [Watch](../../outputs/driving-investigation-19-showcase/10-red-light-stop-route.mp4) |

Across the accepted clips: **1409 executed-command checks**, **2326 trajectory samples**, and 112.4 minutes of capture wall time (excluding discarded trials, orchestration/audit time and video conversion). Every accepted clip passes all recorded checks. The nine moving clips have no recorded near-stop, collision or corner departure. The red clip stops and holds before the line. All ten H.264 playback files have the expected frame counts and durations; the video includes one initial frame before the recorded driving interval.

The three nonselected captures remain available: the gain-1.18 right turn
(departure), gain-1.30 right turn (formal pass with only 2 mm clearance), and
native red-light trial (stop followed by relaunch and crossing the line). These
are not erased or folded into a first-attempt success rate.

Limits: scenes have no other vehicles or pedestrians; moving cases use green
lights. This does not establish traffic handling or reliability across seeds
and starts. The T-junction has only 9 cm recorded margin, and the tight bend is
expensive to generate. The red case demonstrates an assisted held stop about
4.7 m before the line, not live green-release behavior. Geometry checks sample
bounding-box corners, not the full swept volume. Raw model predictions can
still be unsuitable away from the executed lookahead target.

Final launcher defaults are aligned to the validated gain 1.40. The scene
launcher now requires passing capture/audit/video results and returns nonzero
if a requested case fails, is missing, or server shutdown fails. This exit-status
reporting change was made after the captures; controller code is unchanged.
Earlier trial-driver exit codes alone were not evidence of a driving pass.
The original backend/controller and existing launchers remain unchanged.
Rollback is `experiment/projected-speed-selection` at `0b044ac`; this work is
isolated on `experiment/ten-driving-scenes`.

Final verification: **274 experiment tests pass in 4.34 s**; shell syntax and CLI
help checks pass. Backend files under `module/` are unchanged from `0b044ac`.
The owned CARLA server is stopped and no suite/model worker remains.
