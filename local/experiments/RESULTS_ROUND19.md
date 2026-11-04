# Ten distinct driving scenes — round 19

Branch `experiment/ten-driving-scenes`; rollback `experiment/projected-speed-selection`
at `0b044ac`. The user requests ten videos, one start per scenario, including
straight/left/right at the existing intersection, different road layouts, and a
red-light or stop-sign case. This suite uses a red light.

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

The nine moving cases retain the round-18 target ceiling (4.5 m/s), steering gain
(1.18), six single-candidate maximum, 3–5 m/s preference, and original-plan reuse
up to 3 s. They require the declared endpoint, minimum measured speed >=1 m/s,
maximum <=5.5 m/s, no recorded collision or corner departure, and passing
source/world-anchor/time/controller audits. Map paths use connected forward CARLA
lanes; no lateral lane changes or geometric jumps are invented by the exporter.

The red case executes one fresh native prediction per step, with no selection
for motion and no cached substitute. The same steering projection and speed
ceiling apply. It must keep its entire front footprint before the stop line,
finish within 8 m of it, and hold speed <=0.2 m/s for the final 2 s. The normal
moving-case minimum-speed/endpoint criteria do not apply to this stopping task.
Simulator red state configures/checks the scene; it does not inject braking or
replace the model's prediction. Visual signal answers remain recorded.

All cases check live scene scope, including curved roads without nearby traffic
lights. Absence of a visible light is not relabeled green. Every candidate and
failed run is retained. A successful video collection will identify any retries
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

Run (choose available GPU UUIDs first; this launcher starts/stops its owned server):

```bash
export CARLAMAYO_MODEL_GPUS=GPU-ce1bea0e-c85a-f48a-a1ad-5c2bc153e10b
export CARLAMAYO_RENDER_GPU=GPU-3f4e4372-21d9-bac9-f577-755a624eb99d
export CARLAMAYO_PROJECTION_CONTAINER=apomalap-carlamayo-ten-scenes
bash local/run_scene_suite.sh --output outputs/driving-investigation-19-ten \
  --stop-on-failure --cases 3 10 4 6 7 8 9 5 1 2
```

Use a new output directory for every attempt. The initial order exercises newly
added maneuvers first. Rendering uses a fresh owned container with the same pinned
CARLA 0.9.16 image. Other users' GPU jobs remain untouched; hardware availability
can change between attempts and each allocation is recorded.

Results pending.

## First right-turn result and isolated follow-up

Launch `51d41d1`, `outputs/driving-investigation-19-ten/`: the right turn reaches
its endpoint at 6.3 s, with speed 2.710–4.620 m/s and all 63 execution checks
passing. It **fails driving geometry**: the worst recorded corner is 0.195 m
outside the corridor during the late turn/exit. There are 98 model calls and 17
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
