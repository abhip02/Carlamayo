# Map-projected steering comparison — round 17

Branch: `experiment/projected-steering-comparison`. Native rollback branch:
`fix/driving-native-validation`, commit `0d380ad`.

**Completed: six fresh runs, three steering modes, two maneuvers.** Restricting
projection to the commanded route substantially improves tracking in these
trials: straight reaches the exit with 0.795 m minimum corner clearance; left
reaches its exit but has up to 0.290 m corner overhang. Both nearly stop and later
reach about 10.17 m/s. None of the six runs passes all driving criteria. This is
evidence for route-constrained steering, not a complete driving fix.

Watch the annotated [straight](../../outputs/driving-investigation-17-route/straight-route-targets.mp4)
and [left](../../outputs/driving-investigation-17-route/left-route-targets.mp4)
captures, or inspect the [six-run comparison plot](../../outputs/driving-investigation-17-route/all-methods-comparison.png).
Green marks the executed steering target, magenta the original model target,
yellow the full native prediction, and cyan the actual driven trail. These
annotations are added after capture and never enter the model input.

The user explicitly requests testing map projection after completing the five
native-policy cases. Round 16 reused successful cases 2 and 4 and ran only
cases 1, 3 and 5: three passes, centered straight exhausted sampling at 4.7 s,
and opposite-offset straight reached its 10 s cap 1.36 m before the endpoint.

## Predeclared comparison

Run centered straight and centered left, each with native and map-projected
steering, from the original five-case manifest. Caps remain 10 and 12 seconds.
Use the same model, scene, camera inputs, initial conditions, seeds and speed
controller in each pair. Fresh inference starts at zero. Every control step
uses one sampled trajectory, with no rejection/resampling or previous-plan
selection. Format guarding, image-based signal checks and acknowledged frame/
control delivery are shared. This isolates projection under a cheap sampling
policy; it is not a repeat of the heavy native validation benchmark.

The historical CarlaMayo demo selected a lookahead target and called
`map.get_waypoint(location, project_to_road=True, lane_type=Driving)`.
This experiment uses that same nearest-driving-lane projection operation on
the current follower's lookahead target. It retains current pure-pursuit
steering, steering rate limits and native longitudinal control. It does not
reproduce the historical CARLA PID controller, cumulative-distance target
selection or 10 km/h minimum speed. It does not project the entire prediction
or constrain the map lookup to the commanded route. Lane selection inside
junctions may consequently be wrong; road/lane identifiers and projection
distances are recorded explicitly.

Historical source:
https://github.com/aveeslab/Carlamayo/blob/578170cfbcb75222ff1fb6cd9e2d4c7d6167a4f9/carla_alpamayo_closed_loop.py

## Implementation and verification

- `projected_steering.py`: opt-in target projection; original module unchanged.
- `projected_probe.py`: isolated capture wrapper; original runtime restored on exit.
- `run_projection_suite.py`: declared modes run sequentially, frozen source checks, retained
  failures, per-case H.264 videos and automatic shutdown of the owned server.
- `audit_projection.py`: reconstruct original longitudinal control, projected
  steering and map lookup from saved OpenDRIVE; compare executed controls,
  native arrays and snapshot timing; measure recorded corner clearance.

All 234 experiment tests passed in 4.46 seconds before launch. Tests cover native
equivalence, unchanged longitudinal state and arrays, preserved stops, projected
steering rate limits, missing-waypoint failure and wrapper cleanup.

The initial launch used the default sharded model and existing owned server;
that hardware trial was cancelled as described below. The completed captures
use the explicit single-GPU model and separate server options shown at the end
of this report. A completed process is not necessarily a successful drive.

## Hardware trial and comparison allocation

The usual model GPUs were occupied by other users. The initial attempt used
idle TITAN RTX cards, but their older architecture made the unchanged BF16
first inference impractically slow. It was interrupted before any control step.
Evidence is preserved in `outputs/driving-investigation-17/`; it is not a driving
result and no other user's job was interrupted.

The matched comparison instead uses one idle RTX A5000 (host GPU 3) for the
unchanged BF16 model and a separate owned CARLA container on TITAN RTX GPU 4.
Same checkpoint, SDPA attention, simulator image and control settings. One
candidate fits on the A5000 and takes about 2–5 seconds per decision initially.
Both comparison arms use exactly this allocation; older two-GPU benchmark
runtimes are not treated as a hardware-matched speed comparison.

Completed evidence: `outputs/driving-investigation-17-bf16/`, launch `ab7b6f5`.
Six projection tests pass again after the model-placement option. The original
native branch and original CARLA container configuration remain unchanged.

## Completed unrestricted projection comparison

All four fresh captures completed and all independent command, native-speed,
map-query and geometry-recording checks passed. None passed the driving criteria.

| Requested maneuver | Steering | End | Minimum measured speed | Minimum recorded corner clearance |
| --- | --- | --- | --- | --- |
| Straight | Native | Collision at 7.4 s | 0.419 m/s | -17.086 m |
| Straight | Nearest driving lane | 10 s cap, wrong lane | 0.008 m/s | -8.370 m |
| Left | Native | Endpoint at 6.0 s, major corner cutting | 2.636 m/s | -7.185 m |
| Left | Nearest driving lane | 12 s cap, wrong route | 0.013 m/s | -32.266 m |

Negative clearance is relative to the requested route's lane corridor, not an
assertion that every departure was off the road. A pass here requires the route
endpoint, no recorded corner departure/collision and minimum speed >=1 m/s.
The straight map-assisted near-stop corresponds to a native low-speed prediction
whose generated text says to stop for a red light; the separate image-based
check says green. No forced red/yellow intent was injected in either straight run.

The paired initial poses, measured speeds and history tensors match exactly.
Rendered images differ slightly (mean absolute 8-bit pixel difference 1.11 for
straight, 0.83 for left), so these are matched physical conditions, not identical
input tensors or a statistical reliability estimate. Both arms use identical
hardware and settings. Original predictions and their targets are retained.
Annotated `*-targets.mp4` videos distinguish executed targets from raw targets.

## Separate commanded-route projection

After completing the frozen four-run comparison, add `route` as an explicit
third mode. It projects the native lookahead target onto only the map polyline
for the commanded maneuver. This prevents the unrestricted lookup from silently
selecting a different connector or oncoming lane. It is an extension beyond the
historical nearest-lane method. It does not introduce a forward-progress floor,
minimum speed, cached-plan reuse or resampling. Raw predicted positions and
speeds are preserved; the executed steering target is explicitly map-derived.

Predeclare two additional fresh runs: centered straight (10 s cap) and centered
left (12 s cap), same seeds, initial conditions, BF16 A5000 model/TITAN renderer,
signal checks and original longitudinal controller. Keep every outcome.
All 236 experiment tests pass in 4.51 seconds before these additional runs.
Results: `outputs/driving-investigation-17-route/`, launch `6abd2c6`.

## Completed commanded-route results

| Requested maneuver | Exit reached | Measured speed range | Minimum recorded corner clearance | Full driving pass |
| --- | --- | --- | --- | --- |
| Straight | 7.4 s | 0.009–10.173 m/s | +0.795 m | No: near-stop |
| Left | 8.6 s | 0.015–10.174 m/s | -0.290 m | No: near-stop and corner departure |

The straight run remains inside the recorded corridor throughout. The left
vehicle's recorded corners cross its corridor boundary from 4.8 through 6.8 s,
then recover before reaching the correct exit. Neither records a collision.
Geometry checks cover recorded bounding-box corners against the route polyline,
not swept volume, general obstacle safety or arbitrary traffic scenes.

Steering projection does not repair low-speed native predictions. At 2.3 s in
straight, the raw target is 0.884 km/h and the original follower marks a planned
stop; at 1.9 s in left, the raw target is 1.069 km/h. Both apply 0.3 brake and
subsequently nearly stop. All reconstructed longitudinal commands match the
original follower. Minimum measured speed is below 1 m/s at 2.4–2.8 s (straight)
and 2.0–2.3 s (left). Later acceleration to about 10.17 m/s remains undesirable.

All six independent integrity audits pass with zero mismatches, including
unchanged native arrays and speed control, map projection, applied steering and
recorded vehicle state. The commanded-route auditor projects onto saved map
segments independently of the implementation. The two route runs take 4.61 and
4.62 minutes including model startup; their inference totals are 197.6 and
217.5 seconds. There is one candidate per 0.1 s step and no resampling.

These are single trials per mode and maneuver at one empty, all-green Town03
junction, not a reliability estimate. In particular, the historical method's
cumulative-distance target selector, CARLA PID and 10 km/h speed floor were not
reproduced. A saved target-selection diagnostic finds that its index would
differ from the current follower in 28/74, 70/100, 43/60 and 93/120 steps of the
four native/unrestricted captures, respectively. The experiment tests the
historical map-lookup operation with our current follower, and a separate
commanded-route extension; it does not test the complete old demo.

## Evidence and reproduction

- [All six results](../../outputs/driving-investigation-17-route/all-methods-results.json),
  [PNG plot](../../outputs/driving-investigation-17-route/all-methods-comparison.png),
  [PDF plot](../../outputs/driving-investigation-17-route/all-methods-comparison.pdf).
- [Native straight](../../outputs/driving-investigation-17-bf16/straight-native-targets.mp4),
  [unrestricted-map straight](../../outputs/driving-investigation-17-bf16/straight-map-targets.mp4),
  [commanded-route straight](../../outputs/driving-investigation-17-route/straight-route-targets.mp4).
- [Native left](../../outputs/driving-investigation-17-bf16/left-native-targets.mp4),
  [unrestricted-map left](../../outputs/driving-investigation-17-bf16/left-map-targets.mp4),
  [commanded-route left](../../outputs/driving-investigation-17-route/left-route-targets.mp4).
- Each capture retains source hashes, original predictions, controls, map data,
  raw recordings, independent audits and its exact configuration. Generated
  artifacts are local and ignored by Git; code and this report are committed.

From the repo, select available GPUs before sourcing the environment. These were
the GPU UUIDs used for the recorded comparisons (A5000 model, TITAN RTX renderer):

```bash
export CARLAMAYO_MODEL_GPUS=GPU-ce1bea0e-c85a-f48a-a1ad-5c2bc153e10b
export CARLAMAYO_RENDER_GPU=GPU-3f4e4372-21d9-bac9-f577-755a624eb99d
source local/env.sh
.venv/bin/python -m local.experiments.run_projection_suite \
  --output outputs/projected-steering-new-comparison \
  --single-gpu-model \
  --server-script local/experiments/projected_server.sh \
  --steering-modes native map route
```

Use a new output directory. `--steering-modes route` runs only the two
commanded-route cases. The single-GPU option requires a native-BF16-capable GPU
with enough memory for this checkpoint. The dedicated owned container still
uses port 2000; the original server must be stopped before starting it. Both
owned servers are stopped after the recorded comparisons, and no model or suite
process remains active.

All 236 experiment tests pass. `module/`, the standard driving launchers and
VerifAI launchers have no changes from the native rollback branch. Projection
is enabled only through the explicit experimental entrypoint above. To return
to the native checkpoint, switch to `fix/driving-native-validation`; evidence
under `outputs/` remains available. No branch has been pushed.
