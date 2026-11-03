# Map-projected steering comparison — round 17

Branch: `experiment/projected-steering-comparison`. Native rollback branch:
`fix/driving-native-validation`, commit `0d380ad`.

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
- `run_projection_suite.py`: four sequential runs, frozen source checks, retained
  failures, per-case H.264 videos and automatic shutdown of the owned server.
- `audit_projection.py`: reconstruct original longitudinal control, projected
  steering and map lookup from saved OpenDRIVE; compare executed controls,
  native arrays and snapshot timing; measure recorded corner clearance.

All 234 experiment tests passed in 4.46 seconds before launch. Tests cover native
equivalence, unchanged longitudinal state and arrays, preserved stops, projected
steering rate limits, missing-waypoint failure and wrapper cleanup.

Run from the repository with `local/env.sh` sourced:

```bash
.venv/bin/python -m local.experiments.run_projection_suite \
  --output outputs/driving-investigation-17
```

The suite starts/stops only the existing owned CARLA container. If its usual
model GPUs are occupied, `CARLAMAYO_MODEL_GPUS` can explicitly select two idle
GPUs before sourcing the environment. The device-map layout remains unchanged.

Results are pending at this prelaunch checkpoint. Read `status.json` and the
per-case logs in the output directory for actual progress; a completed test
process is not necessarily a successful drive.

## Hardware trial and active comparison

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

Active evidence: `outputs/driving-investigation-17-bf16/`, launch `ab7b6f5`.
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
minimum speed, altered native trajectory, cached-plan reuse or resampling.

Predeclare two additional fresh runs: centered straight (10 s cap) and centered
left (12 s cap), same seeds, initial conditions, BF16 A5000 model/TITAN renderer,
signal checks and original longitudinal controller. Keep every outcome.
All 236 experiment tests pass in 4.51 seconds before these additional runs.
Results: `outputs/driving-investigation-17-route/` (pending at this checkpoint).
