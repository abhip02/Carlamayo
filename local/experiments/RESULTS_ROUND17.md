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
