# Route projection with bounded native speed selection — round 18

Branch: `experiment/projected-speed-selection`. Rollback: round-17 branch
`experiment/projected-steering-comparison`, commit `6bc992a`.

The preceding route projection reaches both exits but nearly stops and surges
to about 10.17 m/s. The left vehicle also overhangs its route corridor by 0.29 m.
Recorded native targets before the stops are only 0.25–0.30 m/s. Projection
does not change these predictions or the resulting longitudinal controller.

## Predeclared first test

Keep the route projection and original follower unchanged. Prefer a native
controller-preview speed of 3–5 m/s, with at most three separate single-candidate
inferences per control step. Retry both text and diffusion seeds. Accept the
first candidate inside the band; if none qualifies, execute the closest valid
native candidate and explicitly label the missed preference. No fabricated
speed, minimum-speed clamp, cached plan, forced go-intent or trajectory edit.
Every proposal and its inference time is retained. A red/yellow visual signal
keeps the first model result without retrying for motion.

This is scoped to the configured empty all-green junction, not a general traffic
policy. The simulator's signal state is checked only to enforce that experiment
scope; visual red/yellow still preserves the stop. No safety claim follows from
finding a speed inside the preferred band.

Run the same centered straight and left cases, caps 10 and 12 s, same initial
speed/pose/seeds and single A5000 BF16 placement as round 17. Retain all outcomes.
If the speed preference alone does not correct the left tracking error, inspect
that result before changing steering. This keeps the first comparison focused.

```bash
export CARLAMAYO_MODEL_GPUS=GPU-ce1bea0e-c85a-f48a-a1ad-5c2bc153e10b
export CARLAMAYO_RENDER_GPU=GPU-3f4e4372-21d9-bac9-f577-755a624eb99d
source local/env.sh
.venv/bin/python -m local.experiments.run_projection_suite \
  --output outputs/driving-investigation-18 \
  --single-gpu-model --server-script local/experiments/projected_server.sh \
  --steering-modes route --speed-selection-attempts 3
```

## First comparison completed

All 242 software tests pass before launch `10fee41`. Both integrity audits pass.
Both physical trials fail the intended driving behavior:

| Maneuver | End | Speed range | Minimum corner clearance | Inference calls |
| --- | --- | --- | --- | --- |
| Straight | Sampling exhausted at 5.7 s | 0.927–10.189 m/s | +0.831 m | 153 |
| Left | Correct exit at 7.6 s | 2.576–10.162 m/s | -0.284 m | 202 |

Straight's last three failed-decision proposals have reverse motion and are
rejected. The original audit counts 150 proposals for executed decisions; the
table includes those three additional failed attempts. All raw proposals and
wall times are retained. The updated auditor includes failed-decision calls.
Startup-inclusive case times are 7.48 and 9.11 minutes. The bounded preference
misses its band at 42/57 straight and 55/76 left executed decisions.

Evidence: `outputs/driving-investigation-18/`. The speed preference is not a
speed limit: all sampled candidates sometimes ask for speeds above 10 m/s.
Later, straight again predicts a stop for a supposed stop sign and slows sharply.
Generated reasoning is an observable accompaniment, not proof of the model's
internal causal process.

## Predeclared controller-limit follow-up

Add an explicit **4.5 m/s target-speed ceiling** (16.2 km/h) to the same original
speed controller. It only reduces targets; model stops remain stops. Raw native
positions/speeds are unchanged, but executed longitudinal policy is now capped
and labeled accordingly. Prefer 3–5 m/s after this ceiling, with at most six
single-candidate attempts when a prediction remains too slow. High predictions
need no resampling when the controller can limit them. There is still no
minimum speed, forced go-intent or cached path.

Add an explicitly logged **1.18 steering gain**, preserving the existing rate
and actuator limits. Round-17 recorded turning motion gives a median effective
wheel-angle gain of 0.84486 (10th–90th percentiles 0.79424–0.88642). This agrees
with the earlier round-8 estimate, 0.84485. Compensation is empirical, not a
universal vehicle model. The live vehicle also exposes speed-dependent steering
limits; unchanged physics are saved in `vehicle-physics.json`.

Run fresh centered straight and left under this combined profile, same scene,
seeds, hardware and 10/12 s caps. This combined follow-up is not an ablation of
the ceiling versus steering gain. Require the endpoint, no collision or recorded
corner departure, minimum measured speed >=1 m/s, and maximum <=5.5 m/s (ceiling
plus 1 m/s transient allowance). Audit both declared controller modifications
independently as well as unchanged raw model predictions.

```bash
.venv/bin/python -m local.experiments.run_projection_suite \
  --output outputs/driving-investigation-18-limited \
  --single-gpu-model --server-script local/experiments/projected_server.sh \
  --steering-modes route --speed-selection-attempts 6 \
  --speed-cap-mps 4.5 --steering-gain 1.18
```

Follow-up results pending at this checkpoint.
