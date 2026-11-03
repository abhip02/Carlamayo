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

Results pending at this prelaunch checkpoint. Software tests, final captures,
audit outcomes and videos will be added after the runs.
