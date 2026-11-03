# Route projection with bounded native speed selection — round 18

Branch: `experiment/projected-speed-selection`. Rollback: round-17 branch
`experiment/projected-steering-comparison`, commit `6bc992a`.

Current checkpoint: **4/5 original cases pass, all five fresh**. The remaining
case stays in lane and moving but reaches its 10 s cap 1.644 m short. All 499
execution audits and 259 software tests pass. A separate fresh repeat with a
12 s allowance **completes at 10.7 s**, also passing all checks. Thus each of the
five starting conditions has a full successful run, while the original benchmark
remains 4/5. Videos and metrics are below; earlier failures remain in the record.

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

## Controller-limit follow-up completed

All 250 software tests pass before launch `bd71616`. Neither run completes:

| Maneuver | End | Speed range | Minimum corner clearance | Inference calls |
| --- | --- | --- | --- | --- |
| Straight | Sampling exhausted at 3.6 s | 0.002–4.611 m/s | +0.832 m | 130 |
| Left | Sampling exhausted at 5.6 s | 0.531–4.626 m/s | +0.181 m | 139 |

The ceiling controls the speed surge. The calibrated left run stays inside its
recorded corridor until termination, but it has not completed the turn, so this
does not establish full-turn tracking success. Several consecutive decisions
produce only slowing/reversing candidates despite a green visual signal. Some
generated explanations say to accelerate even while the trajectory decelerates;
the sampled text alone is not an adequate explanation of trajectory behavior.

Evidence: `outputs/driving-investigation-18-limited/`. Both captures pass the
corrected independent integrity audit. The first left audit flagged four steer
differences of 1.00–1.24e-6: the independent map projection retained double
precision while CARLA's Location API stores float32. The auditor now reproduces
that API conversion without loosening the tolerance. Original audit reports and
status fields are retained; no recorded control or outcome was changed.

## Predeclared native-plan reuse follow-up

Keep the ceiling, calibration and six-attempt budget. After each fresh candidate,
allow a previously sampled model plan to bridge a low-speed prediction if it is
at most **1.0 s old** and passes current checks. Require the actual car's recorded
corners inside the corridor, a projected target ahead along the commanded route,
and an age-adjusted controller target within 3–5 m/s. A visual red/yellow forbids
reuse. Unknown visual labels remain unknown; this experiment continues to require
the explicitly configured empty all-green simulator scene.

The native waypoint array, world anchor and prediction time remain original.
The controller receives real elapsed time, and the video uses the original plan
origin/time too. Every reused source and current check is logged. Fresh inference
still runs every step; a usable cached candidate can avoid further retries.
This adds no map-based full path or minimum-speed rule. It is a separate opt-in
extension, not evidence that the underlying VLA has stopped making bad predictions.

Run fresh centered straight and left with the same conditions and criteria:

```bash
.venv/bin/python -m local.experiments.run_projection_suite \
  --output outputs/driving-investigation-18-retained \
  --single-gpu-model --server-script local/experiments/projected_server.sh \
  --steering-modes route --speed-selection-attempts 6 \
  --speed-cap-mps 4.5 --steering-gain 1.18 --cache-age-s 1
```

## One-second reuse results

All 255 software tests pass before launch `dd3b782`. Independent execution audits
pass for both captures.

| Maneuver | End | Speed range | Minimum corner clearance | Calls / control steps | Reuses |
| --- | --- | --- | --- | --- | --- |
| Straight | Sampling exhausted at 6.1 s | 0.002–4.613 m/s | +0.832 m | 152 / 61 | 21 |
| Left | **Correct exit at 10.7 s; pass** | **2.661–4.611 m/s** | **+0.134 m** | **119 / 107** | 41 |

The left run has no recorded near-stop, collision or corner departure. All
107 executed commands pass the source/anchor/time/controller audits. Its maximum
reuse age is 1.0 s, wall time including startup is 6.20 minutes, and measured
inference time is 298.85 s. This is a successful controlled example, not a claim
of general reliability. Evidence: `outputs/driving-investigation-18-retained/`.

Straight bridges its first bad patch, then encounters another sustained slowdown.
An offline assessment on those actual recorded states finds otherwise-valid
native plans aged 1.2–1.9 s that the one-second limit excludes. This is diagnostic
evidence, not a replayed or completed physical counterfactual; the JSON is saved
as `second-slowdown-cache-diagnostic.json`.

## Predeclared three-second reuse follow-up

Extend only the maximum reuse age to **3.0 s**, matching the original module's
`TRAJECTORY_MAX_AGE_SEC`. All current footprint, target, speed, visual-stop and
original-source checks remain active. The speed preview must still lie inside
the original 6.4 s prediction. Source age is never reset by reuse. Retest fresh
centered straight and left with unchanged caps and all other settings.

```bash
.venv/bin/python -m local.experiments.run_projection_suite \
  --output outputs/driving-investigation-18-retained3 \
  --single-gpu-model --server-script local/experiments/projected_server.sh \
  --steering-modes route --speed-selection-attempts 6 \
  --speed-cap-mps 4.5 --steering-gain 1.18 --cache-age-s 3
```

## Three-second reuse results

All 256 software tests pass before launch `f432895`. **Both fresh captures pass**
the declared driving and independent execution checks:

| Maneuver | Correct exit | Speed range | Minimum corner clearance | Calls / steps | Reuses / maximum age |
| --- | --- | --- | --- | --- | --- |
| Straight | 9.7 s | 1.599–4.628 m/s | +0.832 m | 152 / 97 | 44 / 3.0 s |
| Left | 10.8 s | 2.673–4.613 m/s | +0.137 m | 118 / 108 | 44 / 2.0 s |

Evidence: `outputs/driving-investigation-18-retained3/`. Straight still hesitates
near the exit. At 7.8–8.1 s, a cached native plan has a valid target growing from
2.590 to 2.970 m/s. It is excluded by the hard 3 m/s reuse gate, while fresh targets
of 0.852, 0.468, 0.347 and 0.316 m/s are executed. The saved
`late-slowdown-threshold-diagnostic.json` records the precise missed alternative;
it is an offline assessment, not itself a physical rerun.

## Predeclared preference-ranking correction and five-case run

Treat the 3–5 m/s band as a preference when comparing fresh and cached plans.
A cached plan still needs forward route progress, the current body inside the
corridor, valid original timing, no reverse motion, and a moving controller target
of at least 1 m/s. Within that valid set, prefer the speed band, then recency.
A below-preference cached candidate must rank better than the best fresh one;
it does not stop the search early. Continue the six-attempt budget looking for
a preferred-speed native plan, retaining the best available alternative if none
appears. This changes candidate selection, not the model's speeds or coordinates.

Freeze this correction with the same 4.5 m/s ceiling, 1.18 gain and 3 s age bound,
then run **all five original starting conditions fresh**, with their original
10/12 s duration caps. No earlier success is substituted into this new batch.

```bash
.venv/bin/python -m local.experiments.run_projection_suite \
  --output outputs/driving-investigation-18-five \
  --single-gpu-model --server-script local/experiments/projected_server.sh \
  --steering-modes route --speed-selection-attempts 6 \
  --speed-cap-mps 4.5 --steering-gain 1.18 --cache-age-s 3 \
  --cases 1 2 3 4 5
```

## Five-case simulator startup and recovery

All 259 experiment tests pass for selection commit `da986b3`. The first two
launches, `outputs/driving-investigation-18-five/` and
`outputs/driving-investigation-18-five-retry/`, fail in CARLA camera setup before
any control steps. The owned renderer exits with SIGSEGV (139), not an OOM kill.
Each folder preserves `startup-failure.json` and `simulator-crash.log`; neither
startup is counted as a driving outcome. Their supervisors/model workers were
stopped before retrying.

Commit `65d035a` allows an explicitly named owned renderer container. The active
batch uses a fresh container on another idle TITAN RTX with the same immutable
CARLA 0.9.16 image, model GPU, scene settings and frozen driving code. Ownership
and GPU checks remain enforced. This is renderer recovery, not a simulator or
backend replacement. Allocation details are in `renderer-allocation.json`.

```bash
export CARLAMAYO_MODEL_GPUS=GPU-ce1bea0e-c85a-f48a-a1ad-5c2bc153e10b
export CARLAMAYO_RENDER_GPU=GPU-78628c0c-fb9d-2814-d558-6117b62e7739
export CARLAMAYO_PROJECTION_CONTAINER=apomalap-carlamayo-projection-clean
source local/env.sh
.venv/bin/python -m local.experiments.run_projection_suite \
  --output outputs/driving-investigation-18-five-clean \
  --single-gpu-model --server-script local/experiments/projected_server.sh \
  --steering-modes route --speed-selection-attempts 6 \
  --speed-cap-mps 4.5 --steering-gain 1.18 --cache-age-s 3 \
  --cases 1 2 3 4 5
```

All five fresh cases use this single allocation/profile. Existing output
directories are deliberately refused; use a new output path when reproducing.
The runner starts and stops its owned server, freezes Python source hashes, saves
every candidate, independently audits each capture, and creates H.264 videos.
The 0.1 s control interval still invokes fresh inference every step; cache reuse
avoids additional retries, not the first inference. Six is a maximum number of
single-candidate attempts per decision, not a fixed batch size.

## Five fresh original-case results

Frozen driving source `da986b3`, launch `65d035a`: **four of five pass** their
original duration caps. All **499 executed commands** pass the independent
raw-source/world-anchor/time/controller audits. Every case stays above 2.66 m/s
with no recorded collision or corner departure. Case 3 remains incomplete because
its 10 s cap ends 1.644 m before the endpoint. No earlier success is reused.

| Case | Outcome / exit time | Measured speed | Minimum corner clearance | Calls / steps | Reuses | Wall incl. startup | Video |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Centered straight | Pass / 9.2 s | 2.664–4.610 m/s | +0.832 m | 112 / 92 | 48 | 6.17 min | [Watch](../../outputs/driving-investigation-18-five-clean/01-straight-centered-route-targets.mp4) |
| Offset faster straight | Pass / 9.4 s | 3.083–4.592 m/s | +0.496 m | 94 / 94 | 36 | 5.80 min | [Watch](../../outputs/driving-investigation-18-five-clean/02-straight-offset-faster-route-targets.mp4) |
| Opposite-offset straight | Incomplete / 10 s cap | 2.688–4.617 m/s | +0.496 m | 110 / 100 | 50 | 6.00 min | [Watch](../../outputs/driving-investigation-18-five-clean/03-straight-opposite-offset-route-targets.mp4) |
| Centered left | Pass / 10.4 s | 2.664–4.621 m/s | +0.128 m | 144 / 104 | 45 | 6.82 min | [Watch](../../outputs/driving-investigation-18-five-clean/04-left-centered-route-targets.mp4) |
| Offset faster left | Pass / 10.9 s | 3.062–4.584 m/s | +0.157 m | 109 / 109 | 26 | 6.18 min | [Watch](../../outputs/driving-investigation-18-five-clean/05-left-offset-faster-route-targets.mp4) |

Full machine-readable [results](../../outputs/driving-investigation-18-five-clean/suite-results.json)
and [speed, clearance and projection plots](../../outputs/driving-investigation-18-five-clean/speed-clearance-projection.png)
are retained. The batch takes 31.29 minutes including simulator startup,
model loading and per-case audits/video conversion; the two earlier setup crashes
are additional wall time and are recorded separately. It uses 569 native
inferences across 499 decisions (1.14 per decision), with 205 plan reuses.
Each decision still performs at least one fresh inference. Counts include every
proposal, not just the chosen one. This is synchronous, far slower than real time.

The original native-validation benchmark was 3/5, with two existing successes
reused. This 4/5 result is a new map-assisted profile with all five captures fresh;
it is not a comparison holding the controller or validation policy constant.
The centered straight hesitation improves to a minimum 2.664 m/s, versus
1.599 m/s in the preceding three-second-cache trial. Centered left still slows
near the exit (final speed 2.694 m/s), and both left turns have narrow clearance.
Neither the VLA itself nor general traffic handling is established as fixed.

Video overlays: cyan is executed motion, yellow the original full native plan,
magenta the native lookahead target, green the executed projected target.
The footer gives the plan age at the control decision; the recorder header gives
its age at the subsequent image timestamp (one 0.1 s tick later).

## Predeclared case-3 duration check

The fresh five-case run's opposite-offset straight reaches its 10 s cap with no
recorded corner departure or near-stop: speed 2.688–4.617 m/s, clearance +0.496 m,
all 100 execution audits pass. It ends 1.644 m short of the required endpoint,
at 3.732 m/s. This remains an **incomplete** benchmark outcome.

After all five original cases finish, run case 3 fresh from time zero with only
the duration cap increased to 12 s. Preserve the same scene, seeds, model,
controller, cache age and hardware. This tests the duration explanation; it
neither alters the original benchmark nor changes the driving policy to rush
the endpoint. Evidence will be in
`outputs/driving-investigation-18-case3-duration12/`.

## Case-3 duration check completed

The fresh 12 s repeat **passes at 10.7 s**, with speed
2.730–4.643 m/s, final speed 4.553 m/s, and minimum recorded corner
clearance +0.496 m. All 107 execution checks pass, with no recorded near-stop,
collision or corner departure. It uses 121 calls and 53 plan reuses (maximum
age 2.0 s); capture wall time including model startup is 6.81 minutes.

[Watch the fresh 12 s duration check](../../outputs/driving-investigation-18-case3-duration12/03-straight-opposite-offset-route-targets.mp4).
[Original versus follow-up data](../../outputs/driving-investigation-18-case3-duration12/comparison-to-original.json)
preserves the original timeout. Configuration/options comparisons confirm that
only the duration changes; all Python source hashes match the five-case batch.
This supports the duration-limit explanation, without claiming a bit-for-bit
continuation. The original benchmark remains **4/5**, not a relabeled 5/5.
Across the original batch and this repeat, all **606 executed commands** pass the
independent execution audits. Both supervisors finish and stop their owned server.

To reproduce that duration check after generating the five-case inputs above,
use a new output directory and the same exported GPU/container settings:

```bash
source local/env.sh
.venv/bin/python - <<'PYCONFIG'
from pathlib import Path
import yaml
source = Path('outputs/driving-investigation-18-five-clean')
out = Path('outputs/case3-duration12-repeat')
out.mkdir(exist_ok=False)
name = '03-straight-opposite-offset-route'
config = yaml.safe_load((source / (name + '.yaml')).read_text())
config['duration_s'] = 12
(out / 'config.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
(out / 'options.json').write_bytes((source / (name + '-options.json')).read_bytes())
PYCONFIG
bash local/experiments/projected_server.sh start
.venv/bin/python -m local.experiments.projected_probe capture \
  --config outputs/case3-duration12-repeat/config.yaml \
  --options outputs/case3-duration12-repeat/options.json \
  --duration 12 --capture-frames 0 \
  --output outputs/case3-duration12-repeat/capture
.venv/bin/python -m local.experiments.audit_projection \
  outputs/case3-duration12-repeat/capture
bash local/experiments/projected_server.sh stop
```

The recorded follow-up uses `run_followup.py` in its output folder to additionally
assert source hashes, matching options/configuration, matching GPU allocation,
and automatic server cleanup. A fresh simulation is not a bit-for-bit replay of
the original prefix even with the same configured seeds.

## Code and rollback

- `projected_probe.py` is the isolated capture entrypoint; `run_projection_suite.py`
  runs the preset comparisons and manages the owned simulator lifecycle.
- `projected_steering.py` applies the declared steering-target projection,
  optional target-speed ceiling and steering gain around the original follower.
- `projected_speed_selection.py` handles the finite native-candidate budget;
  `projected_retention.py` rechecks original model plans at their true age.
- `audit_projection.py` independently checks source arrays, original world anchors,
  elapsed time, declared control changes, applied controls and recorded geometry.
- `projected_server.sh` only manages the explicitly owned CARLA container/GPU.
  It uses the same pinned CARLA 0.9.16 image.

The current branch is `experiment/projected-speed-selection`. Native rollback is
`fix/driving-native-validation` at `0d380ad`; projection-only rollback is
`experiment/projected-steering-comparison` at `6bc992a`. The preceding centered
straight/left success with three-second reuse is `f432895`. All changes are local
commits; nothing has been pushed. Existing driving/VerifAI launchers opt into none
of this, and their files and `module/` are unchanged versus the native branch.

The 259 passing experiment tests cover selection ranking, invalid/stop cases,
cache age/anchor handling and controller limits, alongside the previous tests.
The physical results establish only the recorded empty, all-green Town03 cases.
The model still produces low-speed or otherwise unsuitable proposals; target
projection and selection mitigate them. Speed oscillations remain, left clearance
is narrow, and recorded-corner checks do not cover swept volume, obstacles,
traffic interaction, new maps or general reliability.
