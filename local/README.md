# Sharded Alpamayo intersection rollouts

This directory contains the local rollout tools and isolated driving experiments.
Upstream CarlaMayo is pinned at `4914558f868d8e51d482dbcdaf833ad7e2ce7afe`.
The only upstream code change is optional checkpoint-path and attention-backend
arguments in `module/adapters/alpamayo_1_5.py`. The batch runner lives here.

For a concise record of completed work, runs, limitations and slides, see
[PROGRESS.md](PROGRESS.md).

## Driving examples with validation

The driving work is on `fix/driving-native-validation`. Its simple launcher runs
one straight or left example at the empty, all-green Town03 junction. It samples
native Alpamayo trajectories, validates the vehicle footprint and controller
response, and can reuse a recent native plan after checking it again. Selected
waypoints and speeds remain model-generated, with original world anchors and
prediction times preserved. CARLA, the model checkpoint and the original
controller stay the same.
Use `run_driving.sh` to opt into these changes; the existing VerifAI launchers
still use their original driving policy.

```bash
bash local/server.sh start
bash local/run_driving.sh --route straight --output outputs/driving-straight
bash local/run_driving.sh --route left --output outputs/driving-left
bash local/server.sh stop
```

Use a new output directory for each run. The default cap is 15 simulation seconds;
the rollout finishes earlier when it reaches the route exit. Inference runs
synchronously, so a short crossing can take tens of minutes of wall time.
`--print-config` previews the settings without starting a rollout. `--duration`,
`--seed`, `--flow-seed`, and a `--config` containing one explicit rollout allow
overrides. The default planning interval is 0.1 s; `--replan-interval 0.3` selects
the separate bounded-interval experiment.

Recordings are under `OUTPUT/000_verifai/`: `overhead.mp4`, front-camera
`video.mp4`, `ego.csv` and `summary.json`. The capture also saves exact settings, generated map
reference geometry, sampled candidates, selected paths and controller inputs.
The cyan trail is the actual drive; yellow shows the full native future
prediction, including portions that may never be executed.

Both predeclared examples complete from time zero with the final profile:

| Example | Exit reached | Measured speed | Minimum recorded corner clearance | Video |
| --- | --- | --- | --- | --- |
| Offset straight | 9.0 s | 3.27–4.49 m/s | 0.496 m | [Watch](../outputs/driving-investigation-15/02-straight-offset-faster-junction-review.mp4) |
| Centered left | 10.7 s | 2.77–5.01 m/s | 0.103 m | [Watch](../outputs/driving-investigation-15/04-left-centered-junction-review.mp4) |

Clearance measures vehicle corners against the configured route corridor.
All 197 native-path/controller commands pass the independent audits, with no
replayed prefix, recorded near-stop, collision or corner departure. The left
example runs through this launcher, and both resolved defaults exactly match the
completed captures. All 228 experiment tests pass. Capture wall times, including
startup, are 45.2 minutes for straight and 68.7 minutes for left on this machine.

The left turn has a narrow minimum clearance and slows near the exit. The 3–5 m/s
native-target band is a sampling preference, not a guarantee or speed clamp.
Some unused full-horizon predictions remain unsuitable. These two controlled
examples do not establish general driving reliability or replace the deferred
five-case benchmark. Earlier failures, continuations, the measured plots and
rollback details are preserved in [round 15](experiments/RESULTS_ROUND15.md).

## Run

From `/scratch/apomalap/thesis/carlamayo`:

```bash
bash local/server.sh start
bash local/run.sh --config local/configs/demo.yaml
bash local/server.sh stop
```

The demo runs two different starting states. The grid configuration runs 27:

```bash
bash local/run.sh --config local/configs/intersection.yaml
```

For a short run, add `--limit 1 --duration 2`. Duration is **simulation time**;
inference runs synchronously and the experiment can be much slower than real time.
The default model is loaded once per batch, distributed across host GPUs 2 and 1.
CARLA renders on host GPU 3. `local/env.sh` and `local/server.sh` pin GPU UUIDs.
Use `CARLAMAYO_MODEL_GPUS` (two comma-separated GPU UUIDs) and
`CARLAMAYO_RENDER_GPU` to select different cards before launching.

For an existing dedicated remote CARLA 0.9.16 server, skip the server launcher and
pass `--host HOST --port PORT`. The runner loads the configured town, so use a
server assigned to this experiment. It restores world settings/weather/light
states on exit, but does not reload the town that was previously active.

## Initial conditions

The scene is one fixed, straight approach to an intersection in Town03, with
clear noon weather, green frozen traffic lights, and no spawned traffic.
The YAML pins junction 103, approached along lane -1 of junction road 184,
using `junction_entry: {x: ..., y: ..., z: ..., yaw: ...}`. Every batch
records that entry. `--describe-scene` prints it without loading the model.

- `distance_before_intersection_m`: longitudinal distance from the entry.
- `lateral_offset_m`: displacement to the right of the approach centerline.
- `speed_mps`: desired speed at handover, in meters per second.
- `heading_offset_deg`: clockwise/right heading offset from the approach direction.

Locations refer to the CARLA vehicle actor origin. The model's rear-axle frame is
handled by CarlaMayo's existing history conversion and trajectory follower.

Use an explicit `rollouts` list (demo) or `initial_conditions` arrays (Cartesian
product). Each episode settles the vehicle, renders a constant-heading scripted
pre-roll, and fills all 16 ego-history samples and four frames per camera.
The last warm-up tick enables vehicle physics and applies the requested velocity.
Physics can cause small handover deviations; `initial.json` records the requested
world pose and measured handover state. After handover, physics and Alpamayo's
trajectory follower control the vehicle. Warm-up is excluded from the duration.

The runner starts a fresh ego, sensors, history and controller for every episode.
It ends on a collision or duration limit, then starts the next configuration.
Infrastructure/model exceptions stop the batch and preserve the failed summary.
There is no automatic in-episode respawn.

## VerifAI sampling

The optional launcher samples the same four initial-state variables with
VerifAI 2.2.0, then passes an explicit rollout list to the existing batch runner.
The model loads once for the whole batch. The scene remains the empty Town03
intersection described above. This first version uses independent uniform random
sampling, with no adaptive feedback or Scenic scenario scripts.

After completing the existing local model/CARLA setup, install the sampler once:

```bash
bash local/setup_verifai.sh
```

This creates `.runtime/verifai-venv/` with Python 3.12 and the optional dependencies;
it does not modify the model's `.venv` or the shared ML environment. VerifAI itself
depends on the Scenic Python package, but this launcher does not use Scenic to
describe or run scenarios.

Run the example (two eight-second rollouts):

```bash
bash local/server.sh start
bash local/run_verifai.sh
bash local/server.sh stop
```

Edit `local/configs/verifai.yaml`: `sampling.count` selects the number of episodes,
`sampling.seed` selects the repeatable sample sequence, and `sampling.ranges`
contains `[minimum, maximum]` for each initial-state variable. Equal bounds hold a
value fixed. The example samples distance 20–25 m, speed 3–4 m/s, heading −5° to
+5°, and keeps lateral offset at zero. These are configured test distributions,
not estimates of real-world traffic probabilities. The top-level `seed` still
controls the existing policy inference behavior independently.

Inspect samples without CARLA or model inference:

```bash
bash local/run_verifai.sh --sample-only
```

Use `--config`, `--limit`, `--duration`, `--output`, `--host`, `--port`, and
`--checkpoint` to customize a run, for example:

```bash
bash local/run_verifai.sh --limit 1 --duration 1 --output outputs/verifai-smoke
```

The usual output `config.json` records the exact `rollouts`, distribution, scene
seed, and VerifAI version. Replay those states directly, without sampling again:

```bash
bash local/run.sh --config outputs/verifai-smoke/config.json --output outputs/verifai-replay
```

The configuration loader accepts JSON as well as YAML. `--sample-only` also prints
an explicit YAML config that can be saved and passed to `local/run.sh`. Reusing a
scene seed reproduces samples with the pinned sampler; full physical simulation
reproducibility is a separate consideration. Only numeric bounds are checked;
wider ranges may produce invalid spawns or leave the road, and the existing runner
stops on infrastructure/initialization errors.

Simulator-free checks, also run by GitHub Actions:

```bash
uv pip install --python .runtime/verifai-venv/bin/python pytest
.runtime/verifai-venv/bin/python -m pytest -q tests/test_verifai_sampling.py
```

## VerifAI server/client

For sampling with feedback after each episode, use the new launcher on branch
`feature/verifai-server-client`. It starts a VerifAI coordinator and one persistent
CarlaMayo client automatically. Start CARLA separately as usual:

```bash
bash local/server.sh start
bash local/run_verifai_server.sh --limit 2 --duration 1 --output outputs/verifai-server-demo
bash local/server.sh stop
```

Run `bash local/setup_verifai.sh` first if the optional sampler environment has not
been installed. With no options, the launcher reads `local/configs/verifai.yaml`
and runs its two eight-second episodes. Use a new output directory for each run.

- `sampling.count` in the YAML sets the number of episodes; `--limit N` caps it.
- `sampling.ranges` controls distance, lateral offset, speed and heading. The map,
  weather, empty intersection and navigation instruction stay in the existing setup.
- `--duration N` overrides the simulation seconds per episode.
- `--sampler random` is the default uniform sampler. `halton` and `ce`
  (Cross-Entropy) are also available, using VerifAI's default settings.
- `--verifai-port` selects the local coordinator port (default 8888; 0 chooses a
  free port). `--host` and `--port` still select CARLA (default 127.0.0.1:2000).
- `--timeout` bounds waits for the client and socket inactivity (default 1200 s).

The flow is:

```text
VerifAI Server: sample → send initial state → wait for rollout summary
                                               ↓
CarlaMayo client: reset ego/sensors → run rollout in CARLA → return summary
                                               ↓
VerifAI monitor: score summary → feed score into the next sample → repeat
```

The specification is deliberately basic: **+1 if the episode reaches its duration
without a recorded collision; −1 if a collision is recorded**. Spawn, sensor,
model and connection errors abort the batch with an error status, rather than
receiving a safety score. The score checks only collisions during the evaluated
episode; it does not check route following or reaching the intersection's exit.
Cross-Entropy uses negative scores to adjust future samples. This binary score
provides little guidance when every sample passes.

`local/verifai_server.py` subclasses the actual `verifai.server.Server`; its
inherited `run_server()` performs sampling, monitor evaluation and feedback.
`local/verifai_client.py` runs in the model environment and keeps Alpamayo loaded
across episodes. It uses a small JSON socket protocol instead of inheriting
`verifai.client.Client`, so VerifAI's dependencies stay in the sampler environment.
The shared `rollout_session()` in `local/run_intersection.py` handles the model
and CARLA session for both the new client and the original batch runner. No Scenic
scenario scripts are needed.

Recordings remain in `000_verifai/video.mp4`, `000_verifai/overhead.mp4`, and the
corresponding directories for subsequent episodes. The output directory also has:

- `verifai-results.jsonl`: one completed episode per line, with its exact sample,
  score, pass/fail flag, rollout summary and timing.
- `verifai-summary.json`: overall completion/error status, completed count and
  collision violation count.
- `config.json`: sampling settings, specification and samples actually sent to
  the client. Replay these with `bash local/run.sh --config <run>/config.json`.
- The usual `batch-summary.json`, device map and per-episode logs/recordings.

Simulator-free tests exercise real sockets and subprocesses, including feedback,
Cross-Entropy adaptation, saved replay samples and client failures:

```bash
.runtime/verifai-venv/bin/python -m pytest -q tests/test_verifai_sampling.py tests/test_verifai_server.py
```

## Outputs

`outputs/<UTC timestamp>/` contains the resolved configuration, device map, batch
summary, and one directory per rollout:

- `video.mp4`: front-wide camera at 10 simulation frames per second.
- `overhead.mp4`: fixed overhead view, cyan actual trail, yellow latest predicted
  6.4-second path, and a front-camera inset. Both paths use the rear axle as origin.
- `overhead-preview.jpg`: final annotated overhead frame.
- `overhead.json`: observer camera pose, projection settings and overlay colors.
- `initial.json`: requested and actual handover conditions.
- `ego.csv`: time, world pose, speed, controls and inference duration.
- `predictions.jsonl`: model trajectories in the ego frame and generated text.
- `summary.json`: final state, termination reason, simulation and wall time.

The current follower is CarlaMayo's upstream pure-pursuit/speed controller,
including its launch-from-rest rule and speed limits. This is not AlpaSim's MPC.
The initial sharded run uses BF16, SDPA, one trajectory, seed 42 on each inference,
and ordinary navigation conditioning with guidance weight 1.0.

## Overhead view

Overhead recording is enabled by default. The observer is centered 15 m into the
intersection, 80 m above the road, facing straight down with the approach pointing
up in the image. The same view is used for every initial condition. Change these
settings in either YAML:

```yaml
overhead:
  enabled: true
  height_m: 80
  center_ahead_m: 15
  front_inset: true
```

Set `enabled: false` for the original front-only recording. The extra RGB sensor
has its own frame-matched queue and is never passed to Alpamayo. The path lines are
drawn onto the saved overhead video in OpenCV, not into the CARLA world, so the
model's input cameras never see the annotations. Driving and model settings are
unchanged. The cyan line accumulates the measured path since handover; yellow is
the newest available plan, anchored at the vehicle pose where it was predicted.
The video displays the prediction age (normally 0.1 s after its control step).

To view an existing batch from above without repeating model inference:

```bash
bash local/server.sh start
source local/env.sh
.venv/bin/python -m local.replay_overhead \
  --source outputs/demo-two-starts \
  --output outputs/demo-two-starts-overhead
bash local/server.sh stop
```

Use a new output directory for each replay. This is a **saved-pose reconstruction**:
the vehicle follows the logged poses with physics disabled, and predictions are
read from the original JSONL. The inset is the original front-camera video. It
does not rerun the policy or reproduce unlogged wheel/suspension animation. The
original files are untouched, and `replay.json` records the source and method.
New live rollouts also save overhead video directly during their normal physics
simulation.

The original two eight-second runs have been reconstructed under
`outputs/demo-two-starts-overhead/` (81 frames each). A fresh one-second sharded
run under `outputs/overhead-live-smoke/` completed 10 model calls and saved 11
frame-matched overhead images into its video.

## Completed first batch

`outputs/demo-two-starts/` contains two completed eight-second rollouts, each with
80 model calls and no recorded collision. They ran consecutively with one model
load. The first requested 20 m before the entry, 3 m/s and zero heading offset;
the second requested 25 m, 4 m/s and a +5 degree offset. Measured handover speeds
were 2.95 and 3.91 m/s. Rollout wall times, including initialization, were about
247 and 280 seconds. The YAML defaults remain 20 simulated seconds per rollout.

Both paths deviated from the straight-through instruction. These recordings
demonstrate the sharded integration, initial conditions, control loop and batch
reset; reliable navigation remains a model-behavior question to explore.

## Environment and storage

`bash local/setup.sh` creates the private `.venv`. It reads the established ML
dependencies from `../.venv-alpamayo` through a `.pth` file, and installs CARLA
0.9.16 and headless OpenCV only into this checkout. It does not modify the base
environment. The editable Alpamayo source remains `../alpamayo1.5`.

The existing Hugging Face cache is reused in offline mode, including checkpoint
snapshot `7aba8293c09993f2e125c6819df05d7fa3e873ea`. There is no second model copy.
Other caches, installation logs and temporary files live in `.runtime/`.
Generated files, the virtual environment and outputs are ignored by Git.

The one installation outside this directory is the `carlasim/carla:0.9.16`
Docker image, stored in the existing Docker data root on scratch. Its exact ID
and digest are recorded in `local/installed-image.json` after installation.
The launcher uses this host's existing NVIDIA CDI GPU definitions; no Docker
daemon or system configuration was changed. Runtime versions are recorded in
`local/runtime-versions.json`.

## Revert

The previous AlpaSim and Alpamayo repositories and environment were not edited.
`local/setup-baseline.json` records their original heads/status and Docker images.

1. Interrupt any active `local/run.sh` process and let its actor cleanup finish.
2. Run `bash local/server.sh remove` to remove this experiment's named container.
3. To recover the image space, run `docker image rm carlasim/carla:0.9.16`.
   The image was absent before this setup. Do not force removal if another
   container has since started using it, and do not run a global Docker prune.
4. Preserve any desired outputs, then remove this new `carlamayo/` checkout.
   Everything else introduced by this experiment is inside it.

To revert only the code, retain this directory and use the upstream commit above
as the comparison point; the experiment is isolated on its own local branch.
