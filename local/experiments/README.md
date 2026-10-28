# Isolated driving-inference investigation

These entrypoints run only when explicitly invoked. The normal CarlaMayo and
VerifAI launchers do not import them. Production controller and parameter files
remain identical to upstream `4914558`; the starting integration commit is
`9ec8b53` on `feature/verifai-server-client`.

Capture a short run with full native camera/pose inputs and raw model outputs:

```bash
bash local/server.sh start
source local/env.sh
.venv/bin/python -m local.experiments.inference_probe capture \
  --config local/configs/demo.yaml \
  --duration 8 --output outputs/driving-investigation/my-capture
bash local/server.sh stop
```

Capture runs the first configured rollout. Selected input frames are saved as
local PyTorch dictionaries in `inputs/` (about 95 MB per frame).
Malformed generations are captured automatically. Each per-frame JSON includes
the generated token IDs/text, trajectory-marker status, normalized diffusion
actions, decoded acceleration and signed velocity, history-based initial-speed
estimate, sensor speed, final waypoints and controller target-window speed.

Replay frozen inputs without CARLA, loading the model once:

```bash
source local/env.sh
.venv/bin/python -m local.experiments.inference_probe replay \
  --inputs outputs/driving-investigation/my-capture/inputs/frame_015.pt \
  --output outputs/driving-investigation/my-replay
```

Use `--variants file.json` for a mapping of experiment names to option objects.
Use `--options file.json` to apply one option object to an experimental capture.
Options include `initial_speed` (`measured` or `nonnegative`), `flow_steps`,
`flow_temperature`, `flow_seed`, `seed`, `instruction`, `navigation_weight`,
`max_generation_length`, `greedy_text`, `guard_trajectory_format`, and `sdpa_math`.
An empty object uses the original settings. The format guard suppresses answer/end
tokens before the trajectory marker; it is an experiment, not a validity guarantee.
`flow_temperature` scales the motion sampler's initial noise; it is distinct from
the language model's sampling temperature, which remains at the native value.
`initial_speed: measured` substitutes measured longitudinal speed only in the
action-to-waypoint decoder. It does not change the model's conditioning history.
`sdpa_math: true` selects PyTorch's reference attention backend for that call.

For example, `--variants variants.json` can use:

```json
{
  "baseline": {},
  "less_motion_noise": {"flow_temperature": 0.3},
  "measured_initial_speed": {"initial_speed": "measured"}
}
```

`history_probe` runs CPU-only estimator checks on synthetic histories and the
existing saved episodes. `frame_sync` measures snapshot freshness in CARLA without
model inference. Do not run `frame_sync` concurrently with a rollout using CARLA.
Compare completed captures with the environment containing matplotlib:

```bash
.runtime/verifai-venv/bin/python -m local.experiments.summarize \
  --runs outputs/driving-investigation/capture-short-baseline \
         outputs/driving-investigation/capture-short-noise-03 \
  --output outputs/driving-investigation/comparison
```

The resulting JSON and PNG report measured speed, path, near-stop events,
malformed generations, reverse predictions, and actual initial-state differences.

All runtime model-method replacements are restored in `finally` blocks and exist
only within the experimental process. Captures contain the exact inputs used for
replay; reproduce the baseline first before interpreting a variant. Simulator
episodes are not exact A/B replays, so compare measured initial conditions as well
as configured conditions. Frozen-input tests establish local effects, not reliable
closed-loop driving or safety.

Results and recordings are under `outputs/driving-investigation/` and are not
committed. After saving/committing experiment code, return to the baseline with
`git switch feature/verifai-server-client`. Stop the owned CARLA container with
`bash local/server.sh stop` when finished. No dependency/environment changes are
required for this investigation.
