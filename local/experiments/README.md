# Isolated driving-inference investigation

These entrypoints run only when explicitly invoked. The normal CarlaMayo and
VerifAI launchers do not import them. Production controller and parameter files
remain identical to upstream `4914558`; the starting integration commit is
`9ec8b53` on `feature/verifai-server-client`.

See [RESULTS.md](RESULTS.md) for the initial diagnosis and
[RESULTS_ROUND2.md](RESULTS_ROUND2.md) for camera, signal, and multiple-trajectory
experiments. The latter are isolated on `experiment/driving-input-fixes`.
Common-setting validation is recorded in [RESULTS_ROUND3.md](RESULTS_ROUND3.md) on
`experiment/driving-validation`.

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

Additional input/sampling experiments:

- `camera_subset`: canonical camera IDs, e.g. `[1]` for the front camera.
- `camera_projection`: `equidistant` or the control `pinhole_from_wide`.
  Capture renders a wider pinhole rig and remaps it before inference, approximating
  the model's lens geometry. This does not reproduce the full lens calibration.
  Saved tensors and video insets contain the **raw wide pinhole images**; replay
  must specify the same projection. `{}` on these captures is not a native-camera
  baseline. `render_projection` in each input's context identifies the source.
- `traffic_layout: "ego_green"`: the approach light is green and the other lights
  in its group are red. The normal launcher makes every light green. Alternatively,
  `traffic_light_state: "Red"` freezes all lights red for a stopping check.
- `num_samples: 4`: use the model's native multiple-trajectory sampling.
- `select_route_continuity: true`: choose one unmodified candidate using deviation
  from the configured straight route and agreement with the previous plan. The
  selector has no obstacle cost and does not support arbitrary routes.
- `verify_signal: true`: every five planning ticks, ask the model a separate visual
  question about the ego light. A red/yellow answer conditions the driving response
  with a stopping prefix. Unknown leaves driving inference unchanged. This uses
  images, not simulator light state, and can itself make perception mistakes.
- `prefer_stop_on_red: true`: when that visual check reports red/yellow, add total
  predicted path length squared to the candidate cost. This penalizes plans that
  stop briefly and then restart. It still selects a model sample; it does not
  impose a brake command. This option is under evaluation.
- `flow_seed`: fix motion noise independently of the language generation length.
- `wait_for_snapshot: true`: after each simulator tick, wait until the client has
  received that exact frame before reading the ego pose. This isolates a pre-roll
  history race; it does not change the controller or simulated tick duration.
  Waiting alone did not fix the observed pre-roll errors.
- `acknowledged_preroll: true`: use acknowledged CARLA commands during placement
  and read poses from exact-frame actor snapshots. This produced clean histories
  in 10/10 initialization trials. It is separate from learned driving quality.
- `acknowledged_controls: true`: deliver the original controller's unchanged
  command through a synchronous CARLA batch before ticking. The model-free
  control-delivery comparison had 7/40 stale commands natively and 0/40 with this
  delivery method; full driving validation remains pending.
- `reasoning_prefix`, `prompt_style: "trajectory_only"`, `route_in_user_text`,
  and `prompt_hint`: frozen-input prompt ablations; none is a demonstrated general
  driving fix.

`controller_speed_lead_s` exists solely to reproduce a **rejected** experiment
that collided with a pole. It is absent from the saved candidate presets.

The selector and periodic signal checker are stateful; evaluate them with live
capture. Frozen-input replay is intended for independent inference variants.

Small option presets are saved in `presets/`. `straight-crossing.json` reproduces
the selected green experiment; `red-stop-check.json` is the red hold experiment;
`acknowledged-baseline.json` changes only initialization. These are experimental,
with their validation status and limitations in `RESULTS_ROUND2.md`.
Follow-up common-setting validation failed on green; these presets should not be
treated as a reliable driving fix. See `RESULTS_ROUND3.md` before reusing them.
`common-green-43.json` and `common-red-43.json` preserve the failed round-3
validation settings for reproduction.

```bash
source local/env.sh
.venv/bin/python -m local.experiments.inference_probe capture \
  --config local/configs/demo.yaml --duration 8 --capture-frames -1 \
  --options local/experiments/presets/straight-crossing.json \
  --output outputs/driving-investigation-2/my-straight-test
```

Start CARLA first as above. `--capture-frames -1` avoids large regular input saves;
malformed generations are still saved. Capture runs the first configured scene.
Inference probes now take an exclusive process lock to reject overlapping model
launches before loading GPUs or changing simulator settings. This does not guard
unrelated CARLA scripts; run simulator-mutating diagnostics sequentially.

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
model inference. `preroll_probe` compares repeated initializations with and without
snapshot waiting. Do not run either timing probe concurrently with a live rollout.
`control_delivery_probe` compares native and acknowledged control delivery without
loading the driving model. `audit_capture` compares requested/reported controls,
recorded poses and chosen candidates, and can inspect a partial capture.
Compare completed captures with the environment containing matplotlib:

```bash
.runtime/verifai-venv/bin/python -m local.experiments.summarize \
  --runs outputs/driving-investigation/capture-short-baseline \
         outputs/driving-investigation/capture-short-noise-03 \
  --output outputs/driving-investigation/comparison
```

Add `--horizon 8` when comparing the first eight seconds of runs with different
durations. Full-run termination and duration remain explicitly recorded.

The resulting JSON and PNG report measured speed, path, near-stop events,
malformed generations, reverse predictions, and actual initial-state differences.

All runtime model-method replacements are restored in `finally` blocks and exist
only within the experimental process. Captures contain source inputs and the
preprocessing options needed for replay; reproduce the baseline first before
interpreting a variant. Simulator
episodes are not exact A/B replays, so compare measured initial conditions as well
as configured conditions. Frozen-input tests establish local effects, not reliable
closed-loop driving or safety.

Results and recordings are under `outputs/driving-investigation/` and
`outputs/driving-investigation-2/` and are not
committed. After saving/committing experiment code, return to the baseline with
`git switch feature/verifai-server-client`. Stop the owned CARLA container with
`bash local/server.sh stop` when finished. No dependency/environment changes are
required for this investigation.
