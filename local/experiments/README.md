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
Input-contract and acknowledged-control follow-up is recorded in
[RESULTS_ROUND4.md](RESULTS_ROUND4.md) on `experiment/driving-input-contract`.
Left-turn instruction and navigation-guidance checks are recorded in
[RESULTS_ROUND5.md](RESULTS_ROUND5.md) on `experiment/driving-turn-instructions`.
Model-only validation/resampling is tracked in [RESULTS_ROUND6.md](RESULTS_ROUND6.md)
on `experiment/driving-plan-validation`. This opt-in wrapper uses map geometry
for sample selection, preserves the chosen model waypoints exactly, and ends the
test explicitly if no sample passes; it does not provide a fallback planner.
Short-lived retention of previously accepted native paths is tracked separately
in [RESULTS_ROUND7.md](RESULTS_ROUND7.md), on
`experiment/driving-native-plan-retention`. It preserves the original world
anchor and prediction time, revalidates against the current state, and logs
every reuse and rejected new pool. It is an opt-in empty-green-scene experiment.
The separate `proactive_probe` entrypoint also compares a revalidated cached
plan with a valid fresh choice. It archives fresh proposals under
`accepted_frame_*.json`, including those not executed, so native provenance and
real prediction ages remain auditable. It does not change the normal launchers.

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
  `calibrated_fisheye` additionally uses the public rig's fitted distortion and
  principal points. The nominal rig was checked against the 1.5 reference clip;
  simulated optics and appearance still differ. See round 4 for checks and results.
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
  delivery method. The round-4 green rollout also had zero mismatches, but still
  failed to drive the requested route.
- `reasoning_prefix`, `prompt_style: "trajectory_only"`, `route_in_user_text`,
  and `prompt_hint`: frozen-input prompt ablations; none is a demonstrated general
  driving fix.
- `expert_sdpa_math: true`: use reference attention only during motion sampling,
  leaving the language-generation backend unchanged.
- `text_temperature`: vary the native language sampler's temperature separately
  from motion noise. The original value is 0.6.
- `completed_reasoning`: provide a complete assistant intent followed by the
  trajectory marker. This diagnostic bypasses language-generated reasoning; the
  resulting text must not be presented as the model's explanation or perception.
- `complete_visual_stop_intent: true`: when the visual signal check says red or
  yellow, use a completed stop intent instead of a prefix for further reasoning.
  Green/unknown retains native generation. This is an isolated round-4 test.

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
`input_contract` compares reference and adapter tokenization on CPU.
`compare_replays` scores frozen variants against the same recorded previous plan;
its candidate costs do not provide a validity guarantee.
`junction_routes` exports the configured junction's lane paths from an already
loaded CARLA map without ticking or changing it. `audit_turn` compares recorded
vehicle centers with those paths and checks the exit position and heading;
`plot_turns` draws the actual paths. Through round 5 these map references are
evaluation data only; the round-6 validator also uses them to select native
model samples, as described below. The earlier route-continuity selector is straight-only
and must be disabled when testing a left instruction. `instruction_summary`
compares the round-5 frozen navigation variants.
`plot_turn_predictions` overlays saved raw predictions and the actual rear axle
using the original coordinate conversion. It requires the model environment's
CARLA Python package, but does not connect to CARLA or load a model.
### Native-model validation and resampling (round 6)

This is an isolated experimental policy, not the normal VerifAI launcher.
Start CARLA on Town03, source `local/env.sh`, and generate the map reference:

```bash
.venv/bin/python -m local.experiments.junction_routes \
  --config local/configs/demo.yaml --output outputs/driving-validation-routes.json
.venv/bin/python -m local.experiments.validated_probe capture \
  --config local/configs/demo.yaml --duration 16 --capture-frames 0 19 45 65 95 \
  --options local/experiments/presets/validated-front-straight-body-42.json \
  --output outputs/my-validated-straight
```

Use a new output directory for each launch. This preset uses the default
empty all-green scene and the original front pinhole camera only. The older
`presets/validated-front-left-43.json` preserves an incomplete left-turn test
and lacks the later body checks; it is not a demonstrated working left policy.
These are controlled tests, not evidence that mixed signals or traffic are solved.

`presets/validated-front-straight-body-42.json` additionally checks projected
vehicle-box corners using native predicted orientations and actual car
dimensions. This configuration completed an 8.5 s straight crossing on the
corrected ranking code without near-stops or recorded corner overhang. The
seed-43 preset completed a 7.9 s crossing on the earlier ranking checkpoint.
These are two successful controlled tests, not a general driving fix.
`audit_body` checks actual recorded box corners independently of prediction
admission; its CLI takes a capture and `--output` path. Older captures need
`--bounds` pointing to a recorded geometry JSON.

The wrapper admits native model samples only when their marker, finite values,
motion direction, lane-center corridor and heading checks pass. It checks at
least 1.5 s and extends through the original controller's actual steering
lookahead. A failed pool triggers resampling, up to three pools of four in this preset.
`validation_max_pools` sets that bound; `num_samples` sets candidates per pool. If
all fail, the test explicitly terminates; no fallback or repaired path is
executed. A separate image-based signal answer affects ranking. Map geometry
is used to evaluate samples, not to manufacture a trajectory. See
[RESULTS_ROUND6.md](RESULTS_ROUND6.md) for limitations and measured results.

`validation_resample_language: true` also varies the text-generation seed on
rejected pools, allowing another generated intent as well as new motion noise.
It leaves the first pool unchanged. Body checks require the native orientation
trace and recorded dimensions; they do not approximate a missing orientation
or silently disable themselves.

Ranking always compares lane RMS and progress over the same 1.5 s, even when
admission has to inspect farther ahead for the actual steering target. Optional
`validation_green_min_target_mps` rejects low-progress native samples only when
the image question says green; it is for the controlled empty scene. Optional
`validation_flow_temperatures: [1, 1.5, 2]` broadens native motion noise on later
rejected pools. These options never increase an existing path's speed or modify
its waypoints. Left-turn experiments using them still need successful live
validation; see the ledger before choosing a configuration.

`presets/validated-native-four-left-reserve-43.json` preserves the four-camera,
eight-sample, six-pool test that reached 7.9 s and 52.5 degrees of left rotation.
It remained incomplete and had 5.6 cm of final body overhang; it is for
reproduction, not a working-left-turn recommendation.
`presets/validated-native-four-left-margin-43.json` adds 0.10 m predicted
clearance; its live run ends at 5.0 s with all recorded corners inside but
without completing the turn.

Optional `validation_prefer_horizon_s: 3` adds a longer forecast preference
on image-reported green: among samples that pass the usual controller-coverage
checks, prefer those that also pass the same checks for at least 3 s. If none
does, rank the ordinarily admitted samples as before. It does not accept a
rejected sample, change any waypoint, or override red/yellow/unknown ranking.
This experimental preference is absent from the tested straight preset; its
left-turn live results are recorded in the round-6 ledger.

Optional `validation_body_margin_m` requires additional predicted corner
clearance (default zero, requires body checks). A 0.10 m experimental value
is motivated by measured tracking error in the incomplete left turn; it is
not a certified uncertainty bound. Actual recorded footprints must still be
audited. The frozen-pool CLI accepts the same value as `--body-margin`.

Every accepted array can be compared with its original candidate and the
controller's input using `audit_capture`. `proposals/` retains all proposals,
including rejected ones. `validation-failure.json` and its saved input record
failure; `route-exit-reached.json` records reaching the requested exit. Ending
at that exit does not certify the preceding driving. New probe launches save
exact source snapshots, hashes, and option values automatically.

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
