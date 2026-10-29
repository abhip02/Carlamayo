# Driving inference investigation — 2026-10-28

Follow-up camera, signal-layout, and multiple-trajectory experiments are recorded
in [RESULTS_ROUND2.md](RESULTS_ROUND2.md). The conclusions below describe round 1.

The slowdown originates in the model's predicted motion. The original controller
then brakes in response. Several independent issues contribute to erratic plans,
but these experiments have **not established a reliable straight-driving fix**.
No production behavior, controller setting, dependency, or model source was changed.

Work is isolated on `experiment/driving-inference-diagnosis`, based on `9ec8b53`
(`feature/verifai-server-client`). Controller, configuration, and controller tests
still match upstream `4914558`. The external Alpamayo checkout remains unmodified
at `36aeb4c`.

## Direct evidence for the stop

In `outputs/driving-investigation/capture-short-baseline/`:

1. At simulation time 2.1 s, the vehicle is moving at 1.103 m/s. The model's
   history-based initial-speed estimate is positive, 1.473 m/s.
2. Its sampled acceleration drives predicted signed velocity to −0.080 m/s after
   1 s and −0.147 m/s after 1.5 s. The distance-derived controller target over
   that window is only 0.123 m/s, below the original 0.25 m/s stop threshold.
3. The controller applies brake 0.3; the next recorded speed is 0.0056 m/s.
4. The generated explanation nevertheless says to accelerate through the green
   light. That text is not a dependable explanation of the decoded trajectory.

The saved `frame_021.json` and `000_verifai/ego.csv` contain this chain. This
establishes the numerical mechanism, not the model's internal reason for choosing
that acceleration. No visual hazard has been established as the cause.

## What the isolated tests established

| Test | Result | Implication |
| --- | --- | --- |
| Replay identical camera and pose tensors | Baseline waypoints reproduce exactly | Frozen-input comparisons isolate inference changes |
| Different global seeds on one verbose input | Raw target ranges from 0.281 to 2.225 m/s | One sampled plan can be a poor representative of the scene |
| Hold text sampling fixed; vary only motion noise | Large speed/reverse-motion changes remain | Motion sampling contributes independently of the generated explanation |
| More integration steps: 10 → 30 | No useful speed recovery on tested inputs | More diffusion steps alone are not a fix |
| Lower motion noise: 1 → 0.3 or 0 | Removes some reverse segments on frozen inputs | Local improvement does not establish closed-loop improvement |
| Replace negative decoder initial speed with measured speed | 37 backward-moving steps → 0 on the saved post-stop input | Corrects a real post-braking artifact, not the preceding slowdown |
| Clamp decoder initial speed to nonnegative | Same reverse-step improvement on that input | Also limited to the post-stop artifact |
| Change history-estimator regularization | Native setting has best recorded RMSE: 0.136 m/s | No evidence to retune the estimator globally |
| Shorten route to “Go straight” / “Go straight in 5m” | Late unwanted-turn prediction remains | Prompt wording alone did not correct the turn |
| Native navigation guidance, weight 1.5 | Default SDPA run crashed; reference math backend completed a late-turn test, still turning | No driving improvement established; attention failure needs separate diagnosis |
| CARLA snapshot freshness | 0 stale immediate snapshots in 71 model-free ticks | Timing-lag hypothesis not reproduced in that test |

The post-stop initial-speed error is specific and substantial: at 2.2 s the
decoder estimates −0.584 m/s while CARLA reports about +0.006 m/s. Substituting
measured longitudinal speed corrects backward extrapolation. It also changes the
distance-based target from 0.498 to 0.089 m/s: removing an artifact does **not**
necessarily raise the target speed. This substitution has only been tested on
frozen inputs, not validated as a complete driving policy.

The normal decoder integrates acceleration without a nonnegative-velocity
constraint. The controller derives speed from waypoint distance, so backward
segments also count as positive speed. Changing either convention would require
separate tests of actual stop/reverse behavior.

## Eight-second live comparison

Both runs used the original controller, short navigation instruction, seed 42,
20 m approach distance, requested 3 m/s speed, zero heading/lateral offset, and
the same measured initial pose and speed. No collision was recorded in either.

| Metric | Original inference | Motion noise 0.3 |
| --- | ---: | ---: |
| Minimum actual speed | 0.0056 m/s | 0.2813 m/s |
| Near-stop events, speed below 0.25 m/s | 1 | 0 |
| Raw predictions below 0.25 m/s | 2 | 3 |
| Plans containing backward velocity | 3 | 3 |
| Final deviation left of starting heading | 9.99 m | 15.16 m |
| Final heading change | −52.38° | −59.58° |

Lower noise slightly mitigated the stop but worsened route deviation. It remains
experimental and is not enabled in the normal launcher. Both runs reach the
junction and turn instead of continuing straight.

Matching initial states does not make rendered episodes bit-for-bit identical:
two verbose runs produced different predictions despite matching measured starts.
Use frozen inputs for causal comparisons; these live episodes are limited checks,
not statistical validation.

## Verbose-prompt format failures

The first 6 s verbose capture had 17 missing trajectory markers in 60 calls;
both 8 s short-prompt runs had none in 80 calls each. Some malformed generations
hit the 256-token limit. Others ended early with answer/chat-ending tokens, so
increasing the token limit cannot address every case.

The native model warns about the missing marker but still runs motion sampling,
using the final generated position as its fallback. The diagnostic recorder now
automatically saves these full inputs, allowing exact replay instead of guessing
from a warning. A second 3.5 s verbose run captured a malformed input at frame 023.
Exact replay of that same input gave:

| Variant | Generated tokens | Valid trajectory marker | Raw target speed |
| --- | ---: | --- | ---: |
| Original verbose prompt | 256 | No | 0.524 m/s |
| Suppress early answer/end tokens | 256 | No | 0.524 m/s |
| Increase budget to 512 | 512 | No | 0.401 m/s |
| Original short route instruction | 16 | Yes | 0.127 m/s |

Thus a short prompt recovered the expected format on this input, but did not
recover normal speed. The token guard and larger budget are not fixes for this
repetitive-generation failure. A missing marker and a low-speed plan are separate
properties; a valid marker alone is not a quality guarantee.

## Artifacts and next steps

All paths below are relative to `outputs/driving-investigation/`:

- `comparison/metrics.json` and `comparison/comparison.png`: measured comparison.
- `capture-short-baseline/000_verifai/overhead-h264.mp4`: original behavior.
- `capture-short-noise-03/000_verifai/overhead-h264.mp4`: noise experiment.
- `capture-*/frame_*.json`: raw tokens, marker status, accelerations, signed
  velocities, estimated/measured initial speed, and final waypoints.
- `capture-*/inputs/*.pt`: selected full camera/pose inputs, about 95 MB each.
- `replay-suite-1/`, `replay-noise/`, `replay-targeted/`, `replay-format/`:
  controlled inference comparisons.
- `replay-navigation/` and `.runtime/driving-probe-navigation.log` (repo-relative):
  failed SDPA-guidance experiment. `replay-cfg-math/` is the reference-backend test.
- `history-estimator.json` and `frame-sync.json`: independent diagnostic checks.

Keep short navigation instructions and native driving defaults for now. The most
concrete next engineering work is validating measured-speed decoder initialization
across stop/start cases, and explicitly detecting/retrying malformed generations.
Neither is a demonstrated solution to route following. Evaluate route deviation
and unnecessary stops as well as collision status: the existing minimal VerifAI
collision specification passes both of the visibly off-route runs above.

Existing controller, adapter, pose-history, and VerifAI tests passed: **63 tests**.
The simulator's rollout vehicles and sensors were cleaned up, asynchronous world
settings restored, and the owned CARLA container stopped after live testing.

To return to the integration branch after saving this work:

```bash
git switch feature/verifai-server-client
```

The normal launchers never import `local.experiments`; no runtime setting needs
to be reverted. Local output files remain available after switching branches.
