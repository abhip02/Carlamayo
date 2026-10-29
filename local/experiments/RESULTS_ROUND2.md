# Driving inference experiments — round 2, 2026-10-29

**Follow-up:** [Round 3](RESULTS_ROUND3.md) failed to reproduce reliable straight
driving with the combined settings: the new green run stopped early and turned
off course. The favorable episodes below are individual results, not a general fix.

Work branch: `experiment/driving-input-fixes`, starting from `8dee5f7`.
The integration baseline is `feature/verifai-server-client` (`9ec8b53`).
All changes are confined to `local/experiments/`; production launchers do not
import them. Controller, configuration and controller tests still match upstream
`4914558`. External Alpamayo source remains unchanged at `36aeb4c`.

The [first investigation](RESULTS.md) records the original numerical stop mechanism,
noise/seed/decoder/history tests, prompt failures, and attention-backend tests.
This document records the subsequent input and sampling experiments, including
failures. Raw evidence lives in `outputs/driving-investigation-2/` unless specified.

## Current conclusion

Selecting among four native trajectories substantially improved this straight
crossing. On the previously failing seed 43, the first eight seconds had no
near-stop and at most 0.615 m lateral deviation. The original eight-second
baseline stopped and deviated 9.99 m. This is a promising limited experiment,
**not a validated general driving fix**. Camera geometry, scene lighting, sampling
and selection differ together in this comparison; it does not isolate one cause.

The 12-second selected run crossed the target junction and stayed within 0.714 m
of the original heading line. It then nearly stopped at 10.7 s, just beyond the
junction. Images show another junction and a STOP road marking ahead. We have not
established whether that late stop is appropriate. One selected generation lacked
the trajectory marker at 8.4 s; validity filtering is not implemented.

The initial full red-light rerun completed but did not hold continuously: it stopped at
2.3 s and 5.8 s, restarted, and ended moving at 0.773 m/s. Its front remained about
0.27 m behind the stop line at 8 s, so this was a stop-and-go failure, not an
observed line crossing. The final full-horizon stop-cost variant then completed
8 s: after a brief 0.288 m/s restart at 2.1 s, speed stayed below 0.011 m/s from
2.2 s onward. It ended essentially stationary, 10.34 m short of the stop line,
with no collision or missing selected trajectory markers. This is conservative
early stopping, not normal stop-line positioning. Green and red results are
separate scene-specific experiments; the newest combined options have not been
validated across green lights, traffic, or arbitrary routes.

## Completed live experiments

All used the same requested approach: Town03 junction 103, 20 m before entry,
3 m/s initial speed, zero heading/lateral offset, short straight-ahead instruction.
Measured starting pose/speed matched the baseline in the completed comparisons.
Near-stop means actual speed below 0.25 m/s. Lateral deviation is measured against
the initial heading line, not distance to a map-derived curved route.

| Output directory | Change | Duration | Observed result |
| --- | --- | ---: | --- |
| `capture-front-only` | Front camera only | 12 s | Straight crossing; max lateral 0.530 m; still nearly stopped (minimum 0.0144 m/s). Loses peripheral views. |
| `capture-fisheye` | Approximate equidistant lens projection, all cameras | 8 s | No near-stop (minimum 1.966 m/s), but unwanted turn; max lateral 17.19 m. |
| `capture-ego-green` | Ego signal green, other group signals red | 8 s | Max lateral 0.787 m, but near-stop remained (minimum 0.00263 m/s). |
| `capture-ego-green-near-speed` | Signal layout plus controller speed lead 1.0 → 0.2 s | 7.1 s | **Rejected:** hit a pole. No default/controller change retained. |
| `capture-fisheye-ego-green` | Lens + signal layout, seed 42 | 12 s | No near-stop/collision; minimum 1.476 m/s; forward 64.06 m. Changed through lane; max lateral 3.83 m. |
| `capture-fisheye-ego-green-seed43` | Same input changes, seed 43 | 12 s | Failed robustness: near-stop, wrong turn; max lateral 17.27 m; forward only 29.16 m. |
| `capture-fisheye-red` | Lens + all signals frozen red | 8 s | Stopped, then falsely claimed green and restarted around 6.5 s. Front crossed the approach stop line by 0.819 m. |
| `capture-selected-green` | Four samples + route/continuity selection + visual signal check | 0.5 s | Aborted by a VQA generation-count bug. Not a completed driving result. |
| `capture-selected-green-v2` | Same wrapper after VQA bug fix; seed 43 | 12 s | Crossed straight; max lateral 0.714 m; no collision. No near-stop in first 8 s; late near-stop at 10.7 s. |
| `capture-selected-red` | Same wrapper, all signals red | 4.8 s, interrupted | 48 completed calls. Speed 0.00957 m/s at interruption. Full red hold **not validated**. |
| `capture-selected-red-v2` | Fresh full red rerun | 8 s | Stayed behind stop line but repeatedly restarted; final 0.773 m/s. Three selected generations lacked markers. |
| `capture-acknowledged-baseline` | Original inference with acknowledged initialization | 8 s | **Confounded:** a second capture was mistakenly started before this process exited. Its GPU load failed, but shared simulator setup may have affected the first run. Excluded from driving-quality conclusions. |
| `capture-selected-red-stop-cost` | First stop-cost launch attempt | No rollout | GPU OOM while prior model was still resident. No driving result. Retry uses a new output directory after both processes exit. |
| `capture-selected-red-stop-cost-v2` | Stop-length cost on visual red + acknowledged initialization + snapshot wait | 8 s | Stopped and held from 2.2 s onward; one brief initial restart at 2.1 s. Final speed about 0.000001 m/s, minimum front clearance 10.34 m. No collision, zero missing selected markers. |

The first selected red run was gracefully interrupted while the user checked available usage,
not because of a simulator/model failure. Its CSV, frame records and inputs were
preserved. It has no completed batch summary. The subsequent v2 rerun is complete.

## Frozen-input experiments

| Evidence | Tests | Result / interpretation |
| --- | --- | --- |
| `replay-inputs` (24 evaluations) | Original vs fixed motion RNG; reference attention; front only; front + telephoto; omit telephoto; route in user text; straight/lane reasoning prefixes; trajectory without CoT; navigation guidance weights 3 and 8 | Front-only reduced the saved late-turn endpoint substantially. Stronger guidance and prompt-only changes did not fix it. Reference attention was numerically close to native attention. No single prompt change established a live fix. Exact settings are in `input-variants.json`. |
| `replay-red` (6 trajectory evaluations + VQA) | Saved false-restart frame: native; fixed noise; no CoT; explicit red fact; stop instruction; stopping reasoning prefix | Native text claimed green despite rendered red. Explicit fact/route instruction and no-CoT still produced forward motion. The stopping prefix reduced target speed to 0.080 m/s and endpoint to 0.533 m. Separate visual question correctly answered Red on the same images. |
| `replay-multisample` | Four native samples on saved failing seed-43 frame 057 | First three trajectories turned off route; fourth provided a useful straight alternative. Offline selector chose that fourth sample. `candidate-selection-check.json` records its costs. |

`replay-selected-prefix` contains 12 additional evaluations (four variants on
frames 022, 057 and 100 of the selected green run), plus three VQA answers.
`prefix-selection-comparison.json` applies the same recorded previous plan to
every variant. A stronger route prefix lowered the early target from 1.057 to
0.608 m/s and substantially worsened the late route cost; putting the route into
user text also worsened the late route. Removing CoT did not consistently recover
speed. These variants were not promoted to live tests. The broad visual question
returned “No relevant objects detected” on all three inputs and did not resolve
the interpretation of the late stop.

Prepared front-camera /
short-controller-preview combinations without capture directories were not run.

## What the experimental wrapper actually does

1. Render wider pinhole cameras and remap to an approximate equidistant projection
   using the original rig's central focal lengths. This is not full calibration.
2. For the green test, set the ego approach signal green and other signals in its
   group red. The normal launcher freezes all world signals green.
3. Draw four trajectories with native model sampling, fixing motion RNG separately
   from variable-length language generation.
4. Select one **unmodified** trajectory using straight-route deviation and
   agreement with the previous plan. No synthesized waypoints or speed floor.
5. Every five planning ticks, ask a separate image-only question about the ego
   signal. Red/yellow conditions the driving response with a stopping prefix.
   Green/unknown leaves the native response prefix unchanged.

The visual check uses the model's images, not CARLA signal ground truth. Simulator
signal state is used only for setup and validation. All four camera views and the
original controller, speed preview, steering limits and decoder remain in use.

Selection is limited to this straight route and has **no obstacle cost**. It may
select a malformed generation; the 8.4 s frame demonstrates that limitation.
The visual checker can also be wrong. Selecting among four samples costs more:
median inference was 6.79 s per 0.1 s simulation tick in the completed green test;
the 12 s rollout took about 893 s wall time. This is offline simulation.

The initial aborted wrapper run exposed a Transformers behavior: VQA's configured
`num_return_sequences=1` inherited the preceding trajectory call's value 4. The
experimental generation wrapper now explicitly passes this argument. Signal
answers are parsed for an unambiguous color even when the response is a sentence.

## Checks, limitations and evidence locations

- Existing controller/adapter/history/VerifAI regression tests: **63 passed**.
- All 120 selected green trajectories exactly match their chosen raw candidate.
  Motion RNG fingerprints are identical across those calls.
- Analytic camera-map checks passed for central ray, bounds, monotonicity and ray
  angles. This validates the remapping math, not the camera-domain hypothesis.
- Map topology confirms a real straight-through connector in junction 103;
  `junction-topology.json` records it. Exit is near y=149.16 m. The later stop is
  near y=150.16 m; `exit-map.json` records subsequent map geometry.
- One aborted run had an anomalous pre-roll pose-history displacement. Completed
  rollout CSVs did not show stale position repeats or large kinematic residuals.
  No synchronization change was made; this is not an established root cause.
- `selected-comparison-8s/`: matched-duration metrics and plots, including the
  original baseline and the seed-43 input-only failure.
- `selected-green-12s/`: full selected-run metrics, including the late stop and
  one missing trajectory marker. Do not hide these with the 8 s crop.
- `selected-before-after-8s.mp4`: original seed 42 vs selected seed 43, labeled.
- `capture-selected-green-v2/000_verifai/overhead-h264.mp4`: full 12 s recording.
- `red-light-check.json`, `ego-bounds.json`, `red-model-input-065.jpg`: false restart
  and front-bumper stop-line calculation for the input-only red test.
- Per-frame JSON contains options, context, candidate trajectories, selection
  costs, raw text/tokens, signed velocities and visual signal answers.
- Captured lens-test tensors/video insets contain **raw wide pinhole** renders.
  Replay must apply the recorded `camera_projection`; an empty option object is
  not a native-camera baseline on those tensors.

Recorder caveat: the automatic malformed-input capture at
`capture-selected-green-v2/inputs/frame_084.pt` contains already-projected images,
despite its original `wide_pinhole` context label. Replay that particular input
without another projection. Selected regular captures (000, 022, 057, 100) are raw
as documented. The recorder now preserves source images for automatic failure
captures too, preventing double remapping in future experiments.
The same caveat applies to `capture-selected-red-v2/inputs/frame_026.pt` and
`frame_037.pt`, whose process had loaded the old recorder. Its explicitly requested
frames 000, 038 and 065 are source images. The interrupted selected red run had no
automatic failure captures.

A restarted red test reproduced a larger initial history anomaly: a duplicated
penultimate position followed by a 0.593 m jump. Estimated initial speed was
7.675 m/s versus measured 2.952 m/s. A scoped `wait_for_snapshot` option and the
model-free `preroll_probe` were added to investigate this separately.

The repeated initialization tests now establish the following:

| Comparison | Original initialization | Experimental initialization |
| --- | --- | --- |
| Snapshot wait only | 5/10 bad histories | 5/10 bad histories: **did not fix it** |
| Acknowledged placement + exact-frame actor poses | 2/10 bad histories | **0/10 bad histories** |

All immediate snapshot frame numbers were current, even on bad histories. Thus
the earlier 71-tick freshness test did not rule out this separate pose problem.
The maximum pre-roll step error was 4.50 m in the native trials, versus 0.000014 m
with acknowledged commands. A bad history here means a pre-roll step error over
0.05 m or a final handover displacement more than 0.05 m from the usual 0.293 m.
`preroll-comparison.json`, `preroll-acknowledged-comparison.json`, and
`preroll-summary.json` preserve every trial.

The acknowledged variant uses CARLA's synchronous command batches for placement,
physics and velocity changes, and reads the ego pose from the matching frame's
actor snapshot. It changes both command acknowledgment and pose access; these
trials do not isolate which is sufficient alone. The first baseline-only driving
attempt was confounded by an overlapping launch (see table) and does not provide
a valid driving comparison. No driving-quality claim follows from clean
initialization alone.

After the overlapping-launch mistake, the probe acquired an exclusive process
lock before model loading or simulator setup. A second acquisition is rejected,
and closing the first releases it; both cases were checked. This prevents future
overlapping inference-probe processes, not unrelated simulator clients.

The red rerun also exposed a selection/controller interaction. At 2.2 s, the
selected trajectory's 1–1.5 s target was only 0.146 m/s but its full path length
was 7.50 m. Other native candidates stopped within about 0.79–1.14 m. After the
vehicle stopped, selecting long-tail plans engaged the original controller's
launch behavior and it resumed despite the visual checker still reporting red.
`prefer_stop_on_red` adds full path length squared to candidate cost only for a
visual red/yellow answer. An offline comparison using the **same previous plan**
and candidate arrays selected shorter stop plans at frames 022, 024 and 025;
see `red-stop-selection-offline.json`. The final live result is in the table above.
All 80 selected trajectories exactly matched an unmodified candidate. The visual
checker answered red for 75 planning ticks and unknown for five. Actual recorded
signals stayed red throughout. Median inference time was 6.60 s per tick.

`red-validation.json` calculates clearance using all corners of the actual vehicle
bounding box against the approach stop-line plane. This refines the earlier
front-offset approximation: input-only red crossed by 0.826 m; the first selected
red remained 0.253 m behind at 8 s; final stop-cost red remained 10.341 m behind.
`red-final-comparison/` contains measured plots and metrics. Videos:

- `red-before-after-8s.mp4`: previous selection vs stop-cost + acknowledged setup.
- `capture-selected-red-stop-cost-v2/000_verifai/overhead-h264.mp4`: full final run.

The exact final-run experimental sources are preserved in its `source_snapshot/`
directory, verified against the run manifest hashes. The subsequent launch lock
does not change inference behavior.

All 63 existing regression tests passed again after the code changes. Production
controller/configuration/tests and external model sources remained unchanged.
The final cleanup check found zero rollout vehicles and 40 sensors left after the
40 rapid initialization trials. This is consistent with freshly spawned collision
sensors being skipped by native cleanup's client-cache `is_alive` check; sensor
types were not inventoried before shutdown. The probe's
cleanup now destroys acknowledged actor IDs directly. The owned CARLA container
was stopped, removing all remaining simulator actors and freeing its resources.

## Resume and rollback

This round is complete; no tests remain running. Further validation should use one
common configuration on both red and green scenes, then add traffic and route
variation. Stop-cost selection currently stops much too early. The brief initial
red restart also warrants measuring applied controls against requested controls
before attributing it solely to inference. Malformed-generation filtering remains
unimplemented. Do not force away the late green-run stop without checking the
next junction and visible road controls.

Commands and option meanings are in [README.md](README.md). Exact tested option
objects are also committed under `local/experiments/presets/`; the first configured
scene is used by capture. Output folders
must be new so old evidence is not overwritten.

After committing/saving experiment work, `git switch feature/verifai-server-client`
restores the integration baseline; `git switch experiment/driving-inference-diagnosis`
returns to the first investigation. Outputs remain on disk. No production runtime
setting or model dependency needs reverting. Stop the owned simulator with
`bash local/server.sh stop` when finished. No changes have been pushed.
