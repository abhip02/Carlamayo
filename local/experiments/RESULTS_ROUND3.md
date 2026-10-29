# Driving validation — round 3

Branch: `experiment/driving-validation`, based on rollback checkpoint `4bec8f7`.
Previous findings: [round 1](RESULTS.md), [round 2](RESULTS_ROUND2.md).
Evidence root: `outputs/driving-investigation-3/`.

This round checks the same experimental inference settings across signal states
and seeds. The previous best green and red tests used different combinations.
Production driving files and default launchers remain unchanged.

**Current conclusion:** the combined inference wrapper is not a reliable fix.
The completed green validation stopped and turned off route despite clean initial
history. Stronger green-conditioned prefixes and eight samples did not recover
the saved failure inputs. The separate control-delivery correction addresses a
measured integration issue, not the model's choice of bad trajectories.

## Fixed settings and evaluation

The common settings are approximate equidistant camera projection, four native
trajectory samples, fixed motion seed 42, route/previous-plan selection, a periodic
visual signal question, full-path stop cost on visual red/yellow, acknowledged
pre-roll placement, and exact-frame snapshot waiting. Only the scene signal state
and language sampling seed change between the planned tests.

Green tests use the ego signal green and other signals in its group red. Red tests
freeze the signals red. Signal ground truth is recorded for evaluation; it is not
passed to the model or candidate selector. The original controller remains in use.

For green, check unnecessary stops, lateral deviation, and reaching the target
junction exit (about 50.85 m forward from the start). Report later stops separately
because another junction follows immediately. For red, check actual bounding-box
clearance, final stop position, any restart, and sustained holding. A stop 10 m
short of the line is not normal positioning even if it avoids a violation.

The only initial code change adds `context.applied_control`, sampled through
CARLA's `vehicle.get_control()` before each inference. Compare this with the
command recorded at the same CSV time. Agreement does not prove exactly when
the physics engine integrated the control, so this is a diagnostic rather than
a complete actuation-latency measurement.

## Run ledger

| Run | Duration | Status |
| --- | ---: | --- |
| `capture-common-green-43` | 12 s | Failed: near-stop at 2.0 s, minimum 0.0168 m/s; max lateral 31.03 m; forward progress 37.24 m; final heading change −135.48°. No collision or missing selected markers. |
| Green seed 42 | Not run | Prepared options only; deferred after common-settings failure |
| `capture-common-red-43` | 8 s | Failed sustained hold: stopped at 1.9 s and 6.1 s, restarted above 0.25 m/s at 3.2 s and 7.0 s. Final speed 2.181 m/s; front remained 0.833 m behind stop line. Two selected generations lacked markers. No collision. |

Exact option files are `common-green-43.json`, `common-green-42.json`, and
`common-red-43.json` under the evidence root. A process lock prevents overlapping
inference probes. No model-free simulator diagnostic is run beside a rollout.
The two executed common-setting options are also committed under
`local/experiments/presets/` for reproduction without the local output metadata.

The common-setting green run already disproves a general fix: it nearly stopped
at 2.0 s while the visual checker reported green. The saved-input comparison
in `green-prefix-variants.json` compared the native response with two short
green-conditioned reasoning prefixes and eight native samples. This is not a
live behavior change yet.

The eight frozen-input evaluations are complete in `replay-green-prefix/`.
Applying each variant to the same recorded previous plan gives the comparisons
in `green-prefix-comparison.json`. At frame 022, selected targets remained only
0.026–0.037 m/s. At frame 057, neither prefix supplied a straight candidate. Eight
samples fit in GPU memory (peaks 17.54/12.62 GiB) but the best route cost was still
1211 m² versus 1302 m² for four samples. These are failed local interventions and
were not enabled in live runs.

The initial pose and both pose-history tensors exactly match the earlier green
success. Raw images differ by mean absolute values of about 1.4–2.7 per channel on
the 0–255 scale, depending on camera (`initial-input-comparison.json`). Matching
initial state is therefore not a bitwise sensor replay. This does not isolate
the cause of the changed closed-loop behavior.

At 4.5 s, all four candidate route costs were about 397–448 m², with endpoints
roughly 43–46 m left of the ego. The selector cannot recover a straight path when
none of the candidates supplies one. `audit_capture.py` additionally found five
reported-control mismatches consistent with the previous tick's command. None
coincided with the early stop. A separate model-free control-delivery comparison
tested acknowledged command batches without modifying the production control
algorithm or applying different control values.

That model-free comparison completed: **7/40** native control commands differed
from the command requested for that tick, versus **0/40** with synchronous
acknowledged command batches (`control-delivery.json`). The alternating throttle
sequence was identical; it was not model-generated driving. `acknowledged_controls`
is now available as a scoped capture option, but is **not enabled** in the common
green/red validation runs and has not been validated as a driving-quality fix.

The completed green failure is visible in
`capture-common-green-43/000_verifai/overhead-h264.mp4` and compared with the earlier
isolated success in `green-validation-comparison.mp4`. Full metrics are in
`green-comparison-12s/`; these do not hide the failure with a short video crop.

The red run used the same inference options as the common green run, with only
the scene's signal state changed. All 16 visual checks answered red and all
recorded signals remained red, yet motion samples eventually all requested
substantial movement. For example, at 2.8 s the car was stationary but all four
sampled path lengths were approximately 18–23 m. A selector cannot choose a
stationary plan if none is sampled. The recorded driving text still said to stop.

The two malformed selected red generations occurred at 6.7 s and 7.6 s. Their
full source inputs were automatically captured with correct preprocessing
provenance. The raw arrays, text and control traces remain available for exact
replay. The final front position did **not** cross the stop line during the 8 s
observation; future crossing has not been measured.

Red artifacts are `red-validation.json`, `red-seed-comparison/`,
`red-validation-comparison.mp4`, and
`capture-common-red-43/000_verifai/overhead-h264.mp4`. The comparison video includes
the earlier seed-42 hold; these live episodes are not bitwise input replays.

## Final verification and remaining work

All 200 chosen trajectories were checked against their recorded native candidate
and against the trajectory actually written by the rollout controller path.
The CSV and snapshot poses/frame IDs agreed throughout both runs. Five reported
control mismatches occurred in green and two in red; the red brake mismatch at
2.3 s occurred during initial stopping, while the later restarts also had model
plans requesting movement. Neither observation establishes a single root cause
for all failures.

The existing 63 regression tests passed. Production controller/configuration and
controller tests still match upstream `4914558`; external model sources are
unchanged. Exact tested experimental sources are saved under each capture's
`source_snapshot/`, verified against the run manifest. The acknowledged-control
option was added after those sources were loaded and was not used in these runs.

The simulator was restored to asynchronous mode, with zero rollout vehicles and
zero sensors remaining, and the owned container was stopped. No tests are running.

Do not promote the earlier selected-run presets as a reliable fix. The useful
next checks are a matched live test of acknowledged control delivery, exact
replays of the saved malformed/unsafe plans, and a defined rejection/fallback
policy when every candidate violates the requested route or stopping requirement.
Such a fallback would be additional driving logic, not evidence that the model
itself has been repaired. No fallback or speed override was added in this round.

## External integration reports reviewed

[Alpamayo 1.5 issue 10](https://github.com/NVlabs/alpamayo1.5/issues/10) reports
backward/folded trajectories and ineffective navigation prompting in another
CARLA/NuRec integration. It has no demonstrated general fix in the fetched
discussion. This is corroborating context, not proof of our root cause.

[Issue 39](https://github.com/NVlabs/alpamayo1.5/issues/39) reports a different
CARLA collision case using the same pinned model revision and explicitly leaves
input/controller/model attribution unresolved. Its scene and controller differ
from ours.

A fork commit linked from issue 10 changes a fine-tuning loss path, not our
evaluation inference path. We did not apply it or change model dependencies:
[fork commit](https://github.com/ikutasama/alpamayo1.5/commit/ceba1495836d5be769f819f0b2e6c434f32e4eba).

## Rollback

After saving changes, `git switch experiment/driving-input-fixes` returns to the
previous experiment checkpoint. `git switch feature/verifai-server-client`
returns to the integration baseline. Local recordings remain available.
