# Driving latency investigation — 2026-11-07

**Driving is not fully fixed or established as reliable.** The accepted videos
remain controlled, map-assisted demonstrations. The narrow T-junction margin,
cost of repeated sampling, limited starts/seeds and lack of surrounding traffic
remain unresolved. This investigation prepares lower-cost selection; it has
not yet validated that selection in fresh closed-loop driving.

Worktree: `/scratch/apomalap/thesis/carlamayo-latency`, branch
`experiment/driving-latency-validation`, starting at `e1f38d6`. The original
checkout's in-progress FP8/raw-model work was not edited. Controller, simulator,
model weights, ten-scene manifest and standard launcher defaults are unchanged.

## Measured cost of the accepted BF16 collection

The ten captures executed 1409 control decisions, **2326 trajectory samples**
and **286 separate visual signal queries**. Native inference, including those
signal queries, consumed 98.96 minutes; recorded rollout wall time was
112.43 minutes for 140.9 simulated seconds. Initial model loading and later
auditing/video conversion are additional costs. This is not real-time driving:
CARLA waits for inference before advancing its next 0.1 s tick.

The table uses the actual decision timings in `ego.csv`, including selector
overhead. It is historical selected-success data, not a fresh benchmark.

| Case | Samples / decision | Decision p50 / p95 (s) | Wall seconds per simulated second |
| --- | --- | --- | --- |
| Straight | 1.22 | 2.54 / 4.36 | 37.5 |
| Left turn | 1.10 | 2.33 / 3.64 | 31.8 |
| Right turn | 1.40 | 2.40 / 13.54 | 40.8 |
| Left curve | 1.30 | 2.57 / 13.97 | 39.9 |
| Right curve | 1.38 | 2.43 / 13.98 | 39.8 |
| Roundabout first exit | 1.31 | 2.37 / 14.72 | 38.4 |
| Roundabout second exit | 1.32 | 2.43 / 13.93 | 41.6 |
| Tight bend | 3.72 | 14.14 / 16.93 | 101.2 |
| T-junction | 1.67 | 2.48 / 14.44 | 47.9 |
| Red stop | 1.00 | 1.95 / 3.71 | 28.6 |

All ten historical clips pass their recorded driving/integrity checks. Every
decision uses map-projected steering. 518 decisions substitute a validated
older model plan; the red guard holds the achieved stop for 98 decisions.
There is no fabricated map-generated trajectory substitute, but these are
controller-assisted results, not raw VLA trajectory-quality scores.

Detailed timings, route completion, collisions, footprint departures, clearance,
reuse and hold counts: [baseline-performance.json](../../outputs/latency-screen-20261107-v2/baseline-performance.json).

## Why the tight bend is expensive

111 of its 204 decisions exhaust all six attempts. The preferred 3–5 m/s band
keeps searching even when a checked native plan is moving the vehicle at a
lower speed. Across the collection, a trajectory call without a new signal
query has median duration 2.325 s; with a signal query, 3.610 s. Repeating those
calls explains much of the long tail. Counting only one selected prediction
per tick would hide this work.

A shorter language-token cap was considered but **not applied**. Six valid
trajectory-marker outputs exceed 96 generated tokens, including a 219-token
red-stop prediction. Only two 256-token calls lack the marker; together they
consume 23.23 s of 5937.52 s of recorded inference. Blind truncation could discard
valid stop predictions and would not address the dominant repeated sampling.

## Changes prepared and tested

- `--speed-preference-mps LOW HIGH` exposes the existing native ranking band.
  It does not clamp or fabricate speeds. The existing default stays 3–5 m/s.
- `--speed-selection-max-seconds SECONDS` optionally stops starting additional
  retries after the budget. It counts elapsed selection time, including calls
  and selection overhead. **It cannot interrupt an inference already running**;
  one call may exceed the budget. Defaults remain unrestricted in wall time.
- Each retry records its elapsed start time; selection records the stopping
  reason and elapsed total. The independent audit checks that no additional
  retry starts after an enabled budget. All sampling time remains counted.
- The best valid native or independently revalidated cached plan is returned.
  An all-invalid budget raises an error; visual red/yellow keeps its native stop.
  The red mission still uses exactly one native sample and no cached substitute.

The budget changes pass 290 experiment tests (274 existing plus 16 new cases). Tests cover
budget overshoot by an in-progress call, rejection of invalid paths, preserved
visual stops, retained-plan selection, malformed budgets, independent detection
of an illegal late retry, and unchanged defaults. That run took 6.43 s, with one
existing Transformers environment-variable deprecation warning.

## Saved-state policy screen — completed, not a driving test

The screen runs the actual selector and cache checks on recorded ego states and
saved native outputs. Its baseline reproduces all **1409 selected trajectories
and original anchors**, using all 2326 saved calls. Alternative histories contain
only proposals requested by that policy. If another proposal was never saved,
the case stops and records incomplete coverage. No new prediction is invented.

The first screen stopped on an input-provenance assertion. A retained frame
contains its source instruction, which may precede a navigation phase change.
Version 2 reads the current instruction from the decision's fresh first
proposal. Context, instruction, text seed and diffusion seed assertions remain
enabled. The first log is retained; it was a screen bug, not a driving failure.

| Policy | Complete recorded cases | Covered decisions | Interpretation |
| --- | --- | --- | --- |
| Original six attempts, 3–5 band | 10/10 | 1409/1409 | Exact selection/anchor reproduction |
| Two attempts, 3–5 band | 3/10 | 567/1409 | Missing saved proposals prevent evaluating the rest |
| Two attempts, 2.5–5 band | 9/10 | 1308/1409 | Left turn stops at step 3 because the needed proposal was not captured |

For the **same nine fully covered cases**, six-attempt baseline versus
two-attempt/2.5–5 screening uses **2212 versus 1348 calls**, and **94.22 versus
60.06 minutes** of summed historical call time. These are workload estimates,
not measured acceleration or a success rate. A cached history change alters
which proposals future decisions request.

On the tight bend, calls fall from 759 to 218; reconstructed inference-only
p50/p95 falls from **13.43/16.24 s to 2.49/4.64 s**. However, **145/204 chosen
plans change**, and the lowest selected preview becomes 1.70 m/s. Future
physics/images were not regenerated, so these numbers cannot establish lane
keeping, speed compliance or completion under the new policy.

The two-attempt original-band T-junction screen preserves all 112 chosen plans
and anchors while reducing calls from 187 to 127. The relaxed-band screen
changes ten plans. Both red-stop screens preserve all 120 decisions and calls.
These observations prioritize live tests; they do not replace them.

[Full screen and per-decision records](../../outputs/latency-screen-20261107-v2/report.json)
· [Source snapshots and provenance](../../outputs/latency-screen-20261107-v2/provenance.json).
The [compact numerical summary](artifacts/latency-screen-20261107-summary.json)
is tracked in Git; large local captures remain ignored.

Reproduce without GPU inference or CARLA:

```bash
.venv/bin/python -m local.experiments.sampling_trace_screen \
  --source-root /scratch/apomalap/thesis/carlamayo \
  --collection /scratch/apomalap/thesis/carlamayo/outputs/driving-investigation-19-showcase/results.json \
  --output outputs/NEW_LATENCY_SCREEN
```

## Frozen fresh pilot — prepared, not started

[latency-pilot.json](presets/latency-pilot.json) records the manifest hash,
complete case definitions, fixed settings, exact eight commands, thresholds,
resource leases and timeout policy. The comparison is one fresh baseline and
one fresh candidate at each of **left turn, tight bend, T-junction and red stop**.
Left is included because its missing saved proposal leaves the screen incomplete.

Baseline: six attempts, preferred 3–5 m/s. Candidate: two attempts, preferred
2.5–5 m/s. All other generation/controller settings stay fixed. The optional
wall-time budget is tested in software but is **off in this pilot**, isolating
the call-count/band comparison from runtime-dependent decisions.

Report every attempted run, including capture errors, timeout and failed driving
audits. Preserve the original route/speed/footprint/collision/red-stop thresholds.
Candidate targets are at most two moving samples per decision, one in red,
p95 full decision time <=6 s on an idle A5000, and aggregate wall/sim ratio <=70%
of baseline (tight bend <=50%). These are acceptance targets, not achieved results.
Then test held-out seeds/starts before changing the standard defaults.

Resource blocker: no A5000 has enough idle memory for the existing ~21.6 GiB
BF16 inference setup. Other users' jobs remain untouched. The quantization task
is independently using GPU1; it released its renderer on port 2000. No live
rollout or model job from this latency branch is running.

Before launching, coordinate with the benchmark/monitor tasks, select idle or
reserved model/render GPUs, and acquire nonblocking `flock` leases on the two
GPU UUID files and `carla-port-2000.lock` in
`/scratch/apomalap/thesis/.carlamayo-resource-locks/`. Recheck occupancy and port
availability after obtaining leases; hold them for the pilot. Locks coordinate
our tasks, not unrelated users' schedulers. Never kill other jobs or containers.
Use owned container `apomalap-carlamayo-latency-20261107`.

The executable pilot driver enforces these leases and checks GPU memory/ports
again before every job. It refuses occupied devices, requires a separate idle
renderer, verifies the frozen source hashes, preserves failures and stops only
its owned workers/container on timeout. It also requires at least 4 GiB free.
Final verification is **300 tests passed in 6.17 s**, including lease contention,
occupied-device/port refusal, timeout cleanup, preservation of failed outcomes,
and frozen-source checks. No GPU inference is required for these tests.
After setting both GPU UUIDs, first check readiness, then launch the frozen jobs:

```bash
.venv/bin/python -m local.experiments.run_latency_pilot \
  --check-only --output outputs/latency-pilot-20261107
.venv/bin/python -m local.experiments.run_latency_pilot \
  --output outputs/latency-pilot-20261107
```

Execute the frozen jobs in order, retaining nonzero exit codes and partial
outputs, with no substituted historical passes. The full matched pilot is
estimated at 70–90 minutes once resources are available, primarily because it
repeats the expensive six-sample baseline. No GPU start-time ETA is available.
No model copies were made; new screen artifacts occupy only a few MiB.

Rollback: use the original checkout/branch or `e1f38d6`; the standard launcher
still uses the accepted six-attempt profile. This candidate is not promoted.
