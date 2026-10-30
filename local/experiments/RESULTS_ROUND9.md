# Originally admitted native candidate reserves — round 9

Branch: `experiment/driving-native-candidate-reserve`.
Starting checkpoint: `693c95f` on `experiment/driving-tracking-validation`.
Previous evidence: [round 8](RESULTS_ROUND8.md).
Return to `693c95f` to remove this extension. No production launcher, controller,
model decoder, model weights or default configuration changes.

## Motivation

The previous cache retained only each inference step's selected fresh sample.
An offline audit of other **originally admitted** samples found a currently
valid alternative in two terminal failures, under the unchanged half-second
age limit, original world anchors, signals, phases and current physical checks:

| Capture terminal | Original unselected sample | Age | Native clearance | Tracking clearance | Current native target |
| --- | --- | --- | --- | --- | --- |
| Strict 20 cm, 7.7 s | frame 075, pool 0, candidate 0 | 0.2 s | 0.263050 m | 0.211839 m | 3.777970 m/s |
| Prefer 20/minimum 10 cm, 7.2 s | frame 071, pool 3, candidate 4 | 0.1 s | 0.283555 m | 0.168853 m | 7.011264 m/s |

Both candidates were admitted at their original generation time. The second
fails optional speed/reserve preferences but passes hard admission. The
minimum-10 run with an unknown visual signal has no eligible alternative
under unchanged rules. Evidence and exact script:
`outputs/driving-investigation-8/unselected-recent-candidates-terminal-audit.json`,
`check_unselected_cache.py` in that evidence directory.

These initial findings were offline validation only. Both physical replays
have now completed; their results and the separate live test are recorded below.

## Isolated implementation

`candidate_reserve_probe.py` is a separate opt-in entrypoint. The normal fresh
sampler and selected-plan cache run first, unchanged. Admitted but unselected
candidates are retained as CPU copies and archived under `reserved_*.json`.
Originally invalid candidates never enter this reserve. Every source keeps
its original proposal, native trajectory, velocity-derived speed array,
orientation, generated tokens, reasoning, context and timestamp.

Only `NoValidatedPlan` after both ordinary mechanisms fail triggers reserve
selection. Every remaining alternative is rechecked through the same current
native geometry and controller-tracking screen, with the same signal/phase
rules and maximum age of 0.5 s. Selection uses the existing ranking. The
selected native world path and original elapsed time go to the unchanged
controller; no reanchoring, resampling of old timestamps, interpolation,
map-generated path or speed override occurs.

Both the fresh-sampling failure and selected-cache failure are preserved in
`replan-rejections/` before terminal-only markers are removed. Executions are
explicitly labeled `fresh_and_selected_cache_exhausted`. A chosen alternative
then joins the existing selected-plan history at its **original** time.
Unrelated model errors and invalid configuration are never hidden. If the
reserve also fails, the experiment terminates and preserves all failures.

The entrypoint snapshots its own source and hashes its policy, in addition
to the original capture snapshots. The retention audit independently checks
that any used reserve candidate was originally admitted and still matches
its original proposal's native array, token IDs and context.

## Verification before live use

Combined suite: **166 passed in 4.81 s**. New tests cover original admission,
exclusion of the already-selected candidate, exact tensors/tokens/text,
immutable source anchors, real age and expiration, current signal/tracking
rejection, preservation of failure evidence, unchanged parent success,
propagation of unrelated errors, and restoration of process-local wrappers.
Two mismatch cases reject incorrect source-proposal/context provenance.
Real archived candidates from both failures also reconstruct exactly.

The additional reserve-execution assertion passes the native retention auditor;
the focused suite is **9 passed in 2.20 s**. Both earlier completed captures
still pass the updated retention audit. No full-route completion claim is made.


## Physical replay results

Both runs reproduce the complete original prefix and controls exactly, then
use the archived native candidate with its original world anchor and age.
These are model-free diagnostic continuations, not fresh-inference rollouts.
Each holds a candidate for 1.5 s to check tracking, beyond the live cache's
unchanged 0.5 s age bound; this does not authorize longer live retention.

| Original candidate | Prefix | Final time | Total left rotation | Minimum actual clearance | Final speed |
| --- | --- | --- | --- | --- | --- |
| Strict 20 cm, frame 075 / pool 0 / candidate 0 | 7.7 s | 9.2 s | 80.5512 deg | 0.207976 m | 5.0040 m/s |
| Prefer 20/min 10 cm, frame 071 / pool 3 / candidate 4 | 7.2 s | 8.7 s | 74.2850 deg | 0.130532 m | 5.1267 m/s |

Both terminate at the diagnostic duration with every recorded corner inside.
Neither reaches the route endpoint. Evidence under `outputs/driving-investigation-9/`:
`replay-unselected-strict20-frame075-candidate0/` and
`replay-unselected-preferred-frame071-candidate4/`, including native provenance,
full actual states, prefix errors, controls and videos.

## Live reserve test result

`capture-reserve-native-four-left-margin20-43` uses the unchanged strict-20 cm
preset, original controller, original model, eight samples per pool and at
most six pools. The new reserve mechanism is the only policy addition. It
retains the 0.5 s maximum age and all current physical/signal/phase checks.
Duration cap 20 s, same route-exit stopping condition. Started at `370182a`.
The run ends at **7.4 s**, **45.9497 degrees** left, with no route exit.
Actual minimum speed is **2.7880 m/s**, final speed **4.4569 m/s**, and minimum
recorded body clearance **0.285851 m**. Wall time is **1564.7 s (26.08 min)**.
There are **zero reserve rescues**: at termination the three remaining
originally admitted alternatives all fail current native-corner clearance,
as do the three selected cached plans. This is a negative live result, not
evidence that the added reserve improves full-run behavior. Different camera
pixels and resulting earlier selections prevent treating it as an exact paired
counterfactual of the older strict-20 run.

All 74 executed native arrays, anchors, speed arrays and ages match sources;
all controller commands and tracking forecasts independently reproduce with
zero error. There are 48 selected-cache uses (44 better-ranked, four after
fresh-sampling exhaustion), 70 fresh accepted steps, 139 pools, 585 native-
geometry admitted candidates screened, and 130 tracking rejections. Fifteen
visual checks report green. At terminal, 32/48 candidates fail body geometry,
six fail native reserve, and the ten geometry-admitted candidates all fail
tracking clearance. Five post-admission retry steps add 18 pools and change
five fresh choices; three optional speed preferences remain unmet.

Evidence: `reserve-left-margin20-{control,footprint,route,retention,quality,tracking}-audit.json`
in the round-9 output directory. Actual control/snapshot mismatches are zero.
Both `retention-failure.json` and `candidate-reserve-failure.json` are retained.

Reproduction:
```bash
source local/env.sh
.venv/bin/python -m local.experiments.candidate_reserve_probe capture \
  --config local/configs/demo.yaml --duration 20 \
  --capture-frames 0 19 40 50 60 64 68 74 77 80 90 100 110 120 130 \
  --options local/experiments/presets/tracking-native-four-left-quality-margin20-43.json \
  --output outputs/driving-investigation-9/capture-reserve-native-four-left-margin20-43
```


## Verified-prefix continuation harness

`prefix_resume_probe.py` is an opt-in diagnostic entrypoint layered over the
candidate reserve. It replays previously executed native arrays through the
unchanged controller, then begins fresh model sampling at a configured frame.
It requires identical scene configuration, original controller/config bytes,
route and render/signal settings. Before application, every replayed controller
command must match within 1e-8. Every prefix vehicle state, including the resume
boundary, must match within 1e-6 m/degrees/m/s. Any mismatch aborts continuation.

Source proposals and selected records retain original inference metadata and
are copied with SHA-256 provenance. Replayed records additionally identify the
new physical context and explicitly say no fresh inference occurred. Histories
contain original native candidates and original ages. No map path is executed.
The summary separates replayed planning steps from completed fresh-inference
steps. A new image-based signal check is forced at resume; camera images are
new renders and are not claimed to be bit-identical.

Options `replay_prefix_capture` and `replay_prefix_seconds` specify the source
and boundary. `replay_prefix_only: true` verifies the physical prefix and stops
with `prefix_verification_complete`, never route completion. This supports
faster controlled continuation diagnostics without claiming a new full run.

Nine unit tests pass in 2.32 s. They cover native array/text preservation,
no model call before the boundary, fresh signal refresh, mismatch rejection
(states, controller commands, source arrays, incomplete data, route and scene
configuration), verify-only termination and wrapper restoration. An offline
check reconstructs all 120 steps / 49 selected sources of the longer left run
and archives 533 source files with exact native anchors, speed arrays and ages.
Physical prefix verification is tracked in the subsequent round-10 experiment.


## Offline older-candidate follow-up

While the round-10 full run occupies CARLA, an offline diagnostic rechecks all
originally admitted candidates from the preceding **1.0 s** at this run's
terminal 7.4 s state. Only this diagnostic's age limit changes; native clearance
and tracking remain the original strict 20 cm, with original anchors, signal,
phase, speed arrays, distance/alignment and controller state.

Of 32 candidates, **one passes**: originally unselected `frame_067_pool_0`,
candidate 0, age **0.7 s**, native clearance **0.343161 m**, tracking clearance
**0.213240 m**, native target **2.737504 m/s**. It was also admitted at generation
(native clearance 0.396492 m, tracking 0.229348 m). Distance from the current
rear axle to its original world path is 0.092931 m; heading difference is
1.73842 degrees. None of the candidates inside the current 0.5 s limit passes.
The other 31 fail native clearance (29) or tracking (two).

This is **offline only**, not a live age-limit change or a physical replay.
The active full test retains 0.5 s. A physical continuation must test the older
candidate before any longer-retention experiment is considered. Evidence in
`outputs/driving-investigation-10/`: `check_older_admitted_candidates.py` and
`older-admitted-terminal74-audit.json`.


### Older admitted candidate: physical replay completed

The queued round-10 replay `replay-older-strict74-frame067-candidate0/` reproduces
all 7.4 s of the strict run's states and original-controller commands exactly,
then holds the unchanged frame-067/pool-0/candidate-0 native path for 1.5 s.
Every source array remains exact. It reaches **8.9 s** without a recorded body
departure, final speed **2.33675 m/s**, yaw **18.2935 degrees**, final x **3.1110 m**.
Minimum continuation clearance is **0.199018 m**, final **0.588952 m**. The
forecast's 0.213240 m minimum therefore overstates the actual minimum slightly;
this replay is not evidence that the 0.2 m forecast provides a physical bound.
The maximum applied-control representation difference is 2.89e-8.

This diagnostic deliberately starts with a 0.7 s old sample and holds beyond the
live age cap. It uses no fresh inference and does not complete the route. The
live 0.5 s age limit is unchanged. The result supports further study of older
native alternatives, but no age-limit extension has been adopted.
