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

## Live reserve test in progress

`capture-reserve-native-four-left-margin20-43` uses the unchanged strict-20 cm
preset, original controller, original model, eight samples per pool and at
most six pools. The new reserve mechanism is the only policy addition. It
retains the 0.5 s maximum age and all current physical/signal/phase checks.
Duration cap 20 s, same route-exit stopping condition. Started at `370182a`.
No result is claimed until the run and independent execution audits finish.

Reproduction:
```bash
source local/env.sh
.venv/bin/python -m local.experiments.candidate_reserve_probe capture \
  --config local/configs/demo.yaml --duration 20 \
  --capture-frames 0 19 40 50 60 64 68 74 77 80 90 100 110 120 130 \
  --options local/experiments/presets/tracking-native-four-left-quality-margin20-43.json \
  --output outputs/driving-investigation-9/capture-reserve-native-four-left-margin20-43
```
