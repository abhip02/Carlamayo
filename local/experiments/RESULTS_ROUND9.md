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

These are offline validation findings, not simulator continuation results.
Physical replays must complete after the active round-8 speed-preference run
releases the simulator before this extension is tested live.

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
still pass the updated retention audit. Physical replay and live results
are pending; no completion claim is made.
