# Bounded low-speed resampling — round 14

Branch: `experiment/driving-low-speed-resampling`.
Rollback checkpoint: `f73b1f5` on `experiment/driving-phase-transition`.
The round-12 five-case benchmark remains preserved and deferred.

**Result: the baseline stops and fails at 10.1 s; the retry-enabled continuation
reaches the endpoint at 10.7 s, with minimum fresh-continuation speed 3.015 m/s.**
Both replay the same verified 9.0 s history before fresh inference. This is a
targeted continuation, not a full fresh-run validation. No recorded vehicle
corner leaves the lane corridor.

[Side-by-side video](../../outputs/driving-investigation-14/slowdown-comparison.mp4),
[enabled continuation](../../outputs/driving-investigation-14/resume-slow-enabled-review.mp4),
[baseline](../../outputs/driving-investigation-14/resume-slow-baseline-review.mp4),
and [measured speed/clearance](../../outputs/driving-investigation-14/slowdown-speed-clearance.png)
([PDF](../../outputs/driving-investigation-14/slowdown-speed-clearance.pdf)).

## Hypothesis and scope

Round 13 found that the late slowdown in straight case 03 is not limited to the
controller: native predictions themselves fall toward zero despite the controlled
empty, all-green scene. At 9.0, 9.5 and 9.7 s, the first admitted pool contains
slower choices than later pools on identical frozen inputs. VQA reports `unknown`
at these states. The existing loop stops at the first admissible preferred-reserve
pool; reranking only that pool cannot choose alternatives it never generated.

## Opt-in change

`validation_resample_below_target_mps: 3.0` requests further native pools when the
best admitted sample is below 3 m/s and the actual visual label is green or
unknown. This is a bounded retry preference, not a minimum admitted speed. The
unchanged ranking can still prefer a slower plan. The existing six-pool budget,
seeds, language resampling and temperature schedule apply. If later pools are
worse or invalid, the earlier best native sample survives. Red/yellow request
no low-speed quality retry. This option is disabled by default.

Paths, speeds, source ages/anchors, controller, weights, decoder and production
launchers are unchanged. All existing geometry, native-body, tracking and cache
checks remain. This experiment is scoped to the controlled empty scene; it is
not an obstacle-aware policy for arbitrary unknown signals. Each decision logs
the threshold, whether it applied, whether it was met, and individual retry
requests. The independent audit recomputes these from archived pools and checks
native identity and unchanged ranking.

## Validation before live execution

155 experiment tests pass in 3.87 s, including 17 new cases covering default-off
behavior, red/yellow stopping, green/unknown retries, better/worse/invalid later
pools, original-array preservation, the pool limit, invalid thresholds and
metadata tampering. The final test revision also ensures every later candidate
fails in the invalid-pool case and verifies that an already-met preference uses
one pool. Live implementation stayed frozen at `080e8e6`.
The first test run exposed a fixture mismatch: the reused
fixture has no body-clearance measurements, while the full capture audit expects
them. The tests now call the independent retry-evidence checker directly; full
body and ranking audits are performed on archived pools and live captures.
No production code or audit requirement was weakened to satisfy the fixture.

Frozen-loop diagnostic: `outputs/driving-investigation-14/check_frozen_policy.py`
feeds the original round-13 pools through the actual retry loop and unchanged
tracking screen at all three states, with the option off/on. No new model output
or physical driving is generated. All six independent audits pass. Results are
retained in `frozen-policy-results.json`:

| State | Baseline target | Retry target | Pools off/on | Preference met |
| --- | --- | --- | --- | --- |
| 9.0 s | 2.871726 m/s | 3.949724 m/s | 1 / 2 | Yes |
| 9.5 s | 1.574791 m/s | 2.638344 m/s | 1 / 6 | No |
| 9.7 s | 0.346354 m/s | 2.222282 m/s | 1 / 6 | No |

The latter two retain below-preference native plans under the unchanged ranking,
despite other admitted candidates exceeding 3 m/s. The option is working as a
retry preference, with no forced speed floor or change to ranking.

## Predeclared live comparison

Original scene: round-12 case 03, `03-straight-opposite-offset.yaml`. Both jobs
replay and verify every original command/state through 9.0 s, then resume fresh
sampling until the same route endpoint or a 12 s total cap. The original five-case
benchmark's 10 s cap is not changed. Baseline runs first; only the new retry
option differs. Neither job enables the unrelated left-phase option.

Options and runner: `outputs/driving-investigation-14/resume-slow-{baseline,enabled}.json`
and `run_continuations.py`. Boundary camera inputs are retained because rendering
may differ even with identical physical history. All outcomes, including failures,
are retained. These are targeted verified-prefix continuations, not full fresh-run
results. Both runs and all independent audits completed at launch commit
`080e8e6`; the owned CARLA server was stopped successfully.


## Matched physical results

| Measurement | Baseline | Low-speed retry enabled |
| --- | --- | --- |
| Outcome | Stops, then no admissible plan | Route endpoint reached |
| Final simulation time | 10.1 s | 10.7 s |
| Verified replay / fresh execution | 9.0 / 1.1 s | 9.0 / 1.7 s |
| Fresh executed commands | 11 | 17 |
| Final measured speed | 0.000413 m/s | 3.631380 m/s |
| Minimum fresh-suffix speed | 0.000413 m/s | 3.014829 m/s |
| Minimum corner clearance strictly after 9.0 s | 0.741433 m | 0.739368 m |
| Fresh pools / native candidates | 17 / 136 | 47 / 376 |
| Hard-admitted candidates | 81 | 95 |
| Wall time including capture startup | 277.965 s | 647.178 s |

Both prefixes reproduce with zero position/orientation error and maximum
float32 context-speed delta 4.234619e-7 m/s. All 101 baseline and 107 enabled
commands pass the original-controller/native-source checks, with the replayed
90 commands explicitly excluded from fresh sampling counts. Tracking, retention,
snapshot, body and post-admission ranking audits pass. The added audit also
passes on the prior round-12 straight capture (98 fresh sources) and round-13
left continuation (97 fresh sources), without changing those captures.

Only `validation_resample_below_target_mps` differs between option files; the
source hashes and launch commit match. Boundary poses, instructions, history
XYZ/rotations and camera indices are identical. Cameras are not pixel-identical:
mean baseline/enabled absolute RGB difference is 0.699926 out of 255, affecting
16.95% of channels. The comparison with the prior round-13 frozen export differs
by 0.318142 out of 255. See `boundary-input-comparison.json` for exact measurements.
The reproduced baseline failure and same-input frozen-pool comparison support
the local result without asserting pixel-identical live inputs.

The baseline first selects 2.870457 m/s at 9.0 s, then slows to 0.732329 m/s at
10.0 s and essentially zero at 10.1 s. All 48 terminal candidates fail reverse
motion; 44 also fail alignment. Other body/center failures overlap. The native
reasoning frequently still says the lane is clear, including several near-stop
predictions, so prose is not a reliable explanation of commanded speed.

The enabled run first selects 3.948348 m/s from pool 1, then reaches the endpoint
at x=-5.913718, y=149.457428, yaw=89.584175 degrees. There are 11 additional-pool
requests attributed to the low-speed preference and six changed fresh selections
after first admission. Four accepted decisions remain below the preference after
budget exhaustion. Executed native targets span 2.402415–4.868453 m/s; no target
is raised to the threshold. At 10.0 s, only one of 48 candidates passes all hard
checks, so the loop retains its 2.402415 m/s target. Physical speed subsequently
recovers and the continuation finishes. No cached plan is executed in either
fresh suffix; this comparison therefore does not depend on the phase or signal
cache exceptions.

The enabled run uses 576.171 s of recorded pool inference for 1.7 s of fresh
simulation, plus visual checks/startup/other overhead. Increased sampling is
expensive and does not make the model reliable. The unchanged six-pool limit
bounds that expense per planning step. No real-time capability is demonstrated.

## Evidence and limitations

`matched-continuation-results.json`, per-capture `*-audit.json`, options, logs,
source snapshots, native pool JSON and the comparison scripts are retained under
`outputs/driving-investigation-14/`. Video banners distinguish the replayed prefix
and fresh continuation. The videos and corrected full-range plot were visually
reviewed. The yellow 6.4 s predicted path is not a claim that its entire future
passes the shorter admission/tracking checks. Closed snapshots were losslessly
gzipped with SHA-256 verification; `compressed-inputs.json` records each archive.

This addresses one observed slowdown from a particular state in the controlled
empty junction. A full run with fresh inference from the beginning, repeatability,
the other round-12 failures, and the deferred five-case rerun remain outstanding.
Model prose still sometimes describes absent vehicles or unsupported turns.
The policy does not resolve arbitrary traffic, stop-sign perception, obstacles,
or full-future path validity. Production defaults remain unchanged.

To revert only this experiment, disable/omit the low-speed option or return to
`f73b1f5` on `experiment/driving-phase-transition`. To also remove the preceding
phase experiment, return to `b44853e` on `experiment/driving-exit-signal-transition`.
All source arrays, weights, decoder, controller and production launchers are
unchanged. The external model checkout remains clean.
