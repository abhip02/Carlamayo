# Bounded low-speed resampling — round 14

Branch: `experiment/driving-low-speed-resampling`.
Rollback checkpoint: `f73b1f5` on `experiment/driving-phase-transition`.
The round-12 five-case benchmark remains preserved and deferred.

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

154 experiment tests pass in 3.75 s, including 16 new cases covering default-off
behavior, red/yellow stopping, green/unknown retries, better/worse/invalid later
pools, original-array preservation, the pool limit, invalid thresholds and
metadata tampering. The first test run exposed a fixture mismatch: the reused
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
results. Live results are pending at this checkpoint.
