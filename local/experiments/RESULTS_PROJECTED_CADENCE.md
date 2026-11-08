# CPU assessment of less frequent replanning — 2026-11-08

**No cadence policy was changed or run.** On the saved 114-decision NF4
T-junction trace, the evaluated conservative schedules find only 6, 8 and 11
eligible call slots at 0.2, 0.3 and 0.5 seconds. These are results for the
particular schedules evaluated, **not global upper bounds, live savings or
driving successes**. The best of these schedules has only about 38 seconds of
historical decision time attached to eligible slots, against 491.64 seconds
rollout wall time. This does not justify a new cadence GPU experiment on this
case while startup and left/tight driving validation remain unresolved.

## Why the existing interval helper cannot simply be enabled

`plan_interval_probe.py` belongs to the earlier native-path validation stack.
The current cap2 capture is `nf4_policy_pair → projected_probe →
RetainedSpeedProbe`, with route-projected steering. The old interval class is
not selected by that path, and the current strict policy adapter rejects an
undeclared additional option.

| Requirement | Older interval stack | Actual projected cap2 stack |
| --- | --- | --- |
| Cached object | `ModelPlan`, native world/corner arrays, `executing_plan` | Dict history, `execution_record/world/age`, different `ACTIVE` instance |
| Source identity | `selection.chosen/proposal/signal` | `selection.selected_proposal`, original `retention.source_context/source_proposal` |
| Geometry | Native predicted-body corridor, distance/heading and native-controller tracking forecast | Current footprint, projected target ahead/progress, remaining speed horizon and actual source age |
| Options/routes | `validation_*`; tracking supports left/straight only | `projection_cache_age_s`, speed preference/cap, gain 1.4; this case is `t_junction` |
| Signals | Scheduled reuse requires source/current positive green | Fresh inference first; cached substitution may then accept unknown in a verified empty-green scene |

Switching entrypoints would change the controller/selection validation, not just
cadence. A future projected-specific adapter would need to retain original world
anchors and source times, call the existing projected assessment, update the
projected `ACTIVE` state, and preserve per-tick route-phase/exit checks. It must
keep genuine native paths unchanged. Reused steps need explicit zero-native-call
records; control decisions, trajectory samples and VQA queries must be counted
separately. These bookkeeping and scheduling ideas are reusable; the old class
and its native-only geometry checks are not a drop-in replacement.

## Assessment method and limits

`assess_projected_cadence.py` loads saved JSON/CSV only, with CUDA hidden. It
reconstructed **all 114 original world anchors exactly** (maximum coordinate
difference zero), reproduced **all seven existing retained-plan checks**, and
verified the recorded route exit. No model, simulator, trajectory inference or
VQA call was run; CUDA remained uninitialized.

For each evaluated interval, the analysis advances through the **original
recorded vehicle states**. It permits a reuse slot only if the unchanged
projected geometry/age checks pass, the speed preference still holds, source
and current routes/phases agree, the exit has not been reached, the verified
empty-green scene conditions hold, and visual evidence remains fresh. Interval
age is measured from the original proposal time, never from the latest cache
selection. It keeps fresh inference on **all 23 original VQA refresh ticks**;
no query is skipped, postponed or borrowed from the future. Red/yellow never
permit reuse. The main evaluated schedules additionally require positive green
at both source and current tick, matching the old interval helper's conservative
signal requirement.

A source is available only if its original proposal tick would have been a
fresh slot in that evaluated schedule. The analysis never silently retrieves a
cached plan whose generation it has already skipped. If an observed selection
would require such a source, the result flags it and grants no reuse from that
unavailable selection. Further samples that a live run might need are unknown.

There are **100 unknown and 14 green** readings. All positive-green observations
are at indices 100–113, after the phase changes to `exit` at index 96. Thus the
conservative opportunity is concentrated in the final straight segment, not in
the turn itself.

| Evaluated interval | Eligible reuse slots | Remaining fresh slots | VQA refresh ticks retained | Recorded enclosing decision time at reuse slots |
| --- | ---: | ---: | ---: | ---: |
| 0.2 s | 6 | 108 | 23 | 20.91 s |
| 0.3 s | 8 | 106 | 23 | 27.45 s |
| 0.5 s | 11 | 103 | 23 | 37.86 s |

All selected sources in these conservative schedules are available. None of
these numbers measures achieved runtime: skipping a plan can change controls,
future vehicle poses, images, model responses and acceptance. The analysis does
not simulate that changed trajectory. Summed historical timer values indicate
where cost currently lies; they are not a promised speedup. The successful
source run itself has only 8.43 cm minimum corner clearance, so preserved
recorded-state eligibility is insufficient evidence for a live cadence change.

## Unknown-signal sensitivity is not an equivalent policy

The current projected pipeline can retain a plan on `unknown`, but only after
fresh native inference under the controlled empty-green scope. Preemptively
skipping that fresh observation is a different decision. The sensitivity below
keeps the same recorded scope, geometry, age and scheduled VQA checks but removes
the conservative positive-green gate. It is **not validated behavior or a
recommendation to treat unknown as green**.

| Evaluated interval | Candidate reuse slots | Fresh slots | Unavailable observed cached-source selections |
| --- | ---: | ---: | ---: |
| 0.2 s | 43 | 71 | 3 |
| 0.3 s | 63 | 51 | 6 |
| 0.5 s | 86 | 28 | 4 |

These larger counts cannot be converted into credible live savings: the
unknown-signal policy differs, the hypothetical vehicle states are unobserved,
and some original selected sources would never have been generated. In
particular, unavailable proposal 11 affects the first two schedules, and
unavailable proposal 34 affects the latter two. No missing plan is fabricated.

## Recommendation and resource observation

Do **not** launch a cadence GPU experiment on this T-junction now. The evaluated
green-only opportunities cover at most 11 slots among these three schedules;
even subtracting their historical cost optimistically would leave roughly
7.6 minutes for 11.4 simulated seconds. It is not a solution to the current
runtime complaint. Any larger reduction needs separate evidence for the
changed unknown-signal decision or model-inference optimization, with fresh
closed-loop validation; neither is implemented here.

After the numerical assessment, one bounded read-only observation at
**2026-11-08 08:17:16 UTC** found model GPU3 at **16.014 GiB free** and renderer
GPU0 at **8.650 GiB free**. The fixed pair still could not meet its 11.0 / 9.8 GiB
thresholds because renderer memory was insufficient. This instantaneous reading
is not a sustained gate or reservation. No further polling or GPU launch was
part of this assessment.

Reproduce from this worktree with a fresh output directory:

```bash
CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 \
  .venv/bin/python -m local.experiments.assess_projected_cadence \
  outputs/nf4-policy-pilot-20261107-v4/09-t-junction-nf4/09-t-junction-nf4-cap2 \
  --output outputs/projected-cadence-screen-20261108-v2
```

The existing v2 output is preserved; choose another name to reproduce it. Version
2 has exactly the same numerical assessment as v1 and expands input hashing to
include all controller records, the CSV and summary. It is not another driving
trial. The single memory observation was copied, not repeated. Compact evidence
is tracked in `artifacts/projected-cadence-assessment-20261108.json`; detailed
per-tick reasons remain in the output's `detail.json`.
