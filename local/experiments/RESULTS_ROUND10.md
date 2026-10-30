# Unknown-signal exit-cache validation — round 10

Branch: `experiment/driving-exit-cache-validation`.
Starting checkpoint: `0a85708` on `experiment/driving-native-candidate-reserve`.
Return to that checkpoint to remove this policy addition. Production launchers,
controller, parameters, decoder and weights remain unchanged.

## Evidence and scope

The longer round-8 live left turn ends at 12.2 s, about 9 m before the route
endpoint, after all 48 new samples fail native body geometry. Its five recent
selected native plans pass the current physical checks, but the existing cache
rejects them because both original and current image-based signal answers are
unknown. See [round 8](RESULTS_ROUND8.md) for the failed wording comparison,
input continuity audit and explicitly offline gate-isolation results.

This is a controlled empty-scene experiment. It does not solve signal
perception, establish that unknown means green, or model obstacles. New samples
were already eligible under unknown signals; this experiment changes only a
narrow condition for revalidating previously accepted native samples.

## Optional rule

`validation_allow_unknown_exit_cache: true` permits a cached sample only when:

- Both original and current signal labels are **unknown**, and image-based
  signal verification is enabled. Green-to-unknown transitions, red and yellow
  remain ineligible under this exception.
- Both navigation phases are `finish_turn`, and the configured route is left.
- The original and current actor positions are inside the route center corridor
  and past its entry; both actor headings and local route headings are within
  10 degrees of the route's exit heading.
- Every existing age, remaining-path, native orientation/body clearance,
  controller-tracking, reverse-motion and alignment check still passes.

The age limit remains 0.5 s. Neither the source record nor signal label is
rewritten. Ranking retains the existing unknown-signal speed preference; no
speed floor, target override, new path, repaired waypoint or map path is supplied.
The existing cache's minimum-progress admission check is also retained.
`signal_reuse` records the rule, original/current unknown labels and measured
alignment evidence. The independent retention audit recomputes eligibility and
checks that signals were not relabeled. Defaults remain unchanged.

## Tests and offline result

Combined suite: **195 passed in 5.25 s**. Twenty new tests cover native/source
immutability, unchanged green behavior, default-disabled behavior, both heading
checks, current/source corridor positions, phase/route restrictions, nonfinite
states, mixed and red/yellow signals, and physical/age rejection despite signal
eligibility. The preceding prefix harness suite was **175 passed in 5.17 s**.

On the saved 12.2 s input, all five selected plans pass this exact policy and
physical checks, preserving actual unknown labels. Existing unknown-signal
ranking selects `accepted_frame_121`, age **0.1 s**, native target **2.559815 m/s**,
native clearance **0.371000 m**, tracking forecast **0.407542 m**. This differs
from the earlier diagnostic that substituted green solely to isolate the gate;
that diagnostic's ranking must not be reported as the new policy's choice.

Evidence under `outputs/driving-investigation-10/`:
`check_unknown_exit_policy.py`, `unknown-exit-policy-terminal122.json`.
Physical replay and resumed live comparisons are pending.

A further auditor test verifies a valid unknown-exit record, then confirms that
either changing its signal label to green or removing its eligibility evidence
makes the audit fail. The focused exit-cache/reserve/prefix suite is **39 passed
in 2.71 s**. This also enforces explicit eligibility evidence for any non-green
retention; previously recorded green uses retain their existing behavior.

## Planned comparison and labeling

Use the verified-prefix harness to replay the original 12.2 s approach, then
resume fresh inference with either unchanged eligibility or this option enabled.
Every prefix state and original-controller command must reproduce before either
continuation. A separate verify-only run precedes live use. Camera renders may
differ, so this is not a pixel-identical whole-run A/B. Each output will state
where fresh inference resumes and will retain all rejected pools and native
source provenance. A completed continuation would not by itself establish a
new full-run success.

Saved options: `verify-prefix-speed4-terminal122.json`,
`resume-speed4-terminal122-baseline.json`, and
`resume-speed4-terminal122-unknown-exit.json` in the round-10 output directory.


## Physical exit-candidate replay

`replay-terminal122-source121/` reproduces the full 12.2 s original prefix and
controls exactly, then holds the selected original native plan for a 1.5 s
tracking diagnostic. All recorded corners remain inside through **13.7 s**,
final x **15.7134 m**, speed **2.4417 m/s**, total left rotation **95.2454 degrees**.
The route exit remains at x 21.2431 m, so this is not completion. Minimum
whole-run recorded clearance is 0.204931 m, inherited from the prefix; final
clearance is 0.548823 m. Holding beyond 0.5 s here is an explicitly labeled
physical diagnostic, not a live age-limit change or fresh-inference success.

The capture auditor now uses the new physical snapshot context for replayed
frames, preserves the original inference context separately, and distinguishes
archived visual answers from fresh checks. Verify-only output is never marked
completed merely because a batch summary exists. Ten prefix tests pass in
2.63 s, including this regression check.


## Prefix verification and baseline continuation

`verify-prefix-speed4-terminal122/` verifies **122 commands exactly** and zero
position/orientation error. The maximum context-speed difference against CSV
is 3.31e-7 m/s, from the float32 context representation. Native retention,
controller, snapshot and independently recomputed tracking audits all pass.
There are no fresh model calls in this verify-only run. Wall time is **51.99 s**.

`resume-speed4-terminal122-baseline/` replays the same verified prefix and then
attempts fresh sampling with unchanged cache eligibility. It fails at **12.2 s**:
all **48** new native samples fail body geometry; 40 also misalign with the
route, 18 leave the center corridor and six reverse. All five selected cached
plans fail solely `reuse_requires_green`. No new command is executed after the
prefix. Wall time is **137.32 s**. The summary's zero completed fresh planning
steps does not mean zero attempted inference: **six new pools** were sampled.
The new exit-cache comparison is in progress, with all other settings unchanged.

The saved figure `native-exit-failure-comparison.png/.pdf` shows the original
first pool's eight rejected native paths against the selected recent native
path and the validation corridor; it was visually checked. This is an offline
comparison, not a continuation outcome. `native-exit-curvature-audit.json`
confirms the generated native yaw at 1.5 s is 27.8–40.7 degrees left at the
terminal input, versus mostly 0.45–2.49 degrees at 12.0 s (one 10.44-degree
outlier). Decoder speed at failure is 2.7569 m/s against measured 2.7130 m/s.
The generated curvature, not a supplied history-initialized turn rate, creates
the extra turn. A CPU action-integration check differs from stored native yaw
by at most 0.00334 rad across the inspected inputs; it is not claimed exact.


The complete combined suite after capture-auditor changes is **197 passed in
5.01 s**. The opt-in continuation has bridged frame 122 using the exact frame
121 native source, and fresh sampling again finds an admissible model sample
at frame 123. This is an intermediate recovery only; the run is still active
and must reach the route endpoint and pass full audits before any continuation
completion claim.


`resume-boundary-input-comparison.json` confirms original, baseline and opt-in
resume inputs have bit-identical world poses, ego XYZ/rotation histories and
camera indices. Images differ: original-to-baseline mean absolute RGB-channel
difference 1.5616/255 (25.95% unequal channels), baseline-to-option 1.8611/255
(29.41%). This is a physically matched continuation comparison, not an exact
pixel-input A/B. The baseline nevertheless reproduces the all-48-samples native
body failure and the same five signal-only cache rejections.

A separate full-run preset, `tracking-native-four-left-speed4-unknown-exit-43.json`,
adds only the opt-in unknown-exit flag to the preceding speed-4 preset. It contains
no prefix replay settings. Use `candidate_reserve_probe` to test the current
combined native validation/resampling package from the original scene start.
The run is planned after the resumed comparison ends and passes its audits.
