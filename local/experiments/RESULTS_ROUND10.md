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
