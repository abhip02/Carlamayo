# Compatible left-turn phase transition — round 13

Branch: `experiment/driving-phase-transition`.
Rollback checkpoint: `b44853e` on `experiment/driving-exit-signal-transition`.
The five-case round-12 benchmark is preserved; it is not rerun in this round.
Production launchers, controller, decoder and model weights remain unchanged.

## Isolated hypothesis

The centered left turn in round 12 exhausts validation at 5.7 s. The instruction
has crossed the existing 60-degree turn boundary from `approach` to `finish_turn`.
The former unconditional phase gate rejects recent native sources even though
both instructions request the same left maneuver. The prior offline check found
two such sources that pass the remaining native and tracking checks. This is a
specific wrapper failure to investigate, not evidence that the model is fixed.

## Physical checks before changing eligibility

`outputs/driving-investigation-13/run_physical.py` replays the exact 5.7 s source
history, then executes each candidate unchanged for 0.3 s. Each continuation
uses original ages 0.3, 0.4 and 0.5 s, entirely inside the existing age limit.
There is no new inference and no relabeling, reanchoring or speed override.
Both histories and all 57 prefix commands reproduce exactly: zero position,
orientation, speed and controller error. Both continuations reach 6.0 s without
collision or recorded corner departure.

| Native source | Minimum clearance after boundary | Final clearance | Final speed |
| --- | --- | --- | --- |
| `accepted_frame_054` | 0.119870 m | 0.176646 m | 5.31420 m/s |
| `reserved_frame_054_pool_3_candidate_0` | 0.116436 m | 0.167022 m | 5.06904 m/s |

The shared boundary clearance is only 0.105865 m. These are short physical
diagnostics, not completed turns or fresh inference rollouts. Evidence lives in
`physical-results.json` and the two `physical-*` capture directories.

## Opt-in change

`validation_allow_left_phase_cache: true` permits only `approach` to
`finish_turn` for the same left route with source/current verified green.
Source/current poses must be inside the reference corridor and past entry,
with forward route progress and actual left rotation crossing the existing
60-degree boundary. The route itself must describe a 60–120-degree left turn.
The option is off by default. Other phases, maneuvers and signal pairs remain
ineligible under this rule.

The original labels are preserved in explicit `phase_reuse` evidence. All
existing age, native body, remaining horizon, distance, heading, reverse-motion,
speed, controller-tracking and ranking checks still apply. No waypoint or speed
is modified. The independent audit also compares the current phase against the
archived fresh decision or rejection, preventing a relabeling from hiding the
transition.

`check_boundary_policy.py` validates the actual archived sources with the flag
off/on, without changing any source record. The flag admits only the two
physically tested sources. Frame 052 still fails native clearance; frame 055
still fails tracking clearance. See `boundary-policy-audit.json`.

Validation: **138 experiment tests pass in 3.67 s**, including 25 new behavioral
and evidence-tampering cases. The first test invocation could not import pytest
from the model environment; the successful run uses the already installed test
dependencies from `.runtime/verifai-venv/lib/python3.12/site-packages`, appended
to the model interpreter's search path. No dependency or model environment was
changed. Real CARLA is imported before pytest to avoid test-stub substitution.

## Predeclared matched continuations

Both jobs use the same original scene configuration, seeds and 5.7 s prefix,
then resume fresh sampling to the original route endpoint or a 12 s total cap.
Only `validation_allow_left_phase_cache` differs. The baseline runs first.
Every prefix state and controller command must reproduce before new inference.
Rendered cameras can differ, so boundary images are retained and a reproduced
baseline failure must be checked rather than assumed.

Options: `resume-phase-baseline.json`, `resume-phase-enabled.json` in the round-13
output directory. Status: pending live execution. These are verified-prefix
continuations and must not be presented as new whole-run inference successes.
