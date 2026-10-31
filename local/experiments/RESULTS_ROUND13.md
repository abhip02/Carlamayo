# Compatible left-turn phase transition — round 13

Branch: `experiment/driving-phase-transition`.
Rollback checkpoint: `b44853e` on `experiment/driving-exit-signal-transition`.
The five-case round-12 benchmark is preserved; it is not rerun in this round.
Production launchers, controller, decoder and model weights remain unchanged.

**Result: the baseline reproduces the 5.7 s failure; the opt-in continuation
reaches the endpoint at 10.2 s.** This is a 5.7 s verified replay plus 4.5 s of
fresh continuation, not a new full fresh-run result. The phase exception is used
once; all 102 executed native/controller checks pass. Five-case regression is
still deferred.

[Matched comparison video](../../outputs/driving-investigation-13/phase-transition-comparison.mp4),
[enabled continuation](../../outputs/driving-investigation-13/phase-enabled-review.mp4),
and [measured speed/clearance](../../outputs/driving-investigation-13/phase-transition-speed-clearance.png)
([PDF](../../outputs/driving-investigation-13/phase-transition-speed-clearance.pdf)).

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
output directory. Launch commit: `6203fa2`. Both matched continuations and their audits are complete. These are verified-prefix
continuations and must not be presented as new whole-run inference successes.


The baseline reproduces the failure at 5.7 s: six pools / 48 new native
candidates, none admitted. Twenty-nine fail native body containment, three fail
the native clearance margin, and 16 fail the approximate tracking check; three
also fail center-corridor containment. The 57 replayed commands and all native,
controller, tracking, snapshot and ranking audits pass. There are zero executed
fresh steps, but six inference pools; these counts are distinct. Wall time
including startup is 148.85 s.

The sampling-summary helper initially raises `min() iterable argument is empty`
because it assumes at least one fresh executed command. A round-13 copy of the
helper now reports null min/max executed target speeds for an empty suffix,
while retaining all six pools and 48 candidates. The original failure log and
status are preserved with `-initial` names; the repaired report passes without
rerunning the baseline or changing driving code. The first runner stopped before
launching the enabled case, then resumed that pending case only.


## Completed enabled continuation

The option is used exactly once at 5.7 s, selecting unchanged
`accepted_frame_054` at its true age of 0.3 s after 48 fresh candidates fail
(29 native-body, four native-margin and 15 tracking failures; three also fail
center containment). The first changed physical step matches the earlier
no-inference physical replay exactly in position, orientation and speed.
Fresh sampling supplies the executed path at 5.8 s.

The run reaches the endpoint at 10.2 s, x 21.26470 m, y 134.45380 m, yaw
−1.85590°, speed 3.86553 m/s. All 102 executed paths, original world anchors,
speed arrays and ages match; native/controller, tracking, snapshot, retention
and ranking audits pass. There is no recorded corner departure or near-stop.
Minimum speed in the fresh suffix is 3.86553 m/s. Minimum clearance strictly
after the replay boundary is 0.119870 m; the inherited boundary is 0.105865 m.
Whole-run speed ranges from 2.79787 to 5.16169 m/s.

The suffix has 45 commands, 101 pools / 808 native candidates, 226 hard-admitted
candidates and 43 fresh-admitted planning steps. There are 36 cache selections:
34 better-ranked and two after fresh exhaustion. The existing exit-signal rules
are used four times for green-to-unknown and 20 times for matching unknown.
No other phase transition is permitted. Wall time including startup is
1,443.53 s (24.06 min). The full five-case suite and a full fresh run with this
flag remain untested.

`matched-continuation-results.json` confirms the only differing option is
`validation_allow_left_phase_cache`, both captures have identical recorded
source hashes, and the first changed physical step matches the prior replay.
`boundary-input-comparison.json` confirms identical poses, instructions, XYZ
history, rotation history and camera indices. Original–baseline RGB difference
averages 1.89996/255 (34.599% unequal channels); baseline–enabled averages
0.817414/255 (15.807% unequal channels). Thus the resumed model inputs are not
pixel-identical, although both runs exhaust all fresh candidates at the boundary.
The baseline has zero position/orientation/controller prefix error and maximum
float32 context-speed difference 3.92e-7 m/s.

The H.264 videos mark the replay/fresh boundary explicitly; the comparison freezes
its baseline panel at the failure. Reviewed at 9.9 s. The full yellow 6.4 s
prediction is not guaranteed by short-horizon screening; recorded containment
is not a swept-volume or obstacle guarantee. The owned CARLA server is stopped
after the matched jobs. The five-case benchmark and original model remain intact.

## Separate frozen-input slowdown evidence

A read-only inspection of round-12 straight case 03 shows one pool is accepted
at 9.7 s even though all eight candidates target below 0.34 m/s. The generated
stop-sign explanation remains unsupported by the earlier map checks. Sampling
can stop early here because the current visual label is unknown, the green-only
minimum does not apply, and the existing quality retries do not request a retry
for a very low target.

`export_prefix_inputs.py` then reproduces that case's first 9.7 s exactly and
exports freshly rendered inputs at 9.0, 9.5 and 9.7 s. All 97 replayed commands
pass the physical/native audits; no new inference or new command is executed
during export. `straight097-sampling-variants.json` predeclares six pools per
fixed input using the existing seed/temperature schedule. The offline run
produces 144 native candidates; each is screened against the original controller
state and unchanged geometry/tracking thresholds. None is executed.

| Input time | Current speed | Pool-0 selected target | Later selected target passing screening |
| --- | --- | --- | --- |
| 9.0 s | 3.73235 m/s | 2.87173 m/s | Pool 1: 3.94972 m/s |
| 9.5 s | 3.08233 m/s | 1.57479 m/s | Pool 1: 2.63834 m/s; pool 5: 4.40601 m/s |
| 9.7 s | 2.57280 m/s | 0.346354 m/s | Pool 3: 2.22228 m/s; pool 5: 3.60603 m/s |

The first pool reproduces the original low-target pattern, including the stop-sign
explanation at 9.7 s. The later candidates are a lead for bounded low-speed
resampling, not yet a physical recovery. `slowdown-frozen-screening.json` retains
all 18 pools, including rejected candidates and unsuccessful alternatives.
`low-target-pool-evidence.json` preserves the original read-only evidence.
The export and offline-sampling commands/status are in `slowdown-status.json`.
