# Aligned-exit signal transition — round 11

Branch: `experiment/driving-exit-signal-transition`.
Rollback checkpoint: `68efac1` on `experiment/driving-exit-cache-validation`.
Production launchers, controller/configuration, decoder and model weights remain
unchanged. All paths and speed arrays remain exact native model outputs.

## Evidence motivating the isolated change

The round-10 fresh left run ends at 11.0 s with all 48 new native paths failing
body geometry. All recent paths fail an earlier signal gate: their original
labels are green, while the new image-based answer is `None`/unknown. Another
136 sampler variants and 48 wording samples produce no admissible replacement.
Turning the separate visual-question call off/on/off reproduces the same native
arrays and tokens exactly. See [round 10](RESULTS_ROUND10.md).

Offline physical checks find five recent selected paths usable, but only
`accepted_frame_109` has both source/current headings within the existing
10-degree exit-alignment restriction. Its age is 0.1 s, native target 3.32148 m/s,
source/current exit errors 9.57458/8.33540 degrees, native clearance 0.327528 m.
The new exact predicate admits only that source; the other four remain ineligible.

`outputs/driving-investigation-10/replay-terminal110-source109/` reproduces the
full 11.0 s prefix and all original-controller commands exactly, then holds that
unchanged native path for a 1.5 s diagnostic. It stays inside through **12.5 s**,
final x **14.1233 m**, yaw **-1.97876 degrees**, speed **2.25263 m/s**. Minimum
continuation clearance is **0.331761 m**, final **0.539078 m**. Native identity
is exact; applied-control representation difference is at most 2.81e-8. This
holds beyond the live age cap for diagnosis, uses no fresh inference and does
not reach the route endpoint.

## Opt-in rule and validation

`validation_allow_green_unknown_exit_cache: true` separately permits green-source
to unknown-current reuse only when both states already meet every existing
left-exit eligibility condition: verified visual classification, `finish_turn`
phase, position inside the center corridor and past entry, and both vehicle and
local route headings within 10 degrees of the exit. Red/yellow and the reverse
unknown-to-green transition remain ineligible. Matching-unknown reuse still
requires its own preceding option.

Source green and current unknown labels are preserved in explicit eligibility
evidence. The independent audit recomputes the rule, verifies both original labels
and checks the option was enabled. Every native geometry, body, tracking, reverse,
remaining-horizon, distance, heading and **0.5 s age** check remains unchanged.
No map path, repaired waypoint, speed override or controller change is introduced.
Fresh sampling is still attempted before reuse. This is a controlled empty-scene
experiment; it does not resolve perception or provide obstacle handling.

Combined suite: **219 passed in 5.72 s**. Twenty-two new tests cover exact native
preservation, actual signal labels, default-off behavior, independent options,
red/yellow and reverse transitions, both source/current exit conditions, physical
and age rejection, unchanged green pairs, and provenance-audit tampering.

The full-run preset `tracking-native-four-left-exit-transition-43.json` adds only
this flag to the preceding preset. It has **not** been validated from t=0.

## Matched continuations

Options in `outputs/driving-investigation-11/` replay and verify the same 11.0 s
prefix, then resume fresh sampling with the transition flag off or on:
`resume-terminal110-baseline.json`, `resume-terminal110-exit-transition.json`.
The baseline runs first. Every replayed state and controller command must match;
camera renders may differ. Completion would be a verified-prefix continuation,
not a new full fresh-run success.

The baseline reproduces the failure at **11.0 s**: 48/48 new samples fail native
body geometry, 20 also misalign, 12 leave the center corridor and six reverse.
The 110 prefix commands and positions/orientations reproduce exactly; maximum
float32 context-speed difference is 3.76e-7 m/s. Independent native/controller,
physical snapshot and tracking audits pass. Six new pools are attempted and no
new command is executed after the prefix. The zero completed fresh-step count
must not be read as zero model calls. Wall time is **130.25 s**.

The opt-in comparison bridges frame 110 using exactly `accepted_frame_109`,
then retains it at ages 0.2–0.5 s while comparing newly admitted but slower native
samples. It switches to a new native source at frame 115 and has continued beyond
12.0 s. The five green-to-unknown uses preserve both original signal labels.
The complete route outcome and final audits remain pending.

Offline evidence: `check_terminal110_transition_policy.py`,
`terminal110-transition-policy-audit.json` in the round-11 output directory.


`resume-boundary-input-comparison.json` confirms the original, baseline and
opt-in boundary have identical world poses, instruction, XYZ/rotation histories
and camera indices. Mean absolute RGB differences are 1.61947/255
(original–baseline) and 1.67155/255 (baseline–option), with 27.28% and 28.60%
unequal channels respectively. Thus physics/history match exactly but renders
do not; the baseline independently reproduces the same all-48 native-body failure.
