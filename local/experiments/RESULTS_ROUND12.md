# Five short scenario variants — round 12

Policy branch: `experiment/driving-exit-signal-transition`.
Frozen policy checkpoint: `cc9da24`; transition-rule rollback: `68efac1`.

**Result: one of five predeclared trials reaches the route endpoint.** Three
terminate because no native plan passes the existing checks; one reaches its
10 s cap short of the exit and slows below 1 m/s. The offset left turn completes
at 11.9 s. These trials expose remaining failures and do not establish a general
driving fix. All 405 executed commands pass the independent native/controller
audits, with no recorded corner departure. No policy was changed between runs.

[Watch all five together](../../outputs/driving-investigation-12/five-scenarios-comparison.mp4)
or inspect the [measured speed and clearance plot](../../outputs/driving-investigation-12/five-scenario-speed-clearance.png)
([PDF](../../outputs/driving-investigation-12/five-scenario-speed-clearance.pdf)).
The comparison freezes each panel when its run ends and labels failures/caps.

The user requested five different short scenarios after the completed fresh
left turn in [round 11](RESULTS_ROUND11.md). The predeclared suite is
[`presets/five-short-scenarios.json`](presets/five-short-scenarios.json).
All five start fresh, with no archived prefix or policy tuning between runs.
Every executed path/speed remains model-generated; production launchers,
controller, decoder and model weights remain unchanged.

| Scenario | Maneuver | Start before entry | Initial speed | Lateral offset | Heading offset | Text / flow seed | Cap |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 01 | Straight | 5 m | 3 m/s | 0 m | 0° | 44 / 43 | 10 s |
| 02 | Straight | 5 m | 4 m/s | +0.25 m | −2° | 45 / 44 | 10 s |
| 03 | Straight | 10 m | 3 m/s | −0.25 m | +2° | 46 / 45 | 10 s |
| 04 | Left | 5 m | 3 m/s | 0 m | 0° | 44 / 43 | 12 s |
| 05 | Left | 5 m | 4 m/s | +0.20 m | −2° | 45 / 44 | 12 s |

These are variations at the same empty, all-green Town03 intersection, not five
different maps or traffic situations. The current screening implementation
supports straight and left routes. Right turns are not included. Start-position
and seed changes are combined to test different scenarios, not isolate causes.

Outputs: `outputs/driving-investigation-12/`. Each capture saves its expanded
configuration, options, source hashes, predictions, physical states and videos.
Initial model inputs and automatic failure captures are retained. The existing
independent native/control, footprint, route, retention, ranking and tracking
audits were run on each result, including incomplete runs. A capture with no
executed command is a failure, not a pass with empty audits.

Completion requires the recorded route-exit marker. Reaching a duration cap
alone is labeled incomplete for the maneuver. Report any collision, recorded
corner departure, speed below 0.5 m/s, native provenance failure, or controller
mismatch. Minimum recorded speed and clearance are reported even if all
binary checks pass. Short-horizon checks are not a full-trajectory, swept-volume
or obstacle guarantee. Every failure remains in the final suite summary.

Launch commit: `c2d2d6f`. All five attempts and audits are complete. All five use
the same frozen policy and model hardware. Total suite wall time is 3.579 hours
(about 3 h 35 min), for 40.5 s of recorded simulation. There are 1,066 fresh
proposal pools / 8,528 native candidates and 244 revalidated cache selections.
This remains an expensive offline experiment, not real-time driving.

## Results

| Scenario | End | Simulated time | Minimum speed | Minimum corner clearance | Native/controller audit |
| --- | --- | --- | --- | --- | --- |
| 01 straight centered | No admissible path; crossing incomplete | 8.4 s | 1.67927 m/s | 0.551383 m | All 84 commands pass |
| 02 straight offset, faster | No admissible path; crossing incomplete | 4.5 s | 1.93536 m/s | 0.496272 m | All 45 commands pass |
| 03 straight opposite offset | Duration cap; exit 2.43033 m away | 10.0 s | 0.974046 m/s | 0.496365 m | All 100 commands pass |
| 04 left centered | No admissible path; 63.6830° left | 5.7 s | 2.79787 m/s | 0.105865 m | All 57 commands pass |
| 05 left offset, faster | Reached route endpoint; checks pass | 11.9 s | 2.55189 m/s | 0.188681 m | All 119 commands pass |

Scenario 01 exhausts six pools at frame 084. None of the 48 fresh candidates
passes: 47 fail native body geometry and two lack the required marker; rejection
reasons overlap. Eighteen also misalign, ten leave the center corridor and six
reverse. Five recent selected paths and seven reserve candidates are rejected
at the earlier signal gate. The current visual answer is unknown; the existing
unknown-exit exceptions apply only to left turns. This does not establish that
those cached paths would pass the subsequent physical checks.

The 84 executed commands preserve native arrays, speeds, original anchors and
ages exactly. Controller, tracking, snapshot and ranking audits pass. No recorded
corner departs and no speed falls below 0.5 m/s. Actual speed ranges from 1.67927
to 4.36586 m/s. The run uses 262 fresh pools and 52 revalidated cache selections.
Wall time including process/model startup is 3,288.43 s (54.81 min).

The speed profile remains inconsistent: at 5.4 s the executed native preview
target is 1.29453 m/s while its generated text says to accelerate through the
green light. Speed later recovers. At 7.5 s the visual label changes from green
to unknown, while CARLA's controlled signals remain green. The terminal event
at 8.4 s ends the experiment because no admissible path exists; it is not a
physical stop by the car.

[Scenario 01 video](../../outputs/driving-investigation-12/01-straight-centered-review.mp4)
is H.264 with the failure labeled; reviewed at 8.2 s. Per-capture audits and
`suite-results.json` preserve the full evidence. All subsequent cases used the same
policy; no tuning was performed in response to this failure.

Scenario 02 fails at frame 045 while the visual signal remains green. All 48
fresh candidates fail native body geometry; 26 also misalign, 24 leave the
center corridor, 17 fail the minimum green-progress check and nine reverse.
The latest admitted fresh source is frame 039, now 0.6 s old; no selected or
reserve candidate remains inside the unchanged 0.5 s decision-time age limit.
This failure therefore does not depend on an unknown signal label.

All 45 executed commands pass the independent native, controller, tracking,
snapshot and ranking audits. No recorded corner departs or near-stop occurs.
Actual speed ranges from 1.93536 to 4.61860 m/s, with minimum clearance
0.496272 m at the initial offset pose. There are 135 fresh pools and 35 cache
selections. Wall time including startup is 1,660.67 s (27.68 min).
[Scenario 02 video](../../outputs/driving-investigation-12/02-straight-offset-faster-review.mp4)
is labeled as a failure and reviewed at 4.3 s. The video reports prediction age
after the physical tick, so it can show 0.6 s after a command selected at the
allowed 0.5 s age; the independent audit checks the actual command-time age.

Scenario 03 runs the full 10 s window but does not finish the crossing. Its
final route distance before the exit is 2.43033 m and final speed is only
0.974046 m/s. Actual speed ranges from 0.974046 to 5.22085 m/s. All 100 executed
native/controller, tracking, snapshot and ranking checks pass; no recorded
corner departs. It uses 214 fresh pools and 42 cache selections (40 better-ranked,
two after fresh exhaustion). Wall time including startup is 2,552.74 s (42.55 min).

At 9.7 s, the generated explanation says to slow for a stop sign, with a native
preview target of 0.338116 m/s. At 9.8/9.9 s the explanations say to keep lane,
while native targets drop to 0.238840/0.182242 m/s. The unchanged controller
applies brake 0.3 on the final command. The visual signal label has been unknown
since 7.0 s; CARLA's lights remain green. The basic 0.5 m/s near-stop threshold
is not crossed, but this late slowdown is material and must not be hidden by
that binary check.

`scenario03-stop-sign-map-check.json` records a read-only map check: no
`traffic.stop` actors within 60 m of the final position, and no type-206 stop
landmarks returned in the next 40 m from the nearest driving waypoint. This
does not audit every rendered visual prop, nor prove the prose caused the
deceleration, but the stop-sign explanation is unsupported by those map checks.
The query does not tick or change the simulator.

[Scenario 03 video](../../outputs/driving-investigation-12/03-straight-opposite-offset-review.mp4)
is labeled as duration-capped and incomplete; reviewed at 9.8 s. Its completion
of the requested recording window is separate from completing the maneuver.

Scenario 04 terminates at frame 057, after 63.6830° of left rotation. The signal
is still green and the navigation phase is `finish_turn`. Of the 48 fresh
candidates, 30 fail native body containment, four fail the native 0.1 m margin,
and the remaining 14 fail the approximate controller-tracking clearance check.
Two candidates also leave the center corridor. This is a failure to find a path
under the current checks, not proof that every rejected native path would depart
under actual physics.

Three recent selected sources remain: two are rejected at the earlier
`navigation_phase_changed` gate, and one fails tracking clearance. The single
reserve source is also rejected for phase change. No signal-exit exception is
used. An additional read-only offline diagnostic reconstructs all 57 unchanged
controller commands and examines the checks hidden behind the phase gate. It
changes only a diagnostic copy of the phase label, preserving the actual source
and current labels in the report; no altered label or plan is executed. The
frame-052 source then fails native clearance (1.92104 m maximum overhang).
The frame-054 selected source and frame-054/pool-3/candidate-0 reserve pass the
remaining native and approximate tracking checks, with forecast minimum
clearances 0.134396 and 0.128385 m respectively, against a 0.1 m requirement.
Their original ages are 0.3 s and native targets are 6.99642 and 8.09883 m/s.
Neither meets the preferred 0.2 m reserve margin. The forecast assumes constant
measured speed 5.03326 m/s and does not model obstacles, so this is a lead for a
later controlled test, not a demonstrated physical rescue. Native arrays,
speeds, anchors, ages and source files remain unchanged. See
`scenario04-phase-gate-diagnostic.json` and `check_scenario04_phase_gate.py` in
the output directory. The live policy remains frozen for all five scenarios.

All 57 executed native/controller, tracking, snapshot and ranking checks pass.
No recorded corner departs. Minimum clearance is only 0.105865 m at the final
pose; speed ranges from 2.79787 to 5.16169 m/s, ending at 5.03326 m/s. Final
position is x 2.41113 m, y 133.07149 m; the route endpoint remains 18.90885 m
ahead along the reference. There are 186 fresh pools and 31 cache selections:
28 better-ranked and three after fresh exhaustion. Wall time including startup
is 2,129.60 s (35.49 min).
[Scenario 04 video](../../outputs/driving-investigation-12/04-left-centered-review.mp4)
is labeled as a failure and reviewed at 5.5 s. The distant yellow native path
can leave the road; only the configured short horizon is screened and no path
points are repaired. The original controller and all validation thresholds
were unchanged in scenario 05.

Scenario 05 reaches the route endpoint at 11.9 s, independently confirmed by
the route audit. The final state is x 21.32446 m, y 134.38124 m, speed
2.82458 m/s, and left rotation 90.71354°. The final route projection is 0.08004 m
past the endpoint. All 119 native/controller, tracking, snapshot and ranking
checks pass, with no recorded corner departure. Speed ranges from 2.55189 to
4.73175 m/s; minimum clearance is 0.188681 m. Native controller-preview targets
range from 1.55401 to 5.28385 m/s. No near-stop occurs, although speed is lower
in the exit lane than during the approach.

There are 269 fresh pools / 2,152 candidates and 84 cache selections: 83
better-ranked and one after fresh exhaustion. The existing aligned-left-exit
rules are used for five green-to-unknown transitions and 24 matching-unknown
reuses. Both original labels remain recorded; all source arrays, anchors,
ages and physical checks are preserved. This does not authorize unknown-signal
reuse in other scenes. Wall time including model/process startup is 3,213.99 s
(53.57 min).
[Scenario 05 video](../../outputs/driving-investigation-12/05-left-offset-faster-review.mp4)
shows the whole fresh run and labels its successful endpoint result; reviewed
at 11.7 s. Its yellow 6.4 s native prediction still curves off the road farther
ahead. Only the configured short horizon is screened; endpoint completion does
not validate that entire unexecuted prediction or behavior beyond this rollout.

## Interpretation

A second full fresh left turn completes under a different starting condition,
but the same frozen policy fails in the centered left variant and
two straight variants. The capped straight variant exhibits a severe late
slowdown. The different starts and generation seeds were varied together, so
this suite cannot attribute the difference to a single parameter or estimate
a population success rate from one trial per condition.

Three leads remain distinct: lack of acceptable native samples even while the
visual signal is green (scenario 02); signal-label loss and low native target
speeds near exits (01/03); and the phase-change cache gate hiding candidates
that pass approximate physical screening (04). The latter needs an isolated
physical continuation before considering a policy change. No live fallback,
path repair, speed override, controller change or new exception was added in
this benchmark. The 0.5 m/s binary threshold alone is too permissive to claim
normal-speed behavior, as scenario 03 demonstrates.

## Evidence storage and execution

`run_suite.py`, `summarize_suite.py`, `status.py`, `render_comparison.py`,
`plot_suite.py`, `summarize_failures.py` and `audit_suite_consistency.py` are
saved with the generated evidence in `outputs/driving-investigation-12/`.
`suite-results.json` records final outcomes; `suite-status.json` retains all
exact launch commands, return codes and audit return codes. The runner refuses
to overwrite existing captures. Choose a new output directory for any rerun.
`failure-digest.json` keeps overlapping rejection counts and rejected cache
sources without treating an early-gate failure as a later physical failure.
`suite-consistency-audit.json` verifies every recorded configuration and option
against the predeclared manifest, and every recorded source hash against the
frozen policy and current file. All checks pass. Every per-capture audit and
sampling audit returns zero, including for the three aborted experiments.
Artifacts and videos are local generated evidence, excluded from Git.
Captures run serially under the existing experiment lock. A spare-GPU check
found that the free TITAN RTX pair lacks native BF16 support; no hardware,
precision or policy changes were made to enable parallel execution.

To preserve disk space, `archive_inputs.py` losslessly compresses input snapshots
older than two minutes, verifies their decompressed SHA-256 against the original,
and only then replaces the original file with `.pt.gz`. The filename, original
hash and sizes are retained in `compressed-inputs.json`. Captures do not read
these already-saved snapshots. Restore an individual file for frozen-input
replay with `gzip -dk path/to/frame.pt.gz`; this leaves the archive intact.
All 33 input snapshots are archived with verified hashes. No native prediction
records, controls, audit evidence or videos are removed.

Final cleanup: the owned `apomalap-carlamayo-0916` container is stopped (exit
status confirmed); the batch exits successfully after retaining all five
outcomes. Production `module/` and `local/run_intersection.py` remain unchanged
from integration baseline `5b94733`. The external model source tree remains
clean at `36aeb4c`. This round adds a scenario manifest and evidence documentation;
it does not promote experimental driving behavior to the default launchers.
