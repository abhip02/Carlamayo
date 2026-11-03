# Upstream comparison and full-run driving validation — round 15

Branch: `fix/driving-native-validation`; rollback: `ced1ba7`.
User asks to make driving work while keeping the simulator/backend. Preserve
model-generated paths and keep all changes reviewable/revertible.

## Upstream evidence

Fetched `upstream/main` on 2026-11-02; its latest commit is still `4914558`.
There is no newer upstream driving fix to merge. Our original controller/config
match that revision. The original README demo was introduced at `578170c` for
Alpamayo R1, before the current 1.5 setup. Code shipped with that demo uses a
minimum 10 km/h speed target based on trajectory extent, and projects its steering
target onto a driving-lane map waypoint. This is not the same control policy as
following native 1.5 speeds and paths. The demo alone therefore does not establish
that our current configuration works. No change to CARLA or model backend is
needed to investigate this difference; no map-projected executed path is added.

References:
- https://github.com/aveeslab/Carlamayo/blob/578170cfbcb75222ff1fb6cd9e2d4c7d6167a4f9/carla_alpamayo_closed_loop.py
- https://github.com/aveeslab/Carlamayo/blob/4914558/module/trajectory_follower.py

## Predeclared full fresh checks

`presets/native-driving-phase-speed.json` combines the existing opt-in phase
transition and bounded low-speed resampling changes with the prior native-path
validation profile. It does not modify the controller, model, rendering or
validation thresholds. `presets/native-driving-full-checks.json` declares two
previously failing initial conditions: offset/faster straight (previous failure
4.5 s) and centered left (previous failure 5.7 s). Each has a 15 s cap and must
reach the original route endpoint. All inference begins at time zero; no history
is replayed. These are targeted full checks, not a replacement five-case score.

First run: `02-straight-offset-faster`. Second run is queued manually after the
first is inspected, so implementation revisions cannot silently change a running
batch. All outcomes, source hashes, inputs, proposals and audits are retained
under `outputs/driving-investigation-15/`. Results pending at this checkpoint.


## Separate bounded planning-interval experiment

While the unchanged combined-policy full run executes, a separate
`plan_interval_probe` entrypoint adds optional 0.3 s planning intervals. Every
0.1 s control tick revalidates the last executed native plan against actual pose,
unchanged body/age/distance/heading bounds and original-controller tracking.
Reuse additionally requires source/current green, a still-current visual check,
no instruction-phase or endpoint transition, and the configured speed/clearance
preferences. Any failed gate immediately invokes the existing fresh sampler.
Unknown/red/yellow signals therefore continue normal fresh planning. The default
interval is 0.1 s. No native path/speed or controller parameter is edited.

This targets unnecessary per-tick plan changes and inference cost. It is not
assumed to improve driving until physically tested. Source clocks are never reset;
new per-frame evidence explicitly records zero new inference, and the audit
recomputes native geometry and scheduling eligibility in addition to original
controller forecasts. Requested raw input captures remain available on skipped
inference steps. Summary counts separate executed commands, scheduled native
execution, fresh planning and actual sampled pools. The complete 179-test suite passes in 4.09 s, including wrapper restoration,
requested input capture, native identity, audit tampering and failed-gate cases.

Predeclared next comparison: the same offset/faster straight initial condition,
with fresh inference from time zero and the same 15 s cap, changing only the
0.3 s interval and its explicit entrypoint. Preserve both full-run outcomes.
Preset: `presets/native-driving-interval.json`. This new code is not imported by
the already running combined-policy baseline, whose recorded source hashes and
launch commit remain `2d512a2`.


## Footprint-horizon diagnostic (rejected lead)

`screen_body_horizon.py` screens all 96 saved candidates from the original
centered/offset straight terminal failures with native footprint limited to
1.5 seconds, while retaining center/heading/reverse checks over all required
controller inputs and the unchanged tracking forecast. It admits zero candidates
in either case. This does not support shortening native footprint checks; no
validation threshold or horizon was changed. No candidate was executed.
Evidence: `body-horizon-screening.json` and its log.

## Reproducible launcher

`bash local/run_driving.sh --route straight` (or `--route left`) resolves the
predeclared full-run settings. It generates lane-reference geometry from the
configured CARLA map, removing the dependency on ignored investigation files.
`--print-config` previews settings without connecting; `--duration`, `--seed`,
`--flow-seed`, and a one-rollout `--config` provide explicit overrides. The caller
starts/stops the existing server normally. Recordings and exact resolved settings
are saved under the requested `--output`, which must not already exist.

The default replan interval remains 0.1 seconds. `--replan-interval 0.3` selects
the separate interval experiment. The 190-test suite passes in 4.13 seconds,
including launcher validation and unchanged user-config input. Physical launcher
validation remains pending here; the running baseline still uses its original
entrypoint and source snapshot. Expanded source capture includes the launcher,
route exporter, candidate reserve, original runner, adapter, and inference code.

## Full fresh straight result

The combined policy reaches the original straight endpoint at **8.9 s**, starting
all inference at time zero. This is the same initial condition that previously
failed at 4.5 s. Minimum/maximum measured speed is **3.236 / 5.837 m/s**; minimum
recorded vehicle-corner clearance is **0.430 m**. No recorded near-stop, corner
departure, or collision occurs. All 89 original-controller commands, native
source arrays/anchors/ages, snapshot timing, tracking and retry-evidence audits
pass. Forty commands use a revalidated earlier native plan; no source history
was replayed. Sampling uses 172 pools / 1,376 candidates, with seven low-speed
retry requests. Capture wall time including startup is 2,074.9 s (34.6 min).

The result establishes a complete crossing in this controlled scene. Speed is
not constant: native targets rise to 7.76 m/s near the exit, followed by a
slowdown; measured speed remains above 3.236 m/s. This does not establish smooth
comfort behavior, arbitrary traffic handling, or five-case reliability.

[Full fresh straight video](../../outputs/driving-investigation-15/02-straight-offset-faster-review.mp4)
and [measured speed/clearance](../../outputs/driving-investigation-15/full-fresh-speed-clearance.png).
The raw capture, combined audit and sampling audit are under
`outputs/driving-investigation-15/02-straight-offset-faster/`.

The separately declared 0.3 s launcher comparison is running at `86ec832`.
Automatically generated route geometry matches the original reference exactly.
The centered-left full fresh check remains queued; the five-case benchmark has
not been rerun.

## Remaining upper-speed retry gap

The completed straight run selects native targets 5.62, 5.97, 7.19, 7.76 and
7.34 m/s at 7.5–7.9 s. Every candidate in these first pools is admitted, but
the upper-speed preference explicitly reports `applied: false`: the visual
label is `unknown`, and the original upper retry runs only for green. At 7.8 s
even the slowest candidate is 7.76 m/s. The next pool at 8.0 s returns a 3.56 m/s
selection, creating an avoidable target-speed swing.

An additional, default-off option `validation_resample_fast_unknown` extends
the existing upper-speed retry to unknown visual labels. It does not label an
unknown signal green, change the ranking, clamp a target, modify a path, add
pools beyond the existing budget, or change red/yellow behavior. An admitted
fast sample remains available if subsequent pools are worse or invalid.
The independent audit now checks upper-speed annotations and each retry request
against archived pools. All 197 experiment tests pass in 4.50 s; the completed
baseline also passes the extended audit (85 fresh source decisions).

This option has not yet been physically validated or enabled in the launcher
profile. The interval comparison remains unchanged at its launch revision.
Before recommending the new option, run the same full fresh straight condition
with it enabled, then the full centered-left check with the resulting proposed
launcher configuration. Preserve the unmodified full straight and interval
results alongside these additional checks. No new five-case benchmark is claimed.

The next profile is `presets/native-driving-balanced.json`: it keeps the
successful **0.1 s** replanning schedule and adds only the unknown upper-speed
retry, plus automatic reference export. `native-driving-balanced-checks.json`
declares the same straight/left starts and 15 s caps. This isolates the remaining
speed retry fix from the optional 0.3 s scheduling experiment. The launcher's
default manifest will be switched only after the fresh straight check succeeds;
the centered-left run will then exercise that proposed default end to end.

## Full fresh interval result

The user-facing launcher completes the same straight crossing at **8.6 s** with
the optional 0.3 s interval. Minimum/maximum measured speed is **3.206 / 5.751
m/s**; minimum recorded corner clearance is **0.496 m**. No near-stop, recorded
corner departure or collision occurs. All native/controller, geometry, tracking,
sampling and extended retry audits pass. Generated route geometry is identical
to the original reference. There are 86 executed decisions, 24 scheduled native
execution steps, 62 fresh planning decisions and 139 sampled pools. Wall time
including startup is 1,785.7 s (29.8 min), versus 34.6 min for the 0.1 s run.
These different closed-loop trajectories are not a general performance benchmark.

The run exposes a reporting-only bug: the interval finalizer reads the parent's
active-probe reference after the parent has cleared it, so the original summary
reports 86 inference calls rather than 62 fresh decisions. Independent audits
already count the 24 scheduled steps correctly. The original summary is preserved;
`interval-inference-count-correction.json` records corrected counts and its hash.
The finalizer now retains its own instance through cleanup. A test reproduces the
actual parent-cleanup order; the complete **198-test** suite passes in 4.24 s.
The execution policy is unchanged by this correction.

The interval remains optional. The next full straight check uses the declared
0.1 s balanced-speed profile, followed by the centered-left launcher check.

The fresh balanced capture starts at `e1e8cfe`. Its physical initial state
(excluding the episode name), ego-position history, ego-rotation history and
camera-index tensor match the baseline exactly. Camera pixels differ: mean
absolute difference is 1.189 / 255 at the first input, and native predictions
differ from the first decision. These are independent fresh checks, not an
identical-image paired experiment. The saved within-decision candidate pools
support checking the specific effect of extra retries on the same input.
Evidence: `balanced-initial-comparison.json` and
`balanced-first-input-comparison.json`. Quality reports now separate unknown
upper-speed retry counts from the legacy green-only count.

## Fresh repeat failure: straight-junction cache veto

The balanced full fresh repeat **fails at 6.2 s**: none of 48 new native
candidates passes all checks. It has not triggered an unknown upper-speed retry.
Measured speed remains 3.268–4.787 m/s, minimum recorded corner clearance is
0.496 m, and all 62 executed native/controller audits pass. The failure is
sampling exhaustion, with no recorded stop or departure. The new profile is
not promoted. Its complete capture and failure remain intact.

Every recent selected/reserved plan is rejected early with `reuse_requires_green`,
including plans generated under the same unknown label only 0.1–0.2 s earlier.
The existing unknown-cache exceptions cover the left exit only. In a CPU-only
diagnostic removing just that label veto, **32 of 33** archived native plans pass
all remaining geometry, age, distance, heading, speed and controller checks.
`accepted_frame_059` still fails tracking clearance (0.09756 m versus 0.1 m).
The unchanged ranking selects `accepted_frame_057` at age 0.5 s. No source array,
speed, clock or record is modified. Evidence is in
`straight-unknown-cache-screening.json` and `straight-unknown-cache-ranking.json`.

Two physical checks reproduce all 62 prefix states and commands exactly, then
execute original source 057 for one tick (age 0.5 s), and original source 060
for three ticks (ages 0.2, 0.3, 0.4 s). Both succeed with minimum continuation
clearance about 0.771 m; minimum speeds are 3.648 and 3.429 m/s. These are short
native-plan diagnostics, not full fresh runs. See `straight-unknown-physical-results.json`.

The default-off `validation_allow_straight_unknown_cache` removes that label veto
only for an aligned straight connection, unchanged approach phase, green/unknown
source, current unknown, forward progress, and **both complete vehicle boxes past
the junction entry while their centers remain before the exit**. Source visual
labels must agree with their recorded classifications. Signal labels remain
unchanged. All original physical and half-second age checks still apply; fresh
unknown-signal sampling is still attempted every tick. The existing left rules
are unchanged. This remains a controlled empty-scene policy, not authorization
to approach an unknown traffic signal.

All **228 tests pass in 4.27 s**, including 30 new eligibility, classification,
body/age rejection and audit-tampering cases. The strict archived-source check
must admit exactly the 32 physically eligible plans and still reject source 059.
An initial hand-count assertion incorrectly expected 32 total sources; it was
corrected to the 33 actual archived records and a set comparison of their labels.

Next: a fresh continuation after the verified 6.2 s prefix, then a complete fresh
straight run and centered-left launcher check. The proposed complete settings are
`presets/native-driving-junction.json` and `native-driving-junction-checks.json`.
These preserve the 0.1 s schedule and add only the scoped straight unknown-cache
rule to the previous balanced profile. The simple launcher still uses its earlier
profile pending full validation.

The same flag was also screened against the unchanged round-12 archived terminal
states, without running inference or simulation. At the centered-straight 8.4 s
unknown-label failure, 11 of 12 recent native sources pass all remaining checks
with the flag enabled; zero pass with it disabled. The older offset-straight
4.5 s green-label failure has no recent sources to reuse, so this rule cannot
resolve that archived state. Source files, native arrays, anchors and times are
unchanged. This is a retrospective diagnostic, not a new endpoint result or
five-case rerun: `prior-straight-cache-screening.json` and its script/log.

## Straight unknown-cache continuation result

At `8a7e6a5`, the failed balanced run's first **6.2 s** are physically replayed
with all recorded states and commands verified. Fresh inference then reaches the
original exit at **9.0 s**: **2.8 s / 28 commands** are new. Minimum fresh speed
is **3.579 m/s**, with minimum recorded fresh corner clearance **0.769 m**.
There is no recorded collision or departure. Every native identity, original
controller, tracking, signal-reuse and sampling audit passes. The fresh suffix
uses 68 pools, 26 admitted fresh decisions and 17 cached commands (two after
fresh sampling exhaustion, 15 ranked better). The new rule permits one
green-to-unknown and 16 matching-unknown reuses. Startup-inclusive wall time is
972.7 s. The owned CARLA server stops successfully after auditing.

The unknown upper-speed retry is exercised 17 times in this fresh suffix.
Within six saved inputs, it alone causes additional sampling and selects a
lower native target each time. One of those fresh choices is actually executed
(5.366 to 4.329 m/s at 7.0 s); the other five lose to revalidated cached plans.
This distinguishes an improved fresh proposal from an executed speed change.
See `resume-straight-unknown-enabled/upper-retry-effect.json` and the capture's
combined audit. It is not a counterfactual full-rollout comparison.

This is a successful continuation, not a new complete fresh run. Next is the
predeclared `02-straight-offset-faster-junction` full fresh check. The same final
profile will be promoted to the simple launcher only after that passes, then
tested from time zero on `04-left-centered-junction` through that launcher.
