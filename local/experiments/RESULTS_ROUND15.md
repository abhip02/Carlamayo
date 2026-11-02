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
