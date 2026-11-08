# Camera startup investigation — 2026-11-07

**All six model-free left-scene trials passed: original 3/3, pause 3/3.** No
crash was reproduced and no pause benefit was demonstrated. The pause is not
promoted to driving code. Driving is not fully fixed. The successful NF4
T-junction candidate uses 39.0% fewer native
calls, but takes 8.19 minutes for 11.4 simulated seconds. The subsequent left
case crashed before inference, and tight bend was not attempted. See the
[driving result](RESULTS_NF4_POLICY.md) and [failed panel](RESULTS_NF4_FIXED_CASES.md).

## Evidence and unresolved cause

The T candidate v3 and left-panel v2 failures both occurred about 71 seconds
after their owned CARLA containers started, after four camera spawns and before
the first inference. Both exited 139 / signal 11. Their logs contain no useful
stack. `OOMKilled=false` cannot rule out a GPU allocation failure. Left had
23.55 GiB free on its renderer at the startup gate, which weakens a simple
insufficient-startup-headroom explanation. Peak memory during that failure was
not measured. A model-free T camera smoke and a later full T rollout passed.

Read-only GPU0 checks at 2026-11-08 07:03 UTC showed zero remapped rows, no pending
remapping or repair, and 23 C current temperature. ECC was disabled, so its
error counts were unavailable. These are post-event counters, not a crash-time
diagnostic. Kernel `dmesg` access was denied and the journal explicitly hid
system/other-user messages. There is **no basis to claim that no GPU Xid error
occurred**. The owned crashed container's temporary binary-named file was empty;
no large dump was copied.

The ordinary `closed_loop.py` and `live_open_loop.py` have a one-second pause
after camera/collision-sensor setup. Ordinary model loading occurs before CARLA
setup; only OOM-free mode defers it. An earlier note incorrectly generalized the
deferred order and is corrected. Experimental `acknowledged_preroll.py` calls
camera setup, then immediately disables physics and begins its history ticks.
This sequence difference is a **hypothesis**, not an established crash cause.

## Prospectively fixed experiment

`left_camera_study.py` is a separate harness. All 125 prior Python files remain
unchanged. It reuses the model-free camera diagnostic and acknowledged pre-roll;
only the treatment arm sleeps one second immediately after camera setup, before
physics/history commands. It does not move collision-sensor creation, so it does
not claim to reproduce the entire ordinary loop.

| Trial | Camera setup pause |
| --- | ---: |
| 1 | 0 seconds |
| 2 | 1 second |
| 3 | 1 second |
| 4 | 0 seconds |
| 5 | 0 seconds |
| 6 | 1 second |

The three observations per arm are declared before any outcomes. Each uses a
fresh owned container/output, the failed left capture's unchanged JSON config,
renderer GPU0, port 2000, overhead sensor, same four 1920×1080 cameras, requested
3 m/s start and acknowledged pre-roll. Every failure is retained. A completed
failed camera trial does not trigger an extra attempt; the next predeclared
trial proceeds only after verified cleanup. Resource refusal, interruption,
unreadable result or cleanup failure stops the study.

There is no model loading/inference, model GPU allocation, OpenDRIVE export,
route generation or controller setup. Shared Python imports may import torch
and adapter classes; CUDA is hidden in the child. The unmeasured model/processor
loading interval is omitted, with no invented delay to represent it. Therefore
these trials isolate a sensor-startup hypothesis and cannot establish full
capture equivalence, driving quality or inference latency.

Events record connection, pre-roll, camera setup, ticks, each successfully
collected four-camera frame, and actual final acknowledged history state. The
handover includes pose, speed and frame. A camera pass requires a successful
child, four retained history frames, image shape `[4,1080,1920,3]`, and a recorded
handover; cleanup is evaluated independently. Small-N outcomes cannot establish
a reliable fix.

## Logging, resources and bounds

Both arms retain the exact Docker image digest, CDI renderer assignment, host
network, 2 GiB shared memory, NVIDIA environment and CARLA arguments from
`projected_server.sh`. The shipped engine arguments are unchanged in version 2. Version 1 proposed
extra engine logging, but was never executed: review removed those flags because
support in this UE4.26 shipping build was unverified and extra engine logging
could perturb the suspected startup race. Version 1 remains preserved as
superseded. Identical client timestamp instrumentation remains in both arms.
The existing off-screen invocation follows
[CARLA 0.9.16's Docker guidance](https://carla.readthedocs.io/en/0.9.16/build_docker/).

Per-container `json-file` logging is limited to `max-size=10m,max-file=1`, using
[Docker's documented rotation options](https://docs.docker.com/engine/logging/drivers/json-file/).
Old log bytes may rotate; local copied logs and six container logs should each
total at most about 60 MiB. No checkpoint or image copy is created. Diagnostic
events are small, and no driving video is generated.

Each trial holds renderer/port leases, requires a fresh 60-second 9.8 GiB
headroom gate, bounds probes to five seconds, port readiness to 60 seconds, and
its child to 180 seconds. A dead server aborts the waiting child. Start/stop
helpers use owned process groups, bounded termination, and the frozen ownership
labels. Independent cleanup checks require the child gone, owned container
stopped, port closed, and both leases reacquired/released. No other job is killed.

Prospective study cost estimate: **12–18 minutes**, with a conservative **42-minute maximum**
including six gates and bounded startup/cleanup. Actual total was **696.04
seconds (11.60 minutes)**. Execution followed the separate benchmark red-pair
cleanup and explicit monitor handoff; no server was started during preparation.

## Frozen protocol and checks

Current root: `outputs/left-camera-study-20261107-v2/`. Version 1 was unexecuted.

- Protocol SHA256:
  `a43283d63e6efd3a1210eef6e43a9d93c2a0a8e892a91a3f2c4750cc6aac1093`.
- Config SHA256:
  `5d8803e4e05b94604b7338b0d45b5fb4fff5e275573113968da0f244512c2f2e`.
- Registered 2026-11-08 07:11:42 UTC; 127 Python files plus the server shell
  hashed, including all 125 prior Python files.
- Full CPU suite: **343 passed, one CUDA-only test skipped, 8.22 seconds**.
  Focused harness/process supervision: **25 passed, 5.43 seconds**. After
  removing the engine logging flags, all **14 harness tests passed in 0.19 seconds**.

Executed once after review and resource handoff, from this worktree (do not
rerun this completed output):

```bash
source local/env.sh
CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 \
  .venv/bin/python -m local.experiments.left_camera_study run \
  --output outputs/left-camera-study-20261107-v2
```

The runner refuses existing study status or trial directories and verifies all
frozen source/config hashes. Rollback is simply not invoking this diagnostic;
no driving default or model/controller implementation changed.

## Executed six-trial result

The study ran from 2026-11-08 07:19:01 UTC for 696.04 seconds and exited zero.
Every trial passed all 13 gate checks, collected 16 synchronized four-camera
history sets, retained four history images, and reached the acknowledged
handover. All planned trials were executed exactly once in the declared order.

| Trial | Pause | Camera result | Collected history sets | Cleanup |
| --- | ---: | --- | ---: | --- |
| 01 original | 0 s | Pass | 16 | Verified |
| 02 pause | 1 s | Pass | 16 | Verified |
| 03 pause | 1 s | Pass | 16 | Verified |
| 04 original | 0 s | Pass | 16 | Verified |
| 05 original | 0 s | Pass | 16 | Verified |
| 06 pause | 1 s | Pass | 16 | Verified |

Raw signed x/y/z and roll/pitch/yaw differences from the fixed historical left
handover are zero for every trial. Speed differs by only
`-2.942442733910866e-09 m/s` (the state readers use slightly different numeric
operations). All six match the first original arm exactly. Thus every observed
handover satisfies the declared per-axis 0.001 m / 0.001 degree / 0.01 m/s
tolerances. Both the historical reference and the first successful original
comparison rule were recorded before any trials; no best-match selection was
used. Camera success and handover matching remain separate measurements.

Time from camera-setup completion to the first collected frame was 0.420–0.481
seconds without a pause and 1.468–1.500 seconds with the pause included. These
are observed client collection times, not the first sensor callback or a
controlled latency benchmark. Sampled renderer free memory fell from roughly
15.7–15.9 GiB during early children to minima of 11.07 / 10.75 GiB in the last
two. The shared host changed during the study; no idle-device timing claim is
made. All gates still passed and no other job was stopped.

Independent final checks reconfirmed all six owned children gone, containers
stopped, port 2000 closed, and renderer/port leases reacquired and released.
There were **zero model loads and zero inference calls**. The complete outcomes,
raw handover deltas, event/log hashes, sampled memory and cleanup evidence are
in `outputs/left-camera-study-20261107-v2/camera-comparison.json` and the tracked
`artifacts/camera-startup-study-20261107.json`.

The next [model-loaded startup-prefix study](RESULTS_LOADED_CAMERA_STARTUP.md)
is prepared separately: at most four starts, no native or VQA inference, full
failed-left startup sequence through handover, and the reviewed allocator repair.
It has not run and waits for resource handoff and review.

This result provides no evidence for adding the pause. It also cannot exonerate
the startup path when a model is loaded: model allocation/loading time and
some pre-camera setup were explicitly omitted. The full-capture crash cause
remains unresolved, and left/tight driving validation remains incomplete.

## Read-only latency finding

The completed T candidate already made only one native proposal on every step:
114 calls for 114 control ticks. Its probe timers total 441.84 seconds, including
the periodic visual signal queries; the enclosing decision timers total 445.86
seconds out of 491.64 seconds rollout wall time. Median enclosing decision time
is 3.573 seconds. These shared-GPU observations explain the remaining expense;
they do not isolate model kernels or prove quantization acceleration.

`local/run_intersection.py::run_rollout` synchronously requests a prediction on
every 0.1-second simulated control tick. The ordinary `closed_loop.py` sync mode
does the same. Its optional async mode instead has a one-second wall-clock
submission interval and at most one pending worker request; that mode is not
active in these captures. Switching modes would change trajectory age and
closed-loop behavior, so no runtime improvement is claimed from its existence.

A future isolated test could reduce replanning frequency while continuing to
control from a validated, correctly anchored model plan between predictions.
It would need new steering/age/signal-response checks and live driving evidence.
No cadence change has been implemented. Further retry reduction alone cannot
remove the successful candidate's one-call-per-tick cost. Compact evidence and
the complete prospective camera protocol are retained in
`artifacts/camera-startup-study-20261107.json`.
