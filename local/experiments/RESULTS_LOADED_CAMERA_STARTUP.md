# Model-loaded left startup prefixes — 2026-11-07

**Prepared, not executed.** The six [model-free camera trials](RESULTS_CAMERA_STARTUP.md)
all passed, with no demonstrated benefit from a one-second pause. They omitted
model loading and some pre-camera setup. This next diagnostic retains the full
failed left capture prefix, but stops before driving or visual-signal inference.
It cannot establish driving quality or inference speed. No pause is promoted.

## Exact prefix and cutoff

The child calls the unchanged `nf4_policy_pair.capture` entry point with the
failed left YAML/options copied byte-for-byte. That preserves route/OpenDRIVE
export, map/settings/weather/signal initialization, real NF4 model and processor
loading, model/probe setup, overhead sensor, 15 acknowledged settling ticks,
four-camera setup, 16 acknowledged history ticks, and collision-sensor setup.

After the original `acknowledged_preroll.initialize_ego` returns, a scoped wrapper
performs the same `get_ego_state()` read that immediately follows it in
`run_rollout`. It verifies four retained `[4,1080,1920,3]` images and records the
actual pose/speed, final acknowledged history state, requested pose and frame.
It then raises a dedicated `PrefixComplete` sentinel. All original capture,
model/probe, actor and world `finally` cleanup executes before the outer child
accepts that expected cutoff. A cleanup exception is an error, not a pass.

The sentinel precedes follower creation, video recording and the first inference
request. Scoped guards additionally block `Probe.evaluate`, VLM `generate`, both
native trajectory sampling entries and diffusion sampling before any underlying
inference. An attempted call increments a rejection counter and stops the study.
The frozen rollout summary will retain `termination=error` with a
`PrefixComplete` error, because it is intentionally not a completed rollout.
The separate prefix result recognizes only this explicit cutoff, with zero
simulated driving seconds and zero inference calls. It also rejects any generated
`frame_*.json` or `proposal_*.json` artifacts.

Acceptance requires the loaded precision report to show 624 NF4 layers, BF16
compute, no CPU offload, one device and the reviewed 0.45 allocator budget.
Camera/prefix success, handover matching and cleanup are separate measurements.
Raw handover deltas use the same fixed historical left reference and first
successful original arm rule as the earlier study, with 0.001 m / 0.001 degree /
0.01 m/s tolerances. Every failed or missing handover remains visible.

## Reviewed allocator repair and device amendment

The benchmark's NF4 red attempt failed before loading because Transformers'
allocator warmup requested 9.72 GiB while the frozen 0.41 process fraction
allowed only 9.65 GiB. Its reviewed repair raises only the NF4 fraction to 0.45
and records startup allocator peaks. It changes no quantization, inference,
trajectory or controller computation.

This study imports a separate byte-identical copy as `nf4_startup_loader.py`
and temporarily binds the policy capture's bridge reference to it. All original
117 shared sources and all 127 prior camera-study Python sources remain
unchanged. The repair is copied from the benchmark's immutable v9 source
snapshot, SHA256
`fe07bbd4ec257594f3a52da356810f7a1c71648b037ffb9a514033cff4e06b44`.
The copy records its own path/hash in precision evidence; it does not disguise
itself as the original loader. Checkpoint files remain linked and hash verified.

The failed left capture used model GPU1 / renderer GPU0. Model GPU1 now has less
than the required 11 GiB free. Before outcomes, this protocol therefore declares
**model GPU3 / renderer GPU0**, both RTX A5000. The renderer involved in the
observed failure is retained. There is no fallback device or gate reduction.
Changing the physical model GPU and shared host load prevents exact timing
equivalence to the failed capture; both comparison arms use the same new pair.
The benchmark currently owns GPU0/GPU3 for its separate red repair. This study
must wait for verified cleanup and explicit coordination handoff.

## Four predeclared starts and bounds

Order is **original, one-second pause, one-second pause, original**. All four
starts use fresh owned containers, fresh model processes and separate outputs.
The only arm difference is a one-second sleep immediately after camera setup
and before the original physics/history commands. Client events are identical
in both arms; the shipped engine arguments remain unchanged. Docker logs use
the earlier 10 MiB per-container rotation bound.

Each start acquires model, renderer and port-2000 leases together, then requires
60 seconds continuously above 11.0 / 9.8 GiB free. The capture prefix has a
240-second deadline, Docker probes five seconds, port readiness 60 seconds,
and bounded owned-process/group cleanup. Version 2 additionally samples free
memory for **both declared GPUs** while the child runs, at intervals of at least
five seconds with five-second query limits. Failed queries retain explicit errors
and no invented memory values. The prelaunch 11.0/9.8 GiB thresholds are not
reapplied after our own allocations begin. No query starts in the final five
seconds of the child budget, and server probes/waits respect the remaining
240-second deadline. Interruptions remain interruptions, not query errors.
Version 1 was never executed and is preserved as superseded; review added this
monitor because shared allocations can grow after a passed gate. The monitoring
is identical in both arms. A dead server aborts its waiting model
child. Every outcome is preserved. A camera/model startup failure can be followed
by the next predeclared observation; it never adds an attempt. Resource refusal,
cleanup failure, interruption, forbidden inference or corrupt result stops the
study. Maximum starts: four. Retries: zero.

Expected cost is **8–12 minutes**; conservative maximum **32 minutes** including
gates, startup and cleanup. These are startup attempts with **zero native or VQA
inference**, not four driving rollouts. No cadence, controller, backend, prompt,
scene, seed or sampling-policy change is made.

## Frozen inputs and verification

Root: `outputs/loaded-left-camera-study-20261107-v2/`.

- Protocol SHA256:
  `6f7ad9de8cd8d73a1390e32626fabac360d7617844c7c28a2e6fc2e0e1993c05`.
- Registered 2026-11-08 07:58:14 UTC; 130 Python sources plus the server shell
  hashed. Each trial has a separate hashed capture manifest.
- Full CPU suite before the final input-preflight refinement: **369 passed,
  one CUDA-only test skipped, 8.10 seconds**. Final prefix tests: **26 passed,
  3.91 seconds**. After adding runtime memory monitoring, focused prefix and
  process supervision checks: **40 passed in 4.71 seconds**. They cover exact cutoff, cleanup order, hook restoration,
  blocked inference paths, repaired-loader identity, memory gates, supervision
  failures and fixed trial/stop rules.
- CPU-only environment preflight confirms the local controller path and
  bitsandbytes 0.48.2 in the existing benchmark dependency directory. CUDA was
  hidden and uninitialized; no model or server was loaded.

After review and resource handoff, the exact environment is required:

```bash
export CARLAMAYO_MODEL_GPUS=GPU-ce1bea0e-c85a-f48a-a1ad-5c2bc153e10b
source local/env.sh
export PYTHONPATH="/scratch/apomalap/thesis/carlamayo/.runtime/quant-benchmark-deps:$CARLAMAYO_ROOT"
.venv/bin/python -m local.experiments.loaded_camera_study run \
  --output outputs/loaded-left-camera-study-20261107-v2
```

Do not invoke a child directly: the parent supplies the leases, gates, deadlines
and container ownership checks. Nothing has been launched from this protocol.
