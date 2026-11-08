# Bounded sampling-policy comparison — 2026-11-07

Driving is not established as fully fixed. The ten accepted clips are selected,
map-assisted demonstrations at fixed starts in mostly empty Town03 scenes. They
do not establish raw Alpamayo path accuracy or reliability in traffic. Repeated
sampling also makes some accepted runs expensive. This study tests a smaller
sampling budget without changing the simulator, native trajectory generator,
map projection, speed controller, retention checks, or signal handling.

Branch: `experiment/nf4-sampling-quality`, based on `e1f38d6`. The separate BF16
latency investigation stays frozen at `056a83f` on
`experiment/driving-latency-validation`; its eight-job live pilot is still
unexecuted. This NF4 study cannot validate that BF16 pilot.

## Prospective comparison

The baseline uses at most six trajectory samples and prefers previews in
3–5 m/s. The candidate uses at most two samples and prefers 2.5–5 m/s. These are
selection preferences, not fabricated speeds. An already validated older model
plan may still be selected by the existing retention policy. The red-stop case
keeps exactly one native sample and forbids that retention path.

The baseline task registered T-junction and red-stop jobs before either outcome.
This task also declared the pair before either baseline capture existed. Both
sides use the same NF4 checkpoint, 624 packed linear layers, BF16 computation,
single-device placement, no activation rounding and no CPU offload. All 117
shared Python sources, 11 checkpoint files, configs and baseline options are
verified against that registration. Candidate validation permits exactly the
two selection-option changes above. No wall-time selection budget is enabled.

The original candidate protocol was unexecuted. Version 2 archives it and
declares a renderer placement amendment **before any candidate output**:
baseline renderer GPU2 → candidate renderer GPU0, both RTX A5000. The model
remains on GPU1. External jobs consume the original renderer's headroom. The
amendment changes no scene, seed, model, controller or CARLA image. Actual
runtime config, route and initial handover must still match; tolerances are
1 mm position, 0.001 degree rotation and 0.01 m/s initial speed.

GPU sharing and the renderer change confound wall-clock comparisons. Report
call counts and driving quality; timing is diagnostic, not an idle-device,
quantization speedup or real-time-driving claim.

## Frozen artifacts and execution

Local root: `outputs/nf4-policy-pilot-20261107/`.

- `prospective-declaration.json`: this task's pre-outcome declaration.
- `baseline-registration.json`: original baseline registration and hashes.
- `protocol-history/v1-before-resource-reporting-and-renderer-amendment/`:
  preserved, unexecuted first candidate protocol.
- `jobs.json`: aggregate two-case protocol, SHA256
  `e337747ad132ed783284a78289f5ffcb26ac7bfb848f6ec938a308d114814f51`.
- `sources/`: 120 frozen Python files, including the three new policy helpers.
- `09-t-junction-nf4/jobs.json` and `10-red-light-stop-nf4/jobs.json`: separate
  per-case manifests, so their driver status files cannot overwrite each other.
- `policy-comparison.json`: all planned outcomes, independent criteria and
  actual runtime pairing proof; missing/failed jobs remain visible.

The shared baseline manifest SHA256 is
`d32599c5790ac78fcef991a18e1d2fd0b3b71cc28fa7fa60636e900993496c99`.
All paths and per-file hashes are in the local protocol. The large checkpoint
and existing benchmark output are linked, not duplicated.

Only the candidate **T-junction** is authorized in the current resource window.
Baseline red was unattempted because the renderer memory guard failed; its
marker is preserved. Candidate red remains pending, with no valid red pair.
There are at most two additional candidate jobs in this protocol, no retries,
and a 1,800-second capture limit per job. No further case starts automatically.

Version 2's command omitted the dependency environment and is preserved as a
failed attempt. Version 3 uses the following environment and a new output root:

```bash
export CARLAMAYO_MODEL_GPUS=GPU-46394453-71bb-15b9-1612-860b5714f750
source local/env.sh
export PYTHONPATH="/scratch/apomalap/thesis/carlamayo/.runtime/quant-benchmark-deps:$CARLAMAYO_ROOT"
.venv/bin/python -m local.experiments.run_nf4_pilot \
  outputs/nf4-policy-pilot-20261107-v3/09-t-junction-nf4/jobs.json \
  --precision nf4 \
  --model-gpu GPU-46394453-71bb-15b9-1612-860b5714f750 \
  --render-gpu GPU-fd2c05bc-4b86-6451-49ae-c9e3f7f79580 \
  --container apomalap-carlamayo-nf4-policy-20261107-v3 \
  --cases 9 --headroom-seconds 60 --wall-timeout-s 1800

.venv/bin/python -m local.experiments.report_nf4_policy_pair \
  outputs/nf4-policy-pilot-20261107-v3/jobs.json
```

The frozen driver holds common GPU/port leases, checks sustained memory
headroom for 60 seconds, refuses an occupied CARLA port, and stops only its
owned container. Existing status/output directories cannot be reused. Other
users' jobs are left alone. A zero process exit code alone is not a driving
pass: route completion, speed, collision, full vehicle footprint, provenance
audit and the sampling cap all have independent checks.

## Verification and current results

Final pre-run software verification: **309 tests passed in 7.67 seconds**.
Checks cover strict option differences, checkpoint mutations, device assignment,
unattempted/resource-blocked/timeout outcomes, and rejecting integrity-only
success when driving criteria fail. The existing baseline validation remains
strict and still rejects candidate options outside this explicit adapter.

Baseline T-junction passed: 112 decisions over 11.2 simulated seconds,
187 trajectory calls, 15 preference misses, 9 retained decisions, speed
2.609–4.593 m/s, no footprint departures and approximately 8.75 cm minimum
corner clearance. Its model time was 669.70 seconds, rollout wall time 719.38
seconds, and whole capture process 767.72 seconds. Decision latency p50/p95 was
3.57/21.08 seconds under shared GPU load. This remains very far from real time.

The version 2 candidate failed during model loading after 50.41 seconds:
`PackageNotFoundError: bitsandbytes`. It produced **zero trajectory calls and
zero driving decisions**, so this is a startup failure, not a driving outcome.
The driver itself exits zero even for a failed child; the report correctly uses
the child return code (1). All 13 resource checks over 60 seconds had passed.
The owned simulator stopped successfully; the GPU/port locks were reacquired
and released and port 2000 was closed at 2026-11-08 05:27:50 UTC. No automatic
retry was launched. Both the error and the unattempted red case remain in
`policy-comparison.json`; there is currently **no completed comparable pair**.

The baseline owner confirmed that bitsandbytes 0.48.2 lives in the original
checkout's `.runtime/quant-benchmark-deps`, added through `PYTHONPATH`, rather
than in the shared virtual environment. CPU-only import with that existing path
succeeds. A corrected startup must explicitly record this environment change,
source the candidate checkout's `local/env.sh`, resolve its own controller
module, retain the failed first attempt, and obtain a fresh resource handoff.
This does not require a package installation, checkpoint copy or controller
change. The preflight evidence is `environment-preflight.json`.

One corrected-environment attempt is authorized as an explicit infrastructure
repair, with no further retry. Version 3's manifest hash is
`7af5c5caa85aaa07a534ef3c80c1f8e3c3411f01629dcbd15609670ad47c5c21`.
It links the unchanged failed status and preserves all 120 source hashes,
configs and options from version 2. The separate environment amendment was
written before any version 3 capture output.

The preflight has two stages: package metadata and module origins in the exact
child environment, then real imports with only `CUDA_VISIBLE_DEVICES` hidden.
The latter confirms CPU imports, not full visible-GPU execution. A stricter
initial assertion found that importing bitsandbytes with CUDA visible queries
device capabilities and initializes a CUDA context; that subprocess exited
without loading a model. The two-stage check avoids that resource allocation
before leases. Verified versions are Python 3.12.11, PyTorch 2.8.0,
Transformers 4.57.1, Accelerate 1.12.0, bitsandbytes 0.48.2, NumPy 2.3.5,
Hugging Face Hub 0.36.0, safetensors 0.7.0 and CARLA 0.9.16. Controller imports
resolve to the candidate checkout; dependencies resolve to the baseline's
existing installation. No driving outcome influenced this repair.

Version 3 passed 13 new resource checks and loaded the NF4 model successfully:
624 quantized layers, 7.09 GiB reported model allocation. However, the owned
CARLA container exited with **signal 11 / exit code 139** at 2026-11-08 05:34:18
UTC, just after four cameras were spawned. Its Docker `OOMKilled` flag was
false; this does not determine whether GPU memory pressure caused the crash.
The simulator log provides no stack trace sufficient to establish a root
cause. The rollout summary records missing frames from all four cameras,
**zero inference calls and zero simulated seconds**.

The model process remained waiting during cleanup after the server died. Once
the crash was confirmed, only that verified owned child was terminated with
SIGTERM. Its recorded child return code is -15 and capture process wall time
204.95 seconds. This is an infrastructure failure, not a sampling-policy
failure or a timed driving result. The driver stopped its own container with
return code 0. At 05:37:39 UTC, its model PID was gone, port 2000 was closed,
and the model/render/port leases were successfully reacquired and released.
No further retry or red-light run was launched.

| Planned job / attempt | Outcome | Native trajectory calls |
| --- | --- | ---: |
| Registered NF4 T baseline | Driving and provenance pass | 187 |
| T candidate v2 | Dependency startup failure | 0 |
| T candidate v3, explicit environment repair | CARLA camera-startup crash | 0 |
| Registered NF4 red baseline | Unattempted: renderer resource guard | 0 |
| Red candidate | Pending / unattempted, no valid baseline pair | 0 |

There are **zero completed comparable policy pairs**. The candidate has no new
driving video and no measured speedup. The existing saved-state screen remains
evidence for prioritizing two-attempt selection, not proof that it works in a
new closed-loop rollout. Both startup failures remain in the tracked compact
study record, alongside the baseline and unattempted red job.

[Tracked compact record](artifacts/nf4-policy-20261107.json) preserves the exact
metrics, both startup failures, source/checkpoint hashes, environment versions,
prospective amendments and cleanup proof. It is approximately 66 kB. Both local
protocol/output folders together occupy about 14 MB; no model copies or new
videos were created. The shared benchmark support is imported verbatim from
the other task's registered snapshot so this branch does not depend on its
uncommitted source edits.

Next diagnostic: in a coordinated resource window, test CARLA camera startup
without loading Alpamayo, using the same image, map and camera settings. Capture
engine logging and peak renderer memory before considering another expensive
model run. Do not silently substitute another renderer, relax driving criteria,
or count these failures as successful low-latency driving. The separate BF16
pilot still requires its originally declared idle-device conditions.

## Read-only crash comparison and prepared camera diagnostic

The baseline and failed candidate containers have the same immutable image ID,
command (`-RenderOffScreen -nosound -carla-rpc-port=2000`), environment, host
networking, 2 GiB shared memory, and `runc` runtime. Their CDI device requests
differ exactly by the declared GPU2 → GPU0 renderer change. Neither sets a
separate Vulkan adapter override. There is no engine log identifying the
actual Vulkan device; the configured single-device exposure alone is not such
a log. Runtime `config.json` files match exactly. The shared camera code and
Alpamayo 1.5 rig match, and startup logs list the same four 1920×1080 cameras in
the same order. The overhead observer is also part of the configured scene.

Saved crash configuration only contains generic reporting settings; no useful
stack trace was produced. Therefore this inspection establishes **no demonstrated
controller or camera-configuration defect**. GPU memory pressure and renderer
instability remain hypotheses, not diagnoses. No frozen source was changed.
The comparison is saved in `outputs/nf4-policy-pilot-20261107-v3/container-comparison.json`.

The new standalone `camera_startup_smoke` diagnostic is **prepared, not run**.
It uses the existing acknowledged pre-roll, four-camera rig, overhead observer,
Town03 configuration, timestep and clear/green setup without calling any model
loader. CUDA is hidden from its Python client. It checks renderer headroom for
60 seconds while holding the renderer/port leases, refuses existing containers,
limits its camera child to 180 seconds, watches for an exited server, and stops
only the labeled owned container. It records shared-device free-memory samples
and server stderr; those are not measurements of isolated renderer memory.

For a newly coordinated resource window, select the renderer explicitly and
use a new output/container name:

```bash
source local/env.sh
.venv/bin/python -m local.experiments.camera_startup_smoke \
  --config outputs/nf4-policy-pilot-20261107-v3/09-t-junction-nf4/09-t-junction-nf4-cap2/config.json \
  --output outputs/camera-startup-smoke-NEW \
  --render-gpu "$CARLAMAYO_RENDER_GPU" \
  --container apomalap-carlamayo-camera-smoke-NEW
```

The exact render UUID must be recorded when scheduling this separate diagnostic.
CPU syntax and `--help` checks pass. Four CPU-only mocked guard checks also
pass: an existing container is refused, an occupied port prevents server start,
a dead server terminates the owned child, and a logging timeout still attempts
owned-server cleanup. These checks make no Docker/GPU calls. The diagnostic
has not been executed against CARLA, and a future camera-startup pass would
still not validate driving or latency.

Rollback: the accepted driving profile and ordinary launch defaults are
unchanged. Use `e1f38d6` or the accepted driving branch to omit this experiment.
No candidate policy is promoted based on one case. Broader fresh tests,
especially the tight bend and left turn, plus varied starts/traffic, remain
necessary before claiming reliable driving or changing defaults.
