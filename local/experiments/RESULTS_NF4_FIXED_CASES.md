# Fixed left-turn and tight-bend checks — 2026-11-07

**Version 2 prepared, not executed; waiting for the benchmark resource handoff.**
These are two candidate-only specification checks
of the unchanged NF4 policy that passed the T-junction. They do not have fresh
six-attempt baselines. Only the [completed T-junction pair](RESULTS_NF4_POLICY.md)
supports a measured 39.0% call-count reduction. Historical BF16 captures provide
the fixed scene inputs and context; they are not matched performance comparators.

The candidate remains: real NF4 checkpoint with 624 packed layers, BF16 compute,
no activation rounding or CPU offload; at most two native trajectory calls,
preferred preview band 2.5–5 m/s, route-projected native steering target, gain
1.4, 4.5 m/s controller speed cap, native cache age at most 3 seconds, and the
same signal/format/frame/command guards. No simulator, camera rig, model,
trajectory generator or controller implementation changes are introduced.

## Prospective cases and unchanged criteria

| Order | Original case | Text / flow seed | Requested start | Maximum simulation |
| --- | --- | --- | --- | --- |
| 1 | 02: intersection left | 44 / 43 | 5 m before entry, 3 m/s, zero lateral/heading offset | 15 s |
| 2 | 08: tight bend | 51 / 50 | Original lane-path start, 3 m/s, zero offsets | 28 s |

Exact entry poses, routes, instructions, overhead settings and original config
hashes are frozen in `outputs/nf4-fixed-case-panel-20261107-v2/jobs.json`.
Each YAML is copied byte-for-byte from its accepted fixed-case input. Relative
to the successful T candidate, options differ only in scene-specific seeds,
instruction and route fields. Precision, sampling and controller options match.

Each case must reach its declared exit, maintain recorded speed between 1 and
5.5 m/s, have no recorded collision or vehicle-corner corridor departure, pass
the full source/world/time/controller integrity audit, and respect the two-call
limit. Integrity alone or process exit zero alone cannot establish a driving
pass. No extra margin is claimed from a nominal pass; minimum clearance is
reported explicitly.

Run left first, then tight bend **only if left passes**. Stop after the first
failed driving/audit case, execution or cleanup error, unreadable result, or
resource refusal. Preserve that outcome and mark remaining cases unattempted.
No retries, GPU hopping, seed changes, extra baselines or red-light tests.

## Frozen implementation and resources

The new panel adapter is separate from the 117 immutable shared sources. It
reuses the existing policy capture and the reviewed server-death supervisor.
The frozen driver originally accepts cases 9/10; a scoped CLI adapter permits
only the current case number (2 or 8), preserving names and restoring the driver
module after each invocation. It does not relabel these cases as T-junctions.

The internal `baseline_options` field is a validation template derived from
historical case inputs with declared NF4 precision. It is not a newly executed
NF4 baseline. It exists to keep the original strict controller validator and
the two-option policy allowlist unchanged.

- Panel implementation commit: `9e2cf8b`.
- Root manifest SHA256:
  `979265dfe7e5417ab682ff71d707e004bef589a243d9a7577ce6cfb4aa8e9010`.
- Left child manifest SHA256:
  `da09dd814bd8761e29703166542f67cec4cd52cb957b4a9115a8d18f4acd36fc`.
- Tight child manifest SHA256:
  `f930a3fbe98e9f0607f8d133d1d3d6bc3baed4e58323407c03c19a5141176078`.
- 125 current Python sources are frozen; all 123 version 4 sources, including
  the 117 shared sources, are byte-identical. All 11 checkpoint files were
  reverified. No checkpoint is copied.

Version 1 was never executed and is preserved as superseded. Review found that
the frozen driver can return normally with a failed server-stop command. The
version 2 adapter now explicitly rejects that cleanup status before the next
case, even if the driving audit passes. Only the new adapter and its tests
changed; all scene configs/options/starts/seeds remain identical to version 1.

Both cases require the same model GPU1 and renderer GPU0, common GPU/port leases,
and a fresh 60-second gate with at least 10.0 GiB model and 9.8 GiB renderer free.
At 2026-11-08 06:29:28 UTC, instantaneous free memory was 10.227 / 23.547 GiB,
all leases were available, and port 2000 was closed. This is conditional
readiness, not a reservation or a completed sustained gate. The model GPU is
shared; runtime observations are diagnostic.

At 06:34 UTC the panel is queued behind the benchmark's fresh BF16 T comparison
(model GPU0, renderer GPU2). Wait for its explicit verified cleanup/handoff;
do not claim those devices from an instantaneous free-memory reading. The
panel retains its original model GPU1 / renderer GPU0 assignment.

Each capture is capped at 1,800 seconds. Estimated combined capture cost is
20–35 minutes, with startup, two resource gates, auditing and cleanup additional.
Worst-case capture budget is 60 minutes, reduced if the first case fails. The
server-death guard aborts a waiting capture promptly; Docker/GPU probes and
owned-process cleanup are bounded. Standard driving defaults remain unchanged.

## Verification and proposed command

Full CPU verification before the final cleanup-status refinement: **327 tests
passed, one CUDA-only rounding test skipped, 8.91 seconds**. Final panel and
supervisor regression checks: **21 passed in 4.95 seconds**. Checks cover
unchanged policy, scoped CLI case selection,
ordered one-time execution, stopping after a failed quality result despite a
normal launcher return, cleanup failures, unreadable captures, and restoration
of the frozen driver after interruption. Existing supervisor checks cover
server death, probe failures, deadlines, parent SIGTERM and process groups.
No GPU/model/server job was started while preparing this panel.

After coordination/review, from this worktree:

```bash
export CARLAMAYO_MODEL_GPUS=GPU-46394453-71bb-15b9-1612-860b5714f750
source local/env.sh
export PYTHONPATH="/scratch/apomalap/thesis/carlamayo/.runtime/quant-benchmark-deps:$CARLAMAYO_ROOT"
.venv/bin/python -m local.experiments.nf4_fixed_case_panel run \
  --output outputs/nf4-fixed-case-panel-20261107-v2 \
  --container-prefix apomalap-carlamayo-nf4-fixed-20261107-v2
```

`panel-status.json` will retain both planned outcomes. Each case has separate
driver status, supervisor status, capture and audit files. The two new outcome
slots are currently **unattempted**; no new driving or timing conclusion is made.
