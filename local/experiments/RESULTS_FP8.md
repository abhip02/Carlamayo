# FP8 ten-scene experiment — 2026-11-04

Branch: `experiment/ten-driving-scenes-fp8`, based on
`experiment/ten-driving-scenes` at `e1f38d6`. The requested plural
`experiments/ten-driving-scenes` does not exist in this checkout; the singular
branch contains the ten-scene suite and its accepted BF16 captures.

## Precision and hardware

The loaded Alpamayo 1.5 checkpoint contains 11,078,526,194 BF16 parameters.
The available model GPUs are RTX A5000, compute capability 8.6. NVIDIA documents
native FP8 Transformer Engine support for Ada, Hopper and Blackwell, rather than
Ampere: [hardware support](https://nvidia.github.io/TransformerEngine/support_matrix.html).

The first trial uses `fp8-w8a8-emulated`: E4M3FN quantization with FP32 absmax
scales per output channel for weights and per input token for activations.
Values are clamped to ±448 and cast through `torch.float8_e4m3fn`, then
dequantized. This is a local numerical emulation, not an exported FP8 model,
native FP8 GEMM benchmark or memory-saving implementation. Weights retain BF16
storage after rounding once; input activations are rounded at each Linear call.
BF16 matrix multiplication and the original autocast remain. Native hardware
accumulation need not reproduce these numbers exactly.

Coverage: **624 Linear modules, 9,797,695,488 weights (88.4% of parameters)**,
including the vision transformer, language transformer, diffusion expert and
action projections. The vocabulary output head and tied embeddings are excluded.
Embeddings, convolution, normalization, attention matmuls, softmax, KV cache,
biases and trajectory decoding retain their baseline precision. Each capture's
`precision.json` lists every converted and excluded Linear and its checkpoint,
runtime and hardware. No calibration dataset is needed for dynamic scaling.

This resembles the rowwise dynamic W8A8 scheme described in
[PyTorch's Float8DynamicActivationFloat8WeightConfig](https://docs.pytorch.org/ao/stable/api_reference/generated/torchao.quantization.Float8DynamicActivationFloat8WeightConfig.html),
but uses the existing PyTorch 2.8 installation directly, without changing the
pinned model dependencies. `fp8-w8a16-emulated` is available as a weights-only
ablation; it has not been selected for the first full trial.

## Validation and fixed protocol

The initial regression run passes **283 tests**, including known E4M3 rounding
values on CPU/GPU, zero/small rows, dtype preservation, tied-head exclusion,
weight-only versus W8A8 behavior, keyword calls and invalid inputs.

The preserved straight-scene input runs through both BF16 and FP8 with the same
seeds, preprocessing and sampler. Both produce finite trajectories and the
trajectory marker. Mean trajectory displacement is **0.206 m**, final-point
displacement **0.858 m**, and preview speeds are **1.927 / 1.909 m/s** (BF16/FP8).
This one input does not predict closed-loop success. First-call timings are
4.27 / 5.44 s, including visual signal inference; they are not a speed benchmark.
Evidence: `outputs/fp8-preflight-20261104/comparison.json`.

The subsequent paired replay covers the preserved first input of **all ten
scenes**, running BF16 and then FP8 on the same A5000, inputs and seeds. All 20
inferences produce finite trajectories and valid trajectory markers. Generated
driving-token sequences are identical within all ten pairs. Mean trajectory
displacement, averaged over the ten cases, is **0.0526 m**. These are differences
from BF16 predictions, not errors against ground-truth driving trajectories.

| Initial input | Mean trajectory displacement | Final-point displacement | FP8 minus BF16 preview speed |
| --- | --- | --- | --- |
| Straight | 0.206 m | 0.858 m | -0.018 m/s |
| Left turn | 0.065 m | 0.280 m | +0.006 m/s |
| Right turn | 0.025 m | 0.050 m | +0.009 m/s |
| Left curve | 0.012 m | 0.014 m | +0.004 m/s |
| Right curve | 0.082 m | 0.137 m | +0.026 m/s |
| Roundabout first exit | 0.070 m | 0.169 m | +0.016 m/s |
| Roundabout second exit | 0.033 m | 0.074 m | +0.007 m/s |
| Tight bend | 0.016 m | 0.034 m | +0.004 m/s |
| T-junction | 0.009 m | 0.038 m | <0.001 m/s |
| Red light | 0.006 m | 0.024 m | +0.004 m/s |

Evidence: `outputs/fp8-paired-inputs-20261104/comparison.json`, per-input raw
records, input SHA256 hashes, exact source snapshots and `provenance.json`.
This simulator-free replay used a separate idle A5000 and its own lock while
the serial closed-loop trial continued. It sampled one saved initial input per
scene, not every frame or every candidate in the accepted captures.

[Matched-input comparison plot](../../outputs/fp8-paired-inputs-20261104/trajectory-disagreement.png)
([PDF](../../outputs/fp8-paired-inputs-20261104/trajectory-disagreement.pdf)).

The closed-loop trial keeps the original ten-case manifest, starts, instructions,
seeds, caps, gain 1.40, 4.5 m/s ceiling, six-candidate bound, 3 s validated reuse,
red-light visual stop-hold guard and independent audits. Each case gets one fresh
attempt, including failures, with no tuning based on its FP8 result. The historical
BF16 comparison is the accepted collection, which includes documented retries;
it is not a fresh paired BF16 benchmark. These remain empty-scene tests with
map-projected steering and controller assistance.

All ten saved case option dictionaries were compared against their accepted
BF16 counterparts: the only difference is the added precision selection.
The scene configuration dictionaries are identical. The check is saved in
`outputs/ten-scenes-fp8-20261104/protocol-comparison.json`.

## Reproduction

From the `carlamayo` root, select available model/render GPUs and a unique owned
renderer container. The launch manages that container and writes a fresh folder:

```bash
export CARLAMAYO_MODEL_GPUS=GPU-ce1bea0e-c85a-f48a-a1ad-5c2bc153e10b
export CARLAMAYO_RENDER_GPU=GPU-3f4e4372-21d9-bac9-f577-755a624eb99d
export CARLAMAYO_PROJECTION_CONTAINER=apomalap-carlamayo-fp8-20261104
bash local/run_scene_suite.sh --precision fp8-w8a8-emulated \
  --output outputs/ten-scenes-fp8-20261104
source local/env.sh
.venv/bin/python -m local.experiments.render_scene_suite outputs/ten-scenes-fp8-20261104
```

Do not pass `--stop-on-failure` when collecting all ten outcomes. The launcher
returns nonzero when any requested case fails capture, audit, video conversion
or server shutdown. Every result remains in `status.json`; per-case numerical
provenance is in `precision.json`. Full raw predictions and commands are retained.

The saved-input check uses `python -m local.experiments.fp8_replay --cases 1
--output outputs/NEW_REPLAY_FOLDER` after sourcing `local/env.sh` with one GPU.
Omit `--cases` to compare the preserved first inputs for all ten scenes.

## Closed-loop results

**10 / 10 fresh FP8 cases passed.** All ten requested cases were attempted once
with the frozen settings. No failure-driven tuning or substituted retry was used.

| Case | FP8 result | BF16 time | FP8 time | BF16 min. clearance | FP8 min. clearance |
| --- | --- | --- | --- | --- | --- |
| Intersection: straight | Pass | 9.2 s | 9.5 s | 0.832 m | 0.832 m |
| Intersection: left | Pass | 10.4 s | 10.4 s | 0.392 m | 0.418 m |
| Intersection: right | Pass | 6.2 s | 6.2 s | 0.168 m | 0.165 m |
| Curved road: left bend | Pass | 13.3 s | 13.3 s | 0.647 m | 0.648 m |
| Curved road: right bend | Pass | 11.9 s | 11.8 s | 0.637 m | 0.637 m |
| Roundabout: first exit | Pass | 17.6 s | 17.8 s | 0.459 m | 0.485 m |
| Roundabout: second exit | Pass | 28.7 s | 28.4 s | 0.594 m | 0.586 m |
| Tight curved road | Pass | 20.4 s | 19.3 s | 0.438 m | 0.448 m |
| T-junction: right turn | Pass | 11.2 s | 11.2 s | 0.091 m | 0.083 m |
| Red light: stop and hold | Pass | 12.0 s | 12.0 s | 0.828 m | 0.828 m |

The BF16 column is the historical accepted collection described above, including
its documented retries. Clearance is the recorded vehicle-corner margin to the
route corridor. The red-light case also has a separate front-footprint stop-line
and final two-second hold audit; its success includes the original visual stop-hold guard.

The narrowest FP8 corner margin is **0.083 m** at the T-junction, versus
0.091 m in the accepted BF16 capture. This remains a narrow margin in one
controlled, map-assisted test, not evidence of general driving reliability.

The tight-bend run uses **523 inference calls**, versus **759** historically.
Quantization changes which candidate plans and reuse windows satisfy the fixed
selection policy. Such differences change workload, so capture wall times do
not measure native FP8 speed. The emulation retains BF16 storage and computation.

The full FP8 capture/audit batch took approximately **154.2 minutes**.
All capture, audit and review-video commands returned zero; the owned CARLA
server stopped successfully. Annotated video generation also returned zero.
Final verification confirms the frozen Python sources and the intended 624-layer
quantization coverage in every capture. The regression suite passed 283 tests.

[All ten annotated videos](../../outputs/ten-scenes-fp8-20261104/VIDEOS.md) ·
[Measured speed and clearance](../../outputs/ten-scenes-fp8-20261104/scene-metrics.png) ·
[Comparison JSON](../../outputs/ten-scenes-fp8-20261104/fp8-comparison.json) ·
[Comparison CSV](../../outputs/ten-scenes-fp8-20261104/fp8-comparison.csv) ·
[Final verification](../../outputs/ten-scenes-fp8-20261104/final-verification.json)
