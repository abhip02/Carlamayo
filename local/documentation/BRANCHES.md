# Carlamayo experiment branches

Source organization snapshot: **2026-11-17**. Remote: [abhip02/Carlamayo](https://github.com/abhip02/Carlamayo).

## Current work

| Branch | Scope | Source commit |
| --- | --- | --- |
| [`feature/verifai-scenic-traces`](https://github.com/abhip02/Carlamayo/tree/feature/verifai-scenic-traces) | Scenic initial-state sampling, full rollout traces, VerifAI feedback, CI tests and usage documentation | `027e6f8` |
| [`experiment/ten-driving-scenes-fp8`](https://github.com/abhip02/Carlamayo/tree/experiment/ten-driving-scenes-fp8) | BF16/FP8 precision controls, emulated FP8, paired replay and recorded results | `f94860c` |
| [`experiment/quantization-model-scores`](https://github.com/abhip02/Carlamayo/tree/experiment/quantization-model-scores) | Model-score benchmarks, quantization variants and guarded NF4 execution | `69f603b` |
| [`experiment/driving-latency-validation`](https://github.com/abhip02/Carlamayo/tree/experiment/driving-latency-validation) | Async inference, camera buffering, traffic panels and audits | `353cb89` |
| [`experiment/native-traffic`](https://github.com/abhip02/Carlamayo/tree/experiment/native-traffic) | Native trajectory traffic validation and steering safeguards | `62421d2` |
| [`experiment/nf4-sampling-quality`](https://github.com/abhip02/Carlamayo/tree/experiment/nf4-sampling-quality) | Fixed NF4 policy cases and startup resource gates, with earlier sampling-quality history | `ee862f9` |
| [`experiment/native-policy-seeds`](https://github.com/abhip02/Carlamayo/tree/experiment/native-policy-seeds) | Reproducible seed schedules, camera-policy controls and replay | `d94596f` |

These are experiment branches, not merged changes to `main`.

The VerifAI, FP8 and model-score branches form a stack: VerifAI → FP8 → model scores. Each branch adds a focused commit to the preceding one. Native-policy seeds builds on native traffic. Async latency and native traffic preserve separate working-tree snapshots of their shared earlier experiment base; they were not merged together.

The source commits above identify the code snapshots. Later branch-guide documentation commits do not change those source snapshots.

## Local worktrees

| Directory under `/scratch/apomalap/thesis` | Branch |
| --- | --- |
| `carlamayo` | `experiment/quantization-model-scores` |
| `carlamayo-latency` | `experiment/driving-latency-validation` |
| `carlamayo-native-traffic` | `experiment/native-traffic` |
| `carlamayo-nf4-policy` | `experiment/nf4-sampling-quality` |

The native-policy source was imported from `experiments/native-policy-20261117/source` into a full Carlamayo checkout. Its eight differing files were copied exactly, using native traffic as the parent. The original source snapshot and its independent Git history remain unchanged.

The old local `codex/native-traffic` name remains at `056a83f`; use `experiment/native-traffic` for the current implementation.

## Validation at organization time

Validation used existing tests, CPU-only execution and no simulator or model-weight loading:

| Suite | Result |
| --- | --- |
| Core tests on model-score branch | 97 passed, 3 skipped |
| VerifAI/Scenic sampling, sockets and traces | 78 passed |
| FP8/model-score/quantization tests | 20 passed, 1 skipped |
| Async-latency core and targeted experiment tests | 332 passed, 2 skipped |
| Native-traffic core and targeted experiment tests | 341 passed, 2 skipped |
| NF4 resumed fixed cases | 4 passed, 3 subtests passed |
| Imported native-policy core and targeted experiment tests | 417 passed, 2 skipped |

Counts overlap across branches. GPU cases and optional packages account for skipped tests. No fresh GPU inference or CARLA rollout was run for this Git synchronization.

Run core tests and quantization experiment tests in separate pytest processes: the core CARLA-interface tests install a module stub which conflicts with the experiment geometry tests when collected together. The combined run exposed five such failures; both separate runs passed.

## Historical milestones

Earlier `experiment/driving-*`, `experiment/projected-*`, `experiment/ten-driving-scenes`, `feature/verifai-*`, `fix/driving-native-validation`, and `local/sharded-intersection` branches retain their names and commits. They are preserved as experiment milestones alongside the current work.

To list the synchronized branches:

```bash
git fetch origin
git branch -r
git log --graph --decorate --oneline --all
```

Generated videos, weights, environments, caches and run directories remain local. This synchronization is limited to Carlamayo; it does not publish the top-level thesis workspace or update AlpaSim.
