# Driving-test storage cleanup — 2026-11-04

At the user's request, removed obsolete saved inference inputs while preserving
the tested fixes, successes, failures, videos and reproducibility notes. No
driving code or settings changed during cleanup. Branch:
`experiment/ten-driving-scenes`; implementation checkpoint: `c2a0733`.

## Space reclaimed

Removed **451 files: 407 `.pt` tensors and 44 compressed `.pt.gz` tensors**,
totalling **42,513,374,542 bytes (39.59 GiB)**. These were confined to
`outputs/driving-investigation*/`, excluding every accepted round-19 capture.
Available filesystem space rose from 10.24 to 49.83 GiB at verification time;
the entire `outputs/` directory now occupies about 6.4 GiB.

The [cleanup inventory](artifacts/cleanup-20261104.json) records every removed
path and size, the accepted capture paths, retained input/video checksums,
before/after space readings and verification counts. Deletion used an explicit
file list, with path, type, size and modification-time checks before removal.

## Evidence kept

- All tracked experiment reports, presets, scripts, tests and Git history,
  including unsuccessful approaches and rollback commits. Start with the
  [investigation index](README.md) and [final ten-scene report](RESULTS_ROUND19.md).
- Every raw candidate prediction, chosen trajectory, executed-command record,
  audit, configuration, source snapshot, map, log and success/failure video.
  All **43,616 other files** in the investigation directories retained their
  original sizes and modification times.
- The [ten overhead videos](../../outputs/ten-accepted-videos/),
  [ten front-camera videos](../../outputs/ten-accepted-videos-front/) and
  [showcase index, archive and provenance](../../outputs/driving-investigation-19-showcase/VIDEOS.md).
  SHA-256 checks confirm all 20 files in the two requested folders are unchanged.
- All **12 saved input tensors** belonging to the ten accepted captures
  (**1.11 GiB**), verified unchanged by SHA-256. These include the initial input
  for each case and two additional inputs from the second roundabout exit.
- The model checkpoint, runtime environments, CARLA installation and unrelated
  projects. They were outside the deletion scope.

## What worked and what did not

The original reports remain the detailed record, including exact commands,
options, timestamps, metrics and limitations. This table is a navigation aid.

| Tested approach | Recorded outcome | Evidence |
| --- | --- | --- |
| Native-path validation, resampling and retention | The completed five-case collection reached 3/5; centered straight exhausted sampling and another straight case hit its duration cap. | [Round 16 results in the investigation index](README.md), [round 17 context](RESULTS_ROUND17.md) |
| Single-sample native, nearest-lane and commanded-route steering comparison | None of six runs passed every criterion. Commanded-route projection reached both exits, but near-stops, speed surges and left-turn corner overhang remained. | [Round 17](RESULTS_ROUND17.md) |
| Route projection plus bounded speed selection, speed ceiling, steering calibration and short native-plan reuse | 4/5 fresh original cases passed. The remaining straight case finished in a separate 12 s trial at 10.7 s. The original benchmark remains 4/5. | [Round 18](RESULTS_ROUND18.md) |
| Right-turn gain 1.18, then 1.30 | Gain 1.18 left the corridor; 1.30 formally passed with only about 2 mm clearance and was not selected for the showcase. | [Round 19](RESULTS_ROUND19.md) |
| Right-turn gain 1.40 | Passed with 0.168 m minimum corner clearance; selected for the ten-video collection. These were fresh trials, not identical-input comparisons. | [Round 19](RESULTS_ROUND19.md) |
| Native red-light stop without the hold guard | Initially stopped, then the upstream launch floor restarted the car; it crossed the stop line. Failed video and predictions remain. | [Round 19](RESULTS_ROUND19.md) |
| Achieved-stop hold using the model-perceived signal | The isolated red case stopped and held about 4.7 m before the line. This is controller-assisted stopping; live green-release behavior was not demonstrated. | [Round 19](RESULTS_ROUND19.md) |
| Final moving profile and separate red-stop profile | Ten accepted clips after documented development/retries, with 1409 execution audits passing. Empty roads, one fixed start per case; narrow 9 cm T-junction margin and expensive tight-bend sampling remain limitations. | [Final collection](RESULTS_ROUND19.md#final-collection) |

## Replay limits after cleanup

Older reports describe the artifacts available at the time of each experiment.
Paths in the removal inventory no longer exist. Exact frozen-input inference
replay requiring those tensors now needs a new capture; they were ignored by
Git and cannot be restored from its history. Saved predictions and command
records remain available for analysis and auditing.

The 12 retained inputs are sparse snapshots, not every frame of the accepted
videos. They support limited frozen-input comparisons, not a complete paired
quantization benchmark. New full closed-loop comparisons must rerun scenarios
with fixed settings and record all outcomes, rather than treating this selected
video collection as an untouched benchmark.

## Verification and Git

All 451 inventoried paths are absent. All retained tensor/video hashes match.
All **274 experiment tests pass** after cleanup (4.11 s; one existing
Transformers environment-variable deprecation warning). No simulator/model
rollout was needed for this storage-only change. The test runner uses the
existing model environment with pytest supplied by the sampler environment:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python - <<'PY'
import sys
sys.path.append('.runtime/verifai-venv/lib/python3.12/site-packages')
import pytest
raise SystemExit(pytest.main(['-q', 'local/experiments']))
PY
```

The driving implementation is already committed through `c2a0733`; the cleanup
commit adds this record and inventory. `outputs/`, environments and model
weights remain excluded from Git, so pushing the branch publishes code and
documentation, not the local videos or tensor captures. The fork remote is
`origin` (`https://github.com/abhip02/Carlamayo.git`). To publish this branch:

```bash
git push -u origin experiment/ten-driving-scenes
```
