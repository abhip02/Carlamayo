# CarlaMayo progress — 2026-11-03

Latest driving work: [round 19](experiments/RESULTS_ROUND19.md) and the
[ten-video collection](../outputs/driving-investigation-19-showcase/VIDEOS.md).
All ten declared scenarios have accepted clips, using one fixed start each:
straight/left/right at the original intersection, two curved-road directions,
two roundabout exits, a tight bend, an elevated T-junction and a red-light stop.
All 1409 executed-command audits pass. This is a documented collection after
retries, not an untouched benchmark. Moving cases use map-target projection,
gain 1.40, a 4.5 m/s ceiling and bounded native sampling/reuse. The red case adds
an opt-in visual stop-hold guard. The T-junction's 9 cm margin and tight bend's
34.4 minute generation time remain limitations. See the report for exact sources,
commands, criteria and rollback. Earlier five-case work is complete and retained
in [round 18](experiments/RESULTS_ROUND18.md).

The sections below preserve the original integration and rollout history; their
GPU allocations and policy descriptions refer to those earlier checkpoints.

## What is working

- Created the separate `carlamayo/` repository on branch `local/sharded-intersection`, based on upstream `4914558`.
- Installed CARLA 0.9.16 in an owned Docker container. CARLA renders on host GPU 3; the launcher uses the host's existing NVIDIA CDI support.
- Reused the existing Alpamayo 1.5 checkpoint and ML environment, with a private environment for CARLA/OpenCV. No second model download or changes to the existing AlpaSim/Alpamayo repositories were needed.
- Sharded Alpamayo across host GPUs 2 and 1 using BF16/SDPA. Matching VLM and action-expert layers stay together: layers 0–19 on GPU 2, 20–35 on GPU 1. The model loads once per batch.
- Added a simple batch runner: initialize ego, collect camera/history inputs, predict a trajectory, apply controls, advance CARLA by 0.1 s, and repeat. Each rollout gets a fresh ego, history, sensors and controller.
- Kept CarlaMayo's upstream pure-pursuit steering and speed controller. This differs from AlpaSim's MPC and includes launch/speed rules; stop-start behavior is visible in the recordings.

## Scene and initialization

- Fixed Town03 intersection: junction 103, road 184, lane -1. Entry `(x, y) = (-6.094448, 118.324844) m`, approach yaw `89.637459°`.
- Clear noon, traffic lights frozen green, and no spawned traffic. Static map objects remain.
- Initial-state variables: distance before the entry, lateral offset, speed, and heading offset. Positive lateral offset and heading turn right.
- Position is `entry - distance * forward + lateral * right`; heading is `entry_yaw + heading_offset`.
- A scripted constant-speed/heading pre-roll fills 16 ego poses and four images per camera. Physics takes over at handover; requested and measured initial states are both saved.
- Navigation text: `Continue straight through the intersection.` This conditions the model; there is no enforced geometric route or target exit.
- **Original initialization:** explicit states or a Cartesian grid. The configured grid has distance `[10, 20, 30] m`, speed `[0, 3, 6] m/s`, heading `[-10, 0, 10]°`, lateral offset `0`: 27 combinations. This full grid has not been run.
- **VerifAI sampling is now implemented** on `feature/verifai-rollouts`: `bash local/run_verifai.sh` samples continuous uniform initial-state ranges, saves the exact states and scene seed, and runs the existing batch loop. See the VerifAI section below. The older slide example of independent discrete-uniform choices remains illustrative.

## VerifAI integration — 2026-10-28

- Created the personal fork `https://github.com/abhip02/Carlamayo` as `origin`; retained `aveeslab/Carlamayo` as `upstream`, fetched full history, and created `feature/verifai-rollouts` from the local sharded work. A push dry run succeeded; local experiment commits have not been pushed.
- Added `local/run_verifai.sh`, `local/configs/verifai.yaml`, and an isolated optional VerifAI 2.2.0 environment under `.runtime/verifai-venv/`. The model environment and rollout/controller implementation are unchanged.
- The launcher uses VerifAI's random feature sampler for distance, lateral offset, speed, and heading. It launches all samples in one existing batch. `--sample-only` prints a replayable YAML configuration without CARLA/model loading; saved `config.json` files can also be passed directly to `local/run.sh`.
- This first launcher provides passive initial-state sampling for the fixed empty intersection. The server/client extension below adds feedback; actor generation and Scenic scenario scripts are not implemented.
- All 16 simulator-free sampling/handoff tests passed. A dedicated GitHub Actions job runs these tests.
- End-to-end check: `outputs/verifai-integration-smoke/` contains two one-second sampled episodes, with 10 model calls each and one model load total. Both reached the duration limit; each front/overhead video has 11 frames. Episode wall times were 47.4 s and 34.1 s. Saved samples, sampler version, and replay configuration were verified. The dedicated CARLA container was stopped after the test.
- Updated the existing Drive deck to 20 slides: corrected sampling status on slide 13 and added slides 18–20 covering the implementation, server/configuration controls, validation and GitHub setup. Existing slide order and all four embedded videos were preserved. Rendered the deck and verified the Drive readback byte-for-byte; provenance and backup are in `outputs/documentation-verifai-20261028/`.

## VerifAI server/client — 2026-10-28

- Created `feature/verifai-server-client` from `9e0ac6a` on `feature/verifai-rollouts`. The fork remains `origin`; this branch has not been pushed.
- Added `bash local/run_verifai_server.sh`: starts a real `verifai.server.Server` subclass and one persistent CarlaMayo simulation client. CARLA still starts separately with `bash local/server.sh start`. The server samples one initial state, sends it to the client, evaluates the returned summary and feeds the score into the next sample.
- The specification is `no_recorded_collision`: +1 for reaching the episode duration without a recorded collision, −1 for a recorded collision. Infrastructure/model errors abort with an error status. This property does not check route following or reaching an exit.
- Uniform random sampling is the default; `--sampler halton` and `--sampler ce` select VerifAI's Halton and Cross-Entropy samplers. Episode count/ranges/seed remain in `local/configs/verifai.yaml`; `--limit` caps the count and `--duration` overrides episode length.
- The JSON socket client uses the existing model environment, without importing VerifAI there. The actual VerifAI `Server.run_server()` and specification monitor run in the isolated sampler environment. No Scenic scenario scripts are needed. Both launchers share `rollout_session()` to load Alpamayo once and restore CARLA settings after the batch.
- Outputs include the usual videos/logs, exact samples in replayable `config.json`, one scored result per line in `verifai-results.jsonl`, and overall status/counts in `verifai-summary.json`. The coordinator uses port 8888 by default; `--verifai-port 0` selects a free port.
- All 31 simulator-free tests passed (16 existing sampling tests and 15 server/client tests). They exercise real sockets/subprocesses, negative-score feedback and Cross-Entropy distribution updates, replayable output, missing/failed clients, and immediate reuse of the coordinator port. GitHub Actions runs both test files.
- CARLA validation: `outputs/verifai-server-client-smoke/` contains two one-second uniform episodes, both +1, with 10 model calls each, 11 frames per front/overhead video, and one model load. Episode wall times were 47.4 s and 34.2 s. The samples match the previous uniform launcher's seeded samples exactly.
- After fixing socket closure order to permit immediate port reuse, `outputs/verifai-server-client-ce-smoke/` verified the final protocol with two 0.1-second Cross-Entropy episodes, both +1 and one model load. The original batch launcher also replayed one saved sample successfully in `outputs/verifai-server-client-replay-smoke/`.
- Verified recorded scores, samples, videos, restored asynchronous world settings and removal of episode sensors/vehicles. Stopped the dedicated CARLA container afterward. Collision failures were exercised synthetically in tests; these short live runs did not record a collision.

## Runs completed

| Run | Requested distance / speed / heading | Measured handover speed | Simulated time | Model calls | Wall time |
| --- | --- | --- | --- | --- | --- |
| A: straight start | 20 m / 3 m/s / 0° | 2.95 m/s | 8 s | 80 | 247 s |
| B: angled start | 25 m / 4 m/s / +5° | 3.91 m/s | 8 s | 80 | 280 s |

Both runs completed without a recorded collision or GPU out-of-memory error. Both deviated from the straight-through instruction; completion demonstrates the integration, not successful navigation. The second prediction sequence changes direction substantially. Position, speed and heading all changed between runs, so the cause has not been isolated. Further driving investigation is paused.

The two runs are in `outputs/demo-two-starts/`. Earlier startup checks are in `outputs/first-start-cache-fixed/`. A fresh one-second run in `outputs/overhead-live-smoke/` verified live overhead recording with 10 sharded model calls.

## Videos and logs

- Each rollout saves the front camera, ego pose/speed/controls, predicted trajectories and generated text, initialization, and a summary.
- Added an independent overhead camera 80 m above the road, centered 15 m beyond the entry, with the approach pointing up.
- Cyan shows the actual rear-axle trail. Yellow shows the latest full 6.4-second prediction, anchored at the pose where it was generated. Predictions are replaced every 0.1 s of simulation.
- Overlays are drawn after rendering and are never passed to Alpamayo. The overhead view includes the original front view as an inset.
- Reconstructed both original runs under `outputs/demo-two-starts-overhead/` from saved poses and predictions, without new inference. This is a saved-pose replay, not a second physics experiment or a native CARLA recorder replay.
- The presentation includes both runs in both views. MP4 copies are encoded as H.264 for presentation/browser compatibility; simulation content is unchanged.

## Organization and reversal

- Integration commit: `738dee3`. Overhead recording/replay commit: `5d677a9`.
- Experiment code and configuration: `local/`. Generated runs and presentation backups: `outputs/`. Temporary dependencies, logs and caches: `.runtime/`.
- Before installation, verified and removed 28.4 GiB of redundant root Git packs; audit: `../logs/git-pack-cleanup-verification.json`. CARLA's image uses about 19.3 GiB. Scratch had about 43 GiB free after setup (historical reading).
- The only substantial installation outside this checkout is the CARLA Docker image. No Docker daemon or system configuration was changed.
- Stop/remove the owned container with `bash local/server.sh stop` / `bash local/server.sh remove`. Preserve desired outputs, then remove this checkout and the unused CARLA image to undo the setup. Do not globally prune Docker.
- See [README.md](README.md) for run commands, output details and the full rollback procedure.

## Presentation update

- Added the workflow, sharded integration, scene definition, initialization/probabilistic extension, and videos to the existing `abhi_thesis_updates.pptx`. The follow-up includes both rollouts: slides 14–15 show front views A/B, and slides 16–17 show overhead views A/B (17 slides total).
- The first 13 slides are unchanged by the follow-up. Initial documentation artifacts are in `outputs/documentation-20261028/`; the latest source backup, deck and renders are in `outputs/documentation-both-rollouts-20261028/`.
- Video assets are in a `CarlaMayo rollout videos` folder beside the deck on Drive. The PowerPoint embeds all four videos and includes Drive playback links. The Drive-edited source had converted prior embedded videos to posters; the follow-up restores movie data, while links remain the browser playback alternative.
- Deck: https://docs.google.com/presentation/d/1yKSWESk_3YMKYPaRN6RDF43gdp5yf-PY/edit
- Front video: https://drive.google.com/file/d/1mL6NOt5Ek9rSOolD030n4WtIMzBdp30-/view
- Overhead video: https://drive.google.com/file/d/10R5qxdlxMedw4lRRgyi7VGED94fDQlEx/view
- Run A front video: https://drive.google.com/file/d/1irYr8jaVTKbys64Gged4A8HpeOkHmjQ3/view
- Run A overhead video: https://drive.google.com/file/d/1hUe8mfhK59jkom3TQtE1Y750KvQ1wDG_/view

No new simulation or controller/model change was made for this documentation update.
