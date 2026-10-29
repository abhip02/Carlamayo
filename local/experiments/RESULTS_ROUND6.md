# Model-only validation and resampling — round 6

Branch: `experiment/driving-plan-validation`, starting checkpoint `5b94733`.
Evidence root: `outputs/driving-investigation-6/`.
Previous results: [round 5](RESULTS_ROUND5.md).

The user explicitly chose to keep executed paths model-generated. There is no
map-based fallback planner, trajectory correction, forced cruise speed, or
production controller change. This round tests a map-assisted **selection**
layer: CARLA lane geometry is used to evaluate model samples, and the chosen
waypoints are an exact slice of the original model output. This is a different
policy from unmodified first-sample inference; the map is now selection input,
not just evaluation data as in round 5.

## Frozen candidate availability

Completed 25 inference calls, four samples each: **100 proposed trajectories**.
All use equidistant preprocessing, language seed 43 and the existing format
guard. Motion seeds 42, 43 and 44 vary the model's native sampling noise.

- Left: four variants on the native round-5 left run's frames 019, 045, 065,
  095. Three left-command noise pools plus finish-turn/then-straight with seed
  42. At approach frame 045, the original four samples all exceed the 3 s lane
  corridor; one candidate each under motion seeds 43 and 44 remains inside it
  (maximum rear-axle distances 1.660 m and 1.399 m). On frame 019, all targets
  are still low. At 065/095 the actual car is already far off-route and none
  passes; resampling is not a guaranteed recovery method.
- Straight: three noise pools on round-4 `capture-ack-green-43` frames 019,
  045, 057. At 019, the original first target is 0.015 m/s, but another valid
  sample requests 0.842 m/s. At 045, each pool contains valid straight samples.
  At the later bad state 057, none passes; best maximum lane deviation remains
  10.48–11.39 m over the next 3 s.

Evidence: `replay-left-pools/`, `left-pool-audit.json`,
`replay-straight-pools/`, `straight-pool-audit.json`.

## Implemented experimental wrapper

`validated_probe.py` wraps the existing capture process. The normal launcher
does not import it. `validated_selection.py` rejects missing trajectory markers,
nonfinite points, signed reverse velocity below -0.1 m/s, predicted rear-axle
positions outside the mapped lane half-width, or moving-path headings more
than 45 degrees from the route. The initial horizon is 3 s.

The lane checks cover the rear axle, not the full vehicle footprint, and there
is no obstacle model. They do not certify safety or general traffic behavior.
Tests use the existing isolated empty intersection.

Among admitted samples, the score combines lane deviation and progress/speed
agreement. A separate image-based signal question supplies red/yellow/green/
unknown; simulator signal states are logged but not read by the selector. Green
uses a 5 m/s preference, unknown uses the current speed, and red/yellow prefer
shorter sampled paths. This preference only ranks model samples and never
modifies their speed profiles or the controller. Red behavior still needs live
validation.

If no sample passes, request another pool of four using the next motion seed,
up to three pools. If an entire pool lacks markers, also vary the language seed.
If all pools fail, end the experiment explicitly and record every rejection.
Do not execute the least-bad invalid sample or substitute a fallback path.

Left instructions advance to finish-turn/then-straight after 60 degrees of
observed rotation, and to lane following after reaching the exit. A configured
exit position/heading check can end capture at the target exit, avoiding an
unrequested next maneuver or exhaustion of the finite map reference. This is
reported as `route_exit_reached`, and does not erase earlier lane/stopping errors.
All other errors remain errors.

Every proposal and raw candidate is saved under `proposals/`. Root `frame_*.json`
records the selected candidate, signal result, instruction phase, rejection
metrics, and original candidate array, so the executed path can be checked for
exact identity. Full input captures are under `proposals/inputs/`. After the
first live process started, failure logging was extended to save the full input
under `inputs/` when all pools fail; this logging-only addition applies to later
launches. The first process's exact earlier sources are preserved by manifest
hash in its `source_snapshot/`.

## Live execution log

- `capture-validated-left-43`: up to 16 s, or earlier explicit route completion/
  validation failure. Four samples, seed 43, initial motion seed 42, original
  controller, acknowledged initialization/controls, snapshot waiting, ego-green
  layout. Ends explicitly at 3.0 s after all 12 proposed paths leave the lane
  somewhere in the fixed 3 s future window. No near-stop occurs: minimum actual
  speed 1.1628 m/s, maximum actual center deviation 0.0427 m. This improves the
  early stop but is an incomplete rollout, not a driving fix. All 30 executed
  trajectories remain native samples. The exact failure input was not saved by
  this initial process; all 12 rejected arrays/metrics and preceding saved inputs
  are preserved. Later launches include the added full-input failure save.
- Inspecting the actual original-controller steering targets on those rejected
  paths places them 1.5–1.8 s ahead, at 4.555 m lookahead. All paths stay inside
  the lane through those targets (maxima 0.56–1.35 m), before drifting later.
  This motivates a separate adaptive-window test, not accepting the full 3 s
  plans as valid.
- The revised validator checks at least the 1.0–1.5 s speed-preview window and
  the exact original pure-pursuit steering target, including all preceding
  waypoints. It extends farther for slow/short trajectories where the 4 m
  minimum lookahead reaches later points. The nearest-point starting index is
  also logged. Lane-width, marker, reverse and heading rejection bounds stay
  the same. `left-v1-short-horizon-audit.json` preserves the re-evaluation.
- `capture-validated-left-short-43`: prepared 16 s maximum live test of this
  revised window; pending execution at this checkpoint.

## Verification and rollback

Five focused tests cover rejection of wrong-lane/reverse/malformed plans,
green versus red ranking, exact original tensor/associated-text selection,
explicit failure without a fallback, and correct exit recognition. Combined
with existing controller, adapter, dataset and VerifAI tests: **68 passed in
3.37 s** before the failure-input logging addition. Follow-up verification will
be recorded with final results. After the logger addition all five focused tests
pass again (2.33 s); after adding controller-target coverage, all six focused
tests pass (2.45 s). The new test rejects a path that is initially in-lane but
leaves it before reaching the controller's actual distant steering target.

Production defaults, controller, VerifAI launchers, model weights and external
model sources remain unchanged. Return to `experiment/driving-turn-instructions`
or checkpoint `5b94733` to discard this round after saving it. Outputs stay local.
