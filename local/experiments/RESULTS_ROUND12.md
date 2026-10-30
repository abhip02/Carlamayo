# Five short scenario variants — round 12

Policy branch: `experiment/driving-exit-signal-transition`.
Frozen policy checkpoint: `cc9da24`; transition-rule rollback: `68efac1`.

The user requested five different short scenarios after the completed fresh
left turn in [round 11](RESULTS_ROUND11.md). The predeclared suite is
[`presets/five-short-scenarios.json`](presets/five-short-scenarios.json).
All five start fresh, with no archived prefix or policy tuning between runs.
Every executed path/speed remains model-generated; production launchers,
controller, decoder and model weights remain unchanged.

| Scenario | Maneuver | Start before entry | Initial speed | Lateral offset | Heading offset | Text / flow seed | Cap |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 01 | Straight | 5 m | 3 m/s | 0 m | 0° | 44 / 43 | 10 s |
| 02 | Straight | 5 m | 4 m/s | +0.25 m | −2° | 45 / 44 | 10 s |
| 03 | Straight | 10 m | 3 m/s | −0.25 m | +2° | 46 / 45 | 10 s |
| 04 | Left | 5 m | 3 m/s | 0 m | 0° | 44 / 43 | 12 s |
| 05 | Left | 5 m | 4 m/s | +0.20 m | −2° | 45 / 44 | 12 s |

These are variations at the same empty, all-green Town03 intersection, not five
different maps or traffic situations. The current screening implementation
supports straight and left routes. Right turns are not included. Start-position
and seed changes are combined to test different scenarios, not isolate causes.

Outputs: `outputs/driving-investigation-12/`. Each capture saves its expanded
configuration, options, source hashes, predictions, physical states and videos.
Initial model inputs and automatic failure captures are retained. The existing
independent native/control, footprint, route, retention, ranking and tracking
audits will be run on each result, including incomplete runs. A capture with no
executed command is a failure, not a pass with empty audits.

Completion requires the recorded route-exit marker. Reaching a duration cap
alone is labeled incomplete for the maneuver. Report any collision, recorded
corner departure, speed below 0.5 m/s, native provenance failure, or controller
mismatch. Minimum recorded speed and clearance will be reported even if all
binary checks pass. Short-horizon checks are not a full-trajectory, swept-volume
or obstacle guarantee. Every failure remains in the final suite summary.

Status: prepared; results pending.
