# Ten distinct driving scenes — round 19

Branch `experiment/ten-driving-scenes`; rollback `experiment/projected-speed-selection`
at `0b044ac`. The user requests ten videos, one start per scenario, including
straight/left/right at the existing intersection, different road layouts, and a
red-light or stop-sign case. This suite uses a red light.

## Declared cases and criteria

| ID | Scenario | Duration cap |
| --- | --- | --- |
| 01 | Original junction, straight | 15 s |
| 02 | Original junction, left | 15 s |
| 03 | Original junction, right, from its legal approach lane | 15 s |
| 04 | Curved road, left bend | 22 s |
| 05 | Curved road, right bend | 22 s |
| 06 | Roundabout, first exit | 30 s |
| 07 | Roundabout, second exit | 40 s |
| 08 | Tight curved road | 28 s |
| 09 | Elevated T-junction, right turn | 22 s |
| 10 | Original junction, red-light stop | 12 s |

All are in Town03 with no other vehicles or pedestrians. Moving cases set all
simulator lights green. The red case sets them red and records the actual CARLA
stop waypoint on the ego approach, 1.87 m before the junction entry. One initial
condition per case; no offset repeats. The exact road/lane/s anchors, instructions,
seeds, poses and duration caps are in `presets/ten-driving-scenes.json`.

The nine moving cases retain the round-18 target ceiling (4.5 m/s), steering gain
(1.18), six single-candidate maximum, 3–5 m/s preference, and original-plan reuse
up to 3 s. They require the declared endpoint, minimum measured speed >=1 m/s,
maximum <=5.5 m/s, no recorded collision or corner departure, and passing
source/world-anchor/time/controller audits. Map paths use connected forward CARLA
lanes; no lateral lane changes or geometric jumps are invented by the exporter.

The red case executes one fresh native prediction per step, with no selection
for motion and no cached substitute. The same steering projection and speed
ceiling apply. It must keep its entire front footprint before the stop line,
finish within 8 m of it, and hold speed <=0.2 m/s for the final 2 s. The normal
moving-case minimum-speed/endpoint criteria do not apply to this stopping task.
Simulator red state configures/checks the scene; it does not inject braking or
replace the model's prediction. Visual signal answers remain recorded.

All cases check live scene scope, including curved roads without nearby traffic
lights. Absence of a visible light is not relabeled green. Every candidate and
failed run is retained. A successful video collection will identify any retries
and the implementation used; a failed first attempt is not erased.

## Implementation and preflight

- `scene_routes.py`: forward lane route search and live empty-scene checks.
- `projected_probe.py`: right-turn and explicit route instructions; original
  prediction anchors/times remain intact.
- `stop_mission.py`: independent front-footprint stop-line and hold checks.
- `run_projection_suite.py`: optional manifest, frozen sources/manifest, and
  optional stop after a failed capture.
- `render_scene_suite.py`: annotated H.264 videos, measured plots and video index.
- `local/run_scene_suite.sh`: opt-in ten-case launcher; original launchers retain
  their existing behavior.

All ten routes pass map preflight: minimum lane width 3.5 m, maximum adjacent
sample distance <0.61 m. The new right-turn instruction, route connectivity,
red-stop sampling/hold rules and scene-scope checks pass with the existing tests:
268 tests pass in 5.40 s. Simulator/model backend files are unchanged.

Run (choose available GPU UUIDs first; this launcher starts/stops its owned server):

```bash
export CARLAMAYO_MODEL_GPUS=GPU-ce1bea0e-c85a-f48a-a1ad-5c2bc153e10b
export CARLAMAYO_RENDER_GPU=GPU-3f4e4372-21d9-bac9-f577-755a624eb99d
export CARLAMAYO_PROJECTION_CONTAINER=apomalap-carlamayo-ten-scenes
bash local/run_scene_suite.sh --output outputs/driving-investigation-19-ten \
  --stop-on-failure --cases 3 10 4 6 7 8 9 5 1 2
```

Use a new output directory for every attempt. The initial order exercises newly
added maneuvers first. Rendering uses a fresh owned container with the same pinned
CARLA 0.9.16 image. Other users' GPU jobs remain untouched; hardware availability
can change between attempts and each allocation is recorded.

Results pending.
