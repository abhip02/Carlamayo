"""Compare measured initialization histories with and without snapshot waiting.

No model loading or driving-controller changes. Do not run beside a live rollout.
"""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

from local import run_intersection as runner
from local.experiments.camera_projection import wide_render_rig
from local.experiments.frame_sync import wait_for_frame
from module.carla_interface import CARLAInterface


def main():
    import carla
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument("--modes", nargs="+", choices=("native", "wait", "acknowledged"), default=["native", "wait"])
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    client, world = runner.connect_world(SimpleNamespace(host="127.0.0.1", port=2000), config)
    previous = world.get_settings()
    records = []
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = .1
        world.apply_settings(settings)
        for repeat in range(args.repeats):
            for mode in args.modes:
                interface = CARLAInterface(camera_configs=wide_render_rig())
                interface.client, interface.world = client, world
                ticks = []
                original_tick = interface.tick

                def tick():
                    frame = original_tick()
                    immediate = world.get_snapshot().frame
                    if mode == "wait":
                        wait_for_frame(world, frame)
                    ticks.append({"frame": frame, "immediate_snapshot": immediate})
                    return frame

                interface.tick = tick
                try:
                    spec = runner.rollout_specs(config)[0]
                    initialize = runner.initialize_ego
                    if mode == "acknowledged":
                        from local.experiments.acknowledged_preroll import initialize_ego
                        initialize = initialize_ego
                    initialize(interface, config["junction_entry"], spec)
                    xyz, _ = interface.get_history_in_local_frame()
                    dx = np.diff(xyz[:, 0])
                    # First 15 poses are teleported at exactly the requested speed;
                    # final handover tick uses physics and can differ slightly.
                    target = spec["speed_mps"] * .1
                    record = {"repeat": repeat, "mode": mode, "history_forward": xyz[:, 0].tolist(),
                              "max_preroll_step_error_m": float(np.abs(dx[:-1] - target).max()),
                              "last_step_m": float(dx[-1]), "measured_speed_mps": interface.get_ego_state()["speed"],
                              "stale_immediate_snapshots": sum(t["frame"] != t["immediate_snapshot"] for t in ticks),
                              "ticks": ticks}
                    records.append(record)
                    args.output.write_text(json.dumps(records, indent=2) + "\n")
                    print(json.dumps({k: v for k, v in record.items() if k not in ("ticks", "history_forward")}), flush=True)
                finally:
                    # A just-spawned collision sensor may not yet appear alive in
                    # the client's actor cache. Destroy acknowledged IDs directly.
                    actors = list(interface.sensors.values())
                    for sensor in actors:
                        sensor.stop()
                    if interface.ego_vehicle is not None:
                        actors.append(interface.ego_vehicle)
                    client.apply_batch_sync([carla.command.DestroyActor(actor.id) for actor in actors], False)
                    interface.sensors.clear()
                    interface.ego_vehicle = None
    finally:
        world.apply_settings(previous)


if __name__ == "__main__":
    main()
