"""Compare requested and reported controls with native versus acknowledged RPCs.

No driving-model inference. Run only while CARLA is free of other rollout clients.
"""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace

import carla
import yaml

from local import run_intersection as runner
from local.experiments.acknowledged_preroll import initialize_ego
from local.experiments.frame_sync import wait_for_frame
from module.carla_interface import CARLAInterface


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=40)
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
        for mode in ("native", "acknowledged"):
            interface = CARLAInterface()
            interface.client, interface.world = client, world
            try:
                initialize_ego(interface, config["junction_entry"], runner.rollout_specs(config)[0])
                for step in range(args.steps):
                    # Distinct successive commands expose stale control reads.
                    control = carla.VehicleControl(throttle=.15 if step % 2 else .25)
                    if mode == "native":
                        interface.ego_vehicle.apply_control(control)
                    else:
                        response = client.apply_batch_sync([
                            carla.command.ApplyVehicleControl(interface.ego_vehicle.id, control)], False)[0]
                        if response.error:
                            raise RuntimeError(response.error)
                    frame = interface.tick()
                    wait_for_frame(world, frame)
                    interface.get_camera_images(timeout=30)
                    actual = interface.ego_vehicle.get_control()
                    records.append({"mode": mode, "step": step, "frame": frame,
                                    "requested_throttle": control.throttle,
                                    "reported_throttle": actual.throttle,
                                    "mismatch": abs(actual.throttle-control.throttle) > 1e-5})
                args.output.write_text(json.dumps(records, indent=2) + "\n")
                print(json.dumps({"mode": mode, "steps": args.steps,
                                  "mismatches": sum(r["mismatch"] for r in records if r["mode"] == mode)}), flush=True)
            finally:
                actors = list(interface.sensors.values())
                for sensor in actors:
                    sensor.stop()
                if interface.ego_vehicle is not None:
                    actors.append(interface.ego_vehicle)
                client.apply_batch_sync([carla.command.DestroyActor(actor.id) for actor in actors], False)
    finally:
        world.apply_settings(previous)


if __name__ == "__main__":
    main()
