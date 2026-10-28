"""Measure CARLA's client snapshot lag; no model inference or controller tuning."""
import argparse
import json
from pathlib import Path
import time
from types import SimpleNamespace


def wait_for_frame(world, frame, timeout=5):
    deadline = time.monotonic() + timeout
    while True:
        snapshot = world.get_snapshot()
        if snapshot.frame == frame:
            return snapshot
        if snapshot.frame > frame:
            raise RuntimeError(f"Another client advanced CARLA past frame {frame} to {snapshot.frame}")
        if time.monotonic() >= deadline:
            raise TimeoutError(f"CARLA snapshot did not reach frame {frame}")
        time.sleep(.001)


def main():
    import carla
    from local.run_intersection import connect_world, destroy_ego, initialize_ego, rollout_specs
    from module.carla_interface import CARLAInterface

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    client, world = connect_world(SimpleNamespace(host="127.0.0.1", port=2000), config)
    previous = world.get_settings()
    interface = CARLAInterface()
    interface.client, interface.world = client, world
    records = []

    def tick():
        frame = world.tick()
        early = world.get_snapshot()
        started = time.monotonic()
        matched = wait_for_frame(world, frame)
        records.append({"requested_frame": frame, "immediate_snapshot_frame": early.frame,
                        "matched_frame": matched.frame, "wait_seconds": time.monotonic()-started})
        interface.frame_id = frame
        return frame

    interface.tick = tick
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = .1
        world.apply_settings(settings)
        initialize_ego(interface, config["junction_entry"], rollout_specs(config)[0])
        for _ in range(40):
            interface.ego_vehicle.apply_control(carla.VehicleControl(throttle=.3))
            interface.tick()
            interface.get_camera_images(timeout=30)
        result = {"ticks": len(records), "stale_immediate_snapshots": sum(
            row["immediate_snapshot_frame"] != row["requested_frame"] for row in records),
            "max_wait_seconds": max(row["wait_seconds"] for row in records), "records": records}
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps({key:value for key,value in result.items() if key != "records"}))
    finally:
        destroy_ego(interface)
        world.apply_settings(previous)


if __name__ == "__main__":
    main()
