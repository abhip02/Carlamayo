"""Re-render saved ego poses with overhead overlays; no model inference or new driving."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import carla
import cv2
import numpy as np

from local.overhead import OverheadRecorder, rear_axle_xyz
from module.trajectory_follower import trajectory_to_world


def state_transform(state):
    return carla.Transform(carla.Location(**{k: float(state[k]) for k in ("x", "y", "z")}),
                           carla.Rotation(**{k: float(state[k]) for k in ("yaw", "pitch", "roll")}))


def replay_rollout(world, source, output, config):
    output.mkdir()
    with (source / "ego.csv").open() as stream:
        states = list(csv.DictReader(stream))
    predictions = {int(row["frame"]): row for row in
                   (json.loads(line) for line in (source / "predictions.jsonl").read_text().splitlines())}
    front = cv2.VideoCapture(str(source / "video.mp4"))
    vehicle = None
    observer = None
    try:
        blueprint = world.get_blueprint_library().find("vehicle.lincoln.mkz_2020")
        blueprint.set_attribute("role_name", "carlamayo_overhead_pose_replay")
        spawn = state_transform(states[0])
        spawn.location.z += 0.5
        vehicle = world.spawn_actor(blueprint, spawn)
        vehicle.set_simulate_physics(False)
        vehicle.set_transform(state_transform(states[0]))
        observer = OverheadRecorder(world, config["junction_entry"], output, config.get("overhead"),
                                    label=f"{source.name} | saved-pose replay")
        for _ in range(10):
            world.tick()
        latest, latest_time = None, None
        for state in states:
            transform = state_transform(state)
            vehicle.set_transform(transform)
            # Reconstruct the same 'latest available' prediction as the live recorder:
            # a plan produced at frame k appears after its first control step, at k+1.
            new_prediction = predictions.get(int(state["frame"]))
            frame_id = world.tick()
            ok, bgr = front.read()
            observer.add_frame(frame_id, transform, float(state["time_s"]), float(state["speed"]),
                               prediction=latest, prediction_time=latest_time,
                               front_rgb=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB) if ok else None)
            if new_prediction is not None:
                latest = np.vstack((rear_axle_xyz(transform), trajectory_to_world(
                    transform, np.asarray(new_prediction["trajectory_ego"]))))
                latest_time = new_prediction["time_s"]
        (output / "replay.json").write_text(json.dumps(dict(
            source_rollout=str(source.resolve()), mode="saved-pose replay",
            model_inference=False, new_physics_rollout=False, frames=len(states),
            note="Vehicle poses replay ego.csv; plans replay predictions.jsonl. Front inset is original video."
        ), indent=2) + "\n")
        print(f"Rendered {len(states)} frames: {output / 'overhead.mp4'}", flush=True)
    finally:
        front.release()
        try:
            if observer is not None:
                observer.close()
        finally:
            if vehicle is not None and vehicle.is_alive:
                vehicle.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True, help="Saved batch directory")
    parser.add_argument("--output", type=Path, required=True, help="New output directory")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    args = parser.parse_args()
    config = json.loads((args.source / "config.json").read_text())
    rollouts = sorted(path.parent for path in args.source.glob("*/ego.csv"))
    if not rollouts:
        raise ValueError("No recorded rollouts found")
    args.output.mkdir(parents=True, exist_ok=False)
    client = carla.Client(args.host, args.port)
    client.set_timeout(120)
    world = client.get_world()
    if world.get_map().name.split("/")[-1] != config["map"]:
        print(f"Loading {config['map']}...", flush=True)
        world = client.load_world(config["map"])
    original_settings = world.get_settings()
    original_weather = world.get_weather()
    lights = [(light, light.get_state(), light.is_frozen())
              for light in world.get_actors().filter("traffic.traffic_light")]
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.1
        world.apply_settings(settings)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        for light, _, _ in lights:
            light.set_state(carla.TrafficLightState.Green)
            light.freeze(True)
        for source in rollouts:
            replay_rollout(world, source, args.output / source.name, config)
    finally:
        for light, state, frozen in lights:
            light.set_state(state)
            light.freeze(frozen)
        world.set_weather(original_weather)
        world.apply_settings(original_settings)


if __name__ == "__main__":
    main()
