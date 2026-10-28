"""One empty intersection, a sharded model, and sequential initial-condition sweeps."""
from __future__ import annotations

import argparse
from collections import deque
import csv
from datetime import datetime, timezone
import itertools
import json
import math
from pathlib import Path
import random
import re
import time

import carla
import cv2
import numpy as np
import torch
import yaml

from module import config as defaults
from module.adapters.alpamayo_1_5 import Alpamayo15Adapter
from module.carla_interface import CARLAInterface
from module.inference import extract_trajectory_samples
from module.trajectory_follower import TrajectoryFollower, planned_speeds, trajectory_to_world
from local.overhead import OverheadRecorder, rear_axle_xyz

ROOT = Path(__file__).resolve().parents[1]
DT = 0.1
FIELDS = ("distance_before_intersection_m", "lateral_offset_m", "speed_mps", "heading_offset_deg")
SNAPSHOT = "7aba8293c09993f2e125c6819df05d7fa3e873ea"


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + "\n")


def make_transform(pose):
    return carla.Transform(carla.Location(**{k: float(pose[k]) for k in ("x", "y", "z")}),
                           carla.Rotation(**{k: float(pose.get(k, 0)) for k in ("yaw", "pitch", "roll")}))


def rollout_specs(config):
    if "rollouts" in config:
        specs = config["rollouts"]
    else:
        grid = config["initial_conditions"]
        specs = [dict(zip(FIELDS, values)) for values in itertools.product(*(grid[k] for k in FIELDS))]
    for spec in specs:
        for key in FIELDS:
            if not math.isfinite(float(spec[key])):
                raise ValueError(f"Non-finite initial condition: {key}")
        if spec["speed_mps"] < 0 or spec["distance_before_intersection_m"] < 0:
            raise ValueError("Speed and distance before the intersection must be non-negative")
    return specs


def requested_pose(entry, spec):
    yaw = math.radians(entry["yaw"])
    distance = spec["distance_before_intersection_m"]
    lateral = spec["lateral_offset_m"]
    return {**entry,
            "x": entry["x"] - distance * math.cos(yaw) - lateral * math.sin(yaw),
            "y": entry["y"] - distance * math.sin(yaw) + lateral * math.cos(yaw),
            "z": entry["z"] + 0.5,
            "yaw": entry["yaw"] + spec["heading_offset_deg"]}


def destroy_ego(interface):
    # Only actors created for this rollout are touched.
    for sensor in interface.sensors.values():
        if sensor.is_alive:
            sensor.stop()
            sensor.destroy()
    interface.sensors.clear()
    if interface.ego_vehicle is not None and interface.ego_vehicle.is_alive:
        interface.ego_vehicle.destroy()
    interface.ego_vehicle = None


def initialize_ego(interface, entry, spec):
    """Render a constant-heading pre-roll, with physics active at the handover frame."""
    world = interface.world
    pose = requested_pose(entry, spec)
    blueprint = world.get_blueprint_library().find("vehicle.lincoln.mkz_2020")
    blueprint.set_attribute("role_name", "carlamayo_intersection")
    vehicle = world.spawn_actor(blueprint, make_transform(pose))
    interface.ego_vehicle = vehicle
    vehicle.apply_control(carla.VehicleControl(brake=1.0))
    for _ in range(15):
        interface.tick()
    # Preserve the settled suspension height instead of teleporting above the road.
    pose["z"] = vehicle.get_transform().location.z
    interface.setup_cameras()
    vehicle.set_simulate_physics(False)
    frames = deque(maxlen=defaults.NUM_FRAMES)
    yaw = math.radians(pose["yaw"])
    vx, vy = spec["speed_mps"] * math.cos(yaw), spec["speed_mps"] * math.sin(yaw)
    for index in range(defaults.NUM_HISTORY):
        if index < defaults.NUM_HISTORY - 1:
            ago = (defaults.NUM_HISTORY - 1 - index) * DT
            vehicle.set_transform(make_transform({**pose, "x": pose["x"] - vx * ago,
                                                  "y": pose["y"] - vy * ago}))
        else:
            # The last physical tick advances from t=-0.1 into the handover state.
            # Record the measured result: drag/suspension can cause a small deviation.
            vehicle.set_simulate_physics(True)
            vehicle.apply_control(carla.VehicleControl())
            vehicle.set_target_velocity(carla.Vector3D(x=vx, y=vy))
        interface.tick()
        interface.update_history(interface.get_ego_state())
        frames.append(interface.get_camera_images(timeout=30.0))
    interface.setup_collision_sensor()
    return frames, pose


def load_sharded_model(args, adapter):
    if torch.cuda.device_count() != 2:
        raise RuntimeError("Use local/run.sh with exactly two visible model GPUs")
    torch.cuda.set_device(0)
    device_map = json.loads((ROOT / "local/device-map.json").read_text())
    checkpoint = args.checkpoint or str(
        ROOT.parent / "cache/huggingface/hub/models--nvidia--Alpamayo-1.5-10B/snapshots" / SNAPSHOT)
    print("Loading cached Alpamayo 1.5 across two GPUs (BF16, SDPA)...", flush=True)
    model, processor = adapter.load_model(
        checkpoint_path=checkpoint, attn_implementation="sdpa",
        device_map=device_map, max_memory={0: "12GiB", 1: "12GiB"})
    model.eval()
    for index in range(2):
        print(f"Shard {index}: {torch.cuda.memory_allocated(index) / 2**30:.2f} GiB allocated")
    return model, processor


def add_video_frame(writer, rgb, elapsed, state, label):
    bgr = cv2.cvtColor(cv2.resize(rgb, (960, 540)), cv2.COLOR_RGB2BGR)
    cv2.putText(bgr, f"{label}   t={elapsed:.1f}s   v={state['speed']:.2f}m/s",
                (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
    writer.write(bgr)


def run_rollout(client, world, adapter, model, processor, config, entry, spec, out):
    out.mkdir()
    interface = CARLAInterface(camera_configs=adapter.source_camera_configs)
    interface.client, interface.world = client, world
    summary = {"requested": spec, "junction_entry": entry, "termination": "error"}
    writer = None
    overhead = None
    started = time.monotonic()
    elapsed = 0.0
    count = 0
    try:
        if config.get("overhead", {}).get("enabled", True):
            overhead = OverheadRecorder(world, entry, out, config.get("overhead"), label=out.name)
        frames, target = initialize_ego(interface, entry, spec)
        state = interface.get_ego_state()
        summary.update(requested_world_pose=target, actual_handover=state)
        write_json(out / "initial.json", summary)
        follower = TrajectoryFollower(interface.ego_vehicle)
        writer = cv2.VideoWriter(str(out / "video.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10, (960, 540))
        if not writer.isOpened():
            raise RuntimeError("Could not open the output video")
        fieldnames = ["time_s", "frame", "x", "y", "z", "yaw", "pitch", "roll", "speed",
                      "steer", "throttle", "brake", "inference_s"]
        with (out / "ego.csv").open("w", newline="", buffering=1) as csv_file, \
                (out / "predictions.jsonl").open("w", buffering=1) as prediction_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            csv_writer.writeheader()
            csv_writer.writerow(dict(time_s=0, frame=interface.frame_id, **state,
                                     steer=0, throttle=0, brake=0, inference_s=0))
            add_video_frame(writer, frames[-1][adapter.viz_camera_slot], 0, state, out.name)
            if overhead is not None:
                overhead.add_frame(interface.frame_id, interface.ego_vehicle.get_transform(),
                                   0, state["speed"], front_rgb=frames[-1][adapter.viz_camera_slot])
            for step in range(round(config["duration_s"] / DT)):
                xyz, rot = interface.get_history_in_local_frame()
                data = adapter.prepare_model_input(np.stack(frames, axis=1), xyz, rot,
                                                   round(elapsed * 1_000_000))
                data["camera_indices"] = torch.tensor(adapter.source_camera_indices)
                torch.manual_seed(config["seed"])
                torch.cuda.manual_seed_all(config["seed"])
                inference_started = time.monotonic()
                pred, extra = adapter.run_inference(model, processor, data,
                                                    navigation_text=config.get("instruction"),
                                                    navigation_weight=1.0)
                trajectory = extract_trajectory_samples(pred)[0]
                inference_s = time.monotonic() - inference_started
                if not np.isfinite(trajectory).all():
                    raise RuntimeError("Model returned non-finite waypoints")
                tf = interface.ego_vehicle.get_transform()
                world_trajectory = trajectory_to_world(tf, trajectory)
                overlay_trajectory = np.vstack((rear_axle_xyz(tf), world_trajectory))
                prediction_time = elapsed
                steer, throttle, brake, _ = follower.compute_control(
                    tf, world_trajectory, state["speed"], planned_speeds(trajectory))
                prediction_file.write(json.dumps(dict(time_s=elapsed, frame=interface.frame_id,
                    inference_s=inference_s, trajectory_ego=trajectory.tolist(),
                    reasoning=adapter.extract_cot_text(extra))) + "\n")
                interface.apply_control(steer, throttle, brake)
                interface.tick()
                state = interface.get_ego_state()
                interface.update_history(state)
                frames.append(interface.get_camera_images(timeout=30.0))
                elapsed = (step + 1) * DT
                count += 1
                csv_writer.writerow(dict(time_s=elapsed, frame=interface.frame_id, **state,
                                         steer=steer, throttle=throttle, brake=brake, inference_s=inference_s))
                add_video_frame(writer, frames[-1][adapter.viz_camera_slot], elapsed, state, out.name)
                if overhead is not None:
                    overhead.add_frame(interface.frame_id, interface.ego_vehicle.get_transform(),
                                       elapsed, state["speed"], prediction=overlay_trajectory,
                                       prediction_time=prediction_time,
                                       front_rgb=frames[-1][adapter.viz_camera_slot])
                if count == 1 or count % 10 == 0:
                    print(f"{out.name}: {elapsed:.1f}s / {config['duration_s']}s, "
                          f"speed={state['speed']:.2f}m/s, inference={inference_s:.2f}s", flush=True)
                if interface.get_collision_count():
                    summary["termination"] = "collision"
                    summary["collision"] = interface.get_last_collision_event()
                    break
            else:
                summary["termination"] = "duration_reached"
        summary["final_state"] = state
    except BaseException as exc:
        summary["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        summary.update(simulation_seconds=elapsed, inference_calls=count,
                       wall_seconds=time.monotonic() - started)
        if writer is not None:
            writer.release()
        write_json(out / "summary.json", summary)
        try:
            if overhead is not None:
                overhead.close()
        finally:
            destroy_ego(interface)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "local/configs/demo.yaml")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--limit", type=int, help="Run only the first N initial conditions")
    parser.add_argument("--duration", type=float, help="Override rollout length in simulation seconds")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--checkpoint", help="Override the cached checkpoint directory")
    parser.add_argument("--describe-scene", action="store_true", help="Select and print the junction, then exit")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    config.setdefault("seed", 42)
    if args.duration is not None:
        config["duration_s"] = args.duration
    if not math.isfinite(config["duration_s"]) or config["duration_s"] < DT:
        raise ValueError("Duration must be finite and at least 0.1 seconds")
    specs = rollout_specs(config)
    if args.limit is not None:
        if args.limit < 1:
            raise ValueError("--limit must be positive")
        specs = specs[:args.limit]
    random.seed(config["seed"])
    np.random.seed(config["seed"])
    client = carla.Client(args.host, args.port)
    client.set_timeout(120)
    print(f"CARLA server {client.get_server_version()}, client {client.get_client_version()}")
    world = client.get_world()
    if world.get_map().name.split("/")[-1] != config["map"]:
        world = client.load_world(config["map"])
    entry = config["junction_entry"]
    print("Junction entry:", json.dumps(entry))
    if args.describe_scene:
        return
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = args.output or ROOT / "outputs" / stamp
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "config.json", {**config, "junction_entry": entry, "expanded_rollouts": specs})
    original_settings = world.get_settings()
    original_weather = world.get_weather()
    traffic_lights = [(light, light.get_state(), light.is_frozen())
                      for light in world.get_actors().filter("traffic.traffic_light")]
    summaries = []
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = DT
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 10
        world.apply_settings(settings)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        for light, _, _ in traffic_lights:
            light.set_state(carla.TrafficLightState.Green)
            light.freeze(True)
        adapter = Alpamayo15Adapter()
        model, processor = load_sharded_model(args, adapter)
        write_json(output / "device-map.json", model.hf_device_map)
        for index, spec in enumerate(specs):
            label = re.sub(r"[^a-zA-Z0-9_-]", "_", str(spec.get("name", "rollout")))
            name = f"{index:03d}_{label}"
            print(f"Starting {name}: {spec}", flush=True)
            summaries.append(run_rollout(client, world, adapter, model, processor,
                                         config, entry, spec, output / name))
            write_json(output / "batch-summary.json", summaries)
        print(f"Finished {len(summaries)} rollouts: {output}", flush=True)
    finally:
        for light, state, frozen in traffic_lights:
            light.set_state(state)
            light.freeze(frozen)
        world.set_weather(original_weather)
        world.apply_settings(original_settings)


if __name__ == "__main__":
    main()
