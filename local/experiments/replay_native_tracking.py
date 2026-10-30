"""Replay saved native paths through the unchanged follower, then hold one plan.

Diagnostic only: no inference, path repair, speed override or map-generated path.
The prefix must reproduce the source state/control within explicit tolerances
before continuation. Continuation can exceed the live policy's cache age limit;
it tests tracking, and is never represented as a successful inference rollout.
"""
import argparse
import csv
import hashlib
import json
import random
from pathlib import Path
from types import SimpleNamespace

import carla
import numpy as np

from local import run_intersection as runner
from local.experiments.acknowledged_preroll import initialize_ego
from local.experiments.audit_turn import project
from local.experiments.frame_sync import wait_for_frame
from local.experiments.validated_selection import assess, pose_transform, require_green_progress
from module.adapters.alpamayo_1_5 import Alpamayo15Adapter
from module.carla_interface import CARLAInterface
from module.trajectory_follower import TrajectoryFollower, planned_speeds, trajectory_to_world


def native_plan(capture, label, candidate=None):
    record = json.loads((capture / (label + ".json")).read_text())
    proposal = json.loads((capture / "proposals" / (record["selection"]["proposal"] + ".json")).read_text())
    chosen = record["selection"]["chosen"]
    if ("retention" in record or record["trajectory_ego"] != proposal["candidate_trajectories"][chosen]
            or record["generated_token_ids"] != proposal["candidate_generated_tokens"][chosen]):
        raise ValueError(f"Native source identity failed: {label}")
    if candidate is not None:
        if not 0 <= candidate < len(proposal["candidate_trajectories"]):
            raise ValueError("Candidate index is outside the original native pool")
        provenance = {
            "original_chosen": chosen, "source_record_label": label,
            "proposal": record["selection"]["proposal"]}
        # Keep only metadata that belongs to this alternative. Original selected
        # text/speed summaries must not be mislabeled as the alternative's output.
        record = {"label": f"{label}_candidate_{candidate}", "context": record["context"],
                  "selection": {**record["selection"], "chosen": candidate},
                  "diagnostic_candidate_override": provenance,
                  "trajectory_ego": proposal["candidate_trajectories"][candidate],
                  "generated_token_ids": proposal["candidate_generated_tokens"][candidate],
                  **{k: proposal[k] for k in ("candidate_trajectories", "candidate_velocities", "candidate_yaw_rad")}}
    raw = np.asarray(record["trajectory_ego"])
    world = trajectory_to_world(pose_transform(record["context"]["world_pose"]), raw)
    speeds = planned_speeds(raw)
    world.setflags(write=False)
    speeds.setflags(write=False)
    return record, world, speeds


def state_error(actual, expected):
    return {
        "position_m": float(np.linalg.norm([actual[k] - expected[k] for k in ("x", "y", "z")])),
        "orientation_deg": max(abs((actual[k] - expected[k] + 180) % 360 - 180)
                               for k in ("yaw", "pitch", "roll")),
        "speed_mps": abs(actual["speed"] - expected["speed"]),
    }


def native_proposal(capture, label, candidate):
    """Load an untouched rejected proposal for a tracking diagnostic only."""
    proposal = json.loads((capture / "proposals" / (label + ".json")).read_text())
    options = json.loads((capture / "experiment.json").read_text())["options"]
    route_name = options["validation_route"]
    route = json.loads((capture / "route-reference.json").read_text())["routes"][route_name]
    if candidate is None or not 0 <= candidate < len(proposal["candidate_trajectories"]):
        raise ValueError("A proposal diagnostic requires an original candidate index")
    metrics = assess(np.asarray(proposal["candidate_trajectories"]), proposal["context"], route,
                     proposal["candidate_markers"], proposal["candidate_velocities"],
                     options["validation_horizon_s"], yaws=proposal["candidate_yaw_rad"],
                     check_body=True, body_margin_m=options["validation_body_margin_m"])
    require_green_progress(metrics, proposal["visual_signal"], options["validation_green_min_target_mps"])
    if not metrics[candidate]["valid"]:
        raise ValueError(f"Proposal fails original native validation: {metrics[candidate]['reasons']}")
    raw = np.asarray(proposal["candidate_trajectories"][candidate])
    record = {"label": f"{label}_candidate_{candidate}", "context": proposal["context"],
              "trajectory_ego": raw.tolist(), "generated_token_ids": proposal["candidate_generated_tokens"][candidate],
              "selection": {"proposal": label, "chosen": candidate, "route": route_name},
              "diagnostic_proposal": True, "native_admission": metrics[candidate],
              **{k: proposal[k] for k in ("candidate_trajectories", "candidate_yaw_rad", "candidate_velocities")}}
    world = trajectory_to_world(pose_transform(record["context"]["world_pose"]), raw)
    speeds = planned_speeds(raw)
    world.setflags(write=False)
    speeds.setflags(write=False)
    return record, world, speeds


def run(args):
    capture, out = args.capture, args.output
    out.mkdir(parents=True, exist_ok=False)
    config = json.loads((capture / "config.json").read_text())
    controls = [json.loads(s) for s in (capture / "controller-inputs.jsonl").read_text().splitlines()]
    with (capture / "000_verifai/ego.csv").open() as stream:
        reference = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    if len(reference) != len(controls) + 1:
        raise ValueError("Source prefix is incomplete")
    if args.prefix_seconds is not None:
        count = round(args.prefix_seconds / .1)
        if not 0 <= count <= len(controls) or abs(count * .1 - args.prefix_seconds) > 1e-8:
            raise ValueError("Prefix must end at an existing 0.1-second source timestamp")
        controls, reference = controls[:count], reference[:count + 1]
    if not controls:
        raise ValueError("At least one prefix step is required")
    sources = {label: native_plan(capture, label) for label in {row["source_label"] for row in controls}}
    continuation = (native_proposal(capture, args.proposal, args.candidate) if args.proposal else
                    native_plan(capture, args.source, args.candidate))
    for index, row in enumerate(controls):
        record, path, speeds = sources[row["source_label"]]
        if (not np.array_equal(path, row["world_trajectory"])
                or not np.array_equal(speeds, row["speeds_mps"])
                or abs(row["age_s"] - (index * .1 - record["context"]["time_s"])) > 1e-8
                or abs(row["time_s"] - index * .1) > 1e-8):
            raise ValueError(f"Original controller input identity failed at {index}")
    hashes = {}
    for name in ("module/trajectory_follower.py", "module/config.py"):
        current = (runner.ROOT / name).read_bytes()
        if current != (capture / "source_snapshot" / name).read_bytes():
            raise ValueError(f"Original controller/config changed: {name}")
        hashes[name] = hashlib.sha256(current).hexdigest()
    (out / "replay_native_tracking.py").write_bytes(Path(__file__).read_bytes())
    runner.write_json(out / "continuation-source.json", continuation[0])
    first = sources[controls[0]["source_label"]][0]
    route = json.loads((capture / "route-reference.json").read_text())["routes"][first["selection"]["route"]]
    bounds = first["context"]["vehicle_bounds"]
    box = carla.BoundingBox(carla.Location(**bounds["location"]), carla.Vector3D(**bounds["extent"]))
    box.rotation = carla.Rotation(**bounds["rotation"])
    tolerances = {"position_m": .01, "orientation_deg": .1, "speed_mps": .02, "control": 1e-4}
    summary = {"kind": "native-path tracking replay, not a fresh-inference rollout",
               "capture": str(capture), "continuation_source": args.source,
               "continuation_proposal": args.proposal,
               "continuation_candidate_override": args.candidate,
               "prefix_seconds": len(controls) * .1, "requested_continuation_s": args.duration,
               "tolerances": tolerances, "source_hashes": hashes,
               "native_identity_verified": True, "prefix_verified": False,
               "termination": "error", "states": [], "prefix_errors": [], "controls": []}
    random.seed(config["seed"])
    np.random.seed(config["seed"])
    client, world = runner.connect_world(SimpleNamespace(host=args.host, port=args.port), config)
    settings_before, weather_before = world.get_settings(), world.get_weather()
    lights = [(light, light.get_state(), light.is_frozen())
              for light in world.get_actors().filter("traffic.traffic_light")]
    adapter = Alpamayo15Adapter()
    interface = CARLAInterface(camera_configs=adapter.source_camera_configs)
    interface.client, interface.world = client, world
    overhead = None

    def tick():
        frame = world.tick()
        wait_for_frame(world, frame)
        interface.frame_id = frame
        return frame

    interface.tick = tick

    def save_state(elapsed):
        state = interface.get_ego_state()
        vertices = box.get_world_vertices(pose_transform(state))
        distance, _, _, width, _ = project(np.array([[v.x, v.y] for v in vertices]), route)
        summary["states"].append({"time_s": elapsed, **state,
                                  "corner_clearance_m": float((width - distance).min())})
        return state

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = .1
        settings.substepping = True
        settings.max_substep_delta_time = .01
        settings.max_substeps = 10
        world.apply_settings(settings)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        for light, _, _ in lights:
            light.set_state(carla.TrafficLightState.Green)
            light.freeze(True)
        overhead = runner.OverheadRecorder(world, config["junction_entry"], out, config.get("overhead"),
                                          label=f"Native replay {args.proposal or args.source}; no new inference")
        frames, _ = initialize_ego(interface, config["junction_entry"], runner.rollout_specs(config)[0])
        state = save_state(0.)
        overhead.add_frame(interface.frame_id, pose_transform(state), 0., state["speed"],
                           front_rgb=frames[-1][adapter.viz_camera_slot])
        follower = TrajectoryFollower(interface.ego_vehicle)
        physics = interface.ego_vehicle.get_physics_control()
        summary["vehicle_physics"] = {
            "mass": physics.mass,
            "steering_curve": [[p.x, p.y] for p in physics.steering_curve],
            "wheels": [{"max_steer_angle": w.max_steer_angle,
                        "position": [w.position.x, w.position.y, w.position.z]}
                       for w in physics.wheels],
            "follower_max_steer_rad": follower.max_steer_rad}
        for index in range(len(controls) + round(args.duration / .1)):
            elapsed = index * .1
            prefix = index < len(controls)
            if index <= len(controls):
                errors = state_error(state, reference[index])
                summary["prefix_errors"].append({"time_s": elapsed, **errors})
                if any(not np.isfinite(value) or value > tolerances[key] for key, value in errors.items()):
                    raise RuntimeError(f"Replay state mismatch at {elapsed}: {errors}")
            if index == len(controls):
                summary["prefix_verified"] = True
                print("Prefix verified; continuing the unchanged native plan", flush=True)
            record, path, speeds = sources[controls[index]["source_label"]] if prefix else continuation
            label = record["label"]
            age = elapsed - record["context"]["time_s"]
            if age < -1e-8 or age + 1.5 > len(path) * .1:
                raise ValueError("Continuation source does not cover the original controller preview")
            control = follower.compute_control(pose_transform(state), path, state["speed"], speeds, elapsed_sec=age)
            values = dict(zip(("steer", "throttle", "brake"), control[:3]))
            if prefix:
                error = max(abs(values[k] - controls[index]["control"][k]) for k in values)
                summary["prefix_errors"][-1]["control"] = error
                if error > tolerances["control"]:
                    raise RuntimeError(f"Replay control mismatch at {elapsed}: {error}")
            response = client.apply_batch_sync([
                carla.command.ApplyVehicleControl(interface.ego_vehicle.id, carla.VehicleControl(**values))], False)[0]
            if response.error:
                raise RuntimeError(response.error)
            tick()
            images = interface.get_camera_images(timeout=30)
            state = save_state((index + 1) * .1)
            applied = interface.ego_vehicle.get_control()
            mismatch = max(abs(getattr(applied, k) - values[k]) for k in values)
            summary["controls"].append({"time_s": elapsed, "source_label": label, "age_s": age,
                                        "control": values, "controller_debug": control[3],
                                        "applied_control_max_error": mismatch})
            if mismatch > 1e-5:
                raise RuntimeError("Simulator did not apply the requested control")
            overhead.add_frame(interface.frame_id, pose_transform(state), (index + 1) * .1, state["speed"],
                               prediction=path, prediction_time=record["context"]["time_s"],
                               front_rgb=images[adapter.viz_camera_slot])
            if not prefix:
                print(json.dumps(summary["states"][-1]), flush=True)
                if summary["states"][-1]["corner_clearance_m"] < 0:
                    summary["termination"] = "actual_body_departure"
                    break
            if interface.get_collision_count():
                summary["termination"] = "collision"
                break
        else:
            summary["termination"] = "diagnostic_duration_reached"
    except BaseException as exc:
        summary["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        runner.write_json(out / "summary.json", summary)
        try:
            if overhead is not None:
                overhead.close()
        finally:
            runner.destroy_ego(interface)
            for light, state, frozen in lights:
                light.set_state(state)
                light.freeze(frozen)
            world.set_weather(weather_before)
            world.apply_settings(settings_before)
    print(json.dumps({k: v for k, v in summary.items() if k not in ("states", "controls", "prefix_errors")}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--source")
    source.add_argument("--proposal", help="Test a natively valid sample from a rejected original proposal pool")
    parser.add_argument("--candidate", type=int,
                        help="Test a different unmodified sample from the source's original proposal pool")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=1.)
    parser.add_argument("--prefix-seconds", type=float,
                        help="Replay only this much of the recorded prefix, then hold the native source")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    args = parser.parse_args()
    if not np.isfinite(args.duration) or not 0 < args.duration <= 2:
        parser.error("Diagnostic duration must be in (0, 2] seconds")
    run(args)


if __name__ == "__main__":
    main()
