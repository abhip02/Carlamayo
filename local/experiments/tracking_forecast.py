"""Offline diagnostic of a constant-speed bicycle tracking screen.

Never supplies an executed path. Reuses a copy of the unchanged follower and
tests its steering response to native model paths. Longitudinal dynamics, tire
slip, banking and roll are deliberately absent: validate against simulator
replays before considering this as a candidate-admission rule.
"""
import argparse
import copy
import csv
import json
import math
from pathlib import Path
from types import SimpleNamespace

import carla
import numpy as np

from local.experiments.audit_turn import project
from local.experiments.replay_native_tracking import native_plan, native_proposal
from local.experiments.validated_selection import pose_transform
from module import config
from module.trajectory_follower import TrajectoryFollower


def read_capture(capture):
    with (capture / "000_verifai/ego.csv").open() as stream:
        states = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    controls = [json.loads(s) for s in (capture / "controller-inputs.jsonl").read_text().splitlines()]
    return states, controls


def calibrate(captures, maximum_angle):
    """Empirical effective wheel-angle gain; fit only independent older captures."""
    ratios = []
    for capture in captures:
        states, controls = read_capture(capture)
        for a, b, row in zip(states, states[1:], controls):
            steer = row["control"]["steer"]
            if abs(steer) < .08 or min(a["speed"], b["speed"]) < 1:
                continue
            yaw = math.radians((b["yaw"] - a["yaw"] + 180) % 360 - 180)
            distance = (a["speed"] + b["speed"]) / 2 * .1
            effective = math.atan(yaw * config.EGO_WHEELBASE_M / distance)
            ratios.append(effective / (maximum_angle * steer))
    if not ratios:
        raise ValueError("No moving turns available for calibration")
    return {"sources": [str(p) for p in captures], "samples": len(ratios),
            "angle_gain_q10_median_q90": np.quantile(ratios, [.1, .5, .9]).tolist(),
            "note": "Empirical aggregate, not a calibrated bound or universal vehicle model."}


def prefix_follower(states, controls, count, maximum_angle):
    vehicle = SimpleNamespace(get_physics_control=lambda: SimpleNamespace(
        wheels=[SimpleNamespace(max_steer_angle=math.degrees(maximum_angle))]))
    follower = TrajectoryFollower(vehicle)
    for i, row in enumerate(controls[:count]):
        u = follower.compute_control(pose_transform(states[i]), np.asarray(row["world_trajectory"]),
                                     states[i]["speed"], np.asarray(row["speeds_mps"]), row["age_s"])
        error = max(abs(value - row["control"][key]) for key, value in zip(("steer", "throttle", "brake"), u[:3]))
        if error > 1e-8:
            raise ValueError(f"Could not reconstruct unchanged follower at prefix step {i}: {error}")
    return follower


def forecast(follower, state, world, speeds, age, route, bounds, horizon_s, gain):
    """Internal approximate poses only; no waypoint or speed array modifications."""
    follower = copy.deepcopy(follower)
    state = dict(state)
    box = carla.BoundingBox(carla.Location(**bounds["location"]), carla.Vector3D(**bounds["extent"]))
    box.rotation = carla.Rotation(**bounds["rotation"])
    initial = pose_transform(state).transform(carla.Location(x=config.EGO_REAR_AXLE_X))
    rear = np.array([initial.x, initial.y])
    yaw = math.radians(state["yaw"])
    result = []
    for step in range(round(horizon_s / .1)):
        # Call the unchanged base implementation explicitly. The live follower
        # may have experiment wrappers for logging/retention; forecasts must
        # neither execute those wrappers nor mutate their global state.
        u = TrajectoryFollower.compute_control(
            follower, pose_transform(state), world, state["speed"], speeds, age + step * .1)
        curvature = math.tan(u[0] * follower.max_steer_rad * gain) / config.EGO_WHEELBASE_M
        # Integrate at 0.01 s. Speed is a scenario assumption, not an actuator command.
        for _ in range(10):
            rotation = state["speed"] * curvature * .01
            rear += state["speed"] * .01 * np.array([math.cos(yaw + rotation / 2), math.sin(yaw + rotation / 2)])
            yaw += rotation
        actor = rear - config.EGO_REAR_AXLE_X * np.array([math.cos(yaw), math.sin(yaw)])
        state.update(x=float(actor[0]), y=float(actor[1]), yaw=math.degrees(yaw))
        corners = box.get_world_vertices(pose_transform(state))
        distance, _, _, width, _ = project(np.array([[v.x, v.y] for v in corners]), route)
        result.append({"dt_s": (step + 1) * .1, "x": state["x"], "y": state["y"], "yaw": state["yaw"],
                       "corner_clearance_m": float((width - distance).min()), "steer": u[0]})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, nargs="+", required=True)
    parser.add_argument("--replays", type=Path, nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    replay_data = [json.loads((p / "summary.json").read_text()) for p in args.replays]
    physical = next(r["vehicle_physics"] for r in replay_data if "vehicle_physics" in r)
    maximum_angle = physical["follower_max_steer_rad"]
    calibration = calibrate(args.calibration, maximum_angle)
    gains = {"ideal_geometry": 1., "empirical_median": calibration["angle_gain_q10_median_q90"][1]}
    result = {"calibration": calibration, "assumption": "Constant measured initial speed; copied original follower; flat bicycle geometry.",
              "live_admission_enabled": False, "replays": []}
    for path, replay in zip(args.replays, replay_data):
        capture = Path(replay["capture"])
        if capture.resolve() in [p.resolve() for p in args.calibration]:
            raise ValueError("Calibration must exclude the evaluated capture")
        if not replay["prefix_verified"]:
            raise ValueError("Simulator replay prefix did not verify")
        states, controls = read_capture(capture)
        count = round(replay["prefix_seconds"] / .1)
        record, world, speeds = (
            native_proposal(capture, replay["continuation_proposal"], replay["continuation_candidate_override"])
            if replay.get("continuation_proposal") else native_plan(
                capture, replay["continuation_source"], replay.get("continuation_candidate_override")))
        follower = prefix_follower(states, controls, count, maximum_angle)
        route = json.loads((capture / "route-reference.json").read_text())["routes"][record["selection"]["route"]]
        actual = replay["states"][count + 1:]
        item = {"replay": str(path), "prefix_seconds": replay["prefix_seconds"],
                "actual_termination": replay["termination"], "actual_last_time_s": actual[-1]["time_s"],
                "actual_minimum_clearance_m": min(s["corner_clearance_m"] for s in actual), "forecasts": {}}
        for name, gain in gains.items():
            rows = forecast(follower, states[count], world, speeds,
                            replay["prefix_seconds"] - record["context"]["time_s"], route,
                            record["context"]["vehicle_bounds"], replay["requested_continuation_s"], gain)
            item["forecasts"][name] = {
                "gain": gain, "minimum_clearance_m": min(r["corner_clearance_m"] for r in rows),
                "first_departure_time_s": next((replay["prefix_seconds"] + r["dt_s"] for r in rows
                                                if r["corner_clearance_m"] < 0), None),
                "max_clearance_error_on_observed_prefix_m": max(abs(a["corner_clearance_m"] - b["corner_clearance_m"])
                                                                for a, b in zip(actual, rows)),
                "max_position_error_on_observed_prefix_m": max(math.hypot(a["x"] - b["x"], a["y"] - b["y"])
                                                               for a, b in zip(actual, rows)),
                "frames": rows}
        result["replays"].append(item)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"calibration": calibration, "replays": [
        {**r, "forecasts": {name: {k: v for k, v in f.items() if k != "frames"} for name, f in r["forecasts"].items()}}
        for r in result["replays"]]}, indent=2))


if __name__ == "__main__":
    main()
