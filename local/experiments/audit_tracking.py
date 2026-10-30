"""Independently reconstruct tracking checks and original controller history."""
import argparse
import json
import math
from pathlib import Path

import carla
import numpy as np

from local.experiments.audit_turn import project
from local.experiments.tracking_forecast import forecast, prefix_follower, read_capture
from local.experiments.validated_selection import pose_transform


def audit(capture, physics_reference):
    physics = json.loads((physics_reference / "summary.json").read_text())["vehicle_physics"]
    states, controls = read_capture(capture)
    follower = prefix_follower(states, controls, 0, physics["follower_max_steer_rad"])
    routes = json.loads((capture / "route-reference.json").read_text())["routes"]
    checked, mismatches = [], []
    for index, row in enumerate(controls):
        record = json.loads((capture / f"frame_{index:03d}.json").read_text())
        if "retention" in record:
            retention = record["retention"]
            metric = retention["metrics"][retention["chosen_cache_index"]]
        else:
            selection = record["selection"]
            attempt = next(a for a in selection["attempts"] if a["proposal"] == selection["proposal"])
            metric = attempt["metrics"][selection["chosen"]]
        logged = metric["tracking_forecast"]
        context = record["context"]
        world, speeds = np.asarray(row["world_trajectory"]), np.asarray(row["speeds_mps"])
        frames = forecast(follower, {**context["world_pose"], "speed": context["measured_speed_mps"]},
                          world, speeds, row["age_s"], routes[record["selection"]["route"]],
                          context["vehicle_bounds"], logged["horizon_s"], logged["effective_steering_angle_gain"])
        clearance = min(r["corner_clearance_m"] for r in frames)
        delta = abs(clearance - logged["min_corner_clearance_m"])
        bounds = context["vehicle_bounds"]
        box = carla.BoundingBox(carla.Location(**bounds["location"]), carla.Vector3D(**bounds["extent"]))
        box.rotation = carla.Rotation(**bounds["rotation"])
        vertices = box.get_world_vertices(pose_transform(states[index + 1]))
        distance, _, _, width, _ = project(np.array([[v.x, v.y] for v in vertices]),
                                          routes[record["selection"]["route"]])
        actual_clearance = float((width - distance).min())
        # Only this real recorded command advances the reconstructed follower.
        u = follower.compute_control(pose_transform(states[index]), world, states[index]["speed"],
                                     speeds, elapsed_sec=row["age_s"])
        control_error = max(abs(value - row["control"][key])
                            for key, value in zip(("steer", "throttle", "brake"), u[:3]))
        checks = {"forecast_matches": delta < 1e-8,
                  "forecast_passed": logged["valid"] and clearance >= logged["required_clearance_m"],
                  "actual_controller_history_matches": control_error < 1e-8,
                  "real_age": abs(logged["original_plan_age_s"] - row["age_s"]) < 1e-8,
                  "measured_speed": logged["assumed_constant_speed_mps"] == context["measured_speed_mps"]}
        if "preferred_clearance_m" in logged:
            preferred = logged["preferred_clearance_m"]
            checks["reserve_preference_matches"] = metric["preferred_reserve_valid"] == (
                metric["max_corner_overhang_m"] <= -preferred and clearance >= preferred)
        item = {"time_s": row["time_s"], "source": row["source_label"], "age_s": row["age_s"],
                "preferred_reserve_met": metric.get("preferred_reserve_valid"),
                "recomputed_clearance_m": clearance, "forecast_error_m": delta,
                "controller_error": control_error, "checks": checks,
                "first_step_forecast_clearance_m": frames[0]["corner_clearance_m"],
                "actual_next_clearance_m": actual_clearance,
                "first_step_clearance_overestimate_m": frames[0]["corner_clearance_m"] - actual_clearance,
                "first_step_position_error_m": math.hypot(frames[0]["x"] - states[index + 1]["x"],
                                                          frames[0]["y"] - states[index + 1]["y"])}
        checked.append(item)
        if not all(checks.values()):
            mismatches.append(item)
    attempts = {}
    paths = [*capture.glob("accepted_frame_*.json"), *(capture / "replan-rejections").glob("*.json")]
    if (capture / "validation-failure.json").exists():
        paths.append(capture / "validation-failure.json")
    for path in paths:
        r = json.loads(path.read_text())
        for attempt in r.get("selection", r).get("attempts", []):
            attempts[attempt["proposal"]] = attempt
    metrics = [m["tracking_forecast"] for a in attempts.values() for m in a["metrics"] if "tracking_forecast" in m]
    return {"capture": str(capture), "physics_reference": str(physics_reference),
            "executed_steps_checked": len(checked), "all_tracking_and_controller_checks_match": bool(checked) and not mismatches,
            "mismatches": mismatches, "fresh_proposal_pools": len(attempts),
            "fresh_geometry_admitted_candidates_screened": len(metrics),
            "fresh_candidates_rejected_by_tracking": sum(not m["valid"] for m in metrics),
            "executed_steps_below_preferred_reserve": sum(r["preferred_reserve_met"] is False for r in checked),
            "max_forecast_error_m": max((r["forecast_error_m"] for r in checked), default=None),
            "max_controller_error": max((r["controller_error"] for r in checked), default=None),
            "max_observed_first_step_clearance_overestimate_m": max(
                (r["first_step_clearance_overestimate_m"] for r in checked), default=None),
            "max_observed_first_step_position_error_m": max(
                (r["first_step_position_error_m"] for r in checked), default=None),
            "note": "Verifies the approximate screen was applied as logged, not its physical accuracy or safety.",
            "frames": checked}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--physics-reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.capture, args.physics_reference)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "frames"}, indent=2))


if __name__ == "__main__":
    main()
