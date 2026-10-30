"""Experimental lane checks for unmodified model trajectories; no obstacle model."""
import argparse
import json
from pathlib import Path

import carla
import numpy as np

from local.experiments.audit_turn import project
from module import config
from module.trajectory_follower import planned_speeds, trajectory_to_world


def pose_transform(pose):
    return carla.Transform(carla.Location(**{k: pose[k] for k in ("x", "y", "z")}),
                           carla.Rotation(**{k: pose[k] for k in ("pitch", "yaw", "roll")}))


def predicted_corners(transform, candidate, yaws, bounds):
    """Project the real vehicle box at native predicted rear-axle poses.

    Model yaw is left-positive; CARLA local yaw is right-positive. The box is
    actor-relative, so its offset from the rear axle must also be rotated.
    These points are used only for validation, never returned as a trajectory.
    """
    box = carla.BoundingBox(carla.Location(**bounds["location"]), carla.Vector3D(**bounds["extent"]))
    box.rotation = carla.Rotation(**bounds["rotation"])
    offsets = np.array([[p.x, p.y, p.z] for p in box.get_local_vertices()])
    offsets[:, 0] -= config.EGO_REAR_AXLE_X
    rotations = np.zeros((len(yaws), 3, 3))
    cosine, sine = np.cos(yaws), np.sin(yaws)
    rotations[:, 0, 0] = rotations[:, 1, 1] = cosine
    rotations[:, 0, 1], rotations[:, 1, 0] = sine, -sine
    rotations[:, 2, 2] = 1
    world_rotations = np.asarray(transform.get_matrix())[:3, :3] @ rotations
    rear = trajectory_to_world(transform, candidate)
    return rear[:, None, :] + np.einsum("tij,vj->tvi", world_rotations, offsets)


def assess(candidates, context, route, markers, velocities, horizon_s=3., *,
           yaws=None, check_body=False):
    """Check native plans; optional box-corner checks are not an obstacle model."""
    if check_body and (yaws is None or "vehicle_bounds" not in context):
        raise ValueError("Body validation requires native predicted yaw and recorded vehicle bounds")
    tf = pose_transform(context["world_pose"])
    rear = tf.transform(carla.Location(x=config.EGO_REAR_AXLE_X))
    origin = np.array([rear.x, rear.y])
    initial_distance, initial_arc, _, _, _ = project(origin[None], route)
    base_limit = min(round(horizon_s / .1), 64)
    result = []
    for index, candidate in enumerate(candidates):
        item = {"candidate": index, "marker_present": bool(markers[index]),
                "finite": bool(np.isfinite(candidate).all())}
        if not item["finite"]:
            item.update(valid=False, reasons=["nonfinite"])
            result.append(item)
            continue
        world = trajectory_to_world(tf, candidate)
        lookahead = float(np.clip(config.PID_LOOKAHEAD_MIN_M + config.PID_LOOKAHEAD_SPEED_GAIN *
                                  context["measured_speed_mps"], config.PID_LOOKAHEAD_MIN_M,
                                  config.PID_LOOKAHEAD_MAX_M))
        distances_from_rear = np.linalg.norm(world[:, :2] - origin, axis=1)
        start = int(distances_from_rear.argmin())
        ahead = np.flatnonzero(distances_from_rear[start:] >= lookahead)
        target = start + (int(ahead[0]) if len(ahead) else len(world[start:]) - 1)
        speed_window_end = round((config.PID_SPEED_LEAD_SEC + config.PID_SPEED_PREVIEW_SEC) / .1)
        # Cover every waypoint used by the original controller, even at low
        # speed when its 4 m minimum steering lookahead lies beyond 1.5 seconds.
        limit = min(len(world), max(base_limit, target + 1, speed_window_end))
        distance, arc, heading, half_width, _ = project(world[:limit, :2], route)
        delta = np.diff(np.vstack([origin, world[:limit, :2]]), axis=0)
        moving = np.linalg.norm(delta, axis=1) > .05
        motion_heading = np.degrees(np.arctan2(delta[:, 1], delta[:, 0]))
        error = np.abs((motion_heading - heading + 180) % 360 - 180)
        signed = np.asarray(velocities[index])[:limit]
        reverse = bool(np.any(signed < -.1))
        exceeds = bool(np.any(distance > half_width))
        speed = planned_speeds(candidate)
        reasons = []
        if not item["marker_present"]:
            reasons.append("missing_marker")
        if reverse:
            reasons.append("reverse_motion")
        if exceeds:
            reasons.append("outside_lane_center_corridor")
        if np.any(error[moving] > 45):
            reasons.append("misaligned_with_route")
        if check_body:
            native_yaw = np.asarray(yaws[index])
            if native_yaw.shape != (len(candidate),) or not np.isfinite(native_yaw).all():
                reasons.append("invalid_native_orientation")
            else:
                corners = predicted_corners(tf, candidate[:limit], native_yaw[:limit], context["vehicle_bounds"])
                corner_distance, _, _, corner_width, _ = project(corners[..., :2].reshape(-1, 2), route)
                overhang = corner_distance - corner_width
                item.update(max_corner_lane_distance_m=float(corner_distance.max()),
                            max_corner_overhang_m=float(overhang.max()))
                if np.any(overhang > 0):
                    reasons.append("vehicle_corners_outside_lane_corridor")
        item.update(valid=not reasons, reasons=reasons,
                    current_rear_lane_distance_m=float(initial_distance[0]),
                    max_lane_distance_m=float(distance.max()),
                    max_lane_distance_first_1p5s_m=float(distance[:15].max()),
                    rms_lane_distance_m=float(np.sqrt(np.mean(distance**2))),
                    max_heading_error_deg=float(error[moving].max()) if np.any(moving) else 0.,
                    route_progress_m=float(arc[-1] - initial_arc[0]),
                    target_speed_mps=float(speed[10:15].mean()),
                    min_signed_velocity_mps=float(signed.min()),
                    max_signed_velocity_mps=float(signed.max()),
                    controller_start_index=start, controller_target_index=target,
                    controller_lookahead_m=lookahead,
                    horizon_s=limit * .1)
        result.append(item)
    return result


def choose(metrics, candidates, signal, current_speed, desired_speed=5.):
    """Rank admissible samples; signals come from the image-based model question."""
    target = desired_speed if signal == "green" else max(0., min(current_speed, desired_speed))
    for item, candidate in zip(metrics, candidates):
        if not item["valid"]:
            item["selection_cost"] = None
            continue
        speed = planned_speeds(candidate)
        if signal in ("red", "yellow"):
            cost = float((speed.sum() * .1)**2 + item["target_speed_mps"]**2)
        else:
            cost = (4 * item["rms_lane_distance_m"]**2 +
                    .25 * (item["target_speed_mps"] - target)**2 +
                    .05 * (item["route_progress_m"] - target * item["horizon_s"])**2)
        item["selection_cost"] = cost
    accepted = [item for item in metrics if item["valid"]]
    return min(accepted, key=lambda item: item["selection_cost"])["candidate"] if accepted else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path)
    parser.add_argument("--routes", type=Path, required=True)
    parser.add_argument("--route", choices=("left", "straight"), required=True)
    parser.add_argument("--horizon", type=float, default=3.)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    route = json.loads(args.routes.read_text())["routes"][args.route]
    results = []
    for path in sorted(args.replay.glob("frame_*.json")):
        record = json.loads(path.read_text())
        candidates = np.asarray(record["candidate_trajectories"] or [record["trajectory_ego"]])
        metrics = assess(candidates, record["context"], route, record["candidate_markers"],
                         record["candidate_velocities"], args.horizon)
        results.append({"label": record["label"], "route": args.route, "metrics": metrics})
        print(record["label"], "valid", [m["candidate"] for m in metrics if m["valid"]],
              "best distance", round(min(m.get("max_lane_distance_m", float("inf")) for m in metrics), 3),
              "targets", [round(m.get("target_speed_mps", 0), 2) for m in metrics])
    args.output.write_text(json.dumps(results, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
