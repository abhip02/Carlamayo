"""Select an unmodified model sample using the requested route and prior plan."""
import math

import carla
import numpy as np

from module import config
from module.trajectory_follower import trajectory_to_world


def select_candidate(candidates, context, previous_world=None, *, prefer_stop=False):
    pose, route = context["world_pose"], context["route_entry"]
    transform = carla.Transform(carla.Location(x=pose["x"], y=pose["y"], z=pose["z"]),
                                carla.Rotation(yaw=pose["yaw"], pitch=pose["pitch"], roll=pose["roll"]))
    world = np.stack([trajectory_to_world(transform, candidate) for candidate in candidates])
    angle = math.radians(route["yaw"])
    forward, left = np.array([math.cos(angle), math.sin(angle)]), np.array([math.sin(angle), -math.cos(angle)])
    lateral = (world[..., :2] - [route["x"], route["y"]]) @ left
    # Penalize leaving the requested lane, while allowing small within-lane changes.
    route_cost = np.mean(np.maximum(np.abs(lateral) - 1.25, 0)**2, axis=1)
    count = min(30, world.shape[1])
    if previous_world is None:
        rear = transform.transform(carla.Location(x=config.EGO_REAR_AXLE_X))
        speed = max(0, context["measured_signed_speed_mps"])
        reference = [rear.x, rear.y] + np.arange(1, count + 1)[:, None] * .1 * speed * forward
    else:
        # The previous prediction started one 0.1 s planning tick earlier.
        reference = previous_world[1:count + 1, :2]
    continuity_cost = np.mean(np.sum((world[:, :count, :2] - reference)**2, axis=-1), axis=1)
    costs = route_cost + .25 * continuity_cost
    stop_cost = np.zeros(len(candidates))
    if prefer_stop:
        # A short-horizon stop followed by a long tail can trigger the original
        # follower's launch floor. Prefer samples that remain stopped throughout
        # the prediction when the visual checker reports red/yellow.
        origins = np.zeros((len(candidates), 1, 2))
        travel = np.linalg.norm(np.diff(np.concatenate((origins, candidates[..., :2]), axis=1), axis=1), axis=-1).sum(axis=1)
        stop_cost = travel**2
        costs = costs + stop_cost
    chosen = int(np.argmin(costs))
    return chosen, world[chosen], {
        "chosen": chosen, "costs": costs.tolist(), "route_costs": route_cost.tolist(),
        "continuity_costs": continuity_cost.tolist(),
        "stop_costs": stop_cost.tolist(),
    }
