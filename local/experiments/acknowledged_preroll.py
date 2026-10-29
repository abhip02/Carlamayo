"""Experimental pre-roll with acknowledged commands and exact-frame poses."""
from collections import deque
import math

import carla

from local import run_intersection as runner
from local.experiments.frame_sync import wait_for_frame


def initialize_ego(interface, entry, spec):
    world, client = interface.world, interface.client
    pose = runner.requested_pose(entry, spec)
    blueprint = world.get_blueprint_library().find("vehicle.lincoln.mkz_2020")
    blueprint.set_attribute("role_name", "carlamayo_intersection")
    vehicle = world.spawn_actor(blueprint, runner.make_transform(pose))
    interface.ego_vehicle = vehicle

    def send(*commands):
        for response in client.apply_batch_sync(list(commands), False):
            if response.error:
                raise RuntimeError(response.error)

    def tick_state():
        frame = interface.tick()
        actor = wait_for_frame(world, frame).find(vehicle.id)
        transform, velocity = actor.get_transform(), actor.get_velocity()
        return {"x": transform.location.x, "y": transform.location.y, "z": transform.location.z,
                "roll": transform.rotation.roll, "pitch": transform.rotation.pitch,
                "yaw": transform.rotation.yaw, "speed": velocity.length()}

    send(carla.command.ApplyVehicleControl(vehicle.id, carla.VehicleControl(brake=1)))
    for _ in range(15):
        state = tick_state()
    pose["z"] = state["z"]
    interface.setup_cameras()
    send(carla.command.SetSimulatePhysics(vehicle.id, False))
    frames = deque(maxlen=runner.defaults.NUM_FRAMES)
    yaw = math.radians(pose["yaw"])
    vx, vy = spec["speed_mps"] * math.cos(yaw), spec["speed_mps"] * math.sin(yaw)
    for index in range(runner.defaults.NUM_HISTORY):
        if index < runner.defaults.NUM_HISTORY - 1:
            ago = (runner.defaults.NUM_HISTORY - 1 - index) * runner.DT
            transform = runner.make_transform({**pose, "x": pose["x"] - vx * ago, "y": pose["y"] - vy * ago})
            send(carla.command.ApplyTransform(vehicle.id, transform))
        else:
            send(carla.command.SetSimulatePhysics(vehicle.id, True),
                 carla.command.ApplyVehicleControl(vehicle.id, carla.VehicleControl()),
                 carla.command.ApplyTargetVelocity(vehicle.id, carla.Vector3D(x=vx, y=vy)))
        interface.update_history(tick_state())
        frames.append(interface.get_camera_images(timeout=30))
    interface.setup_collision_sensor()
    return frames, pose
