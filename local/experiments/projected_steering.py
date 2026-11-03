"""Opt-in upstream-style nearest-driving-lane steering target projection.

Uses the map lookup from CarlaMayo 578170c. The current target selection,
pure-pursuit steering and native longitudinal controller stay in place.
This deliberately does not reproduce the historical 10 km/h speed floor.
"""
import math

import carla
import numpy as np

from module import config as cfg
from module.trajectory_follower import TrajectoryFollower, pure_pursuit_steer


class ProjectedSteeringFollower(TrajectoryFollower):
    def __init__(self, vehicle, mode='map'):
        if mode not in ('native', 'map'):
            raise ValueError('Steering mode must be native or map')
        super().__init__(vehicle)
        self.steering_mode = mode
        self.world_map = vehicle.get_world().get_map() if mode == 'map' else None

    def compute_control(self, tf, world, speed, speeds, elapsed_sec=0.):
        previous = self._prev_steer
        steer, throttle, brake, debug = super().compute_control(
            tf, world, speed, speeds, elapsed_sec)
        raw = np.asarray(world)[debug['target_idx']].copy()
        debug.update(steering_mode=self.steering_mode, native_target_xyz=raw.tolist(),
                     native_steer=steer, projection_distance_m=0., projected_target_xyz=None)
        if self.steering_mode == 'map':
            target = self.world_map.get_waypoint(
                carla.Location(x=float(raw[0]), y=float(raw[1]), z=float(raw[2])),
                project_to_road=True, lane_type=carla.LaneType.Driving)
            if target is None:
                raise RuntimeError('No driving-lane waypoint for projected steering target')
            point = target.transform.location
            projected = [point.x, point.y, point.z]
            if not np.isfinite(projected).all():
                raise RuntimeError('Nonfinite map-projected steering target')
            axle = tf.transform(carla.Location(x=cfg.EGO_REAR_AXLE_X))
            steer = pure_pursuit_steer([axle.x, axle.y], math.radians(tf.rotation.yaw),
                                      projected[:2], cfg.EGO_WHEELBASE_M, self.max_steer_rad)
            delta = cfg.FOLLOW_STEER_RATE_PER_SEC * cfg.CONTROL_DT
            steer = float(np.clip(steer, previous - delta, previous + delta))
            self._prev_steer = steer
            debug.update(mode='map_target_pure_pursuit_native_speed',
                target_wp_xy=projected[:2], projected_target_xyz=projected,
                projection_distance_m=float(np.linalg.norm(raw[:2] - projected[:2])),
                projected_lane=dict(road_id=target.road_id, lane_id=target.lane_id,
                    s=target.s, is_junction=target.is_junction,
                    yaw=target.transform.rotation.yaw))
        return steer, throttle, brake, debug
