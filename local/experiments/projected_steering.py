"""Opt-in upstream-style nearest-driving-lane steering target projection.

Uses the map lookup from CarlaMayo 578170c. The current target selection,
pure-pursuit steering and native longitudinal controller stay in place.
This deliberately does not reproduce the historical 10 km/h speed floor.
"""
import math
from types import SimpleNamespace

import carla
import numpy as np

from module import config as cfg
from module.trajectory_follower import TrajectoryFollower, pure_pursuit_steer


class CommandedRouteMap:
    """Project only onto the declared maneuver's map polyline, not adjacent lanes.

    This is a separate extension to the historical unrestricted map lookup.
    No forward-point floor, speed floor or synthetic trajectory is introduced.
    """
    def __init__(self, route):
        self.points = route['points']
        self.xyz = np.array([[p['x'], p['y'], p['z']] for p in self.points])

    def get_waypoint(self, location, **kwargs):
        start, delta = self.xyz[:-1], np.diff(self.xyz, axis=0)
        length2 = (delta[:, :2]**2).sum(axis=1)
        valid = length2 > 1e-12
        fractions = np.zeros(len(delta))
        fractions[valid] = np.clip(((np.array([location.x, location.y]) - start[valid,:2]) *
                                    delta[valid,:2]).sum(axis=1) / length2[valid], 0., 1.)
        points = start + fractions[:,None] * delta
        distance2 = ((points[:,:2] - [location.x, location.y])**2).sum(axis=1)
        distance2[~valid] = np.inf
        if not np.isfinite(distance2).any():
            raise ValueError('Commanded route contains no usable segments')
        i = int(distance2.argmin())
        point = points[i]
        metadata = self.points[i]
        yaw = math.degrees(math.atan2(delta[i,1], delta[i,0]))
        return SimpleNamespace(transform=carla.Transform(
            carla.Location(x=float(point[0]),y=float(point[1]),z=float(point[2])),
            carla.Rotation(yaw=yaw)), road_id=metadata['road_id'],lane_id=metadata['lane_id'],
            s=metadata['s'],is_junction=metadata['is_junction'])


class ProjectedSteeringFollower(TrajectoryFollower):
    def __init__(self, vehicle, mode='map', route=None):
        if mode not in ('native', 'map', 'route'):
            raise ValueError('Steering mode must be native, map or route')
        super().__init__(vehicle)
        self.steering_mode = mode
        self.world_map = vehicle.get_world().get_map() if mode == 'map' else None
        if mode == 'route':
            if route is None:
                raise ValueError('Route projection requires an explicit commanded route')
            self.world_map = CommandedRouteMap(route)

    def compute_control(self, tf, world, speed, speeds, elapsed_sec=0.):
        previous = self._prev_steer
        steer, throttle, brake, debug = super().compute_control(
            tf, world, speed, speeds, elapsed_sec)
        raw = np.asarray(world)[debug['target_idx']].copy()
        debug.update(steering_mode=self.steering_mode, native_target_xyz=raw.tolist(),
                     native_steer=steer, projection_distance_m=0., projected_target_xyz=None)
        if self.steering_mode in ('map', 'route'):
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
            debug.update(mode=self.steering_mode+'_target_pure_pursuit_native_speed',
                target_wp_xy=projected[:2], projected_target_xyz=projected,
                projection_distance_m=float(np.linalg.norm(raw[:2] - projected[:2])),
                projected_lane=dict(road_id=target.road_id, lane_id=target.lane_id,
                    s=target.s, is_junction=target.is_junction,
                    yaw=target.transform.rotation.yaw))
        return steer, throttle, brake, debug
