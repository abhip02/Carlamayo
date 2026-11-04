"""Opt-in upstream-style nearest-driving-lane steering target projection.

Uses the map lookup from CarlaMayo 578170c. Optional explicit experiments add a
target-speed ceiling and steering calibration to the current follower. Neither
rewrites the raw trajectory nor adds the historical 10 km/h minimum speed.
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
    def __init__(self, vehicle, mode='map', route=None, speed_cap_mps=None, steering_gain=1.,
                 hold_visual_stop=False):
        if mode not in ('native', 'map', 'route'):
            raise ValueError('Steering mode must be native, map or route')
        super().__init__(vehicle)
        if speed_cap_mps is not None and (not math.isfinite(speed_cap_mps) or speed_cap_mps <= 0):
            raise ValueError('Speed ceiling must be positive and finite')
        if not math.isfinite(steering_gain) or not 0.5 <= steering_gain <= 2.:
            raise ValueError('Steering gain must be between 0.5 and 2')
        self.speed_cap_mps = speed_cap_mps
        self.steering_gain = steering_gain
        self.hold_visual_stop = hold_visual_stop
        self.visual_signal, self.visual_check_index = 'unknown', -1
        self._visual_stop_held = False
        self._hold_check_index = -1
        self._last_planned_stop = False
        self.steering_mode = mode
        self.world_map = vehicle.get_world().get_map() if mode == 'map' else None
        if mode == 'route':
            if route is None:
                raise ValueError('Route projection requires an explicit commanded route')
            self.world_map = CommandedRouteMap(route)

    def _target_speed(self, speeds_mps, speed_mps, elapsed_sec):
        if self._visual_stop_held:
            self._launching = False
            return 0., False
        target, launch = super()._target_speed(speeds_mps, speed_mps, elapsed_sec)
        return (min(target, self.speed_cap_mps) if self.speed_cap_mps is not None else target), launch

    def observe_visual_signal(self, signal, check_index):
        """Receive the model's image answer, never the simulator's signal state."""
        self.visual_signal, self.visual_check_index = signal, check_index

    def compute_control(self, tf, world, speed, speeds, elapsed_sec=0.):
        activated = released = False
        if self.hold_visual_stop:
            if self._visual_stop_held:
                released = (self.visual_signal == 'green'
                            and self.visual_check_index > self._hold_check_index)
                if released:
                    self._visual_stop_held = False
            elif self.visual_signal in ('red', 'yellow') and speed <= .2 and self._last_planned_stop:
                # Preserve a native planned stop once achieved. Without this latch,
                # the upstream launch floor can restart a stopped car on red.
                self._visual_stop_held = activated = True
                self._hold_check_index = self.visual_check_index
        previous = self._prev_steer
        steer, throttle, brake, debug = super().compute_control(
            tf, world, speed, speeds, elapsed_sec)
        raw = np.asarray(world)[debug['target_idx']].copy()
        debug.update(steering_mode=self.steering_mode, native_target_xyz=raw.tolist(),
                     controller_speed_cap_mps=self.speed_cap_mps, steering_gain=self.steering_gain,
                     native_steer=steer, projection_distance_m=0., projected_target_xyz=None)
        if self.hold_visual_stop:
            debug['visual_stop_hold'] = dict(active=self._visual_stop_held, activated=activated,
                released=released, signal=self.visual_signal, check_index=self.visual_check_index,
                activation_check_index=self._hold_check_index)
        self._last_planned_stop = debug['planned_stop']
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
            steer = float(np.clip(steer * self.steering_gain, -1., 1.))
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
