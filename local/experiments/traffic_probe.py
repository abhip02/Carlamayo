"""Add declared lead traffic and explicit native-speed controller variants."""
import hashlib
import json
import math
from pathlib import Path
import time

import carla
import numpy as np

from local.experiments import acknowledged_preroll, projected_probe
from local.experiments import inference_probe as native
from local.experiments.audit_turn import project
from local.experiments.frame_sync import wait_for_frame


class NativeSpeedTrafficFollower(projected_probe.ProjectedSteeringFollower):
    """Keep the native timed speed preview, without the upstream launch boost.

    Remaining path length is not permission to accelerate toward traffic: it
    can include a planned wait, a short stopping path, or backward motion.
    """
    def _target_speed(self, speeds_mps, speed_mps, elapsed_sec):
        self._launching = False
        if self._visual_stop_held:
            return 0., False
        cfg = native.runner.defaults
        now = int(elapsed_sec / .1 + 1e-6)
        lead = getattr(self, 'traffic_speed_preview_lead_s', cfg.PID_SPEED_LEAD_SEC)
        step = now + int(lead / .1 + 1e-6)
        preview = max(1, round(cfg.PID_SPEED_PREVIEW_SEC / .1))
        window = speeds_mps[step:step + preview]
        target = float(np.mean(window)) if len(window) else 0.
        ceiling = cfg.PID_TARGET_SPEED_MAX_KMH / 3.6
        if self.speed_cap_mps is not None:
            ceiling = min(ceiling, self.speed_cap_mps)
        return min(target, ceiling), False

    def compute_control(self, *values, **kwargs):
        steer, throttle, brake, debug = super().compute_control(*values, **kwargs)
        debug['traffic_launch_boost_disabled'] = True
        debug['traffic_speed_preview_lead_s'] = getattr(
            self, 'traffic_speed_preview_lead_s', native.runner.defaults.PID_SPEED_LEAD_SEC)
        return steer, throttle, brake, debug


def point_ahead(route, origin, distance):
    """Place traffic along the same connected map route, without moving the ego."""
    _, arc, _, _, _ = project(np.array([[origin['x'], origin['y']]]), route)
    points = route['points']
    xyz = np.array([[p[k] for k in ('x', 'y', 'z')] for p in points])
    lengths = np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1)
    cumulative = np.r_[0., np.cumsum(lengths)]
    target = float(arc[0] + distance)
    if not 0 <= target < cumulative[-1]:
        raise ValueError('Traffic spawn exceeds the recorded connected route')
    index = int(np.searchsorted(cumulative, target, side='right') - 1)
    fraction = (target - cumulative[index]) / lengths[index]
    position = xyz[index] + fraction * (xyz[index + 1] - xyz[index])
    yaw = math.degrees(math.atan2(*(xyz[index + 1, :2] - xyz[index, :2])[::-1]))
    return carla.Transform(carla.Location(x=float(position[0]), y=float(position[1]),
                                         z=float(position[2]) + .4),
                           carla.Rotation(yaw=yaw)), index + 1


def traffic_manager_path(route_document, route_name, next_index, policy='dense'):
    """Keep unambiguous exit-road hints for a single-junction turn.

    CARLA0.9.16 maps path locations to its own waypoint graph and chooses
    junction branches by exit road id. Dense crossing points can snap to a
    different connector; an exit on the incoming road is also ambiguous.
    Therefore use outgoing-lane anchors only for a declared single junction
    whose incoming and outgoing road ids differ. All other paths stay dense.
    """
    if policy not in ('dense', 'distinct_road_exit_anchors'):
        raise ValueError('Unknown declared Traffic Manager path policy')
    route = route_document['routes'][route_name]
    points = route['points']
    if not isinstance(next_index, int) or not 0 <= next_index < len(points):
        raise ValueError('Traffic path starts outside the recorded route')
    dense = points[next_index:]
    metadata = dict(declared_policy=policy, applied='dense', reason='original_dense_path')
    if policy == 'dense':
        return dense, metadata
    if route_document.get('route_spec', {}).get('kind') != 'junction':
        metadata['reason'] = 'not_a_single_junction_route'
        return dense, metadata
    incoming = [p for p in points[:route['entry_index']] if not p['is_junction']]
    outgoing_indices = [i for i in range(route['exit_index'] + 1, len(points))
                        if not points[i]['is_junction']]
    if not incoming or not outgoing_indices:
        metadata['reason'] = 'no_unambiguous_incoming_or_outgoing_lane'
        return dense, metadata
    first = outgoing_indices[0]
    metadata.update(incoming_road_id=incoming[-1]['road_id'], outgoing_road_id=points[first]['road_id'])
    if incoming[-1]['road_id'] == points[first]['road_id']:
        metadata['reason'] = 'same_road_exit_requires_dense_path'
        return dense, metadata
    # Already beyond the junction: retain the original path, which needs no
    # branch disambiguation. Only future points may be supplied.
    if next_index > first:
        metadata['reason'] = 'already_on_outgoing_lane'
        return dense, metadata
    anchor = first
    distance = 0.
    while anchor + 1 < len(points) and not points[anchor + 1]['is_junction'] and distance < 8.:
        distance += math.hypot(points[anchor + 1]['x'] - points[anchor]['x'],
                               points[anchor + 1]['y'] - points[anchor]['y'])
        anchor += 1
    if anchor == len(points) - 1 or any(p['is_junction'] for p in points[first:]):
        metadata['reason'] = 'insufficient_single_outgoing_lane_suffix'
        return dense, metadata
    metadata.update(applied='outgoing_road_anchors', reason='distinct_incoming_and_outgoing_roads',
                    anchor_distance_after_junction_m=distance)
    return [points[anchor], points[-1]], metadata


def actor_record(actor, snapshot):
    state = snapshot.find(actor.id)
    if state is None:
        raise RuntimeError(f'Traffic actor {actor.id} missing from snapshot')
    transform, velocity = state.get_transform(), state.get_velocity()
    return dict(id=actor.id, blueprint=actor.type_id,
                x=transform.location.x, y=transform.location.y, z=transform.location.z,
                yaw=transform.rotation.yaw, speed_mps=velocity.length(),
                vertices_xy=[[p.x, p.y] for p in actor.bounding_box.get_world_vertices(transform)])


class TrafficScene:
    def __init__(self, spec, output, route_name='straight', signal_state='Green'):
        self.spec, self.output = spec, output
        self.route_name, self.signal_state = route_name, signal_state
        self.vehicle = self.interface = self.manager = None
        self.start_frame = self.start_seconds = None
        self.braked = False
        self.closed = False
        self.trace = (output / 'traffic-states.jsonl').open('w', buffering=1)

    def spawn(self, interface, entry, ego_spec):
        self.interface = interface
        world = interface.world
        if list(world.get_actors().filter('vehicle.*')) or list(world.get_actors().filter('walker.pedestrian.*')):
            raise RuntimeError('Unexpected actors before declared traffic initialization')
        if self.spec['kind'] == 'empty':
            return
        route_document = json.loads((self.output / 'generated-routes.json').read_text())
        route = route_document['routes'][self.route_name]
        origin = native.runner.requested_pose(entry, ego_spec)
        transform, next_index = point_ahead(route, origin, self.spec['initial_center_distance_m'])
        blueprint = world.get_blueprint_library().find('vehicle.lincoln.mkz_2020')
        blueprint.set_attribute('role_name', 'round19_lead')
        self.vehicle = world.spawn_actor(blueprint, transform)
        self.vehicle.apply_control(carla.VehicleControl(brake=1.))
        native.write_json(self.output / 'traffic-spawn.json', dict(
            specification=self.spec, actor_id=self.vehicle.id,
            transform=dict(x=transform.location.x, y=transform.location.y,
                           z=transform.location.z, yaw=transform.rotation.yaw)))
        path_points, path_metadata = traffic_manager_path(
            route_document, self.route_name, next_index, self.spec.get('path_policy', 'dense'))
        self.path = [carla.Location(x=p['x'], y=p['y'], z=p['z']) for p in path_points]
        native.write_json(self.output / 'traffic-path.json', {**path_metadata,
            'next_index': next_index, 'points': path_points,
            'used_by_autopilot': self.spec['kind'] in ('moving_lead', 'braking_lead')})
        if self.spec['kind'] in ('moving_lead', 'braking_lead'):
            self.manager = self.interface.client.get_trafficmanager(8013)
            self.manager.set_synchronous_mode(True)
            self.manager.set_random_device_seed(self.spec['traffic_seed'])
            # Traffic Manager initialization resets signals in this CARLA build.
            # Restore the declared scene before the ego history is captured.
            lights = list(world.get_actors().filter('traffic.traffic_light'))
            before = {str(light.id): str(light.get_state()) for light in lights}
            for light in lights:
                light.set_state(getattr(carla.TrafficLightState, self.signal_state))
                light.freeze(True)
            native.write_json(self.output / 'traffic-signal-initialization.json', dict(
                before=before, declared_state=self.signal_state,
                note='Scene initialization before ego pre-roll; no signal truth supplied to model decisions.'))

    def begin(self):
        snapshot = wait_for_frame(self.interface.world, self.interface.frame_id)
        self.start_frame = snapshot.frame
        self.start_seconds = snapshot.timestamp.elapsed_seconds
        if self.spec['kind'] in ('moving_lead', 'braking_lead'):
            self.vehicle.set_autopilot(True, 8013)
            self.manager.auto_lane_change(self.vehicle, False)
            self.manager.set_desired_speed(self.vehicle, self.spec['lead_speed_mps'] * 3.6)
            self.manager.set_path(self.vehicle, self.path)
        self.record(snapshot)

    def before_tick(self):
        if self.start_frame is None or self.spec['kind'] != 'braking_lead' or self.braked:
            return
        now = self.interface.world.get_snapshot().timestamp.elapsed_seconds - self.start_seconds
        if now + 1e-6 >= self.spec['brake_at_s']:
            self.vehicle.set_autopilot(False, 8013)
            self.vehicle.apply_control(carla.VehicleControl(brake=1.))
            self.braked = True
            native.write_json(self.output / 'lead-braking-event.json', dict(time_s=now, actor_id=self.vehicle.id))

    def verify_scope(self):
        actors = self.interface.world.get_actors()
        expected = {self.interface.ego_vehicle.id}
        if self.vehicle is not None:
            expected.add(self.vehicle.id)
        if {a.id for a in actors.filter('vehicle.*')} != expected:
            raise RuntimeError('Declared traffic actor set changed')
        if list(actors.filter('walker.pedestrian.*')):
            raise RuntimeError('Unexpected pedestrian in lead-vehicle test')
        if any(str(a.get_state()) != self.signal_state for a in actors.filter('traffic.traffic_light')):
            raise RuntimeError('Signal state changed in the declared traffic test')

    def record(self, snapshot):
        if self.start_frame is None:
            return
        self.verify_scope()
        record = dict(time_s=(snapshot.frame - self.start_frame) * .1, frame=snapshot.frame,
                      ego=actor_record(self.interface.ego_vehicle, snapshot),
                      traffic=[] if self.vehicle is None else [actor_record(self.vehicle, snapshot)],
                      lead_braking_commanded=self.braked,
                      scope_verified=True, signal_state=self.signal_state)
        self.trace.write(json.dumps(record) + '\n')

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.trace.close()
        # Stop the owned manager while the world is still synchronous and the
        # ego is alive. Otherwise its background stages can access actors that
        # rollout cleanup has already destroyed.
        if self.manager is not None:
            if self.vehicle is not None and self.vehicle.is_alive:
                self.vehicle.set_autopilot(False, 8013)
            self.manager.shut_down()
        if self.vehicle is not None and self.vehicle.is_alive:
            if not self.vehicle.destroy():
                raise RuntimeError('Owned traffic vehicle could not be destroyed')
        native.write_json(self.output / 'traffic-cleanup.json', dict(
            manager_shutdown=self.manager is not None, owned_vehicle_removed=True,
            note='Traffic cleanup precedes ego destruction and restoration of world settings.'))


def capture(args, output):
    options = json.loads(args.options.read_text())
    if (options.get('speed_selection_attempts', 1) != 1 or options.get('projection_cache_age_s', 0)
            or options.get('async_retention_age_s', 0)):
        raise ValueError('Traffic must preserve one fresh native plan, including its braking intent')
    if options.get('scene_policy'):
        raise ValueError('Traffic is a separate declared scene; empty/red-only guards do not apply')
    if options.get('async_inference') and options.get('async_fresh_planning_interval_s') is None:
        raise ValueError('Traffic async capture requires bounded fresh-state planning')
    scene = TrafficScene(options['traffic'], output, options['validation_route'],
                         options.get('traffic_light_state', 'Green'))
    original_initialize = acknowledged_preroll.initialize_ego
    original_tick = native.runner.CARLAInterface.tick
    original_probe = projected_probe.SingleSampleProbe
    original_follower = projected_probe.ProjectedSteeringFollower
    original_destroy = native.runner.destroy_ego
    timing = (output / 'traffic-step-timing.jsonl').open('w', buffering=1)

    def timed(name, operation):
        def measured(*values, **kwargs):
            started = time.monotonic()
            try:
                return operation(*values, **kwargs)
            finally:
                finished = time.monotonic()
                timing.write(json.dumps(dict(operation=name, started_wall=started,
                    finished_wall=finished, elapsed_s=finished-started)) + '\n')
        return measured

    diagnostic_methods = [
        (native.runner.CARLAInterface, 'get_camera_images', 'sensor_images'),
        (native.runner.CARLAInterface, 'get_history_in_local_frame', 'ego_history'),
        (native.runner.Alpamayo15Adapter, 'prepare_model_input', 'prepare_input'),
        (native.runner.OverheadRecorder, 'add_frame', 'review_video'),
    ]
    originals = [(owner, name, getattr(owner, name)) for owner, name, _ in diagnostic_methods]
    for owner, name, label in diagnostic_methods:
        setattr(owner, name, timed(label, getattr(owner, name)))

    def destroy(interface):
        try:
            scene.close()
        finally:
            original_destroy(interface)

    def initialize(interface, entry, spec):
        scene.spawn(interface, entry, spec)
        result = original_initialize(interface, entry, spec)
        scene.begin()
        return result

    def tick(interface):
        scene.before_tick()
        frame = original_tick(interface)
        if scene.start_frame is not None:
            scene.record(wait_for_frame(interface.world, frame))
        return frame

    class TrafficProbe(original_probe):
        def evaluate(self, *values, **kwargs):
            scene.verify_scope()
            # Actor truth stays in the diagnostic file, outside model inputs.
            return super().evaluate(*values, **kwargs)

    experiment_path = output / 'experiment.json'
    experiment = json.loads(experiment_path.read_text())
    for name in ('traffic_probe.py', 'audit_traffic.py', 'run_traffic_panel.py'):
        relative = 'local/experiments/' + name
        content = (native.runner.ROOT / relative).read_bytes()
        (output / 'source_snapshot' / relative).write_bytes(content)
        experiment['source_sha256'][relative] = hashlib.sha256(content).hexdigest()
    native.write_json(experiment_path, experiment)
    acknowledged_preroll.initialize_ego = initialize
    native.runner.CARLAInterface.tick = timed('simulation_tick_and_traffic_record', tick)
    projected_probe.SingleSampleProbe = TrafficProbe
    if options['traffic'].get('controller_policy') == 'native_speed_no_launch_boost':
        class DeclaredTrafficFollower(NativeSpeedTrafficFollower):
            traffic_speed_preview_lead_s = options['traffic'].get(
                'speed_preview_lead_s', native.runner.defaults.PID_SPEED_LEAD_SEC)
        projected_probe.ProjectedSteeringFollower = DeclaredTrafficFollower
    native.runner.destroy_ego = destroy
    try:
        if options.get('async_inference'):
            from local.experiments import async_projected_probe
            async_projected_probe.capture(args, output)
        else:
            projected_probe.capture(args, output)
    finally:
        for owner, name, operation in originals:
            setattr(owner, name, operation)
        timing.close()
        acknowledged_preroll.initialize_ego = original_initialize
        native.runner.CARLAInterface.tick = original_tick
        projected_probe.SingleSampleProbe = original_probe
        projected_probe.ProjectedSteeringFollower = original_follower
        native.runner.destroy_ego = original_destroy
        scene.close()


if __name__ == '__main__':
    native.capture = capture
    native.main()
