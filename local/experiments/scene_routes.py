"""Build explicit lane-following routes for the opt-in diverse-scene suite."""
import heapq
import itertools
import math

import carla
import numpy as np

from local.experiments.junction_routes import export as export_junction, point


def waypoint(world_map, spec):
    result = world_map.get_waypoint_xodr(int(spec['road_id']), int(spec['lane_id']), float(spec['s']))
    if result is None or result.lane_type != carla.LaneType.Driving:
        raise ValueError(f'No driving waypoint for {spec}')
    return result


def lane_path(start, goal, *, step=.5, maximum_m=220.):
    """Shortest legal forward lane connection; never invent lateral lane changes."""
    def key(w):
        return w.road_id, w.lane_id, round(w.s / step)
    serial = itertools.count()
    initial = key(start)
    costs, nodes, parents = {initial: 0.}, {initial: start}, {initial: None}
    queue = [(0., next(serial), initial)]
    while queue:
        distance, _, current_key = heapq.heappop(queue)
        if distance != costs[current_key]:
            continue
        current = nodes[current_key]
        if (current.road_id, current.lane_id) == (goal.road_id, goal.lane_id):
            ahead = (goal.s - current.s) * (1 if current.lane_id < 0 else -1)
            if -1e-6 <= ahead <= step + 1e-6:
                path = []
                while current_key is not None:
                    path.append(nodes[current_key])
                    current_key = parents[current_key]
                path.reverse()
                if path[-1].transform.location.distance(goal.transform.location) > 1e-5:
                    path.append(goal)
                return path
        if distance >= maximum_m:
            continue
        for following in current.next(step):
            if following.lane_type != carla.LaneType.Driving:
                continue
            next_key = key(following)
            length = current.transform.location.distance(following.transform.location)
            if length > 2. or length < 1e-6:
                continue
            candidate = distance + length
            if candidate < costs.get(next_key, math.inf):
                costs[next_key], nodes[next_key], parents[next_key] = candidate, following, key(current)
                heapq.heappush(queue, (candidate, next(serial), next_key))
        if len(nodes) > 60000:
            raise ValueError('Route search exceeded its finite node budget')
    raise ValueError(f'No forward lane path within {maximum_m} m')


def padding(anchor, method, distance):
    points, current = [], anchor
    for _ in range(round(distance / .5)):
        choices = getattr(current, method)(.5)
        if len(choices) != 1:
            break
        current = choices[0]
        points.append(current)
    return points


def export(world_map, entry, spec, name):
    if spec['kind'] == 'junction':
        result = export_junction(world_map, entry, required_routes=(name,))
    elif spec['kind'] == 'lane_path':
        start, end = waypoint(world_map, spec['start']), waypoint(world_map, spec['end'])
        core = lane_path(start, end, maximum_m=spec.get('maximum_m', 220.))
        prefix, suffix = padding(start, 'previous', 5.), padding(end, 'next', 12.)
        points = list(reversed(prefix)) + core + suffix
        route = dict(entry=point(start), exit=point(end), entry_index=len(prefix),
                     exit_index=len(prefix)+len(core)-1, points=[point(w) for w in points],
                     heading_change_deg=(end.transform.rotation.yaw-start.transform.rotation.yaw+180)%360-180)
        result = dict(map=world_map.name, junction_id=None, routes={name: route})
    else:
        raise ValueError(f'Unknown route kind: {spec["kind"]}')
    route = result['routes'][name]
    xy = np.array([[p['x'], p['y']] for p in route['points']])
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    route['length_from_entry_m'] = float(arc[route['exit_index']]-arc[route['entry_index']])
    if 'instruction' in spec:
        route['instruction'] = spec['instruction']
    if 'exit_instruction' in spec:
        route['exit_instruction'] = spec['exit_instruction']
        route['finish_after_arc_m'] = float(arc[route['exit_index']]-spec.get('finish_before_exit_m', 8.))
    result['route_spec'] = spec
    result['note'] = 'Explicit CARLA lane connectivity supplies map-assisted steering targets and audit geometry.'
    return result


def scene_scope(world, policy):
    """Record live empty-scene/signal checks, including roads with no nearby light."""
    actors = world.get_actors()
    ego_count = sum(a.attributes.get('role_name') == 'carlamayo_intersection' for a in actors.filter('vehicle.*'))
    other_vehicles = [a.id for a in actors.filter('vehicle.*')
                      if a.attributes.get('role_name') != 'carlamayo_intersection']
    pedestrians = [a.id for a in actors.filter('walker.pedestrian.*')]
    lights = list(actors.filter('traffic.traffic_light'))
    expected = 'Red' if policy == 'red_light_stop' else 'Green'
    bad_signals = [a.id for a in lights if str(a.get_state()) != expected]
    if policy not in ('empty_green', 'red_light_stop') or ego_count != 1 or other_vehicles or pedestrians or bad_signals or (policy == 'red_light_stop' and not lights):
        raise RuntimeError('Controlled-scene scope changed: traffic or unexpected signal state')
    return dict(verified=True, policy=policy, ego_count=ego_count, other_vehicle_count=0, pedestrian_count=0,
                map_traffic_light_count=len(lights), expected_signal_state=expected,
                unexpected_signal_count=0)
