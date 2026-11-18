"""Assess declared traffic missions separately from round-19's empty-road criteria."""
import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np

from local.experiments import audit_projection


def box_clearance(a, b):
    """Distance between horizontally projected convex vehicle boxes, in metres."""
    a = cv2.convexHull(np.asarray(a, dtype=np.float32))
    b = cv2.convexHull(np.asarray(b, dtype=np.float32))
    area, _ = cv2.intersectConvexConvex(a, b)
    if area > 0:
        return 0.
    return min(abs(cv2.pointPolygonTest(other, tuple(map(float, point)), True))
               for vertices, other in ((a, b), (b, a)) for point in vertices[:, 0])


def front_gap(vertices, line):
    direction = np.array([math.cos(math.radians(line['yaw'])), math.sin(math.radians(line['yaw']))])
    return float(-np.max((np.asarray(vertices) - [line['x'], line['y']]) @ direction))


def mission_checks(spec, states, geometric):
    rows = states
    if not rows:
        raise ValueError('Traffic audit requires recorded states')
    clearances = [box_clearance(row['ego']['vertices_xy'], actor['vertices_xy'])
                  for row in rows for actor in row['traffic']]
    checks = dict(execution_integrity=geometric['integrity_pass'],
                  no_recorded_collision=geometric['termination'] != 'collision',
                  no_recorded_lane_departure=not geometric['body_departure_times_s'],
                  speed_ceiling=geometric['max_speed_mps'] <= 5.5,
                  one_call_per_decision=geometric['inference_calls'] == (
                      geometric['inference_requests'] if geometric.get('async_inference') else geometric['steps']),
                  no_cached_plan=geometric['cache_uses'] == 0 and geometric.get('async_retention_uses', 0) == 0,
                  declared_actor_count=all(len(row['traffic']) == (0 if spec['kind'] == 'empty' else 1) for row in rows),
                  minimum_box_clearance=not clearances or min(clearances) >= 1.)
    if spec.get('audit_version') == 2:
        checks['traffic_scope_every_tick'] = all(row.get('scope_verified') is True for row in rows)
        checks['declared_signals_every_tick'] = all(row.get('signal_state') == spec['signal_state'] for row in rows)
    if geometric.get('async_inference'):
        checks['bounded_fresh_planning'] = (geometric.get('async_fresh_planning_interval_s') is not None
            and geometric['max_executed_plan_age_s'] <= min(3., max(
                geometric['async_fresh_planning_interval_s'] +
                (geometric.get('async_motion_commitment_s') or 0.),
                geometric.get('async_rest_planning_interval_s') or 0.)) + 1e-6)
        checks['no_expired_plan_execution'] = geometric['expired_plan_braking_steps'] == 0
    if not checks['declared_actor_count']:
        return checks, clearances
    if spec['kind'] in ('stationary_lead', 'braking_lead'):
        tail = [r for r in rows if r['time_s'] >= spec['duration_s'] - 2. - 1e-6]
        checks.update(full_declared_duration=rows[-1]['time_s'] >= spec['duration_s'] - 1e-6,
                      stopped_for_final_two_seconds=len(tail) >= 21 and all(r['ego']['speed_mps'] <= .2 for r in tail),
                      stopped_within_eight_metres=bool(tail) and all(
                          box_clearance(r['ego']['vertices_xy'], r['traffic'][0]['vertices_xy']) <= 8. for r in tail))
        if spec['kind'] == 'braking_lead':
            checks['lead_actually_moved'] = max(r['traffic'][0]['speed_mps'] for r in rows) >= 1.
            checks['lead_actually_stopped'] = bool(tail) and all(r['traffic'][0]['speed_mps'] <= .2 for r in tail)
            checks['lead_braking_event_observed'] = any(r['lead_braking_commanded'] for r in rows)
    else:
        checks['declared_route_completed'] = geometric['reached_exit_time_s'] is not None
        if spec['kind'] == 'moving_lead':
            checks['lead_actually_moved'] = max(r['traffic'][0]['speed_mps'] for r in rows) >= 1.
            if spec.get('audit_version') == 2:
                close = [box_clearance(r['ego']['vertices_xy'], r['traffic'][0]['vertices_xy']) <= 20.
                         for r in rows]
                checks['traffic_proximity_exposure'] = sum(close) / len(close) >= .5
                longest = current = 0
                for r in rows:
                    free_to_move = (r['traffic'][0]['speed_mps'] >= 1. and
                        box_clearance(r['ego']['vertices_xy'], r['traffic'][0]['vertices_xy']) >= 6.)
                    current = current + 1 if free_to_move and r['ego']['speed_mps'] <= .2 else 0
                    longest = max(longest, current)
                checks['no_prolonged_unnecessary_stop'] = longest * .1 <= 2. + 1e-6
    if spec.get('signal_state') == 'Red':
        if spec.get('queue_stop_line'):
            if spec['kind'] != 'stationary_lead':
                raise ValueError('Red queue mission requires a declared stationary lead')
            line=spec['queue_stop_line']
            checks['ego_stays_before_red_line'] = all(front_gap(r['ego']['vertices_xy'], line) >= 0. for r in rows)
            checks['queued_lead_stays_before_red_line'] = all(front_gap(r['traffic'][0]['vertices_xy'], line) >= 0. for r in rows)
            checks['queued_lead_stopped_during_hold'] = bool(tail) and all(r['traffic'][0]['speed_mps'] <= .2 for r in tail)
        else:
            checks['original_red_stop_mission'] = geometric.get('mission', {}).get('passed') is True
    return checks, clearances


def lead_route_audit(states, route):
    """Verify the declared NPC route; never provide actor truth to ego control.

    Once the lead crosses the recorded route's final plane within its lane and
    heading, later travel is outside this reference's scope. Prior departures
    remain failures. Ego mission endpoints and criteria are unaffected.
    """
    points = route['points']
    xy = np.array([[p['x'], p['y']] for p in points], dtype=float)
    delta = np.diff(xy, axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    if (len(delta) == 0 or not np.isfinite(xy).all() or np.any(lengths <= 1e-8)
            or any(not math.isfinite(p['lane_width']) or p['lane_width'] <= 0 for p in points)):
        raise ValueError('Lead route requires finite nondegenerate geometry and lane widths')
    arc = np.r_[0., np.cumsum(lengths)]
    failures, rows = [], []
    reached = None
    furthest = -math.inf
    coverage = bool(states)
    for state in states:
        actors = state.get('traffic', [])
        valid = len(actors) == 1 and all(
            isinstance(actors[0].get(k), (int, float)) and math.isfinite(actors[0][k])
            for k in ('x', 'y', 'yaw', 'speed_mps'))
        coverage = coverage and valid
        if not valid:
            failures.append(dict(time_s=state.get('time_s'), reason='missing_or_nonfinite_lead_state'))
            continue
        if reached is not None:
            continue
        actor = actors[0]
        position = np.array([actor['x'], actor['y']])
        fraction = np.clip(((position - xy[:-1]) * delta).sum(axis=1) / lengths**2, 0., 1.)
        distances = np.linalg.norm(position - xy[:-1] - fraction[:, None] * delta, axis=1)
        index = int(distances.argmin())
        heading = math.degrees(math.atan2(delta[index, 1], delta[index, 0]))
        error = abs((actor['yaw'] - heading + 180) % 360 - 180)
        progress = float(arc[index] + fraction[index] * lengths[index])
        last_direction = delta[-1] / lengths[-1]
        beyond = float((position - xy[-1]) @ last_direction)
        offset = position - xy[-1]
        cross = abs(float(last_direction[0] * offset[1] - last_direction[1] * offset[0]))
        last_heading = math.degrees(math.atan2(last_direction[1], last_direction[0]))
        exit_heading_error = abs((actor['yaw'] - last_heading + 180) % 360 - 180)
        row = dict(time_s=state['time_s'], distance_m=float(distances[index]),
                   heading_error_deg=error, route_arc_m=progress)
        rows.append(row)
        if (index == len(delta) - 1 and fraction[index] >= 1. - 1e-8
                and beyond >= -1e-6 and cross <= points[-1]['lane_width'] / 2. + .25
                and exit_heading_error <= 20.):
            reached = state['time_s']
            continue
        reasons = []
        if distances[index] > points[index]['lane_width'] / 2. + .25:
            reasons.append('outside_declared_route_lane')
        if actor['speed_mps'] > .2 and error > 45.:
            reasons.append('wrong_route_heading')
        if progress < furthest - 1.:
            reasons.append('backward_route_progress')
        furthest = max(furthest, progress)
        if reasons:
            failures.append({**row, 'reasons': reasons})
    return dict(passed=coverage and not failures, record_coverage=coverage,
                checked_ticks=len(rows), total_ticks=len(states),
                route_suffix_reached_s=reached, failures=failures,
                max_centerline_distance_m=max((r['distance_m'] for r in rows), default=None),
                scope='Lead center within local half-lane width plus0.25m, moving heading within45degrees, no backward route jump over1m; checked until a valid crossing of the recorded suffix end. No swept-footprint claim.',
                rows=rows)


def declared_sampling_matches(requests, spec):
    return bool(requests) and all(
        row.get('options', {}).get('num_samples') == 1
        and row['options'].get('flow_temperature', 1.) == spec['flow_temperature']
        and row['options'].get('flow_steps', 10) == 10
        and ('controller_speed_cap_mps' not in spec or
             row['options'].get('controller_speed_cap_mps') == spec['controller_speed_cap_mps'])
        for row in requests)


def declared_generation_matches(requests, spec):
    """Check the effective path, including the unchanged model visual-stop path."""
    mode = spec.get('trajectory_generation', 'reasoned')
    if mode not in ('reasoned', 'direct') or not requests:
        return False
    for row in requests:
        options = row.get('options', {})
        if options.get('traffic', {}).get('trajectory_generation', 'reasoned') != mode:
            return False
        if mode == 'direct':
            stop = row.get('visual_signal') in ('red', 'yellow')
            prefix = (f"<|cot_start|>Stop and remain stopped for the {row['visual_signal']} traffic light"
                      if stop else '<|traj_future_start|>')
            if (row.get('trajectory_generation') != mode
                    or row.get('effective_prompt_style') != 'trajectory_only'
                    or row.get('effective_max_generation_length') != (256 if stop else 1)
                    or row.get('assistant_prefix') != prefix
                    or row.get('trajectory_marker_present') is not True
                    or (not stop and row.get('generated_tokens') != 1)
                    or options.get('verify_signal') is not True):
                return False
    return True


def audit(capture):
    capture = Path(capture)
    geometric = audit_projection.audit(capture)
    options = json.loads((capture / 'experiment.json').read_text())['options']
    spec = options['traffic']
    states = [json.loads(line) for line in (capture / 'traffic-states.jsonl').read_text().splitlines()]
    csv_rows = list(csv.DictReader((capture / '000_verifai/ego.csv').open()))
    if len(states) != len(csv_rows):
        raise ValueError('Traffic trace does not cover every recorded ego tick')
    for row, state in zip(csv_rows, states, strict=True):
        if int(row['frame']) != state['frame'] or abs(float(row['time_s']) - state['time_s']) > 1e-6:
            raise ValueError('Traffic and ego timestamps differ')
    checks, clearances = mission_checks(spec, states, geometric)
    if 'flow_temperature' in spec or 'trajectory_generation' in spec:
        requests = [json.loads(path.read_text()) for path in sorted(capture.glob('async_request_*.json'))]
    if 'flow_temperature' in spec:
        sampling_valid = declared_sampling_matches(requests, spec)
        checks['declared_native_sampling'] = sampling_valid
        checks['execution_integrity'] = checks['execution_integrity'] and sampling_valid
        geometric['integrity_pass'] = geometric['integrity_pass'] and sampling_valid
    if 'trajectory_generation' in spec:
        generation_valid = declared_generation_matches(requests, spec)
        checks['declared_trajectory_generation'] = generation_valid
        checks['execution_integrity'] = checks['execution_integrity'] and generation_valid
        geometric['integrity_pass'] = geometric['integrity_pass'] and generation_valid
    if spec.get('controller_policy') == 'native_speed_no_launch_boost':
        controls = [json.loads(path.read_text()) for path in sorted(capture.glob('control_*.json'))]
        checks['native_speed_without_launch_boost'] = bool(controls) and all(
            row['debug'].get('traffic_launch_boost_disabled') is True
            and row['debug'].get('launch') is False for row in controls)
        if 'speed_preview_lead_s' in spec:
            checks['declared_native_speed_preview'] = bool(controls) and all(
                row['debug'].get('traffic_speed_preview_lead_s') == spec['speed_preview_lead_s']
                for row in controls)
    ego_checks_passed = all(checks.values())
    traffic_setup_pass = True
    if spec.get('lead_route_audit_version') == 1:
        route = json.loads((capture / 'route-reference.json').read_text())['routes'][options['validation_route']]
        lead_audit = lead_route_audit(states, route)
        (capture / 'lead-route-audit.json').write_text(json.dumps(lead_audit, indent=2) + '\n')
        traffic_setup_pass = lead_audit['passed']
        checks['declared_lead_route'] = traffic_setup_pass
        checks['execution_integrity'] = checks['execution_integrity'] and traffic_setup_pass
        geometric['integrity_pass'] = geometric['integrity_pass'] and traffic_setup_pass
    times = [float(r['inference_s']) for r in csv_rows[1:]]
    result = dict(passed=all(checks.values()), integrity_pass=geometric['integrity_pass'],
                  ego_driving_checks_passed=ego_checks_passed, traffic_setup_pass=traffic_setup_pass,
                  checks=checks, traffic_specification=spec,
                  min_vehicle_box_clearance_m=min(clearances) if clearances else None,
                  geometric_audit=geometric,
                  decision_p50_s=float(np.median(times)), decision_p95_s=float(np.percentile(times, 95)),
                  capture_wall_per_simulated_second=geometric['wall_seconds'] / geometric['duration_s'],
                  wall_seconds=geometric['wall_seconds'], duration_s=geometric['duration_s'],
                  inference_calls=geometric['inference_calls'], inference_seconds=geometric['inference_seconds'],
                  note='Traffic mission criteria were declared before capture. The original empty-road audit is retained separately; stopping is required in stop missions. Geometry is sampled at 0.1 s, not a swept-volume proof.')
    (capture / 'traffic-audit.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    args = parser.parse_args()
    result = audit(args.capture)
    print(json.dumps({k: v for k, v in result.items() if k != 'geometric_audit'}, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
