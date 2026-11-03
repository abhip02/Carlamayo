"""Check native speeds, map lookup, executed steering, and recorded route geometry."""
import argparse
import csv
import json
import math
from pathlib import Path
from types import SimpleNamespace

import carla
import numpy as np

from local.experiments import audit_body, audit_turn
from local.experiments.validated_selection import pose_transform
from module import config as cfg
from module.trajectory_follower import TrajectoryFollower, planned_speeds, trajectory_to_world, pure_pursuit_steer


def audit(capture):
    capture = Path(capture)
    records = [json.loads(p.read_text()) for p in sorted(capture.glob('frame_*.json'))]
    controls = [json.loads(p.read_text()) for p in sorted(capture.glob('control_*.json'))]
    rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader((capture / '000_verifai/ego.csv').open())]
    assert records and len(records) == len(controls) == len(rows) - 1
    options = json.loads((capture / 'experiment.json').read_text())['options']
    mode = options['steering_mode']
    speed_cap, steering_gain = options.get('controller_speed_cap_mps'), options.get('steering_gain',1.)
    routes = json.loads((capture / 'route-reference.json').read_text())
    world_map = carla.Map('archived', (capture / 'map.xodr').read_text()) if mode == 'map' else None
    vehicle = SimpleNamespace(get_physics_control=lambda: SimpleNamespace(wheels=[
        SimpleNamespace(max_steer_angle=math.degrees(controls[0]['max_steer_rad']))]))
    class SpeedCeilingReference(TrajectoryFollower):
        def _target_speed(self, speeds_mps, speed_mps, elapsed_sec):
            value, launch = super()._target_speed(speeds_mps, speed_mps, elapsed_sec)
            return (min(value, speed_cap) if speed_cap is not None else value), launch
    shadow = SpeedCeilingReference(vehicle)
    mismatches, distances, lanes = [], [], []
    for i, (record, saved) in enumerate(zip(records, controls)):
        def check(ok, label):
            if not ok:
                mismatches.append({'index': i, 'check': label})
        context = record['context']
        selection = record.get('selection', {})
        if selection.get('kind') == 'bounded_native_speed_preference':
            proposals = selection['proposals']
            check(len(proposals) == selection['attempts'] <= selection['budget'], 'selection_budget')
            chosen = json.loads((capture / (selection['selected_proposal'] + '.json')).read_text())
            check(chosen['trajectory_ego'] == record['trajectory_ego'], 'selected_native_proposal_unchanged')
            check(chosen['context'] == context, 'selected_current_input')
            low, high = selection['preference_mps']
            ranks = []
            total = 0.
            for proposal in proposals:
                raw_proposal = json.loads((capture / (proposal['label'] + '.json')).read_text())
                pts = np.asarray(raw_proposal['trajectory_ego'])
                v = float(planned_speeds(pts)[10:15].mean())
                valid = (pts.shape == (64,3) and np.isfinite(pts).all() and np.isfinite(v)
                         and raw_proposal['trajectory_marker_present'] and not raw_proposal['reverse_velocity_steps'])
                effective = min(v,speed_cap) if speed_cap is not None else v
                ranks.append((max(low-effective,effective-high,0.),abs(effective-(low+high)/2)) if valid else (math.inf,math.inf))
                total += raw_proposal['inference_seconds']
                check(abs(v-proposal['target_mps']) < 1e-6, 'selection_native_speed_score')
            expected = 0 if selection['visual_stop_preserved'] else min(range(len(ranks)),key=lambda j:ranks[j])
            check(proposals[expected]['label'] == selection['selected_proposal'], 'selection_ranking')
            check(abs(total-record['inference_seconds']) < 1e-5, 'all_sampling_time_counted')
            if selection['visual_stop_preserved']:
                check(len(proposals) == 1 and record['visual_signal'] in ('red','yellow'), 'visual_stop_not_resampled')
        tf = pose_transform(context['world_pose'])
        raw = np.array(record['trajectory_ego'])
        world, speeds = trajectory_to_world(tf, raw), planned_speeds(raw)
        check(np.allclose(world, saved['world_trajectory'], atol=1e-6, rtol=0), 'native_world_unchanged')
        check(np.array_equal(speeds, saved['native_speeds']), 'native_speeds_unchanged')
        previous = shadow._prev_steer
        steer, throttle, brake, debug = shadow.compute_control(
            tf, world, context['measured_speed_mps'], speeds, saved['elapsed_sec'])
        target = world[debug['target_idx']]
        check(np.allclose(target, saved['debug']['native_target_xyz'], atol=1e-6, rtol=0), 'native_lookahead_target')
        if mode in ('map','route'):
            if mode == 'map':
                wp = world_map.get_waypoint(carla.Location(x=float(target[0]), y=float(target[1]), z=float(target[2])),
                    project_to_road=True, lane_type=carla.LaneType.Driving)
                assert wp is not None
                point = wp.transform.location
                projected = [point.x, point.y, point.z]
                lane_ids = [wp.road_id, wp.lane_id]
            else:
                # Independent scalar projection onto each commanded-route segment.
                points = routes['routes'][record['selection']['route']]['points']
                candidates = []
                for a,b in zip(points,points[1:]):
                    av=np.array([a['x'],a['y'],a['z']]);bv=np.array([b['x'],b['y'],b['z']]);v=bv-av
                    norm=float(v[0]**2+v[1]**2)
                    if norm<1e-12:continue
                    u=max(0.,min(1.,float(np.dot(target[:2]-av[:2],v[:2]))/norm))
                    q=av+u*v
                    candidates.append((float(np.linalg.norm(target[:2]-q[:2])),q,[a['road_id'],a['lane_id']]))
                _,point,lane_ids=min(candidates,key=lambda c:c[0])
                projected=point.tolist()
            check(np.allclose(projected, saved['debug']['projected_target_xyz'], atol=1e-4, rtol=0), 'map_lookup_matches')
            check(lane_ids == [saved['debug']['projected_lane']['road_id'],saved['debug']['projected_lane']['lane_id']], 'map_lane_matches')
            axle = tf.transform(carla.Location(x=cfg.EGO_REAR_AXLE_X))
            steer = pure_pursuit_steer([axle.x, axle.y], math.radians(tf.rotation.yaw),
                projected[:2], cfg.EGO_WHEELBASE_M, shadow.max_steer_rad)
            steer = float(np.clip(steer * steering_gain,-1.,1.))
            delta = cfg.FOLLOW_STEER_RATE_PER_SEC * cfg.CONTROL_DT
            steer = float(np.clip(steer, previous - delta, previous + delta))
            distance = float(np.linalg.norm(target[:2] - projected[:2]))
            check(abs(distance - saved['debug']['projection_distance_m']) < 1e-4, 'projection_distance_matches')
            distances.append(distance)
            lanes.append(lane_ids)
        shadow._prev_steer = steer
        check(np.allclose([steer, throttle, brake], saved['control'], atol=1e-6, rtol=0), 'controller_command_matches')
        check(np.allclose(saved['control'], [rows[i+1][k] for k in ('steer','throttle','brake')], atol=1e-6, rtol=0), 'applied_control_matches')
        check(abs(debug['target_speed_kmh'] - saved['debug']['target_speed_kmh']) < 1e-6, 'declared_speed_policy')
        if speed_cap is not None:
            check(saved['debug']['target_speed_kmh']/3.6 <= speed_cap+1e-6,'speed_ceiling_applied')
        check(np.allclose([context['world_pose'][k] for k in ('x','y','yaw')],
                          [rows[i][k] for k in ('x','y','yaw')], atol=1e-5, rtol=0), 'snapshot_matches_recording')
    route_report = audit_turn.audit(capture, routes)
    footprint = audit_body.audit(capture)
    summary = json.loads((capture / '000_verifai/summary.json').read_text())
    route_name = records[0]['selection']['route']
    endpoint = route_report['routes'][route_name]['reached_exit_time_s']
    speeds = [r['speed'] for r in rows]
    all_proposals = [json.loads(p.read_text()) for p in sorted(capture.glob('proposal_*.json'))]
    executed_calls = sum(r.get('selection',{}).get('attempts',1) for r in records)
    result = dict(steering_mode=mode, controller_speed_cap_mps=speed_cap, steering_gain=steering_gain,
        route=route_name, steps=len(controls),
        duration_s=rows[-1]['time_s'], termination=summary['termination'],
        reached_exit_time_s=endpoint, min_speed_mps=min(speeds), max_speed_mps=max(speeds),
        final_speed_mps=speeds[-1], near_stop_times_s=[r['time_s'] for r in rows if r['speed'] < 1.],
        min_corner_clearance_m=-footprint['max_corner_overhang_m'],
        body_departure_times_s=footprint['corner_outside_times_s'],
        max_projection_distance_m=max(distances, default=0.),
        projected_lane_switches=sum(a != b for a,b in zip(lanes,lanes[1:])),
        projection_trace=[{'time_s':c['time_s'],'distance_m':c['debug']['projection_distance_m'],
                           'lane':c['debug'].get('projected_lane')} for c in controls],
        inference_seconds=sum(r['inference_seconds'] for r in (all_proposals or records)),
        inference_calls=len(all_proposals) if all_proposals else len(records),
        failed_decision_inference_calls=max(0,len(all_proposals)-executed_calls),
        speed_preference_misses=sum(r.get('selection',{}).get('preference_met') is False for r in records),
        wall_seconds=summary['wall_seconds'], mismatches=mismatches,
        integrity_pass=not mismatches,
        passed=bool(not mismatches and endpoint is not None and not footprint['corner_outside_times_s']
                    and min(speeds) >= 1. and (speed_cap is None or max(speeds) <= speed_cap+1.)
                    and summary['termination'] != 'collision'),
        note='Map-assisted steering is explicitly allowed here. Geometry measures recorded corners, not swept volume or obstacle safety.')
    for name, report in [('projection-audit',result),('route-audit',route_report),('footprint-audit',footprint)]:
        (capture / (name+'.json')).write_text(json.dumps(report,indent=2)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    result = audit(parser.parse_args().capture)
    print(json.dumps({k:v for k,v in result.items() if k not in ('projection_trace','near_stop_times_s','body_departure_times_s')},indent=2))
    raise SystemExit(0 if result['integrity_pass'] else 1)


if __name__ == '__main__':
    main()
