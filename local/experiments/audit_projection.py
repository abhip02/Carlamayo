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


def sampling_budget_checks(selection, options):
    budget = options.get('speed_selection_max_seconds')
    if budget is None:
        return {}
    starts = [p.get('start_elapsed_seconds', math.inf) for p in selection['proposals']]
    return dict(
        sampling_time_budget_config=(math.isfinite(budget) and budget > 0
                                     and selection.get('max_seconds') == budget),
        sampling_retry_time_records=bool(starts) and all(math.isfinite(t) and t >= 0 for t in starts)
                                    and starts == sorted(starts),
        no_retry_after_sampling_budget=all(t < budget for t in starts[1:]),
        sampling_elapsed_record=bool(starts) and selection.get('elapsed_seconds', -1) >= max(starts))


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
        held = False
        def _target_speed(self, speeds_mps, speed_mps, elapsed_sec):
            if self.held:
                self._launching = False
                return 0., False
            value, launch = super()._target_speed(speeds_mps, speed_mps, elapsed_sec)
            return (min(value, speed_cap) if speed_cap is not None else value), launch
    shadow = SpeedCeilingReference(vehicle)
    hold_enabled = options.get('hold_visual_stop', False)
    hold_check_index, previous_stop = -1, False
    mismatches, distances, lanes = [], [], []
    for i, (record, saved) in enumerate(zip(records, controls)):
        def check(ok, label):
            if not ok:
                mismatches.append({'index': i, 'check': label})
        context = record['context']
        retention=record.get('retention')
        source_context=retention['source_context'] if retention else context
        selection = record.get('selection', {})
        if options.get('scene_policy'):
            scope = context.get('controlled_scene', {})
            check(scope.get('verified') is True and scope.get('policy') == options['scene_policy']
                  and scope.get('other_vehicle_count') == 0 and scope.get('pedestrian_count') == 0
                  and scope.get('unexpected_signal_count') == 0, 'controlled_scene_scope')
        if selection.get('kind') == 'single_native_stop_scene':
            chosen = json.loads((capture / (selection['selected_proposal'] + '.json')).read_text())
            check(chosen['trajectory_ego'] == record['trajectory_ego'] and chosen['context'] == context,
                  'stop_scene_native_source_unchanged')
            check(selection['attempts'] == selection['budget'] == 1 and not retention,
                  'stop_scene_no_motion_resampling_or_reuse')
        if selection.get('kind') == 'bounded_native_speed_preference':
            proposals = selection['proposals']
            check(len(proposals) == selection['attempts'] <= selection['budget'], 'selection_budget')
            for label, ok in sampling_budget_checks(selection, options).items():
                check(ok, label)
            chosen = json.loads((capture / (selection['selected_proposal'] + '.json')).read_text())
            check(chosen['trajectory_ego'] == record['trajectory_ego'], 'selected_native_proposal_unchanged')
            check(chosen['context'] == source_context, 'selected_original_input')
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
            if not retention:
                check(proposals[expected]['label'] == selection['selected_proposal'], 'selection_ranking')
            else:
                age=context['time_s']-source_context['time_s']
                check(0<age<=options['projection_cache_age_s']+1e-6,'cache_age_bound')
                check(abs(age-retention['age_s'])<1e-6,'cache_actual_age')
                check(not selection['visual_stop_preserved'] and record['visual_signal'] not in ('red','yellow'),'no_cache_over_visual_stop')
                check(chosen['reverse_velocity_steps']==0 and chosen['trajectory_marker_present'],'cached_native_forward_valid')
            check(abs(total-record['inference_seconds']) < 1e-5, 'all_sampling_time_counted')
            if selection['visual_stop_preserved']:
                check(len(proposals) == 1 and record['visual_signal'] in ('red','yellow'), 'visual_stop_not_resampled')
        tf = pose_transform(context['world_pose'])
        raw = np.array(record['trajectory_ego'])
        world, speeds = trajectory_to_world(pose_transform(source_context['world_pose']), raw), planned_speeds(raw)
        check(abs(saved['elapsed_sec']-(context['time_s']-source_context['time_s']))<1e-6,'original_prediction_time')
        check(np.allclose(world, saved['world_trajectory'], atol=1e-6, rtol=0), 'native_world_unchanged')
        check(np.array_equal(speeds, saved['native_speeds']), 'native_speeds_unchanged')
        previous = shadow._prev_steer
        if hold_enabled:
            signal, signal_index = record['visual_signal'], record['visual_signal_check_index']
            check(isinstance(signal_index, int) and 0 <= signal_index <= i, 'visual_signal_index')
            was_held = shadow.held
            if was_held:
                shadow.held = not (signal == 'green' and signal_index > hold_check_index)
            elif signal in ('red', 'yellow') and context['measured_speed_mps'] <= .2 and previous_stop:
                shadow.held = True
                hold_check_index = signal_index
            expected_hold = dict(active=shadow.held, activated=shadow.held and not was_held,
                released=was_held and not shadow.held, signal=signal, check_index=signal_index,
                activation_check_index=hold_check_index)
            check(saved['debug'].get('visual_stop_hold') == expected_hold, 'visual_stop_hold_state')
        steer, throttle, brake, debug = shadow.compute_control(
            tf, world, context['measured_speed_mps'], speeds, saved['elapsed_sec'])
        previous_stop = debug['planned_stop']
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
                # CARLA locations store float32 coordinates. Keep the independent
                # segment projection, then reproduce that API boundary exactly.
                point=carla.Location(x=float(point[0]),y=float(point[1]),z=float(point[2]))
                projected=[point.x,point.y,point.z]
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
        if retention:
            low,high=selection['preference_mps']
            retained_speed=debug['target_speed_kmh']/3.6
            check(1.-1e-6<=retained_speed<=high+1e-6,'retained_moving_target')
            retained_gap=max(low-retained_speed,retained_speed-high,0.)
            check(retained_gap<=min(ranks)[0]+1e-5,'retained_no_worse_than_fresh_speed_preference')
            if 'preference_distance_mps' in selection:
                check(abs(retained_gap-selection['preference_distance_mps'])<1e-5,'retained_preference_distance')
                check(selection['preference_met']==(selection['preference_distance_mps']==0),'retained_preference_flag')
            offset=np.asarray(projected[:2])-[axle.x,axle.y]
            yaw=math.radians(tf.rotation.yaw)
            check(float(np.dot(offset,[math.cos(yaw),math.sin(yaw)]))>1.,'retained_target_ahead')
            route=routes['routes'][selection['route']]
            _,arc,_,_,_=audit_turn.project(np.array([[axle.x,axle.y],projected[:2]]),route)
            check(arc[1]-arc[0]>.25,'retained_route_progress')
            bounds=context['vehicle_bounds']
            box=carla.BoundingBox(carla.Location(**bounds['location']),carla.Vector3D(**bounds['extent']))
            box.rotation=carla.Rotation(**bounds['rotation'])
            vertices=box.get_world_vertices(tf)
            d,_,_,width,_=audit_turn.project(np.array([[v.x,v.y] for v in vertices]),route)
            check(np.all(d<=width),'retained_current_body_inside')
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
        hold_visual_stop=hold_enabled,
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
        cache_uses=sum('retention' in r for r in records),
        max_cache_age_s=max((r.get('retention',{}).get('age_s',0.) for r in records),default=0.),
        wall_seconds=summary['wall_seconds'], mismatches=mismatches,
        integrity_pass=not mismatches,
        passed=bool(not mismatches and endpoint is not None and not footprint['corner_outside_times_s']
                    and min(speeds) >= 1. and (speed_cap is None or max(speeds) <= speed_cap+1.)
                    and summary['termination'] != 'collision'),
        note='Map-assisted steering is explicitly allowed here. Geometry measures recorded corners, not swept volume or obstacle safety.')
    if options.get('mission', {}).get('kind') == 'red_light_stop':
        from local.experiments.stop_mission import audit_stop
        result['mission'] = audit_stop(rows, records[0]['context']['vehicle_bounds'], options['mission'])
        result['passed'] = bool(not mismatches and result['mission']['passed']
            and not footprint['corner_outside_times_s'] and summary['termination'] != 'collision'
            and (speed_cap is None or max(speeds) <= speed_cap+1.))
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
