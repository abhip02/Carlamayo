"""Check native speeds, map lookup, executed steering, and recorded route geometry."""
import argparse
import csv
import hashlib
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


def motion_interval_reference(request, base_interval, commitment):
    """Independently derive the allowed age from native positions and source state."""
    if commitment is None:
        return base_interval
    points = request['trajectory_ego']
    if (not 0. <= request['context']['measured_speed_mps'] <= .2
            or request.get('visual_signal') in ('red', 'yellow')
            or request.get('reverse_velocity_steps', 0)
            or len(points) != 64 or any(len(p) != 3 for p in points)
            or any(not math.isfinite(v) for p in points for v in p)
            or points[-1][0] < 3.):
        return base_interval
    speeds = [math.dist([0., 0.], points[0][:2]) * 10.]
    speeds += [math.dist(a[:2], b[:2]) * 10. for a, b in zip(points, points[1:])]
    lead = round(request.get('options', {}).get('traffic', {}).get(
        'speed_preview_lead_s', cfg.PID_SPEED_LEAD_SEC) * 10)
    count = round(cfg.PID_SPEED_PREVIEW_SEC * 10)
    if sum(speeds[lead:lead + count]) / count > cfg.PID_STOP_SPEED_MPS:
        return base_interval
    for tick in range(round(base_interval * 10)):
        target = sum(speeds[lead + tick:lead + tick + count]) / count
        if target > cfg.PID_STOP_SPEED_MPS:
            return min(3., max(base_interval, round(tick / 10. + commitment, 10)))
    return base_interval


def rest_schedule_reference(request, interval):
    """Reconstruct restart eligibility without trusting the scheduler flag."""
    points = np.asarray(request['trajectory_ego'])
    if (request['context']['measured_speed_mps'] > .2
            or request.get('visual_signal') in ('red', 'yellow')
            or request.get('reverse_velocity_steps', 0)
            or points.shape != (64, 3) or not np.isfinite(points).all()
            or points[-1, 0] < 3.):
        return False
    values = planned_speeds(points).tolist()
    lead = request.get('options', {}).get('traffic', {}).get('speed_preview_lead_s', cfg.PID_SPEED_LEAD_SEC)
    start = int(round(lead * 10))
    count = int(round(cfg.PID_SPEED_PREVIEW_SEC * 10))
    late = start + int(round(interval * 10)) - 1
    return (sum(values[start:start + count]) / count <= cfg.PID_STOP_SPEED_MPS
            and sum(values[late:late + count]) / count > cfg.PID_STOP_SPEED_MPS)


def rest_schedule_checks(request, capture, options):
    interval = options.get('async_rest_planning_interval_s')
    if interval is None:
        return {}
    candidate = rest_schedule_reference(request, interval)
    perception = request.get('rest_motion_check')
    checks = {'rest_visual_query_required': bool(perception) == candidate}
    status = 'unknown'
    if perception:
        answer = perception['answer'].strip().lower().rstrip('.').strip()
        status = answer if answer in ('moving', 'stopped', 'clear', 'unknown') else 'unknown'
        input_file = Path(capture) / perception['input_file']
        checks.update(
            rest_visual_answer_parsed=perception['status'] == status,
            rest_visual_original_frame=perception['context_index'] == request['context']['index'],
            rest_visual_input_recorded=input_file.is_file() and hashlib.sha256(input_file.read_bytes()).hexdigest() == perception['input_sha256'],
            rest_visual_cost_counted=(perception['seconds'] >= 0 and request['inference_seconds'] >=
                                     request.get('native_prediction_inference_seconds', math.inf) + perception['seconds']))
    checks['rest_eligibility_reconstructed'] = request.get('rest_interval_eligible') is (candidate and status in ('moving', 'clear'))
    return checks


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


def async_selection_checks(request, capture, options):
    """Independently verify selection once per worker request, not per control tick."""
    selection=request.get('selection', {})
    kind=selection.get('kind')
    if kind not in ('single_native_stop_scene','bounded_native_speed_preference'):
        return {'async_selection_present': options.get('speed_selection_attempts',1)==1}
    read=lambda label:json.loads((Path(capture)/(label+'.json')).read_text())
    chosen=read(selection['selected_proposal'])
    original_context=request.get('retention',{}).get('source_context',request['context'])
    checks=dict(async_selected_native_source=(chosen['context']==original_context and
        chosen['trajectory_ego']==request['trajectory_ego']))
    if kind=='single_native_stop_scene':
        checks['async_stop_one_native_sample']=(options.get('scene_policy')=='red_light_stop' and
            selection['attempts']==selection['budget']==1 and 'retention' not in request)
        checks['async_stop_all_cost_counted']=abs(chosen['inference_seconds']-request['inference_seconds'])<1e-5
        return checks
    proposals=selection['proposals']
    checks['async_native_selection_budget']=(len(proposals)==selection['attempts']<=selection['budget']
        and selection['budget']==options['speed_selection_attempts'])
    low,high=selection['preference_mps']
    checks['async_declared_speed_preference']=[low,high]==options.get('speed_preference_mps',[3.,5.])
    ranks=[]
    total=0.
    for p in proposals:
        raw=read(p['label'])
        pts=np.asarray(raw['trajectory_ego'])
        speed=float(planned_speeds(pts)[10:15].mean())
        valid=(pts.shape==(64,3) and np.isfinite(pts).all() and math.isfinite(speed)
               and raw['trajectory_marker_present'] and not raw['reverse_velocity_steps'])
        effective=min(speed,options['controller_speed_cap_mps']) if options.get('controller_speed_cap_mps') else speed
        ranks.append((max(low-effective,effective-high,0.),abs(effective-(low+high)/2)) if valid else (math.inf,math.inf))
        checks['async_proposal_input_'+p['label']]=raw['context']==request['context']
        checks['async_proposal_score_'+p['label']]=abs(speed-p['target_mps'])<1e-6
        total+=raw['inference_seconds']
    visual_stop=request['visual_signal'] in ('red','yellow')
    expected=0 if visual_stop else min(range(len(ranks)),key=lambda i:ranks[i])
    if 'retention' not in request:
        checks['async_native_selection_ranking']=proposals[expected]['label']==selection['selected_proposal']
    else:
        age=request['context']['time_s']-original_context['time_s']
        checks['async_worker_retention_age']=0<age<=options.get('async_retention_age_s',0)+1e-6
        checks['async_worker_retention_stop_preserved']=not visual_stop
        checks['async_worker_retained_forward_native']=(chosen['trajectory_marker_present'] and not chosen['reverse_velocity_steps'])
    checks['async_stop_preserved']=(selection['visual_stop_preserved']==visual_stop and
                                   (not visual_stop or len(proposals)==1))
    checks['async_all_samples_counted']=abs(total-request['inference_seconds'])<1e-5
    checks.update(sampling_budget_checks(selection,options))
    return checks


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
            if options.get('traffic', {}).get('controller_policy') == 'native_speed_no_launch_boost':
                # Independently replay the declared traffic speed policy. Keep
                # all command, applied-control, native-plan and age checks.
                self._launching = False
                lead = options['traffic'].get('speed_preview_lead_s', cfg.PID_SPEED_LEAD_SEC)
                begin = int(elapsed_sec / .1 + 1e-6) + round(lead * 10)
                window = list(speeds_mps[begin:begin + max(1, round(cfg.PID_SPEED_PREVIEW_SEC / .1))])
                value = min(sum(window) / len(window) if window else 0., cfg.PID_TARGET_SPEED_MAX_KMH / 3.6)
                launch = False
            else:
                value, launch = super()._target_speed(speeds_mps, speed_mps, elapsed_sec)
            return (min(value, speed_cap) if speed_cap is not None else value), launch
    shadow = SpeedCeilingReference(vehicle)
    hold_enabled = options.get('hold_visual_stop', False)
    hold_check_index, previous_stop = -1, False
    mismatches, distances, lanes = [], [], []
    cross_navigation_retention_uses=0
    for i, (record, saved) in enumerate(zip(records, controls)):
        def check(ok, label):
            if not ok:
                mismatches.append({'index': i, 'check': label})
        context = record['context']
        retention=record.get('retention')
        async_execution=record.get('async_execution')
        async_retention=record.get('async_retention')
        async_plan_validation=record.get('async_plan_validation')
        source_context=(async_execution['source_context'] if async_execution else
                        retention['source_context'] if retention else context)
        if async_execution:
            chosen=json.loads((capture/(async_execution['source_label']+'.json')).read_text())
            check(options.get('async_inference') is True, 'async_explicitly_enabled')
            check(chosen['context']==source_context and chosen['trajectory_ego']==record['trajectory_ego'],
                  'async_original_input_and_trajectory')
            check(source_context['index']<=context['index'], 'async_source_not_future')
            check(abs(async_execution['age_s']-(context['time_s']-source_context['time_s']))<1e-6,
                  'async_original_timestamp')
            latest_label=async_execution.get('latest_request_label',async_execution['source_label'])
            latest=json.loads((capture/(latest_label+'.json')).read_text())
            check(all(record.get(k)==latest.get(k) for k in ('visual_signal','visual_signal_answer','visual_signal_check_index')),
                  'async_latest_visual_signal')
            fresh_interval=options.get('async_fresh_planning_interval_s')
            if fresh_interval is not None:
                check(not async_execution['pending_inference'],'fresh_planning_no_pending_control')
                decision_age=context['time_s']-latest['context']['time_s']
                commitment=options.get('async_motion_commitment_s')
                if commitment is not None:
                    check(math.isfinite(commitment) and .1<=commitment<=.5
                          and fresh_interval+commitment<=3.+1e-8
                          and abs(commitment*10-round(commitment*10))<1e-6
                          and options.get('async_rest_planning_interval_s') is None,
                          'native_motion_commitment_config')
                    check(abs(latest['context']['measured_speed_mps']-
                              rows[latest['context']['index']]['speed'])<1e-5,
                          'native_motion_original_speed')
                allowed_interval=motion_interval_reference(latest,fresh_interval,commitment)
                rest_interval=options.get('async_rest_planning_interval_s')
                if rest_interval is not None:
                    source_index=latest['context']['index']
                    check(abs(latest['context']['measured_speed_mps']-rows[source_index]['speed'])<1e-5,
                          'rest_schedule_original_speed')
                    perception=latest.get('rest_motion_check', {})
                    answer=perception.get('answer','').strip().lower().rstrip('.').strip()
                    if rest_schedule_reference(latest,rest_interval) and answer in ('moving','clear'):
                        allowed_interval=rest_interval
                check(0<=decision_age<allowed_interval-1e-8,'fresh_planning_interval')
                fine_interval=options.get('async_fine_planning_interval_s')
                deceleration_interval=options.get('async_deceleration_planning_interval_s')
                if deceleration_interval is not None:
                    check(not isinstance(deceleration_interval,bool) and math.isfinite(deceleration_interval)
                          and .1<=deceleration_interval<=fresh_interval
                          and abs(deceleration_interval*10-round(deceleration_interval*10))<1e-6
                          and fine_interval is not None and fine_interval<=deceleration_interval
                          and options.get('async_fine_planning_signal_only') is True,
                          'separate_deceleration_schedule_config')
                if fine_interval is not None:
                    check(abs(context['measured_speed_mps']-rows[i]['speed'])<1e-5,'fine_schedule_recorded_speed')
                    check(abs(context['time_s']-rows[i]['time_s'])<1e-6,'fine_schedule_recorded_time')
                if fine_interval is not None and i>0:
                    before=records[i-1]
                    dt=context['time_s']-before['context']['time_s']
                    deceleration=(before['context']['measured_speed_mps']-context['measured_speed_mps'])/dt
                    fine_due=(context['measured_speed_mps']>.05 and
                              (before['visual_signal'] in ('red','yellow') or
                               (not options.get('async_fine_planning_signal_only',False) and deceleration>=.5)))
                    if fine_due:
                        check(0<=decision_age<fine_interval-1e-8,'fresh_planning_fine_interval')
                    if (deceleration_interval is not None and context['measured_speed_mps']>.05
                            and deceleration>=.5):
                        check(0<=decision_age<deceleration_interval-1e-8,
                              'fresh_planning_deceleration_interval')
                if options.get('async_max_pending_heading_deg') is not None:
                    heading_change=abs((context['world_pose']['yaw']-latest['context']['world_pose']['yaw']+180.)%360.-180.)
                    check(heading_change<options['async_max_pending_heading_deg']-1e-8,'fresh_planning_heading_boundary')
            heading_limit=options.get('async_max_pending_heading_deg')
            if heading_limit is not None and async_execution['pending_inference']:
                pending_index=async_execution['pending_frame']
                valid_index=isinstance(pending_index,int) and 0<=pending_index<=i
                check(valid_index,'async_pending_frame_valid')
                if valid_index:
                    pending_yaw=records[pending_index]['context']['world_pose']['yaw']
                    change=abs((context['world_pose']['yaw']-pending_yaw+180.)%360.-180.)
                    check(change<heading_limit-1e-8,'async_wait_before_further_heading_change')
            if (options.get('async_revalidate_delayed_plans') and options.get('scene_policy')=='empty_green'
                    and record['visual_signal'] not in ('red','yellow') and async_execution['age_s']>1e-8):
                check(bool(async_retention or async_plan_validation),'async_delayed_plan_admitted')
            if async_plan_validation:
                check(options.get('async_revalidate_delayed_plans') is True,'async_delayed_validation_enabled')
                check(async_plan_validation.get('kind')=='delayed_native','async_delayed_validation_kind')
            if async_retention or async_plan_validation:
                cross_navigation_retention_uses+=int(chosen.get('options',{}).get('async_navigation_phase')!=record.get('navigation_phase'))
                check(0<async_execution['age_s']<=options.get('async_retention_age_s',0)+1e-6,'async_retention_age_bound')
                check(options.get('scene_policy')=='empty_green' and record['visual_signal'] not in ('red','yellow'),
                      'async_retention_empty_scene_no_visual_stop')
                check(chosen['trajectory_marker_present'] and not chosen['reverse_velocity_steps'],'async_retention_native_forward')
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
        if async_execution:
            check(np.allclose([context['world_pose'][k] for k in ('x','y','yaw')],
                              [rows[i][k] for k in ('x','y','yaw')], atol=1e-5, rtol=0),
                  'async_snapshot_matches_recording')
            expired=saved['elapsed_sec']>cfg.TRAJECTORY_MAX_AGE_SEC
            check(bool(saved['debug'].get('async_expired_plan'))==expired, 'async_expiry_rule')
            if expired:
                check(saved['control']==[0.,0.,1.], 'async_expired_plan_brakes')
                check(np.allclose(saved['control'], [rows[i+1][k] for k in ('steer','throttle','brake')],
                                  atol=1e-6,rtol=0), 'async_expiry_control_applied')
                continue
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
        if async_retention or async_plan_validation:
            high=options.get('speed_preference_mps',[3.,5.])[1]
            check(1.-1e-6<=debug['target_speed_kmh']/3.6<=high+1e-6,'async_retained_moving_target')
            axle=tf.transform(carla.Location(x=cfg.EGO_REAR_AXLE_X))
            offset=np.asarray(projected[:2])-[axle.x,axle.y]
            yaw=math.radians(tf.rotation.yaw)
            check(float(np.dot(offset,[math.cos(yaw),math.sin(yaw)]))>1.,'async_retained_target_ahead')
            route=routes['routes'][selection['route']]
            _,arc,_,_,_=audit_turn.project(np.array([[axle.x,axle.y],projected[:2]]),route)
            check(arc[1]-arc[0]>.25,'async_retained_route_progress')
            bounds=context['vehicle_bounds']
            box=carla.BoundingBox(carla.Location(**bounds['location']),carla.Vector3D(**bounds['extent']))
            box.rotation=carla.Rotation(**bounds['rotation'])
            vertices=box.get_world_vertices(tf)
            distance,_,_,width,_=audit_turn.project(np.array([[v.x,v.y] for v in vertices]),route)
            check(np.all(distance<=width),'async_retained_current_body_inside')
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
    async_requests = [json.loads(p.read_text()) for p in sorted(capture.glob('async_request_*.json'))]
    for request in async_requests:
        for label,ok in {**async_selection_checks(request,capture,options),
                         **rest_schedule_checks(request,capture,options)}.items():
            if not ok:
                mismatches.append(dict(index=request['context']['index'],check=label))
    if options.get('async_max_pending_sim_s') is not None or options.get('async_fresh_planning_interval_s') is not None:
        limit=options.get('async_max_pending_sim_s')
        requests_by_label={r['label']: r for r in async_requests}
        events=[json.loads(line) for line in (capture/'async-events.jsonl').read_text().splitlines()]
        for event in events:
            if event['event'] in ('consumed','discarded_navigation_revision'):
                source=requests_by_label[event['source']]['context']['index']
                if options.get('async_fresh_planning_interval_s') is not None and event['frame']!=source:
                    mismatches.append(dict(index=event['frame'],check='fresh_planning_current_state_result'))
                if limit is not None and (event['frame']-source)*.1 > limit+1e-6:
                    mismatches.append(dict(index=event['frame'],check='async_pending_simulation_advance_bound'))
    executed_calls = sum(r.get('selection',{}).get('attempts',1) for r in (async_requests or records))
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
        inference_seconds=sum(r['inference_seconds'] for r in (all_proposals or async_requests or records)),
        inference_calls=len(all_proposals or async_requests or records),
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
    if options.get('async_inference'):
        result.update(async_inference=True,
            inference_requests=len(async_requests),
            rest_visual_queries=sum('rest_motion_check' in r for r in async_requests),
            rest_visual_seconds=sum(r.get('rest_motion_check',{}).get('seconds',0.) for r in async_requests),
            async_retention_uses=sum('async_retention' in r for r in records),
            async_cross_navigation_retention_uses=cross_navigation_retention_uses,
            async_delayed_plan_validations=sum('async_plan_validation' in r for r in records),
            async_revalidate_delayed_plans=options.get('async_revalidate_delayed_plans',False),
            async_fresh_planning_interval_s=options.get('async_fresh_planning_interval_s'),
            async_motion_commitment_s=options.get('async_motion_commitment_s'),
            native_motion_commitment_steps=(sum(e['event']=='native_motion_commitment' for e in events)
                if options.get('async_fresh_planning_interval_s') is not None else 0),
            async_rest_planning_interval_s=options.get('async_rest_planning_interval_s'),
            async_fine_planning_interval_s=options.get('async_fine_planning_interval_s'),
            async_fine_planning_signal_only=options.get('async_fine_planning_signal_only',False),
            async_deceleration_planning_interval_s=options.get('async_deceleration_planning_interval_s'),
            async_max_pending_sim_s=options.get('async_max_pending_sim_s'),
            async_max_pending_heading_deg=options.get('async_max_pending_heading_deg'),
            max_executed_plan_age_s=max(c['elapsed_sec'] for c in controls),
            expired_plan_braking_steps=sum(bool(c['debug'].get('async_expired_plan')) for c in controls),
            async_summary=json.loads((capture/'async-summary.json').read_text()),
            timing_note='Inference cost/count includes every completed worker request, including unconsumed shutdown results. Capture wall time includes initial synchronous bootstrap; worker drain is reported by the supervisor.')
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
