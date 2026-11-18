"""Check physical clearance and stopping requirements independently of live inference."""
import pytest
import copy
import json
from pathlib import Path

from local.experiments.audit_traffic import box_clearance, mission_checks
from local.experiments.traffic_probe import point_ahead


def box(x, y=0.):
    return [[x - 2., y - 1.], [x + 2., y - 1.], [x + 2., y + 1.], [x - 2., y + 1.]]


def test_clearance_detects_overlap_touching_and_separation():
    assert box_clearance(box(0), box(3)) == 0.
    assert box_clearance(box(0), box(4)) == 0.
    assert box_clearance(box(0), box(7)) == pytest.approx(3.)
    assert box_clearance(box(0), box(0, 4)) == pytest.approx(2.)


def stop_fixture():
    spec = dict(kind='stationary_lead', duration_s=12.)
    states = [dict(time_s=i / 10, ego=dict(speed_mps=0., vertices_xy=box(0)),
                   traffic=[dict(speed_mps=0., vertices_xy=box(7))]) for i in range(121)]
    geometry = dict(integrity_pass=True, termination='duration_reached', body_departure_times_s=[],
                    max_speed_mps=3., inference_calls=120, steps=120, cache_uses=0)
    return spec, states, geometry


def test_stopped_vehicle_requires_full_hold_and_close_enough_approach():
    spec, states, geometry = stop_fixture()
    checks, _ = mission_checks(spec, states, geometry)
    assert all(checks.values())
    states[-11]['ego']['speed_mps'] = .4
    assert not mission_checks(spec, states, geometry)[0]['stopped_for_final_two_seconds']
    states[-11]['ego']['speed_mps'] = 0.
    states[-1]['traffic'][0]['vertices_xy'] = box(20)
    assert not mission_checks(spec, states, geometry)[0]['stopped_within_eight_metres']


def test_collision_and_incomplete_capture_cannot_pass():
    spec, states, geometry = stop_fixture()
    geometry['termination'] = 'collision'
    assert not all(mission_checks(spec, states, geometry)[0].values())
    geometry['termination'] = 'duration_reached'
    assert not all(mission_checks(spec, states[:-1], geometry)[0].values())


def test_spawn_follows_connected_route_and_rejects_extrapolation():
    route = dict(points=[dict(x=0., y=0., z=0., lane_width=3.5, yaw=0.),
                         dict(x=10., y=0., z=0., lane_width=3.5, yaw=0.),
                         dict(x=10., y=10., z=0., lane_width=3.5, yaw=90.)], exit_index=2)
    pose, _ = point_ahead(route, dict(x=2., y=0.), 12.)
    assert pose.location.x == pytest.approx(10.)
    assert pose.location.y == pytest.approx(4.)
    assert pose.rotation.yaw == pytest.approx(90.)
    with pytest.raises(ValueError):
        point_ahead(route, dict(x=2., y=0.), 30.)


def async_stop_fixture():
    spec, states, geometry = stop_fixture()
    spec.update(audit_version=2, signal_state='Green')
    for row in states:
        row.update(scope_verified=True, signal_state='Green')
    geometry.update(async_inference=True, inference_requests=24, inference_calls=24,
                    async_retention_uses=0, async_fresh_planning_interval_s=.5,
                    max_executed_plan_age_s=.4, expired_plan_braking_steps=0)
    return spec, states, geometry


def test_async_traffic_counts_native_requests_not_control_ticks():
    spec, states, geometry = async_stop_fixture()
    assert all(mission_checks(spec, states, geometry)[0].values())
    geometry['inference_calls'] = 25
    assert not mission_checks(spec, states, geometry)[0]['one_call_per_decision']


def test_motion_commitment_keeps_bounded_age_and_requires_independent_integrity():
    spec, states, geometry = async_stop_fixture()
    geometry.update(async_fresh_planning_interval_s=1., async_motion_commitment_s=.5,
                    max_executed_plan_age_s=1.3)
    assert all(mission_checks(spec, states, geometry)[0].values())
    geometry['integrity_pass'] = False
    assert not all(mission_checks(spec, states, geometry)[0].values())
    geometry['integrity_pass'] = True
    geometry['max_executed_plan_age_s'] = 1.6
    assert not mission_checks(spec, states, geometry)[0]['bounded_fresh_planning']


@pytest.mark.parametrize('field,value', [('async_retention_uses',1),
    ('max_executed_plan_age_s',.6), ('expired_plan_braking_steps',1)])
def test_traffic_rejects_motion_history_and_excessive_age(field,value):
    spec, states, geometry = async_stop_fixture()
    geometry[field] = value
    assert not all(mission_checks(spec, states, geometry)[0].values())


def test_missing_traffic_or_changed_signal_cannot_pass():
    spec, states, geometry = async_stop_fixture()
    states[12]['signal_state'] = 'Red'
    assert not mission_checks(spec, states, geometry)[0]['declared_signals_every_tick']
    states[12]['signal_state'] = 'Green'
    states[-1]['traffic'] = []
    assert not mission_checks(spec, states, geometry)[0]['declared_actor_count']


def test_red_queue_also_requires_original_stop_line_mission():
    spec, states, geometry = async_stop_fixture()
    spec['signal_state'] = 'Red'
    for row in states: row['signal_state'] = 'Red'
    geometry['mission'] = dict(passed=False)
    assert not all(mission_checks(spec, states, geometry)[0].values())
    geometry['mission']['passed'] = True
    assert all(mission_checks(spec, states, geometry)[0].values())


def test_moving_traffic_requires_exposure_and_progress_without_unnecessary_stop():
    spec, states, geometry = async_stop_fixture()
    spec['kind'] = 'moving_lead'
    geometry['reached_exit_time_s'] = 12.
    for row in states:
        row['ego']['speed_mps'] = 2.
        row['traffic'][0].update(speed_mps=3., vertices_xy=box(12))
    assert all(mission_checks(spec, states, geometry)[0].values())
    stopped = copy.deepcopy(states)
    for row in stopped[20:50]: row['ego']['speed_mps'] = 0.
    assert not mission_checks(spec, stopped, geometry)[0]['no_prolonged_unnecessary_stop']
    for row in states: row['traffic'][0]['vertices_xy'] = box(100)
    assert not mission_checks(spec, states, geometry)[0]['traffic_proximity_exposure']


def test_traffic_manifest_retains_original_ego_cases():
    folder=Path(__file__).parent/'presets'
    original=json.loads((folder/'ten-driving-scenes.json').read_text())
    traffic=json.loads((folder/'ten-driving-traffic-scenes.json').read_text())
    assert len(traffic['scenarios']) == len(original['scenarios']) == 10
    for old,new in zip(original['scenarios'],traffic['scenarios'],strict=True):
        assert all(new[key] == value for key,value in old.items() if key != 'scene_policy')
        assert 'scene_policy' not in new
        assert new['traffic']['duration_s'] == old['duration_s']
        assert new['traffic']['audit_version'] == 2


def test_manager_shutdown_precedes_actor_destruction_and_is_idempotent(tmp_path):
    import io
    from types import SimpleNamespace
    from local.experiments.traffic_probe import TrafficScene
    events=[]
    class Vehicle:
        is_alive=True
        def set_autopilot(self,enabled,port):
            assert enabled is False and port==8013
            events.append('deregister')
        def destroy(self):
            events.append('destroy');self.is_alive=False
            return True
    scene=TrafficScene(dict(kind='moving_lead'),tmp_path)
    scene.vehicle=Vehicle()
    scene.manager=SimpleNamespace(shut_down=lambda:events.append('shutdown'))
    scene.close();scene.close()
    assert events==['deregister','shutdown','destroy']
    assert json.loads((tmp_path/'traffic-cleanup.json').read_text())['owned_vehicle_removed']


def red_queue_fixture():
    spec,states,geometry=async_stop_fixture()
    spec.update(signal_state='Red',queue_stop_line=dict(x=11.,y=0.,yaw=0.))
    for row in states:row['signal_state']='Red'
    geometry['mission']=dict(passed=False,held_stop=True,stopped_near_line=False)
    return spec,states,geometry


def test_red_queue_uses_following_gap_instead_of_empty_road_line_proximity():
    spec,states,geometry=red_queue_fixture()
    # Ego front is 9m before the line but only 3m behind the queued car.
    assert all(mission_checks(spec,states,geometry)[0].values())
    assert geometry['mission']['passed'] is False


@pytest.mark.parametrize('actor,key', [('ego','ego_stays_before_red_line'),
                                      ('lead','queued_lead_stays_before_red_line')])
def test_red_queue_rejects_stop_line_crossing_by_either_vehicle(actor,key):
    spec,states,geometry=red_queue_fixture()
    target=states[30]['ego'] if actor=='ego' else states[30]['traffic'][0]
    target['vertices_xy']=box(11.)
    assert not mission_checks(spec,states,geometry)[0][key]


def test_red_queue_rejects_a_rolling_lead_during_hold():
    spec,states,geometry=red_queue_fixture()
    states[-1]['traffic'][0]['speed_mps']=.3
    assert not mission_checks(spec,states,geometry)[0]['queued_lead_stopped_during_hold']


def traffic_follower():
    from types import SimpleNamespace
    from local.experiments.traffic_probe import NativeSpeedTrafficFollower
    vehicle = SimpleNamespace(get_physics_control=lambda: SimpleNamespace(
        wheels=[SimpleNamespace(max_steer_angle=70.)]))
    return NativeSpeedTrafficFollower(vehicle, mode='native', speed_cap_mps=4.5)


def test_traffic_stop_overrides_previous_launch_even_with_a_long_future_path():
    import carla
    import numpy as np
    from module import config as cfg
    follower = traffic_follower()
    follower._launching = True
    speeds = np.r_[np.full(15, .1), np.full(49, 3.)]
    world = np.column_stack((np.cumsum(speeds) * .1, np.zeros(64), np.zeros(64)))
    pose = carla.Transform(carla.Location(x=-cfg.EGO_REAR_AXLE_X))
    _, throttle, brake, debug = follower.compute_control(pose, world, 1.88, speeds)
    assert debug['planned_stop'] and not debug['launch']
    assert throttle == 0. and brake > 0.
    assert debug['traffic_launch_boost_disabled']


def test_backward_tail_cannot_launch_a_stopped_traffic_vehicle():
    import carla
    import numpy as np
    from module import config as cfg
    from module.trajectory_follower import planned_speeds
    follower = traffic_follower()
    world = np.column_stack((-np.cumsum(np.r_[np.full(15, .01), np.full(49, .1)]),
                             np.zeros(64), np.zeros(64)))
    speeds = planned_speeds(world)
    pose = carla.Transform(carla.Location(x=-cfg.EGO_REAR_AXLE_X))
    _, throttle, brake, debug = follower.compute_control(pose, world, 0., speeds)
    assert not debug['launch'] and debug['planned_stop']
    assert throttle == 0. and brake == 1.


def test_native_forward_start_and_speed_ceiling_remain_available_in_traffic():
    import carla
    import numpy as np
    from module import config as cfg
    follower = traffic_follower()
    world = np.column_stack((np.arange(1,65)*.3, np.zeros(64), np.zeros(64)))
    pose = carla.Transform(carla.Location(x=-cfg.EGO_REAR_AXLE_X))
    _, throttle, brake, debug = follower.compute_control(pose, world, 0., np.full(64,3.))
    assert throttle > 0. and brake == 0. and not debug['launch']
    assert debug['target_speed_kmh'] == pytest.approx(10.8)
    _, _, _, debug = follower.compute_control(pose, world, 3., np.full(64,8.))
    assert debug['target_speed_kmh'] == pytest.approx(4.5*3.6)


def test_traffic_native_preview_respects_plan_age_and_red_hold():
    import numpy as np
    follower = traffic_follower()
    speeds = np.r_[np.full(15,3.), np.zeros(49)]
    assert follower._target_speed(speeds,0.,0.) == (3.,False)
    assert follower._target_speed(speeds,0.,.5) == (0.,False)
    follower._visual_stop_held = True
    assert follower._target_speed(np.full(64,3.),0.,0.) == (0.,False)


def test_shorter_traffic_preview_preserves_initial_braking_before_later_acceleration():
    import carla
    import numpy as np
    from module import config as cfg
    old, candidate = traffic_follower(), traffic_follower()
    candidate.traffic_speed_preview_lead_s = .5
    speeds = np.r_[np.full(5,4.),np.full(5,2.),np.full(54,5.)]
    world = np.column_stack((np.cumsum(speeds)*.1,np.zeros(64),np.zeros(64)))
    pose = carla.Transform(carla.Location(x=-cfg.EGO_REAR_AXLE_X))
    _, old_throttle, old_brake, _ = old.compute_control(pose,world,4.,speeds)
    _, throttle, brake, debug = candidate.compute_control(pose,world,4.,speeds)
    assert old_throttle > 0 and old_brake == 0
    assert throttle == 0 and brake > 0
    assert debug['target_speed_kmh'] == pytest.approx(2.*3.6)
    assert debug['traffic_speed_preview_lead_s'] == .5 and not debug['launch']
    assert cfg.PID_SPEED_LEAD_SEC == 1.


def test_shorter_traffic_preview_keeps_original_plan_age_and_stop_hold():
    import numpy as np
    follower=traffic_follower();follower.traffic_speed_preview_lead_s=.5
    speeds=np.r_[np.full(5,3.),np.zeros(5),np.full(54,2.)]
    assert follower._target_speed(speeds,0.,0.) == (0.,False)
    assert follower._target_speed(speeds,0.,.5) == (2.,False)
    follower._visual_stop_held=True
    assert follower._target_speed(speeds,0.,.5) == (0.,False)


@pytest.mark.parametrize('lead',[float('nan'),float('inf'),-.1,1.1,.55])
def test_traffic_preview_rejects_invalid_intervals(lead):
    from local.experiments.run_async_panel import traffic_speed_preview
    manifest={'traffic_protocol':{'speed_preview_lead_s':lead},'scenarios':[{'traffic':{'speed_preview_lead_s':lead}}]}
    with pytest.raises(ValueError,match='multiple'):
        traffic_speed_preview(manifest)


def test_traffic_preview_is_uniform_and_legacy_default_is_preserved():
    from local.experiments.run_async_panel import traffic_speed_preview
    assert traffic_speed_preview({'scenarios':[{'traffic':{}}]}) == 1.
    manifest={'traffic_protocol':{'speed_preview_lead_s':.5},'scenarios':[{'traffic':{'speed_preview_lead_s':.5}}]}
    assert traffic_speed_preview(manifest) == .5
    manifest['scenarios'].append({'traffic':{}})
    with pytest.raises(ValueError,match='Every traffic case'):
        traffic_speed_preview(manifest)


@pytest.mark.parametrize('value',[float('nan'),float('inf'),-.1,1.1,True,'0.5'])
def test_traffic_temperature_rejects_invalid_values(value):
    from local.experiments.run_projection_suite import native_flow_temperature
    with pytest.raises(ValueError,match='finite'):
        native_flow_temperature({'traffic':{'flow_temperature':value}})


def test_traffic_temperature_is_uniform_and_does_not_change_empty_defaults():
    from local.experiments.run_async_panel import traffic_flow_temperature
    from local.experiments.run_projection_suite import native_flow_temperature
    assert native_flow_temperature({}) == 1.
    assert native_flow_temperature({'traffic':{}}) == 1.
    manifest={'traffic_protocol':{'flow_temperature':.5},'scenarios':[{'traffic':{'flow_temperature':.5}}]}
    assert traffic_flow_temperature(manifest) == .5
    manifest['scenarios'].append({'traffic':{}})
    with pytest.raises(ValueError,match='Every traffic case'):
        traffic_flow_temperature(manifest)


def test_declared_temperature_reaches_native_sampler_without_extra_samples():
    from types import SimpleNamespace
    from local.experiments.inference_probe import Probe
    from local.experiments.run_projection_suite import native_flow_temperature
    probe=SimpleNamespace(options={'num_samples':1,'flow_temperature':native_flow_temperature({'traffic':{'flow_temperature':.5}})})
    original={'inference_step':10}
    got=Probe.sample_with_options(probe,lambda **kwargs:kwargs,diffusion_kwargs=original)
    assert got['num_traj_samples'] == 1
    assert got['diffusion_kwargs'] == {'inference_step':10,'temperature':.5}
    assert original == {'inference_step':10}


@pytest.mark.parametrize('field,value',[('flow_temperature',1.),('num_samples',2),('flow_steps',20)])
def test_sampling_audit_rejects_undeclared_temperature_or_extra_compute(field,value):
    from local.experiments.audit_traffic import declared_sampling_matches
    records=[{'options':{'flow_temperature':.5,'num_samples':1}}]
    spec={'flow_temperature':.5}
    assert declared_sampling_matches(records,spec)
    assert not declared_sampling_matches([],spec)
    records[0]['options'][field]=value
    assert not declared_sampling_matches(records,spec)


@pytest.mark.parametrize('value',[float('nan'),float('inf'),.9,4.6,True,'3.5'])
def test_declared_traffic_speed_ceiling_rejects_invalid_values(value):
    from local.experiments.run_projection_suite import declared_traffic_speed_cap
    with pytest.raises(ValueError,match='finite'):
        declared_traffic_speed_cap({'traffic':{'controller_speed_cap_mps':value}})


def test_traffic_speed_ceiling_preserves_empty_defaults_and_rejects_conflicting_override():
    from local.experiments.run_projection_suite import declared_traffic_speed_cap
    assert declared_traffic_speed_cap({}) is None
    assert declared_traffic_speed_cap({},4.5) == 4.5
    case={'traffic':{'controller_speed_cap_mps':3.5}}
    assert declared_traffic_speed_cap(case) == 3.5
    assert declared_traffic_speed_cap(case,3.5) == 3.5
    with pytest.raises(ValueError,match='differs'):
        declared_traffic_speed_cap(case,4.5)


def test_traffic_speed_ceiling_must_be_uniform_across_cases():
    from local.experiments.run_async_panel import traffic_speed_cap
    assert traffic_speed_cap({'scenarios':[{'traffic':{}}]}) == 4.5
    manifest={'traffic_protocol':{'controller_speed_cap_mps':3.5},'scenarios':[{'traffic':{'controller_speed_cap_mps':3.5}}]}
    assert traffic_speed_cap(manifest) == 3.5
    manifest['scenarios'].append({'traffic':{}})
    with pytest.raises(ValueError,match='Every traffic case'):
        traffic_speed_cap(manifest)


def test_sampling_audit_rejects_missing_or_wrong_declared_traffic_ceiling():
    from local.experiments.audit_traffic import declared_sampling_matches
    spec={'flow_temperature':.5,'controller_speed_cap_mps':3.5}
    records=[{'options':{'flow_temperature':.5,'num_samples':1}}]
    assert not declared_sampling_matches(records,spec)
    records[0]['options']['controller_speed_cap_mps']=4.5
    assert not declared_sampling_matches(records,spec)
    records[0]['options']['controller_speed_cap_mps']=3.5
    assert declared_sampling_matches(records,spec)


@pytest.mark.parametrize('fault', ['wrong_lane', 'wrong_heading', 'backtracking', 'missing', 'nonfinite'])
def test_lead_route_audit_rejects_invalid_fixture_even_if_ego_would_pass(fault):
    from local.experiments.audit_traffic import lead_route_audit
    route = dict(points=[dict(x=x,y=0.,lane_width=3.5) for x in (0.,5.,10.)])
    states = [dict(time_s=i*.1,traffic=[dict(x=float(x),y=0.,yaw=0.,speed_mps=2.)])
              for i,x in enumerate((1,2,3,4))]
    if fault=='wrong_lane':states[2]['traffic'][0]['y']=3.
    if fault=='wrong_heading':states[2]['traffic'][0]['yaw']=180.
    if fault=='backtracking':states[3]['traffic'][0]['x']=1.
    if fault=='missing':states[2]['traffic']=[]
    if fault=='nonfinite':states[2]['traffic'][0]['x']=float('nan')
    result=lead_route_audit(states,route)
    assert not result['passed'] and result['failures']


def test_lead_route_audit_valid_suffix_exit_does_not_hide_prior_departure():
    from local.experiments.audit_traffic import lead_route_audit
    route = dict(points=[dict(x=x,y=0.,lane_width=3.5) for x in (0.,5.,10.)])
    states = [dict(time_s=i*.1,traffic=[dict(x=float(x),y=0.,yaw=0.,speed_mps=2.)])
              for i,x in enumerate((1,5,11,20))]
    good=lead_route_audit(states,route)
    assert good['passed'] and good['route_suffix_reached_s']==pytest.approx(.2)
    assert good['checked_ticks']==3 and good['total_ticks']==4
    states[1]['traffic'][0]['y']=3.
    assert not lead_route_audit(states,route)['passed']


def test_lead_route_audit_cannot_claim_exit_far_to_the_side():
    from local.experiments.audit_traffic import lead_route_audit
    route = dict(points=[dict(x=x,y=0.,lane_width=3.5) for x in (0.,5.,10.)])
    result=lead_route_audit([dict(time_s=0.,traffic=[dict(x=20.,y=8.,yaw=0.,speed_mps=2.)])],route)
    assert not result['passed'] and result['route_suffix_reached_s'] is None


@pytest.mark.parametrize('incoming_road,outgoing_road,kind,expected', [
    (30,8,'junction','outgoing_road_anchors'),
    (30,30,'junction','dense'),
    (30,8,'lane_path','dense')])
def test_npc_path_policy_disambiguates_only_distinct_road_single_junctions(incoming_road,outgoing_road,kind,expected):
    from local.experiments.traffic_probe import traffic_manager_path
    points=[dict(x=float(i),y=0.,z=0.,road_id=incoming_road if i<3 else (100 if i<=7 else outgoing_road),
                 lane_id=-1,is_junction=3<=i<=7) for i in range(22)]
    document=dict(route_spec=dict(kind=kind),routes=dict(turn=dict(points=points,entry_index=3,exit_index=7)))
    selected,meta=traffic_manager_path(document,'turn',4,'distinct_road_exit_anchors')
    assert meta['applied']==expected
    if expected=='dense':assert selected==points[4:]
    else:
        assert selected==[points[16],points[-1]]
        assert all(not p['is_junction'] and p['road_id']==outgoing_road for p in selected)
    # The original policy and an already-passed junction retain every point.
    assert traffic_manager_path(document,'turn',4)[0]==points[4:]
    assert traffic_manager_path(document,'turn',12,'distinct_road_exit_anchors')[0]==points[12:]


def test_npc_path_policy_rejects_unknown_policy_and_does_not_extrapolate():
    from local.experiments.traffic_probe import traffic_manager_path
    document=dict(routes=dict(turn=dict(points=[dict(x=0.,y=0.,z=0.)])))
    with pytest.raises(ValueError):traffic_manager_path(document,'turn',0,'unsupported')
    with pytest.raises(ValueError):traffic_manager_path(document,'turn',1)


@pytest.mark.parametrize('suffix', ['short', 'another_junction'])
def test_npc_exit_anchors_keep_dense_path_when_outgoing_segment_is_ambiguous(suffix):
    from local.experiments.traffic_probe import traffic_manager_path
    points=[dict(x=float(i),y=0.,z=0.,road_id=30 if i<3 else (100 if i<=7 else 76),
                 lane_id=-1,is_junction=3<=i<=7) for i in range(22)]
    if suffix=='short':points=points[:12]
    else:points[19]['is_junction']=True
    document=dict(route_spec=dict(kind='junction'),routes=dict(turn=dict(points=points,entry_index=3,exit_index=7)))
    selected,meta=traffic_manager_path(document,'turn',4,'distinct_road_exit_anchors')
    assert selected==points[4:] and meta['reason']=='insufficient_single_outgoing_lane_suffix'


def test_direct_generation_is_optional_and_uniform():
    from local.experiments.run_async_panel import traffic_generation_mode
    from local.experiments.inference_probe import trajectory_generation_options
    legacy={'verify_signal':True}
    assert trajectory_generation_options(legacy, 'red') is legacy
    assert traffic_generation_mode({'scenarios':[{'traffic':{}}]}) == 'reasoned'
    manifest={'traffic_protocol':{'trajectory_generation':'direct'},
              'scenarios':[{'traffic':{'trajectory_generation':'direct'}}]}
    assert traffic_generation_mode(manifest) == 'direct'
    manifest['scenarios'].append({'traffic':{}})
    with pytest.raises(ValueError,match='Every traffic case'):
        traffic_generation_mode(manifest)
    with pytest.raises(ValueError):
        traffic_generation_mode({'traffic_protocol':{'trajectory_generation':'invalid'},'scenarios':[]})


@pytest.mark.parametrize('signal,budget',[('unknown',1),('green',1),(None,1),('red',256),('yellow',256)])
def test_direct_generation_retains_visual_stop_budget_and_native_sampler(signal,budget):
    from types import SimpleNamespace
    from local.experiments.inference_probe import Probe, trajectory_generation_options
    original={'traffic':{'trajectory_generation':'direct'},'num_samples':1,'flow_temperature':1.}
    effective=trajectory_generation_options(original,signal)
    assert 'prompt_style' not in original and 'max_generation_length' not in original
    assert effective['prompt_style']=='trajectory_only'
    probe=SimpleNamespace(options=effective)
    sampled=Probe.sample_with_options(probe,lambda **kwargs:kwargs,
                                     max_generation_length=256,diffusion_kwargs={'inference_step':10})
    assert sampled['max_generation_length']==budget
    assert sampled['num_traj_samples']==1
    assert sampled['diffusion_kwargs']=={'inference_step':10,'temperature':1.}


@pytest.mark.parametrize('key',['prompt_style','max_generation_length','completed_reasoning',
                               'reasoning_prefix','complete_visual_stop_intent'])
def test_direct_generation_rejects_conflicting_supplied_intents_or_budgets(key):
    from local.experiments.inference_probe import trajectory_generation_options
    with pytest.raises(ValueError,match='conflicts'):
        trajectory_generation_options({'traffic':{'trajectory_generation':'direct'},key:1},'red')


@pytest.mark.parametrize('signal',['unknown','red','yellow'])
def test_direct_generation_audit_detects_effective_path_tampering(signal):
    from local.experiments.audit_traffic import declared_generation_matches
    stop=signal in ('red','yellow')
    row={'options':{'traffic':{'trajectory_generation':'direct'},'verify_signal':True},
         'trajectory_generation':'direct','effective_prompt_style':'trajectory_only',
         'effective_max_generation_length':256 if stop else 1,
         'visual_signal':signal,'trajectory_marker_present':True,'generated_tokens':12 if stop else 1,
         'assistant_prefix':f'<|cot_start|>Stop and remain stopped for the {signal} traffic light' if stop else '<|traj_future_start|>'}
    spec={'trajectory_generation':'direct'}
    assert declared_generation_matches([row],spec)
    assert not declared_generation_matches([],spec)
    for key,value in [('effective_max_generation_length',4),('assistant_prefix','Move forward'),
                      ('trajectory_marker_present',False),('trajectory_generation','reasoned')]:
        changed=copy.deepcopy(row);changed[key]=value
        assert not declared_generation_matches([changed],spec)
    changed=copy.deepcopy(row);changed['options']['traffic']['trajectory_generation']='reasoned'
    assert not declared_generation_matches([changed],spec)
    changed=copy.deepcopy(row);changed['options']['verify_signal']=False
    assert not declared_generation_matches([changed],spec)
