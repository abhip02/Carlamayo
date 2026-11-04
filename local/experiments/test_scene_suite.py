import json
from types import SimpleNamespace
import carla
import numpy as np
import pytest
from local.experiments.scene_routes import lane_path, scene_scope
from local.experiments.projected_probe import route_state, SingleSampleProbe
from local.experiments.projected_speed_selection import SpeedSelectedProbe
from local.experiments.stop_mission import audit_stop
from local.experiments.test_projected_speed_selection import candidate


def test_right_instruction_and_finish_phase_across_wrapped_yaw():
    p=dict(x=0.,y=0.,z=0.,yaw=89.6,lane_width=3.5,road_id=1,lane_id=-1,s=0.,is_junction=True)
    route=dict(entry={**p,'yaw':-270.4},exit={**p,'x':-20.,'y':10.,'yaw':180.},exit_index=2,
               points=[p,{**p,'y':10.},{**p,'x':-20.,'y':10.}])
    _,phase,instruction=route_state({'world_pose':dict(x=0.,y=2.,yaw=89.6)},route,'right','approach')
    assert phase=='approach' and instruction=='Turn right at the intersection.'
    _,phase,instruction=route_state({'world_pose':dict(x=-5.,y=10.,yaw=-185.)},route,'right',phase)
    assert phase=='finish_turn' and instruction.startswith('Finish the right turn')


def test_curved_route_keeps_declared_instruction_until_explicit_exit_phase():
    p=dict(x=0.,y=0.,z=0.,yaw=0.,lane_width=3.5,road_id=1,lane_id=-1,s=0.,is_junction=False)
    route=dict(entry=p,exit={**p,'x':20.},exit_index=1,points=[p,{**p,'x':20.}],
               instruction='Take the second roundabout exit.',exit_instruction='Follow the exit lane.',finish_after_arc_m=15.)
    context={'world_pose':dict(x=10.,y=0.,yaw=0.)}
    assert route_state(context,route,'round_second','approach')[2]==route['instruction']
    context['world_pose']['x']=16.
    assert route_state(context,route,'round_second','approach')[1:]==('exit',route['exit_instruction'])


class Node:
    lane_type=carla.LaneType.Driving
    def __init__(self,road,s,x,lane=-1):
        self.road_id,self.s,self.lane_id=road,s,lane
        self.transform=carla.Transform(carla.Location(x=x))
        self.following=[]
    def next(self,step):return self.following


def test_lane_route_takes_connected_branch_and_rejects_discontinuous_jump():
    a,b,dead,end=Node(1,0,0),Node(2,0,.5),Node(3,0,.5),Node(2,.5,1.)
    a.following=[dead,b];b.following=[end]
    assert lane_path(a,end)==[a,b,end]
    b.transform.location.x=20
    with pytest.raises(ValueError,match='No forward lane path'):lane_path(a,end)


def test_positive_lane_goal_uses_decreasing_road_coordinate():
    a,b=Node(1,1,0,1),Node(1,.5,.5,1)
    assert lane_path(a,b)==[a,b]


def stop_fixture():
    rows=[dict(time_s=i*.1,x=7.,y=0.,z=0.,yaw=0.,pitch=0.,roll=0.,speed=0.) for i in range(31)]
    bounds=dict(location=dict(x=0.,y=0.,z=0.),extent=dict(x=2.,y=1.,z=.5),rotation=dict(yaw=0.,pitch=0.,roll=0.))
    mission=dict(stop_line=dict(x=10.,y=0.,yaw=0.),hold_s=2.,maximum_stop_gap_m=8.)
    return rows,bounds,mission


def test_red_stop_requires_full_hold_and_front_bumper_before_line():
    rows,bounds,mission=stop_fixture()
    assert audit_stop(rows,bounds,mission)['passed']
    rows[0]['x']=9.
    assert not audit_stop(rows,bounds,mission)['passed']
    rows[0]['x']=7.;rows[15]['speed']=.3
    assert not audit_stop(rows,bounds,mission)['passed']


def test_red_stop_does_not_pass_a_brief_or_far_away_stop():
    rows,bounds,mission=stop_fixture()
    assert not audit_stop(rows[:10],bounds,mission)['passed']
    for row in rows:row['x']=-3.
    assert not audit_stop(rows,bounds,mission)['passed']


def test_red_scene_never_resamples_or_reuses_even_if_visual_answer_is_unknown(tmp_path,monkeypatch):
    probe=object.__new__(SpeedSelectedProbe);probe.output=tmp_path;calls=[];selected=[]
    def sample(self,adapter,data,context,instruction,options,label,*args):
        record={**candidate(.1,'unknown'),'label':label,'context':context}
        (tmp_path/(label+'.json')).write_text(json.dumps(record));calls.append(label)
        return record['trajectory_ego'],{}
    monkeypatch.setattr(SingleSampleProbe,'sample_plan',sample)
    monkeypatch.setattr(probe,'consider_retained',lambda *a:pytest.fail('Stop scene used cache'))
    monkeypatch.setattr(probe,'selected',lambda r:selected.append(r))
    probe.sample_plan(None,None,dict(index=0,controlled_scene=dict(verified=True,policy='red_light_stop')),
                      'Obey traffic lights.',dict(speed_selection_attempts=6,scene_policy='red_light_stop'),'frame_000')
    assert len(calls)==len(selected)==1
    assert selected[0]['selection']['kind']=='single_native_stop_scene'
    assert selected[0]['trajectory_ego']==candidate(.1,'unknown')['trajectory_ego']


def test_unverified_empty_road_cannot_enable_motion_selection(tmp_path):
    probe=object.__new__(SpeedSelectedProbe);probe.output=tmp_path
    with pytest.raises(ValueError,match='all-green'):
        probe.sample_plan(None,None,dict(index=0,signal_states={}), 'Go',dict(speed_selection_attempts=6),'frame_000')


class Actors(list):
    def filter(self,pattern):
        if pattern=='vehicle.*':return [a for a in self if a.type_id=='vehicle.test']
        if pattern=='walker.pedestrian.*':return [a for a in self if a.type_id=='walker.pedestrian.test']
        return [a for a in self if a.type_id=='traffic.traffic_light']


def test_scope_checks_live_traffic_and_all_map_lights():
    ego=SimpleNamespace(id=1,type_id='vehicle.test',attributes={'role_name':'carlamayo_intersection'})
    actors=Actors([ego]);world=SimpleNamespace(get_actors=lambda:actors)
    assert scene_scope(world,'empty_green')['map_traffic_light_count']==0
    with pytest.raises(RuntimeError):scene_scope(world,'red_light_stop')
    light=SimpleNamespace(id=2,type_id='traffic.traffic_light',get_state=lambda:'Red')
    actors.append(light)
    assert scene_scope(world,'red_light_stop')['verified']
    with pytest.raises(RuntimeError):scene_scope(world,'empty_green')
    actors.append(SimpleNamespace(id=3,type_id='vehicle.test',attributes={'role_name':'npc'}))
    with pytest.raises(RuntimeError):scene_scope(world,'red_light_stop')
