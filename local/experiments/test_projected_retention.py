import copy
import json
from types import SimpleNamespace

import numpy as np
import pytest

from local.experiments import projected_retention as cache
from local.experiments.test_projected_speed_selection import candidate


def scene():
    point=dict(x=0.,y=0.,z=0.,road_id=1,lane_id=-1,s=0.,is_junction=False,
               yaw=0.,lane_width=3.5)
    route={'points':[{**point,'x':-30.},{**point,'x':50.}],
           'entry':point,'exit':{**point,'x':40.},'entry_index':0,'exit_index':1}
    bounds=dict(location=dict(x=0.,y=0.,z=0.),extent=dict(x=2.4,y=.9,z=.7),rotation=dict(yaw=0.,pitch=0.,roll=0.))
    context=dict(time_s=.5,measured_speed_mps=4.,vehicle_bounds=bounds,
                 world_pose=dict(x=2.,y=0.,z=0.,yaw=0.,pitch=0.,roll=0.))
    record=candidate(4.);record.update(label='proposal_000_00',context={**context,'time_s':0.,'world_pose':{**context['world_pose'],'x':0.}},
        visual_signal_answer='Green',visual_signal_check_index=0)
    world=np.array(record['trajectory_ego']);world[:,0]-=1.3886
    plan=dict(record=record,world=world,speeds=np.full(64,4.),result=('unchanged_prediction',{}))
    options=dict(projection_cache_age_s=1.,controller_speed_cap_mps=4.5,steering_gain=1.18)
    return route,context,plan,options


def test_cache_rechecks_real_age_forward_target_and_current_footprint():
    route,context,plan,options=scene()
    assert cache.assess(plan,context,route,options)['valid']
    assert not cache.assess(plan,{**context,'time_s':1.1},route,options)['valid']
    bad=copy.deepcopy(context);bad['world_pose']['y']=2.
    assert not cache.assess(plan,bad,route,options)['valid']
    old=copy.deepcopy(plan);old['world'][:,0]-=40.
    assert not cache.assess(old,context,route,options)['valid']
    slow=copy.deepcopy(plan);slow['speeds'][:]=.1
    assert not cache.assess(slow,context,route,options)['valid']


def test_selected_old_plan_keeps_original_world_anchor_and_prediction_time():
    route,context,plan,options=scene()
    probe=object.__new__(cache.RetainedSpeedProbe)
    record=copy.deepcopy(plan['record']);record.update(context=context,retention={'source_context':plan['record']['context']})
    probe.selected(record)
    np.testing.assert_allclose(probe.execution_world,plan['world'],atol=1e-5)
    assert probe.execution_age==.5
    assert not np.allclose(probe.execution_world[:,0],np.array(record['trajectory_ego'])[:,0]+2.-1.3886)


def test_red_visual_signal_prevents_cache_substitution():
    route,context,plan,options=scene()
    probe=object.__new__(cache.RetainedSpeedProbe);probe.history=[plan];probe.route=route
    red=candidate(.1,'red');red.update(label='proposal_005_00',context=context)
    assert probe.consider_retained(red,('stop_prediction',{}),context,options) is None


def test_valid_cache_is_used_without_changing_its_native_trajectory():
    route,context,plan,options=scene()
    probe=object.__new__(cache.RetainedSpeedProbe);probe.history=[plan];probe.route=route
    new=candidate(.1);new.update(label='proposal_005_00',context=context,
        visual_signal_answer='Green',visual_signal_check_index=5)
    choice=probe.consider_retained(new,('slow_prediction',{}),context,options)
    assert choice is not None
    assert choice[1][0]=='unchanged_prediction'
    assert choice[2]['trajectory_ego']==plan['record']['trajectory_ego']
    assert choice[2]['retention']['age_s']==.5


def test_capture_restores_anchor_and_video_hooks_on_failure(tmp_path,monkeypatch):
    from local.experiments import projected_probe as probe
    runner=probe.native.runner
    original=(runner.TrajectoryFollower,runner.connect_world,probe.native.Probe,
              runner.load_sharded_model,runner.trajectory_to_world,runner.OverheadRecorder)
    options=tmp_path/'options.json'
    options.write_text(json.dumps(dict(steering_mode='route',num_samples=1,verify_signal=True,
                                      speed_selection_attempts=6,projection_cache_age_s=1.)))
    (tmp_path/'experiment.json').write_text('{"source_sha256": {}}')
    def fail(*args):
        assert runner.trajectory_to_world is cache.anchored_world_path
        assert probe.native.Probe is cache.RetainedSpeedProbe
        cache.ACTIVE=object()
        raise RuntimeError('deliberate capture failure')
    monkeypatch.setattr(probe,'ORIGINAL_CAPTURE',fail)
    with pytest.raises(RuntimeError,match='deliberate'):
        probe.capture(SimpleNamespace(options=options),tmp_path)
    assert (runner.TrajectoryFollower,runner.connect_world,probe.native.Probe,
            runner.load_sharded_model,runner.trajectory_to_world,runner.OverheadRecorder)==original
    assert cache.ACTIVE is None
