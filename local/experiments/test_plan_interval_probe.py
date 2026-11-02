"""Bounded scheduling must preserve native plans and replan on every failed gate."""
import copy
import json

import numpy as np
import pytest
import torch

from local.experiments import candidate_reserve_probe as reserve, retained_probe as retained
from local.experiments import plan_interval_probe as interval, tracking_probe as tracking
from local.experiments.audit_retention import audit
from local.experiments.test_tracking_probe import fixture


def make_probe(tmp_path, monkeypatch):
    route, context, options, plan, follower = fixture()
    options.update(validation_replan_interval_s=.3, verify_signal=True, signal_check_interval=5,
                   validation_resample_below_target_mps=2.5, validation_resample_above_target_mps=5.)
    context['index'],context['time_s']=1,.1
    context['world_pose']['x']-=.9
    plan.record['visual_signal']='green'
    probe=object.__new__(interval.PlanIntervalProbe)
    probe.output,probe.history,probe.candidate_reserve=tmp_path,[plan],[]
    probe.executing_plan,probe.executing_age=plan,0.
    probe.validation_route,probe.navigation_phase=route,'approach'
    probe.visual_signal,probe.visual_signal_answer,probe.visual_signal_index='green','green',0
    assess=retained.assess_retained
    def screened(plan, context, route, signal, phase, options):
        metric=assess(plan,context,route,signal,phase,options)
        return tracking.screen(metric,context,route,plan.world,plan.speeds,
            context['time_s']-plan.record['context']['time_s'],options,follower)
    monkeypatch.setattr(retained,'assess_retained',screened)
    calls=[]
    def fresh(*args,**kwargs):
        calls.append(True)
        return 'fresh','extra'
    monkeypatch.setattr(reserve.CandidateReserveProbe,'evaluate',fresh)
    return probe,context,options,plan,calls,assess


def test_reuses_exact_native_source_without_inference_and_audits_it(tmp_path,monkeypatch):
    probe,context,options,plan,calls,assess=make_probe(tmp_path,monkeypatch)
    world,speeds=plan.world.copy(),plan.speeds.copy()
    inputs={'example':torch.tensor([1.,2.])}
    result,extra=probe.evaluate(None,inputs,context,'Go straight',options,'frame_001',capture=True)
    saved=torch.load(tmp_path/'inputs/frame_001.pt',weights_only=True)
    assert saved['inference_performed'] is False and torch.equal(saved['data']['example'],inputs['example'])
    assert not calls and torch.equal(result,plan.prediction)
    assert extra['cot'][0,0,0]=='original model intent'
    np.testing.assert_array_equal(world,plan.world);np.testing.assert_array_equal(speeds,plan.speeds)
    assert probe.executing_plan is plan and probe.executing_age==.1
    assert plan.record['context']['time_s']==0 and len(probe.history)==1
    frame=json.loads((tmp_path/'frame_001.json').read_text())
    assert frame['inference_performed'] is False and frame['inference_seconds']==0
    retained.native.write_json(tmp_path/'frame_000.json',plan.record)
    retained.native.write_json(tmp_path/'experiment.json',{'options':options})
    retained.native.write_json(tmp_path/'route-reference.json',{'routes':{'straight':probe.validation_route}})
    row={'time_s':.1,'source_label':'frame_000','source_context':plan.record['context'],
         'age_s':.1,'world_trajectory':world.tolist(),'speeds_mps':speeds.tolist()}
    (tmp_path/'controller-inputs.jsonl').write_text(json.dumps(row)+'\n')
    # The independent audit runs outside the live tracking wrapper.
    monkeypatch.setattr(retained,'assess_retained',assess)
    assert audit(tmp_path)['all_sources_anchors_speeds_and_ages_match']
    for field in ('interval_s','age_s','visual_signal_check_index'):
        changed=copy.deepcopy(frame);changed['retention']['schedule'][field]+=1
        retained.native.write_json(tmp_path/'frame_001.json',changed)
        assert audit(tmp_path)['mismatches']
    changed=copy.deepcopy(frame);changed['retention']['metrics'][0]['max_corner_overhang_m']-=1
    retained.native.write_json(tmp_path/'frame_001.json',changed)
    assert audit(tmp_path)['mismatches']


@pytest.mark.parametrize('change',['default','boundary','expired','signal_due','red','unknown','source_signal',
    'route','phase','outside','heading','low_target','high_target','reserve','completion','left_transition'])
def test_replans_when_any_scheduling_or_physical_gate_fails(tmp_path,monkeypatch,change):
    probe,context,options,plan,calls,_=make_probe(tmp_path,monkeypatch)
    if change=='default':options.pop('validation_replan_interval_s')
    elif change=='boundary':context.update(time_s=.3,index=3)
    elif change=='expired':context.update(time_s=.6,index=6)
    elif change=='signal_due':probe.visual_signal_index=-4
    elif change in ('red','unknown'):probe.visual_signal=change
    elif change=='source_signal':plan.record['visual_signal']='red'
    elif change=='route':options['validation_route']='left'
    elif change=='phase':probe.navigation_phase='finish_turn'
    elif change=='outside':context['world_pose']['y']=3.
    elif change=='heading':context['world_pose']['yaw']=40.
    elif change=='low_target':options['validation_resample_below_target_mps']=4.
    elif change=='high_target':options['validation_resample_above_target_mps']=2.
    elif change=='reserve':
        screened=retained.assess_retained
        def less_reserve(*a,**kw):
            metric=screened(*a,**kw);metric['preferred_reserve_valid']=False;return metric
        monkeypatch.setattr(retained,'assess_retained',less_reserve)
    elif change=='completion':context['world_pose'].update(x=51.,y=0.)
    elif change=='left_transition':
        options['validation_route']=plan.record['selection']['route']='left'
        context['world_pose']['yaw']=-60.
    assert probe.evaluate(None,{},context,'Go straight',options,'frame_001')==('fresh','extra')
    assert len(calls)==1 and not (tmp_path/'frame_001.json').exists()


@pytest.mark.parametrize('value',[0.,.15,.6,float('nan'),float('inf')])
def test_invalid_interval_fails_before_sampling(tmp_path,monkeypatch,value):
    probe,context,options,_,calls,_=make_probe(tmp_path,monkeypatch)
    options['validation_replan_interval_s']=value
    probe.executing_plan=None
    with pytest.raises(ValueError,match='interval'):
        probe.evaluate(None,{},context,'Go straight',options,'frame_001')
    assert not calls


def test_missing_tracking_screen_fails_closed(tmp_path,monkeypatch):
    probe,context,options,_,calls,assess=make_probe(tmp_path,monkeypatch)
    monkeypatch.setattr(retained,'assess_retained',assess)
    with pytest.raises(RuntimeError,match='tracking'):
        probe.evaluate(None,{},context,'Go straight',options,'frame_001')
    assert not calls


def test_entrypoint_restores_wrapper_on_failure(monkeypatch):
    original=reserve.CandidateReserveProbe
    monkeypatch.setattr(retained,'ACTIVE',None)
    def fail():
        assert reserve.CandidateReserveProbe is interval.PlanIntervalProbe
        raise RuntimeError('test failure')
    monkeypatch.setattr(reserve,'main',fail)
    with pytest.raises(RuntimeError,match='test failure'):
        interval.main()
    assert reserve.CandidateReserveProbe is original


def test_summary_survives_parent_cleanup_and_counts_real_inference(tmp_path, monkeypatch):
    original = reserve.CandidateReserveProbe
    monkeypatch.setattr(interval, 'ACTIVE', None)
    monkeypatch.setattr(reserve.CandidateReserveProbe, '__init__',
                        lambda self, model, output: setattr(self, 'output', output))
    def run():
        reserve.CandidateReserveProbe(None, tmp_path)
        folder = tmp_path / '000_verifai'
        folder.mkdir()
        (folder / 'summary.json').write_text(json.dumps({'inference_calls': 4}))
        for index, performed in enumerate((True, False, False, True)):
            (tmp_path / f'frame_{index:03d}.json').write_text(json.dumps({'inference_performed': performed}))
        proposals = tmp_path / 'proposals'
        proposals.mkdir()
        for name in ('frame_000_pool_0.json', 'frame_003_pool_0.json', 'frame_003_pool_1.json'):
            (proposals / name).write_text('{}')
        # This is what retained.main() actually does before interval.main returns.
        monkeypatch.setattr(retained, 'ACTIVE', None)
    monkeypatch.setattr(reserve, 'main', run)
    interval.main()
    summary = json.loads((tmp_path / '000_verifai/summary.json').read_text())
    assert summary['planning_steps'] == 4 and summary['scheduled_native_execution_steps'] == 2
    assert summary['inference_calls'] == 2 and summary['model_pool_calls'] == 3
    assert summary['replayed_planning_steps'] == 0
    assert reserve.CandidateReserveProbe is original and interval.ACTIVE is None
