"""Check actual concurrent execution, source anchors, and worker error delivery."""
import copy
import json
import queue
import threading

import numpy as np
import pytest
import torch

from module.async_inference import run_inference_worker
from local.experiments import async_projected_probe as async_probe
from local.experiments.audit_projection import async_selection_checks, rest_schedule_checks, rest_schedule_reference
from local.experiments.test_projected_retention import scene
from local.experiments.test_projected_speed_selection import candidate


def restart_record():
    # One second of waiting, then a smooth forward acceleration.
    t=np.arange(1,65)*.1
    raw=np.column_stack((np.maximum(0.,t-1.5)**2,np.zeros(64),np.zeros(64)))
    return dict(label='async_request_0000',context=dict(index=0,time_s=0.,measured_speed_mps=0.,
                world_pose=dict(x=0.,y=0.,z=0.,yaw=0.,pitch=0.,roll=0.)),
                trajectory_ego=raw.tolist(),reverse_velocity_steps=0,visual_signal='green',
                inference_seconds=2.)


@pytest.mark.parametrize('case,expected',[
    ('wait_go',True),('stopped',False),('reverse',False),('immediate_go',False),
    ('already_moving',False),('red',False),('yellow',False),('nonfinite',False)])
def test_restart_extension_requires_a_native_forward_restart(case,expected):
    record=restart_record()
    if case=='stopped':record['trajectory_ego']=np.zeros((64,3)).tolist()
    elif case=='reverse':record['reverse_velocity_steps']=1
    elif case=='immediate_go':record['trajectory_ego']=np.column_stack((np.arange(1,65)*.3,np.zeros(64),np.zeros(64))).tolist()
    elif case=='already_moving':record['context']['measured_speed_mps']=.21
    elif case in ('red','yellow'):record['visual_signal']=case
    elif case=='nonfinite':record['trajectory_ego'][20][0]=float('nan')
    assert async_probe.rest_plan_candidate(record,2.) is expected
    assert bool(rest_schedule_reference(record,2.)) is expected


def test_restart_eligibility_uses_the_declared_traffic_preview():
    record=restart_record()
    speeds=np.r_[np.zeros(10),np.full(54,2.)]
    record['trajectory_ego']=np.column_stack((np.cumsum(speeds)*.1,np.zeros(64),np.zeros(64))).tolist()
    assert not async_probe.rest_plan_candidate(record,2.)
    assert not rest_schedule_reference(record,2.)
    record['options']={'traffic':{'speed_preview_lead_s':.5}}
    assert async_probe.rest_plan_candidate(record,2.)
    assert rest_schedule_reference(record,2.)


@pytest.mark.parametrize('answer,eligible',[
    ('MOVING',True),('clear.',True),('STOPPED',False),('UNKNOWN',False),
    ('moving or stopped',False),('The car may be moving',False)])
@pytest.mark.parametrize('last_executed',[False,True])
def test_visual_restart_query_preserves_native_prediction_rng_and_counts_cost(monkeypatch,tmp_path,answer,eligible,last_executed):
    monkeypatch.setattr(async_probe.BASE_PROBE,'__init__',lambda self,model,output:setattr(self,'output',output))
    monkeypatch.setattr(async_probe.BASE_PROBE,'close',lambda self:None)
    monkeypatch.setattr(torch.cuda,'device_count',lambda:0)
    record=restart_record()
    class Probe(async_probe.AsyncSingleSampleProbe):
        def infer_plan(self,request):
            (self.output/(request['label']+'.json')).write_text(json.dumps(record))
            return torch.tensor(record['trajectory_ego'],dtype=torch.float64)[None,None,None],{}
    probe=Probe(None,tmp_path);probe.model=None
    probe.trace={'native':'preserved'};probe.options={'native':'options'}
    class Adapter:
        probe_processor=None
        def run_vqa(self,*args):
            assert args[-1]==async_probe.REST_MOTION_QUESTION
            probe.trace['vqa']='separate'
            torch.rand(4)
            return answer
        def extract_answer_text(self,value):return value
    options={'async_rest_planning_interval_s':2.,'traffic':{'kind':'moving_lead'}}
    data={'image_frames':torch.arange(12).reshape(1,1,3,2,2),'camera_indices':torch.tensor([0])}
    request=dict(label=record['label'],frame=0,phase='approach',submitted_wall=0.,
                 context=copy.deepcopy(record['context']),options=options,data=data,adapter=Adapter())
    before=torch.get_rng_state().clone()
    try:
        result=probe.infer(request);saved=result['record']
        assert saved['trajectory_ego']==record['trajectory_ego']
        assert np.array_equal(result['prediction'].numpy()[0,0,0],record['trajectory_ego'])
        assert torch.equal(before,torch.get_rng_state())
        assert probe.trace=={'native':'preserved'} and probe.options=={'native':'options'}
        assert saved['rest_interval_eligible'] is eligible
        assert saved['inference_seconds']>=2.+saved['rest_motion_check']['seconds']
        captured=torch.load(tmp_path/saved['rest_motion_check']['input_file'],weights_only=True)
        assert captured['context']==record['context']
        assert torch.equal(captured['data']['image_frames'],data['image_frames'])
        assert all(rest_schedule_checks(saved,tmp_path,options).values())
        tampered=copy.deepcopy(saved);tampered['rest_interval_eligible']=not eligible
        assert not rest_schedule_checks(tampered,tmp_path,options)['rest_eligibility_reconstructed']
        tampered=copy.deepcopy(saved);tampered['inference_seconds']=2.
        assert not rest_schedule_checks(tampered,tmp_path,options)['rest_visual_cost_counted']
        tampered=copy.deepcopy(saved);tampered['rest_motion_check']['input_sha256']='wrong'
        assert not rest_schedule_checks(tampered,tmp_path,options)['rest_visual_input_recorded']
        probe.execution_record={'async_execution':{'source_label':record['label'] if last_executed else 'earlier_request'}}
        probe.close()
        info=json.loads((tmp_path/'last-native-input.json').read_text())
        last=torch.load(tmp_path/info['input_file'],weights_only=True)
        assert info['used_by_last_control'] is last_executed
        assert info['source_label']==last['label']==record['label']
        assert last['context']==info['source_context']==record['context']
        assert torch.equal(last['data']['image_frames'],data['image_frames'])
    finally:
        probe.close();async_probe.ACTIVE=None;async_probe.projected_retention.ACTIVE=None


def test_worker_delivers_failure_and_remains_available():
    requests, results, stop = queue.Queue(1), queue.Queue(1), threading.Event()
    def process(request):
        if request['frame'] == 1:
            raise ValueError('invalid prediction')
        return dict(frame_submitted=request['frame'], answer='ok')
    thread=threading.Thread(target=run_inference_worker,args=(requests,results,stop,process))
    thread.start()
    try:
        requests.put(dict(frame=1,respawn_revision=7))
        error=results.get(timeout=2)
        assert error['frame_submitted']==1 and error['respawn_revision']==7
        assert error['error']=='invalid prediction' and 'ValueError' in error['traceback']
        requests.put(dict(frame=2,respawn_revision=7))
        assert results.get(timeout=2)==dict(frame_submitted=2,answer='ok')
    finally:
        stop.set(); thread.join(timeout=2)
    assert not thread.is_alive()


def test_shutdown_surfaces_unconsumed_worker_error_and_closes_once(monkeypatch,tmp_path):
    closed=[]
    monkeypatch.setattr(async_probe.BASE_PROBE,'__init__',lambda self,model,output:setattr(self,'output',output))
    monkeypatch.setattr(async_probe.BASE_PROBE,'close',lambda self:closed.append(True))
    probe=async_probe.AsyncSingleSampleProbe(None,tmp_path)
    probe.results.put(dict(error='late inference failure',frame_submitted=8))
    try:
        with pytest.raises(RuntimeError,match='late inference failure'):
            probe.close()
        probe.close()
        assert closed == [True]
        assert json.loads((tmp_path/'async-summary.json').read_text())['shutdown_worker_error']
        assert json.loads((tmp_path/'async-worker-error.json').read_text())['frame_submitted']==8
    finally:
        async_probe.ACTIVE=None
        async_probe.projected_retention.ACTIVE=None


def retention_fixture():
    route,context,old,options=scene()
    old['record']['options']={'async_navigation_phase':'approach'}
    slow=candidate(.1)
    slow.update(label='proposal_new',options={'async_navigation_phase':'approach'},
        context={**context,'time_s':.4})
    plan=async_probe.execution_plan(slow,'slow_prediction',{})
    current=dict(execution_plan=plan,phase='approach',record={**slow,'label':'async_request_new'})
    options.update(async_retention_age_s=3.,scene_policy='empty_green')
    return route,context,old,current,options


def test_async_retention_revalidates_age_pose_and_signal_at_execution():
    route,context,old,current,options=retention_fixture()
    chosen,retention=async_probe.choose_execution_plan(current,[old],context,route,options)
    assert chosen is old and retention['checks']['age_s']==.5
    assert chosen['result'][0]=='unchanged_prediction'
    for signal in ['red','yellow']:
        red={**current,'record':{**current['record'],'visual_signal':signal}}
        assert async_probe.choose_execution_plan(red,[old],context,route,options)==(current['execution_plan'],None)
    expired={**context,'time_s':3.1}
    assert async_probe.choose_execution_plan(current,[old],expired,route,options)[0] is current['execution_plan']
    offlane={**context,'world_pose':{**context['world_pose'],'y':2.}}
    assert async_probe.choose_execution_plan(current,[old],offlane,route,options)[0] is current['execution_plan']
    revised={**current,'phase':'finish_turn'}
    chosen,retention=async_probe.choose_execution_plan(revised,[old],context,route,options)
    assert chosen is old and retention['checks']['age_s']==.5
    assert chosen['result'][0]=='unchanged_prediction'


def test_worker_retention_cannot_bypass_revalidation_after_delivery():
    route,context,old,current,options=retention_fixture()
    current.update(execution_plan=old,record={**old['record'],'label':'async_request_new','retention':{}})
    chosen,retention=async_probe.choose_execution_plan(current,[old],context,route,options)
    assert chosen is old and retention is not None
    with pytest.raises(RuntimeError,match='no longer valid'):
        async_probe.choose_execution_plan(current,[old],{**context,'time_s':3.1},route,options)


@pytest.mark.parametrize('navigation_changes',[False,True])
def test_worker_keeps_original_source_when_reusing_a_previous_native_plan(monkeypatch,tmp_path,navigation_changes):
    route,context,old,options=scene()
    monkeypatch.setattr(async_probe.BASE_PROBE,'__init__',lambda self,model,output:setattr(self,'output',output))
    monkeypatch.setattr(async_probe.BASE_PROBE,'close',lambda self:None)
    calls=[]
    def evaluate(self,adapter,data,context,instruction,options,label,capture=False,capture_failures=False):
        record=candidate(4. if not calls else .1)
        record.update(label=label,context=context,options=options,
                      visual_signal_answer='green',visual_signal_check_index=context['index'])
        calls.append(label)
        (self.output/(label+'.json')).write_text(json.dumps(record))
        return torch.tensor(record['trajectory_ego'])[None,None,None],{}
    monkeypatch.setattr(async_probe.BASE_PROBE,'evaluate',evaluate)
    probe=async_probe.AsyncSpeedSelectedProbe(None,tmp_path)
    probe.route=route
    context.update(index=5,signal_states={'1':'Green'})
    initial={**context,'index':0,'time_s':0.,'world_pose':{**context['world_pose'],'x':0.}}
    options.update(scene_policy='empty_green',async_retention_age_s=3.,speed_selection_attempts=6,
                   seed=44,flow_seed=43)
    try:
        first=probe.infer(probe.make_request(None,{},initial,'Go',options,False))
        if navigation_changes:
            probe.phase='exit'
        second=probe.infer(probe.make_request(None,{},context,'Take the exit' if navigation_changes else 'Go',options,False))
        assert len(calls)==2
        assert second['record']['retention']['source_context']==initial
        assert second['record']['async_request']['phase']==('exit' if navigation_changes else 'approach')
        assert second['execution_plan']['record']['options']['async_navigation_phase']=='approach'
        assert second['execution_plan']['record']['context']==initial
        assert all(async_selection_checks(second['record'],tmp_path,options).values())
        actual={**context,'index':7,'time_s':.7,'world_pose':{**context['world_pose'],'x':2.8}}
        chosen,retention=async_probe.choose_execution_plan(second,second['history'],actual,route,options)
        np.testing.assert_array_equal(chosen['world'],first['execution_plan']['world'])
        assert retention['checks']['age_s']==.7
    finally:
        probe.close()
        async_probe.ACTIVE=None
        async_probe.projected_retention.ACTIVE=None


@pytest.mark.parametrize('retained',[True,False])
@pytest.mark.parametrize('pending_valid',[True,False])
def test_expired_retention_waits_without_advancing_physics_then_uses_original_new_anchor(monkeypatch,tmp_path,pending_valid,retained):
    route,context,old,current,options=retention_fixture()
    old['record']['context']['index']=0
    actual={**context,'index':31,'time_s':3.1,'world_pose':{**context['world_pose'],'x':12.4}}
    request_context={**actual,'index':30,'time_s':3.,'world_pose':{**actual['world_pose'],'x':12.}}
    monkeypatch.setattr(async_probe.BASE_PROBE,'__init__',lambda self,model,output:setattr(self,'output',output))
    monkeypatch.setattr(async_probe.BASE_PROBE,'close',lambda self:None)
    probe=async_probe.AsyncSingleSampleProbe(None,tmp_path)
    probe.route=route
    options['async_revalidate_delayed_plans']=not retained
    record={**old['record'],'label':'async_old','context':request_context}
    if retained:record['retention']={}
    probe.current=dict(record=record,execution_plan=old,phase='approach')
    probe.history=[old]
    probe.pending=True;probe.pending_frame=30
    waiting,finished=threading.Event(),threading.Event()
    original_get=probe.results.get
    def observed_get(*args,**kwargs):
        if kwargs.get('timeout')==60.:waiting.set()
        return original_get(*args,**kwargs)
    monkeypatch.setattr(probe.results,'get',observed_get)
    def fresh(ctx,label):
        record=candidate(4.)
        record.update(label=label,context=copy.deepcopy(ctx),options={'async_navigation_phase':'approach'})
        plan=async_probe.execution_plan(record,torch.tensor(record['trajectory_ego'])[None,None,None],{})
        return dict(record=record,execution_plan=plan,phase='approach',history=[])
    def process(request):
        return fresh(request['context'],request['label'])
    probe.worker=threading.Thread(target=run_inference_worker,
        args=(probe.requests,probe.results,probe.stop,process))
    probe.worker.start()
    output=[];errors=[]
    def advance():
        try:
            output.append(probe.ensure_executable_plan(None,{},actual,'Go',options,False))
        except BaseException as exc:
            errors.append(exc)
        finally:
            finished.set()
    control=threading.Thread(target=advance)
    control.start()
    try:
        assert waiting.wait(timeout=2) and not finished.is_set()
        assert actual['time_s']==3.1 and actual['world_pose']['x']==12.4
        probe.results.put(fresh(request_context,'async_pending') if pending_valid else probe.current)
        control.join(timeout=2)
        assert finished.is_set() and not errors
        selected,retention=output[0]
        assert selected['record']['context']['time_s']==(3. if pending_valid else 3.1)
        assert selected['record']['context']['world_pose']['x']==(12. if pending_valid else 12.4)
        assert probe.request_count==(0 if pending_valid else 1)
        assert probe.plan_refresh_wait_count==(1 if pending_valid else 2)
        if not retained and pending_valid:
            assert retention['kind']=='delayed_native' and retention['checks']['valid']
        else:
            assert retention is None
    finally:
        probe.stop.set()
        if control.is_alive():probe.results.put(dict(error='test cleanup'))
        control.join(timeout=2)
        probe.close()
        async_probe.ACTIVE=None
        async_probe.projected_retention.ACTIVE=None


def test_newly_delivered_slow_plan_revalidated_without_overriding_visual_stop_or_fresh_budget():
    route,context,old,current,options=retention_fixture()
    options['async_revalidate_delayed_plans']=True
    with pytest.raises(RuntimeError,match='no longer valid'):
        async_probe.choose_execution_plan(current,[],context,route,options)
    for signal in ['red','yellow']:
        stopping={**current,'record':{**current['record'],'visual_signal':signal}}
        assert async_probe.choose_execution_plan(stopping,[],context,route,options)==(current['execution_plan'],None)
    fresh_context=current['execution_plan']['record']['context']
    assert async_probe.choose_execution_plan(current,[],fresh_context,route,options)==(current['execution_plan'],None)


def test_fresh_current_frame_plan_is_not_replaced_by_a_worse_old_plan():
    route,context,old,current,options=retention_fixture()
    old_record={**old['record'],**candidate(2.)}
    old=async_probe.execution_plan(old_record,'old_prediction',{})
    record=candidate(4.) | dict(label='fresh',context=copy.deepcopy(context))
    current['execution_plan']=async_probe.execution_plan(record,'fresh_prediction',{})
    chosen,retention=async_probe.choose_execution_plan(current,[old],context,route,options)
    assert chosen is current['execution_plan'] and retention is None


@pytest.mark.parametrize('policy,signal,expected_calls',[
    ('empty_green','green',2), ('empty_green','red',1), ('red_light_stop','red',1)])
def test_async_uses_original_bounded_selection_and_preserves_stops(monkeypatch,tmp_path,policy,signal,expected_calls):
    calls=[]
    monkeypatch.setattr(async_probe.BASE_PROBE,'__init__',lambda self,model,output:setattr(self,'output',output))
    monkeypatch.setattr(async_probe.BASE_PROBE,'close',lambda self:None)
    def evaluate(self,adapter,data,context,instruction,options,label,capture=False,capture_failures=False):
        speed=2. if not calls else 4.
        calls.append(label)
        raw=np.column_stack((np.arange(1,65)*.1*speed,np.zeros(64),np.zeros(64)))
        record=dict(label=label,context=context,trajectory_ego=raw.tolist(),inference_seconds=2.,
            controller_preview_speed_mps=speed,trajectory_marker_present=True,
            reverse_velocity_steps=0,visual_signal=signal)
        (self.output/(label+'.json')).write_text(json.dumps(record))
        return torch.tensor(raw)[None,None,None],{}
    monkeypatch.setattr(async_probe.BASE_PROBE,'evaluate',evaluate)
    probe=async_probe.AsyncSpeedSelectedProbe(None,tmp_path)
    context=dict(index=0,time_s=0.,signal_states={'1':'Red' if policy=='red_light_stop' else 'Green'},
        world_pose=dict(x=0.,y=0.,z=0.,yaw=0.,pitch=0.,roll=0.),
        controlled_scene=dict(verified=True,policy=policy,other_vehicle_count=0,
                              pedestrian_count=0,unexpected_signal_count=0))
    options=dict(scene_policy=policy,speed_selection_attempts=6,seed=44,flow_seed=43,
                 controller_speed_cap_mps=4.5)
    try:
        request=probe.make_request(None,{},context,'Continue straight',options,False)
        result=probe.infer(request)
        assert len(calls)==expected_calls
        record=result['record']
        assert record['selection']['attempts']==expected_calls
        assert record['inference_seconds']==expected_calls*2.
        assert all(async_selection_checks(record,tmp_path,options).values())
        record['trajectory_ego'][0][0]+=1
        assert not async_selection_checks(record,tmp_path,options)['async_selected_native_source']
    finally:
        probe.close()
        async_probe.ACTIVE=None
        async_probe.projected_retention.ACTIVE=None


@pytest.mark.parametrize('advance_limit,heading_limit', [(None,None),(.2,None),(1.,10.)])
def test_control_can_advance_while_worker_is_blocked_without_reanchoring(monkeypatch,tmp_path,advance_limit,heading_limit):
    started, release = threading.Event(), threading.Event()
    monkeypatch.setattr(async_probe.native.Probe,'__init__',lambda self,model,output:setattr(self,'output',output))
    monkeypatch.setattr(async_probe.native.Probe,'close',lambda self:None)
    monkeypatch.setattr(async_probe,'route_state',lambda *args:(False,'approach','Continue straight'))
    class FakeProbe(async_probe.AsyncSingleSampleProbe):
        def infer(self,request):
            if request['frame']:
                started.set()
                assert release.wait(timeout=3)
            raw=np.column_stack((np.arange(1,65)*.3,np.zeros(64),np.zeros(64)))
            record=dict(label=request['label'],context=copy.deepcopy(request['context']),trajectory_ego=raw.tolist())
            (self.output/(request['label']+'.json')).write_text(json.dumps(record))
            return dict(record=record, prediction=torch.tensor(raw)[None,None,None],extra={},phase='approach')
    probe=FakeProbe(None,tmp_path)
    probe.route={}
    options=dict(validation_route='straight')
    if advance_limit is not None:
        options['async_max_pending_sim_s']=advance_limit
    if heading_limit is not None:
        options['async_max_pending_heading_deg']=heading_limit
    initial=dict(index=0,time_s=0.,world_pose=dict(x=0.,y=0.,z=0.,yaw=0.,pitch=0.,roll=0.))
    try:
        probe.evaluate(None,{},initial,'',options,'frame_000')
        original_world=probe.execution_world.copy()
        probe.last_submit_wall=float('-inf')
        current={**initial,'index':1,'time_s':.1,'world_pose':{**initial['world_pose'],'x':10.,'yaw':179. if heading_limit else 0.}}
        probe.evaluate(None,{},current,'',options,'frame_001')
        assert started.wait(timeout=2)
        later={**current,'index':2,'time_s':.2,'world_pose':{**current['world_pose'],'yaw':-179. if heading_limit else 0.}}
        probe.evaluate(None,{},later,'',options,'frame_002')
        assert probe.pending and probe.request_count==2
        assert probe.execution_age==pytest.approx(.2)
        np.testing.assert_array_equal(probe.execution_world,original_world)
        record=json.loads((tmp_path/'frame_002.json').read_text())
        assert record['async_execution']['source_context']==initial
        assert record['context']==later
        if advance_limit is not None:
            waiting, finished = threading.Event(), threading.Event()
            original_get = probe.results.get
            def observed_get(*args, **kwargs):
                if kwargs.get('timeout') == 60.:
                    waiting.set()
                return original_get(*args, **kwargs)
            monkeypatch.setattr(probe.results, 'get', observed_get)
            errors=[]
            def advance_to_limit():
                try:
                    boundary=dict(later,index=3,time_s=.3,world_pose={**later['world_pose'],'yaw':-171. if heading_limit else 0.})
                    probe.evaluate(None,{},boundary,'',options,'frame_003')
                except BaseException as exc:
                    errors.append(exc)
                finally:
                    finished.set()
            control=threading.Thread(target=advance_to_limit)
            control.start()
            try:
                assert waiting.wait(timeout=2)
                assert not finished.is_set()
                assert not (tmp_path/'frame_003.json').exists()
                release.set()
                control.join(timeout=2)
                assert finished.is_set() and not errors
                resumed=json.loads((tmp_path/'frame_003.json').read_text())
                assert resumed['async_execution']['source_context']==current
                assert resumed['async_execution']['age_s']==pytest.approx(.2)
                assert probe.wait_count==1
                assert probe.heading_wait_count==int(heading_limit is not None)
            finally:
                release.set()
                control.join(timeout=2)
    finally:
        release.set(); probe.close()
        async_probe.ACTIVE=None
        async_probe.projected_retention.ACTIVE=None


@pytest.mark.parametrize('trigger',['time','heading','red','yellow','decelerating','stationary_red',
                                    'signal_only_red','signal_only_decelerating',
                                    'rest_time','rest_accelerating','rest_heading','rest_red','rest_decelerating',
                                    'motion_time','motion_heading','motion_red','motion_decelerating','motion_navigation',
                                    'coarse_decelerating','coarse_steady','coarse_red','coarse_heading',
                                    'coarse_stationary','coarse_weak_deceleration','coarse_navigation'])
def test_sparse_fresh_planning_waits_at_boundary_and_keeps_current_input_anchor(monkeypatch,tmp_path,trigger):
    started,release=threading.Event(),threading.Event()
    monkeypatch.setattr(async_probe.BASE_PROBE,'__init__',lambda self,model,output:setattr(self,'output',output))
    monkeypatch.setattr(async_probe.BASE_PROBE,'close',lambda self:None)
    monkeypatch.setattr(async_probe,'route_state',lambda context,*args:(False,
        'finish' if trigger in ('motion_navigation','coarse_navigation') and context['index']>=5 else 'approach','Continue straight'))
    class FakeProbe(async_probe.AsyncSingleSampleProbe):
        def infer(self,request):
            if request['frame']:
                started.set()
                assert release.wait(timeout=3)
            raw=np.column_stack((np.arange(1,65)*.4,np.zeros(64),np.zeros(64)))
            if trigger.startswith('motion_'):
                raw=np.column_stack((np.cumsum(np.r_[np.zeros(22),np.full(42,2.)])*.1,np.zeros(64),np.zeros(64)))
            signal='yellow' if trigger=='yellow' else 'red' if trigger in ('red','stationary_red','signal_only_red','rest_red','motion_red','coarse_red') else 'green'
            record=dict(label=request['label'],context=copy.deepcopy(request['context']),trajectory_ego=raw.tolist(),visual_signal=signal)
            record['rest_interval_eligible']=trigger.startswith('rest_')
            (self.output/(request['label']+'.json')).write_text(json.dumps(record))
            return dict(record=record,prediction=torch.tensor(raw)[None,None,None],extra={},phase=request['phase'])
    probe=FakeProbe(None,tmp_path);probe.route={}
    options=dict(validation_route='straight',async_fresh_planning_interval_s=1.,async_max_pending_heading_deg=10.)
    options['async_fine_planning_interval_s']=.1
    options['async_fine_planning_signal_only']=trigger.startswith('signal_only_')
    if trigger.startswith('coarse_'):
        options.update(async_fresh_planning_interval_s=1.5,async_fine_planning_signal_only=True,
                       async_deceleration_planning_interval_s=1.)
    if trigger.startswith('rest_'):
        options['async_rest_planning_interval_s']=2.
    if trigger.startswith('motion_'):
        options['async_motion_commitment_s']=.5
    initial=dict(index=0,time_s=0.,measured_speed_mps=0. if trigger in ('stationary_red','coarse_stationary') or trigger.startswith(('rest_','motion_')) else 4.,world_pose=dict(x=0.,y=0.,z=0.,yaw=179.,pitch=0.,roll=0.))
    final_index=10 if trigger in ('time','stationary_red','signal_only_decelerating') else 3 if trigger=='heading' else 1
    if trigger in ('rest_time','rest_accelerating'):final_index=20
    elif trigger=='rest_heading':final_index=3
    elif trigger in ('rest_decelerating','motion_decelerating'):final_index=2
    elif trigger=='motion_time':final_index=13
    elif trigger=='motion_heading':final_index=3
    elif trigger=='motion_navigation':final_index=5
    if trigger.startswith('coarse_'):
        final_index={'coarse_decelerating':10,'coarse_red':1,'coarse_heading':3,'coarse_navigation':5}.get(trigger,15)
    control=None
    errors=[];finished=threading.Event()
    try:
        probe.evaluate(None,{},initial,'',options,'frame_000')
        for index in range(1,final_index):
            context=dict(initial,index=index,time_s=index*.1,world_pose={**initial['world_pose'],'x':index*.4,'yaw':-179.})
            if trigger in ('signal_only_decelerating','coarse_decelerating','coarse_weak_deceleration'):
                context['measured_speed_mps']=4.-index*(.04 if trigger=='coarse_weak_deceleration' else .06)
            elif trigger=='rest_accelerating':
                context['measured_speed_mps']=index*.1
            elif trigger in ('rest_decelerating','motion_decelerating'):
                context['measured_speed_mps']=1.
            probe.evaluate(None,{},context,'',options,f'frame_{index:03d}')
            assert probe.request_count==1 and not probe.pending
            if trigger=='motion_time' and index>=10:
                saved=json.loads((tmp_path/f'frame_{index:03d}.json').read_text())
                assert saved['async_execution']['source_context']==initial
                assert saved['async_execution']['age_s']==pytest.approx(index*.1)
        boundary=dict(initial,index=final_index,time_s=final_index*.1,world_pose={**initial['world_pose'],'x':final_index*.4,'yaw':-171. if trigger in ('heading','rest_heading','motion_heading','coarse_heading') else -179.})
        if trigger=='decelerating':
            boundary['measured_speed_mps']=3.9
        if trigger=='signal_only_decelerating':
            boundary['measured_speed_mps']=3.4
        if trigger in ('coarse_decelerating','coarse_weak_deceleration'):
            boundary['measured_speed_mps']=4.-final_index*(.04 if trigger=='coarse_weak_deceleration' else .06)
        elif trigger=='rest_accelerating':boundary['measured_speed_mps']=2.
        elif trigger in ('rest_red','motion_red'):boundary['measured_speed_mps']=.1
        elif trigger in ('rest_decelerating','motion_decelerating'):boundary['measured_speed_mps']=.9
        def advance():
            try:probe.evaluate(None,{},boundary,'',options,f'frame_{final_index:03d}')
            except BaseException as exc:errors.append(exc)
            finally:finished.set()
        control=threading.Thread(target=advance);control.start()
        assert started.wait(timeout=2) and not finished.is_set()
        assert not (tmp_path/f'frame_{final_index:03d}.json').exists()
        release.set();control.join(timeout=2)
        assert finished.is_set() and not errors
        record=json.loads((tmp_path/f'frame_{final_index:03d}.json').read_text())
        assert record['async_execution']['source_context']==boundary
        assert record['async_execution']['age_s']==0 and not record['async_execution']['pending_inference']
        assert probe.request_count==2 and probe.fresh_planning_wait_count==1
        assert probe.fine_planning_wait_count==int(trigger in ('red','yellow','decelerating','signal_only_red','rest_red','rest_decelerating','motion_red','motion_decelerating','coarse_decelerating','coarse_red'))
    finally:
        release.set()
        if control is not None:control.join(timeout=2)
        probe.close()
        async_probe.ACTIVE=None;async_probe.projected_retention.ACTIVE=None


@pytest.mark.parametrize('variant,expected',[
    ('late_start',1.3),('disabled',1.),('moving',1.),('negative_speed',1.),('nan_speed',1.),
    ('red',1.),('yellow',1.),('reverse',1.),('nonfinite',1.),('short',1.),
    ('after_boundary',1.),('immediate',1.),('early_start',1.)])
def test_native_motion_commitment_never_waits_for_future_motion_or_overrides_stop(variant,expected):
    from local.experiments.audit_projection import motion_interval_reference
    record=restart_record()
    start=27 if variant=='after_boundary' else 0 if variant=='immediate' else 16 if variant=='early_start' else 22
    speeds=np.r_[np.zeros(start),np.full(64-start,2.)]
    record['trajectory_ego']=np.column_stack((np.cumsum(speeds)*.1,np.zeros(64),np.zeros(64))).tolist()
    commitment=None if variant=='disabled' else .5
    if variant=='moving':record['context']['measured_speed_mps']=.21
    elif variant=='negative_speed':record['context']['measured_speed_mps']=-.1
    elif variant=='nan_speed':record['context']['measured_speed_mps']=float('nan')
    elif variant in ('red','yellow'):record['visual_signal']=variant
    elif variant=='reverse':record['reverse_velocity_steps']=1
    elif variant=='nonfinite':record['trajectory_ego'][3][0]=float('nan')
    elif variant=='short':record['trajectory_ego']=(np.asarray(record['trajectory_ego'])*.2).tolist()
    before=copy.deepcopy(record)
    assert async_probe.native_motion_interval(record,1.,commitment)==pytest.approx(expected)
    assert motion_interval_reference(record,1.,commitment)==pytest.approx(expected)
    # The extension changes no native point, source time, seed or controller target.
    assert json.dumps(record,sort_keys=True)==json.dumps(before,sort_keys=True)


def test_motion_commitment_respects_declared_preview_and_ordinary_boundary():
    from local.experiments.audit_projection import motion_interval_reference
    record=restart_record()
    speeds=np.r_[np.zeros(22),np.full(42,2.)]
    record['trajectory_ego']=np.column_stack((np.cumsum(speeds)*.1,np.zeros(64),np.zeros(64))).tolist()
    record['options']={'traffic':{'speed_preview_lead_s':.5}}
    # Motion now begins after1s with the shorter preview, so it gets no extension.
    assert async_probe.native_motion_interval(record,1.,.5)==1.
    assert motion_interval_reference(record,1.,.5)==1.
    record['options']['traffic']['speed_preview_lead_s']=1.
    assert async_probe.native_motion_interval(record,.5,.5)==.5
    assert motion_interval_reference(record,.5,.5)==.5
