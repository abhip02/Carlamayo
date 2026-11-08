"""CPU-only checks of full-prefix cutoff and owned-process orchestration."""
from contextlib import ExitStack
import json
from pathlib import Path
import signal
import subprocess
from types import SimpleNamespace

import pytest

from local.experiments import loaded_camera_study as study


def fixture_hooks(tmp_path,pause=0,fail=False):
    calls=[]
    class Interface:
        sensors={}
        frame_id=60
        history_buffer=[{'speed':3.}]
        def setup_cameras(self):calls.append('cameras')
        def get_ego_state(self):calls.append('state_read');return {'speed':3.}
    class Probe:
        def original(self,*a,**k):raise AssertionError('Underlying inference executed')
        evaluate=generate=sample=cfg_sample=flow_sample=original
    def initialize(obj,entry,spec):
        calls.append('initialize');obj.setup_cameras()
        if fail:raise RuntimeError('CARLA startup failed')
        return [SimpleNamespace(shape=(4,1080,1920,3))]*4,{'x':1}
    preroll=SimpleNamespace(initialize_ego=initialize)
    result=dict(forbidden_inference_attempts=0,reached_handover=False)
    hooks=study.prefix_hooks(preroll,Interface,Probe,pause,tmp_path,lambda name,**kw:calls.append(name),result)
    return preroll,Interface,Probe,result,hooks,calls


def test_prefix_runs_original_preroll_and_state_read_then_unconditionally_stops(tmp_path):
    preroll,Interface,Probe,result,hooks,calls=fixture_hooks(tmp_path)
    original=preroll.initialize_ego
    with hooks:
        with pytest.raises(study.PrefixComplete):preroll.initialize_ego(Interface(),{},dict(speed_mps=3))
    assert preroll.initialize_ego is original and result['reached_handover']
    assert calls==['preroll_begin','initialize','camera_setup_begin','cameras','camera_setup_end','state_read','handover_verified']
    assert result['forbidden_inference_attempts']==0
    assert study.read(tmp_path/'prefix-handover.json')['retained_history_frames']==4


@pytest.mark.parametrize('method',['evaluate','generate','sample','cfg_sample','flow_sample'])
def test_all_inference_entrypoints_block_before_underlying_execution_and_restore(tmp_path,method):
    preroll,Interface,Probe,result,hooks,calls=fixture_hooks(tmp_path)
    original=getattr(Probe,method)
    with hooks:
        with pytest.raises(RuntimeError,match='forbids ALL'):getattr(Probe(),method)()
    assert getattr(Probe,method) is original and result['forbidden_inference_attempts']==1


def test_camera_failure_is_not_converted_to_expected_cutoff(tmp_path):
    preroll,Interface,Probe,result,hooks,calls=fixture_hooks(tmp_path,fail=True)
    original=Interface.setup_cameras
    with hooks:
        with pytest.raises(RuntimeError,match='CARLA'):preroll.initialize_ego(Interface(),{},dict(speed_mps=3))
    assert not result['reached_handover'] and Interface.setup_cameras is original


def valid_output(tmp_path):
    (tmp_path/'000_verifai').mkdir()
    precision=dict(precision='nf4',quantized_layers=list(range(624)),cpu_offload=False,compute_dtype='bfloat16',
        startup_memory={'devices':[{'allocation_budget_fraction':.45}]})
    summary=dict(simulation_seconds=0,inference_calls=0,error='PrefixComplete: expected cutoff')
    study.write(tmp_path/'precision.json',precision);study.write(tmp_path/'000_verifai/summary.json',summary)
    return dict(model_loaded=True,forbidden_inference_attempts=0,reached_handover=True)


@pytest.mark.parametrize('bad',['none','inference','wrong_budget','artifact','missing_model','wrong_cutoff'])
def test_prefix_acceptance_requires_zero_calls_and_verified_repair(tmp_path,bad):
    result=valid_output(tmp_path)
    if bad in ('inference','wrong_cutoff'):
        p=tmp_path/'000_verifai/summary.json';d=study.read(p)
        d['inference_calls' if bad=='inference' else 'error']=1 if bad=='inference' else 'OtherError'
        study.write(p,d)
    if bad=='wrong_budget':
        p=tmp_path/'precision.json';d=study.read(p);d['startup_memory']['devices'][0]['allocation_budget_fraction']=.41;study.write(p,d)
    if bad=='artifact':(tmp_path/'proposal_000_00.json').write_text('{}')
    if bad=='missing_model':result['model_loaded']=False
    if bad=='none':assert study.verify_prefix_output(tmp_path,result)
    else:
        with pytest.raises(ValueError):study.verify_prefix_output(tmp_path,result)


class Child:
    pid=12345
    returncode=None
    def poll(self):return self.returncode
    def wait(self,timeout):
        if self.returncode is None:raise subprocess.TimeoutExpired('owned',timeout)
        return self.returncode
    def terminate(self):self.returncode=-15
    def kill(self):self.returncode=-9


@pytest.mark.parametrize('fault',['server_dead','probe_timeout','deadline','interrupt'])
def test_supervision_stops_owned_child_on_every_failure(monkeypatch,fault):
    child=Child();state={}
    monkeypatch.setattr(study.subprocess,'Popen',lambda *a,**kw:child)
    times=iter([0,1000 if fault=='deadline' else 1])
    monkeypatch.setattr(study.time,'monotonic',lambda:next(times))
    def inspect(*a,**kw):
        assert kw['timeout']==5
        if fault=='probe_timeout':raise subprocess.TimeoutExpired('docker',5)
        if fault=='interrupt':raise InterruptedError('parent')
        return '{"Running":false}'
    monkeypatch.setattr(study.subprocess,'check_output',inspect)
    with pytest.raises((RuntimeError,TimeoutError,subprocess.TimeoutExpired,InterruptedError)):
        study.supervise(['owned'],{},None,'owned',state,lambda:None)
    assert child.returncode==-15 and state['child_returncode']==-15


def protocol(tmp_path):
    trials=[]
    inputs=tmp_path/'input.json';inputs.write_text('{}')
    for i,pause in enumerate(study.ORDER):
        path=tmp_path/(str(i)+'-jobs.json');cmd=['registered',str(i),'--config',str(inputs),'--options',str(inputs)]
        study.write(path,{'jobs':[{'command':cmd,'config_sha256':study.digest(inputs),'options_sha256':study.digest(inputs)}]})
        trials.append(dict(name=str(i),pause_seconds=pause,command=cmd,manifest=str(path),manifest_sha256=study.digest(path)))
    study.write(tmp_path/'protocol.json',dict(trials=trials,source_sha256={},order=list(study.ORDER),
        execution_assignment=dict(model_gpu=study.MODEL_GPU,renderer_gpu=study.RENDER_GPU)))


@pytest.mark.parametrize('failure',['startup','resource','cleanup','inference','interrupt','result'])
def test_predeclared_starts_are_not_retried_and_critical_failures_stop(monkeypatch,tmp_path,failure):
    protocol(tmp_path);calls=[]
    def run(trial,root):
        calls.append(trial['pause_seconds'])
        return dict(state='trial_error',prefix_pass=False,start_attempted=failure!='resource',
            cleanup={'verified':failure!='cleanup'},prefix_result={'forbidden_inference_attempts':int(failure=='inference')},
            error='InterruptedError()' if failure=='interrupt' else 'startup fault',result_error='bad json' if failure=='result' else None)
    monkeypatch.setattr(study,'run_trial',run)
    if failure=='startup':
        study.run_study(tmp_path);assert calls==[0,1,1,0]
    else:
        with pytest.raises(SystemExit):study.run_study(tmp_path)
        assert len(calls)==1
    with pytest.raises(ValueError,match='Never retry'):study.run_study(tmp_path)


def test_repair_is_exact_reviewed_copy():
    assert study.digest(study.ROOT/'local/experiments/nf4_startup_loader.py')==study.REPAIR_SHA


def test_child_delegates_exact_capture_prefix_and_waits_for_its_cleanup(monkeypatch,tmp_path):
    from local.experiments import nf4_policy_pair as policy, nf4_startup_loader as repaired
    from local.experiments import acknowledged_preroll, inference_probe
    out=tmp_path/'capture';manifest=tmp_path/'jobs.json'
    study.write(manifest,{'startup_only':{'repair_sha256':study.REPAIR_SHA,'pause_seconds':0}})
    calls=[]
    interface=SimpleNamespace(frame_id=60,history_buffer=[{'speed':3.}],get_ego_state=lambda:{'speed':3.})
    initialize=lambda *a:([SimpleNamespace(shape=(4,1080,1920,3))]*4,{'x':1})
    monkeypatch.setattr(acknowledged_preroll,'initialize_ego',initialize)
    loader=lambda *a,**kw:(None,None)
    monkeypatch.setattr(inference_probe.Alpamayo15Adapter,'load_model',loader)
    args=['--jobs-manifest',str(manifest),'--output',str(out)]
    old_bridge=policy.bridge
    def capture(actual):
        assert actual==args and policy.bridge is repaired
        out.mkdir();valid_output(out)
        inference_probe.Alpamayo15Adapter().load_model()
        try:acknowledged_preroll.initialize_ego(interface,{},dict(speed_mps=3))
        finally:calls.append('original_capture_cleanup')
    monkeypatch.setattr(policy,'capture',capture)
    study.child(args)
    result=study.read(tmp_path/'capture-prefix.json')
    assert result['state']=='prefix_complete' and result['model_loaded']
    assert calls==['original_capture_cleanup'] and policy.bridge is old_bridge
    assert inference_probe.Alpamayo15Adapter.load_model is loader
    assert acknowledged_preroll.initialize_ego is initialize


def test_parent_keeps_reviewed_three_resource_leases_gates_and_explicit_gpu_env(monkeypatch,tmp_path):
    from contextlib import contextmanager
    from local.experiments import resource_lease
    calls=[]
    @contextmanager
    def lease(gpus,port):
        assert gpus==[study.MODEL_GPU,study.RENDER_GPU] and port==2000
        calls.append('lease');yield
    monkeypatch.setattr(resource_lease,'lease',lease)
    port=iter([False,True]);monkeypatch.setattr(study,'port_open',lambda:next(port))
    ticks=iter([0,100,101]);monkeypatch.setattr(study.time,'monotonic',lambda:next(ticks))
    def memory(gpus,minimums):
        assert gpus==[study.MODEL_GPU,study.RENDER_GPU] and minimums==[11.,9.8]
        calls.append('gates');return dict(model=12,render=11)
    monkeypatch.setattr(study.guard,'bounded_memory',memory)
    monkeypatch.setattr(study.subprocess,'run',lambda *a,**kw:SimpleNamespace(returncode=1 if a[0][1]=='inspect' else 0))
    def helper(command,kwargs):
        assert kwargs['env']['CUDA_VISIBLE_DEVICES']==study.MODEL_GPU
        assert kwargs['env']['CARLAMAYO_RENDER_GPU']==study.RENDER_GPU
        calls.append('stop' if command[-1]=='stop' else 'start')
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(study.guard,'run_server_helper',helper)
    monkeypatch.setattr(study.camera,'cleanup_snapshot',lambda *a:dict(verified=True))
    def supervised(command,env,log,container,state,save):
        assert command==['registered'] and env['CUDA_VISIBLE_DEVICES']==study.MODEL_GPU
        state['child_returncode']=0
        study.write(tmp_path/'01-prefix.json',dict(state='prefix_complete',forbidden_inference_attempts=0))
    monkeypatch.setattr(study,'supervise',supervised)
    result=study.run_trial(dict(name='01',pause_seconds=0,container='owned',command=['registered']),tmp_path)
    assert result['prefix_pass'] and result['cleanup']['verified']
    assert calls==['lease','gates','start','stop','lease']
