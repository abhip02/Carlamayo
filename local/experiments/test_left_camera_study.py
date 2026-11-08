"""CPU-only checks of isolation, the experimental factor, and stopping rules."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from local.experiments import left_camera_study as study


@pytest.mark.parametrize('pause', [0,1])
def test_only_pause_occurs_after_camera_setup(pause):
    calls = []
    def original(obj):
        calls.append('original'); obj.sensors = {'front':None}; return 123
    def event(name, **fields): calls.append(name)
    def sleep(seconds): calls.append(('sleep',seconds))
    wrapped = study.wrap_camera_setup(original, pause, event, sleep)
    assert wrapped(SimpleNamespace()) == 123
    expected = ['camera_setup_begin','original','camera_setup_end']
    if pause:
        expected += ['pause_begin',('sleep',1),'pause_end']
    assert calls == expected


def test_failed_camera_setup_does_not_pause_or_claim_readiness():
    calls = []
    def fail(obj): raise RuntimeError('camera spawn failed')
    wrapped = study.wrap_camera_setup(fail,1,lambda name,**kw:calls.append(name),lambda n:calls.append(n))
    with pytest.raises(RuntimeError): wrapped(None)
    assert calls == ['camera_setup_begin']


def protocol(tmp_path):
    study.write(tmp_path/'config.json', {'fixture':'config'})
    value = dict(config_sha256=study.digest(tmp_path/'config.json'), source_sha256={},
        render_gpu=study.RENDER_GPU, order=list(study.ORDER), trials=[])
    for index,pause in enumerate(study.ORDER):
        container = 'owned-'+str(index)
        value['trials'].append(dict(name=str(index),pause_seconds=pause,container=container,
                                    start_command=study.start_command(container)))
    study.write(tmp_path/'protocol.json', value)
    return value


@pytest.mark.parametrize('problem', ['none','camera_failure','cleanup_failure','resource_refusal','interrupt','bad_result'])
def test_planned_replicates_continue_after_camera_failure_but_not_cleanup_or_resource_errors(monkeypatch,tmp_path,problem):
    protocol(tmp_path); calls=[]
    def run_one(trial,config,output):
        calls.append(trial['pause_seconds'])
        return dict(state='completed',camera_pass=problem=='none',
            cleanup={'verified':problem!='cleanup_failure'},server_start_attempted=problem!='resource_refusal',
            result_error='bad json' if problem=='bad_result' else None,
            error='InterruptedError()' if problem=='interrupt' else '')
    monkeypatch.setattr(study,'run_one',run_one)
    if problem in ('none','camera_failure'):
        study.run_study(tmp_path)
        assert calls == [0,1,1,0,0,1]
    else:
        with pytest.raises(SystemExit): study.run_study(tmp_path)
        assert len(calls)==1
    result=study.read(tmp_path/'study-status.json')
    assert result['trials'][0]['camera_pass'] == (problem=='none')
    with pytest.raises(ValueError,match='Never overwrite'): study.run_study(tmp_path)


def test_changed_source_stops_before_any_process(monkeypatch,tmp_path):
    value=protocol(tmp_path)
    value['source_sha256']={str(Path(study.__file__).relative_to(study.ROOT)):'wrong'}
    study.write(tmp_path/'protocol.json',value)
    monkeypatch.setattr(study,'run_one',lambda *a:pytest.fail('Must not launch'))
    with pytest.raises(ValueError,match='Frozen source'): study.run_study(tmp_path)


def test_logged_start_preserves_core_container_settings():
    cmd=study.start_command('fresh-owned')
    assert cmd[cmd.index('--device')+1]=='nvidia.com/gpu='+study.RENDER_GPU
    assert '--network=host' in cmd and '--shm-size=2g' in cmd
    assert cmd[cmd.index('-e')+1]=='NVIDIA_VISIBLE_DEVICES=void'
    assert cmd[cmd.index(study.IMAGE)+1:] == ['bash','CarlaUE4.sh','-RenderOffScreen',
        '-nosound','-carla-rpc-port=2000','-StdOut','-FullStdOutLogOutput']
    assert 'max-size=10m' in cmd and 'max-file=1' in cmd


def test_trial_wraps_only_exact_child_and_server_commands_and_restores_module(monkeypatch,tmp_path):
    trial=dict(container='owned',pause_seconds=1,start_command=study.start_command('owned'))
    original=study.smoke.subprocess
    seen=[]
    def popen(command,**kwargs):
        assert command[2:4]==['local.experiments.left_camera_study','child']
        assert command[-2:]==['--pause','1']
        assert kwargs['env']['CUDA_VISIBLE_DEVICES']==''
        seen.append('child');return SimpleNamespace(pid=123)
    monkeypatch.setattr(study.subprocess,'Popen',popen)
    monkeypatch.setattr(study.guard,'run_server_helper',lambda cmd,kw:seen.append(cmd))
    monkeypatch.setattr(study,'cleanup_snapshot',lambda *a:dict(verified=True))
    def diagnostic(args,status,env):
        helper=['bash',str(study.ROOT/'local/experiments/projected_server.sh')]
        study.smoke.subprocess.run(helper+['start'],timeout=30)
        expected=[study.sys.executable,'-m','local.experiments.camera_startup_smoke','--child',
            '--config',str(args.config.resolve()),'--output',str(args.output.resolve())]
        study.smoke.subprocess.Popen(expected,env=env)
        with pytest.raises(ValueError,match='Unexpected child'):
            study.smoke.subprocess.Popen(['unexpected'])
        study.smoke.subprocess.run(helper+['stop'],timeout=25)
        status.update(returncode=0,server_stop_returncode=0)
        study.write(args.output/'camera-result.json',dict(passed=True,
            last_image_shape=[4,1080,1920,3],retained_history_frames=4))
        study.write(args.output/'handover.json',dict(actual={'speed':3}))
    monkeypatch.setattr(study.smoke,'run_diagnostic',diagnostic)
    result=study.run_one(trial,tmp_path/'config.json',tmp_path/'out')
    assert seen[0]==trial['start_command'] and seen[1]=='child' and seen[2][-1]=='stop'
    assert result['camera_pass'] and result['cleanup']['verified']
    assert study.smoke.subprocess is original


def test_module_restored_after_interruption_and_cleanup_recorded(monkeypatch,tmp_path):
    original=study.smoke.subprocess
    def interrupted(*args):raise InterruptedError('parent')
    monkeypatch.setattr(study.smoke,'run_diagnostic',interrupted)
    monkeypatch.setattr(study,'cleanup_snapshot',lambda *a:dict(verified=False))
    result=study.run_one(dict(container='owned',pause_seconds=0,start_command=[]),tmp_path/'config',tmp_path/'out')
    assert not result['camera_pass'] and not result['cleanup']['verified']
    assert 'InterruptedError' in result['error'] and study.smoke.subprocess is original


def test_child_rejects_visible_cuda_before_imports(monkeypatch,tmp_path):
    monkeypatch.setenv('CUDA_VISIBLE_DEVICES','GPU-model')
    with pytest.raises(ValueError,match='hidden CUDA'):
        study.camera_child(tmp_path/'config',tmp_path,0)
