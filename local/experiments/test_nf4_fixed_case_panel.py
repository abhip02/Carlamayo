import json
import signal
import subprocess

import pytest

from local.experiments import nf4_fixed_case_panel as panel


def test_scene_fields_can_change_without_changing_candidate_policy():
    ref=dict(seed=52,flow_seed=51,instruction='T',validation_route='t',route_spec={'kind':'lane'},
             precision='nf4',speed_selection_attempts=2,speed_preference_mps=[2.5,5.],steering_gain=1.4)
    panel.same_policy({**ref,'seed':44,'flow_seed':43,'instruction':'left','validation_route':'left'},ref)
    for change in [{'speed_selection_attempts':3},{'speed_preference_mps':[3.,5.]},
                   {'steering_gain':1.5},{'precision':'bf16'},{'flow_temperature':.5}]:
        with pytest.raises(ValueError,match='only the declared scene'):
            panel.same_policy({**ref,**change},ref)


def test_cli_adapter_changes_only_this_invocations_case_number():
    parser=panel.case_parser(2)()
    parser.add_argument('--cases',nargs='+',type=int,choices=(9,10),default=[9,10])
    parser.add_argument('--precision',choices=('bf16','nf4'))
    assert parser.parse_args(['--cases','2','--precision','nf4']).cases==[2]
    with pytest.raises(SystemExit):parser.parse_args(['--cases','8'])
    with pytest.raises(SystemExit):parser.parse_args(['--cases','2','--precision','wrong'])


def manifest(tmp_path):
    path=tmp_path/'jobs.json'
    path.write_text(json.dumps(dict(cases=[dict(number=n,name=f'{n:02d}-case-nf4-cap2',capture=str(tmp_path/str(n)))
        for n in (2,8)],execution_assignment=dict(model_gpu='model',renderer_gpu='render'))))
    return path


def test_first_quality_failure_stops_panel_even_when_launcher_returns_normally(tmp_path):
    calls=[]
    result=panel.run_panel(manifest(tmp_path),'owned',
        execute=lambda case,*args:calls.append(case['number']),
        assess=lambda capture,name:dict(name=name,state='driving_or_audit_failure',passed=False))
    assert calls==[2] and result['state']=='stopped_after_nonpassing_case'
    assert result['results'][1]['state']=='not_attempted'
    assert 'earlier nonpassing' in result['results'][1]['reason']


def test_launcher_or_cleanup_error_cannot_be_hidden_by_a_passing_driving_audit(tmp_path):
    def broken(*args):raise TimeoutError('server cleanup timed out')
    result=panel.run_panel(manifest(tmp_path),'owned',execute=broken,
        assess=lambda capture,name:dict(name=name,state='passed',passed=True))
    assert result['results'][0]['state']=='launch_or_cleanup_error'
    assert result['results'][0]['assessed_state']=='passed' and not result['results'][0]['passed']
    assert result['results'][1]['state']=='not_attempted'


def test_two_passing_cases_run_once_in_declared_order_and_panel_cannot_repeat(tmp_path):
    path=manifest(tmp_path);calls=[]
    execute=lambda case,*args:calls.append(case['number'])
    assess=lambda capture,name:dict(name=name,state='passed',passed=True)
    result=panel.run_panel(path,'owned',execute=execute,assess=assess)
    assert calls==[2,8] and result['state']=='completed'
    with pytest.raises(ValueError,match='No repeated'):
        panel.run_panel(path,'owned',execute=execute,assess=assess)
    assert calls==[2,8]


def test_wrong_case_order_is_rejected(tmp_path):
    path=manifest(tmp_path);data=json.loads(path.read_text());data['cases'].reverse();path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='Exactly left02 then tight08'):
        panel.run_panel(path,'owned')


def test_unreadable_capture_is_recorded_and_blocks_the_second_case(tmp_path):
    calls=[]
    def broken(*args):raise ValueError('truncated capture JSON')
    result=panel.run_panel(manifest(tmp_path),'owned',
        execute=lambda case,*args:calls.append(case['number']),assess=broken)
    assert calls==[2] and result['results'][0]['state']=='assessment_error'
    assert result['state']=='stopped_after_nonpassing_case'


def test_scoped_driver_adapter_intercepts_left_capture_and_restores_on_failure(monkeypatch,tmp_path):
    from local.experiments import run_nf4_pilot as driver
    command=['python','-m','local.experiments.nf4_policy_pair','capture',
        '--output',str(tmp_path/'capture'),'--config',str(tmp_path/'scene.yaml'),'--options',str(tmp_path/'options.json')]
    path=tmp_path/'jobs.json';assignment=dict(model_gpu='GPU-model',renderer_gpu='GPU-render')
    path.write_text(json.dumps(dict(jobs=[dict(command=command)],execution_assignment=assignment)))
    case=dict(number=2,job_manifest=str(path),job_manifest_sha256=panel.policy.sha256(path))
    monkeypatch.setattr(panel.bridge,'verify_manifest',lambda *a:None)
    captured=[]
    def guarded(actual,kwargs,*args):
        captured.append(actual);assert kwargs['timeout']==1800
        return subprocess.CompletedProcess(actual,0)
    monkeypatch.setattr(panel.supervisor,'guarded_capture',guarded)
    def unexpected(*args,**kwargs):raise AssertionError('Unmocked external process')
    monkeypatch.setattr(panel.subprocess,'run',unexpected)
    def execute():
        parser=driver.argparse.ArgumentParser()
        parser.add_argument('--cases',nargs='+',type=int,choices=(9,10))
        assert parser.parse_args(['--cases','2']).cases==[2]
        driver.subprocess.run(list(command),timeout=1800)
        raise InterruptedError('case interrupted')
    monkeypatch.setattr(driver,'main',execute)
    original=(driver.subprocess,driver.check_memory,driver.argparse,panel.sys.argv,signal.getsignal(signal.SIGTERM))
    with pytest.raises(InterruptedError):panel.run_one(case,assignment,'owned')
    assert captured==[command]
    assert (driver.subprocess,driver.check_memory,driver.argparse,panel.sys.argv,signal.getsignal(signal.SIGTERM))==original
    assert json.loads((tmp_path/'supervision.json').read_text())['state']=='driver_error'
