"""A cleanup allowance cannot extend incomplete inference or hide failures."""
import json
import subprocess

import pytest

from local.experiments.run_async_panel import wait_for_case


def completed_traffic_result(passed=False):
    return dict(returncode=int(not passed), stop_returncode=0, server_after_stop={'Running':False},
        passed=passed, suite=dict(results=[dict(returncode=0, audit_returncode=int(not passed),
        video_returncode=0, audit=dict(passed=passed, integrity_pass=True))]))


@pytest.mark.parametrize('passed',[False,True])
def test_complete_traffic_mission_failure_can_continue_without_changing_verdict(passed):
    from local.experiments.run_async_panel import completed_traffic_attempt
    result = completed_traffic_result(passed)
    before = json.dumps(result,sort_keys=True)
    assert completed_traffic_attempt(result)
    assert json.dumps(result,sort_keys=True) == before


@pytest.mark.parametrize('failure',['timeout','capture','integrity','audit','video','cleanup','running','missing'])
def test_incomplete_or_unverified_traffic_attempt_stops_the_batch(failure):
    from local.experiments.run_async_panel import completed_traffic_attempt
    result = completed_traffic_result()
    capture = result['suite']['results'][0]
    if failure == 'timeout': result['timed_out'] = True
    elif failure == 'capture': capture['returncode'] = -6
    elif failure == 'integrity': capture['audit']['integrity_pass'] = False
    elif failure == 'audit': capture['audit_returncode'] = 2
    elif failure == 'video': capture['video_returncode'] = 1
    elif failure == 'cleanup': result['stop_returncode'] = 1
    elif failure == 'running': result['server_after_stop']['Running'] = True
    elif failure == 'missing': capture.pop('audit')
    assert not completed_traffic_attempt(result)


class DeadlineWorker:
    def __init__(self, finish_cleanup=True):
        self.waits=[]
        self.finish_cleanup=finish_cleanup

    def wait(self, timeout):
        self.waits.append(timeout)
        if len(self.waits)==1 or not self.finish_cleanup:
            raise subprocess.TimeoutExpired('owned-case',timeout)
        return 0


@pytest.mark.parametrize('record',[None,{},dict(returncode=0,audit_returncode=0)])
def test_incomplete_case_keeps_original_deadline(tmp_path,record):
    path=tmp_path/'status.json'
    if record is not None:
        path.write_text(json.dumps(dict(results=[record])))
    worker=DeadlineWorker();result={}
    with pytest.raises(subprocess.TimeoutExpired):
        wait_for_case(worker,path,result)
    assert worker.waits==[600] and 'cleanup_grace_seconds' not in result


@pytest.mark.parametrize('driving_pass',[False,True])
def test_completed_case_receives_only_bounded_cleanup_time(tmp_path,driving_pass):
    path=tmp_path/'status.json'
    status=dict(results=[dict(returncode=0,audit_returncode=0,video_returncode=0,
                              audit=dict(passed=driving_pass))])
    path.write_text(json.dumps(status))
    worker=DeadlineWorker();result={}
    assert wait_for_case(worker,path,result)==0
    assert worker.waits==[600,40] and result['cleanup_grace_seconds']==40
    assert json.loads(path.read_text())==status


def test_cleanup_itself_cannot_wait_indefinitely(tmp_path):
    path=tmp_path/'status.json'
    path.write_text(json.dumps(dict(results=[dict(returncode=0,audit_returncode=0,video_returncode=0)])))
    worker=DeadlineWorker(finish_cleanup=False)
    with pytest.raises(subprocess.TimeoutExpired):
        wait_for_case(worker,path,{})
    assert worker.waits==[600,40]


@pytest.fixture
def startup_case(tmp_path):
    case=tmp_path/'04-curve-left';case.mkdir()
    (case/'04-curve-left-route.log').write_text('CARLA server 0.9.16, client 0.9.16\nModel allocated\nSetting up cameras...\n')
    capture=case/'04-curve-left-route';capture.mkdir()
    (capture/'async-events.jsonl').write_text('')
    return case,capture


def crash_state():
    return dict(Status='exited',Running=False,ExitCode=139,OOMKilled=False)


@pytest.mark.parametrize('artifact',[
    'frame_000.json','control_000.json','proposal_000_00.json',
    'async_request_000.json','async-events.jsonl'])
def test_no_startup_retry_after_any_inference_or_control(startup_case,artifact):
    from local.experiments import run_async_panel as panel
    case,capture=startup_case
    (capture/artifact).write_text('{}\n')
    result=dict(startup_failure={'reason':'crash'},stop_returncode=0,server_after_stop=crash_state())
    assert not panel.no_driving_started(case)
    assert not panel.retry_startup_failure(result,case,0)


@pytest.mark.parametrize('state',[
    dict(Status='running',Running=True,ExitCode=0,OOMKilled=False),
    dict(Status='exited',Running=False,ExitCode=0,OOMKilled=False),
    dict(Status='exited',Running=False,ExitCode=137,OOMKilled=True),
    dict(Status='exited',Running=False,ExitCode=139,OOMKilled=True)])
def test_only_confirmed_preinference_signal11_qualifies(startup_case,monkeypatch,state):
    from local.experiments import run_async_panel as panel
    case,_=startup_case
    monkeypatch.setattr(panel.subprocess,'check_output',lambda *a,**k:json.dumps(state))
    assert panel.confirmed_startup_crash(case/'status.json','owned') is None


def test_startup_crash_is_detected_before_full_deadline(startup_case,monkeypatch):
    from local.experiments import run_async_panel as panel
    case,_=startup_case
    monkeypatch.setattr(panel.subprocess,'check_output',lambda *a,**k:json.dumps(crash_state()))
    worker=DeadlineWorker();result={}
    with pytest.raises(panel.SimulatorStartupCrash):
        panel.wait_for_case(worker,case/'status.json',result,container='owned')
    assert worker.waits==[5]
    assert result['startup_failure']['container_state']==crash_state()
    assert panel.retry_startup_failure({**result,'stop_returncode':0,'server_after_stop':crash_state()},case,0)
    assert not panel.retry_startup_failure({**result,'stop_returncode':0,'server_after_stop':crash_state()},case,1)


def test_no_restart_for_a_driving_failure_or_failed_cleanup(startup_case):
    from local.experiments import run_async_panel as panel
    case,_=startup_case
    assert not panel.retry_startup_failure(dict(passed=False,stop_returncode=0,server_after_stop=crash_state()),case,0)
    assert not panel.retry_startup_failure(dict(startup_failure={'reason':'crash'},stop_returncode=1,server_after_stop=crash_state()),case,0)
    assert not panel.retry_startup_failure(dict(startup_failure={'reason':'crash'},stop_returncode=0,server_after_stop={'Running':True}),case,0)


def test_successful_server_connection_required_and_sensor_data_rechecked(startup_case,monkeypatch):
    from local.experiments import run_async_panel as panel
    case,capture=startup_case
    monkeypatch.setattr(panel.subprocess,'check_output',lambda *a,**k:json.dumps(crash_state()))
    log=case/'04-curve-left-route.log';log.write_text('Loading model\n')
    assert panel.confirmed_startup_crash(case/'status.json','owned') is None
    log.write_text('CARLA server 0.9.16, client 0.9.16\nSetting up cameras...\n')
    assert panel.confirmed_startup_crash(case/'status.json','owned') is not None
    (capture/'async-events.jsonl').write_text('{"event":"submitted"}\n')
    assert panel.confirmed_startup_crash(case/'status.json','owned') is None


def test_failed_startup_archive_preserves_bytes_cost_and_cannot_overwrite(startup_case):
    from local.experiments import run_async_panel as panel
    case,capture=startup_case;root=case.parent
    raw=b'original failure bytes\x00\xff';(capture/'evidence.bin').write_bytes(raw)
    (root/(case.name+'.log')).write_text('supervisor failure')
    result=dict(passed=False,end_to_end_seconds=61.25,startup_failure={'reason':'crash'})
    archived=panel.archive_startup_failure(root,case.name,result)
    destination=root/'startup-failures'/case.name/'attempt-1'
    assert (destination/(case.name+'-route')/'evidence.bin').read_bytes()==raw
    assert (destination/'supervisor.log').read_text()=='supervisor failure'
    assert archived['end_to_end_seconds']==61.25 and archived['passed'] is False
    assert json.loads((destination/'startup-failure.json').read_text())==archived
    assert not case.exists()
    with pytest.raises(FileExistsError):panel.archive_startup_failure(root,case.name,result)


@pytest.mark.parametrize('complete',[False,True])
@pytest.mark.parametrize('work_timeout',[600,900])
def test_watchdog_does_not_extend_active_work_deadline(tmp_path,monkeypatch,complete,work_timeout):
    from local.experiments import run_async_panel as panel
    clock=[0.]
    monkeypatch.setattr(panel.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(panel,'confirmed_startup_crash',lambda *a:None)
    path=tmp_path/'status.json'
    if complete:path.write_text(json.dumps(dict(results=[dict(returncode=0,audit_returncode=0,video_returncode=0)])))
    class Worker:
        waits=[]
        def wait(self,timeout):
            self.waits.append(timeout);clock[0]+=timeout
            if timeout==40:return 0
            raise subprocess.TimeoutExpired('owned-case',timeout)
    worker=Worker();result={}
    if complete:
        assert panel.wait_for_case(worker,path,result,container='owned',work_timeout_s=work_timeout)==0
        assert worker.waits==[5]*(work_timeout//5)+[40]
        assert result['cleanup_grace_seconds']==40
    else:
        with pytest.raises(subprocess.TimeoutExpired):panel.wait_for_case(worker,path,result,container='owned',work_timeout_s=work_timeout)
        assert worker.waits==[5]*(work_timeout//5) and not result


@pytest.mark.parametrize('outcomes',[
    ['startup','pass'],['driving_failure'],['startup','startup']])
@pytest.mark.parametrize('work_timeout',[600,900])
@pytest.mark.parametrize('full_inputs',[False,True])
def test_panel_retains_startup_attempts_counts_all_time_and_never_retries_driving(tmp_path,monkeypatch,outcomes,work_timeout,full_inputs):
    import contextlib
    from types import SimpleNamespace
    from local.experiments import run_async_panel as panel
    root=tmp_path/'repo';presets=root/'local/experiments/presets';presets.mkdir(parents=True)
    (root/'module').mkdir();(root/'module/unchanged.py').write_text('# unchanged driving source\n')
    (presets/'ten-driving-scenes.json').write_text(json.dumps(dict(scenarios=[dict(name='04-curve-left')]*10)))
    monkeypatch.setattr(panel,'__file__',str(root/'local/experiments/run_async_panel.py'))
    output=tmp_path/'out'
    monkeypatch.setattr(panel.sys,'argv',['run_async_panel','--output',str(output),'--container','owned','--cases','4','--case-work-timeout-s',str(work_timeout)]+(['--capture-all-native-inputs'] if full_inputs else []))
    monkeypatch.setenv('CARLAMAYO_MODEL_GPUS','model');monkeypatch.setenv('CARLAMAYO_RENDER_GPU','renderer')
    monkeypatch.setattr(panel,'acquire',lambda *a:contextlib.nullcontext())
    monkeypatch.setattr(panel,'available',lambda *a:[])
    monkeypatch.setattr(panel.shutil,'disk_usage',lambda *a:SimpleNamespace(free=10*1024**3))
    clock=[0.];workers=[]
    monkeypatch.setattr(panel.time,'monotonic',lambda:clock[0])
    class Worker:
        returncode=None
        def __init__(self,command,**kwargs):
            assert ('--capture-all-native-inputs' in command) is full_inputs
            self.directory=panel.Path(command[command.index('--output')+1]);self.directory.mkdir()
            (self.directory/'04-curve-left-route.log').write_text('CARLA server 0.9.16, client 0.9.16\nSetting up cameras...\n')
            capture=self.directory/'04-curve-left-route';capture.mkdir()
            (capture/'original-evidence.txt').write_text('preserve original failed bytes')
            workers.append(self)
    monkeypatch.setattr(panel.subprocess,'Popen',Worker)
    def finish(worker,path,result,**kwargs):
        assert kwargs['work_timeout_s']==work_timeout
        outcome=outcomes[len(workers)-1];clock[0]+=10 if len(workers)==1 else 25
        if outcome=='startup':
            panel.save(path,dict(results=[],passed=False))
            result['startup_failure']=dict(reason='startup',container_state=crash_state())
            raise panel.SimulatorStartupCrash('startup')
        passed=outcome=='pass'
        capture=worker.directory/'04-curve-left-route';(capture/'frame_000.json').write_text('{}')
        panel.save(path,dict(results=[dict(returncode=0,audit_returncode=0,video_returncode=0,audit=dict(passed=passed))],passed=passed))
        worker.returncode=0 if passed else 1
        return worker.returncode
    monkeypatch.setattr(panel,'wait_for_case',finish)
    def stop(worker):
        if worker.returncode is None:worker.returncode=-2
    monkeypatch.setattr(panel,'stop_worker',stop)
    def run(command,**kwargs):
        if command[0]=='bash':clock[0]+=2
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(panel.subprocess,'run',run)
    monkeypatch.setattr(panel,'container_state',lambda *a:crash_state())
    monkeypatch.setattr(panel,'confirmed_startup_crash',lambda *a:None)
    if outcomes[-1]=='pass':panel.main()
    else:
        with pytest.raises(SystemExit) as error:panel.main()
        assert error.value.code==1
    assert json.loads((output/'protocol.json').read_text())['per_case_timeout_s']==work_timeout
    assert json.loads((output/'protocol.json').read_text())['capture_all_native_inputs'] is full_inputs
    status=json.loads((output/'status.json').read_text());result=status['results'][0]
    assert len(workers)==len(outcomes)
    assert result['passed'] is (outcomes[-1]=='pass')
    if len(outcomes)==2:
        assert result['startup_retry_count']==1
        assert result['end_to_end_seconds']==39
        assert result['attempt_end_to_end_seconds']==27
        assert result['startup_retry_extra_seconds']==12
        assert result['startup_attempts'][0]['end_to_end_seconds']==12
        archived=output/'startup-failures/04-curve-left/attempt-1/04-curve-left-route/original-evidence.txt'
        assert archived.read_text()=='preserve original failed bytes'
        assert len(status['startup_failures'])==1
    else:
        assert result['end_to_end_seconds']==12 and result['startup_retry_count']==0
        assert not status.get('startup_failures')
    assert (root/'module/unchanged.py').read_text()=='# unchanged driving source\n'


@pytest.mark.parametrize('stage',['','Model allocated: 20.64 GiB\n','Setting up cameras...\n'])
def test_crash_recovery_is_independent_of_camera_log_stage(startup_case,monkeypatch,stage):
    from local.experiments import run_async_panel as panel
    case,_=startup_case
    (case/'04-curve-left-route.log').write_text('CARLA server 0.9.16, client 0.9.16\n'+stage)
    monkeypatch.setattr(panel.subprocess,'check_output',lambda *a,**k:json.dumps(crash_state()))
    assert panel.confirmed_startup_crash(case/'status.json','owned') is not None


@pytest.mark.parametrize('commitment,base,rest',[
    (float('nan'),1.,None),(float('inf'),1.,None),(.6,1.,None),(.15,1.,None),
    (.5,None,None),(.5,3.,None),(.5,1.,2.),(True,1.,None),(0.,1.,None)])
def test_native_motion_commitment_rejects_unbounded_or_conflicting_settings(commitment,base,rest):
    from local.experiments.run_projection_suite import validate_motion_commitment
    with pytest.raises(ValueError):validate_motion_commitment(commitment,base,rest)


def test_native_motion_commitment_defaults_off_and_accepts_bounded_duration():
    from local.experiments.run_projection_suite import validate_motion_commitment
    validate_motion_commitment(None,None)
    validate_motion_commitment(.5,1.)
    validate_motion_commitment(.1,2.9)


@pytest.mark.parametrize('value,base,fine,signal_only',[
    (float('nan'),1.5,.1,True),(float('inf'),1.5,.1,True),(True,1.5,.1,True),
    (0.,1.5,.1,True),(1.05,1.5,.1,True),(2.,1.5,.1,True),(1.,None,.1,True),
    (1.,1.5,None,True),(1.,1.5,1.2,True),(1.,1.5,.1,False),
    (1.,float('inf'),.1,True),(1.,float('nan'),.1,True),(1.,True,.1,True),
    (1.,1.5,float('nan'),True),(1.,1.5,float('inf'),True),(1.,1.5,True,True),
    (1.,1.5,.1,'yes')])
def test_separate_deceleration_interval_rejects_invalid_settings(value,base,fine,signal_only):
    from local.experiments.run_projection_suite import validate_deceleration_interval
    with pytest.raises(ValueError):validate_deceleration_interval(value,base,fine,signal_only)


def test_separate_deceleration_interval_defaults_off_and_accepts_bounded_update():
    from local.experiments.run_projection_suite import validate_deceleration_interval
    validate_deceleration_interval(None,None,None,False)
    validate_deceleration_interval(1.,1.5,.1,True)
    validate_deceleration_interval(.1,1.5,.1,True)
