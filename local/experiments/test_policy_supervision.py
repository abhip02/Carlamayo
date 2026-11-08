import itertools
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

from local.experiments import supervise_nf4_policy as guard


class Child:
    pid = 12345
    returncode = None

    def __init__(self, kind):
        self.kind = kind

    def poll(self):
        return self.returncode

    def wait(self, timeout):
        assert timeout <= 5
        if self.returncode is not None:
            return self.returncode
        if self.kind == 'normal':
            self.returncode = 0
            return 0
        if self.kind == 'interrupted':
            raise InterruptedError('parent interrupted')
        raise subprocess.TimeoutExpired('owned-child', timeout)

    def terminate(self):
        if self.kind != 'stubborn_child':
            self.returncode = -15

    def kill(self):
        self.returncode = -9


@pytest.mark.parametrize('kind', ['normal', 'dead_server', 'probe_timeout',
                                 'capture_timeout', 'interrupted', 'stubborn_child'])
def test_capture_supervision_never_leaves_owned_child_running(monkeypatch, kind):
    child = Child(kind)
    monkeypatch.setattr(guard.subprocess, 'Popen', lambda *a, **k:child)
    clock = itertools.count(0, 2000 if kind == 'capture_timeout' else 1)
    monkeypatch.setattr(guard.time, 'monotonic', lambda:next(clock))
    probes = []
    def probe(command, **kwargs):
        assert command[:2] == ['docker', 'inspect'] and kwargs['timeout'] == 5
        probes.append(command)
        if kind == 'probe_timeout':
            raise subprocess.TimeoutExpired(command, 5)
        return SimpleNamespace(stdout='{"Running":false,"ExitCode":139}')
    monkeypatch.setattr(guard.subprocess, 'run', probe)
    state = {}
    if kind in ('capture_timeout', 'interrupted'):
        with pytest.raises(subprocess.TimeoutExpired if kind=='capture_timeout' else InterruptedError):
            guard.guarded_capture(['registered-capture'], {'timeout':1800}, 'owned', state, lambda:None)
    else:
        result = guard.guarded_capture(['registered-capture'], {'timeout':1800}, 'owned', state, lambda:None)
        assert result.returncode == (0 if kind=='normal' else -9 if kind=='stubborn_child' else -15)
    assert child.returncode is not None
    if kind == 'probe_timeout':
        assert len(probes) == 3 and state['abort_reason']=='three_failed_server_probes'
    if kind == 'dead_server':
        assert len(probes) == 1 and state['abort_reason']=='owned_server_exited'


def test_memory_probe_keeps_original_thresholds_and_is_bounded(monkeypatch):
    def probe(command, **kwargs):
        assert kwargs['timeout'] == 5
        return 'GPU-model, 10240\nGPU-render, 10036\n'
    monkeypatch.setattr(guard.subprocess, 'check_output', probe)
    assert guard.bounded_memory(['GPU-model','GPU-render'], [10.,9.8])['GPU-model']==10.
    with pytest.raises(RuntimeError, match='Insufficient'):
        guard.bounded_memory(['GPU-model'], [10.1])


def test_parent_sigterm_records_error_cleans_child_and_restores_frozen_driver(monkeypatch, tmp_path):
    from local.experiments import run_nf4_pilot as driver
    command = ['python','-m','local.experiments.nf4_policy_pair','capture']
    manifest = tmp_path/'jobs.json'
    manifest.write_text(json.dumps(dict(jobs=[dict(name='09-t-junction-nf4-cap2',command=command)],
        execution_assignment=dict(model_gpu='GPU-model',renderer_gpu='GPU-render'))))
    child = Child('dead_server')
    original_wait = child.wait
    def wait(timeout):
        if child.returncode is None:
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
        return original_wait(timeout)
    child.wait = wait
    monkeypatch.setattr(guard.subprocess, 'Popen', lambda *a,**k:child)
    monkeypatch.setattr(guard.sys, 'argv', ['supervisor',str(manifest),'--model-gpu','GPU-model',
        '--render-gpu','GPU-render','--container','owned'])
    cleanup = []
    def driver_main():
        try:
            driver.subprocess.run(command, timeout=1800)
        finally:
            cleanup.append(True)
    monkeypatch.setattr(driver, 'main', driver_main)
    old_run, old_memory, old_handler = driver.subprocess, driver.check_memory, signal.getsignal(signal.SIGTERM)
    with pytest.raises(InterruptedError, match='SIGTERM'):
        guard.main()
    assert child.returncode == -15 and cleanup == [True]
    assert driver.subprocess is old_run and driver.check_memory is old_memory
    assert signal.getsignal(signal.SIGTERM) == old_handler
    assert json.loads((tmp_path/'supervision.json').read_text())['state']=='driver_error'


def test_exact_frozen_command_is_intercepted_and_normal_completion_preserved(monkeypatch, tmp_path):
    from local.experiments import run_nf4_pilot as driver
    command = ['python','-m','local.experiments.nf4_policy_pair','capture']
    manifest = tmp_path/'jobs.json'
    manifest.write_text(json.dumps(dict(jobs=[dict(name='09-t-junction-nf4-cap2',command=command)],
        execution_assignment=dict(model_gpu='GPU-model',renderer_gpu='GPU-render'))))
    monkeypatch.setattr(guard.subprocess, 'Popen', lambda *a,**k:Child('normal'))
    monkeypatch.setattr(guard.sys, 'argv', ['supervisor',str(manifest),'--model-gpu','GPU-model',
        '--render-gpu','GPU-render','--container','owned'])
    def driver_main():
        result = driver.subprocess.run(list(command), timeout=1800)
        assert result.returncode == 0
        (tmp_path/'nf4-status.json').write_text(json.dumps(dict(state='completed',server_stop_returncode=0,
            results=[dict(name='09-t-junction-nf4-cap2',returncode=0,audit_returncode=0)])))
    monkeypatch.setattr(driver, 'main', driver_main)
    guard.main()
    status = json.loads((tmp_path/'supervision.json').read_text())
    assert status['state']=='capture_finished' and status['capture_pid']==12345
    assert status['results'][0]['returncode']==0


def test_helper_normal_completion_and_checked_error():
    assert guard.run_server_helper([sys.executable,'-c','pass'], {'timeout':5,'check':True}).returncode==0
    with pytest.raises(subprocess.CalledProcessError):
        guard.run_server_helper([sys.executable,'-c','raise SystemExit(2)'], {'timeout':5,'check':True})


def test_helper_timeout_kills_its_descendant_too(tmp_path):
    # Real CPU-only process tree; no Docker, server or GPU invocation.
    marker = tmp_path/'pids.json'
    code = ('import subprocess,sys,time,os,json; from pathlib import Path; '
            'child=subprocess.Popen([sys.executable,"-c","import time; time.sleep(60) # "+sys.argv[1]]); '
            'Path(sys.argv[1]).write_text(json.dumps([os.getpid(),child.pid])); time.sleep(60)')
    try:
        with pytest.raises(subprocess.TimeoutExpired):
            guard.run_server_helper([sys.executable,'-c',code,str(marker)], {'timeout':1})
        pids = json.loads(marker.read_text())
        for _ in range(50):
            running = [pid for pid in pids if (p:=Path(f'/proc/{pid}/stat')).exists()
                       and p.read_text().split()[2] != 'Z']
            if not running:
                break
            time.sleep(.02)
        assert not running
    finally:
        if marker.exists():
            for pid in json.loads(marker.read_text()):
                try:
                    # Do not signal a PID that exited and was reused.
                    if str(marker).encode() in Path(f'/proc/{pid}/cmdline').read_bytes():
                        os.kill(pid, signal.SIGKILL)
                except (ProcessLookupError, FileNotFoundError):
                    pass
