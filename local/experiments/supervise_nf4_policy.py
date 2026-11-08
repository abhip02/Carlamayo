"""Supervise one frozen T-junction candidate; abort promptly if CARLA dies.

Only orchestration probes, deadlines and cleanup are wrapped. Frozen generation,
selection, controller, checkpoint, scenario and seed inputs remain unchanged.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace


def bounded_memory(gpus, minimums):
    rows = subprocess.check_output(['nvidia-smi', '--query-gpu=uuid,memory.free',
        '--format=csv,noheader,nounits'], text=True, timeout=5)
    free = {parts[0].strip():float(parts[1])/1024 for row in rows.splitlines()
            if (parts := row.split(','))}
    for gpu, minimum in zip(gpus, minimums, strict=True):
        if free[gpu] < minimum:
            raise RuntimeError(f'Insufficient free memory on {gpu}: {free[gpu]:.2f} GiB < {minimum:.2f}')
    return {gpu:free[gpu] for gpu in gpus}


def stop_child(child):
    if child.poll() is None:
        child.terminate()
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=5)


def run_server_helper(command, kwargs):
    """Bound the shell and its Docker client together in an owned process group."""
    kwargs = dict(kwargs)
    timeout = kwargs.pop('timeout')
    check = kwargs.pop('check', False)
    child = subprocess.Popen(command, start_new_session=True, **kwargs)
    try:
        code = child.wait(timeout=timeout)
    except BaseException:
        # Kill the whole group even if its shell exits before its Docker client.
        for signum in (signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(child.pid, signum)
            except ProcessLookupError:
                pass
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        raise
    if check and code:
        raise subprocess.CalledProcessError(code, command)
    return subprocess.CompletedProcess(command, code)


def guarded_capture(command, kwargs, container, status, save):
    kwargs = dict(kwargs)
    limit = kwargs.pop('timeout')
    if limit != 1800:
        raise ValueError('Keep the frozen 1,800-second capture limit')
    child = subprocess.Popen(command, **kwargs)
    status.update(state='capture_running', capture_pid=child.pid)
    save()
    deadline = time.monotonic()+limit
    failed_probes = 0
    try:
        while True:
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                status['abort_reason'] = 'capture_deadline'
                raise subprocess.TimeoutExpired(command, limit)
            try:
                code = child.wait(timeout=min(2, remaining))
                status.update(state='capture_finished', capture_returncode=code)
                return subprocess.CompletedProcess(command, code)
            except subprocess.TimeoutExpired:
                pass
            try:
                checked = subprocess.run(['docker', 'inspect', '--format', '{{json .State}}', container],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True, timeout=5)
                server = json.loads(checked.stdout)
                status['last_container_state'] = server
                failed_probes = 0
                if not server['Running']:
                    status['abort_reason'] = 'owned_server_exited'
                    stop_child(child)
                    status.update(state='capture_aborted', capture_returncode=child.returncode)
                    return subprocess.CompletedProcess(command, child.returncode)
            except (subprocess.SubprocessError, ValueError, KeyError) as error:
                failed_probes += 1
                status.update(consecutive_failed_probes=failed_probes, last_probe_error=repr(error))
                if failed_probes >= 3:
                    status['abort_reason'] = 'three_failed_server_probes'
                    stop_child(child)
                    status.update(state='capture_aborted', capture_returncode=child.returncode)
                    return subprocess.CompletedProcess(command, child.returncode)
            save()
    finally:
        stop_child(child)
        save()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--model-gpu', required=True)
    parser.add_argument('--render-gpu', required=True)
    parser.add_argument('--container', required=True)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    if len(manifest['jobs']) != 1 or not manifest['jobs'][0]['name'].startswith('09-'):
        parser.error('Use only the separate T-junction candidate manifest')
    if manifest['execution_assignment'] != dict(model_gpu=args.model_gpu, renderer_gpu=args.render_gpu):
        parser.error('Preserve the declared model and renderer assignments')
    expected = manifest['jobs'][0]['command']
    if expected[2:4] != ['local.experiments.nf4_policy_pair', 'capture']:
        parser.error('Expected the unchanged NF4 policy capture command')
    path = args.manifest.parent/'supervision.json'
    if path.exists():
        parser.error('Preserve previous supervisor outcomes')
    status = dict(state='prepared', manifest=str(args.manifest.resolve()),
                  model_gpu=args.model_gpu, render_gpu=args.render_gpu, container=args.container,
                  frozen_capture_seconds=1800, server_probe_seconds=5, maximum_failed_probes=3)
    def save():
        path.write_text(json.dumps(status, indent=2)+'\n')
    save()
    from local.experiments import run_nf4_pilot as driver
    original_subprocess, original_memory, original_argv = driver.subprocess, driver.check_memory, sys.argv
    def run(command, **kwargs):
        if command == expected:
            return guarded_capture(command, kwargs, args.container, status, save)
        if len(command) >= 3 and Path(command[1]).name == 'projected_server.sh':
            kwargs.setdefault('timeout', 25 if command[-1]=='stop' else 30)
            return run_server_helper(command, kwargs)
        # The remaining frozen driver call is its independent CPU audit.
        kwargs.setdefault('timeout', 120)
        return subprocess.run(command, **kwargs)
    def interrupted(signum, frame):
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise InterruptedError('Supervisor received SIGTERM')
    old_handler = signal.signal(signal.SIGTERM, interrupted)
    # Replace references only in this driver's module, not global subprocess.
    driver.subprocess = SimpleNamespace(run=run, TimeoutExpired=subprocess.TimeoutExpired, STDOUT=subprocess.STDOUT)
    driver.check_memory = bounded_memory
    sys.argv = ['run_nf4_pilot', str(args.manifest), '--precision', 'nf4', '--model-gpu', args.model_gpu,
        '--render-gpu', args.render_gpu, '--container', args.container,
        '--cases', '9', '--headroom-seconds', '60', '--wall-timeout-s', '1800']
    try:
        driver.main()
    except BaseException as error:
        status.update(state='driver_error', error=repr(error))
        raise
    finally:
        driver.subprocess, driver.check_memory, sys.argv = original_subprocess, original_memory, original_argv
        signal.signal(signal.SIGTERM, old_handler)
        driver_path = args.manifest.parent/'nf4-status.json'
        if driver_path.exists():
            result = json.loads(driver_path.read_text())
            status['driver_state'] = result.get('state')
            status['server_stop_returncode'] = result.get('server_stop_returncode')
            status['results'] = [{k:v for k,v in row.items() if k!='audit'} for row in result.get('results', [])]
        save()
    if not status.get('results') or any(row.get('returncode') != 0 or row.get('audit_returncode') != 0
                                       for row in status['results']):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
