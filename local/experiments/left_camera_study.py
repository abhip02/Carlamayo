"""Six prospectively ordered, model-free camera startup diagnostics.

This harness leaves every frozen driving source untouched. The sole experimental
factor is a one-second pause after camera setup, before acknowledged pre-roll.
Logging and instrumentation are identical in both arms. No driving is evaluated.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
from types import SimpleNamespace

from local.experiments import camera_startup_smoke as smoke
from local.experiments import supervise_nf4_policy as guard

ROOT = Path(__file__).resolve().parents[2]
RENDER_GPU = 'GPU-fd2c05bc-4b86-6451-49ae-c9e3f7f79580'
IMAGE = 'carlasim/carla@sha256:aaf1df22702780ece072069e23d03c4879b002ae028c79744b09c4c7ddbae953'
ORDER = (0, 1, 1, 0, 0, 1)


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def now():
    return datetime.now(timezone.utc).isoformat()


def start_command(container):
    # Same image, CDI device, labels, network, shm, environment and CARLA options
    # as projected_server.sh. Only per-container log rotation differs.
    return ['docker', 'run', '-d', '--name', container,
        '--label', f'apomalap.carlamayo.root={ROOT}',
        '--label', f'apomalap.carlamayo.render_gpu={RENDER_GPU}',
        '--device', f'nvidia.com/gpu={RENDER_GPU}', '--network=host', '--shm-size=2g',
        '--log-driver', 'json-file', '--log-opt', 'max-size=10m', '--log-opt', 'max-file=1',
        '-e', 'NVIDIA_VISIBLE_DEVICES=void', IMAGE,
        'bash', 'CarlaUE4.sh', '-RenderOffScreen', '-nosound', '-carla-rpc-port=2000']


def prepare(config, frozen_manifest, output):
    frozen = read(frozen_manifest)
    for name, expected in frozen['source_sha256'].items():
        if digest(ROOT/name) != expected:
            raise ValueError('Frozen source changed: ' + name)
    source = read(config)
    if source['instruction'] != 'Turn left at the intersection.' or len(source['rollouts']) != 1:
        raise ValueError('Use the failed left capture config unchanged')
    output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(config, output/'config.json')
    sources = {str(p.relative_to(ROOT)):digest(p) for folder in ('module', 'local')
               for p in (ROOT/folder).rglob('*.py')}
    sources['local/experiments/projected_server.sh'] = digest(ROOT/'local/experiments/projected_server.sh')
    trials = []
    for number, pause in enumerate(ORDER, 1):
        name = f'{number:02d}-' + ('pause' if pause else 'original')
        container = 'apomalap-carlamayo-' + output.name + '-' + name
        trials.append(dict(name=name, pause_seconds=pause, container=container,
                           start_command=start_command(container)))
    write(output/'protocol.json', dict(registered_utc=now(), state='prepared_not_executed',
        original_config=str(config.resolve()), config_sha256=digest(config),
        frozen_manifest=str(frozen_manifest.resolve()), frozen_manifest_sha256=digest(frozen_manifest),
        source_sha256=sources, render_gpu=RENDER_GPU, trials=trials, model_loaded=False,
        order=list(ORDER), retries=0, port=2000, gate_seconds=60, renderer_free_gib=9.8,
        child_deadline_seconds=180, server_start_deadline_seconds=60,
        docker_log_cap='10m per owned container; oldest bytes may rotate',
        expected_minutes=[12,18], conservative_maximum_minutes=42,
        limitations=['No model loading/inference or model GPU memory allocation; shared imports may import torch/adapter classes.',
            'Unmeasured model/processor loading interval omitted; no artificial delay substitutes for it.',
            'No OpenDRIVE export/route generation or controller setup; overhead and acknowledged pre-roll retained.',
            'Both arms have identical client instrumentation and Docker log rotation; shipped engine arguments are unchanged.',
            'Three observations per arm cannot establish startup reliability or a driving fix.'],
        continuation='Complete each predeclared trial once even if camera startup fails; stop on resource refusal, interruption, or unverified cleanup.',
        sources={'offscreen':'https://carla.readthedocs.io/en/0.9.16/build_docker/',
            'log_rotation':'https://docs.docker.com/engine/logging/drivers/json-file/'}))


def wrap_camera_setup(original, pause, event, sleep=time.sleep):
    def setup(self):
        event('camera_setup_begin')
        result = original(self)
        event('camera_setup_end', sensor_names=list(self.sensors))
        if pause:
            event('pause_begin', seconds=pause)
            sleep(pause)
            event('pause_end')
        return result
    return setup


def camera_child(config, output, pause):
    if os.environ.get('CUDA_VISIBLE_DEVICES') != '' or pause not in (0, 1):
        raise ValueError('Diagnostic child requires hidden CUDA and registered pause')
    # These imports reach CARLA and NumPy; no adapter.load_model is called.
    from module import carla_interface
    from local import run_intersection as runner
    from local.experiments import acknowledged_preroll as preroll
    base, initialize, connect = carla_interface.CARLAInterface, preroll.initialize_ego, runner.connect_world
    origin = time.monotonic()
    with (output/'events.jsonl').open('x') as stream:
        def event(name, **fields):
            stream.write(json.dumps(dict(event=name, utc=now(), elapsed_s=time.monotonic()-origin, **fields))+'\n')
            stream.flush()

        class ObservedInterface(base):
            setup_cameras = wrap_camera_setup(base.setup_cameras, pause, event)

            def tick(self):
                event('tick_begin')
                frame = super().tick()
                event('tick_returned', frame=int(frame))
                return frame

            def get_camera_images(self, *args, **kwargs):
                event('camera_wait_begin', frame=int(self.frame_id))
                images = super().get_camera_images(*args, **kwargs)
                event('camera_frame_ready', frame=int(self.frame_id), shape=list(images.shape))
                return images

        def observed_connect(*args):
            event('connect_begin')
            result = connect(*args)
            event('connect_end')
            return result

        def observed_initialize(interface, entry, spec):
            event('preroll_begin')
            frames, pose = initialize(interface, entry, spec)
            # The last acknowledged history state is exactly the handover used
            # by inference; reading it adds no RPC or extra world tick.
            actual = dict(interface.history_buffer[-1])
            handover = dict(requested_pose=pose, requested_speed_mps=spec['speed_mps'],
                            actual=actual, frame=int(interface.frame_id),
                            retained_history_frames=len(frames), pause_seconds=pause)
            write(output/'handover.json', handover)
            event('preroll_handover', **handover)
            return frames, pose

        carla_interface.CARLAInterface = ObservedInterface
        preroll.initialize_ego = observed_initialize
        runner.connect_world = observed_connect
        event('child_begin', pause_seconds=pause, model_loaded=False)
        try:
            smoke.camera_child(config, output)
            event('child_completed')
        except BaseException as error:
            event('child_error', error=repr(error))
            raise
        finally:
            carla_interface.CARLAInterface = base
            preroll.initialize_ego, runner.connect_world = initialize, connect


def cleanup_snapshot(container, pid):
    from local.experiments.resource_lease import lease
    result = dict(utc=now(), verified=False)
    try:
        inspected = subprocess.run(['docker', 'inspect', '--format', '{{json .}}', container],
            capture_output=True, text=True, check=True, timeout=5)
        data = json.loads(inspected.stdout)
        result['container_state'] = data['State']
        labels = data['Config']['Labels']
        result['owned_container'] = (labels.get('apomalap.carlamayo.root') == str(ROOT)
            and labels.get('apomalap.carlamayo.render_gpu') == RENDER_GPU)
        result['child_gone'] = pid is None or not Path(f'/proc/{pid}').exists()
        with socket.socket() as probe:
            probe.settimeout(1)
            result['port_closed'] = probe.connect_ex(('127.0.0.1', 2000)) != 0
        with lease([RENDER_GPU], port=2000):
            result['leases_reacquired'] = True
        result['verified'] = (result['owned_container'] and not data['State']['Running']
            and result['child_gone'] and result['port_closed'] and result['leases_reacquired'])
    except Exception as error:
        result['error'] = repr(error)
    return result


def run_one(trial, config, output):
    output.mkdir(exist_ok=False)
    status = dict(state='prepared', model_loaded=False, render_gpu=RENDER_GPU,
        container=trial['container'], headroom=[], started_utc=now(), pause_seconds=trial['pause_seconds'])
    env = dict(os.environ, CUDA_VISIBLE_DEVICES='', CARLAMAYO_RENDER_GPU=RENDER_GPU,
        CARLAMAYO_PROJECTION_CONTAINER=trial['container'], PYTHONDONTWRITEBYTECODE='1', PYTHONUNBUFFERED='1')
    args = SimpleNamespace(config=config, output=output, render_gpu=RENDER_GPU, container=trial['container'])
    helper = ['bash', str(ROOT/'local/experiments/projected_server.sh')]
    original_subprocess = smoke.subprocess

    def run(command, **kwargs):
        if command == helper+['start']:
            status['server_start_attempted'] = True
            return guard.run_server_helper(trial['start_command'], kwargs)
        if command == helper+['stop']:
            return guard.run_server_helper(command, kwargs)
        return subprocess.run(command, **kwargs)

    def popen(command, **kwargs):
        expected = [sys.executable, '-m', 'local.experiments.camera_startup_smoke', '--child',
                    '--config', str(config.resolve()), '--output', str(output.resolve())]
        if command != expected:
            raise ValueError('Unexpected child command')
        command = [sys.executable, '-m', 'local.experiments.left_camera_study', 'child',
            '--config', str(config.resolve()), '--output', str(output.resolve()),
            '--pause', str(trial['pause_seconds'])]
        child = subprocess.Popen(command, **kwargs)
        status.update(child_pid=child.pid, child_command=command)
        return child

    smoke.subprocess = SimpleNamespace(run=run, Popen=popen, check_output=subprocess.check_output,
        TimeoutExpired=subprocess.TimeoutExpired, DEVNULL=subprocess.DEVNULL, STDOUT=subprocess.STDOUT)
    try:
        smoke.run_diagnostic(args, status, env)
    except Exception as error:
        status.update(error=repr(error), state='trial_error')
    finally:
        smoke.subprocess = original_subprocess
        status['finished_utc'] = now()
        status['cleanup'] = cleanup_snapshot(trial['container'], status.get('child_pid'))
        status['cleanup']['verified'] &= status.get('server_stop_returncode') == 0
        try:
            result = read(output/'camera-result.json') if (output/'camera-result.json').exists() else {}
            handover = read(output/'handover.json') if (output/'handover.json').exists() else None
        except (OSError, ValueError) as error:
            result, handover = {}, None
            status['result_error'] = repr(error)
        status['camera_pass'] = (status.get('returncode') == 0 and result.get('passed') is True
            and result.get('last_image_shape') == [4,1080,1920,3] and result.get('retained_history_frames') == 4
            and handover is not None)
        status['handover'] = handover
        write(output/'status.json', status)
    return status


def run_study(output):
    protocol = read(output/'protocol.json')
    if digest(output/'config.json') != protocol['config_sha256']:
        raise ValueError('Config changed')
    if protocol['order'] != list(ORDER) or protocol['render_gpu'] != RENDER_GPU:
        raise ValueError('Registered assignment/order changed')
    for name, expected in protocol['source_sha256'].items():
        if digest(ROOT/name) != expected:
            raise ValueError('Frozen source changed: '+name)
    for trial, pause in zip(protocol['trials'], ORDER, strict=True):
        if trial['pause_seconds'] != pause or trial['start_command'] != start_command(trial['container']):
            raise ValueError('Registered trial command changed')
    status_path = output/'study-status.json'
    if status_path.exists() or any((output/t['name']).exists() for t in protocol['trials']):
        raise ValueError('Never overwrite or retry this study')
    status = dict(state='running', protocol_sha256=digest(output/'protocol.json'), started_utc=now(),
                  trials=[dict(name=t['name'], state='not_attempted') for t in protocol['trials']])

    def interrupted(signum, frame):
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise InterruptedError('Study interrupted; no further trials')

    old_handler = signal.signal(signal.SIGTERM, interrupted)
    write(status_path, status)
    try:
        for index, trial in enumerate(protocol['trials']):
            row = run_one(trial, output/'config.json', output/trial['name'])
            status['trials'][index] = dict(name=trial['name'], state=row['state'],
                pause_seconds=trial['pause_seconds'], camera_pass=row['camera_pass'],
                cleanup_verified=row['cleanup']['verified'], error=row.get('error'))
            write(status_path, status)
            if (not row['cleanup']['verified'] or not row.get('server_start_attempted')
                    or 'InterruptedError' in row.get('error','') or row.get('result_error')):
                status['state'] = 'stopped_before_next_trial'
                break
        else:
            status['state'] = 'completed_all_planned_trials'
    except BaseException as error:
        status.update(state='interrupted_or_error', error=repr(error))
        raise
    finally:
        status['finished_utc'] = now()
        write(status_path, status)
        signal.signal(signal.SIGTERM, old_handler)
    if status['state'] != 'completed_all_planned_trials':
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    prep = sub.add_parser('prepare')
    prep.add_argument('--config', type=Path, required=True)
    prep.add_argument('--frozen-manifest', type=Path, required=True)
    prep.add_argument('--output', type=Path, required=True)
    run = sub.add_parser('run'); run.add_argument('--output', type=Path, required=True)
    child = sub.add_parser('child')
    child.add_argument('--config', type=Path, required=True)
    child.add_argument('--output', type=Path, required=True)
    child.add_argument('--pause', type=int, choices=(0,1), required=True)
    args = parser.parse_args()
    if args.action == 'prepare':
        prepare(args.config, args.frozen_manifest, args.output.resolve())
    elif args.action == 'run':
        run_study(args.output.resolve())
    else:
        camera_child(args.config.resolve(), args.output.resolve(), args.pause)


if __name__ == '__main__':
    main()
