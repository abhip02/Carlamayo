"""Bounded, model-free reproduction of the four-camera startup path.

This is a separate infrastructure diagnostic, not a driving-policy retry.
It also spawns the configured overhead observer, as the failed capture did.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def write(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def renderer_memory(gpu, minimum):
    rows = subprocess.check_output(['nvidia-smi', '--query-gpu=uuid,memory.free',
                                    '--format=csv,noheader,nounits'], text=True, timeout=5)
    free = {parts[0].strip(): float(parts[1])/1024 for row in rows.splitlines()
            if (parts := row.split(','))}
    if free[gpu] < minimum:
        raise RuntimeError(f'Insufficient renderer headroom: {free[gpu]:.2f} GiB < {minimum:.2f}')
    return {gpu: free[gpu]}


def camera_child(config_path, output):
    # CUDA is hidden in this subprocess. No adapter/model loader is called.
    import random
    import carla
    import numpy as np
    from types import SimpleNamespace
    from local import run_intersection as runner
    from local.experiments.acknowledged_preroll import initialize_ego
    from local.experiments.frame_sync import wait_for_frame
    from module.carla_interface import CARLAInterface

    config = json.loads(config_path.read_text())
    random.seed(config.get('seed', 42))
    np.random.seed(config.get('seed', 42))
    client, world = runner.connect_world(SimpleNamespace(host='127.0.0.1', port=2000), config)
    previous = world.get_settings()
    interface = CARLAInterface()
    interface.client, interface.world = client, world
    original_tick = interface.tick
    def tick():
        frame = original_tick()
        wait_for_frame(world, frame)
        return frame
    interface.tick = tick
    overhead = None
    result = dict(passed=False, model_loaded=False, camera_order=interface.camera_order,
                  rig=interface.camera_configs, width=runner.defaults.IMG_WIDTH,
                  height=runner.defaults.IMG_HEIGHT, config=config)
    write(output/'camera-result.json', result)
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = runner.DT
        settings.substepping = True
        settings.max_substep_delta_time = .01
        settings.max_substeps = 10
        world.apply_settings(settings)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        for light in world.get_actors().filter('traffic.traffic_light'):
            light.set_state(carla.TrafficLightState.Green)
            light.freeze(True)
        if config.get('overhead', {}).get('enabled', True):
            overhead = runner.OverheadRecorder(world, config['junction_entry'], output,
                                               config.get('overhead'), label='camera smoke')
        frames, pose = initialize_ego(interface, config['junction_entry'], runner.rollout_specs(config)[0])
        result.update(passed=True, retained_history_frames=len(frames),
                      last_image_shape=list(frames[-1].shape), final_frame=interface.frame_id,
                      requested_pose=pose)
    except BaseException as error:
        result['error'] = repr(error)
        raise
    finally:
        write(output/'camera-result.json', result)
        # A dead server can stall these RPCs; the parent bounds the entire child.
        runner.destroy_ego(interface)
        if overhead is not None:
            overhead.close()
        world.apply_settings(previous)


def run_diagnostic(args, status, env):
    from local.experiments.resource_lease import lease
    server = ['bash', str(ROOT/'local/experiments/projected_server.sh')]
    child = None
    with lease([args.render_gpu], port=2000):
        # Refuse every existing container, including our own old diagnostics.
        if subprocess.run(['docker', 'container', 'inspect', args.container],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5).returncode == 0:
            raise ValueError('Choose a new container name; preserve previous attempts')
        ready_at = time.monotonic() + 60
        while True:
            free = renderer_memory(args.render_gpu, 9.8)
            status['headroom'].append(dict(unix_time=time.time(), free_gib=free))
            write(args.output/'status.json', status)
            if time.monotonic() >= ready_at:
                break
            time.sleep(min(5, max(0, ready_at-time.monotonic())))
        with socket.socket() as probe:
            probe.settimeout(1)
            if probe.connect_ex(('127.0.0.1', 2000)) == 0:
                raise RuntimeError('Port 2000 is already occupied')
        try:
            subprocess.run(server+['start'], env=env, cwd=ROOT, check=True, timeout=30)
            status.update(state='running', renderer_memory_samples=[])
            command = [sys.executable, '-m', 'local.experiments.camera_startup_smoke', '--child',
                       '--config', str(args.config.resolve()), '--output', str(args.output.resolve())]
            with (args.output/'child.log').open('w') as log:
                # A socket gate avoids spending the camera deadline on server startup.
                startup_deadline = time.monotonic()+60
                while True:
                    with socket.socket() as probe:
                        probe.settimeout(1)
                        if probe.connect_ex(('127.0.0.1', 2000)) == 0:
                            break
                    if time.monotonic() >= startup_deadline:
                        raise TimeoutError('Owned server never opened port 2000')
                    time.sleep(1)
                child = subprocess.Popen(command, env=env, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
                deadline = time.monotonic()+180
                while child.poll() is None:
                    state = json.loads(subprocess.check_output(['docker', 'inspect', '--format', '{{json .State}}', args.container], timeout=5))
                    status['container_state'] = state
                    if not state['Running']:
                        raise RuntimeError('Owned CARLA exited during camera startup')
                    if time.monotonic() >= deadline:
                        raise TimeoutError('Camera subprocess exceeded 180 seconds')
                    status['renderer_memory_samples'].append(dict(unix_time=time.time(),
                        free_gib=renderer_memory(args.render_gpu, 0)))
                    write(args.output/'status.json', status)
                    time.sleep(1)
                status.update(state='completed' if child.returncode == 0 else 'child_error', returncode=child.returncode)
        except BaseException as error:
            status.update(state='error', error=repr(error))
            raise
        finally:
            try:
                if child is not None and child.poll() is None:
                    child.terminate()
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait(timeout=5)
                with (args.output/'server.log').open('w') as log:
                    subprocess.run(['docker', 'logs', args.container], stdout=log, stderr=subprocess.STDOUT, timeout=10)
            finally:
                # This frozen helper verifies the root and GPU ownership labels.
                try:
                    status['server_stop_returncode'] = subprocess.run(server+['stop'], env=env, cwd=ROOT, timeout=25).returncode
                finally:
                    write(args.output/'status.json', status)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True, help='Recorded config.json, unchanged')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--render-gpu')
    parser.add_argument('--container')
    parser.add_argument('--child', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.child:
        camera_child(args.config, args.output)
        return
    if not args.render_gpu or not args.container:
        parser.error('--render-gpu and --container are required')
    args.output.mkdir(parents=True, exist_ok=False)
    status = dict(state='prepared', model_loaded=False, render_gpu=args.render_gpu,
                  container=args.container, child_timeout_seconds=180, probe_timeout_seconds=5, headroom=[])
    env = dict(os.environ, CUDA_VISIBLE_DEVICES='', CARLAMAYO_RENDER_GPU=args.render_gpu,
               CARLAMAYO_PROJECTION_CONTAINER=args.container, PYTHONDONTWRITEBYTECODE='1')
    write(args.output/'status.json', status)
    def interrupted(signum, frame):
        # Let bounded cleanup finish if an outer timeout sends another SIGTERM.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise InterruptedError('Diagnostic received SIGTERM')
    previous_handler = signal.signal(signal.SIGTERM, interrupted)
    try:
        run_diagnostic(args, status, env)
    except BaseException as error:
        status.update(state='blocked_before_start' if status['state']=='prepared' else 'error', error=repr(error))
        raise
    finally:
        write(args.output/'status.json', status)
        signal.signal(signal.SIGTERM, previous_handler)
    if status.get('returncode') != 0 or status.get('server_stop_returncode') != 0:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
