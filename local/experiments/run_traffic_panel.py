"""Run a bounded, source-frozen traffic extension of the round-19 straight case."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time

import carla
import yaml

from local.experiments.resource_lease import acquire
from local.experiments.run_latency_pilot import stop_worker


CASES = {
    'empty-reference': dict(kind='empty', duration_s=15.),
    'stationary-lead': dict(kind='stationary_lead', duration_s=12., initial_center_distance_m=18.),
    'braking-lead': dict(kind='braking_lead', duration_s=16., initial_center_distance_m=18.,
                         lead_speed_mps=2., brake_at_s=4., traffic_seed=44),
    'moving-lead': dict(kind='moving_lead', duration_s=20., initial_center_distance_m=18.,
                        lead_speed_mps=2., traffic_seed=44),
}


def save(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def available(model, renderer):
    compute = subprocess.check_output(['nvidia-smi', '--query-compute-apps=gpu_uuid,pid',
                                       '--format=csv,noheader,nounits'], text=True)
    if any(line.split(',')[0].strip() in (model, renderer) for line in compute.splitlines()):
        raise RuntimeError('A selected GPU has an active compute process')
    result = []
    for gpu, minimum in ((model, 23000), (renderer, 10000)):
        fields = subprocess.check_output(['nvidia-smi', '-i', gpu,
            '--query-gpu=uuid,name,memory.free,memory.used', '--format=csv,noheader,nounits'], text=True).strip().split(',')
        if int(fields[2]) < minimum:
            raise RuntimeError(f'GPU {gpu} has insufficient free memory; no job displaced')
        result.append(dict(uuid=fields[0], name=fields[1].strip(), free_mib=int(fields[2]), used_mib=int(fields[3])))
    for port in (2000, 2001, 2002, 8013):
        with socket.socket() as sock:
            sock.settimeout(.3)
            if sock.connect_ex(('127.0.0.1', port)) == 0:
                raise RuntimeError(f'Port {port} is occupied')
    return result


def container_state(name):
    value = subprocess.check_output(['docker', 'inspect', '--format', '{{json .State}}', name], text=True)
    return json.loads(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cases', nargs='+', choices=CASES, default=['empty-reference', 'stationary-lead', 'braking-lead'])
    parser.add_argument('--container', required=True)
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(output)
    model, renderer = os.environ['CARLAMAYO_MODEL_GPUS'], os.environ['CARLAMAYO_RENDER_GPU']
    environment = {**os.environ, 'CARLAMAYO_PROJECTION_CONTAINER': args.container}
    manifest_path = root / 'local/experiments/presets/ten-driving-scenes.json'
    manifest = json.loads(manifest_path.read_text())
    scenario = manifest['scenarios'][0]
    sources = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
               for folder in ('module', 'local') for p in (root / folder).rglob('*.py')}
    protocol = dict(original_scene=scenario, original_manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        cases={name: CASES[name] for name in args.cases}, source_sha256=sources,
        git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
        model_precision='BF16', attention='SDPA', model_gpu=model, renderer_gpu=renderer,
        max_capture_seconds_per_case=1200,
        inherited_controller=dict(steering_mode='route', steering_gain=1.4, speed_ceiling_mps=4.5),
        sampling='One fresh native trajectory each0.1simulatedsecond; no motion preference or plan reuse. Signal VQA remains every5decisions.',
        traffic_criteria=dict(minimum_projected_vehicle_box_clearance_m=1., maximum_speed_mps=5.5,
            no_recorded_collision=True, no_recorded_lane_departure=True,
            stop_missions=dict(required_final_hold_s=2., maximum_hold_speed_mps=.2, maximum_final_gap_m=8.),
            moving_missions='Complete the original route within the declared duration'),
        resource_gate='No selected-GPU compute jobs; >=23000MiB model free, >=10000MiB renderer free, all CARLA/TM ports unused. Permits measured display/driver reservations (~450MiB), unlike the older fixed256MiB idle cutoff.')
    with acquire(root.parent / '.carlamayo-resource-locks', model, renderer):
        resources = available(model, renderer)
        if shutil.disk_usage(root).free < 6 * 1024**3:
            raise RuntimeError('Need at least6GiB free for bounded recordings')
        if args.check_only:
            print(json.dumps(dict(ready=True, resources=resources, protocol=protocol), indent=2))
            return
        output.mkdir(parents=True)
        save(output / 'protocol.json', protocol)
        status = dict(state='preparing', resources=resources, results=[], unattempted=list(args.cases))
        save(output / 'status.json', status)
        for name in args.cases:
            for path, digest in sources.items():
                if hashlib.sha256((root / path).read_bytes()).hexdigest() != digest:
                    raise RuntimeError(f'Frozen source changed: {path}')
            available(model, renderer)
            spec = CASES[name]
            config = yaml.safe_load((root / manifest['base_config']).read_text())
            config.update(duration_s=spec['duration_s'], junction_entry=copy.deepcopy(scenario['junction_entry']),
                          seed=scenario['seed'], instruction=scenario['instruction'])
            config['rollouts'] = [{key: scenario[key] for key in
                ('name', 'distance_before_intersection_m', 'lateral_offset_m', 'speed_mps', 'heading_offset_deg')}]
            options = dict(steering_mode='route', num_samples=1, seed=scenario['seed'], flow_seed=scenario['flow_seed'],
                flow_temperature=1., guard_trajectory_format=True, acknowledged_preroll=True,
                acknowledged_controls=True, wait_for_snapshot=True, verify_signal=True,
                signal_check_interval=5, validation_routes='auto', validation_route='straight',
                route_spec=scenario['route_spec'], instruction=scenario['instruction'],
                controller_speed_cap_mps=4.5, steering_gain=1.4, model_placement='single_gpu', traffic=spec)
            config_path = output / (name + '.yaml')
            options_path = output / (name + '-options.json')
            config_path.write_text(yaml.safe_dump(config, sort_keys=False))
            save(options_path, options)
            destination = output / name
            command = [sys.executable, '-u', '-m', 'local.experiments.traffic_probe', 'capture',
                       '--config', str(config_path), '--options', str(options_path),
                       '--duration', str(spec['duration_s']), '--capture-frames', '0', '40', '80',
                       '--output', str(destination)]
            result = dict(name=name, command=command)
            status.update(state='running', current=name)
            status['unattempted'].remove(name)
            save(output / 'status.json', status)
            worker = None
            started = time.monotonic()
            try:
                with (output / (name + '.log')).open('w') as log:
                    subprocess.run(['bash', 'local/experiments/projected_server.sh', 'start'], cwd=root,
                                   env=environment, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=90)
                    for _ in range(45):
                        if not container_state(args.container)['Running']:
                            raise RuntimeError('CARLA exited during startup')
                        try:
                            client = carla.Client('127.0.0.1', 2000)
                            client.set_timeout(2.)
                            client.get_server_version()
                            break
                        except RuntimeError:
                            time.sleep(2.)
                    else:
                        raise RuntimeError('CARLA startup deadline exceeded')
                    worker = subprocess.Popen(command, cwd=root, env=environment, stdout=log,
                                              stderr=subprocess.STDOUT, start_new_session=True)
                    deadline = time.monotonic() + 1200
                    while worker.poll() is None:
                        if not container_state(args.container)['Running']:
                            result['infrastructure_failure'] = 'CARLA exited during capture'
                            break
                        if time.monotonic() >= deadline:
                            result['timed_out'] = True
                            break
                        time.sleep(2.)
                    stop_worker(worker)
                    result['returncode'] = worker.returncode
                    result['server_before_stop'] = container_state(args.container)
                    with (output / (name + '-server.log')).open('w') as server_log:
                        subprocess.run(['docker', 'logs', args.container], stdout=server_log, stderr=subprocess.STDOUT)
            except BaseException as exc:
                result['error'] = repr(exc)
                if worker is not None:
                    stop_worker(worker)
            finally:
                result['stop_returncode'] = subprocess.run(['bash', 'local/experiments/projected_server.sh', 'stop'],
                    cwd=root, env=environment, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=40).returncode
                result['server_after_stop'] = container_state(args.container)
                result['wall_seconds'] = time.monotonic() - started
            if list(destination.glob('control_*.json')):
                with (output / (name + '-audit.log')).open('w') as log:
                    checked = subprocess.run([sys.executable, '-m', 'local.experiments.audit_traffic', str(destination)],
                        cwd=root, env=environment, stdout=log, stderr=subprocess.STDOUT, timeout=120)
                result['audit_returncode'] = checked.returncode
                path = destination / 'traffic-audit.json'
                if path.exists():
                    result['audit'] = {k: v for k, v in json.loads(path.read_text()).items() if k != 'geometric_audit'}
                video = destination / '000_verifai/overhead.mp4'
                if video.exists():
                    result['video_returncode'] = subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error',
                        '-i', str(video), '-c:v', 'libx264', '-preset', 'fast', '-crf', '21', '-pix_fmt', 'yuv420p',
                        '-movflags', '+faststart', '-an', str(output / (name + '.mp4'))], timeout=120).returncode
            status['results'].append(result)
            save(output / 'status.json', status)
            print(json.dumps(result), flush=True)
            if result.get('returncode') != 0 or not result.get('audit', {}).get('passed') or result['server_after_stop']['Running']:
                status['state'] = 'stopped_after_failure'
                break
        else:
            status['state'] = 'completed'
        status['current'] = None
        save(output / 'status.json', status)
        if status['state'] != 'completed':
            raise SystemExit(1)


if __name__ == '__main__':
    main()
