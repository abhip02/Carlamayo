"""Run the frozen paired pilot only under cooperative leases on idle resources."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time

from local.experiments.resource_lease import acquire, require_idle


def stop_worker(worker):
    if worker.poll() is not None:
        return
    try:
        os.killpg(worker.pid, signal.SIGINT)
    except ProcessLookupError:
        worker.wait()
        return
    try:
        worker.wait(timeout=60)
    except subprocess.TimeoutExpired:
        os.killpg(worker.pid, signal.SIGKILL)
        worker.wait()


def verify_sources(root, protocol):
    for name, expected in protocol['source_sha256'].items():
        if hashlib.sha256((root / name).read_bytes()).hexdigest() != expected:
            raise RuntimeError(f'Frozen pilot source changed: {name}')
    manifest = root / protocol['scenario_manifest']
    if hashlib.sha256(manifest.read_bytes()).hexdigest() != protocol['scenario_manifest_sha256']:
        raise RuntimeError('Frozen pilot scenario manifest changed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check-only', action='store_true', help='Check protocol, leases, GPU occupancy and ports without launching')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    protocol = json.loads((root / 'local/experiments/presets/latency-pilot.json').read_text())
    verify_sources(root, protocol)
    model_gpu, render_gpu = os.environ.get('CARLAMAYO_MODEL_GPUS'), os.environ.get('CARLAMAYO_RENDER_GPU')
    with acquire(root.parent / '.carlamayo-resource-locks', model_gpu, render_gpu):
        resources = require_idle(model_gpu, render_gpu)
        if shutil.disk_usage(root).free < 4 * 1024**3:
            raise RuntimeError('Need at least 4 GiB free for the bounded capture set')
        if args.check_only:
            print(json.dumps({'ready': True, 'resources': resources, 'planned_jobs': len(protocol['jobs'])}))
            return
        out = args.output.resolve()
        out.mkdir(parents=True, exist_ok=False)
        (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
        status = dict(state='running', resources=resources, planned=len(protocol['jobs']),
                      git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(), results=[])
        environment = {**os.environ, 'CARLAMAYO_PROJECTION_CONTAINER': protocol['resource_requirements']['container']}

        def save():
            temporary = out / 'status.tmp'
            temporary.write_text(json.dumps(status, indent=2) + '\n')
            temporary.replace(out / 'status.json')

        save()
        try:
            for job in protocol['jobs']:
                verify_sources(root, protocol)
                require_idle(model_gpu, render_gpu)
                destination = out.joinpath(*Path(job['output']).parts[-2:])
                command = list(job['argv'])
                command[command.index('--output') + 1] = str(destination)
                started = time.monotonic()
                result = dict(name=job['name'], output=str(destination), command=command)
                status['current'] = job['name']
                save()
                with (out / (job['name'] + '.log')).open('w') as stream:
                    worker = subprocess.Popen(command, cwd=root, env=environment,
                        stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
                    try:
                        result['returncode'] = worker.wait(timeout=job['timeout_seconds'])
                    except subprocess.TimeoutExpired:
                        result['timed_out'] = True
                        stop_worker(worker)
                        result['returncode'] = worker.returncode
                    finally:
                        stop_worker(worker)
                        result.setdefault('returncode', worker.returncode)
                        # The server helper verifies ownership labels before stopping.
                        result['cleanup_returncode'] = subprocess.run(
                            ['bash', 'local/experiments/projected_server.sh', 'stop'],
                            cwd=root, env=environment, stdout=stream, stderr=subprocess.STDOUT).returncode
                        result['wall_seconds'] = time.monotonic() - started
                        status['results'].append(result)
                        save()
            status.update(state='completed', current=None)
        except BaseException as exc:
            status.update(state='interrupted', error=repr(exc))
            raise
        finally:
            save()
        if any(r.get('timed_out') or r.get('returncode') != 0 or r.get('cleanup_returncode') != 0 for r in status['results']):
            raise SystemExit(1)


if __name__ == '__main__':
    main()
