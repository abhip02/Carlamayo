"""Resume the fixed left/tight panel with the reviewed NF4 startup loader.

This adapter changes allocation budget and GPU placement only. The frozen
scenario, generation, sampling, controller and audit implementations stay intact.
"""
import argparse
from contextlib import contextmanager
from pathlib import Path

from local.experiments import nf4_fixed_case_panel as panel
from local.experiments import nf4_policy_pair as policy
from local.experiments import supervise_nf4_policy as guard

MODEL_GPU = 'GPU-ce1bea0e-c85a-f48a-a1ad-5c2bc153e10b'
RENDER_GPU = 'GPU-fd2c05bc-4b86-6451-49ae-c9e3f7f79580'
REPAIR_SHA = 'fe07bbd4ec257594f3a52da356810f7a1c71648b037ffb9a514033cff4e06b44'
MODULE = 'local.experiments.resume_fixed_cases'


def amendment():
    return dict(loader='local/experiments/nf4_startup_loader.py',
        loader_sha256=REPAIR_SHA, allocation_fraction=.45,
        model_free_gib=11., renderer_free_gib=9.8,
        model_gpu=MODEL_GPU, renderer_gpu=RENDER_GPU,
        camera_pause_seconds=0, controller_changed=False,
        interpretation='Startup allocation/telemetry and placement amendment only; no policy or scene change.')


def verify_amendment(manifest):
    if manifest.get('resume_amendment') != amendment():
        raise ValueError('Unexpected loader, resource, or controller amendment')
    if policy.sha256(policy.ROOT / amendment()['loader']) != REPAIR_SHA:
        raise ValueError('Reviewed startup loader changed')


def prepare(output, reference):
    panel.prepare(output, reference)
    path = output / 'jobs.json'
    manifest = policy.read(path)
    manifest['resume_amendment'] = amendment()
    manifest['execution_assignment'] = dict(model_gpu=MODEL_GPU, renderer_gpu=RENDER_GPU)
    manifest['resource_gate']['model_free_gib'] = 11.
    for job in manifest['jobs']:
        if job['command'][2:4] != ['local.experiments.nf4_policy_pair', 'capture']:
            raise ValueError('Unexpected original capture entry point')
        job['command'][2] = MODULE
    for case, job in zip(manifest['cases'], manifest['jobs'], strict=True):
        child_path = Path(case['job_manifest'])
        child = {**manifest, 'jobs': [job], 'cases': [case], 'parent_panel': str(path)}
        policy.write(child_path, child)
        case['job_manifest_sha256'] = policy.sha256(child_path)
    verify_amendment(manifest)
    policy.write(path, manifest)


@contextmanager
def repaired_bridge():
    from local.experiments import nf4_startup_loader as repaired
    original = policy.bridge
    policy.bridge = repaired
    try:
        yield
    finally:
        policy.bridge = original


def capture(arguments):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--jobs-manifest', type=Path, required=True)
    args, _ = parser.parse_known_args(arguments)
    verify_amendment(policy.read(args.jobs_manifest))
    with repaired_bridge():
        policy.capture(arguments)


@contextmanager
def repaired_memory_gate():
    original = guard.bounded_memory

    def check(gpus, minimums):
        # Runtime telemetry requests zero thresholds: never impose startup
        # headroom again after our own allocations have begun.
        required = [max(value, 11.) if gpu == MODEL_GPU and value > 0 else value
                    for gpu, value in zip(gpus, minimums, strict=True)]
        return original(gpus, required)

    guard.bounded_memory = check
    try:
        yield
    finally:
        guard.bounded_memory = original


def run_one(case, assignment, container):
    from local.experiments.left_camera_study import cleanup_snapshot
    from local.experiments.resource_lease import lease
    manifest = policy.read(case['job_manifest'])
    verify_amendment(manifest)
    if assignment != dict(model_gpu=MODEL_GPU, renderer_gpu=RENDER_GPU):
        raise ValueError('Use the declared GPU pair')
    try:
        with repaired_memory_gate():
            panel.run_one(case, assignment, container)
    finally:
        directory = Path(case['job_manifest']).parent
        supervision = directory / 'supervision.json'
        state = policy.read(supervision) if supervision.exists() else {}
        cleanup = cleanup_snapshot(container, state.get('capture_pid'))
        try:
            with lease([MODEL_GPU, RENDER_GPU], port=2000):
                cleanup['all_three_leases_reacquired'] = True
        except Exception as error:
            cleanup.update(verified=False, lease_error=repr(error))
        cleanup['verified'] &= state.get('server_stop_returncode') == 0
        policy.write(directory / 'cleanup.json', cleanup)
        if not cleanup['verified']:
            raise RuntimeError('Independent owned-resource cleanup not verified')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=('prepare', 'run', 'capture'))
    args, rest = parser.parse_known_args()
    if args.stage == 'capture':
        capture(rest)
        return
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path)
    parser.add_argument('--container-prefix')
    options = parser.parse_args(rest)
    if args.stage == 'prepare':
        if options.reference is None:
            parser.error('--reference is required')
        prepare(options.output.resolve(), options.reference.resolve())
    else:
        if not options.container_prefix:
            parser.error('--container-prefix is required')
        manifest_path = options.output.resolve() / 'jobs.json'
        verify_amendment(policy.read(manifest_path))
        result = panel.run_panel(manifest_path, options.container_prefix, execute=run_one)
        if result['state'] != 'completed':
            raise SystemExit(1)


if __name__ == '__main__':
    main()
