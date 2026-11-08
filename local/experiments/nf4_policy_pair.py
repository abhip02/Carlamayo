"""Two prospectively paired NF4 sampling-policy jobs, separate from BF16 timing.

The baseline bridge and all shared controller/loader sources remain byte-identical
to the registered precision study. Only two selection options may differ.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

from local.experiments import nf4_closed_loop as bridge

ROOT = Path(__file__).resolve().parents[2]
POLICY = {'speed_selection_attempts': 2, 'speed_preference_mps': [2.5, 5.]}


def sha256(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def validate_candidate(options, baseline, original_validator):
    if baseline.get('precision') != 'nf4' or baseline.get('model_placement') != 'single_gpu':
        raise ValueError('Pairing requires the registered single-GPU NF4 baseline')
    original_validator(baseline)
    if options != {**baseline, **POLICY}:
        raise ValueError('Candidate may change only attempts to2 and preference to[2.5,5]')
    # Original validation applies to every other setting by exact dictionary
    # equality, including red-only hold, source generation seeds and scene scope.


def verify_checkpoint(checkpoint, expected):
    checkpoint = Path(checkpoint)
    names = {p.name for p in checkpoint.iterdir() if p.is_file()}
    if names != set(expected):
        raise ValueError('Checkpoint file inventory differs from registered NF4 export')
    for name, digest in expected.items():
        if sha256(checkpoint / name) != digest:
            raise ValueError(f'Registered checkpoint changed: {name}')


def verify_devices(registration, environment):
    for key in ('CUDA_VISIBLE_DEVICES', 'CARLAMAYO_MODEL_GPUS'):
        if environment.get(key) != registration['model_gpu']:
            raise ValueError('Model GPU differs from the prospective baseline assignment')
    if environment.get('CARLAMAYO_RENDER_GPU') != registration['renderer_gpu']:
        raise ValueError('Renderer GPU differs from the prospective baseline assignment')


def verify_pairing(manifest, root=ROOT, check_weights=True):
    registration_path = Path(manifest['registration_path'])
    if sha256(registration_path) != manifest['registration_sha256']:
        raise ValueError('Prospective baseline registration changed')
    registration = read(registration_path)
    if not registration['registered_before_any_baseline_outcome']:
        raise ValueError('Baseline registration is not prospective')
    if sha256(registration['benchmark_manifest']) != registration['benchmark_manifest_sha256']:
        raise ValueError('Registered baseline jobs changed')
    for name, digest in registration['source_sha256'].items():
        if sha256(root / name) != digest:
            raise ValueError(f'Shared loader/controller source mismatch: {name}')
    for job in registration['jobs']:
        if sha256(job['config_path']) != job['config_sha256'] or sha256(job['options_path']) != job['options_sha256']:
            raise ValueError(f'Registered baseline inputs changed: {job["name"]}')
    if check_weights:
        verify_checkpoint(registration['model_checkpoint'], registration['checkpoint_file_sha256'])
    return registration


def prepare(output, registration_path, declaration_path, render_gpu=None):
    output.mkdir(parents=True, exist_ok=True)
    if (output / 'jobs.json').exists():
        raise ValueError('Preserve the existing protocol; no overwritten jobs')
    registration = read(registration_path)
    declaration = read(declaration_path)
    if not declaration['all_baseline_capture_paths_absent_at_declaration']:
        raise ValueError('Missing prospective pairing declaration')
    if declaration['baseline_manifest_sha256'] != registration['benchmark_manifest_sha256']:
        raise ValueError('Pairing declaration and registered baseline manifest differ')
    manifest = dict(state='prepared_not_executed', study='NF4 sampling policy, quality and call counts only',
        registration_path=str(registration_path.resolve()), registration_sha256=sha256(registration_path),
        prospective_declaration_sha256=sha256(declaration_path),
        fixed_criteria=registration['criteria'], source_sha256={}, jobs=[], pairs=[],
        runtime_pairing_requirements=dict(shared_loader_and_controller_sources='byte-identical',
            checkpoint='same verified export; all11 file hashes checked',
            precision='NF4,624 packed layers,single GPU,BF16 compute,no activation rounding/CPU offload',
            config_and_route='identical parsed runtime data',
            initial_handover_tolerance=dict(position_m=.001, rotation_degrees=.001, speed_mps=.01)),
        extra_jobs_limit=2, wall_timeout_seconds_per_job=1800,
        retries=0, timing_limit='Shared devices: no idle-GPU, BF16, precision-speed or general reliability claim.')
    manifest['execution_assignment'] = dict(model_gpu=registration['model_gpu'],
        renderer_gpu=render_gpu or registration['renderer_gpu'])
    manifest['renderer_placement_amendment'] = dict(
        baseline_renderer=registration['renderer_gpu'],
        candidate_renderer=manifest['execution_assignment']['renderer_gpu'],
        reason='Explicit monitor handoff: original renderer lacks headroom; same A5000 renderer class, same CARLA image/config. Declared before candidate outputs.',
        no_timing_promotion=True)
    verify_pairing(manifest)
    shutil.copyfile(registration_path, output / 'baseline-registration.json')
    if declaration_path.resolve() != (output / 'prospective-declaration.json').resolve():
        shutil.copyfile(declaration_path, output / 'prospective-declaration.json')
    for baseline in registration['jobs']:
        options = read(baseline['options_path'])
        candidate = {**options, **POLICY}
        validate_candidate(candidate, options, bridge.validate_controller)
        name = baseline['name'] + '-cap2'
        job_root = output / baseline['name']
        job_root.mkdir(exist_ok=False)
        config_path, options_path = job_root / (name + '.yaml'), job_root / (name + '-options.json')
        shutil.copyfile(baseline['config_path'], config_path)
        write(options_path, candidate)
        baseline_command = baseline['command']
        duration = baseline_command[baseline_command.index('--duration') + 1]
        command = [str(ROOT / '.venv/bin/python'), '-m', 'local.experiments.nf4_policy_pair', 'capture',
            '--jobs-manifest', str(job_root / 'jobs.json'), '--config', str(config_path),
            '--options', str(options_path), '--checkpoint', registration['model_checkpoint'],
            '--duration', duration, '--capture-frames', '0', '--output', str(job_root / name)]
        manifest['jobs'].append(dict(name=name, precision='nf4', command=command,
            config_sha256=sha256(config_path), options_sha256=sha256(options_path),
            baseline_name=baseline['name'], baseline_options=options,
            audit_command=[str(ROOT / '.venv/bin/python'), '-m', 'local.experiments.audit_projection', str(job_root / name)]))
        manifest['pairs'].append(dict(baseline=baseline['planned_output'], candidate=str(job_root / name),
            baseline_name=baseline['name'], candidate_name=name, baseline_options_sha256=baseline['options_sha256'],
            candidate_options_sha256=sha256(options_path), config_sha256=sha256(config_path),
            text_seed=baseline['text_seed'], flow_seed=baseline['flow_seed'],
            only_changed_options=POLICY))
    if len(manifest['jobs']) != 2:
        raise ValueError('Exactly two registered baseline jobs required')
    manifest['source_sha256'] = {str(p.relative_to(ROOT)): sha256(p)
        for folder in ('module', 'local') for p in (ROOT / folder).rglob('*.py')}
    for name in manifest['source_sha256']:
        target = output / 'sources' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, target)
    write(output / 'jobs.json', manifest)
    for job, pair in zip(manifest['jobs'], manifest['pairs']):
        subset = {**manifest, 'jobs': [job], 'pairs': [pair], 'subset_of': str(output/'jobs.json')}
        write(Path(pair['candidate']).parent / 'jobs.json', subset)
    write(output / 'pairing-verification.json', dict(shared_sources_identical=len(registration['source_sha256']),
        checkpoint_files_verified=len(registration['checkpoint_file_sha256']),
        config_bytes_identical=True, permitted_option_differences=POLICY,
        new_gpu_runs=0, source_and_options_verified=True))


def capture(arguments):
    parser = argparse.ArgumentParser(add_help=False)
    for flag in ('jobs-manifest', 'config', 'options', 'output', 'checkpoint'):
        parser.add_argument('--' + flag, type=Path, required=True)
    args, _ = parser.parse_known_args(arguments)
    manifest = read(args.jobs_manifest)
    verify_devices(manifest['execution_assignment'], os.environ)
    registration = verify_pairing(manifest)
    job = bridge.verify_manifest(args.jobs_manifest, args.output, args.config, args.options)
    if arguments != job['command'][4:]:
        raise ValueError('Candidate command differs from frozen profile')
    if args.checkpoint.resolve() != Path(registration['model_checkpoint']).resolve():
        raise ValueError('Candidate checkpoint differs from the registered baseline')
    original = bridge.validate_controller
    validate_candidate(read(args.options), job['baseline_options'], original)
    bridge.validate_controller = lambda actual: validate_candidate(actual, job['baseline_options'], original)
    try:
        # Same frozen model loader, .41 memory cap, precision reporting, capture,
        # route projection, signal/stop guards and controller as the baseline.
        bridge.capture(arguments, cpu_offload=False)
    finally:
        bridge.validate_controller = original


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=('prepare', 'capture', 'verify'))
    args, rest = parser.parse_known_args()
    if args.stage == 'capture':
        capture(rest)
        return
    sub = argparse.ArgumentParser()
    sub.add_argument('--output', type=Path, required=True)
    if args.stage == 'prepare':
        sub.add_argument('--registration', type=Path, required=True)
        sub.add_argument('--declaration', type=Path, required=True)
        sub.add_argument('--render-gpu', help='Explicitly declared renderer placement amendment; model GPU stays fixed')
        values = sub.parse_args(rest)
        prepare(values.output.resolve(), values.registration.resolve(), values.declaration.resolve(), values.render_gpu)
    else:
        values = sub.parse_args(rest)
        registration = verify_pairing(read(values.output / 'jobs.json'))
        print(json.dumps({'verified': True, 'shared_sources': len(registration['source_sha256']),
                          'checkpoint_files': len(registration['checkpoint_file_sha256'])}))


if __name__ == '__main__':
    main()
