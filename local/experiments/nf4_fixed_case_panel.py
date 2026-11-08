"""Two prospective candidate-only checks: original left turn and tight bend.

Historical BF16 captures provide fixed inputs, not matched outcome comparators.
The completed NF4 T-junction pair remains the only paired call-count result.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import signal
import subprocess
import sys
from types import SimpleNamespace

from local.experiments import nf4_policy_pair as policy
from local.experiments import nf4_closed_loop as bridge
from local.experiments import supervise_nf4_policy as supervisor
from local.experiments.report_nf4_policy_pair import outcome

ROOT = policy.ROOT
SCENE_FIELDS = {'seed', 'flow_seed', 'validation_route', 'instruction', 'route_spec'}


def same_policy(candidate, reference):
    if {k:v for k,v in candidate.items() if k not in SCENE_FIELDS} != {
            k:v for k,v in reference.items() if k not in SCENE_FIELDS}:
        raise ValueError('Panel may change only the declared scene and seed fields')


def prepare(output, reference_path):
    import yaml
    reference = policy.read(reference_path)
    registration = policy.verify_pairing(reference)
    tjob = next(j for j in reference['jobs'] if j['name'].startswith('09-'))
    tc = tjob['command']
    t_options_path = Path(tc[tc.index('--options')+1])
    if policy.sha256(t_options_path) != tjob['options_sha256']:
        raise ValueError('Reference candidate options changed')
    t_options = policy.read(t_options_path)
    if t_options['speed_selection_attempts'] != 2 or t_options['speed_preference_mps'] != [2.5,5.]:
        raise ValueError('Expected the unchanged successful T candidate policy')
    for name, digest in reference['source_sha256'].items():
        if policy.sha256(ROOT/name) != digest:
            raise ValueError('Reference source changed: '+name)
    original_manifest_path = ROOT/'local/experiments/presets/ten-driving-scenes.json'
    original = policy.read(original_manifest_path)
    historical_protocol = ROOT/'outputs/model-score-benchmark-20261105/protocol.json'
    history = {c['name']:c for c in policy.read(historical_protocol)['cases']}
    output.mkdir(parents=True, exist_ok=False)
    manifest = dict(study='Candidate-only fixed left/tight specification checks; no fresh baseline',
        registered_utc=datetime.now(timezone.utc).isoformat(), jobs=[], cases=[], source_sha256={},
        registration_path=reference['registration_path'], registration_sha256=reference['registration_sha256'],
        execution_assignment=reference['execution_assignment'],
        reference_T_manifest=str(reference_path.resolve()), reference_T_manifest_sha256=policy.sha256(reference_path),
        original_scenario_manifest_sha256=policy.sha256(original_manifest_path),
        historical_inputs_only=True, fresh_baseline_jobs=0, case_order=[2,8], retries=0,
        stop_after_first_nonpassing_case=True, capture_timeout_seconds=1800,
        resource_gate=dict(seconds=60, model_free_gib=10., renderer_free_gib=9.8, leases='model, renderer, port2000'),
        fixed_pass_criteria=dict(route_exit_required=True, min_speed_mps=1., max_speed_mps=5.5,
            collisions=0, footprint_departures=0, integrity_pass_required=True, max_samples_per_decision=2),
        interpretation='Only the existing T result has a matched NF4 call-count reduction. Historical BF16 outcomes are context, not comparators.',
        expected_cost_minutes=[20,35], maximum_capture_minutes=60,
        estimate_note='Startup, two60s gates and bounded cleanup add time; stop after first failure; no retry.')
    for number in (2,8):
        case = original['scenarios'][number-1]
        source_root = Path(history[case['name']]['source']).parents[2]
        stem = case['name']+'-route'
        source_config, source_options = source_root/(stem+'.yaml'), source_root/(stem+'-options.json')
        baseline_options = {**policy.read(source_options), 'precision':'nf4'}
        candidate = {**baseline_options, **policy.POLICY}
        policy.validate_candidate(candidate, baseline_options, bridge.validate_controller)
        same_policy(candidate, t_options)
        for key in ('seed','flow_seed','instruction','route_spec'):
            if candidate[key] != case[key]:
                raise ValueError('Original fixed case mismatch: '+key)
        if candidate['validation_route'] != case['route'] or candidate['scene_policy'] != 'empty_green':
            raise ValueError('Original route or scene policy changed')
        config = yaml.safe_load(source_config.read_text())
        if config['duration_s'] != case['duration_s'] or config['junction_entry'] != case['junction_entry']:
            raise ValueError('Original scene/maximum duration changed')
        expected_start = {k:case[k] for k in ('name','distance_before_intersection_m','lateral_offset_m','speed_mps','heading_offset_deg')}
        if config['rollouts'] != [expected_start]:
            raise ValueError('Original requested start changed')
        name = case['name']+'-nf4-cap2'
        job_root = output/case['name'];job_root.mkdir()
        config_path, options_path = job_root/(name+'.yaml'), job_root/(name+'-options.json')
        shutil.copyfile(source_config, config_path);policy.write(options_path, candidate)
        command = [str(ROOT/'.venv/bin/python'),'-m','local.experiments.nf4_policy_pair','capture',
            '--jobs-manifest',str(job_root/'jobs.json'),'--config',str(config_path),'--options',str(options_path),
            '--checkpoint',registration['model_checkpoint'],'--duration',str(case['duration_s']),
            '--capture-frames','0','--output',str(job_root/name)]
        job = dict(name=name, precision='nf4', command=command, config_sha256=policy.sha256(config_path),
            options_sha256=policy.sha256(options_path), baseline_options=baseline_options,
            baseline_options_note='Validation template from historical fixed-case inputs; no fresh NF4 baseline job',
            audit_command=[str(ROOT/'.venv/bin/python'),'-m','local.experiments.audit_projection',str(job_root/name)])
        manifest['jobs'].append(job)
        manifest['cases'].append(dict(number=number, definition=case, job_manifest=str(job_root/'jobs.json'),
            capture=str(job_root/name), name=name, original_config=str(source_config),
            original_config_sha256=policy.sha256(source_config), original_options=str(source_options),
            original_options_sha256=policy.sha256(source_options), historical_capture=str(source_root/stem)))
    manifest['source_sha256'] = {str(p.relative_to(ROOT)):policy.sha256(p)
        for folder in ('module','local') for p in (ROOT/folder).rglob('*.py')}
    for name in manifest['source_sha256']:
        target = output/'sources'/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/name,target)
    policy.write(output/'jobs.json', manifest)
    for job, case in zip(manifest['jobs'],manifest['cases']):
        policy.write(Path(case['job_manifest']), {**manifest,'jobs':[job],'cases':[case], 'parent_panel':str(output/'jobs.json')})
        case['job_manifest_sha256'] = policy.sha256(Path(case['job_manifest']))
    policy.write(output/'jobs.json',manifest)


def case_parser(number):
    class Parser(argparse.ArgumentParser):
        def add_argument(self, *names, **kwargs):
            if names == ('--cases',):
                kwargs.update(choices=(number,), default=[number])
            return super().add_argument(*names, **kwargs)
    return Parser


def run_one(case, assignment, container):
    from local.experiments import run_nf4_pilot as driver
    manifest_path = Path(case['job_manifest']);manifest = policy.read(manifest_path);job = manifest['jobs'][0]
    if policy.sha256(manifest_path) != case['job_manifest_sha256'] or manifest['execution_assignment'] != assignment:
        raise ValueError('Case manifest or declared GPU assignment changed')
    command = job['command'];value=lambda flag:Path(command[command.index(flag)+1])
    bridge.verify_manifest(manifest_path,value('--output'),value('--config'),value('--options'))
    path=manifest_path.parent/'supervision.json'
    if path.exists():
        raise ValueError('Preserve previous case supervision')
    status=dict(state='prepared',case_number=case['number'],container=container)
    def save():policy.write(path,status)
    save()
    originals=(driver.subprocess,driver.check_memory,driver.argparse,sys.argv)
    def run(actual,**kwargs):
        if actual == command:
            return supervisor.guarded_capture(actual,kwargs,container,status,save)
        if len(actual)>=3 and Path(actual[1]).name=='projected_server.sh':
            kwargs.setdefault('timeout',25 if actual[-1]=='stop' else 30)
            return supervisor.run_server_helper(actual,kwargs)
        kwargs.setdefault('timeout',120)
        return subprocess.run(actual,**kwargs)
    def interrupted(signum,frame):
        signal.signal(signal.SIGTERM,signal.SIG_IGN)
        raise InterruptedError('Fixed-case panel received SIGTERM')
    old_handler=signal.signal(signal.SIGTERM,interrupted)
    driver.subprocess=SimpleNamespace(run=run,TimeoutExpired=subprocess.TimeoutExpired,STDOUT=subprocess.STDOUT)
    driver.check_memory=supervisor.bounded_memory
    # Only broaden this invocation's CLI case number; frozen driver code stays unchanged.
    driver.argparse=SimpleNamespace(ArgumentParser=case_parser(case['number']))
    sys.argv=['run_nf4_pilot',str(manifest_path),'--precision','nf4','--model-gpu',assignment['model_gpu'],
        '--render-gpu',assignment['renderer_gpu'],'--container',container,
        '--cases',str(case['number']),'--headroom-seconds','60','--wall-timeout-s','1800']
    try:driver.main()
    except BaseException as error:
        status.update(state='driver_error',error=repr(error));raise
    finally:
        driver.subprocess,driver.check_memory,driver.argparse,sys.argv=originals
        signal.signal(signal.SIGTERM,old_handler)
        driver_path=manifest_path.parent/'nf4-status.json'
        if driver_path.exists():
            driver_status=policy.read(driver_path)
            status.update(driver_state=driver_status.get('state'),server_stop_returncode=driver_status.get('server_stop_returncode'))
        save()
    if status.get('server_stop_returncode') != 0:
        status.update(state='cleanup_error',error='Owned server stop did not succeed');save()
        raise RuntimeError(status['error'])


def run_panel(manifest_path, container_prefix, execute=run_one, assess=outcome):
    manifest=policy.read(manifest_path)
    if [c['number'] for c in manifest['cases']] != [2,8]:
        raise ValueError('Exactly left02 then tight08 required')
    status_path=manifest_path.parent/'panel-status.json'
    if status_path.exists():raise ValueError('No repeated panel attempts')
    status=dict(state='prepared',planned=2,results=[dict(name=c['name'],state='not_attempted') for c in manifest['cases']])
    policy.write(status_path,status)
    for index,case in enumerate(manifest['cases']):
        status.update(state='running',current=case['name']);policy.write(status_path,status)
        error=None
        try:execute(case,manifest['execution_assignment'],f'{container_prefix}-{case["number"]:02d}')
        except BaseException as exc:error=repr(exc)
        try:row=assess(case['capture'],case['name'])
        except Exception as exc:
            row=dict(name=case['name'],state='assessment_error',passed=False,error=repr(exc))
        if error:
            row.update(passed=False,assessed_state=row['state'],state='launch_or_cleanup_error',launcher_error=error)
        status['results'][index]=row;policy.write(status_path,status)
        if not row.get('passed'):
            status.update(state='stopped_after_nonpassing_case',current=None)
            for pending in status['results'][index+1:]:pending['reason']='Not attempted after earlier nonpassing case'
            break
    else:status.update(state='completed',current=None)
    policy.write(status_path,status)
    return status


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage',choices=('prepare','run'))
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--reference',type=Path)
    parser.add_argument('--container-prefix')
    args=parser.parse_args()
    if args.stage=='prepare':
        if not args.reference:parser.error('--reference is required')
        prepare(args.output.resolve(),args.reference.resolve())
    else:
        if not args.container_prefix:parser.error('--container-prefix is required')
        result=run_panel(args.output.resolve()/'jobs.json',args.container_prefix)
        if result['state']!='completed':raise SystemExit(1)


if __name__=='__main__':main()
