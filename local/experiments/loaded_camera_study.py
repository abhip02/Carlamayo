"""Four full model-loaded left-startup prefixes; never run driving inference."""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time

from local.experiments import left_camera_study as camera
from local.experiments import supervise_nf4_policy as guard

ROOT = camera.ROOT
MODEL_GPU = 'GPU-ce1bea0e-c85a-f48a-a1ad-5c2bc153e10b'
RENDER_GPU = camera.RENDER_GPU
ORDER = (0, 1, 1, 0)
REPAIR_SHA = 'fe07bbd4ec257594f3a52da356810f7a1c71648b037ffb9a514033cff4e06b44'
CHILD_LIMIT = 240
read, write, digest, now = camera.read, camera.write, camera.digest, camera.now


class PrefixComplete(BaseException):
    """Expected diagnostic cutoff, never a completed driving rollout."""


def verify_prefix_output(output,result):
    precision=read(output/'precision.json')
    summary=read(output/'000_verifai/summary.json')
    if (precision['precision']!='nf4' or len(precision['quantized_layers'])!=624
            or precision['cpu_offload'] or precision['compute_dtype']!='bfloat16'):
        raise ValueError('Expected loaded624-layer NF4 model with BF16 compute and no offload')
    memory=precision['startup_memory']['devices']
    if len(memory)!=1 or not math.isclose(memory[0]['allocation_budget_fraction'],.45,rel_tol=0,abs_tol=1e-8):
        raise ValueError('Expected reviewed .45 single-device allocator repair')
    if (not result.get('model_loaded') or result['forbidden_inference_attempts']
            or not result['reached_handover'] or summary['simulation_seconds']!=0
            or summary['inference_calls']!=0 or not summary.get('error','').startswith('PrefixComplete:')):
        raise ValueError('Prefix did not stop before inference')
    if list(output.glob('proposal_*.json')) or list(output.glob('frame_*.json')):
        raise ValueError('Unexpected inference artifacts')
    return precision['startup_memory']


@contextmanager
def prefix_hooks(preroll, interface_class, probe_class, pause, output, event, result):
    """Restore every temporary hook even when CARLA or the cutoff raises."""
    original_initialize = preroll.initialize_ego
    original_setup = interface_class.setup_cameras
    blocked_names = ('evaluate', 'generate', 'sample', 'cfg_sample', 'flow_sample')
    original_methods = {name:getattr(probe_class,name) for name in blocked_names}

    def forbidden(*args, **kwargs):
        result['forbidden_inference_attempts'] += 1
        event('forbidden_inference_attempt')
        raise RuntimeError('Startup prefix forbids ALL native, VQA and diffusion inference')

    def initialize(interface, entry, spec):
        event('preroll_begin')
        frames, requested = original_initialize(interface, entry, spec)
        # Exactly the next state-read used by the full run_rollout after pre-roll.
        actual = interface.get_ego_state()
        shapes = [list(frame.shape) for frame in frames]
        if len(frames) != 4 or any(shape != [4,1080,1920,3] for shape in shapes):
            raise ValueError('Incomplete four-camera history')
        handover = dict(actual=actual, requested_pose=requested,
            acknowledged_history_state=dict(interface.history_buffer[-1]),
            requested_speed_mps=spec['speed_mps'], frame=int(interface.frame_id),
            retained_history_frames=len(frames), retained_shapes=shapes)
        write(output/'prefix-handover.json', handover)
        result.update(handover=handover, reached_handover=True)
        event('handover_verified', **handover)
        raise PrefixComplete('Expected cutoff after pre-roll and state read, before follower/inference')

    preroll.initialize_ego = initialize
    interface_class.setup_cameras = camera.wrap_camera_setup(original_setup, pause, event)
    for name in blocked_names:
        setattr(probe_class,name,forbidden)
    try:
        yield
    finally:
        preroll.initialize_ego = original_initialize
        interface_class.setup_cameras = original_setup
        for name, method in original_methods.items():
            setattr(probe_class,name,method)


def prepare(output, left_manifest, repair_source):
    from local.experiments import nf4_policy_pair as policy
    reference = read(left_manifest)
    policy.verify_pairing(reference)
    for name, expected in reference['source_sha256'].items():
        if digest(ROOT/name) != expected:
            raise ValueError('Previous frozen source changed: '+name)
    if digest(repair_source) != REPAIR_SHA or digest(ROOT/'local/experiments/nf4_startup_loader.py') != REPAIR_SHA:
        raise ValueError('Require byte-identical reviewed allocator repair')
    old = reference['jobs'][0]
    if not old['name'].startswith('02-intersection-left'):
        raise ValueError('Use the failed left child manifest')
    value = lambda flag:Path(old['command'][old['command'].index(flag)+1])
    output.mkdir(parents=True,exist_ok=False)
    config, options = output/'left.yaml', output/'left-options.json'
    shutil.copyfile(value('--config'),config);shutil.copyfile(value('--options'),options)
    if digest(config)!=old['config_sha256'] or digest(options)!=old['options_sha256']:
        raise ValueError('Failed left input hashes changed')
    sources={str(p.relative_to(ROOT)):digest(p) for directory in ('local','module')
             for p in (ROOT/directory).rglob('*.py')}
    sources['local/experiments/projected_server.sh']=digest(ROOT/'local/experiments/projected_server.sh')
    assignment=dict(model_gpu=MODEL_GPU,renderer_gpu=RENDER_GPU)
    protocol=dict(registered_utc=now(),source_sha256=sources,execution_assignment=assignment,
        prior_failed_manifest=str(left_manifest.resolve()),prior_failed_manifest_sha256=digest(left_manifest),
        repair_source=str(repair_source.resolve()),repair_sha256=REPAIR_SHA,order=list(ORDER),trials=[],
        native_calls=0,vqa_calls=0,maximum_starts=4,retries=0,child_limit_seconds=CHILD_LIMIT,
        headroom_gate=dict(seconds=60,model_free_gib=11.,renderer_free_gib=9.8),
        model_device_amendment=dict(old='GPU-46394453-71bb-15b9-1612-860b5714f750',new=MODEL_GPU,
            reason='Original model GPU1 below11GiB gate; fixed GPU3 assignment proposed after benchmark release. No device hopping.',
            limitation='Same model/precision, different physical model device and shared load; no exact failed-run timing equivalence.'),
        expected_wall_minutes=[8,12],conservative_maximum_minutes=32,
        stop_rule='Stop on resource refusal, cleanup failure, interruption, forbidden inference or corrupt result; retain each other planned startup outcome once.',
        cutoff='After full acknowledged initialize_ego returns and same get_ego_state call as run_rollout; PrefixComplete propagates through original finally cleanup.',
        interpretation='Startup study only; no driving result. Pause and logging hooks identical across arms except declared1s pause. No cadence change.')
    for index,pause in enumerate(ORDER,1):
        name=f'{index:02d}-'+('pause' if pause else 'original')
        container='apomalap-carlamayo-'+output.name+'-'+name
        trial_root=output/name
        manifest_path=output/(name+'-jobs.json')
        command=[str(ROOT/'.venv/bin/python'),'-m','local.experiments.loaded_camera_study','child',
            '--jobs-manifest',str(manifest_path),'--config',str(config),'--options',str(options),
            '--checkpoint',str(value('--checkpoint')),'--duration','15','--capture-frames','0',
            '--output',str(trial_root)]
        job={**old,'name':name,'command':command}
        job.pop('audit_command',None)
        manifest={**reference,'jobs':[job],'source_sha256':sources,'execution_assignment':assignment,
            'startup_only':dict(pause_seconds=pause,repair_sha256=REPAIR_SHA,maximum_native_vqa_calls=0)}
        write(manifest_path,manifest)
        protocol['trials'].append(dict(name=name,pause_seconds=pause,container=container,command=command,
            manifest=str(manifest_path),manifest_sha256=digest(manifest_path)))
    write(output/'protocol.json',protocol)


def child(arguments):
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--jobs-manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args,_=parser.parse_known_args(arguments)
    manifest=read(args.jobs_manifest)
    if digest(ROOT/'local/experiments/nf4_startup_loader.py') != manifest['startup_only']['repair_sha256']:
        raise ValueError('Reviewed allocator repair changed')
    # Keep diagnostic files beside the capture: original capture creates its own
    # output with exist_ok=False, and every original source remains immutable.
    path=args.output.with_name(args.output.name+'-prefix.json')
    event_path=args.output.with_name(args.output.name+'-events.jsonl')
    if path.exists() or event_path.exists() or args.output.exists():
        raise ValueError('Never overwrite a startup attempt')
    from local.experiments import nf4_policy_pair as policy, nf4_startup_loader as repaired
    from local.experiments import acknowledged_preroll, inference_probe
    from module.carla_interface import CARLAInterface
    result=dict(state='started',reached_handover=False,forbidden_inference_attempts=0,
                native_calls=0,vqa_calls=0,started_utc=now(),pause_seconds=manifest['startup_only']['pause_seconds'])
    write(path,result)
    base_probe=inference_probe.Probe
    original_bridge=policy.bridge
    original_load=inference_probe.Alpamayo15Adapter.load_model
    origin=time.monotonic()
    with event_path.open('x') as stream:
        def event(name,**fields):
            stream.write(json.dumps(dict(event=name,utc=now(),elapsed_s=time.monotonic()-origin,**fields))+'\n');stream.flush()
        def load_model(self,*values,**kwargs):
            event('model_load_begin')
            loaded=original_load(self,*values,**kwargs)
            event('model_load_end')
            result['model_loaded']=True
            return loaded
        policy.bridge=repaired
        inference_probe.Alpamayo15Adapter.load_model=load_model
        try:
            with prefix_hooks(acknowledged_preroll,CARLAInterface,base_probe,
                    manifest['startup_only']['pause_seconds'],args.output,event,result):
                policy.capture(arguments)
            raise RuntimeError('Prefix returned without explicit handover cutoff')
        except PrefixComplete:
            # This point is reached only after original capture/model/world cleanup.
            try:
                memory=verify_prefix_output(args.output,result)
                result.update(state='prefix_complete',startup_memory=memory)
                event('prefix_complete')
            except BaseException as error:
                result.update(state='prefix_error',error=repr(error));raise
        except BaseException as error:
            result.update(state='prefix_error',error=repr(error));raise
        finally:
            policy.bridge=original_bridge
            inference_probe.Alpamayo15Adapter.load_model=original_load
            result['finished_utc']=now();write(path,result)


def port_open():
    with socket.socket() as probe:
        probe.settimeout(1)
        return probe.connect_ex(('127.0.0.1',2000))==0


def supervise(command,env,log,container,state,save):
    child_process=subprocess.Popen(command,env=env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
    state['child_pid']=child_process.pid;save()
    deadline=time.monotonic()+CHILD_LIMIT
    try:
        while child_process.poll() is None:
            if time.monotonic()>=deadline:
                raise TimeoutError('Startup prefix exceeded240seconds')
            server=json.loads(subprocess.check_output(['docker','inspect','--format','{{json .State}}',container],timeout=5))
            state['last_server_state']=server
            if not server['Running']:
                raise RuntimeError('Owned CARLA exited during loaded startup')
            save()
            try:child_process.wait(timeout=1)
            except subprocess.TimeoutExpired:pass
        state['child_returncode']=child_process.returncode
    finally:
        guard.stop_child(child_process)
        state['child_returncode']=child_process.returncode;save()


def run_trial(trial,root):
    from local.experiments.resource_lease import lease
    path=root/(trial['name']+'-status.json')
    state=dict(state='prepared',container=trial['container'],pause_seconds=trial['pause_seconds'],headroom=[])
    def save():write(path,state)
    env=dict(os.environ,CUDA_VISIBLE_DEVICES=MODEL_GPU,CARLAMAYO_MODEL_GPUS=MODEL_GPU,
        CARLAMAYO_RENDER_GPU=RENDER_GPU,CARLAMAYO_PROJECTION_CONTAINER=trial['container'],
        PYTHONDONTWRITEBYTECODE='1',PYTHONUNBUFFERED='1')
    helper=['bash',str(ROOT/'local/experiments/projected_server.sh')]
    save()
    try:
        with lease([MODEL_GPU,RENDER_GPU],port=2000):
            if subprocess.run(['docker','inspect',trial['container']],stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,timeout=5).returncode==0 or port_open():
                raise RuntimeError('Existing container or occupied port; preserve it')
            ready=time.monotonic()+60
            while True:
                free=guard.bounded_memory([MODEL_GPU,RENDER_GPU],[11.,9.8])
                state['headroom'].append(dict(utc=now(),free_gib=free));save()
                if time.monotonic()>=ready:break
                time.sleep(min(5,max(0,ready-time.monotonic())))
            state['start_attempted']=True
            try:
                guard.run_server_helper(camera.start_command(trial['container']),dict(env=env,cwd=ROOT,timeout=30,check=True))
                ready=time.monotonic()+60
                while not port_open():
                    if time.monotonic()>=ready:raise TimeoutError('Owned server port readiness timeout')
                    time.sleep(1)
                state['state']='running';save()
                with (root/(trial['name']+'-child.log')).open('x') as log:
                    supervise(trial['command'],env,log,trial['container'],state,save)
                state['state']='child_finished'
            finally:
                try:
                    with (root/(trial['name']+'-server.log')).open('w') as log:
                        subprocess.run(['docker','logs',trial['container']],stdout=log,stderr=subprocess.STDOUT,timeout=10)
                finally:
                    state['server_stop_returncode']=guard.run_server_helper(helper+['stop'],dict(env=env,cwd=ROOT,timeout=25)).returncode
    except Exception as error:
        state.update(state='trial_error',error=repr(error))
    finally:
        cleanup=camera.cleanup_snapshot(trial['container'],state.get('child_pid'))
        try:
            with lease([MODEL_GPU,RENDER_GPU],port=2000):cleanup['all_three_leases_reacquired']=True
        except Exception as error:cleanup.update(verified=False,error=repr(error))
        cleanup['verified'] &= state.get('server_stop_returncode')==0
        state['cleanup']=cleanup
        prefix_path=root/(trial['name']+'-prefix.json')
        try:prefix=read(prefix_path) if prefix_path.exists() else {}
        except (OSError,ValueError) as error:
            prefix={};state['result_error']=repr(error)
        state['prefix_result']=prefix
        state['prefix_pass']=(state.get('child_returncode')==0 and prefix.get('state')=='prefix_complete'
            and prefix.get('forbidden_inference_attempts')==0)
        save()
    return state


def run_study(root):
    protocol=read(root/'protocol.json')
    if protocol['order']!=list(ORDER) or protocol['execution_assignment']!=dict(model_gpu=MODEL_GPU,renderer_gpu=RENDER_GPU):
        raise ValueError('Registered order/assignment changed')
    for name,expected in protocol['source_sha256'].items():
        if digest(ROOT/name)!=expected:raise ValueError('Frozen source changed: '+name)
    for trial,pause in zip(protocol['trials'],ORDER,strict=True):
        if trial['pause_seconds']!=pause or digest(trial['manifest'])!=trial['manifest_sha256']:
            raise ValueError('Trial changed')
        job=read(trial['manifest'])['jobs'][0]
        if job['command']!=trial['command']:
            raise ValueError('Capture command changed')
        for flag,key in (('--config','config_sha256'),('--options','options_sha256')):
            if digest(trial['command'][trial['command'].index(flag)+1])!=job[key]:
                raise ValueError('Capture input changed')
    path=root/'study-status.json'
    if (path.exists() or list(root.glob('*-status.json')) or list(root.glob('*-prefix.json'))
            or any((root/t['name']).exists() for t in protocol['trials'])):
        raise ValueError('Never retry/overwrite a study')
    result=dict(state='running',protocol_sha256=digest(root/'protocol.json'),started_utc=now(),
        trials=[dict(name=t['name'],state='not_attempted') for t in protocol['trials']])
    def interrupt(signum,frame):
        signal.signal(signal.SIGTERM,signal.SIG_IGN);raise InterruptedError('Study interrupted')
    old_handler=signal.signal(signal.SIGTERM,interrupt);write(path,result)
    try:
        for i,trial in enumerate(protocol['trials']):
            value=run_trial(trial,root)
            result['trials'][i]=dict(name=trial['name'],state=value['state'],prefix_pass=value['prefix_pass'],
                cleanup_verified=value['cleanup']['verified'],error=value.get('error'))
            write(path,result)
            if (not value.get('start_attempted') or not value['cleanup']['verified'] or value.get('result_error')
                    or value['prefix_result'].get('forbidden_inference_attempts',0)
                    or 'InterruptedError' in value.get('error','')):
                result['state']='stopped_before_next_trial';break
        else:result['state']='completed_all_planned_trials'
    except BaseException as error:
        result.update(state='interrupted_or_error',error=repr(error));raise
    finally:
        result['finished_utc']=now();write(path,result);signal.signal(signal.SIGTERM,old_handler)
    if result['state']!='completed_all_planned_trials':raise SystemExit(1)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('prepare','run','child'))
    args,rest=parser.parse_known_args()
    if args.action=='child':child(rest);return
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    if args.action=='prepare':
        parser.add_argument('--left-manifest',type=Path,required=True)
        parser.add_argument('--repair-source',type=Path,required=True)
    args2=parser.parse_args(rest)
    if args.action=='prepare':prepare(args2.output.resolve(),args2.left_manifest,args2.repair_source)
    else:run_study(args2.output.resolve())


if __name__=='__main__':main()
