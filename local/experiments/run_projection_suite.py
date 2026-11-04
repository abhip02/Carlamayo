"""Run declared steering/selection comparisons and preserve every outcome."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import carla
import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--single-gpu-model',action='store_true')
    parser.add_argument('--stop-on-failure',action='store_true',help='Stop after the first failed or unaudited capture')
    parser.add_argument('--server-script',type=Path,default=Path('local/server.sh'))
    parser.add_argument('--steering-modes',nargs='+',choices=('native','map','route'),default=['native','map'])
    parser.add_argument('--speed-selection-attempts',type=int,default=1,choices=range(1,9))
    parser.add_argument('--cases',type=int,nargs='+',default=[1,4])
    parser.add_argument('--manifest',type=Path,default=Path('local/experiments/presets/five-short-scenarios.json'))
    parser.add_argument('--speed-cap-mps',type=float)
    parser.add_argument('--steering-gain',type=float,default=1.)
    parser.add_argument('--cache-age-s',type=float,default=0.)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    out=args.output.resolve()
    server_script=args.server_script.resolve()
    out.mkdir(parents=True,exist_ok=False)
    manifest_path=args.manifest.resolve()
    manifest=json.loads(manifest_path.read_text())
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    if len(set(args.cases)) != len(args.cases) or any(n < 1 or n > len(manifest['scenarios']) for n in args.cases):
        raise ValueError('Case numbers must be unique valid manifest indices')
    base=yaml.safe_load((root/manifest['base_config']).read_text())
    status=dict(state='preparing',started_unix=time.time(),results=[],
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
        model_gpus=os.environ.get('CUDA_VISIBLE_DEVICES'),
        model_placement='single_gpu' if args.single_gpu_model else 'sharded',
        server_script=str(server_script),server_script_sha256=hashlib.sha256(server_script.read_bytes()).hexdigest(),
        steering_modes=args.steering_modes,speed_selection_attempts=args.speed_selection_attempts,
        cases=args.cases, manifest=str(manifest_path),
        manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        renderer_gpu=os.environ.get('CARLAMAYO_RENDER_GPU'),
        renderer_container=os.environ.get('CARLAMAYO_PROJECTION_CONTAINER'),
        controller_speed_cap_mps=args.speed_cap_mps,steering_gain=args.steering_gain,
        projection_cache_age_s=args.cache_age_s,
        scope='One candidate per inference; explicit bounded native-speed selection and bounded cache only if requested. Route mode constrains projection to the commanded map corridor.')
    sources={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
             for folder in ('module','local') for p in (root/folder).rglob('*.py')}
    status['source_sha256']=sources
    def save():
        temporary=out/'status.tmp'
        temporary.write_text(json.dumps(status,indent=2)+'\n')
        temporary.replace(out/'status.json')
    jobs=[]
    for number in args.cases:
        index=number-1
        case=manifest['scenarios'][index]
        for mode in args.steering_modes:
            name=f"{case['route']}-{mode}" if args.cases == [1,4] and len(manifest['scenarios']) == 5 else f"{case['name']}-{mode}"
            config=copy.deepcopy(base)
            config['duration_s']=case['duration_s']
            for key in ('junction_entry', 'overhead'):
                if key in case:config[key]=copy.deepcopy(case[key])
            config['rollouts']=[{k:case[k] for k in ('name','distance_before_intersection_m','lateral_offset_m','speed_mps','heading_offset_deg')}]
            options=dict(steering_mode=mode,num_samples=1,seed=case['seed'],flow_seed=case['flow_seed'],
                flow_temperature=1.,guard_trajectory_format=True,acknowledged_preroll=True,
                acknowledged_controls=True,wait_for_snapshot=True,verify_signal=True,
                signal_check_interval=5,validation_routes='auto',validation_route=case['route'],
                instruction=case.get('instruction',f"Turn {case['route']} at the intersection." if case['route'] in ('left','right') else 'Continue straight through the intersection.'))
            for key in ('route_spec','scene_policy','traffic_light_state','mission','hold_visual_stop'):
                if key in case:options[key]=copy.deepcopy(case[key])
            config['instruction']=options['instruction']
            if args.speed_cap_mps is not None:
                options['controller_speed_cap_mps']=args.speed_cap_mps
            if args.steering_gain != 1.:
                options['steering_gain']=args.steering_gain
            if args.cache_age_s:
                options['projection_cache_age_s']=args.cache_age_s
            if args.speed_selection_attempts > 1:
                options.update(speed_selection_attempts=args.speed_selection_attempts,
                               speed_preference_mps=[3.,5.])
            if args.single_gpu_model:
                options['model_placement']='single_gpu'
            config_path,options_path=out/(name+'.yaml'),out/(name+'-options.json')
            config_path.write_text(yaml.safe_dump(config,sort_keys=False))
            options_path.write_text(json.dumps(options,indent=2)+'\n')
            command=[sys.executable,'-m','local.experiments.projected_probe','capture',
                '--config',str(config_path),'--options',str(options_path),'--duration',str(case['duration_s']),
                '--capture-frames','0','--output',str(out/name)]
            jobs.append((name,command))
    status['queue']=[n for n,_ in jobs]
    save()
    try:
        subprocess.run(['bash',str(server_script),'start'],cwd=root,check=True)
        for _ in range(60):
            try:
                client=carla.Client('127.0.0.1',2000)
                client.set_timeout(2)
                client.get_server_version()
                break
            except RuntimeError:
                time.sleep(2)
        else:
            raise RuntimeError('CARLA did not become ready')
        for name,command in jobs:
            if hashlib.sha256(manifest_path.read_bytes()).hexdigest()!=status['manifest_sha256']:
                raise RuntimeError('Frozen manifest changed')
            for path,digest in sources.items():
                if hashlib.sha256((root/path).read_bytes()).hexdigest()!=digest:
                    raise RuntimeError(f'Frozen source changed: {path}')
            status.update(state='running',current=name)
            status['queue'].remove(name)
            save()
            print('START',name,flush=True)
            started=time.monotonic()
            with (out/(name+'.log')).open('w') as stream:
                completed=subprocess.run(command,cwd=root,stdout=stream,stderr=subprocess.STDOUT)
            result=dict(name=name,returncode=completed.returncode,wall_seconds=time.monotonic()-started)
            capture=out/name
            if list(capture.glob('control_*.json')):
                with (out/(name+'-audit.log')).open('w') as stream:
                    audited=subprocess.run([sys.executable,'-m','local.experiments.audit_projection',str(capture)],
                        cwd=root,stdout=stream,stderr=subprocess.STDOUT)
                result['audit_returncode']=audited.returncode
                report=capture/'projection-audit.json'
                if report.exists():
                    data=json.loads(report.read_text())
                    result['audit']={k:v for k,v in data.items() if k not in ('projection_trace','near_stop_times_s','body_departure_times_s')}
                source=capture/'000_verifai/overhead.mp4'
                video=out/(name+'-review.mp4')
                if source.exists():
                    convert=subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-i',str(source),
                        '-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p',
                        '-movflags','+faststart','-an',str(video)])
                    result['video_returncode']=convert.returncode
                    result['video']=str(video)
            status['results'].append(result)
            save()
            print('FINISHED',json.dumps(result),flush=True)
            if args.stop_on_failure and not result.get('audit',{}).get('passed',False):
                break
        status.update(state='stopped_after_failure' if status['queue'] else 'completed',current=None,finished_unix=time.time())
    except BaseException as exc:
        status.update(state='interrupted',error=repr(exc),finished_unix=time.time())
        raise
    finally:
        status['server_stop_returncode']=subprocess.run(['bash',str(server_script),'stop'],cwd=root).returncode
        save()


if __name__=='__main__':
    main()
