"""Run declared steering/selection comparisons and preserve every outcome."""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import carla
import yaml


def traffic_trajectory_generation(case):
    mode = case.get('traffic', {}).get('trajectory_generation', 'reasoned')
    if mode not in ('reasoned', 'direct'):
        raise ValueError('Traffic trajectory generation must be reasoned or direct')
    return mode


def native_flow_temperature(case):
    value = case.get('traffic', {}).get('flow_temperature', 1.)
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not 0. <= value <= 1.):
        raise ValueError('Traffic flow temperature must be finite and between 0 and 1')
    return float(value)


def declared_traffic_speed_cap(case, requested=None):
    value = case.get('traffic', {}).get('controller_speed_cap_mps')
    if value is None:
        return requested
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not 1. <= value <= 4.5):
        raise ValueError('Traffic speed ceiling must be finite and between 1 and 4.5m/s')
    if requested is not None and requested != value:
        raise ValueError('Requested speed ceiling differs from the traffic declaration')
    return float(value)


def validate_motion_commitment(commitment, base_interval, rest_interval=None):
    if commitment is None:
        return
    if (isinstance(commitment, bool) or not isinstance(commitment, (int, float))
            or not math.isfinite(commitment) or not .1 <= commitment <= .5
            or abs(commitment * 10 - round(commitment * 10)) > 1e-6
            or base_interval is None or not math.isfinite(base_interval)
            or base_interval + commitment > 3. + 1e-8 or rest_interval is not None):
        raise ValueError('Native motion commitment requires fresh planning, a0.1–0.5s duration on the0.1s grid, base+duration<=3s, and no rest extension')


def validate_deceleration_interval(value, base_interval, fine_interval, signal_only):
    if value is None:
        return
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or isinstance(base_interval,bool) or not isinstance(base_interval,(int,float))
            or not math.isfinite(base_interval) or not .1<=base_interval<=3.
            or not .1 <= value <= base_interval
            or abs(value * 10 - round(value * 10)) > 1e-6
            or isinstance(fine_interval,bool) or not isinstance(fine_interval,(int,float))
            or not math.isfinite(fine_interval) or not .1<=fine_interval<=value
            or signal_only is not True):
        raise ValueError('Separate deceleration planning requires a0.1s-grid interval between the signal-only fine interval and base interval')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--single-gpu-model',action='store_true')
    parser.add_argument('--capture-all-native-inputs',action='store_true',
                        help='Diagnostic only: save each requested native input; all recording time counts')
    parser.add_argument('--async-deceleration-planning-interval-s',type=float,
                        help='Optional shorter fresh interval during measured braking; signal-only fine updates remain separate')
    parser.add_argument('--async-inference',action='store_true',help='Use the shared asynchronous worker with source-pose anchoring and expiry braking')
    parser.add_argument('--async-max-pending-sim-s',type=float,
                        help='Pause physics after this much simulation time while a request is pending')
    parser.add_argument('--async-retention-age-s',type=float,default=0.)
    parser.add_argument('--async-fine-planning-interval-s',type=float,
                        help='Shorter fresh planning interval while moving with a visual stop signal or measured deceleration of at least 0.5m/s^2')
    parser.add_argument('--async-fine-planning-signal-only',action='store_true',
                        help='Use the fine interval only with model visual red/yellow, not deceleration alone')
    parser.add_argument('--async-fresh-planning-interval-s',type=float,
                        help='Optional sparse planning with physics paused for current-state inference at these simulation intervals')
    parser.add_argument('--async-motion-commitment-s',type=float,
                        help='Briefly execute native motion that begins before the normal planning boundary; maximum0.5s after its onset')
    parser.add_argument('--async-rest-planning-interval-s',type=float,
                        help='Bounded interval for plans whose original measured speed is at most0.2m/s; earlier fine/heading/navigation refreshes still apply')
    parser.add_argument('--async-revalidate-delayed-plans',action='store_true',
                        help='Reapply existing moving-plan checks at actual execution age; pause for fresh inference when needed')
    parser.add_argument('--async-max-pending-heading-deg',type=float,
                        help='Pause physics when heading changes by this much while inference is pending')
    parser.add_argument('--stop-on-failure',action='store_true',help='Stop after the first failed or unaudited capture')
    parser.add_argument('--require-pass',action='store_true',help='Return a nonzero exit code unless every requested capture passes')
    parser.add_argument('--server-script',type=Path,default=Path('local/server.sh'))
    parser.add_argument('--steering-modes',nargs='+',choices=('native','map','route'),default=['native','map'])
    parser.add_argument('--speed-selection-attempts',type=int,default=1,choices=range(1,9))
    parser.add_argument('--speed-preference-mps', type=float, nargs=2, default=[3., 5.],
                        metavar=('LOW', 'HIGH'), help='Native speed ranking preference, not a speed floor')
    parser.add_argument('--speed-selection-max-seconds', type=float,
                        help='Stop starting retries after this time; an in-progress call may exceed it')
    parser.add_argument('--cases',type=int,nargs='+',default=[1,4])
    parser.add_argument('--manifest',type=Path,default=Path('local/experiments/presets/five-short-scenarios.json'))
    parser.add_argument('--speed-cap-mps',type=float)
    parser.add_argument('--steering-gain',type=float,default=1.)
    parser.add_argument('--native-lookahead-min-m',type=float,
                        help='Explicit native-path lookahead minimum; default preserves upstream behavior')
    parser.add_argument('--cache-age-s',type=float,default=0.)
    parser.add_argument('--seed-policy', choices=('fixed','advancing_separate','advancing_joint'), default='fixed')
    parser.add_argument('--camera-postprocessing', choices=('on','off'))
    args=parser.parse_args()
    if args.seed_policy != 'fixed' or args.camera_postprocessing is not None:
        if args.steering_modes != ['native'] or args.speed_selection_attempts != 1 or args.cache_age_s or args.async_retention_age_s:
            parser.error('Seed/camera experiments require native steering and one unselected prediction')
        if args.seed_policy != 'fixed' and not args.async_inference:
            parser.error('Advancing seeds require async inference request ordinals')
    if args.native_lookahead_min_m is not None:
        if args.steering_modes != ['native'] or not math.isfinite(args.native_lookahead_min_m) or not 0 < args.native_lookahead_min_m <= 12:
            parser.error('Native lookahead requires only native steering and a finite minimum in (0,12] metres')
    if args.async_inference and args.cache_age_s:
        parser.error('Async mode does not support motion-ranked history')
    if args.async_retention_age_s:
        if not args.async_inference or args.speed_selection_attempts<2 or not math.isfinite(args.async_retention_age_s) or not 0<args.async_retention_age_s<=3:
            parser.error('Async retention requires bounded selection and a positive age at most3seconds')
    if args.async_max_pending_sim_s is not None:
        limit = args.async_max_pending_sim_s
        if not args.async_inference or not math.isfinite(limit) or not .1 <= limit <= 3. or abs(limit*10-round(limit*10)) > 1e-6:
            parser.error('Async simulation advance limit must be a multiple of 0.1 seconds between 0.1 and 3')
    if args.async_revalidate_delayed_plans and not args.async_retention_age_s:
        parser.error('Delayed-plan revalidation requires the original async retention admission policy')
    if args.async_max_pending_heading_deg is not None:
        limit=args.async_max_pending_heading_deg
        if not args.async_inference or not math.isfinite(limit) or not 1 <= limit <= 90:
            parser.error('Async heading limit must be finite and between 1 and 90 degrees')
    if args.async_fresh_planning_interval_s is not None:
        interval=args.async_fresh_planning_interval_s
        if not args.async_inference or not math.isfinite(interval) or not .1<=interval<=3 or abs(interval*10-round(interval*10))>1e-6:
            parser.error('Fresh planning interval must be a multiple of 0.1 seconds between 0.1 and 3')
    if args.async_fine_planning_interval_s is not None:
        fine=args.async_fine_planning_interval_s
        if (args.async_fresh_planning_interval_s is None or not math.isfinite(fine)
                or not .1<=fine<=args.async_fresh_planning_interval_s or abs(fine*10-round(fine*10))>1e-6):
            parser.error('Fine planning requires fresh planning and a multiple of 0.1 seconds no larger than its base interval')
    if args.async_rest_planning_interval_s is not None:
        rest=args.async_rest_planning_interval_s
        if (args.async_fresh_planning_interval_s is None or not math.isfinite(rest)
                or not args.async_fresh_planning_interval_s<=rest<=3. or abs(rest*10-round(rest*10))>1e-6):
            parser.error('Rest planning must be a multiple of0.1seconds between the base interval and3seconds')
    if args.async_fine_planning_signal_only and args.async_fine_planning_interval_s is None:
        parser.error('Signal-only fine planning requires a declared fine interval')
    try:
        validate_deceleration_interval(args.async_deceleration_planning_interval_s, args.async_fresh_planning_interval_s, args.async_fine_planning_interval_s, args.async_fine_planning_signal_only)
        validate_motion_commitment(args.async_motion_commitment_s, args.async_fresh_planning_interval_s, args.async_rest_planning_interval_s)
    except ValueError as error:
        parser.error(str(error))
    low, high = args.speed_preference_mps
    if not all(math.isfinite(v) for v in (low, high)) or not 0 < low < high:
        parser.error('speed preference must be finite, positive and increasing')
    if args.speed_selection_max_seconds is not None:
        if not math.isfinite(args.speed_selection_max_seconds) or args.speed_selection_max_seconds <= 0:
            parser.error('sampling time budget must be positive and finite')
        if args.speed_selection_attempts < 2:
            parser.error('sampling time budget requires bounded speed selection (at least two attempts)')
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
        speed_preference_mps=args.speed_preference_mps,
        speed_selection_max_seconds=args.speed_selection_max_seconds,
        capture_all_native_inputs=args.capture_all_native_inputs,
        cases=args.cases, manifest=str(manifest_path),
        manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        renderer_gpu=os.environ.get('CARLAMAYO_RENDER_GPU'),
        renderer_container=os.environ.get('CARLAMAYO_PROJECTION_CONTAINER'),
        controller_speed_cap_mps=args.speed_cap_mps,steering_gain=args.steering_gain,
        native_lookahead_min_m=args.native_lookahead_min_m,
        seed_policy=args.seed_policy,camera_postprocessing=args.camera_postprocessing,
        projection_cache_age_s=args.cache_age_s,
        async_max_pending_sim_s=args.async_max_pending_sim_s,
        async_max_pending_heading_deg=args.async_max_pending_heading_deg,
        async_retention_age_s=args.async_retention_age_s,
        async_fresh_planning_interval_s=args.async_fresh_planning_interval_s,
        async_motion_commitment_s=args.async_motion_commitment_s,
        async_rest_planning_interval_s=args.async_rest_planning_interval_s,
        async_fine_planning_interval_s=args.async_fine_planning_interval_s,
        async_fine_planning_signal_only=args.async_fine_planning_signal_only,
        async_deceleration_planning_interval_s=args.async_deceleration_planning_interval_s,
        async_revalidate_delayed_plans=args.async_revalidate_delayed_plans,
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
            if args.async_inference:
                config['async_inference']=True
            config['duration_s']=case['duration_s']
            for key in ('junction_entry', 'overhead'):
                if key in case:config[key]=copy.deepcopy(case[key])
            config['rollouts']=[{k:case[k] for k in ('name','distance_before_intersection_m','lateral_offset_m','speed_mps','heading_offset_deg')}]
            options=dict(steering_mode=mode,num_samples=1,seed=case['seed'],flow_seed=case['flow_seed'],
                flow_temperature=native_flow_temperature(case),guard_trajectory_format=True,acknowledged_preroll=True,
                acknowledged_controls=True,wait_for_snapshot=True,verify_signal=True,
                signal_check_interval=5,validation_routes='auto',validation_route=case['route'],
                instruction=case.get('instruction',f"Turn {case['route']} at the intersection." if case['route'] in ('left','right') else 'Continue straight through the intersection.'))
            for key in ('route_spec','scene_policy','traffic_light_state','mission','hold_visual_stop','traffic'):
                if key in case:options[key]=copy.deepcopy(case[key])
            traffic_trajectory_generation(case)  # Validate before starting capture.
            options['seed_policy'] = args.seed_policy
            if args.camera_postprocessing is not None:
                options['camera_enable_postprocess_effects'] = args.camera_postprocessing == 'on'
            config['instruction']=options['instruction']
            if args.async_inference:
                options['async_inference']=True
                if args.async_max_pending_sim_s is not None:
                    options['async_max_pending_sim_s']=args.async_max_pending_sim_s
                if args.async_retention_age_s:
                    options['async_retention_age_s']=args.async_retention_age_s
                if args.async_revalidate_delayed_plans:
                    options['async_revalidate_delayed_plans']=True
                if args.async_fresh_planning_interval_s is not None:
                    options['async_fresh_planning_interval_s']=args.async_fresh_planning_interval_s
                if args.async_motion_commitment_s is not None:
                    options['async_motion_commitment_s']=args.async_motion_commitment_s
                if args.async_rest_planning_interval_s is not None:
                    options['async_rest_planning_interval_s']=args.async_rest_planning_interval_s
                if args.async_fine_planning_interval_s is not None:
                    options['async_fine_planning_interval_s']=args.async_fine_planning_interval_s
                if args.async_fine_planning_signal_only:
                    options['async_fine_planning_signal_only']=True
                if args.async_deceleration_planning_interval_s is not None:
                    options['async_deceleration_planning_interval_s']=args.async_deceleration_planning_interval_s
                if args.async_max_pending_heading_deg is not None:
                    options['async_max_pending_heading_deg']=args.async_max_pending_heading_deg
            speed_cap = declared_traffic_speed_cap(case, args.speed_cap_mps)
            if speed_cap is not None:
                options['controller_speed_cap_mps']=speed_cap
            if args.steering_gain != 1.:
                options['steering_gain']=args.steering_gain
            if args.native_lookahead_min_m is not None:
                options['native_lookahead_min_m']=args.native_lookahead_min_m
            if args.cache_age_s:
                options['projection_cache_age_s']=args.cache_age_s
            if args.speed_selection_attempts > 1:
                options.update(speed_selection_attempts=args.speed_selection_attempts,
                               speed_preference_mps=args.speed_preference_mps)
            if args.speed_selection_max_seconds is not None:
                options['speed_selection_max_seconds'] = args.speed_selection_max_seconds
            if args.single_gpu_model:
                options['model_placement']='single_gpu'
            config_path,options_path=out/(name+'.yaml'),out/(name+'-options.json')
            config_path.write_text(yaml.safe_dump(config,sort_keys=False))
            options_path.write_text(json.dumps(options,indent=2)+'\n')
            module='local.experiments.async_projected_probe' if args.async_inference else 'local.experiments.projected_probe'
            if case.get('traffic'):
                module='local.experiments.traffic_probe'
            command=[sys.executable,'-m',module,'capture',
                '--config',str(config_path),'--options',str(options_path),'--duration',str(case['duration_s']),
                '--capture-frames',*([str(i) for i in range(math.ceil(case['duration_s']/.1)+1)]
                                    if args.capture_all_native_inputs else ['0']),
                '--output',str(out/name)]
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
                    options=json.loads((out/(name+'-options.json')).read_text())
                    audit_module='local.experiments.audit_traffic' if options.get('traffic') else 'local.experiments.audit_projection'
                    audited=subprocess.run([sys.executable,'-m',audit_module,str(capture)],
                        cwd=root,stdout=stream,stderr=subprocess.STDOUT)
                result['audit_returncode']=audited.returncode
                report=capture/('traffic-audit.json' if options.get('traffic') else 'projection-audit.json')
                if report.exists():
                    data=json.loads(report.read_text())
                    result['audit']={k:v for k,v in data.items() if k not in ('projection_trace','near_stop_times_s','body_departure_times_s','geometric_audit')}
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
        status['passed'] = len(status['results']) == len(jobs) and all(
            result.get('audit', {}).get('passed', False)
            and all(result.get(key) == 0 for key in ('returncode', 'audit_returncode', 'video_returncode'))
            for result in status['results'])
    except BaseException as exc:
        status.update(state='interrupted',error=repr(exc),finished_unix=time.time())
        raise
    finally:
        status['server_stop_returncode']=subprocess.run(['bash',str(server_script),'stop'],cwd=root).returncode
        save()
    if args.require_pass and (not status['passed'] or status['server_stop_returncode']):
        raise SystemExit(1)


if __name__=='__main__':
    main()
