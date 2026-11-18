"""Bounded async validation using unchanged round-19 cases and driving criteria."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

from local.experiments.resource_lease import acquire
from local.experiments.run_latency_pilot import stop_worker
from local.experiments.run_traffic_panel import available, container_state, save


class SimulatorStartupCrash(RuntimeError):
    """The owned simulator died before the first inference or control."""


def traffic_generation_mode(manifest):
    from local.experiments.run_projection_suite import traffic_trajectory_generation
    mode = traffic_trajectory_generation({'traffic': manifest.get('traffic_protocol', {})})
    if any(traffic_trajectory_generation(case) != mode for case in manifest['scenarios']):
        raise ValueError('Every traffic case must use the declared trajectory generation mode')
    return mode


def traffic_speed_preview(manifest):
    lead = manifest.get('traffic_protocol', {}).get('speed_preview_lead_s', 1.)
    if not math.isfinite(lead) or not 0. <= lead <= 1. or abs(lead * 10 - round(lead * 10)) > 1e-6:
        raise ValueError('Traffic speed preview must be a multiple of0.1seconds between0and1second')
    if any(case['traffic'].get('speed_preview_lead_s', 1.) != lead for case in manifest['scenarios']):
        raise ValueError('Every traffic case must use the declared native speed preview')
    return lead


def traffic_flow_temperature(manifest):
    from local.experiments.run_projection_suite import native_flow_temperature
    value = native_flow_temperature({'traffic': manifest.get('traffic_protocol', {})})
    if any(native_flow_temperature(case) != value for case in manifest['scenarios']):
        raise ValueError('Every traffic case must use the declared flow temperature')
    return value


def traffic_speed_cap(manifest):
    from local.experiments.run_projection_suite import declared_traffic_speed_cap
    value = declared_traffic_speed_cap({'traffic': manifest.get('traffic_protocol', {})})
    value = 4.5 if value is None else value
    for case in manifest['scenarios']:
        cap = declared_traffic_speed_cap(case)
        if (4.5 if cap is None else cap) != value:
            raise ValueError('Every traffic case must use the declared speed ceiling')
    return value


def completed_traffic_attempt(result):
    """A fully audited driving failure may be followed by another independent case."""
    if (result.get('timed_out') or result.get('stop_returncode') != 0
            or result.get('server_after_stop', {}).get('Running') is not False
            or result.get('returncode') not in (0, 1)):
        return False
    captures = result.get('suite', {}).get('results', [])
    if len(captures) != 1:
        return False
    capture = captures[0]
    audit = capture.get('audit', {})
    verdict = audit.get('passed')
    return (capture.get('returncode') == 0 and capture.get('video_returncode') == 0
            and isinstance(verdict, bool) and audit.get('integrity_pass') is True
            and capture.get('audit_returncode') == int(not verdict))


def no_driving_started(case_directory):
    capture=case_directory/(case_directory.name+'-route')
    if any(next(capture.glob(pattern),None) is not None for pattern in
           ('frame_*.json','control_*.json','proposal_*.json','async_request_*.json')):
        return False
    events=capture/'async-events.jsonl'
    return not events.exists() or not events.read_text().strip()


def confirmed_startup_crash(suite_path, container):
    case_directory=suite_path.parent
    log=case_directory/(case_directory.name+'-route.log')
    if not log.exists() or 'CARLA server ' not in log.read_text(errors='replace'):
        return None
    if not no_driving_started(case_directory):
        return None
    try:
        state=json.loads(subprocess.check_output(['docker','inspect','--format',
            '{{json .State}}',container],text=True,timeout=5,stderr=subprocess.DEVNULL))
    except (subprocess.SubprocessError,json.JSONDecodeError):
        return None
    if (state.get('Status')=='exited' and state.get('Running') is False
            and state.get('ExitCode')==139 and state.get('OOMKilled') is False):
        return dict(reason='CARLA signal11 after successful server connection, before inference or control',
                    container_state=state,checked_unix=time.time())
    return None


def retry_startup_failure(result, case_directory, attempt):
    return bool(attempt==0 and result.get('startup_failure')
                and result.get('stop_returncode')==0
                and result.get('server_after_stop',{}).get('Running') is False
                and no_driving_started(case_directory))


def archive_startup_failure(output, name, result):
    destination=output/'startup-failures'/name/'attempt-1'
    destination.parent.mkdir(parents=True,exist_ok=True)
    if destination.exists():
        raise FileExistsError(destination)
    shutil.move(str(output/name),str(destination))
    for suffix in ('.log','-server.log'):
        path=output/(name+suffix)
        if path.exists():
            shutil.move(str(path),str(destination/('supervisor'+suffix)))
    result['artifact_directory']=str(destination)
    result['artifact_note']='Original bytes preserved. Embedded command paths describe the original location before this failed startup was archived.'
    save(destination/'startup-failure.json',result)
    return result


def wait_for_case(worker, suite_path, result, *, container=None, work_timeout_s=600):
    """Keep the work deadline, with bounded teardown time after verified work."""
    try:
        if container is None:
            return worker.wait(timeout=work_timeout_s)
        deadline=time.monotonic()+work_timeout_s
        while True:
            remaining=deadline-time.monotonic()
            if remaining<=0:
                raise subprocess.TimeoutExpired('owned-case',work_timeout_s)
            try:
                return worker.wait(timeout=min(5,remaining))
            except subprocess.TimeoutExpired:
                failure=confirmed_startup_crash(suite_path,container)
                if failure is not None:
                    result['startup_failure']=failure
                    raise SimulatorStartupCrash(failure['reason'])
    except subprocess.TimeoutExpired as timeout:
        try:
            suite=json.loads(suite_path.read_text())
        except (FileNotFoundError,json.JSONDecodeError):
            raise timeout
        captures=suite.get('results',[])
        if len(captures)!=1 or not all(captures[0].get(key)==0
                for key in ('returncode','audit_returncode','video_returncode')):
            raise
        # run_projection_suite saves this record only after capture, auditing
        # and video encoding finish, before its final simulator shutdown.
        # This allowance never extends active inference or relaxes a verdict.
        result['cleanup_grace_seconds']=40
        return worker.wait(timeout=40)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--container',required=True)
    parser.add_argument('--manifest',type=Path,
                        help='Optional declared traffic extension; default is the original ten scenes')
    parser.add_argument('--cases',type=int,nargs='+',default=[1,2,10])
    parser.add_argument('--capture-all-native-inputs',action='store_true',
                        help='Diagnostic only: save every requested native input; does not add requests')
    parser.add_argument('--max-pending-sim-s',type=float)
    parser.add_argument('--max-pending-heading-deg',type=float)
    parser.add_argument('--speed-selection-attempts',type=int,default=1,choices=range(1,9))
    parser.add_argument('--retention-age-s',type=float,default=0.)
    parser.add_argument('--revalidate-delayed-plans',action='store_true')
    parser.add_argument('--fresh-planning-interval-s',type=float)
    parser.add_argument('--motion-commitment-s',type=float)
    parser.add_argument('--deceleration-planning-interval-s',type=float)
    parser.add_argument('--rest-planning-interval-s',type=float)
    parser.add_argument('--fine-planning-interval-s',type=float)
    parser.add_argument('--fine-planning-signal-only',action='store_true')
    parser.add_argument('--case-work-timeout-s',type=int,choices=(600,900),default=600)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    output=args.output.resolve()
    if output.exists():
        raise FileExistsError(output)
    if len(set(args.cases))!=len(args.cases) or any(n not in range(1,11) for n in args.cases):
        raise ValueError('Choose distinct case numbers from the original ten-scene manifest')
    if args.max_pending_sim_s is not None:
        limit=args.max_pending_sim_s
        if not math.isfinite(limit) or not .1 <= limit <= 3. or abs(limit*10-round(limit*10)) > 1e-6:
            raise ValueError('Simulation advance limit must be a multiple of 0.1 seconds between 0.1 and 3')
    if args.max_pending_heading_deg is not None:
        if not math.isfinite(args.max_pending_heading_deg) or not 1 <= args.max_pending_heading_deg <= 90:
            raise ValueError('Heading limit must be finite and between 1 and 90 degrees')
    if args.retention_age_s and (args.speed_selection_attempts<2 or not math.isfinite(args.retention_age_s) or not 0<args.retention_age_s<=3):
        raise ValueError('Async retention requires bounded selection and an age at most3seconds')
    if args.revalidate_delayed_plans and not args.retention_age_s:
        raise ValueError('Delayed-plan revalidation requires original retention admission checks')
    if args.fresh_planning_interval_s is not None:
        interval=args.fresh_planning_interval_s
        if not math.isfinite(interval) or not .1<=interval<=3 or abs(interval*10-round(interval*10))>1e-6:
            raise ValueError('Fresh planning interval must be a multiple of 0.1 seconds between 0.1 and 3')
    if args.fine_planning_interval_s is not None:
        fine=args.fine_planning_interval_s
        if (args.fresh_planning_interval_s is None or not math.isfinite(fine)
                or not .1<=fine<=args.fresh_planning_interval_s or abs(fine*10-round(fine*10))>1e-6):
            raise ValueError('Fine planning requires fresh planning and a multiple of 0.1 seconds no larger than its base interval')
    if args.fine_planning_signal_only and args.fine_planning_interval_s is None:
        raise ValueError('Signal-only fine planning requires a declared fine interval')
    if args.rest_planning_interval_s is not None:
        rest=args.rest_planning_interval_s
        if (args.fresh_planning_interval_s is None or not math.isfinite(rest)
                or not args.fresh_planning_interval_s<=rest<=3. or abs(rest*10-round(rest*10))>1e-6):
            raise ValueError('Rest planning must be a multiple of0.1seconds between the base interval and3seconds')
    from local.experiments.run_projection_suite import validate_deceleration_interval
    validate_deceleration_interval(args.deceleration_planning_interval_s, args.fresh_planning_interval_s, args.fine_planning_interval_s, args.fine_planning_signal_only)
    if args.motion_commitment_s is not None:
        from local.experiments.run_projection_suite import validate_motion_commitment
        validate_motion_commitment(args.motion_commitment_s, args.fresh_planning_interval_s, args.rest_planning_interval_s)
    model,renderer=os.environ['CARLAMAYO_MODEL_GPUS'],os.environ['CARLAMAYO_RENDER_GPU']
    environment={**os.environ,'CARLAMAYO_PROJECTION_CONTAINER':args.container}
    manifest_path=(args.manifest.resolve() if args.manifest else root/'local/experiments/presets/ten-driving-scenes.json')
    manifest=json.loads(manifest_path.read_text())
    traffic_suite=any(case.get('traffic') for case in manifest['scenarios'])
    if traffic_suite and (args.speed_selection_attempts!=1 or args.retention_age_s or args.fresh_planning_interval_s is None):
        raise ValueError('Traffic requires single native predictions, no motion-ranked retention, and fresh-state planning')
    traffic_controller = manifest.get('traffic_protocol', {}).get('controller_policy')
    if traffic_suite and any(case['traffic'].get('controller_policy') != traffic_controller
                             for case in manifest['scenarios']):
        raise ValueError('Every traffic case must use the declared controller policy')
    generation_mode = traffic_generation_mode(manifest) if traffic_suite else 'reasoned'
    preview_lead = traffic_speed_preview(manifest) if traffic_suite else None
    flow_temperature = traffic_flow_temperature(manifest) if traffic_suite else 1.
    speed_cap = traffic_speed_cap(manifest) if traffic_suite else 4.5
    sources={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
             for directory in ('module','local') for p in (root/directory).rglob('*.py')}
    protocol=dict(cases=[dict(number=n,scenario=manifest['scenarios'][n-1]) for n in args.cases],
        manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(), source_sha256=sources,
        capture_all_native_inputs=args.capture_all_native_inputs,
        input_recording='Initial/final inputs by default. Optional full recording saves every requested native input through the existing capture path; it adds no inference call and changes no native input. Recording wall time counts.',
        model_gpu=model,renderer_gpu=renderer,
        model='Existing BF16/SDPA Alpamayo1.5 checkpoint.',
        speed_selection_attempts=args.speed_selection_attempts,
        selection='Original bounded native speed preference, if attempts>1; always one sample on the red-stop scene, preserve visual red/yellow. Optional original retention is rechecked at actual execution pose/time.',
        async_retention_age_s=args.retention_age_s,
        retention_navigation='Match original round19 eligibility: navigation instruction changes do not erase retained native plans that pass the original age, current-pose, route-progress and speed admission checks. No age extension or reanchoring. Pending results from an outdated navigation phase are still discarded, and latest visual red/yellow still prevents motion-ranked retention.',
        fresh_planning_interval_s=args.fresh_planning_interval_s,
        motion_commitment_s=args.motion_commitment_s,
        motion_commitment='Optional0.1–0.5s execution window after the first native speed preview above the existing stop threshold, only when that onset is BEFORE the ordinary base refresh. Source ego speed<=0.2m/s, initial preview below the stop threshold, no reverse velocity, finite64point plan with>=3m forward extent, and no model red/yellow are required. Never wait for motion predicted only after the original boundary. Preserve original source/time, native commands and3s expiry. Fine deceleration/red/yellow, heading and navigation refreshes retain precedence. No extra model query, speed floor or scenario-specific rule.',
        rest_planning_interval_s=args.rest_planning_interval_s,
        rest_planning='Optional longer bounded interval only for a native wait-then-forward plan from measured speed<=0.2m/s: no reverse velocity, final forward extent>=3m, initial speed preview<=0.25m/s and preview near the longer boundary>0.25m/s. An additional camera-history VQA must explicitly report a moving lead or clear lane; stopped, ambiguous and unknown answers retain base cadence. Red/yellow forbids eligibility. All VQA inputs, answers and costs are saved. Native trajectory, RNG and primary model inputs are unchanged. Fine deceleration/visual-stop, heading and navigation refreshes retain precedence. No speed floor, timestamp reset or expiry extension.',
        fine_planning_interval_s=args.fine_planning_interval_s,
        fine_planning_signal_only=args.fine_planning_signal_only,
        deceleration_planning_interval_s=args.deceleration_planning_interval_s,
        separate_deceleration_planning='Optional shorter interval while measured speed exceeds0.05m/s and measured deceleration since the previous control tick is at least0.5m/s^2. Only changes when fresh inference is requested, using ego motion. Signal-only fine red/yellow, heading and navigation boundaries retain precedence; native trajectories, original anchors and3s expiry unchanged. Defaults off.',
        signal_only_planning='When enabled, fine planning is triggered only while moving with model visual red/yellow; observed deceleration alone does not trigger it. The same rule applies to every case, using no scenario identity or stop-line location.',
        fine_planning='Optional uniform shorter interval while measured speed exceeds0.05m/s and the latest model visual signal is red/yellow, or additionally when measured deceleration since the previous control tick is at least0.5m/s^2 unless signal-only planning is enabled. Only the inference schedule changes; no scenario identity or stop-line distance is used. Otherwise use the base or explicitly declared rest interval. All requests pause physics and count fully.',
        fresh_planning='Optional conservative mode: follow the current plan between fixed simulation planning boundaries, shortened by the heading bound or navigation change; pause physics and infer from the current state at each boundary. Uses the same worker and checks, without inference/control overlap. The legacy wall submission interval does not apply in this mode. All waits and samples count.',
        async_revalidate_delayed_plans=args.revalidate_delayed_plans,
        delayed_plan_admission='Optional: apply original actual-pose moving-plan admission to every older native plan in empty-green scenes, including newly delivered plans. Visual red/yellow and the red-stop scene remain native. If none is admissible, pause for pending inference and at most one fresh request from the frozen state. A current-frame native plan still uses the unchanged bounded sampling policy.',
        controller='Original round19 route projection, steering gain1.4, target ceiling4.5m/s and red visual stop hold.',
        scheduling='Existing shared async worker, one pending request, one-second minimum WALL submission interval; simulation/control remains0.1s. First plan computed at the original handover without advancing physics. Original pose/time anchors and3s plan expiry with full braking.',
        max_pending_sim_s=args.max_pending_sim_s,
        max_pending_heading_deg=args.max_pending_heading_deg,
        heading_wait='When set, pause at the first control tick where heading change since the pending request reaches the bound; universal across scenarios, using only recorded ego pose. Original source anchor and controller unchanged.',
        simulation_wait='If an advance limit is set, wait for the pending model request without advancing physics once it is reached. Waiting is counted in capture wall time.',
        retention_wait='If a retained plan becomes inadmissible at execution, pause physics for the pending result and, if needed, at most one fresh request from that frozen state. Never extend the old plan lifetime or repeat the same input frame.',
        inference_seed='Original per-case text and motion seeds, set only in inference worker to avoid RNG races.',
        changes_from_round19='Async requests replace per-tick inference; sampling and retention budgets are explicit. No scenario, endpoint, speed, footprint or stop criteria changed.',
        acceptance='Use unchanged audit_projection and red stop mission pass criteria. Faster failures are not successful trace generation.',
        per_case_timeout_s=args.case_work_timeout_s,
        startup_retry_limit=1,
        startup_retry='At most one retry for a confirmed owned CARLA exit139 after successful server connection, with zero inference events, requests, native proposals, frames or controls. Preserve the failed directory/logs, recheck idle resources and frozen sources, and include all retry time. Never retry a driving or integrity failure. Each attempt keeps the declared work deadline.',
        completed_capture_cleanup_grace_s=40,
        cleanup_grace=f'At the declared {args.case_work_timeout_s}s deadline, allow at most40s for shutdown only when the owned single-case status already records completed capture, independent audit and video encoding with zero return codes. Driving failures remain failures. Partial inference receives no extension beyond the declared deadline; all elapsed time counts.')
    if traffic_suite:
        protocol.update(suite_kind='declared_traffic_extension',
            trajectory_generation=generation_mode,
            generation_policy='Reasoned mode preserves existing text generation. Opt-in direct mode prefills the native trajectory marker and generates one token for its KV cache; the native motion expert still samples one trajectory. Model visual red/yellow retains the existing stop reasoning prefix and256-token budget. Camera/pose inputs, seeds and weights unchanged; generation prompt intentionally differs. No provided moving intent, forced speed or trajectory editing.',
            flow_temperature=flow_temperature,
            controller_speed_cap_mps=speed_cap,
            native_sampling=f'One native prediction with original text and flow seeds and 10 diffusion steps. Explicit upstream initial-noise temperature {flow_temperature}; BF16 weights and inputs unchanged. No resampling or candidate ranking.',
            traffic_input_recording='Save the final completed native request input after worker drain for physical-failure replay. Identify whether it was used by the last control. This adds only post-run recording; all recording time remains in attempt wall time.',
            driving_failure_handling='Continue to the next independent traffic case after a complete capture, valid independent integrity audit, review video and verified cleanup, even if the driving mission failed. Preserve every verdict; any failed case makes the full suite fail. Stop after incomplete execution, failed integrity, timeout or cleanup failure.',
            selection='One native prediction per planning boundary; preserve native slowing and stopping. No motion ranking or substitute history.',
            acceptance='Traffic missions declared in the manifest: original route/footprint and stop-line geometry, collision/clearance checks, lead motion/proximity, endpoint or final stop hold as applicable. Original empty-road verdict is retained separately.',
            changes_from_round19='Add one declared lead vehicle on each original route. A braking-lead case requires a final stop instead of passing through the obstacle; moving-lead cases retain original endpoints and durations. Ego model, controller, starting poses, and seeds unchanged.',
            retention_navigation='No motion-ranked retained-plan history in traffic. Latest native prediction retains its original pose/time anchor between bounded planning boundaries.',
            traffic_manifest=str(manifest_path),traffic_protocol=manifest.get('traffic_protocol'))
        if traffic_controller == 'native_speed_no_launch_boost':
            protocol.update(
                native_speed_diagnostics='Native request controller_preview_speed_mps retains the upstream1s diagnostic. The actual traffic preview lead and target are recorded in each control debug record and independently reconstructed from the unchanged native trajectory.',
                controller=f'Original route projection, steering gain1.4, declared target ceiling {speed_cap}m/s, speed filter/braking and red visual hold. Native speeds averaged over0.5seconds starting at the declared {preview_lead}s preview, anchored to original plan age. Traffic-only removal of the launch-speed boost.',
                changes_from_round19='Add one declared lead vehicle on each original route, with separately declared traffic missions. Preserve model weights, inputs, starts, seeds, routes and durations; use the explicitly declared native flow temperature and common traffic target-speed ceiling. Original route deadlines and driving acceptance limits remain unchanged. Disable the upstream launch-speed boost. Explicit traffic speed-preview lead is recorded in every case and independently replayed; no native trajectory or speed floor is introduced.',
                heading_wait='When set, pause at the heading bound using only ego pose. Keep original source anchors and the declared traffic controller.')
    with acquire(root.parent/'.carlamayo-resource-locks',model,renderer):
        initial_resources=available(model,renderer)
        if shutil.disk_usage(output.parent).free<5*1024**3 or shutil.disk_usage(root).free<256*1024**2:
            raise RuntimeError('Need at least5GiB free for bounded recordings')
        output.mkdir(parents=True)
        save(output/'protocol.json',protocol)
        status=dict(state='running',resources=initial_resources,results=[],unattempted=list(args.cases))
        save(output/'status.json',status)
        for number in args.cases:
            available(model,renderer)
            if hashlib.sha256(manifest_path.read_bytes()).hexdigest()!=protocol['manifest_sha256']:
                raise RuntimeError('Frozen scenario manifest changed')
            for name,digest in sources.items():
                if hashlib.sha256((root/name).read_bytes()).hexdigest()!=digest:
                    raise RuntimeError(f'Frozen source changed: {name}')
            name=manifest['scenarios'][number-1]['name']
            command=[sys.executable,'-u','-m','local.experiments.run_projection_suite',
                '--manifest',str(manifest_path),'--single-gpu-model','--async-inference',
                '--server-script','local/experiments/projected_server.sh','--steering-modes','route',
                '--speed-selection-attempts',str(args.speed_selection_attempts),'--speed-cap-mps',str(speed_cap),'--steering-gain','1.4',
                '--cache-age-s','0','--require-pass','--cases',str(number),'--output',str(output/name)]
            if args.capture_all_native_inputs:
                command.append('--capture-all-native-inputs')
            if args.max_pending_sim_s is not None:
                command.extend(['--async-max-pending-sim-s',str(args.max_pending_sim_s)])
            if args.max_pending_heading_deg is not None:
                command.extend(['--async-max-pending-heading-deg',str(args.max_pending_heading_deg)])
            if args.retention_age_s:
                command.extend(['--async-retention-age-s',str(args.retention_age_s)])
            if args.revalidate_delayed_plans:
                command.append('--async-revalidate-delayed-plans')
            if args.fresh_planning_interval_s is not None:
                command.extend(['--async-fresh-planning-interval-s',str(args.fresh_planning_interval_s)])
            if args.motion_commitment_s is not None:
                command.extend(['--async-motion-commitment-s',str(args.motion_commitment_s)])
            if args.rest_planning_interval_s is not None:
                command.extend(['--async-rest-planning-interval-s',str(args.rest_planning_interval_s)])
            if args.fine_planning_interval_s is not None:
                command.extend(['--async-fine-planning-interval-s',str(args.fine_planning_interval_s)])
            if args.fine_planning_signal_only:
                command.append('--async-fine-planning-signal-only')
            if args.deceleration_planning_interval_s is not None:
                command.extend(['--async-deceleration-planning-interval-s',str(args.deceleration_planning_interval_s)])
            status.update(current=name)
            status['unattempted'].remove(number)
            save(output/'status.json',status)
            case_started=time.monotonic()
            startup_attempts=[]
            for attempt in range(2):
                if attempt:
                    available(model,renderer)
                    for source,digest in sources.items():
                        if hashlib.sha256((root/source).read_bytes()).hexdigest()!=digest:
                            raise RuntimeError(f'Frozen source changed: {source}')
                result=dict(number=number,name=name,command=command)
                started=time.monotonic()
                with (output/(name+'.log')).open('w') as log:
                    worker=subprocess.Popen(command,cwd=root,env=environment,stdout=log,
                                            stderr=subprocess.STDOUT,start_new_session=True)
                    try:
                        result['returncode']=wait_for_case(worker,output/name/'status.json',result,
                            container=args.container,work_timeout_s=args.case_work_timeout_s)
                    except SimulatorStartupCrash:
                        stop_worker(worker)
                        result['returncode']=worker.returncode
                    except subprocess.TimeoutExpired:
                        result['timed_out']=True
                        stop_worker(worker)
                        result['returncode']=worker.returncode
                    finally:
                        stop_worker(worker)
                        result['stop_returncode']=subprocess.run(['bash','local/experiments/projected_server.sh','stop'],
                            cwd=root,env=environment,stdout=log,stderr=subprocess.STDOUT,timeout=40).returncode
                        result['server_after_stop']=container_state(args.container)
                        with (output/(name+'-server.log')).open('w') as server_log:
                            subprocess.run(['docker','logs',args.container],stdout=server_log,stderr=subprocess.STDOUT)
                result['end_to_end_seconds']=time.monotonic()-started
                suite_path=output/name/'status.json'
                if suite_path.exists():
                    suite=json.loads(suite_path.read_text())
                    result['suite']=suite
                    result['passed']=suite.get('passed',False)
                if not result.get('passed') and 'startup_failure' not in result:
                    failure=confirmed_startup_crash(suite_path,args.container)
                    if failure is not None:
                        result['startup_failure']=failure
                result['attempt_end_to_end_seconds']=result['end_to_end_seconds']
                if retry_startup_failure(result,output/name,attempt):
                    archived=archive_startup_failure(output,name,result)
                    startup_attempts.append(archived)
                    status.setdefault('startup_failures',[]).append(archived)
                    save(output/'status.json',status)
                    print(json.dumps(dict(startup_retry=name,failed_attempt=archived)),flush=True)
                    continue
                result['startup_retry_count']=len(startup_attempts)
                result['startup_attempts']=startup_attempts
                result['end_to_end_seconds']=time.monotonic()-case_started
                result['startup_retry_extra_seconds']=result['end_to_end_seconds']-result['attempt_end_to_end_seconds'] if startup_attempts else 0.
                break
            status['results'].append(result)
            save(output/'status.json',status)
            print(json.dumps({k:v for k,v in result.items() if k not in ('command','suite')}),flush=True)
            # Keep independent driving failures, but stop after any incomplete
            # capture, timeout, audit failure, or unverified server cleanup.
            captures=result.get('suite',{}).get('results',[])
            infrastructure_ok=(not result.get('timed_out') and captures and
                all(c.get('returncode')==0 and c.get('audit_returncode')==0 for c in captures)
                and not result['server_after_stop']['Running'])
            if traffic_suite:
                infrastructure_ok = completed_traffic_attempt(result)
            if not infrastructure_ok:
                status['state']='stopped_after_execution_failure'
                break
        else:
            status['state']='completed'
        status['current']=None
        save(output/'status.json',status)
        if status['state']!='completed' or not all(r.get('passed') for r in status['results']):
            raise SystemExit(1)


if __name__=='__main__':
    main()
