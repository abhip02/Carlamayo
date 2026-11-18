"""Use CarlaMayo's async worker with the fixed round-19 scene/controller harness."""
import copy
import hashlib
import json
import math
from pathlib import Path
import queue
import threading
import time
from types import SimpleNamespace

import numpy as np
import torch

from module.async_inference import run_inference_worker
from module.trajectory_follower import trajectory_to_world, planned_speeds
from local.experiments import inference_probe as native
from local.experiments import projected_probe, projected_retention
from local.experiments.projected_probe import route_state
from local.experiments.projected_speed_selection import SpeedSelectedProbe, speed_score
from local.experiments.validated_probe import RouteExitReached
from local.experiments.validated_selection import pose_transform

ACTIVE = None
BASE_PROBE = native.Probe
REST_MOTION_QUESTION = (
    'The ego vehicle is currently nearly stationary. From the camera history, '
    'classify the nearest vehicle directly ahead in the ego lane. '
    'Answer only MOVING if it is visibly moving away, STOPPED if it is stationary, '
    'CLEAR if there is no lead vehicle in the lane, or UNKNOWN if uncertain. '
    'Do not infer motion from a green traffic light.')


def rest_plan_candidate(record, interval):
    """Only a native wait-then-forward plan may receive a longer interval."""
    if (record['context']['measured_speed_mps'] > .2
            or record.get('visual_signal') in ('red', 'yellow')
            or record.get('reverse_velocity_steps', 0)):
        return False
    points = np.asarray(record['trajectory_ego'])
    if points.shape != (64, 3) or not np.isfinite(points).all() or points[-1, 0] < 3.:
        return False
    speeds = planned_speeds(points)
    cfg = native.runner.defaults
    lead_s = record.get('options', {}).get('traffic', {}).get('speed_preview_lead_s', cfg.PID_SPEED_LEAD_SEC)
    lead, count = round(lead_s / .1), round(cfg.PID_SPEED_PREVIEW_SEC / .1)
    later = lead + round(interval / .1) - 1
    return bool(float(speeds[lead:lead + count].mean()) <= cfg.PID_STOP_SPEED_MPS
                and float(speeds[later:later + count].mean()) > cfg.PID_STOP_SPEED_MPS)


def parse_rest_motion(answer):
    value = answer.strip().lower().rstrip('.').strip()
    return value if value in ('moving', 'stopped', 'clear', 'unknown') else 'unknown'


def native_motion_interval(record, base_interval, commitment):
    """Briefly execute motion that begins before the ordinary refresh boundary.

    This never waits for motion beyond that boundary, changes a native speed,
    or resets the plan's timestamp. Fine, heading and navigation updates win.
    """
    if (commitment is None or not 0. <= record['context']['measured_speed_mps'] <= .2
            or record.get('visual_signal') in ('red', 'yellow')
            or record.get('reverse_velocity_steps', 0)):
        return base_interval
    points = np.asarray(record['trajectory_ego'])
    if points.shape != (64, 3) or not np.isfinite(points).all() or points[-1, 0] < 3.:
        return base_interval
    speeds = planned_speeds(points)
    cfg = native.runner.defaults
    lead = round(record.get('options', {}).get('traffic', {}).get(
        'speed_preview_lead_s', cfg.PID_SPEED_LEAD_SEC) / .1)
    count = round(cfg.PID_SPEED_PREVIEW_SEC / .1)
    targets = [float(speeds[lead + tick:lead + tick + count].mean())
               for tick in range(round(base_interval / .1))]
    if not targets or targets[0] > cfg.PID_STOP_SPEED_MPS:
        return base_interval
    first = next((tick for tick, target in enumerate(targets)
                  if target > cfg.PID_STOP_SPEED_MPS), None)
    if first is None:
        return base_interval
    return min(3., max(base_interval, round(first * .1 + commitment, 10)))


class DelayedPlanUnavailable(RuntimeError):
    """An older native plan cannot be executed at the current simulation state."""


def execution_plan(record, prediction, extra):
    return dict(record=record, result=(prediction, extra),
        world=trajectory_to_world(pose_transform(record['context']['world_pose']),record['trajectory_ego']),
        speeds=planned_speeds(record['trajectory_ego']))


def choose_execution_plan(current, history, context, route, options):
    """Recheck original moving-plan admission at the actual execution pose/time."""
    selected=current['execution_plan']
    maximum=options.get('async_retention_age_s',0.)
    latest=current['record']
    if not maximum or options.get('scene_policy')=='red_light_stop' or latest['visual_signal'] in ('red','yellow'):
        return selected,None
    if options.get('scene_policy')!='empty_green':
        raise ValueError('Motion-ranked async retention is limited to the original empty-green scenes')
    checks_options={**options,'projection_cache_age_s':maximum}
    assess=lambda plan:projected_retention.assess(plan,context,route,checks_options)
    current_check=assess(selected)
    low,high=options.get('speed_preference_mps',[3.,5.])
    midpoint=(low+high)/2
    def score(check):
        return ((check['preference_distance_mps'],abs(check['target_speed_mps']-midpoint))
                if check.get('valid') else (math.inf,math.inf))
    current_score=score(current_check)
    if abs(context['time_s']-selected['record']['context']['time_s'])<1e-8:
        current_score=speed_score(selected['record'],low,high,options.get('controller_speed_cap_mps'))
    choices=[]
    for plan in history:
        check=assess(plan)
        if check['valid']:
            choices.append(((check['preference_distance_mps'],check['age_s']),plan,check))
    if choices:
        _,alternative,check=min(choices,key=lambda item:item[0])
        if score(check)<current_score:
            selected,current_check=alternative,check
    is_retained=('retention' in latest or selected['record']['label']!=current['execution_plan']['record']['label'])
    delayed_admission=(options.get('async_revalidate_delayed_plans',False) and
                       context['time_s']-selected['record']['context']['time_s']>1e-8)
    if is_retained or delayed_admission:
        if not current_check.get('valid'):
            raise DelayedPlanUnavailable('Delayed async plan is no longer valid at the actual execution pose/time')
        admission=dict(latest_request_label=latest['label'],checks=current_check)
        if not is_retained:
            admission['kind']='delayed_native'
        return selected,admission
    return selected,None


class AsyncSingleSampleProbe(BASE_PROBE):
    def __init__(self, model, output):
        global ACTIVE
        super().__init__(model, output)
        ACTIVE = self
        projected_retention.ACTIVE = self
        self.requests, self.results = queue.Queue(maxsize=1), queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.worker = None
        self.pending = False
        self.pending_frame = None
        self.pending_yaw = None
        self.heading_wait_count = 0
        self.fresh_planning_wait_count = 0
        self.fine_planning_wait_count = 0
        self.previous_motion = None
        self.last_decision_frame = None
        self.last_decision_yaw = None
        self.wait_count = 0
        self.wait_seconds = 0.
        self.plan_refresh_wait_count = 0
        self.current = None
        self.request_count = self.discarded = 0
        self.last_submit_wall = float('-inf')
        self.phase = 'approach'
        self.events = (output / 'async-events.jsonl').open('w', buffering=1)
        self.execution_record = self.execution_world = None
        self.execution_age = 0.
        self.history = []

    def event(self, kind, **fields):
        self.events.write(json.dumps(dict(event=kind, wall_monotonic=time.monotonic(), **fields)) + '\n')

    def infer(self, request):
        started = time.monotonic()
        pred, extra = self.infer_plan(request)
        # Control ticks must not synchronize with the inference thread's CUDA
        # stream when extracting an already-completed prediction.
        pred = pred.detach().cpu()
        record = json.loads((self.output / (request['label'] + '.json')).read_text())
        interval = request['options'].get('async_rest_planning_interval_s')
        if interval is not None:
            record['rest_interval_eligible'] = False
            if rest_plan_candidate(record, interval):
                check_started = time.monotonic()
                capture_path = self.output / 'inputs' / ('rest_motion_' + request['label'] + '.pt')
                capture_path.parent.mkdir(exist_ok=True)
                torch.save({'data': {k: v.detach().cpu().clone() for k, v in request['data'].items()},
                            'context': request['context']}, capture_path)
                # VQA uses its own token protocol. Preserve the native plan's
                # trace, options and RNG; this query never changes that plan.
                old_trace, old_options = self.trace, self.options
                self.trace, self.options = {}, {}
                try:
                    with torch.random.fork_rng(devices=list(range(torch.cuda.device_count()))):
                        torch.manual_seed(42)
                        torch.cuda.manual_seed_all(42)
                        answer = request['adapter'].run_vqa(
                            self.model, request['adapter'].probe_processor,
                            request['data'], REST_MOTION_QUESTION)
                    answer_text = request['adapter'].extract_answer_text(answer).strip()
                finally:
                    self.trace, self.options = old_trace, old_options
                seconds = time.monotonic() - check_started
                status = parse_rest_motion(answer_text)
                record['rest_motion_check'] = dict(
                    context_index=request['context']['index'], question=REST_MOTION_QUESTION,
                    answer=answer_text, status=status, seconds=seconds,
                    input_file=str(capture_path.relative_to(self.output)),
                    input_sha256=hashlib.sha256(capture_path.read_bytes()).hexdigest())
                record['rest_interval_eligible'] = status in ('moving', 'clear')
                record['native_prediction_inference_seconds'] = record['inference_seconds']
                record['inference_seconds'] += time.monotonic() - check_started
        record['async_request'] = dict(frame=request['frame'], phase=request['phase'],
            submitted_wall=request['submitted_wall'], completed_wall=time.monotonic(),
            processing_seconds=time.monotonic() - started)
        native.write_json(self.output / (request['label'] + '.json'), record)
        if request['options'].get('traffic'):
            # Keep the actual most recent native input for post-run replay of
            # a physical failure. This does not alter planning or control.
            self.last_native_input = dict(data=request['data'],
                context=copy.deepcopy(request['context']), label=request['label'],
                instruction=request.get('instruction'))
        original=record
        if 'selected_proposal' in record.get('selection',{}):
            original=json.loads((self.output/(record['selection']['selected_proposal']+'.json')).read_text())
        history=[]
        for plan in getattr(self,'worker_history',[]):
            cpu_prediction,extra_plan=plan['result']
            history.append({**plan,'result':(cpu_prediction.detach().cpu(),extra_plan)})
        if hasattr(self,'worker_history'):
            self.worker_history=history
        return dict(frame_submitted=request['frame'], prediction=pred, extra=extra,
                    record=record, phase=request['phase'], respawn_revision=0,
                    execution_plan=execution_plan(original,pred,extra),history=history)

    def infer_plan(self, request):
        return BASE_PROBE.evaluate(self, request['adapter'], request['data'],
            request['context'], request['instruction'], request['options'], request['label'],
            capture=request['capture'], capture_failures=True)

    def make_request(self, adapter, data, context, instruction, options, capture):
        label = f'async_request_{self.request_count:04d}'
        self.request_count += 1
        self.last_submit_wall = time.monotonic()
        self.pending_yaw = context['world_pose']['yaw']
        self.last_decision_frame = context['index']
        self.last_decision_yaw = context['world_pose']['yaw']
        self.event('submitted', label=label, frame=context['index'], time_s=context['time_s'], phase=self.phase)
        return dict(frame=context['index'], adapter=adapter, data=data,
                    context=copy.deepcopy(context), instruction=instruction,
                    options={**options, 'instruction': instruction,'async_navigation_phase':self.phase}, label=label,
                    capture=capture, phase=self.phase, respawn_revision=0,
                    submitted_wall=self.last_submit_wall)

    def consume_result(self,result,context):
        self.pending = False
        self.pending_frame = None
        self.pending_yaw = None
        if 'error' in result:
            native.write_json(self.output / 'async-worker-error.json', result)
            raise RuntimeError(result['error'])
        if result['phase'] != self.phase:
            self.discarded += 1
            self.event('discarded_navigation_revision',source=result['record']['label'],frame=context['index'])
            return False
        self.current=result
        self.history=result.get('history',[])
        self.event('consumed',source=result['record']['label'],frame=context['index'],time_s=context['time_s'])
        return True

    def fresh_planning_boundary(self,adapter,data,context,instruction,options,capture):
        """Sparse planning with physics paused for current-state inference.

        This mode deliberately trades inference/control overlap for fresh
        decision inputs. It retains the async worker, native selection, and
        original plan anchors; it never repeats a request at the same frame.
        """
        base_interval=options['async_fresh_planning_interval_s']
        interval=native_motion_interval(self.current['record'], base_interval,
                                       options.get('async_motion_commitment_s'))
        rest_interval=options.get('async_rest_planning_interval_s')
        if (rest_interval is not None
                and self.current['record'].get('rest_interval_eligible') is True):
            # Eligibility is established from the native wait-then-forward
            # plan and the same input's visual motion check, never actor truth.
            interval=rest_interval
        fine_interval=options.get('async_fine_planning_interval_s')
        fine_reason=None
        if fine_interval is not None:
            speed=context['measured_speed_mps']
            deceleration=0.
            if self.previous_motion is not None:
                previous_time,previous_speed=self.previous_motion
                elapsed_motion=context['time_s']-previous_time
                if elapsed_motion>0:
                    deceleration=(previous_speed-speed)/elapsed_motion
            if speed>.05:
                if self.current['record'].get('visual_signal') in ('red','yellow'):
                    fine_reason='visual_stop_signal'
                elif not options.get('async_fine_planning_signal_only',False) and deceleration>=.5:
                    fine_reason='observed_deceleration'
            if fine_reason is not None:
                interval=fine_interval
            elif (speed>.05 and deceleration>=.5
                  and options.get('async_deceleration_planning_interval_s') is not None):
                interval=min(interval,options['async_deceleration_planning_interval_s'])
                fine_reason='separate_observed_deceleration'
        elapsed=(context['index']-self.last_decision_frame)*native.runner.DT
        heading=abs((context['world_pose']['yaw']-self.last_decision_yaw+180.)%360.-180.)
        heading_limit=options.get('async_max_pending_heading_deg')
        heading_due=heading_limit is not None and heading>=heading_limit-1e-8
        if elapsed<interval-1e-8 and not heading_due and self.current['phase']==self.phase:
            if (options.get('async_motion_commitment_s') is not None
                    and elapsed>=base_interval-1e-8 and interval>base_interval):
                self.event('native_motion_commitment', frame=context['index'],
                           source=self.current['record']['label'], time_s=context['time_s'],
                           original_interval_s=base_interval, effective_interval_s=interval)
            return
        if self.pending:
            raise RuntimeError('Fresh-boundary planning cannot carry an in-flight request across simulation ticks')
        request=self.make_request(adapter,data,context,instruction,options,capture)
        self.requests.put_nowait(request)
        self.pending=True
        self.pending_frame=context['index']
        started=time.monotonic()
        try:
            result=self.results.get(timeout=60.)
        except queue.Empty:
            raise RuntimeError('Current-state prediction did not finish within 60 seconds at a planning boundary')
        waited=time.monotonic()-started
        self.wait_count+=1
        self.wait_seconds+=waited
        self.fresh_planning_wait_count+=1
        self.fine_planning_wait_count+=int(fine_reason is not None)
        self.heading_wait_count+=int(heading_due)
        self.event('simulation_wait',frame=context['index'],pending_frame=self.pending_frame,
                   wall_seconds=waited,reason='fresh_planning_boundary',heading_change_deg=heading,
                   effective_interval_s=interval,fine_reason=fine_reason)
        if not self.consume_result(result,context):
            raise RuntimeError('Navigation changed while physics was paused at a planning boundary')

    def ensure_executable_plan(self,adapter,data,context,instruction,options,capture):
        # At most one pending result plus one fresh request from the current
        # frozen state. Never repeat a request at the same input frame or reset
        # the age of the old plan to make it admissible.
        for attempt in range(3):
            if 'execution_plan' not in self.current:
                self.current['execution_plan']=execution_plan(self.current['record'],self.current['prediction'],self.current['extra'])
            try:
                return choose_execution_plan(self.current,self.history,context,self.route,options)
            except DelayedPlanUnavailable:
                if attempt==2:
                    raise
                if not self.pending:
                    if self.current['record']['context']['index']==context['index']:
                        raise
                    request=self.make_request(adapter,data,context,instruction,options,capture)
                    self.requests.put_nowait(request)
                    self.pending=True
                    self.pending_frame=context['index']
                started=time.monotonic()
                frame=self.pending_frame
                try:
                    result=self.results.get(timeout=60.)
                except queue.Empty:
                    raise RuntimeError('Async prediction did not finish within60seconds while revalidating a delayed plan')
                waited=time.monotonic()-started
                self.wait_count+=1
                self.plan_refresh_wait_count+=1
                self.wait_seconds+=waited
                self.event('simulation_wait',frame=context['index'],pending_frame=frame,
                           wall_seconds=waited,reason='delayed_plan_unavailable')
                self.consume_result(result,context)
        raise AssertionError('Unreachable retained-plan refresh state')

    def evaluate(self, adapter, data, context, instruction, options, label,
                 capture=False, capture_failures=False):
        if not hasattr(self, 'route'):
            routes = json.loads(Path(options['validation_routes']).read_text())
            self.route = routes['routes'][options['validation_route']]
            native.write_json(self.output / 'route-reference.json', routes)
        reached, self.phase, instruction = route_state(context, self.route, options['validation_route'], self.phase)
        if reached:
            native.write_json(self.output / 'route-exit-reached.json', dict(context=context))
            raise RouteExitReached(self.output, context)

        if self.current is None:
            # Match round-19's moving handover exactly. The first plan is
            # computed without advancing physics; this cost stays in wall time.
            request = self.make_request(adapter, data, context, instruction, options, capture)
            self.current = self.infer(request)
            self.history=self.current.get('history',[])
            self.event('bootstrap_ready', frame=context['index'])
            self.worker = threading.Thread(target=run_inference_worker,
                args=(self.requests, self.results, self.stop, self.infer),
                name='alpamayo-fixed-scene-worker', daemon=True)
            self.worker.start()
        elif options.get('async_fresh_planning_interval_s') is not None:
            self.fresh_planning_boundary(adapter,data,context,instruction,options,capture)
        else:
            wait_limit = options.get('async_max_pending_sim_s')
            heading_limit = options.get('async_max_pending_heading_deg')
            heading_change = (abs((context['world_pose']['yaw'] - self.pending_yaw + 180.) % 360. - 180.)
                              if self.pending else 0.)
            heading_wait = (self.pending and heading_limit is not None and
                            heading_change >= heading_limit - 1e-8)
            must_wait = heading_wait or (self.pending and wait_limit is not None and
                         (context['index'] - self.pending_frame) * native.runner.DT >= wait_limit - 1e-8)
            wait_started = time.monotonic()
            try:
                result = self.results.get(timeout=60.) if must_wait else self.results.get_nowait()
            except queue.Empty:
                if must_wait:
                    raise RuntimeError('Async prediction did not finish within 60 seconds at the simulation advance or heading limit')
                result = None
            if must_wait:
                waited = time.monotonic() - wait_started
                self.wait_count += 1
                self.heading_wait_count += int(heading_wait)
                self.wait_seconds += waited
                self.event('simulation_wait', frame=context['index'],
                           pending_frame=self.pending_frame, wall_seconds=waited,
                           reason='heading_change' if heading_wait else 'pending_age',
                           heading_change_deg=heading_change)
            if result is not None:
                self.consume_result(result,context)
            if not self.pending and time.monotonic() - self.last_submit_wall >= 1.:
                request = self.make_request(adapter, data, context, instruction, options, capture)
                self.requests.put_nowait(request)
                self.pending = True
                self.pending_frame = context['index']

        plan,retention=self.ensure_executable_plan(adapter,data,context,instruction,options,capture)
        source=plan['record']
        age = (context['index'] - source['context']['index']) * native.runner.DT
        if age < -1e-8:
            raise RuntimeError('Async plan originates in the future')
        record = copy.deepcopy(source)
        for key in ('visual_signal','visual_signal_answer','visual_signal_check_index'):
            if key in self.current['record']:
                record[key]=self.current['record'][key]
        record.update(label=label, context=context, inference_seconds=0.,
            inference_performed=False, navigation_phase=self.phase,
            selection=dict(route=options['validation_route'], kind='async_native_plan'),
            async_execution=dict(source_label=source['label'], source_context=source['context'],
                                 age_s=age, pending_inference=self.pending,
                                 pending_frame=self.pending_frame,
                                 latest_request_label=self.current['record']['label']))
        if retention is not None:
            key='async_plan_validation' if retention.get('kind')=='delayed_native' else 'async_retention'
            record[key]=retention
        native.write_json(self.output / (label + '.json'), record)
        self.execution_record = record
        self.execution_age = age
        self.execution_world=plan['world']
        if options.get('async_fine_planning_interval_s') is not None:
            self.previous_motion=(context['time_s'],context['measured_speed_mps'])
        return plan['result'][0],copy.deepcopy(plan['result'][1])

    def close(self):
        # Both the native capture and the outer experiment wrapper close the
        # probe. Restore model hooks once and preserve any late worker failure.
        if getattr(self, '_closed', False):
            return
        self._closed = True
        self.stop.set()
        if self.worker is not None:
            self.worker.join(timeout=60.)
            if self.worker.is_alive():
                raise RuntimeError('Async inference worker did not stop within 60 seconds')
        pending_results = self.results.qsize()
        try:
            shutdown_result = self.results.get_nowait()
        except queue.Empty:
            shutdown_result = None
        shutdown_error = shutdown_result if shutdown_result and 'error' in shutdown_result else None
        if shutdown_error:
            native.write_json(self.output / 'async-worker-error.json', shutdown_error)
        completed = sorted(self.output.glob('async_request_*.json'))
        native.write_json(self.output / 'async-summary.json', dict(
            submitted=self.request_count, completed=len(completed),
            navigation_revision_discards=self.discarded,
            simulation_wait_count=self.wait_count, simulation_wait_wall_seconds=self.wait_seconds,
            plan_refresh_wait_count=self.plan_refresh_wait_count,
            heading_wait_count=self.heading_wait_count,
            fresh_planning_wait_count=self.fresh_planning_wait_count,
            fine_planning_wait_count=self.fine_planning_wait_count,
            results_pending_at_shutdown=pending_results, worker_stopped=True,
            shutdown_worker_error=shutdown_error is not None,
            note='All completed requests counted, including shutdown results not consumed by control.'))
        try:
            if hasattr(self, 'last_native_input'):
                started = time.monotonic()
                value = self.last_native_input
                path = self.output / 'inputs' / 'last_native_request.pt'
                path.parent.mkdir(exist_ok=True)
                torch.save({**value, 'data': {k: v.detach().cpu().clone()
                           for k, v in value['data'].items()}}, path)
                executed = (self.execution_record or {}).get('async_execution', {}).get('source_label')
                native.write_json(self.output / 'last-native-input.json', dict(
                    source_label=value['label'], source_context=value['context'],
                    input_file=str(path.relative_to(self.output)),
                    input_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    last_executed_source_label=executed,
                    used_by_last_control=executed == value['label'],
                    recording_seconds=time.monotonic() - started,
                    note='Final completed native request after worker drain; an unconsumed request is explicitly identified. Recording cost remains in attempt wall time.'))
        finally:
            self.events.close()
            super().close()
        if shutdown_error:
            raise RuntimeError(shutdown_error['error'])


class AsyncSpeedSelectedProbe(AsyncSingleSampleProbe, SpeedSelectedProbe):
    """Run the existing bounded native selection entirely in the model worker.

    The native selector preserves visual red/yellow stops and takes only one
    sample in the declared red-stop scene. Optional history uses the original
    admission rule in the worker and is rechecked at execution time.
    """
    def __init__(self,model,output):
        super().__init__(model,output)
        self.worker_history=[]

    def consider_retained(self,record,result,context,options):
        maximum=options.get('async_retention_age_s',0.)
        if not maximum:
            return None
        proxy=SimpleNamespace(history=self.worker_history,route=self.route)
        answer=projected_retention.RetainedSpeedProbe.consider_retained(proxy,record,result,context,
            {**options,'projection_cache_age_s':maximum})
        self.worker_history=proxy.history
        return answer

    def infer_plan(self, request):
        # Match the original round19 cache: a navigation instruction change
        # does not erase a native plan that still passes its age and actual-pose
        # admission checks. Wrong-phase pending results remain discarded by
        # consume_result; original anchors, visual-stop precedence and expiry
        # are unchanged.
        return SpeedSelectedProbe.sample_plan(self, request['adapter'], request['data'],
            request['context'], request['instruction'], request['options'], request['label'],
            capture=request['capture'], capture_failures=True)


def capture(args, output):
    global ACTIVE
    options = json.loads(args.options.read_text())
    config = native.yaml.safe_load(args.config.read_text())
    if not options.get('async_inference') or not config.get('async_inference'):
        raise ValueError('Async capture requires explicit flags in both config and options')
    if options.get('projection_cache_age_s', 0):
        raise ValueError('Async experiment does not support motion-ranked history')
    manifest_path = output / 'experiment.json'
    manifest = json.loads(manifest_path.read_text())
    for relative in ('module/async_inference.py', 'local/experiments/async_projected_probe.py'):
        content = (native.runner.ROOT / relative).read_bytes()
        snapshot = output / 'source_snapshot' / relative
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        snapshot.write_bytes(content)
        manifest['source_sha256'][relative] = hashlib.sha256(content).hexdigest()
    native.write_json(manifest_path, manifest)
    probe_class = (AsyncSpeedSelectedProbe if options.get('speed_selection_attempts', 1) > 1
                   else AsyncSingleSampleProbe)
    try:
        projected_probe.capture(args, output, probe_class_override=probe_class)
    finally:
        if ACTIVE is not None:
            ACTIVE.close()
            ACTIVE = None


if __name__ == '__main__':
    native.capture = capture
    native.main()
