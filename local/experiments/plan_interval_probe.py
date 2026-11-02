"""Revalidate a native plan between bounded replanning times; never edit its path.

Controlled empty-scene experiment. Every control tick still checks the original
path, age, geometry, signal freshness and controller forecast. Any failed check
immediately returns to fresh sampling. The default interval remains 0.1 s.
"""
import copy
import json
import math

import numpy as np

from local.experiments import candidate_reserve_probe as reserve
from local.experiments import retained_probe as retained
from local.experiments import tracking_probe as tracking
from local.experiments import inference_probe as native
from local.experiments.audit_turn import project
from local.experiments.validated_selection import choose

ACTIVE = None


def replanning_interval(options):
    interval = options.get('validation_replan_interval_s', .1)
    if (not math.isfinite(interval) or not .1 <= interval <= .5 or
            abs(round(interval / .1) * .1 - interval) > 1e-8):
        raise ValueError('Replanning interval must be on the 0.1 s grid in [0.1, 0.5]')
    return interval


def scheduled_evidence(source, context, route, signal, phase, signal_index, options):
    """Eligibility only; native/physical revalidation is additionally mandatory."""
    interval = replanning_interval(options)
    age = context['time_s'] - source['context']['time_s']
    if (interval <= .1 + 1e-8 or not 0 < age < interval - 1e-8 or
            age > options['validation_cache_max_age_s'] + 1e-8 or
            not options.get('verify_signal') or signal != 'green' or
            source.get('visual_signal') != 'green' or source['selection']['signal'] != 'green' or
            source['selection']['route'] != options['validation_route'] or
            source['navigation_phase'] != phase or
            not 0 <= context['index'] - signal_index < options.get('signal_check_interval', 5)):
        return None
    pose = context['world_pose']
    if not np.isfinite([pose[k] for k in ('x', 'y', 'yaw')]).all():
        return None
    if options['validation_route'] == 'left':
        turn = (route['entry']['yaw'] - pose['yaw'] + 180) % 360 - 180
        if phase == 'approach' and turn >= 60:
            return None  # Let the normal planner handle the instruction transition.
    distance, arc, heading, width, exit_arc = project(np.array([[pose['x'], pose['y']]]), route)
    end = route['exit'];yaw = math.radians(end['yaw'])
    beyond = (pose['x'] - end['x']) * math.cos(yaw) + (pose['y'] - end['y']) * math.sin(yaw) >= 0
    aligned = abs((pose['yaw'] - heading[0] + 180) % 360 - 180) <= 20
    if beyond and arc[0] >= exit_arc and distance[0] <= width[0] and aligned:
        return None  # Check completion at this tick rather than overshooting the endpoint.
    return {'policy': 'revalidated_native_execution_between_replans',
            'interval_s': interval, 'original_source_time_s': source['context']['time_s'],
            'age_s': age, 'source_phase': source['navigation_phase'], 'current_phase': phase,
            'visual_signal': signal, 'visual_signal_check_index': signal_index,
            'current_index': context['index'], 'fresh_inference_performed': False}


def preference_met(metric, options):
    target = metric['target_speed_mps']
    low = options.get('validation_resample_below_target_mps')
    high = options.get('validation_resample_above_target_mps')
    return bool((low is None or target >= low) and (high is None or target <= high) and
                metric.get('preferred_reserve_valid', True))


class PlanIntervalProbe(reserve.CandidateReserveProbe):
    def __init__(self, model, output):
        global ACTIVE
        super().__init__(model, output)
        ACTIVE = self

    def evaluate(self, adapter, data, context, instruction, options, label,
                 capture=False, capture_failures=False):
        plan = self.executing_plan
        if plan is not None:
            tracking.check_options(options)
            evidence = scheduled_evidence(plan.record, context, self.validation_route,
                self.visual_signal, self.navigation_phase, self.visual_signal_index, options)
            if evidence is not None:
                metric = dict(retained.assess_retained(plan, context, self.validation_route,
                    self.visual_signal, self.navigation_phase, options), candidate=0)
                if metric['valid']:
                    if 'tracking_forecast' not in metric:
                        raise RuntimeError('Scheduled execution requires controller tracking validation')
                    if preference_met(metric, options):
                        assert choose([metric], [plan.record['trajectory_ego']], self.visual_signal,
                            context['measured_speed_mps'], options.get('validation_desired_speed_mps', 5.)) == 0
                        record = copy.deepcopy(plan.record)
                        record.update(label=label, context=context, navigation_phase=self.navigation_phase,
                            visual_signal=self.visual_signal, visual_signal_answer=self.visual_signal_answer,
                            visual_signal_check_index=self.visual_signal_index,
                            inference_seconds=0., inference_performed=False,
                            raw_target_speed_mps=metric['target_speed_mps'],
                            controller_preview_speed_mps=metric['target_speed_mps'],
                            selection={'mode': 'retained_native_sample', 'chosen': plan.record['selection']['chosen'],
                                'proposal': plan.record['selection']['proposal'],
                                'route': options['validation_route'], 'signal': self.visual_signal},
                            retention={'reason': 'scheduled_native_execution', 'source_label': plan.record['label'],
                                'source_context': plan.record['context'], 'age_s': evidence['age_s'],
                                'chosen_cache_index': 0, 'metrics': [metric], 'schedule': evidence})
                        native.write_json(self.output / (label + '.json'), record)
                        if capture:
                            inputs = self.output / 'inputs'
                            inputs.mkdir(exist_ok=True)
                            native.torch.save({'data': {k: v.detach().cpu().clone() for k, v in data.items()},
                                'context': context, 'instruction': instruction, 'inference_performed': False},
                                inputs / (label + '.pt'))
                        self.executing_age = evidence['age_s']
                        print(json.dumps({'scheduled_native_execution': label,
                            'source': plan.record['label'], 'age_s': self.executing_age,
                            'target_speed_mps': metric['target_speed_mps']}), flush=True)
                        return plan.prediction, copy.deepcopy(plan.extra)
        else:
            # Validate the option before the first expensive inference as well.
            replanning_interval(options)
        return super().evaluate(adapter, data, context, instruction, options, label,
                               capture=capture, capture_failures=capture_failures)


def main():
    global ACTIVE
    original = reserve.CandidateReserveProbe
    previous_active, ACTIVE = ACTIVE, None
    reserve.CandidateReserveProbe = PlanIntervalProbe
    try:
        reserve.main()
    finally:
        reserve.CandidateReserveProbe = original
        active, ACTIVE = ACTIVE, previous_active
        if isinstance(active, PlanIntervalProbe):
            path = active.output / '000_verifai/summary.json'
            if path.exists():
                summary = json.loads(path.read_text())
                frames = [json.loads(p.read_text()) for p in active.output.glob('frame_*.json')]
                scheduled = sum(r.get('inference_performed') is False for r in frames)
                summary.update(planning_steps=len(frames), replayed_planning_steps=0,
                    scheduled_native_execution_steps=scheduled,
                    fresh_completed_planning_steps=len(frames) - scheduled,
                    inference_calls=len(frames) - scheduled,
                    model_pool_calls=len(list((active.output / 'proposals').glob('frame_*_pool_*.json'))),
                    kind='Full fresh rollout with native path revalidation between bounded replanning times')
                native.write_json(path, summary)
                native.write_json(active.output / 'batch-summary.json', [summary])


if __name__ == '__main__':
    main()
