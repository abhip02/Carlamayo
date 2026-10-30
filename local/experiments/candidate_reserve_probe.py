"""Opt-in rescue using previously admitted, unselected native candidates.

Only after fresh sampling AND the existing selected-plan cache fail. Every
original age, world anchor, speed array, signal, geometry and tracking check
still applies. Production inference/controller and normal selections unchanged.
"""
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from local.experiments import tracking_probe as tracking
from local.experiments import retained_probe as retained
from local.experiments import validated_probe as validated
from local.experiments import inference_probe as native
from local.experiments.validated_selection import choose
from module.trajectory_follower import planned_speeds


def reserved_plan(proposal, fresh, attempt, metric, adapter, tokenizer):
    """Archive an original admitted candidate with its own exact text and tokens."""
    if not metric['valid']:
        raise ValueError('Only originally admitted native candidates may be reserved')
    chosen = metric['candidate']
    if (proposal['label'] != attempt['proposal'] or proposal['context'] != fresh['context'] or
            attempt['metrics'][chosen] != metric):
        raise ValueError('Original proposal and admission provenance do not match')
    raw = np.asarray(proposal['candidate_trajectories'][chosen], dtype=np.float32)
    if raw.tolist() != proposal['candidate_trajectories'][chosen]:
        raise ValueError('Native archive cannot be reconstructed exactly as float32')
    extra = {}
    for key, value in proposal['candidate_texts'].items():
        if isinstance(value, list):
            array = np.asarray(value)
            if not array.ndim or array.shape[-1] != len(proposal['candidate_trajectories']):
                raise ValueError('Unexpected native candidate text shape')
            value = array[..., chosen:chosen + 1].copy()
        extra[key] = copy.deepcopy(value)
    tokens = proposal['candidate_generated_tokens'][chosen]
    velocities = proposal['candidate_velocities'][chosen]
    speeds = planned_speeds(raw)
    start = round(native.runner.defaults.PID_SPEED_LEAD_SEC / .1)
    end = round((native.runner.defaults.PID_SPEED_LEAD_SEC + native.runner.defaults.PID_SPEED_PREVIEW_SEC) / .1)
    record = copy.deepcopy(proposal)
    record.update(
        label=f"reserved_{attempt['proposal']}_candidate_{chosen}",
        navigation_phase=fresh['navigation_phase'], trajectory_ego=raw.tolist(),
        reasoning=adapter.extract_cot_text(extra), generated_token_ids=tokens,
        generated_tokens=len(tokens), raw_generated_text=tokenizer.decode(tokens, skip_special_tokens=False),
        trajectory_marker_present=proposal['candidate_markers'][chosen],
        trajectory_marker_generated=tokenizer.convert_tokens_to_ids('<|traj_future_start|>') in tokens,
        signed_velocity_mps=velocities, acceleration_mps2=proposal['candidate_accelerations'][chosen],
        raw_target_speed_mps=float(speeds[10:15].mean()),
        controller_preview_speed_mps=float(speeds[start:end].mean()),
        first_second_speed_mps=float(speeds[:10].mean()),
        end_forward_m=float(raw[-1, 0]), end_left_m=float(raw[-1, 1]),
        reverse_velocity_steps=sum(v < -.05 for v in velocities),
        selection={'mode': 'reserved_native_sample', 'chosen': chosen,
                   'proposal': attempt['proposal'], 'route': fresh['selection']['route'],
                   'signal': proposal['visual_signal']},
        admitted_reserve={'fresh_record': fresh['label'], 'original_metric': copy.deepcopy(metric),
                          'note': 'Originally admitted but unselected; must pass all checks again before use.'})
    return retained.ModelPlan.from_record(record, torch.from_numpy(raw)[None, None, None], extra)


class CandidateReserveProbe(tracking.TrackingProbe):
    def __init__(self, model, output):
        super().__init__(model, output)
        self.candidate_reserve = []
        source = Path(__file__).read_bytes()
        snapshot = output / 'source_snapshot/local/experiments/candidate_reserve_probe.py'
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        snapshot.write_bytes(source)
        native.write_json(output / 'candidate-reserve-policy.json', {
            'mode': 'Only after fresh sampling and selected-plan retention fail',
            'source_sha256': hashlib.sha256(source).hexdigest(),
            'admission': 'Originally admitted candidates only; unchanged current-state checks and half-second age cap.'})

    def remember_alternatives(self, adapter, label):
        path = self.output / ('accepted_' + label + '.json')
        if not path.exists():
            return  # No fresh admitted pool when an earlier cached plan was used after exhaustion.
        fresh = json.loads(path.read_text())
        selection = fresh['selection']
        for attempt in selection['attempts']:
            proposal = json.loads((self.output / 'proposals' / (attempt['proposal'] + '.json')).read_text())
            for metric in attempt['metrics']:
                if not metric['valid'] or (attempt['proposal'] == selection['proposal'] and
                                           metric['candidate'] == selection['chosen']):
                    continue
                plan = reserved_plan(proposal, fresh, attempt, metric, adapter, self.model.tokenizer)
                native.write_json(self.output / (plan.record['label'] + '.json'), plan.record)
                self.candidate_reserve.append(plan)

    def evaluate(self, adapter, data, context, instruction, options, label,
                 capture=False, capture_failures=False):
        # The parent enforces the same maximum <= 0.5 s before any sampling.
        maximum_age = options.get('validation_cache_max_age_s', 0.)
        self.candidate_reserve = [p for p in self.candidate_reserve if
                                  context['time_s'] - p.record['context']['time_s'] <= maximum_age + 1e-6]
        try:
            result = super().evaluate(adapter, data, context, instruction, options, label,
                                      capture=capture, capture_failures=capture_failures)
        except validated.NoValidatedPlan:
            failure = json.loads((self.output / 'validation-failure.json').read_text())
            metrics = [dict(retained.assess_retained(p, context, self.validation_route,
                                                    self.visual_signal, self.navigation_phase, options), candidate=i)
                       for i, p in enumerate(self.candidate_reserve)]
            chosen = choose(metrics, [p.record['trajectory_ego'] for p in self.candidate_reserve],
                            self.visual_signal, context['measured_speed_mps'],
                            options.get('validation_desired_speed_mps', 5.))
            if chosen is None:
                native.write_json(self.output / 'candidate-reserve-failure.json', {'context': context, 'metrics': metrics})
                raise
            plan = self.candidate_reserve[chosen]
            age = context['time_s'] - plan.record['context']['time_s']
            record = copy.deepcopy(plan.record)
            record.update(
                label=label, context=context, navigation_phase=self.navigation_phase,
                visual_signal=self.visual_signal, visual_signal_answer=self.visual_signal_answer,
                visual_signal_check_index=self.visual_signal_index,
                inference_seconds=sum(a['inference_seconds'] for a in failure['attempts']),
                raw_target_speed_mps=metrics[chosen]['target_speed_mps'],
                controller_preview_speed_mps=metrics[chosen]['target_speed_mps'],
                selection={'mode': 'retained_native_sample', 'chosen': plan.record['selection']['chosen'],
                           'proposal': plan.record['selection']['proposal'], 'route': options['validation_route'],
                           'signal': self.visual_signal},
                retention={'reason': 'fresh_and_selected_cache_exhausted',
                           'source_label': plan.record['label'], 'source_context': plan.record['context'],
                           'age_s': age, 'chosen_cache_index': chosen, 'metrics': metrics,
                           'rejected_new_pools': failure['attempts']})
            rejected = self.output / 'replan-rejections'
            rejected.mkdir(exist_ok=True)
            # Keep both failures as evidence before removing terminal-only markers.
            (self.output / 'validation-failure.json').rename(rejected / (label + '.json'))
            (self.output / 'retention-failure.json').rename(rejected / (label + '-selected-cache.json'))
            native.write_json(self.output / (label + '.json'), record)
            self.executing_plan, self.executing_age = plan, age
            # It is now an actually selected plan; future parent checks still use its original time.
            self.history.append(plan)
            self.candidate_reserve.pop(chosen)
            print(json.dumps({'reserved_candidate': label, 'source': plan.record['label'], 'age_s': age,
                              'reason': 'fresh_and_selected_cache_exhausted',
                              'target_speed_mps': metrics[chosen]['target_speed_mps']}), flush=True)
            return plan.prediction, copy.deepcopy(plan.extra)
        self.remember_alternatives(adapter, label)
        return result


def main():
    original = tracking.TrackingProbe
    tracking.TrackingProbe = CandidateReserveProbe
    try:
        tracking.main()
    finally:
        tracking.TrackingProbe = original


if __name__ == '__main__':
    main()
