"""Bounded native speed selection for the isolated empty, green junction test.

No speed floor, altered trajectory, cached path or forced go-intent. A red or
yellow visual signal keeps the first model result, including its stop intent.
After the finite budget, use the closest valid native candidate and record the
missed preference explicitly. When a speed ceiling is explicitly configured,
score the capped target so fast predictions need no unnecessary retries. This
is not an obstacle-aware safety policy.
"""
import json
import math

import numpy as np

from local.experiments import inference_probe as native
from local.experiments.projected_probe import SingleSampleProbe


def speed_score(record, low=3., high=5., speed_cap_mps=None):
    path = np.asarray(record['trajectory_ego'])
    speed = record['controller_preview_speed_mps']
    if (path.shape != (64, 3) or not np.isfinite(path).all() or
            not math.isfinite(speed) or not record['trajectory_marker_present'] or
            record['reverse_velocity_steps']):
        return math.inf, math.inf
    effective = min(speed, speed_cap_mps) if speed_cap_mps is not None else speed
    return max(low - effective, effective - high, 0.), abs(effective - (low + high) / 2)


class SpeedSelectedProbe(SingleSampleProbe):
    def sample_plan(self, adapter, data, context, instruction, options, label,
                    capture=False, capture_failures=False):
        attempts = options['speed_selection_attempts']
        low, high = options.get('speed_preference_mps', [3., 5.])
        if not isinstance(attempts, int) or not 2 <= attempts <= 8 or not 0 < low < high:
            raise ValueError('Expected 2–8 attempts and a positive increasing speed band')
        # This selector is explicitly scoped to the configured controlled scene.
        if not context.get('signal_states') or set(context['signal_states'].values()) != {'Green'}:
            raise ValueError('Speed-selection experiment requires the configured all-green scene')
        proposals, best = [], None
        for index in range(attempts):
            candidate_label = f'proposal_{context["index"]:03d}_{index:02d}'
            candidate_options = {**options, 'seed': options['seed'] + index * 1009,
                                 'flow_seed': options['flow_seed'] + index * 1013}
            result = super().sample_plan(adapter, data, context, instruction,
                candidate_options, candidate_label, capture and index == 0, capture_failures)
            record = json.loads((self.output / (candidate_label + '.json')).read_text())
            score = speed_score(record, low, high, options.get('controller_speed_cap_mps'))
            proposals.append(dict(label=candidate_label, seed=candidate_options['seed'],
                flow_seed=candidate_options['flow_seed'], target_mps=record['controller_preview_speed_mps'],
                valid=math.isfinite(score[0]), preference_distance_mps=score[0] if math.isfinite(score[0]) else None,
                inference_seconds=record['inference_seconds']))
            visual_stop = record['visual_signal'] in ('red', 'yellow')
            if visual_stop or best is None or score < best[0]:
                best = score, result, record
            if visual_stop or score[0] == 0:
                break
        score, result, record = best
        if not math.isfinite(score[0]):
            raise RuntimeError('No finite forward native trajectory within speed-selection budget')
        selected = record['label']
        record.update(label=label, inference_seconds=sum(p['inference_seconds'] for p in proposals),
            selection=dict(kind='bounded_native_speed_preference', selected_proposal=selected,
                proposals=proposals, attempts=len(proposals), budget=attempts,
                preference_mps=[low, high], preference_met=score[0] == 0,
                visual_stop_preserved=visual_stop))
        native.write_json(self.output / (label + '.json'), record)
        print(json.dumps({'selection':label,'selected':selected,'attempts':len(proposals),
                          'target_mps':record['controller_preview_speed_mps'],
                          'preference_met':score[0] == 0,'visual_stop_preserved':visual_stop}), flush=True)
        return result
