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
import time

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
    def consider_retained(self, record, result, context, options):
        return None

    def selected(self, record):
        pass

    def sample_plan(self, adapter, data, context, instruction, options, label,
                    capture=False, capture_failures=False):
        attempts = options['speed_selection_attempts']
        low, high = options.get('speed_preference_mps', [3., 5.])
        if (not isinstance(attempts, int) or not 2 <= attempts <= 8 or
                not all(math.isfinite(v) for v in (low, high)) or not 0 < low < high):
            raise ValueError('Expected 2–8 attempts and a positive increasing speed band')
        max_seconds = options.get('speed_selection_max_seconds')
        if max_seconds is not None and (not math.isfinite(max_seconds) or max_seconds <= 0):
            raise ValueError('Sampling time budget must be positive and finite')
        # A red-light test executes one native result, with no motion preference
        # or cached substitute. Simulator state scopes the test, not its control.
        scope = context.get('controlled_scene', {})
        if options.get('scene_policy') == 'red_light_stop':
            if not scope.get('verified') or scope.get('policy') != 'red_light_stop':
                raise ValueError('Red-light test requires verified scene scope')
            proposal = f'proposal_{context["index"]:03d}_00'
            result = super().sample_plan(adapter, data, context, instruction, options,
                                         proposal, capture, capture_failures)
            record = json.loads((self.output / (proposal + '.json')).read_text())
            record.update(label=label, selection=dict(kind='single_native_stop_scene',
                selected_proposal=proposal, attempts=1, budget=1))
            native.write_json(self.output / (label + '.json'), record)
            self.selected(record)
            return result
        signals = context.get('signal_states', {})
        checked_empty = (scope.get('verified') is True and scope.get('policy') == 'empty_green'
                         and scope.get('other_vehicle_count') == 0 and scope.get('pedestrian_count') == 0
                         and scope.get('unexpected_signal_count') == 0)
        if (signals and set(signals.values()) != {'Green'}) or (not signals and not checked_empty):
            raise ValueError('Speed-selection experiment requires the configured all-green scene')
        proposals, best = [], None
        started = time.monotonic()
        stop_reason = 'attempt_budget'
        for index in range(attempts):
            start_elapsed = time.monotonic() - started
            # Bound additional work. A native inference already in progress is
            # not preempted, so this is not a real-time deadline guarantee.
            if index and max_seconds is not None and start_elapsed >= max_seconds:
                stop_reason = 'time_budget'
                break
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
                start_elapsed_seconds=start_elapsed,
                inference_seconds=record['inference_seconds']))
            visual_stop = record['visual_signal'] in ('red', 'yellow')
            if visual_stop or best is None or score < best[0]:
                best = score, result, record
            retained = self.consider_retained(record, result, context, options)
            if not visual_stop and score[0] != 0 and retained is not None and retained[0] < best[0]:
                best = retained
            if visual_stop or best[0][0] == 0:
                stop_reason = 'visual_stop' if visual_stop else 'preference_met'
                break
        score, result, record = best
        if not math.isfinite(score[0]):
            raise RuntimeError('No finite forward native trajectory within speed-selection budget')
        selected = record['label']
        record.update(label=label, inference_seconds=sum(p['inference_seconds'] for p in proposals),
            selection=dict(kind='bounded_native_speed_preference', selected_proposal=selected,
                proposals=proposals, attempts=len(proposals), budget=attempts,
                preference_mps=[low, high], preference_met=score[0] == 0,
                preference_distance_mps=score[0],
                max_seconds=max_seconds, elapsed_seconds=time.monotonic() - started,
                stop_reason=stop_reason,
                visual_stop_preserved=visual_stop))
        native.write_json(self.output / (label + '.json'), record)
        self.selected(record)
        print(json.dumps({'selection':label,'selected':selected,'attempts':len(proposals),
                          'target_mps':record['controller_preview_speed_mps'],
                          'preference_met':score[0] == 0,'visual_stop_preserved':visual_stop}), flush=True)
        return result
