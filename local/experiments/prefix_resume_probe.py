"""Replay a verified native prefix, then resume fresh model sampling.

Experimental only. Prefix arrays and metadata are archived observations, never
new inference. Every prefix state and unchanged-controller command must match
before continuation. Rendered images are not promised bit-identical.
"""
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from local.experiments import candidate_reserve_probe as reserve
from local.experiments import inference_probe as native
from local.experiments import retained_probe as retained
from local.experiments.replay_native_tracking import native_plan, state_error
from local.experiments.tracking_forecast import read_capture

ACTIVE = None


class PrefixComplete(RuntimeError):
    def __init__(self, output):
        super().__init__('Requested native prefix verification complete; no fresh inference')
        self.output = output


def load_plan(capture, label):
    record, world, speeds = native_plan(capture, label)
    proposal = json.loads((capture / 'proposals' / (record['selection']['proposal'] + '.json')).read_text())
    chosen = record['selection']['chosen']
    extra = {key: np.asarray(value)[..., chosen:chosen + 1].copy() if isinstance(value, list)
             else copy.deepcopy(value) for key, value in proposal['candidate_texts'].items()}
    raw = np.asarray(record['trajectory_ego'], dtype=np.float32)
    if raw.tolist() != record['trajectory_ego']:
        raise ValueError('Prefix native array is not exactly reconstructible')
    plan = retained.ModelPlan.from_record(record, torch.from_numpy(raw)[None, None, None], extra)
    if not np.array_equal(plan.world, world) or not np.array_equal(plan.speeds, speeds):
        raise ValueError('Prefix plan reconstruction changed its source')
    return plan


class PrefixResumeProbe(reserve.CandidateReserveProbe):
    def __init__(self, model, output):
        global ACTIVE
        super().__init__(model, output)
        self.prefix_ready = False
        self.prefix_index = None
        self.prefix_verified = False
        self.prefix_errors, self.prefix_control_errors = [], []
        ACTIVE = self
        source = Path(__file__).read_bytes()
        path = output / 'source_snapshot/local/experiments/prefix_resume_probe.py'
        path.write_bytes(source)

    def prepare_prefix(self, options):
        source = Path(options['replay_prefix_capture'])
        seconds = options['replay_prefix_seconds']
        count = round(seconds / .1)
        if not np.isfinite(seconds) or count < 1 or abs(count * .1 - seconds) > 1e-8:
            raise ValueError('Replay prefix must end at a positive 0.1 s timestamp')
        self.reference, self.reference_controls = read_capture(source)
        if len(self.reference) != len(self.reference_controls) + 1 or count > len(self.reference_controls):
            raise ValueError('Source prefix is incomplete')
        source_options = json.loads((source / 'experiment.json').read_text())['options']
        if options['validation_route'] != source_options['validation_route']:
            raise ValueError('Prefix and continuation routes differ')
        for key in ('camera_projection', 'traffic_layout', 'traffic_light_state'):
            if options.get(key) != source_options.get(key):
                raise ValueError(f'Prefix scene setting differs: {key}')
        for name in ('module/config.py', 'module/trajectory_follower.py'):
            if (native.runner.ROOT / name).read_bytes() != (source / 'source_snapshot' / name).read_bytes():
                raise ValueError(f'Original controller or configuration differs: {name}')
        configs = [json.loads((p / 'config.json').read_text()) for p in (source, self.output)]
        for config in configs:
            config.pop('duration_s', None)
        if configs[0] != configs[1]:
            raise ValueError('Replay scene configurations differ')
        self.prefix_source, self.prefix_count = source, count
        self.prefix_sources = {}
        self.prefix_manifest = {}
        def archive(relative):
            content = (source / relative).read_bytes()
            target = self.output / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(content)
            self.prefix_manifest[str(relative)] = hashlib.sha256(content).hexdigest()
        archive(Path('route-reference.json'))
        self.validation_route = json.loads((source / 'route-reference.json').read_text())['routes'][options['validation_route']]
        for index, row in enumerate(self.reference_controls[:count]):
            label = f'frame_{index:03d}'
            if abs(row['time_s'] - index * .1) > 1e-8:
                raise ValueError('Prefix controller timestamps are not consecutive')
            if row['source_label'] not in self.prefix_sources:
                self.prefix_sources[row['source_label']] = load_plan(source, row['source_label'])
            plan = self.prefix_sources[row['source_label']]
            if (not np.array_equal(plan.world, row['world_trajectory']) or
                    not np.array_equal(plan.speeds, row['speeds_mps']) or
                    abs(row['age_s'] - (row['time_s'] - plan.record['context']['time_s'])) > 1e-8):
                raise ValueError(f'Prefix source/anchor/speeds/age mismatch at {index}')
            relatives = [Path(label + '.json'), Path(row['source_label'] + '.json')]
            relatives.extend(p.relative_to(source) for p in source.glob('accepted_' + label + '.json'))
            relatives.extend(p.relative_to(source) for p in (source / 'proposals').glob(label + '_pool_*.json'))
            relatives.extend(p.relative_to(source) for p in (source / 'replan-rejections').glob(label + '*.json'))
            for relative in relatives:
                if str(relative) not in self.prefix_manifest:
                    archive(relative)
        native.write_json(self.output / 'prefix-source-manifest.json', {
            'kind': 'Archived native prefix followed by fresh inference', 'source': str(source),
            'prefix_seconds': seconds, 'sha256': self.prefix_manifest,
            'note': 'Prefix records refer to original inference and original images. New rendered inputs may differ.'})
        self.prefix_ready = True

    def verify_state(self, context, index):
        actual = {**context['world_pose'], 'speed': context['measured_speed_mps']}
        if index != len(self.prefix_errors) or abs(context['time_s'] - index * .1) > 1e-8:
            raise RuntimeError('Prefix state sequence or timestamp mismatch')
        errors = state_error(actual, self.reference[index])
        self.prefix_errors.append(dict(index=index, **errors))
        if any(not np.isfinite(value) or value > 1e-6 for value in errors.values()):
            native.write_json(self.output / 'prefix-state-failure.json', {
                k: v if np.isfinite(v) else str(v) for k, v in self.prefix_errors[-1].items()})
            raise RuntimeError(f'Prefix state did not reproduce at {index}: {errors}')

    def verify_control(self, control):
        if self.prefix_index is None or self.prefix_index >= self.prefix_count:
            return
        expected = self.reference_controls[self.prefix_index]['control']
        error = max(abs(value - expected[key]) for key, value in zip(('steer', 'throttle', 'brake'), control[:3]))
        self.prefix_control_errors.append(error)
        if not np.isfinite(control[:3]).all() or error > 1e-8:
            native.write_json(self.output / 'prefix-control-failure.json', {'index': self.prefix_index, 'control': [float(v) if np.isfinite(v) else str(v) for v in control[:3]], 'expected': expected})
            raise RuntimeError(f'Prefix controller mismatch before application: {error}')

    def evaluate(self, adapter, data, context, instruction, options, label,
                 capture=False, capture_failures=False):
        if not self.prefix_ready:
            self.prepare_prefix(options)
        index = context['index']
        self.prefix_index = index
        if index <= self.prefix_count:
            self.verify_state(context, index)
        if index < self.prefix_count:
            row = self.reference_controls[index]
            plan = self.prefix_sources[row['source_label']]
            record = json.loads((self.prefix_source / (label + '.json')).read_text())
            record['prefix_replay'] = {'source_capture': str(self.prefix_source),
                                       'actual_context': context, 'fresh_inference': False}
            native.write_json(self.output / (label + '.json'), record)
            self.navigation_phase = record['navigation_phase']
            self.visual_signal = record['visual_signal']
            self.visual_signal_answer = record['visual_signal_answer']
            self.visual_signal_index = record['visual_signal_check_index']
            self.executing_plan, self.executing_age = plan, row['age_s']
            # Rebuild only the recent histories needed for continuation.
            if index >= self.prefix_count - 5:
                fresh_path = self.prefix_source / ('accepted_' + label + '.json')
                if fresh_path.exists():
                    self.history.append(load_plan(self.prefix_source, fresh_path.stem))
                    self.remember_alternatives(adapter, label)
                if (plan.record.get('admitted_reserve') and
                        all(p.record['label'] != plan.record['label'] for p in self.history)):
                    self.history.append(plan)
            return plan.prediction, copy.deepcopy(plan.extra)
        if not self.prefix_verified:
            if index != self.prefix_count or len(self.prefix_control_errors) != self.prefix_count:
                raise RuntimeError('Prefix ended without checking every control')
            self.prefix_verified = True
            native.write_json(self.output / 'prefix-verified.json', {
                'prefix_seconds': index * .1, 'states': self.prefix_errors,
                'control_errors': self.prefix_control_errors,
                'fresh_inference_begins_at_s': None if options.get('replay_prefix_only') else index * .1,
                'note': 'Physics/controller prefix verified; rendered images need not reproduce exactly.'})
            # A resumed decision must perform a new image-based signal check.
            self.visual_signal_index = -1000
            if options.get('replay_prefix_only'):
                raise PrefixComplete(self.output)
        return super().evaluate(adapter, data, context, instruction, options, label,
                                capture=capture, capture_failures=capture_failures)


def main():
    global ACTIVE
    runner = native.runner
    original_probe, original_follower = reserve.CandidateReserveProbe, runner.TrajectoryFollower
    class PrefixFollower(original_follower):
        def compute_control(self, *args, **kwargs):
            control = super().compute_control(*args, **kwargs)
            ACTIVE.verify_control(control)
            return control
    reserve.CandidateReserveProbe, runner.TrajectoryFollower = PrefixResumeProbe, PrefixFollower
    try:
        reserve.main()
    except PrefixComplete as done:
        path = done.output / '000_verifai/summary.json'
        summary = json.loads(path.read_text())
        summary.pop('error', None)
        summary.update(termination='prefix_verification_complete', fresh_inference_calls=0)
        native.write_json(path, summary)
        native.write_json(done.output / 'batch-summary.json', [summary])
    finally:
        reserve.CandidateReserveProbe, runner.TrajectoryFollower = original_probe, original_follower
        if ACTIVE is not None:
            path = ACTIVE.output / '000_verifai/summary.json'
            if path.exists():
                summary = json.loads(path.read_text())
                steps = summary['inference_calls']
                replayed = min(steps, getattr(ACTIVE, 'prefix_count', 0))
                summary.update(kind='Replayed native prefix followed by fresh inference',
                               planning_steps=steps, replayed_planning_steps=replayed,
                               inference_calls=steps - replayed,
                               fresh_completed_planning_steps=steps - replayed,
                               prefix_verified=ACTIVE.prefix_verified)
                native.write_json(path, summary)
                native.write_json(ACTIVE.output / 'batch-summary.json', [summary])
        ACTIVE = None


if __name__ == '__main__':
    main()
