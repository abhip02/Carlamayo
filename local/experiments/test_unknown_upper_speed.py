"""Unknown-signal upper retry preserves native arrays, ranking and pool bounds."""
import copy
import json

import pytest
import torch

from local.experiments import inference_probe as native
from local.experiments.audit_quality import upper_speed_annotations_match
from local.experiments.test_validated_selection import fake_probe


@pytest.mark.parametrize('later_speed,expected_pool,met', [(4., 1, True), (7.5, 0, False), (None, 0, False)])
def test_unknown_retry_keeps_best_native_candidate(tmp_path, monkeypatch, later_speed, expected_pool, met):
    probe, adapter, context, options, prediction, calls = fake_probe(tmp_path, monkeypatch)
    probe.visual_signal = 'unknown'
    context['measured_speed_mps'] = 4.
    options.update(validation_resample_above_target_mps=5., validation_resample_fast_unknown=True,
                   validation_desired_speed_mps=4., validation_resample_language=True,
                   seed=45, flow_seed=44, validation_flow_temperatures=[1., 1.5])
    original = prediction[:, :, 1].clone()
    generate = native.Probe.evaluate
    generated = []

    def evaluate(*args, **kwargs):
        index = len(calls)
        speed = 7. if not index else later_speed
        prediction[:, :, 1] = original * ((speed or 1.) / 3.)
        if speed is None:
            prediction[:, :, :, :, 1] = 2.5
        result, extra = generate(*args, **kwargs)
        extra['cot'][0, 0, 1] = f'pool-{index}'
        generated.append(result[:, :, 1:2].clone())
        return result, extra

    monkeypatch.setattr(native.Probe, 'evaluate', evaluate)
    selected, extra = probe.evaluate(adapter, {}, context, 'Go straight', options, 'frame_000')
    assert len(calls) == 2
    assert [c['seed'] for c in calls] == [45, 46]
    assert [c['flow_seed'] for c in calls] == [44, 45]
    assert [c['flow_temperature'] for c in calls] == [1., 1.5]
    assert torch.equal(selected, generated[expected_pool])
    assert extra['cot'][0, 0, 0] == f'pool-{expected_pool}'
    frame = json.loads((tmp_path / 'frame_000.json').read_text())
    selection = frame['selection']
    metric = selection['attempts'][expected_pool]['metrics'][1]
    assert selection['speed_quality_preference']['applied']
    assert selection['speed_quality_preference']['met'] == met
    assert frame['trajectory_ego'] == frame['candidate_trajectories'][1]
    assert upper_speed_annotations_match(frame, metric)
    for field in ('met', 'applied'):
        altered = copy.deepcopy(frame)
        altered['selection']['speed_quality_preference'][field] ^= True
        assert not upper_speed_annotations_match(altered, metric)
    altered = copy.deepcopy(frame)
    altered['selection']['attempts'][0]['retry_for_speed_preference'] = False
    assert not upper_speed_annotations_match(altered, metric)
    altered = copy.deepcopy(frame)
    altered['options']['validation_resample_fast_unknown'] = False
    assert not upper_speed_annotations_match(altered, metric)


@pytest.mark.parametrize('signal,enabled,applied', [
    ('unknown', False, False), ('red', True, False), ('yellow', True, False), ('green', False, True)])
def test_signal_scope_and_default_remain_unchanged(tmp_path, monkeypatch, signal, enabled, applied):
    probe, adapter, context, options, prediction, calls = fake_probe(tmp_path, monkeypatch)
    probe.visual_signal = signal
    context['measured_speed_mps'] = 4.
    prediction[:, :, 1] *= 7. / 3.
    options.update(validation_resample_above_target_mps=5., validation_desired_speed_mps=4.)
    if enabled:
        options['validation_resample_fast_unknown'] = True
    selected, _ = probe.evaluate(adapter, {}, context, 'Go straight', options, 'frame_000')
    assert len(calls) == (2 if applied else 1)
    expected = 2 if signal in ('red', 'yellow') else 1
    assert torch.equal(selected, prediction[:, :, expected:expected + 1])
    frame = json.loads((tmp_path / 'frame_000.json').read_text())
    assert frame['selection']['speed_quality_preference']['applied'] == applied
    assert upper_speed_annotations_match(frame, frame['selection']['attempts'][0]['metrics'][expected])
