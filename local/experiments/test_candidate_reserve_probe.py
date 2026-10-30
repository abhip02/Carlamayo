"""Alternative reserve must preserve native provenance and the original cache clock."""
import copy
import json
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from local.experiments import candidate_reserve_probe as reserve
from local.experiments.test_retained_probe import setup_plan


def fixture(tmp_path):
    route, current, options, original, _ = setup_plan()
    path = np.asarray(original.record['trajectory_ego'], dtype=np.float32)
    candidates = np.stack([path, path * np.float32(1.2), path.copy()])
    candidates[2, :, 1] = 2.5
    proposal = dict(label='frame_000_pool_0', context=copy.deepcopy(original.record['context']),
                    candidate_trajectories=candidates.tolist(), candidate_generated_tokens=[[10, 0], [10, 1], [10, 2]],
                    candidate_markers=[True] * 3, candidate_yaw_rad=[[0.] * 64] * 3,
                    candidate_velocities=[[3.] * 64, [3.6] * 64, [3.] * 64],
                    candidate_accelerations=[[0.] * 64] * 3,
                    candidate_texts={'cot': [[['first native intent', 'second native intent', 'bad native intent']]]},
                    visual_signal='green')
    attempt = {'proposal': proposal['label'], 'metrics': [
        {'candidate': i, 'valid': i != 2, 'reasons': [] if i != 2 else ['outside_lane']} for i in range(3)]}
    fresh = {'label': 'accepted_frame_000', 'context': proposal['context'], 'navigation_phase': 'approach',
             'selection': {'chosen': 0, 'proposal': proposal['label'], 'route': 'straight', 'attempts': [attempt]}}
    adapter = SimpleNamespace(extract_cot_text=lambda extra: extra['cot'][0, 0, 0])
    tokenizer = SimpleNamespace(decode=lambda ids, **kwargs: str(ids), convert_tokens_to_ids=lambda token: 10)
    probe = object.__new__(reserve.CandidateReserveProbe)
    probe.output, probe.history, probe.candidate_reserve = tmp_path, [], []
    probe.model = SimpleNamespace(tokenizer=tokenizer)
    probe.validation_route, probe.navigation_phase = route, 'approach'
    probe.visual_signal, probe.visual_signal_answer, probe.visual_signal_index = 'green', 'green', 4
    (tmp_path / 'proposals').mkdir()
    reserve.native.write_json(tmp_path / 'proposals/frame_000_pool_0.json', proposal)
    reserve.native.write_json(tmp_path / 'accepted_frame_000.json', fresh)
    return probe, adapter, current, options, proposal, fresh, attempt


def test_reserving_alternatives_preserves_exact_array_text_tokens_anchor_and_generation_admission(tmp_path):
    probe, adapter, _, _, proposal, fresh, attempt = fixture(tmp_path)
    probe.remember_alternatives(adapter, 'frame_000')
    assert len(probe.candidate_reserve) == 1  # Excludes selected candidate 0 and invalid candidate 2.
    plan = probe.candidate_reserve[0]
    assert plan.record['trajectory_ego'] == proposal['candidate_trajectories'][1]
    assert plan.record['generated_token_ids'] == [10, 1]
    assert plan.record['reasoning'] == 'second native intent'
    assert plan.extra['cot'][0, 0, 0] == 'second native intent'
    assert plan.record['context'] == proposal['context']
    assert plan.record['context']['time_s'] == 0.
    assert not plan.world.flags.writeable
    assert torch.equal(plan.prediction[0, 0, 0], torch.tensor(proposal['candidate_trajectories'][1]))
    proposal['candidate_trajectories'][1][0][0] = 999
    assert plan.record['trajectory_ego'][0][0] != 999
    with pytest.raises(ValueError, match='originally admitted'):
        reserve.reserved_plan(proposal, fresh, attempt, attempt['metrics'][2], adapter, probe.model.tokenizer)


@pytest.mark.parametrize('mismatch', ['proposal', 'context'])
def test_reserved_candidate_rejects_mismatched_original_provenance(tmp_path, mismatch):
    probe, adapter, _, _, proposal, fresh, attempt = fixture(tmp_path)
    if mismatch == 'proposal':
        proposal['label'] = 'different_pool'
    else:
        fresh = copy.deepcopy(fresh)
        fresh['context']['time_s'] = .1
    with pytest.raises(ValueError, match='provenance'):
        reserve.reserved_plan(proposal, fresh, attempt, attempt['metrics'][1], adapter, probe.model.tokenizer)


@pytest.mark.parametrize('failure_mode', ['none', 'expired', 'tracking', 'signal'])
def test_rescue_only_after_parent_exhaustion_preserves_native_clock_and_failure_evidence(tmp_path, monkeypatch, failure_mode):
    probe, adapter, context, options, *_ = fixture(tmp_path)
    probe.remember_alternatives(adapter, 'frame_000')
    plan = probe.candidate_reserve[0]
    if failure_mode == 'expired':
        context['time_s'] = .6
    if failure_mode == 'signal':
        probe.visual_signal = 'unknown'
    called = []
    original = reserve.retained.assess_retained

    def assess(*args, **kwargs):
        called.append(args)
        metric = original(*args, **kwargs)
        if failure_mode == 'tracking':
            metric['valid'] = False
            metric['reasons'].append('insufficient_controller_tracking_clearance')
        return metric

    def exhausted(*args, **kwargs):
        reserve.native.write_json(tmp_path / 'validation-failure.json', {'attempts': [{'inference_seconds': 1.2}]})
        reserve.native.write_json(tmp_path / 'retention-failure.json', {'metrics': [], 'selected_only_failed': True})
        raise reserve.validated.NoValidatedPlan('fresh and selected cache failed')

    monkeypatch.setattr(reserve.retained, 'assess_retained', assess)
    monkeypatch.setattr(reserve.tracking.TrackingProbe, 'evaluate', exhausted)
    if failure_mode != 'none':
        with pytest.raises(reserve.validated.NoValidatedPlan):
            probe.evaluate(adapter, {}, context, 'Go straight', options, 'frame_004')
        assert (tmp_path / 'validation-failure.json').exists()
        assert (tmp_path / 'retention-failure.json').exists()
        assert (tmp_path / 'candidate-reserve-failure.json').exists()
        assert not (tmp_path / 'frame_004.json').exists()
        return
    actual, extra = probe.evaluate(adapter, {}, context, 'Go straight', options, 'frame_004')
    assert len(called) == 1 and called[0][0] is plan and called[0][1] is context
    assert torch.equal(actual, plan.prediction)
    assert probe.executing_plan is plan and probe.executing_age == .4
    assert probe.history == [plan] and not probe.candidate_reserve
    extra['cot'][0, 0, 0] = 'modified returned copy'
    assert plan.extra['cot'][0, 0, 0] == 'second native intent'
    frame = json.loads((tmp_path / 'frame_004.json').read_text())
    assert frame['trajectory_ego'] == plan.record['trajectory_ego']
    assert frame['retention']['source_context']['time_s'] == 0.
    assert frame['retention']['age_s'] == .4 and frame['context']['time_s'] == .4
    assert (tmp_path / 'replan-rejections/frame_004.json').exists()
    assert (tmp_path / 'replan-rejections/frame_004-selected-cache.json').exists()
    assert not (tmp_path / 'validation-failure.json').exists()
    assert not (tmp_path / 'retention-failure.json').exists()
    from local.experiments.audit_retention import audit
    (tmp_path / 'controller-inputs.jsonl').write_text(json.dumps({
        'time_s': .4, 'source_label': plan.record['label'], 'source_context': plan.record['context'],
        'age_s': .4, 'world_trajectory': plan.world.tolist(), 'speeds_mps': plan.speeds.tolist()}) + '\n')
    assert audit(tmp_path)['all_sources_anchors_speeds_and_ages_match']


def test_parent_success_is_unchanged_and_unrelated_errors_are_not_hidden(tmp_path, monkeypatch):
    probe, adapter, context, options, *_ = fixture(tmp_path)
    expected = (torch.zeros((1, 1, 1, 64, 3)), {'cot': np.array([[['fresh']]])})
    monkeypatch.setattr(reserve.tracking.TrackingProbe, 'evaluate', lambda *a, **k: expected)
    assert probe.evaluate(adapter, {}, context, 'Go straight', options, 'frame_000') is expected
    assert len(probe.candidate_reserve) == 1

    def error(*args, **kwargs):
        raise RuntimeError('model failed')

    monkeypatch.setattr(reserve.tracking.TrackingProbe, 'evaluate', error)
    with pytest.raises(RuntimeError, match='model failed'):
        probe.evaluate(adapter, {}, context, 'Go straight', options, 'frame_004')
    assert not (tmp_path / 'candidate-reserve-failure.json').exists()


def test_entrypoint_restores_tracking_class_on_failure(monkeypatch):
    original = reserve.tracking.TrackingProbe

    def fail():
        assert reserve.tracking.TrackingProbe is reserve.CandidateReserveProbe
        raise RuntimeError('startup failed')

    monkeypatch.setattr(reserve.tracking, 'main', fail)
    with pytest.raises(RuntimeError, match='startup failed'):
        reserve.main()
    assert reserve.tracking.TrackingProbe is original
