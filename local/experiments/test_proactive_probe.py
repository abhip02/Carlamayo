"""Fresh and cached proposals must keep distinct, exact native provenance."""
import copy
import json

import numpy as np
import pytest
import torch

from local.experiments import retained_probe as retained
from local.experiments.proactive_probe import ProactiveProbe
from local.experiments.test_retained_probe import setup_plan
from local.experiments.validated_selection import assess, choose


@pytest.mark.parametrize("fresh_speed,age,reuse", [(1.2, .4, True), (5., .4, False), (1.2, .6, False)])
def test_compare_preserves_both_native_sources_and_does_not_refresh_cached_age(
        tmp_path, monkeypatch, fresh_speed, age, reuse):
    route, context, options, old, _ = setup_plan()
    context['time_s'] = age
    context['world_pose']['x'] += (age - .4) * 3.
    path = np.array(old.record['trajectory_ego']) * fresh_speed / 3.
    metrics = assess(np.array([path]), context, route, [True], np.full((1, 64), fresh_speed),
                     1.5, yaws=np.zeros((1, 64)), check_body=True, body_margin_m=.1)
    assert choose(metrics, [path], 'green', context['measured_speed_mps'], 5.) == 0
    fresh = copy.deepcopy(old.record)
    fresh.update(label='frame_004', context=context, trajectory_ego=path.tolist(),
                 candidate_trajectories=[path.tolist()], inference_seconds=.2,
                 generated_token_ids=[10, 1], candidate_generated_tokens=[[10, 1]],
                 candidate_velocities=[[fresh_speed] * 64],
                 selection={'chosen': 0, 'signal': 'green', 'route': 'straight',
                            'proposal': 'frame_004_pool_0',
                            'attempts': [{'proposal': 'frame_004_pool_0', 'chosen': 0, 'metrics': metrics}]})
    prediction = torch.tensor(path[None, None, None])
    extra = {'cot': np.array([[['new native intent']]])}
    probe = object.__new__(ProactiveProbe)
    probe.output, probe.history = tmp_path, [old]
    probe.validation_route, probe.navigation_phase = route, 'approach'
    probe.visual_signal, probe.visual_signal_answer, probe.visual_signal_index = 'green', 'green', 4

    def sample_fresh(self, *args, **kwargs):
        retained.native.write_json(tmp_path / 'frame_004.json', fresh)
        plan = retained.ModelPlan.from_record(fresh, prediction, extra)
        self.history.append(plan)
        self.executing_plan, self.executing_age = plan, 0.
        return prediction, extra

    monkeypatch.setattr(retained.RetainingProbe, 'evaluate', sample_fresh)
    actual, associated = probe.evaluate(None, {}, context, 'Go straight', options, 'frame_004')
    executed = json.loads((tmp_path / 'frame_004.json').read_text())
    archived = json.loads((tmp_path / 'accepted_frame_004.json').read_text())
    assert archived['trajectory_ego'] == path.tolist() and 'retention' not in archived
    assert probe.history[-1].record['label'] == 'accepted_frame_004'
    assert len(probe.history) == 2  # A cached execution is never reinserted as a new prediction.
    assert old.record['context']['time_s'] == 0.
    if reuse:
        assert torch.equal(actual, old.prediction)
        assert associated['cot'][0, 0, 0] == 'original model intent'
        assert executed['retention']['reason'] == 'cached_plan_ranked_better'
        assert executed['retention']['fresh_candidate_source'] == 'accepted_frame_004'
        assert executed['retention']['age_s'] == age
        assert probe.executing_plan is old
    else:
        assert torch.equal(actual, prediction)
        assert associated['cot'][0, 0, 0] == 'new native intent'
        assert 'retention' not in executed and probe.executing_age == 0.
    assert not (tmp_path / 'validation-failure.json').exists()
