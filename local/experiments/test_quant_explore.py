"""Numerical checks for new sampling and activation quantization logic."""
import numpy as np
import pytest
import torch

from local.experiments import model_score_benchmark as base
from local.experiments.quant_explore import activation_qdq, anchors, score_plan
from local.experiments.test_model_score_benchmark import fixture


def test_anchors_have_complete_history_and_future():
    rows = [dict(time_s=i * .1) for i in range(63)]
    assert anchors(rows) == [15, 23, 31]
    with pytest.raises(ValueError):
        anchors(rows[:46])


def test_qdq_uses_independent_token_scales_and_signed_rounding():
    values = torch.tensor([[0., .49, .51, 7., -3.2], [0., .049, .051, .7, -.32]])
    expected = torch.tensor([[0., 0., 1., 7., -3.], [0., 0., .1, .7, -.3]])
    torch.testing.assert_close(activation_qdq(values, 4), expected)
    torch.testing.assert_close(activation_qdq(torch.zeros(2, 8), 4), torch.zeros(2, 8))
    assert activation_qdq(values.bfloat16(), 8).dtype == torch.bfloat16
    error4 = (activation_qdq(values, 4) - values).abs().mean()
    error8 = (activation_qdq(values, 8) - values).abs().mean()
    assert error8 < error4


def test_short_reference_and_original_scorer_are_preserved():
    context, route, trajectory = fixture()
    score, _ = score_plan(trajectory, np.zeros(64), context, route, trajectory[:30], None)
    assert score["recorded_reference_ade_m_3s"] == pytest.approx(0., abs=1e-6)
    assert score["route_progress_m_3s"] == pytest.approx(6., abs=1e-6)
    assert not any(key.endswith("_6s") for key in score)
    assert base.HORIZONS == (1, 3, 6)
    trajectory[0, 0] = np.nan
    with pytest.raises(ValueError):
        score_plan(trajectory, np.zeros(64), context, route, trajectory[:30], None)
    assert base.HORIZONS == (1, 3, 6)
