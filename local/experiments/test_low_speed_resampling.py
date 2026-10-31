"""Behavior and independent audit checks for bounded native low-speed retries."""
import copy
import json

import pytest
import torch

from local.experiments import inference_probe as native
from local.experiments.audit_quality import low_speed_annotations_match
from local.experiments.test_validated_selection import fake_probe


@pytest.mark.parametrize("signal", ["green", "unknown"])
@pytest.mark.parametrize("later_speed,expected_pool,met", [(4., 1, True), (2., 1, False), (.5, 0, False), (None, 0, False)])
def test_bounded_retry_preserves_best_native_sample(tmp_path, monkeypatch, signal, later_speed, expected_pool, met):
    probe, adapter, context, options, prediction, calls = fake_probe(tmp_path, monkeypatch)
    probe.visual_signal = signal
    context["measured_speed_mps"] = 4.
    options.update(validation_resample_below_target_mps=3., validation_desired_speed_mps=4.,
                   validation_resample_language=True, seed=46, flow_seed=45,
                   validation_flow_temperatures=[1., 1.5])
    original = prediction[:, :, 1].clone()
    generate = native.Probe.evaluate
    generated = []

    def evaluate(*args, **kwargs):
        index = len(calls)
        speed = 1. if not index else later_speed
        prediction[:, :, 1] = original * ((speed or 1.) / 3.)
        if speed is None:
            prediction[:, :, :, :, 1] = 2.5
        result, extra = generate(*args, **kwargs)
        extra["cot"][0, 0, 1] = f"pool-{index}"
        generated.append(result[:, :, 1:2].clone())
        return result, extra

    monkeypatch.setattr(native.Probe, "evaluate", evaluate)
    selected, extra = probe.evaluate(adapter, {}, context, "Go straight", options, "frame_000")
    assert len(calls) == 2
    assert [c["seed"] for c in calls] == [46, 47]
    assert [c["flow_seed"] for c in calls] == [45, 46]
    assert [c["flow_temperature"] for c in calls] == [1., 1.5]
    assert torch.equal(selected, generated[expected_pool])
    assert extra["cot"][0, 0, 0] == f"pool-{expected_pool}"
    frame = json.loads((tmp_path / "frame_000.json").read_text())
    selection = frame["selection"]
    assert selection["proposal"] == f"frame_000_pool_{expected_pool}"
    if later_speed is None:
        assert selection["attempts"][-1]["chosen"] is None
    assert selection["low_speed_preference"]["met"] == met
    assert frame["trajectory_ego"] == frame["candidate_trajectories"][1]
    metric = selection["attempts"][expected_pool]["metrics"][1]
    assert low_speed_annotations_match(frame, metric)
    for field in ("met", "applied"):
        tampered = copy.deepcopy(frame)
        tampered["selection"]["low_speed_preference"][field] ^= True
        assert not low_speed_annotations_match(tampered, metric)
    tampered = copy.deepcopy(frame)
    tampered["selection"]["attempts"][0]["retry_for_low_speed_preference"] = False
    assert not low_speed_annotations_match(tampered, metric)


@pytest.mark.parametrize("signal", ["red", "yellow"])
def test_stop_signals_do_not_request_extra_low_speed_pools(tmp_path, monkeypatch, signal):
    probe, adapter, context, options, prediction, calls = fake_probe(tmp_path, monkeypatch)
    probe.visual_signal = signal
    options["validation_resample_below_target_mps"] = 3.
    selected, _ = probe.evaluate(adapter, {}, context, "Go straight", options, "frame_000")
    assert len(calls) == 1
    assert torch.equal(selected, prediction[:, :, 2:3])
    frame = json.loads((tmp_path / "frame_000.json").read_text())
    assert not frame["selection"]["low_speed_preference"]["applied"]
    assert not frame["selection"]["attempts"][0]["retry_for_low_speed_preference"]
    selection = frame["selection"]
    metric = selection["attempts"][0]["metrics"][selection["chosen"]]
    assert low_speed_annotations_match(frame, metric)


def test_low_speed_preference_is_absent_by_default(tmp_path, monkeypatch):
    probe, adapter, context, options, prediction, calls = fake_probe(tmp_path, monkeypatch)
    prediction[:, :, 1] *= .1
    selected, _ = probe.evaluate(adapter, {}, context, "Go straight", options, "frame_000")
    assert len(calls) == 1
    assert torch.equal(selected, prediction[:, :, 1:2])
    frame = json.loads((tmp_path / "frame_000.json").read_text())
    assert "low_speed_preference" not in frame["selection"]
    selection = frame["selection"]
    metric = selection["attempts"][0]["metrics"][selection["chosen"]]
    assert low_speed_annotations_match(frame, metric)


@pytest.mark.parametrize("threshold", [0., -1., float("nan"), float("inf"), 6.])
def test_invalid_retry_band_fails_before_sampling(tmp_path, monkeypatch, threshold):
    probe, adapter, context, options, _, calls = fake_probe(tmp_path, monkeypatch)
    options.update(validation_resample_below_target_mps=threshold, validation_resample_above_target_mps=5.)
    with pytest.raises(ValueError, match="preference"):
        probe.evaluate(adapter, {}, context, "Go straight", options, "frame_000")
    assert not calls


def test_already_met_preference_uses_one_pool(tmp_path, monkeypatch):
    probe, adapter, context, options, prediction, calls = fake_probe(tmp_path, monkeypatch)
    prediction[:, :, 1] *= 4. / 3.
    options.update(validation_resample_below_target_mps=3., validation_max_pools=6)
    selected, _ = probe.evaluate(adapter, {}, context, "Go straight", options, "frame_000")
    assert len(calls) == 1
    assert torch.equal(selected, prediction[:, :, 1:2])
    frame = json.loads((tmp_path / "frame_000.json").read_text())
    assert frame["selection"]["low_speed_preference"]["met"]
    assert not frame["selection"]["attempts"][0]["retry_for_low_speed_preference"]
