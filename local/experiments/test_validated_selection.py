"""Behavior checks for the opt-in model-only validator, without GPUs or CARLA RPC."""
import json
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from local.experiments import inference_probe as native
from local.experiments.validated_probe import RouteExitReached, ValidatingProbe
from local.experiments.validated_selection import assess, choose
from module.config import EGO_REAR_AXLE_X


def fixture_data():
    points = [{"x": x, "y": 0, "yaw": 0, "lane_width": 3.5} for x in (0, 50, 100)]
    route = {"points": points, "entry": points[0], "exit": points[1], "exit_index": 1}
    context = {"closed_loop": True, "index": 0, "time_s": 0,
               "route_entry": {"junction_id": 103}, "measured_speed_mps": 3.,
               "world_pose": {"x": -EGO_REAR_AXLE_X, "y": 0, "z": 0,
                              "yaw": 0, "pitch": 0, "roll": 0}}
    path = np.zeros((64, 3), dtype=np.float32)
    path[:, 0] = np.arange(1, 65) * .3
    return route, context, path


def test_wrong_lane_reverse_and_malformed_paths_are_rejected():
    route, context, path = fixture_data()
    candidates = np.stack([path.copy() for _ in range(4)])
    candidates[1, :, 1] = 2.5
    candidates[2, :, 0] *= -1
    velocities = np.full((4, 64), 3.)
    velocities[2] = -3
    metrics = assess(candidates, context, route, [True, True, True, False], velocities)
    assert [m["valid"] for m in metrics] == [True, False, False, False]
    assert choose(metrics, candidates, "green", 3) == 0


def test_green_prefers_progress_and_red_prefers_model_stop():
    route, context, path = fixture_data()
    candidates = np.stack([np.zeros_like(path), path])
    metrics = assess(candidates, context, route, [True, True], np.array([[0.] * 64, [3.] * 64]))
    assert choose(metrics, candidates, "green", 3) == 1
    assert choose(metrics, candidates, "red", 3) == 0


def test_short_horizon_still_checks_actual_controller_lookahead():
    route, context, path = fixture_data()
    path[:, 0] = np.arange(1, 65) * .1
    path[20:, 1] = np.linspace(0, 3., 44)
    result = assess(np.array([path]), context, route, [True], np.ones((1, 64)), horizon_s=1.5)[0]
    assert result["controller_target_index"] > 15
    assert result["horizon_s"] >= (result["controller_target_index"] + 1) * .1
    assert not result["valid"]


def fake_probe(tmp_path, monkeypatch, all_bad=False):
    route, context, path = fixture_data()
    route_path = tmp_path / "routes.json"
    route_path.write_text(json.dumps({"junction_id": 103, "routes": {"straight": route}}))
    candidates = np.stack([path.copy() for _ in range(4)])
    candidates[0, :, 1] = 2.5
    candidates[2] *= 0
    if all_bad:
        candidates[:, :, 1] = 2.5
    prediction = torch.from_numpy(candidates[None, None])
    calls = []

    def evaluate(self, adapter, data, context, instruction, options, label, **kwargs):
        calls.append(options)
        native.write_json(self.output / (label + ".json"), {
            "label": label, "context": context, "options": options, "inference_seconds": .1,
            "candidate_trajectories": candidates.tolist(), "candidate_markers": [True, True, True, False],
            "candidate_velocities": [[3.] * 64, [3.] * 64, [0.] * 64, [3.] * 64],
            "candidate_accelerations": [[0.] * 64] * 4,
            "candidate_generated_tokens": [[10, i] for i in range(4)]})
        return prediction, {"cot": np.array([[['bad', 'moving', 'stopped', 'malformed']]])}

    monkeypatch.setattr(native.Probe, "evaluate", evaluate)
    probe = object.__new__(ValidatingProbe)
    probe.output = tmp_path
    probe.visual_signal = "green"
    probe.model = SimpleNamespace(tokenizer=SimpleNamespace(
        convert_tokens_to_ids=lambda token: 10, decode=lambda tokens, **kwargs: str(tokens)))
    adapter = SimpleNamespace(extract_cot_text=lambda extra: extra["cot"][0, 0, 0])
    options = {"verify_signal": True, "validation_routes": str(route_path),
               "validation_route": "straight", "validation_max_pools": 2, "instruction": "Go straight"}
    return probe, adapter, context, options, prediction, calls


def test_wrapper_returns_exact_native_sample_and_matching_reasoning(tmp_path, monkeypatch):
    probe, adapter, context, options, prediction, calls = fake_probe(tmp_path, monkeypatch)
    actual, extra = probe.evaluate(adapter, {}, context, "Go straight", options, "frame_000")
    assert torch.equal(actual, prediction[:, :, 1:2])
    assert extra["cot"][0, 0, 0] == "moving"
    record = json.loads((tmp_path / "frame_000.json").read_text())
    assert record["trajectory_ego"] == record["candidate_trajectories"][1]
    assert record["selection"]["chosen"] == 1
    assert len(calls) == 1


def test_no_valid_candidate_terminates_without_executed_path(tmp_path, monkeypatch):
    probe, adapter, context, options, _, calls = fake_probe(tmp_path, monkeypatch, all_bad=True)
    with pytest.raises(RuntimeError, match="No valid model-generated plan"):
        probe.evaluate(adapter, {}, context, "Go straight", options, "frame_000")
    assert len(calls) == 2
    assert [c["flow_seed"] for c in calls] == [42, 43]
    assert (tmp_path / "validation-failure.json").exists()
    assert (tmp_path / "inputs/frame_000.pt").exists()
    assert not (tmp_path / "frame_000.json").exists()


def test_completion_does_not_claim_success_outside_exit_lane(tmp_path, monkeypatch):
    probe, adapter, context, options, _, calls = fake_probe(tmp_path, monkeypatch)
    options["validation_stop_at_exit"] = True
    context["world_pose"].update(x=51., y=0.)
    with pytest.raises(RouteExitReached):
        probe.evaluate(adapter, {}, context, "Go straight", options, "frame_000")
    assert not calls
    context["world_pose"]["y"] = 3.
    with pytest.raises(RuntimeError, match="No valid model-generated plan"):
        probe.evaluate(adapter, {}, context, "Go straight", options, "frame_000")
