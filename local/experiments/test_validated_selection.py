"""Behavior checks for the opt-in model-only validator, without GPUs or CARLA RPC."""
import json
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from local.experiments import inference_probe as native
from local.experiments.validated_probe import RouteExitReached, ValidatingProbe
from local.experiments.validated_selection import assess, choose, pose_transform, predicted_corners, require_green_progress
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


def test_optional_green_progress_requirement_does_not_reject_red_or_unknown_stops():
    for signal in ("green", "red", "unknown"):
        metrics = [{"valid": True, "reasons": [], "target_speed_mps": .1},
                   {"valid": True, "reasons": [], "target_speed_mps": 2.}]
        require_green_progress(metrics, signal, 1.)
        assert metrics[0]["valid"] == (signal != "green")
        assert metrics[1]["valid"]
        assert metrics[0]["target_speed_mps"] == .1


def test_ranking_compares_same_time_window_despite_different_steering_horizons():
    route, context, path = fixture_data()
    candidates = np.zeros((2, 64, 3), dtype=np.float32)
    velocities = np.array([[1.] * 64, [.1] * 15 + [3.] * 49])
    candidates[:, :, 0] = np.cumsum(velocities * .1, axis=1)
    metrics = assess(candidates, context, route, [True, True], velocities, horizon_s=1.5)
    assert all(item["valid"] for item in metrics)
    assert metrics[0]["horizon_s"] != metrics[1]["horizon_s"]
    assert metrics[0]["ranking_horizon_s"] == metrics[1]["ranking_horizon_s"] == 1.5
    assert choose(metrics, candidates, "green", 3) == 0


def test_short_horizon_still_checks_actual_controller_lookahead():
    route, context, path = fixture_data()
    path[:, 0] = np.arange(1, 65) * .1
    path[20:, 1] = np.linspace(0, 3., 44)
    result = assess(np.array([path]), context, route, [True], np.ones((1, 64)), horizon_s=1.5)[0]
    assert result["controller_target_index"] > 15
    assert result["horizon_s"] >= (result["controller_target_index"] + 1) * .1
    assert not result["valid"]


def test_body_check_rejects_lane_overhang_that_rear_axle_check_accepts():
    route, context, path = fixture_data()
    context["vehicle_bounds"] = {
        "location": {"x": 0., "y": 0., "z": .75},
        "extent": {"x": 2.45, "y": .92, "z": .75},
        "rotation": {"pitch": 0., "yaw": 0., "roll": 0.}}
    path[:, 1] = np.minimum(np.arange(1, 65) * .05, 1.2)
    args = (np.array([path]), context, route, [True], np.full((1, 64), 3.))
    assert assess(*args)[0]["valid"]
    result = assess(*args, yaws=np.zeros((1, 64)), check_body=True)[0]
    assert not result["valid"]
    assert result["max_corner_overhang_m"] > .3
    assert "vehicle_corners_outside_lane_corridor" in result["reasons"]


def test_body_clearance_margin_rejects_a_near_edge_native_path_without_repairing_it():
    route, context, path = fixture_data()
    context["vehicle_bounds"] = {
        "location": {"x": 0., "y": 0., "z": .75},
        "extent": {"x": 2.45, "y": .92, "z": .75},
        "rotation": {"pitch": 0., "yaw": 0., "roll": 0.}}
    path[:, 1] = np.minimum(np.arange(1, 65) * .05, .75)
    candidates = np.array([path])
    original = candidates.copy()
    args = (candidates, context, route, [True], np.full((1, 64), 3.))
    body = dict(yaws=np.zeros((1, 64)), check_body=True)
    assert assess(*args, **body)[0]["valid"]
    metric = assess(*args, **body, body_margin_m=.1)[0]
    assert not metric["valid"] and metric["max_corner_overhang_m"] < 0
    assert "insufficient_vehicle_corner_clearance" in metric["reasons"]
    np.testing.assert_array_equal(candidates, original)
    with pytest.raises(ValueError, match="body margin"):
        assess(*args, body_margin_m=.1)


@pytest.mark.parametrize("pitch,roll,native_yaw", [(0., 0., .2), (3., 2., 0.)])
def test_predicted_box_matches_carla_geometry_and_left_yaw_sign(pitch, roll, native_yaw):
    import carla
    _, context, _ = fixture_data()
    context["world_pose"].update(yaw=34., pitch=pitch, roll=roll)
    tf = pose_transform(context["world_pose"])
    bounds = {"location": {"x": -.006, "y": 0., "z": .75},
              "extent": {"x": 2.45, "y": .92, "z": .75},
              "rotation": {"pitch": 0., "yaw": 0., "roll": 0.}}
    candidate = np.array([[5., 0., 0.]])
    actual = predicted_corners(tf, candidate, np.array([native_yaw]), bounds)[0]
    expected_rotation = carla.Rotation(yaw=34. - np.degrees(native_yaw), pitch=pitch, roll=roll)
    expected = carla.Transform(carla.Location(), expected_rotation)
    rear = tf.transform(carla.Location(x=5. + EGO_REAR_AXLE_X))
    offset = expected.transform(carla.Location(x=-EGO_REAR_AXLE_X))
    expected.location = rear + offset
    box = carla.BoundingBox(carla.Location(**bounds["location"]), carla.Vector3D(**bounds["extent"]))
    vertices = np.array([[p.x, p.y, p.z] for p in box.get_world_vertices(expected)])
    np.testing.assert_allclose(actual, vertices, atol=2e-5)


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


def test_retry_can_resample_language_and_return_an_exact_native_candidate(tmp_path, monkeypatch):
    probe, adapter, context, options, prediction, calls = fake_probe(tmp_path, monkeypatch, all_bad=True)
    options.update(seed=43, validation_resample_language=True, validation_flow_temperatures=[1., 1.5])
    generate_pool = native.Probe.evaluate

    def evaluate(*args, **kwargs):
        if len(calls) == 1:
            prediction[:, :, 1, :, 1] = 0  # The second mocked pool contains one valid native plan.
        return generate_pool(*args, **kwargs)

    monkeypatch.setattr(native.Probe, "evaluate", evaluate)
    actual, _ = probe.evaluate(adapter, {}, context, "Go straight", options, "frame_000")
    assert [c["seed"] for c in calls] == [43, 44]
    assert [c["flow_seed"] for c in calls] == [42, 43]
    assert [c["flow_temperature"] for c in calls] == [1., 1.5]
    assert torch.equal(actual, prediction[:, :, 1:2])
    record = json.loads((tmp_path / "frame_000.json").read_text())
    assert [attempt["chosen"] for attempt in record["selection"]["attempts"]] == [None, 1]


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


@pytest.mark.parametrize("later_speed,expected_pool", [(5., 1), (9., 0), (None, 0)])
def test_quality_retry_preserves_best_native_array_and_text_even_if_later_pool_fails(
        tmp_path, monkeypatch, later_speed, expected_pool):
    probe, adapter, context, options, prediction, calls = fake_probe(tmp_path, monkeypatch)
    options.update(validation_resample_above_target_mps=6., validation_green_min_target_mps=1.)
    original_path = prediction[:, :, 1].clone()
    generate_pool = native.Probe.evaluate
    generated = []

    def evaluate(*args, **kwargs):
        index = len(calls)
        speed = 7. if not index else later_speed
        prediction[:, :, 1] = original_path * ((speed or 7.) / 3.)
        if speed is None:
            prediction[:, :, 1, :, 1] = 2.5
        result, extra = generate_pool(*args, **kwargs)
        extra['cot'][0, 0, 1] = f'pool-{index}'
        generated.append(result[:, :, 1:2].clone())
        return result, extra

    monkeypatch.setattr(native.Probe, "evaluate", evaluate)
    actual, extra = probe.evaluate(adapter, {}, context, "Go straight", options, "frame_000")
    assert len(calls) == 2
    assert torch.equal(actual, generated[expected_pool])
    assert extra['cot'][0, 0, 0] == f'pool-{expected_pool}'
    record = json.loads((tmp_path / 'frame_000.json').read_text())
    assert record['selection']['proposal'] == f'frame_000_pool_{expected_pool}'
    assert record['trajectory_ego'] == record['candidate_trajectories'][1]
    assert record['selection']['speed_quality_preference']['met'] == (later_speed == 5.)
    assert not (tmp_path / 'validation-failure.json').exists()


def test_quality_retry_does_not_change_red_stop_selection(tmp_path, monkeypatch):
    probe, adapter, context, options, prediction, calls = fake_probe(tmp_path, monkeypatch)
    probe.visual_signal = 'red'
    options.update(validation_resample_above_target_mps=1., validation_green_min_target_mps=1.)
    actual, _ = probe.evaluate(adapter, {}, context, "Go straight", options, "frame_000")
    assert len(calls) == 1 and torch.equal(actual, prediction[:, :, 2:3])


@pytest.mark.parametrize("later_preferred,later_valid,expected_pool", [(True, True, 1), (False, True, 0), (False, False, 0)])
def test_reserve_retry_prefers_larger_clearance_but_keeps_exact_minimum_admitted_sample(
        tmp_path, monkeypatch, later_preferred, later_valid, expected_pool):
    from local.experiments import validated_probe as validated
    probe, adapter, context, options, prediction, calls = fake_probe(tmp_path, monkeypatch)
    options.update(validation_preferred_reserve_m=.2, validation_body_margin_m=0.,
                   validation_tracking_margin_m=.1, validation_green_min_target_mps=1.)
    source = prediction[:, :, 1].clone()
    generate, assess_original = native.Probe.evaluate, validated.assess
    generated = []

    def evaluate(*args, **kwargs):
        index = len(calls)
        # First candidate has a better speed score. Preference must still win
        # when a later native sample has the larger required clearance.
        prediction[:, :, 1] = source * (5. / 3. if index == 0 else 1.)
        if index and not later_valid:
            prediction[:, :, 1, :, 1] = 2.5
        result, extra = generate(*args, **kwargs)
        extra['cot'][0, 0, 1] = f'pool-{index}'
        generated.append(result[:, :, 1:2].clone())
        return result, extra

    def assess(*args, **kwargs):
        metrics = assess_original(*args, **kwargs)
        for m in metrics:
            m['preferred_reserve_valid'] = bool(len(calls) > 1 and later_preferred and m['valid'])
        return metrics

    monkeypatch.setattr(native.Probe, 'evaluate', evaluate)
    monkeypatch.setattr(validated, 'assess', assess)
    actual, extra = probe.evaluate(adapter, {}, context, 'Go straight', options, 'frame_000')
    assert len(calls) == 2
    assert torch.equal(actual, generated[expected_pool])
    assert extra['cot'][0, 0, 0] == f'pool-{expected_pool}'
    record = json.loads((tmp_path / 'frame_000.json').read_text())
    assert record['selection']['reserve_preference']['met'] == later_preferred
    assert record['selection']['attempts'][0]['retry_for_reserve_preference']
    assert record['trajectory_ego'] == record['candidate_trajectories'][1]
    assert not (tmp_path / 'validation-failure.json').exists()


def test_reserve_preference_never_admits_a_failed_sample():
    route, context, path = fixture_data()
    candidates = np.stack([path * (5. / 3.), path])
    metrics = assess(candidates, context, route, [True, True], np.array([[5.] * 64, [3.] * 64]), 1.5)
    metrics[0]['preferred_reserve_valid'], metrics[1]['preferred_reserve_valid'] = False, True
    assert choose(metrics, candidates, 'green', 3.) == 1
    metrics[1]['valid'] = False
    assert choose(metrics, candidates, 'green', 3.) == 0


@pytest.mark.parametrize("longer_available", [True, False])
def test_longer_forecast_preference_preserves_native_paths_and_short_admission(
        tmp_path, monkeypatch, longer_available):
    probe, adapter, context, options, prediction, calls = fake_probe(tmp_path, monkeypatch)
    options.update(validation_horizon_s=1.5, validation_prefer_horizon_s=3.,
                   validation_green_min_target_mps=1.)
    # Candidate 0 is initially valid and faster, but leaves the lane after 2.5 s.
    prediction[:, :, 0] = prediction[:, :, 1].clone() * 1.4
    prediction[:, :, 0, 25:, 1] = 2.5
    if not longer_available:
        prediction[:, :, 1, 25:, 1] = 2.5
    actual, _ = probe.evaluate(adapter, {}, context, "Go straight", options, "frame_000")
    chosen = 1 if longer_available else 0
    assert torch.equal(actual, prediction[:, :, chosen:chosen + 1])
    record = json.loads((tmp_path / "frame_000.json").read_text())
    metrics = record["selection"]["attempts"][0]["metrics"]
    assert metrics[0]["valid"] and not metrics[0]["preferred_horizon_valid"]
    assert metrics[1]["preferred_horizon_valid"] == longer_available
    assert not metrics[2]["valid"]  # Longer-valid stops still fail the green floor.
    assert len(calls) == 1


def test_longer_forecast_preference_does_not_override_red_stop_ranking(tmp_path, monkeypatch):
    probe, adapter, context, options, prediction, _ = fake_probe(tmp_path, monkeypatch)
    probe.visual_signal = "red"
    options.update(validation_prefer_horizon_s=3., validation_green_min_target_mps=1.)
    actual, _ = probe.evaluate(adapter, {}, context, "Go straight", options, "frame_000")
    assert torch.equal(actual, prediction[:, :, 2:3])
    record = json.loads((tmp_path / "frame_000.json").read_text())
    assert all("preferred_horizon_valid" not in m for m in record["selection"]["attempts"][0]["metrics"])


def test_trajectory_guard_does_not_block_vqa_termination():
    vocab = {"<|traj_future_start|>": 10, "<|answer_start|>": 11,
             "<|answer_end|>": 12, "<|im_end|>": 13, "<|endoftext|>": 14}
    probe = object.__new__(native.Probe)
    probe.options = {"guard_trajectory_format": True}
    probe.trace = {}
    probe.model = SimpleNamespace(tokenizer=SimpleNamespace(
        convert_tokens_to_ids=vocab.__getitem__, get_vocab=lambda: vocab,
        decode=lambda tokens, **kwargs: str(tokens)))
    observed = []

    def generate(**kwargs):
        scores = torch.zeros((1, 20))
        for processor in kwargs.get("logits_processor", []):
            scores = processor(kwargs["input_ids"], scores)
        observed.append(scores)
        return SimpleNamespace(sequences=torch.cat([kwargs["input_ids"], torch.tensor([[12]])], dim=1))

    probe.original_generate = generate
    config = SimpleNamespace(num_return_sequences=1)
    probe.generate(input_ids=torch.tensor([[11]]), generation_config=config)
    probe.generate(input_ids=torch.tensor([[15]]), generation_config=config)
    assert observed[0][0, 12] == 0  # VQA must be able to end its answer.
    assert torch.isneginf(observed[1][0, 12])  # Driving must first emit its trajectory marker.
    assert observed[1][0, 10] == 0
