"""Retention must preserve a native path's world anchor, clock and provenance."""
import copy
import io
import json
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from local.experiments import retained_probe as retained, validated_probe as validated
from local.experiments.test_validated_selection import fixture_data


def setup_plan():
    route, context, path = fixture_data()
    context["vehicle_bounds"] = {
        "location": {"x": 0., "y": 0., "z": .75},
        "extent": {"x": 2.45, "y": .92, "z": .75},
        "rotation": {"pitch": 0., "yaw": 0., "roll": 0.}}
    record = {"label": "frame_000", "context": context, "navigation_phase": "approach",
              "trajectory_ego": path.tolist(), "candidate_trajectories": [path.tolist()],
              "candidate_yaw_rad": [[0.] * 64], "candidate_velocities": [[3.] * 64],
              "selection": {"chosen": 0, "signal": "green", "route": "straight", "proposal": "frame_000_pool_0"}}
    prediction = torch.tensor(path[None, None, None])
    plan = retained.ModelPlan.from_record(record, prediction, {"cot": np.array([[['original model intent']]])})
    current = copy.deepcopy(context)
    current["index"], current["time_s"] = 4, .4
    current["world_pose"]["x"] += 1.2
    options = {"validation_check_body": True, "validation_cache_max_age_s": .5,
               "validation_body_margin_m": .1, "validation_route": "straight"}
    return route, current, options, plan, prediction


def test_cache_checks_remaining_path_at_original_time_and_cannot_refresh_age():
    route, context, options, plan, _ = setup_plan()
    before = plan.world.copy()
    metric = retained.assess_retained(plan, context, route, "green", "approach", options)
    assert metric["valid"] and metric["age_s"] == .4
    assert metric["checked_original_end_index"] >= 18
    context["time_s"] = .6
    expired = retained.assess_retained(plan, context, route, "green", "approach", options)
    assert not expired["valid"] and "expired_or_nonpast_plan" in expired["reasons"]
    np.testing.assert_array_equal(plan.world, before)
    assert plan.record["context"]["time_s"] == 0


@pytest.mark.parametrize("change,reason", [
    ("signal", "reuse_requires_green"), ("phase", "navigation_phase_changed"),
    ("position", "current_body_outside_lane"), ("heading", "misaligned_with_cached_path"),
    ("nonfinite", "nonfinite_current_state")])
def test_cache_is_revalidated_against_current_conditions(change, reason):
    route, context, options, plan, _ = setup_plan()
    signal, phase = "green", "approach"
    if change == "signal":
        signal = "red"
    elif change == "phase":
        phase = "finish_turn"
    elif change == "position":
        context["world_pose"]["y"] = 2.0
    elif change == "nonfinite":
        context["world_pose"]["x"] = float("nan")
    else:
        context["world_pose"]["yaw"] = 60.
    metric = retained.assess_retained(plan, context, route, signal, phase, options)
    assert not metric["valid"] and reason in metric["reasons"]


def test_cached_arrays_are_immutable_copies_and_keep_the_old_world_anchor(monkeypatch):
    _, context, _, plan, prediction = setup_plan()
    prediction.fill_(99.)
    assert not torch.equal(plan.prediction, prediction)
    monkeypatch.setattr(retained, "ACTIVE", SimpleNamespace(executing_plan=plan))
    world = retained.anchored_world_path(retained.pose_transform(context["world_pose"]),
                                         plan.prediction[0, 0, 0].numpy())
    np.testing.assert_array_equal(world, plan.world)
    assert not world.flags.writeable
    # Current car is 1.2 m ahead of the source: no translation follows it.
    assert world[0, 0] == pytest.approx(.3, abs=1e-6)


def test_exhausted_sampling_can_select_old_native_plan_with_explicit_rejection_log(tmp_path, monkeypatch):
    route, context, options, plan, _ = setup_plan()
    probe = object.__new__(retained.RetainingProbe)
    probe.output, probe.history = tmp_path, [plan]
    probe.validation_route, probe.navigation_phase = route, "approach"
    probe.visual_signal, probe.visual_signal_answer, probe.visual_signal_index = "green", "green", 4
    probe.execution_log = io.StringIO()

    def fail(self, *args, **kwargs):
        (tmp_path / "validation-failure.json").write_text(json.dumps({"attempts": [{"inference_seconds": .1}]}))
        raise validated.NoValidatedPlan("new samples rejected")

    monkeypatch.setattr(validated.ValidatingProbe, "evaluate", fail)
    result, extra = probe.evaluate(None, {}, context, "Go straight", options, "frame_004")
    assert torch.equal(result, plan.prediction)
    assert extra["cot"][0, 0, 0] == "original model intent"
    record = json.loads((tmp_path / "frame_004.json").read_text())
    assert record["trajectory_ego"] == plan.record["trajectory_ego"]
    assert record["retention"]["source_context"]["time_s"] == 0
    assert record["retention"]["age_s"] == .4
    assert (tmp_path / "replan-rejections/frame_004.json").exists()
    assert not (tmp_path / "validation-failure.json").exists()
    assert len(probe.history) == 1  # Reuse does not become a newly generated plan.


def test_cache_cannot_hide_inference_errors(tmp_path, monkeypatch):
    route, context, options, plan, _ = setup_plan()
    probe = object.__new__(retained.RetainingProbe)
    probe.output, probe.history = tmp_path, [plan]

    def fail(*args, **kwargs):
        raise RuntimeError("inference kernel failed")

    monkeypatch.setattr(validated.ValidatingProbe, "evaluate", fail)
    with pytest.raises(RuntimeError, match="inference kernel failed"):
        probe.evaluate(None, {}, context, "Go straight", options, "frame_004")


def test_original_controller_receives_original_arrays_and_real_plan_age(monkeypatch):
    _, context, _, plan, _ = setup_plan()
    observed, log = [], io.StringIO()
    monkeypatch.setattr(retained, "ACTIVE", SimpleNamespace(
        executing_plan=plan, executing_age=.4, execution_log=log))

    class Follower:
        def compute_control(self, tf, world, speed, speeds, elapsed_sec=0.):
            observed.append((world, speeds, elapsed_sec))
            return 0., 0., 0., {"elapsed_received": elapsed_sec}

    monkeypatch.setattr(retained.native.runner, "TrajectoryFollower", Follower)

    def drive_once():
        wrapped = retained.native.runner.TrajectoryFollower()
        wrapped.compute_control(retained.pose_transform(context["world_pose"]), plan.world,
                                context["measured_speed_mps"], plan.speeds)

    monkeypatch.setattr(validated, "main", drive_once)
    retained.main()
    assert observed[0][0] is plan.world and observed[0][1] is plan.speeds
    assert observed[0][2] == .4
    assert json.loads(log.getvalue())["age_s"] == .4
    assert retained.native.runner.TrajectoryFollower is Follower
