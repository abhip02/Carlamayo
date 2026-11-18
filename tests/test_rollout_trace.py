"""Trace integrity checks use only the standard library and recorded files."""
import json

import pytest

from local.rollout_trace import load_rollout_result, validate_result


@pytest.mark.parametrize("termination", ["duration_reached", "collision"])
def test_loads_all_recorded_states_controls_predictions_and_collision(recorded_rollout, termination):
    directory, expected = recorded_rollout(termination=termination, steps=40)
    result = load_rollout_result(directory, expected["summary"])
    assert result == expected
    assert len(result["trace"]["samples"]) == 41
    assert len(result["trace"]["predictions"]) == 40
    assert json.loads(json.dumps(result, allow_nan=False)) == result


@pytest.mark.parametrize("mutate,message", [
    (lambda r: r.pop("trace"), "Missing full rollout trace"),
    (lambda r: r["trace"].update(schema_version=2), "schema_version"),
    (lambda r: r["trace"]["samples"].pop(), "every completed step"),
    (lambda r: r["trace"]["predictions"].pop(), "every model prediction"),
    (lambda r: r["trace"]["samples"][0].update(time_s=0.01), "handover time 0"),
    (lambda r: r["trace"]["samples"][1].update(time_s=0), "strictly increasing"),
    (lambda r: r["trace"]["samples"][1].update(frame=100), "strictly increasing"),
    (lambda r: r["trace"]["samples"][1].update(frame=100.5), "integer CARLA frame"),
    (lambda r: r["summary"].update(simulation_seconds=8), "final time"),
    (lambda r: r["trace"]["predictions"][0].update(time_s=0.1), "align"),
    (lambda r: r["trace"]["predictions"][0].update(frame=101), "align"),
    (lambda r: r["trace"]["predictions"][0].update(inference_s=0.5), "align"),
    (lambda r: r["trace"]["predictions"][0].update(trajectory_ego=[[1, 2]]), "XYZ"),
    (lambda r: r["trace"]["samples"][0].update(speed=True), "numeric state"),
    (lambda r: r["trace"]["samples"][1].update(speed=float("nan")), "finite JSON"),
    (lambda r: r["trace"]["predictions"][0].update(trajectory_ego=[[float("inf"), 0, 0]]),
     "finite JSON"),
])
def test_rejects_missing_corrupted_or_misaligned_traces(rollout_result, mutate, message):
    result = rollout_result()
    mutate(result)
    with pytest.raises(ValueError, match=message):
        validate_result(result)


@pytest.mark.parametrize("filename", ["ego.csv", "predictions.jsonl"])
def test_missing_recording_is_an_error(recorded_rollout, filename):
    directory, result = recorded_rollout()
    (directory / filename).unlink()
    with pytest.raises(FileNotFoundError):
        load_rollout_result(directory, result["summary"])


def test_partial_csv_row_cannot_be_silently_discarded(recorded_rollout):
    directory, result = recorded_rollout()
    with (directory / "ego.csv").open("a") as stream:
        stream.write("0.3,103,1.0\n")
    with pytest.raises(ValueError, match="Incomplete"):
        load_rollout_result(directory, result["summary"])


def test_failed_simulation_is_not_a_specification_violation(rollout_result):
    result = rollout_result()
    result["summary"].update(termination="error", error="camera timeout")
    with pytest.raises(RuntimeError, match="camera timeout"):
        validate_result(result)
