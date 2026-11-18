"""Simulator-independent loading and validation of the VerifAI rollout result."""
from __future__ import annotations

import csv
import json
import math
from numbers import Real
from pathlib import Path


STATE_FIELDS = ("time_s", "frame", "x", "y", "z", "yaw", "pitch", "roll", "speed",
                "steer", "throttle", "brake", "inference_s")
TRACE_SCHEMA_VERSION = 1


def finite_number(value):
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def validate_result(result):
    """Reject failed or incomplete episodes before invoking any specification."""
    if not isinstance(result, dict) or not isinstance(result.get("summary"), dict):
        raise ValueError("Rollout result must contain a summary mapping")
    summary = result["summary"]
    if summary.get("error") or summary.get("termination") not in ("collision", "duration_reached"):
        raise RuntimeError(f"Simulation failed or did not complete: {summary}")
    trace = result.get("trace")
    if not isinstance(trace, dict):
        raise ValueError("Missing full rollout trace")
    if type(trace.get("schema_version")) is not int or trace["schema_version"] != TRACE_SCHEMA_VERSION:
        raise ValueError("Unsupported rollout trace schema_version")
    samples, predictions = trace.get("samples"), trace.get("predictions")
    calls = summary.get("inference_calls")
    if type(calls) is not int or calls < 0:
        raise ValueError("Summary inference_calls must be a nonnegative integer")
    if not isinstance(samples, list) or len(samples) != calls + 1:
        raise ValueError("Trace must include handover and every completed step")
    if not isinstance(predictions, list) or len(predictions) != calls:
        raise ValueError("Trace must include every model prediction")
    # Also covers non-finite values nested inside trajectories or summary data.
    try:
        json.dumps(result, allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise ValueError("Rollout result must contain finite JSON data") from exc
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict) or any(
            field not in sample or not finite_number(sample[field]) for field in STATE_FIELDS
        ):
            raise ValueError(f"Trace sample {index} must contain all numeric state/control fields")
        if type(sample["frame"]) is not int or sample["frame"] < 0:
            raise ValueError(f"Trace sample {index} must have an integer CARLA frame")
        if index == 0 and sample["time_s"] != 0:
            raise ValueError("Trace must start at handover time 0")
        if index and (sample["time_s"] <= samples[index - 1]["time_s"]
                      or sample["frame"] <= samples[index - 1]["frame"]):
            raise ValueError("Trace times and frames must be strictly increasing")
    duration = summary.get("simulation_seconds")
    if not finite_number(duration) or not math.isclose(duration, samples[-1]["time_s"], abs_tol=1e-9):
        raise ValueError("Trace final time must match summary simulation_seconds")
    for index, prediction in enumerate(predictions):
        if not isinstance(prediction, dict) or any(
            not finite_number(prediction.get(field)) for field in ("time_s", "frame", "inference_s")
        ):
            raise ValueError(f"Prediction {index} must contain numeric timing and frame fields")
        if (type(prediction["frame"]) is not int
                or prediction["frame"] != samples[index]["frame"]
                or not math.isclose(prediction["time_s"], samples[index]["time_s"], abs_tol=1e-9)
                or not math.isclose(prediction["inference_s"], samples[index + 1]["inference_s"],
                                    abs_tol=1e-9)):
            raise ValueError(f"Prediction {index} does not align with the recorded transition")
        trajectory = prediction.get("trajectory_ego")
        if not isinstance(trajectory, list) or not trajectory or any(
            not isinstance(point, list) or len(point) != 3
            or not all(finite_number(value) for value in point) for point in trajectory
        ):
            raise ValueError(f"Prediction {index} must contain a finite XYZ trajectory")
    return result


def load_rollout_result(directory, summary):
    """Load every saved state/control row and prediction, without downsampling.

    Schema 1 uses seconds since ego handover, CARLA world XYZ (meters) and
    yaw/pitch/roll (degrees), speed in m/s, normalized controls, and wall-clock
    inference seconds. Predictions retain their ego FLU coordinates (meters) at
    prediction time, plus all other logged fields, including reasoning text.
    At time t > 0 the controls describe the transition ending at t. The initial
    row's controls are the runner's zero placeholders. The summary carries the
    recorded collision event, handover, final state, and termination reason.
    """
    directory = Path(directory)
    with (directory / "ego.csv").open(newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != list(STATE_FIELDS):
            raise ValueError("Unexpected ego.csv columns for rollout trace schema 1")
        samples = []
        for row in reader:
            if None in row or any(row[field] is None for field in STATE_FIELDS):
                raise ValueError("Incomplete or malformed ego.csv row")
            samples.append({field: int(row[field]) if field == "frame" else float(row[field])
                            for field in STATE_FIELDS})
    with (directory / "predictions.jsonl").open() as stream:
        predictions = [json.loads(line) for line in stream]
    return validate_result({
        "summary": summary,
        "trace": {"schema_version": TRACE_SCHEMA_VERSION,
                  "samples": samples, "predictions": predictions},
    })
