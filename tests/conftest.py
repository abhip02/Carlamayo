import csv
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


@pytest.fixture
def rollout_result():
    """A complete synthetic recording with the production runner's time alignment."""
    def make(termination="duration_reached", steps=2, speed=3.0):
        samples = [dict(time_s=index * 0.1, frame=100 + index,
                        x=index * 0.3, y=0.0, z=0.0, yaw=0.0, pitch=0.0, roll=0.0,
                        speed=speed, steer=0.0, throttle=0.0 if index == 0 else 0.2,
                        brake=0.0, inference_s=0.0 if index == 0 else 0.025)
                   for index in range(steps + 1)]
        predictions = [dict(time_s=sample["time_s"], frame=sample["frame"],
                            inference_s=0.025, trajectory_ego=[[0.3, 0.0, 0.0], [0.6, 0.01, 0.0]],
                            reasoning="Continue straight.\nCheck the road ahead.")
                       for sample in samples[:-1]]
        summary = dict(termination=termination, inference_calls=steps,
                       simulation_seconds=samples[-1]["time_s"])
        if termination == "collision":
            summary["collision"] = dict(frame=samples[-1]["frame"], other_actor="static.pole",
                                        other_actor_id=0, intensity=100.0)
        return {"summary": summary,
                "trace": {"schema_version": 1, "samples": samples, "predictions": predictions}}
    return make


@pytest.fixture
def recorded_rollout(tmp_path, rollout_result):
    def record(name="0", **kwargs):
        result = rollout_result(**kwargs)
        directory = tmp_path / "recorded" / name
        directory.mkdir(parents=True)
        with (directory / "ego.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(result["trace"]["samples"][0]))
            writer.writeheader()
            writer.writerows(result["trace"]["samples"])
        (directory / "predictions.jsonl").write_text("".join(
            json.dumps(prediction) + "\n" for prediction in result["trace"]["predictions"]))
        (directory / "summary.json").write_text(json.dumps(result["summary"]))
        return directory, result
    return record
