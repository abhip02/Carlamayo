"""CPU-only tests of the checkpoint's speed estimator on saved/synthetic histories."""
import csv
import json
from pathlib import Path

import numpy as np
import torch

from alpamayo1_5.action_space.unicycle_accel_curvature import UnicycleAccelCurvatureActionSpace
from local.run_intersection import ROOT, SNAPSHOT
from module.open_loop_dataset import build_ego_history


def main():
    config_path = ROOT.parent / "cache/huggingface/hub/models--nvidia--Alpamayo-1.5-10B/snapshots" / SNAPSHOT / "config.json"
    config = json.loads(config_path.read_text())["action_space_cfg"]
    kwargs = {key: value for key, value in config.items() if key != "_target_"}
    estimators = {str(value): UnicycleAccelCurvatureActionSpace(**{**kwargs, "v_lambda": value})
                  for value in (0.0, 1e-8, 1e-6, 1e-4)}
    results = []
    for run in ("verifai-trajectory-fix-straight-8s", "verifai-verbose-straight-8s"):
        path = ROOT / "outputs" / run / "000_verifai/ego.csv"
        rows = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(path.open())]
        for index in range(15, len(rows)):
            history = rows[index-15:index+1]
            xyz, rot = build_ego_history({str(i): state for i, state in enumerate(history)}, list(range(16)), 15)
            values = {name: float(est.estimate_t0_states(torch.as_tensor(xyz), torch.as_tensor(rot))["v"])
                      for name, est in estimators.items()}
            results.append({"run": run, "time_s": rows[index]["time_s"],
                            "measured_speed_mps": rows[index]["speed"], "estimates": values})
    synthetic = []
    for name, speeds in {
        "constant_3": np.full(16, 3.0),
        "smooth_stop": np.linspace(3.0, 0.0, 16),
        "abrupt_stop": np.r_[np.full(15, 1.7), 0.0],
    }.items():
        x = np.r_[0.0, np.cumsum(.05 * (speeds[:-1] + speeds[1:]))]
        xyz = torch.tensor(np.column_stack([x-x[-1], np.zeros((16, 2))]), dtype=torch.float32)
        rot = torch.eye(3).repeat(16, 1, 1)
        values = {key: float(est.estimate_t0_states(xyz, rot)["v"]) for key, est in estimators.items()}
        synthetic.append({"case": name, "true_speed": float(speeds[-1]), "estimates": values})
    # Known constant motion must remain correct before drawing conclusions about braking.
    assert abs(synthetic[0]["estimates"]["1e-06"] - 3.0) < .01
    summary = {}
    for key in estimators:
        error = np.array([row["estimates"][key] - row["measured_speed_mps"] for row in results])
        summary[key] = {"rmse_mps": float(np.sqrt(np.mean(error**2))),
                        "max_abs_error_mps": float(np.max(np.abs(error))),
                        "negative_estimates": sum(row["estimates"][key] < -.05 for row in results)}
    output = ROOT / "outputs/driving-investigation/history-estimator.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps({"synthetic": synthetic, "recorded_summary": summary, "recorded": results}, indent=2) + "\n")
    print(json.dumps({"synthetic": synthetic, "recorded_summary": summary}, indent=2))


if __name__ == "__main__":
    main()
