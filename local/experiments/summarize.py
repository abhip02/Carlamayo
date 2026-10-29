"""Compare captured rollouts using measured states and raw inference outputs."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def read_run(path, horizon=None):
    summary = json.loads((path / "000_verifai/summary.json").read_text())
    with (path / "000_verifai/ego.csv").open() as stream:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    frames = [json.loads(p.read_text()) for p in sorted(path.glob("frame_*.json"))]
    if horizon is not None:
        rows = [row for row in rows if row["time_s"] <= horizon + 1e-8]
        frames = [frame for frame in frames if frame["context"]["time_s"] < rows[-1]["time_s"] - 1e-8]
    time = np.array([r["time_s"] for r in rows])
    speed = np.array([r["speed"] for r in rows])
    xy = np.array([[r["x"], r["y"]] for r in rows])
    yaw = np.deg2rad(rows[0]["yaw"])
    displacement = xy - xy[0]
    forward = displacement @ np.array([np.cos(yaw), np.sin(yaw)])
    left = displacement @ np.array([np.sin(yaw), -np.cos(yaw)])
    near_stop = speed < .25
    stop_onset = np.flatnonzero(near_stop[1:] & ~near_stop[:-1]) + 1
    metrics = {
        "duration_s": float(time[-1]),
        "full_run_duration_s": summary["simulation_seconds"],
        "termination": summary["termination"],
        "actual_initial_state": summary["actual_handover"],
        "actual_final_state": {key: rows[-1][key] for key in summary["final_state"]},
        "mean_speed_mps": float(np.trapezoid(speed, time) / time[-1]),
        "min_speed_mps": float(speed.min()),
        "near_stop_onsets_s": time[stop_onset].tolist(),
        "near_stop_threshold_mps": .25,
        "path_length_m": float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()),
        "forward_progress_m": float(forward[-1]),
        "final_lateral_left_m": float(left[-1]),
        "max_abs_lateral_m": float(np.abs(left).max()),
        "final_heading_change_deg": (rows[-1]["yaw"] - rows[0]["yaw"] + 180) % 360 - 180,
        "brake_ticks": sum(row["brake"] > 0 for row in rows[1:]),
        "inference_calls": len(frames),
        "missing_trajectory_markers": sum(not f["trajectory_marker_present"] for f in frames),
        "raw_target_below_025_times_s": [f["context"]["time_s"] for f in frames
                                        if f["raw_target_speed_mps"] < .25],
        "plans_with_reverse_velocity": sum(f["reverse_velocity_steps"] > 0 for f in frames),
        "negative_initial_estimates": sum(f["decoder_estimated_speed_mps"] < -.05 for f in frames),
    }
    return metrics, (time, speed, forward, left, frames)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", nargs="+", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--horizon", type=float, help="Compare only the first N simulation seconds")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results, series = {}, {}
    for run in args.runs:
        results[run.name], series[run.name] = read_run(run, args.horizon)
    first = next(iter(results.values()))["actual_initial_state"]
    for result in results.values():
        initial = result["actual_initial_state"]
        result["initial_difference_from_first"] = {
            "position_m": float(np.linalg.norm([initial[k] - first[k] for k in ("x", "y", "z")])),
            "speed_mps": initial["speed"] - first["speed"],
            "yaw_deg": (initial["yaw"] - first["yaw"] + 180) % 360 - 180,
        }
    (args.output / "metrics.json").write_text(json.dumps(results, indent=2) + "\n")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for name, (time, speed, forward, left, frames) in series.items():
        label = name.replace("capture-", "")
        frame_time = [f["context"]["time_s"] for f in frames]
        axes[0, 0].plot(time, speed, label=label)
        axes[0, 1].plot(frame_time, [f["raw_target_speed_mps"] for f in frames], label=label)
        axes[1, 0].plot(left, forward, label=label)
        axes[1, 1].plot(frame_time, [f["reverse_velocity_steps"] for f in frames], label=label)
    axes[0, 0].set(title="Measured speed", xlabel="Simulation time (s)", ylabel="m/s")
    axes[0, 1].set(title="Raw trajectory speed, 1.0–1.5 s ahead", xlabel="Simulation time (s)", ylabel="m/s")
    axes[1, 0].set(title="Actual path relative to starting heading", xlabel="Left (m)", ylabel="Forward (m)")
    axes[1, 0].set_aspect("equal", adjustable="datalim")
    axes[1, 1].set(title="Predicted steps with signed velocity < −0.05 m/s", xlabel="Simulation time (s)", ylabel="Steps out of 64")
    for ax in axes.flat:
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    axes[0, 0].axhline(.25, color="gray", linestyle=":", alpha=.6)
    axes[0, 1].axhline(.25, color="gray", linestyle=":", alpha=.6)
    fig.suptitle("Isolated inference experiments — original CarlaMayo controller", fontsize=14)
    fig.savefig(args.output / "comparison.png", dpi=160)
    plt.close(fig)
    print(json.dumps({name: {k: v for k, v in result.items() if not k.startswith("actual_")}
                      for name, result in results.items()}, indent=2))


if __name__ == "__main__":
    main()
