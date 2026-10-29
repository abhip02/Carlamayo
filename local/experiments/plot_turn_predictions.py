"""Overlay saved predictions and map lanes without running inference or simulation."""
import argparse
import csv
import json
from pathlib import Path

import carla
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from local.experiments.audit_turn import project
from local.overhead import rear_axle_xyz
from module.trajectory_follower import trajectory_to_world


def transform(pose):
    return carla.Transform(carla.Location(**{k: pose[k] for k in ("x", "y", "z")}),
                           carla.Rotation(**{k: pose[k] for k in ("pitch", "yaw", "roll")}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--routes", type=Path, required=True)
    parser.add_argument("--frames", type=int, nargs="+", default=[45, 65, 85, 95])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    routes = json.loads(args.routes.read_text())["routes"]
    entry = routes["straight"]["entry"]
    origin = np.array([entry["x"], entry["y"]])
    yaw = np.radians(entry["yaw"])
    basis = np.array([[-np.sin(yaw), np.cos(yaw)], [np.cos(yaw), np.sin(yaw)]])

    def view(points):
        return (np.asarray(points)[:, :2] - origin) @ basis.T

    rows = [{k: float(v) for k, v in row.items()}
            for row in csv.DictReader((args.capture / "000_verifai/ego.csv").open())]
    actual = np.array([rear_axle_xyz(transform(row)) for row in rows])
    fig, axes = plt.subplots(1, len(args.frames), figsize=(5 * len(args.frames), 6),
                             sharex=True, sharey=True, squeeze=False)
    metrics = []
    for ax, index in zip(axes[0], args.frames):
        record = json.loads((args.capture / f"frame_{index:03d}.json").read_text())
        context = record["context"]
        tf = transform(context["world_pose"])
        plan = np.vstack((rear_axle_xyz(tf), trajectory_to_world(tf, record["trajectory_ego"])))
        distance = project(plan[:16, :2], routes["left"])[0]
        metrics.append({"frame": index, "time_s": context["time_s"],
                        "current_rear_axle_distance_from_left_lane_m": float(distance[0]),
                        "max_predicted_1p5s_rear_axle_distance_from_left_lane_m": float(distance.max()),
                        "note": "Prediction geometry only; subsequent replanning prevents attributing tracking error to the controller alone."})
        for name, route in routes.items():
            xy = view([[p["x"], p["y"]] for p in route["points"]])
            ax.plot(*xy.T, "--", color="black" if name == "left" else "gray",
                    linewidth=1, label=f"Map {name} lane")
        xy = view(actual[np.array([r["time_s"] <= context["time_s"] + 1e-6 for r in rows])])
        ax.plot(*xy.T, color="tab:blue", linewidth=2, label="Actual rear axle so far")
        xy = view(plan)
        ax.plot(*xy.T, color="tab:orange", label="Unmodified model plan (6.4 s)")
        ax.plot(*xy[:16].T, color="red", linewidth=2, label="First 1.5 s of plan")
        ax.scatter(*xy[0], color="tab:blue", s=20)
        ax.set_title(f"t={context['time_s']:.1f} s; target={record['raw_target_speed_mps']:.2f} m/s")
        ax.set_xlim(-45, 8)
        ax.set_ylim(-24, 42)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("Right of approach centerline (m)")
        ax.grid(alpha=.3)
    axes[0, 0].set_ylabel("Forward from junction entry (m)")
    axes[0, -1].legend(fontsize=7)
    fig.suptitle(args.capture.name + " — map is evaluation only; all paths use the rear axle")
    fig.tight_layout()
    fig.savefig(args.output, dpi=150)
    plt.close(fig)
    args.output.with_suffix(".json").write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
