"""Plot actual rollout paths against the target junction's left/straight lanes."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", nargs="+", type=Path, required=True)
    parser.add_argument("--routes", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    routes = json.loads(args.routes.read_text())["routes"]
    entry = routes["straight"]["entry"]
    origin = np.array([entry["x"], entry["y"]])
    yaw = np.radians(entry["yaw"])
    basis = np.array([[-np.sin(yaw), np.cos(yaw)], [np.cos(yaw), np.sin(yaw)]])
    fig, axes = plt.subplots(1, len(args.runs), figsize=(6 * len(args.runs), 6),
                             sharex=True, sharey=True, squeeze=False)
    for ax, path in zip(axes[0], args.runs):
        for name, route in routes.items():
            xy = (np.array([[p["x"], p["y"]] for p in route["points"]]) - origin) @ basis.T
            ax.plot(xy[:, 0], xy[:, 1], "--", color="black" if name == "left" else "gray",
                    linewidth=1, label=f"Map {name} lane")
        rows = [{k: float(v) for k, v in r.items()}
                for r in csv.DictReader((path / "000_verifai/ego.csv").open())]
        xy = (np.array([[r["x"], r["y"]] for r in rows]) - origin) @ basis.T
        ax.plot(xy[:, 0], xy[:, 1], color="tab:blue", linewidth=2, label="Actual vehicle center")
        ax.scatter(*xy[0], color="green", s=30, label="Start")
        ax.scatter(*xy[-1], color="tab:blue", marker="x", s=50, label=f"End {rows[-1]['time_s']:.1f} s")
        stopped = np.array([r["speed"] < .25 for r in rows])
        ax.scatter(xy[stopped, 0], xy[stopped, 1], color="red", s=10, label="Speed below 0.25 m/s")
        ax.set_title(path.name.replace("capture-", ""), fontsize=10)
        ax.set_xlabel("Right of approach centerline (m)")
        ax.set_ylabel("Forward from junction entry (m)")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=.3)
        ax.legend(fontsize=8)
    fig.suptitle("Recorded vehicle paths against map lane centerlines")
    fig.tight_layout()
    fig.savefig(args.output, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
