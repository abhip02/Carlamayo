"""Compare frozen-input navigation variants and plot their unmodified predictions."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def summarize(replay, output, baseline_name="straight", plot_variants=None):
    records = [json.loads(p.read_text()) for p in sorted(replay.glob("frame_*.json"))]
    indices = sorted({r["context"]["index"] for r in records})
    figure, axes = plt.subplots(1, len(indices), figsize=(5 * len(indices), 6), squeeze=False)
    results = []
    names = plot_variants or ["straight", "left", "no_navigation", "left_completed"]
    colors = dict(zip(names, plt.get_cmap("tab10").colors))
    for ax, index in zip(axes[0], indices):
        group = [r for r in records if r["context"]["index"] == index]
        baseline = next(r for r in group if r["label"] == f"frame_{index:03d}_{baseline_name}")
        reference = np.asarray(baseline["trajectory_ego"])
        for record in group:
            name = record["label"].split("_", 2)[2]
            trajectory = np.asarray(record["trajectory_ego"])
            candidates = np.asarray(record["candidate_trajectories"] or [record["trajectory_ego"]])
            directions = candidates[:, -1, :2] - candidates[:, -4, :2]
            results.append({
                "label": record["label"], "instruction": record["instruction"],
                "options": record["options"], "baseline_label": baseline["label"],
                "assistant_prefix": record["assistant_prefix"], "generated_reasoning": record["reasoning"],
                "reasoning_supplied": "completed_reasoning" in record["options"],
                "target_speed_mps": record["raw_target_speed_mps"],
                "endpoints_forward_left_m": candidates[:, -1, :2].tolist(),
                "end_motion_direction_left_deg": np.degrees(np.arctan2(directions[:, 1], directions[:, 0])).tolist(),
                "mean_waypoint_distance_from_baseline_m": float(np.linalg.norm(trajectory - reference, axis=1).mean()),
                "max_coordinate_difference_from_baseline_m": float(np.max(np.abs(trajectory - reference))),
                "candidate_markers": record["candidate_markers"],
            })
            if name in colors:
                ax.plot(-trajectory[:, 1], trajectory[:, 0], label=name, color=colors[name])
                ax.scatter(-trajectory[-1, 1], trajectory[-1, 0], color=colors[name], s=15)
        ax.set_title(f"Saved frame {index:03d}\nmeasured speed {baseline['context']['measured_speed_mps']:.2f} m/s")
        ax.set_xlabel("Right of ego (m)")
        ax.set_ylabel("Forward of ego (m)")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=.3)
        ax.legend(fontsize=8)
    figure.suptitle("Same camera/pose inputs and fixed motion noise; only stated variant changes")
    figure.tight_layout()
    figure.savefig(output / "instruction-paths.png", dpi=160)
    plt.close(figure)
    (output / "instruction-comparison.json").write_text(json.dumps(results, indent=2, allow_nan=False) + "\n")
    print(json.dumps(results, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--baseline", default="straight")
    parser.add_argument("--plot-variants", nargs="+")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    summarize(args.replay, args.output, args.baseline, args.plot_variants)


if __name__ == "__main__":
    main()
