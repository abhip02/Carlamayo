"""Score frozen replay variants using the same recorded previous plan."""
import argparse
import json
from pathlib import Path

import numpy as np

from local.experiments.candidate_selection import select_candidate
from module.trajectory_follower import planned_speeds


def compare(capture, replay):
    results = []
    for path in sorted(replay.glob("frame_*.json")):
        record = json.loads(path.read_text())
        if "candidate_trajectories" not in record:
            results.append({"label": record["label"], "status": record.get("status", "not_trajectory")})
            continue
        index = record["context"]["index"]
        original = json.loads((capture / f"frame_{index:03d}.json").read_text())
        previous = None
        if index:
            prev = json.loads((capture / f"frame_{index - 1:03d}.json").read_text())
            _, previous, _ = select_candidate(
                np.asarray([prev["trajectory_ego"]]), prev["context"])
        candidates = np.asarray(record["candidate_trajectories"] or [record["trajectory_ego"]])
        prefer_stop = original.get("visual_signal") in ("red", "yellow")
        chosen, _, costs = select_candidate(candidates, record["context"], previous,
                                            prefer_stop=prefer_stop)
        original_candidates = np.asarray(original["candidate_trajectories"] or [original["trajectory_ego"]])
        lengths = [float(np.linalg.norm(np.diff(np.vstack((np.zeros(3), c)), axis=0), axis=1).sum())
                   for c in candidates]
        results.append({
            "label": record["label"], "options": record["options"],
            "selected": chosen, "prefer_stop": prefer_stop, **costs,
            "target_speeds_mps": [float(planned_speeds(c)[10:15].mean()) for c in candidates],
            "selected_target_mps": float(planned_speeds(candidates[chosen])[10:15].mean()),
            "path_lengths_m": lengths, "selected_path_length_m": lengths[chosen],
            "endpoints_xy": candidates[:, -1, :2].tolist(),
            "candidate_markers": record["candidate_markers"],
            "same_shape_as_capture": candidates.shape == original_candidates.shape,
            "max_candidate_difference_from_capture_m": (
                float(np.max(np.abs(candidates - original_candidates)))
                if candidates.shape == original_candidates.shape else None),
            "reasoning": record.get("candidate_texts"),
        })
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = compare(args.capture, args.replay)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    for row in result:
        print(json.dumps({k: row[k] for k in (
            "label", "selected_target_mps", "selected_path_length_m",
            "route_costs", "max_candidate_difference_from_capture_m") if k in row}))


if __name__ == "__main__":
    main()
