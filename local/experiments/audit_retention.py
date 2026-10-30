"""Verify every executed plan's native source, fixed world anchor and real age."""
import argparse
import json
from pathlib import Path

import numpy as np

from local.experiments.validated_selection import pose_transform
from module.trajectory_follower import planned_speeds, trajectory_to_world


def audit(capture):
    rows = [json.loads(line) for line in (capture / "controller-inputs.jsonl").read_text().splitlines()]
    mismatches, reused = [], []
    for row in rows:
        source = json.loads((capture / (row["source_label"] + ".json")).read_text())
        current = json.loads((capture / f"frame_{round(row['time_s'] / .1):03d}.json").read_text())
        path = np.asarray(source["trajectory_ego"])
        candidates = source.get("candidate_trajectories") or [source["trajectory_ego"]]
        expected_world = trajectory_to_world(pose_transform(source["context"]["world_pose"]), path)
        expected_age = current["context"]["time_s"] - source["context"]["time_s"]
        checks = {
            "native_source": source["trajectory_ego"] == candidates[source["selection"]["chosen"]],
            "source_is_fresh": "retention" not in source,
            "unchanged_native_array": current["trajectory_ego"] == source["trajectory_ego"],
            "unchanged_world_path": np.array_equal(expected_world, np.asarray(row["world_trajectory"])),
            "unchanged_speed_array": np.array_equal(planned_speeds(path), np.asarray(row["speeds_mps"])),
            "original_source_context": row["source_context"] == source["context"],
            "real_age": abs(row["age_s"] - expected_age) < 1e-8,
            "bounded_age": -1e-8 <= row["age_s"] <= .5 + 1e-8,
        }
        if row["age_s"] > 1e-8:
            detail = current.get("retention", {})
            checks["explicit_retention"] = (detail.get("source_label") == row["source_label"] and
                                              abs(detail.get("age_s", -100) - row["age_s"]) < 1e-8)
            checks["rejection_preserved"] = (capture / "replan-rejections" / (current["label"] + ".json")).exists()
            reused.append({"time_s": row["time_s"], "source": row["source_label"], "age_s": row["age_s"]})
        if not all(checks.values()):
            mismatches.append({"time_s": row["time_s"], "checks": checks})
    return {"capture": str(capture), "controller_steps_checked": len(rows),
            "retained_steps": reused, "mismatches": mismatches,
            "all_sources_anchors_speeds_and_ages_match": bool(rows) and not mismatches}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.capture)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
