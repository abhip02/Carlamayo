"""Verify every executed plan's native source, fixed world anchor and real age."""
import argparse
import json
from pathlib import Path

import numpy as np

from local.experiments.validated_selection import pose_transform, selection_rank
from local.experiments.exit_cache import unknown_exit_evidence
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
        if "admitted_reserve" in source:
            provenance = source["admitted_reserve"]
            fresh = json.loads((capture / (provenance["fresh_record"] + ".json")).read_text())
            proposal_label = source["selection"]["proposal"]
            proposal = json.loads((capture / "proposals" / (proposal_label + ".json")).read_text())
            original_attempt = next(a for a in fresh["selection"]["attempts"] if a["proposal"] == proposal_label)
            candidate = source["selection"]["chosen"]
            original_metric = original_attempt["metrics"][candidate]
            checks["reserve_was_originally_admitted"] = (
                original_metric["valid"] and original_metric == provenance["original_metric"])
            checks["reserve_native_provenance"] = (
                source["trajectory_ego"] == proposal["candidate_trajectories"][candidate] and
                source["generated_token_ids"] == proposal["candidate_generated_tokens"][candidate] and
                source["context"] == proposal["context"] == fresh["context"])
        if row["age_s"] > 1e-8:
            detail = current.get("retention", {})
            checks["explicit_retention"] = (detail.get("source_label") == row["source_label"] and
                                              abs(detail.get("age_s", -100) - row["age_s"]) < 1e-8)
            selected_metric = detail["metrics"][detail["chosen_cache_index"]]
            checks["signal_eligibility_logged"] = (
                source["selection"]["signal"] == current["selection"]["signal"] == "green" or
                "signal_reuse" in selected_metric)
            if "signal_reuse" in selected_metric:
                options = json.loads((capture / "experiment.json").read_text())["options"]
                routes = json.loads((capture / "route-reference.json").read_text())["routes"]
                evidence = unknown_exit_evidence(source, current["context"],
                    routes[current["selection"]["route"]], current["visual_signal"],
                    current["navigation_phase"], options)
                checks["unknown_exit_eligibility_matches"] = evidence == selected_metric["signal_reuse"]
                checks["unknown_signal_not_relabelled"] = (
                    source["selection"]["signal"] == selected_metric["signal_reuse"].get("source_signal") and
                    current["visual_signal"] == current["selection"]["signal"] ==
                    selected_metric["signal_reuse"].get("current_signal") == "unknown")
            reason = detail.get("reason", "fresh_sampling_exhausted")
            if reason == "cached_plan_ranked_better":
                fresh_path = capture / (detail["fresh_candidate_source"] + ".json")
                fresh = json.loads(fresh_path.read_text())
                proposal = json.loads((capture / "proposals" / (fresh["selection"]["proposal"] + ".json")).read_text())
                checks["fresh_candidate_preserved"] = (
                    "retention" not in fresh and fresh["context"] == current["context"] and
                    fresh["trajectory_ego"] == proposal["candidate_trajectories"][fresh["selection"]["chosen"]] and
                    fresh["generated_token_ids"] == proposal["candidate_generated_tokens"][fresh["selection"]["chosen"]])
                selected = fresh["selection"]
                attempt = next(a for a in selected["attempts"] if a["proposal"] == selected["proposal"])
                fresh_metric = attempt["metrics"][selected["chosen"]]
                checks["fresh_score_preserved"] = fresh_metric["selection_cost"] == detail["fresh_selection_cost"]
                checks["cached_rank_is_better"] = (selection_rank(detail["metrics"][detail["chosen_cache_index"]]) <
                                                   selection_rank(fresh_metric))
                if "fresh_selection_rank" in detail:
                    checks["fresh_rank_preserved"] = detail["fresh_selection_rank"] == list(selection_rank(fresh_metric))
            else:
                checks["rejection_preserved"] = (capture / "replan-rejections" / (current["label"] + ".json")).exists()
            reused.append({"time_s": row["time_s"], "source": row["source_label"], "age_s": row["age_s"],
                           "reason": reason})
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
