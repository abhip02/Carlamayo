"""Compare quality retries with the first admitted pool on each identical input."""
import argparse
import json
from pathlib import Path


def audit(capture):
    records, mismatches = [], []
    for path in sorted(capture.glob("frame_*.json")):
        frame = json.loads(path.read_text())
        if "retention" in frame:
            continue
        selection = frame["selection"]
        attempts = selection["attempts"]
        first_index, first = next((i, a) for i, a in enumerate(attempts) if a["chosen"] is not None)
        first_metric = first["metrics"][first["chosen"]]
        chosen_attempt = next(a for a in attempts if a["proposal"] == selection["proposal"])
        metric = chosen_attempt["metrics"][selection["chosen"]]
        proposal = json.loads((capture / "proposals" / (selection["proposal"] + ".json")).read_text())
        identity = (frame["trajectory_ego"] == proposal["candidate_trajectories"][selection["chosen"]] and
                    frame["generated_token_ids"] == proposal["candidate_generated_tokens"][selection["chosen"]])
        if not identity:
            mismatches.append(frame["label"])
        preference = selection.get("speed_quality_preference")
        records.append({
            "label": frame["label"], "time_s": frame["context"]["time_s"],
            "first_admitted_proposal": first["proposal"], "selected_proposal": selection["proposal"],
            "selection_changed": first["proposal"] != selection["proposal"],
            "extra_pools_after_first_admission": len(attempts) - first_index - 1,
            "first_target_mps": first_metric["target_speed_mps"],
            "selected_target_mps": metric["target_speed_mps"],
            "target_reduction_mps": first_metric["target_speed_mps"] - metric["target_speed_mps"],
            "first_corner_clearance_m": -first_metric["max_corner_overhang_m"],
            "selected_corner_clearance_m": -metric["max_corner_overhang_m"],
            "preference": preference, "source_array_and_tokens_match": identity,
        })
    return {"capture": str(capture), "fresh_steps_checked": len(records),
            "steps_with_extra_pools_after_admission": sum(r["extra_pools_after_first_admission"] > 0 for r in records),
            "extra_pools_after_admission": sum(r["extra_pools_after_first_admission"] for r in records),
            "selections_changed_by_quality_retries": sum(r["selection_changed"] for r in records),
            "steps_with_unmet_green_preference": sum(bool(r["preference"] and r["preference"]["applied"] and
                                                           not r["preference"]["met"]) for r in records),
            "source_array_or_token_mismatches": mismatches, "records": records,
            "note": "Same-input first-admission comparisons; not counterfactual complete rollouts."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.capture)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "records"}, indent=2))


if __name__ == "__main__":
    main()
