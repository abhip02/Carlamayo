"""Compare post-admission retries with the first admitted pool on the same input."""
import argparse
import json
from pathlib import Path


def upper_speed_annotations_match(frame, selected_metric):
    """Check the recorded upper-speed policy independently, including unknown."""
    selection = frame["selection"]
    threshold = frame["options"].get("validation_resample_above_target_mps")
    preference = selection.get("speed_quality_preference")
    attempts = selection["attempts"]
    if threshold is None:
        return preference is None and all("retry_for_speed_preference" not in a for a in attempts)
    signal = selection["signal"]
    applied = signal == "green" or (signal == "unknown" and
        frame["options"].get("validation_resample_fast_unknown", False))
    if not preference or not (
            preference["resample_above_target_mps"] == threshold and
            preference["applied"] == applied and
            preference["met"] == (selected_metric["target_speed_mps"] <= threshold) and
            len(attempts) <= frame["options"].get("validation_max_pools", 3)):
        return False
    admitted = []
    for attempt in attempts:
        admitted.extend(m for m in attempt["metrics"] if m["valid"])
        if attempt["chosen"] is None:
            if "retry_for_speed_preference" in attempt:
                return False
            continue
        best = min(admitted, key=lambda m: (
            not m.get("preferred_reserve_valid", True),
            not m.get("preferred_horizon_valid", True), m["selection_cost"]))
        if attempt.get("retry_for_speed_preference") != (applied and best["target_speed_mps"] > threshold):
            return False
    return True


def low_speed_annotations_match(frame, selected_metric):
    """Independently check the opt-in retry evidence against archived pools."""
    selection = frame["selection"]
    threshold = frame["options"].get("validation_resample_below_target_mps")
    preference = selection.get("low_speed_preference")
    attempts = selection["attempts"]
    if threshold is None:
        return preference is None and all("retry_for_low_speed_preference" not in a for a in attempts)
    applied = selection["signal"] in ("green", "unknown")
    if not preference or not (
            preference["resample_below_target_mps"] == threshold and
            preference["applied"] == applied and
            preference["met"] == (selected_metric["target_speed_mps"] >= threshold) and
            len(attempts) <= frame["options"].get("validation_max_pools", 3)):
        return False
    admitted = []
    for attempt in attempts:
        admitted.extend(m for m in attempt["metrics"] if m["valid"])
        if attempt["chosen"] is None:
            if "retry_for_low_speed_preference" in attempt:
                return False
            continue
        best = min(admitted, key=lambda m: (
            not m.get("preferred_reserve_valid", True),
            not m.get("preferred_horizon_valid", True), m["selection_cost"]))
        if attempt.get("retry_for_low_speed_preference") != (applied and best["target_speed_mps"] < threshold):
            return False
    return True


def audit(capture):
    records, mismatches = [], []
    fresh_sources = sorted(capture.glob("accepted_frame_*.json"))
    for path in fresh_sources or sorted(capture.glob("frame_*.json")):
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
        admitted = [(a["proposal"], m) for a in attempts for m in a["metrics"] if m["valid"]]
        # Reconstruct the preference ordering from archived metrics, separately
        # from the live selection helper. This also checks choices across pools.
        best_proposal, best_metric = min(admitted, key=lambda pair: (
            not pair[1].get("preferred_reserve_valid", True),
            not pair[1].get("preferred_horizon_valid", True), pair[1]["selection_cost"]))
        rank_matches = (selection["proposal"] == best_proposal and
                        selection["chosen"] == best_metric["candidate"])
        reserve = selection.get("reserve_preference")
        reserve_matches = True
        if reserve:
            threshold = reserve["preferred_clearance_m"]
            reserve_matches = reserve["met"] == metric["preferred_reserve_valid"]
            for attempt in attempts:
                for candidate in attempt["metrics"]:
                    forecast = candidate.get("tracking_forecast")
                    if forecast is not None:
                        expected = (candidate["max_corner_overhang_m"] <= -threshold and
                                    forecast["min_corner_clearance_m"] >= threshold)
                        reserve_matches &= candidate["preferred_reserve_valid"] == expected
        records.append({
            "label": frame["label"], "time_s": frame["context"]["time_s"],
            "signal": selection["signal"],
            "first_admitted_proposal": first["proposal"], "selected_proposal": selection["proposal"],
            "selection_changed": first["proposal"] != selection["proposal"],
            "extra_pools_after_first_admission": len(attempts) - first_index - 1,
            "first_target_mps": first_metric["target_speed_mps"],
            "selected_target_mps": metric["target_speed_mps"],
            "target_reduction_mps": first_metric["target_speed_mps"] - metric["target_speed_mps"],
            "first_corner_clearance_m": -first_metric["max_corner_overhang_m"],
            "selected_corner_clearance_m": -metric["max_corner_overhang_m"],
            "preference": preference, "source_array_and_tokens_match": identity,
            "best_admitted_sample_selected": rank_matches,
            "reserve_annotations_match": bool(reserve_matches),
            "upper_speed_annotations_match": upper_speed_annotations_match(frame, metric),
            "low_speed_annotations_match": low_speed_annotations_match(frame, metric),
            "low_speed_preference": selection.get("low_speed_preference"),
            "reserve_preference": reserve,
            "speed_retry_requests": sum(a.get("retry_for_speed_preference", False) for a in attempts[:-1]),
            "reserve_retry_requests": sum(a.get("retry_for_reserve_preference", False) for a in attempts[:-1]),
            "low_speed_retry_requests": sum(a.get("retry_for_low_speed_preference", False) for a in attempts[:-1]),
        })
    return {"capture": str(capture), "fresh_steps_checked": len(records),
            "steps_with_extra_pools_after_admission": sum(r["extra_pools_after_first_admission"] > 0 for r in records),
            "extra_pools_after_admission": sum(r["extra_pools_after_first_admission"] for r in records),
            "selections_changed_by_post_admission_retries": sum(r["selection_changed"] for r in records),
            "speed_retry_requests": sum(r["speed_retry_requests"] for r in records),
            "unknown_upper_speed_retry_requests": sum(r["speed_retry_requests"] for r in records if r["signal"] == "unknown"),
            "reserve_retry_requests": sum(r["reserve_retry_requests"] for r in records),
            "low_speed_retry_requests": sum(r["low_speed_retry_requests"] for r in records),
            "steps_with_unmet_low_speed_preference": sum(bool(r["low_speed_preference"] and
                r["low_speed_preference"]["applied"] and not r["low_speed_preference"]["met"]) for r in records),
            "steps_with_unmet_reserve_preference": sum(bool(r["reserve_preference"] and
                                                             not r["reserve_preference"]["met"]) for r in records),
            "steps_with_unmet_upper_speed_preference": sum(bool(r["preference"] and r["preference"]["applied"] and
                                                           not r["preference"]["met"]) for r in records),
            "steps_with_unmet_green_preference": sum(bool(r["signal"] == "green" and r["preference"] and
                r["preference"]["applied"] and not r["preference"]["met"]) for r in records),
            "source_array_or_token_mismatches": mismatches, "records": records,
            "selection_mismatches": [r["label"] for r in records if not (
                r["best_admitted_sample_selected"] and r["reserve_annotations_match"] and
                r["upper_speed_annotations_match"] and r["low_speed_annotations_match"])],
            "all_post_admission_checks_match": bool(records) and not mismatches and all(
                r["best_admitted_sample_selected"] and r["reserve_annotations_match"] and
                r["upper_speed_annotations_match"] and r["low_speed_annotations_match"] for r in records),
            "record_population": "Archived fresh proposals, including those superseded by cached plans" if fresh_sources else "Executed fresh proposals",
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
