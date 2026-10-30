"""Audit recorded controls and candidate selection; works on partial captures."""
import argparse
import csv
import json
import math
from pathlib import Path


def audit(path):
    with (path / "000_verifai/ego.csv").open() as stream:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    by_time = {round(row["time_s"], 6): row for row in rows}
    frames = [json.loads(p.read_text()) for p in sorted(path.glob("frame_*.json"))]
    with (path / "000_verifai/predictions.jsonl").open() as stream:
        controller_inputs = {round(row["time_s"], 6): row for row in map(json.loads, stream)}
    consumed = []
    differences, snapshot_errors, frame_errors, checks, missing, identities = [], [], [], {}, [], []
    prefix_checks, fresh_checks = {}, {}
    prefix_records = 0
    for frame in frames:
        replay = frame.get("prefix_replay")
        context = replay["actual_context"] if replay else frame["context"]
        prefix_records += bool(replay)
        prediction = controller_inputs.get(round(context["time_s"], 6))
        if prediction is not None:
            consumed.append(prediction["trajectory_ego"] == frame["trajectory_ego"])
        row = by_time.get(round(context["time_s"], 6))
        if row:
            if "applied_control" in context:
                delta = {key: context["applied_control"][key] - row[key]
                         for key in ("steer", "throttle", "brake")}
                if max(abs(value) for value in delta.values()) > 1e-5:
                    differences.append({"time_s": context["time_s"], "delta": delta})
            snapshot_errors.append(math.hypot(context["world_pose"]["x"] - row["x"],
                                               context["world_pose"]["y"] - row["y"]))
            if context["snapshot_frame"] != row["frame"]:
                frame_errors.append(context["time_s"])
        if "visual_signal_check_index" in frame:
            checks[frame["visual_signal_check_index"]] = frame["visual_signal"]
            (prefix_checks if replay else fresh_checks)[frame["visual_signal_check_index"]] = frame["visual_signal"]
        if not frame["trajectory_marker_present"]:
            missing.append(context["time_s"])
        if frame.get("candidate_trajectories") is not None and "selection" in frame:
            identities.append(frame["trajectory_ego"] == frame["candidate_trajectories"][frame["selection"]["chosen"]])
    summary_path = path / "000_verifai/summary.json"
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
    initial = rows[0]
    yaw = math.radians(initial["yaw"])
    lateral = [(row["x"] - initial["x"]) * math.sin(yaw) -
               (row["y"] - initial["y"]) * math.cos(yaw) for row in rows]
    forward = [(row["x"] - initial["x"]) * math.cos(yaw) +
               (row["y"] - initial["y"]) * math.sin(yaw) for row in rows]
    return {"capture": str(path), "simulation_seconds": rows[-1]["time_s"],
            "completed": summary.get("termination") in ("duration_reached", "route_exit_reached"),
            "route_exit_reached": (path / "route-exit-reached.json").exists(),
            "inference_records": len(frames), "replayed_prefix_records": prefix_records,
            "fresh_inference_records": len(frames) - prefix_records,
            "min_speed_mps": min(row["speed"] for row in rows),
            "final_speed_mps": rows[-1]["speed"], "max_abs_lateral_m": max(map(abs, lateral)),
            "forward_progress_m": forward[-1],
            "near_stop_onsets_s": [rows[i]["time_s"] for i in range(1, len(rows))
                                    if rows[i]["speed"] < .25 <= rows[i-1]["speed"]],
            "reported_control_mismatches": differences,
            "max_snapshot_vs_csv_position_error_m": max(snapshot_errors, default=0),
            "snapshot_vs_csv_frame_mismatch_times_s": frame_errors,
            "missing_selected_markers_times_s": missing,
            "selected_candidates_checked": len(identities),
            "selected_candidate_identity": all(identities) if identities else None,
            "controller_inputs_checked": len(consumed),
            "controller_input_identity": all(consumed) if consumed else None,
            "visual_check_counts": {color: list(checks.values()).count(color) for color in sorted(set(checks.values()))},
            "archived_prefix_visual_check_counts": {color: list(prefix_checks.values()).count(color) for color in sorted(set(prefix_checks.values()))},
            "fresh_visual_check_counts": {color: list(fresh_checks.values()).count(color) for color in sorted(set(fresh_checks.values()))}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = json.dumps(audit(args.capture), indent=2) + "\n"
    if args.output:
        args.output.write_text(result)
    print(result, end="")


if __name__ == "__main__":
    main()
