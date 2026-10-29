"""Measure a recorded rollout against map lane paths, without controlling it."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def project(points, route):
    """Nearest points on the lane polyline; arc distance is measured from its start."""
    reference = np.array([[p["x"], p["y"]] for p in route["points"]])
    segments = np.diff(reference, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    cumulative = np.r_[0., np.cumsum(lengths)]
    valid = lengths > 1e-7
    indices = np.flatnonzero(valid)
    starts, vectors, sizes = reference[:-1][valid], segments[valid], lengths[valid]
    fraction = np.clip(np.einsum("nsi,si->ns", points[:, None] - starts, vectors) / sizes**2, 0, 1)
    projections = starts + fraction[..., None] * vectors
    distance = np.linalg.norm(points[:, None] - projections, axis=-1)
    chosen = distance.argmin(axis=1)
    row = np.arange(len(points))
    distance = distance[row, chosen]
    arc = cumulative[indices[chosen]] + fraction[row, chosen] * sizes[chosen]
    headings = np.degrees(np.arctan2(vectors[chosen, 1], vectors[chosen, 0]))
    half_width = np.array([route["points"][i]["lane_width"] / 2 for i in indices[chosen]])
    return distance, arc, headings, half_width, cumulative[route["exit_index"]]


def audit(capture, routes):
    rows = [{key: float(value) for key, value in row.items()}
            for row in csv.DictReader((capture / "000_verifai/ego.csv").open())]
    positions = np.array([[r["x"], r["y"]] for r in rows])
    yaws = np.array([r["yaw"] for r in rows])
    speed = np.array([r["speed"] for r in rows])
    times = np.array([r["time_s"] for r in rows])
    result = {"capture": str(capture), "duration_s": float(times[-1]),
              "instruction": json.loads((capture / "config.json").read_text())["instruction"],
              "reference": "Vehicle actor center; lane-center distance does not prove full-body lane containment.",
              "near_stop_onsets_s": times[1:][(speed[1:] < .25) & (speed[:-1] >= .25)].tolist(),
              "min_speed_mps": float(speed.min()), "final_speed_mps": float(speed[-1]),
              "heading_change_left_deg": float(-np.degrees(np.unwrap(np.radians(yaws))[-1] - np.radians(yaws[0]))),
              "wrapped_heading_change_left_deg": float((yaws[0] - yaws[-1] + 180) % 360 - 180),
              "routes": {}}
    for name, route in routes["routes"].items():
        distance, arc, heading, half_width, exit_arc = project(positions, route)
        heading_error = np.abs((yaws - heading + 180) % 360 - 180)
        end = route["exit"]
        forward = np.array([math.cos(math.radians(end["yaw"])), math.sin(math.radians(end["yaw"]))])
        beyond_exit = (positions - [end["x"], end["y"]]) @ forward >= 0
        reached = np.flatnonzero(beyond_exit & (arc >= exit_arc - .05) &
                                 (distance <= half_width) & (heading_error <= 20))
        stop = int(reached[0]) + 1 if len(reached) else len(rows)
        result["routes"][name] = {
            "reached_exit_time_s": float(times[reached[0]]) if len(reached) else None,
            "max_centerline_distance_until_exit_m": float(distance[:stop].max()),
            "rms_centerline_distance_until_exit_m": float(np.sqrt(np.mean(distance[:stop]**2))),
            "center_outside_lane_halfwidth_until_exit_s": times[:stop][distance[:stop] > half_width[:stop]].tolist(),
            "final_centerline_distance_m": float(distance[-1]),
            "final_route_heading_error_deg": float(heading_error[-1]),
            "final_arc_before_exit_m": float(exit_arc - arc[-1]),
            "trace": [{"time_s": float(t), "distance_m": float(d), "arc_m": float(s),
                       "heading_error_deg": float(h)} for t, d, s, h in zip(times, distance, arc, heading_error)],
        }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--routes", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.capture, json.loads(args.routes.read_text()))
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    result["routes"] = {name: {k: v for k, v in r.items() if k not in (
        "trace", "center_outside_lane_halfwidth_until_exit_s")}
        for name, r in result["routes"].items()}
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
