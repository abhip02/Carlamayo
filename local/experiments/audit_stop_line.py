"""Evaluate actual vehicle-body clearance from a supplied stop-line plane."""
import argparse
import csv
import json
import math
from pathlib import Path

import carla
import numpy as np


def audit(capture, bounds, stop_point, stop_yaw):
    rows = [{key: float(value) for key, value in row.items()}
            for row in csv.DictReader((capture / "000_verifai/ego.csv").open())]
    bbox = carla.BoundingBox(carla.Location(**bounds["location"]),
                             carla.Vector3D(**bounds["extent"]))
    bbox.rotation = carla.Rotation(**bounds["rotation"])
    normal = np.array([math.cos(math.radians(stop_yaw)), math.sin(math.radians(stop_yaw)), 0.])
    clearances = []
    for row in rows:
        transform = carla.Transform(
            carla.Location(**{key: row[key] for key in ("x", "y", "z")}),
            carla.Rotation(**{key: row[key] for key in ("yaw", "pitch", "roll")}))
        vertices = np.array([[v.x, v.y, v.z] for v in bbox.get_world_vertices(transform)])
        clearances.append(float(np.min((np.asarray(stop_point) - vertices) @ normal)))
    return {
        "capture": str(capture), "duration_s": rows[-1]["time_s"],
        "minimum_front_clearance_m": min(clearances),
        "final_front_clearance_m": clearances[-1],
        "stop_line_crossed": min(clearances) < 0,
        "final_speed_mps": rows[-1]["speed"],
        "near_stop_onsets_s": [rows[i]["time_s"] for i in range(1, len(rows))
                               if rows[i]["speed"] < .25 <= rows[i-1]["speed"]],
        "restart_above_025_times_s": [rows[i]["time_s"] for i in range(1, len(rows))
                                      if rows[i]["speed"] >= .25 > rows[i-1]["speed"]],
        "clearance_by_time": [[r["time_s"], c] for r, c in zip(rows, clearances)],
        "stop_plane": {"point": stop_point, "yaw_deg": stop_yaw},
        "note": "Evaluation only; no stop-line or bounding-box data enters inference.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--bounds", type=Path, required=True)
    parser.add_argument("--stop-point", nargs=3, type=float, required=True)
    parser.add_argument("--stop-yaw", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.capture, json.loads(args.bounds.read_text()), args.stop_point, args.stop_yaw)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "clearance_by_time"}, indent=2))


if __name__ == "__main__":
    main()
