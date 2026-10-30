"""Audit recorded CARLA vehicle-box corners against the selected lane corridor."""
import argparse
import csv
import json
from pathlib import Path

import carla
import numpy as np

from local.experiments.audit_turn import project
from local.experiments.validated_selection import pose_transform


def audit(capture, bounds=None):
    first = json.loads(next(iter(sorted(capture.glob("frame_*.json")))).read_text())
    bounds = bounds or first["context"]["vehicle_bounds"]
    name = first["selection"]["route"]
    route = json.loads((capture / "route-reference.json").read_text())["routes"][name]
    box = carla.BoundingBox(carla.Location(**bounds["location"]), carla.Vector3D(**bounds["extent"]))
    box.rotation = carla.Rotation(**bounds["rotation"])
    frames = []
    with (capture / "000_verifai/ego.csv").open() as stream:
        for row in csv.DictReader(stream):
            row = {key: float(value) for key, value in row.items()}
            vertices = box.get_world_vertices(pose_transform(row))
            points = np.array([[p.x, p.y] for p in vertices])
            distance, _, _, width, _ = project(points, route)
            frames.append({"time_s": row["time_s"],
                           "max_corner_lane_distance_m": float(distance.max()),
                           "max_corner_overhang_m": float((distance - width).max())})
    return {"capture": str(capture), "route": name, "geometry": bounds,
            "reference": "Recorded bounding-box corners; not a swept-volume or obstacle audit.",
            "max_corner_overhang_m": max(f["max_corner_overhang_m"] for f in frames),
            "corner_outside_times_s": [f["time_s"] for f in frames if f["max_corner_overhang_m"] > 0],
            "frames": frames}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--bounds", type=Path, help="Geometry for older captures without recorded dimensions")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.capture, json.loads(args.bounds.read_text()) if args.bounds else None)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: value for key, value in result.items() if key != "frames"}, indent=2))


if __name__ == "__main__":
    main()
