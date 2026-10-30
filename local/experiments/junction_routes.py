"""Export map lane paths for audits or the opt-in native-sample validator."""
import argparse
import json
import math
from pathlib import Path

import carla
import yaml


def angle_difference(a, b):
    return (a - b + 180) % 360 - 180


def point(waypoint):
    t = waypoint.transform
    return {"x": t.location.x, "y": t.location.y, "z": t.location.z,
            "yaw": t.rotation.yaw, "lane_width": waypoint.lane_width,
            "road_id": waypoint.road_id, "lane_id": waypoint.lane_id,
            "s": waypoint.s, "is_junction": waypoint.is_junction}


def export(world_map, entry):
    yaw = math.radians(entry["yaw"])
    location = carla.Location(x=entry["x"], y=entry["y"], z=entry["z"])
    inside = carla.Location(x=entry["x"] + 5 * math.cos(yaw),
                            y=entry["y"] + 5 * math.sin(yaw), z=entry["z"])
    junction = world_map.get_waypoint(inside).get_junction()
    if junction is None or junction.id != entry["junction_id"]:
        raise ValueError("The configured entry does not lead into the expected junction")
    routes = {}
    for start, end in junction.get_waypoints(carla.LaneType.Driving):
        if start.transform.location.distance(location) > .5:
            continue
        turn = angle_difference(end.transform.rotation.yaw, start.transform.rotation.yaw)
        name = "straight" if abs(turn) < 30 else "left" if turn < 0 else "right"
        prefix, current = [], start
        for _ in range(60):
            previous = current.previous(.5)
            if len(previous) != 1:
                break
            current = previous[0]
            prefix.append(current)
        connector, current = [start], start
        for _ in range(300):
            if current.transform.location.distance(end.transform.location) < .6:
                break
            choices = [w for w in current.next(.5)
                       if w.road_id == start.road_id and w.lane_id == start.lane_id]
            if len(choices) != 1:
                raise ValueError(f"Ambiguous connector on road {start.road_id}")
            current = choices[0]
            connector.append(current)
        else:
            raise ValueError("Connector traversal did not terminate")
        connector.append(end)
        suffix, current = [], end
        for _ in range(30):
            following = current.next(.5)
            if len(following) != 1:
                break
            current = following[0]
            suffix.append(current)
        if name in routes:
            raise ValueError(f"Multiple {name} routes require an explicit lane choice")
        points = list(reversed(prefix)) + connector + suffix
        routes[name] = {"entry": point(start), "exit": point(end),
                        "entry_index": len(prefix), "exit_index": len(prefix) + len(connector) - 1,
                        "heading_change_deg": turn, "points": [point(w) for w in points]}
    if not {"left", "straight"} <= routes.keys():
        raise ValueError("Both left and straight connections are required for this comparison")
    return {"map": world_map.name, "junction_id": junction.id, "routes": routes,
            "note": "Map lane references; the caller determines whether these are used for audit or sample selection."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(20)
    world_map = client.get_world().get_map()
    if not world_map.name.endswith(config["map"]):
        raise ValueError("The running map differs from the test configuration")
    result = export(world_map, config["junction_entry"])
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({name: {"entry": route["entry"], "exit": route["exit"],
                             "points": len(route["points"])}
                      for name, route in result["routes"].items()}, indent=2))


if __name__ == "__main__":
    main()
