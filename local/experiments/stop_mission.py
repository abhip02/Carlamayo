"""Measure a red-light stop using the recorded vehicle footprint and speed."""
import math
import carla
import numpy as np
from local.experiments.validated_selection import pose_transform


def audit_stop(rows, bounds, mission):
    line = mission['stop_line']
    direction = np.array([math.cos(math.radians(line['yaw'])), math.sin(math.radians(line['yaw']))])
    box = carla.BoundingBox(carla.Location(**bounds['location']), carla.Vector3D(**bounds['extent']))
    box.rotation = carla.Rotation(**bounds['rotation'])
    gaps = []
    for row in rows:
        vertices = box.get_world_vertices(pose_transform(row))
        points = np.array([[p.x, p.y] for p in vertices])
        gaps.append(float(-np.max((points - [line['x'], line['y']]) @ direction)))
    hold_s = mission.get('hold_s', 2.)
    tail = [i for i, r in enumerate(rows) if r['time_s'] >= rows[-1]['time_s']-hold_s-1e-6]
    held = (rows[-1]['time_s'] >= hold_s and all(rows[i]['speed'] <= .2 for i in tail))
    close = all(0 <= gaps[i] <= mission.get('maximum_stop_gap_m', 8.) for i in tail)
    return dict(kind='red_light_stop', hold_s=hold_s, held_stop=held,
                stopped_near_line=close, minimum_front_gap_m=min(gaps), final_front_gap_m=gaps[-1],
                maximum_hold_speed_mps=max(rows[i]['speed'] for i in tail),
                passed=bool(held and close and min(gaps) >= 0.))
