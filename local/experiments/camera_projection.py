"""Experimental equidistant lens projection from wider CARLA pinhole renders.

Uses the existing rig's calibrated central focal lengths. This approximates
f-theta optics; it does not claim to reproduce the full calibration polynomial.
"""
from functools import lru_cache
import math

import cv2
import numpy as np
import torch

from module.adapters._rigs import FOUR_CAMERA_RIG


def wide_render_rig():
    return {name: {**camera, "fov": 35.0 if "tele" in name else 135.0}
            for name, camera in FOUR_CAMERA_RIG.items()}


@lru_cache(maxsize=32)
def projection_map(width, height, source_fov, target_fov, projection):
    source_focal = width / (2 * math.tan(math.radians(source_fov) / 2))
    target_focal = width / (2 * math.tan(math.radians(target_fov) / 2))
    y, x = np.mgrid[:height, :width].astype(np.float32)
    dx, dy = x - width / 2, y - height / 2
    if projection == "equidistant":
        radius = np.hypot(dx, dy)
        angle = radius / target_focal
        scale = np.full_like(radius, source_focal / target_focal)
        np.divide(source_focal * np.tan(angle), radius, out=scale, where=radius > 0)
    elif projection == "pinhole_from_wide":
        scale = source_focal / target_focal
    else:
        raise ValueError(f"Unknown projection: {projection}")
    return (width / 2 + dx * scale).astype(np.float32), (height / 2 + dy * scale).astype(np.float32)


def project_history(data, projection):
    names = dict(zip((0, 1, 2, 6), FOUR_CAMERA_RIG))
    render_rig = wide_render_rig()
    images = data["image_frames"]
    height, width = images.shape[-2:]
    projected = []
    for index, frames in zip(data["camera_indices"].tolist(), images):
        name = names[index]
        map_x, map_y = projection_map(width, height, render_rig[name]["fov"],
                                      FOUR_CAMERA_RIG[name]["fov"], projection)
        output = [cv2.remap(frame.permute(1, 2, 0).cpu().numpy(), map_x, map_y,
                            cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                  for frame in frames]
        projected.append(torch.from_numpy(np.stack(output)).permute(0, 3, 1, 2))
    return {**data, "image_frames": torch.stack(projected).contiguous()}
