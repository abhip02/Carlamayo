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


# Lens values from CARLA's public PhysicalAI camera rig, retrieved 2026-10-29:
# https://github.com/carla-simulator/carla/blob/ue58-dev/PythonAPI/examples/av_stacks/alpamayo/alpamayo_rig.py
# These refine the existing rig. Its nominal poses/focal lengths/centers also
# match the 1.5 reference clip; the fitted lens is still an approximation.
# (focal length, principal point, Kannala-Brandt coefficients), at 1920 x 1080.
LENS_CALIBRATION = {
    0: (921.978679, (965.79980657, 546.77842273),
        (-0.020325173, 0.015561344, -0.019556958, 0.008121544)),
    1: (925.540742, (957.85292258, 537.02554301),
        (-0.041555584, 0.067146887, -0.078102726, 0.031954250)),
    2: (917.283211, (960.73577164, 539.01147371),
        (-0.007380577, -0.015709779, 0.017383842, -0.007038902)),
    6: (3687.248370, (966.87830919, 548.33546092),
        (0.058971681, -1.359812221, 23.21701466, -149.2217783)),
}


@lru_cache(maxsize=8)
def calibrated_projection_map(width, height, camera_index):
    """Map target lens pixels back to the saved wide pinhole camera."""
    focal, center, coefficients = LENS_CALIBRATION[camera_index]
    sx, sy = width / 1920, height / 1080
    intrinsics = np.array([[focal * sx, 0, center[0] * sx],
                           [0, focal * sy, center[1] * sy], [0, 0, 1]], dtype=np.float64)
    y, x = np.mgrid[:height, :width]
    target = np.stack((x, y), axis=-1).astype(np.float64).reshape(-1, 1, 2)
    rays = cv2.fisheye.undistortPoints(target, intrinsics, np.array(coefficients))
    rays = rays.reshape(height, width, 2)
    source_fov = 35 if camera_index == 6 else 135
    source_focal = width / (2 * math.tan(math.radians(source_fov) / 2))
    return ((rays[..., 0] * source_focal + width / 2).astype(np.float32),
            (rays[..., 1] * source_focal + height / 2).astype(np.float32))


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
        if projection == "calibrated_fisheye":
            map_x, map_y = calibrated_projection_map(width, height, index)
        else:
            map_x, map_y = projection_map(width, height, render_rig[name]["fov"],
                                          FOUR_CAMERA_RIG[name]["fov"], projection)
        output = [cv2.remap(frame.permute(1, 2, 0).cpu().numpy(), map_x, map_y,
                            cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                  for frame in frames]
        projected.append(torch.from_numpy(np.stack(output)).permute(0, 3, 1, 2))
    return {**data, "image_frames": torch.stack(projected).contiguous()}
