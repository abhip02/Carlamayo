"""Fixed observer camera with video-only trajectory overlays."""
from __future__ import annotations

import json
import math
from pathlib import Path
import queue
import time

import carla
import cv2
import numpy as np

from module.config import EGO_REAR_AXLE_X

ACTUAL_BGR = (255, 210, 0)
PREDICTED_BGR = (0, 215, 255)


def rear_axle_xyz(transform):
    point = transform.transform(carla.Location(x=EGO_REAR_AXLE_X))
    return np.array([point.x, point.y, point.z], dtype=np.float64)


def project_world(points, camera_transform, width, height, fov):
    """Project CARLA world coordinates through a pinhole camera; NaN behind camera."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    homogeneous = np.column_stack((points, np.ones(len(points))))
    camera = homogeneous @ np.asarray(camera_transform.get_inverse_matrix()).T
    pixels = np.full((len(points), 2), np.nan)
    visible = np.isfinite(camera).all(axis=1) & (camera[:, 0] > 0.05)
    focal = width / (2 * math.tan(math.radians(fov) / 2))
    # UE camera axes: forward/right/up -> image depth/right/down.
    pixels[visible, 0] = width / 2 + focal * camera[visible, 1] / camera[visible, 0]
    pixels[visible, 1] = height / 2 - focal * camera[visible, 2] / camera[visible, 0]
    return pixels


class OverheadRecorder:
    """Independent RGB sensor: never included in the model's camera queue."""

    def __init__(self, world, entry, output, options=None, label=""):
        options = options or {}
        self.output = Path(output)
        self.width, self.height = 1280, 960
        self.fov = 90.0
        self.label = label
        self.inset = options.get("front_inset", True)
        self.queue = queue.Queue(maxsize=4)
        self.trail = []
        self.last_image = None
        self.sensor = None
        self.writer = None
        altitude = float(options.get("height_m", 80))
        offset = float(options.get("center_ahead_m", 15))
        if not math.isfinite(altitude) or altitude <= 0 or not math.isfinite(offset):
            raise ValueError("Overhead height must be positive and camera offsets finite")
        yaw = math.radians(entry["yaw"])
        transform = carla.Transform(
            carla.Location(x=entry["x"] + offset * math.cos(yaw),
                           y=entry["y"] + offset * math.sin(yaw), z=entry["z"] + altitude),
            carla.Rotation(pitch=-90, yaw=entry["yaw"]))
        blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        for key, value in dict(image_size_x=self.width, image_size_y=self.height,
                               fov=self.fov, sensor_tick=0.0, enable_postprocess_effects="true").items():
            blueprint.set_attribute(key, str(value))
        try:
            self.sensor = world.spawn_actor(blueprint, transform)
            self.sensor.listen(self._receive)
            self.writer = cv2.VideoWriter(str(self.output / "overhead.mp4"),
                                          cv2.VideoWriter_fourcc(*"mp4v"), 10,
                                          (self.width, self.height))
            if not self.writer.isOpened():
                raise RuntimeError("Could not open overhead.mp4")
            metadata = dict(width=self.width, height=self.height, fov_degrees=self.fov,
                            camera_world_matrix=transform.get_matrix(), fps=10,
                            path_reference="rear axle", actual_color="cyan", predicted_color="yellow",
                            overlay="postprocessed video only; not rendered into model camera inputs",
                            height_m=altitude, center_ahead_m=offset)
            (self.output / "overhead.json").write_text(json.dumps(metadata, indent=2) + "\n")
        except BaseException:
            self.close()
            raise

    def _receive(self, image):
        try:
            self.queue.put_nowait(image)
        except queue.Full:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait(image)
            except queue.Full:
                pass

    def _image_for_frame(self, frame_id):
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            try:
                image = self.queue.get(timeout=max(0.001, deadline - time.monotonic()))
            except queue.Empty:
                break
            if image.frame == frame_id:
                return image
            if image.frame > frame_id:
                raise RuntimeError(f"Overhead camera skipped requested frame {frame_id}")
        raise TimeoutError(f"No overhead image for CARLA frame {frame_id}")

    def _draw_path(self, image, points, camera_transform, color, thickness):
        pixels = project_world(points, camera_transform, self.width, self.height, self.fov)
        segments = []
        for a, b in zip(pixels, pixels[1:]):
            if not (np.isfinite(a).all() and np.isfinite(b).all()):
                continue
            a, b = [tuple(np.clip(p, -1e6, 1e6).astype(int)) for p in (a, b)]
            inside, a, b = cv2.clipLine((0, 0, self.width, self.height), a, b)
            if inside:
                segments.append((a, b))
        # Draw all outlines first so adjacent segments do not erase each other's color.
        for a, b in segments:
            cv2.line(image, a, b, (20, 20, 20), thickness + 2, cv2.LINE_AA)
        for a, b in segments:
            cv2.line(image, a, b, color, thickness, cv2.LINE_AA)
        return pixels

    def add_frame(self, frame_id, transform, elapsed, speed, *, prediction=None,
                  prediction_time=None, front_rgb=None):
        source = self._image_for_frame(frame_id)
        image = np.frombuffer(source.raw_data, dtype=np.uint8).reshape(
            source.height, source.width, 4)[:, :, :3].copy()
        self.trail.append(rear_axle_xyz(transform))
        if prediction is not None:
            self._draw_path(image, prediction, source.transform, PREDICTED_BGR, 4)
        pixels = self._draw_path(image, self.trail, source.transform, ACTUAL_BGR, 4)
        for point, color in ((pixels[0], (255, 255, 255)), (pixels[-1], ACTUAL_BGR)):
            if np.isfinite(point).all() and 0 <= point[0] < self.width and 0 <= point[1] < self.height:
                center = tuple(point.astype(int))
                cv2.circle(image, center, 6, (20, 20, 20), -1, cv2.LINE_AA)
                cv2.circle(image, center, 4, color, -1, cv2.LINE_AA)
        cv2.rectangle(image, (0, 0), (self.width, 74), (22, 26, 32), -1)
        cv2.putText(image, f"{self.label}   t={elapsed:.1f} s   speed={speed:.2f} m/s",
                    (20, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (245, 245, 245), 1, cv2.LINE_AA)
        for x, text, color in ((20, "Actual path so far", ACTUAL_BGR),
                               (340, "Latest predicted path (6.4 s)", PREDICTED_BGR)):
            cv2.line(image, (x, 55), (x + 36, 55), color, 4, cv2.LINE_AA)
            cv2.putText(image, text, (x + 46, 61), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (235, 235, 235), 1, cv2.LINE_AA)
        status = ("Waiting for prediction" if prediction_time is None
                  else f"Prediction age: {max(0, elapsed - prediction_time):.1f} s")
        cv2.putText(image, status, (960, 59), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (210, 210, 210), 1, cv2.LINE_AA)
        if self.inset and front_rgb is not None:
            inset = cv2.cvtColor(cv2.resize(front_rgb, (320, 180)), cv2.COLOR_RGB2BGR)
            x, y = self.width - 336, self.height - 196
            cv2.rectangle(image, (x - 3, y - 24), (x + 323, y + 183), (22, 26, 32), -1)
            image[y:y + 180, x:x + 320] = inset
            cv2.putText(image, "Front camera", (x + 8, y - 7), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (245, 245, 245), 1, cv2.LINE_AA)
        self.writer.write(image)
        self.last_image = image

    def close(self):
        if self.sensor is not None and self.sensor.is_alive:
            self.sensor.stop()
            self.sensor.destroy()
        self.sensor = None
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        if self.last_image is not None:
            cv2.imwrite(str(self.output / "overhead-preview.jpg"), self.last_image)
