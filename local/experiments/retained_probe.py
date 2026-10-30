"""Reuse a recent, revalidated native world path only after new sampling fails.

Controlled empty-green experiments only. Original waypoints, world anchor and
prediction time are preserved. The production follower is called unchanged,
with its existing elapsed_sec argument. No map path or repaired path is made.
"""
import copy
import json
import math
from dataclasses import dataclass

import carla
import numpy as np

from local.experiments import inference_probe as native, validated_probe as validated
from local.experiments.audit_turn import project
from local.experiments.validated_selection import choose, pose_transform, predicted_corners
from module import config
from module.trajectory_follower import planned_speeds, trajectory_to_world


ACTIVE = None


@dataclass
class ModelPlan:
    record: dict
    prediction: object
    extra: dict
    world: np.ndarray
    corners: np.ndarray
    speeds: np.ndarray

    @classmethod
    def from_record(cls, record, prediction, extra):
        record = copy.deepcopy(record)
        trajectory = np.asarray(record["trajectory_ego"])
        source = record["context"]
        tf = pose_transform(source["world_pose"])
        yaw = np.asarray(record["candidate_yaw_rad"][record["selection"]["chosen"]])
        world = trajectory_to_world(tf, trajectory)
        world.setflags(write=False)
        return cls(record, prediction.detach().clone(), copy.deepcopy(extra), world,
                   predicted_corners(tf, trajectory, yaw, source["vehicle_bounds"]),
                   planned_speeds(trajectory))


def assess_retained(plan, context, route, signal, phase, options):
    """Check an unchanged world path against the current state and its real age."""
    record, source = plan.record, plan.record["context"]
    age = context["time_s"] - source["time_s"]
    maximum_age = options["validation_cache_max_age_s"]
    reasons = []
    item = {"source_label": record["label"], "source_time_s": source["time_s"],
            "age_s": age, "valid": False, "reasons": reasons}
    if age <= 0 or age > maximum_age + 1e-6:
        reasons.append("expired_or_nonpast_plan")
    if signal != "green" or record["selection"]["signal"] != "green":
        reasons.append("reuse_requires_green")
    if phase != record["navigation_phase"]:
        reasons.append("navigation_phase_changed")
    if reasons:
        return item
    if not np.isfinite([*context["world_pose"].values(), context["measured_speed_mps"]]).all():
        reasons.append("nonfinite_current_state")
        return item
    now = round(age / .1)
    speed_start = now + int(config.PID_SPEED_LEAD_SEC / .1 + 1e-6)
    speed_end = speed_start + max(1, round(config.PID_SPEED_PREVIEW_SEC / .1))
    if abs(now * .1 - age) > 1e-6 or max(now + 15, speed_end) > len(plan.world):
        reasons.append("unaligned_or_exhausted_plan_time")
        return item
    tf = pose_transform(context["world_pose"])
    rear = tf.transform(carla.Location(x=config.EGO_REAR_AXLE_X))
    origin = np.array([rear.x, rear.y])
    _, initial_arc, _, _, _ = project(origin[None], route)
    bounds = context["vehicle_bounds"]
    box = carla.BoundingBox(carla.Location(**bounds["location"]), carla.Vector3D(**bounds["extent"]))
    box.rotation = carla.Rotation(**bounds["rotation"])
    actual = np.array([[v.x, v.y] for v in box.get_world_vertices(tf)])
    distance, _, _, width, _ = project(actual, route)
    item["current_corner_overhang_m"] = float((distance - width).max())
    if item["current_corner_overhang_m"] > 0:
        reasons.append("current_body_outside_lane")

    # Distance to the old polyline, not to a temporally reanchored/repaired path.
    segment = np.diff(plan.world[:, :2], axis=0)
    length2 = np.sum(segment**2, axis=1)
    good = length2 > 1e-10
    if not np.any(good):
        reasons.append("no_moving_path_segment")
        return item
    starts, vectors = plan.world[:-1, :2][good], segment[good]
    fraction = np.clip(np.sum((origin - starts) * vectors, axis=1) / length2[good], 0, 1)
    path_distance = np.linalg.norm(starts + fraction[:, None] * vectors - origin, axis=1)
    nearest_segment = int(path_distance.argmin())
    tangent = math.degrees(math.atan2(vectors[nearest_segment, 1], vectors[nearest_segment, 0]))
    heading_error = abs((context["world_pose"]["yaw"] - tangent + 180) % 360 - 180)
    item.update(current_distance_to_cached_path_m=float(path_distance.min()),
                current_heading_to_cached_path_deg=float(heading_error))
    if path_distance.min() > options.get("validation_cache_max_distance_m", .5):
        reasons.append("too_far_from_cached_path")
    if heading_error > options.get("validation_cache_max_heading_deg", 20.):
        reasons.append("misaligned_with_cached_path")

    # Match the original follower's spatial target and age-adjusted speed preview.
    distances = np.linalg.norm(plan.world[:, :2] - origin, axis=1)
    start = int(distances.argmin())
    lookahead = np.clip(config.PID_LOOKAHEAD_MIN_M + config.PID_LOOKAHEAD_SPEED_GAIN *
                        context["measured_speed_mps"], config.PID_LOOKAHEAD_MIN_M,
                        config.PID_LOOKAHEAD_MAX_M)
    ahead = np.flatnonzero(distances[start:] >= lookahead)
    target = start + (int(ahead[0]) if len(ahead) else len(plan.world[start:]) - 1)
    first = min(start, now)
    limit = min(len(plan.world), max(now + max(15, round(options.get("validation_horizon_s", 1.5) / .1)),
                                      speed_end, target + 1))
    distance, arc, headings, widths, _ = project(plan.world[first:limit, :2], route)
    corner_d, _, _, corner_w, _ = project(plan.corners[first:limit, :, :2].reshape(-1, 2), route)
    overhang = float((corner_d - corner_w).max())
    margin = options.get("validation_body_margin_m", 0.)
    item.update(max_corner_overhang_m=overhang, required_corner_clearance_m=margin,
                controller_start_index=start, controller_target_index=target,
                checked_original_start_index=first, checked_original_end_index=limit - 1)
    if np.any(distance > widths):
        reasons.append("outside_lane_center_corridor")
    if overhang > -margin:
        reasons.append("insufficient_cached_corner_clearance")
    velocity = np.asarray(record["candidate_velocities"][record["selection"]["chosen"]])[now:limit]
    if np.any(velocity < -.1):
        reasons.append("reverse_motion")
    # Segment headings belong to the original path, not a fictitious jump from
    # the actual car to an old timestamp's predicted position.
    previous = plan.world[max(0, first - 1):limit - 1, :2]
    future = plan.world[max(1, first):limit, :2]
    delta = future - previous
    moving = np.linalg.norm(delta, axis=1) > .05
    path_heading = np.degrees(np.arctan2(delta[:, 1], delta[:, 0]))
    route_heading = headings[1:] if first == 0 else headings
    if np.any(np.abs((path_heading[moving] - route_heading[moving] + 180) % 360 - 180) > 45):
        reasons.append("misaligned_with_route")
    rank_d, rank_arc, _, _, _ = project(plan.world[now:now + 15, :2], route)
    target_speed = float(plan.speeds[speed_start:speed_end].mean())
    if target_speed < options.get("validation_green_min_target_mps", 0.):
        reasons.append("green_progress_requirement")
    item.update(valid=not reasons, target_speed_mps=target_speed,
                ranking_horizon_s=1.5, ranking_progress_m=float(rank_arc[-1] - initial_arc[0]),
                ranking_lane_rms_m=float(np.sqrt(np.mean(rank_d**2))))
    return item


class RetainingProbe(validated.ValidatingProbe):
    def __init__(self, model, output):
        global ACTIVE
        super().__init__(model, output)
        self.history, self.executing_plan, self.executing_age = [], None, 0.
        self.execution_log = (output / "controller-inputs.jsonl").open("w", buffering=1)
        ACTIVE = self

    def close(self):
        try:
            self.execution_log.close()
        finally:
            super().close()

    def evaluate(self, adapter, data, context, instruction, options, label,
                 capture=False, capture_failures=False):
        maximum_age = options.get("validation_cache_max_age_s", 0.)
        if not 0 < maximum_age <= .5 or not options.get("validation_check_body"):
            raise ValueError("Retention requires body checks and a cache age in (0, 0.5] seconds")
        self.history = [p for p in self.history if context["time_s"] - p.record["context"]["time_s"] <= maximum_age + 1e-6]
        try:
            prediction, extra = super().evaluate(adapter, data, context, instruction, options, label,
                                                capture=capture, capture_failures=capture_failures)
        except validated.NoValidatedPlan:
            failure = json.loads((self.output / "validation-failure.json").read_text())
            metrics = [dict(assess_retained(p, context, self.validation_route, self.visual_signal,
                                          self.navigation_phase, options), candidate=i)
                       for i, p in enumerate(self.history)]
            chosen = choose(metrics, [p.record["trajectory_ego"] for p in self.history],
                            self.visual_signal, context["measured_speed_mps"],
                            options.get("validation_desired_speed_mps", 5.))
            if chosen is None:
                native.write_json(self.output / "retention-failure.json", {"context": context, "metrics": metrics})
                raise
            plan = self.history[chosen]
            age = context["time_s"] - plan.record["context"]["time_s"]
            record = copy.deepcopy(plan.record)
            record.update(label=label, context=context, navigation_phase=self.navigation_phase,
                          visual_signal=self.visual_signal,
                          visual_signal_answer=self.visual_signal_answer,
                          visual_signal_check_index=self.visual_signal_index,
                          inference_seconds=sum(a["inference_seconds"] for a in failure["attempts"]),
                          raw_target_speed_mps=metrics[chosen]["target_speed_mps"],
                          controller_preview_speed_mps=metrics[chosen]["target_speed_mps"],
                          selection={"mode": "retained_native_sample", "chosen": plan.record["selection"]["chosen"],
                                     "proposal": plan.record["selection"]["proposal"],
                                     "route": options["validation_route"], "signal": self.visual_signal},
                          retention={"source_label": plan.record["label"],
                                     "source_context": plan.record["context"], "age_s": age,
                                     "chosen_cache_index": chosen, "metrics": metrics,
                                     "rejected_new_pools": failure["attempts"]})
            rejected = self.output / "replan-rejections"
            rejected.mkdir(exist_ok=True)
            native.write_json(rejected / (label + ".json"), failure)
            native.write_json(self.output / (label + ".json"), record)
            # Preserve the rejection above; this file is reserved for a terminal
            # failure, so a completed run cannot conceal failures in its summary.
            (self.output / "validation-failure.json").unlink()
            self.executing_plan, self.executing_age = plan, age
            print(json.dumps({"retained": label, "source": plan.record["label"], "age_s": age,
                              "target_speed_mps": metrics[chosen]["target_speed_mps"]}), flush=True)
            return plan.prediction, copy.deepcopy(plan.extra)
        record = json.loads((self.output / (label + ".json")).read_text())
        plan = ModelPlan.from_record(record, prediction, extra)
        self.history.append(plan)
        self.executing_plan, self.executing_age = plan, 0.
        return prediction, extra


def anchored_world_path(tf, trajectory):
    plan = ACTIVE.executing_plan
    if not np.array_equal(np.asarray(trajectory), np.asarray(plan.record["trajectory_ego"])):
        raise RuntimeError("Controller requested an array other than the selected native plan")
    return plan.world


def main():
    global ACTIVE
    runner = native.runner
    original_probe, original_world = validated.ValidatingProbe, runner.trajectory_to_world
    original_follower, original_recorder = runner.TrajectoryFollower, runner.OverheadRecorder

    class RetainingFollower(original_follower):
        def compute_control(self, tf, world, speed, speeds, elapsed_sec=0.):
            plan, age = ACTIVE.executing_plan, ACTIVE.executing_age
            if not np.array_equal(world, plan.world) or not np.array_equal(speeds, plan.speeds):
                raise RuntimeError("Retained controller input differs from the original model plan")
            control = super().compute_control(tf, world, speed, speeds, elapsed_sec=age)
            ACTIVE.execution_log.write(json.dumps({
                "time_s": plan.record["context"]["time_s"] + age,
                "source_label": plan.record["label"], "source_context": plan.record["context"],
                "age_s": age, "world_trajectory": world.tolist(), "speeds_mps": speeds.tolist(),
                "control": dict(zip(("steer", "throttle", "brake"), control[:3])),
                "controller_debug": control[3]}) + "\n")
            return control

    class RetainingRecorder(original_recorder):
        def add_frame(self, *args, **kwargs):
            if kwargs.get("prediction") is not None and ACTIVE.executing_plan is not None:
                plan = ACTIVE.executing_plan
                tf = pose_transform(plan.record["context"]["world_pose"])
                kwargs.update(prediction=np.vstack((runner.rear_axle_xyz(tf), plan.world)),
                              prediction_time=plan.record["context"]["time_s"])
            return super().add_frame(*args, **kwargs)

    validated.ValidatingProbe, runner.trajectory_to_world = RetainingProbe, anchored_world_path
    runner.TrajectoryFollower, runner.OverheadRecorder = RetainingFollower, RetainingRecorder
    try:
        validated.main()
    finally:
        validated.ValidatingProbe, runner.trajectory_to_world = original_probe, original_world
        runner.TrajectoryFollower, runner.OverheadRecorder = original_follower, original_recorder
        ACTIVE = None


if __name__ == "__main__":
    main()
