"""Optional controller-aware admission of unchanged native model samples.

Controlled empty-green experiment. Approximate constant-speed tracking is an
extra rejection check, not a certified vehicle/obstacle model or executed path.
Original model arrays, original controller state and original cache clocks are
preserved. Failed fresh candidates trigger the existing native resampling loop.
"""
import math

import numpy as np

from local.experiments import inference_probe as native, proactive_probe as proactive
from local.experiments import retained_probe as retained, validated_probe as validated
from local.experiments.tracking_forecast import forecast
from local.experiments.validated_selection import pose_transform
from module.trajectory_follower import planned_speeds, trajectory_to_world


FOLLOWER = None
OPTIONS = None


def check_options(options):
    for key, low, high in (("validation_tracking_gain", .5, 1.2),
                           ("validation_tracking_horizon_s", .1, 2.),
                           ("validation_tracking_margin_m", 0., 1.)):
        value = options.get(key)
        if value is None or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"{key} must be explicitly specified in [{low}, {high}]")
    if abs(round(options["validation_tracking_horizon_s"] / .1) * .1 -
           options["validation_tracking_horizon_s"]) > 1e-8:
        raise ValueError("Tracking horizon must align with the 0.1 s control grid")
    if not options.get("validation_check_body") or options.get("validation_route") not in ("left", "straight"):
        raise ValueError("Tracking screening requires existing native body and route checks")


def screen(metric, context, route, world, speeds, age, options, follower):
    if not metric["valid"]:
        return metric
    if follower is None:
        raise RuntimeError("Original follower is not available for tracking screening")
    state = {**context["world_pose"], "speed": context["measured_speed_mps"]}
    if not np.isfinite(list(state.values())).all():
        metric["valid"] = False
        metric["reasons"].append("nonfinite_tracking_state")
        return metric
    rows = forecast(follower, state, world, speeds, age, route, context["vehicle_bounds"],
                    options["validation_tracking_horizon_s"], options["validation_tracking_gain"])
    clearances = np.array([r["corner_clearance_m"] for r in rows])
    passed = bool(np.isfinite(clearances).all() and clearances.min() >= options["validation_tracking_margin_m"])
    metric["tracking_forecast"] = {
        "valid": passed, "min_corner_clearance_m": float(clearances.min()),
        "minimum_at_s": rows[int(clearances.argmin())]["dt_s"],
        "horizon_s": options["validation_tracking_horizon_s"],
        "required_clearance_m": options["validation_tracking_margin_m"],
        "effective_steering_angle_gain": options["validation_tracking_gain"],
        "assumed_constant_speed_mps": state["speed"], "original_plan_age_s": age,
        "model": "Copied original follower; approximate constant-speed bicycle; no obstacle model."}
    if not passed:
        metric["valid"] = False
        metric["reasons"].append("insufficient_controller_tracking_clearance")
    return metric


class TrackingProbe(proactive.ProactiveProbe):
    def evaluate(self, adapter, data, context, instruction, options, label,
                 capture=False, capture_failures=False):
        global OPTIONS
        check_options(options)
        OPTIONS = options
        try:
            return super().evaluate(adapter, data, context, instruction, options, label,
                                    capture=capture, capture_failures=capture_failures)
        finally:
            OPTIONS = None


def main():
    global FOLLOWER, OPTIONS
    runner = native.runner
    original_follower, original_probe = runner.TrajectoryFollower, retained.RetainingProbe
    original_assess, original_cached = validated.assess, retained.assess_retained

    class TrackingFollower(original_follower):
        def __init__(self, vehicle):
            global FOLLOWER
            super().__init__(vehicle)
            FOLLOWER = self

    def fresh(candidates, context, route, *args, **kwargs):
        metrics = original_assess(candidates, context, route, *args, **kwargs)
        if OPTIONS is None:
            raise RuntimeError("Missing tracking options")
        for item, raw in zip(metrics, candidates):
            if item["valid"]:
                world = trajectory_to_world(pose_transform(context["world_pose"]), raw)
                screen(item, context, route, world, planned_speeds(raw), 0., OPTIONS, FOLLOWER)
        return metrics

    def cached(plan, context, route, signal, phase, options):
        metric = original_cached(plan, context, route, signal, phase, options)
        return screen(metric, context, route, plan.world, plan.speeds,
                      context["time_s"] - plan.record["context"]["time_s"], options, FOLLOWER)

    runner.TrajectoryFollower, retained.RetainingProbe = TrackingFollower, TrackingProbe
    validated.assess, retained.assess_retained = fresh, cached
    try:
        retained.main()
    finally:
        runner.TrajectoryFollower, retained.RetainingProbe = original_follower, original_probe
        validated.assess, retained.assess_retained = original_assess, original_cached
        FOLLOWER = OPTIONS = None


if __name__ == "__main__":
    main()
