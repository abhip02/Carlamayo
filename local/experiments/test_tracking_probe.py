"""Tracking screening must never mutate the real follower or native arrays."""
import copy
from types import SimpleNamespace

import numpy as np
import pytest

from local.experiments import tracking_probe as tracking
from local.experiments.test_retained_probe import setup_plan
from module.trajectory_follower import TrajectoryFollower


def fixture():
    route, context, options, plan, _ = setup_plan()
    options.update(validation_tracking_gain=.8448538582454754,
                   validation_tracking_horizon_s=1.5, validation_tracking_margin_m=.2)
    vehicle = SimpleNamespace(get_physics_control=lambda: SimpleNamespace(
        wheels=[SimpleNamespace(max_steer_angle=70.)]))

    class WrappedFollower(TrajectoryFollower):
        def compute_control(self, *args, **kwargs):
            raise AssertionError("Forecast executed a live logging/retention wrapper")

    follower = WrappedFollower(vehicle)
    follower._recent_targets.extend([3., 3.2, 2.9])
    follower._integral, follower._prev_speed, follower._prev_steer = .2, 3., .05
    return route, context, options, plan, follower


def test_forecast_copies_controller_state_and_preserves_native_arrays_and_age():
    route, context, options, plan, follower = fixture()
    before = copy.deepcopy(follower.__dict__)
    world, speeds = plan.world.copy(), plan.speeds.copy()
    metric = tracking.screen({"valid": True, "reasons": []}, context, route,
                             plan.world, plan.speeds, .4, options, follower)
    assert metric["valid"]
    assert metric["tracking_forecast"]["original_plan_age_s"] == .4
    assert follower.__dict__ == before
    np.testing.assert_array_equal(plan.world, world)
    np.testing.assert_array_equal(plan.speeds, speeds)


def test_tracking_rejects_departure_without_changing_or_replacing_native_path():
    route, context, options, plan, follower = fixture()
    path = plan.world.copy()
    path[:, 1] += 3.
    before = path.copy()
    metric = tracking.screen({"valid": True, "reasons": []}, context, route,
                             path, plan.speeds, .4, options, follower)
    assert not metric["valid"]
    assert metric["reasons"] == ["insufficient_controller_tracking_clearance"]
    np.testing.assert_array_equal(path, before)


def test_invalid_native_sample_is_never_readmitted_and_missing_follower_fails_closed():
    route, context, options, plan, _ = fixture()
    invalid = {"valid": False, "reasons": ["reverse_motion"]}
    assert tracking.screen(invalid, context, route, plan.world, plan.speeds, .4, options, None) == invalid
    with pytest.raises(RuntimeError, match="not available"):
        tracking.screen({"valid": True, "reasons": []}, context, route,
                        plan.world, plan.speeds, .4, options, None)


def test_entrypoint_restores_all_process_local_wrappers_on_failure(monkeypatch):
    runner = tracking.native.runner
    before = (runner.TrajectoryFollower, tracking.retained.RetainingProbe,
              tracking.validated.assess, tracking.retained.assess_retained)

    def fail():
        assert runner.TrajectoryFollower is not before[0]
        assert tracking.retained.RetainingProbe is tracking.TrackingProbe
        raise RuntimeError("inference failed")

    monkeypatch.setattr(tracking.retained, "main", fail)
    with pytest.raises(RuntimeError, match="inference failed"):
        tracking.main()
    assert (runner.TrajectoryFollower, tracking.retained.RetainingProbe,
            tracking.validated.assess, tracking.retained.assess_retained) == before
    assert tracking.FOLLOWER is None and tracking.OPTIONS is None


@pytest.mark.parametrize("key,value", [("validation_tracking_gain", float("nan")),
                                      ("validation_tracking_margin_m", -.1),
                                      ("validation_tracking_horizon_s", 1.55)])
def test_invalid_forecast_options_fail_before_sampling(key, value):
    _, _, options, _, _ = fixture()
    options[key] = value
    with pytest.raises(ValueError):
        tracking.check_options(options)


def test_preferred_reserve_is_separate_from_hard_admission():
    route, context, options, plan, follower = fixture()
    options['validation_preferred_reserve_m'] = .3
    metric = tracking.screen({'valid': True, 'reasons': [], 'max_corner_overhang_m': -.25},
                             context, route, plan.world, plan.speeds, .4, options, follower)
    assert metric['valid'] and not metric['preferred_reserve_valid']
    metric = tracking.screen({'valid': True, 'reasons': [], 'max_corner_overhang_m': -.4},
                             context, route, plan.world, plan.speeds, .4, options, follower)
    assert metric['valid'] and metric['preferred_reserve_valid']
    options['validation_preferred_reserve_m'] = .05
    with pytest.raises(ValueError, match='Preferred reserve'):
        tracking.check_options(options)
