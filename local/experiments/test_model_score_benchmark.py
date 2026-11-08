"""Analytic checks for coordinate/timestamp alignment and model-only scores."""
import numpy as np
import pytest

from module import config
from local.experiments.model_score_benchmark import recorded_reference, trajectory_scores, aggregate


def fixture():
    pose = dict(x=-config.EGO_REAR_AXLE_X, y=0., z=0., yaw=0., pitch=0., roll=0.)
    context = dict(world_pose=pose, snapshot_frame=100,
                   vehicle_bounds=dict(location=dict(x=0., y=0., z=0.),
                                       extent=dict(x=2., y=1., z=.5),
                                       rotation=dict(pitch=0., yaw=0., roll=0.)))
    route = dict(points=[dict(x=x, y=0., lane_width=3.5) for x in (-100., 0., 100.)], exit_index=2)
    trajectory = np.zeros((64, 3))
    trajectory[:, 0] = np.arange(1, 65) * .2
    return context, route, trajectory


def test_exact_path_has_zero_error_and_correct_progress():
    context, route, trajectory = fixture()
    scores, world = trajectory_scores(trajectory, np.zeros(64), context, route, trajectory[:60])
    np.testing.assert_allclose(world, trajectory, atol=1e-6)
    assert scores['recorded_reference_ade_m_3s'] == pytest.approx(0., abs=1e-6)
    assert scores['recorded_reference_fde_m_3s'] == pytest.approx(0., abs=1e-6)
    assert scores['route_rmse_m_3s'] == pytest.approx(0., abs=1e-6)
    assert scores['route_progress_m_3s'] == pytest.approx(6., abs=1e-6)
    assert not scores['any_corner_outside_3s']
    assert scores['minimum_corner_margin_m_3s'] == pytest.approx(.75, abs=1e-6)
    assert scores['jerk_rms_mps3_3s'] < 1e-9


def test_left_positive_model_coordinates_and_lateral_departure():
    context, route, trajectory = fixture()
    reference = trajectory[:60].copy()
    trajectory[:, 1] = 1.
    scores, world = trajectory_scores(trajectory, np.zeros(64), context, route, reference)
    np.testing.assert_allclose(world[:, 1], -1.)
    assert scores['recorded_reference_ade_m_3s'] == pytest.approx(1., abs=1e-6)
    assert scores['route_rmse_m_3s'] == pytest.approx(1., abs=1e-6)
    assert scores['corner_outside_fraction_3s'] == 1.
    assert scores['minimum_corner_margin_m_3s'] == pytest.approx(-.25, abs=1e-6)


def test_red_line_uses_front_corners_not_rear_axle():
    context, route, trajectory = fixture()
    mission = dict(stop_line=dict(x=8., y=0., yaw=0.))
    scores, _ = trajectory_scores(trajectory, np.zeros(64), context, route, trajectory[:60], mission)
    # Rear axle is only at 6 m, but the forward vehicle corner is beyond 8 m.
    assert scores['red_stop_line_crossed_3s']
    assert not scores['red_stop_line_crossed_1s']


def test_reference_matches_capture_frame_and_interpolates_rear_axle():
    context, _, _ = fixture()
    rows = [dict(**{**context['world_pose'], 'x': context['world_pose']['x'] + i * .4},
                 frame=100+i, time_s=i*.2) for i in range(32)]
    times = np.arange(1, 61) * .1
    reference = recorded_reference(rows, context, times)
    np.testing.assert_allclose(reference[:, 0], times * 2, atol=2e-6)
    np.testing.assert_allclose(reference[:, 1:], 0.)
    with pytest.raises(ValueError, match='whole requested horizon'):
        recorded_reference(rows[:10], context, times)
    with pytest.raises(ValueError, match='exactly one'):
        recorded_reference(rows, {**context, 'snapshot_frame': -1}, times)


def test_bad_trajectory_cannot_silently_receive_a_score():
    context, route, trajectory = fixture()
    trajectory[0, 0] = np.nan
    with pytest.raises(ValueError, match='Nonfinite'):
        trajectory_scores(trajectory, np.zeros(64), context, route, trajectory[:60])


def test_red_only_aggregate_has_no_moving_progress():
    row = dict(case='10-red', valid=True, inference_seconds=1., peak_allocated_gib=2., generated_tokens=3)
    for h in (1, 3, 6):
        row[f'route_progress_m_{h}s'] = 1.
    assert aggregate([row])['moving_route_progress_m_3s'] is None
