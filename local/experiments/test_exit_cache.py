"""Unknown exit reuse stays opt-in and cannot bypass physical admission."""
import copy

import numpy as np
import pytest

from local.experiments.exit_cache import unknown_exit_evidence
from local.experiments.retained_probe import assess_retained
from local.experiments.test_retained_probe import setup_plan


def fixture():
    route, context, options, plan, _ = setup_plan()
    points = [{'x': -10., 'y': -10., 'yaw': 90., 'lane_width': 3.5},
              {'x': -10., 'y': 0., 'yaw': 0., 'lane_width': 3.5}, *route['points']]
    route.update(points=points, entry=points[0], exit_index=3)
    options.update(validation_route='left', verify_signal=True, validation_allow_unknown_exit_cache=True)
    plan.record['selection'].update(signal='unknown', route='left')
    plan.record['navigation_phase'] = 'finish_turn'
    return route, context, options, plan


def test_matching_unknown_exit_preserves_classification_and_original_native_plan():
    route, context, options, plan = fixture()
    original = copy.deepcopy(plan.record)
    world, speeds = plan.world.copy(), plan.speeds.copy()
    metric = assess_retained(plan, context, route, 'unknown', 'finish_turn', options)
    assert metric['valid'] and metric['age_s'] == .4
    assert metric['signal_reuse']['source_signal'] == metric['signal_reuse']['current_signal'] == 'unknown'
    assert plan.record == original
    np.testing.assert_array_equal(plan.world, world)
    np.testing.assert_array_equal(plan.speeds, speeds)
    assert metric['max_corner_overhang_m'] <= -.1


@pytest.mark.parametrize('source,current', [
    ('green', 'unknown'), ('unknown', 'green'), ('red', 'unknown'),
    ('unknown', 'red'), ('yellow', 'unknown'), ('unknown', 'yellow')])
def test_changed_or_red_yellow_signals_remain_ineligible(source, current):
    route, context, options, plan = fixture()
    plan.record['selection']['signal'] = source
    metric = assess_retained(plan, context, route, current, 'finish_turn', options)
    assert not metric['valid'] and 'reuse_requires_green' in metric['reasons']
    assert 'signal_reuse' not in metric


@pytest.mark.parametrize('change', ['disabled', 'no_vqa', 'straight_route', 'approach',
                                    'source_heading', 'current_heading', 'source_outside',
                                    'current_outside', 'source_before_turn', 'nonfinite'])
def test_unknown_reuse_is_restricted_to_both_aligned_exit_states(change):
    route, context, options, plan = fixture()
    phase = 'finish_turn'
    if change == 'disabled': options.pop('validation_allow_unknown_exit_cache')
    elif change == 'no_vqa': options['verify_signal'] = False
    elif change == 'straight_route': options['validation_route'] = 'straight'
    elif change == 'approach': phase = 'approach'
    elif change == 'source_heading': plan.record['context']['world_pose']['yaw'] = 11.
    elif change == 'current_heading': context['world_pose']['yaw'] = 11.
    elif change == 'source_outside': plan.record['context']['world_pose']['y'] = 2.
    elif change == 'current_outside': context['world_pose']['y'] = 2.
    elif change == 'source_before_turn': plan.record['context']['world_pose'].update(x=-10., y=-5.)
    elif change == 'nonfinite': context['world_pose']['yaw'] = float('nan')
    assert unknown_exit_evidence(plan.record, context, route, 'unknown', phase, options) is None
    metric = assess_retained(plan, context, route, 'unknown', phase, options)
    assert not metric['valid']


@pytest.mark.parametrize('change,reason', [('age','expired_or_nonpast_plan'),('body','current_body_outside_lane')])
def test_eligible_signal_cannot_override_age_or_physical_rejection(change, reason):
    route, context, options, plan = fixture()
    if change == 'age': context['time_s'] = .6
    else: context['world_pose']['y'] = 1.
    metric = assess_retained(plan, context, route, 'unknown', 'finish_turn', options)
    assert 'signal_reuse' in metric
    assert not metric['valid'] and reason in metric['reasons']


def test_green_pair_is_unchanged_and_does_not_use_unknown_policy():
    route, context, options, plan = fixture()
    plan.record['selection']['signal'] = 'green'
    with_option = assess_retained(plan, context, route, 'green', 'finish_turn', options)
    options.pop('validation_allow_unknown_exit_cache')
    without_option = assess_retained(plan, context, route, 'green', 'finish_turn', options)
    assert with_option == without_option and with_option['valid']
    assert 'signal_reuse' not in with_option
