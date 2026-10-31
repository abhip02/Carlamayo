"""A compatible instruction transition must not change or refresh a native plan."""
import copy
import json

import numpy as np
import pytest

from local.experiments.audit_retention import audit
from local.experiments.retained_probe import ModelPlan, assess_retained
from local.experiments.test_retained_probe import setup_plan
from local.experiments import tracking_probe


def fixture():
    _, context, options, original, prediction = setup_plan()
    # A local straight segment during a left turn, with a later horizontal exit.
    angle = np.deg2rad(31)
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    record = copy.deepcopy(original.record)
    for item in (record['context'], context):
        pose = item['world_pose']
        pose['x'], pose['y'] = map(float, rotation @ [pose['x'], pose['y']])
        pose['yaw'] = 31.
    context['world_pose']['yaw'] = 29.  # Source turn is 59 degrees; current is 61.
    far = rotation @ [50., 0.]
    points = [{'x': 0., 'y': -20., 'yaw': 90., 'lane_width': 3.5},
              {'x': 0., 'y': 0., 'yaw': 31., 'lane_width': 3.5},
              {'x': float(far[0]), 'y': float(far[1]), 'yaw': 0., 'lane_width': 3.5},
              {'x': float(far[0]+50), 'y': float(far[1]), 'yaw': 0., 'lane_width': 3.5}]
    route = {'points': points, 'entry': points[0], 'exit': points[-1], 'exit_index': 3}
    record['selection']['route'] = 'left'
    record['visual_signal'] = 'green'
    plan = ModelPlan.from_record(record, prediction, original.extra)
    options.update(validation_route='left', verify_signal=True, validation_allow_left_phase_cache=True)
    return route, context, options, plan


def test_phase_bridge_preserves_both_labels_native_arrays_anchor_and_age():
    route, context, options, plan = fixture()
    before = copy.deepcopy(plan.record)
    world, speeds = plan.world.copy(), plan.speeds.copy()
    metric = assess_retained(plan, context, route, 'green', 'finish_turn', options)
    assert metric['valid'] and metric['age_s'] == .4
    assert metric['phase_reuse']['source_phase'] == 'approach'
    assert metric['phase_reuse']['current_phase'] == 'finish_turn'
    assert plan.record == before
    np.testing.assert_array_equal(plan.world, world)
    np.testing.assert_array_equal(plan.speeds, speeds)
    options.pop('validation_allow_left_phase_cache')
    blocked = assess_retained(plan, context, route, 'green', 'finish_turn', options)
    assert not blocked['valid'] and 'navigation_phase_changed' in blocked['reasons']


@pytest.mark.parametrize('change', [
    'straight_route', 'source_route', 'no_vqa', 'red', 'yellow', 'unknown',
    'source_unknown', 'source_visual_mismatch', 'different_phase', 'reverse_phase',
    'source_already_turned', 'current_not_turned', 'source_outside', 'current_outside',
    'nonfinite', 'backwards_progress'])
def test_only_forward_same_left_maneuver_under_verified_green_is_eligible(change):
    route, context, options, plan = fixture()
    signal, phase = 'green', 'finish_turn'
    if change == 'straight_route': options['validation_route'] = 'straight'
    elif change == 'source_route': plan.record['selection']['route'] = 'straight'
    elif change == 'no_vqa': options['verify_signal'] = False
    elif change in ('red', 'yellow', 'unknown'): signal = change
    elif change == 'source_unknown': plan.record['selection']['signal'] = 'unknown'
    elif change == 'source_visual_mismatch': plan.record['visual_signal'] = 'unknown'
    elif change == 'different_phase': phase = 'exit'
    elif change == 'reverse_phase':
        plan.record['navigation_phase'], phase = 'finish_turn', 'approach'
    elif change == 'source_already_turned': plan.record['context']['world_pose']['yaw'] = 29.
    elif change == 'current_not_turned': context['world_pose']['yaw'] = 31.
    elif change == 'source_outside': plan.record['context']['world_pose']['y'] += 5.
    elif change == 'current_outside': context['world_pose']['y'] += 5.
    elif change == 'nonfinite': context['world_pose']['yaw'] = float('nan')
    elif change == 'backwards_progress': context['world_pose']['x'] -= 2.
    metric = assess_retained(plan, context, route, signal, phase, options)
    assert not metric['valid'] and 'phase_reuse' not in metric


@pytest.mark.parametrize('change,reason', [
    ('age', 'expired_or_nonpast_plan'), ('body', 'current_body_outside_lane'),
    ('reverse', 'reverse_motion')])
def test_phase_eligibility_cannot_override_physical_or_age_checks(change, reason):
    route, context, options, plan = fixture()
    if change == 'age': context['time_s'] = .6
    elif change == 'body': context['world_pose']['y'] += 1.2
    else: plan.record['candidate_velocities'][0] = [-3.] * 64
    metric = assess_retained(plan, context, route, 'green', 'finish_turn', options)
    assert 'phase_reuse' in metric
    assert not metric['valid'] and reason in metric['reasons']


def test_tracking_forecast_still_rejects_an_eligible_phase_bridge(monkeypatch):
    route, context, options, plan = fixture()
    metric = assess_retained(plan, context, route, 'green', 'finish_turn', options)
    assert metric['valid']
    options.update(validation_tracking_horizon_s=1.5, validation_tracking_gain=.85,
                   validation_tracking_margin_m=.1)
    monkeypatch.setattr(tracking_probe, 'forecast', lambda *a: [{'corner_clearance_m': .05, 'dt_s': .1}])
    screened = tracking_probe.screen(metric, context, route, plan.world, plan.speeds, .4, options, object())
    assert not screened['valid'] and 'insufficient_controller_tracking_clearance' in screened['reasons']


@pytest.mark.parametrize('tamper', ['missing_evidence', 'source_phase', 'disabled', 'relabel_current'])
def test_audit_requires_original_phases_enabled_rule_and_actual_decision_phase(tmp_path, tamper):
    route, context, options, plan = fixture()
    metric = assess_retained(plan, context, route, 'green', 'finish_turn', options)
    current = copy.deepcopy(plan.record)
    current.update(label='frame_004', context=context, navigation_phase='finish_turn',
                   retention={'source_label': plan.record['label'], 'age_s': .4,
                              'chosen_cache_index': 0, 'metrics': [metric]})
    for name, data in [('frame_000.json', plan.record), ('frame_004.json', current),
                       ('experiment.json', {'options': options}),
                       ('route-reference.json', {'routes': {'left': route}})]:
        (tmp_path/name).write_text(json.dumps(data))
    (tmp_path/'replan-rejections').mkdir()
    (tmp_path/'replan-rejections/frame_004.json').write_text(json.dumps({'navigation_phase': 'finish_turn'}))
    (tmp_path/'controller-inputs.jsonl').write_text(json.dumps({
        'time_s': .4, 'source_label': plan.record['label'], 'source_context': plan.record['context'],
        'age_s': .4, 'world_trajectory': plan.world.tolist(), 'speeds_mps': plan.speeds.tolist()})+'\n')
    assert audit(tmp_path)['all_sources_anchors_speeds_and_ages_match']
    if tamper == 'disabled':
        options.pop('validation_allow_left_phase_cache')
        (tmp_path/'experiment.json').write_text(json.dumps({'options': options}))
    elif tamper == 'source_phase': metric['phase_reuse']['source_phase'] = 'finish_turn'
    else:
        del metric['phase_reuse']
        if tamper == 'relabel_current': current['navigation_phase'] = 'approach'
    (tmp_path/'frame_004.json').write_text(json.dumps(current))
    assert not audit(tmp_path)['all_sources_anchors_speeds_and_ages_match']
