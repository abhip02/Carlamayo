"""Unknown straight-junction reuse is bounded, explicit and physically checked."""
import copy
import json

import numpy as np
import pytest

from local.experiments.exit_cache import unknown_exit_evidence
from local.experiments.retained_probe import assess_retained
from local.experiments.test_retained_probe import setup_plan


def fixture(source_signal='unknown'):
    route, context, options, plan, _ = setup_plan()
    entry = {'x': -10., 'y': 0., 'yaw': 0., 'lane_width': 3.5}
    route.update(points=[entry, *route['points']], entry=entry, exit_index=2)
    options.update(verify_signal=True, validation_allow_straight_unknown_cache=True)
    plan.record['selection']['signal'] = source_signal
    plan.record['visual_signal'] = source_signal
    return route, context, options, plan


@pytest.mark.parametrize('source_signal', ['green', 'unknown'])
def test_valid_native_plan_retains_its_labels_arrays_anchor_and_age(source_signal):
    route, context, options, plan = fixture(source_signal)
    original = copy.deepcopy(plan.record)
    world, speeds = plan.world.copy(), plan.speeds.copy()
    metric = assess_retained(plan, context, route, 'unknown', 'approach', options)
    assert metric['valid'] and metric['age_s'] == .4
    evidence = metric['signal_reuse']
    assert evidence['source_signal'] == source_signal and evidence['current_signal'] == 'unknown'
    assert all(s['rear_entry_clearance_m'] > 0 for s in evidence['states'])
    assert plan.record == original
    np.testing.assert_array_equal(world, plan.world); np.testing.assert_array_equal(speeds, plan.speeds)


@pytest.mark.parametrize('change', ['disabled', 'no_vqa', 'wrong_route', 'source_route', 'curving_route',
    'phase', 'source_phase', 'source_label', 'source_heading', 'heading', 'source_before_entry',
    'before_entry', 'rear_before_entry', 'after_exit', 'backward_progress', 'outside', 'nan', 'no_bounds'])
def test_scope_excludes_unentered_junctions_and_changed_maneuvers(change):
    route, context, options, plan = fixture()
    phase = 'approach'
    if change == 'disabled': options.pop('validation_allow_straight_unknown_cache')
    elif change == 'no_vqa': options['verify_signal'] = False
    elif change == 'wrong_route': options['validation_route'] = 'left'
    elif change == 'source_route': plan.record['selection']['route'] = 'left'
    elif change == 'curving_route': route['exit']['yaw'] = 90.
    elif change == 'phase': phase = 'finish_turn'
    elif change == 'source_phase': plan.record['navigation_phase'] = 'finish_turn'
    elif change == 'source_label': plan.record['visual_signal'] = 'green'
    elif change == 'source_heading': plan.record['context']['world_pose']['yaw'] = 11.
    elif change == 'heading': context['world_pose']['yaw'] = 11.
    elif change == 'source_before_entry': plan.record['context']['world_pose']['x'] = -12.
    elif change == 'before_entry': context['world_pose']['x'] = -12.
    elif change == 'rear_before_entry': route['entry'] = route['points'][1]
    elif change == 'after_exit': context['world_pose']['x'] = 51.
    elif change == 'backward_progress': context['world_pose']['x'] = plan.record['context']['world_pose']['x'] - .1
    elif change == 'outside': context['world_pose']['y'] = 2.
    elif change == 'nan': context['world_pose']['yaw'] = float('nan')
    elif change == 'no_bounds': context.pop('vehicle_bounds')
    assert unknown_exit_evidence(plan.record, context, route, 'unknown', phase, options) is None
    assert not assess_retained(plan, context, route, 'unknown', phase, options)['valid']


@pytest.mark.parametrize('source,current', [('red','unknown'),('yellow','unknown'),('unknown','red'),
    ('unknown','yellow'),('unknown','green'),('green','red')])
def test_other_signal_pairs_do_not_gain_reuse(source, current):
    route, context, options, plan = fixture(source)
    assert not assess_retained(plan, context, route, current, 'approach', options)['valid']


@pytest.mark.parametrize('change,reason', [('age','expired_or_nonpast_plan'),('body','current_body_outside_lane')])
def test_eligibility_does_not_override_time_or_physical_rejection(change, reason):
    route, context, options, plan = fixture()
    if change == 'age': context['time_s'] = .6
    else: context['world_pose']['y'] = 1.
    metric = assess_retained(plan, context, route, 'unknown', 'approach', options)
    assert 'signal_reuse' in metric and not metric['valid'] and reason in metric['reasons']


@pytest.mark.parametrize('source_signal', ['green', 'unknown'])
def test_audit_checks_classification_and_entry_clearance_evidence(tmp_path, source_signal):
    from local.experiments.audit_retention import audit
    route, context, options, plan = fixture(source_signal)
    metric = dict(assess_retained(plan, context, route, 'unknown', 'approach', options), candidate=0)
    current = copy.deepcopy(plan.record)
    current.update(label='frame_004', context=context, visual_signal='unknown',
        retention={'source_label': plan.record['label'], 'age_s': .4, 'chosen_cache_index': 0, 'metrics': [metric]})
    current['selection']['signal'] = 'unknown'
    for name, data in [('frame_000.json', plan.record), ('frame_004.json', current),
                      ('experiment.json', {'options': options}), ('route-reference.json', {'routes': {'straight': route}})]:
        (tmp_path / name).write_text(json.dumps(data))
    (tmp_path / 'replan-rejections').mkdir()
    (tmp_path / 'replan-rejections/frame_004.json').write_text('{"navigation_phase":"approach"}')
    (tmp_path / 'controller-inputs.jsonl').write_text(json.dumps({'time_s': .4,
        'source_label': plan.record['label'], 'source_context': plan.record['context'], 'age_s': .4,
        'world_trajectory': plan.world.tolist(), 'speeds_mps': plan.speeds.tolist()}) + '\n')
    assert audit(tmp_path)['all_sources_anchors_speeds_and_ages_match']
    current['retention']['metrics'][0]['signal_reuse']['states'][0]['rear_entry_clearance_m'] += 1
    (tmp_path / 'frame_004.json').write_text(json.dumps(current))
    assert not audit(tmp_path)['all_sources_anchors_speeds_and_ages_match']
