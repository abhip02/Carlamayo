"""Green-to-unknown reuse requires its own opt-in and both aligned exit states."""
import copy
import json

import numpy as np
import pytest

from local.experiments.audit_retention import audit
from local.experiments.exit_cache import unknown_exit_evidence
from local.experiments.retained_probe import assess_retained
from local.experiments.test_exit_cache import fixture as unknown_fixture


def fixture():
    route, context, options, plan = unknown_fixture()
    options.pop('validation_allow_unknown_exit_cache')
    options['validation_allow_green_unknown_exit_cache'] = True
    plan.record['selection']['signal'] = 'green'
    plan.record['visual_signal'] = 'green'
    return route, context, options, plan


def test_exit_transition_preserves_actual_labels_and_exact_native_arrays():
    route, context, options, plan = fixture()
    original = copy.deepcopy(plan.record)
    world, speeds = plan.world.copy(), plan.speeds.copy()
    metric = assess_retained(plan, context, route, 'unknown', 'finish_turn', options)
    assert metric['valid']
    assert metric['signal_reuse']['policy'] == 'green_to_unknown_after_left_turn_alignment'
    assert metric['signal_reuse']['source_signal'] == 'green'
    assert metric['signal_reuse']['current_signal'] == 'unknown'
    assert plan.record == original
    np.testing.assert_array_equal(plan.world, world)
    np.testing.assert_array_equal(plan.speeds, speeds)
    options.pop('validation_allow_green_unknown_exit_cache')
    options['validation_allow_unknown_exit_cache'] = True
    assert not assess_retained(plan, context, route, 'unknown', 'finish_turn', options)['valid']


@pytest.mark.parametrize('source,current', [
    ('unknown', 'unknown'), ('unknown', 'green'), ('red', 'unknown'),
    ('yellow', 'unknown'), ('green', 'red'), ('green', 'yellow')])
def test_transition_flag_does_not_enable_other_signal_pairs(source, current):
    route, context, options, plan = fixture()
    plan.record['selection']['signal'] = source
    metric = assess_retained(plan, context, route, current, 'finish_turn', options)
    assert not metric['valid'] and 'reuse_requires_green' in metric['reasons']


@pytest.mark.parametrize('change', ['disabled', 'no_vqa', 'straight_route', 'approach',
                                    'source_phase', 'source_heading', 'current_heading',
                                    'source_outside', 'current_outside', 'source_before_turn', 'nonfinite'])
def test_transition_requires_both_aligned_exit_states(change):
    route, context, options, plan = fixture()
    phase = 'finish_turn'
    if change == 'disabled': options.pop('validation_allow_green_unknown_exit_cache')
    elif change == 'no_vqa': options['verify_signal'] = False
    elif change == 'straight_route': options['validation_route'] = 'straight'
    elif change == 'approach': phase = 'approach'
    elif change == 'source_phase': plan.record['navigation_phase'] = 'approach'
    elif change == 'source_heading': plan.record['context']['world_pose']['yaw'] = 11.
    elif change == 'current_heading': context['world_pose']['yaw'] = 11.
    elif change == 'source_outside': plan.record['context']['world_pose']['y'] = 2.
    elif change == 'current_outside': context['world_pose']['y'] = 2.
    elif change == 'source_before_turn': plan.record['context']['world_pose'].update(x=-10., y=-5.)
    elif change == 'nonfinite': context['world_pose']['yaw'] = float('nan')
    assert unknown_exit_evidence(plan.record, context, route, 'unknown', phase, options) is None
    assert not assess_retained(plan, context, route, 'unknown', phase, options)['valid']


@pytest.mark.parametrize('change,reason', [('age','expired_or_nonpast_plan'),('body','current_body_outside_lane')])
def test_transition_cannot_override_physical_or_age_rejection(change, reason):
    route, context, options, plan = fixture()
    if change == 'age': context['time_s'] = .6
    else: context['world_pose']['y'] = 1.
    metric = assess_retained(plan, context, route, 'unknown', 'finish_turn', options)
    assert 'signal_reuse' in metric
    assert not metric['valid'] and reason in metric['reasons']


def test_green_pair_does_not_use_transition_policy():
    route, context, options, plan = fixture()
    with_flag = assess_retained(plan, context, route, 'green', 'finish_turn', options)
    options.pop('validation_allow_green_unknown_exit_cache')
    assert with_flag == assess_retained(plan, context, route, 'green', 'finish_turn', options)
    assert with_flag['valid'] and 'signal_reuse' not in with_flag


def test_retention_audit_checks_both_actual_signal_labels_and_transition_flag(tmp_path):
    route, context, options, plan = fixture()
    metric = assess_retained(plan, context, route, 'unknown', 'finish_turn', options)
    metric['candidate'] = 0
    current = copy.deepcopy(plan.record)
    current.update(label='frame_004', context=context, visual_signal='unknown',
                   retention={'source_label': plan.record['label'], 'age_s': .4,
                              'chosen_cache_index': 0, 'metrics': [metric]})
    current['selection']['signal'] = 'unknown'
    for name, data in [('frame_000.json', plan.record), ('frame_004.json', current),
                       ('experiment.json', {'options': options}),
                       ('route-reference.json', {'routes': {'left': route}})]:
        (tmp_path / name).write_text(json.dumps(data))
    (tmp_path / 'replan-rejections').mkdir()
    (tmp_path / 'replan-rejections/frame_004.json').write_text('{"navigation_phase":"finish_turn"}')
    (tmp_path / 'controller-inputs.jsonl').write_text(json.dumps({
        'time_s': .4, 'source_label': plan.record['label'], 'source_context': plan.record['context'],
        'age_s': .4, 'world_trajectory': plan.world.tolist(), 'speeds_mps': plan.speeds.tolist()}) + '\n')
    assert audit(tmp_path)['all_sources_anchors_speeds_and_ages_match']
    current['retention']['metrics'][0]['signal_reuse']['source_signal'] = 'unknown'
    (tmp_path / 'frame_004.json').write_text(json.dumps(current))
    assert not audit(tmp_path)['all_sources_anchors_speeds_and_ages_match']
    current['retention']['metrics'][0]['signal_reuse']['source_signal'] = 'green'
    (tmp_path / 'frame_004.json').write_text(json.dumps(current))
    options.pop('validation_allow_green_unknown_exit_cache')
    (tmp_path / 'experiment.json').write_text(json.dumps({'options': options}))
    assert not audit(tmp_path)['all_sources_anchors_speeds_and_ages_match']

    # A relabeled selection plus missing eligibility evidence must not make
    # this unknown-current step appear to be an ordinary green/green reuse.
    options['validation_allow_green_unknown_exit_cache'] = True
    (tmp_path / 'experiment.json').write_text(json.dumps({'options': options}))
    current['selection']['signal'] = 'green'
    del current['retention']['metrics'][0]['signal_reuse']
    (tmp_path / 'frame_004.json').write_text(json.dumps(current))
    result = audit(tmp_path)
    assert not result['all_sources_anchors_speeds_and_ages_match']
    assert not result['mismatches'][0]['checks']['current_signal_classification_matches']
