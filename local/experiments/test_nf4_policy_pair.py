import json

import pytest

from local.experiments.nf4_policy_pair import POLICY, validate_candidate, verify_checkpoint, verify_devices, sha256
from local.experiments.nf4_closed_loop import validate_controller
from local.experiments.report_nf4_policy_pair import criteria_pass, outcome


def baseline(red=False):
    return dict(steering_mode='route', num_samples=1, steering_gain=1.4,
        controller_speed_cap_mps=4.5, speed_selection_attempts=6, speed_preference_mps=[3.,5.],
        projection_cache_age_s=3., verify_signal=True, signal_check_interval=5,
        guard_trajectory_format=True, acknowledged_preroll=True, acknowledged_controls=True,
        wait_for_snapshot=True, hold_visual_stop=red, scene_policy='red_light_stop' if red else 'empty_green',
        precision='nf4', model_placement='single_gpu', seed=52, flow_seed=51)


@pytest.mark.parametrize('red', [False, True])
def test_only_explicit_sampling_changes_are_allowed_and_original_guard_stays_strict(red):
    original = baseline(red)
    candidate = {**original, **POLICY}
    validate_candidate(candidate, original, validate_controller)
    with pytest.raises(ValueError, match='Frozen controller mismatch'):
        validate_controller(candidate)
    assert original['speed_selection_attempts'] == 6


@pytest.mark.parametrize('change', [{'seed':53}, {'flow_seed':52}, {'steering_gain':1.5},
    {'precision':'bf16'}, {'speed_selection_attempts':3}, {'hold_visual_stop':True},
    {'speed_selection_max_seconds':3}, {'max_generation_length':32}])
def test_unregistered_controller_generation_precision_or_budget_changes_rejected(change):
    original = baseline()
    with pytest.raises(ValueError, match='only attempts'):
        validate_candidate({**original, **POLICY, **change}, original, validate_controller)


def test_checkpoint_bytes_and_inventory_are_verified(tmp_path):
    file = tmp_path/'weights.safetensors'
    file.write_bytes(b'registered-checkpoint')
    expected = {file.name:sha256(file)}
    verify_checkpoint(tmp_path, expected)
    file.write_bytes(b'modified-checkpoint')
    with pytest.raises(ValueError, match='checkpoint changed'):
        verify_checkpoint(tmp_path, expected)
    (tmp_path/'extra.json').write_text('{}')
    with pytest.raises(ValueError, match='inventory'):
        verify_checkpoint(tmp_path, expected)


def test_integrity_pass_alone_is_not_a_driving_pass():
    audit = dict(integrity_pass=True, body_departure_times_s=[], min_speed_mps=.5,
                 max_speed_mps=4., reached_exit_time_s=10.)
    summary = {'termination':'route_exit_reached'}
    assert not criteria_pass(audit, summary, False)
    audit['min_speed_mps']=2.
    assert criteria_pass(audit, summary, False)
    assert not criteria_pass(audit, {'termination':'collision'}, False)


def test_timeout_and_unattempted_job_stay_in_denominator(tmp_path):
    capture=tmp_path/'09-t-junction-nf4-cap2';capture.mkdir()
    (tmp_path/'nf4-status.json').write_text(json.dumps({'results':[
        {'name':capture.name,'returncode':124,'timed_out':True}]}))
    result=outcome(capture,capture.name)
    assert result['state']=='timeout' and not result['passed']
    missing=outcome(tmp_path/'10-red-light-stop-nf4-cap2','10-red-light-stop-nf4-cap2')
    assert missing['state']=='not_attempted' and not missing['passed']
    blocked=tmp_path/'10-red-light-stop-nf4';blocked.mkdir()
    (blocked/'UNATTEMPTED_RESOURCE_GUARD.json').write_text('{"reason":"renderer unavailable"}')
    result=outcome(blocked,blocked.name)
    assert result['state']=='unattempted_resource_blocked' and not result['passed']


def test_device_assignment_cannot_silently_differ_from_prospective_baseline():
    registration={'model_gpu':'model','renderer_gpu':'renderer'}
    environment=dict(CUDA_VISIBLE_DEVICES='model',CARLAMAYO_MODEL_GPUS='model',CARLAMAYO_RENDER_GPU='renderer')
    verify_devices(registration,environment)
    with pytest.raises(ValueError,match='Renderer GPU'):
        verify_devices(registration,{**environment,'CARLAMAYO_RENDER_GPU':'other'})
    with pytest.raises(ValueError,match='Model GPU'):
        verify_devices(registration,{**environment,'CUDA_VISIBLE_DEVICES':'other'})
