"""A replayed approach must never be mistaken for fresh model inference."""
import copy
import json
from pathlib import Path

import numpy as np
import pytest
import torch

from local.experiments import prefix_resume_probe as prefix
from local.experiments.test_candidate_reserve_probe import fixture as reserve_fixture


def fixture(tmp_path, monkeypatch):
    source, output = tmp_path / 'source', tmp_path / 'output'
    source.mkdir(); output.mkdir()
    base, adapter, _, options, proposal, fresh, _ = reserve_fixture(source)
    fresh.update({k: copy.deepcopy(v) for k, v in proposal.items() if k not in ('label', 'context')})
    fresh.update(trajectory_ego=proposal['candidate_trajectories'][0], generated_token_ids=[10, 0],
                 visual_signal_answer='green', visual_signal_check_index=0)
    fresh['selection']['signal'] = 'green'
    prefix.native.write_json(source / 'accepted_frame_000.json', fresh)
    prefix.native.write_json(source / 'frame_000.json', {**fresh, 'label': 'frame_000'})
    plan = prefix.load_plan(source, 'accepted_frame_000')
    context = copy.deepcopy(fresh['context'])
    first = {**context['world_pose'], 'speed': context['measured_speed_mps']}
    last = {**first, 'x': first['x'] + .3}
    controls = [{'time_s': 0., 'source_label': fresh['label'], 'age_s': 0.,
                 'world_trajectory': plan.world.tolist(), 'speeds_mps': plan.speeds.tolist(),
                 'control': {'steer': 0., 'throttle': .3, 'brake': 0.}}]
    monkeypatch.setattr(prefix, 'read_capture', lambda path: ([first, last], controls))
    for target in (source, output):
        prefix.native.write_json(target / 'config.json', {'duration_s': 2, 'seed': 42})
    prefix.native.write_json(source / 'experiment.json', {'options': options})
    prefix.native.write_json(source / 'route-reference.json', {'routes': {'straight': base.validation_route}})
    for name in ('module/config.py', 'module/trajectory_follower.py'):
        p = source / 'source_snapshot' / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes((prefix.native.runner.ROOT / name).read_bytes())
    probe = object.__new__(prefix.PrefixResumeProbe)
    probe.__dict__.update(base.__dict__)
    probe.output = output
    probe.prefix_ready = probe.prefix_verified = False
    probe.prefix_index = None
    probe.prefix_errors, probe.prefix_control_errors = [], []
    options = {**options, 'replay_prefix_capture': str(source), 'replay_prefix_seconds': .1}
    return probe, adapter, context, options, source, last


def test_prefix_preserves_native_arrays_and_calls_model_only_after_verified_boundary(tmp_path, monkeypatch):
    probe, adapter, context, options, source, last = fixture(tmp_path, monkeypatch)
    calls = []
    sentinel = object()
    monkeypatch.setattr(prefix.reserve.CandidateReserveProbe, 'evaluate', lambda *a, **k: calls.append(a) or sentinel)
    prediction, extra = probe.evaluate(adapter, {}, context, 'Go straight', options, 'frame_000')
    assert not calls
    assert torch.equal(prediction, probe.prefix_sources['accepted_frame_000'].prediction)
    assert extra['cot'][0, 0, 0] == 'first native intent'
    assert probe.executing_age == 0
    assert len(probe.history) == 1 and len(probe.candidate_reserve) == 1
    archived = json.loads((probe.output / 'frame_000.json').read_text())
    assert archived['prefix_replay']['fresh_inference'] is False
    assert archived['context'] == json.loads((source / 'frame_000.json').read_text())['context']
    assert archived['prefix_replay']['actual_context'] == context
    probe.verify_control((0., .3, 0., {}))
    context = {**context, 'index': 1, 'time_s': .1,
               'world_pose': {k: v for k, v in last.items() if k != 'speed'}}
    assert probe.evaluate(adapter, {}, context, 'Go straight', options, 'frame_001') is sentinel
    assert len(calls) == 1 and probe.prefix_verified
    assert probe.visual_signal_index == -1000  # Recheck live images on resume.
    manifest = json.loads((probe.output / 'prefix-verified.json').read_text())
    assert manifest['fresh_inference_begins_at_s'] == .1
    assert max(manifest['control_errors']) == 0


@pytest.mark.parametrize('change', ['pose', 'control', 'incomplete', 'route', 'configuration', 'source_array'])
def test_prefix_rejects_mismatch_before_fresh_inference(tmp_path, monkeypatch, change):
    probe, adapter, context, options, source, _ = fixture(tmp_path, monkeypatch)
    def model(*a, **k):
        pytest.fail('Fresh inference must not run on a mismatched prefix')
    monkeypatch.setattr(prefix.reserve.CandidateReserveProbe, 'evaluate', model)
    if change == 'pose':
        context['world_pose']['x'] += .01
    elif change == 'incomplete':
        options['replay_prefix_seconds'] = .2
    elif change == 'route':
        options['validation_route'] = 'left'
    elif change == 'configuration':
        prefix.native.write_json(probe.output / 'config.json', {'duration_s': 2, 'seed': 99})
    elif change == 'source_array':
        p = source / 'accepted_frame_000.json';r = json.loads(p.read_text())
        r['trajectory_ego'][0][0] += 1
        prefix.native.write_json(p, r)
    if change == 'control':
        probe.evaluate(adapter, {}, context, 'Go straight', options, 'frame_000')
        with pytest.raises(RuntimeError, match='controller mismatch'):
            probe.verify_control((0., .31, 0., {}))
    else:
        with pytest.raises((RuntimeError, ValueError)):
            probe.evaluate(adapter, {}, context, 'Go straight', options, 'frame_000')
    assert not (probe.output / 'prefix-verified.json').exists()


def test_verify_only_stops_without_fresh_inference_or_route_completion(tmp_path, monkeypatch):
    probe, adapter, context, options, _, last = fixture(tmp_path, monkeypatch)
    options['replay_prefix_only'] = True
    probe.evaluate(adapter, {}, context, 'Go straight', options, 'frame_000')
    probe.verify_control((0., .3, 0., {}))
    context = {**context, 'index': 1, 'time_s': .1,
               'world_pose': {k: v for k, v in last.items() if k != 'speed'}}
    with pytest.raises(prefix.PrefixComplete):
        probe.evaluate(adapter, {}, context, 'Go straight', options, 'frame_001')
    assert not (probe.output / 'route-exit-reached.json').exists()
    assert json.loads((probe.output / 'prefix-verified.json').read_text())['fresh_inference_begins_at_s'] is None


def test_entrypoint_restores_process_wrappers_on_exception(monkeypatch):
    original = prefix.reserve.CandidateReserveProbe
    follower = prefix.native.runner.TrajectoryFollower
    def fail():
        assert prefix.reserve.CandidateReserveProbe is prefix.PrefixResumeProbe
        raise RuntimeError('startup failed')
    monkeypatch.setattr(prefix.reserve, 'main', fail)
    with pytest.raises(RuntimeError, match='startup failed'):
        prefix.main()
    assert prefix.reserve.CandidateReserveProbe is original
    assert prefix.native.runner.TrajectoryFollower is follower
    assert prefix.ACTIVE is None


def test_capture_audit_uses_replay_snapshot_and_does_not_call_verify_only_completion(tmp_path):
    import csv
    from local.experiments.audit_capture import audit
    (tmp_path / '000_verifai').mkdir()
    path = [[[.3, 0., 0.]]]
    original = {'time_s': 0., 'snapshot_frame': 42, 'world_pose': {'x': 0., 'y': 0.}}
    actual = {**original, 'snapshot_frame': 101}
    frame = {'context': original, 'prefix_replay': {'actual_context': actual, 'fresh_inference': False},
             'trajectory_ego': path[0], 'candidate_trajectories': path, 'selection': {'chosen': 0},
             'trajectory_marker_present': True, 'visual_signal_check_index': 0, 'visual_signal': 'green'}
    (tmp_path / 'frame_000.json').write_text(json.dumps(frame))
    (tmp_path / '000_verifai/predictions.jsonl').write_text(json.dumps({'time_s': 0., 'trajectory_ego': path[0]})+'\n')
    with (tmp_path / '000_verifai/ego.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=['time_s','frame','x','y','yaw','speed'])
        writer.writeheader()
        writer.writerow({'time_s': 0., 'frame': 101, 'x': 0., 'y': 0., 'yaw': 0., 'speed': 3.})
        writer.writerow({'time_s': .1, 'frame': 102, 'x': .3, 'y': 0., 'yaw': 0., 'speed': 3.})
    (tmp_path / '000_verifai/summary.json').write_text(json.dumps({'termination': 'prefix_verification_complete'}))
    (tmp_path / 'batch-summary.json').write_text('[]')
    result = audit(tmp_path)
    assert not result['snapshot_vs_csv_frame_mismatch_times_s']
    assert result['replayed_prefix_records'] == 1 and result['fresh_inference_records'] == 0
    assert result['archived_prefix_visual_check_counts'] == {'green': 1}
    assert result['fresh_visual_check_counts'] == {}
    assert not result['completed'] and not result['route_exit_reached']
