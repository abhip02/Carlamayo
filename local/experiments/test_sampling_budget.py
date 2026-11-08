import json
from types import SimpleNamespace

import pytest

from local.experiments import projected_speed_selection as selection
from local.experiments.projected_probe import SingleSampleProbe
from local.experiments.test_projected_speed_selection import candidate, soft_cache


def prepare(tmp_path, monkeypatch, proposals, **overrides):
    elapsed, calls = [0.], []
    probe = object.__new__(selection.SpeedSelectedProbe)
    probe.output = tmp_path
    monkeypatch.setattr(selection, 'time', SimpleNamespace(monotonic=lambda: elapsed[0]))

    def sample(self, adapter, data, context, instruction, options, label, *args):
        item, duration = proposals[len(calls)]
        item = {**item, 'label': label, 'context': context, 'inference_seconds': duration}
        calls.append(item)
        elapsed[0] += duration
        (tmp_path / (label + '.json')).write_text(json.dumps(item))
        return item['trajectory_ego'], {'label': label}

    monkeypatch.setattr(SingleSampleProbe, 'sample_plan', sample)
    options = dict(seed=44, flow_seed=43, speed_selection_attempts=6, **overrides)

    def run():
        result = probe.sample_plan(None, None, {'index': 0, 'signal_states': {'1': 'Green'}},
                                   'Go', options, 'frame_000')
        return result, json.loads((tmp_path / 'frame_000.json').read_text())

    return run, calls


def test_time_budget_stops_retries_but_keeps_best_unmodified_prediction(tmp_path, monkeypatch):
    run, calls = prepare(tmp_path, monkeypatch,
        [(candidate(.1), 2.), (candidate(2.8), 2.), (candidate(4.), 2.)], speed_selection_max_seconds=3.)
    result, record = run()
    assert len(calls) == 2
    assert result[0] == record['trajectory_ego'] == calls[1]['trajectory_ego']
    assert record['selection']['stop_reason'] == 'time_budget'
    assert record['selection']['elapsed_seconds'] == record['inference_seconds'] == 4.
    assert not record['selection']['preference_met']
    assert [p['start_elapsed_seconds'] for p in record['selection']['proposals']] == [0., 2.]


def test_budget_does_not_execute_an_invalid_path_or_start_another_call(tmp_path, monkeypatch):
    bad = {**candidate(4.), 'reverse_velocity_steps': 1}
    run, calls = prepare(tmp_path, monkeypatch, [(bad, 5.), (candidate(4.), 1.)],
                         speed_selection_max_seconds=3.)
    with pytest.raises(RuntimeError, match='No finite forward native trajectory'):
        run()
    assert len(calls) == 1
    assert not (tmp_path / 'frame_000.json').exists()


@pytest.mark.parametrize('signal', ['red', 'yellow'])
def test_time_budget_never_replaces_visual_stop_with_a_moving_plan(tmp_path, monkeypatch, signal):
    monkeypatch.setattr(selection.SpeedSelectedProbe, 'consider_retained', soft_cache)
    run, calls = prepare(tmp_path, monkeypatch, [(candidate(.1, signal), 4.), (candidate(4.), 1.)],
                         speed_selection_max_seconds=.5)
    _, record = run()
    assert len(calls) == 1 and record['controller_preview_speed_mps'] == .1
    assert 'retention' not in record
    assert record['selection']['stop_reason'] == 'visual_stop'


def test_budget_can_choose_already_validated_retained_native_plan(tmp_path, monkeypatch):
    monkeypatch.setattr(selection.SpeedSelectedProbe, 'consider_retained', soft_cache)
    run, calls = prepare(tmp_path, monkeypatch, [(candidate(.1), 4.), (candidate(4.), 1.)],
                         speed_selection_max_seconds=3.)
    _, record = run()
    assert len(calls) == 1
    assert record['selection']['selected_proposal'] == 'proposal_old'
    assert record['controller_preview_speed_mps'] == 2.8
    assert record['selection']['stop_reason'] == 'time_budget'


def test_lower_preference_accepts_native_speed_without_editing_it(tmp_path, monkeypatch):
    run, calls = prepare(tmp_path, monkeypatch, [(candidate(2.8), 2.), (candidate(4.), 2.)],
                         speed_preference_mps=[2.5, 5.])
    result, record = run()
    assert len(calls) == 1 and record['selection']['preference_met']
    assert result[0] == calls[0]['trajectory_ego'] == record['trajectory_ego']


def test_default_preserves_original_six_attempt_budget(tmp_path, monkeypatch):
    run, calls = prepare(tmp_path, monkeypatch, [(candidate(2.), 2.)] * 6)
    _, record = run()
    assert len(calls) == 6 and record['inference_seconds'] == 12.
    assert record['selection']['stop_reason'] == 'attempt_budget'
    assert record['selection']['max_seconds'] is None


@pytest.mark.parametrize('budget', [0., -1., float('inf'), float('nan')])
def test_invalid_budget_is_rejected_before_inference(tmp_path, monkeypatch, budget):
    run, calls = prepare(tmp_path, monkeypatch, [], speed_selection_max_seconds=budget)
    with pytest.raises(ValueError, match='positive and finite'):
        run()
    assert not calls


def test_independent_audit_distinguishes_call_overrun_from_illegal_retry():
    from local.experiments.audit_projection import sampling_budget_checks
    options = {'speed_selection_max_seconds': 3.}
    record = dict(max_seconds=3., elapsed_seconds=5.,
                  proposals=[{'start_elapsed_seconds': 0.}, {'start_elapsed_seconds': 2.}])
    assert all(sampling_budget_checks(record, options).values())
    record['proposals'].append({'start_elapsed_seconds': 4.})
    assert not sampling_budget_checks(record, options)['no_retry_after_sampling_budget']


def test_independent_audit_rejects_missing_timing_and_mismatched_budget():
    from local.experiments.audit_projection import sampling_budget_checks
    record = dict(max_seconds=4., proposals=[{}])
    checks = sampling_budget_checks(record, {'speed_selection_max_seconds': 3.})
    assert not checks['sampling_time_budget_config']
    assert not checks['sampling_retry_time_records']
    assert not checks['sampling_elapsed_record']


@pytest.mark.parametrize('flags', [
    ['--speed-selection-max-seconds', '-1'],
    ['--speed-preference-mps', '3', 'nan'],
    ['--speed-selection-max-seconds', '3'],  # Default single-sample mode has no retries.
])
def test_invalid_cli_configuration_fails_before_creating_output(tmp_path, monkeypatch, flags):
    from local.experiments import run_projection_suite
    output = tmp_path / 'not-created'
    monkeypatch.setattr('sys.argv', ['suite', '--output', str(output), *flags])
    with pytest.raises(SystemExit) as exc:
        run_projection_suite.main()
    assert exc.value.code == 2
    assert not output.exists()
