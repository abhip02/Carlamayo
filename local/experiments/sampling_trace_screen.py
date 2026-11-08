"""Screen sampling policies on recorded states; this is NOT a driving rollout.

Replays the actual selector and retention checks, using only saved model outputs.
The cache contains only proposals requested by the screened policy. If a needed
proposal was never captured, stop that case rather than inventing a result.
Subsequent images and ego states are still those from the historical rollout:
changed decisions do not generate their counterfactual physics or observations.
"""
import argparse
from contextlib import redirect_stdout
import hashlib
import json
import os
from pathlib import Path
import statistics
from tempfile import TemporaryDirectory
from types import SimpleNamespace

import numpy as np

from local.experiments import projected_speed_selection as selection
from local.experiments.projected_probe import SingleSampleProbe
from local.experiments.projected_retention import RetainedSpeedProbe


class MissingSavedProposal(Exception):
    pass


def distribution(values):
    if not values:
        return None
    return dict(total=sum(values), median=statistics.median(values),
                p95=float(np.percentile(values, 95)), maximum=max(values))


def screen(capture, overrides):
    frames = [json.loads(p.read_text()) for p in sorted(capture.glob('frame_*.json'))]
    options = {**json.loads((capture / 'experiment.json').read_text())['options'], **overrides}
    route = json.loads((capture / 'route-reference.json').read_text())['routes'][options['validation_route']]
    probe = object.__new__(RetainedSpeedProbe)
    probe.history, probe.route = [], route
    probe.execution_record, probe.execution_world, probe.execution_age = None, None, 0.
    elapsed = [0.]
    used = []
    steps = []
    missing = None
    original_sample, original_time = SingleSampleProbe.sample_plan, selection.time

    def saved_sample(self, adapter, data, context, instruction, candidate_options, label, *args):
        path = capture / (label + '.json')
        if not path.exists():
            raise MissingSavedProposal(label)
        record = json.loads(path.read_text())
        # New policy changes selection only, not the native generation inputs.
        assert record['context'] == context
        for key in ('seed', 'flow_seed'):
            assert record['options'][key] == candidate_options[key]
        assert record['instruction'] == instruction
        elapsed[0] += record['inference_seconds']
        used.append(record)
        (self.output / (label + '.json')).write_text(json.dumps(record))
        return np.asarray(record['trajectory_ego']), {'saved_proposal': label}

    try:
        SingleSampleProbe.sample_plan = saved_sample
        # Reconstruct model-call time; excludes filesystem/selector overhead.
        selection.time = SimpleNamespace(monotonic=lambda: elapsed[0])
        with TemporaryDirectory(prefix='carlamayo-sampling-screen-') as temporary, open(os.devnull, 'w') as quiet:
            probe.output = Path(temporary)
            for original in frames:
                before = len(used)
                # A retained frame keeps its SOURCE instruction. The current
                # decision's navigation text is in its fresh first proposal.
                first = capture / f'proposal_{original["context"]["index"]:03d}_00.json'
                instruction = json.loads(first.read_text())['instruction']
                try:
                    with redirect_stdout(quiet):
                        probe.sample_plan(None, None, original['context'], instruction,
                                          {**options, 'instruction': instruction}, original['label'])
                except MissingSavedProposal as exc:
                    missing = str(exc)
                    break
                record = json.loads((probe.output / (original['label'] + '.json')).read_text())
                picked = record['selection']['selected_proposal']
                same = (picked == original['selection']['selected_proposal']
                        and record['trajectory_ego'] == original['trajectory_ego']
                        and record.get('retention', {}).get('source_context', record['context'])
                        == original.get('retention', {}).get('source_context', original['context']))
                preview = record.get('retention', {}).get('checks', {}).get(
                    'target_speed_mps', record['controller_preview_speed_mps'])
                steps.append(dict(index=original['context']['index'], attempts=len(used)-before,
                    selected_proposal=picked, same_as_recorded=same,
                    selected_preview_mps=preview, retained='retention' in record,
                    inference_seconds=sum(r['inference_seconds'] for r in used[before:]),
                    preference_met=record['selection'].get('preference_met'),
                    stop_reason=record['selection'].get('stop_reason', 'single_native_stop_scene')))
    finally:
        SingleSampleProbe.sample_plan, selection.time = original_sample, original_time

    return dict(overrides=overrides, recorded_steps=len(frames), covered_steps=len(steps),
        complete_saved_trace=missing is None, missing_proposal=missing,
        requested_saved_calls=len(used), changed_decisions=sum(not s['same_as_recorded'] for s in steps),
        inference_seconds=distribution([s['inference_seconds'] for s in steps]),
        selected_preview_below_1=sum(s['selected_preview_mps'] < 1 for s in steps),
        steps=steps)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--collection', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cases', type=int, nargs='+')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = json.loads(args.collection.read_text())
    result = dict(kind='fixed_recorded_state_policy_screen',
        warning='NOT closed-loop validation or measured hardware acceleration. Changed decisions do not alter saved future states/images. Missing proposals stop coverage. Time is the sum of recorded native-call times, excluding policy/I/O overhead.',
        collection=str(args.collection.resolve()),
        collection_sha256=hashlib.sha256(args.collection.read_bytes()).hexdigest(), results=[])
    policies = {'baseline': {}, 'two_attempts': {'speed_selection_attempts': 2},
                'two_attempts_band25': {'speed_selection_attempts': 2, 'speed_preference_mps': [2.5, 5.]}}
    for number, case in enumerate(manifest['results'], 1):
        if args.cases and number not in args.cases:
            continue
        capture = args.source_root / case['capture']
        item = dict(name=case['name'], capture=str(capture.resolve()), policies={})
        for name, overrides in policies.items():
            screened = screen(capture, overrides)
            if name == 'baseline':
                assert screened['complete_saved_trace'] and screened['changed_decisions'] == 0
            item['policies'][name] = screened
            print(case['name'], name, json.dumps({k: v for k, v in screened.items() if k != 'steps'}), flush=True)
            (args.output / (case['name'] + '.json')).write_text(json.dumps(item, indent=2) + '\n')
        result['results'].append(item)
        (args.output / 'report.json').write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
