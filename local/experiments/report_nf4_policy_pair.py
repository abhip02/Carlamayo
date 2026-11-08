"""Report every prospective NF4 policy outcome, with explicit pairing checks."""
import argparse
import csv
import json
from pathlib import Path
import statistics

from local.experiments.nf4_policy_pair import read, write, verify_pairing, sha256

PRECISION_KEYS = ('precision', 'quantized_layers', 'excluded_linear_layers',
    'quantized_original_weight_elements', 'quantization_config', 'compressed_weight_storage',
    'compute_dtype', 'activation_quantization', 'cpu_offload', 'device_map',
    'model_placement', 'input_device', 'bridge_source_sha256', 'checkpoint')
CORE_SOURCES = ('module/config.py', 'module/trajectory_follower.py', 'module/adapters/alpamayo_1_5.py',
    'local/experiments/inference_probe.py', 'local/experiments/projected_probe.py',
    'local/experiments/projected_steering.py', 'local/experiments/projected_speed_selection.py',
    'local/experiments/projected_retention.py')


def percentiles(values):
    if not values:
        return None
    ordered = sorted(values)
    x = .95 * (len(ordered) - 1)
    i = int(x)
    p95 = ordered[i] if i == len(ordered)-1 else ordered[i] + (x-i)*(ordered[i+1]-ordered[i])
    return dict(p50=statistics.median(ordered), p95=p95, maximum=max(ordered))


def criteria_pass(audit, summary, red):
    common = (audit.get('integrity_pass') is True and audit.get('body_departure_times_s') == []
              and summary.get('termination') in ('duration_reached', 'route_exit_reached')
              and not summary.get('collision') and audit.get('max_speed_mps', float('inf')) <= 5.5)
    if red:
        mission = audit.get('mission', {})
        return bool(common and mission.get('passed') is True
            and mission.get('minimum_front_gap_m', -1) >= 0
            and 0 <= mission.get('final_front_gap_m', -1) <= 8
            and mission.get('maximum_hold_speed_mps', float('inf')) <= .2)
    return bool(common and audit.get('reached_exit_time_s') is not None
                and audit.get('min_speed_mps', -1) >= 1.)


def outcome(capture, name):
    capture = Path(capture)
    status_path = capture.parent / 'nf4-status.json'
    status = read(status_path) if status_path.exists() else {}
    row = next((r for r in status.get('results', []) if r['name'] == name), None)
    report = dict(name=name, capture=str(capture), state='not_attempted', passed=False)
    guard_path = capture / 'UNATTEMPTED_RESOURCE_GUARD.json'
    if guard_path.exists():
        report.update(state='unattempted_resource_blocked', resource_guard=read(guard_path))
    elif row and row.get('timed_out'):
        report['state'] = 'timeout'
    elif row and row.get('returncode') != 0:
        report['state'] = 'execution_error'
    elif not row and status.get('current') == name and status.get('state') == 'running':
        report['state'] = 'running'
    elif capture.exists():
        report['state'] = 'unaudited'
    report['driver_result'] = row
    summary_path, audit_path = capture/'000_verifai/summary.json', capture/'projection-audit.json'
    summary = read(summary_path) if summary_path.exists() else {}
    audit = read(audit_path) if audit_path.exists() else {}
    if not guard_path.exists() and row and row.get('returncode') == 0 and audit and not row.get('timed_out'):
        checked = criteria_pass(audit, summary, name.startswith('10-'))
        report.update(passed=checked and audit.get('passed') is True and row.get('audit_returncode') == 0,
                      audit_boolean_consistent=checked == audit.get('passed'))
        report['state'] = 'passed' if report['passed'] else 'driving_or_audit_failure'
    frames = [read(p) for p in sorted(capture.glob('frame_*.json'))]
    proposals = [read(p) for p in sorted(capture.glob('proposal_*.json'))]
    controls = [read(p) for p in sorted(capture.glob('control_*.json'))]
    if guard_path.exists() and (frames or proposals or controls):
        report['state'] = 'resource_guard_with_capture_artifacts'
    csv_path = capture/'000_verifai/ego.csv'
    ego = list(csv.DictReader(csv_path.open()))[1:] if csv_path.exists() else []
    report['metrics'] = dict(decisions=len(frames), recorded_trajectory_samples=len(proposals),
        samples_per_decision=len(proposals)/len(frames) if frames else None,
        max_samples_per_decision=max((r.get('selection', {}).get('attempts', 1) for r in frames), default=0),
        decision_seconds=percentiles([float(r['inference_s']) for r in ego]),
        simulation_seconds=summary.get('simulation_seconds'), rollout_wall_seconds=summary.get('wall_seconds'),
        wall_seconds_per_sim_second=summary.get('wall_seconds', 0)/summary['simulation_seconds'] if summary.get('simulation_seconds') else None,
        collision_recorded=bool(summary.get('collision') or summary.get('termination')=='collision'),
        route_exit_time_s=audit.get('reached_exit_time_s'), min_corner_clearance_m=audit.get('min_corner_clearance_m'),
        min_speed_mps=audit.get('min_speed_mps'), max_speed_mps=audit.get('max_speed_mps'),
        offroad_steps=len(audit['body_departure_times_s']) if 'body_departure_times_s' in audit else None,
        retained_decisions=sum('retention' in r for r in frames),
        projected_decisions=sum(r['debug'].get('steering_mode')=='route' for r in controls),
        held_stop_decisions=sum(r['debug'].get('visual_stop_hold', {}).get('active', False) for r in controls),
        red_mission=audit.get('mission'))
    limit = 1 if name.startswith('10-') else (2 if name.endswith('-cap2') else 6)
    report['policy_bound_verified'] = bool(frames) and all(
        1 <= r.get('selection', {}).get('attempts', 0) <= limit
        and (not name.startswith('10-') or ('retention' not in r and
             r['selection'].get('kind') == 'single_native_stop_scene')) for r in frames)
    if report['passed'] and not report['policy_bound_verified']:
        report.update(passed=False, state='policy_budget_failure')
    return report


def runtime_pairing(baseline, candidate, registration, assignment=None):
    errors = []
    roots = [Path(baseline), Path(candidate)]
    needed = ('precision.json', 'experiment.json', 'config.json', 'route-reference.json', '000_verifai/initial.json')
    if any(not (root/name).exists() for root in roots for name in needed):
        return dict(verified=False, errors=['Missing runtime pairing metadata'])
    precision = [read(root/'precision.json') for root in roots]
    for key in PRECISION_KEYS:
        if key not in precision[0] or key not in precision[1] or precision[0][key] != precision[1][key]:
            errors.append('precision_metadata:' + key)
    for side, (root, meta) in enumerate(zip(roots, precision)):
        if (meta.get('precision') != 'nf4' or len(meta.get('quantized_layers', [])) != 624
                or meta.get('cpu_offload') is not False or meta.get('activation_quantization') != 'none'
                or meta.get('model_placement') != 'single_gpu'):
            errors.append(f'precision_or_placement:{side}')
        if meta.get('bridge_source_sha256') != registration['source_sha256']['local/experiments/nf4_closed_loop.py']:
            errors.append(f'loader_source:{side}')
        sources = read(root/'experiment.json')['source_sha256']
        for name in CORE_SOURCES:
            if sources.get(name) != registration['source_sha256'][name]:
                errors.append(f'controller_or_inference_source:{side}:{name}')
    for name in ('config.json', 'route-reference.json'):
        if read(roots[0]/name) != read(roots[1]/name):
            errors.append('scene_or_route:' + name)
    if assignment is not None:
        for side, root in enumerate(roots):
            status_path = root.parent/'nf4-status.json'
            status = read(status_path) if status_path.exists() else {}
            expected = registration if side == 0 else assignment
            if status.get('model_gpus') != [expected['model_gpu']] or status.get('renderer_gpu') != expected['renderer_gpu']:
                errors.append(f'declared_device_assignment:{side}')
    starts = [read(root/'000_verifai/initial.json')['actual_handover'] for root in roots]
    differences = {}
    for key in ('x', 'y', 'z', 'yaw', 'pitch', 'roll', 'speed'):
        delta = starts[0][key] - starts[1][key]
        if key in ('yaw', 'pitch', 'roll'):
            delta = (delta + 180) % 360 - 180
        differences[key] = abs(delta)
        if abs(delta) > (.01 if key == 'speed' else .001):
            errors.append('initial_handover:' + key)
    return dict(verified=not errors, errors=errors, initial_handover_absolute_differences=differences)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    args = parser.parse_args()
    manifest = read(args.manifest)
    registration = verify_pairing(manifest, check_weights=False)
    report = dict(study=manifest['study'], planned_baseline_jobs=2, planned_candidate_jobs=2,
        manifest_sha256=sha256(args.manifest), pairs=[],
        renderer_placement_amendment=manifest['renderer_placement_amendment'],
        caveat='Shared-device controller-assisted quality/call-count comparison. Timing is diagnostic only; incomplete/mismatched pairs are not pooled. No BF16 or general reliability claim.')
    for pair in manifest['pairs']:
        baseline, candidate = outcome(pair['baseline'], pair['baseline_name']), outcome(pair['candidate'], pair['candidate_name'])
        proof = runtime_pairing(pair['baseline'], pair['candidate'], registration, manifest['execution_assignment'])
        report['pairs'].append(dict(baseline=baseline, candidate=candidate, runtime_pairing=proof,
            complete_comparable_pair=proof['verified'] and baseline['state'] in ('passed', 'driving_or_audit_failure', 'policy_budget_failure')
            and candidate['state'] in ('passed', 'driving_or_audit_failure', 'policy_budget_failure')))
    write(args.manifest.parent/'policy-comparison.json', report)
    for pair in report['pairs']:
        print(json.dumps({key: {k:v for k,v in pair[key].items() if k in ('name','state','passed','metrics')}
                          for key in ('baseline','candidate')}))


if __name__ == '__main__':
    main()
