"""Run a straight or left native-model driving example with validation and resampling."""
from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[1]


def prepare(route, *, duration=None, seed=None, flow_seed=None, interval=.1, config=None):
    manifest = json.loads((ROOT / 'local/experiments/presets/native-driving-junction-checks.json').read_text())
    case = next(c for c in manifest['scenarios'] if c['route'] == route)
    if config is None:
        resolved = yaml.safe_load((ROOT / manifest['base_config']).read_text())
        resolved['rollouts'] = [{k: case[k] for k in ('name', 'distance_before_intersection_m',
            'lateral_offset_m', 'speed_mps', 'heading_offset_deg')}]
        resolved['duration_s'] = case['duration_s']
    else:
        resolved = copy.deepcopy(config)
        if len(resolved.get('rollouts', [])) != 1:
            raise ValueError('A driving config must contain exactly one explicit rollout')
    if duration is not None:
        resolved['duration_s'] = duration
    seconds = resolved['duration_s']
    if not math.isfinite(seconds) or seconds < .1 or abs(round(seconds / .1) * .1 - seconds) > 1e-8:
        raise ValueError('Duration must be positive on the 0.1 s simulation grid')
    if not math.isfinite(interval) or not .1 <= interval <= .5 or abs(round(interval / .1) * .1 - interval) > 1e-8:
        raise ValueError('Replan interval must be on the 0.1 s grid in [0.1, 0.5]')
    text_seed = case['seed'] if seed is None else seed
    motion_seed = case['flow_seed'] if flow_seed is None else flow_seed
    if any(isinstance(v, bool) or not isinstance(v, int) or not 0 <= v < 2**32 for v in (text_seed, motion_seed)):
        raise ValueError('Seeds must be integers in [0, 2**32)')
    options = json.loads((ROOT / manifest['base_options']).read_text())
    options.update(validation_route=route, validation_routes='auto', seed=text_seed, flow_seed=motion_seed,
        validation_replan_interval_s=interval, instruction='Turn left at the intersection.' if route=='left'
        else 'Continue straight through the intersection.')
    resolved['instruction'] = options['instruction']
    return resolved, options


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', choices=('straight', 'left'), required=True)
    parser.add_argument('--config', type=Path, help='Optional YAML with exactly one rollout')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--seed', type=int, help='Language sampling seed')
    parser.add_argument('--flow-seed', type=int, help='Motion sampling seed')
    parser.add_argument('--replan-interval', type=float, default=.1, help='Simulation seconds; default 0.1')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--print-config', action='store_true', help='Print resolved settings without connecting or running')
    args = parser.parse_args(argv)
    config = yaml.safe_load(args.config.read_text()) if args.config else None
    resolved, options = prepare(args.route, duration=args.duration, seed=args.seed,
        flow_seed=args.flow_seed, interval=args.replan_interval, config=config)
    if args.print_config:
        print(json.dumps({'config': resolved, 'options': options}, indent=2))
        return 0
    output = (args.output or ROOT / 'outputs' / ('driving-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))).resolve()
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite {output}')
    temporary_root = ROOT / '.runtime' / 'driving-requests'
    temporary_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=temporary_root) as temporary:
        staging = Path(temporary)
        config_path, options_path = staging / 'config.yaml', staging / 'options.json'
        config_path.write_text(yaml.safe_dump(resolved, sort_keys=False))
        options_path.write_text(json.dumps(options, indent=2)+'\n')
        command = [sys.executable, '-m', 'local.experiments.plan_interval_probe', 'capture',
            '--config', str(config_path), '--options', str(options_path), '--duration', str(resolved['duration_s']),
            '--capture-frames', '0', '--host', args.host, '--port', str(args.port), '--output', str(output)]
        print(f'Running {args.route} with native trajectories; output: {output}', flush=True)
        return subprocess.run(command, cwd=ROOT).returncode


if __name__ == '__main__':
    raise SystemExit(main())
