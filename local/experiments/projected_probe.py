"""Matched single-sample native versus map-target rollouts; no rejection sampling."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from local.experiments import inference_probe as native
from local.experiments.audit_turn import project
from local.experiments.projected_steering import ProjectedSteeringFollower
from local.experiments.validated_probe import RouteExitReached


def json_finite(value):
    if isinstance(value, dict):
        return {k: json_finite(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_finite(v) for v in value]
    return None if isinstance(value, float) and not math.isfinite(value) else value


def route_state(context, route, name, phase):
    pose = context['world_pose']
    distance, arc, heading, width, exit_arc = project(np.array([[pose['x'], pose['y']]]), route)
    end = route['exit']
    yaw = math.radians(end['yaw'])
    beyond = (pose['x'] - end['x']) * math.cos(yaw) + (pose['y'] - end['y']) * math.sin(yaw) >= 0
    aligned = abs((pose['yaw'] - heading[0] + 180) % 360 - 180) <= 20
    reached = bool(beyond and arc[0] >= exit_arc and distance[0] <= width[0] and aligned)
    instruction = 'Continue straight through the intersection.'
    if name == 'left':
        turn = (route['entry']['yaw'] - pose['yaw'] + 180) % 360 - 180
        if phase == 'approach' and turn >= 60:
            phase = 'finish_turn'
        instruction = ('Finish the left turn, then continue straight along the exit lane.'
                       if phase == 'finish_turn' else 'Turn left at the intersection.')
    return reached, phase, instruction


class SingleSampleProbe(native.Probe):
    def evaluate(self, adapter, data, context, instruction, options, label,
                 capture=False, capture_failures=False):
        if options.get('num_samples') != 1 or options.get('select_route_continuity'):
            raise ValueError('Projection comparison requires one candidate without selection')
        if not hasattr(self, 'route'):
            routes = json.loads(Path(options['validation_routes']).read_text())
            self.route = routes['routes'][options['validation_route']]
            self.phase = 'approach'
            native.write_json(self.output / 'route-reference.json', {**routes,
                'note': ('Commanded-route map centerline also used for steering projection.'
                         if options.get('steering_mode') == 'route' else
                         'Route used only for instruction phase and measurement. Map projection uses nearest CARLA driving waypoint, without route constraint.')})
        reached, self.phase, instruction = route_state(
            context, self.route, options['validation_route'], self.phase)
        if reached:
            native.write_json(self.output / 'route-exit-reached.json', {'context': context})
            raise RouteExitReached(self.output, context)
        result = super().evaluate(adapter, data, context, instruction,
            {**options, 'instruction': instruction}, label, capture, capture_failures)
        path = self.output / (label + '.json')
        record = json.loads(path.read_text())
        # Route annotation for the existing recorded-footprint audit; no admission gate.
        record.update(selection={'route': options['validation_route'], 'kind': 'single_sample_no_validation'},
                      navigation_phase=self.phase)
        native.write_json(path, record)
        return result


def capture(args, output):
    options = json.loads(args.options.read_text())
    mode = options.get('steering_mode')
    if mode not in ('native', 'map', 'route') or options.get('num_samples') != 1:
        raise ValueError('Explicit native/map/route steering mode and one sample required')
    runner = native.runner
    originals = runner.TrajectoryFollower, runner.connect_world, native.Probe, runner.load_sharded_model
    # Native capture has already saved the shared sources; add experiment-specific code.
    experiment_path = output / 'experiment.json'
    experiment = json.loads(experiment_path.read_text())
    for name in ('projected_steering.py', 'projected_probe.py', 'audit_projection.py'):
        relative = 'local/experiments/' + name
        content = (runner.ROOT / relative).read_bytes()
        path = output / 'source_snapshot' / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
        experiment['source_sha256'][relative] = hashlib.sha256(content).hexdigest()
    native.write_json(experiment_path, experiment)

    class LoggingFollower(ProjectedSteeringFollower):
        def __init__(self, vehicle):
            route = None
            if mode == 'route':
                route = json.loads((output / 'generated-routes.json').read_text())['routes'][options['validation_route']]
            super().__init__(vehicle, mode, route)
            self.index = 0

        def compute_control(self, tf, world, speed, speeds, elapsed_sec=0.):
            control = super().compute_control(tf, world, speed, speeds, elapsed_sec)
            native.write_json(output / f'control_{self.index:03d}.json', json_finite({
                'time_s': self.index * .1, 'world_trajectory': np.asarray(world).tolist(),
                'native_speeds': np.asarray(speeds).tolist(), 'elapsed_sec': elapsed_sec,
                'max_steer_rad': self.max_steer_rad, 'control': list(control[:3]), 'debug': control[3]}))
            self.index += 1
            return control

    def connect(*args):
        client, world = originals[1](*args)
        (output / 'map.xodr').write_text(world.get_map().to_opendrive())
        return client, world

    def load_single(args, adapter):
        import torch
        if torch.cuda.device_count() != 1 or torch.cuda.get_device_capability(0)[0] < 8:
            raise RuntimeError('Single-GPU comparison requires one visible GPU with native BF16 support')
        checkpoint = args.checkpoint or str(runner.ROOT.parent /
            'cache/huggingface/hub/models--nvidia--Alpamayo-1.5-10B/snapshots' / runner.SNAPSHOT)
        print('Loading unchanged BF16 model on one GPU for single-sample comparison', flush=True)
        model, processor = adapter.load_model(checkpoint_path=checkpoint,
            attn_implementation='sdpa', device_map={'': 0}, max_memory={0: '23GiB'})
        model.eval()
        print(f'Model allocated: {torch.cuda.memory_allocated(0)/2**30:.2f} GiB', flush=True)
        return model, processor

    runner.TrajectoryFollower, runner.connect_world, native.Probe = LoggingFollower, connect, SingleSampleProbe
    if options.get('model_placement') == 'single_gpu':
        runner.load_sharded_model = load_single
    try:
        ORIGINAL_CAPTURE(args, output)
    except RouteExitReached as exc:
        path = output / '000_verifai/summary.json'
        summary = json.loads(path.read_text())
        summary.pop('error', None)
        summary.update(termination='route_exit_reached', final_state={
            **exc.context['world_pose'], 'speed': exc.context['measured_speed_mps']})
        native.write_json(path, summary)
        native.write_json(output / 'batch-summary.json', [summary])
    finally:
        runner.TrajectoryFollower, runner.connect_world, native.Probe, runner.load_sharded_model = originals


ORIGINAL_CAPTURE = native.capture


def main():
    previous = native.capture
    native.capture = capture
    try:
        native.main()
    finally:
        native.capture = previous


if __name__ == '__main__':
    main()
