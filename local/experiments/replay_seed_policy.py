"""Fixed 35-input, three-arm diagnostic; no simulator and no driving verdict."""
import argparse
from contextlib import contextmanager
import csv
import fcntl
import gzip
import hashlib
import io
import json
from pathlib import Path
import re
import subprocess
import time

from local.experiments.native_policy import SEED_POLICIES, resolve_sampling

ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = Path('/scratch/apomalap/thesis/experiments/native-traffic-20261116')
SHARED_LOCKS = Path('/scratch/apomalap/thesis/.carlamayo-resource-locks')


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def save(path, value):
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def load(path):
    return json.loads(Path(path).read_text())


def source_hashes():
    return {str(p.relative_to(ROOT)):digest(p) for folder in ('local','module')
            for p in sorted((ROOT/folder).rglob('*.py'))}


def fixtures():
    """The same selection as the prior navigation diagnostic, without choosing outcomes."""
    full = load(EVIDENCE/'v4-deliverables/input-manifest.json')['inputs']
    pilot = load(EVIDENCE/'pilot-deliverables/input-manifest.json')['inputs']
    rows = []
    for item in full:
        rows.append(dict(case=item['case'], position=item['position'],
            source_record=item['source_record'], source_input=item['source_input'],
            archive=item['archived_input'], raw_sha256=item['raw_sha256']))
    left = sorted((x for x in pilot if x['profile']=='v3'), key=lambda x:x['source_label'])
    if len(full) != 20 or len(left) != 29:
        raise ValueError('Expected all 20 suite observations and all 29 left-turn observations')
    for item in left[::2]:
        original = Path(item['source_input'])
        rows.append(dict(case='v3-left', position=item['source_label'],
            source_record=str(original.parent.parent/(item['source_label']+'.json')),
            source_input=str(original), archive=item['archive'], raw_sha256=item['raw_sha256']))
    if len(rows) != 35:
        raise ValueError('Fixed study requires 35 observations')
    for row in rows:
        record = load(row['source_record'])
        match = re.fullmatch(r'async_request_(\d+)', record['label'])
        if match is None:
            raise ValueError('Cannot recover original inference ordinal')
        row['inference_index'] = int(match[1])
        row['source_record_sha256'] = digest(row['source_record'])
        route = Path(row['source_record']).parent/'route-reference.json'
        row.update(route_reference=str(route), route_sha256=digest(route))
        row['seeds'] = {arm:resolve_sampling({**record['options'], 'seed_policy':arm},
                         {**record['context'], 'inference_index':row['inference_index']})[1]
                        for arm in SEED_POLICIES}
    return rows


def input_bytes(row):
    raw = Path(row['source_input'])
    if raw.is_file():
        data = raw.read_bytes()
    else:
        with gzip.open(row['archive'], 'rb') as stream:
            data = stream.read()
    if hashlib.sha256(data).hexdigest() != row['raw_sha256']:
        raise ValueError(f"Saved input hash mismatch: {raw}")
    return data


def prepare(output):
    output.mkdir(parents=True, exist_ok=False)
    rows = fixtures()
    for row in rows:
        input_bytes(row)
    plan = dict(state='prepared', source_root=str(ROOT), source_sha256=source_hashes(),
                selection='All first/final v4 inputs plus every second v3 left input (0,2,...,28).',
                variants=list(SEED_POLICIES), observations=rows, planned_calls=105,
                model='Unchanged BF16/SDPA Alpamayo 1.5; one trajectory, original diffusion steps/noise.',
                baseline_gate='All 35 fixed-arm trajectories must match their saved originals exactly before any candidate arm runs.',
                vqa='Reuse the recorded image-based signal answer, identically across arms; no ground truth.',
                no_projection=True, no_selection=True,
                interpretation='Offline sensitivity only; includes terminal off-route observations. No driving pass, seed search or stitching of successes.')
    save(output/'protocol.json', plan)
    save(output/'status.json', dict(state='prepared', verified_inputs=len(rows), model_calls=0))
    print(json.dumps(dict(plan=str(output/'protocol.json'), verified_inputs=len(rows), planned_calls=105)))


def require_idle_gpu(gpu):
    if not re.fullmatch(r'GPU-[0-9a-fA-F]{8}(?:-[0-9a-fA-F]{4}){3}-[0-9a-fA-F]{12}', gpu):
        raise ValueError('Use one full GPU UUID')
    jobs = subprocess.check_output(['nvidia-smi','--query-compute-apps=gpu_uuid,pid',
                                    '--format=csv,noheader,nounits'], text=True)
    if any(line.split(',')[0].strip()==gpu for line in jobs.splitlines()):
        raise RuntimeError(f'GPU occupied: {gpu}; no model loaded and no job displaced')
    data = subprocess.check_output(['nvidia-smi','-i',gpu,
        '--query-gpu=uuid,name,memory.free','--format=csv,noheader,nounits'], text=True)
    values = list(csv.reader(data.splitlines(),skipinitialspace=True))
    if len(values)!=1 or values[0][0]!=gpu or 'RTX A5000' not in values[0][1] or int(values[0][2])<23000:
        raise RuntimeError('Need an idle A5000 with at least 23000 MiB free for the frozen BF16 comparison')
    return dict(uuid=gpu, name=values[0][1], free_mib=int(values[0][2]))


@contextmanager
def gpu_lease(gpu):
    require_idle_gpu(gpu)
    # Use the existing shared inode, not a private /tmp lock. No metadata write
    # is needed on the full scratch volume; ownership is recorded in run status.
    with (SHARED_LOCKS/f'gpu-{gpu}.lock').open('r+') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX|fcntl.LOCK_NB)
        try:
            yield require_idle_gpu(gpu)
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)


def metrics(record, original, route):
    import numpy as np
    from local.experiments.audit_turn import project
    from local.experiments.validated_selection import pose_transform, predicted_corners
    from module.trajectory_follower import planned_speeds
    raw = np.asarray(record['trajectory_ego'])
    reference = np.asarray(original['trajectory_ego'])
    yaw = np.asarray(record['candidate_yaw_rad'][0])
    if raw.shape != (64,3) or yaw.shape != (64,) or not np.isfinite(raw).all() or not np.isfinite(yaw).all():
        raise ValueError('Malformed/nonfinite native prediction')
    ctx = original['context']
    corners = predicted_corners(pose_transform(ctx['world_pose']),raw,yaw,ctx['vehicle_bounds'])
    distance,_,_,width,_ = project(corners[:15,:,:2].reshape(-1,2),route)
    overhang = (distance-width).reshape(15,-1).max(axis=1)
    lead = round(original['options'].get('traffic',{}).get('speed_preview_lead_s',1.)*10)
    speeds = planned_speeds(raw)
    dyaw = yaw - np.asarray(original['candidate_yaw_rad'][0])
    return dict(raw_half_second_max_body_overhang_m=float(overhang[:5].max()),
                raw_1_5_second_max_body_overhang_m=float(overhang.max()),
                native_preview_mps=float(speeds[lead:lead+5].mean()),
                max_original_point_difference_m=float(np.abs(raw-reference).max()),
                ade_6_4s_m=float(np.linalg.norm(raw-reference,axis=1).mean()),
                max_yaw_difference_rad=float(np.abs(np.arctan2(np.sin(dyaw),np.cos(dyaw))).max()),
                trajectory_marker_present=record['trajectory_marker_present'],
                reverse_steps=record['reverse_velocity_steps'])


def run(plan_path, gpu):
    import os
    plan_path = Path(plan_path)
    plan = load(plan_path)
    if plan['source_sha256'] != source_hashes():
        raise ValueError('Frozen source changed; prepare a new protocol before inference')
    if len(plan['observations'])!=35 or plan['variants']!=list(SEED_POLICIES):
        raise ValueError('Unexpected study manifest')
    out = plan_path.parent/'replay'
    if out.exists():
        raise FileExistsError(out)
    status_path = plan_path.parent/'status.json'
    try:
        with gpu_lease(gpu) as resources:
            os.environ['CUDA_VISIBLE_DEVICES'] = gpu
            os.environ['HF_HOME'] = '/scratch/apomalap/thesis/cache/huggingface'
            os.environ['HF_HUB_OFFLINE'] = os.environ['TRANSFORMERS_OFFLINE'] = '1'
            import torch
            from local.experiments import inference_probe as native
            from local import run_intersection as runner
            from module.adapters.alpamayo_1_5 import Alpamayo15Adapter
            with native.acquire_experiment_lock():
                out.mkdir()
                started = time.monotonic()
                rows, attempted, probe = [], 0, None
                save(status_path, dict(state='running', pid=os.getpid(), resources=resources, model_calls=0))
                try:
                    adapter = Alpamayo15Adapter()
                    checkpoint = Path(os.environ['HF_HOME'])/'hub/models--nvidia--Alpamayo-1.5-10B/snapshots'/runner.SNAPSHOT
                    model, processor = adapter.load_model(checkpoint_path=str(checkpoint),
                        attn_implementation='sdpa',device_map={'':0},max_memory={0:'23GiB'})
                    model.eval()
                    adapter.probe_processor = processor
                    probe = native.Probe(model,out)
                    for arm in SEED_POLICIES:
                        for fixture in plan['observations']:
                            if digest(fixture['source_record'])!=fixture['source_record_sha256'] or digest(fixture['route_reference'])!=fixture['route_sha256']:
                                raise ValueError('Frozen fixture changed')
                            original = load(fixture['source_record'])
                            saved = torch.load(io.BytesIO(input_bytes(fixture)),map_location='cpu',weights_only=True)
                            if saved['context']!=original['context']:
                                raise ValueError('Input context does not match source prediction')
                            ctx = saved['context']
                            options = {**original['options'],'seed_policy':arm,
                                       'inference_index':fixture['inference_index']}
                            probe.visual_signal = original.get('visual_signal','unknown')
                            probe.visual_signal_answer = original.get('visual_signal_answer','')
                            probe.visual_signal_index = ctx['index']
                            label = f"{fixture['case']}-{fixture['position']}-{arm}"
                            attempted += 1
                            probe.evaluate(adapter,saved['data'],ctx,original['instruction'],options,label)
                            result = load(out/(label+'.json'))
                            route = load(fixture['route_reference'])['routes'][options['validation_route']]
                            rows.append(dict(case=fixture['case'],position=fixture['position'],variant=arm,
                                source_record=fixture['source_record'],input_sha256=fixture['raw_sha256'],
                                seconds=result['inference_seconds'],sampling_policy=result['sampling_policy'],
                                **metrics(result,original,route)))
                            save(out/'results.json',rows)
                            save(status_path,dict(state='running',model_calls=attempted,current=label))
                        if arm=='fixed' and any(r['max_original_point_difference_m']!=0 for r in rows):
                            raise RuntimeError('Fixed baseline failed exact reproduction; candidate arms not run')
                    save(status_path,dict(state='completed',model_calls=attempted,
                        baseline_reproduced=35,driving_verdict=None,
                        note='All outcomes retained; review raw geometry and speed before any live pilot.'))
                except BaseException as exc:
                    save(status_path,dict(state='failed',model_calls=attempted,error=f'{type(exc).__name__}: {exc}'))
                    raise
                finally:
                    if probe is not None: probe.close()
                    save(out/'cost.json',dict(total_wall_seconds=time.monotonic()-started,
                        attempted_model_calls=attempted,completed_predictions=len(rows),
                        inference_seconds=sum(r['seconds'] for r in rows),driving_verdict=None))
    except (RuntimeError,BlockingIOError,FileNotFoundError) as exc:
        if not out.exists():
            save(status_path,dict(state='resource_unavailable',model_calls=0,error=str(exc)))
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command',required=True)
    p = sub.add_parser('prepare'); p.add_argument('--output',type=Path,required=True)
    p = sub.add_parser('run'); p.add_argument('--plan',type=Path,required=True); p.add_argument('--gpu',required=True)
    args = parser.parse_args()
    if args.command=='prepare': prepare(args.output)
    else: run(args.plan,args.gpu)


if __name__=='__main__':
    main()
