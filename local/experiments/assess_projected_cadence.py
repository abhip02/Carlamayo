"""CPU-only eligibility envelopes on recorded states, never a driving replay.

No adapter/model/server invocation, policy patch, or fabricated proposal. The
adaptive schedules below retain current VQA refresh ticks and original source
clocks. Counterfactual states and missing samples are explicitly unobserved.
"""
import argparse
from collections import Counter
import copy
import csv
import hashlib
import json
import math
import os
from pathlib import Path


def read(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def screen(root,output):
    if os.environ.get('CUDA_VISIBLE_DEVICES')!='':
        raise ValueError('Hide CUDA for this CPU-only assessment')
    import numpy as np
    import torch
    from local.experiments import projected_retention as retained
    from local.experiments.projected_probe import route_state
    from local.experiments.projected_speed_selection import speed_score
    from local.experiments.validated_selection import pose_transform
    from module.trajectory_follower import trajectory_to_world, planned_speeds
    assert not torch.cuda.is_initialized()
    frames=[read(p) for p in sorted(root.glob('frame_*.json'))]
    if len(frames)!=114 or any(f['context']['index']!=i for i,f in enumerate(frames)):
        raise ValueError('Use the completed114-decision T trace')
    options=frames[0]['options']
    assert options['speed_selection_attempts']==2 and options['speed_preference_mps']==[2.5,5.]
    route=read(root/'generated-routes.json')['routes'][options['validation_route']]
    calls={f['context']['index']:read(root/f'proposal_{f["context"]["index"]:03d}_00.json') for f in frames}
    assert all(f['selection']['attempts']==1 for f in frames)
    times=[float(row['inference_s']) for row in csv.DictReader((root/'000_verifai/ego.csv').open())]
    assert len(times)==115 and times[0]==0
    timers=times[1:]
    endpoints=read(root/'route-exit-reached.json')['context']

    def plan_at(frame):
        label=frame['selection']['selected_proposal']
        proposal=read(root/(label+'.json'))
        original=frame.get('retention',{}).get('source_context',frame['context'])
        assert original==proposal['context']
        raw=np.asarray(proposal['trajectory_ego'])
        assert np.array_equal(raw,np.asarray(frame['trajectory_ego']))
        world=trajectory_to_world(pose_transform(original['world_pose']),raw)
        return dict(record=proposal,world=world,speeds=planned_speeds(raw),
            phase=frames[original['index']]['navigation_phase'],route=frame['selection']['route'])

    plans=[plan_at(frame) for frame in frames]
    anchor_errors=[]
    retained_checks=0
    for i,(frame,plan) in enumerate(zip(frames,plans,strict=True)):
        control=read(root/f'control_{i:03d}.json')
        error=float(np.max(np.abs(plan['world']-np.asarray(control['world_trajectory']))))
        if error>1e-6:raise ValueError('Original world anchor reconstruction failed')
        anchor_errors.append(error)
        if frame.get('retention'):
            actual=retained.assess(plan,frame['context'],route,options)
            recorded=frame['retention']['checks']
            for key,value in recorded.items():
                if isinstance(value,bool):assert actual[key]==value
                else:assert math.isclose(actual[key],value,rel_tol=1e-7,abs_tol=1e-7),(i,key,actual[key],value)
            retained_checks+=1

    def scoped(context):
        scene=context.get('controlled_scene',{})
        signals=context.get('signal_states',{})
        return (scene.get('verified') is True and scene.get('policy')=='empty_green'
            and scene.get('other_vehicle_count')==0 and scene.get('pedestrian_count')==0
            and scene.get('unexpected_signal_count')==0
            and (not signals or set(signals.values())=={'Green'}))

    def eligible(plan,frame,interval,mode):
        reasons=[];ctx=frame['context'];src=plan['record'];index=ctx['index']
        age=ctx['time_s']-src['context']['time_s']
        if not 0<age<interval-1e-8:reasons.append('replan_due_by_original_source_clock')
        if not options['verify_signal'] or not scoped(ctx) or not scoped(src['context']):reasons.append('scene_or_signal_verification')
        signal=frame['visual_signal'];source_signal=src['visual_signal']
        if signal in ('red','yellow') or source_signal in ('red','yellow'):reasons.append('red_or_yellow_never_reuse')
        if mode=='positive_green_only' and (signal!='green' or source_signal!='green'):
            reasons.append('positive_green_required')
        if signal not in ('unknown','green','red','yellow') or source_signal not in ('unknown','green','red','yellow'):
            reasons.append('invalid_signal_value')
        # Keep every actual scheduled VQA query by executing fresh inference at
        # that tick. Never borrow a later answer or postpone a due query.
        signal_age=index-frame['visual_signal_check_index']
        if not 0<=signal_age<options['signal_check_interval']:reasons.append('signal_stale')
        if signal_age==0:reasons.append('fresh_vqa_due')
        if plan['phase']!=frame['navigation_phase']:reasons.append('instruction_phase_changed')
        if plan['route']!=options['validation_route']:reasons.append('route_changed')
        reached,phase,_=route_state(ctx,route,options['validation_route'],plan['phase'])
        if reached or phase!=plan['phase']:reasons.append('route_phase_or_exit')
        if not math.isfinite(speed_score(src,2.5,5.,options['controller_speed_cap_mps'])[0]):reasons.append('native_format_or_reverse')
        geometry=retained.assess(plan,ctx,route,options)
        if not geometry['valid']:reasons.append('projected_retention_geometry_or_age')
        if geometry.get('preference_distance_mps')!=0:reasons.append('speed_preference_missed')
        return dict(eligible=not reasons,reasons=reasons,geometry=geometry,
                    source_index=src['context']['index'],original_age_s=age)

    summaries=[];details=[]
    for mode in ('positive_green_only','projected_unknown_scope_opportunity_only'):
        for interval in (.2,.3,.5):
            executed=set();active=None;reuse=[];fresh=[];unavailable=[];reasons=Counter();rows=[]
            for i,frame in enumerate(frames):
                evidence=eligible(active,frame,interval,mode) if active is not None else dict(eligible=False,reasons=['no_available_plan'])
                if evidence['eligible']:
                    reuse.append(i)
                else:
                    fresh.append(i);executed.add(i);reasons.update(evidence['reasons'])
                    # Only use a source that would actually have been generated
                    # at one of this schedule's fresh slots. Missing older cached
                    # sources cannot be silently obtained from the baseline.
                    observed=plans[i];source=observed['record']['context']['index']
                    if source in executed:active=observed
                    else:active=None;unavailable.append(dict(index=i,unavailable_source_index=source))
                rows.append(dict(index=i,reuse_eligible=evidence['eligible'],evidence=evidence))
            assert set(reuse).isdisjoint(fresh) and len(reuse)+len(fresh)==114
            assert all(frames[i]['visual_signal_check_index']!=i for i in reuse)
            assert all(i in fresh for i,f in enumerate(frames) if f['visual_signal_check_index']==i)
            summaries.append(dict(mode=mode,interval_s=interval,recorded_decisions=114,
                potentially_avoidable_native_call_slots=len(reuse),remaining_fresh_slots=len(fresh),
                retained_vqa_refreshes=len({f['visual_signal_check_index'] for f in frames}),
                reuse_indices=reuse,fresh_indices=fresh,unavailable_sources=unavailable,
                all_observed_source_choices_available=not unavailable,
                rejected_reasons=dict(reasons),observed_probe_seconds_at_skipped_slots=sum(frames[i]['inference_seconds'] for i in reuse),
                observed_enclosing_seconds_at_skipped_slots=sum(timers[i] for i in reuse),
                demonstrated_live_calls_saved=0,
                interpretation=('Conservative green-only eligibility on fixed observed states; no live driving/cost guarantee.'
                    if mode=='positive_green_only' else
                    'Sensitivity only: current projected retention permits unknown after a fresh candidate. Preemptively skipping that candidate is NOT equivalent or validated. Missing-source choices may require unrecorded samples.')))
            details.append(dict(mode=mode,interval_s=interval,rows=rows))
    reached,_,_=route_state(endpoints,route,options['validation_route'],frames[-1]['navigation_phase'])
    assert reached and endpoints['index']==114
    assert not torch.cuda.is_initialized()
    result=dict(kind='CPU-only eligibility and recorded-cost envelopes, not a rollout',
        trace=str(root.resolve()),native_calls_run=0,vqa_calls_run=0,model_loaded=False,cuda_initialized=False,
        signal_counts=dict(Counter(f['visual_signal'] for f in frames)),
        green_indices=[i for i,f in enumerate(frames) if f['visual_signal']=='green'],
        phase_changes=[dict(index=i,phase=f['navigation_phase']) for i,f in enumerate(frames)
                       if i==0 or f['navigation_phase']!=frames[i-1]['navigation_phase']],
        original_anchors_verified=len(anchor_errors),max_world_anchor_error_m=max(anchor_errors),
        existing_retention_checks_reproduced=retained_checks,original_exit_checked=True,
        original_calls=114,original_vqa_refreshes=23,original_rollout_wall_s=read(root/'batch-summary.json')[0]['wall_seconds'],
        summaries=summaries,
        limitations=['All future vehicle states are the original trace, not states caused by a reused plan.',
            'No unrecorded proposal or new VQA answer is invented; unavailable source choices are flagged.',
            'Avoided-slot times are historical opportunity accounting, not a measured runtime reduction.',
            'The unknown-signal sensitivity is not permission to reuse preemptively on unknown signals.',
            'Projected retention validates current footprint/target/time, not the old native-only tracking forecast.'],
        trace_files_sha256={str(p.relative_to(root)):sha(p) for p in sorted(root.glob('frame_*.json'))
                           +sorted(root.glob('proposal_*.json'))+sorted(root.glob('control_*.json'))
                           +[root/'generated-routes.json',root/'route-exit-reached.json',
                             root/'000_verifai/ego.csv',root/'batch-summary.json']},
        analysis_source_sha256=sha(Path(__file__)))
    output.mkdir(parents=True,exist_ok=False)
    (output/'assessment.json').write_text(json.dumps(result,indent=2)+'\n')
    (output/'detail.json').write_text(json.dumps(details,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('trace_files_sha256','summaries')},indent=2))
    for row in summaries:
        print(json.dumps({k:row[k] for k in ('mode','interval_s','potentially_avoidable_native_call_slots',
            'remaining_fresh_slots','unavailable_sources','observed_probe_seconds_at_skipped_slots')}))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();screen(args.trace,args.output)
