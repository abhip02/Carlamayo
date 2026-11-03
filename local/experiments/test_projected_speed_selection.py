import json
from types import SimpleNamespace

import numpy as np
import pytest

from local.experiments.projected_probe import SingleSampleProbe
from local.experiments.projected_speed_selection import SpeedSelectedProbe, speed_score


def candidate(speed, signal='green'):
    return dict(trajectory_ego=np.column_stack((np.arange(1,65)*speed*.1,
        np.zeros(64),np.zeros(64))).tolist(), controller_preview_speed_mps=speed,
        trajectory_marker_present=True, reverse_velocity_steps=0, visual_signal=signal,
        inference_seconds=2.)


def run(tmp_path, monkeypatch, speeds, signal='green', states=None):
    calls=[]
    probe=object.__new__(SpeedSelectedProbe)
    probe.output=tmp_path
    def fake(self, adapter, data, context, instruction, options, label, *args):
        item=candidate(speeds[len(calls)],signal)
        item.update(label=label,context=context,options=options)
        calls.append(item)
        (tmp_path/(label+'.json')).write_text(json.dumps(item))
        return item['trajectory_ego'], {'label':label}
    monkeypatch.setattr(SingleSampleProbe,'sample_plan',fake)
    result=probe.sample_plan(None,None,{'index':7,'signal_states':states or {'1':'Green'}},'Go',
        {'seed':44,'flow_seed':43,'speed_selection_attempts':3},'frame_007')
    return result,calls,json.loads((tmp_path/'frame_007.json').read_text())


def test_retries_until_preference_met_with_unchanged_selected_native_path(tmp_path,monkeypatch):
    result,calls,record=run(tmp_path,monkeypatch,[.2,4.,9.])
    assert len(calls)==2
    assert result[0]==record['trajectory_ego']==calls[1]['trajectory_ego']
    assert record['selection']['preference_met']
    assert record['inference_seconds']==4.
    assert calls[0]['options']['seed']!=calls[1]['options']['seed']
    assert calls[0]['options']['flow_seed']!=calls[1]['options']['flow_seed']


def test_budget_exhaustion_uses_closest_native_speed_and_labels_miss(tmp_path,monkeypatch):
    _,calls,record=run(tmp_path,monkeypatch,[.1,5.4,8.])
    assert len(calls)==3
    assert record['trajectory_ego']==calls[1]['trajectory_ego']
    assert not record['selection']['preference_met']
    assert record['inference_seconds']==6.


@pytest.mark.parametrize('signal',['red','yellow'])
def test_visual_stop_keeps_first_native_stop_without_retries(tmp_path,monkeypatch,signal):
    _,calls,record=run(tmp_path,monkeypatch,[.1,4.],signal)
    assert len(calls)==1 and record['controller_preview_speed_mps']==.1
    assert record['selection']['visual_stop_preserved']


def test_selector_rejects_other_scene_scope(tmp_path,monkeypatch):
    with pytest.raises(ValueError,match='all-green'):
        run(tmp_path,monkeypatch,[4.],states={'1':'Red'})


def test_invalid_or_reversing_trajectory_cannot_win_speed_ranking():
    item=candidate(4.)
    item['trajectory_ego'][5][0]=float('nan')
    assert np.isinf(speed_score(item)[0])
    item=candidate(4.);item['reverse_velocity_steps']=1
    assert np.isinf(speed_score(item)[0])


def soft_cache(self,record,result,context,options):
    old=candidate(2.8)
    old.update(label='proposal_old',retention={'age_s':.5})
    return (.2,1.2),(old['trajectory_ego'],{'label':'proposal_old'}),old


def test_soft_cache_does_not_end_search_before_a_better_fresh_plan(tmp_path,monkeypatch):
    monkeypatch.setattr(SpeedSelectedProbe,'consider_retained',soft_cache)
    _,calls,record=run(tmp_path,monkeypatch,[.1,4.])
    assert len(calls)==2 and record['controller_preview_speed_mps']==4.
    assert 'retention' not in record


def test_soft_cache_wins_after_budget_when_all_fresh_plans_are_slower(tmp_path,monkeypatch):
    monkeypatch.setattr(SpeedSelectedProbe,'consider_retained',soft_cache)
    _,calls,record=run(tmp_path,monkeypatch,[.1,.2,.3])
    assert len(calls)==3 and record['controller_preview_speed_mps']==2.8
    assert record['selection']['selected_proposal']=='proposal_old'
    assert not record['selection']['preference_met']
    assert record['selection']['preference_distance_mps']==.2
