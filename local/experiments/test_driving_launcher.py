"""The usable launcher resolves explicit reproducible cases without simulator access."""
import copy
import pytest
from local.run_driving import prepare, main


@pytest.mark.parametrize('route,seed,flow', [('straight',45,44),('left',44,43)])
def test_examples_resolve_same_predeclared_initial_states(route,seed,flow):
    config, options=prepare(route)
    assert len(config['rollouts'])==1 and config['rollouts'][0]['distance_before_intersection_m']==5
    assert options['seed']==seed and options['flow_seed']==flow
    assert options['validation_routes']=='auto' and options['validation_route']==route
    assert options['validation_allow_left_phase_cache'] and options['validation_resample_below_target_mps']==3.
    assert options['validation_resample_fast_unknown'] and options['validation_allow_straight_unknown_cache']
    assert config['rollouts'][0]['name'].endswith('-junction')
    assert options['validation_replan_interval_s']==.1
    assert config['instruction']==options['instruction'] and config['duration_s']==15


def test_custom_settings_preserve_user_config_and_model_only_validation():
    original,_=prepare('straight');before=copy.deepcopy(original)
    resolved,options=prepare('left',config=original,duration=12,seed=70,flow_seed=71,interval=.3)
    assert original==before and resolved['duration_s']==12
    assert options['seed']==70 and options['flow_seed']==71 and options['validation_replan_interval_s']==.3
    assert options['validation_check_body'] and options['verify_signal']


@pytest.mark.parametrize('change',[{'interval':0},{'interval':.15},{'duration':-1},{'duration':float('nan')},
                                  {'seed':-1},{'flow_seed':2**32},{'config':{'rollouts':[]}}])
def test_invalid_launch_settings_fail_before_connecting(change):
    with pytest.raises(ValueError):prepare('straight',**change)


def test_print_config_is_a_read_only_preview(capsys):
    assert main(['--route','left','--print-config'])==0
    assert 'validation_routes' in capsys.readouterr().out
