import carla
import numpy as np
import pytest

from local.experiments.test_projected_steering import Road, vehicle
from local.experiments.projected_steering import ProjectedSteeringFollower
from local.experiments.projected_speed_selection import speed_score
from local.experiments.test_projected_speed_selection import candidate


def test_ceiling_reduces_excessive_target_without_rewriting_predictions():
    world=np.column_stack((np.arange(1,65)*1.2,np.zeros(64),np.zeros(64)))
    speeds=np.full(64,12.)
    before_world,before_speeds=world.copy(),speeds.copy()
    original=ProjectedSteeringFollower(vehicle(Road()))
    limited=ProjectedSteeringFollower(vehicle(Road()),speed_cap_mps=4.5)
    a=original.compute_control(carla.Transform(),world,6.,speeds)
    b=limited.compute_control(carla.Transform(),world,6.,speeds)
    assert a[3]['target_speed_kmh']>b[3]['target_speed_kmh']
    assert b[3]['target_speed_kmh']/3.6==pytest.approx(4.5)
    assert b[1]<a[1]
    np.testing.assert_array_equal(world,before_world)
    np.testing.assert_array_equal(speeds,before_speeds)


def test_ceiling_preserves_a_model_stop_and_does_not_introduce_a_floor():
    path=np.column_stack((np.arange(1,65)*.001,np.zeros(64),np.zeros(64)))
    follower=ProjectedSteeringFollower(vehicle(Road()),speed_cap_mps=4.5)
    u=follower.compute_control(carla.Transform(),path,2.,np.full(64,.01))
    assert u[1]==0 and u[2]>0 and u[3]['planned_stop']
    assert u[3]['target_speed_kmh']/3.6==pytest.approx(.01)


def test_calibration_preserves_rate_limit_and_actuator_range():
    follower=ProjectedSteeringFollower(vehicle(Road(-20.)),steering_gain=1.18)
    path=np.array([[4.,20.,0.],[8.,30.,0.]])
    controls=[follower.compute_control(carla.Transform(),path,4.,np.full(64,4.))[0] for _ in range(20)]
    assert np.max(np.abs(np.diff([0.]+controls)))<=.1000001
    assert max(abs(x) for x in controls)<=1.
    assert controls[-1]<-.1


def test_selection_accounts_for_ceiling_without_rejecting_a_low_native_speed():
    assert speed_score(candidate(12.),speed_cap_mps=4.5)[0]==0
    assert speed_score(candidate(.1),speed_cap_mps=4.5)[0]>0
    assert candidate(.1)['controller_preview_speed_mps']==.1


@pytest.mark.parametrize('ceiling',[0.,-1.,float('inf'),float('nan')])
def test_invalid_speed_ceiling_rejected(ceiling):
    with pytest.raises(ValueError,match='positive and finite'):
        ProjectedSteeringFollower(vehicle(Road()),speed_cap_mps=ceiling)
