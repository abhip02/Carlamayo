from types import SimpleNamespace

import carla
import numpy as np
import pytest

from local.experiments.projected_steering import ProjectedSteeringFollower
from module.trajectory_follower import TrajectoryFollower


def vehicle(mapping):
    return SimpleNamespace(get_physics_control=lambda: SimpleNamespace(
        wheels=[SimpleNamespace(max_steer_angle=70.)]),
        get_world=lambda: SimpleNamespace(get_map=lambda: mapping))


class Road:
    def __init__(self, y=0.):
        self.y, self.calls = y, []

    def get_waypoint(self, location, **kwargs):
        self.calls.append((location, kwargs))
        return SimpleNamespace(transform=carla.Transform(carla.Location(x=location.x,y=self.y)),
            road_id=42,lane_id=-1,s=10.,is_junction=False)


def test_projection_changes_only_steering_and_preserves_longitudinal_state_and_arrays():
    mapping = Road(-2.)
    v=vehicle(mapping)
    original, projected = TrajectoryFollower(v), ProjectedSteeringFollower(v)
    world=np.column_stack((np.arange(1,65)*.3,np.full(64,2.),np.zeros(64)))
    speeds=np.full(64,3.)
    world_before, speeds_before = world.copy(), speeds.copy()
    for speed in [3.,2.9,3.1,2.8]:
        native = original.compute_control(carla.Transform(),world,speed,speeds)
        actual = projected.compute_control(carla.Transform(),world,speed,speeds)
        assert native[0] > 0 and actual[0] < 0
        assert native[1:3] == actual[1:3]
        assert native[3]['target_speed_kmh'] == actual[3]['target_speed_kmh']
        for key in ('_integral','_prev_speed','_recent_targets','_launching','_braking'):
            assert getattr(original,key) == getattr(projected,key)
    np.testing.assert_array_equal(world,world_before)
    np.testing.assert_array_equal(speeds,speeds_before)
    assert mapping.calls[0][1] == {'project_to_road':True,'lane_type':carla.LaneType.Driving}
    assert actual[3]['projection_distance_m'] == pytest.approx(4.)


def test_native_arm_is_identical_and_does_not_query_map():
    road=Road()
    a,b=TrajectoryFollower(vehicle(road)),ProjectedSteeringFollower(vehicle(road),'native')
    path=np.column_stack((np.arange(1,65)*.3,np.linspace(0,3,64),np.zeros(64)))
    for speed in (3.,2.,.1):
        x=a.compute_control(carla.Transform(),path,speed,np.full(64,3.))
        y=b.compute_control(carla.Transform(),path,speed,np.full(64,3.))
        assert x[:3] == y[:3]
    assert not road.calls


def test_planned_stop_stays_a_stop_with_projection():
    road=Road()
    a,b=TrajectoryFollower(vehicle(road)),ProjectedSteeringFollower(vehicle(road))
    path=np.column_stack((np.arange(1,65)*.001,np.zeros(64),np.zeros(64)))
    x=a.compute_control(carla.Transform(),path,2.,np.full(64,.01))
    y=b.compute_control(carla.Transform(),path,2.,np.full(64,.01))
    assert x[1:3] == y[1:3]
    assert y[1] == 0 and y[2] > 0 and y[3]['planned_stop']


def test_missing_projection_terminates_without_silent_fallback():
    road=SimpleNamespace(get_waypoint=lambda *args,**kwargs:None)
    follower=ProjectedSteeringFollower(vehicle(road))
    with pytest.raises(RuntimeError,match='No driving-lane'):
        follower.compute_control(carla.Transform(),np.array([[8.,1.,0.]]),3.,np.full(64,3.))


def test_projection_rate_limit_uses_previous_applied_projected_steering():
    road=Road(-20.)
    follower=ProjectedSteeringFollower(vehicle(road))
    path=np.array([[4.,20.,0.],[8.,30.,0.]])
    first=follower.compute_control(carla.Transform(),path,3.,np.full(64,3.))[0]
    second=follower.compute_control(carla.Transform(),path,3.,np.full(64,3.))[0]
    assert first == pytest.approx(-.1)
    assert second == pytest.approx(-.2)


def test_capture_restores_original_runner_on_failure(tmp_path, monkeypatch):
    import json
    from local.experiments import projected_probe as probe
    options=tmp_path/'options.json'
    options.write_text(json.dumps({'steering_mode':'map','num_samples':1}))
    (tmp_path/'experiment.json').write_text('{"source_sha256": {}}')
    originals=(probe.native.runner.TrajectoryFollower,probe.native.runner.connect_world,probe.native.Probe)
    def fail(*args):
        assert probe.native.runner.TrajectoryFollower is not originals[0]
        assert probe.native.Probe is probe.SingleSampleProbe
        raise RuntimeError('test failure')
    monkeypatch.setattr(probe,'ORIGINAL_CAPTURE',fail)
    with pytest.raises(RuntimeError,match='test failure'):
        probe.capture(SimpleNamespace(options=options),tmp_path)
    assert (probe.native.runner.TrajectoryFollower,probe.native.runner.connect_world,probe.native.Probe)==originals


def test_commanded_route_projection_ignores_nearer_unrelated_lane_and_zero_segments():
    from local.experiments.projected_steering import CommandedRouteMap
    point=dict(x=0.,y=0.,z=0.,road_id=7,lane_id=-1,s=0.,is_junction=True)
    route={'points':[point,point,{**point,'y':20.,'s':20.}]}
    wp=CommandedRouteMap(route).get_waypoint(carla.Location(x=8.,y=5.))
    assert wp.transform.location.x == 0.
    assert wp.transform.location.y == 5.
    assert wp.road_id == 7 and wp.lane_id == -1
    assert wp.transform.rotation.yaw == pytest.approx(90.)


def test_route_mode_requires_reference_and_preserves_original_speed():
    v=vehicle(Road())
    with pytest.raises(ValueError,match='explicit commanded route'):
        ProjectedSteeringFollower(v,'route')
    point=dict(x=0.,y=0.,z=0.,road_id=7,lane_id=-1,s=0.,is_junction=True)
    route={'points':[point,{**point,'x':30.,'s':30.}]}
    follower=ProjectedSteeringFollower(v,'route',route)
    native=TrajectoryFollower(v)
    path=np.column_stack((np.arange(1,65)*.3,np.full(64,3.),np.zeros(64)))
    speed=np.full(64,3.)
    a=native.compute_control(carla.Transform(),path,3.,speed)
    b=follower.compute_control(carla.Transform(),path,3.,speed)
    assert b[0] == 0 and a[0] > 0
    assert a[1:3] == b[1:3]
    assert b[3]['steering_mode']=='route'
