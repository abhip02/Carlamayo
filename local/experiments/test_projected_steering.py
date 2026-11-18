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


def test_shorter_native_lookahead_preserves_prediction_differences_without_map_access():
    def forbidden_map_access():
        raise AssertionError('Native steering must not access the map')
    v = vehicle(None)
    v.get_world = forbidden_map_access
    responses = []
    for offset in (-.2, 0., .2):
        follower = ProjectedSteeringFollower(v, 'native', native_lookahead_min_m=2.)
        path = np.column_stack((np.arange(1,65)*.3, np.full(64,offset), np.zeros(64)))
        before = path.copy()
        steer, _, _, debug = follower.compute_control(carla.Transform(), path, 3., np.full(64,3.))
        responses.append(steer)
        assert debug['lookahead_m'] == pytest.approx(3.2)
        assert debug['projected_target_xyz'] is None
        assert debug['projection_distance_m'] == 0.
        np.testing.assert_array_equal(path, before)
    assert responses[0] < responses[1] == 0. < responses[2]
    # The override is instance-local: subsequent default followers remain unchanged.
    default = ProjectedSteeringFollower(v, 'native')
    result = default.compute_control(carla.Transform(), path, 3., np.full(64,3.))
    assert result[3]['lookahead_m'] == pytest.approx(5.2)


@pytest.mark.parametrize('minimum', [0., -1., float('nan'), float('inf'), 13.])
def test_invalid_native_lookahead_is_rejected(minimum):
    with pytest.raises(ValueError, match='Lookahead minimum'):
        ProjectedSteeringFollower(vehicle(None), 'native', native_lookahead_min_m=minimum)


def test_native_lookahead_cannot_silently_change_projected_arm():
    with pytest.raises(ValueError, match='requires native steering'):
        ProjectedSteeringFollower(vehicle(Road()), 'map', native_lookahead_min_m=2.)


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


def test_visual_hold_preserves_native_stop_against_launch_floor_and_releases_on_fresh_green():
    follower = ProjectedSteeringFollower(vehicle(Road()), hold_visual_stop=True)
    path = np.column_stack((np.arange(1,65)*.3, np.zeros(64), np.zeros(64)))
    speeds = np.full(64, 3.)
    before = path.copy(), speeds.copy()
    follower.observe_visual_signal('red', 20)
    first = follower.compute_control(carla.Transform(), path, 1.5, np.full(64, .1))
    assert first[3]['planned_stop'] and not first[3]['visual_stop_hold']['active']
    stopped = follower.compute_control(carla.Transform(), path, .01, speeds)
    assert stopped[1:3] == (0., 1.)
    assert stopped[3]['target_speed_kmh'] == 0 and not stopped[3]['launch']
    assert stopped[3]['visual_stop_hold']['activated']
    for signal, index in [('red', 25), ('unknown', 30), ('green', 20)]:
        follower.observe_visual_signal(signal, index)
        held = follower.compute_control(carla.Transform(), path, 0., speeds)
        assert held[1:3] == (0., 1.) and held[3]['visual_stop_hold']['active']
    follower.observe_visual_signal('green', 35)
    go = follower.compute_control(carla.Transform(), path, 0., speeds)
    assert go[1] > 0 and go[2] == 0 and go[3]['visual_stop_hold']['released']
    assert not go[3]['visual_stop_hold']['active']
    np.testing.assert_array_equal(path, before[0])
    np.testing.assert_array_equal(speeds, before[1])


@pytest.mark.parametrize('enabled,signal,speed,prior_stop', [
    (False,'red',0.,True), (True,'unknown',0.,True), (True,'green',0.,True),
    (True,'red',.3,True), (True,'red',0.,False)])
def test_visual_hold_does_not_invent_a_stop(enabled, signal, speed, prior_stop):
    follower = ProjectedSteeringFollower(vehicle(Road()), hold_visual_stop=enabled)
    follower.observe_visual_signal(signal, 0)
    path = np.column_stack((np.arange(1,65)*.3, np.zeros(64), np.zeros(64)))
    if prior_stop:
        follower.compute_control(carla.Transform(), path, 1.5, np.full(64,.1))
    actual = follower.compute_control(carla.Transform(), path, speed, np.full(64,3.))
    assert not follower._visual_stop_held
    assert actual[3]['target_speed_kmh'] > 0
