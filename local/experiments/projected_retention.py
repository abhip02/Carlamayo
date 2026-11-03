"""Revalidate recent native plans before executing another slowing prediction.

All anchors and times stay original. The only map-derived part is the declared
steering target projection. Scoped to the same controlled empty-green scene.
"""
import copy
import math
from types import SimpleNamespace

import carla
import numpy as np

from local.experiments.audit_turn import project
from local.experiments.projected_speed_selection import SpeedSelectedProbe, speed_score
from local.experiments.projected_steering import ProjectedSteeringFollower
from local.experiments.validated_selection import pose_transform
from module import config as cfg
from module.trajectory_follower import trajectory_to_world, planned_speeds

ACTIVE = None


def assess(plan, context, route, options):
    source = plan['record']['context']
    age = context['time_s'] - source['time_s']
    checks = dict(age_s=age, valid=False)
    if not 0 < age <= options['projection_cache_age_s'] + 1e-6:
        return checks
    step = round(age/.1)
    if abs(step*.1-age)>1e-6 or step+15>len(plan['speeds']):
        return checks
    tf = pose_transform(context['world_pose'])
    bounds = context['vehicle_bounds']
    box = carla.BoundingBox(carla.Location(**bounds['location']),carla.Vector3D(**bounds['extent']))
    box.rotation = carla.Rotation(**bounds['rotation'])
    corners = np.array([[p.x,p.y] for p in box.get_world_vertices(tf)])
    distance,_,_,width,_ = project(corners,route)
    checks['current_corner_clearance_m'] = float((width-distance).min())
    if checks['current_corner_clearance_m'] < 0:
        return checks
    vehicle = SimpleNamespace(get_physics_control=lambda:SimpleNamespace(
        wheels=[SimpleNamespace(max_steer_angle=70.)]))
    follower = ProjectedSteeringFollower(vehicle,'route',route,
        options.get('controller_speed_cap_mps'),options.get('steering_gain',1.))
    debug = follower.compute_control(tf,plan['world'],context['measured_speed_mps'],
                                     plan['speeds'],elapsed_sec=age)[3]
    target = np.array(debug['projected_target_xyz'][:2])
    rear = tf.transform(carla.Location(x=cfg.EGO_REAR_AXLE_X))
    offset = target - [rear.x,rear.y]
    yaw = math.radians(tf.rotation.yaw)
    forward = float(np.dot(offset,[math.cos(yaw),math.sin(yaw)]))
    _,arc,_,_,_=project(np.array([[rear.x,rear.y],target]),route)
    low,high=options.get('speed_preference_mps',[3.,5.])
    target_speed=debug['target_speed_kmh']/3.6
    checks.update(target_speed_mps=target_speed,forward_m=forward,
                  route_progress_m=float(arc[1]-arc[0]),
                  raw_preview_mps=float(plan['speeds'][step+10:step+15].mean()))
    checks['valid']=bool(low<=target_speed<=high and forward>1. and arc[1]-arc[0]>.25)
    return checks


class RetainedSpeedProbe(SpeedSelectedProbe):
    def __init__(self,model,output):
        global ACTIVE
        super().__init__(model,output)
        self.history=[]
        self.execution_record=None
        self.execution_world=None
        self.execution_age=0.
        ACTIVE=self

    def consider_retained(self,record,result,context,options):
        maximum=options['projection_cache_age_s']
        if not 0<maximum<=1.:
            raise ValueError('Projected cache age must be in (0, 1] seconds')
        self.history=[p for p in self.history if context['time_s']-p['record']['context']['time_s']<=maximum+1e-6]
        if math.isfinite(speed_score(record)[0]):
            raw=np.asarray(record['trajectory_ego'])
            self.history.append(dict(record=copy.deepcopy(record),result=result,
                world=trajectory_to_world(pose_transform(context['world_pose']),raw),speeds=planned_speeds(raw)))
        if record['visual_signal'] in ('red','yellow'):
            return None
        choices=[]
        for plan in self.history:
            check=assess(plan,context,self.route,options)
            if check['valid']:
                # Prefer the freshest currently valid original plan.
                choices.append((check['age_s'],plan,check))
        if not choices:
            return None
        _,plan,check=min(choices,key=lambda x:x[0])
        old=copy.deepcopy(plan['record'])
        old.update(context=copy.deepcopy(context),visual_signal=record['visual_signal'],
            visual_signal_answer=record['visual_signal_answer'],visual_signal_check_index=record['visual_signal_check_index'],
            raw_target_speed_mps=check['raw_preview_mps'],controller_preview_speed_mps=check['raw_preview_mps'],
            retention=dict(source_context=copy.deepcopy(plan['record']['context']),
                           source_proposal=plan['record']['label'],age_s=check['age_s'],checks=check))
        return (0.,abs(check['target_speed_mps']-4.)),plan['result'],old

    def selected(self,record):
        self.execution_record=record
        source=record.get('retention',{}).get('source_context',record['context'])
        self.execution_age=record['context']['time_s']-source['time_s']
        self.execution_world=trajectory_to_world(pose_transform(source['world_pose']),
                                                 np.asarray(record['trajectory_ego']))


def anchored_world_path(tf,trajectory):
    if not np.array_equal(trajectory,ACTIVE.execution_record['trajectory_ego']):
        raise RuntimeError('Requested trajectory differs from selected native source')
    return ACTIVE.execution_world
