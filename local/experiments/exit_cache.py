"""Narrow opt-in eligibility for unchanged native plans across label transitions.

Controlled empty scene only. This does not infer green, resolve signal
applicability, detect obstacles, or authorize approaching an unknown signal.
"""
import numpy as np

from local.experiments.audit_turn import project


def angle_error(a, b):
    return abs((a - b + 180) % 360 - 180)


def left_phase_transition_evidence(record, context, route, signal, phase, options):
    """Allow the same left maneuver across its existing 60-degree phase boundary.

    This only removes a label veto. Original age, native geometry, tracking,
    speed and current-state checks still determine whether the plan is usable.
    """
    if (not options.get('validation_allow_left_phase_cache', False) or
            not options.get('verify_signal') or options.get('validation_route') != 'left' or
            record['selection']['route'] != 'left' or
            record['navigation_phase'] != 'approach' or phase != 'finish_turn' or
            signal != 'green' or record['selection']['signal'] != 'green' or
            record.get('visual_signal') != 'green'):
        return None
    entry = route['entry']
    total_turn = (entry['yaw'] - route['exit']['yaw'] + 180) % 360 - 180
    if not np.isfinite(total_turn) or not 60 <= total_turn <= 120:
        return None
    _, entry_arc, _, _, _ = project(np.array([[entry['x'], entry['y']]]), route)
    states = []
    for label, item in (('source', record['context']), ('current', context)):
        pose = item['world_pose']
        if not np.isfinite([pose[k] for k in ('x', 'y', 'yaw')]).all():
            return None
        distance, arc, _, width, _ = project(np.array([[pose['x'], pose['y']]]), route)
        turn = (entry['yaw'] - pose['yaw'] + 180) % 360 - 180
        if (not np.isfinite([distance[0], arc[0], width[0], turn]).all() or
                distance[0] > width[0] or arc[0] <= entry_arc[0]):
            return None
        states.append({'state': label, 'left_rotation_deg': float(turn),
                       'center_distance_m': float(distance[0]), 'route_arc_m': float(arc[0])})
    if not (0 <= states[0]['left_rotation_deg'] < 60 <= states[1]['left_rotation_deg'] < 120 and
            states[1]['route_arc_m'] >= states[0]['route_arc_m']):
        return None
    return {'policy': 'approach_to_finish_left_turn', 'route': 'left',
            'source_phase': 'approach', 'current_phase': 'finish_turn',
            'source_signal': 'green', 'current_signal': 'green',
            'transition_turn_deg': 60, 'states': states,
            'note': 'Original phase labels retained. All physical and age checks still apply.'}


def unknown_exit_evidence(record, context, route, signal, phase, options):
    source_signal = record['selection']['signal']
    if source_signal == 'unknown' and options.get('validation_allow_unknown_exit_cache', False):
        policy = 'matching_unknown_after_left_turn_alignment'
    elif source_signal == 'green' and options.get('validation_allow_green_unknown_exit_cache', False):
        policy = 'green_to_unknown_after_left_turn_alignment'
    else:
        return None
    if (options.get('validation_route') != 'left' or not options.get('verify_signal') or
            signal != 'unknown' or
            phase != 'finish_turn' or record['navigation_phase'] != 'finish_turn'):
        return None
    end_yaw = route['exit']['yaw']
    turn = (route['entry']['yaw'] - end_yaw + 180) % 360 - 180
    if not np.isfinite(turn) or not 60 <= turn <= 120:
        return None
    _, entry_arc, _, _, _ = project(np.array([[route['entry']['x'], route['entry']['y']]]), route)
    states = []
    for label, item in (('source', record['context']), ('current', context)):
        pose = item['world_pose']
        if not np.isfinite([pose[k] for k in ('x', 'y', 'yaw')]).all():
            return None
        distance, arc, heading, width, _ = project(np.array([[pose['x'], pose['y']]]), route)
        errors = (angle_error(pose['yaw'], end_yaw), angle_error(heading[0], end_yaw))
        if (not np.isfinite([distance[0], arc[0], width[0], *errors]).all() or
                distance[0] > width[0] or arc[0] <= entry_arc[0] or max(errors) > 10):
            return None
        states.append({'state': label, 'heading_to_exit_deg': float(errors[0]),
                       'local_route_heading_to_exit_deg': float(errors[1]),
                       'center_distance_m': float(distance[0]), 'route_arc_m': float(arc[0])})
    return {'policy': policy,
            'source_signal': source_signal, 'current_signal': 'unknown',
            'maximum_heading_error_deg': 10, 'states': states,
            'note': 'Signal remains unknown. Same-source native path must pass every physical and age check.'}
