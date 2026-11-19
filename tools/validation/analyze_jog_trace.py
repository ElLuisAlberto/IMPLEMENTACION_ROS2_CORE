#!/usr/bin/env python3
"""Approximate paired JOG horizons from recorded state, intent and decision."""
import json
import math
import statistics
import sys

from thesis_core.joint_prediction import predict_joint_samples
from thesis_core.robot_profile import load_robot_profile

profile = load_robot_profile()
for path in sys.argv[1:]:
    events = [json.loads(line) for line in open(path, encoding='utf-8')]
    intents = {e['message']['command_id']: e for e in events if e['event'] == 'jog_intent'}
    joints = [e for e in events if e['event'] == 'joint_state']
    results, velocities = [], []
    for decision in (e for e in events if e['event'] == 'execution_control' and e['message']['state'] == 'REDUCTION'):
        msg = decision['message']
        intent = intents.get(msg['command_id'])
        if intent is None:
            continue
        candidates = [e for e in joints if e['monotonic'] <= intent['monotonic']]
        if not candidates:
            continue
        state = candidates[-1]
        age = intent['monotonic'] - state['monotonic']
        if age > .1:
            continue
        m = state['message']
        positions = dict(zip(m['name'], m['position']))
        speeds = dict(zip(m['name'], m['velocity']))
        if not all(n in positions for n in profile.joint_names):
            continue
        q = tuple(positions[n] for n in profile.joint_names)
        target = intent['message']['positions']
        duration = intent['message']['duration']['sec'] + intent['message']['duration']['nanosec'] * 1e-9
        scale = msg['speed_scale']
        if not 0 < scale < 1:
            continue
        def distance(s):
            prediction = predict_joint_samples(q, target, duration / s, duration, 21,
                joint_names=profile.joint_names, position_limits=profile.joint_position_limits,
                velocity_limits=profile.joint_velocity_limits, continuous_joints=profile.continuous_joints)
            base = profile.capsule_segments(q)
            return max(math.dist(a, b) for point in prediction.positions
                       for first, other in zip(base, profile.capsule_segments(point))
                       for a, b in ((first.start, other.start), (first.end, other.end)))
        nominal, supervised = distance(1), distance(scale)
        results.append((nominal, supervised, age))
        if profile.joint_names[0] in speeds:
            velocities.append(speeds[profile.joint_names[0]])
    print(json.dumps({'trace': path, 'pairs': len(results),
        'max_state_age_s': max((x[2] for x in results), default=None),
        'nominal_median_m': statistics.median(x[0] for x in results) if results else None,
        'supervised_median_m': statistics.median(x[1] for x in results) if results else None,
        'all_contracted': bool(results) and all(b < a for a, b, _ in results),
        'velocity_median_rad_s': statistics.median(velocities) if velocities else None}))
    exact = []
    for event in events:
        if event['event'] != 'jog_horizon_evidence':
            continue
        data = json.loads(event['message']['data'])
        scale = data['scale']
        if data['state'] != 'REDUCTION' or not 0 < scale < 1:
            continue
        current, target = data['current'], data['bounded_target']
        base = profile.capsule_segments(current)
        def exact_distance(s):
            prediction = predict_joint_samples(current, target, data['duration'] / s,
                data['horizon'], data['sample_count'], joint_names=profile.joint_names,
                position_limits=profile.joint_position_limits, velocity_limits=profile.joint_velocity_limits,
                continuous_joints=profile.continuous_joints)
            return max(math.dist(a, b) for point in prediction.positions
                       for first, other in zip(base, profile.capsule_segments(point))
                       for a, b in ((first.start, other.start), (first.end, other.end)))
        exact.append((exact_distance(1), exact_distance(scale)))
    if exact:
        print(json.dumps({'trace': path, 'exact_pairs': len(exact),
            'all_contracted': all(b < a for a, b in exact),
            'nominal_median_m': statistics.median(a for a, b in exact),
            'supervised_median_m': statistics.median(b for a, b in exact)}))
