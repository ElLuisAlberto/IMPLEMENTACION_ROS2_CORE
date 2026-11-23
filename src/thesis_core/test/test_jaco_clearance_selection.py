"""Historical JACO2 clearance uses one name-based capsule selection."""

from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from thesis_core.clearance_markers import clearance_markers
from thesis_core.clearance_geometry import minimum_configuration_clearance
from thesis_core.horizon_clearance import evaluate_capsule_horizon
from thesis_core.jaco_kinematics import (
    CAPSULE_RADII,
    CLEARANCE_EXCLUDED_SEGMENT_NAMES,
    CLEARANCE_SEGMENT_NAMES,
    SEGMENT_NAMES,
)
from thesis_core.proximity_monitor_node import ProximityMonitorNode
from thesis_core.safety_supervisor_node import SafetySupervisorNode
from thesis_core.work_table import TABLE_CENTER, TABLE_SIZE, WorkTable


def _scene_segments(wrist_z=0.87, tool_z=0.91):
    return (
        ((0.0, 2.0, 0.8), (1.0, 2.0, 0.8)),
        ((0.0, 2.0, 0.8), (1.0, 2.0, 0.8)),
        ((0.0, 2.5, 0.8), (1.0, 2.5, 0.8)),
        ((0.0, 3.0, 0.8), (1.0, 3.0, 0.8)),
        ((0.0, 0.0, wrist_z), (0.1, 0.0, wrist_z)),
        ((0.0, 0.0, tool_z), (0.1, 0.0, tool_z)),
    )


def _monitor(segments, obstacle_center, table):
    points = {}
    start_frames, end_frames = [], []
    for index, (start, end) in enumerate(segments):
        start_frame, end_frame = f'start_{index}', f'end_{index}'
        start_frames.append(start_frame)
        end_frames.append(end_frame)
        points[start_frame] = start
        points[end_frame] = end
    measurements = []
    messages = []
    params = {
        'status_publishing_enabled': True,
        'stop_distance': 0.05,
        'warning_distance': 0.30,
    }
    node = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=params[name]),
        _obstacle=lambda: (
            Point(x=obstacle_center[0], y=obstacle_center[1],
                  z=obstacle_center[2]),
            0.12,
            (0.0, 0.0, 0.0),
            Time(sec=10),
        ),
        _publish_obstacle_marker=lambda *args: measurements.append(args),
        start_frames=start_frames,
        end_frames=end_frames,
        tf_lookup_timeout_sec=0.04,
        _frame_point=lambda frame, *_args: Point(
            x=points[frame][0], y=points[frame][1], z=points[frame][2]
        ),
        missing_frames=set(),
        segment_names=list(SEGMENT_NAMES),
        radii=list(CAPSULE_RADII),
        reference_frame='world',
        table=table,
        publisher=SimpleNamespace(publish=messages.append),
        previous_state=None,
        get_logger=lambda: SimpleNamespace(
            warning=lambda _message: None,
            info=lambda _message: None,
        ),
    )
    node._state_for_clearance = lambda clearance: (
        ProximityMonitorNode._state_for_clearance(node, clearance)
    )
    ProximityMonitorNode.evaluate(node)
    return messages[0], measurements[0]


def test_monitor_and_supervisor_match_at_t0_and_preserve_per_obstacle_winners():
    segments = _scene_segments()
    table = WorkTable(
        TABLE_CENTER,
        TABLE_SIZE,
        frozenset(CLEARANCE_EXCLUDED_SEGMENT_NAMES),
    )
    center = (0.5, 2.0, 0.8)
    status, (_, _, measurements, limiting, state) = _monitor(
        segments, center, table
    )
    sphere_hit = dict((name, hit) for name, hit, _color in measurements)['Esfera']
    table_hit = dict((name, hit) for name, hit, _color in measurements)['Mesa']

    supervisor = SimpleNamespace(
        joint_names=('joint_1',),
        segment_names=SEGMENT_NAMES,
        capsule_radii=CAPSULE_RADII,
        capsule_function=lambda _positions: segments,
        clearance_segment_names=CLEARANCE_SEGMENT_NAMES,
        protective_margin=lambda: 0.0,
        table=table,
    )
    supervised = SafetySupervisorNode.current_sphere_clearance(
        supervisor, (0.0,), center, 0.12
    )
    horizon = evaluate_capsule_horizon(
        (segments,) * 21,
        tuple(index / 20 for index in range(21)),
        CAPSULE_RADII,
        SEGMENT_NAMES,
        center,
        (0.0, 0.0, 0.0),
        0.12,
        table=table,
        included_segment_names=CLEARANCE_SEGMENT_NAMES,
    )

    assert sphere_hit.segment_name == 'upper_arm_to_forearm'
    assert sphere_hit.segment_index == 2
    assert sphere_hit.clearance == pytest.approx(0.295)
    assert table_hit.segment_name == 'wrist_1_to_wrist_2'
    assert table_hit.segment_index == 4
    assert table_hit.clearance == pytest.approx(0.035)
    assert limiting.segment_name == status.limiting_segment
    assert limiting is table_hit
    assert state == status.state == 'STOP'
    assert status.minimum_clearance == pytest.approx(supervised[0])
    assert status.limiting_segment == supervised[1] == 'wrist_1_to_wrist_2'
    assert horizon.current.segment_name == supervised[1]
    assert horizon.current.segment_index == supervised[2] == 4
    assert horizon.current.clearance == pytest.approx(supervised[0])
    assert horizon.obstacle_minima[0].segment_name == sphere_hit.segment_name
    assert horizon.obstacle_minima[1].segment_name == table_hit.segment_name
    assert all(
        result.segment_name not in CLEARANCE_EXCLUDED_SEGMENT_NAMES
        for result in (horizon.current, horizon.minimum)
    )

    mesa_markers = clearance_markers(
        Header(frame_id='world'), 'Mesa', table_hit, (1.0, 0.44, 0.06), state
    )
    assert len(mesa_markers) == 2
    assert mesa_markers[-1].text == f'{table_hit.clearance:.3f} m'


def test_table_measurement_switches_between_distal_capsules():
    table = WorkTable(
        TABLE_CENTER,
        TABLE_SIZE,
        frozenset(CLEARANCE_EXCLUDED_SEGMENT_NAMES),
    )
    names = SEGMENT_NAMES
    radii = CAPSULE_RADII
    first = table.closest(_scene_segments(wrist_z=0.87, tool_z=0.95),
                          names, radii)
    second = table.closest(_scene_segments(wrist_z=0.94, tool_z=0.82),
                           names, radii)

    assert first.segment_name == 'wrist_1_to_wrist_2'
    assert first.segment_index == 4
    assert first.clearance == pytest.approx(0.035)
    assert second.segment_name == 'wrist_2_to_tool'
    assert second.segment_index == 5
    assert second.clearance == pytest.approx(-0.02)
    for hit in (first, second):
        markers = clearance_markers(
            Header(frame_id='world'), 'Mesa', hit, (1.0, 0.44, 0.06), 'STOP'
        )
        assert len(markers) == 2
        assert markers[-1].text == f'{hit.clearance:.3f} m'


def test_sphere_near_excluded_capsule_is_ignored_but_distal_approach_is_not():
    segments = (
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ((0.0, 1.0, 0.0), (1.0, 1.0, 0.0)),
        ((0.0, 2.0, 0.0), (1.0, 2.0, 0.0)),
        ((0.0, 3.0, 0.0), (1.0, 3.0, 0.0)),
        ((0.0, 4.0, 0.0), (1.0, 4.0, 0.0)),
    )
    excluded_pose = minimum_configuration_clearance(
        segments, SEGMENT_NAMES, CAPSULE_RADII, (0.5, 0.0, 0.0), 0.12
    )
    ignored = minimum_configuration_clearance(
        segments,
        SEGMENT_NAMES,
        CAPSULE_RADII,
        (0.5, 0.0, 0.0),
        0.12,
        included_segment_names=CLEARANCE_SEGMENT_NAMES,
    )
    approached = minimum_configuration_clearance(
        segments,
        SEGMENT_NAMES,
        CAPSULE_RADII,
        (0.5, 1.0, 0.0),
        0.12,
        included_segment_names=CLEARANCE_SEGMENT_NAMES,
    )

    assert excluded_pose.segment_index == 0
    assert excluded_pose.clearance < 0.0
    assert ignored.segment_name == 'upper_arm_to_forearm'
    assert ignored.clearance > 0.30
    assert approached.segment_name == 'upper_arm_to_forearm'
    assert approached.clearance <= 0.05
    thresholds = {'stop_distance': 0.05, 'warning_distance': 0.30}
    monitor = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=thresholds[name])
    )
    assert ProximityMonitorNode._state_for_clearance(
        monitor, ignored.clearance
    ) == 'ALLOW'
    assert ProximityMonitorNode._state_for_clearance(
        monitor, approached.clearance
    ) == 'STOP'
