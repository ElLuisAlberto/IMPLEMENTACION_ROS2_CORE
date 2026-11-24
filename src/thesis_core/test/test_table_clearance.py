"""The simulated mounting table is a conservative preventive obstacle."""

import pytest

from thesis_core.clearance_geometry import segment_box_clearance


def test_capsule_touching_tabletop_is_not_reported_as_clear():
    result = segment_box_clearance(
        (0.0, 0.0, 0.90), (0.0, 0.0, 0.78), 0.05,
        (0.0, 0.0, 0.73), (1.20, 0.80, 0.04),
    )
    assert result.clearance <= 0.0


def test_capsule_well_above_tabletop_keeps_positive_clearance():
    result = segment_box_clearance(
        (0.0, 0.0, 1.20), (0.20, 0.0, 1.20), 0.05,
        (0.0, 0.0, 0.73), (1.20, 0.80, 0.04),
    )
    assert result.clearance == pytest.approx(0.40, abs=0.002)


def test_table_box_rejects_nonpositive_dimension():
    with pytest.raises(ValueError, match='box size'):
        segment_box_clearance(
            (0.0, 0.0, 1.0), (0.0, 0.0, 1.1), 0.05,
            (0.0, 0.0, 0.73), (1.20, 0.0, 0.04),
        )


@pytest.mark.parametrize(('start_z', 'end_z', 'expected'), [
    (1.2, 1.15, 'ALLOW'), (0.95, 0.94, 'WARNING'),
    (1.2, 0.74, 'REDUCTION'), (0.79, 0.74, 'STOP'),
])
@pytest.mark.parametrize('mounted_shoulder', (False, True))
def test_supervised_descent_with_distant_sphere(start_z, end_z, expected, mounted_shoulder):
    from types import SimpleNamespace
    from thesis_core.horizon_clearance import evaluate_capsule_horizon
    from thesis_core.work_table import WorkTable, TABLE_CENTER, TABLE_SIZE
    from thesis_core.safety_supervisor_node import SafetySupervisorNode

    table = WorkTable(TABLE_CENTER, TABLE_SIZE, frozenset({'pedestal'}))
    params = dict(stop_distance=.05, warning_distance=.30, predictive_stop_reserve_m=.02,
                  runtime_minimum_speed_scale=.1, runtime_scale_search_iterations=12,
                  withdrawal_speed_scale=.2, withdrawal_minimum_progress_m=.002,
                  withdrawal_monotonic_tolerance_m=.001)
    node = SimpleNamespace(protective_margin=lambda: .025,
                           get_parameter=lambda key: SimpleNamespace(value=params[key]))

    def evaluate(scale):
        samples = []
        for z in [start_z, start_z + scale * (end_z - start_z)]:
            shoulder_z = .85175 if mounted_shoulder else 1.50
            samples.append((((0., 0., .73), (0., 0., .80)),
                            ((0., 0., shoulder_z), (.1, 0., shoulder_z)),
                            ((0., 0., z), (.1, 0., z))))
        geometry = evaluate_capsule_horizon(samples, [0., 1.], [.1, .05, .05],
                                            ['pedestal', 'shoulder', 'wrist'], (5., 0., 1.),
                                            (0., 0., 0.), .12, table=table)
        node.last_horizon_clearance = geometry
        return (geometry.minimum.legacy_tuple(),)

    evaluate(1.)
    nominal = node.last_horizon_clearance
    decision, _, verified = SafetySupervisorNode.decide_horizon(
        node, nominal.current.clearance, nominal, evaluate)
    if mounted_shoulder and expected == 'ALLOW':
        expected = 'WARNING'
    assert decision.state == expected
    if expected == 'REDUCTION':
        assert nominal.minimum.sample_time == 1.
        assert nominal.minimum.clearance < 0
        assert verified.minimum.clearance >= (.05175 - 1e-10 if mounted_shoulder else .07)


def test_moving_sphere_can_overtake_table_as_limiting_obstacle():
    from thesis_core.horizon_clearance import evaluate_capsule_horizon
    from thesis_core.work_table import WorkTable, TABLE_CENTER, TABLE_SIZE

    posture = (((0., 0., 1.), (0., .1, 1.)),)
    result = evaluate_capsule_horizon(
        [posture, posture], [0., 1.], [.04], ['tool'], (1., 0., 1.),
        (-1., 0., 0.), .12, table=WorkTable(TABLE_CENTER, TABLE_SIZE, frozenset()))
    assert result.current.clearance == pytest.approx(.21)
    assert result.minimum.clearance == pytest.approx(-.16)
    assert result.minimum.sample_index == 1


def test_rotated_board_witness_and_rviz_cube_share_world_pose():
    from std_msgs.msg import Header
    from visualization_msgs.msg import Marker
    from thesis_core.work_table import WorkTable, TABLE_SIZE, table_marker

    # Rotate the long side to world Y. This point now lies over the board.
    q = 2 ** -0.5
    table = WorkTable((2., 1., .73), TABLE_SIZE, frozenset(), (0., 0., q, q))
    point = (2., 1.55, .85)
    hit = table.closest([(point, point)], ['tool'], [.07])
    assert hit.clearance == pytest.approx(.03)
    assert hit.closest_robot_point == pytest.approx(point)
    assert hit.capsule_start == point
    marker = table_marker(table, Header(frame_id='world'))
    assert marker.type == Marker.CUBE
    assert marker.header.frame_id == 'world'
    assert (marker.scale.x, marker.scale.y, marker.scale.z) == TABLE_SIZE
    assert marker.pose.orientation.z == pytest.approx(q)
    assert marker.pose.position.z == table.center[2]


def test_current_table_witness_follows_nearest_capsule():
    from std_msgs.msg import Header
    from thesis_core.clearance_markers import clearance_markers
    from thesis_core.work_table import WorkTable, TABLE_CENTER, TABLE_SIZE

    table = WorkTable(TABLE_CENTER, TABLE_SIZE, frozenset())
    shoulder = ((0., 0., .90675), (0., 0., 1.))
    for tool_z, segment, gap in [(1.0, 'shoulder', .05175),
                                 (.87, 'tool', .03)]:
        tool = ((.3, 0., tool_z), (.4, 0., tool_z))
        hit = table.closest((shoulder, tool), ('shoulder', 'tool'), (.105, .09))
        markers = clearance_markers(Header(), 'Mesa', hit, (1., .44, .06), 'WARNING')
        assert hit.segment_name == segment
        assert hit.clearance == pytest.approx(gap)
        assert markers[-1].text == f'{gap:.3f} m'
        assert markers[0].points[1].z == pytest.approx(.75)


def test_historical_jaco_table_uses_only_the_four_distal_capsules():
    from types import SimpleNamespace
    from thesis_core.jaco_kinematics import CLEARANCE_EXCLUDED_SEGMENT_NAMES
    from thesis_core.work_table import configured_table

    parameters = {'table_enabled': True}

    class Node:
        def declare_parameter(self, name, default):
            return SimpleNamespace(value=parameters.get(name, default))

    table = configured_table(Node())
    assert table.excluded == frozenset(CLEARANCE_EXCLUDED_SEGMENT_NAMES)

    segments = tuple(
        ((0.0, 0.0, z), (0.1, 0.0, z))
        for z in (.80, .80, .99, .95, .87, .91)
    )
    names = (
        'base_to_shoulder',
        'shoulder_to_upper_arm',
        'upper_arm_to_forearm',
        'forearm_to_wrist_1',
        'wrist_1_to_wrist_2',
        'wrist_2_to_tool',
    )
    radii = (.105, .105, .085, .075, .085, .09)
    result = table.closest(segments, names, radii)

    assert result.segment_name == 'wrist_1_to_wrist_2'
    assert result.segment_index == 4
    assert result.clearance == pytest.approx(.035)

    parameters['table_excluded_segments'] = ['base_to_shoulder']
    with pytest.raises(ValueError, match='two configured base capsules'):
        configured_table(Node())


def test_standalone_distance_lines_end_on_leg_and_sphere_surfaces():
    from math import dist
    from std_msgs.msg import Header
    from thesis_core.clearance_geometry import segment_sphere_clearance
    from thesis_core.proximity_monitor_node import clearance_markers
    from thesis_core.work_table import JACO2_SIM_TABLE_SIZE, TABLE_CENTER, WorkTable

    table = WorkTable(TABLE_CENTER, JACO2_SIM_TABLE_SIZE, frozenset(), with_legs=True)
    tool = (1.02, .72, .35)
    leg = table.closest([(tool, tool)], ['tool'], [.02])
    sphere = segment_sphere_clearance(tool, tool, .02, (.82, .72, .35), .06)

    for name, hit, expected_end in [('Mesa', leg, (1.09, .72, .35)),
                                    ('Esfera', sphere, (.88, .72, .35))]:
        markers = clearance_markers(
            Header(), name, hit, (1., 1., 1.), None,
        )
        line, label = markers[0], markers[-1]
        a, b = [(p.x, p.y, p.z) for p in line.points]
        assert b == pytest.approx(expected_end)
        assert dist(a, b) == pytest.approx(hit.clearance)
        assert label.text == f'{hit.clearance:.3f} m'
        if name == 'Mesa':
            assert len(markers) == 2
            assert len(line.points) == 2
        else:
            assert len(markers) == 2


def test_nominal_wrist_witness_uses_future_table_surface():
    from std_msgs.msg import Header
    from thesis_core.clearance_markers import clearance_markers
    from thesis_core.horizon_clearance import evaluate_capsule_horizon
    from thesis_core.work_table import TABLE_CENTER, TABLE_SIZE, WorkTable

    shoulder = ((0., 0., .90675), (0., 0., 1.))
    def pose(z):
        return (shoulder, ((.3, .1, z), (.4, .1, z)))

    horizon = evaluate_capsule_horizon(
        (pose(1.), pose(.8)), (0., 1.), (.105, .09), ('shoulder', 'wrist'),
        (2., 0., 1.), (0., 0., 0.), .12,
        table=WorkTable(TABLE_CENTER, TABLE_SIZE, frozenset()),
    )
    sphere, wrist = horizon.obstacle_minima
    assert horizon.current.segment_name == 'shoulder'
    assert sphere.box_point is None
    assert wrist.segment_name == 'wrist' and wrist.clearance == pytest.approx(-.04)
    line, label = clearance_markers(
        Header(frame_id='world'), 'JOG_nominal', wrist, (1., .44, .06), None,
        nominal=True,
    )
    assert line.points[0].z == pytest.approx(.71)
    assert line.points[1].z == pytest.approx(.75)
    assert label.text == '-0.040 m'
