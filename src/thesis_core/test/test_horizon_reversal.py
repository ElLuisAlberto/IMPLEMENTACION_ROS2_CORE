"""Verify that a changed JOG direction prompts an immediate horizon."""

from types import SimpleNamespace

from thesis_core.horizon_preview import (
    HorizonPreview, direction_reversed, movement_directions,
    preview_model_from_profile, validate_preview_model,
)
from thesis_core.joint_model import JOINT_NAMES


def test_direction_reversal_ignores_small_motion_noise():
    """Only a true opposite command triggers an extra prediction."""
    measured = (0.0,) * 6
    forward = movement_directions(measured, (0.1, 0, 0, 0, 0, 0))
    noise = movement_directions(measured, (0.0001, 0, 0, 0, 0, 0))
    reverse = movement_directions(measured, (-0.1, 0, 0, 0, 0, 0))
    assert not direction_reversed(None, forward)
    assert not direction_reversed(forward, noise)
    assert direction_reversed(forward, reverse)


def test_terminal_result_clears_only_matching_execution():
    node = SimpleNamespace(
        execution=SimpleNamespace(command_id='active'), execution_time=0.0,
    )
    HorizonPreview.receive_execution(node, SimpleNamespace(
        status='SUCCEEDED', command_id='old',
    ))
    assert node.execution.command_id == 'active'
    HorizonPreview.receive_execution(node, SimpleNamespace(
        status='CANCELED', command_id='active',
    ))
    assert node.execution is None


def test_jog_color_uses_correlated_supervisor_decision():
    stamp = SimpleNamespace(sec=10, nanosec=0)
    node = SimpleNamespace(
        jog=SimpleNamespace(command_id='jog-1'),
        nominal_jog=SimpleNamespace(command_id='jog-1', stamp=stamp),
        nominal_jog_time=10.0,
        control=SimpleNamespace(
            command_id='jog-1',
            intent_stamp=stamp,
            predictive_threshold=0.055,
            state='REDUCTION',
        ),
        control_time=10.0,
        proximity_status=SimpleNamespace(state='ALLOW'), proximity_time=10.0,
    )
    assert HorizonPreview.visualization_state(node, 'jog', 10.1) == 'REDUCTION'
    node.control.command_id = 'different'
    assert HorizonPreview.visualization_state(node, 'jog', 10.1) == 'UNKNOWN'


def test_jog_overlay_requires_matching_fresh_nominal_and_control():
    nominal = SimpleNamespace(command_id='jog-1')
    control = SimpleNamespace(
        command_id='jog-1',
        intent_stamp=SimpleNamespace(sec=10, nanosec=0),
        predictive_threshold=0.055,
        state='REDUCTION',
    )
    node = SimpleNamespace(
        nominal_jog=SimpleNamespace(
            command_id=nominal.command_id,
            stamp=SimpleNamespace(sec=10, nanosec=0),
        ),
        nominal_jog_time=10.0,
        control=control,
        control_time=10.0,
    )

    assert HorizonPreview.matching_jog_decision(
            node, 'jog-1', 10.1
    ) is control
    assert HorizonPreview.matching_jog_decision(
            node, 'other-command', 10.1
    ) is None
    assert HorizonPreview.matching_jog_decision(
            node, 'jog-1', 11.1
    ) is None
    node.control_time = 10.0
    node.nominal_jog_time = 10.0
    node.control.intent_stamp = SimpleNamespace(sec=9, nanosec=0)
    assert HorizonPreview.matching_jog_decision(
            node, 'jog-1', 10.1
    ) is None


def test_execution_without_correlated_control_is_unknown():
    node = SimpleNamespace(execution=SimpleNamespace(command_id='active'),
                           control=None, control_time=0.0)
    assert HorizonPreview.runtime_state(node) == ('UNKNOWN', None, None)


def test_missing_proximity_never_defaults_to_allow():
    node = SimpleNamespace(proximity_status=None, proximity_time=0.0)
    assert HorizonPreview.visualization_state(node, 'zero', 10.0) == 'UNKNOWN'


def test_published_jog_reference_reduces_predicted_joint_travel():
    node = SimpleNamespace(
        state=(0.0,), horizon=1.0, count=21, intent=None,
        supervised_intent=None,
        joint_names=('axis',), position_limits={'axis': (-2.0, 2.0)},
        velocity_limits={'axis': 1.0}, continuous_joints=(),
        jog=SimpleNamespace(
            positions=(0.08,),
            duration=SimpleNamespace(sec=0, nanosec=100_000_000),
            command_id='fast',
        ),
    )
    fast, _, _ = HorizonPreview.make_poses(node, 'jog', 0.0)
    node.jog.positions = (0.02,)
    node.jog.command_id = 'slow'
    slow, metadata, _ = HorizonPreview.make_poses(node, 'jog', 0.0)
    assert fast[-1][0] == 0.08
    assert slow[-1][0] == 0.02
    assert metadata['command_id'] == 'slow'
    node.jog.positions = (0.0,)
    held, _, _ = HorizonPreview.make_poses(node, 'jog', 0.0)
    assert all(pose[0] == 0.0 for pose in held)


def test_reversal_publishes_prediction_without_waiting_for_tick():
    """A fresh supervised reversal publishes from the measured state."""
    outputs = []
    clock = SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=10_000_000_000,
        to_msg=lambda: 'stamp',
    ))
    node = SimpleNamespace(
        jog=None,
        jog_time=0.0,
        state=(0.0,) * 6,
        state_stamp=SimpleNamespace(sec=10, nanosec=0),
        max_state_age=0.5,
        joint_names=JOINT_NAMES,
        _previous_jog_direction=None,
        _clock_valid=True,
        state_is_fresh=lambda now: True,
        get_clock=lambda: clock,
        select_source=lambda now, ros: 'jog',
        make_poses=lambda source, now: (
            (), {'command_id': 'jog'}, 'prediction',
        ),
        publish_joint_prediction=lambda prediction, metadata, stamp:
            outputs.append((prediction, metadata, stamp)),
    )

    def jog(delta):
        return SimpleNamespace(
            joint_names=JOINT_NAMES,
            positions=(delta,) + (0.0,) * 5,
        )

    HorizonPreview.receive_jog(node, jog(0.1))
    assert not outputs
    HorizonPreview.receive_jog(node, jog(-0.1))
    assert outputs == [('prediction', {'command_id': 'jog'}, 'stamp')]
    assert node.jog.positions[0] == -0.1


def test_five_axis_preview_profile_is_valid():
    names = tuple(f'axis_{index}' for index in range(5))
    radii = (0.12, 0.08, 0.05)

    def segments(positions):
        return ()

    assert validate_preview_model(names, radii, segments) == (
        names, radii,
    )


def test_execution_contract_uses_profile_joint_count_and_names():
    names = tuple(f'axis_{index}' for index in range(5))
    execution = SimpleNamespace(
        joint_names=names,
        start_positions=(0.0,) * 5,
        target_positions=(0.1,) * 5,
        duration_sec=1.0,
        start_time=SimpleNamespace(sec=0, nanosec=0),
    )
    node = SimpleNamespace(execution=execution, joint_names=names)
    assert HorizonPreview.execution_is_active(node, 0.5)

    execution.joint_names = (*names[:-1], 'wrong_axis')
    assert not HorizonPreview.execution_is_active(node, 0.5)


def test_robot_profile_adapter_preserves_capsule_order_and_frame():
    """Adapt one five-axis profile without a JACO2-specific constant."""
    names = tuple(f'axis_{index}' for index in range(5))
    capsules = (
        SimpleNamespace(name='base', radius=0.12),
        SimpleNamespace(name='tool', radius=0.04),
    )

    def capsule_segments(positions):
        assert len(positions) == 5
        return (
            ('base', (0.0, 0.0, 0.0), (0.0, 0.0, 0.5), 0.12),
            ('tool', (0.0, 0.0, 0.5), (positions[0], 0.0, 0.7), 0.04),
        )

    profile = SimpleNamespace(
        joint_names=names,
        collision_capsules=capsules,
        capsule_segments=capsule_segments,
        world_frame='world',
    )
    model_names, radii, segments, frame = preview_model_from_profile(
        profile
    )

    assert model_names == names
    assert radii == (0.12, 0.04)
    assert frame == 'world'
    assert segments((0.2,) * 5)[1][1] == (0.2, 0.0, 0.7)
