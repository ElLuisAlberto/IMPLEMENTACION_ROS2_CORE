from types import SimpleNamespace

from thesis_core.horizon_preview import HorizonPreview


def test_pause_and_rewind_revoke_inputs_before_recovery():
    node = SimpleNamespace(
        _last_ros_time=None, _last_clock_advance=0., max_state_age=0.5,
        state=(1,), intent=1, supervised_intent=1, jog=1, execution=1,
        control=1, proximity_status=1, controller_feedback=1,
        state_stamp=1, _previous_jog_direction=(1,),
    )
    node.reset_inputs = lambda: HorizonPreview.reset_inputs(node)
    assert not HorizonPreview.clock_is_valid(node, 1., 0.)
    assert HorizonPreview.clock_is_valid(node, 1.1, 0.1)
    assert not HorizonPreview.clock_is_valid(node, 1.1, 0.7)
    assert node.state is None and node.jog is None and node.execution is None
    node.state = (2,)
    node.jog = 2
    assert not HorizonPreview.clock_is_valid(node, 0.5, 0.8)
    assert node.state is None and node.jog is None
    assert HorizonPreview.clock_is_valid(node, 0.6, 0.9)
    assert node.state is None


def test_invalidation_publishes_empty_prediction_and_deletion():
    predictions, markers = [], []
    node = SimpleNamespace(
        prediction_publisher=SimpleNamespace(publish=predictions.append),
        publisher=SimpleNamespace(publish=markers.append),
        empty_output=lambda: HorizonPreview.empty_output(None),
    )
    from builtin_interfaces.msg import Time
    HorizonPreview.invalidate_output(node, Time(sec=2))
    assert predictions[0].source == 'invalid'
    assert predictions[0].sample_count == 0 and not predictions[0].positions
    assert markers[0].markers[0].action == markers[0].markers[0].DELETEALL
