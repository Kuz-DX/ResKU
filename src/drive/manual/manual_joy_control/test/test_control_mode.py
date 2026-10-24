from types import SimpleNamespace as NS

from manual_joy_control.manual_joy_control_node import ManualJoyControlNode


def make_node():
    output = []
    node = NS(active_target='drive', manual_mode_axis=7, manual_mode_axis_sign=-1.,
              auto_mode_axis=6, auto_mode_axis_sign=-1., control_mode_axis_threshold=.5,
              _prev_manual_mode=False, _prev_auto_mode=False, _mode_rearm_required=False,
              control_mode_pub=NS(publish=lambda msg: output.append(msg.data)),
              get_logger=lambda: NS(info=lambda msg: None))
    return node, output


def joy(down=0., right=0.):
    return NS(axes=[0.]*6 + [-right, -down])


def test_mapping_and_held_buttons_publish_only_on_press():
    node, output = make_node()
    for msg in [joy(down=1.), joy(down=1.), joy(), joy(right=1.), joy(right=1.)]:
        ManualJoyControlNode._handle_control_mode(node, msg)
    assert output == ['MANUAL', 'AUTO']


def test_diagonal_prefers_manual_without_delayed_auto():
    node, output = make_node()
    for msg in [joy(1., 1.), joy(0., 1.), joy(), joy(0., 1.)]:
        ManualJoyControlNode._handle_control_mode(node, msg)
    assert output == ['MANUAL', 'AUTO']


def test_arm_focus_ignores_mode_and_requires_neutral_after_switch():
    node, output = make_node()
    node.active_target = 'arm'
    ManualJoyControlNode._handle_control_mode(node, joy(right=1.))
    assert not output
    # on_active_target marks rearm when leaving drive.
    node._latest_left_cmd = node._latest_right_cmd = 1.
    node._prev_landmark = False
    ManualJoyControlNode.on_active_target(node, NS(data='arm'))
    ManualJoyControlNode.on_active_target(node, NS(data='drive'))
    ManualJoyControlNode._handle_control_mode(node, joy(right=1.))
    assert not output
    ManualJoyControlNode._handle_control_mode(node, joy())
    ManualJoyControlNode._handle_control_mode(node, joy(right=1.))
    assert output == ['AUTO']


def test_short_axes_and_opposite_directions_do_not_request_mode():
    node, output = make_node()
    ManualJoyControlNode._handle_control_mode(node, NS(axes=[]))
    ManualJoyControlNode._handle_control_mode(node, joy(-1., -1.))
    assert not output
