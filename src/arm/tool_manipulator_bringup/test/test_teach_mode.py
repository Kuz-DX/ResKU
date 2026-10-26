"""Read/teach-mode selection tests without touching hardware."""
from pathlib import Path
from types import SimpleNamespace
from types import ModuleType
import sys

SCRIPTS = Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0, str(SCRIPTS))
# This unit test exercises only pure controller selection; generated ROS
# messages need not already exist in the unbuilt workspace.
if 'dynamixel_interfaces.msg' not in sys.modules:
    package = ModuleType('dynamixel_interfaces')
    messages = ModuleType('dynamixel_interfaces.msg')
    messages.DynamixelState = type('DynamixelState', (), {})
    package.msg = messages
    sys.modules['dynamixel_interfaces'] = package
    sys.modules['dynamixel_interfaces.msg'] = messages
from enter_teach_mode import active_command_controllers


def test_only_active_arm_command_controllers_are_stopped():
    controllers = [
        SimpleNamespace(name='joint_state_broadcaster', state='active'),
        SimpleNamespace(name='manual_position_controller', state='active'),
        SimpleNamespace(name='ee_velocity_controller', state='active'),
        SimpleNamespace(name='arm_controller', state='inactive'),
        SimpleNamespace(name='unrelated_controller', state='active'),
    ]
    assert active_command_controllers(controllers) == [
        'ee_velocity_controller', 'manual_position_controller']
