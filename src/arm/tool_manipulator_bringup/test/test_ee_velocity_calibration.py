"""Exercise the driver's actual calibration/mapping code without ROS or motors."""
from pathlib import Path
import math
import subprocess
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

ARM = Path(__file__).resolve().parents[2]


def method(source, name):
    start = source.index('void DynamixelHardware::'+name+'(')
    opening = source.index('{', start)
    depth = 1
    end = opening+1
    while depth:
        depth += (source[end] == '{')-(source[end] == '}')
        end += 1
    return source[start:end]


def test_velocity_command_and_feedback_share_direction_and_restore_ros_commands(tmp_path):
    source = (ARM/'dynamixel_hardware_interface/src/dynamixel_hardware_interface.cpp').read_text()
    methods = '\n'.join(method(source, name) for name in (
        'MapInterfaces', 'CalcJointToTransmission', 'CalcTransmissionToJoint'))
    # Keep the production method bodies. Only the node/transport members are
    # replaced, so this test never opens a serial device or starts a ROS graph.
    harness = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#define RCLCPP_DEBUG_STREAM(logger, message) do {} while (0)
namespace hardware_interface {
constexpr const char* HW_IF_POSITION = "position";
constexpr const char* HW_IF_VELOCITY = "velocity";
constexpr const char* HW_IF_EFFORT = "effort";
}
namespace dynamixel_hardware_interface {
using Mapping = std::unordered_map<std::string, std::vector<std::string>>;
const Mapping dxl_to_ros2_cmd_map = {
  {"Goal Position", {"position"}}, {"Goal Velocity", {"velocity"}}};
const Mapping ros2_to_dxl_state_map = {
  {"position", {"Present Position"}}, {"velocity", {"Present Velocity"}},
  {"effort", {"Present Current"}}};
struct HandlerVarType {
  std::string name;
  std::vector<std::string> interface_name_vec;
  std::vector<std::shared_ptr<double>> value_ptr_vec;
};
HandlerVarType handler(std::vector<std::string> names, std::vector<double> values) {
  HandlerVarType result; result.name = "ee_joint"; result.interface_name_vec = names;
  for (double value : values) result.value_ptr_vec.push_back(std::make_shared<double>(value));
  return result;
}
class DynamixelHardware {
public:
  struct Calibration {
    double zero_offset{2.7}, direction{1.};
    bool state_wraparound{false}, command_wraparound{false}, command_nearest_turn{false};
  };
  bool use_revolute_to_prismatic_{false};
  size_t num_of_joints_{1}, num_of_transmissions_{1};
  std::string conversion_joint_name_, conversion_dxl_name_;
  std::vector<HandlerVarType> hdl_joint_commands_, hdl_trans_commands_;
  std::vector<HandlerVarType> hdl_joint_states_, hdl_trans_states_;
  std::vector<std::vector<double>> joint_to_transmission_matrix_{{1.}};
  std::vector<std::vector<double>> transmission_to_joint_matrix_{{1.}};
  std::map<std::string, Calibration> joint_position_calibrations_;
  std::map<std::string, double> last_model_positions_;
  double prismaticToRevolute(double value) {return value;}
  double revoluteToPrismatic(double value) {return value;}
  double wrapToPi(double value) {return std::remainder(value, 2.*M_PI);}
  void CalcJointToTransmission();
  void CalcTransmissionToJoint();
  void MapInterfaces(size_t, size_t, std::vector<HandlerVarType>&,
      std::vector<HandlerVarType>&, const std::vector<std::vector<double>>&,
      const Mapping&, const std::string&, const std::string&, std::function<double(double)>);
};
''' + methods + r'''
}
int main() {
  using namespace dynamixel_hardware_interface;
  for (double direction : {-1., 1.}) for (double velocity : {-1., 0., 1.}) {
    DynamixelHardware hw;
    hw.joint_position_calibrations_["ee_joint"].direction = direction;
    hw.hdl_joint_commands_ = {handler({"position", "velocity"}, {.4, velocity})};
    hw.hdl_trans_commands_ = {handler({"Goal Position", "Goal Velocity"}, {0., 0.})};
    for (int repeat = 0; repeat < 2; ++repeat) {
      hw.CalcJointToTransmission();
      assert(std::abs(*hw.hdl_trans_commands_[0].value_ptr_vec[0]-(2.7+direction*.4)) < 1e-12);
      assert(*hw.hdl_trans_commands_[0].value_ptr_vec[1] == direction*velocity);
      assert(*hw.hdl_joint_commands_[0].value_ptr_vec[0] == .4);
      assert(*hw.hdl_joint_commands_[0].value_ptr_vec[1] == velocity);
    }
    hw.hdl_trans_states_ = {handler({"Present Position", "Present Velocity", "Present Current"},
      {2.7+direction*.4, direction*velocity, direction*.7})};
    hw.hdl_joint_states_ = {handler({"position", "velocity", "effort"}, {0., 0., 0.})};
    hw.CalcTransmissionToJoint();
    assert(std::abs(*hw.hdl_joint_states_[0].value_ptr_vec[0]-.4) < 1e-12);
    assert(*hw.hdl_joint_states_[0].value_ptr_vec[1] == velocity);
    assert(*hw.hdl_joint_states_[0].value_ptr_vec[2] == .7);
  }
}
'''
    cpp = tmp_path/'calibration.cpp'
    cpp.write_text(harness)
    binary = tmp_path/'calibration'
    build = subprocess.run(['g++', '-std=c++14', '-O0', str(cpp), '-o', str(binary)],
                           capture_output=True, text=True, timeout=30)
    assert build.returncode == 0, build.stdout+build.stderr
    subprocess.run([str(binary)], check=True, timeout=5)


def test_real_ee_mapping_opens_for_negative_ros_velocity_with_the_same_zero():
    hardware = ARM/'tool_manipulator_bringup/config/hardware.yaml'
    data = yaml.safe_load(hardware.read_text())['joints']['ee_joint']
    assert data['raw_increases_ccw'] is True and data['zero_raw'] == 1992
    xml = xacro.process_file(str(ARM/'tool_manipulator_moveit_config/config/tool_manipulator.urdf.xacro'),
        mappings={'use_mock_hardware': 'false', 'hardware_config_file': str(hardware),
                  'use_mesh': 'false'}).toxml()
    joint = ET.fromstring(xml).find("ros2_control/joint[@name='ee_joint']")
    direction = float(joint.find("param[@name='position_direction']").text)
    offset = float(joint.find("param[@name='position_zero_offset']").text)
    assert direction == 1.
    assert offset == pytest.approx(1992*2*math.pi/4096-math.pi)
    assert joint.find("command_interface[@name='velocity']") is not None
    assert direction*(-1.) == -1.  # Wizard 2.0: negative motor velocity opens.
    model = (ARM/'dynamixel_hardware_interface/param/dxl_model/mx_106.model').read_text()
    velocity = next(line.split() for line in model.splitlines()
                    if line.startswith('Goal Velocity\t'))
    assert float(velocity[2]) > 0. and velocity[4] == 'signed'
