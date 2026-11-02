"""Compile the production transition method with a fake bus, without ROS."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def test_torque_failure_is_returned_to_command_guard(tmp_path):
    source = (ROOT/'src/dynamixel_hardware_interface.cpp').read_text()
    method = source.split('bool DynamixelHardware::ChangeDxlTorqueState()', 1)[1]
    method = method.split('\nvoid DynamixelHardware::get_dxl_data_srv_callback', 1)[0]
    stub = r'''
#include <cassert>
#include <map>
#include <vector>
#include <utility>
#define RCLCPP_WARN_STREAM(...) ((void)0)
#define RCLCPP_ERROR(...) ((void)0)
enum class DxlError { OK, FAIL };
enum { REQUESTED_TO_ENABLE, REQUESTED_TO_DISABLE, TORQUE_DISABLED, TORQUE_ENABLED, TORQUE_OFF=0 };
struct Bus {
 DxlError result=DxlError::OK; int calls=0;
 std::map<int,int> states{{1,1}};
 DxlError DynamixelEnable(const std::vector<int>&){++calls;return result;}
 DxlError DynamixelDisable(const std::vector<int>&){++calls;return result;}
 std::map<int,int> GetDxlTorqueState(){return states;}
};
struct DynamixelHardware {
 std::vector<int> torque_enabled_comm_id_id_{1};
 int dxl_torque_status_=REQUESTED_TO_ENABLE, syncs=0;
 Bus bus; Bus* dxl_comm_=&bus;
 void SyncJointCommandWithStates(){++syncs;}
 bool ChangeDxlTorqueState();
};
'''
    checks = r'''
int main(){
 for(int request:{REQUESTED_TO_ENABLE,REQUESTED_TO_DISABLE}) {
  DynamixelHardware d; d.dxl_torque_status_=request; d.bus.result=DxlError::FAIL;
  assert(!d.ChangeDxlTorqueState()); assert(d.syncs==0);
  assert(d.dxl_torque_status_==request); assert(d.bus.calls==1);
 }
 DynamixelHardware d; assert(d.ChangeDxlTorqueState()); assert(d.syncs==1);
 assert(d.dxl_torque_status_==TORQUE_ENABLED);
 d.dxl_torque_status_=REQUESTED_TO_DISABLE; d.bus.states={{1,0}};
 assert(d.ChangeDxlTorqueState()); assert(d.dxl_torque_status_==TORQUE_DISABLED);
 DynamixelHardware empty; empty.torque_enabled_comm_id_id_.clear();
 assert(empty.ChangeDxlTorqueState()); assert(empty.bus.calls==0);
}
'''
    cpp = tmp_path/'transition.cpp'
    cpp.write_text(stub+'bool DynamixelHardware::ChangeDxlTorqueState()'+method+checks)
    binary = tmp_path/'transition'
    subprocess.run(['g++','-std=c++14',str(cpp),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
