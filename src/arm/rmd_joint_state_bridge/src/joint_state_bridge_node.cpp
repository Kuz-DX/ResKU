// 실기(JECS) 팔의 실측 엔코더를 ros2_control 없이 읽기 전용으로 폴링해서
// RViz(/joint_states)와 진단 토픽(/arm/joint_angle_deg 등)으로 발행한다.
//
// [주의] 이 노드는 포지션/토크 커맨드를 절대 보내지 않는다(RMD/Dynamixel 둘 다
// getter만 호출) - army_manipulator_bringup의 ros2_control 경로(실기에서
// use_mock_hardware:=false로 띄우는 것)와 달리 컨트롤러가 목표 위치를 붙잡고
//있지 않아서, 팔을 손으로 자유롭게 움직이며 자세를 캡처하는 용도로 안전하게
// 쓸 수 있다. RMD는 army_manipulator_ros2_control.xacro의 sign/q_offset과,
// Dynamixel은 같은 파일의 position_direction/position_zero_offset과 정확히
// 같은 보정 공식을 써서 /joint_states 값이 URDF/실제 ros2_control 경로와
// 일치하도록 맞췄다(공식 출처: rmd_hardware_interface.cpp, dynamixel_hardware_interface.cpp).
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <rmd_sdk/actuator_interface.hpp>
#include <rmd_sdk/driver/can_driver.hpp>

#include <dynamixel_sdk/dynamixel_sdk.h>

namespace {
constexpr uint16_t kAddrPresentPosition = 132;
constexpr uint16_t kAddrHardwareErrorStatus = 70;
// Dynamixel Protocol 2.0 위치 단위: 4096 pulse/rev, 모델 좌표계 중심을
// 0(=zero_offset 적용 전)이 아니라 -pi로 두는 이 저장소의 기존 관례를
// dynamixel_hardware_interface.cpp의 주석("pulse*2pi/4096-pi")과 동일하게 맞췄다.
constexpr double kDxlPulseToRad = 2.0 * M_PI / 4096.0;

std::string hardwareErrorDetails(uint8_t status)
{
  struct ErrorBit {uint8_t mask; const char * name;};
  constexpr ErrorBit kErrorBits[] = {
    {0x01, "input voltage"},
    {0x02, "motor Hall sensor"},
    {0x04, "overheating"},
    {0x08, "motor encoder"},
    {0x10, "electrical shock / insufficient power"},
    {0x20, "overload"},
  };
  std::ostringstream out;
  for (const auto & bit : kErrorBits) {
    if (status & bit.mask) {
      if (out.tellp() > 0) {
        out << ", ";
      }
      out << bit.name;
    }
  }
  return out.str().empty() ? "unknown" : out.str();
}
}  // namespace

class RmdJointStateBridge : public rclcpp::Node
{
public:
  RmdJointStateBridge()
  : rclcpp::Node("rmd_joint_state_bridge")
  {
    declare_parameter<std::string>("can_ifname", "can_arm");
    declare_parameter<int>("shoulder_actuator_id", 4);
    declare_parameter<int>("elbow_actuator_id", 5);
    declare_parameter<int>("wrist_actuator_id", 6);
    // army_manipulator_ros2_control.xacro의 shoulder_sign/elbow_sign/wrist_sign,
    // shoulder_q_offset/elbow_q_offset/wrist_q_offset과 동일한 이름/기본값 -
    // 실기 캘리브레이션 값을 그대로 재사용하면 됨.
    // [수정, 2026-09-02] elbow_sign/wrist_sign 기본값이 xacro/joint_state_bridge.launch.py
    // 의 실제 캘리브레이션(-1.0)과 달리 1.0으로 잘못 박혀 있었다 - launch
    // 파일 없이 `ros2 run rmd_joint_state_bridge joint_state_bridge_node`로
    // 바로 띄우면(launch가 오버라이드 안 됨) elbow/wrist 부호가 실제와
    // 반대로 읽혀서 capture_arm_pose.py 캡처값이 좌표계가 뒤집힌 채로
    // 나오는 원인이었다.
    declare_parameter<double>("shoulder_sign", 1.0);
    declare_parameter<double>("elbow_sign", -1.0);
    declare_parameter<double>("wrist_sign", -1.0);
    declare_parameter<double>("shoulder_q_offset", 0.0);
    declare_parameter<double>("elbow_q_offset", 0.0);
    declare_parameter<double>("wrist_q_offset", 0.0);

    declare_parameter<std::string>("dxl_port_name", "/dev/ttyUSB0");
    declare_parameter<int>("dxl_baud_rate", 1000000);
    declare_parameter<int>("base_dxl_id", 0);
    declare_parameter<int>("gripper_dxl_id", 4);
    // army_manipulator_ros2_control.xacro의 position_zero_offset/position_direction과
    // 동일한 이름/기본값(실측 JECS 캘리브레이션 값 그대로).
    // [수정, 2026-09-02] -3.14159265359(=-pi, 플레이스홀더로 보임)는 xacro/
    // joint_state_bridge.launch.py의 실제 실측값(-9.314331344042)과 전혀
    // 달랐다 - launch 파일 없이 바로 띄우면 base_joint 영점이 완전히
    // 틀어진 채로 읽혔다.
    declare_parameter<double>("base_zero_offset", -9.314331344042);
    declare_parameter<double>("base_direction", 1.0);
    declare_parameter<bool>("base_wraparound", true);
    declare_parameter<double>("gripper_zero_offset", -1.375980766733627);
    declare_parameter<double>("gripper_direction", 1.0);
    declare_parameter<bool>("gripper_wraparound", true);

    declare_parameter<double>("publish_rate_hz", 20.0);

    shoulder_sign_ = get_parameter("shoulder_sign").as_double();
    elbow_sign_ = get_parameter("elbow_sign").as_double();
    wrist_sign_ = get_parameter("wrist_sign").as_double();
    shoulder_q_offset_ = get_parameter("shoulder_q_offset").as_double();
    elbow_q_offset_ = get_parameter("elbow_q_offset").as_double();
    wrist_q_offset_ = get_parameter("wrist_q_offset").as_double();

    base_dxl_id_ = get_parameter("base_dxl_id").as_int();
    gripper_dxl_id_ = get_parameter("gripper_dxl_id").as_int();
    base_zero_offset_ = get_parameter("base_zero_offset").as_double();
    base_direction_ = get_parameter("base_direction").as_double();
    base_wraparound_ = get_parameter("base_wraparound").as_bool();
    gripper_zero_offset_ = get_parameter("gripper_zero_offset").as_double();
    gripper_direction_ = get_parameter("gripper_direction").as_double();
    gripper_wraparound_ = get_parameter("gripper_wraparound").as_bool();

    auto const can_ifname = get_parameter("can_ifname").as_string();
    RCLCPP_INFO(get_logger(), "Opening RMD CAN interface '%s'...", can_ifname.c_str());
    can_driver_ = std::make_unique<rmd_sdk::CanDriver>(can_ifname);
    shoulder_ = std::make_unique<rmd_sdk::ActuatorInterface>(
      *can_driver_, static_cast<std::uint32_t>(get_parameter("shoulder_actuator_id").as_int()));
    elbow_ = std::make_unique<rmd_sdk::ActuatorInterface>(
      *can_driver_, static_cast<std::uint32_t>(get_parameter("elbow_actuator_id").as_int()));
    wrist_ = std::make_unique<rmd_sdk::ActuatorInterface>(
      *can_driver_, static_cast<std::uint32_t>(get_parameter("wrist_actuator_id").as_int()));

    auto const dxl_port_name = get_parameter("dxl_port_name").as_string();
    RCLCPP_INFO(get_logger(), "Opening Dynamixel port '%s'...", dxl_port_name.c_str());
    dxl_port_ = dynamixel::PortHandler::getPortHandler(dxl_port_name.c_str());
    dxl_packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    if (!dxl_port_->openPort()) {
      throw std::runtime_error("Failed to open Dynamixel port '" + dxl_port_name + "'");
    }
    if (!dxl_port_->setBaudRate(get_parameter("dxl_baud_rate").as_int())) {
      throw std::runtime_error("Failed to set Dynamixel baud rate");
    }

    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    rmd_angle_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("/arm/joint_angle_deg", 10);
    rmd_raw_angle_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("/arm/rmd_raw_angle_deg", 10);
    rmd_encoder_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("/arm/rmd_encoder", 10);
    dxl_angle_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("/arm/dxl_angle_deg", 10);
    dxl_encoder_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("/arm/dxl_encoder", 10);

    double const rate_hz = get_parameter("publish_rate_hz").as_double();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&RmdJointStateBridge::onTimer, this));

    RCLCPP_INFO(get_logger(), "rmd_joint_state_bridge ready (read-only, no position commands).");
  }

private:
  // rmd_hardware_interface.cpp와 동일 공식: joint_rad = sign * (deg2rad(raw_deg) - q_offset).
  static double calibrateRmd(double raw_deg, double sign, double q_offset)
  {
    return sign * (raw_deg * M_PI / 180.0 - q_offset);
  }

  // dynamixel_hardware_interface.cpp와 동일 공식: joint_rad = direction * (raw_rad - zero_offset).
  static double calibrateDxl(
    std::int32_t raw_pulse, double zero_offset, double direction, bool wraparound)
  {
    double const raw_rad = static_cast<double>(raw_pulse) * kDxlPulseToRad - M_PI;
    double angle = direction * (raw_rad - zero_offset);
    if (wraparound) {
      angle = std::remainder(angle, 2.0 * M_PI);
    }
    return angle;
  }

  bool readDynamixelPulse(std::uint8_t id, std::int32_t & out_pulse)
  {
    std::uint32_t raw{0};
    std::uint8_t error{0};
    int const comm_result =
      dxl_packet_->read4ByteTxRx(dxl_port_, id, kAddrPresentPosition, &raw, &error);
    if (comm_result != COMM_SUCCESS || error != 0) {
      if (comm_result == COMM_SUCCESS && (error & 0x80U)) {
        uint8_t hardware_status{0};
        uint8_t status_read_error{0};
        int const status_result = dxl_packet_->read1ByteTxRx(
          dxl_port_, id, kAddrHardwareErrorStatus, &hardware_status, &status_read_error);
        if (status_result == COMM_SUCCESS) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Dynamixel id=%d Hardware Error Status: 0x%02X (%s). "
            "Power-cycle the base motor after checking the cause; torque is disabled until reset.",
            id, hardware_status, hardwareErrorDetails(hardware_status).c_str());
        }
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dynamixel id=%d read failed (comm=%d, error=%d): %s",
        id, comm_result, error, dxl_packet_->getTxRxResult(comm_result));
      return false;
    }
    out_pulse = static_cast<std::int32_t>(raw);
    return true;
  }

  void onTimer()
  {
    sensor_msgs::msg::JointState joint_states;
    joint_states.header.stamp = now();

    // ----- RMD (shoulder/elbow/wrist), CAN, 읽기 전용 -----
    struct RmdJoint
    {
      std::string name;
      rmd_sdk::ActuatorInterface * actuator;
      double sign;
      double q_offset;
    };
    std::vector<RmdJoint> const rmd_joints{
      {"shoulder_joint", shoulder_.get(), shoulder_sign_, shoulder_q_offset_},
      {"elbow_joint", elbow_.get(), elbow_sign_, elbow_q_offset_},
      {"wrist_joint", wrist_.get(), wrist_sign_, wrist_q_offset_},
    };
    std_msgs::msg::Float64MultiArray angle_deg_msg;
    std_msgs::msg::Float64MultiArray raw_angle_deg_msg;
    std_msgs::msg::Float64MultiArray encoder_msg;
    for (auto const & joint : rmd_joints) {
      try {
        double const raw_deg = static_cast<double>(joint.actuator->getMultiTurnAngle());
        std::int32_t const encoder = joint.actuator->getMultiTurnEncoderOriginalPosition();
        double const calibrated_rad = calibrateRmd(raw_deg, joint.sign, joint.q_offset);

        joint_states.name.push_back(joint.name);
        joint_states.position.push_back(calibrated_rad);
        angle_deg_msg.data.push_back(calibrated_rad * 180.0 / M_PI);
        raw_angle_deg_msg.data.push_back(raw_deg);
        encoder_msg.data.push_back(static_cast<double>(encoder));
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "RMD '%s' read failed: %s", joint.name.c_str(), e.what());
      }
    }
    rmd_angle_pub_->publish(angle_deg_msg);
    rmd_raw_angle_pub_->publish(raw_angle_deg_msg);
    rmd_encoder_pub_->publish(encoder_msg);

    // ----- Dynamixel (base/gripper), TTL, 읽기 전용 -----
    struct DxlJoint
    {
      std::string name;
      std::uint8_t id;
      double zero_offset;
      double direction;
      bool wraparound;
    };
    std::vector<DxlJoint> const dxl_joints{
      {"base_joint", static_cast<std::uint8_t>(base_dxl_id_), base_zero_offset_, base_direction_,
        base_wraparound_},
      {"gripper_joint", static_cast<std::uint8_t>(gripper_dxl_id_), gripper_zero_offset_,
        gripper_direction_, gripper_wraparound_},
    };
    std_msgs::msg::Float64MultiArray dxl_angle_deg_msg;
    std_msgs::msg::Float64MultiArray dxl_encoder_msg;
    for (auto const & joint : dxl_joints) {
      std::int32_t raw_pulse{0};
      if (!readDynamixelPulse(joint.id, raw_pulse)) {
        continue;
      }
      double const calibrated_rad = calibrateDxl(
        raw_pulse, joint.zero_offset, joint.direction, joint.wraparound);
      joint_states.name.push_back(joint.name);
      joint_states.position.push_back(calibrated_rad);
      dxl_angle_deg_msg.data.push_back(calibrated_rad * 180.0 / M_PI);
      dxl_encoder_msg.data.push_back(static_cast<double>(raw_pulse));
    }
    dxl_angle_pub_->publish(dxl_angle_deg_msg);
    dxl_encoder_pub_->publish(dxl_encoder_msg);

    if (!joint_states.name.empty()) {
      joint_states_pub_->publish(joint_states);
    }
  }

  std::unique_ptr<rmd_sdk::CanDriver> can_driver_;
  std::unique_ptr<rmd_sdk::ActuatorInterface> shoulder_, elbow_, wrist_;
  double shoulder_sign_{1.0}, elbow_sign_{1.0}, wrist_sign_{1.0};
  double shoulder_q_offset_{0.0}, elbow_q_offset_{0.0}, wrist_q_offset_{0.0};

  // dynamixel_sdk의 getPortHandler/getPacketHandler는 SDK 내부에서 수명을
  // 관리하는 포인터를 반환한다(프로세스 생애주기 동안 유효, 호출자가
  // delete하면 안 됨) - 그래서 스마트 포인터로 감싸지 않고 raw pointer로 둔다.
  dynamixel::PortHandler * dxl_port_{nullptr};
  dynamixel::PacketHandler * dxl_packet_{nullptr};
  int base_dxl_id_{0}, gripper_dxl_id_{4};
  double base_zero_offset_{0.0}, base_direction_{1.0};
  bool base_wraparound_{true};
  double gripper_zero_offset_{0.0}, gripper_direction_{1.0};
  bool gripper_wraparound_{true};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rmd_angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rmd_raw_angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rmd_encoder_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr dxl_angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr dxl_encoder_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RmdJointStateBridge>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("rmd_joint_state_bridge"), "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
