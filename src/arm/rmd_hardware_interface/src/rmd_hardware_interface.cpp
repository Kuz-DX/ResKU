#include "rmd_hardware_interface/rmd_hardware_interface.hpp"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#if __has_include(<pthread.h>) && __has_include(<sched.h>)
  #include <pthread.h>
  #include <sched.h>
  #define RMD_HARDWARE_INTERFACE__THREAD_PRIORITY
#endif

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rmd_sdk/actuator_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>


#include "rmd_hardware_interface/conversions.hpp"
#include "rmd_hardware_interface/low_pass_filter.hpp"


namespace rmd_hardware_interface {

  using CallbackReturn = MyActuatorRmdHardwareInterface::CallbackReturn;
  // Small non-zero offsets are valid assembly-error calibrations.
  // namespace {
  //   constexpr double kOffsetCalibrationThresholdRad = 0.25 * M_PI / 180.0;
  // }

  MyActuatorRmdHardwareInterface::~MyActuatorRmdHardwareInterface() {
    // If the controller manager is shutdown with Ctrl + C the on_deactivate methods won't be called!
    // We therefore shut down the actuator here.
    on_cleanup(rclcpp_lifecycle::State());
    return;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_configure(rclcpp_lifecycle::State const& /*previous_state*/) {
    if (info_.hardware_parameters.find("ifname") != info_.hardware_parameters.end()) {
      ifname_ = info_.hardware_parameters["ifname"];
    } else {
      RCLCPP_FATAL(getLogger(), "Could not parse CAN interface name!");
      return CallbackReturn::ERROR;
    }
    if (info_.hardware_parameters.find("actuator_id") != info_.hardware_parameters.end()) {
      actuator_id_ = std::stoi(info_.hardware_parameters["actuator_id"]);
    } else {
      RCLCPP_FATAL(getLogger(), "Could not parse CAN actuator id!");
      return CallbackReturn::ERROR;
    }
    if (info_.hardware_parameters.find("torque_constant") != info_.hardware_parameters.end()) {
      torque_constant_ = std::stod(info_.hardware_parameters["torque_constant"]);
    } else {
      torque_constant_ = std::numeric_limits<double>::quiet_NaN();
      RCLCPP_ERROR(getLogger(), "Could not parse torque constant, won't be able to use torque interface!");
    }
    if (info_.hardware_parameters.find("max_velocity") != info_.hardware_parameters.end()) {
      max_velocity_ = std::stod(info_.hardware_parameters["max_velocity"]);
    } else {
      max_velocity_ = 720.0;
      RCLCPP_INFO(getLogger(), "Max velocity not set, defaulting to '%f'.", max_velocity_);
    }
    auto const parse_positive_double = [this](char const* name, double& value) {
      auto const parameter = info_.hardware_parameters.find(name);
      if (parameter == info_.hardware_parameters.end()) {
        RCLCPP_FATAL(getLogger(), "Required hardware parameter '%s' is missing.", name);
        return false;
      }
      value = std::stod(parameter->second);
      if (!std::isfinite(value) || value <= 0.0) {
        RCLCPP_FATAL(getLogger(), "Hardware parameter '%s' must be finite and positive.", name);
        return false;
      }
      return true;
    };
    if (!parse_positive_double("software_current_limit", software_current_limit_) ||
        !parse_positive_double("current_release_threshold", current_release_threshold_) ||
        !parse_positive_double("current_retreat_velocity", current_retreat_velocity_) ||
        !parse_positive_double("current_max_retreat_distance", current_max_retreat_distance_)) {
      return CallbackReturn::ERROR;
    }
    if (current_release_threshold_ >= software_current_limit_) {
      RCLCPP_FATAL(getLogger(),
        "Current release threshold %.3f A must be below software current limit %.3f A.",
        current_release_threshold_, software_current_limit_);
      return CallbackReturn::ERROR;
    }
    auto const duration_parameter = info_.hardware_parameters.find("current_limit_duration_ms");
    if (duration_parameter == info_.hardware_parameters.end()) {
      RCLCPP_FATAL(getLogger(), "Required hardware parameter 'current_limit_duration_ms' is missing.");
      return CallbackReturn::ERROR;
    }
    current_limit_duration_ = std::chrono::milliseconds(std::stol(duration_parameter->second));
    if (current_limit_duration_.count() <= 0) {
      RCLCPP_FATAL(getLogger(), "Hardware parameter 'current_limit_duration_ms' must be positive.");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(getLogger(),
      "Position current protection: limit %.3f A, release %.3f A, trigger %ld ms, retreat %.3f rad/s.",
      software_current_limit_, current_release_threshold_, current_limit_duration_.count(),
      current_retreat_velocity_);
    if (info_.hardware_parameters.find("velocity_alpha") != info_.hardware_parameters.end()) {
      auto const velocity_alpha {std::stod(info_.hardware_parameters["velocity_alpha"])};
      velocity_low_pass_filter_ = std::make_unique<LowPassFilter>(velocity_alpha);
      RCLCPP_INFO(getLogger(), "Using velocity low-pass filter with filter constant '%f'.", velocity_alpha);
    } else {
      velocity_low_pass_filter_ = nullptr;
      RCLCPP_INFO(getLogger(), "Not using velocity low-pass filter.");
    }
    if (info_.hardware_parameters.find("effort_alpha") != info_.hardware_parameters.end()) {
      auto const effort_alpha {std::stod(info_.hardware_parameters["effort_alpha"])};
      effort_low_pass_filter_ = std::make_unique<LowPassFilter>(effort_alpha);
      RCLCPP_INFO(getLogger(), "Using effort low-pass filter with filter constant '%f'.", effort_alpha);
    } else {
      effort_low_pass_filter_ = nullptr;
      RCLCPP_INFO(getLogger(), "Not using effort low-pass filter.");
    }
    if (info_.hardware_parameters.find("cycle_time") != info_.hardware_parameters.end()) {
      cycle_time_ = std::chrono::milliseconds(std::stol(info_.hardware_parameters["cycle_time"]));
    } else {
      cycle_time_ = std::chrono::milliseconds(2);
      RCLCPP_INFO(getLogger(), "Cycle time not set, defaulting to '%ld' ms.", cycle_time_.count());
    }
    if (info_.hardware_parameters.find("timeout") != info_.hardware_parameters.end()) {
      timeout_= std::chrono::milliseconds(std::stoi(info_.hardware_parameters["timeout"]));
    } else {
      timeout_ = std::chrono::milliseconds(0);
      RCLCPP_INFO(getLogger(), "Timeout not set, defaulting to '%ld' ms", timeout_.count());
    }
    if (timeout_ == std::chrono::milliseconds(0)) {
      RCLCPP_INFO(getLogger(), "Timeout set to 0ms, it will not be used!");
    }
    auto const feedback_timeout_parameter = info_.hardware_parameters.find("feedback_timeout_ms");
    feedback_timeout_ = std::chrono::milliseconds(
      feedback_timeout_parameter == info_.hardware_parameters.end() ? 500 :
      std::stol(feedback_timeout_parameter->second));
    auto const can_receive_timeout_parameter =
      info_.hardware_parameters.find("can_receive_timeout_ms");
    can_receive_timeout_ = std::chrono::milliseconds(
      can_receive_timeout_parameter == info_.hardware_parameters.end() ? 100 :
      std::stol(can_receive_timeout_parameter->second));
    auto const initial_feedback_timeout_parameter =
      info_.hardware_parameters.find("initial_feedback_timeout_ms");
    initial_feedback_timeout_ = std::chrono::milliseconds(
      initial_feedback_timeout_parameter == info_.hardware_parameters.end() ? 3000 :
      std::stol(initial_feedback_timeout_parameter->second));
    if (feedback_timeout_.count() <= 0 || can_receive_timeout_.count() <= 0 ||
        initial_feedback_timeout_.count() <= 0) {
      RCLCPP_FATAL(getLogger(),
        "CAN receive, initial feedback, and feedback watchdog timeouts must be positive.");
      return CallbackReturn::ERROR;
    }
    if (can_receive_timeout_ >= feedback_timeout_) {
      RCLCPP_FATAL(getLogger(),
        "CAN receive timeout (%ld ms) must be shorter than feedback watchdog (%ld ms).",
        can_receive_timeout_.count(), feedback_timeout_.count());
      return CallbackReturn::ERROR;
    }
    driver_ = std::make_unique<rmd_sdk::CanDriver>(ifname_);
    driver_->setReceiveTimeout(can_receive_timeout_);
    actuator_interface_ = std::make_unique<rmd_sdk::ActuatorInterface>(*driver_, actuator_id_);
    if (!actuator_interface_) {
      RCLCPP_INFO(getLogger(), "Failed to create actuator interface!");
      return CallbackReturn::ERROR;
    }
    auto const model_deadline {std::chrono::steady_clock::now() + initial_feedback_timeout_};
    bool model_warning_logged {false};
    while (true) {
      try {
        std::string const motor_model {actuator_interface_->getMotorModel()};
        RCLCPP_INFO(getLogger(),
          "Started actuator interface for actuator model '%s'!", motor_model.c_str());
        break;
      } catch (std::exception const& exception) {
        if (!model_warning_logged) {
          RCLCPP_WARN(getLogger(),
            "Motor did not answer during configuration; retrying: %s", exception.what());
          model_warning_logged = true;
        }
        if (std::chrono::steady_clock::now() >= model_deadline) {
          RCLCPP_ERROR(getLogger(),
            "Motor configuration failed after %ld ms: %s",
            initial_feedback_timeout_.count(), exception.what());
          return CallbackReturn::ERROR;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    return CallbackReturn::SUCCESS;
  }
      
  CallbackReturn MyActuatorRmdHardwareInterface::on_cleanup(rclcpp_lifecycle::State const& /*previous_state*/) {
    stopAsyncThread();
    if (actuator_interface_) {
      actuator_interface_->shutdownMotor();
    }
    return CallbackReturn::SUCCESS;
  }
  
  CallbackReturn MyActuatorRmdHardwareInterface::on_shutdown(rclcpp_lifecycle::State const& /*previous_state*/) {
    stopAsyncThread();
    if (actuator_interface_) {
      actuator_interface_->shutdownMotor();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_activate(rclcpp_lifecycle::State const& /*previous_state*/) {
    // Require a fresh sample for every lifecycle activation. This prevents a
    // stale pre-fault position from becoming the initial hold target.
    if (async_thread_.joinable()) {
      stopAsyncThread();
    }
    position_state_valid_.store(false);
    communication_fault_.store(false);
    fault_published_.store(false);
    last_valid_feedback_ns_.store(0);
    next_fault_log_ns_.store(0);
    position_command_valid_.store(false);
    position_interface_running_.store(false);
    velocity_interface_running_.store(false);
    effort_interface_running_.store(false);
    async_position_state_.store(std::numeric_limits<double>::quiet_NaN());
    stop_async_thread_.store(false);
    if (!startAsyncThread(cycle_time_)) {
      RCLCPP_FATAL(getLogger(), "Failed to start CAN communication thread.");
      return CallbackReturn::ERROR;
    }

    auto const deadline {std::chrono::steady_clock::now() + initial_feedback_timeout_};
    while (!position_state_valid_.load() && !communication_fault_.load() &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!position_state_valid_.load() || communication_fault_.load()) {
      RCLCPP_ERROR(getLogger(),
        "Activation failed: no valid initial position feedback within %ld ms.",
        initial_feedback_timeout_.count());
      stopAsyncThread();
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(getLogger(), "Actuator started with fresh initial position feedback.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_deactivate(rclcpp_lifecycle::State const& /*previous_state*/) {
    stopAsyncThread();
    if (actuator_interface_) {
      actuator_interface_->stopMotor();
    }
    RCLCPP_INFO(getLogger(), "Actuator successfully stopped!");
    return CallbackReturn::SUCCESS;
  }
  
  CallbackReturn MyActuatorRmdHardwareInterface::on_error(rclcpp_lifecycle::State const& /*previous_state*/) {
    stopAsyncThread();
    if (actuator_interface_) {
      actuator_interface_->stopMotor();
      actuator_interface_->reset();
      RCLCPP_INFO(getLogger(), "Actuator reset!");
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_init(hardware_interface::HardwareInfo const& info) {
    if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    position_state_ = std::numeric_limits<double>::quiet_NaN();
    velocity_state_ = 0.0;
    effort_state_ = 0.0;
    position_command_ = std::numeric_limits<double>::quiet_NaN();
    velocity_command_ = 0.0;
    effort_command_ = 0.0;

    async_position_state_.store(std::numeric_limits<double>::quiet_NaN());
    async_velocity_state_.store(0.0);
    async_effort_state_.store(0.0);
    async_position_command_.store(std::numeric_limits<double>::quiet_NaN());
    async_velocity_command_.store(0.0);
    async_effort_command_.store(0.0);
    position_state_valid_.store(false);
    communication_fault_.store(false);
    fault_published_.store(false);
    last_valid_feedback_ns_.store(0);
    next_fault_log_ns_.store(0);
    position_command_valid_.store(false);
    position_interface_running_.store(false);
    velocity_interface_running_.store(false);
    effort_interface_running_.store(false);
    current_limit_active_ = false;
    current_limit_latched_ = false;
    blocked_motion_direction_ = 0;
    overcurrent_duration_ = std::chrono::milliseconds(0);
    safe_position_command_ = std::numeric_limits<double>::quiet_NaN();
    current_limit_start_position_ = std::numeric_limits<double>::quiet_NaN();
    next_position_diagnostic_time_ = std::chrono::steady_clock::now();

    if (info_.joints.size() != 1) {
      RCLCPP_FATAL(getLogger(), "Expected a single joint but got %zu joints.", info_.joints.size());
      return CallbackReturn::ERROR;
    }
    hardware_interface::ComponentInfo const& joint = info_.joints.at(0);
    fault_node_ = std::make_shared<rclcpp::Node>("rmd_fault_" + joint.name);
    rclcpp::QoS fault_qos {rclcpp::KeepLast(1)};
    fault_qos.reliable().transient_local();
    fault_publisher_ = fault_node_->create_publisher<std_msgs::msg::String>(
      "/control/hardware_fault", fault_qos);

    auto const sign_parameter = info_.hardware_parameters.find("sign");
    if (sign_parameter == info_.hardware_parameters.end()) {
      RCLCPP_FATAL(getLogger(), "Required hardware parameter 'sign' is missing for joint '%s'.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    auto const parse_finite_parameter = [this, &joint](
        std::string const& name, std::string const& text, double& value) {
      try {
        std::size_t parsed_characters {0};
        value = std::stod(text, &parsed_characters);
        if (parsed_characters != text.size() || !std::isfinite(value)) {
          throw std::invalid_argument("not a finite numeric value");
        }
      } catch (std::exception const& exception) {
        RCLCPP_FATAL(getLogger(), "Joint '%s' hardware parameter '%s' is invalid: %s",
          joint.name.c_str(), name.c_str(), exception.what());
        return false;
      }
      return true;
    };
    if (!parse_finite_parameter("sign", sign_parameter->second, position_sign_)) {
      return CallbackReturn::ERROR;
    }
    if (!std::isfinite(position_sign_) || (position_sign_ != -1.0 && position_sign_ != 1.0)) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' hardware parameter 'sign' must be +1 or -1.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    auto const offset_parameter = info_.hardware_parameters.find("q_offset");
    if (offset_parameter == info_.hardware_parameters.end()) {
      RCLCPP_FATAL(getLogger(), "Required hardware parameter 'q_offset' is missing for joint '%s'.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (!parse_finite_parameter("q_offset", offset_parameter->second, q_offset_)) {
      return CallbackReturn::ERROR;
    }
    // A small non-zero q_offset is intentional: it compensates for residual
    // alignment error after assembling the hardware at the URDF zero pose.
    // if (q_offset_ != 0.0 && std::abs(q_offset_) <= kOffsetCalibrationThresholdRad) {
    //   RCLCPP_FATAL(getLogger(),
    //     "Joint '%s' q_offset %.9f rad is within the 0.25 deg assembly tolerance; use 0.0 rad.",
    //     joint.name.c_str(), q_offset_);
    //   return CallbackReturn::ERROR;
    // }
    RCLCPP_INFO(getLogger(),
      "Joint '%s' encoder correction: sign=%+.0f, q_offset=%.9f rad (%.6f deg).",
      joint.name.c_str(), position_sign_, q_offset_, q_offset_ * 180.0 / M_PI);

    // Position-only command contract. If velocity/effort control is enabled later,
    // extend this validation and export_command_interfaces() together, then add
    // finite-value rejection and safe mode-entry initialization in write() and
    // perform_command_mode_switch().
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size()
      );
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(getLogger(),
        "Joint '%s' has unexpected command interface '%s'. Expected '%s'",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION
      );
      return CallbackReturn::ERROR;
    }
    try {
      position_lower_limit_ = std::stod(joint.command_interfaces[0].min);
      position_upper_limit_ = std::stod(joint.command_interfaces[0].max);
      position_limit_rejection_margin_ = 5.0 * M_PI / 180.0;
      auto const margin = info_.hardware_parameters.find("position_limit_rejection_margin");
      if (margin != info_.hardware_parameters.end()) {
        position_limit_rejection_margin_ = std::stod(margin->second);
      }
    } catch (std::exception const& exception) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' requires finite position command min/max: %s",
        joint.name.c_str(), exception.what());
      return CallbackReturn::ERROR;
    }
    if (!std::isfinite(position_lower_limit_) || !std::isfinite(position_upper_limit_) ||
        position_lower_limit_ >= position_upper_limit_ ||
        !std::isfinite(position_limit_rejection_margin_) || position_limit_rejection_margin_ < 0.0) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has invalid hardware safety limits.", joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu state interfaces. 3 expected.",
        joint.name.c_str(), joint.state_interfaces.size()
      );
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
      );
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY
      );
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
        joint.name.c_str(), joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT
      );
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> MyActuatorRmdHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces {};
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_POSITION, &position_state_)
    );
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_VELOCITY, &velocity_state_)
    );
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_EFFORT, &effort_state_)
    );
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> MyActuatorRmdHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces {};
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_POSITION, &position_command_)
    );
    // Future velocity/effort command support must be exported here only after
    // on_init(), mode switching, and write() implement the safeguards noted there.
    return command_interfaces;
  }

  hardware_interface::return_type MyActuatorRmdHardwareInterface::prepare_command_mode_switch(
    std::vector<std::string> const& start_interfaces, std::vector<std::string> const& stop_interfaces) {
    bool position_interface_claimed {position_interface_running_.load()};
    bool velocity_interface_claimed {velocity_interface_running_.load()};
    bool effort_interface_claimed {effort_interface_running_.load()};

    for (auto const& stop_interface: stop_interfaces) {
      if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_POSITION) {
        position_interface_claimed = false;
      } else if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_VELOCITY) {
        velocity_interface_claimed = false;
      } else if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_EFFORT) {
        effort_interface_claimed = false;
      }
    }

    for (auto const& start_interface: start_interfaces) {
      if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_POSITION) {
        if (velocity_interface_claimed || effort_interface_claimed) {
          RCLCPP_ERROR(getLogger(), "Can't claim position interface: Conflicting interface!");
          return hardware_interface::return_type::ERROR;
        }
        position_interface_claimed = true;
      } else if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_VELOCITY) {
        if (position_interface_claimed || effort_interface_claimed) {
          RCLCPP_ERROR(getLogger(), "Can't claim velocity interface: Conflicting interface!");
          return hardware_interface::return_type::ERROR;
        }
        velocity_interface_claimed = true;
      } else if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_EFFORT) {
        if (std::isnan(torque_constant_)) {
          RCLCPP_ERROR(getLogger(), "Can't claim effort interface: Invalid torque constant!");
          return hardware_interface::return_type::ERROR;
        } else if (position_interface_claimed || velocity_interface_claimed) {
          RCLCPP_ERROR(getLogger(), "Can't claim effort interface: Conflicting interface!");
          return hardware_interface::return_type::ERROR;
        }
        effort_interface_claimed = true;
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MyActuatorRmdHardwareInterface::perform_command_mode_switch(
    std::vector<std::string> const& start_interfaces, std::vector<std::string> const& stop_interfaces) {
    for (auto const& stop_interface: stop_interfaces) {
      if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_POSITION) {
        position_interface_running_.store(false);
        RCLCPP_INFO(getLogger(), "Stopping position interface...");
      } else if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_VELOCITY) {
        velocity_interface_running_.store(false);
        RCLCPP_INFO(getLogger(), "Stopping velocity interface...");
      } else if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_EFFORT) {
        effort_interface_running_.store(false);
        RCLCPP_INFO(getLogger(), "Stopping effort interface...");
      }
    }

    for (auto const& start_interface: start_interfaces) {
      if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_POSITION) {
        if (!position_state_valid_.load()) {
          RCLCPP_ERROR(getLogger(),
            "Refusing to start position interface: no valid motor position has been received yet.");
          return hardware_interface::return_type::ERROR;
        }
        double const measured_position {async_position_state_.load()};
        if (!std::isfinite(measured_position)) {
          RCLCPP_ERROR(getLogger(),
            "Refusing to start position interface: measured motor position is not finite.");
          return hardware_interface::return_type::ERROR;
        }
        // Start by holding the measured position, never the default value zero.
        // Publish both buffers before exposing the running flag to the async thread.
        position_command_ = measured_position;
        async_position_command_.store(measured_position);
        safe_position_command_ = measured_position;
        current_limit_active_ = false;
        current_limit_latched_ = false;
        blocked_motion_direction_ = 0;
        overcurrent_duration_ = std::chrono::milliseconds(0);
        position_command_valid_.store(true);
        position_interface_running_.store(true);
        RCLCPP_INFO(getLogger(),
          "Starting position interface at measured position %.6f rad.", measured_position);
      } else if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_VELOCITY) {
        velocity_interface_running_.store(true);
        RCLCPP_INFO(getLogger(), "Starting velocity interface...");
      } else if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_EFFORT) {
        effort_interface_running_.store(true);
        RCLCPP_INFO(getLogger(), "Starting effort interface...");
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MyActuatorRmdHardwareInterface::read(rclcpp::Time const& /*time*/,
    rclcpp::Duration const& /*period*/) {
    position_state_ = async_position_state_.load();
    velocity_state_ = async_velocity_state_.load();
    effort_state_ = async_effort_state_.load();
    if (communication_fault_.load()) {
      auto const now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      auto next_log_ns = next_fault_log_ns_.load();
      if (now_ns >= next_log_ns && next_fault_log_ns_.compare_exchange_strong(
          next_log_ns, now_ns + std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::seconds(1)).count())) {
        RCLCPP_ERROR(getLogger(),
          "Feedback fault is latched; CAN retries are stopped. Restore communication and "
          "reactivate hardware component '%s'.",
          info_.name.c_str());
      }
      // Keep the controller alive so healthy joints can receive the latched
      // coordinated-hold target. Recovery remains explicit and requires a
      // hardware lifecycle reactivation.
      return hardware_interface::return_type::OK;
    }
    auto const last_ns = last_valid_feedback_ns_.load();
    if (position_state_valid_.load() && last_ns > 0) {
      auto const now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      if (now_ns - last_ns >
          std::chrono::duration_cast<std::chrono::nanoseconds>(feedback_timeout_).count()) {
        RCLCPP_ERROR(getLogger(),
          "RMD feedback watchdog expired for joint '%s': no valid feedback for %ld ms.",
          info_.joints.at(0).name.c_str(), feedback_timeout_.count());
        communication_fault_.store(true);
        stop_async_thread_.store(true);
        if (position_interface_running_.load()) {
          publishHardwareFault("feedback watchdog expired");
        }
        return hardware_interface::return_type::OK;
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MyActuatorRmdHardwareInterface::write(rclcpp::Time const& /*time*/,
    rclcpp::Duration const& /*period*/) {
    if (position_interface_running_) {
      if (std::isfinite(position_command_)) {
        if (position_command_ < position_lower_limit_ - position_limit_rejection_margin_ ||
            position_command_ > position_upper_limit_ + position_limit_rejection_margin_) {
          RCLCPP_ERROR(getLogger(), "Rejecting position command %.6f outside safety range [%.6f, %.6f].",
            position_command_, position_lower_limit_, position_upper_limit_);
          return hardware_interface::return_type::ERROR;
        }
        double const safe_command = std::clamp(
          position_command_, position_lower_limit_, position_upper_limit_);
        if (safe_command != position_command_) {
          RCLCPP_WARN(getLogger(), "Clamping position command %.6f to %.6f.",
            position_command_, safe_command);
        }
        async_position_command_.store(safe_command);
        position_command_valid_.store(true);
      } else {
        RCLCPP_ERROR(getLogger(), "Ignoring non-finite position command.");
      }
    } else if (velocity_interface_running_) {
      // Future velocity command support: reject NaN/Inf here before storing the
      // command, and initialize the command to zero when entering velocity mode.
      async_velocity_command_.store(velocity_command_);
    } else if (effort_interface_running_) {
      // Future effort command support: reject NaN/Inf here before storing the
      // command, and initialize the command to zero when entering effort mode.
      async_effort_command_.store(effort_command_);
    }
    return hardware_interface::return_type::OK;
  }

  rclcpp::Logger MyActuatorRmdHardwareInterface::getLogger() const {
    std::string const joint_name {
      info_.joints.empty() ? "uninitialized" : info_.joints.at(0).name};
    return rclcpp::get_logger("rmd_hardware_interface." + joint_name);
  }

  void MyActuatorRmdHardwareInterface::publishHardwareFault(std::string const& reason) {
    bool expected {false};
    if (!fault_publisher_ || !fault_published_.compare_exchange_strong(expected, true)) {
      return;
    }
    std_msgs::msg::String message {};
    message.data = info_.joints.at(0).name + ": " + reason;
    fault_publisher_->publish(message);
    RCLCPP_ERROR(getLogger(), "Published latched hardware fault: %s", message.data.c_str());
  }

  void MyActuatorRmdHardwareInterface::asyncThread(std::chrono::milliseconds const& cycle_time) {
    auto const setup_deadline {std::chrono::steady_clock::now() + initial_feedback_timeout_};
    bool setup_warning_logged {false};
    while (!stop_async_thread_) {
      try {
        actuator_interface_->setTimeout(timeout_);
        break;
      } catch (std::exception const& exception) {
        if (!setup_warning_logged) {
          RCLCPP_WARN(getLogger(),
            "CAN communication unavailable during activation; retrying: %s", exception.what());
          setup_warning_logged = true;
        }
        if (std::chrono::steady_clock::now() >= setup_deadline) {
          communication_fault_.store(true);
          RCLCPP_ERROR(getLogger(),
            "Failed to configure actuator after %ld ms; activation aborted: %s",
            initial_feedback_timeout_.count(), exception.what());
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    if (stop_async_thread_) {
      return;
    }
    std::int64_t communication_failure_start_ns {0};
    while (!stop_async_thread_) {
      auto const now {std::chrono::steady_clock::now()};
      auto const wakeup_time {now + cycle_time};
      try {
      if (position_interface_running_ && position_command_valid_.load()) {
        double const requested_command {async_position_command_.load()};
        if (std::isfinite(requested_command)) {
          double const measured_position {async_position_state_.load()};
          double const measured_current {std::abs(feedback_.current)};
          double const position_error {requested_command - measured_position};
          int const requested_direction {(position_error > 0.0) - (position_error < 0.0)};

          if (!std::isfinite(safe_position_command_)) {
            safe_position_command_ = measured_position;
          }

          // A command away from the collision is always allowed and clears the latch.
          if ((current_limit_active_ || current_limit_latched_) && blocked_motion_direction_ != 0 &&
              requested_direction != 0 && requested_direction != blocked_motion_direction_) {
            current_limit_active_ = false;
            current_limit_latched_ = false;
            blocked_motion_direction_ = 0;
            overcurrent_duration_ = std::chrono::milliseconds(0);
            safe_position_command_ = requested_command;
            RCLCPP_INFO(getLogger(), "Current protection released by reverse position command.");
          }

          if (!current_limit_active_ && !current_limit_latched_) {
            safe_position_command_ = requested_command;
            if (std::isfinite(measured_current) && measured_current >= software_current_limit_ &&
                requested_direction != 0) {
              overcurrent_duration_ += cycle_time;
              if (overcurrent_duration_ >= current_limit_duration_) {
                current_limit_active_ = true;
                blocked_motion_direction_ = requested_direction;
                safe_position_command_ = measured_position;
                current_limit_start_position_ = measured_position;
                RCLCPP_WARN(getLogger(),
                  "Software current limit active: measured %.3f A >= %.3f A; relaxing position target.",
                  measured_current, software_current_limit_);
              }
            } else {
              overcurrent_duration_ = std::chrono::milliseconds(0);
            }
          }

          if (current_limit_active_) {
            double const retreat_step {
              current_retreat_velocity_ * static_cast<double>(cycle_time.count()) / 1000.0};
            double const retreat_limit {
              current_limit_start_position_ -
              static_cast<double>(blocked_motion_direction_) * current_max_retreat_distance_};
            safe_position_command_ -= static_cast<double>(blocked_motion_direction_) * retreat_step;
            if (blocked_motion_direction_ > 0) {
              safe_position_command_ = std::max(safe_position_command_, retreat_limit);
            } else {
              safe_position_command_ = std::min(safe_position_command_, retreat_limit);
            }

            if (std::isfinite(measured_current) && measured_current <= current_release_threshold_) {
              current_limit_active_ = false;
              current_limit_latched_ = true;
              RCLCPP_WARN(getLogger(),
                "Software current limit latched at relaxed target %.6f rad (current %.3f A).",
                safe_position_command_, measured_current);
            }
          }

          double const encoder_command {q_offset_ + position_sign_ * safe_position_command_};
          feedback_ = actuator_interface_->sendPositionAbsoluteSetpoint(
            radToDeg(encoder_command), max_velocity_);
        } else {
          position_command_valid_.store(false);
          RCLCPP_ERROR(getLogger(), "Blocked non-finite asynchronous position command.");
          feedback_ = actuator_interface_->getMotorStatus2();
        }
      } else if (velocity_interface_running_) {
        feedback_ = actuator_interface_->sendVelocitySetpoint(radToDeg(async_velocity_command_.load()));
      } else if (effort_interface_running_) {
        feedback_ = actuator_interface_->sendTorqueSetpoint(async_effort_command_.load(), torque_constant_);
      } else {
        feedback_ = actuator_interface_->getMotorStatus2();
      }

      double const position_state {feedback_.shaft_angle};
      double velocity_state {feedback_.shaft_speed};
      if (velocity_low_pass_filter_) {
        velocity_state = velocity_low_pass_filter_->apply(velocity_state);
      }
      double current_state {feedback_.current};
      if (effort_low_pass_filter_) {
        current_state = effort_low_pass_filter_->apply(current_state);
      }
      double const encoder_position_rad {degToRad(position_state)};
      double const corrected_position_rad {
        position_sign_ * (encoder_position_rad - q_offset_)};
      if (std::isfinite(corrected_position_rad)) {
        async_position_state_.store(corrected_position_rad);
        position_state_valid_.store(true);
        last_valid_feedback_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
        auto const diagnostic_time {std::chrono::steady_clock::now()};
        if (diagnostic_time >= next_position_diagnostic_time_) {
          RCLCPP_DEBUG(getLogger(),
            "Joint '%s' position diagnostic: raw_encoder=%.9f rad, q_offset=%.9f rad, "
            "sign=%+.0f, corrected_joint=%.9f rad.",
            info_.joints.at(0).name.c_str(), encoder_position_rad, q_offset_, position_sign_,
            corrected_position_rad);
          next_position_diagnostic_time_ = diagnostic_time + std::chrono::seconds(1);
        }
      } else {
        RCLCPP_ERROR(getLogger(),
          "Received non-finite motor position; keeping the last valid state.");
      }
      async_velocity_state_.store(position_sign_ * degToRad(velocity_state));
      async_effort_state_.store(currentToTorque(current_state, torque_constant_));

      if (communication_failure_start_ns != 0) {
        auto const recovered_after_ms = (
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count() -
          communication_failure_start_ns) / 1000000;
        RCLCPP_INFO(getLogger(), "CAN feedback recovered after %ld ms.", recovered_after_ms);
        communication_failure_start_ns = 0;
      }
      } catch (std::exception const& exception) {
        auto const failure_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        if (communication_failure_start_ns == 0) {
          communication_failure_start_ns = failure_now_ns;
          RCLCPP_WARN(getLogger(), "CAN communication failed; retrying: %s", exception.what());
        }
        auto const failed_for_ns = failure_now_ns - communication_failure_start_ns;
        if (failed_for_ns >= std::chrono::duration_cast<std::chrono::nanoseconds>(
            feedback_timeout_).count()) {
          communication_fault_.store(true);
          RCLCPP_ERROR(getLogger(),
            "CAN communication failed continuously for %ld ms; retries stopped: %s",
            feedback_timeout_.count(), exception.what());
          if (position_interface_running_.load()) {
            publishHardwareFault(exception.what());
          }
          break;
        }
      }

      std::this_thread::sleep_until(wakeup_time);
    }
    try {
      actuator_interface_->setTimeout(std::chrono::milliseconds(0));
    } catch (std::exception const& exception) {
      RCLCPP_WARN(getLogger(), "Failed to clear actuator timeout while stopping: %s", exception.what());
    }
    return;
  }

  bool MyActuatorRmdHardwareInterface::startAsyncThread(std::chrono::milliseconds const& cycle_time) {
    if (!async_thread_.joinable()) {
      async_thread_ = std::thread(&MyActuatorRmdHardwareInterface::asyncThread, this, cycle_time);
    } else {
      RCLCPP_WARN(getLogger(), "Could not start command thread, command thread already running!");
      return false;
    }

#ifdef RMD_HARDWARE_INTERFACE__THREAD_PRIORITY
    std::ifstream realtime_file {"/sys/kernel/realtime", std::ios::in};
    bool has_realtime {false};
    if (realtime_file.is_open()) {
      realtime_file >> has_realtime;
    }

    int policy {};
    struct ::sched_param param {};
    ::pthread_getschedparam(async_thread_.native_handle(), &policy, &param);
    if (has_realtime) {
      policy = SCHED_FIFO;
      RCLCPP_INFO(getLogger(), "Real-time system detected: Setting policy to 'SCHED_FIFO'...");
    }
    int const max_thread_priority {::sched_get_priority_max(policy)};
    if (max_thread_priority != -1) {
      param.sched_priority = max_thread_priority;
      if (::pthread_setschedparam(async_thread_.native_handle(), policy, &param) == 0) {
        RCLCPP_INFO(getLogger(), "Set thread priority '%d' and policy '%d' to async thread!",
          param.sched_priority, policy);
      } else {
        RCLCPP_WARN(getLogger(), "Failed to set thread priority '%d' and policy '%d' to async thread!",
          param.sched_priority, policy);
      }
    } else {
      RCLCPP_WARN(getLogger(), "Could not set thread priority to async thread: Failed to get max priority!");
    }
#endif  // RMD_HARDWARE_INTERFACE__THREAD_PRIORITY

    return true;
  }

  void MyActuatorRmdHardwareInterface::stopAsyncThread() {
    if (async_thread_.joinable()) {
      stop_async_thread_.store(true);
      async_thread_.join();
    } else {
      RCLCPP_WARN(getLogger(), "Could not stop command thread: Not running!");
    }
    return;
  }

}  // namespace rmd_hardware_interface

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmd_hardware_interface::MyActuatorRmdHardwareInterface, hardware_interface::ActuatorInterface)
