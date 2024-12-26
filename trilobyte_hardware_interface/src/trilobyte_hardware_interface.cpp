#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/lexical_casts.hpp"

#include "rclcpp/rclcpp.hpp"

#include "trilobyte_hardware_interface/trilobyte_hardware_interface.hpp"

namespace trilobyte_hardware_interface
{
  // on_init (core method) - communication between robot hardware is setup 
  hardware_interface::CallbackReturn TrilobyteControlSystem::on_init(
  const hardware_interface::HardwareInfo& info) {

    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(
    rclcpp::get_logger("TrilobyteControlSystem"), "Starting Trilobyte Hardware Control . . . ");

    std::string& device_name = info_.hardware_parameters["device"];
    uint32_t baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    uint16_t timeout_ms = std::stoi(info_.hardware_parameters["timeout"]);

    _mcu.configure(device_name, baud_rate, timeout_ms);

    return hardware_interface::CallbackReturn::SUCCESS;
  }
    

  // (core method) - returns a list of state interfaces
  std::vector<hardware_interface::StateInterface>
  TrilobyteControlSystem::export_state_interfaces() {

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
  info_.joints[0].name, hardware_interface::HW_IF_POSITION, &left_position_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
  info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_velocity_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
  info_.joints[1].name, hardware_interface::HW_IF_POSITION, &right_position_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
  info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_velocity_));

  return state_interfaces;
  }

  // (core method) - returns a list of command interfaces
  std::vector<hardware_interface::CommandInterface>
  TrilobyteControlSystem::export_command_interfaces() {

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_command_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_command_));

  return command_interfaces;
  }

  // read (core method) - updates the data values of the state_interfaces
  hardware_interface::return_type TrilobyteControlSystem::read(
  const rclcpp::Time& /*time*/, 
  const rclcpp::Duration& period) {

  std::array<int,2> output_encoder = _mcu.read_encoder_values();
  int left_encoder_ticks = output_encoder[0];
  int right_encoder_ticks = output_encoder[1];

  double dt = period.seconds();

  left_position_ += (left_encoder_ticks - prev_left_encoder_ticks_) * (2 * PI_VALUE / 500);
  right_position_ += (right_encoder_ticks - prev_right_encoder_ticks_) * (2 * PI_VALUE / 500);

  left_velocity_ = ((left_encoder_ticks - prev_left_encoder_ticks_) * (2 * PI_VALUE / 500)) / dt;
  right_velocity_ = ((right_encoder_ticks - prev_right_encoder_ticks_) * (2 * PI_VALUE / 500)) / dt;

  // Store current encoder values for next cycle
  prev_left_encoder_ticks_ = left_encoder_ticks;
  prev_right_encoder_ticks_ = right_encoder_ticks;

  RCLCPP_INFO(
  rclcpp::get_logger("TrilobyteControlSystem"), 
  "left: %f right: %f", 
  left_velocity_, right_velocity_);

  // RCLCPP_INFO(
  // rclcpp::get_logger("TrilobyteControlSystem"), 
  // "left: %d right: %d", 
  // left_encoder_ticks, right_encoder_ticks);

  // RCLCPP_INFO(
  // rclcpp::get_logger("TrilobyteControlSystem"), 
  // "left: %f right: %f", 
  // _mcu.pos_wheel_left, _mcu.pos_wheel_right);

  //   RCLCPP_INFO(
  // rclcpp::get_logger("TrilobyteControlSystem"), 
  // "time_diff == %f", 
  // diff_seconds);

  return hardware_interface::return_type::OK;
  }

  // write (core method) - updates the data values of the command_interfaces
  hardware_interface::return_type TrilobyteControlSystem::write(
  const rclcpp::Time& /*time*/, 
  const rclcpp::Duration& /*period*/)  {

  int16_t left_pwm = ((left_command_* 0.1885)/(2 * PI_VALUE) * 278);
  int16_t right_pwm = ((right_command_ * 0.1885)/(2 * PI_VALUE) * 278);

  // RCLCPP_INFO(
  // rclcpp::get_logger("TrilobyteControlSystem"), 
  // "LeftValue: %f RightValue: %f", 
  // left_command_, right_command_);

  _mcu.send_motor_command(left_pwm, right_pwm);

  return hardware_interface::return_type::OK;
  }

} // namespace my_robot_hardware_interface


  #include "pluginlib/class_list_macros.hpp"

  PLUGINLIB_EXPORT_CLASS(
  trilobyte_hardware_interface::TrilobyteControlSystem, hardware_interface::SystemInterface)