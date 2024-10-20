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
  info_.joints[0].name, hardware_interface::HW_IF_POSITION, &_mcu.pos_wheel_left));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
  info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &_mcu.vel_wheel_left));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
  info_.joints[1].name, hardware_interface::HW_IF_POSITION, &_mcu.pos_wheel_right));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
  info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &_mcu.vel_wheel_right));

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
  const rclcpp::Duration& /*period*/) {

  auto time_current = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = time_current - _mcu.time_previous;
  double diff_seconds = diff.count();
  _mcu.time_previous = time_current;

  _mcu.read_encoder_values();

  double pos_previous_left = _mcu.pos_wheel_left;
  double pos_previous_right = _mcu.pos_wheel_right;
  _mcu.update_wheel_positions();
  _mcu.vel_wheel_left = (_mcu.pos_wheel_left - pos_previous_left) / diff_seconds;
  _mcu.vel_wheel_right = (_mcu.pos_wheel_right - pos_previous_right) / diff_seconds;


  RCLCPP_INFO(
  rclcpp::get_logger("TrilobyteControlSystem"), 
  "left: %f right: %f", 
  _mcu.vel_wheel_left, _mcu.vel_wheel_right);

  // RCLCPP_INFO(
  // rclcpp::get_logger("TrilobyteControlSystem"), 
  // "left: %d right: %d", 
  // _mcu.encoder_left, _mcu.encoder_right);

  // RCLCPP_INFO(
  // rclcpp::get_logger("TrilobyteControlSystem"), 
  // "left: %f right: %f", 
  // _mcu.pos_wheel_left, _mcu.pos_wheel_right);

  //   RCLCPP_INFO(
  // rclcpp::get_logger("TrilobyteControlSystem"), 
  // "time_current == %f", 
  // time_current);

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
  // "LeftValue: %d RightValue: %d", 
  // left_pwm, right_pwm);


  _mcu.send_motor_command(left_pwm, right_pwm);

  return hardware_interface::return_type::OK;
  }



} // namespace my_robot_hardware_interface


  #include "pluginlib/class_list_macros.hpp"

  PLUGINLIB_EXPORT_CLASS(
  trilobyte_hardware_interface::TrilobyteControlSystem, hardware_interface::SystemInterface)