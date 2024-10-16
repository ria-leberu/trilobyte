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

    this->left_wheel_name_ = info_.hardware_parameters["left_wheel_name"];
    std::cout << "left wheel == " << this->left_wheel_name_ << "\n";
  // std::string right_wheel_name_;
  // uint16_t loop_rate_;
  // std::string device_name_;
  // uint16_t baud_rate_;
  // uint16_t timeout_ms_;
  // uint16_t pulses_per_meter_;
  // uint16_t pulses_per_revolution_;
  // double wheel_radius_;
  // double wheel_separation_;

    return hardware_interface::CallbackReturn::SUCCESS;
  }
    

  // (core method) - returns a list of state interfaces
  std::vector<hardware_interface::StateInterface>
  TrilobyteControlSystem::export_state_interfaces() {

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
  info_.joints[0].name, hardware_interface::HW_IF_POSITION, &left_position_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
  info_.joints[1].name, hardware_interface::HW_IF_POSITION, &right_position_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
  info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_velocity_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
  info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_velocity_));

  // for (auto i = 0u; i < info_.joints.size(); i++)
  // {
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  // }

  return state_interfaces;
  }

  // (core method) - returns a list of command interfaces
  std::vector<hardware_interface::CommandInterface>
  TrilobyteControlSystem::export_command_interfaces() {

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // for (auto i = 0u; i < info_.joints.size(); i++)
  // {
  //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //   info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  // }

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

  return hardware_interface::return_type::OK;
  }

  // write (core method) - updates the data values of the command_interfaces

  hardware_interface::return_type TrilobyteControlSystem::write(
  const rclcpp::Time& /*time*/, 
  const rclcpp::Duration& /*period*/)  {
  RCLCPP_INFO(
  rclcpp::get_logger("TrilobyteControlSystem"), "Command Value: %.5f", left_command_);


  return hardware_interface::return_type::OK;
  }



} // namespace my_robot_hardware_interface


  #include "pluginlib/class_list_macros.hpp"

  PLUGINLIB_EXPORT_CLASS(
  trilobyte_hardware_interface::TrilobyteControlSystem, hardware_interface::SystemInterface)