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


    RCLCPP_INFO(rclcpp::get_logger("TrilobyteControlSystem"), "Starting Trilobyte Hardware Control . . . %d", hardware_interface::stod(info_.hardware_parameters["baud_rate"]));



    return hardware_interface::CallbackReturn::SUCCESS;
    }
    

    // (core method) - returns a list of state interfaces
    std::vector<hardware_interface::StateInterface>
    TrilobyteControlSystem::export_state_interfaces() {
      std::vector<hardware_interface::StateInterface> state_interfaces;

    return state_interfaces;
    }

    // (core method) - returns a list of command interfaces
    std::vector<hardware_interface::CommandInterface>
    TrilobyteControlSystem::export_command_interfaces() {

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    return command_interfaces;
    }

    // read (core method) - updates the data values of the state_interfaces
    hardware_interface::return_type TrilobyteControlSystem::read(
    const rclcpp::Time& /*time*/, 
    const rclcpp::Duration& /*period*/) {

    
    // RCLCPP_INFO(rclcpp::get_logger("TrilobyteControlSystem"), "%f %f", time, period.seconds());

    return hardware_interface::return_type::OK;
    }

    // write (core method) - updates the data values of the command_interfaces

    hardware_interface::return_type TrilobyteControlSystem::write(
    const rclcpp::Time& /*time*/, 
    const rclcpp::Duration& /*period*/)  {
    
    // RCLCPP_INFO(rclcpp::get_logger("TrilobyteControlSystem"), "%f %f", time, period.seconds());
    // printf("test");
   
    return hardware_interface::return_type::OK;
    }



} // namespace my_robot_hardware_interface


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  trilobyte_hardware_interface::TrilobyteControlSystem, hardware_interface::SystemInterface)